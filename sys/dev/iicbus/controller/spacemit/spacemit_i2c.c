/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Daniel Shue <dgshue@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * iicbus(4) controller driver for the SpacemiT K1 (Ky X1) I2C/TWSI blocks
 * (compatible: "spacemit,k1-i2c").  This is a PXA/Marvell-style two-wire
 * interface (ICR/ISR/IDBR with a per-byte "transfer byte" trigger), not the
 * classic Marvell TWSI register layout that FreeBSD's twsi(4) drives, so it
 * needs its own driver.  We use a polled (PIO) byte-at-a-time transfer, which
 * is simple and robust and is what a foundational bus (PMIC, board sensors)
 * needs.  Register semantics and the transfer state machine were derived from
 * the mainline Linux driver drivers/i2c/busses/i2c-k1.c (GPL-2.0;
 * reimplemented, not copied).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include "iicbus_if.h"

/*
 * PMIC i2c (i2c8/PWR_SCL/PWR_SDA) pad-mux self-check + force.
 *
 * The bus was dead with IBMR SDA=0 SCL=0 (both lines low) and ISR permanently
 * 0 -- the controller's SCL/SDA never reach the pads, i.e. the pinmux was not
 * effective.  The DTS i2c8_cfg group (mux 0 for pins 93/94, byte-identical to
 * mainline k1-pinctrl.dtsi) SHOULD be applied by spacemit_pinctrl, but the
 * dead bus says it is not landing.  Rather than depend on the pinctrl pass, the
 * i2c driver verifies -- and if needed forces -- the two PWR pads directly at
 * attach: map the pinctrl MFPR block and read/set pins 93 (PWR_SCL) and 94
 * (PWR_SDA) to mux MODE0 (i2c function, per mainline pinctrl-k1.c
 * K1_FUNC_PIN(93,1)=GPIO so i2c is the non-GPIO mode 0) + PULL_UP | 1.8V-DS2
 * (open-drain bus needs the pad pull-ups) = MFPR 0xd040.  Logs the before/after
 * MFPR so hardware PROVES the mux field.
 */
#define	K1_PINCTRL_PHYS		0xd401e000u	/* pinctrl@d401e000 */
#define	K1_PINCTRL_SIZE		0x400u
/* MFPR offsets for pins 93/94 = (pin+24)<<2 (k1_pin_to_offset). */
#define	K1_MFPR_PWR_SCL		((93u + 24u) << 2)	/* 0x1d4 */
#define	K1_MFPR_PWR_SDA		((94u + 24u) << 2)	/* 0x1d8 */
#define	K1_MFPR_I2C8		0x0000d040u		/* MUX0|EDGE_NONE|PULL_UP|1V8_DS2 */
#define	K1_MFPR_MUX_MASK	0x7u

/*
 * APBC system-controller bank (clock/reset gates).  TWSI8's gate is at +0x20;
 * its func-clock enable is BIT(1)|BIT(0) (mainline ccu-k1.c twsi8_clk).  The
 * register reads back 0 (HW quirk), so we write the full enable value.
 */
#define	K1_APBC_PHYS		0xd4015000u
#define	K1_APBC_SIZE		0x1000u
#define	K1_APBC_TWSI8		0x20u
#define	K1_TWSI8_GATE_EN	0x3u	/* BIT1 (func) | BIT0 (bus) */

/*
 * Register offsets.
 *
 * NOTE (round 30): the on-board PMIC bus is the vendor "ky,x1-i2c" IP, whose
 * driver is drivers/i2c/busses/i2c-x1.c (+ i2c-x1.h) in the WORKING vendor BSP
 * (orangepi-xunlong/linux-orangepi @ orange-pi-6.6-ky).  That IP has SCL
 * timing registers -- REG_LCR (Load Count, 0x10) and REG_WCR (Wait Count,
 * 0x14) -- that MUST be programmed at controller reset (i2c-x1.h enum reg_addr;
 * i2c-x1.c ky_i2c_controller_reset() writes REG_LCR then REG_WCR).  Our
 * original driver modeled the mainline "spacemit,k1-i2c" (i2c-k1.c) variant,
 * which omits them -- so without LCR/WCR the SCL clock timing was undefined,
 * no clock was generated, and every transfer timed out (dead PMIC bus).
 */
#define	ICR		0x00		/* control register */
#define	ISR		0x04		/* status register */
#define	IDBR		0x0c		/* data buffer register */
#define	ILCR		0x10		/* SCL load-count (timing) register */
#define	IWCR		0x14		/* SCL wait-count (timing) register */
#define	IRCR		0x18		/* reset cycle counter */
#define	IBMR		0x1c		/* bus monitor register */

/*
 * SCL timing values for the K1/X1 i2c buses, taken VERBATIM from the working
 * vendor DTS (orange-pi-6.6-ky arch/riscv/boot/dts/ky/x1.dtsi, EVERY i2c node
 * incl. i2c8@d401d800): ky,i2c-lcr = <0x82c469f>, ky,i2c-wcr = <0x142a>, with
 * ky,i2c-clk-rate = 32 MHz and standard-mode (fast-mode is commented out).
 * The vendor driver reads these from the DT and writes them to REG_LCR/REG_WCR;
 * the SoC uses the same values for all its i2c instances, so hardcode them.
 */
#define	I2C_LCR_VALUE	0x082c469fu
#define	I2C_WCR_VALUE	0x0000142au

/* ICR (control) bits. */
#define	CR_START	(1u << 0)
#define	CR_STOP		(1u << 1)
#define	CR_ACKNAK	(1u << 2)	/* send ACK(0)/NAK(1) */
#define	CR_TB		(1u << 3)	/* transfer byte */
#define	CR_MODE_FAST	(1u << 8)	/* 400 kHz vs 100 kHz */
#define	CR_UR		(1u << 10)	/* unit reset */
#define	CR_RSTREQ	(1u << 11)	/* bus reset request */
#define	CR_SCLE		(1u << 13)	/* master clock enable */
#define	CR_IUE		(1u << 14)	/* unit enable */
#define	CR_GCD		(1u << 21)	/* general call disable */
#define	CR_MSDE		(1u << 26)	/* master stop detected enable */

/* ISR (status) bits. */
#define	SR_ACKNAK	(1u << 14)	/* ACK/NAK received (1 = NAK) */
#define	SR_UB		(1u << 15)	/* unit busy */
#define	SR_IBB		(1u << 16)	/* bus busy */
#define	SR_EBB		(1u << 17)	/* early bus busy */
#define	SR_ALD		(1u << 18)	/* arbitration loss detected */
#define	SR_ITE		(1u << 19)	/* TX buffer empty */
#define	SR_IRF		(1u << 20)	/* RX buffer full */
#define	SR_GCAD		(1u << 21)	/* general call address detected */
#define	SR_BED		(1u << 22)	/* bus error, no ACK/NAK */
#define	SR_SAD		(1u << 23)	/* slave address detected */
#define	SR_SSD		(1u << 24)	/* slave stop detected */
#define	SR_MSD		(1u << 26)	/* master stop detected */
#define	SR_TXDONE	(1u << 27)	/* transaction done */
#define	SR_TXE		(1u << 28)	/* TX FIFO empty */
#define	SR_RXHF		(1u << 29)	/* RX FIFO half-full */
#define	SR_RXF		(1u << 30)	/* RX FIFO full */
#define	SR_RXOV		(1u << 31)	/* RX FIFO overrun */

#define	SR_ERR		(SR_BED | SR_ALD)
/* Write-1-to-clear status bits (matches mainline INT_STATUS_MASK). */
#define	SR_STATUS_MASK	(SR_RXOV | SR_RXF | SR_RXHF | SR_TXE | SR_TXDONE | \
			 SR_MSD | SR_SSD | SR_SAD | SR_BED | SR_GCAD | \
			 SR_IRF | SR_ITE | SR_ALD)

/* IRCR (reset cycle) bits. */
#define	RCR_SDA_GLITCH_NOFIX	(1u << 7)

/* IBMR (bus monitor) bits. */
#define	BMR_SDA		(1u << 0)
#define	BMR_SCL		(1u << 1)

#define	I2C_STD_FREQ	100000
#define	I2C_FAST_FREQ	400000

#define	BUS_IDLE_TIMEOUT_US	100000
#define	XFER_TIMEOUT_US		50000
#define	POLL_INTERVAL_US	5

struct spacemit_i2c_softc {
	device_t	dev;
	device_t	iicbus;
	struct resource	*mem;
	int		mem_rid;
	struct mtx	mtx;
	clk_t		func_clk;
	clk_t		bus_clk;
	hwreset_t	reset;
	uint32_t	bus_freq;
};

#define	I2C_RD4(sc, o)		bus_read_4((sc)->mem, (o))
#define	I2C_WR4(sc, o, v)	bus_write_4((sc)->mem, (o), (v))

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-i2c",	1 },
	{ NULL,			0 }
};

static void
i2c_unit_reset(struct spacemit_i2c_softc *sc)
{

	/*
	 * Mirror the vendor ky_i2c_controller_reset() (i2c-x1.c): unit reset,
	 * then program the SCL Load-Count and Wait-Count timing registers.
	 * Without LCR/WCR the controller generates no valid SCL and every
	 * transfer times out -- this was the dead-bus root cause.
	 */
	I2C_WR4(sc, ICR, CR_UR);
	DELAY(5);
	I2C_WR4(sc, ICR, 0);

	I2C_WR4(sc, ILCR, I2C_LCR_VALUE);
	I2C_WR4(sc, IWCR, I2C_WCR_VALUE);
}

static void
i2c_hw_init(struct spacemit_i2c_softc *sc)
{
	uint32_t val;

	val = CR_GCD | CR_SCLE | CR_MSDE;
	if (sc->bus_freq >= I2C_FAST_FREQ)
		val |= CR_MODE_FAST;
	I2C_WR4(sc, ICR, val);

	/* The K1 restart glitch fix adds delay; disable it (per mainline). */
	val = I2C_RD4(sc, IRCR);
	val |= RCR_SDA_GLITCH_NOFIX;
	I2C_WR4(sc, IRCR, val);

	/* Clear any latched status. */
	I2C_WR4(sc, ISR, SR_STATUS_MASK);
}

static void
i2c_enable(struct spacemit_i2c_softc *sc)
{
	I2C_WR4(sc, ICR, I2C_RD4(sc, ICR) | CR_IUE);
}

static void
i2c_disable(struct spacemit_i2c_softc *sc)
{
	I2C_WR4(sc, ICR, I2C_RD4(sc, ICR) & ~CR_IUE);
}

/* Wait for the unit/bus to go idle before starting a transaction. */
static int
i2c_wait_bus_idle(struct spacemit_i2c_softc *sc)
{
	int i;

	for (i = 0; i < BUS_IDLE_TIMEOUT_US; i += POLL_INTERVAL_US) {
		if ((I2C_RD4(sc, ISR) & (SR_UB | SR_IBB)) == 0)
			return (0);
		DELAY(POLL_INTERVAL_US);
	}
	/*
	 * ROUND 30 diagnostic: on a bus-idle timeout, dump ISR + IBMR.  IBMR
	 * bit0=SDA, bit1=SCL line state: both 1 (idle-high) => pads muxed +
	 * pulled up, bus electrically fine (look at SCL timing); SCL stuck 0 or
	 * both 0 => pinmux/pull/clock-gate problem (lines not driven high).
	 */
	device_printf(sc->dev,
	    "bus-idle timeout: ISR=0x%08x IBMR=0x%02x (SDA=%d SCL=%d) ICR=0x%08x\n",
	    I2C_RD4(sc, ISR), I2C_RD4(sc, IBMR),
	    !!(I2C_RD4(sc, IBMR) & BMR_SDA), !!(I2C_RD4(sc, IBMR) & BMR_SCL),
	    I2C_RD4(sc, ICR));
	i2c_unit_reset(sc);
	return (IIC_ETIMEOUT);
}

/* Poll ISR until any bit of @mask is set (or error/timeout). */
static int
i2c_poll_status(struct spacemit_i2c_softc *sc, uint32_t mask, uint32_t *status)
{
	uint32_t sr;
	int i;

	for (i = 0; i < XFER_TIMEOUT_US; i += POLL_INTERVAL_US) {
		sr = I2C_RD4(sc, ISR);
		if (sr & (mask | SR_ERR)) {
			*status = sr;
			/* Write-1-to-clear the bits we observed. */
			I2C_WR4(sc, ISR, sr & SR_STATUS_MASK);
			return (0);
		}
		DELAY(POLL_INTERVAL_US);
	}
	/* ROUND 30 diagnostic: same IBMR/ISR dump on a transfer-phase timeout. */
	device_printf(sc->dev,
	    "xfer timeout (mask=0x%08x): ISR=0x%08x IBMR=0x%02x (SDA=%d SCL=%d)\n",
	    mask, I2C_RD4(sc, ISR), I2C_RD4(sc, IBMR),
	    !!(I2C_RD4(sc, IBMR) & BMR_SDA), !!(I2C_RD4(sc, IBMR) & BMR_SCL));
	return (IIC_ETIMEOUT);
}

/*
 * Issue the address byte (START) and transfer one message's payload.  Each
 * byte is clocked by re-arming CR_TB; the last byte of the final message gets
 * CR_STOP (and CR_ACKNAK on reads).
 */
static int
i2c_xfer_msg(struct spacemit_i2c_softc *sc, struct iic_msg *msg, bool last_msg)
{
	uint32_t addr, val, status;
	bool read;
	int error;
	uint16_t idx;

	read = (msg->flags & IIC_M_RD) != 0;

	/*
	 * Program the address+RW byte and send START.  In iicbus, msg->slave
	 * is the 7-bit address already shifted left by one (LSB is the R/W
	 * slot); mask it off and set the read bit as needed.
	 */
	addr = msg->slave & ~1u;
	if (read)
		addr |= 1;
	I2C_WR4(sc, IDBR, addr);

	val = I2C_RD4(sc, ICR);
	val &= ~CR_STOP;
	val |= CR_START | CR_TB;
	I2C_WR4(sc, ICR, val);

	/* Wait for the address byte to shift out (TX empty). */
	error = i2c_poll_status(sc, SR_ITE, &status);
	if (error != 0)
		return (error);
	if (status & SR_ERR)
		return (IIC_EBUSERR);
	if (status & SR_ACKNAK)
		return (IIC_ENOACK);

	for (idx = 0; idx < msg->len; idx++) {
		bool last_byte = (idx == msg->len - 1);

		val = I2C_RD4(sc, ICR);
		val &= ~(CR_TB | CR_ACKNAK | CR_STOP | CR_START);
		val |= CR_TB;
		if (last_msg && last_byte) {
			val |= CR_STOP;
			if (read)
				val |= CR_ACKNAK;
		}

		if (read) {
			I2C_WR4(sc, ICR, val);
			error = i2c_poll_status(sc, SR_IRF, &status);
			if (error != 0)
				return (error);
			if (status & SR_ERR)
				return (IIC_EBUSERR);
			msg->buf[idx] = (uint8_t)I2C_RD4(sc, IDBR);
		} else {
			I2C_WR4(sc, IDBR, msg->buf[idx]);
			I2C_WR4(sc, ICR, val);
			error = i2c_poll_status(sc, SR_ITE, &status);
			if (error != 0)
				return (error);
			if (status & SR_ERR)
				return (IIC_EBUSERR);
			if (status & SR_ACKNAK)
				return (IIC_ENOACK);
		}
	}

	return (IIC_NOERR);
}

static int
spacemit_i2c_transfer(device_t dev, struct iic_msg *msgs, uint32_t nmsgs)
{
	struct spacemit_i2c_softc *sc;
	int error;
	uint32_t i;

	sc = device_get_softc(dev);

	mtx_lock(&sc->mtx);

	i2c_hw_init(sc);
	i2c_enable(sc);

	error = i2c_wait_bus_idle(sc);
	if (error != 0)
		goto out;

	for (i = 0; i < nmsgs; i++) {
		error = i2c_xfer_msg(sc, &msgs[i], i == nmsgs - 1);
		if (error != IIC_NOERR)
			break;
	}

	/* Ensure the bus is released; recover if it is stuck. */
	if (I2C_RD4(sc, ISR) & SR_EBB) {
		i2c_unit_reset(sc);
		DELAY(90);
	}

out:
	i2c_disable(sc);
	mtx_unlock(&sc->mtx);
	return (error);
}

static int
spacemit_i2c_reset(device_t dev, u_char speed, u_char addr, u_char *oldaddr)
{
	struct spacemit_i2c_softc *sc;

	sc = device_get_softc(dev);

	mtx_lock(&sc->mtx);
	i2c_unit_reset(sc);
	i2c_hw_init(sc);
	mtx_unlock(&sc->mtx);
	return (IIC_NOERR);
}

static int
spacemit_i2c_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "SpacemiT K1 I2C");
	return (BUS_PROBE_DEFAULT);
}

/*
 * Verify and, if necessary, force the PWR_SCL/PWR_SDA pad mux + pull directly,
 * so the PMIC i2c bus works regardless of whether the pinctrl pass applied the
 * i2c8_cfg group.  Logs the MFPR before/after -- the on-hardware proof of the
 * mux field (the analog of the r29 control-read that localised the fault).
 */
static void
spacemit_i2c_force_pinmux(device_t dev)
{
	void *map;
	volatile uint32_t *base;
	uint32_t scl_before, sda_before, scl_after, sda_after;

	map = pmap_mapdev(K1_PINCTRL_PHYS, K1_PINCTRL_SIZE);
	if (map == NULL) {
		device_printf(dev, "pinmux: cannot map pinctrl MMIO\n");
		return;
	}
	base = (volatile uint32_t *)map;

	scl_before = base[K1_MFPR_PWR_SCL / 4];
	sda_before = base[K1_MFPR_PWR_SDA / 4];

	/*
	 * Force both pads to the i2c config 0xd040.  We overwrite the full word
	 * (mux + pull + drive + edge) with the vendor-exact value -- these two
	 * pads have no other consumer.
	 */
	base[K1_MFPR_PWR_SCL / 4] = K1_MFPR_I2C8;
	base[K1_MFPR_PWR_SDA / 4] = K1_MFPR_I2C8;
	__asm __volatile("fence" ::: "memory");

	scl_after = base[K1_MFPR_PWR_SCL / 4];
	sda_after = base[K1_MFPR_PWR_SDA / 4];

	device_printf(dev,
	    "pinmux PWR_SCL(0x%03x) 0x%04x->0x%04x, PWR_SDA(0x%03x) 0x%04x->0x%04x "
	    "(mux was %u/%u, forced i2c mode0+pullup)\n",
	    K1_MFPR_PWR_SCL, scl_before, scl_after,
	    K1_MFPR_PWR_SDA, sda_before, sda_after,
	    scl_before & K1_MFPR_MUX_MASK, sda_before & K1_MFPR_MUX_MASK);

	pmap_unmapdev(map, K1_PINCTRL_SIZE);
}

static int
spacemit_i2c_attach(device_t dev)
{
	struct spacemit_i2c_softc *sc;
	phandle_t node;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	if (OF_getencprop(node, "clock-frequency", &sc->bus_freq,
	    sizeof(sc->bus_freq)) <= 0 || sc->bus_freq == 0)
		sc->bus_freq = I2C_STD_FREQ;
	if (sc->bus_freq > I2C_FAST_FREQ)
		sc->bus_freq = I2C_FAST_FREQ;

	sc->mem_rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "cannot allocate memory resource\n");
		return (ENXIO);
	}

	/* Functional + bus (APB) clocks are required. */
	if (clk_get_by_ofw_name(dev, 0, "func", &sc->func_clk) != 0) {
		device_printf(dev, "cannot get func clock\n");
		goto fail;
	}
	/*
	 * ROUND 32: set the functional clock to 32 MHz before enabling, exactly
	 * as the vendor driver does (i2c-x1.c ky_i2c_probe:
	 * clk_set_rate(clk, ky,i2c-clk-rate) where x1.dtsi sets 32000000).  Our
	 * clock previously read 31.5 MHz (an un-set CCU default); the SCL timing
	 * (LCR/WCR) and the internal APB divider are derived assuming the 32 MHz
	 * functional clock, so a wrong rate can leave the master unable to drive
	 * the bus.  Best-effort: log if the CCU cannot serve it.
	 */
	if (clk_set_freq(sc->func_clk, 32000000, CLK_SET_ROUND_ANY) != 0)
		device_printf(dev,
		    "note: clk_set_freq(func, 32MHz) failed; using current rate\n");
	if (clk_enable(sc->func_clk) != 0) {
		device_printf(dev, "cannot enable func clock\n");
		goto fail;
	}
	if (clk_get_by_ofw_name(dev, 0, "bus", &sc->bus_clk) != 0 ||
	    clk_enable(sc->bus_clk) != 0) {
		device_printf(dev, "cannot enable bus clock\n");
		goto fail;
	}

	/*
	 * ROUND 33: force the TWSI8 func-clock gate ON directly in the APBC.
	 *
	 * APBC_TWSI8_CLK_RST (0xd4015000 + 0x20) reads back 0 (documented HW
	 * quirk), so any read-modify-write on it is destructive.  The i2c8 block
	 * was found UNCLOCKED (all regs read 0, ISR/IBMR = 0) because a stray
	 * RMW (the CCU's TWSI8 reset-deassert, now removed) had wiped the gate.
	 * Belt-and-suspenders: write the full gate-enable value (BIT1|BIT0 = 0x3,
	 * mainline ccu-k1.c twsi8_clk gate) straight to the register so the block
	 * is definitely clocked, independent of clk_enable ordering.  Reg 0x20 is
	 * TWSI8-specific; do this only for the PMIC bus.  (Read-back is
	 * meaningless due to the quirk, so we just log the write.)
	 */
	if (rman_get_start(sc->mem) == 0xd401d800) {
		void *ab = pmap_mapdev(K1_APBC_PHYS, K1_APBC_SIZE);

		if (ab != NULL) {
			volatile uint32_t *r = (volatile uint32_t *)ab;

			r[K1_APBC_TWSI8 / 4] = K1_TWSI8_GATE_EN;
			__asm __volatile("fence" ::: "memory");
			device_printf(dev,
			    "forced TWSI8 clock gate (APBC+0x20 = 0x%x)\n",
			    K1_TWSI8_GATE_EN);
			pmap_unmapdev(ab, K1_APBC_SIZE);
		}
	}

	/*
	 * ROUND 30 diagnostics: log the func/bus clock rates.  If the func
	 * clock reads 0 or an implausible rate, the SCL clock the master
	 * derives from it is wrong and every transfer times out -- a clock-tree
	 * gap, not a driver-logic bug.  (The K1 i2c master divides its func
	 * clock internally by the CR_MODE_FAST bit; there are no separate SCL
	 * load/wait-count registers on this IP -- confirmed against mainline
	 * i2c-k1.c, whose hw_init writes the identical ICR/IRCR set with no
	 * timing registers.)
	 */
	{
		uint64_t ff = 0, bf = 0;

		(void)clk_get_freq(sc->func_clk, &ff);
		(void)clk_get_freq(sc->bus_clk, &bf);
		device_printf(dev,
		    "func clock %ju Hz, bus clock %ju Hz, target %u Hz\n",
		    (uintmax_t)ff, (uintmax_t)bf, sc->bus_freq);
	}

	/*
	 * ROUND 32: pulse the reset (assert -> deassert), matching the vendor
	 * probe (i2c-x1.c: reset_control_assert() then reset_control_deassert()).
	 * A bare deassert is a no-op if firmware already released the reset,
	 * leaving the controller in whatever (possibly wedged) state it was in;
	 * asserting first forces a true controller reset.
	 */
	if (hwreset_get_by_ofw_idx(dev, 0, 0, &sc->reset) == 0) {
		(void)hwreset_assert(sc->reset);
		DELAY(10);
		(void)hwreset_deassert(sc->reset);
		DELAY(10);
	} else
		sc->reset = NULL;

	mtx_init(&sc->mtx, "spacemit_i2c", NULL, MTX_DEF);

	/*
	 * Ensure the PWR_SCL/PWR_SDA pads are muxed to i2c + pulled up before we
	 * touch the bus (independent of the pinctrl pass).  Only for the PMIC
	 * bus at 0xd401d800; other i2c instances configure their own pads via
	 * the DT pinctrl and must not be forced to these pins.
	 */
	if (rman_get_start(sc->mem) == 0xd401d800)
		spacemit_i2c_force_pinmux(dev);

	i2c_unit_reset(sc);
	i2c_hw_init(sc);

	/*
	 * ROUND 32 diagnostic: after full init, log the idle bus line state and
	 * the (now set) func clock rate.  A correctly muxed+pulled+powered bus
	 * idles HIGH (IBMR SDA=1 SCL=1).  Both low = the pads still are not
	 * driving (domain/electrical); both high = the bus is finally alive and a
	 * transfer should now work.
	 */
	{
		uint64_t ff = 0;

		(void)clk_get_freq(sc->func_clk, &ff);
		device_printf(dev,
		    "post-init IBMR=0x%02x (SDA=%d SCL=%d), func %ju Hz\n",
		    I2C_RD4(sc, IBMR), !!(I2C_RD4(sc, IBMR) & BMR_SDA),
		    !!(I2C_RD4(sc, IBMR) & BMR_SCL), (uintmax_t)ff);
	}

	/*
	 * ROUND 33 DECISIVE diagnostic: is the i2c8 register block actually
	 * clocked, or does it read all-zero because TWSI8's clock gate got
	 * wiped?  Write known patterns to two RW registers (LCR 0x10, WCR 0x14)
	 * and read them back.  If they STICK, the block is live (the fix worked /
	 * the fault is elsewhere); if they read 0, the block is still
	 * unclocked/in-reset.  PMIC bus only.
	 */
	if (rman_get_start(sc->mem) == 0xd401d800) {
		uint32_t lrb, wrb;

		I2C_WR4(sc, ILCR, I2C_LCR_VALUE);
		I2C_WR4(sc, IWCR, I2C_WCR_VALUE);
		lrb = I2C_RD4(sc, ILCR);
		wrb = I2C_RD4(sc, IWCR);
		device_printf(dev,
		    "reg-stick test: LCR wrote 0x%08x read 0x%08x, WCR wrote "
		    "0x%08x read 0x%08x %s\n",
		    I2C_LCR_VALUE, lrb, I2C_WCR_VALUE, wrb,
		    (lrb == I2C_LCR_VALUE && wrb == I2C_WCR_VALUE) ?
		    "(STICK -> block clocked)" : "(NOT sticking -> block "
		    "unclocked/in-reset)");
	}

	sc->iicbus = device_add_child(dev, "iicbus", DEVICE_UNIT_ANY);
	if (sc->iicbus == NULL) {
		device_printf(dev, "cannot add iicbus child\n");
		mtx_destroy(&sc->mtx);
		goto fail;
	}

	bus_attach_children(dev);
	return (0);

fail:
	if (sc->reset != NULL)
		hwreset_release(sc->reset);
	if (sc->bus_clk != NULL)
		clk_release(sc->bus_clk);
	if (sc->func_clk != NULL)
		clk_release(sc->func_clk);
	if (sc->mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem);
	return (ENXIO);
}

static int
spacemit_i2c_detach(device_t dev)
{
	struct spacemit_i2c_softc *sc;
	int error;

	sc = device_get_softc(dev);

	error = bus_generic_detach(dev);
	if (error != 0)
		return (error);

	if (sc->reset != NULL)
		hwreset_release(sc->reset);
	if (sc->bus_clk != NULL)
		clk_release(sc->bus_clk);
	if (sc->func_clk != NULL)
		clk_release(sc->func_clk);
	if (sc->mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem);
	mtx_destroy(&sc->mtx);
	return (0);
}

static phandle_t
spacemit_i2c_get_node(device_t bus, device_t dev)
{

	return (ofw_bus_get_node(bus));
}

static device_method_t spacemit_i2c_methods[] = {
	DEVMETHOD(device_probe,		spacemit_i2c_probe),
	DEVMETHOD(device_attach,	spacemit_i2c_attach),
	DEVMETHOD(device_detach,	spacemit_i2c_detach),

	/* OFW glue so iicbus children get their DT nodes. */
	DEVMETHOD(ofw_bus_get_node,	spacemit_i2c_get_node),

	/* iicbus interface. */
	DEVMETHOD(iicbus_callback,	iicbus_null_callback),
	DEVMETHOD(iicbus_reset,		spacemit_i2c_reset),
	DEVMETHOD(iicbus_transfer,	spacemit_i2c_transfer),

	DEVMETHOD_END
};

static driver_t spacemit_i2c_driver = {
	"spacemit_i2c",
	spacemit_i2c_methods,
	sizeof(struct spacemit_i2c_softc)
};

DRIVER_MODULE(spacemit_i2c, simplebus, spacemit_i2c_driver, NULL, NULL);
DRIVER_MODULE(ofw_iicbus, spacemit_i2c, ofw_iicbus_driver, NULL, NULL);
MODULE_DEPEND(spacemit_i2c, iicbus, 1, 1, 1);
MODULE_VERSION(spacemit_i2c, 1);
