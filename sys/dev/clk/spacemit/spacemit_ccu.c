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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Clock and reset controller driver for the SpacemiT K1 (aka Ky X1) SoC,
 * covering the APBC and APMU system-controller register banks.
 *
 * Register layout derived from the devicetree bindings
 * (dt-bindings/clock/spacemit,k1-syscon.h) and the documentation of the
 * mainline Linux drivers (drivers/clk/spacemit/ccu-k1.c,
 * drivers/reset/spacemit/reset-spacemit-k1.c); driver structure follows
 * FreeBSD's jh7110 clock drivers and OpenBSD's smtclock(4).
 *
 * This is a fixed-configuration provider: it reports rates as programmed
 * by the boot firmware and implements gate/reset control.  It does not
 * model the PLL tree or support reprogramming mux/dividers.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <machine/bus.h>

#include <dev/fdt/simplebus.h>
#include <dev/hwreset/hwreset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/syscon/syscon.h>

#include "clkdev_if.h"
#include "hwreset_if.h"
#include "syscon_if.h"

/* Fixed parent rates (PLL1 VCO = 2457.6 MHz, dividers per mainline names). */
#define	K1_PLL1_D3	819200000UL	/* pll1_d3_819p2 */
#define	K1_PLL1_D4	614400000UL	/* pll1_d4_614p4 */
#define	K1_PLL1_D6	409600000UL	/* pll1_d6_409p6 */
#define	K1_PLL1_D11	(2457600000UL / 11)
#define	K1_PLL1_D13	(2457600000UL / 13)
#define	K1_PLL1_D23	(2457600000UL / 23)

struct smccu_clk {
	int		id;		/* binding ID within this bank */
	const char	*name;
	uint32_t	reg;
	int8_t		gate_shift;	/* -1 = no gate */
	int8_t		mux_shift;	/* -1 = no mux */
	int8_t		mux_width;
	int8_t		div_shift;	/* -1 = no divider */
	int8_t		div_width;
	const uint64_t	*prates;	/* parent rates, indexed by mux */
	int		nrates;
	uint64_t	rate;		/* fixed rate when no mux (0 = unknown) */
};

struct smccu_reset {
	int		id;
	uint32_t	reg;
	uint32_t	assert_mask;
	uint32_t	deassert_mask;
};

struct smccu_bank {
	const struct smccu_clk		*clks;
	int				nclks;
	const struct smccu_reset	*resets;
	int				nresets;
};

/*
 * APBC bank (@0xd4015000).
 * UART functional clock mux (FNCLKSEL, bits [6:4]): 0 = 57.6 MHz,
 * 1 = 14.7456 MHz, 2 = 48 MHz.  Gate: bit 1 (func), bit 0 (bus).
 */
static const uint64_t k1_uart_rates[] = { 57600000, 14745600, 48000000 };

static const struct smccu_clk k1_apbc_clks[] = {
	/* id, name,           reg,  gate, mux,w, div,w, parents */
	{ 0,  "k1_uart0",     0x00,  1,  4, 3, -1, 0, k1_uart_rates, 3, 0 },
	{ 1,  "k1_uart2",     0x04,  1,  4, 3, -1, 0, k1_uart_rates, 3, 0 },
	{ 2,  "k1_uart3",     0x24,  1,  4, 3, -1, 0, k1_uart_rates, 3, 0 },
	{ 3,  "k1_uart4",     0x70,  1,  4, 3, -1, 0, k1_uart_rates, 3, 0 },
	{ 4,  "k1_uart5",     0x74,  1,  4, 3, -1, 0, k1_uart_rates, 3, 0 },
	{ 5,  "k1_uart6",     0x78,  1,  4, 3, -1, 0, k1_uart_rates, 3, 0 },
	{ 6,  "k1_uart7",     0x94,  1,  4, 3, -1, 0, k1_uart_rates, 3, 0 },
	{ 7,  "k1_uart8",     0x98,  1,  4, 3, -1, 0, k1_uart_rates, 3, 0 },
	{ 8,  "k1_uart9",     0x9c,  1,  4, 3, -1, 0, k1_uart_rates, 3, 0 },
	{ 9,  "k1_gpio",      0x08,  1, -1, 0, -1, 0, NULL, 0, 24000000 },
	{ 52, "k1_uart0_bus", 0x00,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 53, "k1_uart2_bus", 0x04,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 54, "k1_uart3_bus", 0x24,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 55, "k1_uart4_bus", 0x70,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 56, "k1_uart5_bus", 0x74,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 57, "k1_uart6_bus", 0x78,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 58, "k1_uart7_bus", 0x94,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 59, "k1_uart8_bus", 0x98,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 60, "k1_uart9_bus", 0x9c,  0, -1, 0, -1, 0, NULL, 0, 0 },
	{ 61, "k1_gpio_bus",  0x08,  0, -1, 0, -1, 0, NULL, 0, 0 },
};

/* APBC resets: bit 2 is an active-high reset in each CLK_RST register. */
static const struct smccu_reset k1_apbc_resets[] = {
	{ 0, 0x00, (1u << 2), 0 },	/* RESET_UART0 */
	{ 1, 0x04, (1u << 2), 0 },	/* RESET_UART2 */
	{ 2, 0x24, (1u << 2), 0 },	/* RESET_UART3 */
	{ 3, 0x70, (1u << 2), 0 },	/* RESET_UART4 */
	{ 4, 0x74, (1u << 2), 0 },	/* RESET_UART5 */
	{ 5, 0x78, (1u << 2), 0 },	/* RESET_UART6 */
	{ 6, 0x94, (1u << 2), 0 },	/* RESET_UART7 */
	{ 7, 0x98, (1u << 2), 0 },	/* RESET_UART8 */
	{ 8, 0x9c, (1u << 2), 0 },	/* RESET_UART9 */
	{ 9, 0x08, (1u << 2), 0 },	/* RESET_GPIO */
};

/*
 * APMU bank (@0xd4282800).
 * SDH0/1/2 functional clocks: divider bits [10:8] (divisor = value + 1),
 * mux bits [7:5], gate bit 4, in the respective CLK_RES_CTRL register.
 */
static const uint64_t k1_sdh01_rates[] = {
	K1_PLL1_D6, K1_PLL1_D4, 0, 0, K1_PLL1_D11, K1_PLL1_D13, K1_PLL1_D23
};
static const uint64_t k1_sdh2_rates[] = {
	K1_PLL1_D6, K1_PLL1_D4, 0, K1_PLL1_D3, K1_PLL1_D11, K1_PLL1_D13,
	K1_PLL1_D23
};

#define	APMU_SDH0_CLK_RES_CTRL	0x054
#define	APMU_SDH1_CLK_RES_CTRL	0x058
#define	APMU_USB_CLK_RES_CTRL	0x05c
#define	APMU_SDH2_CLK_RES_CTRL	0x0e0
#define	APMU_EMAC0_CLK_RES_CTRL	0x3e4
#define	APMU_EMAC1_CLK_RES_CTRL	0x3ec

static const struct smccu_clk k1_apmu_clks[] = {
	/* id, name,          reg,                    gate, mux,w, div,w */
	{ 10, "k1_sdh_axi",   APMU_SDH0_CLK_RES_CTRL,  3, -1, 0, -1, 0,
	    NULL, 0, 0 },
	{ 11, "k1_sdh0",      APMU_SDH0_CLK_RES_CTRL,  4,  5, 3,  8, 3,
	    k1_sdh01_rates, nitems(k1_sdh01_rates), 0 },
	{ 12, "k1_sdh1",      APMU_SDH1_CLK_RES_CTRL,  4,  5, 3,  8, 3,
	    k1_sdh01_rates, nitems(k1_sdh01_rates), 0 },
	{ 13, "k1_sdh2",      APMU_SDH2_CLK_RES_CTRL,  4,  5, 3,  8, 3,
	    k1_sdh2_rates, nitems(k1_sdh2_rates), 0 },
	{ 14, "k1_usb_p1",    APMU_USB_CLK_RES_CTRL,   5, -1, 0, -1, 0,
	    NULL, 0, 0 },
	{ 15, "k1_usb_axi",   APMU_USB_CLK_RES_CTRL,   1, -1, 0, -1, 0,
	    NULL, 0, 0 },
	{ 16, "k1_usb30",     APMU_USB_CLK_RES_CTRL,   8, -1, 0, -1, 0,
	    NULL, 0, 0 },
	{ 37, "k1_emac0_bus", APMU_EMAC0_CLK_RES_CTRL, 0, -1, 0, -1, 0,
	    NULL, 0, 0 },
	{ 39, "k1_emac1_bus", APMU_EMAC1_CLK_RES_CTRL, 0, -1, 0, -1, 0,
	    NULL, 0, 0 },
};

/* APMU resets: the listed bit releases the block (deassert = set bit). */
static const struct smccu_reset k1_apmu_resets[] = {
	{ 2,  APMU_SDH0_CLK_RES_CTRL,  0, (1u << 0) },	/* RESET_SDH_AXI */
	{ 3,  APMU_SDH0_CLK_RES_CTRL,  0, (1u << 1) },	/* RESET_SDH0 */
	{ 4,  APMU_SDH1_CLK_RES_CTRL,  0, (1u << 1) },	/* RESET_SDH1 */
	{ 5,  APMU_SDH2_CLK_RES_CTRL,  0, (1u << 1) },	/* RESET_SDH2 */
	{ 6,  APMU_USB_CLK_RES_CTRL,   0, (1u << 4) },	/* RESET_USBP1_AXI */
	{ 7,  APMU_USB_CLK_RES_CTRL,   0, (1u << 0) },	/* RESET_USB_AXI */
	{ 8,  APMU_USB_CLK_RES_CTRL,   0, (1u << 9) },	/* RESET_USB30_AHB */
	{ 9,  APMU_USB_CLK_RES_CTRL,   0, (1u << 10) },	/* RESET_USB30_VCC */
	{ 10, APMU_USB_CLK_RES_CTRL,   0, (1u << 11) },	/* RESET_USB30_PHY */
	{ 35, APMU_EMAC0_CLK_RES_CTRL, 0, (1u << 1) },	/* RESET_EMAC0 */
	{ 36, APMU_EMAC1_CLK_RES_CTRL, 0, (1u << 1) },	/* RESET_EMAC1 */
};

static const struct smccu_bank k1_apbc_bank = {
	.clks = k1_apbc_clks,
	.nclks = nitems(k1_apbc_clks),
	.resets = k1_apbc_resets,
	.nresets = nitems(k1_apbc_resets),
};

static const struct smccu_bank k1_apmu_bank = {
	.clks = k1_apmu_clks,
	.nclks = nitems(k1_apmu_clks),
	.resets = k1_apmu_resets,
	.nresets = nitems(k1_apmu_resets),
};

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-syscon-apbc",	(uintptr_t)&k1_apbc_bank },
	{ "spacemit,k1-syscon-apmu",	(uintptr_t)&k1_apmu_bank },
	{ NULL,				0 }
};

static struct resource_spec res_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE | RF_SHAREABLE },
	RESOURCE_SPEC_END
};

struct smccu_softc {
	device_t		dev;
	struct mtx		mtx;
	struct resource		*mem_res;
	struct clkdom		*clkdom;
	const struct smccu_bank	*bank;
	struct syscon		*syscon;
};

struct smccu_clknode_sc {
	const struct smccu_clk	*def;
};

#define	READ4(_sc, _off)	bus_read_4((_sc)->mem_res, (_off))
#define	WRITE4(_sc, _off, _val)	bus_write_4((_sc)->mem_res, (_off), (_val))

#define	DEVICE_LOCK(_clk)					\
	CLKDEV_DEVICE_LOCK(clknode_get_device(_clk))
#define	DEVICE_UNLOCK(_clk)					\
	CLKDEV_DEVICE_UNLOCK(clknode_get_device(_clk))

/*
 * Clock node methods.
 */
static int
smccu_clknode_init(struct clknode *clk, device_t dev)
{

	clknode_init_parent_idx(clk, 0);
	return (0);
}

static int
smccu_clknode_recalc_freq(struct clknode *clk, uint64_t *freq)
{
	struct smccu_softc *sc;
	struct smccu_clknode_sc *csc;
	const struct smccu_clk *def;
	uint64_t rate;
	uint32_t reg, idx, div;

	sc = device_get_softc(clknode_get_device(clk));
	csc = clknode_get_softc(clk);
	def = csc->def;

	rate = def->rate;
	if (def->mux_shift >= 0) {
		DEVICE_LOCK(clk);
		reg = READ4(sc, def->reg);
		DEVICE_UNLOCK(clk);

		idx = (reg >> def->mux_shift) & ((1u << def->mux_width) - 1);
		rate = (idx < def->nrates) ? def->prates[idx] : 0;
		if (def->div_shift >= 0 && rate != 0) {
			div = ((reg >> def->div_shift) &
			    ((1u << def->div_width) - 1)) + 1;
			rate /= div;
		}
	}

	*freq = rate;
	return (0);
}

static int
smccu_clknode_set_gate(struct clknode *clk, bool enable)
{
	struct smccu_softc *sc;
	struct smccu_clknode_sc *csc;
	const struct smccu_clk *def;
	uint32_t reg;

	sc = device_get_softc(clknode_get_device(clk));
	csc = clknode_get_softc(clk);
	def = csc->def;

	if (def->gate_shift < 0)
		return (0);

	DEVICE_LOCK(clk);
	reg = READ4(sc, def->reg);
	if (enable)
		reg |= (1u << def->gate_shift);
	else
		reg &= ~(1u << def->gate_shift);
	WRITE4(sc, def->reg, reg);
	DEVICE_UNLOCK(clk);

	return (0);
}

static clknode_method_t smccu_clknode_methods[] = {
	CLKNODEMETHOD(clknode_init,		smccu_clknode_init),
	CLKNODEMETHOD(clknode_set_gate,		smccu_clknode_set_gate),
	CLKNODEMETHOD(clknode_recalc_freq,	smccu_clknode_recalc_freq),
	CLKNODEMETHOD_END
};

DEFINE_CLASS_1(smccu_clknode, smccu_clknode_class, smccu_clknode_methods,
    sizeof(struct smccu_clknode_sc), clknode_class);

/*
 * Reset (hwreset) methods.
 */
static const struct smccu_reset *
smccu_reset_lookup(struct smccu_softc *sc, intptr_t id)
{
	int i;

	for (i = 0; i < sc->bank->nresets; i++) {
		if (sc->bank->resets[i].id == id)
			return (&sc->bank->resets[i]);
	}
	return (NULL);
}

static int
smccu_reset_assert(device_t dev, intptr_t id, bool assert)
{
	struct smccu_softc *sc;
	const struct smccu_reset *rst;
	uint32_t mask, reg;

	sc = device_get_softc(dev);
	rst = smccu_reset_lookup(sc, id);
	if (rst == NULL)
		return (ENXIO);

	mask = rst->assert_mask | rst->deassert_mask;

	mtx_lock(&sc->mtx);
	reg = READ4(sc, rst->reg) & ~mask;
	reg |= assert ? rst->assert_mask : rst->deassert_mask;
	WRITE4(sc, rst->reg, reg);
	mtx_unlock(&sc->mtx);

	return (0);
}

static int
smccu_reset_is_asserted(device_t dev, intptr_t id, bool *reset)
{
	struct smccu_softc *sc;
	const struct smccu_reset *rst;
	uint32_t reg;

	sc = device_get_softc(dev);
	rst = smccu_reset_lookup(sc, id);
	if (rst == NULL)
		return (ENXIO);

	mtx_lock(&sc->mtx);
	reg = READ4(sc, rst->reg);
	mtx_unlock(&sc->mtx);

	if (rst->assert_mask != 0)
		*reset = (reg & rst->assert_mask) != 0;
	else
		*reset = (reg & rst->deassert_mask) == 0;

	return (0);
}

/*
 * Syscon interface: expose the bank registers to consumers such as the
 * EMAC driver, which programs RGMII delays through the APMU bank
 * (devicetree property "spacemit,apmu" = <phandle offset>).
 */
static uint32_t
smccu_syscon_read_4(struct syscon *syscon, bus_size_t offset)
{
	struct smccu_softc *sc;

	sc = device_get_softc(syscon->pdev);
	mtx_assert(&sc->mtx, MA_OWNED);
	return (READ4(sc, offset));
}

static int
smccu_syscon_write_4(struct syscon *syscon, bus_size_t offset, uint32_t val)
{
	struct smccu_softc *sc;

	sc = device_get_softc(syscon->pdev);
	mtx_assert(&sc->mtx, MA_OWNED);
	WRITE4(sc, offset, val);
	return (0);
}

static int
smccu_syscon_modify_4(struct syscon *syscon, bus_size_t offset,
    uint32_t clear_bits, uint32_t set_bits)
{
	struct smccu_softc *sc;
	uint32_t val;

	sc = device_get_softc(syscon->pdev);
	mtx_assert(&sc->mtx, MA_OWNED);
	val = READ4(sc, offset);
	val &= ~clear_bits;
	val |= set_bits;
	WRITE4(sc, offset, val);
	return (0);
}

static syscon_method_t smccu_syscon_methods[] = {
	SYSCONMETHOD(syscon_unlocked_read_4,	smccu_syscon_read_4),
	SYSCONMETHOD(syscon_unlocked_write_4,	smccu_syscon_write_4),
	SYSCONMETHOD(syscon_unlocked_modify_4,	smccu_syscon_modify_4),

	SYSCONMETHOD_END
};
DEFINE_CLASS_1(smccu_syscon, smccu_syscon_class, smccu_syscon_methods, 0,
    syscon_class);

/*
 * Device methods.
 */
static int
smccu_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "SpacemiT K1 clock and reset controller");
	return (BUS_PROBE_DEFAULT);
}

static int
smccu_attach(device_t dev)
{
	struct smccu_softc *sc;
	struct clknode_init_def clkdef;
	struct clknode *clk;
	struct smccu_clknode_sc *csc;
	int err, i;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->bank = (const struct smccu_bank *)
	    ofw_bus_search_compatible(dev, compat_data)->ocd_data;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	err = bus_alloc_resources(dev, res_spec, &sc->mem_res);
	if (err != 0) {
		device_printf(dev, "cannot allocate resources: %d\n", err);
		return (ENXIO);
	}

	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL) {
		device_printf(dev, "cannot create clkdom\n");
		return (ENXIO);
	}

	for (i = 0; i < sc->bank->nclks; i++) {
		memset(&clkdef, 0, sizeof(clkdef));
		clkdef.id = sc->bank->clks[i].id;
		clkdef.name = sc->bank->clks[i].name;
		clkdef.parent_names = NULL;
		clkdef.parent_cnt = 0;

		clk = clknode_create(sc->clkdom, &smccu_clknode_class,
		    &clkdef);
		if (clk == NULL) {
			device_printf(dev, "cannot create clk %s\n",
			    clkdef.name);
			return (ENXIO);
		}
		csc = clknode_get_softc(clk);
		csc->def = &sc->bank->clks[i];
		clknode_register(sc->clkdom, clk);
	}

	if (clkdom_finit(sc->clkdom) != 0) {
		device_printf(dev, "cannot finalize clkdom\n");
		return (ENXIO);
	}

	if (bootverbose)
		clkdom_dump(sc->clkdom);

	hwreset_register_ofw_provider(dev);

	sc->syscon = syscon_create_ofw_node(dev, &smccu_syscon_class,
	    ofw_bus_get_node(dev));
	if (sc->syscon == NULL)
		device_printf(dev, "cannot register syscon provider\n");

	return (0);
}

static int
smccu_detach(device_t dev)
{

	return (EBUSY);
}

static void
smccu_device_lock(device_t dev)
{
	struct smccu_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
smccu_device_unlock(device_t dev)
{
	struct smccu_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

static device_method_t smccu_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		smccu_probe),
	DEVMETHOD(device_attach,	smccu_attach),
	DEVMETHOD(device_detach,	smccu_detach),

	/* clkdev interface */
	DEVMETHOD(clkdev_device_lock,	smccu_device_lock),
	DEVMETHOD(clkdev_device_unlock,	smccu_device_unlock),

	/* syscon device locking (shares the bank mutex) */
	DEVMETHOD(syscon_device_lock,	smccu_device_lock),
	DEVMETHOD(syscon_device_unlock,	smccu_device_unlock),

	/* Reset interface */
	DEVMETHOD(hwreset_assert,	smccu_reset_assert),
	DEVMETHOD(hwreset_is_asserted,	smccu_reset_is_asserted),

	DEVMETHOD_END
};

DEFINE_CLASS_0(spacemit_ccu, spacemit_ccu_driver, smccu_methods,
    sizeof(struct smccu_softc));
EARLY_DRIVER_MODULE(spacemit_ccu, simplebus, spacemit_ccu_driver, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_LATE);
MODULE_VERSION(spacemit_ccu, 1);
