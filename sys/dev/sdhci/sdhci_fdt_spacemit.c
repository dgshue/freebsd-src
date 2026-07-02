/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Derek Shue <dgshue@gmail.com>
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
 * SDHCI glue driver for the SpacemiT K1 (aka Ky X1) SoC.
 *
 * The controller is a standard SDHCI with vendor-specific PHY/pad control
 * registers in the same register block.  A software RESET_ALL clears the
 * PHY configuration, so it is re-applied after every such reset.  Register
 * semantics documented by the mainline Linux driver
 * (drivers/mmc/host/sdhci-of-k1.c) and its devicetree binding.
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/types.h>
#include <sys/taskqueue.h>
#include <sys/module.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>
#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/phy/phy.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcbrvar.h>
#include <dev/mmc/mmcreg.h>

#include <dev/fdt/fdt_common.h>
#include <dev/mmc/mmc_fdt_helpers.h>

#include <dev/sdhci/sdhci.h>
#include <dev/sdhci/sdhci_fdt.h>

#include "mmcbr_if.h"
#include "sdhci_if.h"

#include "opt_mmccam.h"

/* Vendor registers within the SDHCI block. */
#define	SPACEMIT_SDHC_OP_EXT_REG	0x108
#define	 SDHC_OVRRD_CLK_OEN		(1u << 11)
#define	 SDHC_FORCE_CLK_ON		(1u << 12)
#define	SPACEMIT_SDHC_LEGACY_CTRL_REG	0x10c
#define	 SDHC_GEN_PAD_CLK_ON		(1u << 6)
#define	SPACEMIT_SDHC_TX_CFG_REG	0x11c
#define	 SDHC_TX_INT_CLK_SEL		(1u << 30)
#define	SPACEMIT_SDHC_PHY_CTRL_REG	0x160
#define	 SDHC_PHY_FUNC_EN		(1u << 0)
#define	 SDHC_PHY_PLL_LOCK		(1u << 1)
#define	SPACEMIT_SDHC_PHY_PADCFG_REG	0x178
#define	 SDHC_PHY_DRIVE_SEL_MASK	0x7u
#define	 SDHC_PHY_DRIVE_SEL_DEFAULT	4u
#define	 SDHC_RX_BIAS_CTRL		(1u << 5)

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-sdhci",	1 },
	{ NULL,			0 }
};

struct sdhci_fdt_spacemit_softc {
	struct sdhci_fdt_softc	base;	/* must be first */
	uint32_t		io_freq;
};

/*
 * Restore the vendor PHY/pad configuration.  Required after RESET_ALL,
 * which clears it.  This is the SD-card variant of the sequence (the
 * eMMC-only MMC_CARD_MODE setup is not applied).
 */
static void
sdhci_fdt_spacemit_phy_setup(struct sdhci_fdt_softc *sc, int slotnum)
{
	struct resource *mem;
	uint32_t v;

	mem = sc->mem_res[slotnum];

	v = bus_read_4(mem, SPACEMIT_SDHC_PHY_CTRL_REG);
	v |= SDHC_PHY_FUNC_EN | SDHC_PHY_PLL_LOCK;
	bus_write_4(mem, SPACEMIT_SDHC_PHY_CTRL_REG, v);

	v = bus_read_4(mem, SPACEMIT_SDHC_PHY_PADCFG_REG);
	v &= ~SDHC_PHY_DRIVE_SEL_MASK;
	v |= SDHC_RX_BIAS_CTRL | SDHC_PHY_DRIVE_SEL_DEFAULT;
	bus_write_4(mem, SPACEMIT_SDHC_PHY_PADCFG_REG, v);

	v = bus_read_4(mem, SPACEMIT_SDHC_LEGACY_CTRL_REG);
	v |= SDHC_GEN_PAD_CLK_ON;
	bus_write_4(mem, SPACEMIT_SDHC_LEGACY_CTRL_REG, v);

	v = bus_read_4(mem, SPACEMIT_SDHC_OP_EXT_REG);
	v |= SDHC_OVRRD_CLK_OEN | SDHC_FORCE_CLK_ON;
	bus_write_4(mem, SPACEMIT_SDHC_OP_EXT_REG, v);

	/*
	 * Select the internal clock for the TX path.  Required for the
	 * default-speed/high-speed timings (up to SDR50); without it the
	 * transmit path is misclocked and writes fail with CRC errors
	 * although reads work.
	 */
	v = bus_read_4(mem, SPACEMIT_SDHC_TX_CFG_REG);
	v |= SDHC_TX_INT_CLK_SEL;
	bus_write_4(mem, SPACEMIT_SDHC_TX_CFG_REG, v);
}

/*
 * The controller's capabilities register reports no base clock frequency.
 * Inject the actual rate of the "io" clock so the SDHCI layer computes
 * correct divisors (mirrors the Linux driver's get_max_clock op).
 */
static uint32_t
sdhci_fdt_spacemit_read_4(device_t dev, struct sdhci_slot *slot,
    bus_size_t off)
{
	struct sdhci_fdt_spacemit_softc *sc;
	uint32_t val;

	sc = device_get_softc(dev);
	val = bus_read_4(sc->base.mem_res[slot->num], off);

	if (off == SDHCI_CAPABILITIES && sc->io_freq != 0 &&
	    (val & SDHCI_CLOCK_V3_BASE_MASK) == 0) {
		val |= ((sc->io_freq / 1000000) << SDHCI_CLOCK_BASE_SHIFT) &
		    SDHCI_CLOCK_V3_BASE_MASK;
	}
	return (val);
}

/*
 * Intercept byte-wide register writes so the PHY setup can be re-applied
 * immediately after a software RESET_ALL completes.
 */
static void
sdhci_fdt_spacemit_write_1(device_t dev, struct sdhci_slot *slot,
    bus_size_t off, uint8_t val)
{
	struct sdhci_fdt_softc *sc;
	int timeout;

	sc = device_get_softc(dev);
	bus_write_1(sc->mem_res[slot->num], off, val);

	if (off == SDHCI_SOFTWARE_RESET && (val & SDHCI_RESET_ALL) != 0) {
		timeout = 1000;
		while ((bus_read_1(sc->mem_res[slot->num],
		    SDHCI_SOFTWARE_RESET) & SDHCI_RESET_ALL) != 0 &&
		    --timeout > 0)
			DELAY(10);
		sdhci_fdt_spacemit_phy_setup(sc, slot->num);
	}
}

static int
sdhci_fdt_spacemit_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "SpacemiT K1 SDHCI controller");
	return (BUS_PROBE_SPECIFIC);
}

static int
sdhci_fdt_spacemit_attach(device_t dev)
{
	struct sdhci_fdt_softc *sc;
	clk_t clk_core, clk_io;
	hwreset_t rst;
	uint64_t freq;
	int error;

	sc = device_get_softc(dev);

	/* Enable the bus (core/AXI) and functional (io) clocks. */
	error = clk_get_by_ofw_name(dev, 0, "core", &clk_core);
	if (error != 0) {
		device_printf(dev, "cannot get 'core' clock: %d\n", error);
		return (ENXIO);
	}
	error = clk_enable(clk_core);
	if (error != 0) {
		device_printf(dev, "cannot enable 'core' clock: %d\n", error);
		return (ENXIO);
	}

	error = clk_get_by_ofw_name(dev, 0, "io", &clk_io);
	if (error != 0) {
		device_printf(dev, "cannot get 'io' clock: %d\n", error);
		return (ENXIO);
	}
	error = clk_enable(clk_io);
	if (error != 0) {
		device_printf(dev, "cannot enable 'io' clock: %d\n", error);
		return (ENXIO);
	}

	/* Release the controller resets. */
	if (hwreset_get_by_ofw_name(dev, 0, "axi", &rst) == 0) {
		error = hwreset_deassert(rst);
		if (error != 0)
			device_printf(dev,
			    "warning: cannot deassert 'axi' reset: %d\n",
			    error);
	}
	if (hwreset_get_by_ofw_name(dev, 0, "sdh", &rst) == 0) {
		error = hwreset_deassert(rst);
		if (error != 0)
			device_printf(dev,
			    "warning: cannot deassert 'sdh' reset: %d\n",
			    error);
	}

	/*
	 * Use the functional clock rate as the base clock if the
	 * capabilities register does not provide one ("max-frequency"
	 * from the devicetree still takes precedence).
	 */
	freq = 0;
	error = clk_get_freq(clk_io, &freq);
	if (bootverbose || freq == 0)
		device_printf(dev, "'io' clock rate %ju Hz (error %d)\n",
		    (uintmax_t)freq, error);
	((struct sdhci_fdt_spacemit_softc *)sc)->io_freq = (uint32_t)freq;

	return (sdhci_fdt_attach(dev));
}

static device_method_t sdhci_fdt_spacemit_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe,		sdhci_fdt_spacemit_probe),
	DEVMETHOD(device_attach,	sdhci_fdt_spacemit_attach),

	/* SDHCI registers accessors */
	DEVMETHOD(sdhci_read_4,		sdhci_fdt_spacemit_read_4),
	DEVMETHOD(sdhci_write_1,	sdhci_fdt_spacemit_write_1),

	DEVMETHOD_END
};

extern driver_t sdhci_fdt_driver;

DEFINE_CLASS_1(sdhci_spacemit, sdhci_fdt_spacemit_driver,
    sdhci_fdt_spacemit_methods, sizeof(struct sdhci_fdt_spacemit_softc),
    sdhci_fdt_driver);
DRIVER_MODULE(sdhci_spacemit, simplebus, sdhci_fdt_spacemit_driver, NULL,
    NULL);

#ifndef MMCCAM
MMC_DECLARE_BRIDGE(sdhci_spacemit);
#endif
