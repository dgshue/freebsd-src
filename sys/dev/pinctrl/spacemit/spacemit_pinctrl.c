/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Derek Shue <dgshue@gmail.com>
 *
 * Pin mux (pinctrl) driver for the SpacemiT K1 (Ky X1) RISC-V SoC
 * (compatible: spacemit,k1-pinctrl).  Register semantics and the pin->
 * register offset mapping follow the mainline Linux driver
 * (drivers/pinctrl/spacemit/pinctrl-k1.c).
 *
 * Each pad has a 32-bit MFPR register; the low 3 bits select the mux
 * function.  Devicetree pin groups carry a "pinmux" array where each
 * entry is (pin << 16) | function.  We apply the whole tree at attach so
 * that peripherals whose pins the bootloader did not configure (e.g. the
 * second EMAC) come up muxed before their drivers attach.  Bias / drive
 * strength are left at their reset/firmware values for now.
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

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/fdt/fdt_pinctrl.h>

#include <dev/clk/clk.h>

#include "fdt_pinctrl_if.h"

#define	PAD_MUX_MASK	0x7

struct sppinctrl_softc {
	device_t	dev;
	struct mtx	mtx;
	struct resource	*mem_res;
};

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-pinctrl",	1 },
	{ NULL,				0 }
};

#define	RD4(sc, r)	bus_read_4((sc)->mem_res, (r))
#define	WR4(sc, r, v)	bus_write_4((sc)->mem_res, (r), (v))

/* Map a pin number to its MFPR register offset (K1 layout). */
static unsigned int
k1_pin_to_offset(unsigned int pin)
{
	unsigned int off = 0;

	if (pin <= 85)
		off = pin + 1;
	else if (pin <= 92)
		off = pin + 37;
	else if (pin <= 97)
		off = pin + 24;
	else if (pin == 98)
		off = 93;
	else if (pin == 99)
		off = 92;
	else if (pin == 100)
		off = 91;
	else if (pin == 101)
		off = 90;
	else if (pin == 102)
		off = 95;
	else if (pin == 103)
		off = 94;
	else if (pin <= 110)
		off = pin + 6;
	else if (pin <= 127)
		off = pin + 20;
	return (off << 2);
}

static void
sppinctrl_apply_pinmux(struct sppinctrl_softc *sc, phandle_t node)
{
	pcell_t *pinmux;
	uint32_t reg, val;
	unsigned int pin, mux, off;
	int i, n;

	n = OF_getencprop_alloc_multi(node, "pinmux", sizeof(pcell_t),
	    (void **)&pinmux);
	if (n <= 0)
		return;

	mtx_lock(&sc->mtx);
	for (i = 0; i < n; i++) {
		pin = pinmux[i] >> 16;
		mux = pinmux[i] & PAD_MUX_MASK;
		if (pin > 127)
			continue;
		off = k1_pin_to_offset(pin);
		reg = RD4(sc, off);
		val = (reg & ~PAD_MUX_MASK) | mux;
		if (val != reg)
			WR4(sc, off, val);
	}
	mtx_unlock(&sc->mtx);
	OF_prop_free(pinmux);
}

static int
sppinctrl_configure(device_t dev, phandle_t cfgxref)
{
	struct sppinctrl_softc *sc = device_get_softc(dev);
	phandle_t node, child;

	node = OF_node_from_xref(cfgxref);

	/* The pinmux array may be on this node or on child pin subnodes. */
	if (OF_hasprop(node, "pinmux"))
		sppinctrl_apply_pinmux(sc, node);
	for (child = OF_child(node); child != 0; child = OF_peer(child)) {
		if (OF_hasprop(child, "pinmux"))
			sppinctrl_apply_pinmux(sc, child);
	}
	return (0);
}

static int
sppinctrl_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "SpacemiT K1 pinctrl");
	return (BUS_PROBE_DEFAULT);
}

static int
sppinctrl_attach(device_t dev)
{
	struct sppinctrl_softc *sc = device_get_softc(dev);
	clk_t clk;
	int i, rid;

	sc->dev = dev;
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "cannot allocate registers\n");
		mtx_destroy(&sc->mtx);
		return (ENXIO);
	}

	/* Enable the block's clocks if present (best effort). */
	for (i = 0; clk_get_by_ofw_index(dev, 0, i, &clk) == 0; i++)
		(void)clk_enable(clk);

	/*
	 * Register and apply the whole pin-config tree now, before the
	 * peripheral drivers (e.g. the EMACs) attach in the default pass.
	 */
	fdt_pinctrl_register(dev, NULL);
	fdt_pinctrl_configure_tree(dev);
	return (0);
}

static device_method_t sppinctrl_methods[] = {
	DEVMETHOD(device_probe,		sppinctrl_probe),
	DEVMETHOD(device_attach,	sppinctrl_attach),

	/* fdt_pinctrl interface */
	DEVMETHOD(fdt_pinctrl_configure, sppinctrl_configure),

	DEVMETHOD_END
};

static driver_t sppinctrl_driver = {
	"spacemit_pinctrl",
	sppinctrl_methods,
	sizeof(struct sppinctrl_softc),
};

EARLY_DRIVER_MODULE(spacemit_pinctrl, simplebus, sppinctrl_driver, 0, 0,
    BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_EARLY);
