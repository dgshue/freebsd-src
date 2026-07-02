/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Daniel Shue <dgshue@gmail.com>
 *
 * USB3 combo PHY driver for the SpacemiT K1 (Ky X1) RISC-V SoC
 * (compatible: spacemit,k1-combo-phy).  Only USB3 mode is supported here
 * (the PHY is also usable for PCIe/SATA).  Init sequence and register
 * values from the vendor Linux driver
 * (drivers/phy/spacemit/phy-spacemit-k1x-combphy.c); clocks/resets are
 * left as the boot firmware programmed them.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/phy/phy.h>

#define	PHY_TYPE_USB3			4

#define	COMBPHY_USB_REG1		0x68
#define	COMBPHY_USB_REG1_VAL		0x0
#define	COMBPHY_USB_REG2		(0x12 << 2)
#define	COMBPHY_USB_REG2_VAL		0x603a2276
#define	COMBPHY_USB_REG3		(0x02 << 2)
#define	COMBPHY_USB_REG3_VAL		0x97c
#define	COMBPHY_USB_REG4		(0x06 << 2)
#define	COMBPHY_USB_REG4_VAL		0x0
#define	COMBPHY_USB_PLL_REG		0x8
#define	COMBPHY_USB_PLL_MASK		0x1

struct spcomb_softc {
	device_t	dev;
	struct resource	*mem_res;
};

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-combo-phy",	1 },
	{ NULL,				0 }
};

#define	RD4(sc, r)	bus_read_4((sc)->mem_res, (r))
#define	WR4(sc, r, v)	bus_write_4((sc)->mem_res, (r), (v))

static int
spcomb_enable(struct phynode *phynode, bool enable)
{
	struct spcomb_softc *sc;
	device_t dev;
	int i;

	dev = phynode_get_device(phynode);
	sc = device_get_softc(dev);

	/* Only USB3 mode is handled. */
	if (!enable || phynode_get_id(phynode) != PHY_TYPE_USB3)
		return (0);

	WR4(sc, COMBPHY_USB_REG1, COMBPHY_USB_REG1_VAL);
	WR4(sc, COMBPHY_USB_REG2, COMBPHY_USB_REG2_VAL);
	WR4(sc, COMBPHY_USB_REG3, COMBPHY_USB_REG3_VAL);
	WR4(sc, COMBPHY_USB_REG4, COMBPHY_USB_REG4_VAL);

	/* Wait for the PLL to lock. */
	for (i = 0; i < 1000; i++) {
		if (RD4(sc, COMBPHY_USB_PLL_REG) & COMBPHY_USB_PLL_MASK)
			break;
		DELAY(100);
	}
	if (i == 1000)
		device_printf(dev, "warning: USB3 PHY PLL not locked\n");

	return (0);
}

static phynode_method_t spcomb_phynode_methods[] = {
	PHYNODEMETHOD(phynode_enable,	spcomb_enable),
	PHYNODEMETHOD_END
};
DEFINE_CLASS_1(spcomb_phynode, spcomb_phynode_class, spcomb_phynode_methods,
    0, phynode_class);

static int
spcomb_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "SpacemiT K1 combo PHY (USB3)");
	return (BUS_PROBE_DEFAULT);
}

static int
spcomb_attach(device_t dev)
{
	struct spcomb_softc *sc;
	struct phynode_init_def phy_init;
	struct phynode *phynode;
	int rid;

	sc = device_get_softc(dev);
	sc->dev = dev;

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "cannot allocate registers\n");
		return (ENXIO);
	}

	bzero(&phy_init, sizeof(phy_init));
	phy_init.id = PHY_TYPE_USB3;
	phy_init.ofw_node = ofw_bus_get_node(dev);
	phynode = phynode_create(dev, &spcomb_phynode_class, &phy_init);
	if (phynode == NULL) {
		device_printf(dev, "failed to create combo PHY\n");
		return (ENXIO);
	}
	if (phynode_register(phynode) == NULL) {
		device_printf(dev, "failed to register combo PHY\n");
		return (ENXIO);
	}
	return (0);
}

static device_method_t spcomb_methods[] = {
	DEVMETHOD(device_probe,		spcomb_probe),
	DEVMETHOD(device_attach,	spcomb_attach),
	DEVMETHOD_END
};

static driver_t spcomb_driver = {
	"spacemit_combphy",
	spcomb_methods,
	sizeof(struct spcomb_softc),
};

EARLY_DRIVER_MODULE(spacemit_combphy, simplebus, spcomb_driver, 0, 0,
    BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_EARLY);
