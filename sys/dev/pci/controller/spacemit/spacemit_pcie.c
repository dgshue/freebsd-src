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
 * DesignWare PCIe host (root complex) front-end for the SpacemiT K1 (Ky X1)
 * (compatible: "spacemit,k1-pcie").  Subclasses FreeBSD's generic pci_dw
 * DesignWare core (dev/pci/pci_dw.c), providing the K1-specific glue:
 *   - app clocks (dbi/mstr/slv) + resets via the CCU,
 *   - the per-controller APMU control window (reached via the DT
 *     "spacemit,apmu = <&syscon_apmu offset>" property) for soft reset,
 *     PERST#, RC-mode select, aux-power detect and LTSSM enable,
 *   - the dedicated PCIe PHY (our spacemit_pcie_phy driver), and
 *   - the link-status region for link-up detection.
 *
 * This drives ONLY pcie1/pcie2 (the board's dedicated PCIe slots).  It does
 * NOT touch the USB3 combo PHY.  Sequence reimplemented from the mainline
 * Linux driver drivers/pci/controller/dwc/pcie-spacemit-k1.c (GPL-2.0), NOT
 * copied; front-end modeled on dev/pci/pci_dw_mv.c.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/resource.h>

#include <vm/vm.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_pci.h>
#include <dev/ofw/ofwpci.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/phy/phy.h>
#include <dev/syscon/syscon.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcib_private.h>
#include <dev/pci/pci_dw.h>

#include "pcib_if.h"
#include "syscon_if.h"
#include "pci_dw_if.h"

/* APMU per-controller control registers (relative to the DT-supplied offset). */
#define	PCIE_CLK_RESET_CONTROL	0x0000
#define	 LTSSM_EN		(1u << 6)
#define	 PCIE_AUX_PWR_DET	(1u << 9)
#define	 PCIE_RC_PERST		(1u << 12)	/* 1 = assert PERST# */
#define	 APP_HOLD_PHY_RST	(1u << 30)
#define	 DEVICE_TYPE_RC		(1u << 31)
#define	PCIE_CONTROL_LOGIC	0x0004
#define	 PCIE_SOFT_RESET	(1u << 0)

/* Link-status region ("link" reg, offset 0x04). */
#define	K1_PHY_AHB_IRQ_EN	0x0000
#define	 PCIE_INTERRUPT_EN	(1u << 0)
#define	K1_PHY_AHB_LINK_STS	0x0004
#define	 SMLH_LINK_UP		(1u << 1)
#define	 RDLH_LINK_UP		(1u << 12)
#define	INTR_ENABLE		0x0014
#define	 MSI_CTRL_INT		(1u << 11)

#define	PCIE_T_PVPERL_MS	100	/* PERL# deassert delay (CEM spec) */

/*
 * Bounded link-training wait.  With no card in the slot the link never comes
 * up; we must NOT block attach forever (that hangs the whole boot before
 * mountroot).  100 ms is generous for a present card to reach L0.
 */
#define	LINK_TRAIN_TIMEOUT_US	100000
#define	LINK_POLL_DELAY_US	200

#define	PCI_VENDOR_ID_SPACEMIT		0x201f
#define	PCI_DEVICE_ID_SPACEMIT_K1	0x0001

struct spacemit_pcie_softc {
	struct pci_dw_softc	dw_sc;		/* Must be first. */
	device_t		dev;
	phandle_t		node;
	struct resource		*link_res;
	int			link_rid;
	struct syscon		*apmu;
	bus_size_t		apmu_off;
	clk_t			clk_dbi;
	clk_t			clk_mstr;
	clk_t			clk_slv;
	hwreset_t		rst_dbi;
	hwreset_t		rst_mstr;
	hwreset_t		rst_slv;
	phy_t			phy;
};

static struct ofw_compat_data compat_data[] = {
	{ "spacemit,k1-pcie",	1 },
	{ NULL,			0 }
};

#define	LINK_RD4(sc, r)		bus_read_4((sc)->link_res, (r))
#define	LINK_WR4(sc, r, v)	bus_write_4((sc)->link_res, (r), (v))
#define	APMU_MODIFY(sc, r, clr, set) \
	SYSCON_MODIFY_4((sc)->apmu, (sc)->apmu_off + (r), (clr), (set))

static void
spacemit_pcie_toggle_soft_reset(struct spacemit_pcie_softc *sc)
{

	APMU_MODIFY(sc, PCIE_CONTROL_LOGIC, 0, PCIE_SOFT_RESET);
	(void)SYSCON_READ_4(sc->apmu, sc->apmu_off + PCIE_CONTROL_LOGIC);
	DELAY(2000);
	APMU_MODIFY(sc, PCIE_CONTROL_LOGIC, PCIE_SOFT_RESET, 0);
}

static int
spacemit_pcie_enable_resources(struct spacemit_pcie_softc *sc)
{

	if (clk_enable(sc->clk_dbi) != 0 || clk_enable(sc->clk_mstr) != 0 ||
	    clk_enable(sc->clk_slv) != 0)
		return (ENXIO);
	(void)hwreset_deassert(sc->rst_dbi);
	(void)hwreset_deassert(sc->rst_mstr);
	(void)hwreset_deassert(sc->rst_slv);
	return (0);
}

/* pci_dw_if: report link-up status. */
static int
spacemit_pcie_get_link(device_t dev, bool *status)
{
	struct spacemit_pcie_softc *sc;
	uint32_t val;

	sc = device_get_softc(dev);
	val = LINK_RD4(sc, K1_PHY_AHB_LINK_STS);
	*status = (val & RDLH_LINK_UP) && (val & SMLH_LINK_UP);
	return (0);
}

/*
 * Root-complex bring-up (mirrors mainline k1_pcie_init).  Returns 0 on success
 * (clocks/resets up and the controller reachable), or an error if the fabric
 * could not be brought up at all.  A DOWN link (empty slot) is NOT an error --
 * *linkup is set to reflect it, and the caller decides whether to enumerate.
 * Every poll here is bounded so an empty slot can never hang attach/boot.
 */
static int
spacemit_pcie_init_rc(struct spacemit_pcie_softc *sc, bool *linkup)
{
	bool status;
	int error, us;

	*linkup = false;

	spacemit_pcie_toggle_soft_reset(sc);

	error = spacemit_pcie_enable_resources(sc);
	if (error != 0)
		return (error);

	/* Set the SpacemiT vendor/device ID (DBI writable via pci_dw). */
	pci_dw_dbi_wr2(sc->dev, PCIR_VENDOR, PCI_VENDOR_ID_SPACEMIT);
	pci_dw_dbi_wr2(sc->dev, PCIR_DEVICE, PCI_DEVICE_ID_SPACEMIT_K1);

	/* Assert PERST#, wait the CEM-spec power-stable delay. */
	APMU_MODIFY(sc, PCIE_CLK_RESET_CONTROL, 0, PCIE_RC_PERST);
	(void)SYSCON_READ_4(sc->apmu, sc->apmu_off + PCIE_CLK_RESET_CONTROL);
	DELAY(PCIE_T_PVPERL_MS * 1000);

	/* Root-complex mode + aux power present. */
	APMU_MODIFY(sc, PCIE_CLK_RESET_CONTROL, 0,
	    DEVICE_TYPE_RC | PCIE_AUX_PWR_DET);

	/*
	 * Bring up the PHY (dedicated PCIe PHY; USB3 combo PHY untouched).
	 * The PHY's PLL-lock poll is internally bounded.  Treat a PHY that
	 * fails to lock as "no link" (typical for an EMPTY SLOT) -- NON-FATAL:
	 * leave the link down and let the caller skip enumeration cleanly
	 * rather than proceeding into DBI/link-dependent init that can hang.
	 */
	if (sc->phy != NULL) {
		error = phy_enable(sc->phy);
		if (error != 0) {
			device_printf(sc->dev,
			    "PCIe PHY did not come up (no card?); link down\n");
			return (0);	/* non-fatal: *linkup stays false */
		}
	}

	/* Deassert PERST#. */
	APMU_MODIFY(sc, PCIE_CLK_RESET_CONTROL, PCIE_RC_PERST, 0);

	/* Stop holding the PHY in reset and enable link training (LTSSM). */
	APMU_MODIFY(sc, PCIE_CLK_RESET_CONTROL, APP_HOLD_PHY_RST, LTSSM_EN);

	/* Enable link/MSI interrupts in the link region. */
	LINK_WR4(sc, INTR_ENABLE, MSI_CTRL_INT);
	LINK_WR4(sc, K1_PHY_AHB_IRQ_EN,
	    LINK_RD4(sc, K1_PHY_AHB_IRQ_EN) | PCIE_INTERRUPT_EN);

	/*
	 * Wait (bounded) for link training to reach L0.  With no card the link
	 * never comes up; we simply time out and report link-down -- we do NOT
	 * block boot.  A present card normally links within a few ms.
	 */
	for (us = 0; us < LINK_TRAIN_TIMEOUT_US; us += LINK_POLL_DELAY_US) {
		if (spacemit_pcie_get_link(sc->dev, &status) == 0 && status) {
			*linkup = true;
			break;
		}
		DELAY(LINK_POLL_DELAY_US);
	}
	if (!*linkup)
		device_printf(sc->dev, "PCIe link down (no device in slot)\n");

	return (0);
}

static int
spacemit_pcie_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "SpacemiT K1 PCIe");
	return (BUS_PROBE_DEFAULT);
}

static int
spacemit_pcie_attach(device_t dev)
{
	struct resource_map_request req;
	struct resource_map map;
	struct spacemit_pcie_softc *sc;
	pcell_t apmu_prop[2];
	phandle_t node;
	bool linkup;
	int rid, error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);
	sc->node = node;

	/* DBI region (reg index 0), mapped non-prefetchable device memory. */
	rid = 0;
	sc->dw_sc.dbi_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE | RF_UNMAPPED);
	if (sc->dw_sc.dbi_res == NULL) {
		device_printf(dev, "cannot allocate DBI memory\n");
		return (ENXIO);
	}
	resource_init_map_request(&req);
	req.memattr = VM_MEMATTR_DEVICE;
	error = bus_map_resource(dev, SYS_RES_MEMORY, sc->dw_sc.dbi_res, &req,
	    &map);
	if (error != 0) {
		device_printf(dev, "cannot map DBI memory\n");
		return (error);
	}
	rman_set_mapping(sc->dw_sc.dbi_res, &map);

	/* Link-status region (reg index 3, reg-name "link"). */
	sc->link_rid = 3;
	sc->link_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->link_rid, RF_ACTIVE);
	if (sc->link_res == NULL) {
		device_printf(dev, "cannot allocate link region\n");
		return (ENXIO);
	}

	/* APMU control window via the "spacemit,apmu" phandle+offset. */
	if (OF_getencprop(node, "spacemit,apmu", apmu_prop,
	    sizeof(apmu_prop)) != sizeof(apmu_prop)) {
		device_printf(dev, "missing 'spacemit,apmu' property\n");
		return (ENXIO);
	}
	if (syscon_get_by_ofw_node(dev, OF_node_from_xref(apmu_prop[0]),
	    &sc->apmu) != 0) {
		device_printf(dev, "cannot get APMU syscon\n");
		return (ENXIO);
	}
	sc->apmu_off = apmu_prop[1];

	/* App clocks + resets. */
	if (clk_get_by_ofw_name(dev, 0, "dbi", &sc->clk_dbi) != 0 ||
	    clk_get_by_ofw_name(dev, 0, "mstr", &sc->clk_mstr) != 0 ||
	    clk_get_by_ofw_name(dev, 0, "slv", &sc->clk_slv) != 0) {
		device_printf(dev, "cannot get app clocks\n");
		return (ENXIO);
	}
	if (hwreset_get_by_ofw_name(dev, 0, "dbi", &sc->rst_dbi) != 0 ||
	    hwreset_get_by_ofw_name(dev, 0, "mstr", &sc->rst_mstr) != 0 ||
	    hwreset_get_by_ofw_name(dev, 0, "slv", &sc->rst_slv) != 0) {
		device_printf(dev, "cannot get app resets\n");
		return (ENXIO);
	}

	/*
	 * The dedicated PCIe PHY is referenced (unnamed, index 0) from the
	 * root-port child node (`pcie@0`), matching mainline's parse_port.
	 */
	sc->phy = NULL;
	{
		phandle_t port;

		for (port = OF_child(node); port != 0; port = OF_peer(port)) {
			if (!ofw_bus_node_status_okay(port))
				continue;
			if (phy_get_by_ofw_idx(dev, port, 0, &sc->phy) == 0)
				break;
		}
	}

	/*
	 * K1-specific root-complex bring-up before the generic DWC init.
	 * A down link (empty slot) is non-fatal and reported via linkup; the
	 * bring-up itself only fails if the fabric clocks/resets can't come up.
	 */
	error = spacemit_pcie_init_rc(sc, &linkup);
	if (error != 0) {
		device_printf(dev, "root-complex bring-up failed\n");
		return (error);
	}

	/*
	 * Bring up the generic DesignWare core (DBI is clock-gated, not
	 * link-gated, so this is safe with the link down).  Config-space
	 * enumeration of downstream buses is gated on link-up by
	 * spacemit_pcie_get_link(), so an empty slot simply enumerates nothing
	 * rather than hanging.
	 */
	error = pci_dw_init(dev);
	if (error != 0)
		return (error);

	bus_attach_children(dev);
	return (0);
}

static device_method_t spacemit_pcie_methods[] = {
	DEVMETHOD(device_probe,		spacemit_pcie_probe),
	DEVMETHOD(device_attach,	spacemit_pcie_attach),

	DEVMETHOD(pci_dw_get_link,	spacemit_pcie_get_link),

	DEVMETHOD_END
};

DEFINE_CLASS_1(spacemit_pcie, spacemit_pcie_driver, spacemit_pcie_methods,
    sizeof(struct spacemit_pcie_softc), pci_dw_driver);

DRIVER_MODULE(spacemit_pcie, simplebus, spacemit_pcie_driver, NULL, NULL);
MODULE_DEPEND(spacemit_pcie, pci_dw, 1, 1, 1);
MODULE_VERSION(spacemit_pcie, 1);
