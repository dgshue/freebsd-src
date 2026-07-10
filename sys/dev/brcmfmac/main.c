// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2010-2022 Broadcom Corporation
 * Copyright (c) brcmfmac-freebsd contributors
 *
 * Based on the Linux brcmfmac driver.
 */

/*
 * brcmfmac - Broadcom FullMAC WiFi driver for FreeBSD
 *
 * Two DRIVER_MODULE registrations: one for PCIe (pci bus), one for
 * SDIO (sdiob bus). Only the matching one probes at runtime.
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/firmware.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <sys/taskqueue.h>

#include <machine/atomic.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <dev/sdio/sdio_subr.h>
#include <dev/sdio/sdiob.h>

#include "brcmfmac.h"

/* ----------------------------------------------------------------
 * PCIe bus attachment
 * ---------------------------------------------------------------- */

static int brcmf_pci_probe(device_t dev);
static int brcmf_pci_attach(device_t dev);
static int brcmf_pci_detach(device_t dev);

static const struct brcmf_dev_id brcmf_devid_table[] = {
	{ PCI_VENDOR_BROADCOM, PCI_DEVICE_BCM4350, "Broadcom BCM4350 WiFi" },
	{ 0, 0, NULL }
};

static device_method_t brcmf_pci_methods[] = { DEVMETHOD(device_probe,
						   brcmf_pci_probe),
	DEVMETHOD(device_attach, brcmf_pci_attach),
	DEVMETHOD(device_detach, brcmf_pci_detach), DEVMETHOD_END };

static driver_t brcmf_pci_driver = { "brcmfmac", brcmf_pci_methods,
	sizeof(struct brcmf_softc) };

DRIVER_MODULE(if_brcmfmac, pci, brcmf_pci_driver, NULL, NULL);
MODULE_VERSION(if_brcmfmac, 1);
MODULE_DEPEND(if_brcmfmac, pci, 1, 1, 1);
MODULE_DEPEND(if_brcmfmac, firmware, 1, 1, 1);
MODULE_DEPEND(if_brcmfmac, wlan, 1, 1, 1);
MODULE_PNP_INFO("U16:vendor;U16:device", pci, if_brcmfmac, brcmf_devid_table,
    nitems(brcmf_devid_table) - 1);

static int
brcmf_pci_probe(device_t dev)
{
	const struct brcmf_dev_id *id;
	uint16_t vendor, device;

	vendor = pci_get_vendor(dev);
	device = pci_get_device(dev);

	for (id = brcmf_devid_table; id->vendor != 0; id++) {
		if (id->vendor == vendor && id->device == device) {
			device_set_desc(dev, id->desc);
			return (BUS_PROBE_DEFAULT);
		}
	}

	return (ENXIO);
}

static int
brcmf_pci_attach(device_t dev)
{
	return (brcmf_pcie_attach(dev));
}

static int
brcmf_pci_detach(device_t dev)
{
	return (brcmf_pcie_detach(dev));
}

/* ----------------------------------------------------------------
 * SDIO bus attachment
 * ---------------------------------------------------------------- */

#define SDIO_VENDOR_BROADCOM 0x02D0
#define SDIO_DEVICE_BCM43455 0xA9A6
#define SDIO_DEVICE_BCM43455_A9BF 0xA9BF	/* 43455/43456 modules (AP6256) */
#define SDIO_DEVICE_BCM4345  0x4345

static const struct brcmf_dev_id brcmf_sdio_devid_table[] = {
	{ SDIO_VENDOR_BROADCOM, SDIO_DEVICE_BCM43455, "Broadcom BCM43455 WiFi (SDIO)" },
	{ SDIO_VENDOR_BROADCOM, SDIO_DEVICE_BCM43455_A9BF, "Broadcom BCM43455/43456 WiFi (SDIO)" },
	{ SDIO_VENDOR_BROADCOM, SDIO_DEVICE_BCM4345, "Broadcom BCM4345x WiFi (SDIO)" },
	{ 0, 0, NULL }
};

/* CLM blobs, in order of preference (43456 first for the AP6256). */
static const char *brcmf_clm_fw_names[] = {
	"brcmfmac43456-sdio.clm_blob",
	"brcmfmac43455-sdio.clm_blob",
	NULL
};
#define BRCMF_CLM_MAX_CHUNK  1400

/* CLM download header (matches Linux brcmf_dload_data_le) */
struct brcmf_dload_data_le {
	uint16_t flag;
	uint16_t dload_type;
	uint32_t len;
	uint32_t crc;
	uint8_t data[];
} __packed;

#define DL_BEGIN	     0x0002
#define DL_END		     0x0004
#define DL_TYPE_CLM	     2
#define DLOAD_HANDLER_VER    1
#define DLOAD_FLAG_VER_SHIFT 12

static void
brcmf_sdio_load_clm(struct brcmf_softc *sc)
{
	const struct firmware *fw;
	struct brcmf_dload_data_le *chunk;
	uint32_t datalen, cumulative, chunk_len;
	uint16_t dl_flag;
	uint32_t status;
	int error, i;

	fw = NULL;
	for (i = 0; brcmf_clm_fw_names[i] != NULL; i++) {
		fw = firmware_get(brcmf_clm_fw_names[i]);
		if (fw != NULL) {
			device_printf(sc->dev, "using CLM blob %s\n",
			    brcmf_clm_fw_names[i]);
			break;
		}
	}
	if (fw == NULL) {
		device_printf(sc->dev,
		    "no CLM blob available, channels may be limited\n");
		return;
	}

	device_printf(sc->dev, "loading CLM blob (%zu bytes)\n", fw->datasize);

	chunk = malloc(sizeof(*chunk) + BRCMF_CLM_MAX_CHUNK, M_BRCMFMAC,
	    M_WAITOK | M_ZERO);

	datalen = fw->datasize;
	cumulative = 0;
	dl_flag = DL_BEGIN;

	do {
		if (datalen > BRCMF_CLM_MAX_CHUNK) {
			chunk_len = BRCMF_CLM_MAX_CHUNK;
		} else {
			chunk_len = datalen;
			dl_flag |= DL_END;
		}

		chunk->flag = htole16(
		    dl_flag | (DLOAD_HANDLER_VER << DLOAD_FLAG_VER_SHIFT));
		chunk->dload_type = htole16(DL_TYPE_CLM);
		chunk->len = htole32(chunk_len);
		chunk->crc = 0;
		memcpy(chunk->data, (const uint8_t *)fw->data + cumulative,
		    chunk_len);

		/*
		 * Can't use brcmf_fil_iovar_data_set — its 512-byte
		 * stack buffer is too small for CLM chunks. Build the
		 * iovar buffer manually and call ioctl directly.
		 */
		{
			static const char clmload[] = "clmload";
			uint32_t namelen = sizeof(clmload);
			uint32_t paylen = sizeof(*chunk) + chunk_len;
			uint32_t total = namelen + paylen;
			uint8_t *iobuf = malloc(total, M_BRCMFMAC, M_WAITOK);
			memcpy(iobuf, clmload, namelen);
			memcpy(iobuf + namelen, chunk, paylen);
			error = sc->bus_ops->ioctl(sc, 263 /* C_SET_VAR */, 1,
			    iobuf, total, NULL);
			free(iobuf, M_BRCMFMAC);
		}
		if (error != 0) {
			device_printf(sc->dev,
			    "CLM download failed at offset %u: %d\n",
			    cumulative, error);
			break;
		}

		dl_flag &= ~DL_BEGIN;
		cumulative += chunk_len;
		datalen -= chunk_len;
	} while (datalen > 0);

	free(chunk, M_BRCMFMAC);
	firmware_put(fw, FIRMWARE_UNLOAD);

	if (error == 0) {
		error = brcmf_fil_iovar_int_get(sc, "clmload_status", &status);
		if (error == 0 && status != 0) {
			device_printf(sc->dev, "CLM load status: %u\n", status);
		} else if (error == 0) {
			device_printf(sc->dev, "CLM blob loaded\n");
		}
	}
}

static int brcmf_sdio_probe(device_t dev);
static int brcmf_sdio_bus_attach(device_t dev);
static int brcmf_sdio_bus_detach(device_t dev);
static int brcmf_sdio_bus_start(struct brcmf_softc *sc);
static int brcmf_sdio_bus_bringup(struct brcmf_softc *sc);
static void brcmf_sdio_bringup_task(void *arg, int pending);
static int brcmf_sdio_sysctl_bringup(SYSCTL_HANDLER_ARGS);

static device_method_t brcmf_sdio_methods[] = { DEVMETHOD(device_probe,
						    brcmf_sdio_probe),
	DEVMETHOD(device_attach, brcmf_sdio_bus_attach),
	DEVMETHOD(device_detach, brcmf_sdio_bus_detach), DEVMETHOD_END };

static driver_t brcmf_sdio_driver = { "brcmfmac", brcmf_sdio_methods,
	sizeof(struct brcmf_softc) };

DRIVER_MODULE(if_brcmfmac, sdiob, brcmf_sdio_driver, NULL, NULL);
MODULE_DEPEND(if_brcmfmac, sdiob, 1, 1, 1);

static int
brcmf_sdio_bus_start(struct brcmf_softc *sc)
{
	char ver[128];
	char caps[256];
	int error;

	/* Disable BT coexistence ASAP — before any ioctl that might
	 * trigger wl_open. CYW43455 firmware has a bug where btc_mode=1
	 * causes FEM misconfiguration (FIXME bt_coex). */
	brcmf_fil_iovar_int_set(sc, "btc_mode", 0);

	memset(ver, 0, sizeof(ver));
	error = brcmf_fil_iovar_data_get(sc, "ver", ver, sizeof(ver) - 1);
	if (error != 0) {
		device_printf(sc->dev, "firmware ver ioctl failed: %d\n",
		    error);
		return (error);
	}
	{
		char *nl = strchr(ver, '\n');
		if (nl != NULL)
			*nl = '\0';
		device_printf(sc->dev, "firmware: %s\n", ver);
	}

	memset(caps, 0, sizeof(caps));
	error = brcmf_fil_iovar_data_get(sc, "cap", caps, sizeof(caps) - 1);
	if (error == 0)
		device_printf(sc->dev, "cap: %s\n", caps);
	else
		device_printf(sc->dev, "cap iovar failed: %d\n", error);

	/* Linux disables glom on SDIO during preinit. We do not support
	 * glom descriptors yet; leaving it enabled can desynchronize F2 RX. */
	brcmf_fil_cmd_data_set(sc, 89 /* C_SET_GLOM */,
	    &(uint32_t) { htole32(0) }, sizeof(uint32_t));

	brcmf_sdio_load_clm(sc);
	return (0);
}

static int
brcmf_sdio_probe(device_t dev)
{
	const struct brcmf_dev_id *id;
	uint16_t vendor, device;

	vendor = sdio_get_vendor(dev);
	device = sdio_get_device(dev);

	for (id = brcmf_sdio_devid_table; id->vendor != 0; id++) {
		if (id->vendor == vendor && id->device == device) {
			/* Only attach to F1 (backplane access) */
			if (sdio_get_funcnum(dev) != 1)
				return (ENXIO);
			device_set_desc(dev, id->desc);
			return (BUS_PROBE_DEFAULT);
		}
	}
	return (ENXIO);
}

static int
brcmf_sdio_bus_attach(device_t dev)
{
	struct brcmf_softc *sc;
	struct sdio_func *f;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	f = sdio_get_function(dev);
	sc->sdio_func1 = f;

	/*
	 * F2 is a sibling device on the same sdiob bus. We need its
	 * sdio_func pointer for data transfer. Walk the parent's children
	 * to find function 2.
	 */
	{
		device_t parent = device_get_parent(dev);
		device_t *children;
		int nchildren, i;

		if (device_get_children(parent, &children, &nchildren) == 0) {
			for (i = 0; i < nchildren; i++) {
				if (children[i] == dev)
					continue;
				if (sdio_get_vendor(children[i]) ==
					SDIO_VENDOR_BROADCOM &&
				    sdio_get_funcnum(children[i]) == 2) {
					sc->sdio_func2 = sdio_get_function(
					    children[i]);
					break;
				}
			}
			free(children, M_TEMP);
		}
	}

	if (sc->sdio_func2 == NULL) {
		device_printf(dev, "SDIO function 2 not found\n");
		return (ENODEV);
	}

	sc->bus_ops = &brcmf_sdio_bus_ops;
	mtx_init(&sc->ioctl_mtx, "brcmfmac_ioctl", NULL, MTX_DEF);
	brcmf_sdpcm_init(sc);

	/*
	 * ROUND 45 -- DECOUPLE the WLAN firmware bring-up from boot ENTIRELY.
	 *
	 * History: doing the firmware bring-up ANYWHERE in the boot path fails on
	 * this board.  At device-attach (r40-r42) firmware_get() races root-mount
	 * -> namei() null-root store fault ("Root mount waiting for...").  Gating
	 * on a mountroot eventhandler (r43) fixed the panic but running the
	 * bring-up inline in the handler (r43) OR async on the shared
	 * taskqueue_thread (r44) HUNG the boot at root mount -- the long sleeping
	 * SDIO/firmware/HT-clock path either deadlocks against boot work or
	 * monopolizes the shared system taskqueue that other boot work needs.
	 *
	 * So attach registers the device IDLE: SDIO F1/F2 discovery + SDPCM init
	 * are done here (no firmware), but NOTHING that calls firmware_get() runs
	 * at attach or from any boot-time hook.  The kernel boots to multi-user
	 * with brcmfmac attached but with no firmware loaded.
	 *
	 * The firmware-loading bring-up (brcmf_sdio_bus_bringup: firmware_get +
	 * download + HT clock + cfg_attach) runs ON DEMAND, triggered from
	 * userland AFTER full multi-user by writing:
	 *     sysctl dev.brcmfmac.<unit>.bringup=1
	 * It runs on a PRIVATE taskqueue+thread created here -- NEVER the shared
	 * taskqueue_thread -- so even if it hangs, the board is already at
	 * multi-user and is recoverable (reboot/ddb) instead of a power-cycle.
	 */
	TASK_INIT(&sc->bringup_task, 0, brcmf_sdio_bringup_task, sc);
	sc->bringup_tq = taskqueue_create("brcmf_bringup", M_WAITOK,
	    taskqueue_thread_enqueue, &sc->bringup_tq);
	if (sc->bringup_tq == NULL) {
		device_printf(dev, "cannot create bring-up taskqueue\n");
		error = ENXIO;
		goto fail;
	}
	taskqueue_start_threads(&sc->bringup_tq, 1, PWAIT, "%s bringup",
	    device_get_nameunit(dev));

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "bringup",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE | CTLFLAG_NEEDGIANT, sc, 0,
	    brcmf_sdio_sysctl_bringup, "I",
	    "write 1 to run the WiFi firmware bring-up (on-demand, post-boot); "
	    "reads the bring-up state (0=idle 1=requested 2=done)");

	device_printf(dev,
	    "attached IDLE (no firmware). Run 'sysctl dev.%s.%d.bringup=1' "
	    "after multi-user to load firmware and bring up wlan.\n",
	    device_get_name(dev), device_get_unit(dev));

	return (0);

fail:
	if (sc->bringup_tq != NULL) {
		taskqueue_free(sc->bringup_tq);
		sc->bringup_tq = NULL;
	}
	brcmf_sdpcm_stop_poll(sc);
	if (sc->bus_ops != NULL && sc->bus_ops->cleanup != NULL)
		sc->bus_ops->cleanup(sc);
	brcmf_sdio_detach(sc);
	mtx_destroy(&sc->ioctl_mtx);
	return (error);
}

/*
 * ROUND 43/45: the firmware-loading bring-up, split out of bus_attach so it
 * runs on demand (r45) from the private-taskqueue task, never during boot.
 */
static int
brcmf_sdio_bus_bringup(struct brcmf_softc *sc)
{
	int error;

	error = brcmf_sdio_attach(sc);
	if (error != 0)
		return (error);

	brcmf_sdpcm_start_poll(sc);

	error = brcmf_sdio_bus_start(sc);
	if (error != 0)
		return (error);

	error = brcmf_cfg_attach(sc);
	if (error != 0)
		return (error);

	return (0);
}

/*
 * ROUND 45: the on-demand bring-up task -- runs brcmf_sdio_bus_bringup()
 * (firmware_get + download + HT clock + cfg_attach) on the driver's PRIVATE
 * taskqueue thread, triggered from userland long after boot.  A hang here
 * cannot wedge boot; the board is already at multi-user and recoverable.
 */
static void
brcmf_sdio_bringup_task(void *arg, int pending __unused)
{
	struct brcmf_softc *sc = arg;
	int error;

	if (sc->detaching)
		return;

	device_printf(sc->dev, "on-demand WiFi bring-up starting\n");
	error = brcmf_sdio_bus_bringup(sc);
	if (error != 0) {
		device_printf(sc->dev,
		    "on-demand WiFi bring-up failed: %d\n", error);
		/* Allow a retry: clear the request guard on failure. */
		sc->bringup_requested = 0;
		return;
	}
	sc->bringup_done = 1;
	device_printf(sc->dev, "on-demand WiFi bring-up complete\n");
}

/*
 * ROUND 45: the userland trigger.  Reading returns the bring-up state
 * (0=idle, 1=requested/in-progress, 2=done).  Writing a non-zero value
 * schedules the bring-up ONCE on the private taskqueue and returns
 * immediately (does NOT block the sysctl caller on firmware I/O).
 */
static int
brcmf_sdio_sysctl_bringup(SYSCTL_HANDLER_ARGS)
{
	struct brcmf_softc *sc = arg1;
	int state, val, error;

	state = sc->bringup_done ? 2 : (sc->bringup_requested ? 1 : 0);
	val = state;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);

	if (val == 0)
		return (0);
	if (sc->detaching)
		return (ENXIO);
	if (sc->bringup_done)
		return (0);			/* already up */
	if (atomic_cmpset_int(&sc->bringup_requested, 0, 1) == 0)
		return (EINPROGRESS);		/* already scheduled */

	device_printf(sc->dev, "WiFi bring-up requested via sysctl\n");
	taskqueue_enqueue(sc->bringup_tq, &sc->bringup_task);
	return (0);
}

static int
brcmf_sdio_bus_detach(device_t dev)
{
	struct brcmf_softc *sc;

	sc = device_get_softc(dev);
	sc->detaching = 1;

	/*
	 * ROUND 45: tear down the on-demand bring-up machinery.  DRAIN + FREE the
	 * private taskqueue so any queued/in-flight bring-up finishes (or is
	 * cancelled) and cannot touch a torn-down softc.  sc->detaching (set
	 * above) also makes the task early-return if it starts.  Draining is safe
	 * because the bring-up runs on the private tq thread, not in detach's
	 * context.  taskqueue_free implies drain of pending tasks.
	 */
	if (sc->bringup_tq != NULL) {
		taskqueue_drain(sc->bringup_tq, &sc->bringup_task);
		taskqueue_free(sc->bringup_tq);
		sc->bringup_tq = NULL;
	}

	/* Bring firmware down before stopping poll/cleanup */
	if (sc->cfg_attached) {
		brcmf_fil_bss_down(sc);
	}

	sc->fw_dead = 1;

	/* Stop RX poll before tearing down net80211 — the poll task
	 * accesses VAP state that ieee80211_ifdetach destroys. */
	brcmf_sdpcm_stop_poll(sc);

	/* Wake any sleeping ioctl so it doesn't block detach */
	wakeup(&sc->ioctl_completed);

	brcmf_cfg_detach(sc);
	sc->bus_ops->cleanup(sc);
	brcmf_sdio_detach(sc);
	mtx_destroy(&sc->ioctl_mtx);
	return (0);
}
