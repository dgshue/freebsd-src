// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2010-2022 Broadcom Corporation
 * Copyright (c) brcmfmac-freebsd contributors
 *
 * Based on the Linux brcmfmac driver.
 */

/* Firmware interface layer: IOVAR get/set operations */

#include <sys/param.h>
#include <sys/systm.h>

#include "brcmfmac.h"

/* FWIL command codes */
#define BRCMF_C_UP   2
#define BRCMF_C_DOWN 3
#define BRCMF_C_GET_VAR 262
#define BRCMF_C_SET_VAR 263

/* Buffer size */
#define BRCMF_MSGBUF_MAX_CTL_PKT_SIZE 8192

/*
 * Get an IOVAR value from firmware.
 */
int
brcmf_fil_iovar_data_get(struct brcmf_softc *sc, const char *name,
    void *data, uint32_t len)
{
	char buf[512]; /* must fit name + response; largest is ~256 (ver) */
	uint32_t namelen;
	int error;

	namelen = strlen(name) + 1;
	if (namelen + len > sizeof(buf))
		return (EINVAL);

	memset(buf, 0, namelen + len);
	memcpy(buf, name, namelen);

	error = sc->bus_ops->ioctl(sc, BRCMF_C_GET_VAR, 0, buf,
	    namelen + len, NULL);
	if (error == 0 && data != NULL && len > 0)
		memcpy(data, buf, len);

	return (error);
}

/*
 * Set an IOVAR value in firmware.
 */
int
brcmf_fil_iovar_data_set(struct brcmf_softc *sc, const char *name,
    const void *data, uint32_t len)
{
	char buf[512]; /* must fit name + data; largest is ~170 (wsec_key) */
	uint32_t namelen;

	namelen = strlen(name) + 1;
	if (namelen + len > sizeof(buf))
		return (EINVAL);

	memcpy(buf, name, namelen);
	if (data != NULL && len > 0)
		memcpy(buf + namelen, data, len);

	{
		int err = sc->bus_ops->ioctl(sc, BRCMF_C_SET_VAR, 1, buf,
		    namelen + len, NULL);
		if (err != 0)
			BRCMF_DBG(sc, "iovar_set '%s' failed: %d\n",
			    name, err);
		return err;
	}
}

/*
 * Set a bsscfg-indexed IOVAR.
 *
 * Linux behavior: bsscfg_idx == 0 sends a plain iovar (no prefix).
 * Non-zero sends: "bsscfg:" + name + NUL + bsscfg_idx(le32) + data.
 */
int
brcmf_fil_bsscfg_data_set(struct brcmf_softc *sc, const char *name,
    int bsscfg_idx, const void *data, uint32_t len)
{
	if (bsscfg_idx == 0)
		return brcmf_fil_iovar_data_set(sc, name, data, len);

	{
		char buf[512];
		static const char prefix[] = "bsscfg:";
		uint32_t prefixlen = sizeof(prefix) - 1;
		uint32_t namelen = strlen(name) + 1;
		uint32_t iolen = prefixlen + namelen + 4 + len;
		uint32_t idx_le;
		char *p;
		int err;

		if (iolen > sizeof(buf))
			return (EINVAL);

		p = buf;
		memcpy(p, prefix, prefixlen);
		p += prefixlen;
		memcpy(p, name, namelen);
		p += namelen;
		idx_le = htole32(bsscfg_idx);
		memcpy(p, &idx_le, 4);
		p += 4;
		if (data != NULL && len > 0)
			memcpy(p, data, len);

		err = sc->bus_ops->ioctl(sc, BRCMF_C_SET_VAR, 1, buf,
		    iolen, NULL);
		if (err != 0)
			BRCMF_DBG(sc, "bsscfg_set '%s' idx=%d failed: %d\n",
			    name, bsscfg_idx, err);
		return err;
	}
}

/*
 * Set an integer IOVAR.
 */
int
brcmf_fil_iovar_int_set(struct brcmf_softc *sc, const char *name, uint32_t val)
{
	uint32_t le_val = htole32(val);

	return brcmf_fil_iovar_data_set(sc, name, &le_val, sizeof(le_val));
}

/*
 * ROUND 49e: get a (potentially large) string IOVAR into the caller's buffer.
 * The plain brcmf_fil_iovar_data_get() has a 512-byte internal bounce buffer,
 * too small for the firmware "cap" capability string (mainline uses 768).  For
 * a GET_VAR the firmware writes the response into the same buffer that carried
 * the iovar name, so we can operate on the caller's buffer directly and avoid
 * the bounce entirely.  Returns the response NUL-terminated within buflen.
 */
int
brcmf_fil_iovar_get_buf(struct brcmf_softc *sc, const char *name,
    char *buf, uint32_t buflen)
{
	uint32_t namelen;
	int error;

	namelen = strlen(name) + 1;
	if (buflen < namelen || buflen == 0)
		return (EINVAL);

	memset(buf, 0, buflen);
	memcpy(buf, name, namelen);

	error = sc->bus_ops->ioctl(sc, BRCMF_C_GET_VAR, 0, buf, buflen, NULL);
	buf[buflen - 1] = '\0';	/* guarantee termination for string iovars */

	return (error);
}

/*
 * Get an integer IOVAR.
 */
int
brcmf_fil_iovar_int_get(struct brcmf_softc *sc, const char *name, uint32_t *val)
{
	uint32_t tmp;
	int error;

	error = brcmf_fil_iovar_data_get(sc, name, &tmp, sizeof(tmp));
	if (error == 0 && val != NULL)
		*val = le32toh(tmp);

	return (error);
}

/*
 * Send a command with data to firmware.
 */
int
brcmf_fil_cmd_data_set(struct brcmf_softc *sc, uint32_t cmd,
    const void *data, uint32_t len)
{
	return sc->bus_ops->ioctl(sc, cmd, 1, __DECONST(void *, data), len,
	    NULL);
}

/*
 * Get data from firmware via command.
 */
int
brcmf_fil_cmd_data_get(struct brcmf_softc *sc, uint32_t cmd,
    void *data, uint32_t len)
{
	return sc->bus_ops->ioctl(sc, cmd, 0, data, len, NULL);
}

/*
 * Bring firmware interface up.
 */
int
brcmf_fil_bss_up(struct brcmf_softc *sc)
{
	uint32_t val = 0;

	return brcmf_fil_cmd_data_set(sc, BRCMF_C_UP, &val, sizeof(val));
}

/*
 * Bring firmware interface down.
 */
int
brcmf_fil_bss_down(struct brcmf_softc *sc)
{
	uint32_t val = htole32(1);

	return brcmf_fil_cmd_data_set(sc, BRCMF_C_DOWN, &val, sizeof(val));
}
