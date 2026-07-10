// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2010-2022 Broadcom Corporation
 * Copyright (c) brcmfmac-freebsd contributors
 *
 * Based on the Linux brcmfmac driver.
 */

/* Security: wsec/wpa_auth configuration, key installation, PSK */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/endian.h>
#include <sys/bus.h>
#include <sys/socket.h>
#include <sys/sysctl.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_media.h>
#include <net/ethernet.h>

#include <net80211/ieee80211_var.h>

#include <crypto/sha1.h>

#include "cfg.h"

uint32_t
brcmf_detect_security(struct brcmf_scan_result *sr, uint32_t *wpa_auth)
{
	/*
	 * PRIVACY bit alone doesn't distinguish WEP from WPA/WPA2;
	 * that requires parsing RSN/WPA IEs. Flag it as "encrypted"
	 * so the caller can reject what it can't handle.
	 */
	*wpa_auth = WPA_AUTH_DISABLED;

	if (sr->capinfo & IEEE80211_CAPINFO_PRIVACY)
		return WEP_ENABLED;

	return WSEC_NONE;
}

int
brcmf_set_security(struct brcmf_softc *sc, uint32_t wsec, uint32_t wpa_auth)
{
	int error;

	/* C_SET_AUTH: open system. Linux sets this per-connect. */
	{
		uint32_t val = htole32(0);
		error = brcmf_fil_cmd_data_set(sc, 22 /* BRCMF_C_SET_AUTH */,
		    &val, sizeof(val));
		if (error != 0)
			device_printf(sc->dev, "set auth: %d\n", error);
	}

	error = brcmf_fil_iovar_int_set(sc, "wsec", wsec);
	if (error != 0) {
		device_printf(sc->dev, "failed to set wsec: %d\n", error);
		return error;
	}

	error = brcmf_fil_iovar_int_set(sc, "wpa_auth", wpa_auth);
	if (error != 0) {
		device_printf(sc->dev, "failed to set wpa_auth: %d\n", error);
		return error;
	}

	return 0;
}

/*
 * ROUND 49 (AP Route A) -- configure WPA2-PSK on the hostap firmware
 * authenticator.  The FullMAC firmware runs the 4-way handshake itself, so we
 * only push it the PMK/passphrase plus the cipher (CCMP/AES) + AKM (WPA2-PSK)
 * iovars; the firmware then advertises the RSN IE in beacons/probe-responses
 * and negotiates the PTK/GTK with the client.  There is no hostapd and no
 * in-host EAPOL on this path.
 *
 * Ground truth: Linux brcmf_cfg80211_start_ap() ->
 *   (after C_UP) brcmf_set_pmk() [C_SET_WSEC_PMK] ->
 *   brcmf_parse_configure_security()/brcmf_configure_wpaie()
 *     [bsscfg "auth"=0, "wsec"=AES, "wpa_auth"=WPA2_AUTH_PSK]
 * (contrib .../brcmfmac/cfg80211.c).  The port issues the plain (primary-
 * interface / bsscfgidx 0) iovars, which are what those bsscfg iovars reduce
 * to for the first BSS.
 *
 * ROUND 49b (hardware-validated failure fix): the r49 code pushed the
 * passphrase itself with flags=BRCMF_WSEC_PASSPHRASE and key_len=strlen -- the
 * AP6256/43456 firmware rejects that on the AP path (SET_WSEC_PMK error 5/EIO
 * on hardware).  Mainline Linux NEVER sends a passphrase for a WPA2 AP:
 * brcmf_cfg80211_start_ap() -> brcmf_set_pmk(ifp, crypto->psk,
 * BRCMF_WSEC_MAX_PSK_LEN (=32)) -> brcmf_set_wsec(..., flags=0) pushes a
 * RAW 32-byte PMK with key_len=32, flags=0 (cfg80211.c:5419-5424 + 1766-1798;
 * nl80211 crypto->psk is always the derived PMK).  BRCMF_WSEC_PASSPHRASE
 * appears in mainline only for SAE passwords (wcc/core.c).  Note the
 * passphrase form is also logically impossible at this point in the AP
 * sequence: PBKDF2 needs the SSID and C_SET_SSID has not been issued yet.
 *
 * So the driver now derives the PMK itself: PMK = PBKDF2-HMAC-SHA1(passphrase,
 * ssid, 4096, 32) per IEEE 802.11 Annex J, using the kernel's crypto/sha1.c
 * (always present: conf/files marks it optional on `ether`).  A one-time cost
 * of ~8k HMACs at AP start.
 *
 * PMK source: the driver's own `psk` sysctl (dev.brcmfmac.<u>.psk).  net80211
 * does not keep the PSK in-kernel for a hostap vap (hostapd normally owns it),
 * and Route A has no hostapd, so the sysctl is the authoritative source.  An
 * 8-63 char string is a passphrase (PMK derived in-driver as above); a
 * 64-hex-char string is decoded directly to the raw 256-bit PMK.
 */
static int
brcmf_hexval(char c)
{
	if (c >= '0' && c <= '9')
		return (c - '0');
	if (c >= 'a' && c <= 'f')
		return (c - 'a' + 10);
	if (c >= 'A' && c <= 'F')
		return (c - 'A' + 10);
	return (-1);
}

/*
 * HMAC-SHA1 over up to two data segments (avoids concatenation buffers in
 * PBKDF2).  Key is the WPA passphrase (8-63 chars, always < the 64-byte SHA1
 * block size, but handle the general case anyway).
 */
static void
brcmf_hmac_sha1(const uint8_t *key, size_t keylen,
    const uint8_t *d1, size_t l1, const uint8_t *d2, size_t l2,
    uint8_t out[SHA1_RESULTLEN])
{
	struct sha1_ctxt ctx;
	uint8_t kbuf[64], pad[64];
	int i;

	memset(kbuf, 0, sizeof(kbuf));
	if (keylen > sizeof(kbuf)) {
		sha1_init(&ctx);
		sha1_loop(&ctx, key, keylen);
		sha1_result(&ctx, (char *)kbuf);
	} else {
		memcpy(kbuf, key, keylen);
	}

	for (i = 0; i < 64; i++)
		pad[i] = kbuf[i] ^ 0x36;	/* ipad */
	sha1_init(&ctx);
	sha1_loop(&ctx, pad, sizeof(pad));
	if (l1 > 0)
		sha1_loop(&ctx, d1, l1);
	if (l2 > 0)
		sha1_loop(&ctx, d2, l2);
	sha1_result(&ctx, (char *)out);

	for (i = 0; i < 64; i++)
		pad[i] = kbuf[i] ^ 0x5c;	/* opad */
	sha1_init(&ctx);
	sha1_loop(&ctx, pad, sizeof(pad));
	sha1_loop(&ctx, out, SHA1_RESULTLEN);
	sha1_result(&ctx, (char *)out);

	explicit_bzero(kbuf, sizeof(kbuf));
	explicit_bzero(pad, sizeof(pad));
}

/*
 * PBKDF2-HMAC-SHA1 (RFC 2898).  For WPA2: iters=4096, salt=SSID, outlen=32
 * (IEEE 802.11 Annex J PSK-to-PMK mapping -- identical to what the client
 * derives from the same passphrase+SSID).
 */
static void
brcmf_pbkdf2_sha1(const uint8_t *pass, size_t passlen,
    const uint8_t *salt, size_t saltlen, uint32_t iters,
    uint8_t *out, size_t outlen)
{
	uint8_t u[SHA1_RESULTLEN], t[SHA1_RESULTLEN], cnt[4];
	uint32_t blk, it;
	size_t off, n;
	int i;

	for (blk = 1, off = 0; off < outlen; blk++) {
		cnt[0] = (blk >> 24) & 0xff;
		cnt[1] = (blk >> 16) & 0xff;
		cnt[2] = (blk >> 8) & 0xff;
		cnt[3] = blk & 0xff;
		brcmf_hmac_sha1(pass, passlen, salt, saltlen, cnt, 4, u);
		memcpy(t, u, SHA1_RESULTLEN);
		for (it = 1; it < iters; it++) {
			brcmf_hmac_sha1(pass, passlen, u, SHA1_RESULTLEN,
			    NULL, 0, u);
			for (i = 0; i < SHA1_RESULTLEN; i++)
				t[i] ^= u[i];
		}
		n = outlen - off;
		if (n > SHA1_RESULTLEN)
			n = SHA1_RESULTLEN;
		memcpy(out + off, t, n);
		off += n;
	}
	explicit_bzero(u, sizeof(u));
	explicit_bzero(t, sizeof(t));
}

int
brcmf_ap_set_wpa2(struct brcmf_softc *sc, const uint8_t *ssid,
    uint32_t ssidlen)
{
	struct brcmf_wsec_pmk_le pmk;
	uint32_t val;
	int error, i, is_hex;

	if (sc->psk_len == 0) {
		device_printf(sc->dev,
		    "AP WPA2 requested but no PSK set "
		    "(sysctl dev.brcmfmac.<unit>.psk); cannot secure AP\n");
		return (EINVAL);
	}
	if (ssidlen == 0) {
		device_printf(sc->dev,
		    "AP WPA2: empty SSID, cannot derive PMK\n");
		return (EINVAL);
	}

	/*
	 * ROUND 49c: the r49b raw-32B PMK (byte-identical to Linux
	 * brcmf_set_pmk) STILL got firmware error 5 on hardware, and the SDIO
	 * BCDC path flattens every firmware BCME code to EIO before returning,
	 * so we never saw *why*.  Two changes here to diagnose + try a fix:
	 *
	 *  (1) DIAGNOSTIC: log sc->last_fwerr (the raw BCME code the firmware
	 *      returned, preserved by brcmf_sdpcm_ioctl before the EIO mapping)
	 *      after EVERY firmware call, unconditionally (not behind BRCMF_DBG).
	 *      We continue through all four calls even on error so a single run
	 *      surfaces the code for each of auth / wsec / wpa_auth / PMK.
	 *
	 *  (2) ALTERNATIVE ORDER: set wsec(AES) + wpa_auth(WPA2_PSK) BEFORE the
	 *      PMK.  r49/r49b used the Linux *AP* order (PMK first, then
	 *      configure_wpaie sets wsec/wpa_auth), but that gets rejected on
	 *      this 43456 firmware.  The Linux *STA* connect path -- which is
	 *      known-good on THIS board -- sets wsec (set_wsec_mode) and
	 *      wpa_auth (set_key_mgmt) FIRST, then the PMK (brcmf_set_pmk)
	 *      (cfg80211.c brcmf_cfg80211_connect: 2533/2540/2571).  The
	 *      firmware likely refuses a PMK until the cipher/AKM context
	 *      exists.  So mirror the STA order for the AP too.
	 */
	sc->last_fwerr = 0;

	/* Open-system 802.11 auth; WPA2 layers the 4-way handshake on top. */
	val = htole32(0);
	error = brcmf_fil_cmd_data_set(sc, 22 /* BRCMF_C_SET_AUTH */,
	    &val, sizeof(val));
	device_printf(sc->dev, "AP WPA2 [1/4] set auth=0: err=%d fwerr=%d\n",
	    error, sc->last_fwerr);

	/* (2) CCMP/AES pairwise+group cipher -- BEFORE the PMK now. */
	error = brcmf_fil_iovar_int_set(sc, "wsec", AES_ENABLED);
	device_printf(sc->dev,
	    "AP WPA2 [2/4] set wsec=AES(0x%x): err=%d fwerr=%d\n",
	    AES_ENABLED, error, sc->last_fwerr);

	/* (2) WPA2-PSK AKM -- BEFORE the PMK now.  Firmware builds the RSN IE. */
	error = brcmf_fil_iovar_int_set(sc, "wpa_auth", WPA2_AUTH_PSK);
	device_printf(sc->dev,
	    "AP WPA2 [3/4] set wpa_auth=WPA2_PSK(0x%x): err=%d fwerr=%d\n",
	    WPA2_AUTH_PSK, error, sc->last_fwerr);

	/* Decide passphrase vs raw 64-hex PMK. */
	is_hex = 0;
	if (sc->psk_len == 64) {
		is_hex = 1;
		for (i = 0; i < 64; i++) {
			if (brcmf_hexval(sc->psk[i]) < 0) {
				is_hex = 0;
				break;
			}
		}
	}

	/*
	 * ROUND 49b/c: always hand the firmware the RAW 32-byte PMK with
	 * key_len=32 and flags=0, exactly like Linux brcmf_set_pmk().
	 * Passphrases are expanded in-driver via PBKDF2-HMAC-SHA1.
	 */
	memset(&pmk, 0, sizeof(pmk));
	if (is_hex) {
		for (i = 0; i < 32; i++)
			pmk.key[i] = (uint8_t)(
			    (brcmf_hexval(sc->psk[2 * i]) << 4) |
			    brcmf_hexval(sc->psk[2 * i + 1]));
	} else {
		brcmf_pbkdf2_sha1((const uint8_t *)sc->psk, sc->psk_len,
		    ssid, ssidlen, 4096, pmk.key, BRCMF_WSEC_MAX_PSK_LEN);
	}
	pmk.key_len = htole16(BRCMF_WSEC_MAX_PSK_LEN);	/* 32 */
	pmk.flags = htole16(0);

	/*
	 * ROUND 49d: push the raw 32-byte PMK.  r49c proved the legacy numeric
	 * ioctl command BRCMF_C_SET_WSEC_PMK (268) returns fwerr=-23
	 * (BCME_UNSUPPORTED) on this CYW43456 firmware, while auth/wsec/wpa_auth
	 * all succeed.  So the firmware dropped the old command and exposes the
	 * PMK only through a different form.  Try the supported forms in order,
	 * accepting the first that returns err=0; log err+fwerr for each so one
	 * hardware run tells us exactly which the firmware accepts:
	 *
	 *  (A) "wsec_pmk" IOVAR (the named-string form; same wsec_pmk_le payload).
	 *      Many brcmfmac/Cypress firmwares expose the PMK only here.  AP is on
	 *      bsscfgidx 0, so the plain iovar == the bsscfg form byte-for-byte.
	 *  (B) enable the in-firmware supplicant/authenticator ("sup_wpa"=1, as the
	 *      Linux STA path does before brcmf_set_pmk, cfg80211.c:2563) then
	 *      retry the "wsec_pmk" iovar -- some firmwares gate the PMK on this.
	 *  (C) the legacy command (baseline; expected -23) so its code is on record.
	 */
	error = brcmf_fil_iovar_data_set(sc, "wsec_pmk", &pmk, sizeof(pmk));
	device_printf(sc->dev,
	    "AP WPA2 [4a] wsec_pmk iovar (raw 32B, %s): err=%d fwerr=%d\n",
	    is_hex ? "hex" : "pbkdf2(passphrase,ssid)", error, sc->last_fwerr);

	if (error != 0) {
		int e2;

		e2 = brcmf_fil_iovar_int_set(sc, "sup_wpa", 1);
		device_printf(sc->dev,
		    "AP WPA2 [4b-pre] sup_wpa=1: err=%d fwerr=%d\n",
		    e2, sc->last_fwerr);
		error = brcmf_fil_iovar_data_set(sc, "wsec_pmk", &pmk,
		    sizeof(pmk));
		device_printf(sc->dev,
		    "AP WPA2 [4b] wsec_pmk iovar after sup_wpa: err=%d fwerr=%d\n",
		    error, sc->last_fwerr);
	}

	if (error != 0) {
		error = brcmf_fil_cmd_data_set(sc, BRCMF_C_SET_WSEC_PMK,
		    &pmk, sizeof(pmk));
		device_printf(sc->dev,
		    "AP WPA2 [4c] legacy C_SET_WSEC_PMK cmd: err=%d fwerr=%d\n",
		    error, sc->last_fwerr);
	}

	explicit_bzero(&pmk, sizeof(pmk));

	if (error != 0) {
		device_printf(sc->dev,
		    "AP: all PMK-set forms failed (last err=%d fwerr=%d); "
		    "aborting\n", error, sc->last_fwerr);
		return (error);
	}

	device_printf(sc->dev,
	    "AP WPA2-PSK: wsec=AES(CCMP) wpa_auth=WPA2_PSK pmk=%s (raw 32B)\n",
	    is_hex ? "hex" : "pbkdf2(passphrase,ssid)");
	return (0);
}

int
brcmf_key_set(struct ieee80211vap *vap, const struct ieee80211_key *k)
{
	struct ieee80211com *ic = vap->iv_ic;
	struct brcmf_softc *sc = ic->ic_softc;
	struct ieee80211_node_table *nt = &ic->ic_sta;
	struct brcmf_wsec_key key;
	const uint8_t *macaddr;
	int error, com_locked, node_locked;

	BRCMF_DBG(sc, "key_set: idx=%u len=%u cipher=%u flags=0x%x\n",
	    k->wk_keyix, k->wk_keylen, k->wk_cipher->ic_cipher, k->wk_flags);

	memset(&key, 0, sizeof(key));
	key.index = htole32(k->wk_keyix);
	key.len = htole32(k->wk_keylen);

	if (k->wk_keylen > sizeof(key.data))
		return 0;
	memcpy(key.data, k->wk_key, k->wk_keylen);

	switch (k->wk_cipher->ic_cipher) {
	case IEEE80211_CIPHER_WEP:
		if (k->wk_keylen == 5)
			key.algo = htole32(CRYPTO_ALGO_WEP1);
		else
			key.algo = htole32(CRYPTO_ALGO_WEP128);
		break;
	case IEEE80211_CIPHER_TKIP:
		key.algo = htole32(CRYPTO_ALGO_TKIP);
		break;
	case IEEE80211_CIPHER_AES_CCM:
		key.algo = htole32(CRYPTO_ALGO_AES_CCM);
		break;
	default:
		return 0;
	}

	if (k->wk_flags & IEEE80211_KEY_GROUP) {
		/* BCM4350 note: ea must stay zeroed for group keys;
		 * broadcast ea returns BCME_UNSUPPORTED. */
		key.flags = htole32(BRCMF_PRIMARY_KEY);
	} else {
		macaddr = k->wk_macaddr;
		if (macaddr == NULL || IEEE80211_ADDR_EQ(macaddr, ieee80211broadcastaddr))
			macaddr = vap->iv_bss->ni_bssid;
		memcpy(key.ea, macaddr, 6);
	}

	com_locked = IEEE80211_IS_LOCKED(ic);
	node_locked = IEEE80211_NODE_IS_LOCKED(nt);
	if (node_locked)
		IEEE80211_NODE_UNLOCK(nt);
	if (com_locked)
		IEEE80211_UNLOCK(ic);
	error = brcmf_fil_iovar_data_set(sc, "wsec_key", &key, sizeof(key));
	if (com_locked)
		IEEE80211_LOCK(ic);
	if (node_locked)
		IEEE80211_NODE_LOCK(nt);
	if (error != 0)
		device_printf(sc->dev, "wsec_key set failed: %d\n", error);

	return (error == 0);
}

int
brcmf_key_delete(struct ieee80211vap *vap, const struct ieee80211_key *k)
{
	struct ieee80211com *ic = vap->iv_ic;
	struct brcmf_softc *sc = ic->ic_softc;
	struct ieee80211_node_table *nt = &ic->ic_sta;
	struct brcmf_wsec_key key;
	int error, com_locked, node_locked;

	/*
	 * Skip when the interface is down. brcmf_parent already cleared
	 * wsec/wpa_auth, so the firmware won't use stale keys. Issuing
	 * wsec_key ioctls while the firmware is tearing down a DFS channel
	 * association causes 2s timeouts that cascade into fw_dead.
	 */
	if (!sc->running)
		return 1;

	memset(&key, 0, sizeof(key));
	key.index = htole32(k->wk_keyix);
	key.algo = htole32(CRYPTO_ALGO_OFF);
	key.flags = htole32(BRCMF_PRIMARY_KEY);

	com_locked = IEEE80211_IS_LOCKED(ic);
	node_locked = IEEE80211_NODE_IS_LOCKED(nt);
	if (node_locked)
		IEEE80211_NODE_UNLOCK(nt);
	if (com_locked)
		IEEE80211_UNLOCK(ic);
	error = brcmf_fil_iovar_data_set(sc, "wsec_key", &key, sizeof(key));
	if (com_locked)
		IEEE80211_LOCK(ic);
	if (node_locked)
		IEEE80211_NODE_LOCK(nt);
	if (error != 0)
		device_printf(sc->dev, "wsec_key delete failed: %d\n", error);

	return 1;
}

static int
brcmf_sysctl_psk(SYSCTL_HANDLER_ARGS)
{
	struct brcmf_softc *sc = arg1;
	char buf[65];
	int error, len;

	/* Never expose the PSK on reads */
	memset(buf, 0, sizeof(buf));

	error = sysctl_handle_string(oidp, buf, sizeof(buf), req);
	if (error != 0 || req->newptr == NULL)
		return (error);

	len = strlen(buf);
	/*
	 * 8-63 chars = WPA passphrase; exactly 64 = raw hex PMK (round 49 AP
	 * fw-authenticator accepts either — see brcmf_ap_set_wpa2).
	 */
	if ((len < 8 || len > 63) && len != 64) {
		device_printf(sc->dev,
		    "PSK must be 8-63 chars (passphrase) or 64 (hex PMK)\n");
		return (EINVAL);
	}

	memcpy(sc->psk, buf, len);
	sc->psk[len] = '\0';
	sc->psk_len = len;

	return (0);
}

/*
 * ROUND 49e: firmware capability dump.  r49c/r49d proved that on this CYW43456
 * firmware the AP-mode PMK set (both the legacy C_SET_WSEC_PMK ioctl and the
 * "wsec_pmk" iovar) AND the firmware-supplicant enable ("sup_wpa"=1) all return
 * BCME_UNSUPPORTED (-23), while wsec/wpa_auth take fine.  That pattern says the
 * firmware has no in-chip WPA authenticator/supplicant -- the premise of the
 * Route A design.
 *
 * The definitive datum is the firmware "cap" iovar: a space-separated feature
 * list the firmware advertises.  Mainline brcmfmac reads it in
 * brcmf_feat_firmware_capabilities() and maps tokens to features; the
 * authenticator feature BRCMF_FEAT_FWAUTH is the token "idauth" (and the STA
 * supplicant BRCMF_FEAT_FWSUP is probed via the "sup_wpa" iovar).  Reading
 * `sysctl dev.brcmfmac.<u>.fwcap` triggers a live "cap" read (firmware must be
 * brought up first) and logs the full string plus a verdict on the
 * authenticator-related tokens, and a wsec/wpa_auth readback.  Read-only.
 */
static int
brcmf_sysctl_fwcap(SYSCTL_HANDLER_ARGS)
{
	struct brcmf_softc *sc = arg1;
	char caps[768];
	uint32_t wsec, wpa_auth;
	int error, e2;

	error = brcmf_fil_iovar_get_buf(sc, "cap", caps, sizeof(caps));
	if (error != 0) {
		snprintf(caps, sizeof(caps),
		    "<cap read failed err=%d fwerr=%d; is firmware up? "
		    "sysctl dev.brcmfmac.%d.bringup=1>",
		    error, sc->last_fwerr, device_get_unit(sc->dev));
		device_printf(sc->dev, "fwcap: %s\n", caps);
		return (sysctl_handle_string(oidp, caps, sizeof(caps), req));
	}

	device_printf(sc->dev, "fwcap: [ %s]\n", caps);
	device_printf(sc->dev,
	    "fwcap verdict: idauth(fw-authenticator/AP-PSK)=%s  sae=%s  "
	    "mbss=%s  mchan=%s\n",
	    strstr(caps, "idauth") ? "YES" : "no",
	    strstr(caps, "sae") ? "YES" : "no",
	    strstr(caps, "mbss") ? "YES" : "no",
	    strstr(caps, "mchan") ? "YES" : "no");

	wsec = wpa_auth = 0xffffffff;
	e2 = brcmf_fil_iovar_int_get(sc, "wsec", &wsec);
	device_printf(sc->dev, "fwcap: wsec readback=0x%x (err=%d)\n", wsec, e2);
	e2 = brcmf_fil_iovar_int_get(sc, "wpa_auth", &wpa_auth);
	device_printf(sc->dev, "fwcap: wpa_auth readback=0x%x (err=%d)\n",
	    wpa_auth, e2);

	return (sysctl_handle_string(oidp, caps, sizeof(caps), req));
}

static int
brcmf_sysctl_country(SYSCTL_HANDLER_ARGS)
{
	struct brcmf_softc *sc = arg1;
	char buf[4];
	int error;

	strlcpy(buf, sc->country, sizeof(buf));

	error = sysctl_handle_string(oidp, buf, sizeof(buf), req);
	if (error != 0 || req->newptr == NULL)
		return (error);

	if (strlen(buf) != 2)
		return (EINVAL);

	/* Update sc->country; firmware is set on next ifconfig up */
	strlcpy(sc->country, buf, sizeof(sc->country));

	return (0);
}

static int
brcmf_sysctl_pm(SYSCTL_HANDLER_ARGS)
{
	struct brcmf_softc *sc = arg1;
	uint32_t val;
	int error;

	val = 0;
	brcmf_fil_cmd_data_get(sc, 85 /* BRCMF_C_GET_PM */,
	    &val, sizeof(val));

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);

	if (val > 2)
		return (EINVAL);

	return brcmf_fil_cmd_data_set(sc, 86 /* BRCMF_C_SET_PM */,
	    &val, sizeof(val));
}

void
brcmf_security_sysctl_init(struct brcmf_softc *sc)
{
	struct sysctl_oid *oid;

	oid = device_get_sysctl_tree(sc->dev);
	if (oid == NULL)
		return;

	SYSCTL_ADD_PROC(&sc->sysctl_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "psk", CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    brcmf_sysctl_psk, "A", "WPA PSK passphrase");

	SYSCTL_ADD_PROC(&sc->sysctl_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "pm", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    brcmf_sysctl_pm, "I", "Power management (0=off, 1=PM1, 2=PM2)");

	/*
	 * ROUND 49e: read-only diagnostic -- dumps the firmware "cap" string +
	 * an idauth/sae/mbss verdict and a wsec/wpa_auth readback (firmware must
	 * be brought up first).  Decides whether the fw authenticator (Route A)
	 * exists on this build.
	 */
	SYSCTL_ADD_PROC(&sc->sysctl_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "fwcap", CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    brcmf_sysctl_fwcap, "A",
	    "Firmware capability string + authenticator verdict (diagnostic)");

	SYSCTL_ADD_PROC(&sc->sysctl_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "country", CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    brcmf_sysctl_country, "A", "Regulatory country code (2 chars)");

	SYSCTL_ADD_INT(&sc->sysctl_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "debug", CTLFLAG_RW, &sc->debug, 0,
	    "Debug verbosity (0=off, 1=events, 2=verbose)");

	SYSCTL_ADD_UINT(&sc->sysctl_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "tx_count", CTLFLAG_RD, &sc->tx_count, 0,
	    "TX packets submitted to flowring");
	SYSCTL_ADD_UINT(&sc->sysctl_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "tx_drops", CTLFLAG_RD, &sc->tx_drops, 0,
	    "TX packets dropped (ring full)");
	SYSCTL_ADD_UINT(&sc->sysctl_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "tx_complete", CTLFLAG_RD, &sc->tx_complete_count, 0,
	    "TX completions processed");
	SYSCTL_ADD_UINT(&sc->sysctl_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "isr_filter", CTLFLAG_RD, &sc->isr_filter_count, 0,
	    "ISR filter invocations");
	SYSCTL_ADD_UINT(&sc->sysctl_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "isr_task", CTLFLAG_RD, &sc->isr_task_count, 0,
	    "ISR task invocations");
	SYSCTL_ADD_UINT(&sc->sysctl_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "rx_complete", CTLFLAG_RD, &sc->rx_complete_count, 0,
	    "RX completions processed");
	SYSCTL_ADD_UINT(&sc->sysctl_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "rx_deliver_fail", CTLFLAG_RD, &sc->rx_deliver_fail, 0,
	    "RX mbuf allocation failures");
	SYSCTL_ADD_UINT(&sc->sysctl_ctx, SYSCTL_CHILDREN(oid), OID_AUTO,
	    "rx_repost_fail", CTLFLAG_RD, &sc->rx_repost_fail, 0,
	    "RX buffer repost failures (H2D rxpost ring full)");
}
