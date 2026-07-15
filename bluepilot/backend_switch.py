"""BluePilot: comma connect <-> Konik backend switching.

The "Use Konik instead of comma connect" toggle (BPUseKonik) redirects API_HOST /
ATHENA_HOST at launch (see launch_env.sh), but the device also has to identify with a
dongle ID the selected backend actually knows:

- comma connect: factory dongle ID, recoverable from /persist/comma/dongle_id (devices
  built since 2/28/24) or by re-registering (comma's backend recognizes the persistent
  RSA keypair).
- Konik (connect-killer): deterministic dongle ID, sha256(imei+imei2+serial+public_key)[:16],
  assigned by its /v2/pilotauth on first registration. Registration is idempotent, so the
  same device always maps to the same Konik ID.

This module reconciles the DongleId param with the selected backend at registration time
(register() in system/athena/registration.py runs on every manager start, before anything
uses the dongle ID). Each backend's ID is cached in a param the first time it is seen, so
after one registration per backend, switching is a pure param swap on reboot — reversible
in both directions with no re-pairing.
"""

from openpilot.common.swaglog import cloudlog

UNREGISTERED_DONGLE_ID = "UnregisteredDevice"

BACKEND_COMMA = "comma"
BACKEND_KONIK = "konik"

CACHE_PARAM = {
  BACKEND_COMMA: "BPDongleIdComma",
  BACKEND_KONIK: "BPDongleIdKonik",
}


def reconcile_backend(params) -> bool:
  """Align the DongleId param with the backend selected by BPUseKonik.

  Called at the top of register(). Returns True when Konik is the active backend
  (callers use this to skip the /persist comma dongle ID restore, which would
  otherwise short-circuit Konik registration on devices built since 2/28/24).
  """
  try:
    use_konik = params.get_bool("BPUseKonik")
    target = BACKEND_KONIK if use_konik else BACKEND_COMMA
    active = params.get("BPActiveBackend") or BACKEND_COMMA
    dongle_id = params.get("DongleId")
    registered = dongle_id is not None and dongle_id != UNREGISTERED_DONGLE_ID

    if target == active:
      # Keep the cache fresh: after a successful registration under this backend,
      # remember the ID so future switches away and back need no network.
      if registered and params.get(CACHE_PARAM[target]) != dongle_id:
        params.put(CACHE_PARAM[target], dongle_id)
      return use_konik

    # Backend changed: stash the outgoing backend's ID, then restore the target's
    # cached ID, or clear DongleId so register() re-registers against the new API_HOST.
    if registered:
      params.put(CACHE_PARAM[active], dongle_id)

    cached = params.get(CACHE_PARAM[target])
    if cached:
      params.put("DongleId", cached, block=True)
      cloudlog.event("bp_backend_switch", backend=target, dongle_id=cached, restored=True)
    else:
      params.remove("DongleId")
      cloudlog.event("bp_backend_switch", backend=target, restored=False)

    params.put("BPActiveBackend", target)
    return use_konik
  except Exception:
    # Never block registration: fall back to stock comma behavior on any failure
    # (e.g. params not yet defined in a dev environment).
    cloudlog.exception("bp_backend_switch failed, using comma connect behavior")
    return False
