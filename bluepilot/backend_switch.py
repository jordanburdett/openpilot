"""BluePilot: connect backend selection (comma / Konik / offline).

BPConnectBackend (int) selects where the device sends routes and telemetry:

  0 = Comma Connect  — stock openpilot defaults (API_HOST / ATHENA_HOST unset)
  1 = Konik Stable   — api.konik.ai / athena.konik.ai (stable.konik.ai pairing)
  2 = Offline Mode   — bogus hosts so uploads and athena can never succeed

launch_env.sh exports API_HOST / ATHENA_HOST from this param. openpilot already
reads those env vars everywhere it talks to the backend (common/api/comma_connect.py,
system/athena/athenad.py, registration, uploader), so no other host wiring is needed.

Dongle ID handling (register() calls reconcile_backend on every manager start):

- comma connect: factory dongle ID, recoverable from /persist/comma/dongle_id (devices
  built since 2/28/24) or by re-registering (comma's backend recognizes the persistent
  RSA keypair).
- Konik (connect-killer): deterministic dongle ID, sha256(imei+imei2+serial+public_key)[:16],
  assigned by its /v2/pilotauth on first registration. Registration is idempotent, so the
  same device always maps to the same Konik ID.
- Offline: keep the current DongleId (or UnregisteredDevice); never hit the network.

Each real backend's ID is cached the first time it is seen, so after one registration
per backend, switching comma <-> Konik is a pure param swap on reboot.
"""

from openpilot.common.swaglog import cloudlog

UNREGISTERED_DONGLE_ID = "UnregisteredDevice"

BACKEND_COMMA = "comma"
BACKEND_KONIK = "konik"
BACKEND_OFFLINE = "offline"

# Index order matches BPConnectBackend and the settings UI option lists.
BACKENDS = (BACKEND_COMMA, BACKEND_KONIK, BACKEND_OFFLINE)

BACKEND_LABELS = {
  BACKEND_COMMA: "Comma Connect",
  BACKEND_KONIK: "Konik Stable",
  BACKEND_OFFLINE: "Offline Mode",
}

PAIRING_HOST = {
  BACKEND_COMMA: "connect.comma.ai",
  BACKEND_KONIK: "stable.konik.ai",
  BACKEND_OFFLINE: "pairing.invalid",  # RFC 2606 .invalid never resolves
}

CACHE_PARAM = {
  BACKEND_COMMA: "BPDongleIdComma",
  BACKEND_KONIK: "BPDongleIdKonik",
}

PARAM_KEY = "BPConnectBackend"
LEGACY_KONIK_PARAM = "BPUseKonik"


def backend_index(backend: str) -> int:
  try:
    return BACKENDS.index(backend)
  except ValueError:
    return 0


def backend_label(backend: str) -> str:
  return BACKEND_LABELS.get(backend, BACKEND_LABELS[BACKEND_COMMA])


def pairing_host(backend: str) -> str:
  return PAIRING_HOST.get(backend, PAIRING_HOST[BACKEND_COMMA])


def _maybe_migrate_legacy_konik(params) -> None:
  """One-shot: BPUseKonik=1 -> BPConnectBackend=1, then clear the legacy flag."""
  try:
    if not params.get_bool(LEGACY_KONIK_PARAM):
      return
    current = params.get(PARAM_KEY)
    try:
      idx = int(current) if current not in (None, "") else 0
    except (TypeError, ValueError):
      idx = 0
    if idx == 0:
      params.put(PARAM_KEY, 1)
      cloudlog.event("bp_backend_migrate", from_param=LEGACY_KONIK_PARAM, to=BACKEND_KONIK)
    params.put_bool(LEGACY_KONIK_PARAM, False)
  except Exception:
    pass


def get_connect_backend(params) -> str:
  """Return the selected backend name: comma, konik, or offline."""
  _maybe_migrate_legacy_konik(params)
  try:
    raw = params.get(PARAM_KEY)
    idx = int(raw) if raw not in (None, "") else 0
  except (TypeError, ValueError, Exception):
    idx = 0
  if 0 <= idx < len(BACKENDS):
    return BACKENDS[idx]
  return BACKEND_COMMA


def reconcile_backend(params) -> str:
  """Align DongleId with the backend selected by BPConnectBackend.

  Called at the top of register(). Returns the active backend name. Callers skip
  the /persist comma dongle ID restore when the backend is not comma (that restore
  would short-circuit Konik registration on devices built since 2/28/24). Offline
  never attempts network registration.
  """
  try:
    target = get_connect_backend(params)
    active = params.get("BPActiveBackend") or BACKEND_COMMA
    dongle_id = params.get("DongleId")
    registered = dongle_id is not None and dongle_id != UNREGISTERED_DONGLE_ID

    # BluePilot: diagnostic -- runs on every boot regardless of branch below, so field logs
    # always show what this function saw and decided, even on the common comma/comma no-op path.
    cloudlog.event("bp_backend_reconcile", target=target, active=active, registered=registered,
                   dongle_id=dongle_id)

    if target == active:
      # Keep the cache fresh for real backends after a successful registration.
      if target in CACHE_PARAM and registered and params.get(CACHE_PARAM[target]) != dongle_id:
        params.put(CACHE_PARAM[target], dongle_id)
      return target

    # Backend changed: stash the outgoing backend's ID when it has a cache slot.
    if registered and active in CACHE_PARAM:
      params.put(CACHE_PARAM[active], dongle_id)

    if target == BACKEND_OFFLINE:
      # Stay offline with whatever DongleId we already have; no registration.
      params.put("BPActiveBackend", target)
      cloudlog.event("bp_backend_switch", backend=target, dongle_id=dongle_id, restored=False)
      return target

    cached = params.get(CACHE_PARAM[target])
    if cached:
      params.put("DongleId", cached, block=True)
      cloudlog.event("bp_backend_switch", backend=target, dongle_id=cached, restored=True)
    else:
      params.remove("DongleId")
      cloudlog.event("bp_backend_switch", backend=target, restored=False)

    params.put("BPActiveBackend", target)
    return target
  except Exception:
    # Never block registration: fall back to stock comma behavior on any failure
    # (e.g. params not yet defined in a dev environment).
    cloudlog.exception("bp_backend_switch failed, using comma connect behavior")
    return BACKEND_COMMA
