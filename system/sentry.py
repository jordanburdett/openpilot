"""Install exception handler for process crash."""
import logging  # BluePilot: LoggingIntegration below
import os
import traceback
from datetime import datetime
from enum import Enum
from importlib.metadata import PackageNotFoundError, version as pkg_version  # BluePilot: enable_logs version guard

import sentry_sdk
from sentry_sdk.integrations.threading import ThreadingIntegration
from sentry_sdk.integrations.logging import LoggingIntegration  # BluePilot: max-coverage log capture (see init())

from openpilot.common.params import Params
from openpilot.system.athena.registration import UNREGISTERED_DONGLE_ID
from openpilot.system.hardware import HARDWARE
from openpilot.system.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog
from openpilot.system.version import get_build_metadata, get_version

from openpilot.sunnypilot.sunnylink.api import UNREGISTERED_SUNNYLINK_DONGLE_ID

CRASHES_DIR = Paths.crash_log_root()


class SentryProject(Enum):
  # BluePilot: report to our OWN self-hosted GlitchTip (Sentry-API compatible), which we fully control
  # — replaces the coworker-owned Sentry we could neither see nor configure.
  # Public HTTPS via Cloudflare Tunnel (glitchtip.bluepilot.dev) — reachable by the whole fleet,
  # not just devices on the IoT VLAN. GlitchTip VM's LAN/IoT addresses (10.88.1.236 / 10.88.20.236)
  # remain valid for on-bench testing if the tunnel is ever down.
  # Prior DSNs for reference:  sunnypilot upstream  = o1138119.ingest.us.sentry.io/4508660076052480
  #                            BluePilot Sentry(6.0) = o4510926103379968.ingest.us.sentry.io/4510926105739264
  # python project
  SELFDRIVE = "https://06436d643ff64a6eadc42b3691de2198@glitchtip.bluepilot.dev/1"
  # native project
  SELFDRIVE_NATIVE = SELFDRIVE


# BluePilot: known-noisy log lines that log at ERROR severity for routine/expected conditions, not
# bugs — e.g. isotp_parallel_query logs a bad ECU response at .error() on every fingerprint scan where
# an ECU doesn't answer a diagnostic query cleanly (normal on real vehicles), and NNLC logs "no model
# for this car" at .error() for the majority of supported vehicles that simply lack a custom model.
# Filtered here (client-side, so they never leave the device) instead of at each call site, so this
# stays one central, reviewable/extendable list rather than scattered severity fixes.
# A filtered log line is still recorded as a breadcrumb (LoggingIntegration adds those independently
# of before_send), so it still shows up as trailing context on a real crash from the same process —
# only suppressed from becoming its own standalone issue.
_NOISY_LOG_SUBSTRINGS = (
  "iso-tp query bad response",
  "iso-tp query response pending",
  "car doesn't match any Neural Network model",
  # opendbc/car/car_helpers.py — routine, every boot; capture_fingerprint() above already tags
  # carFingerprint/carName on this process's scope, which is the reliable way to get fingerprint
  # context on a real crash (a tag doesn't decay out of the breadcrumb ring buffer like this would).
  "'event': 'fingerprinted'",
  # opendbc/car/fw_versions.py — fuzzy-match fingerprint variant of the same routine event.
  "using fuzzy match",
)


def _before_send(event: dict, hint: dict) -> dict | None:
  formatted = event.get("logentry", {}).get("formatted", "")
  if any(s in formatted for s in _NOISY_LOG_SUBSTRINGS):
    return None
  return event
# End BluePilot


def report_tombstone(fn: str, message: str, contents: str) -> None:
  cloudlog.error({'tombstone': message})

  with sentry_sdk.configure_scope() as scope:
    set_user()
    scope.set_extra("tombstone_fn", fn)
    scope.set_extra("tombstone", contents)
    sentry_sdk.capture_message(message=message)
    sentry_sdk.flush()


def capture_exception(*args, **kwargs) -> None:
  cloudlog.error("crash", exc_info=kwargs.get('exc_info', 1))

  try:
    save_exception(traceback.format_exc())

    set_user()
    sentry_sdk.capture_exception(*args, **kwargs)
    sentry_sdk.flush()  # https://github.com/getsentry/sentry-python/issues/291
  except Exception:
    cloudlog.exception("sentry exception")


def save_exception(content: str) -> None:
  try:
    if not os.path.exists(CRASHES_DIR):
      os.makedirs(CRASHES_DIR)

    files = [
      os.path.join(CRASHES_DIR, datetime.now().strftime("%Y-%m-%d--%H-%M-%S.log")),
      os.path.join(CRASHES_DIR, "error.log")
    ]

    for fn in files:
      with open(fn, 'w') as f:
        if fn == "error.log":
          lines = content.splitlines()[-3:]
          f.write("\n".join(lines))
        else:
          f.write(content)

    cloudlog.error(f"logged crash to {files}")
  except Exception:
    cloudlog.exception("error when attempting to save exception")


def capture_fingerprint_mock() -> None:
  try:
    set_user()
    message = "car doesn't match any fingerprints"
    sentry_sdk.capture_message(message=message, level="error")
    sentry_sdk.flush()
  except Exception as e:
    cloudlog.exception(f"sentry fingerprint MOCK exception: {e}")


def capture_fingerprint(candidate: str, car_name: str) -> None:
  # BluePilot: tag-only, no capture_message() — a successful fingerprint is not itself a crash and
  # shouldn't create its own issue. The tags persist on this process's scope, so they're attached
  # automatically to any real crash this process reports later.
  try:
    set_user()
    sentry_sdk.set_tag("carFingerprint", candidate)
    sentry_sdk.set_tag("carName", car_name)
  except Exception as e:
    cloudlog.exception(f"sentry fingerprint exception: {e}")
  # End BluePilot


def set_tag(key: str, value: str) -> None:
  sentry_sdk.set_tag(key, value)


def set_user() -> None:
  dongle_id, git_username, _ = get_properties()
  sentry_sdk.set_user({"id": dongle_id, "name": git_username})


def get_properties() -> tuple[str, str, str]:
  params = Params()
  hardware_serial: str = params.get("HardwareSerial") or ""
  git_username: str = params.get("GithubUsername") or ""
  dongle_id: str = params.get("DongleId") or f"{UNREGISTERED_DONGLE_ID}-{hardware_serial}"
  sunnylink_dongle_id: str = params.get("SunnylinkDongleId") or UNREGISTERED_SUNNYLINK_DONGLE_ID

  return dongle_id, git_username, sunnylink_dongle_id


# BluePilot: enable_logs exists only on sentry-sdk >= ~2.35; older builds (incl. some AGNOS venvs)
# raise TypeError on sentry_sdk.init(enable_logs=...). Gate it on the installed version.
def _sentry_sdk_supports_enable_logs() -> bool:
  try:
    v = pkg_version("sentry-sdk")
  except PackageNotFoundError:
    return False
  parts: list[int] = []
  for segment in v.split(".")[:3]:
    digits = "".join(ch for ch in segment if ch.isdigit())
    if not digits:
      break
    parts.append(min(int(digits), 9999))
    if len(parts) == 3:
      break
  while len(parts) < 3:
    parts.append(0)
  return tuple(parts) >= (2, 35, 0)
# End BluePilot


def init(project: SentryProject) -> bool:
  build_metadata = get_build_metadata()

  env = build_metadata.channel_type
  dongle_id, git_username, sunnylink_dongle_id = get_properties()

  integrations = []
  if project == SentryProject.SELFDRIVE:
    integrations.append(ThreadingIntegration(propagate_hub=True))

  # BluePilot: maximum coverage. cloudlog is a SwagLogger(logging.Logger) with propagate=True, so a
  # LoggingIntegration on the root logger captures it: ERROR+ become Sentry events (this catches the
  # daemon errors that are caught-and-logged rather than crashing the process — the "some things, not
  # all" gap), INFO+ become breadcrumbs for context. Lower event_level to WARNING for even more volume.
  integrations.append(LoggingIntegration(level=logging.INFO, event_level=logging.ERROR))

  init_kw = dict(
    default_integrations=False,
    release=get_version(),
    integrations=integrations,
    traces_sample_rate=1.0,
    max_value_length=8192,
    environment=env,
    before_send=_before_send,
  )
  # enable_logs = structured Sentry Logs (separate from the LoggingIntegration events above).
  if _sentry_sdk_supports_enable_logs():
    init_kw["enable_logs"] = True

  sentry_sdk.init(project.value, **init_kw)
  # End BluePilot

  sentry_sdk.set_user({"id": dongle_id, "name": git_username})
  sentry_sdk.set_tag("dirty", build_metadata.openpilot.is_dirty)
  sentry_sdk.set_tag("origin", build_metadata.openpilot.git_origin)
  sentry_sdk.set_tag("branch", build_metadata.channel)
  sentry_sdk.set_tag("commit", build_metadata.openpilot.git_commit)
  sentry_sdk.set_tag("device", HARDWARE.get_device_type())
  # BluePilot: comma dongle ID as its own tag (not just sentry_sdk.set_user above) — Discord/webhook
  # notifications only surface issue *tags* (apps.alerts.webhooks.gather_issue_tags), never the
  # Sentry "user" object, so without this the dongle ID never appears outside the GlitchTip UI.
  sentry_sdk.set_tag("dongle_id", dongle_id)
  sentry_sdk.set_tag("sunnylink_dongle_id", sunnylink_dongle_id)
  # End BluePilot

  return True
