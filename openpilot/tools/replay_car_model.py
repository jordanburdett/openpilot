#!/usr/bin/env python3
"""Replay a real drive through the car stack, offline, before you drive again.

This runs opendbc's own `TestCarModelBase` suite -- the same one CI uses against comma's
hosted routes -- but pointed at logs you already have: a local rlog, or segments pulled
straight off the device over SSH.

What it actually exercises, against real CAN from a real drive:

  * CarInterface / CarState parsing            (test_car_interface)
  * RadarInterface                             (test_radar_interface)
  * CarController message generation           (test_panda_safety_tx_cases)
  * the compiled panda safety, cross-checked   (test_panda_safety_carstate, _rx_checks)
    against CarState frame by frame

Why this and not tools/sim: the MetaDrive simulator never produces Ford CAN, so it cannot
reach brand car/radar code at all. Replay can.

    # last drive still on the device
    ./openpilot/tools/replay_car_model.py --device

    # a specific route already on the device
    ./openpilot/tools/replay_car_model.py --device --route 00000001--b3f614d4d5

    # logs already on disk (repeatable, use this in a pre-flight check)
    ./openpilot/tools/replay_car_model.py --logs /path/to/seg0.zst /path/to/seg1.zst

IMPORTANT -- replay only catches what the log contains. A code path guarded by a condition
that never occurred in the log (a detected lead, a specific button, a fault) is not covered
just because the run was green. Prefer a log with the driving you care about, and pass
several segments; they are concatenated. Under ~5000 CAN messages total the suite refuses
to run, which is why single short segments must be combined.
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

DEFAULT_HOST = "comma3x"
REALDATA = "/data/media/0/realdata"
MIN_CAN_MSGS = 5000  # TestCarModelBase asserts len(can_msgs) > 50 / DT_CTRL


def ssh(host: str, cmd: str) -> str:
  out = subprocess.run(["ssh", "-o", "ConnectTimeout=20", host, cmd],
                       capture_output=True, text=True, check=True)
  return out.stdout.strip()


def pick_route(host: str) -> str:
  """Newest route on the device that has more than one segment of real driving."""
  listing = ssh(host, f"ls -1 {REALDATA} 2>/dev/null")
  routes: dict[str, int] = {}
  for name in listing.splitlines():
    if "--" not in name or name in ("boot", "crash"):
      continue
    route, _, seg = name.rpartition("--")
    if seg.isdigit():
      routes[route] = routes.get(route, 0) + 1
  if not routes:
    raise SystemExit(f"no routes found under {REALDATA} on {host}")
  # newest by directory mtime, not by name: a boot before NTP sync stamps stale dates
  newest = ssh(host, f"ls -1dt {REALDATA}/*--* 2>/dev/null | head -40")
  for path in newest.splitlines():
    route, _, seg = Path(path).name.rpartition("--")
    if seg.isdigit() and route in routes:
      return route
  return next(iter(routes))


def fetch_route(host: str, route: str, dest: Path) -> list[Path]:
  names = ssh(host, f"ls -1d {REALDATA}/{route}--* 2>/dev/null")
  segs = sorted((Path(p) for p in names.splitlines()),
                key=lambda p: int(p.name.rpartition("--")[2]))
  if not segs:
    raise SystemExit(f"route {route} not found on {host}")
  local = []
  for seg in segs:
    out = dest / f"{seg.name}.zst"
    print(f"  fetching {seg.name}/rlog.zst", flush=True)
    r = subprocess.run(["scp", "-q", f"{host}:{seg}/rlog.zst", str(out)])
    if r.returncode == 0 and out.exists():
      local.append(out)
  if not local:
    raise SystemExit(f"no rlogs could be fetched for {route}")
  return local


def merge(logs: list[Path], dest: Path) -> Path:
  """Concatenate rlogs into one uncompressed capnp stream.

  LogReader sniffs zstd by magic bytes, so a plain .rlog is read as-is. Segments have to be
  combined because the suite requires more CAN messages than one short segment holds.
  """
  import zstandard
  merged = dest / "merged.rlog"
  with open(merged, "wb") as out:
    for log in logs:
      raw = log.read_bytes()
      if raw[:4] == b"\x28\xB5\x2F\xFD":
        raw = zstandard.ZstdDecompressor().stream_reader(raw).read()
      out.write(raw)
  return merged


def build_case(log: Path, platform: str | None):
  from opendbc.car.logreader import LogReader
  from opendbc.car.tests.routes import CarTestRoute
  from opendbc.car.tests.test_models import TestCarModelBase

  class TestReplayedRoute(TestCarModelBase):
    # platform=None lets setUpClass fingerprint from the log's own carParams
    globals()["_platform"] = platform
    test_route = CarTestRoute("local", platform or "UNKNOWN")

    @classmethod
    def get_testing_data(cls):
      return cls.get_testing_data_from_logreader(
        LogReader(str(log), only_union_types=True, sort_by_time=True))

  TestReplayedRoute.platform = platform
  return TestReplayedRoute


def main() -> int:
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  src = p.add_mutually_exclusive_group(required=True)
  src.add_argument("--device", action="store_true", help=f"pull logs over SSH (default host {DEFAULT_HOST})")
  src.add_argument("--logs", nargs="+", type=Path, help="local rlog files, concatenated in order")
  p.add_argument("--host", default=DEFAULT_HOST)
  p.add_argument("--route", help="route name on the device; default is the newest")
  p.add_argument("--platform", help="force a platform instead of reading it from the log")
  p.add_argument("--keep", action="store_true", help="keep the fetched/merged logs")
  args = p.parse_args()

  work = Path(tempfile.mkdtemp(prefix="replay_car_model_"))
  try:
    if args.device:
      route = args.route or pick_route(args.host)
      print(f"host   : {args.host}\nroute  : {route}")
      logs = fetch_route(args.host, route, work)
    else:
      logs = list(args.logs)
      for log in logs:
        if not log.exists():
          raise SystemExit(f"no such log: {log}")

    log = logs[0] if len(logs) == 1 and logs[0].suffix == ".rlog" else merge(logs, work)
    print(f"log    : {log} ({log.stat().st_size / 1e6:.1f} MB from {len(logs)} segment(s))\n")

    case = build_case(log, args.platform)
    suite = unittest.TestLoader().loadTestsFromTestCase(case)
    result = unittest.TextTestRunner(verbosity=2).run(suite)

    print(f"\nran={result.testsRun} failures={len(result.failures)} errors={len(result.errors)} skipped={len(result.skipped)}")
    if not (result.failures or result.errors):
      print("\nPASS -- but only for code paths this drive actually reached.")
    return 1 if (result.failures or result.errors) else 0
  finally:
    if args.keep:
      print(f"\nlogs kept in {work}")
    else:
      shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
  os.environ.setdefault("PYTHONUNBUFFERED", "1")
  sys.exit(main())
