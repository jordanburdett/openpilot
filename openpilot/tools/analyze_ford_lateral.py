#!/usr/bin/env python3
"""Diagnose Ford angle-mode cornering from a drive: what limited the steering, and by how much.

Answers one question: when the car runs wide at a corner, was it

  (a) the +-CURVATURE_ERROR deviation clip -- kappa_cmd is clipped to the *measured* curvature
      +- 0.002 (1/m) above 9 m/s, which caps how fast curvature can build at CURVATURE_ERROR/tau.
      If the car's response time tau exceeds ~0.18 s, that ceiling falls below what an ordinary
      90-degree turn needs and the command physically cannot keep up -> understeer, runs wide.
      Look for: deviation-clip % high, and commanded curvature pinned just above measured.

  (b) phase lead from the predicted-curvature blend -- requested = predicted(t+T)*b + desired*(1-b)
      with b = 0.50 and T = 0.15..0.30 s. On entry the model's future curvature is higher, so the
      command leads; on exit it is lower, so the command unwinds early.
      Look for: lead_ms consistently positive and large, with a low deviation-clip %.

Both can be present. The deviation-clip percentage is the discriminator, and it is already in
the logs as controllerStateBP.curvatureDeviationLimited -- nothing new has to be flashed.

    ./openpilot/tools/analyze_ford_lateral.py --device
    ./openpilot/tools/analyze_ford_lateral.py --logs seg0.zst seg1.zst
"""

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import capnp
import numpy as np
import zstandard

from openpilot.cereal import log

DEFAULT_HOST = "comma3x"
REALDATA = "/data/media/0/realdata"
CURVATURE_ERROR = 0.002   # opendbc/car/ford/values.py CarControllerParams
CORNER_KAPPA = 0.004      # 1/m -- below this it is effectively straight (250 m radius)
MIN_CORNER_S = 1.5        # ignore blips shorter than this


def ssh(host, cmd):
  return subprocess.run(["ssh", "-o", "ConnectTimeout=20", host, cmd],
                        capture_output=True, text=True, check=True).stdout.strip()


def fetch_newest(host, dest, route=None):
  if route is None:
    listing = ssh(host, f"ls -1dt {REALDATA}/*--* 2>/dev/null | head -40")
    for path in listing.splitlines():
      name = Path(path).name
      base, _, seg = name.rpartition("--")
      if seg.isdigit():
        route = base
        break
  if route is None:
    raise SystemExit("no routes found on device")
  print(f"route  : {route}")
  names = ssh(host, f"ls -1d {REALDATA}/{route}--* 2>/dev/null")
  out = []
  for seg in sorted((Path(p) for p in names.splitlines()),
                    key=lambda p: int(p.name.rpartition("--")[2])):
    target = dest / f"{seg.name}.zst"
    if subprocess.run(["scp", "-q", f"{host}:{seg}/rlog.zst", str(target)]).returncode == 0:
      out.append(target)
      print(f"  fetched {seg.name}")
  return out


def read_signals(paths):
  """Pull the lateral signals onto one timeline, forward-filling between message rates."""
  rows = []
  cur = {"t": 0.0, "v": 0.0, "desired": 0.0, "meas": 0.0, "path_angle": 0.0, "lat_active": False,
         "dev_clip": False, "rate_lim": False, "mode": None, "model_kappa": 0.0, "steer_angle": 0.0}
  for path in paths:
    raw = zstandard.ZstdDecompressor().stream_reader(open(path, "rb")).read()
    try:
      for ev in log.Event.read_multiple_bytes(raw):
        w = ev.which()
        t = ev.logMonoTime / 1e9
        if w == "carState":
          cur["v"] = ev.carState.vEgo
          cur["steer_angle"] = ev.carState.steeringAngleDeg
        elif w == "controlsState":
          cur["meas"] = ev.controlsState.curvature
        elif w == "carControl":
          cc = ev.carControl
          cur["desired"] = cc.actuators.curvature
          cur["lat_active"] = bool(cc.latActive)
        elif w == "modelV2":
          try:
            cur["model_kappa"] = ev.modelV2.action.desiredCurvature
          except Exception:
            pass
        elif w == "controllerStateBP":
          c = ev.controllerStateBP
          cur["dev_clip"] = bool(c.curvatureDeviationLimited)
          cur["rate_lim"] = bool(c.angleRateLimited)
          try:
            cur["mode"] = str(c.activeLateralMode)
          except Exception:
            pass
          cur["t"] = t
          rows.append(dict(cur))
    except capnp.KjException:
      pass  # truncated tail
  return rows


def find_corners(rows):
  kap = np.array([abs(r["desired"]) for r in rows])
  act = np.array([r["lat_active"] for r in rows])
  t = np.array([r["t"] for r in rows])
  inside = (kap > CORNER_KAPPA) & act
  corners, start = [], None
  for i, flag in enumerate(inside):
    if flag and start is None:
      start = i
    elif not flag and start is not None:
      if t[i - 1] - t[start] >= MIN_CORNER_S:
        corners.append((start, i))
      start = None
  if start is not None and t[-1] - t[start] >= MIN_CORNER_S:
    corners.append((start, len(rows)))
  return corners


def estimate_tau(rows, lo, hi):
  """First-order fit: kappa_meas' = (kappa_cmd - kappa_meas)/tau. tau from least squares."""
  seg = rows[lo:hi]
  if len(seg) < 10:
    return float("nan")
  t = np.array([r["t"] for r in seg])
  meas = np.array([r["meas"] for r in seg])
  cmd = np.array([r["desired"] for r in seg])
  dt = np.diff(t)
  good = dt > 1e-4
  if good.sum() < 5:
    return float("nan")
  dmeas = np.diff(meas)[good] / dt[good]
  err = (cmd[:-1] - meas[:-1])[good]
  denom = float(dmeas @ dmeas)
  if denom < 1e-12:
    return float("nan")
  return float(err @ dmeas) / denom


def lead_ms(rows, lo, hi):
  """Cross-correlate commanded vs model curvature; positive = command leads the model."""
  seg = rows[lo:hi]
  if len(seg) < 20:
    return float("nan")
  a = np.array([r["desired"] for r in seg])
  b = np.array([r["model_kappa"] for r in seg])
  if np.std(a) < 1e-9 or np.std(b) < 1e-9:
    return float("nan")
  a = (a - a.mean()) / np.std(a)
  b = (b - b.mean()) / np.std(b)
  t = np.array([r["t"] for r in seg])
  dt = np.median(np.diff(t)) or 0.05
  best, best_lag = -np.inf, 0
  span = min(30, len(a) // 3)
  for lag in range(-span, span + 1):
    if lag < 0:
      c = float(a[-lag:] @ b[:lag]) / (len(a) + lag)
    elif lag > 0:
      c = float(a[:-lag] @ b[lag:]) / (len(a) - lag)
    else:
      c = float(a @ b) / len(a)
    if c > best:
      best, best_lag = c, lag
  return best_lag * dt * 1000.0


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  src = p.add_mutually_exclusive_group(required=True)
  src.add_argument("--device", action="store_true")
  src.add_argument("--logs", nargs="+", type=Path)
  p.add_argument("--host", default=DEFAULT_HOST)
  p.add_argument("--route")
  args = p.parse_args()

  work = Path(tempfile.mkdtemp(prefix="ford_lat_"))
  try:
    paths = fetch_newest(args.host, work, args.route) if args.device else list(args.logs)
    rows = read_signals(paths)
    if not rows:
      raise SystemExit("no controllerStateBP in these logs -- is this a BluePilot build?")

    engaged = [r for r in rows if r["lat_active"]]
    modes = {r["mode"] for r in engaged if r["mode"]}
    print(f"\nsamples: {len(rows)}   lat-active: {len(engaged)}   modes: {modes or '-'}")
    if not engaged:
      raise SystemExit("lateral never active in this drive -- nothing to analyse")

    dev = sum(r["dev_clip"] for r in engaged) / len(engaged) * 100
    rate = sum(r["rate_lim"] for r in engaged) / len(engaged) * 100
    print(f"deviation clip bound : {dev:5.1f}% of engaged frames")
    print(f"angle rate-limit bit : {rate:5.1f}% of engaged frames")

    corners = find_corners(rows)
    print(f"\ncorners found: {len(corners)}  (|kappa| > {CORNER_KAPPA}, >= {MIN_CORNER_S}s)")
    if corners:
      print(f"\n{'#':>3} {'dur':>6} {'v':>7} {'peak k':>8} {'radius':>8} {'dev-clip':>9} {'rate-lim':>9} {'tau':>7} {'lead':>8}")
      print("-" * 76)
      for i, (lo, hi) in enumerate(corners, 1):
        seg = rows[lo:hi]
        pk = max(abs(r["desired"]) for r in seg)
        vv = np.mean([r["v"] for r in seg])
        d = sum(r["dev_clip"] for r in seg) / len(seg) * 100
        rl = sum(r["rate_lim"] for r in seg) / len(seg) * 100
        tau = estimate_tau(rows, lo, hi)
        ld = lead_ms(rows, lo, hi)
        dur = seg[-1]["t"] - seg[0]["t"]
        radius = 1 / pk if pk > 1e-6 else 0
        cols = f"{i:>3} {dur:>5.1f}s {vv*2.237:>5.1f}mph {pk:>8.4f} {radius:>7.0f}m"
        print(f"{cols} {d:>8.0f}% {rl:>8.0f}% {tau:>6.2f}s {ld:>6.0f}ms")

      taus = [estimate_tau(rows, lo, hi) for lo, hi in corners]
      taus = [x for x in taus if np.isfinite(x) and 0.01 < x < 2.0]
      print("\n--- verdict ---")
      if taus:
        tau_med = float(np.median(taus))
        ceiling = CURVATURE_ERROR / tau_med
        print(f"median tau            : {tau_med:.3f}s")
        print(f"=> max buildable dk/dt: {ceiling:.4f} (1/m)/s   [CURVATURE_ERROR/tau]")
        print(f"   a 40m-radius turn taken over 2s needs ~{(1/40)/2:.4f} (1/m)/s")
        if ceiling < (1 / 40) / 2:
          print("   -> ceiling is BELOW what a normal 90-degree turn needs: deviation clip is the limiter")
      if dev > 10:
        print(f"deviation clip bound {dev:.0f}% of the time -> mechanism (a) dominates.")
        print("The command cannot demand more curvature than measured+0.002, so the car understeers wide.")
      else:
        print(f"deviation clip rarely bound ({dev:.0f}%) -> look at mechanism (b), blend phase lead.")
        print("Check the per-corner 'lead' column (positive = command leads the model).")
    return 0
  finally:
    shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
  sys.exit(main())
