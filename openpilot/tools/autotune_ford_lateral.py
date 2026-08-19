#!/usr/bin/env python3
"""Recommend Ford angle-mode lateral settings from real drives.

Three indicators, all measurable from logs the device already writes:

  1. STEADY-STATE GAIN  -- regression slope of measured vs commanded curvature through
     sustained curves. Says "not enough authority" (<1) or "too much" (>1). This is the
     strong one: it is a least-squares fit over thousands of samples, it is stable against
     how tightly you filter for steadiness, and it moves monotonically with the gain factor.

  2. CORNER OVERSHOOT   -- peak|measured| / peak|desired| per corner. Says "cutting corners".
     Noisier: a drive yields tens of corners, not thousands of samples.

  3. LANE WANDER        -- standard deviation of the car's offset from lane centre, taken from
     drivingModelData.laneLineMeta ((leftY + rightY) / 2), on near-straight road. Says
     "ping-ponging", and it is the constraint that stops indicator 1 from just being turned up
     until the gain reads 1.0.

     This one took some finding. Steering-rate RMS, wheel-reversal rate, steering jerk and
     spectral band power of the tracking error were all tried against a known-good/known-bad
     pair of settings and NONE of them separated the two -- they are dominated by road surface
     and traffic, not by tuning. Lane offset does separate them, cleanly, because it is the
     thing the driver actually perceives. It traces a U: too little authority wanders because
     the car drifts and corrects, too much wanders because it hunts, and the floor is the
     right setting.

Gain and oscillation disagree on purpose. A car that under-turns AND ping-pongs when you add
gain does not have a gain problem, it has a phase problem -- and the fix is lead
(FordVLTBaseMax / FordPathAngleBlendRatio), not authority. This tool says which case you are in.

    ./openpilot/tools/autotune_ford_lateral.py --device
    ./openpilot/tools/autotune_ford_lateral.py --logs <dir-of-qlogs>

qlogs are enough -- they carry controllerStateBP, carControl, controlsState and carState at
~10 Hz, which is ample, and they are ~25x smaller than rlogs.
"""

import argparse
import glob
import os
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

# thresholds -- deliberately conservative; these gate a steering-authority change
TARGET_GAIN = 1.00
GAIN_DEADBAND = 0.03          # inside this, leave it alone
MIN_STEADY_SAMPLES = 400      # per factor bucket, before a fit is trusted
WANDER_TOLERANCE = 1.15       # how much worse than the best bucket counts as degraded
MIN_WANDER_SAMPLES = 200      # lane-offset samples before a bucket is trusted
MIN_LANE_PROB = 0.5           # both lane lines must be this confident
OVERSHOOT_LIMIT = 1.04        # peak measured/desired
FACTOR_MIN, FACTOR_MAX = 0.5, 1.5
MAX_STEP = 0.05               # never recommend a jump larger than this


def ssh(host, cmd):
  return subprocess.run(["ssh", "-o", "ConnectTimeout=20", host, cmd],
                        capture_output=True, text=True, check=True).stdout.strip()


def fetch(host, dest, route=None, max_routes=2):
  listing = ssh(host, f"ls -1dt {REALDATA}/*--* 2>/dev/null | head -200")
  routes, order = {}, []
  for path in listing.splitlines():
    base, _, seg = Path(path).name.rpartition("--")
    if seg.isdigit():
      if base not in routes:
        routes[base] = []
        order.append(base)
      routes[base].append(path)
  wanted = [route] if route else order[:max_routes]
  out = []
  for r in wanted:
    for path in routes.get(r, []):
      name = Path(path).name
      target = dest / f"{name}.zst"
      if subprocess.run(["scp", "-q", f"{host}:{path}/qlog.zst", str(target)]).returncode == 0:
        out.append(target)
    print(f"  {r}: {len(routes.get(r, []))} segments")
  return out


def read(paths):
  rows = []
  cur = {"t": 0.0, "v": 0.0, "des": 0.0, "meas": 0.0, "act": False, "press": False,
         "ss": False, "hf": 1.0, "lo": 1.0, "sa": 0.0, "mode": None, "dev": False,
         "off": float("nan"), "prob": 0.0}
  for p in paths:
    raw = zstandard.ZstdDecompressor().stream_reader(open(p, "rb")).read()
    try:
      for ev in log.Event.read_multiple_bytes(raw):
        w = ev.which()
        if w == "carState":
          cs = ev.carState
          cur.update(v=cs.vEgo, press=bool(cs.steeringPressed),
                     ss=bool(cs.standstill), sa=cs.steeringAngleDeg)
        elif w == "controlsState":
          cur["meas"] = ev.controlsState.curvature
        elif w == "carControl":
          cur["des"] = ev.carControl.actuators.curvature
          cur["act"] = bool(ev.carControl.latActive)
        elif w == "drivingModelData":
          lm = ev.drivingModelData.laneLineMeta
          cur["off"] = (float(lm.leftY) + float(lm.rightY)) / 2.0
          cur["prob"] = min(float(lm.leftProb), float(lm.rightProb))
        elif w == "controllerStateBP":
          c = ev.controllerStateBP
          cur["dev"] = bool(c.curvatureDeviationLimited)
          try:
            cur["mode"] = str(c.activeLateralMode)
            cur["hf"] = float(c.bmsHighSpeedAdjustmentFactor)
            cur["lo"] = float(c.bmsLowSpeedAdjustmentFactor)
          except Exception:
            pass
          cur["t"] = ev.logMonoTime / 1e9
          rows.append(dict(cur))
    except capnp.KjException:
      pass
  return rows


def analyse(rows, speed_lo, speed_hi, factor_key):
  a = lambda k: np.array([r[k] for r in rows])  # noqa: E731
  t, des, meas, v = a("t"), a("des"), a("meas"), a("v")
  act, press, ss = a("act"), a("press"), a("ss")
  off, prob = a("off"), a("prob")
  fac = a(factor_key)
  angle_mode = np.array([r["mode"] == "angle" for r in rows])

  dt = np.gradient(t)
  dt[dt <= 0] = 0.1
  ddes = np.gradient(des) / dt

  ok = act & ~press & ~ss & angle_mode & (v >= speed_lo) & (v < speed_hi)
  steady = ok & (np.abs(ddes) < 0.0006) & (np.abs(des) > 0.0004)

  out = {}
  for val in sorted({round(x, 2) for x in fac[ok]}):
    m = steady & (np.round(fac, 2) == val)
    if m.sum() < MIN_STEADY_SAMPLES:
      continue
    A = np.vstack([des[m], np.ones(m.sum())]).T
    gain, bias = np.linalg.lstsq(A, meas[m], rcond=None)[0]

    # lane wander, on near-straight road only so it is hunting rather than cornering
    o = (ok & (np.round(fac, 2) == val) & (np.abs(des) < 0.0006)
         & np.isfinite(off) & (prob >= MIN_LANE_PROB))
    wander = float(off[o].std()) if o.sum() >= MIN_WANDER_SAMPLES else float("nan")

    # corner overshoot
    ratios = []
    idx = np.where(ok & (np.round(fac, 2) == val))[0]
    k = 0
    while k < len(idx) - 40:
      j = idx[k]
      if abs(des[j]) < 0.0008 and np.abs(des[j:j + 25]).max() > 0.0015:
        w = slice(j, j + 50)
        d_, m_ = np.abs(des[w]).max(), np.abs(meas[w]).max()
        if d_ > 0.0015:
          ratios.append(m_ / d_)
        k += 50
        continue
      k += 1
    out[val] = {"n": int(m.sum()), "gain": float(gain), "bias": float(bias), "wander": wander,
                "overshoot": float(np.median(ratios)) if len(ratios) >= 5 else float("nan"),
                "corners": len(ratios), "mph": float(v[m].mean() * 2.237)}
  return out


def recommend(res, current, label):
  print(f"\n--- {label} ---")
  if not res:
    print("  not enough steady-state data in this speed range")
    return
  print(f"  {'factor':>7} {'n':>7} {'gain':>7} {'lane wander(m)':>15} {'overshoot':>10} {'mph':>6}")
  for val, r in sorted(res.items()):
    ov = f"{r['overshoot']:.3f}" if np.isfinite(r["overshoot"]) else "-"
    wd = f"{r['wander']:.4f}" if np.isfinite(r["wander"]) else "-"
    print(f"  {val:>7.2f} {r['n']:>7} {r['gain']:>7.3f} {wd:>15} {ov:>10} {r['mph']:>6.1f}")

  # --- indicator 1: what does steady-state gain want? ---
  xs = np.array(sorted(res))
  ys = np.array([res[x]["gain"] for x in xs])
  ws = np.array([res[x]["n"] for x in xs], dtype=float)
  gain_target, slope, icept = None, None, None
  if len(xs) >= 2 and np.ptp(xs) > 0.03:
    W = np.diag(ws / ws.sum())
    A = np.vstack([xs, np.ones(len(xs))]).T
    slope, icept = np.linalg.lstsq(W @ A, W @ ys, rcond=None)[0]
    print(f"  fit: gain = {slope:.3f} * factor + {icept:.3f}")
  near = min(res, key=lambda x: abs(x - current))
  r = res[near]
  if slope is not None and slope > 0.2:
    gain_target = (TARGET_GAIN - icept) / slope
  elif r["gain"] > 0.3:
    gain_target = near / r["gain"] * TARGET_GAIN
  if gain_target is not None:
    gain_target = float(np.clip(gain_target, FACTOR_MIN, FACTOR_MAX))

  # --- indicator 3: where is lane wander minimised? ---
  wandered = {v: x["wander"] for v, x in res.items() if np.isfinite(x["wander"])}
  wander_best = min(wandered, key=wandered.get) if wandered else None

  print(f"\n  currently set to {current:.3f}; measured gain here {r['gain']:.3f}")
  if gain_target is None:
    print("  -> not enough signal to recommend")
    return
  print(f"  steady-state gain wants : {gain_target:.2f}")
  if wander_best is None:
    print("  lane wander             : no confident lane-line data; treat the above with caution")
    step = float(np.clip(gain_target - current, -MAX_STEP, MAX_STEP))
    print(f"  => set to {current + step:.2f} (step capped at {MAX_STEP})")
    return

  best_w = wandered[wander_best]
  degraded = sorted(v for v, w in wandered.items() if w > best_w * WANDER_TOLERANCE)
  print(f"  lane wander minimised at: {wander_best:.2f}  ({best_w:.4f} m)")
  if degraded:
    print(f"  degraded (> {WANDER_TOLERANCE:.2f}x best) at: {', '.join(f'{d:.2f}' for d in degraded)}")

  # The two indicators disagreeing IS the diagnosis: the comfort optimum sitting below the
  # accuracy optimum means the car needs the command earlier, not bigger.
  above = [d for d in degraded if d > wander_best]
  if gain_target > wander_best + GAIN_DEADBAND:
    onset = min(above) if above else gain_target
    print("\n  => PHASE PROBLEM, not a gain problem.")
    print(f"     Gain wants {gain_target:.2f}, but wander is already minimised at {wander_best:.2f}"
          + (f" and degrades by {onset:.2f}." if above else "."))
    print(f"     Hold the factor at {wander_best:.2f} and buy the missing authority with lead:")
    print("       FordVLTBaseMax          -> raise toward your learned lateralDelay")
    print("       FordPathAngleBlendRatio -> raise from 0.50")
    print("     Both change WHEN the command arrives, not how much of it there is, so they")
    print("     close the gap without re-introducing the wander.")
    return

  target = gain_target if not above else min(gain_target, wander_best)
  if abs(TARGET_GAIN - r["gain"]) < GAIN_DEADBAND and abs(current - wander_best) < 0.03:
    print("\n  => both indicators agree with the current setting; leave it alone")
    return
  step = float(np.clip(target - current, -MAX_STEP, MAX_STEP))
  print(f"\n  => set to {current + step:.2f}  (target {target:.2f}, step capped at {MAX_STEP})")


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  src = p.add_mutually_exclusive_group(required=True)
  src.add_argument("--device", action="store_true")
  src.add_argument("--logs", nargs="+", type=Path, help="qlog files, or directories of them")
  p.add_argument("--host", default=DEFAULT_HOST)
  p.add_argument("--route")
  p.add_argument("--routes", type=int, default=2, help="how many recent routes to pull")
  args = p.parse_args()

  work = Path(tempfile.mkdtemp(prefix="ford_autotune_"))
  try:
    if args.device:
      paths = fetch(args.host, work, args.route, args.routes)
    else:
      paths = []
      for item in args.logs:
        paths.extend(sorted(Path(x) for x in glob.glob(os.path.join(item, "*.zst")))) if item.is_dir() else paths.append(item)
    if not paths:
      raise SystemExit("no logs")
    rows = read(paths)
    ang = [r for r in rows if r["mode"] == "angle" and r["act"]]
    print(f"\nsamples {len(rows)}   angle-mode engaged {len(ang)} ({len(ang)*0.1/60:.1f} min)")
    if len(ang) < 600:
      raise SystemExit("not enough angle-mode driving to tune on")
    cur_hi = float(np.median([r["hf"] for r in ang]))
    cur_lo = float(np.median([r["lo"] for r in ang]))
    dev = 100 * np.mean([r["dev"] for r in ang])
    print(f"deviation clip bound {dev:.1f}% of engaged frames"
          + ("  (not a limiter)" if dev < 5 else "  <-- HIGH: the clip is limiting you, fix that first"))

    recommend(analyse(rows, 26.82, 100, "hf"), cur_hi, "HIGH speed (>60 mph) -> FordHighSpeedFactor_ang")
    recommend(analyse(rows, 5, 13.5, "lo"), cur_lo, "LOW speed (<30 mph) -> FordLowSpeedFactor_ang")
    return 0
  finally:
    shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
  sys.exit(main())
