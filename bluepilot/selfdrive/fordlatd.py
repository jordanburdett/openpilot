#!/usr/bin/env python3
"""BluePilot: continuous learning of Ford angle-mode lateral gain.

Learns a small DELTA on top of whatever you have set by hand. Your setting stays the anchor:

    effective = clip(user_setting + learned_delta, 0.5, 1.5)

so the learner starts from your preference and tunes around it rather than replacing it. Change
the setting yourself and the delta resets to zero -- you have expressed a new intent, and the
old correction was learned relative to the old anchor.

Two indicators drive it, both validated offline against real drives (see
openpilot/tools/autotune_ford_lateral.py):

  GAIN   -- least-squares slope of measured vs commanded curvature in sustained curves.
            Below 1.0 the car under-turns and runs wide; above 1.0 it over-turns.
  WANDER -- std-dev of offset from lane centre on near-straight road, from
            drivingModelData.laneLineMeta. This is what a driver perceives as ping-ponging.

Gain alone is not safe to chase. Measured on a Mach-E, gain reaches 1.0 around factor 1.20, but
lane wander is minimised at 1.15 and is 44% worse by 1.21 -- the driver reported exactly that as
"snippy, cuts corners, ping-pongs". So gain sets the direction and wander is a veto: if a step
makes wander meaningfully worse than the best seen at this anchor, it is taken back and the
learner stops pushing that way.

Safety, in order of how much it matters:
  * the delta is clamped to +-MAX_DELTA around YOUR value, and the sum is clamped to 0.5-1.5
  * steps are STEP per update, and an update needs MIN_UPDATE_SECONDS of qualifying data
  * qualifying means: engaged, angle mode, no driver torque, not standstill, in the speed band,
    and (for wander) both lane lines confident
  * a version key and the car fingerprint are stored with the state; either changing wipes it
  * the toggle off, or FordAngleLearningReset, reverts to your value immediately
  * nothing is learned or applied unless FordAngleLearningEnabled is on -- default off
"""

import json
import time

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog

VERSION = 1                    # bump to invalidate stored deltas

LEARNED_PARAM = "FordAngleLearned"
ENABLED_PARAM = "FordAngleLearningEnabled"
RESET_PARAM = "FordAngleLearningReset"
USER_PARAMS = {"high": "FordHighSpeedFactor_ang", "low": "FordLowSpeedFactor_ang"}

MAX_DELTA = 0.15               # how far the learner may move from your setting
STEP = 0.01                    # per update
GAIN_TARGET = 1.00
GAIN_DEADBAND = 0.03           # inside this, gain is happy
WANDER_VETO = 1.15             # worse than this multiple of best-seen -> step back

MIN_UPDATE_SECONDS = 180.0     # qualifying seconds before an update is considered
MIN_STEADY_SAMPLES = 400
MIN_WANDER_SAMPLES = 200
MIN_LANE_PROB = 0.5

# speed bands, matching which user setting dominates the gain schedule
BANDS = {"high": (26.82, 100.0), "low": (5.0, 13.5)}
STEADY_DKAPPA = 0.0006         # |d(desired)/dt| below this counts as steady
MIN_KAPPA = 0.0004             # real curvature, not straight
STRAIGHT_KAPPA = 0.0006        # near-straight, for wander


class BandLearner:
  """Accumulates the two indicators for one speed band and steps the delta."""

  def __init__(self, name: str):
    self.name = name
    self.delta = 0.0
    self._last_des: float | None = None
    self.best_wander: float | None = None
    self.best_delta = 0.0
    self.updates = 0
    self.wander_limited = False
    self.reset_accumulators()

  def reset_accumulators(self):
    self._last_des = None
    self._des: list[float] = []
    self._meas: list[float] = []
    self._off: list[float] = []
    self._secs = 0.0

  def is_steady(self, desired, dt):
    """Steady = real curvature that is not currently ramping. Delay effects wash out here,
    which is what makes the gain fit robust to how tightly this is filtered."""
    prev, self._last_des = self._last_des, desired
    if prev is None or dt <= 0:
      return False
    return abs(desired) > MIN_KAPPA and abs(desired - prev) / dt < STEADY_DKAPPA

  def add(self, dt, desired, measured, steady, offset, straight):
    self._secs += dt
    if steady:
      self._des.append(desired)
      self._meas.append(measured)
    if straight and offset is not None:
      self._off.append(offset)

  def ready(self):
    return (self._secs >= MIN_UPDATE_SECONDS
            and len(self._des) >= MIN_STEADY_SAMPLES
            and len(self._off) >= MIN_WANDER_SAMPLES)

  def measure(self):
    des = np.array(self._des)
    meas = np.array(self._meas)
    A = np.vstack([des, np.ones(len(des))]).T
    gain, _ = np.linalg.lstsq(A, meas, rcond=None)[0]
    wander = float(np.std(self._off))
    return float(gain), wander

  def step(self):
    """Update the delta from the accumulated window. Returns a log dict, or None."""
    if not self.ready():
      return None
    gain, wander = self.measure()
    self.reset_accumulators()
    if not np.isfinite(gain) or not (0.2 < gain < 3.0):
      return None

    prev = self.delta
    action = "hold"

    # WANDER VETO first: comfort outranks accuracy.
    if self.best_wander is not None and wander > self.best_wander * WANDER_VETO:
      # this delta is worse than the best we have seen -- go back toward that
      self.delta = float(np.clip(self.best_delta, -MAX_DELTA, MAX_DELTA))
      self.wander_limited = True
      action = "veto"
    else:
      if self.best_wander is None or wander < self.best_wander:
        self.best_wander, self.best_delta = wander, self.delta
      if gain < GAIN_TARGET - GAIN_DEADBAND:
        if self.wander_limited and self.delta >= self.best_delta:
          action = "blocked"          # gain wants more, wander says no: a phase problem
        else:
          self.delta = float(np.clip(self.delta + STEP, -MAX_DELTA, MAX_DELTA))
          action = "up"
      elif gain > GAIN_TARGET + GAIN_DEADBAND:
        self.delta = float(np.clip(self.delta - STEP, -MAX_DELTA, MAX_DELTA))
        action = "down"

    self.updates += 1
    return {"band": self.name, "gain": round(gain, 4), "wander": round(wander, 4),
            "delta": round(self.delta, 4), "was": round(prev, 4), "action": action,
            "best_wander": round(self.best_wander, 4) if self.best_wander else None,
            "wander_limited": self.wander_limited, "updates": self.updates}


class FordLatLearner:
  def __init__(self, params: Params):
    self.params = params
    self.bands = {k: BandLearner(k) for k in BANDS}
    self.fingerprint = ""
    self.anchors = dict.fromkeys(BANDS)
    self.dirty = False

  # ---- persistence -------------------------------------------------------
  def state(self):
    return {"version": VERSION, "fingerprint": self.fingerprint,
            "anchors": self.anchors,
            "bands": {k: {"delta": b.delta, "best_wander": b.best_wander,
                          "best_delta": b.best_delta, "updates": b.updates,
                          "wander_limited": b.wander_limited}
                      for k, b in self.bands.items()}}

  def save(self):
    self.params.put(LEARNED_PARAM, json.dumps(self.state()))
    self.dirty = False

  def load(self, fingerprint):
    self.fingerprint = fingerprint
    raw = self.params.get(LEARNED_PARAM)
    if not raw:
      return
    try:
      st = json.loads(raw if isinstance(raw, str) else raw.decode())
    except Exception:
      cloudlog.exception("fordlatd: unreadable learned state, discarding")
      self.reset("unreadable")
      return
    if st.get("version") != VERSION or st.get("fingerprint") != fingerprint:
      cloudlog.info("fordlatd: state is for a different version/car, discarding")
      self.reset("version_or_car")
      return
    self.anchors = {k: st.get("anchors", {}).get(k) for k in BANDS}
    for k, b in self.bands.items():
      d = st.get("bands", {}).get(k, {})
      b.delta = float(d.get("delta", 0.0))
      b.best_wander = d.get("best_wander")
      b.best_delta = float(d.get("best_delta", 0.0))
      b.updates = int(d.get("updates", 0))
      b.wander_limited = bool(d.get("wander_limited", False))

  def reset(self, why):
    cloudlog.info(f"fordlatd: reset ({why})")
    for k in self.bands:
      self.bands[k] = BandLearner(k)
    self.anchors = dict.fromkeys(BANDS)
    self.save()

  # ---- anchor tracking ---------------------------------------------------
  def check_anchors(self):
    """If the driver changed a setting, the old delta was learned against a different anchor."""
    for band, key in USER_PARAMS.items():
      try:
        raw = self.params.get(key, return_default=True)
        val = round(float(raw.decode() if isinstance(raw, bytes) else raw), 3)
      except (TypeError, ValueError, AttributeError):
        continue
      if self.anchors[band] is None:
        self.anchors[band] = val
      elif abs(self.anchors[band] - val) > 1e-6:
        cloudlog.info(f"fordlatd: {key} changed {self.anchors[band]} -> {val}, resetting {band}")
        self.bands[band] = BandLearner(band)
        self.anchors[band] = val
        self.dirty = True


def main():
  params = Params()
  sm = messaging.SubMaster(["carState", "carControl", "controlsState",
                            "controllerStateBP", "drivingModelData", "carParams"])
  learner = FordLatLearner(params)
  rk = Ratekeeper(20.0, print_delay_threshold=None)

  loaded = False
  last_t = time.monotonic()
  offset = None
  lane_prob = 0.0

  while True:
    sm.update(0)

    if params.get_bool(RESET_PARAM):
      learner.reset("user requested")
      params.put_bool(RESET_PARAM, False)

    enabled = params.get_bool(ENABLED_PARAM)

    if not loaded and sm.valid.get("carParams", False):
      learner.load(str(sm["carParams"].carFingerprint))
      loaded = True

    now = time.monotonic()
    dt = min(now - last_t, 0.5)
    last_t = now

    if sm.updated.get("drivingModelData", False):
      lm = sm["drivingModelData"].laneLineMeta
      offset = (float(lm.leftY) + float(lm.rightY)) / 2.0
      lane_prob = min(float(lm.leftProb), float(lm.rightProb))

    if enabled and loaded and all(sm.valid.get(s, False) for s in ("carState", "carControl", "controllerStateBP")):
      learner.check_anchors()
      cs, cc, bp = sm["carState"], sm["carControl"], sm["controllerStateBP"]
      angle_mode = str(bp.activeLateralMode) == "angle"
      qualifies = (angle_mode and cc.latActive and not cs.steeringPressed
                   and not cs.standstill and not bp.curvatureDeviationLimited)
      if qualifies:
        desired = float(cc.actuators.curvature)
        measured = float(sm["controlsState"].curvature)
        v = float(cs.vEgo)
        for band, (lo, hi) in BANDS.items():
          if not (lo <= v < hi):
            continue
          b = learner.bands[band]
          steady = b.is_steady(desired, dt)
          straight = abs(desired) < STRAIGHT_KAPPA and lane_prob >= MIN_LANE_PROB
          b.add(dt, desired, measured, steady, offset, straight)

      for band in BANDS:
        info = learner.bands[band].step()
        if info is not None:
          cloudlog.info(f"fordlatd: {json.dumps(info)}")
          learner.dirty = True

      if learner.dirty:
        learner.save()

    rk.keep_time()


if __name__ == "__main__":
  main()
