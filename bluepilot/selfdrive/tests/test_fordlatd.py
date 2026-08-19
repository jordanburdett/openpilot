"""Tests for the Ford angle-mode continuous learner.

The learner adjusts steering authority on a moving car, so the properties that matter are the
ones that bound it: it must never exceed its delta clamp, must not chase gain past the point
where the car starts wandering, and must forget everything the moment the driver changes their
own setting or asks for a reset.
"""

import json
import unittest

from bluepilot.selfdrive.fordlatd import (
    BandLearner, FordLatLearner, MAX_DELTA, STEP, GAIN_TARGET, GAIN_DEADBAND,
    WANDER_VETO, MIN_STEADY_SAMPLES, MIN_WANDER_SAMPLES, MIN_UPDATE_SECONDS, VERSION,
)


def feed(band, gain, wander, n_steady=MIN_STEADY_SAMPLES, n_wander=MIN_WANDER_SAMPLES,
         secs=MIN_UPDATE_SECONDS):
  """Push a window whose least-squares fit is exactly `gain` and whose offset sd is `wander`."""
  band._secs = secs
  band._des = []
  band._meas = []
  for i in range(n_steady):
    d = 0.001 + 0.00002 * (i % 50)
    band._des.append(d)
    band._meas.append(d * gain)
  # offsets with the requested standard deviation, zero mean
  band._off = [wander if i % 2 else -wander for i in range(n_wander)]


class TestBandLearner(unittest.TestCase):
  def test_under_gain_steps_up(self):
    b = BandLearner("high")
    info = (feed(b, 0.85, 0.10), b.step())[1]
    assert info["action"] == "up"
    assert b.delta == STEP

  def test_over_gain_steps_down(self):
    b = BandLearner("high")
    feed(b, 1.20, 0.10)
    info = b.step()
    assert info["action"] == "down"
    assert b.delta == -STEP

  def test_gain_in_deadband_holds(self):
    b = BandLearner("high")
    feed(b, GAIN_TARGET, 0.10)
    info = b.step()
    assert info["action"] == "hold"
    assert b.delta == 0.0

  def test_delta_is_clamped(self):
    b = BandLearner("high")
    for _ in range(200):          # far more steps than the clamp allows
      feed(b, 0.5, 0.10)
      b.step()
    assert b.delta <= MAX_DELTA + 1e-9
    assert abs(b.delta - MAX_DELTA) < 1e-9

  def test_wander_veto_reverts_and_latches(self):
    b = BandLearner("high")
    feed(b, 0.85, 0.10)           # establishes best_wander = 0.10 at delta 0
    b.step()
    assert b.delta == STEP
    feed(b, 0.85, 0.10 * WANDER_VETO * 1.5)   # much worse wander
    info = b.step()
    assert info["action"] == "veto"
    assert b.delta == 0.0         # back to the best-known delta
    assert b.wander_limited

  def test_veto_blocks_further_gain_chasing(self):
    """The whole point: gain wanting more must not win once wander has objected."""
    b = BandLearner("high")
    feed(b, 0.85, 0.10)
    b.step()
    feed(b, 0.85, 0.50)
    b.step()                      # veto -> back to 0.0, latched
    feed(b, 0.85, 0.10)           # gain still low, wander fine again
    info = b.step()
    assert info["action"] == "blocked"
    assert b.delta == 0.0

  def test_veto_still_allows_stepping_down(self):
    b = BandLearner("high")
    b.wander_limited = True
    b.best_wander, b.best_delta = 0.10, 0.0
    feed(b, 1.30, 0.10)
    info = b.step()
    assert info["action"] == "down"

  def test_not_enough_data_does_nothing(self):
    b = BandLearner("high")
    feed(b, 0.5, 0.10, n_steady=10, n_wander=10, secs=5.0)
    assert b.step() is None
    assert b.delta == 0.0

  def test_absurd_gain_is_ignored(self):
    b = BandLearner("high")
    feed(b, 50.0, 0.10)
    assert b.step() is None
    assert b.delta == 0.0


class _FakeParams:
  def __init__(self, d=None):
    self.d = dict(d or {})

  def get(self, k, return_default=False):
    return self.d.get(k)

  def get_bool(self, k):
    return bool(self.d.get(k, False))

  def put(self, k, v):
    self.d[k] = v

  def put_bool(self, k, v):
    self.d[k] = v


class TestPersistenceAndAnchors(unittest.TestCase):
  def test_roundtrip(self):
    p = _FakeParams()
    a = FordLatLearner(p)
    a.fingerprint = "FORD_MUSTANG_MACH_E_MK1"
    a.bands["high"].delta = 0.07
    a.bands["high"].best_wander = 0.11
    a.anchors["high"] = 1.15
    a.save()

    b = FordLatLearner(p)
    b.load("FORD_MUSTANG_MACH_E_MK1")
    assert b.bands["high"].delta == 0.07
    assert b.bands["high"].best_wander == 0.11
    assert b.anchors["high"] == 1.15

  def test_different_car_discards(self):
    p = _FakeParams()
    a = FordLatLearner(p)
    a.fingerprint = "FORD_MUSTANG_MACH_E_MK1"
    a.bands["high"].delta = 0.07
    a.save()
    b = FordLatLearner(p)
    b.load("FORD_F_150_LIGHTNING_MK1")
    assert b.bands["high"].delta == 0.0

  def test_version_bump_discards(self):
    p = _FakeParams()
    a = FordLatLearner(p)
    a.fingerprint = "X"
    a.bands["high"].delta = 0.07
    a.save()
    stored = json.loads(p.d["FordAngleLearned"])
    stored["version"] = VERSION + 1
    p.d["FordAngleLearned"] = json.dumps(stored)
    b = FordLatLearner(p)
    b.load("X")
    assert b.bands["high"].delta == 0.0

  def test_corrupt_state_discards(self):
    p = _FakeParams({"FordAngleLearned": "{not json"})
    a = FordLatLearner(p)
    a.load("X")
    assert a.bands["high"].delta == 0.0

  def test_user_changing_their_setting_resets_that_band(self):
    p = _FakeParams({"FordHighSpeedFactor_ang": "1.15", "FordLowSpeedFactor_ang": "1.00"})
    a = FordLatLearner(p)
    a.check_anchors()
    a.bands["high"].delta = 0.09
    a.bands["low"].delta = 0.05

    p.d["FordHighSpeedFactor_ang"] = "1.20"      # driver moves the high knob
    a.check_anchors()
    assert a.bands["high"].delta == 0.0, "high band must forget when its anchor moves"
    assert a.bands["low"].delta == 0.05, "low band is unaffected"

  def test_reset_clears_everything(self):
    p = _FakeParams()
    a = FordLatLearner(p)
    a.bands["high"].delta = 0.09
    a.bands["low"].delta = -0.04
    a.reset("test")
    assert a.bands["high"].delta == 0.0
    assert a.bands["low"].delta == 0.0


class TestConvergence(unittest.TestCase):
  def test_converges_and_stops(self):
    """Model a car whose gain rises with the delta; the learner should settle in the deadband."""
    b = BandLearner("high")
    for _ in range(60):
      gain = 0.85 + b.delta * 1.0        # matches the measured ~1.0 slope
      feed(b, gain, 0.10)
      b.step()
    assert abs(GAIN_TARGET - (0.85 + b.delta)) <= GAIN_DEADBAND + 1e-9
    assert b.delta <= MAX_DELTA

  def test_stops_at_wander_floor_instead_of_chasing_gain(self):
    """The Mach-E case: gain wants more than wander tolerates. Must not keep climbing."""
    b = BandLearner("high")
    for _ in range(60):
      gain = 0.85 + b.delta * 1.0
      wander = 0.10 if b.delta <= 0.05 else 0.10 * (1 + 4 * (b.delta - 0.05))
      feed(b, gain, wander)
      b.step()
    assert b.delta <= 0.07, f"kept chasing gain past the wander floor: delta={b.delta}"
    assert b.wander_limited


if __name__ == "__main__":
  unittest.main()
