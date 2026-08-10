"""
BluePilot: angle-mode "advanced lane positioning" -- a curvature-domain trim toward true
lane-line center (plus an optional user left/right bias), for use with path_angle-primary
control (``lateral_angle_ext.py``).

Under curvature-primary control, lane positioning rode on a second DBC signal (path_angle as
a heading trim, c3) layered on top of curvature (c1, the primary). In angle mode, path_angle
IS the primary signal -- there is no separate trim slot on the wire. An earlier attempt ported
the old PID controller as an additive delta on the *final* path_angle value and it never tracked
lane center correctly: path_angle there is a derived quantity (``kappa_cmd * v_ego *
curvature_factor``), so an additive trim in that domain has the wrong (inverted) speed-dependence
for a lane-centering nudge, and it bypasses every limiter angle mode already applies to
``kappa_cmd`` (deviation clip, PSCM-saturation clamp, DBC clip, soft ROC).

This class instead computes a small correction in the **curvature domain**, meant to be added to
``kappa_cmd`` before any of those limiters run -- so the correction automatically inherits all of
them, with no new safety-relevant code path. This mirrors where a GM-focused fork (StarPilot,
u/jc01rho -- ``selfdrive/controls/lib/drive_helpers.py::LaneCenteringController`` on
``feat/lane-centering-camera-offset``) places their own lane-centering trim: on the universal
``desired_curvature`` signal, upstream of any car-specific actuator conversion. The geometry here
(``raw_correction = 2*error / lookahead**2``) is the same ``y ~= 1/2 * kappa * x**2`` relation
BluePilot already uses for PSCM d_ref / path_angle elsewhere in this package, solved for the
curvature that would eliminate a lateral position error at the lookahead distance.
"""
import numpy as np
from numpy import interp

# Confidence gates on lane-line quality (lane-detection-quality checks, not BluePilot tuning --
# ported as-is from the reference implementation).
_MIN_LANE_PROB = 0.6
_MAX_LANE_STD = 0.3
_MIN_LANE_WIDTH_M = 2.6
_MAX_LANE_WIDTH_M = 4.8

# Speed ramp: reuses curvature-mode's already-tuned centering-authority envelope
# (LC_PID_speed_bp/v in lateral_curv_ext.py) instead of inventing a new one.
_SPEED_RAMP_BP = (0.0, 9.0, 15.0)  # m/s
_SPEED_RAMP_V = (0.0, 0.0, 1.0)

# Lookahead distance for the position-error sample, clipped to a sane range (m).
_LOOKAHEAD_MIN_M = 8.0
_LOOKAHEAD_MAX_M = 35.0

# Hard safety ceiling on the raw correction magnitude (1/m) -- fixed, not a "feel" knob, same
# treatment as _PSCM_SAT_UNWIND_RATE / _soft_roc in lateral_angle_ext.py.
_MAX_RAW_CORRECTION = 0.004

# First-order smoothing time constant (s) -- avoids abrupt jumps in the trim.
_SMOOTH_TAU_S = 0.4


class LaneCenterTrim:
  def __init__(self):
    self._correction = 0.0

  def reset(self) -> None:
    self._correction = 0.0

  def update(self, kappa_cmd: float, model, v_ego: float, enabled: bool, offset: float,
             gain: float, lat_active: bool, lane_change: bool) -> float:
    """Returns ``kappa_cmd``, nudged toward lane-line center + ``offset`` when confident.

    ``offset`` (m): positive shifts the target right, negative left (same sign convention as
    curvature mode's ``custom_path_offset_curv``).
    ``gain`` (0.0-1.0): user-tunable authority -- how much of the (already magnitude-clipped)
    raw correction is actually applied. 0 disables the trim's effect without disabling detection.
    """
    if not enabled or not lat_active or lane_change or model is None:
      self.reset()
      return kappa_cmd

    speed_factor = float(interp(v_ego, _SPEED_RAMP_BP, _SPEED_RAMP_V))
    if speed_factor <= 0.0:
      self.reset()
      return kappa_cmd

    valid, raw = self._raw_correction(model, v_ego, offset)
    if not valid:
      self.reset()
      return kappa_cmd

    target = float(np.clip(raw, -_MAX_RAW_CORRECTION, _MAX_RAW_CORRECTION)) * float(np.clip(gain, 0.0, 1.0)) * speed_factor
    alpha = 1.0 - np.exp(-0.05 / _SMOOTH_TAU_S)  # BluePilot lateral tick is 20 Hz (dt=0.05s)
    self._correction = float(alpha * target + (1.0 - alpha) * self._correction)
    return kappa_cmd + self._correction

  @property
  def correction(self) -> float:
    """Telemetry: last applied correction (1/m)."""
    return self._correction

  def _raw_correction(self, model, v_ego: float, offset: float) -> tuple[bool, float]:
    try:
      lane_lines = model.laneLines
      probs = model.laneLineProbs
      stds = model.laneLineStds
      if len(lane_lines) < 3 or len(probs) < 3 or len(stds) < 3:
        return False, 0.0
      if probs[1] < _MIN_LANE_PROB or probs[2] < _MIN_LANE_PROB:
        return False, 0.0
      if stds[1] > _MAX_LANE_STD or stds[2] > _MAX_LANE_STD:
        return False, 0.0

      left_x = np.asarray(lane_lines[1].x, dtype=float)
      left_y = np.asarray(lane_lines[1].y, dtype=float)
      right_x = np.asarray(lane_lines[2].x, dtype=float)
      right_y = np.asarray(lane_lines[2].y, dtype=float)
      if (left_x.size < 2 or left_x.size != left_y.size or
          right_x.size < 2 or right_x.size != right_y.size):
        return False, 0.0
      if not (np.isfinite(left_x).all() and np.isfinite(left_y).all() and
              np.isfinite(right_x).all() and np.isfinite(right_y).all()):
        return False, 0.0
      if not (np.all(np.diff(left_x) > 0) and np.all(np.diff(right_x) > 0)):
        return False, 0.0

      lookahead = float(np.clip(v_ego, _LOOKAHEAD_MIN_M, _LOOKAHEAD_MAX_M))
      left = float(np.interp(lookahead, left_x, left_y))
      right = float(np.interp(lookahead, right_x, right_y))
      width = right - left
      if width < _MIN_LANE_WIDTH_M or width > _MAX_LANE_WIDTH_M:
        return False, 0.0

      center_y = 0.5 * (left + right)

      pos_x = np.asarray(model.position.x, dtype=float)
      pos_y = np.asarray(model.position.y, dtype=float)
      if pos_x.size < 2 or pos_x.size != pos_y.size:
        return False, 0.0
      if not (np.isfinite(pos_x).all() and np.isfinite(pos_y).all() and np.all(np.diff(pos_x) > 0)):
        return False, 0.0
      model_y = float(np.interp(lookahead, pos_x, pos_y))

      error = (center_y + offset) - model_y
      raw = 2.0 * error / (lookahead ** 2)
      return True, float(raw)
    except (AttributeError, IndexError, TypeError, ValueError):
      return False, 0.0
