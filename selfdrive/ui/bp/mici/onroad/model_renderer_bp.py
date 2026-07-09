import numpy as np
import pyray as rl
from openpilot.common.params import Params
from openpilot.selfdrive.ui.mici.onroad.model_renderer import ModelRenderer, THROTTLE_COLORS, NO_THROTTLE_COLORS
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
# BluePilot: Rainbow shader moved to BP module after upstream removal
from openpilot.bluepilot.ui.lib.bp_shaders import draw_rainbow_polygon

class ModelRendererBP(ModelRenderer):
  def __init__(self):
    super().__init__()
    self._bp_params = Params()
    self._rainbow_v = 20
    self._disable_lane_line_status_color = self._bp_params.get_bool("BPDisableLaneLineStatusColor")
    self._rainbow_lane_lines = self._bp_params.get_bool("BPRainbowLines")

  def _update_state(self):
    super()._update_state()
    sm = ui_state.sm

    if self._counter % 60 == 0:
      self._disable_lane_line_status_color = self._bp_params.get_bool("BPDisableLaneLineStatusColor")
      self._rainbow_lane_lines = self._bp_params.get_bool("BPRainbowLines")

    if ui_state.rainbow_path or self._rainbow_lane_lines:
      v = sm['carState'].vEgo
      self._rainbow_v = np.clip(v, 2.5, 35) / 30

  def _draw_path(self, sm):
    if ui_state.rainbow_path:
      draw_rainbow_polygon(self._rect, self._path.projected_points, rainbow_v=self._rainbow_v)
    else:
      super()._draw_path(sm)

  def _draw_lane_lines(self):
    """Draw lane lines and road edges, with optional rainbow inner lane lines."""
    offset = np.array([self._rect.x, self._rect.y], dtype=np.float32)
    rainbow_lane_lines_active = self._rainbow_lane_lines_active(ui_state.sm)

    for i, lane_line in enumerate(self._lane_lines):
      if lane_line.projected_points.size == 0:
        continue

      points = lane_line.projected_points + offset
      if rainbow_lane_lines_active and i in (1, 2):
        alpha = float(np.clip(self._lane_line_probs[i], 0.0, 0.7))
        draw_rainbow_polygon(self._rect, points, rainbow_v=self._rainbow_v, alpha=alpha)
      else:
        color = self._get_ll_color(float(self._lane_line_probs[i]), i in (1, 2), i in (0, 1))
        draw_polygon(self._rect, points, color)

    for i, road_edge in enumerate(self._road_edges):
      if road_edge.projected_points.size == 0:
        continue

      color = self._get_ll_color(float(1.0 - self._road_edge_stds[i]), float(self._lane_line_probs[i + 1]) < 0.25, i == 0)
      draw_polygon(self._rect, road_edge.projected_points + offset, color)

  def _rainbow_lane_lines_active(self, sm) -> bool:
    if not self._rainbow_lane_lines or self._disable_lane_line_status_color:
      return False

    if sm.valid.get('carControl', False) and sm['carControl'].longActive:
      return True

    return ui_state.status in (UIStatus.ENGAGED, UIStatus.LONG_ONLY)
