import pyray as rl

from msgq.visionipc import VisionStreamType
from openpilot.common.params import Params
from openpilot.selfdrive.ui.onroad.cameraview import CameraView as TiciCameraView
from openpilot.selfdrive.ui.ui_state import ui_state


class CameraViewBP(TiciCameraView):
  """BluePilot camera view overrides."""

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._bp_camera_params = Params()
    self._bp_hide_camera_view = self._bp_camera_params.get_bool("BPHideCameraView")
    self._bp_camera_param_counter = 0

  def _update_state(self):
    super()._update_state()
    self._bp_camera_param_counter += 1
    if self._bp_camera_param_counter >= 60:
      self._bp_camera_param_counter = 0
      self._bp_hide_camera_view = self._bp_camera_params.get_bool("BPHideCameraView")

  def _should_hide_camera_view(self) -> bool:
    return (
      self._bp_hide_camera_view and
      ui_state.is_onroad() and
      self._stream_type != VisionStreamType.VISION_STREAM_DRIVER
    )

  def _render(self, rect: rl.Rectangle):
    if self._should_hide_camera_view():
      self._calc_frame_matrix(rect)
      rl.draw_rectangle_rec(rect, rl.BLACK)

    TiciCameraView._render(self, rect)

  def _draw_placeholder(self, rect: rl.Rectangle):
    if self._should_hide_camera_view():
      rl.draw_rectangle_rec(rect, rl.BLACK)
    else:
      super()._draw_placeholder(rect)

  def _render_egl(self, src_rect: rl.Rectangle, dst_rect: rl.Rectangle) -> None:
    if self._should_hide_camera_view():
      return

    TiciCameraView._render_egl(self, src_rect, dst_rect)

  def _render_textures(self, src_rect: rl.Rectangle, dst_rect: rl.Rectangle) -> None:
    if self._should_hide_camera_view():
      return

    TiciCameraView._render_textures(self, src_rect, dst_rect)
