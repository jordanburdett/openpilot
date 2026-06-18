
import pyray as rl
from collections.abc import Callable
from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigButtonBP
from openpilot.selfdrive.ui.bp.mici.widgets.big_input_dialog_bp import BigInputDialogBP
from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app, MousePos, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached

CONTENT_MARGIN = 20
LINE_L = 40
LINE_W = 8
LABEL_HORIZONTAL_PADDING = 40

# Adjust-mode overlay (enlarged centered value display while user is tweaking)
ADJUST_DURATION_S = 5.0
ADJUST_FADE_IN_S = 0.15
ADJUST_FADE_OUT_S = 0.3
ADJUST_FONT_BASE = 30
ADJUST_FONT_TARGET = 90  # 3x base
SCROLL_RESET_PX = 8


def _draw_adjust_overlay(widget, last_adjust_time, last_rect_x):
  """Render the enlarged centered value when the widget is being adjusted.

  Returns updated (last_adjust_time, last_rect_x). Reset triggers:
    - 5s elapsed since last change
    - widget moved horizontally (scroller drag)
  """
  if last_adjust_time is not None and last_rect_x is not None:
    if abs(widget._rect.x - last_rect_x) > SCROLL_RESET_PX:
      last_adjust_time = None
  last_rect_x = widget._rect.x

  if last_adjust_time is None:
    return last_adjust_time, last_rect_x

  elapsed = rl.get_time() - last_adjust_time
  if elapsed < 0 or elapsed >= ADJUST_DURATION_S:
    return None, last_rect_x

  fade_in = min(elapsed / ADJUST_FADE_IN_S, 1.0)
  fade_out = min((ADJUST_DURATION_S - elapsed) / ADJUST_FADE_OUT_S, 1.0)
  factor = max(0.0, min(fade_in, fade_out, 1.0))

  value_str = widget._sub_label.text
  if factor <= 0 or not value_str:
    return last_adjust_time, last_rect_x

  font_size = int(ADJUST_FONT_BASE + (ADJUST_FONT_TARGET - ADJUST_FONT_BASE) * factor)
  font = gui_app.font(FontWeight.BOLD)
  text_size = measure_text_cached(font, value_str, font_size)

  pad_x, pad_y = 32, 16
  bg_w = text_size.x + pad_x * 2
  bg_h = text_size.y + pad_y * 2
  bg_x = widget._rect.x + (widget._rect.width - bg_w) / 2
  bg_y = widget._rect.y + (widget._rect.height - bg_h) / 2

  bg_rect = rl.Rectangle(bg_x, bg_y, bg_w, bg_h)
  rl.draw_rectangle_rounded(bg_rect, 0.3, 12, rl.Color(15, 15, 15, int(235 * factor)))

  rl.draw_text_ex(font, value_str, rl.Vector2(bg_x + pad_x, bg_y + pad_y), font_size, 0,
                  rl.Color(255, 255, 255, int(255 * factor)))

  return last_adjust_time, last_rect_x

class BigParamFloatControl(BigButtonBP):
  def __init__(self, text: str, param: str, is_active_param: str = None, is_active: Callable[[], bool] = None,
               min: float = None, max: float = None, step: float = 0.05, tint: rl.Color = rl.WHITE):
    active_fn = is_active
    if active_fn is None and is_active_param is not None:
      active_fn = lambda: Params().get_bool(is_active_param)
    super().__init__(text, "", tint=tint, is_active=active_fn)
    self.min = min
    self.max = max
    self.step = step

    self._sub_label.set_font_size(30)

    self.margin = self._rect.width * 0.1
    self.rect_size = LINE_L + 2 * CONTENT_MARGIN

    self.param = param
    self.params = Params()
    self.set_click_callback(self._on_click)
    self.update_label()

    self._last_adjust_time: float | None = None
    self._last_rect_x: float | None = None

  def _on_click(self):
    if self.min is not None or self.max is not None:
      message = f"({self.min}-{self.max})"
    else:
      message = "enter a numberic value..."

    def _wrapped_callback(val):
      self._callback(val)
      gui_app.pop_widget()

    dlg = BigInputDialogBP(message, str(self.get_param()),
                         confirm_callback=_wrapped_callback, show_special_keys=True, minimum_length=0)
    gui_app.push_widget(dlg)

  def _callback(self, password: str):
    if password:
      try:
        float_value = float(password)
        self.set_param(float_value)
      except ValueError:
        pass
    else:
      #revert to default
      self.params.remove(self.param)
      self.update_label()
    self._last_adjust_time = rl.get_time()

  def get_param(self) -> float:
    try:
      return float(self.params.get(self.param, return_default=True))
    except (TypeError, ValueError):
      return 0.0

  def set_param(self, value: float):
    if self.min is not None and value < self.min:
      value = self.min
    elif self.max is not None and value > self.max:
      value = self.max

    self.params.put(self.param, value, block=False)
    self.update_label(value)

  def update_label(self, value: float = None):
    if value is None:
      value = self.get_param()
    self.set_value(f"{round(value,4)}")

  def _get_label_font_size(self):
    font_size = super()._get_label_font_size()
    return font_size - 10

  def _draw_content(self, btn_y: float):
    offset = self.rect_size / 3
    self.rect.height -= offset
    super()._draw_content(btn_y + offset)
    self.rect.height += offset

  def _render(self, _):
    super()._render(_)

    self.left = self._rect.x + self.margin
    self.right = self._rect.x + self._rect.width - self.margin
    self.top = self._rect.y + self.margin

    self.minus_hit_rect = rl.Rectangle(
      self.left - CONTENT_MARGIN, self.top - self.rect_size / 2, self.rect_size, self.rect_size
    )
    self.plus_hit_rect = rl.Rectangle(
      self.right - self.rect_size / 2 - CONTENT_MARGIN, self.top - self.rect_size / 2, self.rect_size, self.rect_size
    )

    #rl.draw_rectangle_lines_ex(self.minus_hit_rect, 1, rl.RED)
    #rl.draw_rectangle_lines_ex(self.plus_hit_rect, 1, rl.GREEN)

    rl.draw_line_ex((self.left,self.top), (self.left+LINE_L, self.top), LINE_W, rl.WHITE)

    rl.draw_line_ex((self.right-LINE_L,self.top), (self.right, self.top), LINE_W, rl.WHITE)
    m = self.right - LINE_L/2
    rl.draw_line_ex((m,self.top-LINE_L/2), (m, self.top+LINE_L/2), LINE_W, rl.WHITE)

    self._last_adjust_time, self._last_rect_x = _draw_adjust_overlay(
      self, self._last_adjust_time, self._last_rect_x)

  def minus_clicked(self):
    self.set_param(self.get_param() - self.step)
    self._last_adjust_time = rl.get_time()

  def plus_clicked(self):
    self.set_param(self.get_param() + self.step)
    self._last_adjust_time = rl.get_time()

  def _handle_mouse_release(self, mouse_pos: MousePos):
    if rl.check_collision_point_rec(mouse_pos, self.minus_hit_rect):
      self.minus_clicked()
    elif rl.check_collision_point_rec(mouse_pos, self.plus_hit_rect):
      self.plus_clicked()
    else:
      super()._handle_mouse_release(mouse_pos)


class BigParamIntControl(BigButtonBP):
  def __init__(self, text: str, param: str, is_active_param: str = None, min: int = None, max: int = None, step: int = 1, tint: rl.Color = rl.WHITE):
    super().__init__(text, "", tint=tint, is_active=(lambda: Params().get_bool(is_active_param)) if is_active_param is not None else None)
    self.min = min
    self.max = max
    self.step = step

    self._sub_label.set_font_size(30)

    self.margin = self._rect.width * 0.1
    self.rect_size = LINE_L + 2 * CONTENT_MARGIN

    self.param = param
    self.params = Params()
    self.set_click_callback(self._on_click)
    self.update_label()

    self._last_adjust_time: float | None = None
    self._last_rect_x: float | None = None

  def _on_click(self):
    if self.min is not None or self.max is not None:
      message = f"({self.min}-{self.max})"
    else:
      message = "enter a numberic value..."

    def _wrapped_callback(val):
      self._callback(val)
      gui_app.pop_widget()

    dlg = BigInputDialogBP(message, str(self.get_param()),
                         confirm_callback=_wrapped_callback, show_special_keys=True, minimum_length=0)
    gui_app.push_widget(dlg)

  def _callback(self, password: str):
    if password:
      try:
        int_value = int(password)
        self.set_param(int_value)
      except ValueError:
        pass
    else:
      #revert to default
      self.params.remove(self.param)
      self.update_label()
    self._last_adjust_time = rl.get_time()

  def get_param(self) -> int:
    try:
      return int(self.params.get(self.param, return_default=True))
    except (TypeError, ValueError):
      return 0

  def set_param(self, value: int):
    value=int(value)
    if self.min is not None and value < self.min:
      value = self.min
    elif self.max is not None and value > self.max:
      value = self.max

    self.params.put(self.param, value, block=False)
    self.update_label(value)

  def update_label(self, value: int = None):
    if value is None:
      value = self.get_param()
    self.set_value(f"{value}")

  def _get_label_font_size(self):
    font_size = super()._get_label_font_size()
    return font_size - 10

  def _draw_content(self, btn_y: float):
    offset = self.rect_size / 3
    self.rect.height -= offset
    super()._draw_content(btn_y + offset)
    self.rect.height += offset

  def _render(self, _):
    super()._render(_)

    self.left = self._rect.x + self.margin
    self.right = self._rect.x + self._rect.width - self.margin
    self.top = self._rect.y + self.margin

    self.minus_hit_rect = rl.Rectangle(
      self.left - CONTENT_MARGIN, self.top - self.rect_size / 2, self.rect_size, self.rect_size
    )
    self.plus_hit_rect = rl.Rectangle(
      self.right - self.rect_size / 2 - CONTENT_MARGIN, self.top - self.rect_size / 2, self.rect_size, self.rect_size
    )

    #rl.draw_rectangle_lines_ex(self.minus_hit_rect, 1, rl.RED)
    #rl.draw_rectangle_lines_ex(self.plus_hit_rect, 1, rl.GREEN)

    rl.draw_line_ex((self.left,self.top), (self.left+LINE_L, self.top), LINE_W, rl.WHITE)

    rl.draw_line_ex((self.right-LINE_L,self.top), (self.right, self.top), LINE_W, rl.WHITE)
    m = self.right - LINE_L/2
    rl.draw_line_ex((m,self.top-LINE_L/2), (m, self.top+LINE_L/2), LINE_W, rl.WHITE)

    self._last_adjust_time, self._last_rect_x = _draw_adjust_overlay(
      self, self._last_adjust_time, self._last_rect_x)

  def set_step(self, value: int):
     value -= value % self.step
     self.set_param(value)

  def minus_clicked(self):
    self.set_step(self.get_param() - self.step)
    self._last_adjust_time = rl.get_time()

  def plus_clicked(self):
    self.set_step(self.get_param() + self.step)
    self._last_adjust_time = rl.get_time()

  def _handle_mouse_release(self, mouse_pos: MousePos):
    if rl.check_collision_point_rec(mouse_pos, self.minus_hit_rect):
      self.minus_clicked()
    elif rl.check_collision_point_rec(mouse_pos, self.plus_hit_rect):
      self.plus_clicked()
    else:
      super()._handle_mouse_release(mouse_pos)
