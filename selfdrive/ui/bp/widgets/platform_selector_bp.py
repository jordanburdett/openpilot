"""
BluePilot: vehicle-screen additions on top of the sunnypilot platform selector.

Kept here rather than patched into selfdrive/ui/sunnypilot/layouts/settings/vehicle/platform_selector.py
so upstream merges don't clobber them. The only upstream touchpoint is the import line in
selfdrive/ui/sunnypilot/layouts/settings/vehicle/__init__.py.

Adds a "(by VIN)" marker when the car was identified by decoding its VIN rather than by its
firmware -- see match_vin_to_car() in opendbc/car/ford/values.py, the last-resort branch of Ford's
fuzzy matcher.
"""

import pyray as rl

from opendbc.car.ford.values import match_vin_to_car
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.platform_selector import LegendWidget, PlatformSelector
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.lib.styles import style


def fingerprinted_by_vin() -> bool:
  """True when the live CarParams came from a VIN decode: firmware matching gave up (fuzzy) and
  the VIN independently decodes to the same platform."""
  CP = ui_state.CP
  if CP is None or not CP.fuzzyFingerprint:
    return False
  return CP.carFingerprint in match_vin_to_car(CP.carVin)


class PlatformSelectorBP(PlatformSelector):
  def refresh(self):
    super().refresh()
    if not ui_state.params.get("CarPlatformBundle") and fingerprinted_by_vin():
      self.set_text(tr("{} (by VIN)").format(self._platform))


class LegendWidgetBP(LegendWidget):
  def __init__(self, platform_selector):
    super().__init__(platform_selector)
    self._rect.height += 60  # room for the extra VIN line

  def _render(self, rect):
    super()._render(rect)
    rl.draw_text_ex(self._font, tr("\"(by VIN)\" means the firmware was unrecognized and the VIN was decoded instead."),
                    rl.Vector2(rect.x + 20, rect.y + 350), 40, 0, style.ITEM_DESC_TEXT_COLOR)
