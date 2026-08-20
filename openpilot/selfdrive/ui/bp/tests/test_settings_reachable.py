"""Settings must be reachable in the on-device menu, not just declared somewhere.

This exists because of a real miss. Three angle-mode tuning params were added to
openpilot/sunnypilot/sunnylink/settings_ui_src/pages/vehicle.yaml, which compiles to
settings_ui.json -- and that file is the sunnylink companion-app schema. **Nothing on the
device reads it.** The params were declared, wired, tested and shipped, and the driver still
could not find them, because the on-device menu is built in Python in
openpilot/selfdrive/ui/bp/layouts/settings/bluepilot.py (tici) and .../bp/mici/... (mici).

Declaring a param and exposing it are different things, and neither the linter, the unit tests
nor the settings-schema test noticed the gap. This closes it: every param a driver is expected
to be able to change must appear in the layout that actually renders on their device.

Parsed statically from the source so the test needs no window, no raylib and no device.
"""

import ast
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[5]
TICI_SETTINGS = REPO_ROOT / "openpilot/selfdrive/ui/bp/layouts/settings/bluepilot.py"

# Params a driver must be able to reach from the comma 3X screen.
REQUIRED_TICI_PARAMS = {
  # angle-mode gain schedule
  "FordLowSpeedFactor_ang",
  "FordHighSpeedFactor_ang",
  "FordHighSpeedDampening_ang",
  # angle-mode phase lead
  "FordPathAngleBlendRatio",
  "FordVLTBaseMax",
  "FordVLTExtraMax",
  # continuous learning
  "FordAngleLearningEnabled",
  # angle-mode lane positioning
  "enable_lane_positioning_ang",
  "custom_path_offset_ang",
  "lane_centering_strength_ang",
}


def params_referenced(path: Path) -> set[str]:
  """Every param name the layout binds a control to.

  Covers the three shapes used in these layouts:
    float_control_item(..., param="X")           -> keyword
    toggle_item(..., callback=lambda s: self._toggle_callback(s, "X"))
    self._params.get_bool("X") / put_bool("X")
  """
  tree = ast.parse(path.read_text())
  found: set[str] = set()
  for node in ast.walk(tree):
    if not isinstance(node, ast.Call):
      continue
    for kw in node.keywords:
      if kw.arg == "param" and isinstance(kw.value, ast.Constant) and isinstance(kw.value.value, str):
        found.add(kw.value.value)
    fn = node.func
    name = fn.attr if isinstance(fn, ast.Attribute) else getattr(fn, "id", "")
    if name in ("_toggle_callback", "_safe_get_bool", "get_bool", "put_bool", "get", "put", "remove"):
      for arg in node.args:
        if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
          found.add(arg.value)
  return found


class TestDeviceSettingsReachable(unittest.TestCase):
  def test_tici_layout_exists(self):
    assert TICI_SETTINGS.is_file(), f"{TICI_SETTINGS} missing -- did the settings layout move?"

  def test_required_params_are_in_the_device_menu(self):
    referenced = params_referenced(TICI_SETTINGS)
    missing = sorted(REQUIRED_TICI_PARAMS - referenced)
    where = TICI_SETTINGS.relative_to(REPO_ROOT)
    hint = f"Add them to {where}. sunnylink's settings_ui yaml is NOT enough: the device never reads it."
    detail = "these params are NOT reachable from the on-device settings menu:\n  "
    assert not missing, detail + "\n  ".join(missing) + "\n\n" + hint

  def test_new_items_are_gated_on_angle_mode(self):
    """The angle-mode items must be enabled only in angle mode, like their neighbours."""
    src = TICI_SETTINGS.read_text()
    for attr in ("_path_angle_blend_ratio", "_vlt_base_max", "_vlt_extra_max",
                 "_angle_learning_enabled", "_angle_learning_reset"):
      with self.subTest(attr=attr):
        assert f"self.{attr}.action_item.set_enabled(is_angle)" in src, \
          f"{attr} is not gated on angle mode"

  def test_new_items_are_in_the_visible_angle_list(self):
    """Being constructed is not enough -- an item only renders if it is in angle_items."""
    tree = ast.parse(TICI_SETTINGS.read_text())
    in_list: set[str] = set()
    for node in ast.walk(tree):
      if isinstance(node, ast.Assign) and len(node.targets) == 1:
        tgt = node.targets[0]
        if isinstance(tgt, ast.Name) and tgt.id == "angle_items" and isinstance(node.value, ast.List):
          for el in node.value.elts:
            if isinstance(el, ast.Attribute):
              in_list.add(el.attr)
    assert in_list, "could not find the angle_items list"
    for attr in ("_path_angle_blend_ratio", "_vlt_base_max", "_vlt_extra_max",
                 "_angle_learning_enabled", "_angle_learning_reset"):
      with self.subTest(attr=attr):
        assert attr in in_list, f"{attr} is built but never added to angle_items, so it never renders"


if __name__ == "__main__":
  unittest.main()
