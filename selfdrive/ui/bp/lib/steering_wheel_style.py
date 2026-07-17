from enum import IntEnum

from openpilot.common.params import Params
from openpilot.common.params_pyx import UnknownKeyName


class SteeringWheelIconStyle(IntEnum):
  COMMA_4 = 0
  COMMA_3X = 1


def get_steering_wheel_icon_style(params: Params, device_default: SteeringWheelIconStyle) -> SteeringWheelIconStyle:
  """Return the configured wheel style without modifying Params."""
  try:
    raw_style = params.get("BPSteeringWheelIconStyle")
  except UnknownKeyName:
    # BluePilot: Allow UI development before common/params_pyx.so has been rebuilt.
    return device_default

  try:
    style = SteeringWheelIconStyle(raw_style) if raw_style is not None else device_default
  except (TypeError, ValueError):
    style = device_default

  return style


def ensure_steering_wheel_icon_style_initialized(
  params: Params, device_default: SteeringWheelIconStyle,
) -> SteeringWheelIconStyle:
  """Persist the device default when the wheel-style param is missing or invalid."""
  style = get_steering_wheel_icon_style(params, device_default)

  try:
    raw_style = params.get("BPSteeringWheelIconStyle")
  except UnknownKeyName:
    return style

  if raw_style != int(style):
    params.put("BPSteeringWheelIconStyle", int(style), block=False)

  return style
