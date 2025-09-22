import math
import numpy as np

from common.filter_simple import FirstOrderFilter
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, DT_CTRL
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.ford.carcontroller import (
  CarController as FordCarController,
  anti_overshoot,
  apply_creep_compensation,
  apply_ford_curvature_limits,
  LongCtrlState,
  VisualAlert,
)
from opendbc.car.ford import fordcan
from opendbc.car.ford.values import CarControllerParams, CAR, FordFlags
from opendbc.car.interfaces import V_CRUISE_MAX


class _MachELateralController:
  def __init__(self, CP):
    self.VM = VehicleModel(CP)
    self.dt = DT_CTRL * CarControllerParams.STEER_STEP

    self.curvature_filter = FirstOrderFilter(0.0, 0.35, self.dt, initialized=False)
    self.path_angle_filter = FirstOrderFilter(0.0, 0.3, self.dt, initialized=False)
    self.curvature_rate_filter = FirstOrderFilter(0.0, 0.25, self.dt, initialized=False)

    self.curvature_rate_max = 0.0008  # 1/m^2
    self.path_angle_max = 0.12  # radians
    self.path_angle_gain = 0.7

    self.last_filtered_curvature = 0.0
    self.last_limited_curvature = 0.0

  def _reset_filters(self):
    self.curvature_filter.initialized = False
    self.curvature_filter.x = 0.0
    for filt in (self.path_angle_filter, self.curvature_rate_filter):
      filt.initialized = False
      filt.x = 0.0
    self.last_filtered_curvature = 0.0
    self.last_limited_curvature = 0.0

  def update(self, lat_active: bool, desired_curvature: float, steering_angle_deg: float,
             v_ego_raw: float, steering_pressed: bool) -> tuple[float, float]:
    if not lat_active or v_ego_raw < 0.5:
      self._reset_filters()
      return desired_curvature, 0.0

    v_ref = max(v_ego_raw, 1.0)

    filtered_curvature = self.curvature_filter.update(desired_curvature)
    curvature_delta = filtered_curvature - self.last_filtered_curvature
    self.last_filtered_curvature = filtered_curvature

    desired_angle_deg = math.degrees(self.VM.get_steer_from_curvature(-filtered_curvature, v_ref, 0.0))
    steering_delta = (steering_angle_deg - desired_angle_deg) * math.pi / 180.0

    if steering_pressed:
      self.path_angle_filter.initialized = False
      self.path_angle_filter.x = 0.0
      path_angle = 0.0
    else:
      path_angle_target = steering_delta * self.path_angle_gain
      path_angle = self.path_angle_filter.update(path_angle_target)
      path_angle = float(np.clip(path_angle, -self.path_angle_max, self.path_angle_max))

    self.last_filtered_curvature = filtered_curvature
    return filtered_curvature, path_angle

  def curvature_rate(self, limited_curvature: float, v_ego_raw: float, lat_active: bool, steering_pressed: bool) -> float:
    if not lat_active or v_ego_raw < 0.5 or steering_pressed:
      self.curvature_rate_filter.initialized = False
      self.curvature_rate_filter.x = 0.0
      self.last_limited_curvature = limited_curvature
      return 0.0

    v_ref = max(v_ego_raw, 1.0)
    delta = limited_curvature - self.last_limited_curvature
    self.last_limited_curvature = limited_curvature

    rate_raw = delta / (self.dt * v_ref)
    rate_filtered = self.curvature_rate_filter.update(rate_raw)
    return float(np.clip(rate_filtered, -self.curvature_rate_max, self.curvature_rate_max))


class MachECarController(FordCarController):
  """Ford Mach-E specific L2 controller that augments the stock Ford controller."""

  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self._mach_e_controller = _MachELateralController(CP)

  def _create_lat_ctl2_msg(self, mode: int, ramp_type: int, precision_type: int,
                           path_angle: float, curvature: float, curvature_rate: float, counter: int):
    values = {
      "LatCtl_D2_Rq": mode,
      "LatCtlRampType_D_Rq": ramp_type,
      "LatCtlPrecision_D_Rq": precision_type,
      "LatCtlPathOffst_L_Actl": 0.0,
      "LatCtlPath_An_Actl": path_angle,
      "LatCtlCurv_No_Actl": curvature,
      "LatCtlCrv_NoRate2_Actl": curvature_rate,
      "HandsOffCnfm_B_Rq": 0,
      "LatCtlPath_No_Cnt": counter,
      "LatCtlPath_No_Cs": 0,
    }

    dat = self.packer.make_can_msg("LateralMotionControl2", 0, values)[1]
    values["LatCtlPath_No_Cs"] = fordcan.calculate_lat_ctl2_checksum(mode, counter, dat)
    return self.packer.make_can_msg("LateralMotionControl2", self.CAN.main, values)

  def update(self, CC, CC_SP, CS, now_nanos):  # pylint: disable=too-many-locals
    can_sends = []

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

    ### acc buttons ###
    if CC.cruiseControl.cancel:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, cancel=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, cancel=True))
    elif CC.cruiseControl.resume and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, resume=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, resume=True))
    # if stock lane centering isn't off, send a button press to toggle it off
    # the stock system checks for steering pressed, and eventually disengages cruise control
    elif CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0 and (self.frame % CarControllerParams.ACC_UI_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, tja_toggle=True))

    ### lateral control ###
    # send steer msg at 20Hz
    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      apply_curvature = actuators.curvature
      # Bronco and some other cars consistently overshoot curv requests
      # Apply some deadzone + smoothing convergence to avoid oscillations
      if self.CP.carFingerprint in (CAR.FORD_BRONCO_SPORT_MK1, CAR.FORD_F_150_MK14):
        self.anti_overshoot_curvature_last = anti_overshoot(actuators.curvature, self.anti_overshoot_curvature_last, CS.out.vEgoRaw)
        apply_curvature = self.anti_overshoot_curvature_last

      path_angle_cmd = 0.0
      apply_curvature, path_angle_cmd = self._mach_e_controller.update(
        CC.latActive, apply_curvature, CS.out.steeringAngleDeg, CS.out.vEgoRaw, CS.out.steeringPressed
      )

      # apply rate limits, curvature error limit, and clip to signal range
      current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)

      self.apply_curvature_last = apply_ford_curvature_limits(apply_curvature, self.apply_curvature_last, current_curvature,
                                                              CS.out.vEgoRaw, 0., CC.latActive, self.CP)

      curvature_rate_cmd = self._mach_e_controller.curvature_rate(self.apply_curvature_last, CS.out.vEgoRaw,
                                                                  CC.latActive, CS.out.steeringPressed)

      if self.CP.flags & FordFlags.CANFD:
        mode = 1 if CC.latActive else 0
        counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
        ramp_type = 2 if CC.latActive else 0
        precision_type = 1
        can_sends.append(self._create_lat_ctl2_msg(mode, ramp_type, precision_type,
                                                   -path_angle_cmd, -self.apply_curvature_last,
                                                   -curvature_rate_cmd, counter))
      else:
        can_sends.append(fordcan.create_lat_ctl_msg(self.packer, self.CAN, CC.latActive, 0., 0., -self.apply_curvature_last, 0.))

    # send lka msg at 33Hz
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      can_sends.append(fordcan.create_lka_msg(self.packer, self.CAN))

    ### longitudinal control ###
    # send acc msg at 50Hz
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      accel = actuators.accel
      gas = accel

      if CC.longActive:
        accel = apply_creep_compensation(accel, CS.out.vEgo)
        accel = max(accel, self.accel - (3.5 * CarControllerParams.ACC_CONTROL_STEP * DT_CTRL))

      accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      gas = float(np.clip(gas, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

      if not CC.longActive or gas < CarControllerParams.MIN_GAS:
        gas = CarControllerParams.INACTIVE_GAS

      accel_due_to_pitch = 0.0
      if len(CC.orientationNED) == 3:
        accel_due_to_pitch = math.sin(CC.orientationNED[1]) * ACCELERATION_DUE_TO_GRAVITY

      accel_pitch_compensated = accel + accel_due_to_pitch
      if accel_pitch_compensated > 0.3 or not CC.longActive:
        self.brake_request = False
      elif accel_pitch_compensated < 0.0:
        self.brake_request = True

      stopping = CC.actuators.longControlState == LongCtrlState.stopping
      can_sends.append(fordcan.create_acc_msg(self.packer, self.CAN, CC.longActive, gas, accel, stopping, self.brake_request, v_ego_kph=V_CRUISE_MAX))

      self.accel = accel
      self.gas = gas

    ### ui ###
    send_ui = (self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or (self.steer_alert_last != steer_alert)
    if (self.frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_lkas_ui_msg(self.packer, self.CAN, main_on, CC.latActive, steer_alert, hud_control, CS.lkas_status_stock_values))

    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      send_ui = True
      self.distance_bar_frame = self.frame

    if (self.frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      show_distance_bars = self.frame - self.distance_bar_frame < 400
      can_sends.append(fordcan.create_acc_ui_msg(self.packer, self.CAN, self.CP, main_on, CC.latActive,
                                                 fcw_alert, CS.out.cruiseState.standstill, show_distance_bars,
                                                 hud_control, CS.acc_tja_status_stock_values))

    self.main_on_last = main_on
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert
    self.lead_distance_bars_last = hud_control.leadDistanceBars

    new_actuators = actuators.as_builder()
    new_actuators.curvature = self.apply_curvature_last
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    self.frame += 1
    return new_actuators, can_sends
