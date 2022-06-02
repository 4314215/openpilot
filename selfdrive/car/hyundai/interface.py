#!/usr/bin/env python3
from cereal import car
from panda import Panda
from common.conversions import Conversions as CV
from selfdrive.car.hyundai.values import CAR, DBC, EV_CAR, HYBRID_CAR, LEGACY_SAFETY_MODE_CAR, Buttons, CarControllerParams
from selfdrive.car.hyundai.radar_interface import RADAR_START_ADDR
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.disable_ecu import disable_ecu

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[], disable_radar=False):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "hyundai"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundai, 0)]
    ret.radarOffCan = RADAR_START_ADDR not in fingerprint[1] or DBC[ret.carFingerprint]["radar"] is None

    # WARNING: disabling radar also disables AEB (and we show the same warning on the instrument cluster as if you manually disabled AEB)
    ret.openpilotLongitudinalControl = disable_radar and (candidate not in LEGACY_SAFETY_MODE_CAR)

    ret.pcmCruise = not ret.openpilotLongitudinalControl

    # These cars have been put into dashcam only due to both a lack of users and test coverage.
    # These cars likely still work fine. Once a user confirms each car works and a test route is
    # added to selfdrive/car/tests/routes.py, we can remove it from this list.
    ret.dashcamOnly = candidate in {CAR.KIA_OPTIMA_H, CAR.ELANTRA_GT_I30}

    ret.steerActuatorDelay = 0.1  # Default delay
    ret.steerRateCost = 0.5
    ret.steerLimitTimer = 0.4
    tire_stiffness_factor = 1.

    ret.stoppingControl = True
    ret.vEgoStopping = 1.0

    ret.longitudinalTuning.kpV = [1.0]
    ret.longitudinalTuning.kiV = [0.0]
    ret.stopAccel = 0.0

    ret.longitudinalActuatorDelayUpperBound = 1.0 # s

    ret.lateralTuning.pid.kf = 0.00005
    ret.mass = 3982. * CV.LB_TO_KG + STD_CARGO_KG
    ret.wheelbase = 2.766
    # Values from optimizer
    ret.steerRatio = 16.55  # 13.8 is spec end-to-end
    tire_stiffness_factor = 0.82
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[8.941, 20.12, 24.59, 29.06, 40.24], [8.941, 20.12, 24.59, 29.06, 40.24]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.05, 0.1, 0.2, 0.32, 0.35], [0.001, 0.005, 0.05, 0.07, 0.09]]
    
    ret.centerToFront = ret.wheelbase * 0.4

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.enableBsm = 0x58b in fingerprint[0]

    if ret.openpilotLongitudinalControl:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_HYUNDAI_LONG

    return ret

  @staticmethod
  def init(CP, logcan, sendcan):
    if CP.openpilotLongitudinalControl:
      disable_ecu(logcan, sendcan, addr=0x7d0, com_cont_req=b'\x28\x83\x01')

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    events = self.create_common_events(ret, pcm_enable=self.CS.CP.pcmCruise)

    if self.CS.brake_error:
      events.add(EventName.brakeUnavailable)

    if self.CS.CP.openpilotLongitudinalControl:
      buttonEvents = []

      if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
        be = car.CarState.ButtonEvent.new_message()
        be.type = ButtonType.unknown
        if self.CS.cruise_buttons != 0:
          be.pressed = True
          but = self.CS.cruise_buttons
        else:
          be.pressed = False
          but = self.CS.prev_cruise_buttons
        if but == Buttons.RES_ACCEL:
          be.type = ButtonType.accelCruise
        elif but == Buttons.SET_DECEL:
          be.type = ButtonType.decelCruise
        elif but == Buttons.GAP_DIST:
          be.type = ButtonType.gapAdjustCruise
        elif but == Buttons.CANCEL:
          be.type = ButtonType.cancel
        buttonEvents.append(be)

        ret.buttonEvents = buttonEvents

        for b in ret.buttonEvents:
          # do enable on both accel and decel buttons
          if b.type in (ButtonType.accelCruise, ButtonType.decelCruise) and not b.pressed:
            events.add(EventName.buttonEnable)
          # do disable on button down
          if b.type == ButtonType.cancel and b.pressed:
            events.add(EventName.buttonCancel)

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + 2.) and self.CP.minSteerSpeed > 10.:
      self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 4.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    return ret

  def apply(self, c):
    ret = self.CC.update(c, self.CS)
    return ret
