from cereal import car
from common.numpy_fast import clip
from selfdrive.car import apply_std_steer_torque_limits
from opendbc.can.packer import CANPacker
from selfdrive.car.ford import fordcan
from selfdrive.car.ford.values import CANBUS, CarControllerParams

VisualAlert = car.CarControl.HUDControl.VisualAlert


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.VM = VM
    self.packer = CANPacker(dbc_name)
    self.frame = 0

    self.apply_steer_last = 0
    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False

  def update(self, CC, CS):
    can_sends = []

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)

    ### acc buttons ###
    if CC.cruiseControl.cancel:
      can_sends.append(fordcan.create_button_command(self.packer, CS.buttons_stock_values, cancel=True))
    elif CC.cruiseControl.resume:
      can_sends.append(fordcan.create_button_command(self.packer, CS.buttons_stock_values, resume=True))
    elif CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0:
      # if stock lane centering is active or in standby, toggle it off
      # the stock system checks for steering pressed, and eventually disengages cruise control
      can_sends.append(fordcan.create_button_command(self.packer, CS.buttons_stock_values, tja_toggle=True, bus=CANBUS.camera))


    ### lateral control ###
    if CC.latActive:
      new_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX))
      apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, CarControllerParams)
    else:
      apply_steer = actuators.steer * CarControllerParams.STEER_MAX

    # send steering commands at 20Hz
    if (self.frame % CarControllerParams.LKAS_STEER_STEP) == 0:
      lca_rq = 1 if CC.latActive else 0

      # use LatCtlPath_An_Actl to actuate steering
      # path angle is the car wheel angle, not the steering wheel angle
      path_angle = clip(apply_steer * CarControllerParams.TORQUE_RATIO, -0.4995, 0.5240)

      # ramp rate: 0=Slow, 1=Medium, 2=Fast, 3=Immediately
      # TODO: try slower ramp speed when driver torque detected
      ramp_type = 3
      precision = 1  # 0=Comfortable, 1=Precise (the stock system always uses comfortable)

      self.apply_steer_last = apply_steer
      can_sends.append(fordcan.create_lka_command(self.packer, 0, 0))
      can_sends.append(fordcan.create_tja_command(self.packer, lca_rq, ramp_type, precision,
                                                  0, path_angle, 0, 0))


    ### ui ###
    send_ui = (self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or (self.steer_alert_last != steer_alert)

    # send lkas ui command at 1Hz or if ui state changes
    if (self.frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_lkas_ui_command(self.packer, main_on, CC.latActive, steer_alert, CS.lkas_status_stock_values))

    # send acc ui command at 20Hz or if ui state changes
    if (self.frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_acc_ui_command(self.packer, main_on, CC.latActive, CS.acc_tja_status_stock_values))

    self.main_on_last = main_on
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer

    self.frame += 1
    return new_actuators, can_sends
