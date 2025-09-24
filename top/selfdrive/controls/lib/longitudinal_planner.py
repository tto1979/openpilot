"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import messaging, custom
from openpilot.top.selfdrive.controls.lib.accel_personality.accel_controller import AccelController
from openpilot.top.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl

Source = custom.LongitudinalPlanTOP.LongitudinalPlanSource
class LongitudinalPlannerTOP:
  def __init__(self):
    self.accel_controller = AccelController()
    self.scc = SmartCruiseControl()
    self.source = Source.cruise

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    self.scc.update(sm, v_ego, a_ego, v_cruise)

    targets = {
      Source.cruise: (v_cruise, a_ego),
      Source.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
    }

    self.source = min(targets, key=lambda k: targets[k][0])
    v_target, a_target = targets[self.source]

    return v_target, a_target

  def update(self, sm: messaging.SubMaster) -> None:
    if hasattr(sm, 'updated') and sm.updated['carState']:
      carstate = sm['carState']
      self.accel_controller.update(carstate)
    else:
      self.accel_controller.update()

  def publish_longitudinal_plan_top(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_top_send = messaging.new_message('longitudinalPlanTOP')

    plan_top_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanTOP = plan_top_send.longitudinalPlanTOP
    longitudinalPlanTOP.longitudinalPlanSource = self.source
    # Smart Cruise Control
    smartCruiseControl = longitudinalPlanTOP.smartCruiseControl
    # Vision Control
    sccVision = smartCruiseControl.vision
    sccVision.state = self.scc.vision.state
    sccVision.vTarget = float(self.scc.vision.output_v_target)
    sccVision.aTarget = float(self.scc.vision.output_a_target)
    sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
    sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
    sccVision.enabled = self.scc.vision.is_enabled
    sccVision.active = self.scc.vision.is_active

    plan_top_send.longitudinalPlanTOP.accelPersonality = self.accel_controller.personality
    pm.send('longitudinalPlanTOP', plan_top_send)
