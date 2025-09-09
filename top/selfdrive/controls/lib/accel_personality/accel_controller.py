"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import custom
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params

from openpilot.top.selfdrive.controls.lib.accel_personality.accel_profiles import (
  get_max_accel_hermite,
  get_min_accel_hermite
)


AccelPersonality = custom.LongitudinalPlanTOP.AccelerationPersonality

class AccelController:
  def __init__(self):
    self.params = Params()
    self.personality = AccelPersonality.stock
    self.frame = 0
    self.accel_profile_init = False
    self.prev_car_accel_profile = None

  def update_from_carstate(self, carstate):
    if (hasattr(carstate, 'toyota_drive_mode') and carstate.toyota_drive_mode and
        hasattr(carstate, 'accel_profile') and carstate.accel_profile is not None):

        if (not self.accel_profile_init or
            carstate.accel_profile != self.prev_car_accel_profile):

          self.params.put_nonblocking('AccelPersonality', int(carstate.accel_profile))
          self.accel_profile_init = True
          self.prev_car_accel_profile = carstate.accel_profile

  def _update_personality_from_param(self):
    if self.frame % int(1. / DT_MDL) == 0:
      personality_int = self.params.get("AccelPersonality")
      if personality_int is not None:
        if personality_int in [AccelPersonality.stock, AccelPersonality.normal, AccelPersonality.eco, AccelPersonality.sport]:
          self.personality = personality_int

  def _get_max_accel_for_speed(self, v_ego: float) -> float:
    self._update_personality_from_param()

    if self.personality == AccelPersonality.eco:
      mode = "eco"
    elif self.personality == AccelPersonality.sport:
      mode = "sport"
    else:
      mode = "normal"

    return get_max_accel_hermite(v_ego, mode)

  def _get_min_accel_for_speed(self, v_ego: float) -> float:
    self._update_personality_from_param()

    if self.personality == AccelPersonality.eco:
      mode = "eco"
    elif self.personality == AccelPersonality.sport:
      mode = "sport"
    elif self.personality == AccelPersonality.normal:
      mode = "normal"
    else:
      mode = "stock"

    return get_min_accel_hermite(v_ego, mode)

  def get_accel_limits(self, v_ego: float, accel_limits: list[float]) -> tuple[float, float]:
    self._update_personality_from_param()

    if self.personality == AccelPersonality.stock:
      return (accel_limits[0], accel_limits[1])
    else:
      max_accel = self._get_max_accel_for_speed(v_ego)
      min_accel = self._get_min_accel_for_speed(v_ego)
      return (min_accel, max_accel)

  def is_personality_enabled(self, accel_personality: int = AccelPersonality.stock) -> bool:
    self.personality = accel_personality
    self._update_personality_from_param()
    return bool(self.personality != AccelPersonality.stock)

  def update(self, carstate=None):
    self.frame += 1

    if carstate is not None:
      self.update_from_carstate(carstate)

    self._update_personality_from_param()
