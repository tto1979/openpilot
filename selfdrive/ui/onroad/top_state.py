"""
TOP onroad state shared by the comma 3/3X (selfdrive/ui/onroad/top_hud.py) and comma four
(selfdrive/ui/mici/onroad/top_hud.py) HUDs: speed limit, smart cruise control, blinkers / blind spot
and a few car states, read from the TOP messages. Mixed into the HUD renderers, no drawing here.
"""
from cereal import custom
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr

AssistState = custom.LongitudinalPlanTOP.SpeedLimit.AssistState
SpeedLimitSource = custom.LongitudinalPlanTOP.SpeedLimit.Source

SPEED_LIMIT_MODE_OFF = 0
SPEED_LIMIT_MODE_WARNING = 2
SPEED_LIMIT_AHEAD_VALID_FRAME_THRESHOLD = 5

METER_TO_FOOT = 3.28084
METER_TO_MILE = 0.000621371


def nearbyint(v: float) -> int:
  # std::nearbyint: round half to even like Qt, avoids flicker at .5 with python's round()
  return int(round(v))


class TopOnroadState:
  """Mixin for the HUD renderers: call _init_top_state() in __init__ and _update_top_state() in _update_state()."""

  speed: float  # from the HUD renderer

  def _init_top_state(self):
    self.brake_lights = False
    self.long_override = False
    self.right_hand_dm = False
    self.hide_bottom_icons = False
    self.road_name = ""
    self.personality = 1

    # speed limit
    self.speed_limit = 0.0
    self.speed_limit_last = 0.0
    self.speed_limit_offset = 0.0
    self.speed_limit_valid = False
    self.speed_limit_last_valid = False
    self.speed_limit_final_last = 0.0
    self.speed_limit_source = SpeedLimitSource.none
    self.speed_limit_assist_state = AssistState.disabled
    self.speed_limit_assist_active = False
    self.speed_limit_ahead_valid = False
    self.speed_limit_ahead = 0.0
    self.speed_limit_ahead_distance = 0.0
    self._speed_limit_ahead_distance_prev = 0.0
    self._speed_limit_ahead_valid_frame = 0
    self._speed_limit_assist_frame = 0

    # smart cruise control
    self.scc_vision_enabled = False
    self.scc_vision_active = False
    self.scc_map_enabled = False
    self.scc_map_active = False
    self._scc_vision_frame = 0
    self._scc_map_frame = 0

    # turn signals / blind spot
    self.turn_signal_left = False
    self.turn_signal_right = False
    self.blind_spot_left = False
    self.blind_spot_right = False

  def _update_top_state(self) -> None:
    sm = ui_state.sm

    ss = sm['selfdriveState']
    self.hide_bottom_icons = sm.recv_frame['selfdriveState'] > ui_state.started_frame and ss.alertSize != 0
    self.personality = int(ss.personality.raw)

    lmd = sm['liveMapDataTOP']
    self.road_name = lmd.roadName if sm.alive['liveMapDataTOP'] and sm.recv_frame['liveMapDataTOP'] > 0 else ""

    if sm.recv_frame['carState'] < ui_state.started_frame:
      self.brake_lights = False
      self.turn_signal_left = self.turn_signal_right = False
      self.blind_spot_left = self.blind_spot_right = False
      return

    car_state = sm['carState']
    self.brake_lights = car_state.brakeLights
    self.turn_signal_left = car_state.leftBlinker
    self.turn_signal_right = car_state.rightBlinker
    self.blind_spot_left = car_state.leftBlindspot
    self.blind_spot_right = car_state.rightBlindspot
    self.long_override = sm['carControl'].cruiseControl.override
    self.right_hand_dm = sm['driverMonitoringState'].isRHD

    conv = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    lp_top = sm['longitudinalPlanTOP']
    resolver = lp_top.speedLimit.resolver
    assist = lp_top.speedLimit.assist
    self.speed_limit = resolver.speedLimit * conv
    self.speed_limit_last = resolver.speedLimitLast * conv
    self.speed_limit_offset = resolver.speedLimitOffset * conv
    self.speed_limit_valid = resolver.speedLimitValid
    self.speed_limit_last_valid = resolver.speedLimitLastValid
    self.speed_limit_final_last = resolver.speedLimitFinalLast * conv
    self.speed_limit_source = resolver.source.raw
    self.speed_limit_assist_state = assist.state.raw
    self.speed_limit_assist_active = assist.active

    scc = lp_top.smartCruiseControl
    self.scc_vision_enabled = scc.vision.enabled
    self.scc_vision_active = scc.vision.active
    self.scc_map_enabled = scc.map.enabled
    self.scc_map_active = scc.map.active

    if sm.updated['liveMapDataTOP']:
      self.speed_limit_ahead_valid = lmd.speedLimitAheadValid
      self.speed_limit_ahead = lmd.speedLimitAhead * conv
      self.speed_limit_ahead_distance = lmd.speedLimitAheadDistance
      if self.speed_limit_ahead_distance < self._speed_limit_ahead_distance_prev and \
         self._speed_limit_ahead_valid_frame < SPEED_LIMIT_AHEAD_VALID_FRAME_THRESHOLD:
        self._speed_limit_ahead_valid_frame += 1
      elif self.speed_limit_ahead_distance > self._speed_limit_ahead_distance_prev and self._speed_limit_ahead_valid_frame > 0:
        self._speed_limit_ahead_valid_frame -= 1
      self._speed_limit_ahead_distance_prev = self.speed_limit_ahead_distance

  # *** helpers for drawing ***

  @staticmethod
  def _pulse(frame: int) -> bool:
    """Blink at 1 Hz, on 60% of the time."""
    fps = max(gui_app.target_fps, 1)
    return bool(frame % fps >= fps / 2.5)

  def has_speed_limit(self) -> bool:
    return bool(self.speed_limit_valid or self.speed_limit_last_valid)

  def speed_limit_overspeed(self) -> bool:
    return self.has_speed_limit() and nearbyint(self.speed_limit_final_last) < nearbyint(self.speed)

  def speed_limit_text(self) -> str:
    return str(nearbyint(self.speed_limit_last)) if self.has_speed_limit() else "---"

  def speed_limit_offset_text(self) -> str:
    offset = nearbyint(self.speed_limit_offset)
    if offset == 0 or not self.has_speed_limit():
      return ""
    return ("" if self.speed_limit_offset > 0 else "-") + str(abs(offset))

  def speed_limit_stale(self) -> bool:
    """Showing the last known limit, the current one isn't valid."""
    return bool(not self.speed_limit_valid and self.speed_limit_last_valid)

  def show_speed_limit_ahead(self) -> bool:
    return bool(self.speed_limit_ahead_valid and self.speed_limit_ahead > 0 and self.speed_limit_ahead != self.speed_limit and
                self._speed_limit_ahead_valid_frame > 0 and self.speed_limit_source == SpeedLimitSource.map)

  def _ahead_distance_text(self) -> str:
    d = self.speed_limit_ahead_distance

    def round_to(distance: float, interval: int, threshold: int) -> int:
      base = int(distance / interval) * interval
      return base + interval if distance - base >= threshold else base

    if ui_state.is_metric:
      if d < 50:
        return tr("Near")
      if d >= 1000:
        return f"{d / 1000:.1f}" + tr("km")
      rounded = max(10, round_to(d, 10, 5)) if d < 200 else round_to(d, 100, 50)
      return f"{rounded}" + tr("m")
    ft = d * METER_TO_FOOT
    if ft < 100:
      return tr("Near")
    if ft >= 900:
      return f"{d * METER_TO_MILE:.1f}" + tr("mi")
    rounded = max(50, round_to(ft, 50, 25)) if ft < 500 else round_to(ft, 100, 50)
    return f"{rounded}" + tr("ft")
