"""
Settings for the opview app, in dashy's /api/settings format (dragonpilot), so one app works on both.

Same settings and rules as the device's own settings pages:
- TOP panel: TOP_TOGGLES (+ car model), with apply_toggle's side effects (NNFF)
- common openpilot toggles, driving / acceleration personality
- speed limit settings

Item fields (dashy): key, type, title, description, current_value, options, min_val, max_val, step, suffix.
TOP adds: enabled (false = read only right now), disabled_reason, icon.
Types: toggle_item (bool), text_spin_button_item (int index into options), spin_button_item (int, min_val..max_val),
select_item (TOP: string, one of options, for the car model).
"""
import os
import threading
from dataclasses import dataclass, field
from typing import Any

from cereal import car
from openpilot.common.params import Params
from openpilot.selfdrive.ui.layouts.settings.top_common import TOP_TOGGLES, DISENGAGED_ONLY, NOT_SELECTED, apply_toggle, \
  lag_learning_locked, load_car_list
from openpilot.system.ui.lib.multilang import tr_noop
from openpilot.top.system.opview_api.i18n import Translator

LITE = os.getenv("LITE") is not None

# turning these off from the phone would cut the phone off (webrtcd stops with opview)
READ_ONLY = {"dp_dev_opview"}

# openpilot toggles (selfdrive/ui/layouts/settings/toggles.py): param -> (title, description, icon, needs restart)
OPENPILOT_TOGGLES: dict[str, tuple[str, str, str, bool]] = {
  "OpenpilotEnabledToggle": (tr_noop("Enable openpilot"), tr_noop(
    "Use the openpilot system for adaptive cruise control and lane keep driver assistance. " +
    "Your attention is required at all times to use this feature."), "chffr_wheel.png", True),
  "ExperimentalMode": (tr_noop("Experimental Mode"), "", "experimental_white.png", False),
  "DisengageOnAccelerator": (tr_noop("Disengage on Accelerator Pedal"),
                             tr_noop("When enabled, pressing the accelerator pedal will disengage openpilot."),
                             "disengage_on_accelerator.png", False),
  "IsLdwEnabled": (tr_noop("Enable Lane Departure Warnings"), tr_noop(
    "Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line " +
    "without a turn signal activated while driving over 31 mph (50 km/h)."), "warning.png", False),
  "AlwaysOnDM": (tr_noop("Always-On Driver Monitoring"), tr_noop("Enable driver monitoring even when openpilot is not engaged."),
                 "monitoring.png", False),
  "RecordFront": (tr_noop("Record and Upload Driver Camera"),
                  tr_noop("Upload data from the driver facing camera and help improve the driver monitoring algorithm."),
                  "monitoring.png", True),
  "RecordAudio": (tr_noop("Record and Upload Microphone Audio"), tr_noop(
    "Record and store microphone audio while driving. The audio will be included in the dashcam video in comma connect."),
                  "microphone.png", True),
  "IsMetric": (tr_noop("Use Metric System"), tr_noop("Display speed in km/h instead of mph."), "metric.png", False),
}
LITE_HIDDEN = {"AlwaysOnDM", "RecordFront", "RecordAudio"}  # comma three without driver camera / microphone

LONG_PERSONALITY_DESC = tr_noop(
  "Standard is recommended. In aggressive mode, openpilot will follow lead cars closer and be more aggressive with the gas and brake. " +
  "In relaxed mode openpilot will stay further away from lead cars. On supported cars, you can cycle through these personalities with " +
  "your steering wheel distance button.")
ACCEL_PERSONALITY_DESC = tr_noop(
  "Normal is recommended. In sport mode, TOP will provide aggressive acceleration for a dynamic driving experience. " +
  "In eco mode, TOP will apply smoother and more relaxed acceleration. On supported cars, you can cycle through these " +
  "acceleration personality within Onroad Settings on the driving screen.")
SPEED_LIMIT_MODE_DESC = [
  tr_noop("Off: Disables the Speed Limit functions."),
  tr_noop("Information: Displays the current road's speed limit."),
  tr_noop("Warning: Provides a warning when exceeding the current road's speed limit."),
  tr_noop("Assist: Adjusts the vehicle's cruise speed based on the current road's speed limit when operating the +/- buttons."),
]
SPEED_LIMIT_SOURCE_DESC = [
  tr_noop("⦿ Car Only: Use Speed Limit data only from Car"),
  tr_noop("⦿ Map Only: Use Speed Limit data only from OpenStreetMaps"),
  tr_noop("⦿ Car First: Use Speed Limit data from Car if available, else use from OpenStreetMaps"),
  tr_noop("⦿ Map First: Use Speed Limit data from OpenStreetMaps if available, else use from Car"),
  tr_noop("⦿ Combined: Use combined Speed Limit data from Car & OpenStreetMaps"),
]
SPEED_LIMIT_OFFSET_DESC = [
  tr_noop("None: No Offset"),
  tr_noop("Fixed: Adds a fixed offset [Speed Limit + Offset]"),
  tr_noop("Percent: Adds a percent offset [Speed Limit + (Offset % Speed Limit)]"),
]

ONROAD_LOCKED_REASON = tr_noop("Change this on the device, or after the car is off.")
LONG_UNAVAILABLE_REASON = tr_noop("Experimental mode is currently unavailable on this car since the car's stock ACC is used for longitudinal control.")
EXP_CONFIRM_REASON = tr_noop("Turn on Experimental Mode once on the device to confirm its description first.")
LOCKED_BY_NNFF_REASON = tr_noop("Always on with NNFF Torque Control.")


class SettingError(Exception):
  def __init__(self, status: int, message: str):
    super().__init__(message)
    self.status = status
    self.message = message


@dataclass
class Setting:
  key: str
  type: str
  title: str
  description: str = ""
  icon: str = ""
  options: list[str] = field(default_factory=list)
  min_val: int = 0
  max_val: int = 0
  step: int = 1
  default: Any = None
  needs_restart: bool = False  # restarts openpilot onroad: not while driving, OnroadCycleRequested


def _long_available(params: Params) -> bool | None:
  """openpilot longitudinal control, like ui_state.has_longitudinal_control. None before the car was ever seen."""
  cp_bytes = params.get("CarParamsPersistent")
  if not cp_bytes:
    return None
  with car.CarParams.from_bytes(cp_bytes) as CP:
    if CP.alphaLongitudinalAvailable:
      return params.get_bool("AlphaLongitudinalEnabled")
    return bool(CP.openpilotLongitudinalControl)


class SettingsAPI:
  def __init__(self, params: Params | None = None):
    self.params = params or Params()
    self._cars: list[str] | None = None
    self._cars_lock = threading.Lock()

  # *** schema ***

  def _cars_list(self) -> list[str]:
    with self._cars_lock:
      if not self._cars:
        self._cars = load_car_list()  # generates the list on first use (subprocess), keep it once it worked
      return self._cars

  def sections(self) -> list[tuple[str, list[Setting]]]:
    top = [Setting("CarModel", "select_item", tr_noop("Car Model"), icon="road.png", default=NOT_SELECTED)]
    top += [Setting(param, "toggle_item", title, desc, icon, needs_restart=param in DISENGAGED_ONLY)
            for param, title, desc, icon in TOP_TOGGLES]

    openpilot = []
    for param, (title, desc, icon, restart) in OPENPILOT_TOGGLES.items():
      if LITE and param in LITE_HIDDEN:
        continue
      openpilot.append(Setting(param, "toggle_item", title, desc, icon, needs_restart=restart))
      if param == "DisengageOnAccelerator":
        openpilot.append(Setting("LongitudinalPersonality", "text_spin_button_item", tr_noop("Driving Personality"), LONG_PERSONALITY_DESC,
                                 "speed_limit.png", [tr_noop("Aggressive"), tr_noop("Standard"), tr_noop("Relaxed")], default=1))
        openpilot.append(Setting("AccelPersonality", "text_spin_button_item", tr_noop("Acceleration Personality"), ACCEL_PERSONALITY_DESC,
                                 "speed_limit.png", [tr_noop("Sport"), tr_noop("Normal"), tr_noop("Eco"), tr_noop("Stock")], default=3))

    speed_limit = [
      Setting("SpeedLimitMode", "text_spin_button_item", tr_noop("Speed Limit Mode"), "<br>".join(SPEED_LIMIT_MODE_DESC),
              options=[tr_noop("Off"), tr_noop("Info."), tr_noop("Warning"), tr_noop("Assist")], default=1),
      Setting("SpeedLimitPolicy", "text_spin_button_item", tr_noop("Speed Limit Source"), "<br>".join(SPEED_LIMIT_SOURCE_DESC),
              options=[tr_noop("Car Only"), tr_noop("Map Only"), tr_noop("Car First"), tr_noop("Map First"), tr_noop("Combined")], default=3),
      Setting("SpeedLimitOffsetType", "text_spin_button_item", tr_noop("Speed Limit Offset"), "<br>".join(SPEED_LIMIT_OFFSET_DESC),
              options=[tr_noop("None"), tr_noop("Fixed"), tr_noop("Percent")], default=0),
      Setting("SpeedLimitValueOffset", "spin_button_item", tr_noop("Offset"), min_val=-30, max_val=30, step=1, default=0),
    ]
    return [("TOP", top), (tr_noop("Toggles"), openpilot), (tr_noop("Speed Limit"), speed_limit)]

  def _settings(self) -> dict[str, Setting]:
    return {s.key: s for _, items in self.sections() for s in items}

  # *** values and rules ***

  def value(self, s: Setting) -> Any:
    p = self.params
    if s.type == "toggle_item":
      return p.get_bool(s.key)
    if s.type == "select_item":
      return p.get(s.key, return_default=True) or s.default
    v = p.get(s.key, return_default=True)
    if s.key == "AccelPersonality" and v not in (0, 1, 2, 3):
      return 3  # unset is stock, same as the device's settings page
    try:
      return int(v) if v is not None else s.default
    except (TypeError, ValueError):
      return s.default

  def disabled_reason(self, s: Setting) -> str | None:
    """None when it can be changed from the phone right now, else why not (untranslated)"""
    p = self.params
    if s.key in READ_ONLY:
      return ""
    try:
      if p.get_bool(s.key + "Lock"):
        return ""
    except Exception:
      pass
    if s.needs_restart and p.get_bool("IsOnroad"):
      return ONROAD_LOCKED_REASON
    if s.key == "DisableLateralLagLearning" and lag_learning_locked(p):
      return LOCKED_BY_NNFF_REASON
    if s.key in ("ExperimentalMode", "LongitudinalPersonality") and _long_available(p) is False:
      return LONG_UNAVAILABLE_REASON
    return None

  def schema(self, tr: Translator) -> dict:
    sections = []
    for title, items in self.sections():
      out = []
      for s in items:
        item: dict[str, Any] = {"key": s.key, "type": s.type, "title": tr(s.title), "description": tr(s.description).replace("\n", "<br>"),
                                "current_value": self.value(s)}
        if s.icon:
          item["icon"] = s.icon
        if s.type == "text_spin_button_item":
          item["options"] = [tr(o) for o in s.options]
        elif s.type == "spin_button_item":
          item.update(min_val=s.min_val, max_val=s.max_val, step=s.step)
          if s.key == "SpeedLimitValueOffset":
            offset_type = self.value(self._settings()["SpeedLimitOffsetType"])
            item["suffix"] = "%" if offset_type == 2 else (" km/h" if self.params.get_bool("IsMetric") else " mph")
        elif s.type == "select_item":
          item["options"] = [NOT_SELECTED, *self._cars_list()]
        reason = self.disabled_reason(s)
        item["enabled"] = reason is None
        if reason:
          item["disabled_reason"] = tr(reason)
        out.append(item)
      sections.append({"title": tr(title), "settings": out})
    return {"settings": sections}

  def get(self, key: str) -> Any:
    s = self._settings().get(key)
    if s is None:
      raise SettingError(403, "Unknown param")
    return self.value(s)

  def put(self, key: str, value: Any) -> Any:
    """Validate and write one setting with the same side effects as the device's settings pages. Returns the new value."""
    s = self._settings().get(key)
    if s is None:
      raise SettingError(403, "Unknown param")
    reason = self.disabled_reason(s)
    if reason is not None:
      raise SettingError(409, reason or "Read-only param")

    p = self.params
    if s.type == "toggle_item":
      if not isinstance(value, bool):
        raise SettingError(400, "value must be true or false")
      if key == "ExperimentalMode":
        self._put_experimental_mode(value)
      elif key in OPENPILOT_TOGGLES:
        p.put_bool(key, value, block=True)
        if s.needs_restart:
          p.put_bool("OnroadCycleRequested", True, block=True)
      else:
        apply_toggle(p, key, value, block=True)
    elif s.type in ("text_spin_button_item", "spin_button_item"):
      if isinstance(value, bool) or not isinstance(value, int):
        raise SettingError(400, "value must be an integer")
      lo, hi = (0, len(s.options) - 1) if s.type == "text_spin_button_item" else (s.min_val, s.max_val)
      if not lo <= value <= hi:
        raise SettingError(400, f"value must be between {lo} and {hi}")
      p.put(key, value, block=True)
    elif s.type == "select_item":
      if not isinstance(value, str) or value not in (NOT_SELECTED, *self._cars_list()):
        raise SettingError(400, "unknown car model")
      p.put(key, value, block=True)
    return value

  def _put_experimental_mode(self, enabled: bool) -> None:
    p = self.params
    if enabled:
      if not _long_available(p):
        raise SettingError(409, LONG_UNAVAILABLE_REASON)
      if not p.get_bool("ExperimentalModeConfirmed"):
        raise SettingError(409, EXP_CONFIRM_REASON)
    p.put_bool("ExperimentalMode", enabled, block=True)
