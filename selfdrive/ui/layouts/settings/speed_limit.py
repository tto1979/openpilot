"""
Speed limit settings (raylib port of TOP's Qt SpeedLimitSettings / SpeedLimitPolicy, originally from sunnypilot).
"""
from collections.abc import Callable
from enum import IntEnum

import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.widgets.top_list import value_step_item
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import button_item, multiple_button_item
from openpilot.system.ui.widgets.network import NavButton
from openpilot.system.ui.widgets.scroller_tici import Scroller

BACK_BUTTON_HEIGHT = 100
BACK_BUTTON_SPACING = 30


class SpeedLimitMode(IntEnum):
  OFF = 0
  INFO = 1
  WARNING = 2
  ASSIST = 3


class SpeedLimitOffsetType(IntEnum):
  NONE = 0
  FIXED = 1
  PERCENT = 2


class SpeedLimitSourcePolicy(IntEnum):
  CAR_ONLY = 0
  MAP_ONLY = 1
  CAR_FIRST = 2
  MAP_FIRST = 3
  COMBINED = 4


MODE_DESCRIPTIONS: dict[int, str] = {
  SpeedLimitMode.OFF: tr_noop("Off: Disables the Speed Limit functions."),
  SpeedLimitMode.INFO: tr_noop("Information: Displays the current road's speed limit."),
  SpeedLimitMode.WARNING: tr_noop("Warning: Provides a warning when exceeding the current road's speed limit."),
  SpeedLimitMode.ASSIST: tr_noop("Assist: Adjusts the vehicle's cruise speed based on the current road's speed limit when operating the +/- buttons."),
}

OFFSET_DESCRIPTIONS: dict[int, str] = {
  SpeedLimitOffsetType.NONE: tr_noop("None: No Offset"),
  SpeedLimitOffsetType.FIXED: tr_noop("Fixed: Adds a fixed offset [Speed Limit + Offset]"),
  SpeedLimitOffsetType.PERCENT: tr_noop("Percent: Adds a percent offset [Speed Limit + (Offset % Speed Limit)]"),
}

SOURCE_DESCRIPTIONS: dict[int, str] = {
  SpeedLimitSourcePolicy.CAR_ONLY: tr_noop("⦿ Car Only: Use Speed Limit data only from Car"),
  SpeedLimitSourcePolicy.MAP_ONLY: tr_noop("⦿ Map Only: Use Speed Limit data only from OpenStreetMaps"),
  SpeedLimitSourcePolicy.CAR_FIRST: tr_noop("⦿ Car First: Use Speed Limit data from Car if available, else use from OpenStreetMaps"),
  SpeedLimitSourcePolicy.MAP_FIRST: tr_noop("⦿ Map First: Use Speed Limit data from OpenStreetMaps if available, else use from Car"),
  SpeedLimitSourcePolicy.COMBINED: tr_noop("⦿ Combined: Use combined Speed Limit data from Car & OpenStreetMaps"),
}


def _param_int(params: Params, key: str, default: int) -> int:
  try:
    return int(params.get(key, return_default=True))
  except (TypeError, ValueError):
    return default


class SubPage(Widget):
  """A settings sub page: back button on top, a Scroller below."""

  def __init__(self, items: list[Widget], back_callback: Callable[[], None]):
    super().__init__()
    self._back_btn = NavButton(tr("Back"))
    self._back_btn.set_click_callback(back_callback)
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def show_event(self):
    self._scroller.show_event()

  def _render(self, rect: rl.Rectangle):
    self._back_btn.set_position(rect.x, rect.y)
    self._back_btn.render()
    content = rl.Rectangle(rect.x, rect.y + BACK_BUTTON_HEIGHT + BACK_BUTTON_SPACING, rect.width,
                           rect.height - BACK_BUTTON_HEIGHT - BACK_BUTTON_SPACING)
    self._scroller.render(content)


class SpeedLimitPolicyLayout(SubPage):
  def __init__(self, back_callback: Callable[[], None]):
    self._params = Params()
    self._policy = multiple_button_item(
      lambda: tr("Speed Limit Source"), self._description,
      buttons=[lambda: tr("Car Only"), lambda: tr("Map Only"), lambda: tr("Car First"), lambda: tr("Map First"), lambda: tr("Combined")],
      button_width=230,
      selected_index=_param_int(self._params, "SpeedLimitPolicy", SpeedLimitSourcePolicy.MAP_FIRST),
      callback=lambda index: self._params.put("SpeedLimitPolicy", index),
    )
    super().__init__([self._policy], back_callback)

  def _description(self) -> str:
    current = _param_int(self._params, "SpeedLimitPolicy", SpeedLimitSourcePolicy.MAP_FIRST)
    lines = []
    for policy, text in SOURCE_DESCRIPTIONS.items():
      lines.append(f"<b>{tr(text)}</b>" if policy == current else tr(text))
    return "<br>".join(lines)

  def show_event(self):
    super().show_event()
    self._policy.action_item.set_selected_button(_param_int(self._params, "SpeedLimitPolicy", SpeedLimitSourcePolicy.MAP_FIRST))
    self._policy.description_visible = True


class SpeedLimitLayout(SubPage):
  def __init__(self, back_callback: Callable[[], None]):
    self._params = Params()
    self._policy_page = SpeedLimitPolicyLayout(lambda: self._show_policy(False))
    self._showing_policy = False

    self._mode = multiple_button_item(
      lambda: tr("Speed Limit Mode"), lambda: tr(MODE_DESCRIPTIONS.get(self._mode_value(), MODE_DESCRIPTIONS[SpeedLimitMode.OFF])),
      buttons=[lambda: tr("Off"), lambda: tr("Info."), lambda: tr("Warning"), lambda: tr("Assist")],
      button_width=220,
      selected_index=self._mode_value(),
      callback=lambda index: self._params.put("SpeedLimitMode", index),
    )
    self._source = button_item(lambda: tr("Speed Limit Source"), lambda: tr("CUSTOMIZE"),
                               lambda: tr("Choose where the speed limit comes from: the car, OpenStreetMaps or both."),
                               callback=lambda: self._show_policy(True))
    self._offset_type = multiple_button_item(
      lambda: tr("Speed Limit Offset"), lambda: tr(OFFSET_DESCRIPTIONS.get(self._offset_type_value(), OFFSET_DESCRIPTIONS[SpeedLimitOffsetType.NONE])),
      buttons=[lambda: tr("None"), lambda: tr("Fixed"), lambda: tr("Percent")],
      button_width=240,
      selected_index=self._offset_type_value(),
      callback=lambda index: self._params.put("SpeedLimitOffsetType", index),
    )
    self._offset = value_step_item(
      lambda: tr("Offset"), lambda: tr(OFFSET_DESCRIPTIONS.get(self._offset_type_value(), OFFSET_DESCRIPTIONS[SpeedLimitOffsetType.NONE])),
      get_value=lambda: _param_int(self._params, "SpeedLimitValueOffset", 0),
      set_value=lambda v: self._params.put("SpeedLimitValueOffset", v),
      min_value=-30, max_value=30, label=self._offset_unit,
    )
    self._offset.set_visible(lambda: self._offset_type_value() != SpeedLimitOffsetType.NONE)

    super().__init__([self._mode, self._source, self._offset_type, self._offset], back_callback)

  def _mode_value(self) -> int:
    return _param_int(self._params, "SpeedLimitMode", SpeedLimitMode.INFO)

  def _offset_type_value(self) -> int:
    return _param_int(self._params, "SpeedLimitOffsetType", SpeedLimitOffsetType.NONE)

  def _offset_unit(self) -> str:
    offset_type = self._offset_type_value()
    if offset_type == SpeedLimitOffsetType.PERCENT:
      return "%"
    if offset_type == SpeedLimitOffsetType.FIXED:
      return " km/h" if ui_state.is_metric else " mph"
    return ""

  def _show_policy(self, show: bool):
    self._showing_policy = show
    if show:
      self._policy_page.show_event()
    else:
      self.show_event()

  def show_event(self):
    super().show_event()
    self._mode.action_item.set_selected_button(self._mode_value())
    self._offset_type.action_item.set_selected_button(self._offset_type_value())

  def _render(self, rect: rl.Rectangle):
    if self._showing_policy:
      self._policy_page.render(rect)
    else:
      super()._render(rect)
