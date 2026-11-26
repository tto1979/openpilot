"""
Small list_view extensions for TOP's settings panels.
"""
from collections.abc import Callable

import pyray as rl

from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets.list_view import ItemAction, ListItem, BUTTON_HEIGHT, RIGHT_ITEM_PADDING, _resolve_value

STEP_BUTTON_WIDTH = 150
VALUE_WIDTH = 260


class ValueStepAction(ItemAction):
  """[-] value [+] stepper, like the Qt OptionControl TOP used for the speed limit offset."""

  def __init__(self, get_value: Callable[[], int], set_value: Callable[[int], None], min_value: int, max_value: int,
               step: int = 1, label: str | Callable[[], str] = "", enabled: bool | Callable[[], bool] = True):
    super().__init__(width=STEP_BUTTON_WIDTH * 2 + VALUE_WIDTH + RIGHT_ITEM_PADDING * 2, enabled=enabled)
    self._get_value = get_value
    self._set_value = set_value
    self._min = min_value
    self._max = max_value
    self._step = step
    self._label = label
    self._font = gui_app.font(FontWeight.MEDIUM)

  def _button_rects(self, rect: rl.Rectangle) -> tuple[rl.Rectangle, rl.Rectangle, rl.Rectangle]:
    y = rect.y + (rect.height - BUTTON_HEIGHT) / 2
    minus = rl.Rectangle(rect.x, y, STEP_BUTTON_WIDTH, BUTTON_HEIGHT)
    value = rl.Rectangle(minus.x + STEP_BUTTON_WIDTH + RIGHT_ITEM_PADDING, y, VALUE_WIDTH, BUTTON_HEIGHT)
    plus = rl.Rectangle(value.x + VALUE_WIDTH + RIGHT_ITEM_PADDING, y, STEP_BUTTON_WIDTH, BUTTON_HEIGHT)
    return minus, value, plus

  def _render(self, rect: rl.Rectangle):
    minus, value_rect, plus = self._button_rects(rect)
    mouse = rl.get_mouse_position()
    value = self._get_value()
    for btn, text, usable in ((minus, "-", value > self._min), (plus, "+", value < self._max)):
      pressed = self.is_pressed and rl.check_collision_point_rec(mouse, btn)
      color = rl.Color(74, 74, 74, 255) if pressed else rl.Color(57, 57, 57, 255)
      if not (self.enabled and usable):
        color = rl.Color(color.r, color.g, color.b, 120)
      rl.draw_rectangle_rounded(btn, 1.0, 20, color)
      size = measure_text_cached(self._font, text, 60)
      rl.draw_text_ex(self._font, text, rl.Vector2(btn.x + (btn.width - size.x) / 2, btn.y + (btn.height - size.y) / 2), 60, 0, rl.WHITE)

    text = f"{value}{_resolve_value(self._label, '')}"
    size = measure_text_cached(self._font, text, 50)
    rl.draw_text_ex(self._font, text, rl.Vector2(value_rect.x + (value_rect.width - size.x) / 2,
                                                 value_rect.y + (value_rect.height - size.y) / 2), 50, 0, rl.WHITE)

  def _handle_mouse_release(self, mouse_pos: MousePos):
    if not self.enabled:
      return
    minus, _, plus = self._button_rects(self._rect)
    value = self._get_value()
    if rl.check_collision_point_rec(mouse_pos, minus):
      self._set_value(max(self._min, value - self._step))
    elif rl.check_collision_point_rec(mouse_pos, plus):
      self._set_value(min(self._max, value + self._step))


def value_step_item(title: str | Callable[[], str], description: str | Callable[[], str] | None,
                    get_value: Callable[[], int], set_value: Callable[[int], None], min_value: int, max_value: int,
                    step: int = 1, label: str | Callable[[], str] = "", enabled: bool | Callable[[], bool] = True) -> ListItem:
  action = ValueStepAction(get_value, set_value, min_value, max_value, step, label, enabled)
  return ListItem(title=title, description=description, action_item=action)
