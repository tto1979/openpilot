"""
TOP dialogs for the comma four (mici) UI: option wheel picker, dashcam footage QR code and the error log.
"""
import time
from collections.abc import Callable
from typing import cast

import pyray as rl

from openpilot.selfdrive.ui.mici.widgets.dialog import BigDialogBase
from openpilot.selfdrive.ui.widgets.footage_dialog import footage_url, make_qr_texture
from openpilot.system.ui.lib.application import gui_app, FontWeight, MouseEvent, MousePos
from openpilot.system.ui.lib.scroll_panel2 import ScrollState
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.nav_widget import NavWidget
from openpilot.system.ui.widgets.html_render import HtmlRenderer, ElementType
from openpilot.system.ui.widgets.label import UnifiedLabel
from openpilot.system.ui.widgets.scroller import _Scroller, NavRawScrollPanel

SIDE_BUTTON_WIDTH, SIDE_BUTTON_HEIGHT = 112, 240


CARD_VALUE_WIDTH = 402 - 2 * 40  # BigButton width minus label padding
CARD_VALUE_SIZE = 36               # BigButton COMPLICATION_SIZE


def elide_value(text: str, width: int = CARD_VALUE_WIDTH) -> str:
  """One line card value: BigButton wraps long values into the title otherwise."""
  font = gui_app.font(FontWeight.ROMAN)
  if measure_text_cached(font, text, CARD_VALUE_SIZE).x <= width:
    return text
  while text and measure_text_cached(font, text + "...", CARD_VALUE_SIZE).x > width:
    text = text[:-1]
  return text.rstrip() + "..."


class FootageDialogMici(NavWidget):
  """QR code to the fleetmanager footage page, laid out like the mici pairing dialog."""
  REFRESH_INTERVAL = 30  # s, the IP can change when switching networks

  def __init__(self):
    super().__init__()
    self._qr_texture: rl.Texture | None = None
    self._url = ""
    self._last_refresh = float("-inf")
    self._title = UnifiedLabel("scan to watch dashcam footage", 40, font_weight=FontWeight.BOLD,
                               text_color=rl.Color(255, 255, 255, int(255 * 0.9)), line_height=0.9, wrap_text=True)
    self._hint = UnifiedLabel("", 24, font_weight=FontWeight.ROMAN, text_color=rl.Color(255, 255, 255, int(255 * 0.5)), wrap_text=True)

  def _refresh(self):
    url = footage_url()
    if url == self._url and self._hint.text:
      return
    self._url = url
    self._unload()
    if url:
      # white on black like the pairing QR code
      self._qr_texture = make_qr_texture(url, fill_color="white", back_color="black", border=0)
      self._hint.set_text(url.replace("http://", "") + "\nphone on the same network")
    else:
      self._hint.set_text("connect to Wi-Fi or your phone's hotspot first")

  def _unload(self):
    if self._qr_texture is not None and self._qr_texture.id != 0:
      rl.unload_texture(self._qr_texture)
    self._qr_texture = None

  def _render(self, rect: rl.Rectangle) -> int:
    now = time.monotonic()
    if now - self._last_refresh >= self.REFRESH_INTERVAL:
      self._refresh()
      self._last_refresh = now

    qr_size = self._rect.height - 16
    if self._qr_texture is not None:
      rl.draw_texture_pro(self._qr_texture, rl.Rectangle(0, 0, self._qr_texture.width, self._qr_texture.height),
                          rl.Rectangle(self._rect.x + 8, self._rect.y + 8, qr_size, qr_size), rl.Vector2(0, 0), 0, rl.WHITE)
    else:
      rl.draw_rectangle_rounded(rl.Rectangle(self._rect.x + 8, self._rect.y + 8, qr_size, qr_size), 0.1, 10, rl.Color(40, 40, 40, 255))

    label_x = self._rect.x + 8 + qr_size + 24
    width = int(self._rect.x + self._rect.width - label_x - 8)
    self._title.set_max_width(width)
    self._title.set_position(label_x, self._rect.y + 16)
    self._title.render()
    self._hint.set_max_width(width)
    self._hint.set_position(label_x, self._rect.y + self._rect.height - 80)
    self._hint.render()
    return -1

  def __del__(self):
    self._unload()


class TextModalMici(NavRawScrollPanel):
  """Scrollable text (HTML) shown full screen, swipe down at the top to close. Used for the error log."""

  def __init__(self, text: str):
    super().__init__()
    self._content = HtmlRenderer(text=text, text_size={ElementType.P: 22})

  def _render(self, rect: rl.Rectangle):
    margin = 16
    content_rect = rl.Rectangle(rect.x + margin, rect.y, rect.width - 2 * margin, rect.height)
    height = self._content.get_total_height(int(content_rect.width)) + 2 * margin
    offset = round(self._scroll_panel.update(content_rect, height))
    self._content.render(rl.Rectangle(content_rect.x, content_rect.y + margin + offset, content_rect.width, height))
    return -1


class SideCheckButton(Widget):
  """Check button on the right edge of the option picker (upstream removed its side button in v0.11.0)."""

  def __init__(self):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, SIDE_BUTTON_WIDTH, SIDE_BUTTON_HEIGHT))
    self._txt_btn = gui_app.texture("icons_mici/buttons/button_side_check.png", 100, 224)
    self._txt_btn_pressed = gui_app.texture("icons_mici/buttons/button_side_check_pressed.png", 100, 224)

  def _render(self, _):
    x = self._rect.x + 12
    y = self._rect.y + (self._rect.height - self._txt_btn.height) / 2
    rl.draw_texture_ex(self._txt_btn_pressed if self.is_pressed else self._txt_btn, rl.Vector2(x, y), 0, 1.0, rl.WHITE)


class _VerticalSnapScroller(_Scroller):
  """Vertical scroller that snaps the closest row to the center when released. Upstream's only snaps horizontally
  since v0.11.1 (the snap moved into GuiScrollPanel2, which handles either direction)."""
  def __init__(self, items: list[Widget], **kwargs):
    super().__init__(items, horizontal=False, snap_items=False, **kwargs)

  def _get_scroll(self, visible_items: list[Widget], content_size: float) -> float:
    scroll_enabled = self._scroll_enabled() if callable(self._scroll_enabled) else self._scroll_enabled
    self.scroll_panel.set_enabled(scroll_enabled and self.enabled and not self._scrolling_to[1])

    snap_target: float | None = None
    if visible_items and self._scrolling_to[0] is None:
      center_pos = self._rect.y + self._rect.height / 2
      closest_delta_pos = min((((item.rect.y + item.rect.height / 2) - center_pos) for item in visible_items), key=abs)
      snap_target = self.scroll_panel.get_offset() - closest_delta_pos

    return self.scroll_panel.update(self._rect, content_size, snap_target=snap_target)


class BigDialogOptionButton(Widget):
  """One row of the option wheel (upstream's BigDialogOptionButton, removed in v0.11.0)."""
  HEIGHT = 64
  SELECTED_HEIGHT = 74

  def __init__(self, option: str):
    super().__init__()
    self.option = option
    self.set_rect(rl.Rectangle(0, 0, int(gui_app.width / 2 + 220), self.HEIGHT))
    self._selected = False
    self._label = UnifiedLabel(option, font_size=70, text_color=rl.Color(255, 255, 255, int(255 * 0.58)),
                               font_weight=FontWeight.DISPLAY_REGULAR, alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_MIDDLE,
                               scroll=True)

  def show_event(self):
    super().show_event()
    self._label.reset_scroll()

  def set_selected(self, selected: bool):
    self._selected = selected
    self._rect.height = self.SELECTED_HEIGHT if selected else self.HEIGHT

  def _render(self, _):
    if self._selected:
      self._label.set_font_size(self.SELECTED_HEIGHT)
      self._label.set_color(rl.Color(255, 255, 255, int(255 * 0.9)))
      self._label.set_font_weight(FontWeight.DISPLAY)
    else:
      self._label.set_font_size(self.HEIGHT)
      self._label.set_color(rl.Color(255, 255, 255, int(255 * 0.58)))
      self._label.set_font_weight(FontWeight.DISPLAY_REGULAR)
    self._label.render(self._rect)


class BigMultiOptionDialog(BigDialogBase):
  """
  Vertical option wheel with a check button (upstream's BigMultiOptionDialog, removed in v0.11.0), on the nav stack:
  push it with gui_app.push_widget, the check button dismisses it and then calls right_btn_callback, swiping down cancels.
  """
  BACK_TOUCH_AREA_PERCENTAGE = 0.1

  def __init__(self, options: list[str], default: str | None, right_btn_callback: Callable[[], None] | None = None):
    super().__init__()
    if default is not None:
      assert default in options
    self._options = options
    self._default_option: str | None = default
    self._selected_option: str = self._default_option or (options[0] if len(options) > 0 else "")
    self._last_selected_option: str = self._selected_option

    # Widget doesn't differentiate between click and drag
    self._can_click = True

    self._right_btn = self._child(SideCheckButton())
    self._right_btn.set_click_callback(lambda: self.dismiss(right_btn_callback))
    self._right_btn.set_enabled(lambda: self.enabled and not self.is_dismissing)

    self._scroller = self._child(_VerticalSnapScroller([], pad=100, spacing=0))
    self._scroller.set_enabled(lambda: self.enabled and not self.is_dismissing and not self._right_btn.is_pressed)
    self._scroller.add_widgets(self._make_buttons(options))
    self._recenter_pending = False

  def _make_buttons(self, options: list[str]) -> list[Widget]:
    return [BigDialogOptionButton(option) for option in options]

  def show_event(self):
    super().show_event()
    if self._default_option is not None:
      self._center_option(self._default_option)
    self._recenter_pending = True

  def _center_option(self, option: str):
    """Scroll the option to the center from the row heights: the rows aren't laid out yet when the dialog is pushed."""
    btns = self._option_buttons()
    idx = next((i for i, btn in enumerate(btns) if btn.option == option), None)
    if idx is None:
      return
    for i, btn in enumerate(btns):
      btn.set_selected(i == idx)
    btn_cls = type(btns[idx])
    offset = self._rect.height / 2 - self._scroller._pad - idx * (btn_cls.HEIGHT + self._scroller._spacing) - btn_cls.SELECTED_HEIGHT / 2
    self._scroller.scroll_panel.set_offset(offset)
    self._selected_option = option

  def get_selected_option(self) -> str:
    return self._selected_option

  def _option_buttons(self) -> list[BigDialogOptionButton]:
    return cast(list[BigDialogOptionButton], self._scroller.items)

  def _on_option_selected(self, option: str):
    y_pos = 0.0
    btn_cls = type(self._option_buttons()[0]) if self._option_buttons() else BigDialogOptionButton
    for btn in self._option_buttons():
      if btn.option == option:
        rect_center_y = self._rect.y + self._rect.height / 2
        if btn._selected:
          height = btn.rect.height
        else:
          # when selecting an option under current, account for changing heights
          btn_center_y = btn.rect.y + btn.rect.height / 2  # not accurate, just to determine direction
          height_offset = btn_cls.SELECTED_HEIGHT - btn_cls.HEIGHT
          height = (btn_cls.HEIGHT - height_offset) if rect_center_y < btn_center_y else btn_cls.SELECTED_HEIGHT
        y_pos = rect_center_y - (btn.rect.y + height / 2)
        break

    self._scroller.scroll_to(-y_pos)

  def _handle_mouse_press(self, mouse_pos: MousePos):
    super()._handle_mouse_press(mouse_pos)
    self._can_click = True

  def _handle_mouse_event(self, mouse_event: MouseEvent) -> None:
    super()._handle_mouse_event(mouse_event)
    if not self._scroller.scroll_panel.is_touch_valid():
      self._can_click = False

  def _handle_mouse_release(self, mouse_pos: MousePos):
    super()._handle_mouse_release(mouse_pos)
    if not self._can_click or self.is_dismissing:
      return
    # tap re-centers the selected option
    for btn in self._option_buttons():
      if btn.option == self._selected_option:
        self._on_option_selected(btn.option)
        break

  def _update_state(self):
    super()._update_state()

    # selection is whichever row is closest to the center, frozen once dismissing: the check button's callback
    # runs after the slide out, while the lagging rows would drift the selection to the next one
    if self.is_dismissing:
      return
    center_y = self._rect.y + self._rect.height / 2
    closest: tuple[BigDialogOptionButton | None, float] = (None, float('inf'))
    for btn in self._option_buttons():
      dist_y = abs((btn.rect.y + btn.rect.height / 2) - center_y)
      if dist_y < closest[1]:
        closest = (btn, dist_y)

    if closest[0] is not None:
      for btn in self._option_buttons():
        btn.set_selected(btn.option == closest[0].option)
      self._selected_option = closest[0].option
    self._last_selected_option = self._selected_option

    # while sliding in, the rows lag the dialog by a frame and the snap pulls the wheel towards the wrong row:
    # center the default again once the slide in is over, unless the user scrolled
    if self._recenter_pending and abs(self._y_pos_filter.x) < 1:
      self._recenter_pending = False
      if self._default_option is not None and self._scroller.scroll_panel.state == ScrollState.STEADY:
        self._center_option(self._default_option)

  def _render(self, _):
    self._scroller.render(self._rect)
    self._right_btn.set_position(self._rect.x + self._rect.width - self._right_btn.rect.width, self._rect.y)
    self._right_btn.render()


class CompactOptionButton(BigDialogOptionButton):
  """Smaller option row for long names (car models, countries), kept clear of the check button."""
  HEIGHT = 40
  SELECTED_HEIGHT = 46
  LEFT = 16

  def __init__(self, option: str, display: str | None = None):
    super().__init__(option)
    self.set_rect(rl.Rectangle(0, 0, int(gui_app.width / 2 + 220), self.HEIGHT))
    if display is not None:
      self._label.set_text(display)

  def _render(self, _):
    # the scroller centers rows horizontally, draw the text from the left edge up to the check button instead
    rect = self._rect
    self._rect = rl.Rectangle(self.LEFT, rect.y, gui_app.width - SIDE_BUTTON_WIDTH - self.LEFT - 8, rect.height)
    super()._render(_)
    self._rect = rect


class CompactOptionDialog(BigMultiOptionDialog):
  """BigMultiOptionDialog with CompactOptionButton rows. `display` maps an option to the text shown for it."""

  def __init__(self, options: list[str], default: str | None, right_btn_callback: Callable[[], None] | None = None,
               display: dict[str, str] | None = None):
    self._display = display or {}
    super().__init__(options, default, right_btn_callback=right_btn_callback)

  def _make_buttons(self, options: list[str]) -> list[Widget]:
    return [CompactOptionButton(option, self._display.get(option)) for option in options]
