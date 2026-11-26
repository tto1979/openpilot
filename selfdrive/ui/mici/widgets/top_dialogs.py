"""
TOP dialogs for the comma four (mici) UI: dashcam footage QR code and the error log.
"""
import time
from collections.abc import Callable
from typing import cast

import pyray as rl

from openpilot.selfdrive.ui.mici.widgets.dialog import BigDialogOptionButton, BigMultiOptionDialog
from openpilot.selfdrive.ui.mici.widgets.side_button import WIDTH as SIDE_BUTTON_WIDTH
from openpilot.selfdrive.ui.widgets.footage_dialog import footage_url, make_qr_texture
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.scroll_panel2 import GuiScrollPanel2, ScrollState
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import NavWidget
from openpilot.system.ui.widgets.html_render import HtmlRenderer, ElementType
from openpilot.system.ui.widgets.label import MiciLabel


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
    self.set_back_callback(lambda: gui_app.set_modal_overlay(None))
    self._qr_texture: rl.Texture | None = None
    self._url = ""
    self._last_refresh = float("-inf")
    self._title = MiciLabel("scan to watch dashcam footage", 40, font_weight=FontWeight.BOLD,
                            color=rl.Color(255, 255, 255, int(255 * 0.9)), line_height=36, wrap_text=True)
    self._hint = MiciLabel("", 24, font_weight=FontWeight.ROMAN, color=rl.Color(255, 255, 255, int(255 * 0.5)), wrap_text=True)

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
    self._title.set_width(width)
    self._title.set_position(label_x, self._rect.y + 16)
    self._title.render()
    self._hint.set_width(width)
    self._hint.set_position(label_x, self._rect.y + self._rect.height - 80)
    self._hint.render()
    return -1

  def __del__(self):
    self._unload()


class TextModalMici(NavWidget):
  """Scrollable text (HTML) shown full screen, swipe down to close. Used for the error log."""

  def __init__(self, text: str):
    super().__init__()
    self.set_back_callback(lambda: gui_app.set_modal_overlay(None))
    self._content = HtmlRenderer(text=text, text_size={ElementType.P: 22})
    self._scroll_panel = GuiScrollPanel2(horizontal=False)

  def _render(self, rect: rl.Rectangle):
    margin = 16
    content_rect = rl.Rectangle(rect.x + margin, rect.y, rect.width - 2 * margin, rect.height)
    height = self._content.get_total_height(int(content_rect.width)) + 2 * margin
    offset = round(self._scroll_panel.update(content_rect, height))
    self._content.render(rl.Rectangle(content_rect.x, content_rect.y + margin + offset, content_rect.width, height))
    return -1


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
    super().__init__(options, default, right_btn_callback=right_btn_callback)
    self._scroller._items.clear()
    for option in options:
      self._scroller.add_widget(CompactOptionButton(option, (display or {}).get(option)))
    self._recenter_pending = False

  def show_event(self):
    super().show_event()
    self._recenter_pending = True

  def _update_state(self):
    super()._update_state()
    # while sliding in, the rows lag the dialog by a frame and the snap pulls the wheel towards the wrong row
    # (more than half of a compact row): center the default once the slide in is over, unless the user scrolled
    if self._recenter_pending and abs(self._pos_filter.x) < 1:
      self._recenter_pending = False
      if self._default_option is not None and self._scroller.scroll_panel.state == ScrollState.STEADY:
        self._on_option_selected(self._default_option)

  def _on_option_selected(self, option: str):
    # same as BigMultiOptionDialog, with the compact row heights
    y_pos = 0.0
    for btn in self._scroller._items:
      btn = cast(CompactOptionButton, btn)
      if btn.option == option:
        rect_center_y = self._rect.y + self._rect.height / 2
        if btn._selected:
          height = btn.rect.height
        else:
          btn_center_y = btn.rect.y + btn.rect.height / 2
          height_offset = CompactOptionButton.SELECTED_HEIGHT - CompactOptionButton.HEIGHT
          height = (CompactOptionButton.HEIGHT - height_offset) if rect_center_y < btn_center_y else CompactOptionButton.SELECTED_HEIGHT
        y_pos = rect_center_y - (btn.rect.y + height / 2)
        break
    self._scroller.scroll_to(-y_pos)
