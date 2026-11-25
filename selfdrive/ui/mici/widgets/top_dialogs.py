"""
TOP dialogs for the comma four (mici) UI: dashcam footage QR code and the error log.
"""
import time

import pyray as rl

from openpilot.selfdrive.ui.widgets.footage_dialog import footage_url, make_qr_texture
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.scroll_panel2 import GuiScrollPanel2
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
