"""
QR code to the fleetmanager footage page (raylib port of TOP's Qt MyFootagePopup).
"""
import time

import numpy as np
import pyray as rl
import qrcode

from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import HARDWARE
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.wrap_text import wrap_text
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.button import IconButton

FLEETMANAGER_PORT = 8082


class FootageDialog(Widget):
  REFRESH_INTERVAL = 30  # s, the IP can change when switching networks

  def __init__(self):
    super().__init__()
    self._qr_texture: rl.Texture | None = None
    self._url = ""
    self._last_refresh = float('-inf')
    self._close_btn = IconButton(gui_app.texture("icons/close.png", 80, 80))
    self._close_btn.set_click_callback(lambda: gui_app.set_modal_overlay(None))

  def _refresh(self):
    ip = HARDWARE.get_ip_address() or ""
    url = f"http://{ip}:{FLEETMANAGER_PORT}/footage" if ip and ip != "--" else ""
    if url == self._url and self._qr_texture is not None:
      return
    self._url = url
    self._unload()
    if not url:
      return

    try:
      qr = qrcode.QRCode(version=1, error_correction=qrcode.constants.ERROR_CORRECT_L, box_size=10, border=4)
      qr.add_data(url)
      qr.make(fit=True)
      pil_img = qr.make_image(fill_color="black", back_color="white").convert('RGBA')
      img_array = np.array(pil_img, dtype=np.uint8)

      rl_image = rl.Image()
      rl_image.data = rl.ffi.cast("void *", img_array.ctypes.data)
      rl_image.width = pil_img.width
      rl_image.height = pil_img.height
      rl_image.mipmaps = 1
      rl_image.format = rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8
      self._qr_texture = rl.load_texture_from_image(rl_image)
    except Exception:
      cloudlog.exception("footage QR code generation failed")
      self._qr_texture = None

  def _unload(self):
    if self._qr_texture is not None and self._qr_texture.id != 0:
      rl.unload_texture(self._qr_texture)
    self._qr_texture = None

  def _render(self, rect: rl.Rectangle) -> int:
    rl.clear_background(rl.Color(224, 224, 224, 255))

    now = time.monotonic()
    if now - self._last_refresh >= self.REFRESH_INTERVAL:
      self._refresh()
      self._last_refresh = now

    margin = 70
    content = rl.Rectangle(rect.x + margin, rect.y + margin, rect.width - 2 * margin, rect.height - 2 * margin)
    y = content.y

    close_size, pad = 80, 20
    self._close_btn.render(rl.Rectangle(content.x - pad, y - pad, close_size + pad * 2, close_size + pad * 2))
    y += close_size + 40

    left_width = int(content.width * 0.5 - 15)
    font = gui_app.font(FontWeight.NORMAL)
    title = wrap_text(font, tr("Watch and/or download recordings from comma device cameras"), 75, left_width)
    rl.draw_text_ex(font, "\n".join(title), rl.Vector2(content.x, y), 75, 0.0, rl.BLACK)
    y += len(title) * 75 + 60

    bold = gui_app.font(FontWeight.BOLD)
    if self._url:
      hint = tr("Scan the QR code with a phone on the same network as the device, or open:") + "\n" + self._url
    else:
      hint = tr("Connect the device to Wi-Fi (or your phone's hotspot) to watch the recordings.")
    hint_lines = wrap_text(bold, hint, 47, left_width)
    rl.draw_text_ex(bold, "\n".join(hint_lines), rl.Vector2(content.x, y), 47, 0.0, rl.BLACK)

    right_width = content.width // 2 - 20
    qr_size = min(right_width, content.height) - 40
    qr_rect = rl.Rectangle(content.x + left_width + 40 + (right_width - qr_size) // 2, content.y, qr_size, qr_size)
    if self._qr_texture is not None:
      rl.draw_texture_pro(self._qr_texture, rl.Rectangle(0, 0, self._qr_texture.width, self._qr_texture.height),
                          qr_rect, rl.Vector2(0, 0), 0, rl.WHITE)
    else:
      rl.draw_rectangle_rounded(qr_rect, 0.1, 20, rl.Color(240, 240, 240, 255))
    return -1

  def __del__(self):
    self._unload()
