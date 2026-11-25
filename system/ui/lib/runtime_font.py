"""
TOP: draw text with characters the prebuilt font atlases don't have (road names from mapd).

The UI fonts are atlases baked at build time with the characters of the translations only. RuntimeFont loads
the glyphs a given text needs from a TTF/OTF on the fly and keeps the font until the text changes.
"""
from importlib.resources import as_file

import pyray as rl
from openpilot.system.ui.lib.application import FONT_DIR

ASCII = range(32, 127)


class RuntimeFont:
  def __init__(self, font_file: str = "OpFont-SemiBold.otf", size: int = 64):
    self._font_file = font_file
    self._size = size
    self._text: str | None = None
    self._font: rl.Font | None = None

  def get(self, text: str, fallback: rl.Font) -> rl.Font:
    """A font with all characters of `text` (plus ASCII), `fallback` if the font file can't be loaded."""
    if text != self._text:
      self._unload()
      self._text = text
      codepoints = sorted(set(ASCII) | {ord(c) for c in text})
      cp_buffer = rl.ffi.new("int[]", codepoints)
      cp = rl.ffi.cast("int *", cp_buffer)
      with as_file(FONT_DIR.joinpath(self._font_file)) as path:
        if path.exists():
          font = rl.load_font_ex(path.as_posix(), self._size, cp, len(codepoints))
          if font.texture.id != 0:
            rl.set_texture_filter(font.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
            self._font = font
    return self._font if self._font is not None else fallback

  def _unload(self):
    if self._font is not None:
      rl.unload_font(self._font)
      self._font = None

  def __del__(self):
    try:
      self._unload()
    except Exception:
      pass
