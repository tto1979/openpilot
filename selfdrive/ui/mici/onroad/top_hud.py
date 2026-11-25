"""TOP additions to the comma four (mici) onroad HUD, sized for the small screen.

- speed limit sign top right (round sign in metric, US style in imperial) with the offset bubble,
  red when over the limit in warning mode, the upcoming limit below it and the speed limit assist arrow
- V-TSC / M-TSC chips next to the sign, blinking while they slow the car down
- blinker (green) / blind spot (yellow) strips on the left and right edge
- steering wheel icon in teal while lateral only (ALKA) is active, upstream hides it when disengaged

State comes from TopOnroadState, shared with the comma 3/3X HUD.
"""
import math
import time

import pyray as rl
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.onroad.top_state import TopOnroadState, AssistState, SPEED_LIMIT_MODE_OFF, SPEED_LIMIT_MODE_WARNING, \
  nearbyint
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached

MARGIN = 14
SIGN_SIZE = 64            # metric sign diameter, imperial sign height
SIGN_WIDTH_IMPERIAL = 52
OFFSET_SIZE = 28
AHEAD_HEIGHT = 44
CHIP_WIDTH = 70
CHIP_HEIGHT = 24

RED = rl.Color(230, 20, 20, 255)
BLACK = rl.Color(0, 0, 0, 255)
GREEN = rl.Color(0, 255, 0, 255)
OVERRIDE_GREY = rl.Color(0x91, 0x9b, 0x95, 0xf1)
ALKA_TEAL = rl.Color(0x6f, 0xc0, 0xc9, 230)

EDGE_WIDTH = 26
BLINKER_COLOR = rl.Color(0, 255, 0, 200)
BSM_COLOR = rl.Color(255, 220, 0, 220)
BLINK_STD = 0.4   # s
BLINK_FAST = 0.2  # s, blinker towards a car in the blind spot


class _EdgeBlink:
  def __init__(self):
    self._start = 0.0

  def color(self, blinker: bool, bsm: bool) -> "rl.Color | None":
    now = time.monotonic()
    if not blinker and not bsm:
      self._start = now
      return None
    if not blinker:
      return BSM_COLOR
    period = BLINK_FAST if bsm else BLINK_STD
    if int((now - self._start) / period) % 2 == 0:
      return None
    return BSM_COLOR if bsm else BLINKER_COLOR


class TopHudRendererMici(TopOnroadState, HudRenderer):
  def __init__(self):
    super().__init__()
    self._init_top_state()
    self._font_display_regular = gui_app.font(FontWeight.DISPLAY_REGULAR)
    self._edge_left = _EdgeBlink()
    self._edge_right = _EdgeBlink()

  def _update_state(self) -> None:
    super()._update_state()
    self._update_top_state()

  def _render(self, rect: rl.Rectangle) -> None:
    self._draw_edges(rect)
    super()._render(rect)

    # alerts cover the top of the screen
    if self._can_draw_top_icons and self.is_cruise_available:
      sign_rect = self._draw_speed_limit(rect)
      self._draw_scc_chips(rect, sign_rect)

  # *** helpers ***

  def _text_in_rect(self, font: rl.Font, text: str, size: float, r: rl.Rectangle, color: rl.Color) -> None:
    sz = measure_text_cached(font, text, int(size))
    rl.draw_text_ex(font, text, rl.Vector2(r.x + (r.width - sz.x) / 2, r.y + (r.height - sz.y) / 2), size, 0, color)

  # *** blinker / blind spot ***

  def _draw_edges(self, rect: rl.Rectangle) -> None:
    left = self._edge_left.color(self.turn_signal_left, self.blind_spot_left)
    right = self._edge_right.color(self.turn_signal_right, self.blind_spot_right)
    if left is not None:
      rl.draw_rectangle_gradient_h(int(rect.x), int(rect.y), EDGE_WIDTH, int(rect.height), left, rl.BLANK)
    if right is not None:
      rl.draw_rectangle_gradient_h(int(rect.x + rect.width - EDGE_WIDTH), int(rect.y), EDGE_WIDTH, int(rect.height), rl.BLANK, right)

  # *** ALKA steering wheel ***

  def _draw_steering_wheel(self, rect: rl.Rectangle) -> None:
    if not (ui_state.alka_active and ui_state.status == UIStatus.DISENGAGED and not self._show_wheel_critical):
      super()._draw_steering_wheel(rect)
      return

    wheel_txt = self._txt_wheel
    self._wheel_alpha_filter.update(255 * 0.9)
    self._wheel_y_filter.update(0)
    pos_x = int(rect.x + 21 + wheel_txt.width / 2)
    pos_y = int(rect.y + rect.height - 14 - wheel_txt.height / 2 + self._wheel_y_filter.x)

    turn_intent_margin = 25
    self._turn_intent.render(rl.Rectangle(pos_x - wheel_txt.width / 2 - turn_intent_margin, pos_y - wheel_txt.height / 2 - turn_intent_margin,
                                          wheel_txt.width + turn_intent_margin * 2, wheel_txt.height + turn_intent_margin * 2))
    color = rl.Color(ALKA_TEAL.r, ALKA_TEAL.g, ALKA_TEAL.b, int(self._wheel_alpha_filter.x))
    rl.draw_texture_pro(wheel_txt, rl.Rectangle(0, 0, wheel_txt.width, wheel_txt.height),
                        rl.Rectangle(pos_x, pos_y, wheel_txt.width, wheel_txt.height),
                        rl.Vector2(wheel_txt.width / 2, wheel_txt.height / 2), -ui_state.sm['carState'].steeringAngleDeg, color)

  # *** speed limit ***

  def _draw_speed_limit(self, rect: rl.Rectangle) -> "rl.Rectangle | None":
    width = SIGN_SIZE if ui_state.is_metric else SIGN_WIDTH_IMPERIAL
    sign_rect = rl.Rectangle(rect.x + rect.width - MARGIN - width, rect.y + MARGIN, width, SIGN_SIZE)

    if self.speed_limit_assist_state == AssistState.preActive:
      self._speed_limit_assist_frame += 1
      show = self._pulse(self._speed_limit_assist_frame)
      self._draw_assist_arrow(sign_rect)
    else:
      self._speed_limit_assist_frame = 0
      show = ui_state.speed_limit_mode != SPEED_LIMIT_MODE_OFF

    if not show:
      return sign_rect if self.speed_limit_assist_state == AssistState.preActive else None

    warning = ui_state.speed_limit_mode >= SPEED_LIMIT_MODE_WARNING
    if warning and self.speed_limit_overspeed():
      text_color = RED
    elif self.speed_limit_stale():
      text_color = OVERRIDE_GREY
    else:
      text_color = BLACK
    text = self.speed_limit_text()

    if ui_state.is_metric:
      center = rl.Vector2(sign_rect.x + width / 2, sign_rect.y + SIGN_SIZE / 2)
      rl.draw_circle_v(center, SIGN_SIZE / 2, RED)
      rl.draw_circle_v(center, SIGN_SIZE / 2 * 0.76, rl.WHITE)
      self._text_in_rect(self._font_bold, text, 22 if len(text) >= 3 else 28, sign_rect, text_color)
    else:
      rl.draw_rectangle_rounded(sign_rect, 0.3, 8, rl.WHITE)
      inner = rl.Rectangle(sign_rect.x + 3, sign_rect.y + 3, sign_rect.width - 6, sign_rect.height - 6)
      rl.draw_rectangle_rounded_lines_ex(inner, 0.3, 8, 2, BLACK)
      self._text_in_rect(self._font_semi_bold, "LIMIT", 12, rl.Rectangle(inner.x, inner.y + 4, inner.width, 12), BLACK)
      self._text_in_rect(self._font_bold, text, 22 if len(text) >= 3 else 26, rl.Rectangle(inner.x, inner.y + 14, inner.width, inner.height - 14),
                         text_color)

    offset = self.speed_limit_offset_text()
    if offset:
      bubble = rl.Vector2(sign_rect.x + 4, sign_rect.y + 6)
      rl.draw_circle_v(bubble, OFFSET_SIZE / 2 + 2, rl.Color(77, 77, 77, 255))
      rl.draw_circle_v(bubble, OFFSET_SIZE / 2, BLACK)
      self._text_in_rect(self._font_bold, offset, 13 if len(offset) >= 3 else 15,
                         rl.Rectangle(bubble.x - OFFSET_SIZE / 2, bubble.y - OFFSET_SIZE / 2, OFFSET_SIZE, OFFSET_SIZE), rl.WHITE)

    if self.speed_limit_assist_state != AssistState.preActive and self.show_speed_limit_ahead():
      ahead = rl.Rectangle(sign_rect.x + width / 2 - 34, sign_rect.y + SIGN_SIZE + 6, 68, AHEAD_HEIGHT)
      rl.draw_rectangle_rounded(ahead, 0.3, 8, rl.Color(0, 0, 0, 180))
      self._text_in_rect(self._font_bold, str(nearbyint(self.speed_limit_ahead)), 22, rl.Rectangle(ahead.x, ahead.y + 2, ahead.width, 24), rl.WHITE)
      self._text_in_rect(self._font_medium, self._ahead_distance_text(), 14, rl.Rectangle(ahead.x, ahead.y + 25, ahead.width, 16),
                         rl.Color(190, 190, 190, 255))
    return sign_rect

  def _draw_assist_arrow(self, sign_rect: rl.Rectangle) -> None:
    set_speed = self.set_speed * (1.0 if ui_state.is_metric else 0.621371)
    target = nearbyint(self.speed_limit_final_last)
    if nearbyint(set_speed) == target:
      return
    up = nearbyint(set_speed) < target
    fps = max(gui_app.target_fps, 1)
    bounce = 5 * math.sin(self._speed_limit_assist_frame * 2.0 * math.pi / fps)
    cx = sign_rect.x - 22
    cy = sign_rect.y + SIGN_SIZE / 2 + (bounce if up else -bounce)
    color = rl.Color(255, 255, 255, 230)
    if up:
      rl.draw_triangle(rl.Vector2(cx, cy - 12), rl.Vector2(cx - 11, cy + 8), rl.Vector2(cx + 11, cy + 8), color)
    else:
      rl.draw_triangle(rl.Vector2(cx - 11, cy - 8), rl.Vector2(cx, cy + 12), rl.Vector2(cx + 11, cy - 8), color)

  # *** smart cruise control ***

  def _draw_scc_chips(self, rect: rl.Rectangle, sign_rect: "rl.Rectangle | None") -> None:
    if sign_rect is None:
      right = rect.x + rect.width - MARGIN
    else:
      # clear of the offset bubble on the sign's top left corner and the assist arrow
      right = sign_rect.x - (44 if self.speed_limit_assist_state == AssistState.preActive else 22)
    y = rect.y + MARGIN
    chips = (("V-TSC", self.scc_vision_enabled, self.scc_vision_active, "_scc_vision_frame"),
             ("M-TSC", self.scc_map_enabled, self.scc_map_active, "_scc_map_frame"))
    for text, enabled, active, frame_attr in chips:
      frame = getattr(self, frame_attr)
      setattr(self, frame_attr, frame + 1 if active else 0)
      if not enabled:
        continue
      if not active or self._pulse(frame):
        chip = rl.Rectangle(right - CHIP_WIDTH, y, CHIP_WIDTH, CHIP_HEIGHT)
        rl.draw_rectangle_rounded(chip, 0.4, 8, OVERRIDE_GREY if self.long_override else GREEN)
        self._text_in_rect(self._font_bold, text, 16, chip, rl.Color(0, 0, 0, 220))
      y += CHIP_HEIGHT + 6
