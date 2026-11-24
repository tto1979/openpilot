"""TOP additions to the onroad HUD (port of TOP's Qt hud.cc).

- brake light tint on the header, road name (mapd)
- speed limit sign (Vienna style in metric, MUTCD style in imperial) with offset bubble,
  upcoming speed limit box and the speed limit assist +/- arrows
- Smart Cruise Control V-TSC / M-TSC badges
- set speed colors for speed limit assist
- driving personality icon (DrivingPersonalitiesUIWheel)
- turn signal / blind spot animation (TimSignals)
"""
import math
import time

import pyray as rl
from cereal import custom
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.onroad.hud_renderer import HudRenderer, UI_CONFIG, FONT_SIZES, COLORS, CRUISE_DISABLED_CHAR
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.lib.text_measure import measure_text_cached

AssistState = custom.LongitudinalPlanTOP.SpeedLimit.AssistState
SpeedLimitSource = custom.LongitudinalPlanTOP.SpeedLimit.Source

SPEED_LIMIT_MODE_OFF = 0
SPEED_LIMIT_MODE_WARNING = 2
SPEED_LIMIT_AHEAD_VALID_FRAME_THRESHOLD = 5

METER_TO_FOOT = 3.28084
METER_TO_MILE = 0.000621371

SIGN_Y = 45
SIGN_HEIGHT = 204

PERSONALITY_IMAGES = ("aggressive.png", "standard.png", "relaxed.png")
PERSONALITY_NAMES = (tr_noop("Aggressive"), tr_noop("Standard"), tr_noop("Relaxed"))
PERSONALITY_TEXT_DURATION = 3.0
PERSONALITY_FADE_DURATION = 1.0

TURN_SIGNAL_SIZE = 142
TURN_SIGNAL_FRAME_TIME = 0.4  # s per animation frame

GREEN = rl.Color(0, 255, 0, 255)
OVERRIDE_GREY = rl.Color(0x91, 0x9b, 0x95, 0xf1)
RED = rl.Color(255, 0, 0, 255)
BLACK = rl.Color(0, 0, 0, 255)
OFFSET_BORDER = rl.Color(77, 77, 77, 255)
ROAD_NAME_COLOR = rl.Color(255, 215, 0, 255)
BRAKE_GRADIENT_START = rl.Color(255, 122, 128, 114)


def _round(v: float) -> int:
  # std::nearbyint: round half to even like Qt, avoids flicker at .5 with python's round()
  return int(round(v))


class TopHudRenderer(HudRenderer):
  def __init__(self):
    super().__init__()
    self._font_normal: rl.Font = gui_app.font(FontWeight.NORMAL)
    # TOP logo on the experimental mode button, no dark circle behind it (like the Qt UI)
    logo_size = int(UI_CONFIG.wheel_icon_size * 1.4)
    self._exp_button._txt_wheel = gui_app.texture("images/button_home.png", logo_size, logo_size)
    self._exp_button._black_bg = rl.Color(0, 0, 0, 0)

    self.brake_lights = False
    self.long_override = False
    self.right_hand_dm = False
    self.hide_bottom_icons = False
    self.road_name = ""

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

    # personality
    self.personality = 1
    self._last_personality: int | None = None
    self._personality_changed_t = -100.0

  # ---------- state ----------

  def _update_state(self) -> None:
    super()._update_state()
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

  # ---------- render ----------

  def _render(self, rect: rl.Rectangle) -> None:
    start = BRAKE_GRADIENT_START if self.brake_lights else COLORS.HEADER_GRADIENT_START
    rl.draw_rectangle_gradient_v(int(rect.x), int(rect.y), int(rect.width), UI_CONFIG.header_height, start, COLORS.HEADER_GRADIENT_END)
    self._draw_road_name(rect)

    if self.is_cruise_available:
      self._draw_set_speed(rect)
      self._draw_scc_badges(rect)
      self._draw_speed_limit(rect)

    self._draw_current_speed(rect)

    button_x = rect.x + rect.width - UI_CONFIG.border_size - UI_CONFIG.button_size
    button_y = rect.y + UI_CONFIG.border_size
    self._exp_button.render(rl.Rectangle(button_x, button_y, UI_CONFIG.button_size, UI_CONFIG.button_size))

    if ui_state.driving_personalities_ui_wheel and not self.hide_bottom_icons:
      self._draw_personality(rect)

    if ui_state.tim_signals and (self.turn_signal_left or self.turn_signal_right):
      self._draw_turn_signals(rect)

  # ---------- helpers ----------

  @staticmethod
  def _pulse(frame: int) -> bool:
    fps = max(gui_app.target_fps, 1)
    return bool(frame % fps >= fps / 2.5)

  def _text_centered(self, font: rl.Font, text: str, size: float, cx: float, top: float, color: rl.Color) -> None:
    w = measure_text_cached(font, text, int(size)).x
    rl.draw_text_ex(font, text, rl.Vector2(cx - w / 2, top), size, 0, color)

  def _text_in_rect(self, font: rl.Font, text: str, size: float, r: rl.Rectangle, color: rl.Color) -> None:
    sz = measure_text_cached(font, text, int(size))
    rl.draw_text_ex(font, text, rl.Vector2(r.x + (r.width - sz.x) / 2, r.y + (r.height - sz.y) / 2), size, 0, color)

  # ---------- set speed ----------

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    set_speed_width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial
    x = rect.x + 60 + (UI_CONFIG.set_speed_width_imperial - set_speed_width) // 2
    y = rect.y + 45

    set_speed_rect = rl.Rectangle(x, y, set_speed_width, UI_CONFIG.set_speed_height)
    rl.draw_rectangle_rounded(set_speed_rect, 0.35, 10, COLORS.BLACK_TRANSLUCENT)
    rl.draw_rectangle_rounded_lines_ex(set_speed_rect, 0.35, 10, 6, COLORS.BORDER_TRANSLUCENT)

    max_color = COLORS.GREY
    set_speed_color = COLORS.DARK_GREY
    if self.is_cruise_set:
      set_speed_color = COLORS.WHITE
      if self.speed_limit_assist_active:
        set_speed_color = OVERRIDE_GREY if self.long_override else GREEN
        max_color = OVERRIDE_GREY if self.long_override else COLORS.ENGAGED
      elif ui_state.status == UIStatus.DISENGAGED:
        max_color = COLORS.WHITE
      elif ui_state.status == UIStatus.OVERRIDE:
        max_color = COLORS.OVERRIDE
      else:
        max_color = COLORS.ENGAGED

    cx = x + set_speed_width / 2
    self._text_centered(self._font_semi_bold, tr("MAX"), FONT_SIZES.max_speed, cx, y + 27, max_color)
    set_speed_text = str(_round(self.set_speed)) if self.is_cruise_set else CRUISE_DISABLED_CHAR
    self._text_centered(self._font_bold, set_speed_text, FONT_SIZES.set_speed, cx, y + 77, set_speed_color)

  # ---------- road name ----------

  def _draw_road_name(self, rect: rl.Rectangle) -> None:
    if not self.road_name:
      return
    size = 48
    text = self.road_name
    max_width = rect.width - 40
    text_w = measure_text_cached(self._font_semi_bold, text, size).x
    if text_w + 40 > max_width:
      while text and measure_text_cached(self._font_semi_bold, text + "…", size).x + 40 > max_width:
        text = text[:-1]
      text += "…"
      text_w = measure_text_cached(self._font_semi_bold, text, size).x
    w = max(200.0, min(text_w + 40, max_width))
    r = rl.Rectangle(rect.x + rect.width / 2 - w / 2, rect.y - 4, w, 60)
    rl.draw_rectangle_rounded(r, 12 * 2 / 60, 10, rl.Color(0, 0, 0, 120))
    self._text_in_rect(self._font_semi_bold, text, size, r, ROAD_NAME_COLOR)

  # ---------- smart cruise control ----------

  def _draw_scc_badges(self, rect: rl.Rectangle) -> None:
    orders = [-70, -150]
    y_v = y_m = 0
    i = 0
    if self.scc_vision_enabled:
      y_v = orders[i]
      i += 1
    if self.scc_map_enabled:
      y_m = orders[i]

    if (self.scc_vision_enabled and not self.scc_vision_active) or (self.scc_vision_active and self._pulse(self._scc_vision_frame)):
      self._draw_scc_badge(rect, -260, y_v, "V-TSC")
    self._scc_vision_frame = self._scc_vision_frame + 1 if self.scc_vision_active else 0

    if (self.scc_map_enabled and not self.scc_map_active) or (self.scc_map_active and self._pulse(self._scc_map_frame)):
      self._draw_scc_badge(rect, -260, y_m, "M-TSC")
    self._scc_map_frame = self._scc_map_frame + 1 if self.scc_map_active else 0

  def _draw_scc_badge(self, rect: rl.Rectangle, x_offset: int, y_offset: int, text: str) -> None:
    size = 42
    text_size = measure_text_cached(self._font_bold, text, size)
    box_w, box_h = 160, text_size.y + 10
    cx = rect.x + rect.width / 2 + x_offset
    cy = rect.y + rect.height / 4 + y_offset
    r = rl.Rectangle(cx - box_w / 2, cy - box_h / 2, box_w, box_h)
    rl.draw_rectangle_rounded(r, 20 / box_h, 10, OVERRIDE_GREY if self.long_override else GREEN)
    # the Qt version cut the text out of the box, dark text reads the same over the camera
    self._text_in_rect(self._font_bold, text, size, r, rl.Color(0, 0, 0, 200))

  # ---------- speed limit ----------

  def _sign_rect(self, rect: rl.Rectangle) -> rl.Rectangle:
    width = 200 if ui_state.is_metric else 172
    x = 280 if ui_state.is_metric else 272
    return rl.Rectangle(rect.x + x, rect.y + SIGN_Y, width, SIGN_HEIGHT)

  def _draw_speed_limit(self, rect: rl.Rectangle) -> None:
    sign_rect = self._sign_rect(rect)
    if self.speed_limit_assist_state == AssistState.preActive:
      self._speed_limit_assist_frame += 1
      show = self._pulse(self._speed_limit_assist_frame)
      self._draw_pre_active_arrow(sign_rect)
    else:
      self._speed_limit_assist_frame = 0
      show = ui_state.speed_limit_mode != SPEED_LIMIT_MODE_OFF

    if show:
      self._draw_speed_limit_sign(sign_rect)
      if self.speed_limit_assist_state != AssistState.preActive:
        self._draw_upcoming_speed_limit(sign_rect)

  def _draw_speed_limit_sign(self, sign_rect: rl.Rectangle) -> None:
    warning_enabled = ui_state.speed_limit_mode >= SPEED_LIMIT_MODE_WARNING
    has_limit = self.speed_limit_valid or self.speed_limit_last_valid
    overspeed = has_limit and _round(self.speed_limit_final_last) < _round(self.speed)
    limit_text = str(_round(self.speed_limit_last)) if has_limit else "---"

    sub_text = ""
    if _round(self.speed_limit_offset) != 0:
      sub_text = ("" if self.speed_limit_offset > 0 else "-") + str(abs(_round(self.speed_limit_offset)))
    sub_factor = 0.475 if len(sub_text) >= 3 else (0.5 if ui_state.is_metric else 0.6)

    if warning_enabled and overspeed:
      speed_color = RED
    elif not self.speed_limit_valid and self.speed_limit_last_valid:
      speed_color = OVERRIDE_GREY
    else:
      speed_color = BLACK

    if ui_state.is_metric:
      # EU Vienna Convention style round sign
      size = min(sign_rect.width, sign_rect.height)
      cx = sign_rect.x + sign_rect.width / 2
      cy = sign_rect.y + sign_rect.height / 2
      radius = size / 2
      rl.draw_circle_v(rl.Vector2(cx, cy), radius, RED)
      ring = size * 0.12
      rl.draw_circle_v(rl.Vector2(cx, cy), radius - ring, rl.WHITE)
      font_size = 70 if len(limit_text) >= 3 else 85
      inner = rl.Rectangle(cx - radius + ring, cy - radius + ring, size - 2 * ring, size - 2 * ring)
      self._text_in_rect(self._font_bold, limit_text, font_size, inner, speed_color)

      if sub_text and has_limit:
        off_size = size * 0.4
        overlap = off_size * 0.25
        ox = cx + radius - off_size / 1.25 + overlap
        oy = cy - radius - off_size / 1.75 + overlap
        center = rl.Vector2(ox + off_size / 2, oy + off_size / 2)
        rl.draw_circle_v(center, off_size / 2 + 3, OFFSET_BORDER)
        rl.draw_circle_v(center, off_size / 2 - 3, BLACK)
        self._text_in_rect(self._font_bold, sub_text, off_size * sub_factor, rl.Rectangle(ox, oy, off_size, off_size), rl.WHITE)
    else:
      # US / Canada MUTCD style sign
      rl.draw_rectangle_rounded(sign_rect, 64 / sign_rect.width, 10, rl.WHITE)
      inner = rl.Rectangle(sign_rect.x + 10, sign_rect.y + 10, sign_rect.width - 20, sign_rect.height - 20)
      rl.draw_rectangle_rounded_lines_ex(inner, 44 / inner.width, 10, 4, BLACK)
      cx = inner.x + inner.width / 2
      self._text_centered(self._font_semi_bold, tr("SPEED"), 40, cx, inner.y + 10, BLACK)
      self._text_centered(self._font_semi_bold, tr("LIMIT"), 40, cx, inner.y + 50, BLACK)
      self._text_centered(self._font_bold, limit_text, 90, cx, inner.y + 80, speed_color)

      if sub_text and has_limit:
        box = sign_rect.width * 0.4
        overlap = box * 0.25
        r = rl.Rectangle(sign_rect.x + sign_rect.width - box / 1.5 + overlap, sign_rect.y - box / 1.25 + overlap, box, box)
        rl.draw_rectangle_rounded(r, 0.4, 10, BLACK)
        rl.draw_rectangle_rounded_lines_ex(r, 0.4, 10, 6, OFFSET_BORDER)
        self._text_in_rect(self._font_bold, sub_text, box * sub_factor, r, rl.WHITE)

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

  def _draw_upcoming_speed_limit(self, sign_rect: rl.Rectangle) -> None:
    ahead = (self.speed_limit_ahead_valid and self.speed_limit_ahead > 0 and self.speed_limit_ahead != self.speed_limit and
             self._speed_limit_ahead_valid_frame > 0 and self.speed_limit_source == SpeedLimitSource.map)
    if not ahead:
      return
    w, h = 170, 160
    r = rl.Rectangle(sign_rect.x + (sign_rect.width - w) / 2, sign_rect.y + sign_rect.height + 10, w, h)
    rl.draw_rectangle_rounded(r, 32 / h, 10, rl.Color(0, 0, 0, 180))
    rl.draw_rectangle_rounded_lines_ex(r, 32 / h, 10, 3, rl.Color(255, 255, 255, 100))
    cx = r.x + w / 2
    self._text_centered(self._font_semi_bold, tr("AHEAD"), 40, cx, r.y + 4, rl.Color(200, 200, 200, 255))
    self._text_centered(self._font_bold, str(_round(self.speed_limit_ahead)), 70, cx, r.y + 38, rl.WHITE)
    self._text_centered(self._font_normal, self._ahead_distance_text(), 40, cx, r.y + 110, rl.Color(180, 180, 180, 255))

  def _draw_pre_active_arrow(self, sign_rect: rl.Rectangle) -> None:
    set_speed = _round(self.set_speed)
    limit = _round(self.speed_limit_final_last)
    if set_speed == limit:
      return
    fps = max(gui_app.target_fps, 1)
    bounce = 20 * math.sin(self._speed_limit_assist_frame * 2.0 * math.pi / fps)
    up = set_speed < limit
    img = gui_app.texture("../../top/selfdrive/assets/img_plus_arrow_up.png" if up else "../../top/selfdrive/assets/img_minus_arrow_down.png",
                          105, 105)
    x = sign_rect.x + sign_rect.width + 36
    y = sign_rect.y + sign_rect.height / 2 - img.height / 2 + (bounce if up else -bounce)
    rl.draw_texture_v(img, rl.Vector2(x, y), rl.WHITE)

  # ---------- personality ----------

  def _draw_personality(self, rect: rl.Rectangle) -> None:
    now = time.monotonic()
    index = max(0, min(2, self.personality))
    if self._last_personality != index:  # show the name for a few seconds after a change (and when going onroad)
      self._personality_changed_t = now
      self._last_personality = index
    elapsed = now - self._personality_changed_t

    # same spot as the Qt UI: bottom right, moved left of the driver monitoring icon for RHD
    right = rect.x + rect.width
    x = right - 113 - 60 - 250 if self.right_hand_dm else right - 77 - 60
    y = rect.y + rect.height - 30 - 80

    text_opacity = max(0.0, min(1.0, 1.0 - (elapsed - PERSONALITY_TEXT_DURATION) / PERSONALITY_FADE_DURATION))
    image_opacity = max(0.0, min(1.0, (elapsed - PERSONALITY_TEXT_DURATION) / PERSONALITY_FADE_DURATION))

    if elapsed < PERSONALITY_TEXT_DURATION and text_opacity > 0:
      text = tr(PERSONALITY_NAMES[index])
      sz = measure_text_cached(self._font_bold, text, 40)
      rl.draw_text_ex(self._font_bold, text, rl.Vector2(x - sz.x / 2, y - sz.y / 2), 40, 0, rl.Color(255, 255, 255, int(255 * text_opacity)))
    if image_opacity > 0:
      img = gui_app.texture(PERSONALITY_IMAGES[index])
      rl.draw_texture_v(img, rl.Vector2(x - img.width / 2, y - img.height / 2), rl.Color(255, 255, 255, int(255 * image_opacity)))

  # ---------- turn signals ----------

  def _draw_turn_signals(self, rect: rl.Rectangle) -> None:
    frame = int(time.monotonic() / TURN_SIGNAL_FRAME_TIME) % 2
    signal_img = gui_app.texture(f"images/tim_turn_signal_{frame + 1}.png", TURN_SIGNAL_SIZE, TURN_SIGNAL_SIZE)
    bsm_img = gui_app.texture("images/tim_turn_signal_1_red.png", TURN_SIGNAL_SIZE, TURN_SIGNAL_SIZE, keep_aspect_ratio=False)

    y = rect.y + ((rect.height - TURN_SIGNAL_SIZE) / 2 if (self.blind_spot_left or self.blind_spot_right) else 350)
    left_x = rect.x + rect.width / 2 - 50 - 360 * (2 if self.blind_spot_left else 0)
    right_x = rect.x + rect.width / 2 - 50 + 360 * (2 if self.blind_spot_right else 0)

    def draw(x: float, flip: bool, blind_spot: bool) -> None:
      img = bsm_img if blind_spot else signal_img
      src = rl.Rectangle(0, 0, -img.width if flip else img.width, img.height)
      rl.draw_texture_pro(img, src, rl.Rectangle(x, y, TURN_SIGNAL_SIZE, TURN_SIGNAL_SIZE), rl.Vector2(0, 0), 0, rl.WHITE)

    if self.turn_signal_left:
      draw(left_x, False, self.blind_spot_left)
    if self.turn_signal_right:
      draw(right_x, True, self.blind_spot_right)
