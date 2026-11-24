"""TOP additions to the model renderer (port of TOP's Qt model.cc).

- lead chevron with a base, distance (m) and lead speed (km/h, green when the lead
  is pulling away, red when it is closing in) next to the first lead
- second lead only drawn when it is more than 3 m away from the first one
- "lock-on" boxes around the model's leads (from Ichiro's UI)
"""
from dataclasses import dataclass

import numpy as np
import pyray as rl
from openpilot.selfdrive.ui.onroad.model_renderer import ModelRenderer, LeadVehicle
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached

LOCKON_MAX = 2
LOCKON_GREEN = (23, 241, 66)
LOCKON_RED = (245, 0, 0)
HOMEBASE_H = 12


@dataclass
class Lockon:
  x: float = 0.0
  y: float = 0.0
  d: float = 0.0
  a: float = 0.0
  lxt: float = 0.0
  lxf: float = 0.0
  lock_ok: float = 0.0


@dataclass
class TopLead:
  d_rel: float = 0.0
  v_rel: float = 0.0
  x: float = 0.0
  y: float = 0.0


def _color(rgb: tuple[int, int, int], alpha: float) -> rl.Color:
  return rl.Color(rgb[0], rgb[1], rgb[2], int(max(0.0, min(255.0, alpha))))


class TopModelRenderer(ModelRenderer):
  def __init__(self):
    super().__init__()
    self._font_semi_bold = gui_app.font(FontWeight.SEMI_BOLD)
    self._lockon = [Lockon() for _ in range(LOCKON_MAX)]
    # last projected radar lead positions (kept like the Qt lead_vertices)
    self._lead_points = [(0.0, 0.0) for _ in range(LOCKON_MAX)]
    self._top_leads: list[TopLead | None] = [None, None]
    self._v_ego = 0.0
    self._a_rel = 0.0
    self._a_rel_col = 0.0

  # ---------- state ----------

  def _update_leads(self, radar_state, path_x_array):
    self._lead_vehicles = [LeadVehicle(), LeadVehicle()]
    self._top_leads = [None, None]
    leads = [radar_state.leadOne, radar_state.leadTwo]
    for i, lead_data in enumerate(leads):
      if not (lead_data and lead_data.status):
        continue
      if i == 1 and abs(leads[0].dRel - lead_data.dRel) <= 3.0:
        continue
      d_rel, y_rel, v_rel = lead_data.dRel, lead_data.yRel, lead_data.vRel
      idx = self._get_path_length_idx(path_x_array, d_rel)
      z = self._path.raw_points[idx, 2] if idx < len(self._path.raw_points) else 0.0
      point = self._map_to_screen(d_rel, -y_rel, z + self._path_offset_z)
      if point:
        self._lead_points[i] = point
        self._lead_vehicles[i] = self._top_lead_vehicle(d_rel, v_rel, point, self._rect)
        lv = self._lead_vehicles[i]
        self._top_leads[i] = TopLead(d_rel, v_rel, lv.chevron[0][0], lv.chevron[0][1])

  def _top_lead_vehicle(self, d_rel, v_rel, point, rect) -> LeadVehicle:
    speed_buff, lead_buff = 10.0, 40.0
    fill_alpha = 0.0
    if d_rel < lead_buff:
      fill_alpha = 255 * (1.0 - (d_rel / lead_buff))
      if v_rel < 0:
        fill_alpha += 255 * (-1 * (v_rel / speed_buff))
      fill_alpha = min(fill_alpha, 255)

    sz = float(np.clip((25 * 30) / (d_rel / 3 + 30), 15.0, 30.0) * 2.35)
    x = float(np.clip(point[0], rect.x, rect.x + rect.width - sz / 2))
    y = min(point[1], rect.y + rect.height - sz * 0.6)
    g_xo, g_yo, hb = sz / 5, sz / 10, HOMEBASE_H

    # triangle fans start at the top point and go counter-clockwise on screen
    glow: list = [(x, y - g_yo),
            (x - sz * 1.35 - g_xo, y + sz + g_yo), (x - sz * 1.35 - g_xo, y + sz + g_yo + hb),
            (x, y + sz + hb + g_yo + 10),
            (x + sz * 1.35 + g_xo, y + sz + g_yo + hb), (x + sz * 1.35 + g_xo, y + sz + g_yo)]
    chevron: list = [(x, y),
               (x - sz * 1.25, y + sz), (x - sz * 1.25, y + sz + hb),
               (x, y + sz + hb - 7),
               (x + sz * 1.25, y + sz + hb), (x + sz * 1.25, y + sz)]
    return LeadVehicle(glow=glow, chevron=chevron, fill_alpha=int(fill_alpha))

  # ---------- render ----------

  def _render(self, rect: rl.Rectangle):
    self._v_ego = ui_state.sm['carState'].vEgo
    super()._render(rect)

  def _draw_lead_indicator(self):
    sm = ui_state.sm
    if sm.recv_frame['modelV2'] >= ui_state.started_frame:
      leads = sm['modelV2'].leadsV3
      rl.begin_blend_mode(rl.BlendMode.BLEND_ADDITIVE)
      for i in range(min(len(leads), LOCKON_MAX)):
        if leads[i].prob > 0.2:
          self._draw_lockon(leads[i], self._lead_points[i], i)
      rl.end_blend_mode()

    for i, lead in enumerate(self._lead_vehicles):
      if not lead.glow or not lead.chevron:
        continue
      rl.draw_triangle_fan(lead.glow, len(lead.glow), rl.Color(218, 202, 37, 210))
      rl.draw_triangle_fan(lead.chevron, len(lead.chevron), rl.Color(201, 34, 49, lead.fill_alpha))
      top_lead = self._top_leads[i]
      if i == 0 and top_lead is not None:
        self._draw_lead_text(top_lead)

  def _draw_lead_text(self, lead: TopLead) -> None:
    size = 44
    dist = f"{lead.d_rel:.0f}m"
    kmph = f"{(lead.v_rel + self._v_ego) * 3.6:.0f}k"
    dx = 2  # keep clear of the chevron tip
    dist_sz = measure_text_cached(self._font_semi_bold, dist, size)
    kmph_sz = measure_text_cached(self._font_semi_bold, kmph, size)
    base_y = lead.y - dist_sz.y  # bottom aligned to the chevron tip

    if self._a_rel >= self._a_rel_col:
      self._a_rel_col = -0.1  # hysteresis
      speed_color = rl.Color(23, 241, 66, 255)
    else:
      self._a_rel_col = 0.0
      speed_color = rl.Color(245, 0, 0, 255)

    shadow = rl.Color(0, 0, 0, 200)
    dist_pos = rl.Vector2(lead.x + dx, base_y)
    kmph_pos = rl.Vector2(lead.x - dx - 2 - kmph_sz.x, base_y)
    rl.draw_text_ex(self._font_semi_bold, dist, rl.Vector2(dist_pos.x + 2, dist_pos.y + 2), size, 0, shadow)
    rl.draw_text_ex(self._font_semi_bold, kmph, rl.Vector2(kmph_pos.x + 2, kmph_pos.y + 2), size, 0, shadow)
    rl.draw_text_ex(self._font_semi_bold, dist, dist_pos, size, 0, rl.WHITE)
    rl.draw_text_ex(self._font_semi_bold, kmph, kmph_pos, size, 0, speed_color)

  def _draw_lockon(self, lead_data, vd: tuple[float, float], num: int) -> None:
    r_rect = self._rect
    left, top = r_rect.x, r_rect.y
    right, bottom = r_rect.x + r_rect.width, r_rect.y + r_rect.height

    d_rel = lead_data.x[0]
    a_rel = lead_data.a[0]
    self._a_rel = a_rel

    sz = float(np.clip((25 * 30) / (d_rel / 3 + 30), 15.0, 30.0) * 2.35)
    x = float(np.clip(vd[0], left, right - sz / 2))
    y = float(vd[1])

    prob_alpha = max(0.0, min(1.0, lead_data.prob)) * 245
    green = _color(LOCKON_GREEN, prob_alpha)
    ww = hh = 300 * 1.25
    d = max(d_rel, 1.0)

    lk = self._lockon[num]
    lk.x += (x - lk.x) / 6
    lk.y += (y - lk.y) / 6
    lk.d += (d - lk.d) / 6
    x, y, d = lk.x, lk.y, max(lk.d, 1.0)
    lk.a += (a_rel - lk.a) / 10
    a_rel = lk.a

    # narrow road camera
    dh = 50.0
    dd = max((d - 25) / (75.0 / 2) + 1, 1.0)
    dh /= dd

    ww = ww * 2 * 5 / d
    hh = hh * 2 * 5 / d
    y = min(bottom, y - dh) + dh
    r = rl.Rectangle(x - ww / 2, y - hh - dh, ww, hh)
    r_right, r_bottom = r.x + r.width, r.y + r.height
    cx, cy = r.x + r.width / 2, r.y + r.height / 2

    y0 = self._lockon[0].x * self._lockon[0].d
    y1 = self._lockon[1].x * self._lockon[1].d
    font_size = 38

    if num == 0:
      rl.draw_rectangle_lines_ex(r, 2, green)
      if self._lockon[0].x > self._lockon[1].x - 20:
        lk.lxt += (r_right - lk.lxt) / 20
        lk.lxf += (right - lk.lxf) / 20
      else:
        lk.lxt += (r.x - lk.lxt) / 20
        lk.lxf += (left - lk.lxf) / 20
      rl.draw_text_ex(self._font_semi_bold, f" {num + 1}", rl.Vector2(r.x, r.y), font_size, 0, green)
      lxt = min(max(lk.lxt, r.x), r_right)
      rl.draw_line_ex(rl.Vector2(lxt, r.y), rl.Vector2(lk.lxf, top), 2, green)

      if ww >= 40:
        wwa = min(max(ww * 0.15, 10), 40, ww)
        if a_rel > 0:
          hha = max(1 - 0.1 / a_rel, 0) * hh
          meter = [(x + ww / 2 - wwa / 2 - wwa / 2 * hha / hh, y - hh - dh + (hh - hha)),
                   (x + ww / 2 - wwa / 2, y - hh - dh + hh),
                   (x + ww / 2, y - hh - dh + hh),
                   (x + ww / 2, y - hh - dh + (hh - hha))]
          rl.draw_triangle_fan(meter, len(meter), _color(LOCKON_GREEN, prob_alpha * 0.9))
        elif a_rel < 0:
          hha = max(1 + 0.1 / a_rel, 0) * hh
          meter = [(x + ww / 2 - wwa / 2, y - hh - dh),
                   (x + ww / 2 - wwa / 2 - wwa / 2 * hha / hh, y - hh - dh + hha),
                   (x + ww / 2, y - hh - dh + hha),
                   (x + ww / 2, y - hh - dh)]
          rl.draw_triangle_fan(meter, len(meter), _color(LOCKON_RED, prob_alpha))

      lk.lock_ok += ((40 if abs(y0 - y1) <= 300 else 0) - lk.lock_ok) / 5
      td = lk.lock_ok
      if td >= 3:
        ddd = (max(lk.d, 10) - 10) / (90.0 / 2) + 1
        td /= ddd
        tlw = 8.0
        h = tlw / 2
        rl.draw_line_ex(rl.Vector2(cx, r.y - h), rl.Vector2(cx, r.y - td), tlw, green)
        rl.draw_line_ex(rl.Vector2(r.x - h, cy), rl.Vector2(r.x - td, cy), tlw, green)
        rl.draw_line_ex(rl.Vector2(r_right + h, cy), rl.Vector2(r_right + td, cy), tlw, green)
        rl.draw_line_ex(rl.Vector2(cx, r_bottom + h), rl.Vector2(cx, r_bottom + td), tlw, green)
    else:
      color = _color(LOCKON_RED, prob_alpha) if abs(y0 - y1) > 300 else green
      if self._lockon[0].x > self._lockon[1].x - 20:
        lk.lxt += (r.x - lk.lxt) / 20
        lk.lxf += (left - lk.lxf) / 20
      else:
        lk.lxt += (r_right - lk.lxt) / 20
        lk.lxf += (right - lk.lxf) / 20
      lxt = min(max(lk.lxt, r.x), r_right)
      rl.draw_line_ex(rl.Vector2(lxt, r.y), rl.Vector2(lk.lxf, top), 2, color)
      rl.draw_rectangle_lines_ex(r, 2, color)

      if ww >= 80 and (d_rel < 32 or abs(y0 - y1) > 300):
        sz_txt = measure_text_cached(self._font_semi_bold, f" {num + 1}", font_size)
        rl.draw_text_ex(self._font_semi_bold, f" {num + 1}", rl.Vector2(r.x, r_bottom - sz_txt.y), font_size, 0, color)
