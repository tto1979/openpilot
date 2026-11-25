import numpy as np

import cereal.messaging as messaging
from cereal import log
from openpilot.selfdrive.controls.lib import lane_centering as lc
from openpilot.selfdrive.controls.lib.lane_centering import LaneCentering

X = np.linspace(0.0, 120.0, 33)
V = 20.0


def model(plan_y=0.0, left=-1.6, right=1.6, probs=(0.95, 0.95), lane_change=log.LaneChangeState.off, stds=(0.06, 0.06),
          edges=(-6.0, 6.0), edge_stds=(0.5, 0.5)):
  """Straight lane, lines at left/right (y right positive), plan parallel at plan_y, road edges at edges."""
  msg = messaging.new_message('modelV2')
  md = msg.modelV2
  lines = md.init('laneLines', 4)
  for line, y in zip(lines, (left - 3.2, left, right, right + 3.2), strict=True):
    line.x, line.y = X.tolist(), [y] * len(X)
  md.laneLineProbs = [0.1, probs[0], probs[1], 0.1]
  md.laneLineStds = [0.3, stds[0], stds[1], 0.3]
  md.position.x, md.position.y = X.tolist(), [plan_y] * len(X)
  road_edges = md.init('roadEdges', 2)
  for edge, y in zip(road_edges, edges, strict=True):
    edge.x, edge.y = X.tolist(), [y] * len(X)
  md.roadEdgeStds = list(edge_stds)
  md.meta.laneChangeState = lane_change
  return md


def settle(L, md, v=V, pressed=False, enabled=True, secs=3.0):
  for k in range(int(secs / 0.01)):
    out = L.update(enabled, md, k % 5 == 0, v, pressed)
  return out


def settle_full(L, md, model_curvature=0.0, measured=None, v=V, secs=3.0, lat_delay=0.4):
  """With the measured curvature (default: the car follows the command at once), the model's curvature, the delay."""
  out = 0.0
  for k in range(int(secs / 0.01)):
    curvature = model_curvature + out if measured is None else measured
    out = L.update(True, md, k % 5 == 0, v, False, curvature, model_curvature, lat_delay)
  return out


def proportional(offset, v=V):
  dist = v * lc.LOOKAHEAD_T
  return lc.WEIGHT * 2.0 * (abs(offset) - lc.DEADBAND) / dist ** 2


def test_plan_left_of_centre_steers_right_and_the_integral_adds():
  L = LaneCentering()
  out = settle(L, model(plan_y=-0.3), secs=1.0)
  assert out > 0.0
  assert np.isclose(L.offset, -0.3)
  later = settle(L, model(plan_y=-0.3), secs=4.0)
  assert later > proportional(0.3) * 1.3  # the offset persisted: the integral has added to it
  assert later * V ** 2 <= lc.MAX_LAT_ACCEL + 1e-9


def test_centred_plan_does_nothing():
  assert settle(LaneCentering(), model(plan_y=0.0)) == 0.0


def test_capped_and_faded_in():
  L = LaneCentering()
  md = model(plan_y=-1.5)
  first = L.update(True, md, True, V, False)
  assert 0.0 < first * V ** 2 <= lc.MAX_LAT_JERK * 0.01 + 1e-9
  assert np.isclose(settle(L, md) * V ** 2, lc.MAX_LAT_ACCEL)


def test_off_in_lane_change_override_faint_lines_and_low_speed():
  assert settle(LaneCentering(), model(plan_y=-0.6, lane_change=log.LaneChangeState.laneChangeStarting)) == 0.0
  assert settle(LaneCentering(), model(plan_y=-0.6), pressed=True) == 0.0
  assert settle(LaneCentering(), model(plan_y=-0.6), enabled=False) == 0.0
  assert settle(LaneCentering(), model(plan_y=-0.6, probs=(0.4, 0.45))) == 0.0
  assert settle(LaneCentering(), model(plan_y=-0.6), v=5.0) == 0.0
  # both lines clear but far too wide apart (a split): leave it to the model
  assert settle(LaneCentering(), model(plan_y=-0.6, left=-2.6, right=2.6)) == 0.0


def test_override_fades_out_and_clears_the_integral():
  L = LaneCentering()
  on = settle(L, model(plan_y=-0.6))
  after = L.update(True, model(plan_y=-0.6), True, V, True)
  assert 0.0 < after < on
  assert L.integral == 0.0


def test_integral_leaks_away_once_the_plan_is_centred():
  L = LaneCentering()
  settle(L, model(plan_y=-0.4), secs=6.0)
  held = L.integral
  assert held > 0.0
  settle(L, model(plan_y=0.0), secs=lc.LEAK_T + lc.PROPORTIONAL_TC)  # the smoothed proportional part adds for ~1 s
  assert 0.0 <= L.integral < held * 0.45


def test_one_line_opens_up_centre_stays_half_a_lane_from_the_other():
  L = LaneCentering()
  settle(L, model(plan_y=0.0, left=-1.6, right=1.6), secs=20.0)  # learns a 3.2 m lane
  assert np.isclose(L.lane_width, 3.2, atol=0.01)
  # right line fades and moves out (a turn bay); the plan drifts toward the left line
  out = settle(L, model(plan_y=-0.3, left=-1.6, right=2.6, probs=(0.95, 0.4)))
  assert np.isclose(L.offset, -0.3, atol=1e-6)  # centre still 1.6 m from the left line
  assert out > 0.0


def test_not_in_alleys_and_slow_streets():
  # 40 km/h: below the 45-60 km/h ramp
  assert settle(LaneCentering(), model(plan_y=-0.6), v=40 / 3.6) == 0.0
  # a narrow street lane (2.5 m): learned, then left to the model
  L = LaneCentering()
  out = settle(L, model(plan_y=-0.4, left=-1.25, right=1.25), secs=20.0)
  assert L.lane_width < 2.6 and out == 0.0


def test_unsure_lane_lines_count_less():
  sure = settle(LaneCentering(), model(plan_y=-0.3), secs=1.0)
  assert sure > 0.0
  assert settle(LaneCentering(), model(plan_y=-0.3, stds=(0.35, 0.35)), secs=1.0) == 0.0


# the car 0.4 m left of the lane centre, the plan holding it there
OFF_CENTRE = dict(plan_y=0.0, left=-1.2, right=2.0)


def test_limits_change_nothing_while_the_car_isnt_heading_back():
  plain = settle(LaneCentering(), model(**OFF_CENTRE), secs=1.0)
  L = LaneCentering()
  assert np.isclose(settle_full(L, model(**OFF_CENTRE), secs=1.0), plain)
  assert L.recovery == 1.0 and L.edge == 1.0 and not L.held


def test_held_wheel_stops_the_integral():
  # the model asks for 0.2 m/s^2 to the right, the command for more, the wheel stays straight
  held = LaneCentering()
  settle_full(held, model(**OFF_CENTRE), model_curvature=0.2 / V ** 2, measured=0.0, secs=3.0)
  turning = LaneCentering()
  settle_full(turning, model(**OFF_CENTRE), model_curvature=0.2 / V ** 2, secs=3.0)
  assert held.held and not turning.held
  assert turning.integral > 0.0 and held.integral < 0.05 * turning.integral  # one step before the first command


def test_no_more_push_when_the_car_already_heads_back():
  L = LaneCentering()
  settle_full(L, model(**OFF_CENTRE), measured=0.0, secs=2.0)
  full = L.target
  # already turning right at 0.6 m/s^2: its path crosses the centre within the next 1.5 s
  L = LaneCentering()
  settle_full(L, model(**OFF_CENTRE), model_curvature=0.6 / V ** 2, measured=0.6 / V ** 2, secs=2.0)
  assert full > 0.0 and L.recovery < 0.5 and L.target < 0.5 * full


def test_not_toward_a_road_edge_the_body_is_already_close_to():
  # a barrier 0.35 m from the car's right side (body half width 0.95), 0.7 m inside the lane: some
  barrier = dict(OFF_CENTRE, edges=(-6.0, 1.3), edge_stds=(0.5, 0.1))
  L = LaneCentering()
  out = settle_full(L, model(**barrier), secs=3.0)
  assert L.edge < 1.0 and out < settle_full(LaneCentering(), model(**OFF_CENTRE), secs=3.0)
  # right at it: nothing toward it at all
  L = LaneCentering()
  assert settle_full(L, model(**dict(OFF_CENTRE, edges=(-6.0, 1.1), edge_stds=(0.5, 0.1))), secs=3.0) == 0.0 and L.edge == 0.0
  # the same edge but the model unsure of it, or on the other side: no limit
  L = LaneCentering()
  settle_full(L, model(**dict(OFF_CENTRE, edges=(-6.0, 1.1), edge_stds=(0.5, 0.5))), secs=1.0)
  assert L.edge == 1.0
  L = LaneCentering()
  settle_full(L, model(**dict(OFF_CENTRE, edges=(-1.1, 6.0), edge_stds=(0.1, 0.5))), secs=1.0)
  assert L.edge == 1.0


def test_proportional_part_follows_the_offset_smoothly():
  L = LaneCentering()
  settle(L, model(plan_y=-0.3), secs=lc.PROPORTIONAL_TC)
  assert 0.55 * proportional(0.3) < L.proportional < 0.7 * proportional(0.3)  # 1 - 1/e after one time constant
  # the offset flips +-0.1 m every model frame (lane lines far out jump): the correction hardly moves
  L = LaneCentering()
  for k in range(300):
    L.update(True, model(plan_y=0.1 if (k // 5) % 2 else -0.1), k % 5 == 0, V, False)
  assert abs(L.curvature) * V ** 2 < 0.01
