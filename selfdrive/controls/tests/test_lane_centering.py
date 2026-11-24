import numpy as np

import cereal.messaging as messaging
from cereal import log
from openpilot.selfdrive.controls.lib import lane_centering as lc
from openpilot.selfdrive.controls.lib.lane_centering import LaneCentering

X = np.linspace(0.0, 120.0, 33)
V = 20.0


def model(plan_y=0.0, left=-1.6, right=1.6, probs=(0.95, 0.95), lane_change=log.LaneChangeState.off, stds=(0.06, 0.06)):
  """Straight lane, lines at left/right (y right positive), plan parallel at plan_y."""
  msg = messaging.new_message('modelV2')
  md = msg.modelV2
  lines = md.init('laneLines', 4)
  for line, y in zip(lines, (left - 3.2, left, right, right + 3.2), strict=True):
    line.x, line.y = X.tolist(), [y] * len(X)
  md.laneLineProbs = [0.1, probs[0], probs[1], 0.1]
  md.laneLineStds = [0.3, stds[0], stds[1], 0.3]
  md.position.x, md.position.y = X.tolist(), [plan_y] * len(X)
  md.meta.laneChangeState = lane_change
  return md


def settle(L, md, v=V, pressed=False, enabled=True, secs=3.0):
  for k in range(int(secs / 0.01)):
    out = L.update(enabled, md, k % 5 == 0, v, pressed)
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
  settle(L, model(plan_y=0.0), secs=lc.LEAK_T)
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
