import numpy as np

import cereal.messaging as messaging
from cereal import log
from openpilot.selfdrive.controls.lib import lane_centering as lc
from openpilot.selfdrive.controls.lib.lane_centering import LaneCentering

X = np.linspace(0.0, 120.0, 33)
V = 20.0


def model(plan_y=0.0, left=-1.6, right=1.6, probs=(0.95, 0.95), lane_change=log.LaneChangeState.off):
  """Straight lane, lines at left/right (y right positive), plan parallel at plan_y."""
  msg = messaging.new_message('modelV2')
  md = msg.modelV2
  lines = md.init('laneLines', 4)
  for line, y in zip(lines, (left - 3.2, left, right, right + 3.2), strict=True):
    line.x, line.y = X.tolist(), [y] * len(X)
  md.laneLineProbs = [0.1, probs[0], probs[1], 0.1]
  md.position.x, md.position.y = X.tolist(), [plan_y] * len(X)
  md.meta.laneChangeState = lane_change
  return md


def settle(L, md, v=V, pressed=False, enabled=True, secs=3.0):
  for k in range(int(secs / 0.01)):
    out = L.update(enabled, md, k % 5 == 0, v, pressed)
  return out


def test_plan_left_of_centre_steers_right():
  L = LaneCentering()
  out = settle(L, model(plan_y=-0.6))
  dist = V * lc.LOOKAHEAD_T
  expected = lc.WEIGHT * 2.0 * (0.6 - lc.DEADBAND) / dist ** 2
  assert out > 0.0
  assert np.isclose(out, min(expected, lc.MAX_LAT_ACCEL / V ** 2), rtol=1e-3)
  assert np.isclose(L.offset, -0.6)


def test_centred_plan_or_inside_deadband_does_nothing():
  for plan_y in (0.0, lc.DEADBAND * 0.9, -lc.DEADBAND * 0.9):
    assert settle(LaneCentering(), model(plan_y=plan_y)) == 0.0


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


def test_override_fades_out():
  L = LaneCentering()
  on = settle(L, model(plan_y=-0.6))
  after = L.update(True, model(plan_y=-0.6), True, V, True)
  assert 0.0 < after < on


def test_one_line_opens_up_centre_stays_half_a_lane_from_the_other():
  L = LaneCentering()
  settle(L, model(plan_y=0.0, left=-1.6, right=1.6), secs=20.0)  # learns a 3.2 m lane
  assert np.isclose(L.lane_width, 3.2, atol=0.01)
  # right line fades and moves out (a turn bay); the plan drifts toward the left line
  out = settle(L, model(plan_y=-0.3, left=-1.6, right=2.6, probs=(0.95, 0.4)))
  assert np.isclose(L.offset, -0.3, atol=1e-6)  # centre still 1.6 m from the left line
  assert out > 0.0
