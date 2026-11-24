"""TOP: lane centering on top of the driving model's plan.

The driving model doesn't always keep to the middle of the lane: on the Prius v it cut gentle curves by 0.4-0.6 m and,
where the lane widened on the right, held the car 0.5-0.7 m left of the centre with its left wheels 0.1-0.25 m from the
line, on straight road too. Steering control followed the plan; the plan itself was off centre.

Where the model sees the lane lines, this aims the plan part of the way back to the lane centre:

  aim point  LOOKAHEAD_T ahead (LOOKAHEAD_MIN-LOOKAHEAD_MAX m)
  centre     half a lane width in from each lane line (lines 1 and 2), averaged by how sure the model is of each line
             (probability 0.5 -> 0, 0.8 -> full). The lane width is learned while both lines are clear, so where one
             line fades or opens up (a turn bay, a bus stop, a wide shoulder) the centre stays a lane's half width from
             the other line
  offset     plan minus centre at the aim point; the first DEADBAND metres are left alone, so the model keeps its own
             line near the centre
  correction curvature that moves the aim point WEIGHT x the rest of the offset toward the centre (2 x offset /
             distance^2), at most MAX_LAT_ACCEL of lateral acceleration, fading in and out at MAX_LAT_JERK

The weight drops to 0 without a usable lane line, where both lines are clear but the lane is far off a normal width
(merges, splits), below SPEED_BP, during a lane change and while the driver steers. Curvatures and y are right
positive, as desiredCurvature and the model frame.

Tuned in closed loop on 7 Prius v rlogs, with the driving model's reaction to the car's position fitted from the same
logs: wheels within 0.2 m of a lane line 16% of the time -> 8% (today's segment with the left lean 28% -> 9%), 95th
percentile offset 0.92 -> 0.63 m. A car the model holds 0.6 m off centre settles at 0.3 m in 3 s. It stays stable up to
0.8 s of steering delay (the Prius v follows in 0.3-0.45 s); a shorter lookahead or more weight centres harder but
starts to weave at 0.6-0.8 s.
"""
import numpy as np

from cereal import log
from openpilot.common.realtime import DT_CTRL, DT_MDL

LaneChangeState = log.LaneChangeState

LOOKAHEAD_T = 2.0      # s
LOOKAHEAD_MIN = 15.0   # m
LOOKAHEAD_MAX = 50.0   # m
WEIGHT = 0.7           # share of the offset beyond the deadband that is corrected
DEADBAND = 0.1         # m
MAX_LAT_ACCEL = 0.3    # m/s^2
MAX_LAT_JERK = 0.5     # m/s^3
PROB_BP, PROB_V = [0.5, 0.8], [0.0, 1.0]
WIDTH_BP, WIDTH_V = [2.2, 2.5, 4.3, 4.6], [0.0, 1.0, 1.0, 0.0]  # both lines clear but this far apart: no centring
SPEED_BP, SPEED_V = [8.0, 11.0], [0.0, 1.0]
LANE_WIDTH_INIT = 3.2  # m
LANE_WIDTH_MIN, LANE_WIDTH_MAX = 2.7, 3.8
LANE_WIDTH_TC = 2.0    # s


class LaneCentering:
  def __init__(self) -> None:
    self.lane_width = LANE_WIDTH_INIT
    self.curvature = 0.0  # correction added to the model's desired curvature (1/m)
    self.target = 0.0
    self.offset = 0.0     # plan minus lane centre at the aim point (m)
    self.weight = 0.0

  def compute(self, model_v2, v_ego: float) -> tuple[float, float, float]:
    """(correction curvature before the fade, offset, weight) for one modelV2 message; learns the lane width."""
    lines, probs, pos = model_v2.laneLines, model_v2.laneLineProbs, model_v2.position
    if len(lines) < 3 or len(probs) < 3 or len(pos.x) < 2 or len(lines[1].x) < 2 or len(lines[2].x) < 2:
      return 0.0, 0.0, 0.0
    dist = float(np.clip(v_ego * LOOKAHEAD_T, LOOKAHEAD_MIN, LOOKAHEAD_MAX))
    left = float(np.interp(dist, lines[1].x, lines[1].y))
    right = float(np.interp(dist, lines[2].x, lines[2].y))
    p_left = float(np.interp(probs[1], PROB_BP, PROB_V))
    p_right = float(np.interp(probs[2], PROB_BP, PROB_V))
    if p_left + p_right < 1e-3:
      return 0.0, 0.0, 0.0
    both = p_left * p_right
    width = right - left
    if both >= 1.0 and LANE_WIDTH_MIN <= width <= LANE_WIDTH_MAX:
      self.lane_width += (width - self.lane_width) * DT_MDL / LANE_WIDTH_TC
    half = self.lane_width / 2
    centre = (p_left * (left + half) + p_right * (right - half)) / (p_left + p_right)
    offset = float(np.interp(dist, pos.x, pos.y)) - centre
    weight = WEIGHT * (p_left + p_right - both) * (1.0 - both * (1.0 - float(np.interp(width, WIDTH_BP, WIDTH_V)))) \
             * float(np.interp(v_ego, SPEED_BP, SPEED_V))
    error = -np.sign(offset) * max(abs(offset) - DEADBAND, 0.0)  # toward the centre
    limit = MAX_LAT_ACCEL / max(v_ego, 1.0) ** 2
    return float(np.clip(weight * 2.0 * error / dist ** 2, -limit, limit)), offset, weight

  def update(self, enabled: bool, model_v2, model_updated: bool, v_ego: float, steering_pressed: bool) -> float:
    if model_updated:
      self.target, self.offset, self.weight = self.compute(model_v2, v_ego)
    if not enabled or steering_pressed or model_v2.meta.laneChangeState != LaneChangeState.off:
      self.target, self.weight = 0.0, 0.0
    step = MAX_LAT_JERK / max(v_ego, 1.0) ** 2 * DT_CTRL
    self.curvature += float(np.clip(self.target - self.curvature, -step, step))
    return self.curvature
