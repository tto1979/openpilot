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
  offset     plan minus centre at the aim point
  correction curvature that moves the aim point WEIGHT x the offset toward the centre (2 x offset / distance^2), plus
             an integral: while the offset persists, the same amount again every INTEGRAL_T, leaking away over LEAK_T.
             At most MAX_LAT_ACCEL of lateral acceleration, fading in and out at MAX_LAT_JERK

The weight drops to 0 without a usable lane line (probability, and the model's std of the line), where both lines are
clear but the lane is far off a normal width (merges, splits), in narrow lanes, below 45 km/h (full from 60), during a
lane change and while the driver steers. Curvatures and y are right positive, as desiredCurvature and the model frame.

Not in alleys and slow streets: there the "lane" the model sees often has parked cars inside it and the model keeps
away from them on purpose. In an alley at 28-37 km/h (00000039--dc1aece740--18) the correction pushed 0.03 m/s^2 toward
the parked cars on the right for 3 s (radar tracks 1.5-2 m to the side) before the driver steered away; it was
active from 29 km/h then. From 45 km/h, in lanes 2.8 m or wider, the simulation results below are unchanged above
60 km/h (wheel within 0.2 m of a line 3.4%, over it 1.8%) and the alley segment gets no correction at all.

The model pulls back: in a long right curve at 75 km/h (1.9 m/s^2) it raised its own request as the correction took
curvature away, and with the first tuning (weight 0.7, 0.1 m deadband, no integral, 0.3 m/s^2) the car still sat
0.5-0.9 m inside, the wheel up to 0.4 m over the line (00000038--5caa4048ef--28/29; without the correction 1.0-1.5 m).
The integral keeps adding while the plan stays off centre, so the pull can't hold the car there.

Tuned in closed loop on 11 Prius v rlogs, the driving model's reaction to the car's position fitted from the same
logs (with the first tuning the simulation matched the road: wheel over the line 19% vs 22% of the time on segment
29): wheels within 0.2 m of a line 15% of the time without, 8.7% with the first tuning, 3.6% now; over the line
9.4% / 3.9% / 1.7%; in that curve the wheel stays 0.12 m inside the line (first tuning: 0.23 m over). A car the model
holds 0.8 m off centre on straight road settles within 0.1 m of the centre in 7 s. Stable up to 0.6 s of steering
delay in that test and up to 0.8 s on the logs (the Prius v follows in 0.3-0.45 s).
"""
import numpy as np

from cereal import log
from openpilot.common.realtime import DT_CTRL, DT_MDL

LaneChangeState = log.LaneChangeState

LOOKAHEAD_T = 2.0      # s
LOOKAHEAD_MIN = 15.0   # m
LOOKAHEAD_MAX = 50.0   # m
WEIGHT = 1.0           # share of the offset beyond the deadband corrected at once
DEADBAND = 0.0         # m
INTEGRAL_T = 4.0       # s
LEAK_T = 8.0           # s
MAX_LAT_ACCEL = 0.5    # m/s^2
MAX_LAT_JERK = 0.5     # m/s^3
PROB_BP, PROB_V = [0.5, 0.8], [0.0, 1.0]
WIDTH_BP, WIDTH_V = [2.2, 2.5, 4.3, 4.6], [0.0, 1.0, 1.0, 0.0]  # both lines clear but this far apart: no centring
SPEED_BP, SPEED_V = [12.5, 16.7], [0.0, 1.0]   # 45 -> 60 km/h: not in alleys and slow streets
STD_BP, STD_V = [0.2, 0.3], [1.0, 0.0]         # lane line std (m): unsure lines count less
NARROW_BP, NARROW_V = [2.6, 2.8], [0.0, 1.0]   # learned lane width (m): narrow street lanes are left to the model
LANE_WIDTH_INIT = 3.2  # m
LANE_WIDTH_MIN, LANE_WIDTH_MAX = 2.3, 3.8   # range the width is learned in
LANE_WIDTH_TC = 2.0    # s


class LaneCentering:
  def __init__(self) -> None:
    self.lane_width = LANE_WIDTH_INIT
    self.curvature = 0.0  # correction added to the model's desired curvature (1/m)
    self.target = 0.0
    self.offset = 0.0     # plan minus lane centre at the aim point (m)
    self.weight = 0.0
    self.integral = 0.0   # 1/m

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
    stds = model_v2.laneLineStds
    if len(stds) > 2:
      p_left *= float(np.interp(stds[1], STD_BP, STD_V))
      p_right *= float(np.interp(stds[2], STD_BP, STD_V))
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
             * float(np.interp(v_ego, SPEED_BP, SPEED_V)) * float(np.interp(self.lane_width, NARROW_BP, NARROW_V))
    error = -np.sign(offset) * max(abs(offset) - DEADBAND, 0.0)  # toward the centre
    limit = MAX_LAT_ACCEL / max(v_ego, 1.0) ** 2
    return float(np.clip(weight * 2.0 * error / dist ** 2, -limit, limit)), offset, weight

  def update(self, enabled: bool, model_v2, model_updated: bool, v_ego: float, steering_pressed: bool) -> float:
    active = enabled and not steering_pressed and model_v2.meta.laneChangeState == LaneChangeState.off
    limit = MAX_LAT_ACCEL / max(v_ego, 1.0) ** 2
    if model_updated:
      p, self.offset, self.weight = self.compute(model_v2, v_ego)
      if active and self.weight > 0.0:
        self.integral += p * DT_MDL / INTEGRAL_T - self.integral * DT_MDL / LEAK_T
      else:
        self.integral = 0.0
      self.integral = float(np.clip(self.integral, -limit, limit))
      self.target = float(np.clip(p + self.integral, -limit, limit))
    if not active:
      self.target, self.weight, self.integral = 0.0, 0.0, 0.0
    step = MAX_LAT_JERK / max(v_ego, 1.0) ** 2 * DT_CTRL
    self.curvature += float(np.clip(self.target - self.curvature, -step, step))
    return self.curvature
