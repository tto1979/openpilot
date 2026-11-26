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
  correction curvature that moves the aim point WEIGHT x the offset toward the centre (2 x offset / distance^2),
             smoothed over PROPORTIONAL_TC, plus an integral: while the offset persists, the same amount again every
             INTEGRAL_T, leaking away over LEAK_T. At most MAX_LAT_ACCEL of lateral acceleration, fading in and out at
             MAX_LAT_JERK

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

With the measured curvature, the model's own desired curvature and the lateral delay (controlsd passes them), three
limits, ideas from StarPilot's lane centering (StarPilotHQ/StarPilot stable, read 2026-11-24; written anew here):

  held wheel  the integral doesn't grow while the car lags the command from lat_delay ago by more than TRACK_LAG_MAX
              in the direction it pushes (measured curvature outside the range between the model's curvature and the
              command then). A held wheel otherwise lets the integral build up and it all comes out when the wheel
              lets go
  recovery    the car's path over HORIZON_T is predicted from the measured curvature, then after lat_delay the model's
              curvature plus the integral. The proportional part is cut to the share that brings that path closest to
              the lane centre (least squares), none of it where the car is already heading back and would cross the
              centre anyway; the integral doesn't grow meanwhile. Only the proportional part: the integral holds
              against the model's pull, and cutting it too (as StarPilot cuts its trim) let the model pull the car back
              and swing 0.2 m with 0.7 s of steering delay
  road edge   the correction is cut to the largest of EDGE_FACTORS that doesn't bring the body (BODY_HALF_WIDTH) closer
              to a road edge the model is sure of (std up to EDGE_STD_MAX) than without it, where that is nearer than
              EDGE_MARGIN + the edge's std. For barriers and curbs inside the painted lane; parked cars aren't road edges

Simulated as above (steering delay 0.3 s + 0.1 s EPS lag, 75 km/h): on the 11 rlogs about the same as without them
(wheel within 0.2 m of a line 4.0% vs 3.8%, over it 2.1% both, the long curve 0.15 m inside vs 0.16 m; the road edges
were never close enough to act). On straight road with the model holding the car 0.8 m off centre and a wheel that
holds until the command differs by 0.1 / 0.2 m/s^2, the car swung 0.21 / 0.41 m peak to peak every 7 s without them,
0.10 / 0.27 m with them (with 0.55 s of delay 0.25 / 0.49 m -> 0.10 / 0.32 m) and settles on the centre instead of
0.07 m past it. A barrier 0.6 m right of the lane centre with the model keeping the car 0.5 m left: without the road
edge limit the body ended up 0.40 m into it, with it the car stays where the model put it (0.15 m clear).

The proportional part is smoothed: the offset at the aim point moves 0.03-0.07 m from one model frame to the next
(mostly the lane lines that far out) and the model's plan wanders at 0.3-0.7 Hz; at weight 1 the correction followed
all of it. On the road (0000003d--ca235d1ccc--0..2, 80-90 km/h, gentle curves and straights) the steering torque
hunted +-0.1-0.2 of max every 2-3 s while the wheel held 80-90% of the time; the driver felt constant small
corrections, far fewer with lane centering off. Across 5 s windows that hunting followed the correction's 0.25-0.7 Hz
part (correlation 0.42, 3x as strongly as the model's own). Replayed on those segments, smoothing over 1 s halves
that part (0.010-0.016 -> 0.006-0.008 m/s^2) and the correction turns round 1-1.6 times a second instead of 6-7. In
closed loop on the 11 rlogs: the request lane centering adds (with the model's reaction) at 0.25-0.7 Hz 0.021 ->
0.011 m/s^2; wheel within 0.2 m of a line 4.0% -> 5.1%, over it 2.1% -> 2.6% (without lane centering 15% / 9.4%),
the long curve 0.12 m inside; a held wheel (0.1 / 0.2 m/s^2) swings 0.13 / 0.41 m (0.10 / 0.27 m unsmoothed); when
the model steers round something it gets 0.20 m over instead of 0.11 m.
"""
from collections import deque

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

TRACK_LAG_MAX = 0.1    # m/s^2: the car may lag the command from lat_delay ago this much before the integral stops
HORIZON_T, HORIZON_MIN, HORIZON_MAX = 1.5, 8.0, 35.0  # s, m, m: predicted path for the recovery and road edge limits
HANDOFF_T = 0.2        # s: after lat_delay the car goes from the measured curvature to the command over this long
BODY_HALF_WIDTH = 0.95 # m
EDGE_MARGIN = 0.1      # m, plus the road edge's std
EDGE_STD_MAX = 0.3     # m: road edges the model is less sure of are ignored
EDGE_FACTORS = (1.0, 0.75, 0.5, 0.25)
HISTORY = 150          # commands kept, at DT_CTRL (1.5 s)
PROPORTIONAL_TC = 1.0  # s: the proportional part follows the offset this slowly


class LaneCentering:
  def __init__(self) -> None:
    self.lane_width = LANE_WIDTH_INIT
    self.curvature = 0.0  # correction added to the model's desired curvature (1/m)
    self.target = 0.0
    self.offset = 0.0     # plan minus lane centre at the aim point (m)
    self.weight = 0.0
    self.integral = 0.0   # 1/m
    self.held = False     # integral stopped: the car lagged the command
    self.recovery = 1.0   # share of the proportional part kept by the recovery limit
    self.edge = 1.0       # share of the correction kept by the road edge limit
    self.centre_x = np.zeros(0)  # lane centre path of the last modelV2 (m)
    self.centre_y = np.zeros(0)
    self.history: deque[tuple[float, float]] = deque(maxlen=HISTORY)  # (model's curvature, command)
    self.proportional = 0.0  # 1/m, smoothed

  def compute(self, model_v2, v_ego: float) -> tuple[float, float, float]:
    """(correction curvature before the fade, offset, weight) for one modelV2 message; learns the lane width."""
    self.centre_x = np.zeros(0)
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
    xs = np.asarray(lines[1].x, dtype=float)
    self.centre_x = xs
    self.centre_y = (p_left * (np.asarray(lines[1].y, dtype=float) + half) +
                     p_right * (np.interp(xs, lines[2].x, lines[2].y) - half)) / (p_left + p_right)
    offset = float(np.interp(dist, pos.x, pos.y)) - centre
    weight = WEIGHT * (p_left + p_right - both) * (1.0 - both * (1.0 - float(np.interp(width, WIDTH_BP, WIDTH_V)))) \
             * float(np.interp(v_ego, SPEED_BP, SPEED_V)) * float(np.interp(self.lane_width, NARROW_BP, NARROW_V))
    error = -np.sign(offset) * max(abs(offset) - DEADBAND, 0.0)  # toward the centre
    limit = MAX_LAT_ACCEL / max(v_ego, 1.0) ** 2
    return float(np.clip(weight * 2.0 * error / dist ** 2, -limit, limit)), offset, weight

  @staticmethod
  def predict(measured: float, v_ego: float, lat_delay: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Distances ahead, the path at the measured curvature, and how far a commanded curvature of 1 moves it: the car
    keeps the measured curvature for lat_delay, then takes the command over HANDOFF_T."""
    horizon = float(np.clip(v_ego * HORIZON_T, HORIZON_MIN, HORIZON_MAX))
    ds = horizon / 16
    x = (np.arange(16) + 0.5) * ds
    blend = np.clip((x / max(v_ego, 1.0) - lat_delay) / HANDOFF_T, 0.0, 1.0)
    return x, np.cumsum(np.cumsum(np.full(16, measured) * ds) * ds), np.cumsum(np.cumsum(blend * ds) * ds)

  def path(self, base: float, measured: float, v_ego: float, lat_delay: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Distances ahead, the predicted path at a command of base, and how far 1 more of command moves it."""
    x, y_measured, moved = self.predict(measured, v_ego, lat_delay)
    return x, y_measured + (base - measured) * moved, moved

  def lagging(self, push: float, curvature: float, v_ego: float, lat_delay: float) -> bool:
    if not len(self.history):
      return False
    past = self.history[-int(np.clip(round(lat_delay / DT_CTRL), 1, len(self.history)))]
    lag = (float(np.clip(curvature, min(past), max(past))) - curvature) * v_ego ** 2
    return lag * push > 0.0 and abs(lag) > TRACK_LAG_MAX

  def recovery_share(self, base: float, push: float, curvature: float, v_ego: float, lat_delay: float) -> float:
    if len(self.centre_x) < 2 or push == 0.0:
      return 1.0
    x, y, moved = self.path(base, curvature, v_ego, lat_delay)
    error = y - np.interp(x, self.centre_x, self.centre_y)
    delta = push * moved
    sens = float(delta @ delta)
    return 1.0 if sens < 1e-12 else float(np.clip(-float(error @ delta) / sens, 0.0, 1.0))

  def edge_share(self, model_v2, base: float, correction: float, curvature: float, v_ego: float, lat_delay: float) -> float:
    edges, stds = model_v2.roadEdges, model_v2.roadEdgeStds
    if correction == 0.0 or len(edges) < 2 or len(stds) < 2:
      return 1.0
    x, y, moved = self.path(base, curvature, v_ego, lat_delay)
    checks = []
    for i, side in ((0, 1.0), (1, -1.0)):  # clearance: body's left side minus left edge, right edge minus right side
      if side * correction > 0.0 or stds[i] > EDGE_STD_MAX or len(edges[i].x) < 2:
        continue  # moving away from it, or the model isn't sure of it
      edge = np.interp(x, edges[i].x, edges[i].y)
      margin = EDGE_MARGIN + float(stds[i])
      checks.append((side, edge, min(float(np.min(side * (y - edge))) - BODY_HALF_WIDTH, margin), margin))
    for f in EDGE_FACTORS:
      y_f = y + f * correction * moved
      if all(min(float(np.min(side * (y_f - edge))) - BODY_HALF_WIDTH, margin) >= kept - 1e-6 for side, edge, kept, margin in checks):
        return f
    return 0.0

  def update(self, enabled: bool, model_v2, model_updated: bool, v_ego: float, steering_pressed: bool,
             curvature: float | None = None, model_curvature: float | None = None, lat_delay: float | None = None) -> float:
    """curvature: measured; model_curvature: the model's desired curvature; lat_delay: steering delay (s). Without
    them the held wheel, recovery and road edge limits are off."""
    active = enabled and not steering_pressed and model_v2.meta.laneChangeState == LaneChangeState.off
    limit = MAX_LAT_ACCEL / max(v_ego, 1.0) ** 2
    if model_updated:
      p, self.offset, self.weight = self.compute(model_v2, v_ego)
      if active and self.weight > 0.0:
        self.proportional += (p - self.proportional) * DT_MDL / PROPORTIONAL_TC
      else:
        self.proportional = 0.0
      p = self.proportional
      self.held, self.recovery, self.edge = False, 1.0, 1.0
      if active and self.weight > 0.0:
        grow = p * DT_MDL / INTEGRAL_T
        if curvature is not None and model_curvature is not None and lat_delay is not None:
          if self.lagging(p, curvature, v_ego, lat_delay):
            grow, self.held = 0.0, True
          self.recovery = self.recovery_share(model_curvature + self.integral, p, curvature, v_ego, lat_delay)
          if self.recovery < 1.0:
            grow, p = 0.0, p * self.recovery
          target = float(np.clip(p + self.integral + grow, -limit, limit))
          self.edge = self.edge_share(model_v2, model_curvature, target, curvature, v_ego, lat_delay)
          if self.edge < 1.0:
            grow = 0.0
        self.integral += grow - self.integral * DT_MDL / LEAK_T
      else:
        self.integral = 0.0
      self.integral = float(np.clip(self.integral, -limit, limit))
      self.target = float(np.clip(p + self.integral, -limit, limit)) * self.edge
    if not active:
      self.target, self.weight, self.integral = 0.0, 0.0, 0.0
      self.history.clear()
    step = MAX_LAT_JERK / max(v_ego, 1.0) ** 2 * DT_CTRL
    self.curvature += float(np.clip(self.target - self.curvature, -step, step))
    if active and model_curvature is not None:
      self.history.append((model_curvature, model_curvature + self.curvature))
    return self.curvature
