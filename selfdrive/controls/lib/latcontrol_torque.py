from collections import deque
import math
import numpy as np

from cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from opendbc.car.tests.test_lateral_limits import MAX_LAT_JERK_UP
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, MIN_SPEED
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]
LOW_SPEED_Y_NN = [12, 3, 1, 0]

LAT_PLAN_MIN_IDX = 5
LATERAL_LAG_MOD = 0.1

def get_predicted_lateral_jerk(lat_accels, t_diffs):
  # compute finite difference between subsequent model_data.acceleration.y values
  # this is just two calls of np.diff followed by an element-wise division
  lat_accel_diffs = np.diff(lat_accels)
  lat_jerk = lat_accel_diffs / t_diffs
  # return as python list
  return lat_jerk.tolist()

def sign(x):
  return 1.0 if x > 0.0 else (-1.0 if x < 0.0 else 0.0)

def get_lookahead_value(future_vals, current_val):
  if len(future_vals) == 0:
    return current_val

  same_sign_vals = [v for v in future_vals if sign(v) == sign(current_val)]

  # if any future val has opposite sign of current val, return 0
  if len(same_sign_vals) < len(future_vals):
    return 0.0

  # otherwise return the value with minimum absolute value
  min_val = min(same_sign_vals + [current_val], key=lambda x: abs(x))
  return min_val

# At a given roll, if pitch magnitude increases, the
# gravitational acceleration component starts pointing
# in the longitudinal direction, decreasing the lateral
# acceleration component. Here we do the same thing
# to the roll value itself, then passed to nnff.
def roll_pitch_adjust(roll, pitch):
  return roll * math.cos(pitch)

class LatControlTorque(LatControl):
  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki,
                             k_f=self.torque_params.kf, rate=1/self.dt)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.LATACCEL_REQUEST_BUFFER_NUM_FRAMES = int(1 / self.dt)
    self.requested_lateral_accel_buffer = deque(
        [0.] * self.LATACCEL_REQUEST_BUFFER_NUM_FRAMES,
        maxlen=self.LATACCEL_REQUEST_BUFFER_NUM_FRAMES)
    self.previous_measurement = 0.0
    self.measurement_rate_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * (MAX_LAT_JERK_UP - 0.5)), self.dt)

    # Twilsonco's Lateral Neural Network Feedforward
    self.use_nn = CI.has_lateral_torque_nn if hasattr(CI, 'has_lateral_torque_nn') else False
    self.use_lateral_jerk = False  # self.param_s.get_bool("TorqueLateralJerk")

    # NNFF: Lateral jerk lookahead configuration
    if self.use_nn or self.use_lateral_jerk:
      # Scaling the lateral acceleration "friction response" could be helpful for some.
      # Increase for a stronger response, decrease for a weaker response.
      nnff_lateral_jerk_factor = 1.0  # TODO: read from params
      nnff_lateral_jerk_factor = max(0.0, min(1.0, nnff_lateral_jerk_factor))
      self.lat_jerk_friction_factor = 0.4 * nnff_lateral_jerk_factor
      # Increasing lat accel friction factor to account for any decrease of the lat jerk friction factor from default
      self.lat_accel_friction_factor = 0.7 + (0.3 * (1.0 - nnff_lateral_jerk_factor))  # in [0, 3], in 0.05 increments. 3 is arbitrary safety limit

      # Lookahead configuration (speed-dependent)
      self.friction_look_ahead_v = [1.4, 2.0]   # seconds
      self.friction_look_ahead_bp = [9.0, 30.0]  # m/s

      # Precompute time differences for jerk calculation
      self.t_diffs = np.diff(ModelConstants.T_IDXS)
      self.desired_lat_jerk_time = CP.steerActuatorDelay + LATERAL_LAG_MOD

    # NNFF: Neural network specific setup
    if self.use_nn:
      self.pitch = FirstOrderFilter(0.0, 0.5, 0.01)
      # NN model takes current v_ego, lateral_accel, lat accel/jerk error, roll, and past/future/planned data
      # of lat accel and roll
      # Past value is computed using previous desired lat accel and observed roll
      self.torque_from_nn = CI.get_ff_nn
      self.nn_friction_override = CI.lat_torque_nn_model.friction_override

      # setup future time offsets
      self.future_times = [0.3, 0.6, 1.0, 1.5]  # seconds in the future
      self.nn_future_times = [t + self.desired_lat_jerk_time for t in self.future_times]
      self.nn_future_times_np = np.array(self.nn_future_times)

      # setup past time offsets
      self.past_times = [-0.3, -0.2, -0.1]
      history_check_frames = [int(abs(t) * 100) for t in self.past_times]
      self.history_frame_offsets = [history_check_frames[0] - f for f in history_check_frames]
      self.lateral_accel_desired_deque = deque(maxlen=history_check_frames[0])
      self.roll_deque = deque(maxlen=history_check_frames[0])

  def update_lateral_lag(self, lag):
    self.desired_lat_jerk_time = max(0.01, lag) + LATERAL_LAG_MOD

    if self.use_nn:
      self.nn_future_times = [t + self.desired_lat_jerk_time for t in self.future_times]
      self.nn_future_times_np = np.array(self.nn_future_times)

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def reset(self):
    super().reset()
    # Clear buffer to avoid stale data after disengagement
    if hasattr(self, 'requested_lateral_accel_buffer'):
      self.requested_lateral_accel_buffer.clear()
      for _ in range(self.LATACCEL_REQUEST_BUFFER_NUM_FRAMES):
        self.requested_lateral_accel_buffer.append(0.0)

    if hasattr(self, 'previous_measurement'):
      self.previous_measurement = 0.0
    if hasattr(self, 'measurement_rate_filter'):
      self.measurement_rate_filter.x = 0.0

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay, model_data=None):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    nn_log = None

    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      delay_frames = int(np.clip(lat_delay / self.dt, 1, self.LATACCEL_REQUEST_BUFFER_NUM_FRAMES))
      expected_lateral_accel = self.requested_lateral_accel_buffer[-delay_frames]
      # TODO factor out lateral jerk from error to later replace it with delay independent alternative
      future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      self.requested_lateral_accel_buffer.append(future_desired_lateral_accel)
      gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
      desired_lateral_jerk = (future_desired_lateral_accel - expected_lateral_accel) / lat_delay

      measurement = measured_curvature * CS.vEgo ** 2
      raw_measurement_rate = (measurement - self.previous_measurement) / self.dt
      measurement_rate = self.measurement_rate_filter.update(raw_measurement_rate)
      self.previous_measurement = measurement

      low_speed_factor = (np.interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y if not self.use_nn else LOW_SPEED_Y_NN) / max(CS.vEgo, MIN_SPEED)) ** 2
      setpoint = lat_delay * desired_lateral_jerk + expected_lateral_accel
      error = setpoint - measurement
      error_lsf = error + low_speed_factor / self.torque_params.kp * error

      lookahead_lateral_jerk = 0.0
      model_good = (model_data is not None and
                    hasattr(model_data, 'orientation') and
                    hasattr(model_data.orientation, 'x') and
                    len(list(model_data.orientation.x)) >= CONTROL_N)

      if model_good and (self.use_nn or self.use_lateral_jerk):
        # prepare "look-ahead" desired lateral jerk
        lookahead = np.interp(CS.vEgo, self.friction_look_ahead_bp, self.friction_look_ahead_v)
        friction_upper_idx = next((i for i, val in enumerate(ModelConstants.T_IDXS) if val > lookahead), 16)
        predicted_lateral_jerk = get_predicted_lateral_jerk(model_data.acceleration.y, self.t_diffs)
        desired_lateral_jerk_model = (np.interp(self.desired_lat_jerk_time, ModelConstants.T_IDXS, model_data.acceleration.y) - future_desired_lateral_accel) / self.desired_lat_jerk_time
        lookahead_lateral_jerk = get_lookahead_value(predicted_lateral_jerk[LAT_PLAN_MIN_IDX:friction_upper_idx], desired_lateral_jerk_model)

      lat_accel_friction_factor = (self.lat_accel_friction_factor if lookahead_lateral_jerk != 0.0 else 1.0) if (self.use_nn or self.use_lateral_jerk) else 1.0

      if self.use_nn and model_good:
        pitch = 0.0
        roll = params.roll
        if (model_data is not None and hasattr(model_data.orientation, 'y') and len(model_data.orientation.y) > 0):
          pitch = self.pitch.update(model_data.orientation.y[0])
          roll = roll_pitch_adjust(roll, pitch)
        self.roll_deque.append(roll)
        self.lateral_accel_desired_deque.append(future_desired_lateral_accel)

        past_rolls = [self.roll_deque[min(len(self.roll_deque) - 1, offset)] for offset in self.history_frame_offsets]
        past_lateral_accels_desired = [self.lateral_accel_desired_deque[min(len(self.lateral_accel_desired_deque) - 1, offset)] for offset in self.history_frame_offsets]
        adjusted_future_times = [t + 0.5 * CS.aEgo * (t / max(CS.vEgo, 1.0)) for t in self.nn_future_times]
        T_IDXS = [float(x) for x in ModelConstants.T_IDXS[:CONTROL_N]]
        lateral_accels = [float(y) for y in list(model_data.acceleration.y)[:CONTROL_N]]

        future_rolls = [
            roll_pitch_adjust(
                np.interp(t, ModelConstants.T_IDXS, model_data.orientation.x) + roll,
                np.interp(t, ModelConstants.T_IDXS, model_data.orientation.y) + pitch
            )
            for t in adjusted_future_times
        ]
        future_planned_lateral_accels = [
            np.interp(float(t), T_IDXS, lateral_accels)
            for t in adjusted_future_times
        ]

        friction_input = (
            lat_accel_friction_factor * error_lsf +
            self.lat_jerk_friction_factor * lookahead_lateral_jerk
        )
        nn_input = [CS.vEgo, future_desired_lateral_accel, friction_input, roll] + \
                   past_lateral_accels_desired + future_planned_lateral_accels + \
                   past_rolls + future_rolls

        nn_torque = self.torque_from_nn(nn_input)
        nn_lat_accel = self.lateral_accel_from_torque(nn_torque, self.torque_params)

        if self.nn_friction_override:
          ff = nn_lat_accel
        else:
          friction_torque = get_friction(error, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)
          friction_lat_accel = self.lateral_accel_from_torque(friction_torque, self.torque_params)
          ff = nn_lat_accel + friction_lat_accel

        ff -= self.torque_params.latAccelOffset
        pid_log.error = float(error_lsf)
        nn_log = nn_input
      else:
        ff = gravity_adjusted_future_lateral_accel
        # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
        ff -= self.torque_params.latAccelOffset

        if self.use_lateral_jerk and model_good:
          friction_input = (lat_accel_friction_factor * error_lsf + self.lat_jerk_friction_factor * lookahead_lateral_jerk)
          friction_torque = get_friction(friction_input, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)
          ff += self.lateral_accel_from_torque(friction_torque, self.torque_params)
        else:
          # TODO jerk is weighted by lat_delay for legacy reasons, but should be made independent of it
          ff += get_friction(error, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

        pid_log.error = float(error_lsf)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      output_lataccel = self.pid.update(pid_log.error,
                                       -measurement_rate,
                                        feedforward=ff,
                                        speed=CS.vEgo,
                                        freeze_integrator=freeze_integrator)
      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque)  # TODO: log lat accel?
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))
      if nn_log is not None:
        pid_log.nnLog = [float(x) for x in nn_log]
      else:
        pid_log.nnLog = [0.0]

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
