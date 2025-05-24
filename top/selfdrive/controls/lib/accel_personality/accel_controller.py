# The MIT License
#
# Copyright (c) 2019-, Rick Lan, dragonpilot community, and a number of other contributors.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# Last updated: February 20, 2025
import numpy as np
from cereal import custom
from numpy import interp
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params

AccelPersonality = custom.LongitudinalPlanTOP.AccelerationPersonality

# Hermite interpolation functions
def compute_symmetric_slopes(x, y):
  n = len(x)
  if n < 2:
    raise ValueError("At least two points are required to compute slopes")

  if len(x) != len(y):
    raise ValueError(f"x and y must have same length, got x:{len(x)}, y:{len(y)}")

  m = np.zeros(n)
  for i in range(n):
    if i == 0:
      m[i] = (y[i+1] - y[i]) / (x[i+1] - x[i])
    elif i == n-1:
      m[i] = (y[i] - y[i-1]) / (x[i] - x[i-1])
    else:
      m[i] = ((y[i+1] - y[i]) / (x[i+1] - x[i]) + (y[i] - y[i-1]) / (x[i] - x[i-1])) / 2
  return m

def hermite_interpolate(x, xp, yp, slopes):
  # Safety checks
  if len(xp) != len(yp) or len(xp) != len(slopes):
    raise ValueError("xp, yp and slopes must have same length")

  if len(xp) < 2:
    raise ValueError("At least two points are required for interpolation")

  # Clip x to domain
  x = np.clip(x, xp[0], xp[-1])

  # Find interval
  idx = np.searchsorted(xp, x) - 1
  idx = np.clip(idx, 0, len(slopes) - 2)

  x0, x1 = xp[idx], xp[idx+1]
  y0, y1 = yp[idx], yp[idx+1]
  m0, m1 = slopes[idx], slopes[idx+1]

  # Avoid division by zero
  if x1 - x0 == 0:
    return float(y0)

  t = (x - x0) / (x1 - x0)
  h00 = 2*t**3 - 3*t**2 + 1
  h10 = t**3 - 2*t**2 + t
  h01 = -2*t**3 + 3*t**2
  h11 = t**3 - t**2

  interpolated = (h00 * y0) + (h10 * (x1 - x0) * m0) + (h01 * y1) + (h11 * (x1 - x0) * m1)
  return float(interpolated)

_DP_CRUISE_MIN_V_ECO =    [-0.01, -0.01, -0.10, -1.2]
_DP_CRUISE_MIN_V_NORMAL = [-0.015, -0.015, -0.12, -1.21]
_DP_CRUISE_MIN_V_SPORT =  [-0.02, -0.02, -0.14, -1.22]
_DP_CRUISE_MIN_BP =       [0.,     2.0,  11,   25.]
_DP_CRUISE_MAX_V_ECO =    [2.0, 1.7, 1.0,  .85, .70, .60, .44, .32, .22, .16, .01]
_DP_CRUISE_MAX_V_NORMAL = [2.2, 1.9, 1.4, 1.22, .95, .83, .71, .54, .45, .38, .15]
_DP_CRUISE_MAX_V_SPORT =  [3.0, 2.4, 2.0, 1.55, 1.3, 1.1, .92, .75, .63, .55, .25]
# CRUISE_MAX_BP in kmh =  [0.,  3,   10,   20,   30,  40,  53,  72,  90,  107, 150]
_DP_CRUISE_MAX_BP =       [0.,  1,   3.,   6.,   8.,  11., 15., 20., 25., 30., 55.]

# Pre-compute slopes for all modes with error handling
try:
  _DP_CRUISE_MAX_SLOPES = {
    'eco': compute_symmetric_slopes(_DP_CRUISE_MAX_BP, _DP_CRUISE_MAX_V_ECO),
    'normal': compute_symmetric_slopes(_DP_CRUISE_MAX_BP, _DP_CRUISE_MAX_V_NORMAL),
    'sport': compute_symmetric_slopes(_DP_CRUISE_MAX_BP, _DP_CRUISE_MAX_V_SPORT),
  }

  _DP_CRUISE_MIN_SLOPES = {
    'eco': compute_symmetric_slopes(_DP_CRUISE_MIN_BP, _DP_CRUISE_MIN_V_ECO),
    'normal': compute_symmetric_slopes(_DP_CRUISE_MIN_BP, _DP_CRUISE_MIN_V_NORMAL),
    'sport': compute_symmetric_slopes(_DP_CRUISE_MIN_BP, _DP_CRUISE_MIN_V_SPORT),
  }
  USE_HERMITE = True
except ValueError as e:
  print(f"Warning: Cannot compute Hermite slopes, will use linear interpolation: {e}")
  USE_HERMITE = False

class AccelController:
  def __init__(self):
    self._params = Params()
    self._personality = AccelPersonality.stock
    self._frame = 0

  def _read_params(self):
    if self._frame % int(1. / DT_MDL) == 0:
      personality_str = self._params.get("AccelPersonality", encoding='utf-8')
      if personality_str is not None:
        try:
          personality_int = int(personality_str)
          if personality_int in [AccelPersonality.stock, AccelPersonality.normal, 
                               AccelPersonality.eco, AccelPersonality.sport]:
            self._personality = personality_int
        except ValueError:
          pass  # Keep current personality

  def _dp_calc_cruise_accel_limits(self, v_ego: float) -> float:
    self._read_params()

    if self._personality == AccelPersonality.stock:
      # Stock mode always uses linear interpolation
      return float(interp(v_ego, _DP_CRUISE_MAX_BP, _DP_CRUISE_MAX_V_NORMAL))

    # Determine mode
    if self._personality == AccelPersonality.eco:
      mode = 'eco'
      max_v = _DP_CRUISE_MAX_V_ECO
    elif self._personality == AccelPersonality.sport:
      mode = 'sport'
      max_v = _DP_CRUISE_MAX_V_SPORT
    else:
      mode = 'normal'
      max_v = _DP_CRUISE_MAX_V_NORMAL

    # Fall back to linear interpolation if Hermite fails
    if USE_HERMITE:
      try:
        a_cruise_max = hermite_interpolate(
          v_ego, 
          _DP_CRUISE_MAX_BP,
          max_v,
          _DP_CRUISE_MAX_SLOPES[mode]
        )
      except Exception as e:
        print(f"Hermite interpolation failed, using linear: {e}")
        a_cruise_max = float(interp(v_ego, _DP_CRUISE_MAX_BP, max_v))
    else:
      a_cruise_max = float(interp(v_ego, _DP_CRUISE_MAX_BP, max_v))

    return a_cruise_max

  def get_accel_limits(self, v_ego: float, accel_limits: list[float]) -> tuple[float, float]:
    self._read_params()

    if self._personality == AccelPersonality.stock:
      return (accel_limits[0], accel_limits[1])
    else:
      a_cruise_max = self._dp_calc_cruise_accel_limits(v_ego)
      return (accel_limits[0], a_cruise_max)

  def is_enabled(self, accel_personality: int = AccelPersonality.stock) -> bool:
    self._personality = accel_personality
    self._read_params()
    enabled: bool = (self._personality != AccelPersonality.stock)
    return enabled

  def update(self):
    self._frame += 1
