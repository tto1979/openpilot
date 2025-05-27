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
from openpilot.common.hermite_utils import create_hermite_interpolator

AccelPersonality = custom.LongitudinalPlanTOP.AccelerationPersonality

# Accel personality by @arne182 modified by ct921
_DP_CRUISE_MAX_V_ECO =    [2.0, 1.7, 1.0,  .85, .70, .60, .44, .32, .22, .16, .01]
_DP_CRUISE_MAX_V_NORMAL = [2.2, 1.9, 1.4, 1.22, .95, .83, .71, .54, .45, .38, .15]
_DP_CRUISE_MAX_V_SPORT =  [3.0, 2.4, 2.0, 1.55, 1.3, 1.1, .92, .75, .63, .55, .25]
# CRUISE_MAX_BP in kmh =  [0.,  3,   10,   20,   30,  40,  53,  72,  90,  107, 150]
_DP_CRUISE_MAX_BP =       [0.,  1,   3.,   6.,   8.,  11., 15., 20., 25., 30., 55.]

# Create interpolators for all modes
_DP_CRUISE_MAX_INTERPOLATORS = {}

# Initialize max interpolators
for mode, values in [('eco', _DP_CRUISE_MAX_V_ECO),
                     ('normal', _DP_CRUISE_MAX_V_NORMAL),
                     ('sport', _DP_CRUISE_MAX_V_SPORT)]:
  _DP_CRUISE_MAX_INTERPOLATORS[mode], _ = create_hermite_interpolator(_DP_CRUISE_MAX_BP, values)


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
    mode = 'eco' if self._personality == AccelPersonality.eco else \
           'sport' if self._personality == AccelPersonality.sport else 'normal'

    # Select appropriate values for fallback
    max_vals = _DP_CRUISE_MAX_V_ECO if mode == 'eco' else \
               _DP_CRUISE_MAX_V_SPORT if mode == 'sport' else \
               _DP_CRUISE_MAX_V_NORMAL

    # Try to use Hermite interpolator if available
    if mode in _DP_CRUISE_MAX_INTERPOLATORS and _DP_CRUISE_MAX_INTERPOLATORS[mode] is not None:
      try:
        a_cruise_max = float(_DP_CRUISE_MAX_INTERPOLATORS[mode](v_ego))
        return a_cruise_max
      except Exception as e:
        # Log error only once per minute to avoid spam
        current_time = int(self._frame * DT_MDL)
        if current_time - self._last_error_log > 60:
          cloudlog.warning(f"Hermite interpolation failed for {mode} mode at v_ego={v_ego:.1f}: {e}")
          self._last_error_log = current_time
        # Fall through to linear interpolation

    # Fallback to linear interpolation
    return float(interp(v_ego, _DP_CRUISE_MAX_BP, max_vals))

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
