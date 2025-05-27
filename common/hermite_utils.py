"""
Hermite interpolation utilities for smooth acceleration/deceleration curves
"""
Hermite interpolation utilities for smooth acceleration/deceleration curves
"""
Hermite interpolation utilities for smooth acceleration/deceleration curves
import numpy as np
from openpilot.common.swaglog import cloudlog


def compute_symmetric_slopes(x, y):
  """
  Compute symmetric slopes for Hermite interpolation

  Args:
    x: Array of x coordinates (must be monotonically increasing)
    y: Array of y values corresponding to x

  Returns:
    Array of slopes at each point
  """
  n = len(x)
  if n < 2:
    raise ValueError("At least two points are required to compute slopes")

  if len(x) != len(y):
    raise ValueError(f"x and y must have same length, got x:{len(x)}, y:{len(y)}")

  m = np.zeros(n)
  for i in range(n):
    if i == 0:
      # Forward difference for first point
      m[i] = (y[i+1] - y[i]) / (x[i+1] - x[i])
    elif i == n-1:
      # Backward difference for last point
      m[i] = (y[i] - y[i-1]) / (x[i] - x[i-1])
    else:
      # Average of forward and backward differences for interior points
      m[i] = ((y[i+1] - y[i]) / (x[i+1] - x[i]) + (y[i] - y[i-1]) / (x[i] - x[i-1])) / 2
  return m


def hermite_interpolate(x, xp, yp, slopes):
  """
  Perform Hermite cubic interpolation

  Args:
    x: Point(s) at which to evaluate the interpolation
    xp: Array of x coordinates of known points
    yp: Array of y values at known points
    slopes: Array of slopes at known points

  Returns:
    Interpolated value(s) at x
  """
  # Input validation
  if len(xp) != len(yp) or len(xp) != len(slopes):
    raise ValueError("xp, yp and slopes must have same length")

  if len(xp) < 2:
    raise ValueError("At least two points are required for interpolation")

  # Handle scalar and array inputs
  x_scalar = np.isscalar(x)
  x = np.atleast_1d(x)

  # Clip x to domain
  x = np.clip(x, xp[0], xp[-1])

  # Initialize output
  result = np.zeros_like(x, dtype=float)

  # Vectorized interpolation
  for j, x_val in enumerate(x):
    # Find interval
    idx = np.searchsorted(xp, x_val) - 1
    idx = np.clip(idx, 0, len(slopes) - 2)

    x0, x1 = xp[idx], xp[idx+1]
    y0, y1 = yp[idx], yp[idx+1]
    m0, m1 = slopes[idx], slopes[idx+1]

    # Avoid division by zero
    if x1 - x0 == 0:
      result[j] = y0
      continue

    # Hermite basis functions
    t = (x_val - x0) / (x1 - x0)
    h00 = 2*t**3 - 3*t**2 + 1
    h10 = t**3 - 2*t**2 + t
    h01 = -2*t**3 + 3*t**2
    h11 = t**3 - t**2

    result[j] = (h00 * y0) + (h10 * (x1 - x0) * m0) + (h01 * y1) + (h11 * (x1 - x0) * m1)

  return float(result[0]) if x_scalar else result


def create_hermite_interpolator(xp, yp):
  """
  Create a Hermite interpolator function with pre-computed slopes

  Args:
    xp: Array of x coordinates of known points
    yp: Array of y values at known points

  Returns:
    Interpolator function that takes x and returns interpolated y
  """
  try:
    slopes = compute_symmetric_slopes(xp, yp)

    def interpolator(x):
      try:
        return hermite_interpolate(x, xp, yp, slopes)
      except Exception as e:
        cloudlog.warning(f"Hermite interpolation failed: {e}, falling back to linear")
        return np.interp(x, xp, yp)

    return interpolator, True
  except ValueError as e:
    cloudlog.warning(f"Cannot create Hermite interpolator: {e}, using linear interpolation")
    return lambda x: np.interp(x, xp, yp), False
