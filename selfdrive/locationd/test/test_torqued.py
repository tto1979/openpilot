import numpy as np

from cereal import car, custom
from openpilot.common.params import Params
from openpilot.selfdrive.locationd.torqued import TorqueEstimator


def test_cal_percent():
  est = TorqueEstimator(car.CarParams())
  msg = est.get_msg()
  assert msg.liveTorqueParameters.calPerc == 0

  for (low, high), min_pts in zip(est.filtered_points.buckets.keys(),
                                  est.filtered_points.buckets_min_points.values(), strict=True):
    for _ in range(int(min_pts)):
      est.filtered_points.add_point((low + high) / 2.0, 0.0)

  # enough bucket points, but not enough total points
  msg = est.get_msg()
  assert msg.liveTorqueParameters.calPerc == (len(est.filtered_points) / est.min_points_total * 100 + 100) / 2

  # add enough points to bucket with most capacity
  key = list(est.filtered_points.buckets)[0]
  for _ in range(est.min_points_total - len(est.filtered_points)):
    est.filtered_points.add_point((key[0] + key[1]) / 2.0, 0.0)

  msg = est.get_msg()
  assert msg.liveTorqueParameters.calPerc == 100


def _torque_cp():
  CP = car.CarParams.new_message()
  CP.brand = "toyota"
  CP.lateralTuning.init('torque')
  CP.lateralTuning.torque.latAccelFactor = 2.0
  CP.lateralTuning.torque.friction = 0.1
  return CP


def _learn(est):
  # a car that needs 20% more torque and has 30% more friction than the offline params
  rng = np.random.default_rng(0)
  for (low, high), min_pts in zip(est.filtered_points.buckets.keys(), est.filtered_points.buckets_min_points.values(), strict=True):
    for _ in range(int(min_pts) * 3):
      steer = rng.uniform(low, high)
      est.filtered_points.add_point(steer, steer * 2.0 / 1.2 + rng.normal(0.0, 0.1))
  for _ in range(200):
    msg = est.get_msg()
  return msg.liveTorqueParameters


def _car_params_top(nnff, model):
  CP_TOP = custom.CarParamsTOP.new_message()
  CP_TOP.twilsoncoNNFF = nnff
  CP_TOP.nnModelName = model
  params = Params()
  params.put_bool("NNFF", nnff)
  params.put("CarParamsTOP", CP_TOP.to_bytes())


def test_learns_with_the_nnff_toggle_on_but_no_nn_model():
  _car_params_top(True, "")  # toggle on, no model for this car: official controller
  ltp = _learn(TorqueEstimator(_torque_cp().as_reader()))
  assert ltp.liveValid and ltp.latAccelFactorFiltered < 1.99


def test_no_learning_while_nnff_runs():
  _car_params_top(True, "TOYOTA_PRIUS_V")
  ltp = _learn(TorqueEstimator(_torque_cp().as_reader()))
  assert ltp.latAccelFactorFiltered == 2.0 and abs(ltp.frictionCoefficientFiltered - 0.1) < 1e-6
