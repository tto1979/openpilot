import cereal.messaging as messaging
from cereal import car, custom
from openpilot.common.params import Params


def get_car_params_top(params: Params | None = None) -> custom.CarParamsTOP:
  """CarParamsTOP as written by card next to CarParams. Falls back to the schema defaults if it's missing."""
  dat = (params or Params()).get("CarParamsTOP")
  if dat is None:
    return custom.CarParamsTOP.new_message().as_reader()
  return messaging.log_from_bytes(dat, custom.CarParamsTOP)


def nnff_active(CP: car.CarParams, CP_TOP: custom.CarParamsTOP) -> bool:
  """NNFF is on and this car has an NN model, so LatControlTorque runs the NN path, not the official torque controller."""
  return bool(CP_TOP.twilsoncoNNFF and CP_TOP.nnModelName and CP.lateralTuning.which() == 'torque')
