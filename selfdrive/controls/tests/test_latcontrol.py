import numpy as np
from openpilot.common.parameterized import parameterized

from cereal import car, log
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.nissan.values import CAR as NISSAN
from opendbc.car.gm.values import CAR as GM
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from openpilot.selfdrive.locationd.helpers import Pose
from openpilot.common.mock.generators import generate_livePose
from openpilot.selfdrive.modeld.constants import ModelConstants


class TestLatControl:

  @parameterized.expand([(HONDA.HONDA_CIVIC, LatControlPID), (TOYOTA.TOYOTA_RAV4, LatControlTorque),
                         (NISSAN.NISSAN_LEAF, LatControlAngle), (GM.CHEVROLET_BOLT_EUV, LatControlTorque)])
  def test_saturation(self, car_name, controller):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CI = CarInterface(CP)
    VM = VehicleModel(CP)

    controller = controller(CP.as_reader(), CI, DT_CTRL)

    CS = car.CarState.new_message()
    CS.vEgo = 30
    CS.steeringPressed = False

    params = log.LiveParametersData.new_message()

    lp = generate_livePose()
    pose = Pose.from_live_pose(lp.livePose)

    # Saturate for curvature limited and controller limited
    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, VM, params, False, 0, pose, True, 0.2)
    assert lac_log.saturated

    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, VM, params, False, 0, pose, False, 0.2)
    assert not lac_log.saturated

    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, VM, params, False, 1, pose, False, 0.2)
    assert lac_log.saturated

  def test_nnff(self):
    # TOP: with the NNFF flag the interface loads its NN model and LatControlTorque runs the NN path
    platform = TOYOTA.TOYOTA_RAV4_TSS2
    CarInterface = interfaces[platform]
    fw = car.CarParams.CarFw.new_message(ecu="eps", fwVersion=b"8965B42170\x00\x00\x00\x00\x00\x00")
    CP = CarInterface.get_params(platform, gen_empty_fingerprint(), [fw], False, False, structs.TopFlags.NNFF, False)
    CP.carFw = [fw]
    CI = CarInterface(CP, structs.TopFlags.NNFF)
    assert CI.has_lateral_torque_nn

    controller = LatControlTorque(CP.as_reader(), CI, DT_CTRL)
    assert controller.use_nn

    VM = VehicleModel(CP)
    CS = car.CarState.new_message()
    CS.vEgo = 20
    params = log.LiveParametersData.new_message()
    pose = Pose.from_live_pose(generate_livePose().livePose)

    desired_curvature = 0.004
    model = log.ModelDataV2.new_message()
    n = len(ModelConstants.T_IDXS)
    model.orientation.x, model.orientation.y = [0.] * n, [0.] * n
    model.acceleration.y = [desired_curvature * CS.vEgo ** 2] * n

    outputs = []
    for _ in range(300):
      output, _, lac_log = controller.update(True, CS, VM, params, False, desired_curvature, pose, False, 0.2, model)
      outputs.append(output)
    assert controller.pid is controller.pid_nnff
    assert lac_log.active
    assert np.all(np.isfinite(outputs)) and np.max(np.abs(outputs)) <= controller.steer_max
