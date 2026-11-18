#!/usr/bin/env python3
import os
from openpilot.selfdrive.modeld.helpers import MODELS_DIR, load_oob
from tinygrad.tensor import Tensor
import math
import time
import pickle
import ctypes
import numpy as np

from cereal import messaging
from cereal.messaging import PubMaster, SubMaster
from msgq.visionipc import VisionIpcClient, VisionStreamType, VisionBuf
from openpilot.common.swaglog import cloudlog
from openpilot.common.realtime import config_realtime_process
from openpilot.common.transformations.model import dmonitoringmodel_intrinsics
from openpilot.common.transformations.camera import _ar_ox_fisheye, _os_fisheye
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
from openpilot.selfdrive.modeld.parse_model_outputs import sigmoid

CALIB_LEN = 3
FEATURE_LEN = 512
OUTPUT_SIZE = 83 + FEATURE_LEN

PROCESS_NAME = "selfdrive.modeld.dmonitoringmodeld"
SEND_RAW_PRED = os.getenv('SEND_RAW_PRED')
MODEL_PKL_PATH = MODELS_DIR / 'dmonitoring_model_tinygrad.pkl'

# TODO: slice from meta
class DriverStateResult(ctypes.Structure):
  _fields_ = [
    ("face_orientation", ctypes.c_float*3),
    ("face_position", ctypes.c_float*3),
    ("face_orientation_std", ctypes.c_float*3),
    ("face_position_std", ctypes.c_float*3),
    ("face_prob", ctypes.c_float),
    ("_unused_a", ctypes.c_float*8),
    ("left_eye_prob", ctypes.c_float),
    ("_unused_b", ctypes.c_float*8),
    ("right_eye_prob", ctypes.c_float),
    ("left_blink_prob", ctypes.c_float),
    ("right_blink_prob", ctypes.c_float),
    ("sunglasses_prob", ctypes.c_float),
    ("_unused_c", ctypes.c_float),
    ("_unused_d", ctypes.c_float*4),
    ("not_ready_prob", ctypes.c_float*2)]


class DMonitoringModelResult(ctypes.Structure):
  _fields_ = [
    ("driver_state_lhd", DriverStateResult),
    ("driver_state_rhd", DriverStateResult),
    ("wheel_on_right_prob", ctypes.c_float),
    ("features", ctypes.c_float*FEATURE_LEN)]


class ModelState:
  inputs: dict[str, np.ndarray]
  output: np.ndarray

  def __init__(self, cam_w: int, cam_h: int):
    assert ctypes.sizeof(DMonitoringModelResult) == OUTPUT_SIZE * ctypes.sizeof(ctypes.c_float)

    jits = load_oob(MODEL_PKL_PATH)
    self.DEV = jits['input_specs']['input_img'][2]
    self.numpy_inputs = {
      'calib': np.zeros((1, CALIB_LEN), dtype=np.float32),
    }

    self.warp_inputs_np = {'transform': np.zeros((3,3), dtype=np.float32)}
    self.warp_inputs = {k: Tensor(v, device='NPY') for k,v in self.warp_inputs_np.items()}
    self.frame_buf_params = get_nv12_info(cam_w, cam_h)
    self.tensor_inputs = {k: Tensor(v, device=self.DEV).realize() for k,v in self.numpy_inputs.items()}
    self.calib_host = Tensor(self.numpy_inputs['calib'], device='NPY')._buffer()
    self._blob_cache: dict[int, Tensor] = {}
    self.model_run = jits['run']
    self.outputs = {name: Tensor(np.zeros(shape, dtype=dtype), device=device).realize() for name, (shape, dtype, device) in jits['output_specs'].items()}
    self.output_name = next(iter(self.outputs))
    with open(MODELS_DIR / f'dm_warp_{cam_w}x{cam_h}_tinygrad.pkl', "rb") as f:
      self.image_warp = pickle.load(f)['run']

  def run(self, buf: VisionBuf, calib: np.ndarray, transform: np.ndarray) -> tuple[np.ndarray, float]:
    self.numpy_inputs['calib'][0,:] = calib

    t1 = time.perf_counter()
    self.tensor_inputs['calib']._buffer().copy_from(self.calib_host)

    ptr = np.frombuffer(buf.data, dtype=np.uint8).ctypes.data
    # There is a ringbuffer of imgs, just cache tensors pointing to all of them
    if ptr not in self._blob_cache:
      self._blob_cache[ptr] = Tensor.from_blob(ptr, (self.frame_buf_params[3],), dtype='uint8', device=self.DEV)

    self.warp_inputs_np['transform'][:] = transform[:]
    self.tensor_inputs['input_img'] = self.image_warp(input_frame=self._blob_cache[ptr], M_inv=self.warp_inputs['transform'])

    self.model_run(output_buffers=self.outputs, **self.tensor_inputs)
    output = self.outputs[self.output_name].numpy().astype(np.float32).reshape(-1)

    t2 = time.perf_counter()
    return output, t2 - t1


def fill_driver_state(msg, ds_result: DriverStateResult):
  msg.faceOrientation = list(ds_result.face_orientation)
  msg.faceOrientationStd = [math.exp(x) for x in ds_result.face_orientation_std]
  msg.facePosition = list(ds_result.face_position[:2])
  msg.facePositionStd = [math.exp(x) for x in ds_result.face_position_std[:2]]
  msg.faceProb = float(sigmoid(ds_result.face_prob))
  msg.leftEyeProb = float(sigmoid(ds_result.left_eye_prob))
  msg.rightEyeProb = float(sigmoid(ds_result.right_eye_prob))
  msg.leftBlinkProb = float(sigmoid(ds_result.left_blink_prob))
  msg.rightBlinkProb = float(sigmoid(ds_result.right_blink_prob))
  msg.sunglassesProb = float(sigmoid(ds_result.sunglasses_prob))
  msg.notReadyProb = [float(sigmoid(x)) for x in ds_result.not_ready_prob]


def get_driverstate_packet(model_output: np.ndarray, frame_id: int, location_ts: int, execution_time: float, gpu_execution_time: float):
  model_result = ctypes.cast(model_output.ctypes.data, ctypes.POINTER(DMonitoringModelResult)).contents
  msg = messaging.new_message('driverStateV2', valid=True)
  ds = msg.driverStateV2
  ds.frameId = frame_id
  ds.modelExecutionTime = execution_time
  ds.gpuExecutionTime = gpu_execution_time
  ds.wheelOnRightProb = float(sigmoid(model_result.wheel_on_right_prob))
  ds.rawPredictions = model_output.tobytes() if SEND_RAW_PRED else b''
  fill_driver_state(ds.leftDriverData, model_result.driver_state_lhd)
  fill_driver_state(ds.rightDriverData, model_result.driver_state_rhd)
  return msg


def main():
  config_realtime_process(7, 5)

  cloudlog.warning("connecting to driver stream")
  vipc_client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_DRIVER, True)
  while not vipc_client.connect(False):
    time.sleep(0.1)
  assert vipc_client.is_connected()
  cam_w, cam_h = vipc_client.width, vipc_client.height
  assert cam_w is not None and cam_h is not None
  cloudlog.warning(f"connected with buffer size: {vipc_client.buffer_len}")

  model = ModelState(cam_w, cam_h)
  cloudlog.warning("models loaded, dmonitoringmodeld starting")

  sm = SubMaster(["liveCalibration"])
  pm = PubMaster(["driverStateV2"])

  calib = np.zeros(CALIB_LEN, dtype=np.float32)
  model_transform = None

  while True:
    buf = vipc_client.recv()
    if buf is None:
      continue

    if model_transform is None:
      cam = _os_fisheye if buf.width == _os_fisheye.width else _ar_ox_fisheye
      model_transform = np.linalg.inv(np.dot(dmonitoringmodel_intrinsics, np.linalg.inv(cam.intrinsics))).astype(np.float32)

    sm.update(0)
    if sm.updated["liveCalibration"]:
      calib[:] = np.array(sm["liveCalibration"].rpyCalib)

    t1 = time.perf_counter()
    model_output, gpu_execution_time = model.run(buf, calib, model_transform)
    t2 = time.perf_counter()

    pm.send("driverStateV2", get_driverstate_packet(model_output, vipc_client.frame_id, vipc_client.timestamp_sof, t2 - t1, gpu_execution_time))


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    cloudlog.warning("got SIGINT")
