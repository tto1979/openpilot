#!/usr/bin/env python3
"""Compile the stateful driving_supercombo.onnx into a TinyJit pkl for cd210-c3's modeld.

Since upstream #38916 the history queues (images, desire, features) live inside the ONNX:
  inputs:  new_img (2, 6, H, W) uint8 = current frame of [img, big_img], desire, traffic_convention, action_t,
           state_* history queues
  outputs: outputs, next_state_* (the updated queues)

cd210-c3 still warps with OpenCL (DrivingModelFrame), which hands over 12 channels = [frame t-4, frame t] per camera.
The JIT takes those buffers as img/big_img, builds new_img from the current frame, keeps the state queues on the
device (written back in place every run) and returns `outputs` as float32.
"""
import os
import sys
import pickle
import subprocess
import tempfile
import numpy as np
# same defaults as tinygrad's examples/openpilot/compile3.py
for k, v in {"FLOAT16": "1", "IMAGE": "2", "NOLOCALS": "1", "JIT_BATCH_SIZE": "0"}.items():
  os.environ.setdefault(k, v)

from tinygrad import Tensor, TinyJit, Device, GlobalCounters, dtypes
from tinygrad.dtype import _to_np_dtype
from tinygrad.frontend.onnx import OnnxRunner

VISION_INPUTS = ('img', 'big_img')


def host_dtype(dt):
  # float inputs and state buffers are float32 outside the model, same convention as compile3.py
  return dtypes.float32 if dt == dtypes.float16 else dt


def make_run(runner, specs, state_pairs, npy_names, frame_channels):
  def run(**kwargs):
    # current frame of each camera: channels [frame_channels:] of the [t-4, t] pair from DrivingModelFrame
    new_img = Tensor.cat(*[kwargs[name][:, frame_channels:] for name in VISION_INPUTS], dim=0).reshape(specs['new_img'][0])
    inputs = {'new_img': new_img,
              **{name: kwargs[name].to(Device.DEFAULT) for name in npy_names},
              **{name: kwargs[name] for name in state_pairs}}
    inputs = {name: value.cast(specs[name][1]) for name, value in inputs.items()}
    outputs = {name: value.contiguous() for name, value in runner(inputs).items()}
    Tensor.realize(*outputs.values())
    if state_pairs:
      Tensor.realize(*(kwargs[name].assign(outputs[next_name].cast(kwargs[name].dtype)) for name, next_name in state_pairs.items()))
    return outputs['outputs'].cast('float32')
  return run


def make_inputs(input_specs, npy_names, state_pairs, seed):
  rng = np.random.default_rng(seed)
  inputs = {}
  for name, (shape, dtype) in input_specs.items():
    if name in VISION_INPUTS:
      inputs[name] = Tensor(rng.integers(0, 256, shape, dtype=np.uint8), device=Device.DEFAULT).realize()
    elif name in npy_names:
      inputs[name] = Tensor(rng.standard_normal(shape).astype(dtype), device='NPY').realize()
    else:  # state queues start empty, like in modeld
      inputs[name] = Tensor(np.zeros(shape, dtype=dtype), device=Device.DEFAULT).realize()
  return inputs


def compile_model(onnx_path, output_path):
  runner = OnnxRunner(onnx_path)
  print("loaded model")
  specs = {name: (tuple(spec.shape), spec.dtype) for name, spec in runner.graph_inputs.items()}
  assert 'new_img' in specs and 'outputs' in runner.graph_outputs, \
    f"expected a stateful supercombo (new_img -> outputs), got inputs {list(specs)} outputs {list(runner.graph_outputs)}"
  state_pairs = {name: f'next_{name}' for name in specs if f'next_{name}' in runner.graph_outputs}
  npy_names = [name for name in specs if name != 'new_img' and name not in state_pairs]
  n_cams, frame_channels, h, w = specs['new_img'][0]
  assert n_cams == len(VISION_INPUTS), specs['new_img']
  print(f"{state_pairs=} {npy_names=}")

  input_specs = {name: ((1, 2 * frame_channels, h, w), np.dtype(np.uint8).str) for name in VISION_INPUTS}
  input_specs |= {name: (specs[name][0], np.dtype(_to_np_dtype(host_dtype(specs[name][1]))).str) for name in npy_names + list(state_pairs)}

  run_jit = TinyJit(make_run(runner, specs, state_pairs, npy_names, frame_channels), prune=True)
  for i in range(3):
    GlobalCounters.reset()
    print(f"run {i}")
    ret = run_jit(**make_inputs(input_specs, npy_names, state_pairs, 42)).numpy()
    if i == 1:
      test_val = np.copy(ret)
  print(f"captured {len(run_jit.captured.jit_cache)} kernels")
  np.testing.assert_equal(test_val, ret, "JIT run failed")
  # the history has to carry over between runs: once the queues fill up (frames t-4 and t both hold the input), the
  # output with the same inputs has to change
  inputs = make_inputs(input_specs, npy_names, state_pairs, 42)
  outs = [run_jit(**inputs).numpy().copy() for _ in range(6)]
  if state_pairs and np.array_equal(outs[0], outs[-1]):
    print("WARNING: output didn't change between runs with the same inputs, is the history state used?")
  print("jit run validated")

  artifact = {'run': run_jit, 'input_specs': input_specs, 'state_pairs': state_pairs, 'npy_inputs': npy_names,
              'vision_inputs': list(VISION_INPUTS)}
  with open(output_path, "wb") as f:
    pickle.dump(artifact, f)

  # check the pickled JIT in a fresh process, like modeld loads it (an in-process round trip can read back stale outputs)
  with tempfile.NamedTemporaryFile(suffix='.npy') as expected:
    np.save(expected.name, test_val)
    subprocess.check_call([sys.executable, __file__, '--verify', output_path, expected.name])
  print(f"mdl size is {os.path.getsize(onnx_path)/1e6:.2f}M")
  print(f"pkl size is {os.path.getsize(output_path)/1e6:.2f}M")
  print("**** compile done ****")


def verify(pkl_path, expected_path):
  with open(pkl_path, "rb") as f:
    art = pickle.load(f)
  out = art['run'](**make_inputs(art['input_specs'], art['npy_inputs'], art['state_pairs'], 42)).numpy()
  np.testing.assert_equal(out, np.load(expected_path), "pickled JIT run failed")
  print("pickled jit validated")


if __name__ == "__main__":
  if sys.argv[1] == '--verify':
    verify(sys.argv[2], sys.argv[3])
  else:
    compile_model(sys.argv[1], sys.argv[2])
