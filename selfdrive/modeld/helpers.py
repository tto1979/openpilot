import sys
from pathlib import Path

from openpilot.system.hardware import TICI

MODELS_DIR = Path(__file__).resolve().parent / 'models'


def load_oob(path):
  # pkls built by tinygrad's examples/openpilot/compile_onnx.py --out-of-band
  from tinygrad import Context
  device = 'QCOM' if TICI else 'METAL' if sys.platform == 'darwin' else 'CPU:LLVM'
  with Context(DEV=device):
    from tinygrad_repo.examples.openpilot.helpers import load_pickle
    return load_pickle(path, out_of_band=True)
