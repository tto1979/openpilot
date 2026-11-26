#!/usr/bin/env python3
import os
import glob
import onnx

BASEDIR = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../"))
MASTER_PATH = os.getenv("MASTER_PATH", BASEDIR)
MODEL_PATH = "/selfdrive/modeld/models/"

def get_checkpoint(f):
  model = onnx.load(f)
  metadata = {prop.key: prop.value for prop in model.metadata_props}
  # "<uuid>" or ".../<run_uuid>/<step>"; combined models list vision then policy
  parts = metadata['model_checkpoint'].split('/')
  return parts[-2] if len(parts) > 1 else parts[0]

if __name__ == "__main__":
  print("| | master | PR branch |")
  print("|-| -----  | --------- |")

  for f in glob.glob(BASEDIR + MODEL_PATH + "/*.onnx"):
    # TODO: add checkpoint to DM
    if "dmonitoring" in f:
      continue

    fn = os.path.basename(f)
    master_path = MASTER_PATH + MODEL_PATH + fn
    if os.path.exists(master_path):
      master = get_checkpoint(master_path)
      master_col = f"[{master}](https://reporter.comma.life/{master})"
    else:
      master_col = "N/A (new model)"
    pr = get_checkpoint(BASEDIR + MODEL_PATH + fn)
    print("|", fn, "|", master_col, "|", f"[{pr}](https://reporter.comma.life/{pr})", "|")
