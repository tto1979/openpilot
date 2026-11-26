"""
Imported automatically by Python at startup (site.py), because the openpilot root is
on PYTHONPATH (/data/pythonpath in the AGNOS /etc/profile, $PWD in launch_chffrplus.sh).

comma three with the internal "dos" (STM32F4) panda: upstream panda dropped F4, so
`import panda` / `from panda... import` resolve to panda_tici instead. Unlike
dragonpilot's sys.modules["panda"] alias in manager.py, this also covers scripts run
by hand over SSH (selfdrive/debug, SecOC key tools, ...), not only manager processes.

No-op everywhere else: PC, CI, comma 3X and tici with an H7 "tres" panda.
"""
import importlib
import importlib.abc
import importlib.util
import os
import sys
from collections.abc import Sequence
from importlib.machinery import ModuleSpec
from types import ModuleType


def _is_tici_dos() -> bool:
  if "TICI_DOS" in os.environ:
    return True
  if "TICI_TRES" in os.environ:
    return False

  try:
    with open("/sys/firmware/devicetree/base/model") as f:
      if f.read().strip("\x00").split("comma ")[-1] != "tici":
        return False
  except OSError:
    return False

  # MCU type cached by launch_chffrplus.sh; unknown means dos, same fallback as the launch script
  try:
    with open("/data/tici_panda_mcu") as f:
      return f.read().strip() != "H7"
  except OSError:
    return True


class _PandaTiciAlias(importlib.abc.MetaPathFinder, importlib.abc.Loader):
  """Serve panda and panda.* from the already imported panda_tici modules."""

  def find_spec(self, fullname: str, path: Sequence[str] | None, target: ModuleType | None = None) -> ModuleSpec | None:
    if fullname == "panda" or fullname.startswith("panda."):
      return importlib.util.spec_from_loader(fullname, self)
    return None

  def create_module(self, spec: ModuleSpec) -> ModuleType:
    return importlib.import_module("panda_tici" + spec.name[len("panda"):])

  def exec_module(self, module: ModuleType) -> None:
    pass


if _is_tici_dos():
  sys.meta_path.insert(0, _PandaTiciAlias())
