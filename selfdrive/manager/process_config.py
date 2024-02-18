import os

from cereal import car
from openpilot.common.params import Params
from openpilot.system.hardware import PC, TICI, EON
from openpilot.selfdrive.manager.process import PythonProcess, NativeProcess, DaemonProcess

WEBCAM = os.getenv("USE_WEBCAM") is not None

dp_jetson = Params().get_bool("dp_jetson")
dp_mapd = Params().get_bool("dp_mapd")

def driverview(started: bool, params: Params, CP: car.CarParams) -> bool:
  return params.get_bool("IsDriverViewEnabled")  # type: ignore

def notcar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return CP.notCar  # type: ignore

def iscar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return not CP.notCar

def logging(started, params, CP: car.CarParams) -> bool:
  run = (not CP.notCar) or not params.get_bool("DisableLogging")
  return started and run

def ublox_available() -> bool:
  if EON:
    return True
  return os.path.exists('/dev/ttyHS0') and not os.path.exists('/persist/comma/use-quectel-gps')

def ublox(started, params, CP: car.CarParams) -> bool:
  use_ublox = ublox_available()
  if use_ublox != params.get_bool("UbloxAvailable"):
    params.put_bool("UbloxAvailable", use_ublox)
  return started and use_ublox

def qcomgps(started, params, CP: car.CarParams) -> bool:
  return started and not ublox_available()

procs = [
  NativeProcess("camerad", "selfdrive/camerad", ["./camerad"], callback=driverview),
  NativeProcess("clocksd", "system/clocksd", ["./clocksd"]),
  NativeProcess("logcatd", "system/logcatd", ["./logcatd"]),
  NativeProcess("proclogd", "system/proclogd", ["./proclogd"]),
  PythonProcess("logmessaged", "system.logmessaged", offroad=True),
  # PythonProcess("micd", "system.micd", callback=iscar),
  # PythonProcess("timezoned", "system.timezoned", enabled=not PC, offroad=True),

  DaemonProcess("manage_athenad", "selfdrive.athena.manage_athenad", "AthenadPid"),
  NativeProcess("dmonitoringmodeld", "selfdrive/hybrid_modeld", ["./dmonitoringmodeld"], enabled=(not PC or WEBCAM) and not dp_jetson, callback=driverview),
  # NativeProcess("encoderd", "system/loggerd", ["./encoderd"]),
  # NativeProcess("stream_encoderd", "system/loggerd", ["./encoderd", "--stream"], onroad=False, callback=notcar),
  NativeProcess("loggerd", "selfdrive/loggerd", ["./loggerd"], onroad=False, callback=logging),
  NativeProcess("modeld", "selfdrive/hybrid_modeld" if not Params().get_bool("dp_0813") else "selfdrive/legacy_modeld", ["./modeld"]),
  # NativeProcess("mapsd", "selfdrive/navd", ["./mapsd"]),
  # NativeProcess("navmodeld", "selfdrive/modeld", ["./navmodeld"]),
  NativeProcess("sensord", "system/sensord", ["./sensord"], enabled=not PC, offroad=EON),
  NativeProcess("ui", "selfdrive/ui", ["./ui"], offroad=True, watchdog_max_dt=(5 if not PC else None)),
  NativeProcess("soundd", "selfdrive/ui/soundd", ["./soundd"]),
  NativeProcess("locationd", "selfdrive/locationd", ["./locationd"]),
  NativeProcess("boardd", "selfdrive/boardd", ["./boardd"], enabled=False),
  PythonProcess("calibrationd", "selfdrive.locationd.calibrationd"),
  PythonProcess("torqued", "selfdrive.locationd.torqued"),
  PythonProcess("controlsd", "selfdrive.controls.controlsd"),
  PythonProcess("deleter", "selfdrive.loggerd.deleter", offroad=True),
  PythonProcess("dmonitoringd", "selfdrive.legacy_monitoring.dmonitoringd", enabled=(not PC or WEBCAM) and not dp_jetson, callback=driverview),
  # PythonProcess("laikad", "selfdrive.locationd.laikad"),
  # PythonProcess("rawgpsd", "system.sensord.rawgps.rawgpsd", enabled=TICI, onroad=False, callback=qcomgps),
  # PythonProcess("navd", "selfdrive.navd.navd"),
  PythonProcess("pandad", "selfdrive.boardd.pandad", offroad=True),
  PythonProcess("paramsd", "selfdrive.locationd.paramsd"),
  NativeProcess("ubloxd", "system/ubloxd", ["./ubloxd"], enabled=not PC, onroad=False, callback=ublox),
  # PythonProcess("pigeond", "system.sensord.pigeond", enabled=TICI, onroad=False, callback=ublox),
  PythonProcess("plannerd", "selfdrive.controls.plannerd"),
  PythonProcess("radard", "selfdrive.controls.radard"),
  PythonProcess("thermald", "selfdrive.thermald.thermald", offroad=True),
  PythonProcess("tombstoned", "selfdrive.tombstoned", enabled=not PC, offroad=True),
  PythonProcess("updated", "selfdrive.updated", enabled=not PC, onroad=False, offroad=True),
  PythonProcess("uploader", "selfdrive.loggerd.uploader", offroad=True),
  # PythonProcess("statsd", "selfdrive.statsd", offroad=True),

  # debug procs
  NativeProcess("bridge", "cereal/messaging", ["./bridge"], onroad=False, callback=notcar),
  # rick - webjoystick needs aiohttp, install additional modules manually: pip install aiohttp aiortc
  # PythonProcess("webjoystick", "tools.bodyteleop.web", onroad=False, callback=notcar),

  # EON only
  PythonProcess("rtshield", "selfdrive.rtshield", enabled=EON),
  PythonProcess("shutdownd", "system.hardware.eon.shutdownd", enabled=EON),
  PythonProcess("androidd", "system.hardware.eon.androidd", enabled=EON, offroad=True),

  # mapd
  PythonProcess("mapd", "selfdrive.mapd.mapd", enabled=dp_mapd),
]

managed_processes = {p.name: p for p in procs}
