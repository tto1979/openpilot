"""
TOP settings shared by the comma 3/3X (selfdrive/ui/layouts) and comma four (selfdrive/ui/mici) UIs:
toggle list, forced car model list and the side effects of some toggles. No UI code here.
"""
import os
import subprocess

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.ui.lib.multilang import tr_noop

CARS_DIR = os.path.join(BASEDIR, "selfdrive", "car", "top_tmp")
ALL_CARS_FILE = os.path.join(CARS_DIR, "AllCars")
CARS_FILE = os.path.join(CARS_DIR, "Cars")
NOT_SELECTED = "[-Not selected-]"

# param, title, description, icon
TOP_TOGGLES: list[tuple[str, str, str, str]] = [
  ("QuietDrive", tr_noop("Quiet Drive"),
   tr_noop("TOP will display alerts but only play the most important warning sounds. This feature can be toggled while the car is on."),
   "mute.png"),
  ("OnroadScreenOff", tr_noop("Driving Screen Off"),
   tr_noop("Turn off the device screen to protect the OLED panel after driving starts. " +
           "It automatically brightens or turns on when a touch or event occurs."),
   "metric.png"),
  ("dp_atl", tr_noop("Lateral Controls Always On"),
   tr_noop("Lateral control will always be on and will not be interrupted by braking."),
   "road.png"),
  ("NNFF", tr_noop("NNFF Torque Control"),
   tr_noop("Use Twilsonco's Neural Network Feedforward torque system for more precise lateral control.\n" +
           "Changing this setting will restart openpilot."),
   "road.png"),
  ("DisableLateralLagLearning", tr_noop("Disable Steering Delay Learning"),
   tr_noop("Keep the fixed default steering delay instead of learning the delay of the car while driving. " +
           "Turn on for cars whose steering wheel sticks: the learned delay comes out too long and the steering can wobble.\n" +
           "Always on with NNFF Torque Control, which uses the fixed delay. Takes effect on the next drive."),
   "road.png"),
  ("LaneCentering", tr_noop("Lane Centering Correction"),
   tr_noop("When the driving model keeps the car off the middle of a marked lane, cutting curves or hugging a line, " +
           "steer it back toward the lane centre. Only from 45 km/h (full from 60 km/h) in normal-width lanes: " +
           "off in alleys and slow streets, during lane changes, while you steer and where the lane lines are faint. " +
           "Takes effect on the next drive."),
   "road.png"),
  ("topsng", tr_noop("Stop And Go"),
   tr_noop("Enabled the Stop And Go feature and get auto hold."),
   "road.png"),
  ("sng_e2e", tr_noop("Enable Experiment Mode In SNG"),
   tr_noop("When vehicle is stopped, enable experiment mode to prevent radar misjudgment of other moving vehicles " +
           "that could cause continuous forward sliding."),
   "road.png"),
  ("ToyotaTune", tr_noop("TSS-P Longitudinal Tuning"),
   tr_noop("Using the longitudinal tuning can improve the smoothness on Toyota TSS-P models when equipped with SDSU for longitudinal control."),
   "road.png"),
  ("Dynamic_Follow", tr_noop("Dynamic Distance Adjustment"),
   tr_noop("The distance to the lead car will no longer be a fixed reaction time, " +
           "but will be dynamically adjusted based on the speed of the vehicle."),
   "road.png"),
  ("SmoothStops", tr_noop("Smooth Stops"),
   tr_noop("Eases off the brake in the final moments of a stop for a gentle landing. " +
           "Planned braking is never softened. Requires openpilot longitudinal control."),
   "road.png"),
  ("NudgelessLaneChange", tr_noop("Blinker Lane Change"),
   tr_noop("Change lanes without the need to nudge the steering wheel first.\n" +
           "Disabled: Need to nudge the steering wheel to change lanes.\nEnabled: Nudgeless.\n" +
           "Speed limit: Normal mode: above 20mph, Enabled Lateral Controls Always On: above 35mph."),
   "lane.png"),
  ("road_edge_detection", tr_noop("Edge Detection During Lane Changes"),
   tr_noop("When the system detects obstacles at the vehicle's edge, lane change assist functionality will be temporarily suspended.\n" +
           "NOTE: This will show 'Car Detected in Blindspot' warning."),
   "lane.png"),
  ("SmartCruiseControlVision", tr_noop("Vision Based Turn Control"),
   tr_noop("Use vision path predictions to estimate the appropriate speed to drive through turns ahead."),
   "road.png"),
  ("SmartCruiseControlMap", tr_noop("Map Based Turn Control"),
   tr_noop("Use map data to estimate the appropriate speed to drive through turns ahead."),
   "road.png"),
  ("fleetmanager", tr_noop("Enable Local File Server"),
   tr_noop("This will allow you to play or download openpilot driving record files through your browser.\n" +
           "Use web interface to control it: http://[device_ip]:8082\n" +
           "Internet access from mobile phone (tethering) is required."),
   "road.png"),
  ("toyota_stock_long", tr_noop("Use Toyota Stock Longitudinal Control"),
   tr_noop("Enable to use Toyota's stock longitudinal control."),
   "road.png"),
  ("ToyotaDriveMode", tr_noop("Enable Toyota Drive Mode Button"),
   tr_noop("TOP will link the Acceleration Personality to the car's physical drive mode selector.\nReboot Required."),
   "road.png"),
  ("toyotaautolock", tr_noop("Enable Door Auto Lock"),
   tr_noop("Enabled this to lock doors when drive above 25 km/h. Only work on some Toyota vehicles."),
   "road.png"),
  ("toyotaautounlock", tr_noop("Enable Door Auto Unlock"),
   tr_noop("Enabled this to unlock doors when shift to gear P. Only work on some Toyota vehicles."),
   "road.png"),
  ("AleSato_AutomaticBrakeHold", tr_noop("Automatic Brake Hold"),
   tr_noop("Activates the car's brakes after 1 seconds stopped. (Only support on Toyota TSS2 Hybrid vehicles)"),
   "road.png"),
  ("toyota_bsm", tr_noop("Fix Toyota BSM Signal"),
   tr_noop("Enhance BSM function for some Toyota vehicles that openpilot currently does not support."),
   "road.png"),
  ("ReverseAccChange", tr_noop("ACC +/-: Long Press Reverse"),
   tr_noop("Change the ACC +/- buttons behavior with cruise speed change in openpilot.\n" +
           "Disabled (Stock): Short = 1, Long = 5.\nEnabled: Short and Long = 5."),
   "acc_change.png"),
]

# can't be changed while engaged: flipping these restarts openpilot
DISENGAGED_ONLY = {"NNFF"}


def load_car_list() -> list[str]:
  """Same files the Qt panel used: launch_chffrplus.sh writes AllCars, force_car_recognition.py turns it into Cars."""
  if not os.path.isfile(CARS_FILE):
    try:
      os.makedirs(CARS_DIR, exist_ok=True)
      if not os.path.isfile(ALL_CARS_FILE):
        with open(ALL_CARS_FILE, "w") as f:
          subprocess.run(["python3", "opendbc_repo/opendbc/car/fingerprints.py"], cwd=BASEDIR, stdout=f, timeout=30, check=True)
      subprocess.run(["python3", "force_car_recognition.py"], cwd=BASEDIR, timeout=30, check=True)
    except Exception:
      cloudlog.exception("top: failed to generate the car list")

  try:
    with open(CARS_FILE) as f:
      return [line.strip() for line in f if line.strip()]
  except OSError:
    return []


def apply_toggle(params: Params, param: str, state: bool) -> None:
  """Write a TOP toggle and run its side effects."""
  params.put_bool(param, state)
  if param == "NNFF":
    # the learned torque / delay belong to the other controller, relearn and restart onroad
    params.remove("LiveTorqueParameters")
    params.remove("LiveDelay")
    params.put_bool("OnroadCycleRequested", True)
    if state:
      params.put_bool("DisableLateralLagLearning", True)


def lag_learning_locked(params: Params) -> bool:
  """NNFF always runs on a fixed steering delay, so delay learning is forced off while NNFF is on."""
  return bool(params.get_bool("NNFF"))


# *** device / software actions ***

ERROR_LOG_PATH = "/data/community/crashes/error.txt"  # written by system/sentry.py save_exception()
FLASH_PANDA_LOG = "/data/flash_panda.log"


def read_error_log() -> str:
  """Last crash traceback, empty if there is none."""
  try:
    with open(ERROR_LOG_PATH, encoding="utf-8", errors="replace") as f:
      return f.read().strip()
  except OSError:
    return ""


def error_log_html(text: str) -> str:
  # the HTML renderer keeps unknown tags like <module> as text, only line breaks need converting
  return "<p>" + "<br>".join(text.splitlines()) + "</p>"


def run_flash_panda() -> bool:
  """Reflash the panda through pandad (top/system/flash_panda.py). Blocking, run it in a thread."""
  try:
    with open(FLASH_PANDA_LOG, "w") as log_file:
      ret = subprocess.call(["python3", "top/system/flash_panda.py"], cwd=BASEDIR, stdout=log_file, stderr=subprocess.STDOUT)
  except Exception:
    cloudlog.exception("flash panda failed")
    ret = -1
  return ret == 0
