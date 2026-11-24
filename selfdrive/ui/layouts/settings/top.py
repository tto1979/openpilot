"""
T.O.P settings panel (raylib port of TOP's Qt TimpilotPanel).
"""
import os
import subprocess
import threading

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.confirm_dialog import alert_dialog
from openpilot.system.ui.widgets.list_view import button_item, toggle_item
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
from openpilot.system.ui.widgets.scroller_tici import Scroller

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


def _load_car_list() -> list[str]:
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


class TopLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._car_dialog: MultiOptionDialog | None = None
    self._car_list: list[str] = []
    self._car_list_loading = False
    self._pending_open = False

    self._car_btn = button_item(lambda: tr("Car Model"), self._car_button_text,
                                lambda: tr("Force a car model instead of fingerprinting. Select [-Not selected-] to go back to automatic detection."),
                                callback=self._open_car_dialog, enabled=lambda: not self._car_list_loading)

    self._toggles = {}
    for param, title, desc, icon in TOP_TOGGLES:
      enabled = (lambda: not ui_state.engaged) if param in DISENGAGED_ONLY else True
      # descriptions are rendered as HTML: newlines need <br>
      self._toggles[param] = toggle_item(lambda t=title: tr(t), lambda d=desc: tr(d).replace("\n", "<br>"), self._params.get_bool(param),
                                         callback=lambda state, p=param: self._toggle_callback(p, state), icon=icon, enabled=enabled)

    self._scroller = Scroller([self._car_btn, *self._toggles.values()], line_separator=True, spacing=0)

  def show_event(self):
    self._scroller.show_event()
    self._refresh_toggles()

  def _render(self, rect):
    self._scroller.render(rect)

  # *** toggles ***

  def _refresh_toggles(self):
    for param, item in self._toggles.items():
      item.action_item.set_state(self._params.get_bool(param))
    self._lock_lag_toggle(self._params.get_bool("NNFF"))

  def _lock_lag_toggle(self, nnff_on: bool):
    # NNFF always runs on a fixed delay, so with NNFF on, delay learning is switched off and locked
    lag = self._toggles["DisableLateralLagLearning"]
    if nnff_on:
      self._params.put_bool("DisableLateralLagLearning", True)
      lag.action_item.set_state(True)
    lag.action_item.set_enabled(not nnff_on)

  def _toggle_callback(self, param: str, state: bool):
    self._params.put_bool(param, state)
    if param == "NNFF":
      self._params.remove("LiveTorqueParameters")
      self._params.remove("LiveDelay")
      self._params.put_bool("OnroadCycleRequested", True)
      self._lock_lag_toggle(state)

  # *** car selection ***

  def _car_button_text(self) -> str:
    car = self._params.get("CarModel") or NOT_SELECTED
    if self._car_list_loading:
      return tr("LOADING")
    return tr("SELECT") if car == NOT_SELECTED else car

  def _open_car_dialog(self):
    if self._car_list:
      self._show_car_dialog()
      return

    def load():
      self._car_list = _load_car_list()
      self._car_list_loading = False

    self._car_list_loading = True
    self._pending_open = True
    threading.Thread(target=load, daemon=True).start()

  def _show_car_dialog(self):
    current = self._params.get("CarModel") or NOT_SELECTED
    options = [NOT_SELECTED, *self._car_list]

    def on_close(result: int):
      if result == DialogResult.CONFIRM and self._car_dialog is not None:
        selection = self._car_dialog.selection
        if selection == NOT_SELECTED:
          self._params.remove("CarModel")
        else:
          self._params.put("CarModel", selection)
      self._car_dialog = None

    self._car_dialog = MultiOptionDialog(tr("Select your car"), options, current if current in options else NOT_SELECTED)
    gui_app.set_modal_overlay(self._car_dialog, callback=on_close)

  def _update_state(self):
    # open the dialog once the background car list load finishes
    if self._pending_open and not self._car_list_loading:
      self._pending_open = False
      if self._car_list:
        self._show_car_dialog()
      else:
        gui_app.set_modal_overlay(alert_dialog(tr("Car list is not available.")))
