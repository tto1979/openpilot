"""
TOP settings for the comma four (mici) UI. Same params and side effects as the comma 3/3X TOP panel
(selfdrive/ui/layouts/settings/top.py), laid out as mici cards.
"""
import threading
from collections.abc import Callable

import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.ui.layouts.settings.top_common import TOP_TOGGLES, DISENGAGED_ONLY, load_car_list, apply_toggle, \
  lag_learning_locked
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigParamControl
from openpilot.selfdrive.ui.mici.widgets.dialog import BigDialog
from openpilot.selfdrive.ui.mici.widgets.top_dialogs import CompactOptionDialog, elide_value
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import NavWidget
from openpilot.system.ui.widgets.scroller import Scroller

# short card titles in the mici style, same order as TOP_TOGGLES
MICI_TITLES = {
  "QuietDrive": "quiet drive",
  "OnroadScreenOff": "driving screen off",
  "dp_atl": "lateral always on",
  "NNFF": "NNFF torque control",
  "DisableLateralLagLearning": "disable steering delay learning",
  "LaneCentering": "lane centering correction",
  "topsng": "stop and go",
  "sng_e2e": "experimental mode in stop and go",
  "ToyotaTune": "TSS-P longitudinal tuning",
  "Dynamic_Follow": "dynamic follow distance",
  "SmoothStops": "smooth stops",
  "NudgelessLaneChange": "blinker lane change",
  "road_edge_detection": "road edge detection",
  "SmartCruiseControlVision": "vision turn control",
  "SmartCruiseControlMap": "map turn control",
  "fleetmanager": "local file server",
  "dp_dev_opview": "opview phone display",
  "toyota_stock_long": "toyota stock longitudinal",
  "ToyotaDriveMode": "toyota drive mode button",
  "toyotaautolock": "door auto lock",
  "toyotaautounlock": "door auto unlock",
  "AleSato_AutomaticBrakeHold": "automatic brake hold",
  "toyota_bsm": "fix toyota BSM signal",
  "ReverseAccChange": "ACC +/- long press reverse",
}
AUTO_DETECT = "auto detect"
HIDDEN_BRANDS = {"MOCK"}


def car_brand(car: str) -> str:
  return car.split("_", 1)[0]


def car_model_name(car: str) -> str:
  """TOYOTA_COROLLA_TSS2 -> COROLLA TSS2"""
  parts = car.split("_", 1)
  return parts[1].replace("_", " ") if len(parts) > 1 else car


class TopLayoutMici(NavWidget):
  def __init__(self, back_callback: Callable):
    super().__init__()
    self.set_back_callback(back_callback)
    self._params = Params()

    self._car_list: list[str] = []
    self._car_list_loading = False
    self._pending_open = False
    # brand picked, model picker opens on the next frame: a modal opened in the frame another one
    # closes doesn't get its show_event
    self._pending_brand: str | None = None
    self._car_btn = BigButton("car model", self._car_value(), "icons/road.png")
    self._car_btn.set_click_callback(self._open_car_dialog)
    self._car_btn.set_enabled(lambda: not self._car_list_loading)

    self._toggles: dict[str, BigParamControl] = {}
    for param, title, _desc, _icon in TOP_TOGGLES:
      toggle = BigParamControl(MICI_TITLES.get(param, title.lower()), param,
                               toggle_callback=lambda checked, p=param: self._toggle_callback(p, checked))
      if param in DISENGAGED_ONLY:
        toggle.set_enabled(lambda: not ui_state.engaged)
      self._toggles[param] = toggle

    self._scroller = Scroller([self._car_btn, *self._toggles.values()], snap_items=False)

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._refresh()

  def _refresh(self):
    for toggle in self._toggles.values():
      toggle.refresh()
    self._car_btn.set_value(self._car_value())
    self._lock_lag_toggle()

  def _lock_lag_toggle(self):
    lag = self._toggles["DisableLateralLagLearning"]
    locked = lag_learning_locked(self._params)
    if locked:
      lag.set_checked(True)
    lag.set_enabled(not locked)

  def _toggle_callback(self, param: str, checked: bool):
    # BigParamControl writes the param right after this, apply_toggle writes it too (with the side effects)
    apply_toggle(self._params, param, checked)
    if param == "NNFF":
      self._lock_lag_toggle()

  # *** car selection ***

  def _car_value(self) -> str:
    if self._car_list_loading:
      return "loading..."
    car = self._params.get("CarModel")
    return elide_value(car.replace("_", " ")) if car else AUTO_DETECT

  def _open_car_dialog(self):
    if self._car_list:
      self._show_car_dialog()
      return

    def load():
      self._car_list = load_car_list()
      self._car_list_loading = False

    self._car_list_loading = True
    self._pending_open = True
    self._car_btn.set_value(self._car_value())
    threading.Thread(target=load, daemon=True).start()

  def _show_car_dialog(self):
    """Two steps on the small screen: brand, then the brand's models (names without the brand)."""
    current = self._params.get("CarModel") or ""
    brands = sorted({car_brand(c) for c in self._car_list} - HIDDEN_BRANDS)
    options = [AUTO_DETECT, *brands]
    dlg: CompactOptionDialog

    def on_confirm():
      brand = dlg.get_selected_option()
      if brand == AUTO_DETECT:
        self._params.remove("CarModel")
        self._car_btn.set_value(self._car_value())
      else:
        self._pending_brand = brand

    current_brand = car_brand(current) if current else AUTO_DETECT
    dlg = CompactOptionDialog(options, current_brand if current_brand in options else AUTO_DETECT, right_btn_callback=on_confirm)
    gui_app.set_modal_overlay(dlg)

  def _show_model_dialog(self, brand: str):
    cars = [c for c in self._car_list if car_brand(c) == brand]
    current = self._params.get("CarModel") or ""
    dlg: CompactOptionDialog

    def on_confirm():
      self._params.put("CarModel", dlg.get_selected_option())
      self._car_btn.set_value(self._car_value())

    dlg = CompactOptionDialog(cars, current if current in cars else cars[0], right_btn_callback=on_confirm,
                              display={c: car_model_name(c) for c in cars})
    gui_app.set_modal_overlay(dlg)

  def _update_state(self):
    super()._update_state()
    # open the picker once the background car list load finishes
    if self._pending_open and not self._car_list_loading:
      self._pending_open = False
      self._car_btn.set_value(self._car_value())
      if self._car_list:
        self._show_car_dialog()
      else:
        gui_app.set_modal_overlay(BigDialog("car list not available", ""))

  def _render(self, rect: rl.Rectangle):
    # only rendered while no modal is shown
    if self._pending_brand is not None:
      brand, self._pending_brand = self._pending_brand, None
      self._show_model_dialog(brand)
    self._scroller.render(rect)
