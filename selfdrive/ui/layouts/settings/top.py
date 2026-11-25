"""
T.O.P settings panel (raylib port of TOP's Qt TimpilotPanel).
"""
import threading

from openpilot.common.params import Params
from openpilot.selfdrive.ui.layouts.settings.top_common import TOP_TOGGLES, DISENGAGED_ONLY, NOT_SELECTED, load_car_list, \
  apply_toggle, lag_learning_locked
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.confirm_dialog import alert_dialog
from openpilot.system.ui.widgets.list_view import button_item, toggle_item
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
from openpilot.system.ui.widgets.scroller_tici import Scroller


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
    self._lock_lag_toggle(lag_learning_locked(self._params))

  def _lock_lag_toggle(self, nnff_on: bool):
    # NNFF always runs on a fixed delay, so with NNFF on, delay learning is switched off and locked
    lag = self._toggles["DisableLateralLagLearning"]
    if nnff_on:
      self._params.put_bool("DisableLateralLagLearning", True)
      lag.action_item.set_state(True)
    lag.action_item.set_enabled(not nnff_on)

  def _toggle_callback(self, param: str, state: bool):
    apply_toggle(self._params, param, state)
    if param == "NNFF":
      self._lock_lag_toggle(lag_learning_locked(self._params))

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
      self._car_list = load_car_list()
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
