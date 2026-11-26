"""
OSM offline maps and speed limit settings for the comma four (mici) UI. Same params and mapd handling as the
comma 3/3X OSM panel (selfdrive/ui/layouts/settings/osm.py), shared through osm_common.OsmState.
"""
import threading
import time
from collections.abc import Callable

import pyray as rl

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.layouts.settings.osm_common import NATIONS_URL, US_STATES_URL, NONE_LOCATION, ALL_STATES, OsmState, \
  fetch_locations, format_size
from openpilot.selfdrive.ui.layouts.settings.speed_limit import SpeedLimitOffsetType
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigMultiParamToggle
from openpilot.selfdrive.ui.mici.widgets.dialog import BigDialog, BigMultiOptionDialog, BigConfirmationDialogV2
from openpilot.selfdrive.ui.mici.widgets.top_dialogs import CompactOptionDialog, elide_value
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import NavWidget
from openpilot.system.ui.widgets.scroller import Scroller

REFRESH_FAST = 1.0  # s, while downloading
REFRESH_SLOW = 5.0  # s
SPEED_LIMIT_SOURCES = ["car only", "map only", "car first", "map first", "combined"]  # SpeedLimitSourcePolicy order
OFFSET_RANGE = range(-30, 31)


class DefaultParamToggle(BigMultiParamToggle):
  """BigMultiParamToggle that falls back to the param's default from params_keys.h instead of the first option."""
  def _load_value(self):
    value = self._params.get(self._param, return_default=True)
    self.set_value(self._options[value if value in range(len(self._options)) else 0])


class OsmLayoutMici(NavWidget):
  def __init__(self, back_callback: Callable):
    super().__init__()
    self.set_back_callback(back_callback)
    self._osm = OsmState()
    self._params = self._osm.params
    self._last_refresh = 0.0
    self._busy = ""
    self._pending: tuple[str, list[tuple[str, str]]] | None = None  # (kind, locations) after a background fetch
    # open the download slider once the location picker is gone: a modal opened in the same frame
    # another one closes doesn't get its show_event
    self._pending_download = False

    # BigButton sizes the title once in __init__: give every card a value now so titles leave room for it
    self._update_btn = BigButton("update maps", "-", "icons_mici/settings/device/update.png")
    self._update_btn.set_click_callback(self._update_prompt)
    self._country_btn = BigButton("map country", "-")
    self._country_btn.set_click_callback(lambda: self._select_location("country"))
    self._country_btn.set_enabled(lambda: not self._busy and not self._osm.downloading())
    self._state_btn = BigButton("map state", "-")
    self._state_btn.set_click_callback(lambda: self._select_location("state"))
    self._state_btn.set_enabled(lambda: not self._busy and not self._osm.downloading())
    self._state_btn.set_visible(lambda: self._params.get("OsmLocationName") == "US")
    self._delete_btn = BigButton("downloaded maps", "-", "icons_mici/settings/device/uninstall.png")
    self._delete_btn.set_click_callback(self._delete_prompt)
    self._delete_btn.set_enabled(lambda: not self._busy and not self._osm.downloading())

    self._mode_toggle = DefaultParamToggle("speed limit mode", "SpeedLimitMode", ["off", "info", "warning", "assist"])
    self._source_btn = BigButton("speed limit source", "-")
    self._source_btn.set_click_callback(self._select_source)
    self._offset_type_toggle = DefaultParamToggle("speed limit offset type", "SpeedLimitOffsetType", ["none", "fixed", "percent"])
    self._offset_btn = BigButton("speed limit offset", "-")
    self._offset_btn.set_click_callback(self._select_offset)
    self._offset_btn.set_visible(lambda: self._offset_type() != SpeedLimitOffsetType.NONE)
    self._version_btn = BigButton("mapd version", "-")
    self._version_btn.set_enabled(False)

    self._scroller = Scroller([self._update_btn, self._country_btn, self._state_btn, self._delete_btn, self._mode_toggle,
                               self._source_btn, self._offset_type_toggle, self._offset_btn, self._version_btn], snap_items=False)

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._last_refresh = 0.0
    self._mode_toggle._load_value()
    self._offset_type_toggle._load_value()

  # *** values ***

  def _offset_type(self) -> int:
    return self._params.get("SpeedLimitOffsetType", return_default=True) or 0

  def _offset_text(self) -> str:
    value = self._params.get("SpeedLimitValueOffset", return_default=True) or 0
    unit = "%" if self._offset_type() == SpeedLimitOffsetType.PERCENT else (" km/h" if ui_state.is_metric else " mph")
    return f"{value:+d}{unit}"

  def _refresh_values(self):
    if self._busy:
      self._update_btn.set_value(self._busy)
    else:
      self._update_btn.set_value(self._osm.status_text() or ("not downloaded" if self._params.get("OsmLocationName") else "select a country"))
    self._country_btn.set_value(elide_value(self._params.get("OsmLocationTitle") or "none"))
    self._state_btn.set_value(elide_value(self._params.get("OsmStateTitle") or "all"))
    self._delete_btn.set_value(format_size(self._osm.maps_size))
    source = self._params.get("SpeedLimitPolicy", return_default=True) or 0
    self._source_btn.set_value(SPEED_LIMIT_SOURCES[source] if source in range(len(SPEED_LIMIT_SOURCES)) else "")
    self._offset_btn.set_value(self._offset_text())
    self._version_btn.set_value(self._params.get("MapdVersion") or "loading...")

  def _update_state(self):
    super()._update_state()
    now = time.monotonic()
    if now - self._last_refresh >= (REFRESH_FAST if self._osm.downloading() else REFRESH_SLOW):
      self._last_refresh = now
      self._osm.refresh()
    self._refresh_values()
    self._handle_pending()

  # *** maps ***

  def _confirm(self, title: str, icon: str, on_confirm: Callable, red: bool = False):
    gui_app.set_modal_overlay(BigConfirmationDialogV2(title, icon, red=red, confirm_callback=on_confirm))

  def _download_prompt(self):
    metered = ui_state.sm["deviceState"].networkMetered
    self._confirm("slide to download\non metered network" if metered else "slide to download",
                  "icons_mici/settings/device/update.png", self._start_download, red=metered)

  def _start_download(self):
    self._osm.start_download()
    self._last_refresh = 0.0

  def _update_prompt(self):
    if self._osm.downloading():
      self._last_refresh = 0.0
      return
    if not self._params.get("OsmLocationName"):
      gui_app.set_modal_overlay(BigDialog("select a map country first", ""))
      return
    self._download_prompt()

  def _delete_prompt(self):
    def delete():
      self._busy = "deleting..."
      self._osm.delete_maps()
      self._busy = ""
      self._last_refresh = 0.0

    self._confirm("slide to delete\nall maps", "icons_mici/settings/device/uninstall.png",
                  lambda: threading.Thread(target=delete, daemon=True).start(), red=True)

  def _select_location(self, kind: str):
    def fetch():
      try:
        locations = fetch_locations(NATIONS_URL, NONE_LOCATION) if kind == "country" else fetch_locations(US_STATES_URL, ALL_STATES)
      except Exception:
        cloudlog.exception("osm: failed to fetch locations")
        locations = []
      self._pending = (kind, locations)
      self._busy = ""

    self._busy = "loading list..."
    threading.Thread(target=fetch, daemon=True).start()

  def _handle_pending(self):
    if self._pending is None or gui_app._modal_overlay.overlay is not None:
      return
    kind, locations = self._pending
    self._pending = None
    if not locations:
      gui_app.set_modal_overlay(BigDialog("unable to fetch the list", "check the internet connection"))
      return

    titles = [loc[0] for loc in locations]
    current = self._params.get("OsmLocationTitle" if kind == "country" else "OsmStateTitle") or titles[0]
    dlg: CompactOptionDialog

    def on_confirm():
      title = dlg.get_selected_option()
      code = dict(locations).get(title, "")
      if kind == "country":
        self._osm.set_country(title, code)
        if code == "US":
          self._select_location("state")
          return
        if title == NONE_LOCATION[0]:
          return
      else:
        self._osm.set_state(title, code)
      self._pending_download = True

    dlg = CompactOptionDialog(titles, current if current in titles else titles[0], right_btn_callback=on_confirm)
    gui_app.set_modal_overlay(dlg)

  # *** speed limit ***

  def _select_source(self):
    current = self._params.get("SpeedLimitPolicy", return_default=True) or 0
    dlg: BigMultiOptionDialog

    def on_confirm():
      self._params.put("SpeedLimitPolicy", SPEED_LIMIT_SOURCES.index(dlg.get_selected_option()))

    dlg = BigMultiOptionDialog(SPEED_LIMIT_SOURCES, SPEED_LIMIT_SOURCES[current] if current in range(len(SPEED_LIMIT_SOURCES)) else None,
                               right_btn_callback=on_confirm)
    gui_app.set_modal_overlay(dlg)

  def _select_offset(self):
    unit = "%" if self._offset_type() == SpeedLimitOffsetType.PERCENT else ""
    options = [f"{v:+d}{unit}" for v in OFFSET_RANGE]
    current = self._params.get("SpeedLimitValueOffset", return_default=True) or 0
    dlg: BigMultiOptionDialog

    def on_confirm():
      self._params.put("SpeedLimitValueOffset", OFFSET_RANGE[options.index(dlg.get_selected_option())])

    dlg = BigMultiOptionDialog(options, f"{current:+d}{unit}" if current in OFFSET_RANGE else options[30], right_btn_callback=on_confirm)
    gui_app.set_modal_overlay(dlg)

  def _render(self, rect: rl.Rectangle):
    # only rendered while no modal is shown
    if self._pending_download:
      self._pending_download = False
      self._download_prompt()
    self._scroller.render(rect)
