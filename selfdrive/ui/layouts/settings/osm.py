"""
OSM offline maps panel (raylib port of TOP's Qt OsmPanel, originally from sunnypilot).
"""
import threading
import time

import pyray as rl

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.layouts.settings.osm_common import NATIONS_URL, US_STATES_URL, NONE_LOCATION, ALL_STATES, OsmState, \
  fetch_locations, format_size
from openpilot.selfdrive.ui.layouts.settings.speed_limit import SpeedLimitLayout
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog, alert_dialog
from openpilot.system.ui.widgets.keyboard import Keyboard
from openpilot.system.ui.widgets.list_view import button_item, text_item
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
from openpilot.system.ui.widgets.scroller_tici import Scroller

REFRESH_FAST = 1.0  # s, while downloading
REFRESH_SLOW = 5.0  # s


class OsmLayout(Widget):
  def __init__(self):
    super().__init__()
    self._osm = OsmState()
    self._params = self._osm.params
    self._speed_limit_page = SpeedLimitLayout(lambda: self._show_speed_limit(False))
    self._showing_speed_limit = False

    self._busy = ""            # text shown while fetching a list or deleting maps
    self._last_refresh = 0.0

    self._dialog: MultiOptionDialog | None = None
    self._keyboard = Keyboard(min_text_size=0)
    self._pending: tuple[str, list[tuple[str, str]]] | None = None  # (kind, locations) after a background fetch

    self._version = text_item(lambda: tr("Mapd Version"), lambda: self._params.get("MapdVersion") or tr("Loading..."))
    self._delete_btn = button_item(lambda: tr("Downloaded Maps"), lambda: tr("DELETE"), callback=self._delete_maps_prompt,
                                   enabled=lambda: not self._busy and not self._downloading())
    self._delete_btn.action_item.set_value(lambda: self._busy if self._busy == tr("Deleting...") else format_size(self._osm.maps_size))
    self._eta = text_item(lambda: tr("Offline Maps ETA"), lambda: self._osm.eta_text)
    self._eta.set_visible(lambda: self._downloading() and self._osm.download_start is not None)
    self._elapsed = text_item(lambda: tr("Time Elapsed"), self._osm.elapsed_text)
    self._elapsed.set_visible(lambda: self._downloading() and self._osm.download_start is not None)
    self._update_btn = button_item(lambda: tr("Database Update"), self._update_button_text, callback=self._update_prompt)
    self._update_btn.action_item.set_value(self._osm.status_text)
    self._update_btn.set_visible(lambda: bool(self._params.get("OsmLocationName")))
    self._country_btn = button_item(lambda: tr("Country"), lambda: tr("SELECT"), callback=lambda: self._select_location("country"),
                                    enabled=lambda: not self._busy and not self._downloading())
    self._country_btn.action_item.set_value(lambda: self._busy if self._busy == tr("Fetching Country list...")
                                            else (self._params.get("OsmLocationTitle") or ""))
    self._state_btn = button_item(lambda: tr("State"), lambda: tr("SELECT"), callback=lambda: self._select_location("state"),
                                  enabled=lambda: not self._busy and not self._downloading())
    self._state_btn.action_item.set_value(lambda: self._busy if self._busy == tr("Fetching State list...")
                                          else (self._params.get("OsmStateTitle") or ""))
    self._state_btn.set_visible(lambda: self._params.get("OsmLocationName") == "US")
    self._speed_limit_btn = button_item(lambda: tr("Speed Limit"), lambda: tr("CONFIGURE"),
                                        lambda: tr("Configure speed limit settings including source policy, mode, and offset options."),
                                        callback=lambda: self._show_speed_limit(True))

    self._scroller = Scroller([self._version, self._delete_btn, self._eta, self._elapsed, self._update_btn,
                               self._country_btn, self._state_btn, self._speed_limit_btn], line_separator=True, spacing=0)

  # *** state ***

  def _downloading(self) -> bool:
    return bool(self._osm.downloading())

  def _update_button_text(self) -> str:
    return tr("REFRESH") if self._downloading() else tr("UPDATE")

  def _update_state(self):
    now = time.monotonic()
    if now - self._last_refresh >= (REFRESH_FAST if self._downloading() else REFRESH_SLOW):
      self._last_refresh = now
      self._osm.refresh()
    self._handle_pending()

  # *** actions ***

  def _confirm_download(self, message: str | None = None, confirm_text: str | None = None, on_confirm=None):
    metered = ui_state.sm["deviceState"].networkMetered
    text = message or tr("This will start the download process and it might take a while to complete.")
    if metered:
      text += "\n\n" + tr("Warning: You are on a metered connection!")
    button = confirm_text or (tr("Continue on Metered") if metered else tr("Start Download"))

    def callback(result: int):
      if result == DialogResult.CONFIRM and on_confirm is not None:
        on_confirm()

    gui_app.set_modal_overlay(ConfirmDialog(text, button), callback=callback)

  def _start_download(self):
    self._osm.start_download()
    self._last_refresh = 0.0

  def _update_prompt(self):
    if self._downloading():
      self._last_refresh = 0.0
      return
    self._confirm_download(on_confirm=self._start_download)

  def _delete_maps_prompt(self):
    def delete():
      self._busy = tr("Deleting...")
      self._osm.delete_maps()
      self._busy = ""
      self._last_refresh = 0.0

    self._confirm_download(tr("This will delete ALL downloaded maps\n\nAre you sure you want to delete all the maps?"),
                           tr("Yes, delete all the maps."), lambda: threading.Thread(target=delete, daemon=True).start())

  def _select_location(self, kind: str):
    def fetch():
      try:
        if kind == "country":
          locations = fetch_locations(NATIONS_URL, NONE_LOCATION)
        else:
          locations = fetch_locations(US_STATES_URL, ALL_STATES)
      except Exception:
        cloudlog.exception("osm: failed to fetch locations")
        locations = []
      self._pending = (kind, locations)
      self._busy = ""

    self._busy = tr("Fetching Country list...") if kind == "country" else tr("Fetching State list...")
    threading.Thread(target=fetch, daemon=True).start()

  def _handle_pending(self):
    if self._pending is None or gui_app._modal_overlay.overlay is not None:
      return
    kind, locations = self._pending
    self._pending = None
    if not locations:
      gui_app.set_modal_overlay(alert_dialog(tr("Unable to fetch the list. Check the internet connection.")))
      return

    title = tr("Search Country") if kind == "country" else tr("Search State")
    hint = tr("Enter search keywords, or leave blank to list all countries.") if kind == "country" else \
           tr("Enter search keywords, or leave blank to list all states.")
    self._keyboard.reset(min_text_size=0)
    self._keyboard.set_title(title, hint)

    def on_search(result: int):
      if result != 1:
        return
      query = self._keyboard.text.strip().lower()
      matches = [loc for loc in locations if query in loc[0].lower() or query == loc[1].lower()] if query else locations
      if not matches:
        gui_app.set_modal_overlay(alert_dialog(tr("No results found for keywords: {}").format(query)))
        return
      self._show_location_dialog(kind, matches)

    gui_app.set_modal_overlay(self._keyboard, callback=on_search)

  def _show_location_dialog(self, kind: str, locations: list[tuple[str, str]]):
    titles = [loc[0] for loc in locations]
    current = self._params.get("OsmLocationTitle" if kind == "country" else "OsmStateTitle") or ""

    def on_select(result: int):
      dialog, self._dialog = self._dialog, None
      if result != DialogResult.CONFIRM or dialog is None:
        return
      title = dialog.selection
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
      self._confirm_download(on_confirm=self._start_download)

    title = tr("Select Country") if kind == "country" else tr("Select State")
    self._dialog = MultiOptionDialog(title, titles, current if current in titles else "")
    gui_app.set_modal_overlay(self._dialog, callback=on_select)

  # *** layout ***

  def _show_speed_limit(self, show: bool):
    self._showing_speed_limit = show
    if show:
      self._speed_limit_page.show_event()
    else:
      self._scroller.show_event()

  def show_event(self):
    self._showing_speed_limit = False
    self._last_refresh = 0.0
    self._scroller.show_event()

  def _render(self, rect: rl.Rectangle):
    if self._showing_speed_limit:
      self._speed_limit_page.render(rect)
    else:
      self._scroller.render(rect)
