"""
OSM offline maps panel (raylib port of TOP's Qt OsmPanel, originally from sunnypilot).
"""
import datetime
import json
import os
import shutil
import threading
import time

import pyray as rl
import requests

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.layouts.settings.speed_limit import SpeedLimitLayout
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.hardware import PC
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog, alert_dialog
from openpilot.system.ui.widgets.keyboard import Keyboard
from openpilot.system.ui.widgets.list_view import button_item, text_item
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
from openpilot.system.ui.widgets.scroller_tici import Scroller

MAP_PATH = os.path.expanduser("~/.comma/media/0/osm/offline/") if PC else "/data/media/0/osm/offline/"
NATIONS_URL = "https://raw.githubusercontent.com/pfeiferj/openpilot-mapd/main/nation_bounding_boxes.json"
US_STATES_URL = "https://raw.githubusercontent.com/pfeiferj/openpilot-mapd/main/us_states_bounding_boxes.json"
NONE_LOCATION = ("== None ==", "")
ALL_STATES = ("All States (~4.8 GB)", "All")
REFRESH_FAST = 1.0  # s, while downloading
REFRESH_SLOW = 5.0  # s


def _fetch_locations(url: str, first: tuple[str, str]) -> list[tuple[str, str]]:
  """[(full name, code)], sorted by name, with `first` on top."""
  resp = requests.get(url, timeout=10)
  resp.raise_for_status()
  locations = sorted(((v.get("full_name", code), code) for code, v in resp.json().items()), key=lambda x: x[0])
  return [first, *locations]


def _dir_size(path: str) -> int:
  total = 0
  for root, _, files in os.walk(path):
    for f in files:
      try:
        total += os.path.getsize(os.path.join(root, f))
      except OSError:
        pass
  return total


def _format_size(size: int | None) -> str:
  if size is None:
    return tr("Calculating...")
  gb = 1024 ** 3
  return f"{size / gb:.2f} GB" if size >= gb else f"{size / 1024 ** 2:.2f} MB"


def _format_time(seconds: float) -> str:
  minutes, secs = divmod(int(seconds), 60)
  return (f"{minutes}" + tr("m ") if minutes > 0 else "") + f"{secs}" + tr("s")


class OsmLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._mem_params = self._params if PC else Params("/dev/shm/params")
    self._speed_limit_page = SpeedLimitLayout(lambda: self._show_speed_limit(False))
    self._showing_speed_limit = False

    self._maps_size: int | None = None
    self._size_thread: threading.Thread | None = None
    self._busy = ""            # text shown while fetching a list or deleting maps
    self._last_refresh = 0.0
    self._download_start: float | None = None
    self._rate_history: list[float] = []
    self._eta_text = ""

    self._dialog: MultiOptionDialog | None = None
    self._keyboard = Keyboard(min_text_size=0)
    self._pending: tuple[str, list[tuple[str, str]]] | None = None  # (kind, locations) after a background fetch

    self._version = text_item(lambda: tr("Mapd Version"), lambda: self._params.get("MapdVersion") or tr("Loading..."))
    self._delete_btn = button_item(lambda: tr("Downloaded Maps"), lambda: tr("DELETE"), callback=self._delete_maps_prompt,
                                   enabled=lambda: not self._busy and not self._downloading())
    self._delete_btn.action_item.set_value(lambda: self._busy if self._busy == tr("Deleting...") else _format_size(self._maps_size))
    self._eta = text_item(lambda: tr("Offline Maps ETA"), lambda: self._eta_text)
    self._eta.set_visible(lambda: self._downloading() and self._download_start is not None)
    self._elapsed = text_item(lambda: tr("Time Elapsed"), self._elapsed_text)
    self._elapsed.set_visible(lambda: self._downloading() and self._download_start is not None)
    self._update_btn = button_item(lambda: tr("Database Update"), self._update_button_text, callback=self._update_prompt)
    self._update_btn.action_item.set_value(self._update_status)
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
    return bool(self._mem_params.get("OSMDownloadLocations"))

  def _progress(self) -> tuple[int, int]:
    progress = self._params.get("OSMDownloadProgress") or {}
    if isinstance(progress, (str, bytes)):
      try:
        progress = json.loads(progress)
      except ValueError:
        progress = {}
    return int(progress.get("total_files", 0) or 0), int(progress.get("downloaded_files", 0) or 0)

  def _last_downloaded(self) -> float | None:
    try:
      ts = float(self._params.get("OsmDownloadedDate") or 0)
    except (TypeError, ValueError):
      return None
    return ts if ts > 0 else None

  def _elapsed_text(self) -> str:
    total, done = self._progress()
    if total <= 0 or done >= total:
      return tr("Downloaded")
    if self._download_start is None or done == 0:
      return tr("Calculating...")
    return _format_time(time.monotonic() - self._download_start)

  def _update_eta(self):
    total, done = self._progress()
    if total <= 0 or done >= total:
      self._eta_text = tr("Ready") if total <= 0 else tr("Downloaded")
      return
    if self._download_start is None or done == 0:
      self._eta_text = tr("Calculating ETA...")
      return
    elapsed = time.monotonic() - self._download_start
    if elapsed <= 0:
      return
    self._rate_history = (self._rate_history + [done / elapsed])[-10:]
    if len(self._rate_history) < 3:
      self._eta_text = tr("Calculating ETA...")
      return
    weights = range(1, len(self._rate_history) + 1)
    rate = sum(r * w for r, w in zip(self._rate_history, weights, strict=True)) / sum(weights)
    if rate > 0:
      self._eta_text = tr("Time remaining: ") + _format_time((total - done) / rate)

  def _update_status(self) -> str:
    total, done = self._progress()
    downloading = self._downloading()
    if self._params.get_bool("OsmDbUpdatesCheck") and not downloading and not total:
      return tr("Download starting...")
    if downloading and total > done:
      return f"{done}/{total} ({int(100 * done / total)}%)"
    if downloading and total and done >= total:
      return tr("Download complete!")
    last = self._last_downloaded()
    if last is not None:
      return datetime.datetime.fromtimestamp(last).strftime("%Y-%m-%d %H:%M:%S")
    return ""

  def _update_button_text(self) -> str:
    return tr("REFRESH") if self._downloading() else tr("UPDATE")

  def _update_state(self):
    now = time.monotonic()
    if now - self._last_refresh < (REFRESH_FAST if self._downloading() else REFRESH_SLOW):
      self._handle_pending()
      return
    self._last_refresh = now

    if self._downloading():
      if self._download_start is None:
        self._download_start = now
        self._rate_history = []
      self._update_eta()
    else:
      self._download_start = None

    if self._size_thread is None or not self._size_thread.is_alive():
      def measure():
        self._maps_size = _dir_size(MAP_PATH)
      self._size_thread = threading.Thread(target=measure, daemon=True)
      self._size_thread.start()

    # same clean up as the Qt panel
    location = self._params.get("OsmLocationName")
    if location:
      if location != "US":
        self._params.remove("OsmStateName")
        self._params.remove("OsmStateTitle")
    else:
      for key in ("OsmLocal", "OsmLocationName", "OsmLocationTitle", "OsmStateName", "OsmStateTitle"):
        self._params.remove(key)
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
    self._params.put_bool("OsmDbUpdatesCheck", True)
    self._last_refresh = 0.0

  def _update_prompt(self):
    if self._downloading():
      self._last_refresh = 0.0
      return
    self._confirm_download(on_confirm=self._start_download)

  def _delete_maps_prompt(self):
    def delete():
      self._busy = tr("Deleting...")
      shutil.rmtree(MAP_PATH, ignore_errors=True)
      self._maps_size = None
      self._busy = ""
      self._last_refresh = 0.0

    self._confirm_download(tr("This will delete ALL downloaded maps\n\nAre you sure you want to delete all the maps?"),
                           tr("Yes, delete all the maps."), lambda: threading.Thread(target=delete, daemon=True).start())

  def _select_location(self, kind: str):
    def fetch():
      try:
        if kind == "country":
          locations = _fetch_locations(NATIONS_URL, NONE_LOCATION)
        else:
          locations = _fetch_locations(US_STATES_URL, ALL_STATES)
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
        self._params.put_bool("OsmLocal", True)
        self._params.put("OsmLocationTitle", title)
        self._params.put("OsmLocationName", code)
        if code == "US":
          self._select_location("state")
          return
        if title == NONE_LOCATION[0]:
          return
      else:
        self._params.put("OsmStateTitle", title)
        self._params.put("OsmStateName", code)
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
