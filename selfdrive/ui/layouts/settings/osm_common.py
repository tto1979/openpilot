"""
OSM offline maps state shared by the comma 3/3X and comma four settings UIs (port of TOP's Qt OsmPanel,
originally from sunnypilot). mapd does the work, this only reads and writes its params. No UI code here.
"""
import datetime
import json
import os
import shutil
import threading
import time

import requests

from openpilot.common.params import Params
from openpilot.system.hardware import PC
from openpilot.system.ui.lib.multilang import tr

MAP_PATH = os.path.expanduser("~/.comma/media/0/osm/offline/") if PC else "/data/media/0/osm/offline/"
NATIONS_URL = "https://raw.githubusercontent.com/pfeiferj/openpilot-mapd/main/nation_bounding_boxes.json"
US_STATES_URL = "https://raw.githubusercontent.com/pfeiferj/openpilot-mapd/main/us_states_bounding_boxes.json"
NONE_LOCATION = ("== None ==", "")
ALL_STATES = ("All States (~4.8 GB)", "All")


def fetch_locations(url: str, first: tuple[str, str]) -> list[tuple[str, str]]:
  """[(full name, code)], sorted by name, with `first` on top."""
  resp = requests.get(url, timeout=10)
  resp.raise_for_status()
  locations = sorted(((v.get("full_name", code), code) for code, v in resp.json().items()), key=lambda x: x[0])
  return [first, *locations]


def dir_size(path: str) -> int:
  total = 0
  for root, _, files in os.walk(path):
    for f in files:
      try:
        total += os.path.getsize(os.path.join(root, f))
      except OSError:
        pass
  return total


def format_size(size: int | None) -> str:
  if size is None:
    return tr("Calculating...")
  gb = 1024 ** 3
  return f"{size / gb:.2f} GB" if size >= gb else f"{size / 1024 ** 2:.2f} MB"


def format_time(seconds: float) -> str:
  minutes, secs = divmod(int(seconds), 60)
  return (f"{minutes}" + tr("m ") if minutes > 0 else "") + f"{secs}" + tr("s")


class OsmState:
  def __init__(self):
    self.params = Params()
    self.mem_params = self.params if PC else Params("/dev/shm/params")
    self.maps_size: int | None = None
    self.eta_text = ""
    self.download_start: float | None = None
    self._rate_history: list[float] = []
    self._size_thread: threading.Thread | None = None

  # *** download progress ***

  def downloading(self) -> bool:
    return bool(self.mem_params.get("OSMDownloadLocations"))

  def progress(self) -> tuple[int, int]:
    progress = self.params.get("OSMDownloadProgress") or {}
    if isinstance(progress, (str, bytes)):
      try:
        progress = json.loads(progress)
      except ValueError:
        progress = {}
    return int(progress.get("total_files", 0) or 0), int(progress.get("downloaded_files", 0) or 0)

  def last_downloaded(self) -> float | None:
    try:
      ts = float(self.params.get("OsmDownloadedDate") or 0)
    except (TypeError, ValueError):
      return None
    return ts if ts > 0 else None

  def elapsed_text(self) -> str:
    total, done = self.progress()
    if total <= 0 or done >= total:
      return tr("Downloaded")
    if self.download_start is None or done == 0:
      return tr("Calculating...")
    return format_time(time.monotonic() - self.download_start)

  def status_text(self) -> str:
    total, done = self.progress()
    downloading = self.downloading()
    if self.params.get_bool("OsmDbUpdatesCheck") and not downloading and not total:
      return tr("Download starting...")
    if downloading and total > done:
      return f"{done}/{total} ({int(100 * done / total)}%)"
    if downloading and total and done >= total:
      return tr("Download complete!")
    last = self.last_downloaded()
    if last is not None:
      return datetime.datetime.fromtimestamp(last).strftime("%Y-%m-%d %H:%M:%S")
    return ""

  def _update_eta(self):
    total, done = self.progress()
    if total <= 0 or done >= total:
      self.eta_text = tr("Ready") if total <= 0 else tr("Downloaded")
      return
    if self.download_start is None or done == 0:
      self.eta_text = tr("Calculating ETA...")
      return
    elapsed = time.monotonic() - self.download_start
    if elapsed <= 0:
      return
    self._rate_history = (self._rate_history + [done / elapsed])[-10:]
    if len(self._rate_history) < 3:
      self.eta_text = tr("Calculating ETA...")
      return
    weights = range(1, len(self._rate_history) + 1)
    rate = sum(r * w for r, w in zip(self._rate_history, weights, strict=True)) / sum(weights)
    if rate > 0:
      self.eta_text = tr("Time remaining: ") + format_time((total - done) / rate)

  def refresh(self):
    """Call every few seconds (every second while downloading)."""
    now = time.monotonic()
    if self.downloading():
      if self.download_start is None:
        self.download_start = now
        self._rate_history = []
      self._update_eta()
    else:
      self.download_start = None

    if self._size_thread is None or not self._size_thread.is_alive():
      def measure():
        self.maps_size = dir_size(MAP_PATH)
      self._size_thread = threading.Thread(target=measure, daemon=True)
      self._size_thread.start()

    # same clean up as the Qt panel
    location = self.params.get("OsmLocationName")
    if location:
      if location != "US":
        self.params.remove("OsmStateName")
        self.params.remove("OsmStateTitle")
    else:
      for key in ("OsmLocal", "OsmLocationName", "OsmLocationTitle", "OsmStateName", "OsmStateTitle"):
        self.params.remove(key)

  # *** actions ***

  def start_download(self):
    self.params.put_bool("OsmDbUpdatesCheck", True, block=True)

  def delete_maps(self):
    """Blocking, run it in a thread."""
    shutil.rmtree(MAP_PATH, ignore_errors=True)
    self.maps_size = None

  def set_country(self, title: str, code: str):
    self.params.put_bool("OsmLocal", True, block=True)
    self.params.put("OsmLocationTitle", title, block=True)
    self.params.put("OsmLocationName", code, block=True)

  def set_state(self, title: str, code: str):
    self.params.put("OsmStateTitle", title, block=True)
    self.params.put("OsmStateName", code, block=True)
