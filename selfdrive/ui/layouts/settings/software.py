import os
import re
import time
import datetime
from openpilot.common.time_helpers import system_time_valid
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, trn
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.selfdrive.ui.layouts.settings.top_common import read_error_log, error_log_html
from openpilot.system.hardware import HARDWARE
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog, alert_dialog
from openpilot.system.ui.widgets.html_render import HtmlModal
from openpilot.system.ui.widgets.list_view import button_item, text_item, ListItem
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
from openpilot.system.ui.widgets.scroller_tici import Scroller

# TODO: remove this. updater fails to respond on startup if time is not correct
UPDATED_TIMEOUT = 10  # seconds to wait for updated to respond

# TOP
TOP_C3_BRANCH_RE = re.compile(r"^TOP\d+-c3$", re.IGNORECASE)

# Mapping updater internal states to translated display strings
STATE_TO_DISPLAY_TEXT = {
  "checking...": tr("checking..."),
  "downloading...": tr("downloading..."),
  "finalizing update...": tr("finalizing update..."),
}


def time_ago(date: datetime.datetime | None) -> str:
  if not date:
    return tr("never")

  if not system_time_valid():
    return date.strftime("%a %b %d %Y")

  now = datetime.datetime.now(datetime.UTC)
  if date.tzinfo is None:
    date = date.replace(tzinfo=datetime.UTC)

  diff_seconds = int((now - date).total_seconds())
  if diff_seconds < 60:
    return tr("now")
  if diff_seconds < 3600:
    m = diff_seconds // 60
    return trn("{} minute ago", "{} minutes ago", m).format(m)
  if diff_seconds < 86400:
    h = diff_seconds // 3600
    return trn("{} hour ago", "{} hours ago", h).format(h)
  if diff_seconds < 604800:
    d = diff_seconds // 86400
    return trn("{} day ago", "{} days ago", d).format(d)
  return date.strftime("%a %b %d %Y")


class SoftwareLayout(Widget):
  def __init__(self):
    super().__init__()

    self._onroad_label = ListItem(lambda: tr("Updates are only downloaded while the car is off."))
    self._version_item = text_item(lambda: tr("Current Version"), ui_state.params.get("UpdaterCurrentDescription") or "")
    self._download_btn = button_item(lambda: tr("Download"), lambda: tr("CHECK"), callback=self._on_download_update)

    # Install button is initially hidden
    self._install_btn = button_item(lambda: tr("Install Update"), lambda: tr("INSTALL"), callback=self._on_install_update)
    self._install_btn.set_visible(False)

    # Track waiting-for-updater transition to avoid brief re-enable while still idle
    self._waiting_for_updater = False
    self._waiting_start_ts: float = 0.0

    # Branch switcher
    self._branch_btn = button_item(lambda: tr("Target Branch"), lambda: tr("SELECT"), callback=self._on_select_branch)
    self._branch_btn.set_visible(not ui_state.params.get_bool("IsTestedBranch"))
    self._branch_btn.action_item.set_value(ui_state.params.get("UpdaterTargetBranch") or "")
    self._branch_dialog: MultiOptionDialog | None = None

    # TOP: keep openpilot offroad with the car on (dp_device_offroad, cleared on manager start)
    self._offroad_btn = button_item(lambda: tr("Onroad/Offroad Mode"),
                                    lambda: tr("Go Onroad") if ui_state.params.get_bool("dp_device_offroad") else tr("Go Offroad"),
                                    lambda: tr("Keep openpilot offroad while the car is on, e.g. to change settings or update. " +
                                               "Back to normal after a reboot. Not available while engaged."),
                                    callback=self._on_offroad_mode, enabled=lambda: not ui_state.engaged)
    # TOP: last crash traceback
    self._error_log_btn = button_item(lambda: tr("Error Log"), lambda: tr("VIEW"),
                                      lambda: tr("View the error log for debugging purposes when openpilot crashes."),
                                      callback=self._on_error_log)

    self._scroller = Scroller([
      self._onroad_label,
      self._version_item,
      self._offroad_btn,
      self._download_btn,
      self._install_btn,
      self._branch_btn,
      button_item(lambda: tr("Uninstall"), lambda: tr("UNINSTALL"), callback=self._on_uninstall),
      self._error_log_btn,
    ], line_separator=True, spacing=0)

  def show_event(self):
    super().show_event()
    self._scroller.show_event()

  def _render(self, rect):
    self._scroller.render(rect)

  def _update_state(self):
    # Show/hide onroad warning
    self._onroad_label.set_visible(ui_state.is_onroad())

    # Update current version and release notes
    current_desc = ui_state.params.get("UpdaterCurrentDescription") or ""
    current_release_notes = (ui_state.params.get("UpdaterCurrentReleaseNotes") or b"").decode("utf-8", "replace")
    self._version_item.action_item.set_text(current_desc)
    self._version_item.set_description(current_release_notes)

    # Update download button visibility and state
    self._download_btn.set_visible(ui_state.is_offroad())

    updater_state = ui_state.params.get("UpdaterState") or "idle"
    failed_count = ui_state.params.get("UpdateFailedCount") or 0
    fetch_available = ui_state.params.get_bool("UpdaterFetchAvailable")
    update_available = ui_state.params.get_bool("UpdateAvailable")

    if updater_state != "idle":
      # Updater responded
      self._waiting_for_updater = False
      self._download_btn.action_item.set_enabled(False)
      # Use the mapping, with a fallback to the original state string
      display_text = STATE_TO_DISPLAY_TEXT.get(updater_state, updater_state)
      self._download_btn.action_item.set_value(display_text)
    else:
      if failed_count > 0:
        self._download_btn.action_item.set_value(tr("failed to check for update"))
        self._download_btn.action_item.set_text(tr("CHECK"))
      elif fetch_available:
        self._download_btn.action_item.set_value(tr("update available"))
        self._download_btn.action_item.set_text(tr("DOWNLOAD"))
      else:
        last_update = ui_state.params.get("LastUpdateTime")
        if last_update:
          formatted = time_ago(last_update)
          self._download_btn.action_item.set_value(tr("up to date, last checked {}").format(formatted))
        else:
          self._download_btn.action_item.set_value(tr("up to date, last checked never"))
        self._download_btn.action_item.set_text(tr("CHECK"))

      # If we've been waiting too long without a state change, reset state
      if self._waiting_for_updater and (time.monotonic() - self._waiting_start_ts > UPDATED_TIMEOUT):
        self._waiting_for_updater = False

      # Only enable if we're not waiting for updater to flip out of idle
      self._download_btn.action_item.set_enabled(not self._waiting_for_updater)

    # Update target branch button value
    current_branch = ui_state.params.get("UpdaterTargetBranch") or ""
    self._branch_btn.action_item.set_value(current_branch)

    # Update install button
    self._install_btn.set_visible(ui_state.is_offroad() and update_available)
    if update_available:
      new_desc = ui_state.params.get("UpdaterNewDescription") or ""
      new_release_notes = (ui_state.params.get("UpdaterNewReleaseNotes") or b"").decode("utf-8", "replace")
      self._install_btn.action_item.set_text(tr("INSTALL"))
      self._install_btn.action_item.set_value(new_desc)
      self._install_btn.set_description(new_release_notes)
      # Enable install button for testing (like Qt showEvent)
      self._install_btn.action_item.set_enabled(True)
    else:
      self._install_btn.set_visible(False)

  def _on_download_update(self):
    # Check if we should start checking or start downloading
    self._download_btn.action_item.set_enabled(False)
    if self._download_btn.action_item.text == tr("CHECK"):
      # Start checking for updates
      self._waiting_for_updater = True
      self._waiting_start_ts = time.monotonic()
      os.system("pkill -SIGUSR1 -f system.updated.updated")
    else:
      # Start downloading
      self._waiting_for_updater = True
      self._waiting_start_ts = time.monotonic()
      os.system("pkill -SIGHUP -f system.updated.updated")

  def _on_uninstall(self):
    def handle_uninstall_confirmation(result: DialogResult):
      if result == DialogResult.CONFIRM:
        ui_state.params.put_bool("DoUninstall", True, block=True)

    dialog = ConfirmDialog(tr("Are you sure you want to uninstall?"), tr("Uninstall"), callback=handle_uninstall_confirmation)
    gui_app.push_widget(dialog)

  def _on_offroad_mode(self):
    def handle_confirmation(result):
      if result == DialogResult.CONFIRM and not ui_state.engaged:
        ui_state.params.put_bool("dp_device_offroad", not ui_state.params.get_bool("dp_device_offroad"), block=True)

    dialog = ConfirmDialog(tr("Are you sure you want to switch driving mode?"), tr("CONFIRM"), callback=handle_confirmation)
    gui_app.push_widget(dialog)

  def _on_error_log(self):
    text = read_error_log()
    if not text:
      gui_app.push_widget(alert_dialog(tr("No error log found.")))
      return
    gui_app.push_widget(HtmlModal(text=error_log_html(text)))

  def _on_install_update(self):
    # Trigger reboot to install update
    self._install_btn.action_item.set_enabled(False)
    ui_state.params.put_bool("DoReboot", True, block=True)

  def _on_select_branch(self):
    # Get available branches and order
    current_git_branch = ui_state.params.get("GitBranch") or ""
    branches_str = ui_state.params.get("UpdaterAvailableBranches") or ""
    branches = [b for b in branches_str.split(",") if b]

    # TOP: same ordering as the Qt UI. Legacy -c3 branches only run on the comma three.
    def move_to(branch: str, index: int):
      if branch in branches:
        branches.remove(branch)
        branches.insert(index, branch)

    if HARDWARE.get_device_type() == "tici":
      for b in [current_git_branch, "top-c3", "dev-c3", "release-c3"]:
        move_to(b, 0)
      for b in [b for b in branches if TOP_C3_BRANCH_RE.match(b)]:
        move_to(b, 1)
    else:
      branches = [b for b in branches if not b.lower().endswith("-c3")]
      for b in [current_git_branch, "devel-staging", "devel", "nightly", "nightly-dev", "master"]:
        move_to(b, 0)

    current_target = ui_state.params.get("UpdaterTargetBranch") or ""

    def handle_selection(result: DialogResult):
      # Confirmed selection
      if result == DialogResult.CONFIRM and self._branch_dialog is not None and self._branch_dialog.selection:
        selection = self._branch_dialog.selection
        ui_state.params.put("UpdaterTargetBranch", selection, block=True)
        self._branch_btn.action_item.set_value(selection)
        os.system("pkill -SIGUSR1 -f system.updated.updated")
      self._branch_dialog = None

    self._branch_dialog = MultiOptionDialog(tr("Select a branch"), branches, current_target, callback=handle_selection)
    gui_app.push_widget(self._branch_dialog)
