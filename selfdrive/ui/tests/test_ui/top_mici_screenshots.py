#!/usr/bin/env python3
"""
Screenshots of TOP's comma four (mici) settings pages, saved next to the mici UI report
(selfdrive/ui/tests/diff/report/top) so CI uploads them with it.
"""
import os
import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.ui.layouts.settings.top_common import ERROR_LOG_PATH
from openpilot.selfdrive.ui.tests.diff.diff import DIFF_OUT_DIR
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.ui_state import ui_state

OUT_DIR = os.path.join(DIFF_OUT_DIR, "top")
FRAMES_PER_STEP = 30


def setup_params():
  params = Params()
  params.put("OsmLocationName", "TW")
  params.put("OsmLocationTitle", "Taiwan")
  params.put("MapdVersion", "v1.11.0")
  params.put("SpeedLimitOffsetType", 1)
  params.put("SpeedLimitValueOffset", 5)
  params.put("CarModel", "TOYOTA_COROLLA_TSS2")
  params.put_bool("NNFF", True)
  try:
    os.makedirs(os.path.dirname(ERROR_LOG_PATH), exist_ok=True)
    with open(ERROR_LOG_PATH, "w") as f:
      f.write("\n".join(["Traceback (most recent call last):",
                         '  File "/data/openpilot/selfdrive/controls/controlsd.py", line 212, in <module>',
                         "    main()",
                         "KeyError: 'LiveTorqueParameters'"]))
  except OSError:
    pass


def main():
  from openpilot.selfdrive.ui.mici.layouts.settings.settings import SettingsLayout, PanelType
  from openpilot.selfdrive.ui.mici.widgets.top_dialogs import FootageDialogMici

  setup_params()
  os.makedirs(OUT_DIR, exist_ok=True)
  rl.set_config_flags(rl.FLAG_WINDOW_HIDDEN)
  gui_app.init_window("top mici screenshots")
  # draw into an offscreen texture and read it back: reading the window framebuffer mid-frame is unreliable on xvfb
  if gui_app._render_texture is None:
    gui_app._render_texture = rl.load_render_texture(gui_app.width, gui_app.height)
  settings = SettingsLayout()
  settings.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
  settings.show_event()

  def panel(t):
    return lambda: settings._set_current_panel(t)

  def scroll(t, offset):
    def f():
      sp = settings._panels[t].instance._scroller.scroll_panel
      sp.set_offset(offset)
    return f

  def close_overlay():
    gui_app.set_modal_overlay(None)
    gui_app._modal_overlay_shown = False  # normally reset by a frame without a modal

  top = settings._panels[PanelType.TOP].instance
  maps = settings._panels[PanelType.MAPS].instance
  steps = [
    ("hub", lambda: None),
    ("hub_2", lambda: settings._scroller.scroll_panel.set_offset(-420)),
    ("top", panel(PanelType.TOP)),
    ("top_2", scroll(PanelType.TOP, -1300)),
    ("top_car_picker", lambda: (setattr(top, "_car_list", ["TOYOTA_COROLLA_TSS2", "TOYOTA_RAV4_TSS2", "LEXUS_ES_TSS2"]),
                                top._show_car_dialog())),
    ("maps", lambda: (close_overlay(), panel(PanelType.MAPS)())),
    ("maps_2", scroll(PanelType.MAPS, -1300)),
    ("maps_source_picker", lambda: maps._select_source()),
    ("maps_offset_picker", lambda: (close_overlay(), maps._select_offset())),
    ("toggles", lambda: (close_overlay(), panel(PanelType.TOGGLES)())),
    ("device_top", lambda: (panel(PanelType.DEVICE)(), scroll(PanelType.DEVICE, -1700)())),
    ("device_top_2", scroll(PanelType.DEVICE, -2500)),
    ("device_footage", lambda: gui_app.set_modal_overlay(FootageDialogMici())),
    ("device_error_log", lambda: (close_overlay(), settings._panels[PanelType.DEVICE].instance._show_error_log())),
  ]

  frame = 0
  step = 0
  steps[0][1]()
  for _ in gui_app.render():
    ui_state.update()
    if gui_app._modal_overlay.overlay is None:
      settings.render()
    frame += 1
    if frame % FRAMES_PER_STEP == 0:
      rl.rl_draw_render_batch_active()
      image = rl.load_image_from_texture(gui_app._render_texture.texture)
      rl.image_flip_vertical(image)
      rl.export_image(image, os.path.join(OUT_DIR, f"{step:02d}_{steps[step][0]}.png"))
      rl.unload_image(image)
      step += 1
      if step >= len(steps):
        break
      steps[step][1]()
  gui_app.close()
  print(f"saved {step} screenshots to {OUT_DIR}")


if __name__ == "__main__":
  main()
