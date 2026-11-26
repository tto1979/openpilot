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
CARS = ["HONDA_CIVIC", "LEXUS_ES_TSS2", "LEXUS_RX_TSS2", "TOYOTA_ALPHARD_TSS2", "TOYOTA_CAMRY", "TOYOTA_CHR_TSS2",
        "TOYOTA_COROLLA_TSS2", "TOYOTA_HIGHLANDER_TSS2", "TOYOTA_PRIUS_TSS2", "TOYOTA_RAV4_TSS2", "TOYOTA_RAV4_TSS2_2023"]
COUNTRIES = [("== None ==", ""), ("Taiwan", "TW"), ("Thailand", "TH"), ("United Kingdom of Great Britain and Northern Ireland", "GB"),
             ("United States of America", "US")]
FRAMES_PER_STEP = 60  # nav stack slide in animation


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
  from openpilot.selfdrive.ui.mici.layouts.settings.settings import SettingsLayout
  from openpilot.selfdrive.ui.mici.layouts.settings.top import TopLayoutMici
  from openpilot.selfdrive.ui.mici.layouts.settings.osm import OsmLayoutMici
  from openpilot.selfdrive.ui.mici.layouts.settings.toggles import TogglesLayoutMici
  from openpilot.selfdrive.ui.mici.layouts.settings.device import DeviceLayoutMici
  from openpilot.selfdrive.ui.mici.widgets.top_dialogs import FootageDialogMici

  setup_params()
  os.makedirs(OUT_DIR, exist_ok=True)
  rl.set_config_flags(rl.FLAG_WINDOW_HIDDEN)
  gui_app.init_window("top mici screenshots")
  # draw into an offscreen texture and read it back: reading the window framebuffer mid-frame is unreliable on xvfb
  if gui_app._render_texture is None:
    gui_app._render_texture = rl.load_render_texture(gui_app.width, gui_app.height)
  settings = SettingsLayout()
  gui_app.push_widget(settings)
  top, maps, toggles, device = TopLayoutMici(), OsmLayoutMici(), TogglesLayoutMici(), DeviceLayoutMici()

  def show(panel):
    # back to the settings hub instantly, then slide the panel in
    def f():
      while len(gui_app._nav_stack) > 1:
        gui_app.pop_widget()
      if panel is not None:
        gui_app.push_widget(panel)
    return f

  def scroll(panel, offset):
    return lambda: panel._scroller.scroll_panel.set_offset(offset)

  steps = [
    ("hub", lambda: None),
    ("hub_2", scroll(settings, -420)),
    ("top", show(top)),
    ("top_2", scroll(top, -1300)),
    ("top_car_brand_picker", lambda: (setattr(top, "_car_list", CARS), top._show_car_dialog())),
    ("top_car_model_picker", lambda: (show(top)(), top._show_model_dialog("TOYOTA"))),
    ("maps", show(maps)),
    ("maps_2", scroll(maps, -1300)),
    ("maps_country_picker", lambda: (setattr(maps, "_pending", ("country", COUNTRIES)), maps._handle_pending())),
    ("maps_source_picker", lambda: (show(maps)(), maps._select_source())),
    ("maps_offset_picker", lambda: (show(maps)(), maps._select_offset())),
    ("toggles", show(toggles)),
    ("device_top", lambda: (show(device)(), scroll(device, -1700)())),
    ("device_top_2", scroll(device, -2500)),
    ("device_footage", lambda: gui_app.push_widget(FootageDialogMici())),
    ("device_error_log", lambda: (show(device)(), scroll(device, -2500)(), device._show_error_log())),
  ]

  frame = 0
  step = 0
  steps[0][1]()
  for _ in gui_app.render():
    ui_state.update()
    frame += 1
    if frame % FRAMES_PER_STEP == 0:
      rl.rl_draw_render_batch_active()
      assert gui_app._render_texture is not None
      image = rl.load_image_from_texture(gui_app._render_texture.texture)
      rl.image_flip_vertical(image)
      rl.export_image(image, os.path.join(OUT_DIR, f"{step:02d}_{steps[step][0]}.png"))
      rl.unload_image(image)
      step += 1
      if step >= len(steps):
        break
      steps[step][1]()
  print(f"saved {step} screenshots to {OUT_DIR}", flush=True)
  # skip gui_app.close() and the destructors: widgets unloading textures after the window is gone segfault on exit
  os._exit(0)


if __name__ == "__main__":
  main()
