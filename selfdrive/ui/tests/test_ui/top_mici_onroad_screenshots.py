#!/usr/bin/env python3
"""
Screenshots of TOP's comma four (mici) onroad HUD with fake messages, saved next to the mici UI report
(selfdrive/ui/tests/diff/report/top). No camera or model: only the HUD on the camera placeholder.
"""
import os
import sys

import pyray as rl
from cereal import messaging
from openpilot.common.params import Params
from openpilot.selfdrive.ui.tests.diff.diff import DIFF_OUT_DIR
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.ui_state import ui_state

OUT_DIR = os.path.join(DIFF_OUT_DIR, "top")
FRAMES = 50

# name, metric, scenario
SCENES = [
  ("onroad_metric", True, "normal"),
  ("onroad_imperial_assist", False, "assist"),
  ("onroad_alka", True, "alka"),
]


def build_messages(scenario: str) -> dict:
  msgs: dict = {}

  def m(name):
    msg = messaging.new_message(name)
    msgs[name] = msg
    return getattr(msg, name)

  ds = m("deviceState")
  ds.started = True
  ds.deviceType = "mici"
  ps = messaging.new_message("pandaStates", 1)
  ps.pandaStates[0].ignitionLine = True
  ps.pandaStates[0].pandaType = "tres"
  msgs["pandaStates"] = ps
  msgs["onroadEvents"] = messaging.new_message("onroadEvents", 0)

  engaged = scenario != "alka"
  ss = m("selfdriveState")
  ss.enabled = engaged
  ss.state = "enabled" if engaged else "disabled"
  ss.engageable = True
  cs = m("carState")
  cs.vEgo = cs.vEgoCluster = 25
  cs.vCruiseCluster = 100
  cs.leftBlinker = True
  cs.leftBlindspot = scenario == "normal"
  cs.rightBlindspot = scenario == "assist"
  m("carParams").openpilotLongitudinalControl = True
  for name in ("controlsState", "carControl", "driverMonitoringState", "longitudinalPlan", "radarState", "modelV2", "liveCalibration"):
    m(name)
  m("roadCameraState").sensor = "os04c10"

  lp = m("longitudinalPlanTOP")
  r = lp.speedLimit.resolver
  r.speedLimit = r.speedLimitLast = 80 / 3.6
  r.speedLimitValid = r.speedLimitLastValid = True
  r.speedLimitOffset = 5 / 3.6
  r.speedLimitFinalLast = 85 / 3.6
  r.source = "map"
  if scenario == "assist":
    lp.speedLimit.assist.state = "preActive"
  scc = lp.smartCruiseControl
  scc.vision.enabled = scc.map.enabled = True
  scc.map.active = True
  lmd = m("liveMapDataTOP")
  lmd.speedLimitAheadValid = True
  lmd.speedLimitAhead = 60 / 3.6
  m("topControlsState").alkaActive = scenario == "alka"
  return msgs


def render_scene(name: str, metric: bool, scenario: str):
  from openpilot.selfdrive.ui.mici.onroad.augmented_road_view import AugmentedRoadView

  params = Params()
  params.put_bool("IsMetric", metric)
  params.put("SpeedLimitMode", 2)

  view = AugmentedRoadView()
  view._model_renderer.set_visible(False)  # no camera or model data, HUD only

  msgs = build_messages(scenario)
  sm = ui_state.sm
  lmd = msgs["liveMapDataTOP"].liveMapDataTOP
  distance = 400.0

  def fake_update():
    nonlocal distance
    sm.frame += 1
    distance -= 2  # approaching the next limit
    lmd.speedLimitAheadDistance = distance
    for service, msg in msgs.items():
      sm.data[service] = getattr(msg, service)
      sm.recv_frame[service] = sm.frame
      sm.updated[service] = sm.alive[service] = sm.valid[service] = sm.seen[service] = True
    ui_state._update_state()
    ui_state._update_status()

  ui_state.update_params()
  ui_state.has_longitudinal_control = True
  for frame, _ in enumerate(gui_app.render()):
    fake_update()
    view.render(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
    if frame == FRAMES:
      rl.rl_draw_render_batch_active()
      assert gui_app._render_texture is not None
      image = rl.load_image_from_texture(gui_app._render_texture.texture)
      rl.image_flip_vertical(image)
      rl.export_image(image, os.path.join(OUT_DIR, f"{name}.png"))
      rl.unload_image(image)
      break


def main():
  os.makedirs(OUT_DIR, exist_ok=True)
  rl.set_config_flags(rl.FLAG_WINDOW_HIDDEN)
  gui_app.init_window("top mici onroad screenshots")
  # draw into an offscreen texture and read it back: reading the window framebuffer mid-frame is unreliable on xvfb
  if gui_app._render_texture is None:
    gui_app._render_texture = rl.load_render_texture(gui_app.width, gui_app.height)

  scenes = [s for s in SCENES if len(sys.argv) < 2 or s[0] in sys.argv[1:]]
  for name, metric, scenario in scenes:
    render_scene(name, metric, scenario)
  print(f"saved {len(scenes)} onroad screenshots to {OUT_DIR}", flush=True)
  # skip gui_app.close() and the destructors: widgets unloading textures after the window is gone segfault on exit
  os._exit(0)


if __name__ == "__main__":
  main()
