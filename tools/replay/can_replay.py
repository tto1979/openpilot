#!/usr/bin/env python3
import argparse
import os
import time
import threading

os.environ['FILEREADER_CACHE'] = '1'

from openpilot.common.realtime import config_realtime_process, Ratekeeper, DT_CTRL
from openpilot.selfdrive.boardd.boardd import can_capnp_to_can_list
from openpilot.tools.lib.logreader import LogReader
from panda import Panda, PandaJungle

def send_thread(s, flock):
  if "Jungle" in str(type(s)):
    if "FLASH" in os.environ:
      with flock:
        s.flash()

    for i in [0, 1, 2, 3, 0xFFFF]:
      s.can_clear(i)
      s.set_can_speed_kbps(i, 500)
      s.set_can_data_speed_kbps(i, 500)
    s.set_ignition(False)
    time.sleep(5)
    s.set_ignition(True)
    s.set_panda_power(True)
  else:
    s.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  s.set_can_loopback(False)

  idx = 0
  ign = True
  rk = Ratekeeper(1 / DT_CTRL, print_delay_threshold=None)
  while True:
    # handle ignition cycling
    if ENABLE_IGN:
      i = (rk.frame*DT_CTRL) % (IGN_ON + IGN_OFF) < IGN_ON
      if i != ign:
        ign = i
        s.set_ignition(ign)

    snd = CAN_MSGS[idx]
    snd = list(filter(lambda x: x[-1] <= 2, snd))
    s.can_send_many(snd)
    idx = (idx + 1) % len(CAN_MSGS)

    # Drain panda message buffer
    s.can_recv()
    rk.keep_time()


def connect():
  config_realtime_process(3, 55)

  serials = {}
  flashing_lock = threading.Lock()
  while True:
    # look for new devices
    for p in [Panda, PandaJungle]:
      if p is None:
        continue

      for s in p.list():
        if s not in serials:
          with p(s) as pp:
            if pp.get_type() == Panda.HW_TYPE_TRES:
              serials[s] = None
              continue

          print("starting send thread for", s)
          serials[s] = threading.Thread(target=send_thread, args=(p(s), flashing_lock))
          serials[s].start()

    # try to join all send threads
    cur_serials = serials.copy()
    for s, t in cur_serials.items():
      if t is  not None:
        t.join(0.01)
        if not t.is_alive():
          del serials[s]

    time.sleep(1)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Replay CAN messages from a route to all connected pandas and jungles in a loop.",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("route_or_segment_name", nargs='?', help="The route or segment name to replay. If not specified, a default public route will be used.")
  args = parser.parse_args()

  def process(lr):
    return [can_capnp_to_can_list(m.can) for m in lr if m.which() == 'can']

  print("Loading log...")
  if args.route_or_segment_name is None:
    args.route_or_segment_name = "77611a1fac303767/2020-03-24--09-50-38/10:16"

  sr = LogReader(args.route_or_segment_name)

  CAN_MSGS = sr.run_across_segments(24, process)

  print("Finished loading...")

  # set both to cycle ignition
  IGN_ON = int(os.getenv("ON", "0"))
  IGN_OFF = int(os.getenv("OFF", "0"))
  ENABLE_IGN = IGN_ON > 0 and IGN_OFF > 0
  if ENABLE_IGN:
    print(f"Cycling ignition: on for {IGN_ON}s, off for {IGN_OFF}s")

  connect()
