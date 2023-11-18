#!/usr/bin/env python3
import capnp
import os
import importlib
import pytest
import unittest
from collections import defaultdict, Counter
from typing import List, Optional, Tuple
from parameterized import parameterized_class

from cereal import log, car
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.fingerprints import all_known_cars
from openpilot.selfdrive.car.car_helpers import FRAME_FINGERPRINT, interfaces
from openpilot.selfdrive.car.honda.values import CAR as HONDA, HONDA_BOSCH
from openpilot.selfdrive.car.tests.routes import non_tested_cars, routes, CarTestRoute
from openpilot.selfdrive.controls.controlsd import Controls
from openpilot.selfdrive.test.openpilotci import get_url
from openpilot.tools.lib.logreader import LogReader
from openpilot.tools.lib.route import Route, SegmentName, RouteName

from panda.tests.libpanda import libpanda_py

EventName = car.CarEvent.EventName
PandaType = log.PandaState.PandaType
SafetyModel = car.CarParams.SafetyModel

NUM_JOBS = int(os.environ.get("NUM_JOBS", "1"))
JOB_ID = int(os.environ.get("JOB_ID", "0"))
INTERNAL_SEG_LIST = os.environ.get("INTERNAL_SEG_LIST", "")
INTERNAL_SEG_CNT = int(os.environ.get("INTERNAL_SEG_CNT", "0"))


def get_test_cases() -> List[Tuple[str, Optional[CarTestRoute]]]:
  # build list of test cases
  test_cases = []
  if not len(INTERNAL_SEG_LIST):
    routes_by_car = defaultdict(set)
    for r in routes:
      routes_by_car[r.car_model].add(r)

    for i, c in enumerate(sorted(all_known_cars())):
      if i % NUM_JOBS == JOB_ID:
        test_cases.extend(sorted((c.value, r) for r in routes_by_car.get(c, (None,))))

  else:
    with open(os.path.join(BASEDIR, INTERNAL_SEG_LIST), "r") as f:
      seg_list = f.read().splitlines()

    cnt = INTERNAL_SEG_CNT or len(seg_list)
    seg_list_iter = iter(seg_list[:cnt])

    for platform in seg_list_iter:
      platform = platform[2:]  # get rid of comment
      segment_name = SegmentName(next(seg_list_iter))
      test_cases.append((platform, CarTestRoute(segment_name.route_name.canonical_name, platform,
                                                segment=segment_name.segment_num)))
  return test_cases


@pytest.mark.slow
class TestCarModelBase(unittest.TestCase):
  car_model: Optional[str] = None
  test_route: Optional[CarTestRoute] = None
  ci: bool = True

  can_msgs: List[capnp.lib.capnp._DynamicStructReader]
  elm_frame: Optional[int]
  car_safety_mode_frame: Optional[int]

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == 'TestCarModel' or cls.__name__.endswith('Base'):
      raise unittest.SkipTest

    if 'FILTER' in os.environ:
      if not cls.car_model.startswith(tuple(os.environ.get('FILTER').split(','))):
        raise unittest.SkipTest

    if cls.test_route is None:
      if cls.car_model in non_tested_cars:
        print(f"Skipping tests for {cls.car_model}: missing route")
        raise unittest.SkipTest
      raise Exception(f"missing test route for {cls.car_model}")

    test_segs = (2, 1, 0)
    if cls.test_route.segment is not None:
      test_segs = (cls.test_route.segment,)

    for seg in test_segs:
      try:
        if len(INTERNAL_SEG_LIST):
          route_name = RouteName(cls.test_route.route)
          lr = LogReader(f"cd:/{route_name.dongle_id}/{route_name.time_str}/{seg}/rlog.bz2")
        elif cls.ci:
          lr = LogReader(get_url(cls.test_route.route, seg))
        else:
          lr = LogReader(Route(cls.test_route.route).log_paths()[seg])
      except Exception:
        continue

      car_fw = []
      can_msgs = []
      cls.elm_frame = None
      cls.car_safety_mode_frame = None
      fingerprint = defaultdict(dict)
      experimental_long = False
      for msg in lr:
        if msg.which() == "can":
          can_msgs.append(msg)
          if len(can_msgs) <= FRAME_FINGERPRINT:
            for m in msg.can:
              if m.src < 64:
                fingerprint[m.src][m.address] = len(m.dat)

        elif msg.which() == "carParams":
          car_fw = msg.carParams.carFw
          if msg.carParams.openpilotLongitudinalControl:
            experimental_long = True
          if cls.car_model is None and not cls.ci:
            cls.car_model = msg.carParams.carFingerprint

        # Log which can frame the panda safety mode left ELM327, for CAN validity checks
        elif msg.which() == 'pandaStates':
          for ps in msg.pandaStates:
            if cls.elm_frame is None and ps.safetyModel != SafetyModel.elm327:
              cls.elm_frame = len(can_msgs)
            if cls.car_safety_mode_frame is None and ps.safetyModel not in \
              (SafetyModel.elm327, SafetyModel.noOutput):
              cls.car_safety_mode_frame = len(can_msgs)

        elif msg.which() == 'pandaStateDEPRECATED':
          if cls.elm_frame is None and msg.pandaStateDEPRECATED.safetyModel != SafetyModel.elm327:
            cls.elm_frame = len(can_msgs)
          if cls.car_safety_mode_frame is None and msg.pandaStateDEPRECATED.safetyModel not in \
            (SafetyModel.elm327, SafetyModel.noOutput):
            cls.car_safety_mode_frame = len(can_msgs)

      if len(can_msgs) > int(50 / DT_CTRL):
        break
    else:
      raise Exception(f"Route: {repr(cls.test_route.route)} with segments: {test_segs} not found or no CAN msgs found. Is it uploaded?")

    # if relay is expected to be open in the route
    cls.openpilot_enabled = cls.car_safety_mode_frame is not None

    cls.can_msgs = sorted(can_msgs, key=lambda msg: msg.logMonoTime)

    cls.CarInterface, cls.CarController, cls.CarState = interfaces[cls.car_model]
    cls.CP = cls.CarInterface.get_params(cls.car_model, fingerprint, car_fw, experimental_long, docs=False)
    assert cls.CP
    assert cls.CP.carFingerprint == cls.car_model

  @classmethod
  def tearDownClass(cls):
    del cls.can_msgs

  def setUp(self):
    self.CI = self.CarInterface(self.CP.copy(), self.CarController, self.CarState)
    assert self.CI

    Params().put_bool("OpenpilotEnabledToggle", self.openpilot_enabled)

    # TODO: check safetyModel is in release panda build
    self.safety = libpanda_py.libpanda

    cfg = self.CP.safetyConfigs[-1]
    set_status = self.safety.set_safety_hooks(cfg.safetyModel.raw, cfg.safetyParam)
    self.assertEqual(0, set_status, f"failed to set safetyModel {cfg}")
    self.safety.init_tests()

  def test_car_params(self):
    if self.CP.dashcamOnly:
      self.skipTest("no need to check carParams for dashcamOnly")

    # make sure car params are within a valid range
    self.assertGreater(self.CP.mass, 1)

    if self.CP.steerControlType != car.CarParams.SteerControlType.angle:
      tuning = self.CP.lateralTuning.which()
      if tuning == 'pid':
        self.assertTrue(len(self.CP.lateralTuning.pid.kpV))
      elif tuning == 'torque':
        self.assertTrue(self.CP.lateralTuning.torque.kf > 0)
      else:
        raise Exception("unknown tuning")

  def test_car_interface(self):
    # TODO: also check for checksum violations from can parser
    can_invalid_cnt = 0
    can_valid = False
    CC = car.CarControl.new_message()

    for i, msg in enumerate(self.can_msgs):
      CS = self.CI.update(CC, (msg.as_builder().to_bytes(),))
      self.CI.apply(CC, msg.logMonoTime)

      if CS.canValid:
        can_valid = True

      # wait max of 2s for low frequency msgs to be seen
      if i > 200 or can_valid:
        can_invalid_cnt += not CS.canValid

    self.assertEqual(can_invalid_cnt, 0)

  def test_radar_interface(self):
    os.environ['NO_RADAR_SLEEP'] = "1"
    RadarInterface = importlib.import_module(f'selfdrive.car.{self.CP.carName}.radar_interface').RadarInterface
    RI = RadarInterface(self.CP)
    assert RI

    # Since OBD port is multiplexed to bus 1 (commonly radar bus) while fingerprinting,
    # start parsing CAN messages after we've left ELM mode and can expect CAN traffic
    error_cnt = 0
    for i, msg in enumerate(self.can_msgs[self.elm_frame:]):
      rr = RI.update((msg.as_builder().to_bytes(),))
      if rr is not None and i > 50:
        error_cnt += car.RadarData.Error.canError in rr.errors
    self.assertEqual(error_cnt, 0)

  def test_panda_safety_rx_valid(self):
    if self.CP.dashcamOnly:
      self.skipTest("no need to check panda safety for dashcamOnly")

    start_ts = self.can_msgs[0].logMonoTime

    failed_addrs = Counter()
    for can in self.can_msgs:
      # update panda timer
      t = (can.logMonoTime - start_ts) / 1e3
      self.safety.set_timer(int(t))

      # run all msgs through the safety RX hook
      for msg in can.can:
        if msg.src >= 64:
          continue

        to_send = libpanda_py.make_CANPacket(msg.address, msg.src % 4, msg.dat)
        if self.safety.safety_rx_hook(to_send) != 1:
          failed_addrs[hex(msg.address)] += 1

      # ensure all msgs defined in the addr checks are valid
      self.safety.safety_tick_current_rx_checks()
      if t > 1e6:
        self.assertTrue(self.safety.addr_checks_valid())

      # Don't check relay malfunction on disabled routes (relay closed),
      # or before fingerprinting is done (elm327 and noOutput)
      if self.openpilot_enabled and t / 1e4 > self.car_safety_mode_frame:
        self.assertFalse(self.safety.get_relay_malfunction())
      else:
        self.safety.set_relay_malfunction(False)

    self.assertFalse(len(failed_addrs), f"panda safety RX check failed: {failed_addrs}")

  def test_panda_safety_tx_cases(self, data=None):
    """Asserts we can tx common messages"""
    if self.CP.notCar:
      self.skipTest("Skipping test for notCar")

    def test_car_controller(car_control):
      now_nanos = 0
      msgs_sent = 0
      CI = self.CarInterface(self.CP, self.CarController, self.CarState)
      for _ in range(round(10.0 / DT_CTRL)):  # make sure we hit the slowest messages
        CI.update(car_control, [])
        _, sendcan = CI.apply(car_control, now_nanos)

        now_nanos += DT_CTRL * 1e9
        msgs_sent += len(sendcan)
        for addr, _, dat, bus in sendcan:
          to_send = libpanda_py.make_CANPacket(addr, bus % 4, dat)
          self.assertTrue(self.safety.safety_tx_hook(to_send), (addr, dat, bus))

      # Make sure we attempted to send messages
      self.assertGreater(msgs_sent, 50)

    # Make sure we can send all messages while inactive
    CC = car.CarControl.new_message()
    test_car_controller(CC)

    # Test cancel + general messages (controls_allowed=False & cruise_engaged=True)
    self.safety.set_cruise_engaged_prev(True)
    CC = car.CarControl.new_message(cruiseControl={'cancel': True})
    test_car_controller(CC)

    # Test resume + general messages (controls_allowed=True & cruise_engaged=True)
    self.safety.set_controls_allowed(True)
    CC = car.CarControl.new_message(cruiseControl={'resume': True})
    test_car_controller(CC)

  def test_panda_safety_carstate(self):
    """
      Assert that panda safety matches openpilot's carState
    """
    if self.CP.dashcamOnly:
      self.skipTest("no need to check panda safety for dashcamOnly")

    CC = car.CarControl.new_message()

    # warm up pass, as initial states may be different
    for can in self.can_msgs[:300]:
      self.CI.update(CC, (can.as_builder().to_bytes(), ))
      for msg in filter(lambda m: m.src in range(64), can.can):
        to_send = libpanda_py.make_CANPacket(msg.address, msg.src % 4, msg.dat)
        self.safety.safety_rx_hook(to_send)

    controls_allowed_prev = False
    CS_prev = car.CarState.new_message()
    checks = defaultdict(lambda: 0)
    controlsd = Controls(CI=self.CI)
    controlsd.initialized = True
    for idx, can in enumerate(self.can_msgs):
      CS = self.CI.update(CC, (can.as_builder().to_bytes(), ))
      for msg in filter(lambda m: m.src in range(64), can.can):
        to_send = libpanda_py.make_CANPacket(msg.address, msg.src % 4, msg.dat)
        ret = self.safety.safety_rx_hook(to_send)
        self.assertEqual(1, ret, f"safety rx failed ({ret=}): {to_send}")

      # Skip first frame so CS_prev is properly initialized
      if idx == 0:
        CS_prev = CS
        # Button may be left pressed in warm up period
        if not self.CP.pcmCruise:
          self.safety.set_controls_allowed(0)
        continue

      # TODO: check rest of panda's carstate (steering, ACC main on, etc.)

      checks['gasPressed'] += CS.gasPressed != self.safety.get_gas_pressed_prev()
      checks['standstill'] += CS.standstill == self.safety.get_vehicle_moving()

      # TODO: remove this exception once this mismatch is resolved
      brake_pressed = CS.brakePressed
      if CS.brakePressed and not self.safety.get_brake_pressed_prev():
        if self.CP.carFingerprint in (HONDA.PILOT, HONDA.RIDGELINE) and CS.brake > 0.05:
          brake_pressed = False
      checks['brakePressed'] += brake_pressed != self.safety.get_brake_pressed_prev()
      checks['regenBraking'] += CS.regenBraking != self.safety.get_regen_braking_prev()

      if self.CP.pcmCruise:
        # On most pcmCruise cars, openpilot's state is always tied to the PCM's cruise state.
        # On Honda Nidec, we always engage on the rising edge of the PCM cruise state, but
        # openpilot brakes to zero even if the min ACC speed is non-zero (i.e. the PCM disengages).
        if self.CP.carName == "honda" and self.CP.carFingerprint not in HONDA_BOSCH:
          # only the rising edges are expected to match
          if CS.cruiseState.enabled and not CS_prev.cruiseState.enabled:
            checks['controlsAllowed'] += not self.safety.get_controls_allowed()
        else:
          checks['controlsAllowed'] += not CS.cruiseState.enabled and self.safety.get_controls_allowed()

        # TODO: fix notCar mismatch
        if not self.CP.notCar:
          checks['cruiseState'] += CS.cruiseState.enabled != self.safety.get_cruise_engaged_prev()
      else:
        # Check for enable events on rising edge of controls allowed
        controlsd.update_events(CS)
        controlsd.CS_prev = CS
        button_enable = (any(evt.enable for evt in CS.events) and
                         not any(evt == EventName.pedalPressed for evt in controlsd.events.names))
        mismatch = button_enable != (self.safety.get_controls_allowed() and not controls_allowed_prev)
        checks['controlsAllowed'] += mismatch
        controls_allowed_prev = self.safety.get_controls_allowed()
        if button_enable and not mismatch:
          self.safety.set_controls_allowed(False)

      if self.CP.carName == "honda":
        checks['mainOn'] += CS.cruiseState.available != self.safety.get_acc_main_on()

      CS_prev = CS

    failed_checks = {k: v for k, v in checks.items() if v > 0}
    self.assertFalse(len(failed_checks), f"panda safety doesn't agree with openpilot: {failed_checks}")


@parameterized_class(('car_model', 'test_route'), get_test_cases())
@pytest.mark.xdist_group_class_property('car_model')
class TestCarModel(TestCarModelBase):
  pass


if __name__ == "__main__":
  unittest.main()
