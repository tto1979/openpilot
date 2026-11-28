import asyncio
import json
import time
# for aiortc and its dependencies
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
warnings.filterwarnings("ignore", category=RuntimeWarning) # TODO: remove this when google-crc32c publish a python3.12 wheel

from aiortc import RTCDataChannel
from aiortc.mediastreams import VIDEO_CLOCK_RATE, VIDEO_TIME_BASE
import capnp
from cereal import messaging, log

from openpilot.system.webrtc.webrtcd import CerealOutgoingMessageProxy, CerealIncomingMessageProxy
from openpilot.system.webrtc.device.video import LiveStreamVideoStreamTrack


class TestStreamSession:
  def setup_method(self):
    self.loop = asyncio.new_event_loop()

  def teardown_method(self):
    self.loop.stop()
    self.loop.close()

  @staticmethod
  def _outgoing_proxy(mocker, msgs):
    proxy = CerealOutgoingMessageProxy()
    assert proxy.add_services(["customReservedRawData0"]) == ["customReservedRawData0"]
    mocker.patch.object(messaging, "recv_one_or_none", side_effect=lambda sock: msgs.pop(0) if msgs else None)
    return proxy

  @staticmethod
  def _channel(mocker, buffered=0, state="open"):
    channel = mocker.Mock(spec=RTCDataChannel)
    channel.readyState = state
    channel.bufferedAmount = buffered
    return channel

  @staticmethod
  def _msg(mono_time=123):
    msg = log.Event.new_message()
    msg.logMonoTime = mono_time
    msg.valid = True
    msg.customReservedRawData0 = b"test"
    return msg.as_reader()

  def test_outgoing_proxy(self, mocker):
    expected_dict = {"type": "customReservedRawData0", "logMonoTime": 123, "valid": True, "data": "test"}
    expected_json = json.dumps(expected_dict).encode()

    proxy = self._outgoing_proxy(mocker, [self._msg()])
    channel = self._channel(mocker)
    proxy.add_channel(channel, ["customReservedRawData0"])

    proxy.update()

    channel.send.assert_called_once_with(expected_json)

  def test_outgoing_proxy_rate_limit(self, mocker):
    msgs = [self._msg(i) for i in range(10)]
    proxy = self._outgoing_proxy(mocker, msgs)
    channel = self._channel(mocker)
    proxy.add_channel(channel, ["customReservedRawData0"])

    for _ in range(10):
      proxy.update()

    # one message per 1 / MAX_RATE_HZ, the rest stays in the (conflated) socket
    assert channel.send.call_count == 1
    assert len(msgs) == 9

  def test_outgoing_proxy_backed_up_channel(self, mocker):
    proxy = self._outgoing_proxy(mocker, [self._msg()])
    backed_up = self._channel(mocker, buffered=CerealOutgoingMessageProxy.MAX_BUFFERED_BYTES + 1)
    ok = self._channel(mocker)
    proxy.add_channel(backed_up, ["customReservedRawData0"])
    proxy.add_channel(ok, ["customReservedRawData0"])

    proxy.update()

    backed_up.send.assert_not_called()
    ok.send.assert_called_once()

  def test_outgoing_proxy_closed_channel(self, mocker):
    proxy = self._outgoing_proxy(mocker, [self._msg()])
    closed = self._channel(mocker, state="closed")
    proxy.add_channel(closed, ["customReservedRawData0"])

    proxy.update()

    closed.send.assert_not_called()
    assert closed not in proxy.channels

  def test_incoming_proxy(self, mocker):
    tested_msgs = [
      {"type": "customReservedRawData0", "data": "test"}, # primitive
      {"type": "can", "data": [{"address": 0, "dat": "", "src": 0}]}, # list
      {"type": "testJoystick", "data": {"axes": [0, 0], "buttons": [False]}}, # dict
    ]

    mocked_pubmaster = mocker.MagicMock(spec=messaging.PubMaster)

    proxy = CerealIncomingMessageProxy(mocked_pubmaster)

    for msg in tested_msgs:
      proxy.send(json.dumps(msg).encode())

      mocked_pubmaster.send.assert_called_once()
      mt, md = mocked_pubmaster.send.call_args.args
      assert mt == msg["type"]
      assert isinstance(md, capnp._DynamicStructBuilder)
      assert hasattr(md, msg["type"])

      mocked_pubmaster.reset_mock()

  def test_livestream_track(self, mocker):
    fake_msg = messaging.new_message("livestreamDriverEncodeData")

    config = {"receive.return_value": fake_msg.to_bytes()}
    mocker.patch("msgq.SubSocket", spec=True, **config)
    track = LiveStreamVideoStreamTrack("driver")

    assert track.id.startswith("driver")
    assert track.codec_preference() == "H264"

    for i in range(5):
      packet = self.loop.run_until_complete(track.recv())
      assert packet.time_base == VIDEO_TIME_BASE
      if i == 0:
        start_ns = time.monotonic_ns()
        start_pts = packet.pts
      assert abs(i + packet.pts - (start_pts + (((time.monotonic_ns() - start_ns) * VIDEO_CLOCK_RATE) // 1_000_000_000))) < 450 #5ms
      assert packet.size == 0

  def test_shutdown_stops_open_sessions(self, mocker):
    # the server's shutdown runs inside its event loop: stopping a session there must not call asyncio.run()
    from openpilot.system.webrtc.webrtcd import StreamSession, on_shutdown
    session = StreamSession.__new__(StreamSession)
    session.run_task = self.loop.create_task(asyncio.sleep(10))
    session.post_run_cleanup = mocker.AsyncMock()
    app = {'streams': {'phone': session}}
    self.loop.run_until_complete(on_shutdown(app))
    session.post_run_cleanup.assert_awaited_once()
    assert 'streams' not in app
