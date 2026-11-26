#!/usr/bin/env python3

import argparse
import asyncio
import json
import os
import struct
import time
import uuid
import logging
from dataclasses import dataclass, field
from typing import Any, TYPE_CHECKING

# aiortc and its dependencies have lots of internal warnings :(
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
warnings.filterwarnings("ignore", category=RuntimeWarning) # TODO: remove this when google-crc32c publish a python3.12 wheel

import capnp
from aiohttp import web
if TYPE_CHECKING:
  from aiortc.rtcdatachannel import RTCDataChannel

from openpilot.system.webrtc.schema import generate_field
from cereal import messaging, log


MSGQ_NUM_READERS = 15  # NUM_READERS in msgq_repo/msgq/msgq.h


def msgq_reader_slot_free(service: str) -> bool:
  """Whether one more subscriber fits in a msgq queue. msgq has 15 reader slots per queue: a 16th subscriber
  evicts all of them, they reconnect and evict each other again, so the processes keep losing messages
  (commIssue). Counts the live subscribers of other processes from the queue header (thread id in the low 32
  bits of each reader uid). Unknown (ZMQ, queue not created yet) counts as free."""
  if "ZMQ" in os.environ:
    return True
  prefix = os.environ.get("OPENPILOT_PREFIX")
  path = "/dev/shm/msgq_" + (f"{prefix}/" if prefix else "") + service
  n_fields = 3 + 3 * MSGQ_NUM_READERS  # num_readers, write_pointer, write_uid, read_pointers, read_valids, read_uids
  try:
    with open(path, "rb") as f:
      header = struct.unpack(f"<{n_fields}Q", f.read(8 * n_fields))
  except (OSError, struct.error):
    return True
  num_readers = min(header[0], MSGQ_NUM_READERS)
  read_uids = header[3 + 2 * MSGQ_NUM_READERS:]
  live_tids = set()
  for uid in read_uids[:num_readers]:
    tid = uid & 0xFFFFFFFF
    # closed sockets keep their slot until the next eviction: skip our own (old sessions) and dead threads
    if uid and not os.path.exists(f"/proc/self/task/{tid}") and os.path.exists(f"/proc/{tid}"):
      live_tids.add(tid)
  return len(live_tids) < MSGQ_NUM_READERS


def round_floats(obj: Any, ndigits: int) -> Any:
  if isinstance(obj, float):
    return round(obj, ndigits)
  if isinstance(obj, list):
    return [round_floats(v, ndigits) for v in obj]
  if isinstance(obj, dict):
    return {k: round_floats(v, ndigits) for k, v in obj.items()}
  return obj


class CerealOutgoingMessageProxy:
  """Forwards cereal messages to the data channels of all sessions.

  TOP: one instance per webrtcd process, shared by the sessions:
  - each service is subscribed once and kept. A client that reconnects (opview switching camera) used to
    subscribe again, which pushed carState over msgq's 15 reader slots and evicted openpilot's own
    subscribers once (communication issue)
  - each service is sent at most MAX_RATE_HZ (carState and friends are 100 Hz)
  - a channel whose send buffer is backed up gets no new messages until it drains, instead of queueing up
    seconds of lag (aiortc's SCTP is pure Python, slow on the device)
  - modelV2 floats are rounded (cm, display only): it's most of the data
  """
  MAX_RATE_HZ = 25.
  MAX_BUFFERED_BYTES = 128 * 1024
  ROUND_FLOATS = {"modelV2": 3}

  def __init__(self):
    self.socks: dict[str, messaging.SubSocket] = {}
    self.channels: dict[RTCDataChannel, set[str]] = {}
    self.next_send: dict[str, float] = {}
    self.logger = logging.getLogger("webrtcd")

  def add_services(self, services: list[str]) -> list[str]:
    """Subscribe the services not subscribed yet, returns the ones that can be streamed."""
    available = []
    for service in services:
      if service not in self.socks:
        if not msgq_reader_slot_free(service):
          # subscribing anyway would make openpilot's own processes lose messages while driving
          self.logger.warning("No free msgq reader slot, not streaming %s", service)
          continue
        self.socks[service] = messaging.sub_sock(service, conflate=True)
      available.append(service)
    return available

  def add_channel(self, channel: 'RTCDataChannel', services: list[str]):
    self.channels[channel] = set(services)

  def remove_channel(self, channel: 'RTCDataChannel'):
    self.channels.pop(channel, None)

  def to_json(self, msg_content: Any):
    if isinstance(msg_content, capnp._DynamicStructReader):
      msg_dict = msg_content.to_dict()
    elif isinstance(msg_content, capnp._DynamicListReader):
      msg_dict = [self.to_json(msg) for msg in msg_content]
    elif isinstance(msg_content, bytes):
      msg_dict = msg_content.decode()
    else:
      msg_dict = msg_content

    return msg_dict

  def update(self):
    # this is blocking in async context...
    for channel in [c for c in self.channels if c.readyState != "open"]:
      self.remove_channel(channel)
    if not self.channels:
      return

    now = time.monotonic()
    wanted = set().union(*self.channels.values())
    for service in wanted:
      # not due: leave it in the (conflated) socket, the next read gets the newest message
      if now < self.next_send.get(service, 0.) or service not in self.socks:
        continue
      msg = messaging.recv_one_or_none(self.socks[service])
      if msg is None:
        continue
      self.next_send[service] = now + 1. / self.MAX_RATE_HZ

      targets = [c for c, services in self.channels.items() if service in services and c.bufferedAmount <= self.MAX_BUFFERED_BYTES]
      if not targets:
        continue
      msg_dict = self.to_json(getattr(msg, service))
      if service in self.ROUND_FLOATS:
        msg_dict = round_floats(msg_dict, self.ROUND_FLOATS[service])
      outgoing_msg = {"type": service, "logMonoTime": msg.logMonoTime, "valid": msg.valid, "data": msg_dict}
      encoded_msg = json.dumps(outgoing_msg).encode()
      for channel in targets:
        channel.send(encoded_msg)


class CerealIncomingMessageProxy:
  def __init__(self, pm: messaging.PubMaster):
    self.pm = pm

  def send(self, message: bytes):
    msg_json = json.loads(message)
    msg_type, msg_data = msg_json["type"], msg_json["data"]
    size = None
    if not isinstance(msg_data, dict):
      size = len(msg_data)

    msg = messaging.new_message(msg_type, size=size)
    setattr(msg, msg_type, msg_data)
    self.pm.send(msg_type, msg)


class CerealProxyRunner:
  def __init__(self, proxy: CerealOutgoingMessageProxy):
    self.proxy = proxy
    self.is_running = False
    self.task = None
    self.logger = logging.getLogger("webrtcd")

  def start(self):
    assert self.task is None
    self.task = asyncio.create_task(self.run())

  def stop(self):
    if self.task is None or self.task.done():
      return
    self.task.cancel()
    self.task = None

  async def run(self):
    from aiortc.exceptions import InvalidStateError

    while True:
      try:
        self.proxy.update()
      except InvalidStateError:
        # a channel closed while sending: dropped on the next update, keep serving the others
        self.logger.warning("Cereal outgoing proxy invalid state (connection closed)")
      except Exception:
        self.logger.exception("Cereal outgoing proxy failure")
      await asyncio.sleep(0.01)


class DynamicPubMaster(messaging.PubMaster):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.lock = asyncio.Lock()

  async def add_services_if_needed(self, services):
    async with self.lock:
      for service in services:
        if service not in self.sock:
          self.sock[service] = messaging.pub_sock(service)


class StreamSession:
  shared_pub_master = DynamicPubMaster([])
  shared_outgoing_proxy: CerealOutgoingMessageProxy | None = None
  shared_outgoing_runner: CerealProxyRunner | None = None

  def __init__(self, sdp: str, cameras: list[str], incoming_services: list[str], outgoing_services: list[str], debug_mode: bool = False):
    from aiortc.mediastreams import VideoStreamTrack
    from openpilot.system.webrtc.device.video import LiveStreamVideoStreamTrack
    from teleoprtc import WebRTCAnswerBuilder
    from teleoprtc.info import parse_info_from_offer

    config = parse_info_from_offer(sdp)
    builder = WebRTCAnswerBuilder(sdp)

    assert len(cameras) == config.n_expected_camera_tracks, "Incoming stream has misconfigured number of video tracks"
    for cam in cameras:
      builder.add_video_stream(cam, LiveStreamVideoStreamTrack(cam) if not debug_mode else VideoStreamTrack())

    self.stream = builder.stream()
    self.identifier = str(uuid.uuid4())

    self.incoming_bridge: CerealIncomingMessageProxy | None = None
    self.incoming_bridge_services = incoming_services
    self.outgoing_channel: RTCDataChannel | None = None
    if len(incoming_services) > 0:
      self.incoming_bridge = CerealIncomingMessageProxy(self.shared_pub_master)
    if len(outgoing_services) > 0:
      if StreamSession.shared_outgoing_proxy is None:
        StreamSession.shared_outgoing_proxy = CerealOutgoingMessageProxy()
      outgoing_services = StreamSession.shared_outgoing_proxy.add_services(outgoing_services)
    self.outgoing_services = outgoing_services

    self.run_task: asyncio.Task | None = None
    self.logger = logging.getLogger("webrtcd")
    self.logger.info("New stream session (%s), cameras %s, incoming services %s, outgoing services %s",
                      self.identifier, cameras, incoming_services, outgoing_services)

  def start(self):
    self.run_task = asyncio.create_task(self.run())

  def stop(self):
    if self.run_task.done():
      return
    self.run_task.cancel()
    self.run_task = None
    asyncio.run(self.post_run_cleanup())

  async def get_answer(self):
    return await self.stream.start()

  async def message_handler(self, message: bytes):
    assert self.incoming_bridge is not None
    try:
      self.incoming_bridge.send(message)
    except Exception:
      self.logger.exception("Cereal incoming proxy failure")

  async def run(self):
    try:
      await self.stream.wait_for_connection()
      if self.stream.has_messaging_channel():
        if self.incoming_bridge is not None:
          await self.shared_pub_master.add_services_if_needed(self.incoming_bridge_services)
          self.stream.set_message_handler(self.message_handler)
        proxy = StreamSession.shared_outgoing_proxy
        if proxy is not None and len(self.outgoing_services) > 0:
          self.outgoing_channel = self.stream.get_messaging_channel()
          proxy.add_channel(self.outgoing_channel, self.outgoing_services)
          if StreamSession.shared_outgoing_runner is None:
            StreamSession.shared_outgoing_runner = CerealProxyRunner(proxy)
            StreamSession.shared_outgoing_runner.start()
      self.logger.info("Stream session (%s) connected", self.identifier)

      await self.stream.wait_for_disconnection()
      await self.post_run_cleanup()

      self.logger.info("Stream session (%s) ended", self.identifier)
    except Exception:
      self.logger.exception("Stream session failure")

  async def post_run_cleanup(self):
    await self.stream.stop()
    if self.outgoing_channel is not None and StreamSession.shared_outgoing_proxy is not None:
      StreamSession.shared_outgoing_proxy.remove_channel(self.outgoing_channel)


@dataclass
class StreamRequestBody:
  sdp: str
  cameras: list[str]
  bridge_services_in: list[str] = field(default_factory=list)
  bridge_services_out: list[str] = field(default_factory=list)


async def get_stream(request: 'web.Request'):
  stream_dict, debug_mode = request.app['streams'], request.app['debug']
  raw_body = await request.json()
  body = StreamRequestBody(**raw_body)

  session = StreamSession(body.sdp, body.cameras, body.bridge_services_in, body.bridge_services_out, debug_mode)
  answer = await session.get_answer()
  session.start()

  stream_dict[session.identifier] = session

  return web.json_response({"sdp": answer.sdp, "type": answer.type})


async def get_schema(request: 'web.Request'):
  services = request.query["services"].split(",")
  services = [s for s in services if s]
  assert all(s in log.Event.schema.fields and not s.endswith("DEPRECATED") for s in services), "Invalid service name"
  schema_dict = {s: generate_field(log.Event.schema.fields[s]) for s in services}
  return web.json_response(schema_dict)

async def post_notify(request: 'web.Request'):
  try:
    payload = await request.json()
  except Exception as e:
    raise web.HTTPBadRequest(text="Invalid JSON") from e

  for session in list(request.app.get('streams', {}).values()):
    try:
      ch = session.stream.get_messaging_channel()
      ch.send(json.dumps(payload))
    except Exception:
      continue

  return web.Response(status=200, text="OK")

async def on_shutdown(app: 'web.Application'):
  for session in app['streams'].values():
    session.stop()
  del app['streams']


def webrtcd_thread(host: str, port: int, debug: bool):
  logging.basicConfig(level=logging.CRITICAL, handlers=[logging.StreamHandler()])
  logging_level = logging.DEBUG if debug else logging.INFO
  logging.getLogger("WebRTCStream").setLevel(logging_level)
  logging.getLogger("webrtcd").setLevel(logging_level)

  app = web.Application()

  app['streams'] = dict()
  app['debug'] = debug
  app.on_shutdown.append(on_shutdown)
  app.router.add_post("/stream", get_stream)
  app.router.add_post("/notify", post_notify)
  app.router.add_get("/schema", get_schema)

  web.run_app(app, host=host, port=port)


def main():
  parser = argparse.ArgumentParser(description="WebRTC daemon")
  parser.add_argument("--host", type=str, default="0.0.0.0", help="Host to listen on")
  parser.add_argument("--port", type=int, default=5001, help="Port to listen on")
  parser.add_argument("--debug", action="store_true", help="Enable debug mode")
  args = parser.parse_args()

  webrtcd_thread(args.host, args.port, args.debug)


if __name__=="__main__":
  main()
