#!/usr/bin/env python3
"""
opview check: connect to webrtcd on this device the way the opview app does (H.264 video + data
channel) and report what arrives. Tells a problem on the device (webrtcd, its Python packages) apart
from one in the network or the phone.

Run on the device while onroad with "Enable opview" on, with the opview app closed:
  cd /data/openpilot && python3 top/system/opview_check.py
"""
import asyncio
import importlib
import json
import sys
import time
from collections import Counter

HOST = "127.0.0.1"
SECONDS = 10
# opview's services without carState: it takes msgq's last carState reader slot (see webrtcd.py)
SERVICES = ["selfdriveState", "controlsState", "modelV2", "liveCalibration", "radarState",
            "longitudinalPlan", "deviceState", "roadCameraState"]


def print_versions() -> None:
  print("== packages")
  for name in ("aiortc", "aioice", "av", "OpenSSL", "cryptography", "aiohttp", "teleoprtc"):
    try:
      mod = importlib.import_module(name)
      print(f"  {name:13s} {getattr(mod, '__version__', 'ok')}")
    except Exception as e:
      print(f"  {name:13s} IMPORT FAILED: {e!r}")


async def check() -> int:
  import aiohttp
  from aiortc import RTCPeerConnection, RTCSessionDescription, RTCRtpSender

  pc = RTCPeerConnection()
  events: list[str] = []
  t0 = time.monotonic()

  def log(event: str) -> None:
    events.append(f"  {time.monotonic() - t0:5.1f}s {event}")

  @pc.on("connectionstatechange")
  def _conn_state():
    log(f"connection: {pc.connectionState}")

  @pc.on("iceconnectionstatechange")
  def _ice_state():
    log(f"ICE: {pc.iceConnectionState}")

  data: Counter = Counter()
  channel = pc.createDataChannel("data")
  channel.on("open", lambda: log("data channel open"))

  @channel.on("message")
  def _on_message(msg):
    # webrtcd sends JSON as bytes; an exception here would close the connection
    try:
      data.update([json.loads(msg)["type"]])
    except Exception:
      data.update(["unparsable"])

  frames: list[tuple[int, int]] = []
  video_errors: list[str] = []

  @pc.on("track")
  def _on_track(track):
    log(f"incoming {track.kind} track")

    async def pull():
      while True:
        try:
          frame = await track.recv()
        except Exception as e:
          video_errors.append(repr(e))
          return
        if not frames:
          log(f"first video frame {frame.width}x{frame.height}")
        frames.append((frame.width, frame.height))
    asyncio.ensure_future(pull())

  transceiver = pc.addTransceiver("video", direction="recvonly")
  transceiver.setCodecPreferences([c for c in RTCRtpSender.getCapabilities("video").codecs if c.mimeType == "video/H264"])
  await pc.setLocalDescription(await pc.createOffer())
  # like libwebrtc / opview: direction on the data channel m-line
  offer = pc.localDescription.sdp.replace("a=sctp-port:", "a=sendrecv\r\na=sctp-port:")

  print(f"== POST http://{HOST}:5001/stream")
  try:
    async with aiohttp.ClientSession() as session:
      body = {"sdp": offer, "cameras": ["road"], "bridge_services_in": [], "bridge_services_out": SERVICES}
      async with session.post(f"http://{HOST}:5001/stream", json=body, timeout=aiohttp.ClientTimeout(total=10)) as resp:
        if resp.status != 200:
          print(f"  webrtcd returned {resp.status}:\n{await resp.text()}")
          return 1
        answer = await resp.json()
  except Exception as e:
    print(f"  webrtcd not reachable: {e!r} (onroad with 'Enable opview' on?)")
    return 1
  print("  answer ok, addresses webrtcd offers to the phone:")
  candidates = []
  for line in answer["sdp"].splitlines():
    if line.startswith("a=candidate:"):
      parts = line.split()
      candidate = f"    {parts[4]}:{parts[5]} {parts[2].lower()} ({parts[7]})"
      if candidate not in candidates:  # repeated for each bundled m-line
        candidates.append(candidate)
  print("\n".join(candidates) if candidates else "    none")
  await pc.setRemoteDescription(RTCSessionDescription(sdp=answer["sdp"], type=answer["type"]))

  await asyncio.sleep(SECONDS)
  print("== events")
  print("\n".join(events) if events else "  none")
  print(f"== result after {SECONDS} s")
  print(f"  video frames: {len(frames)}" + (f" ({frames[-1][0]}x{frames[-1][1]})" if frames else ""))
  for err in video_errors[:3]:
    print(f"  video error: {err}")
  print(f"  data messages: {sum(data.values())} {dict(data)}")
  await pc.close()
  return 0 if frames and data else 2


def main() -> int:
  print_versions()
  ret = asyncio.run(check())
  print("== verdict")
  if ret == 0:
    print("  webrtcd works on this device: the problem is between the device and the phone (network / app)")
  elif ret == 2:
    print("  webrtcd answered but video or data didn't arrive even on this device: device side problem")
  return ret


if __name__ == "__main__":
  sys.exit(main())
