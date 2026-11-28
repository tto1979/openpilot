"""
Drives (routes) for the opview app: listing, HLS playback of the qcamera and downloads, from the log root.
/api/files and /api/manifest.m3u8?file= are dashy's (dragonpilot), /api/routes and ?route= are TOP's.

A segment folder is <route>--<segment number> (e.g. 0000002a--b4c8e1f2d3--7), its files: qcamera.ts (small, HLS
playable), fcamera.hevc / ecamera.hevc / dcamera.hevc (full quality), qlog.zst / rlog.zst (logs).
"""
import os
import re
from datetime import datetime
from urllib.parse import quote

from openpilot.system.hardware.hw import Paths

SEGMENT_RE = re.compile(r"^(?P<route>.+)--(?P<seg>\d+)$")
SEGMENT_SECONDS = 60
PLAYABLE = "qcamera.ts"


def log_root() -> str:
  return os.path.realpath(Paths.log_root())


def safe_path(requested: str, root: str | None = None) -> str | None:
  """Absolute path of requested inside the log root, None if it escapes it (.., symlinks)"""
  root = root or log_root()
  path = os.path.realpath(os.path.join(root, requested.lstrip("/")))
  return path if os.path.commonpath((path, root)) == root else None


def list_files(path: str = "/", root: str | None = None) -> dict | None:
  """dashy /api/files: one directory, folders first then files, newest first"""
  root = root or log_root()
  directory = safe_path(path, root)
  if directory is None or not os.path.isdir(directory):
    return None
  items = []
  for entry in os.listdir(directory):
    full = os.path.join(directory, entry)
    if os.path.commonpath((os.path.realpath(full), root)) != root:
      continue
    try:
      st = os.stat(full)
    except FileNotFoundError:
      continue
    is_dir = os.path.isdir(full)
    items.append({"name": entry, "is_dir": is_dir, "mtime": datetime.fromtimestamp(st.st_mtime).strftime("%Y-%m-%d %H:%M"),
                  "size": 0 if is_dir else st.st_size})
  dirs = sorted((i for i in items if i["is_dir"]), key=lambda i: i["mtime"], reverse=True)
  files = sorted((i for i in items if not i["is_dir"]), key=lambda i: i["mtime"], reverse=True)
  rel = os.path.relpath(directory, root)
  return {"path": "" if rel == "." else rel, "files": dirs + files}


def list_routes(root: str | None = None) -> list[dict]:
  """TOP /api/routes: segments grouped into drives, newest drive first"""
  root = root or log_root()
  routes: dict[str, dict] = {}
  try:
    entries = os.listdir(root)
  except FileNotFoundError:
    return []
  for entry in entries:
    m = SEGMENT_RE.match(entry)
    full = os.path.join(root, entry)
    if m is None or not os.path.isdir(full) or os.path.islink(full):
      continue
    try:
      files = {f: os.path.getsize(os.path.join(full, f)) for f in os.listdir(full) if os.path.isfile(os.path.join(full, f))}
      mtime = os.path.getmtime(full)
    except FileNotFoundError:
      continue
    r = routes.setdefault(m["route"], {"route": m["route"], "segments": [], "size": 0, "start": mtime, "end": mtime})
    r["segments"].append({"name": entry, "index": int(m["seg"]), "files": files, "size": sum(files.values())})
    r["size"] += sum(files.values())
    r["start"], r["end"] = min(r["start"], mtime - SEGMENT_SECONDS), max(r["end"], mtime)

  out = []
  for r in routes.values():
    r["segments"].sort(key=lambda s: s["index"])
    r["duration"] = len(r["segments"]) * SEGMENT_SECONDS
    r["playable"] = any(PLAYABLE in s["files"] for s in r["segments"])
    r["start"] = datetime.fromtimestamp(r["start"]).strftime("%Y-%m-%d %H:%M")
    r["end"] = datetime.fromtimestamp(r["end"]).strftime("%Y-%m-%d %H:%M")
    out.append(r)
  return sorted(out, key=lambda r: r["end"], reverse=True)


def _media_url(rel: str, token: str | None) -> str:
  url = "/media/" + quote(rel)
  return url + ("?token=" + quote(token) if token else "")


def file_manifest(file: str, token: str | None = None) -> str:
  """dashy: one qcamera.ts as a VOD playlist"""
  lines = ["#EXTM3U", "#EXT-X-VERSION:3", f"#EXT-X-TARGETDURATION:{SEGMENT_SECONDS}", "#EXT-X-PLAYLIST-TYPE:VOD",
           f"#EXTINF:{SEGMENT_SECONDS}.0,", _media_url(file.lstrip("/"), token), "#EXT-X-ENDLIST"]
  return "\n".join(lines) + "\n"


def route_manifest(route: str, root: str | None = None, token: str | None = None) -> str | None:
  """TOP: the whole drive as one VOD playlist, a discontinuity between segments (each qcamera.ts starts over)"""
  r = next((r for r in list_routes(root) if r["route"] == route), None)
  if r is None:
    return None
  lines = ["#EXTM3U", "#EXT-X-VERSION:3", f"#EXT-X-TARGETDURATION:{SEGMENT_SECONDS}", "#EXT-X-PLAYLIST-TYPE:VOD"]
  first = True
  for s in r["segments"]:
    if PLAYABLE not in s["files"]:
      continue
    if not first:
      lines.append("#EXT-X-DISCONTINUITY")
    first = False
    lines += [f"#EXTINF:{SEGMENT_SECONDS}.0,", _media_url(f"{s['name']}/{PLAYABLE}", token)]
  lines.append("#EXT-X-ENDLIST")
  return "\n".join(lines) + "\n"
