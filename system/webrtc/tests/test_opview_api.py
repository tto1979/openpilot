import asyncio
import os

import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from cereal import car
from openpilot.common.params import Params
from openpilot.top.system.opview_api import server
from openpilot.top.system.opview_api.pairing import pairing_code
from openpilot.top.system.opview_api.settings import SettingsAPI

CARS = ["TOYOTA_COROLLA_TSS2", "TOYOTA_RAV4_TSS2"]
ROUTE = "0000002a--b4c8e1f2d3"


def make_drive(root):
  for seg in range(3):
    d = os.path.join(root, f"{ROUTE}--{seg}")
    os.makedirs(d)
    for name, size in (("qcamera.ts", 100), ("fcamera.hevc", 1000), ("qlog.zst", 10)):
      with open(os.path.join(d, name), "wb") as f:
        f.write(bytes(size))
  os.makedirs(os.path.join(root, "boot"))


class TestOpviewAPI:
  @pytest.fixture(autouse=True)
  def setup_api(self, tmp_path, mocker):
    mocker.patch.object(SettingsAPI, "_cars_list", return_value=CARS)
    self.params = Params()
    self.params.put_bool("IsOnroad", False, block=True)
    self.root = str(tmp_path / "realdata")
    os.makedirs(self.root)
    make_drive(self.root)
    self.app = web.Application()
    server.setup(self.app, self.params, self.root)
    self.code = pairing_code(self.params)
    self.loop = asyncio.new_event_loop()
    self.client = TestClient(TestServer(self.app), loop=self.loop)
    self.loop.run_until_complete(self.client.start_server())
    yield
    self.loop.run_until_complete(self.client.close())
    self.loop.close()

  def request(self, method, path, auth=True, **kwargs):
    async def go():
      headers = ({server.TOKEN_HEADER: self.code} if auth else {}) | kwargs.pop("headers", {})
      resp = await self.client.request(method, path, headers=headers, **kwargs)
      body = await resp.read()
      return resp, body
    return self.loop.run_until_complete(go())

  def json(self, method, path, auth=True, **kwargs):
    resp, _ = self.request(method, path, auth, **kwargs)
    return resp.status, self.loop.run_until_complete(resp.json())

  def settings(self, **kwargs):
    status, data = self.json("GET", "/api/settings", **kwargs)
    assert status == 200
    return {s["key"]: s for section in data["settings"] for s in section["settings"]}

  def put(self, key, value):
    return self.json("POST", f"/api/settings/params/{key}", json={"value": value})

  # *** pairing ***

  def test_pairing_code(self):
    assert len(self.code) == 6 and self.code.isdigit()
    assert pairing_code(self.params) == self.code

  def test_init_is_open(self):
    status, data = self.json("GET", "/api/init", auth=False)
    assert status == 200
    assert data["isOffroad"] is True
    assert data["top"]["api"] == server.API_VERSION

  @pytest.mark.parametrize("path", ["/api/settings", "/api/files", "/api/routes", f"/media/{ROUTE}--0/qcamera.ts",
                                    f"/download/{ROUTE}--0/fcamera.hevc", "/api/settings/params/QuietDrive"])
  def test_needs_pairing_code(self, path):
    resp, _ = self.request("GET", path, auth=False)
    assert resp.status == 401
    resp, _ = self.request("GET", path, auth=False, params={"token": "000000" if self.code != "000000" else "111111"})
    assert resp.status == 401
    resp, _ = self.request("GET", path, auth=False, params={"token": self.code})
    assert resp.status == 200

  # *** settings ***

  def test_schema(self):
    s = self.settings()
    assert s["QuietDrive"]["type"] == "toggle_item" and s["QuietDrive"]["enabled"]
    assert s["CarModel"]["type"] == "select_item" and s["CarModel"]["options"][1:] == CARS
    assert s["LongitudinalPersonality"]["options"] == ["Aggressive", "Standard", "Relaxed"]
    assert s["LongitudinalPersonality"]["current_value"] == 1
    assert s["AccelPersonality"]["current_value"] == 3  # unset is stock
    assert s["SpeedLimitValueOffset"]["min_val"] == -30 and s["SpeedLimitValueOffset"]["max_val"] == 30
    assert s["dp_dev_opview"]["enabled"] is False  # would cut the app off

  def test_schema_translated(self):
    en = self.settings()
    zh = self.settings(params={"lang": "zh-CHT"})
    assert zh["dp_dev_opview"]["title"] == "啟用 opview"
    assert zh["LongitudinalPersonality"]["title"] != en["LongitudinalPersonality"]["title"]
    tw = self.settings(headers={"Accept-Language": "zh-TW,zh;q=0.9"})
    assert tw["dp_dev_opview"]["title"] == "啟用 opview"

  def test_toggle(self):
    assert self.put("QuietDrive", True) == (200, {"status": "success", "key": "QuietDrive", "value": True})
    assert self.params.get_bool("QuietDrive")
    assert self.json("GET", "/api/settings/params/QuietDrive") == (200, {"key": "QuietDrive", "value": True})
    assert self.settings()["QuietDrive"]["current_value"] is True

  def test_nnff_side_effects(self):
    assert self.put("NNFF", True)[0] == 200
    assert self.params.get_bool("OnroadCycleRequested")
    assert self.params.get_bool("DisableLateralLagLearning")
    s = self.settings()["DisableLateralLagLearning"]
    assert not s["enabled"] and s["disabled_reason"]
    assert self.put("DisableLateralLagLearning", False)[0] == 409

  def test_onroad_locks_restart_settings(self):
    self.params.put_bool("IsOnroad", True, block=True)
    status, data = self.put("NNFF", True)
    assert status == 409 and data["error"]
    assert not self.params.get_bool("NNFF")
    assert self.put("QuietDrive", True)[0] == 200  # plain toggles still work while driving

  def test_lock_param(self):
    self.params.put_bool("RecordFrontLock", True, block=True)
    assert self.put("RecordFront", True)[0] == 409

  def test_rejects(self):
    assert self.put("DongleId", "x")[0] == 403
    assert self.put("dp_dev_opview", False)[0] == 409
    assert self.put("QuietDrive", 1)[0] == 400
    assert self.put("LongitudinalPersonality", 3)[0] == 400
    assert self.put("LongitudinalPersonality", True)[0] == 400
    assert self.put("SpeedLimitValueOffset", 31)[0] == 400
    assert self.put("CarModel", "NOT_A_CAR")[0] == 400
    resp, _ = self.request("POST", "/api/settings/params/QuietDrive", data=b"nope")
    assert resp.status == 400

  def test_choices(self):
    assert self.put("LongitudinalPersonality", 0)[0] == 200
    assert self.params.get("LongitudinalPersonality") == 0
    assert self.put("SpeedLimitValueOffset", -5)[0] == 200
    assert self.params.get("SpeedLimitValueOffset") == -5
    assert self.put("CarModel", CARS[1])[0] == 200
    assert self.params.get("CarModel") == CARS[1]

  def test_experimental_mode(self):
    assert self.put("ExperimentalMode", True)[0] == 409  # car never seen
    CP = car.CarParams.new_message(openpilotLongitudinalControl=True)
    self.params.put("CarParamsPersistent", CP.to_bytes(), block=True)
    assert self.put("ExperimentalMode", True)[0] == 409  # description not confirmed on the device
    self.params.put_bool("ExperimentalModeConfirmed", True, block=True)
    assert self.put("ExperimentalMode", True)[0] == 200
    assert self.params.get_bool("ExperimentalMode")

    CP = car.CarParams.new_message(openpilotLongitudinalControl=False)
    self.params.put("CarParamsPersistent", CP.to_bytes(), block=True)
    assert not self.settings()["ExperimentalMode"]["enabled"]
    assert self.put("ExperimentalMode", False)[0] == 409

  # *** drives ***

  def test_files(self):
    status, data = self.json("GET", "/api/files", params={"path": "/"})
    assert status == 200 and data["path"] == ""
    names = [f["name"] for f in data["files"]]
    assert set(names) == {f"{ROUTE}--0", f"{ROUTE}--1", f"{ROUTE}--2", "boot"}
    assert all(f["is_dir"] for f in data["files"])
    status, data = self.json("GET", "/api/files", params={"path": f"{ROUTE}--1"})
    assert status == 200 and {f["name"]: f["size"] for f in data["files"]}["fcamera.hevc"] == 1000

  @pytest.mark.parametrize("path", ["..", "../..", "/../../etc", f"{ROUTE}--1/../../"])
  def test_files_stay_in_root(self, path):
    assert self.json("GET", "/api/files", params={"path": path})[0] == 404

  def test_routes(self):
    status, data = self.json("GET", "/api/routes")
    assert status == 200 and len(data["routes"]) == 1
    r = data["routes"][0]
    assert r["route"] == ROUTE and r["playable"]
    assert [s["index"] for s in r["segments"]] == [0, 1, 2]
    assert r["duration"] == 180 and r["size"] == 3 * 1110

  def test_route_manifest(self):
    resp, body = self.request("GET", "/api/manifest.m3u8", params={"route": ROUTE})
    assert resp.status == 200
    text = body.decode()
    assert text.count("#EXTINF") == 3 and text.count("#EXT-X-DISCONTINUITY") == 2 and text.endswith("#EXT-X-ENDLIST\n")
    seg_url = next(line for line in text.splitlines() if line.startswith("/media/"))
    resp, body = self.request("GET", seg_url, auth=False)  # players only have the URL: the token is in it
    assert resp.status == 200 and len(body) == 100
    assert self.request("GET", "/api/manifest.m3u8", params={"route": "nope"})[0].status == 404

  def test_file_manifest(self):
    resp, body = self.request("GET", "/api/manifest.m3u8", params={"file": f"{ROUTE}--0/qcamera.ts"})
    assert resp.status == 200 and f"/media/{ROUTE}--0/qcamera.ts?token={self.code}" in body.decode()
    assert self.request("GET", "/api/manifest.m3u8", params={"file": "../x"})[0].status == 400

  def test_download(self):
    resp, body = self.request("GET", f"/download/{ROUTE}--2/fcamera.hevc")
    assert resp.status == 200 and len(body) == 1000
    assert resp.headers["Content-Disposition"] == f'attachment; filename="{ROUTE}--2_fcamera.hevc"'
    assert self.request("GET", "/download/../secret")[0].status in (400, 404)

  def test_no_big_files_while_driving(self):
    self.params.put_bool("IsOnroad", True, block=True)
    assert self.request("GET", f"/download/{ROUTE}--2/qcamera.ts")[0].status == 409
    assert self.request("GET", f"/media/{ROUTE}--2/fcamera.hevc")[0].status == 409
    assert self.request("GET", f"/media/{ROUTE}--2/qcamera.ts")[0].status == 200  # watching the small video is fine
