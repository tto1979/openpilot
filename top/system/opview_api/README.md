# opview App API（TOP）

opview App 除了即時畫面，也能看行程、下載影片、直接改設定。這些 API 掛在 webrtcd 的 HTTP 伺服器上（port **5001**，和 `/stream` 同一個）。
路徑與 JSON 格式沿用 dragonpilot 的 dashy（port 5088），同一個 App 可以兩邊共用；TOP 多出來的部分標示「TOP」。

- 啟用：TOP 設定頁打開「啟用 opview」（`dp_dev_opview`）。打開後 webrtcd 停車時也會執行，即時串流仍只在行駛中。
- 找裝置：mDNS `_ssh._tcp`（opview 現有做法），連 `http://<ip>:5001`。

## 配對

除了 `/api/init` 和 `/stream`，其他路徑都要帶配對碼：

- Header `X-Opview-Token: <配對碼>`，或
- Query `?token=<配對碼>`（給播放器/下載器這類不能加 header 的用）

配對碼是 6 位數字，存在 `OpviewPairingCode`，第一次用時產生：

- C3/C3X：TOP 設定頁「啟用 opview」的說明文字最後一行
- C4：TOP 設定頁「opview pairing code」卡片（opview 打開時才顯示）

錯誤時回 `401 {"error": "pairing code required", "code": "unpaired"}`，App 請跳出輸入配對碼。

## 語言

設定名稱、說明、選項、錯誤原因會依語言翻譯，優先順序：

1. `?lang=zh-CHT`（裝置語言代碼：`en` `zh-CHT` `zh-CHS` `ja` `ko` `de` …）
2. `Accept-Language`（`zh-TW` / `zh-HK` / `zh-Hant` → `zh-CHT`，`zh-CN` / `zh-Hans` → `zh-CHS`）
3. 裝置目前的語言

## 端點

### `GET /api/init`（不用配對碼）

```json
{"isOffroad": true,
 "top": {"api": 1, "pairing": true, "device_type": "tici", "version": "0.11.1", "dongle_id": "...",
         "hud": {"is_metric": true, "speed_limit_mode": 1, "tim_signals": true, "personality_wheel": true}}}
```

`top.hud` 是即時畫面畫 TOP HUD 用的裝置顯示設定：`is_metric`（公制）、`speed_limit_mode`（0 關／1 資訊／2 警告／3 輔助）、`tim_signals`（方向燈動畫）、`personality_wheel`（駕駛風格圖示）。

有 `top` 欄位就是 TOP 裝置；dashy（dragonpilot）沒有這個欄位，回的是 `{"dp_dev_dashy": ..., "isOffroad": ...}`。

### `GET /api/settings`

```json
{"settings": [
  {"title": "TOP", "settings": [
    {"key": "NNFF", "type": "toggle_item", "title": "開啟 NNFF 控制器", "description": "...",
     "current_value": false, "icon": "...png", "enabled": true},
    {"key": "CarModel", "type": "select_item", "title": "車型", "current_value": "[-Not selected-]",
     "options": ["[-Not selected-]", "TOYOTA_COROLLA_TSS2", "..."], "enabled": true}
  ]},
  {"title": "切換", "settings": [
    {"key": "LongitudinalPersonality", "type": "text_spin_button_item", "title": "駕駛風格",
     "options": ["積極", "標準", "從容"], "current_value": 1, "enabled": true}
  ]},
  {"title": "速限控制", "settings": [
    {"key": "SpeedLimitValueOffset", "type": "spin_button_item", "min_val": -30, "max_val": 30, "step": 1,
     "suffix": " km/h", "current_value": 5, "enabled": true}
  ]}
]}
```

| type | 值 | 說明 |
|---|---|---|
| `toggle_item` | `true` / `false` | 開關 |
| `text_spin_button_item` | 整數，`options` 的索引 | 選項 |
| `spin_button_item` | 整數，`min_val`～`max_val` | 數值 |
| `select_item`（TOP） | 字串，`options` 其中之一 | 車型清單（項目多，請用可搜尋的清單） |

- `description` 是 HTML（`<br>`、`<b>`）。
- `enabled: false` 表示目前不能從手機改，`disabled_reason` 說明原因（已翻譯）；沒有 `disabled_reason` 是唯讀項目（例如 `dp_dev_opview`，關掉會讓 App 斷線）。
- 內容與裝置上的 TOP 設定頁、openpilot 常用開關、速限設定一致。
- 第一次讀可能要幾秒（產生車型清單）。

### `GET /api/settings/params/{key}`

`{"key": "NNFF", "value": true}`

### `POST /api/settings/params/{key}`

Body `{"value": <值>}`，成功回 `{"status": "success", "key": "NNFF", "value": true}`。
寫入規則和裝置設定頁相同（例如 NNFF 會清除學習值並要求重新上路、鎖住轉向延遲學習；實驗模式需要 openpilot 縱向控制且已在裝置上確認過說明）。

| 狀態碼 | 意思 |
|---|---|
| 400 | 值的型別或範圍不對、body 不是 JSON |
| 403 | 不在清單裡的 key |
| 409 | 目前不能改（`error` 是已翻譯的原因，例如行駛中不能改需要重啟的項目） |

改完請重新讀 `/api/settings`：有些設定會連動其他項目（例如 NNFF 與轉向延遲學習）。

### `GET /api/routes`（TOP）

依行程分組的片段清單，新的在前：

```json
{"routes": [
  {"route": "0000002a--b4c8e1f2d3", "start": "2026-11-27 08:01", "end": "2026-11-27 08:40",
   "duration": 2340, "size": 1234567890, "playable": true,
   "segments": [{"name": "0000002a--b4c8e1f2d3--0", "index": 0, "size": 61234567,
                 "files": {"qcamera.ts": 2345678, "fcamera.hevc": 55555555, "qlog.zst": 123456}}]}
]}
```

- `start`/`end` 是由檔案時間推算的本地時間；裝置沒對時前，時間可能不準。
- 片段檔案：`qcamera.ts`（小，可直接播）、`fcamera.hevc`（前鏡頭原始畫質）、`ecamera.hevc`（廣角）、`dcamera.hevc`（車內）、`qlog.zst` / `rlog.zst`（紀錄）。

### `GET /api/files?path=<路徑>`（dashy）

列出一個資料夾，資料夾在前、新的在前：

`{"path": "", "files": [{"name": "...", "is_dir": true, "mtime": "2026-11-27 08:01", "size": 0}]}`

路徑超出紀錄資料夾回 404。

### `GET /api/manifest.m3u8`

HLS 播放清單，給 `video_player` 之類的播放器：

- `?route=<route>`（TOP）：整趟行程的 `qcamera.ts` 串成一個清單
- `?file=<片段>/qcamera.ts`（dashy）：單一片段

清單裡的影片網址會自動帶上 `token`，播放器只要能開清單網址（`...m3u8?route=...&token=...`）就能播。

### `GET /media/<片段>/<檔案>`、`GET /download/<片段>/<檔案>`

- `/media`：直接回檔案（播放用）
- `/download`：同一個檔案，加上 `Content-Disposition: attachment`，檔名如 `0000002a--b4c8e1f2d3--7_fcamera.hevc`

行駛中（`isOffroad: false`）不能下載，`.hevc` / `.zst` 也不能讀，回 `409 {"code": "onroad"}`，避免和錄影搶磁碟；`qcamera.ts` 行駛中仍可播放。

### `POST /stream`

即時畫面，維持 opview 現有格式，不需要配對碼。TOP 的 HUD 另外要 `bridge_services_out` 加上 `longitudinalPlanTOP`、`liveMapDataTOP`、`topControlsState`、`carControl`（只對 TOP 裝置要求：dragonpilot 的 webrtcd 沒有這些服務會回錯誤）。
