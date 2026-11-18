# HTTP API

Both daemons serve the web page on `/` and `GET /api/config`. Everything else
is specific to each daemon. Errors are plain text with a 4xx or 5xx status.
There is no authentication: run this on a trusted network only.

## Shared

| Endpoint | Description |
|---|---|
| `GET /` | The web page |
| `GET /api/config` | `{"robot": <prefix or null>, "base": <prefix or null>}`. A prefix is `""` for this daemon and `"/proxy/..."` for the peer. `null` means not configured |
| `* /proxy/base/...` (robot) | Relay to the dock daemon (`-base-url`). `502` if it is offline |
| `* /proxy/robot/...` (base) | Relay to the robot daemon (`-robot-url`). `502` if it is offline |

## Robot daemon (madroombad, port 8080)

| Endpoint | Description |
|---|---|
| `GET /api/events` | Server-Sent Events. Events: `state` (about 5 Hz: pose, 360-value scan, battery, lidar rpm, sim flag), `map` (about every 2 s: base64 PNG and geometry), `logs` (full log history, sent once on connect, JSON array), `log` (one new log line, JSON string). The last `state` and `map` are replayed on connect |
| `POST /api/drive` | Body `{"left": mm_s, "right": mm_s}`, each from -500 to 500. The robot stops by itself after 1 s without a command, so keep sending while moving |
| `POST /api/cmd` | Body `{"action": "stop" \| "clean" \| "dock" \| "safe" \| "reset_map" \| "reset_pose" \| "restart"}`. `restart` stops the wheels and exits the daemon with code 1, so systemd (`Restart=on-failure`) starts it again after about 5 s |
| `GET /api/map.png` | Snapshot of the occupancy grid |
| `GET /camera` | MJPEG stream (`multipart/x-mixed-replace`), `503` if there is no camera |

## Dock daemon (basestationd, port 8081)

| Endpoint | Description |
|---|---|
| `GET /camera` | MJPEG stream of the dock camera. It ends when the camera is restarted (settings change); clients should reconnect |
| `GET /api/status` | `{"motion": bool, "score": number, "uptime": string, "last_motion": "HH:MM:SS" (if any), "dock": {"state": "unknown" \| "docked" \| "empty", "has_docked": bool, "has_empty": bool, "d_docked": number, "d_empty": number}}` |
| `GET /api/logs` | Last 200 log lines, JSON array |
| `GET /api/snapshot.jpg` | Current frame. `?download=1` sends it as an attachment |
| `POST /api/snap` | Store the current frame as a manual snapshot. Returns `{"name": "..."}` |
| `GET /api/snaps` | Snapshot names, newest first, JSON array (or `null` when empty) |
| `GET /snaps/<name>` | A stored snapshot |
| `POST /api/snaps/delete?name=<name>` | Delete one snapshot. `404` if the name is not in the list |
| `POST /api/snaps/delete?all=1` | Delete every snapshot |
| `GET /api/camera/settings` | Current camera settings |
| `POST /api/camera/settings` | Validate, apply (restarts the camera, 1 to 2 s) and persist. Body below |
| `POST /api/dock/learn?state=docked\|empty` | Store the current image as the reference for that state |
| `POST /api/dock/reset` | Forget both references |

Camera settings body (all fields optional, missing ones take the defaults):

```json
{"brightness": 50, "contrast": 0, "saturation": 0,
 "awb": "auto", "exposure": "auto", "hflip": false, "vflip": false}
```

Ranges: `brightness` 0 to 100, `contrast` and `saturation` -100 to 100. `awb`:
auto, off, sun, cloud, shade, tungsten, fluorescent, incandescent, flash,
horizon. `exposure`: auto, night, nightpreview, backlight, spotlight, sports,
snow, beach, verylong, fixedfps, antishake, fireworks. They map to the
`raspivid` flags `-br -co -sa -awb -ex -hf -vf`.

Snapshots are taken automatically on each new motion (at most one every 15 s,
and not during the 6 s after a camera start, while the exposure settles).
