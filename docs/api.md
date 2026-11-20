# HTTP API

Both daemons serve the web page on `/` and `GET /api/config`. Everything else
is specific to each daemon. Errors are plain text with a 4xx or 5xx status.
There is no authentication: run this on a trusted network only (the local
network, or a Tailscale tailnet).

## Shared

| Endpoint | Description |
|---|---|
| `GET /` | The web page |
| `GET /api/config` | `{"robot": <prefix or null>, "base": <prefix or null>}`. A prefix is `""` for this daemon and `"/proxy/..."` for the peer. `null` means not configured |
| `* /proxy/base/...` (robot) | Relay to the dock daemon (`-base-url`). `502` if it is offline |
| `* /proxy/robot/...` (base) | Relay to the robot daemon (`-robot-url`). `502` if it is offline |

## Robot daemon (madroombad, port 80 on the robot, 8080 by default)

### Telemetry

`GET /api/events` is a Server-Sent Events stream. The last payload of each
event except `log` is replayed when a page connects.

| Event | Rate | Payload |
|---|---|---|
| `state` | 5 Hz | `pose` `{x, y, th}` (m, rad), `scan` (360 ranges in m, robot frame, 0 = no echo), `batt` (see below), `rpm` (lidar), `sim`, `auto` (smart mode on), `lidar_on`, `eco` (energy saving on the dock enabled), `dock` `{x, y, r}` (dock position and no-go radius) |
| `sensors` | 2 Hz | Every other OI sensor, see [Sensors](#sensors) |
| `map` | every 2 s | `png` (base64 grayscale image), `w`, `h` (cells), `cell` (m), `updates`, `filtered` |
| `zones` | on change | The zones, as `GET /api/zones` |
| `job` | on change | The autonomous job (zone cleaning or exploration), see [Zones, cleaning and exploration](#zones-cleaning-and-exploration) |
| `logs` | on connect | The last 200 log lines, JSON array |
| `log` | live | One new log line, JSON string |

`batt` is `{"charge_mah", "capacity_mah", "voltage_mv", "current_ma",
"temp_c", "charging"}`. The current is negative while discharging. `charging`
is OI packet 21: 0 not charging, 1 reconditioning, 2 full charging, 3
trickle, 4 waiting, 5 fault. `charge_mah` and `capacity_mah` are the Roomba's
own estimates.

### Driving and commands

| Endpoint | Description |
|---|---|
| `POST /api/drive` | Body `{"left": mm_s, "right": mm_s}`, each from -500 to 500. The robot stops by itself after 1 s without a command, so keep sending while moving. A move stops a cleaning job, takes control in smart mode, and is refused (`500`, "no-go zone ahead") toward a no-go zone or a virtual wall |
| `POST /api/cmd` | Body `{"action": "..."}`, actions below. `500` with the reason when the robot refuses or does not answer |
| `POST /api/motors` | Body `{"main_brush": bool, "vacuum": bool, "side_brush": bool}` |
| `POST /api/leds` | Body `{"debris", "spot", "dock", "check_robot": bool, "power_color": 0-255 (green to red), "power_intensity": 0-255}` |
| `POST /api/song` | Body `{"notes": [{"pitch": 31-127 (MIDI, 60 = middle C, other values are rests), "dur": 1-255 (1/64 s)}]}`, 1 to 16 notes. Stored as song 3 and played |
| `GET /camera` | MJPEG stream of the robot camera, `503` if there is no camera. The camera process only runs while someone watches: it starts with the first viewer (a few seconds) and stops 30 s after the last one leaves |

Motors, LEDs and songs need Safe or Full mode; with smart mode on, the
daemon takes control first.

| Action of `POST /api/cmd` | Effect |
|---|---|
| `stop` | Stop the wheels and any cleaning job |
| `clean`, `spot`, `max` | Start a built-in cleaning cycle (the robot goes Passive) |
| `dock` | Seek the dock (Passive) |
| `explore` | Explore the place to build or complete the map, then seek the dock (see below) |
| `safe` | Button "Commande": wake the robot through BRC if `-brc-gpio` is set, put the OI in Safe mode and read the mode back |
| `passive` | Back to Passive mode: the robot behaves and charges normally |
| `full` | Full mode: drive commands without the cliff and wheel-drop safety |
| `power_off` | Turn the Roomba off. It must then be woken by a button or BRC |
| `auto_on`, `auto_off` | Smart mode on or off |
| `lidar_on`, `lidar_off` | Start or stop the lidar motor and laser |
| `eco_on`, `eco_off` | Energy saving on the dock on or off |
| `reset_map`, `reset_pose` | Clear the map, or put the pose back at the origin |
| `relocalize` | Switch the lidar on and search the whole map for the pose that explains the next scan. An error says the position was not found or is ambiguous (two places explain the scan about as well); the pose is then left alone |
| `map_filter_on`, `map_filter_off` | Publish the filtered map (confident cells only, isolated obstacles dropped) or the raw one |
| `restart` | Stop the wheels and exit with code 1, so systemd (`Restart=on-failure`) starts the daemon again after about 5 s |

### Smart mode

On by default (`-auto-mode`). The Roomba does not charge in Safe or Full
mode, so:

- a drive, motor, LED or song request first puts the robot in Safe mode;
- on the dock, after 20 s without activity, the robot goes back to Passive
  so it charges;
- Full mode left alone for one minute goes back to Safe;
- at start-up, on the dock, the daemon leaves the robot in Passive mode.

With energy saving on (`-eco-on-dock`), the lidar stops after one minute of
rest on the dock in Passive mode, and starts again as soon as the robot is
driven, leaves the dock or starts a job. A lidar the user switched off stays
off.

On a low battery (under 15 %, discharging, off the dock) the daemon plays
the `low_battery` melody and, once nobody has driven for 30 s, sends the
robot back to its dock. A cleaning job handles its own return.

### Maps

| Endpoint | Description |
|---|---|
| `GET /api/maps` | Saved map names, JSON array |
| `POST /api/maps/save?name=<name>` | Save the filtered map as `<name>.png` + `<name>.yaml` (ROS map_server format), the current zones as `<name>.zones.json` and the heatmap as `<name>.heat.json`, in `<data-dir>/maps`. Names: 1 to 40 letters, digits, `-` or `_` |
| `POST /api/maps/load?name=<name>` | Replace the live map, the zones and the heatmap with a saved map and reset the pose to the map origin (the dock). The next lidar scan then looks for the robot on the map and moves the pose there if it is somewhere else; when the search is not sure, the pose stays at the origin. `404` if it does not exist |
| `POST /api/maps/delete?name=<name>` | Delete a saved map, its zones and its heatmap |
| `GET /maps/<name>.png`, `GET /maps/<name>.yaml` | Download a saved map file |
| `GET /api/map.png` | The live map, filtered or not as published |

### Zones, cleaning and exploration

| Endpoint | Description |
|---|---|
| `GET /api/zones` | The zones of the current map, JSON array |
| `POST /api/zones` | Replace them all (JSON array). Returns the validated list, with IDs given to new zones. `400` with the reason on an invalid zone |
| `POST /api/rooms` | Split the map into rooms at openings narrower than 1 m and add each one as a zone to clean named « Pièce N ». A room whose centre already lies in a zone to clean is skipped. Returns `{"added": n}` |
| `POST /api/clean?zone=<id>&dock=1` | Clean a zone, then seek the dock (`dock=0` to stay). `409` if a job runs or the zone is not a zone to clean |
| `POST /api/clean?zone=<id>,<id>&passes=2&dock=1` | Clean several zones in this order, each swept `passes` times (1 or 2; the second pass sweeps back). On a low battery the robot goes to its dock, charges to 80 %, then goes on where it stopped (job state `charging`) |
| `POST /api/goto?x=<m>&y=<m>` | Drive to a point of the map (job kind `goto`). `409` if a job runs |
| `POST /api/explore?dock=1` | Explore the place, then seek the dock (`dock=0` to stay). `409` if a job runs |
| `POST /api/clean/stop` | Stop the running job, cleaning or exploration |

A zone is `{"id", "name", "kind", "points": [{"x", "y"}, ...]}` in the map
frame (meters):

| Kind | Shape | Meaning |
|---|---|---|
| `clean` | simple polygon, 3 to 100 points, at least 0.04 m² | A zone the robot can clean, on demand or on schedule |
| `avoid` | simple polygon, 3 to 100 points, at least 0.04 m² | A no-go zone, for teleoperation and autonomous moves |
| `wall` | open line, 2 to 100 points, at least 0.2 m | A virtual wall the robot does not cross |

Polygons may be concave but their edges must not cross. Zones of the first
format (`x0`, `y0`, `x1`, `y1` rectangles) are still accepted and converted.

The `job` event is `{"kind" ("clean" or "explore"), "state", "zone", "name",
"progress" (0 to 1, cleaning), "visited" (frontiers reached, exploration),
"message"}`. States: `idle`, `going` (driving to the zone), `cleaning`
(sweeping, motors on), `exploring`, `docking`, `charging`, `done`, `failed`, `canceled`. A job plans
paths around mapped obstacles, no-go zones, virtual walls and the dock,
sweeps the zone in parallel lanes 25 cm apart, backs off and skips a point
on a bumper hit, and stops on a cliff or a wheel drop.

An exploration drives to the nearest frontier of the map (free space next to
unknown space, at least 30 cm long), again and again, while the lidar
completes the map. A frontier tried twice is given up; the job ends when no
reachable frontier is left, after 20 minutes, or on a low battery. On a
loaded map it only visits what is still unknown, so it updates the map.

### Heatmap

| Endpoint | Description |
|---|---|
| `GET /api/heatmap` | `{"cell": 0.1, "visits": [[x, y, passes], ...], "max", "incidents": [{"t", "kind", "x", "y"}, ...]}`. `x`, `y` in meters in the map frame (cell centers for `visits`), `t` in RFC 3339 |
| `POST /api/heatmap/reset` | Forget the passes and incidents of the current map |

A pass is counted on every 10 cm cell under the robot body (17 cm radius)
each time the robot has moved 5 cm, sampled every 500 ms. Incident kinds:
`bump`, `cliff` (not counted on the dock), `wheel_drop`, `overcurrent` (on
the rising edge of the sensor) and `stuck` (a cleaning or exploration job
gave up). The same kind within 30 cm and 10 s counts once; the 500 newest
are kept. The heatmap follows the current map: it is saved to
`<data-dir>/heatmap.json` once a minute when it changed, and saved and
loaded with the maps.

### Schedule

| Endpoint | Description |
|---|---|
| `GET /api/schedule` | `{"entries": [...], "now": "YYYY-MM-DD HH:MM", "next": {"at", "weekday", "action"}}`. `next` is missing when nothing is planned in the coming week |
| `POST /api/schedule` | Body: the full list of entries `{"days": [0-6, 0 = Sunday], "time": "HH:MM", "action", "enabled": bool}`. Actions: `clean`, `spot`, `max`, `explore`, `dock`, or `zone:<id>`. Returns the same as `GET` |

The schedule runs in the robot daemon, in the Pi's local time. At each
planned time it wakes the robot (BRC, if wired), puts it in Safe mode and
starts the action.

### Melodies

| Endpoint | Description |
|---|---|
| `GET /api/sounds` | `{"start", "done", "stuck", "low_battery"}`: the melody of each event, in the text notation below, `""` for none |
| `POST /api/sounds` | Some or all events. `400` on an invalid melody |

Notation: space-separated notes, a name with an optional `#` and an octave
(`C4` is middle C) or `R` for a rest, optionally `/duration` in 1/64 s (16 by
default). Example: `C5/12 E5/12 G5 R/8 C6/24`. Event melodies only play when
the robot is already in Safe or Full mode, so they never interrupt charging
or a built-in cleaning cycle.

### Battery

`GET /api/battery?hours=N` (24 by default, at most 168) returns:

```json
{"history": {"t": [unix seconds], "pct": [], "mv": [], "ma": [], "charging": [], "temp_c": []},
 "lifetime": {"since", "updated", "discharged_mah", "charged_mah", "equivalent_cycles", "charge_sessions", "capacity_mah"}}
```

The history holds one point per 30 s (mean current over the 30 s), averaged
down to at most 1000 points. It is kept for 7 days in
`<data-dir>/battery.csv`. The lifetime counters integrate the measured
current, are never pruned, and live in `<data-dir>/battery-lifetime.json`.
Equivalent cycles are the discharged charge divided by `-battery-mah` (or
by the Roomba's own capacity estimate when that flag is 0).

### Sensors

The `sensors` event carries, decoded for a Roomba 500:

| Field | OI packet | Meaning |
|---|---|---|
| `oi_mode` | 35 | 0 off, 1 passive, 2 safe, 3 full |
| `bump_left`, `bump_right`, `wheel_drop_left`, `wheel_drop_right` | 7 | |
| `wall`, `wall_signal` | 8, 27 | wall seen, signal 0-1023 |
| `cliffs`, `cliff_signals` | 9-12, 28-31 | left, front left, front right, right; signals 0-4095 |
| `virtual_wall` | 13 | an iRobot infrared virtual wall |
| `overcurrents` | 14 | bit 0 side brush, 2 main brush, 3 right wheel, 4 left wheel |
| `dirt_detect` | 15 | 0-255 |
| `ir_omni` | 17 | remote or dock character, 0 = none |
| `buttons` | 18 | bit 0 clean, 1 spot, 2 dock, 3 minute, 4 hour, 5 day, 6 schedule, 7 clock |
| `charging_sources` | 34 | bit 0 internal charger, bit 1 home base |
| `song_playing` | 37 | |
| `extended` | | true when the robot answers packets 43 to 58 |
| `encoder_left`, `encoder_right` | 44, 43 | raw wheel encoder counts |
| `light_bumper`, `light_signals` | 45-51 | one bit and one signal per sensor, left to right |
| `ir_left`, `ir_right` | 52, 53 | |
| `current_left_ma`, `current_right_ma`, `current_main_brush_ma`, `current_side_brush_ma` | 54-57 | motor currents |
| `stasis` | 58 | bit 0 = moving forward |

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
