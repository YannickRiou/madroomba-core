# madroomba-core

Software for the MadRoomba: a Roomba 500 with a Raspberry Pi Zero on its serial
port (SCI), an LDS-01 lidar and a Pi camera, plus a second Pi Zero with a camera
at the charging dock. The goal is a "new generation" robot: live mapping,
teleoperation, telemetry, and a view of the dock.

Two daemons, written in Go with the standard library plus one serial library,
run on the two Pis. Both serve the same web page, so you open either address
and see the robot and the base side by side.

```
 Robot Pi Zero                                  Dock Pi Zero
 madroombad  :8080                              basestationd  :8081
 |- robot.Real  --SCI 115200-- Roomba 500       |- camera (raspivid / libcamera)
 |- lidar       --USB 230400-- LDS-01           |- motion detector + snapshots
 |- odom (OI packets 19/20)                     |- dock presence (2 references)
 |- mapping (log-odds grid, scan matching)      |- camera settings (persisted)
 |- camera (robot view)                         |- snapshot store
 '- web page + API  <---- /proxy/base/* ---->   '- web page + API  <---- /proxy/robot/*
                           (each relays the other)
```

## Repository layout

| Path | Role |
|---|---|
| `cmd/madroombad` | Robot daemon: odometry and lidar fusion into a map, teleoperation, camera, web page |
| `cmd/basestationd` | Dock daemon, split in `main.go` (wiring, camera lifecycle, analysis loop), `handlers.go` (HTTP API), `settings.go`, `snaps.go`, `dock.go` |
| `internal/ui` | The single web page (`index.html`, embedded), `/api/config` and the peer proxy |
| `internal/server` | Robot HTTP server: SSE telemetry, drive and command endpoints, log ring |
| `internal/camera` | MJPEG streamer around `raspivid` / `rpicam-vid` / `libcamera-vid`, motion detector, image comparison helpers |
| `internal/lidar` | LDS-01 driver (42-byte frames, six 6-byte measurements, 1 degree each) |
| `internal/odom` | Integration of OI packets 19 (distance) and 20 (angle) into a 2D pose |
| `internal/mapping` | Log-odds occupancy grid, Bresenham ray casting, hill-climbing scan matcher |
| `internal/robot` | `Driver` interface with the real implementation (roomba-go-lib) and the simulator |
| `internal/sim` | Virtual flat (walls, raycast) and a noisy differential robot, for development without hardware |
| `internal/logbuf` | In-memory ring of recent log lines, shown in the web console |
| `deploy/` | systemd units, cross-compile script, deploy script |
| `docs/` | HTTP API reference and Raspberry Pi setup |

The Roomba OI library is expected in `../roomba-go-lib` (see the `replace`
directive in `go.mod`).

## Try it without hardware

```sh
go run ./cmd/madroombad -sim
# open http://localhost:8080  (ZQSD, WASD or arrow keys to drive)
```

The simulator runs an 8 by 6 m flat with a partition and furniture. Odometry
is deliberately noisy so that the scan matcher has something to correct.

## The web page

One page, fixed to the window (no scrolling on a desktop screen). Left to
right:

- **Robot map**: live occupancy grid, lidar scan, robot pose. Zoom and
  "follow" controls. The robot camera can be switched on as a small picture in
  picture over the map.
- **Teleoperation and actions**: pad, speed slider, clean, dock, reset pose,
  reset map. The robot stops by itself after 1 s without a drive command.
- **Base**: dock camera, robot presence (docked or empty), and three tabs:
  snapshots (view, download, delete one or all), camera settings, and info.
- **Header chips**: robot connection, battery, pose, lidar, base connection,
  activity, dock state.
- **Console** (bottom bar): last log line, click to expand. Tabs for the robot
  and base logs.

If one daemon is offline, its half shows "hors ligne" and the other keeps
working. The page is in French.

### How the two halves are connected

Each daemon is started with the URL of the other one (`-base-url` for the
robot, `-robot-url` for the base). It then relays the other daemon under
`/proxy/base/*` or `/proxy/robot/*`, and `/api/config` tells the page which
prefix to use. The browser only talks to one address, so it works from any
machine that can reach one of the Pis, and no CORS setup is needed. Without
the flag, that half of the page is hidden.

## Configuration

### madroombad (robot)

| Flag | Default | Meaning |
|---|---|---|
| `-listen` | `:8080` | HTTP address |
| `-sim` | off | Use the simulator instead of hardware |
| `-roomba-port` | `/dev/serial0` | Roomba SCI serial port |
| `-lidar-port` | `/dev/ttyUSB0` | LDS-01 serial port. If it cannot be opened the daemon keeps running without lidar |
| `-lidar-yaw`, `-lidar-invert` | `0`, `true` | Lidar mounting offset (degrees) and rotation direction |
| `-map-size`, `-cell-mm` | `20`, `50` | Map edge (m) and cell size (mm) |
| `-scanmatch` | on | Correct odometry drift against the map |
| `-brc-gpio` | `-1` | GPIO (BCM) wired to the Roomba BRC pin to wake it, `-1` to disable |
| `-camera` | `auto` | `auto`, `off`, or a custom command that writes MJPEG to stdout |
| `-base-url` | empty | Dock daemon to show on the page, e.g. `http://madserver:8081` |

### basestationd (dock)

| Flag | Default | Meaning |
|---|---|---|
| `-listen` | `:8081` | HTTP address |
| `-robot-url` | empty | Robot daemon to show on the page, e.g. `http://192.168.1.95:8080` |
| `-camera` | `auto` | `auto` or a custom MJPEG command (disables the camera settings flags) |
| `-motion-threshold` | `5.0` | Mean gray-level difference (0 to 255) that counts as motion |
| `-data-dir` | `/home/pi/madroomba-data` | Storage for the files below |
| `-snap-keep` | `60` | Snapshots kept on disk, oldest are deleted first |

Files in `-data-dir`: `snaps/<YYYYMMDD-HHMMSS>-<motion|manual>.jpg`,
`settings.json` (camera settings), `dock.json` (dock references). Delete
`settings.json` or `dock.json` to go back to defaults.

The systemd units in `deploy/` carry the flags used on this robot
(`-base-url http://madserver:8081` and `-robot-url http://192.168.1.95:8080`).
Edit them if your hostnames or addresses differ.

## Deployment

```powershell
.\deploy\deploy.ps1                 # build, copy and restart both daemons
.\deploy\deploy.ps1 -Only base      # or: -Only robot
```

The script defaults to `pi@192.168.1.95` for the robot and `pi@madserver` for
the base. Override with `-Robot` and `-Base`, and pass `-RobotKey` / `-BaseKey`
(SSH private key paths) if the default SSH identity does not work. It
cross-compiles for ARMv6 (`deploy/build.ps1`), copies the binary and the
systemd unit, restarts the service and prints its status and last log lines.

Manual steps, if you prefer:

```powershell
.\deploy\build.ps1
scp dist/madroombad deploy/madroombad.service pi@<robot>:/tmp/
ssh pi@<robot> "install -m755 /tmp/madroombad /home/pi/madroombad && sudo mv /tmp/madroombad.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now madroombad"
```

Pi configuration (UART, camera) is described in
[docs/pi-setup.md](docs/pi-setup.md). HTTP endpoints are listed in
[docs/api.md](docs/api.md).

## Working on the code

```sh
go vet ./...
go test ./...
```

Tests cover the pure logic: settings validation, dock decision, snapshot
store (including path safety), image comparison, MJPEG frame splitting, log
ring, page config and proxy. Hardware code (serial, camera processes) is not
unit tested; use `-sim` for the robot side.

Where to change things:

- **A new base endpoint**: add the handler in `cmd/basestationd/handlers.go`,
  document it in `docs/api.md`. Use `postOnly` for anything that changes state.
- **A new robot endpoint**: `internal/server/server.go`, in `Run`.
- **The web page**: `internal/ui/index.html`, one file with inline CSS and
  JavaScript, no build step. It is embedded in both binaries, so rebuild and
  redeploy both daemons after a change. Colors are CSS variables at the top.
- **Camera tuning flags**: `camSettings` in `cmd/basestationd/settings.go` and
  the matching form in the page. Values are validated because they reach a
  command line.
- **Dock presence logic**: `cmd/basestationd/dock.go` (pure function
  `dockDecision`, tested).

Conventions: keep changes small, keep handlers thin, log with the daemon name
prefix (`basestationd: ...`), and add a test when you add logic that does not
need hardware.

## Design notes

- **Odometry**: OI packets 19 and 20 are polled at 10 Hz with `QueryList`.
  Reading them resets their counters, which gives clean deltas. Packet 20 is
  in whole degrees, so precision is limited, hence the scan matching.
- **Scan matching**: hill climbing on (x, y, theta) against the grid, three
  coarse-to-fine steps, correction capped at 30 cm and 12 degrees so the robot
  is never teleported. Light enough for an ARMv6 single core at 5 scans/s.
- **SSE rather than WebSocket**: standard library only, enough for one-way
  telemetry with POSTs going the other way.
- **Safe mode** by default (not Full), which keeps the cliff and wheel-drop
  protections.
- **Camera fallback**: the daemon starts each known camera command and keeps
  the first one that produces a frame within 8 s. On Pis using the legacy camera
  stack, `libcamera-vid` exists but sees no camera, so `raspivid` is used.
  Camera tuning flags only apply to `raspivid`.
- **Dock presence**: a heuristic, not detection of the robot itself. It
  compares a 32 by 24 gray image of the dock to two images you teach it
  ("docked" and "empty"), ignoring overall brightness. It needs a fixed camera
  and reasonably stable lighting, and it must be taught again if the camera
  moves.

## Wiring (robot)

- **Roomba SCI (mini-DIN 7)** to the Pi UART (`/dev/serial0`, 115200 baud). The
  SCI is 5 V TTL and the Pi is 3.3 V: a level shifter on the Pi RXD line is
  required.
- **BRC pin** of the SCI to a free GPIO (for example GPIO 17) to wake the robot
  (`-brc-gpio 17`).
- **LDS-01** to a USB serial adapter (`/dev/ttyUSB0`, 230400 baud). The Pi Zero
  mini-UART is unreliable at that speed, USB is safer.
- **Power**: the SCI supplies battery voltage (about 16 V). Use a 5 V buck
  converter rated for at least 1.5 A with lidar and camera.
- Adjust `-lidar-yaw` and `-lidar-invert` to match the lidar mounting.

## Roadmap

- [ ] ArUco marker on the robot, seen by the dock camera, to give an absolute
      pose and close the loop at the end of a mission.
- [ ] Save and reload the map between sessions.
- [ ] Autonomous exploration (frontier based) on top of the map.
- [ ] Notification when the robot is docked, from the dock presence state.
- [ ] Raw encoder packets (43 and 44) instead of 19 and 20 for finer odometry.
- [ ] Camera tuning for `libcamera-vid`.
