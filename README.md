# madroomba-core

Software for the MadRoomba: a Roomba 500 with a Raspberry Pi Zero on its serial
port (SCI), an LDS-01 lidar and a Pi camera, plus a second Pi Zero with a camera
at the charging dock. The goal is a "new generation" robot: live mapping,
autonomous exploration, saved maps, cleaning zones, no-go zones and virtual walls, a weekly schedule,
teleoperation, full telemetry with a battery history, and a view of the dock.

Two daemons, written in Go with the standard library plus one serial library,
run on the two Pis. Both serve the same web page, so you open either address
and see the robot and the base side by side.

```
 Robot Pi Zero                                  Dock Pi Zero
 madroombad  :80 (madroomba.local)              basestationd  :8081 (madserver)
 |- robot.Real  --SCI 115200-- Roomba 500       |- camera (raspivid / libcamera)
 |- lidar       --USB 230400-- LDS-01           |- motion detector + snapshots
 |- odom (wheel encoders, OI 43/44)             |- dock presence (2 references)
 |- mapping (log-odds grid, scan matching, A*)  |- camera settings (persisted)
 |- zones, nav (cleaning, exploration), schedule|- snapshot store
 |- battery history and lifetime counters       |
 |- camera (robot view)                         |
 '- web page + API  <---- /proxy/base/* ---->   '- web page + API  <---- /proxy/robot/*
                           (each relays the other)
```

## Repository layout

| Path | Role |
|---|---|
| `cmd/madroombad` | Robot daemon. One file per job: `main.go` (flags), `app.go` (shared state), `control.go` (drive, commands, motors, LEDs, songs), `loops.go` (background loops), `lidar.go`, `automode.go` (smart mode), `eco.go` (lidar switch, energy saving), `maps.go`, `zones.go`, `schedule.go`, `sounds.go`, `battery.go`, `heat.go` (heatmap), `httputil.go` |
| `cmd/basestationd` | Dock daemon: `main.go` (wiring, camera lifecycle, analysis loop), `handlers.go` (HTTP API), `settings.go`, `snaps.go`, `dock.go` |
| `internal/robot` | `Driver` interface, the Roomba 500 implementation over the Open Interface, song notation |
| `internal/sim` | Virtual flat (walls, raycast) and a noisy differential robot, for development without hardware |
| `internal/lidar` | LDS-01 driver (42-byte frames with checksum, six measurements of 1 degree each) |
| `internal/odom` | Integration of wheel odometry into a 2D pose |
| `internal/mapping` | Log-odds occupancy grid, scan matcher, map filter, ROS map files, A* path planner, frontiers |
| `internal/geom` | Points, polygons and polylines: inside, distance, area |
| `internal/zones` | Zones to clean, no-go zones and virtual walls: validation, storage, drive checks |
| `internal/nav` | Autonomous moves: path following, coverage of a zone, zone cleaning and exploration jobs |
| `internal/battery` | Battery history (7 days) and lifetime counters (cycles, charge sessions) |
| `internal/heatmap` | Where the robot went (passes per 10 cm cell) and where it had trouble (incidents) |
| `internal/server` | Robot HTTP server: SSE telemetry, drive and command endpoints, log ring |
| `internal/ui` | The single web page (`index.html`, embedded), `/api/config` and the peer proxy |
| `internal/camera` | MJPEG streamer around `raspivid` / `rpicam-vid` / `libcamera-vid`, camera on demand, motion detector, image comparison helpers |
| `internal/logbuf` | In-memory ring of recent log lines, shown in the web console |
| `deploy/` | systemd units, cross-compile script, deploy script |
| `docs/` | HTTP API reference and Raspberry Pi setup |

The Roomba OI library is expected in `../roomba-go-lib` (see the `replace`
directive in `go.mod`).

## Try it without hardware

```sh
go run ./cmd/madroombad -sim -data-dir ./simdata
# open http://localhost:8080  (ZQSD, WASD or arrow keys to drive)
```

The simulator runs an 8 by 6 m flat with a partition and furniture. Odometry
is deliberately noisy so that the scan matcher has something to correct.
Zones, cleaning jobs and the schedule all work in simulation. The bumper is
pressed when the robot pushes against a wall, and the dock, at the start
point, is found from up to 1 m away. The battery charges on the dock and
otherwise stays at its level (the tests make it run down).

## The web page

One page, fixed to the window (no scrolling on a desktop screen). The page is
in French. Left to right:

- **Main panel**, with four views switched at the top left:
  - **Carte**: live occupancy grid (drawn as a night map), lidar scan, robot
    pose, zones, virtual walls and the dock. Zoom, "follow" and filter
    controls; save, load, download and delete maps. "passages" overlays the
    heatmap (viridis scale, from one pass to the most passed cell),
    "incidents" draws a cross where the robot bumped, met a cliff, lifted a
    wheel, overloaded a motor or got stuck, with what and when on hover.
    The teleoperation pad and
    speed sit over the bottom left corner of the map, the running job
    (cleaning or exploration, with its stop button) over the top left. The
    robot camera can be shown in the bottom right corner.
  - **Batterie**: charge and current over 6 h to 7 days, charging periods
    shaded, values on hover; lifetime counters (equivalent cycles, charge
    sessions, charge in and out, tracked since).
  - **Capteurs**: every Roomba sensor, battery details and time left, in a
    grid of readouts.
  - **Journal**: the robot and base logs, full height.
- **Control column**, in tabs:
  - **Actions**: built-in cleaning cycles, exploration, dock, the OI mode
    (Passive, Commande = Safe, Full) with smart mode and warnings, cleaning
    motors, the lidar switch and energy saving, power off, daemon restart.
  - **Zones**: draw zones to clean, no-go zones and virtual walls point by
    point on the map, start cleaning a zone.
  - **Planning**: weekly schedule (built-in cycles, exploration or zones).
  - **Son et LED**: play melodies, choose the melody of each event, set the
    LEDs.
- **Base**: dock camera, robot presence (docked or empty), and three tabs:
  snapshots, camera settings, and info.
- **Header chips**: robot connection, OI mode, battery and time left, pose,
  lidar, base connection, activity, dock state.

The pad's stop button also stops a running job, whatever the view.
If one daemon is offline, its half shows "hors ligne" and the other keeps
working.

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
| `-listen` | `:8080` | HTTP address (the unit uses `:80`) |
| `-sim` | off | Use the simulator instead of hardware |
| `-roomba-port` | `/dev/serial0` | Roomba SCI serial port |
| `-lidar-port` | `auto` | LDS-01 serial port. `auto` takes the first `/dev/ttyUSB*` or `/dev/ttyACM*`, looked up again at each attempt. The daemon retries every 5 s, and reopens the port after a USB disconnect, without stopping the rest |
| `-lidar-yaw`, `-lidar-invert` | `0`, `true` | Lidar mounting offset (degrees) and rotation direction |
| `-map-size`, `-cell-mm` | `20`, `50` | Map edge (m) and cell size (mm) |
| `-scanmatch` | on | Correct odometry drift against the map |
| `-brc-gpio` | `-1` | GPIO (BCM) wired to the Roomba BRC pin to wake it, `-1` to disable |
| `-camera` | `auto` | `auto`, `off`, or a custom command that writes MJPEG to stdout |
| `-base-url` | empty | Dock daemon to show on the page, e.g. `http://madserver:8081` |
| `-data-dir` | `/home/pi/madroomba-data` | Where the files below are kept |
| `-wheel-base-mm` | `235` | Distance between the drive wheels, for encoder odometry. Tune it if a full turn in place shows less or more than 360° |
| `-auto-mode` | on | Smart mode: take control (Safe) to drive, go back to Passive on the dock so the battery charges |
| `-dock-avoid-m` | `0.35` | Radius of the no-go disc around the dock for autonomous paths, `0` to disable |
| `-eco-on-dock` | on | Energy saving: switch the lidar off while the robot rests on its dock, back on as soon as it is used |
| `-battery-mah` | `0` | Rated capacity of the battery pack, for the equivalent cycles. `0` uses the Roomba's own estimate, which is only right after many full cycles |

Files in `-data-dir` on the robot:

| File | Content |
|---|---|
| `maps/<name>.png`, `.yaml` | Saved maps, ROS map_server format |
| `maps/<name>.zones.json` | The zones saved with that map |
| `maps/<name>.heat.json` | The heatmap saved with that map |
| `zones.json` | The zones of the current map |
| `heatmap.json` | Passes and incidents on the current map |
| `schedule.json` | The weekly schedule |
| `sounds.json` | The melody of each event |
| `battery.csv` | Battery history, one line per 30 s, last 7 days |
| `battery-lifetime.json` | Lifetime battery counters, never pruned |

### basestationd (dock)

| Flag | Default | Meaning |
|---|---|---|
| `-listen` | `:8081` | HTTP address |
| `-robot-url` | empty | Robot daemon to show on the page, e.g. `http://192.168.1.95` (an IP or a DNS name: `.local` names do not resolve, see below) |
| `-camera` | `auto` | `auto` or a custom MJPEG command (disables the camera settings flags) |
| `-motion-threshold` | `5.0` | Mean gray-level difference (0 to 255) that counts as motion |
| `-data-dir` | `/home/pi/madroomba-data` | Storage for the files below |
| `-snap-keep` | `60` | Snapshots kept on disk, oldest are deleted first |

Files in `-data-dir` on the base: `snaps/<YYYYMMDD-HHMMSS>-<motion|manual>.jpg`,
`settings.json` (camera settings), `dock.json` (dock references). Delete
`settings.json` or `dock.json` to go back to defaults.

### Addresses

The systemd units in `deploy/` carry the flags used on this robot
(`-base-url http://madserver:8081`, `-robot-url http://192.168.1.95`,
`-battery-mah 12800`). Edit them if your hostnames, addresses or battery
differ. The robot daemon listens on port 80, so the page is at
`http://madroomba.local`: the unit grants it `CAP_NET_BIND_SERVICE` instead
of running it as root. `.local` names come from avahi (mDNS), installed by
default on Raspberry Pi OS. The daemons themselves are built without cgo, so
Go cannot resolve `.local` names: give `-robot-url` an IP address (reserved
in the router) or a Tailscale name.

### Remote access with Tailscale

Install Tailscale on each Pi (`curl -fsSL https://tailscale.com/install.sh |
sh`, then `sudo tailscale up` and open the printed link to add the Pi to your
tailnet). The daemons listen on every interface, so with MagicDNS on, the
page is at `http://madroomba` from any device of the tailnet. Only the robot
needs to be reachable: it relays the base over the local network.

## Deployment

```powershell
.\deploy\deploy.ps1                 # build, copy and restart both daemons
.\deploy\deploy.ps1 -Only base      # or: -Only robot
```

The script defaults to `pi@madroomba.local` for the robot and `pi@madserver` for
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

Tests cover the logic that does not need hardware: settings validation, dock
decision, snapshot store (including path safety), image comparison, MJPEG
frame splitting, log ring, page config and proxy, lidar frames and checksum
(with a frame captured from the real sensor), map filter, ROS map round
trip, path planning, polygons and virtual walls, zone validation and drive
checks, coverage of convex and concave zones, frontiers, a full cleaning job
and a full exploration on the simulator (`go test -short` skips the 30 s
exploration), energy saving decisions, song notation, battery history and lifetime counters, smart mode
decisions, schedule validation and next run. Serial and camera processes are
not unit tested; use `-sim` for the robot side.

Where to change things:

- **A new base endpoint**: add the handler in `cmd/basestationd/handlers.go`,
  document it in `docs/api.md`. Use `postOnly` for anything that changes state.
- **A new robot endpoint**: register it in `app.routes` (`cmd/madroombad/control.go`)
  or in the file of its feature (as `maps.go`, `zones.go`, `sounds.go` do),
  and document it in `docs/api.md`.
- **A new Roomba command or sensor**: `internal/robot/real.go` (raw OI bytes,
  packet sizes in `packetLen`), the `Driver` interface in `robot.go`, and a
  stub in `internal/sim/sim.go`.
- **Navigation tuning** (speeds, lane spacing, clearance, dock disc):
  `nav.DefaultConfig` in `internal/nav/nav.go`.
- **Smart mode rules**: `autoDecision` in `cmd/madroombad/automode.go`
  (pure function, tested).
- **The web page**: `internal/ui/index.html`, one file with inline CSS and
  JavaScript, no build step. It is embedded in both binaries, so rebuild and
  redeploy both daemons after a change. Colors, radii and fonts are the CSS
  variables at the top (see [Page design](#page-design)).
- **Camera tuning flags**: `camSettings` in `cmd/basestationd/settings.go` and
  the matching form in the page. Values are validated because they reach a
  command line.
- **Dock presence logic**: `cmd/basestationd/dock.go` (pure function
  `dockDecision`, tested).

Conventions: keep changes small, keep handlers thin, keep logic in `internal/`
packages behind small interfaces so it can be tested on the simulator, log
with the daemon name prefix (`madroombad: ...`), and add a test when you add
logic that does not need hardware.

## Page design

The page has its own palette, made for a workbench at night: every surface
comes from one graphite ramp (`--bg-deep` #0f0f10, `--bg` #141415, `--card`
#19191b, `--raised` #232325), with hairlines at 8 % of the text color and
three text roles (`--text` #ededed, `--text-2` #a1a1a6, `--text-3` #85858b).
One teal accent (`--accent` #2dd4bf) marks the main action and the
selection, and nothing else. Color otherwise lives where it means something:
the map (laser red #ff5a52 for the scan, the robot in teal, zones in green,
red and amber) and the battery chart. Live figures (pose, battery, lidar,
currents) are set in tabular monospace with their unit in muted text. Rules
followed: a 4 px spacing grid, weights 400 and 500 only, sentence case,
radii of 6, 8 and 12 px, line icons drawn inline (no emoji, no icon font),
one easing curve, a visible focus ring, no glow, no gradient. They come
from the *unslop* design skill by Damir Sokolovsky
(https://github.com/heydamir/unslop, CC BY 4.0). The page loads nothing from
the Internet.

## Design notes

- **Odometry**: the raw wheel encoders (OI packets 43 right, 44 left) are
  polled at 10 Hz, with 72 mm wheels and 508.8 counts per turn (Create 2
  values: the Roomba 500 spec does not give them). On a firmware without
  packets 43 to 58, the daemon falls back to packets 19 (mm) and 20 (whole
  degrees, which loses most of a slow turn). Scan matching corrects the rest.
- **OI access**: the daemon builds its own sensor queries (opcode 149) with
  the sizes of packets 7 to 58, since the library only knows packets up to
  40. Every read has a 300 ms timeout, so a robot that is off or asleep
  never blocks the other commands.
- **Smart mode**: the Roomba does not charge in Safe or Full mode. The daemon
  takes control when asked to drive and gives it back on the dock after 20 s
  without activity; on start-up on the dock it stays Passive.
- **Map files**: the ROS map_server format (trinary PNG + YAML) can be edited
  in any image editor and read by ROS tools. A loaded map puts the robot back
  at the origin, so maps are made starting from the dock.
- **Zones**: polygons (zones to clean, no-go zones) and polylines (virtual
  walls) in the map frame, saved with each map. Drive commands that would
  bring the robot within its radius of a no-go zone or a wall within the next
  0.6 s are refused; moves that take it further away are always allowed.
- **Zone cleaning**: A* on a 10 cm copy of the grid, with obstacles inflated
  by the robot radius plus half a cell diagonal, so no straight segment
  between free cells comes too close. Paths also avoid no-go zones, walls and
  a disc around the dock (the Roomba 500 docks front first, so the dock sits
  25 cm in front of the map origin). The zone is swept in lanes 25 cm apart
  along its longer side, sampled every 5 cm and kept at a robot radius from
  its outline. A bumper hit backs off 5 cm and skips the point; a cliff or a
  wheel drop ends the job.
- **Exploration**: frontier based. The robot drives to the nearest group
  of free cells touching unknown space, while the lidar loop completes the
  map, and gives a frontier up after two tries. It stops when no reachable
  frontier is left, after 20 minutes or on a low battery, then seeks the
  dock. On a loaded map it only visits what is unknown, so it updates it.
- **Energy saving**: the Pi, the lidar and the camera run from the Roomba
  battery. The lidar (400 mA at 5 V) stops after one minute of rest on the
  dock in Passive mode, and the robot camera only runs while the page shows
  it, so more of the charger current goes into the battery. A Pi Zero has no
  suspend: the Pi itself stays on to serve the page and the schedule.
- **Battery**: the history averages the current over 30 s and appends one
  CSV line, gentle on the SD card. The lifetime counters integrate the
  measured current; equivalent cycles use the rated capacity of the pack
  (`-battery-mah`), since the Roomba only learns the real capacity over many
  full cycles.
- **Scan matching**: hill climbing on (x, y, theta) against the grid, three
  coarse-to-fine steps, correction capped at 30 cm and 12 degrees so the robot
  is never teleported. Light enough for an ARMv6 single core at 5 scans/s.
- **SSE rather than WebSocket**: standard library only, enough for one-way
  telemetry with POSTs going the other way.
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
  (`-brc-gpio 17`). Without it, the schedule only works while the Roomba is
  awake.
- **LDS-01** to a USB serial adapter (`/dev/ttyUSB0`, 230400 baud, 3.3 V
  logic). The Pi Zero mini-UART is unreliable at that speed, USB is safer. It
  draws 400 mA at 5 V, 1 A for a moment at start. Its 2-pin motor connector is
  `Vcc` and `PWM`: the lidar drives the motor speed through `PWM`, so never
  tie that pin to ground (the motor then spins at full speed, without
  control, and the lidar sends nothing). The adapter must share its ground
  with the Pi.
- **Power**: the SCI supplies battery voltage (about 16 V). Use a 5 V buck
  converter rated for at least 1.5 A with lidar and camera.
- Adjust `-lidar-yaw` and `-lidar-invert` to match the lidar mounting.

## Roadmap

- [x] Save and reload the map between sessions.
- [x] Raw encoder packets (43 and 44) instead of 19 and 20 for finer odometry.
- [x] Weekly schedule run by the robot daemon.
- [x] No-go zones and virtual walls drawn on the map, enforced on every move.
- [x] Cleaning zones: path planning, coverage, brushes and vacuum on.
- [x] Battery history and lifetime counters.
- [ ] ArUco marker on the robot, seen by the dock camera, to give an absolute
      pose and close the loop at the end of a mission.
- [ ] Docking by our own planner instead of the Roomba's seek-dock behavior.
- [x] Autonomous exploration (frontier based) to build or update the map.
- [ ] Notification when the robot is docked, from the dock presence state.
- [ ] Camera tuning for `libcamera-vid`.
