// madroombad is the on-robot daemon: it fuses Roomba odometry and LDS-01
// lidar scans into an occupancy grid, and serves teleoperation, telemetry,
// the live map, saved maps, zones, zone cleaning, the schedule and the
// battery history over HTTP.
//
// Run with -sim to try everything without hardware.
//
// Files of this command:
//
//	main.go      flags and start-up
//	app.go       the app struct (shared state) and its construction
//	control.go   drive, commands, cleaning motors, LEDs and songs
//	loops.go     background loops: odometry, sensors, battery, lidar, telemetry
//	lidar.go     lidar port discovery and scan frame conversion
//	automode.go  smart mode (Safe to drive, Passive on the dock to charge)
//	eco.go       lidar switch and energy saving on the dock
//	maps.go      saved maps (ROS format) and their HTTP API
//	zones.go     zones and zone cleaning HTTP API
//	schedule.go  weekly schedule
//	sounds.go    melodies played on events
//	battery.go   battery history and lifetime counters HTTP API
//	heat.go      heatmap of passes and incidents
//	httputil.go  small HTTP helpers
package main

import (
	"flag"
	"log"
	"time"
)

var (
	listen      = flag.String("listen", ":8080", "HTTP listen address")
	simMode     = flag.Bool("sim", false, "run against the built-in simulator (no hardware)")
	roombaPort  = flag.String("roomba-port", "/dev/serial0", "Roomba SCI serial port")
	lidarPort   = flag.String("lidar-port", "auto", "LDS-01 serial port, or 'auto' for the first /dev/ttyUSB* or /dev/ttyACM*")
	lidarYawDeg = flag.Float64("lidar-yaw", 0, "lidar mounting yaw offset in degrees (0 = lidar 0° facing robot front)")
	lidarInvert = flag.Bool("lidar-invert", true, "invert lidar rotation direction (LDS-01 spins clockwise)")
	mapSizeM    = flag.Float64("map-size", 20, "map edge length in meters")
	cellMM      = flag.Float64("cell-mm", 50, "map cell size in millimeters")
	scanMatch   = flag.Bool("scanmatch", true, "correct odometry drift by matching scans against the map")
	brcGPIO     = flag.Int("brc-gpio", -1, "GPIO (BCM) wired to the Roomba BRC pin for wake-up, -1 to disable")
	baseURL     = flag.String("base-url", "", "dock daemon URL shown in the UI, e.g. http://madserver:8081 (empty = hidden)")
	cameraCmd   = flag.String("camera", "auto", "camera command: 'auto', 'off', or a custom command line emitting MJPEG on stdout")
	dataDir     = flag.String("data-dir", "/home/pi/madroomba-data", "where maps, zones, schedule, sounds and battery history are kept")
	wheelBaseMM = flag.Float64("wheel-base-mm", 235, "distance between the drive wheels, used by encoder odometry")
	autoModeOn  = flag.Bool("auto-mode", true, "take control (Safe) when asked to drive, go back to Passive on the dock when idle so the battery charges")
	dockAvoidM  = flag.Float64("dock-avoid-m", 0.35, "radius of the no-go disc around the dock for autonomous paths, 0 to disable")
	batteryMAh  = flag.Float64("battery-mah", 0, "rated capacity of the battery pack for the cycle count, 0 to use the Roomba's own estimate")
	ecoOnDock   = flag.Bool("eco-on-dock", true, "switch the lidar off while the robot rests on its dock, back on when it is used")
)

// Smart mode timings: idle time before giving the robot back to Passive on
// the dock, and before leaving Full mode.
const (
	idleOnDock = 20 * time.Second
	idleInFull = time.Minute
)

func main() {
	flag.Parse()
	a := newApp()
	defer a.driver.Close()
	a.routes()
	a.startLoops()
	log.Fatal(a.srv.Run(*listen))
}
