// madroombad is the on-robot daemon: it fuses Roomba odometry and LDS-01
// lidar scans into an occupancy grid, and serves teleoperation, telemetry
// and the live map over HTTP.
//
// Run with -sim to try everything without hardware.
package main

import (
	"flag"
	"io"
	"log"
	"math"
	"os"
	"strings"
	"sync"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/camera"
	"github.com/YannickRiou/madroomba-core/internal/lidar"
	"github.com/YannickRiou/madroomba-core/internal/mapping"
	"github.com/YannickRiou/madroomba-core/internal/odom"
	"github.com/YannickRiou/madroomba-core/internal/robot"
	"github.com/YannickRiou/madroomba-core/internal/server"
	"github.com/YannickRiou/madroomba-core/internal/sim"
)

var (
	listen      = flag.String("listen", ":8080", "HTTP listen address")
	simMode     = flag.Bool("sim", false, "run against the built-in simulator (no hardware)")
	roombaPort  = flag.String("roomba-port", "/dev/serial0", "Roomba SCI serial port")
	lidarPort   = flag.String("lidar-port", "/dev/ttyUSB0", "LDS-01 serial port")
	lidarYawDeg = flag.Float64("lidar-yaw", 0, "lidar mounting yaw offset in degrees (0 = lidar 0° facing robot front)")
	lidarInvert = flag.Bool("lidar-invert", true, "invert lidar rotation direction (LDS-01 spins clockwise)")
	mapSizeM    = flag.Float64("map-size", 20, "map edge length in meters")
	cellMM      = flag.Float64("cell-mm", 50, "map cell size in millimeters")
	scanMatch   = flag.Bool("scanmatch", true, "correct odometry drift by matching scans against the map")
	brcGPIO     = flag.Int("brc-gpio", -1, "GPIO (BCM) wired to the Roomba BRC pin for wake-up, -1 to disable")
	baseURL     = flag.String("base-url", "", "dock daemon URL shown in the UI, e.g. http://madserver:8081 (empty = hidden)")
	cameraCmd   = flag.String("camera", "auto", "camera command: 'auto', 'off', or a custom command line emitting MJPEG on stdout")
)

// scanSource abstracts real lidar vs simulator.
type scanSource interface {
	ReadScan() (*lidar.Scan, error)
}

// simScanner paces simulated scans at the real sensor's ~5 Hz.
type simScanner struct{ r *sim.Robot }

func (s simScanner) ReadScan() (*lidar.Scan, error) {
	time.Sleep(200 * time.Millisecond)
	return s.r.Scan(), nil
}

func main() {
	flag.Parse()

	// Created first so the UI console also shows the startup logs.
	srv := server.New()
	srv.BaseURL = strings.TrimRight(*baseURL, "/")
	log.SetOutput(io.MultiWriter(os.Stderr, srv))

	var (
		driver  robot.Driver
		scanner scanSource
	)

	if *simMode {
		log.Println("madroombad: SIMULATION mode")
		world := sim.DefaultWorld()
		bot := sim.NewRobot(world)
		go func() {
			tick := time.NewTicker(50 * time.Millisecond)
			for range tick.C {
				bot.Step(50 * time.Millisecond)
			}
		}()
		driver = bot
		scanner = simScanner{bot}
	} else {
		var err error
		driver, err = robot.OpenReal(*roombaPort, *brcGPIO)
		if err != nil {
			log.Fatalf("madroombad: roomba init: %v", err)
		}
		// The lidar is opened by its own goroutine below, with retries.
	}
	defer driver.Close()

	cells := int(*mapSizeM * 1000 / *cellMM)
	grid := mapping.NewGrid(cells, cells, *cellMM/1000)
	tracker := &odom.Tracker{}

	srv.MapPNG = func() []byte {
		png, _ := grid.ToPNG()
		return png
	}

	// --- camera (robot's own Pi camera) ---
	if !*simMode && *cameraCmd != "off" {
		var custom []string
		if *cameraCmd != "auto" && *cameraCmd != "" {
			custom = strings.Fields(*cameraCmd)
		}
		if cam, err := camera.Start(custom, 640, 480, 15); err == nil {
			srv.Camera = cam
		} else {
			log.Printf("madroombad: %v (continuing without camera)", err)
		}
	}

	// --- shared state for telemetry ---
	var stateMu sync.Mutex
	var lastScan *lidar.Scan
	var lastBatt robot.Battery

	// --- drive, with a dead-man stop if the UI goes silent ---
	var driveMu sync.Mutex
	lastDrive := time.Now()
	moving := false
	srv.OnDrive = func(d server.DriveRequest) error {
		driveMu.Lock()
		lastDrive = time.Now()
		moving = d.Left != 0 || d.Right != 0
		driveMu.Unlock()
		return driver.SetWheels(d.Left, d.Right)
	}
	go func() {
		for range time.Tick(300 * time.Millisecond) {
			driveMu.Lock()
			stale := moving && time.Since(lastDrive) > time.Second
			if stale {
				moving = false
			}
			driveMu.Unlock()
			if stale {
				log.Println("madroombad: dead-man stop (no drive command for 1s)")
				driver.SetWheels(0, 0)
			}
		}
	}()

	srv.OnCommand = func(c server.Command) error {
		log.Printf("madroombad: command %q", c.Action)
		switch c.Action {
		case "stop":
			return driver.SetWheels(0, 0)
		case "clean":
			return driver.Clean()
		case "dock":
			return driver.Dock()
		case "safe":
			return driver.Reactivate()
		case "reset_map":
			grid.Reset()
			return nil
		case "reset_pose":
			tracker.Reset()
			return nil
		case "restart":
			// Stop the wheels, let the HTTP reply go out, then exit non-zero
			// so that systemd (Restart=on-failure) starts a fresh daemon.
			driver.SetWheels(0, 0)
			go func() {
				time.Sleep(300 * time.Millisecond)
				os.Exit(1)
			}()
			return nil
		}
		return nil
	}

	// --- odometry polling (10 Hz) ---
	go func() {
		for range time.Tick(100 * time.Millisecond) {
			d, a, err := driver.Odometry()
			if err != nil {
				log.Printf("madroombad: odometry: %v", err)
				continue
			}
			tracker.Update(d, a)
		}
	}()

	// --- battery polling (0.2 Hz) ---
	go func() {
		for {
			b, err := driver.Battery()
			if err == nil {
				stateMu.Lock()
				lastBatt = b
				stateMu.Unlock()
			}
			time.Sleep(5 * time.Second)
		}
	}()

	// --- lidar → normalization → scan matching → map ---
	go func() {
		if scanner == nil {
			scanner = openLidar(*lidarPort)
		}
		for {
			raw, err := scanner.ReadScan()
			if err != nil {
				log.Printf("madroombad: lidar: %v", err)
				time.Sleep(time.Second)
				continue
			}
			scan := normalizeScan(raw, *lidarYawDeg, *lidarInvert && !*simMode)

			pose := tracker.Pose()
			if *scanMatch {
				corrected := grid.ScanMatch(pose, scan)
				if corrected != pose {
					tracker.Set(corrected)
					pose = corrected
				}
			}
			grid.IntegrateScan(pose, scan)

			stateMu.Lock()
			lastScan = scan
			stateMu.Unlock()
		}
	}()

	// --- telemetry publishers ---
	go func() {
		for range time.Tick(200 * time.Millisecond) {
			stateMu.Lock()
			scan := lastScan
			batt := lastBatt
			stateMu.Unlock()
			p := tracker.Pose()

			ranges := make([]float64, 360)
			rpm := 0.0
			if scan != nil {
				rpm = scan.RPM
				for i, r := range scan.Ranges {
					ranges[i] = math.Round(r*1000) / 1000
				}
			}
			srv.Publish("state", map[string]interface{}{
				"pose": map[string]float64{"x": p.X, "y": p.Y, "th": p.Theta},
				"scan": ranges,
				"batt": batt,
				"rpm":  rpm,
				"sim":  *simMode,
			})
		}
	}()
	go func() {
		for range time.Tick(2 * time.Second) {
			png, meta := grid.ToPNG()
			srv.Publish("map", map[string]interface{}{
				"png":     png, // encoding/json emits []byte as base64
				"w":       meta.W,
				"h":       meta.H,
				"cell":    meta.CellM,
				"updates": meta.Updates,
			})
		}
	}()

	log.Fatal(srv.Run(*listen))
}

// openLidar keeps trying until the lidar opens and starts. At boot the daemon
// can start before udev has given /dev/ttyUSB0 to the dialout group, so a
// single attempt fails with "permission denied" and the lidar stays off.
func openLidar(port string) *lidar.Device {
	for warned := false; ; time.Sleep(5 * time.Second) {
		dev, err := lidar.Open(port)
		if err == nil {
			if err = dev.Start(); err == nil {
				log.Printf("madroombad: lidar started on %s", port)
				return dev
			}
			dev.Close()
		}
		if !warned {
			log.Printf("madroombad: lidar: %v (retrying every 5s)", err)
			warned = true
		}
	}
}

// normalizeScan converts a raw sensor-frame scan to the robot frame
// (0° = front, counter-clockwise positive).
func normalizeScan(raw *lidar.Scan, yawDeg float64, invert bool) *lidar.Scan {
	out := &lidar.Scan{RPM: raw.RPM}
	off := int(math.Round(yawDeg))
	for deg := 0; deg < 360; deg++ {
		src := deg
		if invert {
			src = (360 - deg) % 360
		}
		src = ((src+off)%360 + 360) % 360
		out.Ranges[deg] = raw.Ranges[src]
		out.Intensities[deg] = raw.Intensities[src]
	}
	return out
}
