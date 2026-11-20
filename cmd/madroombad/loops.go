package main

import (
	"errors"
	"fmt"
	"log"
	"math"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/lidar"
	"github.com/YannickRiou/madroomba-core/internal/odom"
	"github.com/YannickRiou/madroomba-core/internal/robot"
)

// startLoops starts the background loops. Each one owns one job and runs
// for the life of the daemon.
func (a *app) startLoops() {
	go a.auto.loop()
	go a.ecoLoop()
	go a.heatLoop()
	go a.sched.loop()
	go a.deadManLoop()
	go a.odometryLoop()
	go a.batteryLoop()
	go a.sensorsLoop()
	go a.lidarLoop()
	go a.stateLoop()
	go a.mapLoop()
}

// deadManLoop stops the wheels when the page stops sending drive commands
// for a second (lost connection, closed tab).
func (a *app) deadManLoop() {
	for range time.Tick(300 * time.Millisecond) {
		a.driveMu.Lock()
		stale := a.moving && time.Since(a.lastDrive) > time.Second
		if stale {
			a.moving = false
		}
		a.driveMu.Unlock()
		if stale {
			log.Println("madroombad: dead-man stop (no drive command for 1s)")
			a.driver.SetWheels(0, 0)
			stopped := false
			a.auto.activity(&stopped, nil)
		}
	}
}

// odometryLoop integrates wheel odometry into the pose at 10 Hz.
func (a *app) odometryLoop() {
	failing := false // log the first failure and the recovery, not 10 lines/s
	for range time.Tick(100 * time.Millisecond) {
		d, ang, err := a.driver.Odometry()
		if err != nil {
			if !failing {
				log.Printf("madroombad: odometry: %v (Roomba not answering)", err)
				failing = true
			}
			continue
		}
		if failing {
			log.Println("madroombad: odometry: Roomba answering again")
			failing = false
		}
		a.tracker.Move(d, ang)
	}
}

// batteryLoop reads the battery every 2 s, feeds the history and the
// lifetime counters, and handles a low battery.
func (a *app) batteryLoop() {
	low := lowBattery{}
	for {
		if b, err := a.driver.Battery(); err == nil {
			a.stateMu.Lock()
			a.lastBatt = b
			a.stateMu.Unlock()
			now := time.Now()
			a.battery.Add(b, now)
			a.lifetime.Add(b, now)
			a.checkLowBattery(&low, b)
		}
		time.Sleep(2 * time.Second)
	}
}

// lowBattery remembers what was already done for the current discharge.
type lowBattery struct{ warned, docked bool }

// idleBeforeDocking is how long nobody must have driven before the robot
// goes back to its dock by itself on a low battery.
const idleBeforeDocking = 30 * time.Second

// checkLowBattery plays the low battery melody once per discharge and
// sends the robot back to its dock when nobody is driving it. A cleaning
// job handles its own return (nav.Cleaner.BatteryLow).
func (a *app) checkLowBattery(low *lowBattery, b robot.Battery) {
	pct, ok := a.batteryPercent()
	s, haveSensors := a.auto.latest()
	onDock := haveSensors && s.ChargingSources != 0
	if !ok || onDock || pct > batteryRecoverPct {
		*low = lowBattery{} // charging or charged again: rearm
		return
	}
	if pct >= lowBatteryPct || b.CurrentMA >= 0 {
		return
	}
	if !low.warned {
		low.warned = true
		log.Printf("madroombad: battery low (%.0f %%)", pct)
		a.sounds.event("low_battery")
	}
	idle, busy := a.auto.idle()
	if low.docked || a.cleaner.Running() || busy || idle < idleBeforeDocking {
		return
	}
	low.docked = true
	log.Printf("madroombad: battery low (%.0f %%), going back to the dock", pct)
	if err := a.driver.Dock(); err != nil {
		log.Printf("madroombad: seek dock: %v", err)
	}
}

// sensorsLoop publishes the other OI sensors at 2 Hz and records the
// incidents they reveal.
func (a *app) sensorsLoop() {
	var prev robot.Sensors
	for range time.Tick(500 * time.Millisecond) {
		if s, err := a.driver.Sensors(); err == nil {
			a.auto.observe(s)
			a.recordIncidents(prev, s)
			prev = s
			a.srv.Publish("sensors", s)
		}
	}
}

// lidarLoop reads scans, corrects the pose by scan matching and updates
// the map. While the lidar is switched off it sends no frames, so the read
// simply waits.
func (a *app) lidarLoop() {
	scanner := a.scanner
	if scanner == nil {
		dev := openLidar(*lidarPort)
		a.lidar.attach(dev)
		scanner = dev
	}
	for {
		if _, real := scanner.(*lidar.Device); !real && !a.lidar.enabled() {
			time.Sleep(200 * time.Millisecond) // simulated lidar switched off
			continue
		}
		raw, err := scanner.ReadScan()
		if err != nil {
			log.Printf("madroombad: lidar: %v", err)
			// A USB disconnect leaves a dead port that returns EOF forever:
			// close it and wait for the adapter to come back.
			if dev, ok := scanner.(*lidar.Device); ok {
				dev.Close()
				dev = openLidar(*lidarPort)
				a.lidar.attach(dev)
				scanner = dev
			} else {
				time.Sleep(time.Second)
			}
			continue
		}
		if !a.lidar.enabled() {
			continue // a last frame read while the motor was stopping
		}
		scan := normalizeScan(raw, *lidarYawDeg, *lidarInvert && !*simMode)
		pose := a.tracker.Pose()
		select {
		case reply := <-a.relocReq:
			// Before anything else: a scan integrated at a wrong pose
			// damages the map.
			reply <- a.relocalize(scan, pose)
			pose = a.tracker.Pose()
		default:
		}
		if *scanMatch {
			if corrected := a.grid.ScanMatch(pose, scan); corrected != pose {
				a.tracker.Set(corrected)
				pose = corrected
			}
		}
		a.grid.IntegrateScan(pose, scan)
		a.stateMu.Lock()
		a.lastScan = scan
		a.stateMu.Unlock()
	}
}

// relocate asks the lidar loop to find the robot on the map with its next
// scan. It returns the channel of the outcome, or nil when a request is
// already waiting.
func (a *app) relocate() chan error {
	reply := make(chan error, 1)
	select {
	case a.relocReq <- reply:
		return reply
	default:
		return nil
	}
}

// relocalize searches the whole map for the pose that explains scan, taken
// at the odometry pose at, and moves the tracker there. The pose is left
// alone when the search finds nothing or cannot choose between two places.
func (a *app) relocalize(scan *lidar.Scan, at odom.Pose) error {
	loc, ok := a.grid.Relocalize(scan)
	switch {
	case !ok && loc.Second > 0:
		return fmt.Errorf("position ambiguous (two places explain %.0f %% and %.0f %% of the scan): move the robot and try again", 100*loc.Score, 100*loc.Second)
	case !ok:
		return errors.New("position not found: the scan matches no place of the map")
	}
	// The robot may have moved during the search: replay that move from
	// the pose found.
	now := a.tracker.Pose()
	rot := loc.Pose.Theta - at.Theta
	c, s := math.Cos(rot), math.Sin(rot)
	dx, dy := now.X-at.X, now.Y-at.Y
	a.tracker.Set(odom.Pose{
		X:     loc.Pose.X + c*dx - s*dy,
		Y:     loc.Pose.Y + s*dx + c*dy,
		Theta: odom.NormalizeAngle(now.Theta + rot),
	})
	log.Printf("madroombad: robot found on the map at (%.2f, %.2f) m, %.0f°, %.0f %% of the scan explained",
		loc.Pose.X, loc.Pose.Y, loc.Pose.Theta*180/math.Pi, 100*loc.Score)
	return nil
}

// mapLoaded runs after a saved map replaced the live one. The pose is at
// the map origin, right for a robot on its dock; the next scan tells
// whether it is somewhere else.
func (a *app) mapLoaded() {
	a.publishZones()
	if reply := a.relocate(); reply != nil {
		go func() {
			if err := <-reply; err != nil {
				log.Printf("madroombad: map loaded, robot taken to be at its origin: %v", err)
			}
		}()
	}
}

// stateLoop publishes the pose, the last scan and the battery at 5 Hz.
func (a *app) stateLoop() {
	for range time.Tick(200 * time.Millisecond) {
		a.stateMu.Lock()
		scan, batt := a.lastScan, a.lastBatt
		a.stateMu.Unlock()
		p := a.tracker.Pose()
		ranges := make([]float64, 360)
		rpm := 0.0
		if scan != nil {
			rpm = scan.RPM
			for i, r := range scan.Ranges {
				ranges[i] = math.Round(r*1000) / 1000
			}
		}
		a.srv.Publish("state", map[string]interface{}{
			"pose":     map[string]float64{"x": p.X, "y": p.Y, "th": p.Theta},
			"scan":     ranges,
			"batt":     batt,
			"rpm":      rpm,
			"sim":      *simMode,
			"auto":     a.auto.enabled(),
			"lidar_on": a.lidar.enabled(),
			"eco":      a.lidar.ecoEnabled(),
			"dock":     map[string]float64{"x": a.cleaner.Cfg.Dock.X, "y": a.cleaner.Cfg.Dock.Y, "r": a.cleaner.Cfg.DockRadius},
		})
	}
}

// mapLoop publishes the map image every 2 s.
func (a *app) mapLoop() {
	for range time.Tick(2 * time.Second) {
		f := a.filtered()
		png, meta := a.grid.ToPNG(f)
		a.srv.Publish("map", map[string]interface{}{
			"png":      png, // encoding/json emits []byte as base64
			"w":        meta.W,
			"h":        meta.H,
			"cell":     meta.CellM,
			"updates":  meta.Updates,
			"filtered": f,
		})
	}
}
