package main

import (
	"io"
	"log"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/battery"
	"github.com/YannickRiou/madroomba-core/internal/camera"
	"github.com/YannickRiou/madroomba-core/internal/heatmap"
	"github.com/YannickRiou/madroomba-core/internal/lidar"
	"github.com/YannickRiou/madroomba-core/internal/mapping"
	"github.com/YannickRiou/madroomba-core/internal/nav"
	"github.com/YannickRiou/madroomba-core/internal/odom"
	"github.com/YannickRiou/madroomba-core/internal/robot"
	"github.com/YannickRiou/madroomba-core/internal/server"
	"github.com/YannickRiou/madroomba-core/internal/sim"
	"github.com/YannickRiou/madroomba-core/internal/zones"
)

// app holds everything the daemon shares between its HTTP handlers and
// background loops.
type app struct {
	srv      *server.Server
	driver   robot.Driver
	scanner  scanSource // simulator scans; nil on the robot until the lidar opens
	grid     *mapping.Grid
	tracker  *odom.Tracker
	auto     *autoMode
	lidar    *lidarSwitch
	heat     *heatmap.Map // passes and incidents of the current map
	zones    *zones.Store
	cleaner  *nav.Cleaner
	sounds   *soundStore
	battery  *battery.Recorder // last days, for the chart
	lifetime *battery.Lifetime // counters kept for the life of the battery
	maps     *mapStore
	sched    *scheduler

	mapFilter int32 // 1 = publish the filtered map (atomic)

	// relocReq holds at most one request to find the robot on the map; the
	// lidar loop serves it with its next scan and answers on the channel.
	relocReq chan chan error

	stateMu  sync.Mutex // guards the latest readings below
	lastScan *lidar.Scan
	lastBatt robot.Battery

	driveMu   sync.Mutex // guards the teleoperation state below
	lastDrive time.Time
	moving    bool
}

// Battery levels for warnings and for ending a cleaning job.
const (
	lowBatteryPct     = 15.0
	batteryRecoverPct = 20.0
	// resumeBatteryPct is the charge at which a cleaning job that went to
	// the dock on a low battery goes on.
	resumeBatteryPct = 80.0
)

// newApp builds the app from the flags: driver (simulator or Roomba),
// map, stores in -data-dir, camera and cleaning job runner.
func newApp() *app {
	a := &app{srv: server.New(), tracker: &odom.Tracker{}, mapFilter: 1, lastDrive: time.Now(), relocReq: make(chan chan error, 1)}
	a.srv.BaseURL = strings.TrimRight(*baseURL, "/")
	// Set up first so the page console also shows the start-up logs.
	log.SetOutput(io.MultiWriter(os.Stderr, a.srv))

	if *simMode {
		log.Println("madroombad: SIMULATION mode")
		bot := sim.NewRobot(sim.DefaultWorld())
		go func() {
			for range time.Tick(50 * time.Millisecond) {
				bot.Step(50 * time.Millisecond)
			}
		}()
		a.driver, a.scanner = bot, simScanner{bot}
	} else {
		d, err := robot.OpenReal(*roombaPort, *brcGPIO, *wheelBaseMM/1000)
		if err != nil {
			log.Fatalf("madroombad: roomba init: %v", err)
		}
		a.driver = d // the lidar is opened by its loop, with retries
	}

	cells := int(*mapSizeM * 1000 / *cellMM)
	a.grid = mapping.NewGrid(cells, cells, *cellMM/1000)

	a.zones = &zones.Store{Path: filepath.Join(*dataDir, "zones.json")}
	if err := a.zones.Load(); err != nil {
		log.Printf("madroombad: zones: %v", err)
	}
	a.heat = &heatmap.Map{Path: filepath.Join(*dataDir, "heatmap.json")}
	if err := a.heat.Load(); err != nil {
		log.Printf("madroombad: heatmap: %v", err)
	}
	a.maps = &mapStore{dir: filepath.Join(*dataDir, "maps"), grid: a.grid, tracker: a.tracker, zones: a.zones, heat: a.heat, loaded: a.mapLoaded}
	a.publishZones()

	a.battery = &battery.Recorder{Path: filepath.Join(*dataDir, "battery.csv"), Interval: 30 * time.Second, Keep: 7 * 24 * time.Hour}
	if err := a.battery.Load(); err != nil {
		log.Printf("madroombad: battery history: %v", err)
	}
	a.lifetime = &battery.Lifetime{Path: filepath.Join(*dataDir, "battery-lifetime.json"), NominalMAh: *batteryMAh}
	if err := a.lifetime.Load(); err != nil {
		log.Printf("madroombad: battery lifetime counters: %v", err)
	}

	// Nobody used the robot before the daemon started: count it as idle
	// already, so that on the dock the energy saving switches the lidar off
	// at once instead of a minute after each start.
	a.auto = &autoMode{driver: a.driver, last: time.Now().Add(-ecoIdle)}
	a.auto.set(*autoModeOn)
	a.lidar = &lidarSwitch{on: true}
	a.lidar.setEco(*ecoOnDock)

	a.sounds = &soundStore{path: filepath.Join(*dataDir, "sounds.json"), play: a.driver.PlaySong, canPlay: a.auto.inControl}
	a.sounds.load()

	cfg := nav.DefaultConfig()
	cfg.DockRadius = *dockAvoidM
	a.cleaner = &nav.Cleaner{
		Base: a.driver, Pose: a.tracker, Planner: a.grid, Frontiers: a.grid, Cfg: cfg,
		Blocked: func(p mapping.Point) bool { return a.zones.Blocked(p, cfg.RobotRadius+cfg.Clearance) },
		Control: a.control,
		FullMode: func(on bool) error {
			if on {
				return a.driver.Action("full")
			}
			return a.driver.Reactivate() // Safe
		},
		BatteryLow: a.batteryLow,
		Charged: func() bool {
			pct, ok := a.batteryPercent()
			return ok && pct >= resumeBatteryPct
		},
		Event:      a.jobEvent,
		Publish:    func(s nav.Status) { a.srv.Publish("job", s) },
	}
	a.auto.jobRunning = a.cleaner.Running
	a.srv.Publish("job", a.cleaner.Status())

	a.sched = &scheduler{path: filepath.Join(*dataDir, "schedule.json"), run: a.runScheduled}
	a.sched.load()

	if !*simMode && *cameraCmd != "off" {
		var custom []string
		if *cameraCmd != "auto" && *cameraCmd != "" {
			custom = strings.Fields(*cameraCmd)
		}
		// The robot camera only runs while the page shows it.
		a.srv.Camera = &camera.OnDemand{Command: custom, Width: 640, Height: 480, FPS: 15, IdleStop: 30 * time.Second}
	}
	return a
}

// filtered tells whether the published map is the filtered view.
func (a *app) filtered() bool { return atomic.LoadInt32(&a.mapFilter) == 1 }

// control puts the robot in Safe mode for an autonomous move, whatever the
// smart mode setting.
func (a *app) control() error {
	if a.auto.enabled() {
		return a.auto.takeControl()
	}
	return a.driver.Reactivate()
}

// batteryPercent returns the last state of charge and whether a dock or
// charger powers the robot.
func (a *app) batteryPercent() (pct float64, known bool) {
	a.stateMu.Lock()
	b := a.lastBatt
	a.stateMu.Unlock()
	if b.CapacityMAh == 0 {
		return 0, false
	}
	return 100 * float64(b.ChargeMAh) / float64(b.CapacityMAh), true
}

// batteryLow tells a cleaning job to stop and go back to the dock.
func (a *app) batteryLow() bool {
	pct, ok := a.batteryPercent()
	return ok && pct < lowBatteryPct
}

// runScheduled starts a planned action: a built-in cleaning cycle, the
// dock, or a zone ("zone:<id>").
func (a *app) runScheduled(action string) {
	log.Printf("madroombad: schedule: starting %q", action)
	if id := strings.TrimPrefix(action, "zone:"); id != action {
		if err := a.startZone(id, true); err != nil {
			log.Printf("madroombad: schedule: %v", err)
		}
		return
	}
	if action == "explore" {
		a.setLidar(true, false)
		if err := a.cleaner.Explore(true); err != nil {
			log.Printf("madroombad: schedule: %v", err)
		}
		return
	}
	if err := a.driver.Reactivate(); err != nil {
		log.Printf("madroombad: schedule: %v", err)
	}
	if err := a.command(action); err != nil {
		log.Printf("madroombad: schedule: %s: %v", action, err)
	}
}
