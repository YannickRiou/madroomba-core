// basestationd runs on the Raspberry Pi Zero attached to the Roomba's
// charging dock. It streams the dock camera over MJPEG and, from that image:
//   - detects motion in front of the dock (timestamp and automatic snapshot),
//   - tells whether the robot is sitting on the dock (see dock.go),
//   - lets you tune the camera, take and delete snapshots and read the
//     daemon's logs from the web page (see handlers.go).
//
// Files: main.go (wiring, camera lifecycle, analysis loop), handlers.go (HTTP
// API), settings.go (camera settings), snaps.go (snapshot storage), dock.go
// (robot presence). The web page is shared with madroombad: internal/ui.
package main

import (
	"flag"
	"io"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/camera"
	"github.com/YannickRiou/madroomba-core/internal/logbuf"
	"github.com/YannickRiou/madroomba-core/internal/ui"
)

var (
	robotURL  = flag.String("robot-url", "", "robot daemon URL relayed in the UI, e.g. http://192.168.1.95:8080 (empty = robot half hidden)")
	listen    = flag.String("listen", ":8081", "HTTP listen address")
	cameraCmd = flag.String("camera", "auto", "camera command: 'auto' or a custom command line emitting MJPEG on stdout")
	threshold = flag.Float64("motion-threshold", 5.0, "mean gray-level difference (0-255) that counts as motion")
	dataDir   = flag.String("data-dir", "/home/pi/madroomba-data", "where snapshots, camera settings and dock references are stored")
	snapKeep  = flag.Int("snap-keep", 60, "number of snapshots kept on disk")
)

const (
	snapCooldown = 15 * time.Second // min delay between automatic motion snapshots
	camWarmup    = 6 * time.Second  // motion/dock analysis paused after a camera (re)start
	analysisTick = time.Second      // one JPEG decode (over 100 ms on a Pi Zero) per tick
)

// app holds the daemon state. mu protects everything below it.
type app struct {
	start   time.Time
	custom  []string // user-supplied camera command, nil for autodetection
	logs    *logbuf.Buffer
	snaps   snapStore
	setPath string // settings.json
	refPath string // dock.json

	mu       sync.Mutex
	cam      *camera.Streamer
	camStart time.Time // when the current camera process was started
	set      camSettings
	refs     dockRefs
	grid     []float64 // latest image grid, used to learn references
	dState   string    // confirmed dock state: unknown | docked | empty
	dD, dE   float64   // distances to the "docked" / "empty" references
	dPending string    // candidate state being confirmed
	dCount   int
}

// serveCamera streams the current camera. The camera is replaced when the
// settings change; its clients are then disconnected and the page reconnects.
func (a *app) serveCamera(w http.ResponseWriter, r *http.Request) {
	a.mu.Lock()
	c := a.cam
	a.mu.Unlock()
	c.ServeHTTP(w, r)
}

// latest returns the most recent JPEG frame, or nil.
func (a *app) latest() []byte {
	a.mu.Lock()
	defer a.mu.Unlock()
	if a.cam == nil {
		return nil
	}
	return a.cam.Latest()
}

// restartCamera applies s; on failure it restores the previous camera.
func (a *app) restartCamera(s camSettings) error {
	a.mu.Lock()
	old, prevSet := a.cam, a.set
	a.mu.Unlock()
	if old != nil {
		old.Stop()
	}
	cam, err := camera.StartWithArgs(a.custom, 640, 480, 10, s.args())
	if err != nil {
		if prev, perr := camera.StartWithArgs(a.custom, 640, 480, 10, prevSet.args()); perr == nil {
			a.mu.Lock()
			a.cam = prev
			a.mu.Unlock()
		}
		return err
	}
	a.mu.Lock()
	a.cam, a.set, a.camStart = cam, s, time.Now()
	a.mu.Unlock()
	return nil
}

// analyse runs forever: motion detection (with an automatic snapshot on each
// new motion) and the dock state machine, every analysisTick.
func (a *app) analyse(det *camera.MotionDetector) {
	var lastSnap time.Time
	wasMotion := false
	for range time.Tick(analysisTick) {
		f := a.latest()
		if f == nil {
			continue
		}
		// A freshly started camera ramps its exposure: not real motion.
		a.mu.Lock()
		warm := time.Since(a.camStart) < camWarmup
		a.mu.Unlock()
		if warm {
			det.Reset()
			wasMotion = false
			continue
		}
		g, err := camera.GrayGrid(f) // one JPEG decode for both analyses
		if err != nil {
			continue
		}
		motion := det.FeedGrid(g)
		if motion && !wasMotion && time.Since(lastSnap) > snapCooldown {
			lastSnap = time.Now()
			if name, err := a.snaps.save(f, "motion"); err == nil {
				log.Printf("basestationd: motion detected, saved %s", name)
			}
		}
		wasMotion = motion
		a.observeDock(g)
	}
}

func main() {
	flag.Parse()
	logs := logbuf.New(200)
	log.SetOutput(io.MultiWriter(os.Stderr, logs))

	snapDir := filepath.Join(*dataDir, "snaps")
	if err := os.MkdirAll(snapDir, 0o755); err != nil {
		log.Fatalf("basestationd: data dir: %v", err)
	}
	a := &app{
		start:   time.Now(),
		logs:    logs,
		snaps:   snapStore{dir: snapDir, keep: *snapKeep},
		setPath: filepath.Join(*dataDir, "settings.json"),
		refPath: filepath.Join(*dataDir, "dock.json"),
		set:     defaultSettings(),
		dState:  "unknown",
	}
	if *cameraCmd != "auto" && *cameraCmd != "" {
		a.custom = strings.Fields(*cameraCmd)
	}
	loadJSON(a.setPath, &a.set)
	if a.set.valid() != nil {
		a.set = defaultSettings()
	}
	loadJSON(a.refPath, &a.refs)

	if err := a.restartCamera(a.set); err != nil {
		log.Fatalf("basestationd: %v", err)
	}
	det := camera.NewMotionDetector(*threshold)
	go a.analyse(det)

	mux := http.NewServeMux()
	self := ""
	var robot *string
	if u := strings.TrimRight(*robotURL, "/"); u != "" {
		if err := ui.Proxy(mux, "/proxy/robot", u); err != nil {
			log.Fatalf("basestationd: -robot-url: %v", err)
		}
		p := "/proxy/robot"
		robot = &p
	}
	ui.Mount(mux, robot, &self)
	a.routes(mux, det)

	log.Printf("basestationd: listening on http://%s", *listen)
	log.Fatal(http.ListenAndServe(*listen, mux))
}
