package main

import (
	"log"
	"net/http"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/geom"
	"github.com/YannickRiou/madroomba-core/internal/heatmap"
	"github.com/YannickRiou/madroomba-core/internal/robot"
)

// heatRobotRadius is the body the passes are counted for.
const heatRobotRadius = 0.17

// heatLoop counts the robot's passes from its pose and saves the heatmap
// once a minute after a change.
func (a *app) heatLoop() {
	tick := time.NewTicker(500 * time.Millisecond)
	defer tick.Stop()
	lastSave := time.Now()
	for range tick.C {
		p := a.tracker.Pose()
		a.heat.Visit(geom.Point{X: p.X, Y: p.Y}, heatRobotRadius)
		if time.Since(lastSave) > time.Minute {
			lastSave = time.Now()
			if err := a.heat.SaveIfChanged(); err != nil {
				log.Printf("madroombad: heatmap: %v", err)
			}
		}
	}
}

// recordIncidents compares two sensor readings and records an incident at
// the robot's position for each problem that just appeared. A cliff seen
// on the dock is the dock ramp, not an incident.
func (a *app) recordIncidents(prev, s robot.Sensors) {
	p := a.tracker.Pose()
	at := geom.Point{X: p.X, Y: p.Y}
	now := time.Now()
	anyCliff := func(s robot.Sensors) bool { return s.Cliffs[0] || s.Cliffs[1] || s.Cliffs[2] || s.Cliffs[3] }
	rise := func(was, is bool) bool { return is && !was }
	if rise(prev.BumpLeft || prev.BumpRight, s.BumpLeft || s.BumpRight) {
		a.heat.Incident(heatmap.Bump, at, now)
	}
	if rise(anyCliff(prev), anyCliff(s)) && s.ChargingSources == 0 {
		a.heat.Incident(heatmap.Cliff, at, now)
	}
	if rise(prev.WheelDropLeft || prev.WheelDropRight, s.WheelDropLeft || s.WheelDropRight) {
		a.heat.Incident(heatmap.WheelDrop, at, now)
	}
	if rise(prev.Overcurrents != 0, s.Overcurrents != 0) {
		a.heat.Incident(heatmap.Overcurrent, at, now)
	}
}

// jobEvent receives the events of autonomous jobs: it plays their melody
// and records a "stuck" job as an incident.
func (a *app) jobEvent(name string) {
	if name == "stuck" {
		p := a.tracker.Pose()
		a.heat.Incident(heatmap.Stuck, geom.Point{X: p.X, Y: p.Y}, time.Now())
	}
	a.sounds.event(name)
}

// registerHeatmap serves GET /api/heatmap (passes and incidents) and POST
// /api/heatmap/reset.
func (a *app) registerHeatmap() {
	a.srv.Handle("/api/heatmap", func(w http.ResponseWriter, r *http.Request) {
		writeJSON(w, a.heat.Snapshot())
	})
	a.srv.Handle("/api/heatmap/reset", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			http.Error(w, "POST only", http.StatusMethodNotAllowed)
			return
		}
		a.heat.Reset()
		if err := a.heat.Save(); err != nil {
			http.Error(w, err.Error(), http.StatusInternalServerError)
			return
		}
		w.WriteHeader(http.StatusNoContent)
	})
}
