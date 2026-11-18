package heatmap

import (
	"path/filepath"
	"testing"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/geom"
)

func TestVisitCountsPassesNotRest(t *testing.T) {
	m := &Map{}
	p := geom.Point{X: 1.03, Y: 0.47}
	m.Visit(p, 0.17)
	m.Visit(p, 0.17) // resting: no new pass
	m.Visit(geom.Point{X: 1.04, Y: 0.47}, 0.17)
	s := m.Snapshot()
	if len(s.Visits) < 9 {
		t.Fatalf("robot body covers %d cells, want about 9", len(s.Visits))
	}
	if s.Max != 1 {
		t.Errorf("max = %v, want 1 (one pass, then rest)", s.Max)
	}
	// drive back and forth over the same spot: passes add up
	for i := 0; i < 4; i++ {
		m.Visit(geom.Point{X: 1.03 + float64(i%2)*0.1, Y: 0.47}, 0.17)
	}
	if s := m.Snapshot(); s.Max < 3 {
		t.Errorf("max after four passes = %v", s.Max)
	}
}

func TestIncidentDebounce(t *testing.T) {
	m := &Map{}
	t0 := time.Now()
	p := geom.Point{X: 2, Y: 1}
	if !m.Incident(Bump, p, t0) {
		t.Fatal("first bump not recorded")
	}
	if m.Incident(Bump, geom.Point{X: 2.1, Y: 1}, t0.Add(2*time.Second)) {
		t.Error("the same bump, pressed for 2 s, recorded twice")
	}
	if !m.Incident(Cliff, p, t0.Add(2*time.Second)) {
		t.Error("another kind at the same place not recorded")
	}
	if !m.Incident(Bump, p, t0.Add(time.Minute)) {
		t.Error("a later bump at the same place not recorded")
	}
	if !m.Incident(Bump, geom.Point{X: 4, Y: 1}, t0.Add(61*time.Second)) {
		t.Error("a bump elsewhere not recorded")
	}
	if n := len(m.Snapshot().Incidents); n != 4 {
		t.Errorf("%d incidents, want 4", n)
	}
}

func TestSaveLoadCopy(t *testing.T) {
	dir := t.TempDir()
	m := &Map{Path: filepath.Join(dir, "heatmap.json")}
	m.Visit(geom.Point{X: 0.5, Y: 0.5}, 0.17)
	m.Incident(WheelDrop, geom.Point{X: 0.5, Y: 0.5}, time.Now())
	if err := m.SaveIfChanged(); err != nil {
		t.Fatal(err)
	}
	saved := filepath.Join(dir, "maps", "salon.heat.json")
	if err := m.SaveCopy(saved); err != nil {
		t.Fatal(err)
	}
	m.Reset()
	if len(m.Snapshot().Visits) != 0 {
		t.Fatal("reset left passes")
	}
	if err := m.LoadCopy(saved); err != nil {
		t.Fatal(err)
	}
	s := m.Snapshot()
	if len(s.Visits) == 0 || len(s.Incidents) != 1 || s.Incidents[0].Kind != WheelDrop {
		t.Fatalf("copy not restored: %+v", s)
	}
	// a map saved without heatmap empties it
	if err := m.LoadCopy(filepath.Join(dir, "maps", "none.heat.json")); err != nil || len(m.Snapshot().Visits) != 0 {
		t.Errorf("missing copy did not empty the heatmap: %v", err)
	}
}
