// Package heatmap records where the robot went and where it ran into
// trouble, in the map frame: a count of passes per 10 cm cell, and a list
// of incidents (bumper hits, cliffs, wheel drops, overcurrents, jobs that
// got stuck) with their position and time. The page draws both over the
// map.
//
// A pass is counted when the robot has moved at least MinMove since the
// last count, for every cell under its body, so a robot resting on its
// dock does not heat up. Both the passes and the incidents belong to one
// map: they are saved next to it and loaded with it, like the zones.
package heatmap

import (
	"encoding/json"
	"math"
	"os"
	"path/filepath"
	"sort"
	"sync"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/geom"
)

// Tunables.
const (
	CellM        = 0.10 // heat cell edge
	MinMove      = 0.05 // m moved before the next pass is counted
	MaxIncidents = 500  // oldest are dropped first
	// An incident of the same kind closer than this, in space and time, to
	// the previous one is the same incident (a bumper pressed for a while).
	SameIncidentM = 0.3
	SameIncidentT = 10 * time.Second
)

// Incident kinds.
const (
	Bump        = "bump"
	Cliff       = "cliff"
	WheelDrop   = "wheel_drop"
	Overcurrent = "overcurrent"
	Stuck       = "stuck"
)

// Incident is one problem at one place.
type Incident struct {
	Time time.Time `json:"t"`
	Kind string    `json:"kind"`
	X    float64   `json:"x"`
	Y    float64   `json:"y"`
}

// Snapshot is the heatmap as sent to the page and saved on disk.
type Snapshot struct {
	CellM     float64      `json:"cell"`
	Visits    [][3]float64 `json:"visits"` // x, y (cell center, m) and pass count
	Max       float64      `json:"max"`
	Incidents []Incident   `json:"incidents"`
}

// Map holds the passes and incidents. It is safe for concurrent use.
type Map struct {
	Path string // JSON file of the current heatmap

	mu        sync.Mutex
	visits    map[[2]int]int
	incidents []Incident
	last      geom.Point
	hasLast   bool
	dirty     bool
}

func (m *Map) init() {
	if m.visits == nil {
		m.visits = map[[2]int]int{}
	}
}

func cellOf(x, y float64) [2]int {
	return [2]int{int(math.Floor(x / CellM)), int(math.Floor(y / CellM))}
}

// Visit records the robot (a disc of the given radius) at p. It counts a
// pass for every cell under the robot once the robot has moved MinMove
// since the last count.
func (m *Map) Visit(p geom.Point, radius float64) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.init()
	if m.hasLast && geom.Dist(p, m.last) < MinMove {
		return
	}
	m.last, m.hasLast = p, true
	c := cellOf(p.X, p.Y)
	r := int(math.Ceil(radius / CellM))
	for dy := -r; dy <= r; dy++ {
		for dx := -r; dx <= r; dx++ {
			k := [2]int{c[0] + dx, c[1] + dy}
			center := geom.Point{X: (float64(k[0]) + 0.5) * CellM, Y: (float64(k[1]) + 0.5) * CellM}
			if geom.Dist(center, p) <= radius {
				m.visits[k]++
			}
		}
	}
	m.dirty = true
}

// Incident records a problem of the given kind at p, unless it repeats the
// previous incident of that kind (same place, a few seconds later). It
// reports whether the incident was recorded.
func (m *Map) Incident(kind string, p geom.Point, t time.Time) bool {
	m.mu.Lock()
	defer m.mu.Unlock()
	for i := len(m.incidents) - 1; i >= 0; i-- {
		prev := m.incidents[i]
		if prev.Kind != kind {
			continue
		}
		if t.Sub(prev.Time) < SameIncidentT && geom.Dist(p, geom.Point{X: prev.X, Y: prev.Y}) < SameIncidentM {
			return false
		}
		break
	}
	m.incidents = append(m.incidents, Incident{Time: t, Kind: kind, X: p.X, Y: p.Y})
	if len(m.incidents) > MaxIncidents {
		m.incidents = m.incidents[len(m.incidents)-MaxIncidents:]
	}
	m.dirty = true
	return true
}

// Snapshot returns a copy of the heatmap, cells sorted for stable output.
func (m *Map) Snapshot() Snapshot {
	m.mu.Lock()
	defer m.mu.Unlock()
	s := Snapshot{CellM: CellM, Visits: [][3]float64{}, Incidents: append([]Incident{}, m.incidents...)}
	keys := make([][2]int, 0, len(m.visits))
	for k := range m.visits {
		keys = append(keys, k)
	}
	sort.Slice(keys, func(i, j int) bool {
		if keys[i][1] != keys[j][1] {
			return keys[i][1] < keys[j][1]
		}
		return keys[i][0] < keys[j][0]
	})
	for _, k := range keys {
		n := float64(m.visits[k])
		s.Visits = append(s.Visits, [3]float64{(float64(k[0]) + 0.5) * CellM, (float64(k[1]) + 0.5) * CellM, n})
		s.Max = math.Max(s.Max, n)
	}
	return s
}

// Reset forgets every pass and incident.
func (m *Map) Reset() {
	m.mu.Lock()
	m.visits, m.incidents, m.hasLast, m.dirty = map[[2]int]int{}, nil, false, true
	m.mu.Unlock()
}

// set replaces the content with a snapshot.
func (m *Map) set(s Snapshot) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.visits = map[[2]int]int{}
	for _, v := range s.Visits {
		m.visits[cellOf(v[0], v[1])] = int(v[2])
	}
	m.incidents = s.Incidents
	m.hasLast = false
}

// Load reads Path; a missing file is an empty heatmap.
func (m *Map) Load() error { return m.LoadCopy(m.Path) }

// LoadCopy replaces the heatmap with the file at path (saved with a map)
// and saves it as the current one. A missing file empties it.
func (m *Map) LoadCopy(path string) error {
	data, err := os.ReadFile(path)
	if os.IsNotExist(err) {
		m.Reset()
		return nil
	}
	if err != nil {
		return err
	}
	var s Snapshot
	if err := json.Unmarshal(data, &s); err != nil {
		return err
	}
	m.set(s)
	if path != m.Path {
		return m.Save()
	}
	return nil
}

// Save writes the heatmap to Path.
func (m *Map) Save() error { return m.SaveCopy(m.Path) }

// SaveIfChanged saves only after a change, for a periodic call.
func (m *Map) SaveIfChanged() error {
	m.mu.Lock()
	dirty := m.dirty
	m.dirty = false
	m.mu.Unlock()
	if !dirty {
		return nil
	}
	return m.Save()
}

// SaveCopy writes the heatmap to path (next to a saved map), atomically.
func (m *Map) SaveCopy(path string) error {
	if path == "" {
		return nil
	}
	data, _ := json.Marshal(m.Snapshot())
	if err := os.MkdirAll(filepath.Dir(path), 0755); err != nil {
		return err
	}
	tmp := path + ".tmp"
	if err := os.WriteFile(tmp, data, 0644); err != nil {
		return err
	}
	return os.Rename(tmp, path)
}
