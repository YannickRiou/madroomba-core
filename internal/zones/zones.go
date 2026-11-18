// Package zones holds what is drawn on the map: zones to clean, no-go
// zones and virtual walls. A zone is any simple polygon (concave shapes
// included); a virtual wall is an open line of one or more segments. The
// package validates them, stores them as JSON and answers the geometric
// questions the daemon needs: is a point blocked, may the robot drive
// this way.
//
// Zones live in the map frame (meters, X right, Y up), the same frame as
// the occupancy grid and the robot pose, so they stay valid as long as the
// map they were drawn on is used.
package zones

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"regexp"
	"strconv"
	"sync"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/geom"
)

// Kinds of zone.
const (
	Clean = "clean" // a polygon the robot can be sent to clean
	Avoid = "avoid" // a no-go polygon, for teleoperation and autonomous moves
	Wall  = "wall"  // a virtual wall: a line the robot does not cross
)

// Limits on what a zone may be.
const (
	MaxZones     = 50
	MaxPoints    = 100
	MinAreaM2    = 0.04 // smaller zones are drawing mistakes (20 x 20 cm)
	MinWallM     = 0.2  // shorter walls are drawing mistakes
	MaxCoordM    = 50.0 // far beyond any map
	MaxNameLen   = 40
	MinVertexGap = 0.01 // vertices closer than 1 cm are merged
)

// blocks tells whether the robot must keep away from this kind.
func blocks(kind string) bool { return kind == Avoid || kind == Wall }

var validID = regexp.MustCompile(`^[a-z0-9-]{1,32}$`)

// Zone is a named polygon in the map frame.
type Zone struct {
	ID     string       `json:"id"`
	Name   string       `json:"name"`
	Kind   string       `json:"kind"`
	Points geom.Polygon `json:"points"`

	// X0, Y0, X1, Y1 accept the rectangles of the first version of the
	// format; Validate turns them into Points.
	X0 float64 `json:"x0,omitempty"`
	Y0 float64 `json:"y0,omitempty"`
	X1 float64 `json:"x1,omitempty"`
	Y1 float64 `json:"y1,omitempty"`
}

// Distance is the distance from p to the zone (0 inside a polygon), or to
// the line of a virtual wall.
func (z Zone) Distance(p geom.Point) float64 {
	if z.Kind == Wall {
		return geom.PolylineDist(z.Points, p)
	}
	return z.Points.Distance(p)
}

// clean converts a legacy rectangle and merges duplicate vertices,
// including, for a polygon, a closing vertex equal to the first one.
func (z *Zone) clean() {
	if len(z.Points) == 0 && (z.X0 != z.X1 || z.Y0 != z.Y1) {
		z.Points = geom.Rect(math.Min(z.X0, z.X1), math.Min(z.Y0, z.Y1), math.Max(z.X0, z.X1), math.Max(z.Y0, z.Y1))
	}
	z.X0, z.Y0, z.X1, z.Y1 = 0, 0, 0, 0
	var out geom.Polygon
	for _, p := range z.Points {
		if len(out) == 0 || geom.Dist(p, out[len(out)-1]) >= MinVertexGap {
			out = append(out, p)
		}
	}
	if z.Kind != Wall && len(out) > 1 && geom.Dist(out[0], out[len(out)-1]) < MinVertexGap {
		out = out[:len(out)-1]
	}
	z.Points = out
}

// checkShape validates the points of a polygon or of a virtual wall.
func (z Zone) checkShape() error {
	if z.Kind == Wall {
		if len(z.Points) < 2 || len(z.Points) > MaxPoints {
			return fmt.Errorf("wall %q: 2 to %d points, got %d", z.Name, MaxPoints, len(z.Points))
		}
		if geom.Length(z.Points) < MinWallM {
			return fmt.Errorf("wall %q: shorter than %.1f m", z.Name, MinWallM)
		}
		return nil
	}
	switch {
	case len(z.Points) < 3 || len(z.Points) > MaxPoints:
		return fmt.Errorf("zone %q: 3 to %d points, got %d", z.Name, MaxPoints, len(z.Points))
	case z.Points.Area() < MinAreaM2:
		return fmt.Errorf("zone %q: smaller than %.2f m²", z.Name, MinAreaM2)
	case !z.Points.Simple():
		return fmt.Errorf("zone %q: its edges cross each other", z.Name)
	}
	return nil
}

// Validate cleans the zones, gives an ID to new ones and checks them.
func Validate(zs []Zone) error {
	if len(zs) > MaxZones {
		return fmt.Errorf("at most %d zones", MaxZones)
	}
	seen := map[string]bool{}
	for i := range zs {
		z := &zs[i]
		z.clean()
		if z.ID == "" {
			z.ID = newID(i)
		}
		switch {
		case !validID.MatchString(z.ID) || seen[z.ID]:
			return fmt.Errorf("zone %d: bad or duplicate id %q", i+1, z.ID)
		case z.Kind != Clean && z.Kind != Avoid && z.Kind != Wall:
			return fmt.Errorf("zone %d: kind %q, want %q, %q or %q", i+1, z.Kind, Clean, Avoid, Wall)
		case z.Name == "" || len(z.Name) > MaxNameLen:
			return fmt.Errorf("zone %d: name must have 1 to %d characters", i+1, MaxNameLen)
		}
		if err := z.checkShape(); err != nil {
			return err
		}
		for _, p := range z.Points {
			if math.Abs(p.X) > MaxCoordM || math.Abs(p.Y) > MaxCoordM {
				return fmt.Errorf("zone %q: outside the map", z.Name)
			}
		}
		seen[z.ID] = true
	}
	return nil
}

func newID(i int) string {
	return "z" + strconv.FormatInt(time.Now().UnixNano()/1e6, 36) + strconv.Itoa(i)
}

// Store keeps the current zones, persisted as JSON in Path. It is safe for
// concurrent use.
type Store struct {
	Path string

	mu    sync.Mutex
	zones []Zone
}

// Load reads Path. A missing file means no zones.
func (s *Store) Load() error {
	data, err := os.ReadFile(s.Path)
	if os.IsNotExist(err) {
		s.replace(nil)
		return nil
	}
	if err != nil {
		return err
	}
	return s.SetJSON(data, false)
}

// SetJSON replaces the zones with a JSON array, saving it when persist is
// true.
func (s *Store) SetJSON(data []byte, persist bool) error {
	var zs []Zone
	if err := json.Unmarshal(data, &zs); err != nil {
		return err
	}
	if err := Validate(zs); err != nil {
		return err
	}
	if persist {
		if err := writeFile(s.Path, zs); err != nil {
			return err
		}
	}
	s.replace(zs)
	return nil
}

// Set validates, saves and replaces the zones.
func (s *Store) Set(zs []Zone) error {
	if err := Validate(zs); err != nil {
		return err
	}
	if err := writeFile(s.Path, zs); err != nil {
		return err
	}
	s.replace(zs)
	return nil
}

func (s *Store) replace(zs []Zone) {
	if zs == nil {
		zs = []Zone{}
	}
	s.mu.Lock()
	s.zones = zs
	s.mu.Unlock()
}

// SaveCopy writes the current zones to another file (next to a saved map).
func (s *Store) SaveCopy(path string) error {
	return writeFile(path, s.List())
}

// LoadCopy replaces the zones with those of another file (from a saved
// map) and saves them as the current ones. A missing file clears them.
func (s *Store) LoadCopy(path string) error {
	data, err := os.ReadFile(path)
	if os.IsNotExist(err) {
		return s.Set(nil)
	}
	if err != nil {
		return err
	}
	return s.SetJSON(data, true)
}

func writeFile(path string, zs []Zone) error {
	if zs == nil {
		zs = []Zone{}
	}
	data, _ := json.MarshalIndent(zs, "", "  ")
	if err := os.MkdirAll(filepath.Dir(path), 0755); err != nil {
		return err
	}
	return os.WriteFile(path, data, 0644)
}

// List returns a copy of the zones.
func (s *Store) List() []Zone {
	s.mu.Lock()
	defer s.mu.Unlock()
	return append([]Zone{}, s.zones...)
}

// Get returns the zone with this ID.
func (s *Store) Get(id string) (Zone, bool) {
	for _, z := range s.List() {
		if z.ID == id {
			return z, true
		}
	}
	return Zone{}, false
}

// Blocked tells whether p is within margin of a no-go zone or a virtual
// wall. It is the "blocked" function given to the path planner.
func (s *Store) Blocked(p geom.Point, margin float64) bool {
	for _, z := range s.List() {
		if blocks(z.Kind) && z.Distance(p) < margin {
			return true
		}
	}
	return false
}

// DriveAllowed tells whether a move from p, heading theta (radians) at
// speed v (m/s, negative backwards) keeps the robot (a disc of the given
// radius) away from the no-go zones and virtual walls over the next
// lookahead seconds. A move that takes the robot further from a zone it is
// already too close to is allowed, so it can always get out.
func (s *Store) DriveAllowed(p geom.Point, theta, v, radius, lookahead float64) bool {
	next := geom.Point{X: p.X + v*lookahead*math.Cos(theta), Y: p.Y + v*lookahead*math.Sin(theta)}
	for _, z := range s.List() {
		if !blocks(z.Kind) {
			continue
		}
		if d := z.Distance(next); d < radius && d < z.Distance(p) {
			return false
		}
	}
	return true
}
