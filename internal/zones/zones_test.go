package zones

import (
	"math"
	"path/filepath"
	"testing"

	"github.com/YannickRiou/madroomba-core/internal/geom"
)

// poly builds a polygon (or a wall line) from x, y pairs.
func poly(xy ...float64) geom.Polygon {
	var p geom.Polygon
	for i := 0; i+1 < len(xy); i += 2 {
		p = append(p, geom.Point{X: xy[i], Y: xy[i+1]})
	}
	return p
}

func TestValidate(t *testing.T) {
	zs := []Zone{
		{Name: "Salon", Kind: Clean, Points: poly(0, 0, 2, 0, 2, 1, 1, 1, 1, 2, 0, 2, 0, 0)},
		{Name: "Ancien", Kind: Avoid, X0: 3, Y0: 1, X1: 2, Y1: 2}, // first format: a rectangle
		{Name: "Escalier", Kind: Wall, Points: poly(0, 3, 1, 3)},
	}
	if err := Validate(zs); err != nil {
		t.Fatal(err)
	}
	if len(zs[0].Points) != 6 || zs[0].ID == "" {
		t.Errorf("closing vertex not merged or no id: %+v", zs[0])
	}
	if len(zs[1].Points) != 4 || zs[1].X0 != 0 || zs[1].Points.Area() != 1 {
		t.Errorf("legacy rectangle not converted: %+v", zs[1])
	}
	for name, z := range map[string]Zone{
		"kind":       {Name: "a", Kind: "paint", Points: geom.Rect(0, 0, 1, 1)},
		"name":       {Kind: Clean, Points: geom.Rect(0, 0, 1, 1)},
		"small":      {Name: "a", Kind: Clean, Points: geom.Rect(0, 0, 0.1, 0.1)},
		"far":        {Name: "a", Kind: Clean, Points: geom.Rect(100, 0, 101, 1)},
		"id":         {ID: "../x", Name: "a", Kind: Clean, Points: geom.Rect(0, 0, 1, 1)},
		"two points": {Name: "a", Kind: Avoid, Points: poly(0, 0, 1, 1)},
		"crossing":   {Name: "a", Kind: Avoid, Points: poly(0, 0, 1, 1, 1, 0, 0, 1)},
		"short wall": {Name: "a", Kind: Wall, Points: poly(0, 0, 0.1, 0)},
	} {
		if Validate([]Zone{z}) == nil {
			t.Errorf("%s: accepted %+v", name, z)
		}
	}
}

func TestBlocked(t *testing.T) {
	s := &Store{}
	s.replace([]Zone{
		{Kind: Avoid, Points: geom.Rect(0, 0, 1, 1)},
		{Kind: Clean, Points: geom.Rect(5, 5, 6, 6)},
		{Kind: Wall, Points: poly(0, 3, 2, 3)},
	})
	for _, c := range []struct {
		p    geom.Point
		want bool
	}{
		{geom.Point{X: 1.1, Y: 0.5}, true},  // next to the no-go zone
		{geom.Point{X: 5.5, Y: 5.5}, false}, // inside a clean zone
		{geom.Point{X: 1, Y: 3.1}, true},    // next to the wall
		{geom.Point{X: 3, Y: 3}, false},     // beyond the wall end
	} {
		if got := s.Blocked(c.p, 0.2); got != c.want {
			t.Errorf("Blocked(%v) = %v", c.p, got)
		}
	}
}

func TestDriveAllowed(t *testing.T) {
	s := &Store{}
	s.replace([]Zone{{Kind: Avoid, Points: geom.Rect(1, -1, 2, 1)}})
	p := geom.Point{X: 0.7} // 0.3 m from the zone; 1 s at 0.2 m/s ends 0.1 m from it
	if s.DriveAllowed(p, 0, 0.2, 0.17, 1) {
		t.Error("driving into the zone allowed")
	}
	if !s.DriveAllowed(p, math.Pi, 0.2, 0.17, 1) {
		t.Error("driving away refused")
	}
	if !s.DriveAllowed(p, math.Pi/2, 0.2, 0.17, 1) {
		t.Error("driving along the zone, far enough, refused")
	}
	// already too close: backing out is allowed, going on is not
	q := geom.Point{X: 0.9}
	if !s.DriveAllowed(q, 0, -0.2, 0.17, 1) || s.DriveAllowed(q, 0, 0.1, 0.17, 1) {
		t.Error("escape rule broken")
	}
	// a virtual wall across the way
	w := &Store{}
	w.replace([]Zone{{Kind: Wall, Points: poly(0.5, -2, 0.5, 2)}})
	if w.DriveAllowed(geom.Point{X: 0.2}, 0, 0.3, 0.17, 1) { // would end on the wall
		t.Error("driving through a virtual wall allowed")
	}
}

func TestStoreRoundTrip(t *testing.T) {
	dir := t.TempDir()
	s := &Store{Path: filepath.Join(dir, "zones.json")}
	if err := s.Load(); err != nil || len(s.List()) != 0 {
		t.Fatalf("empty load: %v %v", s.List(), err)
	}
	if err := s.Set([]Zone{{Name: "Cuisine", Kind: Clean, Points: geom.Rect(0, 0, 2, 2)}}); err != nil {
		t.Fatal(err)
	}
	copyPath := filepath.Join(dir, "maps", "salon.zones.json")
	if err := s.SaveCopy(copyPath); err != nil {
		t.Fatal(err)
	}
	if err := s.Set(nil); err != nil || len(s.List()) != 0 {
		t.Fatal("clear failed")
	}
	if err := s.LoadCopy(copyPath); err != nil || len(s.List()) != 1 {
		t.Fatalf("copy not restored: %v %v", s.List(), err)
	}
	r := &Store{Path: s.Path}
	if err := r.Load(); err != nil || len(r.List()) != 1 || r.List()[0].Name != "Cuisine" {
		t.Fatalf("not persisted: %v %v", r.List(), err)
	}
}
