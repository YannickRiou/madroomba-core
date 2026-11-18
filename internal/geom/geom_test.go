package geom

import (
	"math"
	"testing"
)

// an L shape: 2 x 2 m square without its top right 1 x 1 m quarter
var lShape = Polygon{{0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}}

func TestContainsAndDistance(t *testing.T) {
	for _, c := range []struct {
		p    Point
		in   bool
		dist float64
	}{
		{Point{0.5, 0.5}, true, 0},
		{Point{1.5, 0.5}, true, 0},
		{Point{0.5, 1.5}, true, 0},
		{Point{1.5, 1.5}, false, 0.5}, // in the missing quarter
		{Point{3, 0.5}, false, 1},
		{Point{-1, -1}, false, math.Sqrt2},
	} {
		if got := lShape.Contains(c.p); got != c.in {
			t.Errorf("Contains(%v) = %v", c.p, got)
		}
		if got := lShape.Distance(c.p); math.Abs(got-c.dist) > 1e-9 {
			t.Errorf("Distance(%v) = %v, want %v", c.p, got, c.dist)
		}
	}
	if d := lShape.EdgeDist(Point{0.5, 0.5}); math.Abs(d-0.5) > 1e-9 {
		t.Errorf("EdgeDist inside = %v", d)
	}
}

func TestPolyline(t *testing.T) {
	wall := []Point{{0, 0}, {2, 0}, {2, 1}}
	if d := PolylineDist(wall, Point{1, 0.3}); math.Abs(d-0.3) > 1e-9 {
		t.Errorf("distance = %v, want 0.3", d)
	}
	if d := PolylineDist(wall, Point{1, 0.8}); math.Abs(d-0.8) > 1e-9 {
		t.Errorf("open polyline is not closed: distance = %v, want 0.8", d)
	}
	if l := Length(wall); l != 3 {
		t.Errorf("length = %v", l)
	}
}

func TestAreaBoundsSimple(t *testing.T) {
	if a := lShape.Area(); math.Abs(a-3) > 1e-9 {
		t.Errorf("area = %v, want 3", a)
	}
	min, max := lShape.Bounds()
	if min != (Point{0, 0}) || max != (Point{2, 2}) {
		t.Errorf("bounds %v %v", min, max)
	}
	if !lShape.Simple() || !Rect(0, 0, 1, 1).Simple() {
		t.Error("simple polygons reported as crossing")
	}
	bowtie := Polygon{{0, 0}, {1, 1}, {1, 0}, {0, 1}}
	if bowtie.Simple() {
		t.Error("bow tie reported as simple")
	}
}
