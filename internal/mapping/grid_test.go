package mapping

import (
	"bytes"
	"image/png"
	"math"
	"testing"

	"github.com/YannickRiou/madroomba-core/internal/lidar"
	"github.com/YannickRiou/madroomba-core/internal/odom"
	"github.com/YannickRiou/madroomba-core/internal/sim"
)

// scanAt raycasts a synthetic scan of the world from pose p (robot frame).
func scanAt(w *sim.World, p odom.Pose) *lidar.Scan {
	s := &lidar.Scan{}
	for deg := 0; deg < 360; deg++ {
		a := p.Theta + float64(deg)*math.Pi/180
		d := w.Raycast(p.X, p.Y, a, lidar.MaxRangeM)
		if d >= lidar.MinRangeM {
			s.Ranges[deg] = d
		}
	}
	return s
}

func TestIntegrateScanMarksHitAndFree(t *testing.T) {
	g := NewGrid(100, 100, 0.05)
	s := &lidar.Scan{}
	s.Ranges[0] = 1.0 // single ray straight ahead (+X), hit at 1m

	g.IntegrateScan(odom.Pose{}, s)

	// Hit cell: world (1, 0) -> cell (50+20, 50)
	if v := g.at(70, 50); v <= 0 {
		t.Errorf("hit cell log-odds = %d, want > 0", v)
	}
	// Free cell halfway: world (0.5, 0) -> cell (60, 50)
	if v := g.at(60, 50); v >= 0 {
		t.Errorf("free cell log-odds = %d, want < 0", v)
	}
	// Untouched cell stays unknown.
	if v := g.at(50, 60); v != 0 {
		t.Errorf("unknown cell log-odds = %d, want 0", v)
	}
}

func TestScanMatchRecoversOffset(t *testing.T) {
	w := sim.DefaultWorld()
	g := NewGrid(400, 400, 0.05)

	// Build a map from several true poses.
	truth := odom.Pose{X: 0, Y: 0, Theta: 0}
	for i := 0; i < 8; i++ {
		g.IntegrateScan(truth, scanAt(w, truth))
	}

	// New scan from the same spot, but odometry drifted 12cm / 4°.
	scan := scanAt(w, truth)
	guess := odom.Pose{X: 0.12, Y: -0.08, Theta: 4 * math.Pi / 180}

	got := g.ScanMatch(guess, scan)

	if d := math.Hypot(got.X-truth.X, got.Y-truth.Y); d > 0.06 {
		t.Errorf("position error after match = %.3fm (guess had 0.144m)", d)
	}
	if a := math.Abs(odom.NormalizeAngle(got.Theta - truth.Theta)); a > 2*math.Pi/180 {
		t.Errorf("angle error after match = %.1f deg", a*180/math.Pi)
	}
}

func TestScanMatchRejectsWildCorrection(t *testing.T) {
	g := NewGrid(100, 100, 0.05)
	s := &lidar.Scan{}
	s.Ranges[0] = 1.0
	for i := 0; i < 6; i++ {
		g.IntegrateScan(odom.Pose{}, s)
	}
	// With a single ray the matcher must not invent big jumps.
	guess := odom.Pose{X: 0.01, Y: 0.01}
	got := g.ScanMatch(guess, s)
	if math.Hypot(got.X-guess.X, got.Y-guess.Y) > 0.30 {
		t.Errorf("correction too large: %+v from guess %+v", got, guess)
	}
}

func TestToPNG(t *testing.T) {
	g := NewGrid(64, 64, 0.05)
	for _, filter := range []bool{false, true} {
		png, meta := g.ToPNG(filter)
		if len(png) == 0 {
			t.Fatal("empty png")
		}
		if meta.W != 64 || meta.H != 64 || meta.CellM != 0.05 {
			t.Errorf("bad meta: %+v", meta)
		}
	}
}

func TestFilterDropsIsolatedObstacle(t *testing.T) {
	g := NewGrid(20, 20, 0.05)
	for i := range g.cells {
		g.cells[i] = -60 // all free
	}
	g.cells[5*20+5] = 60 // lone obstacle
	for x := 10; x < 15; x++ {
		g.cells[12*20+x] = 60 // wall segment
	}
	c := g.classes(true)
	if c[5*20+5] != cellFree {
		t.Errorf("lone obstacle kept: %d", c[5*20+5])
	}
	for x := 10; x < 15; x++ {
		if c[12*20+x] != cellOccupied {
			t.Errorf("wall cell %d dropped", x)
		}
	}
}

func TestPlanPathGoesAroundWall(t *testing.T) {
	g := NewGrid(100, 100, 0.05) // 5 x 5 m, origin in the middle
	// vertical wall at x = 0 from y = -1.5 to y = 2.5
	for y := -30; y <= 50; y++ {
		g.cells[(50-y)*100+50] = 100
	}
	a, b := Point{X: -1}, Point{X: 1}
	path, ok := g.PlanPath(a, b, 0.17, nil)
	if !ok || len(path) < 2 {
		t.Fatalf("no path around the wall: %v %v", path, ok)
	}
	if last := path[len(path)-1]; last != b {
		t.Errorf("path ends at %v, want %v", last, b)
	}
	// every leg must stay clear of the wall: sample along each segment
	prev := a
	for _, p := range path {
		for s := 0.0; s <= 1; s += 0.02 {
			x, y := prev.X+(p.X-prev.X)*s, prev.Y+(p.Y-prev.Y)*s
			// distance to the wall segment x = 0, y in [-1.5, 2.5]
			d := math.Hypot(x, y-math.Max(-1.5, math.Min(2.5, y)))
			if d < 0.17 {
				t.Fatalf("segment %v -> %v passes %.3f m from the wall at (%.2f, %.2f)", prev, p, d, x, y)
			}
		}
		prev = p
	}
	// the wall has a gap below y = -1.5: the path must go that way
	if path[0].Y > -1.4 {
		t.Errorf("first waypoint %v does not go around the bottom end", path[0])
	}
}

func TestPlanPathBlockedAndUnreachable(t *testing.T) {
	g := NewGrid(60, 60, 0.05)
	a, b := Point{X: -1}, Point{X: 1}
	// a no-go band across the whole map makes b unreachable
	band := func(p Point) bool { return math.Abs(p.X) < 0.3 }
	if _, ok := g.PlanPath(a, b, 0.17, band); ok {
		t.Error("path found through a full no-go band")
	}
	// a goal inside a no-go zone is refused
	if _, ok := g.PlanPath(a, Point{}, 0.17, band); ok {
		t.Error("path to a goal inside a no-go zone")
	}
	// open map: straight line, a single waypoint (the goal)
	path, ok := g.PlanPath(a, b, 0.17, nil)
	if !ok || len(path) != 1 {
		t.Errorf("open map path = %v %v, want just the goal", path, ok)
	}
}

func TestFrontiers(t *testing.T) {
	g := NewGrid(40, 40, 0.05)
	// a free 1 m square around the origin, unknown everywhere else
	for y := 10; y < 30; y++ {
		for x := 10; x < 30; x++ {
			g.cells[y*40+x] = -60
		}
	}
	f := g.Frontiers(6)
	if len(f) == 0 {
		t.Fatal("no frontier around a free square in unknown space")
	}
	for _, p := range f {
		if math.Abs(p.X) > 0.55 || math.Abs(p.Y) > 0.55 {
			t.Errorf("frontier %v outside the free square", p)
		}
	}
	// walls all around: nothing left to explore
	for i := 9; i <= 30; i++ {
		for _, c := range [][2]int{{i, 9}, {i, 30}, {9, i}, {30, i}} {
			g.cells[c[1]*40+c[0]] = 60
		}
	}
	if f := g.Frontiers(6); len(f) != 0 {
		t.Errorf("frontiers inside a closed room: %v", f)
	}
}

func TestROSRoundTrip(t *testing.T) {
	g := NewGrid(40, 30, 0.05)
	for i := range g.cells {
		g.cells[i] = -60
	}
	for x := 5; x < 25; x++ {
		g.cells[7*40+x] = 60
	}
	g.cells[20*40+3] = 0 // unknown
	pngData, yml, err := g.EncodeROS("salon")
	if err != nil {
		t.Fatal(err)
	}
	m, err := ParseROSYAML(yml)
	if err != nil {
		t.Fatal(err)
	}
	if m.Image != "salon.png" || m.Resolution != 0.05 {
		t.Errorf("bad meta: %+v", m)
	}
	img, err := png.Decode(bytes.NewReader(pngData))
	if err != nil {
		t.Fatal(err)
	}
	h := NewGrid(40, 30, 0.05)
	h.DecodeROS(m, img)
	want := g.classes(true)
	got := h.classes(false)
	for i := range want {
		if want[i] != got[i] {
			t.Fatalf("cell %d (%d,%d): got %d, want %d", i, i%40, i/40, got[i], want[i])
		}
	}
	if h.Updates() < 5 {
		t.Error("scan matching not enabled after load")
	}
}
