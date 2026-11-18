package nav

import (
	"math"
	"sync"
	"testing"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/geom"
	"github.com/YannickRiou/madroomba-core/internal/mapping"
	"github.com/YannickRiou/madroomba-core/internal/odom"
	"github.com/YannickRiou/madroomba-core/internal/sim"
)

// poly builds a polygon from x, y pairs.
func poly(xy ...float64) geom.Polygon {
	var p geom.Polygon
	for i := 0; i+1 < len(xy); i += 2 {
		p = append(p, geom.Point{X: xy[i], Y: xy[i+1]})
	}
	return p
}

func TestCoverageRectangle(t *testing.T) {
	// 2 x 1 m, lanes along X every 0.25 m, 0.17 m margin
	rect := geom.Rect(0, 0, 2, 1)
	pts := Coverage(rect, 0.25, 0.17)
	if len(pts)%2 != 0 || len(pts) < 6 {
		t.Fatalf("got %d points", len(pts))
	}
	for i, p := range pts {
		if !rect.Contains(p) || rect.EdgeDist(p) < 0.17-1e-9 {
			t.Errorf("point %d %v closer than the margin to the outline", i, p)
		}
	}
	for i := 0; i+1 < len(pts); i += 2 {
		if pts[i].Y != pts[i+1].Y || math.Abs(pts[i].X-pts[i+1].X) < 1.4 {
			t.Errorf("lane %d not a full lane along X: %v %v", i/2, pts[i], pts[i+1])
		}
	}
	if last := pts[len(pts)-1]; math.Abs(last.Y-0.83) > 1e-9 {
		t.Errorf("last lane at y=%v, want the far edge 0.83", last.Y)
	}
	if pts[0].X > pts[1].X || pts[2].X < pts[3].X {
		t.Error("lanes do not alternate direction")
	}
	// a tall rectangle sweeps along Y
	if tall := Coverage(geom.Rect(0, 0, 1, 3), 0.25, 0.17); tall[0].X != tall[1].X {
		t.Errorf("tall zone not swept along Y: %v %v", tall[0], tall[1])
	}
	// too narrow for the robot: nothing
	if s := Coverage(geom.Rect(0, 0, 0.3, 0.3), 0.25, 0.17); len(s) != 0 {
		t.Errorf("tiny zone: %v", s)
	}
}

func TestCoverageConcave(t *testing.T) {
	// a U: 3 x 2 m with a 1 m wide notch from the top, down to y = 1
	u := poly(0, 0, 3, 0, 3, 2, 2, 2, 2, 1, 1, 1, 1, 2, 0, 2)
	pts := Coverage(u, 0.25, 0.17)
	split := false
	for i := 0; i+3 < len(pts); i += 2 {
		if pts[i].Y == pts[i+2].Y { // two segments on one lane
			split = true
		}
	}
	if !split {
		t.Error("no lane split in two by the notch")
	}
	for i := 0; i+1 < len(pts); i += 2 { // no segment crosses the notch
		mid := geom.Point{X: (pts[i].X + pts[i+1].X) / 2, Y: pts[i].Y}
		if !u.Contains(mid) {
			t.Errorf("segment %v -> %v crosses the notch", pts[i], pts[i+1])
		}
	}
}

func TestWheelsFor(t *testing.T) {
	p := odom.Pose{}
	if l, r := wheelsFor(p, mapping.Point{X: 2}, 200, 120); l != 200 || r != 200 {
		t.Errorf("straight ahead: %d %d", l, r)
	}
	if l, r := wheelsFor(p, mapping.Point{Y: 1}, 200, 120); l != -120 || r != 120 {
		t.Errorf("target on the left: %d %d, want a counter-clockwise turn", l, r)
	}
	if l, r := wheelsFor(p, mapping.Point{X: 2, Y: 0.2}, 200, 120); !(r > l && l > 0) {
		t.Errorf("slightly left: %d %d", l, r)
	}
	if l, r := wheelsFor(p, mapping.Point{X: 0.02}, 200, 120); l != 60 || r != 60 {
		t.Errorf("near the target: %d %d, want the minimum speed", l, r)
	}
}

// simBase runs the simulator ten times faster than real time.
type simBase struct {
	*sim.Robot
	mu    sync.Mutex
	trace []odom.Pose
	stop  chan struct{}
}

func newSimBase() *simBase {
	b := &simBase{Robot: sim.NewRobot(sim.DefaultWorld()), stop: make(chan struct{})}
	go func() {
		for {
			select {
			case <-b.stop:
				return
			case <-time.After(5 * time.Millisecond):
				b.Step(50 * time.Millisecond)
				b.mu.Lock()
				b.trace = append(b.trace, b.TruePose())
				b.mu.Unlock()
			}
		}
	}()
	return b
}

func (b *simBase) Pose() odom.Pose { return b.TruePose() }

func TestCleanerSweepsZoneInSim(t *testing.T) {
	b := newSimBase()
	defer close(b.stop)
	cfg := DefaultConfig()
	cfg.Tick = 10 * time.Millisecond
	done := make(chan Status, 1)
	c := &Cleaner{
		Base: b, Pose: b, Planner: mapping.NewGrid(200, 200, 0.05), Cfg: cfg,
		Publish: func(s Status) {
			if s.State == StateDone || s.State == StateFailed || s.State == StateCanceled {
				done <- s
			}
		},
	}
	// an L in the free space left of the start point
	area := Area{ID: "z1", Name: "test", Polygon: poly(-1.3, -0.5, -0.5, -0.5, -0.5, 0, -0.9, 0, -0.9, 0.6, -1.3, 0.6)}
	if err := c.Start(area, false); err != nil {
		t.Fatal(err)
	}
	if c.Start(area, false) == nil {
		t.Error("a second job started while one is running")
	}
	select {
	case s := <-done:
		if s.State != StateDone || s.Progress != 1 {
			t.Fatalf("job ended %+v", s)
		}
	case <-time.After(20 * time.Second):
		c.Stop()
		t.Fatal("job did not finish")
	}
	// the robot must have passed near every lane end
	b.mu.Lock()
	trace := append([]odom.Pose{}, b.trace...)
	b.mu.Unlock()
	for _, wp := range Coverage(area.Polygon, cfg.LaneSpacing, cfg.RobotRadius) {
		best := math.Inf(1)
		for _, p := range trace {
			best = math.Min(best, math.Hypot(p.X-wp.X, p.Y-wp.Y))
		}
		if best > 0.08 {
			t.Errorf("waypoint %v missed by %.2f m", wp, best)
		}
	}
	if c.Running() {
		t.Error("still running after done")
	}
}

func TestExploreMapsTheFlat(t *testing.T) {
	if testing.Short() {
		t.Skip("about 30 s of simulated driving")
	}
	b := newSimBase()
	defer close(b.stop)
	grid := mapping.NewGrid(200, 200, 0.05)
	// the lidar loop of the daemon: integrate a scan from the true pose
	go func() {
		for {
			select {
			case <-b.stop:
				return
			case <-time.After(20 * time.Millisecond):
				grid.IntegrateScan(b.TruePose(), b.Scan())
			}
		}
	}()
	time.Sleep(100 * time.Millisecond)
	before := len(grid.Frontiers(6))
	cfg := DefaultConfig()
	cfg.Tick = 10 * time.Millisecond
	done := make(chan Status, 1)
	c := &Cleaner{Base: b, Pose: b, Planner: grid, Frontiers: grid, Cfg: cfg,
		Publish: func(s Status) {
			if s.State == StateDone || s.State == StateFailed || s.State == StateCanceled {
				done <- s
			}
		}}
	if err := c.Explore(false); err != nil {
		t.Fatal(err)
	}
	select {
	case s := <-done:
		if s.State != StateDone || s.Visited == 0 || s.Kind != KindExplore {
			t.Fatalf("exploration ended %+v", s)
		}
		t.Logf("%d frontiers at start, %d reached, %d left", before, s.Visited, len(grid.Frontiers(6)))
	case <-time.After(60 * time.Second):
		c.Stop()
		t.Fatal("exploration did not finish")
	}
}

func TestCleanerStop(t *testing.T) {
	b := newSimBase()
	defer close(b.stop)
	cfg := DefaultConfig()
	cfg.Tick = 10 * time.Millisecond
	ended := make(chan Status, 4)
	c := &Cleaner{Base: b, Pose: b, Planner: mapping.NewGrid(200, 200, 0.05), Cfg: cfg,
		Publish: func(s Status) {
			if s.State == StateCanceled {
				ended <- s
			}
		}}
	c.Start(Area{Name: "far", Polygon: geom.Rect(-3, -1, -2, 0)}, false)
	time.Sleep(200 * time.Millisecond)
	c.Stop()
	select {
	case <-ended:
	case <-time.After(5 * time.Second):
		t.Fatal("not canceled")
	}
}
