package mapping

import (
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
	png, meta := g.ToPNG()
	if len(png) == 0 {
		t.Fatal("empty png")
	}
	if meta.W != 64 || meta.H != 64 || meta.CellM != 0.05 {
		t.Errorf("bad meta: %+v", meta)
	}
}
