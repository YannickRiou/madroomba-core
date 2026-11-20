package mapping

import (
	"bytes"
	"image/png"
	"math"
	"math/rand"
	"testing"

	"github.com/YannickRiou/madroomba-core/internal/lidar"
	"github.com/YannickRiou/madroomba-core/internal/odom"
	"github.com/YannickRiou/madroomba-core/internal/sim"
)

// relocScan is a scan of the world with 1 cm gaussian range noise, like the
// simulator's.
func relocScan(w *sim.World, p odom.Pose, rng *rand.Rand) *lidar.Scan {
	s := &lidar.Scan{}
	for deg := 0; deg < 360; deg++ {
		a := p.Theta + float64(deg)*math.Pi/180
		d := w.Raycast(p.X, p.Y, a, lidar.MaxRangeM)
		if d == 0 {
			continue
		}
		d += rng.NormFloat64() * 0.01
		if d >= lidar.MinRangeM && d <= lidar.MaxRangeM {
			s.Ranges[deg] = d
		}
	}
	return s
}

// relocRoom is a free spot of the default world: at least 30 cm from every
// wall and outside the two boxes.
func relocRoom(w *sim.World, x, y, clear float64) bool {
	if x > -2.6 && x < -1.4 && y > 1.4 && y < 2.4 || x > -0.5 && x < 0.7 && y > -2.5 && y < -1.9 {
		return false
	}
	for _, s := range w.Segments {
		ex, ey := s.X2-s.X1, s.Y2-s.Y1
		u := math.Max(0, math.Min(1, ((x-s.X1)*ex+(y-s.Y1)*ey)/(ex*ex+ey*ey)))
		if math.Hypot(x-s.X1-u*ex, y-s.Y1-u*ey) < clear {
			return false
		}
	}
	return true
}

// relocMap integrates scans taken on a regular lattice of the world, as a
// real exploration would, into a 400x400 grid.
func relocMap(w *sim.World, rng *rand.Rand) *Grid {
	g := NewGrid(400, 400, 0.05)
	for x := -3.8; x <= 3.8; x += 0.25 {
		for y := -2.8; y <= 2.8; y += 0.25 {
			if !relocRoom(w, x, y, 0.3) {
				continue
			}
			for _, th := range []float64{0, 2.1, 4.2} {
				g.IntegrateScan(odom.Pose{X: x, Y: y, Theta: th}, relocScan(w, odom.Pose{X: x, Y: y, Theta: th}, rng))
			}
		}
	}
	return g
}

// relocCheck relocalizes 30 random poses of the default world and counts the
// refusals and the confident answers that are wrong.
func relocCheck(t *testing.T, g *Grid, w *sim.World, rng *rand.Rand) {
	t.Helper()
	solved, refused, wrong := 0, 0, 0
	const total = 30
	for solved+refused+wrong < total {
		p := odom.Pose{X: rng.Float64()*7.6 - 3.8, Y: rng.Float64()*5.6 - 2.8, Theta: (rng.Float64()*2 - 1) * math.Pi}
		if !relocRoom(w, p.X, p.Y, 1.0) {
			continue
		}
		loc, ok := g.Relocalize(relocScan(w, p, rng))
		switch {
		case !ok:
			refused++
			t.Logf("refused %+v: best %+v score %.2f second %.2f", p, loc.Pose, loc.Score, loc.Second)
		case math.Hypot(loc.Pose.X-p.X, loc.Pose.Y-p.Y) < 0.10 &&
			math.Abs(odom.NormalizeAngle(loc.Pose.Theta-p.Theta)) < 3*math.Pi/180:
			solved++
		default:
			wrong++
			t.Errorf("confident but wrong: true %+v, got %+v score %.2f", p, loc.Pose, loc.Score)
		}
	}
	t.Logf("solved %d, refused %d, wrong %d of %d", solved, refused, wrong, total)
	// The default world is a long room seen through a 3.5 m lidar: from many
	// spots a wall segment looks like another, so some refusals are right.
	if solved*10 < total*8 {
		t.Errorf("solved %d of %d, want at least 80%%", solved, total)
	}
}

func TestRelocalizeLiveGrid(t *testing.T) {
	rng := rand.New(rand.NewSource(1))
	w := sim.DefaultWorld()
	relocCheck(t, relocMap(w, rng), w, rng)
}

func TestRelocalizeSavedGrid(t *testing.T) {
	rng := rand.New(rand.NewSource(2))
	w := sim.DefaultWorld()
	src := relocMap(w, rng)
	pngData, yaml, err := src.EncodeROS("m")
	if err != nil {
		t.Fatal(err)
	}
	meta, err := ParseROSYAML(yaml)
	if err != nil {
		t.Fatal(err)
	}
	img, err := png.Decode(bytes.NewReader(pngData))
	if err != nil {
		t.Fatal(err)
	}
	g := NewGrid(400, 400, 0.05)
	g.DecodeROS(meta, img)
	relocCheck(t, g, w, rng)
}

func TestRelocalizeRefusesWhenBlind(t *testing.T) {
	rng := rand.New(rand.NewSource(3))
	w := sim.DefaultWorld()
	p := odom.Pose{X: -1, Y: 0.5, Theta: 1}
	scan := relocScan(w, p, rng)

	if _, ok := NewGrid(400, 400, 0.05).Relocalize(scan); ok {
		t.Error("empty grid: ok, want refusal")
	}

	g := relocMap(w, rng)
	few := &lidar.Scan{}
	for deg := 0; deg < 360; deg += 20 {
		few.Ranges[deg] = scan.Ranges[deg]
	}
	if _, ok := g.Relocalize(few); ok {
		t.Error("scan with 18 returns: ok, want refusal")
	}
	if _, ok := g.Relocalize(&lidar.Scan{}); ok {
		t.Error("scan without returns: ok, want refusal")
	}
}

// A plain rectangle looks the same from two poses: the answer is a refusal or
// one of them, never a pose that does not explain the scan.
func TestRelocalizeSymmetricRoom(t *testing.T) {
	rng := rand.New(rand.NewSource(4))
	w := &sim.World{Segments: []sim.Segment{
		{X1: -1.5, Y1: -1, X2: 1.5, Y2: -1}, {X1: 1.5, Y1: -1, X2: 1.5, Y2: 1},
		{X1: 1.5, Y1: 1, X2: -1.5, Y2: 1}, {X1: -1.5, Y1: 1, X2: -1.5, Y2: -1},
	}}
	g := NewGrid(400, 400, 0.05)
	for x := -1.3; x <= 1.3; x += 0.2 {
		for y := -0.8; y <= 0.8; y += 0.2 {
			p := odom.Pose{X: x, Y: y}
			g.IntegrateScan(p, relocScan(w, p, rng))
		}
	}
	for i := 0; i < 10; i++ {
		p := odom.Pose{X: rng.Float64()*2 - 1, Y: rng.Float64()*1.2 - 0.6, Theta: (rng.Float64()*2 - 1) * math.Pi}
		scan := relocScan(w, p, rng)
		loc, ok := g.Relocalize(scan)
		if !ok {
			continue
		}
		good, n := 0, 0
		for deg, r := range scan.Ranges {
			if r == 0 {
				continue
			}
			n++
			a := loc.Pose.Theta + float64(deg)*math.Pi/180
			if math.Abs(w.Raycast(loc.Pose.X, loc.Pose.Y, a, lidar.MaxRangeM)-r) < 0.1 {
				good++
			}
		}
		if good*10 < n*9 {
			t.Errorf("pose %+v answered %+v, which explains only %d of %d rays", p, loc.Pose, good, n)
		}
	}
}

func BenchmarkRelocalize(b *testing.B) {
	rng := rand.New(rand.NewSource(1))
	w := sim.DefaultWorld()
	g := relocMap(w, rng)
	scan := relocScan(w, odom.Pose{X: -1.7, Y: -0.6, Theta: 0.8}, rng)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		g.Relocalize(scan)
	}
}
