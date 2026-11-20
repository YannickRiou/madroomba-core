// Package mapping implements a log-odds occupancy grid built from lidar
// scans, plus a small hill-climbing scan matcher to correct odometry drift.
package mapping

import (
	"bytes"
	"image"
	"image/png"
	"math"
	"sync"

	"github.com/YannickRiou/madroomba-core/internal/lidar"
	"github.com/YannickRiou/madroomba-core/internal/odom"
)

const (
	logOddsHit  = 18
	logOddsFree = -6
	logOddsMin  = -120
	logOddsMax  = 120
	// occupiedThreshold is the log-odds value above which a cell counts as
	// an obstacle for scan matching.
	occupiedThreshold = 30
)

// Grid is a square occupancy grid centered on the map origin.
// Cell (W/2, H/2) is world (0,0); world +X goes right (increasing cx),
// world +Y goes up (decreasing cy), so the exported image is
// "north up" and matches a standard screen once drawn.
type Grid struct {
	mu    sync.Mutex
	W, H  int
	CellM float64 // cell edge, meters
	cells []int16
	// updates counts integrated scans, used to know when scan matching
	// becomes meaningful.
	updates int
}

// NewGrid creates a grid of w x h cells with the given cell size in meters.
func NewGrid(w, h int, cellM float64) *Grid {
	return &Grid{W: w, H: h, CellM: cellM, cells: make([]int16, w*h)}
}

// Reset clears the whole grid.
func (g *Grid) Reset() {
	g.mu.Lock()
	defer g.mu.Unlock()
	for i := range g.cells {
		g.cells[i] = 0
	}
	g.updates = 0
}

// Updates returns how many scans have been integrated so far.
func (g *Grid) Updates() int {
	g.mu.Lock()
	defer g.mu.Unlock()
	return g.updates
}

func (g *Grid) worldToCell(x, y float64) (int, int, bool) {
	cx := g.W/2 + int(math.Round(x/g.CellM))
	cy := g.H/2 - int(math.Round(y/g.CellM))
	if cx < 0 || cx >= g.W || cy < 0 || cy >= g.H {
		return 0, 0, false
	}
	return cx, cy, true
}

func (g *Grid) at(cx, cy int) int16 {
	return g.cells[cy*g.W+cx]
}

func (g *Grid) bump(cx, cy int, delta int16) {
	i := cy*g.W + cx
	v := int(g.cells[i]) + int(delta)
	if v > logOddsMax {
		v = logOddsMax
	}
	if v < logOddsMin {
		v = logOddsMin
	}
	g.cells[i] = int16(v)
}

// IntegrateScan carves free space along each ray and marks the hit cell.
// The scan must already be in the robot frame (0 deg = robot front, CCW).
func (g *Grid) IntegrateScan(p odom.Pose, s *lidar.Scan) {
	g.mu.Lock()
	defer g.mu.Unlock()

	rx, ry, ok := g.worldToCell(p.X, p.Y)
	if !ok {
		return
	}
	for deg := 0; deg < 360; deg++ {
		r := s.Ranges[deg]
		if r <= 0 {
			continue
		}
		a := p.Theta + float64(deg)*math.Pi/180.0
		hx := p.X + r*math.Cos(a)
		hy := p.Y + r*math.Sin(a)
		cx, cy, ok := g.worldToCell(hx, hy)
		if !ok {
			continue
		}
		g.line(rx, ry, cx, cy)
		g.bump(cx, cy, logOddsHit)
	}
	g.updates++
}

// line marks cells free along (x0,y0)->(x1,y1), excluding the endpoint.
func (g *Grid) line(x0, y0, x1, y1 int) {
	dx := abs(x1 - x0)
	dy := -abs(y1 - y0)
	sx := 1
	if x0 > x1 {
		sx = -1
	}
	sy := 1
	if y0 > y1 {
		sy = -1
	}
	err := dx + dy
	for x0 != x1 || y0 != y1 {
		g.bump(x0, y0, logOddsFree)
		e2 := 2 * err
		if e2 >= dy {
			err += dy
			x0 += sx
		}
		if e2 <= dx {
			err += dx
			y0 += sy
		}
	}
}

func abs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

// score evaluates how well scan endpoints land on known obstacles.
func (g *Grid) score(p odom.Pose, s *lidar.Scan) int {
	total := 0
	for deg := 0; deg < 360; deg += 2 { // every other ray is plenty
		r := s.Ranges[deg]
		if r <= 0 {
			continue
		}
		a := p.Theta + float64(deg)*math.Pi/180.0
		cx, cy, ok := g.worldToCell(p.X+r*math.Cos(a), p.Y+r*math.Sin(a))
		if !ok {
			continue
		}
		v := g.at(cx, cy)
		if v > 0 {
			total += int(v)
		}
	}
	return total
}

// ScanMatch refines an odometry pose guess by hill climbing over
// (x, y, theta) so the scan endpoints line up with mapped obstacles.
// Returns the guess unchanged while the map is still mostly empty.
func (g *Grid) ScanMatch(guess odom.Pose, s *lidar.Scan) odom.Pose {
	g.mu.Lock()
	defer g.mu.Unlock()

	if g.updates < 5 {
		return guess
	}

	best := guess
	bestScore := g.score(best, s)
	if bestScore == 0 {
		return guess
	}

	// Coarse-to-fine search steps.
	steps := []struct{ d, a float64 }{
		{0.05, 2.0 * math.Pi / 180},
		{0.02, 0.8 * math.Pi / 180},
		{0.01, 0.3 * math.Pi / 180},
	}
	for _, st := range steps {
		for iter := 0; iter < 12; iter++ {
			improved := false
			for _, c := range [...]odom.Pose{
				{X: best.X + st.d, Y: best.Y, Theta: best.Theta},
				{X: best.X - st.d, Y: best.Y, Theta: best.Theta},
				{X: best.X, Y: best.Y + st.d, Theta: best.Theta},
				{X: best.X, Y: best.Y - st.d, Theta: best.Theta},
				{X: best.X, Y: best.Y, Theta: best.Theta + st.a},
				{X: best.X, Y: best.Y, Theta: best.Theta - st.a},
			} {
				if sc := g.score(c, s); sc > bestScore {
					best, bestScore = c, sc
					improved = true
				}
			}
			if !improved {
				break
			}
		}
	}
	best.Theta = odom.NormalizeAngle(best.Theta)

	// Reject wild corrections: odometry is decent over one scan interval.
	if math.Hypot(best.X-guess.X, best.Y-guess.Y) > 0.30 ||
		math.Abs(odom.NormalizeAngle(best.Theta-guess.Theta)) > 12*math.Pi/180 {
		return guess
	}
	return best
}

// Meta describes the exported map image geometry.
type Meta struct {
	W       int     `json:"w"`
	H       int     `json:"h"`
	CellM   float64 `json:"cell"`
	Updates int     `json:"updates"`
}

// ToPNG renders the grid as a grayscale PNG: unknown = mid gray,
// free = white, occupied = black. Raw, the gray level follows the
// confidence of each cell; filtered, only confident cells are shown and
// isolated obstacles are dropped (see classes).
// The grid is only locked while the pixels are filled: encoding takes most
// of the time on a Pi Zero and must not hold up the lidar or the planner.
func (g *Grid) ToPNG(filter bool) ([]byte, Meta) {
	g.mu.Lock()
	img := image.NewGray(image.Rect(0, 0, g.W, g.H))
	meta := Meta{W: g.W, H: g.H, CellM: g.CellM, Updates: g.updates}
	if filter {
		for i, c := range g.classes(true) {
			switch c {
			case cellFree:
				img.Pix[i] = 255
			case cellOccupied:
				img.Pix[i] = 40
			default:
				img.Pix[i] = 190
			}
		}
		g.mu.Unlock()
		var buf bytes.Buffer
		png.Encode(&buf, img)
		return buf.Bytes(), meta
	}
	for i, v := range g.cells {
		var y uint8
		switch {
		case v == 0:
			y = 190
		case v > 0:
			// occupied: darker as confidence grows
			y = uint8(120 - int(v))
		default:
			// free: lighter as confidence grows
			y = uint8(200 + int(-v)*55/logOddsMax)
		}
		img.Pix[i] = y
	}
	g.mu.Unlock()
	var buf bytes.Buffer
	png.Encode(&buf, img)
	return buf.Bytes(), meta
}
