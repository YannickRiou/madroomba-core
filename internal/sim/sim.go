// Package sim provides a simulated Roomba + LDS-01 in a small virtual
// apartment, so the whole stack (odometry, mapping, UI) can be developed
// and demonstrated without hardware.
package sim

import (
	"math"
	"math/rand"
	"sync"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/lidar"
	"github.com/YannickRiou/madroomba-core/internal/odom"
	"github.com/YannickRiou/madroomba-core/internal/robot"
)

// Segment is a wall from (X1,Y1) to (X2,Y2), meters.
type Segment struct {
	X1, Y1, X2, Y2 float64
}

// World is a set of wall segments.
type World struct {
	Segments []Segment
}

// DefaultWorld is an 8x6 m room with a partition wall and two boxes.
func DefaultWorld() *World {
	w := &World{}
	rect := func(x0, y0, x1, y1 float64) {
		w.Segments = append(w.Segments,
			Segment{x0, y0, x1, y0},
			Segment{x1, y0, x1, y1},
			Segment{x1, y1, x0, y1},
			Segment{x0, y1, x0, y0},
		)
	}
	// Outer walls, robot starts at (0,0).
	rect(-4, -3, 4, 3)
	// Partition with a door opening.
	w.Segments = append(w.Segments,
		Segment{1.2, -3, 1.2, -0.8},
		Segment{1.2, 0.4, 1.2, 3},
	)
	// Furniture.
	rect(-2.6, 1.4, -1.4, 2.4)
	rect(-0.5, -2.5, 0.7, -1.9)
	return w
}

// Raycast returns the distance from (x,y) along heading a (radians) to the
// nearest wall, capped at maxR. Returns 0 if nothing is hit within maxR.
func (w *World) Raycast(x, y, a, maxR float64) float64 {
	dx := math.Cos(a)
	dy := math.Sin(a)
	best := math.Inf(1)
	for _, s := range w.Segments {
		ex := s.X2 - s.X1
		ey := s.Y2 - s.Y1
		den := dx*ey - dy*ex
		if math.Abs(den) < 1e-12 {
			continue
		}
		// Solve x + t*dx = X1 + u*ex ; y + t*dy = Y1 + u*ey
		t := ((s.X1-x)*ey - (s.Y1-y)*ex) / den
		var u float64
		if math.Abs(ex) > math.Abs(ey) {
			u = (x + t*dx - s.X1) / ex
		} else {
			u = (y + t*dy - s.Y1) / ey
		}
		if t > 0 && u >= 0 && u <= 1 && t < best {
			best = t
		}
	}
	if math.IsInf(best, 1) || best > maxR {
		return 0
	}
	return best
}

// Robot is a differential-drive robot implementing robot.Driver and
// producing lidar scans of the world. Call Step regularly.
type Robot struct {
	mu    sync.Mutex
	world *World
	pose  odom.Pose // ground truth
	vl    float64   // m/s
	vr    float64   // m/s

	// accumulated odometry since last Odometry() call, with sub-unit
	// remainders kept so quantization doesn't bias the estimate
	accDistM   float64
	accThetaRd float64

	rng *rand.Rand
}

const (
	wheelBaseM  = 0.298 // Roomba 500 wheel separation
	robotRadius = 0.17
)

// NewRobot places a simulated robot at the origin of the given world.
func NewRobot(w *World) *Robot {
	return &Robot{world: w, rng: rand.New(rand.NewSource(time.Now().UnixNano()))}
}

// Step advances the simulation by dt.
func (r *Robot) Step(dt time.Duration) {
	r.mu.Lock()
	defer r.mu.Unlock()
	s := dt.Seconds()
	v := (r.vl + r.vr) / 2
	w := (r.vr - r.vl) / wheelBaseM

	mid := r.pose.Theta + w*s/2
	nx := r.pose.X + v*s*math.Cos(mid)
	ny := r.pose.Y + v*s*math.Sin(mid)

	// Collision: block translation if the bumper would cross a wall.
	if v != 0 {
		heading := math.Atan2(ny-r.pose.Y, nx-r.pose.X)
		d := r.world.Raycast(r.pose.X, r.pose.Y, heading, robotRadius+math.Abs(v*s))
		if d != 0 && d <= robotRadius {
			nx, ny = r.pose.X, r.pose.Y
			v = 0
		}
	}

	r.pose.X, r.pose.Y = nx, ny
	r.pose.Theta = odom.NormalizeAngle(r.pose.Theta + w*s)

	// Odometry with slight noise, as the real encoders would report.
	r.accDistM += v * s * (1 + r.rng.NormFloat64()*0.01)
	r.accThetaRd += w * s * (1 + r.rng.NormFloat64()*0.02)
}

// TruePose returns the ground-truth pose (for debugging).
func (r *Robot) TruePose() odom.Pose {
	r.mu.Lock()
	defer r.mu.Unlock()
	return r.pose
}

// Scan simulates one full lidar revolution from the current pose.
func (r *Robot) Scan() *lidar.Scan {
	r.mu.Lock()
	p := r.pose
	r.mu.Unlock()

	s := &lidar.Scan{RPM: 300}
	for deg := 0; deg < 360; deg++ {
		a := p.Theta + float64(deg)*math.Pi/180
		d := r.world.Raycast(p.X, p.Y, a, lidar.MaxRangeM)
		if d == 0 {
			continue
		}
		d += r.rng.NormFloat64() * 0.01
		if d < lidar.MinRangeM || d > lidar.MaxRangeM {
			continue
		}
		s.Ranges[deg] = d
		s.Intensities[deg] = 200
	}
	return s
}

// --- robot.Driver implementation ---

// SetWheels sets the wheel speeds in mm/s.
func (r *Robot) SetWheels(leftMM, rightMM int16) error {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.vl = float64(leftMM) / 1000
	r.vr = float64(rightMM) / 1000
	return nil
}

// Odometry quantizes like the real encoders (about 0.44 mm per count) and
// keeps the remainders so quantization doesn't bias the estimate.
func (r *Robot) Odometry() (float64, float64, error) {
	r.mu.Lock()
	defer r.mu.Unlock()
	const step = 0.000444
	d := math.Trunc(r.accDistM/step) * step
	a := math.Trunc(r.accThetaRd/(step/wheelBaseM)) * (step / wheelBaseM)
	r.accDistM -= d
	r.accThetaRd -= a
	return d, a, nil
}

// Battery reports a fixed, partly charged battery.
func (r *Robot) Battery() (robot.Battery, error) {
	return robot.Battery{
		ChargeMAh:   2100,
		CapacityMAh: 2696,
		VoltageMV:   16200,
		CurrentMA:   -350,
		TempC:       28,
		Charging:    0,
	}, nil
}

// Sensors reports a robot in Safe mode with nothing triggered.
func (r *Robot) Sensors() (robot.Sensors, error) {
	return robot.Sensors{OIMode: 2}, nil
}

// The commands below have no effect in the simulator.

func (r *Robot) Clean() error                 { return nil }
func (r *Robot) Dock() error                  { return nil }
func (r *Robot) Reactivate() error            { return nil }
func (r *Robot) SetMotors(robot.Motors) error { return nil }
func (r *Robot) SetLEDs(robot.LEDs) error     { return nil }
func (r *Robot) PlaySong([]robot.Note) error  { return nil }
func (r *Robot) Close() error                 { return nil }

// Action accepts the names in robot.Actions and does nothing.
func (r *Robot) Action(name string) error {
	for _, a := range robot.Actions {
		if a == name {
			return nil
		}
	}
	return robot.ErrUnknownAction
}
