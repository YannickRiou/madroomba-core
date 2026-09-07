package sim

import (
	"math"
	"testing"
	"time"
)

func TestRaycastHitsWall(t *testing.T) {
	w := DefaultWorld()
	// From origin looking +X: partition wall sits at x=1.2 but has a door
	// between y=-0.8 and y=0.4, so the ray goes through to the wall at x=4,
	// which is beyond typical lidar range. Use the -X direction: wall at -4.
	d := w.Raycast(0, 0, math.Pi, 10)
	if math.Abs(d-4.0) > 1e-9 {
		t.Errorf("raycast -X = %f, want 4.0", d)
	}
	// Looking +Y: outer wall at y=3.
	d = w.Raycast(0, 0, math.Pi/2, 10)
	if math.Abs(d-3.0) > 1e-9 {
		t.Errorf("raycast +Y = %f, want 3.0", d)
	}
	// Capped by maxR.
	if d := w.Raycast(0, 0, math.Pi/2, 2); d != 0 {
		t.Errorf("raycast beyond maxR = %f, want 0", d)
	}
}

func TestDriveStraightOdometry(t *testing.T) {
	r := NewRobot(DefaultWorld())
	r.SetWheels(200, 200) // 0.2 m/s forward
	for i := 0; i < 20; i++ {
		r.Step(50 * time.Millisecond)
	}
	dist, ang, _ := r.Odometry()
	// 1 second at 0.2 m/s with 1% noise -> ~200mm.
	if dist < 150 || dist > 250 {
		t.Errorf("distance = %dmm, want ~200", dist)
	}
	if ang < -3 || ang > 3 {
		t.Errorf("angle = %d deg, want ~0", ang)
	}
	p := r.TruePose()
	if p.X < 0.15 || p.X > 0.25 {
		t.Errorf("true X = %f, want ~0.2", p.X)
	}
}

func TestCollisionStopsRobot(t *testing.T) {
	r := NewRobot(DefaultWorld())
	r.SetWheels(400, 400)
	// Drive +X for 20 simulated seconds (8m requested): the robot passes
	// through the partition door then must stop at the east wall (x=4),
	// bumper first (radius 0.17).
	for i := 0; i < 400; i++ {
		r.Step(50 * time.Millisecond)
	}
	p := r.TruePose()
	if p.X > 4.0-robotRadius+0.02 {
		t.Errorf("robot went through the east wall: %+v", p)
	}
	if p.X < 3.5 {
		t.Errorf("robot should have reached the east wall: %+v", p)
	}
}

func TestScanSeesRoom(t *testing.T) {
	r := NewRobot(DefaultWorld())
	s := r.Scan()
	valid := 0
	for _, v := range s.Ranges {
		if v > 0 {
			valid++
		}
	}
	if valid < 180 {
		t.Errorf("only %d valid rays, expected most of the room in range", valid)
	}
	// Ray to +Y (deg 90 from theta=0) should be ~3m.
	if s.Ranges[90] < 2.8 || s.Ranges[90] > 3.2 {
		t.Errorf("ranges[90] = %f, want ~3.0", s.Ranges[90])
	}
}
