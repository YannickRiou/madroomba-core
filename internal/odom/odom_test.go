package odom

import (
	"math"
	"testing"
)

func almost(a, b float64) bool { return math.Abs(a-b) < 1e-6 }

func TestStraightLine(t *testing.T) {
	tr := &Tracker{}
	for i := 0; i < 10; i++ {
		tr.Update(100, 0) // 10 x 100mm forward
	}
	p := tr.Pose()
	if !almost(p.X, 1.0) || !almost(p.Y, 0) || !almost(p.Theta, 0) {
		t.Errorf("pose = %+v, want (1, 0, 0)", p)
	}
}

func TestTurnInPlace(t *testing.T) {
	tr := &Tracker{}
	tr.Update(0, 90)
	p := tr.Pose()
	if !almost(p.Theta, math.Pi/2) {
		t.Errorf("theta = %f, want pi/2", p.Theta)
	}
	if !almost(p.X, 0) || !almost(p.Y, 0) {
		t.Errorf("position moved during pure rotation: %+v", p)
	}
}

func TestQuarterArcHeadsNorth(t *testing.T) {
	tr := &Tracker{}
	// Quarter circle: 90° left in small steps while moving.
	for i := 0; i < 9; i++ {
		tr.Update(50, 10)
	}
	p := tr.Pose()
	if !almost(p.Theta, math.Pi/2) {
		t.Errorf("theta = %f, want pi/2", p.Theta)
	}
	if p.X <= 0 || p.Y <= 0 {
		t.Errorf("arc should end up in +X +Y quadrant, got %+v", p)
	}
}

func TestNormalizeAngle(t *testing.T) {
	if !almost(NormalizeAngle(3*math.Pi), math.Pi) {
		t.Error("3pi should normalize to pi")
	}
	if !almost(NormalizeAngle(-3*math.Pi), math.Pi) {
		t.Error("-3pi should normalize to pi")
	}
}
