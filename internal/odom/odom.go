// Package odom integrates the Roomba's distance/angle sensor packets
// (Open Interface packets 19 and 20) into a 2D pose estimate.
package odom

import (
	"math"
	"sync"
)

// Pose is a 2D pose in the map frame. X/Y in meters, Theta in radians,
// counter-clockwise positive, 0 = facing +X.
type Pose struct {
	X, Y, Theta float64
}

// NormalizeAngle wraps an angle to (-pi, pi].
func NormalizeAngle(a float64) float64 {
	for a > math.Pi {
		a -= 2 * math.Pi
	}
	for a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// Tracker accumulates odometry deltas into a pose. Safe for concurrent use.
type Tracker struct {
	mu   sync.Mutex
	pose Pose
}

// Update applies one odometry delta: distance traveled in millimeters
// (packet 19) and angle turned in degrees, CCW positive (packet 20).
// Uses the midpoint model: translation along the average heading of the arc.
func (t *Tracker) Update(distanceMM, angleDeg int16) {
	t.mu.Lock()
	defer t.mu.Unlock()
	d := float64(distanceMM) / 1000.0
	dth := float64(angleDeg) * math.Pi / 180.0
	mid := t.pose.Theta + dth/2
	t.pose.X += d * math.Cos(mid)
	t.pose.Y += d * math.Sin(mid)
	t.pose.Theta = NormalizeAngle(t.pose.Theta + dth)
}

// Pose returns the current estimate.
func (t *Tracker) Pose() Pose {
	t.mu.Lock()
	defer t.mu.Unlock()
	return t.pose
}

// Set overrides the current estimate (used after scan matching).
func (t *Tracker) Set(p Pose) {
	t.mu.Lock()
	defer t.mu.Unlock()
	t.pose = p
}

// Reset moves the estimate back to the origin.
func (t *Tracker) Reset() {
	t.Set(Pose{})
}
