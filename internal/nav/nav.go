// Package nav moves the robot on its own: it plans a path on the map,
// follows it with the wheels, and cleans a zone by sweeping it in parallel
// lanes. It only depends on small interfaces (Base, PoseSource, Planner),
// so it runs the same on the real Roomba and on the simulator.
//
// Safety comes first: the follower stops on a bumper hit, gives up on a
// cliff or a wheel drop, and the robot must stay in Safe or Full mode (the
// Roomba falls back to Passive by itself on a cliff or a wheel drop in
// Safe mode). Paths keep clear of mapped obstacles, of no-go zones and of
// the dock.
package nav

import (
	"errors"
	"math"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/geom"
	"github.com/YannickRiou/madroomba-core/internal/mapping"
	"github.com/YannickRiou/madroomba-core/internal/odom"
	"github.com/YannickRiou/madroomba-core/internal/robot"
)

// Base is what navigation needs from the robot.
type Base interface {
	SetWheels(leftMM, rightMM int16) error
	SetMotors(robot.Motors) error
	Sensors() (robot.Sensors, error)
	Dock() error
}

// PoseSource gives the current robot pose in the map frame.
type PoseSource interface {
	Pose() odom.Pose
}

// Planner finds a path on the map (implemented by *mapping.Grid).
type Planner interface {
	PlanPath(a, b mapping.Point, radius float64, blocked func(mapping.Point) bool) ([]mapping.Point, bool)
}

// Config holds the tunables of autonomous moves.
type Config struct {
	RobotRadius float64       // m, the Roomba 500 is about 0.17
	Clearance   float64       // m, extra distance kept from obstacles when planning
	LaneSpacing float64       // m, distance between cleaning lanes (brush width minus overlap)
	Cruise      int16         // mm/s, straight-line speed
	TurnSpeed   int16         // mm/s per wheel when turning in place
	Tick        time.Duration // control period
	// Dock is the dock position in the map frame. Maps are started with the
	// robot on its dock, at the origin facing +X, and the Roomba 500 docks
	// front first, so the dock sits just in front of the origin.
	Dock       mapping.Point
	DockRadius float64 // m, no-go disc around Dock for planned paths, 0 to disable

	ExploreTime   time.Duration // longest exploration
	FrontierCells int           // smallest frontier worth a visit, in map cells
	FrontierReach float64       // m, how close the robot drives to a frontier
}

// DefaultConfig suits a Roomba 500.
func DefaultConfig() Config {
	return Config{
		RobotRadius: 0.17,
		Clearance:   0.03,
		LaneSpacing: 0.25,
		Cruise:      200,
		TurnSpeed:   120,
		Tick:        100 * time.Millisecond,
		Dock:        mapping.Point{X: 0.25},
		DockRadius:  0.35,

		ExploreTime:   20 * time.Minute,
		FrontierCells: 6, // 30 cm with 5 cm cells
		FrontierReach: 0.25,
	}
}

// Errors returned by the follower.
var (
	ErrBumped      = errors.New("bumper hit")
	ErrStuck       = errors.New("cliff, wheel drop or control lost")
	ErrUnreachable = errors.New("no path")
	ErrTimeout     = errors.New("took too long")
)

// headingTolerance is the heading error above which the robot turns in
// place before driving.
const headingTolerance = 25 * math.Pi / 180

// wheelsFor computes wheel speeds that bring the robot from p toward
// target: turn in place while the heading error is large, else drive with
// a proportional heading correction, slowing down at the end.
func wheelsFor(p odom.Pose, target mapping.Point, cruise, turn int16) (left, right int16) {
	dx, dy := target.X-p.X, target.Y-p.Y
	errA := odom.NormalizeAngle(math.Atan2(dy, dx) - p.Theta)
	if math.Abs(errA) > headingTolerance {
		if errA > 0 { // target on the left: turn counter-clockwise
			return -turn, turn
		}
		return turn, -turn
	}
	v := math.Min(float64(cruise), math.Max(60, math.Hypot(dx, dy)*1000*1.5))
	k := 300.0 // mm/s of wheel difference per radian of heading error
	return clamp(v - k*errA), clamp(v + k*errA)
}

func clamp(v float64) int16 {
	return int16(math.Max(-500, math.Min(500, math.Round(v))))
}

// coverageStep is the sampling step along a lane.
const coverageStep = 0.05

// Coverage returns the waypoints that sweep a polygon in parallel lanes (a
// boustrophedon), along the longer side of its bounding box, as pairs of
// segment ends. Only points inside the polygon and at least margin from
// its outline are kept, so the robot body stays inside; a concave polygon
// can give several segments on one lane. Lanes alternate direction. A
// polygon too narrow for the robot gives no waypoint.
func Coverage(poly geom.Polygon, spacing, margin float64) []mapping.Point {
	min, max := poly.Bounds()
	alongX := max.X-min.X >= max.Y-min.Y
	// a0, a1: range along the lanes; c0, c1: range across them
	a0, a1, c0, c1 := min.X, max.X, min.Y, max.Y
	if !alongX {
		a0, a1, c0, c1 = min.Y, max.Y, min.X, max.X
	}
	pt := func(along, acr float64) mapping.Point {
		if alongX {
			return mapping.Point{X: along, Y: acr}
		}
		return mapping.Point{X: acr, Y: along}
	}
	ok := func(p mapping.Point) bool { return poly.Contains(p) && poly.EdgeDist(p) >= margin }

	var across []float64
	for c := c0 + margin; c < c1-margin-spacing/2; c += spacing {
		across = append(across, c)
	}
	across = append(across, math.Max(c0+margin, c1-margin)) // last lane along the far edge

	var out []mapping.Point
	for i, c := range across {
		// runs of valid samples along this lane
		var segs [][2]mapping.Point
		var start, last mapping.Point
		in := false
		for a := a0; a <= a1+1e-9; a += coverageStep {
			p := pt(a, c)
			if ok(p) {
				if !in {
					start, in = p, true
				}
				last = p
			} else if in {
				segs, in = append(segs, [2]mapping.Point{start, last}), false
			}
		}
		if in {
			segs = append(segs, [2]mapping.Point{start, last})
		}
		if i%2 == 1 { // odd lanes run backwards
			for l, r := 0, len(segs)-1; l < r; l, r = l+1, r-1 {
				segs[l], segs[r] = segs[r], segs[l]
			}
			for k := range segs {
				segs[k][0], segs[k][1] = segs[k][1], segs[k][0]
			}
		}
		for _, s := range segs {
			out = append(out, s[0], s[1])
		}
	}
	return out
}
