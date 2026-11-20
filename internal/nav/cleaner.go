package nav

import (
	"context"
	"errors"
	"fmt"
	"log"
	"math"
	"sync"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/geom"
	"github.com/YannickRiou/madroomba-core/internal/mapping"
	"github.com/YannickRiou/madroomba-core/internal/robot"
)

// Job states, as shown on the page.
const (
	StateIdle      = "idle"
	StateGoing     = "going"     // driving to the zone
	StateCleaning  = "cleaning"  // sweeping the zone, motors on
	StateExploring = "exploring" // driving to the frontiers of the map
	StateDocking   = "docking"   // the Roomba is seeking its dock
	StateDone      = "done"
	StateFailed    = "failed"
	StateCanceled  = "canceled"
)

// Job kinds.
const (
	KindClean   = "clean"
	KindExplore = "explore"
)

// Status describes the current or last job.
type Status struct {
	Kind     string  `json:"kind,omitempty"` // KindClean or KindExplore
	State    string  `json:"state"`
	Zone     string  `json:"zone,omitempty"`    // zone ID
	Name     string  `json:"name,omitempty"`    // zone name
	Progress float64 `json:"progress"`          // 0 to 1, cleaning only
	Visited  int     `json:"visited,omitempty"` // frontiers reached, exploring only
	Message  string  `json:"message,omitempty"`
}

// FrontierSource tells where the map is still unknown (implemented by
// *mapping.Grid).
type FrontierSource interface {
	Frontiers(minCells int) []mapping.Point
}

// Area is the polygon to clean, in the map frame.
type Area struct {
	ID, Name string
	Polygon  geom.Polygon
}

// Cleaner runs one autonomous job at a time: cleaning a zone, or exploring
// the place to build or complete the map.
type Cleaner struct {
	Base    Base
	Pose    PoseSource
	Planner Planner
	Cfg     Config
	// Frontiers is needed for exploration only.
	Frontiers FrontierSource
	// Blocked marks points the robot must keep away from (no-go zones,
	// with their margin). The dock disc is added by the cleaner.
	Blocked func(mapping.Point) bool
	// Control puts the robot under control (Safe mode) before moving.
	Control func() error
	// FullMode, if set, switches Full mode on (true) or back to Safe
	// (false), to leave the dock.
	FullMode func(on bool) error
	// BatteryLow tells when to stop and go back to the dock.
	BatteryLow func() bool
	// Event signals "start", "done" and "stuck" (melodies).
	Event func(name string)
	// Publish is called on every status change and once a second.
	Publish func(Status)

	mu     sync.Mutex
	cancel context.CancelFunc
	status Status
}

// Status returns the current status.
func (c *Cleaner) Status() Status {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.status.State == "" {
		return Status{State: StateIdle}
	}
	return c.status
}

// Running tells whether a job is in progress.
func (c *Cleaner) Running() bool {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.cancel != nil
}

// Start launches the cleaning of area. With returnToDock, the robot seeks
// its dock at the end (or when the battery runs low).
func (c *Cleaner) Start(area Area, returnToDock bool) error {
	return c.launch(func(ctx context.Context) { c.run(ctx, area, returnToDock) })
}

// Explore launches an exploration: the robot drives to the frontiers of
// the map, one after the other, until none is left, then seeks its dock
// if returnToDock is set.
func (c *Cleaner) Explore(returnToDock bool) error {
	if c.Frontiers == nil {
		return errors.New("exploration needs a map")
	}
	return c.launch(func(ctx context.Context) { c.explore(ctx, returnToDock) })
}

// launch runs job in the background unless a job is already running.
func (c *Cleaner) launch(job func(context.Context)) error {
	c.mu.Lock()
	if c.cancel != nil {
		c.mu.Unlock()
		return errors.New("a job is already running")
	}
	ctx, cancel := context.WithCancel(context.Background())
	c.cancel = cancel
	c.mu.Unlock()
	go func() {
		defer func() {
			c.Base.SetMotors(robot.Motors{})
			c.Base.SetWheels(0, 0)
			c.mu.Lock()
			c.cancel = nil
			c.mu.Unlock()
		}()
		job(ctx)
	}()
	return nil
}

// Stop cancels the running job, if any. The wheels and motors stop.
func (c *Cleaner) Stop() {
	c.mu.Lock()
	if c.cancel != nil {
		c.cancel()
	}
	c.mu.Unlock()
}

func (c *Cleaner) set(s Status) {
	c.mu.Lock()
	c.status = s
	c.mu.Unlock()
	if c.Publish != nil {
		c.Publish(s)
	}
}

func (c *Cleaner) event(name string) {
	if c.Event != nil {
		c.Event(name)
	}
}

// blocked combines the no-go zones and the dock disc.
func (c *Cleaner) blocked(p mapping.Point) bool {
	if c.Cfg.DockRadius > 0 && math.Hypot(p.X-c.Cfg.Dock.X, p.Y-c.Cfg.Dock.Y) < c.Cfg.DockRadius {
		return true
	}
	return c.Blocked != nil && c.Blocked(p)
}

// begin takes control, plays the start melody, publishes st and backs out
// of the dock if the robot is on it. It returns an error when the robot
// cannot be controlled.
func (c *Cleaner) begin(ctx context.Context, st Status) error {
	if c.Control != nil {
		if err := c.Control(); err != nil {
			return err
		}
	}
	c.event("start")
	c.set(st)
	if s, err := c.Base.Sensors(); err == nil && s.ChargingSources&2 != 0 {
		// On the dock: back out straight before turning, as the Roomba does.
		// In Safe mode the Roomba stops after 2 cm (its front cliff sensors
		// see the dock ramp as a drop): back out in Full mode, then back to
		// Safe. Reversing loses no safety, the Roomba has no rear cliff sensor.
		if c.FullMode != nil {
			if err := c.FullMode(true); err != nil {
				return err
			}
		}
		c.Base.SetWheels(-150, -150)
		select {
		case <-ctx.Done():
		case <-time.After(2 * time.Second):
		}
		c.Base.SetWheels(0, 0)
		if c.FullMode != nil {
			if err := c.FullMode(false); err != nil {
				return err
			}
		}
	}
	return nil
}

func (c *Cleaner) run(ctx context.Context, area Area, returnToDock bool) {
	st := Status{Kind: KindClean, State: StateGoing, Zone: area.ID, Name: area.Name}
	finish := func(state, msg string) {
		st.State, st.Message = state, msg
		log.Printf("madroombad: cleaning %q: %s %s", area.Name, state, msg)
		c.set(st)
	}
	if err := c.begin(ctx, st); err != nil {
		finish(StateFailed, err.Error())
		return
	}

	lanes := Coverage(area.Polygon, c.Cfg.LaneSpacing, c.Cfg.RobotRadius)
	if len(lanes) == 0 {
		finish(StateFailed, "zone too narrow for the robot")
		return
	}
	skipped := 0
	for i, wp := range lanes {
		if c.BatteryLow != nil && c.BatteryLow() {
			c.Base.SetMotors(robot.Motors{})
			finish(StateDocking, "battery low")
			c.Base.Dock()
			return
		}
		err := c.goTo(ctx, wp, 0.05)
		switch {
		case ctx.Err() != nil:
			finish(StateCanceled, "")
			return
		case errors.Is(err, ErrStuck):
			c.event("stuck")
			finish(StateFailed, err.Error())
			return
		case err != nil: // bumped, unreachable or too slow: try the next point
			skipped++
			log.Printf("madroombad: cleaning %q: waypoint %d skipped: %v", area.Name, i+1, err)
		}
		if i == 0 { // reached the zone: start cleaning
			c.Base.SetMotors(robot.Motors{MainBrush: true, Vacuum: true, SideBrush: true})
			st.State = StateCleaning
		}
		st.Progress = float64(i+1) / float64(len(lanes))
		c.set(st)
	}
	c.Base.SetMotors(robot.Motors{})
	c.event("done")
	msg := ""
	if skipped > 0 {
		msg = fmt.Sprintf("%d of %d points skipped", skipped, len(lanes))
	}
	if returnToDock {
		finish(StateDocking, msg)
		c.Base.Dock()
		return
	}
	finish(StateDone, msg)
}

// explore drives to the nearest frontier of the map, again and again,
// while the lidar loop completes the map, until no reachable frontier is
// left, the time is up or the battery runs low. A frontier reached twice
// but still unknown behind is given up, an unreachable one at once.
func (c *Cleaner) explore(ctx context.Context, returnToDock bool) {
	st := Status{Kind: KindExplore, State: StateExploring, Name: "exploration"}
	finish := func(state, msg string) {
		st.State, st.Message = state, msg
		log.Printf("madroombad: exploration: %s %s", state, msg)
		c.set(st)
	}
	if err := c.begin(ctx, st); err != nil {
		finish(StateFailed, err.Error())
		return
	}
	tries := map[[2]int]int{} // per frontier, on a 25 cm grid
	key := func(p mapping.Point) [2]int { return [2]int{int(math.Round(p.X * 4)), int(math.Round(p.Y * 4))} }
	unreachable := 0
	deadline := time.Now().Add(c.Cfg.ExploreTime)
	for {
		if ctx.Err() != nil {
			finish(StateCanceled, "")
			return
		}
		if c.BatteryLow != nil && c.BatteryLow() {
			finish(StateDocking, "battery low")
			c.Base.Dock()
			return
		}
		if time.Now().After(deadline) {
			break
		}
		// nearest frontier not given up, outside no-go areas
		p := c.Pose.Pose()
		var target mapping.Point
		found, best := false, math.Inf(1)
		for _, f := range c.Frontiers.Frontiers(c.Cfg.FrontierCells) {
			if tries[key(f)] >= 2 || c.blocked(f) {
				continue
			}
			if d := math.Hypot(f.X-p.X, f.Y-p.Y); d < best {
				target, best, found = f, d, true
			}
		}
		if !found {
			break
		}
		tries[key(target)]++
		err := c.goTo(ctx, target, c.Cfg.FrontierReach)
		switch {
		case ctx.Err() != nil:
			finish(StateCanceled, "")
			return
		case errors.Is(err, ErrStuck):
			c.event("stuck")
			finish(StateFailed, err.Error())
			return
		case errors.Is(err, ErrUnreachable):
			// The robot has not moved: planning again would fail the same
			// way, and each plan costs a Pi Zero tens of milliseconds.
			tries[key(target)] = 2
			unreachable++
		case err == nil:
			st.Visited++
			c.set(st)
			// let the lidar sweep the new view before choosing the next frontier
			select {
			case <-ctx.Done():
			case <-time.After(time.Second):
			}
		}
	}
	if st.Visited == 0 && unreachable > 0 {
		// Usually a pose that no longer matches the map: the robot sees
		// itself walled in. Say so instead of a silent "0 frontiers".
		p := c.Pose.Pose()
		finish(StateFailed, fmt.Sprintf("no path to any of %d frontiers from (%.2f, %.2f): pose off the map?", unreachable, p.X, p.Y))
		if returnToDock {
			c.Base.Dock()
		}
		return
	}
	c.event("done")
	msg := fmt.Sprintf("%d frontiers reached", st.Visited)
	if unreachable > 0 {
		msg += fmt.Sprintf(", %d unreachable", unreachable)
	}
	if returnToDock {
		finish(StateDocking, msg)
		c.Base.Dock()
		return
	}
	finish(StateDone, msg)
}

// goTo plans a path to target and follows it, stopping within finalTol of
// the target.
func (c *Cleaner) goTo(ctx context.Context, target mapping.Point, finalTol float64) error {
	p := c.Pose.Pose()
	from := mapping.Point{X: p.X, Y: p.Y}
	path, ok := c.Planner.PlanPathNear(from, target, c.Cfg.RobotRadius+c.Cfg.Clearance, finalTol, c.blocked)
	if !ok {
		return ErrUnreachable
	}
	for i, wp := range path {
		tol := 0.08
		if i == len(path)-1 {
			tol = finalTol
		}
		if err := c.drive(ctx, wp, tol); err != nil {
			return err
		}
	}
	return nil
}

// drive steers toward wp until within tol, checking the safety sensors at
// every control period.
func (c *Cleaner) drive(ctx context.Context, wp mapping.Point, tol float64) error {
	p := c.Pose.Pose()
	// generous: straight-line time at a slow 0.1 m/s, plus time to turn
	limit := time.Duration((math.Hypot(wp.X-p.X, wp.Y-p.Y)/0.1+10)*1000) * time.Millisecond
	deadline := time.Now().Add(limit)
	tick := time.NewTicker(c.Cfg.Tick)
	defer tick.Stop()
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-tick.C:
		}
		p = c.Pose.Pose()
		if math.Hypot(wp.X-p.X, wp.Y-p.Y) < tol {
			return nil
		}
		if time.Now().After(deadline) {
			c.Base.SetWheels(0, 0)
			return ErrTimeout
		}
		if s, err := c.Base.Sensors(); err == nil {
			switch {
			case s.WheelDropLeft || s.WheelDropRight || s.Cliffs[0] || s.Cliffs[1] || s.Cliffs[2] || s.Cliffs[3] ||
				(s.OIMode != 2 && s.OIMode != 3):
				c.Base.SetWheels(0, 0)
				return ErrStuck
			case s.BumpLeft || s.BumpRight:
				c.backOff(ctx)
				return ErrBumped
			}
		}
		l, r := wheelsFor(p, wp, c.Cfg.Cruise, c.Cfg.TurnSpeed)
		if err := c.Base.SetWheels(l, r); err != nil {
			return err
		}
	}
}

// backOff reverses about 5 cm after a bumper hit.
func (c *Cleaner) backOff(ctx context.Context) {
	c.Base.SetWheels(-100, -100)
	select {
	case <-ctx.Done():
	case <-time.After(500 * time.Millisecond):
	}
	c.Base.SetWheels(0, 0)
}
