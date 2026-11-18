package battery

import (
	"math"
	"path/filepath"
	"testing"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/robot"
)

func TestLifetimeCounts(t *testing.T) {
	path := filepath.Join(t.TempDir(), "battery-lifetime.json")
	l := &Lifetime{Path: path}
	t0 := time.Now()
	at := func(s float64) time.Time { return t0.Add(time.Duration(s * float64(time.Second))) }
	// one hour of discharge at -1300 mA, readings every 2 s: 1300 mAh,
	// half a cycle of a 2600 mAh battery
	for s := 0.0; s <= 3600; s += 2 {
		l.Add(robot.Battery{CurrentMA: -1300, CapacityMAh: 2600}, at(s))
	}
	// a 30 minute gap (daemon stopped) is not integrated
	// then two charge sessions of 6 minutes at +1000 mA
	for _, start := range []float64{5400, 9000} {
		for s := start; s <= start+360; s += 2 {
			l.Add(robot.Battery{CurrentMA: 1000, CapacityMAh: 2600, Charging: 2}, at(s))
		}
		l.Add(robot.Battery{CurrentMA: 0, CapacityMAh: 2600, Charging: 0}, at(start+362))
	}
	got := l.Get()
	if math.Abs(got.DischargedMAh-1300) > 1 {
		t.Errorf("discharged = %.1f mAh, want 1300", got.DischargedMAh)
	}
	if math.Abs(got.EquivalentCycles-0.5) > 0.001 {
		t.Errorf("cycles = %.4f, want 0.5", got.EquivalentCycles)
	}
	if math.Abs(got.ChargedMAh-200) > 1 { // 2 x 6 min at 1 A
		t.Errorf("charged = %.1f mAh, want 200", got.ChargedMAh)
	}
	if got.ChargeSessions != 2 {
		t.Errorf("sessions = %d, want 2", got.ChargeSessions)
	}

	// a rated capacity overrides the robot's estimate for the cycles
	n := &Lifetime{NominalMAh: 13000}
	for s := 0.0; s <= 3600; s += 2 {
		n.Add(robot.Battery{CurrentMA: -1300, CapacityMAh: 2600}, at(s))
	}
	if c := n.Get(); math.Abs(c.EquivalentCycles-0.1) > 0.001 || c.CapacityMAh != 13000 {
		t.Errorf("with a 13000 mAh pack: %+v, want 0.1 cycle", c)
	}

	// the counters survive a restart
	l2 := &Lifetime{Path: path}
	if err := l2.Load(); err != nil {
		t.Fatal(err)
	}
	if l2.Get().ChargeSessions != 2 || l2.Get().Since.IsZero() {
		t.Errorf("reloaded %+v", l2.Get())
	}
}
