package battery

import (
	"path/filepath"
	"testing"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/robot"
)

func TestRecorderAveragesAndPersists(t *testing.T) {
	path := filepath.Join(t.TempDir(), "battery.csv")
	r := &Recorder{Path: path, Interval: 30 * time.Second, Keep: time.Hour}
	t0 := time.Now().Add(-48 * time.Hour).Truncate(time.Second) // older than an hour
	// 16 readings 2 s apart: -100 mA then -300 mA, one sample at 30 s
	for i := 0; i <= 15; i++ {
		ma := int16(-100)
		if i >= 8 {
			ma = -300
		}
		r.Add(robot.Battery{ChargeMAh: 2000, CapacityMAh: 2600, VoltageMV: 15000, CurrentMA: ma, Charging: 0}, t0.Add(time.Duration(2*i)*time.Second))
	}
	got := r.Since(time.Time{}, 0)
	if len(got) != 1 {
		t.Fatalf("got %d samples, want 1", len(got))
	}
	if got[0].CurrentMA != -200 {
		t.Errorf("mean current = %d, want -200", got[0].CurrentMA)
	}
	if p := got[0].Percent(); p < 76.9 || p > 77 {
		t.Errorf("percent = %.2f", p)
	}

	// a new recorder reads the file back, but only within Keep
	r2 := &Recorder{Path: path, Interval: 30 * time.Second, Keep: 100 * 365 * 24 * time.Hour}
	if err := r2.Load(); err != nil {
		t.Fatal(err)
	}
	if s := r2.Since(time.Time{}, 0); len(s) != 1 || s[0] != got[0] {
		t.Errorf("reloaded %v, want %v", s, got)
	}
	r3 := &Recorder{Path: path, Interval: 30 * time.Second, Keep: time.Hour}
	r3.Load()
	if s := r3.Since(time.Time{}, 0); len(s) != 0 {
		t.Errorf("old sample kept: %v", s)
	}
}

func TestSinceDownsamples(t *testing.T) {
	r := &Recorder{Interval: time.Second, Keep: 24 * time.Hour}
	t0 := time.Now().Add(-time.Hour)
	for i := 0; i < 1000; i++ {
		r.samples = append(r.samples, Sample{Time: t0.Add(time.Duration(i) * time.Second), CurrentMA: int16(i % 2 * 100)})
	}
	s := r.Since(time.Time{}, 100)
	if len(s) != 100 {
		t.Fatalf("got %d points, want 100", len(s))
	}
	if s[0].CurrentMA != 50 {
		t.Errorf("bucket mean = %d, want 50", s[0].CurrentMA)
	}
}
