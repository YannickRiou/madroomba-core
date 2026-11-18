package battery

import (
	"encoding/json"
	"os"
	"path/filepath"
	"sync"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/robot"
)

// Totals are the lifetime battery counters. They are never pruned.
type Totals struct {
	Since            time.Time `json:"since"`             // first reading ever recorded
	Updated          time.Time `json:"updated"`           // last reading
	DischargedMAh    float64   `json:"discharged_mah"`    // charge drawn from the battery
	ChargedMAh       float64   `json:"charged_mah"`       // charge put into the battery
	EquivalentCycles float64   `json:"equivalent_cycles"` // discharged charge divided by the capacity at the time
	ChargeSessions   int       `json:"charge_sessions"`   // times charging started
	CapacityMAh      float64   `json:"capacity_mah"`      // capacity used for the cycles (see Lifetime.NominalMAh)
}

// Lifetime integrates the battery current over time into Totals and keeps
// them in a JSON file at Path. The battery current is read every few
// seconds; gaps longer than maxGap (daemon stopped, robot off) are not
// integrated rather than guessed.
type Lifetime struct {
	Path string
	// NominalMAh is the rated capacity of the battery pack, used for the
	// equivalent cycles. 0 uses the capacity the Roomba reports, which is
	// its own estimate and can be far from the truth with a replacement
	// pack.
	NominalMAh float64

	mu       sync.Mutex
	t        Totals
	last     time.Time
	charging bool
	saved    time.Time
}

const (
	maxGap    = 10 * time.Second
	saveEvery = time.Minute
)

// Load reads the counters. A missing file starts them from zero.
func (l *Lifetime) Load() error {
	data, err := os.ReadFile(l.Path)
	if os.IsNotExist(err) {
		return nil
	}
	if err != nil {
		return err
	}
	l.mu.Lock()
	defer l.mu.Unlock()
	return json.Unmarshal(data, &l.t)
}

// isCharging tells whether an OI charging state means current flows in.
func isCharging(state byte) bool { return state >= 1 && state <= 3 }

// Add feeds one battery reading taken at t.
func (l *Lifetime) Add(b robot.Battery, t time.Time) {
	l.mu.Lock()
	if l.t.Since.IsZero() {
		l.t.Since = t
	}
	capacity := l.NominalMAh
	if capacity <= 0 {
		capacity = float64(b.CapacityMAh)
	}
	l.t.CapacityMAh = capacity
	if dt := t.Sub(l.last); !l.last.IsZero() && dt > 0 && dt <= maxGap {
		mah := float64(b.CurrentMA) * dt.Hours()
		if mah < 0 {
			l.t.DischargedMAh -= mah
			if capacity > 0 {
				l.t.EquivalentCycles -= mah / capacity
			}
		} else {
			l.t.ChargedMAh += mah
		}
	}
	session := isCharging(b.Charging) && !l.charging && !l.last.IsZero()
	if session {
		l.t.ChargeSessions++
	}
	l.charging = isCharging(b.Charging)
	l.last, l.t.Updated = t, t
	save := l.Path != "" && (session || t.Sub(l.saved) >= saveEvery)
	snapshot := l.t
	if save {
		l.saved = t
	}
	l.mu.Unlock()
	if save {
		writeAtomic(l.Path, snapshot)
	}
}

// Get returns the counters.
func (l *Lifetime) Get() Totals {
	l.mu.Lock()
	defer l.mu.Unlock()
	return l.t
}

// writeAtomic writes the counters to a temporary file and renames it, so
// a power cut never leaves a half-written file.
func writeAtomic(path string, t Totals) error {
	data, _ := json.MarshalIndent(t, "", "  ")
	if err := os.MkdirAll(filepath.Dir(path), 0755); err != nil {
		return err
	}
	tmp := path + ".tmp"
	if err := os.WriteFile(tmp, data, 0644); err != nil {
		return err
	}
	return os.Rename(tmp, path)
}
