// Package battery records the battery history of the robot: charge,
// voltage, current and charging state over the last days, for the page's
// charge/discharge chart.
//
// The daemon feeds every battery reading (every few seconds) to Add. The
// recorder averages them over Interval, keeps the samples in memory and
// appends each one to a CSV file, so the history survives restarts with
// one small write every Interval (gentle on the SD card).
package battery

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/robot"
)

// Sample is one point of the history.
type Sample struct {
	Time      time.Time
	ChargeMAh uint16
	Capacity  uint16
	VoltageMV uint16
	CurrentMA int16 // mean over the interval, negative while discharging
	Charging  byte  // OI charging state at the end of the interval
	TempC     int8
}

// Percent is the state of charge, 0 to 100.
func (s Sample) Percent() float64 {
	if s.Capacity == 0 {
		return 0
	}
	return math.Min(100, 100*float64(s.ChargeMAh)/float64(s.Capacity))
}

// Recorder keeps the history. Set Path, Interval and Keep before Load.
type Recorder struct {
	Path     string        // CSV file, "" to keep the history in memory only
	Interval time.Duration // one sample per Interval
	Keep     time.Duration // history length

	mu      sync.Mutex
	samples []Sample
	sum     int // current accumulated over the interval
	n       int
	start   time.Time
}

// Load reads the CSV file, drops samples older than Keep and rewrites the
// file when it holds too many old lines.
func (r *Recorder) Load() error {
	if r.Path == "" {
		return nil
	}
	f, err := os.Open(r.Path)
	if os.IsNotExist(err) {
		return nil
	}
	if err != nil {
		return err
	}
	defer f.Close()
	cut := time.Now().Add(-r.Keep)
	lines, kept := 0, []Sample{}
	sc := bufio.NewScanner(f)
	for sc.Scan() {
		lines++
		if s, ok := parseLine(sc.Text()); ok && s.Time.After(cut) {
			kept = append(kept, s)
		}
	}
	r.mu.Lock()
	r.samples = kept
	r.mu.Unlock()
	if lines > len(kept)+len(kept)/2+100 {
		return r.rewrite(kept)
	}
	return sc.Err()
}

// Add feeds one battery reading taken at t.
func (r *Recorder) Add(b robot.Battery, t time.Time) {
	r.mu.Lock()
	if r.n == 0 {
		r.start = t
	}
	r.sum += int(b.CurrentMA)
	r.n++
	if t.Sub(r.start) < r.Interval {
		r.mu.Unlock()
		return
	}
	s := Sample{
		Time: t, ChargeMAh: b.ChargeMAh, Capacity: b.CapacityMAh, VoltageMV: b.VoltageMV,
		CurrentMA: int16(r.sum / r.n), Charging: b.Charging, TempC: b.TempC,
	}
	r.sum, r.n = 0, 0
	r.samples = append(r.samples, s)
	cut := t.Add(-r.Keep)
	i := 0
	for i < len(r.samples) && r.samples[i].Time.Before(cut) {
		i++
	}
	r.samples = r.samples[i:]
	r.mu.Unlock()
	r.appendLine(s)
}

// Since returns the samples newer than t, averaged down to at most max
// points (0 means no limit).
func (r *Recorder) Since(t time.Time, max int) []Sample {
	r.mu.Lock()
	var in []Sample
	for _, s := range r.samples {
		if s.Time.After(t) {
			in = append(in, s)
		}
	}
	r.mu.Unlock()
	if max <= 0 || len(in) <= max {
		return in
	}
	step := (len(in) + max - 1) / max
	out := make([]Sample, 0, max)
	for i := 0; i < len(in); i += step {
		end := i + step
		if end > len(in) {
			end = len(in)
		}
		// mean current over the bucket, the rest from its last sample
		sum := 0
		for _, s := range in[i:end] {
			sum += int(s.CurrentMA)
		}
		s := in[end-1]
		s.CurrentMA = int16(sum / (end - i))
		out = append(out, s)
	}
	return out
}

// CSV: unix seconds, charge mAh, capacity mAh, mV, mA, charging state, °C.
func formatLine(s Sample) string {
	return fmt.Sprintf("%d,%d,%d,%d,%d,%d,%d\n", s.Time.Unix(), s.ChargeMAh, s.Capacity, s.VoltageMV, s.CurrentMA, s.Charging, s.TempC)
}

func parseLine(line string) (Sample, bool) {
	f := strings.Split(strings.TrimSpace(line), ",")
	if len(f) != 7 {
		return Sample{}, false
	}
	var v [7]int64
	for i, x := range f {
		n, err := strconv.ParseInt(x, 10, 64)
		if err != nil {
			return Sample{}, false
		}
		v[i] = n
	}
	return Sample{
		Time: time.Unix(v[0], 0), ChargeMAh: uint16(v[1]), Capacity: uint16(v[2]),
		VoltageMV: uint16(v[3]), CurrentMA: int16(v[4]), Charging: byte(v[5]), TempC: int8(v[6]),
	}, true
}

func (r *Recorder) appendLine(s Sample) {
	if r.Path == "" {
		return
	}
	os.MkdirAll(filepath.Dir(r.Path), 0755)
	f, err := os.OpenFile(r.Path, os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0644)
	if err != nil {
		return
	}
	f.WriteString(formatLine(s))
	f.Close()
}

func (r *Recorder) rewrite(ss []Sample) error {
	var b strings.Builder
	for _, s := range ss {
		b.WriteString(formatLine(s))
	}
	tmp := r.Path + ".tmp"
	if err := os.WriteFile(tmp, []byte(b.String()), 0644); err != nil {
		return err
	}
	return os.Rename(tmp, r.Path)
}
