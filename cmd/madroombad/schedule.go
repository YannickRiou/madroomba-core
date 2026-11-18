package main

import (
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"regexp"
	"sync"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/server"
)

// scheduleEntry starts an action at a given time on some week days. Times
// are in the Pi's local time.
type scheduleEntry struct {
	Days    []int  `json:"days"` // 0 = Sunday ... 6 = Saturday
	Time    string `json:"time"` // "HH:MM"
	Action  string `json:"action"`
	Enabled bool   `json:"enabled"`
}

// scheduleActions are the actions a schedule may start, besides
// "zone:<id>" which cleans a zone.
var scheduleActions = map[string]bool{"clean": true, "spot": true, "max": true, "dock": true, "explore": true}

var zoneAction = regexp.MustCompile(`^zone:[a-z0-9-]{1,32}$`)

func validateSchedule(es []scheduleEntry) error {
	if len(es) > 50 {
		return fmt.Errorf("at most 50 entries")
	}
	for i, e := range es {
		if _, err := time.Parse("15:04", e.Time); err != nil {
			return fmt.Errorf("entry %d: time %q, want HH:MM", i+1, e.Time)
		}
		if !scheduleActions[e.Action] && !zoneAction.MatchString(e.Action) {
			return fmt.Errorf("entry %d: unknown action %q", i+1, e.Action)
		}
		if len(e.Days) == 0 {
			return fmt.Errorf("entry %d: no day selected", i+1)
		}
		for _, d := range e.Days {
			if d < 0 || d > 6 {
				return fmt.Errorf("entry %d: day %d, want 0 (Sunday) to 6", i+1, d)
			}
		}
	}
	return nil
}

// matches tells whether e fires at the minute of t.
func (e scheduleEntry) matches(t time.Time) bool {
	if !e.Enabled || t.Format("15:04") != e.Time {
		return false
	}
	for _, d := range e.Days {
		if time.Weekday(d) == t.Weekday() {
			return true
		}
	}
	return false
}

// nextRun returns the first minute after now at which an entry fires, and
// that entry, looking one week ahead.
func nextRun(es []scheduleEntry, now time.Time) (time.Time, scheduleEntry, bool) {
	t := now.Truncate(time.Minute).Add(time.Minute)
	for end := t.Add(7 * 24 * time.Hour); t.Before(end); t = t.Add(time.Minute) {
		for _, e := range es {
			if e.matches(t) {
				return t, e, true
			}
		}
	}
	return time.Time{}, scheduleEntry{}, false
}

// scheduler persists the entries in path and runs them.
type scheduler struct {
	mu      sync.Mutex
	path    string
	entries []scheduleEntry
	run     func(action string)
}

func (s *scheduler) load() {
	data, err := os.ReadFile(s.path)
	if err != nil {
		if !os.IsNotExist(err) {
			log.Printf("madroombad: schedule: %v", err)
		}
		return
	}
	var es []scheduleEntry
	if err := json.Unmarshal(data, &es); err != nil || validateSchedule(es) != nil {
		log.Printf("madroombad: schedule: ignoring invalid %s", s.path)
		return
	}
	s.entries = es
}

func (s *scheduler) set(es []scheduleEntry) error {
	if err := validateSchedule(es); err != nil {
		return err
	}
	data, _ := json.MarshalIndent(es, "", "  ")
	if err := os.MkdirAll(filepath.Dir(s.path), 0755); err != nil {
		return err
	}
	if err := os.WriteFile(s.path, data, 0644); err != nil {
		return err
	}
	s.mu.Lock()
	s.entries = es
	s.mu.Unlock()
	return nil
}

// loop checks the entries every few seconds and runs each due one once.
func (s *scheduler) loop() {
	last := ""
	for now := range time.Tick(10 * time.Second) {
		minute := now.Format("2006-01-02 15:04")
		if minute == last {
			continue
		}
		last = minute
		s.mu.Lock()
		es := s.entries
		s.mu.Unlock()
		for _, e := range es {
			if e.matches(now) {
				go s.run(e.Action)
			}
		}
	}
}

func (s *scheduler) status() map[string]interface{} {
	s.mu.Lock()
	es := s.entries
	s.mu.Unlock()
	if es == nil {
		es = []scheduleEntry{}
	}
	now := time.Now()
	out := map[string]interface{}{"entries": es, "now": now.Format("2006-01-02 15:04")}
	if t, e, ok := nextRun(es, now); ok {
		out["next"] = map[string]interface{}{"at": t.Format("2006-01-02 15:04"), "weekday": int(t.Weekday()), "action": e.Action}
	}
	return out
}

func (s *scheduler) register(srv *server.Server) {
	srv.Handle("/api/schedule", func(w http.ResponseWriter, r *http.Request) {
		if r.Method == http.MethodPost {
			var es []scheduleEntry
			if err := json.NewDecoder(r.Body).Decode(&es); err != nil {
				http.Error(w, err.Error(), http.StatusBadRequest)
				return
			}
			if err := s.set(es); err != nil {
				http.Error(w, err.Error(), http.StatusBadRequest)
				return
			}
		}
		writeJSON(w, s.status())
	})
}
