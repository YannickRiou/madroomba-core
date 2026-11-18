package main

import (
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"sync"

	"github.com/YannickRiou/madroomba-core/internal/robot"
	"github.com/YannickRiou/madroomba-core/internal/server"
)

// soundEvents are the events that can play a melody, with their defaults
// (notation of robot.ParseNotes; "" is silent).
var soundEvents = map[string]string{
	"start":       "C5/12 E5/12 G5/12 C6/24", // a cleaning job starts
	"done":        "C6/12 G5/12 E5/12 C5/24", // a cleaning job is finished
	"stuck":       "A5/8 R/8 A5/8 R/8 A5/24", // a job stopped on a cliff or a wheel drop
	"low_battery": "E5/16 C5/32",             // the battery went under 15 % while running
}

// soundStore keeps the melody of each event in a JSON file and plays them.
// A melody needs Safe or Full mode: events only play when the robot is
// already under control, so a sound never interrupts charging or a
// built-in cleaning cycle.
type soundStore struct {
	path    string
	play    func([]robot.Note) error
	canPlay func() bool

	mu     sync.Mutex
	melody map[string]string
}

func (s *soundStore) load() {
	m := map[string]string{}
	for k, v := range soundEvents {
		m[k] = v
	}
	if data, err := os.ReadFile(s.path); err == nil {
		var saved map[string]string
		if json.Unmarshal(data, &saved) == nil && validateSounds(saved) == nil {
			for k, v := range saved {
				m[k] = v
			}
		}
	}
	s.mu.Lock()
	s.melody = m
	s.mu.Unlock()
}

func validateSounds(m map[string]string) error {
	for k, v := range m {
		if _, ok := soundEvents[k]; !ok {
			return fmt.Errorf("unknown event %q", k)
		}
		if v == "" {
			continue
		}
		if _, err := robot.ParseNotes(v); err != nil {
			return fmt.Errorf("%s: %w", k, err)
		}
	}
	return nil
}

func (s *soundStore) get() map[string]string {
	s.mu.Lock()
	defer s.mu.Unlock()
	out := map[string]string{}
	for k, v := range s.melody {
		out[k] = v
	}
	return out
}

func (s *soundStore) set(m map[string]string) error {
	if err := validateSounds(m); err != nil {
		return err
	}
	all := s.get()
	for k, v := range m {
		all[k] = v
	}
	data, _ := json.MarshalIndent(all, "", "  ")
	if err := os.MkdirAll(filepath.Dir(s.path), 0755); err != nil {
		return err
	}
	if err := os.WriteFile(s.path, data, 0644); err != nil {
		return err
	}
	s.mu.Lock()
	s.melody = all
	s.mu.Unlock()
	return nil
}

// event plays the melody of an event, if any and if the robot is under
// control.
func (s *soundStore) event(name string) {
	text := s.get()[name]
	if text == "" || !s.canPlay() {
		return
	}
	notes, err := robot.ParseNotes(text)
	if err == nil {
		err = s.play(notes)
	}
	if err != nil {
		log.Printf("madroombad: sound %s: %v", name, err)
	}
}

// register serves GET /api/sounds (event -> melody) and POST /api/sounds
// (some or all events; "" silences one).
func (s *soundStore) register(srv *server.Server) {
	srv.Handle("/api/sounds", func(w http.ResponseWriter, r *http.Request) {
		if r.Method == http.MethodPost {
			var m map[string]string
			if err := json.NewDecoder(r.Body).Decode(&m); err != nil {
				http.Error(w, err.Error(), http.StatusBadRequest)
				return
			}
			if err := s.set(m); err != nil {
				http.Error(w, err.Error(), http.StatusBadRequest)
				return
			}
		}
		writeJSON(w, s.get())
	})
}
