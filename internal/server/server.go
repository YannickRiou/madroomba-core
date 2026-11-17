// Package server exposes the robot over HTTP: an embedded single-page UI,
// a Server-Sent Events telemetry stream (stdlib only, no websocket
// dependency) and JSON control endpoints.
package server

import (
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"strings"
	"sync"

	"github.com/YannickRiou/madroomba-core/internal/ui"
)

const logKeep = 200 // log lines kept for late joiners

// Command is a UI action forwarded to the daemon.
type Command struct {
	Action string `json:"action"` // stop | clean | dock | safe | reset_map | reset_pose
}

// DriveRequest carries wheel speeds in mm/s.
type DriveRequest struct {
	Left  int16 `json:"left"`
	Right int16 `json:"right"`
}

// Server is the HTTP facade. Set the callbacks before calling Run.
type Server struct {
	OnDrive   func(DriveRequest) error
	OnCommand func(Command) error
	// Camera, if non-nil, handles GET /camera (MJPEG).
	Camera http.Handler
	// MapPNG, if non-nil, returns the current occupancy grid as PNG for
	// GET /api/map.png (handy for scripts and archiving).
	MapPNG func() []byte
	// BaseURL, if set, is where the shared page finds the dock daemon
	// (basestationd), e.g. "http://madserver:8081".
	BaseURL string

	mu     sync.Mutex
	subs   map[chan event]struct{}
	last   map[string][]byte // last payload per event name, replayed on connect
	hasCam bool
	logs   []string
}

type event struct {
	name string
	data []byte
}

// New creates an empty server.
func New() *Server {
	return &Server{
		subs: make(map[chan event]struct{}),
		last: make(map[string][]byte),
	}
}

// Publish sends a named SSE event with a JSON-encoded payload to all
// connected clients, and caches it for late joiners.
func (s *Server) Publish(name string, payload interface{}) {
	data, err := json.Marshal(payload)
	if err != nil {
		log.Printf("server: marshal %s: %v", name, err)
		return
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	s.last[name] = data
	for ch := range s.subs {
		select {
		case ch <- event{name, data}:
		default: // slow client: drop
		}
	}
}

// Write makes the server a log sink (log.SetOutput): lines are kept in a
// small ring and pushed live to the UI console as "log" events. It must not
// log itself.
func (s *Server) Write(p []byte) (int, error) {
	for _, line := range strings.Split(strings.TrimRight(string(p), "\n"), "\n") {
		data, _ := json.Marshal(line)
		s.mu.Lock()
		s.logs = append(s.logs, line)
		if len(s.logs) > logKeep {
			s.logs = s.logs[len(s.logs)-logKeep:]
		}
		for ch := range s.subs {
			select {
			case ch <- event{"log", data}:
			default: // slow client: drop
			}
		}
		s.mu.Unlock()
	}
	return len(p), nil
}

func (s *Server) handleEvents(w http.ResponseWriter, r *http.Request) {
	fl, ok := w.(http.Flusher)
	if !ok {
		http.Error(w, "streaming unsupported", http.StatusInternalServerError)
		return
	}
	w.Header().Set("Content-Type", "text/event-stream")
	w.Header().Set("Cache-Control", "no-cache")
	w.Header().Set("Connection", "keep-alive")

	ch := make(chan event, 16)
	s.mu.Lock()
	s.subs[ch] = struct{}{}
	// Replay the last known payloads so the UI is populated instantly.
	replay := make([]event, 0, len(s.last))
	for name, data := range s.last {
		replay = append(replay, event{name, data})
	}
	hist, _ := json.Marshal(s.logs)
	replay = append(replay, event{"logs", hist})
	s.mu.Unlock()
	defer func() {
		s.mu.Lock()
		delete(s.subs, ch)
		s.mu.Unlock()
	}()

	for _, ev := range replay {
		fmt.Fprintf(w, "event: %s\ndata: %s\n\n", ev.name, ev.data)
	}
	fl.Flush()

	for {
		select {
		case <-r.Context().Done():
			return
		case ev := <-ch:
			fmt.Fprintf(w, "event: %s\ndata: %s\n\n", ev.name, ev.data)
			fl.Flush()
		}
	}
}

func (s *Server) handleDrive(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "POST only", http.StatusMethodNotAllowed)
		return
	}
	var req DriveRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	if req.Left < -500 || req.Left > 500 || req.Right < -500 || req.Right > 500 {
		http.Error(w, "wheel speed out of range", http.StatusBadRequest)
		return
	}
	if err := s.OnDrive(req); err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}
	w.WriteHeader(http.StatusNoContent)
}

func (s *Server) handleCommand(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "POST only", http.StatusMethodNotAllowed)
		return
	}
	var cmd Command
	if err := json.NewDecoder(r.Body).Decode(&cmd); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	if err := s.OnCommand(cmd); err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}
	w.WriteHeader(http.StatusNoContent)
}

// Run blocks, serving on addr.
func (s *Server) Run(addr string) error {
	mux := http.NewServeMux()

	self := ""
	var base *string
	if s.BaseURL != "" {
		if err := ui.Proxy(mux, "/proxy/base", s.BaseURL); err != nil {
			return err
		}
		p := "/proxy/base"
		base = &p
	}
	ui.Mount(mux, &self, base)
	mux.HandleFunc("/api/events", s.handleEvents)
	mux.HandleFunc("/api/drive", s.handleDrive)
	mux.HandleFunc("/api/cmd", s.handleCommand)
	mux.HandleFunc("/api/map.png", func(w http.ResponseWriter, r *http.Request) {
		if s.MapPNG == nil {
			http.Error(w, "no map", http.StatusServiceUnavailable)
			return
		}
		w.Header().Set("Content-Type", "image/png")
		w.Write(s.MapPNG())
	})
	mux.HandleFunc("/camera", func(w http.ResponseWriter, r *http.Request) {
		if s.Camera == nil {
			http.Error(w, "no camera configured", http.StatusServiceUnavailable)
			return
		}
		s.Camera.ServeHTTP(w, r)
	})

	log.Printf("server: listening on http://%s", addr)
	return http.ListenAndServe(addr, mux)
}
