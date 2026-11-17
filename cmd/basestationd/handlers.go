package main

import (
	"encoding/json"
	"log"
	"net/http"
	"os"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/camera"
)

func writeJSON(w http.ResponseWriter, v interface{}) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(v)
}

// postOnly rejects anything but POST; it returns true when the request was
// rejected.
func postOnly(w http.ResponseWriter, r *http.Request) bool {
	if r.Method != http.MethodPost {
		http.Error(w, "POST only", http.StatusMethodNotAllowed)
		return true
	}
	return false
}

// routes registers the API. The page itself ("/", "/api/config" and the
// robot proxy) is mounted by main through the ui package.
func (a *app) routes(mux *http.ServeMux, det *camera.MotionDetector) {
	mux.HandleFunc("/camera", a.serveCamera)
	mux.HandleFunc("/api/status", func(w http.ResponseWriter, r *http.Request) {
		score, last := det.Status()
		a.mu.Lock()
		dock := map[string]interface{}{
			"state": a.dState, "has_docked": a.refs.Docked != nil, "has_empty": a.refs.Empty != nil,
			"d_docked": a.dD, "d_empty": a.dE,
		}
		a.mu.Unlock()
		resp := map[string]interface{}{
			"motion": !last.IsZero() && time.Since(last) < 3*time.Second,
			"score":  score,
			"uptime": time.Since(a.start).Round(time.Second).String(),
			"dock":   dock,
		}
		if !last.IsZero() {
			resp["last_motion"] = last.Format("15:04:05")
		}
		writeJSON(w, resp)
	})
	mux.HandleFunc("/api/logs", func(w http.ResponseWriter, r *http.Request) {
		writeJSON(w, a.logs.Lines())
	})

	// Snapshots. GET /api/snapshot.jpg is the live frame (?download=1 makes it
	// a download); POST /api/snap stores one; /snaps/<name> serves stored ones.
	mux.HandleFunc("/api/snapshot.jpg", func(w http.ResponseWriter, r *http.Request) {
		f := a.latest()
		if f == nil {
			http.Error(w, "no frame yet", http.StatusServiceUnavailable)
			return
		}
		w.Header().Set("Content-Type", "image/jpeg")
		if r.URL.Query().Get("download") != "" {
			w.Header().Set("Content-Disposition", `attachment; filename="dock-`+time.Now().Format("20060102-150405")+`.jpg"`)
		}
		w.Write(f)
	})
	mux.HandleFunc("/api/snap", func(w http.ResponseWriter, r *http.Request) {
		if postOnly(w, r) {
			return
		}
		f := a.latest()
		if f == nil {
			http.Error(w, "no frame yet", http.StatusServiceUnavailable)
			return
		}
		name, err := a.snaps.save(f, "manual")
		if err != nil {
			http.Error(w, err.Error(), http.StatusInternalServerError)
			return
		}
		log.Printf("basestationd: snapshot saved %s", name)
		writeJSON(w, map[string]string{"name": name})
	})
	mux.HandleFunc("/api/snaps", func(w http.ResponseWriter, r *http.Request) {
		writeJSON(w, a.snaps.list())
	})
	mux.HandleFunc("/api/snaps/delete", func(w http.ResponseWriter, r *http.Request) {
		if postOnly(w, r) {
			return
		}
		if r.URL.Query().Get("all") != "" {
			a.snaps.deleteAll()
			log.Printf("basestationd: all snapshots deleted")
		} else {
			name := r.URL.Query().Get("name")
			if err := a.snaps.delete(name); err != nil {
				http.Error(w, err.Error(), http.StatusNotFound)
				return
			}
			log.Printf("basestationd: snapshot deleted %s", name)
		}
		w.WriteHeader(http.StatusNoContent)
	})
	mux.Handle("/snaps/", http.StripPrefix("/snaps/", http.FileServer(http.Dir(a.snaps.dir))))

	// Camera settings: GET returns them, POST validates, applies (restarts
	// the camera) and persists them.
	mux.HandleFunc("/api/camera/settings", func(w http.ResponseWriter, r *http.Request) {
		if r.Method == http.MethodPost {
			s := defaultSettings()
			if err := json.NewDecoder(r.Body).Decode(&s); err != nil {
				http.Error(w, err.Error(), http.StatusBadRequest)
				return
			}
			if err := s.valid(); err != nil {
				http.Error(w, err.Error(), http.StatusBadRequest)
				return
			}
			if err := a.restartCamera(s); err != nil {
				http.Error(w, err.Error(), http.StatusInternalServerError)
				return
			}
			saveJSON(a.setPath, s)
			log.Printf("basestationd: camera settings applied: %v", s.args())
		}
		a.mu.Lock()
		s := a.set
		a.mu.Unlock()
		writeJSON(w, s)
	})

	// Dock presence: POST /api/dock/learn?state=docked|empty stores the current
	// image as a reference; POST /api/dock/reset forgets both.
	mux.HandleFunc("/api/dock/learn", func(w http.ResponseWriter, r *http.Request) {
		if postOnly(w, r) {
			return
		}
		st := r.URL.Query().Get("state")
		a.mu.Lock()
		g := append([]float64(nil), a.grid...)
		switch {
		case len(g) == 0:
			a.mu.Unlock()
			http.Error(w, "no frame yet", http.StatusServiceUnavailable)
			return
		case st == "docked":
			a.refs.Docked = g
		case st == "empty":
			a.refs.Empty = g
		default:
			a.mu.Unlock()
			http.Error(w, "state must be docked or empty", http.StatusBadRequest)
			return
		}
		refs := a.refs
		a.mu.Unlock()
		saveJSON(a.refPath, refs)
		log.Printf("basestationd: learned reference %q", st)
		w.WriteHeader(http.StatusNoContent)
	})
	mux.HandleFunc("/api/dock/reset", func(w http.ResponseWriter, r *http.Request) {
		if postOnly(w, r) {
			return
		}
		a.mu.Lock()
		a.refs, a.dD, a.dE, a.dState, a.dPending = dockRefs{}, 0, 0, "unknown", ""
		a.mu.Unlock()
		os.Remove(a.refPath)
		log.Printf("basestationd: dock references cleared")
		w.WriteHeader(http.StatusNoContent)
	})
}
