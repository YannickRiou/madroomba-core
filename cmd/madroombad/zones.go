package main

import (
	"fmt"
	"io"
	"net/http"

	"github.com/YannickRiou/madroomba-core/internal/nav"
	"github.com/YannickRiou/madroomba-core/internal/zones"
)

// registerZones serves the zones and the zone cleaning jobs:
//
//	GET  /api/zones                  the zones of the current map
//	POST /api/zones                  replace them (JSON array), returns them
//	POST /api/clean?zone=<id>&dock=1 clean a zone, then seek the dock
//	POST /api/explore?dock=1         explore to build or complete the map, then seek the dock
//	POST /api/clean/stop             stop the running job (cleaning or exploration)
//
// The zones are also published as the "zones" event whenever they change,
// so every open page stays in sync; the job status is the "job" event.
func (a *app) registerZones() {
	a.srv.Handle("/api/zones", func(w http.ResponseWriter, r *http.Request) {
		if r.Method == http.MethodPost {
			body, err := io.ReadAll(io.LimitReader(r.Body, 1<<20))
			if err == nil {
				err = a.zones.SetJSON(body, true)
			}
			if err != nil {
				http.Error(w, err.Error(), http.StatusBadRequest)
				return
			}
			a.publishZones()
		}
		writeJSON(w, a.zones.List())
	})
	a.srv.Handle("/api/clean", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			http.Error(w, "POST only", http.StatusMethodNotAllowed)
			return
		}
		if err := a.startZone(r.URL.Query().Get("zone"), r.URL.Query().Get("dock") != "0"); err != nil {
			http.Error(w, err.Error(), http.StatusConflict)
			return
		}
		w.WriteHeader(http.StatusNoContent)
	})
	a.srv.Handle("/api/explore", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			http.Error(w, "POST only", http.StatusMethodNotAllowed)
			return
		}
		a.setLidar(true, false)
		if err := a.cleaner.Explore(r.URL.Query().Get("dock") != "0"); err != nil {
			http.Error(w, err.Error(), http.StatusConflict)
			return
		}
		w.WriteHeader(http.StatusNoContent)
	})
	a.srv.Handle("/api/clean/stop", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			http.Error(w, "POST only", http.StatusMethodNotAllowed)
			return
		}
		a.cleaner.Stop()
		w.WriteHeader(http.StatusNoContent)
	})
}

// publishZones sends the current zones to every open page.
func (a *app) publishZones() { a.srv.Publish("zones", a.zones.List()) }

// startZone starts cleaning the zone with this ID.
func (a *app) startZone(id string, returnToDock bool) error {
	z, ok := a.zones.Get(id)
	if !ok {
		return fmt.Errorf("no zone %q", id)
	}
	if z.Kind != zones.Clean {
		return fmt.Errorf("%q is not a zone to clean", z.Name)
	}
	return a.cleaner.Start(nav.Area{ID: z.ID, Name: z.Name, Polygon: z.Points}, returnToDock)
}
