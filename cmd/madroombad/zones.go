package main

import (
	"fmt"
	"io"
	"net/http"
	"strconv"
	"strings"

	"github.com/YannickRiou/madroomba-core/internal/mapping"
	"github.com/YannickRiou/madroomba-core/internal/nav"
	"github.com/YannickRiou/madroomba-core/internal/zones"
)

// registerZones serves the zones and the zone cleaning jobs:
//
//	GET  /api/zones                  the zones of the current map
//	POST /api/zones                  replace them (JSON array), returns them
//	POST /api/rooms                  add the rooms found in the map as zones to clean
//	POST /api/clean?zone=<id>&dock=1 clean a zone, then seek the dock; zone takes
//	                                 several IDs, cleaned in this order, and passes=2 sweeps each twice
//	POST /api/goto?x=<m>&y=<m>       drive to a point of the map
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
	a.srv.Handle("/api/rooms", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			http.Error(w, "POST only", http.StatusMethodNotAllowed)
			return
		}
		added, err := a.addRooms()
		if err != nil {
			http.Error(w, err.Error(), http.StatusBadRequest)
			return
		}
		a.publishZones()
		writeJSON(w, map[string]int{"added": added})
	})
	a.srv.Handle("/api/clean", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			http.Error(w, "POST only", http.StatusMethodNotAllowed)
			return
		}
		q := r.URL.Query()
		passes := 1
		if q.Get("passes") == "2" {
			passes = 2
		}
		if err := a.startZones(strings.Split(q.Get("zone"), ","), passes, q.Get("dock") != "0"); err != nil {
			http.Error(w, err.Error(), http.StatusConflict)
			return
		}
		w.WriteHeader(http.StatusNoContent)
	})
	a.srv.Handle("/api/goto", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			http.Error(w, "POST only", http.StatusMethodNotAllowed)
			return
		}
		x, errX := strconv.ParseFloat(r.URL.Query().Get("x"), 64)
		y, errY := strconv.ParseFloat(r.URL.Query().Get("y"), 64)
		if errX != nil || errY != nil {
			http.Error(w, "x and y: map coordinates in meters", http.StatusBadRequest)
			return
		}
		if err := a.cleaner.GoTo(mapping.Point{X: x, Y: y}); err != nil {
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

// roomDoorM is the widest opening that still separates two rooms: a little
// above a usual doorway. Much wider, and rooms split at furniture gaps.
const roomDoorM = 1.0

// addRooms adds the rooms found in the map as zones to clean and returns
// how many. A room whose centre already lies in a zone to clean is skipped,
// so asking again after more exploration only adds the new rooms.
func (a *app) addRooms() (int, error) {
	zs := a.zones.List()
	added := 0
	for _, room := range a.grid.Rooms(roomDoorM) {
		covered := false
		for _, z := range zs {
			if z.Kind == zones.Clean && z.Points.Contains(room.Center) {
				covered = true
				break
			}
		}
		if covered {
			continue
		}
		added++
		zs = append(zs, zones.Zone{Name: fmt.Sprintf("Pièce %d", len(zs)+1), Kind: zones.Clean, Points: room.Outline})
	}
	if added == 0 {
		return 0, nil
	}
	return added, a.zones.Set(zs)
}

// startZone starts cleaning the zone with this ID.
func (a *app) startZone(id string, returnToDock bool) error {
	return a.startZones([]string{id}, 1, returnToDock)
}

// startZones starts cleaning the zones with these IDs, in this order, each
// swept passes times.
func (a *app) startZones(ids []string, passes int, returnToDock bool) error {
	var areas []nav.Area
	for _, id := range ids {
		z, ok := a.zones.Get(id)
		if !ok {
			return fmt.Errorf("no zone %q", id)
		}
		if z.Kind != zones.Clean {
			return fmt.Errorf("%q is not a zone to clean", z.Name)
		}
		areas = append(areas, nav.Area{ID: z.ID, Name: z.Name, Polygon: z.Points})
	}
	return a.cleaner.StartAll(areas, passes, returnToDock)
}
