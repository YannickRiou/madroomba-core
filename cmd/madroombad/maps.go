package main

import (
	"fmt"
	"image/png"
	"net/http"
	"os"
	"path/filepath"
	"regexp"
	"sort"
	"strings"

	"github.com/YannickRiou/madroomba-core/internal/heatmap"
	"github.com/YannickRiou/madroomba-core/internal/mapping"
	"github.com/YannickRiou/madroomba-core/internal/odom"
	"github.com/YannickRiou/madroomba-core/internal/server"
	"github.com/YannickRiou/madroomba-core/internal/zones"
)

// validName keeps map names safe to use as file names.
var validName = regexp.MustCompile(`^[A-Za-z0-9_-]{1,40}$`)

// mapStore keeps saved maps in dir as <name>.png + <name>.yaml, the ROS
// map_server format, with the zones drawn on them in <name>.zones.json.
type mapStore struct {
	dir     string
	grid    *mapping.Grid
	tracker *odom.Tracker
	zones   *zones.Store
	heat    *heatmap.Map
	// loaded, if set, is called after a map (and its zones) is loaded.
	loaded func()
}

func (m *mapStore) zonesPath(name string) string {
	return filepath.Join(m.dir, name+".zones.json")
}

func (m *mapStore) heatPath(name string) string {
	return filepath.Join(m.dir, name+".heat.json")
}

func (m *mapStore) list() ([]string, error) {
	files, err := filepath.Glob(filepath.Join(m.dir, "*.yaml"))
	if err != nil {
		return nil, err
	}
	names := []string{}
	for _, f := range files {
		names = append(names, strings.TrimSuffix(filepath.Base(f), ".yaml"))
	}
	sort.Strings(names)
	return names, nil
}

func (m *mapStore) save(name string) error {
	pngData, yml, err := m.grid.EncodeROS(name)
	if err != nil {
		return err
	}
	if err := os.MkdirAll(m.dir, 0755); err != nil {
		return err
	}
	if err := os.WriteFile(filepath.Join(m.dir, name+".png"), pngData, 0644); err != nil {
		return err
	}
	if err := m.zones.SaveCopy(m.zonesPath(name)); err != nil {
		return err
	}
	if err := m.heat.SaveCopy(m.heatPath(name)); err != nil {
		return err
	}
	return os.WriteFile(filepath.Join(m.dir, name+".yaml"), yml, 0644)
}

// load replaces the live map and puts the robot back at the map origin:
// maps are made starting from the dock, so the robot must be on the dock
// (or where mapping started) when a map is loaded.
func (m *mapStore) load(name string) error {
	yml, err := os.ReadFile(filepath.Join(m.dir, name+".yaml"))
	if err != nil {
		return err
	}
	meta, err := mapping.ParseROSYAML(yml)
	if err != nil {
		return err
	}
	f, err := os.Open(filepath.Join(m.dir, filepath.Base(meta.Image)))
	if err != nil {
		return err
	}
	defer f.Close()
	img, err := png.Decode(f)
	if err != nil {
		return fmt.Errorf("%s: %w", meta.Image, err)
	}
	if err := m.zones.LoadCopy(m.zonesPath(name)); err != nil {
		return fmt.Errorf("zones of %s: %w", name, err)
	}
	if err := m.heat.LoadCopy(m.heatPath(name)); err != nil {
		return fmt.Errorf("heatmap of %s: %w", name, err)
	}
	m.grid.DecodeROS(meta, img)
	m.tracker.Reset()
	if m.loaded != nil {
		m.loaded()
	}
	return nil
}

func (m *mapStore) remove(name string) error {
	if err := os.Remove(filepath.Join(m.dir, name+".yaml")); err != nil {
		return err
	}
	os.Remove(filepath.Join(m.dir, name+".png"))
	os.Remove(m.zonesPath(name))
	os.Remove(m.heatPath(name))
	return nil
}

// postName wraps an action on a map named by the ?name= parameter.
func postName(fn func(string) error) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			http.Error(w, "POST only", http.StatusMethodNotAllowed)
			return
		}
		name := r.URL.Query().Get("name")
		if !validName.MatchString(name) {
			http.Error(w, "name: 1 to 40 letters, digits, - or _", http.StatusBadRequest)
			return
		}
		if err := fn(name); err != nil {
			code := http.StatusInternalServerError
			if os.IsNotExist(err) {
				code = http.StatusNotFound
			}
			http.Error(w, err.Error(), code)
			return
		}
		w.WriteHeader(http.StatusNoContent)
	}
}

func (m *mapStore) register(srv *server.Server) {
	srv.Handle("/api/maps", func(w http.ResponseWriter, r *http.Request) {
		names, err := m.list()
		if err != nil {
			http.Error(w, err.Error(), http.StatusInternalServerError)
			return
		}
		writeJSON(w, names)
	})
	srv.Handle("/api/maps/save", postName(m.save))
	srv.Handle("/api/maps/load", postName(m.load))
	srv.Handle("/api/maps/delete", postName(m.remove))
	// GET /maps/<name>.png or .yaml downloads a saved map file.
	srv.Handle("/maps/", func(w http.ResponseWriter, r *http.Request) {
		file := strings.TrimPrefix(r.URL.Path, "/maps/")
		ext := filepath.Ext(file)
		if (ext != ".png" && ext != ".yaml") || !validName.MatchString(strings.TrimSuffix(file, ext)) {
			http.NotFound(w, r)
			return
		}
		w.Header().Set("Content-Disposition", "attachment; filename="+file)
		http.ServeFile(w, r, filepath.Join(m.dir, file))
	})
}
