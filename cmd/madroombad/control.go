package main

import (
	"errors"
	"fmt"
	"log"
	"os"
	"sync/atomic"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/mapping"
	"github.com/YannickRiou/madroomba-core/internal/robot"
	"github.com/YannickRiou/madroomba-core/internal/server"
)

// errNoGo is returned when a drive command would enter a no-go zone.
var errNoGo = errors.New("no-go zone ahead: drive command refused")

// Teleoperation keeps this far from no-go zones, looking this far ahead.
const (
	teleopRadiusM  = 0.17
	teleopLookSecs = 0.6
)

// routes registers every HTTP endpoint of the robot daemon.
func (a *app) routes() {
	a.srv.OnDrive = a.drive
	a.srv.OnCommand = func(c server.Command) error { return a.command(c.Action) }
	a.srv.MapPNG = func() []byte {
		png, _ := a.grid.ToPNG(a.filtered())
		return png
	}

	// The cleaning motors, LEDs and songs need Safe or Full mode: smart
	// mode takes control first.
	a.srv.Handle("/api/motors", postJSON(func(m robot.Motors) error {
		on := m.MainBrush || m.Vacuum || m.SideBrush
		if on {
			if err := a.auto.takeControl(); err != nil {
				return err
			}
		}
		a.auto.activity(nil, &on)
		return a.driver.SetMotors(m)
	}))
	a.srv.Handle("/api/leds", postJSON(func(l robot.LEDs) error {
		if err := a.auto.takeControl(); err != nil {
			return err
		}
		a.auto.activity(nil, nil)
		return a.driver.SetLEDs(l)
	}))
	a.srv.Handle("/api/song", postJSON(func(s struct {
		Notes []robot.Note `json:"notes"`
	}) error {
		if len(s.Notes) == 0 || len(s.Notes) > robot.MaxNotes {
			return fmt.Errorf("a song has 1 to %d notes, got %d", robot.MaxNotes, len(s.Notes))
		}
		if err := a.auto.takeControl(); err != nil {
			return err
		}
		a.auto.activity(nil, nil)
		return a.driver.PlaySong(s.Notes)
	}))

	a.maps.register(a.srv)
	a.sched.register(a.srv)
	a.registerZones()
	a.sounds.register(a.srv)
	a.registerBattery()
	a.registerHeatmap()
}

// drive handles a teleoperation command. Moving cancels a cleaning job,
// takes control in smart mode, and is refused toward a no-go zone.
func (a *app) drive(d server.DriveRequest) error {
	mv := d.Left != 0 || d.Right != 0
	if mv {
		if a.cleaner.Running() {
			log.Println("madroombad: teleoperation takes over, cleaning job stopped")
			a.cleaner.Stop()
		}
		p := a.tracker.Pose()
		v := float64(d.Left+d.Right) / 2 / 1000
		if !a.zones.DriveAllowed(mapping.Point{X: p.X, Y: p.Y}, p.Theta, v, teleopRadiusM, teleopLookSecs) {
			a.driver.SetWheels(0, 0)
			return errNoGo
		}
		if err := a.auto.takeControl(); err != nil {
			return err
		}
	}
	a.auto.activity(&mv, nil)
	a.driveMu.Lock()
	a.lastDrive = time.Now()
	a.moving = mv
	a.driveMu.Unlock()
	return a.driver.SetWheels(d.Left, d.Right)
}

// command runs one action of POST /api/cmd (listed in docs/api.md).
func (a *app) command(action string) error {
	log.Printf("madroombad: command %q", action)
	switch action {
	case "stop":
		a.cleaner.Stop()
		return a.driver.SetWheels(0, 0)
	case "clean":
		a.cleaner.Stop()
		return a.driver.Clean()
	case "dock":
		a.cleaner.Stop()
		return a.driver.Dock()
	case "spot", "max", "power_off":
		a.cleaner.Stop()
		return a.driver.Action(action)
	case "explore":
		// map the place: the lidar must spin, the robot returns to its dock
		a.setLidar(true, false)
		return a.cleaner.Explore(true)
	case "passive":
		return a.driver.Action(action)
	case "full":
		a.auto.activity(nil, nil) // Full stays on for idleInFull from now
		return a.driver.Action(action)
	case "safe":
		a.auto.activity(nil, nil)
		err := a.driver.Reactivate()
		if err != nil {
			log.Printf("madroombad: safe mode: %v", err)
		} else {
			log.Println("madroombad: safe mode on, drive commands accepted")
		}
		return err
	case "auto_on", "auto_off":
		a.auto.set(action == "auto_on")
		return nil
	case "lidar_on", "lidar_off":
		a.auto.activity(nil, nil) // the energy saving waits ecoIdle from now
		a.setLidar(action == "lidar_on", false)
		return nil
	case "eco_on", "eco_off":
		a.lidar.setEco(action == "eco_on")
		return nil
	case "map_filter_on", "map_filter_off":
		v := int32(0)
		if action == "map_filter_on" {
			v = 1
		}
		atomic.StoreInt32(&a.mapFilter, v)
		return nil
	case "reset_map":
		a.grid.Reset()
		return nil
	case "reset_pose":
		a.tracker.Reset()
		return nil
	case "restart":
		// Stop the wheels, let the HTTP reply go out, then exit non-zero so
		// that systemd (Restart=on-failure) starts a fresh daemon.
		// SetWheels runs on its own: it waits for the OI lock, which a read
		// stuck on a silent Roomba never releases, and the exit must happen
		// anyway.
		go a.driver.SetWheels(0, 0)
		go func() {
			time.Sleep(300 * time.Millisecond)
			os.Exit(1)
		}()
		return nil
	}
	return fmt.Errorf("unknown action %q", action)
}
