package main

import (
	"log"
	"sync"
	"sync/atomic"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/lidar"
)

// ecoIdle is how long the robot must rest on its dock, in Passive mode,
// before the energy saving switches the lidar off.
const ecoIdle = time.Minute

// ecoDecision is the energy saving rule, kept pure for tests. The lidar
// (about 400 mA at 5 V, from the Roomba battery) is the biggest load of the
// Pi: it goes off while the robot charges on its dock, and back on as soon
// as the robot is used. It returns "off", "on" or "" (no change). ecoOff
// tells that the lidar was switched off by this rule, not by the user: a
// lidar the user switched off stays off.
func ecoDecision(lidarOn, ecoOff bool, mode byte, onDock bool, idle time.Duration, busy bool) string {
	switch {
	case lidarOn && onDock && mode == modePassive && !busy && idle >= ecoIdle:
		return "off"
	case !lidarOn && ecoOff && (!onDock || mode == modeSafe || mode == modeFull || busy):
		return "on"
	}
	return ""
}

// lidarSwitch turns the lidar motor and laser on or off (commands 'b' and
// 'e' of the LDS-01). It remembers the wish while the lidar is not open.
type lidarSwitch struct {
	eco int32 // 1 = energy saving on the dock enabled (atomic)

	mu     sync.Mutex
	dev    *lidar.Device // nil in simulation or before the lidar opens
	on     bool
	ecoOff bool
}

func (l *lidarSwitch) ecoEnabled() bool { return atomic.LoadInt32(&l.eco) == 1 }

func (l *lidarSwitch) setEco(on bool) {
	v := int32(0)
	if on {
		v = 1
	}
	atomic.StoreInt32(&l.eco, v)
}

// enabled tells whether the lidar should spin.
func (l *lidarSwitch) enabled() bool {
	l.mu.Lock()
	defer l.mu.Unlock()
	return l.on
}

// state returns whether the lidar spins and whether energy saving stopped it.
func (l *lidarSwitch) state() (on, ecoOff bool) {
	l.mu.Lock()
	defer l.mu.Unlock()
	return l.on, l.ecoOff
}

// set switches the lidar; byEco marks a switch made by the energy saving.
func (l *lidarSwitch) set(on, byEco bool) {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.on, l.ecoOff = on, !on && byEco
	if l.dev == nil {
		return
	}
	var err error
	if on {
		err = l.dev.Start()
	} else {
		err = l.dev.Stop()
	}
	if err != nil {
		log.Printf("madroombad: lidar switch: %v", err)
	}
}

// attach records a freshly opened (and started) lidar and applies the
// current wish to it.
func (l *lidarSwitch) attach(dev *lidar.Device) {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.dev = dev
	if !l.on {
		dev.Stop()
	}
}

// ecoLoop applies ecoDecision every few seconds.
func (a *app) ecoLoop() {
	for range time.Tick(5 * time.Second) {
		if !a.lidar.ecoEnabled() {
			continue
		}
		s, known := a.auto.latest()
		if !known {
			continue
		}
		idle, busy := a.auto.idle()
		busy = busy || a.cleaner.Running()
		on, ecoOff := a.lidar.state()
		switch ecoDecision(on, ecoOff, s.OIMode, s.ChargingSources != 0, idle, busy) {
		case "off":
			log.Println("madroombad: energy saving: on the dock and idle, lidar off")
			a.setLidar(false, true)
		case "on":
			log.Println("madroombad: energy saving: robot in use, lidar on")
			a.setLidar(true, false)
		}
	}
}

// setLidar switches the lidar and clears the last scan when it stops.
func (a *app) setLidar(on, byEco bool) {
	a.lidar.set(on, byEco)
	if !on {
		a.stateMu.Lock()
		a.lastScan = nil
		a.stateMu.Unlock()
	}
}
