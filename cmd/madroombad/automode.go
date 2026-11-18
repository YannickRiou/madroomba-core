package main

import (
	"log"
	"sync"
	"sync/atomic"
	"time"

	"github.com/YannickRiou/madroomba-core/internal/robot"
)

// OI modes (packet 35).
const (
	modeOff     = 0
	modePassive = 1
	modeSafe    = 2
	modeFull    = 3
)

// autoDecision is the smart mode rule, kept pure for tests. It returns
// "passive" to give the robot back on the dock (it does not charge in Safe
// or Full mode), "safe" to leave a forgotten Full mode, or "" to do
// nothing. busy means the wheels or the cleaning motors are running.
func autoDecision(mode byte, onCharger bool, idle time.Duration, busy bool) string {
	if busy {
		return ""
	}
	controlled := mode == modeSafe || mode == modeFull
	switch {
	case controlled && onCharger && idle >= idleOnDock:
		return "passive"
	case mode == modeFull && idle >= idleInFull:
		return "safe"
	}
	return ""
}

// autoMode switches the OI mode for the user: Safe when they drive or use
// the motors, LEDs or songs, Passive again on the dock once idle.
type autoMode struct {
	driver robot.Driver
	on     int32 // 1 = enabled
	// jobRunning, if set, tells when an autonomous job keeps the robot busy:
	// the robot is then never handed back to Passive.
	jobRunning func() bool

	mu     sync.Mutex
	sens   robot.Sensors
	have   bool
	last   time.Time // last user activity
	motors bool
	moving bool
}

func (a *autoMode) enabled() bool { return atomic.LoadInt32(&a.on) == 1 }

func (a *autoMode) set(on bool) {
	v := int32(0)
	if on {
		v = 1
	}
	atomic.StoreInt32(&a.on, v)
}

// observe records the latest sensors (OI mode, charging sources).
func (a *autoMode) observe(s robot.Sensors) {
	a.mu.Lock()
	a.sens, a.have = s, true
	a.mu.Unlock()
}

// activity records a user action; moving and motors tell whether the
// wheels or cleaning motors are now running.
func (a *autoMode) activity(moving, motors *bool) {
	a.mu.Lock()
	a.last = time.Now()
	if moving != nil {
		a.moving = *moving
	}
	if motors != nil {
		a.motors = *motors
	}
	a.mu.Unlock()
}

// latest returns the last sensor reading, if any.
func (a *autoMode) latest() (robot.Sensors, bool) {
	a.mu.Lock()
	defer a.mu.Unlock()
	return a.sens, a.have
}

// idle returns how long the user has done nothing, and whether the wheels
// or the cleaning motors are running.
func (a *autoMode) idle() (time.Duration, bool) {
	a.mu.Lock()
	defer a.mu.Unlock()
	return time.Since(a.last), a.moving || a.motors
}

// inControl tells whether the robot was last seen in Safe or Full mode.
func (a *autoMode) inControl() bool {
	a.mu.Lock()
	defer a.mu.Unlock()
	return a.have && (a.sens.OIMode == modeSafe || a.sens.OIMode == modeFull)
}

// takeControl puts the robot in Safe mode if smart mode is on and the
// robot is not already under control.
func (a *autoMode) takeControl() error {
	if !a.enabled() {
		return nil
	}
	a.mu.Lock()
	mode, known := a.sens.OIMode, a.have
	a.mu.Unlock()
	if known && (mode == modeSafe || mode == modeFull) {
		return nil
	}
	log.Println("madroombad: auto mode: taking control (Safe mode)")
	if err := a.driver.Reactivate(); err != nil {
		return err
	}
	a.mu.Lock()
	a.sens.OIMode = modeSafe // until the next sensor reading
	a.mu.Unlock()
	return nil
}

// loop applies autoDecision every second.
func (a *autoMode) loop() {
	for range time.Tick(time.Second) {
		if !a.enabled() {
			continue
		}
		a.mu.Lock()
		s, known, idle, busy := a.sens, a.have, time.Since(a.last), a.moving || a.motors
		a.mu.Unlock()
		if a.jobRunning != nil && a.jobRunning() {
			busy = true
		}
		if !known {
			continue
		}
		switch autoDecision(s.OIMode, s.ChargingSources != 0, idle, busy) {
		case "passive":
			log.Printf("madroombad: auto mode: on the dock and idle for %s, back to Passive so the battery charges", idleOnDock)
			if err := a.driver.Action("passive"); err != nil {
				log.Printf("madroombad: auto mode: %v", err)
				continue
			}
			a.mu.Lock()
			a.sens.OIMode = modePassive
			a.mu.Unlock()
		case "safe":
			log.Printf("madroombad: auto mode: Full mode idle for %s, back to Safe", idleInFull)
			if err := a.driver.Reactivate(); err != nil {
				log.Printf("madroombad: auto mode: %v", err)
			}
		}
	}
}
