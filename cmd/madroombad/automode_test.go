package main

import (
	"testing"
	"time"
)

func TestAutoDecision(t *testing.T) {
	for _, c := range []struct {
		name    string
		mode    byte
		charger bool
		idle    time.Duration
		busy    bool
		want    string
	}{
		{"safe on dock, idle", modeSafe, true, idleOnDock, false, "passive"},
		{"full on dock, idle", modeFull, true, idleOnDock, false, "passive"},
		{"safe on dock, just used", modeSafe, true, 5 * time.Second, false, ""},
		{"safe on dock, still driving", modeSafe, true, time.Hour, true, ""},
		{"passive on dock", modePassive, true, time.Hour, false, ""},
		{"safe off the dock", modeSafe, false, time.Hour, false, ""},
		{"full off the dock, idle", modeFull, false, idleInFull, false, "safe"},
		{"full off the dock, recent", modeFull, false, 10 * time.Second, false, ""},
		{"off", modeOff, true, time.Hour, false, ""},
	} {
		if got := autoDecision(c.mode, c.charger, c.idle, c.busy); got != c.want {
			t.Errorf("%s: got %q, want %q", c.name, got, c.want)
		}
	}
}
