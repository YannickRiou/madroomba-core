package main

import (
	"testing"
	"time"
)

func TestEcoDecision(t *testing.T) {
	for _, c := range []struct {
		name           string
		lidarOn, ecoOf bool
		mode           byte
		dock           bool
		idle           time.Duration
		busy           bool
		want           string
	}{
		{"resting on the dock", true, false, modePassive, true, ecoIdle, false, "off"},
		{"just docked", true, false, modePassive, true, 10 * time.Second, false, ""},
		{"docked but in Safe mode", true, false, modeSafe, true, time.Hour, false, ""},
		{"off the dock", true, false, modePassive, false, time.Hour, false, ""},
		{"saved, then driven", false, true, modeSafe, true, 0, true, "on"},
		{"saved, then left the dock", false, true, modePassive, false, time.Hour, false, "on"},
		{"saved, still resting", false, true, modePassive, true, time.Hour, false, ""},
		{"switched off by the user", false, false, modeSafe, false, 0, true, ""},
	} {
		if got := ecoDecision(c.lidarOn, c.ecoOf, c.mode, c.dock, c.idle, c.busy); got != c.want {
			t.Errorf("%s: got %q, want %q", c.name, got, c.want)
		}
	}
}
