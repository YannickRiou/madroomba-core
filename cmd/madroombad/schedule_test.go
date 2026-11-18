package main

import (
	"testing"
	"time"
)

func TestValidateSchedule(t *testing.T) {
	ok := []scheduleEntry{
		{Days: []int{1, 3}, Time: "09:30", Action: "clean", Enabled: true},
		{Days: []int{6}, Time: "10:00", Action: "zone:zabc1", Enabled: true},
	}
	if err := validateSchedule(ok); err != nil {
		t.Fatalf("valid schedule rejected: %v", err)
	}
	for name, e := range map[string]scheduleEntry{
		"bad time":   {Days: []int{1}, Time: "25:00", Action: "clean"},
		"bad action": {Days: []int{1}, Time: "09:30", Action: "power_off"},
		"bad zone":   {Days: []int{1}, Time: "09:30", Action: "zone:../x"},
		"no day":     {Time: "09:30", Action: "clean"},
		"bad day":    {Days: []int{7}, Time: "09:30", Action: "clean"},
	} {
		if validateSchedule([]scheduleEntry{e}) == nil {
			t.Errorf("%s: accepted", name)
		}
	}
}

func TestMatchesAndNextRun(t *testing.T) {
	// Wednesday 18 November 2026, 08:00 local time.
	now := time.Date(2026, 11, 18, 8, 0, 0, 0, time.Local)
	es := []scheduleEntry{
		{Days: []int{3}, Time: "07:00", Action: "clean", Enabled: true}, // Wednesday, already past
		{Days: []int{5}, Time: "10:15", Action: "spot", Enabled: true},  // Friday
		{Days: []int{3}, Time: "09:00", Action: "max", Enabled: false},  // disabled
	}
	if !es[0].matches(time.Date(2026, 11, 18, 7, 0, 30, 0, time.Local)) {
		t.Error("entry should match at 07:00 on Wednesday")
	}
	if es[2].matches(time.Date(2026, 11, 18, 9, 0, 0, 0, time.Local)) {
		t.Error("disabled entry matched")
	}
	at, e, ok := nextRun(es, now)
	if !ok || e.Action != "spot" || at.Weekday() != time.Friday || at.Format("15:04") != "10:15" {
		t.Errorf("next run = %v %+v %v, want Friday 10:15 spot", at, e, ok)
	}
	if _, _, ok := nextRun(nil, now); ok {
		t.Error("empty schedule has a next run")
	}
}
