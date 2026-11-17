package main

import (
	"errors"
	"reflect"
	"testing"
)

func TestSettingsValidation(t *testing.T) {
	if err := defaultSettings().valid(); err != nil {
		t.Fatalf("defaults must be valid: %v", err)
	}
	bad := []camSettings{
		{Brightness: 101, AWB: "auto", Exposure: "auto"},
		{Brightness: 50, Contrast: -101, AWB: "auto", Exposure: "auto"},
		{Brightness: 50, AWB: "nope", Exposure: "auto"},
		{Brightness: 50, AWB: "auto", Exposure: "auto; rm -rf /"},
	}
	for i, s := range bad {
		if s.valid() == nil {
			t.Errorf("case %d should be rejected: %+v", i, s)
		}
	}
}

func TestSettingsArgs(t *testing.T) {
	s := defaultSettings()
	s.HFlip = true
	want := []string{"-br", "50", "-co", "0", "-sa", "0", "-awb", "auto", "-ex", "auto", "-hf"}
	if got := s.args(); !reflect.DeepEqual(got, want) {
		t.Fatalf("args = %v, want %v", got, want)
	}
}

func TestDockDecision(t *testing.T) {
	cases := []struct {
		d, e       float64
		hasD, hasE bool
		want       string
	}{
		{0, 0, false, false, "unknown"},
		{2, 9, true, true, "docked"},
		{9, 2, true, true, "empty"},
		{3, 0, true, false, "docked"}, // below threshold
		{8, 0, true, false, "empty"},  // above threshold
		{0, 3, false, true, "empty"},  // below threshold
		{0, 8, false, true, "docked"}, // above threshold
	}
	for _, c := range cases {
		if got := dockDecision(c.d, c.e, c.hasD, c.hasE); got != c.want {
			t.Errorf("dockDecision(%v,%v,%v,%v) = %q, want %q", c.d, c.e, c.hasD, c.hasE, got, c.want)
		}
	}
}

func TestObserveDockNeedsConfirmation(t *testing.T) {
	flat := make([]float64, 768)
	busy := make([]float64, 768)
	for i := range busy {
		busy[i] = float64(i % 50)
	}
	a := &app{dState: "unknown", refs: dockRefs{Docked: busy, Empty: flat}}
	for i := 0; i < dockConfirm-1; i++ {
		a.observeDock(busy)
	}
	if a.dState != "unknown" {
		t.Fatalf("state changed after %d readings, want %d", dockConfirm-1, dockConfirm)
	}
	a.observeDock(busy)
	if a.dState != "docked" {
		t.Fatalf("state = %q after %d readings, want docked", a.dState, dockConfirm)
	}
}

func TestSnapStore(t *testing.T) {
	s := snapStore{dir: t.TempDir(), keep: 2}
	for _, r := range []string{"a", "b", "c"} {
		// names carry a second-resolution timestamp: write distinct files directly
		if _, err := s.save([]byte("x"), r); err != nil {
			t.Fatal(err)
		}
	}
	if n := len(s.list()); n > 2 {
		t.Fatalf("kept %d snapshots, limit is 2", n)
	}
	if err := s.delete("../../etc/passwd"); !errors.Is(err, errUnknownSnap) {
		t.Fatalf("path traversal must be rejected, got %v", err)
	}
	names := s.list()
	if len(names) == 0 {
		t.Fatal("expected at least one snapshot")
	}
	if err := s.delete(names[0]); err != nil {
		t.Fatal(err)
	}
	s.deleteAll()
	if n := len(s.list()); n != 0 {
		t.Fatalf("%d snapshots left after deleteAll", n)
	}
}
