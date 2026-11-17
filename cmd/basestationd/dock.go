package main

import (
	"log"

	"github.com/YannickRiou/madroomba-core/internal/camera"
)

// Dock presence works from two learned reference images, stored as coarse
// grayscale grids: "robot docked" and "dock empty". The current image is
// compared to both (camera.GridDistance, brightness-insensitive) and the
// closest one wins. With a single reference, a fixed distance threshold is
// used instead.
const (
	dockThreshold = 5.0 // single-reference decision distance
	dockConfirm   = 3   // consecutive identical readings before the state changes
)

// dockRefs are the learned reference grids, persisted in <data-dir>/dock.json.
type dockRefs struct {
	Docked []float64 `json:"docked,omitempty"`
	Empty  []float64 `json:"empty,omitempty"`
}

// dockDecision returns "docked", "empty" or "unknown" from the distances to
// the references (hasD / hasE tell which references exist).
func dockDecision(d, e float64, hasD, hasE bool) string {
	switch {
	case hasD && hasE:
		if d <= e {
			return "docked"
		}
		return "empty"
	case hasD:
		if d < dockThreshold {
			return "docked"
		}
		return "empty"
	case hasE:
		if e < dockThreshold {
			return "empty"
		}
		return "docked"
	}
	return "unknown"
}

// observeDock feeds one image grid into the dock state machine. The confirmed
// state only changes after dockConfirm identical readings in a row, which
// filters out frames where someone walks past.
func (a *app) observeDock(g []float64) {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.grid = g
	if a.refs.Docked != nil {
		a.dD = camera.GridDistance(g, a.refs.Docked)
	}
	if a.refs.Empty != nil {
		a.dE = camera.GridDistance(g, a.refs.Empty)
	}
	next := dockDecision(a.dD, a.dE, a.refs.Docked != nil, a.refs.Empty != nil)
	if next == a.dPending {
		a.dCount++
	} else {
		a.dPending, a.dCount = next, 1
	}
	if a.dCount >= dockConfirm && next != a.dState {
		log.Printf("basestationd: dock state %s -> %s", a.dState, next)
		a.dState = next
	}
}
