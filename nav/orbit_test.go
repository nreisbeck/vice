// nav/orbit_test.go
// Copyright(c) 2022-2026 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package nav

import (
	"testing"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/wx"
)

// TestOrbit: a commanded left 360 turns through the full circle at
// standard rate and then clears, leaving heading state untouched.
func TestOrbit(t *testing.T) {
	n := Nav{}
	n.FlightState.Heading = 90

	intent := n.StartOrbit(av.TurnLeft, 360)
	if o, ok := intent.(av.OrbitIntent); !ok || !o.Left || o.Degrees != 360 {
		t.Fatalf("unexpected intent %T: %v", intent, intent)
	}

	var minutes int
	for range 3600 { // bounded; expect ~120s at 3 deg/s
		if n.Orbit == nil {
			break
		}
		n.updateHeading("TEST", wx.Sample{}, Time{})
		minutes++
	}
	if n.Orbit != nil {
		t.Fatalf("orbit never completed")
	}
	if got := n.FlightState.Heading; got < 89 || got > 91 {
		t.Errorf("heading after orbit = %v, want ~90", got)
	}
	if minutes < 115 || minutes > 125 {
		t.Errorf("orbit took %d ticks, want ~120", minutes)
	}
}
