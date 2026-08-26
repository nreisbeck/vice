// sim/goaround_route_test.go
// Copyright(c) 2022-2026 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package sim

import (
	"testing"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/math"
)

// TestGoAroundPublishedMissed: an IFR arrival on an instrument approach
// with a published missed approach flies it on a go-around.
func TestGoAroundPublishedMissed(t *testing.T) {
	airportLoc := math.Point2LL{0, 0}
	setupTestRunway(t, "KJFK", av.Runway{Id: "13L", Heading: 130, Threshold: airportLoc, Elevation: 13})
	vs := NewVisualScenario(t, airportLoc, "13L", math.Point2LL{0, 5.0 / 60}, 130)
	ac := vs.AC
	ac.FlightPlan.Rules = av.FlightRulesIFR
	ap := ac.Nav.Approach.Assigned
	ap.Threshold = airportLoc
	ap.OppositeThreshold = math.Offset2LL(airportLoc, math.TrueHeading(130), 2, 52)
	ap.MissedApproach = av.WaypointArray{{Fix: "MISSD", Location: math.Point2LL{1.0 / 52, 0}}}
	ac.Nav.Approach.Cleared = true

	vs.Sim.goAround(ac)

	if len(ac.Nav.Waypoints) == 0 || ac.Nav.Waypoints[0].Fix != "MISSD" {
		t.Fatalf("expected published missed route, got %v", wpNames(ac.Nav.Waypoints))
	}
}

// TestGoAroundPatternRejoinForPiston: a light piston going around from a
// visual approach rejoins the traffic pattern and asks for resequencing
// from the downwind.
func TestGoAroundPatternRejoinForPiston(t *testing.T) {
	airportLoc := math.Point2LL{0, 0}
	oppLoc := math.Point2LL{0, 1.0 / 60}
	setupTestRunways(t, "KPAT", []av.Runway{
		{Id: "36", Heading: 360, Threshold: airportLoc, Elevation: 0},
		{Id: "18", Heading: 180, Threshold: oppLoc, Elevation: 0},
	})
	vs := NewVisualScenario(t, airportLoc, "36", math.Point2LL{0, -3.0 / 60}, 360)
	ac := vs.AC
	ac.FlightPlan.ArrivalAirport = "KPAT"
	ac.FlightPlan.Rules = av.FlightRulesVFR
	ac.FlightPlan.AircraftType = "C172"
	ap := ac.Nav.Approach.Assigned
	ap.Type = av.VisualApproach
	ap.Runway = "36"
	ap.Threshold = airportLoc
	ap.OppositeThreshold = oppLoc

	vs.Sim.goAround(ac)

	wps := ac.Nav.Waypoints
	if len(wps) != 7 || wps[0].Fix != "_ga_upwind" || wps[6].Fix != "_ga_threshold" {
		t.Fatalf("expected closed-pattern rejoin waypoints, got %v", wpNames(wps))
	}
	if !wps[6].Land() {
		t.Errorf("expected Land flag on the pattern threshold waypoint")
	}
	// The aircraft stays in the tower's pattern; no controller change.
	if wps[0].GoAroundContactController() != "" {
		t.Errorf("pattern rejoin should not switch controllers")
	}
	// Runway "36" (no parallel suffix) flies standard left traffic: the
	// downwind is west of the runway.
	if wps[2].Location[0] >= 0 {
		t.Errorf("expected left-traffic downwind west of runway 36, got lon %v", wps[2].Location[0])
	}
}

// TestGoAroundPatternRejoinRightTraffic: a go-around from a right-hand
// parallel (e.g. 36R) rejoins in right traffic, away from the other
// parallel.
func TestGoAroundPatternRejoinRightTraffic(t *testing.T) {
	airportLoc := math.Point2LL{0, 0}
	oppLoc := math.Point2LL{0, 1.0 / 60}
	setupTestRunways(t, "KPAR", []av.Runway{
		{Id: "36R", Heading: 360, Threshold: airportLoc, Elevation: 0},
		{Id: "18L", Heading: 180, Threshold: oppLoc, Elevation: 0},
	})
	vs := NewVisualScenario(t, airportLoc, "36R", math.Point2LL{0, -3.0 / 60}, 360)
	ac := vs.AC
	ac.FlightPlan.ArrivalAirport = "KPAR"
	ac.FlightPlan.Rules = av.FlightRulesVFR
	ac.FlightPlan.AircraftType = "C172"
	ap := ac.Nav.Approach.Assigned
	ap.Type = av.VisualApproach
	ap.Runway = "36R"
	ap.Threshold = airportLoc
	ap.OppositeThreshold = oppLoc

	vs.Sim.goAround(ac)

	wps := ac.Nav.Waypoints
	if len(wps) != 7 {
		t.Fatalf("expected closed-pattern rejoin waypoints, got %v", wpNames(wps))
	}
	// Right traffic: the downwind is east of the runway.
	if wps[2].Location[0] <= 0 {
		t.Errorf("expected right-traffic downwind east of runway 36R, got lon %v", wps[2].Location[0])
	}
}

// TestPatternCircuitHoldsBaseForFinalTraffic: a circuit aircraft
// approaching its base turn spins when the final is occupied and
// proceeds when it's clear.
func TestPatternCircuitHoldsBaseForFinalTraffic(t *testing.T) {
	airportLoc := math.Point2LL{0, 0}
	oppLoc := math.Point2LL{0, 1.0 / 60}
	setupTestRunways(t, "KPAT", []av.Runway{
		{Id: "36", Heading: 360, Threshold: airportLoc, Elevation: 0},
		{Id: "18", Heading: 180, Threshold: oppLoc, Elevation: 0},
	})
	vs := NewVisualScenario(t, airportLoc, "36", math.Point2LL{0, -3.0 / 60}, 360)
	ac := vs.AC
	ac.FlightPlan.ArrivalAirport = "KPAT"
	ac.FlightPlan.Rules = av.FlightRulesVFR
	ac.FlightPlan.AircraftType = "C172"
	ap := ac.Nav.Approach.Assigned
	ap.Type = av.VisualApproach
	ap.Runway = "36"
	ap.Threshold = airportLoc
	ap.OppositeThreshold = oppLoc
	vs.Sim.goAround(ac)

	// Position the circuit aircraft just before its base turn, heading
	// down the (left) downwind.
	wps := ac.Nav.Waypoints
	baseIdx := -1
	for i, wp := range wps {
		if wp.Fix == "_ga_base" {
			baseIdx = i
		}
	}
	if baseIdx < 0 {
		t.Fatalf("no base waypoint in %v", wpNames(wps))
	}
	ac.Nav.Waypoints = wps[baseIdx:]
	ac.Nav.FlightState.Position = math.Offset2LL(wps[baseIdx].Location, math.TrueHeading(90), 0.5, 60)
	ac.Nav.FlightState.Heading = 180

	// A second aircraft on a 2nm final for the same runway.
	final := &Aircraft{
		ADSBCallsign: "FINAL1",
		FlightPlan:   av.FlightPlan{ArrivalAirport: "KPAT"},
	}
	final.Nav.FlightState.Position = math.Point2LL{0, -2.0 / 60}
	final.Nav.Approach.Assigned = &av.Approach{Type: av.ILSApproach, Runway: "36"}
	vs.Sim.Aircraft["FINAL1"] = final

	vs.Sim.managePatternCircuits()
	if ac.Nav.Orbit == nil {
		t.Fatalf("expected a spacing orbit with traffic on final")
	}

	// Clear the final: the next pass leaves the base turn alone.
	ac.Nav.Orbit = nil
	delete(vs.Sim.Aircraft, "FINAL1")
	vs.Sim.managePatternCircuits()
	if ac.Nav.Orbit != nil {
		t.Fatalf("unexpected orbit with a clear final")
	}
}
