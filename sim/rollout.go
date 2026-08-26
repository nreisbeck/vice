// sim/rollout.go
// Copyright(c) 2022-2026 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package sim

import (
	gomath "math"
	"time"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/math"
)

// RolloutState tracks an aircraft decelerating on the runway after
// touchdown at a staffed-tower airport.
type RolloutState struct {
	Dir       [2]float32 // unit direction along the runway, nm coordinates
	GS        float32    // groundspeed, knots
	Remaining float32    // nm of roll left before exiting (or stopping, for LAHSO)
	HoldShort bool       // stop at the end of the roll instead of exiting
	Exiting   bool       // has turned off the runway
	Until     Time       // when the exit taxi / hold-short dwell ends and the aircraft is deleted
}

// rolloutDistanceNM returns a representative landing roll for the
// aircraft's wake class.
func rolloutDistanceNM(cwt string) float32 {
	if cwt == "" {
		return 0.8
	}
	switch cwt[0] {
	case 'A', 'B', 'C', 'D':
		return 1.1
	case 'E', 'F':
		return 0.9
	case 'G':
		return 0.7
	default: // H, I: small types
		return 0.5
	}
}

// holdShortDistanceNM returns the distance (nm) from the landing runway's
// threshold to the intersection with the hold-short runway, less a margin,
// or false if the runways don't intersect within their lengths.
func (s *Sim) holdShortDistanceNM(airport string, ap *av.Approach, holdShort string) (float32, bool) {
	rwy, ok := av.LookupRunway(airport, holdShort)
	if !ok {
		return 0, false
	}
	opp, ok := av.LookupOppositeRunway(airport, holdShort)
	if !ok {
		return 0, false
	}

	nmPerLong := s.State.NmPerLongitude
	a := math.LL2NM(ap.Threshold, nmPerLong)
	u := math.Sub2f(math.LL2NM(ap.OppositeThreshold, nmPerLong), a)
	b := math.LL2NM(rwy.Threshold, nmPerLong)
	v := math.Sub2f(math.LL2NM(opp.Threshold, nmPerLong), b)

	denom := u[0]*v[1] - u[1]*v[0]
	if gomath.Abs(float64(denom)) < 1e-6 {
		return 0, false // parallel
	}
	w := math.Sub2f(b, a)
	tt := (w[0]*v[1] - w[1]*v[0]) / denom // fraction along landing runway
	ss := (w[0]*u[1] - w[1]*u[0]) / -denom
	_ = ss
	sfrac := (w[0]*u[1] - w[1]*u[0]) / (v[0]*u[1] - v[1]*u[0])
	if sfrac < 0 || sfrac > 1 {
		return 0, false // beyond the crossing runway's extent
	}
	dist := tt * math.Length2f(u)
	const margin = 0.08
	if dist <= margin {
		return 0, false
	}
	return dist - margin, true
}

// beginRollout starts the post-touchdown ground roll.
func (s *Sim) beginRollout(ac *Aircraft) {
	ap := ac.Nav.Approach.Assigned
	if ap == nil {
		s.deleteAircraft(ac)
		return
	}
	nmPerLong := s.State.NmPerLongitude
	thr := math.LL2NM(ap.Threshold, nmPerLong)
	opp := math.LL2NM(ap.OppositeThreshold, nmPerLong)
	dir := math.Normalize2f(math.Sub2f(opp, thr))

	dist := rolloutDistanceNM(string(ac.CWT()))
	holdShort := false
	if ac.LandingHoldShortRunway != "" {
		if d, ok := s.holdShortDistanceNM(ac.FlightPlan.ArrivalAirport, ap, ac.LandingHoldShortRunway); ok && d < dist {
			dist = d
			holdShort = true
		}
	}

	gs := ac.Nav.FlightState.GS
	if gs < 60 || gs > 180 {
		gs = 125
	}
	ac.Rollout = &RolloutState{Dir: dir, GS: gs, Remaining: dist, HoldShort: holdShort}
	if apDB, ok := av.DB.Airports[ac.FlightPlan.ArrivalAirport]; ok {
		ac.Nav.FlightState.Altitude = float32(apDB.Elevation)
	}
}

// updateRollout advances a rolling-out aircraft by one one-second tick.
func (s *Sim) updateRollout(ac *Aircraft) {
	r := ac.Rollout
	now := s.State.SimTime

	if (r.Exiting || (r.HoldShort && r.Remaining <= 0)) && now.After(r.Until) {
		s.deleteAircraft(ac)
		return
	}
	if r.HoldShort && r.Remaining <= 0 {
		return // holding short; wait out the dwell
	}

	// Decelerate toward taxi speed.
	r.GS = gomax(15, r.GS-3.5)
	step := r.GS / 3600 // nm per second
	r.Remaining -= step

	nmPerLong := s.State.NmPerLongitude
	pos := math.LL2NM(ac.Nav.FlightState.Position, nmPerLong)
	pos = math.Add2f(pos, math.Scale2f(r.Dir, step))
	ac.Nav.FlightState.Position = math.NM2LL(pos, nmPerLong)

	if r.Remaining <= 0 && !r.Exiting {
		if r.HoldShort {
			r.GS = 0
			r.Until = now.Add(45 * time.Second)
		} else {
			// Turn off the runway and taxi clear briefly before deletion.
			const cos40, sin40 = 0.766, 0.643
			r.Dir = [2]float32{r.Dir[0]*cos40 - r.Dir[1]*sin40, r.Dir[0]*sin40 + r.Dir[1]*cos40}
			r.GS = 15
			r.Exiting = true
			r.Until = now.Add(12 * time.Second)
		}
	}
}

func gomax(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
}
