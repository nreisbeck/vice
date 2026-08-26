// sim/goaround.go
// Copyright(c) 2022-2026 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package sim

import (
	"cmp"
	"fmt"
	"log/slog"
	"slices"
	"strings"
	"time"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/math"
)

func (s *Sim) contactDeparture(ac *Aircraft, fp *NASFlightPlan) {
	tcp := fp.InboundHandoffController
	s.lg.Debug("contacting departure controller", slog.String("tcp", string(tcp)))

	// Mark as already contacted so we only send one contact message
	ac.DepartureContactAltitude = -1

	// Queue the contact (may be delayed due to radio activity)
	s.enqueueDepartureContact(ac, tcp)
}

func (s *Sim) isRadarVisible(ac *Aircraft) bool {
	filters := s.State.FacilityAdaptation.Filters
	return !filters.SurfaceTracking.Inside(ac.Position(), int(ac.Altitude()))
}

func (s *Sim) goAround(ac *Aircraft) {
	// Capture approach info before anything clears it.
	approach := ac.Nav.Approach.Assigned
	if approach == nil {
		s.lg.Warn("goAround called without assigned approach",
			slog.String("callsign", string(ac.ADSBCallsign)))
		return
	}
	airport := ac.FlightPlan.ArrivalAirport
	runway := approach.Runway

	proc := s.getGoAroundProcedureForAircraft(ac)
	if proc.HandoffController == "" {
		proc.HandoffController = s.getGoAroundController(ac)
	}

	ac.WentAround = true
	ac.GotContactTower = false
	ac.AskedAboutTowerSwitch = false
	ac.SpacingGoAroundDeclined = false
	ac.GoAroundOnRunwayHeading = proc.IsRunwayHeading

	if s.tryPatternGoAround(ac, airport, runway) {
		s.holdDeparturesForGoAround(airport, append([]string{runway}, proc.HoldDepartures...), proc.HandoffController)
		return
	}
	if s.tryPublishedMissed(ac, approach, proc) {
		s.holdDeparturesForGoAround(airport, append([]string{runway}, proc.HoldDepartures...), proc.HandoffController)
		return
	}

	altitude := float32(proc.Altitude)

	// Waypoint at the opposite threshold recording who to contact when it's reached.
	wp := av.Waypoint{
		Location:       approach.OppositeThreshold,
		Flags:          av.WaypointFlagFlyOver | av.WaypointFlagHasAltRestriction,
		Heading:        int16(proc.Heading),
		AltRestriction: av.MakeAtAltitudeRestriction(altitude),
		Extra: &av.WaypointExtra{
			GoAroundContactController: proc.HandoffController,
		},
	}

	ac.Nav.GoAroundWithProcedure(altitude, wp)

	holdRunways := append([]string{runway}, proc.HoldDepartures...)
	s.holdDeparturesForGoAround(airport, holdRunways, proc.HandoffController)
}

// getGoAroundController returns the TCP that should handle a go-around for the given aircraft.
// Lookup priority: go_around_assignments for airport/runway, airport, then departure_assignments for airport.
func (s *Sim) getGoAroundController(ac *Aircraft) TCP {
	airport := ac.FlightPlan.ArrivalAirport
	runway := ""
	if ac.Nav.Approach.Assigned != nil {
		runway = ac.Nav.Approach.Assigned.Runway
	}

	// Check go_around_assignments for specific runway
	if runway != "" {
		if tcp, ok := s.GoAroundAssignments[airport+"/"+runway]; ok {
			return tcp
		}
	}

	// Check go_around_assignments for airport
	if tcp, ok := s.GoAroundAssignments[airport]; ok {
		return tcp
	}

	// Fall back to departure_assignments for airport
	if tcp, ok := s.DepartureAssignments[airport]; ok {
		return tcp
	}

	// We shouldn't get here but just in case--current controller
	return TCP(ac.ControllerFrequency)
}

// holdDeparturesForGoAround sets GoAroundHoldUntil on the specified runways and
// posts a status message to the go-around controller.
func (s *Sim) holdDeparturesForGoAround(airport string, holdRunways []string, goAroundTCP TCP) {
	if len(holdRunways) == 0 {
		return
	}

	depState, ok := s.DepartureState[airport]
	if !ok {
		return
	}

	holdUntil := s.State.SimTime.Add(time.Minute)

	// Set the hold state on matching runways
	for rwy, state := range depState {
		rwyBase := rwy.Base()
		for _, holdRwy := range holdRunways {
			if rwyBase == av.RunwayID(holdRwy).Base() {
				state.GoAroundHoldUntil = holdUntil
				s.lg.Info("holding departures on runway due to go-around",
					slog.String("airport", airport), slog.String("runway", rwyBase))
			}
		}
	}

	s.eventStream.Post(Event{
		Type:         StatusMessageEvent,
		ToController: ControlPosition(goAroundTCP),
		WrittenText:  fmt.Sprintf("%s DEPARTURES HELD FOR 1 MINUTE", airport),
	})
}

// getGoAroundProcedureForAircraft returns the go-around procedure defined for the
// aircraft's arrival airport/runway, if one exists in the scenario's arrival_runways.
func (s *Sim) getGoAroundProcedureForAircraft(ac *Aircraft) *GoAroundProcedure {
	airport := ac.FlightPlan.ArrivalAirport
	runway := ac.Nav.Approach.Assigned.Runway

	// Find matching arrival runway with a go-around procedure
	for _, ar := range s.State.ArrivalRunways {
		if ar.Airport == airport && ar.Runway.Base() == runway && ar.GoAround != nil {
			return ar.GoAround
		}
	}

	approach := ac.Nav.Approach.Assigned
	return &GoAroundProcedure{
		Heading:           int(math.TrueToMagnetic(approach.RunwayHeading(s.State.NmPerLongitude), s.State.MagneticVariation) + 0.5),
		IsRunwayHeading:   true,
		Altitude:          1000 * int((ac.Nav.FlightState.ArrivalAirportElevation+2500)/1000),
		HandoffController: s.getGoAroundController(ac),
	}
}

// checkFinalApproachSpacing checks for spacing violations between IFR aircraft
// on the same final approach and triggers go-arounds when separation is insufficient.
func (s *Sim) checkFinalApproachSpacing() {
	if !s.State.LaunchConfig.EnableTowerGoArounds {
		return
	}

	type runwayKey struct{ airport, runway string }
	aircraftByRunway := make(map[runwayKey][]*Aircraft)

	// Group IFR aircraft with assigned approaches by airport+runway
	for _, ac := range s.Aircraft {
		// Only tower sends aircraft around; don't include ones that have already been sent around
		// since presumably we'll have vertical separation soon if not already.
		if ac.Nav.Approach.Assigned != nil && ac.GotContactTower && !ac.SentAroundForSpacing {
			key := runwayKey{ac.FlightPlan.ArrivalAirport, ac.Nav.Approach.Assigned.Runway}
			aircraftByRunway[key] = append(aircraftByRunway[key], ac)
		}
	}

	for _, aircraft := range aircraftByRunway {
		// Sort by distance to threshold (closest first)
		threshold := aircraft[0].Nav.Approach.Assigned.Threshold
		slices.SortFunc(aircraft, func(a, b *Aircraft) int {
			return cmp.Compare(math.NMDistance2LL(a.Position(), threshold),
				math.NMDistance2LL(b.Position(), threshold))
		})

		// Check each adjacent pair
		for i := 1; i < len(aircraft); i++ {
			front, trailing := aircraft[i-1], aircraft[i]

			// Get required separation
			vol := trailing.ATPAVolume()
			eligible25nm := vol != nil && vol.Enable25nmApproach &&
				s.State.IsATPAVolume25nmEnabled(vol.Id) &&
				trailing.OnExtendedCenterline(0.2) && front.OnExtendedCenterline(0.2)
			reqSep := av.CWTApproachSeparation(front.CWT(), trailing.CWT(), eligible25nm)

			actualSep := math.NMDistance2LL(front.Position(), trailing.Position())

			majorBust := actualSep < reqSep*0.8
			minorBust := actualSep < reqSep*0.9

			// >20% violation: always go around
			// >10% but <=20% violation: 50% chance (one-time roll); skip check if already declined
			issueGoAround := majorBust || (minorBust && !trailing.SpacingGoAroundDeclined && s.Rand.Float32() < 0.5)
			if issueGoAround {
				if s.trySpacingOrbit(trailing, threshold) {
					continue
				}
				s.goAroundForSpacing(trailing)
			} else if minorBust {
				trailing.SpacingGoAroundDeclined = true
			}
		}
	}
}

// goAroundForSpacing initiates a tower-commanded go-around for spacing violations.
func (s *Sim) goAroundForSpacing(ac *Aircraft) {
	ac.SentAroundForSpacing = true
	s.goAround(ac)
}

// GoAroundProcedure defines go-around parameters for a specific arrival runway.
type GoAroundProcedure struct {
	Heading           int      `json:"heading"` // degrees 1-360; 0 (or unset) means runway heading
	IsRunwayHeading   bool     // true when heading was 0 (runway heading) before resolution
	Altitude          int      `json:"altitude"`           // feet, e.g., 2000, 3000
	HandoffController TCP      `json:"handoff_controller"` // TCP (e.g., "1D")
	HoldDepartures    []string `json:"hold_departures"`    // runways to hold, empty = no holds
}

// tryPatternGoAround sends a light piston going around from a visual
// approach (or as a VFR arrival) back into the traffic pattern -- upwind,
// crosswind, downwind -- where the pilot asks to be resequenced, instead
// of flying a straight-out departure-leg go-around.
func (s *Sim) tryPatternGoAround(ac *Aircraft, airport, runway string) bool {
	perf, ok := av.DB.AircraftPerformance[ac.FlightPlan.AircraftType]
	if !ok || perf.Engine.AircraftType != "P" {
		return false
	}
	if ac.FlightPlan.Rules == av.FlightRulesIFR {
		ap := ac.Nav.Approach.Assigned
		if ap == nil || (ap.Type != av.VisualApproach && ap.Type != av.ChartedVisualApproach) {
			return false
		}
	}
	rwy, ok := av.LookupRunway(airport, runway)
	if !ok {
		return false
	}
	opp, ok := av.LookupOppositeRunway(airport, runway)
	if !ok {
		return false
	}
	elevation := 0
	if ap, ok := av.DB.Airports[airport]; ok {
		elevation = ap.Elevation
	}

	b := newPatternBuilder(rwy, elevation, s.State.NmPerLongitude, s.State.MagneticVariation)
	rwyLen := math.NMDistance2LL(rwy.Threshold, opp.Threshold)

	// Pattern side: parallels dictate it -- a runway ending in R flies
	// right traffic, L flies left; otherwise standard left traffic. The
	// pattern builder's lateral offsets are left-positive, so right
	// traffic negates them.
	pdist := float32(0.75) // pattern lateral offset (nm)
	if strings.HasSuffix(runway, "R") {
		pdist = -pdist
	}
	const upwindExt = 0.5 // extension past the departure end

	// A full closed circuit back to the runway: the aircraft stays in
	// the tower's pattern and re-lands, rather than returning to the
	// approach controller. (The tower resequences pattern traffic; with
	// tower go-arounds enabled, the spacing check can send it around
	// again if the gap doesn't develop.)
	wps := av.WaypointArray{
		b.waypoint("_ga_upwind", rwyLen+upwindExt, 0, 500, 80, av.VFRPhaseUpwind),
		b.waypoint("_ga_crosswind", rwyLen+upwindExt, pdist, 800, 80, av.VFRPhaseCrosswind),
		b.waypoint("_ga_downwind", rwyLen/2, pdist, 1000, 80, av.VFRPhaseDownwind),
		b.waypoint("_ga_late_downwind", -0.5, pdist, 1000, 80, av.VFRPhaseDownwind),
		b.waypoint("_ga_base", -1.0, pdist/2, 500, 70, av.VFRPhaseBase),
		b.waypoint("_ga_final", -1.0, 0, 200, 65, av.VFRPhaseFinal),
	}
	threshold := b.waypoint("_ga_threshold", 0, 0, 0, 60, av.VFRPhaseFinal)
	threshold.SetLand(true)
	threshold.SetFlyOver(true)
	wps = append(wps, threshold)

	// Stay with the tower; the go-around call goes out on the current
	// frequency. Restore the tower-contact flag the go-around reset so
	// the tower's final-spacing checks keep watching the aircraft.
	if s.isVirtualController(ac.ControllerFrequency) || ac.ControllerFrequency == "_TOWER" {
		ac.GotContactTower = true
	}
	s.enqueuePilotTransmission(ac.ADSBCallsign, TCP(ac.ControllerFrequency), PendingTransmissionGoAround)

	ac.Nav.GoAroundWithRoute(float32(elevation+1000), wps)
	ac.PatternCircuitRunway = runway
	return true
}

// tryPublishedMissed has an IFR arrival on an instrument approach fly the
// published missed approach when the CIFP provides one and the scenario
// doesn't adapt an explicit go-around procedure for the runway.
func (s *Sim) tryPublishedMissed(ac *Aircraft, approach *av.Approach, proc *GoAroundProcedure) bool {
	if ac.FlightPlan.Rules != av.FlightRulesIFR {
		return false
	}
	if approach.Type == av.VisualApproach || approach.Type == av.ChartedVisualApproach {
		return false
	}
	if len(approach.MissedApproach) == 0 {
		return false
	}
	// A scenario-adapted go-around procedure takes precedence.
	for _, ar := range s.State.ArrivalRunways {
		if ar.Airport == ac.FlightPlan.ArrivalAirport && ar.Runway.Base() == approach.Runway && ar.GoAround != nil {
			return false
		}
	}

	wps := slices.Clone(approach.MissedApproach)
	extra := av.WaypointExtra{GoAroundContactController: proc.HandoffController}
	if wps[0].Extra != nil {
		extra = *wps[0].Extra
		extra.GoAroundContactController = proc.HandoffController
	}
	wps[0].Extra = &extra

	ac.Nav.GoAroundWithRoute(float32(proc.Altitude), wps)
	return true
}

// trySpacingOrbit has the tower spin a pattern-capable aircraft for
// spacing -- a 360 away from the final -- instead of sending it around.
// Only light pistons on a visual (or VFR) that are still far enough out
// qualify; jets and short-final traffic still go around.
func (s *Sim) trySpacingOrbit(ac *Aircraft, threshold math.Point2LL) bool {
	if ac.Nav.Orbit != nil {
		return true // already orbiting; give it time to develop spacing
	}
	perf, ok := av.DB.AircraftPerformance[ac.FlightPlan.AircraftType]
	if !ok || perf.Engine.AircraftType != "P" {
		return false
	}
	if ac.FlightPlan.Rules == av.FlightRulesIFR {
		ap := ac.Nav.Approach.Assigned
		if ap == nil || (ap.Type != av.VisualApproach && ap.Type != av.ChartedVisualApproach) {
			return false
		}
	}
	// Inside ~3.5nm a spin is worse than a go-around.
	if math.NMDistance2LL(ac.Position(), threshold) < 3.5 {
		return false
	}

	// No radio call on the approach frequency: the aircraft is with the
	// tower, and that exchange happens there. The turn itself is the
	// only thing the approach scope sees.
	ac.Nav.StartOrbit(s.orbitAwayFrom(ac, threshold), 360)
	return true
}

// orbitAwayFrom returns the 360 direction whose initial turn takes the
// aircraft away from the given point.
func (s *Sim) orbitAwayFrom(ac *Aircraft, p math.Point2LL) av.TurnDirection {
	nmPerLong := s.State.NmPerLongitude
	acNM := math.LL2NM(ac.Position(), nmPerLong)
	toP := math.Sub2f(math.LL2NM(p, nmPerLong), acNM)
	hdg := math.MagneticToTrue(ac.Nav.FlightState.Heading, s.State.MagneticVariation)
	hv := math.SinCos(math.Radians(float32(hdg)))
	if hv[0]*toP[1]-hv[1]*toP[0] > 0 {
		// The point is to the left of the aircraft's heading; orbit right.
		return av.TurnRight
	}
	return av.TurnLeft
}

// managePatternCircuits is the virtual tower working its pattern: an
// aircraft on a post-go-around circuit approaching its base turn is held
// on the downwind -- spun in place -- while the final is occupied, and
// turns base once the gap exists.
func (s *Sim) managePatternCircuits() {
	for _, ac := range s.Aircraft {
		if ac.PatternCircuitRunway == "" || ac.Nav.Orbit != nil {
			continue
		}
		wps := ac.Nav.Waypoints
		if len(wps) == 0 || !strings.HasSuffix(wps[0].Fix, "_ga_base") {
			continue
		}
		// Only decide when the base turn is imminent.
		if math.NMDistance2LL(ac.Position(), wps[0].Location) > 1.0 {
			continue
		}
		if s.finalOccupied(ac) {
			if rwy, ok := av.LookupRunway(ac.FlightPlan.ArrivalAirport, ac.PatternCircuitRunway); ok {
				ac.Nav.StartOrbit(s.orbitAwayFrom(ac, rwy.Threshold), 360)
			}
		}
	}
}

// finalOccupied reports whether another aircraft is inbound to the
// circuit aircraft's runway inside 5nm of the threshold.
func (s *Sim) finalOccupied(circuit *Aircraft) bool {
	rwy, ok := av.LookupRunway(circuit.FlightPlan.ArrivalAirport, circuit.PatternCircuitRunway)
	if !ok {
		return false
	}
	rwyBase := av.RunwayID(circuit.PatternCircuitRunway).Base()
	for _, other := range s.Aircraft {
		if other == circuit || other.FlightPlan.ArrivalAirport != circuit.FlightPlan.ArrivalAirport {
			continue
		}
		onFinal := false
		if ap := other.Nav.Approach.Assigned; ap != nil && av.RunwayID(ap.Runway).Base() == rwyBase {
			onFinal = true
		} else if other.PatternCircuitRunway != "" &&
			av.RunwayID(other.PatternCircuitRunway).Base() == rwyBase &&
			len(other.Nav.Waypoints) > 0 && !strings.HasSuffix(other.Nav.Waypoints[0].Fix, "_ga_base") {
			// Another circuit aircraft already inside its own base turn.
			onFinal = true
		}
		if onFinal && math.NMDistance2LL(other.Position(), rwy.Threshold) < 5 {
			return true
		}
	}
	return false
}
