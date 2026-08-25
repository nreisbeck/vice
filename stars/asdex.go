// stars/asdex.go
// Copyright(c) 2022-2026 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package stars

import (
	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/panes"
	"github.com/mmp/vice/radar"
	"github.com/mmp/vice/renderer"
)

// asdexMaxRange is the display range (nm) at or below which the surface
// view is drawn.
const asdexMaxRange = 4

// asdexRunwayHalfWidthNM is the half-width used to draw runway outlines;
// runway widths aren't in the database, so a representative value is used.
const asdexRunwayHalfWidthNM = 0.02

// drawASDEX draws an ASDE-X-style airport surface depiction--runway
// outlines and runway number labels for the scenario's airports--when the
// scope is zoomed in far enough that surface operations are what's being
// watched. Aircraft targets are drawn by the regular track rendering, so
// departures rolling and arrivals over the threshold appear on the surface
// view automatically.
func (sp *STARSPane) drawASDEX(ctx *panes.Context, transforms radar.ScopeTransformations, cb *renderer.CommandBuffer) {
	ps := sp.currentPrefs()
	if ps.Range > asdexMaxRange {
		return
	}

	td := renderer.GetTextDrawBuilder()
	defer renderer.ReturnTextDrawBuilder(td)
	ld := renderer.GetLinesDrawBuilder()
	defer renderer.ReturnLinesDrawBuilder(ld)

	color := ps.Brightness.Lists.ScaleRGB(sp.Colors.Compass)
	style := renderer.TextStyle{
		Font:  sp.systemFont(ctx, ps.CharSize.Tools),
		Color: color,
	}

	// Union of the scenario's airports.
	airports := make(map[string]interface{})
	for name := range ctx.Client.State.DepartureAirports {
		airports[name] = nil
	}
	for name := range ctx.Client.State.ArrivalAirports {
		airports[name] = nil
	}

	nmPerLongitude := ctx.NmPerLongitude
	for icao := range airports {
		ap, ok := av.DB.Airports[icao]
		if !ok {
			continue
		}

		for _, rwy := range ap.Runways {
			opp, ok := av.LookupOppositeRunway(icao, rwy.Id)
			if !ok {
				continue
			}
			// Draw each physical runway once: take the lexically smaller
			// id of the pair.
			if rwy.Id > opp.Id {
				continue
			}

			p0 := math.LL2NM(rwy.Threshold, nmPerLongitude)
			p1 := math.LL2NM(opp.Threshold, nmPerLongitude)
			v := math.Sub2f(p1, p0)
			if math.Length2f(v) == 0 {
				continue
			}
			dir := math.Normalize2f(v)
			perp := [2]float32{-dir[1], dir[0]}
			off := math.Scale2f(perp, asdexRunwayHalfWidthNM)

			corner := func(p [2]float32, o [2]float32) [2]float32 {
				return transforms.WindowFromLatLongP(math.NM2LL(math.Add2f(p, o), nmPerLongitude))
			}
			c00, c01 := corner(p0, off), corner(p0, math.Scale2f(off, -1))
			c10, c11 := corner(p1, off), corner(p1, math.Scale2f(off, -1))

			// Outline: two edges plus threshold lines at each end.
			ld.AddLine(c00, c10)
			ld.AddLine(c01, c11)
			ld.AddLine(c00, c01)
			ld.AddLine(c10, c11)

			// Runway number labels just beyond each threshold.
			label := func(p [2]float32, d [2]float32, id string) {
				pw := transforms.WindowFromLatLongP(math.NM2LL(math.Add2f(p, math.Scale2f(d, -0.12)), nmPerLongitude))
				td.AddTextCentered(id, pw, style)
			}
			label(p0, dir, rwy.Id)
			label(p1, math.Scale2f(dir, -1), opp.Id)
		}
	}

	transforms.LoadWindowViewingMatrices(cb)
	cb.LineWidth(1, ctx.DPIScale)
	cb.SetRGB(color)
	ld.GenerateCommands(cb)
	td.GenerateCommands(cb)
}
