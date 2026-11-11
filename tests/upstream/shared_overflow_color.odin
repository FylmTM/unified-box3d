// Ported from box3d shared/overflow_color.c and shared/overflow_color.h at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import "core:math"

import b3 "../../box3d"

// One heavy dynamic hub surrounded by enough dynamic neighbors that the hub's
// degree in the dyn-dyn contact graph exceeds B3_DYNAMIC_COLOR_COUNT (= 20).
// The excess contacts land in the overflow color (B3_GRAPH_COLOR_COUNT - 1),
// exercising the b3*_Overflow solver path that no other scene reaches.
OverflowColorPileData :: struct {
	groundShapeId: b3.ShapeId,
	hubId:         b3.BodyId,
	neighborCount: i32,
}

@(private = "file")
OVERFLOW_PILE_RING_COUNT :: 5
@(private = "file")
OVERFLOW_PILE_PER_RING :: 5

CreateOverflowColorPile :: proc(worldId: b3.WorldId) -> OverflowColorPileData {
	data: OverflowColorPileData = {}
	data.neighborCount = OVERFLOW_PILE_RING_COUNT * OVERFLOW_PILE_PER_RING

	// Static ground (top surface at y = 0)
	{
		bodyDef := b3.DefaultBodyDef()
		bodyDef.position = b3.Pos{0.0, -1.0, 0.0}
		groundId := b3.CreateBody(worldId, &bodyDef)

		box := b3.MakeBoxHull(20.0, 1.0, 20.0)
		shapeDef := b3.DefaultShapeDef()
		data.groundShapeId = b3.CreateHullShape(groundId, &shapeDef, &box.base)
	}

	// Tall, heavy hub. Tall so neighbors can ring around it in multiple
	// vertical layers; heavy so it stays roughly still under uneven inward
	// pressure from the ring.
	hubHalfX: f32 = 0.5
	hubHalfY: f32 = 2.5
	hubHalfZ: f32 = 0.5
	{
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{0.0, hubHalfY, 0.0}
		data.hubId = b3.CreateBody(worldId, &bodyDef)

		box := b3.MakeBoxHull(hubHalfX, hubHalfY, hubHalfZ)
		shapeDef := b3.DefaultShapeDef()
		shapeDef.density = 50.0
		b3.CreateHullShape(data.hubId, &shapeDef, &box.base)
	}

	// Neighbors: vertical rings around the hub, each box slightly overlapping
	// the hub so a contact exists on the very first step.
	neighborHalf: f32 = 0.2
	ringRadius := hubHalfX + neighborHalf - 0.03

	neighborBox := b3.MakeBoxHull(neighborHalf, neighborHalf, neighborHalf)
	neighborShape := b3.DefaultShapeDef()

	ringSpacing: f32 = 0.5
	baseY := neighborHalf + 0.05

	for ring: i32 = 0; ring < OVERFLOW_PILE_RING_COUNT; ring += 1 {
		y := baseY + ringSpacing * f32(ring)

		// Offset alternate rings by half a slot so neighbors don't sit
		// directly above each other.
		thetaOffset: f32 = (ring & 1) != 0 ? (b3.PI / OVERFLOW_PILE_PER_RING) : 0.0

		for slot: i32 = 0; slot < OVERFLOW_PILE_PER_RING; slot += 1 {
			theta := thetaOffset + (2.0 * b3.PI * f32(slot)) / OVERFLOW_PILE_PER_RING

			bodyDef := b3.DefaultBodyDef()
			bodyDef.type = .dynamicBody
			// unified: core:math cos/sin for C's cosf/sinf (test scene setup, not simulation code)
			bodyDef.position = b3.Pos{ringRadius * math.cos(theta), y, ringRadius * math.sin(theta)}
			// unified end
			bodyId := b3.CreateBody(worldId, &bodyDef)

			b3.CreateHullShape(bodyId, &neighborShape, &neighborBox.base)
		}
	}

	return data
}
