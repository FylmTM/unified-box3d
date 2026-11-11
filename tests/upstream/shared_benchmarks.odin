// Ported from box3d shared/benchmarks.c and shared/benchmarks.h at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"

// unified: only the scenes the upstream tests use are ported (the junkyard and the ground shape id);
// the other benchmark scenes live with the benchmarks.

// This allows benchmarks to be tested on the benchmark app and also visualized in the samples app

// unified: C's NDEBUG maps to Odin's -disable-assert
@(private = "file")
BENCHMARK_DEBUG :: !ODIN_DISABLE_ASSERT
// unified end

@(private = "file")
g_groundShapeId: b3.ShapeId = {}

// This is used by the sample app to draw a ground grid. Should only
// be used for box hulls.
GetGroundShapeId :: proc() -> b3.ShapeId {
	return g_groundShapeId
}

ResetGroundShapeId :: proc() {
	g_groundShapeId = b3.nullShapeId
}

@(private = "file")
JunkyardData :: struct {
	pusherId: b3.BodyId,
	degrees:  f32,
	radius:   f32,
}

@(private = "file")
g_junkyardData: JunkyardData

CreateJunkyard :: proc(worldId: b3.WorldId) {
	groundId: b3.BodyId
	{
		bodyDef := b3.DefaultBodyDef()
		bodyDef.position.y = -1.0
		groundId = b3.CreateBody(worldId, &bodyDef)
	}

	{
		shapeDef := b3.DefaultShapeDef()
		{
			box := b3.MakeBoxHull(120.0, 1.0, 120.0)
			g_groundShapeId = b3.CreateHullShape(groundId, &shapeDef, &box.base)
		}
		{
			offset := b3.Vec3{-50.0, 8.0, 0.0}
			box := b3.MakeOffsetBoxHull(1.0, 8.0, 50.0, offset)
			b3.CreateHullShape(groundId, &shapeDef, &box.base)
		}
		{
			offset := b3.Vec3{50.0, 8.0, 0.0}
			box := b3.MakeOffsetBoxHull(1.0, 8.0, 50.0, offset)
			b3.CreateHullShape(groundId, &shapeDef, &box.base)
		}
		{
			offset := b3.Vec3{0.0, 8.0, -50.0}
			box := b3.MakeOffsetBoxHull(50.0, 8.0, 1.0, offset)
			b3.CreateHullShape(groundId, &shapeDef, &box.base)
		}
		{
			offset := b3.Vec3{0.0, 8.0, 50.0}
			box := b3.MakeOffsetBoxHull(50.0, 8.0, 1.0, offset)
			b3.CreateHullShape(groundId, &shapeDef, &box.base)
		}
	}
	{
		rockHull := b3.CreateRock(1.5)

		count: i32 = BENCHMARK_DEBUG ? 2 : 24
		height: f32 = 24.0
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		shapeDef := b3.DefaultShapeDef()
		for Y: i32 = 0; Y < count; Y += 1 {
			for X: i32 = 0; X <= 20; X += 1 {
				for Z: i32 = 0; Z <= 20; Z += 1 {
					bodyDef.position.x = -40.0 + 4.0 * f32(X)
					bodyDef.position.y = 4.0 * f32(Y) + height + 1.0
					bodyDef.position.z = -40.0 + 4.0 * f32(Z)
					bodyId := b3.CreateBody(worldId, &bodyDef)
					b3.CreateHullShape(bodyId, &shapeDef, rockHull)
				}
			}
		}

		b3.DestroyHull(rockHull)
	}

	g_junkyardData.radius = 35.0
	mHeight: f32 = 24.0

	hull := b3.CreateCylinder(mHeight, 4.0, 0.0, 16)
	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .kinematicBody
	bodyDef.position = b3.Pos{g_junkyardData.radius, 0.0, 0.0}
	g_junkyardData.pusherId = b3.CreateBody(worldId, &bodyDef)
	g_junkyardData.degrees = 0.0
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(g_junkyardData.pusherId, &shapeDef, hull)
	b3.DestroyHull(hull)
}

StepJunkyard :: proc(worldId: b3.WorldId, stepCount: i32) {
	_ = worldId
	_ = stepCount

	timeStep: f32 = 1.0 / 60.0
	omega: f32 = -6.0
	g_junkyardData.degrees += omega * timeStep
	cs := b3.ComputeCosSin(g_junkyardData.degrees * b3.PI / 180.0)
	r := g_junkyardData.radius
	targetPos := b3.Pos{r * cs.cosine, 0.0, r * cs.sine}
	target := b3.WorldTransform {
		p = targetPos,
		q = b3.Quat_identity,
	}
	b3.Body_SetTargetTransform(g_junkyardData.pusherId, target, timeStep, false)
}
