// Ported from box3d test/test_large_world.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import "base:runtime"
import "core:testing"

import b3 "../../box3d"

@(private = "file")
STACK_COUNT :: 6
@(private = "file")
MAX_STEPS :: 400

@(private = "file")
StackResult :: struct {
	// Final body positions relative to the base, so origin and far runs are directly comparable
	relativePositions: [STACK_COUNT]b3.Vec3,

	// First step on which the top body fell asleep, or -1 if it never settled
	sleepStep:         i32,
}

// Drop a short stack of boxes onto a ground box centered at baseX. Records each body's final
// position relative to the base and the step on which the stack settles.
@(private = "file")
RunStack :: proc(baseX: f32) -> StackResult {
	base := b3.Pos{baseX, 0.0, 0.0}

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	groundDef := b3.DefaultBodyDef()
	groundDef.position = base
	groundId := b3.CreateBody(worldId, &groundDef)
	groundBox := b3.MakeBoxHull(10.0, 1.0, 10.0)
	groundShapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(groundId, &groundShapeDef, &groundBox.base)

	bodies: [STACK_COUNT]b3.BodyId
	for i: i32 = 0; i < STACK_COUNT; i += 1 {
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.OffsetPos(base, b3.Vec3{0.0, 2.0 + 1.05 * f32(i), 0.0})
		bodies[i] = b3.CreateBody(worldId, &bodyDef)

		box := b3.MakeCubeHull(0.5)
		shapeDef := b3.DefaultShapeDef()
		shapeDef.density = 1.0
		b3.CreateHullShape(bodies[i], &shapeDef, &box.base)
	}

	result: StackResult = {}
	result.sleepStep = -1

	for step: i32 = 0; step < MAX_STEPS; step += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)
		if result.sleepStep < 0 && b3.Body_IsAwake(bodies[STACK_COUNT - 1]) == false {
			result.sleepStep = step
		}
	}

	for i: i32 = 0; i < STACK_COUNT; i += 1 {
		p := b3.Body_GetPosition(bodies[i])
		result.relativePositions[i] = b3.SubPos(p, base)
	}

	b3.DestroyWorld(worldId)
	return result
}

// A stack at the origin and a stack far from the origin should behave identically in double
// precision: same settle frame, same relative geometry.
@(private = "file")
LargeWorldStackTest :: proc(t: ^testing.T) -> bool {
	origin := RunStack(0.0)
	ENSURE(t, origin.sleepStep >= 0) or_return

	// unified: not ported: the BOX3D_DOUBLE_PRECISION far-from-origin run (single precision only)

	return true
}

// Fire a fast bullet at a thin wall. Returns the bullet's final x relative to the base. If
// continuous collision works the bullet stops at the wall instead of tunneling past it.
@(private = "file")
RunBullet :: proc(baseX: f32) -> f32 {
	base := b3.Pos{baseX, 0.0, 0.0}

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	// Thin static wall at x = base + 5, spanning y and z
	wallDef := b3.DefaultBodyDef()
	wallDef.type = .staticBody
	wallDef.position = b3.OffsetPos(base, b3.Vec3{5.0, 0.0, 0.0})
	wallId := b3.CreateBody(worldId, &wallDef)
	wallBox := b3.MakeBoxHull(0.05, 5.0, 5.0)
	wallShapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(wallId, &wallShapeDef, &wallBox.base)

	// Small fast bullet aimed at the wall, no gravity
	bulletDef := b3.DefaultBodyDef()
	bulletDef.type = .dynamicBody
	bulletDef.isBullet = true
	bulletDef.gravityScale = 0.0
	bulletDef.position = base
	bulletDef.linearVelocity = b3.Vec3{200.0, 0.0, 0.0}
	bulletId := b3.CreateBody(worldId, &bulletDef)
	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.1}
	bulletShapeDef := b3.DefaultShapeDef()
	bulletShapeDef.density = 1.0
	b3.CreateSphereShape(bulletId, &bulletShapeDef, &sphere)

	for step: i32 = 0; step < 30; step += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)
	}

	relative := b3.SubPos(b3.Body_GetPosition(bulletId), base)
	b3.DestroyWorld(worldId)
	return relative.x
}

// The bullet must be caught by the wall, not tunnel through it. The blocking check is that this
// still holds far from the origin where the swept query box rounds back to float with large ULP.
@(private = "file")
LargeWorldBulletTest :: proc(t: ^testing.T) -> bool {
	// Wall front face is at x = 5 - 0.05; the bullet radius is 0.1, so a caught bullet stays well
	// short of the wall center at x = 5.
	originX := RunBullet(0.0)
	ENSURE(t, originX < 5.0) or_return

	// unified: not ported: the BOX3D_DOUBLE_PRECISION far-from-origin run (single precision only)

	return true
}

@(private = "file")
QueryResult :: struct {
	castHit:       bool,
	castRelX:      f32, // shape cast hit point x relative to the base
	overlapHit:    bool,
	moverFraction: f32,
	planeCount:    i32,
	rayHit:        bool,
	rayRelX:       f32, // world ray cast hit point x relative to the base
	shapeRayHit:   bool,
	shapeRayRelX:  f32, // direct shape ray cast hit point x relative to the base
}

@(private = "file")
CastContext :: struct {
	base: b3.Pos,
	hit:  bool,
	relX: f32,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
QueryCastCallback :: proc "c" (
	shapeId: b3.ShapeId,
	point: b3.Pos,
	normal: b3.Vec3,
	fraction: f32,
	materialId: u64,
	triangleIndex: i32,
	childIndex: i32,
	context_: rawptr,
) -> f32 {
	context = runtime.default_context()
	// unified end
	_, _, _, _, _ = shapeId, normal, materialId, triangleIndex, childIndex
	ctx := (^CastContext)(context_)
	ctx.hit = true
	ctx.relX = b3.SubPos(point, ctx.base).x
	return fraction
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
QueryOverlapCallback :: proc "c" (shapeId: b3.ShapeId, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	_ = shapeId
	(^bool)(context_)^ = true
	return true
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
QueryPlaneCallback :: proc "c" (shapeId: b3.ShapeId, planes: ^b3.PlaneResult, count: i32, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	_, _ = shapeId, planes
	(^i32)(context_)^ += count
	return true
}

// Run the four origin relative spatial queries against a static box centered at the base. The query
// inputs are all relative to the base, so passing base as the origin keeps them precise far out.
@(private = "file")
RunQueries :: proc(baseX: f32) -> QueryResult {
	base := b3.Pos{baseX, 0.0, 0.0}

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .staticBody
	bodyDef.position = base
	bodyId := b3.CreateBody(worldId, &bodyDef)
	box := b3.MakeBoxHull(1.0, 1.0, 1.0)
	shapeDef := b3.DefaultShapeDef()
	shapeId := b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	b3.World_Step(worldId, 1.0 / 60.0, 1)

	result: QueryResult = {}

	// Sphere proxy swept from the left into the box, hitting the left face at relative x = -1
	castPoint := b3.Vec3{-5.0, 0.0, 0.0}
	castProxy := b3.ShapeProxy{&castPoint, 1, 0.25}
	castCtx := CastContext{base, false, 0.0}
	b3.World_CastShape(
		worldId,
		base,
		&castProxy,
		b3.Vec3{10.0, 0.0, 0.0},
		b3.DefaultQueryFilter(),
		QueryCastCallback,
		&castCtx,
	)
	result.castHit = castCtx.hit
	result.castRelX = castCtx.relX

	// Sphere proxy sitting at the box center
	overlapPoint := b3.Vec3{0.0, 0.0, 0.0}
	overlapProxy := b3.ShapeProxy{&overlapPoint, 1, 0.5}
	overlapHit := false
	b3.World_OverlapShape(worldId, base, &overlapProxy, b3.DefaultQueryFilter(), QueryOverlapCallback, &overlapHit)
	result.overlapHit = overlapHit

	// Capsule swept from the left into the box
	moverCast := b3.Capsule{{-5.0, -0.3, 0.0}, {-5.0, 0.3, 0.0}, 0.25}
	result.moverFraction = b3.World_CastMover(
		worldId,
		base,
		&moverCast,
		b3.Vec3{10.0, 0.0, 0.0},
		b3.DefaultQueryFilter(),
		nil,
		nil,
	)

	// Capsule overlapping the left face, should report a contact plane
	moverCollide := b3.Capsule{{-1.1, -0.3, 0.0}, {-1.1, 0.3, 0.0}, 0.3}
	planeCount: i32 = 0
	b3.World_CollideMover(worldId, base, &moverCollide, b3.DefaultQueryFilter(), QueryPlaneCallback, &planeCount)
	result.planeCount = planeCount

	// World ray cast from the left into the box, hitting the left face at relative x = -1
	rayOrigin := b3.OffsetPos(base, b3.Vec3{-5.0, 0.0, 0.0})
	ray := b3.World_CastRayClosest(worldId, rayOrigin, b3.Vec3{10.0, 0.0, 0.0}, b3.DefaultQueryFilter())
	result.rayHit = ray.hit
	result.rayRelX = ray.hit ? b3.SubPos(ray.point, base).x : 0.0

	// Direct shape ray cast against the same box
	shapeRay := b3.Shape_RayCast(shapeId, rayOrigin, b3.Vec3{10.0, 0.0, 0.0})
	result.shapeRayHit = shapeRay.hit
	result.shapeRayRelX = shapeRay.hit ? b3.SubPos(shapeRay.point, base).x : 0.0

	b3.DestroyWorld(worldId)
	return result
}

// The origin relative queries hit at the origin and stay precise far from it: same hit geometry and
// sweep fraction whether the box is at the origin or at 1e7.
@(private = "file")
LargeWorldQueryTest :: proc(t: ^testing.T) -> bool {
	origin := RunQueries(0.0)
	ENSURE(t, origin.castHit) or_return
	ENSURE(t, origin.overlapHit) or_return
	ENSURE(t, origin.moverFraction < 1.0) or_return
	ENSURE(t, origin.planeCount > 0) or_return
	ENSURE_SMALL(t, origin.castRelX + 1.0, 0.05) or_return
	ENSURE(t, origin.rayHit) or_return
	ENSURE_SMALL(t, origin.rayRelX + 1.0, 0.05) or_return
	ENSURE(t, origin.shapeRayHit) or_return
	ENSURE_SMALL(t, origin.shapeRayRelX + 1.0, 0.05) or_return

	// unified: not ported: the BOX3D_DOUBLE_PRECISION far-from-origin run (single precision only)

	return true
}

@(test)
LargeWorldTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, LargeWorldStackTest) {return}
	if !RUN_SUBTEST(t, LargeWorldBulletTest) {return}
	if !RUN_SUBTEST(t, LargeWorldQueryTest) {return}
}
