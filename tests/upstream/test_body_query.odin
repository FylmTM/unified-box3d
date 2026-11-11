// Ported from box3d test/test_body_query.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "base:runtime"
import "core:testing"

// The per-body query functions take an explicit world origin and a world body transform.
// Everything is re-centered on the origin so the float collision math stays accurate far from
// the world origin. These tests pin that framing: results come back in world space, the supplied
// transform drives the geometry (not the body's stored pose), and a large origin offset must not
// change a hit fraction or normal.
//
// CastRay and CastShape never touch the body's stored transform, so a static body at the origin
// holding local-frame shapes is enough. No step is needed for any query.

@(private = "file")
CreateQueryWorld :: proc(bodyId: ^b3.BodyId) -> b3.WorldId {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)
	bodyDef := b3.DefaultBodyDef()
	bodyId^ = b3.CreateBody(worldId, &bodyDef)
	return worldId
}

@(private = "file")
IdentityAt :: proc(x: f32, y: f32, z: f32) -> b3.WorldTransform {
	return b3.WorldTransform{p = b3.Pos{x, y, z}, q = b3.Quat_identity}
}

// CastRay ----------------------------------------------------------------------------------

@(private = "file")
CastRayHitsSphere :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 1.0}
	shapeDef := b3.DefaultShapeDef()
	b3.CreateSphereShape(bodyId, &shapeDef, &sphere)

	// Body sphere at world (5,0,0), ray straight at it along +X.
	bodyTransform := IdentityAt(5.0, 0.0, 0.0)
	result := b3.Body_CastRay(
		bodyId,
		b3.Pos{0.0, 0.0, 0.0},
		b3.Vec3{10.0, 0.0, 0.0},
		b3.DefaultQueryFilter(),
		1.0,
		bodyTransform,
	)

	ENSURE(t, result.hit) or_return
	ENSURE(t, b3.Shape_IsValid(result.shapeId)) or_return
	ENSURE_SMALL(t, result.fraction - 0.4, 1e-5) or_return
	ENSURE_SMALL(t, result.normal.x + 1.0, 1e-5) or_return
	ENSURE_SMALL(t, result.normal.y, 1e-5) or_return
	ENSURE_SMALL(t, result.normal.z, 1e-5) or_return

	point := b3.ToVec3(result.point)
	ENSURE_SMALL(t, point.x - 4.0, 1e-4) or_return
	ENSURE_SMALL(t, point.y, 1e-4) or_return
	ENSURE_SMALL(t, point.z, 1e-4) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
CastRayMiss :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 1.0}
	shapeDef := b3.DefaultShapeDef()
	b3.CreateSphereShape(bodyId, &shapeDef, &sphere)

	// Ray runs parallel to the body, never reaching it.
	bodyTransform := IdentityAt(5.0, 0.0, 0.0)
	result := b3.Body_CastRay(
		bodyId,
		b3.Pos{0.0, 0.0, 0.0},
		b3.Vec3{0.0, 10.0, 0.0},
		b3.DefaultQueryFilter(),
		1.0,
		bodyTransform,
	)

	ENSURE(t, result.hit == false) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
CastRayClosestShape :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	shapeDef := b3.DefaultShapeDef()
	nearSphere := b3.Sphere{{0.0, 0.0, 0.0}, 1.0}
	farSphere := b3.Sphere{{4.0, 0.0, 0.0}, 1.0}
	nearId := b3.CreateSphereShape(bodyId, &shapeDef, &nearSphere)
	b3.CreateSphereShape(bodyId, &shapeDef, &farSphere)

	// Ray crosses both spheres; the loop must shrink maxFraction to the nearer hit.
	bodyTransform := IdentityAt(0.0, 0.0, 0.0)
	result := b3.Body_CastRay(
		bodyId,
		b3.Pos{-5.0, 0.0, 0.0},
		b3.Vec3{10.0, 0.0, 0.0},
		b3.DefaultQueryFilter(),
		1.0,
		bodyTransform,
	)

	ENSURE(t, result.hit) or_return
	ENSURE(t, result.shapeId.index1 == nearId.index1) or_return
	ENSURE(t, result.shapeId.generation == nearId.generation) or_return
	ENSURE_SMALL(t, result.fraction - 0.4, 1e-5) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
CastRayRotatedBody :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	// Local center (0,2,0) rotated +90 deg about Z lands at world (-2,0,0).
	sphere := b3.Sphere{{0.0, 2.0, 0.0}, 0.5}
	shapeDef := b3.DefaultShapeDef()
	b3.CreateSphereShape(bodyId, &shapeDef, &sphere)

	bodyTransform := b3.WorldTransform {
		p = b3.Pos{0.0, 0.0, 0.0},
		q = b3.MakeQuatFromAxisAngle(b3.Vec3{0.0, 0.0, 1.0}, 0.5 * b3.PI),
	}
	result := b3.Body_CastRay(
		bodyId,
		b3.Pos{0.0, 0.0, 0.0},
		b3.Vec3{-4.0, 0.0, 0.0},
		b3.DefaultQueryFilter(),
		1.0,
		bodyTransform,
	)

	ENSURE(t, result.hit) or_return
	ENSURE_SMALL(t, result.fraction - 0.375, 1e-5) or_return
	ENSURE_SMALL(t, result.normal.x - 1.0, 1e-5) or_return

	point := b3.ToVec3(result.point)
	ENSURE_SMALL(t, point.x + 1.5, 1e-4) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
CastRayFarFromOrigin :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 1.0}
	shapeDef := b3.DefaultShapeDef()
	b3.CreateSphereShape(bodyId, &shapeDef, &sphere)

	// Same geometry as CastRayHitsSphere shifted far from the world origin. The relative framing
	// keeps the subtraction exact, so fraction and normal must be unchanged.
	origin := b3.Pos{1.0e6, -2.0e6, 5.0e5}
	bodyTransform := b3.WorldTransform {
		p = b3.OffsetPos(origin, b3.Vec3{5.0, 0.0, 0.0}),
		q = b3.Quat_identity,
	}
	result := b3.Body_CastRay(bodyId, origin, b3.Vec3{10.0, 0.0, 0.0}, b3.DefaultQueryFilter(), 1.0, bodyTransform)

	ENSURE(t, result.hit) or_return
	ENSURE_SMALL(t, result.fraction - 0.4, 1e-5) or_return
	ENSURE_SMALL(t, result.normal.x + 1.0, 1e-5) or_return
	ENSURE_SMALL(t, result.normal.y, 1e-5) or_return
	ENSURE_SMALL(t, result.normal.z, 1e-5) or_return

	b3.DestroyWorld(worldId)
	return true
}

// CastShape --------------------------------------------------------------------------------

@(private = "file")
CastShapeHitsBox :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	box := b3.MakeBoxHull(1.0, 1.0, 1.0)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	// Sphere proxy of radius 0.5 cast along +X into a box whose front face is at world x = 4.
	point := b3.Vec3{0.0, 0.0, 0.0}
	proxy := b3.ShapeProxy{&point, 1, 0.5}
	bodyTransform := IdentityAt(5.0, 0.0, 0.0)
	result := b3.Body_CastShape(
		bodyId,
		b3.Pos{0.0, 0.0, 0.0},
		&proxy,
		b3.Vec3{10.0, 0.0, 0.0},
		b3.DefaultQueryFilter(),
		1.0,
		false,
		bodyTransform,
	)

	// Front face at world x = 4. The fraction carries a small shape-cast skin, the contact point
	// and normal do not.
	ENSURE(t, result.hit) or_return
	ENSURE(t, b3.Shape_IsValid(result.shapeId)) or_return
	ENSURE_SMALL(t, result.fraction - 0.35, 1e-2) or_return
	ENSURE_SMALL(t, result.normal.x + 1.0, 1e-4) or_return

	hit := b3.ToVec3(result.point)
	ENSURE_SMALL(t, hit.x - 4.0, 1e-3) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
CastShapeMiss :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	box := b3.MakeBoxHull(1.0, 1.0, 1.0)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	point := b3.Vec3{0.0, 0.0, 0.0}
	proxy := b3.ShapeProxy{&point, 1, 0.5}
	bodyTransform := IdentityAt(5.0, 0.0, 0.0)
	result := b3.Body_CastShape(
		bodyId,
		b3.Pos{0.0, 0.0, 0.0},
		&proxy,
		b3.Vec3{0.0, 10.0, 0.0},
		b3.DefaultQueryFilter(),
		1.0,
		false,
		bodyTransform,
	)

	ENSURE(t, result.hit == false) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
CastShapeRotatedBody :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	// Body sphere local center (0,2,0) rotated +90 deg about Z lands at world (-2,0,0).
	sphere := b3.Sphere{{0.0, 2.0, 0.0}, 1.0}
	shapeDef := b3.DefaultShapeDef()
	b3.CreateSphereShape(bodyId, &shapeDef, &sphere)

	point := b3.Vec3{0.0, 0.0, 0.0}
	proxy := b3.ShapeProxy{&point, 1, 0.5}
	bodyTransform := b3.WorldTransform {
		p = b3.Pos{0.0, 0.0, 0.0},
		q = b3.MakeQuatFromAxisAngle(b3.Vec3{0.0, 0.0, 1.0}, 0.5 * b3.PI),
	}
	result := b3.Body_CastShape(
		bodyId,
		b3.Pos{0.0, 0.0, 0.0},
		&proxy,
		b3.Vec3{-4.0, 0.0, 0.0},
		b3.DefaultQueryFilter(),
		1.0,
		false,
		bodyTransform,
	)

	ENSURE(t, result.hit) or_return
	ENSURE_SMALL(t, result.fraction - 0.125, 1e-2) or_return
	ENSURE_SMALL(t, result.normal.x - 1.0, 1e-4) or_return

	hit := b3.ToVec3(result.point)
	ENSURE_SMALL(t, hit.x + 1.0, 1e-3) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
CastShapeFarFromOrigin :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	box := b3.MakeBoxHull(1.0, 1.0, 1.0)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	point := b3.Vec3{0.0, 0.0, 0.0}
	proxy := b3.ShapeProxy{&point, 1, 0.5}
	origin := b3.Pos{1.0e6, -2.0e6, 5.0e5}
	bodyTransform := b3.WorldTransform {
		p = b3.OffsetPos(origin, b3.Vec3{5.0, 0.0, 0.0}),
		q = b3.Quat_identity,
	}
	result := b3.Body_CastShape(
		bodyId,
		origin,
		&proxy,
		b3.Vec3{10.0, 0.0, 0.0},
		b3.DefaultQueryFilter(),
		1.0,
		false,
		bodyTransform,
	)

	ENSURE(t, result.hit) or_return
	ENSURE_SMALL(t, result.fraction - 0.35, 1e-2) or_return
	ENSURE_SMALL(t, result.normal.x + 1.0, 1e-4) or_return

	b3.DestroyWorld(worldId)
	return true
}

// OverlapShape -----------------------------------------------------------------------------

@(private = "file")
OverlapTrue :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	box := b3.MakeBoxHull(1.0, 1.0, 1.0)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	// Proxy sits at the box center.
	point := b3.Vec3{0.0, 0.0, 0.0}
	proxy := b3.ShapeProxy{&point, 1, 0.5}
	bodyTransform := IdentityAt(5.0, 0.0, 0.0)
	overlaps := b3.Body_OverlapShape(bodyId, b3.Pos{5.0, 0.0, 0.0}, &proxy, b3.DefaultQueryFilter(), bodyTransform)

	ENSURE(t, overlaps) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
OverlapFalse :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	box := b3.MakeBoxHull(1.0, 1.0, 1.0)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	point := b3.Vec3{0.0, 0.0, 0.0}
	proxy := b3.ShapeProxy{&point, 1, 0.5}
	bodyTransform := IdentityAt(5.0, 0.0, 0.0)
	overlaps := b3.Body_OverlapShape(bodyId, b3.Pos{20.0, 0.0, 0.0}, &proxy, b3.DefaultQueryFilter(), bodyTransform)

	ENSURE(t, overlaps == false) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
OverlapRespectsBodyTransform :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	box := b3.MakeBoxHull(1.0, 1.0, 1.0)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	// Fixed proxy and origin: only the supplied transform decides the overlap.
	point := b3.Vec3{0.0, 0.0, 0.0}
	proxy := b3.ShapeProxy{&point, 1, 0.5}
	origin := b3.Pos{0.0, 0.0, 0.0}

	ENSURE(
		t,
		b3.Body_OverlapShape(bodyId, origin, &proxy, b3.DefaultQueryFilter(), IdentityAt(0.0, 0.0, 0.0)),
	) or_return
	ENSURE(
		t,
		b3.Body_OverlapShape(bodyId, origin, &proxy, b3.DefaultQueryFilter(), IdentityAt(20.0, 0.0, 0.0)) == false,
	) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
OverlapFilter :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	box := b3.MakeBoxHull(1.0, 1.0, 1.0)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	point := b3.Vec3{0.0, 0.0, 0.0}
	proxy := b3.ShapeProxy{&point, 1, 0.5}
	bodyTransform := IdentityAt(0.0, 0.0, 0.0)

	// Geometry overlaps, but a zero mask rejects every category.
	filter := b3.DefaultQueryFilter()
	filter.maskBits = 0
	overlaps := b3.Body_OverlapShape(bodyId, b3.Pos{0.0, 0.0, 0.0}, &proxy, filter, bodyTransform)

	ENSURE(t, overlaps == false) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
CountOverlapCallback :: proc "c" (shapeId: b3.ShapeId, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	_ = shapeId
	(^i32)(context_)^ += 1
	return true
}

// A baked compound keeps its tree and its child placements in the compound frame while the query
// proxy is in world space, so a compound on a turned body only answers correctly if the proxy is
// brought back into that frame first.
@(private = "file")
OverlapCompoundRotatedBody :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	mat := b3.DefaultSurfaceMaterial()
	spheres := [2]b3.CompoundSphereDef {
		{sphere = {{-3.0, 0.0, 0.0}, 0.5}, material = mat},
		{sphere = {{3.0, 0.0, 0.0}, 0.5}, material = mat},
	}
	compoundDef := b3.CompoundDef {
		spheres     = &spheres[0],
		sphereCount = 2,
	}
	compound := b3.CreateCompound(&compoundDef)
	ENSURE(t, compound != nil) or_return

	pose := b3.Transform {
		p = {10.0, 20.0, 30.0},
		q = b3.MakeQuatFromAxisAngle(b3.Vec3_axisZ, 0.5 * b3.PI),
	}

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .staticBody
	bodyDef.position = b3.Pos{pose.p.x, pose.p.y, pose.p.z}
	bodyDef.rotation = pose.q
	bodyId := b3.CreateBody(worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()
	b3.CreateBakedCompoundShape(bodyId, &shapeDef, compound)

	filter := b3.DefaultQueryFilter()

	// Sitting on the second child, which the body pose has swung onto the +Y side
	onChild := b3.TransformPoint(pose, b3.Vec3{3.0, 0.0, 0.0})
	hitProxy := b3.ShapeProxy{&onChild, 1, 0.1}

	hits: i32 = 0
	b3.World_OverlapShape(worldId, b3.Pos_zero, &hitProxy, filter, CountOverlapCallback, &hits)
	ENSURE(t, hits == 1) or_return

	bodyTransform := b3.WorldTransform {
		p = bodyDef.position,
		q = bodyDef.rotation,
	}
	ENSURE(t, b3.Body_OverlapShape(bodyId, b3.Pos_zero, &hitProxy, filter, bodyTransform)) or_return

	// The gap between the two children stays empty
	inGap := b3.TransformPoint(pose, b3.Vec3{0.0, 0.0, 0.0})
	gapProxy := b3.ShapeProxy{&inGap, 1, 0.25}

	hits = 0
	b3.World_OverlapShape(worldId, b3.Pos_zero, &gapProxy, filter, CountOverlapCallback, &hits)
	ENSURE(t, hits == 0) or_return
	ENSURE(t, b3.Body_OverlapShape(bodyId, b3.Pos_zero, &gapProxy, filter, bodyTransform) == false) or_return

	b3.DestroyWorld(worldId)
	b3.DestroyCompound(compound)
	return true
}

// A box hull proxy built around a world target with a zero origin must hit the same shapes as the
// same box built at the local origin and queried with the target as origin. This is the origin
// relative equivalence the world query promises, and the pattern users reach for when they bake a
// query box with b3MakeTransformedBoxHull.
@(private = "file")
OverlapHullProxyEquivalence :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .staticBody
	bodyDef.position = b3.Pos{10.0, 0.0, 0.0}
	bodyId := b3.CreateBody(worldId, &bodyDef)
	box := b3.MakeBoxHull(1.0, 1.0, 1.0)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	b3.World_Step(worldId, f32(1.0) / f32(60.0), 1)

	filter := b3.DefaultQueryFilter()

	// Overlapping target: a 10 wide query box centered on the body.
	{
		offset := b3.Vec3{10.0, 0.0, 0.0}

		baked := b3.MakeTransformedBoxHull(5.0, 5.0, 5.0, b3.Transform{offset, b3.Quat_identity})
		bakedProxy := b3.ShapeProxy{&baked.boxPoints[0], baked.base.vertexCount, 0.0}
		bakedHits: i32 = 0
		b3.World_OverlapShape(worldId, b3.Pos_zero, &bakedProxy, filter, CountOverlapCallback, &bakedHits)

		origin := b3.OffsetPos(b3.Pos_zero, offset)
		local := b3.MakeBoxHull(5.0, 5.0, 5.0)
		localProxy := b3.ShapeProxy{&local.boxPoints[0], local.base.vertexCount, 0.0}
		localHits: i32 = 0
		b3.World_OverlapShape(worldId, origin, &localProxy, filter, CountOverlapCallback, &localHits)

		ENSURE(t, bakedHits == 1) or_return
		ENSURE(t, localHits == bakedHits) or_return
	}

	// Clearing target: same box far from the body, both formulations agree on the miss.
	{
		offset := b3.Vec3{100.0, 0.0, 0.0}

		baked := b3.MakeTransformedBoxHull(5.0, 5.0, 5.0, b3.Transform{offset, b3.Quat_identity})
		bakedProxy := b3.ShapeProxy{&baked.boxPoints[0], baked.base.vertexCount, 0.0}
		bakedHits: i32 = 0
		b3.World_OverlapShape(worldId, b3.Pos_zero, &bakedProxy, filter, CountOverlapCallback, &bakedHits)

		origin := b3.OffsetPos(b3.Pos_zero, offset)
		local := b3.MakeBoxHull(5.0, 5.0, 5.0)
		localProxy := b3.ShapeProxy{&local.boxPoints[0], local.base.vertexCount, 0.0}
		localHits: i32 = 0
		b3.World_OverlapShape(worldId, origin, &localProxy, filter, CountOverlapCallback, &localHits)

		ENSURE(t, bakedHits == 0) or_return
		ENSURE(t, localHits == bakedHits) or_return
	}

	b3.DestroyWorld(worldId)
	return true
}

// A quarter turn baked into the query hull must reach the overlap test. A long thin bar hits a body
// off the origin when aligned along X and clears it once rotated to lie along Z.
@(private = "file")
OverlapHullProxyRotation :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .staticBody
	bodyDef.position = b3.Pos{3.0, 0.0, 0.0}
	bodyId := b3.CreateBody(worldId, &bodyDef)
	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	b3.World_Step(worldId, f32(1.0) / f32(60.0), 1)

	filter := b3.DefaultQueryFilter()

	// Bar long in local X, centered at the origin, reaches the body at x = 3.
	aligned := b3.MakeTransformedBoxHull(4.0, 0.3, 0.3, b3.Transform_identity)
	alignedProxy := b3.ShapeProxy{&aligned.boxPoints[0], aligned.base.vertexCount, 0.0}
	alignedHits: i32 = 0
	b3.World_OverlapShape(worldId, b3.Pos_zero, &alignedProxy, filter, CountOverlapCallback, &alignedHits)
	ENSURE(t, alignedHits == 1) or_return

	// Rotated a quarter turn about Y the long axis points along Z, so the bar no longer reaches x = 3.
	q := b3.MakeQuatFromAxisAngle(b3.Vec3{0.0, 1.0, 0.0}, 0.5 * b3.PI)
	turned := b3.MakeTransformedBoxHull(4.0, 0.3, 0.3, b3.Transform{b3.Vec3_zero, q})
	turnedProxy := b3.ShapeProxy{&turned.boxPoints[0], turned.base.vertexCount, 0.0}
	turnedHits: i32 = 0
	b3.World_OverlapShape(worldId, b3.Pos_zero, &turnedProxy, filter, CountOverlapCallback, &turnedHits)
	ENSURE(t, turnedHits == 0) or_return

	b3.DestroyWorld(worldId)
	return true
}

// CollideMover -----------------------------------------------------------------------------

@(private = "file")
MoverTouchesBox :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	// Mover core runs above the +Y face; its 0.2 radius reaches 0.1 into it.
	mover := b3.Capsule{{-0.3, 0.6, 0.0}, {0.3, 0.6, 0.0}, 0.2}
	planes: [4]b3.BodyPlaneResult
	bodyTransform := IdentityAt(0.0, 0.0, 0.0)
	count := b3.Body_CollideMover(
		bodyId,
		&planes[0],
		4,
		b3.Pos{0.0, 0.0, 0.0},
		&mover,
		b3.DefaultQueryFilter(),
		bodyTransform,
	)

	ENSURE(t, count == 1) or_return
	ENSURE(t, b3.Shape_IsValid(planes[0].shapeId)) or_return
	ENSURE(t, b3.IsNormalized(planes[0].result.plane.normal)) or_return
	ENSURE(t, planes[0].result.plane.normal.y > 0.99) or_return
	ENSURE_SMALL(t, planes[0].result.plane.offset - 0.1, 1e-4) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
MoverSeparated :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	mover := b3.Capsule{{-0.3, 5.0, 0.0}, {0.3, 5.0, 0.0}, 0.2}
	planes: [4]b3.BodyPlaneResult
	bodyTransform := IdentityAt(0.0, 0.0, 0.0)
	count := b3.Body_CollideMover(
		bodyId,
		&planes[0],
		4,
		b3.Pos{0.0, 0.0, 0.0},
		&mover,
		b3.DefaultQueryFilter(),
		bodyTransform,
	)

	ENSURE(t, count == 0) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
MoverRotatedBody :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	// Rotating +90 deg about X turns the local +Y face toward world +Z. The mover sits above the
	// world +Z face, so the returned normal must come back rotated into world space.
	mover := b3.Capsule{{-0.3, 0.0, 0.6}, {0.3, 0.0, 0.6}, 0.2}
	planes: [4]b3.BodyPlaneResult
	bodyTransform := b3.WorldTransform {
		p = b3.Pos{0.0, 0.0, 0.0},
		q = b3.MakeQuatFromAxisAngle(b3.Vec3{1.0, 0.0, 0.0}, 0.5 * b3.PI),
	}
	count := b3.Body_CollideMover(
		bodyId,
		&planes[0],
		4,
		b3.Pos{0.0, 0.0, 0.0},
		&mover,
		b3.DefaultQueryFilter(),
		bodyTransform,
	)

	ENSURE(t, count == 1) or_return
	ENSURE(t, b3.IsNormalized(planes[0].result.plane.normal)) or_return
	ENSURE(t, planes[0].result.plane.normal.z > 0.99) or_return
	ENSURE_SMALL(t, planes[0].result.plane.offset - 0.1, 1e-4) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
MoverCapacity :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	// Two spheres each touch a mover that runs between them along X at y = 0.
	shapeDef := b3.DefaultShapeDef()
	left := b3.Sphere{{-0.4, 0.6, 0.0}, 0.5}
	right := b3.Sphere{{0.4, 0.6, 0.0}, 0.5}
	b3.CreateSphereShape(bodyId, &shapeDef, &left)
	b3.CreateSphereShape(bodyId, &shapeDef, &right)

	mover := b3.Capsule{{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.2}
	planes: [4]b3.BodyPlaneResult
	bodyTransform := IdentityAt(0.0, 0.0, 0.0)

	// Capacity caps the result and prevents writing past the buffer.
	capped := b3.Body_CollideMover(
		bodyId,
		&planes[0],
		1,
		b3.Pos{0.0, 0.0, 0.0},
		&mover,
		b3.DefaultQueryFilter(),
		bodyTransform,
	)
	ENSURE(t, capped == 1) or_return

	full := b3.Body_CollideMover(
		bodyId,
		&planes[0],
		4,
		b3.Pos{0.0, 0.0, 0.0},
		&mover,
		b3.DefaultQueryFilter(),
		bodyTransform,
	)
	ENSURE(t, full == 2) or_return

	b3.DestroyWorld(worldId)
	return true
}

// TimeOfImpactMover ------------------------------------------------------------------------

// The mover capsule is expressed in the query frame, so a core segment starting at the origin
// stands the character on the query point. Targets sit on the sweep line at y = 0.
@(private = "file")
MakeStandingMover :: proc() -> b3.Capsule {
	return b3.Capsule{{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.25}
}

@(private = "file")
MoverTOIHitsBox :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	shapeDef := b3.DefaultShapeDef()
	shapeId := b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	// Face at x = 4.5, mover radius 0.25, so 4.25 of the 10 unit sweep is free.
	mover := MakeStandingMover()
	bodyTransform := IdentityAt(5.0, 0.0, 0.0)
	result := b3.Body_TimeOfImpactMover(
		bodyId,
		b3.Pos_zero,
		&mover,
		b3.Vec3{10.0, 0.0, 0.0},
		b3.DefaultQueryFilter(),
		bodyTransform,
		bodyTransform,
	)

	ENSURE_SMALL(t, result.fraction - 0.425, 1e-2) or_return
	ENSURE(t, b3.IsNormalized(result.normal)) or_return
	ENSURE_SMALL(t, result.normal.x + 1.0, 1e-3) or_return

	// The result carries a shape id, so the hit shape must come back identified.
	ENSURE(t, b3.Shape_IsValid(result.shapeId)) or_return
	ENSURE(t, result.shapeId.index1 == shapeId.index1) or_return
	ENSURE(t, result.shapeId.generation == shapeId.generation) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
MoverTOISeparated :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	// Sweeping along +Y holds the X gap at 4.25 for the whole interval.
	mover := MakeStandingMover()
	bodyTransform := IdentityAt(5.0, 0.0, 0.0)
	result := b3.Body_TimeOfImpactMover(
		bodyId,
		b3.Pos_zero,
		&mover,
		b3.Vec3{0.0, 10.0, 0.0},
		b3.DefaultQueryFilter(),
		bodyTransform,
		bodyTransform,
	)

	ENSURE_SMALL(t, result.fraction - 1.0, 1e-6) or_return
	ENSURE(t, b3.Shape_IsValid(result.shapeId) == false) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
MoverTOIOverlapped :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	// Mover starts buried in the box, so there is no free interval to search.
	mover := MakeStandingMover()
	bodyTransform := IdentityAt(0.0, 0.0, 0.0)
	result := b3.Body_TimeOfImpactMover(
		bodyId,
		b3.Pos_zero,
		&mover,
		b3.Vec3{10.0, 0.0, 0.0},
		b3.DefaultQueryFilter(),
		bodyTransform,
		bodyTransform,
	)

	// Overlap should be ignored.
	ENSURE_SMALL(t, result.fraction - 1.0, 1e-6) or_return
	ENSURE(t, b3.IS_NULL(result.shapeId)) or_return

	b3.DestroyWorld(worldId)
	return true
}

// Two shapes on the sweep line must resolve to the nearer one whatever order the shape list
// hands them to the loop. Shapes are pushed on the head of the list, so the two bodies below
// walk their shapes in opposite orders.
@(private = "file")
MoverTOIClosestShape :: proc(t: ^testing.T) -> bool {
	nearFirstId: b3.BodyId
	worldId := CreateQueryWorld(&nearFirstId)

	bodyDef := b3.DefaultBodyDef()
	nearLastId := b3.CreateBody(worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()
	nearSphere := b3.Sphere{{5.0, 0.0, 0.0}, 0.5}
	farSphere := b3.Sphere{{9.0, 0.0, 0.0}, 0.5}

	nearFirstHit := b3.CreateSphereShape(nearFirstId, &shapeDef, &nearSphere)
	b3.CreateSphereShape(nearFirstId, &shapeDef, &farSphere)

	b3.CreateSphereShape(nearLastId, &shapeDef, &farSphere)
	nearLastHit := b3.CreateSphereShape(nearLastId, &shapeDef, &nearSphere)

	mover := MakeStandingMover()
	bodyTransform := IdentityAt(0.0, 0.0, 0.0)
	translation := b3.Vec3{10.0, 0.0, 0.0}

	nearFirst := b3.Body_TimeOfImpactMover(
		nearFirstId,
		b3.Pos_zero,
		&mover,
		translation,
		b3.DefaultQueryFilter(),
		bodyTransform,
		bodyTransform,
	)
	nearLast := b3.Body_TimeOfImpactMover(
		nearLastId,
		b3.Pos_zero,
		&mover,
		translation,
		b3.DefaultQueryFilter(),
		bodyTransform,
		bodyTransform,
	)

	ENSURE_SMALL(t, nearFirst.fraction - 0.425, 1e-2) or_return
	ENSURE(t, nearFirst.shapeId.index1 == nearFirstHit.index1) or_return

	ENSURE_SMALL(t, nearLast.fraction - nearFirst.fraction, 1e-4) or_return
	ENSURE(t, nearLast.shapeId.index1 == nearLastHit.index1) or_return

	b3.DestroyWorld(worldId)
	return true
}

// A shape the sweep clears must not erase a hit found earlier in the list. The off path sphere
// is created first so the list hands it over last.
@(private = "file")
MoverTOIKeepsHitAfterMiss :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	shapeDef := b3.DefaultShapeDef()
	offPath := b3.Sphere{{5.0, 0.0, 20.0}, 0.5}
	onPath := b3.Sphere{{5.0, 0.0, 0.0}, 0.5}
	b3.CreateSphereShape(bodyId, &shapeDef, &offPath)
	onPathId := b3.CreateSphereShape(bodyId, &shapeDef, &onPath)

	mover := MakeStandingMover()
	bodyTransform := IdentityAt(0.0, 0.0, 0.0)
	result := b3.Body_TimeOfImpactMover(
		bodyId,
		b3.Pos_zero,
		&mover,
		b3.Vec3{10.0, 0.0, 0.0},
		b3.DefaultQueryFilter(),
		bodyTransform,
		bodyTransform,
	)

	ENSURE_SMALL(t, result.fraction - 0.425, 1e-2) or_return
	ENSURE(t, result.shapeId.index1 == onPathId.index1) or_return

	b3.DestroyWorld(worldId)
	return true
}

// The mover capsule points live in the query frame, so sliding both the mover and the body by
// the same offset must not move the impact.
@(private = "file")
MoverTOIMoverOffset :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	translation := b3.Vec3{10.0, 0.0, 0.0}
	filter := b3.DefaultQueryFilter()

	atOrigin := MakeStandingMover()
	transformA := IdentityAt(5.0, 0.0, 0.0)
	resultA := b3.Body_TimeOfImpactMover(bodyId, b3.Pos_zero, &atOrigin, translation, filter, transformA, transformA)

	offset := b3.Capsule{{2.0, 0.0, 0.0}, {2.0, 1.0, 0.0}, 0.25}
	transformB := IdentityAt(7.0, 0.0, 0.0)
	resultB := b3.Body_TimeOfImpactMover(bodyId, b3.Pos_zero, &offset, translation, filter, transformB, transformB)

	ENSURE_SMALL(t, resultB.fraction - resultA.fraction, 1e-4) or_return
	ENSURE_SMALL(t, resultB.point.x - resultA.point.x - 2.0, 1e-3) or_return

	b3.DestroyWorld(worldId)
	return true
}

// The body sweeps between the two supplied transforms. A bar spinning a quarter turn about Y
// reaches a mover that its start pose clears.
@(private = "file")
MoverTOIRotatingBody :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	bar := b3.MakeBoxHull(2.0, 0.25, 0.25)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, &bar.base)

	// Query origin puts the mover at world (0,0,2.1), just inside the swept end of the bar.
	origin := b3.Pos{0.0, 0.0, 2.1}
	mover := MakeStandingMover()
	transform1 := IdentityAt(0.0, 0.0, 0.0)
	transform2 := b3.WorldTransform {
		p = b3.Pos{0.0, 0.0, 0.0},
		q = b3.MakeQuatFromAxisAngle(b3.Vec3{0.0, 1.0, 0.0}, 0.5 * b3.PI),
	}

	spinning := b3.Body_TimeOfImpactMover(
		bodyId,
		origin,
		&mover,
		b3.Vec3_zero,
		b3.DefaultQueryFilter(),
		transform1,
		transform2,
	)

	ENSURE(t, 0.0 < spinning.fraction && spinning.fraction < 1.0) or_return

	// Holding the start pose leaves the bar along X and well clear.
	still := b3.Body_TimeOfImpactMover(
		bodyId,
		origin,
		&mover,
		b3.Vec3_zero,
		b3.DefaultQueryFilter(),
		transform1,
		transform1,
	)

	ENSURE_SMALL(t, still.fraction - 1.0, 1e-6) or_return
	ENSURE(t, b3.IS_NULL(still.shapeId)) or_return

	b3.DestroyWorld(worldId)
	return true
}

// Everything is re-centered on the origin, so a huge origin must not shift the fraction and the
// hit point comes back in the origin frame.
@(private = "file")
MoverTOIFarFromOrigin :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	origin := b3.Pos{1.0e6, -2.0e6, 5.0e5}
	mover := MakeStandingMover()
	bodyTransform := b3.WorldTransform {
		p = b3.OffsetPos(origin, b3.Vec3{5.0, 0.0, 0.0}),
		q = b3.Quat_identity,
	}
	result := b3.Body_TimeOfImpactMover(
		bodyId,
		origin,
		&mover,
		b3.Vec3{10.0, 0.0, 0.0},
		b3.DefaultQueryFilter(),
		bodyTransform,
		bodyTransform,
	)

	ENSURE_SMALL(t, result.fraction - 0.425, 1e-2) or_return
	ENSURE_SMALL(t, result.normal.x + 1.0, 1e-3) or_return
	ENSURE_SMALL(t, (result.point.x - origin.x) - 4.4, 0.5) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
MoverTOIFilter :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	// Geometry is on the sweep line, but a zero mask rejects every category.
	mover := MakeStandingMover()
	filter := b3.DefaultQueryFilter()
	filter.maskBits = 0
	bodyTransform := IdentityAt(5.0, 0.0, 0.0)
	result := b3.Body_TimeOfImpactMover(
		bodyId,
		b3.Pos_zero,
		&mover,
		b3.Vec3{10.0, 0.0, 0.0},
		filter,
		bodyTransform,
		bodyTransform,
	)

	ENSURE_SMALL(t, result.fraction - 1.0, 1e-6) or_return

	b3.DestroyWorld(worldId)
	return true
}

// A shape the mover starts inside is skipped, but the sweep still runs against the other shapes
// on the body.
@(private = "file")
MoverTOIOverlapSkipsShape :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	shapeDef := b3.DefaultShapeDef()

	buried := b3.MakeBoxHull(0.5, 0.5, 0.5)
	b3.CreateHullShape(bodyId, &shapeDef, &buried.base)

	// Face at x = 4.5 like MoverTOIHitsBox
	onPath := b3.MakeOffsetBoxHull(0.5, 0.5, 0.5, b3.Vec3{5.0, 0.0, 0.0})
	onPathId := b3.CreateHullShape(bodyId, &shapeDef, &onPath.base)

	mover := MakeStandingMover()
	bodyTransform := IdentityAt(0.0, 0.0, 0.0)
	result := b3.Body_TimeOfImpactMover(
		bodyId,
		b3.Pos_zero,
		&mover,
		b3.Vec3{10.0, 0.0, 0.0},
		b3.DefaultQueryFilter(),
		bodyTransform,
		bodyTransform,
	)

	ENSURE_SMALL(t, result.fraction - 0.425, 1e-2) or_return
	ENSURE(t, result.shapeId.index1 == onPathId.index1) or_return
	ENSURE_SMALL(t, result.normal.x + 1.0, 1e-3) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
RandomFloat :: proc(seed: ^u32, lower: f32, upper: f32) -> f32 {
	x := seed^
	x ~= x << 13
	x ~= x >> 17
	x ~= x << 5
	seed^ = x
	unit := f32(x >> 8) / 16777216.0
	return lower + (upper - lower) * unit
}

@(private = "file")
RandomVector :: proc(seed: ^u32, lower: f32, upper: f32) -> b3.Vec3 {
	v: b3.Vec3
	v.x = RandomFloat(seed, lower, upper)
	v.y = RandomFloat(seed, lower, upper)
	v.z = RandomFloat(seed, lower, upper)
	return v
}

@(private = "file")
RandomDirection :: proc(seed: ^u32) -> b3.Vec3 {
	for {
		v := RandomVector(seed, -1.0, 1.0)
		length := b3.Length(v)
		if 0.1 < length && length < 1.0 {
			return b3.MulSV(1.0 / length, v)
		}
	}
}

// The mover sweep follows the solver's continuous rule: any time of impact strictly inside the
// sweep counts, whatever the root finder reports. A capsule pivoting around a hull vertex can
// exhaust the iteration cap, and that must not read as a clean miss. Compare against the raw
// sweep built from the same inputs.
@(private = "file")
MoverTOIMatchesSweep :: proc(t: ^testing.T) -> bool {
	bodyId: b3.BodyId
	worldId := CreateQueryWorld(&bodyId)

	bar := b3.MakeBoxHull(2.0, 0.25, 0.25)
	shapeDef := b3.DefaultShapeDef()
	shapeId := b3.CreateHullShape(bodyId, &shapeDef, &bar.base)

	seed: u32 = 0x9E3779B9
	hitCount := 0
	for i := 0; i < 2000; i += 1 {
		mover: b3.Capsule
		mover.center1 = RandomVector(&seed, -0.2, 0.2)
		// unified: C leaves the order of argument evaluation unspecified (so the draws below differ by
		// compiler); Odin evaluates left to right. The test compares the query with a raw sweep built
		// from the same draws, so any order is equally valid.
		mover.center2 = b3.MulAdd(mover.center1, RandomFloat(&seed, 0.05, 1.5), RandomDirection(&seed))
		mover.radius = RandomFloat(&seed, 0.05, 0.5)
		capsulePoints := [2]b3.Vec3{mover.center1, mover.center2}

		o := RandomVector(&seed, -3.5, 3.5)
		origin := b3.Pos{o.x, o.y, o.z}
		translation := b3.MulSV(RandomFloat(&seed, 0.0, 3.0), RandomDirection(&seed))

		transform1 := b3.WorldTransform {
			p = b3.Pos_zero,
			q = b3.MakeQuatFromAxisAngle(RandomDirection(&seed), RandomFloat(&seed, 0.0, b3.PI)),
		}
		spin := b3.MakeQuatFromAxisAngle(RandomDirection(&seed), RandomFloat(&seed, 0.001, b3.PI))
		shift := RandomVector(&seed, -1.0, 1.0)
		transform2 := b3.WorldTransform {
			p = b3.Pos{shift.x, shift.y, shift.z},
			q = b3.MulQuat(spin, transform1.q),
		}

		result := b3.Body_TimeOfImpactMover(
			bodyId,
			origin,
			&mover,
			translation,
			b3.DefaultQueryFilter(),
			transform1,
			transform2,
		)

		// Same sweep the body query builds. The static body keeps its center at the body origin.
		xf1 := b3.ToRelativeTransform(transform1, origin)
		xf2 := b3.ToRelativeTransform(transform2, origin)

		input := b3.TOIInput{}
		input.proxyA = b3.ShapeProxy{b3.GetHullPoints(&bar.base), bar.base.vertexCount, 0.0}
		input.proxyB = b3.ShapeProxy{&capsulePoints[0], 2, mover.radius}
		input.sweepA.c1 = xf1.p
		input.sweepA.c2 = xf2.p
		input.sweepA.q1 = transform1.q
		input.sweepA.q2 = transform2.q
		input.sweepA.localCenter = b3.Vec3_zero
		input.sweepB.c1 = b3.Vec3_zero
		input.sweepB.c2 = translation
		input.sweepB.q1 = b3.Quat_identity
		input.sweepB.q2 = b3.Quat_identity
		input.sweepB.localCenter = b3.Vec3_zero
		input.maxFraction = 1.0

		raw := b3.TimeOfImpact(&input)

		if 0.0 < raw.fraction && raw.fraction < 1.0 {
			ENSURE_SMALL(t, result.fraction - raw.fraction, 1e-6) or_return
			ENSURE(t, result.shapeId.index1 == shapeId.index1) or_return
			ENSURE(t, b3.IsNormalized(result.normal)) or_return
			ENSURE_SMALL(t, b3.Distance(b3.SubPos(result.point, origin), raw.point), 1e-4) or_return
			hitCount += 1
		} else {
			ENSURE_SMALL(t, result.fraction - 1.0, 1e-6) or_return
			ENSURE(t, b3.IS_NULL(result.shapeId)) or_return
		}
	}

	ENSURE(t, hitCount > 0) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(test)
BodyQueryTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, CastRayHitsSphere) {return}
	if !RUN_SUBTEST(t, CastRayMiss) {return}
	if !RUN_SUBTEST(t, CastRayClosestShape) {return}
	if !RUN_SUBTEST(t, CastRayRotatedBody) {return}
	if !RUN_SUBTEST(t, CastRayFarFromOrigin) {return}

	if !RUN_SUBTEST(t, CastShapeHitsBox) {return}
	if !RUN_SUBTEST(t, CastShapeMiss) {return}
	if !RUN_SUBTEST(t, CastShapeRotatedBody) {return}
	if !RUN_SUBTEST(t, CastShapeFarFromOrigin) {return}

	if !RUN_SUBTEST(t, OverlapTrue) {return}
	if !RUN_SUBTEST(t, OverlapFalse) {return}
	if !RUN_SUBTEST(t, OverlapRespectsBodyTransform) {return}
	if !RUN_SUBTEST(t, OverlapFilter) {return}
	if !RUN_SUBTEST(t, OverlapHullProxyEquivalence) {return}
	if !RUN_SUBTEST(t, OverlapHullProxyRotation) {return}
	if !RUN_SUBTEST(t, OverlapCompoundRotatedBody) {return}

	if !RUN_SUBTEST(t, MoverTouchesBox) {return}
	if !RUN_SUBTEST(t, MoverSeparated) {return}
	if !RUN_SUBTEST(t, MoverRotatedBody) {return}
	if !RUN_SUBTEST(t, MoverCapacity) {return}
	if !RUN_SUBTEST(t, MoverTOIHitsBox) {return}
	if !RUN_SUBTEST(t, MoverTOISeparated) {return}
	if !RUN_SUBTEST(t, MoverTOIOverlapped) {return}
	if !RUN_SUBTEST(t, MoverTOIClosestShape) {return}
	if !RUN_SUBTEST(t, MoverTOIKeepsHitAfterMiss) {return}
	if !RUN_SUBTEST(t, MoverTOIMoverOffset) {return}
	if !RUN_SUBTEST(t, MoverTOIRotatingBody) {return}
	if !RUN_SUBTEST(t, MoverTOIFarFromOrigin) {return}
	if !RUN_SUBTEST(t, MoverTOIFilter) {return}
	if !RUN_SUBTEST(t, MoverTOIOverlapSkipsShape) {return}
	if !RUN_SUBTEST(t, MoverTOIMatchesSweep) {return}
}
