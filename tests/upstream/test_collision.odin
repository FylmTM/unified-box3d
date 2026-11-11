// Ported from box3d test/test_collision.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "core:testing"

@(private = "file")
AABBTest :: proc(t: ^testing.T) -> bool {
	a: b3.AABB
	a.lowerBound = b3.Vec3{-1.0, -1.0, -1.0}
	a.upperBound = b3.Vec3{-2.0, -2.0, -2.0}

	ENSURE(t, b3.IsValidAABB(a) == false) or_return

	a.upperBound = b3.Vec3{1.0, 1.0, 0.0}
	ENSURE(t, b3.IsValidAABB(a) == true) or_return

	b := b3.AABB{{2.0, 2.0, 0.0}, {4.0, 4.0, 0.0}}
	ENSURE(t, b3.AABB_Overlaps(a, b) == false) or_return
	ENSURE(t, b3.AABB_Contains(a, b) == false) or_return

	return true
}

@(private = "file")
TestRayAABBIntersection :: proc(t: ^testing.T) -> bool {
	// Test 1: Ray passing through center of AABB
	{
		a := b3.AABB{{-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}}
		p1 := b3.Vec3{-2.0, 0.0, 0.0}
		p2 := b3.Vec3{2.0, 0.0, 0.0}
		minFraction, maxFraction: f32

		hit := b3.RayCastAABB(a, p1, p2, &minFraction, &maxFraction)

		ENSURE(t, hit == true) or_return
		ENSURE(t, b3.AbsFloat(minFraction - 0.25) < 0.001) or_return // Enters at 25% of ray
		ENSURE(t, b3.AbsFloat(maxFraction - 0.75) < 0.001) or_return // Exits at 75% of ray
	}

	// Test 2: Ray starting inside AABB
	{
		a := b3.AABB{{-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}}
		p1 := b3.Vec3{0.0, 0.0, 0.0}
		p2 := b3.Vec3{2.0, 0.0, 0.0}
		minFraction, maxFraction: f32

		hit := b3.RayCastAABB(a, p1, p2, &minFraction, &maxFraction)

		ENSURE(t, hit == true) or_return
		ENSURE(t, minFraction == 0.0) or_return // Starts inside
		ENSURE(t, b3.AbsFloat(maxFraction - 0.5) < 0.001) or_return // Exits at 50% of ray
	}

	// Test 3: Ray ending inside AABB
	{
		a := b3.AABB{{-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}}
		p1 := b3.Vec3{-2.0, 0.0, 0.0}
		p2 := b3.Vec3{0.0, 0.0, 0.0}
		minFraction, maxFraction: f32

		hit := b3.RayCastAABB(a, p1, p2, &minFraction, &maxFraction)

		ENSURE(t, hit == true) or_return
		ENSURE(t, b3.AbsFloat(minFraction - 0.5) < 0.001) or_return // Enters at 50% of ray
		ENSURE(t, maxFraction == 1.0) or_return // Ends inside
	}

	// Test 4: Ray completely inside AABB
	{
		a := b3.AABB{{-2.0, -2.0, -2.0}, {2.0, 2.0, 2.0}}
		p1 := b3.Vec3{-1.0, 0.0, 0.0}
		p2 := b3.Vec3{1.0, 0.0, 0.0}
		minFraction, maxFraction: f32

		hit := b3.RayCastAABB(a, p1, p2, &minFraction, &maxFraction)

		ENSURE(t, hit == true) or_return
		ENSURE(t, minFraction == 0.0) or_return
		ENSURE(t, maxFraction == 1.0) or_return
	}

	// Test 5: Ray missing AABB
	{
		a := b3.AABB{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}}
		p1 := b3.Vec3{-1.0, 2.0, 0.5}
		p2 := b3.Vec3{2.0, 2.0, 0.5}
		minFraction, maxFraction: f32

		hit := b3.RayCastAABB(a, p1, p2, &minFraction, &maxFraction)

		ENSURE(t, hit == false) or_return
	}

	// Test 6: Ray parallel to AABB face (no intersection)
	{
		a := b3.AABB{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}}
		p1 := b3.Vec3{-1.0, 2.0, 0.5}
		p2 := b3.Vec3{2.0, 2.0, 0.5}
		minFraction, maxFraction: f32

		hit := b3.RayCastAABB(a, p1, p2, &minFraction, &maxFraction)

		ENSURE(t, hit == false) or_return
	}

	// Test 7: Ray parallel to AABB face (within bounds)
	{
		a := b3.AABB{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}}
		p1 := b3.Vec3{-1.0, 0.5, 0.5}
		p2 := b3.Vec3{2.0, 0.5, 0.5}
		minFraction, maxFraction: f32

		hit := b3.RayCastAABB(a, p1, p2, &minFraction, &maxFraction)

		ENSURE(t, hit == true) or_return
		ENSURE(t, b3.AbsFloat(minFraction - f32(1.0) / f32(3.0)) < 0.001) or_return
		ENSURE(t, b3.AbsFloat(maxFraction - f32(2.0) / f32(3.0)) < 0.001) or_return
	}

	// Test 8: Degenerate ray (point) inside AABB
	{
		a := b3.AABB{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}}
		p1 := b3.Vec3{0.5, 0.5, 0.5}
		p2 := b3.Vec3{0.5, 0.5, 0.5}
		minFraction, maxFraction: f32

		hit := b3.RayCastAABB(a, p1, p2, &minFraction, &maxFraction)

		ENSURE(t, hit == true) or_return
		ENSURE(t, minFraction == 0.0) or_return
		ENSURE(t, maxFraction == 0.0) or_return
	}

	// Test 9: Degenerate ray (point) outside AABB
	{
		a := b3.AABB{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}}
		p1 := b3.Vec3{2.0, 2.0, 2.0}
		p2 := b3.Vec3{2.0, 2.0, 2.0}
		minFraction, maxFraction: f32

		hit := b3.RayCastAABB(a, p1, p2, &minFraction, &maxFraction)

		ENSURE(t, hit == false) or_return
	}

	// Test 10: Ray pointing away from AABB
	{
		a := b3.AABB{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}}
		p1 := b3.Vec3{-1.0, 0.5, 0.5}
		p2 := b3.Vec3{-2.0, 0.5, 0.5}
		minFraction, maxFraction: f32

		hit := b3.RayCastAABB(a, p1, p2, &minFraction, &maxFraction)

		ENSURE(t, hit == false) or_return
	}

	// Test 11: Ray hitting corner of AABB
	{
		a := b3.AABB{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}}
		p1 := b3.Vec3{-1.0, -1.0, -1.0}
		p2 := b3.Vec3{2.0, 2.0, 2.0}
		minFraction, maxFraction: f32

		hit := b3.RayCastAABB(a, p1, p2, &minFraction, &maxFraction)

		ENSURE(t, hit == true) or_return
		ENSURE(t, b3.AbsFloat(minFraction - f32(1.0) / f32(3.0)) < 0.001) or_return
		ENSURE(t, b3.AbsFloat(maxFraction - f32(2.0) / f32(3.0)) < 0.001) or_return
	}

	// Test 12: Ray grazing edge of AABB
	{
		a := b3.AABB{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}}
		p1 := b3.Vec3{-1.0, 0.0, 0.5}
		p2 := b3.Vec3{2.0, 0.0, 0.5}
		minFraction, maxFraction: f32

		hit := b3.RayCastAABB(a, p1, p2, &minFraction, &maxFraction)

		ENSURE(t, hit == true) or_return
		ENSURE(t, b3.AbsFloat(minFraction - f32(1.0) / f32(3.0)) < 0.001) or_return
		ENSURE(t, b3.AbsFloat(maxFraction - f32(2.0) / f32(3.0)) < 0.001) or_return
	}

	return true
}

// The narrow phase differences the two world positions in double then works in frame A, so a
// manifold far from the origin must match the same manifold at the origin. Float loses this past
// ~1e7 m where the ULP grows larger than the overlap, which is the whole point of large world mode.
@(private = "file")
LargeWorldManifoldTest :: proc(t: ^testing.T) -> bool {
	boxA := b3.MakeBoxHull(0.5, 0.5, 0.5)
	boxB := b3.MakeBoxHull(0.5, 0.5, 0.5)

	// Centers 0.9 apart so the cubes overlap by 0.1 along x
	sep := b3.Vec3{0.9, 0.0, 0.0}

	pointsOrigin: [8]b3.LocalManifoldPoint
	mOrigin := b3.LocalManifold{}
	mOrigin.points = &pointsOrigin[0]

	xfAo := b3.WorldTransform_identity
	xfBo := b3.WorldTransform{b3.OffsetPos(b3.Pos_zero, sep), b3.Quat_identity}
	cacheOrigin := b3.SATCache{}
	b3.CollideHulls(&mOrigin, 8, &boxA.base, &boxB.base, b3.InvMulWorldTransforms(xfAo, xfBo), &cacheOrigin)

	// Two cube faces overlap, so the clipped manifold has four points
	ENSURE(t, mOrigin.pointCount == 4) or_return
	for i: i32 = 0; i < mOrigin.pointCount; i += 1 {
		ENSURE_SMALL(t, mOrigin.points[i].separation + 0.1, 0.01) or_return
	}

	// unified: not ported: the BOX3D_DOUBLE_PRECISION block (the port is single precision only).

	return true
}

// Broad-phase AABBs are built in double and narrowed to float with directed outward rounding, so a
// shape and its speculative margin stay inside their box far from the origin. A float build would
// round the extent away into the ULP (~1 m at 1e7) and clip the shape out of its own box.
@(private = "file")
LargeWorldAABBTest :: proc(t: ^testing.T) -> bool {
	// Unit cube, so the tight extent is 0.5 each way
	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	shape := b3.Shape{}
	shape.type = .hullShape
	shape.hull = &box.base

	aabbOrigin := b3.ComputeFatShapeAABB(&shape, b3.WorldTransform_identity, 0.0)
	ENSURE_SMALL(t, aabbOrigin.lowerBound.x + 0.5, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, aabbOrigin.lowerBound.y + 0.5, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, aabbOrigin.lowerBound.z + 0.5, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, aabbOrigin.upperBound.x - 0.5, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, aabbOrigin.upperBound.y - 0.5, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, aabbOrigin.upperBound.z - 0.5, b3.FLT_EPSILON) or_return

	// unified: not ported: the BOX3D_DOUBLE_PRECISION block (the port is single precision only).

	return true
}

@(test)
CollisionTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, AABBTest) {return}
	if !RUN_SUBTEST(t, TestRayAABBIntersection) {return}
	if !RUN_SUBTEST(t, LargeWorldManifoldTest) {return}
	if !RUN_SUBTEST(t, LargeWorldAABBTest) {return}
}
