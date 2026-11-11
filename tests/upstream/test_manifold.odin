// Ported from box3d test/test_manifold.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "core:math"
import "core:testing"

// A face contact keeps the whole clipped polygon when the manifold has room for it, so a build
// with a larger B3_MAX_MANIFOLD_POINTS legitimately reports more than four points. At the default
// this is still an exact count.
@(private = "file")
ENSURE_FACE_POINTS :: proc(t: ^testing.T, count: i32, loc := #caller_location) -> bool {
	return ENSURE(t, 4 <= (count) && (count) <= b3.MAX_MANIFOLD_POINTS, loc = loc)
}

@(private = "file")
kRoot2 :: f32(1.41421356)
@(private = "file")
kHalfRoot2 :: f32(0.70710678)

@(private = "file")
kAxisX :: b3.Vec3{1.0, 0.0, 0.0}
@(private = "file")
kAxisY :: b3.Vec3{0.0, 1.0, 0.0}
@(private = "file")
kAxisZ :: b3.Vec3{0.0, 0.0, 1.0}

// unified: the SATCache stores the feature as a u8; these compare and seed it with b3SeparatingFeature values.
@(private = "file")
FEATURE :: #force_inline proc(f: b3.SeparatingFeature) -> u8 {
	return u8(f)
}
// unified end

// b3ComputeCosSin is a rational approximation good to about 1e-3. That is coarse enough to
// shift an edge off the position the analytic result expects, so these fixtures need libm.
@(private = "file")
ExactQuat :: proc(axis: b3.Vec3, radians: f32) -> b3.Quat {
	half := 0.5 * radians
	s := math.sin(half)
	return b3.Quat{{s * axis.x, s * axis.y, s * axis.z}, math.cos(half)}
}

@(private = "file")
ExactRotation :: proc(axis: b3.Vec3, radians: f32) -> b3.Transform {
	return b3.Transform{b3.Vec3_zero, ExactQuat(axis, radians)}
}

// Cube A yawed 45 degrees presents an edge along y at x = +h*root2.
// Cube B rolled 45 degrees presents an edge along z at x = -h*root2.
// Sliding B along x makes those two edges the closest features, so the axis of minimum
// penetration is x, the separation is d - 2*h*root2 and the contact point sits at x = d/2.
// Both hulls are far from a face axis here, which keeps the edge query in charge.
@(private = "file")
MakeCrossedEdgeHulls :: proc(hullA: ^b3.BoxHull, hullB: ^b3.BoxHull, halfWidth: f32) {
	hullA^ = b3.MakeTransformedBoxHull(halfWidth, halfWidth, halfWidth, ExactRotation(kAxisY, 0.25 * b3.PI))
	hullB^ = b3.MakeTransformedBoxHull(halfWidth, halfWidth, halfWidth, ExactRotation(kAxisZ, 0.25 * b3.PI))
}

@(private = "file")
SlideX :: proc(x: f32) -> b3.Transform {
	return b3.Transform{{x, 0.0, 0.0}, b3.Quat_identity}
}

@(private = "file")
SlideY :: proc(y: f32) -> b3.Transform {
	return b3.Transform{{0.0, y, 0.0}, b3.Quat_identity}
}

@(private = "file")
MinSeparation :: proc(manifold: ^b3.LocalManifold) -> f32 {
	minSeparation := b3.FLT_MAX
	for i: i32 = 0; i < manifold.pointCount; i += 1 {
		minSeparation = b3.MinFloat(minSeparation, manifold.points[i].separation)
	}

	return minSeparation
}

// The edge pair axis is built by intersecting the two Gauss map arcs. Walk the crossed edges from
// speculative contact into deep overlap and check the axis, the separation and the point.
@(private = "file")
CrossedEdgeTest :: proc(t: ^testing.T) -> bool {
	hullA, hullB: b3.BoxHull
	MakeCrossedEdgeHulls(&hullA, &hullB, 0.5)

	distances := [?]f32{1.42, kRoot2, 1.41, 1.3}

	for i: i32 = 0; i < i32(len(distances)); i += 1 {
		d := distances[i]
		expected := d - kRoot2

		points: [8]b3.LocalManifoldPoint
		manifold := b3.LocalManifold{}
		manifold.points = &points[0]
		cache := b3.SATCache{}
		b3.CollideHulls(&manifold, 8, &hullA.base, &hullB.base, SlideX(d), &cache)

		ENSURE(t, manifold.pointCount == 1) or_return
		ENSURE(t, cache.type == FEATURE(.edgePairAxis)) or_return
		ENSURE_SMALL(t, manifold.normal.x - 1.0, 1e-6) or_return
		ENSURE_SMALL(t, manifold.normal.y, 1e-6) or_return
		ENSURE_SMALL(t, manifold.normal.z, 1e-6) or_return
		ENSURE_SMALL(t, manifold.points[0].separation - expected, 1e-5) or_return
		ENSURE_SMALL(t, manifold.points[0].point.x - 0.5 * d, 1e-5) or_return
		ENSURE_SMALL(t, manifold.points[0].point.y, 1e-5) or_return
		ENSURE_SMALL(t, manifold.points[0].point.z, 1e-5) or_return

		// The forced edge query must agree with what the full solver chose
		manual := b3.LocalManifold{}
		manual.points = &points[0]
		manualCache := b3.SATCache {
			type = FEATURE(.manualEdgePairAxis),
		}
		b3.CollideHulls(&manual, 8, &hullA.base, &hullB.base, SlideX(d), &manualCache)

		ENSURE(t, manual.pointCount == 1) or_return
		ENSURE_SMALL(t, manual.points[0].separation - expected, 1e-5) or_return
	}

	// Beyond the speculative distance the query reports the axis without building a contact.
	// The axis carries its own orientation now, so a sign error here would read as deep overlap.
	{
		points: [8]b3.LocalManifoldPoint
		manifold := b3.LocalManifold{}
		manifold.points = &points[0]
		cache := b3.SATCache{}
		b3.CollideHulls(&manifold, 8, &hullA.base, &hullB.base, SlideX(1.5), &cache)

		ENSURE(t, manifold.pointCount == 0) or_return
		ENSURE(t, cache.type == FEATURE(.edgePairAxis)) or_return
		ENSURE_SMALL(t, cache.separation - (1.5 - kRoot2), 1e-5) or_return
	}

	return true
}

// The parallel edge rejection compares dot products against the edge length, so it is a sine
// threshold and must hold at any size.
@(private = "file")
EdgeAxisScaleTest :: proc(t: ^testing.T) -> bool {
	scales := [?]f32{100.0, 1.0, 0.2}

	for i: i32 = 0; i < i32(len(scales)); i += 1 {
		s := scales[i]
		hullA, hullB: b3.BoxHull
		MakeCrossedEdgeHulls(&hullA, &hullB, 0.5 * s)

		expected: f32 = -0.002
		d := s * kRoot2 + expected

		points: [8]b3.LocalManifoldPoint
		manifold := b3.LocalManifold{}
		manifold.points = &points[0]
		cache := b3.SATCache{}
		b3.CollideHulls(&manifold, 8, &hullA.base, &hullB.base, SlideX(d), &cache)

		// Differencing coordinates of magnitude d costs precision proportional to the scale
		tolerance := 1e-5 * s + 1e-6

		ENSURE(t, manifold.pointCount == 1) or_return
		ENSURE(t, cache.type == FEATURE(.edgePairAxis)) or_return
		ENSURE_SMALL(t, manifold.normal.x - 1.0, 1e-6) or_return
		ENSURE_SMALL(t, manifold.points[0].separation - expected, tolerance) or_return
		ENSURE_SMALL(t, manifold.points[0].point.x - 0.5 * d, tolerance) or_return
		ENSURE_SMALL(t, manifold.points[0].point.y, tolerance) or_return
		ENSURE_SMALL(t, manifold.points[0].point.z, tolerance) or_return
	}

	return true
}

// The cached edge pair rebuilds the axis without a fresh query. An untouched cache proves the
// cached branch answered rather than falling through to the full SAT.
@(private = "file")
EdgeCacheTest :: proc(t: ^testing.T) -> bool {
	hullA, hullB: b3.BoxHull
	MakeCrossedEdgeHulls(&hullA, &hullB, 0.5)

	points: [8]b3.LocalManifoldPoint
	manifold := b3.LocalManifold{}
	manifold.points = &points[0]
	cache := b3.SATCache{}

	b3.CollideHulls(&manifold, 8, &hullA.base, &hullB.base, SlideX(1.41), &cache)
	ENSURE(t, manifold.pointCount == 1) or_return
	ENSURE(t, cache.type == FEATURE(.edgePairAxis)) or_return
	// Cached edges are the even half of each twin pair
	ENSURE(t, (cache.indexA & 1) == 0 && i32(cache.indexA) < hullA.base.edgeCount) or_return
	ENSURE(t, (cache.indexB & 1) == 0 && i32(cache.indexB) < hullB.base.edgeCount) or_return

	seededSeparation := cache.separation
	ENSURE_SMALL(t, seededSeparation - (1.41 - kRoot2), 1e-5) or_return

	// Small motion, the cached features still describe the contact
	b3.CollideHulls(&manifold, 8, &hullA.base, &hullB.base, SlideX(1.4105), &cache)
	ENSURE(t, manifold.pointCount == 1) or_return
	ENSURE(t, cache.separation == seededSeparation) or_return
	ENSURE_SMALL(t, manifold.points[0].separation - (1.4105 - kRoot2), 1e-5) or_return
	ENSURE_SMALL(t, manifold.normal.x - 1.0, 1e-6) or_return

	// Jump past the speculative distance. The cached axis alone must report the separation.
	b3.CollideHulls(&manifold, 8, &hullA.base, &hullB.base, SlideX(1.5), &cache)
	ENSURE(t, manifold.pointCount == 0) or_return
	ENSURE(t, cache.separation == seededSeparation) or_return

	return true
}

// Sliding B along the direction of edge A walks the closest point off the end of the segment.
@(private = "file")
EdgeEndpointTest :: proc(t: ^testing.T) -> bool {
	hullA, hullB: b3.BoxHull
	MakeCrossedEdgeHulls(&hullA, &hullB, 0.5)

	d: f32 = 1.41
	expected := d - kRoot2

	// Just inside the end of edge A
	{
		transform := b3.Transform{{d, 0.49, 0.0}, b3.Quat_identity}
		points: [8]b3.LocalManifoldPoint
		manifold := b3.LocalManifold{}
		manifold.points = &points[0]
		cache := b3.SATCache {
			type = FEATURE(.manualEdgePairAxis),
		}
		b3.CollideHulls(&manifold, 8, &hullA.base, &hullB.base, transform, &cache)

		ENSURE(t, manifold.pointCount == 1) or_return
		ENSURE_SMALL(t, manifold.points[0].separation - expected, 1e-5) or_return
		ENSURE_SMALL(t, manifold.points[0].point.y - 0.49, 1e-5) or_return
	}

	// Off the end. The edge pair no longer describes a contact, so the builder rejects it and
	// clears the cache rather than clamping to a point that is not on the hulls.
	{
		transform := b3.Transform{{d, 0.55, 0.0}, b3.Quat_identity}
		points: [8]b3.LocalManifoldPoint
		manifold := b3.LocalManifold{}
		manifold.points = &points[0]
		cache := b3.SATCache {
			type = FEATURE(.manualEdgePairAxis),
		}
		b3.CollideHulls(&manifold, 8, &hullA.base, &hullB.base, transform, &cache)

		ENSURE(t, manifold.pointCount == 0) or_return
		ENSURE(t, cache.type == FEATURE(.invalidAxis)) or_return

		// The true gap is a vertex to edge distance well past the speculative distance
		freshCache := b3.SATCache{}
		b3.CollideHulls(&manifold, 8, &hullA.base, &hullB.base, transform, &freshCache)
		ENSURE(t, manifold.pointCount == 0) or_return
		ENSURE(t, freshCache.separation > 0.0) or_return
	}

	return true
}

@(private = "file")
kTiltAxes := [?]b3.Vec3 {
	{0.57735027, 0.57735027, 0.57735027},
	{0.70710678, 0.0, 0.70710678},
	{0.26726124, 0.53452248, 0.80178373},
	{-0.48507125, 0.72760688, -0.48507125},
}

// Angles that straddle the 0.005 rejection threshold
@(private = "file")
kTiltAngles := [?]f32{0.0, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 0.004, 0.005, 0.006, 0.01, 0.05}

// A cube corner is root3/2 from the center of rotation
@(private = "file")
kHalfDiagonal :: f32(0.87)

// Cubes stacked face to face and tipped by a hair. A third of the edge pairs are then nearly
// parallel, the angle between them is at the noise floor and the arc intersection carries no
// information. The face contact has to survive that untouched.
@(private = "file")
ParallelEdgeTest :: proc(t: ^testing.T) -> bool {
	hullA := b3.MakeBoxHull(0.5, 0.5, 0.5)
	hullB := b3.MakeBoxHull(0.5, 0.5, 0.5)

	overlap: f32 = 0.01

	for i: i32 = 0; i < i32(len(kTiltAxes)); i += 1 {
		for j: i32 = 0; j < i32(len(kTiltAngles)); j += 1 {
			angle := kTiltAngles[j]
			transform := b3.Transform{{0.0, 1.0 - overlap, 0.0}, ExactQuat(kTiltAxes[i], angle)}

			points: [8]b3.LocalManifoldPoint
			manifold := b3.LocalManifold{}
			manifold.points = &points[0]
			cache := b3.SATCache{}
			b3.CollideHulls(&manifold, 8, &hullA.base, &hullB.base, transform, &cache)

			ENSURE_FACE_POINTS(t, manifold.pointCount) or_return
			ENSURE(t, cache.type == FEATURE(.faceAxisA) || cache.type == FEATURE(.faceAxisB)) or_return
			ENSURE(t, b3.Dot(manifold.normal, kAxisY) > 0.998) or_return

			// The tilt can only lift or sink a face point by the length of the arc it sweeps
			bound := kHalfDiagonal * angle + 1e-5
			for k: i32 = 0; k < manifold.pointCount; k += 1 {
				ENSURE_SMALL(t, manifold.points[k].separation + overlap, bound) or_return
			}
		}
	}

	return true
}

// Same stack, but force the edge query to answer. With the edges exactly parallel no pair forms a
// Minkowski face at all, and once a pair does form its separation can never be positive because
// the hulls overlap.
@(private = "file")
ParallelEdgeManualTest :: proc(t: ^testing.T) -> bool {
	hullA := b3.MakeBoxHull(0.5, 0.5, 0.5)
	hullB := b3.MakeBoxHull(0.5, 0.5, 0.5)

	overlap: f32 = 0.01

	for i: i32 = 0; i < i32(len(kTiltAxes)); i += 1 {
		for j: i32 = 0; j < i32(len(kTiltAngles)); j += 1 {
			angle := kTiltAngles[j]
			transform := b3.Transform{{0.0, 1.0 - overlap, 0.0}, ExactQuat(kTiltAxes[i], angle)}

			points: [8]b3.LocalManifoldPoint
			manifold := b3.LocalManifold{}
			manifold.points = &points[0]
			cache := b3.SATCache {
				type = FEATURE(.manualEdgePairAxis),
			}
			b3.CollideHulls(&manifold, 8, &hullA.base, &hullB.base, transform, &cache)

			if angle == 0.0 {
				// Every pair is parallel so the query finds nothing and leaves the cache alone
				ENSURE(t, manifold.pointCount == 0) or_return
				ENSURE(t, cache.type == FEATURE(.manualEdgePairAxis)) or_return
				continue
			}

			// The closest points can fall off the ends of the segments
			if manifold.pointCount == 0 {
				continue
			}

			ENSURE(t, manifold.pointCount == 1) or_return
			ENSURE(t, b3.Dot(manifold.normal, kAxisY) > 0.99) or_return

			separation := manifold.points[0].separation
			ENSURE(t, separation <= 0.0) or_return
			ENSURE(t, separation >= -overlap - kHalfDiagonal * angle - 1e-4) or_return
		}
	}

	return true
}

@(private = "file")
g_seed: u32 = 12345

@(private = "file")
NextFloat :: proc(lower: f32, upper: f32) -> f32 {
	g_seed = 1664525 * g_seed + 1013904223
	t := f32(g_seed >> 8) / f32(1 << 24)
	return lower + t * (upper - lower)
}

@(private = "file")
NextDirection :: proc() -> b3.Vec3 {
	v := b3.Vec3{NextFloat(-1.0, 1.0), NextFloat(-1.0, 1.0), NextFloat(-1.0, 1.0)}
	return b3.Normalize(v)
}

// Overlapping hulls admit no separating axis, so an edge separation that comes back positive is
// always noise. It shows up as a manifold with no points, which the solver reads as no contact.
@(private = "file")
OverlapNeverEmptyTest :: proc(t: ^testing.T) -> bool {
	hullA := b3.MakeBoxHull(0.5, 0.5, 0.5)
	hullB := b3.MakeBoxHull(0.4, 0.6, 0.5)

	for i: i32 = 0; i < 2000; i += 1 {
		axis := NextDirection()

		// Half the samples are nearly aligned, where the edge cross products are smallest
		angle := (i & 1) != 0 ? NextFloat(-0.01, 0.01) : NextFloat(-b3.PI, b3.PI)

		// Shorter than the smallest half width, so the center of B is inside A
		offset := b3.MulSV(0.4, NextDirection())

		transform := b3.Transform{offset, ExactQuat(axis, angle)}

		points: [8]b3.LocalManifoldPoint
		manifold := b3.LocalManifold{}
		manifold.points = &points[0]
		cache := b3.SATCache{}
		b3.CollideHulls(&manifold, 8, &hullA.base, &hullB.base, transform, &cache)

		ENSURE(t, manifold.pointCount > 0) or_return
		ENSURE_SMALL(t, b3.Length(manifold.normal) - 1.0, 1e-5) or_return

		// Clipping keeps points that are separated, but the deepest one must penetrate
		ENSURE(t, MinSeparation(&manifold) < 0.0) or_return
	}

	return true
}

// A cube pitched 45 degrees rests on an edge along x at y = -h*root2. The two faces meeting there
// have normals (0,-r,r) and (0,-r,-r), so the arc between them spans the whole lower quadrant.
// A triangle edge crossing under it at an angle picks out an interior point of that arc, which is
// where a wrong lerp parameter would show up.
@(private = "file")
TriangleEdgeTest :: proc(t: ^testing.T) -> bool {
	beta := 20.0 * b3.PI / 180.0
	gamma := 30.0 * b3.PI / 180.0

	hull := b3.MakeTransformedBoxHull(0.5, 0.5, 0.5, ExactRotation(kAxisX, 0.25 * b3.PI))

	// Tipping the triangle plane about z keeps its normal off the hull edge, which the Minkowski
	// test needs. Tipping the edge within that plane moves the arc intersection off the midpoint.
	triNormal := b3.Vec3{math.sin(beta), math.cos(beta), 0.0}
	triEdge := b3.Vec3{math.sin(gamma) * math.cos(beta), -math.sin(gamma) * math.sin(beta), math.cos(gamma)}

	// Perpendicular to both edges and pointing out of the hull
	axis := b3.Normalize(b3.Cross(kAxisX, triEdge))
	hullPoint := b3.Vec3{0.0, -kHalfRoot2, 0.0}

	gaps := [?]f32{0.03, 0.01, 0.0, -0.01, -0.1}

	for i: i32 = 0; i < i32(len(gaps)); i += 1 {
		gap := gaps[i]
		trianglePoint := b3.MulAdd(hullPoint, gap, axis)

		v1 := b3.MulAdd(trianglePoint, -1.0, triEdge)
		v2 := b3.MulAdd(trianglePoint, 1.0, triEdge)
		v3 := b3.MulAdd(v1, 1.5, b3.Cross(triNormal, triEdge))

		points: [8]b3.LocalManifoldPoint
		manifold := b3.LocalManifold{}
		manifold.points = &points[0]
		cache := b3.SATCache {
			type = FEATURE(.manualEdgePairAxis),
		}
		b3.CollideTriangleAndHull(&manifold, 8, v1, v2, v3, 0, &hull.base, &cache, true)

		expectedNormal := b3.Neg(axis)
		expectedPoint := b3.MulAdd(hullPoint, 0.5 * gap, axis)

		ENSURE(t, manifold.pointCount == 1) or_return
		ENSURE(t, cache.type == FEATURE(.edgePairAxis)) or_return
		ENSURE_SMALL(t, manifold.normal.x - expectedNormal.x, 1e-5) or_return
		ENSURE_SMALL(t, manifold.normal.y - expectedNormal.y, 1e-5) or_return
		ENSURE_SMALL(t, manifold.normal.z - expectedNormal.z, 1e-5) or_return
		ENSURE_SMALL(t, manifold.points[0].separation - gap, 1e-5) or_return
		ENSURE_SMALL(t, manifold.points[0].point.x - expectedPoint.x, 1e-5) or_return
		ENSURE_SMALL(t, manifold.points[0].point.y - expectedPoint.y, 1e-5) or_return
		ENSURE_SMALL(t, manifold.points[0].point.z - expectedPoint.z, 1e-5) or_return
	}

	// The tipped triangle plane buries a corner of the hull, so neither face axis separates and
	// the edge axis has to carry the speculative cull on its own.
	culled := [?]f32{0.03, 0.05}

	for i: i32 = 0; i < i32(len(culled)); i += 1 {
		gap := culled[i]
		trianglePoint := b3.MulAdd(hullPoint, gap, axis)

		v1 := b3.MulAdd(trianglePoint, -1.0, triEdge)
		v2 := b3.MulAdd(trianglePoint, 1.0, triEdge)
		v3 := b3.MulAdd(v1, 1.5, b3.Cross(triNormal, triEdge))

		points: [8]b3.LocalManifoldPoint
		manifold := b3.LocalManifold{}
		manifold.points = &points[0]
		cache := b3.SATCache{}
		b3.CollideTriangleAndHull(&manifold, 8, v1, v2, v3, 0, &hull.base, &cache, true)

		ENSURE(t, manifold.pointCount == 0) or_return
		ENSURE(t, cache.type == FEATURE(.edgePairAxis)) or_return
		ENSURE_SMALL(t, cache.separation - gap, 1e-5) or_return
	}

	return true
}

// The same cube resting its bottom edge on a triangle whose first edge runs along x. Tipping the
// triangle takes that pair from exactly parallel through the rejection threshold.
@(private = "file")
TriangleParallelEdgeTest :: proc(t: ^testing.T) -> bool {
	hull := b3.MakeTransformedBoxHull(0.5, 0.5, 0.5, ExactRotation(kAxisX, 0.25 * b3.PI))

	overlap: f32 = 0.01
	y := -kHalfRoot2 + overlap

	for i: i32 = 0; i < i32(len(kTiltAxes)); i += 1 {
		for j: i32 = 0; j < i32(len(kTiltAngles)); j += 1 {
			q := ExactQuat(kTiltAxes[i], kTiltAngles[j])

			v1 := b3.RotateVector(q, b3.Vec3{-2.0, y, -1.0})
			v2 := b3.RotateVector(q, b3.Vec3{0.0, y, 2.0})
			v3 := b3.RotateVector(q, b3.Vec3{2.0, y, -1.0})

			points: [8]b3.LocalManifoldPoint
			manifold := b3.LocalManifold{}
			manifold.points = &points[0]
			cache := b3.SATCache{}
			b3.CollideTriangleAndHull(&manifold, 8, v1, v2, v3, 0, &hull.base, &cache, true)

			ENSURE_FACE_POINTS(t, manifold.pointCount) or_return
			ENSURE(t, cache.type == FEATURE(.faceAxisA)) or_return
			ENSURE(t, b3.Dot(manifold.normal, kAxisY) > 0.99) or_return

			// The tilt can only sink the contact by the length of the arc it sweeps
			bound := kHalfDiagonal * kTiltAngles[j] + 1e-5
			ENSURE_SMALL(t, MinSeparation(&manifold) + overlap, bound) or_return
		}
	}

	return true
}

// A crossed ridge pair must land on a four point roof face contact. The clipped face
// separation can be no deeper than root2 times the vertical overlap.
@(private = "file")
CheckRoofFaceContact :: proc(t: ^testing.T, manifold: ^b3.LocalManifold, cache: ^b3.SATCache, overlap: f32) -> bool {
	ENSURE_FACE_POINTS(t, manifold.pointCount) or_return
	ENSURE(t, cache.type == FEATURE(.faceAxisA) || cache.type == FEATURE(.faceAxisB)) or_return

	// A roof face of one hull, so 45 degrees off the vertical
	ENSURE_SMALL(t, manifold.normal.y - kHalfRoot2, 1e-4) or_return

	minSeparation := MinSeparation(manifold)
	ENSURE(t, minSeparation < -kHalfRoot2 * overlap + 1e-4) or_return
	ENSURE(t, minSeparation > -kRoot2 * overlap - 1e-4) or_return

	return true
}

// Two long roof ridges laid across each other. The axis of minimum penetration is the edge
// pair, but a one point edge contact is weak for stacking. The collider builds the roof face
// contact first and only switches to the edge contact when the edge axis beats the clipped
// face separation by more than the slop. This pins all three regimes of that policy.
@(private = "file")
RidgeCrossingTest :: proc(t: ^testing.T) -> bool {
	hullA := b3.MakeTransformedBoxHull(1.5, 0.1, 0.1, ExactRotation(kAxisX, 0.25 * b3.PI))
	hullB := b3.MakeTransformedBoxHull(1.5, 0.1, 0.1, ExactRotation(kAxisX, 0.25 * b3.PI))

	ridgeY := 0.1 * kRoot2

	// Shallow overlap. The edge axis is better by only ( root2 - 1 ) * overlap, inside the
	// slop, so the four point face contact carries the crossing at every angle.
	{
		overlap: f32 = 0.01
		lift := 2.0 * ridgeY - overlap
		crossingAngles := [?]f32{0.0, 1e-3, 0.02, 0.1, 0.5}

		for i: i32 = 0; i < i32(len(crossingAngles)); i += 1 {
			transform := b3.Transform{{0.0, lift, 0.0}, ExactQuat(kAxisY, crossingAngles[i])}

			points: [8]b3.LocalManifoldPoint
			manifold := b3.LocalManifold{}
			manifold.points = &points[0]
			cache := b3.SATCache{}
			b3.CollideHulls(&manifold, 8, &hullA.base, &hullB.base, transform, &cache)

			CheckRoofFaceContact(t, &manifold, &cache, overlap) or_return
		}
	}

	// Deep overlap at a clear crossing. The edge axis now beats the clipped face separation
	// by more than the slop, so the edge contact replaces the face contact.
	{
		overlap: f32 = 0.05
		lift := 2.0 * ridgeY - overlap
		crossingAngles := [?]f32{0.05, 0.1, 0.2, 0.5}

		for i: i32 = 0; i < i32(len(crossingAngles)); i += 1 {
			transform := b3.Transform{{0.0, lift, 0.0}, ExactQuat(kAxisY, crossingAngles[i])}

			points: [8]b3.LocalManifoldPoint
			manifold := b3.LocalManifold{}
			manifold.points = &points[0]
			cache := b3.SATCache{}
			b3.CollideHulls(&manifold, 8, &hullA.base, &hullB.base, transform, &cache)

			ENSURE(t, manifold.pointCount == 1) or_return
			ENSURE(t, cache.type == FEATURE(.edgePairAxis)) or_return
			ENSURE_SMALL(t, manifold.normal.x, 1e-4) or_return
			ENSURE_SMALL(t, manifold.normal.y - 1.0, 1e-4) or_return
			ENSURE_SMALL(t, manifold.normal.z, 1e-4) or_return
			ENSURE_SMALL(t, manifold.points[0].separation + overlap, 1e-4) or_return
			ENSURE_SMALL(t, manifold.points[0].point.y - (ridgeY - 0.5 * overlap), 1e-4) or_return

			// Only has to land near the crossing, not at the end of a three meter beam
			ENSURE_SMALL(t, manifold.points[0].point.x, 0.01) or_return
			ENSURE_SMALL(t, manifold.points[0].point.z, 0.01) or_return
		}
	}

	// Deep overlap near parallel. A one point edge contact off a parallel pair would have a
	// normal built from noise, so the roof faces keep the contact.
	{
		overlap: f32 = 0.05
		lift := 2.0 * ridgeY - overlap
		shallowAngles := [?]f32{0.0, 1e-4, 1e-3, 0.003}

		for i: i32 = 0; i < i32(len(shallowAngles)); i += 1 {
			transform := b3.Transform{{0.0, lift, 0.0}, ExactQuat(kAxisY, shallowAngles[i])}

			points: [8]b3.LocalManifoldPoint
			manifold := b3.LocalManifold{}
			manifold.points = &points[0]
			cache := b3.SATCache{}
			b3.CollideHulls(&manifold, 8, &hullA.base, &hullB.base, transform, &cache)

			CheckRoofFaceContact(t, &manifold, &cache, overlap) or_return
		}
	}

	return true
}

// The edge pair axis produced by the arc intersection must match the classic edge cross product.
// Rebuild the axis, the separation and the contact point from the two contributing edges and the
// convex radius, then compare against the manifold. orientRef fixes the sign of the axis to match
// the manifold normal convention for the shape pair. e1 belongs to the shape whose contact point is
// pulled in by the radius (the hull or triangle when it meets a capsule), e2 to the other edge.
@(private = "file")
CheckEdgeContact :: proc(
	t: ^testing.T,
	manifold: ^b3.LocalManifold,
	p1: b3.Vec3,
	e1: b3.Vec3,
	p2: b3.Vec3,
	e2: b3.Vec3,
	orientRef: b3.Vec3,
	radius: f32,
	normalTol: f32,
	sepTol: f32,
	pointTol: f32,
) -> bool {
	axis := b3.Normalize(b3.Cross(e1, e2))
	if b3.Dot(axis, orientRef) < 0.0 {
		axis = b3.Neg(axis)
	}

	// Normal matches the cross product and is perpendicular to both edges
	ENSURE_SMALL(t, manifold.normal.x - axis.x, normalTol) or_return
	ENSURE_SMALL(t, manifold.normal.y - axis.y, normalTol) or_return
	ENSURE_SMALL(t, manifold.normal.z - axis.z, normalTol) or_return
	ENSURE_SMALL(t, b3.Dot(manifold.normal, b3.Normalize(e1)), normalTol) or_return
	ENSURE_SMALL(t, b3.Dot(manifold.normal, b3.Normalize(e2)), normalTol) or_return

	closest := b3.LineDistance(p1, e1, p2, e2)

	// Signed gap between the edge lines along the axis, less the capsule radius
	expectedSeparation := b3.Dot(axis, b3.Sub(closest.point2, closest.point1)) - radius
	ENSURE_SMALL(t, manifold.points[0].separation - expectedSeparation, sepTol) or_return

	// Midpoint of the closest approach, pulling the first point in by the radius
	expectedPoint := b3.MulSV(0.5, b3.Add(b3.MulSub(closest.point1, radius, axis), closest.point2))
	ENSURE_SMALL(t, manifold.points[0].point.x - expectedPoint.x, pointTol) or_return
	ENSURE_SMALL(t, manifold.points[0].point.y - expectedPoint.y, pointTol) or_return
	ENSURE_SMALL(t, manifold.points[0].point.z - expectedPoint.z, pointTol) or_return

	return true
}

@(private = "file")
HullEdgeSegment :: proc(hull: ^b3.HullData, edgeIndex: i32, transform: b3.Transform, point: ^b3.Vec3, edge: ^b3.Vec3) {
	edges := b3.GetHullEdges(hull)
	points := b3.GetHullPoints(hull)
	e := &edges[edgeIndex]
	tail := b3.TransformPoint(transform, points[e.origin])
	head := b3.TransformPoint(transform, points[edges[e.twin].origin])
	point^ = tail
	edge^ = b3.Sub(head, tail)
}

// Two boxes crossing edge to edge. A holds a vertical edge, B is rolled to present a crossing edge
// and yawed so the arc intersection walks off the midpoint. For every configuration that resolves
// to an edge pair the recovered axis, separation and point must match the cross product oracle to
// tight tolerance. The oracle reads the edges the solver actually latched onto, so the check is
// exact regardless of which pair wins.
@(private = "file")
EdgeAxisOracleTest :: proc(t: ^testing.T) -> bool {
	hullA := b3.MakeTransformedBoxHull(0.5, 0.5, 0.5, ExactRotation(kAxisY, 0.25 * b3.PI))

	rolls := [?]f32{0.18 * b3.PI, 0.25 * b3.PI, 0.32 * b3.PI}
	yaws := [?]f32{-0.35, -0.15, 0.0, 0.15, 0.35}
	distances := [?]f32{1.38, 1.40, kRoot2, 1.44}

	edgeContacts: i32 = 0

	for i: i32 = 0; i < i32(len(rolls)); i += 1 {
		hullB := b3.MakeTransformedBoxHull(0.5, 0.5, 0.5, ExactRotation(kAxisZ, rolls[i]))

		for j: i32 = 0; j < i32(len(yaws)); j += 1 {
			for k: i32 = 0; k < i32(len(distances)); k += 1 {
				transform := b3.Transform{{distances[k], 0.0, 0.0}, ExactQuat(kAxisY, yaws[j])}

				points: [8]b3.LocalManifoldPoint
				manifold := b3.LocalManifold{}
				manifold.points = &points[0]
				cache := b3.SATCache{}
				b3.CollideHulls(&manifold, 8, &hullA.base, &hullB.base, transform, &cache)

				if cache.type != FEATURE(.edgePairAxis) || manifold.pointCount != 1 {
					continue
				}

				p1, e1, p2, e2: b3.Vec3
				HullEdgeSegment(&hullA.base, i32(cache.indexA), b3.Transform_identity, &p1, &e1)
				HullEdgeSegment(&hullB.base, i32(cache.indexB), transform, &p2, &e2)

				centerA := hullA.base.center
				centerB := b3.TransformPoint(transform, hullB.base.center)
				orientRef := b3.Sub(centerB, centerA)

				CheckEdgeContact(t, &manifold, p1, e1, p2, e2, orientRef, 0.0, 2e-4, 2e-4, 2e-3) or_return

				edgeContacts += 1
			}
		}
	}

	// The sweep is only meaningful if it actually drove the edge path
	ENSURE(t, edgeContacts >= 15) or_return

	return true
}

// The same oracle over randomly oriented box pairs. Whenever the solver reports an edge pair the
// recovered axis must be perpendicular to both edges and match the cross product. This casts a wide
// net over the arc that the structured sweep cannot reach.
@(private = "file")
EdgeAxisRandomOracleTest :: proc(t: ^testing.T) -> bool {
	g_seed = 246813579

	edgeContacts: i32 = 0

	for i: i32 = 0; i < 2000; i += 1 {
		angleA := NextFloat(0.2, 0.5) * b3.PI
		angleB := NextFloat(0.2, 0.5) * b3.PI
		hullA := b3.MakeTransformedBoxHull(0.5, 0.5, 0.5, ExactRotation(NextDirection(), angleA))
		hullB := b3.MakeBoxHull(0.5, 0.5, 0.5)

		d := NextFloat(1.2, 1.55)
		transform := b3.Transform{b3.MulSV(d, NextDirection()), ExactQuat(NextDirection(), angleB)}

		points: [8]b3.LocalManifoldPoint
		manifold := b3.LocalManifold{}
		manifold.points = &points[0]
		cache := b3.SATCache{}
		b3.CollideHulls(&manifold, 8, &hullA.base, &hullB.base, transform, &cache)

		if cache.type != FEATURE(.edgePairAxis) || manifold.pointCount != 1 {
			continue
		}

		p1, e1, p2, e2: b3.Vec3
		HullEdgeSegment(&hullA.base, i32(cache.indexA), b3.Transform_identity, &p1, &e1)
		HullEdgeSegment(&hullB.base, i32(cache.indexB), transform, &p2, &e2)

		// Skip crossings near parallel where the closest point solve is ill conditioned. The
		// parallel rejection itself is covered by ParallelEdgeTest.
		sine := b3.Length(b3.Cross(b3.Normalize(e1), b3.Normalize(e2)))
		if sine < 0.1 {
			continue
		}

		orientRef := b3.Sub(b3.TransformPoint(transform, hullB.base.center), hullA.base.center)

		CheckEdgeContact(t, &manifold, p1, e1, p2, e2, orientRef, 0.0, 1e-3, 1e-3, 5e-3) or_return

		edgeContacts += 1
	}

	ENSURE(t, edgeContacts >= 100) or_return

	return true
}

// A thin capsule stabbed through the +x +y edge of a box so the edge pair is the axis of minimum
// penetration. This drives the isolated edge axis (arc versus circle on the Gauss map) that a
// capsule presents. The edge is nearly parallel to a box face normal, exactly where the old center
// based orientation flickered, so the axis, the penetration and the point are all checked.
@(private = "file")
HullCapsuleEdgeDeepTest :: proc(t: ^testing.T) -> bool {
	hull := b3.MakeBoxHull(0.5, 0.5, 0.5)

	// The +x +y edge runs along z between the +x and +y faces
	edgePoint := b3.Vec3{0.5, 0.5, 0.0}
	edgeDir := b3.Vec3{0.0, 0.0, 1.0}
	outward := b3.Normalize(b3.Vec3{1.0, 1.0, 0.0})

	// Penetrate far enough that the core segment clearly overlaps the box so the deep path runs,
	// but keep the radius small enough that the edge stays the axis of minimum penetration.
	depths := [?]f32{0.12, 0.18, 0.25}
	radii := [?]f32{0.05, 0.1, 0.2}
	tilts := [?]f32{0.0, 0.25, -0.25}

	count: i32 = 0

	for i: i32 = 0; i < i32(len(depths)); i += 1 {
		for j: i32 = 0; j < i32(len(radii)); j += 1 {
			for k: i32 = 0; k < i32(len(tilts)); k += 1 {
				capsuleDir := b3.Normalize(b3.Vec3{1.0, -1.0, tilts[k]})
				mid := b3.MulAdd(edgePoint, -depths[i], outward)
				c1 := b3.MulAdd(mid, -0.5, capsuleDir)
				c2 := b3.MulAdd(mid, 0.5, capsuleDir)
				capsule := b3.Capsule{c1, c2, radii[j]}

				points: [8]b3.LocalManifoldPoint
				manifold := b3.LocalManifold{}
				manifold.points = &points[0]
				cache := b3.SimplexCache{}
				b3.CollideHullAndCapsule(&manifold, 8, &hull.base, &capsule, b3.Transform_identity, &cache)

				ENSURE(t, manifold.pointCount == 1) or_return
				ENSURE(t, manifold.points[0].separation < 0.0) or_return

				// Hull edge is e1, capsule axis is e2, normal points out of the hull
				CheckEdgeContact(
					t,
					&manifold,
					edgePoint,
					edgeDir,
					c1,
					b3.Sub(c2, c1),
					outward,
					radii[j],
					1e-4,
					1e-4,
					1e-4,
				) or_return

				count += 1
			}
		}
	}

	ENSURE(t, count == i32(len(depths) * len(radii) * len(tilts))) or_return

	return true
}

// Force the triangle versus hull edge query over a broad sweep of crossing geometries. A cube tipped
// 45 degrees rests an edge along x at y = -h*root2. A triangle edge is laid across it at a range of
// yaws, plane tips and gaps so the arc intersection lands all over the arc. The manual axis hands
// the winning pair to the builder, and the recovered axis must match the cross product of the chosen
// triangle and hull edges and point from the triangle into the hull.
@(private = "file")
TriangleHullEdgeSweepTest :: proc(t: ^testing.T) -> bool {
	hull := b3.MakeTransformedBoxHull(0.5, 0.5, 0.5, ExactRotation(kAxisX, 0.25 * b3.PI))
	hullEdgePoint := b3.Vec3{0.0, -kHalfRoot2, 0.0}

	// Degrees: triangle plane tip about z, and triangle edge yaw
	betas := [?]f32{8.0, 20.0, 32.0}
	gammas := [?]f32{20.0, 35.0, 50.0, 70.0}
	gaps := [?]f32{0.02, 0.0, -0.03, -0.08}

	edgeContacts: i32 = 0

	for a: i32 = 0; a < i32(len(betas)); a += 1 {
		for b: i32 = 0; b < i32(len(gammas)); b += 1 {
			for c: i32 = 0; c < i32(len(gaps)); c += 1 {
				beta := betas[a] * b3.PI / 180.0
				gamma := gammas[b] * b3.PI / 180.0

				// Tip the plane off the hull edge so the Minkowski test holds, then yaw the edge
				triNormal := b3.Vec3{math.sin(beta), math.cos(beta), 0.0}
				triEdge := b3.Vec3 {
					math.sin(gamma) * math.cos(beta),
					-math.sin(gamma) * math.sin(beta),
					math.cos(gamma),
				}

				// Perpendicular to both edges and pointing out of the hull
				axis := b3.Normalize(b3.Cross(kAxisX, triEdge))
				trianglePoint := b3.MulAdd(hullEdgePoint, gaps[c], axis)

				v1 := b3.MulAdd(trianglePoint, -1.0, triEdge)
				v2 := b3.MulAdd(trianglePoint, 1.0, triEdge)
				v3 := b3.MulAdd(v1, 1.5, b3.Cross(triNormal, triEdge))

				triangleVerts := [?]b3.Vec3{v1, v2, v3}
				triangleEdges := [?]b3.Vec3{b3.Sub(v2, v1), b3.Sub(v3, v2), b3.Sub(v1, v3)}
				triangleCenter := b3.MulSV(f32(1.0) / f32(3.0), b3.Add(v1, b3.Add(v2, v3)))

				points: [8]b3.LocalManifoldPoint
				manifold := b3.LocalManifold{}
				manifold.points = &points[0]
				cache := b3.SATCache {
					type = FEATURE(.manualEdgePairAxis),
				}
				b3.CollideTriangleAndHull(&manifold, 8, v1, v2, v3, 0, &hull.base, &cache, true)

				if cache.type != FEATURE(.edgePairAxis) || manifold.pointCount != 1 {
					continue
				}

				p1 := triangleVerts[cache.indexA]
				e1 := triangleEdges[cache.indexA]

				p2, e2: b3.Vec3
				HullEdgeSegment(&hull.base, i32(cache.indexB), b3.Transform_identity, &p2, &e2)

				// Normal points from the triangle into the hull
				orientRef := b3.Sub(hull.base.center, triangleCenter)

				CheckEdgeContact(t, &manifold, p1, e1, p2, e2, orientRef, 0.0, 1e-4, 1e-4, 1e-3) or_return

				edgeContacts += 1
			}
		}
	}

	ENSURE(t, edgeContacts >= 30) or_return

	return true
}

// A capsule laid nearly in a triangle plane and pushed across one edge so the edge pair drives the
// deep contact. This exercises the two sided triangle edge, where the side normal trick chooses
// which half of the arc holds the axis. Validate every edge contact the sweep produces.
@(private = "file")
CapsuleTriangleEdgeDeepTest :: proc(t: ^testing.T) -> bool {
	// Triangle in the y = 0 plane. The v1 v2 edge runs along x at z = 0, the interior lies at z < 0.
	v1 := b3.Vec3{-2.0, 0.0, 0.0}
	v2 := b3.Vec3{2.0, 0.0, 0.0}
	v3 := b3.Vec3{0.0, 0.0, -2.0}
	triangle := [?]b3.Vec3{v1, v2, v3}
	triangleEdges := [?]b3.Vec3{b3.Sub(v2, v1), b3.Sub(v3, v2), b3.Sub(v1, v3)}
	triangleCenter := b3.MulSV(f32(1.0) / f32(3.0), b3.Add(v1, b3.Add(v2, v3)))

	// A nearly in plane core crossing the edge at (0,0,z0) with a small out of plane tilt. The core
	// pierces the triangle just inside the edge so the deep path runs and the tilted edge pair wins.
	z0s := [?]f32{-0.05, -0.03, -0.01}
	tilts := [?]f32{0.2, 0.3, 0.4}
	yaws := [?]f32{0.4, 0.6, 0.8}
	radii := [?]f32{0.05, 0.1}

	edgeContacts: i32 = 0

	for i: i32 = 0; i < i32(len(z0s)); i += 1 {
		for j: i32 = 0; j < i32(len(tilts)); j += 1 {
			for y: i32 = 0; y < i32(len(yaws)); y += 1 {
				for r: i32 = 0; r < i32(len(radii)); r += 1 {
					capsuleDir := b3.Normalize(b3.Vec3{math.sin(yaws[y]), tilts[j], math.cos(yaws[y])})
					mid := b3.Vec3{0.0, 0.0, z0s[i]}
					c1 := b3.MulAdd(mid, -0.6, capsuleDir)
					c2 := b3.MulAdd(mid, 0.6, capsuleDir)
					capsule := b3.Capsule{c1, c2, radii[r]}

					points: [8]b3.LocalManifoldPoint
					manifold := b3.LocalManifold{}
					manifold.points = &points[0]
					cache := b3.SimplexCache{}
					b3.CollideTriangleAndCapsule(&manifold, 8, &triangle[0], &capsule, &cache)

					// Only the edge contacts exercise the new axis. Face contacts are handled elsewhere.
					if manifold.pointCount != 1 ||
					   manifold.feature < .featureEdge1 ||
					   manifold.feature > .featureEdge3 {
						continue
					}

					edgeIndex := i32(manifold.feature) - i32(b3.TriangleFeature.featureEdge1)
					p1 := triangle[edgeIndex]
					e1 := triangleEdges[edgeIndex]

					// Normal points from the triangle toward the capsule
					capsuleEdge := b3.Sub(c2, c1)
					capsuleCenter := b3.Lerp(c1, c2, 0.5)
					orientRef := b3.Sub(capsuleCenter, triangleCenter)

					CheckEdgeContact(
						t,
						&manifold,
						p1,
						e1,
						c1,
						capsuleEdge,
						orientRef,
						radii[r],
						1e-4,
						1e-4,
						2e-4,
					) or_return

					edgeContacts += 1
				}
			}
		}
	}

	// The sweep must actually reach the edge path
	ENSURE(t, edgeContacts >= 15) or_return

	return true
}

// A sphere driven straight through a box face, from separated, across the surface where the collider
// switches from GJK closest points to the SAT face pick, and on into deep overlap. The separation
// must stay the analytic gap the whole way and the normal must not flip. A jump at the seam would
// read as a pop in the solver. The sweep is fine enough to land samples on both sides of the seam.
@(private = "file")
SphereHullSeamTest :: proc(t: ^testing.T) -> bool {
	hull := b3.MakeBoxHull(0.5, 0.5, 0.5)
	radius: f32 = 0.15

	yStart := 0.5 + radius + 0.4 * b3.SPECULATIVE_DISTANCE
	yEnd: f32 = 0.1
	steps: i32 = 400
	dy := (yStart - yEnd) / f32(steps)

	previous: f32 = 0.0
	shallowSamples: i32 = 0
	deepSamples: i32 = 0

	for i: i32 = 0; i <= steps; i += 1 {
		y := yStart - f32(i) * dy
		sphere := b3.Sphere{{0.0, y, 0.0}, radius}

		points: [8]b3.LocalManifoldPoint
		manifold := b3.LocalManifold{}
		manifold.points = &points[0]
		cache := b3.SimplexCache{}
		b3.CollideHullAndSphere(&manifold, 8, &hull.base, &sphere, b3.Transform_identity, &cache)

		ENSURE(t, manifold.pointCount == 1) or_return

		separation := manifold.points[0].separation
		expected := (y - 0.5) - radius

		// Separation is the analytic gap on both sides of the seam
		ENSURE_SMALL(t, separation - expected, 1e-5) or_return

		// Normal holds the face direction with no flip
		ENSURE_SMALL(t, manifold.normal.x, 1e-5) or_return
		ENSURE_SMALL(t, manifold.normal.y - 1.0, 1e-5) or_return
		ENSURE_SMALL(t, manifold.normal.z, 1e-5) or_return

		// No jump across the seam: consecutive separations track the step
		if i > 0 {
			ENSURE_SMALL(t, (previous - separation) - dy, 1e-5) or_return
		}
		previous = separation

		if y > 0.5 {
			shallowSamples += 1
		} else {
			deepSamples += 1
		}
	}

	// The sweep must straddle the surface so both the GJK and the SAT branch run
	ENSURE(t, shallowSamples > 0 && deepSamples > 0) or_return

	return true
}

// The same seam for a capsule laid parallel to the face. Above the surface the shallow path clips two
// points, in overlap the face path builds two, and every point must sit at the analytic gap through
// the transition with the normal fixed on the face.
@(private = "file")
CapsuleHullSeamTest :: proc(t: ^testing.T) -> bool {
	hull := b3.MakeBoxHull(0.5, 0.5, 0.5)
	radius: f32 = 0.15
	halfLength: f32 = 0.3

	yStart := 0.5 + radius + 0.4 * b3.SPECULATIVE_DISTANCE
	yEnd: f32 = 0.1
	steps: i32 = 400
	dy := (yStart - yEnd) / f32(steps)

	previous: f32 = 0.0
	shallowSamples: i32 = 0
	deepSamples: i32 = 0

	for i: i32 = 0; i <= steps; i += 1 {
		y := yStart - f32(i) * dy
		capsule := b3.Capsule{{-halfLength, y, 0.0}, {halfLength, y, 0.0}, radius}

		points: [8]b3.LocalManifoldPoint
		manifold := b3.LocalManifold{}
		manifold.points = &points[0]
		cache := b3.SimplexCache{}
		b3.CollideHullAndCapsule(&manifold, 8, &hull.base, &capsule, b3.Transform_identity, &cache)

		ENSURE(t, manifold.pointCount >= 1) or_return

		expected := (y - 0.5) - radius

		// Every point sits at the analytic gap
		for k: i32 = 0; k < manifold.pointCount; k += 1 {
			ENSURE_SMALL(t, manifold.points[k].separation - expected, 1e-5) or_return
		}

		// Normal holds the face direction with no flip
		ENSURE_SMALL(t, manifold.normal.x, 1e-5) or_return
		ENSURE_SMALL(t, manifold.normal.y - 1.0, 1e-5) or_return
		ENSURE_SMALL(t, manifold.normal.z, 1e-5) or_return

		// No jump across the seam
		minSeparation := MinSeparation(&manifold)
		if i > 0 {
			ENSURE_SMALL(t, (previous - minSeparation) - dy, 1e-5) or_return
		}
		previous = minSeparation

		if y > 0.5 {
			shallowSamples += 1
		} else {
			deepSamples += 1
		}
	}

	ENSURE(t, shallowSamples > 0 && deepSamples > 0) or_return

	return true
}

// A capsule core straddling a triangle face inside the interior. The core pierces the plane so the
// deep path runs, and with the tilt kept small the face stays the axis of minimum penetration, so the
// clip must return two points on the triangle face. This is the branch that had no coverage.
@(private = "file")
CapsuleTriangleFaceDeepTest :: proc(t: ^testing.T) -> bool {
	// Triangle in the y = 0 plane, normal +y, centroid at the origin
	v1 := b3.Vec3{-3.0, 0.0, -2.0}
	v2 := b3.Vec3{0.0, 0.0, 4.0}
	v3 := b3.Vec3{3.0, 0.0, -2.0}
	triangle := [?]b3.Vec3{v1, v2, v3}

	yaws := [?]f32{0.0, 0.6, 1.2, 1.8, 2.4}
	tilts := [?]f32{0.06, 0.1, 0.15}
	radii := [?]f32{0.05, 0.1, 0.2}

	// Bias the center just above the plane so the back side cull passes while the lower endpoint dips through
	bias: f32 = 0.01
	halfLength: f32 = 1.0

	faceContacts: i32 = 0

	for i: i32 = 0; i < i32(len(yaws)); i += 1 {
		for j: i32 = 0; j < i32(len(tilts)); j += 1 {
			for r: i32 = 0; r < i32(len(radii)); r += 1 {
				// Axis is the in-plane heading tipped up so the segment straddles the plane
				axis := b3.Vec3 {
					math.cos(tilts[j]) * math.cos(yaws[i]),
					math.sin(tilts[j]),
					math.cos(tilts[j]) * math.sin(yaws[i]),
				}
				center := b3.Vec3{0.0, bias, 0.0}
				c1 := b3.MulAdd(center, -halfLength, axis)
				c2 := b3.MulAdd(center, halfLength, axis)
				capsule := b3.Capsule{c1, c2, radii[r]}

				points: [8]b3.LocalManifoldPoint
				manifold := b3.LocalManifold{}
				manifold.points = &points[0]
				cache := b3.SimplexCache{}
				b3.CollideTriangleAndCapsule(&manifold, 8, &triangle[0], &capsule, &cache)

				// Two points on the triangle face with the plane normal
				ENSURE(t, manifold.pointCount == 2) or_return
				ENSURE(t, manifold.feature == .featureTriangleFace) or_return
				ENSURE_SMALL(t, manifold.normal.x, 1e-5) or_return
				ENSURE_SMALL(t, manifold.normal.y - 1.0, 1e-5) or_return
				ENSURE_SMALL(t, manifold.normal.z, 1e-5) or_return

				// Separations are the endpoint heights pulled in by the radius. The lower endpoint is below
				// the plane, so the deepest separation is negative.
				lower := b3.MinFloat(c1.y, c2.y) - radii[r]
				upper := b3.MaxFloat(c1.y, c2.y) - radii[r]
				minSep := b3.MinFloat(manifold.points[0].separation, manifold.points[1].separation)
				maxSep := b3.MaxFloat(manifold.points[0].separation, manifold.points[1].separation)
				ENSURE_SMALL(t, minSep - lower, 1e-5) or_return
				ENSURE_SMALL(t, maxSep - upper, 1e-5) or_return
				ENSURE(t, minSep < 0.0) or_return

				faceContacts += 1
			}
		}
	}

	// Every configuration must reach the face path
	ENSURE(t, faceContacts == i32(len(yaws) * len(tilts) * len(radii))) or_return

	return true
}

// A capsule laid flat with its core below a box top face. The core sits inside the box so the deep
// path runs, and across a sweep of depths, headings and radii the face clip must return two points.
@(private = "file")
HullCapsuleFaceDeepTest :: proc(t: ^testing.T) -> bool {
	hull := b3.MakeBoxHull(0.5, 0.5, 0.5)

	depths := [?]f32{0.1, 0.2, 0.3, 0.4, 0.45}
	yaws := [?]f32{0.0, 0.4, 0.8, 1.2}
	radii := [?]f32{0.1, 0.15, 0.2}
	offsets := [?]f32{-0.1, 0.0, 0.1}
	halfLength: f32 = 0.3

	faceContacts: i32 = 0

	for i: i32 = 0; i < i32(len(depths)); i += 1 {
		for j: i32 = 0; j < i32(len(yaws)); j += 1 {
			for r: i32 = 0; r < i32(len(radii)); r += 1 {
				for o: i32 = 0; o < i32(len(offsets)); o += 1 {
					y := depths[i]
					dir := b3.Vec3{math.cos(yaws[j]), 0.0, math.sin(yaws[j])}
					center := b3.Vec3{offsets[o], y, 0.0}
					c1 := b3.MulAdd(center, -halfLength, dir)
					c2 := b3.MulAdd(center, halfLength, dir)
					capsule := b3.Capsule{c1, c2, radii[r]}

					points: [8]b3.LocalManifoldPoint
					manifold := b3.LocalManifold{}
					manifold.points = &points[0]
					cache := b3.SimplexCache{}
					b3.CollideHullAndCapsule(&manifold, 8, &hull.base, &capsule, b3.Transform_identity, &cache)

					// Two points on the top face. The hull path does not tag a feature, so the face is
					// identified by the normal and the point count.
					ENSURE(t, manifold.pointCount == 2) or_return
					ENSURE_SMALL(t, manifold.normal.x, 1e-5) or_return
					ENSURE_SMALL(t, manifold.normal.y - 1.0, 1e-5) or_return
					ENSURE_SMALL(t, manifold.normal.z, 1e-5) or_return

					// Flat capsule, so both points sit at the same analytic gap
					expected := (y - 0.5) - radii[r]
					ENSURE_SMALL(t, manifold.points[0].separation - expected, 1e-5) or_return
					ENSURE_SMALL(t, manifold.points[1].separation - expected, 1e-5) or_return
					ENSURE(t, expected < 0.0) or_return

					faceContacts += 1
				}
			}
		}
	}

	ENSURE(t, faceContacts == i32(len(depths) * len(yaws) * len(radii) * len(offsets))) or_return

	return true
}

@(private = "file")
CheckSpeculativePoints :: proc(t: ^testing.T, manifold: ^b3.LocalManifold, gap: f32) -> bool {
	ENSURE(t, manifold.pointCount > 0) or_return
	ENSURE_SMALL(t, MinSeparation(manifold) - gap, 1e-4) or_return
	return true
}

// Contact recycling treats a pair without points as separated by at least the speculative distance.
// Every manifold routine must report a pair inside that distance or recycling can skip real overlap.
// Discussion #175.
@(private = "file")
SpeculativeMarginTest :: proc(t: ^testing.T) -> bool {
	gaps := [?]f32{0.0, 0.25 * b3.SPECULATIVE_DISTANCE, 0.75 * b3.SPECULATIVE_DISTANCE}
	gapCount := i32(len(gaps))

	sphere := b3.Sphere{b3.Vec3_zero, 0.25}
	bigSphere := b3.Sphere{b3.Vec3_zero, 0.5}
	capsuleX := b3.Capsule{{-0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, 0.25}
	capsuleZ := b3.Capsule{{0.0, 0.0, -0.5}, {0.0, 0.0, 0.5}, 0.25}
	box := b3.MakeBoxHull(0.5, 0.5, 0.5)

	// Normal is +y
	triangle := [3]b3.Vec3{{-2.0, 0.0, -2.0}, {0.0, 0.0, 2.0}, {2.0, 0.0, -2.0}}

	points: [b3.MAX_MANIFOLD_POINTS]b3.LocalManifoldPoint
	manifold := b3.LocalManifold{}
	manifold.points = &points[0]

	for i: i32 = 0; i < gapCount; i += 1 {
		gap := gaps[i]

		manifold.pointCount = 0
		b3.CollideSpheres(&manifold, b3.MAX_MANIFOLD_POINTS, &bigSphere, &sphere, SlideY(0.75 + gap))
		ENSURE(t, CheckSpeculativePoints(t, &manifold, gap)) or_return

		manifold.pointCount = 0
		b3.CollideCapsuleAndSphere(&manifold, b3.MAX_MANIFOLD_POINTS, &capsuleX, &sphere, SlideY(0.5 + gap))
		ENSURE(t, CheckSpeculativePoints(t, &manifold, gap)) or_return

		manifold.pointCount = 0
		b3.CollideCapsules(&manifold, b3.MAX_MANIFOLD_POINTS, &capsuleX, &capsuleZ, SlideY(0.5 + gap))
		ENSURE(t, CheckSpeculativePoints(t, &manifold, gap)) or_return

		{
			manifold.pointCount = 0
			cache := b3.SimplexCache{}
			b3.CollideHullAndSphere(&manifold, b3.MAX_MANIFOLD_POINTS, &box.base, &sphere, SlideY(0.75 + gap), &cache)
			ENSURE(t, CheckSpeculativePoints(t, &manifold, gap)) or_return
		}

		{
			manifold.pointCount = 0
			cache := b3.SimplexCache{}
			b3.CollideHullAndCapsule(
				&manifold,
				b3.MAX_MANIFOLD_POINTS,
				&box.base,
				&capsuleX,
				SlideY(0.75 + gap),
				&cache,
			)
			ENSURE(t, CheckSpeculativePoints(t, &manifold, gap)) or_return
		}

		{
			manifold.pointCount = 0
			cache := b3.SATCache{}
			b3.CollideHulls(&manifold, b3.MAX_MANIFOLD_POINTS, &box.base, &box.base, SlideY(1.0 + gap), &cache)
			ENSURE(t, CheckSpeculativePoints(t, &manifold, gap)) or_return
		}

		{
			manifold.pointCount = 0
			raised := b3.Sphere{{0.0, 0.25 + gap, 0.0}, 0.25}
			b3.CollideTriangleAndSphere(&manifold, b3.MAX_MANIFOLD_POINTS, &triangle[0], &raised)
			ENSURE(t, CheckSpeculativePoints(t, &manifold, gap)) or_return
		}

		{
			manifold.pointCount = 0
			raised := capsuleX
			raised.center1.y = 0.25 + gap
			raised.center2.y = 0.25 + gap
			cache := b3.SimplexCache{}
			b3.CollideTriangleAndCapsule(&manifold, b3.MAX_MANIFOLD_POINTS, &triangle[0], &raised, &cache)
			ENSURE(t, CheckSpeculativePoints(t, &manifold, gap)) or_return
		}

		{
			manifold.pointCount = 0
			raised := b3.MakeTransformedBoxHull(0.5, 0.5, 0.5, SlideY(0.5 + gap))
			cache := b3.SATCache{}
			b3.CollideTriangleAndHull(
				&manifold,
				b3.MAX_MANIFOLD_POINTS,
				triangle[0],
				triangle[1],
				triangle[2],
				0,
				&raised.base,
				&cache,
				true,
			)
			ENSURE(t, CheckSpeculativePoints(t, &manifold, gap)) or_return
		}
	}

	// Beyond the speculative distance the closed form routines report nothing
	farGap := 1.5 * b3.SPECULATIVE_DISTANCE

	manifold.pointCount = 0
	b3.CollideSpheres(&manifold, b3.MAX_MANIFOLD_POINTS, &bigSphere, &sphere, SlideY(0.75 + farGap))
	ENSURE(t, manifold.pointCount == 0) or_return

	manifold.pointCount = 0
	b3.CollideCapsuleAndSphere(&manifold, b3.MAX_MANIFOLD_POINTS, &capsuleX, &sphere, SlideY(0.5 + farGap))
	ENSURE(t, manifold.pointCount == 0) or_return

	return true
}

// The library reports the point count it was built with. A mismatch against the manifold
// this translation unit sees means the app and the library disagree on b3Manifold.
@(private = "file")
MaxPointCountTest :: proc(t: ^testing.T) -> bool {
	manifold: b3.Manifold
	ENSURE(t, b3.GetMaxManifoldPoints() == i32(len(manifold.points))) or_return

	return true
}

@(test)
ManifoldTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, MaxPointCountTest) {return}
	if !RUN_SUBTEST(t, CrossedEdgeTest) {return}
	if !RUN_SUBTEST(t, EdgeAxisScaleTest) {return}
	if !RUN_SUBTEST(t, EdgeCacheTest) {return}
	if !RUN_SUBTEST(t, EdgeEndpointTest) {return}
	if !RUN_SUBTEST(t, ParallelEdgeTest) {return}
	if !RUN_SUBTEST(t, ParallelEdgeManualTest) {return}
	if !RUN_SUBTEST(t, OverlapNeverEmptyTest) {return}
	if !RUN_SUBTEST(t, RidgeCrossingTest) {return}
	if !RUN_SUBTEST(t, TriangleEdgeTest) {return}
	if !RUN_SUBTEST(t, TriangleParallelEdgeTest) {return}
	if !RUN_SUBTEST(t, EdgeAxisOracleTest) {return}
	if !RUN_SUBTEST(t, EdgeAxisRandomOracleTest) {return}
	if !RUN_SUBTEST(t, HullCapsuleEdgeDeepTest) {return}
	if !RUN_SUBTEST(t, TriangleHullEdgeSweepTest) {return}
	if !RUN_SUBTEST(t, CapsuleTriangleEdgeDeepTest) {return}
	if !RUN_SUBTEST(t, CapsuleTriangleFaceDeepTest) {return}
	if !RUN_SUBTEST(t, HullCapsuleFaceDeepTest) {return}
	if !RUN_SUBTEST(t, SphereHullSeamTest) {return}
	if !RUN_SUBTEST(t, CapsuleHullSeamTest) {return}
	if !RUN_SUBTEST(t, SpeculativeMarginTest) {return}
}
