// Ported from box3d test/test_sat.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "core:math"
import "core:testing"

@(private = "file")
kRoot2 :: f32(1.41421356)
@(private = "file")
kAxisY :: b3.Vec3{0.0, 1.0, 0.0}
@(private = "file")
kAxisZ :: b3.Vec3{0.0, 0.0, 1.0}

// b3ComputeCosSin is a coarse rational approximation, so the fixtures build their rotations from
// libm to keep the analytic edge and face positions exact.
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

// Directed separation along a unit axis n pointing from A to B. Positive means B clears A along n.
// This is the projection gap min over B minus max over A, which is exactly what a valid separating
// axis measures, so the winning face or edge axis must reproduce it.
@(private = "file")
SepAlong :: proc(hullA: ^b3.HullData, hullB: ^b3.HullData, xfB: b3.Transform, n: b3.Vec3) -> f32 {
	pointsA := b3.GetHullPoints(hullA)
	maxA := -b3.FLT_MAX
	for i: i32 = 0; i < hullA.vertexCount; i += 1 {
		maxA = b3.MaxFloat(maxA, b3.Dot(n, pointsA[i]))
	}

	pointsB := b3.GetHullPoints(hullB)
	minB := b3.FLT_MAX
	for i: i32 = 0; i < hullB.vertexCount; i += 1 {
		minB = b3.MinFloat(minB, b3.Dot(n, b3.TransformPoint(xfB, pointsB[i])))
	}

	return minB - maxA
}

@(private = "file")
TryAxis :: proc(
	hullA: ^b3.HullData,
	hullB: ^b3.HullData,
	xfB: b3.Transform,
	n: b3.Vec3,
	best: ^f32,
	bestNormal: ^b3.Vec3,
) {
	s := SepAlong(hullA, hullB, xfB, n)
	if s > best^ {
		best^ = s
		bestNormal^ = n
	}
}

// Brute force SAT. The maximum separation over every face normal and every edge cross product is
// the true answer the SIMD query has to match. Each undirected axis is tried both ways so the
// winner comes out oriented from A to B with no sign bookkeeping.
@(private = "file")
OracleSeparation :: proc(hullA: ^b3.HullData, hullB: ^b3.HullData, xfB: b3.Transform, outNormal: ^b3.Vec3) -> f32 {
	best := -b3.FLT_MAX
	bestNormal := b3.Vec3{0.0, 0.0, 0.0}

	planesA := b3.GetHullPlanes(hullA)
	for i: i32 = 0; i < hullA.faceCount; i += 1 {
		TryAxis(hullA, hullB, xfB, planesA[i].normal, &best, &bestNormal)
		TryAxis(hullA, hullB, xfB, b3.Neg(planesA[i].normal), &best, &bestNormal)
	}

	planesB := b3.GetHullPlanes(hullB)
	for i: i32 = 0; i < hullB.faceCount; i += 1 {
		n := b3.RotateVector(xfB.q, planesB[i].normal)
		TryAxis(hullA, hullB, xfB, n, &best, &bestNormal)
		TryAxis(hullA, hullB, xfB, b3.Neg(n), &best, &bestNormal)
	}

	edgesA := b3.GetHullEdges(hullA)
	pointsA := b3.GetHullPoints(hullA)
	edgesB := b3.GetHullEdges(hullB)
	pointsB := b3.GetHullPoints(hullB)

	// Half edges are stored as adjacent twin pairs, so the even index and its successor bound one edge.
	for i: i32 = 0; i < hullA.edgeCount; i += 2 {
		dirA := b3.Sub(pointsA[edgesA[i + 1].origin], pointsA[edgesA[i].origin])
		for j: i32 = 0; j < hullB.edgeCount; j += 2 {
			dirB := b3.RotateVector(xfB.q, b3.Sub(pointsB[edgesB[j + 1].origin], pointsB[edgesB[j].origin]))
			cross := b3.Cross(dirA, dirB)

			// Near parallel edges give no useful axis and cannot be the true maximum.
			if b3.Dot(cross, cross) < 1e-10 {
				continue
			}

			cross = b3.Normalize(cross)
			TryAxis(hullA, hullB, xfB, cross, &best, &bestNormal)
			TryAxis(hullA, hullB, xfB, b3.Neg(cross), &best, &bestNormal)
		}
	}

	outNormal^ = bestNormal
	return best
}

// Two axis aligned cubes pushed apart along x. A's plus x face is the separating axis, the gap is
// large enough to trip the speculative early out, and the reference vertex on B is its minus x side.
@(private = "file")
FaceAxisASeparatedTest :: proc(t: ^testing.T) -> bool {
	hullA := b3.MakeBoxHull(0.5, 0.5, 0.5)
	hullB := b3.MakeBoxHull(0.5, 0.5, 0.5)

	xfB := b3.Transform{{1.2, 0.0, 0.0}, b3.Quat_identity}
	aq := b3.ComputeSeparatingAxis(&hullA.base, &hullB.base, xfB, true)
	q := b3.GetBestAxis(&aq)

	ENSURE(t, q.type == .faceAxisA) or_return
	ENSURE_SMALL(t, q.separation - 0.2, 1e-5) or_return
	ENSURE_SMALL(t, q.normal.x - 1.0, 1e-6) or_return
	ENSURE_SMALL(t, q.normal.y, 1e-6) or_return
	ENSURE_SMALL(t, q.normal.z, 1e-6) or_return

	planesA := b3.GetHullPlanes(&hullA.base)
	ENSURE_SMALL(t, planesA[q.indexA].normal.x - 1.0, 1e-6) or_return

	pointsB := b3.GetHullPoints(&hullB.base)
	ENSURE_SMALL(t, pointsB[q.indexB].x + 0.5, 1e-6) or_return

	ENSURE_SMALL(t, SepAlong(&hullA.base, &hullB.base, xfB, q.normal) - q.separation, 1e-5) or_return
	return true
}

// A yawed 45 degrees about z so all of its faces sit oblique to x. Only B still has a face square to
// the gap, so the reference has to be B's minus x face. This is the case that ties when both hulls
// are aligned, and the tie is what a face A bias would hide.
@(private = "file")
FaceAxisBSeparatedTest :: proc(t: ^testing.T) -> bool {
	hullA := b3.MakeTransformedBoxHull(0.5, 0.5, 0.5, ExactRotation(kAxisZ, 0.25 * b3.PI))
	hullB := b3.MakeBoxHull(0.5, 0.5, 0.5)

	aExtent := 0.5 * kRoot2
	gap: f32 = 0.2
	d := aExtent + 0.5 + gap
	xfB := b3.Transform{{d, 0.0, 0.0}, b3.Quat_identity}

	aq := b3.ComputeSeparatingAxis(&hullA.base, &hullB.base, xfB, true)
	q := b3.GetBestAxis(&aq)

	ENSURE(t, q.type == .faceAxisB) or_return
	ENSURE_SMALL(t, q.separation - gap, 1e-5) or_return
	ENSURE_SMALL(t, q.normal.x - 1.0, 1e-5) or_return
	ENSURE_SMALL(t, q.normal.y, 1e-5) or_return
	ENSURE_SMALL(t, q.normal.z, 1e-5) or_return

	planesB := b3.GetHullPlanes(&hullB.base)
	ENSURE_SMALL(t, planesB[q.indexB].normal.x + 1.0, 1e-5) or_return

	pointsA := b3.GetHullPoints(&hullA.base)
	ENSURE_SMALL(t, pointsA[q.indexA].x - aExtent, 1e-4) or_return

	ENSURE_SMALL(t, SepAlong(&hullA.base, &hullB.base, xfB, q.normal) - q.separation, 1e-4) or_return
	return true
}

// Well beyond the speculative distance the face phase must report the axis and separation without
// running the edge phase. A sign slip in the early out would read as deep overlap.
@(private = "file")
FaceFarSeparatedTest :: proc(t: ^testing.T) -> bool {
	hullA := b3.MakeBoxHull(0.5, 0.5, 0.5)
	hullB := b3.MakeBoxHull(0.5, 0.5, 0.5)

	xfB := b3.Transform{{3.0, 0.0, 0.0}, b3.Quat_identity}
	aq := b3.ComputeSeparatingAxis(&hullA.base, &hullB.base, xfB, true)
	q := b3.GetBestAxis(&aq)

	ENSURE(t, q.type == .faceAxisA) or_return
	ENSURE_SMALL(t, q.separation - 2.0, 1e-5) or_return
	ENSURE_SMALL(t, q.normal.x - 1.0, 1e-6) or_return

	oracleNormal: b3.Vec3
	oracleSep := OracleSeparation(&hullA.base, &hullB.base, xfB, &oracleNormal)
	ENSURE_SMALL(t, q.separation - oracleSep, 1e-5) or_return
	return true
}

// The support bias is derived from each hull's AABB, not from an origin assumed to sit inside it. A
// box built far from its own local origin is the case the old diagonal bias got wrong. A is yawed
// and pushed out to x = 5, so the face B query has to run getSupport over vertices clustered near
// x = 5 and still pick the right one.
@(private = "file")
OffsetFaceAxisBTest :: proc(t: ^testing.T) -> bool {
	cx: f32 = 5.0
	placeA := b3.Transform{{cx, 0.0, 0.0}, ExactQuat(kAxisZ, 0.25 * b3.PI)}
	hullA := b3.MakeTransformedBoxHull(0.5, 0.5, 0.5, placeA)
	hullB := b3.MakeBoxHull(0.5, 0.5, 0.5)

	aExtent := cx + 0.5 * kRoot2
	gap: f32 = 0.2
	d := aExtent + 0.5 + gap
	xfB := b3.Transform{{d, 0.0, 0.0}, b3.Quat_identity}

	aq := b3.ComputeSeparatingAxis(&hullA.base, &hullB.base, xfB, true)
	q := b3.GetBestAxis(&aq)

	ENSURE(t, q.type == .faceAxisB) or_return
	ENSURE_SMALL(t, q.separation - gap, 1e-4) or_return
	ENSURE_SMALL(t, q.normal.x - 1.0, 1e-5) or_return
	ENSURE_SMALL(t, q.normal.y, 1e-5) or_return
	ENSURE_SMALL(t, q.normal.z, 1e-5) or_return

	planesB := b3.GetHullPlanes(&hullB.base)
	ENSURE_SMALL(t, planesB[q.indexB].normal.x + 1.0, 1e-5) or_return

	pointsA := b3.GetHullPoints(&hullA.base)
	ENSURE_SMALL(t, pointsA[q.indexA].x - aExtent, 1e-4) or_return

	oracleNormal: b3.Vec3
	oracleSep := OracleSeparation(&hullA.base, &hullB.base, xfB, &oracleNormal)
	ENSURE_SMALL(t, q.separation - oracleSep, 1e-4) or_return
	ENSURE_SMALL(t, SepAlong(&hullA.base, &hullB.base, xfB, q.normal) - q.separation, 1e-4) or_return
	return true
}

// Cube A yawed 45 about y presents an edge along y at x = h*root2. Cube B rolled 45 about z presents
// an edge along z at x = -h*root2. Sliding B along x makes those edges the closest features, so the
// axis is x and the separation is d - root2. The sweep straddles contact into overlap while staying
// under the speculative distance, so no face axis can short circuit the edge phase. The overlap rows
// pin the maximum separation, which is where a premature edge early out shows up.
@(private = "file")
EdgePairSweepTest :: proc(t: ^testing.T) -> bool {
	hullA := b3.MakeTransformedBoxHull(0.5, 0.5, 0.5, ExactRotation(kAxisY, 0.25 * b3.PI))
	hullB := b3.MakeTransformedBoxHull(0.5, 0.5, 0.5, ExactRotation(kAxisZ, 0.25 * b3.PI))

	distances := [?]f32{1.43, 1.42, kRoot2, 1.40, 1.38, 1.35, 1.30}

	for i: i32 = 0; i < i32(len(distances)); i += 1 {
		d := distances[i]
		expected := d - kRoot2

		xfB := b3.Transform{{d, 0.0, 0.0}, b3.Quat_identity}
		aq := b3.ComputeSeparatingAxis(&hullA.base, &hullB.base, xfB, true)
		q := b3.GetBestAxis(&aq)

		ENSURE(t, q.type == .edgePairAxis) or_return
		ENSURE_SMALL(t, q.separation - expected, 1e-4) or_return
		ENSURE_SMALL(t, q.normal.x - 1.0, 1e-4) or_return
		ENSURE_SMALL(t, q.normal.y, 1e-4) or_return
		ENSURE_SMALL(t, q.normal.z, 1e-4) or_return

		// Half edge indices are the even twin of each pair.
		ENSURE(t, (q.indexA & 1) == 0) or_return
		ENSURE(t, (q.indexB & 1) == 0) or_return

		oracleNormal: b3.Vec3
		oracleSep := OracleSeparation(&hullA.base, &hullB.base, xfB, &oracleNormal)
		ENSURE_SMALL(t, q.separation - oracleSep, 1e-4) or_return
		ENSURE_SMALL(t, SepAlong(&hullA.base, &hullB.base, xfB, q.normal) - q.separation, 1e-4) or_return
	}

	return true
}

@(private = "file")
g_seed: u32 = 987654321

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

// A wide net over randomly oriented box pairs, from clearly separated to deep overlap. The brute
// force oracle is the source of truth. Whatever axis the query returns it must be a unit vector that
// reproduces its own separation, it must not exceed the true maximum, and it must not fall short of
// it. The short fall bound is what a suboptimal edge pick trips. Thin boxes are mixed in because a
// crossed pair of long edges stays the axis of minimum penetration even deep in overlap, which is
// the hardest case for the edge phase.
@(private = "file")
SeparatingAxisOracleTest :: proc(t: ^testing.T) -> bool {
	g_seed = 987654321

	separated: i32 = 0
	penetrating: i32 = 0
	edgeWins: i32 = 0
	worstBandShortfall: f32 = 0.0
	worstExcess: f32 = 0.0

	for i: i32 = 0; i < 4000; i += 1 {
		halfA, halfB: b3.Vec3
		reach: f32

		if (i & 1) != 0 {
			// Thin crossed beams driven into deep overlap.
			halfA = b3.Vec3{NextFloat(1.0, 1.6), NextFloat(0.08, 0.15), NextFloat(0.08, 0.15)}
			halfB = b3.Vec3{NextFloat(1.0, 1.6), NextFloat(0.08, 0.15), NextFloat(0.08, 0.15)}
			reach = NextFloat(0.0, 0.4)
		} else {
			// Chunky boxes across the whole separated to overlapping range.
			halfA = b3.Vec3{NextFloat(0.3, 0.8), NextFloat(0.3, 0.8), NextFloat(0.3, 0.8)}
			halfB = b3.Vec3{NextFloat(0.3, 0.8), NextFloat(0.3, 0.8), NextFloat(0.3, 0.8)}
			reach = NextFloat(0.0, 1.7)
		}

		// Rotate A in place so it stays centered on the origin, which keeps the support bias valid.
		hullA := b3.MakeTransformedBoxHull(
			halfA.x,
			halfA.y,
			halfA.z,
			ExactRotation(NextDirection(), NextFloat(0.0, b3.PI)),
		)
		hullB := b3.MakeBoxHull(halfB.x, halfB.y, halfB.z)

		xfB := b3.Transform{b3.MulSV(reach, NextDirection()), ExactQuat(NextDirection(), NextFloat(0.0, b3.PI))}

		aq := b3.ComputeSeparatingAxis(&hullA.base, &hullB.base, xfB, true)
		q := b3.GetBestAxis(&aq)

		oracleNormal: b3.Vec3
		oracleSep := OracleSeparation(&hullA.base, &hullB.base, xfB, &oracleNormal)

		ENSURE_SMALL(t, b3.Length(q.normal) - 1.0, 1e-3) or_return
		ENSURE_SMALL(t, SepAlong(&hullA.base, &hullB.base, xfB, q.normal) - q.separation, 2e-3) or_return

		worstExcess = b3.MaxFloat(worstExcess, q.separation - oracleSep)

		// Never overstate the true maximum. This holds on every path since the reported axis is real.
		ENSURE(t, q.separation <= oracleSep + 2e-3) or_return

		// The scan stops at the first axis that clears the speculative band, so a widely separated pair
		// reports some separating axis rather than the deepest one. That is all the collide path needs,
		// since past the band it drops the pair either way. Inside the band nothing stops the scan, so
		// the result there is the exhaustive maximum, and that is what a manifold gets built from.
		if q.separation <= b3.SPECULATIVE_DISTANCE {
			worstBandShortfall = b3.MaxFloat(worstBandShortfall, oracleSep - q.separation)

			// A suboptimal axis pick lands a discrete step below the true maximum. The bound sits well
			// under that step yet clear of the edge normalization noise floor near 6e-4.
			ENSURE(t, q.separation >= oracleSep - 4e-3) or_return
		}

		if oracleSep > 0.0 {
			separated += 1
		} else {
			penetrating += 1
		}

		if q.type == .edgePairAxis {
			edgeWins += 1
		}
	}

	printf(
		"    oracle: separated=%d penetrating=%d edgeWins=%d worstBandShortfall=%.2e worstExcess=%.2e\n",
		separated,
		penetrating,
		edgeWins,
		worstBandShortfall,
		worstExcess,
	)

	// The sweep has to cover both regimes and actually drive the edge path.
	ENSURE(t, separated > 100) or_return
	ENSURE(t, penetrating > 100) or_return
	ENSURE(t, edgeWins > 20) or_return
	return true
}

// The oracle over hulls whose vertices sit far from their own local origin, which the AABB based
// bias must handle. Half the pairs offset A in its own frame, half offset B, and B is placed so the
// two centers land near each other across the separated to overlapping range. Any bias that assumed
// the origin was inside the hull would pick the wrong support and blow the separation.
@(private = "file")
OffsetHullOracleTest :: proc(t: ^testing.T) -> bool {
	g_seed = 24681012

	separated: i32 = 0
	penetrating: i32 = 0
	edgeWins: i32 = 0
	worstBandShortfall: f32 = 0.0
	worstExcess: f32 = 0.0
	worstConsistency: f32 = 0.0

	for i: i32 = 0; i < 3000; i += 1 {
		halfA := b3.Vec3{NextFloat(0.3, 0.8), NextFloat(0.3, 0.8), NextFloat(0.3, 0.8)}
		halfB := b3.Vec3{NextFloat(0.3, 0.8), NextFloat(0.3, 0.8), NextFloat(0.3, 0.8)}

		offset := b3.MulSV(NextFloat(2.0, 6.0), NextDirection())
		reach := NextFloat(0.0, 1.7)
		dir := NextDirection()

		hullA: b3.BoxHull
		hullB: b3.BoxHull
		xfB: b3.Transform

		if (i & 1) != 0 {
			// A carries the offset in its own frame. B is centered and dropped near A's center.
			placeA := b3.Transform{offset, ExactQuat(NextDirection(), NextFloat(0.0, b3.PI))}
			hullA = b3.MakeTransformedBoxHull(halfA.x, halfA.y, halfA.z, placeA)
			hullB = b3.MakeBoxHull(halfB.x, halfB.y, halfB.z)
			xfB = b3.Transform{b3.MulAdd(offset, reach, dir), ExactQuat(NextDirection(), NextFloat(0.0, b3.PI))}
		} else {
			// B carries the offset in its own frame. Place it so its world center lands near the origin.
			qB := ExactQuat(NextDirection(), NextFloat(0.0, b3.PI))
			hullA = b3.MakeBoxHull(halfA.x, halfA.y, halfA.z)
			hullB = b3.MakeOffsetBoxHull(halfB.x, halfB.y, halfB.z, offset)
			xfB = b3.Transform{b3.Sub(b3.MulSV(reach, dir), b3.RotateVector(qB, offset)), qB}
		}

		aq := b3.ComputeSeparatingAxis(&hullA.base, &hullB.base, xfB, true)
		q := b3.GetBestAxis(&aq)

		oracleNormal: b3.Vec3
		oracleSep := OracleSeparation(&hullA.base, &hullB.base, xfB, &oracleNormal)

		consistency := b3.AbsFloat(SepAlong(&hullA.base, &hullB.base, xfB, q.normal) - q.separation)
		worstConsistency = b3.MaxFloat(worstConsistency, consistency)
		worstExcess = b3.MaxFloat(worstExcess, q.separation - oracleSep)

		// Consistency is the sharp bias check: a wrong support pick on an offset hull would throw the
		// returned normal off its own separation by a vertex spacing, not a noise floor. It holds on
		// every path, early out or not, so it stays unconditional. The oracle bounds are looser since
		// differencing coordinates out at radius six costs a few more digits.
		ENSURE_SMALL(t, b3.Length(q.normal) - 1.0, 1e-3) or_return
		ENSURE(t, consistency < 1e-3) or_return
		ENSURE(t, q.separation <= oracleSep + 3e-3) or_return

		// Only the in band result feeds a manifold, and only there does the scan run to completion.
		if q.separation <= b3.SPECULATIVE_DISTANCE {
			worstBandShortfall = b3.MaxFloat(worstBandShortfall, oracleSep - q.separation)
			ENSURE(t, q.separation >= oracleSep - 8e-3) or_return
		}

		if oracleSep > 0.0 {
			separated += 1
		} else {
			penetrating += 1
		}

		if q.type == .edgePairAxis {
			edgeWins += 1
		}
	}

	printf(
		"    offset: separated=%d penetrating=%d edgeWins=%d worstBandShortfall=%.2e worstExcess=%.2e worstConsistency=%.2e\n",
		separated,
		penetrating,
		edgeWins,
		worstBandShortfall,
		worstExcess,
		worstConsistency,
	)

	ENSURE(t, separated > 100) or_return
	ENSURE(t, penetrating > 100) or_return
	ENSURE(t, edgeWins > 20) or_return
	return true
}

@(test)
SeparatingAxisTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, FaceAxisASeparatedTest) {return}
	if !RUN_SUBTEST(t, FaceAxisBSeparatedTest) {return}
	if !RUN_SUBTEST(t, FaceFarSeparatedTest) {return}
	if !RUN_SUBTEST(t, OffsetFaceAxisBTest) {return}
	if !RUN_SUBTEST(t, EdgePairSweepTest) {return}
	if !RUN_SUBTEST(t, SeparatingAxisOracleTest) {return}
	if !RUN_SUBTEST(t, OffsetHullOracleTest) {return}
}
