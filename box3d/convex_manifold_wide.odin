// Ported from box3d src/convex_manifold_wide.inl at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:intrinsics"

// This is the wide4.c instantiation (B3_SIMD_WIDTH 4): B3_WIDE(name) is nameW4. `./b3 gen` writes the wide8.c one
// (convex_manifold_wide8.odin, B3_SIMD_WIDTH 8, AVX2) from this file.

// unified: B3_SIMD_WIDTH, file-private (simd_wide.odin). The B3_SIMD_NONE scalar branch is kept behind a switch
// (off, as on every SIMD platform).
@(private = "file")
SIMD_WIDTH :: 4
@(private = "file")
SIMD_NONE :: false
// unified end

#assert(SIMD_WIDTH <= HULL_SOA_PADDING, "hull SoA padding must cover the SIMD width")

@(private = "file")
RoundUpToSIMDWidth :: #force_inline proc(count: i32) -> i32 {
	return (count + SIMD_WIDTH - 1) & ~i32(SIMD_WIDTH - 1)
}

// Transform a SoA point/normal stream (already split into X/Y/Z) by out = -(R*v (+t)).
// The inputs come straight from the hull's stored SoA arrays, so there's no transpose here.
@(private = "file")
NegativeTransformFromSoA :: #force_inline proc(
	R: Matrix3,
	p: Vec3,
	inX: [^]f32,
	inY: [^]f32,
	inZ: [^]f32,
	n: i32,
	outX: [^]f32,
	outY: [^]f32,
	outZ: [^]f32,
	isPoint: bool,
) {
	// row-column
	r0 := Vec3W{SplatW(R.cx.x), SplatW(R.cy.x), SplatW(R.cz.x)}
	r1 := Vec3W{SplatW(R.cx.y), SplatW(R.cy.y), SplatW(R.cz.y)}
	r2 := Vec3W{SplatW(R.cx.z), SplatW(R.cy.z), SplatW(R.cz.z)}

	t := Vec3W{ZeroW(), ZeroW(), ZeroW()}
	if isPoint {
		t = SplatVW(p)
	}

	for i: i32 = 0; i < n; i += SIMD_WIDTH {
		v := LoadVW(inX[i:], inY[i:], inZ[i:])

		// Rotate one vector per lane
		out := Vec3W{DotW(r0, v), DotW(r1, v), DotW(r2, v)}

		if isPoint {
			out = AddVW(out, t)
		}

		StoreVW(outX[i:], outY[i:], outZ[i:], NegVW(out))
	}
}

#assert(MAX_HULL_VERTICES == 128, "must be 128")

@(private = "file")
HULL_BIT_COUNT :: 7

// SIMD support point calculation using a SoA vertex array padded with repeats of the first vertex
// to a multiple of 8.
//
// This minimizes (bias - dot), where the caller is expected to provide a bias that makes this always positive.
// It can be direction dependent. The bias should be just big enough to ensure the value is positive because
// an excessive bias causes a precision loss in the support calculation.
//
// The vertex index is embedded in the low B3_HULL_BIT_COUNT mantissa bits of the value. By minimizing a value that
// is always positive, the minimum carries the smallest index so that padded SoA values will never win. This is the
// purpose of using the bias instead of maximizing the dot directly.
//
// The support is then recomputed exactly as dot(normal, vertex), without the embedded index.
// todo consider using this for GJK
@(private = "file")
GetSupportWide :: #force_inline proc(
	normal: Vec3,
	vx: [^]f32,
	vy: [^]f32,
	vz: [^]f32,
	n: i32,
	bias: f32,
	support: ^f32,
	vertexIndex: ^i32,
) {
	normalW := SplatVW(normal)
	biasV := SplatW(bias)

	// Start the minimum at a large value.
	minValue := SplatW(f32(0h7F800000))

	// Tail lanes hold vertex 0 with index bits >= vertexCount, so they never become the min value.
	for i: i32 = 0; i < n; i += SIMD_WIDTH {
		v := LoadVW(vx[i:], vy[i:], vz[i:])
		d := DotW(normalW, v)

		// This is always positive.
		value := SubW(biasV, d)
		augmentedValue := EmbedIndexW(value, i, HULL_BIT_COUNT)
		minValue = MinW(minValue, augmentedValue)
	}

	// One horizontal min, the winning lane's value and index bits ride through.
	vi := MinIndexW(minValue, HULL_BIT_COUNT)

	// Exact support for the chosen vertex.
	vertexIndex^ = vi

	// Dot product
	support^ = normal.x * vx[vi] + normal.y * vy[vi] + normal.z * vz[vi]
}

@(private = "file")
GetFaceSeparation :: #force_inline proc(
	direction: Vec3,
	planeSeparation: f32,
	vx: [^]f32,
	vy: [^]f32,
	vz: [^]f32,
	n: i32,
	center: Vec3,
	extents: Vec3,
	vertexIndex: ^i32,
) -> f32 {
	bias := Dot(direction, center) + 1.0625 * Dot(Abs(direction), extents)
	support: f32
	GetSupportWide(direction, vx, vy, vz, n, bias, &support, vertexIndex)
	return planeSeparation - support
}

// Wide dot(n, d) for all face normals n of the hull, padded to the SIMD width.
@(private = "file")
GetFaceDots :: #force_inline proc(hull: ^HullData, d: Vec3, dots: [^]f32) {
	faceStride := GetHullSoaStride(hull.faceCount)
	nx := GetHullSoaNormals(hull)
	ny := nx[faceStride:]
	nz := ny[faceStride:]

	dW := SplatVW(d)

	wideFaceCount := RoundUpToSIMDWidth(hull.faceCount)
	for i: i32 = 0; i < wideFaceCount; i += SIMD_WIDTH {
		// dot product per lane
		m := DotW(LoadVW(nx[i:], ny[i:], nz[i:]), dW)
		StoreW(dots[i:], m)
	}
}

@(private = "file")
PARALLEL_TOL :: f32(1e-4)

// Inscribed sphere edge test. https://box2d.org/posts/2026/09/inscribed-spheres/
// The implementation here is a mix of the versions from Cairn Overturf and Dirk Gregorius:
// https://gist.github.com/cairnc/dee7a2866da0709f2d9a77b6493b57b5
// https://gist.github.com/dgregorius/e6751b5c00937cd21af63cba3c53c861
// This benchmarks faster than the version from the blog post.
@(private = "file")
TestEdgeCandidateSorted :: #force_inline proc(a1: f32, a2: f32, c: f32, bound: f32) -> i32 {
	// We want to maximize
	//
	//   f(t) = dot(d, nlerp(n1, n2, t))
	//
	// over t in [0, 1], where
	//
	//   a1 = dot(n1, d)
	//   a2 = dot(n2, d)
	//   c  = dot(n1, n2).
	//
	// The maximum can occur either at an endpoint or at an interior
	// critical point where f'(t) = 0.

	// Since the endpoint values are a1 and a2, only the larger endpoint
	// needs to be tested against the bound.
	hi := MaxFloat(a1, a2)
	lo := MinFloat(a1, a2)
	exterior := i32(hi >= bound)

	// Differentiating f(t) gives
	//
	//           a2 - c*a1 - (1 - c)*(a1 + a2)*t
	//   f'(t) = --------------------------------------
	//           ((1 - t)^2 + t^2 + 2*c*t*(1 - t))^(3/2)
	//
	// The denominator is positive and the numerator is linear in t.
	// An interior maximum therefore requires
	//
	//   f'(0) >= 0  ->  a2 >= c*a1
	//   f'(1) <= 0  ->  a1 >= c*a2.
	//
	// After sorting these become
	//
	//   hi >= c*lo
	//   lo >= c*hi.
	//
	// The second is always the stricter condition since
	//
	//   (hi - c*lo) - (lo - c*hi)
	//       = (1 + c)*(hi - lo) >= 0.
	//
	// Thus both conditions reduce to u >= 0.
	u := lo - c * hi
	maxIsInterior := i32(u >= 0.0)

	// Solving f'(t) = 0 and evaluating f(t) at tmax gives
	//
	//   f(tmax)^2 =
	//       (a1^2 + a2^2 - 2*c*a1*a2) / (1 - c^2).
	//
	// After sorting,
	//
	//   hi^2 + lo^2 - 2*c*hi*lo
	//       = hi^2*(1 - c^2) + u^2.
	//
	// Let s = 1 - c^2 and rearrange f(tmax) >= bound:
	//
	//   u^2 >= (bound^2 - hi^2)*s.
	s := 1.0 - c * c
	boundTerm := (bound - hi) * (bound + hi)
	lhs := u * u
	rhs := boundTerm * s
	maxBeatsBound := i32(lhs >= rhs)

	// The interior expression contains 1 - c^2 in its denominator.
	// When this approaches zero the arc is degenerate or ill-conditioned,
	// so conservatively keep the edge as a candidate.
	nearlyParallel := i32(s < PARALLEL_TOL)

	interior := maxIsInterior & (maxBeatsBound | nearlyParallel)

	return exterior | interior
}

// dot(n, otherCenter) - offset for all faces of the hull, padded to the SIMD width.
@(private = "file")
GetFacePlaneSeparations :: #force_inline proc(hull: ^HullData, otherCenter: Vec3, separations: [^]f32) {
	GetFaceDots(hull, otherCenter, separations)

	planes := GetHullPlanes(hull)
	faceCount := hull.faceCount
	for i: i32 = 0; i < faceCount; i += 1 {
		separations[i] -= planes[i].offset
	}
}

// The number of edge pair tests needed to make the extra culling pass worthwhile.
@(private = "file")
EDGE_PROBE_MIN_TESTS :: 10

// Re-test edge candidates against the plane separations of a probe point on the other hull
@(private = "file")
FilterEdgeCandidates :: #force_inline proc(
	hull: ^HullData,
	planeSeparations: [^]f32,
	bound: f32,
	edgeIndices: [^]i32,
	count: i32,
) -> i32 {
	halfEdges := GetHullEdges(hull)
	cosines := GetHullEdgeCosines(hull)
	keptCount: i32 = 0

	for k: i32 = 0; k < count; k += 1 {
		i := edgeIndices[k]
		i1 := halfEdges[i].face
		i2 := halfEdges[i + 1].face
		c := cosines[i >> 1]
		edgeIndices[keptCount] = i
		keptCount += TestEdgeCandidateSorted(planeSeparations[i1], planeSeparations[i2], c, bound)
	}

	return keptCount
}

// Temporary abbreviations for convenience.
@(private = "file")
NE :: (MAX_HULL_EDGES + SIMD_WIDTH)
@(private = "file")
NF :: (MAX_HULL_FACES + SIMD_WIDTH)
@(private = "file")
NV :: (MAX_HULL_VERTICES + SIMD_WIDTH)

// SIMD separating axis test based on an implementation developed by Cairn Overturf.
// See his article: https://cairnc.github.io/posts/improvements-to-the-separating-axis/
ComputeSeparatingAxisW4 :: proc(hullA: ^HullData, hullB: ^HullData, xfB: Transform, earlyReturn: bool) -> AxisQuery {
	R := MakeMatrixFromQuat(xfB.q)
	invR := Transpose(R)

	speculativeDistance := SPECULATIVE_DISTANCE

	res := AxisQuery {
		faceA = {
			normal = Vec3_zero,
			separation = -f32(0h7F800000),
			indexA = NULL_INDEX,
			indexB = NULL_INDEX,
			type = .faceAxisA,
		},
		faceB = {
			normal = Vec3_zero,
			separation = -f32(0h7F800000),
			indexA = NULL_INDEX,
			indexB = NULL_INDEX,
			type = .faceAxisB,
		},
		edge = {
			normal = Vec3_zero,
			separation = -f32(0h7F800000),
			indexA = NULL_INDEX,
			indexB = NULL_INDEX,
			type = .edgePairAxis,
		},
		separatedFeature = .invalidAxis,
	}

	faceCountA := hullA.faceCount
	planesA := GetHullPlanes(hullA)

	vertexStrideB := GetHullSoaStride(hullB.vertexCount)
	vxB := GetHullSoaVertices(hullB)
	vyB := vxB[vertexStrideB:]
	vzB := vyB[vertexStrideB:]
	wideVertexCountB := RoundUpToSIMDWidth(hullB.vertexCount)

	cB := AABB_Center(hullB.aabb)
	hB := AABB_Extents(hullB.aabb)

	// The hulls have a precomputed inner radius and centroid.
	// A given axis cannot achieve a separation larger than:
	// dot(axis, centerB - centerA) - innerRadiusA - innerRadiusB
	// So this is the upper bound for the separation of a candidate axis.
	// An axis can be skipped if it has an upper bound that is less than the current
	// best separation. This lets me skip many of the candidates without computing
	// support points.

	deltaCenter := Sub(Add(MulMV(R, hullB.center), xfB.p), hullA.center)
	centerDistance := Length(deltaCenter)
	radius := hullA.innerRadius + hullB.innerRadius

	// Adjust the radius to ensure the best axis isn't skipped.
	radiusBound := radius - (LINEAR_SLOP + 0.001 * (centerDistance + radius))

	// unified: not ported: _Alignas(B3_WIDE_ALIGNMENT) on the local arrays (Odin can't align a local; LoadW/StoreW are unaligned)

	// Compute dot(normalA, centerDelta) for all face normals of hullA.
	dotA: [NF]f32 = ---
	GetFaceDots(hullA, deltaCenter, &dotA[0])

	// Find the face of hullA that most aligns with centerDelta.
	seedIndexA: i32 = 0
	maxDotA := dotA[0]
	for i: i32 = 1; i < faceCountA; i += 1 {
		if dotA[i] > maxDotA {
			maxDotA = dotA[i]
			seedIndexA = i
		}
	}

	// Use the seed to get a lower bound on the separation for the faces of hullA.
	floorA := -f32(0h7F800000)
	seedSeparationA := -f32(0h7F800000)
	seedVertexB: i32 = 0
	if earlyReturn {
		plane := planesA[seedIndexA]
		direction := Neg(MulMV(invR, plane.normal))
		planeSeparation := Dot(plane.normal, xfB.p) - plane.offset
		seedSeparationA = GetFaceSeparation(
			direction,
			planeSeparation,
			vxB,
			vyB,
			vzB,
			wideVertexCountB,
			cB,
			hB,
			&seedVertexB,
		)
		floorA = MinFloat(seedSeparationA, speculativeDistance)
	}

	// Test A's face planes against B's vertices.
	for i: i32 = 0; i < faceCountA; i += 1 {
		// The bound offset ensures the seed will be evaluated.
		if dotA[i] - radiusBound < MaxFloat(floorA, res.faceA.separation) {
			continue
		}

		plane := planesA[i]
		vertexIndex := seedVertexB
		separation := seedSeparationA

		// Avoid recomputing the seed face separation.
		if earlyReturn == false || i != seedIndexA {
			direction := Neg(MulMV(invR, plane.normal))
			planeSeparation := Dot(plane.normal, xfB.p) - plane.offset
			separation = GetFaceSeparation(
				direction,
				planeSeparation,
				vxB,
				vyB,
				vzB,
				wideVertexCountB,
				cB,
				hB,
				&vertexIndex,
			)
		}

		if separation > res.faceA.separation {
			res.faceA.normal = plane.normal
			res.faceA.separation = separation
			res.faceA.indexA = i
			res.faceA.indexB = vertexIndex
			if separation > speculativeDistance && earlyReturn {
				res.separatedFeature = .faceAxisA
				return res
			}
		}
	}

	VALIDATE(res.faceA.indexA != NULL_INDEX)

	faceCountB := hullB.faceCount
	planesB := GetHullPlanes(hullB)

	vertexStrideA := GetHullSoaStride(hullA.vertexCount)
	vxA := GetHullSoaVertices(hullA)
	vyA := vxA[vertexStrideA:]
	vzA := vyA[vertexStrideA:]
	wideVertexCountA := RoundUpToSIMDWidth(hullA.vertexCount)

	cA := AABB_Center(hullA.aabb)
	hA := AABB_Extents(hullA.aabb)

	// Similarly, find the face of hullB that most aligns with the vector pointing from centerB to centerA.
	dotB: [NF]f32 = ---
	GetFaceDots(hullB, Neg(MulMV(invR, deltaCenter)), &dotB[0])

	seedIndexB: i32 = 0
	maxDotB := dotB[0]
	for i: i32 = 1; i < faceCountB; i += 1 {
		if dotB[i] > maxDotB {
			maxDotB = dotB[i]
			seedIndexB = i
		}
	}

	// Get a lower bound on the separation for the faces of hullB.
	floorB := -f32(0h7F800000)
	seedSeparationB := -f32(0h7F800000)
	seedVertexA: i32 = 0
	if earlyReturn {
		plane := planesB[seedIndexB]
		direction := Neg(MulMV(R, plane.normal))
		planeSeparation := Dot(direction, xfB.p) - plane.offset
		seedSeparationB = GetFaceSeparation(
			direction,
			planeSeparation,
			vxA,
			vyA,
			vzA,
			wideVertexCountA,
			cA,
			hA,
			&seedVertexA,
		)

		// Include the floor set by hull A faces.
		floorB = MaxFloat(seedSeparationB, res.faceA.separation)
		floorB = MinFloat(floorB, speculativeDistance)
	}

	// Test B's face planes against A's vertices.
	for i: i32 = 0; i < faceCountB; i += 1 {
		if dotB[i] - radiusBound < MaxFloat(floorB, res.faceB.separation) {
			continue
		}

		plane := planesB[i]
		direction := Neg(MulMV(R, plane.normal))
		vertexIndex := seedVertexA
		separation := seedSeparationB

		// Avoid recomputing the seed face separation.
		if earlyReturn == false || i != seedIndexB {
			planeSeparation := Dot(direction, xfB.p) - plane.offset
			separation = GetFaceSeparation(
				direction,
				planeSeparation,
				vxA,
				vyA,
				vzA,
				wideVertexCountA,
				cA,
				hA,
				&vertexIndex,
			)
		}

		if separation > res.faceB.separation {
			res.faceB.normal = direction
			res.faceB.separation = separation
			res.faceB.indexA = vertexIndex
			res.faceB.indexB = i
			if separation > speculativeDistance && earlyReturn {
				res.separatedFeature = .faceAxisB
				return res
			}
		}
	}

	// Transform B into A's space once, into SoA arrays. Extra space so tail can be set to zero.
	#assert((MAX_HULL_EDGES & (SIMD_WIDTH - 1)) == 0, "must be multiple of SIMD width")
	#assert((MAX_HULL_FACES & (HULL_SOA_PADDING - 1)) == 0, "must be multiple of hull SoA padding")
	#assert((MAX_HULL_VERTICES & (HULL_SOA_PADDING - 1)) == 0, "must be multiple of hull SoA padding")

	VALIDATE(
		earlyReturn == false || centerDistance >= MaxFloat(res.faceA.separation, res.faceB.separation) + radiusBound,
	)

	// Gather edges of A that can feasibly create a separating axis that beats the maximum face separation.
	halfEdgeCountA := hullA.edgeCount
	halfEdgesA := GetHullEdges(hullA)
	edgeIndicesA: [NE]i32 = ---
	na: i32 = 0

	halfEdgeCountB := hullB.edgeCount
	halfEdgesB := GetHullEdges(hullB)
	edgeIndicesB: [MAX_HULL_EDGES]i32 = ---
	nb: i32 = 0

	maxFaceSeparation := MaxFloat(res.faceA.separation, res.faceB.separation)
	boundSlack := radius - radiusBound
	thresholdA := earlyReturn ? maxFaceSeparation + hullB.innerRadius - boundSlack : -f32(0h7F800000)
	thresholdB := earlyReturn ? maxFaceSeparation + hullA.innerRadius - boundSlack : -f32(0h7F800000)

	centerBinA := Add(MulMV(R, hullB.center), xfB.p)
	centerAinB := MulMV(invR, Sub(hullA.center, xfB.p))

	planeDotA: [NF]f32 = ---
	planeDotB: [NF]f32 = ---
	GetFacePlaneSeparations(hullA, centerBinA, &planeDotA[0])
	GetFacePlaneSeparations(hullB, centerAinB, &planeDotB[0])

	cosinesA := GetHullEdgeCosines(hullA)
	cosinesB := GetHullEdgeCosines(hullB)

	for i: i32 = 0; i < halfEdgeCountA; i += 2 {
		i1 := halfEdgesA[i].face
		i2 := halfEdgesA[i + 1].face
		c := cosinesA[i >> 1]
		edgeIndicesA[na] = i
		na += TestEdgeCandidateSorted(planeDotA[i1], planeDotA[i2], c, thresholdA)
	}

	for i: i32 = 0; i < halfEdgeCountB; i += 2 {
		i1 := halfEdgesB[i].face
		i2 := halfEdgesB[i + 1].face
		c := cosinesB[i >> 1]
		edgeIndicesB[nb] = i
		nb += TestEdgeCandidateSorted(planeDotB[i1], planeDotB[i2], c, thresholdB)
	}

	// Apply additional culling use the support points for the best face axes.
	// The support vertices of the best faces are points on the other hull, so an edge pair axis cannot
	// have a larger separation than the plane separation of these points over the arc of the edge.
	// This can slow down boxes so skip this if there are not enough edge candidates.
	if earlyReturn && nb * ((na + 3) >> 2) >= EDGE_PROBE_MIN_TESTS {
		probeBound := maxFaceSeparation - (0.1 * LINEAR_SLOP + 0.001 * AbsFloat(centerDistance + radius))

		vertexB := res.faceA.indexB != NULL_INDEX ? res.faceA.indexB : seedVertexB
		probeB := Vec3{vxB[vertexB], vyB[vertexB], vzB[vertexB]}
		GetFacePlaneSeparations(hullA, Add(MulMV(R, probeB), xfB.p), &planeDotA[0])
		na = FilterEdgeCandidates(hullA, &planeDotA[0], probeBound, &edgeIndicesA[0], na)

		vertexA := res.faceB.indexA != NULL_INDEX ? res.faceB.indexA : seedVertexA
		probeA := Vec3{vxA[vertexA], vyA[vertexA], vzA[vertexA]}
		GetFacePlaneSeparations(hullB, MulMV(invR, Sub(probeA, xfB.p)), &planeDotB[0])
		nb = FilterEdgeCandidates(hullB, &planeDotB[0], probeBound, &edgeIndicesB[0], nb)
	}

	if na == 0 || nb == 0 {
		// No edge candidates found.
		return res
	}

	// The alignments below are not necessary, but they don't hurt.

	// B face normals in A space, negated.
	bFNx: [NF]f32 = ---
	bFNy: [NF]f32 = ---
	bFNz: [NF]f32 = ---

	// B vertices in A space, negated.
	bWx: [NV]f32 = ---
	bWy: [NV]f32 = ---
	bWz: [NV]f32 = ---

	faceStrideB := GetHullSoaStride(faceCountB)
	nxB := GetHullSoaNormals(hullB)
	nyB := nxB[faceStrideB:]
	nzB := nyB[faceStrideB:]

	wideFaceCountB := RoundUpToSIMDWidth(faceCountB)
	NegativeTransformFromSoA(R, xfB.p, nxB, nyB, nzB, wideFaceCountB, &bFNx[0], &bFNy[0], &bFNz[0], false)
	NegativeTransformFromSoA(R, xfB.p, vxB, vyB, vzB, wideVertexCountB, &bWx[0], &bWy[0], &bWz[0], true)

	// Per A edge data, already in A's space so just gathered. n0 and n1 are the two face
	// normals, d the edge vector av1-av0, v0 the first vertex. Tol is the
	// parallel edge tolerance, scaled by the edge length.
	aN0x: [NE]f32 = ---
	aN0y: [NE]f32 = ---
	aN0z: [NE]f32 = ---
	aN1x: [NE]f32 = ---
	aN1y: [NE]f32 = ---
	aN1z: [NE]f32 = ---
	// dir = av1 - av0
	aDx: [NE]f32 = ---
	aDy: [NE]f32 = ---
	aDz: [NE]f32 = ---
	aV0x: [NE]f32 = ---
	aV0y: [NE]f32 = ---
	aV0z: [NE]f32 = ---
	aTol: [NE]f32 = ---

	squaredTol := PARALLEL_EDGE_TOL * PARALLEL_EDGE_TOL
	for k: i32 = 0; k < na; k += 1 {
		edge := &halfEdgesA[edgeIndicesA[k]]
		twin := &halfEdgesA[edgeIndicesA[k] + 1]

		A := planesA[edge.face].normal
		B := planesA[twin.face].normal
		aN0x[k] = A.x
		aN0y[k] = A.y
		aN0z[k] = A.z
		aN1x[k] = B.x
		aN1y[k] = B.y
		aN1z[k] = B.z

		v0 := edge.origin
		v1 := twin.origin

		aDx[k] = vxA[v1] - vxA[v0]
		aDy[k] = vyA[v1] - vyA[v0]
		aDz[k] = vzA[v1] - vzA[v0]
		aV0x[k] = vxA[v0]
		aV0y[k] = vyA[v0]
		aV0z[k] = vzA[v0]

		aTol[k] = squaredTol * (aDx[k] * aDx[k] + aDy[k] * aDy[k] + aDz[k] * aDz[k])
	}

	// Zero the tail lanes.
	zero := ZeroW()
	zeroV := Vec3W{zero, zero, zero}
	StoreVW(&aN0x[na], &aN0y[na], &aN0z[na], zeroV)
	StoreVW(&aN1x[na], &aN1y[na], &aN1z[na], zeroV)
	StoreVW(&aDx[na], &aDy[na], &aDz[na], zeroV)
	StoreVW(&aV0x[na], &aV0y[na], &aV0z[na], zeroV)

	StoreW(&aTol[na], zero)

	linearSlop := LINEAR_SLOP

	when SIMD_NONE {

		// The SIMD emulated version of this code is very slow. This is a purely scalar version
		// for platforms that don't have SIMD capability. It is much faster than SIMD emulation.
		// WARNING: this math needs to match the SIMD version for cross platform determinism.

		EPS := -linearSlop * linearSlop

		for j: i32 = 0; j < nb; j += 1 {
			edge := &halfEdgesB[edgeIndicesB[j]]
			twin := &halfEdgesB[edgeIndicesB[j] + 1]
			f0 := edge.face
			f1 := twin.face
			v0 := edge.origin
			v1 := twin.origin

			C := Vec3{bFNx[f0], bFNy[f0], bFNz[f0]}
			D := Vec3{bFNx[f1], bFNy[f1], bFNz[f1]}
			bv0 := Vec3{bWx[v0], bWy[v0], bWz[v0]}
			bv1 := Vec3{bWx[v1], bWy[v1], bWz[v1]}
			DC := Sub(bv1, bv0)

			for i: i32 = 0; i < na; i += 1 {
				d := Vec3{aDx[i], aDy[i], aDz[i]}

				// CBA = C.dir, DBA = D.dir, where dir = B_x_A
				CBA := Dot(C, d)
				DBA := Dot(D, d)
				if CBA * DBA >= EPS {
					continue
				}

				n0 := Vec3{aN0x[i], aN0y[i], aN0z[i]}
				n1 := Vec3{aN1x[i], aN1y[i], aN1z[i]}

				// ADC = n0.DC, BDC = n1.DC, where DC = D_x_C
				ADC := Dot(n0, DC)
				BDC := Dot(n1, DC)
				if ADC * BDC >= EPS || CBA * BDC >= EPS {
					continue
				}

				// Reject near parallel edges
				maxCD := MaxFloat(CBA * CBA, DBA * DBA)
				if maxCD <= aTol[i] {
					continue
				}

				// t = -CBA / (DBA - CBA)
				t := -CBA / (DBA - CBA)

				// normal = lerp(t, C, D) = C + (D-C)*t
				n := MulAdd(C, t, Sub(D, C))
				len2 := Dot(n, n)

				inv := 1.0 / intrinsics.sqrt(len2)
				n = MulSV(inv, n)

				// separation = -dot(normal, av0 + bv0)
				av0 := Vec3{aV0x[i], aV0y[i], aV0z[i]}
				separation := -Dot(Add(av0, bv0), n)

				if separation > res.edge.separation {
					res.edge.normal = n
					res.edge.separation = separation

					// Half edge index
					res.edge.indexA = edgeIndicesA[i]
					res.edge.indexB = edgeIndicesB[j]

					if separation > speculativeDistance && earlyReturn {
						res.separatedFeature = .edgePairAxis
						return res
					}
				}
			}
		}

	} else {

		// This tolerance can skip edges shorter than 1cm.
		EPS := SplatW(-linearSlop * linearSlop)
		INF := SplatW(f32(0h7F800000))

		for j: i32 = 0; j < nb; j += 1 {
			edge := &halfEdgesB[edgeIndicesB[j]]
			twin := &halfEdgesB[edgeIndicesB[j] + 1]
			f0 := edge.face
			f1 := twin.face
			v0 := edge.origin
			v1 := twin.origin

			nC := Vec3{bFNx[f0], bFNy[f0], bFNz[f0]}
			nD := Vec3{bFNx[f1], bFNy[f1], bFNz[f1]}
			pB := Vec3{bWx[v0], bWy[v0], bWz[v0]}
			qB := Vec3{bWx[v1], bWy[v1], bWz[v1]}

			C := SplatVW(nC)
			D := SplatVW(nD)
			DC := SplatVW(Sub(qB, pB))
			bv0 := SplatVW(pB)

			for i: i32 = 0; i < na; i += SIMD_WIDTH {
				n0 := LoadVW(&aN0x[i], &aN0y[i], &aN0z[i])
				n1 := LoadVW(&aN1x[i], &aN1y[i], &aN1z[i])
				d := LoadVW(&aDx[i], &aDy[i], &aDz[i])
				av0 := LoadVW(&aV0x[i], &aV0y[i], &aV0z[i])

				tol := LoadW(&aTol[i])

				// CBA = C.dir, DBA = D.dir, where dir = B_x_A
				CBA := DotW(C, d)
				DBA := DotW(D, d)
				// ADC = n0.DC, BDC = n1.DC, where DC = D_x_C
				ADC := DotW(n0, DC)
				BDC := DotW(n1, DC)

				// Gauss map arc crossing test, CBA*DBA<eps and ADC*BDC<eps and CBA*BDC<eps
				m1 := LessThanW(MulW(CBA, DBA), EPS)
				m2 := LessThanW(MulW(ADC, BDC), EPS)
				m3 := LessThanW(MulW(CBA, BDC), EPS)

				// Reject near parallel edges. The arc lerp is ill conditioned when both of B's normals are nearly
				// perpendicular to edge A, a scale invariant sine threshold relative to the edge length.
				maxCD := MaxW(MulW(CBA, CBA), MulW(DBA, DBA))
				notParallel := GreaterThanW(maxCD, tol)
				mask := AndW(AndW(m1, m2), AndW(m3, notParallel))

				// Most A-edges fail the Gauss test, so skip the divide, sqrt and support work when no
				// lane passed.
				if AnyTrueW(mask) == false {
					continue
				}

				// t = -CBA / (DBA - CBA)
				t := DivW(NegW(CBA), SubW(DBA, CBA))

				// normal = lerp(t, C, D) = C + (D-C)*t
				n := MulAddSVW(C, t, SubVW(D, C))

				// normalize
				len2 := DotW(n, n)
				inv := DivW(SplatW(1.0), SqrtW(len2))
				n = MulSVW(inv, n)

				// support = dot(normal, av0 + bv0)
				support := DotW(AddVW(av0, bv0), n)

				// Lanes that fail the Gauss test can never win.
				support = BlendW(INF, support, mask)
				separation := NegW(support)

				// Test all supports against the running best at once. If none beats it, skip the
				// store and scalar reduction.
				improves := GreaterThanW(separation, SplatW(res.edge.separation))
				if AnyTrueW(improves) == false {
					continue
				}

				sA: [SIMD_WIDTH]f32 = ---
				nxA: [SIMD_WIDTH]f32 = ---
				nyA: [SIMD_WIDTH]f32 = ---
				nzA: [SIMD_WIDTH]f32 = ---
				StoreW(&sA[0], separation)
				StoreVW(&nxA[0], &nyA[0], &nzA[0], n)

				// Reduce in lane order so ties keep the first edge and the early out takes the first
				// improving support below zero. Padded tail lanes carry +INF support, so they never
				// update or index edges out of range.
				for lane: i32 = 0; lane < SIMD_WIDTH; lane += 1 {
					ei := i + lane
					s := sA[lane]
					if s > res.edge.separation {
						res.edge.normal = Vec3{nxA[lane], nyA[lane], nzA[lane]}
						res.edge.separation = s

						// Half edge index
						res.edge.indexA = edgeIndicesA[ei]
						res.edge.indexB = edgeIndicesB[j]

						if s > speculativeDistance && earlyReturn {
							res.separatedFeature = .edgePairAxis
							return res
						}
					}
				}
			}
		}
	}

	return res
}
