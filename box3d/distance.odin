// Ported from box3d src/distance.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

// Dirk Gregorius contributed portions of this code

@(private = "file")
MAX_SIMPLEX_VERTICES :: 4
@(private = "file")
MAX_GJK_ITERATIONS :: 32

GetProxySupport :: proc(proxy: ^ShapeProxy, axis: Vec3) -> i32 {
	count := proxy.count
	points := proxy.points

	assert(count > 0)
	assert(points != nil)

	// We move the first vertex into the origin for improved precision.
	// This is necessary since we don't have shape transforms and
	// vertices can potentially be far away from the origin (large).
	origin := points[0]
	maxIndex: i32 = 0
	maxProjection: f32 = 0.0

	for index: i32 = 1; index < count; index += 1 {
		// We subtract the first vertex since we are shifting into the origin.
		projection := Dot(axis, Sub(points[index], origin))
		if projection > maxProjection {
			maxIndex = index
			maxProjection = projection
		}
	}

	return maxIndex
}

GetPointSupport :: proc(points: [^]Vec3, count: i32, axis: Vec3) -> i32 {
	assert(count > 0)
	assert(points != nil)

	// We move the first vertex into the origin for improved precision.
	// This is necessary since we don't have shape transforms and
	// vertices can potentially be far away from the origin (large).
	origin := points[0]
	maxIndex: i32 = 0
	maxProjection: f32 = 0.0

	for index: i32 = 1; index < count; index += 1 {
		// We subtract the first vertex since we are shifting into the origin.
		projection := Dot(axis, Sub(points[index], origin))
		if projection > maxProjection {
			maxIndex = index
			maxProjection = projection
		}
	}

	return maxIndex
}

@(private = "file")
BarycentricCoordsEdge :: proc(out: ^[3]f32, a: Vec3, b: Vec3) {
	ab := Sub(b, a)

	// Last element is divisor
	divisor := Dot(ab, ab)

	out[0] = Dot(b, ab)
	out[1] = -Dot(a, ab)
	out[2] = divisor
}

@(private = "file")
BarycentricCoordsTri :: proc(out: ^[4]f32, a: Vec3, b: Vec3, c: Vec3) {
	ab := Sub(b, a)
	ac := Sub(c, a)

	bXC := Cross(b, c)
	cXA := Cross(c, a)
	aXB := Cross(a, b)

	abXAc := Cross(ab, ac)

	// Last element is divisor
	divisor := Dot(abXAc, abXAc)

	out[0] = Dot(bXC, abXAc)
	out[1] = Dot(cXA, abXAc)
	out[2] = Dot(aXB, abXAc)
	out[3] = divisor
}

@(private = "file")
BarycentricCoordsTet :: proc(out: ^[5]f32, a: Vec3, b: Vec3, c: Vec3, d: Vec3) {
	ab := Sub(b, a)
	ac := Sub(c, a)
	ad := Sub(d, a)

	// Last element is divisor (forced to be positive)
	divisor := ScalarTripleProduct(ab, ac, ad)

	sign: f32 = divisor < 0.0 ? -1.0 : 1.0
	out[0] = sign * ScalarTripleProduct(b, c, d)
	out[1] = sign * ScalarTripleProduct(a, d, c)
	out[2] = sign * ScalarTripleProduct(a, b, d)
	out[3] = sign * ScalarTripleProduct(a, c, b)
	out[4] = sign * divisor
}

@(private = "file")
GetMetric :: proc(simplex: ^Simplex) -> f32 {
	count := simplex.count
	assert(1 <= count && count <= 4)

	vertices := &simplex.vertices

	switch count {
	case 1:
		return 0.0

	case 2:
		a := vertices[0].w
		b := vertices[1].w
		return Distance(a, b)

	case 3:
		a := vertices[0].w
		b := vertices[1].w
		c := vertices[2].w
		return Length(Cross(Sub(b, a), Sub(c, a))) / 2.0

	case 4:
		a := vertices[0].w
		b := vertices[1].w
		c := vertices[2].w
		d := vertices[3].w
		return ScalarTripleProduct(Sub(b, a), Sub(c, a), Sub(d, a)) / 6.0

	case:
		assert(false, "Should never get here!")
	}

	return 0.0
}

@(private = "file")
WriteCache :: proc(cache: ^SimplexCache, simplex: ^Simplex) {
	count := simplex.count
	cache.metric = GetMetric(simplex)
	cache.count = u16(count)
	for index: i32 = 0; index < count; index += 1 {
		cache.indexA[index] = u8(simplex.vertices[index].indexA)
		cache.indexB[index] = u8(simplex.vertices[index].indexB)
	}
}

@(private = "file")
SolveSimplex2 :: proc(simplex: ^Simplex) -> bool {
	vs := &simplex.vertices
	assert(simplex.count == 2)

	// Vertex regions
	//float wAB[3];

	a := vs[0].w
	b := vs[1].w
	ab := Sub(b, a)

	// Last element is divisor
	divisor := Dot(ab, ab)

	u := Dot(b, ab)
	v := -Dot(a, ab)
	//wAB[2] = divisor;

	// V( A )
	if v <= 0.0 {
		// Reduce simplex
		simplex.count = 1
		vs[0].a = 1.0

		return true
	}

	// V( B )
	if u <= 0.0 {
		// Reduce simplex
		simplex.count = 1
		vs[0] = vs[1]
		vs[0].a = 1.0

		return true
	}

	// Edge region
	if divisor <= 0.0 {
		return false
	}

	// VR( AB )
	denominator := 1.0 / divisor
	vs[0].a = denominator * u
	vs[1].a = denominator * v

	return true
}

@(private = "file")
SolveSimplex3 :: proc(simplex: ^Simplex) -> bool {
	vs := &simplex.vertices
	assert(simplex.count == 3)

	// Get simplex (be aware of aliasing here!)
	v1 := vs[0]
	v2 := vs[1]
	v3 := vs[2]

	// Vertex regions
	wAB, wBC, wCA: [3]f32
	BarycentricCoordsEdge(&wAB, v1.w, v2.w)
	BarycentricCoordsEdge(&wBC, v2.w, v3.w)
	BarycentricCoordsEdge(&wCA, v3.w, v1.w)

	// VR( A )
	if wAB[1] <= 0.0 && wCA[0] <= 0.0 {
		// Reduce simplex
		simplex.count = 1
		vs[0] = v1
		vs[0].a = 1.0

		return true
	}

	// VR( B )
	if wBC[1] <= 0.0 && wAB[0] <= 0.0 {
		// Reduce simplex
		simplex.count = 1
		vs[0] = v2
		vs[0].a = 1.0

		return true
	}

	// VR( C )
	if wCA[1] <= 0.0 && wBC[0] <= 0.0 {
		// Reduce simplex
		simplex.count = 1
		vs[0] = v3
		vs[0].a = 1.0

		return true
	}

	// Edge regions
	wABC: [4]f32
	BarycentricCoordsTri(&wABC, v1.w, v2.w, v3.w)

	// VR( AB )
	if wABC[2] <= 0.0 && wAB[0] > 0.0 && wAB[1] > 0.0 {
		// Reduce simplex
		simplex.count = 2
		vs[0] = v1
		vs[1] = v2

		// Normalize
		divisor := wAB[2]
		if divisor <= 0.0 {
			return false
		}

		vs[0].a = wAB[0] / divisor
		vs[1].a = wAB[1] / divisor

		return true
	}

	// VR( BC )
	if wABC[0] <= 0.0 && wBC[0] > 0.0 && wBC[1] > 0.0 {
		// Reduce simplex
		simplex.count = 2
		vs[0] = v2
		vs[1] = v3

		// Normalize
		divisor := wBC[2]
		if divisor <= 0.0 {
			return false
		}

		vs[0].a = wBC[0] / divisor
		vs[1].a = wBC[1] / divisor

		return true
	}

	// VR( CA )
	if wABC[1] <= 0.0 && wCA[0] > 0.0 && wCA[1] > 0.0 {
		// Reduce simplex
		simplex.count = 2
		vs[0] = v3
		vs[1] = v1

		// Normalize
		divisor := wCA[2]
		if divisor <= 0.0 {
			return false
		}

		vs[0].a = wCA[0] / divisor
		vs[1].a = wCA[1] / divisor

		return true
	}

	// Face region
	divisor := wABC[3]
	if divisor <= 0.0 {
		return false
	}

	// VR( ABC )
	vs[0].a = wABC[0] / divisor
	vs[1].a = wABC[1] / divisor
	vs[2].a = wABC[2] / divisor

	return true
}

@(private = "file")
SolveSimplex4 :: proc(simplex: ^Simplex) -> bool {
	vs := &simplex.vertices

	// Get simplex (be aware of aliasing here!)
	assert(simplex.count == 4)
	vertexA := vs[0]
	vertexB := vs[1]
	vertexC := vs[2]
	vertexD := vs[3]

	// Vertex region
	wAB, wAC, wAD, wBC, wCD, wDB: [3]f32
	BarycentricCoordsEdge(&wAB, vertexA.w, vertexB.w)
	BarycentricCoordsEdge(&wAC, vertexA.w, vertexC.w)
	BarycentricCoordsEdge(&wAD, vertexA.w, vertexD.w)
	BarycentricCoordsEdge(&wBC, vertexB.w, vertexC.w)
	BarycentricCoordsEdge(&wCD, vertexC.w, vertexD.w)
	BarycentricCoordsEdge(&wDB, vertexD.w, vertexB.w)

	// VR( A )
	if wAB[1] <= 0.0 && wAC[1] <= 0.0 && wAD[1] <= 0.0 {
		// Reduce simplex
		simplex.count = 1
		vs[0] = vertexA

		vs[0].a = 1.0

		return true
	}

	// VR( B )
	if wAB[0] <= 0.0 && wDB[0] <= 0.0 && wBC[1] <= 0.0 {
		// Reduce simplex
		simplex.count = 1
		vs[0] = vertexB

		vs[0].a = 1.0

		return true
	}

	// VR( C )
	if wAC[0] <= 0.0 && wBC[0] <= 0.0 && wCD[1] <= 0.0 {
		// Reduce simplex
		simplex.count = 1
		vs[0] = vertexC

		vs[0].a = 1.0

		return true
	}

	// VR( D )
	if wAD[0] <= 0.0 && wCD[0] <= 0.0 && wDB[1] <= 0.0 {
		// Reduce simplex
		simplex.count = 1
		vs[0] = vertexD

		vs[0].a = 1.0

		return true
	}

	// Edge region
	wACB, wABD, wADC, wBCD: [4]f32
	BarycentricCoordsTri(&wACB, vertexA.w, vertexC.w, vertexB.w)
	BarycentricCoordsTri(&wABD, vertexA.w, vertexB.w, vertexD.w)
	BarycentricCoordsTri(&wADC, vertexA.w, vertexD.w, vertexC.w)
	BarycentricCoordsTri(&wBCD, vertexB.w, vertexC.w, vertexD.w)

	// VR( AB )
	if wABD[2] <= 0.0 && wACB[1] <= 0.0 && wAB[0] > 0.0 && wAB[1] > 0.0 {
		// Reduce simplex
		simplex.count = 2
		vs[0] = vertexA
		vs[1] = vertexB

		// Normalize
		divisor := wAB[2]
		if divisor <= 0.0 {
			return false
		}

		vs[0].a = wAB[0] / divisor
		vs[1].a = wAB[1] / divisor

		return true
	}

	// VR( AC )
	if wACB[2] <= 0.0 && wADC[1] <= 0.0 && wAC[0] > 0.0 && wAC[1] > 0.0 {
		// Reduce simplex
		simplex.count = 2
		vs[0] = vertexA
		vs[1] = vertexC

		// Normalize
		divisor := wAC[2]
		if divisor <= 0.0 {
			return false
		}

		vs[0].a = wAC[0] / divisor
		vs[1].a = wAC[1] / divisor

		return true
	}

	// VR( AD )
	if wADC[2] <= 0.0 && wABD[1] <= 0.0 && wAD[0] > 0.0 && wAD[1] > 0.0 {
		// Reduce simplex
		simplex.count = 2
		vs[0] = vertexA
		vs[1] = vertexD

		// Normalize
		divisor := wAD[2]
		if divisor <= 0.0 {
			return false
		}

		vs[0].a = wAD[0] / divisor
		vs[1].a = wAD[1] / divisor

		return true
	}

	// VR( BC )
	if wACB[0] <= 0.0 && wBCD[2] <= 0.0 && wBC[0] > 0.0 && wBC[1] > 0.0 {
		// Reduce simplex
		simplex.count = 2
		vs[0] = vertexB
		vs[1] = vertexC

		// Normalize
		divisor := wBC[2]
		if divisor <= 0.0 {
			return false
		}

		vs[0].a = wBC[0] / divisor
		vs[1].a = wBC[1] / divisor

		return true
	}

	// VR( CD )
	if wADC[0] <= 0.0 && wBCD[0] <= 0.0 && wCD[0] > 0.0 && wCD[1] > 0.0 {
		// Reduce simplex
		simplex.count = 2
		vs[0] = vertexC
		vs[1] = vertexD

		// Normalize
		divisor := wCD[2]
		if divisor <= 0.0 {
			return false
		}

		vs[0].a = wCD[0] / divisor
		vs[1].a = wCD[1] / divisor

		return true
	}

	// VR( DB )
	if wABD[0] <= 0.0 && wBCD[1] <= 0.0 && wDB[0] > 0.0 && wDB[1] > 0.0 {
		// Reduce simplex
		simplex.count = 2
		vs[0] = vertexD
		vs[1] = vertexB

		// Normalize
		divisor := wDB[2]
		if divisor <= 0.0 {
			return false
		}

		vs[0].a = wDB[0] / divisor
		vs[1].a = wDB[1] / divisor

		return true
	}

	// Face regions
	wABCD: [5]f32
	BarycentricCoordsTet(&wABCD, vertexA.w, vertexB.w, vertexC.w, vertexD.w)

	// VR( ACB )
	if wABCD[3] < 0.0 && wACB[0] > 0.0 && wACB[1] > 0.0 && wACB[2] > 0.0 {
		// Reduce simplex
		simplex.count = 3
		vs[0] = vertexA
		vs[1] = vertexC
		vs[2] = vertexB

		// Normalize
		divisor := wACB[3]
		if divisor <= 0.0 {
			return false
		}

		vs[0].a = wACB[0] / divisor
		vs[1].a = wACB[1] / divisor
		vs[2].a = wACB[2] / divisor

		return true
	}

	// VR( ABD )
	if wABCD[2] < 0.0 && wABD[0] > 0.0 && wABD[1] > 0.0 && wABD[2] > 0.0 {
		// Reduce simplex
		simplex.count = 3
		vs[0] = vertexA
		vs[1] = vertexB
		vs[2] = vertexD

		// Normalize
		divisor := wABD[3]
		if divisor <= 0.0 {
			return false
		}

		vs[0].a = wABD[0] / divisor
		vs[1].a = wABD[1] / divisor
		vs[2].a = wABD[2] / divisor

		return true
	}

	// VR( ADC )
	if wABCD[1] < 0.0 && wADC[0] > 0.0 && wADC[1] > 0.0 && wADC[2] > 0.0 {
		// Reduce simplex
		simplex.count = 3
		vs[0] = vertexA
		vs[1] = vertexD
		vs[2] = vertexC

		// Normalize
		divisor := wADC[3]
		if divisor <= 0.0 {
			return false
		}

		vs[0].a = wADC[0] / divisor
		vs[1].a = wADC[1] / divisor
		vs[2].a = wADC[2] / divisor

		return true
	}

	// VR( BCD )
	if wABCD[0] < 0.0 && wBCD[0] > 0.0 && wBCD[1] > 0.0 && wBCD[2] > 0.0 {
		// Reduce simplex
		simplex.count = 3
		vs[0] = vertexB
		vs[1] = vertexC
		vs[2] = vertexD

		// Normalize
		divisor := wBCD[3]
		if divisor <= 0.0 {
			return false
		}

		vs[0].a = wBCD[0] / divisor
		vs[1].a = wBCD[1] / divisor
		vs[2].a = wBCD[2] / divisor

		return true
	}

	// *** Inside tetrahedron ***
	divisor := wABCD[4]
	if divisor <= 0.0 {
		return false
	}

	// VR( ABCD )
	vs[0].a = wABCD[0] / divisor
	vs[1].a = wABCD[1] / divisor
	vs[2].a = wABCD[2] / divisor
	vs[3].a = wABCD[3] / divisor

	return true
}

@(private = "file")
ComputeWitnessPoints :: proc(simplex: ^Simplex, vertexA: ^Vec3, vertexB: ^Vec3) {
	vs := &simplex.vertices
	count := simplex.count
	assert(1 <= count && count <= 4)

	switch count {
	case 1:
		vertexA^ = vs[0].wA
		vertexB^ = vs[0].wB

	case 2:
		vertexA^ = Blend2(vs[0].a, vs[0].wA, vs[1].a, vs[1].wA)
		vertexB^ = Blend2(vs[0].a, vs[0].wB, vs[1].a, vs[1].wB)

	case 3:
		vertexA^ = Blend3(vs[0].a, vs[0].wA, vs[1].a, vs[1].wA, vs[2].a, vs[2].wA)
		vertexB^ = Blend3(vs[0].a, vs[0].wB, vs[1].a, vs[1].wB, vs[2].a, vs[2].wB)

	case 4:
		// Force identical points and *zero* distance
		sum := Add(Blend2(vs[0].a, vs[0].wA, vs[1].a, vs[1].wA), Blend2(vs[2].a, vs[2].wA, vs[3].a, vs[3].wA))
		vertexA^ = sum
		vertexB^ = sum

	case:
		assert(false, "Should never get here!")
	}
}

ShapeDistance :: proc(
	input: ^DistanceInput,
	cache: ^SimplexCache,
	simplexes: [^]Simplex,
	simplexCapacity: i32,
) -> DistanceOutput {
	// The query runs in frame A using the relative pose of B in A.
	xf := input.transform

	// Use matrices for faster math
	m := MakeMatrixFromQuat(xf.q)
	mt := Transpose(m)

	proxyA := &input.proxyA
	proxyB := &input.proxyB

	// Compute initial simplex from cache
	assert(cache.count <= MAX_SIMPLEX_VERTICES)

	simplex := Simplex{}
	vs := &simplex.vertices

	simplex.count = i32(cache.count)
	for i: i32 = 0; i < i32(cache.count); i += 1 {
		index1 := i32(cache.indexA[i])
		index2 := i32(cache.indexB[i])

		assert(0 <= index1 && index1 < proxyA.count)
		assert(0 <= index2 && index2 < proxyB.count)

		vertex1 := proxyA.points[index1]
		vertex2 := Add(MulMV(m, proxyB.points[index2]), xf.p)

		vs[i].indexA = index1
		vs[i].indexB = index2
		vs[i].wA = vertex1
		vs[i].wB = vertex2
		vs[i].w = Sub(vertex2, vertex1)
		vs[i].a = 0.0
	}

	// Compute the new simplex metric, if it is substantially
	// different than the old metric flush the simplex.
	if simplex.count > 0 {
		metric1 := cache.metric
		metric2 := GetMetric(&simplex)

		// todo the tetrahedron metric can be negative
		if 2.0 * metric1 < metric2 || metric2 < 0.5 * metric1 || metric2 < FLT_EPSILON {
			// Flush the simplex
			simplex.count = 0
		}
	}

	// If the cache is invalid or empty
	if simplex.count == 0 {
		vertex1 := proxyA.points[0]
		vertex2 := Add(MulMV(m, proxyB.points[0]), xf.p)

		simplex.count = 1
		simplex.vertices[0].indexA = 0
		simplex.vertices[0].indexB = 0
		simplex.vertices[0].wA = vertex1
		simplex.vertices[0].wB = vertex2
		simplex.vertices[0].w = Sub(vertex2, vertex1)
		simplex.vertices[0].a = 0.0
	}

	backup := Simplex{}

	simplexIndex: i32 = 0
	if simplexes != nil && simplexIndex < simplexCapacity {
		simplexes[simplexIndex] = simplex
		simplexIndex += 1
	}

	distanceOutput := DistanceOutput{}

	// Keep track of squared distance
	distanceSq := max(f32)

	normal := Vec3_zero

	// Run GJK
	iteration: i32 = 0
	for ; iteration < MAX_GJK_ITERATIONS; iteration += 1 {
		// Solve simplex
		solved := false
		switch simplex.count {
		case 1:
			simplex.vertices[0].a = 1.0
			solved = true

		case 2:
			solved = SolveSimplex2(&simplex)

		case 3:
			solved = SolveSimplex3(&simplex)

		case 4:
			solved = SolveSimplex4(&simplex)

		case:
			assert(false, "Should never get here!")
		}

		if solved == false {
			// No progress - reconstruct last simplex
			assert(backup.count != 0)
			simplex = backup
			break
		}

		if simplexes != nil && simplexIndex < simplexCapacity {
			simplexes[simplexIndex] = simplex
			simplexIndex += 1
			distanceOutput.iterations = iteration
			distanceOutput.simplexCount = simplexIndex
		}

		if simplex.count == MAX_SIMPLEX_VERTICES {
			// Overlap
			localPointA, localPointB: Vec3
			ComputeWitnessPoints(&simplex, &localPointA, &localPointB)
			distanceOutput.pointA = localPointA
			distanceOutput.pointB = localPointB
			distanceOutput.normal = Vec3_zero
			distanceOutput.distance = 0.0
			distanceOutput.iterations = iteration
			distanceOutput.simplexCount = simplexIndex
			return distanceOutput
		}

		// Assure distance progression
		oldDistanceSq := distanceSq

		// Compute closest point
		closestPoint := Vec3{}

		switch simplex.count {
		case 1:
			closestPoint = vs[0].w

		case 2:
			closestPoint = Blend2(vs[0].a, vs[0].w, vs[1].a, vs[1].w)

		case 3:
			closestPoint = Blend3(vs[0].a, vs[0].w, vs[1].a, vs[1].w, vs[2].a, vs[2].w)

		case 4:
			closestPoint = Add(Blend2(vs[0].a, vs[0].w, vs[1].a, vs[1].w), Blend2(vs[2].a, vs[2].w, vs[3].a, vs[3].w))

		case:
			assert(false, "Should never get here!")
		}

		distanceSq = Dot(closestPoint, closestPoint)

		if distanceSq >= oldDistanceSq {
			// No progress - reconstruct last simplex
			assert(backup.count != 0)
			simplex = backup
			break
		}

		// Build new tentative support point
		searchDirection := Vec3{}

		switch simplex.count {
		case 1:
			// v = -A
			searchDirection = Neg(vs[0].w)

		case 2:
			// v = (AB x AO) x AB
			a := vs[0].w
			b := vs[1].w

			ab := Sub(b, a)

			searchDirection = Cross(Cross(ab, Neg(a)), ab)

		case 3:
			// v = AB x AC or v = AC x AB
			a := vs[0].w
			b := vs[1].w
			c := vs[2].w

			ab := Sub(b, a)
			ac := Sub(c, a)

			n := Cross(ab, ac)

			searchDirection = Dot(n, a) < 0.0 ? n : Neg(n)

		case:
			assert(false, "Should never get here!")
		}

		if LengthSquared(searchDirection) < 1000.0 * FLT_MIN {
			// The origin is probably contained by a line segment or triangle.
			// Thus the shapes are overlapped.
			localPointA, localPointB: Vec3
			ComputeWitnessPoints(&simplex, &localPointA, &localPointB)
			distanceOutput.pointA = localPointA
			distanceOutput.pointB = localPointB
			distanceOutput.normal = Vec3_zero
			distanceOutput.distance = 0.0
			distanceOutput.iterations = iteration
			distanceOutput.simplexCount = simplexIndex
			VALIDATE(Distance(localPointA, localPointB) < FLT_EPSILON)
			return distanceOutput
		}

		normal = Neg(searchDirection)

		// Get new support points
		searchDirection1 := searchDirection
		indexA := GetProxySupport(&input.proxyA, Neg(searchDirection1))
		supportA := input.proxyA.points[indexA]
		searchDirection2 := MulMV(mt, searchDirection)
		indexB := GetProxySupport(&input.proxyB, searchDirection2)
		supportB := Add(MulMV(m, input.proxyB.points[indexB]), xf.p)

		// Save current simplex and add new vertex - this can fail if we detect cycling
		backup = simplex

		// Check for duplicate support points. This is the main termination criteria.
		duplicate := false
		for i: i32 = 0; i < simplex.count; i += 1 {
			if vs[i].indexA == indexA && vs[i].indexB == indexB {
				duplicate = true
				break
			}
		}

		if duplicate {
			break
		}

		vs[simplex.count].indexA = indexA
		vs[simplex.count].indexB = indexB
		vs[simplex.count].wA = supportA
		vs[simplex.count].wB = supportB
		vs[simplex.count].w = Sub(supportB, supportA)
		simplex.count += 1
	}

	// Build witness points.
	localPointA, localPointB: Vec3
	ComputeWitnessPoints(&simplex, &localPointA, &localPointB)

	distanceOutput.pointA = localPointA
	distanceOutput.pointB = localPointB
	distanceOutput.iterations = iteration
	distanceOutput.simplexCount = simplexIndex

	normal = Normalize(normal)
	if IsNormalized(normal) == false {
		// Treat as overlap
		distanceOutput.distance = 0.0
		distanceOutput.normal = Vec3_zero
		return distanceOutput
	}

	// Results stay in frame A
	distanceOutput.distance = Distance(localPointA, localPointB)
	distanceOutput.normal = normal

	// Apply radii if requested
	if input.useRadii {
		rA := input.proxyA.radius
		rB := input.proxyB.radius
		distanceOutput.distance = MaxFloat(0.0, distanceOutput.distance - rA - rB)

		// Keep closest points on perimeter even if overlapped, this way the points move smoothly.
		distanceOutput.pointA = Add(distanceOutput.pointA, MulSV(rA, normal))
		distanceOutput.pointB = Sub(distanceOutput.pointB, MulSV(rB, normal))
	}

	// Simplex is useful, cache it.
	WriteCache(cache, &simplex)

	return distanceOutput
}

// Separation function:
// f(t) = (c2 + t * dp2 - c1 - t * dp1 ) * n

// Root finding : f(t) - target = 0
// (c2 + t * dp2 - c1 - t * dp1 ) * n - target = 0
// (c2 - c1) * n + t * (dp2 - dp1) * n - target = 0
// t = [target - (c2 - c1) * n] / [(dp2 - dp1) * n]
// t = (target - d) / [(dp2 - dp1) * n]

ShapeCast :: proc(input: ^ShapeCastPairInput) -> CastOutput {
	// Compute tolerance
	linearSlop: f32 = LINEAR_SLOP
	totalRadius := input.proxyA.radius + input.proxyB.radius
	target := MaxFloat(linearSlop, totalRadius - linearSlop)
	tolerance := 0.25 * linearSlop

	assert(target > tolerance)

	// Prepare input for distance query
	cache := SimplexCache{}

	alpha: f32 = 0.0

	distanceInput := DistanceInput{}
	distanceInput.proxyA = input.proxyA
	distanceInput.proxyB = input.proxyB
	distanceInput.useRadii = false

	// The whole cast runs in frame A. Advance the relative pose of B in float each iteration,
	// which keeps the math near the local origin and avoids re-relativizing world poses.
	distanceInput.transform = input.transform

	delta2 := input.translationB
	distanceOutput := DistanceOutput{}
	output := CastOutput{}
	output.triangleIndex = NULL_INDEX

	iteration: i32 = 0
	maxIterations: i32 = 20

	for ; iteration < maxIterations; iteration += 1 {
		output.iterations += 1

		distanceOutput = ShapeDistance(&distanceInput, &cache, nil, 0)

		if distanceOutput.distance < target + tolerance {
			if iteration == 0 {
				if input.canEncroach && distanceOutput.distance > 2.0 * linearSlop {
					target = distanceOutput.distance - linearSlop
				} else if input.canEncroach && distanceOutput.distance > 0.0 {
					// This is for box movers and can't normally be reached by capsule
					// movers or regular shape casts.

					// Near-target start without overlap: advance by first order while
					// retaining a rest separation. Sweeps that cannot consume the free
					// gap (receding, tangential, grazing) miss.
					retained := 0.5 * linearSlop
					needed := distanceOutput.distance - retained
					approach := -Dot(delta2, distanceOutput.normal)
					if approach <= MaxFloat(needed, 0.0) {
						// Moving away, no hit.
						return output
					}

					fraction: f32 = needed > 0.0 ? needed / approach : 0.0
					if fraction >= input.maxFraction {
						// Still no hit because the max fraction is small.
						return output
					}

					output.fraction = fraction
					output.point = MulAdd(distanceOutput.pointA, input.proxyA.radius, distanceOutput.normal)
					output.normal = distanceOutput.normal
					output.hit = true
					return output
				} else {
					// Initial overlap
					output.hit = true

					// Compute a common point
					c1 := MulAdd(distanceOutput.pointA, input.proxyA.radius, distanceOutput.normal)
					c2 := MulAdd(distanceOutput.pointB, -input.proxyB.radius, distanceOutput.normal)
					output.point = Lerp(c1, c2, 0.5)
					return output
				}
			} else {
				// Logging for bad input data
				if distanceOutput.distance > 0.0 && IsNormalized(distanceOutput.normal) == false {
					for i: i32 = 0; i < input.proxyA.count; i += 1 {
						p := input.proxyA.points[i]
						Log("pointA[%d] = {%.9f, %.9f, %.9f}", i, p.x, p.y, p.z)
					}
					Log("radiusA = %.9f", input.proxyA.radius)

					for i: i32 = 0; i < input.proxyB.count; i += 1 {
						p := input.proxyB.points[i]
						Log("pointB[%d] = {%.9f, %.9f, %.9f}", i, p.x, p.y, p.z)
					}
					Log("radiusB = %.9f", input.proxyB.radius)

					{
						xf := input.transform
						Log(
							"transform = {{%.9f, %.9f, %.9f}, {{%.9f, %.9f, %.9f}, %.9f}",
							xf.p.x,
							xf.p.y,
							xf.p.z,
							xf.q.v.x,
							xf.q.v.y,
							xf.q.v.z,
							xf.q.s,
						)
					}

					{
						t := input.translationB
						Log("t = {%.9f, %.9f, %.9f}", t.x, t.y, t.z)
					}

					Log("maxFraction = %.9f, canEncroach = %d", input.maxFraction, input.canEncroach ? i32(1) : i32(0))

					// Numerical problem. Likely extreme input.
					return output
				}

				// Hitting this assert implies that the algorithm brought the shapes too close.
				// B3_ASSERT( distanceOutput.distance > 0.0f && b3IsNormalized( distanceOutput.normal ) );

				output.fraction = alpha
				output.point = MulAdd(distanceOutput.pointA, input.proxyA.radius, distanceOutput.normal)
				output.normal = distanceOutput.normal
				output.hit = true
				return output
			}
		}

		assert(distanceOutput.distance > 0.0)
		assert(IsNormalized(distanceOutput.normal))

		// Check if shapes are approaching each other
		denominator := Dot(delta2, distanceOutput.normal)
		if denominator >= 0.0 {
			// Miss
			return output
		}

		// Advance sweep
		alpha += (target - distanceOutput.distance) / denominator
		if alpha >= input.maxFraction {
			// Success!
			return output
		}

		distanceInput.transform.p = MulAdd(input.transform.p, alpha, delta2)
	}

	// Failure!
	return output
}

GetSweepTransform :: proc(sweep: ^Sweep, time: f32) -> Transform {
	transform: Transform
	transform.q = NLerp(sweep.q1, sweep.q2, time)
	transform.p = Sub(Lerp(sweep.c1, sweep.c2, time), RotateVector(transform.q, sweep.localCenter))
	return transform
}

@(private = "file")
GetFinalSweepTransform :: proc(sweep: ^Sweep) -> Transform {
	transform: Transform
	transform.q = sweep.q2
	transform.p = Sub(sweep.c2, RotateVector(transform.q, sweep.localCenter))
	return transform
}

@(private = "file")
UniqueCount :: proc(vertexCount: i32, vertices: ^[3]i32) -> i32 {
	assert(1 <= vertexCount && vertexCount <= 3)

	switch vertexCount {
	case 1:
		return 1

	case 2:
		return vertices[0] != vertices[1] ? 2 : 1

	case 3:
		if vertices[0] != vertices[1] && vertices[0] != vertices[2] && vertices[1] != vertices[2] {
			// All different
			return 3
		}

		if vertices[0] == vertices[1] && vertices[0] == vertices[2] && vertices[1] == vertices[2] {
			// All equal
			return 1
		}

		return 2

	case:
		assert(false, "Should never get here!")
	}

	return 0
}

// This checks if the cross product of two edges switches direction.
@(private = "file")
CheckFastEdges :: proc(xfA: Transform, localEdgeA: Vec3, xfB: Transform, localEdgeB: Vec3, axis0: Vec3) -> bool {
	// By taking the local witness axes we make sure that we
	// get the correct orientations (e.g. if one axis was flipped)!
	edgeA := RotateVector(xfA.q, localEdgeA)
	edgeB := RotateVector(xfB.q, localEdgeB)
	axis := Cross(edgeA, edgeB)
	return Dot(axis, axis0) < 0.0
}

@(private = "file")
SeparationType :: enum i32 {
	separationUnknown = 0,
	separationVertices,
	separationEdges,
	separationFaceA,
	separationFaceB,
}

@(private = "file")
SeparationFunction :: struct {
	proxyA:   ^ShapeProxy,
	proxyB:   ^ShapeProxy,
	sweepA:   Sweep,
	sweepB:   Sweep,

	// These are associated with different bodies depending on the separation function type.
	// It could be two local vectors/points on the same body (for example, both on bodyA).
	witness1: Vec3,
	witness2: Vec3,
	type:     SeparationType,
}

@(private = "file")
MakeSeparationFunction :: proc(
	cache: SimplexCache,
	proxyA: ^ShapeProxy,
	sweepA: ^Sweep,
	proxyB: ^ShapeProxy,
	sweepB: ^Sweep,
	worldNormal: Vec3,
	t1: f32,
) -> SeparationFunction {
	assert(1 <= cache.count && cache.count <= 3)
	VALIDATE(IsNormalized(worldNormal))

	fcn := SeparationFunction{}
	fcn.proxyA = proxyA
	fcn.proxyB = proxyB
	fcn.sweepA = sweepA^
	fcn.sweepB = sweepB^
	fcn.type = .separationUnknown

	indexA := [3]i32{i32(cache.indexA[0]), i32(cache.indexA[1]), i32(cache.indexA[2])}
	indexB := [3]i32{i32(cache.indexB[0]), i32(cache.indexB[1]), i32(cache.indexB[2])}

	uniqueCountA := UniqueCount(i32(cache.count), &indexA)
	uniqueCountB := UniqueCount(i32(cache.count), &indexB)

	xfA1 := GetSweepTransform(sweepA, t1)
	xfB1 := GetSweepTransform(sweepB, t1)

	qA := xfA1.q
	qB := xfB1.q

	// Minimize round-off
	deltaP := Sub(xfB1.p, xfA1.p)

	switch cache.count {
	case 1:
		// Witness is the world space direction
		fcn.type = .separationVertices
		fcn.witness1 = worldNormal

	case 2:
		if uniqueCountA == 2 && uniqueCountB == 2 {
			// Edge/Edge
			vA1 := proxyA.points[indexA[0]]
			localEdgeA := Sub(proxyA.points[indexA[1]], vA1)
			localEdgeA = Normalize(localEdgeA)
			edgeA := RotateVector(qA, localEdgeA)

			vB1 := proxyB.points[indexB[0]]
			localEdgeB := Sub(proxyB.points[indexB[1]], vB1)
			localEdgeB = Normalize(localEdgeB)
			edgeB := RotateVector(qB, localEdgeB)

			axis := Cross(edgeA, edgeB)
			lengthSquared := LengthSquared(axis)

			// Skip near parallel edges: |e1 x e1| = sin(alpha) * |e1| * |e2|
			kToleranceSquared: f32 = f32(0.05) * f32(0.05)
			if lengthSquared < kToleranceSquared {
				// The axis is not safe to normalize so we use a world axis instead!
				fcn.type = .separationVertices
				fcn.witness1 = worldNormal
			} else {
				delta := Add(Sub(RotateVector(qB, vB1), RotateVector(qA, vA1)), deltaP)
				if Dot(delta, axis) < 0.0 {
					// Make axis point from A to B
					axis = Neg(axis)
					localEdgeB = Neg(localEdgeB)
				}

				// Check for possible sign flip in edge/edge cross product
				xfA2 := GetFinalSweepTransform(sweepA)
				xfB2 := GetFinalSweepTransform(sweepB)
				fastEdges := CheckFastEdges(xfA2, localEdgeA, xfB2, localEdgeB, axis)
				if fastEdges == true {
					// Not safe to use local edges, fall back to initial world space axis instead
					fcn.type = .separationVertices
					fcn.witness1 = Normalize(axis)
				} else {
					// Edge cross product is safe. This converges faster than a fixed axis.
					fcn.type = .separationEdges
					fcn.witness1 = localEdgeA
					fcn.witness2 = localEdgeB
				}
			}
		} else {
			VALIDATE(IsNormalized(worldNormal))

			// Vertex versus edge, use world axis witness
			fcn.type = .separationVertices
			fcn.witness1 = worldNormal
		}

	case 3:
		if uniqueCountA == 3 {
			vA1 := proxyA.points[indexA[0]]
			vA2 := proxyA.points[indexA[1]]
			vA3 := proxyA.points[indexA[2]]
			localAxisA := Cross(Sub(vA2, vA1), Sub(vA3, vA1))
			localAxisA = Normalize(localAxisA)
			axisA := RotateVector(qA, localAxisA)

			localPointA := MulSV(1.0 / 3.0, Add(Add(vA1, vA2), vA3))
			localPointB := proxyB.points[indexB[0]]
			delta := Add(Sub(RotateVector(qB, localPointB), RotateVector(qA, localPointA)), deltaP)

			if Dot(delta, axisA) < 0.0 {
				// Make axis point from A to B
				localAxisA = Neg(localAxisA)
			}

			// Witness is the local plane of faceA
			fcn.type = .separationFaceA
			fcn.witness1 = localAxisA
			fcn.witness2 = localPointA
		} else if uniqueCountB == 3 {
			vB1 := proxyB.points[indexB[0]]
			vB2 := proxyB.points[indexB[1]]
			vB3 := proxyB.points[indexB[2]]
			localAxisB := Cross(Sub(vB2, vB1), Sub(vB3, vB1))
			localAxisB = Normalize(localAxisB)
			axisB := RotateVector(qB, localAxisB)

			localPointA := proxyA.points[indexA[0]]
			localPointB := MulSV(1.0 / 3.0, Add(Add(vB1, vB2), vB3))
			delta := Sub(Sub(RotateVector(qA, localPointA), RotateVector(qB, localPointB)), deltaP)

			if Dot(delta, axisB) < 0.0 {
				// Make axis point from B to A
				localAxisB = Neg(localAxisB)
			}

			// Witness is the local plane of faceB
			fcn.type = .separationFaceB
			fcn.witness1 = localAxisB
			fcn.witness2 = localPointB
		} else {
			assert(uniqueCountA == 2 && uniqueCountB == 2)

			if indexA[0] == indexA[1] {
				// Make first two indices are unique
				indexA[1] = indexA[2]
				assert(indexA[0] != indexA[1])
			}

			vA1 := proxyA.points[indexA[0]]
			vA2 := proxyA.points[indexA[1]]
			localEdgeA := Normalize(Sub(vA2, vA1))
			edgeA := RotateVector(qA, localEdgeA)

			if indexB[0] == indexB[1] {
				// Make first two indices are unique
				indexB[1] = indexB[2]
				assert(indexB[0] != indexB[1])
			}

			vB1 := proxyB.points[indexB[0]]
			vB2 := proxyB.points[indexB[1]]
			localEdgeB := Normalize(Sub(vB2, vB1))
			edgeB := RotateVector(qB, localEdgeB)

			axis := Cross(edgeA, edgeB)
			lengthSquared := LengthSquared(axis)

			// Skip near parallel edges: |e1 x e1| = sin(alpha) * |e1| * |e2|
			kToleranceSquared: f32 = f32(0.005) * f32(0.005)
			if lengthSquared < kToleranceSquared {
				// The axis is not safe to normalize so we use a world axis instead!
				fcn.type = .separationVertices
				fcn.witness1 = worldNormal
			} else {
				delta := Add(Sub(RotateVector(qB, vB1), RotateVector(qA, vA1)), deltaP)
				if Dot(delta, axis) < 0.0 {
					// Make axis point from A to B
					axis = Neg(axis)
					localEdgeB = Neg(localEdgeB)
				}

				// Check for possible sign flip in edge/edge cross product
				xfA2 := GetFinalSweepTransform(sweepA)
				xfB2 := GetFinalSweepTransform(sweepB)
				fastEdges := CheckFastEdges(xfA2, localEdgeA, xfB2, localEdgeB, axis)
				if fastEdges {
					// Not safe to use local edges, fall back to initial world space axis instead
					fcn.type = .separationVertices
					fcn.witness1 = Normalize(axis)
				} else {
					// Edge cross product is safe. This converges faster than a fixed axis.
					fcn.type = .separationEdges
					fcn.witness1 = localEdgeA
					fcn.witness2 = localEdgeB
				}
			}
		}

	case:
		assert(false, "Should never get here!")
	}

	return fcn
}

@(private = "file")
FindMinSeparation :: proc(fcn: ^SeparationFunction, indexA: ^i32, indexB: ^i32, t: f32) -> f32 {
	xfA := GetSweepTransform(&fcn.sweepA, t)
	xfB := GetSweepTransform(&fcn.sweepB, t)

	#partial switch fcn.type {
	case .separationVertices:
		axis := fcn.witness1

		localAxisA := InvRotateVector(xfA.q, axis)
		localAxisB := InvRotateVector(xfB.q, Neg(axis))

		indexA^ = GetPointSupport(fcn.proxyA.points, fcn.proxyA.count, localAxisA)
		indexB^ = GetPointSupport(fcn.proxyB.points, fcn.proxyB.count, localAxisB)

		deltaP := Sub(xfB.p, xfA.p)
		localPointA := fcn.proxyA.points[indexA^]
		localPointB := fcn.proxyB.points[indexB^]
		delta := Add(Sub(RotateVector(xfB.q, localPointB), RotateVector(xfA.q, localPointA)), deltaP)
		return Dot(delta, axis)

	case .separationEdges:
		edgeA := RotateVector(xfA.q, fcn.witness1)
		edgeB := RotateVector(xfB.q, fcn.witness2)
		axis := Cross(edgeA, edgeB)
		assert(axis.x != 0.0 || axis.y != 0.0 || axis.z != 0.0)
		axis = Normalize(axis)

		axisA := InvRotateVector(xfA.q, axis)
		indexA^ = GetPointSupport(fcn.proxyA.points, fcn.proxyA.count, axisA)

		axisB := InvRotateVector(xfB.q, axis)
		indexB^ = GetPointSupport(fcn.proxyB.points, fcn.proxyB.count, Neg(axisB))

		deltaP := Sub(xfB.p, xfA.p)
		localPointA := fcn.proxyA.points[indexA^]
		localPointB := fcn.proxyB.points[indexB^]
		delta := Add(Sub(RotateVector(xfB.q, localPointB), RotateVector(xfA.q, localPointA)), deltaP)

		return Dot(delta, axis)

	case .separationFaceA:
		normal := RotateVector(xfA.q, fcn.witness1)
		indexA^ = -1
		pointA := TransformPoint(xfA, fcn.witness2)

		axisB := InvRotateVector(xfB.q, normal)
		indexB^ = GetPointSupport(fcn.proxyB.points, fcn.proxyB.count, Neg(axisB))
		pointB := TransformPoint(xfB, fcn.proxyB.points[indexB^])

		return Dot(Sub(pointB, pointA), normal)

	case .separationFaceB:
		normal := RotateVector(xfB.q, fcn.witness1)

		axisA := InvRotateVector(xfA.q, normal)
		indexA^ = GetPointSupport(fcn.proxyA.points, fcn.proxyA.count, Neg(axisA))
		pointA := TransformPoint(xfA, fcn.proxyA.points[indexA^])

		indexB^ = -1
		pointB := TransformPoint(xfB, fcn.witness2)

		return Dot(Sub(pointA, pointB), normal)

	case:
		assert(false, "Should never get here!")
	}

	return 0.0
}

@(private = "file")
EvaluateSeparation :: proc(fcn: ^SeparationFunction, index1: i32, index2: i32, beta: f32) -> f32 {
	transform1 := GetSweepTransform(&fcn.sweepA, beta)
	transform2 := GetSweepTransform(&fcn.sweepB, beta)

	#partial switch fcn.type {
	case .separationVertices:
		axis := fcn.witness1

		point1 := TransformPoint(transform1, fcn.proxyA.points[index1])
		point2 := TransformPoint(transform2, fcn.proxyB.points[index2])

		return Dot(Sub(point2, point1), axis)

	case .separationEdges:
		edge1 := RotateVector(transform1.q, fcn.witness1)
		edge2 := RotateVector(transform2.q, fcn.witness2)
		axis := Cross(edge1, edge2)
		axis = Normalize(axis)

		point1 := TransformPoint(transform1, fcn.proxyA.points[index1])
		point2 := TransformPoint(transform2, fcn.proxyB.points[index2])

		return Dot(Sub(point2, point1), axis)

	case .separationFaceA:
		axis := RotateVector(transform1.q, fcn.witness1)

		point1 := TransformPoint(transform1, fcn.witness2)
		point2 := TransformPoint(transform2, fcn.proxyB.points[index2])

		return Dot(Sub(point2, point1), axis)

	case .separationFaceB:
		axis := RotateVector(transform2.q, fcn.witness1)

		point1 := TransformPoint(transform1, fcn.proxyA.points[index1])
		point2 := TransformPoint(transform2, fcn.witness2)

		return Dot(Sub(point1, point2), axis)

	case:
		assert(false, "Should never get here!")
	}

	return 0.0
}

@(private = "file")
ForceFixedAxis :: proc(fcn: ^SeparationFunction, beta: f32) {
	assert(fcn.type == .separationEdges)

	transform1 := GetSweepTransform(&fcn.sweepA, beta)
	transform2 := GetSweepTransform(&fcn.sweepB, beta)

	edge1 := RotateVector(transform1.q, fcn.witness1)
	edge2 := RotateVector(transform2.q, fcn.witness2)
	axis := Cross(edge1, edge2)
	axis = Normalize(axis)

	fcn.type = .separationVertices
	fcn.witness1 = axis
	fcn.witness2 = Vec3_zero
}

// Time of Impact using root finding
TimeOfImpact :: proc(input: ^TOIInput) -> TOIOutput {
	output := TOIOutput{}

	// Set these to invalid values so they can be validated on exit
	output.state = .toiStateUnknown
	output.fraction = -1.0

	sweepA := input.sweepA
	sweepB := input.sweepB

	// Shift to origin
	origin := sweepA.c1
	sweepA.c1 = Vec3_zero
	sweepA.c2 = Sub(sweepA.c2, origin)
	sweepB.c1 = Sub(sweepB.c1, origin)
	sweepB.c2 = Sub(sweepB.c2, origin)

	proxyA := input.proxyA
	proxyB := input.proxyB

	maxPushBackIterations := proxyA.count + proxyB.count
	tMax := input.maxFraction

	// Setup target distance and tolerance
	linearSlop: f32 = LINEAR_SLOP
	totalRadius := proxyA.radius + proxyB.radius
	target := MaxFloat(linearSlop, totalRadius - linearSlop)
	tolerance := 0.25 * linearSlop
	assert(target > tolerance)

	t1: f32 = 0.0
	maxIterations: i32 = 25
	distanceIterations: i32 = 0

	// Prepare input for distance query.
	cache := SimplexCache{}
	distanceInput := DistanceInput{}
	distanceInput.proxyA = proxyA
	distanceInput.proxyB = proxyB
	distanceInput.useRadii = false

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	for {
		// Get the distance between shapes. We can also use the results to get a separating axis.
		xfA := GetSweepTransform(&sweepA, t1)
		xfB := GetSweepTransform(&sweepB, t1)
		distanceInput.transform = InvMulTransforms(xfA, xfB)
		distanceOutput := ShapeDistance(&distanceInput, &cache, nil, 0)
		output.distance = distanceOutput.distance

		// The distance query runs in frame A, project the witness data back to the shifted world
		worldNormal := RotateVector(xfA.q, distanceOutput.normal)
		worldPointA := TransformPoint(xfA, distanceOutput.pointA)
		worldPointB := TransformPoint(xfA, distanceOutput.pointB)

		output.distanceIterations += 1
		distanceIterations += 1

		// If the shapes are overlapped, we give up on continuous collision.
		if distanceOutput.distance <= 0.0 {
			output.state = .toiStateOverlapped
			output.fraction = 0.0

			// Averaged hit point
			pA := MulAdd(worldPointA, proxyA.radius, worldNormal)
			pB := MulAdd(worldPointB, -proxyB.radius, worldNormal)
			output.point = Lerp(pA, pB, 0.5)
			output.point = Add(output.point, origin)
			output.normal = worldNormal

			break
		}

		if distanceOutput.distance <= target + tolerance {
			// Success!
			output.state = .toiStateHit

			// Averaged hit point
			pA := MulAdd(worldPointA, proxyA.radius, worldNormal)
			pB := MulAdd(worldPointB, -proxyB.radius, worldNormal)
			output.point = Lerp(pA, pB, 0.5)
			output.point = Add(output.point, origin)
			output.normal = worldNormal
			output.fraction = t1
			break
		}

		if distanceIterations == maxIterations {
			// Progress too slow. This can happen when a capsule rotates around a
			// triangle vertex.
			output.state = .toiStateFailed
			output.fraction = t1

			// Averaged hit point
			pA := MulAdd(worldPointA, input.proxyA.radius, worldNormal)
			pB := MulAdd(worldPointB, -input.proxyB.radius, worldNormal)
			output.point = Lerp(pA, pB, 0.5)
			output.point = Add(output.point, origin)
			output.normal = worldNormal
			break
		}

		// Initialize the separating axis.
		function := MakeSeparationFunction(cache, &proxyA, &sweepA, &proxyB, &sweepB, worldNormal, t1)

		// unified: C has `#if B3_ENABLE_VALIDATION && 0` here (disabled upstream)
		when false {
			// todo this can give a negative value for diagonal edge contact on faces, typical GJK problem
			// to fix this I think the separation function would need to identify faces
			{
				index1, index2: i32
				minSeparation := FindMinSeparation(&function, &index1, &index2, t1)
				// SAT should give a closer result than GJK
				VALIDATE(
					minSeparation > target - tolerance && minSeparation - distanceOutput.distance < 0.1 * LINEAR_SLOP,
				)
			}
		}
		// unified end

		// Compute the TOI on the separating axis. We do this by successively resolving the deepest point.
		done := false
		t2 := tMax
		pushBackIterations: i32 = 0
		for {
			indexA, indexB: i32
			s2 := FindMinSeparation(&function, &indexA, &indexB, t2)

			// Dump the function seen by the root finder
			// 			for ( int Step = 0; Step <= 100; ++Step )
			// 				{
			// 				float Alpha = 0.01f * Step;
			// 				float Separation = Function.Evaluate( Index1, Index2, Alpha );
			//
			// 				b3Report( "s(%4.2g) = %g\n", Alpha, Separation );
			// 				}

			// Is the final configuration separated?
			if s2 - target > tolerance {
				// Success!
				output.state = .toiStateSeparated
				output.fraction = input.maxFraction
				done = true
				break
			}

			// Has the separation reached tolerance?
			if s2 >= target - tolerance {
				// Advance the sweeps
				t1 = t2
				break
			}

			// Compute the initial separation of the witness points
			s1 := EvaluateSeparation(&function, indexA, indexB, t1)

			// Check for overlap. This might happen if the root finder runs out of iterations.
			if s1 < target - tolerance {
				// Failed!
				VALIDATE(false)
				output.state = .toiStateFailed
				output.fraction = t1
				done = true
				break
			}

			// Has the separation reached tolerance?
			if s1 <= target + tolerance {
				// Success! t1 should hold the TOI (could be 0.0)
				output.state = .toiStateHit
				output.fraction = t1
				done = true
				break
			}

			// Compute 1D root of: f(x) - target = 0
			rootIterationCount: i32 = 0
			maxRootIterations: i32 = 50
			a1 := t1
			a2 := t2
			for {
				// Use a mix of false position and bisection.
				t: f32
				if rootIterationCount & 1 != 0 {
					// False position to improve convergence.
					t = a1 + (target - s1) * (a2 - a1) / (s2 - s1)
				} else {
					// Bisection to guarantee progress.
					t = 0.5 * (a1 + a2)
				}

				output.rootIterations += 1
				rootIterationCount += 1

				s := EvaluateSeparation(&function, indexA, indexB, t)

				// Has the separation reached tolerance?
				if AbsFloat(s - target) <= tolerance {
					// t2 holds a tentative value for t1
					t2 = t
					break
				}

				// Ensure we continue to bracket the root.
				if s > target {
					a1 = t
					s1 = s
				} else {
					a2 = t
					s2 = s
				}

				if rootIterationCount == maxRootIterations {
					VALIDATE(false)
					break
				}
			}

			// Restart the inner loop if we have a failing edge case.
			if rootIterationCount == maxRootIterations - 1 && function.type == .separationEdges {
				VALIDATE(false)

				rootIterationCount = 0
				t2 = input.maxFraction
				ForceFixedAxis(&function, t1)
				assert(function.type != .separationEdges)
			}

			output.pushBackIterations += 1
			pushBackIterations += 1

			if pushBackIterations == maxPushBackIterations {
				break
			}
		}

		if done {
			// Averaged hit point
			pA := MulAdd(worldPointA, input.proxyA.radius, worldNormal)
			pB := MulAdd(worldPointB, -input.proxyB.radius, worldNormal)
			output.point = Lerp(pA, pB, 0.5)
			output.point = Add(output.point, origin)
			output.normal = worldNormal
			break
		}
	}

	// It is expected that the state and fraction are set before reaching this
	assert(output.state != .toiStateUnknown)
	assert(output.fraction >= 0.0)

	return output
}
