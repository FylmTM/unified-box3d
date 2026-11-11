// Ported from box3d src/triangle_manifold.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:intrinsics"

@(private = "file")
TriangleData :: struct {
	v1, v2, v3: Vec3,
	e1, e2, e3: Vec3,
	plane:      Plane,
	flags:      i32,
}

// Indexed by the 3-bit vertex mask
@(rodata, private = "file")
s_triangleFeatures := [8]TriangleFeature {
	.featureNone, // 000  (unreachable)
	.featureVertex1, // 001
	.featureVertex2, // 010
	.featureEdge1, // 011  v1,v2
	.featureVertex3, // 100
	.featureEdge3, // 101  v1,v3
	.featureEdge2, // 110  v2,v3
	.featureTriangleFace, // 111
}

@(private = "file")
GetTriangleFeature :: proc(cache: ^SimplexCache) -> TriangleFeature {
	count := i32(cache.count)
	assert(0 < count && count < 4)

	// Bit i set means triangle vertex i participates in the simplex.
	mask: i32 = 0
	for i: i32 = 0; i < count; i += 1 {
		assert(cache.indexA[i] < 3)
		mask |= i32(1) << cache.indexA[i]
	}

	return s_triangleFeatures[mask]
}

CollideTriangleAndSphere :: proc(manifold: ^LocalManifold, capacity: i32, triangleA: [^]Vec3, sphereB: ^Sphere) {
	manifold.pointCount = 0

	if capacity == 0 {
		return
	}

	center := sphereB.center
	v1, v2, v3 := triangleA[0], triangleA[1], triangleA[2]
	plane := MakePlaneFromPoints(v1, v2, v3)

	offset := PlaneSeparation(plane, center)
	if offset < 0.0 {
		// Cull back side collision
		return
	}

	radius := sphereB.radius

	// Closest point on triangle to sphere center
	closest := ClosestPointOnTriangle(v1, v2, v3, center)

	// Test separating axis
	squaredDistance := DistanceSquared(closest.point, center)
	speculativeDistance: f32 = SPECULATIVE_DISTANCE
	maxDistance := radius + speculativeDistance
	if squaredDistance > maxDistance * maxDistance {
		return
	}

	distance := intrinsics.sqrt(squaredDistance)

	// Normal points from triangle to sphere
	normal: Vec3
	if distance * distance > 1000.0 * FLT_MIN {
		normal = MulSV(1.0 / distance, Sub(center, closest.point))
	} else {
		normal = Normalize(Cross(Sub(v2, v1), Sub(v3, v1)))
	}

	// contact point mid-way
	// p = 0.5 * (c + q) - 0.5 * r * n
	contactPoint := MulSV(0.5, Add(Sub(center, MulSV(radius, normal)), closest.point))

	manifold.normal = normal
	manifold.pointCount = 1
	manifold.feature = closest.feature
	manifold.squaredDistance = squaredDistance

	mp := &manifold.points[0]
	mp.point = contactPoint
	mp.separation = distance - radius
	mp.pair = FeaturePair_single
}

@(private = "file")
ClipSegmentToTriangleFace :: proc(segment: [^]ClipVertex, points: [^]Vec3, plane: Plane) -> bool {
	vertex1 := points[2]
	for i: i32 = 0; i < 3; i += 1 {
		vertex2 := points[i]
		tangent := Normalize(Sub(vertex2, vertex1))
		binormal := Cross(tangent, plane.normal)

		clipPlane := MakePlaneFromNormalAndPoint(binormal, vertex1)

		vertexCount: i32 = 0
		p1 := segment[0]
		p2 := segment[1]

		distance1 := PlaneSeparation(clipPlane, p1.position)
		distance2 := PlaneSeparation(clipPlane, p2.position)

		// If the points are behind the plane
		if distance1 <= 0.0 {
			segment[vertexCount] = p1
			vertexCount += 1
		}
		if distance2 <= 0.0 {
			segment[vertexCount] = p2
			vertexCount += 1
		}

		// If the points are on different sides of the plane
		if (distance1 > 0.0) != (distance2 > 0.0) {
			// Find intersection point of edge and plane
			t := distance1 / (distance1 - distance2)
			segment[vertexCount].position = Lerp(p1.position, p2.position, t)
			segment[vertexCount].pair = distance1 > 0.0 ? p1.pair : p2.pair
			vertexCount += 1
		}

		if vertexCount != 2 {
			return false
		}

		vertex1 = vertex2
	}

	return true
}

@(private = "file")
QueryTriangleFaceAndCapsule :: proc(plane: Plane, capsule: ^Capsule) -> SeparatingAxis {
	separation1 := PlaneSeparation(plane, capsule.center1)
	separation2 := PlaneSeparation(plane, capsule.center2)

	if separation1 < separation2 {
		return SeparatingAxis{normal = plane.normal, separation = separation1, indexA = 0, indexB = 0}
	}

	return SeparatingAxis{normal = plane.normal, separation = separation2, indexA = 0, indexB = 1}
}

@(private = "file")
QueryTriangleAndCapsuleEdges :: proc(vertices: [^]Vec3, plane: Plane, capsule: ^Capsule) -> SeparatingAxis {
	// Work in the local space of the capsule
	p1 := capsule.center1
	p2 := capsule.center2
	capsuleEdge := Sub(p2, p1)

	// Find axis of minimum penetration
	maxNormal := Vec3_zero
	maxSeparation := -max(f32)
	maxIndex1: i32 = NULL_INDEX
	maxIndex2: i32 = NULL_INDEX
	squaredTolerance: f32 = f32(0.005) * f32(0.005)

	edgeIndex: i32 = 2
	a := Dot(capsuleEdge, plane.normal)
	v1 := vertices[2]
	for index: i32 = 0; index < 3; index += 1 {
		v2 := vertices[index]
		triangleEdge := Sub(v2, v1)
		sideNormal := Normalize(Cross(triangleEdge, plane.normal))

		// Pretend the triangle edge embeds a zero area face with a side normal. This
		// provides a way to find an edge-edge normal that points outward from
		// the triangle.
		b := Dot(capsuleEdge, sideNormal)

		// Is the capsule edge parallel to the triangle edge? If so, face contact can handle it.
		if a * a + b * b < squaredTolerance * LengthSquared(capsuleEdge) {
			v1 = v2
			edgeIndex = index
			continue
		}

		// Similar to hull vs hull (b3QueryEdgeDirections)
		axis: Vec3
		if a * b <= 0.0 {
			t := b / (b - a)
			axis = Lerp(sideNormal, plane.normal, t)
		} else {
			t := b / (a + b)
			axis = Lerp(sideNormal, Neg(plane.normal), t)
		}

		VALIDATE(LengthSquared(axis) > 1000.0 * FLT_MIN)
		axis = Normalize(axis)
		separation := Dot(axis, Sub(p1, v1))

		if separation > maxSeparation {
			// Note: We don't exit early if we find a separating axis here since we want to
			// find the best one for caching and account for the convex radius later.
			maxNormal = axis
			maxSeparation = separation
			maxIndex1 = edgeIndex
			maxIndex2 = 0
		}

		v1 = v2
		edgeIndex = index
	}

	// Save result
	return SeparatingAxis{normal = maxNormal, separation = maxSeparation, indexA = maxIndex1, indexB = maxIndex2}
}

@(private = "file")
BuildTriangleAndCapsuleFaceContact :: proc(
	manifold: ^LocalManifold,
	triangle: [^]Vec3,
	plane: Plane,
	capsule: ^Capsule,
) {
	assert(manifold.pointCount == 0)

	segment: [2]ClipVertex = ---
	segment[0].position = capsule.center1
	segment[0].separation = 0.0
	segment[0].pair = MakeFeaturePair(.featureShapeA, 0, .featureShapeA, 0)
	segment[1].position = capsule.center2
	segment[1].separation = 0.0
	segment[1].pair = MakeFeaturePair(.featureShapeA, 1, .featureShapeA, 1)

	havePoints := ClipSegmentToTriangleFace(&segment[0], triangle, plane)
	if havePoints == false {
		return
	}

	radius := capsule.radius
	distance1 := PlaneSeparation(plane, segment[0].position)
	distance2 := PlaneSeparation(plane, segment[1].position)

	speculativeDistance: f32 = SPECULATIVE_DISTANCE
	if distance1 > speculativeDistance + radius && distance2 > speculativeDistance + radius {
		return
	}

	// Average points. Half-way between capsule bottom and triangle plane.
	point1 := MulSub(segment[0].position, 0.5 * (distance1 + capsule.radius), plane.normal)
	point2 := MulSub(segment[1].position, 0.5 * (distance2 + capsule.radius), plane.normal)

	manifold.normal = plane.normal
	manifold.feature = .featureTriangleFace
	manifold.pointCount = 2

	pt := &manifold.points[0]
	pt.point = point1
	pt.separation = distance1 - capsule.radius
	pt.pair = segment[0].pair

	pt = &manifold.points[1]
	pt.point = point2
	pt.separation = distance2 - capsule.radius
	pt.pair = segment[1].pair
}

@(private = "file")
BuildTriangleAndCapsuleEdgeContact :: proc(
	manifold: ^LocalManifold,
	triangle: [^]Vec3,
	plane: Plane,
	capsule: ^Capsule,
	query: SeparatingAxis,
) {
	assert(0 <= query.indexA && query.indexA < 3)

	p1 := capsule.center1
	p2 := capsule.center2
	capsuleEdge := Sub(p2, p1)

	vs := triangle

	v1 := vs[query.indexA]
	v2 := vs[(query.indexA + 1) % 3]
	triangleEdge := Sub(v2, v1)

	sideNormal := Normalize(Cross(triangleEdge, plane.normal))

	// Pretend the triangle edge embeds a zero area face with a side normal. This
	// provides a way to find an edge-edge normal that points outward from
	// the triangle.
	a := Dot(capsuleEdge, plane.normal)
	b := Dot(capsuleEdge, sideNormal)

	// Is the capsule edge parallel to the triangle edge? If so, face contact can handle it.
	squaredTolerance: f32 = f32(0.005) * f32(0.005)
	if a * a + b * b < squaredTolerance * LengthSquared(capsuleEdge) {
		return
	}

	// Similar to hull vs hull (b3QueryEdgeDirections)
	normal := query.normal
	result := LineDistance(v1, triangleEdge, p1, capsuleEdge)

	if result.fraction1 < 0.0 || 1.0 < result.fraction1 || result.fraction2 < 0.0 || 1.0 < result.fraction2 {
		// closest point beyond end points
		return
	}

	point := Lerp(result.point1, MulSub(result.point2, capsule.radius, normal), 0.5)
	separation := Dot(normal, Sub(p1, v1))

	manifold.normal = normal
	manifold.pointCount = 1

	edgesFeatures := [3]TriangleFeature{.featureEdge1, .featureEdge2, .featureEdge3}
	manifold.feature = edgesFeatures[query.indexA]

	pt := &manifold.points[0]
	pt.point = point
	pt.separation = separation - capsule.radius
	pt.pair = MakeFeaturePair(.featureShapeA, query.indexA, .featureShapeB, query.indexB)
}

CollideTriangleAndCapsule :: proc(
	manifold: ^LocalManifold,
	capacity: i32,
	triangleA: [^]Vec3,
	capsuleB: ^Capsule,
	cache: ^SimplexCache,
) {
	manifold.pointCount = 0

	if capacity < 2 {
		return
	}

	v1, v2, v3 := triangleA[0], triangleA[1], triangleA[2]
	plane := MakePlaneFromPoints(v1, v2, v3)
	capsuleCenter := Lerp(capsuleB.center1, capsuleB.center2, 0.5)

	offset := PlaneSeparation(plane, capsuleCenter)
	if offset < 0.0 {
		// Cull back side collision
		return
	}

	distanceInput: DistanceInput
	distanceInput.proxyA = ShapeProxy{triangleA, 3, 0.0}
	distanceInput.proxyB = ShapeProxy{&capsuleB.center1, 2, 0.0}
	distanceInput.transform = Transform_identity
	distanceInput.useRadii = false

	distanceOutput := ShapeDistance(&distanceInput, cache, nil, 0)
	speculativeDistance: f32 = SPECULATIVE_DISTANCE
	radius := capsuleB.radius
	if distanceOutput.distance > radius + speculativeDistance {
		// Shapes are separated, persist the cache
		return
	}

	if distanceOutput.distance > 100.0 * FLT_EPSILON {
		// Shallow penetration
		delta := Normalize(Sub(distanceOutput.pointB, distanceOutput.pointA))

		// Try to create two contact points if closest points difference is nearly parallel to face normal
		kTolerance: f32 : 0.2
		cosAngle := AbsFloat(Dot(plane.normal, delta))
		if cosAngle > kTolerance {
			// Clip capsule segment against side planes of reference face
			segment: [2]ClipVertex = ---
			segment[0].position = capsuleB.center1
			segment[0].separation = 0.0
			segment[0].pair = MakeFeaturePair(.featureShapeA, 0, .featureShapeA, 0)
			segment[1].position = capsuleB.center2
			segment[1].separation = 0.0
			segment[1].pair = MakeFeaturePair(.featureShapeA, 1, .featureShapeA, 1)

			havePoints := ClipSegmentToTriangleFace(&segment[0], triangleA, plane)

			if havePoints == true {
				distance1 := PlaneSeparation(plane, segment[0].position)
				distance2 := PlaneSeparation(plane, segment[1].position)

				normal := plane.normal
				point1 := MulSub(segment[0].position, 0.5 * (radius + distance1), normal)
				point2 := MulSub(segment[1].position, 0.5 * (radius + distance2), normal)

				// Normal points from triangle to capsule
				manifold.normal = normal
				manifold.feature = .featureTriangleFace
				manifold.pointCount = 2

				mp := &manifold.points[0]
				mp.point = point1
				mp.separation = distance1 - radius
				mp.pair = segment[0].pair

				mp = &manifold.points[1]
				mp.point = point2
				mp.separation = distance2 - radius
				mp.pair = segment[1].pair

				return
			}
		}

		// Create contact from closest points.
		point := Lerp(distanceOutput.pointA, MulSub(distanceOutput.pointB, radius, delta), 0.5)

		// Normal points from triangle to capsule.
		manifold.normal = delta
		manifold.pointCount = 1
		manifold.feature = GetTriangleFeature(cache)

		mp := &manifold.points[0]
		mp.point = point
		mp.separation = distanceOutput.distance - radius
		mp.pair = FeaturePair_single

		return
	}

	// Deep penetration

	faceQuery := QueryTriangleFaceAndCapsule(plane, capsuleB)
	if faceQuery.separation > radius {
		// Shapes are separated. Should be impossible for a reasonable capsule radius.
		return
	}

	edgeQuery := QueryTriangleAndCapsuleEdges(triangleA, plane, capsuleB)
	if edgeQuery.separation > radius {
		// Shapes are separated. Should be impossible for a reasonable capsule radius.
		return
	}

	// Create face contact
	faceSeparation := faceQuery.separation - radius
	BuildTriangleAndCapsuleFaceContact(manifold, triangleA, plane, capsuleB)
	VALIDATE(manifold.pointCount == 0 || manifold.pointCount == 2)
	if manifold.pointCount == 2 {
		// This becomes the clipped separation.
		faceSeparation = MinFloat(manifold.points[0].separation, manifold.points[1].separation)
	}

	if edgeQuery.indexA == NULL_INDEX {
		// No valid edge contact.
		return
	}

	// Face contact can be empty if it does not realize the axis of minimum penetration.
	// Create edge contact if face contact fails or edge contact is significantly better!
	linearSlop: f32 = LINEAR_SLOP
	edgeSeparation := edgeQuery.separation - radius
	if manifold.pointCount == 0 || edgeSeparation > faceSeparation + linearSlop {
		// Edge contact
		BuildTriangleAndCapsuleEdgeContact(manifold, triangleA, plane, capsuleB, edgeQuery)
	}
}

@(private = "file")
GetTriangleSupport :: proc(points: [^]Vec3, direction: Vec3) -> i32 {
	index: i32 = 0
	distance := Dot(points[0], direction)

	d := Dot(points[1], direction)
	if d > distance {
		distance = d
		index = 1
	}

	d = Dot(points[2], direction)
	if d > distance {
		return 2
	}

	return index
}

@(private = "file")
QueryTriangleFace :: proc(triangle: ^TriangleData, hull: ^HullData) -> SeparatingAxis {
	hullPoints := GetHullPoints(hull)
	plane := triangle.plane
	normal := Neg(plane.normal)
	vertexIndex := FindHullSupportVertex(hull, normal)
	support := hullPoints[vertexIndex]
	separation := PlaneSeparation(plane, support)

	return SeparatingAxis {
		normal = plane.normal,
		separation = separation,
		indexA = 0,
		indexB = vertexIndex,
		type = .faceAxisA,
	}
}

@(private = "file")
QueryHullFace :: proc(triangle: ^TriangleData, hull: ^HullData) -> SeparatingAxis {
	hullPlanes := GetHullPlanes(hull)
	faceCount := hull.faceCount

	trianglePoints := [3]Vec3{triangle.v1, triangle.v2, triangle.v3}

	maxNormal := Vec3_zero
	// unified: -INFINITY as an exact f32 bit pattern (no core:math)
	maxFaceSeparation := -f32(0h7F800000)
	// unified end
	maxFaceIndex: i32 = NULL_INDEX
	maxVertexIndex: i32 = NULL_INDEX

	for faceIndex: i32 = 0; faceIndex < faceCount; faceIndex += 1 {
		plane := hullPlanes[faceIndex]

		vertexIndex := GetTriangleSupport(&trianglePoints[0], Neg(plane.normal))
		support := trianglePoints[vertexIndex]
		separation := PlaneSeparation(plane, support)
		if separation > maxFaceSeparation {
			maxNormal = plane.normal
			maxFaceSeparation = separation
			maxFaceIndex = faceIndex
			maxVertexIndex = vertexIndex
		}
	}

	// Normal points from triangle to hull
	return SeparatingAxis {
		normal = Neg(maxNormal),
		separation = maxFaceSeparation,
		indexA = maxVertexIndex,
		indexB = maxFaceIndex,
		type = .faceAxisB,
	}
}

// A: hull, B: triangle
@(private = "file")
QueryTriangleAndHullEdges :: proc(triangle: ^TriangleData, hull: ^HullData) -> SeparatingAxis {
	result := SeparatingAxis {
		normal     = Vec3_zero,
		// unified: -INFINITY as an exact f32 bit pattern (no core:math)
		separation = -f32(0h7F800000),
		// unified end
		indexA     = NULL_INDEX,
		indexB     = NULL_INDEX,
		type       = .edgePairAxis,
	}

	trianglePoints := [3]Vec3{triangle.v1, triangle.v2, triangle.v3}
	triangleEdges := [3]Vec3{triangle.e1, triangle.e2, triangle.e3}
	// int edgeFlags[] = { b3_concaveEdge1, b3_concaveEdge1, b3_concaveEdge3 };

	when FORCE_GHOST_COLLISIONS {
		triangleFlags: i32 = 0xFF
	} else {
		triangleFlags := triangle.flags
	}
	_ = triangleFlags

	triNormal := triangle.plane.normal

	hullEdges := GetHullEdges(hull)
	hullPoints := GetHullPoints(hull)
	hullPlanes := GetHullPlanes(hull)
	edgeCount := hull.edgeCount
	squaredTolerance: f32 = f32(0.005) * f32(0.005)

	for i: i32 = 0; i < edgeCount; i += 2 {
		edge := &hullEdges[i]
		twin := &hullEdges[i + 1]
		assert(i32(edge.twin) == i + 1 && i32(twin.twin) == i)

		hullPoint := hullPoints[edge.origin]
		hullEdge := Sub(hullPoints[twin.origin], hullPoint)

		hullNormal1 := hullPlanes[edge.face].normal
		hullNormal2 := hullPlanes[twin.face].normal

		for j: i32 = 0; j < 3; j += 1 {
			triEdge := triangleEdges[j]

			cab := Dot(hullNormal1, triEdge)
			dab := Dot(hullNormal2, triEdge)
			bcd := Dot(triNormal, hullEdge)
			if cab * dab >= 0.0 || cab * bcd <= 0.0 {
				continue
			}

			// Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
			if MaxFloat(cab * cab, dab * dab) < squaredTolerance * LengthSquared(triEdge) {
				continue
			}

			// Similar to hull vs hull (b3QueryEdgeDirections)
			// dot(hullNormal1 + t * (hullNormal2 - hullNormal1), triEdge) = 0
			// Normal points out of hull by construction.
			t := cab / (cab - dab)
			axis := Lerp(hullNormal1, hullNormal2, t)
			VALIDATE(LengthSquared(axis) > 1000.0 * FLT_MIN)
			axis = Normalize(axis)
			separation := Dot(axis, Sub(trianglePoints[j], hullPoint))

			// if ( separation > result.separation && ( edgeFlags[j] & triangleFlags ) == 0 )
			if separation > result.separation {
				// Note: We don't exit early if we find a separating axis here since we want to
				// find the best one for caching.
				// Flip normal to point from triangle to hull.
				result.normal = Neg(axis)
				result.separation = separation
				result.indexA = j
				result.indexB = i
			}
		}
	}

	return result
}

@(private = "file")
CollideHullFace :: proc(
	manifold: ^LocalManifold,
	pointCapacity: i32,
	triangle: ^TriangleData,
	hull: ^HullData,
	query: SeparatingAxis,
	cache: ^SATCache,
	enableSpeculative: bool,
) -> f32 {
	VALIDATE(query.type == .faceAxisB)
	VALIDATE(0 <= query.indexA && query.indexA < 3)
	VALIDATE(0 <= query.indexB && query.indexB < hull.faceCount)

	manifold.pointCount = 0

	hullFaces := GetHullFaces(hull)
	hullEdges := GetHullEdges(hull)
	hullPlanes := GetHullPlanes(hull)
	hullPoints := GetHullPoints(hull)

	// Reference hull face
	refPlane := hullPlanes[query.indexB]

	// Build clip polygon from triangle face (the incident face)
	buffer1, buffer2: [MAX_CLIP_POINTS]ClipVertex = ---, ---

	v1 := triangle.v1
	v2 := triangle.v2
	v3 := triangle.v3
	buffer1[0].position = v1
	buffer1[0].separation = PlaneSeparation(refPlane, v1)
	buffer1[0].pair = MakeFeaturePair(.featureShapeB, 2, .featureShapeB, 0)
	buffer1[1].position = v2
	buffer1[1].separation = PlaneSeparation(refPlane, v2)
	buffer1[1].pair = MakeFeaturePair(.featureShapeB, 0, .featureShapeB, 1)
	buffer1[2].position = v3
	buffer1[2].separation = PlaneSeparation(refPlane, v3)
	buffer1[2].pair = MakeFeaturePair(.featureShapeB, 1, .featureShapeB, 2)
	pointCount: i32 = 3

	// Clip triangle face against side planes of reference face
	input: [^]ClipVertex = &buffer1[0]
	output: [^]ClipVertex = &buffer2[0]

	face := &hullFaces[query.indexB]
	edgeIndex := i32(face.edge)

	for {
		edge := &hullEdges[edgeIndex]
		nextEdgeIndex := i32(edge.next)
		next := &hullEdges[nextEdgeIndex]
		vertex1 := hullPoints[edge.origin]
		vertex2 := hullPoints[next.origin]
		tangent := Normalize(Sub(vertex2, vertex1))
		binormal := Cross(tangent, refPlane.normal)

		clipPlane := MakePlaneFromNormalAndPoint(binormal, vertex1)

		pointCount = ClipPolygon(output, input, pointCount, clipPlane, edgeIndex, refPlane)
		assert(pointCount <= MAX_CLIP_POINTS)

		if pointCount < 3 {
			// Using a stale cache
			cache^ = SATCache{}
			return query.separation
		}

		// Swap buffers, output becomes input for the next clipping plane
		output, input = input, output
		edgeIndex = nextEdgeIndex

		if edgeIndex == i32(face.edge) {
			break
		}
	}

	pointCount = MinInt(pointCount, pointCapacity)
	minSeparation := max(f32)
	finalPointCount: i32 = 0

	for i: i32 = 0; i < pointCount; i += 1 {
		clipPoint := &input[i]
		minSeparation = MinFloat(minSeparation, clipPoint.separation)

		if enableSpeculative == false && clipPoint.separation > 0.0 {
			continue
		}

		// Move point onto hull face improved culling
		point := MulSub(clipPoint.position, clipPoint.separation, refPlane.normal)

		pt := &manifold.points[finalPointCount]
		pt.point = point
		pt.separation = clipPoint.separation
		pt.pair = FlipPair(clipPoint.pair)

		finalPointCount += 1
	}

	speculativeDistance: f32 = enableSpeculative ? SPECULATIVE_DISTANCE : 0.0
	if minSeparation > speculativeDistance {
		// This can occur with a stale SAT cache
		manifold.pointCount = 0
		cache^ = SATCache{}
		return minSeparation
	}

	manifold.pointCount = finalPointCount
	manifold.normal = Neg(refPlane.normal)
	manifold.feature = .featureHullFace

	// Save cache
	cache.separation = minSeparation
	cache.type = u8(SeparatingFeature.faceAxisB)
	cache.indexA = u8(query.indexA)
	cache.indexB = u8(query.indexB)
	return minSeparation
}

@(private = "file")
CollideTriangleFace :: proc(
	manifold: ^LocalManifold,
	pointCapacity: i32,
	triangle: ^TriangleData,
	hull: ^HullData,
	query: SeparatingAxis,
	cache: ^SATCache,
	enableSpeculative: bool,
) -> f32 {
	VALIDATE(query.type == .faceAxisA)
	VALIDATE(query.indexA == 0)
	VALIDATE(0 <= query.indexB && query.indexB < hull.vertexCount)
	VALIDATE(manifold.pointCount == 0)

	hullFaces := GetHullFaces(hull)
	hullEdges := GetHullEdges(hull)
	hullPoints := GetHullPoints(hull)

	// Find incident face
	refPlane := triangle.plane

	incFace := FindIncidentFace(hull, refPlane.normal, query.indexB)

	// Build clip polygon from incident face
	buffer1, buffer2: [2 * MAX_CLIP_POINTS]ClipVertex = ---, ---
	pointCount: i32 = 0
	face := &hullFaces[incFace]
	hullEdgeIndex := i32(face.edge)

	for {
		edge := &hullEdges[hullEdgeIndex]

		nextEdgeIndex := i32(edge.next)
		next := &hullEdges[nextEdgeIndex]

		hullPoint := hullPoints[next.origin]
		buffer1[pointCount].position = hullPoint
		buffer1[pointCount].separation = PlaneSeparation(refPlane, hullPoint)
		buffer1[pointCount].pair = MakeFeaturePair(.featureShapeB, hullEdgeIndex, .featureShapeB, nextEdgeIndex)

		pointCount += 1
		hullEdgeIndex = nextEdgeIndex

		if !(hullEdgeIndex != i32(face.edge) && pointCount < 2 * MAX_CLIP_POINTS) {
			break
		}
	}

	assert(pointCount >= 3)

	// Clip incident face against side planes of reference face (triangle)
	input: [^]ClipVertex = &buffer1[0]
	output: [^]ClipVertex = &buffer2[0]

	trianglePoints := [3]Vec3{triangle.v1, triangle.v2, triangle.v3}
	triangleEdges := [3]Vec3{triangle.e1, triangle.e2, triangle.e3}

	for i: i32 = 0; i < 3 && pointCount > 0; i += 1 {
		sideNormal := Cross(triangleEdges[i], refPlane.normal)
		sideNormal = Normalize(sideNormal)

		clipPlane := MakePlaneFromNormalAndPoint(sideNormal, trianglePoints[i])

		pointCount = ClipPolygon(output, input, pointCount, clipPlane, i, refPlane)
		assert(pointCount <= 2 * MAX_CLIP_POINTS)

		output, input = input, output
	}

	if pointCount == 0 {
		// Triangle face clipped away. Invalidate cache.
		cache^ = SATCache{}
		return max(f32)
	}

	pointCount = MinInt(pointCount, pointCapacity)

	minSeparation := max(f32)

	finalPointCount: i32 = 0
	for i: i32 = 0; i < pointCount; i += 1 {
		clipPoint := &input[i]
		minSeparation = MinFloat(minSeparation, clipPoint.separation)

		if enableSpeculative == false && clipPoint.separation > 0.0 {
			continue
		}

		// Move point onto triangle surface for improved culling
		// b3Vec3 point = b3MulSub( clipPoint->position, clipPoint->separation, refPlane.normal );
		point := clipPoint.position

		pt := &manifold.points[finalPointCount]
		pt.point = point
		pt.separation = clipPoint.separation
		pt.pair = clipPoint.pair

		finalPointCount += 1
	}

	speculativeDistance: f32 = enableSpeculative ? SPECULATIVE_DISTANCE : 0.0
	if minSeparation >= speculativeDistance {
		// This can happens if the objects move a part while re-using a cached axis
		cache^ = SATCache{}
		return minSeparation
	}

	manifold.pointCount = finalPointCount
	manifold.normal = refPlane.normal
	manifold.feature = .featureTriangleFace

	// Save cache
	cache.separation = minSeparation
	cache.type = u8(SeparatingFeature.faceAxisA)
	cache.indexA = u8(query.indexA)
	cache.indexB = u8(query.indexB)
	return minSeparation
}

@(private = "file")
CollideTriangleAndHullEdges :: proc(
	manifold: ^LocalManifold,
	capacity: i32,
	trianglePoint: Vec3,
	triangleEdge: Vec3,
	hull: ^HullData,
	query: SeparatingAxis,
	cache: ^SATCache,
) {
	VALIDATE(query.type == .edgePairAxis)
	VALIDATE(0 <= query.indexA && query.indexA < 3)
	VALIDATE(0 <= query.indexB && query.indexB < hull.edgeCount)
	VALIDATE(query.separation <= 2.0 * SPECULATIVE_DISTANCE)

	pA := trianglePoint
	eA := triangleEdge

	edgesB := GetHullEdges(hull)
	pointsB := GetHullPoints(hull)
	edgeB := &edgesB[query.indexB]
	twinB := &edgesB[edgeB.twin]
	pB := pointsB[edgeB.origin]
	qB := pointsB[twinB.origin]
	eB := Sub(qB, pB)

	// Get the closest points between the infinite edge lines
	result := LineDistance(pA, eA, pB, eB)

	// Is one of the closest points outside of the associated edge segment?
	if capacity == 0 ||
	   result.fraction1 < 0.0 ||
	   1.0 < result.fraction1 ||
	   result.fraction2 < 0.0 ||
	   1.0 < result.fraction2 {
		// Invalid edge pair, no points generated
		assert(manifold.pointCount == 0)
		cache^ = SATCache{}
		return
	}

	separation := Dot(query.normal, Sub(pB, pA))
	point := MulSV(0.5, Add(result.point1, result.point2))

	pt := &manifold.points[0]
	pt.point = point
	pt.separation = separation
	pt.pair = MakeFeaturePair(.featureShapeA, query.indexA, .featureShapeB, query.indexB)

	// Save cache
	cache.separation = separation
	cache.type = u8(SeparatingFeature.edgePairAxis)
	cache.indexA = u8(query.indexA)
	cache.indexB = u8(query.indexB)

	manifold.normal = query.normal
	manifold.pointCount = 1

	edgesFeatures := [3]TriangleFeature{.featureEdge1, .featureEdge2, .featureEdge3}
	manifold.feature = edgesFeatures[query.indexA]
}

triangleConvexCalls: AtomicInt
triangleCacheHits: AtomicInt

// Triangle is in the local space of the hull for efficiency.
CollideTriangleAndHull :: proc(
	manifold: ^LocalManifold,
	capacity: i32,
	v1, v2, v3: Vec3,
	triangleFlags: i32,
	hullB: ^HullData,
	cache: ^SATCache,
	enableSpeculative: bool,
) {
	manifold.pointCount = 0
	manifold.feature = .featureNone

	if capacity < 4 {
		return
	}

	trianglePlane := MakePlaneFromPoints(v1, v2, v3)
	linearSlop: f32 = LINEAR_SLOP

	offset := PlaneSeparation(trianglePlane, hullB.center)
	if cache.type == u8(SeparatingFeature.backsideAxis) {
		// Use hysteresis to avoid jitter on wavy meshes
		if AbsFloat(cache.separation - offset) < linearSlop {
			return
		}

		cache.type = u8(SeparatingFeature.invalidAxis)
	}

	if offset < -linearSlop {
		// Cull back side collision. Cache offset to add hysteresis.
		cache.type = u8(SeparatingFeature.backsideAxis)
		cache.separation = offset
		return
	}

	trianglePoints := [3]Vec3{v1, v2, v3}
	triangleEdges := [3]Vec3{Sub(v2, v1), Sub(v3, v2), Sub(v1, v3)}

	triangle := TriangleData {
		v1    = v1,
		v2    = v2,
		v3    = v3,
		e1    = triangleEdges[0],
		e2    = triangleEdges[1],
		e3    = triangleEdges[2],
		plane = trianglePlane,
		flags = triangleFlags,
	}

	edges := GetHullEdges(hullB)
	hullPlanes := GetHullPlanes(hullB)
	hullPoints := GetHullPoints(hullB)

	speculativeDistance: f32 = enableSpeculative ? SPECULATIVE_DISTANCE : 0.0
	cache.hit = 1

	// Attempt to use the cache to speed up collision
	#partial switch SeparatingFeature(cache.type) {
	case .faceAxisA:
		assert(cache.indexA == 0)

		vertexIndex := FindHullSupportVertex(hullB, Neg(trianglePlane.normal))
		support := hullPoints[vertexIndex]
		separation := PlaneSeparation(trianglePlane, support)
		if separation > speculativeDistance {
			// Cache hit, shapes are separated
			return
		}

		faceQuery: SeparatingAxis
		faceQuery.normal = trianglePlane.normal
		faceQuery.separation = separation
		faceQuery.indexA = i32(cache.indexA)
		faceQuery.indexB = vertexIndex
		faceQuery.type = .faceAxisA

		// Read cache but don't modify it
		localCache := cache^
		clippedSeparation := CollideTriangleFace(
			manifold,
			capacity,
			&triangle,
			hullB,
			faceQuery,
			&localCache,
			enableSpeculative,
		)

		if manifold.pointCount > 0 && AbsFloat(cache.separation - clippedSeparation) < linearSlop {
			// Cache hit, contact points generated
			return
		}

		// Invalidate cache and fall through
		manifold.pointCount = 0
		cache^ = SATCache{}

	case .faceAxisB:
		assert(i32(cache.indexB) < hullB.faceCount)

		plane := hullPlanes[cache.indexB]

		// Get triangle support point
		vertexIndex: i32 = 0
		distance := -Dot(v1, plane.normal)
		for i: i32 = 1; i < 3; i += 1 {
			d := -Dot(trianglePoints[i], plane.normal)
			if d > distance {
				distance = d
				vertexIndex = i
			}
		}

		support := trianglePoints[vertexIndex]

		// Separation of triangle support point with hull plane
		separation := PlaneSeparation(plane, support)
		if separation > speculativeDistance {
			// Cache hit, shapes are separated
			return
		}

		// Deep overlap may lead to an invalid cache
		// todo confirm
		isDeep := separation < -2.0 * linearSlop

		// Don't persist deep cache or allow separation to change too much
		if isDeep == false {
			//  Try to rebuild contact from last features
			faceQuery: SeparatingAxis
			faceQuery.normal = Neg(plane.normal)
			faceQuery.separation = separation
			faceQuery.indexA = vertexIndex
			faceQuery.indexB = i32(cache.indexB)
			faceQuery.type = .faceAxisB

			// Read cache but don't modify it
			localCache := cache^
			clippedSeparation := CollideHullFace(
				manifold,
				capacity,
				&triangle,
				hullB,
				faceQuery,
				&localCache,
				enableSpeculative,
			)

			// Cache reuse is only successful if it creates contact points and the clipped separation didn't change much.
			if manifold.pointCount > 0 && AbsFloat(cache.separation - clippedSeparation) < linearSlop {
				// Cache hit, contact points generated
				return
			}
		}

		// Invalidate cache and fall through
		manifold.pointCount = 0
		cache^ = SATCache{}

	case .edgePairAxis:
		assert(cache.indexA < 3)
		indexA := i32(cache.indexA)

		triPoint := trianglePoints[indexA]
		triEdge := triangleEdges[indexA]

		assert(i32(cache.indexB) < hullB.edgeCount - 1)
		indexB := i32(cache.indexB)

		edge2 := &edges[indexB]
		twin2 := &edges[indexB + 1]
		assert(i32(edge2.twin) == indexB + 1 && i32(twin2.twin) == indexB)

		hullPoint := hullPoints[edge2.origin]
		hullEdge := Sub(hullPoints[twin2.origin], hullPoint)
		hullNormal1 := hullPlanes[edge2.face].normal
		hullNormal2 := hullPlanes[twin2.face].normal

		// Confirm the edge pair is still a Minkowski face.
		// See "Collision Detection of Convex Polyhedra Based on Duality Transformation"
		// Simplified for triangle versus hull.
		cab := Dot(hullNormal1, triEdge)
		dab := Dot(hullNormal2, triEdge)
		bcd := Dot(trianglePlane.normal, hullEdge)

		if cab * dab < 0.0 && cab * bcd > 0.0 {
			squaredTolerance: f32 = f32(0.005) * f32(0.005)

			// Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
			if MaxFloat(cab * cab, dab * dab) >= squaredTolerance * LengthSquared(triEdge) {
				// Similar to hull vs hull (b3QueryEdgeDirections)
				// dot(hullNormal1 + t * (hullNormal2 - hullNormal1), triEdge) = 0
				// Normal points out of hull by construction.
				t := cab / (cab - dab)
				axis := Lerp(hullNormal1, hullNormal2, t)
				VALIDATE(LengthSquared(axis) > 1000.0 * FLT_MIN)
				axis = Normalize(axis)
				separation := Dot(axis, Sub(triPoint, hullPoint))
				if separation > speculativeDistance {
					// Cache hit, shapes are separated
					return
				}

				if AbsFloat(cache.separation - separation) < linearSlop {
					// Try to rebuild contact from last features
					// Flip normal to point from triangle to hull
					edgeQuery: SeparatingAxis
					edgeQuery.normal = Neg(axis)
					edgeQuery.indexA = indexA
					edgeQuery.indexB = indexB
					edgeQuery.separation = separation
					edgeQuery.type = .edgePairAxis

					// Read cache but don't modify it
					localCache := cache^
					CollideTriangleAndHullEdges(manifold, capacity, triPoint, triEdge, hullB, edgeQuery, &localCache)

					if manifold.pointCount > 0 {
						// Cache hit, contact point generated
						return
					}
				}
			}
		}

		// Invalidate cache and fall through
		cache^ = SATCache{}

	// This case is for testing
	case .manualFaceAxisA:
		query := QueryTriangleFace(&triangle, hullB)
		CollideTriangleFace(manifold, capacity, &triangle, hullB, query, cache, enableSpeculative)
		return

	// This case is for testing
	case .manualFaceAxisB:
		query := QueryHullFace(&triangle, hullB)
		CollideHullFace(manifold, capacity, &triangle, hullB, query, cache, enableSpeculative)
		return

	// This case is for testing
	case .manualEdgePairAxis:
		query := QueryTriangleAndHullEdges(&triangle, hullB)
		if query.indexA != NULL_INDEX {
			trianglePoint := trianglePoints[query.indexA]
			triangleEdge := triangleEdges[query.indexA]
			CollideTriangleAndHullEdges(manifold, capacity, trianglePoint, triangleEdge, hullB, query, cache)
		}
		return

	case:
		assert(cache.type == u8(SeparatingFeature.invalidAxis))
	}

	// Cache miss
	cache.hit = 0

	// Find axis of minimum penetration
	faceQueryA := QueryTriangleFace(&triangle, hullB)
	if faceQueryA.separation > speculativeDistance {
		// Separating axis found
		cache.separation = faceQueryA.separation
		cache.type = u8(SeparatingFeature.faceAxisA)
		cache.indexA = u8(faceQueryA.indexA)
		cache.indexB = u8(faceQueryA.indexB)
		return
	}

	faceQueryB := QueryHullFace(&triangle, hullB)
	if faceQueryB.separation > speculativeDistance {
		// Separating axis found
		cache.separation = faceQueryB.separation
		cache.type = u8(SeparatingFeature.faceAxisB)
		cache.indexA = u8(faceQueryB.indexA)
		cache.indexB = u8(faceQueryB.indexB)
		return
	}

	edgeQuery := QueryTriangleAndHullEdges(&triangle, hullB)
	if edgeQuery.separation > speculativeDistance {
		// Separating axis found
		cache.separation = edgeQuery.separation
		cache.type = u8(SeparatingFeature.edgePairAxis)
		cache.indexA = u8(edgeQuery.indexA)
		cache.indexB = u8(edgeQuery.indexB)
		return
	}

	clipSeparation: f32

	// Don't admit a hull face significantly opposed to the triangle face.
	// Need a tolerance to avoid ghost collisions.
	pushingDown := Dot(faceQueryB.normal, trianglePlane.normal) < -0.25
	if faceQueryB.separation >= faceQueryA.separation && pushingDown == false {
		clipSeparation = CollideHullFace(manifold, capacity, &triangle, hullB, faceQueryB, cache, enableSpeculative)
	} else {
		clipSeparation = CollideTriangleFace(
			manifold,
			capacity,
			&triangle,
			hullB,
			faceQueryA,
			cache,
			enableSpeculative,
		)
	}

	// Does an edge axis exist?
	if edgeQuery.indexA != NULL_INDEX {
		// When axes are aligned the edge separation can be garbage.
		// If a face axis has positive separation there may be no points.
		maxFaceSeparation := MaxFloat(faceQueryA.separation, faceQueryB.separation)

		if (manifold.pointCount == 0 && edgeQuery.separation > maxFaceSeparation) ||
		   (manifold.pointCount == 1 && edgeQuery.separation > clipSeparation + linearSlop) {
			assert(0 <= edgeQuery.indexA && edgeQuery.indexA < 3)
			trianglePoint := trianglePoints[edgeQuery.indexA]
			triangleEdge := triangleEdges[edgeQuery.indexA]
			manifold.pointCount = 0
			CollideTriangleAndHullEdges(manifold, capacity, trianglePoint, triangleEdge, hullB, edgeQuery, cache)
		}
	}

	// Using the speculative distance means that sometimes there are no valid contact points from SAT.
	// In this fall back to GJK. This is important to prevent tunneling in rare cases.
	if manifold.pointCount == 0 {
		triangleB := [3]Vec3{v1, v2, v3}
		input := DistanceInput{}
		input.proxyA = ShapeProxy {
			points = &triangleB[0],
			count  = 3,
			radius = 0.0,
		}
		input.proxyB = ShapeProxy {
			points = hullPoints,
			count  = hullB.vertexCount,
			radius = 0.0,
		}
		input.transform = Transform_identity
		input.useRadii = false

		simplexCache := SimplexCache{}
		output := ShapeDistance(&input, &simplexCache, nil, 0)

		if output.distance > 0.0 {
			assert(0 < simplexCache.count && simplexCache.count <= 3)

			manifold.pointCount = 1
			manifold.feature = GetTriangleFeature(&simplexCache)
			manifold.normal = output.normal
			manifold.points[0].point = output.pointB
			manifold.points[0].separation = output.distance

			// This feature pair not accurate but maybe it doesn't matter
			manifold.points[0].pair = FeaturePair_single
		}

		// No way to cache this scenario
		cache^ = SATCache{}
	}
}
