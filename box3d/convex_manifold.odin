// Ported from box3d src/convex_manifold.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:intrinsics"

@(private = "file")
ClipSegment :: proc(segment: [^]ClipVertex, plane: Plane) -> i32 {
	vertexCount: i32 = 0
	vertex1 := segment[0]
	vertex2 := segment[1]

	distance1 := PlaneSeparation(plane, vertex1.position)
	distance2 := PlaneSeparation(plane, vertex2.position)

	// If the points are behind the plane
	if distance1 <= 0.0 {
		segment[vertexCount] = vertex1
		vertexCount += 1
	}
	if distance2 <= 0.0 {
		segment[vertexCount] = vertex2
		vertexCount += 1
	}

	// If the points are on different sides of the plane
	if distance1 * distance2 < 0.0 {
		// Find intersection point of edge and plane
		t := distance1 / (distance1 - distance2)
		segment[vertexCount].position = Add(MulSV(1.0 - t, vertex1.position), MulSV(t, vertex2.position))
		segment[vertexCount].pair = distance1 > 0.0 ? vertex1.pair : vertex2.pair
		vertexCount += 1
	}

	return vertexCount
}

@(private = "file")
ClipSegmentToHullFace :: proc(segment: [^]ClipVertex, hull: ^HullData, refFace: i32) -> i32 {
	faces := GetHullFaces(hull)
	planes := GetHullPlanes(hull)
	edges := GetHullEdges(hull)
	points := GetHullPoints(hull)

	refPlane := planes[refFace]

	face := &faces[refFace]

	edgeIndex := i32(face.edge)

	for {
		edge := &edges[edgeIndex]
		nextEdgeIndex := i32(edge.next)
		next := &edges[nextEdgeIndex]

		vertex1 := points[edge.origin]
		vertex2 := points[next.origin]
		tangent := Normalize(Sub(vertex2, vertex1))
		binormal := Cross(tangent, refPlane.normal)

		pointCount := ClipSegment(segment, MakePlaneFromNormalAndPoint(binormal, vertex1))
		if pointCount < 2 {
			return 0
		}

		edgeIndex = nextEdgeIndex

		if edgeIndex == i32(face.edge) {
			break
		}
	}

	return 2
}

@(private = "file")
QueryFaceDirectionHullAndCapsule :: proc(
	hull: ^HullData,
	capsule: ^Capsule,
	capsuleTransform: Transform,
) -> SeparatingAxis {
	maxFaceIndex: i32 = -1
	maxVertexIndex: i32 = -1
	maxFaceSeparation := -max(f32)
	planes := GetHullPlanes(hull)

	capsulePoints := [2]Vec3 {
		TransformPoint(capsuleTransform, capsule.center1),
		TransformPoint(capsuleTransform, capsule.center2),
	}

	for faceIndex: i32 = 0; faceIndex < hull.faceCount; faceIndex += 1 {
		plane := planes[faceIndex]

		vertexIndex := GetPointSupport(&capsulePoints[0], 2, Neg(plane.normal))
		support := capsulePoints[vertexIndex]
		separation := PlaneSeparation(plane, support)
		if separation > maxFaceSeparation {
			maxVertexIndex = vertexIndex
			maxFaceIndex = faceIndex
			maxFaceSeparation = separation
		}
	}

	return SeparatingAxis {
		normal = planes[maxFaceIndex].normal,
		separation = maxFaceSeparation,
		indexA = i32(u8(maxFaceIndex)),
		indexB = i32(u8(maxVertexIndex)),
	}
}

@(private = "file")
QueryEdgeDirectionHullAndCapsule :: proc(
	hull: ^HullData,
	capsule: ^Capsule,
	capsuleTransform: Transform,
) -> SeparatingAxis {
	// Find axis of minimum penetration
	maxNormal := Vec3_zero
	maxSeparation := -max(f32)
	maxIndexA: i32 = NULL_INDEX
	maxIndexB: i32 = NULL_INDEX

	// We perform all computations in local space of the hull
	pA := TransformPoint(capsuleTransform, capsule.center1)
	qA := TransformPoint(capsuleTransform, capsule.center2)
	eA := Sub(qA, pA)

	edges := GetHullEdges(hull)
	points := GetHullPoints(hull)
	planes := GetHullPlanes(hull)
	squaredTolerance: f32 = PARALLEL_EDGE_TOL * PARALLEL_EDGE_TOL

	for index: i32 = 0; index < hull.edgeCount; index += 2 {
		edge := &edges[index]
		twin := &edges[index + 1]
		assert(i32(edge.twin) == index + 1 && i32(twin.twin) == index)

		qB := points[twin.origin]
		uB := planes[edge.face].normal
		vB := planes[twin.face].normal

		// An isolated edge (e.g. like in a capsule) defines a circle through the
		// origin on the Gauss map. So testing for overlap between this circle and
		// the arc AB simplifies to a plane test.
		cba := Dot(uB, eA)
		dba := Dot(vB, eA)

		if cba * dba < 0.0 {
			// Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
			if MaxFloat(cba * cba, dba * dba) < squaredTolerance * LengthSquared(eA) {
				continue
			}

			// The intersection of the arcs on the Gauss map is the edge pair axis. Cast the
			// arc of hull B (from uB to vB) against the plane containing the arc of hull A:
			// dot(uB + t * (vB - uB), eA) == 0
			// then
			// t = cba / (cba - dba)
			//
			// The signs of cba and dba differ (Minkowski test), so the division is safe.
			//
			// The axis generated points from B to A by construction since it lands between
			// two face normals on B. This removes the need to orient the separation axis
			// using the hull centers.
			//
			// The axis is perpendicular to both edges so I can use qA and qB as arbitrary
			// points on edgeA and edgeB to measure the separation.

			t := cba / (cba - dba)
			axis := Lerp(uB, vB, t)
			VALIDATE(LengthSquared(axis) > 1000.0 * FLT_MIN)
			axis = Normalize(axis)
			separation := Dot(axis, Sub(qA, qB))

			if separation > maxSeparation {
				// Note: We don't exit early if we find a separating axis here since we want to
				// find the best one for caching and account for the convex radius later.
				maxNormal = axis
				maxSeparation = separation
				maxIndexA = 0
				maxIndexB = index
			}
		}
	}

	// Save result
	return SeparatingAxis{normal = maxNormal, separation = maxSeparation, indexA = maxIndexA, indexB = maxIndexB}
}

CollideSpheres :: proc(
	manifold: ^LocalManifold,
	capacity: i32,
	sphereA: ^Sphere,
	sphereB: ^Sphere,
	transformBtoA: Transform,
) {
	if capacity == 0 {
		return
	}

	// Work in shapeB coordinates
	center1 := sphereA.center
	center2 := TransformPoint(transformBtoA, sphereB.center)

	totalRadius := sphereA.radius + sphereB.radius
	maxDistance := totalRadius + SPECULATIVE_DISTANCE
	offset := Sub(center2, center1)
	distanceSq := LengthSquared(offset)

	if distanceSq > maxDistance * maxDistance {
		// We found a separating axis
		return
	}

	normal := Vec3{0.0, 1.0, 0.0}
	distance := intrinsics.sqrt(distanceSq)
	if distance * distance > 1000.0 * FLT_MIN {
		normal = MulSV(1.0 / distance, offset)
	}

	// Contact at the midpoint
	// 0.5 * ( ((c1 + rA*n) + c2) - rB*n )
	point := MulSV(0.5, MulSub(Add(MulAdd(center1, sphereA.radius, normal), center2), sphereB.radius, normal))

	// Manifold in frame B
	manifold.normal = normal
	manifold.pointCount = 1

	pt := &manifold.points[0]
	pt.point = point
	pt.separation = distance - totalRadius
	pt.pair = FeaturePair_single
}

CollideCapsuleAndSphere :: proc(
	manifold: ^LocalManifold,
	capacity: i32,
	capsuleA: ^Capsule,
	sphereB: ^Sphere,
	transformBtoA: Transform,
) {
	manifold.pointCount = 0

	if capacity < 1 {
		return
	}

	// Work in shape B coordinates
	center := TransformPoint(transformBtoA, sphereB.center)
	center1 := capsuleA.center1
	center2 := capsuleA.center2

	totalRadius := sphereB.radius + capsuleA.radius
	maxDistance := totalRadius + SPECULATIVE_DISTANCE

	closestPoint := PointToSegmentDistance(center1, center2, center)
	offset := Sub(center, closestPoint)
	distanceSq := LengthSquared(offset)

	if distanceSq > maxDistance * maxDistance {
		// We found a separating axis
		return
	}

	normal := Vec3{0.0, 1.0, 0.0}
	distance := intrinsics.sqrt(distanceSq)
	if distance * distance > 1000.0 * FLT_MIN {
		normal = MulSV(1.0 / distance, offset)
	}

	// Contact at the midpoint
	// 0.5 * (((center - sB*n) + closestPoint) + cA*n)
	point := MulSV(0.5, MulAdd(Add(MulSub(center, sphereB.radius, normal), closestPoint), capsuleA.radius, normal))

	// Manifold in frame B
	manifold.normal = normal
	manifold.pointCount = 1

	pt := &manifold.points[0]
	pt.point = point
	pt.separation = distance - totalRadius
	pt.pair = FeaturePair_single
}

CollideHullAndSphere :: proc(
	manifold: ^LocalManifold,
	capacity: i32,
	hullA: ^HullData,
	sphereB: ^Sphere,
	transformBtoA: Transform,
	cache: ^SimplexCache,
) {
	manifold.pointCount = 0

	if capacity == 0 {
		return
	}

	center := TransformPoint(transformBtoA, sphereB.center)

	speculativeDistance: f32 = SPECULATIVE_DISTANCE

	// Work in shapeA coordinates

	distanceInput: DistanceInput
	distanceInput.proxyA = ShapeProxy{GetHullPoints(hullA), hullA.vertexCount, 0.0}
	distanceInput.proxyB = ShapeProxy{&center, 1, 0.0}
	distanceInput.transform = Transform_identity
	distanceInput.useRadii = false

	radiusA: f32 = 0.0
	radiusB := sphereB.radius
	radius := radiusA + radiusB

	distanceOutput := ShapeDistance(&distanceInput, cache, nil, 0)

	if distanceOutput.distance > radius + speculativeDistance {
		// We found a separating axis
		cache^ = SimplexCache{}
		return
	}

	if distanceOutput.distance > 100.0 * FLT_EPSILON {
		// Shallow penetration
		normal := Normalize(Sub(distanceOutput.pointB, distanceOutput.pointA))

		// cA is the projection of the sphere center onto to the hull (pointA if radiusA == 0).
		cA := MulAdd(center, radiusA - Dot(Sub(center, distanceOutput.pointA), normal), normal)

		// cB is the deepest point on the sphere with respect to the reference f
		cB := MulSub(center, radiusB, normal)

		point := Lerp(cA, cB, 0.5)

		// Manifold in frame A
		manifold.normal = normal
		manifold.pointCount = 1

		pt := &manifold.points[0]
		pt.point = point
		pt.separation = distanceOutput.distance - radius
		pt.pair = FeaturePair_single
	} else {
		// Deep penetration
		bestIndex: i32 = -1
		bestDistance := -max(f32)
		planes := GetHullPlanes(hullA)

		for index: i32 = 0; index < hullA.faceCount; index += 1 {
			plane := planes[index]

			distance := PlaneSeparation(plane, center)
			if distance > bestDistance {
				bestIndex = index
				bestDistance = distance
			}
		}
		assert(bestIndex >= 0)

		normal := planes[bestIndex].normal

		// cA is the projection of the sphere center onto to the hull
		cA := MulAdd(center, radiusA - Dot(Sub(center, distanceOutput.pointA), normal), normal)

		// cB is the deepest point on the sphere with respect to the reference f
		cB := MulSub(center, radiusB, normal)

		point := Lerp(cA, cB, 0.5)

		// Manifold in frame A
		manifold.normal = normal
		manifold.pointCount = 1

		pt := &manifold.points[0]
		pt.point = point
		pt.separation = bestDistance - radius
		pt.pair = FeaturePair_single
	}
}

CollideCapsules :: proc(
	manifold: ^LocalManifold,
	capacity: i32,
	capsuleA: ^Capsule,
	capsuleB: ^Capsule,
	transformBtoA: Transform,
) {
	manifold.pointCount = 0

	if capacity < 2 {
		return
	}

	// Work in shapeA coordinates
	centerA1 := capsuleA.center1
	centerA2 := capsuleA.center2
	centerB1 := TransformPoint(transformBtoA, capsuleB.center1)
	centerB2 := TransformPoint(transformBtoA, capsuleB.center2)

	radius := capsuleA.radius + capsuleB.radius
	maxDistance := radius + SPECULATIVE_DISTANCE

	result := SegmentDistance(centerA1, centerA2, centerB1, centerB2)
	offset := Sub(result.point2, result.point1)
	distanceSquared := LengthSquared(offset)
	linearSlop: f32 = LINEAR_SLOP
	minDistance := 0.01 * linearSlop

	if distanceSquared > maxDistance * maxDistance || distanceSquared < minDistance * minDistance {
		// We found a separating axis
		return
	}

	lengthA: f32
	segmentA := Sub(centerA2, centerA1)
	edgeA := GetLengthAndNormalize(&lengthA, segmentA)
	if lengthA < MIN_CAPSULE_LENGTH {
		return
	}

	lengthB: f32
	segmentB := Sub(centerB2, centerB1)
	edgeB := GetLengthAndNormalize(&lengthB, segmentB)
	if lengthB < MIN_CAPSULE_LENGTH {
		return
	}

	// Parallel edges: |eA x eB| = sin(alpha)
	alphaTol: f32 : 0.05
	alphaTolSqr: f32 : alphaTol * alphaTol
	axis := Cross(edgeA, edgeB)

	// Try to create two contact points if the capsules are nearly parallel
	if LengthSquared(axis) < alphaTolSqr {
		// Clip segment B against side planes of segment A

		// Sides planes of A
		planesA: [2]Plane = ---
		planesA[0].normal = Neg(edgeA)
		planesA[0].offset = -Dot(edgeA, capsuleA.center1)
		planesA[1].normal = edgeA
		planesA[1].offset = Dot(edgeA, capsuleA.center2)

		// Clip points for B
		verticesB: [2]ClipVertex = ---
		verticesB[0].position = centerB1
		verticesB[0].separation = 0.0
		verticesB[0].pair = MakeFeaturePair(.featureShapeA, 0, .featureShapeA, 0)
		verticesB[1].position = centerB2
		verticesB[1].separation = 0.0
		verticesB[1].pair = MakeFeaturePair(.featureShapeA, 1, .featureShapeA, 1)

		pointCount := ClipSegment(&verticesB[0], planesA[0])
		if pointCount == 2 {
			pointCount = ClipSegment(&verticesB[0], planesA[1])
		}

		if pointCount == 2 {
			// Closest points on A to the clipped points on B.
			closestPoint1 := PointToSegmentDistance(centerA1, centerA2, verticesB[0].position)
			closestPoint2 := PointToSegmentDistance(centerA1, centerA2, verticesB[1].position)

			distance1 := Distance(closestPoint1, verticesB[0].position)
			distance2 := Distance(closestPoint2, verticesB[1].position)
			if distance1 <= radius && distance2 <= radius {
				if distance1 < minDistance || distance2 < minDistance {
					// Avoid divide by zero
					return
				}

				normal1 := MulSV(1.0 / distance1, Sub(verticesB[0].position, closestPoint1))
				normal2 := MulSV(1.0 / distance2, Sub(verticesB[1].position, closestPoint2))
				normal := Normalize(Add(normal1, normal2))
				radiusA := capsuleA.radius
				radiusB := capsuleB.radius

				// Contact is at the midpoint: 0.5 * (((vB.pos + rA*nK) + cP) - rB*n)
				point1 := MulSV(
					0.5,
					MulSub(Add(MulAdd(verticesB[0].position, radiusA, normal1), closestPoint1), radiusB, normal),
				)
				point2 := MulSV(
					0.5,
					MulSub(Add(MulAdd(verticesB[1].position, radiusA, normal2), closestPoint2), radiusB, normal),
				)

				// Manifold in frame A
				manifold.normal = normal
				manifold.pointCount = 2

				pt1 := &manifold.points[0]
				pt1.point = point1
				pt1.separation = distance1 - radius
				pt1.pair = verticesB[0].pair

				pt2 := &manifold.points[1]
				pt2.point = point2
				pt2.separation = distance2 - radius
				pt2.pair = verticesB[1].pair

				return
			}
		}
	}

	distance: f32
	normal := GetLengthAndNormalize(&distance, offset)
	// Contact at the midpoint 0.5 * (((p1 + rA*n) + p2) - rB*n)
	point := MulSV(
		0.5,
		MulSub(Add(MulAdd(result.point1, capsuleA.radius, normal), result.point2), capsuleB.radius, normal),
	)

	// Manifold in frame A
	manifold.normal = normal
	manifold.pointCount = 1

	pt := &manifold.points[0]
	pt.point = point
	pt.separation = distance - radius
	pt.pair = FeaturePair_single
}

@(private = "file")
BuildHullFaceAndCapsuleContact :: proc(
	manifold: ^LocalManifold,
	hullA: ^HullData,
	capsuleB: ^Capsule,
	transformBtoA: Transform,
	query: SeparatingAxis,
) -> bool {
	// Work in shapeA coordinates
	planes := GetHullPlanes(hullA)

	// Clip the capsule edge against the side planes of the reference face
	refFace := query.indexA
	refPlane := planes[refFace]

	segmentB: [2]ClipVertex = ---
	segmentB[0].position = TransformPoint(transformBtoA, capsuleB.center1)
	segmentB[0].separation = 0.0
	segmentB[0].pair = MakeFeaturePair(.featureShapeA, 0, .featureShapeA, 0)
	segmentB[1].position = TransformPoint(transformBtoA, capsuleB.center2)
	segmentB[1].separation = 0.0
	segmentB[1].pair = MakeFeaturePair(.featureShapeA, 1, .featureShapeA, 1)

	pointCount := ClipSegmentToHullFace(&segmentB[0], hullA, refFace)
	if pointCount < 2 {
		return false
	}

	distance1 := PlaneSeparation(refPlane, segmentB[0].position)
	distance2 := PlaneSeparation(refPlane, segmentB[1].position)
	speculativeDistance: f32 = SPECULATIVE_DISTANCE

	if distance1 <= speculativeDistance || distance2 <= speculativeDistance {
		normal := refPlane.normal
		point1 := MulSub(segmentB[0].position, 0.5 * (distance1 + capsuleB.radius), normal)
		point2 := MulSub(segmentB[1].position, 0.5 * (distance2 + capsuleB.radius), normal)

		// Manifold in frame A
		manifold.normal = normal
		manifold.pointCount = 2

		pt1 := &manifold.points[0]
		pt1.point = point1
		pt1.separation = distance1 - capsuleB.radius
		pt1.pair = segmentB[0].pair

		pt2 := &manifold.points[1]
		pt2.point = point2
		pt2.separation = distance2 - capsuleB.radius
		pt2.pair = segmentB[1].pair

		return true
	}

	return false
}

@(private = "file")
BuildHullAndCapsuleEdgeContact :: proc(
	manifold: ^LocalManifold,
	capacity: i32,
	hullA: ^HullData,
	capsuleB: ^Capsule,
	transformBtoA: Transform,
	query: SeparatingAxis,
) -> bool {
	if capacity < 1 {
		return false
	}

	// Work in shapeA coordinates

	pc := TransformPoint(transformBtoA, capsuleB.center1)
	qc := TransformPoint(transformBtoA, capsuleB.center2)
	ec := Sub(qc, pc)

	edges := GetHullEdges(hullA)
	points := GetHullPoints(hullA)

	edge2 := &edges[query.indexB]
	twin2 := &edges[edge2.twin]
	ph := points[edge2.origin]
	qh := points[twin2.origin]
	eh := Sub(qh, ph)

	normal := query.normal

	result := LineDistance(ph, eh, pc, ec)

	if IsWithinSegments(&result) == false {
		// closest point beyond end points
		return false
	}

	point := MulSV(0.5, Add(MulSub(result.point1, capsuleB.radius, normal), result.point2))
	separation := Dot(normal, Sub(result.point2, result.point1))
	VALIDATE(AbsFloat(separation - query.separation) < LINEAR_SLOP)

	// Manifold in frame A
	manifold.normal = normal
	manifold.pointCount = 1

	pt := &manifold.points[0]
	pt.point = point
	pt.separation = separation - capsuleB.radius
	pt.pair = MakeFeaturePair(.featureShapeA, query.indexA, .featureShapeB, query.indexB)
	return true
}

CollideHullAndCapsule :: proc(
	manifold: ^LocalManifold,
	capacity: i32,
	hullA: ^HullData,
	capsuleB: ^Capsule,
	transformBtoA: Transform,
	cache: ^SimplexCache,
) {
	manifold.pointCount = 0

	if capacity < 2 {
		return
	}

	// Work in shapeA coordinates
	distanceInput: DistanceInput
	distanceInput.proxyA = ShapeProxy{GetHullPoints(hullA), hullA.vertexCount, 0.0}
	distanceInput.proxyB = ShapeProxy{&capsuleB.center1, 2, 0.0}
	distanceInput.transform = transformBtoA
	distanceInput.useRadii = false

	distanceOutput := ShapeDistance(&distanceInput, cache, nil, 0)
	speculativeDistance: f32 = SPECULATIVE_DISTANCE

	if distanceOutput.distance > capsuleB.radius + speculativeDistance {
		// We found a separating axis
		cache^ = SimplexCache{}
		return
	}

	if distanceOutput.distance > 100.0 * FLT_EPSILON {
		planes := GetHullPlanes(hullA)

		// Shallow penetration
		delta := distanceOutput.normal
		refFace := FindHullSupportFace(hullA, delta)
		refPlane := planes[refFace]

		// Try to create two contact points if closest
		// points difference is nearly parallel to face normal
		kTolerance: f32 : 0.998
		if AbsFloat(Dot(refPlane.normal, delta)) > kTolerance {
			// Clip capsule segment against side planes of reference face
			verticesB: [2]ClipVertex = ---
			verticesB[0].position = TransformPoint(transformBtoA, capsuleB.center1)
			verticesB[0].separation = 0.0
			verticesB[0].pair = MakeFeaturePair(.featureShapeA, 0, .featureShapeA, 0)
			verticesB[1].position = TransformPoint(transformBtoA, capsuleB.center2)
			verticesB[1].separation = 0.0
			verticesB[1].pair = MakeFeaturePair(.featureShapeA, 1, .featureShapeA, 1)

			pointCount := ClipSegmentToHullFace(&verticesB[0], hullA, refFace)

			if pointCount == 2 {
				distance1 := PlaneSeparation(refPlane, verticesB[0].position)
				distance2 := PlaneSeparation(refPlane, verticesB[1].position)
				if distance1 <= capsuleB.radius + speculativeDistance ||
				   distance2 <= capsuleB.radius + speculativeDistance {
					normal := refPlane.normal
					point1 := MulSub(verticesB[0].position, 0.5 * (capsuleB.radius + distance1), normal)
					point2 := MulSub(verticesB[1].position, 0.5 * (capsuleB.radius + distance2), normal)

					// Manifold in frame A
					manifold.normal = normal
					manifold.pointCount = 2

					pt1 := &manifold.points[0]
					pt1.point = point1
					pt1.separation = distance1 - capsuleB.radius
					pt1.pair = verticesB[0].pair

					pt2 := &manifold.points[1]
					pt2.point = point2
					pt2.separation = distance2 - capsuleB.radius
					pt2.pair = verticesB[1].pair

					return
				}
			}
		}

		// Create contact from closest points
		point := MulSV(0.5, Add(MulSub(distanceOutput.pointA, capsuleB.radius, delta), distanceOutput.pointB))

		// Manifold in frame A
		manifold.normal = delta
		manifold.pointCount = 1

		pt := &manifold.points[0]
		pt.point = point
		pt.separation = distanceOutput.distance - capsuleB.radius
		pt.pair = FeaturePair_single
		return
	}

	// Deep penetration

	faceQuery := QueryFaceDirectionHullAndCapsule(hullA, capsuleB, transformBtoA)
	if faceQuery.separation > capsuleB.radius {
		// We found a separating axis
		return
	}

	edgeQuery := QueryEdgeDirectionHullAndCapsule(hullA, capsuleB, transformBtoA)
	if edgeQuery.separation > capsuleB.radius {
		// We found a separating axis
		return
	}

	// Create face contact
	faceSeparation := faceQuery.separation - capsuleB.radius
	BuildHullFaceAndCapsuleContact(manifold, hullA, capsuleB, transformBtoA, faceQuery)
	VALIDATE(manifold.pointCount == 0 || manifold.pointCount == 2)
	if manifold.pointCount == 2 {
		// This becomes the clipped separation.
		faceSeparation = MinFloat(manifold.points[0].separation, manifold.points[1].separation)
	}

	// Is there a valid edge-edge axis?
	if edgeQuery.indexA == NULL_INDEX {
		return
	}

	// Face contact can be empty if it does not realize the axis of minimum penetration.
	// Create edge contact if face contact fails or edge contact is significantly better!
	linearSlop: f32 = LINEAR_SLOP
	edgeSeparation := edgeQuery.separation - capsuleB.radius
	if manifold.pointCount == 0 || edgeSeparation > faceSeparation + linearSlop {
		// Edge contact
		BuildHullAndCapsuleEdgeContact(manifold, capacity, hullA, capsuleB, transformBtoA, edgeQuery)
	}
}

#assert(MAX_MANIFOLD_POINTS >= 4, "must be 4 or more")

when MAX_MANIFOLD_POINTS == 4 {

	// Reduce the manifold points to a maximum of 4 points.
	// Note: this modifies the input point array to improve performance
	@(private = "file")
	ReduceManifoldPoints :: proc(manifold: ^LocalManifold, capacity: i32, points: [^]LocalManifoldPoint, count: i32) {
		count := count

		if capacity < 4 {
			return
		}

		if count <= 4 {
			for i: i32 = 0; i < count; i += 1 {
				manifold.points[i] = points[i]
			}

			manifold.pointCount = count
			return
		}

		normal := manifold.normal
		// float linearSlop = B3_LINEAR_SLOP;
		speculativeDistance: f32 = SPECULATIVE_DISTANCE
		tolSqr := speculativeDistance * speculativeDistance

		// This bias is very important for contact point consistency across time steps.
		// It creates a pecking order to avoid flickering between candidates with similar scores.
		bias: f32 = 0.95

		// Step 1: find extreme point that is touching
		bestIndex: i32 = NULL_INDEX
		bestScore := -max(f32)

		// Arbitrary tangent direction
		// b3Vec3 perp1 = b3Perp( normal );
		// b3Vec3 perp2 = b3Cross( perp1, normal );
		// b3Vec3 searchDirection = -0.4535961214255773f * perp1 + 0.8912073600614354f * perp2;
		searchDirection := ArbitraryPerp(normal)
		for index: i32 = 0; index < count; index += 1 {
			pt := &points[index]

			if pt.separation > speculativeDistance {
				continue
			}

			// The deeper the better
			score := -pt.separation + Dot(searchDirection, pt.point)
			if bias * score > bestScore {
				bestIndex = index
				bestScore = score
			}
		}

		VALIDATE(0 <= bestIndex && bestIndex < count)
		if bestIndex == NULL_INDEX {
			manifold.pointCount = 0
			return
		}

		manifold.points[0] = points[bestIndex]
		manifold.pointCount = 1

		// Remove best point from array
		points[bestIndex] = points[count - 1]
		count -= 1

		a := manifold.points[0].point

		// Step 2: Find farthest point in 2D
		bestScore = 0.0
		bestIndex = NULL_INDEX

		for index: i32 = 0; index < count; index += 1 {
			p := points[index].point
			d := Sub(p, a)
			v := MulSub(d, Dot(d, normal), normal)
			distanceSquared := LengthSquared(v)
			separation := MaxFloat(0.0, -points[index].separation)
			score := distanceSquared + 4.0 * separation * separation
			if bias * score > bestScore {
				bestScore = score
				bestIndex = index
			}
		}

		if bestScore < tolSqr {
			return
		}

		assert(0 <= bestIndex && bestIndex < count)
		manifold.points[1] = points[bestIndex]
		manifold.pointCount = 2

		// Remove best point from array
		points[bestIndex] = points[count - 1]
		count -= 1

		b := manifold.points[1].point

		// Step 3: Find the point with the maximum triangular area
		bestScore = tolSqr
		bestIndex = NULL_INDEX
		bestSignedArea: f32 = 0.0
		ba := Sub(b, a)
		for index: i32 = 0; index < count; index += 1 {
			p := points[index].point
			signedArea := Dot(normal, Cross(ba, Sub(p, a)))
			score := AbsFloat(signedArea)
			if bias * score >= bestScore {
				bestScore = score
				bestIndex = index
				bestSignedArea = signedArea
			}
		}

		if bestIndex == NULL_INDEX {
			return
		}

		assert(bestIndex != NULL_INDEX)

		manifold.points[2] = points[bestIndex]
		manifold.pointCount = 3
		points[bestIndex] = points[count - 1]
		count -= 1

		c := manifold.points[2].point

		// Step 4: get the point that adds the most area outside the current triangle
		bestScore = tolSqr
		bestIndex = NULL_INDEX
		sign: f32 = bestSignedArea < 0.0 ? -1.0 : 1.0
		for index: i32 = 0; index < count; index += 1 {
			p := points[index].point
			u1 := sign * Dot(normal, Cross(Sub(p, a), ba))
			u2 := sign * Dot(normal, Cross(Sub(p, b), Sub(c, b)))
			u3 := sign * Dot(normal, Cross(Sub(p, c), Sub(a, c)))
			score := MaxFloat(u1, MaxFloat(u2, u3))

			if bias * score > bestScore {
				bestScore = score
				bestIndex = index
			}
		}

		if bestIndex != NULL_INDEX {
			manifold.points[manifold.pointCount] = points[bestIndex]
			manifold.pointCount += 1
		}
	}

} else {

	@(private = "file")
	ReduceManifoldPoints :: proc(manifold: ^LocalManifold, capacity: i32, points: [^]LocalManifoldPoint, count: i32) {
		if capacity < 4 {
			return
		}

		assert(count <= MAX_CLIP_POINTS)

		target := MinInt(capacity, MAX_MANIFOLD_POINTS)

		if count <= target {
			for i: i32 = 0; i < count; i += 1 {
				manifold.points[i] = points[i]
			}

			manifold.pointCount = count
			return
		}

		normal := manifold.normal
		u := Perp(normal)
		v := Cross(normal, u)
		origin := points[0].point

		pts: [MAX_CLIP_POINTS]Point2D = ---
		for i: i32 = 0; i < count; i += 1 {
			d := Sub(points[i].point, origin)
			pts[i].p = Vec2{Dot(d, u), Dot(d, v)}
			pts[i].separation = points[i].separation
			pts[i].originalIndex = i
		}

		hull: [2 * MAX_CLIP_POINTS]Point2D = ---
		hullCount := Hull2D(&pts[0], count, &hull[0])
		finalCount := SimplifyHull2D(&hull[0], hullCount, target)
		assert(0 < finalCount && finalCount <= target)

		for i: i32 = 0; i < finalCount; i += 1 {
			index := hull[i].originalIndex
			assert(0 <= index && index < count)
			manifold.points[i] = points[index]
		}

		manifold.pointCount = finalCount
	}

}

@(private = "file")
BuildPolygon :: proc(out: [^]ClipVertex, transform: Transform, hull: ^HullData, incFace: i32, refPlane: Plane) -> i32 {
	faces := GetHullFaces(hull)
	edges := GetHullEdges(hull)
	points := GetHullPoints(hull)

	face := &faces[incFace]
	edgeIndex := i32(face.edge)
	assert(i32(edges[edgeIndex].face) == incFace)

	outCount: i32 = 0

	matrix_ := MakeMatrixFromQuat(transform.q)

	for {
		edge := &edges[edgeIndex]

		nextEdgeIndex := i32(edge.next)
		next := &edges[nextEdgeIndex]

		vertex: ClipVertex
		vertex.position = Add(MulMV(matrix_, points[next.origin]), transform.p)
		vertex.separation = PlaneSeparation(refPlane, vertex.position)
		vertex.pair = MakeFeaturePair(.featureShapeB, edgeIndex, .featureShapeB, nextEdgeIndex)

		out[outCount] = vertex
		outCount += 1

		edgeIndex = nextEdgeIndex

		if !(edgeIndex != i32(face.edge) && outCount < MAX_CLIP_POINTS) {
			break
		}
	}

	VALIDATE(ValidatePolygon(out, outCount))

	return outCount
}

@(private = "file")
BuildFaceAContact :: proc(
	manifold: ^LocalManifold,
	capacity: i32,
	hullA: ^HullData,
	hullB: ^HullData,
	transformBtoA: Transform,
	query: SeparatingAxis,
	cache: ^SATCache,
) -> bool {
	assert(capacity > 0)
	VALIDATE(query.type == .faceAxisA)
	VALIDATE(0 <= query.indexA && query.indexA < hullA.faceCount)
	VALIDATE(0 <= query.indexB && query.indexB < hullB.vertexCount)

	facesA := GetHullFaces(hullA)
	edgesA := GetHullEdges(hullA)
	planesA := GetHullPlanes(hullA)
	pointsA := GetHullPoints(hullA)

	// Reference face
	refFace := query.indexA
	refPlane := planesA[refFace]

	// Find incident face
	refNormalInB := InvRotateVector(transformBtoA.q, refPlane.normal)
	incFace := FindIncidentFace(hullB, refNormalInB, query.indexB)

	// Build clip polygon from incident face in frame A
	buffer1, buffer2: [MAX_CLIP_POINTS]ClipVertex = ---, ---
	pointCount := BuildPolygon(&buffer1[0], transformBtoA, hullB, incFace, refPlane)

	// Clip incident face against side planes of reference face
	input: [^]ClipVertex = &buffer1[0]
	output: [^]ClipVertex = &buffer2[0]

	face := &facesA[refFace]
	edgeIndex := i32(face.edge)

	for {
		edge := &edgesA[edgeIndex]
		nextEdgeIndex := i32(edge.next)
		next := &edgesA[nextEdgeIndex]
		vertex1 := pointsA[edge.origin]
		vertex2 := pointsA[next.origin]
		tangent := Normalize(Sub(vertex2, vertex1))
		binormal := Cross(tangent, refPlane.normal)

		clipPlane := MakePlaneFromNormalAndPoint(binormal, vertex1)

		pointCount = ClipPolygon(output, input, pointCount, clipPlane, edgeIndex, refPlane)
		assert(pointCount <= MAX_CLIP_POINTS)

		output, input = input, output

		if pointCount < 3 {
			cache^ = SATCache{}
			return false
		}

		edgeIndex = nextEdgeIndex

		if edgeIndex == i32(face.edge) {
			break
		}
	}

	pointCount = MinInt(pointCount, MAX_CLIP_POINTS)

	points: [MAX_CLIP_POINTS]LocalManifoldPoint = ---
	minSeparation := max(f32)

	manifold.normal = refPlane.normal

	for i: i32 = 0; i < pointCount; i += 1 {
		clipPoint := &input[i]
		pt := &points[i]
		pt^ = LocalManifoldPoint{}

		// Using the half-way point keeps the points in the same position when swapping reference face from A to B.
		point := MulSub(clipPoint.position, 0.5 * clipPoint.separation, refPlane.normal)

		// Old way of pushing onto the reference face.
		// b3Vec3 point = clipPoint->position - clipPoint->separation * refPlane.normal;

		pt.point = point
		pt.separation = clipPoint.separation
		pt.pair = clipPoint.pair

		minSeparation = MinFloat(minSeparation, clipPoint.separation)
	}

	if minSeparation >= SPECULATIVE_DISTANCE {
		cache^ = SATCache{}
		return false
	}

	ReduceManifoldPoints(manifold, capacity, &points[0], pointCount)

	// Save cache
	cache.separation = minSeparation
	cache.type = u8(SeparatingFeature.faceAxisA)
	cache.indexA = u8(query.indexA)
	cache.indexB = u8(query.indexB)

	return true
}

@(private = "file")
BuildFaceBContact :: proc(
	manifold: ^LocalManifold,
	capacity: i32,
	hullA: ^HullData,
	hullB: ^HullData,
	transformBtoA: Transform,
	query: SeparatingAxis,
	cache: ^SATCache,
) -> bool {
	VALIDATE(query.type == .faceAxisB)

	transformAtoB := InvertTransform(transformBtoA)
	flippedQuery := SeparatingAxis {
		normal     = Neg(query.normal),
		separation = query.separation,
		indexA     = query.indexB,
		indexB     = query.indexA,
		type       = .faceAxisA,
	}

	touching := BuildFaceAContact(manifold, capacity, hullB, hullA, transformAtoB, flippedQuery, cache)
	if touching == false {
		cache^ = SATCache{}
		return false
	}

	// Results are in frame B, need to transform them into frame A
	matrix_ := MakeMatrixFromQuat(transformBtoA.q)

	// Transform and flip normal so it points from A to B, even though the B has the reference face.
	manifold.normal = Neg(MulMV(matrix_, manifold.normal))

	// Transform points from frame B to frame A.
	// Also flip the pairs to ensure correct matches.
	for i: i32 = 0; i < manifold.pointCount; i += 1 {
		pt := &manifold.points[i]
		pt.point = Add(MulMV(matrix_, pt.point), transformBtoA.p)
		pt.pair = FlipPair(pt.pair)
	}

	cache.type = u8(SeparatingFeature.faceAxisB)
	cache.indexA = u8(query.indexA)
	cache.indexB = u8(query.indexB)

	return true
}

@(private = "file")
BuildEdgeContact :: proc(
	manifold: ^LocalManifold,
	hullA: ^HullData,
	hullB: ^HullData,
	transformBtoA: Transform,
	query: SeparatingAxis,
	cache: ^SATCache,
) -> bool {
	VALIDATE(query.type == .edgePairAxis)
	VALIDATE(0 <= query.indexA && query.indexA < hullA.edgeCount)
	VALIDATE(0 <= query.indexB && query.indexB < hullB.edgeCount)

	// Work in shapeA coordinates
	edgesA := GetHullEdges(hullA)
	pointsA := GetHullPoints(hullA)

	edgesB := GetHullEdges(hullB)
	pointsB := GetHullPoints(hullB)

	// B3_VALIDATE( query.separation <= 2.0f * B3_SPECULATIVE_DISTANCE );

	edgeA := &edgesA[query.indexA]
	twinA := &edgesA[edgeA.twin]
	pA := pointsA[edgeA.origin]
	qA := pointsA[twinA.origin]
	eA := Sub(qA, pA)

	edgeB := &edgesB[query.indexB]
	twinB := &edgesB[edgeB.twin]
	pB := TransformPoint(transformBtoA, pointsB[edgeB.origin])
	qB := TransformPoint(transformBtoA, pointsB[twinB.origin])
	eB := Sub(qB, pB)

	normal := query.normal
	result := LineDistance(pA, eA, pB, eB)

	if IsWithinSegments(&result) == false {
		cache^ = SATCache{}
		return false
	}

	// This can slide off the end from caching
	separation := Dot(normal, Sub(result.point2, result.point1))
	point := MulSV(0.5, Add(result.point1, result.point2))

	// Result in frame A
	manifold.normal = normal
	manifold.pointCount = 1

	pt := &manifold.points[0]
	pt.point = point
	pt.separation = separation
	pt.pair = MakeFeaturePair(.featureShapeA, query.indexA, .featureShapeB, query.indexB)

	// Save cache
	cache.separation = separation
	cache.type = u8(SeparatingFeature.edgePairAxis)
	cache.indexA = u8(query.indexA)
	cache.indexB = u8(query.indexB)

	return true
}

ComputeSeparatingAxisAtWidth :: proc(
	hullA: ^HullData,
	hullB: ^HullData,
	xfB: Transform,
	earlyReturn: bool,
	simdWidth: i32,
) -> AxisQuery {
	// unified: not ported: the width-8 dispatch (B3_SIMD_HAS_WIDTH_8, AVX2); every platform runs width 4
	_ = simdWidth

	return ComputeSeparatingAxisW4(hullA, hullB, xfB, earlyReturn)
}

ComputeSeparatingAxis :: proc(hullA: ^HullData, hullB: ^HullData, xfB: Transform, earlyReturn: bool) -> AxisQuery {
	return ComputeSeparatingAxisAtWidth(hullA, hullB, xfB, earlyReturn, GetSIMDWidth())
}

CollideHullsAtWidth :: proc(
	manifold: ^LocalManifold,
	capacity: i32,
	hullA: ^HullData,
	hullB: ^HullData,
	transformBtoA: Transform,
	cache: ^SATCache,
	simdWidth: i32,
) {
	manifold.pointCount = 0

	if capacity < 4 {
		return
	}

	// Work in shapeA coordinates
	speculativeDistance: f32 = SPECULATIVE_DISTANCE

	linearSlop: f32 = LINEAR_SLOP
	edgesA := GetHullEdges(hullA)
	planesA := GetHullPlanes(hullA)
	pointsA := GetHullPoints(hullA)

	edgesB := GetHullEdges(hullB)
	planesB := GetHullPlanes(hullB)
	pointsB := GetHullPoints(hullB)

	cache.hit = 0

	// Attempt to use the cache to speed up collision
	#partial switch SeparatingFeature(cache.type) {
	case .invalidAxis:

	case .faceAxisA:
		assert(i32(cache.indexA) < hullA.faceCount)

		// Check for separation using cached face
		plane := planesA[cache.indexA]
		searchDirectionInB := Neg(InvRotateVector(transformBtoA.q, plane.normal))

		vertexIndex := FindHullSupportVertex(hullB, searchDirectionInB)
		support := TransformPoint(transformBtoA, pointsB[vertexIndex])
		separation := PlaneSeparation(plane, support)

		if separation >= speculativeDistance {
			// Cache hit, shapes are separated
			cache.hit = 1
			return
		}

		// Attempt face contact using cached feature
		faceQuery: SeparatingAxis
		faceQuery.normal = plane.normal
		faceQuery.separation = 0.0
		faceQuery.indexA = i32(cache.indexA)
		faceQuery.indexB = vertexIndex
		faceQuery.type = .faceAxisA

		localCache := SATCache{}
		touching := BuildFaceAContact(manifold, capacity, hullA, hullB, transformBtoA, faceQuery, &localCache)
		if touching == true && AbsFloat(cache.separation - localCache.separation) < linearSlop {
			// Cache hit, contact points generated
			cache.hit = 1
			return
		}

	case .faceAxisB:
		assert(i32(cache.indexB) < hullB.faceCount)

		// Check for separation using cached face
		plane := planesB[cache.indexB]
		searchDirectionInA := Neg(RotateVector(transformBtoA.q, plane.normal))

		// todo use b3GetSupportWide
		vertexIndex := FindHullSupportVertex(hullA, searchDirectionInA)
		support := InvTransformPoint(transformBtoA, pointsA[vertexIndex])
		separation := PlaneSeparation(plane, support)

		if separation >= speculativeDistance {
			// Cache hit, shapes are separated
			cache.hit = 1
			return
		}

		// Attempt face contact using cached feature
		faceQuery: SeparatingAxis
		faceQuery.normal = Neg(plane.normal)
		faceQuery.separation = 0.0
		faceQuery.indexA = vertexIndex
		faceQuery.indexB = i32(cache.indexB)
		faceQuery.type = .faceAxisB

		localCache := SATCache{}
		touching := BuildFaceBContact(manifold, capacity, hullA, hullB, transformBtoA, faceQuery, &localCache)
		if touching == true && AbsFloat(cache.separation - localCache.separation) < linearSlop {
			// Cache hit, contact points generated
			cache.hit = 1
			return
		}

	case .edgePairAxis:
		indexA := i32(cache.indexA)
		edge1 := &edgesA[indexA]
		twin1 := &edgesA[indexA + 1]
		assert(i32(edge1.twin) == indexA + 1 && i32(twin1.twin) == indexA)

		pA := pointsA[edge1.origin]
		qA := pointsA[twin1.origin]
		eA := Sub(qA, pA)

		uA := planesA[edge1.face].normal
		vA := planesA[twin1.face].normal

		indexB := i32(cache.indexB)
		edge2 := &edgesB[indexB]
		twin2 := &edgesB[indexB + 1]
		assert(i32(edge2.twin) == indexB + 1 && i32(twin2.twin) == indexB)

		pB := TransformPoint(transformBtoA, pointsB[edge2.origin])
		qB := TransformPoint(transformBtoA, pointsB[twin2.origin])
		eB := Sub(qB, pB)

		uB := RotateVector(transformBtoA.q, planesB[edge2.face].normal)
		vB := RotateVector(transformBtoA.q, planesB[twin2.face].normal)

		// flipping the signs of u2 and v2
		// cross(v2, u2) == cross(-v2, -u2)
		// so we still use -e2
		// but we can also use e1 = cross(u1, v1) and e2 = cross(u2, v2)
		cba := Dot(uB, eA)
		dba := Dot(vB, eA)
		adc := -Dot(uA, eB)
		bdc := -Dot(vA, eB)

		if cba * dba < 0.0 && adc * bdc < 0.0 && cba * bdc > 0.0 {
			// Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
			squaredTolerance: f32 = PARALLEL_EDGE_TOL * PARALLEL_EDGE_TOL
			if MaxFloat(cba * cba, dba * dba) >= squaredTolerance * LengthSquared(eA) {
				// Transform reference center of the first hull into local space of the second hull
				t := cba / (cba - dba)
				axis := Lerp(uB, vB, t)
				VALIDATE(LengthSquared(axis) > 1000.0 * FLT_MIN)
				axis = Normalize(axis)
				separation := Dot(axis, Sub(qA, qB))

				if separation > speculativeDistance {
					// Cache hit, shapes are separated
					cache.hit = 1
					return
				}

				// Try to rebuild contact from last features
				edgeQuery := SeparatingAxis{}
				edgeQuery.normal = Neg(axis)
				edgeQuery.separation = 0.0
				edgeQuery.indexA = i32(cache.indexA)
				edgeQuery.indexB = i32(cache.indexB)
				edgeQuery.type = .edgePairAxis

				localCache := SATCache{}
				touching := BuildEdgeContact(manifold, hullA, hullB, transformBtoA, edgeQuery, &localCache)

				// This separation tolerance may have a big impact on performance in some benchmarks.
				if touching && AbsFloat(cache.separation - localCache.separation) < linearSlop {
					// Cache hit, contact point generated
					cache.hit = 1
					return
				}
			}
		}

	// This case is for testing
	case .manualFaceAxisA:
		axisQuery := ComputeSeparatingAxisAtWidth(hullA, hullB, transformBtoA, false, simdWidth)
		faceQuery := axisQuery.faceA
		BuildFaceAContact(manifold, capacity, hullA, hullB, transformBtoA, faceQuery, cache)
		return

	// This case is for testing
	case .manualFaceAxisB:
		axisQuery := ComputeSeparatingAxisAtWidth(hullA, hullB, transformBtoA, false, simdWidth)
		faceQuery := axisQuery.faceB
		BuildFaceBContact(manifold, capacity, hullA, hullB, transformBtoA, faceQuery, cache)
		return

	// This case is for testing
	case .manualEdgePairAxis:
		axisQuery := ComputeSeparatingAxisAtWidth(hullA, hullB, transformBtoA, false, simdWidth)
		edgeQuery := axisQuery.edge
		if edgeQuery.indexA != NULL_INDEX {
			BuildEdgeContact(manifold, hullA, hullB, transformBtoA, edgeQuery, cache)
		}
		return

	case:
		assert(false)
	}

	manifold.pointCount = 0
	cache^ = SATCache{}

	axisQuery := ComputeSeparatingAxisAtWidth(hullA, hullB, transformBtoA, true, simdWidth)

	if axisQuery.separatedFeature != .invalidAxis {
		// We found a separating axis
		cache.type = u8(axisQuery.separatedFeature)

		if axisQuery.separatedFeature == .faceAxisA {
			VALIDATE(axisQuery.faceA.separation > speculativeDistance)
			cache.separation = axisQuery.faceA.separation
			cache.indexA = u8(axisQuery.faceA.indexA)
			cache.indexB = u8(axisQuery.faceA.indexB)
		} else if axisQuery.separatedFeature == .faceAxisB {
			VALIDATE(axisQuery.faceB.separation > speculativeDistance)
			cache.separation = axisQuery.faceB.separation
			cache.indexA = u8(axisQuery.faceB.indexA)
			cache.indexB = u8(axisQuery.faceB.indexB)
		} else {
			assert(axisQuery.separatedFeature == .edgePairAxis)
			VALIDATE(axisQuery.edge.separation > speculativeDistance)
			cache.separation = axisQuery.edge.separation
			cache.indexA = u8(axisQuery.edge.indexA)
			cache.indexB = u8(axisQuery.edge.indexB)
		}
		return
	}

	VALIDATE(
		axisQuery.faceA.separation <= speculativeDistance ||
		axisQuery.faceB.separation <= speculativeDistance ||
		axisQuery.edge.separation <= speculativeDistance,
	)

	if axisQuery.faceA.separation > axisQuery.faceB.separation {
		faceQuery := axisQuery.faceA
		VALIDATE(0 <= faceQuery.indexA && faceQuery.indexA < hullA.faceCount)
		VALIDATE(0 <= faceQuery.indexB && faceQuery.indexB < hullB.vertexCount)

		// Face contact A
		BuildFaceAContact(manifold, capacity, hullA, hullB, transformBtoA, faceQuery, cache)

		VALIDATE(i32(cache.indexA) < hullA.faceCount)
		VALIDATE(i32(cache.indexB) < hullB.vertexCount)
	} else {
		faceQuery := axisQuery.faceB
		VALIDATE(0 <= faceQuery.indexA && faceQuery.indexA < hullA.vertexCount)
		VALIDATE(0 <= faceQuery.indexB && faceQuery.indexB < hullB.faceCount)

		// Face contact B
		BuildFaceBContact(manifold, capacity, hullA, hullB, transformBtoA, faceQuery, cache)

		VALIDATE(i32(cache.indexA) < hullA.vertexCount)
		VALIDATE(i32(cache.indexB) < hullB.faceCount)
	}

	edgeQuery := axisQuery.edge

	if edgeQuery.indexA == NULL_INDEX {
		// There are no valid edge pairs (all edges parallel)
		return
	}

	faceSeparation := MaxFloat(axisQuery.faceA.separation, axisQuery.faceB.separation)
	clipSeparation := cache.separation
	edgeTol := linearSlop

	// Face contact can be empty if it is not the axis of maximum separation. It can also
	// be empty in narrow cases in the speculative region. If that case was important then
	// a GJK fallback would be used. So far it doesn't seem important.
	// Create edge contact if face contact fails or edge contact is significantly better.
	if (manifold.pointCount == 0 && edgeQuery.separation > faceSeparation) ||
	   edgeQuery.separation > clipSeparation + edgeTol {
		assert(0 <= edgeQuery.indexA && edgeQuery.indexA < hullA.edgeCount)
		assert(0 <= edgeQuery.indexB && edgeQuery.indexB < hullB.edgeCount)

		// Edge contact
		edgeManifold := LocalManifold{}
		edgePoint := LocalManifoldPoint{}
		edgeManifold.points = &edgePoint

		edgeCache := SATCache{}
		BuildEdgeContact(&edgeManifold, hullA, hullB, transformBtoA, edgeQuery, &edgeCache)

		// It is possible with speculation to have vertex-vertex collision that is missed by SAT,
		// so edge contact yields no points. In that case perhaps the face contact has some points.
		if edgeManifold.pointCount == 1 {
			// Copy edge manifold out, being careful to preserve manifold point buffer.
			points := manifold.points
			// unified: the buffer is swapped back before the copy (same result; Odin warns on a stack pointer escape)
			edgeManifold.points = points
			manifold^ = edgeManifold
			// unified end
			manifold.points[0] = edgePoint
			cache^ = edgeCache
		}
	}
}

CollideHulls :: proc(
	manifold: ^LocalManifold,
	capacity: i32,
	hullA: ^HullData,
	hullB: ^HullData,
	transformBtoA: Transform,
	cache: ^SATCache,
) {
	CollideHullsAtWidth(manifold, capacity, hullA, hullB, transformBtoA, cache, GetSIMDWidth())
}
