// Ported from box3d src/manifold.c and src/manifold.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

// manifold.h

MAX_CLIP_POINTS :: 64

SeparatingAxis :: struct {
	normal:     Vec3,
	separation: f32,
	indexA:     i32,
	indexB:     i32,
	type:       SeparatingFeature,
}

AxisQuery :: struct {
	faceA:            SeparatingAxis,
	faceB:            SeparatingAxis,
	edge:             SeparatingAxis,
	separatedFeature: SeparatingFeature,
}

ClipVertex :: struct {
	position:   Vec3,
	separation: f32,
	pair:       FeaturePair,
}

FeatureOwner :: enum i32 {
	featureShapeA = 0,
	featureShapeB = 1,
}

// unified: ComputeSeparatingAxis, ComputeSeparatingAxisAtWidth, CollideHullsAtWidth, ComputeSeparatingAxisW4 and
// ComputeSeparatingAxisW8 are declared in manifold.h and defined in convex_manifold.c (their port lives there).

// For single point contact, such as sphere-sphere, sphere-capsule, sphere-triangle
@(rodata)
FeaturePair_single := FeaturePair{}

MakeFeatureId :: proc(pair: FeaturePair) -> u32 {
	return (u32(pair.owner1) << 24) | (u32(pair.index1) << 16) | (u32(pair.owner2) << 8) | u32(pair.index2)
}

GetBestAxis :: proc(query: ^AxisQuery) -> SeparatingAxis {
	VALIDATE(query.faceA.type == .faceAxisA)
	VALIDATE(query.edge.type == .edgePairAxis)
	VALIDATE(query.faceB.type == .faceAxisB)

	if query.faceA.separation > query.faceB.separation {
		if query.edge.separation > query.faceA.separation {
			return query.edge
		}

		return query.faceA
	}

	if query.edge.separation > query.faceB.separation {
		return query.edge
	}

	return query.faceB
}

// manifold.c

// Dirk Gregorius contributed portions of this code

// This was extended to make the wedge shape get the correct incident face.
// Instead of looking directly for the most anti-parallel face, we first find the closest vertex (passed in).
// Then we look for all edges coming out of that vertex and look for the edge that is
// most perpendicular to the reference normal.
// Then from that edge, we select the adjacent face that is most anti-parallel to the reference normal.
FindIncidentFace :: proc(hull: ^HullData, refNormal: Vec3, vertexIndex: i32) -> i32 {
	assert(0 <= vertexIndex && vertexIndex < hull.vertexCount)

	vertices := GetHullVertices(hull)
	edges := GetHullEdges(hull)
	planes := GetHullPlanes(hull)
	points := GetHullPoints(hull)

	minEdgeIndex: i32 = -1
	minEdgeProjection := max(f32)

	vertex := &vertices[vertexIndex]
	assert(vertex != nil)

	edgeIndex := i32(vertex.edge)
	edge := &edges[edgeIndex]
	edgeOrigin := points[edge.origin]
	assert(i32(edge.origin) == vertexIndex)

	for {
		twin := &edges[edge.twin]
		twinOrigin := points[twin.origin]

		axis := Normalize(Sub(twinOrigin, edgeOrigin))
		edgeProjection := AbsFloat(Dot(axis, refNormal))
		if edgeProjection < minEdgeProjection {
			minEdgeIndex = edgeIndex
			minEdgeProjection = edgeProjection
		}

		edgeIndex = i32(twin.next)
		edge = &edges[edgeIndex]
		assert(i32(edge.origin) == vertexIndex)

		if edge == &edges[vertex.edge] {
			break
		}
	}
	assert(minEdgeIndex >= 0)

	minEdge := &edges[minEdgeIndex]
	minFaceIndex1 := i32(minEdge.face)
	minPlane1 := planes[minFaceIndex1]

	minTwin := &edges[minEdge.twin]
	minFaceIndex2 := i32(minTwin.face)
	minPlane2 := planes[minFaceIndex2]

	return Dot(minPlane1.normal, refNormal) < Dot(minPlane2.normal, refNormal) ? minFaceIndex1 : minFaceIndex2
}

MakeFeaturePair :: proc(owner1: FeatureOwner, index1: i32, owner2: FeatureOwner, index2: i32) -> FeaturePair {
	assert(0 <= index1 && index1 <= i32(max(u8)))
	assert(0 <= index2 && index2 <= i32(max(u8)))

	pair: FeaturePair
	pair.index1 = u8(index1)
	pair.owner1 = u8(owner1)
	pair.index2 = u8(index2)
	pair.owner2 = u8(owner2)
	return pair
}

// This logic seems wrong but it is designed so that choosing
// face A or B as the reference face does not change the resulting
// feature pair. This way the contact impulses are persisted even
// if there is reference face flip-flop. This is verified in the
// HullAndHull sample using the b3SATCache with a manual feature
// specified such as b3_manualFaceAxisA.
FlipPair :: proc(pair: FeaturePair) -> FeaturePair {
	pair := pair
	assert(pair.owner1 == 0 || pair.owner1 == 1)
	assert(pair.owner2 == 0 || pair.owner2 == 1)
	pair.owner1, pair.owner2 = pair.owner2, pair.owner1
	pair.owner1 = 1 - pair.owner1
	pair.owner2 = 1 - pair.owner2
	pair.index1, pair.index2 = pair.index2, pair.index1
	return pair
}

// unified: compiled unconditionally (C: #if B3_ENABLE_VALIDATION); only VALIDATE calls it.
ValidatePolygon :: proc(polygon: [^]ClipVertex, count: i32) -> bool {
	// Empty polygons are valid (we can clip away all points when re-constructing manifolds from cache)
	if count == 0 {
		return true
	}

	// Validate that incoming and outgoing edges match
	vertex1 := polygon[count - 1]
	for i: i32 = 0; i < count; i += 1 {
		vertex2 := polygon[i]

		if vertex1.pair.owner2 != vertex2.pair.owner1 {
			return false
		}

		if vertex1.pair.index2 != vertex2.pair.index1 {
			return false
		}

		vertex1 = vertex2
	}

	return true
}
// unified end

ClipPolygon :: proc(
	out: [^]ClipVertex,
	polygon: [^]ClipVertex,
	count: i32,
	clipPlane: Plane,
	edge: i32,
	refPlane: Plane,
) -> i32 {
	assert(count >= 3)

	vertex1 := polygon[count - 1]
	distance1 := PlaneSeparation(clipPlane, vertex1.position)
	outCount: i32 = 0

	for index: i32 = 0; index < count; index += 1 {
		vertex2 := polygon[index]
		distance2 := PlaneSeparation(clipPlane, vertex2.position)

		// Clip edge against plane (Sutherland-Hodgman clipping)
		if distance1 <= 0.0 && distance2 <= 0.0 {
			// Both vertices are behind the plane - keep vertex2
			out[outCount] = vertex2
			outCount += 1
		} else if distance1 <= 0.0 && distance2 > 0.0 {
			// Vertex1 is behind of the plane, vertex2 is in front -> intersection point
			fraction := distance1 / (distance1 - distance2)
			position := MulAdd(vertex1.position, fraction, Sub(vertex2.position, vertex1.position))

			// Keep intersection point and adjust outgoing edge
			vertex: ClipVertex
			vertex.position = position
			vertex.separation = PlaneSeparation(refPlane, position)
			vertex.pair = vertex2.pair
			vertex.pair.owner2 = u8(FeatureOwner.featureShapeA)
			vertex.pair.index2 = u8(edge)
			out[outCount] = vertex
			outCount += 1
		} else if distance2 <= 0.0 && distance1 > 0.0 {
			// Vertex1 is in front, vertex2 is behind of the plane, -> intersection point
			fraction := distance1 / (distance1 - distance2)
			position := MulAdd(vertex1.position, fraction, Sub(vertex2.position, vertex1.position))

			// Keep intersection point and adjust incoming edge
			vertex: ClipVertex
			vertex.position = position
			vertex.separation = PlaneSeparation(refPlane, position)
			vertex.pair = vertex1.pair
			vertex.pair.owner1 = u8(FeatureOwner.featureShapeA)
			vertex.pair.index1 = u8(edge)
			out[outCount] = vertex
			outCount += 1

			// And also keep vertex2
			out[outCount] = vertex2
			outCount += 1
		}

		// Keep vertex2 as starting vertex for next edge
		vertex1 = vertex2
		distance1 = distance2
	}

	VALIDATE(ValidatePolygon(out, outCount))

	return outCount
}
