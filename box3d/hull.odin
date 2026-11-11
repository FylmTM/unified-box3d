// Ported from box3d src/hull.c and src/hull.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:intrinsics"
import "core:mem"

// hull.h

// Big enough to handle AVX2.
HULL_SOA_PADDING :: 8

GetHullSoaStride :: proc(count: i32) -> i32 {
	return (count + HULL_SOA_PADDING - 1) & ~i32(HULL_SOA_PADDING - 1)
}

// Map keyed by hull content. The world hull database uses the value as a reference count,
// while compound baking stores uses the value as a byte offset. Implementation is in hull.c.
// unified: verstable's b3HullMap is a small chained hash map here (see "Implement HullMap" below).

// hull.c

// Dirk Gregorius contributed portions of this code

@(private = "file")
AXIS_X :: 0
@(private = "file")
AXIS_Y :: 1
@(private = "file")
AXIS_Z :: 2

@(private = "file")
MARK_VISIBLE :: 0
@(private = "file")
MARK_DELETE :: 1

// Final hull is index-encoded with uint8_t, so vertex/edge/face counts are capped at 256.
@(private = "file")
HULL_MAX_COUNT :: i32(max(u8)) + 1

@(private = "file")
QHListNode :: struct {
	prev: ^QHListNode,
	next: ^QHListNode,
}

@(private = "file")
QHVertex :: struct {
	// Intrusive list link. Must be first so (b3QHVertex*)nodePtr is valid.
	link:         QHListNode,
	conflictFace: ^QHFace,
	position:     Vec3,

	// Index in the finalized hull, stamped during emit. B3_NULL_INDEX until then.
	finalIndex:   i32,
	reachable:    bool,
}

@(private = "file")
QHHalfEdge :: struct {
	// Edge ring (CCW) around the owning face. Not an external list.
	prev:       ^QHHalfEdge,
	next:       ^QHHalfEdge,
	origin:     ^QHVertex,
	face:       ^QHFace,
	twin:       ^QHHalfEdge,

	// Index in the finalized hull, stamped during emit. B3_NULL_INDEX until then.
	finalIndex: i32,
}

@(private = "file")
QHFace :: struct {
	// Intrusive list link. Must be first so (b3QHFace*)nodePtr is valid.
	link:                QHListNode,
	edge:                ^QHHalfEdge,
	mark:                i32,
	area:                f32,
	plane:               Plane,
	centroid:            Vec3,
	maxConflictDistance: f32,

	// Sentinel head for this face's conflict list of b3QHVertex.
	conflictListHead:    QHVertex,

	// Cached farthest conflict vertex (above b3HullBuilder::minOutside).
	// NULL when no conflict above threshold; maxConflictDistance is then minOutside.
	maxConflict:         ^QHVertex,

	// Index in the finalized hull, stamped during emit. B3_NULL_INDEX until then.
	finalIndex:          i32,
	flipped:             bool,
}

// One frame of the iterative horizon DFS. Replaces a recursive call to b3HullBuilder_BuildHorizon.
@(private = "file")
HorizonFrame :: struct {
	face:      ^QHFace,
	startEdge: ^QHHalfEdge, // ring termination sentinel
	edge:      ^QHHalfEdge, // next edge to process
	started:   bool, // false until the first edge of this ring has been processed
}

// All working memory for one hull build, carved from a single b3Alloc block.
@(private = "file")
HullBuilder :: struct {
	tolerance:            f32,
	minRadius:            f32,
	minOutside:           f32,
	interiorPoint:        Vec3,

	// List sentinels. Only the link is meaningful; other fields are unused.
	orphanedList:         QHVertex,
	vertexList:           QHVertex,
	faceList:             QHFace,

	// Bump-allocated pools with pointer-based free lists for faces and edges.
	vertexBase:           [^]QHVertex,
	vertexCapacity:       i32,
	vertexCount:          i32,
	edgeBase:             [^]QHHalfEdge,
	edgeCapacity:         i32,
	edgeCount:            i32,

	// LIFO free list. Built with edge->next.
	edgeFreeHead:         ^QHHalfEdge,
	faceBase:             [^]QHFace,
	faceCapacity:         i32,
	faceCount:            i32,

	// LIFO free list. Built with face->link.next.
	faceFreeHead:         ^QHFace,

	// Reusable scratch buffers.
	horizon:              [^]^QHHalfEdge,
	horizonCapacity:      i32,
	horizonCount:         i32,
	cone:                 [^]^QHFace,
	coneCapacity:         i32,
	coneCount:            i32,
	mergedFaces:          [^]^QHFace,
	mergedFacesCapacity:  i32,
	mergedFacesCount:     i32,

	// DFS stack used by the iterative b3HullBuilder_BuildHorizon. Depth bounded by live faces.
	horizonStack:         [^]HorizonFrame,
	horizonStackCapacity: i32,

	// Final counts of the constructed hull (vertexList / faceList / half-edges around faces).
	finalVertexCount:     i32,
	finalHalfEdgeCount:   i32,
	finalFaceCount:       i32,
}

@(private = "file")
QHList_Init :: proc(head: ^QHListNode) {
	head.prev = head
	head.next = head
}

@(private = "file")
LIST_EMPTY :: #force_inline proc(A: ^QHListNode) -> bool {
	return A.next == A
}

@(private = "file")
QHList_Contains :: proc(node: ^QHListNode) -> bool {
	return node.prev != nil && node.next != nil
}

// Insert node before `where`.
@(private = "file")
QHList_Insert :: proc(node: ^QHListNode, where_: ^QHListNode) {
	assert(!QHList_Contains(node) && QHList_Contains(where_))

	node.prev = where_.prev
	node.next = where_

	node.prev.next = node
	node.next.prev = node
}

@(private = "file")
QHList_Remove :: proc(node: ^QHListNode) {
	assert(QHList_Contains(node))

	node.prev.next = node.next
	node.next.prev = node.prev

	node.prev = nil
	node.next = nil
}

@(private = "file")
QHList_PushBack :: proc(head: ^QHListNode, node: ^QHListNode) {
	QHList_Insert(node, head.prev)
}

@(private = "file")
HullBuilder_NewVertex :: proc(b: ^HullBuilder, position: Vec3) -> ^QHVertex {
	assert(b.vertexCount < b.vertexCapacity)
	vertex := &b.vertexBase[b.vertexCount]
	b.vertexCount += 1

	vertex.link.prev = nil
	vertex.link.next = nil
	vertex.conflictFace = nil
	vertex.position = position
	vertex.finalIndex = NULL_INDEX
	vertex.reachable = false

	return vertex
}

@(private = "file")
HullBuilder_NewEdge :: proc(b: ^HullBuilder) -> ^QHHalfEdge {
	edge: ^QHHalfEdge
	if b.edgeFreeHead != nil {
		edge = b.edgeFreeHead
		b.edgeFreeHead = edge.next
	} else {
		assert(b.edgeCount < b.edgeCapacity)
		edge = &b.edgeBase[b.edgeCount]
		b.edgeCount += 1
	}
	// All other fields (prev/next/origin/face/twin) are written by NewFace immediately after.
	edge.finalIndex = NULL_INDEX
	return edge
}

@(private = "file")
HullBuilder_RetireEdge :: proc(b: ^HullBuilder, edge: ^QHHalfEdge) {
	edge.next = b.edgeFreeHead
	b.edgeFreeHead = edge
}

@(private = "file")
HullBuilder_NewFace :: proc(b: ^HullBuilder, v1: ^QHVertex, v2: ^QHVertex, v3: ^QHVertex) -> ^QHFace {
	face: ^QHFace
	if b.faceFreeHead != nil {
		face = b.faceFreeHead
		// link.next was used as free-list pointer; recover next head before we clobber.
		b.faceFreeHead = (^QHFace)(face.link.next)
	} else {
		assert(b.faceCount < b.faceCapacity)
		face = &b.faceBase[b.faceCount]
		b.faceCount += 1
	}

	// link.prev: NULL on retired faces (cleared by Remove); NULL here so PushBack's
	// !b3QHList_Contains assert holds for fresh bump slots too.
	// link.next: was the free-list pointer on reused slots, now stale; PushBack overwrites.
	face.link.prev = nil
	face.link.next = nil
	face.maxConflict = nil
	face.maxConflictDistance = 0.0
	face.finalIndex = NULL_INDEX

	edge1 := HullBuilder_NewEdge(b)
	edge2 := HullBuilder_NewEdge(b)
	edge3 := HullBuilder_NewEdge(b)

	p1 := v1.position
	p2 := v2.position
	p3 := v3.position

	plane: Plane
	plane.normal = Cross(Sub(p2, p1), Sub(p3, p1))
	length: f32
	plane.normal = GetLengthAndNormalize(&length, plane.normal)
	plane.offset = Dot(plane.normal, p1)

	area := 0.5 * length

	face.edge = edge1
	face.mark = MARK_VISIBLE
	face.area = area
	face.centroid = MulSV(1.0 / 3.0, Add(v1.position, Add(v2.position, v3.position)))
	face.plane = plane
	face.flipped = PlaneSeparation(plane, b.interiorPoint) > 0.0
	QHList_Init(&face.conflictListHead.link)

	edge1.prev = edge3
	edge1.next = edge2
	edge1.origin = v1
	edge1.face = face
	edge1.twin = nil

	edge2.prev = edge1
	edge2.next = edge3
	edge2.origin = v2
	edge2.face = face
	edge2.twin = nil

	edge3.prev = edge2
	edge3.next = edge1
	edge3.origin = v3
	edge3.face = face
	edge3.twin = nil

	return face
}

// Remove face and add to free list.
@(private = "file")
HullBuilder_RetireFace :: proc(b: ^HullBuilder, face: ^QHFace) {
	// Sometimes a cone face gets merged and never added to the list.
	if QHList_Contains(&face.link) {
		QHList_Remove(&face.link)
	}

	face.edge = nil
	// link.prev is already NULL after Remove (or was never set).
	VALIDATE(face.link.prev == nil)
	face.link.next = (^QHListNode)(b.faceFreeHead)
	b.faceFreeHead = face
}

@(private = "file")
BuildBounds :: proc(vertexCount: i32, vertices: [^]Vec3) -> AABB {
	bounds := BOUNDS3_EMPTY
	for i: i32 = 0; i < vertexCount; i += 1 {
		bounds.lowerBound = Min(bounds.lowerBound, vertices[i])
		bounds.upperBound = Max(bounds.upperBound, vertices[i])
	}
	return bounds
}

@(private = "file")
FindFarthestPointsAlongCardinalAxes :: proc(
	index1Out: ^i32,
	index2Out: ^i32,
	tolerance: f32,
	vertexCount: i32,
	vertexBase: [^]Vec3,
) {
	index1Out^ = NULL_INDEX
	index2Out^ = NULL_INDEX

	v0 := vertexBase[0]
	minPt := [3]Vec3{v0, v0, v0}
	maxPt := [3]Vec3{v0, v0, v0}

	minIndex := [3]i32{0, 0, 0}
	maxIndex := [3]i32{0, 0, 0}

	for i: i32 = 1; i < vertexCount; i += 1 {
		v := vertexBase[i]

		if v.x < minPt[AXIS_X].x {
			minPt[AXIS_X] = v
			minIndex[AXIS_X] = i
		} else if v.x > maxPt[AXIS_X].x {
			maxPt[AXIS_X] = v
			maxIndex[AXIS_X] = i
		}

		if v.y < minPt[AXIS_Y].y {
			minPt[AXIS_Y] = v
			minIndex[AXIS_Y] = i
		} else if v.y > maxPt[AXIS_Y].y {
			maxPt[AXIS_Y] = v
			maxIndex[AXIS_Y] = i
		}

		if v.z < minPt[AXIS_Z].z {
			minPt[AXIS_Z] = v
			minIndex[AXIS_Z] = i
		} else if v.z > maxPt[AXIS_Z].z {
			maxPt[AXIS_Z] = v
			maxIndex[AXIS_Z] = i
		}
	}

	distance: Vec3
	distance.x = maxPt[AXIS_X].x - minPt[AXIS_X].x
	distance.y = maxPt[AXIS_Y].y - minPt[AXIS_Y].y
	distance.z = maxPt[AXIS_Z].z - minPt[AXIS_Z].z

	distanceArray := [3]f32{distance.x, distance.y, distance.z}
	maxElement := MaxElementIndex(distance)

	if distanceArray[maxElement] > 2.0 * tolerance {
		index1Out^ = minIndex[maxElement]
		index2Out^ = maxIndex[maxElement]
	}
}

@(private = "file")
FindFarthestPointFromLine :: proc(
	index1: i32,
	index2: i32,
	tolerance: f32,
	vertexCount: i32,
	vertexBase: [^]Vec3,
) -> i32 {
	a := vertexBase[index1]
	b := vertexBase[index2]

	// |ap x ab|^2 / |ab|^2 is the squared perpendicular distance from p to the line.
	// Compares against (2 * tolerance)^2
	ab := Sub(b, a)
	abLengthSqr := Dot(ab, ab)
	assert(abLengthSqr > 0.0)

	invAbLengthSqr := 1.0 / abLengthSqr
	maxDistanceSqr := 4.0 * tolerance * tolerance
	maxIndex: i32 = NULL_INDEX

	for i: i32 = 0; i < vertexCount; i += 1 {
		if i == index1 || i == index2 {
			continue
		}

		ap := Sub(vertexBase[i], a)
		cross := Cross(ap, ab)
		distanceSqr := Dot(cross, cross) * invAbLengthSqr
		if distanceSqr > maxDistanceSqr {
			maxDistanceSqr = distanceSqr
			maxIndex = i
		}
	}

	return maxIndex
}

@(private = "file")
FindFarthestPointFromPlane :: proc(
	index1: i32,
	index2: i32,
	index3: i32,
	tolerance: f32,
	vertexCount: i32,
	vertexBase: [^]Vec3,
) -> i32 {
	a := vertexBase[index1]
	b := vertexBase[index2]
	c := vertexBase[index3]

	plane := MakePlaneFromPoints(a, b, c)

	maxDistance := 2.0 * tolerance
	maxIndex: i32 = NULL_INDEX

	for i: i32 = 0; i < vertexCount; i += 1 {
		if i == index1 || i == index2 || i == index3 {
			continue
		}

		distance := AbsFloat(PlaneSeparation(plane, vertexBase[i]))
		if distance > maxDistance {
			maxDistance = distance
			maxIndex = i
		}
	}

	return maxIndex
}

@(private = "file")
IsEdgeConvex :: proc(edge: ^QHHalfEdge, tolerance: f32) -> bool {
	distance := PlaneSeparation(edge.face.plane, edge.twin.face.centroid)
	return distance < -tolerance
}

@(private = "file")
IsEdgeConcave :: proc(edge: ^QHHalfEdge, tolerance: f32) -> bool {
	distance := PlaneSeparation(edge.face.plane, edge.twin.face.centroid)
	return distance > tolerance
}

@(private = "file")
VertexCountOfFace :: proc(face: ^QHFace) -> i32 {
	count: i32 = 0
	edge := face.edge
	for {
		count += 1
		edge = edge.next
		if edge == face.edge {
			break
		}
	}

	return count
}

@(private = "file")
LinkFace :: proc(face: ^QHFace, index: i32, twin: ^QHHalfEdge) {
	assert(face != twin.face)

	index := index
	edge := face.edge
	for index > 0 {
		index -= 1
		assert(edge.face == face)
		edge = edge.next
	}

	assert(edge != twin)
	edge.twin = twin
	twin.twin = edge
}

@(private = "file")
LinkFaces :: proc(face1: ^QHFace, index1: i32, face2: ^QHFace, index2: i32) {
	assert(face1 != face2)

	index1 := index1
	index2 := index2

	edge1 := face1.edge
	for index1 > 0 {
		index1 -= 1
		edge1 = edge1.next
	}

	edge2 := face2.edge
	for index2 > 0 {
		index2 -= 1
		edge2 = edge2.next
	}

	assert(edge1 != edge2)
	edge1.twin = edge2
	edge2.twin = edge1
}

@(private = "file")
NewellPlane :: proc(face: ^QHFace) {
	count: i32 = 0
	centroid := Vec3_zero
	normal := Vec3_zero

	edge := face.edge
	assert(edge.face == face)

	// Use the first vertex as the origin to reduce round-off
	origin := edge.origin.position

	for {
		twin := edge.twin
		assert(twin.twin == edge)

		v1 := Sub(edge.origin.position, origin)
		v2 := Sub(twin.origin.position, origin)

		count += 1
		centroid = Add(centroid, v1)
		normal.x += (v1.y - v2.y) * (v1.z + v2.z)
		normal.y += (v1.z - v2.z) * (v1.x + v2.x)
		normal.z += (v1.x - v2.x) * (v1.y + v2.y)

		edge = edge.next
		if edge == face.edge {
			break
		}
	}

	assert(count > 0)
	centroid = MulSV(1.0 / f32(count), centroid)
	centroid = Add(centroid, origin)

	length := Length(normal)
	VALIDATE(length > 0.0)
	normal = MulSV(1.0 / length, normal)

	face.centroid = centroid
	face.plane = MakePlaneFromNormalAndPoint(normal, centroid)
	face.area = 0.5 * length
}

// unified: C compiles this only #if B3_DEBUG; Odin checks it always and calls it `when B3_DEBUG`.
@(private = "file")
CheckConsistency :: proc(face: ^QHFace) -> bool {
	if face.mark == MARK_DELETE {
		return false
	}

	if VertexCountOfFace(face) < 3 {
		return false
	}

	edge := face.edge

	for {
		twin := edge.twin

		if twin == nil {
			return false
		}
		if twin.face == nil {
			return false
		}
		if twin.face == face {
			return false
		}
		if twin.face.mark == MARK_DELETE {
			return false
		}
		if twin.twin != edge {
			return false
		}
		if edge.next.origin != twin.origin {
			return false
		}
		if edge.origin != twin.next.origin {
			return false
		}
		if edge.face != face {
			return false
		}

		edge = edge.next
		if edge == face.edge {
			break
		}
	}

	return true
}
// unified end

@(private = "file")
HullBuilder_ComputeTolerance :: proc(b: ^HullBuilder, pointCount: i32, points: [^]Vec3) {
	bounds := BuildBounds(pointCount, points)
	maxAbs := Max(Abs(bounds.lowerBound), Abs(bounds.upperBound))

	maxSum := maxAbs.x + maxAbs.y + maxAbs.z
	maxCoord := MaxFloat(maxAbs.x, MaxFloat(maxAbs.y, maxAbs.z))
	maxDistance := MinFloat(SQRT3 * maxCoord, maxSum)

	tolerance := (3.0 * maxDistance * 1.01 + maxCoord) * FLT_EPSILON

	b.tolerance = tolerance
	b.minRadius = 4.0 * b.tolerance
	b.minOutside = 2.0 * b.minRadius
	assert(b.minRadius < b.minOutside + 3.0 * FLT_EPSILON)
}

@(private = "file")
HullBuilder_BuildInitialHull :: proc(b: ^HullBuilder, pointCount: i32, points: [^]Vec3) -> bool {
	index1, index2: i32
	FindFarthestPointsAlongCardinalAxes(&index1, &index2, b.tolerance, pointCount, points)
	if index1 < 0 || index2 < 0 {
		return false
	}

	index3 := FindFarthestPointFromLine(index1, index2, b.tolerance, pointCount, points)
	if index3 < 0 {
		return false
	}

	index4 := FindFarthestPointFromPlane(index1, index2, index3, b.tolerance, pointCount, points)
	if index4 < 0 {
		return false
	}

	v1 := Sub(points[index1], points[index4])
	v2 := Sub(points[index2], points[index4])
	v3 := Sub(points[index3], points[index4])

	if ScalarTripleProduct(v1, v2, v3) < 0.0 {
		temp := index2
		index2 = index3
		index3 = temp
	}

	b.interiorPoint = Vec3_zero
	b.interiorPoint = Add(b.interiorPoint, points[index1])
	b.interiorPoint = Add(b.interiorPoint, points[index2])
	b.interiorPoint = Add(b.interiorPoint, points[index3])
	b.interiorPoint = Add(b.interiorPoint, points[index4])
	b.interiorPoint = MulSV(0.25, b.interiorPoint)

	vertex1 := HullBuilder_NewVertex(b, points[index1])
	QHList_PushBack(&b.vertexList.link, &vertex1.link)
	vertex2 := HullBuilder_NewVertex(b, points[index2])
	QHList_PushBack(&b.vertexList.link, &vertex2.link)
	vertex3 := HullBuilder_NewVertex(b, points[index3])
	QHList_PushBack(&b.vertexList.link, &vertex3.link)
	vertex4 := HullBuilder_NewVertex(b, points[index4])
	QHList_PushBack(&b.vertexList.link, &vertex4.link)

	face1 := HullBuilder_NewFace(b, vertex1, vertex2, vertex3)
	QHList_PushBack(&b.faceList.link, &face1.link)
	face2 := HullBuilder_NewFace(b, vertex4, vertex2, vertex1)
	QHList_PushBack(&b.faceList.link, &face2.link)
	face3 := HullBuilder_NewFace(b, vertex4, vertex3, vertex2)
	QHList_PushBack(&b.faceList.link, &face3.link)
	face4 := HullBuilder_NewFace(b, vertex4, vertex1, vertex3)
	QHList_PushBack(&b.faceList.link, &face4.link)

	LinkFaces(face1, 0, face2, 1)
	LinkFaces(face1, 1, face3, 1)
	LinkFaces(face1, 2, face4, 1)

	LinkFaces(face2, 0, face3, 2)
	LinkFaces(face3, 0, face4, 2)
	LinkFaces(face4, 0, face2, 2)

	when B3_DEBUG {
		assert(CheckConsistency(face1))
		assert(CheckConsistency(face2))
		assert(CheckConsistency(face3))
		assert(CheckConsistency(face4))
	}

	for index: i32 = 0; index < pointCount; index += 1 {
		if index == index1 || index == index2 || index == index3 || index == index4 {
			continue
		}

		point := points[index]

		maxDistance := b.minOutside
		maxFace: ^QHFace = nil

		for node := b.faceList.link.next; node != &b.faceList.link; node = node.next {
			face := (^QHFace)(node)
			distance := PlaneSeparation(face.plane, point)
			if distance > maxDistance {
				maxDistance = distance
				maxFace = face
			}
		}

		if maxFace != nil {
			vertex := HullBuilder_NewVertex(b, point)
			vertex.conflictFace = maxFace
			QHList_PushBack(&maxFace.conflictListHead.link, &vertex.link)
			if maxDistance > maxFace.maxConflictDistance {
				maxFace.maxConflictDistance = maxDistance
				maxFace.maxConflict = vertex
			}
		}
	}

	return true
}

// Recompute the farthest-conflict cache after a face's plane changes.
// Walks the existing conflict list once; cost is bounded by that list, not the global pool.
@(private = "file")
HullBuilder_RecacheConflicts :: proc(face: ^QHFace, minOutside: f32) {
	maxVertex: ^QHVertex = nil
	maxDistance := minOutside

	for node := face.conflictListHead.link.next; node != &face.conflictListHead.link; node = node.next {
		vertex := (^QHVertex)(node)
		distance := PlaneSeparation(face.plane, vertex.position)
		if distance > maxDistance {
			maxDistance = distance
			maxVertex = vertex
		}
	}

	face.maxConflict = maxVertex
	face.maxConflictDistance = maxDistance
}

@(private = "file")
HullBuilder_NextConflictVertex :: proc(b: ^HullBuilder) -> ^QHVertex {
	maxVertex: ^QHVertex = nil
	maxDistance := b.minOutside

	for faceNode := b.faceList.link.next; faceNode != &b.faceList.link; faceNode = faceNode.next {
		face := (^QHFace)(faceNode)
		if face.maxConflict != nil && face.maxConflictDistance > maxDistance {
			maxDistance = face.maxConflictDistance
			maxVertex = face.maxConflict
		}
	}

	return maxVertex
}

// Move every conflict vertex of `face` onto the orphaned list and clear their conflictFace.
@(private = "file")
HullBuilder_DrainConflictList :: proc(b: ^HullBuilder, face: ^QHFace) {
	node := face.conflictListHead.link.next
	for node != &face.conflictListHead.link {
		orphan := (^QHVertex)(node)
		node = node.next

		orphan.conflictFace = nil
		QHList_Remove(&orphan.link)
		QHList_PushBack(&b.orphanedList.link, &orphan.link)
	}
	assert(LIST_EMPTY(&face.conflictListHead.link))
}

// Mark a face for deletion, drain its conflict list, and populate a fresh DFS frame for it.
// `entryEdge` is the half-edge in `face` whose twin lies in the just-deleted parent face, or
// NULL for the seed. The frame skips `entryEdge` on recursive entries (it would be ignored
// anyway since the parent's mark is now DELETE, but skipping saves one iteration).
@(private = "file")
HullBuilder_EnterHorizonFace :: proc(b: ^HullBuilder, face: ^QHFace, entryEdge: ^QHHalfEdge, frameOut: ^HorizonFrame) {
	face.mark = MARK_DELETE
	HullBuilder_DrainConflictList(b, face)

	frameOut.face = face
	frameOut.started = false
	if entryEdge != nil {
		frameOut.startEdge = entryEdge
		frameOut.edge = entryEdge.next
	} else {
		frameOut.startEdge = face.edge
		frameOut.edge = face.edge
	}
}

@(private = "file")
HullBuilder_BuildHorizon :: proc(b: ^HullBuilder, apex: ^QHVertex, seed: ^QHFace) {
	stack := b.horizonStack
	top: i32 = 0

	assert(top < b.horizonStackCapacity)
	HullBuilder_EnterHorizonFace(b, seed, nil, &stack[top])
	top += 1

	for top > 0 {
		f := &stack[top - 1]

		if f.started && f.edge == f.startEdge {
			top -= 1
			continue
		}
		f.started = true

		edge := f.edge
		twin := edge.twin
		f.edge = edge.next

		if twin.face.mark != MARK_VISIBLE {
			continue
		}

		distance := PlaneSeparation(twin.face.plane, apex.position)
		if distance > b.minRadius {
			assert(top < b.horizonStackCapacity)
			HullBuilder_EnterHorizonFace(b, twin.face, twin, &stack[top])
			top += 1
		} else {
			assert(b.horizonCount < b.horizonCapacity)
			b.horizon[b.horizonCount] = edge
			b.horizonCount += 1
		}
	}
}

@(private = "file")
HullBuilder_BuildCone :: proc(b: ^HullBuilder, apex: ^QHVertex) {
	for i: i32 = 0; i < b.horizonCount; i += 1 {
		edge := b.horizon[i]
		assert(edge.twin.twin == edge)

		face := HullBuilder_NewFace(b, apex, edge.origin, edge.twin.origin)
		assert(b.coneCount < b.coneCapacity)
		b.cone[b.coneCount] = face
		b.coneCount += 1

		LinkFace(face, 1, edge.twin)
	}

	face1 := b.cone[b.coneCount - 1]
	for i: i32 = 0; i < b.coneCount; i += 1 {
		face2 := b.cone[i]
		LinkFaces(face1, 2, face2, 0)
		face1 = face2
	}
}

// Retire half-edges in the half-open ring range [begin, end) by pushing each onto edgeFreeHead.
// The caller has already detached these edges from the live face ring (rewire happens before
// destroy), so they are unreachable from the live hull.
@(private = "file")
HullBuilder_DestroyEdges :: proc(b: ^HullBuilder, begin: ^QHHalfEdge, end: ^QHHalfEdge) {
	edge := begin
	for edge != end {
		next := edge.next
		HullBuilder_RetireEdge(b, edge)
		edge = next
	}
}

@(private = "file")
HullBuilder_ConnectEdges :: proc(b: ^HullBuilder, prev: ^QHHalfEdge, next: ^QHHalfEdge) {
	assert(prev != next)
	assert(prev.face == next.face)

	// If both shared neighbors are the same face, prev and next together would orphan that face.
	if prev.twin.face == next.twin.face {
		// next is redundant.
		if next.face.edge == next {
			next.face.edge = prev
		}

		twin: ^QHHalfEdge
		if VertexCountOfFace(prev.twin.face) == 3 {
			// Capture all 3 half-edges of the dead triangle before the rewire overwrites prev->twin.
			deadEdge0 := prev.twin // prev->twin (will be rewired below)
			deadEdge1 := next.twin // next->twin
			deadEdge2 := next.twin.prev // third edge of the dead triangle

			twin = deadEdge2.twin
			assert(twin.face.mark != MARK_DELETE)

			opposingFace := prev.twin.face
			opposingFace.mark = MARK_DELETE
			assert(b.mergedFacesCount < b.mergedFacesCapacity)
			b.mergedFaces[b.mergedFacesCount] = opposingFace
			b.mergedFacesCount += 1

			prev.next = next.next
			prev.next.prev = prev

			prev.twin = twin
			twin.twin = prev

			// Drop the redundant vertex (slot abandoned in the bump allocator).
			QHList_Remove(&next.origin.link)

			// Retire the 3 half-edges of the dead triangle now that the rewire is complete.
			HullBuilder_RetireEdge(b, deadEdge0)
			HullBuilder_RetireEdge(b, deadEdge1)
			HullBuilder_RetireEdge(b, deadEdge2)
		} else {
			twin = next.twin

			if twin.face.edge == prev.twin {
				twin.face.edge = twin
			}

			twin.next = prev.twin.next
			twin.next.prev = twin
			// prev->twin slot is retired to the edge free list.
			HullBuilder_RetireEdge(b, prev.twin)

			prev.next = next.next
			prev.next.prev = prev

			prev.twin = twin
			twin.twin = prev

			// Drop the redundant vertex (slot abandoned in the bump allocator).
			QHList_Remove(&next.origin.link)
		}

		// Twin->face changed shape; recompute its plane and refresh its cached max conflict.
		NewellPlane(twin.face)
		HullBuilder_RecacheConflicts(twin.face, b.minOutside)
	} else {
		prev.next = next
		next.prev = prev
	}
}

@(private = "file")
HullBuilder_AbsorbFaces :: proc(b: ^HullBuilder, face: ^QHFace) {
	for i: i32 = 0; i < b.mergedFacesCount; i += 1 {
		assert(b.mergedFaces[i].mark == MARK_DELETE)
		head := &b.mergedFaces[i].conflictListHead.link

		node := head.next
		for node != head {
			vertex := (^QHVertex)(node)
			node = node.next

			QHList_Remove(&vertex.link)

			distance := PlaneSeparation(face.plane, vertex.position)
			if distance > b.minOutside {
				QHList_PushBack(&face.conflictListHead.link, &vertex.link)
				vertex.conflictFace = face
				if distance > face.maxConflictDistance {
					face.maxConflictDistance = distance
					face.maxConflict = vertex
				}
			} else {
				QHList_PushBack(&b.orphanedList.link, &vertex.link)
				vertex.conflictFace = nil
			}
		}

		assert(LIST_EMPTY(head))

		// Conflict list is now drained. Retire this face to the free list.
		HullBuilder_RetireFace(b, b.mergedFaces[i])
	}
}

@(private = "file")
HullBuilder_ConnectFaces :: proc(b: ^HullBuilder, edge: ^QHHalfEdge) {
	face := edge.face

	twin := edge.twin

	edgePrev := edge.prev
	edgeNext := edge.next
	twinPrev := twin.prev
	twinNext := twin.next

	for edgePrev.twin.face == twin.face {
		assert(edgePrev.twin == twinNext)
		assert(twinNext.twin == edgePrev)

		edgePrev = edgePrev.prev
		twinNext = twinNext.next
	}
	assert(edgePrev.face != twinNext.face)

	for edgeNext.twin.face == twin.face {
		assert(edgeNext.twin == twinPrev)
		assert(twinPrev.twin == edgeNext)

		edgeNext = edgeNext.next
		twinPrev = twinPrev.prev
	}
	assert(edgeNext.face != twinPrev.face)

	face.edge = edgePrev

	// Discard opposing face. mergedFaces is single-buffered: ConnectFaces does not nest.
	b.mergedFacesCount = 0
	assert(b.mergedFacesCount < b.mergedFacesCapacity)
	b.mergedFaces[b.mergedFacesCount] = twin.face
	b.mergedFacesCount += 1
	twin.face.mark = MARK_DELETE
	twin.face.edge = nil

	for absorbed := twinNext; absorbed != twinPrev.next; absorbed = absorbed.next {
		absorbed.face = face
	}

	HullBuilder_DestroyEdges(b, edgePrev.next, edgeNext)
	HullBuilder_DestroyEdges(b, twinPrev.next, twinNext)

	HullBuilder_ConnectEdges(b, edgePrev, twinNext)
	HullBuilder_ConnectEdges(b, twinPrev, edgeNext)

	NewellPlane(face)
	// Existing conflicts now have stale distances under the new plane; AbsorbFaces will then
	// add more incrementally and update the cache as it goes.
	HullBuilder_RecacheConflicts(face, b.minOutside)
	when B3_DEBUG {
		assert(CheckConsistency(face))
	}

	HullBuilder_AbsorbFaces(b, face)
}

@(private = "file")
HullBuilder_MergeConcave :: proc(b: ^HullBuilder, face: ^QHFace) -> bool {
	edge := face.edge

	for {
		twin := edge.twin

		if IsEdgeConcave(edge, b.minRadius) || IsEdgeConcave(twin, b.minRadius) {
			HullBuilder_ConnectFaces(b, edge)
			return true
		}

		edge = edge.next
		if edge == face.edge {
			break
		}
	}

	return false
}

@(private = "file")
HullBuilder_MergeCoplanar :: proc(b: ^HullBuilder, face: ^QHFace) -> bool {
	edge := face.edge

	for {
		twin := edge.twin

		if !IsEdgeConvex(edge, b.minRadius) || !IsEdgeConvex(twin, b.minRadius) {
			HullBuilder_ConnectFaces(b, edge)
			return true
		}

		edge = edge.next
		if edge == face.edge {
			break
		}
	}

	return false
}

@(private = "file")
HullBuilder_MergeFaces :: proc(b: ^HullBuilder) {
	for i: i32 = 0; i < b.coneCount; i += 1 {
		face := b.cone[i]
		if face.mark == MARK_VISIBLE && face.flipped {
			face.flipped = false

			bestArea: f32 = 0
			bestEdge: ^QHHalfEdge = nil

			edge := face.edge
			for {
				twin := edge.twin
				area := twin.face.area
				if area > bestArea {
					bestArea = area
					bestEdge = edge
				}

				edge = edge.next
				if edge == face.edge {
					break
				}
			}

			assert(bestEdge != nil)
			HullBuilder_ConnectFaces(b, bestEdge)
		}
	}

	for i: i32 = 0; i < b.coneCount; i += 1 {
		face := b.cone[i]
		if face.mark == MARK_VISIBLE {
			for HullBuilder_MergeConcave(b, face) {
			}
		}
	}

	for i: i32 = 0; i < b.coneCount; i += 1 {
		face := b.cone[i]
		if face.mark == MARK_VISIBLE {
			for HullBuilder_MergeCoplanar(b, face) {
			}
		}
	}
}

@(private = "file")
HullBuilder_ResolveVertices :: proc(b: ^HullBuilder) {
	node := b.orphanedList.link.next
	for node != &b.orphanedList.link {
		vertex := (^QHVertex)(node)
		node = node.next
		QHList_Remove(&vertex.link)

		maxDistance := b.minOutside
		maxFace: ^QHFace = nil

		for i: i32 = 0; i < b.coneCount; i += 1 {
			if b.cone[i].mark == MARK_VISIBLE {
				distance := PlaneSeparation(b.cone[i].plane, vertex.position)
				if distance > maxDistance {
					maxDistance = distance
					maxFace = b.cone[i]
				}
			}
		}

		if maxFace != nil {
			assert(maxFace.mark == MARK_VISIBLE)
			QHList_PushBack(&maxFace.conflictListHead.link, &vertex.link)
			vertex.conflictFace = maxFace
			if maxDistance > maxFace.maxConflictDistance {
				maxFace.maxConflictDistance = maxDistance
				maxFace.maxConflict = vertex
			}
		}
		// Otherwise: vertex is interior to the hull. Its slot in the bump pool is abandoned.
	}

	assert(LIST_EMPTY(&b.orphanedList.link))
}

@(private = "file")
HullBuilder_ResolveFaces :: proc(b: ^HullBuilder) {
	// Splice deleted faces out of the face list. Faces already retired by AbsorbFaces are no
	// longer on faceList, so guard with b3QHList_Contains before removing.
	node := b.faceList.link.next
	for node != &b.faceList.link {
		face := (^QHFace)(node)
		node = node.next

		if face.mark == MARK_DELETE && QHList_Contains(&face.link) {
			assert(LIST_EMPTY(&face.conflictListHead.link))

			// Each half-edge is owned by exactly one face, so ring walks over the
			// dead region retire every interior edge exactly once. Merge deleted
			// faces are already off the face list.
			start := face.edge
			edge := start
			for {
				next := edge.next
				HullBuilder_RetireEdge(b, edge)
				edge = next
				if edge == start {
					break
				}
			}

			HullBuilder_RetireFace(b, face)
		}
	}

	for i: i32 = 0; i < b.coneCount; i += 1 {
		face := b.cone[i]
		if face.mark == MARK_DELETE {
			continue
		}
		QHList_PushBack(&b.faceList.link, &face.link)
	}
}

@(private = "file")
HullBuilder_AddVertexToHull :: proc(b: ^HullBuilder, vertex: ^QHVertex) {
	face := vertex.conflictFace
	vertex.conflictFace = nil
	QHList_Remove(&vertex.link)
	QHList_PushBack(&b.vertexList.link, &vertex.link)

	b.horizonCount = 0
	HullBuilder_BuildHorizon(b, vertex, face)
	assert(b.horizonCount >= 3)

	b.coneCount = 0
	HullBuilder_BuildCone(b, vertex)
	assert(b.coneCount >= 3)

	HullBuilder_MergeFaces(b)
	HullBuilder_ResolveVertices(b)
	HullBuilder_ResolveFaces(b)
}

@(private = "file")
HullBuilder_CleanHull :: proc(b: ^HullBuilder, origin: Vec3) {
	faceCount: i32 = 0
	halfEdgeCount: i32 = 0

	for faceNode := b.faceList.link.next; faceNode != &b.faceList.link; faceNode = faceNode.next {
		face := (^QHFace)(faceNode)
		edge := face.edge

		for {
			edge.origin.reachable = true
			edge = edge.next
			halfEdgeCount += 1
			if edge == face.edge {
				break
			}
		}

		face.plane.offset += Dot(face.plane.normal, origin)
		face.centroid = Add(face.centroid, origin)
		faceCount += 1
	}

	vertexCount: i32 = 0
	node := b.vertexList.link.next
	for node != &b.vertexList.link {
		vertex := (^QHVertex)(node)
		node = node.next

		if !vertex.reachable {
			QHList_Remove(&vertex.link)
		} else {
			vertex.position = Add(vertex.position, origin)
			vertexCount += 1
		}
	}

	b.interiorPoint = Add(b.interiorPoint, origin)

	b.finalVertexCount = vertexCount
	b.finalHalfEdgeCount = halfEdgeCount
	b.finalFaceCount = faceCount
}

// unified: C compiles this only #if B3_DEBUG; Odin checks it always and calls it `when B3_DEBUG`.
@(private = "file")
HullBuilder_IsConsistent :: proc(b: ^HullBuilder) -> bool {
	v := b.finalVertexCount
	e := b.finalHalfEdgeCount / 2
	f := b.finalFaceCount

	if v - e + f != 2 {
		return false
	}

	for faceNode := b.faceList.link.next; faceNode != &b.faceList.link; faceNode = faceNode.next {
		face := (^QHFace)(faceNode)
		if face.edge.face != face {
			return false
		}

		if !CheckConsistency(face) {
			return false
		}

		if PlaneSeparation(face.plane, b.interiorPoint) > 0 {
			return false
		}

		if face.mark != MARK_VISIBLE {
			return false
		}

		edge := face.edge

		for {
			if edge.next.origin != edge.twin.origin {
				return false
			}
			if edge.prev.next != edge {
				return false
			}
			if edge.next.prev != edge {
				return false
			}
			if edge.twin.twin != edge {
				return false
			}
			if edge.face != face {
				return false
			}
			if DistanceSquared(edge.origin.position, edge.twin.origin.position) < 1000.0 * FLT_MIN {
				return false
			}

			edge = edge.next
			if edge == face.edge {
				break
			}
		}
	}

	return true
}
// unified end

@(private = "file")
HullBuilder_HasHull :: proc(b: ^HullBuilder) -> bool {
	v := b.finalVertexCount
	e := b.finalHalfEdgeCount / 2
	f := b.finalFaceCount
	return v - e + f == 2 && f >= 4
}

// Build the entire hull. Returns true iff the result satisfies Euler's identity.
@(private = "file")
HullBuilder_Construct :: proc(
	b: ^HullBuilder,
	points: [^]Vec3,
	pointCount: i32,
	maxVertexCount: i32,
	origin: Vec3,
	shiftedPoints: [^]Vec3,
) -> bool {
	if pointCount < 4 {
		return false
	}

	for i: i32 = 0; i < pointCount; i += 1 {
		shiftedPoints[i] = Sub(points[i], origin)
	}

	HullBuilder_ComputeTolerance(b, pointCount, shiftedPoints)
	haveInitialHull := HullBuilder_BuildInitialHull(b, pointCount, shiftedPoints)
	if haveInitialHull == false {
		return false
	}

	budget := ClampInt(maxVertexCount - 4, 0, HULL_MAX_COUNT - 4)

	vertex := HullBuilder_NextConflictVertex(b)
	for vertex != nil && budget > 0 {
		HullBuilder_AddVertexToHull(b, vertex)
		vertex = HullBuilder_NextConflictVertex(b)
		budget -= 1
	}

	HullBuilder_CleanHull(b, origin)

	when B3_DEBUG {
		assert(HullBuilder_IsConsistent(b))
	}

	return HullBuilder_HasHull(b)
}

@(private = "file")
HullWorkSizes :: struct {
	// Input point count
	N:                    i32,
	// Output point limit
	M:                    i32,
	vertexCapacity:       i32,
	edgeCapacity:         i32,
	faceCapacity:         i32,
	horizonCapacity:      i32,
	coneCapacity:         i32,
	mergedFacesCapacity:  i32,
	horizonStackCapacity: i32,
	totalBytes:           int,
	offsetVertex:         int,
	offsetEdge:           int,
	offsetFace:           int,
	offsetHorizon:        int,
	offsetCone:           int,
	offsetMergedFaces:    int,
	offsetHorizonStack:   int,
	offsetShiftedPoints:  int,
}

@(private = "file")
ComputeHullWorkSizes :: proc(pointCount: i32, clampedMaxCount: i32) -> HullWorkSizes {
	s: HullWorkSizes
	s.N = pointCount
	s.M = clampedMaxCount

	// Vertices: 4 initial hull vertices + at most one per remaining input point. No free list.
	s.vertexCapacity = pointCount + 4

	// Edges and faces use free-list recycling; capacity is proportional to live hull size.
	// edgeCapacity: peak is ~twice live edges plus cone edges. Minimum 48.
	s.edgeCapacity = MaxInt(48, 24 * s.M - 48)

	// faceCapacity: peak intermediate state live faces (<=2*M-4) plus full cone (<=3*M-6). Minimum 16.
	s.faceCapacity = MaxInt(16, 5 * s.M - 10)

	// Horizon/cone bounded by current half-edge count. Merged faces by face count.
	s.horizonCapacity = MaxInt(6, 3 * s.M - 6)
	s.coneCapacity = s.horizonCapacity
	s.mergedFacesCapacity = MaxInt(4, 2 * s.M - 4)

	// Horizon DFS depth is bounded by the number of live faces (Euler: <=2*M-4).
	s.horizonStackCapacity = MaxInt(4, 2 * s.M - 4)

	offset := 0

	s.offsetVertex = offset
	offset = AlignUp8(offset + int(s.vertexCapacity) * size_of(QHVertex))

	s.offsetEdge = offset
	offset = AlignUp8(offset + int(s.edgeCapacity) * size_of(QHHalfEdge))

	s.offsetFace = offset
	offset = AlignUp8(offset + int(s.faceCapacity) * size_of(QHFace))

	s.offsetHorizon = offset
	offset = AlignUp8(offset + int(s.horizonCapacity) * size_of(^QHHalfEdge))

	s.offsetCone = offset
	offset = AlignUp8(offset + int(s.coneCapacity) * size_of(^QHFace))

	s.offsetMergedFaces = offset
	offset = AlignUp8(offset + int(s.mergedFacesCapacity) * size_of(^QHFace))

	s.offsetHorizonStack = offset
	offset = AlignUp8(offset + int(s.horizonStackCapacity) * size_of(HorizonFrame))

	s.offsetShiftedPoints = offset
	offset += int(pointCount) * size_of(Vec3)

	s.totalBytes = offset
	return s
}

@(private = "file")
HullBuilder_Init :: proc(b: ^HullBuilder, mem_: [^]u8, s: ^HullWorkSizes) {
	mem.zero(b, size_of(b^))
	QHList_Init(&b.orphanedList.link)
	QHList_Init(&b.vertexList.link)
	QHList_Init(&b.faceList.link)

	b.vertexBase = cast([^]QHVertex)(&mem_[s.offsetVertex])
	b.vertexCapacity = s.vertexCapacity

	b.edgeBase = cast([^]QHHalfEdge)(&mem_[s.offsetEdge])
	b.edgeCapacity = s.edgeCapacity

	b.faceBase = cast([^]QHFace)(&mem_[s.offsetFace])
	b.faceCapacity = s.faceCapacity

	b.horizon = cast([^]^QHHalfEdge)(&mem_[s.offsetHorizon])
	b.horizonCapacity = s.horizonCapacity

	b.cone = cast([^]^QHFace)(&mem_[s.offsetCone])
	b.coneCapacity = s.coneCapacity

	b.mergedFaces = cast([^]^QHFace)(&mem_[s.offsetMergedFaces])
	b.mergedFacesCapacity = s.mergedFacesCapacity

	b.horizonStack = cast([^]HorizonFrame)(&mem_[s.offsetHorizonStack])
	b.horizonStackCapacity = s.horizonStackCapacity
}

@(private = "file")
GetHullPointsWrite :: proc(hull: ^HullData) -> [^]Vec3 {
	if hull.pointOffset == 0 {
		return nil
	}
	return cast([^]Vec3)(uintptr(hull) + uintptr(hull.pointOffset))
}

@(private = "file")
GetHullPlanesWrite :: proc(hull: ^HullData) -> [^]Plane {
	if hull.planeOffset == 0 {
		return nil
	}
	return cast([^]Plane)(uintptr(hull) + uintptr(hull.planeOffset))
}

@(private = "file")
GetHullVerticesWrite :: proc(hull: ^HullData) -> [^]HullVertex {
	if hull.vertexOffset == 0 {
		return nil
	}
	return cast([^]HullVertex)(uintptr(hull) + uintptr(hull.vertexOffset))
}

@(private = "file")
GetHullEdgesWrite :: proc(hull: ^HullData) -> [^]HullHalfEdge {
	if hull.edgeOffset == 0 {
		return nil
	}
	return cast([^]HullHalfEdge)(uintptr(hull) + uintptr(hull.edgeOffset))
}

@(private = "file")
GetHullSoaVerticesWrite :: proc(hull: ^HullData) -> [^]f32 {
	if hull.soaVertexOffset == 0 {
		return nil
	}
	return cast([^]f32)(uintptr(hull) + uintptr(hull.soaVertexOffset))
}

@(private = "file")
GetHullSoaNormalsWrite :: proc(hull: ^HullData) -> [^]f32 {
	if hull.soaNormalOffset == 0 {
		return nil
	}
	return cast([^]f32)(uintptr(hull) + uintptr(hull.soaNormalOffset))
}

@(private = "file")
UpdateHullEdgeCosines :: proc(hull: ^HullData) {
	edges := GetHullEdges(hull)
	planes := GetHullPlanes(hull)
	cosines := cast([^]f32)(uintptr(hull) + uintptr(hull.edgeCosineOffset))
	count := hull.edgeCount / 2

	for i: i32 = 0; i < count; i += 1 {
		cosines[i] = Dot(planes[edges[2 * i].face].normal, planes[edges[2 * i + 1].face].normal)
	}
}

FindHullSupportVertex :: proc(hull: ^HullData, direction: Vec3) -> i32 {
	bestIndex: i32 = NULL_INDEX
	bestDot := -max(f32)

	vertexCount := hull.vertexCount
	vx := GetHullSoaVertices(hull)

	soaVertexCount := GetHullSoaStride(vertexCount)
	vy := vx[soaVertexCount:]
	vz := vy[soaVertexCount:]

	dx := SplatW4(direction.x)
	dy := SplatW4(direction.y)
	dz := SplatW4(direction.z)
	four := SplatW4(4.0)
	index := SetW4(0.0, 1.0, 2.0, 3.0)
	bestDotW := SplatW4(-max(f32))
	bestIndexW := SplatW4(-1.0)

	// Keep this at 4 wide to avoid regressing 4 wide platforms.
	loopCount := (vertexCount + 3) & ~i32(3)
	for i: i32 = 0; i < loopCount; i += 4 {
		dot := AddW4(AddW4(MulW4(dx, LoadW4(vx[i:])), MulW4(dy, LoadW4(vy[i:]))), MulW4(dz, LoadW4(vz[i:])))
		mask := GreaterThanW4(dot, bestDotW)
		bestDotW = BlendW4(bestDotW, dot, mask)
		bestIndexW = BlendW4(bestIndexW, index, mask)
		index = AddW4(index, four)
	}

	dots: [4]f32
	indices: [4]f32
	StoreW4(&dots[0], bestDotW)
	StoreW4(&indices[0], bestIndexW)

	for lane := 0; lane < 4; lane += 1 {
		laneIndex := i32(indices[lane])
		if laneIndex >= 0 && (dots[lane] > bestDot || (dots[lane] == bestDot && laneIndex < bestIndex)) {
			bestIndex = laneIndex
			bestDot = dots[lane]
		}
	}

	assert(0 <= bestIndex && bestIndex < vertexCount)
	return bestIndex
}

FindHullSupportFace :: proc(hull: ^HullData, direction: Vec3) -> i32 {
	bestIndex: i32 = NULL_INDEX
	bestDot := -max(f32)

	faceCount := hull.faceCount
	planes := GetHullPlanes(hull)

	for index: i32 = 0; index < faceCount; index += 1 {
		dot := Dot(planes[index].normal, direction)
		if dot > bestDot {
			bestDot = dot
			bestIndex = index
		}
	}
	assert(bestIndex >= 0)

	return bestIndex
}

IsValidHull :: proc(hull: ^HullData) -> bool {
	when ENABLE_VALIDATION {
		if hull.version != HULL_VERSION {
			return false
		}

		v := hull.vertexCount
		e := hull.edgeCount / 2
		f := hull.faceCount

		if v - e + f != 2 {
			return false
		}

		vertices := GetHullVertices(hull)
		edges := GetHullEdges(hull)
		for index: i32 = 0; index < hull.vertexCount; index += 1 {
			vertex := &vertices[index]
			edge := &edges[vertex.edge]

			if i32(edge.origin) != index {
				return false
			}
		}

		for index: i32 = 0; index < hull.edgeCount; index += 2 {
			edge := &edges[index + 0]
			twin := &edges[index + 1]

			if i32(edge.twin) != index + 1 {
				return false
			}

			if i32(twin.twin) != index + 0 {
				return false
			}
		}

		faces := GetHullFaces(hull)
		planes := GetHullPlanes(hull)
		for faceIndex: i32 = 0; faceIndex < hull.faceCount; faceIndex += 1 {
			face := &faces[faceIndex]

			baseEdgeIndex := i32(face.edge)
			edge := &edges[baseEdgeIndex]

			plane := planes[faceIndex]
			if PlaneSeparation(plane, hull.center) >= 0.0 {
				return false
			}

			edgeIndex := baseEdgeIndex
			for {
				edge = &edges[edgeIndex]
				next := &edges[edge.next]
				twin := &edges[edge.twin]

				if i32(edge.face) != faceIndex {
					return false
				}

				if i32(twin.twin) != edgeIndex {
					return false
				}

				if next.origin != twin.origin {
					return false
				}

				edgeIndex = i32(edge.next)
				if edgeIndex == baseEdgeIndex {
					break
				}
			}
		}

		if hull.volume <= 0.0 {
			return false
		}

		if hull.surfaceArea <= 0.0 {
			return false
		}

		if hull.innerRadius <= 0.0 {
			return false
		}

		return true
	} else {
		_ = hull
		return true
	}
}

CreateCylinder :: proc(height: f32, radius: f32, yOffset: f32, sides: i32) -> ^HullData {
	assert(height > 0.0)
	assert(radius > 0.0)
	assert(3 <= sides && sides <= 32)

	pointCount := 2 * sides
	points := cast([^]Vec3)Alloc(int(pointCount) * size_of(Vec3))
	assert(points != nil)

	alpha: f32 = 0.0
	deltaAlpha := 2.0 * PI / f32(sides)

	for index: i32 = 0; index < sides; index += 1 {
		sinAlpha := Sin(alpha)
		cosAlpha := Cos(alpha)

		points[2 * index + 0] = Vec3{radius * cosAlpha, yOffset, radius * sinAlpha}
		points[2 * index + 1] = Vec3{radius * cosAlpha, yOffset + height, radius * sinAlpha}

		alpha += deltaAlpha
	}

	hull := CreateHull(points, pointCount, pointCount)
	assert(hull.vertexCount == pointCount)
	assert(hull.edgeCount == 6 * sides)
	assert(hull.faceCount == sides + 2)

	Free(points, int(pointCount) * size_of(Vec3))

	return hull
}

CreateCone :: proc(height: f32, radius1: f32, radius2: f32, slices: i32) -> ^HullData {
	assert(height > 0.0)
	assert(radius1 > 0.0)
	assert(radius2 > 0.0)
	assert(4 <= slices && slices <= 32)

	pointCount := 2 * slices
	points := cast([^]Vec3)Alloc(int(pointCount) * size_of(Vec3))
	assert(points != nil)

	alpha: f32 = 0.0
	deltaAlpha := 2.0 * PI / f32(slices)

	for index: i32 = 0; index < slices; index += 1 {
		sinAlpha := Sin(alpha)
		cosAlpha := Cos(alpha)

		points[2 * index + 0] = Vec3{radius1 * cosAlpha, 0.0, radius1 * sinAlpha}
		points[2 * index + 1] = Vec3{radius2 * cosAlpha, height, radius2 * sinAlpha}

		alpha += deltaAlpha
	}

	hull := CreateHull(points, pointCount, pointCount)
	assert(hull.vertexCount == pointCount)
	assert(hull.edgeCount == 6 * slices)
	assert(hull.faceCount == slices + 2)

	Free(points, int(pointCount) * size_of(Vec3))

	return hull
}

CreateRock :: proc(radius: f32) -> ^HullData {
	pointCount: i32 = 10

	// Golden ratio
	phi := (1.0 + intrinsics.sqrt(f32(5.0))) / 2.0

	// Fibonacci lattice
	points: [10]Vec3

	// Azimuthal angle
	theta := 2.0 * PI / phi

	cs := CosSin{1.0, 0.0}
	deltaCS := ComputeCosSin(theta)

	for i: i32 = 0; i < pointCount; i += 1 {
		// Z coordinate
		z := 1.0 - (2.0 * f32(i) + 1.0) / f32(pointCount)
		// Radius in xy-plane
		radius_XY := intrinsics.sqrt(1.0 - z * z)

		points[i].x = radius * radius_XY * cs.cosine
		points[i].y = radius * radius_XY * cs.sine
		points[i].z = radius * z

		cs0 := cs
		cs.cosine = deltaCS.cosine * cs0.cosine - deltaCS.sine * cs0.sine
		cs.sine = deltaCS.sine * cs0.cosine + deltaCS.cosine * cs0.sine
	}

	return CreateHull(&points[0], pointCount, pointCount)
}

// PEEL's BasicRandom. This is just for testing.
@(private = "file")
NextComplexHullRandom :: proc(state: ^u32) -> u32 {
	state^ = state^ * 2147001325 + 715136305
	return state^
}

@(private = "file")
ComplexHullRandomFloat :: proc(state: ^u32) -> f32 {
	return f32(NextComplexHullRandom(state) & 0xffff) / 65535.0 - 0.5
}

@(private = "file")
ComplexHullRandomDirection :: proc(state: ^u32) -> Vec3 {
	point: Vec3
	lengthSquared: f32
	for {
		point.x = ComplexHullRandomFloat(state)
		point.y = ComplexHullRandomFloat(state)
		point.z = ComplexHullRandomFloat(state)
		lengthSquared = Dot(point, point)
		if !(lengthSquared > 0.25) {
			break
		}
	}

	return Normalize(point)
}

CreateComplexHull :: proc(radius: f32) -> ^HullData {
	pointCount :: 32

	points: [pointCount]Vec3
	state: u32 = 42
	for i: i32 = 0; i < pointCount; i += 1 {
		points[i] = MulSV(radius, ComplexHullRandomDirection(&state))
	}

	return CreateHull(&points[0], pointCount, pointCount)
}

@(private = "file")
UpdateHullBounds :: proc(hull: ^HullData) {
	points := GetHullPoints(hull)
	vertexCount := hull.vertexCount

	assert(vertexCount > 0)
	bounds: AABB
	bounds.lowerBound = points[0]
	bounds.upperBound = points[0]

	for i: i32 = 1; i < vertexCount; i += 1 {
		p := points[i]
		bounds.lowerBound = Min(bounds.lowerBound, p)
		bounds.upperBound = Max(bounds.upperBound, p)
	}

	hull.aabb = bounds
}

// M. Kallay - "Computing the Moment of Inertia of a Solid Defined by a Triangle Mesh"
@(private = "file")
UpdateHullBulkProperties :: proc(hull: ^HullData) -> bool {
	points := GetHullPoints(hull)
	faces := GetHullFaces(hull)
	edges := GetHullEdges(hull)
	planes := GetHullPlanes(hull)

	area: f32 = 0.0
	volume: f32 = 0.0
	center := Vec3_zero

	// Use the first vertex to reduce round-off errors.
	origin := points[0]

	xx: f32 = 0.0
	xy: f32 = 0.0
	yy: f32 = 0.0
	xz: f32 = 0.0
	zz: f32 = 0.0
	yz: f32 = 0.0

	faceCount := hull.faceCount

	for faceIndex: i32 = 0; faceIndex < faceCount; faceIndex += 1 {
		face := &faces[faceIndex]
		edge1 := &edges[face.edge]
		edge2 := &edges[edge1.next]
		edge3 := &edges[edge2.next]

		assert(edge1 != edge3)
		assert(i32(edge1.origin) < hull.vertexCount)

		v1 := Sub(points[edge1.origin], origin)

		for {
			assert(i32(edge2.origin) < hull.vertexCount)
			assert(i32(edge3.origin) < hull.vertexCount)

			v2 := Sub(points[edge2.origin], origin)
			v3 := Sub(points[edge3.origin], origin)

			area += Length(Cross(Sub(v2, v1), Sub(v3, v1)))

			det := ScalarTripleProduct(v1, v2, v3)

			volume += det

			v4 := Add(v1, Add(v2, v3))
			center = Add(center, MulSV(det, v4))

			xx += det * (v1.x * v1.x + v2.x * v2.x + v3.x * v3.x + v4.x * v4.x)
			yy += det * (v1.y * v1.y + v2.y * v2.y + v3.y * v3.y + v4.y * v4.y)
			zz += det * (v1.z * v1.z + v2.z * v2.z + v3.z * v3.z + v4.z * v4.z)
			xy += det * (v1.x * v1.y + v2.x * v2.y + v3.x * v3.y + v4.x * v4.y)
			xz += det * (v1.x * v1.z + v2.x * v2.z + v3.x * v3.z + v4.x * v4.z)
			yz += det * (v1.y * v1.z + v2.y * v2.z + v3.y * v3.z + v4.y * v4.z)

			edge2 = edge3
			edge3 = &edges[edge3.next]
			if edge1 == edge3 {
				break
			}
		}
	}

	VALIDATE(volume > 0.0)

	localCenter := volume > 0.0 ? MulSV(0.25 / volume, center) : Vec3_zero
	center = Add(localCenter, origin)

	radius := max(f32)
	for faceIndex: i32 = 0; faceIndex < faceCount; faceIndex += 1 {
		plane := planes[faceIndex]
		distance := PlaneSeparation(plane, center)
		VALIDATE(distance < 0.0)

		radius = MinFloat(radius, -distance)
	}

	VALIDATE(0.0 < radius && radius < max(f32))

	inertia: Matrix3
	inertia.cx.x = yy + zz
	inertia.cy.x = -xy
	inertia.cz.x = -xz
	inertia.cx.y = -xy
	inertia.cy.y = xx + zz
	inertia.cz.y = -yz
	inertia.cx.z = -xz
	inertia.cy.z = -yz
	inertia.cz.z = xx + yy

	mass := volume / 6.0

	centralInertia := MulSM(1.0 / 120.0, inertia)
	centralInertia = SubMM(centralInertia, Steiner(mass, localCenter))

	hull.center = center
	hull.centralInertia = centralInertia
	hull.volume = mass
	hull.surfaceArea = 0.5 * area
	hull.innerRadius = radius

	if mass <= 0.0 {
		return false
	}

	if volume <= 0.0 {
		return false
	}

	if area <= 0.0 {
		return false
	}

	if radius <= 0.0 {
		return false
	}

	return true
}

CreateHull :: proc(points: [^]Vec3, pointCount: i32, maxVertexCount: i32) -> ^HullData {
	if pointCount < 4 {
		return nil
	}

	origin := points[0]
	clampedMaxCount := ClampInt(maxVertexCount, 4, MAX_HULL_VERTICES)

	// Single allocation for all working memory.
	sizes := ComputeHullWorkSizes(pointCount, clampedMaxCount)
	work := cast([^]u8)Alloc(sizes.totalBytes)

	builder: HullBuilder
	HullBuilder_Init(&builder, work, &sizes)

	shiftedPoints := cast([^]Vec3)(&work[sizes.offsetShiftedPoints])

	ok := HullBuilder_Construct(&builder, points, pointCount, clampedMaxCount, origin, shiftedPoints)
	if ok == false {
		Free(work, sizes.totalBytes)
		return nil
	}

	if builder.finalVertexCount > MAX_HULL_VERTICES {
		Log("hull final vertex count of %d exceeds limit of %d", builder.finalVertexCount, MAX_HULL_VERTICES)
		Free(work, sizes.totalBytes)
		return nil
	}

	if builder.finalFaceCount > MAX_HULL_FACES {
		Log("hull final face count of %d exceeds limit of %d", builder.finalFaceCount, MAX_HULL_FACES)
		Free(work, sizes.totalBytes)
		return nil
	}

	maxHalfEdgeCount: i32 = 2 * MAX_HULL_EDGES
	if builder.finalHalfEdgeCount > maxHalfEdgeCount {
		Log("hull final half edge count of %d exceeds limit of %d", builder.finalHalfEdgeCount, maxHalfEdgeCount)
		Free(work, sizes.totalBytes)
		return nil
	}

	// Walk lists into temp arrays bounded by B3_HULL_MAX_COUNT, stamping finalIndex on each node so
	// the resolution pass below is O(E + F) instead of O(E^2 + F^2).
	tempVertices: [HULL_MAX_COUNT]^QHVertex
	vertexCount: i32 = 0
	for node := builder.vertexList.link.next; node != &builder.vertexList.link; node = node.next {
		assert(vertexCount < HULL_MAX_COUNT)

		vertex := (^QHVertex)(node)
		vertex.finalIndex = vertexCount
		tempVertices[vertexCount] = vertex
		vertexCount += 1
	}

	// Collect edges in twin-paired order (i, i+1) by stamping each pair as we discover it.
	tempFaces: [HULL_MAX_COUNT]^QHFace
	tempEdges: [HULL_MAX_COUNT]^QHHalfEdge
	faceCount: i32 = 0
	edgeCount: i32 = 0

	for faceNode := builder.faceList.link.next; faceNode != &builder.faceList.link; faceNode = faceNode.next {
		assert(faceCount < HULL_MAX_COUNT)

		face := (^QHFace)(faceNode)
		face.finalIndex = faceCount
		tempFaces[faceCount] = face
		faceCount += 1

		edge := face.edge
		for {
			if edge.finalIndex < 0 {
				assert(edgeCount + 1 < HULL_MAX_COUNT)

				edge.finalIndex = edgeCount
				tempEdges[edgeCount] = edge
				edgeCount += 1
				edge.twin.finalIndex = edgeCount
				tempEdges[edgeCount] = edge.twin
				edgeCount += 1
			}
			edge = edge.next
			if edge == face.edge {
				break
			}
		}
	}

	soaVertexCount := GetHullSoaStride(vertexCount)
	soaNormalCount := GetHullSoaStride(faceCount)

	// Allocate the hull. Arrays hang off the end.
	byteCount := AlignUp8(size_of(HullData))
	vertexOffset := i32(byteCount)
	byteCount += AlignUp8(int(vertexCount * i32(size_of(HullVertex))))
	pointOffset := i32(byteCount)
	byteCount += AlignUp8(int(vertexCount * i32(size_of(Vec3))))
	edgeOffset := i32(byteCount)
	byteCount += AlignUp8(int(edgeCount * i32(size_of(HullHalfEdge))))
	planeOffset := i32(byteCount)
	byteCount += AlignUp8(int(faceCount * i32(size_of(Plane))))
	faceOffset := i32(byteCount)
	byteCount += AlignUp8(int(faceCount * i32(size_of(HullFace))))
	soaVertexOffset := i32(byteCount)
	byteCount += AlignUp8(int(3 * soaVertexCount * i32(size_of(f32))))
	soaNormalOffset := i32(byteCount)
	byteCount += AlignUp8(int(3 * soaNormalCount * i32(size_of(f32))))
	edgeCosineOffset := i32(byteCount)
	byteCount += AlignUp8(int((edgeCount / 2) * i32(size_of(f32))))

	hull := cast(^HullData)AllocHost(byteCount) // unified: geometry is the user's, never in a world arena (world_arena.odin)
	mem.zero(hull, byteCount)

	hull.version = HULL_VERSION
	hull.vertexOffset = vertexOffset
	hull.pointOffset = pointOffset
	hull.edgeOffset = edgeOffset
	hull.planeOffset = planeOffset
	hull.faceOffset = faceOffset
	hull.soaVertexOffset = soaVertexOffset
	hull.soaNormalOffset = soaNormalOffset
	hull.edgeCosineOffset = edgeCosineOffset

	hull.vertexCount = vertexCount
	hull.edgeCount = edgeCount
	hull.faceCount = faceCount

	hull.byteCount = i32(byteCount)

	vertices := GetHullVerticesWrite(hull)
	edges := GetHullEdgesWrite(hull)
	faces := cast([^]HullFace)(uintptr(hull) + uintptr(hull.faceOffset))
	finalPoints := GetHullPointsWrite(hull)
	planes := GetHullPlanesWrite(hull)
	soaVertices := GetHullSoaVerticesWrite(hull)
	soaNormals := GetHullSoaNormalsWrite(hull)

	vx := soaVertices
	vy := vx[soaVertexCount:]
	vz := vy[soaVertexCount:]

	for index: i32 = 0; index < vertexCount; index += 1 {
		vertices[index].edge = 0
		p := tempVertices[index].position
		finalPoints[index] = p

		vx[index] = p.x
		vy[index] = p.y
		vz[index] = p.z
	}

	for index := vertexCount; index < soaVertexCount; index += 1 {
		vx[index] = vx[0]
		vy[index] = vy[0]
		vz[index] = vz[0]
	}

	for index: i32 = 0; index < edgeCount; index += 1 {
		edge := tempEdges[index]
		assert(0 <= edge.next.finalIndex && edge.next.finalIndex <= i32(max(u8)))
		assert(0 <= edge.twin.finalIndex && edge.twin.finalIndex <= i32(max(u8)))
		assert(0 <= edge.face.finalIndex && edge.face.finalIndex <= i32(max(u8)))
		assert(0 <= edge.origin.finalIndex && edge.origin.finalIndex <= i32(max(u8)))

		edges[index].next = u8(edge.next.finalIndex)
		edges[index].twin = u8(edge.twin.finalIndex)
		edges[index].face = u8(edge.face.finalIndex)
		edges[index].origin = u8(edge.origin.finalIndex)

		vertices[edge.origin.finalIndex].edge = u8(index)
	}

	nx := soaNormals
	ny := nx[soaNormalCount:]
	nz := ny[soaNormalCount:]

	for index: i32 = 0; index < faceCount; index += 1 {
		face := tempFaces[index]
		assert(0 <= face.edge.finalIndex && face.edge.finalIndex <= i32(max(u8)))

		faces[index].edge = u8(face.edge.finalIndex)
		planes[index] = face.plane

		n := face.plane.normal
		nx[index] = n.x
		ny[index] = n.y
		nz[index] = n.z
	}

	for index := faceCount; index < soaNormalCount; index += 1 {
		nx[index] = 0.0
		ny[index] = 0.0
		nz[index] = 0.0
	}

	// All builder pointers are dead from here on.
	Free(work, sizes.totalBytes)

	UpdateHullEdgeCosines(hull)

	UpdateHullBounds(hull)
	success := UpdateHullBulkProperties(hull)
	if success == false {
		DestroyHull(hull)
		return nil
	}

	if IsValidHull(hull) == false {
		DestroyHull(hull)
		return nil
	}

	hull.hash = 0
	hull.hash = Hash64NonZero(cast([^]u8)hull, hull.byteCount)

	return hull
}

CloneHull :: proc(hull: ^HullData) -> ^HullData {
	if hull == nil || IsValidHull(hull) == false {
		return nil
	}

	clone := cast(^HullData)AllocHost(int(hull.byteCount)) // unified: geometry is the user's, never in a world arena (world_arena.odin)
	mem.copy(clone, hull, int(hull.byteCount))

	return clone
}

HashHullData :: proc(hull: ^HullData) -> u64 {
	return hull.hash
}

CompareHullData :: proc(hull1: ^HullData, hull2: ^HullData) -> bool {
	if hull1 == hull2 {
		return true
	}

	if hull1.byteCount != hull2.byteCount {
		return false
	}

	return mem.compare_ptrs(hull1, hull2, int(hull1.byteCount)) == 0
}

// Hull identity covers every byte, so the structs carry explicit padding. These lock
// the layout, re-audit padding if a size changes.
#assert(size_of(HullData) == 152, "unexpected hull data size")
#assert(size_of(BoxHull) == 696, "unexpected box hull size")

// Implement HullMap.
// unified: verstable's b3HullMap (open addressing, keyed by hull content) is a chained map: an Odin map from
// HashHullData to a chain of buckets compared with CompareHullData, plus an intrusive list of all buckets in
// insertion order for HullMap_first/HullMap_next. Same API shape (init, reserve, get, insert, get_or_insert,
// erase_itr, size, bucket_count, first, next, is_end, cleanup). Iteration order is insertion order, so it is
// deterministic (upstream iterates in bucket order; its only iteration sums byte counts).
HullMap_bucket :: struct {
	key:       ^HullData,
	val:       i32,

	// chain of buckets with the same hash
	chainNext: ^HullMap_bucket,

	// all buckets in insertion order
	prevAll:   ^HullMap_bucket,
	nextAll:   ^HullMap_bucket,
}

HullMap :: struct {
	chains:  map[u64]^HullMap_bucket,
	key_cnt: int,
	headAll: ^HullMap_bucket,
	tailAll: ^HullMap_bucket,
}

HullMap_itr :: struct {
	data: ^HullMap_bucket,
}

HullMap_init :: proc(table: ^HullMap) {
	table^ = {}
}

HullMap_reserve :: proc(table: ^HullMap, size: int) -> bool {
	reserve(&table.chains, size)
	return true
}

HullMap_size :: proc(table: ^HullMap) -> int {
	return table.key_cnt
}

HullMap_bucket_count :: proc(table: ^HullMap) -> int {
	return cap(table.chains)
}

// Total map allocation in bytes, excluding the stored buckets' hull data
HullMap_total_alloc_size :: proc(table: ^HullMap) -> int {
	// approximate: slot storage of the Odin map (key, value, hash) plus one bucket per entry
	return(
		cap(table.chains) * (size_of(u64) + size_of(^HullMap_bucket) + size_of(uintptr)) +
		table.key_cnt * size_of(HullMap_bucket) \
	)
}

HullMap_is_end :: proc(itr: HullMap_itr) -> bool {
	return itr.data == nil
}

HullMap_get :: proc(table: ^HullMap, key: ^HullData) -> HullMap_itr {
	head, found := table.chains[HashHullData(key)]
	if found {
		for bucket := head; bucket != nil; bucket = bucket.chainNext {
			if CompareHullData(bucket.key, key) {
				return HullMap_itr{bucket}
			}
		}
	}
	return HullMap_itr{nil}
}

// Inserts or replaces the value for key.
HullMap_insert :: proc(table: ^HullMap, key: ^HullData, val: i32) -> HullMap_itr {
	itr := HullMap_get(table, key)
	if itr.data != nil {
		itr.data.key = key
		itr.data.val = val
		return itr
	}

	bucket := cast(^HullMap_bucket)AllocHost(size_of(HullMap_bucket))
	bucket^ = {}
	bucket.key = key
	bucket.val = val

	hash := HashHullData(key)
	head := table.chains[hash]
	bucket.chainNext = head
	table.chains[hash] = bucket

	bucket.prevAll = table.tailAll
	if table.tailAll != nil {
		table.tailAll.nextAll = bucket
	} else {
		table.headAll = bucket
	}
	table.tailAll = bucket

	table.key_cnt += 1
	return HullMap_itr{bucket}
}

// Inserts the key with val if it does not exist; returns the existing or new entry.
HullMap_get_or_insert :: proc(table: ^HullMap, key: ^HullData, val: i32) -> HullMap_itr {
	itr := HullMap_get(table, key)
	if itr.data != nil {
		return itr
	}
	return HullMap_insert(table, key, val)
}

HullMap_erase_itr :: proc(table: ^HullMap, itr: HullMap_itr) {
	bucket := itr.data
	assert(bucket != nil)

	hash := HashHullData(bucket.key)
	head := table.chains[hash]
	if head == bucket {
		if bucket.chainNext != nil {
			table.chains[hash] = bucket.chainNext
		} else {
			delete_key(&table.chains, hash)
		}
	} else {
		prev := head
		for prev.chainNext != bucket {
			prev = prev.chainNext
		}
		prev.chainNext = bucket.chainNext
	}

	if bucket.prevAll != nil {
		bucket.prevAll.nextAll = bucket.nextAll
	} else {
		table.headAll = bucket.nextAll
	}
	if bucket.nextAll != nil {
		bucket.nextAll.prevAll = bucket.prevAll
	} else {
		table.tailAll = bucket.prevAll
	}

	Free(bucket, size_of(HullMap_bucket))
	table.key_cnt -= 1
}

HullMap_first :: proc(table: ^HullMap) -> HullMap_itr {
	return HullMap_itr{table.headAll}
}

HullMap_next :: proc(itr: HullMap_itr) -> HullMap_itr {
	return HullMap_itr{itr.data.nextAll}
}

HullMap_cleanup :: proc(table: ^HullMap) {
	bucket := table.headAll
	for bucket != nil {
		next := bucket.nextAll
		Free(bucket, size_of(HullMap_bucket))
		bucket = next
	}
	delete(table.chains)
	table^ = {}
}
// unified end

HullMapByteCount :: proc(table: ^HullMap) -> int {
	// The map owns a combined bucket and metadata allocation, valid only when buckets exist
	byteCount := size_of(HullMap)
	if HullMap_bucket_count(table) > 0 {
		byteCount += HullMap_total_alloc_size(table)
	}
	return byteCount
}

CloneAndTransformHull :: proc(original: ^HullData, transform: Transform, scale: Vec3) -> ^HullData {
	if original == nil || IsValidHull(original) == false {
		return nil
	}

	hull := cast(^HullData)AllocHost(int(original.byteCount)) // unified: geometry is the user's, never in a world arena (world_arena.odin)
	mem.copy(hull, original, int(original.byteCount))

	safeScale := SafeScale(scale)

	edges := GetHullEdgesWrite(hull)
	faces := GetHullFaces(hull)
	faceCount := hull.faceCount
	vertexCount := hull.vertexCount

	if safeScale.x * safeScale.y * safeScale.z < 0.0 {
		// Reflected: reverse edge winding for each face.
		for i: i32 = 0; i < faceCount; i += 1 {
			face := &faces[i]

			startEdgeIndex := i32(face.edge)
			currentEdgeIndex := startEdgeIndex
			prevEdgeIndex: i32 = NULL_INDEX

			for {
				edge := &edges[currentEdgeIndex]

				if i32(edge.next) == startEdgeIndex {
					prevEdgeIndex = currentEdgeIndex
					break
				}

				currentEdgeIndex = i32(edge.next)
				if currentEdgeIndex == startEdgeIndex {
					break
				}
			}

			assert(prevEdgeIndex != NULL_INDEX)

			currentEdgeIndex = startEdgeIndex

			for {
				edge := &edges[currentEdgeIndex]
				nextIndex := edge.next
				edge.next = u8(prevEdgeIndex)

				if currentEdgeIndex < i32(edge.twin) {
					twin := &edges[edge.twin]
					edge.origin, twin.origin = twin.origin, edge.origin
				}

				prevEdgeIndex = currentEdgeIndex
				currentEdgeIndex = i32(nextIndex)
				if currentEdgeIndex == startEdgeIndex {
					break
				}
			}
		}

		vertices := GetHullVerticesWrite(hull)

		for i: i32 = 0; i < vertexCount; i += 1 {
			vertex := &vertices[i]
			edge := &edges[vertex.edge]
			vertex.edge = edge.twin
		}
	}

	matrix_ := MakeMatrixFromQuat(transform.q)
	points := GetHullPointsWrite(hull)

	soaVertexCount := GetHullSoaStride(vertexCount)
	vx := GetHullSoaVerticesWrite(hull)
	vy := vx[soaVertexCount:]
	vz := vy[soaVertexCount:]

	for i: i32 = 0; i < vertexCount; i += 1 {
		p := Add(MulMV(matrix_, Mul(safeScale, points[i])), transform.p)
		points[i] = p
		vx[i] = p.x
		vy[i] = p.y
		vz[i] = p.z
	}

	for i := vertexCount; i < soaVertexCount; i += 1 {
		vx[i] = vx[0]
		vy[i] = vy[0]
		vz[i] = vz[0]
	}

	planes := GetHullPlanesWrite(hull)
	soaNormalCount := GetHullSoaStride(faceCount)
	nx := GetHullSoaNormalsWrite(hull)
	ny := nx[soaNormalCount:]
	nz := ny[soaNormalCount:]

	for i: i32 = 0; i < faceCount; i += 1 {
		count: i32 = 0
		centroid := Vec3_zero
		normal := Vec3_zero

		face := &faces[i]
		startEdgeIndex := face.edge
		currentEdgeIndex := startEdgeIndex

		startEdge := &edges[currentEdgeIndex]
		assert(i32(startEdge.face) == i)
		assert(i32(startEdge.origin) < vertexCount)

		origin := points[startEdge.origin]

		for {
			edge := &edges[currentEdgeIndex]
			twin := &edges[edge.twin]
			assert(twin.twin == currentEdgeIndex)

			v1 := Sub(points[edge.origin], origin)
			v2 := Sub(points[twin.origin], origin)

			count += 1
			centroid = Add(centroid, v1)
			normal.x += (v1.y - v2.y) * (v1.z + v2.z)
			normal.y += (v1.z - v2.z) * (v1.x + v2.x)
			normal.z += (v1.x - v2.x) * (v1.y + v2.y)

			currentEdgeIndex = edge.next
			if currentEdgeIndex == startEdgeIndex {
				break
			}
		}

		assert(count > 0)
		centroid = MulSV(1.0 / f32(count), centroid)
		centroid = Add(centroid, origin)

		area := Length(normal)
		assert(area > 0.0)
		normal = MulSV(1.0 / area, normal)

		planes[i] = MakePlaneFromNormalAndPoint(normal, centroid)

		nx[i] = normal.x
		ny[i] = normal.y
		nz[i] = normal.z
	}

	for i := faceCount; i < soaNormalCount; i += 1 {
		nx[i] = 0.0
		ny[i] = 0.0
		nz[i] = 0.0
	}

	UpdateHullEdgeCosines(hull)

	UpdateHullBounds(hull)
	success := UpdateHullBulkProperties(hull)
	if success == false {
		Free(hull, int(original.byteCount))
		return nil
	}

	// Must ensure the hash is 0 so it doesn't contribute to itself.
	hull.hash = 0
	hull.hash = Hash64NonZero(cast([^]u8)hull, hull.byteCount)

	VALIDATE(IsValidHull(hull))

	return hull
}

DestroyHull :: proc(hull: ^HullData) {
	Free(hull, int(hull.byteCount))
}

ComputeHullMass :: proc(shape: ^HullData, density: f32) -> MassData {
	out: MassData
	out.mass = density * shape.volume
	out.center = shape.center

	// Inertia about the center of mass
	out.inertia = MulSM(density, shape.centralInertia)
	return out
}

ComputeHullAABB :: proc(shape: ^HullData, transform: Transform) -> AABB {
	return AABB_Transform(transform, shape.aabb)
}

ComputeSweptHullAABB :: proc(shape: ^HullData, xf1: Transform, xf2: Transform) -> AABB {
	aabb1 := AABB_Transform(xf1, shape.aabb)
	aabb2 := AABB_Transform(xf2, shape.aabb)
	return AABB_Union(aabb1, aabb2)
}

OverlapHull :: proc(shape: ^HullData, shapeTransform: Transform, proxy: ^ShapeProxy) -> bool {
	points := GetHullPoints(shape)

	input: DistanceInput
	input.proxyA = ShapeProxy{points, shape.vertexCount, 0.0}
	input.proxyB = proxy^
	input.transform = InvMulTransforms(shapeTransform, Transform_identity)
	input.useRadii = true

	cache := SimplexCache{}
	output := ShapeDistance(&input, &cache, nil, 0)
	return output.distance < OVERLAP_SLOP
}

RayCastHull :: proc(shape: ^HullData, input: ^RayCastInput) -> CastOutput {
	assert(IsValidRay(input))
	output := CastOutput{}

	lower: f32 = 0.0
	upper := input.maxFraction
	bestFace: i32 = NULL_INDEX

	planes := GetHullPlanes(shape)

	for faceIndex: i32 = 0; faceIndex < shape.faceCount; faceIndex += 1 {
		plane := planes[faceIndex]

		distance := plane.offset - Dot(plane.normal, input.origin)
		denominator := Dot(plane.normal, input.translation)

		if denominator == 0.0 {
			if distance < 0.0 {
				return output
			}
		} else {
			fraction := distance / denominator

			if denominator < 0.0 {
				if fraction > lower {
					bestFace = faceIndex
					lower = fraction
				}
			} else {
				if fraction < upper {
					upper = fraction
				}
			}

			if upper < lower {
				return output
			}
		}
	}

	if bestFace >= 0 {
		output.point = Add(input.origin, MulSV(lower, input.translation))
		output.normal = planes[bestFace].normal
		output.fraction = lower
		output.hit = true
	} else {
		output.point = input.origin
		output.hit = true
	}

	return output
}

ShapeCastHull :: proc(shape: ^HullData, input: ^ShapeCastInput) -> CastOutput {
	points := GetHullPoints(shape)

	pairInput: ShapeCastPairInput
	pairInput.proxyA = ShapeProxy{points, shape.vertexCount, 0.0}
	pairInput.proxyB = input.proxy
	pairInput.transform = Transform_identity
	pairInput.translationB = input.translation
	pairInput.maxFraction = input.maxFraction
	pairInput.canEncroach = input.canEncroach

	output := ShapeCast(&pairInput)
	return output
}

CollideMoverAndHull :: proc(result: ^PlaneResult, shape: ^HullData, mover: ^Capsule) -> i32 {
	points := GetHullPoints(shape)
	distanceInput: DistanceInput
	distanceInput.proxyA = ShapeProxy{points, shape.vertexCount, 0.0}
	distanceInput.proxyB = ShapeProxy{([^]Vec3)(&mover.center1), 2, mover.radius}
	distanceInput.transform = Transform_identity
	distanceInput.useRadii = false

	totalRadius := mover.radius

	cache := SimplexCache{}
	distanceOutput := ShapeDistance(&distanceInput, &cache, nil, 0)

	if distanceOutput.distance == 0.0 {
		// I could handle deep overlap on hulls, but there is no reasonable solution for
		// deep overlap on meshes. So if someone converted a hull to a mesh there would be
		// different behavior. So I think there is not a good reason to handle deep overlap
		// on hulls.
		return 0
	}

	if distanceOutput.distance <= totalRadius {
		plane := Plane{distanceOutput.normal, totalRadius - distanceOutput.distance}
		result^ = PlaneResult{plane, distanceOutput.pointA, 0, 0, 0}
		return 1
	}

	return 0
}

ComputeHullExtent :: proc(hull: ^HullData, origin: Vec3) -> ShapeExtent {
	points := GetHullPoints(hull)

	extent: ShapeExtent
	extent.minExtent = hull.innerRadius
	extent.maxExtent = Vec3_zero
	for index: i32 = 0; index < hull.vertexCount; index += 1 {
		point := points[index]
		extent.maxExtent = Max(extent.maxExtent, Abs(Sub(point, origin)))
	}

	return extent
}

ComputeHullProjectedArea :: proc(hull: ^HullData, direction: Vec3) -> f32 {
	area: f32 = 0.0

	faceCount := hull.faceCount
	hullFaces := GetHullFaces(hull)
	hullEdges := GetHullEdges(hull)
	hullPoints := GetHullPoints(hull)

	for i: i32 = 0; i < faceCount; i += 1 {
		face := &hullFaces[i]

		baseEdge := i32(face.edge)
		edge := &hullEdges[baseEdge]
		p1 := hullPoints[edge.origin]

		edgeIndex := i32(edge.next)
		edge = &hullEdges[edgeIndex]
		p2 := hullPoints[edge.origin]

		edgeIndex = i32(edge.next)

		for {
			edge = &hullEdges[edgeIndex]
			p3 := hullPoints[edge.origin]

			e1 := Sub(p2, p1)
			e2 := Sub(p3, p1)
			n := Cross(e1, e2)
			a := Dot(n, direction)
			area += MaxFloat(a, 0.0)

			p2 = p3
			edgeIndex = i32(edge.next)
			if edgeIndex == baseEdge {
				break
			}
		}
	}

	return 0.5 * area
}

// Constant template box (vertex/edge/face/topology). b3MakeTransformedBoxHull copies and
// fills in the runtime-dependent fields (boxPoints, boxPlanes, aabb, mass properties, hash).
@(private = "file", rodata)
s_boxHull := BoxHull {
	base = {
		version = HULL_VERSION,
		byteCount = i32(size_of(BoxHull)),
		hash = 0,
		vertexCount = 8,
		edgeCount = 24,
		faceCount = 6,
		vertexOffset = i32(offset_of(BoxHull, boxVertices)),
		pointOffset = i32(offset_of(BoxHull, boxPoints)),
		edgeOffset = i32(offset_of(BoxHull, boxEdges)),
		planeOffset = i32(offset_of(BoxHull, boxPlanes)),
		faceOffset = i32(offset_of(BoxHull, boxFaces)),
		soaVertexOffset = i32(offset_of(BoxHull, vx)),
		soaNormalOffset = i32(offset_of(BoxHull, nx)),
		edgeCosineOffset = i32(offset_of(BoxHull, edgeCosines)),
	},
	boxVertices = {
		0 = {edge = 8},
		1 = {edge = 1},
		2 = {edge = 0},
		3 = {edge = 9},
		4 = {edge = 13},
		5 = {edge = 3},
		6 = {edge = 5},
		7 = {edge = 11},
	},
	boxEdges = {
		0 = {2, 1, 2, 0},
		1 = {17, 0, 1, 5},
		2 = {4, 3, 1, 0},
		3 = {20, 2, 5, 3},
		4 = {6, 5, 5, 0},
		5 = {23, 4, 6, 4},
		6 = {0, 7, 6, 0},
		7 = {18, 6, 2, 2},
		8 = {10, 9, 0, 1},
		9 = {21, 8, 3, 5},
		10 = {12, 11, 3, 1},
		11 = {16, 10, 7, 2},
		12 = {14, 13, 7, 1},
		13 = {19, 12, 4, 4},
		14 = {8, 15, 4, 1},
		15 = {22, 14, 0, 3},
		16 = {7, 17, 3, 2},
		17 = {9, 16, 2, 5},
		18 = {11, 19, 6, 2},
		19 = {5, 18, 7, 4},
		20 = {15, 21, 1, 3},
		21 = {1, 20, 0, 5},
		22 = {3, 23, 4, 3},
		23 = {13, 22, 5, 4},
	},
	boxFaces = {0 = {edge = 0}, 1 = {edge = 8}, 2 = {edge = 16}, 3 = {edge = 20}, 4 = {edge = 19}, 5 = {edge = 21}},
}

MakeTransformedBoxHull :: proc(hx: f32, hy: f32, hz: f32, transform: Transform) -> BoxHull {
	assert(IsValidTransform(transform))

	boxHull := s_boxHull

	minH: f32 = 0.2 * LINEAR_SLOP
	h := Max(Vec3{minH, minH, minH}, Vec3{hx, hy, hz})

	boxHull.base.aabb = AABB_Transform(transform, AABB{Neg(h), h})
	boxHull.base.surfaceArea = 8.0 * (h.x * h.y + h.x * h.z + h.y * h.z)
	boxHull.base.volume = 8.0 * h.x * h.y * h.z
	boxHull.base.innerRadius = MinFloat(h.x, MinFloat(h.y, h.z))
	boxHull.base.center = transform.p

	boxInertia := BoxInertia(boxHull.base.volume, Neg(h), h)
	boxHull.base.centralInertia = RotateInertia(transform.q, boxInertia)

	lower := Neg(h)
	upper := h

	boxHull.boxPlanes[0] = TransformPlane(transform, MakePlaneFromNormalAndPoint(Neg(Vec3_axisX), lower))
	boxHull.boxPlanes[1] = TransformPlane(transform, MakePlaneFromNormalAndPoint(Vec3_axisX, upper))
	boxHull.boxPlanes[2] = TransformPlane(transform, MakePlaneFromNormalAndPoint(Neg(Vec3_axisY), lower))
	boxHull.boxPlanes[3] = TransformPlane(transform, MakePlaneFromNormalAndPoint(Vec3_axisY, upper))
	boxHull.boxPlanes[4] = TransformPlane(transform, MakePlaneFromNormalAndPoint(Neg(Vec3_axisZ), lower))
	boxHull.boxPlanes[5] = TransformPlane(transform, MakePlaneFromNormalAndPoint(Vec3_axisZ, upper))

	boxHull.boxPoints[0] = TransformPoint(transform, Vec3{h.x, h.y, h.z})
	boxHull.boxPoints[1] = TransformPoint(transform, Vec3{-h.x, h.y, h.z})
	boxHull.boxPoints[2] = TransformPoint(transform, Vec3{-h.x, -h.y, h.z})
	boxHull.boxPoints[3] = TransformPoint(transform, Vec3{h.x, -h.y, h.z})
	boxHull.boxPoints[4] = TransformPoint(transform, Vec3{h.x, h.y, -h.z})
	boxHull.boxPoints[5] = TransformPoint(transform, Vec3{-h.x, h.y, -h.z})
	boxHull.boxPoints[6] = TransformPoint(transform, Vec3{-h.x, -h.y, -h.z})
	boxHull.boxPoints[7] = TransformPoint(transform, Vec3{h.x, -h.y, -h.z})

	// SOA
	boxHull.vx[0] = boxHull.boxPoints[0].x
	boxHull.vx[1] = boxHull.boxPoints[1].x
	boxHull.vx[2] = boxHull.boxPoints[2].x
	boxHull.vx[3] = boxHull.boxPoints[3].x
	boxHull.vx[4] = boxHull.boxPoints[4].x
	boxHull.vx[5] = boxHull.boxPoints[5].x
	boxHull.vx[6] = boxHull.boxPoints[6].x
	boxHull.vx[7] = boxHull.boxPoints[7].x

	boxHull.vy[0] = boxHull.boxPoints[0].y
	boxHull.vy[1] = boxHull.boxPoints[1].y
	boxHull.vy[2] = boxHull.boxPoints[2].y
	boxHull.vy[3] = boxHull.boxPoints[3].y
	boxHull.vy[4] = boxHull.boxPoints[4].y
	boxHull.vy[5] = boxHull.boxPoints[5].y
	boxHull.vy[6] = boxHull.boxPoints[6].y
	boxHull.vy[7] = boxHull.boxPoints[7].y

	boxHull.vz[0] = boxHull.boxPoints[0].z
	boxHull.vz[1] = boxHull.boxPoints[1].z
	boxHull.vz[2] = boxHull.boxPoints[2].z
	boxHull.vz[3] = boxHull.boxPoints[3].z
	boxHull.vz[4] = boxHull.boxPoints[4].z
	boxHull.vz[5] = boxHull.boxPoints[5].z
	boxHull.vz[6] = boxHull.boxPoints[6].z
	boxHull.vz[7] = boxHull.boxPoints[7].z

	boxHull.nx[0] = boxHull.boxPlanes[0].normal.x
	boxHull.nx[1] = boxHull.boxPlanes[1].normal.x
	boxHull.nx[2] = boxHull.boxPlanes[2].normal.x
	boxHull.nx[3] = boxHull.boxPlanes[3].normal.x
	boxHull.nx[4] = boxHull.boxPlanes[4].normal.x
	boxHull.nx[5] = boxHull.boxPlanes[5].normal.x
	boxHull.nx[6] = 0.0
	boxHull.nx[7] = 0.0

	boxHull.ny[0] = boxHull.boxPlanes[0].normal.y
	boxHull.ny[1] = boxHull.boxPlanes[1].normal.y
	boxHull.ny[2] = boxHull.boxPlanes[2].normal.y
	boxHull.ny[3] = boxHull.boxPlanes[3].normal.y
	boxHull.ny[4] = boxHull.boxPlanes[4].normal.y
	boxHull.ny[5] = boxHull.boxPlanes[5].normal.y
	boxHull.ny[6] = 0.0
	boxHull.ny[7] = 0.0

	boxHull.nz[0] = boxHull.boxPlanes[0].normal.z
	boxHull.nz[1] = boxHull.boxPlanes[1].normal.z
	boxHull.nz[2] = boxHull.boxPlanes[2].normal.z
	boxHull.nz[3] = boxHull.boxPlanes[3].normal.z
	boxHull.nz[4] = boxHull.boxPlanes[4].normal.z
	boxHull.nz[5] = boxHull.boxPlanes[5].normal.z
	boxHull.nz[6] = 0.0
	boxHull.nz[7] = 0.0

	// Must ensure the hash is 0 so it doesn't contribute to itself.
	boxHull.base.hash = 0
	boxHull.base.hash = Hash64NonZero(cast([^]u8)&boxHull.base, boxHull.base.byteCount)

	return boxHull
}

MakeCubeHull :: proc(halfWidth: f32) -> BoxHull {
	return MakeBoxHull(halfWidth, halfWidth, halfWidth)
}

MakeOffsetBoxHull :: proc(hx: f32, hy: f32, hz: f32, offset: Vec3) -> BoxHull {
	transform := Transform {
		p = offset,
		q = Quat_identity,
	}
	return MakeTransformedBoxHull(hx, hy, hz, transform)
}

MakeBoxHull :: proc(hx: f32, hy: f32, hz: f32) -> BoxHull {
	return MakeTransformedBoxHull(hx, hy, hz, Transform_identity)
}

ScaleBox :: proc(halfWidths: ^Vec3, transform: ^Transform, postScale: Vec3, minHalfWidth: f32) {
	assert(IsValidFloat(minHalfWidth) && minHalfWidth > 0.0)

	q := transform.q

	if postScale.x < 0.0 || postScale.y < 0.0 || postScale.z < 0.0 {
		// todo this might be unnecessary if rotation is identity
		// todo compare with polar decomposition (much more expensive)
		m := MakeMatrixFromQuat(q)
		m.cx.x *= postScale.x
		m.cy.x *= postScale.x
		m.cz.x *= postScale.x
		m.cx.y *= postScale.y
		m.cy.y *= postScale.y
		m.cz.y *= postScale.y
		m.cx.z *= postScale.z
		m.cy.z *= postScale.z
		m.cz.z *= postScale.z
		m.cx = Normalize(m.cx)
		m.cy = Normalize(m.cy)
		m.cz = Normalize(m.cz)
		m.cx = postScale.x < 0.0 ? Neg(m.cx) : m.cx
		m.cy = postScale.y < 0.0 ? Neg(m.cy) : m.cy
		m.cz = postScale.z < 0.0 ? Neg(m.cz) : m.cz
		q = MakeQuatFromMatrix(&m)
	}

	absScale := Abs(postScale)

	h := halfWidths^
	p1 := Mul(absScale, RotateVector(q, Neg(h)))
	p2 := Mul(absScale, RotateVector(q, h))

	localP1 := InvRotateVector(q, p1)
	localP2 := InvRotateVector(q, p2)

	lower := Min(localP1, localP2)
	upper := Max(localP1, localP2)

	scaledHalfWidth := MulSV(0.5, Sub(upper, lower))

	mLimit := Vec3{minHalfWidth, minHalfWidth, minHalfWidth}
	halfWidths^ = Max(scaledHalfWidth, mLimit)
	transform.p = Mul(postScale, transform.p)
	transform.q = q
}

// todo use new hull scaling technique
MakeScaledBoxHull :: proc(halfWidths: Vec3, transform: Transform, postScale: Vec3) -> BoxHull {
	assert(IsValidTransform(transform))

	h := halfWidths
	xf := transform
	ScaleBox(&h, &xf, postScale, 4.0 * LINEAR_SLOP)
	return MakeTransformedBoxHull(h.x, h.y, h.z, xf)
}

// 2D hull (Andrew's monotone chain)

// cross(b - a, c - a)
@(private = "file")
Cross2D :: proc(a: Vec2, b: Vec2, c: Vec2) -> f32 {
	return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

// Iteratively remove the vertices contributing the least area.
SimplifyHull2D :: proc(hull: [^]Point2D, count1: i32, target: i32) -> i32 {
	assert(target >= 3)

	if count1 <= 3 {
		return count1
	}

	linearSlop: f32 = LINEAR_SLOP
	areaTol := 0.25 * linearSlop * linearSlop
	count2 := count1

	for {
		minArea := max(f32)
		minIndex: i32 = 0

		for i: i32 = 0; i < count2; i += 1 {
			prev := (i + count2 - 1) % count2
			next := (i + 1) % count2
			area := Cross2D(hull[prev].p, hull[i].p, hull[next].p)
			VALIDATE(area >= -FLT_EPSILON)

			if area + areaTol < minArea {
				minArea = area
				minIndex = i
			}
		}

		if count2 <= 3 || (minArea > areaTol && count2 <= target) {
			break
		}

		count2 -= 1
		if minIndex < count2 {
			mem.copy(&hull[minIndex], &hull[minIndex + 1], int(count2 - minIndex) * size_of(Point2D))
		}
	}

	return count2
}

// Sort comparator: by x, then by y. Insertion since n is typically 20 or less.
@(private = "file")
Sort2D :: proc(pts: [^]Point2D, count: i32) {
	for i: i32 = 1; i < count; i += 1 {
		base := pts[i]
		j := i - 1
		for j >= 0 && (pts[j].p.x > base.p.x || (pts[j].p.x == base.p.x && pts[j].p.y > base.p.y)) {
			pts[j + 1] = pts[j]
			j -= 1
		}
		pts[j + 1] = base
	}
}

// Welds points that have been sorted O(n) time
@(private = "file")
Weld2D :: proc(pts: [^]Point2D, count1: i32) -> i32 {
	if count1 <= 1 {
		return count1
	}

	linearSlop: f32 = 0.25 * LINEAR_SLOP
	tolSqr := linearSlop * linearSlop

	count2: i32 = 1
	baseIndex: i32 = 0
	for i: i32 = 1; i < count1; i += 1 {
		distanceSqr := DistanceSquared2(pts[i].p, pts[baseIndex].p)
		if distanceSqr < tolSqr {
			if pts[i].originalIndex < pts[baseIndex].originalIndex {
				pts[baseIndex] = pts[i]
			}
		} else {
			pts[count2] = pts[i]
			baseIndex = count2
			count2 += 1
		}
	}

	return count2
}

// Andrew's monotone chain convex hull algorithm
// hull must have space for 2 * count1 points
Hull2D :: proc(pts: [^]Point2D, count: i32, hull: [^]Point2D) -> i32 {
	count1 := count

	if count1 <= 0 {
		return 0
	}

	if count1 == 1 {
		hull[0] = pts[0]
		return 1
	}

	Sort2D(pts, count1)
	count1 = Weld2D(pts, count1)

	if count1 == 1 {
		hull[0] = pts[0]
		return 1
	}

	if count1 == 2 {
		hull[0] = pts[0]
		hull[1] = pts[1]
		return 2
	}

	count2: i32 = 0

	for i: i32 = 0; i < count1; i += 1 {
		for count2 >= 2 {
			area := Cross2D(hull[count2 - 2].p, hull[count2 - 1].p, pts[i].p)
			if area > 0.0 {
				break
			}

			count2 -= 1
		}

		VALIDATE(count2 < 2 * count1)
		hull[count2] = pts[i]
		count2 += 1
	}

	lowerCount := count2 + 1

	for i := count1 - 2; i >= 0; i -= 1 {
		for count2 >= lowerCount {
			area := Cross2D(hull[count2 - 2].p, hull[count2 - 1].p, pts[i].p)
			if area > 0.0 {
				break
			}

			count2 -= 1
		}

		VALIDATE(count2 < 2 * count1)
		hull[count2] = pts[i]
		count2 += 1
	}

	VALIDATE(hull[0].originalIndex == hull[count2 - 1].originalIndex)
	return count2 - 1
}
