// Ported from box3d src/mesh_contact.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "core:mem"

@(private = "file")
MAX_POINTS_PER_TRIANGLE :: 32

when ENABLE_VALIDATION {
	@(private = "file")
	IsSorted :: proc(array: [^]i32, count: i32) -> bool {
		for i: i32 = 0; i < count - 1; i += 1 {
			if array[i] >= array[i + 1] {
				return false
			}
		}

		return true
	}
}

TriangleQueryContext :: struct {
	indices:  [^]i32,
	capacity: i32,
	count:    i32,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
CollectTriangleIndicesCallback :: proc "c" (a, b, c: Vec3, triangleIndex: i32, context_: rawptr) -> bool {
	// unified end
	_ = a
	_ = b
	_ = c
	triangleContext := cast(^TriangleQueryContext)context_
	if triangleContext.count == triangleContext.capacity {
		return false
	}
	triangleContext.indices[triangleContext.count] = triangleIndex
	triangleContext.count += 1
	return triangleContext.count < triangleContext.capacity
}

@(private = "file")
QueryMeshTriangles :: proc(indices: [^]i32, capacity: i32, mesh: ^Mesh, bounds: AABB) -> i32 {
	context_ := TriangleQueryContext {
		indices  = indices,
		capacity = capacity,
		count    = 0,
	}

	QueryMesh(mesh, bounds, CollectTriangleIndicesCallback, &context_)
	return context_.count
}

@(private = "file")
QueryHeightFieldTriangles :: proc(indices: [^]i32, capacity: i32, heightField: ^HeightFieldData, bounds: AABB) -> i32 {
	context_ := TriangleQueryContext {
		indices  = indices,
		capacity = capacity,
		count    = 0,
	}

	QueryHeightField(heightField, bounds, CollectTriangleIndicesCallback, &context_)
	return context_.count
}

// static bool s_once in b3RefreshCache
@(private = "file")
s_once := false

@(private = "file")
RefreshCache :: proc(contact: ^Contact, shapeA: ^Shape, xfA: WorldTransform, bounds: ^AABB, arena: Arena) {
	arena := arena
	assert(shapeA.type == .meshShape || shapeA.type == .heightShape)

	meshContact := &contact.meshContact

	// If the dynamic body didn't move out of the cached query bounds we are done!
	if AABB_Contains(meshContact.queryBounds, bounds^) {
		if shapeA.type == .meshShape {
			for i: i32 = 0; i < contact.meshContact.triangleCache.count; i += 1 {
				assert(
					0 <= contact.meshContact.triangleCache.data[i].triangleIndex &&
					contact.meshContact.triangleCache.data[i].triangleIndex < shapeA.mesh.data.triangleCount,
				)
			}
		}

		return
	}

	// Enlarge to the query bounds to absorb small movement
	radius := MAX_AABB_MARGIN + SPECULATIVE_DISTANCE
	extension := Vec3{radius, radius, radius}
	meshContact.queryBounds.lowerBound = Sub(bounds.lowerBound, extension)
	meshContact.queryBounds.upperBound = Add(bounds.upperBound, extension)

	// Query triangles
	triangleCapacity: i32 = MAX_MESH_CONTACT_TRIANGLES
	triangleIndices := cast([^]i32)Bump(&arena, MAX_MESH_CONTACT_TRIANGLES * size_of(i32))

	// Bounds are in world space. Convert to the local mesh frame. The broadphase bounds are float,
	// so the demoted mesh transform is the matching float world frame (exact in float mode).
	meshTransform := ToRelativeTransform(xfA, Pos_zero)
	localBounds := AABB_Transform(InvertTransform(meshTransform), meshContact.queryBounds)
	triangleCount: i32
	if shapeA.type == .meshShape {
		triangleCount = QueryMeshTriangles(triangleIndices, triangleCapacity, &shapeA.mesh, localBounds)
	} else {
		assert(shapeA.type == .heightShape)
		triangleCount = QueryHeightFieldTriangles(triangleIndices, triangleCapacity, shapeA.heightField, localBounds)
	}

	if triangleCount == triangleCapacity {
		if s_once == false {
			Log("WARNING: complex mesh detected, triangle buffer capacity of %d reached", triangleCapacity)
			s_once = true
		}
	}

	// Triangle indices must be sorted to match caches.
	when ENABLE_VALIDATION {
		VALIDATE(IsSorted(triangleIndices, triangleCount))
	}

	// Create new contact cache and match with old one
	contactCache := cast([^]ContactCache)Bump(&arena, triangleCount * size_of(ContactCache))

	index2: i32 = 0
	for index1: i32 = 0; index1 < triangleCount; index1 += 1 {
		contactCache[index1] = {}

		for index2 < contact.meshContact.triangleCache.count &&
		    contact.meshContact.triangleCache.data[index2].triangleIndex < triangleIndices[index1] {
			index2 += 1
		}

		if index2 < contact.meshContact.triangleCache.count &&
		   contact.meshContact.triangleCache.data[index2].triangleIndex == triangleIndices[index1] {
			contactCache[index1] = contact.meshContact.triangleCache.data[index2].cache
		}
	}

	// Save new cache
	Array_Resize(&contact.meshContact.triangleCache, triangleCount)
	for i: i32 = 0; i < triangleCount; i += 1 {
		contact.meshContact.triangleCache.data[i] = TriangleCache{triangleIndices[i], contactCache[i]}

		if shapeA.type == .meshShape {
			assert(
				0 <= contact.meshContact.triangleCache.data[i].triangleIndex &&
				contact.meshContact.triangleCache.data[i].triangleIndex < shapeA.mesh.data.triangleCount,
			)
		}
	}
}

TentativeTriangle :: struct {
	squaredDistance: f32,
	index:           i32,
}

@(private = "file")
MAX_EDGE_COUNT :: 64

FoundEdges :: struct {
	keys:  [MAX_EDGE_COUNT]u64,
	count: i32,
}

@(private = "file")
AddEdge :: #force_inline proc(edges: ^FoundEdges, vertex1: i32, vertex2: i32) -> bool {
	i1 := u64(MinInt(vertex1, vertex2))
	i2 := u64(MaxInt(vertex1, vertex2))
	key := i1 << 32 | i2

	count := edges.count
	for i: i32 = 0; i < count; i += 1 {
		if edges.keys[i] == key {
			return false
		}
	}

	if count == MAX_EDGE_COUNT {
		// This will lead to a potential ghost collision
		return true
	}

	edges.keys[count] = key
	edges.count += 1

	return true
}

@(private = "file")
FindEdge :: #force_inline proc(edges: ^FoundEdges, vertex1: i32, vertex2: i32) -> bool {
	i1 := u64(MinInt(vertex1, vertex2))
	i2 := u64(MaxInt(vertex1, vertex2))
	key := i1 << 32 | i2

	count := edges.count
	for i: i32 = 0; i < count; i += 1 {
		if edges.keys[i] == key {
			return true
		}
	}

	return false
}

// unified: not ported: the disabled (#if 0) b3TrianglesShareEdge.

@(private = "file")
MAX_VERTEX_COUNT :: 64

FoundVertices :: struct {
	keys:  [MAX_VERTEX_COUNT]i32,
	count: i32,
}

@(private = "file")
AddVertex :: #force_inline proc(vertices: ^FoundVertices, vertex: i32) -> bool {
	key := vertex

	count := vertices.count
	for i: i32 = 0; i < count; i += 1 {
		if vertices.keys[i] == key {
			return false
		}
	}

	if count == MAX_VERTEX_COUNT {
		// This will lead to a potential ghost collision
		return true
	}

	vertices.keys[count] = key
	vertices.count += 1

	return true
}

when MAX_MANIFOLD_POINTS == 4 {

	// Returns true if (score, separation) should replace (bestScore, bestSeparation).
	@(private = "file")
	IsBetterCullCandidate :: #force_inline proc(
		score: f32,
		separation: f32,
		bestScore: f32,
		bestSeparation: f32,
		scoreTol: f32,
		separationTol: f32,
	) -> bool {
		if score > bestScore + scoreTol {
			return true
		}
		if score < bestScore - scoreTol {
			return false
		}

		// Break the tie using separation
		return separation < bestSeparation - separationTol
	}

	@(private = "file")
	CullPoints :: proc(points: [^]Point2D, count: i32) -> i32 {
		if count <= 1 {
			return count
		}

		tol := 0.25 * LINEAR_SLOP
		tolSqr := tol * tol
		separationTol := LINEAR_SLOP

		finalPoints: [4]Point2D
		count1 := count

		// Step 1: the two points with the largest distance, ties broken by deepest combined separation
		bestScore: f32 = 0.0
		bestSeparation := FLT_MAX
		bestIndex1: i32 = NULL_INDEX
		bestIndex2: i32 = NULL_INDEX

		for i: i32 = 0; i < count1; i += 1 {
			p1 := points[i].p
			for j: i32 = i + 1; j < count1; j += 1 {
				score := DistanceSquared2(p1, points[j].p)
				// Separation sum heuristic
				separation := points[i].separation + points[j].separation

				if IsBetterCullCandidate(score, separation, bestScore, bestSeparation, tolSqr, separationTol) {
					bestIndex1 = i
					bestIndex2 = j
					bestScore = score
					bestSeparation = separation
				}
			}
		}

		if bestScore < tolSqr {
			// Choose deepest point
			deepestIndex: i32 = 0
			for i: i32 = 1; i < count1; i += 1 {
				if points[i].separation < points[deepestIndex].separation {
					deepestIndex = i
				}
			}

			if deepestIndex != 0 {
				points[0] = points[deepestIndex]
			}
			return 1
		}

		finalPoints[0] = points[bestIndex1]
		finalPoints[1] = points[bestIndex2]

		// Cull
		points[bestIndex2] = points[count1 - 1]
		points[bestIndex1] = points[count1 - 2]
		count1 -= 2

		if count1 == 0 {
			points[0] = finalPoints[0]
			points[1] = finalPoints[1]
			return 2
		}

		// First anchor point
		a := finalPoints[0].p

		// Second anchor point
		b := finalPoints[1].p
		ba := Sub2(b, a)
		// float length = b3Length2( ba );
		// float areaTol = tol * length;

		// Step 2: find the point with the maximum triangular area, ties broken by deepest separation
		bestScore = 0.0
		bestSeparation = FLT_MAX
		bestIndex: i32 = NULL_INDEX
		bestSignedArea: f32 = 0.0
		for i: i32 = 0; i < count1; i += 1 {
			p := points[i].p
			signedArea := Cross2(ba, Sub2(p, a))
			score := AbsFloat(signedArea)

			if IsBetterCullCandidate(score, points[i].separation, bestScore, bestSeparation, tolSqr, separationTol) {
				bestSignedArea = signedArea
				bestScore = score
				bestSeparation = points[i].separation
				bestIndex = i
			}
		}

		if bestIndex == NULL_INDEX {
			// All points collinear
			points[0] = finalPoints[0]
			points[1] = finalPoints[1]
			return 2
		}

		// Store best point
		finalPoints[2] = points[bestIndex]

		if count1 == 1 {
			points[0] = finalPoints[0]
			points[1] = finalPoints[1]
			points[2] = finalPoints[2]
			return 3
		}

		// Cull
		points[bestIndex] = points[count1 - 1]
		count1 -= 1

		// Step 4: get the point that adds the most area outside the current triangle

		// Third anchor
		c := finalPoints[2].p

		// Ensure CCW ordering
		if bestSignedArea < 0.0 {
			b, c = c, b
			ba = Sub2(b, a)
		}

		cb := Sub2(c, b)
		ac := Sub2(a, c)

		bestScore = 0.0
		bestSeparation = FLT_MAX
		bestIndex = NULL_INDEX
		for i: i32 = 0; i < count1; i += 1 {
			p := points[i].p
			u1 := Cross2(Sub2(p, a), ba)
			u2 := Cross2(Sub2(p, b), cb)
			u3 := Cross2(Sub2(p, c), ac)
			score := MaxFloat(u1, MaxFloat(u2, u3))

			// Use the area tolerance for collinear points and hysteresis
			if IsBetterCullCandidate(score, points[i].separation, bestScore, bestSeparation, tolSqr, separationTol) {
				bestScore = score
				bestSeparation = points[i].separation
				bestIndex = i
			}
		}

		if bestIndex == NULL_INDEX {
			// No additional area
			points[0] = finalPoints[0]
			points[1] = finalPoints[1]
			points[2] = finalPoints[2]
			return 3
		}

		// Store best point
		finalPoints[3] = points[bestIndex]

		// Full quad
		points[0] = finalPoints[0]
		points[1] = finalPoints[1]
		points[2] = finalPoints[2]
		points[3] = finalPoints[3]
		return 4
	}

}

@(private = "file")
ReduceCluster :: proc(points: [^]LocalManifoldPoint, count1: i32, normal: Vec3, arena: Arena) -> i32 {
	arena := arena
	targetCount: i32 = 1
	if count1 <= targetCount {
		return count1
	}

	pts := cast([^]Point2D)Bump(&arena, count1 * size_of(Point2D))
	u := Perp(normal)
	v := Cross(normal, u)
	origin := points[0].point

	for i: i32 = 0; i < count1; i += 1 {
		d := Sub(points[i].point, origin)
		pts[i].p = Vec2{Dot(d, u), Dot(d, v)}
		pts[i].separation = points[i].separation
		pts[i].originalIndex = i
	}

	hull: [^]Point2D
	count2: i32
	when MAX_MANIFOLD_POINTS == 4 {
		hull = pts
		count2 = CullPoints(pts, count1)
	} else {
		hull = cast([^]Point2D)Bump(&arena, 2 * count1 * size_of(Point2D))
		hullCount := Hull2D(pts, count1, hull)
		count2 = SimplifyHull2D(hull, hullCount, MAX_MANIFOLD_POINTS)
	}
	assert(count2 <= MAX_MANIFOLD_POINTS)

	finalPoints: [MAX_MANIFOLD_POINTS]LocalManifoldPoint
	for i: i32 = 0; i < count2; i += 1 {
		index := hull[i].originalIndex
		assert(0 <= index && index < count1)
		finalPoints[i] = points[index]
	}

	mem.copy_non_overlapping(points, &finalPoints[0], int(count2) * size_of(LocalManifoldPoint))
	return count2
}

Cluster :: struct {
	manifoldNormal: Vec3,
	triangleNormal: Vec3,
	points:         [^]LocalManifoldPoint,
	pointCapacity:  i32,
	pointCount:     i32,
}

// unified: QSORT's LESS compares two elements (tentativeTriangles[i] < tentativeTriangles[j] in C)
@(private = "file")
TentativeTriangleLess :: proc(a, b: TentativeTriangle) -> bool {
	return a.squaredDistance < b.squaredDistance
}
// unified end

ComputeMeshManifolds :: proc(
	world: ^World,
	workerIndex: i32,
	contact: ^Contact,
	shapeA: ^Shape,
	materialMap: [^]u16,
	xfA: WorldTransform,
	shapeB: ^Shape,
	xfB: WorldTransform,
	isFast: bool,
	arena: Arena,
) -> bool {
	arena := arena
	assert(shapeA.type == .meshShape || shapeA.type == .heightShape)
	_ = workerIndex
	_ = isFast
	_ = materialMap

	context_ := Array_Get(&world.taskContexts, workerIndex)

	RefreshCache(contact, shapeA, xfA, &shapeB.aabb, arena)

	// Collide with triangles and build manifolds
	meshContact := &contact.meshContact
	triangleCount := meshContact.triangleCache.count

	acceptedManifolds := cast([^]^LocalManifold)Bump(&arena, triangleCount * size_of(^LocalManifold))
	acceptedManifoldCount: i32 = 0
	tentativeManifolds := cast([^]^LocalManifold)Bump(&arena, triangleCount * size_of(^LocalManifold))
	tentativeManifoldCount: i32 = 0
	tentativeTriangles := cast([^]TentativeTriangle)Bump(&arena, triangleCount * size_of(TentativeTriangle))
	tentativeTriangleCount: i32 = 0

	foundEdges: FoundEdges
	foundVertices: FoundVertices
	foundEdges.count = 0
	foundVertices.count = 0

	// This transform converts from mesh frame into the shapeB frame
	transformAtoB := InvMulWorldTransforms(xfB, xfA)
	relativeMatrix := MakeMatrixFromQuat(transformAtoB.q)
	linearSlop := LINEAR_SLOP

	// This should push apart shapes after a time of impact event.
	// In the past I've called this `polygon skin`, but PhysX and Unreal
	// call it `rest offset` which seems appropriate in this case.
	// It leads to a small visual gap but seems to improve the quality of mesh
	// collision, especially for hull versus mesh.
	restOffset := MESH_REST_OFFSET
	enableSpeculative := (contact.flags & enableSpeculativePoints) != 0

	// Make room for clip points
	pointBufferCapacity := MAX_POINTS_PER_TRIANGLE * triangleCount

	pointBuffer := cast([^]LocalManifoldPoint)Bump(&arena, pointBufferCapacity * size_of(LocalManifoldPoint))
	totalPointCount: i32 = 0

	manifoldBuffer := cast([^]LocalManifold)Bump(&arena, triangleCount * size_of(LocalManifold))
	manifoldCount: i32 = 0

	triangleCaches := meshContact.triangleCache.data

	hullB: ^HullData = shapeB.type == .hullShape ? shapeB.hull : nil
	hullSpeculativeDistance := enableSpeculative ? SPECULATIVE_DISTANCE : 0.0
	hullMargin := Vec3{hullSpeculativeDistance, hullSpeculativeDistance, hullSpeculativeDistance}

	survivorIndices := cast([^]i32)Bump(&arena, triangleCount * size_of(i32))
	survivorVertices := cast([^]Vec3)Bump(&arena, 3 * triangleCount * size_of(Vec3))
	survivorCount: i32 = 0

	meshTriangles: [^]MeshTriangle = nil
	meshVertices: [^]Vec3 = nil
	meshScale := Vec3_one
	meshFlipped := false
	if shapeA.type == .meshShape {
		meshTriangles = GetMeshTriangles(shapeA.mesh.data)
		meshVertices = GetMeshVertices(shapeA.mesh.data)
		meshScale = shapeA.mesh.scale
		meshFlipped = meshScale.x * meshScale.y * meshScale.z < 0.0
	}

	cxx := SplatW4(relativeMatrix.cx.x)
	cxy := SplatW4(relativeMatrix.cx.y)
	cxz := SplatW4(relativeMatrix.cx.z)
	cyx := SplatW4(relativeMatrix.cy.x)
	cyy := SplatW4(relativeMatrix.cy.y)
	cyz := SplatW4(relativeMatrix.cy.z)
	czx := SplatW4(relativeMatrix.cz.x)
	czy := SplatW4(relativeMatrix.cz.y)
	czz := SplatW4(relativeMatrix.cz.z)
	px := SplatW4(transformAtoB.p.x)
	py := SplatW4(transformAtoB.p.y)
	pz := SplatW4(transformAtoB.p.z)

	marginW := SplatW4(hullMargin.x)
	hullBounds := hullB != nil ? hullB.aabb : AABB{}
	hullLowerX := SplatW4(hullBounds.lowerBound.x)
	hullLowerY := SplatW4(hullBounds.lowerBound.y)
	hullLowerZ := SplatW4(hullBounds.lowerBound.z)
	hullUpperX := SplatW4(hullBounds.upperBound.x)
	hullUpperY := SplatW4(hullBounds.upperBound.y)
	hullUpperZ := SplatW4(hullBounds.upperBound.z)

	for baseIndex: i32 = 0; baseIndex < triangleCount; baseIndex += 4 {
		laneCount := MinInt(4, triangleCount - baseIndex)

		lanes: [3][4]Vec3
		for lane: i32 = 0; lane < 4; lane += 1 {
			index := baseIndex + MinInt(lane, laneCount - 1)
			triangleIndex := triangleCaches[index].triangleIndex

			if meshVertices != nil {
				meshTriangle := meshTriangles[triangleIndex]
				lanes[0][lane] = Mul(meshScale, meshVertices[meshTriangle.index1])
				lanes[1][lane] = Mul(meshScale, meshVertices[meshFlipped ? meshTriangle.index3 : meshTriangle.index2])
				lanes[2][lane] = Mul(meshScale, meshVertices[meshFlipped ? meshTriangle.index2 : meshTriangle.index3])
			} else {
				assert(shapeA.type == .heightShape)
				triangle := GetHeightFieldTriangle(shapeA.heightField, triangleIndex)
				lanes[0][lane] = triangle.vertices[0]
				lanes[1][lane] = triangle.vertices[1]
				lanes[2][lane] = triangle.vertices[2]
			}
		}

		// Transform triangle into the shape frame
		vx, vy, vz: [3]FloatW4
		for k := 0; k < 3; k += 1 {
			ax := SetW4(lanes[k][0].x, lanes[k][1].x, lanes[k][2].x, lanes[k][3].x)
			ay := SetW4(lanes[k][0].y, lanes[k][1].y, lanes[k][2].y, lanes[k][3].y)
			az := SetW4(lanes[k][0].z, lanes[k][1].z, lanes[k][2].z, lanes[k][3].z)

			vx[k] = AddW4(AddW4(AddW4(MulW4(cxx, ax), MulW4(cyx, ay)), MulW4(czx, az)), px)
			vy[k] = AddW4(AddW4(AddW4(MulW4(cxy, ax), MulW4(cyy, ay)), MulW4(czy, az)), py)
			vz[k] = AddW4(AddW4(AddW4(MulW4(cxz, ax), MulW4(cyz, ay)), MulW4(czz, az)), pz)
		}

		keepMask: u32 = (u32(1) << u32(laneCount)) - 1
		if hullB != nil {
			lowerX := SubW4(MinW4(vx[0], MinW4(vx[1], vx[2])), marginW)
			lowerY := SubW4(MinW4(vy[0], MinW4(vy[1], vy[2])), marginW)
			lowerZ := SubW4(MinW4(vz[0], MinW4(vz[1], vz[2])), marginW)
			upperX := AddW4(MaxW4(vx[0], MaxW4(vx[1], vx[2])), marginW)
			upperY := AddW4(MaxW4(vy[0], MaxW4(vy[1], vy[2])), marginW)
			upperZ := AddW4(MaxW4(vz[0], MaxW4(vz[1], vz[2])), marginW)

			separated := OrW4(LessThanW4(upperX, hullLowerX), GreaterThanW4(lowerX, hullUpperX))
			separated = OrW4(separated, OrW4(LessThanW4(upperY, hullLowerY), GreaterThanW4(lowerY, hullUpperY)))
			separated = OrW4(separated, OrW4(LessThanW4(upperZ, hullLowerZ), GreaterThanW4(lowerZ, hullUpperZ)))

			cullMask := keepMask & u32(MoveMaskW4(separated))
			keepMask &= ~cullMask

			for cullMask != 0 {
				lane := CTZ32(cullMask)
				cullMask &= cullMask - 1
				triangleCaches[baseIndex + i32(lane)].cache.satCache = SATCache{}
			}
		}

		if keepMask == 0 {
			continue
		}

		x, y, z: [3][4]f32
		for k := 0; k < 3; k += 1 {
			StoreW4(&x[k][0], vx[k])
			StoreW4(&y[k][0], vy[k])
			StoreW4(&z[k][0], vz[k])
		}

		for keepMask != 0 {
			lane := CTZ32(keepMask)
			keepMask &= keepMask - 1

			survivor := survivorVertices[3 * survivorCount:]
			survivor[0] = Vec3{x[0][lane], y[0][lane], z[0][lane]}
			survivor[1] = Vec3{x[1][lane], y[1][lane], z[1][lane]}
			survivor[2] = Vec3{x[2][lane], y[2][lane], z[2][lane]}
			survivorIndices[survivorCount] = baseIndex + i32(lane)
			survivorCount += 1
		}
	}

	for survivorIndex: i32 = 0;
	    survivorIndex < survivorCount && totalPointCount + 3 < pointBufferCapacity;
	    survivorIndex += 1 {
		index := survivorIndices[survivorIndex]
		triangleIndex := triangleCaches[index].triangleIndex
		vertices := survivorVertices[3 * survivorIndex:]

		triangle: Triangle
		if shapeA.type == .meshShape {
			triangle = GetMeshTriangle(&shapeA.mesh, triangleIndex)
		} else {
			assert(shapeA.type == .heightShape)
			triangle = GetHeightFieldTriangle(shapeA.heightField, triangleIndex)
		}

		cache := &triangleCaches[index].cache
		pointCapacity := pointBufferCapacity - totalPointCount
		manifold := &manifoldBuffer[manifoldCount]
		manifold.points = pointBuffer[totalPointCount:]
		manifold.pointCount = 0
		manifold.triangleFlags = triangle.flags
		manifold.feature = .featureNone

		#partial switch shapeB.type {
		case .capsuleShape:
			CollideTriangleAndCapsule(manifold, pointCapacity, vertices, &shapeB.capsule, &cache.simplexCache)

		case .hullShape:
			// Cached edge contact is dangerous at high speed because the hull can rotate around the edge and tunnel
			// through the triangle.
			if isFast && cache.satCache.type == u8(SeparatingFeature.edgePairAxis) {
				cache.satCache = SATCache{}
			}

			CollideTriangleAndHull(
				manifold,
				pointCapacity,
				vertices[0],
				vertices[1],
				vertices[2],
				triangle.flags,
				hullB,
				&cache.satCache,
				enableSpeculative,
			)
			context_.satCallCount += 1
			context_.satCacheHitCount += i32(cache.satCache.hit)

		case .sphereShape:
			CollideTriangleAndSphere(manifold, pointCapacity, vertices, &shapeB.sphere)

		case:
			assert(false)
			return false
		}

		manifoldPointCount := manifold.pointCount

		if manifoldPointCount > 0 {
			assert(manifold.feature != .featureNone)

			manifoldCount += 1
			totalPointCount += manifoldPointCount
			manifold.triangleIndex = triangleIndex
			manifold.triangleNormal = MakeNormalFromPoints(vertices[0], vertices[1], vertices[2])
			manifold.i1 = triangle.i1
			manifold.i2 = triangle.i2
			manifold.i3 = triangle.i3

			if manifold.feature == .featureTriangleFace || FORCE_GHOST_COLLISIONS {
				_ = AddEdge(&foundEdges, triangle.i1, triangle.i2)
				_ = AddEdge(&foundEdges, triangle.i2, triangle.i3)
				_ = AddEdge(&foundEdges, triangle.i3, triangle.i1)
				_ = AddVertex(&foundVertices, triangle.i1)
				_ = AddVertex(&foundVertices, triangle.i2)
				_ = AddVertex(&foundVertices, triangle.i3)

				acceptedManifolds[acceptedManifoldCount] = manifold
				acceptedManifoldCount += 1
			} else if manifold.feature == .featureHullFace {
				cosNormalAngle := Dot(manifold.triangleNormal, manifold.normal)
				if cosNormalAngle > 0.5 {
					_ = AddEdge(&foundEdges, triangle.i1, triangle.i2)
					_ = AddEdge(&foundEdges, triangle.i2, triangle.i3)
					_ = AddEdge(&foundEdges, triangle.i3, triangle.i1)
					_ = AddVertex(&foundVertices, triangle.i1)
					_ = AddVertex(&foundVertices, triangle.i2)
					_ = AddVertex(&foundVertices, triangle.i3)

					acceptedManifolds[acceptedManifoldCount] = manifold
					acceptedManifoldCount += 1
				} else {
					minSeparation := manifold.points[0].separation
					for i: i32 = 1; i < manifoldPointCount; i += 1 {
						minSeparation = MinFloat(minSeparation, manifold.points[i].separation)
					}

					if minSeparation < -2.0 * linearSlop {
						// Deep overlap
						_ = AddEdge(&foundEdges, triangle.i1, triangle.i2)
						_ = AddEdge(&foundEdges, triangle.i2, triangle.i3)
						_ = AddEdge(&foundEdges, triangle.i3, triangle.i1)
						_ = AddVertex(&foundVertices, triangle.i1)
						_ = AddVertex(&foundVertices, triangle.i2)
						_ = AddVertex(&foundVertices, triangle.i3)
						acceptedManifolds[acceptedManifoldCount] = manifold
						acceptedManifoldCount += 1
					} else {
						tentativeTriangle := TentativeTriangle {
							squaredDistance = manifold.squaredDistance,
							index           = tentativeManifoldCount,
						}
						tentativeTriangles[tentativeTriangleCount] = tentativeTriangle
						tentativeTriangleCount += 1
						tentativeManifolds[tentativeManifoldCount] = manifold
						tentativeManifoldCount += 1
					}
				}
			} else {
				tentativeTriangle := TentativeTriangle {
					squaredDistance = manifold.squaredDistance,
					index           = tentativeManifoldCount,
				}
				tentativeTriangles[tentativeTriangleCount] = tentativeTriangle
				tentativeTriangleCount += 1
				tentativeManifolds[tentativeManifoldCount] = manifold
				tentativeManifoldCount += 1
			}
		}
	}

	assert(acceptedManifoldCount <= triangleCount)
	assert(tentativeManifoldCount <= triangleCount)
	assert(tentativeTriangleCount <= triangleCount)

	if shapeB.type == .sphereShape {
		// Sort triangles so the closest triangles are processed first
		{
			QSORT(tentativeTriangles, tentativeTriangleCount, TentativeTriangleLess)
		}

		// Add tentative manifolds in sorted order. Avoid adding manifolds that generate ghost collisions.
		for i: i32 = 0; i < tentativeTriangleCount; i += 1 {
			m := tentativeManifolds[tentativeTriangles[i].index]

			addedEdge1 := AddEdge(&foundEdges, m.i1, m.i2)
			addedEdge2 := AddEdge(&foundEdges, m.i2, m.i3)
			addedEdge3 := AddEdge(&foundEdges, m.i3, m.i1)
			addedVertex1 := AddVertex(&foundVertices, m.i1)
			addedVertex2 := AddVertex(&foundVertices, m.i2)
			addedVertex3 := AddVertex(&foundVertices, m.i3)

			feature := m.feature
			shouldCollide := false
			#partial switch feature {
			case .featureNone, .featureTriangleFace:
				assert(false)

			case .featureEdge1:
				shouldCollide = addedEdge1

			case .featureEdge2:
				shouldCollide = addedEdge2

			case .featureEdge3:
				shouldCollide = addedEdge3

			case .featureVertex1:
				shouldCollide = addedVertex1

			case .featureVertex2:
				shouldCollide = addedVertex2

			case .featureVertex3:
				shouldCollide = addedVertex3

			case:
				assert(false)
			}

			if shouldCollide == true {
				acceptedManifolds[acceptedManifoldCount] = m
				acceptedManifoldCount += 1
			}
		}
	} else {
		// Problem: hull can tunnel if time of impact is at concave edge
		// Example: flat box sliding down a ramp to a flat bottom
		// Solution: only ignore flat edges
		for i: i32 = 0; i < tentativeManifoldCount; i += 1 {
			m := tentativeManifolds[i]
			triangleFlags := m.triangleFlags

			if (triangleFlags & allFlatEdges) == allFlatEdges {
				continue
			}

			if (triangleFlags & flatEdge1) == flatEdge1 {
				if FindEdge(&foundEdges, m.i1, m.i2) {
					continue
				}
			}

			if (triangleFlags & flatEdge2) == flatEdge2 {
				if FindEdge(&foundEdges, m.i2, m.i3) {
					continue
				}
			}

			if (triangleFlags & flatEdge3) == flatEdge3 {
				if FindEdge(&foundEdges, m.i3, m.i1) {
					continue
				}
			}

			acceptedManifolds[acceptedManifoldCount] = m
			acceptedManifoldCount += 1
		}
	}

	assert(acceptedManifoldCount <= triangleCount)

	if acceptedManifoldCount == 0 {
		if contact.manifoldCount > 0 {
			FreeManifolds(world, contact.manifolds, contact.manifoldCount)
			contact.manifolds = nil
			contact.manifoldCount = 0
		}
		return false
	}

	clusters := cast([^]Cluster)Bump(&arena, acceptedManifoldCount * size_of(Cluster))
	clusterMemberships := cast([^]i32)Bump(&arena, acceptedManifoldCount * size_of(i32))

	// Cluster tolerance is tighter than the warm starting manifold matching tolerance. These
	// serve different purposes.
	clusterThreshold: f32 = 0.996
	clusterCount: i32 = 0
	clusterPointCount: i32 = 0
	for i: i32 = 0; i < acceptedManifoldCount; i += 1 {
		clusterMemberships[i] = NULL_INDEX

		manifold := acceptedManifolds[i]
		clusterPointCount += manifold.pointCount

		// Cluster based on the triangle normal and contact normal.
		// The first cluster found is accepted because the tolerance is tight.
		// todo consider requiring the triangles to be connect by an edge.
		// todo consider looking for the best cluster instead of the first one within tolerance
		// This bool is here to allow quick testing with and without clustering.
		allowClustering := true
		manifoldNormal := manifold.normal
		triangleNormal := manifold.triangleNormal
		clusterIndex: i32 = NULL_INDEX
		for j: i32 = 0; j < clusterCount && allowClustering; j += 1 {
			cosManifoldAngle := Dot(clusters[j].manifoldNormal, manifoldNormal)
			cosTriangleAngle := Dot(clusters[j].triangleNormal, triangleNormal)
			if cosManifoldAngle <= clusterThreshold || cosTriangleAngle <= clusterThreshold {
				continue
			}

			// unified: not ported: the disabled (#if 0) edge-connected cluster test; the #else branch follows.

			// Found a cluster
			clusterIndex = j
			break
		}

		if clusterIndex != NULL_INDEX {
			clusterMemberships[i] = clusterIndex
			clusters[clusterIndex].pointCapacity += manifold.pointCount
		} else {
			clusters[clusterCount].manifoldNormal = manifoldNormal
			clusters[clusterCount].triangleNormal = triangleNormal
			clusters[clusterCount].pointCapacity = manifold.pointCount
			clusterMemberships[i] = clusterCount
			clusterCount += 1
		}
	}

	if clusterPointCount == 0 {
		return false
	}

	// Setup clusters
	clusterPoints := cast([^]LocalManifoldPoint)Bump(&arena, clusterPointCount * size_of(LocalManifoldPoint))
	pointOffset: i32 = 0

	for i: i32 = 0; i < clusterCount; i += 1 {
		cluster := &clusters[i]
		cluster.points = clusterPoints[pointOffset:]
		cluster.pointCount = 0
		pointOffset += cluster.pointCapacity
	}

	// Populate clusters
	for i: i32 = 0; i < acceptedManifoldCount; i += 1 {
		clusterIndex := clusterMemberships[i]
		if clusterIndex == NULL_INDEX {
			continue
		}

		assert(0 <= clusterIndex && clusterIndex < clusterCount)

		am := acceptedManifolds[i]
		cm := &clusters[clusterIndex]
		for j: i32 = 0; j < am.pointCount; j += 1 {
			assert(cm.pointCount < cm.pointCapacity)
			ap := &am.points[j]
			cp := &cm.points[cm.pointCount]

			cp.triangleIndex = am.triangleIndex
			cp.point = ap.point
			cp.separation = ap.separation
			cp.pair = ap.pair
			cm.pointCount += 1
		}
	}

	// Simplify clusters
	for i: i32 = 0; i < clusterCount; i += 1 {
		cm := &clusters[i]
		assert(cm.pointCount == cm.pointCapacity)
		reducedCount := ReduceCluster(cm.points, cm.pointCount, cm.triangleNormal, arena)
		cm.pointCount = reducedCount
	}

	// Make a temporary copy of previous manifolds
	oldManifoldCount := contact.manifoldCount
	oldManifolds: [^]Manifold = nil
	if oldManifoldCount > 0 {
		oldManifolds = cast([^]Manifold)Bump(&arena, oldManifoldCount * size_of(Manifold))
		mem.copy_non_overlapping(oldManifolds, contact.manifolds, int(oldManifoldCount) * size_of(Manifold))
	}

	// Resize manifolds if needed
	if oldManifoldCount != clusterCount {
		FreeManifolds(world, contact.manifolds, contact.manifoldCount)
		contact.manifolds = AllocateManifolds(world, clusterCount)
		contact.manifoldCount = i32(u16(clusterCount))
	} else {
		// Mem zero manifolds
		mem.zero(contact.manifolds, int(contact.manifoldCount) * size_of(Manifold))
	}

	consumed: [^]bool = nil
	if oldManifoldCount > 0 {
		consumed = cast([^]bool)Bump(&arena, oldManifoldCount * size_of(bool))
		mem.zero(consumed, int(oldManifoldCount) * size_of(bool))
	}

	matrixB := MakeMatrixFromQuat(xfB.q)
	offsetA := SubPos(xfB.p, xfA.p)

	normalMatchTolerance: f32 = 0.995
	for i: i32 = 0; i < clusterCount; i += 1 {
		cm := &clusters[i]
		pointCount := cm.pointCount
		assert(0 < pointCount && pointCount <= MAX_MANIFOLD_POINTS)

		manifold := &contact.manifolds[i]
		manifold.pointCount = pointCount
		manifold.normal = MulMV(matrixB, cm.manifoldNormal)

		clusterNormal := MulMV(matrixB, cm.manifoldNormal)
		bestDot := normalMatchTolerance
		bestIndex: i32 = NULL_INDEX

		for j: i32 = 0; j < oldManifoldCount; j += 1 {
			if consumed[j] == true {
				continue
			}

			dot := Dot(oldManifolds[j].normal, clusterNormal)
			if dot > bestDot {
				bestIndex = j
				bestDot = dot
			}
		}

		matchedManifold: ^Manifold = nil
		if bestIndex != NULL_INDEX {
			matchedManifold = &oldManifolds[bestIndex]
			manifold.frictionImpulse = matchedManifold.frictionImpulse
			manifold.rollingImpulse = matchedManifold.rollingImpulse
			manifold.twistImpulse = matchedManifold.twistImpulse
			consumed[bestIndex] = true
		}

		for j: i32 = 0; j < pointCount; j += 1 {
			source := &cm.points[j]
			target := &manifold.points[j]

			// Contact points are computed in frame B
			target.anchorB = MulMV(matrixB, source.point)
			target.anchorA = Add(target.anchorB, offsetA)
			target.separation = source.separation - restOffset
			target.featureId = MakeFeatureId(source.pair)
			target.triangleIndex = source.triangleIndex

			// Preserve normal impulse if possible
			if matchedManifold != nil {
				oldPointCount := matchedManifold.pointCount
				for k: i32 = 0; k < oldPointCount; k += 1 {
					oldPt := &matchedManifold.points[k]

					if target.featureId == oldPt.featureId && target.triangleIndex == oldPt.triangleIndex {
						target.normalImpulse = oldPt.normalImpulse
						target.persisted = true

						// claimed
						oldPt.triangleIndex = NULL_INDEX
						break
					}
				}
			}
		}
	}

	materialsA := GetShapeMaterials(shapeA)
	materialB := &GetShapeMaterials(shapeB)[0]
	tangentVelocityA := Vec3_zero

	// Update friction and restitution if the mesh has per triangle material
	if shapeA.materialCount > 0 {
		friction: f32 = 0.0
		restitution: f32 = 0.0
		sampleCount: f32 = 0.0

		materialIndices: [^]u8
		if shapeA.type == .meshShape {
			materialIndices = GetMeshMaterialIndices(shapeA.mesh.data)
		} else {
			materialIndices = GetHeightFieldMaterialIndices(shapeA.heightField)
		}

		for i: i32 = 0; i < clusterCount; i += 1 {
			manifold := &contact.manifolds[i]
			pointCount := manifold.pointCount
			for j: i32 = 0; j < pointCount; j += 1 {
				triangleIndex := manifold.points[j].triangleIndex
				materialIndex: i32
				if shapeA.type == .meshShape {
					materialIndex = i32(materialIndices[triangleIndex])

					if materialMap != nil {
						materialIndex = i32(materialMap[materialIndex])
					}
				} else {
					materialIndex = i32(materialIndices[triangleIndex >> 1])
				}

				materialIndex = ClampInt(materialIndex, 0, shapeA.materialCount - 1)
				material := materialsA[materialIndex]
				friction += world.frictionCallback(
					material.friction,
					material.userMaterialId,
					materialB.friction,
					materialB.userMaterialId,
				)
				restitution += world.restitutionCallback(
					material.restitution,
					material.userMaterialId,
					materialB.restitution,
					materialB.userMaterialId,
				)

				tangentVelocityA = Add(tangentVelocityA, material.tangentVelocity)

				sampleCount += 1.0
			}
		}

		if sampleCount > 0.0 {
			invCount := 1.0 / sampleCount
			contact.friction = invCount * friction
			contact.restitution = invCount * restitution
			tangentVelocityA = MulSV(invCount, tangentVelocityA)
		}

		assert(IsValidFloat(contact.friction) && contact.friction >= 0.0)
		assert(IsValidFloat(contact.restitution) && contact.restitution >= 0.0)
	} else {
		// Keep these updated in case the values on the shapes are modified
		contact.friction = world.frictionCallback(
			materialsA[0].friction,
			materialsA[0].userMaterialId,
			materialB.friction,
			materialB.userMaterialId,
		)
		contact.restitution = world.restitutionCallback(
			materialsA[0].restitution,
			materialsA[0].userMaterialId,
			materialB.restitution,
			materialB.userMaterialId,
		)
		tangentVelocityA = materialsA[0].tangentVelocity
	}

	tangentVelocityA = RotateVector(xfA.q, tangentVelocityA)

	radiusB: f32 = 0.0
	if shapeB.type == .sphereShape {
		radiusB = shapeB.sphere.radius
	} else if shapeB.type == .capsuleShape {
		radiusB = shapeB.capsule.radius
	} else if shapeB.type == .hullShape {
		radiusB = shapeB.hull.innerRadius
	}

	contact.rollingResistance = materialB.rollingResistance * radiusB

	tangentVelocityB := RotateVector(xfB.q, materialB.tangentVelocity)
	contact.tangentVelocity = Sub(tangentVelocityA, tangentVelocityB)
	return true
}
