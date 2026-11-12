// Ported from box3d src/mesh.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:runtime"
import "core:mem"

// Dirk Gregorius contributed portions of this code

@(private = "file")
BIN_COUNT :: 8
@(private = "file")
DESIRED_TRIANGLES_PER_LEAF :: 4
@(private = "file")
LEAF_NODE :: 3
@(private = "file")
MAXIMUM_TRIANGLES_PER_LEAF :: 8
@(private = "file")
MESH_STACK_SIZE :: 256

@(private = "file")
IsLeaf :: proc(node: ^MeshNode) -> bool {
	return node.data.asLeaf.type == LEAF_NODE
}

@(private = "file")
GetMeshNodesWrite :: proc(mesh: ^MeshData) -> [^]MeshNode {
	if mesh.nodeOffset == 0 {
		return nil
	}

	return cast([^]MeshNode)(uintptr(mesh) + uintptr(mesh.nodeOffset))
}

@(private = "file")
GetLeftChildWrite :: proc(node: ^MeshNode) -> ^MeshNode {
	// The left child follows its parent.
	assert(!IsLeaf(node))
	return &([^]MeshNode)(node)[1]
}

@(private = "file")
GetLeftChild :: proc(node: ^MeshNode) -> ^MeshNode {
	// The left child follows its parent.
	assert(!IsLeaf(node))
	return &([^]MeshNode)(node)[1]
}

@(private = "file")
GetRightChildWrite :: proc(node: ^MeshNode) -> ^MeshNode {
	// We store the offset of the right child relative to its parent
	assert(!IsLeaf(node))
	return &([^]MeshNode)(node)[node.data.asNode.childOffset]
}

@(private = "file")
GetRightChild :: proc(node: ^MeshNode) -> ^MeshNode {
	// We store the offset of the right child relative to its parent
	assert(!IsLeaf(node))
	return &([^]MeshNode)(node)[node.data.asNode.childOffset]
}

@(private = "file")
GetRoot :: proc(mesh: ^MeshData) -> ^MeshNode {
	// The first node is the root
	return (^MeshNode)(GetMeshNodes(mesh))
}

@(private = "file")
GetRootWrite :: proc(mesh: ^MeshData) -> ^MeshNode {
	// The first node is the root
	return (^MeshNode)(GetMeshNodesWrite(mesh))
}

@(private = "file")
GetMeshTrianglesWrite :: proc(mesh: ^MeshData) -> [^]MeshTriangle {
	if mesh.triangleOffset == 0 {
		return nil
	}

	return cast([^]MeshTriangle)(uintptr(mesh) + uintptr(mesh.triangleOffset))
}

@(private = "file")
GetMeshVerticesWrite :: proc(mesh: ^MeshData) -> [^]Vec3 {
	if mesh.vertexOffset == 0 {
		return nil
	}

	return cast([^]Vec3)(uintptr(mesh) + uintptr(mesh.vertexOffset))
}

// static b3Vec3 b3GetVertex( b3MeshData& mesh, int vertexIndex )
//{
//	B3_ASSERT( 0 <= vertexIndex && vertexIndex < mesh.vertexCount );
//	b3Vec3* vertices = b3GetMeshVertices( &mesh );
//	return vertices[vertexIndex];
// }

@(private = "file")
GetMeshMaterialIndicesWrite :: proc(mesh: ^MeshData) -> [^]u8 {
	if mesh.materialOffset == 0 {
		return nil
	}

	return cast([^]u8)(uintptr(mesh) + uintptr(mesh.materialOffset))
}

@(private = "file")
GetMeshFlagsWrite :: proc(mesh: ^MeshData) -> [^]u8 {
	if mesh.flagsOffset == 0 {
		return nil
	}

	return cast([^]u8)(uintptr(mesh) + uintptr(mesh.flagsOffset))
}

@(private = "file")
GetNodeHeight :: proc(node: ^MeshNode) -> i32 {
	if IsLeaf(node) {
		return 0
	}

	leftChild := GetLeftChild(node)
	leftHeight := GetNodeHeight(leftChild)
	rightChild := GetRightChild(node)
	rightHeight := GetNodeHeight(rightChild)

	return 1 + MaxInt(leftHeight, rightHeight)
}

GetHeight :: proc(mesh: ^MeshData) -> i32 {
	root := GetRoot(mesh)
	if root == nil {
		return 0
	}

	return GetNodeHeight(root)
}

when ENABLE_VALIDATION {

	@(private = "file")
	IsDegenerate :: proc(v1, v2, v3: Vec3, minArea: f32) -> bool {
		normal := Cross(Sub(v2, v1), Sub(v3, v1))
		lengthSq := LengthSquared(normal)
		return lengthSq < minArea * minArea
	}

	@(private = "file")
	IsNonDegenerate :: proc(mesh: ^MeshData, minArea: f32) -> bool {
		triangles := GetMeshTriangles(mesh)
		vertices := GetMeshVertices(mesh)

		// Check triangles
		for index: i32 = 0; index < mesh.triangleCount; index += 1 {
			// Index range
			triangle := triangles[index]
			if triangle.index1 >= mesh.vertexCount {
				return false
			}

			if triangle.index2 >= mesh.vertexCount {
				return false
			}

			if triangle.index3 >= mesh.vertexCount {
				return false
			}

			// Degenerate topology
			if triangle.index1 == triangle.index2 {
				return false
			}
			if triangle.index1 == triangle.index3 {
				return false
			}
			if triangle.index2 == triangle.index3 {
				return false
			}

			// Degenerate geometry
			vertex1 := vertices[triangle.index1]
			vertex2 := vertices[triangle.index2]
			vertex3 := vertices[triangle.index3]
			if IsDegenerate(vertex1, vertex2, vertex3, minArea) {
				return false
			}
		}

		return true
	}

	@(private = "file")
	GetNodeAABB :: proc(node: ^MeshNode) -> AABB {
		return AABB{node.lowerBound, node.upperBound}
	}

	@(private = "file")
	IsConsistent :: proc(mesh: ^MeshData) -> bool {
		triangles := GetMeshTriangles(mesh)
		vertices := GetMeshVertices(mesh)

		// Check nodes
		count: i32 = 0
		stack: [64]^MeshNode = ---
		stack[count] = GetRoot(mesh)
		count += 1

		for count > 0 {
			count -= 1
			node := stack[count]
			nodeBounds := GetNodeAABB(node)

			if IsLeaf(node) == false {
				child1 := GetLeftChild(node)
				bounds1 := GetNodeAABB(child1)
				child2 := GetRightChild(node)
				bounds2 := GetNodeAABB(child2)

				if AABB_Contains(nodeBounds, bounds1) == false {
					return false
				}

				if AABB_Contains(nodeBounds, bounds2) == false {
					return false
				}

				stack[count] = child2
				count += 1
				stack[count] = child1
				count += 1
			} else {
				triangleBounds := BOUNDS3_EMPTY
				for index: u32 = 0; index < node.data.asLeaf.triangleCount; index += 1 {
					triangleIndex := i32(node.triangleOffset + index)
					assert(0 <= triangleIndex && triangleIndex < mesh.triangleCount)

					triangle := triangles[triangleIndex]

					vertexBounds := BOUNDS3_EMPTY
					vertexBounds = AABB_AddPoint(vertexBounds, vertices[triangle.index1])
					vertexBounds = AABB_AddPoint(vertexBounds, vertices[triangle.index2])
					vertexBounds = AABB_AddPoint(vertexBounds, vertices[triangle.index3])

					triangleBounds = AABB_Union(triangleBounds, vertexBounds)
				}

				if !AABB_Contains(nodeBounds, triangleBounds) {
					return false
				}
			}
		}

		return true
	}

	IsValidMesh :: proc(meshData: ^MeshData) -> bool {
		if meshData == nil {
			return false
		}

		if meshData.version != MESH_VERSION {
			return false
		}

		if meshData.byteCount < i32(size_of(MeshData)) {
			return false
		}

		return IsConsistent(meshData)
	}

} else {

	IsValidMesh :: proc(meshData: ^MeshData) -> bool {
		if meshData == nil {
			return false
		}

		if meshData.version != MESH_VERSION {
			return false
		}

		if meshData.byteCount < i32(size_of(MeshData)) {
			return false
		}

		return true
	}

}

@(private = "file")
GetStridedVertex :: #force_inline proc(vertices: [^]Vec3, index: i32, stride: int) -> Vec3 {
	return (cast(^Vec3)(uintptr(vertices) + uintptr(int(index) * stride)))^
}

// Node for a vertex linked list
VertexNode :: struct {
	vertexIndex:   i32,
	nextNodeIndex: i32,
}

// unified: verstable's b3VertexMap (u64 -> int) is an Odin map on the heap allocator; it is only used for lookups
// and updates, never iterated. Its storage is not counted by GetByteCount.
VertexMap :: map[u64]i32
// unified end

SpatialHash :: struct {
	nodes:       Array(VertexNode),
	vertices:    [^]Vec3,
	vertexCount: i32,
	stride:      int,
	vertexMap:   VertexMap,
	cellSize:    f32,
	tolerance:   f32,
}

@(private = "file")
CreateSpatialHash :: proc(h: ^SpatialHash, vertices: [^]Vec3, vertexCount: i32, stride: int, tolerance: f32) {
	h.vertices = vertices
	h.vertexCount = vertexCount
	h.stride = stride
	h.tolerance = tolerance
	h.cellSize = 2.0 * tolerance
	Array_CreateN(&h.nodes, vertexCount)

	// unified: b3VertexMap_init and b3VertexMap_reserve
	h.vertexMap = make(VertexMap, int(vertexCount), runtime.heap_allocator())
	// unified end

	assert(h.cellSize > 0.0)
}

@(private = "file")
DestroySpatialHash :: proc(h: ^SpatialHash) {
	// unified: b3VertexMap_cleanup
	delete(h.vertexMap)
	// unified end
	Array_Destroy(&h.nodes)
}

// Welding works by bucketing nearby vertices into identical keys in a hash table.
// Bucketing is done manually with an array.
@(private = "file")
SpatialHash_FindDuplicate :: proc(h: ^SpatialHash, currentIndex: i32) -> i32 {
	assert(currentIndex < h.vertexCount)
	stride := h.stride
	vertex := GetStridedVertex(h.vertices, currentIndex, stride)
	cellSize := h.cellSize
	tolerance := h.tolerance

	// Get the grid coordinates for the current vertex
	baseX := i32(floorf(vertex.x / cellSize))
	baseY := i32(floorf(vertex.y / cellSize))
	baseZ := i32(floorf(vertex.z / cellSize))

	// Check the current cell and all 26 neighboring cells (3x3x3 - 1)
	for dx: i32 = -1; dx <= 1; dx += 1 {
		for dy: i32 = -1; dy <= 1; dy += 1 {
			for dz: i32 = -1; dz <= 1; dz += 1 {
				x := baseX + dx
				y := baseY + dy
				z := baseZ + dz

				// Compute hash for this neighboring cell (this is the key in the map)
				key: u64 = 0
				key ~= u64(i64(x)) + 0x9e3779b9 + (key << 6) + (key >> 2)
				key ~= u64(i64(y)) + 0x9e3779b9 + (key << 6) + (key >> 2)
				key ~= u64(i64(z)) + 0x9e3779b9 + (key << 6) + (key >> 2)

				// unified: b3VertexMap_get and b3VertexMap_is_end
				val, found := h.vertexMap[key]
				// unified end
				if found {
					// Check all vertices in this key
					nodeIndex := val

					for nodeIndex != NULL_INDEX {
						node := h.nodes.data[nodeIndex]

						existingIndex := node.vertexIndex
						assert(existingIndex < currentIndex)
						assert(existingIndex < h.vertexCount)

						other := GetStridedVertex(h.vertices, existingIndex, stride)

						// IsEqual inlined: check if vertices are within tolerance
						if abs(vertex.x - other.x) <= tolerance &&
						   abs(vertex.y - other.y) <= tolerance &&
						   abs(vertex.z - other.z) <= tolerance {
							// Found duplicate
							return existingIndex
						}

						nodeIndex = node.nextNodeIndex
					}
				}
			}
		}
	}

	// No duplicate found, add to hash table
	currentKey: u64 = 0
	currentKey ~= u64(i64(baseX)) + 0x9e3779b9 + (currentKey << 6) + (currentKey >> 2)
	currentKey ~= u64(i64(baseY)) + 0x9e3779b9 + (currentKey << 6) + (currentKey >> 2)
	currentKey ~= u64(i64(baseZ)) + 0x9e3779b9 + (currentKey << 6) + (currentKey >> 2)

	// unified: b3VertexMap_get and b3VertexMap_is_end
	val, found := h.vertexMap[currentKey]
	// unified end
	if found {
		nodeIndex := val

		node := VertexNode {
			vertexIndex   = currentIndex,
			nextNodeIndex = nodeIndex,
		}

		// unified: it.data->val = h->nodes.count
		h.vertexMap[currentKey] = h.nodes.count
		// unified end
		Array_Push(&h.nodes, node)
	} else {
		node := VertexNode {
			vertexIndex   = currentIndex,
			nextNodeIndex = NULL_INDEX,
		}

		// unified: b3VertexMap_insert
		h.vertexMap[currentKey] = h.nodes.count
		// unified end
		Array_Push(&h.nodes, node)
	}

	// Not welded
	return NULL_INDEX
}

WeldData :: struct {
	srcVertices: [^]Vec3,
	srcIndices:  [^]i32,
	dstVertices: [^]Vec3,
	dstIndices:  [^]i32,
	vertexCount: i32,
	indexCount:  i32,
	stride:      int,
}

@(private = "file")
WeldVertices :: proc(data: ^WeldData, tolerance: f32) -> i32 {
	vertexCount := data.vertexCount
	uniqueCount: i32 = 0
	stride := data.stride

	// Create spatial hash and find duplicates
	spatialHash: SpatialHash
	CreateSpatialHash(&spatialHash, data.srcVertices, vertexCount, stride, tolerance)
	vertexMapping: Array(i32)
	Array_Resize(&vertexMapping, vertexCount)

	for i: i32 = 0; i < vertexCount; i += 1 {
		duplicateIndex := SpatialHash_FindDuplicate(&spatialHash, i)

		if duplicateIndex == NULL_INDEX {
			// New unique vertex
			vertexMapping.data[i] = uniqueCount
			data.dstVertices[uniqueCount] = GetStridedVertex(data.srcVertices, i, stride)
			uniqueCount += 1
		} else {
			// Found duplicate, map to existing vertex
			vertexMapping.data[i] = vertexMapping.data[duplicateIndex]
		}
	}

	// Update indices to reference the new vertex array
	indexCount := data.indexCount
	for i: i32 = 0; i < indexCount; i += 1 {
		srcIndex := data.srcIndices[i]
		assert(0 <= srcIndex && srcIndex < vertexCount)
		data.dstIndices[i] = vertexMapping.data[srcIndex]
	}

	DestroySpatialHash(&spatialHash)
	Array_Destroy(&vertexMapping)

	return uniqueCount
}

@(private = "file")
StoreLeaf :: #force_inline proc(node: ^MeshNode, aabb: ^AABB, triangleCount: i32, triangleOffset: i32) {
	node.data.asLeaf.type = LEAF_NODE
	node.data.asLeaf.triangleCount = u32(triangleCount)
	node.triangleOffset = u32(triangleOffset)
	node.lowerBound = aabb.lowerBound
	node.upperBound = aabb.upperBound
}

Primitive :: struct {
	aabb:          AABB,
	center:        Vec3,
	triangleIndex: i32,
}

Bucket :: struct {
	count:  i32,
	bounds: AABB,
}

Split :: struct {
	leftBounds:  AABB,
	rightBounds: AABB,
	axis:        i32,
	index:       i32,
}

@(private = "file")
SplitBinnedSah :: proc(count: i32, primitives: [^]Primitive) -> Split {
	split: Split
	split.axis = -1
	split.index = -1

	// Compute bounds of primitive centroids and choose split axis
	bounds := AABB{primitives[0].center, primitives[0].center}
	for i: i32 = 1; i < count; i += 1 {
		bounds = AABB_AddPoint(bounds, primitives[i].center)
	}

	// Compute costs for splitting after each bucket and keep track of best split
	// This is a small O(n^2) loop. This can be further optimized, but it is already
	// very fast and is kept for simplicity right now.
	bestBucket: i32 = -1
	bestCost := FLT_MAX

	for axis: i32 = 0; axis < 3; axis += 1 {
		extent := AABB_Extents(bounds)
		if GetByIndex(extent, axis) < LINEAR_SLOP {
			continue
		}

		// Initialize buckets
		buckets: [BIN_COUNT]Bucket
		for i := 0; i < BIN_COUNT; i += 1 {
			buckets[i].count = 0
			buckets[i].bounds = BOUNDS3_EMPTY
		}

		// Fill buckets
		factor :=
			f32(BIN_COUNT) *
			(1.0 - FLT_EPSILON) /
			(GetByIndex(bounds.upperBound, axis) - GetByIndex(bounds.lowerBound, axis))
		for i: i32 = 0; i < count; i += 1 {
			center := primitives[i].center
			index := i32(factor * (GetByIndex(center, axis) - GetByIndex(bounds.lowerBound, axis)))
			assert(0 <= index && index < BIN_COUNT)

			buckets[index].count += 1
			buckets[index].bounds = AABB_Union(buckets[index].bounds, primitives[i].aabb)
		}

		// Evaluate splits
		for i: i32 = 0; i < BIN_COUNT - 1; i += 1 {
			leftCount: i32 = 0
			leftBounds := BOUNDS3_EMPTY
			for k: i32 = 0; k <= i; k += 1 {
				leftCount += buckets[k].count
				leftBounds = AABB_Union(leftBounds, buckets[k].bounds)
			}

			rightCount: i32 = 0
			rightBounds := BOUNDS3_EMPTY
			for k: i32 = i + 1; k < BIN_COUNT; k += 1 {
				rightCount += buckets[k].count
				rightBounds = AABB_Union(rightBounds, buckets[k].bounds)
			}

			assert(leftCount + rightCount == count)
			if leftCount > 0 && rightCount > 0 {
				cost := f32(leftCount) * AABB_Area(leftBounds) + f32(rightCount) * AABB_Area(rightBounds)

				if cost < bestCost {
					bestBucket = i
					bestCost = cost

					split.axis = axis
					split.index = leftCount
					split.leftBounds = leftBounds
					split.rightBounds = rightBounds
				}
			}
		}
	}

	// Partition
	if bestBucket >= 0 {
		axis := split.axis
		factor :=
			f32(BIN_COUNT) *
			(1.0 - FLT_EPSILON) /
			(GetByIndex(bounds.upperBound, axis) - GetByIndex(bounds.lowerBound, axis))

		splitIndex: i32 = 0
		for i: i32 = 0; i < count; i += 1 {
			center := primitives[i].center
			index := i32(factor * (GetByIndex(center, axis) - GetByIndex(bounds.lowerBound, axis)))

			if index <= bestBucket {
				temp := primitives[i]
				primitives[i] = primitives[splitIndex]
				primitives[splitIndex] = temp
				splitIndex += 1
			}
		}
		assert(splitIndex == split.index)
	}

	return split
}

@(private = "file")
SplitHalf :: proc(count: i32, primitives: [^]Primitive) -> Split {
	// Split in the middle
	splitIndex := count / 2

	leftBounds := BOUNDS3_EMPTY
	for i: i32 = 0; i < splitIndex; i += 1 {
		leftBounds = AABB_Union(leftBounds, primitives[i].aabb)
	}

	rightBounds := BOUNDS3_EMPTY
	for i: i32 = splitIndex; i < count; i += 1 {

		rightBounds = AABB_Union(rightBounds, primitives[i].aabb)
	}

	bounds := AABB_Union(leftBounds, rightBounds)
	axis := MajorAxis(AABB_Extents(bounds))

	split: Split
	split.axis = axis
	split.index = splitIndex
	split.leftBounds = leftBounds
	split.rightBounds = rightBounds

	return split
}

@(private = "file")
SplitMedian :: proc(count: i32, primitives: [^]Primitive) -> Split {
	assert(count > 2)

	lowerBound := primitives[0].center
	upperBound := primitives[0].center

	for i: i32 = 1; i < count; i += 1 {
		lowerBound = Min(lowerBound, primitives[i].center)
		upperBound = Max(upperBound, primitives[i].center)
	}

	d := Sub(upperBound, lowerBound)
	c := MulSV(0.5, Add(lowerBound, upperBound))

	split := Split{}
	split.index = -1

	// Partition longest axis using the Hoare partition scheme
	// https://en.wikipedia.org/wiki/Quicksort
	// https://nicholasvadivelu.com/2021/01/11/array-partition/
	i1, i2: i32 = 0, count
	if d.x >= d.y && d.x >= d.z {
		split.axis = 0

		pivot := c.x

		for i1 < i2 {
			for i1 < i2 && primitives[i1].center.x < pivot {
				i1 += 1
			}

			for i1 < i2 && primitives[i2 - 1].center.x >= pivot {
				i2 -= 1
			}

			if i1 < i2 {
				// Swap primitives
				temp := primitives[i1]
				primitives[i1] = primitives[i2 - 1]
				primitives[i2 - 1] = temp

				i1 += 1
				i2 -= 1
			}
		}
	} else if d.y >= d.z {
		split.axis = 1

		pivot := c.y

		for i1 < i2 {
			for i1 < i2 && primitives[i1].center.y < pivot {
				i1 += 1
			}

			for i1 < i2 && primitives[i2 - 1].center.y >= pivot {
				i2 -= 1
			}

			if i1 < i2 {
				// Swap primitives
				temp := primitives[i1]
				primitives[i1] = primitives[i2 - 1]
				primitives[i2 - 1] = temp

				i1 += 1
				i2 -= 1
			}
		}
	} else {
		split.axis = 2

		pivot := c.z

		for i1 < i2 {
			for i1 < i2 && primitives[i1].center.z < pivot {
				i1 += 1
			}

			for i1 < i2 && primitives[i2 - 1].center.z >= pivot {
				i2 -= 1
			}

			if i1 < i2 {
				// Swap primitives
				temp := primitives[i1]
				primitives[i1] = primitives[i2 - 1]
				primitives[i2 - 1] = temp

				i1 += 1
				i2 -= 1
			}
		}
	}
	assert(i1 == i2)
	assert(0 <= i1 && i1 < count)

	if i1 == 0 || i1 == count - 1 {
		// failed to split
		i1 = count / 2
	}

	leftBounds := BOUNDS3_EMPTY
	for i: i32 = 0; i < i1; i += 1 {
		leftBounds = AABB_Union(leftBounds, primitives[i].aabb)
	}

	rightBounds := BOUNDS3_EMPTY
	for i: i32 = i1; i < count; i += 1 {
		rightBounds = AABB_Union(rightBounds, primitives[i].aabb)
	}

	split.index = i1
	split.leftBounds = leftBounds
	split.rightBounds = rightBounds
	return split
}

when ENABLE_VALIDATION {

	@(private = "file")
	ValidateSplit :: proc(count: i32, primitives: [^]Primitive, split: ^Split) -> bool {
		if split.axis < 0 {
			return false
		}

		for i: i32 = 0; i < split.index; i += 1 {
			if !AABB_Contains(split.leftBounds, primitives[i].aabb) {
				return false
			}
		}

		for i: i32 = split.index; i < count; i += 1 {
			if !AABB_Contains(split.rightBounds, primitives[i].aabb) {
				return false
			}
		}

		return true
	}

}

@(private = "file")
BuildRecursive :: proc(
	nodes: ^Array(MeshNode),
	count: i32,
	primitives: [^]Primitive,
	base: [^]Primitive,
	useMedianSplit: bool,
	height: ^i32,
) -> i32 {
	if count > DESIRED_TRIANGLES_PER_LEAF {
		// Try to split the input set using the SAH
		split: Split
		if useMedianSplit {
			split = SplitMedian(count, primitives)
		} else {
			split = SplitBinnedSah(count, primitives)
		}

		if split.axis < 0 {
			if count > MAXIMUM_TRIANGLES_PER_LEAF {
				// Re-split. This is a less optimal split and can create more false positives!
				split = SplitHalf(count, primitives)
			} else {
				bounds := BOUNDS3_EMPTY
				for i: i32 = 0; i < count; i += 1 {
					bounds = AABB_Union(bounds, primitives[i].aabb)
				}

				// We have only a few triangles left. Create a leaf.
				index := Array_AddIndex(nodes)
				StoreLeaf(
					&nodes.data[index],
					&bounds,
					count,
					i32((uintptr(primitives) - uintptr(base)) / size_of(Primitive)),
				)

				return index
			}
		}
		when ENABLE_VALIDATION {
			VALIDATE(ValidateSplit(count, primitives, &split))
		}

		// Allocate node and recurse
		index := Array_AddIndex(nodes)
		heightLeft, heightRight: i32 = 0, 0
		leftIndex := BuildRecursive(nodes, split.index, primitives, base, useMedianSplit, &heightLeft)
		rightIndex := BuildRecursive(
			nodes,
			count - split.index,
			primitives[split.index:],
			base,
			useMedianSplit,
			&heightRight,
		)

		height^ = MaxInt(heightLeft, heightRight) + 1

		_ = leftIndex
		assert(leftIndex - index == 1 && rightIndex - index > 1)

		aabb := AABB_Union(split.leftBounds, split.rightBounds)
		node := Array_Get(nodes, index)
		node.data.asNode.axis = u32(split.axis)
		node.data.asNode.childOffset = u32(rightIndex - index)
		node.lowerBound = aabb.lowerBound
		node.upperBound = aabb.upperBound

		// Zero so mesh->hash is deterministic
		node.triangleOffset = 0

		return index
	}

	aabb := BOUNDS3_EMPTY
	for i: i32 = 0; i < count; i += 1 {
		aabb = AABB_Union(aabb, primitives[i].aabb)
	}

	index := Array_AddIndex(nodes)
	StoreLeaf(&nodes.data[index], &aabb, count, i32((uintptr(primitives) - uintptr(base)) / size_of(Primitive)))

	height^ = 1

	return index
}

@(private = "file")
SortMeshTriangles :: proc(mesh: ^MeshData) -> bool {
	triangles := GetMeshTrianglesWrite(mesh)
	materialIndices := GetMeshMaterialIndicesWrite(mesh)

	// Sort triangles in depth-first-order
	offset: i32 = 0
	tempTriangles: Array(MeshTriangle)
	Array_CreateN(&tempTriangles, mesh.triangleCount)

	tempMaterialIndices: Array(u8)
	Array_CreateN(&tempMaterialIndices, mesh.triangleCount)

	count: i32 = 0
	stack: [MESH_STACK_SIZE]^MeshNode = ---
	stack[count] = GetRootWrite(mesh)
	count += 1

	for count > 0 {
		count -= 1
		node := stack[count]

		if IsLeaf(node) == false {
			if count >= MESH_STACK_SIZE - 2 {
				return false
			}

			stack[count] = GetRightChildWrite(node)
			count += 1
			stack[count] = GetLeftChildWrite(node)
			count += 1
		} else {
			triangleCount := i32(node.data.asLeaf.triangleCount)
			triangleOffset := i32(node.triangleOffset)

			for triangle: i32 = 0; triangle < triangleCount; triangle += 1 {
				index := triangleOffset + triangle
				Array_Push(&tempTriangles, triangles[index])
				Array_Push(&tempMaterialIndices, materialIndices[index])
			}

			node.triangleOffset = u32(offset)
			offset += triangleCount
		}
	}

	assert(offset == tempTriangles.count)
	assert(tempTriangles.count == mesh.triangleCount)
	assert(tempMaterialIndices.count == mesh.triangleCount)

	// Copy sorted triangle array back to tree
	mem.copy_non_overlapping(triangles, tempTriangles.data, int(mesh.triangleCount) * size_of(MeshTriangle))
	mem.copy_non_overlapping(materialIndices, tempMaterialIndices.data, int(mesh.triangleCount) * size_of(u8))

	Array_Destroy(&tempTriangles)
	Array_Destroy(&tempMaterialIndices)

	return true
}

MeshEdge :: struct {
	vertex1:            i32,
	vertex2:            i32,
	triangle1:          i32,
	triangle2:          i32,
	triangleCount:      u16,

	// The index of an edge within the parent triangle: 0, 1, or 2. 0xFF is unset
	triangleEdgeIndex1: u8,
	triangleEdgeIndex2: u8,
}

// unified: verstable's b3EdgeMap (u64 -> int) is an Odin map on the heap allocator; it is only used for lookups and
// inserts, never iterated. Its storage is not counted by GetByteCount.
EdgeMap :: map[u64]i32
// unified end

// unified: not ported: the disabled (#if 0) b3TestHash used for testing other hash tables.

// Results for MeshCreationBenchmark
// eastl::hash_map : 4.9703 ms and 5.1445ms with FastHash function
// std::unordered_map : 4.8755 ms and 4.4780ms with FastHash function
// verstable : 3.3968 ms with default FastHash
// no edge identification : 1.7396 ms
@(private = "file")
IdentifyEdges :: proc(mesh: ^MeshData) {
	triangles := GetMeshTrianglesWrite(mesh)
	vertices := GetMeshVertices(mesh)
	flags := GetMeshFlagsWrite(mesh)

	triangleCount := mesh.triangleCount
	edgeCount := 3 * triangleCount
	edges := ALLOC(MeshEdge, edgeCount)
	normals := ALLOC(Vec3, triangleCount)

	for i: i32 = 0; i < triangleCount; i += 1 {
		triangle := &triangles[i]
		i1 := triangle.index1
		i2 := triangle.index2
		i3 := triangle.index3

		edges[3 * i + 0].vertex1 = MinInt(i1, i2)
		edges[3 * i + 0].vertex2 = MaxInt(i1, i2)
		edges[3 * i + 0].triangle1 = i
		edges[3 * i + 0].triangle2 = NULL_INDEX
		edges[3 * i + 0].triangleEdgeIndex1 = 0
		edges[3 * i + 0].triangleEdgeIndex2 = 0xFF
		edges[3 * i + 0].triangleCount = 1

		edges[3 * i + 1].vertex1 = MinInt(i2, i3)
		edges[3 * i + 1].vertex2 = MaxInt(i2, i3)
		edges[3 * i + 1].triangle1 = i
		edges[3 * i + 1].triangle2 = NULL_INDEX
		edges[3 * i + 1].triangleEdgeIndex1 = 1
		edges[3 * i + 1].triangleEdgeIndex2 = 0xFF
		edges[3 * i + 1].triangleCount = 1

		edges[3 * i + 2].vertex1 = MinInt(i3, i1)
		edges[3 * i + 2].vertex2 = MaxInt(i3, i1)
		edges[3 * i + 2].triangle1 = i
		edges[3 * i + 2].triangle2 = NULL_INDEX
		edges[3 * i + 2].triangleEdgeIndex1 = 2
		edges[3 * i + 2].triangleEdgeIndex2 = 0xFF
		edges[3 * i + 2].triangleCount = 1

		v1 := vertices[i1]
		v2 := vertices[i2]
		v3 := vertices[i3]

		e1 := Sub(v2, v1)
		e2 := Sub(v3, v1)
		n := Cross(e1, e2)

		normals[i] = Normalize(n)
	}

	// unified: b3EdgeMap_init and b3EdgeMap_reserve
	map_ := make(EdgeMap, int(edgeCount), runtime.heap_allocator())
	// unified end

	key := u64(edges[0].vertex1) << 32 | u64(edges[0].vertex2)
	// unified: b3EdgeMap_insert
	map_[key] = 0
	// unified end

	// Find unique edges and assign adjacency
	for i: i32 = 1; i < edgeCount; i += 1 {
		edge := &edges[i]
		key = u64(edge.vertex1) << 32 | u64(edge.vertex2)
		// unified: b3EdgeMap_get and b3EdgeMap_is_end
		val, found := map_[key]
		// unified end

		if found == false {
			// unified: b3EdgeMap_insert
			map_[key] = i
			// unified end
		} else {
			otherIndex := val
			assert(otherIndex < i)

			base := &edges[otherIndex]
			if base.triangleCount == 1 {
				base.triangle2 = edge.triangle1
				base.triangleEdgeIndex2 = edge.triangleEdgeIndex1
			}

			base.triangleCount += 1
		}
	}

	// unified: b3EdgeMap_cleanup
	delete(map_)
	// unified end

	for i: i32 = 0; i < edgeCount; i += 1 {
		edge := &edges[i]
		if edge.triangleCount != 2 {
			continue
		}

		assert(edge.triangleEdgeIndex1 < 3)
		assert(edge.triangleEdgeIndex2 < 3)

		triangle1 := &triangles[edge.triangle1]
		triangle2 := &triangles[edge.triangle2]
		flag1 := &flags[edge.triangle1]
		flag2 := &flags[edge.triangle2]

		j1 := triangle2.index1
		j2 := triangle2.index2
		j3 := triangle2.index3

		opposite: i32 = NULL_INDEX

		switch edge.triangleEdgeIndex2 {
		case 0:
			opposite = j3

		case 1:
			opposite = j1

		case 2:
			opposite = j2

		case:
			assert(false)
		}

		i1 := triangle1.index1
		i2 := triangle1.index2
		i3 := triangle1.index3

		v1 := vertices[i1]
		v2 := vertices[i2]
		v3 := vertices[i3]
		p := vertices[opposite]

		cos5Deg: f32 = 0.9962
		signedVolume := SignedVolume(v1, v2, v3, p)
		n1 := normals[edge.triangle1]
		n2 := normals[edge.triangle2]
		cosAngle := Dot(n1, n2)
		if signedVolume > 0.0 || cosAngle > cos5Deg {
			edgeFlags := [3]i32{concaveEdge1, concaveEdge2, concaveEdge3}
			flag1^ |= u8(edgeFlags[edge.triangleEdgeIndex1])
			flag2^ |= u8(edgeFlags[edge.triangleEdgeIndex2])
		}

		if signedVolume < 0.0 || cosAngle > cos5Deg {
			edgeFlags := [3]i32{inverseConcaveEdge1, inverseConcaveEdge2, inverseConcaveEdge3}
			flag1^ |= u8(edgeFlags[edge.triangleEdgeIndex1])
			flag2^ |= u8(edgeFlags[edge.triangleEdgeIndex2])
		}
	}

	FREE(normals, Vec3, triangleCount)
	FREE(edges, MeshEdge, edgeCount)
}

CreateGridMesh :: proc(
	xCount: i32,
	zCount: i32,
	cellWidth: f32,
	materialCount: i32,
	identifyEdges: bool,
) -> ^MeshData {
	assert(0 <= materialCount && materialCount <= i32(max(u8)))

	// Create vertices
	vertexCount := (xCount + 1) * (zCount + 1)

	vertices: Array(Vec3)
	Array_Resize(&vertices, vertexCount)
	index: i32 = 0

	xWidth := cellWidth * f32(xCount)
	zWidth := cellWidth * f32(zCount)

	x := -0.5 * xWidth
	for ix: i32 = 0; ix <= xCount; ix += 1 {
		z := -0.5 * zWidth
		for iz: i32 = 0; iz <= zCount; iz += 1 {
			vertices.data[index] = Vec3{x, 0.0, z}
			z += cellWidth
			index += 1
		}
		x += cellWidth
	}
	assert(index == vertexCount)

	// Triangles
	triangleCount := 2 * xCount * zCount

	indices: Array(i32)
	Array_Resize(&indices, 3 * triangleCount)

	materialIndices: Array(u8)
	Array_Resize(&materialIndices, triangleCount)

	materialIndex: i32 = 0
	index = 0
	for ix: i32 = 0; ix < xCount; ix += 1 {
		for iz: i32 = 0; iz < zCount; iz += 1 {
			index1 := iz + (zCount + 1) * ix
			index2 := index1 + 1
			index3 := index2 + (zCount + 1)
			index4 := index3 - 1

			assert(index1 < vertexCount)
			assert(index2 < vertexCount)
			assert(index3 < vertexCount)
			assert(index4 < vertexCount)

			indices.data[index + 0] = index1
			indices.data[index + 1] = index2
			indices.data[index + 2] = index3

			indices.data[index + 3] = index3
			indices.data[index + 4] = index4
			indices.data[index + 5] = index1

			if materialCount > 0 {
				materialIndices.data[2 * materialIndex + 0] = u8(materialIndex % materialCount)
				materialIndices.data[2 * materialIndex + 1] = u8(materialIndex % materialCount)
			}

			materialIndex += 1
			index += 6
		}
	}
	assert(index == 3 * triangleCount)

	def: MeshDef
	def.vertexCount = vertices.count
	def.vertices = vertices.data
	def.triangleCount = indices.count / 3
	def.indices = indices.data
	def.materialIndices = materialCount > 0 ? materialIndices.data : nil
	def.useMedianSplit = true
	def.identifyEdges = identifyEdges

	meshData := CreateMesh(&def, nil, 0)

	Array_Destroy(&indices)
	Array_Destroy(&vertices)
	Array_Destroy(&materialIndices)

	return meshData
}

CreateWaveMesh :: proc(
	xCount: i32,
	zCount: i32,
	cellWidth: f32,
	amplitude: f32,
	rowFrequency: f32,
	columnFrequency: f32,
) -> ^MeshData {
	// Create vertices
	vertexCount := (xCount + 1) * (zCount + 1)

	vertices: Array(Vec3)
	Array_Resize(&vertices, vertexCount)
	index: i32 = 0

	xWidth := cellWidth * f32(xCount)
	zWidth := cellWidth * f32(zCount)

	omegaZ := 2.0 * PI * rowFrequency * cellWidth
	omegaX := 2.0 * PI * columnFrequency * cellWidth

	x := -0.5 * xWidth
	cs := CosSin{}
	for ix: i32 = 0; ix <= xCount; ix += 1 {
		cs = ComputeCosSin(omegaX * f32(ix))
		rowHeight := cs.sine

		z := -0.5 * zWidth
		for iz: i32 = 0; iz <= zCount; iz += 1 {
			cs = ComputeCosSin(omegaZ * f32(iz))
			columnHeight := cs.sine

			y := amplitude * rowHeight * columnHeight
			vertices.data[index] = Vec3{x, y, z}
			z += cellWidth
			index += 1
		}
		x += cellWidth
	}
	assert(index == vertexCount)

	// Triangles
	triangleCount := 2 * xCount * zCount

	indices: Array(i32)
	Array_Resize(&indices, 3 * triangleCount)

	index = 0
	for ix: i32 = 0; ix < xCount; ix += 1 {
		for iz: i32 = 0; iz < zCount; iz += 1 {
			index1 := iz + (zCount + 1) * ix
			index2 := index1 + 1
			index3 := index2 + (zCount + 1)
			index4 := index3 - 1

			assert(index1 < vertexCount)
			assert(index2 < vertexCount)
			assert(index3 < vertexCount)
			assert(index4 < vertexCount)

			indices.data[index + 0] = index1
			indices.data[index + 1] = index2
			indices.data[index + 2] = index3

			indices.data[index + 3] = index3
			indices.data[index + 4] = index4
			indices.data[index + 5] = index1

			index += 6
		}
	}
	assert(index == 3 * triangleCount)

	def: MeshDef
	def.vertexCount = vertices.count
	def.vertices = vertices.data
	def.triangleCount = indices.count / 3
	def.indices = indices.data
	def.useMedianSplit = true
	def.identifyEdges = true

	meshData := CreateMesh(&def, nil, 0)

	Array_Destroy(&indices)
	Array_Destroy(&vertices)

	return meshData
}

CreateTorusMesh :: proc(radialResolution: i32, tubularResolution: i32, radius: f32, thickness: f32) -> ^MeshData {
	// Create vertices
	vertices: Array(Vec3)

	for radialIndex: i32 = 0; radialIndex < radialResolution; radialIndex += 1 {
		for tubularIndex: i32 = 0; tubularIndex < tubularResolution; tubularIndex += 1 {
			u := f32(tubularIndex) / f32(tubularResolution) * TWO_PI
			v := f32(radialIndex) / f32(radialResolution) * TWO_PI

			x := (radius + thickness * Cos(v)) * Cos(u)
			y := (radius + thickness * Cos(v)) * Sin(u)
			z := thickness * Sin(v)

			vertex := Vec3{x, y, z}
			Array_Push(&vertices, vertex)
		}
	}

	// Triangles
	indices: Array(i32)
	for radialIndex1: i32 = 0; radialIndex1 < radialResolution; radialIndex1 += 1 {
		radialIndex2 := (radialIndex1 + 1) % radialResolution
		for tubularIndex1: i32 = 0; tubularIndex1 < tubularResolution; tubularIndex1 += 1 {
			tubularIndex2 := (tubularIndex1 + 1) % tubularResolution
			index1 := radialIndex1 * tubularResolution + tubularIndex1
			index2 := radialIndex1 * tubularResolution + tubularIndex2
			index3 := radialIndex2 * tubularResolution + tubularIndex2
			index4 := radialIndex2 * tubularResolution + tubularIndex1

			Array_Push(&indices, index1)
			Array_Push(&indices, index2)
			Array_Push(&indices, index3)

			Array_Push(&indices, index3)
			Array_Push(&indices, index4)
			Array_Push(&indices, index1)
		}
	}

	def: MeshDef
	def.vertexCount = vertices.count
	def.vertices = vertices.data
	def.triangleCount = indices.count / 3
	def.indices = indices.data
	def.useMedianSplit = false
	def.identifyEdges = true

	meshData := CreateMesh(&def, nil, 0)

	Array_Destroy(&vertices)
	Array_Destroy(&indices)
	return meshData
}

CreateBoxMesh :: proc(center: Vec3, extent: Vec3, identifyEdges: bool) -> ^MeshData {
	x := extent.x
	y := extent.y
	z := extent.z
	vertices := [8]Vec3 {
		{x, y, z},
		{-x, y, z},
		{-x, -y, z},
		{x, -y, z},
		{x, y, -z},
		{-x, y, -z},
		{-x, -y, -z},
		{x, -y, -z},
	}

	for i := 0; i < 8; i += 1 {
		vertices[i] = Add(vertices[i], center)
	}
	
	// odinfmt: disable
	indices := [36]i32 {
		0, 1, 3, 1, 2, 3, // front
		0, 4, 1, 1, 4, 5, // top
		0, 3, 7, 4, 0, 7, // right
		4, 7, 5, 6, 5, 7, // back
		1, 5, 2, 6, 2, 5, // left
		3, 2, 7, 6, 7, 2, // bottom
	}
	// odinfmt: enable

	def: MeshDef
	def.vertexCount = 8
	def.vertices = &vertices[0]
	def.triangleCount = 12
	def.indices = &indices[0]
	def.useMedianSplit = false
	def.identifyEdges = identifyEdges

	return CreateMesh(&def, nil, 0)
}

CreateHollowBoxMesh :: proc(center: Vec3, extent: Vec3) -> ^MeshData {
	x := extent.x
	y := extent.y
	z := extent.z
	vertices := [8]Vec3 {
		{x, y, z},
		{-x, y, z},
		{-x, -y, z},
		{x, -y, z},
		{x, y, -z},
		{-x, y, -z},
		{-x, -y, -z},
		{x, -y, -z},
	}

	for i := 0; i < 8; i += 1 {
		vertices[i] = Add(vertices[i], center)
	}
	
	// odinfmt: disable
	indices := [36]i32 {
		3, 1, 0, 3, 2, 1, // front
		1, 4, 0, 5, 4, 1, // top
		7, 3, 0, 7, 0, 4, // right
		5, 7, 4, 7, 5, 6, // back
		2, 5, 1, 5, 2, 6, // left
		7, 2, 3, 2, 7, 6, // bottom
	}
	// odinfmt: enable

	def: MeshDef
	def.vertexCount = 8
	def.vertices = &vertices[0]
	def.triangleCount = 12
	def.indices = &indices[0]
	def.useMedianSplit = false
	def.identifyEdges = true

	return CreateMesh(&def, nil, 0)
}

CreatePlatformMesh :: proc(center: Vec3, height: f32, topWidth: f32, bottomWidth: f32) -> ^MeshData {
	hb := 0.5 * bottomWidth
	ht := 0.5 * topWidth
	hy := 0.5 * height
	vertices := [8]Vec3 {
		{ht, hy, ht},
		{-ht, hy, ht},
		{-hb, -hy, hb},
		{hb, -hy, hb},
		{ht, hy, -ht},
		{-ht, hy, -ht},
		{-hb, -hy, -hb},
		{hb, -hy, -hb},
	}

	for i := 0; i < 8; i += 1 {
		vertices[i] = Add(vertices[i], center)
	}
	
	// odinfmt: disable
	indices := [36]i32 {
		0, 1, 3, 1, 2, 3, // front
		0, 4, 1, 1, 4, 5, // top
		0, 3, 7, 4, 0, 7, // right
		4, 7, 5, 6, 5, 7, // back
		1, 5, 2, 6, 2, 5, // left
		3, 2, 7, 6, 7, 2, // bottom
	}
	// odinfmt: enable

	def: MeshDef
	def.vertexCount = 8
	def.vertices = &vertices[0]
	def.triangleCount = 12
	def.indices = &indices[0]
	def.useMedianSplit = true
	def.identifyEdges = true

	return CreateMesh(&def, nil, 0)
}

@(private = "file")
CopyVerticesWithStride :: proc(dst: [^]Vec3, src: [^]Vec3, count: i32, stride: int) {
	if stride == size_of(Vec3) {
		mem.copy_non_overlapping(dst, src, int(count) * size_of(Vec3))
		return
	}

	for i: i32 = 0; i < count; i += 1 {
		dst[i] = (cast(^Vec3)(uintptr(src) + uintptr(int(i) * stride)))^
	}
}

// Stride larger than this likely indicates stride is uninitialized memory.
@(private = "file")
MAX_STRIDE :: 4096

// todo this should fail if the mesh has a height greater than B3_MESH_STACK_SIZE
CreateMesh :: proc(def: ^MeshDef, degenerateTriangleIndices: [^]i32, degenerateCapacity: i32) -> ^MeshData {
	if def.stride != 0 && (def.stride < size_of(Vec3) || MAX_STRIDE < def.stride) {
		return nil
	}

	if (def.stride & 3) != 0 {
		return nil
	}

	if def.vertexCount < 3 || def.vertices == nil || def.triangleCount <= 0 || def.indices == nil {
		return nil
	}

	triangleCount := def.triangleCount
	if triangleCount == 0 {
		return nil
	}

	vertexCount := def.vertexCount
	stride := def.stride == 0 ? size_of(Vec3) : def.stride

	meshBounds := BOUNDS3_EMPTY

	// Clone indices and vertices to support welding
	indices: Array(i32)
	Array_CreateN(&indices, 3 * triangleCount)

	vertices: Array(Vec3)
	Array_CreateN(&vertices, vertexCount)

	if def.weldVertices && def.weldTolerance > 0.0 {
		Array_Resize(&vertices, vertexCount)
		Array_Resize(&indices, 3 * triangleCount)

		data := WeldData {
			srcVertices = def.vertices,
			srcIndices  = def.indices,
			dstVertices = vertices.data,
			dstIndices  = indices.data,
			vertexCount = vertexCount,
			indexCount  = 3 * triangleCount,
			stride      = stride,
		}

		vertices.count = WeldVertices(&data, def.weldTolerance)
		vertexCount = vertices.count
		assert(vertexCount <= def.vertexCount)
	} else {
		vertices.count = vertexCount
		CopyVerticesWithStride(vertices.data, def.vertices, vertexCount, stride)
		Array_Append(&indices, def.indices, 3 * triangleCount)
	}

	primitives: Array(Primitive)
	Array_CreateN(&primitives, triangleCount)
	degenerateCount: i32 = 0
	minArea := 0.01 * LINEAR_SLOP * LINEAR_SLOP
	surfaceArea: f32 = 0.0
	materialCount: i32 = 1
	clockWise := def.clockWiseWinding

	for index: i32 = 0; index < triangleCount; index += 1 {
		index1 := indices.data[3 * index + 0]
		index2 := indices.data[3 * index + 1]
		index3 := indices.data[3 * index + 2]

		assert(0 <= index1 && index1 < vertexCount)
		assert(0 <= index2 && index2 < vertexCount)
		assert(0 <= index3 && index3 < vertexCount)

		vertex1 := vertices.data[index1]
		vertex2 := vertices.data[index2]
		vertex3 := vertices.data[index3]

		normal := Cross(Sub(vertex2, vertex1), Sub(vertex3, vertex1))
		area := 0.5 * Length(normal)

		if area < minArea {
			// b3Log( "degenerate: %d %d %d\n", index1, index2, index3 );

			if index1 != index2 && index1 != index3 && index2 != index3 {
				if degenerateTriangleIndices != nil && degenerateCount < degenerateCapacity {
					degenerateTriangleIndices[degenerateCount] = index
				}

				degenerateCount += 1
			}

			continue
		}

		surfaceArea += area

		box := AABB {
			lowerBound = Min(vertex1, Min(vertex2, vertex3)),
			upperBound = Max(vertex1, Max(vertex2, vertex3)),
		}

		center := AABB_Center(box)

		primitive := Primitive {
			aabb          = box,
			center        = center,
			triangleIndex = index,
		}
		Array_Push(&primitives, primitive)

		if def.materialIndices != nil {
			materialCount = MaxInt(materialCount, i32(def.materialIndices[index]) + 1)
		}

		meshBounds = AABB_Union(meshBounds, box)
	}

	// Update triangle count due to degenerates being skipped
	triangleCount = primitives.count

	if IsSaneAABB(meshBounds) == false {
		Array_Destroy(&primitives)
		Array_Destroy(&indices)
		Array_Destroy(&vertices)
		return nil
	}

	// Build the tree (this reorders the builder triangles)
	tempNodes: Array(MeshNode)
	Array_CreateN(&tempNodes, 2 * triangleCount - 1)

	treeHeight: i32 = 0
	BuildRecursive(&tempNodes, triangleCount, primitives.data, primitives.data, def.useMedianSplit, &treeHeight)

	// Allocate the mesh
	byteCount := AlignUp8(size_of(MeshData))
	nodeOffset := i32(byteCount)
	byteCount += AlignUp8(int(tempNodes.count) * size_of(MeshNode))
	vertexOffset := i32(byteCount)
	byteCount += AlignUp8(int(vertexCount) * size_of(Vec3))
	triangleOffset := i32(byteCount)
	byteCount += AlignUp8(int(triangleCount) * size_of(MeshTriangle))
	materialIndicesOffset := i32(byteCount)
	byteCount += AlignUp8(int(triangleCount) * size_of(u8))
	flagsOffset := i32(byteCount)
	byteCount += AlignUp8(int(triangleCount) * size_of(u8))

	mesh := cast(^MeshData)AllocHost(byteCount) // unified: geometry is the user's, never in a world arena (world_arena.odin)

	// zero initialize for determinism
	mem.zero(mesh, byteCount)

	mesh.version = MESH_VERSION
	mesh.byteCount = i32(byteCount)
	mesh.bounds = meshBounds
	mesh.surfaceArea = surfaceArea
	mesh.nodeCount = tempNodes.count
	mesh.treeHeight = treeHeight
	mesh.vertexCount = vertexCount
	mesh.triangleCount = triangleCount
	mesh.degenerateCount = degenerateCount
	mesh.nodeOffset = nodeOffset
	mesh.vertexOffset = vertexOffset
	mesh.triangleOffset = triangleOffset
	mesh.materialOffset = materialIndicesOffset
	mesh.materialCount = materialCount
	mesh.flagsOffset = flagsOffset

	nodes := GetMeshNodesWrite(mesh)
	triangles := GetMeshTrianglesWrite(mesh)
	meshVertices := GetMeshVerticesWrite(mesh)
	materialIndices := GetMeshMaterialIndicesWrite(mesh)
	flags := GetMeshFlagsWrite(mesh)

	mem.copy_non_overlapping(nodes, tempNodes.data, int(tempNodes.count) * size_of(MeshNode))
	mem.copy_non_overlapping(meshVertices, vertices.data, int(vertexCount) * size_of(Vec3))

	for index: i32 = 0; index < triangleCount; index += 1 {
		primitive := primitives.data[index]

		i1 := 3 * primitive.triangleIndex + 0
		i2 := 3 * primitive.triangleIndex + 1
		i3 := 3 * primitive.triangleIndex + 2
		if clockWise {
			i2, i3 = i3, i2
		}

		triangles[index].index1 = indices.data[i1]
		triangles[index].index2 = indices.data[i2]
		triangles[index].index3 = indices.data[i3]
		flags[index] = 0

		// Copy material indices if they exist. Otherwise the material indices are all zeroes.
		if def.materialIndices != nil {
			materialIndex := def.materialIndices[primitive.triangleIndex]
			materialIndices[index] = materialIndex
		}
	}

	// Sort triangle in DFS order. Casts and volume queries will return sorted arrays.
	// This also sorts material indices, but not the materials.
	// This can fail if the BVH height is too large.
	success := SortMeshTriangles(mesh)
	if success == false {
		Array_Destroy(&tempNodes)
		Array_Destroy(&primitives)
		Array_Destroy(&indices)
		Array_Destroy(&vertices)
		Free(mesh, byteCount)
		return nil
	}

	if def.identifyEdges {
		IdentifyEdges(mesh)
	}

	when ENABLE_VALIDATION {
		VALIDATE(IsNonDegenerate(mesh, minArea))
		VALIDATE(IsConsistent(mesh))
	}

	Array_Destroy(&tempNodes)
	Array_Destroy(&primitives)
	Array_Destroy(&indices)
	Array_Destroy(&vertices)

	mesh.hash = 0
	mesh.hash = Hash64NonZero(cast([^]u8)mesh, mesh.byteCount)

	return mesh
}

DestroyMesh :: proc(mesh: ^MeshData) {
	Free(mesh, int(mesh.byteCount))
}

OverlapMesh :: proc(shape: ^Mesh, shapeTransform: Transform, proxy: ^ShapeProxy) -> bool {
	assert(proxy.count > 0)
	cache := SimplexCache{}

	buffer: [MAX_SHAPE_CAST_POINTS]Vec3 = ---
	localProxy := MakeLocalProxy(proxy, shapeTransform, &buffer[0])
	aabb := ComputeProxyAABB(&localProxy)

	meshScale := shape.scale

	// Scale may have reflection so min/max may become invalid when unscaled
	scale := LoadV(&meshScale.x)
	invScale := DivV(oneV, scale)
	temp1 := MulV(invScale, LoadV(&aabb.lowerBound.x))
	temp2 := MulV(invScale, LoadV(&aabb.upperBound.x))
	invScaledBoundsMin := MinV(temp1, temp2)
	invScaledBoundsMax := MaxV(temp1, temp2)
	invScaledBoundsCenter := MulV(halfV, AddV(invScaledBoundsMin, invScaledBoundsMax))
	invScaledBoundsExtent := SubV(invScaledBoundsMax, invScaledBoundsCenter)

	input: DistanceInput
	input.proxyB = localProxy
	input.transform = Transform_identity
	input.useRadii = true

	count: i32 = 0
	stack: [MESH_STACK_SIZE]^MeshNode = ---
	node := GetRoot(shape.data)
	triangles := GetMeshTriangles(shape.data)
	vertices := GetMeshVertices(shape.data)

	for true {
		// Test node overlap in unscaled space
		nodeMin := LoadV(&node.lowerBound.x)
		nodeMax := LoadV(&node.upperBound.x)
		if TestBoundsOverlap(nodeMin, nodeMax, invScaledBoundsMin, invScaledBoundsMax) {
			if IsLeaf(node) {
				triangleCount := i32(node.data.asLeaf.triangleCount)
				triangleOffset := i32(node.triangleOffset)

				for index: i32 = 0; index < triangleCount; index += 1 {
					triangleIndex := triangleOffset + index
					triangle := triangles[triangleIndex]

					vertex1 := vertices[triangle.index1]
					vertex2 := vertices[triangle.index2]
					vertex3 := vertices[triangle.index3]
					v1 := LoadV(&vertex1.x)
					v2 := LoadV(&vertex2.x)
					v3 := LoadV(&vertex3.x)

					// Bounding box overlap test in unscaled space
					if TestBoundsTriangleOverlap(invScaledBoundsCenter, invScaledBoundsExtent, v1, v2, v3) {
						// Shape-triangle overlap test in scaled space. Winding order doesn't matter.
						triangleVertices := [3]Vec3 {
							Mul(meshScale, vertex1),
							Mul(meshScale, vertex2),
							Mul(meshScale, vertex3),
						}
						input.proxyA = ShapeProxy{&triangleVertices[0], 3, 0.0}

						// reset the cache
						cache.count = 0

						// get distance between triangle and query shape
						output := ShapeDistance(&input, &cache, nil, 0)

						tolerance := 0.1 * LINEAR_SLOP
						if output.distance < tolerance {
							// overlap detected
							return true
						}
					}
				}
			} else {
				// Recurse
				assert(count <= MESH_STACK_SIZE - 1)
				stack[count] = GetRightChild(node)
				count += 1
				node = GetLeftChild(node)

				continue
			}
		}

		if count == 0 {
			break
		}
		count -= 1
		node = stack[count]
	}

	return false
}

ComputeMeshAABB :: proc(shape: ^MeshData, transform: Transform, scale: Vec3) -> AABB {
	scaledLower := Mul(scale, shape.bounds.lowerBound)
	scaledUpper := Mul(scale, shape.bounds.upperBound)
	bounds := AABB{Min(scaledLower, scaledUpper), Max(scaledLower, scaledUpper)}
	return AABB_Transform(transform, bounds)
}

RayCastMesh :: proc(mesh: ^Mesh, input: ^RayCastInput) -> CastOutput {
	data := mesh.data
	meshScale := mesh.scale

	bestOutput := CastOutput{}
	bestOutput.fraction = input.maxFraction
	bestOutput.triangleIndex = NULL_INDEX

	lambda := SplatV(input.maxFraction)

	rayStart := LoadV(&input.origin.x)
	rayDelta := LoadV(&input.translation.x)

	scale := LoadV(&meshScale.x)
	invScale := DivV(oneV, scale)
	ccw := meshScale.x * meshScale.y * meshScale.z > 0.0

	// Use the inverse scaled ray for traversal of the BVH
	invScaledRayStart := MulV(invScale, rayStart)
	invScaledRayDelta := MulV(invScale, rayDelta)
	invScaledRayEnd := AddV(invScaledRayStart, MulV(lambda, invScaledRayDelta))
	invScaledRayMin := MinV(invScaledRayStart, invScaledRayEnd)
	invScaledRayMax := MaxV(invScaledRayStart, invScaledRayEnd)

	count: i32 = 0
	stack: [MESH_STACK_SIZE]^MeshNode = ---
	node := GetRoot(data)
	triangles := GetMeshTriangles(data)
	vertices := GetMeshVertices(data)
	materialIndices := GetMeshMaterialIndices(data)

	for true {
		// Test node/ray overlap using SAT
		nodeMin := LoadV(&node.lowerBound.x)
		nodeMax := LoadV(&node.upperBound.x)
		if TestBoundsOverlap(nodeMin, nodeMax, invScaledRayMin, invScaledRayMax) &&
		   TestBoundsRayOverlap(nodeMin, nodeMax, invScaledRayStart, invScaledRayDelta) {
			// SAT: The node and ray overlap - process leaf node or recurse
			if IsLeaf(node) {
				triangleCount := i32(node.data.asLeaf.triangleCount)
				triangleOffset := i32(node.triangleOffset)

				for index: i32 = 0; index < triangleCount; index += 1 {
					triangleIndex := triangleOffset + index
					triangle := triangles[triangleIndex]

					// Collide ray with triangle in scaled space
					vertex1 := Mul(meshScale, vertices[triangle.index1])
					vertex2, vertex3: Vec3

					// The CPU should predict this branch
					if ccw {
						vertex2 = Mul(meshScale, vertices[triangle.index2])
						vertex3 = Mul(meshScale, vertices[triangle.index3])
					} else {
						vertex2 = Mul(meshScale, vertices[triangle.index3])
						vertex3 = Mul(meshScale, vertices[triangle.index2])
					}

					// Collide ray with triangle in scaled space
					v1 := LoadV(&vertex1.x)
					v2 := LoadV(&vertex2.x)
					v3 := LoadV(&vertex3.x)

					alpha := IntersectRayTriangle(rayStart, rayDelta, v1, v2, v3)
					assert(0 <= alpha && alpha <= 1.0)

					if alpha < bestOutput.fraction {
						edge1 := Sub(vertex2, vertex1)
						edge2 := Sub(vertex3, vertex1)
						bestOutput.normal = Normalize(Cross(edge1, edge2))
						bestOutput.point = Add(input.origin, MulSV(alpha, input.translation))
						bestOutput.fraction = alpha
						bestOutput.triangleIndex = triangleIndex
						bestOutput.materialIndex = i32(materialIndices[triangleIndex])
						bestOutput.hit = true

						// Update ray bounds in unscaled space
						lambda = SplatV(alpha)
						invScaledRayEnd = AddV(invScaledRayStart, MulV(lambda, invScaledRayDelta))
						invScaledRayMin = MinV(invScaledRayStart, invScaledRayEnd)
						invScaledRayMax = MaxV(invScaledRayStart, invScaledRayEnd)
					}
				}
			} else {
				// Determine traversal order (front -> back) and recurse
				axis := i32(node.data.asNode.axis)
				if GetV(invScaledRayDelta, axis) > 0.0 {
					assert(count <= MESH_STACK_SIZE - 1)
					stack[count] = GetRightChild(node)
					count += 1
					node = GetLeftChild(node)
				} else {
					assert(count <= MESH_STACK_SIZE - 1)
					stack[count] = GetLeftChild(node)
					count += 1
					node = GetRightChild(node)
				}

				continue
			}
		}

		if count == 0 {
			break
		}
		count -= 1
		node = stack[count]
	}

	return bestOutput
}

ShapeCastMesh :: proc(mesh: ^Mesh, input: ^ShapeCastInput) -> CastOutput {
	data := mesh.data
	meshScale := mesh.scale

	bestOutput := CastOutput{}
	bestOutput.fraction = input.maxFraction
	bestOutput.triangleIndex = NULL_INDEX

	lambda := SplatV(input.maxFraction)

	shapeBounds := MakeAABB(input.proxy.points, input.proxy.count, input.proxy.radius)
	center := AABB_Center(shapeBounds)
	extents := AABB_Extents(shapeBounds)
	shapeExtent := LoadV(&extents.x)

	rayStart := LoadV(&center.x)
	rayDelta := LoadV(&input.translation.x)
	rayEnd := AddV(rayStart, MulV(lambda, rayDelta))
	rayMin := MinV(rayStart, rayEnd)
	rayMax := MaxV(rayStart, rayEnd)

	scale := LoadV(&meshScale.x)
	invScale := DivV(oneV, scale)
	absInvScale := AbsV(invScale)
	ccw := meshScale.x * meshScale.y * meshScale.z > 0.0

	// Use the inverse scaled shape cast for traversal of the BVH
	invScaledRayStart := MulV(invScale, rayStart)
	invScaledRayDelta := MulV(invScale, rayDelta)
	invScaledRayEnd := AddV(invScaledRayStart, MulV(lambda, invScaledRayDelta))
	invScaledRayMin := MinV(invScaledRayStart, invScaledRayEnd)
	invScaledRayMax := MaxV(invScaledRayStart, invScaledRayEnd)
	invScaledShapeExtent := MulV(absInvScale, shapeExtent)

	count: i32 = 0
	stack: [MESH_STACK_SIZE]^MeshNode = ---
	node := GetRoot(data)
	triangles := GetMeshTriangles(data)
	vertices := GetMeshVertices(data)
	materialIndices := GetMeshMaterialIndices(data)

	for true {
		// Test node/ray overlap using SAT in unscaled space
		nodeMin := SubV(LoadV(&node.lowerBound.x), invScaledShapeExtent)
		nodeMax := AddV(LoadV(&node.upperBound.x), invScaledShapeExtent)

		if TestBoundsOverlap(nodeMin, nodeMax, invScaledRayMin, invScaledRayMax) &&
		   TestBoundsRayOverlap(nodeMin, nodeMax, invScaledRayStart, invScaledRayDelta) {
			// SAT: The node and ray overlap - process leaf node or recurse
			if IsLeaf(node) {
				triangleCount := i32(node.data.asLeaf.triangleCount)
				triangleOffset := i32(node.triangleOffset)

				for index: i32 = 0; index < triangleCount; index += 1 {
					triangleIndex := triangleOffset + index
					triangle := triangles[triangleIndex]

					// Collide shape with triangle in scaled space
					vertex1 := Mul(meshScale, vertices[triangle.index1])
					vertex2, vertex3: Vec3

					// The CPU should predict this branch
					if ccw {
						vertex2 = Mul(meshScale, vertices[triangle.index2])
						vertex3 = Mul(meshScale, vertices[triangle.index3])
					} else {
						vertex2 = Mul(meshScale, vertices[triangle.index3])
						vertex3 = Mul(meshScale, vertices[triangle.index2])
					}

					v1 := LoadV(&vertex1.x)
					v2 := LoadV(&vertex2.x)
					v3 := LoadV(&vertex3.x)

					triangleMin := SubV(MinV(v1, MinV(v2, v3)), shapeExtent)
					triangleMax := AddV(MaxV(v1, MaxV(v2, v3)), shapeExtent)

					// Test extended triangle vs ray bounds overlap in scaled space. Skip edge tests.
					overlap := TestBoundsOverlap(triangleMin, triangleMax, rayMin, rayMax)
					if overlap == false {
						continue
					}

					// This test is in scaled space. vertices and center are scaled.
					signedVolume := SignedVolume(vertex1, vertex2, vertex3, center)
					if signedVolume < 0.0 {
						// Backside
						continue
					}

					// Collide shape with triangle in scaled space.
					origin := vertex1
					triangleVertices := [3]Vec3{Vec3_zero, Sub(vertex2, origin), Sub(vertex3, origin)}
					shiftedOrigin := Transform{Neg(origin), Quat_identity}

					pairInput: ShapeCastPairInput
					pairInput.proxyA = ShapeProxy{&triangleVertices[0], 3, 0.0}
					pairInput.proxyB = input.proxy
					pairInput.transform = shiftedOrigin
					pairInput.maxFraction = bestOutput.fraction
					pairInput.translationB = input.translation
					pairInput.canEncroach = input.canEncroach

					pairOutput := ShapeCast(&pairInput)

					if pairOutput.hit {
						pairOutput.point = Add(pairOutput.point, origin)

						bestOutput = pairOutput
						bestOutput.triangleIndex = triangleIndex
						bestOutput.materialIndex = i32(materialIndices[triangleIndex])

						// Update ray bounds in scaled space
						lambda = SplatV(pairOutput.fraction)
						rayEnd = AddV(rayStart, MulV(lambda, rayDelta))
						rayMin = MinV(rayStart, rayEnd)
						rayMax = MaxV(rayStart, rayEnd)

						// Ray bounds in unscaled space
						invScaledRayEnd = AddV(invScaledRayStart, MulV(lambda, invScaledRayDelta))
						invScaledRayMin = MinV(invScaledRayStart, invScaledRayEnd)
						invScaledRayMax = MaxV(invScaledRayStart, invScaledRayEnd)
					}
				}
			} else {
				// Determine traversal order (front -> back) and recurse
				axis := i32(node.data.asNode.axis)
				if GetV(invScaledRayDelta, axis) > 0.0 {
					assert(count <= MESH_STACK_SIZE - 1)
					stack[count] = GetRightChild(node)
					count += 1
					node = GetLeftChild(node)
				} else {
					assert(count <= MESH_STACK_SIZE - 1)
					stack[count] = GetLeftChild(node)
					count += 1
					node = GetRightChild(node)
				}

				continue
			}
		}

		if count == 0 {
			break
		}
		count -= 1
		node = stack[count]
	}

	return bestOutput
}

CollideMoverAndMesh :: proc(planes: [^]PlaneResult, capacity: i32, shape: ^Mesh, mover: ^Capsule) -> i32 {
	if capacity == 0 {
		return 0
	}

	distanceInput := DistanceInput{}
	distanceInput.proxyB = ShapeProxy{&mover.center1, 2, 0.0}
	distanceInput.transform = Transform_identity
	distanceInput.useRadii = false

	cache := SimplexCache{}
	radius := mover.radius
	center := Lerp(mover.center1, mover.center2, 0.5)
	center1 := LoadV(&mover.center1.x)
	center2 := LoadV(&mover.center2.x)
	r := SplatV(radius)
	boundsMin := SubV(MinV(center1, center2), r)
	boundsMax := AddV(MaxV(center1, center2), r)

	// Scale may have reflection so min/max may become invalid when unscaled
	meshScale := shape.scale
	ccw := meshScale.x * meshScale.y * meshScale.z > 0.0
	scale := LoadV(&meshScale.x)
	invScale := DivV(oneV, scale)
	temp1 := MulV(invScale, boundsMin)
	temp2 := MulV(invScale, boundsMax)
	invScaledBoundsMin := MinV(temp1, temp2)
	invScaledBoundsMax := MaxV(temp1, temp2)
	invScaledBoundsCenter := MulV(halfV, AddV(invScaledBoundsMin, invScaledBoundsMax))
	invScaledBoundsExtent := SubV(invScaledBoundsMax, invScaledBoundsCenter)

	count: i32 = 0
	stack: [MESH_STACK_SIZE]^MeshNode = ---
	node := GetRoot(shape.data)
	triangles := GetMeshTriangles(shape.data)
	vertices := GetMeshVertices(shape.data)
	materialIndices := GetMeshMaterialIndices(shape.data)

	planeCount: i32 = 0
	for planeCount < capacity {
		// Test node overlap in unscaled space
		nodeMin := LoadV(&node.lowerBound.x)
		nodeMax := LoadV(&node.upperBound.x)
		if TestBoundsOverlap(nodeMin, nodeMax, invScaledBoundsMin, invScaledBoundsMax) {
			if IsLeaf(node) {
				triangleCount := i32(node.data.asLeaf.triangleCount)
				triangleOffset := i32(node.triangleOffset)

				for index: i32 = 0; index < triangleCount; index += 1 {
					triangleIndex := triangleOffset + index
					triangle := triangles[triangleIndex]

					vertex1 := vertices[triangle.index1]
					vertex2 := vertices[triangle.index2]
					vertex3 := vertices[triangle.index3]

					if ccw == false {
						vertex2, vertex3 = vertex3, vertex2
					}

					v1 := LoadV(&vertex1.x)
					v2 := LoadV(&vertex2.x)
					v3 := LoadV(&vertex3.x)

					// Test triangle bounds overlap in unscaled space
					if TestBoundsTriangleOverlap(invScaledBoundsCenter, invScaledBoundsExtent, v1, v2, v3) {
						// Compute shape distance in scaled space.
						triangleVertices := [3]Vec3 {
							Mul(meshScale, vertex1),
							Mul(meshScale, vertex2),
							Mul(meshScale, vertex3),
						}

						// Vertices and center are in scaled space.
						signedVolume := SignedVolume(
							triangleVertices[0],
							triangleVertices[1],
							triangleVertices[2],
							center,
						)
						if signedVolume < 0.0 {
							// Backside
							continue
						}

						distanceInput.proxyA = ShapeProxy{&triangleVertices[0], 3, 0.0}

						// reset the cache
						cache.count = 0

						// get distance between triangle and query shape
						distanceOutput := ShapeDistance(&distanceInput, &cache, nil, 0)

						if distanceOutput.distance == 0.0 {
							// deep overlap
						} else if distanceOutput.distance <= mover.radius {
							plane := Plane{distanceOutput.normal, mover.radius - distanceOutput.distance}
							planes[planeCount] = PlaneResult {
								plane,
								distanceOutput.pointA,
								triangleIndex,
								0,
								i32(materialIndices[triangleIndex]),
							}
							planeCount += 1

							if planeCount == capacity {
								return planeCount
							}
						}
					}
				}
			} else {
				// Recurse
				assert(count <= MESH_STACK_SIZE - 1)
				stack[count] = GetRightChild(node)
				count += 1
				node = GetLeftChild(node)

				continue
			}
		}

		if count == 0 {
			break
		}
		count -= 1
		node = stack[count]
	}

	return planeCount
}

QueryMesh :: proc(mesh: ^Mesh, bounds: AABB, fcn: MeshQueryFcn, context_: rawptr) {
	bounds := bounds
	meshScale := mesh.scale
	ccw := meshScale.x * meshScale.y * meshScale.z > 0.0

	// Scale may have reflection so min/max may become invalid when unscaled
	scale := LoadV(&meshScale.x)
	invScale := DivV(oneV, scale)
	temp1 := MulV(invScale, LoadV(&bounds.lowerBound.x))
	temp2 := MulV(invScale, LoadV(&bounds.upperBound.x))
	invScaledBoundsMin := MinV(temp1, temp2)
	invScaledBoundsMax := MaxV(temp1, temp2)
	invScaledBoundsCenter := MulV(halfV, AddV(invScaledBoundsMin, invScaledBoundsMax))
	invScaledBoundsExtent := SubV(invScaledBoundsMax, invScaledBoundsCenter)

	data := mesh.data

	count: i32 = 0
	stack: [MESH_STACK_SIZE]^MeshNode = ---
	node := GetRoot(data)
	triangles := GetMeshTriangles(data)
	vertices := GetMeshVertices(data)

	// Test node overlap in unscaled space
	if TestBoundsOverlap(
		   LoadV(&node.lowerBound.x),
		   LoadV(&node.upperBound.x),
		   invScaledBoundsMin,
		   invScaledBoundsMax,
	   ) ==
	   false {
		return
	}

	for true {
		if IsLeaf(node) {
			triangleCount := i32(node.data.asLeaf.triangleCount)
			triangleOffset := i32(node.triangleOffset)

			for index: i32 = 0; index < triangleCount; index += 1 {
				triangleIndex := triangleOffset + index
				triangle := triangles[triangleIndex]

				vertex1 := vertices[triangle.index1]
				vertex2 := vertices[triangle.index2]
				vertex3 := vertices[triangle.index3]
				v1 := LoadV(&vertex1.x)
				v2 := LoadV(&vertex2.x)
				v3 := LoadV(&vertex3.x)

				// Perform triangle overlap test in unscaled space.
				// todo it is possible that some margins are getting scaled
				if TestBoundsTriangleOverlap(invScaledBoundsCenter, invScaledBoundsExtent, v1, v2, v3) {
					a := Mul(meshScale, vertex1)
					b, c: Vec3
					if ccw {
						b = Mul(meshScale, vertex2)
						c = Mul(meshScale, vertex3)
					} else {
						b = Mul(meshScale, vertex3)
						c = Mul(meshScale, vertex2)
					}

					result := fcn(a, b, c, triangleIndex, context_)
					if result == false {
						return
					}
				}
			}
		} else {
			leftChild := GetLeftChild(node)
			rightChild := GetRightChild(node)

			overlapLeft := TestBoundsOverlap(
				LoadV(&leftChild.lowerBound.x),
				LoadV(&leftChild.upperBound.x),
				invScaledBoundsMin,
				invScaledBoundsMax,
			)
			overlapRight := TestBoundsOverlap(
				LoadV(&rightChild.lowerBound.x),
				LoadV(&rightChild.upperBound.x),
				invScaledBoundsMin,
				invScaledBoundsMax,
			)

			if overlapLeft {
				if overlapRight {
					assert(count <= MESH_STACK_SIZE - 1)
					stack[count] = rightChild
					count += 1
				}

				node = leftChild
				continue
			}

			if overlapRight {
				node = rightChild
				continue
			}
		}

		if count == 0 {
			break
		}
		count -= 1
		node = stack[count]
	}
}
