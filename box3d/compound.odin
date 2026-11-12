// Ported from box3d src/compound.c and src/compound.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:runtime"
import "core:mem"

// compound.h

GetCompoundTreeView :: #force_inline proc(compound: ^CompoundData) -> TreeView {
	view := TreeView {
		nodes   = cast([^]TreeNode)(uintptr(compound) + uintptr(compound.nodeOffset)),
		proxies = nil,
	}
	return view
}

// unified: not ported: b3CompoundTimeOfImpact is declared here but its definition is disabled (#if 0) in compound.c.

// Transforms a sweep for a compound child shape
// (MakeCompoundChildSweep, defined below)

// (CollideMoverAndCompound, defined below)

// compound.c

SharedHull :: struct {
	hull:       ^HullData,
	hullOffset: i32,
}

SharedMesh :: struct {
	meshData:   ^MeshData,
	meshOffset: i32,
}

HullInstance :: struct {
	transform:     Transform,
	hullOffset:    u32,
	materialIndex: u16,
	padding:       u16,
}

#assert(size_of(HullInstance) == size_of(Transform) + size_of(u32) + 2 * size_of(u16), "review padding")

MeshInstance :: struct {
	transform:      Transform,
	scale:          Vec3,
	meshOffset:     u32,
	materialOffset: u32,
	materialCount:  u32,
}

#assert(size_of(CompoundCapsule) == size_of(Capsule) + 2 * size_of(u16), "review padding")
#assert(size_of(CompoundSphere) == size_of(Sphere) + 2 * size_of(u16), "review padding")
#assert(size_of(CompoundData) == size_of(u64) + size_of(AABB) + 16 * size_of(i32), "review padding")

GetCompoundMaterials :: proc(compound: ^CompoundData) -> [^]SurfaceMaterial {
	if compound.materialOffset == 0 {
		return nil
	}

	return cast([^]SurfaceMaterial)(uintptr(compound) + uintptr(compound.materialOffset))
}

GetCompoundCapsule :: proc(compound: ^CompoundData, index: i32) -> CompoundCapsule {
	assert(0 <= index && index < compound.capsuleCount && compound.capsuleOffset > 0)

	result := CompoundCapsule{}
	if compound.capsuleOffset == 0 {
		return result
	}

	capsules := cast([^]CompoundCapsule)(uintptr(compound) + uintptr(compound.capsuleOffset))
	return capsules[index]
}

GetCompoundHull :: proc(compound: ^CompoundData, index: i32) -> CompoundHull {
	assert(0 <= index && index < compound.hullCount && compound.hullOffset > 0)

	result := CompoundHull{}
	if compound.hullOffset == 0 {
		return result
	}

	hullInstances := cast([^]HullInstance)(uintptr(compound) + uintptr(compound.hullOffset))
	hullOffset := hullInstances[index].hullOffset
	assert(int(hullOffset) >= int(compound.hullOffset) + int(compound.hullCount) * size_of(HullInstance))
	result.hull = cast(^HullData)(uintptr(compound) + uintptr(hullOffset))
	result.transform = hullInstances[index].transform
	result.materialIndex = hullInstances[index].materialIndex
	return result
}

GetCompoundMesh :: proc(compound: ^CompoundData, index: i32) -> CompoundMesh {
	assert(0 <= index && index < compound.meshCount && compound.meshOffset > 0)

	result := CompoundMesh{}
	if compound.meshOffset == 0 {
		return result
	}

	meshInstances := cast([^]MeshInstance)(uintptr(compound) + uintptr(compound.meshOffset))
	meshOffset := meshInstances[index].meshOffset
	assert(int(meshOffset) >= int(compound.meshOffset) + int(compound.meshCount) * size_of(HullInstance))
	result.meshData = cast(^MeshData)(uintptr(compound) + uintptr(meshOffset))
	result.transform = meshInstances[index].transform
	result.scale = meshInstances[index].scale
	result.materialIndices = cast([^]u16)(uintptr(compound) + uintptr(meshInstances[index].materialOffset))
	result.materialCount = i32(meshInstances[index].materialCount)
	return result
}

GetCompoundSphere :: proc(compound: ^CompoundData, index: i32) -> CompoundSphere {
	assert(0 <= index && index < compound.sphereCount && compound.sphereOffset > 0)

	result := CompoundSphere{}
	if compound.sphereOffset == 0 {
		return result
	}

	spheres := cast([^]CompoundSphere)(uintptr(compound) + uintptr(compound.sphereOffset))
	return spheres[index]
}

GetCompoundChild :: proc(compound: ^CompoundData, childIndex: i32) -> ChildShape {
	childIndex := childIndex

	// Capsule?
	if 0 <= childIndex && childIndex < compound.capsuleCount {
		assert(compound.capsuleOffset > 0)
		capsules := cast([^]CompoundCapsule)(uintptr(compound) + uintptr(compound.capsuleOffset))
		child: ChildShape
		child.capsule = capsules[childIndex].capsule
		child.transform = Transform_identity
		child.materialIndices = &capsules[childIndex].materialIndex
		child.materialCount = 1
		child.type = .capsuleShape
		return child
	}
	childIndex -= compound.capsuleCount

	// Hull?
	if 0 <= childIndex && childIndex < compound.hullCount {
		assert(compound.hullOffset > 0)
		hullInstances := cast([^]HullInstance)(uintptr(compound) + uintptr(compound.hullOffset))
		hullOffset := hullInstances[childIndex].hullOffset
		child: ChildShape
		child.hull = cast(^HullData)(uintptr(compound) + uintptr(hullOffset))
		child.transform = hullInstances[childIndex].transform
		child.materialIndices = &hullInstances[childIndex].materialIndex
		child.materialCount = 1
		child.type = .hullShape
		return child
	}
	childIndex -= compound.hullCount

	// Mesh?
	if 0 <= childIndex && childIndex < compound.meshCount {
		compoundMesh := GetCompoundMesh(compound, childIndex)

		child: ChildShape
		child.mesh = Mesh {
			data  = compoundMesh.meshData,
			scale = compoundMesh.scale,
		}
		child.transform = compoundMesh.transform
		child.materialIndices = compoundMesh.materialIndices
		child.materialCount = compoundMesh.materialCount
		child.type = .meshShape
		return child
	}
	childIndex -= compound.meshCount

	assert(0 <= childIndex && childIndex < compound.sphereCount)

	// Sphere?
	{
		assert(compound.sphereOffset > 0)
		spheres := cast([^]CompoundSphere)(uintptr(compound) + uintptr(compound.sphereOffset))
		child: ChildShape
		child.sphere = spheres[childIndex].sphere
		child.transform = Transform_identity
		child.materialIndices = &spheres[childIndex].materialIndex
		child.materialCount = 1
		child.type = .sphereShape
		return child
	}
}

// unified: vt_wyhash (verstable's default hash) is rapidhash here. The hash only spreads keys over the map's chains:
// the maps are lookup-only (never iterated), so results don't depend on it.
@(private = "file")
HashMesh :: #force_inline proc(mesh: ^MeshData) -> u64 {
	return rapidhash(mesh, int(mesh.byteCount))
}
// unified end

@(private = "file")
CompareMeshes :: proc(mesh1: ^MeshData, mesh2: ^MeshData) -> bool {
	if mesh1 == mesh2 {
		return true
	}

	if mesh1.byteCount != mesh2.byteCount {
		return false
	}

	result := mem.compare_ptrs(mesh1, mesh2, int(mesh1.byteCount))
	return result == 0
}

// unified: verstable's b3MeshMap and b3MaterialMap (keyed by the content behind a pointer: HASH_FN and CMPR_FN)
// are one small chained map here: an Odin map (heap allocator) from the content hash to the first bucket of a chain,
// buckets in a dynamic array, compared with CMPR_FN. Only get_or_insert is used and the maps are never iterated, so
// pointer values and hash order never affect results. Their storage is not counted by GetByteCount.
@(private = "file")
ContentMap_bucket :: struct($K: typeid) {
	key:       K,
	val:       i32,
	chainNext: i32,
}

@(private = "file")
ContentMap :: struct($K: typeid) {
	chains:  map[u64]i32,
	buckets: [dynamic]ContentMap_bucket(K),
}

@(private = "file")
ContentMap_itr :: struct($K: typeid) {
	data: ^ContentMap_bucket(K),
}

@(private = "file")
ContentMap_init :: proc(table: ^ContentMap($K)) {
	table.chains = make(map[u64]i32, runtime.heap_allocator())
	table.buckets = make([dynamic]ContentMap_bucket(K), runtime.heap_allocator())
}

@(private = "file")
ContentMap_reserve :: proc(table: ^ContentMap($K), size: i32) {
	reserve(&table.chains, int(size))
	reserve(&table.buckets, int(size))
}

// Inserts the key with val if it does not exist; returns the existing or new entry.
@(private = "file")
ContentMap_get_or_insert :: proc(
	table: ^ContentMap($K),
	key: K,
	val: i32,
	$HASH_FN: proc(key: K) -> u64,
	$CMPR_FN: proc(a, b: K) -> bool,
) -> ContentMap_itr(K) {
	hash := HASH_FN(key)
	head, found := table.chains[hash]
	if found {
		for index := head; index != NULL_INDEX; index = table.buckets[index].chainNext {
			if CMPR_FN(table.buckets[index].key, key) {
				return {&table.buckets[index]}
			}
		}
	}

	append(&table.buckets, ContentMap_bucket(K){key, val, found ? head : NULL_INDEX})
	index := i32(len(table.buckets) - 1)
	table.chains[hash] = index
	return {&table.buckets[index]}
}

@(private = "file")
ContentMap_cleanup :: proc(table: ^ContentMap($K)) {
	delete(table.chains)
	delete(table.buckets)
	table^ = {}
}

@(private = "file")
MeshMap :: ContentMap(^MeshData)

@(private = "file")
MeshMap_itr :: ContentMap_itr(^MeshData)

@(private = "file")
MeshMap_get_or_insert :: proc(table: ^MeshMap, key: ^MeshData, val: i32) -> MeshMap_itr {
	return ContentMap_get_or_insert(table, key, val, HashMesh, CompareMeshes)
}
// unified end

#assert(size_of(SurfaceMaterial) == 40, "review padding")

// unified: vt_wyhash is rapidhash here (see HashMesh)
@(private = "file")
HashMaterial :: #force_inline proc(material: ^SurfaceMaterial) -> u64 {
	assert(material.padding == 0)

	return rapidhash(material, size_of(SurfaceMaterial))
}
// unified end

@(private = "file")
CompareMaterials :: proc(mat1: ^SurfaceMaterial, mat2: ^SurfaceMaterial) -> bool {
	assert(mat1.padding == 0 && mat2.padding == 0)

	if mat1 == mat2 {
		return true
	}

	result := mem.compare_ptrs(mat1, mat2, size_of(SurfaceMaterial))
	return result == 0
}

// unified: b3MaterialMap is a ContentMap (see MeshMap above)
@(private = "file")
MaterialMap :: ContentMap(^SurfaceMaterial)

@(private = "file")
MaterialMap_itr :: ContentMap_itr(^SurfaceMaterial)

@(private = "file")
MaterialMap_get_or_insert :: proc(table: ^MaterialMap, key: ^SurfaceMaterial, val: i32) -> MaterialMap_itr {
	return ContentMap_get_or_insert(table, key, val, HashMaterial, CompareMaterials)
}
// unified end

CreateCompound :: proc(def: ^CompoundDef) -> ^CompoundData {
	convexCount := def.capsuleCount + def.hullCount + def.sphereCount
	shapeCount := convexCount + def.meshCount

	if shapeCount >= MAX_CHILD_SHAPES {
		assert(false)
		return nil
	}

	tree := DynamicTree_Create(shapeCount)

	childIndex: i32 = 0

	// Instances
	capsuleCount := def.capsuleCount
	capsuleInstances := cast([^]CompoundCapsule)AllocZero(int(capsuleCount) * size_of(CompoundCapsule))
	hullCount := def.hullCount
	hullInstances := cast([^]HullInstance)AllocZero(int(hullCount) * size_of(HullInstance))
	meshCount := def.meshCount
	meshInstances := cast([^]MeshInstance)AllocZero(int(meshCount) * size_of(MeshInstance))
	sphereCount := def.sphereCount
	sphereInstances := cast([^]CompoundSphere)AllocZero(int(sphereCount) * size_of(CompoundSphere))

	// Determine material capacity.
	materialCapacity := convexCount
	for i: i32 = 0; i < def.meshCount; i += 1 {
		assert(def.meshes[i].materialCount > 0)
		materialCapacity += def.meshes[i].materialCount
	}

	// Material map for convex material sharing. Mesh materials are not shared for simplicity.
	materialMap: MaterialMap
	ContentMap_init(&materialMap)
	ContentMap_reserve(&materialMap, materialCapacity)
	materials := cast([^]SurfaceMaterial)AllocZero(int(materialCapacity) * size_of(SurfaceMaterial))
	materialCount: i32 = 0

	meshMaterialTotal := materialCapacity - convexCount
	meshMaterialStaging := cast([^]u16)AllocZero(int(meshMaterialTotal) * size_of(u16))
	meshMaterialCursor: i32 = 0

	for i: i32 = 0; i < def.capsuleCount; i += 1 {
		capsuleDef := &def.capsules[i]
		capsuleInstances[i].capsule = capsuleDef.capsule

		// Look for an existing material.
		materialItr := MaterialMap_get_or_insert(&materialMap, &capsuleDef.material, materialCount)

		// Get the shared material index.
		materialIndex := materialItr.data.val
		capsuleInstances[i].materialIndex = u16(materialIndex)

		// Is this a new material?
		if materialIndex == materialCount {
			materials[materialIndex] = capsuleDef.material
			materialCount += 1
		}

		aabb := ComputeCapsuleAABB(&capsuleDef.capsule, Transform_identity)
		DynamicTree_CreateProxy(&tree, aabb, max(u64), u64(childIndex))
		childIndex += 1
	}

	// Hulls
	sharedHulls := cast([^]SharedHull)AllocZero(int(hullCount) * size_of(SharedHull))
	sharedHullCount: i32 = 0

	if hullCount > 0 {
		hullMap: HullMap
		HullMap_init(&hullMap)
		HullMap_reserve(&hullMap, int(hullCount))

		for i: i32 = 0; i < hullCount; i += 1 {
			hullDef := &def.hulls[i]
			hull := hullDef.hull
			aabb := ComputeHullAABB(hull, hullDef.transform)
			DynamicTree_CreateProxy(&tree, aabb, max(u64), u64(childIndex))
			childIndex += 1

			// Look for an existing material.
			materialItr := MaterialMap_get_or_insert(&materialMap, &hullDef.material, materialCount)

			// Get the shared material index.
			materialIndex := materialItr.data.val
			hullInstances[i].materialIndex = u16(materialIndex)

			// Is this a new material?
			if materialIndex == materialCount {
				materials[materialIndex] = hullDef.material
				materialCount += 1
			}

			hullInstances[i].transform = hullDef.transform

			// Look for an existing matching hull
			itr := HullMap_get_or_insert(&hullMap, hull, sharedHullCount)

			// Get the unique index for this hull
			sharedHullIndex := itr.data.val

			// The offset isn't known yet, so store the index of the shared hull
			hullInstances[i].hullOffset = u32(sharedHullIndex)

			// Is this a new hull?
			if sharedHullIndex == sharedHullCount {
				// Create a shared hull. The offset is determined below.
				sharedHulls[sharedHullIndex].hull = hull
				sharedHulls[sharedHullIndex].hullOffset = NULL_INDEX
				sharedHullCount += 1
			}
		}

		HullMap_cleanup(&hullMap)
	}

	// Meshes
	sharedMeshes := cast([^]SharedMesh)AllocZero(int(meshCount) * size_of(SharedMesh))
	sharedMeshCount: i32 = 0

	if meshCount > 0 {
		meshMap: MeshMap
		ContentMap_init(&meshMap)
		ContentMap_reserve(&meshMap, meshCount)

		for i: i32 = 0; i < meshCount; i += 1 {
			meshDef := &def.meshes[i]

			meshData := meshDef.meshData
			aabb := ComputeMeshAABB(meshData, meshDef.transform, meshDef.scale)
			DynamicTree_CreateProxy(&tree, aabb, max(u64), u64(childIndex))
			childIndex += 1

			// No effort to share mesh materials. It would be easier to do if the number of materials was limited.
			assert(meshData.materialCount == meshDef.materialCount)

			meshMaterialStart := meshMaterialCursor

			for j: i32 = 0; j < meshDef.materialCount; j += 1 {
				// Look for an existing material.
				materialItr := MaterialMap_get_or_insert(&materialMap, &meshDef.materials[j], materialCount)

				// Get the shared material index.
				materialIndex := materialItr.data.val
				meshMaterialStaging[meshMaterialCursor] = u16(materialIndex)
				meshMaterialCursor += 1

				// Is this a new material?
				if materialIndex == materialCount {
					materials[materialIndex] = meshDef.materials[j]
					materialCount += 1
				}
			}

			// Look for an existing matching mesh.
			itr := MeshMap_get_or_insert(&meshMap, meshData, sharedMeshCount)

			// Get the shared mesh index.
			sharedMeshIndex := itr.data.val

			// Create mesh instance.
			meshInstances[i].transform = def.meshes[i].transform
			meshInstances[i].scale = def.meshes[i].scale

			meshInstances[i].materialOffset = u32(meshMaterialStart)
			meshInstances[i].materialCount = u32(meshDef.materialCount)

			// The offset isn't known yet, so store the index of the shared mesh.
			meshInstances[i].meshOffset = u32(sharedMeshIndex)

			// Is this a new mesh?
			if sharedMeshIndex == sharedMeshCount {
				// Create a shared mesh. The offset is determined below.
				sharedMeshes[sharedMeshIndex].meshData = meshData
				sharedMeshes[sharedMeshIndex].meshOffset = NULL_INDEX
				sharedMeshCount += 1
			}
		}

		ContentMap_cleanup(&meshMap)
	}

	// Spheres
	for i: i32 = 0; i < def.sphereCount; i += 1 {
		sphereDef := &def.spheres[i]
		sphereInstances[i].sphere = sphereDef.sphere

		// Look for an existing material.
		materialItr := MaterialMap_get_or_insert(&materialMap, &sphereDef.material, materialCount)

		// Get the shared material index.
		materialIndex := materialItr.data.val
		sphereInstances[i].materialIndex = u16(materialIndex)

		// Is this a new material?
		if materialIndex == materialCount {
			materials[materialIndex] = sphereDef.material
			materialCount += 1
		}

		aabb := ComputeSphereAABB(&sphereDef.sphere, Transform_identity)
		DynamicTree_CreateProxy(&tree, aabb, max(u64), u64(childIndex))
		childIndex += 1
	}

	assert(materialCount <= materialCapacity)
	assert(materialCount <= i32(max(u16)))
	assert(tree.proxyCount > 0)

	DynamicTree_Rebuild(&tree, true)

	// Tree nodes. The rebuild is dense, so only the live nodes travel.
	byteCount := AlignUp8(size_of(CompoundData))
	nodeOffset := i32(byteCount)
	byteCount += AlignUp8(int(tree.nodeEnd) * size_of(TreeNode))
	materialOffset := i32(byteCount)
	byteCount += AlignUp8(int(materialCount) * size_of(SurfaceMaterial))
	capsuleOffset := i32(byteCount)
	byteCount += AlignUp8(int(def.capsuleCount) * size_of(CompoundCapsule))

	// Hull data layout has another level of indirection to allow for tight data packing
	// 1. hull instance array : hull count array of b3HullInstance with individual hull transforms and offsets
	// 2. heterogeneous array of shared hull data : each shared hull can have a different byte count, so direct indexing is not
	// possible
	hullArrayOffset := i32(byteCount)

	// Array of hull instances
	byteCount += AlignUp8(int(hullCount) * size_of(HullInstance))

	// Packed shared hull blobs
	for i: i32 = 0; i < sharedHullCount; i += 1 {
		sharedHulls[i].hullOffset = i32(byteCount)
		byteCount += AlignUp8(int(sharedHulls[i].hull.byteCount))
	}

	// Mesh data layout has another level of indirection to allow for tight data packing
	// 1. mesh instance array : mesh count array of b3MeshInstance with individual mesh transform, scale, and offset
	// 2. heterogeneous array of shared mesh data : each shared mesh can have a different byte count, so direct indexing is not
	// possible
	meshArrayOffset := i32(byteCount)

	// Array of mesh instances
	byteCount += AlignUp8(int(meshCount) * size_of(MeshInstance))

	meshMaterialMapOffset := i32(byteCount)
	byteCount += AlignUp8(int(meshMaterialTotal) * size_of(u16))

	// Packed shared mesh blobs
	for i: i32 = 0; i < sharedMeshCount; i += 1 {
		sharedMeshes[i].meshOffset = i32(byteCount)
		byteCount += AlignUp8(int(sharedMeshes[i].meshData.byteCount))
	}

	sphereOffset := i32(byteCount)
	byteCount += AlignUp8(int(def.sphereCount) * size_of(CompoundSphere))

	compound := cast(^CompoundData)AllocHost(byteCount) // unified: geometry is the user's, never in a world arena (world_arena.odin)
	mem.zero(compound, byteCount)

	compound.version = COMPOUND_VERSION
	compound.byteCount = i32(byteCount)
	compound.nodeOffset = nodeOffset
	compound.bounds = DynamicTree_GetRootBounds(&tree)
	compound.treeHeight = DynamicTree_GetHeight(&tree)
	compound.nodeCount = tree.nodeEnd
	compound.materialOffset = materialOffset
	compound.materialCount = materialCount
	compound.capsuleOffset = capsuleOffset
	compound.capsuleCount = capsuleCount
	compound.hullOffset = hullArrayOffset
	compound.hullCount = hullCount
	compound.meshOffset = meshArrayOffset
	compound.meshCount = meshCount
	compound.sphereOffset = sphereOffset
	compound.sphereCount = sphereCount

	// Tree nodes
	nodes := cast([^]TreeNode)(uintptr(compound) + uintptr(nodeOffset))
	mem.copy_non_overlapping(nodes, tree.nodes, int(tree.nodeEnd) * size_of(TreeNode))

	// Materials
	assert(materialCount > 0)
	destinationMaterials := cast([^]SurfaceMaterial)(uintptr(compound) + uintptr(compound.materialOffset))
	if materials != nil {
		mem.copy_non_overlapping(destinationMaterials, materials, int(materialCount) * size_of(SurfaceMaterial))
	}

	// Capsules
	if def.capsuleCount > 0 {
		assert(compound.capsuleOffset > 0)
		capsules := cast([^]CompoundCapsule)(uintptr(compound) + uintptr(compound.capsuleOffset))
		mem.copy_non_overlapping(capsules, capsuleInstances, int(capsuleCount) * size_of(CompoundCapsule))
	}

	// Hulls
	for i: i32 = 0; i < hullCount; i += 1 {
		// Fix up offsets
		sharedIndex := i32(hullInstances[i].hullOffset)
		assert(0 <= sharedIndex && sharedIndex < sharedHullCount)
		hullInstances[i].hullOffset = u32(sharedHulls[sharedIndex].hullOffset)
	}

	destinationHullInstances := cast([^]HullInstance)(uintptr(compound) + uintptr(hullArrayOffset))
	mem.copy_non_overlapping(destinationHullInstances, hullInstances, int(hullCount) * size_of(HullInstance))

	for i: i32 = 0; i < sharedHullCount; i += 1 {
		offset := sharedHulls[i].hullOffset
		destinationHull := cast(^HullData)(uintptr(compound) + uintptr(offset))
		mem.copy_non_overlapping(destinationHull, sharedHulls[i].hull, int(sharedHulls[i].hull.byteCount))
	}

	compound.sharedHullCount = sharedHullCount

	// Meshes
	for i: i32 = 0; i < meshCount; i += 1 {
		// Fix up offsets
		sharedIndex := i32(meshInstances[i].meshOffset)
		assert(0 <= sharedIndex && sharedIndex < sharedMeshCount)
		meshInstances[i].meshOffset = u32(sharedMeshes[sharedIndex].meshOffset)

		materialStart := meshInstances[i].materialOffset
		meshInstances[i].materialOffset = u32(meshMaterialMapOffset) + materialStart * u32(size_of(u16))
	}

	destinationMeshInstances := cast([^]MeshInstance)(uintptr(compound) + uintptr(meshArrayOffset))
	mem.copy_non_overlapping(destinationMeshInstances, meshInstances, int(meshCount) * size_of(MeshInstance))

	if meshMaterialTotal > 0 {
		destinationMeshMaterials := cast([^]u16)(uintptr(compound) + uintptr(meshMaterialMapOffset))
		mem.copy_non_overlapping(destinationMeshMaterials, meshMaterialStaging, int(meshMaterialTotal) * size_of(u16))
	}

	for i: i32 = 0; i < sharedMeshCount; i += 1 {
		offset := sharedMeshes[i].meshOffset
		destinationMesh := cast(^MeshData)(uintptr(compound) + uintptr(offset))
		mem.copy_non_overlapping(destinationMesh, sharedMeshes[i].meshData, int(sharedMeshes[i].meshData.byteCount))
	}

	compound.sharedMeshCount = sharedMeshCount

	// Spheres
	if def.sphereCount > 0 {
		assert(compound.sphereOffset > 0)
		spheres := cast([^]CompoundSphere)(uintptr(compound) + uintptr(compound.sphereOffset))
		mem.copy_non_overlapping(spheres, sphereInstances, int(sphereCount) * size_of(CompoundSphere))
	}

	ContentMap_cleanup(&materialMap)
	Free(sharedHulls, int(hullCount) * size_of(SharedHull))
	Free(sharedMeshes, int(meshCount) * size_of(SharedMesh))
	Free(capsuleInstances, int(capsuleCount) * size_of(CompoundCapsule))
	Free(hullInstances, int(hullCount) * size_of(HullInstance))
	Free(meshInstances, int(meshCount) * size_of(MeshInstance))
	Free(sphereInstances, int(sphereCount) * size_of(CompoundSphere))
	Free(materials, int(materialCapacity) * size_of(SurfaceMaterial))
	Free(meshMaterialStaging, int(meshMaterialTotal) * size_of(u16))
	DynamicTree_Destroy(&tree)

	return compound
}

DestroyCompound :: proc(compound: ^CompoundData) {
	Free(compound, int(compound.byteCount))
}

ValidateCompound :: proc(bytes: [^]u8, byteCount: i32) -> ^CompoundData {
	if bytes == nil || byteCount < i32(size_of(CompoundData)) {
		return nil
	}

	compound := cast(^CompoundData)bytes
	if compound.version != COMPOUND_VERSION {
		return nil
	}

	if compound.byteCount != byteCount {
		return nil
	}

	if compound.nodeOffset < i32(size_of(CompoundData)) || (compound.nodeOffset & 7) != 0 {
		return nil
	}

	if compound.nodeCount < 2 {
		return nil
	}

	nodeEnd := i64(compound.nodeOffset) + i64(compound.nodeCount) * i64(size_of(TreeNode))
	if nodeEnd > i64(byteCount) {
		return nil
	}

	return compound
}

ComputeCompoundAABB :: proc(shape: ^CompoundData, transform: Transform) -> AABB {
	return AABB_Transform(transform, shape.bounds)
}

CompoundOverlapContext :: struct {
	compound: ^CompoundData,
	// proxy in the compound frame
	proxy:    ShapeProxy,
	overlap:  bool,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
CompoundOverlapCallback :: proc "c" (proxyId: i32, userData: u64, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	_ = proxyId

	childIndex := i32(userData)
	overlapContext := cast(^CompoundOverlapContext)context_
	child := GetCompoundChild(overlapContext.compound, childIndex)

	overlap := false
	#partial switch child.type {
	case .capsuleShape:
		overlap = OverlapCapsule(&child.capsule, child.transform, &overlapContext.proxy)

	case .hullShape:
		overlap = OverlapHull(child.hull, child.transform, &overlapContext.proxy)

	case .meshShape:
		overlap = OverlapMesh(&child.mesh, child.transform, &overlapContext.proxy)

	case .sphereShape:
		overlap = OverlapSphere(&child.sphere, child.transform, &overlapContext.proxy)

	case:
		assert(false)
	}

	if overlap {
		// Done
		overlapContext.overlap = true
		return false
	}

	// Continue the query if there is no overlap
	return true
}

// This is dealing with multiple transforms:
// - the compound shape transform
// - the compound child shape transforms
OverlapCompound :: proc(shape: ^CompoundData, shapeTransform: Transform, proxy: ^ShapeProxy) -> bool {
	assert(0 < proxy.count && proxy.count <= MAX_SHAPE_CAST_POINTS)

	// Use local proxy.
	buffer: [MAX_SHAPE_CAST_POINTS]Vec3 = ---
	context_ := CompoundOverlapContext {
		compound = shape,
		proxy    = MakeLocalProxy(proxy, shapeTransform, &buffer[0]),
		overlap  = false,
	}

	aabb := ComputeProxyAABB(&context_.proxy)

	// This query must be in the compound frame.
	view := GetCompoundTreeView(shape)
	_ = TreeView_Query(&view, aabb, max(u64), false, CompoundOverlapCallback, &context_)

	return context_.overlap
}

CompoundCastContext :: struct {
	compound:   ^CompoundData,
	output:     ^CastOutput,
	// origin of the shape cast, the box cast callback only carries the advancing fraction
	shapeInput: ^ShapeCastInput,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
CompoundRayCastCallback :: proc "c" (input: ^RayCastInput, proxyId: i32, userData: u64, context_: rawptr) -> f32 {
	context = runtime.default_context()
	// unified end
	_ = proxyId

	castContext := cast(^CompoundCastContext)context_
	compound := castContext.compound

	childIndex := i32(userData)

	child := GetCompoundChild(compound, childIndex)

	// Get the input in child local space.
	localInput := input^
	localInput.origin = InvTransformPoint(child.transform, input.origin)
	localInput.translation = InvRotateVector(child.transform.q, input.translation)

	output := CastOutput{}

	#partial switch child.type {
	case .capsuleShape:
		output = RayCastCapsule(&child.capsule, &localInput)
		output.materialIndex = i32(child.materialIndices[0])

	case .hullShape:
		output = RayCastHull(child.hull, &localInput)
		output.materialIndex = i32(child.materialIndices[0])

	case .meshShape:
		{
			output = RayCastMesh(&child.mesh, &localInput)
			assert(0 <= output.materialIndex && output.materialIndex < child.materialCount)
			output.materialIndex = i32(child.materialIndices[output.materialIndex])
		}

	case .sphereShape:
		output = RayCastSphere(&child.sphere, &localInput)
		output.materialIndex = i32(child.materialIndices[0])

	case:
		assert(false)
	}

	if output.hit {
		output.point = TransformPoint(child.transform, output.point)
		output.normal = RotateVector(child.transform.q, output.normal)
		output.childIndex = childIndex
		castContext.output^ = output
		return output.fraction
	}

	return input.maxFraction
}

RayCastCompound :: proc(shape: ^CompoundData, input: ^RayCastInput) -> CastOutput {
	result := CastOutput{}

	context_ := CompoundCastContext {
		compound = shape,
		output   = &result,
	}
	view := GetCompoundTreeView(shape)
	_ = TreeView_RayCast(&view, input, max(u64), false, CompoundRayCastCallback, &context_)
	return result
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
CompoundShapeCastCallback :: proc "c" (input: ^BoxCastInput, proxyId: i32, userData: u64, context_: rawptr) -> f32 {
	context = runtime.default_context()
	// unified end
	_ = proxyId

	castContext := cast(^CompoundCastContext)context_
	compound := castContext.compound
	shapeInput := castContext.shapeInput

	childIndex := i32(userData)

	child := GetCompoundChild(compound, childIndex)

	// Get the input in the child local space.
	localInput := ShapeCastInput{}
	localPoints: [MAX_SHAPE_CAST_POINTS]Vec3 = ---
	localInput.proxy = MakeLocalProxy(&shapeInput.proxy, child.transform, &localPoints[0])
	localInput.translation = InvRotateVector(child.transform.q, shapeInput.translation)
	localInput.maxFraction = input.maxFraction
	localInput.canEncroach = shapeInput.canEncroach

	output := CastOutput{}

	#partial switch child.type {
	case .capsuleShape:
		output = ShapeCastCapsule(&child.capsule, &localInput)
		output.materialIndex = i32(child.materialIndices[0])

	case .hullShape:
		output = ShapeCastHull(child.hull, &localInput)
		output.materialIndex = i32(child.materialIndices[0])

	case .meshShape:
		{
			output = ShapeCastMesh(&child.mesh, &localInput)
			assert(0 <= output.materialIndex && output.materialIndex < child.materialCount)
			output.materialIndex = i32(child.materialIndices[output.materialIndex])
		}

	case .sphereShape:
		output = ShapeCastSphere(&child.sphere, &localInput)
		output.materialIndex = i32(child.materialIndices[0])

	case:
		assert(false)
	}

	if output.hit {
		output.point = TransformPoint(child.transform, output.point)
		output.normal = RotateVector(child.transform.q, output.normal)
		output.childIndex = childIndex
		castContext.output^ = output
		return output.fraction
	}

	return input.maxFraction
}

ShapeCastCompound :: proc(shape: ^CompoundData, input: ^ShapeCastInput) -> CastOutput {
	result := CastOutput{}

	if input.proxy.count == 0 {
		return result
	}

	context_ := CompoundCastContext {
		compound   = shape,
		output     = &result,
		shapeInput = input,
	}

	// The compound tree is in the compound local frame, so the proxy box needs no origin offset
	box := MakeAABB(input.proxy.points, input.proxy.count, input.proxy.radius)
	treeInput := BoxCastInput{box, input.translation, input.maxFraction}
	view := GetCompoundTreeView(shape)
	_ = TreeView_BoxCast(&view, &treeInput, max(u64), false, CompoundShapeCastCallback, &context_)
	return result
}

CompoundQueryContext :: struct {
	compound:    ^CompoundData,
	fcn:         CompoundQueryFcn,
	userContext: rawptr,
}

// unified: TreeQueryCallbackFcn_ (C: static TreeQueryCallbackFcn) collides with the TreeQueryCallbackFcn type
@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
TreeQueryCallbackFcn_ :: proc "c" (proxyId: i32, userData: u64, treeContext: rawptr) -> bool {
	// unified end
	_ = proxyId
	context_ := cast(^CompoundQueryContext)treeContext
	return context_.fcn(context_.compound, i32(userData), context_.userContext)
}

QueryCompound :: proc(compound: ^CompoundData, aabb: AABB, fcn: CompoundQueryFcn, context_: rawptr) {
	compoundContext := CompoundQueryContext {
		compound    = compound,
		fcn         = fcn,
		userContext = context_,
	}

	view := GetCompoundTreeView(compound)
	TreeView_Query(&view, aabb, DEFAULT_MASK_BITS, false, TreeQueryCallbackFcn_, &compoundContext)
}

// unified: not ported: the disabled (#if 0) b3CompoundImpactContext, b3CompoundTimeOfImpactFcn and
// b3CompoundTimeOfImpact.

// xf = xfP * xfC
MakeCompoundChildSweep :: proc(compoundTransform: Transform, childTransform: Transform) -> Sweep {
	xf := MulTransforms(compoundTransform, childTransform)
	return Sweep{localCenter = Vec3_zero, c1 = xf.p, c2 = xf.p, q1 = xf.q, q2 = xf.q}
}

CompoundMoverContext :: struct {
	compound:      ^CompoundData,
	planes:        [^]PlaneResult,
	planeCapacity: i32,
	planeCount:    i32,
	mover:         Capsule,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
CompoundMoverCallback :: proc "c" (proxyId: i32, userData: u64, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	_ = proxyId

	childIndex := i32(userData)
	moverContext := cast(^CompoundMoverContext)context_
	child := GetCompoundChild(moverContext.compound, childIndex)

	// Transform mover to child space
	localMover: Capsule
	localMover.center1 = InvTransformPoint(child.transform, moverContext.mover.center1)
	localMover.center2 = InvTransformPoint(child.transform, moverContext.mover.center2)
	localMover.radius = moverContext.mover.radius

	capacity := moverContext.planeCapacity - moverContext.planeCount
	assert(capacity > 0)

	planes := moverContext.planes[moverContext.planeCount:]
	planeCount: i32 = 0

	#partial switch child.type {
	case .capsuleShape:
		planeCount = CollideMoverAndCapsule(&planes[0], &child.capsule, &localMover)

	case .hullShape:
		planeCount = CollideMoverAndHull(&planes[0], child.hull, &localMover)

	case .meshShape:
		planeCount = CollideMoverAndMesh(planes, capacity, &child.mesh, &localMover)

	case .sphereShape:
		planeCount = CollideMoverAndSphere(&planes[0], &child.sphere, &localMover)

	case:
		assert(false)
	}

	// Transform results back to shape space
	for i: i32 = 0; i < planeCount; i += 1 {
		planes[i].plane.normal = RotateVector(child.transform.q, planes[i].plane.normal)
		planes[i].point = TransformPoint(child.transform, planes[i].point)

		planes[i].childIndex = childIndex
		assert(0 <= planes[i].materialIndex && planes[i].materialIndex < child.materialCount)
		planes[i].materialIndex = i32(child.materialIndices[planes[i].materialIndex])
	}

	moverContext.planeCount += planeCount

	// Continue query while there is room for more planes
	return moverContext.planeCount < moverContext.planeCapacity
}

CollideMoverAndCompound :: proc(planes: [^]PlaneResult, capacity: i32, shape: ^CompoundData, mover: ^Capsule) -> i32 {
	context_ := CompoundMoverContext {
		compound      = shape,
		planes        = planes,
		planeCapacity = capacity,
		planeCount    = 0,
		mover         = mover^,
	}

	aabb: AABB
	aabb.lowerBound = Min(mover.center1, mover.center2)
	aabb.upperBound = Max(mover.center1, mover.center2)
	r := Vec3{mover.radius, mover.radius, mover.radius}
	aabb.lowerBound = Sub(aabb.lowerBound, r)
	aabb.upperBound = Add(aabb.upperBound, r)

	view := GetCompoundTreeView(shape)
	_ = TreeView_Query(&view, aabb, max(u64), false, CompoundMoverCallback, &context_)

	return context_.planeCount
}
