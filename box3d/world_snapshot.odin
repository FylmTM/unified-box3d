// Ported from box3d src/world_snapshot.c and src/world_snapshot.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "core:fmt"
import "core:mem"

// world_snapshot.h

// Serialize the live world into buf, interning shape geometry into rec->registry.
// On success buf holds a self-contained snapshot image. Returns the byte count.
// (SerializeWorld)

// Overwrite a freshly-created (shell) world with the simulation state held in the
// snapshot image [data, size). Geometry references are resolved via the shared
// registry slots in rdr. Returns false on a corrupt or incompatible image.
// (DeserializeIntoShell)

// world_snapshot.c

// Snapshot image magic 'BNS3' and version
SNAP_MAGIC :: u32(0x33534E42)
SNAP_VERSION :: u32(11) // isotropic inertia tolerance

SNAP_FLAG_VALIDATION :: u32(0x1)
SNAP_FLAG_DOUBLE_PRECISION :: u32(0x2)

// Layout hash over all POD-copied structs + key constants.
// Changing a struct size updates this, catching ABI drift early.
@(private = "file")
ComputeLayoutHash :: proc() -> u32 {
	h: u32 = 2166136261
	// unified: the MIX( x ) macro is a local proc
	MIX :: #force_inline proc(h: ^u32, x: u32) {
		h^ ~= x
		h^ *= 16777619
	}
	// unified end
	MIX(&h, u32(size_of(Body)))
	MIX(&h, u32(size_of(BodySim)))
	MIX(&h, u32(size_of(BodyState)))
	MIX(&h, u32(size_of(Shape)))
	MIX(&h, u32(size_of(Contact)))
	MIX(&h, u32(size_of(Manifold)))
	MIX(&h, u32(size_of(Joint)))
	MIX(&h, u32(size_of(JointSim)))
	MIX(&h, u32(size_of(Island)))
	MIX(&h, u32(size_of(IslandSim)))
	MIX(&h, u32(size_of(ContactLink)))
	MIX(&h, u32(size_of(JointLink)))
	MIX(&h, u32(size_of(Sensor)))
	MIX(&h, u32(size_of(Visitor)))
	MIX(&h, u32(size_of(SolverSet)))
	MIX(&h, u32(size_of(GraphColor)))
	MIX(&h, u32(size_of(DynamicTree)))
	MIX(&h, u32(size_of(TreeNode)))
	MIX(&h, u32(size_of(TreeProxy)))
	MIX(&h, u32(size_of(HullData)))
	MIX(&h, u32(size_of(HullVertex)))
	MIX(&h, u32(size_of(HullHalfEdge)))
	MIX(&h, u32(size_of(HullFace)))
	MIX(&h, u32(size_of(Plane)))
	MIX(&h, u32(size_of(MeshData)))
	MIX(&h, u32(size_of(MeshNode)))
	MIX(&h, u32(size_of(MeshTriangle)))
	MIX(&h, u32(size_of(HeightFieldData)))
	MIX(&h, u32(size_of(CompoundData)))
	MIX(&h, u32(size_of(CompoundCapsule)))
	MIX(&h, u32(size_of(CompoundSphere)))
	MIX(&h, u32(HULL_VERSION & 0xFFFFFFFF))
	MIX(&h, u32(HULL_VERSION >> 32))
	MIX(&h, u32(MESH_VERSION & 0xFFFFFFFF))
	MIX(&h, u32(MESH_VERSION >> 32))
	MIX(&h, u32(HEIGHT_FIELD_VERSION & 0xFFFFFFFF))
	MIX(&h, u32(HEIGHT_FIELD_VERSION >> 32))
	MIX(&h, u32(COMPOUND_VERSION & 0xFFFFFFFF))
	MIX(&h, u32(COMPOUND_VERSION >> 32))
	MIX(&h, u32(size_of(SetItem)))
	MIX(&h, u32(size_of(IdPool)))
	MIX(&h, u32(size_of(SurfaceMaterial)))
	MIX(&h, u32(size_of(ContactSpec)))
	MIX(&h, u32(size_of(TriangleCache)))
	MIX(&h, u32(GRAPH_COLOR_COUNT))
	MIX(&h, u32(BodyType.bodyTypeCount))
	MIX(&h, u32(size_of(rawptr)))
	return h
}

@(private = "file")
SnapHeader :: struct {
	magic:      u32,
	version:    u32,
	layoutHash: u32,
	flags:      u32,
}

// Bounds-checked read cursor
@(private = "file")
SnapReader :: struct {
	data:   [^]u8,
	cursor: i32,
	size:   i32,
	ok:     bool,
}

@(private = "file")
SnapRCheck :: proc(r: ^SnapReader, need: i32) {
	if need < 0 || i64(r.cursor) + i64(need) > i64(r.size) {
		r.ok = false
	}
}

@(private = "file")
SnapR_Bytes :: proc(r: ^SnapReader, dst: rawptr, n: i32) {
	SnapRCheck(r, n)
	if !r.ok {
		return
	}
	mem.copy(dst, &r.data[r.cursor], int(n))
	r.cursor += n
}

@(private = "file")
SnapR_I32 :: proc(r: ^SnapReader) -> i32 {
	v: i32 = 0
	SnapR_Bytes(r, &v, 4)
	return v
}

@(private = "file")
SnapR_U32 :: proc(r: ^SnapReader) -> u32 {
	v: u32 = 0
	SnapR_Bytes(r, &v, 4)
	return v
}

@(private = "file")
SnapW_I32 :: proc(buf: ^RecBuffer, v: i32) {
	w := v
	RecBufAppend(buf, &w, 4)
}

@(private = "file")
SnapW_U32 :: proc(buf: ^RecBuffer, v: u32) {
	v := v
	RecBufAppend(buf, &v, 4)
}

@(private = "file")
SnapW_Bytes :: proc(buf: ^RecBuffer, src: rawptr, n: i32) {
	RecBufAppend(buf, src, n)
}

// Bounds check before allocating from image
@(private = "file")
SnapCheckCount :: proc(r: ^SnapReader, count: i32, memSize: i32, minStreamBytes: i32) -> bool {
	if count < 0 || memSize < 0 || minStreamBytes < 0 {
		return false
	}
	if memSize > 0 && count > 0x7FFFFFFF / memSize {
		return false
	}
	remaining := i64(r.size) - i64(r.cursor)
	return i64(count) * i64(minStreamBytes) <= remaining
}

// POD array: count + raw bytes
// unified: the b3SerPodArray and b3DesPodArray macros are generic procs
@(private = "file")
SerPodArray :: proc(buf: ^RecBuffer, arr: ^Array($T)) {
	SnapW_I32(buf, arr.count)
	if arr.count > 0 {
		// unified: canonical bytes: padding and host values zeroed in the copy (world_image.odin)
		SnapW_Canonical(buf, arr.data, T, arr.count)
		// unified end
	}
}

@(private = "file")
DesPodArray :: proc(r: ^SnapReader, arr: ^Array($T)) {
	cnt := SnapR_I32(r)
	elemSize := i32(size_of(T))
	if r.ok && SnapCheckCount(r, cnt, elemSize, elemSize) == false {
		r.ok = false
	}
	if r.ok && cnt > 0 {
		Array_Resize(arr, cnt)
		SnapR_Bytes(r, arr.data, cnt * elemSize)
	} else if r.ok {
		arr.count = 0
	}
}
// unified end

// Id pool: nextIndex + freeArray
@(private = "file")
SerIdPool :: proc(buf: ^RecBuffer, pool: ^IdPool) {
	SnapW_I32(buf, pool.nextIndex)
	SerPodArray(buf, &pool.freeArray)
}

@(private = "file")
DesIdPool :: proc(r: ^SnapReader, pool: ^IdPool) {
	pool.nextIndex = SnapR_I32(r)
	DesPodArray(r, &pool.freeArray)
}

// BitSet: blockCount + raw words
@(private = "file")
SerBitSet :: proc(buf: ^RecBuffer, bs: ^BitSet) {
	SnapW_U32(buf, bs.blockCount)
	if bs.blockCount > 0 {
		SnapW_Bytes(buf, bs.bits, i32(bs.blockCount * size_of(u64)))
	}
}

@(private = "file")
DesBitSet :: proc(r: ^SnapReader, bs: ^BitSet) {
	blockCount := SnapR_U32(r)
	if r.ok && SnapCheckCount(r, i32(blockCount), i32(size_of(u64)), i32(size_of(u64))) == false {
		r.ok = false
	}
	DestroyBitSet(bs)
	if !r.ok {
		return
	}
	blockCapacity := blockCount > 0 ? blockCount : 1
	bs.bits = cast([^]u64)Alloc(int(blockCapacity * size_of(u64)))
	mem.zero(bs.bits, int(blockCapacity * size_of(u64)))
	bs.blockCapacity = blockCapacity
	bs.blockCount = blockCount
	if blockCount > 0 {
		SnapR_Bytes(r, bs.bits, i32(blockCount * size_of(u64)))
	}
}

// HashSet: capacity + count + raw items (probe order depends on layout)
@(private = "file")
SerHashSet :: proc(buf: ^RecBuffer, hs: ^HashSet) {
	SnapW_U32(buf, hs.capacity)
	SnapW_U32(buf, hs.count)
	if hs.capacity > 0 {
		// unified: canonical bytes (world_image.odin)
		SnapW_Canonical(buf, hs.items, SetItem, i32(hs.capacity))
		// unified end
	}
}

@(private = "file")
DesHashSet :: proc(r: ^SnapReader, hs: ^HashSet) {
	cap := SnapR_U32(r)
	cnt := SnapR_U32(r)
	valid :=
		SnapCheckCount(r, i32(cap), i32(size_of(SetItem)), i32(size_of(SetItem))) &&
		(cap & (cap - 1)) == 0 &&
		cnt <= cap
	if r.ok && valid == false && (cap != 0 || cnt != 0) {
		r.ok = false
	}
	DestroySet(hs)
	if !r.ok {
		return
	}
	if cap > 0 {
		hs.items = cast([^]SetItem)Alloc(int(cap * size_of(SetItem)))
		hs.capacity = cap
		hs.count = cnt
		SnapR_Bytes(r, hs.items, i32(cap * size_of(SetItem)))
	} else {
		hs.items = nil
		hs.capacity = 0
		hs.count = 0
	}
}

// DynamicTree: version, scalars, then nodes and parents to nodeEnd and proxies at capacity.
// The empty node and the free pairs are sentinels below the end and travel as they are. Only the
// stale tail above the end, the spare array and the rebuild scratch are skipped.
@(private = "file")
SerTree :: proc(buf: ^RecBuffer, tree: ^DynamicTree) {
	SnapW_Bytes(buf, &tree.version, size_of(u64))
	SnapW_I32(buf, tree.nodeEnd)
	SnapW_I32(buf, tree.pairFreeList)
	SnapW_I32(buf, tree.proxyCount)
	SnapW_I32(buf, tree.proxyCapacity)
	SnapW_I32(buf, tree.proxyFreeList)
	SnapW_I32(buf, tree.dfsOrdered ? 1 : 0)
	if tree.nodeEnd > 0 {
		// unified: canonical bytes (world_image.odin)
		SnapW_Canonical(buf, tree.nodes, TreeNode, tree.nodeEnd)
		// unified end
		SnapW_Bytes(buf, tree.parents, tree.nodeEnd * i32(size_of(i32)))
	}
	if tree.proxyCapacity > 0 {
		// unified: canonical bytes (world_image.odin)
		SnapW_Canonical(buf, tree.proxies, TreeProxy, tree.proxyCapacity)
		// unified end
	}
}

@(private = "file")
DesTree :: proc(r: ^SnapReader, tree: ^DynamicTree) {
	version: u64 = 0
	SnapR_Bytes(r, &version, size_of(u64))
	nodeEnd := SnapR_I32(r)
	pairFreeList := SnapR_I32(r)
	proxyCount := SnapR_I32(r)
	proxyCapacity := SnapR_I32(r)
	proxyFreeList := SnapR_I32(r)
	dfsOrdered := SnapR_I32(r) != 0

	if r.ok && SnapCheckCount(r, nodeEnd, i32(size_of(TreeNode)), i32(size_of(TreeNode))) == false {
		r.ok = false
	}

	if r.ok && SnapCheckCount(r, proxyCapacity, i32(size_of(TreeProxy)), i32(size_of(TreeProxy))) == false {
		r.ok = false
	}

	if r.ok && (nodeEnd < 2 || (nodeEnd & 1) != 0 || proxyCapacity < 1) {
		r.ok = false
	}

	// Free existing allocation including the spare array and any rebuild scratch
	Free(tree.nodes, int(tree.nodeCapacity * i32(size_of(TreeNode))))
	Free(tree.parents, int(tree.nodeCapacity * i32(size_of(i32))))
	Free(tree.proxies, int(tree.proxyCapacity * i32(size_of(TreeProxy))))
	Free(tree.swapNodes, int(tree.nodeCapacity * i32(size_of(TreeNode))))
	Free(tree.leafIndices, int(tree.rebuildCapacity * i32(size_of(i32))))
	Free(tree.leafNodes, int(tree.rebuildCapacity * i32(size_of(TreeNode))))
	Free(tree.leafBoxes, int(tree.rebuildCapacity * i32(size_of(AABB))))
	Free(tree.leafCenters, int(tree.rebuildCapacity * i32(size_of(Vec3))))
	Free(tree.binIndices, int(tree.rebuildCapacity * i32(size_of(i32))))
	tree.nodes = nil
	tree.parents = nil
	tree.proxies = nil
	tree.swapNodes = nil
	tree.leafIndices = nil
	tree.leafNodes = nil
	tree.leafBoxes = nil
	tree.leafCenters = nil
	tree.binIndices = nil
	tree.nodeEnd = 0
	tree.nodeCapacity = 0
	tree.proxyCapacity = 0
	tree.rebuildCapacity = 0

	if !r.ok {
		return
	}

	tree.version = version
	tree.nodeEnd = nodeEnd
	tree.nodeCapacity = nodeEnd
	tree.pairFreeList = pairFreeList
	tree.proxyCount = proxyCount
	tree.proxyCapacity = proxyCapacity
	tree.proxyFreeList = proxyFreeList
	tree.dfsOrdered = dfsOrdered

	tree.nodes = cast([^]TreeNode)Alloc(int(nodeEnd * i32(size_of(TreeNode))))
	tree.parents = cast([^]i32)Alloc(int(nodeEnd * i32(size_of(i32))))
	SnapR_Bytes(r, tree.nodes, nodeEnd * i32(size_of(TreeNode)))
	SnapR_Bytes(r, tree.parents, nodeEnd * i32(size_of(i32)))

	tree.proxies = cast([^]TreeProxy)Alloc(int(proxyCapacity * i32(size_of(TreeProxy))))
	SnapR_Bytes(r, tree.proxies, proxyCapacity * i32(size_of(TreeProxy)))
}

// Solver set: setIndex + 4 arrays (note: contactIndices is int array, not contactSims)
@(private = "file")
SerSolverSet :: proc(buf: ^RecBuffer, set: ^SolverSet) {
	SnapW_I32(buf, set.setIndex)
	SerPodArray(buf, &set.bodySims)
	SerPodArray(buf, &set.bodyStates)
	SerPodArray(buf, &set.jointSims)
	SerPodArray(buf, &set.contactIndices)
	SerPodArray(buf, &set.islandSims)
}

@(private = "file")
DesSolverSet :: proc(r: ^SnapReader, set: ^SolverSet) {
	set.setIndex = SnapR_I32(r)
	DesPodArray(r, &set.bodySims)
	DesPodArray(r, &set.bodyStates)
	DesPodArray(r, &set.jointSims)
	DesPodArray(r, &set.contactIndices)
	DesPodArray(r, &set.islandSims)
}

@(private = "file")
SerNames :: proc(buf: ^RecBuffer, cache: ^NameCache) {
	SnapW_I32(buf, cache.entries.count)
	count := cache.entries.count
	for i: i32 = 0; i < count; i += 1 {
		entry := &cache.entries.data[i]
		SnapW_U32(buf, entry.hash)
		SnapW_I32(buf, entry.length)
		RecBufAppend(buf, rawptr(entry.name), entry.length)
	}
}

@(private = "file")
DesNames :: proc(r: ^SnapReader, cache: ^NameCache) {
	count := SnapR_I32(r)

	if r.ok && SnapCheckCount(r, count, i32(size_of(NameEntry)), 8) == false {
		r.ok = false
	}

	if r.ok == false {
		return
	}

	Array_Reserve(&cache.entries, count)
	for i: i32 = 0; i < count; i += 1 {
		hash := SnapR_U32(r)
		length := SnapR_I32(r)
		if r.ok == false || length < 0 || length > r.size - r.cursor {
			r.ok = false
			return
		}
		name := cast([^]u8)Alloc(int(length + 1))
		SnapR_Bytes(r, name, length)
		name[length] = 0
		LoadName(cache, hash, cstring(name), length)
	}
}

// Graph color: bodySet (non-overflow only) + jointSims + convexContacts + contacts
@(private = "file")
SerGraphColor :: proc(buf: ^RecBuffer, color: ^GraphColor, isOverflow: bool) {
	if !isOverflow {
		SerBitSet(buf, &color.bodySet)
	}
	SerPodArray(buf, &color.jointSims)
	SerPodArray(buf, &color.convexContacts)
	SerPodArray(buf, &color.contacts)
	// wideConstraints / manifoldConstraints / contactConstraints are transient, not serialized
}

@(private = "file")
DesGraphColor :: proc(r: ^SnapReader, color: ^GraphColor, isOverflow: bool) {
	if !isOverflow {
		DesBitSet(r, &color.bodySet)
	}
	DesPodArray(r, &color.jointSims)
	DesPodArray(r, &color.convexContacts)
	DesPodArray(r, &color.contacts)
	// Transient pointers left at NULL/0 from shell
}

// World simulation scalars (never host/callback/worker state)
@(private = "file")
SerWorldConfig :: proc(buf: ^RecBuffer, world: ^World) {
	SnapW_Bytes(buf, &world.gravity, size_of(Vec3))
	SnapW_Bytes(buf, &world.hitEventThreshold, size_of(f32))
	SnapW_Bytes(buf, &world.restitutionThreshold, size_of(f32))
	SnapW_I32(buf, world.restitutionIterations)
	SnapW_Bytes(buf, &world.maxLinearSpeed, size_of(f32))
	SnapW_Bytes(buf, &world.isotropicInertiaTolerance, size_of(f32))
	SnapW_Bytes(buf, &world.contactSpeed, size_of(f32))
	SnapW_Bytes(buf, &world.contactHertz, size_of(f32))
	SnapW_Bytes(buf, &world.contactDampingRatio, size_of(f32))
	SnapW_Bytes(buf, &world.contactRecycleDistance, size_of(f32))
	SnapW_Bytes(buf, &world.stepIndex, size_of(u64))
	SnapW_I32(buf, world.splitIslandId)
	SnapW_Bytes(buf, &world.inv_h, size_of(f32))
	SnapW_Bytes(buf, &world.inv_dt, size_of(f32))
	SnapW_I32(buf, world.endEventArrayIndex)
	SnapW_I32(buf, world.compoundShapeCount)
	SnapW_Bytes(buf, &world.maxCapacity, size_of(Capacity))
	flags: u8 = 0
	flags |= world.enableSleep ? 0x01 : 0
	flags |= world.enableWarmStarting ? 0x02 : 0
	flags |= world.enableContinuous ? 0x04 : 0
	flags |= world.enableSpeculative ? 0x08 : 0
	flags |= world.enableRestitutionPropagation ? 0x10 : 0
	RecBufAppend(buf, &flags, 1)
}

@(private = "file")
DesWorldConfig :: proc(r: ^SnapReader, world: ^World) {
	SnapR_Bytes(r, &world.gravity, size_of(Vec3))
	SnapR_Bytes(r, &world.hitEventThreshold, size_of(f32))
	SnapR_Bytes(r, &world.restitutionThreshold, size_of(f32))
	world.restitutionIterations = SnapR_I32(r)
	SnapR_Bytes(r, &world.maxLinearSpeed, size_of(f32))
	SnapR_Bytes(r, &world.isotropicInertiaTolerance, size_of(f32))
	SnapR_Bytes(r, &world.contactSpeed, size_of(f32))
	SnapR_Bytes(r, &world.contactHertz, size_of(f32))
	SnapR_Bytes(r, &world.contactDampingRatio, size_of(f32))
	SnapR_Bytes(r, &world.contactRecycleDistance, size_of(f32))
	SnapR_Bytes(r, &world.stepIndex, size_of(u64))
	world.splitIslandId = SnapR_I32(r)
	SnapR_Bytes(r, &world.inv_h, size_of(f32))
	SnapR_Bytes(r, &world.inv_dt, size_of(f32))
	world.endEventArrayIndex = SnapR_I32(r)
	world.compoundShapeCount = SnapR_I32(r)
	SnapR_Bytes(r, &world.maxCapacity, size_of(Capacity))
	flags: u8 = 0
	SnapR_Bytes(r, &flags, 1)
	world.enableSleep = (flags & 0x01) != 0
	world.enableWarmStarting = (flags & 0x02) != 0
	world.enableContinuous = (flags & 0x04) != 0
	world.enableSpeculative = (flags & 0x08) != 0
	world.enableRestitutionPropagation = (flags & 0x10) != 0
}

// Shapes carry pointer fields: materials, userData, userShape, and the geometry union.
// Serialize the POD scalars with pointers nulled, then the owned materials array, then geometry.
// A single material lives inline in the struct image.
// Hull/mesh/heightField/compound are interned into the recording registry; sphere/capsule inline.
// unified: img != nil: geometry is named by the image's own table (world_image.odin), not the recording registry
@(private = "file")
SerShapes :: proc(buf: ^RecBuffer, world: ^World, rec: ^Recording, img: ^WorldImageData) {
	// unified end
	count := world.shapes.count
	SnapW_I32(buf, count)

	for i: i32 = 0; i < count; i += 1 {
		shape := world.shapes.data[i]
		isLive := (shape.id == i)

		// Null out pointer fields before writing the raw struct
		shape.materials = nil
		shape.userData = nil
		shape.userShape = nil
		// Zero the geometry union so free-slot images have deterministic bytes
		if !isLive {
			mem.zero(&shape.capsule, size_of(shape.capsule))
		}
		// unified: canonical bytes: padding and the whole geometry union zeroed (world_image.odin)
		SnapW_Canonical(buf, &shape, Shape, 1)
		// unified end

		if !isLive {
			// Free slot: no materials or geometry
			SnapW_I32(buf, 0) // materialCount
			SnapW_I32(buf, -1) // geometry kind sentinel
			continue
		}

		// Owned material array. Only multi material meshes and compounds have one. A single material
		// already rode along inline in the struct image, so write a zero length for it.
		src := &world.shapes.data[i]
		if src.materials != nil {
			SnapW_I32(buf, src.materialCount)
			// unified: canonical bytes (world_image.odin)
			SnapW_Canonical(buf, src.materials, SurfaceMaterial, src.materialCount)
			// unified end
		} else {
			SnapW_I32(buf, 0)
		}

		// Geometry
		// unified: the geometry id is the image table's index (world_image.odin) when img != nil
		#partial switch src.type {
		case .sphereShape:
			SnapW_I32(buf, i32(ShapeType.sphereShape))
			SnapW_Bytes(buf, &src.sphere, size_of(Sphere))
		case .capsuleShape:
			SnapW_I32(buf, i32(ShapeType.capsuleShape))
			SnapW_Bytes(buf, &src.capsule, size_of(Capsule))
		case .hullShape:
			SnapW_I32(buf, i32(ShapeType.hullShape))
			gid := img != nil ? u32(img.shapeGeometry[i]) : RecInternHull(rec, src.hull)
			SnapW_U32(buf, gid)
		case .meshShape:
			SnapW_I32(buf, i32(ShapeType.meshShape))
			gid := img != nil ? u32(img.shapeGeometry[i]) : RecInternMesh(rec, src.mesh.data)
			SnapW_U32(buf, gid)
			SnapW_Bytes(buf, &src.mesh.scale, size_of(Vec3))
		case .heightShape:
			SnapW_I32(buf, i32(ShapeType.heightShape))
			gid := img != nil ? u32(img.shapeGeometry[i]) : RecInternHeightField(rec, src.heightField)
			SnapW_U32(buf, gid)
		case .compoundShape:
			SnapW_I32(buf, i32(ShapeType.compoundShape))
			gid := img != nil ? u32(img.shapeGeometry[i]) : RecInternCompound(rec, src.compound)
			SnapW_U32(buf, gid)
		// unified end
		case:
			// A live shape must have a known geometry type. Fail loudly rather than emit a shape
			// with no geometry that would silently lose its collision on restore.
			assert(false)
			SnapW_I32(buf, -1)
		}
	}
}

@(private = "file")
DesShapes :: proc(r: ^SnapReader, world: ^World, rdr: ^RecReader) {
	count := SnapR_I32(r)
	if r.ok && SnapCheckCount(r, count, i32(size_of(Shape)), i32(size_of(Shape))) == false {
		r.ok = false
	}
	if !r.ok {
		return
	}

	// Save renderer handles before the array is wiped. A keyframe restore is a deterministic replay
	// state, so a live shape that still occupies the same slot with the same generation is the same
	// shape with the same geometry. Carrying its handle over avoids tearing down and rebuilding every
	// GPU mesh on each seek, which the host (a 3D renderer) would otherwise pay for. Handles that are
	// not reclaimed below belong to shapes that are gone or were replaced, and get released so the host
	// pool does not leak across seeks.
	oldShapeCount := world.shapes.count
	savedUserShape: [^]rawptr = nil
	savedGeneration: [^]u16 = nil
	if oldShapeCount > 0 {
		savedUserShape = cast([^]rawptr)Alloc(int(oldShapeCount) * size_of(rawptr))
		savedGeneration = cast([^]u16)Alloc(int(oldShapeCount) * size_of(u16))
		for i: i32 = 0; i < oldShapeCount; i += 1 {
			old := &world.shapes.data[i]
			oldLive := (old.id == i)
			savedUserShape[i] = oldLive ? old.userShape : nil
			savedGeneration[i] = old.generation
		}
	}

	Array_Resize(&world.shapes, count)
	mem.zero(world.shapes.data, int(count) * size_of(Shape))

	for i: i32 = 0; i < count && r.ok; i += 1 {
		dst := &world.shapes.data[i]
		SnapR_Bytes(r, dst, size_of(Shape))
		// Pointer fields were written as NULL; set them cleanly
		dst.materials = nil
		dst.userData = nil
		dst.userShape = nil
		mem.zero(&dst.capsule, size_of(dst.capsule))

		isLive := (dst.id == i)

		// Carry the renderer handle over when the same shape still occupies this slot. Consumed
		// handles are nulled so the teardown sweep only releases the ones that vanished.
		if isLive &&
		   i < oldShapeCount &&
		   savedUserShape != nil &&
		   savedUserShape[i] != nil &&
		   savedGeneration[i] == dst.generation {
			dst.userShape = savedUserShape[i]
			savedUserShape[i] = nil
		}

		// Serializer writes: matCount, matData, geoKind, geoData
		matCount := SnapR_I32(r)

		if !r.ok {
			break
		}

		if !isLive {
			// Free slot: matCount=0, geoKind=-1
			_ = matCount
			SnapR_I32(r) // consume the geoKind sentinel
			continue
		}

		// Owned material array (written before geoKind in serializer). A zero length means the single
		// material is already inline in the restored struct image, so leave materialCount as restored.
		if matCount > 0 {
			if SnapCheckCount(r, matCount, i32(size_of(SurfaceMaterial)), i32(size_of(SurfaceMaterial))) == false {
				r.ok = false
				break
			}
			dst.materialCount = matCount
			dst.materials = cast([^]SurfaceMaterial)Alloc(int(matCount) * size_of(SurfaceMaterial))
			SnapR_Bytes(r, dst.materials, matCount * i32(size_of(SurfaceMaterial)))
		} else {
			dst.materials = nil
		}

		geoKind := SnapR_I32(r)

		// Geometry
		#partial switch ShapeType(geoKind) {
		case .sphereShape:
			SnapR_Bytes(r, &dst.sphere, size_of(Sphere))
		case .capsuleShape:
			SnapR_Bytes(r, &dst.capsule, size_of(Capsule))
		case .hullShape:
			gid := SnapR_U32(r)
			if !r.ok {
				break
			}
			slot := RecGetSlot(rdr, gid, .geometryHull)
			if slot == nil {
				r.ok = false
				break
			}
			// Hull is cloned into the world DB; pass raw bytes directly
			dst.hull = AddHullToDatabase(world, (^HullData)(slot.bytes))
		case .meshShape:
			gid := SnapR_U32(r)
			scale: Vec3
			SnapR_Bytes(r, &scale, size_of(Vec3))
			if !r.ok {
				break
			}
			slot := RecGetSlot(rdr, gid, .geometryMesh)
			if slot == nil {
				r.ok = false
				break
			}
			// Mesh is a self-contained blob used by reference; point straight at the pristine bytes.
			dst.mesh.data = (^MeshData)(slot.bytes)
			dst.mesh.scale = scale
		case .heightShape:
			gid := SnapR_U32(r)
			if !r.ok {
				break
			}
			slot := RecGetSlot(rdr, gid, .geometryHeightField)
			if slot == nil {
				r.ok = false
				break
			}
			// Self-contained blob used by reference; point straight at the pristine bytes.
			dst.heightField = (^HeightFieldData)(slot.bytes)
		case .compoundShape:
			gid := SnapR_U32(r)
			if !r.ok {
				break
			}
			slot := RecGetSlot(rdr, gid, .geometryCompound)
			if slot == nil {
				r.ok = false
				break
			}
			dst.compound = (^CompoundData)(slot.bytes)
		case:
			// Unknown geometry kind means a corrupt or unsupported snapshot. Fail the load instead
			// of leaving a shape with no geometry.
			r.ok = false
		}
	}

	// Release handles for shapes that are gone or were replaced this restore, so the host pool and any
	// GPU resources they pinned do not leak across seeks.
	if savedUserShape != nil {
		for i: i32 = 0; i < oldShapeCount; i += 1 {
			if savedUserShape[i] != nil && world.destroyDebugShape != nil {
				world.destroyDebugShape(savedUserShape[i], world.userDebugShapeContext)
			}
		}
		Free(savedUserShape, int(oldShapeCount) * size_of(rawptr))
		Free(savedGeneration, int(oldShapeCount) * size_of(u16))
	}
}

// Contact serialization. b3Contact is not fully POD:
// - manifolds: heap array of b3Manifold, allocated via b3AllocateManifolds
// - meshContact.triangleCache: heap b3Array, active when b3_simMeshContact flag is set
// Serialize: raw struct (with nulled manifolds/triangleCache), then manifolds, then triangleCache.
@(private = "file")
SerContacts :: proc(buf: ^RecBuffer, world: ^World) {
	count := world.contacts.count
	SnapW_I32(buf, count)

	for i: i32 = 0; i < count; i += 1 {
		c := &world.contacts.data[i]
		isLive := (c.contactId == i)

		// Write raw struct with pointer fields zeroed
		copy := c^
		copy.manifolds = nil
		if copy.flags & simMeshContact != 0 {
			copy.meshContact.triangleCache.data = nil
			copy.meshContact.triangleCache.count = 0
			copy.meshContact.triangleCache.capacity = 0
		}
		// unified: canonical bytes: padding and the inactive union bytes zeroed (world_image.odin)
		SnapW_Canonical(buf, &copy, Contact, 1)
		// unified end

		if !isLive {
			// Free slot: no heap data
			SnapW_I32(buf, 0) // manifoldCount
			// No triangleCache
			continue
		}

		// Manifolds
		SnapW_I32(buf, c.manifoldCount)
		if c.manifoldCount > 0 && c.manifolds != nil {
			// unified: canonical bytes (world_image.odin)
			SnapW_Canonical(buf, c.manifolds, Manifold, c.manifoldCount)
			// unified end
		}

		// Mesh triangleCache
		if c.flags & simMeshContact != 0 {
			SnapW_I32(buf, c.meshContact.triangleCache.count)
			if c.meshContact.triangleCache.count > 0 {
				// unified: canonical bytes (world_image.odin)
				SnapW_Canonical(
					buf,
					c.meshContact.triangleCache.data,
					TriangleCache,
					c.meshContact.triangleCache.count,
				)
				// unified end
			}
		}
	}
}

@(private = "file")
DesContacts :: proc(r: ^SnapReader, world: ^World) {
	count := SnapR_I32(r)
	if r.ok && SnapCheckCount(r, count, i32(size_of(Contact)), i32(size_of(Contact))) == false {
		r.ok = false
	}
	if !r.ok {
		return
	}

	Array_Resize(&world.contacts, count)
	mem.zero(world.contacts.data, int(count) * size_of(Contact))

	for i: i32 = 0; i < count && r.ok; i += 1 {
		dst := &world.contacts.data[i]
		SnapR_Bytes(r, dst, size_of(Contact))
		dst.manifolds = nil
		if dst.flags & simMeshContact != 0 {
			dst.meshContact.triangleCache.data = nil
			dst.meshContact.triangleCache.count = 0
			dst.meshContact.triangleCache.capacity = 0
		}

		isLive := (dst.contactId == i)

		manifoldCount := SnapR_I32(r)

		if !r.ok {
			break
		}

		if isLive && manifoldCount > 0 {
			if SnapCheckCount(r, manifoldCount, i32(size_of(Manifold)), i32(size_of(Manifold))) == false {
				r.ok = false
				break
			}
			dst.manifolds = AllocateManifolds(world, manifoldCount)
			dst.manifoldCount = manifoldCount
			SnapR_Bytes(r, dst.manifolds, manifoldCount * i32(size_of(Manifold)))
		} else {
			dst.manifolds = nil
			dst.manifoldCount = 0
		}

		// Mesh triangleCache
		if isLive && (dst.flags & simMeshContact != 0) {
			cacheCount := SnapR_I32(r)
			if !r.ok {
				break
			}
			if cacheCount > 0 {
				if SnapCheckCount(r, cacheCount, i32(size_of(TriangleCache)), i32(size_of(TriangleCache))) == false {
					r.ok = false
					break
				}
				Array_Resize(&dst.meshContact.triangleCache, cacheCount)
				SnapR_Bytes(r, dst.meshContact.triangleCache.data, cacheCount * i32(size_of(TriangleCache)))
			}
		}
	}
}

// unified: an image's geometry table (world_image.odin), resolved against the world's geometry store (blobs the
// image carries are checked and added to it) into reader slots, so DesShapes finds geometry as it does in a recording.
@(private = "file")
DesGeometryTable :: proc(r: ^SnapReader, img: ^WorldImageData, withBytes: bool) -> bool {
	count := SnapR_I32(r)
	headerSize := i32(size_of(ImageGeometryHeader))
	if !r.ok || SnapCheckCount(r, count, i32(size_of(RegistrySlot)), headerSize) == false {
		return false
	}
	resize(&img.slots, int(count))
	for i: i32 = 0; i < count; i += 1 {
		h: ImageGeometryHeader
		SnapR_Bytes(r, &h, headerSize)
		if !r.ok || h.kind < 0 || h.kind > i32(max(GeometryKind)) || h.byteCount <= 0 {
			return false
		}
		kind := GeometryKind(h.kind)
		bytes: [^]u8
		if withBytes {
			SnapRCheck(r, h.byteCount)
			if !r.ok {
				return false
			}
			bytes = ImageStoreLoad(img, kind, h.contentHash, &r.data[r.cursor], h.byteCount)
			r.cursor += h.byteCount
		} else {
			bytes = ImageStoreFind(img, kind, h.contentHash, h.byteCount)
		}
		if bytes == nil {
			return false
		}
		img.slots[i] = RegistrySlot{kind, h.byteCount, bytes}
	}
	img.reader.slots = raw_data(img.slots)
	img.reader.slotCount = count
	return true
}
// unified end

// Free per-object heap that b3DeserializeIntoShell will overwrite,
// so restoring over a populated world doesn't leak.
@(private = "file")
FreeLiveSimElements :: proc(world: ^World) {
	// Shape heap: materials and hull DB references
	for i: i32 = 0; i < world.shapes.count; i += 1 {
		s := &world.shapes.data[i]
		if s.id != i {
			continue
		}
		// A single material lives inline (materials == NULL). Multi material meshes and compounds own
		// the array, so free it exactly as b3DestroyShapeAllocations does.
		if s.materials != nil {
			Free(s.materials, int(s.materialCount) * size_of(SurfaceMaterial))
			s.materials = nil
			s.materialCount = 0
		}
		// Hull is ref-counted in the world DB; release before overwrite so re-adding is ref-neutral.
		if s.type == .hullShape && s.hull != nil {
			RemoveHullFromDatabase(world, s.hull)
			s.hull = nil
		}
		// name / userData / userShape are host-owned; do not free
	}

	// Contact heap: manifolds + mesh triangleCache
	for i: i32 = 0; i < world.contacts.count; i += 1 {
		c := &world.contacts.data[i]
		if c.contactId == i {
			if c.manifolds != nil {
				FreeManifolds(world, c.manifolds, c.manifoldCount)
				c.manifolds = nil
				c.manifoldCount = 0
			}
			if c.flags & simMeshContact != 0 {
				Array_Destroy(&c.meshContact.triangleCache)
			}
		}
	}

	// Sensor heap: inner arrays
	for i: i32 = 0; i < world.sensors.count; i += 1 {
		sensor := &world.sensors.data[i]
		Array_Destroy(&sensor.hits)
		Array_Destroy(&sensor.overlaps1)
		Array_Destroy(&sensor.overlaps2)
	}

	// Island heap: inner arrays
	for i: i32 = 0; i < world.islands.count; i += 1 {
		island := &world.islands.data[i]
		Array_Destroy(&island.bodies)
		Array_Destroy(&island.contacts)
		Array_Destroy(&island.joints)
	}
}

// unified: img != nil writes a canonical image with its own geometry table (world_image.odin) instead of interning
// geometry into rec's registry
SerializeWorld :: proc(world: ^World, buf: ^RecBuffer, rec: ^Recording, img: ^WorldImageData = nil) -> i32 {
	// unified end
	startSize := buf.size

	// Snapshot header
	hdr: SnapHeader
	hdr.magic = SNAP_MAGIC
	hdr.version = SNAP_VERSION
	hdr.layoutHash = ComputeLayoutHash()
	hdr.flags = ENABLE_VALIDATION ? SNAP_FLAG_VALIDATION : 0
	// unified: single precision only: the BOX3D_DOUBLE_PRECISION flag is never set.
	// unified: the geometry table follows the header (world_image.odin)
	if img != nil {
		ImageBuildTable(img, world)
		hdr.flags |= SNAP_FLAG_GEOMETRY_TABLE | (img.embed ? SNAP_FLAG_GEOMETRY_BYTES : 0)
	}
	SnapW_Bytes(buf, &hdr, i32(size_of(hdr)))
	if img != nil {
		ImageWriteTable(img, buf)
	}
	// unified end

	// World scalars
	SerWorldConfig(buf, world)

	// 6 id pools (Box3D has no chainIdPool)
	SerIdPool(buf, &world.bodyIdPool)
	SerIdPool(buf, &world.shapeIdPool)
	SerIdPool(buf, &world.contactIdPool)
	SerIdPool(buf, &world.jointIdPool)
	SerIdPool(buf, &world.islandIdPool)
	SerIdPool(buf, &world.solverSetIdPool)

	// Solver sets
	setCount := world.solverSets.count
	SnapW_I32(buf, setCount)
	for i: i32 = 0; i < setCount; i += 1 {
		SerSolverSet(buf, &world.solverSets.data[i])
	}

	// Sparse body array (userData is host wiring, zero it on the copy)
	{
		bodyCount := world.bodies.count
		SnapW_I32(buf, bodyCount)
		for i: i32 = 0; i < bodyCount; i += 1 {
			elem := world.bodies.data[i]
			elem.userData = nil
			// unified: canonical bytes (world_image.odin)
			SnapW_Canonical(buf, &elem, Body, 1)
			// unified end
		}
	}

	// Shape sparse array with geometry interning
	SerShapes(buf, world, rec, img) // unified: img

	SerPodArray(buf, &world.fatAABBs)

	// Contact sparse array with manifold and mesh triangleCache
	SerContacts(buf, world)

	// Joint sparse array (userData scrubbed)
	{
		jointCount := world.joints.count
		SnapW_I32(buf, jointCount)
		for i: i32 = 0; i < jointCount; i += 1 {
			elem := world.joints.data[i]
			elem.userData = nil
			// unified: canonical bytes (world_image.odin)
			SnapW_Canonical(buf, &elem, Joint, 1)
			// unified end
		}
	}

	// Sensors: shapeId + 3 inner arrays each
	{
		sensorCount := world.sensors.count
		SnapW_I32(buf, sensorCount)
		for i: i32 = 0; i < sensorCount; i += 1 {
			s := &world.sensors.data[i]
			SnapW_I32(buf, s.shapeId)
			SerPodArray(buf, &s.hits)
			SerPodArray(buf, &s.overlaps1)
			SerPodArray(buf, &s.overlaps2)
		}
	}

	// Islands: 4 scalars + 3 inner arrays each
	{
		islandCount := world.islands.count
		SnapW_I32(buf, islandCount)
		for i: i32 = 0; i < islandCount; i += 1 {
			island := &world.islands.data[i]
			SnapW_I32(buf, island.setIndex)
			SnapW_I32(buf, island.localIndex)
			SnapW_I32(buf, island.islandId)
			SnapW_I32(buf, island.constraintRemoveCount)
			SerPodArray(buf, &island.bodies)
			SerPodArray(buf, &island.contacts)
			SerPodArray(buf, &island.joints)
		}
	}

	// Broad phase
	bp := &world.broadPhase
	for t: i32 = 0; t < i32(BodyType.bodyTypeCount); t += 1 {
		SerTree(buf, &bp.trees[t])
	}
	SerHashSet(buf, &bp.pairSet)

	// Constraint graph
	graph := &world.constraintGraph
	for c: i32 = 0; c < GRAPH_COLOR_COUNT; c += 1 {
		SerGraphColor(buf, &graph.colors[c], c == OVERFLOW_INDEX)
	}

	SerNames(buf, &world.names)

	return buf.size - startSize
}

DeserializeIntoShell :: proc(data: [^]u8, size: i32, world: ^World, rdr: ^RecReader) -> bool {
	if data == nil || size < i32(size_of(SnapHeader)) {
		return false
	}

	// Validate header
	hdr: SnapHeader
	mem.copy(&hdr, data, size_of(hdr))
	if hdr.magic != SNAP_MAGIC || hdr.version != SNAP_VERSION {
		fmt.printf("b3DeserializeIntoShell: bad magic/version\n")
		return false
	}
	imageDouble := (hdr.flags & SNAP_FLAG_DOUBLE_PRECISION) != 0
	// unified: single precision only: the BOX3D_DOUBLE_PRECISION branch is not ported.
	buildDouble := false
	if imageDouble != buildDouble {
		fmt.printf("b3DeserializeIntoShell: precision mismatch\n")
		return false
	}
	if hdr.layoutHash != ComputeLayoutHash() {
		fmt.printf("b3DeserializeIntoShell: layout hash mismatch\n")
		return false
	}

	readerStorage: SnapReader
	r := &readerStorage
	r.data = data
	r.cursor = i32(size_of(SnapHeader))
	r.size = size
	r.ok = true

	// unified: an image with its own geometry table is resolved against the world's geometry store first, so one
	// naming geometry this world lacks is refused before the world is touched (world_image.odin)
	rdr := rdr
	if hdr.flags & SNAP_FLAG_GEOMETRY_TABLE != 0 {
		img := GetWorldImageData(world)
		if !DesGeometryTable(r, img, (hdr.flags & SNAP_FLAG_GEOMETRY_BYTES) != 0) {
			return false
		}
		rdr = &img.reader
	}
	// unified end

	// Free existing per-object heap before overwriting
	FreeLiveSimElements(world)

	// 1. World scalars
	DesWorldConfig(r, world)

	// 2. 6 id pools; destroy the pre-created sets' pool state first
	DesIdPool(r, &world.bodyIdPool)
	DesIdPool(r, &world.shapeIdPool)
	DesIdPool(r, &world.contactIdPool)
	DesIdPool(r, &world.jointIdPool)
	DesIdPool(r, &world.islandIdPool)
	DesIdPool(r, &world.solverSetIdPool)

	// 3. Solver sets: destroy inner arrays of existing sets first
	for i: i32 = 0; i < world.solverSets.count; i += 1 {
		set := &world.solverSets.data[i]
		Array_Destroy(&set.bodySims)
		Array_Destroy(&set.bodyStates)
		Array_Destroy(&set.jointSims)
		Array_Destroy(&set.contactIndices)
		Array_Destroy(&set.islandSims)
	}

	setCount := SnapR_I32(r)
	if r.ok && SnapCheckCount(r, setCount, i32(size_of(SolverSet)), 6 * i32(size_of(i32))) == false {
		r.ok = false
	}
	if r.ok {
		Array_Resize(&world.solverSets, setCount)
		mem.zero(world.solverSets.data, int(setCount) * size_of(SolverSet))
		for i: i32 = 0; i < setCount; i += 1 {
			DesSolverSet(r, &world.solverSets.data[i])
		}
	}

	if !r.ok {
		return false
	}

	// 4. Body sparse array
	{
		bodyCount := SnapR_I32(r)
		if r.ok && SnapCheckCount(r, bodyCount, i32(size_of(Body)), i32(size_of(Body))) == false {
			r.ok = false
		}
		if r.ok {
			Array_Resize(&world.bodies, bodyCount)
			for i: i32 = 0; i < bodyCount; i += 1 {
				SnapR_Bytes(r, &world.bodies.data[i], size_of(Body))
				world.bodies.data[i].userData = nil
			}
		}
	}

	if !r.ok {
		return false
	}

	// 5. Shape sparse array
	DesShapes(r, world, rdr)

	if !r.ok {
		return false
	}

	DesPodArray(r, &world.fatAABBs)

	if !r.ok {
		return false
	}

	// 6. Contact sparse array
	DesContacts(r, world)

	if !r.ok {
		return false
	}

	// 7. Joint sparse array
	{
		jointCount := SnapR_I32(r)
		if r.ok && SnapCheckCount(r, jointCount, i32(size_of(Joint)), i32(size_of(Joint))) == false {
			r.ok = false
		}
		if r.ok {
			Array_Resize(&world.joints, jointCount)
			for i: i32 = 0; i < jointCount; i += 1 {
				SnapR_Bytes(r, &world.joints.data[i], size_of(Joint))
				world.joints.data[i].userData = nil
			}
		}
	}

	// 8. Sensors
	{
		Array_Destroy(&world.sensors)
		Array_Create(&world.sensors)

		sensorCount := SnapR_I32(r)
		if r.ok && SnapCheckCount(r, sensorCount, i32(size_of(Sensor)), 4 * i32(size_of(i32))) == false {
			r.ok = false
		}
		if r.ok {
			Array_Resize(&world.sensors, sensorCount)
			mem.zero(world.sensors.data, int(sensorCount) * size_of(Sensor))
		}

		for i: i32 = 0; i < sensorCount && r.ok; i += 1 {
			s := &world.sensors.data[i]
			s.shapeId = SnapR_I32(r)
			Array_Create(&s.hits)
			Array_Create(&s.overlaps1)
			Array_Create(&s.overlaps2)
			DesPodArray(r, &s.hits)
			DesPodArray(r, &s.overlaps1)
			DesPodArray(r, &s.overlaps2)
		}
	}

	// 9. Islands
	{
		Array_Destroy(&world.islands)
		Array_Create(&world.islands)

		islandCount := SnapR_I32(r)
		if r.ok && SnapCheckCount(r, islandCount, i32(size_of(Island)), 7 * i32(size_of(i32))) == false {
			r.ok = false
		}
		if r.ok {
			Array_Resize(&world.islands, islandCount)
			mem.zero(world.islands.data, int(islandCount) * size_of(Island))
		}

		for i: i32 = 0; i < islandCount && r.ok; i += 1 {
			island := &world.islands.data[i]
			island.setIndex = SnapR_I32(r)
			island.localIndex = SnapR_I32(r)
			island.islandId = SnapR_I32(r)
			island.constraintRemoveCount = SnapR_I32(r)
			Array_Create(&island.bodies)
			Array_Create(&island.contacts)
			Array_Create(&island.joints)
			DesPodArray(r, &island.bodies)
			DesPodArray(r, &island.contacts)
			DesPodArray(r, &island.joints)
		}
	}

	// 10. Broad phase
	{
		bp := &world.broadPhase

		for t: i32 = 0; t < i32(BodyType.bodyTypeCount); t += 1 {
			DesTree(r, &bp.trees[t])
		}
		DesHashSet(r, &bp.pairSet)
		// The gathered moved siblings are per step scratch and stay at the shell's NULL
	}

	// 11. Constraint graph
	{
		graph := &world.constraintGraph
		for c: i32 = 0; c < GRAPH_COLOR_COUNT; c += 1 {
			DesGraphColor(r, &graph.colors[c], c == OVERFLOW_INDEX)
		}
	}

	DesNames(r, &world.names)

	return r.ok
}
