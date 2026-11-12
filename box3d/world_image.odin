// unified: canonical, portable world images (SPEC.md, phase 2 items 3 and 4). Not upstream.
//
// The serializer (world_snapshot.odin) writes canonical bytes: every struct it writes raw goes through a byte mask
// built once from Odin's type info, which zeroes padding, explicit padding fields, pointers and other host values,
// and embedded arrays (written separately). Unions whose active member is known (a shape's geometry, a contact's
// convex or mesh data, a joint sim's type) keep only that member's bytes. Identical world state gives identical
// image bytes in any process, on any platform.
//
// Geometry is named by a table at the head of the image, in first-use order over the shapes: kind, byte count and a
// 64-bit content hash per hull, mesh, height field and compound a live shape uses, and optionally the bytes. The
// content hash is the blob's own baked hash (rapidhash over its bytes with the hash field zeroed, computed when the
// hull, mesh or height field was made) or, for a compound, which has none, rapidhash over its bytes. Geometry blobs
// are offset-based (no pointers inside) and zero-initialized when baked, so their bytes are already canonical.
//
// Each world has a geometry store outside its arena, append-only: every blob a by-reference save named, and every
// blob a loaded image carried, owned by the store until the world is destroyed. Loading an image resolves its table
// against the store (by content hash) before touching the world, so an image naming geometry this world never saw
// is refused cleanly. Hulls then go through the world's hull database as upstream's loader does; mesh, height field
// and compound shapes point at the store's copy.
package box3d

import "base:intrinsics"
import "base:runtime"
import "core:mem"

// Image header flags (SnapHeader.flags), next to upstream's validation and double-precision flags.
SNAP_FLAG_GEOMETRY_TABLE :: u32(0x4) // shapes name geometry by an index into the image's own table
SNAP_FLAG_GEOMETRY_BYTES :: u32(0x8) // the table carries every blob's bytes: a self-contained image

// A table entry as written: kind, byte count, content hash, then the bytes when the image carries them.
ImageGeometryHeader :: struct {
	kind:        i32,
	byteCount:   i32,
	contentHash: u64,
}
#assert(size_of(ImageGeometryHeader) == 16)

// A blob owned by a world's geometry store.
ImageStoreEntry :: struct {
	contentHash: u64,
	kind:        GeometryKind,
	byteCount:   i32,
	bytes:       [^]u8,
}

// A table entry of the image being written (bytes point at the live blob) or read (at the store's copy).
ImageTableEntry :: struct {
	contentHash: u64,
	kind:        GeometryKind,
	byteCount:   i32,
	bytes:       [^]u8,
}

// Per-world image state, outside the world and its arena (host memory, heap allocator).
WorldImageData :: struct {
	generation:    u16,
	live:          bool,

	// The geometry store
	store:         [dynamic]ImageStoreEntry,
	storeByHash:   map[u64]i32,

	// Options of the image being written
	embed:         bool, // write the blobs' bytes into the table
	register:      bool, // add every blob to the store (a by-reference image can be loaded here later)

	// Scratch of the image being written or read, reused
	table:         [dynamic]ImageTableEntry,
	tableByHash:   map[u64]i32,
	tableByPtr:    map[rawptr]i32,
	shapeGeometry: [dynamic]i32, // per shape slot: table index, -1 for none
	slots:         [dynamic]RegistrySlot, // the table of the image being read, resolved to the store's blobs
	reader:        RecReader, // a reader over slots, for DeserializeIntoShell's DesShapes

	// World_HashState's image
	hashBuffer:    RecBuffer,
}

@(private = "file")
worldImages: [MAX_WORLDS]WorldImageData

// The image state of a world, reset when its slot holds a new world.
GetWorldImageData :: proc(world: ^World) -> ^WorldImageData {
	img := &worldImages[world.worldId]
	if img.live && img.generation == world.generation {
		return img
	}
	ReleaseWorldImageData(img)
	alloc := runtime.heap_allocator()
	img.generation = world.generation
	img.live = true
	img.store = make([dynamic]ImageStoreEntry, alloc)
	img.storeByHash = make(map[u64]i32, alloc)
	img.table = make([dynamic]ImageTableEntry, alloc)
	img.tableByHash = make(map[u64]i32, alloc)
	img.tableByPtr = make(map[rawptr]i32, alloc)
	img.shapeGeometry = make([dynamic]i32, alloc)
	img.slots = make([dynamic]RegistrySlot, alloc)
	return img
}

@(private = "file")
ReleaseWorldImageData :: proc(img: ^WorldImageData) {
	if !img.live {
		return
	}
	for &e in img.store {
		FreeHost(e.bytes, int(e.byteCount))
	}
	delete(img.store)
	delete(img.storeByHash)
	delete(img.table)
	delete(img.tableByHash)
	delete(img.tableByPtr)
	delete(img.shapeGeometry)
	delete(img.slots)
	RecBufFree(&img.hashBuffer)
	img^ = {}
}

// Frees a world's geometry store and image scratch. For DestroyWorld, after its shapes are gone (until it calls
// this, a world slot's store is released when the slot next holds a new world).
DestroyWorldImageData :: proc(world: ^World) {
	ReleaseWorldImageData(&worldImages[world.worldId])
}

// ---- API -------------------------------------------------------------------------------------------------------

// Save the canonical image of the world into state. With geometry, the image is self-contained (late join);
// without, it names geometry by content hash and loads only into this world (or one that has the same geometry).
// Call at a step boundary. Returns the image size in bytes.
World_SaveImage :: proc(worldId: WorldId, state: ^WorldState, withGeometry: bool) -> i32 {
	world := GetWorldFromId(worldId)
	assert(world.locked == false)
	if world.locked {
		return 0
	}
	img := GetWorldImageData(world)
	img.embed = withGeometry
	img.register = !withGeometry
	state.buffer.size = 0
	state.buffer.countOnly = false
	return SerializeWorld(world, &state.buffer, nil, img)
}

// Load an image saved by World_SaveImage (in any process, for a self-contained image) over the world, in place.
// Returns false on a bad image; an image naming geometry this world lacks is refused before the world is touched.
World_LoadImage :: proc(worldId: WorldId, bytes: []u8) -> bool {
	world := GetWorldFromId(worldId)
	assert(world.locked == false)
	if world.locked || len(bytes) == 0 || len(bytes) > int(max(i32)) {
		return false
	}
	return DeserializeIntoShell(raw_data(bytes), i32(len(bytes)), world, nil)
}

// The checksum of the world: rapidhash of its canonical image, with geometry by content hash. Equal states give
// equal hashes in any process. Reuses a per-world buffer: no allocation once warm. Call at a step boundary.
World_HashState :: proc(worldId: WorldId) -> u64 {
	world := GetWorldFromId(worldId)
	assert(world.locked == false)
	img := GetWorldImageData(world)
	img.embed = false
	img.register = false
	img.hashBuffer.size = 0
	img.hashBuffer.countOnly = false
	size := SerializeWorld(world, &img.hashBuffer, nil, img)
	return rapidhash(img.hashBuffer.data, int(size))
}

// The checksum of the awake bodies: rapidhash over each one's id, transform, and linear and angular velocity, in the
// awake set's order (so a body asleep on one machine and awake on another changes it too). A fraction of
// World_HashState's cost, for checking every tick: it leaves out contacts, warm starts, joints, islands, trees and
// sleeping bodies, which World_HashState covers. Call at a step boundary.
World_HashAwakeBodies :: proc(worldId: WorldId) -> u64 {
	world := GetWorldFromId(worldId)
	assert(world.locked == false)
	AwakeBody :: struct {
		bodyId:          i32,
		transform:       WorldTransform,
		linearVelocity:  Vec3,
		angularVelocity: Vec3,
	}
	#assert(size_of(AwakeBody) == 4 + 28 + 12 + 12) // no padding: the hash sees only these bytes

	buf := &GetWorldImageData(world).hashBuffer
	buf.size = 0
	buf.countOnly = false
	set := Array_Get(&world.solverSets, i32(SetType.awakeSet))
	for i: i32 = 0; i < set.bodySims.count; i += 1 {
		sim := &set.bodySims.data[i]
		state := &set.bodyStates.data[i]
		record := AwakeBody{sim.bodyId, sim.transform, state.linearVelocity, state.angularVelocity}
		RecBufAppend(buf, &record, size_of(record))
	}
	return rapidhash(buf.data, int(buf.size))
}

// ---- geometry table ----------------------------------------------------------------------------------------------

// The content hash and byte count of a live shape's geometry blob.
@(private = "file")
GeometryContentHash :: proc(kind: GeometryKind, blob: rawptr) -> (hash: u64, byteCount: i32) {
	switch kind {
	case .geometryHull:
		hull := (^HullData)(blob)
		hash, byteCount = hull.hash, hull.byteCount
	case .geometryMesh:
		mesh := (^MeshData)(blob)
		hash, byteCount = mesh.hash, mesh.byteCount
	case .geometryHeightField:
		hf := (^HeightFieldData)(blob)
		hash, byteCount = hf.hash, hf.byteCount
	case .geometryCompound:
		byteCount = (^CompoundData)(blob).byteCount
		hash = Hash64NonZero(cast([^]u8)blob, byteCount)
	}
	if hash == 0 {
		hash = BakedContentHash(kind, cast([^]u8)blob, byteCount)
	}
	return
}

// The content hash a blob's bytes should have: the baked hash recomputed (hash field zeroed), or for a compound,
// rapidhash over the bytes. The blob must be writable (its hash field is zeroed and put back).
@(private = "file")
BakedContentHash :: proc(kind: GeometryKind, bytes: [^]u8, byteCount: i32) -> u64 {
	if kind == .geometryCompound {
		return Hash64NonZero(bytes, byteCount)
	}
	// hull, mesh and height field: version u64, then hash u64
	field := (^u64)(&bytes[8])
	saved := field^
	field^ = 0
	hash := Hash64NonZero(bytes, byteCount)
	field^ = saved
	return hash
}

// Fill the table of the image about to be written, in first-use order over the shapes, and each live shape's
// table index (WorldImageData.shapeGeometry).
ImageBuildTable :: proc(img: ^WorldImageData, world: ^World) {
	clear(&img.table)
	clear(&img.tableByHash)
	clear(&img.tableByPtr)
	count := world.shapes.count
	resize(&img.shapeGeometry, int(count))

	lastPtr: rawptr = nil
	lastIndex: i32 = -1
	for i: i32 = 0; i < count; i += 1 {
		shape := &world.shapes.data[i]
		img.shapeGeometry[i] = -1
		if shape.id != i {
			continue
		}
		kind: GeometryKind
		blob: rawptr
		#partial switch shape.type {
		case .hullShape:
			kind, blob = .geometryHull, shape.hull
		case .meshShape:
			kind, blob = .geometryMesh, shape.mesh.data
		case .heightShape:
			kind, blob = .geometryHeightField, shape.heightField
		case .compoundShape:
			kind, blob = .geometryCompound, shape.compound
		case:
			continue
		}
		if blob == lastPtr {
			img.shapeGeometry[i] = lastIndex
			continue
		}
		index, known := img.tableByPtr[blob]
		if !known {
			hash, byteCount := GeometryContentHash(kind, blob)
			found: bool
			index, found = img.tableByHash[hash]
			if !found {
				index = i32(len(img.table))
				append(&img.table, ImageTableEntry{hash, kind, byteCount, cast([^]u8)blob})
				img.tableByHash[hash] = index
				if img.register {
					ImageStoreAdd(img, kind, hash, cast([^]u8)blob, byteCount)
				}
			}
			img.tableByPtr[blob] = index
		}
		img.shapeGeometry[i] = index
		lastPtr, lastIndex = blob, index
	}
}

// Write the table built by ImageBuildTable.
ImageWriteTable :: proc(img: ^WorldImageData, buf: ^RecBuffer) {
	count := i32(len(img.table))
	RecBufAppend(buf, &count, 4)
	for &e in img.table {
		h := ImageGeometryHeader{i32(e.kind), e.byteCount, e.contentHash}
		RecBufAppend(buf, &h, size_of(h))
		if img.embed {
			RecBufAppend(buf, e.bytes, e.byteCount)
		}
	}
}

// The store's copy of a blob, by content hash, or nil.
ImageStoreFind :: proc(img: ^WorldImageData, kind: GeometryKind, hash: u64, byteCount: i32) -> [^]u8 {
	index, found := img.storeByHash[hash]
	if !found {
		return nil
	}
	e := &img.store[index]
	if e.kind != kind || e.byteCount != byteCount {
		return nil
	}
	return e.bytes
}

// Add a copy of a blob to the store unless it has one. Returns the store's copy.
ImageStoreAdd :: proc(img: ^WorldImageData, kind: GeometryKind, hash: u64, bytes: [^]u8, byteCount: i32) -> [^]u8 {
	index, found := img.storeByHash[hash]
	if found {
		return img.store[index].bytes
	}
	copy := cast([^]u8)AllocHost(int(byteCount))
	mem.copy_non_overlapping(copy, bytes, int(byteCount))
	img.storeByHash[hash] = i32(len(img.store))
	append(&img.store, ImageStoreEntry{hash, kind, byteCount, copy})
	return copy
}

// Take in a blob carried by an image: checked (version, byte count, content hash) and copied into the store.
// Returns the store's copy, or nil for a damaged blob.
ImageStoreLoad :: proc(img: ^WorldImageData, kind: GeometryKind, hash: u64, bytes: [^]u8, byteCount: i32) -> [^]u8 {
	if existing := ImageStoreFind(img, kind, hash, byteCount); existing != nil {
		return existing
	}
	if _, taken := img.storeByHash[hash]; taken {
		return nil // the hash names another kind or size here
	}
	minSize: int
	version: u64
	switch kind {
	case .geometryHull:
		minSize, version = size_of(HullData), HULL_VERSION
	case .geometryMesh:
		minSize, version = size_of(MeshData), MESH_VERSION
	case .geometryHeightField:
		minSize, version = size_of(HeightFieldData), HEIGHT_FIELD_VERSION
	case .geometryCompound:
		minSize, version = size_of(CompoundData), COMPOUND_VERSION
	case:
		return nil
	}
	if int(byteCount) < minSize {
		return nil
	}
	copy := cast([^]u8)AllocHost(int(byteCount))
	mem.copy_non_overlapping(copy, bytes, int(byteCount))
	// Every kind starts with its version, and its byteCount field must match (hull: after the header fields)
	storedCount: i32
	switch kind {
	case .geometryHull:
		storedCount = (^HullData)(copy).byteCount
	case .geometryMesh:
		storedCount = (^MeshData)(copy).byteCount
	case .geometryHeightField:
		storedCount = (^HeightFieldData)(copy).byteCount
	case .geometryCompound:
		storedCount = (^CompoundData)(copy).byteCount
	}
	ok :=
		(^u64)(copy)^ == version &&
		storedCount == byteCount &&
		BakedContentHash(kind, copy, byteCount) == hash &&
		(kind != .geometryHull || IsValidHull((^HullData)(copy)))
	if !ok {
		FreeHost(copy, int(byteCount))
		return nil
	}
	img.storeByHash[hash] = i32(len(img.store))
	append(&img.store, ImageStoreEntry{hash, kind, byteCount, copy})
	return copy
}

// ---- canonical bytes ---------------------------------------------------------------------------------------------

// Byte masks (0xFF keep, 0x00 zero) per struct the serializer writes raw, built once from type info. A nil mask:
// every byte is kept.
@(private = "file")
imageMasks: map[typeid][]u8
@(private = "file")
contactMasks: [2][]u8 // convex, mesh (simMeshContact)
@(private = "file")
jointSimMasks: [JointType][]u8

// The structs the serializer writes raw, directly or in arrays.
@(private = "file")
IMAGE_TYPES := [?]typeid {
	Body,
	BodySim,
	BodyState,
	Shape,
	Manifold,
	TriangleCache,
	Joint,
	IslandSim,
	AABB,
	TreeNode,
	TreeProxy,
	SetItem,
	Visitor,
	ContactSpec,
	ContactLink,
	JointLink,
	SurfaceMaterial,
	Sphere,
	Capsule,
	Capacity,
	Vec3,
}

@(init, private = "file")
InitImageMasks :: proc "contextless" () {
	context = runtime.default_context()
	alloc := runtime.heap_allocator()
	imageMasks = make(map[typeid][]u8, alloc)
	for id in IMAGE_TYPES {
		ti := type_info_of(id)
		mask := make([]u8, ti.size, alloc)
		MaskKeep(ti, mask)
		imageMasks[id] = MaskOrNil(mask)
	}

	// A shape's geometry is written after it: the union is zeroed (it holds the hull or mesh pointer).
	{
		shapeMask := imageMasks[Shape]
		if shapeMask == nil {
			shapeMask = make([]u8, size_of(Shape), alloc)
			MaskKeep(type_info_of(Shape), shapeMask)
		}
		offset, union_ := UsingUnion(type_info_of(Shape))
		mem.zero(&shapeMask[offset], union_.size)
		imageMasks[Shape] = shapeMask
	}

	// A contact keeps its convex cache, or the mesh contact's bounds (its triangle cache array is written apart).
	{
		ti := type_info_of(Contact)
		offset, union_ := UsingUnion(ti)
		for member, k in ([2]typeid{ConvexContact, MeshContact}) {
			mask := make([]u8, ti.size, alloc)
			MaskKeep(ti, mask)
			mem.zero(&mask[offset], union_.size)
			mti := type_info_of(member)
			MaskKeep(mti, mask[offset:][:mti.size])
			contactMasks[k] = mask
		}
	}

	// A joint sim keeps the union member named like its type (the filter joint has none).
	{
		ti := type_info_of(JointSim)
		offset, union_ := UsingUnion(ti)
		u := runtime.type_info_base(union_).variant.(runtime.Type_Info_Struct)
		names := runtime.type_info_base(type_info_of(JointType)).variant.(runtime.Type_Info_Enum)
		for type in JointType {
			mask := make([]u8, ti.size, alloc)
			MaskKeep(ti, mask)
			mem.zero(&mask[offset], union_.size)
			name: string
			for value, k in names.values {
				if i64(value) == i64(type) {
					name = names.names[k]
				}
			}
			for f in 0 ..< int(u.field_count) {
				if u.names[f] == name {
					mti := u.types[f]
					MaskKeep(mti, mask[offset + int(u.offsets[f]):][:mti.size])
				}
			}
			jointSimMasks[type] = mask
		}
	}
}

// Sets the mask bytes (or bits, for bit fields) of the values worth keeping in a value of type ti.
@(private = "file")
MaskKeep :: proc(ti: ^runtime.Type_Info, dst: []u8) {
	#partial switch v in ti.variant {
	case runtime.Type_Info_Named:
		// Box3D's Array(T) (data, count, capacity): a struct that embeds one writes its contents separately.
		if v.pkg == "box3d" && len(v.name) >= 5 && v.name[:5] == "Array" {
			return
		}
		MaskKeep(v.base, dst)
	case runtime.Type_Info_Struct:
		// Raw unions OR their members' masks (every field at offset 0).
		for i in 0 ..< int(v.field_count) {
			name := v.names[i]
			if !v.usings[i] && (name == "_" || HasPrefix(name, "padding") || HasPrefix(name, "_pad")) {
				continue
			}
			ft := v.types[i]
			off := int(v.offsets[i])
			MaskKeep(ft, dst[off:][:ft.size])
		}
	case runtime.Type_Info_Array:
		for k in 0 ..< v.count {
			MaskKeep(v.elem, dst[k * v.elem_size:][:v.elem_size])
		}
	case runtime.Type_Info_Enumerated_Array:
		for k in 0 ..< v.count {
			MaskKeep(v.elem, dst[k * v.elem_size:][:v.elem_size])
		}
	case runtime.Type_Info_Bit_Field:
		for i in 0 ..< v.field_count {
			for b in v.bit_offsets[i] ..< v.bit_offsets[i] + v.bit_sizes[i] {
				dst[b / 8] |= 1 << (b % 8)
			}
		}
	case runtime.Type_Info_Integer,
	     runtime.Type_Info_Float,
	     runtime.Type_Info_Boolean,
	     runtime.Type_Info_Rune,
	     runtime.Type_Info_Enum,
	     runtime.Type_Info_Bit_Set,
	     runtime.Type_Info_Simd_Vector,
	     runtime.Type_Info_Matrix,
	     runtime.Type_Info_Quaternion,
	     runtime.Type_Info_Complex:
		for &b in dst {
			b = 0xFF
		}
	case:
	// Pointers, procedures, strings, slices, maps: host values, zeroed.
	}
}

@(private = "file")
HasPrefix :: proc(s, prefix: string) -> bool {
	return len(s) >= len(prefix) && s[:len(prefix)] == prefix
}

// The offset and type of a struct's `using _: struct #raw_union {...}` field.
@(private = "file")
UsingUnion :: proc(ti: ^runtime.Type_Info) -> (offset: int, union_: ^runtime.Type_Info) {
	s := runtime.type_info_base(ti).variant.(runtime.Type_Info_Struct)
	for i in 0 ..< int(s.field_count) {
		ft := s.types[i]
		if fs, ok := runtime.type_info_base(ft).variant.(runtime.Type_Info_Struct);
		   ok && s.usings[i] && .raw_union in fs.flags {
			return int(s.offsets[i]), ft
		}
	}
	panic("box3d: no using raw union")
}

@(private = "file")
MaskOrNil :: proc(mask: []u8) -> []u8 {
	for b in mask {
		if b != 0xFF {
			return mask
		}
	}
	delete(mask, runtime.heap_allocator())
	return nil
}

// dst = src & mask, a value at a time (dst at any alignment).
@(private = "file")
MaskedCopy :: #force_inline proc "contextless" (dst, src: [^]u8, mask: []u8) {
	n := len(mask)
	i := 0
	for ; i + 16 <= n; i += 16 {
		v := intrinsics.unaligned_load((^#simd[16]u8)(&src[i])) & intrinsics.unaligned_load((^#simd[16]u8)(&mask[i]))
		intrinsics.unaligned_store((^#simd[16]u8)(&dst[i]), v)
	}
	for ; i + 8 <= n; i += 8 {
		v := intrinsics.unaligned_load((^u64)(&src[i])) & intrinsics.unaligned_load((^u64)(&mask[i]))
		intrinsics.unaligned_store((^u64)(&dst[i]), v)
	}
	for ; i < n; i += 1 {
		dst[i] = src[i] & mask[i]
	}
}

@(private = "file")
ImageMaskOf :: proc(id: typeid) -> []u8 {
	return imageMasks[id]
}

// Append count values of type T to buf with their non-canonical bytes zeroed, in one pass.
SnapW_Canonical :: proc(buf: ^RecBuffer, src: rawptr, $T: typeid, count: i32) {
	n := count * i32(size_of(T))
	if count <= 0 || buf.countOnly {
		RecBufAppend(buf, src, n)
		return
	}
	mask: []u8
	when T != JointSim && T != Contact {
		mask = ImageMaskOf(T)
		if mask == nil {
			RecBufAppend(buf, src, n)
			return
		}
	}
	// RecBufAppend's growth, then a masked copy instead of its memcpy
	if buf.size + n > buf.capacity {
		newCap := max(buf.capacity * 2, buf.size + n + 64)
		if buf.data == nil {
			buf.data = cast([^]u8)AllocHost(int(newCap))
		} else {
			buf.data = cast([^]u8)GrowAllocHost(buf.data, buf.capacity, newCap)
		}
		buf.capacity = newCap
	}
	dst := ([^]u8)(&buf.data[buf.size])
	from := ([^]u8)(src)
	for k: i32 = 0; k < count; k += 1 {
		at := int(k) * size_of(T)
		when T == JointSim {
			// The union keeps the member named like the joint's type.
			type := (^JointSim)(&from[at]).type
			if type < min(JointType) || type > max(JointType) {
				mem.zero(&dst[at], size_of(JointSim))
				continue
			}
			mask = jointSimMasks[type]
		} else when T == Contact {
			// The union keeps the convex cache or the mesh contact's bounds.
			mask = contactMasks[((^Contact)(&from[at]).flags & simMeshContact) != 0 ? 1 : 0]
		}
		MaskedCopy(&dst[at], &from[at], mask)
	}
	buf.size += n
}
