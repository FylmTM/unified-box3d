// Ported from box3d src/recording.c and src/recording.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:runtime"
import "core:mem"
import "core:os"

// FNV-1a 64-bit constants
SNAP_FNV_INIT :: u64(14695981039346656037)
SNAP_FNV_PRIME :: u64(1099511628211)

// Mix a world position at full width so the determinism gate validates past float precision
// when the body is far from the origin.
FnvMixPosition :: #force_inline proc(hash: u64, p: Pos) -> u64 {
	hash := hash
	// unified: single precision only: the BOX3D_DOUBLE_PRECISION branch is not ported.
	fx := transmute(u32)p.x
	fy := transmute(u32)p.y
	fz := transmute(u32)p.z
	bx, by, bz := u64(fx), u64(fy), u64(fz)
	hash = (hash ~ bx) * SNAP_FNV_PRIME
	hash = (hash ~ by) * SNAP_FNV_PRIME
	hash = (hash ~ bz) * SNAP_FNV_PRIME
	return hash
}

// Magic 'B3RC' in little-endian: bytes B(0x42) 3(0x33) R(0x52) C(0x43)
REC_MAGIC :: u32(0x43523342)

// Major recording version is bumped when writers change.
// Major version 6 widened the body def with the safety factor.
REC_VERSION_MAJOR :: 6

// Minor tracks op-stream additions that keep the 48 byte header shape.
// Minor version 7 made the contact body sim locators persistent
REC_VERSION_MINOR :: 10

// File header, fixed 48 bytes. Contains the registry locator so the player
// can load geometry before replaying any ops.
RecHeader :: struct {
	magic:             u32, // 'B3RC' = 0x43523342
	versionMajor:      u16, // B3_REC_VERSION_MAJOR
	versionMinor:      u16, // B3_REC_VERSION_MINOR
	pointerWidth:      u8, // sizeof(void*), gates POD-def struct layout
	bigEndian:         u8, // 0 on all supported targets
	validationEnabled: u8, // 1 if built with BOX3D_VALIDATE, diagnostic only
	reserved:          u8,
	lengthScale:       f32, // b3GetLengthUnitsPerMeter()
	reserved2:         u32,
	reserved3:         u32, // explicit pad so the 64-bit fields align with no implicit gap
	snapshotSize:      u64, // bytes of snapshot blob after the header (0 in Phase 1)
	registryOffset:    u64, // absolute offset to trailing registry block, backpatched at stop
	registryByteCount: u64, // size of the registry block
}

#assert(size_of(RecHeader) == 48, "recording header must be 48 bytes")

// Growable append-only byte buffer. Doubles on demand. countOnly mode tallies size without
// allocating, used to size a buffer cheaply before a second filling pass.
RecBuffer :: struct {
	data:      [^]u8,
	capacity:  i32,
	size:      i32,
	countOnly: bool,
}

// Geometry kinds for the trailing registry section
GeometryKind :: enum i32 {
	geometryHull,
	geometryMesh,
	geometryHeightField,
	geometryCompound,
}

// One entry per unique geometry blob. id == index in the entries array.
// hashNext chains entries that share a content hash so dedup stays exact under a hash collision,
// which the keyframe registry depends on to never grow during capture. B3_NULL_INDEX ends the chain.
GeometryEntry :: struct {
	contentHash: u64,
	id:          u32,
	kind:        GeometryKind,
	byteCount:   i32,
	bytes:       [^]u8,
	hashNext:    i32,
}

// Growable array of geometry entries. Ids are array indices, so the array is serialized in order.
// dedupMap maps content hash to entry id for O(1) dedup; it is opaque here and owned by recording.c.
GeometryRegistry :: struct {
	entries:  Array(GeometryEntry),
	dedupMap: rawptr,
}

// Limit the maximum query name length to make recording simpler. Query names longer than this
// probably indicate a bug in user code.
MAX_QUERY_NAME_LENGTH :: 64

// Query tag from b3QueryFilter.
// Stored once per key in the trailing block so a tagged query carries only the 8 byte key on the wire.
// Shared by the recorder (accumulate) and the player (load).
RecTag :: struct {
	// hash of (id, queryName)
	key:       u64,
	id:        u64,
	queryName: [MAX_QUERY_NAME_LENGTH + 1]u8,
}

// User-owned recording buffer. The world appends into it while active; the host saves and
// destroys it. Opaque across the public API.
Recording :: struct {
	buffer:            RecBuffer,
	recordStart:       i32, // offset of the 3-byte size field for u24 backpatch
	lock:              ^Mutex, // serializes record writes from concurrent threads
	registry:          GeometryRegistry,

	// Interned query tags accumulated during capture, written to the tail of the registry block at stop.
	// tagMap maps a tag key to its index for O(1) dedup.
	tags:              [^]RecTag,
	tagCount:          i32,
	tagCapacity:       i32,
	tagMap:            rawptr,

	// Union of world bounds over every recorded step, written at stop.
	accumulatedBounds: AABB,
	haveBounds:        bool,
}

// C type aliases per TAG, used in the X-macro codegen arg structs
RecCType_BOOL :: bool
RecCType_I32 :: i32
RecCType_U8 :: u8
RecCType_U16 :: u16
RecCType_U32 :: u32
RecCType_U64 :: u64
RecCType_F32 :: f32
RecCType_F64 :: f64
RecCType_VEC3 :: Vec3
RecCType_QUAT :: Quat
RecCType_TRANSFORM :: Transform
RecCType_POSITION :: Pos
RecCType_WORLDXF :: WorldTransform
RecCType_MATRIX3 :: Matrix3
RecCType_AABB :: AABB
RecCType_SPHERE :: Sphere
RecCType_CAPSULE :: Capsule
RecCType_QUERYFILTER :: QueryFilter
RecCType_SHAPEPROXY :: ShapeProxy
// Geometry reference: a plain u32 id into the trailing registry
RecCType_GEOMID :: u32
RecCType_FILTER :: Filter
RecCType_MATERIAL :: SurfaceMaterial
RecCType_MASSDATA :: MassData
RecCType_LOCKS :: MotionLocks
RecCType_STR :: cstring
RecCType_WORLDID :: WorldId
RecCType_BODYID :: BodyId
RecCType_SHAPEID :: ShapeId
RecCType_JOINTID :: JointId
RecCType_BODYDEF :: BodyDef
RecCType_SHAPEDEF :: ShapeDef
RecCType_EXPLOSIONDEF :: ExplosionDef
RecCType_PARALLELJOINTDEF :: ParallelJointDef
RecCType_DISTANCEJOINTDEF :: DistanceJointDef
RecCType_FILTERJOINTDEF :: FilterJointDef
RecCType_MOTORJOINTDEF :: MotorJointDef
RecCType_PRISMATICJOINTDEF :: PrismaticJointDef
RecCType_REVOLUTEJOINTDEF :: RevoluteJointDef
RecCType_SPHERICALJOINTDEF :: SphericalJointDef
RecCType_WELDJOINTDEF :: WeldJointDef
RecCType_WHEELJOINTDEF :: WheelJointDef
RecCType_MOVERJOINTDEF :: MoverJointDef
RecCType_POGOJOINTDEF :: PogoJointDef

// unified: the arg structs, opcode constants and per-op writers that recording.h generates from the manifest
// are written out in recording_ops.odin. The B3_REC and B3_REC_CREATE macros are expanded at each call site.

// Per-query writer context: holds the user fcn+ctx, the local payload buffer, and the hit counter
RecQueryWriter :: struct {
	userFcn:     struct #raw_union {
		overlapFcn:     OverlapResultFcn,
		castFcn:        CastResultFcn,
		planeFcn:       PlaneResultFcn,
		moverFilterFcn: MoverFilterFcn,
	},
	userContext: rawptr,
	buf:         RecBuffer, // per-call local payload, heap-backed
	countOffset: i32, // offset of the reserved u32 hit-count slot
	hitCount:    u32,
	tagId:       u64, // caller query id, 0 = untagged. Emitted as a QueryTag before the record.
	tagName:     cstring, // caller query name, interned by id. NULL = none.
}

// Buffer helpers

RecBufAppend :: proc(buf: ^RecBuffer, data: rawptr, size: i32) {
	if size <= 0 {
		return
	}

	if buf.countOnly {
		buf.size += size
		return
	}

	if buf.size + size > buf.capacity {
		newCap := buf.capacity * 2
		if newCap < buf.size + size + 64 {
			newCap = buf.size + size + 64
		}
		if buf.data == nil {
			buf.data = cast([^]u8)AllocHost(int(newCap))
		} else {
			buf.data = cast([^]u8)GrowAllocHost(buf.data, buf.capacity, newCap)
		}
		buf.capacity = newCap
	}

	mem.copy(&buf.data[buf.size], data, int(size))
	buf.size += size
}

RecBufFree :: proc(buf: ^RecBuffer) {
	if buf.data != nil {
		Free(buf.data, int(buf.capacity))
		buf.data = nil
		buf.capacity = 0
		buf.size = 0
	}
}

// Write primitives

RecW_U8 :: proc(buf: ^RecBuffer, v: u8) {
	v := v
	RecBufAppend(buf, &v, 1)
}

RecW_U16 :: proc(buf: ^RecBuffer, v: u16) {
	b := [2]u8{u8(v), u8(v >> 8)}
	RecBufAppend(buf, &b, 2)
}

RecW_U32 :: proc(buf: ^RecBuffer, v: u32) {
	b := [4]u8{u8(v), u8(v >> 8), u8(v >> 16), u8(v >> 24)}
	RecBufAppend(buf, &b, 4)
}

RecW_U64 :: proc(buf: ^RecBuffer, v: u64) {
	b := [8]u8{u8(v), u8(v >> 8), u8(v >> 16), u8(v >> 24), u8(v >> 32), u8(v >> 40), u8(v >> 48), u8(v >> 56)}
	RecBufAppend(buf, &b, 8)
}

RecW_I32 :: proc(buf: ^RecBuffer, v: i32) {
	RecW_U32(buf, u32(v))
}

RecW_F32 :: proc(buf: ^RecBuffer, v: f32) {
	bits := transmute(u32)v
	RecW_U32(buf, bits)
}

RecW_F64 :: proc(buf: ^RecBuffer, v: f64) {
	bits := transmute(u64)v
	RecW_U64(buf, bits)
}

RecW_BOOL :: proc(buf: ^RecBuffer, v: bool) {
	RecW_U8(buf, v ? 1 : 0)
}

RecW_VEC3 :: proc(buf: ^RecBuffer, v: Vec3) {
	RecW_F32(buf, v.x)
	RecW_F32(buf, v.y)
	RecW_F32(buf, v.z)
}

RecW_QUAT :: proc(buf: ^RecBuffer, v: Quat) {
	RecW_F32(buf, v.v.x)
	RecW_F32(buf, v.v.y)
	RecW_F32(buf, v.v.z)
	RecW_F32(buf, v.s)
}

RecW_TRANSFORM :: proc(buf: ^RecBuffer, v: Transform) {
	RecW_VEC3(buf, v.p)
	RecW_QUAT(buf, v.q)
}

// World position at full precision so recordings reproduce the simulation far from the origin.
// In the float build this is three floats, wire-identical to VEC3.
RecW_POSITION :: proc(buf: ^RecBuffer, v: Pos) {
	// unified: single precision only: the BOX3D_DOUBLE_PRECISION branch is not ported.
	RecW_F32(buf, v.x)
	RecW_F32(buf, v.y)
	RecW_F32(buf, v.z)
}

RecW_WORLDXF :: proc(buf: ^RecBuffer, v: WorldTransform) {
	RecW_POSITION(buf, v.p)
	RecW_QUAT(buf, v.q)
}

RecW_MATRIX3 :: proc(buf: ^RecBuffer, v: Matrix3) {
	RecW_VEC3(buf, v.cx)
	RecW_VEC3(buf, v.cy)
	RecW_VEC3(buf, v.cz)
}

RecW_AABB :: proc(buf: ^RecBuffer, v: AABB) {
	RecW_VEC3(buf, v.lowerBound)
	RecW_VEC3(buf, v.upperBound)
}

RecW_QUERYFILTER :: proc(buf: ^RecBuffer, v: QueryFilter) {
	RecW_U64(buf, v.categoryBits)
	RecW_U64(buf, v.maskBits)
}

// Variable length: count, then count points, then radius. The point cloud lives behind a pointer so
// it cannot ride along as POD.
RecW_SHAPEPROXY :: proc(buf: ^RecBuffer, v: ShapeProxy) {
	count := v.count
	if count < 0 {
		count = 0
	}
	if count > MAX_SHAPE_CAST_POINTS {
		count = MAX_SHAPE_CAST_POINTS
	}
	RecW_I32(buf, count)
	for i: i32 = 0; i < count; i += 1 {
		RecW_VEC3(buf, v.points[i])
	}
	RecW_F32(buf, v.radius)
}

RecW_TREESTATS :: proc(buf: ^RecBuffer, v: TreeStats) {
	RecW_I32(buf, v.nodeVisits)
	RecW_I32(buf, v.leafVisits)
}

RecW_RAYRESULT :: proc(buf: ^RecBuffer, v: RayResult) {
	RecW_SHAPEID(buf, v.shapeId)
	RecW_POSITION(buf, v.point)
	RecW_VEC3(buf, v.normal)
	RecW_U64(buf, v.userMaterialId)
	RecW_F32(buf, v.fraction)
	RecW_I32(buf, v.triangleIndex)
	RecW_I32(buf, v.childIndex)
	RecW_BOOL(buf, v.hit)
}

RecW_PLANERESULT :: proc(buf: ^RecBuffer, v: PlaneResult) {
	RecW_VEC3(buf, v.plane.normal)
	RecW_F32(buf, v.plane.offset)
	RecW_VEC3(buf, v.point)
	RecW_I32(buf, v.triangleIndex)
	RecW_I32(buf, v.childIndex)
	RecW_I32(buf, v.materialIndex)
}

RecW_WORLDID :: proc(buf: ^RecBuffer, v: WorldId) {
	RecW_U32(buf, StoreWorldId(v))
}

RecW_BODYID :: proc(buf: ^RecBuffer, v: BodyId) {
	RecW_U64(buf, StoreBodyId(v))
}

RecW_SHAPEID :: proc(buf: ^RecBuffer, v: ShapeId) {
	RecW_U64(buf, StoreShapeId(v))
}

RecW_JOINTID :: proc(buf: ^RecBuffer, v: JointId) {
	RecW_U64(buf, StoreJointId(v))
}

// Pointer-free POD; pointerWidth in the header gates the layout on replay
RecW_SPHERE :: proc(buf: ^RecBuffer, v: Sphere) {
	v := v
	RecBufAppend(buf, &v, i32(size_of(Sphere)))
}

RecW_CAPSULE :: proc(buf: ^RecBuffer, v: Capsule) {
	v := v
	RecBufAppend(buf, &v, i32(size_of(Capsule)))
}

RecW_GEOMID :: proc(buf: ^RecBuffer, v: u32) {
	RecW_U32(buf, v)
}

RecW_FILTER :: proc(buf: ^RecBuffer, v: Filter) {
	RecW_U64(buf, v.categoryBits)
	RecW_U64(buf, v.maskBits)
	RecW_I32(buf, v.groupIndex)
}

RecW_MATERIAL :: proc(buf: ^RecBuffer, v: SurfaceMaterial) {
	RecW_F32(buf, v.friction)
	RecW_F32(buf, v.restitution)
	RecW_F32(buf, v.rollingResistance)
	RecW_VEC3(buf, v.tangentVelocity)
	RecW_U64(buf, v.userMaterialId)
	RecW_U32(buf, v.customColor)
}

RecW_MASSDATA :: proc(buf: ^RecBuffer, v: MassData) {
	RecW_F32(buf, v.mass)
	RecW_VEC3(buf, v.center)
	RecW_MATRIX3(buf, v.inertia)
}

RecW_LOCKS :: proc(buf: ^RecBuffer, v: MotionLocks) {
	RecW_BOOL(buf, v.linearX)
	RecW_BOOL(buf, v.linearY)
	RecW_BOOL(buf, v.linearZ)
	RecW_BOOL(buf, v.angularX)
	RecW_BOOL(buf, v.angularY)
	RecW_BOOL(buf, v.angularZ)
}

// unified: not file-private (C: static): recording_ops.odin's generated arg writers call it.
RecW_STR :: proc(buf: ^RecBuffer, s: cstring) {
	if s == nil {
		RecW_U16(buf, 0xFFFF)
		return
	}
	str := ([^]u8)(s)
	len: i32 = 0
	for str[len] != 0 && len < 65534 {
		len += 1
	}
	RecW_U16(buf, u16(len))
	if len > 0 {
		RecBufAppend(buf, str, len)
	}
}
// unified end

// Hand-written def helpers. Zero pointer and cookie fields before serializing.
// Readers call b3Default*Def() first to get the cookie, then overwrite fields.

// Tripwire: each def serializer below is paired with a reader in recording_replay.c, and the two must
// stay field-for-field in sync. Add a field to a def and the size changes, firing the matching assert
// so the writer and reader both get updated. Only enforced on the 64-bit target; each def lists the
// single-precision and double-precision sizes (equal for most), so either build configuration passes.
#assert(
	size_of(rawptr) != 8 || size_of(ExplosionDef) == 32 || size_of(ExplosionDef) == 48,
	"b3ExplosionDef changed: update b3RecW_EXPLOSIONDEF and b3RecR_EXPLOSIONDEF together",
)
#assert(
	size_of(rawptr) != 8 || size_of(BodyDef) == 112 || size_of(BodyDef) == 128,
	"b3BodyDef changed: update b3RecW_BODYDEF and b3RecR_BODYDEF together",
)
#assert(
	size_of(rawptr) != 8 || size_of(ShapeDef) == 120,
	"b3ShapeDef changed: update b3RecW_SHAPEDEF and b3RecR_SHAPEDEF together",
)
#assert(
	size_of(rawptr) != 8 || size_of(ParallelJointDef) == 128,
	"b3ParallelJointDef changed: update b3RecW_PARALLELJOINTDEF and its reader together",
)
#assert(
	size_of(rawptr) != 8 || size_of(DistanceJointDef) == 160,
	"b3DistanceJointDef changed: update b3RecW_DISTANCEJOINTDEF and its reader together",
)
#assert(
	size_of(rawptr) != 8 || size_of(FilterJointDef) == 112,
	"b3FilterJointDef changed: update b3RecW_FILTERJOINTDEF and its reader together",
)
#assert(
	size_of(rawptr) != 8 || size_of(MotorJointDef) == 168,
	"b3MotorJointDef changed: update b3RecW_MOTORJOINTDEF and its reader together",
)
#assert(
	size_of(rawptr) != 8 || size_of(PrismaticJointDef) == 152,
	"b3PrismaticJointDef changed: update b3RecW_PRISMATICJOINTDEF and its reader together",
)
#assert(
	size_of(rawptr) != 8 || size_of(RevoluteJointDef) == 152,
	"b3RevoluteJointDef changed: update b3RecW_REVOLUTEJOINTDEF and its reader together",
)
#assert(
	size_of(rawptr) != 8 || size_of(SphericalJointDef) == 184,
	"b3SphericalJointDef changed: update b3RecW_SPHERICALJOINTDEF and its reader together",
)
#assert(
	size_of(rawptr) != 8 || size_of(WeldJointDef) == 128,
	"b3WeldJointDef changed: update b3RecW_WELDJOINTDEF and its reader together",
)
#assert(
	size_of(rawptr) != 8 || size_of(WheelJointDef) == 184,
	"b3WheelJointDef changed: update b3RecW_WHEELJOINTDEF and its reader together",
)
#assert(
	size_of(rawptr) != 8 || size_of(MoverJointDef) == 128,
	"b3MoverJointDef changed: update b3RecW_MOVERJOINTDEF and its reader together",
)
#assert(
	size_of(rawptr) != 8 || size_of(PogoJointDef) == 152,
	"b3PogoJointDef changed: update b3RecW_POGOJOINTDEF and its reader together",
)

RecW_EXPLOSIONDEF :: proc(buf: ^RecBuffer, v: ExplosionDef) {
	RecW_U64(buf, v.maskBits)
	RecW_POSITION(buf, v.position)
	RecW_F32(buf, v.radius)
	RecW_F32(buf, v.falloff)
	RecW_F32(buf, v.impulsePerArea)
}

RecW_BODYDEF :: proc(buf: ^RecBuffer, v: BodyDef) {
	RecW_I32(buf, i32(v.type))
	RecW_POSITION(buf, v.position)
	RecW_QUAT(buf, v.rotation)
	RecW_VEC3(buf, v.linearVelocity)
	RecW_VEC3(buf, v.angularVelocity)
	RecW_F32(buf, v.linearDamping)
	RecW_F32(buf, v.angularDamping)
	RecW_F32(buf, v.gravityScale)
	RecW_F32(buf, v.sleepThreshold)
	RecW_F32(buf, v.safetyFactor)
	RecW_STR(buf, v.name)
	// userData: not preserved
	RecW_U64(buf, 0)
	RecW_LOCKS(buf, v.motionLocks)
	RecW_BOOL(buf, v.enableSleep)
	RecW_BOOL(buf, v.isAwake)
	RecW_BOOL(buf, v.isBullet)
	RecW_BOOL(buf, v.isEnabled)
	RecW_BOOL(buf, v.allowFastRotation)
	RecW_BOOL(buf, v.enableContactRecycling)
	// internalValue omitted
}

RecW_SHAPEDEF :: proc(buf: ^RecBuffer, v: ShapeDef) {
	RecW_STR(buf, v.name)

	// userData: not preserved
	RecW_U64(buf, 0)
	// Per-triangle materials: length-prefixed so the reader can rebuild the array.
	// Guard NULL so a default def (materialCount=0, materials=NULL) round-trips cleanly.
	matCount := (v.materials != nil) ? v.materialCount : 0
	RecW_I32(buf, matCount)
	for i: i32 = 0; i < matCount; i += 1 {
		RecW_MATERIAL(buf, v.materials[i])
	}
	RecW_MATERIAL(buf, v.baseMaterial)
	RecW_F32(buf, v.density)
	RecW_F32(buf, v.explosionScale)
	RecW_FILTER(buf, v.filter)
	RecW_BOOL(buf, v.enableCustomFiltering)
	RecW_BOOL(buf, v.isSensor)
	RecW_BOOL(buf, v.enableSensorEvents)
	RecW_BOOL(buf, v.enableContactEvents)
	RecW_BOOL(buf, v.enableHitEvents)
	RecW_BOOL(buf, v.enablePreSolveEvents)
	RecW_BOOL(buf, v.invokeContactCreation)
	RecW_BOOL(buf, v.updateBodyMass)
	RecW_BOOL(buf, v.enableSpeculativeContact)
	// internalValue omitted
}

// Joint defs share a base. Body ids are written as packed ids for replay remapping.
@(private = "file")
RecW_JointBase :: proc(buf: ^RecBuffer, base: ^JointDef) {
	// userData: not preserved
	RecW_U64(buf, 0)
	RecW_BODYID(buf, base.bodyIdA)
	RecW_BODYID(buf, base.bodyIdB)
	RecW_TRANSFORM(buf, base.localFrameA)
	RecW_TRANSFORM(buf, base.localFrameB)
	RecW_F32(buf, base.forceThreshold)
	RecW_F32(buf, base.torqueThreshold)
	RecW_F32(buf, base.constraintHertz)
	RecW_F32(buf, base.constraintDampingRatio)
	RecW_F32(buf, base.drawScale)
	RecW_BOOL(buf, base.collideConnected)
	// internalValue omitted
}

RecW_PARALLELJOINTDEF :: proc(buf: ^RecBuffer, v: ParallelJointDef) {
	v := v
	RecW_JointBase(buf, &v.base)
	RecW_F32(buf, v.hertz)
	RecW_F32(buf, v.dampingRatio)
	RecW_F32(buf, v.maxTorque)
}

RecW_DISTANCEJOINTDEF :: proc(buf: ^RecBuffer, v: DistanceJointDef) {
	v := v
	RecW_JointBase(buf, &v.base)
	RecW_F32(buf, v.length)
	RecW_BOOL(buf, v.enableSpring)
	RecW_F32(buf, v.lowerSpringForce)
	RecW_F32(buf, v.upperSpringForce)
	RecW_F32(buf, v.hertz)
	RecW_F32(buf, v.dampingRatio)
	RecW_BOOL(buf, v.enableLimit)
	RecW_F32(buf, v.minLength)
	RecW_F32(buf, v.maxLength)
	RecW_BOOL(buf, v.enableMotor)
	RecW_F32(buf, v.maxMotorForce)
	RecW_F32(buf, v.motorSpeed)
}

RecW_FILTERJOINTDEF :: proc(buf: ^RecBuffer, v: FilterJointDef) {
	v := v
	RecW_JointBase(buf, &v.base)
}

RecW_MOTORJOINTDEF :: proc(buf: ^RecBuffer, v: MotorJointDef) {
	v := v
	RecW_JointBase(buf, &v.base)
	RecW_VEC3(buf, v.linearVelocity)
	RecW_F32(buf, v.maxVelocityForce)
	RecW_VEC3(buf, v.angularVelocity)
	RecW_F32(buf, v.maxVelocityTorque)
	RecW_F32(buf, v.linearHertz)
	RecW_F32(buf, v.linearDampingRatio)
	RecW_F32(buf, v.maxSpringForce)
	RecW_F32(buf, v.angularHertz)
	RecW_F32(buf, v.angularDampingRatio)
	RecW_F32(buf, v.maxSpringTorque)
}

RecW_PRISMATICJOINTDEF :: proc(buf: ^RecBuffer, v: PrismaticJointDef) {
	v := v
	RecW_JointBase(buf, &v.base)
	RecW_BOOL(buf, v.enableSpring)
	RecW_F32(buf, v.hertz)
	RecW_F32(buf, v.dampingRatio)
	RecW_F32(buf, v.targetTranslation)
	RecW_BOOL(buf, v.enableLimit)
	RecW_F32(buf, v.lowerTranslation)
	RecW_F32(buf, v.upperTranslation)
	RecW_BOOL(buf, v.enableMotor)
	RecW_F32(buf, v.maxMotorForce)
	RecW_F32(buf, v.motorSpeed)
}

RecW_REVOLUTEJOINTDEF :: proc(buf: ^RecBuffer, v: RevoluteJointDef) {
	v := v
	RecW_JointBase(buf, &v.base)
	RecW_F32(buf, v.targetAngle)
	RecW_BOOL(buf, v.enableSpring)
	RecW_F32(buf, v.hertz)
	RecW_F32(buf, v.dampingRatio)
	RecW_BOOL(buf, v.enableLimit)
	RecW_F32(buf, v.lowerAngle)
	RecW_F32(buf, v.upperAngle)
	RecW_BOOL(buf, v.enableMotor)
	RecW_F32(buf, v.maxMotorTorque)
	RecW_F32(buf, v.motorSpeed)
}

RecW_SPHERICALJOINTDEF :: proc(buf: ^RecBuffer, v: SphericalJointDef) {
	v := v
	RecW_JointBase(buf, &v.base)
	RecW_BOOL(buf, v.enableSpring)
	RecW_F32(buf, v.hertz)
	RecW_F32(buf, v.dampingRatio)
	RecW_QUAT(buf, v.targetRotation)
	RecW_BOOL(buf, v.enableConeLimit)
	RecW_F32(buf, v.coneAngle)
	RecW_BOOL(buf, v.enableTwistLimit)
	RecW_F32(buf, v.lowerTwistAngle)
	RecW_F32(buf, v.upperTwistAngle)
	RecW_BOOL(buf, v.enableMotor)
	RecW_F32(buf, v.maxMotorTorque)
	RecW_VEC3(buf, v.motorVelocity)
}

RecW_WELDJOINTDEF :: proc(buf: ^RecBuffer, v: WeldJointDef) {
	v := v
	RecW_JointBase(buf, &v.base)
	RecW_F32(buf, v.linearHertz)
	RecW_F32(buf, v.angularHertz)
	RecW_F32(buf, v.linearDampingRatio)
	RecW_F32(buf, v.angularDampingRatio)
}

RecW_WHEELJOINTDEF :: proc(buf: ^RecBuffer, v: WheelJointDef) {
	v := v
	RecW_JointBase(buf, &v.base)
	RecW_BOOL(buf, v.enableSuspensionSpring)
	RecW_F32(buf, v.suspensionHertz)
	RecW_F32(buf, v.suspensionDampingRatio)
	RecW_BOOL(buf, v.enableSuspensionLimit)
	RecW_F32(buf, v.lowerSuspensionLimit)
	RecW_F32(buf, v.upperSuspensionLimit)
	RecW_BOOL(buf, v.enableSpinMotor)
	RecW_F32(buf, v.maxSpinTorque)
	RecW_F32(buf, v.spinSpeed)
	RecW_BOOL(buf, v.enableSteering)
	RecW_F32(buf, v.steeringHertz)
	RecW_F32(buf, v.steeringDampingRatio)
	RecW_F32(buf, v.targetSteeringAngle)
	RecW_F32(buf, v.maxSteeringTorque)
	RecW_BOOL(buf, v.enableSteeringLimit)
	RecW_F32(buf, v.lowerSteeringLimit)
	RecW_F32(buf, v.upperSteeringLimit)
}

RecW_MOVERJOINTDEF :: proc(buf: ^RecBuffer, v: MoverJointDef) {
	v := v
	RecW_JointBase(buf, &v.base)
	RecW_VEC3(buf, v.linearVelocity)
	RecW_F32(buf, v.maxForce)
}

RecW_POGOJOINTDEF :: proc(buf: ^RecBuffer, v: PogoJointDef) {
	v := v
	RecW_JointBase(buf, &v.base)
	RecW_VEC3(buf, v.normal)
	RecW_F32(buf, v.hertz)
	RecW_F32(buf, v.dampingRatio)
	RecW_F32(buf, v.restLength)
	RecW_F32(buf, v.maxTensionForce)
	RecW_F32(buf, v.maxCompressionForce)
	RecW_F32(buf, v.impulse)
	RecW_F32(buf, v.velocity)
}

// Query recording. A query collects a variable number of hits through a user callback, so the count
// is not known until the callback stops firing. The record is built in a local buffer with a
// reserved hit-count slot, then committed whole under the lock so concurrent query threads never
// interleave records in the shared buffer.

RecReserveU32 :: proc(buf: ^RecBuffer) -> i32 {
	offset := buf.size
	zero := [4]u8{0, 0, 0, 0}
	RecBufAppend(buf, &zero, 4)
	return offset
}

RecPatchU32 :: proc(buf: ^RecBuffer, offset: i32, v: u32) {
	assert(offset >= 0 && offset + 4 <= buf.size)
	p := buf.data[offset:]
	p[0] = u8(v)
	p[1] = u8(v >> 8)
	p[2] = u8(v >> 16)
	p[3] = u8(v >> 24)
}

// Frame and append one record into the buffer. Caller holds rec->lock.
@(private = "file")
RecCommitRecordLocked :: proc(rec: ^Recording, opcode: u8, payload: [^]u8, payloadSize: i32) {
	assert(payloadSize >= 0 && payloadSize < (1 << 24))
	RecW_U8(&rec.buffer, opcode)
	sz := [3]u8{u8(payloadSize), u8(payloadSize >> 8), u8(payloadSize >> 16)}
	RecBufAppend(&rec.buffer, &sz, 3)
	RecBufAppend(&rec.buffer, payload, payloadSize)
}

RecCommitRecord :: proc(rec: ^Recording, opcode: u8, payload: [^]u8, payloadSize: i32) {
	LockMutex(rec.lock)
	RecCommitRecordLocked(rec, opcode, payload, payloadSize)
	UnlockMutex(rec.lock)
}

RecQueryBegin :: proc(w: ^RecQueryWriter, context_: rawptr, tagId: u64, tagName: cstring) {
	w.buf = RecBuffer{}
	w.userFcn.overlapFcn = nil
	w.userContext = context_
	w.hitCount = 0
	w.countOffset = 0
	w.tagId = tagId
	w.tagName = tagName
}

RecQueryCommit :: proc(rec: ^Recording, opcode: u8, w: ^RecQueryWriter) {
	LockMutex(rec.lock)
	// A tagged query writes its identity key right before the query record, under one lock so the pair
	// stays adjacent even with concurrent queries. The key is the hash of the caller (id, name), which
	// are interned once into the trailing tag table so the viewer can show them.
	tagged := w.tagId != 0 || (w.tagName != nil && ([^]u8)(w.tagName)[0] != 0)
	if tagged {
		key := HashQueryTag(w.tagId, w.tagName)
		RecInternTag(rec, key, w.tagId, w.tagName)
		tagBuf := RecBuffer{}
		RecW_U64(&tagBuf, key)
		RecCommitRecordLocked(rec, recOpQueryTag, tagBuf.data, tagBuf.size)
		RecBufFree(&tagBuf)
	}
	RecCommitRecordLocked(rec, opcode, w.buf.data, w.buf.size)
	UnlockMutex(rec.lock)
	RecBufFree(&w.buf)
}

// Recording trampolines: replace the user fcn so hits are captured before dispatch. The overlap
// trampoline doubles for the mover filter, which has the same bool(shapeId, ctx) shape.
// unified: C calling convention (the library's API is upstream's C ABI)
RecOverlapTrampoline :: proc "c" (id: ShapeId, ctx: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	w := (^RecQueryWriter)(ctx)
	// The user fcn is NULL for an unfiltered mover cast: accept all, still record the decision so
	// replay reproduces the same per-shape accept stream.
	ret := w.userFcn.overlapFcn != nil ? w.userFcn.overlapFcn(id, w.userContext) : true
	RecW_SHAPEID(&w.buf, id)
	RecW_BOOL(&w.buf, ret)
	w.hitCount += 1
	return ret
}

// unified: C calling convention (the library's API is upstream's C ABI)
RecCastTrampoline :: proc "c" (
	id: ShapeId,
	point: Pos,
	normal: Vec3,
	fraction: f32,
	userMaterialId: u64,
	triangleIndex: i32,
	childIndex: i32,
	ctx: rawptr,
) -> f32 {
	context = runtime.default_context()
	// unified end
	w := (^RecQueryWriter)(ctx)
	ret := w.userFcn.castFcn(id, point, normal, fraction, userMaterialId, triangleIndex, childIndex, w.userContext)
	RecW_SHAPEID(&w.buf, id)
	RecW_POSITION(&w.buf, point)
	RecW_VEC3(&w.buf, normal)
	RecW_F32(&w.buf, fraction)
	RecW_U64(&w.buf, userMaterialId)
	RecW_I32(&w.buf, triangleIndex)
	RecW_I32(&w.buf, childIndex)
	RecW_F32(&w.buf, ret)
	w.hitCount += 1
	return ret
}

// 3D delivers every plane for one shape in a single call. Record the shape, its plane count, each
// plane, then the user return so replay reproduces the same per-shape batch. One hit per shape.
// unified: C calling convention (the library's API is upstream's C ABI)
RecPlaneTrampoline :: proc "c" (id: ShapeId, planes: ^PlaneResult, planeCount: i32, ctx: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	w := (^RecQueryWriter)(ctx)
	ret := w.userFcn.planeFcn(id, planes, planeCount, w.userContext)
	RecW_SHAPEID(&w.buf, id)
	RecW_I32(&w.buf, planeCount)
	planeArray := ([^]PlaneResult)(planes)
	for i: i32 = 0; i < planeCount; i += 1 {
		RecW_PLANERESULT(&w.buf, planeArray[i])
	}
	RecW_BOOL(&w.buf, ret)
	w.hitCount += 1
	return ret
}

// Record framing

RecBeginRecord :: proc(rec: ^Recording, opcode: u8) {
	RecW_U8(&rec.buffer, opcode)
	rec.recordStart = rec.buffer.size
	// Reserve 3 bytes for the u24 payload size, backpatched in b3RecEndRecord.
	zero := [3]u8{0, 0, 0}
	RecBufAppend(&rec.buffer, &zero, 3)
}

RecEndRecord :: proc(rec: ^Recording) {
	payloadSize := rec.buffer.size - rec.recordStart - 3
	assert(payloadSize >= 0 && payloadSize < (1 << 24))
	p := rec.buffer.data[rec.recordStart:]
	p[0] = u8(payloadSize)
	p[1] = u8(payloadSize >> 8)
	p[2] = u8(payloadSize >> 16)
}

// unified: codegen passes 1b (arg writers), full writers and create-op writers are written out in
// recording_ops.odin.

// Geometry registry

// Content hash to chain head, so dedup is near O(1). Colliding hashes share a head and are walked
// through b3GeometryEntry::hashNext, so the byteCount + memcmp check always finds an existing blob.
// unified: the verstable maps (u64 -> u32) are Odin maps on the heap allocator; they are only used for lookups.
// Their storage is not counted by GetByteCount.
@(private = "file")
GeometryHashMap :: map[u64]u32

// Tag key to tag index, so interning a query tag is O(1) rather than a linear scan over the tag table.
@(private = "file")
RecTagMap :: map[u64]u32
// unified end

// Append a fresh entry and splice it onto the front of its hash chain. The map value is the chain head.
// unified: the verstable iterator is the chain head read from the map (head, valid when hashPresent)
@(private = "file")
RegistryPush :: proc(
	reg: ^GeometryRegistry,
	m: ^GeometryHashMap,
	head: u32,
	hashPresent: bool,
	kind: GeometryKind,
	contentHash: u64,
	bytes: [^]u8,
	byteCount: i32,
) -> u32 {
	HostScope() // unified: recordings are host memory (world_arena.odin)
	assert(byteCount > 0)
	copy := cast([^]u8)AllocHost(int(byteCount))
	mem.copy(copy, bytes, int(byteCount))

	id := u32(reg.entries.count)
	entry := Array_Emplace(&reg.entries)
	entry.contentHash = contentHash
	entry.id = id
	entry.kind = kind
	entry.byteCount = byteCount
	entry.bytes = copy
	entry.hashNext = hashPresent ? i32(head) : NULL_INDEX

	// unified: itr.data->val = id when present, insert otherwise: both are a map store
	m[contentHash] = id
	// unified end
	return id
}

@(private = "file")
RegistryMap :: proc(reg: ^GeometryRegistry) -> ^GeometryHashMap {
	if reg.dedupMap == nil {
		fresh := cast(^GeometryHashMap)AllocHost(size_of(GeometryHashMap))
		// unified: b3GeometryHashMap_init
		fresh^ = make(GeometryHashMap, runtime.heap_allocator())
		// unified end
		reg.dedupMap = fresh
	}
	return (^GeometryHashMap)(reg.dedupMap)
}

InternGeometry :: proc(
	reg: ^GeometryRegistry,
	kind: GeometryKind,
	contentHash: u64,
	bytes: [^]u8,
	byteCount: i32,
) -> u32 {
	HostScope() // unified: recordings are host memory (world_arena.odin)
	m := RegistryMap(reg)

	head, hashPresent := m[contentHash]
	if hashPresent {
		// Walk every entry sharing this hash so a collision still finds the identical blob.
		for index := i32(head); index != NULL_INDEX; index = reg.entries.data[index].hashNext {
			e := &reg.entries.data[index]
			if e.byteCount == byteCount && mem.compare_ptrs(e.bytes, bytes, int(byteCount)) == 0 {
				return e.id
			}
		}
	}

	return RegistryPush(reg, m, head, hashPresent, kind, contentHash, bytes, byteCount)
}

AppendGeometry :: proc(
	reg: ^GeometryRegistry,
	kind: GeometryKind,
	contentHash: u64,
	bytes: [^]u8,
	byteCount: i32,
) -> u32 {
	HostScope() // unified: recordings are host memory (world_arena.odin)
	m := RegistryMap(reg)
	head, hashPresent := m[contentHash]
	return RegistryPush(reg, m, head, hashPresent, kind, contentHash, bytes, byteCount)
}

FreeRegistry :: proc(reg: ^GeometryRegistry) {
	for i: i32 = 0; i < reg.entries.count; i += 1 {
		Free(reg.entries.data[i].bytes, int(reg.entries.data[i].byteCount))
	}

	Array_Destroy(&reg.entries)

	if reg.dedupMap != nil {
		// unified: b3GeometryHashMap_cleanup
		delete((^GeometryHashMap)(reg.dedupMap)^)
		// unified end
		Free(reg.dedupMap, size_of(GeometryHashMap))
	}
	reg.dedupMap = nil
}

HashQueryTag :: proc(id: u64, name: cstring) -> u64 {
	h := SNAP_FNV_INIT
	for i: u64 = 0; i < 8; i += 1 {
		h = (h ~ ((id >> (8 * i)) & 0xFF)) * SNAP_FNV_PRIME
	}
	if name != nil {
		s := ([^]u8)(name)
		for i := 0; s[i] != 0; i += 1 {
			h = (h ~ u64(s[i])) * SNAP_FNV_PRIME
		}
	}
	// Never 0 so the key doubles as the tagged flag.
	return h != 0 ? h : 1
}

@(private = "file")
RecTags :: proc(rec: ^Recording) -> ^RecTagMap {
	if rec.tagMap == nil {
		fresh := cast(^RecTagMap)AllocHost(size_of(RecTagMap))
		// unified: b3RecTagMap_init
		fresh^ = make(RecTagMap, runtime.heap_allocator())
		// unified end
		rec.tagMap = fresh
	}
	return (^RecTagMap)(rec.tagMap)
}

RecInternTag :: proc(rec: ^Recording, key: u64, id: u64, name: cstring) {
	m := RecTags(rec)
	if key in m {
		return // first id/name for a key wins
	}

	if rec.tagCount == rec.tagCapacity {
		newCap := rec.tagCapacity == 0 ? 8 : 2 * rec.tagCapacity
		rec.tags = cast([^]RecTag)GrowAllocHost(
			rec.tags,
			rec.tagCapacity * i32(size_of(RecTag)),
			newCap * i32(size_of(RecTag)),
		)
		rec.tagCapacity = newCap
	}

	index := u32(rec.tagCount)
	tag := &rec.tags[rec.tagCount]
	rec.tagCount += 1
	tag.key = key
	tag.id = id
	s := ([^]u8)(name)
	n := 0
	for name != nil && s[n] != 0 && n < MAX_QUERY_NAME_LENGTH {
		tag.queryName[n] = s[n]
		n += 1
	}
	tag.queryName[n] = 0
	m[key] = index
}

// Write the trailing registry block: u32 entryCount then per-entry { u8 kind, u32 byteCount, bytes },
// followed by the query-tag table { u32 tagCount, per-tag uu64 id, STR name }. A reader built before
// the tag table stops after the geometry entries and ignores the trailing tag bytes.
RecWriteRegistry :: proc(rec: ^Recording) {
	RecW_U32(&rec.buffer, u32(rec.registry.entries.count))
	for i: i32 = 0; i < rec.registry.entries.count; i += 1 {
		e := &rec.registry.entries.data[i]
		RecW_U8(&rec.buffer, u8(e.kind))
		RecW_U32(&rec.buffer, u32(e.byteCount))
		RecBufAppend(&rec.buffer, e.bytes, e.byteCount)
	}

	RecW_U32(&rec.buffer, u32(rec.tagCount))
	for i: i32 = 0; i < rec.tagCount; i += 1 {
		RecW_U64(&rec.buffer, rec.tags[i].key)
		RecW_U64(&rec.buffer, rec.tags[i].id)
		RecW_STR(&rec.buffer, cstring(&rec.tags[i].queryName[0]))
	}
}

// Lifecycle

CreateRecording :: proc(byteCapacity: i32) -> ^Recording {
	rec := cast(^Recording)AllocHost(size_of(Recording))
	rec^ = Recording{}

	initCap := byteCapacity > 0 ? byteCapacity : 65536
	rec.buffer.data = cast([^]u8)AllocHost(int(initCap))
	rec.buffer.capacity = initCap
	rec.buffer.size = 0
	rec.lock = CreateMutex()
	return rec
}

DestroyRecording :: proc(recording: ^Recording) {
	if recording == nil {
		return
	}

	RecBufFree(&recording.buffer)
	FreeRegistry(&recording.registry)
	if recording.tags != nil {
		Free(recording.tags, int(recording.tagCapacity) * size_of(RecTag))
	}
	if recording.tagMap != nil {
		// unified: b3RecTagMap_cleanup
		delete((^RecTagMap)(recording.tagMap)^)
		// unified end
		Free(recording.tagMap, size_of(RecTagMap))
	}
	DestroyMutex(recording.lock)
	Free(recording, size_of(Recording))
}

Recording_GetData :: proc(recording: ^Recording) -> [^]u8 {
	return recording.buffer.data
}

Recording_GetSize :: proc(recording: ^Recording) -> i32 {
	return recording.buffer.size
}

RecAccumulateBounds :: proc(rec: ^Recording, bounds: AABB) {
	rec.accumulatedBounds = rec.haveBounds ? AABB_Union(rec.accumulatedBounds, bounds) : bounds
	rec.haveBounds = true
}

StartRecordingIntoBuffer :: proc(world: ^World, recording: ^Recording) {
	// Reset so a recording handle can be reused for a fresh session
	recording.buffer.size = 0
	recording.recordStart = 0
	recording.haveBounds = false
	FreeRegistry(&recording.registry)
	if recording.tags != nil {
		Free(recording.tags, int(recording.tagCapacity) * size_of(RecTag))
		recording.tags = nil
	}
	if recording.tagMap != nil {
		// unified: b3RecTagMap_cleanup
		delete((^RecTagMap)(recording.tagMap)^)
		// unified end
		Free(recording.tagMap, size_of(RecTagMap))
		recording.tagMap = nil
	}
	recording.tagCount = 0
	recording.tagCapacity = 0

	hdr := RecHeader{}
	hdr.magic = REC_MAGIC
	hdr.versionMajor = REC_VERSION_MAJOR
	hdr.versionMinor = REC_VERSION_MINOR
	hdr.pointerWidth = u8(size_of(rawptr))
	hdr.bigEndian = 0
	hdr.validationEnabled = ENABLE_VALIDATION ? 1 : 0
	hdr.lengthScale = GetLengthUnitsPerMeter()
	hdr.registryOffset = 0 // backpatched in b3StopRecordingInternal
	hdr.registryByteCount = 0

	world.recording = recording

	// Every recording is snapshot-seeded. The seed blob follows the header so replay restores in
	// place and the world id stays stable across a restart or backward scrub. An empty world still
	// serializes a valid blob, so there is no from-creation special case.
	snapBuf := RecBuffer{}
	SerializeWorld(world, &snapBuf, recording)
	hdr.snapshotSize = u64(snapBuf.size)

	RecBufAppend(&recording.buffer, &hdr, i32(size_of(hdr)))
	RecBufAppend(&recording.buffer, snapBuf.data, snapBuf.size)
	RecBufFree(&snapBuf)

	// Anchor the recording with the current world state hash so replay can assert
	// determinism from the very first step.
	worldId := WorldId{u16(world.worldId + 1), world.generation}
	stateHash := RecArgs_StateHash{worldId, HashWorldState(world)}
	RecWrite_StateHash(recording, &stateHash)
}

StopRecordingInternal :: proc(world: ^World) {
	if world.recording == nil {
		return
	}

	rec := world.recording
	world.recording = nil

	// Write accumulated bounds so a viewer can frame the whole recorded motion
	rb := RecArgs_RecordingBounds{}
	if rec.haveBounds {
		rb.bounds = rec.accumulatedBounds
	}
	RecWrite_RecordingBounds(rec, &rb)

	// End-of-stream marker; the buffer is now self-contained
	wid := WorldId{u16(world.worldId + 1), world.generation}
	a := RecArgs_DestroyWorld{wid}
	RecWrite_DestroyWorld(rec, &a)

	// Write the trailing registry block
	registryOffset := rec.buffer.size
	RecWriteRegistry(rec)
	registryByteCount := rec.buffer.size - registryOffset

	// Backpatch registryOffset and registryByteCount into the header
	hdrBytes := rec.buffer.data
	regOff := u64(registryOffset)
	regSz := u64(registryByteCount)
	// Little-endian backpatch in place; offsetof keeps this correct if the header layout shifts
	pOff := hdrBytes[offset_of(RecHeader, registryOffset):]
	pSz := hdrBytes[offset_of(RecHeader, registryByteCount):]
	for i: u64 = 0; i < 8; i += 1 {
		pOff[i] = u8(regOff >> (8 * i))
		pSz[i] = u8(regSz >> (8 * i))
	}
}

// Convenience file I/O

SaveRecordingToFile :: proc(recording: ^Recording, path: cstring) -> bool {
	if recording == nil || path == nil {
		return false
	}

	// unified: core:os instead of stdio (fopen/fwrite/fclose)
	f, err := os.create(string(path))
	if err != nil {
		return false
	}

	written, _ := os.write(f, recording.buffer.data[:recording.buffer.size])
	os.close(f)
	return i32(written) == recording.buffer.size
	// unified end
}

LoadRecordingFromFile :: proc(path: cstring) -> ^Recording {
	if path == nil {
		return nil
	}

	// unified: core:os instead of stdio (fopen/fseek/ftell/fread/fclose)
	f, err := os.open(string(path))
	if err != nil {
		return nil
	}

	fileSize, seekErr := os.seek(f, 0, .End)
	if seekErr != nil {
		os.close(f)
		return nil
	}

	// Anything smaller than the fixed header can't be a recording
	if fileSize < i64(size_of(RecHeader)) || fileSize > i64(max(i32)) {
		os.close(f)
		return nil
	}
	_, _ = os.seek(f, 0, .Start)

	rec := CreateRecording(i32(fileSize))
	readSize, _ := os.read_full(f, rec.buffer.data[:fileSize])
	os.close(f)

	if i64(readSize) != fileSize {
		DestroyRecording(rec)
		return nil
	}
	// unified end

	// Validate magic so a wrong file fails at load rather than deep in the player
	hdr: RecHeader
	mem.copy(&hdr, rec.buffer.data, size_of(hdr))
	if hdr.magic != REC_MAGIC {
		DestroyRecording(rec)
		return nil
	}

	rec.buffer.size = i32(fileSize)
	return rec
}

// Geometry interning helpers

@(private = "file")
RecInternBlob :: proc(rec: ^Recording, kind: GeometryKind, blob: rawptr, byteCount: i32) -> u32 {
	bytes := cast([^]u8)blob
	h := Hash64NonZero(bytes, byteCount)
	return InternGeometry(&rec.registry, kind, h, bytes, byteCount)
}

RecInternHull :: proc(rec: ^Recording, hull: ^HullData) -> u32 {
	return RecInternBlob(rec, .geometryHull, hull, hull.byteCount)
}

RecInternMesh :: proc(rec: ^Recording, mesh: ^MeshData) -> u32 {
	return RecInternBlob(rec, .geometryMesh, mesh, mesh.byteCount)
}

RecInternHeightField :: proc(rec: ^Recording, hf: ^HeightFieldData) -> u32 {
	return RecInternBlob(rec, .geometryHeightField, hf, hf.byteCount)
}

RecInternCompound :: proc(rec: ^Recording, compound: ^CompoundData) -> u32 {
	return RecInternBlob(rec, .geometryCompound, compound, compound.byteCount)
}

// Deterministic hash over all body transforms and velocities.
// Called by both recorder and replayer to verify simulation reproduces exactly.
HashWorldState :: proc(world: ^World) -> u64 {
	hash := SNAP_FNV_INIT
	prime := SNAP_FNV_PRIME

	bodyCount := world.bodies.count
	for i: i32 = 0; i < bodyCount; i += 1 {
		body := &world.bodies.data[i]
		if body.id != i {
			// Free or never-used slot
			continue
		}

		sim := GetBodySim(world, body)

		// unified: B3_HASH_FLOAT( f ) is a local proc literal
		HASH_FLOAT :: #force_inline proc(hash: ^u64, f: f32, prime: u64) {
			bits := transmute(u32)f
			hash^ = (hash^ ~ u64(bits)) * prime
		}
		// unified end

		hash = FnvMixPosition(hash, sim.transform.p)
		HASH_FLOAT(&hash, sim.transform.q.v.x, prime)
		HASH_FLOAT(&hash, sim.transform.q.v.y, prime)
		HASH_FLOAT(&hash, sim.transform.q.v.z, prime)
		HASH_FLOAT(&hash, sim.transform.q.s, prime)

		state := GetBodyState(world, body)
		if state != nil {
			HASH_FLOAT(&hash, state.linearVelocity.x, prime)
			HASH_FLOAT(&hash, state.linearVelocity.y, prime)
			HASH_FLOAT(&hash, state.linearVelocity.z, prime)
			HASH_FLOAT(&hash, state.angularVelocity.x, prime)
			HASH_FLOAT(&hash, state.angularVelocity.y, prime)
			HASH_FLOAT(&hash, state.angularVelocity.z, prime)
		}
	}

	return hash
}
