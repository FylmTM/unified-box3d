// Ported from box3d src/replay.c and src/replay.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:runtime"
import "core:fmt"
import "core:mem"

// A single recorded callback hit, used both as reader scratch during a query replay and as the
// per-frame draw store. For collide-mover, one hit is one plane, with planeCount and userReturnB
// replicated across a shape's planes so the replay walker can re-group and compare per shape.
RecRecordedHit :: struct {
	id:             ShapeId,
	point:          Pos,
	normal:         Vec3,
	fraction:       f32,
	userMaterialId: u64,
	triangleIndex:  i32,
	childIndex:     i32,
	plane:          PlaneResult, // collide-mover: this plane
	planeCount:     i32, // collide-mover: planes in this hit's shape group (replicated)
	userReturnF:    f32, // cast queries
	userReturnB:    bool, // overlap / collide-mover (per shape, replicated)
}

// Recorded query kind, matching the public b3RecQueryType order.
RecQueryKind :: enum i32 {
	RECQ_OVERLAP_AABB,
	RECQ_OVERLAP_SHAPE,
	RECQ_CAST_RAY,
	RECQ_CAST_SHAPE,
	RECQ_CAST_RAY_CLOSEST,
	RECQ_CAST_MOVER,
	RECQ_COLLIDE_MOVER,
}

// Per-frame draw record for one query call. Self-contained (no aliased pointers) so the player's
// frameQueries array can grow with a plain memcpy. Geometry is origin relative except the overlap
// AABB, which is world space in 3D.
RecDrawQuery :: struct {
	kind:         i32,
	key:          u64, // identity key (hash of caller id+name), 0 = untagged
	filter:       QueryFilter,
	aabb:         AABB, // world-space bounds of the query, swept for casts
	proxyPoints:  [MAX_SHAPE_CAST_POINTS]Vec3, // overlap/cast shape proxy, origin relative
	proxyCount:   i32,
	proxyRadius:  f32,
	mover:        Capsule, // cast/collide mover, origin relative
	origin:       Pos,
	translation:  Vec3,
	castFraction: f32, // cast-mover result fraction
	rayResult:    RayResult, // cast-ray-closest result
	shape:        ShapeId,
	hitStart:     i32, // first hit in the player's frameHits store
	hitCount:     i32,
}

// One slot in the preloaded geometry registry. Loaded from the trailing block before any
// ops run.
RegistrySlot :: struct {
	kind:      GeometryKind,
	byteCount: i32,
	bytes:     [^]u8, // raw bytes from the file (always freed at teardown)
}

// This is used to simplify the scratch buffer lifetime. Names longer than this probably
// indicate a bug.
MAX_NAME_LENGTH :: 256

// Reader state threaded through the replay loop and all dispatch functions
RecReader :: struct {
	data:            [^]u8,
	size:            i32,
	cursor:          i32,
	replayWorldId:   WorldId,
	ok:              bool, // false on read overrun or id mismatch, fatal stop
	diverged:        bool, // a StateHash failed, non-fatal

	// Player that owns this reader, or NULL during a headless b3ValidateReplay. Body
	// create/destroy and the bounds record fold back into it for the outliner and camera framing.
	owner:           ^RecPlayer,

	// Scratch for per-triangle materials in shape defs (grown on demand, freed at teardown)
	matScratch:      [^]SurfaceMaterial,
	matScratchCap:   i32,

	// Scratch for string reads. Used by b3RecR_STR to pass names to b3BodyDef and b3ShapeDef.
	stringBuffers:   [4][MAX_NAME_LENGTH + 1]u8,
	nextString:      i32,

	// Preloaded geometry registry
	slots:           [^]RegistrySlot,
	slotCount:       i32,

	// Preloaded query-tag table (key -> id, name), loaded with the registry. Resolves the caller id and
	// label for the viewer. tagMap maps a key to its tag index for O(1) lookup; opaque, owned by the player.
	tags:            [^]RecTag,
	tagCount:        i32, // tags that loaded; a truncated tail loads fewer
	tagCapacity:     i32, // tags allocated, used to free the array
	tagMap:          rawptr,

	// Key from the QueryTag op preceding the next query, consumed by the next stash. 0 = untagged.
	pendingQueryKey: u64,

	// Scratch for recorded query hits; grown on demand, freed with the player.
	hits:            [^]RecRecordedHit,
	hitCap:          i32,

	// Scratch for a shape-proxy point cloud read from the stream. b3ShapeProxy holds the points
	// behind a pointer, so a decoded proxy borrows this until the next proxy read or teardown.
	proxyScratch:    [^]Vec3,
	proxyScratchCap: i32,
}

// Stored snapshot for fast backward seek.
RecKeyframe :: struct {
	image:         [^]u8, // serialized world image at the end of this frame
	imageSize:     i32,
	imageCapacity: i32, // allocation size (may exceed imageSize)
	frame:         i32, // frame index this restores to
	cursor:        i32, // op-stream cursor for the frame AFTER this one
	divergeFrame:  i32, // divergeFrame state at capture
	diverged:      bool, // rdr.diverged state at capture

	// Outliner body list as it stood at this frame, restored verbatim so ordinals are stable.
	bodyIds:       [^]BodyId,
	bodyIdCount:   i32,
}

RecPlayer :: struct {
	data:                 [^]u8, // owned copy of recording bytes
	size:                 i32,
	headerEnd:            i32, // first byte of op stream (past header + snapshot blob)
	registryEnd:          i32, // end of op stream = start of registry block (or size)
	lengthScale:          f32,
	previousLengthScale:  f32,
	frame:                i32,
	frameCount:           i32,
	recordedDt:           f32,
	recordedSubStepCount: i32,
	recordedWorkerCount:  i32, // worker count requested for the replay world
	bounds:               AABB, // accumulated world bounds, decoded from the trailing record
	atEnd:                bool,
	// Indicates all ops for the step have been consumed up to the world step op. The next sub-step
	// will clear this and perform the world step.
	atPreStep:            bool,
	divergeFrame:         i32, // first frame that diverged, -1 until then

	// Outliner body list, indexed by creation ordinal. Holes (null ids) mark destroyed bodies so
	// later ordinals never shift. Snapshotted into each keyframe and the frame-0 copy, not rebuilt
	// from the world, so a stored selection survives backward seeks.
	bodyIds:              [^]BodyId,
	bodyIdCount:          i32,
	bodyIdCap:            i32,
	frame0BodyIds:        [^]BodyId,
	frame0BodyIdCount:    i32,

	// Per-frame query store, reset at the top of each StepFrame and filled by the query dispatchers.
	// Drawn by b3RecPlayer_DrawFrameQueries and inspected via the public GetFrameQuery API.
	frameQueries:         [^]RecDrawQuery,
	frameQueryCount:      i32,
	frameQueryCap:        i32,
	frameHits:            [^]RecRecordedHit,
	frameHitCount:        i32,
	frameHitCap:          i32,

	// Host debug-shape callbacks applied to every world the player creates. The 3D
	// sample renderer builds GPU meshes here, so a replay world without them draws
	// nothing. Set once via b3RecPlayer_SetDebugShapeCallbacks; persisted so a world
	// rebuilt under new callbacks keeps drawing.
	createDebugShape:     CreateDebugShapeCallback,
	destroyDebugShape:    DestroyDebugShapeCallback,
	debugShapeContext:    rawptr,
	rdr:                  RecReader,

	// Frame-0 restore image, points into the owned data copy. Restart and backward seek
	// deserialize this in place so the replay world id stays stable.
	frame0Image:          [^]u8,
	frame0Size:           i32,

	// Keyframe ring
	keyframes:            [^]RecKeyframe,
	keyframeCount:        i32,
	keyframeCapacity:     i32,
	keyframeBudget:       int,
	keyframeBytes:        int,
	keyframeMinInterval:  i32,
	keyframeInterval:     i32,
	lastKeyframeFrame:    i32,

	// Pre-populated recording used by b3SerializeWorld during keyframe capture.
	// Its registry mirrors rdr.slots so geometry ids stay stable.
	keyframeRec:          ^Recording,
}

// Read primitives

@(private = "file")
RecRdrCheck :: proc(rdr: ^RecReader, size: i32) {
	if size < 0 || i64(rdr.cursor) + i64(size) > i64(rdr.size) {
		rdr.ok = false
	}
}

@(private = "file")
RecRdrBlob :: proc(rdr: ^RecReader, out: rawptr, size: i32) {
	RecRdrCheck(rdr, size)
	if !rdr.ok {
		mem.zero(out, int(size))
		return
	}
	mem.copy(out, &rdr.data[rdr.cursor], int(size))
	rdr.cursor += size
}

RecR_U8 :: proc(rdr: ^RecReader) -> u8 {
	RecRdrCheck(rdr, 1)
	if !rdr.ok {
		return 0
	}
	v := rdr.data[rdr.cursor]
	rdr.cursor += 1
	return v
}

RecR_U16 :: proc(rdr: ^RecReader) -> u16 {
	RecRdrCheck(rdr, 2)
	if !rdr.ok {
		return 0
	}
	v := u16(rdr.data[rdr.cursor]) | (u16(rdr.data[rdr.cursor + 1]) << 8)
	rdr.cursor += 2
	return v
}

RecR_U24 :: proc(rdr: ^RecReader) -> u32 {
	RecRdrCheck(rdr, 3)
	if !rdr.ok {
		return 0
	}
	v := u32(rdr.data[rdr.cursor]) | (u32(rdr.data[rdr.cursor + 1]) << 8) | (u32(rdr.data[rdr.cursor + 2]) << 16)
	rdr.cursor += 3
	return v
}

RecR_U32 :: proc(rdr: ^RecReader) -> u32 {
	RecRdrCheck(rdr, 4)
	if !rdr.ok {
		return 0
	}
	v :=
		u32(rdr.data[rdr.cursor]) |
		(u32(rdr.data[rdr.cursor + 1]) << 8) |
		(u32(rdr.data[rdr.cursor + 2]) << 16) |
		(u32(rdr.data[rdr.cursor + 3]) << 24)
	rdr.cursor += 4
	return v
}

RecR_U64 :: proc(rdr: ^RecReader) -> u64 {
	RecRdrCheck(rdr, 8)
	if !rdr.ok {
		return 0
	}
	v :=
		u64(rdr.data[rdr.cursor]) |
		(u64(rdr.data[rdr.cursor + 1]) << 8) |
		(u64(rdr.data[rdr.cursor + 2]) << 16) |
		(u64(rdr.data[rdr.cursor + 3]) << 24) |
		(u64(rdr.data[rdr.cursor + 4]) << 32) |
		(u64(rdr.data[rdr.cursor + 5]) << 40) |
		(u64(rdr.data[rdr.cursor + 6]) << 48) |
		(u64(rdr.data[rdr.cursor + 7]) << 56)
	rdr.cursor += 8
	return v
}

RecR_I32 :: proc(rdr: ^RecReader) -> i32 {
	return i32(RecR_U32(rdr))
}

RecR_F32 :: proc(rdr: ^RecReader) -> f32 {
	bits := RecR_U32(rdr)
	return transmute(f32)bits
}

RecR_F64 :: proc(rdr: ^RecReader) -> f64 {
	bits := RecR_U64(rdr)
	return transmute(f64)bits
}

RecR_BOOL :: proc(rdr: ^RecReader) -> bool {
	return RecR_U8(rdr) != 0
}

RecR_VEC3 :: proc(rdr: ^RecReader) -> Vec3 {
	v: Vec3
	v.x = RecR_F32(rdr)
	v.y = RecR_F32(rdr)
	v.z = RecR_F32(rdr)
	return v
}

RecR_QUAT :: proc(rdr: ^RecReader) -> Quat {
	q: Quat
	q.v.x = RecR_F32(rdr)
	q.v.y = RecR_F32(rdr)
	q.v.z = RecR_F32(rdr)
	q.s = RecR_F32(rdr)
	return q
}

RecR_TRANSFORM :: proc(rdr: ^RecReader) -> Transform {
	t: Transform
	t.p = RecR_VEC3(rdr)
	t.q = RecR_QUAT(rdr)
	return t
}

RecR_POSITION :: proc(rdr: ^RecReader) -> Pos {
	p: Pos
	// unified: single precision only: the BOX3D_DOUBLE_PRECISION branch is not ported.
	p.x = RecR_F32(rdr)
	p.y = RecR_F32(rdr)
	p.z = RecR_F32(rdr)
	return p
}

RecR_WORLDXF :: proc(rdr: ^RecReader) -> WorldTransform {
	t: WorldTransform
	t.p = RecR_POSITION(rdr)
	t.q = RecR_QUAT(rdr)
	return t
}

RecR_MATRIX3 :: proc(rdr: ^RecReader) -> Matrix3 {
	m: Matrix3
	m.cx = RecR_VEC3(rdr)
	m.cy = RecR_VEC3(rdr)
	m.cz = RecR_VEC3(rdr)
	return m
}

RecR_AABB :: proc(rdr: ^RecReader) -> AABB {
	v: AABB
	v.lowerBound = RecR_VEC3(rdr)
	v.upperBound = RecR_VEC3(rdr)
	return v
}

RecR_QUERYFILTER :: proc(rdr: ^RecReader) -> QueryFilter {
	// id and name are not on the wire here, they ride the separate QueryTag op. Start from the default
	// so they keep the untagged sentinel instead of garbage.
	f := DefaultQueryFilter()
	f.categoryBits = RecR_U64(rdr)
	f.maskBits = RecR_U64(rdr)
	return f
}

// Reserve reader scratch for a count taken from an untrusted file. Every recorded element
// consumes at least one byte, so a valid count can never exceed the bytes left in the file.
// Reject anything larger (or negative, or that would overflow the byte size) by failing the read
// rather than allocating wildly. A grow keeps the old contents so callers can accumulate across
// reserves, as the collide-mover dispatcher does one shape group at a time.
@(private = "file")
RecReserveScratch :: proc(rdr: ^RecReader, data: ^rawptr, cap: ^i32, need: i32, elemSize: i32) -> bool {
	remaining := rdr.size - rdr.cursor
	if need < 0 || remaining < 0 || need > remaining || need > max(i32) / elemSize {
		rdr.ok = false
		return false
	}
	if need <= cap^ {
		return true
	}
	newCap := need <= max(i32) / elemSize - 8 ? need + 8 : need
	grown := AllocHost(int(newCap) * int(elemSize))
	if data^ != nil {
		mem.copy(grown, data^, int(cap^) * int(elemSize))
		Free(data^, int(cap^) * int(elemSize))
	}
	data^ = grown
	cap^ = newCap
	return true
}

// Variable length, mirrors b3RecW_SHAPEPROXY: count, count points, radius. The decoded proxy borrows
// the reader's scratch for its point cloud, valid until the next proxy read.
RecR_SHAPEPROXY :: proc(rdr: ^RecReader) -> ShapeProxy {
	p := ShapeProxy{}
	count := RecR_I32(rdr)
	if count < 0 {
		count = 0
	}
	if count > MAX_SHAPE_CAST_POINTS {
		count = MAX_SHAPE_CAST_POINTS
	}
	if count > 0 &&
	   RecReserveScratch(rdr, (^rawptr)(&rdr.proxyScratch), &rdr.proxyScratchCap, count, i32(size_of(Vec3))) {
		for i: i32 = 0; i < count; i += 1 {
			rdr.proxyScratch[i] = RecR_VEC3(rdr)
		}
		p.points = rdr.proxyScratch
		p.count = count
	}
	p.radius = RecR_F32(rdr)
	return p
}

RecR_TREESTATS :: proc(rdr: ^RecReader) -> TreeStats {
	v: TreeStats
	v.nodeVisits = RecR_I32(rdr)
	v.leafVisits = RecR_I32(rdr)
	return v
}

RecR_RAYRESULT :: proc(rdr: ^RecReader) -> RayResult {
	v := RayResult{}
	// shapeId keeps the recorded world0; b3RecMakeShapeId is applied at compare time
	v.shapeId = RecR_SHAPEID(rdr)
	v.point = RecR_POSITION(rdr)
	v.normal = RecR_VEC3(rdr)
	v.userMaterialId = RecR_U64(rdr)
	v.fraction = RecR_F32(rdr)
	v.triangleIndex = RecR_I32(rdr)
	v.childIndex = RecR_I32(rdr)
	v.hit = RecR_BOOL(rdr)
	return v
}

RecR_PLANERESULT :: proc(rdr: ^RecReader) -> PlaneResult {
	v: PlaneResult
	v.plane.normal = RecR_VEC3(rdr)
	v.plane.offset = RecR_F32(rdr)
	v.point = RecR_VEC3(rdr)
	v.triangleIndex = RecR_I32(rdr)
	v.childIndex = RecR_I32(rdr)
	v.materialIndex = RecR_I32(rdr)
	return v
}

RecR_WORLDID :: proc(rdr: ^RecReader) -> WorldId {
	return LoadWorldId(RecR_U32(rdr))
}

RecR_BODYID :: proc(rdr: ^RecReader) -> BodyId {
	return LoadBodyId(RecR_U64(rdr))
}

RecR_SHAPEID :: proc(rdr: ^RecReader) -> ShapeId {
	return LoadShapeId(RecR_U64(rdr))
}

RecR_JOINTID :: proc(rdr: ^RecReader) -> JointId {
	return LoadJointId(RecR_U64(rdr))
}

RecR_SPHERE :: proc(rdr: ^RecReader) -> Sphere {
	s: Sphere
	RecRdrBlob(rdr, &s, i32(size_of(s)))
	return s
}

RecR_CAPSULE :: proc(rdr: ^RecReader) -> Capsule {
	c: Capsule
	RecRdrBlob(rdr, &c, i32(size_of(c)))
	return c
}

RecR_GEOMID :: proc(rdr: ^RecReader) -> u32 {
	return RecR_U32(rdr)
}

RecR_FILTER :: proc(rdr: ^RecReader) -> Filter {
	f: Filter
	f.categoryBits = RecR_U64(rdr)
	f.maskBits = RecR_U64(rdr)
	f.groupIndex = RecR_I32(rdr)
	return f
}

RecR_MATERIAL :: proc(rdr: ^RecReader) -> SurfaceMaterial {
	m := DefaultSurfaceMaterial()
	m.friction = RecR_F32(rdr)
	m.restitution = RecR_F32(rdr)
	m.rollingResistance = RecR_F32(rdr)
	m.tangentVelocity = RecR_VEC3(rdr)
	m.userMaterialId = RecR_U64(rdr)
	m.customColor = RecR_U32(rdr)
	return m
}

RecR_MASSDATA :: proc(rdr: ^RecReader) -> MassData {
	md: MassData
	md.mass = RecR_F32(rdr)
	md.center = RecR_VEC3(rdr)
	md.inertia = RecR_MATRIX3(rdr)
	return md
}

RecR_LOCKS :: proc(rdr: ^RecReader) -> MotionLocks {
	locks: MotionLocks
	locks.linearX = RecR_BOOL(rdr)
	locks.linearY = RecR_BOOL(rdr)
	locks.linearZ = RecR_BOOL(rdr)
	locks.angularX = RecR_BOOL(rdr)
	locks.angularY = RecR_BOOL(rdr)
	locks.angularZ = RecR_BOOL(rdr)
	return locks
}

// Rotating set of static string buffers, valid until the next 4 STR reads.
RecR_STR :: proc(rdr: ^RecReader) -> cstring {
	buf := &rdr.stringBuffers[rdr.nextString]
	rdr.nextString = (rdr.nextString + 1) & 3

	len := RecR_U16(rdr)
	if len == 0xFFFF {
		return nil
	}

	n := i32(len)
	if n > MAX_NAME_LENGTH {
		n = MAX_NAME_LENGTH
	}
	RecRdrCheck(rdr, i32(len))
	if rdr.ok && n > 0 {
		mem.copy(&buf[0], &rdr.data[rdr.cursor], int(n))
	}
	rdr.cursor += i32(len)
	buf[n] = 0
	return cstring(&buf[0])
}

// Def readers: start from b3Default*Def() then overlay each serialized field
// in the exact order the writer produced them.

RecR_EXPLOSIONDEF :: proc(rdr: ^RecReader) -> ExplosionDef {
	def := DefaultExplosionDef()
	def.maskBits = RecR_U64(rdr)
	def.position = RecR_POSITION(rdr)
	def.radius = RecR_F32(rdr)
	def.falloff = RecR_F32(rdr)
	def.impulsePerArea = RecR_F32(rdr)
	return def
}

RecR_BODYDEF :: proc(rdr: ^RecReader) -> BodyDef {
	def := DefaultBodyDef()
	def.type = BodyType(RecR_I32(rdr))
	def.position = RecR_POSITION(rdr)
	def.rotation = RecR_QUAT(rdr)
	def.linearVelocity = RecR_VEC3(rdr)
	def.angularVelocity = RecR_VEC3(rdr)
	def.linearDamping = RecR_F32(rdr)
	def.angularDamping = RecR_F32(rdr)
	def.gravityScale = RecR_F32(rdr)
	def.sleepThreshold = RecR_F32(rdr)
	def.safetyFactor = RecR_F32(rdr)
	def.name = RecR_STR(rdr)
	_ = RecR_U64(rdr) // userData placeholder
	def.motionLocks = RecR_LOCKS(rdr)
	def.enableSleep = RecR_BOOL(rdr)
	def.isAwake = RecR_BOOL(rdr)
	def.isBullet = RecR_BOOL(rdr)
	def.isEnabled = RecR_BOOL(rdr)
	def.allowFastRotation = RecR_BOOL(rdr)
	def.enableContactRecycling = RecR_BOOL(rdr)
	def.userData = nil
	return def
}

RecR_SHAPEDEF :: proc(rdr: ^RecReader) -> ShapeDef {
	def := DefaultShapeDef()

	def.name = RecR_STR(rdr)
	_ = RecR_U64(rdr) // userData placeholder

	matCount := RecR_I32(rdr)
	if matCount < 0 {
		matCount = 0
	}
	if matCount > 0 &&
	   RecReserveScratch(
		   rdr,
		   (^rawptr)(&rdr.matScratch),
		   &rdr.matScratchCap,
		   matCount,
		   i32(size_of(SurfaceMaterial)),
	   ) {
		for i: i32 = 0; i < matCount; i += 1 {
			rdr.matScratch[i] = RecR_MATERIAL(rdr)
		}
		def.materials = rdr.matScratch
		def.materialCount = matCount
	} else {
		for i: i32 = 0; i < matCount; i += 1 {
			_ = RecR_MATERIAL(rdr)
		}
		def.materials = nil
		def.materialCount = 0
	}

	def.baseMaterial = RecR_MATERIAL(rdr)
	def.density = RecR_F32(rdr)
	def.explosionScale = RecR_F32(rdr)
	def.filter = RecR_FILTER(rdr)
	def.enableCustomFiltering = RecR_BOOL(rdr)
	def.isSensor = RecR_BOOL(rdr)
	def.enableSensorEvents = RecR_BOOL(rdr)
	def.enableContactEvents = RecR_BOOL(rdr)
	def.enableHitEvents = RecR_BOOL(rdr)
	def.enablePreSolveEvents = RecR_BOOL(rdr)
	def.invokeContactCreation = RecR_BOOL(rdr)
	def.updateBodyMass = RecR_BOOL(rdr)
	def.enableSpeculativeContact = RecR_BOOL(rdr)
	def.userData = nil
	return def
}

// Shared base for all joint defs. Body ids come in with recorded world0; callers remap them.
@(private = "file")
RecR_JointBase :: proc(rdr: ^RecReader, base: ^JointDef) {
	_ = RecR_U64(rdr) // userData
	base.bodyIdA = RecR_BODYID(rdr)
	base.bodyIdB = RecR_BODYID(rdr)
	base.localFrameA = RecR_TRANSFORM(rdr)
	base.localFrameB = RecR_TRANSFORM(rdr)
	base.forceThreshold = RecR_F32(rdr)
	base.torqueThreshold = RecR_F32(rdr)
	base.constraintHertz = RecR_F32(rdr)
	base.constraintDampingRatio = RecR_F32(rdr)
	base.drawScale = RecR_F32(rdr)
	base.collideConnected = RecR_BOOL(rdr)
	base.userData = nil
}

RecR_PARALLELJOINTDEF :: proc(rdr: ^RecReader) -> ParallelJointDef {
	def := DefaultParallelJointDef()
	RecR_JointBase(rdr, &def.base)
	def.hertz = RecR_F32(rdr)
	def.dampingRatio = RecR_F32(rdr)
	def.maxTorque = RecR_F32(rdr)
	return def
}

RecR_DISTANCEJOINTDEF :: proc(rdr: ^RecReader) -> DistanceJointDef {
	def := DefaultDistanceJointDef()
	RecR_JointBase(rdr, &def.base)
	def.length = RecR_F32(rdr)
	def.enableSpring = RecR_BOOL(rdr)
	def.lowerSpringForce = RecR_F32(rdr)
	def.upperSpringForce = RecR_F32(rdr)
	def.hertz = RecR_F32(rdr)
	def.dampingRatio = RecR_F32(rdr)
	def.enableLimit = RecR_BOOL(rdr)
	def.minLength = RecR_F32(rdr)
	def.maxLength = RecR_F32(rdr)
	def.enableMotor = RecR_BOOL(rdr)
	def.maxMotorForce = RecR_F32(rdr)
	def.motorSpeed = RecR_F32(rdr)
	return def
}

RecR_FILTERJOINTDEF :: proc(rdr: ^RecReader) -> FilterJointDef {
	def := DefaultFilterJointDef()
	RecR_JointBase(rdr, &def.base)
	return def
}

RecR_MOTORJOINTDEF :: proc(rdr: ^RecReader) -> MotorJointDef {
	def := DefaultMotorJointDef()
	RecR_JointBase(rdr, &def.base)
	def.linearVelocity = RecR_VEC3(rdr)
	def.maxVelocityForce = RecR_F32(rdr)
	def.angularVelocity = RecR_VEC3(rdr)
	def.maxVelocityTorque = RecR_F32(rdr)
	def.linearHertz = RecR_F32(rdr)
	def.linearDampingRatio = RecR_F32(rdr)
	def.maxSpringForce = RecR_F32(rdr)
	def.angularHertz = RecR_F32(rdr)
	def.angularDampingRatio = RecR_F32(rdr)
	def.maxSpringTorque = RecR_F32(rdr)
	return def
}

RecR_PRISMATICJOINTDEF :: proc(rdr: ^RecReader) -> PrismaticJointDef {
	def := DefaultPrismaticJointDef()
	RecR_JointBase(rdr, &def.base)
	def.enableSpring = RecR_BOOL(rdr)
	def.hertz = RecR_F32(rdr)
	def.dampingRatio = RecR_F32(rdr)
	def.targetTranslation = RecR_F32(rdr)
	def.enableLimit = RecR_BOOL(rdr)
	def.lowerTranslation = RecR_F32(rdr)
	def.upperTranslation = RecR_F32(rdr)
	def.enableMotor = RecR_BOOL(rdr)
	def.maxMotorForce = RecR_F32(rdr)
	def.motorSpeed = RecR_F32(rdr)
	return def
}

RecR_REVOLUTEJOINTDEF :: proc(rdr: ^RecReader) -> RevoluteJointDef {
	def := DefaultRevoluteJointDef()
	RecR_JointBase(rdr, &def.base)
	def.targetAngle = RecR_F32(rdr)
	def.enableSpring = RecR_BOOL(rdr)
	def.hertz = RecR_F32(rdr)
	def.dampingRatio = RecR_F32(rdr)
	def.enableLimit = RecR_BOOL(rdr)
	def.lowerAngle = RecR_F32(rdr)
	def.upperAngle = RecR_F32(rdr)
	def.enableMotor = RecR_BOOL(rdr)
	def.maxMotorTorque = RecR_F32(rdr)
	def.motorSpeed = RecR_F32(rdr)
	return def
}

RecR_SPHERICALJOINTDEF :: proc(rdr: ^RecReader) -> SphericalJointDef {
	def := DefaultSphericalJointDef()
	RecR_JointBase(rdr, &def.base)
	def.enableSpring = RecR_BOOL(rdr)
	def.hertz = RecR_F32(rdr)
	def.dampingRatio = RecR_F32(rdr)
	def.targetRotation = RecR_QUAT(rdr)
	def.enableConeLimit = RecR_BOOL(rdr)
	def.coneAngle = RecR_F32(rdr)
	def.enableTwistLimit = RecR_BOOL(rdr)
	def.lowerTwistAngle = RecR_F32(rdr)
	def.upperTwistAngle = RecR_F32(rdr)
	def.enableMotor = RecR_BOOL(rdr)
	def.maxMotorTorque = RecR_F32(rdr)
	def.motorVelocity = RecR_VEC3(rdr)
	return def
}

RecR_WELDJOINTDEF :: proc(rdr: ^RecReader) -> WeldJointDef {
	def := DefaultWeldJointDef()
	RecR_JointBase(rdr, &def.base)
	def.linearHertz = RecR_F32(rdr)
	def.angularHertz = RecR_F32(rdr)
	def.linearDampingRatio = RecR_F32(rdr)
	def.angularDampingRatio = RecR_F32(rdr)
	return def
}

RecR_WHEELJOINTDEF :: proc(rdr: ^RecReader) -> WheelJointDef {
	def := DefaultWheelJointDef()
	RecR_JointBase(rdr, &def.base)
	def.enableSuspensionSpring = RecR_BOOL(rdr)
	def.suspensionHertz = RecR_F32(rdr)
	def.suspensionDampingRatio = RecR_F32(rdr)
	def.enableSuspensionLimit = RecR_BOOL(rdr)
	def.lowerSuspensionLimit = RecR_F32(rdr)
	def.upperSuspensionLimit = RecR_F32(rdr)
	def.enableSpinMotor = RecR_BOOL(rdr)
	def.maxSpinTorque = RecR_F32(rdr)
	def.spinSpeed = RecR_F32(rdr)
	def.enableSteering = RecR_BOOL(rdr)
	def.steeringHertz = RecR_F32(rdr)
	def.steeringDampingRatio = RecR_F32(rdr)
	def.targetSteeringAngle = RecR_F32(rdr)
	def.maxSteeringTorque = RecR_F32(rdr)
	def.enableSteeringLimit = RecR_BOOL(rdr)
	def.lowerSteeringLimit = RecR_F32(rdr)
	def.upperSteeringLimit = RecR_F32(rdr)
	return def
}

RecR_MOVERJOINTDEF :: proc(rdr: ^RecReader) -> MoverJointDef {
	def := DefaultMoverJointDef()
	RecR_JointBase(rdr, &def.base)
	def.linearVelocity = RecR_VEC3(rdr)
	def.maxForce = RecR_F32(rdr)
	return def
}

RecR_POGOJOINTDEF :: proc(rdr: ^RecReader) -> PogoJointDef {
	def := DefaultPogoJointDef()
	RecR_JointBase(rdr, &def.base)
	def.normal = RecR_VEC3(rdr)
	def.hertz = RecR_F32(rdr)
	def.dampingRatio = RecR_F32(rdr)
	def.restLength = RecR_F32(rdr)
	def.maxTensionForce = RecR_F32(rdr)
	def.maxCompressionForce = RecR_F32(rdr)
	def.impulse = RecR_F32(rdr)
	def.velocity = RecR_F32(rdr)
	return def
}

// Outliner body tracking. Defined after the player struct; forward declared here because the
// create/destroy dispatch sits above that struct.
// (RecTrackBodyCreate, RecTrackBodyDestroy)

// Id retargeting: replace world0 with the replay world's slot index.

@(private = "file")
RecMakeBodyId :: proc(rdr: ^RecReader, recorded: BodyId) -> BodyId {
	id: BodyId
	id.index1 = recorded.index1
	id.world0 = u16(rdr.replayWorldId.index1 - 1)
	id.generation = recorded.generation
	return id
}

@(private = "file")
RecMakeShapeId :: proc(rdr: ^RecReader, recorded: ShapeId) -> ShapeId {
	id: ShapeId
	id.index1 = recorded.index1
	id.world0 = u16(rdr.replayWorldId.index1 - 1)
	id.generation = recorded.generation
	return id
}

@(private = "file")
RecMakeJointId :: proc(rdr: ^RecReader, recorded: JointId) -> JointId {
	id: JointId
	id.index1 = recorded.index1
	id.world0 = u16(rdr.replayWorldId.index1 - 1)
	id.generation = recorded.generation
	return id
}

// A create op appends the returned id after args. index1 and generation must match;
// world0 always differs so we ignore it.
@(private = "file")
RecCheckId :: proc(rdr: ^RecReader, kind: string, gotIndex: i32, gotGen: u32, recIndex: i32, recGen: u32) {
	if gotIndex != recIndex || gotGen != recGen {
		fmt.printf(
			"b3ReplayFile: %s id mismatch (rec index1=%d gen=%d, got index1=%d gen=%d)\n",
			kind,
			recIndex,
			recGen,
			gotIndex,
			gotGen,
		)
		rdr.ok = false
	}
}

@(private = "file")
RecCheckBodyId :: proc(rdr: ^RecReader, got: BodyId, rec: BodyId) {
	RecCheckId(rdr, "body", got.index1, u32(got.generation), rec.index1, u32(rec.generation))
}

@(private = "file")
RecCheckShapeId :: proc(rdr: ^RecReader, got: ShapeId, rec: ShapeId) {
	RecCheckId(rdr, "shape", got.index1, u32(got.generation), rec.index1, u32(rec.generation))
}

@(private = "file")
RecCheckJointId :: proc(rdr: ^RecReader, got: JointId, rec: JointId) {
	RecCheckId(rdr, "joint", got.index1, u32(got.generation), rec.index1, u32(rec.generation))
}

// Get a geometry slot with kind validation.
RecGetSlot :: proc(rdr: ^RecReader, id: u32, kind: GeometryKind) -> ^RegistrySlot {
	if rdr == nil || id >= u32(rdr.slotCount) || rdr.slots[id].kind != kind {
		return nil
	}
	return &rdr.slots[id]
}

// Dispatch functions, one per op

@(private = "file")
RecDispatch_DestroyWorld :: proc(a: ^RecArgs_DestroyWorld, rdr: ^RecReader) {
	_ = a
	_ = rdr
	// End-of-session marker. The replay world is torn down in ValidateReplay, not here.
}

@(private = "file")
RecDispatch_Step :: proc(a: ^RecArgs_Step, rdr: ^RecReader) {
	_ = a
	World_Step(rdr.replayWorldId, a.dt, a.subStepCount)
}

@(private = "file")
RecDispatch_WorldEnableSleeping :: proc(a: ^RecArgs_WorldEnableSleeping, rdr: ^RecReader) {
	World_EnableSleeping(rdr.replayWorldId, a.flag)
}

@(private = "file")
RecDispatch_WorldEnableContinuous :: proc(a: ^RecArgs_WorldEnableContinuous, rdr: ^RecReader) {
	World_EnableContinuous(rdr.replayWorldId, a.flag)
}

@(private = "file")
RecDispatch_WorldSetRestitutionThreshold :: proc(a: ^RecArgs_WorldSetRestitutionThreshold, rdr: ^RecReader) {
	World_SetRestitutionThreshold(rdr.replayWorldId, a.value)
}

@(private = "file")
RecDispatch_WorldSetHitEventThreshold :: proc(a: ^RecArgs_WorldSetHitEventThreshold, rdr: ^RecReader) {
	World_SetHitEventThreshold(rdr.replayWorldId, a.value)
}

@(private = "file")
RecDispatch_WorldSetGravity :: proc(a: ^RecArgs_WorldSetGravity, rdr: ^RecReader) {
	World_SetGravity(rdr.replayWorldId, a.gravity)
}

@(private = "file")
RecDispatch_WorldExplode :: proc(a: ^RecArgs_WorldExplode, rdr: ^RecReader) {
	World_Explode(rdr.replayWorldId, &a.def)
}

@(private = "file")
RecDispatch_WorldSetContactTuning :: proc(a: ^RecArgs_WorldSetContactTuning, rdr: ^RecReader) {
	World_SetContactTuning(rdr.replayWorldId, a.hertz, a.dampingRatio, a.contactSpeed)
}

@(private = "file")
RecDispatch_WorldSetContactRecycleDistance :: proc(a: ^RecArgs_WorldSetContactRecycleDistance, rdr: ^RecReader) {
	World_SetContactRecycleDistance(rdr.replayWorldId, a.recycleDistance)
}

@(private = "file")
RecDispatch_WorldSetMaximumLinearSpeed :: proc(a: ^RecArgs_WorldSetMaximumLinearSpeed, rdr: ^RecReader) {
	World_SetMaximumLinearSpeed(rdr.replayWorldId, a.maximumLinearSpeed)
}

@(private = "file")
RecDispatch_WorldEnableWarmStarting :: proc(a: ^RecArgs_WorldEnableWarmStarting, rdr: ^RecReader) {
	World_EnableWarmStarting(rdr.replayWorldId, a.flag)
}

@(private = "file")
RecDispatch_WorldRebuildStaticTree :: proc(a: ^RecArgs_WorldRebuildStaticTree, rdr: ^RecReader) {
	_ = a
	World_RebuildStaticTree(rdr.replayWorldId)
}

@(private = "file")
RecDispatch_WorldEnableSpeculative :: proc(a: ^RecArgs_WorldEnableSpeculative, rdr: ^RecReader) {
	World_EnableSpeculative(rdr.replayWorldId, a.flag)
}

@(private = "file")
RecDispatch_WorldSetRestitutionIterations :: proc(a: ^RecArgs_WorldSetRestitutionIterations, rdr: ^RecReader) {
	World_SetRestitutionIterations(rdr.replayWorldId, a.iterations)
}

@(private = "file")
RecDispatch_WorldEnableRestitutionPropagation :: proc(a: ^RecArgs_WorldEnableRestitutionPropagation, rdr: ^RecReader) {
	World_EnableRestitutionPropagation(rdr.replayWorldId, a.flag)
}

@(private = "file")
RecDispatch_WorldSetIsotropicInertiaTolerance :: proc(a: ^RecArgs_WorldSetIsotropicInertiaTolerance, rdr: ^RecReader) {
	World_SetIsotropicInertiaTolerance(rdr.replayWorldId, a.tolerance)
}

@(private = "file")
RecDispatch_CreateBody :: proc(a: ^RecArgs_CreateBody, rdr: ^RecReader) {
	recId := RecR_BODYID(rdr)
	gotId := CreateBody(rdr.replayWorldId, &a.def)
	RecCheckBodyId(rdr, gotId, recId)
	if rdr.owner != nil {
		RecTrackBodyCreate(rdr.owner, gotId)
	}
}

@(private = "file")
RecDispatch_DestroyBody :: proc(a: ^RecArgs_DestroyBody, rdr: ^RecReader) {
	id := RecMakeBodyId(rdr, a.body)
	if rdr.owner != nil {
		RecTrackBodyDestroy(rdr.owner, id)
	}
	DestroyBody(id)
}

@(private = "file")
RecDispatch_BodySetTransform :: proc(a: ^RecArgs_BodySetTransform, rdr: ^RecReader) {
	Body_SetTransform(RecMakeBodyId(rdr, a.body), a.position, a.rotation)
}

@(private = "file")
RecDispatch_BodySetLinearVelocity :: proc(a: ^RecArgs_BodySetLinearVelocity, rdr: ^RecReader) {
	Body_SetLinearVelocity(RecMakeBodyId(rdr, a.body), a.v)
}

@(private = "file")
RecDispatch_BodySetType :: proc(a: ^RecArgs_BodySetType, rdr: ^RecReader) {
	Body_SetType(RecMakeBodyId(rdr, a.body), BodyType(a.type))
}

@(private = "file")
RecDispatch_BodySetName :: proc(a: ^RecArgs_BodySetName, rdr: ^RecReader) {
	Body_SetName(RecMakeBodyId(rdr, a.body), a.name)
}

@(private = "file")
RecDispatch_BodySetAngularVelocity :: proc(a: ^RecArgs_BodySetAngularVelocity, rdr: ^RecReader) {
	Body_SetAngularVelocity(RecMakeBodyId(rdr, a.body), a.w)
}

@(private = "file")
RecDispatch_BodySetTargetTransform :: proc(a: ^RecArgs_BodySetTargetTransform, rdr: ^RecReader) {
	Body_SetTargetTransform(RecMakeBodyId(rdr, a.body), a.target, a.timeStep, a.wake)
}

@(private = "file")
RecDispatch_BodyApplyForce :: proc(a: ^RecArgs_BodyApplyForce, rdr: ^RecReader) {
	Body_ApplyForce(RecMakeBodyId(rdr, a.body), a.force, a.point, a.wake)
}

@(private = "file")
RecDispatch_BodyApplyForceToCenter :: proc(a: ^RecArgs_BodyApplyForceToCenter, rdr: ^RecReader) {
	Body_ApplyForceToCenter(RecMakeBodyId(rdr, a.body), a.force, a.wake)
}

@(private = "file")
RecDispatch_BodyApplyTorque :: proc(a: ^RecArgs_BodyApplyTorque, rdr: ^RecReader) {
	Body_ApplyTorque(RecMakeBodyId(rdr, a.body), a.torque, a.wake)
}

@(private = "file")
RecDispatch_BodyApplyLinearImpulse :: proc(a: ^RecArgs_BodyApplyLinearImpulse, rdr: ^RecReader) {
	Body_ApplyLinearImpulse(RecMakeBodyId(rdr, a.body), a.impulse, a.point, a.wake)
}

@(private = "file")
RecDispatch_BodyApplyLinearImpulseToCenter :: proc(a: ^RecArgs_BodyApplyLinearImpulseToCenter, rdr: ^RecReader) {
	Body_ApplyLinearImpulseToCenter(RecMakeBodyId(rdr, a.body), a.impulse, a.wake)
}

@(private = "file")
RecDispatch_BodyApplyAngularImpulse :: proc(a: ^RecArgs_BodyApplyAngularImpulse, rdr: ^RecReader) {
	Body_ApplyAngularImpulse(RecMakeBodyId(rdr, a.body), a.impulse, a.wake)
}

@(private = "file")
RecDispatch_BodySetMassData :: proc(a: ^RecArgs_BodySetMassData, rdr: ^RecReader) {
	Body_SetMassData(RecMakeBodyId(rdr, a.body), a.massData)
}

@(private = "file")
RecDispatch_BodyApplyMassFromShapes :: proc(a: ^RecArgs_BodyApplyMassFromShapes, rdr: ^RecReader) {
	Body_ApplyMassFromShapes(RecMakeBodyId(rdr, a.body))
}

@(private = "file")
RecDispatch_BodySetLinearDamping :: proc(a: ^RecArgs_BodySetLinearDamping, rdr: ^RecReader) {
	Body_SetLinearDamping(RecMakeBodyId(rdr, a.body), a.damping)
}

@(private = "file")
RecDispatch_BodySetAngularDamping :: proc(a: ^RecArgs_BodySetAngularDamping, rdr: ^RecReader) {
	Body_SetAngularDamping(RecMakeBodyId(rdr, a.body), a.damping)
}

@(private = "file")
RecDispatch_BodySetGravityScale :: proc(a: ^RecArgs_BodySetGravityScale, rdr: ^RecReader) {
	Body_SetGravityScale(RecMakeBodyId(rdr, a.body), a.scale)
}

@(private = "file")
RecDispatch_BodySetAwake :: proc(a: ^RecArgs_BodySetAwake, rdr: ^RecReader) {
	Body_SetAwake(RecMakeBodyId(rdr, a.body), a.awake)
}

@(private = "file")
RecDispatch_BodyEnableSleep :: proc(a: ^RecArgs_BodyEnableSleep, rdr: ^RecReader) {
	Body_EnableSleep(RecMakeBodyId(rdr, a.body), a.flag)
}

@(private = "file")
RecDispatch_BodySetSleepThreshold :: proc(a: ^RecArgs_BodySetSleepThreshold, rdr: ^RecReader) {
	Body_SetSleepThreshold(RecMakeBodyId(rdr, a.body), a.threshold)
}

@(private = "file")
RecDispatch_BodySetSafetyFactor :: proc(a: ^RecArgs_BodySetSafetyFactor, rdr: ^RecReader) {
	Body_SetSafetyFactor(RecMakeBodyId(rdr, a.body), a.value)
}

@(private = "file")
RecDispatch_BodyDisable :: proc(a: ^RecArgs_BodyDisable, rdr: ^RecReader) {
	Body_Disable(RecMakeBodyId(rdr, a.body))
}

@(private = "file")
RecDispatch_BodyEnable :: proc(a: ^RecArgs_BodyEnable, rdr: ^RecReader) {
	Body_Enable(RecMakeBodyId(rdr, a.body))
}

@(private = "file")
RecDispatch_BodySetMotionLocks :: proc(a: ^RecArgs_BodySetMotionLocks, rdr: ^RecReader) {
	Body_SetMotionLocks(RecMakeBodyId(rdr, a.body), a.locks)
}

@(private = "file")
RecDispatch_BodySetBullet :: proc(a: ^RecArgs_BodySetBullet, rdr: ^RecReader) {
	Body_SetBullet(RecMakeBodyId(rdr, a.body), a.flag)
}

@(private = "file")
RecDispatch_BodyAllowFastRotation :: proc(a: ^RecArgs_BodyAllowFastRotation, rdr: ^RecReader) {
	Body_AllowFastRotation(RecMakeBodyId(rdr, a.body), a.flag)
}

@(private = "file")
RecDispatch_BodyEnableContactRecycling :: proc(a: ^RecArgs_BodyEnableContactRecycling, rdr: ^RecReader) {
	Body_EnableContactRecycling(RecMakeBodyId(rdr, a.body), a.flag)
}

@(private = "file")
RecDispatch_BodyEnableHitEvents :: proc(a: ^RecArgs_BodyEnableHitEvents, rdr: ^RecReader) {
	Body_EnableHitEvents(RecMakeBodyId(rdr, a.body), a.flag)
}

@(private = "file")
RecDispatch_CreateSphereShape :: proc(a: ^RecArgs_CreateSphereShape, rdr: ^RecReader) {
	recId := RecR_SHAPEID(rdr)
	bodyId := RecMakeBodyId(rdr, a.body)
	gotId := CreateSphereShape(bodyId, &a.def, &a.sphere)
	RecCheckShapeId(rdr, gotId, recId)
}

@(private = "file")
RecDispatch_CreateCapsuleShape :: proc(a: ^RecArgs_CreateCapsuleShape, rdr: ^RecReader) {
	recId := RecR_SHAPEID(rdr)
	bodyId := RecMakeBodyId(rdr, a.body)
	gotId := CreateCapsuleShape(bodyId, &a.def, &a.capsule)
	RecCheckShapeId(rdr, gotId, recId)
}

@(private = "file")
RecDispatch_CreateHullShape :: proc(a: ^RecArgs_CreateHullShape, rdr: ^RecReader) {
	recId := RecR_SHAPEID(rdr)
	if !rdr.ok {
		return
	}
	id := a.geometryId
	slot := RecGetSlot(rdr, id, .geometryHull)
	if slot == nil {
		fmt.printf("b3ReplayFile: hull geometryId %d is invalid\n", id)
		rdr.ok = false
		return
	}
	bodyId := RecMakeBodyId(rdr, a.body)
	// Hull is cloned by CreateHullShape into the world DB; no caching needed.
	gotId := CreateHullShape(bodyId, &a.def, (^HullData)(slot.bytes))
	RecCheckShapeId(rdr, gotId, recId)
}

@(private = "file")
RecDispatch_CreateMeshShape :: proc(a: ^RecArgs_CreateMeshShape, rdr: ^RecReader) {
	recId := RecR_SHAPEID(rdr)
	if !rdr.ok {
		return
	}
	id := a.geometryId
	slot := RecGetSlot(rdr, id, .geometryMesh)
	if slot == nil {
		fmt.printf("b3ReplayFile: mesh geometryId %d is invalid\n", id)
		rdr.ok = false
		return
	}
	mesh := (^MeshData)(slot.bytes)
	bodyId := RecMakeBodyId(rdr, a.body)
	gotId := CreateMeshShape(bodyId, &a.def, mesh, a.scale)
	RecCheckShapeId(rdr, gotId, recId)
}

@(private = "file")
RecDispatch_CreateHeightFieldShape :: proc(a: ^RecArgs_CreateHeightFieldShape, rdr: ^RecReader) {
	recId := RecR_SHAPEID(rdr)
	if !rdr.ok {
		return
	}
	id := a.geometryId
	slot := RecGetSlot(rdr, id, .geometryHeightField)
	if slot == nil {
		fmt.printf("b3ReplayFile: heightfield geometryId %d is invalid\n", id)
		rdr.ok = false
		return
	}
	hf := (^HeightFieldData)(slot.bytes)
	bodyId := RecMakeBodyId(rdr, a.body)
	gotId := CreateHeightFieldShape(bodyId, &a.def, hf)
	RecCheckShapeId(rdr, gotId, recId)
}

@(private = "file")
RecDispatch_CreateCompoundShape :: proc(a: ^RecArgs_CreateCompoundShape, rdr: ^RecReader) {
	recId := RecR_SHAPEID(rdr)
	if !rdr.ok {
		return
	}
	id := a.geometryId
	slot := RecGetSlot(rdr, id, .geometryCompound)
	if slot == nil {
		fmt.printf("b3ReplayFile: compound geometryId %d is invalid\n", id)
		rdr.ok = false
		return
	}
	compound := (^CompoundData)(slot.bytes)
	bodyId := RecMakeBodyId(rdr, a.body)
	// CreateCompoundShape takes a non-const def pointer; cast away const for the scratch def
	shapeDef := a.def
	gotId := CreateBakedCompoundShape(bodyId, &shapeDef, compound)
	RecCheckShapeId(rdr, gotId, recId)
}

@(private = "file")
RecDispatch_DestroyShape :: proc(a: ^RecArgs_DestroyShape, rdr: ^RecReader) {
	DestroyShape(RecMakeShapeId(rdr, a.shape), a.updateBodyMass)
}

@(private = "file")
RecDispatch_ShapeSetName :: proc(a: ^RecArgs_ShapeSetName, rdr: ^RecReader) {
	Shape_SetName(RecMakeShapeId(rdr, a.shape), a.name)
}

@(private = "file")
RecDispatch_ShapeSetDensity :: proc(a: ^RecArgs_ShapeSetDensity, rdr: ^RecReader) {
	Shape_SetDensity(RecMakeShapeId(rdr, a.shape), a.density, a.updateBodyMass)
}

@(private = "file")
RecDispatch_ShapeSetFriction :: proc(a: ^RecArgs_ShapeSetFriction, rdr: ^RecReader) {
	Shape_SetFriction(RecMakeShapeId(rdr, a.shape), a.friction)
}

@(private = "file")
RecDispatch_ShapeSetRestitution :: proc(a: ^RecArgs_ShapeSetRestitution, rdr: ^RecReader) {
	Shape_SetRestitution(RecMakeShapeId(rdr, a.shape), a.restitution)
}

@(private = "file")
RecDispatch_ShapeSetSurfaceMaterial :: proc(a: ^RecArgs_ShapeSetSurfaceMaterial, rdr: ^RecReader) {
	Shape_SetSurfaceMaterial(RecMakeShapeId(rdr, a.shape), a.material)
}

@(private = "file")
RecDispatch_ShapeSetMeshMaterial :: proc(a: ^RecArgs_ShapeSetMeshMaterial, rdr: ^RecReader) {
	Shape_SetMeshMaterial(RecMakeShapeId(rdr, a.shape), a.material, a.index)
}

@(private = "file")
RecDispatch_ShapeSetFilter :: proc(a: ^RecArgs_ShapeSetFilter, rdr: ^RecReader) {
	Shape_SetFilter(RecMakeShapeId(rdr, a.shape), a.filter, a.invokeContacts)
}

@(private = "file")
RecDispatch_ShapeEnableSensorEvents :: proc(a: ^RecArgs_ShapeEnableSensorEvents, rdr: ^RecReader) {
	Shape_EnableSensorEvents(RecMakeShapeId(rdr, a.shape), a.flag)
}

@(private = "file")
RecDispatch_ShapeEnableContactEvents :: proc(a: ^RecArgs_ShapeEnableContactEvents, rdr: ^RecReader) {
	Shape_EnableContactEvents(RecMakeShapeId(rdr, a.shape), a.flag)
}

@(private = "file")
RecDispatch_ShapeEnablePreSolveEvents :: proc(a: ^RecArgs_ShapeEnablePreSolveEvents, rdr: ^RecReader) {
	Shape_EnablePreSolveEvents(RecMakeShapeId(rdr, a.shape), a.flag)
}

@(private = "file")
RecDispatch_ShapeEnableHitEvents :: proc(a: ^RecArgs_ShapeEnableHitEvents, rdr: ^RecReader) {
	Shape_EnableHitEvents(RecMakeShapeId(rdr, a.shape), a.flag)
}

@(private = "file")
RecDispatch_ShapeSetSphere :: proc(a: ^RecArgs_ShapeSetSphere, rdr: ^RecReader) {
	Shape_SetSphere(RecMakeShapeId(rdr, a.shape), &a.sphere)
}

@(private = "file")
RecDispatch_ShapeSetCapsule :: proc(a: ^RecArgs_ShapeSetCapsule, rdr: ^RecReader) {
	Shape_SetCapsule(RecMakeShapeId(rdr, a.shape), &a.capsule)
}

@(private = "file")
RecDispatch_ShapeSetHull :: proc(a: ^RecArgs_ShapeSetHull, rdr: ^RecReader) {
	id := a.geometryId
	slot := RecGetSlot(rdr, id, .geometryHull)
	if slot == nil {
		fmt.printf("b3ReplayFile: hull geometryId %d is invalid\n", id)
		rdr.ok = false
		return
	}
	shapeId := RecMakeShapeId(rdr, a.shape)
	Shape_SetHull(shapeId, (^HullData)(slot.bytes))
}

@(private = "file")
RecDispatch_ShapeSetMesh :: proc(a: ^RecArgs_ShapeSetMesh, rdr: ^RecReader) {
	id := a.geometryId
	slot := RecGetSlot(rdr, id, .geometryMesh)
	if slot == nil {
		fmt.printf("b3ReplayFile: mesh geometryId %d is invalid\n", id)
		rdr.ok = false
		return
	}
	shapeId := RecMakeShapeId(rdr, a.shape)
	mesh := (^MeshData)(slot.bytes)
	Shape_SetMesh(shapeId, mesh, a.scale)
}

@(private = "file")
RecDispatch_ShapeApplyWind :: proc(a: ^RecArgs_ShapeApplyWind, rdr: ^RecReader) {
	Shape_ApplyWind(RecMakeShapeId(rdr, a.shape), a.wind, a.drag, a.lift, a.maxSpeed, a.wake)
}

// Joint creates: remap body ids in the def before calling the API.

@(private = "file")
RecDispatch_CreateParallelJoint :: proc(a: ^RecArgs_CreateParallelJoint, rdr: ^RecReader) {
	recId := RecR_JOINTID(rdr)
	def := a.def
	def.base.bodyIdA = RecMakeBodyId(rdr, def.base.bodyIdA)
	def.base.bodyIdB = RecMakeBodyId(rdr, def.base.bodyIdB)
	RecCheckJointId(rdr, CreateParallelJoint(rdr.replayWorldId, &def), recId)
}

@(private = "file")
RecDispatch_CreateDistanceJoint :: proc(a: ^RecArgs_CreateDistanceJoint, rdr: ^RecReader) {
	recId := RecR_JOINTID(rdr)
	def := a.def
	def.base.bodyIdA = RecMakeBodyId(rdr, def.base.bodyIdA)
	def.base.bodyIdB = RecMakeBodyId(rdr, def.base.bodyIdB)
	RecCheckJointId(rdr, CreateDistanceJoint(rdr.replayWorldId, &def), recId)
}

@(private = "file")
RecDispatch_CreateFilterJoint :: proc(a: ^RecArgs_CreateFilterJoint, rdr: ^RecReader) {
	recId := RecR_JOINTID(rdr)
	def := a.def
	def.base.bodyIdA = RecMakeBodyId(rdr, def.base.bodyIdA)
	def.base.bodyIdB = RecMakeBodyId(rdr, def.base.bodyIdB)
	RecCheckJointId(rdr, CreateFilterJoint(rdr.replayWorldId, &def), recId)
}

@(private = "file")
RecDispatch_CreateMotorJoint :: proc(a: ^RecArgs_CreateMotorJoint, rdr: ^RecReader) {
	recId := RecR_JOINTID(rdr)
	def := a.def
	def.base.bodyIdA = RecMakeBodyId(rdr, def.base.bodyIdA)
	def.base.bodyIdB = RecMakeBodyId(rdr, def.base.bodyIdB)
	RecCheckJointId(rdr, CreateMotorJoint(rdr.replayWorldId, &def), recId)
}

@(private = "file")
RecDispatch_CreatePrismaticJoint :: proc(a: ^RecArgs_CreatePrismaticJoint, rdr: ^RecReader) {
	recId := RecR_JOINTID(rdr)
	def := a.def
	def.base.bodyIdA = RecMakeBodyId(rdr, def.base.bodyIdA)
	def.base.bodyIdB = RecMakeBodyId(rdr, def.base.bodyIdB)
	RecCheckJointId(rdr, CreatePrismaticJoint(rdr.replayWorldId, &def), recId)
}

@(private = "file")
RecDispatch_CreateRevoluteJoint :: proc(a: ^RecArgs_CreateRevoluteJoint, rdr: ^RecReader) {
	recId := RecR_JOINTID(rdr)
	def := a.def
	def.base.bodyIdA = RecMakeBodyId(rdr, def.base.bodyIdA)
	def.base.bodyIdB = RecMakeBodyId(rdr, def.base.bodyIdB)
	RecCheckJointId(rdr, CreateRevoluteJoint(rdr.replayWorldId, &def), recId)
}

@(private = "file")
RecDispatch_CreateSphericalJoint :: proc(a: ^RecArgs_CreateSphericalJoint, rdr: ^RecReader) {
	recId := RecR_JOINTID(rdr)
	def := a.def
	def.base.bodyIdA = RecMakeBodyId(rdr, def.base.bodyIdA)
	def.base.bodyIdB = RecMakeBodyId(rdr, def.base.bodyIdB)
	RecCheckJointId(rdr, CreateSphericalJoint(rdr.replayWorldId, &def), recId)
}

@(private = "file")
RecDispatch_CreateWeldJoint :: proc(a: ^RecArgs_CreateWeldJoint, rdr: ^RecReader) {
	recId := RecR_JOINTID(rdr)
	def := a.def
	def.base.bodyIdA = RecMakeBodyId(rdr, def.base.bodyIdA)
	def.base.bodyIdB = RecMakeBodyId(rdr, def.base.bodyIdB)
	RecCheckJointId(rdr, CreateWeldJoint(rdr.replayWorldId, &def), recId)
}

@(private = "file")
RecDispatch_CreateWheelJoint :: proc(a: ^RecArgs_CreateWheelJoint, rdr: ^RecReader) {
	recId := RecR_JOINTID(rdr)
	def := a.def
	def.base.bodyIdA = RecMakeBodyId(rdr, def.base.bodyIdA)
	def.base.bodyIdB = RecMakeBodyId(rdr, def.base.bodyIdB)
	RecCheckJointId(rdr, CreateWheelJoint(rdr.replayWorldId, &def), recId)
}

@(private = "file")
RecDispatch_CreateMoverJoint :: proc(a: ^RecArgs_CreateMoverJoint, rdr: ^RecReader) {
	recId := RecR_JOINTID(rdr)
	def := a.def
	def.base.bodyIdA = RecMakeBodyId(rdr, def.base.bodyIdA)
	def.base.bodyIdB = RecMakeBodyId(rdr, def.base.bodyIdB)
	RecCheckJointId(rdr, CreateMoverJoint(rdr.replayWorldId, &def), recId)
}

@(private = "file")
RecDispatch_CreatePogoJoint :: proc(a: ^RecArgs_CreatePogoJoint, rdr: ^RecReader) {
	recId := RecR_JOINTID(rdr)
	def := a.def
	def.base.bodyIdA = RecMakeBodyId(rdr, def.base.bodyIdA)
	def.base.bodyIdB = RecMakeBodyId(rdr, def.base.bodyIdB)
	RecCheckJointId(rdr, CreatePogoJoint(rdr.replayWorldId, &def), recId)
}

@(private = "file")
RecDispatch_DestroyJoint :: proc(a: ^RecArgs_DestroyJoint, rdr: ^RecReader) {
	DestroyJoint(RecMakeJointId(rdr, a.joint), a.wakeAttached)
}

@(private = "file")
RecDispatch_JointSetLocalFrameA :: proc(a: ^RecArgs_JointSetLocalFrameA, rdr: ^RecReader) {
	Joint_SetLocalFrameA(RecMakeJointId(rdr, a.joint), a.localFrame)
}

@(private = "file")
RecDispatch_JointSetLocalFrameB :: proc(a: ^RecArgs_JointSetLocalFrameB, rdr: ^RecReader) {
	Joint_SetLocalFrameB(RecMakeJointId(rdr, a.joint), a.localFrame)
}

@(private = "file")
RecDispatch_JointSetCollideConnected :: proc(a: ^RecArgs_JointSetCollideConnected, rdr: ^RecReader) {
	Joint_SetCollideConnected(RecMakeJointId(rdr, a.joint), a.shouldCollide)
}

@(private = "file")
RecDispatch_JointWakeBodies :: proc(a: ^RecArgs_JointWakeBodies, rdr: ^RecReader) {
	Joint_WakeBodies(RecMakeJointId(rdr, a.joint))
}

@(private = "file")
RecDispatch_JointSetConstraintTuning :: proc(a: ^RecArgs_JointSetConstraintTuning, rdr: ^RecReader) {
	Joint_SetConstraintTuning(RecMakeJointId(rdr, a.joint), a.hertz, a.dampingRatio)
}

@(private = "file")
RecDispatch_JointSetForceThreshold :: proc(a: ^RecArgs_JointSetForceThreshold, rdr: ^RecReader) {
	Joint_SetForceThreshold(RecMakeJointId(rdr, a.joint), a.threshold)
}

@(private = "file")
RecDispatch_JointSetTorqueThreshold :: proc(a: ^RecArgs_JointSetTorqueThreshold, rdr: ^RecReader) {
	Joint_SetTorqueThreshold(RecMakeJointId(rdr, a.joint), a.threshold)
}

@(private = "file")
RecDispatch_ParallelJointSetSpringHertz :: proc(a: ^RecArgs_ParallelJointSetSpringHertz, rdr: ^RecReader) {
	ParallelJoint_SetSpringHertz(RecMakeJointId(rdr, a.joint), a.hertz)
}

@(private = "file")
RecDispatch_ParallelJointSetSpringDampingRatio :: proc(
	a: ^RecArgs_ParallelJointSetSpringDampingRatio,
	rdr: ^RecReader,
) {
	ParallelJoint_SetSpringDampingRatio(RecMakeJointId(rdr, a.joint), a.dampingRatio)
}

@(private = "file")
RecDispatch_ParallelJointSetMaxTorque :: proc(a: ^RecArgs_ParallelJointSetMaxTorque, rdr: ^RecReader) {
	ParallelJoint_SetMaxTorque(RecMakeJointId(rdr, a.joint), a.maxTorque)
}

@(private = "file")
RecDispatch_DistanceJointSetLength :: proc(a: ^RecArgs_DistanceJointSetLength, rdr: ^RecReader) {
	DistanceJoint_SetLength(RecMakeJointId(rdr, a.joint), a.length)
}

@(private = "file")
RecDispatch_DistanceJointEnableSpring :: proc(a: ^RecArgs_DistanceJointEnableSpring, rdr: ^RecReader) {
	DistanceJoint_EnableSpring(RecMakeJointId(rdr, a.joint), a.enableSpring)
}

@(private = "file")
RecDispatch_DistanceJointSetSpringForceRange :: proc(a: ^RecArgs_DistanceJointSetSpringForceRange, rdr: ^RecReader) {
	DistanceJoint_SetSpringForceRange(RecMakeJointId(rdr, a.joint), a.lowerForce, a.upperForce)
}

@(private = "file")
RecDispatch_DistanceJointSetSpringHertz :: proc(a: ^RecArgs_DistanceJointSetSpringHertz, rdr: ^RecReader) {
	DistanceJoint_SetSpringHertz(RecMakeJointId(rdr, a.joint), a.hertz)
}

@(private = "file")
RecDispatch_DistanceJointSetSpringDampingRatio :: proc(
	a: ^RecArgs_DistanceJointSetSpringDampingRatio,
	rdr: ^RecReader,
) {
	DistanceJoint_SetSpringDampingRatio(RecMakeJointId(rdr, a.joint), a.dampingRatio)
}

@(private = "file")
RecDispatch_DistanceJointEnableLimit :: proc(a: ^RecArgs_DistanceJointEnableLimit, rdr: ^RecReader) {
	DistanceJoint_EnableLimit(RecMakeJointId(rdr, a.joint), a.enableLimit)
}

@(private = "file")
RecDispatch_DistanceJointSetLengthRange :: proc(a: ^RecArgs_DistanceJointSetLengthRange, rdr: ^RecReader) {
	DistanceJoint_SetLengthRange(RecMakeJointId(rdr, a.joint), a.minLength, a.maxLength)
}

@(private = "file")
RecDispatch_DistanceJointEnableMotor :: proc(a: ^RecArgs_DistanceJointEnableMotor, rdr: ^RecReader) {
	DistanceJoint_EnableMotor(RecMakeJointId(rdr, a.joint), a.enableMotor)
}

@(private = "file")
RecDispatch_DistanceJointSetMotorSpeed :: proc(a: ^RecArgs_DistanceJointSetMotorSpeed, rdr: ^RecReader) {
	DistanceJoint_SetMotorSpeed(RecMakeJointId(rdr, a.joint), a.motorSpeed)
}

@(private = "file")
RecDispatch_DistanceJointSetMaxMotorForce :: proc(a: ^RecArgs_DistanceJointSetMaxMotorForce, rdr: ^RecReader) {
	DistanceJoint_SetMaxMotorForce(RecMakeJointId(rdr, a.joint), a.force)
}

@(private = "file")
RecDispatch_MotorJointSetLinearVelocity :: proc(a: ^RecArgs_MotorJointSetLinearVelocity, rdr: ^RecReader) {
	MotorJoint_SetLinearVelocity(RecMakeJointId(rdr, a.joint), a.velocity)
}

@(private = "file")
RecDispatch_MotorJointSetAngularVelocity :: proc(a: ^RecArgs_MotorJointSetAngularVelocity, rdr: ^RecReader) {
	MotorJoint_SetAngularVelocity(RecMakeJointId(rdr, a.joint), a.velocity)
}

@(private = "file")
RecDispatch_MotorJointSetMaxVelocityForce :: proc(a: ^RecArgs_MotorJointSetMaxVelocityForce, rdr: ^RecReader) {
	MotorJoint_SetMaxVelocityForce(RecMakeJointId(rdr, a.joint), a.maxForce)
}

@(private = "file")
RecDispatch_MotorJointSetMaxVelocityTorque :: proc(a: ^RecArgs_MotorJointSetMaxVelocityTorque, rdr: ^RecReader) {
	MotorJoint_SetMaxVelocityTorque(RecMakeJointId(rdr, a.joint), a.maxTorque)
}

@(private = "file")
RecDispatch_MotorJointSetLinearHertz :: proc(a: ^RecArgs_MotorJointSetLinearHertz, rdr: ^RecReader) {
	MotorJoint_SetLinearHertz(RecMakeJointId(rdr, a.joint), a.hertz)
}

@(private = "file")
RecDispatch_MotorJointSetLinearDampingRatio :: proc(a: ^RecArgs_MotorJointSetLinearDampingRatio, rdr: ^RecReader) {
	MotorJoint_SetLinearDampingRatio(RecMakeJointId(rdr, a.joint), a.damping)
}

@(private = "file")
RecDispatch_MotorJointSetAngularHertz :: proc(a: ^RecArgs_MotorJointSetAngularHertz, rdr: ^RecReader) {
	MotorJoint_SetAngularHertz(RecMakeJointId(rdr, a.joint), a.hertz)
}

@(private = "file")
RecDispatch_MotorJointSetAngularDampingRatio :: proc(a: ^RecArgs_MotorJointSetAngularDampingRatio, rdr: ^RecReader) {
	MotorJoint_SetAngularDampingRatio(RecMakeJointId(rdr, a.joint), a.damping)
}

@(private = "file")
RecDispatch_MotorJointSetMaxSpringForce :: proc(a: ^RecArgs_MotorJointSetMaxSpringForce, rdr: ^RecReader) {
	MotorJoint_SetMaxSpringForce(RecMakeJointId(rdr, a.joint), a.maxForce)
}

@(private = "file")
RecDispatch_MotorJointSetMaxSpringTorque :: proc(a: ^RecArgs_MotorJointSetMaxSpringTorque, rdr: ^RecReader) {
	MotorJoint_SetMaxSpringTorque(RecMakeJointId(rdr, a.joint), a.maxTorque)
}

@(private = "file")
RecDispatch_PrismaticJointEnableSpring :: proc(a: ^RecArgs_PrismaticJointEnableSpring, rdr: ^RecReader) {
	PrismaticJoint_EnableSpring(RecMakeJointId(rdr, a.joint), a.enableSpring)
}

@(private = "file")
RecDispatch_PrismaticJointSetSpringHertz :: proc(a: ^RecArgs_PrismaticJointSetSpringHertz, rdr: ^RecReader) {
	PrismaticJoint_SetSpringHertz(RecMakeJointId(rdr, a.joint), a.hertz)
}

@(private = "file")
RecDispatch_PrismaticJointSetSpringDampingRatio :: proc(
	a: ^RecArgs_PrismaticJointSetSpringDampingRatio,
	rdr: ^RecReader,
) {
	PrismaticJoint_SetSpringDampingRatio(RecMakeJointId(rdr, a.joint), a.dampingRatio)
}

@(private = "file")
RecDispatch_PrismaticJointSetTargetTranslation :: proc(
	a: ^RecArgs_PrismaticJointSetTargetTranslation,
	rdr: ^RecReader,
) {
	PrismaticJoint_SetTargetTranslation(RecMakeJointId(rdr, a.joint), a.translation)
}

@(private = "file")
RecDispatch_PrismaticJointEnableLimit :: proc(a: ^RecArgs_PrismaticJointEnableLimit, rdr: ^RecReader) {
	PrismaticJoint_EnableLimit(RecMakeJointId(rdr, a.joint), a.enableLimit)
}

@(private = "file")
RecDispatch_PrismaticJointSetLimits :: proc(a: ^RecArgs_PrismaticJointSetLimits, rdr: ^RecReader) {
	PrismaticJoint_SetLimits(RecMakeJointId(rdr, a.joint), a.lower, a.upper)
}

@(private = "file")
RecDispatch_PrismaticJointEnableMotor :: proc(a: ^RecArgs_PrismaticJointEnableMotor, rdr: ^RecReader) {
	PrismaticJoint_EnableMotor(RecMakeJointId(rdr, a.joint), a.enableMotor)
}

@(private = "file")
RecDispatch_PrismaticJointSetMotorSpeed :: proc(a: ^RecArgs_PrismaticJointSetMotorSpeed, rdr: ^RecReader) {
	PrismaticJoint_SetMotorSpeed(RecMakeJointId(rdr, a.joint), a.motorSpeed)
}

@(private = "file")
RecDispatch_PrismaticJointSetMaxMotorForce :: proc(a: ^RecArgs_PrismaticJointSetMaxMotorForce, rdr: ^RecReader) {
	PrismaticJoint_SetMaxMotorForce(RecMakeJointId(rdr, a.joint), a.force)
}

@(private = "file")
RecDispatch_RevoluteJointEnableSpring :: proc(a: ^RecArgs_RevoluteJointEnableSpring, rdr: ^RecReader) {
	RevoluteJoint_EnableSpring(RecMakeJointId(rdr, a.joint), a.enableSpring)
}

@(private = "file")
RecDispatch_RevoluteJointSetSpringHertz :: proc(a: ^RecArgs_RevoluteJointSetSpringHertz, rdr: ^RecReader) {
	RevoluteJoint_SetSpringHertz(RecMakeJointId(rdr, a.joint), a.hertz)
}

@(private = "file")
RecDispatch_RevoluteJointSetSpringDampingRatio :: proc(
	a: ^RecArgs_RevoluteJointSetSpringDampingRatio,
	rdr: ^RecReader,
) {
	RevoluteJoint_SetSpringDampingRatio(RecMakeJointId(rdr, a.joint), a.dampingRatio)
}

@(private = "file")
RecDispatch_RevoluteJointSetTargetAngle :: proc(a: ^RecArgs_RevoluteJointSetTargetAngle, rdr: ^RecReader) {
	RevoluteJoint_SetTargetAngle(RecMakeJointId(rdr, a.joint), a.angle)
}

@(private = "file")
RecDispatch_RevoluteJointEnableLimit :: proc(a: ^RecArgs_RevoluteJointEnableLimit, rdr: ^RecReader) {
	RevoluteJoint_EnableLimit(RecMakeJointId(rdr, a.joint), a.enableLimit)
}

@(private = "file")
RecDispatch_RevoluteJointSetLimits :: proc(a: ^RecArgs_RevoluteJointSetLimits, rdr: ^RecReader) {
	RevoluteJoint_SetLimits(RecMakeJointId(rdr, a.joint), a.lower, a.upper)
}

@(private = "file")
RecDispatch_RevoluteJointEnableMotor :: proc(a: ^RecArgs_RevoluteJointEnableMotor, rdr: ^RecReader) {
	RevoluteJoint_EnableMotor(RecMakeJointId(rdr, a.joint), a.enableMotor)
}

@(private = "file")
RecDispatch_RevoluteJointSetMotorSpeed :: proc(a: ^RecArgs_RevoluteJointSetMotorSpeed, rdr: ^RecReader) {
	RevoluteJoint_SetMotorSpeed(RecMakeJointId(rdr, a.joint), a.motorSpeed)
}

@(private = "file")
RecDispatch_RevoluteJointSetMaxMotorTorque :: proc(a: ^RecArgs_RevoluteJointSetMaxMotorTorque, rdr: ^RecReader) {
	RevoluteJoint_SetMaxMotorTorque(RecMakeJointId(rdr, a.joint), a.torque)
}

@(private = "file")
RecDispatch_SphericalJointEnableConeLimit :: proc(a: ^RecArgs_SphericalJointEnableConeLimit, rdr: ^RecReader) {
	SphericalJoint_EnableConeLimit(RecMakeJointId(rdr, a.joint), a.enableLimit)
}

@(private = "file")
RecDispatch_SphericalJointSetConeLimit :: proc(a: ^RecArgs_SphericalJointSetConeLimit, rdr: ^RecReader) {
	SphericalJoint_SetConeLimit(RecMakeJointId(rdr, a.joint), a.angleRadians)
}

@(private = "file")
RecDispatch_SphericalJointEnableTwistLimit :: proc(a: ^RecArgs_SphericalJointEnableTwistLimit, rdr: ^RecReader) {
	SphericalJoint_EnableTwistLimit(RecMakeJointId(rdr, a.joint), a.enableLimit)
}

@(private = "file")
RecDispatch_SphericalJointSetTwistLimits :: proc(a: ^RecArgs_SphericalJointSetTwistLimits, rdr: ^RecReader) {
	SphericalJoint_SetTwistLimits(RecMakeJointId(rdr, a.joint), a.lower, a.upper)
}

@(private = "file")
RecDispatch_SphericalJointEnableSpring :: proc(a: ^RecArgs_SphericalJointEnableSpring, rdr: ^RecReader) {
	SphericalJoint_EnableSpring(RecMakeJointId(rdr, a.joint), a.enableSpring)
}

@(private = "file")
RecDispatch_SphericalJointSetSpringHertz :: proc(a: ^RecArgs_SphericalJointSetSpringHertz, rdr: ^RecReader) {
	SphericalJoint_SetSpringHertz(RecMakeJointId(rdr, a.joint), a.hertz)
}

@(private = "file")
RecDispatch_SphericalJointSetSpringDampingRatio :: proc(
	a: ^RecArgs_SphericalJointSetSpringDampingRatio,
	rdr: ^RecReader,
) {
	SphericalJoint_SetSpringDampingRatio(RecMakeJointId(rdr, a.joint), a.dampingRatio)
}

@(private = "file")
RecDispatch_SphericalJointSetTargetRotation :: proc(a: ^RecArgs_SphericalJointSetTargetRotation, rdr: ^RecReader) {
	SphericalJoint_SetTargetRotation(RecMakeJointId(rdr, a.joint), a.targetRotation)
}

@(private = "file")
RecDispatch_SphericalJointEnableMotor :: proc(a: ^RecArgs_SphericalJointEnableMotor, rdr: ^RecReader) {
	SphericalJoint_EnableMotor(RecMakeJointId(rdr, a.joint), a.enableMotor)
}

@(private = "file")
RecDispatch_SphericalJointSetMotorVelocity :: proc(a: ^RecArgs_SphericalJointSetMotorVelocity, rdr: ^RecReader) {
	SphericalJoint_SetMotorVelocity(RecMakeJointId(rdr, a.joint), a.motorVelocity)
}

@(private = "file")
RecDispatch_SphericalJointSetMaxMotorTorque :: proc(a: ^RecArgs_SphericalJointSetMaxMotorTorque, rdr: ^RecReader) {
	SphericalJoint_SetMaxMotorTorque(RecMakeJointId(rdr, a.joint), a.torque)
}

@(private = "file")
RecDispatch_WeldJointSetLinearHertz :: proc(a: ^RecArgs_WeldJointSetLinearHertz, rdr: ^RecReader) {
	WeldJoint_SetLinearHertz(RecMakeJointId(rdr, a.joint), a.hertz)
}

@(private = "file")
RecDispatch_WeldJointSetLinearDampingRatio :: proc(a: ^RecArgs_WeldJointSetLinearDampingRatio, rdr: ^RecReader) {
	WeldJoint_SetLinearDampingRatio(RecMakeJointId(rdr, a.joint), a.dampingRatio)
}

@(private = "file")
RecDispatch_WeldJointSetAngularHertz :: proc(a: ^RecArgs_WeldJointSetAngularHertz, rdr: ^RecReader) {
	WeldJoint_SetAngularHertz(RecMakeJointId(rdr, a.joint), a.hertz)
}

@(private = "file")
RecDispatch_WeldJointSetAngularDampingRatio :: proc(a: ^RecArgs_WeldJointSetAngularDampingRatio, rdr: ^RecReader) {
	WeldJoint_SetAngularDampingRatio(RecMakeJointId(rdr, a.joint), a.dampingRatio)
}

@(private = "file")
RecDispatch_WheelJointEnableSuspension :: proc(a: ^RecArgs_WheelJointEnableSuspension, rdr: ^RecReader) {
	WheelJoint_EnableSuspension(RecMakeJointId(rdr, a.joint), a.flag)
}

@(private = "file")
RecDispatch_WheelJointSetSuspensionHertz :: proc(a: ^RecArgs_WheelJointSetSuspensionHertz, rdr: ^RecReader) {
	WheelJoint_SetSuspensionHertz(RecMakeJointId(rdr, a.joint), a.hertz)
}

@(private = "file")
RecDispatch_WheelJointSetSuspensionDampingRatio :: proc(
	a: ^RecArgs_WheelJointSetSuspensionDampingRatio,
	rdr: ^RecReader,
) {
	WheelJoint_SetSuspensionDampingRatio(RecMakeJointId(rdr, a.joint), a.dampingRatio)
}

@(private = "file")
RecDispatch_WheelJointEnableSuspensionLimit :: proc(a: ^RecArgs_WheelJointEnableSuspensionLimit, rdr: ^RecReader) {
	WheelJoint_EnableSuspensionLimit(RecMakeJointId(rdr, a.joint), a.flag)
}

@(private = "file")
RecDispatch_WheelJointSetSuspensionLimits :: proc(a: ^RecArgs_WheelJointSetSuspensionLimits, rdr: ^RecReader) {
	WheelJoint_SetSuspensionLimits(RecMakeJointId(rdr, a.joint), a.lower, a.upper)
}

@(private = "file")
RecDispatch_WheelJointEnableSpinMotor :: proc(a: ^RecArgs_WheelJointEnableSpinMotor, rdr: ^RecReader) {
	WheelJoint_EnableSpinMotor(RecMakeJointId(rdr, a.joint), a.flag)
}

@(private = "file")
RecDispatch_WheelJointSetSpinMotorSpeed :: proc(a: ^RecArgs_WheelJointSetSpinMotorSpeed, rdr: ^RecReader) {
	WheelJoint_SetSpinMotorSpeed(RecMakeJointId(rdr, a.joint), a.speed)
}

@(private = "file")
RecDispatch_WheelJointSetMaxSpinTorque :: proc(a: ^RecArgs_WheelJointSetMaxSpinTorque, rdr: ^RecReader) {
	WheelJoint_SetMaxSpinTorque(RecMakeJointId(rdr, a.joint), a.torque)
}

@(private = "file")
RecDispatch_WheelJointEnableSteering :: proc(a: ^RecArgs_WheelJointEnableSteering, rdr: ^RecReader) {
	WheelJoint_EnableSteering(RecMakeJointId(rdr, a.joint), a.flag)
}

@(private = "file")
RecDispatch_WheelJointSetSteeringHertz :: proc(a: ^RecArgs_WheelJointSetSteeringHertz, rdr: ^RecReader) {
	WheelJoint_SetSteeringHertz(RecMakeJointId(rdr, a.joint), a.hertz)
}

@(private = "file")
RecDispatch_WheelJointSetSteeringDampingRatio :: proc(a: ^RecArgs_WheelJointSetSteeringDampingRatio, rdr: ^RecReader) {
	WheelJoint_SetSteeringDampingRatio(RecMakeJointId(rdr, a.joint), a.dampingRatio)
}

@(private = "file")
RecDispatch_WheelJointSetMaxSteeringTorque :: proc(a: ^RecArgs_WheelJointSetMaxSteeringTorque, rdr: ^RecReader) {
	WheelJoint_SetMaxSteeringTorque(RecMakeJointId(rdr, a.joint), a.torque)
}

@(private = "file")
RecDispatch_WheelJointEnableSteeringLimit :: proc(a: ^RecArgs_WheelJointEnableSteeringLimit, rdr: ^RecReader) {
	WheelJoint_EnableSteeringLimit(RecMakeJointId(rdr, a.joint), a.flag)
}

@(private = "file")
RecDispatch_WheelJointSetSteeringLimits :: proc(a: ^RecArgs_WheelJointSetSteeringLimits, rdr: ^RecReader) {
	WheelJoint_SetSteeringLimits(RecMakeJointId(rdr, a.joint), a.lower, a.upper)
}

@(private = "file")
RecDispatch_WheelJointSetTargetSteeringAngle :: proc(a: ^RecArgs_WheelJointSetTargetSteeringAngle, rdr: ^RecReader) {
	WheelJoint_SetTargetSteeringAngle(RecMakeJointId(rdr, a.joint), a.radians)
}

@(private = "file")
RecDispatch_MoverJointSetLinearVelocity :: proc(a: ^RecArgs_MoverJointSetLinearVelocity, rdr: ^RecReader) {
	MoverJoint_SetLinearVelocity(RecMakeJointId(rdr, a.joint), a.velocity)
}

@(private = "file")
RecDispatch_MoverJointSetMaxVelocityForce :: proc(a: ^RecArgs_MoverJointSetMaxVelocityForce, rdr: ^RecReader) {
	MoverJoint_SetMaxVelocityForce(RecMakeJointId(rdr, a.joint), a.maxForce)
}

@(private = "file")
RecDispatch_PogoJointSetRestLength :: proc(a: ^RecArgs_PogoJointSetRestLength, rdr: ^RecReader) {
	PogoJoint_SetRestLength(RecMakeJointId(rdr, a.joint), a.length)
}

@(private = "file")
RecDispatch_PogoJointSetSpringHertz :: proc(a: ^RecArgs_PogoJointSetSpringHertz, rdr: ^RecReader) {
	PogoJoint_SetSpringHertz(RecMakeJointId(rdr, a.joint), a.hertz)
}

@(private = "file")
RecDispatch_PogoJointSetSpringDampingRatio :: proc(a: ^RecArgs_PogoJointSetSpringDampingRatio, rdr: ^RecReader) {
	PogoJoint_SetSpringDampingRatio(RecMakeJointId(rdr, a.joint), a.dampingRatio)
}

@(private = "file")
RecDispatch_PogoJointSetMaxTensionForce :: proc(a: ^RecArgs_PogoJointSetMaxTensionForce, rdr: ^RecReader) {
	PogoJoint_SetMaxTensionForce(RecMakeJointId(rdr, a.joint), a.force)
}

@(private = "file")
RecDispatch_PogoJointSetMaxCompressionForce :: proc(a: ^RecArgs_PogoJointSetMaxCompressionForce, rdr: ^RecReader) {
	PogoJoint_SetMaxCompressionForce(RecMakeJointId(rdr, a.joint), a.force)
}

@(private = "file")
RecDispatch_PogoJointSetNormal :: proc(a: ^RecArgs_PogoJointSetNormal, rdr: ^RecReader) {
	PogoJoint_SetNormal(RecMakeJointId(rdr, a.joint), a.normal)
}

@(private = "file")
RecDispatch_StateHash :: proc(a: ^RecArgs_StateHash, rdr: ^RecReader) {
	world := GetWorldFromId(rdr.replayWorldId)
	computed := HashWorldState(world)
	if computed != a.hash {
		fmt.printf("b3ReplayFile: StateHash mismatch (recorded=0x%x, computed=0x%x)\n", a.hash, computed)
		rdr.diverged = true
	}
}

@(private = "file")
RecDispatch_RecordingBounds :: proc(a: ^RecArgs_RecordingBounds, rdr: ^RecReader) {
	// Primary resolve is the open-time scan, this keeps the value right if it ever moves earlier
	if rdr.owner != nil {
		rdr.owner.bounds = a.bounds
	}
}

// Spatial query replay. The recorded inputs come through the manifest; here the variable-length hit
// tail is read back, the query re-issued against the replay world, and each callback hit compared to
// what was recorded. Any mismatch latches rdr->diverged. When a player owns the reader the hits are
// also stashed for the viewer overlay. The stash helpers dereference the player struct, defined later
// in this file, so they are forward declared and implemented in Block B below.
// (RecGrow, RecStashQueryBegin)

// Grow the reader's hit scratch to at least n entries, preserving contents. n is bounded by the file
// size since every recorded hit consumes at least one byte, so a corrupt count fails the read.
RecEnsureHits :: proc(rdr: ^RecReader, n: i32) {
	RecReserveScratch(rdr, (^rawptr)(&rdr.hits), &rdr.hitCap, n, i32(size_of(RecRecordedHit)))
}

// Bitwise float compare so the determinism check is exact, not within a tolerance.
@(private = "file")
RecF32Differs :: proc(a: f32, b: f32) -> bool {
	ua := transmute(u32)a
	ub := transmute(u32)b
	return ua != ub
}

@(private = "file")
RecVec3Differs :: proc(a: Vec3, b: Vec3) -> bool {
	return RecF32Differs(a.x, b.x) || RecF32Differs(a.y, b.y) || RecF32Differs(a.z, b.z)
}

// Shared context for the replay trampolines: walks recorded hits in order, flagging any divergence
// from the re-issued query.
@(private = "file")
RecReplayQueryCtx :: struct {
	rdr:    ^RecReader,
	hits:   [^]RecRecordedHit,
	count:  i32,
	cursor: i32,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
RecReplayOverlapTrampoline :: proc "c" (id: ShapeId, ctx: rawptr) -> bool {
	// unified end
	rc := (^RecReplayQueryCtx)(ctx)
	if rc.cursor >= rc.count {
		rc.rdr.diverged = true
		return false
	}
	h := &rc.hits[rc.cursor]
	rc.cursor += 1
	if id.index1 != h.id.index1 || id.generation != h.id.generation {
		rc.rdr.diverged = true
	}
	return h.userReturnB
}

// The mover filter has the same bool(shapeId, ctx) shape as an overlap callback, so it replays the
// same way. A distinct typed wrapper keeps the function-pointer types clean.
@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
RecReplayMoverFilterTrampoline :: proc "c" (id: ShapeId, ctx: rawptr) -> bool {
	// unified end
	return RecReplayOverlapTrampoline(id, ctx)
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
RecReplayCastTrampoline :: proc "c" (
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
	rc := (^RecReplayQueryCtx)(ctx)
	if rc.cursor >= rc.count {
		rc.rdr.diverged = true
		return 0.0
	}
	h := &rc.hits[rc.cursor]
	rc.cursor += 1
	// Positions compared through a full-width delta, truncating both sides would pass vacuously far
	// from the origin.
	if id.index1 != h.id.index1 ||
	   id.generation != h.id.generation ||
	   RecVec3Differs(SubPos(point, h.point), Vec3_zero) ||
	   RecVec3Differs(normal, h.normal) ||
	   RecF32Differs(fraction, h.fraction) ||
	   userMaterialId != h.userMaterialId ||
	   triangleIndex != h.triangleIndex ||
	   childIndex != h.childIndex {
		rc.rdr.diverged = true
	}
	return h.userReturnF
}

// 3D delivers a whole shape's planes in one call. The recorded group starts at the cursor with its
// plane count and user return replicated on each hit, so compare the batch and advance by the
// recorded count to stay aligned with the stream even when the count itself diverged.
@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
RecReplayPlaneTrampoline :: proc "c" (id: ShapeId, planes: ^PlaneResult, planeCount: i32, ctx: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	rc := (^RecReplayQueryCtx)(ctx)
	if rc.cursor >= rc.count {
		rc.rdr.diverged = true
		return true
	}
	planeArray := ([^]PlaneResult)(planes)
	head := &rc.hits[rc.cursor]
	recordedCount := head.planeCount
	ret := head.userReturnB
	if id.index1 != head.id.index1 || id.generation != head.id.generation || recordedCount != planeCount {
		rc.rdr.diverged = true
	}
	n := recordedCount < planeCount ? recordedCount : planeCount
	for i: i32 = 0; i < n; i += 1 {
		h := &rc.hits[rc.cursor + i]
		if RecVec3Differs(h.plane.plane.normal, planeArray[i].plane.normal) ||
		   RecF32Differs(h.plane.plane.offset, planeArray[i].plane.offset) ||
		   RecVec3Differs(h.plane.point, planeArray[i].point) ||
		   h.plane.triangleIndex != planeArray[i].triangleIndex ||
		   h.plane.childIndex != planeArray[i].childIndex ||
		   h.plane.materialIndex != planeArray[i].materialIndex {
			rc.rdr.diverged = true
		}
	}
	rc.cursor += recordedCount
	return ret
}

// Copy a decoded proxy's points into a draw record so the overlay does not depend on reader scratch.
@(private = "file")
RecStashProxy :: proc(q: ^RecDrawQuery, proxy: ^ShapeProxy) {
	count := proxy.count
	if count > MAX_SHAPE_CAST_POINTS {
		count = MAX_SHAPE_CAST_POINTS
	}
	q.proxyCount = count
	q.proxyRadius = proxy.radius
	for i: i32 = 0; i < count; i += 1 {
		q.proxyPoints[i] = proxy.points[i]
	}
}

// Tight world-space bounds of a query's swept geometry, so the viewer can frame any query and not
// just the overlap AABB. Mover and proxy points are origin relative. A cast sweeps the shape from the
// origin to origin plus translation. The overlap AABB is already a world-space box.
@(private = "file")
RecComputeQueryBounds :: proc(q: ^RecDrawQuery) {
	if q.kind == i32(RecQueryKind.RECQ_OVERLAP_AABB) {
		return
	}

	// Shape points relative to the origin, plus the fattening radius. A ray has no shape, so it falls
	// through to a single point at the origin.
	local: [MAX_SHAPE_CAST_POINTS]Vec3
	count: i32 = 0
	radius: f32 = 0.0
	#partial switch RecQueryKind(q.kind) {
	case .RECQ_CAST_MOVER, .RECQ_COLLIDE_MOVER:
		local[0] = q.mover.center1
		local[1] = q.mover.center2
		count = 2
		radius = q.mover.radius

	case .RECQ_OVERLAP_SHAPE, .RECQ_CAST_SHAPE:
		count = q.proxyCount
		for i: i32 = 0; i < count; i += 1 {
			local[i] = q.proxyPoints[i]
		}
		radius = q.proxyRadius

	case:
	}
	if count == 0 {
		local[0] = Vec3_zero
		count = 1
	}

	// Sweep each point across the translation. A non-cast query has zero translation, so both ends
	// coincide and the duplicates fold away.
	end := OffsetPos(q.origin, q.translation)
	world: [2 * MAX_SHAPE_CAST_POINTS]Vec3 = {}
	n: i32 = 0
	for i: i32 = 0; i < count; i += 1 {
		world[n] = ToVec3(OffsetPos(q.origin, local[i]))
		n += 1
		world[n] = ToVec3(OffsetPos(end, local[i]))
		n += 1
	}
	q.aabb = MakeAABB(&world[0], n, radius)
}

@(private = "file")
RecDispatch_QueryOverlapAABB :: proc(a: ^RecArgs_QueryOverlapAABB, rdr: ^RecReader) {
	n := RecR_U32(rdr)
	RecEnsureHits(rdr, i32(n))
	if !rdr.ok {
		return
	}
	for i: u32 = 0; i < n; i += 1 {
		rdr.hits[i].id = RecMakeShapeId(rdr, RecR_SHAPEID(rdr))
		rdr.hits[i].userReturnB = RecR_BOOL(rdr)
	}
	_ = RecR_TREESTATS(rdr)
	if !rdr.ok {
		return
	}
	rc := RecReplayQueryCtx{rdr, rdr.hits, i32(n), 0}
	World_OverlapAABB(rdr.replayWorldId, a.aabb, a.filter, RecReplayOverlapTrampoline, &rc)
	if rc.cursor != i32(n) {
		rdr.diverged = true
	}
	if rdr.owner != nil {
		q := RecStashQueryBegin(rdr.owner, i32(RecQueryKind.RECQ_OVERLAP_AABB), rdr.hits, i32(n))
		q.filter = a.filter
		q.aabb = a.aabb
		RecComputeQueryBounds(q)
	}
}

@(private = "file")
RecDispatch_QueryOverlapShape :: proc(a: ^RecArgs_QueryOverlapShape, rdr: ^RecReader) {
	n := RecR_U32(rdr)
	RecEnsureHits(rdr, i32(n))
	if !rdr.ok {
		return
	}
	for i: u32 = 0; i < n; i += 1 {
		rdr.hits[i].id = RecMakeShapeId(rdr, RecR_SHAPEID(rdr))
		rdr.hits[i].userReturnB = RecR_BOOL(rdr)
	}
	_ = RecR_TREESTATS(rdr)
	if !rdr.ok {
		return
	}
	rc := RecReplayQueryCtx{rdr, rdr.hits, i32(n), 0}
	World_OverlapShape(rdr.replayWorldId, a.origin, &a.proxy, a.filter, RecReplayOverlapTrampoline, &rc)
	if rc.cursor != i32(n) {
		rdr.diverged = true
	}
	if rdr.owner != nil {
		q := RecStashQueryBegin(rdr.owner, i32(RecQueryKind.RECQ_OVERLAP_SHAPE), rdr.hits, i32(n))
		q.filter = a.filter
		q.origin = a.origin
		RecStashProxy(q, &a.proxy)
		RecComputeQueryBounds(q)
	}
}

@(private = "file")
RecDispatch_QueryCastRay :: proc(a: ^RecArgs_QueryCastRay, rdr: ^RecReader) {
	n := RecR_U32(rdr)
	RecEnsureHits(rdr, i32(n))
	if !rdr.ok {
		return
	}
	for i: u32 = 0; i < n; i += 1 {
		rdr.hits[i].id = RecMakeShapeId(rdr, RecR_SHAPEID(rdr))
		rdr.hits[i].point = RecR_POSITION(rdr)
		rdr.hits[i].normal = RecR_VEC3(rdr)
		rdr.hits[i].fraction = RecR_F32(rdr)
		rdr.hits[i].userMaterialId = RecR_U64(rdr)
		rdr.hits[i].triangleIndex = RecR_I32(rdr)
		rdr.hits[i].childIndex = RecR_I32(rdr)
		rdr.hits[i].userReturnF = RecR_F32(rdr)
	}
	_ = RecR_TREESTATS(rdr)
	if !rdr.ok {
		return
	}
	rc := RecReplayQueryCtx{rdr, rdr.hits, i32(n), 0}
	World_CastRay(rdr.replayWorldId, a.origin, a.translation, a.filter, RecReplayCastTrampoline, &rc)
	if rc.cursor != i32(n) {
		rdr.diverged = true
	}
	if rdr.owner != nil {
		q := RecStashQueryBegin(rdr.owner, i32(RecQueryKind.RECQ_CAST_RAY), rdr.hits, i32(n))
		q.filter = a.filter
		q.origin = a.origin
		q.translation = a.translation
		RecComputeQueryBounds(q)
	}
}

@(private = "file")
RecDispatch_QueryCastShape :: proc(a: ^RecArgs_QueryCastShape, rdr: ^RecReader) {
	n := RecR_U32(rdr)
	RecEnsureHits(rdr, i32(n))
	if !rdr.ok {
		return
	}
	for i: u32 = 0; i < n; i += 1 {
		rdr.hits[i].id = RecMakeShapeId(rdr, RecR_SHAPEID(rdr))
		rdr.hits[i].point = RecR_POSITION(rdr)
		rdr.hits[i].normal = RecR_VEC3(rdr)
		rdr.hits[i].fraction = RecR_F32(rdr)
		rdr.hits[i].userMaterialId = RecR_U64(rdr)
		rdr.hits[i].triangleIndex = RecR_I32(rdr)
		rdr.hits[i].childIndex = RecR_I32(rdr)
		rdr.hits[i].userReturnF = RecR_F32(rdr)
	}
	_ = RecR_TREESTATS(rdr)
	if !rdr.ok {
		return
	}
	rc := RecReplayQueryCtx{rdr, rdr.hits, i32(n), 0}
	World_CastShape(rdr.replayWorldId, a.origin, &a.proxy, a.translation, a.filter, RecReplayCastTrampoline, &rc)
	if rc.cursor != i32(n) {
		rdr.diverged = true
	}
	if rdr.owner != nil {
		q := RecStashQueryBegin(rdr.owner, i32(RecQueryKind.RECQ_CAST_SHAPE), rdr.hits, i32(n))
		q.filter = a.filter
		q.origin = a.origin
		q.translation = a.translation
		RecStashProxy(q, &a.proxy)
		RecComputeQueryBounds(q)
	}
}

@(private = "file")
RecDispatch_QueryCastRayClosest :: proc(a: ^RecArgs_QueryCastRayClosest, rdr: ^RecReader) {
	rec := RecR_RAYRESULT(rdr)
	if !rdr.ok {
		return
	}
	got := World_CastRayClosest(rdr.replayWorldId, a.origin, a.translation, a.filter)
	recId := RecMakeShapeId(rdr, rec.shapeId)
	if got.hit != rec.hit ||
	   (got.hit &&
			   (got.shapeId.index1 != recId.index1 ||
					   got.shapeId.generation != recId.generation ||
					   RecVec3Differs(SubPos(got.point, rec.point), Vec3_zero) ||
					   RecVec3Differs(got.normal, rec.normal) ||
					   RecF32Differs(got.fraction, rec.fraction) ||
					   got.userMaterialId != rec.userMaterialId)) {
		rdr.diverged = true
	}
	if rdr.owner != nil {
		// Stash the closest result as a single pooled hit so the shared draw loop renders its point.
		h := RecRecordedHit{}
		h.id = recId
		h.point = rec.point
		h.normal = rec.normal
		h.fraction = rec.fraction
		q := RecStashQueryBegin(rdr.owner, i32(RecQueryKind.RECQ_CAST_RAY_CLOSEST), &h, rec.hit ? 1 : 0)
		q.filter = a.filter
		q.origin = a.origin
		q.translation = a.translation
		q.rayResult = rec
		RecComputeQueryBounds(q)
	}
}

@(private = "file")
RecDispatch_QueryCastMover :: proc(a: ^RecArgs_QueryCastMover, rdr: ^RecReader) {
	n := RecR_U32(rdr)
	RecEnsureHits(rdr, i32(n))
	if !rdr.ok {
		return
	}
	for i: u32 = 0; i < n; i += 1 {
		rdr.hits[i].id = RecMakeShapeId(rdr, RecR_SHAPEID(rdr))
		rdr.hits[i].userReturnB = RecR_BOOL(rdr)
	}
	recFraction := RecR_F32(rdr)
	if !rdr.ok {
		return
	}
	rc := RecReplayQueryCtx{rdr, rdr.hits, i32(n), 0}
	got := World_CastMover(
		rdr.replayWorldId,
		a.origin,
		&a.mover,
		a.translation,
		a.filter,
		RecReplayMoverFilterTrampoline,
		&rc,
	)
	if rc.cursor != i32(n) || RecF32Differs(got, recFraction) {
		rdr.diverged = true
	}
	if rdr.owner != nil {
		q := RecStashQueryBegin(rdr.owner, i32(RecQueryKind.RECQ_CAST_MOVER), nil, 0)
		q.filter = a.filter
		q.origin = a.origin
		q.mover = a.mover
		q.translation = a.translation
		q.castFraction = recFraction
		RecComputeQueryBounds(q)
	}
}

@(private = "file")
RecDispatch_QueryCollideMover :: proc(a: ^RecArgs_QueryCollideMover, rdr: ^RecReader) {
	// Recorded as shapeCount groups, each: shapeId, planeCount, planeCount planes, user return. Flatten
	// into one hit per plane with the group's count and return replicated, so the replay walker can
	// re-group per shape.
	shapeCount := RecR_U32(rdr)
	total: i32 = 0
	for s: u32 = 0; s < shapeCount; s += 1 {
		id := RecMakeShapeId(rdr, RecR_SHAPEID(rdr))
		planeCount := RecR_I32(rdr)
		if planeCount < 0 {
			planeCount = 0
		}
		RecEnsureHits(rdr, total + planeCount)
		if !rdr.ok {
			return
		}
		for i: i32 = 0; i < planeCount; i += 1 {
			rdr.hits[total + i].plane = RecR_PLANERESULT(rdr)
		}
		ret := RecR_BOOL(rdr)
		for i: i32 = 0; i < planeCount; i += 1 {
			rdr.hits[total + i].id = id
			rdr.hits[total + i].planeCount = planeCount
			rdr.hits[total + i].userReturnB = ret
		}
		total += planeCount
	}
	if !rdr.ok {
		return
	}
	rc := RecReplayQueryCtx{rdr, rdr.hits, total, 0}
	World_CollideMover(rdr.replayWorldId, a.origin, &a.mover, a.filter, RecReplayPlaneTrampoline, &rc)
	if rc.cursor != total {
		rdr.diverged = true
	}
	if rdr.owner != nil {
		q := RecStashQueryBegin(rdr.owner, i32(RecQueryKind.RECQ_COLLIDE_MOVER), rdr.hits, total)
		q.filter = a.filter
		q.origin = a.origin
		q.mover = a.mover
		RecComputeQueryBounds(q)
	}
}

// Stash the identity key of the query that immediately follows. Consumed by the next stash.
@(private = "file")
RecDispatch_QueryTag :: proc(a: ^RecArgs_QueryTag, rdr: ^RecReader) {
	rdr.pendingQueryKey = a.key
}

// X-macro dispatch switch: read opcode+u24 payloadSize, dispatch, skip unknown ops.
// Returns the opcode dispatched, or -1 when the stream is exhausted or broken.

@(private = "file")
RecDispatchOne :: proc(rdr: ^RecReader) -> i32 {
	if rdr.cursor >= rdr.size || !rdr.ok {
		return -1
	}
	opcode := RecR_U8(rdr)
	payloadSize := RecR_U24(rdr)
	if !rdr.ok {
		return -1
	}

	payloadStart := rdr.cursor

	// unified: the X-macro expansion over recording_ops.inl is written out, one case per op in manifest order
	switch opcode {
	case recOpDestroyWorld:
		a: RecArgs_DestroyWorld
		a.world = RecR_WORLDID(rdr)
		if rdr.ok {
			RecDispatch_DestroyWorld(&a, rdr)
		}
	case recOpStep:
		a: RecArgs_Step
		a.world = RecR_WORLDID(rdr)
		a.dt = RecR_F32(rdr)
		a.subStepCount = RecR_I32(rdr)
		if rdr.ok {
			RecDispatch_Step(&a, rdr)
		}
	case recOpWorldEnableSleeping:
		a: RecArgs_WorldEnableSleeping
		a.world = RecR_WORLDID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_WorldEnableSleeping(&a, rdr)
		}
	case recOpWorldEnableContinuous:
		a: RecArgs_WorldEnableContinuous
		a.world = RecR_WORLDID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_WorldEnableContinuous(&a, rdr)
		}
	case recOpWorldSetRestitutionThreshold:
		a: RecArgs_WorldSetRestitutionThreshold
		a.world = RecR_WORLDID(rdr)
		a.value = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WorldSetRestitutionThreshold(&a, rdr)
		}
	case recOpWorldSetHitEventThreshold:
		a: RecArgs_WorldSetHitEventThreshold
		a.world = RecR_WORLDID(rdr)
		a.value = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WorldSetHitEventThreshold(&a, rdr)
		}
	case recOpWorldSetGravity:
		a: RecArgs_WorldSetGravity
		a.world = RecR_WORLDID(rdr)
		a.gravity = RecR_VEC3(rdr)
		if rdr.ok {
			RecDispatch_WorldSetGravity(&a, rdr)
		}
	case recOpWorldExplode:
		a: RecArgs_WorldExplode
		a.world = RecR_WORLDID(rdr)
		a.def = RecR_EXPLOSIONDEF(rdr)
		if rdr.ok {
			RecDispatch_WorldExplode(&a, rdr)
		}
	case recOpWorldSetContactTuning:
		a: RecArgs_WorldSetContactTuning
		a.world = RecR_WORLDID(rdr)
		a.hertz = RecR_F32(rdr)
		a.dampingRatio = RecR_F32(rdr)
		a.contactSpeed = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WorldSetContactTuning(&a, rdr)
		}
	case recOpWorldSetContactRecycleDistance:
		a: RecArgs_WorldSetContactRecycleDistance
		a.world = RecR_WORLDID(rdr)
		a.recycleDistance = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WorldSetContactRecycleDistance(&a, rdr)
		}
	case recOpWorldSetMaximumLinearSpeed:
		a: RecArgs_WorldSetMaximumLinearSpeed
		a.world = RecR_WORLDID(rdr)
		a.maximumLinearSpeed = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WorldSetMaximumLinearSpeed(&a, rdr)
		}
	case recOpWorldEnableWarmStarting:
		a: RecArgs_WorldEnableWarmStarting
		a.world = RecR_WORLDID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_WorldEnableWarmStarting(&a, rdr)
		}
	case recOpWorldRebuildStaticTree:
		a: RecArgs_WorldRebuildStaticTree
		a.world = RecR_WORLDID(rdr)
		if rdr.ok {
			RecDispatch_WorldRebuildStaticTree(&a, rdr)
		}
	case recOpWorldEnableSpeculative:
		a: RecArgs_WorldEnableSpeculative
		a.world = RecR_WORLDID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_WorldEnableSpeculative(&a, rdr)
		}
	case recOpWorldSetRestitutionIterations:
		a: RecArgs_WorldSetRestitutionIterations
		a.world = RecR_WORLDID(rdr)
		a.iterations = RecR_I32(rdr)
		if rdr.ok {
			RecDispatch_WorldSetRestitutionIterations(&a, rdr)
		}
	case recOpWorldEnableRestitutionPropagation:
		a: RecArgs_WorldEnableRestitutionPropagation
		a.world = RecR_WORLDID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_WorldEnableRestitutionPropagation(&a, rdr)
		}
	case recOpWorldSetIsotropicInertiaTolerance:
		a: RecArgs_WorldSetIsotropicInertiaTolerance
		a.world = RecR_WORLDID(rdr)
		a.tolerance = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WorldSetIsotropicInertiaTolerance(&a, rdr)
		}
	case recOpCreateBody:
		a: RecArgs_CreateBody
		a.world = RecR_WORLDID(rdr)
		a.def = RecR_BODYDEF(rdr)
		if rdr.ok {
			RecDispatch_CreateBody(&a, rdr)
		}
	case recOpDestroyBody:
		a: RecArgs_DestroyBody
		a.body = RecR_BODYID(rdr)
		if rdr.ok {
			RecDispatch_DestroyBody(&a, rdr)
		}
	case recOpBodySetTransform:
		a: RecArgs_BodySetTransform
		a.body = RecR_BODYID(rdr)
		a.position = RecR_POSITION(rdr)
		a.rotation = RecR_QUAT(rdr)
		if rdr.ok {
			RecDispatch_BodySetTransform(&a, rdr)
		}
	case recOpBodySetLinearVelocity:
		a: RecArgs_BodySetLinearVelocity
		a.body = RecR_BODYID(rdr)
		a.v = RecR_VEC3(rdr)
		if rdr.ok {
			RecDispatch_BodySetLinearVelocity(&a, rdr)
		}
	case recOpBodySetType:
		a: RecArgs_BodySetType
		a.body = RecR_BODYID(rdr)
		a.type = RecR_I32(rdr)
		if rdr.ok {
			RecDispatch_BodySetType(&a, rdr)
		}
	case recOpBodySetName:
		a: RecArgs_BodySetName
		a.body = RecR_BODYID(rdr)
		a.name = RecR_STR(rdr)
		if rdr.ok {
			RecDispatch_BodySetName(&a, rdr)
		}
	case recOpBodySetAngularVelocity:
		a: RecArgs_BodySetAngularVelocity
		a.body = RecR_BODYID(rdr)
		a.w = RecR_VEC3(rdr)
		if rdr.ok {
			RecDispatch_BodySetAngularVelocity(&a, rdr)
		}
	case recOpBodySetTargetTransform:
		a: RecArgs_BodySetTargetTransform
		a.body = RecR_BODYID(rdr)
		a.target = RecR_WORLDXF(rdr)
		a.timeStep = RecR_F32(rdr)
		a.wake = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_BodySetTargetTransform(&a, rdr)
		}
	case recOpBodyApplyForce:
		a: RecArgs_BodyApplyForce
		a.body = RecR_BODYID(rdr)
		a.force = RecR_VEC3(rdr)
		a.point = RecR_POSITION(rdr)
		a.wake = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_BodyApplyForce(&a, rdr)
		}
	case recOpBodyApplyForceToCenter:
		a: RecArgs_BodyApplyForceToCenter
		a.body = RecR_BODYID(rdr)
		a.force = RecR_VEC3(rdr)
		a.wake = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_BodyApplyForceToCenter(&a, rdr)
		}
	case recOpBodyApplyTorque:
		a: RecArgs_BodyApplyTorque
		a.body = RecR_BODYID(rdr)
		a.torque = RecR_VEC3(rdr)
		a.wake = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_BodyApplyTorque(&a, rdr)
		}
	case recOpBodyApplyLinearImpulse:
		a: RecArgs_BodyApplyLinearImpulse
		a.body = RecR_BODYID(rdr)
		a.impulse = RecR_VEC3(rdr)
		a.point = RecR_POSITION(rdr)
		a.wake = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_BodyApplyLinearImpulse(&a, rdr)
		}
	case recOpBodyApplyLinearImpulseToCenter:
		a: RecArgs_BodyApplyLinearImpulseToCenter
		a.body = RecR_BODYID(rdr)
		a.impulse = RecR_VEC3(rdr)
		a.wake = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_BodyApplyLinearImpulseToCenter(&a, rdr)
		}
	case recOpBodyApplyAngularImpulse:
		a: RecArgs_BodyApplyAngularImpulse
		a.body = RecR_BODYID(rdr)
		a.impulse = RecR_VEC3(rdr)
		a.wake = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_BodyApplyAngularImpulse(&a, rdr)
		}
	case recOpBodySetMassData:
		a: RecArgs_BodySetMassData
		a.body = RecR_BODYID(rdr)
		a.massData = RecR_MASSDATA(rdr)
		if rdr.ok {
			RecDispatch_BodySetMassData(&a, rdr)
		}
	case recOpBodyApplyMassFromShapes:
		a: RecArgs_BodyApplyMassFromShapes
		a.body = RecR_BODYID(rdr)
		if rdr.ok {
			RecDispatch_BodyApplyMassFromShapes(&a, rdr)
		}
	case recOpBodySetLinearDamping:
		a: RecArgs_BodySetLinearDamping
		a.body = RecR_BODYID(rdr)
		a.damping = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_BodySetLinearDamping(&a, rdr)
		}
	case recOpBodySetAngularDamping:
		a: RecArgs_BodySetAngularDamping
		a.body = RecR_BODYID(rdr)
		a.damping = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_BodySetAngularDamping(&a, rdr)
		}
	case recOpBodySetGravityScale:
		a: RecArgs_BodySetGravityScale
		a.body = RecR_BODYID(rdr)
		a.scale = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_BodySetGravityScale(&a, rdr)
		}
	case recOpBodySetAwake:
		a: RecArgs_BodySetAwake
		a.body = RecR_BODYID(rdr)
		a.awake = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_BodySetAwake(&a, rdr)
		}
	case recOpBodyEnableSleep:
		a: RecArgs_BodyEnableSleep
		a.body = RecR_BODYID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_BodyEnableSleep(&a, rdr)
		}
	case recOpBodySetSleepThreshold:
		a: RecArgs_BodySetSleepThreshold
		a.body = RecR_BODYID(rdr)
		a.threshold = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_BodySetSleepThreshold(&a, rdr)
		}
	case recOpBodyDisable:
		a: RecArgs_BodyDisable
		a.body = RecR_BODYID(rdr)
		if rdr.ok {
			RecDispatch_BodyDisable(&a, rdr)
		}
	case recOpBodyEnable:
		a: RecArgs_BodyEnable
		a.body = RecR_BODYID(rdr)
		if rdr.ok {
			RecDispatch_BodyEnable(&a, rdr)
		}
	case recOpBodySetMotionLocks:
		a: RecArgs_BodySetMotionLocks
		a.body = RecR_BODYID(rdr)
		a.locks = RecR_LOCKS(rdr)
		if rdr.ok {
			RecDispatch_BodySetMotionLocks(&a, rdr)
		}
	case recOpBodySetBullet:
		a: RecArgs_BodySetBullet
		a.body = RecR_BODYID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_BodySetBullet(&a, rdr)
		}
	case recOpBodyEnableContactRecycling:
		a: RecArgs_BodyEnableContactRecycling
		a.body = RecR_BODYID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_BodyEnableContactRecycling(&a, rdr)
		}
	case recOpBodyEnableHitEvents:
		a: RecArgs_BodyEnableHitEvents
		a.body = RecR_BODYID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_BodyEnableHitEvents(&a, rdr)
		}
	case recOpBodyAllowFastRotation:
		a: RecArgs_BodyAllowFastRotation
		a.body = RecR_BODYID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_BodyAllowFastRotation(&a, rdr)
		}
	case recOpBodySetSafetyFactor:
		a: RecArgs_BodySetSafetyFactor
		a.body = RecR_BODYID(rdr)
		a.value = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_BodySetSafetyFactor(&a, rdr)
		}
	case recOpCreateSphereShape:
		a: RecArgs_CreateSphereShape
		a.body = RecR_BODYID(rdr)
		a.def = RecR_SHAPEDEF(rdr)
		a.sphere = RecR_SPHERE(rdr)
		if rdr.ok {
			RecDispatch_CreateSphereShape(&a, rdr)
		}
	case recOpCreateCapsuleShape:
		a: RecArgs_CreateCapsuleShape
		a.body = RecR_BODYID(rdr)
		a.def = RecR_SHAPEDEF(rdr)
		a.capsule = RecR_CAPSULE(rdr)
		if rdr.ok {
			RecDispatch_CreateCapsuleShape(&a, rdr)
		}
	case recOpCreateHullShape:
		a: RecArgs_CreateHullShape
		a.body = RecR_BODYID(rdr)
		a.def = RecR_SHAPEDEF(rdr)
		a.geometryId = RecR_GEOMID(rdr)
		if rdr.ok {
			RecDispatch_CreateHullShape(&a, rdr)
		}
	case recOpCreateMeshShape:
		a: RecArgs_CreateMeshShape
		a.body = RecR_BODYID(rdr)
		a.def = RecR_SHAPEDEF(rdr)
		a.geometryId = RecR_GEOMID(rdr)
		a.scale = RecR_VEC3(rdr)
		if rdr.ok {
			RecDispatch_CreateMeshShape(&a, rdr)
		}
	case recOpCreateHeightFieldShape:
		a: RecArgs_CreateHeightFieldShape
		a.body = RecR_BODYID(rdr)
		a.def = RecR_SHAPEDEF(rdr)
		a.geometryId = RecR_GEOMID(rdr)
		if rdr.ok {
			RecDispatch_CreateHeightFieldShape(&a, rdr)
		}
	case recOpCreateCompoundShape:
		a: RecArgs_CreateCompoundShape
		a.body = RecR_BODYID(rdr)
		a.def = RecR_SHAPEDEF(rdr)
		a.geometryId = RecR_GEOMID(rdr)
		if rdr.ok {
			RecDispatch_CreateCompoundShape(&a, rdr)
		}
	case recOpDestroyShape:
		a: RecArgs_DestroyShape
		a.shape = RecR_SHAPEID(rdr)
		a.updateBodyMass = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_DestroyShape(&a, rdr)
		}
	case recOpShapeSetDensity:
		a: RecArgs_ShapeSetDensity
		a.shape = RecR_SHAPEID(rdr)
		a.density = RecR_F32(rdr)
		a.updateBodyMass = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_ShapeSetDensity(&a, rdr)
		}
	case recOpShapeSetFriction:
		a: RecArgs_ShapeSetFriction
		a.shape = RecR_SHAPEID(rdr)
		a.friction = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_ShapeSetFriction(&a, rdr)
		}
	case recOpShapeSetRestitution:
		a: RecArgs_ShapeSetRestitution
		a.shape = RecR_SHAPEID(rdr)
		a.restitution = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_ShapeSetRestitution(&a, rdr)
		}
	case recOpShapeSetSurfaceMaterial:
		a: RecArgs_ShapeSetSurfaceMaterial
		a.shape = RecR_SHAPEID(rdr)
		a.material = RecR_MATERIAL(rdr)
		if rdr.ok {
			RecDispatch_ShapeSetSurfaceMaterial(&a, rdr)
		}
	case recOpShapeSetFilter:
		a: RecArgs_ShapeSetFilter
		a.shape = RecR_SHAPEID(rdr)
		a.filter = RecR_FILTER(rdr)
		a.invokeContacts = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_ShapeSetFilter(&a, rdr)
		}
	case recOpShapeEnableSensorEvents:
		a: RecArgs_ShapeEnableSensorEvents
		a.shape = RecR_SHAPEID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_ShapeEnableSensorEvents(&a, rdr)
		}
	case recOpShapeEnableContactEvents:
		a: RecArgs_ShapeEnableContactEvents
		a.shape = RecR_SHAPEID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_ShapeEnableContactEvents(&a, rdr)
		}
	case recOpShapeEnablePreSolveEvents:
		a: RecArgs_ShapeEnablePreSolveEvents
		a.shape = RecR_SHAPEID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_ShapeEnablePreSolveEvents(&a, rdr)
		}
	case recOpShapeEnableHitEvents:
		a: RecArgs_ShapeEnableHitEvents
		a.shape = RecR_SHAPEID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_ShapeEnableHitEvents(&a, rdr)
		}
	case recOpShapeSetSphere:
		a: RecArgs_ShapeSetSphere
		a.shape = RecR_SHAPEID(rdr)
		a.sphere = RecR_SPHERE(rdr)
		if rdr.ok {
			RecDispatch_ShapeSetSphere(&a, rdr)
		}
	case recOpShapeSetCapsule:
		a: RecArgs_ShapeSetCapsule
		a.shape = RecR_SHAPEID(rdr)
		a.capsule = RecR_CAPSULE(rdr)
		if rdr.ok {
			RecDispatch_ShapeSetCapsule(&a, rdr)
		}
	case recOpShapeApplyWind:
		a: RecArgs_ShapeApplyWind
		a.shape = RecR_SHAPEID(rdr)
		a.wind = RecR_VEC3(rdr)
		a.drag = RecR_F32(rdr)
		a.lift = RecR_F32(rdr)
		a.maxSpeed = RecR_F32(rdr)
		a.wake = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_ShapeApplyWind(&a, rdr)
		}
	case recOpShapeSetName:
		a: RecArgs_ShapeSetName
		a.shape = RecR_SHAPEID(rdr)
		a.name = RecR_STR(rdr)
		if rdr.ok {
			RecDispatch_ShapeSetName(&a, rdr)
		}
	case recOpShapeSetMeshMaterial:
		a: RecArgs_ShapeSetMeshMaterial
		a.shape = RecR_SHAPEID(rdr)
		a.material = RecR_MATERIAL(rdr)
		a.index = RecR_I32(rdr)
		if rdr.ok {
			RecDispatch_ShapeSetMeshMaterial(&a, rdr)
		}
	case recOpShapeSetHull:
		a: RecArgs_ShapeSetHull
		a.shape = RecR_SHAPEID(rdr)
		a.geometryId = RecR_GEOMID(rdr)
		if rdr.ok {
			RecDispatch_ShapeSetHull(&a, rdr)
		}
	case recOpShapeSetMesh:
		a: RecArgs_ShapeSetMesh
		a.shape = RecR_SHAPEID(rdr)
		a.geometryId = RecR_GEOMID(rdr)
		a.scale = RecR_VEC3(rdr)
		if rdr.ok {
			RecDispatch_ShapeSetMesh(&a, rdr)
		}
	case recOpCreateParallelJoint:
		a: RecArgs_CreateParallelJoint
		a.world = RecR_WORLDID(rdr)
		a.def = RecR_PARALLELJOINTDEF(rdr)
		if rdr.ok {
			RecDispatch_CreateParallelJoint(&a, rdr)
		}
	case recOpCreateDistanceJoint:
		a: RecArgs_CreateDistanceJoint
		a.world = RecR_WORLDID(rdr)
		a.def = RecR_DISTANCEJOINTDEF(rdr)
		if rdr.ok {
			RecDispatch_CreateDistanceJoint(&a, rdr)
		}
	case recOpCreateFilterJoint:
		a: RecArgs_CreateFilterJoint
		a.world = RecR_WORLDID(rdr)
		a.def = RecR_FILTERJOINTDEF(rdr)
		if rdr.ok {
			RecDispatch_CreateFilterJoint(&a, rdr)
		}
	case recOpCreateMotorJoint:
		a: RecArgs_CreateMotorJoint
		a.world = RecR_WORLDID(rdr)
		a.def = RecR_MOTORJOINTDEF(rdr)
		if rdr.ok {
			RecDispatch_CreateMotorJoint(&a, rdr)
		}
	case recOpCreatePrismaticJoint:
		a: RecArgs_CreatePrismaticJoint
		a.world = RecR_WORLDID(rdr)
		a.def = RecR_PRISMATICJOINTDEF(rdr)
		if rdr.ok {
			RecDispatch_CreatePrismaticJoint(&a, rdr)
		}
	case recOpCreateRevoluteJoint:
		a: RecArgs_CreateRevoluteJoint
		a.world = RecR_WORLDID(rdr)
		a.def = RecR_REVOLUTEJOINTDEF(rdr)
		if rdr.ok {
			RecDispatch_CreateRevoluteJoint(&a, rdr)
		}
	case recOpCreateSphericalJoint:
		a: RecArgs_CreateSphericalJoint
		a.world = RecR_WORLDID(rdr)
		a.def = RecR_SPHERICALJOINTDEF(rdr)
		if rdr.ok {
			RecDispatch_CreateSphericalJoint(&a, rdr)
		}
	case recOpCreateWeldJoint:
		a: RecArgs_CreateWeldJoint
		a.world = RecR_WORLDID(rdr)
		a.def = RecR_WELDJOINTDEF(rdr)
		if rdr.ok {
			RecDispatch_CreateWeldJoint(&a, rdr)
		}
	case recOpCreateWheelJoint:
		a: RecArgs_CreateWheelJoint
		a.world = RecR_WORLDID(rdr)
		a.def = RecR_WHEELJOINTDEF(rdr)
		if rdr.ok {
			RecDispatch_CreateWheelJoint(&a, rdr)
		}
	case recOpDestroyJoint:
		a: RecArgs_DestroyJoint
		a.joint = RecR_JOINTID(rdr)
		a.wakeAttached = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_DestroyJoint(&a, rdr)
		}
	case recOpJointSetLocalFrameA:
		a: RecArgs_JointSetLocalFrameA
		a.joint = RecR_JOINTID(rdr)
		a.localFrame = RecR_TRANSFORM(rdr)
		if rdr.ok {
			RecDispatch_JointSetLocalFrameA(&a, rdr)
		}
	case recOpJointSetLocalFrameB:
		a: RecArgs_JointSetLocalFrameB
		a.joint = RecR_JOINTID(rdr)
		a.localFrame = RecR_TRANSFORM(rdr)
		if rdr.ok {
			RecDispatch_JointSetLocalFrameB(&a, rdr)
		}
	case recOpJointSetCollideConnected:
		a: RecArgs_JointSetCollideConnected
		a.joint = RecR_JOINTID(rdr)
		a.shouldCollide = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_JointSetCollideConnected(&a, rdr)
		}
	case recOpJointWakeBodies:
		a: RecArgs_JointWakeBodies
		a.joint = RecR_JOINTID(rdr)
		if rdr.ok {
			RecDispatch_JointWakeBodies(&a, rdr)
		}
	case recOpJointSetConstraintTuning:
		a: RecArgs_JointSetConstraintTuning
		a.joint = RecR_JOINTID(rdr)
		a.hertz = RecR_F32(rdr)
		a.dampingRatio = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_JointSetConstraintTuning(&a, rdr)
		}
	case recOpJointSetForceThreshold:
		a: RecArgs_JointSetForceThreshold
		a.joint = RecR_JOINTID(rdr)
		a.threshold = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_JointSetForceThreshold(&a, rdr)
		}
	case recOpJointSetTorqueThreshold:
		a: RecArgs_JointSetTorqueThreshold
		a.joint = RecR_JOINTID(rdr)
		a.threshold = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_JointSetTorqueThreshold(&a, rdr)
		}
	case recOpParallelJointSetSpringHertz:
		a: RecArgs_ParallelJointSetSpringHertz
		a.joint = RecR_JOINTID(rdr)
		a.hertz = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_ParallelJointSetSpringHertz(&a, rdr)
		}
	case recOpParallelJointSetSpringDampingRatio:
		a: RecArgs_ParallelJointSetSpringDampingRatio
		a.joint = RecR_JOINTID(rdr)
		a.dampingRatio = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_ParallelJointSetSpringDampingRatio(&a, rdr)
		}
	case recOpParallelJointSetMaxTorque:
		a: RecArgs_ParallelJointSetMaxTorque
		a.joint = RecR_JOINTID(rdr)
		a.maxTorque = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_ParallelJointSetMaxTorque(&a, rdr)
		}
	case recOpDistanceJointSetLength:
		a: RecArgs_DistanceJointSetLength
		a.joint = RecR_JOINTID(rdr)
		a.length = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_DistanceJointSetLength(&a, rdr)
		}
	case recOpDistanceJointEnableSpring:
		a: RecArgs_DistanceJointEnableSpring
		a.joint = RecR_JOINTID(rdr)
		a.enableSpring = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_DistanceJointEnableSpring(&a, rdr)
		}
	case recOpDistanceJointSetSpringForceRange:
		a: RecArgs_DistanceJointSetSpringForceRange
		a.joint = RecR_JOINTID(rdr)
		a.lowerForce = RecR_F32(rdr)
		a.upperForce = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_DistanceJointSetSpringForceRange(&a, rdr)
		}
	case recOpDistanceJointSetSpringHertz:
		a: RecArgs_DistanceJointSetSpringHertz
		a.joint = RecR_JOINTID(rdr)
		a.hertz = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_DistanceJointSetSpringHertz(&a, rdr)
		}
	case recOpDistanceJointSetSpringDampingRatio:
		a: RecArgs_DistanceJointSetSpringDampingRatio
		a.joint = RecR_JOINTID(rdr)
		a.dampingRatio = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_DistanceJointSetSpringDampingRatio(&a, rdr)
		}
	case recOpDistanceJointEnableLimit:
		a: RecArgs_DistanceJointEnableLimit
		a.joint = RecR_JOINTID(rdr)
		a.enableLimit = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_DistanceJointEnableLimit(&a, rdr)
		}
	case recOpDistanceJointSetLengthRange:
		a: RecArgs_DistanceJointSetLengthRange
		a.joint = RecR_JOINTID(rdr)
		a.minLength = RecR_F32(rdr)
		a.maxLength = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_DistanceJointSetLengthRange(&a, rdr)
		}
	case recOpDistanceJointEnableMotor:
		a: RecArgs_DistanceJointEnableMotor
		a.joint = RecR_JOINTID(rdr)
		a.enableMotor = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_DistanceJointEnableMotor(&a, rdr)
		}
	case recOpDistanceJointSetMotorSpeed:
		a: RecArgs_DistanceJointSetMotorSpeed
		a.joint = RecR_JOINTID(rdr)
		a.motorSpeed = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_DistanceJointSetMotorSpeed(&a, rdr)
		}
	case recOpDistanceJointSetMaxMotorForce:
		a: RecArgs_DistanceJointSetMaxMotorForce
		a.joint = RecR_JOINTID(rdr)
		a.force = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_DistanceJointSetMaxMotorForce(&a, rdr)
		}
	case recOpMotorJointSetLinearVelocity:
		a: RecArgs_MotorJointSetLinearVelocity
		a.joint = RecR_JOINTID(rdr)
		a.velocity = RecR_VEC3(rdr)
		if rdr.ok {
			RecDispatch_MotorJointSetLinearVelocity(&a, rdr)
		}
	case recOpMotorJointSetAngularVelocity:
		a: RecArgs_MotorJointSetAngularVelocity
		a.joint = RecR_JOINTID(rdr)
		a.velocity = RecR_VEC3(rdr)
		if rdr.ok {
			RecDispatch_MotorJointSetAngularVelocity(&a, rdr)
		}
	case recOpMotorJointSetMaxVelocityForce:
		a: RecArgs_MotorJointSetMaxVelocityForce
		a.joint = RecR_JOINTID(rdr)
		a.maxForce = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_MotorJointSetMaxVelocityForce(&a, rdr)
		}
	case recOpMotorJointSetMaxVelocityTorque:
		a: RecArgs_MotorJointSetMaxVelocityTorque
		a.joint = RecR_JOINTID(rdr)
		a.maxTorque = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_MotorJointSetMaxVelocityTorque(&a, rdr)
		}
	case recOpMotorJointSetLinearHertz:
		a: RecArgs_MotorJointSetLinearHertz
		a.joint = RecR_JOINTID(rdr)
		a.hertz = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_MotorJointSetLinearHertz(&a, rdr)
		}
	case recOpMotorJointSetLinearDampingRatio:
		a: RecArgs_MotorJointSetLinearDampingRatio
		a.joint = RecR_JOINTID(rdr)
		a.damping = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_MotorJointSetLinearDampingRatio(&a, rdr)
		}
	case recOpMotorJointSetAngularHertz:
		a: RecArgs_MotorJointSetAngularHertz
		a.joint = RecR_JOINTID(rdr)
		a.hertz = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_MotorJointSetAngularHertz(&a, rdr)
		}
	case recOpMotorJointSetAngularDampingRatio:
		a: RecArgs_MotorJointSetAngularDampingRatio
		a.joint = RecR_JOINTID(rdr)
		a.damping = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_MotorJointSetAngularDampingRatio(&a, rdr)
		}
	case recOpMotorJointSetMaxSpringForce:
		a: RecArgs_MotorJointSetMaxSpringForce
		a.joint = RecR_JOINTID(rdr)
		a.maxForce = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_MotorJointSetMaxSpringForce(&a, rdr)
		}
	case recOpMotorJointSetMaxSpringTorque:
		a: RecArgs_MotorJointSetMaxSpringTorque
		a.joint = RecR_JOINTID(rdr)
		a.maxTorque = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_MotorJointSetMaxSpringTorque(&a, rdr)
		}
	case recOpPrismaticJointEnableSpring:
		a: RecArgs_PrismaticJointEnableSpring
		a.joint = RecR_JOINTID(rdr)
		a.enableSpring = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_PrismaticJointEnableSpring(&a, rdr)
		}
	case recOpPrismaticJointSetSpringHertz:
		a: RecArgs_PrismaticJointSetSpringHertz
		a.joint = RecR_JOINTID(rdr)
		a.hertz = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_PrismaticJointSetSpringHertz(&a, rdr)
		}
	case recOpPrismaticJointSetSpringDampingRatio:
		a: RecArgs_PrismaticJointSetSpringDampingRatio
		a.joint = RecR_JOINTID(rdr)
		a.dampingRatio = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_PrismaticJointSetSpringDampingRatio(&a, rdr)
		}
	case recOpPrismaticJointSetTargetTranslation:
		a: RecArgs_PrismaticJointSetTargetTranslation
		a.joint = RecR_JOINTID(rdr)
		a.translation = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_PrismaticJointSetTargetTranslation(&a, rdr)
		}
	case recOpPrismaticJointEnableLimit:
		a: RecArgs_PrismaticJointEnableLimit
		a.joint = RecR_JOINTID(rdr)
		a.enableLimit = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_PrismaticJointEnableLimit(&a, rdr)
		}
	case recOpPrismaticJointSetLimits:
		a: RecArgs_PrismaticJointSetLimits
		a.joint = RecR_JOINTID(rdr)
		a.lower = RecR_F32(rdr)
		a.upper = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_PrismaticJointSetLimits(&a, rdr)
		}
	case recOpPrismaticJointEnableMotor:
		a: RecArgs_PrismaticJointEnableMotor
		a.joint = RecR_JOINTID(rdr)
		a.enableMotor = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_PrismaticJointEnableMotor(&a, rdr)
		}
	case recOpPrismaticJointSetMotorSpeed:
		a: RecArgs_PrismaticJointSetMotorSpeed
		a.joint = RecR_JOINTID(rdr)
		a.motorSpeed = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_PrismaticJointSetMotorSpeed(&a, rdr)
		}
	case recOpPrismaticJointSetMaxMotorForce:
		a: RecArgs_PrismaticJointSetMaxMotorForce
		a.joint = RecR_JOINTID(rdr)
		a.force = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_PrismaticJointSetMaxMotorForce(&a, rdr)
		}
	case recOpRevoluteJointEnableSpring:
		a: RecArgs_RevoluteJointEnableSpring
		a.joint = RecR_JOINTID(rdr)
		a.enableSpring = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_RevoluteJointEnableSpring(&a, rdr)
		}
	case recOpRevoluteJointSetSpringHertz:
		a: RecArgs_RevoluteJointSetSpringHertz
		a.joint = RecR_JOINTID(rdr)
		a.hertz = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_RevoluteJointSetSpringHertz(&a, rdr)
		}
	case recOpRevoluteJointSetSpringDampingRatio:
		a: RecArgs_RevoluteJointSetSpringDampingRatio
		a.joint = RecR_JOINTID(rdr)
		a.dampingRatio = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_RevoluteJointSetSpringDampingRatio(&a, rdr)
		}
	case recOpRevoluteJointSetTargetAngle:
		a: RecArgs_RevoluteJointSetTargetAngle
		a.joint = RecR_JOINTID(rdr)
		a.angle = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_RevoluteJointSetTargetAngle(&a, rdr)
		}
	case recOpRevoluteJointEnableLimit:
		a: RecArgs_RevoluteJointEnableLimit
		a.joint = RecR_JOINTID(rdr)
		a.enableLimit = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_RevoluteJointEnableLimit(&a, rdr)
		}
	case recOpRevoluteJointSetLimits:
		a: RecArgs_RevoluteJointSetLimits
		a.joint = RecR_JOINTID(rdr)
		a.lower = RecR_F32(rdr)
		a.upper = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_RevoluteJointSetLimits(&a, rdr)
		}
	case recOpRevoluteJointEnableMotor:
		a: RecArgs_RevoluteJointEnableMotor
		a.joint = RecR_JOINTID(rdr)
		a.enableMotor = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_RevoluteJointEnableMotor(&a, rdr)
		}
	case recOpRevoluteJointSetMotorSpeed:
		a: RecArgs_RevoluteJointSetMotorSpeed
		a.joint = RecR_JOINTID(rdr)
		a.motorSpeed = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_RevoluteJointSetMotorSpeed(&a, rdr)
		}
	case recOpRevoluteJointSetMaxMotorTorque:
		a: RecArgs_RevoluteJointSetMaxMotorTorque
		a.joint = RecR_JOINTID(rdr)
		a.torque = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_RevoluteJointSetMaxMotorTorque(&a, rdr)
		}
	case recOpSphericalJointEnableConeLimit:
		a: RecArgs_SphericalJointEnableConeLimit
		a.joint = RecR_JOINTID(rdr)
		a.enableLimit = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_SphericalJointEnableConeLimit(&a, rdr)
		}
	case recOpSphericalJointSetConeLimit:
		a: RecArgs_SphericalJointSetConeLimit
		a.joint = RecR_JOINTID(rdr)
		a.angleRadians = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_SphericalJointSetConeLimit(&a, rdr)
		}
	case recOpSphericalJointEnableTwistLimit:
		a: RecArgs_SphericalJointEnableTwistLimit
		a.joint = RecR_JOINTID(rdr)
		a.enableLimit = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_SphericalJointEnableTwistLimit(&a, rdr)
		}
	case recOpSphericalJointSetTwistLimits:
		a: RecArgs_SphericalJointSetTwistLimits
		a.joint = RecR_JOINTID(rdr)
		a.lower = RecR_F32(rdr)
		a.upper = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_SphericalJointSetTwistLimits(&a, rdr)
		}
	case recOpSphericalJointEnableSpring:
		a: RecArgs_SphericalJointEnableSpring
		a.joint = RecR_JOINTID(rdr)
		a.enableSpring = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_SphericalJointEnableSpring(&a, rdr)
		}
	case recOpSphericalJointSetSpringHertz:
		a: RecArgs_SphericalJointSetSpringHertz
		a.joint = RecR_JOINTID(rdr)
		a.hertz = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_SphericalJointSetSpringHertz(&a, rdr)
		}
	case recOpSphericalJointSetSpringDampingRatio:
		a: RecArgs_SphericalJointSetSpringDampingRatio
		a.joint = RecR_JOINTID(rdr)
		a.dampingRatio = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_SphericalJointSetSpringDampingRatio(&a, rdr)
		}
	case recOpSphericalJointSetTargetRotation:
		a: RecArgs_SphericalJointSetTargetRotation
		a.joint = RecR_JOINTID(rdr)
		a.targetRotation = RecR_QUAT(rdr)
		if rdr.ok {
			RecDispatch_SphericalJointSetTargetRotation(&a, rdr)
		}
	case recOpSphericalJointEnableMotor:
		a: RecArgs_SphericalJointEnableMotor
		a.joint = RecR_JOINTID(rdr)
		a.enableMotor = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_SphericalJointEnableMotor(&a, rdr)
		}
	case recOpSphericalJointSetMotorVelocity:
		a: RecArgs_SphericalJointSetMotorVelocity
		a.joint = RecR_JOINTID(rdr)
		a.motorVelocity = RecR_VEC3(rdr)
		if rdr.ok {
			RecDispatch_SphericalJointSetMotorVelocity(&a, rdr)
		}
	case recOpSphericalJointSetMaxMotorTorque:
		a: RecArgs_SphericalJointSetMaxMotorTorque
		a.joint = RecR_JOINTID(rdr)
		a.torque = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_SphericalJointSetMaxMotorTorque(&a, rdr)
		}
	case recOpWeldJointSetLinearHertz:
		a: RecArgs_WeldJointSetLinearHertz
		a.joint = RecR_JOINTID(rdr)
		a.hertz = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WeldJointSetLinearHertz(&a, rdr)
		}
	case recOpWeldJointSetLinearDampingRatio:
		a: RecArgs_WeldJointSetLinearDampingRatio
		a.joint = RecR_JOINTID(rdr)
		a.dampingRatio = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WeldJointSetLinearDampingRatio(&a, rdr)
		}
	case recOpWeldJointSetAngularHertz:
		a: RecArgs_WeldJointSetAngularHertz
		a.joint = RecR_JOINTID(rdr)
		a.hertz = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WeldJointSetAngularHertz(&a, rdr)
		}
	case recOpWeldJointSetAngularDampingRatio:
		a: RecArgs_WeldJointSetAngularDampingRatio
		a.joint = RecR_JOINTID(rdr)
		a.dampingRatio = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WeldJointSetAngularDampingRatio(&a, rdr)
		}
	case recOpWheelJointEnableSuspension:
		a: RecArgs_WheelJointEnableSuspension
		a.joint = RecR_JOINTID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_WheelJointEnableSuspension(&a, rdr)
		}
	case recOpWheelJointSetSuspensionHertz:
		a: RecArgs_WheelJointSetSuspensionHertz
		a.joint = RecR_JOINTID(rdr)
		a.hertz = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WheelJointSetSuspensionHertz(&a, rdr)
		}
	case recOpWheelJointSetSuspensionDampingRatio:
		a: RecArgs_WheelJointSetSuspensionDampingRatio
		a.joint = RecR_JOINTID(rdr)
		a.dampingRatio = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WheelJointSetSuspensionDampingRatio(&a, rdr)
		}
	case recOpWheelJointEnableSuspensionLimit:
		a: RecArgs_WheelJointEnableSuspensionLimit
		a.joint = RecR_JOINTID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_WheelJointEnableSuspensionLimit(&a, rdr)
		}
	case recOpWheelJointSetSuspensionLimits:
		a: RecArgs_WheelJointSetSuspensionLimits
		a.joint = RecR_JOINTID(rdr)
		a.lower = RecR_F32(rdr)
		a.upper = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WheelJointSetSuspensionLimits(&a, rdr)
		}
	case recOpWheelJointEnableSpinMotor:
		a: RecArgs_WheelJointEnableSpinMotor
		a.joint = RecR_JOINTID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_WheelJointEnableSpinMotor(&a, rdr)
		}
	case recOpWheelJointSetSpinMotorSpeed:
		a: RecArgs_WheelJointSetSpinMotorSpeed
		a.joint = RecR_JOINTID(rdr)
		a.speed = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WheelJointSetSpinMotorSpeed(&a, rdr)
		}
	case recOpWheelJointSetMaxSpinTorque:
		a: RecArgs_WheelJointSetMaxSpinTorque
		a.joint = RecR_JOINTID(rdr)
		a.torque = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WheelJointSetMaxSpinTorque(&a, rdr)
		}
	case recOpWheelJointEnableSteering:
		a: RecArgs_WheelJointEnableSteering
		a.joint = RecR_JOINTID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_WheelJointEnableSteering(&a, rdr)
		}
	case recOpWheelJointSetSteeringHertz:
		a: RecArgs_WheelJointSetSteeringHertz
		a.joint = RecR_JOINTID(rdr)
		a.hertz = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WheelJointSetSteeringHertz(&a, rdr)
		}
	case recOpWheelJointSetSteeringDampingRatio:
		a: RecArgs_WheelJointSetSteeringDampingRatio
		a.joint = RecR_JOINTID(rdr)
		a.dampingRatio = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WheelJointSetSteeringDampingRatio(&a, rdr)
		}
	case recOpWheelJointSetMaxSteeringTorque:
		a: RecArgs_WheelJointSetMaxSteeringTorque
		a.joint = RecR_JOINTID(rdr)
		a.torque = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WheelJointSetMaxSteeringTorque(&a, rdr)
		}
	case recOpWheelJointEnableSteeringLimit:
		a: RecArgs_WheelJointEnableSteeringLimit
		a.joint = RecR_JOINTID(rdr)
		a.flag = RecR_BOOL(rdr)
		if rdr.ok {
			RecDispatch_WheelJointEnableSteeringLimit(&a, rdr)
		}
	case recOpWheelJointSetSteeringLimits:
		a: RecArgs_WheelJointSetSteeringLimits
		a.joint = RecR_JOINTID(rdr)
		a.lower = RecR_F32(rdr)
		a.upper = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WheelJointSetSteeringLimits(&a, rdr)
		}
	case recOpWheelJointSetTargetSteeringAngle:
		a: RecArgs_WheelJointSetTargetSteeringAngle
		a.joint = RecR_JOINTID(rdr)
		a.radians = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_WheelJointSetTargetSteeringAngle(&a, rdr)
		}
	case recOpCreateMoverJoint:
		a: RecArgs_CreateMoverJoint
		a.world = RecR_WORLDID(rdr)
		a.def = RecR_MOVERJOINTDEF(rdr)
		if rdr.ok {
			RecDispatch_CreateMoverJoint(&a, rdr)
		}
	case recOpMoverJointSetLinearVelocity:
		a: RecArgs_MoverJointSetLinearVelocity
		a.joint = RecR_JOINTID(rdr)
		a.velocity = RecR_VEC3(rdr)
		if rdr.ok {
			RecDispatch_MoverJointSetLinearVelocity(&a, rdr)
		}
	case recOpMoverJointSetMaxVelocityForce:
		a: RecArgs_MoverJointSetMaxVelocityForce
		a.joint = RecR_JOINTID(rdr)
		a.maxForce = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_MoverJointSetMaxVelocityForce(&a, rdr)
		}
	case recOpCreatePogoJoint:
		a: RecArgs_CreatePogoJoint
		a.world = RecR_WORLDID(rdr)
		a.def = RecR_POGOJOINTDEF(rdr)
		if rdr.ok {
			RecDispatch_CreatePogoJoint(&a, rdr)
		}
	case recOpPogoJointSetRestLength:
		a: RecArgs_PogoJointSetRestLength
		a.joint = RecR_JOINTID(rdr)
		a.length = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_PogoJointSetRestLength(&a, rdr)
		}
	case recOpPogoJointSetSpringHertz:
		a: RecArgs_PogoJointSetSpringHertz
		a.joint = RecR_JOINTID(rdr)
		a.hertz = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_PogoJointSetSpringHertz(&a, rdr)
		}
	case recOpPogoJointSetSpringDampingRatio:
		a: RecArgs_PogoJointSetSpringDampingRatio
		a.joint = RecR_JOINTID(rdr)
		a.dampingRatio = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_PogoJointSetSpringDampingRatio(&a, rdr)
		}
	case recOpPogoJointSetMaxTensionForce:
		a: RecArgs_PogoJointSetMaxTensionForce
		a.joint = RecR_JOINTID(rdr)
		a.force = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_PogoJointSetMaxTensionForce(&a, rdr)
		}
	case recOpPogoJointSetMaxCompressionForce:
		a: RecArgs_PogoJointSetMaxCompressionForce
		a.joint = RecR_JOINTID(rdr)
		a.force = RecR_F32(rdr)
		if rdr.ok {
			RecDispatch_PogoJointSetMaxCompressionForce(&a, rdr)
		}
	case recOpPogoJointSetNormal:
		a: RecArgs_PogoJointSetNormal
		a.joint = RecR_JOINTID(rdr)
		a.normal = RecR_VEC3(rdr)
		if rdr.ok {
			RecDispatch_PogoJointSetNormal(&a, rdr)
		}
	case recOpQueryOverlapAABB:
		a: RecArgs_QueryOverlapAABB
		a.world = RecR_WORLDID(rdr)
		a.aabb = RecR_AABB(rdr)
		a.filter = RecR_QUERYFILTER(rdr)
		if rdr.ok {
			RecDispatch_QueryOverlapAABB(&a, rdr)
		}
	case recOpQueryOverlapShape:
		a: RecArgs_QueryOverlapShape
		a.world = RecR_WORLDID(rdr)
		a.origin = RecR_POSITION(rdr)
		a.proxy = RecR_SHAPEPROXY(rdr)
		a.filter = RecR_QUERYFILTER(rdr)
		if rdr.ok {
			RecDispatch_QueryOverlapShape(&a, rdr)
		}
	case recOpQueryCastRay:
		a: RecArgs_QueryCastRay
		a.world = RecR_WORLDID(rdr)
		a.origin = RecR_POSITION(rdr)
		a.translation = RecR_VEC3(rdr)
		a.filter = RecR_QUERYFILTER(rdr)
		if rdr.ok {
			RecDispatch_QueryCastRay(&a, rdr)
		}
	case recOpQueryCastShape:
		a: RecArgs_QueryCastShape
		a.world = RecR_WORLDID(rdr)
		a.origin = RecR_POSITION(rdr)
		a.proxy = RecR_SHAPEPROXY(rdr)
		a.translation = RecR_VEC3(rdr)
		a.filter = RecR_QUERYFILTER(rdr)
		if rdr.ok {
			RecDispatch_QueryCastShape(&a, rdr)
		}
	case recOpQueryCastRayClosest:
		a: RecArgs_QueryCastRayClosest
		a.world = RecR_WORLDID(rdr)
		a.origin = RecR_POSITION(rdr)
		a.translation = RecR_VEC3(rdr)
		a.filter = RecR_QUERYFILTER(rdr)
		if rdr.ok {
			RecDispatch_QueryCastRayClosest(&a, rdr)
		}
	case recOpQueryCastMover:
		a: RecArgs_QueryCastMover
		a.world = RecR_WORLDID(rdr)
		a.origin = RecR_POSITION(rdr)
		a.mover = RecR_CAPSULE(rdr)
		a.translation = RecR_VEC3(rdr)
		a.filter = RecR_QUERYFILTER(rdr)
		if rdr.ok {
			RecDispatch_QueryCastMover(&a, rdr)
		}
	case recOpQueryCollideMover:
		a: RecArgs_QueryCollideMover
		a.world = RecR_WORLDID(rdr)
		a.origin = RecR_POSITION(rdr)
		a.mover = RecR_CAPSULE(rdr)
		a.filter = RecR_QUERYFILTER(rdr)
		if rdr.ok {
			RecDispatch_QueryCollideMover(&a, rdr)
		}
	case recOpQueryTag:
		a: RecArgs_QueryTag
		a.key = RecR_U64(rdr)
		if rdr.ok {
			RecDispatch_QueryTag(&a, rdr)
		}
	case recOpStateHash:
		a: RecArgs_StateHash
		a.world = RecR_WORLDID(rdr)
		a.hash = RecR_U64(rdr)
		if rdr.ok {
			RecDispatch_StateHash(&a, rdr)
		}
	case recOpRecordingBounds:
		a: RecArgs_RecordingBounds
		a.bounds = RecR_AABB(rdr)
		if rdr.ok {
			RecDispatch_RecordingBounds(&a, rdr)
		}
	case:
		fmt.printf("b3ReplayFile: unknown opcode 0x%02X, skipping %d bytes\n", opcode, payloadSize)
		// payloadStart is in bounds, so size - payloadStart is the bytes left to skip over
		if payloadSize > u32(rdr.size - payloadStart) {
			rdr.ok = false
		} else {
			rdr.cursor = payloadStart + i32(payloadSize)
		}
	}
	// unified end
	return i32(opcode)
}

// Public entry point

ValidateReplay :: proc(data: rawptr, size: i32, workerCount: i32) -> bool {
	player := CreatePlayer(data, size, workerCount)
	if player == nil {
		return false
	}

	for RecPlayer_StepFrame(player) {
		if player.rdr.diverged {
			break
		}
	}

	ok := player.rdr.ok && player.rdr.diverged == false
	DestroyPlayer(player)
	return ok
}

// b3RecPlayer implementation

REC_KEYFRAME_INTERVAL_DEFAULT :: 16
REC_KEYFRAME_BUDGET_DEFAULT :: int(512 * 1024 * 1024)

// Overflow-safe growth for the player's accumulating arrays. Counts come from the replay itself,
// not the file, so this only guards the byte-size multiply. Preserves keep elements.
@(private = "file")
RecGrow :: proc(data: ^rawptr, capacity: ^i32, need: i32, keep: i32, elemSize: i32) {
	if need <= capacity^ {
		return
	}
	newCap := capacity^ == 0 ? 8 : 2 * capacity^
	if newCap < need {
		newCap = need
	}
	grown := AllocHost(int(newCap) * int(elemSize))
	if data^ != nil {
		if keep > 0 {
			mem.copy(grown, data^, int(keep) * int(elemSize))
		}
		Free(data^, int(capacity^) * int(elemSize))
	}
	data^ = grown
	capacity^ = newCap
}

// Block B: per-frame query store helpers. Forward declared above the query dispatchers, which run
// before the player struct is defined, so the player-dereferencing code lives here.

@(private = "file")
RecGrowFrameQueries :: proc(player: ^RecPlayer) {
	RecGrow(
		(^rawptr)(&player.frameQueries),
		&player.frameQueryCap,
		player.frameQueryCount + 1,
		player.frameQueryCount,
		i32(size_of(RecDrawQuery)),
	)
}

@(private = "file")
RecGrowFrameHits :: proc(player: ^RecPlayer, need: i32) {
	RecGrow(
		(^rawptr)(&player.frameHits),
		&player.frameHitCap,
		player.frameHitCount + need,
		player.frameHitCount,
		i32(size_of(RecRecordedHit)),
	)
}

// Push a draw record for one query and copy its hits into the per-frame store. Ids in hits[] are
// already remapped to the replay world by the dispatcher.
@(private = "file")
RecStashQueryBegin :: proc(player: ^RecPlayer, kind: i32, hits: [^]RecRecordedHit, hitCount: i32) -> ^RecDrawQuery {
	RecGrowFrameQueries(player)
	q := &player.frameQueries[player.frameQueryCount]
	mem.zero(q, size_of(q^))
	q.kind = kind
	// Pair the query with the key from its preceding QueryTag op, if any, then clear it so the next
	// untagged query reads 0.
	q.key = player.rdr.pendingQueryKey
	player.rdr.pendingQueryKey = 0
	q.hitStart = player.frameHitCount
	q.hitCount = hitCount
	RecGrowFrameHits(player, hitCount)
	for i: i32 = 0; i < hitCount; i += 1 {
		player.frameHits[player.frameHitCount + i] = hits[i]
	}
	player.frameHitCount += hitCount
	player.frameQueryCount += 1
	return q
}

// Append a created body to the outliner list. Ordinals are creation order and never reused.
@(private = "file")
RecTrackBodyCreate :: proc(player: ^RecPlayer, id: BodyId) {
	RecGrow(
		(^rawptr)(&player.bodyIds),
		&player.bodyIdCap,
		player.bodyIdCount + 1,
		player.bodyIdCount,
		i32(size_of(BodyId)),
	)
	player.bodyIds[player.bodyIdCount] = id
	player.bodyIdCount += 1
}

// Leave a hole so later ordinals do not shift, keeping a stored selection stable.
@(private = "file")
RecTrackBodyDestroy :: proc(player: ^RecPlayer, id: BodyId) {
	for i: i32 = 0; i < player.bodyIdCount; i += 1 {
		if ID_EQUALS(player.bodyIds[i], id) {
			player.bodyIds[i] = nullBodyId
			return
		}
	}
}

// Snapshot bodies are restored as a struct image and never hit the CreateBody hook the tracker keys
// on, so the seed world must be walked once to populate the outliner list. Slot order is stable.
@(private = "file")
RecSeedBodyIds :: proc(player: ^RecPlayer) {
	world := GetWorldFromId(player.rdr.replayWorldId)
	player.bodyIdCount = 0
	count := world.bodies.count
	for i: i32 = 0; i < count; i += 1 {
		if world.bodies.data[i].id != i {
			continue // free slot
		}
		RecTrackBodyCreate(player, MakeBodyId(world, i))
	}
}

// Seed the outliner list from the current world and save it as the frame-0 restore copy.
@(private = "file")
RecSeedFrame0BodyIds :: proc(player: ^RecPlayer) {
	RecSeedBodyIds(player)
	if player.frame0BodyIds != nil {
		Free(player.frame0BodyIds, int(player.frame0BodyIdCount) * size_of(BodyId))
		player.frame0BodyIds = nil
	}
	player.frame0BodyIdCount = player.bodyIdCount
	if player.bodyIdCount > 0 {
		player.frame0BodyIds = cast([^]BodyId)AllocHost(int(player.bodyIdCount) * size_of(BodyId))
		mem.copy(player.frame0BodyIds, player.bodyIds, int(player.bodyIdCount) * size_of(BodyId))
	}
}

// Tag key to tag index, so the viewer resolves a query's caller id and label in O(1) instead of a
// linear scan over the tag table.
// unified: the verstable map (u64 -> u32) is an Odin map on the heap allocator; it is only used for lookups.
// Its storage is not counted by GetByteCount.
@(private = "file")
RecTagLookup :: map[u64]u32
// unified end

// Read the optional query-tag table trailing the geometry entries: u32 tagCount then per tag
// { u64 key, u64 id, u16 len, name bytes }. A recording written before the tag table leaves rp at
// dataEnd, so nothing loads. Bounds-checked; tagCount reflects only the tags that fully fit, so a
// truncated tail loads what it can and reports the real count.
@(private = "file")
RecLoadTags :: proc(rdr: ^RecReader, rp: [^]u8, dataEnd: [^]u8) {
	sub := RecReader{}
	sub.data = rp
	sub.size = i32(uintptr(dataEnd) - uintptr(rp))
	sub.ok = true

	count := RecR_U32(&sub)
	if sub.ok == false || count == 0 {
		return
	}

	// Each tag is at least 18 bytes (8 key + 8 id + 2 length). Reject a count that cannot fit the
	// remaining bytes so a corrupt table cannot request a wild allocation.
	if int(count) > int(sub.size - sub.cursor) / 18 {
		return
	}

	tags := cast([^]RecTag)AllocHost(int(count) * size_of(RecTag))
	mem.zero(tags, int(count) * size_of(RecTag))

	m := cast(^RecTagLookup)AllocHost(size_of(RecTagLookup))
	// unified: b3RecTagLookup_init
	m^ = make(RecTagLookup, runtime.heap_allocator())
	// unified end

	loaded: u32 = 0
	for i: u32 = 0; i < count; i += 1 {
		key := RecR_U64(&sub)
		id := RecR_U64(&sub)
		len := RecR_U16(&sub)
		if sub.ok == false {
			break
		}
		if len == 0xFFFF {
			len = 0 // a null name is written as 0xFFFF
		}
		if i64(sub.cursor) + i64(len) > i64(sub.size) {
			break
		}
		n := len > MAX_QUERY_NAME_LENGTH ? MAX_QUERY_NAME_LENGTH : i32(len)
		tags[loaded].key = key
		tags[loaded].id = id
		if n > 0 {
			mem.copy(&tags[loaded].queryName[0], &sub.data[sub.cursor], int(n))
		}
		tags[loaded].queryName[n] = 0
		sub.cursor += i32(len)
		m[key] = loaded
		loaded += 1
	}

	rdr.tags = tags
	rdr.tagCount = i32(loaded)
	rdr.tagCapacity = i32(count)
	rdr.tagMap = m
}

// Free slots loaded by b3RecLoadSlots.
@(private = "file")
RecFreeSlots :: proc(slots: [^]RegistrySlot, slotCount: i32) {
	if slots == nil {
		return
	}
	for i: i32 = 0; i < slotCount; i += 1 {
		slot := &slots[i]
		if slot.bytes != nil {
			Free(slot.bytes, int(slot.byteCount))
		}
	}
	Free(slots, int(slotCount) * size_of(RegistrySlot))
}

// Load the trailing registry block and fill rdr->slots/slotCount, then the optional tag table.
// Returns true on success. On failure sets rdr->ok = false and returns false.
@(private = "file")
RecLoadSlots :: proc(rdr: ^RecReader, data: rawptr, size: i32, registryOffset: u64, registryByteCount: u64) -> bool {
	if registryOffset == 0 || registryByteCount == 0 {
		rdr.slots = nil
		rdr.slotCount = 0
		return true
	}

	regStart := i32(registryOffset)
	regEnd := regStart + i32(registryByteCount)
	if regEnd > size {
		fmt.printf("b3ReplayFile: registry block out of bounds\n")
		return false
	}
	if regStart + 4 > size {
		fmt.printf("b3ReplayFile: registry too small\n")
		return false
	}

	dataEnd := ([^]u8)(data)[regEnd:]
	rp := ([^]u8)(data)[regStart:]
	count := u32(rp[0]) | (u32(rp[1]) << 8) | (u32(rp[2]) << 16) | (u32(rp[3]) << 24)
	rp = rp[4:]

	if count == 0 {
		rdr.slots = nil
		rdr.slotCount = 0
		RecLoadTags(rdr, rp, dataEnd)
		return true
	}

	// Each entry is at least 5 bytes (kind + 4-byte length). A count that cannot fit the remaining
	// registry bytes is a corrupt header, so reject it before allocating.
	if uintptr(rp) > uintptr(dataEnd) || int(count) > int(uintptr(dataEnd) - uintptr(rp)) / 5 {
		fmt.printf("b3ReplayFile: registry count out of range\n")
		return false
	}

	slots := cast([^]RegistrySlot)AllocHost(int(count) * size_of(RegistrySlot))
	mem.zero(slots, int(count) * size_of(RegistrySlot))

	for i: u32 = 0; i < count; i += 1 {
		if uintptr(rp) + 5 > uintptr(dataEnd) {
			fmt.printf("b3ReplayFile: registry truncated at entry %d\n", i)
			RecFreeSlots(slots, i32(count))
			return false
		}
		kind := rp[0]
		byteCount := u32(rp[1]) | (u32(rp[2]) << 8) | (u32(rp[3]) << 16) | (u32(rp[4]) << 24)
		rp = rp[5:]
		error: string = ""
		if byteCount == 0 || byteCount > u32(max(i32)) {
			error = "has an invalid byte count"
		} else if int(byteCount) > int(uintptr(dataEnd) - uintptr(rp)) {
			error = "bytes out of bounds"
		} else {
			bytes := cast([^]u8)AllocHost(int(byteCount))
			mem.copy(bytes, rp, int(byteCount))
			slots[i].kind = GeometryKind(kind)
			slots[i].byteCount = i32(byteCount)
			slots[i].bytes = bytes

			// Validate the compound.
			// todo validate other types?
			if kind == u8(GeometryKind.geometryCompound) && ValidateCompound(bytes, i32(byteCount)) == nil {
				error = "is not a valid compound"
			}
		}
		if error != "" {
			fmt.printf("b3ReplayFile: registry entry %d %s\n", i, error)
			RecFreeSlots(slots, i32(count))
			return false
		}
		rp = rp[byteCount:]
	}

	rdr.slots = slots
	rdr.slotCount = i32(count)
	RecLoadTags(rdr, rp, dataEnd)
	return true
}

// Walk the op stream once without dispatching: count Step ops and grab the first step's tuning.
@(private = "file")
RecScanFile :: proc(player: ^RecPlayer) {
	data := player.data
	size := player.registryEnd
	cursor := player.headerEnd
	frameCount: i32 = 0
	gotStep := false

	for cursor + 4 <= size {
		opcode := data[cursor]
		payloadSize := u32(data[cursor + 1]) | (u32(data[cursor + 2]) << 8) | (u32(data[cursor + 3]) << 16)
		payloadStart := cursor + 4
		if payloadStart + i32(payloadSize) > size {
			break
		}
		if opcode == recOpStep {
			frameCount += 1
			if !gotStep && payloadSize >= 12 {
				dtBits :=
					u32(data[payloadStart + 4]) |
					(u32(data[payloadStart + 5]) << 8) |
					(u32(data[payloadStart + 6]) << 16) |
					(u32(data[payloadStart + 7]) << 24)
				player.recordedDt = transmute(f32)dtBits
				player.recordedSubStepCount = i32(
					u32(data[payloadStart + 8]) |
					(u32(data[payloadStart + 9]) << 8) |
					(u32(data[payloadStart + 10]) << 16) |
					(u32(data[payloadStart + 11]) << 24),
				)
				gotStep = true
			}
		} else if opcode == 0xF2 && payloadSize >= u32(size_of(AABB)) { 	// RecordingBounds
			// Payload is a single b3AABB (lower xyz, upper xyz as f32), written at stop so the
			// viewer can frame the whole recorded motion without playing to the end.
			mem.copy(&player.bounds, &data[payloadStart], size_of(AABB))
		}
		cursor = payloadStart + i32(payloadSize)
	}
	player.frameCount = frameCount
}

// Free one keyframe's heap.
@(private = "file")
FreeKeyframe :: proc(kf: ^RecKeyframe) {
	if kf.image != nil {
		Free(kf.image, int(kf.imageCapacity))
	}
	if kf.bodyIds != nil {
		Free(kf.bodyIds, int(kf.bodyIdCount) * size_of(BodyId))
	}
}

// Pre-populate keyframeRec's registry to mirror rdr.slots so geometry ids stay stable during
// b3SerializeWorld. Each slot becomes one entry with id == slot index, even byte-identical slots that
// a hash collision left undeduplicated in an already-recorded file. b3SerializeWorld then resolves a
// live blob back to a valid slot index via the registry's exact dedup, so capture never grows it.
@(private = "file")
RecSeedKeyframeRegistry :: proc(player: ^RecPlayer) {
	reg := &player.keyframeRec.registry
	for i: i32 = 0; i < player.rdr.slotCount; i += 1 {
		slot := &player.rdr.slots[i]
		h := Hash64NonZero(slot.bytes, slot.byteCount)
		id := AppendGeometry(reg, slot.kind, h, slot.bytes, slot.byteCount)
		// Seeding in order without dedup keeps id == slot index.
		assert(id == u32(i))
		_ = id
	}
}

// Capture a restore-point keyframe for the just-completed frame. rdr.cursor already points to
// the next frame's Step op.
@(private = "file")
RecCaptureKeyframe :: proc(player: ^RecPlayer) {
	world := GetWorldFromId(player.rdr.replayWorldId)
	buf := RecBuffer{}

	regCountBefore := player.keyframeRec.registry.entries.count
	_ = regCountBefore

	SerializeWorld(world, &buf, player.keyframeRec)
	// Registry must not grow: all geometry was pre-seeded and the registry dedups exactly.
	assert(player.keyframeRec.registry.entries.count == regCountBefore)

	bodyBytes := int(player.bodyIdCount) * size_of(BodyId)
	newBytes := int(buf.capacity) + bodyBytes

	// Make room under the budget by doubling the spacing and evicting off-grid keyframes.
	for player.keyframeCount > 0 && player.keyframeBytes + newBytes > player.keyframeBudget {
		player.keyframeInterval *= 2
		kept: i32 = 0
		keptBytes := 0
		for i: i32 = 0; i < player.keyframeCount; i += 1 {
			kf := &player.keyframes[i]
			if kf.frame % player.keyframeInterval == 0 {
				player.keyframes[kept] = kf^
				keptBytes += int(kf.imageCapacity) + int(kf.bodyIdCount) * size_of(BodyId)
				kept += 1
			} else {
				FreeKeyframe(kf)
			}
		}
		progress := (kept < player.keyframeCount)
		player.keyframeCount = kept
		player.keyframeBytes = keptBytes
		if !progress {
			break
		}
	}

	// Grow the keyframe ring if needed.
	if player.keyframeCount >= player.keyframeCapacity {
		newCap := player.keyframeCapacity < 8 ? 8 : player.keyframeCapacity * 2
		player.keyframes = cast([^]RecKeyframe)GrowAllocHost(
			player.keyframes,
			player.keyframeCapacity * i32(size_of(RecKeyframe)),
			newCap * i32(size_of(RecKeyframe)),
		)
		player.keyframeCapacity = newCap
	}

	kf := &player.keyframes[player.keyframeCount]
	kf.image = buf.data
	kf.imageSize = buf.size
	kf.imageCapacity = buf.capacity
	kf.frame = player.frame
	kf.cursor = player.rdr.cursor
	kf.divergeFrame = player.divergeFrame
	kf.diverged = player.rdr.diverged
	kf.bodyIdCount = player.bodyIdCount
	kf.bodyIds = nil
	if bodyBytes > 0 {
		kf.bodyIds = cast([^]BodyId)AllocHost(bodyBytes)
		mem.copy(kf.bodyIds, player.bodyIds, bodyBytes)
	}

	player.keyframeBytes += newBytes
	player.keyframeCount += 1
	player.lastKeyframeFrame = player.frame
}

// Restore the world in-place from a keyframe image.
@(private = "file")
RecPlayerRestoreKeyframe :: proc(player: ^RecPlayer, kf: ^RecKeyframe) {
	world := GetWorldFromId(player.rdr.replayWorldId)
	if DeserializeIntoShell(kf.image, kf.imageSize, world, &player.rdr) == false {
		player.rdr.ok = false
		return
	}
	player.rdr.cursor = kf.cursor
	player.rdr.ok = true
	player.rdr.diverged = kf.diverged
	player.frame = kf.frame
	player.divergeFrame = kf.divergeFrame
	player.atEnd = false
	player.atPreStep = false

	// Restore the outliner list verbatim so ordinals match this frame.
	RecGrow((^rawptr)(&player.bodyIds), &player.bodyIdCap, kf.bodyIdCount, 0, i32(size_of(BodyId)))
	player.bodyIdCount = kf.bodyIdCount
	if kf.bodyIdCount > 0 {
		mem.copy(player.bodyIds, kf.bodyIds, int(kf.bodyIdCount) * size_of(BodyId))
	}
}

// Create a replay world carrying the host debug-shape callbacks. Every world the player
// stands up funnels through here so the sample renderer can draw replayed shapes.
@(private = "file")
RecPlayerCreateWorld :: proc(player: ^RecPlayer) -> WorldId {
	worldDef := DefaultWorldDef()
	worldDef.createDebugShape = player.createDebugShape
	worldDef.destroyDebugShape = player.destroyDebugShape
	worldDef.userDebugShapeContext = player.debugShapeContext
	// Carry the requested worker count so a rebuild on Restart or backward seek keeps the same
	// graph partitioning. Replaying at a different count than recorded is a determinism check.
	worldDef.workerCount = u32(MaxInt(1, player.recordedWorkerCount))
	return CreateWorld(&worldDef)
}

CreatePlayer :: proc(data: rawptr, size: i32, workerCount: i32) -> ^RecPlayer {
	if data == nil || size < i32(size_of(RecHeader)) {
		fmt.printf("b3RecPlayer_Create: recording too small\n")
		return nil
	}

	hdr: RecHeader
	mem.copy(&hdr, data, size_of(hdr))

	if hdr.magic != REC_MAGIC {
		fmt.printf("b3RecPlayer_Create: bad magic 0x%08X\n", hdr.magic)
		return nil
	}
	// Only the major version is breaking. Minor bumps are additive op-stream changes that keep the
	// header shape, and the dispatcher skips opcodes it doesn't know, so a minor mismatch still loads.
	if hdr.versionMajor != REC_VERSION_MAJOR {
		fmt.printf(
			"b3RecPlayer_Create: version mismatch %d.%d vs %d.%d\n",
			hdr.versionMajor,
			hdr.versionMinor,
			REC_VERSION_MAJOR,
			REC_VERSION_MINOR,
		)
		return nil
	}
	if hdr.pointerWidth != u8(size_of(rawptr)) {
		fmt.printf("b3RecPlayer_Create: pointer width mismatch %d vs %d\n", hdr.pointerWidth, u32(size_of(rawptr)))
		return nil
	}
	if hdr.bigEndian != 0 {
		fmt.printf("b3RecPlayer_Create: big-endian recording not supported\n")
		return nil
	}

	// Every recording is snapshot-seeded: the seed blob sits between the header and the op stream.
	if hdr.snapshotSize == 0 {
		fmt.printf("b3RecPlayer_Create: missing snapshot seed\n")
		return nil
	}

	// snapshotSize and registryOffset are 64-bit and come from the file. Validate in 64-bit so a
	// hostile value can't wrap when narrowed to int, then narrow once the bounds are known good.
	headerEnd64 := u64(size_of(RecHeader)) + hdr.snapshotSize
	registryEnd64 := (hdr.registryOffset != 0) ? hdr.registryOffset : u64(size)

	if headerEnd64 < size_of(RecHeader) || headerEnd64 > registryEnd64 || registryEnd64 > u64(size) {
		fmt.printf("b3RecPlayer_Create: corrupt offsets\n")
		return nil
	}

	headerEnd := i32(headerEnd64)
	registryEnd := i32(registryEnd64)

	// Own a private copy so the caller can free their buffer right away.
	copy := cast([^]u8)AllocHost(int(size))
	mem.copy(copy, data, int(size))

	player := cast(^RecPlayer)AllocHost(size_of(RecPlayer))
	mem.zero(player, size_of(RecPlayer))

	player.data = copy
	player.size = size
	player.headerEnd = headerEnd
	player.registryEnd = registryEnd
	player.lengthScale = hdr.lengthScale
	player.previousLengthScale = GetLengthUnitsPerMeter()
	player.frame = 0
	player.frameCount = 0
	player.recordedDt = 0.0
	player.recordedSubStepCount = 0
	player.recordedWorkerCount = workerCount
	player.atEnd = false
	player.atPreStep = false
	player.divergeFrame = -1
	player.keyframeMinInterval = REC_KEYFRAME_INTERVAL_DEFAULT
	player.keyframeInterval = REC_KEYFRAME_INTERVAL_DEFAULT
	player.keyframeBudget = REC_KEYFRAME_BUDGET_DEFAULT
	player.lastKeyframeFrame = 0

	// Set length scale so replay reproduces the same tuning constants.
	if hdr.lengthScale > 0.0 {
		SetLengthUnitsPerMeter(hdr.lengthScale)
	}

	// Count frames and read first step's dt so the viewer can show hz up front.
	RecScanFile(player)

	// Create the replay world. Debug-shape callbacks are NULL here; the sample wires
	// them right after Create via b3RecPlayer_SetDebugShapeCallbacks, which rebuilds.
	worldId := RecPlayerCreateWorld(player)

	// Initialize the reader.
	player.rdr.data = copy
	player.rdr.size = size
	player.rdr.cursor = headerEnd
	player.rdr.replayWorldId = worldId
	player.rdr.ok = true
	player.rdr.diverged = false
	player.rdr.owner = player

	// Load the trailing geometry registry.
	if !RecLoadSlots(&player.rdr, copy, size, hdr.registryOffset, hdr.registryByteCount) {
		DestroyWorld(worldId)
		Free(copy, int(size))
		Free(player, size_of(RecPlayer))
		return nil
	}

	// Restore the seed snapshot to stand up the replay world. The blob doubles as the frame-0
	// restore image, owned by the copy held above.
	{
		snapStart := i32(size_of(RecHeader))
		snapSize := i32(hdr.snapshotSize)
		replayWorld := GetWorldFromId(worldId)
		if DeserializeIntoShell(&copy[snapStart], snapSize, replayWorld, &player.rdr) == false {
			fmt.printf("b3RecPlayer_Create: snapshot deserialization failed\n")
			DestroyWorld(worldId)
			RecFreeSlots(player.rdr.slots, player.rdr.slotCount)
			if player.rdr.tags != nil {
				Free(player.rdr.tags, int(player.rdr.tagCapacity) * size_of(RecTag))
			}
			if player.rdr.tagMap != nil {
				// unified: b3RecTagLookup_cleanup
				delete((^RecTagLookup)(player.rdr.tagMap)^)
				// unified end
				Free(player.rdr.tagMap, size_of(RecTagLookup))
			}
			Free(copy, int(size))
			Free(player, size_of(RecPlayer))
			return nil
		}
		player.rdr.cursor = headerEnd
		player.frame0Image = &copy[snapStart]
		player.frame0Size = snapSize
	}

	// Seed the outliner from the restored world (snapshot bodies bypass the create hook) and save
	// the frame-0 restore copy.
	RecSeedFrame0BodyIds(player)

	// Build the keyframe recording with a pre-seeded registry that mirrors rdr.slots,
	// so b3SerializeWorld geometry ids stay stable across captures.
	player.keyframeRec = CreateRecording(0)
	RecSeedKeyframeRegistry(player)

	return player
}

DestroyPlayer :: proc(player: ^RecPlayer) {
	if player == nil {
		return
	}

	if World_IsValid(player.rdr.replayWorldId) {
		DestroyWorld(player.rdr.replayWorldId)
	}

	// Free shape data.
	RecFreeSlots(player.rdr.slots, player.rdr.slotCount)

	// Free reader scratch.
	if player.rdr.matScratch != nil {
		Free(player.rdr.matScratch, int(player.rdr.matScratchCap) * size_of(SurfaceMaterial))
	}
	if player.rdr.proxyScratch != nil {
		Free(player.rdr.proxyScratch, int(player.rdr.proxyScratchCap) * size_of(Vec3))
	}
	if player.rdr.hits != nil {
		Free(player.rdr.hits, int(player.rdr.hitCap) * size_of(RecRecordedHit))
	}
	if player.rdr.tags != nil {
		Free(player.rdr.tags, int(player.rdr.tagCapacity) * size_of(RecTag))
	}
	if player.rdr.tagMap != nil {
		// unified: b3RecTagLookup_cleanup
		delete((^RecTagLookup)(player.rdr.tagMap)^)
		// unified end
		Free(player.rdr.tagMap, size_of(RecTagLookup))
	}

	// Free the per-frame query store.
	if player.frameQueries != nil {
		Free(player.frameQueries, int(player.frameQueryCap) * size_of(RecDrawQuery))
	}
	if player.frameHits != nil {
		Free(player.frameHits, int(player.frameHitCap) * size_of(RecRecordedHit))
	}

	// Free keyframe ring.
	for i: i32 = 0; i < player.keyframeCount; i += 1 {
		FreeKeyframe(&player.keyframes[i])
	}
	if player.keyframes != nil {
		Free(player.keyframes, int(player.keyframeCapacity) * size_of(RecKeyframe))
	}

	// The keyframe recording owns only its buffer and registry; b3DestroyRecording frees both.
	if player.keyframeRec != nil {
		DestroyRecording(player.keyframeRec)
	}

	// Free the outliner body lists.
	if player.bodyIds != nil {
		Free(player.bodyIds, int(player.bodyIdCap) * size_of(BodyId))
	}
	if player.frame0BodyIds != nil {
		Free(player.frame0BodyIds, int(player.frame0BodyIdCount) * size_of(BodyId))
	}

	// frame0Image points into the owned data copy, not separately allocated.

	Free(player.data, int(player.size))

	// Restore the global length scale.
	SetLengthUnitsPerMeter(player.previousLengthScale)

	Free(player, size_of(RecPlayer))
}

RecPlayer_StepFrame :: proc(player: ^RecPlayer) -> bool {
	// This is never true when full stepping
	player.atPreStep = false

	if player.atEnd {
		return false
	}

	// Reset the per-frame query store before this frame's records are dispatched.
	player.frameQueryCount = 0
	player.frameHitCount = 0

	// A frame is its leading inputs (queries and between-step mutators), one Step, and the Step's
	// trailing StateHash. Queries are recorded before the Step they belong to, so they stash here
	// against the world state they were computed for.
	stepped := false
	for {
		if player.rdr.cursor >= player.registryEnd || !player.rdr.ok {
			player.atEnd = true
			return stepped
		}

		// Once stepped, the StateHash is the only record still belonging to this frame. Anything else
		// begins the next frame, so stop and let the next StepFrame consume it. Capture a keyframe at
		// the boundary.
		if stepped && player.rdr.data[player.rdr.cursor] != recOpStateHash {
			if player.frame > player.lastKeyframeFrame && player.frame % player.keyframeInterval == 0 {
				RecCaptureKeyframe(player)
			}
			return true
		}

		op := RecDispatchOne(&player.rdr)
		if op < 0 {
			player.atEnd = true
			return stepped
		}
		if op == recOpDestroyWorld { 	// end of recording
			player.atEnd = true
			return stepped
		}
		if op == recOpStep {
			player.frame += 1
			stepped = true
		} else if op == recOpStateHash { 	// trailing record of the frame just stepped
			// Latch the first frame whose state hash diverged. The hash belongs to the frame Step just
			// advanced, so latch against the current frame, not the next Step which would be one late.
			if player.divergeFrame < 0 && player.rdr.diverged {
				player.divergeFrame = player.frame
			}
		}
	}
}

RecPlayer_SubStepFrame :: proc(player: ^RecPlayer) {
	if player.atEnd {
		return
	}

	// Reset the per-frame query store before this frame's records are dispatched.
	if player.atPreStep == false {
		player.frameQueryCount = 0
		player.frameHitCount = 0
	}

	// A frame is its leading inputs (queries and between-step mutators), one Step, and the Step's
	// trailing StateHash. Queries are recorded before the Step they belong to, so they stash here
	// against the world state they were computed for.
	stepped := false
	haveCreateBodyOp := false
	for {
		if player.rdr.cursor >= player.registryEnd || !player.rdr.ok {
			player.atEnd = true
			player.atPreStep = false
			return
		}

		// Once stepped, the StateHash is the only record still belonging to this frame. Anything else
		// begins the next frame, so stop and let the next StepFrame consume it. Capture a keyframe at
		// the boundary.
		currentOpCode := player.rdr.data[player.rdr.cursor]
		if stepped && currentOpCode != recOpStateHash {
			if player.frame > player.lastKeyframeFrame && player.frame % player.keyframeInterval == 0 {
				RecCaptureKeyframe(player)
			}
			return
		}

		if player.atPreStep == false && haveCreateBodyOp == true && currentOpCode == recOpStep {
			player.atPreStep = true
			return
		}

		op := RecDispatchOne(&player.rdr)
		if op < 0 {
			player.atEnd = true
			player.atPreStep = false
			return
		}
		if op == recOpDestroyWorld { 	// end of recording
			player.atEnd = true
			player.atPreStep = false
			return
		}

		if op == recOpCreateBody {
			assert(player.atPreStep == false)
			haveCreateBodyOp = true
		}

		if op == recOpStep {
			player.atPreStep = false
			player.frame += 1
			stepped = true
		} else if op == recOpStateHash { 	// trailing record of the frame just stepped
			// Latch the first frame whose state hash diverged. The hash belongs to the frame Step just
			// advanced, so latch against the current frame, not the next Step which would be one late.
			if player.divergeFrame < 0 && player.rdr.diverged {
				player.divergeFrame = player.frame
			}
		}
	}
}

RecPlayer_Restart :: proc(player: ^RecPlayer) {
	// Restore the frame-0 image in place so the replay world id stays stable across a restart or
	// backward scrub. Stepping resumes at the first Step, which rebuilds the body list deterministically.
	world := GetWorldFromId(player.rdr.replayWorldId)
	if DeserializeIntoShell(player.frame0Image, player.frame0Size, world, &player.rdr) == false {
		player.rdr.ok = false
		return
	}
	player.rdr.cursor = player.headerEnd
	player.rdr.ok = true
	player.rdr.diverged = false
	player.frame = 0
	player.divergeFrame = -1
	player.atEnd = false
	player.atPreStep = false

	// Frame 0 is the pre-step snapshot with no recorded queries, so clear the per-frame store. This
	// keeps the last stepped frame's queries from lingering on a restart or a backward scrub to 0.
	player.frameQueryCount = 0
	player.frameHitCount = 0

	// Roll the outliner body list back to its frame-0 contents.
	RecGrow((^rawptr)(&player.bodyIds), &player.bodyIdCap, player.frame0BodyIdCount, 0, i32(size_of(BodyId)))
	player.bodyIdCount = player.frame0BodyIdCount
	if player.frame0BodyIdCount > 0 {
		mem.copy(player.bodyIds, player.frame0BodyIds, int(player.frame0BodyIdCount) * size_of(BodyId))
	}
}

RecPlayer_SeekFrame :: proc(player: ^RecPlayer, targetFrame: i32) {
	if player == nil {
		return
	}

	player.atPreStep = false

	targetFrame := targetFrame
	if targetFrame < 0 {
		targetFrame = 0
	}

	// Find the best keyframe strictly before the target.
	best: ^RecKeyframe = nil
	for i: i32 = 0; i < player.keyframeCount; i += 1 {
		kf := &player.keyframes[i]
		if kf.frame < targetFrame && (best == nil || kf.frame > best.frame) {
			best = kf
		}
	}

	if targetFrame < player.frame {
		// Backward seek: restore keyframe or restart from frame 0.
		if best != nil {
			RecPlayerRestoreKeyframe(player, best)
		} else {
			RecPlayer_Restart(player)
		}
	} else if best != nil && best.frame > player.frame {
		// Forward seek that can skip ahead via a keyframe.
		RecPlayerRestoreKeyframe(player, best)
	}

	for player.frame < targetFrame && RecPlayer_StepFrame(player) {
	}
}

RecPlayer_GetWorldId :: proc(player: ^RecPlayer) -> WorldId {
	return player != nil ? player.rdr.replayWorldId : nullWorldId
}

RecPlayer_GetFrame :: proc(player: ^RecPlayer) -> i32 {
	return player != nil ? player.frame : 0
}

RecPlayer_GetFrameCount :: proc(player: ^RecPlayer) -> i32 {
	return player != nil ? player.frameCount : 0
}

RecPlayer_IsAtEnd :: proc(player: ^RecPlayer) -> bool {
	return player != nil ? player.atEnd : true
}

RecPlayer_IsAtPreStep :: proc(player: ^RecPlayer) -> bool {
	return player != nil ? player.atPreStep : false
}

RecPlayer_HasDiverged :: proc(player: ^RecPlayer) -> bool {
	return player != nil ? player.rdr.diverged : false
}

RecPlayer_GetInfo :: proc(player: ^RecPlayer) -> RecPlayerInfo {
	info := RecPlayerInfo{}
	if player != nil {
		info.frameCount = player.frameCount
		info.workerCount = player.recordedWorkerCount
		info.timeStep = player.recordedDt
		info.subStepCount = player.recordedSubStepCount
		info.lengthScale = player.lengthScale
		info.bounds = player.bounds
	}
	return info
}

RecPlayer_GetDivergeFrame :: proc(player: ^RecPlayer) -> i32 {
	return player != nil ? player.divergeFrame : -1
}

RecPlayer_SetWorkerCount :: proc(player: ^RecPlayer, count: i32) {
	if player == nil {
		return
	}

	player.recordedWorkerCount = ClampInt(count, 1, MAX_WORKERS)

	// Apply to the live world now so the next steps re-partition without a rebuild. Worker count is
	// host state, not part of a keyframe image, so it survives an in-place restore. A rebuild on
	// Restart or deep backward seek picks the count back up through b3RecPlayerCreateWorld.
	if World_IsValid(player.rdr.replayWorldId) {
		World_SetWorkerCount(player.rdr.replayWorldId, player.recordedWorkerCount)
	}
}

RecPlayer_SetKeyframePolicy :: proc(player: ^RecPlayer, budgetBytes: int, minIntervalFrames: i32) {
	if player == nil {
		return
	}
	if budgetBytes > 0 {
		player.keyframeBudget = budgetBytes
	}
	if minIntervalFrames > 0 {
		player.keyframeMinInterval = minIntervalFrames
	}

	// Drop the ring so it repopulates under the new policy on the next replay.
	for i: i32 = 0; i < player.keyframeCount; i += 1 {
		FreeKeyframe(&player.keyframes[i])
	}
	player.keyframeCount = 0
	player.keyframeBytes = 0
	player.keyframeInterval = player.keyframeMinInterval
	player.lastKeyframeFrame = 0
}

RecPlayer_GetKeyframeBudget :: proc(player: ^RecPlayer) -> int {
	return player != nil ? player.keyframeBudget : 0
}

RecPlayer_GetKeyframeMinInterval :: proc(player: ^RecPlayer) -> i32 {
	return player != nil ? player.keyframeMinInterval : 0
}

RecPlayer_GetKeyframeInterval :: proc(player: ^RecPlayer) -> i32 {
	return player != nil ? player.keyframeInterval : 0
}

RecPlayer_GetKeyframeBytes :: proc(player: ^RecPlayer) -> int {
	return player != nil ? player.keyframeBytes : 0
}

RecPlayer_GetBodyCount :: proc(player: ^RecPlayer) -> i32 {
	return player != nil ? player.bodyIdCount : 0
}

RecPlayer_GetBodyId :: proc(player: ^RecPlayer, index: i32) -> BodyId {
	if player == nil || index < 0 || index >= player.bodyIdCount {
		return nullBodyId
	}
	return player.bodyIds[index]
}

// A selected query draws in one reserved color so it stands out when every query is drawn at once.
@(private = "file")
RecQuerySelColor :: proc(selected: bool, base: HexColor) -> HexColor {
	return selected ? .colorPlum : base
}

// Highlight each reported overlap shape by its AABB. Skip any destroyed since the query, per the
// b3Shape_GetAABB contract that overlap results may contain stale shapes.
@(private = "file")
RecDrawHitBounds :: proc(player: ^RecPlayer, q: ^RecDrawQuery, draw: ^DebugDraw, color: HexColor) {
	if draw.DrawBoundsFcn == nil {
		return
	}
	for hi := q.hitStart; hi < q.hitStart + q.hitCount; hi += 1 {
		id := player.frameHits[hi].id
		if Shape_IsValid(id) == false {
			continue
		}
		draw.DrawBoundsFcn(Shape_GetAABB(id), color, draw.context_)
	}
}

// Draw a recorded shape proxy at basePos. A lone point with no radius draws a fat point, a lone point
// with a radius draws a translucent sphere, and a multi-point cloud draws its points. Capsule and hull
// proxies are rare and fall through to the point cloud for now.
@(private = "file")
RecDrawProxy :: proc(draw: ^DebugDraw, basePos: Pos, q: ^RecDrawQuery, color: HexColor) {
	if q.proxyCount == 1 {
		p := OffsetPos(basePos, q.proxyPoints[0])
		if q.proxyRadius > 0.0 {
			if draw.DrawSphereFcn != nil {
				draw.DrawSphereFcn(p, q.proxyRadius, color, 0.5, draw.context_)
			}
		} else if draw.DrawPointFcn != nil {
			draw.DrawPointFcn(p, 10.0, color, draw.context_)
		}
	} else if q.proxyCount == 2 && q.proxyRadius > 0.0 {
		if draw.DrawCapsuleFcn != nil {
			p1 := OffsetPos(basePos, q.proxyPoints[0])
			p2 := OffsetPos(basePos, q.proxyPoints[1])
			draw.DrawCapsuleFcn(p1, p2, q.proxyRadius, color, 0.5, draw.context_)
		}
	} else if q.proxyCount >= 2 && draw.DrawPointFcn != nil {
		for i: i32 = 0; i < q.proxyCount; i += 1 {
			draw.DrawPointFcn(OffsetPos(basePos, q.proxyPoints[i]), 6.0, color, draw.context_)
		}
	}
}

RecPlayer_DrawFrameQueries :: proc(player: ^RecPlayer, draw: ^DebugDraw, queryIndex: i32, selectedIndex: i32) {
	if player == nil || draw == nil {
		return
	}

	// queryIndex < 0 draws every query, otherwise just the one selected in the viewer. The query at
	// selectedIndex draws in one reserved color and is labeled, so it stands out among the rest.
	for qi: i32 = 0; qi < player.frameQueryCount; qi += 1 {
		if queryIndex >= 0 && qi != queryIndex {
			continue
		}

		q := &player.frameQueries[qi]
		selected := (qi == selectedIndex)

		#partial switch RecQueryKind(q.kind) {
		case .RECQ_CAST_RAY, .RECQ_CAST_RAY_CLOSEST:
			origin := q.origin
			end := OffsetPos(origin, q.translation)
			if draw.DrawSegmentFcn != nil {
				draw.DrawSegmentFcn(origin, end, RecQuerySelColor(selected, .colorYellow), draw.context_)
			}
			for hi := q.hitStart; hi < q.hitStart + q.hitCount; hi += 1 {
				h := &player.frameHits[hi]
				if draw.DrawPointFcn != nil {
					draw.DrawPointFcn(h.point, 4.0, RecQuerySelColor(selected, .colorYellow), draw.context_)
				}
				if draw.DrawSegmentFcn != nil {
					draw.DrawSegmentFcn(
						h.point,
						OffsetPos(h.point, MulSV(0.2, h.normal)),
						RecQuerySelColor(selected, .colorYellowGreen),
						draw.context_,
					)
				}
			}
		case .RECQ_CAST_SHAPE:
			// Draw the cast line and the proxy at its start, then each hit point and normal.
			if draw.DrawSegmentFcn != nil {
				draw.DrawSegmentFcn(
					q.origin,
					OffsetPos(q.origin, q.translation),
					RecQuerySelColor(selected, .colorSkyBlue),
					draw.context_,
				)
			}
			RecDrawProxy(draw, q.origin, q, RecQuerySelColor(selected, .colorLightGreen))
			for hi := q.hitStart; hi < q.hitStart + q.hitCount; hi += 1 {
				h := &player.frameHits[hi]
				if draw.DrawPointFcn != nil {
					draw.DrawPointFcn(h.point, 4.0, RecQuerySelColor(selected, .colorSkyBlue), draw.context_)
				}
				if draw.DrawSegmentFcn != nil {
					draw.DrawSegmentFcn(
						h.point,
						OffsetPos(h.point, MulSV(0.2, h.normal)),
						RecQuerySelColor(selected, .colorLightSkyBlue),
						draw.context_,
					)
				}
				if draw.DrawSphereFcn != nil {
					p := OffsetPos(q.origin, MulSV(h.fraction, q.translation))
					RecDrawProxy(draw, p, q, RecQuerySelColor(selected, .colorSkyBlue))
				}
			}
		case .RECQ_CAST_MOVER:
			c1 := OffsetPos(q.origin, q.mover.center1)
			c2 := OffsetPos(q.origin, q.mover.center2)
			c := HexColor.colorLightSkyBlue
			if draw.DrawCapsuleFcn != nil {
				draw.DrawCapsuleFcn(c1, c2, q.mover.radius, RecQuerySelColor(selected, c), 0.6, draw.context_)

				if q.castFraction > 0.01 {
					d := MulSV(q.castFraction, q.translation)
					c1 = OffsetPos(c1, d)
					c2 = OffsetPos(c2, d)
					draw.DrawCapsuleFcn(c1, c2, q.mover.radius, c, 0.3, draw.context_)
				}
			}

		case .RECQ_COLLIDE_MOVER:
			c1 := OffsetPos(q.origin, q.mover.center1)
			c2 := OffsetPos(q.origin, q.mover.center2)
			c := HexColor.colorTan
			if draw.DrawCapsuleFcn != nil {
				draw.DrawCapsuleFcn(c1, c2, q.mover.radius, RecQuerySelColor(selected, c), 0.6, draw.context_)
			}

			for hi := q.hitStart; hi < q.hitStart + q.hitCount; hi += 1 {
				h := &player.frameHits[hi]
				point := OffsetPos(q.origin, h.plane.point)
				if draw.DrawSegmentFcn != nil {
					draw.DrawSegmentFcn(
						point,
						OffsetPos(point, MulSV(0.2, h.plane.plane.normal)),
						RecQuerySelColor(selected, .colorOrange),
						draw.context_,
					)
				}
			}

		case .RECQ_OVERLAP_AABB:
			if draw.DrawBoundsFcn != nil {
				draw.DrawBoundsFcn(q.aabb, RecQuerySelColor(selected, .colorLimeGreen), draw.context_)
			}
			RecDrawHitBounds(player, q, draw, RecQuerySelColor(selected, .colorMagenta))
		case .RECQ_OVERLAP_SHAPE:
			// The overlap proxy sits at the origin; draw it, then the overlapping shape bounds.
			RecDrawProxy(draw, q.origin, q, RecQuerySelColor(selected, .colorLimeGreen))
			RecDrawHitBounds(player, q, draw, RecQuerySelColor(selected, .colorMagenta))
		case:
		}

		// Label the selected query at its origin so it reads by name among the others. The overlap AABB
		// has no origin, so anchor at the box center. Untagged queries (no key) rely on the color alone.
		if selected && q.key != 0 && draw.DrawStringFcn != nil {
			name: cstring = nil
			id: u64 = 0
			if player.rdr.tagMap != nil {
				if index, found := (^RecTagLookup)(player.rdr.tagMap)[q.key]; found {
					tag := &player.rdr.tags[index]
					name = cstring(&tag.queryName[0])
					id = tag.id
				}
			}
			label: [64]u8
			// unified: snprintf is Odin's fmt into the buffer, NUL-terminated; "%.40s" is the name truncated to
			// 40 bytes before formatting
			nameStr := string(name)
			if len(nameStr) > 40 {
				nameStr = nameStr[:40]
			}
			s: string
			if name != nil && ([^]u8)(name)[0] != 0 && id != 0 {
				s = fmt.bprintf(label[:len(label) - 1], "%s (%d)", nameStr, id)
			} else if name != nil && ([^]u8)(name)[0] != 0 {
				s = fmt.bprintf(label[:len(label) - 1], "%s", nameStr)
			} else {
				s = fmt.bprintf(label[:len(label) - 1], "#%d", id)
			}
			label[len(s)] = 0
			text := cstring(&label[0])
			// unified end
			labelPos := q.origin
			if q.kind == i32(RecQueryKind.RECQ_OVERLAP_AABB) {
				labelPos = ToPos(AABB_Center(q.aabb))
			} else if q.kind == i32(RecQueryKind.RECQ_CAST_MOVER) || q.kind == i32(RecQueryKind.RECQ_COLLIDE_MOVER) {
				// Sit the label just past the center2 end cap, which for an upright mover is above it.
				c1 := OffsetPos(q.origin, q.mover.center1)
				c2 := OffsetPos(q.origin, q.mover.center2)
				dir := Normalize(SubPos(c2, c1))
				labelPos = OffsetPos(c2, MulSV(1.25 * q.mover.radius, dir))
			}
			draw.DrawStringFcn(labelPos, text, .colorWhite, draw.context_)
		}
	}
}

// The internal b3RecQueryKind values match the public b3RecQueryType, so the kind copies across as a
// plain cast. Pin the first and last kinds to catch enum drift.
#assert(
	i32(RecQueryType.recQueryOverlapAABB) == 0 && i32(RecQueryKind.RECQ_OVERLAP_AABB) == 0,
	"query type enum drift",
)
#assert(
	i32(RecQueryType.recQueryCollideMover) == 6 && i32(RecQueryKind.RECQ_COLLIDE_MOVER) == 6,
	"query type enum drift",
)

RecPlayer_GetFrameQueryCount :: proc(player: ^RecPlayer) -> i32 {
	return player != nil ? player.frameQueryCount : 0
}

RecPlayer_GetFrameQuery :: proc(player: ^RecPlayer, index: i32) -> RecQueryInfo {
	info := RecQueryInfo{}
	if player == nil || index < 0 || index >= player.frameQueryCount {
		return info
	}

	q := &player.frameQueries[index]
	info.type = RecQueryType(q.kind)
	info.filter = q.filter
	info.aabb = q.aabb
	info.origin = q.origin
	info.translation = q.translation
	info.hitCount = q.hitCount
	info.key = q.key
	info.id = 0
	info.name = nil
	if q.key != 0 && player.rdr.tagMap != nil {
		if index, found := (^RecTagLookup)(player.rdr.tagMap)[q.key]; found {
			tag := &player.rdr.tags[index]
			info.id = tag.id
			// An id-only tag interns an empty name; report it as none so the viewer shows the id alone.
			info.name = tag.queryName[0] != 0 ? cstring(&tag.queryName[0]) : nil
		}
	}
	return info
}

RecPlayer_GetFrameQueryHit :: proc(player: ^RecPlayer, queryIndex: i32, hitIndex: i32) -> RecQueryHit {
	hit := RecQueryHit{}
	if player == nil || queryIndex < 0 || queryIndex >= player.frameQueryCount {
		return hit
	}

	q := &player.frameQueries[queryIndex]
	if hitIndex < 0 || hitIndex >= q.hitCount {
		return hit
	}

	h := &player.frameHits[q.hitStart + hitIndex]
	hit.shape = h.id
	hit.point = h.point
	hit.normal = h.normal
	hit.fraction = h.fraction
	return hit
}

RecPlayer_SetDebugShapeCallbacks :: proc(
	player: ^RecPlayer,
	createDebugShape: CreateDebugShapeCallback,
	destroyDebugShape: DestroyDebugShapeCallback,
	context_: rawptr,
) {
	if player == nil {
		return
	}

	player.createDebugShape = createDebugShape
	player.destroyDebugShape = destroyDebugShape
	player.debugShapeContext = context_

	// A world fixes its debug-shape callbacks at creation, so rebuild frame 0 under the new
	// wiring. The old world held no adapter shapes (its callbacks were NULL), so the tear-down
	// is balanced. Geometry slots are byte blobs reused as-is, the same path Restart relies on.
	if World_IsValid(player.rdr.replayWorldId) {
		DestroyWorld(player.rdr.replayWorldId)
	}
	player.rdr.replayWorldId = RecPlayerCreateWorld(player)
	player.rdr.cursor = player.headerEnd
	player.rdr.ok = true
	player.rdr.diverged = false
	player.frame = 0
	player.divergeFrame = -1
	player.atEnd = false

	// Re-seed the world so its shapes are recreated through the new callbacks.
	world := GetWorldFromId(player.rdr.replayWorldId)
	if DeserializeIntoShell(player.frame0Image, player.frame0Size, world, &player.rdr) == false {
		player.rdr.ok = false
		return
	}
	player.rdr.cursor = player.headerEnd

	// Rebuild the outliner from the frame-0 world that was just stood up under the new callbacks.
	RecSeedFrame0BodyIds(player)
}
