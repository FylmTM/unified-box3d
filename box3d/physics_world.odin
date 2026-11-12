// Ported from box3d src/physics_world.c and src/physics_world.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:intrinsics"
import "base:runtime"
import "core:fmt"
import "core:mem"

DEBUG_POINT_CAPACITY :: 64
DEBUG_LINE_CAPACITY :: 64

SetType :: enum i32 {
	staticSet        = 0,
	disabledSet      = 1,
	awakeSet         = 2,
	firstSleepingSet = 3,
}

DebugPoint :: struct {
	p:     Pos,
	label: i32,
	value: f32,
	color: HexColor,
}

DebugLine :: struct {
	p1, p2: Pos,
	label:  i32,
	color:  HexColor,
}

// Per thread task storage
TaskContext :: struct {
	arena:                Arena,

	// Collect per thread sensor continuous hit events.
	sensorHits:           Array(SensorHit),

	// Broad-phase pairs.
	pairKeys:             Array(u64),

	// These bits align with the b3ConstraintGraph::contactBlocks and signal a change in contact status
	contactStateBitSet:   BitSet,

	// These bits align with the joint id capacity and signal a change in contact status
	jointStateBitSet:     BitSet,

	// These bits align with the contact id capacity and signal a hit event.
	hitEventBitSet:       BitSet,

	// Fast-path flag: true when this worker set at least one bit in hitEventBitSet this step.
	hasHitEvents:         bool,

	// Used to put islands to sleep
	awakeIslandBitSet:    BitSet,

	// Per worker split island candidate
	splitSleepTime:       f32,
	splitIslandId:        i32,

	// Profiling
	satCallCount:         i32,
	satCacheHitCount:     i32,
	distanceIterations:   i32,
	pushBackIterations:   i32,
	rootIterations:       i32,

	// Number of contacts recycled this step (collide pass).
	recycledContactCount: i32,
	points:               [DEBUG_POINT_CAPACITY]DebugPoint,
	pointCount:           i32,
	lines:                [DEBUG_LINE_CAPACITY]DebugLine,
	lineCount:            i32,
	manifoldCounts:       [CONTACT_MANIFOLD_COUNT_BUCKETS]i32,
	// Prevent false sharing
	cacheLine:            [64]u8,
}

// The world struct manages all physics entities, dynamic simulation,  and asynchronous queries.
// The world also contains efficient memory management facilities.
World :: struct {
	// unified: everything the world keeps between steps is allocated from here (world_arena.odin)
	arena:                        WorldArena,
	// unified end
	stack:                        Stack,
	broadPhase:                   BroadPhase,
	constraintGraph:              ConstraintGraph,

	// Manifold allocators have one allocator for each manifold count.
	manifoldAllocators:           Array(BlockAllocator),
	manifoldAllocatorMutex:       ^Mutex,

	// The body id pool is used to allocate and recycle body ids. Body ids
	// provide a stable identifier for users, but incur caches misses when used
	// to access body data. Aligns with b3Body.
	bodyIdPool:                   IdPool,

	// This is a sparse array that maps body ids to the body data
	// stored in solver sets. As sims move within a set or across set.
	// Indices come from id pool.
	bodies:                       Array(Body),

	// Provides free list for solver sets.
	solverSetIdPool:              IdPool,

	// Solvers sets allow sims to be stored in contiguous arrays. The first
	// set is all static sims. The second set is active sims. The third set is disabled
	// sims. The remaining sets are sleeping islands.
	solverSets:                   Array(SolverSet),

	// Used to create stable ids for joints
	jointIdPool:                  IdPool,

	// This is a sparse array that maps joint ids to the joint data stored in the constraint graph
	// or in the solver sets.
	joints:                       Array(Joint),

	// Used to create stable ids for contacts
	contactIdPool:                IdPool,

	// This is a sparse array that maps contact ids to the contact data stored in the constraint graph
	// or in the solver sets.
	contacts:                     Array(Contact),

	// Used to create stable ids for islands
	islandIdPool:                 IdPool,

	// This is a sparse array that maps island ids to the island data stored in the solver sets.
	islands:                      Array(Island),
	shapeIdPool:                  IdPool,

	// These are sparse arrays that point into the pools above
	shapes:                       Array(Shape),

	// This follows the shape array and is split out for the hot path in the narrow phase.
	fatAABBs:                     Array(AABB),

	// Compound shapes are static only, so only the broad-phase static pass can meet one. This
	// lets a world without compounds skip the shape type read in the pair batch.
	compoundShapeCount:           i32,

	// Reference counted store of shared hull data keyed by content. Shapes hold a
	// pointer to the hull stored in the db. Type erased to avoid leaking the verstable map
	// type into this header.
	hullDatabase:                 rawptr,

	// Name cache for shape and body names. This works with recording.
	names:                        NameCache,

	// This is a dense array of sensor data.
	sensors:                      Array(Sensor),

	// Per thread storage
	taskContexts:                 Array(TaskContext),
	sensorTaskContexts:           Array(SensorTaskContext),
	bodyMoveEvents:               Array(BodyMoveEvent),
	sensorBeginEvents:            Array(SensorBeginTouchEvent),
	contactBeginEvents:           Array(ContactBeginTouchEvent),

	// End events are double buffered so that the user doesn't need to flush events
	sensorEndEvents:              [2]Array(SensorEndTouchEvent),
	contactEndEvents:             [2]Array(ContactEndTouchEvent),
	endEventArrayIndex:           i32,
	contactHitEvents:             Array(ContactHitEvent),
	jointEvents:                  Array(JointEvent),

	// Used to track debug draw
	debugBodySet:                 BitSet,
	debugJointSet:                BitSet,
	debugContactSet:              BitSet,
	debugIslandSet:               BitSet,
	createDebugShape:             CreateDebugShapeCallback,
	destroyDebugShape:            DestroyDebugShapeCallback,
	userDebugShapeContext:        rawptr,

	// Id that is incremented every time step
	stepIndex:                    u64,

	// Identify islands for splitting as follows:
	// - I want to split islands so smaller islands can sleep
	// - when a body comes to rest and its sleep timer trips, I can look at the island and flag it for splitting
	//   if it has removed constraints
	// - islands that have removed constraints must be put split first because I don't want to wake bodies incorrectly
	// - otherwise I can use the awake islands that have bodies wanting to sleep as the splitting candidates
	// - if no bodies want to sleep then there is no reason to perform island splitting
	splitIslandId:                i32,
	gravity:                      Vec3,
	hitEventThreshold:            f32,
	restitutionThreshold:         f32,
	restitutionIterations:        i32,
	enableRestitutionPropagation: bool,
	maxLinearSpeed:               f32,
	isotropicInertiaTolerance:    f32,
	contactSpeed:                 f32,
	contactHertz:                 f32,
	contactDampingRatio:          f32,
	contactRecycleDistance:       f32,
	frictionCallback:             FrictionCallback,
	restitutionCallback:          RestitutionCallback,
	generation:                   u16,
	profile:                      Profile,
	satCallCount:                 i32,
	satCacheHitCount:             i32,
	manifoldCounts:               [CONTACT_MANIFOLD_COUNT_BUCKETS]i32,
	maxCapacity:                  Capacity,
	preSolveFcn:                  PreSolveFcn,
	preSolveContext:              rawptr,
	customFilterFcn:              CustomFilterFcn,
	customFilterContext:          rawptr,
	workerCount:                  i32,
	simdWidth:                    i32,
	enqueueTaskFcn:               EnqueueTaskCallback,
	finishTaskFcn:                FinishTaskCallback,
	userTaskContext:              rawptr,
	userTreeTask:                 rawptr,
	scheduler:                    ^Scheduler,
	userData:                     rawptr,

	// Non-NULL while a recording session is active. Set by b3World_StartRecording,
	// cleared by b3World_StopRecording. Hooks in mutators check this before writing.
	recording:                    ^Recording,

	// latest inverse sub-step
	inv_h:                        f32,

	// latest inverse full-step
	inv_dt:                       f32,
	activeTaskCount:              i32,
	taskCount:                    i32,
	worldId:                      u16,
	enableSleep:                  bool,

	// This indicates there is a world write operation in progress. This is for debugging and
	// not a real mutex. This should have minimal performance impact.
	locked:                       bool,
	enableWarmStarting:           bool,
	enableContinuous:             bool,
	enableSpeculative:            bool,
	inUse:                        bool,
}

AllocateManifolds :: proc(world: ^World, count: i32) -> [^]Manifold {
	if count == 0 {
		return nil
	}

	index := count - 1

	// Need lock because this is called from the parallel narrow phase
	LockMutex(world.manifoldAllocatorMutex)
	currentCount := world.manifoldAllocators.count
	for i := currentCount; i < count; i += 1 {
		allocator := CreateBlockAllocator((i + 1) * size_of(Manifold), 2 * BLOCK_SIZE)
		Array_Push(&world.manifoldAllocators, allocator)
	}

	allocator := Array_Get(&world.manifoldAllocators, index)
	manifolds := cast([^]Manifold)AllocateElement(allocator)
	UnlockMutex(world.manifoldAllocatorMutex)
	mem.zero(manifolds, int(count) * size_of(Manifold))
	return manifolds
}

FreeManifolds :: proc(world: ^World, manifolds: [^]Manifold, count: i32) {
	if count == 0 {
		return
	}

	index := count - 1
	LockMutex(world.manifoldAllocatorMutex)
	allocator := Array_Get(&world.manifoldAllocators, index)
	FreeElement(allocator, manifolds)
	UnlockMutex(world.manifoldAllocatorMutex)
}

// physics_world.c

#assert(MAX_WORLDS > 0, "must be 1 or more")
#assert(MAX_WORLDS < max(u16), "B3_MAX_WORLDS limit exceeded")
worlds: [MAX_WORLDS]World
worldCount: AtomicInt
maxWorldCount: i32

AddHullToDatabase :: proc(world: ^World, src: ^HullData) -> ^HullData {
	database := (^HullMap)(world.hullDatabase)

	// Compare by content to de-duplicate. Not trusting the hash.
	itr := HullMap_get(database, src)
	if HullMap_is_end(itr) == false {
		// Bump reference count.
		itr.data.val += 1
		return itr.data.key
	}

	// unified: the database is host memory (world_arena.odin): a rollback doesn't take hulls back
	HostScope()
	// unified end
	clone := CloneHull(src)
	assert(clone != nil)

	// Start with reference count of 1.
	HullMap_insert(database, clone, 1)
	return clone
}

AddOwnedHullToDatabase :: proc(world: ^World, owned: ^HullData) -> ^HullData {
	database := (^HullMap)(world.hullDatabase)

	itr := HullMap_get(database, owned)
	if HullMap_is_end(itr) == false {
		itr.data.val += 1
		DestroyHull(owned)
		return itr.data.key
	}

	// unified: the database is host memory (world_arena.odin); a hull made in the world's arena moves out
	owned := owned
	if ArenaOf(owned) != nil {
		HostScope()
		hostCopy := CloneHull(owned)
		DestroyHull(owned)
		owned = hostCopy
	}
	// unified end

	// Take ownership of input hull.
	HullMap_insert(database, owned, 1)
	return owned
}

RemoveHullFromDatabase :: proc(world: ^World, data: ^HullData) {
	database := (^HullMap)(world.hullDatabase)

	itr := HullMap_get(database, data)
	assert(HullMap_is_end(itr) == false)

	// unified: append-only while the world lives (a rollback may bring back a shape that uses this hull); the hulls
	// are destroyed with the world
	if itr.data.val > 0 {
		itr.data.val -= 1
	}
	// unified end
}

@(deferred_out = LeaveWorld) // unified: the world's arena for the rest of the caller's scope (world_arena.odin)
GetUnlockedWorldFromId :: proc(id: WorldId) -> ^World {
	assert(1 <= id.index1 && id.index1 <= MAX_WORLDS)
	world := &worlds[id.index1 - 1]
	EnterWorld(world) // unified: world_arena.odin
	assert(id.index1 == world.worldId + 1)
	assert(id.generation == world.generation)

	// A world accessed from an id should not be locked
	if world.locked {
		assert(false)
		return nil
	}
	return world
}

@(deferred_out = LeaveWorld) // unified: the world's arena for the rest of the caller's scope (world_arena.odin)
GetWorldFromId :: proc(id: WorldId) -> ^World {
	assert(1 <= id.index1 && id.index1 <= MAX_WORLDS)
	world := &worlds[id.index1 - 1]
	EnterWorld(world) // unified: world_arena.odin
	assert(id.index1 == world.worldId + 1)
	assert(id.generation == world.generation)
	return world
}

@(deferred_out = LeaveWorld) // unified: the world's arena for the rest of the caller's scope (world_arena.odin)
GetWorld :: proc(index: i32) -> ^World {
	assert(0 <= index && index < MAX_WORLDS)
	world := &worlds[index]
	EnterWorld(world) // unified: world_arena.odin
	assert(i32(world.worldId) == index)
	return world
}

@(deferred_out = LeaveWorld) // unified: the world's arena for the rest of the caller's scope (world_arena.odin)
GetUnlockedWorld :: proc(index: i32) -> ^World {
	assert(0 <= index && index < MAX_WORLDS)
	world := &worlds[index]
	EnterWorld(world) // unified: world_arena.odin
	assert(i32(world.worldId) == index)
	if world.locked {
		assert(false)
		return nil
	}

	return world
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
DefaultAddTaskFcn :: proc "c" (task: TaskCallback, taskContext: rawptr, userContext: rawptr, name: cstring) -> rawptr {
	// unified end
	_, _ = userContext, name
	task(taskContext)
	return nil
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
DefaultFinishTaskFcn :: proc "c" (userTask: rawptr, userContext: rawptr) {
	// unified end
	_ = userTask
	_ = userContext
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
DefaultFrictionCallback :: proc "c" (frictionA: f32, materialA: u64, frictionB: f32, materialB: u64) -> f32 {
	// unified end
	_, _ = materialA, materialB
	return intrinsics.sqrt(frictionA * frictionB)
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
DefaultRestitutionCallback :: proc "c" (restitutionA: f32, materialA: u64, restitutionB: f32, materialB: u64) -> f32 {
	_, _ = materialA, materialB
	return restitutionA > restitutionB ? restitutionA : restitutionB // b3MaxFloat, which needs a context in Odin
	// unified end
}

@(private = "file")
CreateWorkerContexts :: proc(world: ^World) {
	Array_Resize(&world.taskContexts, world.workerCount)
	Array_MemZero(&world.taskContexts)

	Array_Resize(&world.sensorTaskContexts, world.workerCount)
	Array_MemZero(&world.sensorTaskContexts)

	for i: i32 = 0; i < world.workerCount; i += 1 {
		world.taskContexts.data[i].arena = CreateArena(128 * 1024)
		Array_Reserve(&world.taskContexts.data[i].sensorHits, 8)
		Array_Reserve(&world.taskContexts.data[i].pairKeys, 64)
		world.taskContexts.data[i].contactStateBitSet = CreateBitSet(1024)
		world.taskContexts.data[i].hitEventBitSet = CreateBitSet(1024)
		world.taskContexts.data[i].hasHitEvents = false
		world.taskContexts.data[i].jointStateBitSet = CreateBitSet(1024)
		world.taskContexts.data[i].awakeIslandBitSet = CreateBitSet(256)
		world.taskContexts.data[i].splitIslandId = NULL_INDEX

		world.sensorTaskContexts.data[i].eventBits = CreateBitSet(128)
	}
}

@(private = "file")
DestroyWorkerContexts :: proc(world: ^World) {
	for i: i32 = 0; i < world.workerCount; i += 1 {
		DestroyArena(&world.taskContexts.data[i].arena)
		Array_Destroy(&world.taskContexts.data[i].sensorHits)
		Array_Destroy(&world.taskContexts.data[i].pairKeys)
		DestroyBitSet(&world.taskContexts.data[i].contactStateBitSet)
		DestroyBitSet(&world.taskContexts.data[i].hitEventBitSet)
		DestroyBitSet(&world.taskContexts.data[i].jointStateBitSet)
		DestroyBitSet(&world.taskContexts.data[i].awakeIslandBitSet)

		DestroyBitSet(&world.sensorTaskContexts.data[i].eventBits)
	}

	Array_Destroy(&world.taskContexts)
	Array_Destroy(&world.sensorTaskContexts)
}

CreateWorld :: proc(def: ^WorldDef) -> WorldId {
	CHECK_DEF(def)

	assert(LINEAR_SLOP <= MESH_REST_OFFSET)
	assert(MESH_REST_OFFSET < SPECULATIVE_DISTANCE)

	worldId: i32 = NULL_INDEX
	for i: i32 = 0; i < MAX_WORLDS; i += 1 {
		if worlds[i].inUse == false {
			worldId = i
			break
		}
	}

	if worldId == NULL_INDEX {
		Log("B3_MAX_WORLDS of %d exceeded!!!", MAX_WORLDS)
		assert(worldId != NULL_INDEX)
		return WorldId{}
	}

	// b3Log( "b3_lengthUnitsPerMeter = %g", b3_lengthUnitsPerMeter );

	oldCount := AtomicFetchAddInt(&worldCount, 1)
	maxWorldCount = MaxInt(maxWorldCount, oldCount + 1)

	// b3Log( "Created world %d", worldId );

	InitializeContactRegisters()

	world := &worlds[worldId]
	revision := world.generation

	mem.zero(world, size_of(World))

	world.simdWidth = GetSIMDWidth()

	world.worldId = u16(worldId)
	world.generation = revision
	world.inUse = true

	world.stack = CreateStack(2048)
	world.manifoldAllocatorMutex = CreateMutex()

	// unified: from here on, the world's memory comes from its arena (world_arena.odin); the scratch stack and the
	// mutex above are host memory
	CreateWorldArena(&world.arena, worldId)
	PushArena(&world.arena)
	defer PopArena()
	// unified end

	Array_Reserve(&world.manifoldAllocators, 16)

	CreateBroadPhase(&world.broadPhase, &def.capacity)
	CreateGraph(&world.constraintGraph, 16)

	// pools
	world.bodyIdPool = CreateIdPool()

	bodyCapacity := MaxInt(16, def.capacity.staticBodyCount + def.capacity.dynamicBodyCount)
	Array_Reserve(&world.bodies, bodyCapacity)
	Array_Reserve(&world.solverSets, 8)

	// add empty static, active, and disabled body sets
	world.solverSetIdPool = CreateIdPool()
	set := SolverSet{}

	// static set
	set.setIndex = AllocId(&world.solverSetIdPool)
	Array_Push(&world.solverSets, set)
	Array_Reserve(&world.solverSets.data[SetType.staticSet].bodySims, MaxInt(16, def.capacity.staticBodyCount))
	assert(world.solverSets.data[SetType.staticSet].setIndex == i32(SetType.staticSet))

	// disabled set
	set.setIndex = AllocId(&world.solverSetIdPool)
	Array_Push(&world.solverSets, set)
	assert(world.solverSets.data[SetType.disabledSet].setIndex == i32(SetType.disabledSet))

	// awake set
	set.setIndex = AllocId(&world.solverSetIdPool)
	Array_Push(&world.solverSets, set)
	Array_Reserve(&world.solverSets.data[SetType.awakeSet].bodySims, MaxInt(16, def.capacity.dynamicBodyCount))
	Array_Reserve(&world.solverSets.data[SetType.awakeSet].bodyStates, MaxInt(16, def.capacity.dynamicBodyCount))
	Array_Reserve(&world.solverSets.data[SetType.awakeSet].contactIndices, MaxInt(16, def.capacity.contactCount))
	assert(world.solverSets.data[SetType.awakeSet].setIndex == i32(SetType.awakeSet))

	world.shapeIdPool = CreateIdPool()

	shapeCapacity := MaxInt(16, def.capacity.staticShapeCount + def.capacity.dynamicShapeCount)
	Array_Reserve(&world.shapes, shapeCapacity)
	Array_Reserve(&world.fatAABBs, shapeCapacity)

	world.hullDatabase = AllocHost(size_of(HullMap)) // unified: host memory (world_arena.odin)
	HullMap_init((^HullMap)(world.hullDatabase))

	world.names = CreateNameCache()

	world.contactIdPool = CreateIdPool()
	Array_Reserve(&world.contacts, MaxInt(16, def.capacity.contactCount))

	world.jointIdPool = CreateIdPool()
	Array_Reserve(&world.joints, 16)

	world.islandIdPool = CreateIdPool()
	Array_Reserve(&world.islands, MaxInt(16, def.capacity.dynamicBodyCount))

	Array_Reserve(&world.sensors, 4)

	Array_Reserve(&world.bodyMoveEvents, 4)
	Array_Reserve(&world.sensorBeginEvents, 4)
	Array_Reserve(&world.sensorEndEvents[0], 4)
	Array_Reserve(&world.sensorEndEvents[1], 4)
	Array_Reserve(&world.contactBeginEvents, 4)
	Array_Reserve(&world.contactEndEvents[0], 4)
	Array_Reserve(&world.contactEndEvents[1], 4)
	Array_Reserve(&world.contactHitEvents, 4)
	Array_Reserve(&world.jointEvents, 4)
	world.endEventArrayIndex = 0

	world.stepIndex = 0
	world.splitIslandId = NULL_INDEX
	world.activeTaskCount = 0
	world.taskCount = 0
	world.gravity = def.gravity
	world.hitEventThreshold = def.hitEventThreshold
	world.restitutionThreshold = def.restitutionThreshold
	world.restitutionIterations = ClampInt(def.restitutionIterations, 0, MAX_RESTITUTION_ITERATIONS)
	world.enableRestitutionPropagation = def.enableRestitutionPropagation
	world.maxLinearSpeed = def.maximumLinearSpeed
	world.isotropicInertiaTolerance = ClampFloat(def.isotropicInertiaTolerance, 0.0, 1.0)
	world.contactSpeed = def.contactSpeed
	world.contactHertz = def.contactHertz
	world.contactDampingRatio = def.contactDampingRatio
	world.contactRecycleDistance = CONTACT_RECYCLE_DISTANCE

	if def.frictionCallback == nil {
		world.frictionCallback = DefaultFrictionCallback
	} else {
		world.frictionCallback = def.frictionCallback
	}

	if def.restitutionCallback == nil {
		world.restitutionCallback = DefaultRestitutionCallback
	} else {
		world.restitutionCallback = def.restitutionCallback
	}

	world.enableSleep = def.enableSleep
	world.locked = false
	world.enableWarmStarting = true
	world.enableContinuous = def.enableContinuous
	world.enableSpeculative = true
	world.userTreeTask = nil
	world.userData = def.userData

	if def.workerCount > 0 && def.enqueueTask != nil && def.finishTask != nil {
		// External task system
		world.workerCount = MinInt(i32(def.workerCount), MAX_WORKERS)
		world.enqueueTaskFcn = def.enqueueTask
		world.finishTaskFcn = def.finishTask
		world.userTaskContext = def.userTaskContext
		world.scheduler = nil
	} else if def.workerCount > 1 {
		// Built-in scheduler
		world.workerCount = MinInt(i32(def.workerCount), MAX_WORKERS)
		world.scheduler = CreateScheduler(world.workerCount)
		world.scheduler.arena = &world.arena // unified: its threads allocate from the world's arena (world_arena.odin)
		world.enqueueTaskFcn = SchedulerEnqueueTask
		world.finishTaskFcn = SchedulerFinishTask
		world.userTaskContext = world.scheduler
	} else {
		// Serial fallback
		world.workerCount = 1
		world.enqueueTaskFcn = DefaultAddTaskFcn
		world.finishTaskFcn = DefaultFinishTaskFcn
		world.userTaskContext = nil
		world.scheduler = nil
	}

	// unified: per-worker scratch is host memory, kept as is by a restore (world_arena.odin)
	{
		HostScope()
		CreateWorkerContexts(world)
	}
	// unified end

	world.debugBodySet = CreateBitSet(256)
	world.debugJointSet = CreateBitSet(256)
	world.debugContactSet = CreateBitSet(256)
	world.debugIslandSet = CreateBitSet(256)
	world.createDebugShape = def.createDebugShape
	world.destroyDebugShape = def.destroyDebugShape
	world.userDebugShapeContext = def.userDebugShapeContext

	// add one to worldId so that 0 represents a null b3WorldId
	return WorldId{u16(worldId + 1), world.generation}
}

DestroyWorld :: proc(worldId: WorldId) {
	AtomicFetchAddInt(&worldCount, -1)

	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	world.locked = true

	// Detach any recording before teardown. The user owns and frees the recording buffer.
	StopRecordingInternal(world)

	if world.scheduler != nil {
		DestroyScheduler(world.scheduler)
		world.scheduler = nil
	}

	DestroyBitSet(&world.debugBodySet)
	DestroyBitSet(&world.debugJointSet)
	DestroyBitSet(&world.debugContactSet)
	DestroyBitSet(&world.debugIslandSet)

	DestroyWorkerContexts(world)

	Array_Destroy(&world.bodyMoveEvents)
	Array_Destroy(&world.sensorBeginEvents)
	Array_Destroy(&world.sensorEndEvents[0])
	Array_Destroy(&world.sensorEndEvents[1])
	Array_Destroy(&world.contactBeginEvents)
	Array_Destroy(&world.contactEndEvents[0])
	Array_Destroy(&world.contactEndEvents[1])
	Array_Destroy(&world.contactHitEvents)
	Array_Destroy(&world.jointEvents)

	sensorCount := world.sensors.count
	for i: i32 = 0; i < sensorCount; i += 1 {
		Array_Destroy(&world.sensors.data[i].hits)
		Array_Destroy(&world.sensors.data[i].overlaps1)
		Array_Destroy(&world.sensors.data[i].overlaps2)
	}

	Array_Destroy(&world.sensors)
	Array_Destroy(&world.bodies)

	shapeCapacity := world.shapes.count
	shapes := world.shapes.data
	for i: i32 = 0; i < shapeCapacity; i += 1 {
		shape := &shapes[i]
		if shape.id != NULL_INDEX {
			DestroyShapeAllocations(world, &shapes[i])
		}
	}

	contactCapacity := world.contacts.count
	contacts := world.contacts.data
	for i: i32 = 0; i < contactCapacity; i += 1 {
		contact := &contacts[i]
		if contact.contactId != NULL_INDEX {
			if (contact.flags & simMeshContact) != 0 {
				Array_Destroy(&contact.meshContact.triangleCache)
			}
		}
	}

	// unified: the database is append-only while the world lives (world_arena.odin), so it still holds every hull
	// the world ever used: destroy them with it
	{
		database := (^HullMap)(world.hullDatabase)
		for itr := HullMap_first(database); !HullMap_is_end(itr); itr = HullMap_next(itr) {
			DestroyHull(itr.data.key)
		}
	}
	// unified end
	HullMap_cleanup((^HullMap)(world.hullDatabase))
	Free(world.hullDatabase, size_of(HullMap))
	world.hullDatabase = nil

	DestroyNameCache(&world.names)
	DestroyWorldImageData(world) // unified: the image geometry store and hash buffer (world_image.odin)

	Array_Destroy(&world.shapes)
	Array_Destroy(&world.fatAABBs)
	Array_Destroy(&world.contacts)
	Array_Destroy(&world.joints)

	for i: i32 = 0; i < world.islands.count; i += 1 {
		Array_Destroy(&world.islands.data[i].bodies)
		Array_Destroy(&world.islands.data[i].contacts)
		Array_Destroy(&world.islands.data[i].joints)
	}
	Array_Destroy(&world.islands)

	// Destroy solver sets
	setCapacity := world.solverSets.count
	for i: i32 = 0; i < setCapacity; i += 1 {
		set := &world.solverSets.data[i]
		if set.setIndex != NULL_INDEX {
			DestroySolverSet(world, i)
		}
	}

	Array_Destroy(&world.solverSets)

	DestroyGraph(&world.constraintGraph)
	DestroyBroadPhase(&world.broadPhase)

	DestroyIdPool(&world.bodyIdPool)
	DestroyIdPool(&world.shapeIdPool)
	DestroyIdPool(&world.contactIdPool)
	DestroyIdPool(&world.jointIdPool)
	DestroyIdPool(&world.islandIdPool)
	DestroyIdPool(&world.solverSetIdPool)

	for i: i32 = 0; i < world.manifoldAllocators.count; i += 1 {
		DestroyBlockAllocator(&world.manifoldAllocators.data[i])
	}
	Array_Destroy(&world.manifoldAllocators)
	DestroyMutex(world.manifoldAllocatorMutex)

	DestroyStack(&world.stack)

	// unified: release the arena (world_arena.odin); everything in it was freed above
	DestroyWorldArena(&world.arena)
	// unified end

	// Wipe world but preserve generation
	generation := world.generation
	mem.zero(world, size_of(World))
	world.generation = generation + 1

	// b3Log( "Destroyed world %d", worldId.index1 - 1 );
}

GetWorldCount :: proc() -> i32 {
	return AtomicLoadInt(&worldCount)
}

GetMaxWorldCount :: proc() -> i32 {
	return maxWorldCount
}

// Issues T0 prefetches across the cache lines of a b3Contact (216 B / 4 lines).
// Used to hide the random-access latency of contact lookups while we work on an
// earlier index.
@(private = "file")
PrefetchContact :: #force_inline proc(contact: ^Contact) {
	p := ([^]u8)(contact)
	Prefetch(p)
	Prefetch(&p[64])
	Prefetch(&p[128])
	Prefetch(&p[192])
}

@(private = "file")
ResolveContactBodySim :: #force_inline proc(
	world: ^World,
	awakeSims: [^]BodySim,
	staticSims: [^]BodySim,
	encodedIndex: i32,
	bodyId: i32,
) -> ^BodySim {
	if encodedIndex >= 0 {
		return &awakeSims[encodedIndex]
	}

	if IsStaticSimIndex(encodedIndex) {
		return &staticSims[-encodedIndex - 2]
	}

	body := Array_Get(&world.bodies, bodyId)
	set := Array_Get(&world.solverSets, body.setIndex)
	return Array_Get(&set.bodySims, body.localIndex)
}

@(private = "file")
CollideTask :: proc(startIndex: i32, endIndex: i32, workerIndex: i32, context_: rawptr) {
	stepContext := (^StepContext)(context_)
	world := stepContext.world
	graph := &world.constraintGraph
	taskContext := &world.taskContexts.data[workerIndex]
	contactIndices := stepContext.awakeContactIndices
	contacts := world.contacts.data
	shapes := world.shapes.data
	fatAABBs := world.fatAABBs.data
	awakeSims := world.solverSets.data[SetType.awakeSet].bodySims.data
	staticSims := world.solverSets.data[SetType.staticSet].bodySims.data

	assert(startIndex < endIndex)

	recycleDistance := world.contactRecycleDistance
	speculativeDistance := SPECULATIVE_DISTANCE
	recycleDistanceNonTouching := MinFloat(recycleDistance, speculativeDistance)

	contactPrefetchDistance: i32 : 4
	prefetchEnd := endIndex - contactPrefetchDistance

	for i := startIndex; i < endIndex; i += 1 {
		if i < prefetchEnd {
			PrefetchContact(&contacts[contactIndices[i + contactPrefetchDistance]])
		}

		contactIndex := contactIndices[i]
		assert(contactIndex < world.contacts.count)

		contact := &contacts[contactIndex]
		VALIDATE(contact.contactId == contactIndex)

		shapeIdA := contact.shapeIdA
		shapeIdB := contact.shapeIdB

		// Do proxies still overlap?
		overlap := OverlapV(&fatAABBs[shapeIdA], &fatAABBs[shapeIdB])
		if overlap == false {
			// This contact will be destroyed
			contact.flags |= simDisjoint
			contact.flags &= ~u32(simTouchingFlag)
			SetBit(&taskContext.contactStateBitSet, u32(contactIndex))
			continue
		}

		shapeA := &shapes[shapeIdA]
		shapeB := &shapes[shapeIdB]

		// Update contact respecting shape/body order (A,B)
		encodedA := contact.encodedBodySimA
		encodedB := contact.encodedBodySimB
		bodySimA := ResolveContactBodySim(world, awakeSims, staticSims, encodedA, contact.edges[0].bodyId)
		bodySimB := ResolveContactBodySim(world, awakeSims, staticSims, encodedB, contact.edges[1].bodyId)

		wasTouching := (contact.flags & simTouchingFlag) != 0

		transformA := bodySimA.transform
		transformB := bodySimB.transform

		// This is really just needed for mesh collision to prevent tunneling.
		// unified: the local isFast is isFast_ (the body flag constant is isFast)
		isFast_ := ((bodySimA.flags | bodySimB.flags) & isFast) != 0
		// unified end

		recycleTolerance := wasTouching ? recycleDistance : recycleDistanceNonTouching

		// Contact recycling optimization. Please cite this library if you use this optimization.
		// This is inspired by persistent contact manifolds used in some physics engines, such as PhysX.
		// However, this allows larger relative motion and has fewer tuning parameters (just one).
		if isFast_ == false &&
		   recycleDistance > 0.0 &&
		   (contact.flags & relativeTransformValid) != 0 &&
		   (contact.flags & contactRecycleFlag) != 0 {
			// The scalar part of b3InvMulQuat is just the quaternion dot product.
			// cos(relative_angle/2) = scalar(conj(q1) * q2) = dot(q1, q2)
			// A small relative angle means this value is close to 1. Need to use abs or square
			// due to double cover.
			angleA := DotQuat(transformA.q, contact.cachedRotationA)
			angleB := DotQuat(transformB.q, contact.cachedRotationB)
			angularDistance := MinFloat(angleA * angleA, angleB * angleB)

			xf := InvMulWorldTransforms(transformA, transformB)
			xfc := contact.cachedRelativePose
			maxExtentA := IsStaticSimIndex(encodedA) ? Vec3_zero : bodySimA.maxExtent
			maxExtentB := IsStaticSimIndex(encodedB) ? Vec3_zero : bodySimB.maxExtent
			maxExtent := Max(maxExtentA, maxExtentB)

			// Variation of Conservative Advancement
			// distance + 2 * length(modified_cross(|qr.v|, maxExtent)) < recycleTolerance.
			// 2*|qr.v| == 2*|sin(theta/2)| ~= theta for small angles.
			distSquared := DistanceSquared(xf.p, xfc.p)

			if angularDistance > CONTACT_RECYCLE_ANGULAR_DISTANCE &&
			   distSquared < recycleTolerance * recycleTolerance {
				distance := intrinsics.sqrt(distSquared)
				slack := recycleTolerance - distance

				// qr = inv( inv(qA0) * qB0 ) * inv(qA) * qB
				//    = inv(qB0) * qA0 * inv(qA) * qB
				// Suppose A is static
				// qr = inv(qB0) * qA0 * inv(qA0) * qB
				//    = inv(qB0) * qB
				// qB = qB0 * qr
				// Therefore qr is associated with the local angular velocity of body B when A is static.
				qr := InvMulQuat(xfc.q, xf.q)
				arc := ModifiedCross(Abs(qr.v), maxExtent)

				arcSq := 4.0 * LengthSquared(arc)
				if arcSq < slack * slack {
					dqA := MulQuat(transformA.q, Conjugate(contact.cachedRotationA))
					dqB := MulQuat(transformB.q, Conjugate(contact.cachedRotationB))
					matrixA := MakeMatrixFromQuat(dqA)
					matrixB := MakeMatrixFromQuat(dqB)

					// Minimize round-off
					dc := SubPos(bodySimB.center, bodySimA.center)

					manifoldCount := contact.manifoldCount
					for manifoldIndex: i32 = 0; manifoldIndex < manifoldCount; manifoldIndex += 1 {
						manifold := &contact.manifolds[manifoldIndex]
						normal := manifold.normal

						pointCount := manifold.pointCount
						for pointIndex: i32 = 0; pointIndex < pointCount; pointIndex += 1 {
							// Keep anchors but update separation, same as sub-stepping. This eliminates jitter.
							mp := &manifold.points[pointIndex]
							rA := MulMV(matrixA, mp.anchorA)
							rB := MulMV(matrixB, mp.anchorB)
							dp := Add(dc, Sub(rB, rA))
							mp.separation = mp.baseSeparation + Dot(dp, normal)
							mp.normalVelocity = 0.0
							mp.persisted = true
						}
					}

					// Diagnostics
					taskContext.recycledContactCount += 1
					bucketIndex := MinInt(manifoldCount, CONTACT_MANIFOLD_COUNT_BUCKETS - 1)
					if bucketIndex > 0 {
						taskContext.manifoldCounts[bucketIndex - 1] += 1
					}

					// Contact is recycled. This also skips updating other aspects of the contact
					// such as material parameters.
					continue
				}
			}
		}

		// Caching for contact recycling.
		contact.cachedRotationA = transformA.q
		contact.cachedRotationB = transformB.q
		contact.cachedRelativePose = InvMulWorldTransforms(transformA, transformB)
		contact.flags |= relativeTransformValid

		// This updates solid contacts
		touching := UpdateContact(
			world,
			workerIndex,
			contact,
			shapeA,
			bodySimA.localCenter,
			transformA,
			shapeB,
			bodySimB.localCenter,
			transformB,
			isFast_,
			taskContext.arena,
		)

		bucketIndex := MinInt(contact.manifoldCount, CONTACT_MANIFOLD_COUNT_BUCKETS - 1)
		if bucketIndex > 0 {
			taskContext.manifoldCounts[bucketIndex - 1] += 1
		}

		// Update the mesh contact spec
		if touching == true && wasTouching == true && (contact.flags & simMeshContact) != 0 {
			assert(contact.colorIndex != NULL_INDEX)
			assert(0 <= contact.colorIndex && contact.colorIndex < GRAPH_COLOR_COUNT)
			color := &graph.colors[contact.colorIndex]
			spec := Array_Get(&color.contacts, contact.localIndex)
			spec.manifoldCount = u16(contact.manifoldCount)
		}

		// State changes that affect island connectivity. Also affects contact events.
		if touching == true && wasTouching == false {
			contact.flags |= simStartedTouching
			SetBit(&taskContext.contactStateBitSet, u32(contactIndex))
		} else if touching == false && wasTouching == true {
			contact.flags |= simStoppedTouching
			SetBit(&taskContext.contactStateBitSet, u32(contactIndex))
		}

		if touching {
			manifoldCount := contact.manifoldCount
			for manifoldIndex: i32 = 0; manifoldIndex < manifoldCount; manifoldIndex += 1 {
				manifold := &contact.manifolds[manifoldIndex]
				pointCount := manifold.pointCount
				for pointIndex: i32 = 0; pointIndex < pointCount; pointIndex += 1 {
					// Cache separation
					mp := &manifold.points[pointIndex]
					mp.baseSeparation = mp.separation
				}
			}
		}
	}
}

@(private = "file")
AddNonTouchingContact :: proc(world: ^World, contact: ^Contact) {
	assert(contact.setIndex == i32(SetType.awakeSet))
	set := Array_Get(&world.solverSets, i32(SetType.awakeSet))
	contact.colorIndex = NULL_INDEX
	contact.localIndex = set.contactIndices.count
	Array_Push(&set.contactIndices, contact.contactId)
}

@(private = "file")
RemoveNonTouchingContact :: proc(world: ^World, setIndex: i32, localIndex: i32) {
	set := Array_Get(&world.solverSets, setIndex)
	movedIndex := Array_RemoveSwap(&set.contactIndices, localIndex)
	if movedIndex != NULL_INDEX {
		movedContactIndex := set.contactIndices.data[localIndex]
		movedContact := Array_Get(&world.contacts, movedContactIndex)
		assert(movedContact.setIndex == setIndex)
		assert(movedContact.colorIndex == NULL_INDEX)
		assert(movedContact.localIndex == movedIndex)
		movedContact.localIndex = localIndex
	}
}

// Narrow-phase collision
@(private = "file")
Collide :: proc(context_: ^StepContext) {
	world := context_.world

	assert(world.workerCount > 0)

	// Gather contacts from all the graph colors into a single array for easier parallel-for
	touchingCount: i32 = 0

	graphColors := &world.constraintGraph.colors
	for i := 0; i < GRAPH_COLOR_COUNT; i += 1 {
		touchingCount += graphColors[i].convexContacts.count + graphColors[i].contacts.count
	}

	awakeSet := Array_Get(&world.solverSets, i32(SetType.awakeSet))
	nonTouchingCount := awakeSet.contactIndices.count

	contactCount := touchingCount + nonTouchingCount

	if contactCount == 0 {
		return
	}

	contactIndices := cast([^]i32)StackAlloc(&world.stack, contactCount * size_of(i32), "contact indices")

	contactIndex: i32 = 0
	for i := 0; i < GRAPH_COLOR_COUNT; i += 1 {
		color := &graphColors[i]
		count := color.convexContacts.count
		for j: i32 = 0; j < count; j += 1 {
			contactIndices[contactIndex] = color.convexContacts.data[j]
			contactIndex += 1
		}

		count = color.contacts.count
		for j: i32 = 0; j < count; j += 1 {
			contactIndices[contactIndex] = color.contacts.data[j].contactId
			contactIndex += 1
		}
	}

	assert(contactIndex == touchingCount)

	if nonTouchingCount > 0 {
		nonTouchingIndices := awakeSet.contactIndices.data
		mem.copy(&contactIndices[touchingCount], nonTouchingIndices, int(nonTouchingCount) * size_of(i32))
	}

	context_.awakeContactIndices = contactIndices

	// Contact bit set on ids because contact pointers are unstable as they move between touching and not touching.
	contactIdCapacity := GetIdCapacity(&world.contactIdPool)
	for i: i32 = 0; i < world.workerCount; i += 1 {
		taskContext := &world.taskContexts.data[i]
		SetBitCountAndClear(&taskContext.contactStateBitSet, u32(contactIdCapacity))
		taskContext.satCallCount = 0
		taskContext.satCacheHitCount = 0
		taskContext.recycledContactCount = 0
		mem.zero(&taskContext.manifoldCounts, size_of(taskContext.manifoldCounts))
	}

	// Task should take at least 40us on a 4GHz CPU (10K cycles)
	minRange: i32 = 20
	ParallelFor(world, CollideTask, contactCount, minRange, context_, "collide")

	StackFree(&world.stack, contactIndices)
	context_.awakeContactIndices = nil
	contactIndices = nil

	// Serially update contact state
	// todo bring this zone together with island merge

	satMultiplier: i32 = context_.dt > 0.0 ? 1 : 0

	// Bitwise OR all contact bits
	bitSet := &world.taskContexts.data[0].contactStateBitSet
	world.satCallCount = satMultiplier * world.taskContexts.data[0].satCallCount
	world.satCacheHitCount = satMultiplier * world.taskContexts.data[0].satCacheHitCount
	mem.copy(
		&world.manifoldCounts,
		&world.taskContexts.data[0].manifoldCounts,
		CONTACT_MANIFOLD_COUNT_BUCKETS * size_of(i32),
	)

	for i: i32 = 1; i < world.workerCount; i += 1 {
		InPlaceUnion(bitSet, &world.taskContexts.data[i].contactStateBitSet)
		world.satCallCount += world.taskContexts.data[i].satCallCount
		world.satCacheHitCount += world.taskContexts.data[i].satCacheHitCount
		for j := 0; j < CONTACT_MANIFOLD_COUNT_BUCKETS; j += 1 {
			world.manifoldCounts[j] += world.taskContexts.data[i].manifoldCounts[j]
		}
	}

	// Release per-step overflow blocks and grow the backing capacity if last
	// step's demand exceeded it. Contact processing is the only consumer of
	// these arenas and is finished by this point.
	for i: i32 = 0; i < world.workerCount; i += 1 {
		ArenaSync(&world.taskContexts.data[i].arena)
	}

	endEventArrayIndex := world.endEventArrayIndex

	shapes := world.shapes.data
	worldId := world.worldId

	// Process contact state changes. Iterate over set bits
	for k: u32 = 0; k < bitSet.blockCount; k += 1 {
		bits := bitSet.bits[k]
		for bits != 0 {
			ctz := CTZ64(bits)
			contactId := i32(64 * k + ctz)

			contact := Array_Get(&world.contacts, contactId)
			assert(contact.setIndex == i32(SetType.awakeSet))

			shapeA := &shapes[contact.shapeIdA]
			shapeB := &shapes[contact.shapeIdB]
			shapeIdA := ShapeId{shapeA.id + 1, worldId, shapeA.generation}
			shapeIdB := ShapeId{shapeB.id + 1, worldId, shapeB.generation}
			contactFullId := ContactId {
				index1     = contactId + 1,
				world0     = worldId,
				padding    = 0,
				generation = contact.generation,
			}
			flags := contact.flags

			if (flags & simDisjoint) != 0 {
				// Bounding boxes no longer overlap
				DestroyContact(world, contact, false)
				contact = nil
			} else if (flags & simStartedTouching) != 0 {
				assert(contact.islandId == NULL_INDEX)

				if (flags & contactEnableContactEvents) != 0 {
					event := ContactBeginTouchEvent{shapeIdA, shapeIdB, contactFullId}
					Array_Push(&world.contactBeginEvents, event)
				}

				assert(contact.manifoldCount > 0)
				assert(contact.setIndex == i32(SetType.awakeSet))

				// Link first because this wakes colliding bodies and ensures the body sims
				// are in the correct place.
				contact.flags &= ~u32(simStartedTouching)
				contact.flags |= contactTouchingFlag
				LinkContact(world, contact)

				// Make sure these didn't change
				assert(contact.colorIndex == NULL_INDEX)

				// Contact sim pointer may have become orphaned due to awake set growth,
				// so I just need to refresh it.

				oldLocalIndex := contact.localIndex

				AddContactToGraph(world, contact)
				RemoveNonTouchingContact(world, i32(SetType.awakeSet), oldLocalIndex)
			} else if (flags & simStoppedTouching) != 0 {
				contact.flags &= ~u32(simStoppedTouching)
				contact.flags &= ~u32(contactTouchingFlag)

				if (contact.flags & contactEnableContactEvents) != 0 {
					event := ContactEndTouchEvent{shapeIdA, shapeIdB, contactFullId}
					Array_Push(&world.contactEndEvents[endEventArrayIndex], event)
				}

				assert(contact.manifoldCount == 0)

				// Cache these here for the remove below
				colorIndex := contact.colorIndex
				localIndex := contact.localIndex

				UnlinkContact(world, contact)
				bodyIdA := contact.edges[0].bodyId
				bodyIdB := contact.edges[1].bodyId

				AddNonTouchingContact(world, contact)

				isMeshContact := (contact.flags & simMeshContact) != 0
				RemoveContactFromGraph(world, bodyIdA, bodyIdB, colorIndex, localIndex, isMeshContact)
				contact = nil
			}

			// Clear the smallest set bit
			bits = bits & (bits - 1)
		}
	}

	ValidateSolverSets(world)
	ValidateContacts(world)
}

World_Step :: proc(worldId: WorldId, timeStep: f32, subStepCount: i32) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_Step{worldId, timeStep, subStepCount}
		RecWrite_Step(world.recording, &recArgs)
	}

	world.locked = true

	// Clear debug buffers
	for i: i32 = 0; i < world.workerCount; i += 1 {
		world.taskContexts.data[i].pointCount = 0
		world.taskContexts.data[i].lineCount = 0
	}

	// Prepare to capture events
	// Ensure user does not access stale data if there is an early return
	Array_Clear(&world.bodyMoveEvents)
	Array_Clear(&world.sensorBeginEvents)
	Array_Clear(&world.contactBeginEvents)
	Array_Clear(&world.contactHitEvents)
	Array_Clear(&world.jointEvents)

	world.profile = Profile{}

	world.activeTaskCount = 0
	world.taskCount = 0

	if world.scheduler != nil {
		ResetScheduler(world.scheduler)
	}

	stepTicks := GetTicks()

	{
		c := &world.maxCapacity
		c.staticShapeCount = MaxInt(c.staticShapeCount, world.broadPhase.trees[BodyType.staticBody].proxyCount)
		c.dynamicShapeCount = MaxInt(c.dynamicShapeCount, world.broadPhase.trees[BodyType.dynamicBody].proxyCount)

		staticBodyCount := world.solverSets.data[SetType.staticSet].bodySims.count
		c.staticBodyCount = MaxInt(c.staticBodyCount, staticBodyCount)

		// this includes kinematic bodies
		totalBodyCount := GetIdCount(&world.bodyIdPool)
		c.dynamicBodyCount = MaxInt(c.dynamicBodyCount, totalBodyCount - staticBodyCount)

		totalContactCount := GetIdCount(&world.contactIdPool)
		c.contactCount = MaxInt(c.contactCount, totalContactCount)
	}

	// Update collision pairs and create contacts
	{
		pairTicks := GetTicks()
		UpdateBroadPhasePairs(world)
		world.profile.pairs = GetMilliseconds(pairTicks)
	}

	awakeSet := Array_Get(&world.solverSets, i32(SetType.awakeSet))

	context_ := StepContext{}
	context_.world = world
	context_.states = awakeSet.bodyStates.data
	context_.dt = timeStep
	context_.subStepCount = MaxInt(1, subStepCount)

	if timeStep > 0.0 {
		context_.inv_dt = 1.0 / timeStep
		context_.h = timeStep / f32(context_.subStepCount)
		context_.inv_h = f32(context_.subStepCount) * context_.inv_dt
	} else {
		context_.inv_dt = 0.0
		context_.h = 0.0
		context_.inv_h = 0.0
	}

	world.inv_h = context_.inv_h
	world.inv_dt = context_.inv_dt

	// Hertz values get reduced for large time steps
	contactHertz := MinFloat(world.contactHertz, 0.125 * context_.inv_h)
	context_.contactSoftness = MakeSoft(contactHertz, world.contactDampingRatio, context_.h)
	context_.staticSoftness = MakeSoft(2.0 * contactHertz, 0.5 * world.contactDampingRatio, context_.h)

	context_.maxLinearVelocity = world.maxLinearSpeed
	context_.enableWarmStarting = world.enableWarmStarting

	// Narrow phase : update contacts
	{
		collideTicks := GetTicks()
		Collide(&context_)
		world.profile.collide = GetMilliseconds(collideTicks)
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if timeStep > 0.0 {
		solveTicks := GetTicks()
		Solve(world, &context_)
		world.profile.solve = GetMilliseconds(solveTicks)
	}

	// Finish the tree task in case b3Solve didn't finish it
	if world.userTreeTask != nil {
		world.finishTaskFcn(world.userTreeTask, world.userTaskContext)
		world.userTreeTask = nil
		world.activeTaskCount -= 1
		ValidateNoMoved(&world.broadPhase)
	}

	// Update sensors
	{
		sensorTicks := GetTicks()
		OverlapSensors(world)
		world.profile.sensors = GetMilliseconds(sensorTicks)
	}

	world.profile.step = GetMilliseconds(stepTicks)

	assert(world.stack.allocation == 0)

	// Ensure stack is large enough
	GrowStack(&world.stack)

	// Make sure all tasks that were started were also finished
	assert(world.activeTaskCount == 0)

	// Swap end event array buffers
	world.endEventArrayIndex = 1 - world.endEventArrayIndex
	Array_Clear(&world.sensorEndEvents[world.endEventArrayIndex])
	Array_Clear(&world.contactEndEvents[world.endEventArrayIndex])
	world.locked = false

	if world.recording != nil {
		hash := HashWorldState(world)
		stateHash := RecArgs_StateHash{worldId, hash}
		RecWrite_StateHash(world.recording, &stateHash)

		// Fold this step's world bounds into the recording so a viewer can frame the whole motion.
		worldBounds := AABB{}
		haveBounds := false
		for i: i32 = 0; i < i32(BodyType.bodyTypeCount); i += 1 {
			tree := &world.broadPhase.trees[i]
			if DynamicTree_GetProxyCount(tree) == 0 {
				continue
			}
			bounds := DynamicTree_GetRootBounds(tree)
			worldBounds = haveBounds ? AABB_Union(worldBounds, bounds) : bounds
			haveBounds = true
		}
		if haveBounds {
			RecAccumulateBounds(world.recording, worldBounds)
		}
	}
}

// unified: snprintf for the debug draw labels: Odin's fmt into the buffer, NUL-terminated (fmt verbs match printf's
// for these labels, except that %#X prints Odin's 0X prefix also for zero)
@(private = "file")
snprintf :: proc(buffer: []u8, format: string, args: ..any) -> cstring {
	s := fmt.bprintf(buffer[:len(buffer) - 1], format, ..args)
	buffer[len(s)] = 0
	return cstring(raw_data(buffer))
}
// unified end

DrawContext :: struct {
	world: ^World,
	draw:  ^DebugDraw,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
DrawQueryCallback :: proc "c" (proxyId: i32, userData: u64, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	shapeId := i32(userData)

	_ = proxyId

	drawContext := (^DrawContext)(context_)
	world := drawContext.world
	draw := drawContext.draw

	shape := Array_Get(&world.shapes, shapeId)
	assert(shape.id == shapeId)

	SetBit(&world.debugBodySet, u32(shape.bodyId))

	if draw.drawShapes {
		body := Array_Get(&world.bodies, shape.bodyId)
		bodySim := GetBodySim(world, body)

		color: HexColor

		surfaceMaterial := GetShapeMaterials(shape)
		if surfaceMaterial[0].customColor != 0 {
			// May already carry a packed material preset, pass through unchanged
			color = HexColor(surfaceMaterial[0].customColor)
		} else {
			// Hue carries the state, material carries its energy. Calm matte for the
			// resting masses, glossy for fast bodies, metallic for the driven kinematic.
			// Diagnostic states keep a saturated hue and the default material so they pop.
			rgb: HexColor
			material := DebugMaterial.debugMaterialDefault

			if body.type == .dynamicBody && body.mass == 0.0 {
				// Bad body
				rgb = .colorRed
			} else if body.setIndex == i32(SetType.disabledSet) {
				rgb = .colorSlateGray
			} else if shape.sensorIndex != NULL_INDEX {
				rgb = .colorWheat
			} else if (bodySim.flags & hadTimeOfImpact) != 0 {
				rgb = .colorLime
			} else if (bodySim.flags & isBullet) != 0 && body.setIndex == i32(SetType.awakeSet) {
				rgb = .colorTurquoise
			} else if (body.flags & isSpeedCapped) != 0 {
				rgb = .colorYellow
			} else if (bodySim.flags & isFast) != 0 {
				rgb = .colorOrange
				material = .debugMaterialGlossy
			} else if body.type == .staticBody {
				rgb = .colorDarkGray
				material = .debugMaterialMatte
			} else if body.type == .kinematicBody {
				if body.setIndex == i32(SetType.awakeSet) {
					rgb = .colorSteelBlue
					material = .debugMaterialMetallic
				} else {
					rgb = .colorLightSteelBlue
					material = .debugMaterialMatte
				}
			} else if body.setIndex == i32(SetType.awakeSet) {
				rgb = .colorTan
				material = .debugMaterialSoft
			} else {
				rgb = .colorLightSlateGray
				material = .debugMaterialDead
			}

			color = HexColor(MakeDebugColor(rgb, material))
		}

		if shape.userShape == nil && world.createDebugShape != nil {
			debugShape := DebugShape{}
			debugShape.shapeId = ShapeId {
				world0     = world.worldId,
				index1     = shapeId + 1,
				generation = shape.generation,
			}
			debugShape.type = shape.type

			#partial switch shape.type {
			case .capsuleShape:
				debugShape.capsule = &shape.capsule
				shape.userShape = world.createDebugShape(&debugShape, world.userDebugShapeContext)
			case .compoundShape:
				debugShape.compound = shape.compound
				shape.userShape = world.createDebugShape(&debugShape, world.userDebugShapeContext)
			case .heightShape:
				debugShape.heightField = shape.heightField
				shape.userShape = world.createDebugShape(&debugShape, world.userDebugShapeContext)
			case .hullShape:
				debugShape.hull = shape.hull
				shape.userShape = world.createDebugShape(&debugShape, world.userDebugShapeContext)
			case .meshShape:
				debugShape.mesh = &shape.mesh
				shape.userShape = world.createDebugShape(&debugShape, world.userDebugShapeContext)
			case .sphereShape:
				debugShape.sphere = &shape.sphere
				shape.userShape = world.createDebugShape(&debugShape, world.userDebugShapeContext)
			case:
				assert(false)
			}
		}

		if shape.userShape != nil {
			draw.DrawShapeFcn(shape.userShape, bodySim.transform, color, draw.context_)
		}
	}

	if draw.drawBounds {
		draw.DrawBoundsFcn(world.fatAABBs.data[shapeId], .colorGold, draw.context_)
	}

	return true
}

World_Draw :: proc(worldId: WorldId, draw: ^DebugDraw, maskBits: u64) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	assert(IsValidAABB(draw.drawingBounds))

	lengthScale := GetLengthUnitsPerMeter()
	axisScale := 0.3 * lengthScale
	farColor := HexColor.colorDarkBlue
	speculativeColor := HexColor.colorBlue
	addColor := HexColor.colorLimeGreen
	persistColor := HexColor.colorLightBlue
	// b3HexColor normalColors[3] = { b3_colorLightGray, b3_colorLightSalmon, b3_colorLightSeaGreen };
	impulseColor := HexColor.colorMagenta
	frictionColor := HexColor.colorYellow

	graphColors := [GRAPH_COLOR_COUNT]HexColor {
		.colorRed,
		.colorOrange,
		.colorYellow,
		.colorGreen,
		.colorCyan,
		.colorBlue,
		.colorViolet,
		.colorPink,
		.colorChocolate,
		.colorGoldenRod,
		.colorCoral,
		.colorRosyBrown,
		.colorAqua,
		.colorPeru,
		.colorLime,
		.colorGold,
		.colorPlum,
		.colorSnow,
		.colorTeal,
		.colorKhaki,
		.colorSalmon,
		.colorPeachPuff,
		.colorHoneyDew,
		.colorBlack,
	}

	bodyCapacity := GetIdCapacity(&world.bodyIdPool)
	SetBitCountAndClear(&world.debugBodySet, u32(bodyCapacity))

	jointCapacity := GetIdCapacity(&world.jointIdPool)
	SetBitCountAndClear(&world.debugJointSet, u32(jointCapacity))

	contactCapacity := GetIdCapacity(&world.contactIdPool)
	SetBitCountAndClear(&world.debugContactSet, u32(contactCapacity))

	islandCapacity := GetIdCapacity(&world.islandIdPool)
	SetBitCountAndClear(&world.debugIslandSet, u32(islandCapacity))

	drawContext := DrawContext{world, draw}

	for i: i32 = 0; i < i32(BodyType.bodyTypeCount); i += 1 {
		DynamicTree_Query(
			&world.broadPhase.trees[i],
			draw.drawingBounds,
			maskBits,
			false,
			DrawQueryCallback,
			&drawContext,
		)
	}

	wordCount := world.debugBodySet.blockCount
	bits := world.debugBodySet.bits
	for wordIndex: u32 = 0; wordIndex < wordCount; wordIndex += 1 {
		word := bits[wordIndex]
		for word != 0 {
			ctz := CTZ64(word)
			bodyId := i32(64 * wordIndex + ctz)

			body := Array_Get(&world.bodies, bodyId)
			bodySim := GetBodySim(world, body)

			if draw.drawBodyNames && body.nameId != NULL_NAME {
				offset := Vec3{0.05, 0.05, 0.05}
				transform := WorldTransform{bodySim.center, bodySim.transform.q}
				p := TransformWorldPoint(transform, offset)
				name := FindName(&world.names, body.nameId)
				if name != nil {
					draw.DrawStringFcn(p, name, .colorWhite, draw.context_)
				}
			}

			if draw.drawMass {
				transform := WorldTransform{bodySim.center, bodySim.transform.q}
				draw.DrawTransformFcn(transform, draw.context_)

				if body.type == .dynamicBody {
					offset := Vec3{0.05, 0.05, 0.05}
					p := TransformWorldPoint(transform, offset)
					buffer: [32]u8
					text := snprintf(buffer[:], "%.2f", body.mass)
					draw.DrawStringFcn(p, text, .colorWhite, draw.context_)
				}
			}

			if draw.drawSleep {
				bodyState := GetBodyState(world, body)

				if bodyState != nil {
					colors := [4]HexColor{.colorBlue, .colorSkyBlue, .colorOrange, .colorRed}

					color := HexColor.colorBlack
					if body.sleepThreshold > 0.0 {
						ratio := body.sleepVelocity / body.sleepThreshold
						index := ClampInt(i32(ratio), 0, 3)
						color = colors[index]
					}

					center := bodySim.center
					draw.DrawPointFcn(center, 10.0, color, draw.context_)

					offset := Vec3{0.1, 0.1, 0.1}
					p := OffsetPos(center, offset)

					buffer: [32]u8
					text := snprintf(buffer[:], "  %.3f", body.sleepVelocity)
					draw.DrawStringFcn(p, text, color, draw.context_)
				}
			}

			if draw.drawJoints {
				jointKey := body.headJointKey
				for jointKey != NULL_INDEX {
					jointId := jointKey >> 1
					edgeIndex := jointKey & 1
					joint := Array_Get(&world.joints, jointId)

					// avoid double draw
					if GetBit(&world.debugJointSet, u32(jointId)) == false {
						DrawJoint(draw, world, joint)
						SetBit(&world.debugJointSet, u32(jointId))
					}

					jointKey = joint.edges[edgeIndex].nextKey
				}
			}

			speculativeDistance := SPECULATIVE_DISTANCE
			if draw.drawContacts && body.type == .dynamicBody && body.setIndex == i32(SetType.awakeSet) {
				contactKey := body.headContactKey
				for contactKey != NULL_INDEX {
					contactId := contactKey >> 1
					edgeIndex := contactKey & 1
					contact := Array_Get(&world.contacts, contactId)
					contactKey = contact.edges[edgeIndex].nextKey

					if contact.setIndex != i32(SetType.awakeSet) || contact.colorIndex == NULL_INDEX {
						continue
					}

					// avoid double draw
					if GetBit(&world.debugContactSet, u32(contactId)) == false {
						bodyA := Array_Get(&world.bodies, contact.edges[0].bodyId)
						bodySimA := GetBodySim(world, bodyA)
						bodyB := Array_Get(&world.bodies, contact.edges[1].bodyId)
						bodySimB := GetBodySim(world, bodyB)

						for manifoldIndex: i32 = 0; manifoldIndex < contact.manifoldCount; manifoldIndex += 1 {
							manifold := &contact.manifolds[manifoldIndex]
							assert(manifold.pointCount > 0)

							normal := manifold.normal

							// Average the anchors not the world points so the friction center stays exact far from the origin
							contactCenter := draw.drawAnchorA ? bodySimA.center : bodySimB.center
							frictionAnchor := Vec3_zero
							totalWeight: f32 = 0.0
							invTau := 1.0 / SPECULATIVE_DISTANCE

							points := &manifold.points
							for pointIndex: i32 = 0; pointIndex < manifold.pointCount; pointIndex += 1 {
								mp := &points[pointIndex]

								buffer: [32]u8

								anchor := draw.drawAnchorA ? mp.anchorA : mp.anchorB
								p := OffsetPos(contactCenter, anchor)

								// See similar friction anchor weights in b3PrepareContacts_Mesh.
								weight := ClampFloat(2.0 - mp.separation * invTau, MIN_FRICTION_WEIGHT, 1.0)
								frictionAnchor = MulAdd(frictionAnchor, weight, anchor)
								totalWeight += weight

								if draw.drawContactNormals {
									p1 := p
									p2 := OffsetPos(p1, MulSV(axisScale, normal))
									draw.DrawSegmentFcn(p1, p2, .colorLightGray, draw.context_)

									text := snprintf(buffer[:], "   %.2f", 100.0 * mp.separation)
									draw.DrawStringFcn(p, text, .colorWhite, draw.context_)
								} else if draw.drawContactForces {
									// Hack inv_dt for single step debugging
									inv_dt: f32 = world.inv_dt > 0.0 ? world.inv_dt : 60.0
									force := mp.totalNormalImpulse * inv_dt
									p1 := p
									p2 := OffsetPos(p1, MulSV(draw.forceScale * force, normal))
									draw.DrawSegmentFcn(p1, p2, impulseColor, draw.context_)
									text := snprintf(buffer[:], "  %.1f", force)
									draw.DrawStringFcn(p1, text, .colorWhite, draw.context_)
								} else if draw.drawContactFeatures {
									text := snprintf(buffer[:], "   %#X", mp.featureId)
									draw.DrawStringFcn(p, text, .colorWhite, draw.context_)
								}

								if draw.drawGraphColors {
									// graph color
									pointSize: f32 = contact.colorIndex == OVERFLOW_INDEX ? 15.0 : 10.0
									draw.DrawPointFcn(p, pointSize, graphColors[contact.colorIndex], draw.context_)
									// g_draw.DrawString(point->position, "%d", point->color);
								} else if mp.persisted == false {
									// Add
									draw.DrawPointFcn(p, 20.0, addColor, draw.context_)
								} else if mp.separation > speculativeDistance {
									// Speculative
									draw.DrawPointFcn(p, 10.0, farColor, draw.context_)
								} else if mp.separation > 0.0 {
									// Speculative
									draw.DrawPointFcn(p, 10.0, speculativeColor, draw.context_)
								} else {
									// Persist
									draw.DrawPointFcn(p, 10.0, persistColor, draw.context_)
								}
							}

							if draw.drawContactForces {
								// Hack inv_dt for single step debugging
								inv_dt: f32 = world.inv_dt > 0.0 ? world.inv_dt : 60.0
								frictionAnchor = MulSV(1.0 / totalWeight, frictionAnchor)
								p1 := OffsetPos(contactCenter, frictionAnchor)
								frictionForce := MulSV(0.5 * inv_dt, manifold.frictionImpulse)
								p2 := OffsetPos(p1, MulSV(draw.forceScale, frictionForce))
								draw.DrawSegmentFcn(p1, p2, frictionColor, draw.context_)
								draw.DrawPointFcn(p1, 5.0, frictionColor, draw.context_)

								p1 = OffsetPos(p1, MulSV(0.05 * lengthScale, normal))
								buffer: [32]u8
								text := snprintf(buffer[:], "%.2f", Length(frictionForce))
								draw.DrawStringFcn(p1, text, .colorWhite, draw.context_)
							}
						}

						SetBit(&world.debugContactSet, u32(contactId))
					}

					contactKey = contact.edges[edgeIndex].nextKey
				}
			}

			if draw.drawIslands {
				islandId := body.islandId
				if islandId != NULL_INDEX && GetBit(&world.debugIslandSet, u32(islandId)) == false {
					island := &world.islands.data[islandId]
					if island.setIndex == NULL_INDEX {
						continue
					}

					shapeCount: i32 = 0
					aabb := AABB {
						lowerBound = {FLT_MAX, FLT_MAX, FLT_MAX},
						upperBound = {-FLT_MAX, -FLT_MAX, -FLT_MAX},
					}

					for b: i32 = 0; b < island.bodies.count; b += 1 {
						islandBodyId := island.bodies.data[b]
						islandBody := Array_Get(&world.bodies, islandBodyId)
						shapeId := islandBody.headShapeId
						for shapeId != NULL_INDEX {
							shape := Array_Get(&world.shapes, shapeId)
							aabb = AABB_Union(aabb, world.fatAABBs.data[shapeId])
							shapeCount += 1
							shapeId = shape.nextShapeId
						}
					}

					if shapeCount > 0 {
						draw.DrawBoundsFcn(aabb, .colorOrangeRed, draw.context_)
					}

					SetBit(&world.debugIslandSet, u32(islandId))
				}
			}

			// Clear the smallest set bit
			word = word & (word - 1)
		}
	}

	buffer: [32]u8 = {}
	lengthUnits := GetLengthUnitsPerMeter()
	offset := Vec3{0.002 * lengthUnits, 0.002 * lengthUnits, 0.002 * lengthUnits}
	for i: i32 = 0; i < world.workerCount; i += 1 {
		pointCount := world.taskContexts.data[i].pointCount
		points := &world.taskContexts.data[i].points
		for j: i32 = 0; j < pointCount; j += 1 {
			point := &points[j]
			p := point.p
			draw.DrawPointFcn(p, 5.0, point.color, draw.context_)
			text := snprintf(buffer[:], "   %d, %.2f", point.label, point.value)
			ps := OffsetPos(p, offset)
			draw.DrawStringFcn(ps, text, point.color, draw.context_)
		}

		lineCount := world.taskContexts.data[i].lineCount
		lines := &world.taskContexts.data[i].lines
		for j: i32 = 0; j < lineCount; j += 1 {
			line := &lines[j]
			p1 := line.p1
			p2 := line.p2
			draw.DrawSegmentFcn(p1, p2, line.color, draw.context_)
			draw.DrawPointFcn(p1, 10.0, line.color, draw.context_)
			text := snprintf(buffer[:], "%d", line.label)
			ps := OffsetPos(p1, offset)
			draw.DrawStringFcn(ps, text, line.color, draw.context_)
		}
	}
}

World_GetBounds :: proc(worldId: WorldId) -> AABB {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return AABB{}
	}

	worldBounds := AABB{}
	haveBounds := false

	for i: i32 = 0; i < i32(BodyType.bodyTypeCount); i += 1 {
		tree := &world.broadPhase.trees[i]
		if DynamicTree_GetProxyCount(tree) == 0 {
			continue
		}

		bounds := DynamicTree_GetRootBounds(&world.broadPhase.trees[i])

		if haveBounds {
			worldBounds = AABB_Union(worldBounds, bounds)
		} else {
			worldBounds = bounds
			haveBounds = true
		}
	}

	return worldBounds
}

World_GetBodyEvents :: proc(worldId: WorldId) -> BodyEvents {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return BodyEvents{}
	}

	count := world.bodyMoveEvents.count
	events := BodyEvents{world.bodyMoveEvents.data, count}
	return events
}

World_GetSensorEvents :: proc(worldId: WorldId) -> SensorEvents {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return SensorEvents{}
	}

	// Careful to use previous buffer
	endEventArrayIndex := 1 - world.endEventArrayIndex

	beginCount := world.sensorBeginEvents.count
	endCount := world.sensorEndEvents[endEventArrayIndex].count

	events := SensorEvents {
		beginEvents = world.sensorBeginEvents.data,
		endEvents   = world.sensorEndEvents[endEventArrayIndex].data,
		beginCount  = beginCount,
		endCount    = endCount,
	}
	return events
}

World_GetContactEvents :: proc(worldId: WorldId) -> ContactEvents {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return ContactEvents{}
	}

	// Careful to use previous buffer
	endEventArrayIndex := 1 - world.endEventArrayIndex

	beginCount := world.contactBeginEvents.count
	endCount := world.contactEndEvents[endEventArrayIndex].count
	hitCount := world.contactHitEvents.count

	events := ContactEvents {
		beginEvents = world.contactBeginEvents.data,
		endEvents   = world.contactEndEvents[endEventArrayIndex].data,
		hitEvents   = world.contactHitEvents.data,
		beginCount  = beginCount,
		endCount    = endCount,
		hitCount    = hitCount,
	}

	return events
}

World_GetJointEvents :: proc(worldId: WorldId) -> JointEvents {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return JointEvents{}
	}

	count := world.jointEvents.count
	events := JointEvents{world.jointEvents.data, count}
	return events
}

World_IsValid :: proc(id: WorldId) -> bool {
	if id.index1 < 1 || MAX_WORLDS < id.index1 {
		return false
	}

	world := &worlds[id.index1 - 1]

	if world.worldId != id.index1 - 1 {
		// world is not allocated
		return false
	}

	return id.generation == world.generation
}

Body_IsValid :: proc(id: BodyId) -> bool {
	if MAX_WORLDS <= id.world0 {
		// invalid world
		return false
	}

	world := &worlds[id.world0]
	if world.worldId != id.world0 {
		// world is free
		return false
	}

	if id.index1 < 1 || world.bodies.count < id.index1 {
		// invalid index
		return false
	}

	body := &world.bodies.data[id.index1 - 1]
	if body.setIndex == NULL_INDEX {
		// this was freed
		return false
	}

	assert(body.localIndex != NULL_INDEX)

	if body.generation != id.generation {
		// this id is orphaned
		return false
	}

	return true
}

Shape_IsValid :: proc(id: ShapeId) -> bool {
	if MAX_WORLDS <= id.world0 {
		return false
	}

	world := &worlds[id.world0]
	if world.worldId != id.world0 {
		// world is free
		return false
	}

	shapeId := id.index1 - 1
	if shapeId < 0 || world.shapes.count <= shapeId {
		return false
	}

	shape := &world.shapes.data[shapeId]
	if shape.id == NULL_INDEX {
		// shape is free
		return false
	}

	assert(shape.id == shapeId)

	return id.generation == shape.generation
}

Joint_IsValid :: proc(id: JointId) -> bool {
	if MAX_WORLDS <= id.world0 {
		return false
	}

	world := &worlds[id.world0]
	if world.worldId != id.world0 {
		// world is free
		return false
	}

	jointId := id.index1 - 1
	if jointId < 0 || world.joints.count <= jointId {
		return false
	}

	joint := &world.joints.data[jointId]
	if joint.jointId == NULL_INDEX {
		// joint is free
		return false
	}

	assert(joint.jointId == jointId)

	return id.generation == joint.generation
}

Contact_IsValid :: proc(id: ContactId) -> bool {
	if MAX_WORLDS <= id.world0 {
		return false
	}

	world := &worlds[id.world0]
	if world.worldId != id.world0 {
		// world is free
		return false
	}

	contactId := id.index1 - 1
	if contactId < 0 || world.contacts.count <= contactId {
		return false
	}

	contact := &world.contacts.data[contactId]
	if contact.contactId == NULL_INDEX {
		// contact is free
		return false
	}

	assert(contact.contactId == contactId)

	return id.generation == contact.generation
}

World_EnableSleeping :: proc(worldId: WorldId, flag: bool) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if flag == world.enableSleep {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_WorldEnableSleeping{worldId, flag}
		RecWrite_WorldEnableSleeping(world.recording, &recArgs)
	}

	world.enableSleep = flag

	if flag == false {
		setCount := world.solverSets.count
		for i := i32(SetType.firstSleepingSet); i < setCount; i += 1 {
			set := Array_Get(&world.solverSets, i)
			if set.bodySims.count > 0 {
				WakeSolverSet(world, i)
			}
		}
	}
}

World_IsSleepingEnabled :: proc(worldId: WorldId) -> bool {
	world := GetWorldFromId(worldId)
	return world.enableSleep
}

World_EnableWarmStarting :: proc(worldId: WorldId, flag: bool) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_WorldEnableWarmStarting{worldId, flag}
		RecWrite_WorldEnableWarmStarting(world.recording, &recArgs)
	}

	world.enableWarmStarting = flag
}

World_IsWarmStartingEnabled :: proc(worldId: WorldId) -> bool {
	world := GetWorldFromId(worldId)
	return world.enableWarmStarting
}

World_EnableSSE2Fallback :: proc(worldId: WorldId, flag: bool) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	world.simdWidth = flag ? 4 : GetSIMDWidth()
}

World_GetAwakeBodyCount :: proc(worldId: WorldId) -> i32 {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return 0
	}
	awakeSet := Array_Get(&world.solverSets, i32(SetType.awakeSet))
	return awakeSet.bodySims.count
}

World_EnableContinuous :: proc(worldId: WorldId, flag: bool) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_WorldEnableContinuous{worldId, flag}
		RecWrite_WorldEnableContinuous(world.recording, &recArgs)
	}

	world.enableContinuous = flag
}

World_IsContinuousEnabled :: proc(worldId: WorldId) -> bool {
	world := GetWorldFromId(worldId)
	return world.enableContinuous
}

World_SetRestitutionThreshold :: proc(worldId: WorldId, value: f32) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_WorldSetRestitutionThreshold{worldId, value}
		RecWrite_WorldSetRestitutionThreshold(world.recording, &recArgs)
	}

	world.restitutionThreshold = ClampFloat(value, 0.0, FLT_MAX)
}

World_GetRestitutionThreshold :: proc(worldId: WorldId) -> f32 {
	world := GetWorldFromId(worldId)
	return world.restitutionThreshold
}

World_SetRestitutionIterations :: proc(worldId: WorldId, iterations: i32) {
	iterations := iterations
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	iterations = ClampInt(iterations, 0, MAX_RESTITUTION_ITERATIONS)
	if iterations == world.restitutionIterations {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_WorldSetRestitutionIterations{worldId, iterations}
		RecWrite_WorldSetRestitutionIterations(world.recording, &recArgs)
	}

	world.restitutionIterations = iterations
}

World_GetRestitutionIterations :: proc(worldId: WorldId) -> i32 {
	world := GetWorldFromId(worldId)
	return world.restitutionIterations
}

World_EnableRestitutionPropagation :: proc(worldId: WorldId, flag: bool) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if flag == world.enableRestitutionPropagation {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_WorldEnableRestitutionPropagation{worldId, flag}
		RecWrite_WorldEnableRestitutionPropagation(world.recording, &recArgs)
	}

	world.enableRestitutionPropagation = flag
}

World_IsRestitutionPropagationEnabled :: proc(worldId: WorldId) -> bool {
	world := GetWorldFromId(worldId)
	return world.enableRestitutionPropagation
}

World_SetHitEventThreshold :: proc(worldId: WorldId, value: f32) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_WorldSetHitEventThreshold{worldId, value}
		RecWrite_WorldSetHitEventThreshold(world.recording, &recArgs)
	}

	world.hitEventThreshold = ClampFloat(value, 0.0, FLT_MAX)
}

World_GetHitEventThreshold :: proc(worldId: WorldId) -> f32 {
	world := GetWorldFromId(worldId)
	return world.hitEventThreshold
}

World_SetContactTuning :: proc(worldId: WorldId, hertz: f32, dampingRatio: f32, contactSpeed: f32) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_WorldSetContactTuning{worldId, hertz, dampingRatio, contactSpeed}
		RecWrite_WorldSetContactTuning(world.recording, &recArgs)
	}

	world.contactHertz = ClampFloat(hertz, 0.0, FLT_MAX)
	world.contactDampingRatio = ClampFloat(dampingRatio, 0.0, FLT_MAX)
	world.contactSpeed = ClampFloat(contactSpeed, 0.0, FLT_MAX)
}

World_SetContactRecycleDistance :: proc(worldId: WorldId, recycleDistance: f32) {
	world := GetWorldFromId(worldId)
	assert(world.locked == false)
	if world.locked {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_WorldSetContactRecycleDistance{worldId, recycleDistance}
		RecWrite_WorldSetContactRecycleDistance(world.recording, &recArgs)
	}

	world.contactRecycleDistance = ClampFloat(recycleDistance, 0.0, FLT_MAX)
}

World_GetContactRecycleDistance :: proc(worldId: WorldId) -> f32 {
	world := GetWorldFromId(worldId)
	return world.contactRecycleDistance
}

World_SetMaximumLinearSpeed :: proc(worldId: WorldId, maximumLinearSpeed: f32) {
	assert(IsValidFloat(maximumLinearSpeed) && maximumLinearSpeed > 0.0)

	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_WorldSetMaximumLinearSpeed{worldId, maximumLinearSpeed}
		RecWrite_WorldSetMaximumLinearSpeed(world.recording, &recArgs)
	}

	world.maxLinearSpeed = maximumLinearSpeed
}

World_GetMaximumLinearSpeed :: proc(worldId: WorldId) -> f32 {
	world := GetWorldFromId(worldId)
	return world.maxLinearSpeed
}

World_SetIsotropicInertiaTolerance :: proc(worldId: WorldId, tolerance: f32) {
	assert(IsValidFloat(tolerance))

	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_WorldSetIsotropicInertiaTolerance{worldId, tolerance}
		RecWrite_WorldSetIsotropicInertiaTolerance(world.recording, &recArgs)
	}

	world.isotropicInertiaTolerance = ClampFloat(tolerance, 0.0, 1.0)
}

World_GetIsotropicInertiaTolerance :: proc(worldId: WorldId) -> f32 {
	world := GetWorldFromId(worldId)
	return world.isotropicInertiaTolerance
}

World_GetProfile :: proc(worldId: WorldId) -> Profile {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return Profile{}
	}
	return world.profile
}

World_GetMaxCapacity :: proc(worldId: WorldId) -> Capacity {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return Capacity{}
	}
	return world.maxCapacity
}

World_SetUserData :: proc(worldId: WorldId, userData: rawptr) {
	world := GetWorldFromId(worldId)
	world.userData = userData
}

World_GetUserData :: proc(worldId: WorldId) -> rawptr {
	world := GetWorldFromId(worldId)
	return world.userData
}

World_SetFrictionCallback :: proc(worldId: WorldId, callback: FrictionCallback) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	world.frictionCallback = callback != nil ? callback : DefaultFrictionCallback
}

World_SetRestitutionCallback :: proc(worldId: WorldId, callback: RestitutionCallback) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	world.restitutionCallback = callback != nil ? callback : DefaultRestitutionCallback
}

World_SetWorkerCount :: proc(worldId: WorldId, count: i32) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if count == world.workerCount {
		return
	}

	DestroyWorkerContexts(world)
	world.workerCount = ClampInt(count, 1, MAX_WORKERS)
	CreateWorkerContexts(world)
}

World_GetWorkerCount :: proc(worldId: WorldId) -> i32 {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return 0
	}

	return world.workerCount
}

World_StartRecording :: proc(worldId: WorldId, recording: ^Recording) {
	// Must be a step boundary, so refuse a locked world
	world := GetUnlockedWorldFromId(worldId)

	if world == nil || recording == nil || world.recording != nil {
		return
	}

	StartRecordingIntoBuffer(world, recording)
}

World_StopRecording :: proc(worldId: WorldId) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	StopRecordingInternal(world)
}

@(private = "file")
GetWorldByteCount :: proc(world: ^World, log: bool) -> u64 {
	total: u64 = 0

	// id pools
	bodyIdBytes := GetIdBytes(&world.bodyIdPool)
	solverSetIdBytes := GetIdBytes(&world.solverSetIdPool)
	jointIdBytes := GetIdBytes(&world.jointIdPool)
	contactIdBytes := GetIdBytes(&world.contactIdPool)
	islandIdBytes := GetIdBytes(&world.islandIdPool)
	shapeIdBytes := GetIdBytes(&world.shapeIdPool)
	total +=
		u64(bodyIdBytes) +
		u64(solverSetIdBytes) +
		u64(jointIdBytes) +
		u64(contactIdBytes) +
		u64(islandIdBytes) +
		u64(shapeIdBytes)

	if log {
		Log("id pools")
		Log("body ids: %d", bodyIdBytes)
		Log("solver set ids: %d", solverSetIdBytes)
		Log("joint ids: %d", jointIdBytes)
		Log("contact ids: %d", contactIdBytes)
		Log("island ids: %d", islandIdBytes)
		Log("shape ids: %d", shapeIdBytes)
	}

	// Islands own per-island body/contact/joint link arrays
	islandLinkBytes: i32 = 0
	for i: i32 = 0; i < world.islands.count; i += 1 {
		island := &world.islands.data[i]
		islandLinkBytes += Array_ByteCount(&island.bodies)
		islandLinkBytes += Array_ByteCount(&island.contacts)
		islandLinkBytes += Array_ByteCount(&island.joints)
	}

	// world arrays
	bodyArrayBytes := Array_ByteCount(&world.bodies)
	solverSetArrayBytes := Array_ByteCount(&world.solverSets)
	jointArrayBytes := Array_ByteCount(&world.joints)
	contactArrayBytes := Array_ByteCount(&world.contacts)
	islandArrayBytes := Array_ByteCount(&world.islands)
	shapeArrayBytes := Array_ByteCount(&world.shapes)
	sensorArrayBytes := Array_ByteCount(&world.sensors)
	total +=
		u64(bodyArrayBytes) +
		u64(solverSetArrayBytes) +
		u64(jointArrayBytes) +
		u64(contactArrayBytes) +
		u64(islandArrayBytes) +
		u64(islandLinkBytes) +
		u64(shapeArrayBytes) +
		u64(sensorArrayBytes)

	if log {
		Log("world arrays")
		Log("bodies: %d", bodyArrayBytes)
		Log("solver sets: %d", solverSetArrayBytes)
		Log("joints: %d", jointArrayBytes)
		Log("contacts: %d", contactArrayBytes)
		Log("islands: %d", islandArrayBytes)
		Log("island links: %d", islandLinkBytes)
		Log("shapes: %d", shapeArrayBytes)
		Log("sensors: %d", sensorArrayBytes)
	}

	// Sensors own overlap tracking arrays. The sensor array is dense.
	sensorOverlapBytes: i32 = 0
	for i: i32 = 0; i < world.sensors.count; i += 1 {
		sensor := &world.sensors.data[i]
		sensorOverlapBytes += Array_ByteCount(&sensor.hits)
		sensorOverlapBytes += Array_ByteCount(&sensor.overlaps1)
		sensorOverlapBytes += Array_ByteCount(&sensor.overlaps2)
	}
	total += u64(sensorOverlapBytes)

	if log {
		Log("owned arrays")
		Log("sensor overlaps: %d", sensorOverlapBytes)
	}

	// Shared hull database. The map owns a combined bucket and metadata allocation
	// plus the small map struct. Each stored key is an owned clone sized by byteCount.
	hullDatabase := (^HullMap)(world.hullDatabase)
	hullCount := i32(HullMap_size(hullDatabase))
	hullBucketCount := i32(HullMap_bucket_count(hullDatabase))
	hullMapBytes := u64(HullMapByteCount(hullDatabase))
	hullDataBytes: u64 = 0
	for itr := HullMap_first(hullDatabase); HullMap_is_end(itr) == false; itr = HullMap_next(itr) {
		hullDataBytes += u64(itr.data.key.byteCount)
	}
	total += hullMapBytes + hullDataBytes

	if log {
		Log("hulls")
		Log("database: %d (%d, %d)", i32(hullMapBytes), hullCount, hullBucketCount)
		Log("hull data: %d", i32(hullDataBytes))
	}

	// broad-phase
	staticTreeBytes := DynamicTree_GetByteCount(&world.broadPhase.trees[BodyType.staticBody])
	kinematicTreeBytes := DynamicTree_GetByteCount(&world.broadPhase.trees[BodyType.kinematicBody])
	dynamicTreeBytes := DynamicTree_GetByteCount(&world.broadPhase.trees[BodyType.dynamicBody])
	pairSet := &world.broadPhase.pairSet
	pairSetBytes := GetHashSetBytes(pairSet)
	total += u64(staticTreeBytes) + u64(kinematicTreeBytes) + u64(dynamicTreeBytes) + u64(pairSetBytes)

	if log {
		Log("broad-phase")
		Log("static tree: %d", staticTreeBytes)
		Log("kinematic tree: %d", kinematicTreeBytes)
		Log("dynamic tree: %d", dynamicTreeBytes)
		Log("pairSet: %d (%d, %d)", pairSetBytes, pairSet.count, pairSet.capacity)
	}

	// Manifold block allocators, one per manifold point count
	manifoldArrayBytes := Array_ByteCount(&world.manifoldAllocators)
	manifoldBlockBytes: i32 = 0
	for i: i32 = 0; i < world.manifoldAllocators.count; i += 1 {
		allocator := &world.manifoldAllocators.data[i]
		manifoldBlockBytes += Array_ByteCount(&allocator.blocks)
		manifoldBlockBytes += allocator.blocks.count * BLOCK_SIZE * allocator.elementSize
	}
	total += u64(manifoldArrayBytes) + u64(manifoldBlockBytes)

	if log {
		Log("manifold allocators")
		Log("allocator array: %d", manifoldArrayBytes)
		Log("blocks: %d", manifoldBlockBytes)
	}

	// solver sets
	bodySimCapacity: i32 = 0
	bodyStateCapacity: i32 = 0
	jointSimCapacity: i32 = 0
	contactIndexCapacity: i32 = 0
	islandSimCapacity: i32 = 0
	solverSetCapacity := world.solverSets.count
	for i: i32 = 0; i < solverSetCapacity; i += 1 {
		set := &world.solverSets.data[i]
		if set.setIndex == NULL_INDEX {
			continue
		}

		bodySimCapacity += set.bodySims.capacity
		bodyStateCapacity += set.bodyStates.capacity
		jointSimCapacity += set.jointSims.capacity
		contactIndexCapacity += set.contactIndices.capacity
		islandSimCapacity += set.islandSims.capacity
	}

	setBodySimBytes := bodySimCapacity * i32(size_of(BodySim))
	setBodyStateBytes := bodyStateCapacity * i32(size_of(BodyState))
	setJointSimBytes := jointSimCapacity * i32(size_of(JointSim))
	setContactSimBytes := contactIndexCapacity * i32(size_of(i32))
	setIslandSimBytes := islandSimCapacity * i32(size_of(IslandSim))
	total +=
		u64(setBodySimBytes) +
		u64(setBodyStateBytes) +
		u64(setJointSimBytes) +
		u64(setContactSimBytes) +
		u64(setIslandSimBytes)

	if log {
		Log("solver sets")
		Log("body sim: %d", setBodySimBytes)
		Log("body state: %d", setBodyStateBytes)
		Log("joint sim: %d", setJointSimBytes)
		Log("contact sim: %d", setContactSimBytes)
		Log("island sim: %d", setIslandSimBytes)
	}

	// constraint graph
	bodyBitSetBytes: i32 = 0
	graphContactBytes: i32 = 0
	graphJointSimBytes: i32 = 0
	for i := 0; i < GRAPH_COLOR_COUNT; i += 1 {
		c := &world.constraintGraph.colors[i]
		bodyBitSetBytes += GetBitSetBytes(&c.bodySet)
		graphContactBytes += Array_ByteCount(&c.convexContacts) + Array_ByteCount(&c.contacts)
		graphJointSimBytes += Array_ByteCount(&c.jointSims)
	}
	total += u64(bodyBitSetBytes) + u64(graphJointSimBytes) + u64(graphContactBytes)

	if log {
		Log("constraint graph")
		Log("body bit sets: %d", bodyBitSetBytes)
		Log("joint sim: %d", graphJointSimBytes)
		Log("contact sim: %d", graphContactBytes)
	}

	// Per worker task storage and its bit sets
	taskContextBytes := Array_ByteCount(&world.taskContexts)
	for i: i32 = 0; i < world.taskContexts.count; i += 1 {
		taskContext := &world.taskContexts.data[i]
		taskContextBytes += Array_ByteCount(&taskContext.sensorHits)
		taskContextBytes += Array_ByteCount(&taskContext.pairKeys)
		taskContextBytes += GetBitSetBytes(&taskContext.contactStateBitSet)
		taskContextBytes += GetBitSetBytes(&taskContext.jointStateBitSet)
		taskContextBytes += GetBitSetBytes(&taskContext.hitEventBitSet)
		taskContextBytes += GetBitSetBytes(&taskContext.awakeIslandBitSet)
	}

	sensorTaskContextBytes := Array_ByteCount(&world.sensorTaskContexts)
	for i: i32 = 0; i < world.sensorTaskContexts.count; i += 1 {
		taskContext := &world.sensorTaskContexts.data[i]
		sensorTaskContextBytes += GetBitSetBytes(&taskContext.eventBits)
	}
	total += u64(taskContextBytes) + u64(sensorTaskContextBytes)

	if log {
		Log("task contexts")
		Log("worker: %d", taskContextBytes)
		Log("sensor: %d", sensorTaskContextBytes)
	}

	// Double buffered event arrays
	eventBytes: i32 = 0
	eventBytes += Array_ByteCount(&world.bodyMoveEvents)
	eventBytes += Array_ByteCount(&world.sensorBeginEvents)
	eventBytes += Array_ByteCount(&world.contactBeginEvents)
	eventBytes += Array_ByteCount(&world.sensorEndEvents[0])
	eventBytes += Array_ByteCount(&world.sensorEndEvents[1])
	eventBytes += Array_ByteCount(&world.contactEndEvents[0])
	eventBytes += Array_ByteCount(&world.contactEndEvents[1])
	eventBytes += Array_ByteCount(&world.contactHitEvents)
	eventBytes += Array_ByteCount(&world.jointEvents)
	total += u64(eventBytes)

	if log {
		Log("events: %d", eventBytes)
	}

	// Debug draw bit sets
	debugBytes: i32 = 0
	debugBytes += GetBitSetBytes(&world.debugBodySet)
	debugBytes += GetBitSetBytes(&world.debugJointSet)
	debugBytes += GetBitSetBytes(&world.debugContactSet)
	debugBytes += GetBitSetBytes(&world.debugIslandSet)
	total += u64(debugBytes)

	if log {
		Log("debug draw: %d", debugBytes)
	}

	// stack allocator
	total += u64(world.stack.capacity)

	if log {
		Log("stack allocator: %d", world.stack.capacity)
		// unified: %u is %d in Odin's fmt
		Log("total: %d KB", u32(total / 1024))
		// unified end
	}

	return total
}

World_DumpMemoryStats :: proc(worldId: WorldId) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	GetWorldByteCount(world, true)
}

World_GetCounters :: proc(worldId: WorldId) -> Counters {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return Counters{}
	}

	s := Counters{}
	s.bodyCount = GetIdCount(&world.bodyIdPool)
	s.shapeCount = GetIdCount(&world.shapeIdPool)
	s.contactCount = GetIdCount(&world.contactIdPool)
	s.jointCount = GetIdCount(&world.jointIdPool)
	s.islandCount = GetIdCount(&world.islandIdPool)

	staticTree := &world.broadPhase.trees[BodyType.staticBody]
	s.staticTreeHeight = DynamicTree_GetHeight(staticTree)

	dynamicTree := &world.broadPhase.trees[BodyType.dynamicBody]
	kinematicTree := &world.broadPhase.trees[BodyType.kinematicBody]
	s.treeHeight = MaxInt(DynamicTree_GetHeight(dynamicTree), DynamicTree_GetHeight(kinematicTree))

	s.satCallCount = world.satCallCount
	s.satCacheHitCount = world.satCacheHitCount
	mem.copy(&s.manifoldCounts, &world.manifoldCounts, CONTACT_MANIFOLD_COUNT_BUCKETS * size_of(i32))
	s.stackUsed = world.stack.maxAllocation
	s.byteCount = i64(GetWorldByteCount(world, false))
	s.taskCount = world.taskCount

	#assert(GRAPH_COLOR_COUNT == len(s.colorCounts), "colorCounts size mismatch")

	s.awakeContactCount = 0
	for i := 0; i < GRAPH_COLOR_COUNT; i += 1 {
		color := &world.constraintGraph.colors[i]
		colorContactCount := color.convexContacts.count + color.contacts.count
		s.colorCounts[i] = colorContactCount + color.jointSims.count
		s.awakeContactCount += colorContactCount
	}
	s.awakeContactCount += world.solverSets.data[SetType.awakeSet].contactIndices.count

	s.recycledContactCount = 0
	s.arenaCapacity = 0
	s.distanceIterations = 0
	s.pushBackIterations = 0
	s.rootIterations = 0
	for i: i32 = 0; i < world.workerCount; i += 1 {
		s.recycledContactCount += world.taskContexts.data[i].recycledContactCount

		s.distanceIterations = MaxInt(s.distanceIterations, world.taskContexts.data[i].distanceIterations)
		s.pushBackIterations = MaxInt(s.pushBackIterations, world.taskContexts.data[i].pushBackIterations)
		s.rootIterations = MaxInt(s.rootIterations, world.taskContexts.data[i].rootIterations)

		peak := world.taskContexts.data[i].arena.shared.peakDemand
		if peak > s.arenaCapacity {
			s.arenaCapacity = peak
		}
	}

	return s
}

WorldQueryContext :: struct {
	world:       ^World,
	fcn:         OverlapResultFcn,
	filter:      QueryFilter,
	userContext: rawptr,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
TreeQueryCallback :: proc "c" (proxyId: i32, userData: u64, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	_ = proxyId

	shapeId := i32(userData)
	worldContext := (^WorldQueryContext)(context_)
	world := worldContext.world

	shape := Array_Get(&world.shapes, shapeId)

	shapeFilter := shape.filter
	queryFilter := worldContext.filter

	if ShouldQueryCollide(&shapeFilter, &queryFilter) == false {
		return true
	}

	id := ShapeId{shapeId + 1, world.worldId, shape.generation}
	result := worldContext.fcn(id, worldContext.userContext)
	return result
}

World_OverlapAABB :: proc(
	worldId: WorldId,
	aabb: AABB,
	filter: QueryFilter,
	fcn: OverlapResultFcn,
	context_: rawptr,
) -> TreeStats {
	fcn, context_ := fcn, context_
	treeStats := TreeStats{}

	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return treeStats
	}

	assert(IsValidAABB(aabb))

	recWriter := RecQueryWriter{}
	if world.recording != nil {
		RecQueryBegin(&recWriter, context_, filter.id, filter.name)
		recWriter.userFcn.overlapFcn = fcn
		RecW_WORLDID(&recWriter.buf, worldId)
		RecW_AABB(&recWriter.buf, aabb)
		RecW_QUERYFILTER(&recWriter.buf, filter)
		recWriter.countOffset = RecReserveU32(&recWriter.buf)
		fcn = RecOverlapTrampoline
		context_ = &recWriter
	}

	worldContext := WorldQueryContext{world, fcn, filter, context_}

	for i: i32 = 0; i < i32(BodyType.bodyTypeCount); i += 1 {
		treeResult := DynamicTree_Query(
			&world.broadPhase.trees[i],
			aabb,
			filter.maskBits,
			false,
			TreeQueryCallback,
			&worldContext,
		)

		treeStats.nodeVisits += treeResult.nodeVisits
		treeStats.leafVisits += treeResult.leafVisits
	}

	if world.recording != nil {
		RecPatchU32(&recWriter.buf, recWriter.countOffset, recWriter.hitCount)
		RecW_TREESTATS(&recWriter.buf, treeStats)
		RecQueryCommit(world.recording, recOpQueryOverlapAABB, &recWriter)
	}

	return treeStats
}

WorldOverlapContext :: struct {
	world:       ^World,
	fcn:         OverlapResultFcn,
	filter:      QueryFilter,
	proxy:       ShapeProxy,
	origin:      Pos,
	userContext: rawptr,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
TreeOverlapCallback :: proc "c" (proxyId: i32, userData: u64, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	_ = proxyId

	shapeId := i32(userData)
	worldContext := (^WorldOverlapContext)(context_)
	world := worldContext.world

	shape := Array_Get(&world.shapes, shapeId)

	shapeFilter := shape.filter
	queryFilter := worldContext.filter

	if ShouldQueryCollide(&shapeFilter, &queryFilter) == false {
		return true
	}

	// Re-center on the query origin so the overlap test stays in float precision far from the origin
	body := Array_Get(&world.bodies, shape.bodyId)
	transform := ToRelativeTransform(GetBodyTransformQuick(world, body), worldContext.origin)

	overlapping := OverlapShape(shape, transform, &worldContext.proxy)
	if overlapping == false {
		return true
	}

	id := ShapeId{shape.id + 1, world.worldId, shape.generation}
	result := worldContext.fcn(id, worldContext.userContext)
	return result
}

World_OverlapShape :: proc(
	worldId: WorldId,
	origin: Pos,
	proxy: ^ShapeProxy,
	filter: QueryFilter,
	fcn: OverlapResultFcn,
	context_: rawptr,
) -> TreeStats {
	fcn, context_ := fcn, context_
	treeStats := TreeStats{}

	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return treeStats
	}

	assert(IsValidPosition(origin))

	recWriter := RecQueryWriter{}
	if world.recording != nil {
		RecQueryBegin(&recWriter, context_, filter.id, filter.name)
		recWriter.userFcn.overlapFcn = fcn
		RecW_WORLDID(&recWriter.buf, worldId)
		RecW_POSITION(&recWriter.buf, origin)
		RecW_SHAPEPROXY(&recWriter.buf, proxy^)
		RecW_QUERYFILTER(&recWriter.buf, filter)
		recWriter.countOffset = RecReserveU32(&recWriter.buf)
		fcn = RecOverlapTrampoline
		context_ = &recWriter
	}

	// Bound the proxy in origin relative space then lift to a conservative world float box
	aabb := OffsetAABB(MakeAABB(proxy.points, proxy.count, proxy.radius), origin)
	worldContext := WorldOverlapContext{world, fcn, filter, proxy^, origin, context_}

	for i: i32 = 0; i < i32(BodyType.bodyTypeCount); i += 1 {
		treeResult := DynamicTree_Query(
			&world.broadPhase.trees[i],
			aabb,
			filter.maskBits,
			false,
			TreeOverlapCallback,
			&worldContext,
		)

		treeStats.nodeVisits += treeResult.nodeVisits
		treeStats.leafVisits += treeResult.leafVisits
	}

	if world.recording != nil {
		RecPatchU32(&recWriter.buf, recWriter.countOffset, recWriter.hitCount)
		RecW_TREESTATS(&recWriter.buf, treeStats)
		RecQueryCommit(world.recording, recOpQueryOverlapShape, &recWriter)
	}

	return treeStats
}

WorldMoverContext :: struct {
	world:       ^World,
	fcn:         PlaneResultFcn,
	filter:      QueryFilter,
	mover:       Capsule,
	origin:      Pos,
	userContext: rawptr,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
TreeCollideCallback :: proc "c" (proxyId: i32, userData: u64, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	_ = proxyId

	shapeId := i32(userData)
	worldContext := (^WorldMoverContext)(context_)
	world := worldContext.world

	shape := Array_Get(&world.shapes, shapeId)

	shapeFilter := shape.filter
	queryFilter := worldContext.filter

	if ShouldQueryCollide(&shapeFilter, &queryFilter) == false {
		return true
	}

	// Re-center on the query origin, the mover and the resulting planes are origin relative
	body := Array_Get(&world.bodies, shape.bodyId)
	bodyTransform := GetBodyTransformQuick(world, body)
	transform := ToRelativeTransform(bodyTransform, worldContext.origin)

	buffer: [64]PlaneResult
	count := CollideMover(raw_data(&buffer), 64, shape, transform, &worldContext.mover)

	if count > 0 {
		id := ShapeId{shape.id + 1, world.worldId, shape.generation}
		return worldContext.fcn(id, &buffer[0], count, worldContext.userContext)
	}

	return true
}

// It is tempting to use a shape proxy for the mover, but this makes handling deep overlap difficult and the generality may
// not be worth it.
World_CollideMover :: proc(
	worldId: WorldId,
	origin: Pos,
	mover: ^Capsule,
	filter: QueryFilter,
	fcn: PlaneResultFcn,
	context_: rawptr,
) {
	fcn, context_ := fcn, context_
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	assert(IsValidPosition(origin))

	recWriter := RecQueryWriter{}
	if world.recording != nil {
		RecQueryBegin(&recWriter, context_, filter.id, filter.name)
		recWriter.userFcn.planeFcn = fcn
		RecW_WORLDID(&recWriter.buf, worldId)
		RecW_POSITION(&recWriter.buf, origin)
		RecW_CAPSULE(&recWriter.buf, mover^)
		RecW_QUERYFILTER(&recWriter.buf, filter)
		recWriter.countOffset = RecReserveU32(&recWriter.buf)
		fcn = RecPlaneTrampoline
		context_ = &recWriter
	}

	r := Vec3{mover.radius, mover.radius, mover.radius}

	relBox: AABB
	relBox.lowerBound = Sub(Min(mover.center1, mover.center2), r)
	relBox.upperBound = Add(Max(mover.center1, mover.center2), r)
	aabb := OffsetAABB(relBox, origin)

	worldContext := WorldMoverContext{world, fcn, filter, mover^, origin, context_}

	for i: i32 = 0; i < i32(BodyType.bodyTypeCount); i += 1 {
		DynamicTree_Query(&world.broadPhase.trees[i], aabb, filter.maskBits, false, TreeCollideCallback, &worldContext)
	}

	if world.recording != nil {
		RecPatchU32(&recWriter.buf, recWriter.countOffset, recWriter.hitCount)
		RecQueryCommit(world.recording, recOpQueryCollideMover, &recWriter)
	}
}

WorldRayCastContext :: struct {
	world:       ^World,
	fcn:         CastResultFcn,
	filter:      QueryFilter,
	fraction:    f32,
	origin:      Pos,
	userContext: rawptr,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
RayCastCallback :: proc "c" (input: ^RayCastInput, proxyId: i32, userData: u64, context_: rawptr) -> f32 {
	context = runtime.default_context()
	// unified end
	_ = proxyId

	shapeId := i32(userData)
	worldContext := (^WorldRayCastContext)(context_)
	world := worldContext.world

	shape := Array_Get(&world.shapes, shapeId)
	shapeFilter := shape.filter
	queryFilter := worldContext.filter

	if ShouldQueryCollide(&shapeFilter, &queryFilter) == false {
		return input.maxFraction
	}

	body := Array_Get(&world.bodies, shape.bodyId)
	bodyTransform := GetBodyTransformQuick(world, body)
	transform := ToRelativeTransform(bodyTransform, worldContext.origin)

	localInput := input^
	localInput.origin = Vec3_zero
	output := RayCastShape(shape, transform, &localInput)

	if output.hit {
		assert(output.fraction <= input.maxFraction)

		id := ShapeId{shapeId + 1, world.worldId, shape.generation}
		point := OffsetPos(worldContext.origin, output.point)
		materialIndex := ClampInt(output.materialIndex, 0, shape.materialCount - 1)
		userMaterialId := GetShapeMaterials(shape)[materialIndex].userMaterialId

		triangleIndex := output.triangleIndex
		childIndex := output.childIndex
		fraction := worldContext.fcn(
			id,
			point,
			output.normal,
			output.fraction,
			userMaterialId,
			triangleIndex,
			childIndex,
			worldContext.userContext,
		)

		// The user may return -1 to skip this shape
		if 0.0 <= fraction && fraction <= 1.0 {
			worldContext.fraction = fraction
		}

		return fraction
	}

	return input.maxFraction
}

World_CastRay :: proc(
	worldId: WorldId,
	origin: Pos,
	translation: Vec3,
	filter: QueryFilter,
	fcn: CastResultFcn,
	context_: rawptr,
) -> TreeStats {
	fcn, context_ := fcn, context_
	treeStats := TreeStats{}

	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return treeStats
	}

	assert(IsValidPosition(origin))
	assert(IsValidVec3(translation))

	recWriter := RecQueryWriter{}
	if world.recording != nil {
		RecQueryBegin(&recWriter, context_, filter.id, filter.name)
		recWriter.userFcn.castFcn = fcn
		RecW_WORLDID(&recWriter.buf, worldId)
		RecW_POSITION(&recWriter.buf, origin)
		RecW_VEC3(&recWriter.buf, translation)
		RecW_QUERYFILTER(&recWriter.buf, filter)
		recWriter.countOffset = RecReserveU32(&recWriter.buf)
		fcn = RecCastTrampoline
		context_ = &recWriter
	}

	// The tree traverses in float relative to the world origin. Each shape is then re-differenced at
	// full precision against the origin, so a hit stays accurate far from the origin.
	input := RayCastInput{ToVec3(origin), translation, 1.0}

	worldContext := WorldRayCastContext{world, fcn, filter, 1.0, origin, context_}

	for i: i32 = 0; i < i32(BodyType.bodyTypeCount); i += 1 {
		treeResult := DynamicTree_RayCast(
			&world.broadPhase.trees[i],
			&input,
			filter.maskBits,
			false,
			RayCastCallback,
			&worldContext,
		)
		treeStats.nodeVisits += treeResult.nodeVisits
		treeStats.leafVisits += treeResult.leafVisits

		if worldContext.fraction == 0.0 {
			break
		}

		input.maxFraction = worldContext.fraction
	}

	if world.recording != nil {
		RecPatchU32(&recWriter.buf, recWriter.countOffset, recWriter.hitCount)
		RecW_TREESTATS(&recWriter.buf, treeStats)
		RecQueryCommit(world.recording, recOpQueryCastRay, &recWriter)
	}

	return treeStats
}

// This callback finds the closest hit. This is the most common callback used in games.
@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
RayCastClosestFcn :: proc "c" (
	shapeId: ShapeId,
	point: Pos,
	normal: Vec3,
	fraction: f32,
	userMaterialId: u64,
	triangleIndex: i32,
	childIndex: i32,
	context_: rawptr,
) -> f32 {
	context = runtime.default_context()
	// unified end
	// Ignore initial overlap
	if fraction == 0.0 {
		return -1.0
	}

	rayResult := (^RayResult)(context_)
	rayResult.shapeId = shapeId
	rayResult.point = point
	rayResult.normal = normal
	rayResult.fraction = fraction
	rayResult.userMaterialId = userMaterialId
	rayResult.triangleIndex = triangleIndex
	rayResult.childIndex = childIndex
	rayResult.hit = true
	return fraction
}

World_CastRayClosest :: proc(worldId: WorldId, origin: Pos, translation: Vec3, filter: QueryFilter) -> RayResult {
	result := RayResult{}

	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return result
	}

	assert(IsValidPosition(origin))
	assert(IsValidVec3(translation))

	// The tree traverses in float relative to the world origin. Each shape is then re-differenced at
	// full precision against its body, so a hit stays accurate far from the origin.
	input := RayCastInput{ToVec3(origin), translation, 1.0}
	worldContext := WorldRayCastContext {
		world       = world,
		fcn         = RayCastClosestFcn,
		filter      = filter,
		fraction    = 1.0,
		origin      = origin,
		userContext = &result,
	}

	for i: i32 = 0; i < i32(BodyType.bodyTypeCount); i += 1 {
		treeResult := DynamicTree_RayCast(
			&world.broadPhase.trees[i],
			&input,
			filter.maskBits,
			false,
			RayCastCallback,
			&worldContext,
		)
		result.nodeVisits += treeResult.nodeVisits
		result.leafVisits += treeResult.leafVisits

		if worldContext.fraction == 0.0 {
			break
		}

		input.maxFraction = worldContext.fraction
	}

	// Closed query, no user callback: record the inputs and the single result for the replay compare.
	if world.recording != nil {
		recWriter := RecQueryWriter{}
		RecQueryBegin(&recWriter, nil, filter.id, filter.name)
		RecW_WORLDID(&recWriter.buf, worldId)
		RecW_POSITION(&recWriter.buf, origin)
		RecW_VEC3(&recWriter.buf, translation)
		RecW_QUERYFILTER(&recWriter.buf, filter)
		RecW_RAYRESULT(&recWriter.buf, result)
		RecQueryCommit(world.recording, recOpQueryCastRayClosest, &recWriter)
	}

	return result
}

WorldShapeCastContext :: struct {
	world:       ^World,
	fcn:         CastResultFcn,
	filter:      QueryFilter,
	fraction:    f32,
	origin:      Pos,
	// origin relative input
	input:       ShapeCastInput,
	userContext: rawptr,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
ShapeCastCallback :: proc "c" (input: ^BoxCastInput, proxyId: i32, userData: u64, context_: rawptr) -> f32 {
	context = runtime.default_context()
	// unified end
	_ = proxyId

	shapeId := i32(userData)
	worldContext := (^WorldShapeCastContext)(context_)
	world := worldContext.world

	shape := Array_Get(&world.shapes, shapeId)
	shapeFilter := shape.filter
	queryFilter := worldContext.filter

	if ShouldQueryCollide(&shapeFilter, &queryFilter) == false {
		return input.maxFraction
	}

	// Rebuild from the origin relative input, taking only the advancing fraction from the tree.
	// The tree box is world float and would lose the cast far from the origin.
	localInput := worldContext.input
	localInput.maxFraction = input.maxFraction

	// Re-center on the query origin so the per-shape cast stays in float precision far from the origin
	body := Array_Get(&world.bodies, shape.bodyId)
	transform := ToRelativeTransform(GetBodyTransformQuick(world, body), worldContext.origin)

	output := ShapeCastShape(shape, transform, &localInput)

	if output.hit {
		id := ShapeId{shapeId + 1, world.worldId, shape.generation}
		materialIndex := ClampInt(output.materialIndex, 0, shape.materialCount - 1)
		userMaterialId := GetShapeMaterials(shape)[materialIndex].userMaterialId

		triangleIndex := output.triangleIndex
		childIndex := output.childIndex
		fraction := worldContext.fcn(
			id,
			OffsetPos(worldContext.origin, output.point),
			output.normal,
			output.fraction,
			userMaterialId,
			triangleIndex,
			childIndex,
			worldContext.userContext,
		)

		// The user may return -1 to skip this shape
		if 0.0 <= fraction && fraction <= 1.0 {
			worldContext.fraction = fraction
		}

		return fraction
	}

	return input.maxFraction
}

World_CastShape :: proc(
	worldId: WorldId,
	origin: Pos,
	proxy: ^ShapeProxy,
	translation: Vec3,
	filter: QueryFilter,
	fcn: CastResultFcn,
	context_: rawptr,
) -> TreeStats {
	fcn, context_ := fcn, context_
	treeStats := TreeStats{}

	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return treeStats
	}

	assert(IsValidPosition(origin))
	assert(IsValidVec3(translation))

	recWriter := RecQueryWriter{}
	if world.recording != nil {
		RecQueryBegin(&recWriter, context_, filter.id, filter.name)
		recWriter.userFcn.castFcn = fcn
		RecW_WORLDID(&recWriter.buf, worldId)
		RecW_POSITION(&recWriter.buf, origin)
		RecW_SHAPEPROXY(&recWriter.buf, proxy^)
		RecW_VEC3(&recWriter.buf, translation)
		RecW_QUERYFILTER(&recWriter.buf, filter)
		recWriter.countOffset = RecReserveU32(&recWriter.buf)
		fcn = RecCastTrampoline
		context_ = &recWriter
	}

	worldContext := WorldShapeCastContext{}
	worldContext.world = world
	worldContext.fcn = fcn
	worldContext.filter = filter
	worldContext.fraction = 1.0
	worldContext.origin = origin
	worldContext.input.proxy = proxy^
	worldContext.input.translation = translation
	worldContext.input.maxFraction = 1.0
	worldContext.input.canEncroach = false
	worldContext.userContext = context_

	// Bound the proxy in origin relative space then lift to a conservative world float box. The tree
	// node boxes use the same directed rounding, so the swept box never clips a shape far from the
	// origin. Per shape casts re-difference at full precision against the carried origin.
	localBox := MakeAABB(proxy.points, proxy.count, proxy.radius)
	treeInput := BoxCastInput{OffsetAABB(localBox, origin), translation, 1.0}

	for i: i32 = 0; i < i32(BodyType.bodyTypeCount); i += 1 {
		treeResult := DynamicTree_BoxCast(
			&world.broadPhase.trees[i],
			&treeInput,
			filter.maskBits,
			false,
			ShapeCastCallback,
			&worldContext,
		)
		treeStats.nodeVisits += treeResult.nodeVisits
		treeStats.leafVisits += treeResult.leafVisits

		if worldContext.fraction == 0.0 {
			break
		}

		treeInput.maxFraction = worldContext.fraction
	}

	if world.recording != nil {
		RecPatchU32(&recWriter.buf, recWriter.countOffset, recWriter.hitCount)
		RecW_TREESTATS(&recWriter.buf, treeStats)
		RecQueryCommit(world.recording, recOpQueryCastShape, &recWriter)
	}

	return treeStats
}

WorldMoverCastContext :: struct {
	world:       ^World,
	fcn:         MoverFilterFcn,
	filter:      QueryFilter,
	fraction:    f32,
	origin:      Pos,
	// origin relative input
	input:       ShapeCastInput,
	userContext: rawptr,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
MoverCastCallback :: proc "c" (input: ^BoxCastInput, proxyId: i32, userData: u64, context_: rawptr) -> f32 {
	context = runtime.default_context()
	// unified end
	_ = proxyId

	shapeId := i32(userData)
	worldContext := (^WorldMoverCastContext)(context_)
	world := worldContext.world

	shape := Array_Get(&world.shapes, shapeId)
	shapeFilter := shape.filter
	queryFilter := worldContext.filter

	if ShouldQueryCollide(&shapeFilter, &queryFilter) == false {
		return worldContext.fraction
	}

	if worldContext.fcn != nil {
		id := ShapeId{shapeId + 1, world.worldId, shape.generation}
		shouldCollide := worldContext.fcn(id, worldContext.userContext)
		if shouldCollide == false {
			return worldContext.fraction
		}
	}

	// Rebuild from the origin relative input, taking only the advancing fraction from the tree
	localInput := worldContext.input
	localInput.maxFraction = input.maxFraction

	// Re-center on the query origin so the per-shape cast stays in float precision far from the origin
	body := Array_Get(&world.bodies, shape.bodyId)
	transform := ToRelativeTransform(GetBodyTransformQuick(world, body), worldContext.origin)

	output := ShapeCastShape(shape, transform, &localInput)
	if output.fraction == 0.0 {
		// Ignore overlapping shapes
		return worldContext.fraction
	}

	worldContext.fraction = output.fraction
	return output.fraction
}

World_CastMover :: proc(
	worldId: WorldId,
	origin: Pos,
	mover: ^Capsule,
	translation: Vec3,
	filter: QueryFilter,
	fcn: MoverFilterFcn,
	context_: rawptr,
) -> f32 {
	fcn, context_ := fcn, context_
	assert(IsValidPosition(origin))
	assert(IsValidVec3(translation))

	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return 1.0
	}

	// The mover filter is a per-shape bool(shapeId, ctx) decision, the same shape as an overlap
	// callback, so the overlap trampoline captures it. Installing it even when the user passed no
	// filter records an accept-all stream that replays identically.
	recWriter := RecQueryWriter{}
	if world.recording != nil {
		RecQueryBegin(&recWriter, context_, filter.id, filter.name)
		recWriter.userFcn.moverFilterFcn = fcn
		RecW_WORLDID(&recWriter.buf, worldId)
		RecW_POSITION(&recWriter.buf, origin)
		RecW_CAPSULE(&recWriter.buf, mover^)
		RecW_VEC3(&recWriter.buf, translation)
		RecW_QUERYFILTER(&recWriter.buf, filter)
		recWriter.countOffset = RecReserveU32(&recWriter.buf)
		fcn = RecOverlapTrampoline
		context_ = &recWriter
	}

	worldContext := WorldMoverCastContext {
		world       = world,
		fcn         = fcn,
		filter      = filter,
		fraction    = 1.0,
		origin      = origin,
		userContext = context_,
	}
	worldContext.input.proxy = ShapeProxy{([^]Vec3)(&mover.center1), 2, mover.radius}
	worldContext.input.translation = translation
	worldContext.input.maxFraction = 1.0
	worldContext.input.canEncroach = mover.radius > 0.0

	// Bound the capsule in origin relative space then lift to a conservative world float box
	centers := [2]Vec3{mover.center1, mover.center2}
	treeInput := BoxCastInput{OffsetAABB(MakeAABB(raw_data(&centers), 2, mover.radius), origin), translation, 1.0}

	for i: i32 = 0; i < i32(BodyType.bodyTypeCount); i += 1 {
		DynamicTree_BoxCast(
			&world.broadPhase.trees[i],
			&treeInput,
			filter.maskBits,
			false,
			MoverCastCallback,
			&worldContext,
		)

		if worldContext.fraction == 0.0 {
			break
		}

		treeInput.maxFraction = worldContext.fraction
	}

	if world.recording != nil {
		// The mover filter type aliases the overlap trampoline, so the user fcn lands in the same
		// union slot. Backpatch the accept count, then record the returned fraction as the tail.
		RecPatchU32(&recWriter.buf, recWriter.countOffset, recWriter.hitCount)
		RecW_F32(&recWriter.buf, worldContext.fraction)
		RecQueryCommit(world.recording, recOpQueryCastMover, &recWriter)
	}

	return worldContext.fraction
}

World_SetCustomFilterCallback :: proc(worldId: WorldId, fcn: CustomFilterFcn, context_: rawptr) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}
	world.customFilterFcn = fcn
	world.customFilterContext = context_
}

World_SetPreSolveCallback :: proc(worldId: WorldId, fcn: PreSolveFcn, context_: rawptr) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}
	world.preSolveFcn = fcn
	world.preSolveContext = context_
}

World_SetGravity :: proc(worldId: WorldId, gravity: Vec3) {
	world := GetWorldFromId(worldId)

	if world.recording != nil {
		recArgs := RecArgs_WorldSetGravity{worldId, gravity}
		RecWrite_WorldSetGravity(world.recording, &recArgs)
	}

	world.gravity = gravity
}

World_GetGravity :: proc(worldId: WorldId) -> Vec3 {
	world := GetWorldFromId(worldId)
	return world.gravity
}

ExplosionContext :: struct {
	world:          ^World,
	position:       Pos,
	radius:         f32,
	falloff:        f32,
	impulsePerArea: f32,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
ExplosionCallback :: proc "c" (proxyId: i32, userData: u64, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	_ = proxyId

	shapeId := i32(userData)
	explosionContext := (^ExplosionContext)(context_)
	world := explosionContext.world

	shape := Array_Get(&world.shapes, shapeId)
	if shape.explosionScale == 0.0 {
		return true
	}

	body := Array_Get(&world.bodies, shape.bodyId)
	assert(body.type == .dynamicBody)

	xf := GetBodyTransformQuick(world, body)

	// Re-center the explosion into the shape local frame so distance and direction stay precise
	// far from the origin. Everything below runs in that near-origin frame.
	localPosition := InvTransformWorldPoint(xf, explosionContext.position)

	input: DistanceInput
	input.proxyA = MakeShapeProxy(shape)
	input.proxyB = ShapeProxy{([^]Vec3)(&localPosition), 1, 0.0}
	input.transform = Transform_identity
	input.useRadii = true

	cache := SimplexCache{}
	output := ShapeDistance(&input, &cache, nil, 0)

	radius := explosionContext.radius
	falloff := explosionContext.falloff
	if output.distance > radius + falloff {
		return true
	}

	WakeBody(world, body)

	if body.setIndex != i32(SetType.awakeSet) {
		return true
	}

	// Witness point is already in the body local query frame
	closestPoint := output.pointA
	if output.distance == 0.0 {
		closestPoint = GetShapeCentroid(shape)
	}

	direction := Sub(closestPoint, localPosition)
	if LengthSquared(direction) > f32(100.0) * FLT_EPSILON * FLT_EPSILON {
		direction = Normalize(direction)
	} else {
		direction = Vec3{1.0, 0.0, 0.0}
	}

	area := GetShapeProjectedArea(shape, direction)
	scale: f32 = 1.0
	if output.distance > radius && falloff > 0.0 {
		scale = ClampFloat((radius + falloff - output.distance) / falloff, 0.0, 1.0)
	}

	magnitude := explosionContext.impulsePerArea * area * scale * shape.explosionScale
	impulse := MulSV(magnitude, RotateVector(xf.q, direction))

	localIndex := body.localIndex
	set := Array_Get(&world.solverSets, i32(SetType.awakeSet))
	state := Array_Get(&set.bodyStates, localIndex)
	bodySim := Array_Get(&set.bodySims, localIndex)
	state.linearVelocity = MulAdd(state.linearVelocity, bodySim.invMass, impulse)

	// Lever arm from the center of mass to the closest point, rotated to world
	r := RotateVector(xf.q, Sub(closestPoint, bodySim.localCenter))
	state.angularVelocity = Add(state.angularVelocity, MulMV(bodySim.invInertiaWorld, Cross(r, impulse)))

	return true
}

World_Explode :: proc(worldId: WorldId, explosionDef: ^ExplosionDef) {
	maskBits := explosionDef.maskBits
	position := explosionDef.position
	radius := explosionDef.radius
	falloff := explosionDef.falloff
	impulsePerArea := explosionDef.impulsePerArea

	assert(IsValidPosition(position))
	assert(IsValidFloat(radius) && radius >= 0.0)
	assert(IsValidFloat(falloff) && falloff >= 0.0)
	assert(IsValidFloat(impulsePerArea))

	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_WorldExplode{worldId, explosionDef^}
		RecWrite_WorldExplode(world.recording, &recArgs)
	}

	// Locked due to waking
	world.locked = true

	explosionContext := ExplosionContext{world, position, radius, falloff, impulsePerArea}

	// The broad-phase tree is float, so translate a local query box out to world with outward rounding
	extent := radius + falloff
	localBox := AABB{{-extent, -extent, -extent}, {extent, extent, extent}}
	aabb := OffsetAABB(localBox, position)

	DynamicTree_Query(
		&world.broadPhase.trees[BodyType.dynamicBody],
		aabb,
		maskBits,
		false,
		ExplosionCallback,
		&explosionContext,
	)

	world.locked = false
}

World_RebuildStaticTree :: proc(worldId: WorldId) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_WorldRebuildStaticTree{worldId}
		RecWrite_WorldRebuildStaticTree(world.recording, &recArgs)
	}

	staticTree := &world.broadPhase.trees[BodyType.staticBody]
	DynamicTree_Rebuild(staticTree, true)
}

World_EnableSpeculative :: proc(worldId: WorldId, flag: bool) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_WorldEnableSpeculative{worldId, flag}
		RecWrite_WorldEnableSpeculative(world.recording, &recArgs)
	}

	world.enableSpeculative = flag
}

when ENABLE_VALIDATION {

	// This validates island graph connectivity for each body
	ValidateConnectivity :: proc(world: ^World) {
		bodies := world.bodies.data
		bodyCapacity := world.bodies.count

		for bodyIndex: i32 = 0; bodyIndex < bodyCapacity; bodyIndex += 1 {
			body := &bodies[bodyIndex]
			if body.id == NULL_INDEX {
				ValidateFreeId(&world.bodyIdPool, bodyIndex)
				continue
			}

			assert(bodyIndex == body.id)

			// Need to get the root island because islands are not merged until the next time step
			bodyIslandId := body.islandId
			bodySetIndex := body.setIndex

			contactKey := body.headContactKey
			for contactKey != NULL_INDEX {
				contactId := contactKey >> 1
				edgeIndex := contactKey & 1

				contact := Array_Get(&world.contacts, contactId)

				touching := (contact.flags & contactTouchingFlag) != 0
				if touching {
					if bodySetIndex != i32(SetType.staticSet) {
						contactIslandId := contact.islandId
						assert(contactIslandId == bodyIslandId)
					}
				} else {
					assert(contact.islandId == NULL_INDEX)
				}

				contactKey = contact.edges[edgeIndex].nextKey
			}

			jointKey := body.headJointKey
			for jointKey != NULL_INDEX {
				jointId := jointKey >> 1
				edgeIndex := jointKey & 1

				joint := Array_Get(&world.joints, jointId)

				otherEdgeIndex := edgeIndex ~ 1

				otherBody := Array_Get(&world.bodies, joint.edges[otherEdgeIndex].bodyId)

				if bodySetIndex == i32(SetType.disabledSet) || otherBody.setIndex == i32(SetType.disabledSet) {
					assert(joint.islandId == NULL_INDEX)
				} else if bodySetIndex == i32(SetType.staticSet) {
					// Intentional nesting
					if otherBody.setIndex == i32(SetType.staticSet) {
						assert(joint.islandId == NULL_INDEX)
					}
				} else if body.type != .dynamicBody && otherBody.type != .dynamicBody {
					assert(joint.islandId == NULL_INDEX)
				} else {
					jointIslandId := joint.islandId
					assert(jointIslandId == bodyIslandId)
				}

				jointKey = joint.edges[edgeIndex].nextKey
			}
		}
	}

	// Validates solver sets, but not island connectivity
	ValidateSolverSets :: proc(world: ^World) {
		assert(GetIdCapacity(&world.bodyIdPool) == world.bodies.count)
		assert(GetIdCapacity(&world.contactIdPool) == world.contacts.count)
		assert(GetIdCapacity(&world.jointIdPool) == world.joints.count)
		assert(GetIdCapacity(&world.islandIdPool) == world.islands.count)
		assert(GetIdCapacity(&world.solverSetIdPool) == world.solverSets.count)

		// Ensure the encoded body sim indices are correct.
		for contactIndex: i32 = 0; contactIndex < world.contacts.count; contactIndex += 1 {
			contact := &world.contacts.data[contactIndex]
			if contact.contactId == NULL_INDEX {
				continue
			}

			bodyA := Array_Get(&world.bodies, contact.edges[0].bodyId)
			bodyB := Array_Get(&world.bodies, contact.edges[1].bodyId)
			assert(contact.encodedBodySimA == EncodeBodySimIndex(bodyA))
			assert(contact.encodedBodySimB == EncodeBodySimIndex(bodyB))
		}

		activeSetCount: i32 = 0
		totalBodyCount: i32 = 0
		totalJointCount: i32 = 0
		totalContactCount: i32 = 0
		totalIslandCount: i32 = 0

		// Validate all solver sets
		setCount := world.solverSets.count
		for setIndex: i32 = 0; setIndex < setCount; setIndex += 1 {
			set := &world.solverSets.data[setIndex]
			if set.setIndex != NULL_INDEX {
				activeSetCount += 1

				if setIndex == i32(SetType.staticSet) {
					assert(set.contactIndices.count == 0)
					assert(set.islandSims.count == 0)
					assert(set.bodyStates.count == 0)
				} else if setIndex == i32(SetType.disabledSet) {
					assert(set.islandSims.count == 0)
					assert(set.bodyStates.count == 0)
				} else if setIndex == i32(SetType.awakeSet) {
					assert(set.bodySims.count == set.bodyStates.count)
					assert(set.jointSims.count == 0)
				} else {
					assert(set.bodyStates.count == 0)
				}

				// Validate bodies
				{
					bodies := world.bodies.data
					assert(set.bodySims.count >= 0)
					totalBodyCount += set.bodySims.count
					for i: i32 = 0; i < set.bodySims.count; i += 1 {
						bodySim := &set.bodySims.data[i]

						bodyId := bodySim.bodyId
						assert(0 <= bodyId && bodyId < world.bodies.count)
						body := &bodies[bodyId]
						assert(body.setIndex == setIndex)
						assert(body.localIndex == i)

						syncedFlags := body.flags & ~u32(isFast | bodyTransientFlags)
						assert((bodySim.flags & syncedFlags) == syncedFlags)

						bodyState := GetBodyState(world, body)
						if bodyState != nil {
							assert((bodyState.flags & syncedFlags) == syncedFlags)
						}

						if body.type == .dynamicBody {
							assert((body.flags & dynamicFlag) != 0)
						}

						if setIndex == i32(SetType.disabledSet) {
							assert(body.headContactKey == NULL_INDEX)
						}

						// Validate body shapes
						prevShapeId: i32 = NULL_INDEX
						shapeId := body.headShapeId
						for shapeId != NULL_INDEX {
							shape := Array_Get(&world.shapes, shapeId)
							assert(shape.id == shapeId)
							assert(shape.prevShapeId == prevShapeId)

							if setIndex == i32(SetType.disabledSet) {
								assert(shape.proxyKey == NULL_INDEX)
							} else if setIndex == i32(SetType.staticSet) {
								assert(PROXY_TYPE(shape.proxyKey) == .staticBody)
							} else {
								proxyType := PROXY_TYPE(shape.proxyKey)
								assert(proxyType == .kinematicBody || proxyType == .dynamicBody)
							}

							prevShapeId = shapeId
							shapeId = shape.nextShapeId
						}

						// Validate body contacts
						contactKey := body.headContactKey
						for contactKey != NULL_INDEX {
							contactId := contactKey >> 1
							edgeIndex := contactKey & 1

							contact := Array_Get(&world.contacts, contactId)
							assert(contact.setIndex != i32(SetType.staticSet))
							assert(contact.edges[0].bodyId == bodyId || contact.edges[1].bodyId == bodyId)
							contactKey = contact.edges[edgeIndex].nextKey
						}

						// Validate body joints
						jointKey := body.headJointKey
						for jointKey != NULL_INDEX {
							jointId := jointKey >> 1
							edgeIndex := jointKey & 1

							joint := Array_Get(&world.joints, jointId)

							otherEdgeIndex := edgeIndex ~ 1

							otherBody := Array_Get(&world.bodies, joint.edges[otherEdgeIndex].bodyId)

							if setIndex == i32(SetType.disabledSet) || otherBody.setIndex == i32(SetType.disabledSet) {
								assert(joint.setIndex == i32(SetType.disabledSet))
							} else if setIndex == i32(SetType.staticSet) &&
							   otherBody.setIndex == i32(SetType.staticSet) {
								assert(joint.setIndex == i32(SetType.staticSet))
							} else if body.type != .dynamicBody && otherBody.type != .dynamicBody {
								assert(joint.setIndex == i32(SetType.staticSet))
							} else if setIndex == i32(SetType.awakeSet) {
								assert(joint.setIndex == i32(SetType.awakeSet))
							} else if setIndex >= i32(SetType.firstSleepingSet) {
								assert(joint.setIndex == setIndex)
							}

							jointSim := GetJointSim(world, joint)
							assert(jointSim.jointId == jointId)
							assert(jointSim.bodyIdA == joint.edges[0].bodyId)
							assert(jointSim.bodyIdB == joint.edges[1].bodyId)

							jointKey = joint.edges[edgeIndex].nextKey
						}
					}
				}

				// Validate contacts
				{
					assert(set.contactIndices.count >= 0)
					totalContactCount += set.contactIndices.count
					for i: i32 = 0; i < set.contactIndices.count; i += 1 {
						contactIndex := set.contactIndices.data[i]
						contact := Array_Get(&world.contacts, contactIndex)
						if setIndex == i32(SetType.awakeSet) {
							// contact should be non-touching if awake
							// or it could be this contact hasn't been transferred yet
							assert(contact.manifoldCount == 0 || (contact.flags & simStartedTouching) != 0)
						}
						assert(contact.setIndex == setIndex)
						assert(contact.colorIndex == NULL_INDEX)
						assert(contact.localIndex == i)
					}
				}

				// Validate joints
				{
					assert(set.jointSims.count >= 0)
					totalJointCount += set.jointSims.count
					for i: i32 = 0; i < set.jointSims.count; i += 1 {
						jointSim := &set.jointSims.data[i]
						joint := Array_Get(&world.joints, jointSim.jointId)
						assert(joint.setIndex == setIndex)
						assert(joint.colorIndex == NULL_INDEX)
						assert(joint.localIndex == i)
					}
				}

				// Validate islands
				{
					assert(set.islandSims.count >= 0)
					totalIslandCount += set.islandSims.count
					for i: i32 = 0; i < set.islandSims.count; i += 1 {
						islandSim := &set.islandSims.data[i]
						island := Array_Get(&world.islands, islandSim.islandId)
						assert(island.setIndex == setIndex)
						assert(island.localIndex == i)
					}
				}
			} else {
				assert(set.bodySims.count == 0)
				assert(set.contactIndices.count == 0)
				assert(set.jointSims.count == 0)
				assert(set.islandSims.count == 0)
				assert(set.bodyStates.count == 0)
			}
		}

		setIdCount := GetIdCount(&world.solverSetIdPool)
		assert(activeSetCount == setIdCount)

		bodyIdCount := GetIdCount(&world.bodyIdPool)
		assert(totalBodyCount == bodyIdCount)

		islandIdCount := GetIdCount(&world.islandIdPool)
		assert(totalIslandCount == islandIdCount)

		// Validate constraint graph
		for colorIndex: i32 = 0; colorIndex < GRAPH_COLOR_COUNT; colorIndex += 1 {
			color := &world.constraintGraph.colors[colorIndex]
			bitCount: i32 = 0

			assert(color.convexContacts.count >= 0)
			totalContactCount += color.convexContacts.count
			for i: i32 = 0; i < color.convexContacts.count; i += 1 {
				contactId := color.convexContacts.data[i]
				contact := Array_Get(&world.contacts, contactId)
				// contact should be touching in the constraint graph or awaiting transfer to non-touching
				assert(contact.manifoldCount > 0 || (contact.flags & (simStoppedTouching | simDisjoint)) != 0)
				assert(contact.setIndex == i32(SetType.awakeSet))
				assert(contact.colorIndex == colorIndex)
				assert(contact.localIndex == i)

				bodyIdA := contact.edges[0].bodyId
				bodyIdB := contact.edges[1].bodyId

				if colorIndex < OVERFLOW_INDEX {
					bodyA := Array_Get(&world.bodies, bodyIdA)
					bodyB := Array_Get(&world.bodies, bodyIdB)
					assert(GetBit(&color.bodySet, u32(bodyIdA)) == (bodyA.type == .dynamicBody))
					assert(GetBit(&color.bodySet, u32(bodyIdB)) == (bodyB.type == .dynamicBody))

					bitCount += bodyA.type == .dynamicBody ? 1 : 0
					bitCount += bodyB.type == .dynamicBody ? 1 : 0
				}
			}

			totalContactCount += color.contacts.count
			for i: i32 = 0; i < color.contacts.count; i += 1 {
				contactId := color.contacts.data[i].contactId
				contact := Array_Get(&world.contacts, contactId)
				// contact should be touching in the constraint graph or awaiting transfer to non-touching
				assert(contact.manifoldCount > 0 || (contact.flags & (simStoppedTouching | simDisjoint)) != 0)
				assert(contact.setIndex == i32(SetType.awakeSet))
				assert(contact.colorIndex == colorIndex)
				assert(contact.localIndex == i)

				bodyIdA := contact.edges[0].bodyId
				bodyIdB := contact.edges[1].bodyId

				if colorIndex < OVERFLOW_INDEX {
					bodyA := Array_Get(&world.bodies, bodyIdA)
					bodyB := Array_Get(&world.bodies, bodyIdB)
					assert(GetBit(&color.bodySet, u32(bodyIdA)) == (bodyA.type == .dynamicBody))
					assert(GetBit(&color.bodySet, u32(bodyIdB)) == (bodyB.type == .dynamicBody))

					bitCount += bodyA.type == .dynamicBody ? 1 : 0
					bitCount += bodyB.type == .dynamicBody ? 1 : 0
				}
			}

			assert(color.jointSims.count >= 0)
			totalJointCount += color.jointSims.count
			for i: i32 = 0; i < color.jointSims.count; i += 1 {
				jointSim := &color.jointSims.data[i]
				joint := Array_Get(&world.joints, jointSim.jointId)
				assert(joint.setIndex == i32(SetType.awakeSet))
				assert(joint.colorIndex == colorIndex)
				assert(joint.localIndex == i)

				bodyIdA := joint.edges[0].bodyId
				bodyIdB := joint.edges[1].bodyId

				if colorIndex < OVERFLOW_INDEX {
					bodyA := Array_Get(&world.bodies, bodyIdA)
					bodyB := Array_Get(&world.bodies, bodyIdB)
					assert(GetBit(&color.bodySet, u32(bodyIdA)) == (bodyA.type == .dynamicBody))
					assert(GetBit(&color.bodySet, u32(bodyIdB)) == (bodyB.type == .dynamicBody))

					bitCount += bodyA.type == .dynamicBody ? 1 : 0
					bitCount += bodyB.type == .dynamicBody ? 1 : 0
				}
			}

			// Validate the bit population for this graph color
			assert(bitCount == CountSetBits(&color.bodySet))
		}

		contactIdCount := GetIdCount(&world.contactIdPool)
		assert(totalContactCount == contactIdCount)
		assert(totalContactCount == i32(world.broadPhase.pairSet.count))

		jointIdCount := GetIdCount(&world.jointIdPool)
		assert(totalJointCount == jointIdCount)

		// Validate shapes
		// This is very slow on compounds
		// unified: not ported: an `#if 0` block upstream (shape validation written against an older API)
	}

	// Validate contact touching status.
	ValidateContacts :: proc(world: ^World) {
		graph := &world.constraintGraph
		contactCount := world.contacts.count
		assert(contactCount == GetIdCapacity(&world.contactIdPool))
		allocatedContactCount: i32 = 0

		for contactIndex: i32 = 0; contactIndex < contactCount; contactIndex += 1 {
			contact := Array_Get(&world.contacts, contactIndex)
			if contact.contactId == NULL_INDEX {
				continue
			}

			assert(contact.contactId == contactIndex)

			allocatedContactCount += 1

			touching := (contact.flags & contactTouchingFlag) != 0

			setId := contact.setIndex
			set := Array_Get(&world.solverSets, setId)

			if setId == i32(SetType.awakeSet) {
				if touching {
					assert(0 <= contact.colorIndex && contact.colorIndex < GRAPH_COLOR_COUNT)

					if (contact.flags & simMeshContact) != 0 || contact.colorIndex == OVERFLOW_INDEX {
						color := &graph.colors[contact.colorIndex]
						contactId := Array_Get(&color.contacts, contact.localIndex).contactId
						assert(contactId == contactIndex)
					} else {
						color := &graph.colors[contact.colorIndex]
						contactId := Array_Get(&color.convexContacts, contact.localIndex)^
						assert(contactId == contactIndex)
					}
				} else {
					assert(contact.colorIndex == NULL_INDEX)
					assert(contact.manifolds == nil)
					assert(contact.manifoldCount == 0)

					index := Array_Get(&set.contactIndices, contact.localIndex)
					assert(index^ == contactIndex)
				}
			} else if setId >= i32(SetType.firstSleepingSet) {
				// Only touching contacts allowed in a sleeping set
				assert(touching == true)
				assert(contact.manifolds != nil)
				assert(contact.manifoldCount > 0)
				index := Array_Get(&set.contactIndices, contact.localIndex)
				assert(index^ == contactIndex)
			} else {
				// Sleeping and non-touching contacts belong in the disabled set
				assert(touching == false && setId == i32(SetType.disabledSet))
				assert(contact.manifolds == nil)
				assert(contact.manifoldCount == 0)
				index := Array_Get(&set.contactIndices, contact.localIndex)
				assert(index^ == contactIndex)
			}

			if (contact.flags & simMeshContact) != 0 {
				cacheCount := contact.meshContact.triangleCache.count
				if cacheCount > 0 {
					assert(contact.meshContact.triangleCache.data != nil)
					assert(contact.meshContact.triangleCache.capacity >= cacheCount)

					shapeA := Array_Get(&world.shapes, contact.shapeIdA)
					if shapeA.type == .meshShape {
						triangleCount := shapeA.mesh.data.triangleCount
						for i: i32 = 0; i < cacheCount; i += 1 {
							triangleIndex := contact.meshContact.triangleCache.data[i].triangleIndex
							assert(0 <= triangleIndex && triangleIndex < triangleCount)
						}
					} else if shapeA.type == .heightShape {
						triangleCount := GetHeightFieldTriangleCount(shapeA.heightField)
						for i: i32 = 0; i < cacheCount; i += 1 {
							triangleIndex := contact.meshContact.triangleCache.data[i].triangleIndex
							assert(0 <= triangleIndex && triangleIndex < triangleCount)
						}
					} else {
						assert(shapeA.type == .compoundShape)
						child := GetCompoundChild(shapeA.compound, contact.childIndex)
						assert(child.type == .meshShape)

						triangleCount := child.mesh.data.triangleCount
						for i: i32 = 0; i < cacheCount; i += 1 {
							triangleIndex := contact.meshContact.triangleCache.data[i].triangleIndex
							assert(0 <= triangleIndex && triangleIndex < triangleCount)
						}
					}
				}
			}
		}

		contactIdCount := GetIdCount(&world.contactIdPool)
		assert(allocatedContactCount == contactIdCount)
	}

} else {

	ValidateConnectivity :: proc(world: ^World) {
		_ = world
	}

	ValidateSolverSets :: proc(world: ^World) {
		_ = world
	}

	ValidateContacts :: proc(world: ^World) {
		_ = world
	}

}
