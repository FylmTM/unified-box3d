// Ported from box3d src/body.c and src/body.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

// B3_REC and B3_REC_CREATE are expanded inline at each call site (`if world.recording != nil {...}`):
// the macros paste the op name into the args type and writer names, which Odin cannot do.

// b3BodyFlags
// This body has fixed translation along the x-axis
lockLinearX :: 0x00000001

// This body has fixed translation along the y-axis
lockLinearY :: 0x00000002

// This body has fixed translation along the z-axis
lockLinearZ :: 0x00000004

// This body has fixed rotation around the x-axis
lockAngularX :: 0x00000008

// This body has fixed rotation around the y-axis
lockAngularY :: 0x00000010

// This body has fixed rotation around the z-axis
lockAngularZ :: 0x00000020

// This flag is used for debug draw and contact recycling. Only lives on b3BodySim.
isFast :: 0x00000040

// This dynamic body does a final CCD pass against all body types, but not other bullets
isBullet :: 0x00000080

// This body was speed capped in the current time step
isSpeedCapped :: 0x00000100

// This body had a time of impact event in the current time step (body sim only)
hadTimeOfImpact :: 0x00000200

// This body has no limit on angular velocity
allowFastRotation :: 0x00000400

// This bullet body needs to have its AABB increased. Needed because bullets don't follow
// the standard broad-phase update.
enlargeBulletBounds :: 0x00000800

// This body is dynamic so the solver should write to it.
// This prevents writing to kinematic bodies that causes a multithreaded sharing
// cache coherence problem even when the values are not changing.
// Used for b3BodyState flags.
dynamicFlag :: 0x00001000

enableSleep :: 0x00002000

enableContactRecycling :: 0x00004000

// The user deferred mass computation via the updateBodyMass shape option and mass
// data still hasn't been set.
dirtyMass :: 0x00008000

enableGyroscopicTorque :: 0x00010000

// All lock flags
allLocks :: lockLinearX | lockLinearY | lockLinearZ | lockAngularX | lockAngularY | lockAngularZ

// If all these flags are set then the body has fixed rotation
fixedRotation :: lockAngularX | lockAngularY | lockAngularZ

// These flags are transient per time step. These may be different across b3Body, b3BodySim, and b3BodyState.
bodyTransientFlags :: isSpeedCapped | hadTimeOfImpact

// Body organizational details that are not used in the solver.
Body :: struct {
	userData:       rawptr,

	// index of solver set stored in b3World
	// may be B3_NULL_INDEX
	setIndex:       i32,

	// body sim and state index within set
	// may be B3_NULL_INDEX
	localIndex:     i32,

	// [31 : contactId | 1 : edgeIndex]
	headContactKey: i32,
	contactCount:   i32,

	// todo maybe move this to the body sim
	headShapeId:    i32,
	shapeCount:     i32,
	headChainId:    i32,

	// [31 : jointId | 1 : edgeIndex]
	headJointKey:   i32,
	jointCount:     i32,

	// All enabled dynamic and kinematic bodies are in an island.
	islandId:       i32,

	// Index into the island's bodies array for O(1) swap-removal.
	// B3_NULL_INDEX when not in an island.
	islandIndex:    i32,
	sleepThreshold: f32,
	sleepTime:      f32,
	sleepVelocity:  f32,
	safetyFactor:   f32,
	mass:           f32,

	// this is used to adjust the fellAsleep flag in the body move array
	bodyMoveIndex:  i32,
	id:             i32,

	// b3BodyFlags
	flags:          u32,
	nameId:         u32,
	type:           BodyType,

	// This is monotonically advanced when a body is allocated in this slot
	// Used to check for invalid b3BodyId
	generation:     u16,
}

// Body State
// The body state is designed for fast conversion to and from SIMD via scatter-gather.
// Only awake dynamic and kinematic bodies have a body state.
// This is used in the performance critical constraint solver
//
// The solver operates on the body state. The body state array does not hold static bodies. Static bodies are shared
// across worker threads. It would be okay to read their states, but writing to them would cause cache thrashing across
// workers, even if the values don't change.
// This causes some trouble when computing anchors. I rotate joint anchors using the body rotation every sub-step. For static
// bodies the anchor doesn't rotate. Body A or B could be static and this can lead to lots of branching. This branching
// should be minimized.
//
// Solution 1:
// Use delta rotations. This means anchors need to be prepared in world space. The delta rotation for static bodies will be
// identity using a dummy state. Base separation and angles need to be computed. Manifolds will be behind a frame, but that
// is probably best if bodies move fast.
//
// Solution 2:
// Use full rotation. The anchors for static bodies will be in world space while the anchors for dynamic bodies will be in local
// space. Potentially confusing and bug prone.
//
// Note:
// I rotate joint anchors each sub-step but not contact anchors. Joint stability improves a lot by rotating joint anchors
// according to substep progress. Contacts have reduced stability when anchors are rotated during substeps, especially for
// round shapes.
//
// 64 bytes
BodyState :: struct {
	linearVelocity:  Vec3, // 12
	padding1:        f32,
	angularVelocity: Vec3, // 12
	padding2:        f32,

	// Using delta position reduces round-off error far from the origin
	deltaPosition:   Vec3, // 12

	// b3BodyFlags
	// Important flags: locking, dynamic
	flags:           u32, // 4

	// Using delta rotation because I cannot access the full rotation on static bodies in
	// the solver and must use zero delta rotation for static bodies (c,s) = (1,0)
	deltaRotation:   Quat, // 16
}

// Identity body state, notice the deltaRotation is identity
@(rodata)
identityBodyState := BodyState{{0.0, 0.0, 0.0}, 0.0, {0.0, 0.0, 0.0}, 0.0, {0.0, 0.0, 0.0}, 0, {{0.0, 0.0, 0.0}, 1.0}}

// Body simulation data used for integration of position and velocity
// Transform data used for collision and solver preparation.
BodySim :: struct {
	// transform for body origin, double translation in large world mode
	transform:       WorldTransform,

	// center of mass position in world space
	center:          Pos,

	// b3BodyFlags
	flags:           u32,
	minExtent:       f32,
	maxExtent:       Vec3,

	// previous rotation and COM for TOI
	rotation0:       Quat,
	center0:         Pos,

	// location of center of mass relative to the body origin
	localCenter:     Vec3,
	force:           Vec3,
	torque:          Vec3,
	invMass:         f32,

	// Local space inertia
	inertia:         Matrix3,

	// Rotational inertia about the center of mass. The world space inverse inertia tensor
	// must be updated whenever the body rotation is modified.
	invInertiaLocal: Matrix3,
	invInertiaWorld: Matrix3,
	linearDamping:   f32,
	angularDamping:  f32,
	gravityScale:    f32,

	// Index of b3Body
	bodyId:          i32,
}

// Encode the body sim index for storage in the contact.
EncodeBodySimIndex :: proc(body: ^Body) -> i32 {
	if body.setIndex == i32(SetType.awakeSet) {
		return body.localIndex
	}

	if body.setIndex == i32(SetType.staticSet) {
		return -(body.localIndex + 2)
	}

	return NULL_INDEX
}

IsStaticSimIndex :: proc(encodedBodySimIndex: i32) -> bool {
	return encodedBodySimIndex < NULL_INDEX
}

DecodeAwakeIndex :: proc(encodedBodySimIndex: i32) -> i32 {
	return encodedBodySimIndex >= 0 ? encodedBodySimIndex : NULL_INDEX
}

SleepBodySimIndex :: proc(encodedBodySimIndex: i32) -> i32 {
	return encodedBodySimIndex >= 0 ? NULL_INDEX : encodedBodySimIndex
}

// Make a sweep relative to a base position to keep TOI in float precision far from the origin.
MakeRelativeSweep :: proc(bodySim: ^BodySim, base: Pos) -> Sweep {
	s: Sweep
	s.c1 = SubPos(bodySim.center0, base)
	s.c2 = SubPos(bodySim.center, base)
	s.q1 = bodySim.rotation0
	s.q2 = bodySim.transform.q
	s.localCenter = bodySim.localCenter
	return s
}

// Get a validated body from a world using an id.
GetBodyFullId :: proc(world: ^World, bodyId: BodyId) -> ^Body {
	assert(Body_IsValid(bodyId))

	// id index starts at one so that zero can represent null
	// id index starts at one so that zero can represent null
	return Array_Get(&world.bodies, bodyId.index1 - 1)
}

GetBodyTransformQuick :: proc(world: ^World, body: ^Body) -> WorldTransform {
	set := Array_Get(&world.solverSets, body.setIndex)
	bodySim := Array_Get(&set.bodySims, body.localIndex)
	return bodySim.transform
}

GetBodyTransform :: proc(world: ^World, bodyId: i32) -> WorldTransform {
	body := Array_Get(&world.bodies, bodyId)
	return GetBodyTransformQuick(world, body)
}

// Create a b3BodyId from a raw id.
MakeBodyId :: proc(world: ^World, bodyId: i32) -> BodyId {
	body := Array_Get(&world.bodies, bodyId)
	return BodyId{bodyId + 1, world.worldId, body.generation}
}

GetBodySim :: proc(world: ^World, body: ^Body) -> ^BodySim {
	set := Array_Get(&world.solverSets, body.setIndex)
	bodySim := Array_Get(&set.bodySims, body.localIndex)
	return bodySim
}

GetBodyState :: proc(world: ^World, body: ^Body) -> ^BodyState {
	if body.setIndex == i32(SetType.awakeSet) {
		set := Array_Get(&world.solverSets, i32(SetType.awakeSet))
		return Array_Get(&set.bodyStates, body.localIndex)
	}

	return nil
}

RefreshBodyContactIndices :: proc(world: ^World, body: ^Body) {
	encodedBodySimIndex := EncodeBodySimIndex(body)

	contactKey := body.headContactKey
	for contactKey != NULL_INDEX {
		edgeIndex := contactKey & 1
		contactId := contactKey >> 1

		contact := Array_Get(&world.contacts, contactId)
		contactKey = contact.edges[edgeIndex].nextKey

		if edgeIndex == 0 {
			contact.encodedBodySimA = encodedBodySimIndex
		} else {
			contact.encodedBodySimB = encodedBodySimIndex
		}
	}
}

SyncBodyFlags :: proc(world: ^World, body: ^Body) {
	bodySim := GetBodySim(world, body)

	// Preserve the sim only flags: fast for contact recycling, time of impact for debug draw.
	bodySim.flags = (bodySim.flags & (isFast | hadTimeOfImpact)) | (body.flags &~ bodyTransientFlags)

	bodyState := GetBodyState(world, body)
	if bodyState != nil {
		bodyState.flags = body.flags &~ bodyTransientFlags
	}
}

@(private = "file")
CreateIslandForBody :: proc(world: ^World, setIndex: i32, body: ^Body) {
	assert(body.islandId == NULL_INDEX)
	assert(setIndex != i32(SetType.disabledSet))

	island := CreateIsland(world, setIndex)
	Array_Push(&island.bodies, body.id)
	body.islandId = island.islandId
	body.islandIndex = 0

	ValidateIsland(world, island.islandId)
}

@(private = "file")
RemoveBodyFromIsland :: proc(world: ^World, body: ^Body) {
	if body.islandId == NULL_INDEX {
		assert(body.islandIndex == NULL_INDEX)
		return
	}

	islandId := body.islandId
	island := Array_Get(&world.islands, islandId)
	{
		localIndex := body.islandIndex
		movedBodyId := island.bodies.data[island.bodies.count - 1]
		island.bodies.data[localIndex] = movedBodyId
		VALIDATE(world.bodies.data[movedBodyId].islandIndex == island.bodies.count - 1)
		world.bodies.data[movedBodyId].islandIndex = localIndex
		island.bodies.count -= 1
	}

	if island.bodies.count == 0 {
		// Destroy empty island
		assert(island.contacts.count == 0)
		assert(island.joints.count == 0)

		// Free the island
		DestroyIsland(world, island.islandId)
	} else {
		ValidateIsland(world, islandId)
	}

	body.islandId = NULL_INDEX
	body.islandIndex = NULL_INDEX
}

@(private = "file")
DestroyBodyContacts :: proc(world: ^World, body: ^Body, wakeBodies: bool) {
	// Destroy the attached contacts
	edgeKey := body.headContactKey
	for edgeKey != NULL_INDEX {
		contactId := edgeKey >> 1
		edgeIndex := edgeKey & 1

		contact := Array_Get(&world.contacts, contactId)
		edgeKey = contact.edges[edgeIndex].nextKey
		DestroyContact(world, contact, wakeBodies)
	}

	ValidateSolverSets(world)
}

CreateBody :: proc(worldId: WorldId, def: ^BodyDef) -> BodyId {
	CHECK_DEF(def)
	assert(IsValidPosition(def.position))
	assert(IsValidQuat(def.rotation))
	assert(IsValidVec3(def.linearVelocity))
	assert(IsValidVec3(def.angularVelocity))
	assert(IsValidFloat(def.linearDamping) && def.linearDamping >= 0.0)
	assert(IsValidFloat(def.angularDamping) && def.angularDamping >= 0.0)
	assert(IsValidFloat(def.sleepThreshold) && def.sleepThreshold >= 0.0)
	assert(IsValidFloat(def.safetyFactor) && def.safetyFactor >= 0.0)
	assert(IsValidFloat(def.gravityScale))

	world := GetUnlockedWorldFromId(worldId)

	if world == nil {
		return nullBodyId
	}

	world.locked = true

	isAwake := (def.isAwake || def.enableSleep == false) && def.isEnabled

	// determine the solver set
	setId: i32
	if def.isEnabled == false {
		// any body type can be disabled
		setId = i32(SetType.disabledSet)
	} else if def.type == .staticBody {
		setId = i32(SetType.staticSet)
	} else if isAwake == true {
		setId = i32(SetType.awakeSet)
	} else {
		// new set for a sleeping body in its own island
		setId = AllocId(&world.solverSetIdPool)
		if setId == world.solverSets.count {
			// Create a zero initialized solver set. All sub-arrays are also zero initialized.
			Array_Push(&world.solverSets, SolverSet{})
		} else {
			assert(world.solverSets.data[setId].setIndex == NULL_INDEX)
		}

		world.solverSets.data[setId].setIndex = setId
	}

	assert(0 <= setId && setId < world.solverSets.count)

	bodyId := AllocId(&world.bodyIdPool)

	lockFlags: u32 = 0
	lockFlags |= def.motionLocks.linearX ? lockLinearX : 0
	lockFlags |= def.motionLocks.linearY ? lockLinearY : 0
	lockFlags |= def.motionLocks.linearZ ? lockLinearZ : 0
	lockFlags |= def.motionLocks.angularX ? lockAngularX : 0
	lockFlags |= def.motionLocks.angularY ? lockAngularY : 0
	lockFlags |= def.motionLocks.angularZ ? lockAngularZ : 0

	set := Array_Get(&world.solverSets, setId)
	bodySim := Array_Emplace(&set.bodySims)
	bodySim^ = BodySim{}
	bodySim.transform.p = def.position
	bodySim.transform.q = def.rotation
	bodySim.center = def.position
	bodySim.rotation0 = bodySim.transform.q
	bodySim.center0 = bodySim.center
	bodySim.localCenter = Vec3_zero
	bodySim.force = Vec3_zero
	bodySim.torque = Vec3_zero
	bodySim.invMass = 0.0
	bodySim.inertia = Mat3_zero
	bodySim.invInertiaLocal = Mat3_zero
	bodySim.minExtent = HUGE
	bodySim.maxExtent = Vec3_zero
	bodySim.linearDamping = def.linearDamping
	bodySim.angularDamping = def.angularDamping
	bodySim.gravityScale = def.gravityScale
	bodySim.bodyId = bodyId
	bodySim.flags = lockFlags
	bodySim.flags |= def.isBullet ? isBullet : 0
	bodySim.flags |= def.allowFastRotation ? allowFastRotation : 0
	bodySim.flags |= def.type == .dynamicBody ? dynamicFlag : 0
	bodySim.flags |= def.enableSleep ? enableSleep : 0
	bodySim.flags |= def.enableContactRecycling ? enableContactRecycling : 0

	if setId == i32(SetType.awakeSet) {
		bodyState := Array_Emplace(&set.bodyStates)

		bodyState^ = BodyState{}
		bodyState.linearVelocity = def.linearVelocity
		bodyState.angularVelocity = def.angularVelocity
		bodyState.deltaRotation = Quat_identity
		bodyState.flags = bodySim.flags
	}

	if bodyId == world.bodies.count {
		Array_Push(&world.bodies, Body{})
	} else {
		assert(world.bodies.data[bodyId].id == NULL_INDEX)
	}

	body := Array_Get(&world.bodies, bodyId)
	body.userData = def.userData
	body.setIndex = setId
	body.localIndex = set.bodySims.count - 1
	body.generation += 1
	body.headShapeId = NULL_INDEX
	body.shapeCount = 0
	body.headChainId = NULL_INDEX
	body.headContactKey = NULL_INDEX
	body.contactCount = 0
	body.headJointKey = NULL_INDEX
	body.jointCount = 0
	body.islandId = NULL_INDEX
	body.islandIndex = NULL_INDEX
	body.bodyMoveIndex = NULL_INDEX
	body.id = bodyId
	body.sleepThreshold = def.sleepThreshold
	body.sleepTime = 0.0
	body.sleepVelocity = 0.0
	body.safetyFactor = def.safetyFactor
	body.mass = 0.0
	body.nameId = AddName(&world.names, def.name)
	body.type = def.type
	body.flags = bodySim.flags

	// dynamic and kinematic bodies that are enabled need a island
	if setId >= i32(SetType.awakeSet) {
		CreateIslandForBody(world, setId, body)
	}

	ValidateSolverSets(world)

	id := BodyId{bodyId + 1, world.worldId, body.generation}

	world.locked = false

	if world.recording != nil {
		recCreateArgs := RecArgs_CreateBody{worldId, def^}
		RecWriteRet_CreateBody(world.recording, &recCreateArgs, id)
	}

	return id
}

IsBodyAwake :: proc(world: ^World, body: ^Body) -> bool {
	_ = world
	return body.setIndex == i32(SetType.awakeSet)
}

WakeBody :: proc(world: ^World, body: ^Body) -> bool {
	if body.setIndex >= i32(SetType.firstSleepingSet) {
		WakeSolverSet(world, body.setIndex)
		ValidateSolverSets(world)
		return true
	}

	return false
}

WakeBodyWithLock :: proc(world: ^World, body: ^Body) -> bool {
	assert(world.locked == false)
	world.locked = true
	woke := WakeBody(world, body)
	world.locked = false
	return woke
}

DestroyBody :: proc(bodyId: BodyId) {
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_DestroyBody{bodyId}
		RecWrite_DestroyBody(world.recording, &recArgs)
	}

	world.locked = true

	body := GetBodyFullId(world, bodyId)

	// Wake bodies attached to this body, even if this body is static.
	wakeBodies := true

	// Destroy the attached joints
	edgeKey := body.headJointKey
	for edgeKey != NULL_INDEX {
		jointId := edgeKey >> 1
		edgeIndex := edgeKey & 1

		joint := Array_Get(&world.joints, jointId)
		edgeKey = joint.edges[edgeIndex].nextKey

		// Careful because this modifies the list being traversed
		DestroyJointInternal(world, joint, wakeBodies)
	}

	// Destroy all contacts attached to this body.
	DestroyBodyContacts(world, body, wakeBodies)

	// Destroy the attached shapes and their broad-phase proxies.
	shapeId := body.headShapeId
	for shapeId != NULL_INDEX {
		shape := Array_Get(&world.shapes, shapeId)

		if shape.type == .compoundShape {
			assert(world.compoundShapeCount > 0)
			world.compoundShapeCount -= 1
		}

		if shape.sensorIndex != NULL_INDEX {
			DestroySensor(world, shape)
		}

		DestroyShapeProxy(shape, &world.broadPhase)

		DestroyShapeAllocations(world, shape)

		// Return shape to free list.
		FreeId(&world.shapeIdPool, shapeId)
		shape.id = NULL_INDEX

		shapeId = shape.nextShapeId
	}

	RemoveBodyFromIsland(world, body)

	// Remove body sim from solver set that owns it
	set := Array_Get(&world.solverSets, body.setIndex)
	movedIndex := Array_RemoveSwap(&set.bodySims, body.localIndex)
	if movedIndex != NULL_INDEX {
		// Fix moved body index
		movedSim := &set.bodySims.data[body.localIndex]
		movedId := movedSim.bodyId
		movedBody := Array_Get(&world.bodies, movedId)
		assert(movedBody.localIndex == movedIndex)
		movedBody.localIndex = body.localIndex
		RefreshBodyContactIndices(world, movedBody)
	}

	// Remove body state from awake set
	if body.setIndex == i32(SetType.awakeSet) {
		result := Array_RemoveSwap(&set.bodyStates, body.localIndex)
		_ = result
		assert(result == movedIndex)
	} else if set.setIndex >= i32(SetType.firstSleepingSet) && set.bodySims.count == 0 {
		// Remove solver set if it's now an orphan.
		DestroySolverSet(world, set.setIndex)
	}

	// Free body and id (preserve body revision)
	FreeId(&world.bodyIdPool, body.id)

	body.setIndex = NULL_INDEX
	body.localIndex = NULL_INDEX
	body.id = NULL_INDEX

	ValidateSolverSets(world)

	world.locked = false
}

Body_GetContactCapacity :: proc(bodyId: BodyId) -> i32 {
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return 0
	}

	body := GetBodyFullId(world, bodyId)

	// Conservative and fast
	return body.contactCount
}

Body_GetContactData :: proc(bodyId: BodyId, contactData: [^]ContactData, capacity: i32) -> i32 {
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return 0
	}

	body := GetBodyFullId(world, bodyId)

	contactKey := body.headContactKey
	index: i32 = 0
	for contactKey != NULL_INDEX && index < capacity {
		contactId := contactKey >> 1
		edgeIndex := contactKey & 1

		contact := Array_Get(&world.contacts, contactId)

		// Is contact touching?
		if (contact.flags & contactTouchingFlag) != 0 {
			shapeA := Array_Get(&world.shapes, contact.shapeIdA)
			shapeB := Array_Get(&world.shapes, contact.shapeIdB)

			contactData[index].contactId = ContactId{contact.contactId + 1, bodyId.world0, 0, contact.generation}
			contactData[index].shapeIdA = ShapeId{shapeA.id + 1, bodyId.world0, shapeA.generation}
			contactData[index].shapeIdB = ShapeId{shapeB.id + 1, bodyId.world0, shapeB.generation}
			contactData[index].manifolds = contact.manifolds
			contactData[index].manifoldCount = contact.manifoldCount
			index += 1
		}

		contactKey = contact.edges[edgeIndex].nextKey
	}

	assert(index <= capacity)

	return index
}

Body_ComputeAABB :: proc(bodyId: BodyId) -> AABB {
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return AABB{}
	}

	body := GetBodyFullId(world, bodyId)
	if body.headShapeId == NULL_INDEX {
		transform := GetBodyTransform(world, body.id)
		p := ToVec3(transform.p)
		return AABB{p, p}
	}

	shape := Array_Get(&world.shapes, body.headShapeId)
	aabb := shape.aabb
	for shape.nextShapeId != NULL_INDEX {
		shape = Array_Get(&world.shapes, shape.nextShapeId)
		aabb = AABB_Union(aabb, shape.aabb)
	}

	return aabb
}

Body_GetClosestPoint :: proc(bodyId: BodyId, result: ^Vec3, target: Vec3) -> f32 {
	target := target
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		result^ = Vec3{}
		return 0.0
	}

	body := GetBodyFullId(world, bodyId)
	worldTransform := GetBodyTransform(world, body.id)
	transform := ToRelativeTransform(worldTransform, Pos_zero)

	closestDistance := FLT_MAX
	closestPoint := transform.p

	input := DistanceInput{}
	input.proxyA = ShapeProxy{&target, 1, 0.0}

	// Target rides in frame A at the origin, so the relative pose of the shape in A is the body transform
	input.transform = transform
	input.useRadii = false

	shapeId := body.headShapeId
	for shapeId != NULL_INDEX {
		shape := Array_Get(&world.shapes, shapeId)
		shapeId = shape.nextShapeId

		type := shape.type
		if type != .sphereShape && type != .capsuleShape && type != .hullShape {
			continue
		}

		input.proxyB = MakeShapeProxy(shape)

		cache := SimplexCache{}
		output := ShapeDistance(&input, &cache, nil, 0)
		if output.distance < closestDistance {
			closestDistance = output.distance
			closestPoint = output.pointB
		}
	}

	result^ = closestPoint
	return closestDistance
}

Body_CastRay :: proc(
	bodyId: BodyId,
	origin: Pos,
	translation: Vec3,
	filter: QueryFilter,
	maxFraction: f32,
	bodyTransform: WorldTransform,
) -> BodyCastResult {
	filter := filter
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return BodyCastResult{}
	}

	result := BodyCastResult{}
	body := GetBodyFullId(world, bodyId)

	// The consistent framing is to center on the ray origin.
	shapeInput := RayCastInput{}
	shapeInput.origin = Vec3_zero
	shapeInput.translation = translation
	shapeInput.maxFraction = maxFraction

	transform := ToRelativeTransform(bodyTransform, origin)

	shapeId := body.headShapeId
	for shapeId != NULL_INDEX {
		shape := Array_Get(&world.shapes, shapeId)
		shapeId = shape.nextShapeId

		if ShouldQueryCollide(&shape.filter, &filter) == false {
			continue
		}

		shapeOutput := RayCastShape(shape, transform, &shapeInput)

		if shapeOutput.hit == false {
			continue
		}

		if shapeOutput.fraction > shapeInput.maxFraction {
			continue
		}

		// Careful with id, shapeId is the next shape.
		id := ShapeId{shape.id + 1, bodyId.world0, shape.generation}

		materialIndex := ClampInt(shapeOutput.materialIndex, 0, shape.materialCount - 1)
		userMaterialId := GetShapeMaterials(shape)[materialIndex].userMaterialId

		result = BodyCastResult {
			shapeId        = id,
			point          = OffsetPos(origin, shapeOutput.point),
			normal         = shapeOutput.normal,
			fraction       = shapeOutput.fraction,
			triangleIndex  = shapeOutput.triangleIndex,
			userMaterialId = userMaterialId,
			iterations     = shapeOutput.iterations,
			hit            = true,
		}

		shapeInput.maxFraction = shapeOutput.fraction
	}

	return result
}

Body_CastShape :: proc(
	bodyId: BodyId,
	origin: Pos,
	proxy: ^ShapeProxy,
	translation: Vec3,
	filter: QueryFilter,
	maxFraction: f32,
	canEncroach: bool,
	bodyTransform: WorldTransform,
) -> BodyCastResult {
	filter := filter
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return BodyCastResult{}
	}

	result := BodyCastResult{}
	body := GetBodyFullId(world, bodyId)

	transform := ToRelativeTransform(bodyTransform, origin)

	shapeInput := ShapeCastInput{}
	shapeInput.proxy = proxy^
	shapeInput.translation = translation
	shapeInput.maxFraction = maxFraction
	shapeInput.canEncroach = canEncroach

	shapeId := body.headShapeId
	for shapeId != NULL_INDEX {
		shape := Array_Get(&world.shapes, shapeId)
		shapeId = shape.nextShapeId

		if ShouldQueryCollide(&shape.filter, &filter) == false {
			continue
		}

		shapeOutput := ShapeCastShape(shape, transform, &shapeInput)

		if shapeOutput.hit == false {
			continue
		}

		if shapeOutput.fraction > shapeInput.maxFraction {
			continue
		}

		// Careful with id, shapeId is the next shape.
		id := ShapeId{shape.id + 1, bodyId.world0, shape.generation}
		materialIndex := ClampInt(shapeOutput.materialIndex, 0, shape.materialCount - 1)
		userMaterialId := GetShapeMaterials(shape)[materialIndex].userMaterialId

		result = BodyCastResult {
			shapeId        = id,
			point          = OffsetPos(origin, shapeOutput.point),
			normal         = shapeOutput.normal,
			fraction       = shapeOutput.fraction,
			triangleIndex  = shapeOutput.triangleIndex,
			userMaterialId = userMaterialId,
			iterations     = shapeOutput.iterations,
			hit            = true,
		}

		shapeInput.maxFraction = shapeOutput.fraction
	}

	return result
}

Body_OverlapShape :: proc(
	bodyId: BodyId,
	origin: Pos,
	proxy: ^ShapeProxy,
	filter: QueryFilter,
	bodyTransform: WorldTransform,
) -> bool {
	filter := filter
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return false
	}

	body := GetBodyFullId(world, bodyId)
	transform := ToRelativeTransform(bodyTransform, origin)

	shapeId := body.headShapeId
	for shapeId != NULL_INDEX {
		shape := Array_Get(&world.shapes, shapeId)
		shapeId = shape.nextShapeId

		if ShouldQueryCollide(&shape.filter, &filter) == false {
			continue
		}

		overlaps := OverlapShape(shape, transform, proxy)
		if overlaps {
			return true
		}
	}

	return false
}

Body_CollideMover :: proc(
	bodyId: BodyId,
	bodyPlanes: [^]BodyPlaneResult,
	planeCapacity: i32,
	origin: Pos,
	mover: ^Capsule,
	filter: QueryFilter,
	bodyTransform: WorldTransform,
) -> i32 {
	filter := filter
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return 0
	}

	if planeCapacity == 0 {
		return 0
	}

	resultCount: i32 = 0
	body := GetBodyFullId(world, bodyId)

	transform := ToRelativeTransform(bodyTransform, origin)

	shapeId := body.headShapeId
	for shapeId != NULL_INDEX {
		shape := Array_Get(&world.shapes, shapeId)
		shapeId = shape.nextShapeId

		if ShouldQueryCollide(&shape.filter, &filter) == false {
			continue
		}

		type := shape.type
		if type != .sphereShape && type != .capsuleShape && type != .hullShape {
			continue
		}

		plane: PlaneResult
		count := CollideMover(&plane, 1, shape, transform, mover)

		if count > 0 {
			id := ShapeId{shape.id + 1, bodyId.world0, shape.generation}
			bodyPlanes[resultCount] = BodyPlaneResult {
				shapeId = id,
				result  = plane,
			}
			resultCount += 1
			if resultCount == planeCapacity {
				return resultCount
			}
		}
	}

	return resultCount
}

Body_TimeOfImpactMover :: proc(
	bodyId: BodyId,
	origin: Pos,
	mover: ^Capsule,
	moverTranslation: Vec3,
	filter: QueryFilter,
	bodyTransform1: WorldTransform,
	bodyTransform2: WorldTransform,
) -> BodyTOIResult {
	filter := filter
	result := BodyTOIResult{}
	result.fraction = 1.0

	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return result
	}

	xf1 := ToRelativeTransform(bodyTransform1, origin)
	xf2 := ToRelativeTransform(bodyTransform2, origin)

	body := GetBodyFullId(world, bodyId)
	bodySim := GetBodySim(world, body)
	localCenter := bodySim.localCenter

	capsulePoints := [2]Vec3{mover.center1, mover.center2}
	input := TOIInput{}
	input.proxyB = ShapeProxy {
		points = &capsulePoints[0],
		count  = 2,
		radius = mover.radius,
	}
	input.sweepA.c1 = TransformPoint(xf1, localCenter)
	input.sweepA.c2 = TransformPoint(xf2, localCenter)
	input.sweepA.q1 = bodyTransform1.q
	input.sweepA.q2 = bodyTransform2.q
	input.sweepA.localCenter = localCenter

	input.sweepB.c1 = Vec3_zero
	input.sweepB.c2 = moverTranslation
	input.sweepB.q1 = Quat_identity
	input.sweepB.q2 = Quat_identity
	input.sweepB.localCenter = Vec3_zero

	input.maxFraction = 1.0

	shapeId := body.headShapeId
	for shapeId != NULL_INDEX {
		shape := Array_Get(&world.shapes, shapeId)
		shapeId = shape.nextShapeId

		if ShouldQueryCollide(&shape.filter, &filter) == false {
			continue
		}

		type := shape.type
		if type != .sphereShape && type != .capsuleShape && type != .hullShape {
			continue
		}

		input.proxyA = MakeShapeProxy(shape)

		output := TimeOfImpact(&input)
		VALIDATE(output.state != .toiStateUnknown)

		// Mimic behavior in b3ContinuousQueryCallback. Ignore shapes that initially overlap.
		if 0.0 < output.fraction && output.fraction < result.fraction {
			input.maxFraction = output.fraction

			result.point = OffsetPos(origin, output.point)
			result.normal = output.normal
			result.fraction = output.fraction
			result.shapeId = ShapeId {
				index1     = shape.id + 1,
				world0     = world.worldId,
				generation = shape.generation,
			}
		}
	}

	return result
}

// This computes the deviation of the principal inertia values without computing eigenvalues.
// It computes:
// ((I1 - I2)^2 + (I1 - I3)^2 + (I2 - I3)^2) / (I1^2 + I2^2 + I3^2)
// https://en.wikipedia.org/wiki/Cauchy_stress_tensor#Stress_deviator_tensor
@(private = "file")
UpdateIsotropicInertia :: proc(world: ^World, body: ^Body, bodySim: ^BodySim) {
	inertia := bodySim.inertia
	mean := (inertia.cx.x + inertia.cy.y + inertia.cz.z) / 3.0
	dx := inertia.cx.x - mean
	dy := inertia.cy.y - mean
	dz := inertia.cz.z - mean
	offDiagonal := inertia.cy.x * inertia.cy.x + inertia.cz.x * inertia.cz.x + inertia.cz.y * inertia.cz.y

	// This is the square of the Frobenius matrix norm.
	// https://en.wikipedia.org/wiki/Matrix_norm
	deviation := dx * dx + dy * dy + dz * dz + 2.0 * offDiagonal

	tolerance := world.isotropicInertiaTolerance

	if 2.0 * deviation > 3.0 * tolerance * tolerance * mean * mean {
		body.flags |= enableGyroscopicTorque
	} else {
		body.flags &~= enableGyroscopicTorque
	}

	SyncBodyFlags(world, body)
}

UpdateBodyMassData :: proc(world: ^World, body: ^Body) {
	bodySim := GetBodySim(world, body)

	// Mass is no longer dirty
	body.flags &~= dirtyMass
	body.flags &~= enableGyroscopicTorque
	SyncBodyFlags(world, body)

	// Compute mass data from shapes. Each shape has its own density.
	body.mass = 0.0
	bodySim.inertia = Mat3_zero
	bodySim.invMass = 0.0
	bodySim.invInertiaLocal = Mat3_zero
	bodySim.invInertiaWorld = Mat3_zero
	bodySim.localCenter = Vec3_zero
	bodySim.minExtent = HUGE
	bodySim.maxExtent = Vec3_zero

	if body.headShapeId == NULL_INDEX {
		return
	}

	// Static and kinematic sims have zero mass.
	if body.type != .dynamicBody {
		bodySim.center = bodySim.transform.p
		bodySim.center0 = bodySim.center

		// Need extents for kinematic bodies for sleeping to work correctly.
		if body.type == .kinematicBody {
			shapeId := body.headShapeId
			for shapeId != NULL_INDEX {
				s := Array_Get(&world.shapes, shapeId)

				extent := ComputeShapeExtent(s, Vec3_zero)
				bodySim.minExtent = MinFloat(bodySim.minExtent, extent.minExtent)
				bodySim.maxExtent = Max(bodySim.maxExtent, extent.maxExtent)

				shapeId = s.nextShapeId
			}
		}

		return
	}

	shapeCount := body.shapeCount
	masses := cast([^]MassData)StackAlloc(&world.stack, shapeCount * size_of(MassData), "mass data")

	// Accumulate mass over all shapes.
	localCenter := Vec3_zero
	shapeId := body.headShapeId
	shapeIndex: i32 = 0
	for shapeId != NULL_INDEX {
		s := Array_Get(&world.shapes, shapeId)
		shapeId = s.nextShapeId

		if s.density == 0.0 {
			masses[shapeIndex] = MassData{}
			shapeIndex += 1
			continue
		}

		massData := ComputeShapeMass(s)
		body.mass += massData.mass
		localCenter = MulAdd(localCenter, massData.mass, massData.center)

		masses[shapeIndex] = massData
		shapeIndex += 1
	}

	// Compute center of mass.
	if body.mass > 0.0 {
		bodySim.invMass = 1.0 / body.mass
		localCenter = MulSV(bodySim.invMass, localCenter)
	}

	// Second loop to accumulate the rotational inertia about the center of mass
	for shapeIndex = 0; shapeIndex < shapeCount; shapeIndex += 1 {
		massData := masses[shapeIndex]
		if massData.mass == 0.0 {
			continue
		}

		// Shift to center of mass. This is safe because it can only increase.
		offset := Sub(localCenter, massData.center)
		inertia := AddMM(massData.inertia, Steiner(massData.mass, offset))
		bodySim.inertia = AddMM(bodySim.inertia, inertia)
	}

	StackFree(&world.stack, masses)
	masses = nil

	det := Det(bodySim.inertia)
	assert(det >= 0.0)

	if det > 0.0 {
		// This call is faster than b3Invert
		bodySim.invInertiaLocal = InvertT(bodySim.inertia)

		rotationMatrix := MakeMatrixFromQuat(bodySim.transform.q)
		bodySim.invInertiaWorld = MulMM(MulMM(rotationMatrix, bodySim.invInertiaLocal), Transpose(rotationMatrix))
	}

	// Move center of mass.
	oldCenter := bodySim.center
	bodySim.localCenter = localCenter
	bodySim.center = TransformWorldPoint(bodySim.transform, bodySim.localCenter)
	bodySim.center0 = bodySim.center

	// Update center of mass velocity
	state := GetBodyState(world, body)
	if state != nil {
		deltaLinear := Cross(state.angularVelocity, SubPos(bodySim.center, oldCenter))
		state.linearVelocity = Add(state.linearVelocity, deltaLinear)
	}

	// Compute body extents relative to center of mass
	shapeId = body.headShapeId
	for shapeId != NULL_INDEX {
		s := Array_Get(&world.shapes, shapeId)

		extent := ComputeShapeExtent(s, localCenter)
		bodySim.minExtent = MinFloat(bodySim.minExtent, extent.minExtent)
		bodySim.maxExtent = Max(bodySim.maxExtent, extent.maxExtent)

		shapeId = s.nextShapeId
	}

	// When the center of mass changes, any cached contact manifold becomes invalid.
	edgeKey := body.headContactKey
	for edgeKey != NULL_INDEX {
		contactId := edgeKey >> 1
		edgeIndex := edgeKey & 1

		contact := Array_Get(&world.contacts, contactId)
		contact.flags &~= relativeTransformValid

		edgeKey = contact.edges[edgeIndex].nextKey
	}

	// Apply fixed rotation
	if (bodySim.flags & fixedRotation) == fixedRotation {
		bodySim.inertia = Mat3_zero
		bodySim.invInertiaLocal = Mat3_zero
		bodySim.invInertiaWorld = Mat3_zero
	}

	UpdateIsotropicInertia(world, body, bodySim)
}

Body_GetPosition :: proc(bodyId: BodyId) -> Pos {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	transform := GetBodyTransformQuick(world, body)
	return transform.p
}

Body_GetRotation :: proc(bodyId: BodyId) -> Quat {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	transform := GetBodyTransformQuick(world, body)
	return transform.q
}

Body_GetTransform :: proc(bodyId: BodyId) -> WorldTransform {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	return GetBodyTransformQuick(world, body)
}

Body_GetLocalPoint :: proc(bodyId: BodyId, worldPoint: Pos) -> Vec3 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	transform := GetBodyTransformQuick(world, body)
	return InvTransformWorldPoint(transform, worldPoint)
}

Body_GetWorldPoint :: proc(bodyId: BodyId, localPoint: Vec3) -> Pos {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	transform := GetBodyTransformQuick(world, body)
	return TransformWorldPoint(transform, localPoint)
}

Body_GetLocalVector :: proc(bodyId: BodyId, worldVector: Vec3) -> Vec3 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	transform := GetBodyTransformQuick(world, body)
	return InvRotateVector(transform.q, worldVector)
}

Body_GetWorldVector :: proc(bodyId: BodyId, localVector: Vec3) -> Vec3 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	transform := GetBodyTransformQuick(world, body)
	return RotateVector(transform.q, localVector)
}

Body_SetTransform :: proc(bodyId: BodyId, position: Pos, rotation: Quat) {
	assert(IsValidPosition(position))
	assert(IsValidQuat(rotation))
	assert(Body_IsValid(bodyId))
	world := GetWorld(i32(bodyId.world0))
	assert(world.locked == false)

	if world.recording != nil {
		recArgs := RecArgs_BodySetTransform{bodyId, position, rotation}
		RecWrite_BodySetTransform(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)
	bodySim := GetBodySim(world, body)

	bodySim.transform.p = position
	bodySim.transform.q = rotation
	bodySim.center = TransformWorldPoint(bodySim.transform, bodySim.localCenter)

	rotationMatrix := MakeMatrixFromQuat(bodySim.transform.q)
	bodySim.invInertiaWorld = MulMM(MulMM(rotationMatrix, bodySim.invInertiaLocal), Transpose(rotationMatrix))

	bodySim.rotation0 = bodySim.transform.q
	bodySim.center0 = bodySim.center

	broadPhase := &world.broadPhase

	transform := bodySim.transform
	speculativeDistance := SPECULATIVE_DISTANCE

	shapeId := body.headShapeId
	for shapeId != NULL_INDEX {
		shape := Array_Get(&world.shapes, shapeId)
		aabb := ComputeFatShapeAABB(shape, transform, speculativeDistance)
		shape.aabb = aabb

		shapeFatAABB := &world.fatAABBs.data[shapeId]
		if AABB_Contains(shapeFatAABB^, aabb) == false {
			fatAABB := AABB_Inflate(aabb, shape.aabbMargin)
			shapeFatAABB^ = fatAABB

			// The body could be disabled
			if shape.proxyKey != NULL_INDEX {
				BroadPhase_MoveProxy(broadPhase, shape.proxyKey, fatAABB)
			}
		}

		shapeId = shape.nextShapeId
	}
}

Body_GetLinearVelocity :: proc(bodyId: BodyId) -> Vec3 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	state := GetBodyState(world, body)
	if state != nil {
		return state.linearVelocity
	}
	return Vec3_zero
}

Body_GetAngularVelocity :: proc(bodyId: BodyId) -> Vec3 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	state := GetBodyState(world, body)
	if state != nil {
		return state.angularVelocity
	}
	return Vec3_zero
}

Body_SetLinearVelocity :: proc(bodyId: BodyId, linearVelocity: Vec3) {
	assert(IsValidVec3(linearVelocity))

	world := GetWorld(i32(bodyId.world0))

	if world.recording != nil {
		recArgs := RecArgs_BodySetLinearVelocity{bodyId, linearVelocity}
		RecWrite_BodySetLinearVelocity(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)

	if body.type == .staticBody {
		return
	}

	if LengthSquared(linearVelocity) > 0.0 {
		WakeBodyWithLock(world, body)
	}

	state := GetBodyState(world, body)
	if state == nil {
		return
	}

	state.linearVelocity = linearVelocity
}

Body_SetAngularVelocity :: proc(bodyId: BodyId, angularVelocity: Vec3) {
	assert(IsValidVec3(angularVelocity))

	world := GetWorld(i32(bodyId.world0))

	if world.recording != nil {
		recArgs := RecArgs_BodySetAngularVelocity{bodyId, angularVelocity}
		RecWrite_BodySetAngularVelocity(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)

	if body.type == .staticBody {
		return
	}

	// Apply locks to avoid waking
	w: Vec3
	w.x = (body.flags & lockAngularX) != 0 ? 0.0 : angularVelocity.x
	w.y = (body.flags & lockAngularY) != 0 ? 0.0 : angularVelocity.y
	w.z = (body.flags & lockAngularZ) != 0 ? 0.0 : angularVelocity.z

	if LengthSquared(w) != 0.0 {
		WakeBodyWithLock(world, body)
	}

	state := GetBodyState(world, body)
	if state == nil {
		return
	}

	state.angularVelocity = w
}

Body_SetTargetTransform :: proc(bodyId: BodyId, target: WorldTransform, timeStep: f32, wake: bool) {
	assert(IsValidWorldTransform(target))

	world := GetWorld(i32(bodyId.world0))

	if world.recording != nil {
		recArgs := RecArgs_BodySetTargetTransform{bodyId, target, timeStep, wake}
		RecWrite_BodySetTargetTransform(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)

	if body.setIndex == i32(SetType.disabledSet) {
		return
	}

	if body.type == .staticBody || timeStep <= 0.0 {
		return
	}

	if body.setIndex != i32(SetType.awakeSet) && wake == false {
		return
	}

	sim := GetBodySim(world, body)

	// Compute linear velocity. The center difference is taken in world precision then demoted.
	center1 := sim.center
	center2 := TransformWorldPoint(target, sim.localCenter)
	invTimeStep := 1.0 / timeStep
	linearVelocity := MulSV(invTimeStep, SubPos(center2, center1))

	// Compute angular velocity:
	// q' = 0.5 * w * q
	// <~> ( q2 - q1 ) / dt =  0.5 * w * q1
	// <=> w = 2 * ( q2 - q1 ) * Conjugate( q1 ) / dt
	q1 := sim.transform.q
	q2 := target.q

	// Use the shortest arc quaternion
	if DotQuat(q1, q2) < 0.0 {
		q2 = NegateQuat(q2)
	}

	dq := Quat{Sub(q2.v, q1.v), q2.s - q1.s}
	omega := MulQuat(dq, Conjugate(q1))
	angularVelocity := MulSV(2.0 * invTimeStep, omega.v)

	// Early out if the body is asleep already and the desired movement is small
	if body.setIndex != i32(SetType.awakeSet) {
		maxVelocity := Length(linearVelocity) + Length(Mul(angularVelocity, sim.maxExtent))

		// Return if velocity would be sleepy
		if maxVelocity < body.sleepThreshold {
			return
		}

		// Must wake for state to exist
		WakeBodyWithLock(world, body)
	}

	assert(body.setIndex == i32(SetType.awakeSet))

	state := GetBodyState(world, body)
	state.linearVelocity = linearVelocity
	state.angularVelocity = angularVelocity
}

Body_GetLocalPointVelocity :: proc(bodyId: BodyId, localPoint: Vec3) -> Vec3 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	state := GetBodyState(world, body)
	if state == nil {
		return Vec3_zero
	}

	set := Array_Get(&world.solverSets, body.setIndex)
	bodySim := Array_Get(&set.bodySims, body.localIndex)

	r := RotateVector(bodySim.transform.q, Sub(localPoint, bodySim.localCenter))
	v := Add(state.linearVelocity, Cross(state.angularVelocity, r))
	return v
}

Body_GetWorldPointVelocity :: proc(bodyId: BodyId, worldPoint: Pos) -> Vec3 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	state := GetBodyState(world, body)
	if state == nil {
		return Vec3_zero
	}

	set := Array_Get(&world.solverSets, body.setIndex)
	bodySim := Array_Get(&set.bodySims, body.localIndex)

	r := SubPos(worldPoint, bodySim.center)
	v := Add(state.linearVelocity, Cross(state.angularVelocity, r))
	return v
}

Body_ApplyForce :: proc(bodyId: BodyId, force: Vec3, point: Pos, wake: bool) {
	assert(IsValidVec3(force))

	world := GetWorld(i32(bodyId.world0))

	if world.recording != nil {
		recArgs := RecArgs_BodyApplyForce{bodyId, force, point, wake}
		RecWrite_BodyApplyForce(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)

	if wake && body.setIndex >= i32(SetType.firstSleepingSet) {
		WakeBodyWithLock(world, body)
	}

	if body.setIndex == i32(SetType.awakeSet) {
		bodySim := GetBodySim(world, body)
		bodySim.force = Add(bodySim.force, force)
		bodySim.torque = Add(bodySim.torque, Cross(SubPos(point, bodySim.center), force))
	}
}

Body_ApplyForceToCenter :: proc(bodyId: BodyId, force: Vec3, wake: bool) {
	assert(IsValidVec3(force))

	world := GetWorld(i32(bodyId.world0))

	if world.recording != nil {
		recArgs := RecArgs_BodyApplyForceToCenter{bodyId, force, wake}
		RecWrite_BodyApplyForceToCenter(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)

	if wake && body.setIndex >= i32(SetType.firstSleepingSet) {
		WakeBodyWithLock(world, body)
	}

	if body.setIndex == i32(SetType.awakeSet) {
		bodySim := GetBodySim(world, body)
		bodySim.force = Add(bodySim.force, force)
	}
}

Body_ApplyTorque :: proc(bodyId: BodyId, torque: Vec3, wake: bool) {
	assert(IsValidVec3(torque))

	world := GetWorld(i32(bodyId.world0))

	if world.recording != nil {
		recArgs := RecArgs_BodyApplyTorque{bodyId, torque, wake}
		RecWrite_BodyApplyTorque(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)

	if wake && body.setIndex >= i32(SetType.firstSleepingSet) {
		WakeBodyWithLock(world, body)
	}

	if body.setIndex == i32(SetType.awakeSet) {
		bodySim := GetBodySim(world, body)
		bodySim.torque = Add(bodySim.torque, torque)
	}
}

Body_ApplyLinearImpulse :: proc(bodyId: BodyId, impulse: Vec3, point: Pos, wake: bool) {
	assert(IsValidVec3(impulse))
	assert(IsValidPosition(point))

	world := GetWorld(i32(bodyId.world0))

	if world.recording != nil {
		recArgs := RecArgs_BodyApplyLinearImpulse{bodyId, impulse, point, wake}
		RecWrite_BodyApplyLinearImpulse(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)

	if wake && body.setIndex >= i32(SetType.firstSleepingSet) {
		WakeBodyWithLock(world, body)
	}

	if body.setIndex == i32(SetType.awakeSet) {
		localIndex := body.localIndex
		set := Array_Get(&world.solverSets, i32(SetType.awakeSet))
		state := Array_Get(&set.bodyStates, localIndex)
		bodySim := Array_Get(&set.bodySims, localIndex)

		state.linearVelocity = MulAdd(state.linearVelocity, bodySim.invMass, impulse)

		maxLinearSpeed := world.maxLinearSpeed
		if LengthSquared(state.linearVelocity) > maxLinearSpeed * maxLinearSpeed {
			state.linearVelocity = MulSV(maxLinearSpeed, Normalize(state.linearVelocity))
		}

		delta := MulMV(bodySim.invInertiaWorld, Cross(SubPos(point, bodySim.center), impulse))
		state.angularVelocity = Add(state.angularVelocity, delta)
	}
}

Body_ApplyLinearImpulseToCenter :: proc(bodyId: BodyId, impulse: Vec3, wake: bool) {
	assert(IsValidVec3(impulse))

	world := GetWorld(i32(bodyId.world0))

	if world.recording != nil {
		recArgs := RecArgs_BodyApplyLinearImpulseToCenter{bodyId, impulse, wake}
		RecWrite_BodyApplyLinearImpulseToCenter(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)

	if wake && body.setIndex >= i32(SetType.firstSleepingSet) {
		WakeBodyWithLock(world, body)
	}

	if body.setIndex == i32(SetType.awakeSet) {
		localIndex := body.localIndex
		set := Array_Get(&world.solverSets, i32(SetType.awakeSet))
		state := Array_Get(&set.bodyStates, localIndex)
		bodySim := Array_Get(&set.bodySims, localIndex)
		state.linearVelocity = MulAdd(state.linearVelocity, bodySim.invMass, impulse)

		maxLinearSpeed := world.maxLinearSpeed
		if LengthSquared(state.linearVelocity) > maxLinearSpeed * maxLinearSpeed {
			state.linearVelocity = MulSV(maxLinearSpeed, Normalize(state.linearVelocity))
		}
	}
}

Body_ApplyAngularImpulse :: proc(bodyId: BodyId, impulse: Vec3, wake: bool) {
	assert(IsValidVec3(impulse))
	assert(Body_IsValid(bodyId))

	world := GetWorld(i32(bodyId.world0))

	if world.recording != nil {
		recArgs := RecArgs_BodyApplyAngularImpulse{bodyId, impulse, wake}
		RecWrite_BodyApplyAngularImpulse(world.recording, &recArgs)
	}

	id := bodyId.index1 - 1
	body := Array_Get(&world.bodies, id)
	assert(body.generation == bodyId.generation)

	if wake && body.setIndex >= i32(SetType.firstSleepingSet) {
		// this will not invalidate body pointer
		WakeBodyWithLock(world, body)
	}

	if body.setIndex == i32(SetType.awakeSet) {
		localIndex := body.localIndex
		set := Array_Get(&world.solverSets, i32(SetType.awakeSet))
		state := Array_Get(&set.bodyStates, localIndex)
		bodySim := Array_Get(&set.bodySims, localIndex)

		localImpulse := InvRotateVector(bodySim.transform.q, impulse)
		localAngularVelocityDelta := MulMV(bodySim.invInertiaLocal, localImpulse)
		state.angularVelocity = Add(
			state.angularVelocity,
			RotateVector(bodySim.transform.q, localAngularVelocityDelta),
		)
	}
}

Body_GetType :: proc(bodyId: BodyId) -> BodyType {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	return body.type
}

// This should follow similar steps as you would get destroying and recreating the body, shapes, and joints.
// Contacts are difficult to preserve because the broad-phase pairs change, so I just destroy them.
// todo with a bit more effort I could support an option to let the body sleep
//
// Revised steps:
// 1 Skip disabled bodies
// 2 Destroy all contacts on the body
// 3 Wake the body
// 4 For all joints attached to the body
//  - wake attached bodies
//  - remove from island
//  - move to static set temporarily
// 5 Change the body type and transfer the body
// 6 If the body was static
//   - create an island for the body
//   Else if the body is becoming static
//   - remove it from the island
// 7 For all joints
//  - if either body is non-static
//    - link into island
//    - transfer to constraint graph
// 8 For all shapes
//  - Destroy proxy in old tree
//  - Create proxy in new tree
// Notes:
// - the implementation below tries to minimize the number of predicates, so some
//   operations may have no effect, such as transferring a joint to the same set
Body_SetType :: proc(bodyId: BodyId, type: BodyType) {
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_BodySetType{bodyId, i32(type)}
		RecWrite_BodySetType(world.recording, &recArgs)
	}

	world.locked = true
	body := GetBodyFullId(world, bodyId)

	originalType := body.type
	if originalType == type {
		world.locked = false
		return
	}

	if type != .staticBody {
		shapeId := body.headShapeId
		for shapeId != NULL_INDEX {
			shape := Array_Get(&world.shapes, shapeId)
			if shape.type == .compoundShape || shape.type == .heightShape {
				// Setting the body type is not supported for bodies with compound shapes
				return
			}

			shapeId = shape.nextShapeId
		}
	}

	// Stage 1: skip disabled bodies
	if body.setIndex == i32(SetType.disabledSet) {
		// Disabled bodies don't change solver sets or islands when they change type.
		body.type = type

		if type == .dynamicBody {
			body.flags |= dynamicFlag
		} else {
			body.flags &~= dynamicFlag
		}

		SyncBodyFlags(world, body)

		// Body type affects the mass properties
		UpdateBodyMassData(world, body)
		world.locked = false
		return
	}

	// Stage 2: destroy all contacts but don't wake bodies (because we don't need to)
	wakeBodies := false
	DestroyBodyContacts(world, body, wakeBodies)

	// Stage 3: wake this body (does nothing if body is static), otherwise it will also wake
	// all bodies in the same sleeping solver set.
	WakeBody(world, body)

	// Stage 4: move joints to temporary storage
	staticSet := Array_Get(&world.solverSets, i32(SetType.staticSet))

	jointKey := body.headJointKey
	for jointKey != NULL_INDEX {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1

		joint := Array_Get(&world.joints, jointId)
		jointKey = joint.edges[edgeIndex].nextKey

		// Joint may be disabled by other body
		if joint.setIndex == i32(SetType.disabledSet) {
			continue
		}

		// Wake attached bodies. The b3WakeBody call above does not wake bodies
		// attached to a static body. But it is necessary because the body may have
		// no joints.
		bodyA := Array_Get(&world.bodies, joint.edges[0].bodyId)
		bodyB := Array_Get(&world.bodies, joint.edges[1].bodyId)
		WakeBody(world, bodyA)
		WakeBody(world, bodyB)

		// Remove joint from island
		UnlinkJoint(world, joint)

		// It is necessary to transfer all joints to the static set
		// so they can be added to the constraint graph below and acquire consistent colors.
		jointSourceSet := Array_Get(&world.solverSets, joint.setIndex)
		TransferJoint(world, staticSet, jointSourceSet, joint)
	}

	// Stage 5: change the body type and transfer body
	body.type = type

	if type == .dynamicBody {
		body.flags |= dynamicFlag
	} else {
		body.flags &~= dynamicFlag
	}

	awakeSet := Array_Get(&world.solverSets, i32(SetType.awakeSet))
	sourceSet := Array_Get(&world.solverSets, body.setIndex)
	targetSet := type == .staticBody ? staticSet : awakeSet

	// Transfer body
	TransferBody(world, targetSet, sourceSet, body)

	// Stage 6: update island participation for the body
	if originalType == .staticBody {
		// Create island for body
		CreateIslandForBody(world, i32(SetType.awakeSet), body)
	} else if type == .staticBody {
		// Remove body from island.
		RemoveBodyFromIsland(world, body)
	}

	// Stage 7: Transfer joints to the target set
	jointKey = body.headJointKey
	for jointKey != NULL_INDEX {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1

		joint := Array_Get(&world.joints, jointId)

		jointKey = joint.edges[edgeIndex].nextKey

		// Joint may be disabled by other body
		if joint.setIndex == i32(SetType.disabledSet) {
			continue
		}

		// All joints were transferred to the static set in an earlier stage
		assert(joint.setIndex == i32(SetType.staticSet))

		bodyA := Array_Get(&world.bodies, joint.edges[0].bodyId)
		bodyB := Array_Get(&world.bodies, joint.edges[1].bodyId)
		assert(bodyA.setIndex == i32(SetType.staticSet) || bodyA.setIndex == i32(SetType.awakeSet))
		assert(bodyB.setIndex == i32(SetType.staticSet) || bodyB.setIndex == i32(SetType.awakeSet))

		if bodyA.type == .dynamicBody || bodyB.type == .dynamicBody {
			TransferJoint(world, awakeSet, staticSet, joint)
		}
	}

	// Recreate shape proxies in broadphase
	transform := GetBodyTransformQuick(world, body)
	shapeId := body.headShapeId
	for shapeId != NULL_INDEX {
		shape := Array_Get(&world.shapes, shapeId)

		// Setting the body type is not supported for bodies with compound shapes
		assert(shape.type != .compoundShape)

		shapeId = shape.nextShapeId
		DestroyShapeProxy(shape, &world.broadPhase)
		forcePairCreation := true
		CreateShapeProxy(world, shape, type, transform, forcePairCreation)
	}

	// Relink all joints
	jointKey = body.headJointKey
	for jointKey != NULL_INDEX {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1

		joint := Array_Get(&world.joints, jointId)
		jointKey = joint.edges[edgeIndex].nextKey

		otherEdgeIndex := edgeIndex ~ 1
		otherBodyId := joint.edges[otherEdgeIndex].bodyId
		otherBody := Array_Get(&world.bodies, otherBodyId)

		if otherBody.setIndex == i32(SetType.disabledSet) {
			continue
		}

		if body.type != .dynamicBody && otherBody.type != .dynamicBody {
			continue
		}

		LinkJoint(world, joint)
	}

	SyncBodyFlags(world, body)

	// Body type affects the mass
	UpdateBodyMassData(world, body)

	ValidateSolverSets(world)
	ValidateIsland(world, body.islandId)

	world.locked = false
}

Body_SetName :: proc(bodyId: BodyId, name: cstring) {
	world := GetWorld(i32(bodyId.world0))

	if world.recording != nil {
		recArgs := RecArgs_BodySetName{bodyId, name}
		RecWrite_BodySetName(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)
	body.nameId = AddName(&world.names, name)
}

Body_GetName :: proc(bodyId: BodyId) -> cstring {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	return FindNameWithDefault(&world.names, body.nameId, "")
}

Body_SetUserData :: proc(bodyId: BodyId, userData: rawptr) {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	body.userData = userData
}

Body_GetUserData :: proc(bodyId: BodyId) -> rawptr {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	return body.userData
}

Body_GetMass :: proc(bodyId: BodyId) -> f32 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	return body.mass
}

Body_GetLocalRotationalInertia :: proc(bodyId: BodyId) -> Matrix3 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	bodySim := GetBodySim(world, body)
	return bodySim.inertia
}

Body_GetInverseMass :: proc(bodyId: BodyId) -> f32 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	sim := GetBodySim(world, body)
	return sim.invMass
}

Body_GetWorldInverseRotationalInertia :: proc(bodyId: BodyId) -> Matrix3 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	sim := GetBodySim(world, body)
	return sim.invInertiaWorld
}

Body_GetLocalCenter :: proc(bodyId: BodyId) -> Vec3 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	bodySim := GetBodySim(world, body)
	return bodySim.localCenter
}

Body_GetWorldCenter :: proc(bodyId: BodyId) -> Pos {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	bodySim := GetBodySim(world, body)
	return bodySim.center
}

Body_SetMassData :: proc(bodyId: BodyId, massData: MassData) {
	assert(IsValidFloat(massData.mass) && massData.mass >= 0.0)
	assert(IsValidMatrix3(massData.inertia))
	assert(IsValidVec3(massData.center))

	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_BodySetMassData{bodyId, massData}
		RecWrite_BodySetMassData(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)
	bodySim := GetBodySim(world, body)

	// Mass is no longer dirty
	body.flags &~= dirtyMass
	SyncBodyFlags(world, body)

	body.mass = massData.mass
	bodySim.localCenter = massData.center

	oldCenter := bodySim.center
	center := TransformWorldPoint(bodySim.transform, massData.center)
	bodySim.center = center
	bodySim.center0 = center
	bodySim.invMass = body.mass > 0.0 ? 1.0 / body.mass : 0.0
	bodySim.inertia = massData.inertia

	// Update center of mass velocity
	state := GetBodyState(world, body)
	if state != nil {
		deltaLinear := Cross(state.angularVelocity, SubPos(bodySim.center, oldCenter))
		state.linearVelocity = Add(state.linearVelocity, deltaLinear)
	}

	det := Det(bodySim.inertia)
	assert(det >= 0.0)

	if det > 0.0 {
		// This call is faster than b3Invert
		bodySim.invInertiaLocal = InvertT(bodySim.inertia)

		rotationMatrix := MakeMatrixFromQuat(bodySim.transform.q)
		bodySim.invInertiaWorld = MulMM(MulMM(rotationMatrix, bodySim.invInertiaLocal), Transpose(rotationMatrix))
	} else {
		bodySim.invInertiaLocal = Mat3_zero
		bodySim.invInertiaWorld = Mat3_zero
	}

	// Apply fixed rotation
	if (bodySim.flags & fixedRotation) == fixedRotation {
		bodySim.inertia = Mat3_zero
		bodySim.invInertiaLocal = Mat3_zero
		bodySim.invInertiaWorld = Mat3_zero
	}

	UpdateIsotropicInertia(world, body, bodySim)

	// Update extents using supplied mass center.
	bodySim.minExtent = HUGE
	bodySim.maxExtent = Vec3_zero
	shapeId := body.headShapeId
	for shapeId != NULL_INDEX {
		s := Array_Get(&world.shapes, shapeId)
		extent := ComputeShapeExtent(s, massData.center)
		bodySim.minExtent = MinFloat(bodySim.minExtent, extent.minExtent)
		bodySim.maxExtent = Max(bodySim.maxExtent, extent.maxExtent)
		shapeId = s.nextShapeId
	}

	// When the center of mass changes, any cached contact manifold becomes invalid.
	edgeKey := body.headContactKey
	for edgeKey != NULL_INDEX {
		contactId := edgeKey >> 1
		edgeIndex := edgeKey & 1

		contact := Array_Get(&world.contacts, contactId)
		contact.flags &~= relativeTransformValid

		edgeKey = contact.edges[edgeIndex].nextKey
	}
}

Body_GetMassData :: proc(bodyId: BodyId) -> MassData {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	bodySim := GetBodySim(world, body)
	massData := MassData{body.mass, bodySim.localCenter, bodySim.inertia}
	return massData
}

Body_ApplyMassFromShapes :: proc(bodyId: BodyId) {
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_BodyApplyMassFromShapes{bodyId}
		RecWrite_BodyApplyMassFromShapes(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)
	UpdateBodyMassData(world, body)
}

Body_SetLinearDamping :: proc(bodyId: BodyId, linearDamping: f32) {
	assert(IsValidFloat(linearDamping) && linearDamping >= 0.0)

	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_BodySetLinearDamping{bodyId, linearDamping}
		RecWrite_BodySetLinearDamping(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)
	bodySim := GetBodySim(world, body)
	bodySim.linearDamping = linearDamping
}

Body_GetLinearDamping :: proc(bodyId: BodyId) -> f32 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	bodySim := GetBodySim(world, body)
	return bodySim.linearDamping
}

Body_SetAngularDamping :: proc(bodyId: BodyId, angularDamping: f32) {
	assert(IsValidFloat(angularDamping) && angularDamping >= 0.0)

	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_BodySetAngularDamping{bodyId, angularDamping}
		RecWrite_BodySetAngularDamping(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)
	bodySim := GetBodySim(world, body)
	bodySim.angularDamping = angularDamping
}

Body_GetAngularDamping :: proc(bodyId: BodyId) -> f32 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	bodySim := GetBodySim(world, body)
	return bodySim.angularDamping
}

Body_SetGravityScale :: proc(bodyId: BodyId, gravityScale: f32) {
	assert(Body_IsValid(bodyId))
	assert(IsValidFloat(gravityScale))

	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_BodySetGravityScale{bodyId, gravityScale}
		RecWrite_BodySetGravityScale(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)
	bodySim := GetBodySim(world, body)
	bodySim.gravityScale = gravityScale
}

Body_GetGravityScale :: proc(bodyId: BodyId) -> f32 {
	assert(Body_IsValid(bodyId))
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	bodySim := GetBodySim(world, body)
	return bodySim.gravityScale
}

Body_IsAwake :: proc(bodyId: BodyId) -> bool {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	return body.setIndex == i32(SetType.awakeSet)
}

Body_SetAwake :: proc(bodyId: BodyId, awake: bool) {
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_BodySetAwake{bodyId, awake}
		RecWrite_BodySetAwake(world.recording, &recArgs)
	}

	world.locked = true

	body := GetBodyFullId(world, bodyId)

	if awake && body.setIndex >= i32(SetType.firstSleepingSet) {
		WakeBody(world, body)
	} else if awake == false && body.setIndex == i32(SetType.awakeSet) {
		island := Array_Get(&world.islands, body.islandId)
		if island.constraintRemoveCount > 0 {
			// Must split the island before sleeping. This is expensive.
			SplitIsland(world, body.islandId)
		}

		TrySleepIsland(world, body.islandId)
	}

	world.locked = false
}

Body_IsEnabled :: proc(bodyId: BodyId) -> bool {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	return body.setIndex != i32(SetType.disabledSet)
}

Body_IsSleepEnabled :: proc(bodyId: BodyId) -> bool {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	return (body.flags & enableSleep) == enableSleep
}

Body_SetSleepThreshold :: proc(bodyId: BodyId, sleepThreshold: f32) {
	world := GetWorld(i32(bodyId.world0))

	if world.recording != nil {
		recArgs := RecArgs_BodySetSleepThreshold{bodyId, sleepThreshold}
		RecWrite_BodySetSleepThreshold(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)
	body.sleepThreshold = sleepThreshold
}

Body_GetSleepThreshold :: proc(bodyId: BodyId) -> f32 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	return body.sleepThreshold
}

Body_SetSafetyFactor :: proc(bodyId: BodyId, safetyFactor: f32) {
	assert(IsValidFloat(safetyFactor) && safetyFactor >= 0.0)

	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_BodySetSafetyFactor{bodyId, safetyFactor}
		RecWrite_BodySetSafetyFactor(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)
	body.safetyFactor = safetyFactor
}

Body_GetSafetyFactor :: proc(bodyId: BodyId) -> f32 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	return body.safetyFactor
}

Body_EnableSleep :: proc(bodyId: BodyId, enableSleep_: bool) {
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_BodyEnableSleep{bodyId, enableSleep_}
		RecWrite_BodyEnableSleep(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)

	flag := (body.flags & enableSleep) == enableSleep
	if enableSleep_ == flag {
		return
	}

	world.locked = true

	body.flags = enableSleep_ ? body.flags | enableSleep : body.flags &~ enableSleep
	SyncBodyFlags(world, body)

	if enableSleep_ == false {
		WakeBody(world, body)
	}

	world.locked = false
}

// Disabling a body requires a lot of detailed bookkeeping, but it is a valuable feature.
// The most challenging aspect that joints may connect to bodies that are not disabled.
Body_Disable :: proc(bodyId: BodyId) {
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_BodyDisable{bodyId}
		RecWrite_BodyDisable(world.recording, &recArgs)
	}

	world.locked = true

	body := GetBodyFullId(world, bodyId)
	if body.setIndex == i32(SetType.disabledSet) {
		world.locked = false
		return
	}

	// Destroy contacts and wake bodies touching this body. This avoid floating bodies.
	// This is necessary even for static bodies.
	wakeBodies := true
	DestroyBodyContacts(world, body, wakeBodies)

	// The current solver set of the body
	set := Array_Get(&world.solverSets, body.setIndex)

	// Disabled bodies and connected joints are moved to the disabled set
	disabledSet := Array_Get(&world.solverSets, i32(SetType.disabledSet))

	// Unlink joints and transfer them to the disabled set
	jointKey := body.headJointKey
	for jointKey != NULL_INDEX {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1

		joint := Array_Get(&world.joints, jointId)
		jointKey = joint.edges[edgeIndex].nextKey

		// joint may already be disabled by other body
		if joint.setIndex == i32(SetType.disabledSet) {
			continue
		}

		assert(
			joint.setIndex == set.setIndex ||
			set.setIndex == i32(SetType.staticSet) ||
			joint.setIndex == i32(SetType.staticSet),
		)

		// Remove joint from island
		UnlinkJoint(world, joint)

		// Transfer joint to disabled set
		jointSet := Array_Get(&world.solverSets, joint.setIndex)
		TransferJoint(world, disabledSet, jointSet, joint)
	}

	// Remove shapes from broad-phase
	shapeId := body.headShapeId
	for shapeId != NULL_INDEX {
		shape := Array_Get(&world.shapes, shapeId)
		shapeId = shape.nextShapeId
		DestroyShapeProxy(shape, &world.broadPhase)
	}

	// Disabled bodies are not in an island. If the island becomes empty it will be destroyed.
	RemoveBodyFromIsland(world, body)

	// Transfer body sim
	TransferBody(world, disabledSet, set, body)

	ValidateConnectivity(world)
	ValidateSolverSets(world)

	world.locked = false
}

Body_Enable :: proc(bodyId: BodyId) {
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_BodyEnable{bodyId}
		RecWrite_BodyEnable(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)
	if body.setIndex != i32(SetType.disabledSet) {
		return
	}

	disabledSet := Array_Get(&world.solverSets, i32(SetType.disabledSet))
	setId := body.type == .staticBody ? i32(SetType.staticSet) : i32(SetType.awakeSet)
	targetSet := Array_Get(&world.solverSets, setId)

	TransferBody(world, targetSet, disabledSet, body)

	transform := GetBodyTransformQuick(world, body)

	// Add shapes to broad-phase
	proxyType := body.type
	forcePairCreation := true
	shapeId := body.headShapeId
	for shapeId != NULL_INDEX {
		shape := Array_Get(&world.shapes, shapeId)
		shapeId = shape.nextShapeId

		CreateShapeProxy(world, shape, proxyType, transform, forcePairCreation)
	}

	if setId != i32(SetType.staticSet) {
		CreateIslandForBody(world, setId, body)
	}

	// Transfer joints. If the other body is disabled, don't transfer.
	// If the other body is sleeping, wake it.
	jointKey := body.headJointKey
	for jointKey != NULL_INDEX {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1

		joint := Array_Get(&world.joints, jointId)
		assert(joint.setIndex == i32(SetType.disabledSet))
		assert(joint.islandId == NULL_INDEX)

		jointKey = joint.edges[edgeIndex].nextKey

		bodyA := Array_Get(&world.bodies, joint.edges[0].bodyId)
		bodyB := Array_Get(&world.bodies, joint.edges[1].bodyId)

		if bodyA.setIndex == i32(SetType.disabledSet) || bodyB.setIndex == i32(SetType.disabledSet) {
			// one body is still disabled
			continue
		}

		// Transfer joint first
		jointSetId: i32
		if bodyA.type != .dynamicBody && bodyB.type != .dynamicBody {
			jointSetId = i32(SetType.staticSet)
		} else if bodyA.setIndex == i32(SetType.staticSet) {
			jointSetId = bodyB.setIndex
		} else {
			jointSetId = bodyA.setIndex
		}

		jointSet := Array_Get(&world.solverSets, jointSetId)
		TransferJoint(world, jointSet, disabledSet, joint)

		// Now that the joint is in the correct set, I can link the joint in the island.
		if jointSetId != i32(SetType.staticSet) {
			LinkJoint(world, joint)
		}
	}

	ValidateSolverSets(world)
}

Body_SetMotionLocks :: proc(bodyId: BodyId, locks: MotionLocks) {
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_BodySetMotionLocks{bodyId, locks}
		RecWrite_BodySetMotionLocks(world.recording, &recArgs)
	}

	newLocks: u32 = 0
	newLocks |= locks.linearX ? lockLinearX : 0
	newLocks |= locks.linearY ? lockLinearY : 0
	newLocks |= locks.linearZ ? lockLinearZ : 0
	newLocks |= locks.angularX ? lockAngularX : 0
	newLocks |= locks.angularY ? lockAngularY : 0
	newLocks |= locks.angularZ ? lockAngularZ : 0

	body := GetBodyFullId(world, bodyId)
	if (body.flags & allLocks) == newLocks {
		return
	}

	fixedRotation1 := (body.flags & fixedRotation) == fixedRotation
	fixedRotation2 := (newLocks & fixedRotation) == fixedRotation

	body.flags &~= allLocks
	body.flags |= newLocks

	SyncBodyFlags(world, body)

	state := GetBodyState(world, body)

	if state != nil {
		if locks.linearX {
			state.linearVelocity.x = 0.0
		}

		if locks.linearY {
			state.linearVelocity.y = 0.0
		}

		if locks.linearZ {
			state.linearVelocity.z = 0.0
		}

		if locks.angularX {
			state.angularVelocity.x = 0.0
		}

		if locks.angularY {
			state.angularVelocity.y = 0.0
		}

		if locks.angularZ {
			state.angularVelocity.z = 0.0
		}
	}

	if fixedRotation1 != fixedRotation2 {
		UpdateBodyMassData(world, body)
	}
}

Body_GetMotionLocks :: proc(bodyId: BodyId) -> MotionLocks {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)

	locks: MotionLocks
	locks.linearX = (body.flags & lockLinearX) != 0
	locks.linearY = (body.flags & lockLinearY) != 0
	locks.linearZ = (body.flags & lockLinearZ) != 0
	locks.angularX = (body.flags & lockAngularX) != 0
	locks.angularY = (body.flags & lockAngularY) != 0
	locks.angularZ = (body.flags & lockAngularZ) != 0
	return locks
}

Body_SetBullet :: proc(bodyId: BodyId, flag: bool) {
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_BodySetBullet{bodyId, flag}
		RecWrite_BodySetBullet(world.recording, &recArgs)
	}

	newFlag: u32 = flag ? isBullet : 0

	body := GetBodyFullId(world, bodyId)
	if (body.flags & isBullet) == newFlag {
		return
	}

	body.flags &~= isBullet
	body.flags |= newFlag

	SyncBodyFlags(world, body)
}

Body_IsBullet :: proc(bodyId: BodyId) -> bool {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	return (body.flags & isBullet) != 0
}

Body_AllowFastRotation :: proc(bodyId: BodyId, flag: bool) {
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_BodyAllowFastRotation{bodyId, flag}
		RecWrite_BodyAllowFastRotation(world.recording, &recArgs)
	}

	newFlag: u32 = flag ? allowFastRotation : 0

	body := GetBodyFullId(world, bodyId)
	if (body.flags & allowFastRotation) == newFlag {
		return
	}

	body.flags &~= allowFastRotation
	body.flags |= newFlag

	SyncBodyFlags(world, body)
}

Body_IsFastRotationAllowed :: proc(bodyId: BodyId) -> bool {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	return (body.flags & allowFastRotation) != 0
}

Body_EnableContactRecycling :: proc(bodyId: BodyId, flag: bool) {
	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_BodyEnableContactRecycling{bodyId, flag}
		RecWrite_BodyEnableContactRecycling(world.recording, &recArgs)
	}

	newFlag: u32 = flag ? enableContactRecycling : 0

	body := GetBodyFullId(world, bodyId)
	if (body.flags & enableContactRecycling) == newFlag {
		return
	}

	body.flags &~= enableContactRecycling
	body.flags |= newFlag

	SyncBodyFlags(world, body)
}

Body_IsContactRecyclingEnabled :: proc(bodyId: BodyId) -> bool {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	return (body.flags & enableContactRecycling) != 0
}

Body_EnableHitEvents :: proc(bodyId: BodyId, flag: bool) {
	world := GetWorld(i32(bodyId.world0))

	if world.recording != nil {
		recArgs := RecArgs_BodyEnableHitEvents{bodyId, flag}
		RecWrite_BodyEnableHitEvents(world.recording, &recArgs)
	}

	body := GetBodyFullId(world, bodyId)
	shapeId := body.headShapeId
	for shapeId != NULL_INDEX {
		shape := Array_Get(&world.shapes, shapeId)
		shape.flags = flag ? shape.flags | enableHitEvents : shape.flags &~ enableHitEvents
		shapeId = shape.nextShapeId
	}
}

Body_GetWorld :: proc(bodyId: BodyId) -> WorldId {
	world := GetWorld(i32(bodyId.world0))
	return WorldId{u16(bodyId.world0 + 1), world.generation}
}

Body_GetShapeCount :: proc(bodyId: BodyId) -> i32 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	return body.shapeCount
}

Body_GetShapes :: proc(bodyId: BodyId, shapeArray: [^]ShapeId, capacity: i32) -> i32 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	shapeId := body.headShapeId
	shapeCount: i32 = 0
	for shapeId != NULL_INDEX && shapeCount < capacity {
		shape := Array_Get(&world.shapes, shapeId)
		id := ShapeId{shape.id + 1, bodyId.world0, shape.generation}
		shapeArray[shapeCount] = id
		shapeCount += 1

		shapeId = shape.nextShapeId
	}

	return shapeCount
}

Body_GetJointCount :: proc(bodyId: BodyId) -> i32 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	return body.jointCount
}

Body_GetJoints :: proc(bodyId: BodyId, jointArray: [^]JointId, capacity: i32) -> i32 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	jointKey := body.headJointKey

	jointCount: i32 = 0
	for jointKey != NULL_INDEX && jointCount < capacity {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1

		joint := Array_Get(&world.joints, jointId)

		id := JointId{jointId + 1, bodyId.world0, joint.generation}
		jointArray[jointCount] = id
		jointCount += 1

		jointKey = joint.edges[edgeIndex].nextKey
	}

	return jointCount
}

ShouldBodiesCollide :: proc(world: ^World, bodyA: ^Body, bodyB: ^Body) -> bool {
	if bodyA.type != .dynamicBody && bodyB.type != .dynamicBody {
		return false
	}

	jointKey: i32
	otherBodyId: i32
	if bodyA.jointCount < bodyB.jointCount {
		jointKey = bodyA.headJointKey
		otherBodyId = bodyB.id
	} else {
		jointKey = bodyB.headJointKey
		otherBodyId = bodyA.id
	}

	for jointKey != NULL_INDEX {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1
		otherEdgeIndex := edgeIndex ~ 1

		joint := Array_Get(&world.joints, jointId)
		if joint.collideConnected == false && joint.edges[otherEdgeIndex].bodyId == otherBodyId {
			return false
		}

		jointKey = joint.edges[edgeIndex].nextKey
	}

	return true
}

Body_GetMinExtent :: proc(bodyId: BodyId) -> f32 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	bodySim := GetBodySim(world, body)
	return bodySim.minExtent
}

Body_GetMaxExtent :: proc(bodyId: BodyId) -> Vec3 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	bodySim := GetBodySim(world, body)
	return bodySim.maxExtent
}

Body_GetMaxExtentOrigin :: proc(bodyId: BodyId) -> Vec3 {
	world := GetWorld(i32(bodyId.world0))
	body := GetBodyFullId(world, bodyId)
	bodySim := GetBodySim(world, body)
	return Add(bodySim.maxExtent, Abs(bodySim.localCenter))
}
