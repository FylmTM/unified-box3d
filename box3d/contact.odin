// Ported from box3d src/contact.c and src/contact.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "core:mem"

// contact.h

FORCE_GHOST_COLLISIONS :: false

ContactCache :: struct #raw_union {
	satCache:     SATCache,
	simplexCache: SimplexCache,
}

TriangleCache :: struct {
	triangleIndex: i32,
	cache:         ContactCache,
}

// b3ContactFlags
// Set when the solid shapes are touching.
contactTouchingFlag :: 0x00000001

// Contact has a hit event
contactHitEventFlag :: 0x00000002

// This contact wants contact events
contactEnableContactEvents :: 0x00000004

// This is contact is between a dynamic and static body
contactStaticFlag :: 0x00000008

contactRecycleFlag :: 0x00000010

// Set when the shapes are touching
simTouchingFlag :: 0x00010000

// This contact no longer has overlapping AABBs
simDisjoint :: 0x00020000

// This contact started touching
simStartedTouching :: 0x00040000

// This contact stopped touching
simStoppedTouching :: 0x00080000

// This contact has a hit event
simEnableHitEvent :: 0x00100000

// This contact wants pre-solve events
simEnablePreSolveEvents :: 0x00200000

// This is a mesh contact
simMeshContact :: 0x00400000

// Relative transform is cached for contact recycling
relativeTransformValid :: 0x00800000

// Enable speculative contact points
enableSpeculativePoints :: 0x01000000

// A contact edge is used to connect bodies and contacts together
// in a contact graph where each body is a node and each contact
// is an edge. A contact edge belongs to a doubly linked list
// maintained in each attached body. Each contact has two contact
// edges, one for each attached body.
ContactEdge :: struct {
	bodyId:  i32,
	prevKey: i32,
	nextKey: i32,
}

MeshContact :: struct {
	triangleCache: Array(TriangleCache),
	queryBounds:   AABB,
}

ConvexContact :: struct {
	cache: ContactCache,
}

// Represents the persistent interaction between two shapes
Contact :: struct {
	// index of simulation set stored in b3World
	// B3_NULL_INDEX when slot is free
	setIndex:           i32,

	// index into the constraint graph color array
	// B3_NULL_INDEX for non-touching or sleeping contacts
	// B3_NULL_INDEX when slot is free
	colorIndex:         i32,

	// contact index within set or graph color
	// B3_NULL_INDEX when slot is free
	localIndex:         i32,
	edges:              [2]ContactEdge,
	shapeIdA:           i32,
	shapeIdB:           i32,
	childIndex:         i32,

	// A contact only belongs to an island if touching, otherwise B3_NULL_INDEX.
	islandId:           i32,

	// Index into the island's contacts array for O(1) swap-removal.
	// B3_NULL_INDEX when not in an island.
	islandIndex:        i32,

	// Back index into b3World::contacts
	contactId:          i32,

	// Encoded body sim indices to avoid a cache miss accessing the body in the narrow phase.
	encodedBodySimA:    i32,
	encodedBodySimB:    i32,

	// b3ContactFlags
	flags:              u32,

	// todo embed single manifold with ability to allocate overfloat, use union.
	manifolds:          [^]Manifold,
	manifoldCount:      i32,

	// Cache for contact recycling.
	cachedRotationA:    Quat,
	cachedRotationB:    Quat,
	cachedRelativePose: Transform,

	// Mixed friction and restitution
	friction:           f32,

	// Usage determined by b3_simMeshContact in simFlags
	using _:            struct #raw_union {
		convexContact: ConvexContact,
		meshContact:   MeshContact,
	},
	restitution:        f32,
	rollingResistance:  f32,
	tangentVelocity:    Vec3,

	// This is monotonically advanced when a contact is allocated in this slot
	// Used to check for invalid b3ContactId
	generation:         u32,
}

ContactSpec :: struct {
	contactId:     i32,

	// Start of the global manifold constraint array
	manifoldStart: i32,
	manifoldCount: u16,
}

// unified: ComputeMeshManifolds is declared in contact.h and defined in mesh_contact.c (its port lives there).

// contact.c

// Contacts and determinism
// A deterministic simulation requires contacts to exist in the same order in b3Island no matter the thread count.
// The order must reproduce from run to run. This is necessary because the Gauss-Seidel constraint solver is order dependent.
//
// Creation:
// - Contacts are created using results from b3UpdateBroadPhasePairs
// - These results are ordered according to the order of the broad-phase move array
// - The move array is ordered according to the shape creation order using a bitset.
// - The island/shape/body order is determined by creation order
// - Logically contacts are only created for awake bodies, so they are immediately added to the awake contact array (serially)
//
// Island linking:
// - The awake contact array is built from the body-contact graph for all awake bodies in awake islands.
// - Awake contacts are solved in parallel and they generate contact state changes.
// - These state changes may link islands together using union find.
// - The state changes are ordered using a bit array that encompasses all contacts
// - As long as contacts are created in deterministic order, island link order is deterministic.
// - This keeps the order of contacts in islands deterministic

// Manifold functions should compute important results in local space to improve precision. However, this
// interface function takes two world transforms instead of a relative transform for these reasons:
//
// First:
// The anchors need to be computed relative to the shape origin in world space. This is necessary so the
// solver does not need to access static body transforms. Not even in constraint preparation. This approach
// has world space vectors yet retains precision.
//
// Second:
// b3ManifoldPoint::point is very useful for debugging and it is in world space.
//
// Third:
// The user may call the manifold functions directly and they should be easy to use and have easy to use
// results.
// typedef b3Manifold b3ManifoldFcn( const b3Shape* shapeA, b3Transform xfA, const b3Shape* shapeB, b3Transform xfB,
//								  b3ContactCache* cache );

@(private = "file")
GetContactFullId :: proc(world: ^World, contactId: ContactId) -> ^Contact {
	id := contactId.index1 - 1
	contact := Array_Get(&world.contacts, id)
	assert(contact.contactId == id && contact.generation == contactId.generation)
	return contact
}

Contact_GetData :: proc(contactId: ContactId) -> ContactData {
	world := GetWorld(i32(contactId.world0))
	contact := GetContactFullId(world, contactId)

	shapeA := Array_Get(&world.shapes, contact.shapeIdA)
	shapeB := Array_Get(&world.shapes, contact.shapeIdB)

	data := ContactData{}
	data.contactId = contactId
	data.shapeIdA = ShapeId {
		index1     = shapeA.id + 1,
		world0     = contactId.world0,
		generation = shapeA.generation,
	}
	data.shapeIdB = ShapeId {
		index1     = shapeB.id + 1,
		world0     = contactId.world0,
		generation = shapeB.generation,
	}

	if contact.manifoldCount > 0 {
		data.manifolds = contact.manifolds
		data.manifoldCount = contact.manifoldCount
	} else {
		data.manifolds = nil
		data.manifoldCount = 0
	}

	return data
}

ContactRegister :: struct {
	// b3ManifoldFcn* fcn;
	supported: bool,
	primary:   bool,
}

@(private = "file")
s_registers: [ShapeType.shapeTypeCount][ShapeType.shapeTypeCount]ContactRegister
@(private = "file")
s_initialized := false

@(private = "file")
AddType :: proc(type1: ShapeType, type2: ShapeType) {
	assert(0 <= i32(type1) && type1 < .shapeTypeCount)
	assert(0 <= i32(type2) && type2 < .shapeTypeCount)

	s_registers[type1][type2].supported = true
	s_registers[type1][type2].primary = true

	if type1 != type2 {
		s_registers[type2][type1].supported = true
		s_registers[type2][type1].primary = false
	}
}

InitializeContactRegisters :: proc() {
	if s_initialized == false {
		AddType(.sphereShape, .sphereShape)
		AddType(.capsuleShape, .sphereShape)
		AddType(.capsuleShape, .capsuleShape)
		AddType(.compoundShape, .sphereShape)
		AddType(.compoundShape, .capsuleShape)
		AddType(.compoundShape, .hullShape)
		AddType(.hullShape, .sphereShape)
		AddType(.hullShape, .capsuleShape)
		AddType(.hullShape, .hullShape)
		AddType(.meshShape, .sphereShape)
		AddType(.meshShape, .capsuleShape)
		AddType(.meshShape, .hullShape)
		AddType(.heightShape, .sphereShape)
		AddType(.heightShape, .capsuleShape)
		AddType(.heightShape, .hullShape)
		s_initialized = true
	}
}

CreateContact :: proc(world: ^World, shapeA: ^Shape, shapeB: ^Shape, childIndex: i32) {
	typeA := shapeA.type
	typeB := shapeB.type

	assert(0 <= i32(typeA) && typeA < .shapeTypeCount)
	assert(0 <= i32(typeB) && typeB < .shapeTypeCount)

	if s_registers[typeA][typeB].supported == false {
		// For example, no mesh vs mesh collision
		return
	}

	if s_registers[typeA][typeB].primary == false {
		// flip order
		CreateContact(world, shapeB, shapeA, childIndex)
		return
	}

	bodyA := Array_Get(&world.bodies, shapeA.bodyId)
	bodyB := Array_Get(&world.bodies, shapeB.bodyId)

	assert(bodyA.setIndex != i32(SetType.disabledSet) && bodyB.setIndex != i32(SetType.disabledSet))
	assert(bodyA.setIndex != i32(SetType.staticSet) || bodyB.setIndex != i32(SetType.staticSet))

	setIndex: i32
	if bodyA.setIndex == i32(SetType.awakeSet) || bodyB.setIndex == i32(SetType.awakeSet) {
		setIndex = i32(SetType.awakeSet)
	} else {
		// sleeping and non-touching contacts live in the disabled set
		// later if this set is found to be touching then the sleeping
		// islands will be linked and the contact moved to the merged island

		// This is possible if a shape moves slightly then falls asleep
		setIndex = i32(SetType.disabledSet)
	}

	set := Array_Get(&world.solverSets, setIndex)

	// Create contact key and contact
	contactId := AllocId(&world.contactIdPool)
	if contactId == world.contacts.count {
		emptyContact := Contact{}
		Array_Push(&world.contacts, emptyContact)
	}

	shapeIdA := shapeA.id
	shapeIdB := shapeB.id

	contact := Array_Get(&world.contacts, contactId)
	generation := contact.generation
	contact^ = Contact{}
	contact.contactId = contactId
	contact.generation = generation + 1
	contact.setIndex = setIndex
	contact.colorIndex = NULL_INDEX
	contact.localIndex = set.contactIndices.count
	contact.islandId = NULL_INDEX
	contact.islandIndex = NULL_INDEX
	contact.shapeIdA = shapeIdA
	contact.shapeIdB = shapeIdB
	contact.childIndex = childIndex
	contact.encodedBodySimA = EncodeBodySimIndex(bodyA)
	contact.encodedBodySimB = EncodeBodySimIndex(bodyB)

	if shapeA.type == .meshShape || shapeA.type == .heightShape {
		contact.flags |= simMeshContact
	} else if shapeA.type == .compoundShape {
		child := GetCompoundChild(shapeA.compound, childIndex)
		if child.type == .meshShape {
			contact.flags |= simMeshContact
		}
	}

	// todo impose these restrictions to make life easier
	assert(shapeB.type == .sphereShape || shapeB.type == .capsuleShape || shapeB.type == .hullShape)
	// B3_ASSERT( bodyB->type != b3_staticBody );

	// Is either body static?
	// Note: it is possible to have a dynamic mesh collide with a static convex shape. Maybe I should disallow this.
	if bodyA.type == .staticBody || bodyB.type == .staticBody {
		contact.flags |= contactStaticFlag
	}

	assert(shapeA.sensorIndex == NULL_INDEX && shapeB.sensorIndex == NULL_INDEX)

	if (shapeA.flags & enableContactEvents) != 0 || (shapeB.flags & enableContactEvents) != 0 {
		contact.flags |= contactEnableContactEvents
	}

	if (shapeA.flags & enableSpeculative) != 0 && (shapeB.flags & enableSpeculative) != 0 {
		contact.flags |= enableSpeculativePoints
	}

	// Disabling speculative collision also disables recycling.
	if (bodyA.flags & enableContactRecycling) != 0 &&
	   (bodyB.flags & enableContactRecycling) != 0 &&
	   (contact.flags & enableSpeculativePoints) != 0 {
		contact.flags |= contactRecycleFlag
	}

	// Connect to body A
	{
		contact.edges[0].bodyId = shapeA.bodyId
		contact.edges[0].prevKey = NULL_INDEX
		contact.edges[0].nextKey = bodyA.headContactKey

		keyA := (contactId << 1) | 0
		headContactKey := bodyA.headContactKey
		if headContactKey != NULL_INDEX {
			headContact := Array_Get(&world.contacts, headContactKey >> 1)
			headContact.edges[headContactKey & 1].prevKey = keyA
		}
		bodyA.headContactKey = keyA
		bodyA.contactCount += 1
	}

	// Connect to body B
	{
		contact.edges[1].bodyId = shapeB.bodyId
		contact.edges[1].prevKey = NULL_INDEX
		contact.edges[1].nextKey = bodyB.headContactKey

		keyB := (contactId << 1) | 1
		headContactKey := bodyB.headContactKey
		if bodyB.headContactKey != NULL_INDEX {
			headContact := Array_Get(&world.contacts, headContactKey >> 1)
			headContact.edges[headContactKey & 1].prevKey = keyB
		}
		bodyB.headContactKey = keyB
		bodyB.contactCount += 1
	}

	// Add to pair set for fast lookup
	pairKey := ShapePairKey(shapeIdA, shapeIdB, childIndex)
	AddKey(&world.broadPhase.pairSet, pairKey)

	// Contacts are created as non-touching. Later if they are found to be touching
	// they will link islands and be moved into the constraint graph.
	Array_Push(&set.contactIndices, contactId)

	radiusA: f32 = 0.0
	if typeA == .sphereShape {
		radiusA = shapeA.sphere.radius
	} else if typeA == .capsuleShape {
		radiusA = shapeA.capsule.radius
	}

	radiusB: f32 = 0.0
	if typeB == .sphereShape {
		radiusB = shapeB.sphere.radius
	} else if typeB == .capsuleShape {
		radiusB = shapeB.capsule.radius
	}

	maxRadius := MaxFloat(radiusA, radiusB)

	// Assuming the rolling resistance doesn't change
	contact.rollingResistance =
		MaxFloat(GetShapeMaterials(shapeA)[0].rollingResistance, GetShapeMaterials(shapeB)[0].rollingResistance) *
		maxRadius

	if (shapeA.flags & enablePreSolveEvents) != 0 || (shapeB.flags & enablePreSolveEvents) != 0 {
		contact.flags |= simEnablePreSolveEvents
	}
}

// A contact is destroyed when:
// - broad-phase proxies stop overlapping
// - a body is destroyed
// - a body is disabled
// - a body changes type from dynamic to kinematic or static
// - a shape is destroyed
// - contact filtering is modified
DestroyContact :: proc(world: ^World, contact: ^Contact, wakeBodies: bool) {
	// Remove pair from set
	pairKey := ShapePairKey(contact.shapeIdA, contact.shapeIdB, contact.childIndex)
	RemoveKey(&world.broadPhase.pairSet, pairKey)

	FreeManifolds(world, contact.manifolds, contact.manifoldCount)
	contact.manifolds = nil
	contact.manifoldCount = 0

	edgeA := &contact.edges[0]
	edgeB := &contact.edges[1]

	bodyIdA := edgeA.bodyId
	bodyIdB := edgeB.bodyId
	bodyA := Array_Get(&world.bodies, bodyIdA)
	bodyB := Array_Get(&world.bodies, bodyIdB)

	flags := contact.flags
	touching := (flags & contactTouchingFlag) != 0

	// End touch event
	if touching && (flags & contactEnableContactEvents) != 0 {
		worldId := world.worldId
		shapeA := Array_Get(&world.shapes, contact.shapeIdA)
		shapeB := Array_Get(&world.shapes, contact.shapeIdB)
		shapeIdA := ShapeId{shapeA.id + 1, worldId, shapeA.generation}
		shapeIdB := ShapeId{shapeB.id + 1, worldId, shapeB.generation}

		contactId := ContactId {
			index1     = contact.contactId + 1,
			world0     = world.worldId,
			padding    = 0,
			generation = contact.generation,
		}

		event := ContactEndTouchEvent {
			shapeIdA  = shapeIdA,
			shapeIdB  = shapeIdB,
			contactId = contactId,
		}

		Array_Push(&world.contactEndEvents[world.endEventArrayIndex], event)
	}

	// Remove from body A
	if edgeA.prevKey != NULL_INDEX {
		prevContact := Array_Get(&world.contacts, edgeA.prevKey >> 1)
		prevEdge := &prevContact.edges[edgeA.prevKey & 1]
		prevEdge.nextKey = edgeA.nextKey
	}

	if edgeA.nextKey != NULL_INDEX {
		nextContact := Array_Get(&world.contacts, edgeA.nextKey >> 1)
		nextEdge := &nextContact.edges[edgeA.nextKey & 1]
		nextEdge.prevKey = edgeA.prevKey
	}

	contactId := contact.contactId

	edgeKeyA := (contactId << 1) | 0
	if bodyA.headContactKey == edgeKeyA {
		bodyA.headContactKey = edgeA.nextKey
	}

	bodyA.contactCount -= 1

	// Remove from body B
	if edgeB.prevKey != NULL_INDEX {
		prevContact := Array_Get(&world.contacts, edgeB.prevKey >> 1)
		prevEdge := &prevContact.edges[edgeB.prevKey & 1]
		prevEdge.nextKey = edgeB.nextKey
	}

	if edgeB.nextKey != NULL_INDEX {
		nextContact := Array_Get(&world.contacts, edgeB.nextKey >> 1)
		nextEdge := &nextContact.edges[edgeB.nextKey & 1]
		nextEdge.prevKey = edgeB.prevKey
	}

	edgeKeyB := (contactId << 1) | 1
	if bodyB.headContactKey == edgeKeyB {
		bodyB.headContactKey = edgeB.nextKey
	}

	bodyB.contactCount -= 1

	if (contact.flags & simMeshContact) != 0 {
		Array_Destroy(&contact.meshContact.triangleCache)
	}

	// Remove contact from the array that owns it
	if contact.islandId != NULL_INDEX {
		UnlinkContact(world, contact)
	}

	if contact.colorIndex != NULL_INDEX {
		// contact is an active constraint
		assert(contact.setIndex == i32(SetType.awakeSet))
		meshContact := (contact.flags & simMeshContact) != 0
		RemoveContactFromGraph(world, bodyIdA, bodyIdB, contact.colorIndex, contact.localIndex, meshContact)
	} else {
		// contact is non-touching or is sleeping or is a sensor
		assert(contact.setIndex != i32(SetType.awakeSet) || (contact.flags & contactTouchingFlag) == 0)
		set := Array_Get(&world.solverSets, contact.setIndex)

		localIndex := contact.localIndex
		movedIndex := Array_RemoveSwap(&set.contactIndices, localIndex)
		if movedIndex != NULL_INDEX {
			movedContactIndex := set.contactIndices.data[localIndex]
			movedContact := Array_Get(&world.contacts, movedContactIndex)
			movedContact.localIndex = localIndex
		}
	}

	// Free contact and id (preserve generation)
	contact.contactId = NULL_INDEX
	contact.setIndex = NULL_INDEX
	contact.colorIndex = NULL_INDEX
	contact.localIndex = NULL_INDEX
	FreeId(&world.contactIdPool, contactId)

	if wakeBodies && touching {
		WakeBody(world, bodyA)
		WakeBody(world, bodyB)
	}
}

@(private = "file")
ComputeConvexManifold :: proc(
	world: ^World,
	workerIndex: i32,
	contact: ^Contact,
	shapeA: ^Shape,
	xfA: WorldTransform,
	shapeB: ^Shape,
	xfB: WorldTransform,
	arena: Arena,
) -> bool {
	arena := arena

	typeA := shapeA.type
	typeB := shapeB.type

	cache := &contact.convexContact.cache

	pointCapacity: i32 = MAX_MANIFOLD_POINTS
	pointBuffer := cast([^]LocalManifoldPoint)Bump(&arena, pointCapacity * size_of(LocalManifoldPoint))

	geomManifold := LocalManifold{}
	geomManifold.points = pointBuffer

	transformBtoA := InvMulWorldTransforms(xfA, xfB)

	if typeA == .sphereShape {
		assert(typeB == .sphereShape)
		CollideSpheres(&geomManifold, pointCapacity, &shapeA.sphere, &shapeB.sphere, transformBtoA)
	} else if typeA == .capsuleShape {
		if typeB == .sphereShape {
			CollideCapsuleAndSphere(&geomManifold, pointCapacity, &shapeA.capsule, &shapeB.sphere, transformBtoA)
		} else {
			assert(typeB == .capsuleShape)
			CollideCapsules(&geomManifold, pointCapacity, &shapeA.capsule, &shapeB.capsule, transformBtoA)
		}
	} else {
		assert(typeA == .hullShape)

		if typeB == .sphereShape {
			CollideHullAndSphere(
				&geomManifold,
				pointCapacity,
				shapeA.hull,
				&shapeB.sphere,
				transformBtoA,
				&cache.simplexCache,
			)
		} else if typeB == .capsuleShape {
			CollideHullAndCapsule(
				&geomManifold,
				pointCapacity,
				shapeA.hull,
				&shapeB.capsule,
				transformBtoA,
				&cache.simplexCache,
			)
		} else {
			assert(typeB == .hullShape)
			CollideHullsAtWidth(
				&geomManifold,
				pointCapacity,
				shapeA.hull,
				shapeB.hull,
				transformBtoA,
				&cache.satCache,
				world.simdWidth,
			)
			world.taskContexts.data[workerIndex].satCallCount += 1
			world.taskContexts.data[workerIndex].satCacheHitCount += i32(cache.satCache.hit)
		}
	}

	if geomManifold.pointCount == 0 {
		if contact.manifoldCount > 0 {
			FreeManifolds(world, contact.manifolds, contact.manifoldCount)
			contact.manifolds = nil
			contact.manifoldCount = 0
		}

		return false
	}

	oldPoints: [MAX_MANIFOLD_POINTS]ManifoldPoint = ---
	oldCount: i32 = 0

	if contact.manifoldCount == 0 {
		contact.manifolds = AllocateManifolds(world, 1)
		contact.manifoldCount = 1
	} else {
		oldCount = contact.manifolds[0].pointCount
		mem.copy(&oldPoints[0], &contact.manifolds[0].points[0], int(oldCount) * size_of(ManifoldPoint))
	}

	manifold := &contact.manifolds[0]
	assert(0 < geomManifold.pointCount && geomManifold.pointCount <= MAX_MANIFOLD_POINTS)
	manifold.pointCount = geomManifold.pointCount

	matrixA := MakeMatrixFromQuat(xfA.q)
	manifold.normal = MulMV(matrixA, geomManifold.normal)

	// Store point data in contact
	for i: i32 = 0; i < geomManifold.pointCount; i += 1 {
		source := &geomManifold.points[i]
		target := &manifold.points[i]

		// Contact points are computed in frame A
		target.anchorA = MulMV(matrixA, source.point)
		target.anchorB = Add(target.anchorA, SubPos(xfA.p, xfB.p))
		target.separation = source.separation
		target.featureId = MakeFeatureId(source.pair)
		target.triangleIndex = NULL_INDEX
		target.normalVelocity = 0.0
	}

	// Copy impulses from old points
	for i: i32 = 0; i < geomManifold.pointCount; i += 1 {
		pt2 := &manifold.points[i]
		pt2.totalNormalImpulse = 0.0
		pt2.persisted = false

		for j: i32 = 0; j < oldCount; j += 1 {
			pt1 := &oldPoints[j]

			if pt2.featureId == pt1.featureId {
				pt2.normalImpulse = pt1.normalImpulse
				pt2.persisted = true

				// claimed
				pt1.featureId = max(u32)

				break
			}
		}

		if pt2.persisted == false {
			pt2.normalImpulse = 0.0
		}
	}

	return true
}

@(private = "file")
UpdateConvexContact :: proc(
	world: ^World,
	workerIndex: i32,
	contact: ^Contact,
	shapeA: ^Shape,
	xfA: WorldTransform,
	shapeB: ^Shape,
	xfB: WorldTransform,
	flip: bool,
	arena: Arena,
) -> bool {
	// Compute new manifold
	touching := ComputeConvexManifold(world, workerIndex, contact, shapeA, xfA, shapeB, xfB, arena)

	if touching == false {
		assert(contact.manifolds == nil && contact.manifoldCount == 0)
		return false
	}

	assert(contact.manifoldCount == 1)

	if flip {
		// Not flipping the feature ids because they just need to match and flipping is consistent.
		manifold := &contact.manifolds[0]
		manifold.normal = Neg(manifold.normal)
		pointCount := manifold.pointCount
		for i: i32 = 0; i < pointCount; i += 1 {
			mp := &manifold.points[i]
			mp.anchorA, mp.anchorB = mp.anchorB, mp.anchorA
		}
	}

	materialA := &GetShapeMaterials(shapeA)[0]
	materialB := &GetShapeMaterials(shapeB)[0]

	// Keep these updated in case the values on the shapes are modified
	contact.friction = world.frictionCallback(
		materialA.friction,
		materialA.userMaterialId,
		materialB.friction,
		materialB.userMaterialId,
	)
	contact.restitution = world.restitutionCallback(
		materialA.restitution,
		materialA.userMaterialId,
		materialB.restitution,
		materialB.userMaterialId,
	)

	if materialA.rollingResistance > 0.0 || materialB.rollingResistance > 0.0 {
		typeA := shapeA.type
		typeB := shapeB.type

		radiusA: f32 = 0.0
		if typeA == .sphereShape {
			radiusA = shapeA.sphere.radius
		} else if typeA == .capsuleShape {
			radiusA = shapeA.capsule.radius
		} else if typeA == .hullShape {
			radiusA = 0.25 * shapeA.hull.innerRadius
		}

		radiusB: f32 = 0.0
		if typeB == .sphereShape {
			radiusB = shapeB.sphere.radius
		} else if typeB == .capsuleShape {
			radiusB = shapeB.capsule.radius
		} else if typeB == .hullShape {
			radiusB = 0.25 * shapeB.hull.innerRadius
		}

		maxRadius := MaxFloat(radiusA, radiusB)
		contact.rollingResistance = MaxFloat(materialA.rollingResistance, materialB.rollingResistance) * maxRadius
	} else {
		contact.rollingResistance = 0.0
	}

	tangentVelocityA := RotateVector(xfA.q, materialA.tangentVelocity)
	tangentVelocityB := RotateVector(xfB.q, materialB.tangentVelocity)
	contact.tangentVelocity = Sub(tangentVelocityA, tangentVelocityB)

	if world.preSolveFcn != nil && (contact.flags & simEnablePreSolveEvents) != 0 {
		shapeIdA := ShapeId{shapeA.id + 1, world.worldId, shapeA.generation}
		shapeIdB := ShapeId{shapeB.id + 1, world.worldId, shapeB.generation}

		// this call assumes thread safety
		point := OffsetPos(xfA.p, contact.manifolds[0].points[0].anchorA)
		normal := contact.manifolds[0].normal
		touching = world.preSolveFcn(shapeIdA, shapeIdB, point, normal, world.preSolveContext)
		if touching == false {
			// disable contact
			FreeManifolds(world, contact.manifolds, contact.manifoldCount)
			contact.manifolds = nil
			contact.manifoldCount = 0

			// Invalidate recycling cache.
			contact.flags &~= relativeTransformValid
			return false
		}
	}

	if (shapeA.flags & enableHitEvents) != 0 || (shapeB.flags & enableHitEvents) != 0 {
		contact.flags |= simEnableHitEvent
	} else {
		contact.flags &~= simEnableHitEvent
	}

	return true
}

// Update the contact manifold and touching status.
// Note: do not assume the shape AABBs are overlapping or are valid.
UpdateContact :: proc(
	world: ^World,
	workerIndex: i32,
	contact: ^Contact,
	shapeA: ^Shape,
	localCenterA: Vec3,
	xfA: WorldTransform,
	shapeB: ^Shape,
	localCenterB: Vec3,
	xfB: WorldTransform,
	isFast: bool,
	arena: Arena,
) -> bool {
	touching: bool

	assert(shapeB.type != .compoundShape)

	if shapeA.type == .compoundShape {
		childIndex := contact.childIndex
		child := GetCompoundChild(shapeA.compound, childIndex)

		// Temporary child shape to match existing function signatures
		childShapeA: Shape
		mem.copy(&childShapeA, shapeA, size_of(Shape))

		childShapeA.type = child.type

		// Handle child material for non-meshes.
		if child.type != .meshShape {
			assert(0 <= i32(child.materialIndices[0]) && i32(child.materialIndices[0]) < shapeA.materialCount)
			parentMaterials := GetShapeMaterials(shapeA)
			childShapeA.material = parentMaterials[child.materialIndices[0]]
			childShapeA.materials = nil
			childShapeA.materialCount = 1
		}

		if child.type == .capsuleShape {
			childShapeA.capsule = child.capsule
			if shapeB.type == .hullShape {
				// Flip
				flip := true
				touching = UpdateConvexContact(
					world,
					workerIndex,
					contact,
					shapeB,
					xfB,
					&childShapeA,
					xfA,
					flip,
					arena,
				)
			} else {
				flip := false
				touching = UpdateConvexContact(
					world,
					workerIndex,
					contact,
					&childShapeA,
					xfA,
					shapeB,
					xfB,
					flip,
					arena,
				)
			}
		} else if child.type == .hullShape {
			childShapeA.hull = child.hull
			xfChild := MulWorldTransforms(xfA, child.transform)
			flip := false
			touching = UpdateConvexContact(
				world,
				workerIndex,
				contact,
				&childShapeA,
				xfChild,
				shapeB,
				xfB,
				flip,
				arena,
			)
		} else if child.type == .meshShape {
			childShapeA.mesh = child.mesh
			xfChild := MulWorldTransforms(xfA, child.transform)

			touching = ComputeMeshManifolds(
				world,
				workerIndex,
				contact,
				&childShapeA,
				child.materialIndices,
				xfChild,
				shapeB,
				xfB,
				isFast,
				arena,
			)

			if touching && ((shapeA.flags & enableHitEvents) != 0 || (shapeB.flags & enableHitEvents) != 0) {
				contact.flags |= simEnableHitEvent
			} else {
				contact.flags &~= simEnableHitEvent
			}

			assert(
				(touching == true && contact.manifoldCount > 0) || (touching == false && contact.manifoldCount == 0),
			)
		} else {
			assert(child.type == .sphereShape)

			childShapeA.sphere = child.sphere
			if shapeB.type == .capsuleShape || shapeB.type == .hullShape {
				// Flip
				flip := true
				touching = UpdateConvexContact(
					world,
					workerIndex,
					contact,
					shapeB,
					xfB,
					&childShapeA,
					xfA,
					flip,
					arena,
				)
			} else {
				flip := false
				touching = UpdateConvexContact(
					world,
					workerIndex,
					contact,
					&childShapeA,
					xfA,
					shapeB,
					xfB,
					flip,
					arena,
				)
			}
		}

		// The anchor is relative to the child origin but oriented in world space.
		// Offset the anchor to be relative to the compound origin.
		manifoldCount := contact.manifoldCount
		offset := RotateVector(xfA.q, child.transform.p)
		for i: i32 = 0; i < manifoldCount; i += 1 {
			manifold := &contact.manifolds[i]
			pointCount := manifold.pointCount
			for j: i32 = 0; j < pointCount; j += 1 {
				mp := &manifold.points[j]
				mp.anchorA = Add(mp.anchorA, offset)
			}
		}
	} else if shapeA.type == .meshShape || shapeA.type == .heightShape {
		// Does this contact touch a mesh or height-field?

		// Compute mesh manifolds
		touching = ComputeMeshManifolds(world, workerIndex, contact, shapeA, nil, xfA, shapeB, xfB, isFast, arena)

		if touching && ((shapeA.flags & enableHitEvents) != 0 || (shapeB.flags & enableHitEvents) != 0) {
			contact.flags |= simEnableHitEvent
		} else {
			contact.flags &~= simEnableHitEvent
		}

		assert((touching == true && contact.manifoldCount > 0) || (touching == false && contact.manifoldCount == 0))
	} else {
		// Convex-vs-convex
		flip := false
		touching = UpdateConvexContact(world, workerIndex, contact, shapeA, xfA, shapeB, xfB, flip, arena)
	}

	if touching {
		centerA := RotateVector(xfA.q, localCenterA)
		centerB := RotateVector(xfB.q, localCenterB)

		// Adjust anchors to be relative to center of mass
		for i: i32 = 0; i < contact.manifoldCount; i += 1 {
			manifold := &contact.manifolds[i]
			for j: i32 = 0; j < manifold.pointCount; j += 1 {
				mp := &manifold.points[j]
				mp.anchorA = Sub(mp.anchorA, centerA)
				mp.anchorB = Sub(mp.anchorB, centerB)
			}
		}

		contact.flags |= simTouchingFlag
	} else {
		contact.flags &~= simTouchingFlag
	}

	return touching
}
