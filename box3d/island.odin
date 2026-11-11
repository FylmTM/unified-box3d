// Ported from box3d src/island.c and src/island.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:runtime"

// island.h

// Cached contact data stored in the island for fast contiguous iteration.
// Avoids touching b3Contact during union-find in b3SplitIsland.
ContactLink :: struct {
	contactId: i32,
	bodyIdA:   i32,
	bodyIdB:   i32,
}

// Cached joint data stored in the island for fast contiguous iteration.
JointLink :: struct {
	jointId: i32,
	bodyIdA: i32,
	bodyIdB: i32,
}

// Deterministic solver
//
// Collide all awake contacts
// Use bit array to emit start/stop touching events in defined order, per thread. Try using contact index, assuming contacts are
// created in a deterministic order. bit-wise OR together bit arrays and issue changes:
// - start touching: merge islands - temporary linked list - mark root island dirty - wake all - largest island is root
// - stop touching: increment constraintRemoveCount

// Persistent island for awake bodies, joints, and contacts.
// Contacts are touching.
// Contacts and joints may connect to static bodies, but static bodies are not in the island.
// https://en.wikipedia.org/wiki/Component_(graph_theory)
// https://en.wikipedia.org/wiki/Dynamic_connectivity
Island :: struct {
	// index of solver set stored in b3World
	// may be B3_NULL_INDEX
	setIndex:              i32,

	// island index within set
	// may be B3_NULL_INDEX
	localIndex:            i32,
	islandId:              i32,

	// Keeps track of how many contacts have been removed from this island.
	// This is used to determine if an island is a candidate for splitting.
	constraintRemoveCount: i32,

	// I tried using a stack array for this but the data pointer goes out of
	// sync when the world island array grows.
	bodies:                Array(i32),

	// Contacts and joints that belong to this island. May connect to static
	// bodies not in the island.
	// Each link has the two body ids so that b3SplitIsland's union-find pass
	// never needs to touch b3Contact/b3Joint.
	contacts:              Array(ContactLink),
	joints:                Array(JointLink),
}

// This is used to move islands across solver sets
IslandSim :: struct {
	islandId: i32,
}

// island.c

CreateIsland :: proc(world: ^World, setIndex: i32) -> ^Island {
	assert(setIndex == i32(SetType.awakeSet) || setIndex >= i32(SetType.firstSleepingSet))

	islandId := AllocId(&world.islandIdPool)

	if islandId == world.islands.count {
		emptyIsland := Island{}
		Array_Push(&world.islands, emptyIsland)
	} else {
		assert(world.islands.data[islandId].setIndex == NULL_INDEX)
	}

	set := Array_Get(&world.solverSets, setIndex)

	island := Array_Get(&world.islands, islandId)
	island.setIndex = setIndex
	island.localIndex = set.islandSims.count
	island.islandId = islandId
	Array_Create(&island.bodies)
	Array_Create(&island.contacts)
	Array_Create(&island.joints)
	island.constraintRemoveCount = 0

	islandSim := Array_Emplace(&set.islandSims)
	islandSim.islandId = islandId

	return island
}

DestroyIsland :: proc(world: ^World, islandId: i32) {
	if world.splitIslandId == islandId {
		world.splitIslandId = NULL_INDEX
	}

	// assume island is empty
	island := Array_Get(&world.islands, islandId)
	set := Array_Get(&world.solverSets, island.setIndex)
	{
		localIndex := island.localIndex
		lastIndex := set.islandSims.count - 1
		assert(0 <= localIndex && localIndex <= lastIndex)
		moveIslandId := set.islandSims.data[lastIndex].islandId
		set.islandSims.data[localIndex] = set.islandSims.data[lastIndex]
		world.islands.data[moveIslandId].localIndex = localIndex
		set.islandSims.count -= 1
	}

	// Free island and id (preserve island generation)
	Array_Destroy(&island.bodies)
	Array_Destroy(&island.contacts)
	Array_Destroy(&island.joints)
	island.constraintRemoveCount = 0
	island.localIndex = NULL_INDEX
	island.islandId = NULL_INDEX
	island.setIndex = NULL_INDEX

	FreeId(&world.islandIdPool, islandId)
}

@(private = "file")
MergeIslands :: proc(world: ^World, islandIdA: i32, islandIdB: i32) -> i32 {
	if islandIdA == islandIdB {
		return islandIdA
	}

	if islandIdA == NULL_INDEX {
		assert(islandIdB != NULL_INDEX)
		return islandIdB
	}

	if islandIdB == NULL_INDEX {
		assert(islandIdA != NULL_INDEX)
		return islandIdA
	}

	smallIsland: ^Island
	bigIsland: ^Island
	{
		islandA := Array_Get(&world.islands, islandIdA)
		islandB := Array_Get(&world.islands, islandIdB)

		// Keep the biggest island to reduce cache misses
		if islandA.bodies.count >= islandB.bodies.count {
			bigIsland = islandA
			smallIsland = islandB
		} else {
			bigIsland = islandB
			smallIsland = islandA
		}
	}

	bigIslandId := bigIsland.islandId
	Array_Reserve(&bigIsland.bodies, bigIsland.bodies.count + smallIsland.bodies.count)

	// Move bodies from smaller island to larger island
	for i: i32 = 0; i < smallIsland.bodies.count; i += 1 {
		bodyId := smallIsland.bodies.data[i]
		body := Array_Get(&world.bodies, bodyId)
		VALIDATE(body.islandId == smallIsland.islandId)
		body.islandId = bigIslandId
		body.islandIndex = bigIsland.bodies.count
		Array_Push(&bigIsland.bodies, bodyId)
	}

	// Migrate contacts from smaller island to larger island
	if smallIsland.contacts.count > 0 {
		Array_Reserve(&bigIsland.contacts, bigIsland.contacts.count + smallIsland.contacts.count)

		for i: i32 = 0; i < smallIsland.contacts.count; i += 1 {
			link := &smallIsland.contacts.data[i]
			contact := Array_Get(&world.contacts, link.contactId)
			contact.islandId = bigIslandId
			contact.islandIndex = bigIsland.contacts.count
			Array_Push(&bigIsland.contacts, link^)
		}
	}

	// Migrate joints from smaller island to larger island
	if smallIsland.joints.count > 0 {
		Array_Reserve(&bigIsland.joints, bigIsland.joints.count + smallIsland.joints.count)

		for i: i32 = 0; i < smallIsland.joints.count; i += 1 {
			link := &smallIsland.joints.data[i]
			joint := Array_Get(&world.joints, link.jointId)
			joint.islandId = bigIslandId
			joint.islandIndex = bigIsland.joints.count
			Array_Push(&bigIsland.joints, link^)
		}
	}

	// Track removed constraints
	bigIsland.constraintRemoveCount += smallIsland.constraintRemoveCount

	DestroyIsland(world, smallIsland.islandId)

	ValidateIsland(world, bigIslandId)

	return bigIslandId
}

@(private = "file")
AddContactToIsland :: proc(world: ^World, islandId: i32, contact: ^Contact) {
	assert(contact.islandId == NULL_INDEX)
	assert(contact.islandIndex == NULL_INDEX)

	island := Array_Get(&world.islands, islandId)

	contact.islandId = islandId
	contact.islandIndex = island.contacts.count

	link: ContactLink
	link.contactId = contact.contactId
	link.bodyIdA = contact.edges[0].bodyId
	link.bodyIdB = contact.edges[1].bodyId
	Array_Push(&island.contacts, link)

	ValidateIsland(world, islandId)
}

// Link a contact into an island.
// Link contacts into the island graph when it starts having contact points
LinkContact :: proc(world: ^World, contact: ^Contact) {
	assert((contact.flags & contactTouchingFlag) != 0)

	bodyIdA := contact.edges[0].bodyId
	bodyIdB := contact.edges[1].bodyId

	bodyA := Array_Get(&world.bodies, bodyIdA)
	bodyB := Array_Get(&world.bodies, bodyIdB)

	assert(bodyA.setIndex != i32(SetType.disabledSet) && bodyB.setIndex != i32(SetType.disabledSet))
	assert(bodyA.setIndex != i32(SetType.staticSet) || bodyB.setIndex != i32(SetType.staticSet))

	// Wake bodyB if bodyA is awake and bodyB is sleeping
	if bodyA.setIndex == i32(SetType.awakeSet) && bodyB.setIndex >= i32(SetType.firstSleepingSet) {
		WakeSolverSet(world, bodyB.setIndex)
	}

	// Wake bodyA if bodyB is awake and bodyA is sleeping
	if bodyB.setIndex == i32(SetType.awakeSet) && bodyA.setIndex >= i32(SetType.firstSleepingSet) {
		WakeSolverSet(world, bodyA.setIndex)
	}

	islandIdA := bodyA.islandId
	islandIdB := bodyB.islandId

	// Static bodies have null island indices.
	assert(bodyA.setIndex != i32(SetType.staticSet) || islandIdA == NULL_INDEX)
	assert(bodyB.setIndex != i32(SetType.staticSet) || islandIdB == NULL_INDEX)
	assert(islandIdA != NULL_INDEX || islandIdB != NULL_INDEX)

	// Merge islands. This will destroy one of the islands.
	finalIslandId := MergeIslands(world, islandIdA, islandIdB)

	// Add contact to the island that survived
	AddContactToIsland(world, finalIslandId, contact)
}

// This is called when a contact no longer has contact points or when a contact is destroyed.
// Unlink contact from the island graph when it stops having contact points
UnlinkContact :: proc(world: ^World, contact: ^Contact) {
	assert(contact.islandId != NULL_INDEX)

	// remove from island
	islandId := contact.islandId
	island := Array_Get(&world.islands, islandId)

	removeIndex := contact.islandIndex
	assert(0 <= removeIndex && removeIndex < island.contacts.count)
	assert(island.contacts.data[removeIndex].contactId == contact.contactId)

	movedIndex := Array_RemoveSwap(&island.contacts, removeIndex)
	if movedIndex != NULL_INDEX {
		// Fix islandIndex on the contact that was swapped into removeIndex
		movedLink := &island.contacts.data[removeIndex]
		movedContact := Array_Get(&world.contacts, movedLink.contactId)
		assert(movedContact.islandIndex == movedIndex)
		movedContact.islandIndex = removeIndex
	}

	contact.islandId = NULL_INDEX
	contact.islandIndex = NULL_INDEX
	island.constraintRemoveCount += 1

	ValidateIsland(world, islandId)
}

@(private = "file")
AddJointToIsland :: proc(world: ^World, islandId: i32, joint: ^Joint) {
	assert(joint.islandId == NULL_INDEX)
	assert(joint.islandIndex == NULL_INDEX)

	island := Array_Get(&world.islands, islandId)

	joint.islandId = islandId
	joint.islandIndex = island.joints.count

	link: JointLink
	link.jointId = joint.jointId
	link.bodyIdA = joint.edges[0].bodyId
	link.bodyIdB = joint.edges[1].bodyId
	Array_Push(&island.joints, link)

	ValidateIsland(world, islandId)
}

// Link a joint into the island graph when it is created
LinkJoint :: proc(world: ^World, joint: ^Joint) {
	bodyA := Array_Get(&world.bodies, joint.edges[0].bodyId)
	bodyB := Array_Get(&world.bodies, joint.edges[1].bodyId)

	assert(bodyA.type == .dynamicBody || bodyB.type == .dynamicBody)

	if bodyA.setIndex == i32(SetType.awakeSet) && bodyB.setIndex >= i32(SetType.firstSleepingSet) {
		WakeSolverSet(world, bodyB.setIndex)
	} else if bodyB.setIndex == i32(SetType.awakeSet) && bodyA.setIndex >= i32(SetType.firstSleepingSet) {
		WakeSolverSet(world, bodyA.setIndex)
	}

	islandIdA := bodyA.islandId
	islandIdB := bodyB.islandId

	assert(islandIdA != NULL_INDEX || islandIdB != NULL_INDEX)

	// Merge islands. This will destroy one of the islands.
	finalIslandId := MergeIslands(world, islandIdA, islandIdB)

	// Add joint the island that survived
	AddJointToIsland(world, finalIslandId, joint)
}

// Unlink a joint from the island graph when it is destroyed
UnlinkJoint :: proc(world: ^World, joint: ^Joint) {
	if joint.islandId == NULL_INDEX {
		return
	}

	// remove from island
	islandId := joint.islandId
	island := Array_Get(&world.islands, islandId)

	removeIndex := joint.islandIndex
	assert(0 <= removeIndex && removeIndex < island.joints.count)
	assert(island.joints.data[removeIndex].jointId == joint.jointId)

	movedIndex := Array_RemoveSwap(&island.joints, removeIndex)
	if movedIndex != NULL_INDEX {
		// Fix islandIndex on the joint that was swapped into removeIndex
		movedLink := &island.joints.data[removeIndex]
		movedJoint := Array_Get(&world.joints, movedLink.jointId)
		assert(movedJoint.islandIndex == movedIndex)
		movedJoint.islandIndex = removeIndex
	}

	joint.islandId = NULL_INDEX
	joint.islandIndex = NULL_INDEX
	island.constraintRemoveCount += 1

	ValidateIsland(world, islandId)
}

// Find parent of a node. Use path halving to speed up further queries.
@(private = "file")
IslandFindParent :: proc(parents: [^]i32, node: i32) -> i32 {
	node := node

	// Walk the chain of parents to find the node that is its own parent (the root)
	for parents[node] != node {
		grandParent := parents[parents[node]]
		parents[node] = grandParent
		node = grandParent
	}

	return node
}

// Connect the components containing node1 and node2.
// Uses rank to keep tree balanced. Tracks per-component contact and joint counts.
@(private = "file")
IslandUnion :: proc(
	parents: [^]i32,
	ranks: [^]i32,
	node1: i32,
	node2: i32,
	contactCounts: [^]i32,
	jointCounts: [^]i32,
) {
	root1 := IslandFindParent(parents, node1)
	root2 := IslandFindParent(parents, node2)
	if root1 != root2 {
		if ranks[root1] < ranks[root2] {
			parents[root1] = root2
			contactCounts[root2] += contactCounts[root1]
			jointCounts[root2] += jointCounts[root1]
		} else if ranks[root1] > ranks[root2] {
			parents[root2] = root1
			contactCounts[root1] += contactCounts[root2]
			jointCounts[root1] += jointCounts[root2]
		} else {
			parents[root2] = root1
			ranks[root1] += 1
			contactCounts[root1] += contactCounts[root2]
			jointCounts[root1] += jointCounts[root2]
		}
	}
}

// This uses union-find.
// https://en.wikipedia.org/wiki/Disjoint-set_data_structure
SplitIsland :: proc(world: ^World, baseId: i32) {
	baseIsland := Array_Get(&world.islands, baseId)
	assert(baseIsland.constraintRemoveCount > 0)
	assert(baseIsland.setIndex == i32(SetType.awakeSet))

	ValidateIsland(world, baseId)

	// Cache base island fields before b3CreateIsland, which may reallocate
	// world->islands and invalidate the baseIsland pointer.
	baseBodyCount := baseIsland.bodies.count
	baseBodyIds := baseIsland.bodies.data
	baseBodyCapacity := baseIsland.bodies.capacity

	baseContactCount := baseIsland.contacts.count
	baseContacts := baseIsland.contacts.data
	baseContactCapacity := baseIsland.contacts.capacity

	baseJointCount := baseIsland.joints.count
	baseJoints := baseIsland.joints.data
	baseJointCapacity := baseIsland.joints.capacity

	alloc := &world.stack

	// No lock is needed because I ensure the allocator is not used while this task is active.
	// Allocate contactCounts and jointCounts before ranks so ranks can be freed first (LIFO arena).
	parents := cast([^]i32)StackAlloc(alloc, baseBodyCount * size_of(i32), "parents")
	contactCounts := cast([^]i32)StackAlloc(alloc, baseBodyCount * size_of(i32), "contact counts")
	jointCounts := cast([^]i32)StackAlloc(alloc, baseBodyCount * size_of(i32), "joint counts")
	ranks := cast([^]i32)StackAlloc(alloc, baseBodyCount * size_of(i32), "ranks")
	for i: i32 = 0; i < baseBodyCount; i += 1 {
		parents[i] = i
		ranks[i] = 0
		contactCounts[i] = 0
		jointCounts[i] = 0
	}

	bodies := world.bodies.data

	// Union over contacts, tracking per-component contact counts
	for i: i32 = 0; i < baseContactCount; i += 1 {
		bodyIdA := baseContacts[i].bodyIdA
		bodyIdB := baseContacts[i].bodyIdB
		VALIDATE(0 <= bodyIdA && bodyIdA < world.bodies.count)
		VALIDATE(0 <= bodyIdB && bodyIdB < world.bodies.count)
		bodyA := &bodies[bodyIdA]
		bodyB := &bodies[bodyIdB]
		islandIndexA := bodyA.islandIndex
		islandIndexB := bodyB.islandIndex

		// Only connect non-static bodies
		if islandIndexA != NULL_INDEX && islandIndexB != NULL_INDEX {
			VALIDATE(0 <= islandIndexA && islandIndexA < baseBodyCount)
			VALIDATE(0 <= islandIndexB && islandIndexB < baseBodyCount)
			IslandUnion(parents, ranks, islandIndexA, islandIndexB, contactCounts, jointCounts)
			root := IslandFindParent(parents, islandIndexA)
			contactCounts[root] += 1
		} else {
			islandIndex := islandIndexA != NULL_INDEX ? islandIndexA : islandIndexB
			root := IslandFindParent(parents, islandIndex)
			contactCounts[root] += 1
		}
	}

	// Union over joints, tracking per-component joint counts
	for i: i32 = 0; i < baseJointCount; i += 1 {
		bodyIdA := baseJoints[i].bodyIdA
		bodyIdB := baseJoints[i].bodyIdB
		VALIDATE(0 <= bodyIdA && bodyIdA < world.bodies.count)
		VALIDATE(0 <= bodyIdB && bodyIdB < world.bodies.count)
		bodyA := &bodies[bodyIdA]
		bodyB := &bodies[bodyIdB]
		islandIndexA := bodyA.islandIndex
		islandIndexB := bodyB.islandIndex

		// Only connect non-static bodies
		if islandIndexA != NULL_INDEX && islandIndexB != NULL_INDEX {
			VALIDATE(0 <= islandIndexA && islandIndexA < baseBodyCount)
			VALIDATE(0 <= islandIndexB && islandIndexB < baseBodyCount)
			IslandUnion(parents, ranks, islandIndexA, islandIndexB, contactCounts, jointCounts)
			root := IslandFindParent(parents, islandIndexA)
			jointCounts[root] += 1
		} else {
			islandIndex := islandIndexA != NULL_INDEX ? islandIndexA : islandIndexB
			root := IslandFindParent(parents, islandIndex)
			jointCounts[root] += 1
		}
	}

	// Done with ranks
	StackFree(alloc, ranks)
	ranks = nil

	// Flatten all parent indices and count connected components.
	componentCount: i32 = 0
	for i: i32 = 0; i < baseBodyCount; i += 1 {
		parents[i] = IslandFindParent(parents, i)
		if parents[i] == i {
			componentCount += 1
		}
	}

	// Island is still fully connected, no split needed.
	if componentCount == 1 {
		baseIsland.constraintRemoveCount = 0
		StackFree(alloc, jointCounts)
		StackFree(alloc, contactCounts)
		StackFree(alloc, parents)
		return
	}

	// Detach body/contact/joint arrays from base island so b3DestroyIsland won't free them
	baseIsland.bodies.data = nil
	baseIsland.bodies.count = 0
	baseIsland.bodies.capacity = 0

	baseIsland.contacts.data = nil
	baseIsland.contacts.count = 0
	baseIsland.contacts.capacity = 0

	baseIsland.joints.data = nil
	baseIsland.joints.count = 0
	baseIsland.joints.capacity = 0

	// Null so code below doesn't accidentally use this.
	baseIsland = nil

	// Map from body index to new island index. Only set for root bodies.
	rootMap := cast([^]i32)StackAlloc(alloc, baseBodyCount * size_of(i32), "root map")
	for i: i32 = 0; i < baseBodyCount; i += 1 {
		rootMap[i] = NULL_INDEX
	}

	componentBodyCounts := cast([^]i32)StackAlloc(alloc, componentCount * size_of(i32), "component body counts")
	componentContactCounts := cast([^]i32)StackAlloc(alloc, componentCount * size_of(i32), "component contact counts")
	componentJointCounts := cast([^]i32)StackAlloc(alloc, componentCount * size_of(i32), "component joint counts")
	islandCount: i32 = 0

	// Find the root body for each body and create islands as needed.
	// Extract per-component counts from the root nodes' accumulated counts.
	for i: i32 = 0; i < baseBodyCount; i += 1 {
		rootIndex := parents[i]
		if rootMap[rootIndex] == NULL_INDEX {
			rootMap[rootIndex] = islandCount
			componentBodyCounts[islandCount] = 0
			componentContactCounts[islandCount] = contactCounts[rootIndex]
			componentJointCounts[islandCount] = jointCounts[rootIndex]
			islandCount += 1
		}

		componentBodyCounts[rootMap[rootIndex]] += 1
	}

	assert(islandCount == componentCount)

	// Map from new island index to island id
	islandIds := cast([^]i32)StackAlloc(alloc, islandCount * size_of(i32), "island ids")

	// Create new islands and reserve body/contact/joint arrays
	for i: i32 = 0; i < islandCount; i += 1 {
		// WARNING: this invalidates baseIsland pointer
		newIsland := CreateIsland(world, i32(SetType.awakeSet))
		islandIds[i] = newIsland.islandId

		// Reserve arrays to avoid wasteful growth and memcpy.
		Array_Reserve(&newIsland.bodies, componentBodyCounts[i])
		Array_Reserve(&newIsland.contacts, componentContactCounts[i])
		Array_Reserve(&newIsland.joints, componentJointCounts[i])
	}

	// Assign bodies to new islands
	for i: i32 = 0; i < baseBodyCount; i += 1 {
		bodyId := baseBodyIds[i]
		root := IslandFindParent(parents, i)
		newIslandId := islandIds[rootMap[root]]

		body := Array_Get(&world.bodies, bodyId)
		newIsland := Array_Get(&world.islands, newIslandId)

		body.islandId = newIslandId
		body.islandIndex = newIsland.bodies.count

		// Ensure the array has the correct capacity
		VALIDATE(newIsland.bodies.count < newIsland.bodies.capacity)
		Array_Push(&newIsland.bodies, bodyId)
	}

	// Assign contacts to the island of their bodies
	for i: i32 = 0; i < baseContactCount; i += 1 {
		link := &baseContacts[i]
		contact := Array_Get(&world.contacts, link.contactId)

		// Static bodies don't have an island id.
		bodyA := Array_Get(&world.bodies, link.bodyIdA)
		bodyB := Array_Get(&world.bodies, link.bodyIdB)
		targetIslandId := bodyA.islandId != NULL_INDEX ? bodyA.islandId : bodyB.islandId

		targetIsland := Array_Get(&world.islands, targetIslandId)
		contact.islandId = targetIslandId
		contact.islandIndex = targetIsland.contacts.count

		// Ensure the array has the correct capacity
		VALIDATE(targetIsland.contacts.count < targetIsland.contacts.capacity)
		Array_Push(&targetIsland.contacts, link^)
	}

	// Assign joints to the island of their bodies
	for i: i32 = 0; i < baseJointCount; i += 1 {
		link := &baseJoints[i]
		joint := Array_Get(&world.joints, link.jointId)

		// Static bodies don't have an island id.
		bodyA := Array_Get(&world.bodies, link.bodyIdA)
		bodyB := Array_Get(&world.bodies, link.bodyIdB)
		targetIslandId := bodyA.islandId != NULL_INDEX ? bodyA.islandId : bodyB.islandId

		targetIsland := Array_Get(&world.islands, targetIslandId)
		joint.islandId = targetIslandId
		joint.islandIndex = targetIsland.joints.count

		// Ensure the array has the correct capacity
		VALIDATE(targetIsland.joints.count < targetIsland.joints.capacity)
		Array_Push(&targetIsland.joints, link^)
	}

	// Destroy the base island
	DestroyIsland(world, baseId)

	// Free the detached arrays manually
	Free(baseBodyIds, int(baseBodyCapacity) * size_of(i32))
	Free(baseContacts, int(baseContactCapacity) * size_of(ContactLink))
	Free(baseJoints, int(baseJointCapacity) * size_of(JointLink))

	// Free arena items in LIFO order
	StackFree(alloc, islandIds)
	StackFree(alloc, componentJointCounts)
	StackFree(alloc, componentContactCounts)
	StackFree(alloc, componentBodyCounts)
	StackFree(alloc, rootMap)
	StackFree(alloc, jointCounts)
	StackFree(alloc, contactCounts)
	StackFree(alloc, parents)
}

// Split an island because some contacts and/or joints have been removed.
// This is called during the constraint solve while islands are not being touched. This uses union find and
// touches a lot of memory, so it can be slow.
// Note: contacts/joints connected to static bodies must belong to an island but don't affect island connectivity
// Note: static bodies are never in an island
// Note: this task interacts with some allocators without locks under the assumption that no other tasks
// are interacting with these data structures.
// unified: C calling convention (the library's API is upstream's C ABI)
SplitIslandTask :: proc "c" (context_: rawptr) {
	context = runtime.default_context()
	// unified end
	ticks := GetTicks()
	world := (^World)(context_)

	assert(world.splitIslandId != NULL_INDEX)

	SplitIsland(world, world.splitIslandId)

	world.splitIslandId = NULL_INDEX
	world.profile.splitIslands += GetMilliseconds(ticks)
}

when ENABLE_VALIDATION {

	ValidateIsland :: proc(world: ^World, islandId: i32) {
		if islandId == NULL_INDEX {
			return
		}

		island := Array_Get(&world.islands, islandId)
		assert(island.islandId == islandId)
		assert(island.setIndex != NULL_INDEX)

		{
			assert(island.bodies.count > 0)
			assert(island.bodies.count <= GetIdCount(&world.bodyIdPool))

			for i: i32 = 0; i < island.bodies.count; i += 1 {
				body := Array_Get(&world.bodies, island.bodies.data[i])
				assert(body.islandId == islandId)
				assert(body.islandIndex == i)
				assert(body.setIndex == island.setIndex)
			}
		}

		if island.contacts.count > 0 {
			assert(island.contacts.count <= GetIdCount(&world.contactIdPool))

			for i: i32 = 0; i < island.contacts.count; i += 1 {
				link := &island.contacts.data[i]
				contact := Array_Get(&world.contacts, link.contactId)
				assert(contact.setIndex == island.setIndex)
				assert(contact.islandId == islandId)
				assert(contact.islandIndex == i)
			}
		}

		if island.joints.count > 0 {
			assert(island.joints.count <= GetIdCount(&world.jointIdPool))

			for i: i32 = 0; i < island.joints.count; i += 1 {
				link := &island.joints.data[i]
				joint := Array_Get(&world.joints, link.jointId)
				assert(joint.setIndex == island.setIndex)
				assert(joint.islandId == islandId)
				assert(joint.islandIndex == i)
			}
		}
	}

} else {

	ValidateIsland :: proc(world: ^World, islandId: i32) {
		_ = world
		_ = islandId
	}

}
