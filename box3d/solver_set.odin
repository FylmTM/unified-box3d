// Ported from box3d src/solver_set.c and src/solver_set.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "core:mem"

// solver_set.h

// This holds solver set data. The following sets are used:
// - static set for all static bodies and joints between static bodies
// - active set for all active bodies with body states (no
// contacts or joints)
// - disabled set for disabled bodies and their joints
// - all further sets are sleeping island sets along with their contacts and joints
// The purpose of solver sets is to achieve high memory locality.
// https://www.youtube.com/watch?v=nZNd5FjSquk
SolverSet :: struct {
	// Body array. Empty for unused set.
	bodySims:       Array(BodySim),

	// Body state only exists for active set
	bodyStates:     Array(BodyState),

	// This holds sleeping/disabled joints. Empty for static/active set.
	jointSims:      Array(JointSim),

	// This holds all contacts for sleeping sets.
	// This holds non-touching contacts for the awake set.
	// This should be empty for the static and disabled sets.
	contactIndices: Array(i32),

	// The awake set has an array of islands. Sleeping sets normally have a single islands. However, joints
	// created between sleeping sets causes the sets to merge, leaving them with multiple islands. These sleeping
	// islands will be naturally merged with the set is woken.
	// The static and disabled sets have no islands.
	// Islands live in the solver sets to limit the number of islands that need to be considered for sleeping.
	islandSims:     Array(IslandSim),

	// Aligns with b3World::solverSetIdPool. Used to create a stable id for body/contact/joint/islands.
	setIndex:       i32,
}

// solver_set.c

DestroySolverSet :: proc(world: ^World, setIndex: i32) {
	set := Array_Get(&world.solverSets, setIndex)

	Array_Destroy(&set.bodySims)
	Array_Destroy(&set.bodyStates)
	Array_Destroy(&set.contactIndices)
	Array_Destroy(&set.jointSims)
	Array_Destroy(&set.islandSims)

	FreeId(&world.solverSetIdPool, setIndex)
	set^ = SolverSet{}
	set.setIndex = NULL_INDEX
}

// Wake a solver set. Does not merge islands.
// Contacts can be in several places:
// 1. non-touching contacts in the disabled set
// 2. non-touching contacts already in the awake set
// 3. touching contacts in the sleeping set
// This handles contact types 1 and 3. Type 2 doesn't need any action.
WakeSolverSet :: proc(world: ^World, setIndex: i32) {
	assert(setIndex >= i32(SetType.firstSleepingSet))
	set := Array_Get(&world.solverSets, setIndex)
	awakeSet := Array_Get(&world.solverSets, i32(SetType.awakeSet))
	disabledSet := Array_Get(&world.solverSets, i32(SetType.disabledSet))

	bodies := world.bodies.data

	bodyCount := set.bodySims.count
	for i: i32 = 0; i < bodyCount; i += 1 {
		simSrc := &set.bodySims.data[i]

		body := &bodies[simSrc.bodyId]
		assert(body.setIndex == setIndex)
		body.setIndex = i32(SetType.awakeSet)
		body.localIndex = awakeSet.bodySims.count

		// Reset sleep timer
		body.sleepTime = 0.0

		simDst := Array_Emplace(&awakeSet.bodySims)
		mem.copy(simDst, simSrc, size_of(BodySim))

		state := Array_Emplace(&awakeSet.bodyStates)
		state^ = identityBodyState
		state.flags = body.flags

		// Move non-touching contacts from disabled set to awake set.
		encodedBodySimIndex := body.localIndex
		contactKey := body.headContactKey
		for contactKey != NULL_INDEX {
			edgeIndex := contactKey & 1
			contactId := contactKey >> 1

			contact := Array_Get(&world.contacts, contactId)
			contactKey = contact.edges[edgeIndex].nextKey

			if contact.setIndex != i32(SetType.disabledSet) {
				assert(contact.setIndex == i32(SetType.awakeSet) || contact.setIndex == setIndex)

				// If this contact is awake but not touching I need to fixup the body sim index here.
				if contact.setIndex == i32(SetType.awakeSet) {
					assert(contact.colorIndex == NULL_INDEX)
					if edgeIndex == 0 {
						contact.encodedBodySimA = encodedBodySimIndex
					} else {
						contact.encodedBodySimB = encodedBodySimIndex
					}
				}

				continue
			}

			localIndex := contact.localIndex

			assert(0 <= localIndex && localIndex < disabledSet.contactIndices.count)
			assert(disabledSet.contactIndices.data[localIndex] == contactId)

			assert((contact.flags & contactTouchingFlag) == 0 && contact.manifoldCount == 0)

			contact.setIndex = i32(SetType.awakeSet)
			contact.localIndex = awakeSet.contactIndices.count
			Array_Push(&awakeSet.contactIndices, contactId)

			if edgeIndex == 0 {
				contact.encodedBodySimA = encodedBodySimIndex
			} else {
				contact.encodedBodySimB = encodedBodySimIndex
			}

			movedLocalIndex := Array_RemoveSwap(&disabledSet.contactIndices, localIndex)
			if movedLocalIndex != NULL_INDEX {
				// fix moved element
				movedContactIndex := disabledSet.contactIndices.data[localIndex]
				movedContact := Array_Get(&world.contacts, movedContactIndex)
				assert(movedContact.localIndex == movedLocalIndex)
				movedContact.localIndex = localIndex
			}
		}
	}

	// Transfer touching contacts from sleeping set to constraint graph.
	{
		contactCount := set.contactIndices.count
		for i: i32 = 0; i < contactCount; i += 1 {
			contactIndex := set.contactIndices.data[i]
			contact := Array_Get(&world.contacts, contactIndex)
			assert((contact.flags & contactTouchingFlag) != 0)
			assert((contact.flags & simTouchingFlag) != 0)
			assert(contact.setIndex == setIndex)
			AddContactToGraph(world, contact)
			contact.setIndex = i32(SetType.awakeSet)
		}
	}

	// transfer joints from sleeping set to awake set
	{
		jointCount := set.jointSims.count
		for i: i32 = 0; i < jointCount; i += 1 {
			jointSim := &set.jointSims.data[i]
			joint := Array_Get(&world.joints, jointSim.jointId)
			assert(joint.setIndex == setIndex)
			AddJointToGraph(world, jointSim, joint)
			joint.setIndex = i32(SetType.awakeSet)
		}
	}

	// transfer island from sleeping set to awake set
	// Usually a sleeping set has only one island, but it is possible
	// that joints are created between sleeping islands and they
	// are moved to the same sleeping set.
	{
		islandCount := set.islandSims.count
		for i: i32 = 0; i < islandCount; i += 1 {
			islandSrc := &set.islandSims.data[i]
			island := Array_Get(&world.islands, islandSrc.islandId)
			island.setIndex = i32(SetType.awakeSet)
			island.localIndex = awakeSet.islandSims.count
			islandDst := Array_Emplace(&awakeSet.islandSims)
			mem.copy(islandDst, islandSrc, size_of(IslandSim))
		}
	}

	// destroy the sleeping set
	DestroySolverSet(world, setIndex)
}

TrySleepIsland :: proc(world: ^World, islandId: i32) {
	island := Array_Get(&world.islands, islandId)
	assert(island.setIndex == i32(SetType.awakeSet))

	// Cannot put an island to sleep while it has a pending split and more than one body.
	if island.constraintRemoveCount > 0 && island.bodies.count > 1 {
		return
	}

	// island is sleeping
	// - create new sleeping solver set
	// - move island to sleeping solver set
	// - identify non-touching contacts that should move to sleeping solver set or disabled set
	// - remove old island
	// - fix island
	sleepSetId := AllocId(&world.solverSetIdPool)
	if sleepSetId == world.solverSets.count {
		set := SolverSet{}
		set.setIndex = NULL_INDEX
		Array_Push(&world.solverSets, set)
	}

	sleepSet := Array_Get(&world.solverSets, sleepSetId)
	sleepSet^ = SolverSet{}

	// grab awake set after creating the sleep set because the solver set array may have been resized
	awakeSet := Array_Get(&world.solverSets, i32(SetType.awakeSet))
	disabledSet := Array_Get(&world.solverSets, i32(SetType.disabledSet))
	assert(0 <= island.localIndex && island.localIndex < awakeSet.islandSims.count)

	sleepSet.setIndex = sleepSetId
	Array_Reserve(&sleepSet.bodySims, island.bodies.count)
	Array_Reserve(&sleepSet.contactIndices, island.contacts.count)
	Array_Reserve(&sleepSet.jointSims, island.joints.count)

	// move awake bodies to sleeping set
	// this shuffles around bodies in the awake set
	{
		for i: i32 = 0; i < island.bodies.count; i += 1 {
			bodyId := island.bodies.data[i]
			body := Array_Get(&world.bodies, bodyId)
			assert(body.setIndex == i32(SetType.awakeSet))
			assert(body.islandId == islandId)
			assert(body.islandIndex == i)

			// Update the body move event to indicate this body fell asleep
			// It could happen the body is forced asleep before it ever moves.
			if body.bodyMoveIndex != NULL_INDEX {
				moveEvent := Array_Get(&world.bodyMoveEvents, body.bodyMoveIndex)
				assert(moveEvent.bodyId.index1 - 1 == bodyId)
				assert(moveEvent.bodyId.generation == body.generation)
				moveEvent.fellAsleep = true
				body.bodyMoveIndex = NULL_INDEX
			}

			awakeBodyIndex := body.localIndex
			awakeSim := Array_Get(&awakeSet.bodySims, awakeBodyIndex)

			// move body sim to sleep set
			sleepBodyIndex := sleepSet.bodySims.count
			sleepBodySim := Array_Emplace(&sleepSet.bodySims)
			mem.copy(sleepBodySim, awakeSim, size_of(BodySim))

			movedIndex := Array_RemoveSwap(&awakeSet.bodySims, awakeBodyIndex)
			if movedIndex != NULL_INDEX {
				// fix local index on moved element
				movedSim := &awakeSet.bodySims.data[awakeBodyIndex]
				movedId := movedSim.bodyId
				movedBody := Array_Get(&world.bodies, movedId)
				assert(movedBody.localIndex == movedIndex)
				movedBody.localIndex = awakeBodyIndex

				if movedBody.islandId != islandId {
					RefreshBodyContactIndices(world, movedBody)
				}
			}

			// destroy state, no need to clone
			Array_RemoveSwap(&awakeSet.bodyStates, awakeBodyIndex)

			body.setIndex = sleepSetId
			body.localIndex = sleepBodyIndex

			// Move non-touching contacts to the disabled set.
			// Non-touching contacts may exist between sleeping islands and there is no clear ownership.
			contactKey := body.headContactKey
			for contactKey != NULL_INDEX {
				contactId := contactKey >> 1
				edgeIndex := contactKey & 1

				contact := Array_Get(&world.contacts, contactId)

				assert(contact.setIndex == i32(SetType.awakeSet))

				contactKey = contact.edges[edgeIndex].nextKey

				if contact.colorIndex != NULL_INDEX {
					// contact is touching and will be moved separately
					assert((contact.flags & contactTouchingFlag) != 0)
					continue
				}

				if edgeIndex == 0 {
					contact.encodedBodySimA = NULL_INDEX
				} else {
					contact.encodedBodySimB = NULL_INDEX
				}

				// the other body may still be awake, it still may go to sleep and then it will be responsible
				// for moving this contact to the disabled set.
				otherEdgeIndex := edgeIndex ~ 1
				otherBodyId := contact.edges[otherEdgeIndex].bodyId
				otherBody := Array_Get(&world.bodies, otherBodyId)
				if otherBody.setIndex == i32(SetType.awakeSet) {
					continue
				}

				localIndex := contact.localIndex
				assert(awakeSet.contactIndices.data[localIndex] == contactId)

				assert(contact.manifoldCount == 0)
				assert((contact.flags & contactTouchingFlag) == 0)

				// Move the non-touching contact to the disabled set.
				contact.setIndex = i32(SetType.disabledSet)

				// This is mandatory for validation to work correctly
				contact.localIndex = disabledSet.contactIndices.count
				Array_Push(&disabledSet.contactIndices, contact.contactId)

				movedLocalIndex := Array_RemoveSwap(&awakeSet.contactIndices, localIndex)
				if movedLocalIndex != NULL_INDEX {
					// fix moved element
					movedContactIndex := awakeSet.contactIndices.data[localIndex]
					movedContact := Array_Get(&world.contacts, movedContactIndex)
					assert(movedContact.localIndex == movedLocalIndex)
					movedContact.localIndex = localIndex
				}
			}
		}
	}

	// move touching contacts to sleeping set
	// this shuffles contacts in the awake set
	{
		for i: i32 = 0; i < island.contacts.count; i += 1 {
			contactId := island.contacts.data[i].contactId
			contact := Array_Get(&world.contacts, contactId)
			assert(contact.setIndex == i32(SetType.awakeSet))
			assert(contact.islandId == islandId)
			assert(contact.islandIndex == i)
			colorIndex := contact.colorIndex
			assert(0 <= colorIndex && colorIndex < GRAPH_COLOR_COUNT)

			color := &world.constraintGraph.colors[colorIndex]

			// Remove bodies from graph coloring associated with this constraint
			if colorIndex != OVERFLOW_INDEX {
				// might clear a bit for a static body, but this has no effect
				ClearBit(&color.bodySet, u32(contact.edges[0].bodyId))
				ClearBit(&color.bodySet, u32(contact.edges[1].bodyId))
			}

			sleepContactIndex := sleepSet.contactIndices.count
			Array_Push(&sleepSet.contactIndices, contactId)

			localIndex := contact.localIndex
			if (contact.flags & simMeshContact) != 0 || colorIndex == OVERFLOW_INDEX {
				movedLocalIndex := Array_RemoveSwap(&color.contacts, localIndex)
				if movedLocalIndex != NULL_INDEX {
					// fix moved element
					movedContactId := color.contacts.data[localIndex].contactId
					movedContact := Array_Get(&world.contacts, movedContactId)
					assert(movedContact.localIndex == movedLocalIndex)
					movedContact.localIndex = localIndex
				}
			} else {
				movedLocalIndex := Array_RemoveSwap(&color.convexContacts, localIndex)
				if movedLocalIndex != NULL_INDEX {
					// fix moved element
					movedContactId := color.convexContacts.data[localIndex]
					movedContact := Array_Get(&world.contacts, movedContactId)
					assert(movedContact.localIndex == movedLocalIndex)
					movedContact.localIndex = localIndex
				}
			}

			contact.setIndex = sleepSetId
			contact.colorIndex = NULL_INDEX
			contact.localIndex = sleepContactIndex
			contact.encodedBodySimA = SleepBodySimIndex(contact.encodedBodySimA)
			contact.encodedBodySimB = SleepBodySimIndex(contact.encodedBodySimB)
		}
	}

	// move joints
	// this shuffles joints in the awake set
	{
		for i: i32 = 0; i < island.joints.count; i += 1 {
			jointId := island.joints.data[i].jointId
			joint := Array_Get(&world.joints, jointId)
			assert(joint.setIndex == i32(SetType.awakeSet))
			assert(joint.islandId == islandId)
			assert(joint.islandIndex == i)
			colorIndex := joint.colorIndex
			localIndex := joint.localIndex

			assert(0 <= colorIndex && colorIndex < GRAPH_COLOR_COUNT)

			color := &world.constraintGraph.colors[colorIndex]

			awakeJointSim := Array_Get(&color.jointSims, localIndex)

			if colorIndex != OVERFLOW_INDEX {
				// might clear a bit for a static body, but this has no effect
				ClearBit(&color.bodySet, u32(joint.edges[0].bodyId))
				ClearBit(&color.bodySet, u32(joint.edges[1].bodyId))
			}

			sleepJointIndex := sleepSet.jointSims.count
			sleepJointSim := Array_Emplace(&sleepSet.jointSims)
			mem.copy(sleepJointSim, awakeJointSim, size_of(JointSim))

			movedIndex := Array_RemoveSwap(&color.jointSims, localIndex)
			if movedIndex != NULL_INDEX {
				// fix moved element
				movedJointSim := &color.jointSims.data[localIndex]
				movedId := movedJointSim.jointId
				movedJoint := Array_Get(&world.joints, movedId)
				assert(movedJoint.localIndex == movedIndex)
				movedJoint.localIndex = localIndex
			}

			joint.setIndex = sleepSetId
			joint.colorIndex = NULL_INDEX
			joint.localIndex = sleepJointIndex
		}
	}

	// move island struct
	{
		assert(island.setIndex == i32(SetType.awakeSet))

		islandIndex := island.localIndex
		sleepIsland := Array_Emplace(&sleepSet.islandSims)
		sleepIsland.islandId = islandId

		movedIslandIndex := Array_RemoveSwap(&awakeSet.islandSims, islandIndex)
		if movedIslandIndex != NULL_INDEX {
			// fix index on moved element
			movedIslandSim := &awakeSet.islandSims.data[islandIndex]
			movedIslandId := movedIslandSim.islandId
			movedIsland := Array_Get(&world.islands, movedIslandId)
			assert(movedIsland.localIndex == movedIslandIndex)
			movedIsland.localIndex = islandIndex
		}

		island.setIndex = sleepSetId
		island.localIndex = 0
	}

	if world.splitIslandId == islandId {
		world.splitIslandId = NULL_INDEX
	}

	ValidateSolverSets(world)
}

// This is called when joints are created between sets. I want to allow the sets
// to continue sleeping if both are asleep. Otherwise one set is waked.
// Islands will get merge when the set is woke.
// Merge set 2 into set 1 then destroy set 2.
// Warning: any pointers into these sets will be orphaned.
MergeSolverSets :: proc(world: ^World, setId1: i32, setId2: i32) {
	setId1, setId2 := setId1, setId2

	assert(setId1 >= i32(SetType.firstSleepingSet))
	assert(setId2 >= i32(SetType.firstSleepingSet))
	set1 := Array_Get(&world.solverSets, setId1)
	set2 := Array_Get(&world.solverSets, setId2)

	// Move the fewest number of bodies
	if set1.bodySims.count < set2.bodySims.count {
		tempSet := set1
		set1 = set2
		set2 = tempSet

		tempId := setId1
		setId1 = setId2
		setId2 = tempId
	}

	// transfer bodies
	{
		bodies := world.bodies.data
		bodyCount := set2.bodySims.count
		for i: i32 = 0; i < bodyCount; i += 1 {
			simSrc := &set2.bodySims.data[i]

			body := &bodies[simSrc.bodyId]
			assert(body.setIndex == setId2)
			body.setIndex = setId1
			body.localIndex = set1.bodySims.count

			simDst := Array_Emplace(&set1.bodySims)
			mem.copy(simDst, simSrc, size_of(BodySim))
		}
	}

	// transfer contacts
	{
		contactCount := set2.contactIndices.count
		for i: i32 = 0; i < contactCount; i += 1 {
			contactIndex := set2.contactIndices.data[i]
			contact := Array_Get(&world.contacts, contactIndex)
			assert(contact.setIndex == setId2)
			contact.setIndex = setId1
			contact.localIndex = set1.contactIndices.count
			Array_Push(&set1.contactIndices, contactIndex)
		}
	}

	// transfer joints
	{
		jointCount := set2.jointSims.count
		for i: i32 = 0; i < jointCount; i += 1 {
			jointSrc := &set2.jointSims.data[i]

			joint := Array_Get(&world.joints, jointSrc.jointId)
			assert(joint.setIndex == setId2)
			joint.setIndex = setId1
			joint.localIndex = set1.jointSims.count

			jointDst := Array_Emplace(&set1.jointSims)
			mem.copy(jointDst, jointSrc, size_of(JointSim))
		}
	}

	// transfer islands
	{
		islandCount := set2.islandSims.count
		for i: i32 = 0; i < islandCount; i += 1 {
			islandSrc := &set2.islandSims.data[i]
			islandId := islandSrc.islandId

			island := Array_Get(&world.islands, islandId)
			island.setIndex = setId1
			island.localIndex = set1.islandSims.count

			islandDst := Array_Emplace(&set1.islandSims)
			mem.copy(islandDst, islandSrc, size_of(IslandSim))
		}
	}

	// destroy the merged set
	// Warning: need to be careful not to destroy things that got transferred, like triangle caches.
	DestroySolverSet(world, setId2)

	ValidateSolverSets(world)
}

TransferBody :: proc(world: ^World, targetSet: ^SolverSet, sourceSet: ^SolverSet, body: ^Body) {
	assert(body.headContactKey == NULL_INDEX)

	if targetSet == sourceSet {
		return
	}

	sourceIndex := body.localIndex
	sourceSim := Array_Get(&sourceSet.bodySims, sourceIndex)

	targetIndex := targetSet.bodySims.count
	targetSim := Array_Emplace(&targetSet.bodySims)
	mem.copy(targetSim, sourceSim, size_of(BodySim))

	// Clear transient body flags
	body.flags &~= bodyTransientFlags
	targetSim.flags &~= (isFast | bodyTransientFlags)

	// Remove body sim from solver set that owns it
	movedIndex := Array_RemoveSwap(&sourceSet.bodySims, sourceIndex)
	if movedIndex != NULL_INDEX {
		// Fix moved body index
		movedSim := &sourceSet.bodySims.data[sourceIndex]
		movedId := movedSim.bodyId
		movedBody := Array_Get(&world.bodies, movedId)
		assert(movedBody.localIndex == movedIndex)
		movedBody.localIndex = sourceIndex
		RefreshBodyContactIndices(world, movedBody)
	}

	if sourceSet.setIndex == i32(SetType.awakeSet) {
		Array_RemoveSwap(&sourceSet.bodyStates, sourceIndex)
	} else if targetSet.setIndex == i32(SetType.awakeSet) {
		state := Array_Emplace(&targetSet.bodyStates)
		state^ = identityBodyState
		state.flags = body.flags
	}

	body.setIndex = targetSet.setIndex
	body.localIndex = targetIndex
}

TransferJoint :: proc(world: ^World, targetSet: ^SolverSet, sourceSet: ^SolverSet, joint: ^Joint) {
	if targetSet == sourceSet {
		return
	}

	localIndex := joint.localIndex
	colorIndex := joint.colorIndex

	// Retrieve source.
	sourceSim: ^JointSim
	if sourceSet.setIndex == i32(SetType.awakeSet) {
		assert(0 <= colorIndex && colorIndex < GRAPH_COLOR_COUNT)
		color := &world.constraintGraph.colors[colorIndex]

		sourceSim = Array_Get(&color.jointSims, localIndex)
	} else {
		assert(colorIndex == NULL_INDEX)
		sourceSim = Array_Get(&sourceSet.jointSims, localIndex)
	}

	// Create target and copy. Fix joint.
	if targetSet.setIndex == i32(SetType.awakeSet) {
		AddJointToGraph(world, sourceSim, joint)
		joint.setIndex = i32(SetType.awakeSet)
	} else {
		joint.setIndex = targetSet.setIndex
		joint.localIndex = targetSet.jointSims.count
		joint.colorIndex = NULL_INDEX

		targetSim := Array_Emplace(&targetSet.jointSims)
		mem.copy(targetSim, sourceSim, size_of(JointSim))
	}

	// Destroy source.
	if sourceSet.setIndex == i32(SetType.awakeSet) {
		RemoveJointFromGraph(world, joint.edges[0].bodyId, joint.edges[1].bodyId, colorIndex, localIndex)
	} else {
		movedIndex := Array_RemoveSwap(&sourceSet.jointSims, localIndex)
		if movedIndex != NULL_INDEX {
			// fix swapped element
			movedJointSim := &sourceSet.jointSims.data[localIndex]
			movedId := movedJointSim.jointId
			movedJoint := Array_Get(&world.joints, movedId)
			movedJoint.localIndex = localIndex
		}
	}
}
