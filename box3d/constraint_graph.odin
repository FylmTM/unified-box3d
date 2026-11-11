// Ported from box3d src/constraint_graph.c and src/constraint_graph.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "core:mem"

// constraint_graph.h

// This holds constraints that cannot fit the graph color limit. This happens when a single dynamic body
// is touching many other bodies.
OVERFLOW_INDEX :: (GRAPH_COLOR_COUNT - 1)

// This keeps constraints involving two dynamic bodies at a lower solver priority than constraints
// involving a dynamic and static bodies. This reduces tunneling due to push through.
DYNAMIC_COLOR_COUNT :: (GRAPH_COLOR_COUNT - 4)

// todo optimize mesh contact constraints
// They can be lumped with convex contacts and I can use the bitset event to re-link if the manifold count increases
// Could also create a group for two wide manifolds and use bitset event
// This could create ping-pong jitter so may need to pin to high water mark or introduce hysteresis somehow
//
// Dirk has the idea to do graph coloring based on manifolds. This suggests mesh contact will have manifolds
// in multiple graph colors. So each manifold with have a color and local index.
// Some concerns about this:
// - manifolds don't have a strong identity, would this affect stability/jitter?
// - this creates a lot of static graph colors and can overflow

GraphColor :: struct {
	// This bitset is indexed by bodyId so this is over-sized to encompass static bodies
	// however I never traverse these bits or use the bit count for anything
	// This bitset is unused on the overflow color.
	bodySet:                 BitSet,

	// cache friendly arrays
	jointSims:               Array(JointSim),
	convexContacts:          Array(i32),
	contacts:                Array(ContactSpec),

	// These are used for convex contacts
	wideConstraints:         rawptr,
	wideConstraintCount:     i32,
	wideMeshConstraints:     rawptr,
	wideMeshManifoldStarts:  [^]i32,
	wideMeshConstraintCount: i32,

	// These are used for mesh and overflow contacts
	manifoldConstraints:     [^]ManifoldConstraint,
	contactConstraints:      [^]ContactConstraint,
}

ConstraintGraph :: struct {
	// including overflow at the end
	colors: [GRAPH_COLOR_COUNT]GraphColor,
}

// constraint_graph.c

// Solver using graph coloring. Islands are only used for sleep.
// High-Performance Physical Simulations on Next-Generation Architecture with Many Cores
// http://web.eecs.umich.edu/~msmelyan/papers/physsim_onmanycore_itj.pdf

// Kinematic bodies have to be treated like dynamic bodies in graph coloring. Unlike static bodies, we cannot use a dummy solver
// body for kinematic bodies. We cannot access a kinematic body from multiple threads efficiently because the SIMD solver body
// scatter would write to the same kinematic body from multiple threads. Even if these writes don't modify the body, they will
// cause horrible cache stalls. To make this feasible I would need a way to block these writes.

// This is used for debugging by making all constraints be assigned to overflow.
@(private = "file")
FORCE_OVERFLOW :: 0

@(rodata, private = "file")
graphColors := [GRAPH_COLOR_COUNT]HexColor {
	.colorRed,
	.colorOrange,
	.colorYellow,
	.colorLimeGreen,
	.colorSpringGreen,
	.colorAqua,
	.colorDodgerBlue,
	.colorBlueViolet,
	.colorMagenta,
	.colorDeepPink,
	.colorCrimson,
	.colorCoral,
	.colorGold,
	.colorGreenYellow,
	.colorMediumSeaGreen,
	.colorTurquoise,
	.colorDeepSkyBlue,
	.colorCornflowerBlue,
	.colorMediumSlateBlue,
	.colorMediumOrchid,
	.colorHotPink,
	.colorTomato,
	.colorKhaki,
	.colorSilver,
}

GetGraphColor :: proc(index: i32) -> HexColor {
	assert(0 <= index && index < GRAPH_COLOR_COUNT)
	return graphColors[index]
}

CreateGraph :: proc(graph: ^ConstraintGraph, bodyCapacity: i32) {
	#assert(GRAPH_COLOR_COUNT >= 2, "must have at least two constraint graph colors")
	#assert(OVERFLOW_INDEX == GRAPH_COLOR_COUNT - 1, "bad over flow index")

	graph^ = {}

	bodyCapacity := bodyCapacity
	bodyCapacity = MaxInt(bodyCapacity, 8)

	// Initialize graph color bit set.
	// No bitset for overflow color.
	for i: i32 = 0; i < OVERFLOW_INDEX; i += 1 {
		color := &graph.colors[i]
		color.bodySet = CreateBitSet(u32(bodyCapacity))
		SetBitCountAndClear(&color.bodySet, u32(bodyCapacity))
	}
}

DestroyGraph :: proc(graph: ^ConstraintGraph) {
	for i: i32 = 0; i < GRAPH_COLOR_COUNT; i += 1 {
		color := &graph.colors[i]

		// The bit set should never be used on the overflow color
		assert(i != OVERFLOW_INDEX || color.bodySet.bits == nil)

		DestroyBitSet(&color.bodySet)

		Array_Destroy(&color.convexContacts)
		Array_Destroy(&color.contacts)
		Array_Destroy(&color.jointSims)
	}
}

// Contacts are always created as non-touching. They get cloned into the constraint
// graph once they are found to be touching.
AddContactToGraph :: proc(world: ^World, contact: ^Contact) {
	assert(contact.manifoldCount > 0)
	assert((contact.flags & contactTouchingFlag) != 0)

	graph := &world.constraintGraph
	colorIndex: i32 = OVERFLOW_INDEX

	bodyIdA := contact.edges[0].bodyId
	bodyIdB := contact.edges[1].bodyId
	bodyA := Array_Get(&world.bodies, bodyIdA)
	bodyB := Array_Get(&world.bodies, bodyIdB)

	typeA := bodyA.type
	typeB := bodyB.type
	assert(typeA == .dynamicBody || typeB == .dynamicBody)

	when FORCE_OVERFLOW == 0 {
		if typeA == .dynamicBody && typeB == .dynamicBody {
			// Dynamic constraint colors cannot encroach on colors reserved for static constraints
			for i: i32 = 0; i < DYNAMIC_COLOR_COUNT; i += 1 {
				color := &graph.colors[i]
				if GetBit(&color.bodySet, u32(bodyIdA)) || GetBit(&color.bodySet, u32(bodyIdB)) {
					continue
				}

				SetBitGrow(&color.bodySet, u32(bodyIdA))
				SetBitGrow(&color.bodySet, u32(bodyIdB))
				colorIndex = i
				break
			}
		} else if typeA == .dynamicBody {
			// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
			for i: i32 = OVERFLOW_INDEX - 1; i >= 1; i -= 1 {
				color := &graph.colors[i]
				if GetBit(&color.bodySet, u32(bodyIdA)) {
					continue
				}

				SetBitGrow(&color.bodySet, u32(bodyIdA))
				colorIndex = i
				break
			}
		} else if typeB == .dynamicBody {
			// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
			for i: i32 = OVERFLOW_INDEX - 1; i >= 1; i -= 1 {
				color := &graph.colors[i]
				if GetBit(&color.bodySet, u32(bodyIdB)) {
					continue
				}

				SetBitGrow(&color.bodySet, u32(bodyIdB))
				colorIndex = i
				break
			}
		}
	}

	isScalar := (contact.flags & simMeshContact) != 0 || colorIndex == OVERFLOW_INDEX

	color := &graph.colors[colorIndex]
	contact.colorIndex = colorIndex
	contact.localIndex = isScalar ? color.contacts.count : color.convexContacts.count
	contact.encodedBodySimA = EncodeBodySimIndex(bodyA)
	contact.encodedBodySimB = EncodeBodySimIndex(bodyB)

	if isScalar {
		assert(contact.manifoldCount < i32(max(u16)))
		spec := ContactSpec {
			contactId     = contact.contactId,
			manifoldStart = 0,
			manifoldCount = u16(contact.manifoldCount),
		}
		Array_Push(&color.contacts, spec)
	} else {
		Array_Push(&color.convexContacts, contact.contactId)
	}
}

RemoveContactFromGraph :: proc(
	world: ^World,
	bodyIdA: i32,
	bodyIdB: i32,
	colorIndex: i32,
	localIndex: i32,
	meshContact: bool,
) {
	graph := &world.constraintGraph

	assert(0 <= colorIndex && colorIndex < GRAPH_COLOR_COUNT)
	color := &graph.colors[colorIndex]

	if colorIndex != OVERFLOW_INDEX {
		// This might clear a bit for a static body, but this has no effect
		ClearBit(&color.bodySet, u32(bodyIdA))
		ClearBit(&color.bodySet, u32(bodyIdB))
	}

	if meshContact || colorIndex == OVERFLOW_INDEX {
		movedIndex := Array_RemoveSwap(&color.contacts, localIndex)
		if movedIndex != NULL_INDEX {
			// Fix index on swapped contact
			movedContactId := color.contacts.data[localIndex].contactId
			movedContact := Array_Get(&world.contacts, movedContactId)
			assert(movedContact.setIndex == i32(SetType.awakeSet))
			assert(movedContact.colorIndex == colorIndex)
			assert(movedContact.localIndex == movedIndex)
			movedContact.localIndex = localIndex
		}
	} else {
		movedIndex := Array_RemoveSwap(&color.convexContacts, localIndex)
		if movedIndex != NULL_INDEX {
			// Fix index on swapped contact
			movedContactId := color.convexContacts.data[localIndex]
			movedContact := Array_Get(&world.contacts, movedContactId)
			assert(movedContact.setIndex == i32(SetType.awakeSet))
			assert(movedContact.colorIndex == colorIndex)
			assert(movedContact.localIndex == movedIndex)
			assert((movedContact.flags & simMeshContact) == 0)
			movedContact.localIndex = localIndex
		}
	}
}

@(private = "file")
AssignJointColor :: proc(
	graph: ^ConstraintGraph,
	bodyIdA: i32,
	bodyIdB: i32,
	typeA: BodyType,
	typeB: BodyType,
) -> i32 {
	assert(typeA == .dynamicBody || typeB == .dynamicBody)
	_ = graph
	_ = bodyIdA
	_ = bodyIdB
	_ = typeA
	_ = typeB

	when FORCE_OVERFLOW == 0 {
		if typeA == .dynamicBody && typeB == .dynamicBody {
			// Dynamic constraint colors cannot encroach on colors reserved for static constraints
			for i: i32 = 0; i < DYNAMIC_COLOR_COUNT; i += 1 {
				color := &graph.colors[i]
				if GetBit(&color.bodySet, u32(bodyIdA)) || GetBit(&color.bodySet, u32(bodyIdB)) {
					continue
				}

				SetBitGrow(&color.bodySet, u32(bodyIdA))
				SetBitGrow(&color.bodySet, u32(bodyIdB))
				return i
			}
		} else if typeA == .dynamicBody {
			// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
			for i: i32 = OVERFLOW_INDEX - 1; i >= 1; i -= 1 {
				color := &graph.colors[i]
				if GetBit(&color.bodySet, u32(bodyIdA)) {
					continue
				}

				SetBitGrow(&color.bodySet, u32(bodyIdA))
				return i
			}
		} else if typeB == .dynamicBody {
			// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
			for i: i32 = OVERFLOW_INDEX - 1; i >= 1; i -= 1 {
				color := &graph.colors[i]
				if GetBit(&color.bodySet, u32(bodyIdB)) {
					continue
				}

				SetBitGrow(&color.bodySet, u32(bodyIdB))
				return i
			}
		}
	}

	return OVERFLOW_INDEX
}

CreateJointInGraph :: proc(world: ^World, joint: ^Joint) -> ^JointSim {
	graph := &world.constraintGraph

	bodyIdA := joint.edges[0].bodyId
	bodyIdB := joint.edges[1].bodyId
	bodyA := Array_Get(&world.bodies, bodyIdA)
	bodyB := Array_Get(&world.bodies, bodyIdB)

	colorIndex := AssignJointColor(graph, bodyIdA, bodyIdB, bodyA.type, bodyB.type)

	jointSim := Array_Emplace(&graph.colors[colorIndex].jointSims)
	mem.zero(jointSim, size_of(JointSim))

	joint.colorIndex = colorIndex
	joint.localIndex = graph.colors[colorIndex].jointSims.count - 1
	return jointSim
}

AddJointToGraph :: proc(world: ^World, jointSim: ^JointSim, joint: ^Joint) {
	jointDst := CreateJointInGraph(world, joint)
	mem.copy(jointDst, jointSim, size_of(JointSim))
}

RemoveJointFromGraph :: proc(world: ^World, bodyIdA: i32, bodyIdB: i32, colorIndex: i32, localIndex: i32) {
	graph := &world.constraintGraph

	assert(0 <= colorIndex && colorIndex < GRAPH_COLOR_COUNT)
	color := &graph.colors[colorIndex]

	if colorIndex != OVERFLOW_INDEX {
		// May clear static bodies, no effect
		ClearBit(&color.bodySet, u32(bodyIdA))
		ClearBit(&color.bodySet, u32(bodyIdB))
	}

	movedIndex := Array_RemoveSwap(&color.jointSims, localIndex)
	if movedIndex != NULL_INDEX {
		// Fix moved joint
		movedJointSim := &color.jointSims.data[localIndex]
		movedId := movedJointSim.jointId
		movedJoint := Array_Get(&world.joints, movedId)
		assert(movedJoint.setIndex == i32(SetType.awakeSet))
		assert(movedJoint.colorIndex == colorIndex)
		assert(movedJoint.localIndex == movedIndex)
		movedJoint.localIndex = localIndex
	}
}
