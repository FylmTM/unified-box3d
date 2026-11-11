// Ported from box3d test/test_broad_phase.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "base:runtime"
import "core:testing"

// Brute force check of the broad-phase pair set against every live shape pair. Must run right
// after a zero time step so the pair update and the disjoint contact purge have both happened
// against the current fat boxes.
@(private = "file")
CheckPairSet :: proc(worldId: b3.WorldId) -> i32 {
	world := b3.GetWorldFromId(worldId)
	bp := &world.broadPhase
	shapes := world.shapes.data
	fatAABBs := world.fatAABBs.data
	shapeCount := world.shapes.count

	for i: i32 = 0; i < shapeCount; i += 1 {
		shapeA := &shapes[i]
		if shapeA.id == b3.NULL_INDEX || shapeA.proxyKey == b3.NULL_INDEX || shapeA.sensorIndex != b3.NULL_INDEX {
			continue
		}

		bodyA := b3.Array_Get(&world.bodies, shapeA.bodyId)

		for j: i32 = i + 1; j < shapeCount; j += 1 {
			shapeB := &shapes[j]
			if shapeB.id == b3.NULL_INDEX || shapeB.proxyKey == b3.NULL_INDEX || shapeB.sensorIndex != b3.NULL_INDEX {
				continue
			}

			if shapeA.bodyId == shapeB.bodyId {
				continue
			}

			bodyB := b3.Array_Get(&world.bodies, shapeB.bodyId)

			expected :=
				b3.ShouldShapesCollide(shapeA.filter, shapeB.filter) &&
				b3.ShouldBodiesCollide(world, bodyA, bodyB) &&
				b3.AABB_Overlaps(fatAABBs[i], fatAABBs[j])

			actual := b3.ContainsKey(&bp.pairSet, b3.ShapePairKey(i, j, 0))

			// A sleeping contact is never refreshed, so a stale pair can outlive the overlap when
			// the other body is teleported, and a set woken by the narrow phase carries its stale
			// contacts into the awake set one collide late. With sleeping on only a missing pair
			// is a bug.
			if world.enableSleep && actual && expected == false {
				continue
			}

			if expected != actual {
				printf(
					"  pair mismatch: shapes %d (body %d type %d set %d) and %d (body %d type %d set %d), expected %d, " +
					"actual %d\n",
					i,
					shapeA.bodyId,
					bodyA.type,
					bodyA.setIndex,
					j,
					shapeB.bodyId,
					bodyB.type,
					bodyB.setIndex,
					expected,
					actual,
				)
				return 1
			}
		}
	}

	return 0
}

@(private = "file")
RandomState :: struct {
	seed: u32,
}

@(private = "file")
NextRandom :: proc(state: ^RandomState) -> u32 {
	// xorshift32
	x := state.seed
	x ~= x << 13
	x ~= x >> 17
	x ~= x << 5
	state.seed = x
	return x
}

@(private = "file")
RandomFloat :: proc(state: ^RandomState, lower: f32, upper: f32) -> f32 {
	u := f32(NextRandom(state) & 0xFFFFFF) / f32(0x1000000)
	return lower + (upper - lower) * u
}

@(private = "file")
RandomInt :: proc(state: ^RandomState, count: i32) -> i32 {
	return i32(NextRandom(state) % u32(count))
}

@(private = "file")
MAX_BODIES :: 64
@(private = "file")
STATIC_COUNT :: 60
@(private = "file")
TILE_COUNT :: 9

@(private = "file")
BodySlot :: struct {
	bodyId: b3.BodyId,
	active: bool,
}

@(private = "file")
StressContext :: struct {
	worldId:    b3.WorldId,
	random:     RandomState,
	origin:     b3.Pos,
	extent:     f32,
	slots:      [MAX_BODIES]BodySlot,
	joints:     [4]b3.JointId,
	jointCount: i32,
	box:        b3.BoxHull,
}

@(private = "file")
RandomPosition :: proc(context_: ^StressContext) -> b3.Pos {
	e := context_.extent
	offset := b3.Vec3 {
		RandomFloat(&context_.random, -e, e),
		RandomFloat(&context_.random, -e, e),
		RandomFloat(&context_.random, -e, e),
	}
	return b3.OffsetPos(context_.origin, offset)
}

@(private = "file")
RandomVelocity :: proc(context_: ^StressContext) -> b3.Vec3 {
	// Some bodies outrun their margin every step and go through continuous collision
	s: f32 = RandomInt(&context_.random, 4) == 0 ? 40.0 : 12.0
	return b3.Vec3 {
		RandomFloat(&context_.random, -s, s),
		RandomFloat(&context_.random, -s, s),
		RandomFloat(&context_.random, -s, s),
	}
}

@(private = "file")
CreateSlotBody :: proc(context_: ^StressContext, slot: i32) {
	random := &context_.random

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = RandomInt(random, 10) == 0 ? .kinematicBody : .dynamicBody
	bodyDef.position = RandomPosition(context_)
	bodyDef.linearVelocity = RandomVelocity(context_)
	bodyDef.isBullet = RandomInt(random, 3) == 0
	bodyDef.gravityScale = 0.0
	bodyId := b3.CreateBody(context_.worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()
	shapeDef.filter.categoryBits = u64(1) << u64(RandomInt(random, 3))
	shapeDef.filter.maskBits = RandomInt(random, 4) == 0 ? 0x5 : b3.DEFAULT_MASK_BITS

	shapeCount := 1 + RandomInt(random, 2)
	for i: i32 = 0; i < shapeCount; i += 1 {
		if RandomInt(random, 2) == 0 {
			sphere := b3.Sphere{{RandomFloat(random, -1.0, 1.0), 0.0, 0.0}, RandomFloat(random, 0.2, 1.5)}
			b3.CreateSphereShape(bodyId, &shapeDef, &sphere)
		} else {
			b3.CreateHullShape(bodyId, &shapeDef, &context_.box.base)
		}
	}

	context_.slots[slot].bodyId = bodyId
	context_.slots[slot].active = true
}

// A swarm of bodies churning through every proxy mutation the world API offers, with the pair
// set checked against a brute force scan after every step.
@(private = "file")
RunBroadPhaseStress :: proc(workerCount: i32, seed: u32, enableSleep: bool) -> i32 {
	worldDef := b3.DefaultWorldDef()
	worldDef.workerCount = u32(workerCount)
	worldDef.enableSleep = enableSleep
	worldDef.gravity = b3.Vec3_zero
	worldId := b3.CreateWorld(&worldDef)

	context_ := StressContext{}
	context_.worldId = worldId
	context_.random.seed = seed
	context_.origin = b3.Pos{-1900.0, 0.0, 1330.0}
	context_.extent = 12.0
	context_.box = b3.MakeBoxHull(0.95, 0.6, 2.2)

	// Kilometre tiles far from the origin under the swarm, like a streamed terrain
	tile := b3.CreateGrid(65, 65, b3.Vec3{16.0, 1.0, 16.0}, false)
	for i: i32 = 0; i < TILE_COUNT; i += 1 {
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .staticBody
		bodyDef.position = b3.Pos{-3072.0 + 1024.0 * f32(i % 3), -30.0, 1024.0 * f32(i / 3)}
		tileId := b3.CreateBody(worldId, &bodyDef)
		shapeDef := b3.DefaultShapeDef()
		shapeDef.enableSpeculativeContact = false
		b3.CreateHeightFieldShape(tileId, &shapeDef, tile)
	}

	// Static clutter inside the swarm. Some pieces ask for contacts on creation.
	staticIds: [STATIC_COUNT]b3.BodyId
	grid := b3.CreateGrid(5, 5, b3.Vec3{2.0, 1.0, 2.0}, false)
	staticBox := b3.MakeBoxHull(3.0, 0.5, 3.0)
	for i: i32 = 0; i < STATIC_COUNT; i += 1 {
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .staticBody
		bodyDef.position = RandomPosition(&context_)
		staticIds[i] = b3.CreateBody(worldId, &bodyDef)

		shapeDef := b3.DefaultShapeDef()
		shapeDef.invokeContactCreation = (i % 4) == 0
		shapeDef.enableSpeculativeContact = (i % 2) == 0

		if i % 3 == 0 {
			b3.CreateHeightFieldShape(staticIds[i], &shapeDef, grid)
		} else {
			b3.CreateHullShape(staticIds[i], &shapeDef, &staticBox.base)
		}
	}

	for i: i32 = 0; i < MAX_BODIES; i += 1 {
		if RandomInt(&context_.random, 4) != 0 {
			CreateSlotBody(&context_, i)
		}
	}

	timeStep: f32 = 1.0 / 60.0
	stepCount: i32 = 400

	for step: i32 = 0; step < stepCount; step += 1 {
		random := &context_.random

		mutationCount := RandomInt(random, 4)
		for m: i32 = 0; m < mutationCount; m += 1 {
			slot := RandomInt(random, MAX_BODIES)
			body := &context_.slots[slot]
			action := RandomInt(random, 12)

			if body.active == false {
				if action < 6 {
					CreateSlotBody(&context_, slot)
				}
				continue
			}

			switch action {
			case 0:
				b3.DestroyBody(body.bodyId)
				body.active = false

			case 1, 2:
				b3.Body_SetTransform(body.bodyId, RandomPosition(&context_), b3.Quat_identity)

			case 3:
				b3.Body_SetLinearVelocity(body.bodyId, RandomVelocity(&context_))

			case 4:
				b3.Body_Disable(body.bodyId)

			case 5:
				b3.Body_Enable(body.bodyId)

			case 6:
				b3.Body_SetBullet(body.bodyId, RandomInt(random, 2) == 0)

			case 7:
				{
					types := [3]b3.BodyType{.staticBody, .kinematicBody, .dynamicBody}
					b3.Body_SetType(body.bodyId, types[RandomInt(random, 3)])
					if b3.Body_GetType(body.bodyId) != .staticBody {
						b3.Body_SetLinearVelocity(body.bodyId, RandomVelocity(&context_))
					}
				}

			case 8:
				{
					// Add a joint that blocks collision, or toggle one
					if context_.jointCount < 4 {
						otherSlot := RandomInt(random, MAX_BODIES)
						other := &context_.slots[otherSlot]
						if otherSlot != slot &&
						   other.active &&
						   b3.Body_GetType(body.bodyId) == .dynamicBody &&
						   b3.Body_GetType(other.bodyId) == .dynamicBody {
							jointDef := b3.DefaultWeldJointDef()
							jointDef.base.bodyIdA = body.bodyId
							jointDef.base.bodyIdB = other.bodyId
							jointDef.base.collideConnected = false
							jointIndex := context_.jointCount
							context_.jointCount += 1
							context_.joints[jointIndex] = b3.CreateWeldJoint(worldId, &jointDef)
						}
					} else {
						jointId := context_.joints[RandomInt(random, context_.jointCount)]
						if b3.Joint_IsValid(jointId) {
							b3.Joint_SetCollideConnected(jointId, RandomInt(random, 2) == 0)
						}
					}
				}

			case 9:
				{
					staticIndex := RandomInt(random, STATIC_COUNT)
					b3.Body_SetTransform(staticIds[staticIndex], RandomPosition(&context_), b3.Quat_identity)
				}

			case 10:
				b3.World_RebuildStaticTree(worldId)

			case:
			}
		}

		// Joints whose bodies were destroyed are gone too
		for j: i32 = 0; j < context_.jointCount; {
			if b3.Joint_IsValid(context_.joints[j]) == false {
				context_.joints[j] = context_.joints[context_.jointCount - 1]
				context_.jointCount -= 1
			} else {
				j += 1
			}
		}

		// Keep the swarm inside the clutter
		for i: i32 = 0; i < MAX_BODIES; i += 1 {
			body := &context_.slots[i]
			if body.active == false || b3.Body_IsEnabled(body.bodyId) == false {
				continue
			}

			p := b3.Body_GetPosition(body.bodyId)
			d := b3.SubPos(p, context_.origin)
			if b3.AbsFloat(d.x) > 2.0 * context_.extent ||
			   b3.AbsFloat(d.y) > 2.0 * context_.extent ||
			   b3.AbsFloat(d.z) > 2.0 * context_.extent {
				b3.Body_SetTransform(body.bodyId, RandomPosition(&context_), b3.Quat_identity)
				b3.Body_SetLinearVelocity(body.bodyId, RandomVelocity(&context_))
			}
		}

		b3.World_Step(worldId, timeStep, 4)

		// A zero step finds pairs and purges disjoint contacts without moving anything
		b3.World_Step(worldId, 0.0, 1)

		if CheckPairSet(worldId) != 0 {
			printf("  workers=%d seed=%u sleep=%d step=%d\n", workerCount, seed, enableSleep, step)
			b3.DestroyWorld(worldId)
			b3.DestroyHeightField(grid)
			b3.DestroyHeightField(tile)
			return 1
		}
	}

	b3.DestroyWorld(worldId)
	b3.DestroyHeightField(grid)
	b3.DestroyHeightField(tile)
	return 0
}

@(private = "file")
BroadPhaseStressTest :: proc(t: ^testing.T) -> bool {
	seeds := [3]u32{12345, 0xC0FFEE, 777}
	for i := 0; i < 3; i += 1 {
		ENSURE(t, RunBroadPhaseStress(1, seeds[i], false) == 0) or_return
		ENSURE(t, RunBroadPhaseStress(4, seeds[i], false) == 0) or_return
		ENSURE(t, RunBroadPhaseStress(3, seeds[i], true) == 0) or_return
	}

	return true
}

// The broad-phase keeps its pending pair marks inside the static tree, so a full rebuild between
// a static mutation and the next step used to wipe them and the contact never appeared. Covers
// both ways a static proxy gets marked: a teleport and a shape created with invokeContactCreation.
@(private = "file")
RunStaticRebuildKeepsMarks :: proc(t: ^testing.T, useInvoke: bool) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldDef.gravity = b3.Vec3_zero
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = b3.Pos{0.0, 1.0, 0.0}
	sphereId := b3.CreateBody(worldId, &bodyDef)
	shapeDef := b3.DefaultShapeDef()
	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}
	b3.CreateSphereShape(sphereId, &shapeDef, &sphere)

	bodyDef.type = .staticBody
	bodyDef.position = b3.Pos{100.0, 0.0, 0.0}
	groundId := b3.CreateBody(worldId, &bodyDef)
	box := b3.MakeBoxHull(2.0, 0.5, 2.0)
	if useInvoke == false {
		b3.CreateHullShape(groundId, &shapeDef, &box.base)
	}

	// Settle so the resting sphere carries no mark of its own
	for i := 0; i < 3; i += 1 {
		b3.World_Step(worldId, f32(1.0) / f32(60.0), 4)
	}

	b3.Body_SetTransform(groundId, b3.Pos_zero, b3.Quat_identity)
	if useInvoke {
		invokeDef := b3.DefaultShapeDef()
		invokeDef.invokeContactCreation = true
		b3.CreateHullShape(groundId, &invokeDef, &box.base)
	}

	b3.World_RebuildStaticTree(worldId)
	b3.World_Step(worldId, f32(1.0) / f32(60.0), 4)

	ENSURE(t, b3.Body_GetContactCapacity(sphereId) == 1) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
StaticRebuildKeepsMarks :: proc(t: ^testing.T) -> bool {
	ENSURE(t, RunStaticRebuildKeepsMarks(t, false)) or_return
	ENSURE(t, RunStaticRebuildKeepsMarks(t, true)) or_return
	return true
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
FlushingAssert :: proc "c" (condition: cstring, fileName: cstring, lineNumber: i32) -> i32 {
	context = runtime.default_context()
	// unified end
	printf("BOX3D ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber)
	return 1
}

@(test)
BroadPhaseTest :: proc(t: ^testing.T) {
	LOCK()

	// A crash through a pipe loses buffered output, so flush the assertion first
	b3.SetAssertFcn(FlushingAssert)

	if !RUN_SUBTEST(t, StaticRebuildKeepsMarks) {return}
	if !RUN_SUBTEST(t, BroadPhaseStressTest) {return}
}
