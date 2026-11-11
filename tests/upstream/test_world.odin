// Ported from box3d test/test_world.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import "base:intrinsics"
import "base:runtime"
import "core:testing"

import b3 "../../box3d"

// This is a simple example of building and running a simulation
// using Box3D. Here we create a large ground box and a small dynamic
// box.
// There are no graphics for this example. Box3D is meant to be used
// with your rendering engine in your game engine.
@(private = "file")
HelloWorld :: proc(t: ^testing.T) -> bool {
	// Construct a world object, which will hold and simulate the rigid bodies.
	worldDef := b3.DefaultWorldDef()
	worldDef.gravity = b3.Vec3{0.0, -10.0, 0.0}

	worldId := b3.CreateWorld(&worldDef)
	ENSURE(t, b3.World_IsValid(worldId)) or_return

	// Define the ground body.
	groundBodyDef := b3.DefaultBodyDef()
	groundBodyDef.position = b3.Pos{0.0, -10.0, 0.0}

	// Call the body factory which allocates memory for the ground body
	// from a pool and creates the ground box shape (also from a pool).
	// The body is also added to the world.
	groundId := b3.CreateBody(worldId, &groundBodyDef)
	ENSURE(t, b3.Body_IsValid(groundId)) or_return

	// Define the ground box shape. The extents are the half-widths of the box.
	groundBox := b3.MakeBoxHull(50.0, 10.0, 50.0)

	// Add the box shape to the ground body.
	groundShapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(groundId, &groundShapeDef, &groundBox.base)

	// Define the dynamic body. We set its position and call the body factory.
	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = b3.Pos{0.0, 4.0, 0.0}

	bodyId := b3.CreateBody(worldId, &bodyDef)

	// Define another box shape for our dynamic body.
	dynamicBox := b3.MakeCubeHull(1.0)

	// Define the dynamic body shape
	shapeDef := b3.DefaultShapeDef()

	// Set the box density to be non-zero, so it will be dynamic.
	shapeDef.density = 1.0

	// Override the default friction.
	shapeDef.baseMaterial.friction = 0.3

	// Add the shape to the body.
	b3.CreateHullShape(bodyId, &shapeDef, &dynamicBox.base)

	// Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 4 sub-steps. This provides a high quality simulation
	// in most game scenarios.
	timeStep: f32 = 1.0 / 60.0
	subStepCount: i32 = 4

	position := b3.Body_GetPosition(bodyId)
	rotation := b3.Body_GetRotation(bodyId)

	// This is our little game loop.
	for i: i32 = 0; i < 90; i += 1 {
		// Instruct the world to perform a single step of simulation.
		// It is generally best to keep the time step and iterations fixed.
		b3.World_Step(worldId, timeStep, subStepCount)

		// Now print the position and angle of the body.
		position = b3.Body_GetPosition(bodyId)
		rotation = b3.Body_GetRotation(bodyId)

		// printf("%4.2f %4.2f %4.2f\n", position.x, position.y, b3Rot_GetAngle(rotation));
	}

	// When the world destructor is called, all bodies and joints are freed. This can
	// create orphaned ids, so be careful about your world management.
	b3.DestroyWorld(worldId)

	ENSURE_SMALL(t, position.y - 1.00, 0.01) or_return
	ENSURE_SMALL(t, rotation.v.x, 0.01) or_return
	ENSURE_SMALL(t, rotation.v.z, 0.01) or_return

	return true
}

@(private = "file")
EmptyWorld :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)
	ENSURE(t, b3.World_IsValid(worldId) == true) or_return

	timeStep: f32 = 1.0 / 60.0
	subStepCount: i32 = 1

	for i: i32 = 0; i < 60; i += 1 {
		b3.World_Step(worldId, timeStep, subStepCount)
	}

	b3.DestroyWorld(worldId)

	ENSURE(t, b3.World_IsValid(worldId) == false) or_return

	return true
}

@(private = "file")
BODY_COUNT :: 10

@(private = "file")
DestroyAllBodiesWorld :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)
	ENSURE(t, b3.World_IsValid(worldId) == true) or_return

	count: i32 = 0
	creating := true

	bodyIds: [BODY_COUNT]b3.BodyId
	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	cube := b3.MakeCubeHull(0.5)

	for i: i32 = 0; i < 2 * BODY_COUNT + 10; i += 1 {
		if creating {
			if count < BODY_COUNT {
				bodyIds[count] = b3.CreateBody(worldId, &bodyDef)

				shapeDef := b3.DefaultShapeDef()
				b3.CreateHullShape(bodyIds[count], &shapeDef, &cube.base)
				count += 1
			} else {
				creating = false
			}
		} else if count > 0 {
			b3.DestroyBody(bodyIds[count - 1])
			bodyIds[count - 1] = b3.nullBodyId
			count -= 1
		}

		b3.World_Step(worldId, 1.0 / 60.0, 3)
	}

	counters := b3.World_GetCounters(worldId)
	ENSURE(t, counters.bodyCount == 0) or_return

	b3.DestroyWorld(worldId)

	ENSURE(t, b3.World_IsValid(worldId) == false) or_return

	return true
}

@(private = "file")
TestIsValid :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)
	ENSURE(t, b3.World_IsValid(worldId)) or_return

	bodyDef := b3.DefaultBodyDef()

	bodyId1 := b3.CreateBody(worldId, &bodyDef)
	ENSURE(t, b3.Body_IsValid(bodyId1) == true) or_return

	bodyId2 := b3.CreateBody(worldId, &bodyDef)
	ENSURE(t, b3.Body_IsValid(bodyId2) == true) or_return

	b3.DestroyBody(bodyId1)
	ENSURE(t, b3.Body_IsValid(bodyId1) == false) or_return

	b3.DestroyBody(bodyId2)
	ENSURE(t, b3.Body_IsValid(bodyId2) == false) or_return

	b3.DestroyWorld(worldId)

	ENSURE(t, b3.World_IsValid(worldId) == false) or_return
	ENSURE(t, b3.Body_IsValid(bodyId2) == false) or_return
	ENSURE(t, b3.Body_IsValid(bodyId1) == false) or_return

	return true
}

@(private = "file")
WORLD_COUNT :: (b3.MAX_WORLDS / 2)

@(private = "file")
TestWorldRecycle :: proc(t: ^testing.T) -> bool {
	#assert(WORLD_COUNT > 0, "world count")

	count: i32 = 100

	worldIds: [WORLD_COUNT]b3.WorldId

	for i: i32 = 0; i < count; i += 1 {
		worldDef := b3.DefaultWorldDef()
		for j: i32 = 0; j < WORLD_COUNT; j += 1 {
			worldIds[j] = b3.CreateWorld(&worldDef)
			ENSURE(t, b3.World_IsValid(worldIds[j]) == true) or_return

			bodyDef := b3.DefaultBodyDef()
			b3.CreateBody(worldIds[j], &bodyDef)
		}

		for j: i32 = 0; j < WORLD_COUNT; j += 1 {
			timeStep: f32 = 1.0 / 60.0
			subStepCount: i32 = 1

			for k: i32 = 0; k < 10; k += 1 {
				b3.World_Step(worldIds[j], timeStep, subStepCount)
			}
		}

		for j: i32 = WORLD_COUNT - 1; j >= 0; j -= 1 {
			b3.DestroyWorld(worldIds[j])
			ENSURE(t, b3.World_IsValid(worldIds[j]) == false) or_return
			worldIds[j] = b3.nullWorldId
		}
	}

	return true
}

// unified: C's ENSURE inside these bool callbacks returns 1 (true) on failure and only prints; there is no
// testing.T here, so the check keeps that behaviour: return true early.
@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
CustomFilter :: proc "c" (shapeIdA: b3.ShapeId, shapeIdB: b3.ShapeId, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	_ = shapeIdA
	_ = shapeIdB
	if context_ != nil {return true}
	return true
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
PreSolveStatic :: proc "c" (
	shapeIdA: b3.ShapeId,
	shapeIdB: b3.ShapeId,
	point: b3.Pos,
	normal: b3.Vec3,
	context_: rawptr,
) -> bool {
	context = runtime.default_context()
	// unified end
	_ = shapeIdA
	_ = shapeIdB
	_ = point
	_ = normal
	if context_ != nil {return true}
	return false
}
// unified end

// This test is here to ensure all API functions link correctly.
@(private = "file")
TestWorldCoverage :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()

	worldId := b3.CreateWorld(&worldDef)
	ENSURE(t, b3.World_IsValid(worldId)) or_return

	b3.World_EnableSleeping(worldId, true)
	b3.World_EnableSleeping(worldId, false)
	flag := b3.World_IsSleepingEnabled(worldId)
	ENSURE(t, flag == false) or_return

	b3.World_EnableContinuous(worldId, false)
	b3.World_EnableContinuous(worldId, true)
	flag = b3.World_IsContinuousEnabled(worldId)
	ENSURE(t, flag == true) or_return

	b3.World_SetRestitutionThreshold(worldId, 0.0)
	b3.World_SetRestitutionThreshold(worldId, 2.0)
	value := b3.World_GetRestitutionThreshold(worldId)
	ENSURE(t, value == 2.0) or_return

	b3.World_SetHitEventThreshold(worldId, 0.0)
	b3.World_SetHitEventThreshold(worldId, 100.0)
	value = b3.World_GetHitEventThreshold(worldId)
	ENSURE(t, value == 100.0) or_return

	b3.World_SetCustomFilterCallback(worldId, CustomFilter, nil)
	b3.World_SetPreSolveCallback(worldId, PreSolveStatic, nil)

	g := b3.Vec3{1.0, 2.0, 0.0}
	b3.World_SetGravity(worldId, g)
	v := b3.World_GetGravity(worldId)
	ENSURE(t, v.x == g.x) or_return
	ENSURE(t, v.y == g.y) or_return

	explosionDef := b3.DefaultExplosionDef()
	b3.World_Explode(worldId, &explosionDef)

	b3.World_SetContactTuning(worldId, 10.0, 2.0, 4.0)

	b3.World_SetMaximumLinearSpeed(worldId, 10.0)
	value = b3.World_GetMaximumLinearSpeed(worldId)
	ENSURE(t, value == 10.0) or_return

	b3.World_SetIsotropicInertiaTolerance(worldId, 0.5)
	value = b3.World_GetIsotropicInertiaTolerance(worldId)
	ENSURE(t, value == 0.5) or_return

	b3.World_SetIsotropicInertiaTolerance(worldId, 2.0)
	value = b3.World_GetIsotropicInertiaTolerance(worldId)
	ENSURE(t, value == 1.0) or_return

	b3.World_EnableWarmStarting(worldId, true)
	flag = b3.World_IsWarmStartingEnabled(worldId)
	ENSURE(t, flag == true) or_return

	count := b3.World_GetAwakeBodyCount(worldId)
	ENSURE(t, count == 0) or_return

	b3.World_SetUserData(worldId, &value)
	userData := b3.World_GetUserData(worldId)
	ENSURE(t, userData == rawptr(&value)) or_return

	b3.World_Step(worldId, 1.0, 1)

	b3.DestroyWorld(worldId)

	return true
}

@(private = "file")
TestSensor :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	// Wall from x = 1 to x = 2
	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .staticBody
	bodyDef.position = b3.Pos{1.5, 11.0, 0.0}
	wallId := b3.CreateBody(worldId, &bodyDef)
	box := b3.MakeBoxHull(0.5, 10.0, 1.0)
	shapeDef := b3.DefaultShapeDef()
	shapeDef.enableSensorEvents = true
	b3.CreateHullShape(wallId, &shapeDef, &box.base)

	// Bullet fired towards the wall
	bodyDef = b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.isBullet = true
	bodyDef.gravityScale = 0.0
	bodyDef.position = b3.Pos{7.39814, 4.0, 0.0}
	bodyDef.linearVelocity = b3.Vec3{-20.0, 0.0, 0.0}
	bulletId := b3.CreateBody(worldId, &bodyDef)
	shapeDef = b3.DefaultShapeDef()
	shapeDef.isSensor = true
	shapeDef.enableSensorEvents = true
	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.1}
	b3.CreateSphereShape(bulletId, &shapeDef, &sphere)

	beginCount: i32 = 0
	endCount: i32 = 0

	for true {
		timeStep: f32 = 1.0 / 60.0
		subStepCount: i32 = 4
		b3.World_Step(worldId, timeStep, subStepCount)

		bulletPos := b3.Body_GetPosition(bulletId)
		// printf( "Bullet pos: %g %g\n", bulletPos.x, bulletPos.y );

		events := b3.World_GetSensorEvents(worldId)

		if events.beginCount > 0 {
			beginCount += 1
		}

		if events.endCount > 0 {
			endCount += 1
		}

		if bulletPos.x < -1.0 {
			break
		}
	}

	b3.DestroyWorld(worldId)

	ENSURE(t, beginCount == 1) or_return
	ENSURE(t, endCount == 1) or_return

	return true
}

@(private = "file")
TestContactEvents :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	// Static ground
	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .staticBody
	bodyDef.position = b3.Pos{0.0, -0.5, 0.0}
	groundId := b3.CreateBody(worldId, &bodyDef)
	groundBox := b3.MakeBoxHull(10.0, 0.5, 10.0)
	groundShapeDef := b3.DefaultShapeDef()
	groundShapeId := b3.CreateHullShape(groundId, &groundShapeDef, &groundBox.base)

	// Dynamic sphere dropped onto the ground; restitution causes it to bounce so we get end events
	bodyDef = b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = b3.Pos{0.0, 5.0, 0.0}
	sphereBodyId := b3.CreateBody(worldId, &bodyDef)
	shapeDef := b3.DefaultShapeDef()
	shapeDef.density = 1.0
	shapeDef.enableContactEvents = true
	shapeDef.baseMaterial.restitution = 0.6
	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}
	sphereShapeId := b3.CreateSphereShape(sphereBodyId, &shapeDef, &sphere)

	beginCount: i32 = 0
	endCount: i32 = 0
	idsChecked := false

	for i: i32 = 0; i < 120; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)

		events := b3.World_GetContactEvents(worldId)

		if events.beginCount > 0 && idsChecked == false {
			be := events.beginEvents[0]
			aIsSphere := b3.ID_EQUALS(be.shapeIdA, sphereShapeId)
			bIsSphere := b3.ID_EQUALS(be.shapeIdB, sphereShapeId)
			aIsGround := b3.ID_EQUALS(be.shapeIdA, groundShapeId)
			bIsGround := b3.ID_EQUALS(be.shapeIdB, groundShapeId)
			ENSURE(t, (aIsSphere && bIsGround) || (aIsGround && bIsSphere)) or_return
			ENSURE(t, b3.Contact_IsValid(be.contactId)) or_return
			idsChecked = true
		}

		beginCount += events.beginCount
		endCount += events.endCount
	}

	b3.DestroyWorld(worldId)

	ENSURE(t, idsChecked) or_return
	ENSURE(t, beginCount >= 1) or_return
	ENSURE(t, endCount >= 1) or_return

	return true
}

@(private = "file")
TestHitEvents :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldDef.hitEventThreshold = 1.0
	worldId := b3.CreateWorld(&worldDef)

	// Static ground
	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .staticBody
	bodyDef.position = b3.Pos{0.0, -0.5, 0.0}
	groundId := b3.CreateBody(worldId, &bodyDef)
	groundBox := b3.MakeBoxHull(10.0, 0.5, 10.0)
	groundShapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(groundId, &groundShapeDef, &groundBox.base)

	// Sphere driven into the ground fast enough to clear the hit threshold
	bodyDef = b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.gravityScale = 0.0
	bodyDef.position = b3.Pos{0.0, 2.0, 0.0}
	bodyDef.linearVelocity = b3.Vec3{0.0, -30.0, 0.0}
	sphereBodyId := b3.CreateBody(worldId, &bodyDef)
	shapeDef := b3.DefaultShapeDef()
	shapeDef.density = 1.0
	shapeDef.enableHitEvents = true
	shapeDef.baseMaterial.userMaterialId = 7
	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}
	b3.CreateSphereShape(sphereBodyId, &shapeDef, &sphere)

	hitCount: i32 = 0
	capturedSpeed: f32 = 0.0
	capturedMaterialA: u64 = 0
	capturedMaterialB: u64 = 0
	capturedNormal := b3.Vec3{0.0, 0.0, 0.0}

	for i: i32 = 0; i < 30; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)

		events := b3.World_GetContactEvents(worldId)
		if events.hitCount > 0 && hitCount == 0 {
			hit := events.hitEvents[0]
			capturedSpeed = hit.approachSpeed
			capturedNormal = hit.normal
			capturedMaterialA = hit.userMaterialIdA
			capturedMaterialB = hit.userMaterialIdB
		}

		hitCount += events.hitCount
	}

	b3.DestroyWorld(worldId)

	ENSURE(t, hitCount >= 1) or_return
	ENSURE(t, capturedSpeed > 1.0) or_return
	// Head-on vertical impact: normal lies along Y
	ENSURE_SMALL(t, capturedNormal.x, 0.01) or_return
	ENSURE_SMALL(t, capturedNormal.z, 0.01) or_return
	// One side of the contact carries the sphere's user material
	ENSURE(t, capturedMaterialA == 7 || capturedMaterialB == 7) or_return

	return true
}

// Hit-event material lookup must respect the compound child that participated in the
// contact. Two children with distinct userMaterialIds at separated positions, dropped
// sphere strikes one specifically. Without the fix, both children would report
// materials[0] and the strike on hull 1 would be misattributed.
@(private = "file")
TestCompoundHitEvents :: proc(t: ^testing.T) -> bool {
	kHullMaterialA: u64 : 11
	kHullMaterialB: u64 : 22
	kSphereMaterial: u64 : 99
	kHullCenterX: f32 : 3.0

	for side: i32 = 0; side < 2; side += 1 {
		expectedHullMaterial := (side == 0) ? kHullMaterialA : kHullMaterialB
		spawnX := (side == 0) ? -kHullCenterX : kHullCenterX

		worldDef := b3.DefaultWorldDef()
		worldDef.hitEventThreshold = 1.0
		worldId := b3.CreateWorld(&worldDef)

		// Build a compound with two hulls at opposite x positions, distinct userMaterialIds
		boxA := b3.MakeBoxHull(1.0, 1.0, 1.0)
		boxB := b3.MakeBoxHull(1.0, 1.0, 1.0)

		matA := b3.DefaultSurfaceMaterial()
		matA.userMaterialId = kHullMaterialA

		matB := b3.DefaultSurfaceMaterial()
		matB.userMaterialId = kHullMaterialB

		hulls: [2]b3.CompoundHullDef
		hulls[0].hull = &boxA.base
		hulls[0].transform = b3.Transform{{-kHullCenterX, 0.0, 0.0}, b3.Quat_identity}
		hulls[0].material = matA
		hulls[1].hull = &boxB.base
		hulls[1].transform = b3.Transform{{kHullCenterX, 0.0, 0.0}, b3.Quat_identity}
		hulls[1].material = matB

		compoundDef: b3.CompoundDef = {}
		compoundDef.hulls = &hulls[0]
		compoundDef.hullCount = 2
		compound := b3.CreateCompound(&compoundDef)
		ENSURE(t, compound != nil) or_return

		// Static body holds the compound
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .staticBody
		compoundBodyId := b3.CreateBody(worldId, &bodyDef)
		compoundShapeDef := b3.DefaultShapeDef()
		b3.CreateBakedCompoundShape(compoundBodyId, &compoundShapeDef, compound)

		// Sphere driven straight down onto the chosen child
		bodyDef = b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.gravityScale = 0.0
		bodyDef.position = b3.Pos{spawnX, 3.0, 0.0}
		bodyDef.linearVelocity = b3.Vec3{0.0, -30.0, 0.0}
		sphereBodyId := b3.CreateBody(worldId, &bodyDef)
		sphereShapeDef := b3.DefaultShapeDef()
		sphereShapeDef.density = 1.0
		sphereShapeDef.enableHitEvents = true
		sphereShapeDef.baseMaterial.userMaterialId = kSphereMaterial
		sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}
		b3.CreateSphereShape(sphereBodyId, &sphereShapeDef, &sphere)

		hitCount: i32 = 0
		capturedMaterialA: u64 = 0
		capturedMaterialB: u64 = 0

		for i: i32 = 0; i < 30; i += 1 {
			b3.World_Step(worldId, 1.0 / 60.0, 4)

			events := b3.World_GetContactEvents(worldId)
			if events.hitCount > 0 && hitCount == 0 {
				hit := events.hitEvents[0]
				capturedMaterialA = hit.userMaterialIdA
				capturedMaterialB = hit.userMaterialIdB
			}
			hitCount += events.hitCount
		}

		b3.DestroyWorld(worldId)
		b3.DestroyCompound(compound)

		ENSURE(t, hitCount >= 1) or_return
		// Sphere material on one side
		ENSURE(t, capturedMaterialA == kSphereMaterial || capturedMaterialB == kSphereMaterial) or_return
		// Struck compound child's material on the other side. The pre-fix code returned
		// materials[0] (kHullMaterialA) for both sides, so a strike on the +x child would fail.
		ENSURE(t, capturedMaterialA == expectedHullMaterial || capturedMaterialB == expectedHullMaterial) or_return
	}

	return true
}

@(private = "file")
kChild0MaterialId :: 101
@(private = "file")
kChild1MaterialId :: 202
@(private = "file")
kProbeMaterialId :: 999

// No context pointer on mixing callbacks, so capture through file scope
@(private = "file")
materialCapture: struct {
	callCount:     i32,
	sawChild0:     bool,
	sawChild1:     bool,
	mixedFriction: f32,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
CaptureFrictionMix :: proc "c" (frictionA: f32, userMaterialIdA: u64, frictionB: f32, userMaterialIdB: u64) -> f32 {
	context = runtime.default_context()
	// unified end
	materialCapture.callCount += 1

	if userMaterialIdA == kChild0MaterialId || userMaterialIdB == kChild0MaterialId {
		materialCapture.sawChild0 = true
	}

	if userMaterialIdA == kChild1MaterialId || userMaterialIdB == kChild1MaterialId {
		materialCapture.sawChild1 = true
		materialCapture.mixedFriction = intrinsics.sqrt(frictionA * frictionB)
	}

	return intrinsics.sqrt(frictionA * frictionB)
}

// Contact material selection must use the struck compound child's material, not entry 0
// of the compound material table. Child 0 gets low friction, child 1 high friction, and a
// unit friction sphere strikes only child 1, so the mixing callback must see child 1's
// values. Covers sphere, capsule, and hull children. Issue #69.
@(private = "file")
TestCompoundContactMaterials :: proc(t: ^testing.T) -> bool {
	mat0 := b3.DefaultSurfaceMaterial()
	mat0.friction = 0.04
	mat0.userMaterialId = kChild0MaterialId

	mat1 := b3.DefaultSurfaceMaterial()
	mat1.friction = 0.81
	mat1.userMaterialId = kChild1MaterialId

	box := b3.MakeBoxHull(0.5, 0.5, 0.5)

	// Children at x = -3 and x = +3, all with their top face or surface at y = 0.5
	spheres := [2]b3.CompoundSphereDef {
		{sphere = {{-3.0, 0.0, 0.0}, 0.5}, material = mat0},
		{sphere = {{3.0, 0.0, 0.0}, 0.5}, material = mat1},
	}

	capsules := [2]b3.CompoundCapsuleDef {
		{capsule = {{-3.0, 0.0, -0.5}, {-3.0, 0.0, 0.5}, 0.5}, material = mat0},
		{capsule = {{3.0, 0.0, -0.5}, {3.0, 0.0, 0.5}, 0.5}, material = mat1},
	}

	hulls := [2]b3.CompoundHullDef {
		{hull = &box.base, transform = {{-3.0, 0.0, 0.0}, b3.Quat_identity}, material = mat0},
		{hull = &box.base, transform = {{3.0, 0.0, 0.0}, b3.Quat_identity}, material = mat1},
	}

	for childType: i32 = 0; childType < 3; childType += 1 {
		compoundDef: b3.CompoundDef = {}
		if childType == 0 {
			compoundDef.spheres = &spheres[0]
			compoundDef.sphereCount = 2
		} else if childType == 1 {
			compoundDef.capsules = &capsules[0]
			compoundDef.capsuleCount = 2
		} else {
			compoundDef.hulls = &hulls[0]
			compoundDef.hullCount = 2
		}

		compound := b3.CreateCompound(&compoundDef)
		ENSURE(t, compound != nil) or_return

		materialCapture = {}

		worldDef := b3.DefaultWorldDef()
		worldDef.frictionCallback = CaptureFrictionMix
		worldId := b3.CreateWorld(&worldDef)

		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .staticBody
		compoundBodyId := b3.CreateBody(worldId, &bodyDef)
		compoundShapeDef := b3.DefaultShapeDef()
		b3.CreateBakedCompoundShape(compoundBodyId, &compoundShapeDef, compound)

		// Sphere driven straight down onto child 1
		bodyDef = b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.gravityScale = 0.0
		bodyDef.position = b3.Pos{3.0, 3.0, 0.0}
		bodyDef.linearVelocity = b3.Vec3{0.0, -30.0, 0.0}
		sphereBodyId := b3.CreateBody(worldId, &bodyDef)
		sphereShapeDef := b3.DefaultShapeDef()
		sphereShapeDef.density = 1.0
		sphereShapeDef.baseMaterial.friction = 1.0
		sphereShapeDef.baseMaterial.userMaterialId = kProbeMaterialId
		sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}
		b3.CreateSphereShape(sphereBodyId, &sphereShapeDef, &sphere)

		for i: i32 = 0; i < 30; i += 1 {
			b3.World_Step(worldId, 1.0 / 60.0, 4)
		}

		b3.DestroyWorld(worldId)
		b3.DestroyCompound(compound)

		ENSURE(t, materialCapture.callCount > 0) or_return
		// The pre-fix code fed material table entry 0 to the mixing callback for every child
		ENSURE(t, materialCapture.sawChild0 == false) or_return
		ENSURE(t, materialCapture.sawChild1 == true) or_return
		// sqrt(0.81 * 1.0)
		ENSURE_SMALL(t, materialCapture.mixedFriction - 0.9, 1e-5) or_return
	}

	return true
}

@(private = "file")
TestSetWorkerCount :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldDef.workerCount = 1
	worldId := b3.CreateWorld(&worldDef)
	ENSURE(t, b3.World_IsValid(worldId)) or_return
	ENSURE(t, b3.World_GetWorkerCount(worldId) == 1) or_return

	CreateJunkyard(worldId)
	StepJunkyard(worldId, 1)

	b3.World_SetWorkerCount(worldId, 4)
	ENSURE(t, b3.World_GetWorkerCount(worldId) == 4) or_return

	StepJunkyard(worldId, 2)

	b3.World_SetWorkerCount(worldId, 4)
	ENSURE(t, b3.World_GetWorkerCount(worldId) == 4) or_return

	StepJunkyard(worldId, 3)

	b3.World_SetWorkerCount(worldId, 0)
	ENSURE(t, b3.World_GetWorkerCount(worldId) == 1) or_return

	StepJunkyard(worldId, 4)

	b3.World_SetWorkerCount(worldId, -5)
	ENSURE(t, b3.World_GetWorkerCount(worldId) == 1) or_return

	StepJunkyard(worldId, 5)

	b3.World_SetWorkerCount(worldId, b3.MAX_WORKERS + 10)
	ENSURE(t, b3.World_GetWorkerCount(worldId) == b3.MAX_WORKERS) or_return

	StepJunkyard(worldId, 2)

	b3.DestroyWorld(worldId)

	return true
}

// Verifies the b3*_Overflow solver path. The scene puts >B3_DYNAMIC_COLOR_COUNT
// dyn-dyn contacts on a single hub body so several land in the overflow color.
// The new Prepare/Store contactId-pairing asserts in contact_solver.c fire here
// if Store reads constraints from the wrong (base, spans) pair, so the test
// catches the bug even though world state ends up plausible (memory layout is
// stable within a build, so determinism alone is not a witness).
@(private = "file")
TestOverflowColorPile :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldDef.workerCount = 1
	worldId := b3.CreateWorld(&worldDef)

	data := CreateOverflowColorPile(worldId)
	_ = data

	timeStep: f32 = 1.0 / 60.0
	subStepCount: i32 = 4

	// One step would be enough to trip the asserts, but several steps also
	// exercise the warm-start path (Store -> manifold impulse -> Prepare).
	stepCount: i32 = 10
	for i: i32 = 0; i < stepCount; i += 1 {
		b3.World_Step(worldId, timeStep, subStepCount)
	}

	// Confirm the scene actually populated the overflow color. Without this,
	// a future change to graph coloring could silently turn the test into a
	// no-op.
	counters := b3.World_GetCounters(worldId)
	overflowContacts := counters.colorCounts[b3.GRAPH_COLOR_COUNT - 1]

	b3.DestroyWorld(worldId)

	ENSURE(t, overflowContacts > 0) or_return
	return true
}

// Exposes that b3Body_EnableSleep mutates body->flags but never syncs
// bodySim->flags / bodyState->flags. When a body created with
// enableSleep=false is later flipped on, body->flags gains b3_enableSleep
// but bodySim/bodyState do not. The flag-sync assertion in
// b3ValidateSolverSets then fires on the next world step.
//
// Under Debug + BOX3D_VALIDATE this crashes; after b3Body_EnableSleep
// calls b3SyncBodyFlags the test runs cleanly.
@(private = "file")
EnableSleepFlagSyncTest :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.enableSleep = false
	bodyId := b3.CreateBody(worldId, &bodyDef)

	ENSURE(t, b3.Body_IsSleepEnabled(bodyId) == false) or_return

	b3.Body_EnableSleep(bodyId, true)
	ENSURE(t, b3.Body_IsSleepEnabled(bodyId) == true) or_return

	b3.World_Step(worldId, 1.0 / 60.0, 4)

	b3.DestroyWorld(worldId)
	return true
}

// Exposes that b3Body_SetBullet writes only bodySim->flags without
// touching body->flags, while b3SyncBodyFlags overwrites bodySim from
// body->flags. Any subsequent b3Body_SetMotionLocks or b3Body_SetType
// silently wipes (or re-asserts) the bullet bit.
//
// Reproducer A: create with isBullet=false, SetBullet(true), then
// SetMotionLocks. b3Body_IsBullet returns false today (wiped); should be
// true after fix.
//
// Reproducer B: create with isBullet=true, SetBullet(false), then
// SetMotionLocks. b3Body_IsBullet returns true today (re-asserted); should
// be false after fix.
@(private = "file")
SetBulletDriftTest :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	{
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.isBullet = false
		bodyId := b3.CreateBody(worldId, &bodyDef)

		ENSURE(t, b3.Body_IsBullet(bodyId) == false) or_return

		b3.Body_SetBullet(bodyId, true)
		ENSURE(t, b3.Body_IsBullet(bodyId) == true) or_return

		locks: b3.MotionLocks = {}
		locks.linearX = true
		b3.Body_SetMotionLocks(bodyId, locks)

		ENSURE(t, b3.Body_IsBullet(bodyId) == true) or_return
	}

	{
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.isBullet = true
		bodyId := b3.CreateBody(worldId, &bodyDef)

		ENSURE(t, b3.Body_IsBullet(bodyId) == true) or_return

		b3.Body_SetBullet(bodyId, false)
		ENSURE(t, b3.Body_IsBullet(bodyId) == false) or_return

		locks: b3.MotionLocks = {}
		locks.linearX = true
		b3.Body_SetMotionLocks(bodyId, locks)

		ENSURE(t, b3.Body_IsBullet(bodyId) == false) or_return
	}

	b3.DestroyWorld(worldId)
	return true
}

// Regression: b3Body_EnableSleep used to set world->locked = true before checking
// for a no-op change, then early-return without unlocking. The next mutator call
// would then trip the assert in b3GetUnlockedWorld.
@(private = "file")
EnableSleepNoopUnlockTest :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.enableSleep = true
	bodyId := b3.CreateBody(worldId, &bodyDef)

	// No-op: enableSleep is already true. Must not leak the world lock.
	b3.Body_EnableSleep(bodyId, true)

	// Would assert in b3GetUnlockedWorld if the lock had leaked.
	b3.Body_EnableSleep(bodyId, false)
	ENSURE(t, b3.Body_IsSleepEnabled(bodyId) == false) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
EnableContactRecyclingTest :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody

	// Default is enabled
	bodyA := b3.CreateBody(worldId, &bodyDef)
	ENSURE(t, b3.Body_IsContactRecyclingEnabled(bodyA) == true) or_return

	b3.Body_EnableContactRecycling(bodyA, false)
	ENSURE(t, b3.Body_IsContactRecyclingEnabled(bodyA) == false) or_return

	b3.Body_EnableContactRecycling(bodyA, true)
	ENSURE(t, b3.Body_IsContactRecyclingEnabled(bodyA) == true) or_return

	// Per-def opt-out at creation
	bodyDef.enableContactRecycling = false
	bodyB := b3.CreateBody(worldId, &bodyDef)
	ENSURE(t, b3.Body_IsContactRecyclingEnabled(bodyB) == false) or_return

	// Stepping after toggling must not trip the flag-sync validator
	b3.World_Step(worldId, 1.0 / 60.0, 4)

	b3.DestroyWorld(worldId)
	return true
}

// A slow sphere closes on a static round shape from just outside contact. The step is too short
// to make the sphere fast, so the pair is eligible for recycling. A routine without a speculative
// margin left the pair empty and recycling then let the sphere sink without a contact. Discussion #175.
@(private = "file")
RunRecycledApproach :: proc(t: ^testing.T, capsuleTarget: bool) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldDef.gravity = b3.Vec3_zero
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .staticBody
	targetId := b3.CreateBody(worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()
	targetRadius: f32 = 0.5
	if capsuleTarget {
		capsule := b3.Capsule{{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, targetRadius}
		b3.CreateCapsuleShape(targetId, &shapeDef, &capsule)
	} else {
		sphere := b3.Sphere{b3.Vec3_zero, targetRadius}
		b3.CreateSphereShape(targetId, &shapeDef, &sphere)
	}

	radius: f32 = 0.25
	totalRadius := targetRadius + radius
	timeStep: f32 = 1.0 / 60.0

	// Half the speculative distance per step
	speed := 0.5 * b3.SPECULATIVE_DISTANCE / timeStep

	bodyDef = b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = b3.Pos{0.0, totalRadius + 0.25 * b3.SPECULATIVE_DISTANCE, 0.0}
	bodyDef.linearVelocity = b3.Vec3{0.0, -speed, 0.0}
	bodyId := b3.CreateBody(worldId, &bodyDef)

	sphere := b3.Sphere{b3.Vec3_zero, radius}
	b3.CreateSphereShape(bodyId, &shapeDef, &sphere)

	minSeparation := b3.FLT_MAX
	for i: i32 = 0; i < 20; i += 1 {
		b3.World_Step(worldId, timeStep, 4)

		p := b3.Body_GetPosition(bodyId)
		minSeparation = b3.MinFloat(minSeparation, f32(p.y) - totalRadius)
	}

	b3.DestroyWorld(worldId)

	ENSURE(t, minSeparation > -b3.LINEAR_SLOP) or_return

	return true
}

@(private = "file")
RecycledSphereApproachTest :: proc(t: ^testing.T) -> bool {
	ENSURE(t, RunRecycledApproach(t, false)) or_return
	ENSURE(t, RunRecycledApproach(t, true)) or_return
	return true
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
PreSolveReject :: proc "c" (
	shapeIdA: b3.ShapeId,
	shapeIdB: b3.ShapeId,
	point: b3.Pos,
	normal: b3.Vec3,
	context_: rawptr,
) -> bool {
	context = runtime.default_context()
	// unified end
	_ = shapeIdA
	_ = shapeIdB
	_ = point
	_ = normal
	reject := (^bool)(context_)
	return reject^ == false
}

// A rejected pre-solve frees the manifold while the shapes may already touch. The next step must
// rebuild the contact and ask again instead of recycling the empty pair.
@(private = "file")
PreSolveRejectRecycleTest :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldDef.enableSleep = false
	worldId := b3.CreateWorld(&worldDef)

	reject := true
	b3.World_SetPreSolveCallback(worldId, PreSolveReject, &reject)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .staticBody
	bodyDef.position = b3.Pos{0.0, -0.5, 0.0}
	groundId := b3.CreateBody(worldId, &bodyDef)
	groundBox := b3.MakeBoxHull(10.0, 0.5, 10.0)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(groundId, &shapeDef, &groundBox.base)

	radius: f32 = 0.25
	bodyDef = b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = b3.Pos{0.0, radius, 0.0}
	bodyId := b3.CreateBody(worldId, &bodyDef)

	shapeDef.enablePreSolveEvents = true
	sphere := b3.Sphere{b3.Vec3_zero, radius}
	b3.CreateSphereShape(bodyId, &shapeDef, &sphere)

	b3.World_Step(worldId, 1.0 / 60.0, 4)

	reject = false

	minSeparation := b3.FLT_MAX
	for i: i32 = 0; i < 30; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)

		p := b3.Body_GetPosition(bodyId)
		minSeparation = b3.MinFloat(minSeparation, f32(p.y) - radius)
	}

	b3.DestroyWorld(worldId)

	ENSURE(t, minSeparation > -b3.LINEAR_SLOP) or_return

	return true
}

@(private = "file")
CountRecycledResting :: proc(enableSpeculative: bool) -> i32 {
	worldDef := b3.DefaultWorldDef()
	worldDef.enableSleep = false
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .staticBody
	bodyDef.position = b3.Pos{0.0, -0.5, 0.0}
	groundId := b3.CreateBody(worldId, &bodyDef)
	groundBox := b3.MakeBoxHull(10.0, 0.5, 10.0)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(groundId, &shapeDef, &groundBox.base)

	bodyDef = b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = b3.Pos{0.0, 0.5, 0.0}
	bodyId := b3.CreateBody(worldId, &bodyDef)

	shapeDef.enableSpeculativeContact = enableSpeculative
	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	recycledCount: i32 = 0
	for i: i32 = 0; i < 60; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)
		recycledCount += b3.World_GetCounters(worldId).recycledContactCount
	}

	b3.DestroyWorld(worldId)

	return recycledCount
}

// Without speculative points an empty pair may be closer than the speculative distance, so these
// contacts must never recycle
@(private = "file")
SpeculativeOffRecycleTest :: proc(t: ^testing.T) -> bool {
	ENSURE(t, CountRecycledResting(true) > 0) or_return
	ENSURE(t, CountRecycledResting(false) == 0) or_return
	return true
}

@(private = "file")
CountMovedNodes :: proc(tree: ^b3.DynamicTree) -> i32 {
	count: i32 = 0
	for i: i32 = 0; i < tree.nodeEnd; i += 1 {
		node := &tree.nodes[i]
		if b3.IsEmptyNode(node) == false && b3.IsNodeMoved(node) {
			count += 1
		}
	}

	return count
}

// A proxy can be enlarged in one step and destroyed before the next, emptying the move buffer.
// The trees must still be rebuilt or the enlarged nodes survive into the next step. Issue #149.
@(private = "file")
TestEnlargedProxyDestroyed :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldDef.gravity = b3.Vec3_zero
	worldId := b3.CreateWorld(&worldDef)

	world := b3.GetWorldFromId(worldId)
	tree := &world.broadPhase.trees[b3.BodyType.dynamicBody]

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody

	shapeDef := b3.DefaultShapeDef()
	sphere := b3.Sphere{b3.Vec3_zero, 0.25}

	// Resting bodies keep the tree deep enough that the mover has an ancestor that outlives it
	for i: i32 = 0; i < 8; i += 1 {
		bodyDef.position = b3.Pos{2.0 * f32(i), 0.0, 0.0}
		restingId := b3.CreateBody(worldId, &bodyDef)
		b3.CreateSphereShape(restingId, &shapeDef, &sphere)
	}

	// This body outruns its AABB margin every step
	bodyDef.position = b3.Pos{7.0, 1.0, 0.0}
	bodyDef.linearVelocity = b3.Vec3{6.0, 0.0, 0.0}
	moverId := b3.CreateBody(worldId, &bodyDef)
	b3.CreateSphereShape(moverId, &shapeDef, &sphere)

	timeStep: f32 = 1.0 / 60.0
	b3.World_Step(worldId, timeStep, 4)

	ENSURE(t, CountMovedNodes(tree) > 0) or_return

	b3.DestroyBody(moverId)

	// Removing the leaf re-derives the marks along its path, so the tree is already consistent
	// here and nothing is left marked to drive a rebuild. What the destroy can still leave
	// stale is the DFS order the sweep refit needs, so the step has to rebuild regardless.
	b3.DynamicTree_Validate(tree)
	ENSURE(t, CountMovedNodes(tree) == 0) or_return

	b3.World_Step(worldId, timeStep, 4)

	ENSURE(t, CountMovedNodes(tree) == 0) or_return
	ENSURE(t, tree.dfsOrdered) or_return

	b3.DestroyWorld(worldId)
	return true
}

// b3DestroyBody tears shapes down inline instead of going through the shape destroy path, so it
// needs its own compound census decrement. A leaked count pins the broad-phase on the slow
// per-candidate compound path for the life of the world and rides along into snapshots.
@(private = "file")
TestCompoundShapeCount :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	world := b3.GetWorldFromId(worldId)
	ENSURE(t, world.compoundShapeCount == 0) or_return

	box := b3.MakeBoxHull(1.0, 1.0, 1.0)

	hull: b3.CompoundHullDef
	hull.hull = &box.base
	hull.transform = b3.Transform{b3.Vec3_zero, b3.Quat_identity}
	hull.material = b3.DefaultSurfaceMaterial()

	compoundDef: b3.CompoundDef = {}
	compoundDef.hulls = &hull
	compoundDef.hullCount = 1
	compound := b3.CreateCompound(&compoundDef)
	ENSURE(t, compound != nil) or_return

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .staticBody
	shapeDef := b3.DefaultShapeDef()

	// Shape destroyed on its own
	bodyId := b3.CreateBody(worldId, &bodyDef)
	shapeId := b3.CreateBakedCompoundShape(bodyId, &shapeDef, compound)
	ENSURE(t, world.compoundShapeCount == 1) or_return
	b3.DestroyShape(shapeId, true)
	ENSURE(t, world.compoundShapeCount == 0) or_return
	b3.DestroyBody(bodyId)

	// Shape carried away by its body
	bodyId = b3.CreateBody(worldId, &bodyDef)
	b3.CreateBakedCompoundShape(bodyId, &shapeDef, compound)
	ENSURE(t, world.compoundShapeCount == 1) or_return
	b3.DestroyBody(bodyId)
	ENSURE(t, world.compoundShapeCount == 0) or_return

	b3.DestroyWorld(worldId)
	b3.DestroyCompound(compound)
	return true
}

// Identical hull data is shared through a reference counted world database.
@(private = "file")
TestHullDatabase :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	box := b3.MakeBoxHull(0.5, 0.5, 0.5)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyA := b3.CreateBody(worldId, &bodyDef)
	bodyB := b3.CreateBody(worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()

	// Two shapes built from identical data share one owned copy in the world database.
	shapeA := b3.CreateHullShape(bodyA, &shapeDef, &box.base)
	shapeB := b3.CreateHullShape(bodyB, &shapeDef, &box.base)

	gotA := b3.Shape_GetHull(shapeA)
	gotB := b3.Shape_GetHull(shapeB)

	// Both shapes point at the single shared copy
	ENSURE(t, gotA == gotB) or_return

	// The shared copy is owned by the world, not the caller's stack hull
	ENSURE(t, gotA != &box.base) or_return

	// A box built on an independent stack frame must de-duplicate to the same shared copy.
	// This holds only if content hashing sees deterministic padding bytes.
	box2 := b3.MakeBoxHull(0.5, 0.5, 0.5)
	bodyC := b3.CreateBody(worldId, &bodyDef)
	shapeC := b3.CreateHullShape(bodyC, &shapeDef, &box2.base)
	ENSURE(t, b3.Shape_GetHull(shapeC) == gotA) or_return
	b3.DestroyShape(shapeC, true)

	// Setting a shape's hull to its own sole shared copy must not free it mid update.
	box3 := b3.MakeBoxHull(0.3, 0.3, 0.3)
	bodyD := b3.CreateBody(worldId, &bodyDef)
	shapeD := b3.CreateHullShape(bodyD, &shapeDef, &box3.base)
	gotD := b3.Shape_GetHull(shapeD)
	b3.Shape_SetHull(shapeD, gotD)
	ENSURE(t, b3.Shape_GetHull(shapeD) == gotD) or_return
	b3.DestroyShape(shapeD, true)

	// Releasing one reference keeps the other alive
	b3.DestroyShape(shapeA, true)
	stillB := b3.Shape_GetHull(shapeB)
	ENSURE(t, stillB == gotB) or_return

	b3.DestroyShape(shapeB, true)

	// World destroy asserts the database drained to zero references
	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
ExplosionResult :: struct {
	linearVelocity:  b3.Vec3,
	angularVelocity: b3.Vec3,
}

// Explode just off the +x side of a centered sphere and capture the impulse it receives.
// The shape, the blast point, and the witness math all run in the body local frame, so the
// result must not depend on how far the body sits from the world origin.
@(private = "file")
RunExplosion :: proc(base: b3.Pos) -> ExplosionResult {
	worldDef := b3.DefaultWorldDef()
	worldDef.gravity = b3.Vec3_zero
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = base
	bodyId := b3.CreateBody(worldId, &bodyDef)

	sphere := b3.Sphere{b3.Vec3_zero, 1.0}
	shapeDef := b3.DefaultShapeDef()
	b3.CreateSphereShape(bodyId, &shapeDef, &sphere)

	// Blast sits 3 units along +x, so the body is pushed back along -x
	explosionDef := b3.DefaultExplosionDef()
	explosionDef.position = b3.OffsetPos(base, b3.Vec3{3.0, 0.0, 0.0})
	explosionDef.radius = 5.0
	explosionDef.falloff = 0.0
	explosionDef.impulsePerArea = 10.0
	b3.World_Explode(worldId, &explosionDef)

	result: ExplosionResult
	result.linearVelocity = b3.Body_GetLinearVelocity(bodyId)
	result.angularVelocity = b3.Body_GetAngularVelocity(bodyId)

	b3.DestroyWorld(worldId)
	return result
}

@(private = "file")
TestExplosion :: proc(t: ^testing.T) -> bool {
	origin := RunExplosion(b3.Pos_zero)

	// Pushed away from the blast along -x. A centered sphere has no transverse or angular component.
	ENSURE(t, origin.linearVelocity.x < -1.0e-4) or_return
	ENSURE_SMALL(t, origin.linearVelocity.y, 1.0e-6) or_return
	ENSURE_SMALL(t, origin.linearVelocity.z, 1.0e-6) or_return
	ENSURE_SMALL(t, b3.Length(origin.angularVelocity), 1.0e-6) or_return

	// The same blast far from the origin must produce the same impulse. The world position only
	// reaches float in the relative difference, so the result holds where a naive cast would not.
	far := RunExplosion(b3.Pos{1.0e7, 1.0e7, 1.0e7})
	ENSURE_SMALL(t, far.linearVelocity.x - origin.linearVelocity.x, 1.0e-5) or_return
	ENSURE_SMALL(t, far.linearVelocity.y - origin.linearVelocity.y, 1.0e-5) or_return
	ENSURE_SMALL(t, far.linearVelocity.z - origin.linearVelocity.z, 1.0e-5) or_return

	return true
}

// Ensure correct move events from bodies involved in CCD.
@(private = "file")
TestContinuousMoveEvent :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)
	b3.World_EnableContinuous(worldId, true)

	// Thin static wall, near face at x = 0.1
	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .staticBody
	bodyDef.position = b3.Pos{0.0, 0.0, 0.0}
	wallId := b3.CreateBody(worldId, &bodyDef)
	wallBox := b3.MakeBoxHull(0.1, 5.0, 5.0)
	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(wallId, &shapeDef, &wallBox.base)

	// Fast dynamic sphere fired at the wall.
	bodyDef = b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.gravityScale = 0.0
	bodyDef.position = b3.Pos{3.0, 0.0, 0.0}
	bodyDef.linearVelocity = b3.Vec3{-30.0, 0.0, 0.0}
	ballId := b3.CreateBody(worldId, &bodyDef)
	shapeDef = b3.DefaultShapeDef()
	shapeDef.density = 1.0
	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.25}
	b3.CreateSphereShape(ballId, &shapeDef, &sphere)

	timeStep: f32 = 1.0 / 60.0
	subStepCount: i32 = 4
	haveMove := false

	for step: i32 = 0; step < 30; step += 1 {
		b3.World_Step(worldId, timeStep, subStepCount)

		xf := b3.Body_GetTransform(ballId)

		events := b3.World_GetBodyEvents(worldId)
		for i: i32 = 0; i < events.moveCount; i += 1 {
			event := &events.moveEvents[i]
			if b3.ID_EQUALS(event.bodyId, ballId) == false {
				continue
			}

			haveMove = true

			// The move event must carry the same pose the body reports, CCD rewind included
			ENSURE(t, event.transform.p.x == xf.p.x) or_return
			ENSURE(t, event.transform.p.y == xf.p.y) or_return
			ENSURE(t, event.transform.p.z == xf.p.z) or_return
			ENSURE(t, event.transform.q.v.x == xf.q.v.x) or_return
			ENSURE(t, event.transform.q.v.y == xf.q.v.y) or_return
			ENSURE(t, event.transform.q.v.z == xf.q.v.z) or_return
			ENSURE(t, event.transform.q.s == xf.q.s) or_return
		}
	}

	ENSURE(t, haveMove == true) or_return

	// Tunnel check
	finalPos := b3.Body_GetPosition(ballId)
	ENSURE(t, 0.2 < finalPos.x && finalPos.x < 0.8) or_return

	b3.DestroyWorld(worldId)

	return true
}

// The contact body sim locators are persistent state. Every contact in every set carries the
// current encoding for both of its bodies: the awake local index, a static tag, or null for a
// body that fell asleep. The scenes below drive each site that can invalidate one. The gate is
// b3ValidateSolverSets, which asserts the invariant on every mutation and every step, so a
// dropped refresh trips there rather than producing a plausible but wrong simulation. Each scene
// also runs at worker counts 1 and 4 with the state hashes compared.

@(private = "file")
CreateLocatorWorld :: proc(workerCount: i32) -> b3.WorldId {
	worldDef := b3.DefaultWorldDef()
	worldDef.workerCount = u32(workerCount)
	return b3.CreateWorld(&worldDef)
}

@(private = "file")
CreateLocatorBody :: proc(
	worldId: b3.WorldId,
	type: b3.BodyType,
	position: b3.Pos,
	hx: f32,
	hy: f32,
	hz: f32,
) -> b3.BodyId {
	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = type
	bodyDef.position = position
	bodyId := b3.CreateBody(worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()
	box := b3.MakeBoxHull(hx, hy, hz)
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	return bodyId
}

@(private = "file")
StepLocatorWorld :: proc(worldId: b3.WorldId, stepCount: i32) {
	for i: i32 = 0; i < stepCount; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)
	}
}

// Steps until the body sleeps or the budget runs out. Returns the steps taken.
@(private = "file")
StepUntilAsleep :: proc(worldId: b3.WorldId, bodyId: b3.BodyId, maxSteps: i32) -> i32 {
	for i: i32 = 0; i < maxSteps; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)
		if b3.Body_IsAwake(bodyId) == false {
			return i + 1
		}
	}

	return maxSteps
}

// Sleep and wake: first touch puts contacts in the graph, sleeping nulls the locators of the
// island bodies, waking restores their awake indices. The plate beside the stack holds a
// non-touching contact across the whole cycle, and that is the contact the wake path has to fix up
// itself. A touching contact would get a fresh encoding from the graph add and hide a dropped
// refresh.
@(private = "file")
LocatorSleepWakeScene :: proc(workerCount: i32, failed: ^i32) -> u64 {
	worldId := CreateLocatorWorld(workerCount)
	CreateLocatorBody(worldId, .staticBody, b3.Pos{0.0, -1.0, 0.0}, 5.0, 1.0, 5.0)

	stackId: [3]b3.BodyId
	for i: i32 = 0; i < 3; i += 1 {
		stackId[i] = CreateLocatorBody(worldId, .dynamicBody, b3.Pos{0.0, 0.5 + f32(i), 0.0}, 0.5, 0.5, 0.5)
	}

	// Beside the top box, close enough that the fat bounds overlap and far enough that the
	// shapes never touch. Kinematic and awake, so the contact stays in the awake set while the
	// stack sleeps. Off to the side so it does not catch the body dropped later.
	plateId := CreateLocatorBody(worldId, .kinematicBody, b3.Pos{1.04, 2.5, 0.0}, 0.5, 0.5, 0.5)
	b3.Body_EnableSleep(plateId, false)

	steps := StepUntilAsleep(worldId, stackId[0], 300)
	if steps == 300 || b3.Body_IsAwake(stackId[2]) == true {
		failed^ = 1
	}

	// Wake without contact so the non-touching contact keeps its stale locator if the wake
	// path forgets to refresh
	b3.Body_SetAwake(stackId[0], true)
	StepLocatorWorld(worldId, 5)

	if b3.Body_IsAwake(stackId[2]) == false {
		failed^ = 1
	}

	steps = StepUntilAsleep(worldId, stackId[0], 300)
	if steps == 300 {
		failed^ = 1
	}

	// Dropping a body on the pile drives the wake path through first touch. The pile can settle
	// and sleep again, so catch the wake as it happens.
	CreateLocatorBody(worldId, .dynamicBody, b3.Pos{0.0, 8.0, 0.0}, 0.5, 0.5, 0.5)

	woke := false
	for i: i32 = 0; i < 180; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)
		woke = woke || b3.Body_IsAwake(stackId[0])
	}

	if woke == false {
		failed^ = 1
	}

	hash := b3.HashWorldState(b3.GetWorldFromId(worldId))
	b3.DestroyWorld(worldId)
	return hash
}

// An island falling asleep and a body leaving the awake set both swap the last awake sim into
// the hole. The pile that never sleeps supplies a moved body that still owns contacts.
@(private = "file")
LocatorAwakeNeighborScene :: proc(workerCount: i32, failed: ^i32) -> u64 {
	worldId := CreateLocatorWorld(workerCount)
	CreateLocatorBody(worldId, .staticBody, b3.Pos{0.0, -1.0, 0.0}, 30.0, 1.0, 5.0)

	// This island sleeps
	sleeperId: [2]b3.BodyId
	for i: i32 = 0; i < 2; i += 1 {
		sleeperId[i] = CreateLocatorBody(worldId, .dynamicBody, b3.Pos{-10.0, 0.5 + f32(i), 0.0}, 0.5, 0.5, 0.5)
	}

	// Created after the sleepers so its sims sit at the end of the awake set and get swapped
	// into the holes the sleeping island leaves behind
	spareId := CreateLocatorBody(worldId, .dynamicBody, b3.Pos{20.0, 0.5, 0.0}, 0.5, 0.5, 0.5)
	b3.Body_EnableSleep(spareId, false)

	awakeId: [3]b3.BodyId
	for i: i32 = 0; i < 3; i += 1 {
		awakeId[i] = CreateLocatorBody(worldId, .dynamicBody, b3.Pos{10.0, 0.5 + f32(i), 0.0}, 0.5, 0.5, 0.5)
		b3.Body_EnableSleep(awakeId[i], false)
	}

	steps := StepUntilAsleep(worldId, sleeperId[0], 300)
	if steps == 300 || b3.Body_IsAwake(awakeId[0]) == false {
		failed^ = 1
	}

	StepLocatorWorld(worldId, 10)

	// Leaving the awake set swaps a body that owns contacts into the spare's slot
	b3.Body_Disable(spareId)
	StepLocatorWorld(worldId, 10)

	b3.Body_Enable(spareId)
	StepLocatorWorld(worldId, 10)

	hash := b3.HashWorldState(b3.GetWorldFromId(worldId))
	b3.DestroyWorld(worldId)
	return hash
}

// Destroying a body swaps the last sim of its set into the hole. The moved body keeps its
// contacts, so its side of each one has to follow it.
@(private = "file")
LocatorDestroyScene :: proc(workerCount: i32, failed: ^i32) -> u64 {
	worldId := CreateLocatorWorld(workerCount)
	CreateLocatorBody(worldId, .staticBody, b3.Pos{0.0, -1.0, 0.0}, 5.0, 1.0, 5.0)

	stackId: [4]b3.BodyId
	for i: i32 = 0; i < 4; i += 1 {
		stackId[i] = CreateLocatorBody(worldId, .dynamicBody, b3.Pos{0.0, 0.5 + f32(i), 0.0}, 0.5, 0.5, 0.5)
	}

	StepLocatorWorld(worldId, 30)

	// The last awake sim moves into this hole while its contacts live
	b3.DestroyBody(stackId[0])
	StepLocatorWorld(worldId, 30)

	// Three static boxes, one resting box each. Destroying the first moves the last static sim.
	staticId: [3]b3.BodyId
	for i: i32 = 0; i < 3; i += 1 {
		x := 10.0 + 4.0 * f32(i)
		staticId[i] = CreateLocatorBody(worldId, .staticBody, b3.Pos{x, -1.0, 0.0}, 1.0, 1.0, 1.0)
		CreateLocatorBody(worldId, .dynamicBody, b3.Pos{x, 0.5, 0.0}, 0.5, 0.5, 0.5)
	}

	StepLocatorWorld(worldId, 30)

	b3.DestroyBody(staticId[0])
	StepLocatorWorld(worldId, 30)

	_ = failed

	hash := b3.HashWorldState(b3.GetWorldFromId(worldId))
	b3.DestroyWorld(worldId)
	return hash
}

// A kinematic body parked a hair above a pile keeps a non-touching contact to a body that falls
// asleep, which is the one case the locator cannot name and has to leave to the cold path.
@(private = "file")
LocatorKinematicScene :: proc(workerCount: i32, failed: ^i32) -> u64 {
	worldId := CreateLocatorWorld(workerCount)
	CreateLocatorBody(worldId, .staticBody, b3.Pos{0.0, -1.0, 0.0}, 5.0, 1.0, 5.0)

	boxId := CreateLocatorBody(worldId, .dynamicBody, b3.Pos{0.0, 0.5, 0.0}, 0.5, 0.5, 0.5)

	// Close enough that the fat bounds overlap and a non-touching contact exists
	plateId := CreateLocatorBody(worldId, .kinematicBody, b3.Pos{0.0, 1.52, 0.0}, 1.0, 0.5, 1.0)

	steps := StepUntilAsleep(worldId, boxId, 300)
	if steps == 300 {
		failed^ = 1
	}

	// Drive the plate down to touch and wake the sleeper
	b3.Body_SetLinearVelocity(plateId, b3.Vec3{0.0, -1.0, 0.0})
	StepLocatorWorld(worldId, 60)

	if b3.Body_IsAwake(boxId) == false {
		failed^ = 1
	}

	hash := b3.HashWorldState(b3.GetWorldFromId(worldId))
	b3.DestroyWorld(worldId)
	return hash
}

// Set type and disable move a body between solver sets, which changes the encoding of every
// contact it has. A joint across two sleeping sets drives the merge path.
@(private = "file")
LocatorTransferScene :: proc(workerCount: i32, failed: ^i32) -> u64 {
	worldId := CreateLocatorWorld(workerCount)
	CreateLocatorBody(worldId, .staticBody, b3.Pos{0.0, -1.0, 0.0}, 20.0, 1.0, 5.0)

	boxId := CreateLocatorBody(worldId, .dynamicBody, b3.Pos{0.0, 0.5, 0.0}, 0.5, 0.5, 0.5)
	StepLocatorWorld(worldId, 30)

	b3.Body_SetType(boxId, .staticBody)
	StepLocatorWorld(worldId, 5)
	b3.Body_SetType(boxId, .dynamicBody)
	StepLocatorWorld(worldId, 5)

	b3.Body_Disable(boxId)
	StepLocatorWorld(worldId, 5)
	b3.Body_Enable(boxId)
	StepLocatorWorld(worldId, 30)

	// Two piles far enough apart to sleep in separate sets
	sleeperId: [2]b3.BodyId
	sleeperId[0] = CreateLocatorBody(worldId, .dynamicBody, b3.Pos{-12.0, 0.5, 0.0}, 0.5, 0.5, 0.5)
	sleeperId[1] = CreateLocatorBody(worldId, .dynamicBody, b3.Pos{12.0, 0.5, 0.0}, 0.5, 0.5, 0.5)

	steps := StepUntilAsleep(worldId, sleeperId[0], 300)
	if steps == 300 || b3.Body_IsAwake(sleeperId[1]) == true {
		failed^ = 1
	}

	jointDef := b3.DefaultSphericalJointDef()
	jointDef.base.bodyIdA = sleeperId[0]
	jointDef.base.bodyIdB = sleeperId[1]
	b3.CreateSphericalJoint(worldId, &jointDef)

	StepLocatorWorld(worldId, 30)

	hash := b3.HashWorldState(b3.GetWorldFromId(worldId))
	b3.DestroyWorld(worldId)
	return hash
}

// The fast flag lives on the body sim and gates contact recycling, but a flag sync rebuilds the
// sim flags from the body row, which never carries it. A public setter called on a body in
// flight must not drop it, or the recycle path can take a fast pair.
@(private = "file")
LocatorFastFlagTest :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	CreateLocatorBody(worldId, .staticBody, b3.Pos{0.0, -1.0, 0.0}, 5.0, 1.0, 5.0)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = b3.Pos{-20.0, 2.0, 0.0}
	bodyDef.linearVelocity = b3.Vec3{400.0, 0.0, 0.0}
	bulletId := b3.CreateBody(worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()
	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.1}
	b3.CreateSphereShape(bulletId, &shapeDef, &sphere)

	b3.World_Step(worldId, 1.0 / 60.0, 4)

	world := b3.GetWorldFromId(worldId)
	body := b3.GetBodyFullId(world, bulletId)
	sim := b3.GetBodySim(world, body)
	ENSURE(t, (sim.flags & b3.isFast) != 0) or_return

	// Any public setter that syncs flags. The locks have to actually change or the setter
	// returns early and never syncs.
	locks: b3.MotionLocks = {}
	locks.angularX = true
	b3.Body_SetMotionLocks(bulletId, locks)

	sim = b3.GetBodySim(world, b3.GetBodyFullId(world, bulletId))
	ENSURE(t, (sim.flags & b3.isFast) != 0) or_return

	b3.World_Step(worldId, 1.0 / 60.0, 4)

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
BodySimLocatorTest :: proc(t: ^testing.T) -> bool {
	LocatorScene :: #type proc(workerCount: i32, failed: ^i32) -> u64
	scenes := [5]LocatorScene {
		LocatorSleepWakeScene,
		LocatorAwakeNeighborScene,
		LocatorDestroyScene,
		LocatorKinematicScene,
		LocatorTransferScene,
	}

	for i: i32 = 0; i < 5; i += 1 {
		failed: i32 = 0
		hash1 := scenes[i](1, &failed)
		hash4 := scenes[i](4, &failed)
		ENSURE(t, failed == 0) or_return
		ENSURE(t, hash1 == hash4) or_return
	}

	return LocatorFastFlagTest(t)
}

@(test)
WorldTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, HelloWorld) {return}
	if !RUN_SUBTEST(t, EmptyWorld) {return}
	if !RUN_SUBTEST(t, DestroyAllBodiesWorld) {return}
	if !RUN_SUBTEST(t, TestIsValid) {return}
	if !RUN_SUBTEST(t, TestWorldRecycle) {return}
	if !RUN_SUBTEST(t, TestWorldCoverage) {return}
	if !RUN_SUBTEST(t, TestExplosion) {return}
	if !RUN_SUBTEST(t, TestSensor) {return}
	if !RUN_SUBTEST(t, TestContinuousMoveEvent) {return}
	if !RUN_SUBTEST(t, TestContactEvents) {return}
	if !RUN_SUBTEST(t, TestHitEvents) {return}
	if !RUN_SUBTEST(t, TestCompoundHitEvents) {return}
	if !RUN_SUBTEST(t, TestCompoundContactMaterials) {return}
	if !RUN_SUBTEST(t, TestOverflowColorPile) {return}
	if !RUN_SUBTEST(t, SetBulletDriftTest) {return}
	if !RUN_SUBTEST(t, EnableSleepFlagSyncTest) {return}
	if !RUN_SUBTEST(t, EnableSleepNoopUnlockTest) {return}
	if !RUN_SUBTEST(t, EnableContactRecyclingTest) {return}
	if !RUN_SUBTEST(t, RecycledSphereApproachTest) {return}
	if !RUN_SUBTEST(t, PreSolveRejectRecycleTest) {return}
	if !RUN_SUBTEST(t, SpeculativeOffRecycleTest) {return}
	if !RUN_SUBTEST(t, TestSetWorkerCount) {return}
	if !RUN_SUBTEST(t, TestHullDatabase) {return}
	if !RUN_SUBTEST(t, TestEnlargedProxyDestroyed) {return}
	if !RUN_SUBTEST(t, TestCompoundShapeCount) {return}
	if !RUN_SUBTEST(t, BodySimLocatorTest) {return}
}
