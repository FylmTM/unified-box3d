// Ported from box3d test/test_determinism.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import "core:testing"

import b3 "../../box3d"

// unified: not ported: the TracyCFrameMark calls (no profiler in the port).

// Double precision accumulates body positions in double, so the settle/sleep step and the
// state hash differ from the float build. Both modes are internally deterministic.
// unified: single precision only: the BOX3D_DOUBLE_PRECISION goldens are not ported.
@(private = "file")
RAGDOLL_SLEEP_STEP :: 291
@(private = "file")
RAGDOLL_HASH :: 0xFF4C605B
@(private = "file")
WAVE_PILE_SLEEP_STEP :: 272
@(private = "file")
WAVE_PILE_HASH :: 0x266F83B0
@(private = "file")
QUERY_SPAWN_SLEEP_STEP :: 242
@(private = "file")
QUERY_SPAWN_HASH :: 0xF3AD9BA6
@(private = "file")
QUERY_SPAWN_HIT_COUNT :: 59
@(private = "file")
QUERY_SPAWN_QUERY_HASH :: 0xE3271F3D
@(private = "file")
MESH_DROP_SLEEP_STEP :: 226
@(private = "file")
MESH_DROP_HASH :: 0xD2FF069D

// The goldens above pin exact values for the default four point manifold. A build that
// overrides B3_MAX_MANIFOLD_POINTS produces a different contact set, so those checks drop
// out and only the run to run agreement below applies.
@(private = "file")
ENSURE_GOLDEN :: proc(
	t: ^testing.T,
	condition: bool,
	expr := #caller_expression(condition),
	loc := #caller_location,
) -> bool {
	when b3.MAX_MANIFOLD_POINTS == 4 {
		return ENSURE(t, condition, expr, loc)
	} else {
		return true
	}
}

@(private = "file")
DeterminismResult :: struct {
	sleepStep:     i32,
	hash:          u32,
	queryHitCount: i32,
	queryHash:     u32,
	seeded:        bool,
}

// Every worker count has to reproduce the first run exactly. Scenarios without queries leave
// the query fields zero in both the reference and the result.
@(private = "file")
EnsureRepeatable :: proc(t: ^testing.T, reference: ^DeterminismResult, result: DeterminismResult) -> bool {
	result := result
	if reference.seeded == false {
		result.seeded = true
		reference^ = result
		return true
	}

	ENSURE(t, result.sleepStep == reference.sleepStep) or_return
	ENSURE(t, result.hash == reference.hash) or_return
	ENSURE(t, result.queryHitCount == reference.queryHitCount) or_return
	ENSURE(t, result.queryHash == reference.queryHash) or_return

	return true
}

@(private = "file")
SingleMultithreadingTest :: proc(t: ^testing.T, workerCount: i32, reference: ^DeterminismResult) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldDef.workerCount = u32(workerCount)

	worldId := b3.CreateWorld(&worldDef)

	data := CreateFallingRagdolls(worldId)

	timeStep: f32 = 1.0 / 60.0

	stepLimit: i32 = 500
	for i: i32 = 0; i < stepLimit; i += 1 {
		subStepCount: i32 = 4
		b3.World_Step(worldId, timeStep, subStepCount)

		done := UpdateFallingRagdolls(worldId, &data)
		if done {
			break
		}
	}

	b3.DestroyWorld(worldId)

	if data.sleepStep != RAGDOLL_SLEEP_STEP || data.hash != RAGDOLL_HASH {
		printf("  workers=%d sleepStep=%d hash=0x%08X\n", workerCount, data.sleepStep, data.hash)
	}

	ENSURE_GOLDEN(t, data.sleepStep == RAGDOLL_SLEEP_STEP) or_return
	ENSURE_GOLDEN(t, data.hash == RAGDOLL_HASH) or_return
	ENSURE(
		t,
		EnsureRepeatable(t, reference, DeterminismResult{sleepStep = data.sleepStep, hash = data.hash}),
	) or_return

	DestroyFallingRagdolls(&data)

	return true
}

// Test multithreaded determinism.
@(private = "file")
MultithreadingTest :: proc(t: ^testing.T) -> bool {
	reference: DeterminismResult = {}
	for workerCount: i32 = 1; workerCount < 6; workerCount += 1 {
		result := SingleMultithreadingTest(t, workerCount, &reference)
		ENSURE(t, result) or_return
	}

	return true
}

// Test cross platform determinism.
@(private = "file")
CrossPlatformTest :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	data := CreateFallingRagdolls(worldId)

	timeStep: f32 = 1.0 / 60.0

	done := false
	for done == false {
		subStepCount: i32 = 4
		b3.World_Step(worldId, timeStep, subStepCount)

		done = UpdateFallingRagdolls(worldId, &data)
	}

	if data.sleepStep != RAGDOLL_SLEEP_STEP || data.hash != RAGDOLL_HASH {
		printf("  cross-platform sleepStep=%d hash=0x%08X\n", data.sleepStep, data.hash)
	}

	ENSURE_GOLDEN(t, data.sleepStep == RAGDOLL_SLEEP_STEP) or_return
	ENSURE_GOLDEN(t, data.hash == RAGDOLL_HASH) or_return

	DestroyFallingRagdolls(&data)

	b3.DestroyWorld(worldId)

	return true
}

@(private = "file")
SingleWavePileTest :: proc(t: ^testing.T, workerCount: i32, reference: ^DeterminismResult) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldDef.workerCount = u32(workerCount)

	worldId := b3.CreateWorld(&worldDef)

	data := CreateWavePile(worldId)

	timeStep: f32 = 1.0 / 60.0

	// Rolling resistance must put the pile to sleep within 500 steps
	done := false
	for i: i32 = 0; i < 500 && done == false; i += 1 {
		subStepCount: i32 = 4
		b3.World_Step(worldId, timeStep, subStepCount)

		done = UpdateWavePile(worldId, &data)
	}

	b3.DestroyWorld(worldId)

	if data.sleepStep != WAVE_PILE_SLEEP_STEP || data.hash != WAVE_PILE_HASH {
		printf("  wave pile workers=%d sleepStep=%d hash=0x%08X\n", workerCount, data.sleepStep, data.hash)
	}

	ENSURE(t, done == true) or_return
	ENSURE_GOLDEN(t, data.sleepStep == WAVE_PILE_SLEEP_STEP) or_return
	ENSURE_GOLDEN(t, data.hash == WAVE_PILE_HASH) or_return
	ENSURE(
		t,
		EnsureRepeatable(t, reference, DeterminismResult{sleepStep = data.sleepStep, hash = data.hash}),
	) or_return

	DestroyWavePile(&data)

	return true
}

// Test multithreaded determinism of a mixed convex pile on a wave height field.
@(private = "file")
WavePileTest :: proc(t: ^testing.T) -> bool {
	reference: DeterminismResult = {}
	for workerCount: i32 = 1; workerCount <= 4; workerCount += 1 {
		result := SingleWavePileTest(t, workerCount, &reference)
		ENSURE(t, result) or_return
	}

	return true
}

@(private = "file")
SingleQuerySpawnTest :: proc(t: ^testing.T, workerCount: i32, reference: ^DeterminismResult) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldDef.workerCount = u32(workerCount)

	worldId := b3.CreateWorld(&worldDef)

	data := CreateQuerySpawn(worldId)

	timeStep: f32 = 1.0 / 60.0

	done := false
	for i: i32 = 0; i < 1000 && done == false; i += 1 {
		subStepCount: i32 = 4
		b3.World_Step(worldId, timeStep, subStepCount)

		done = UpdateQuerySpawn(worldId, &data)
	}

	b3.DestroyWorld(worldId)

	if data.sleepStep != QUERY_SPAWN_SLEEP_STEP ||
	   data.hash != QUERY_SPAWN_HASH ||
	   data.queryHitCount != QUERY_SPAWN_HIT_COUNT ||
	   data.queryHash != QUERY_SPAWN_QUERY_HASH {
		printf(
			"  query spawn workers=%d sleepStep=%d hash=0x%08X hits=%d queryHash=0x%08X\n",
			workerCount,
			data.sleepStep,
			data.hash,
			data.queryHitCount,
			data.queryHash,
		)
	}

	ENSURE(t, done == true) or_return
	ENSURE(t, data.spawnCount == QUERY_SPAWN_COUNT) or_return
	ENSURE_GOLDEN(t, data.sleepStep == QUERY_SPAWN_SLEEP_STEP) or_return
	ENSURE_GOLDEN(t, data.hash == QUERY_SPAWN_HASH) or_return
	ENSURE_GOLDEN(t, data.queryHitCount == QUERY_SPAWN_HIT_COUNT) or_return
	ENSURE_GOLDEN(t, data.queryHash == QUERY_SPAWN_QUERY_HASH) or_return
	ENSURE(
		t,
		EnsureRepeatable(
			t,
			reference,
			DeterminismResult {
				sleepStep = data.sleepStep,
				hash = data.hash,
				queryHitCount = data.queryHitCount,
				queryHash = data.queryHash,
			},
		),
	) or_return

	DestroyQuerySpawn(&data)

	return true
}

// Test determinism of world queries by feeding their results back into the simulation.
@(private = "file")
QuerySpawnTest :: proc(t: ^testing.T) -> bool {
	reference: DeterminismResult = {}
	for workerCount: i32 = 1; workerCount <= 4; workerCount += 1 {
		result := SingleQuerySpawnTest(t, workerCount, &reference)
		ENSURE(t, result) or_return
	}

	return true
}

@(private = "file")
SingleMeshDropTest :: proc(t: ^testing.T, workerCount: i32, reference: ^DeterminismResult) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldDef.workerCount = u32(workerCount)

	worldId := b3.CreateWorld(&worldDef)

	data := CreateMeshDrop(worldId, b3.Pos_zero)

	timeStep: f32 = 1.0 / 60.0

	done := false
	for i: i32 = 0; i < 400 && done == false; i += 1 {
		subStepCount: i32 = 4
		b3.World_Step(worldId, timeStep, subStepCount)

		done = UpdateMeshDrop(worldId, &data)
	}

	b3.DestroyWorld(worldId)

	if data.sleepStep != MESH_DROP_SLEEP_STEP || data.hash != MESH_DROP_HASH {
		printf("  mesh drop workers=%d sleepStep=%d hash=0x%08X\n", workerCount, data.sleepStep, data.hash)
	}

	ENSURE(t, done == true) or_return
	ENSURE_GOLDEN(t, data.sleepStep == MESH_DROP_SLEEP_STEP) or_return
	ENSURE_GOLDEN(t, data.hash == MESH_DROP_HASH) or_return
	ENSURE(
		t,
		EnsureRepeatable(t, reference, DeterminismResult{sleepStep = data.sleepStep, hash = data.hash}),
	) or_return

	DestroyMeshDrop(&data)

	return true
}

// Test continuous collision determinism. Thin fast boxes need CCD against the wave mesh.
// The scene is large, so only the single threaded and widest schedules run.
@(private = "file")
MeshDropTest :: proc(t: ^testing.T) -> bool {
	reference: DeterminismResult = {}
	workerCounts := [2]i32{1, 4}
	for i: i32 = 0; i < 2; i += 1 {
		result := SingleMeshDropTest(t, workerCounts[i], &reference)
		ENSURE(t, result) or_return
	}

	return true
}

@(private = "file")
HashFloats :: proc(hash: u32, values: [^]f32, count: i32) -> u32 {
	return b3.Hash(hash, cast([^]u8)values, count * i32(size_of(f32)))
}

// Rolling resistance is mixed per contact and the manifolds have one, two, and four points, so a width 8
// constraint can mix lanes that width 4 keeps in separate constraints. The hash covers the stored contact
// impulses bitwise, including the sign of zero, because those survive into warm starting and contact data.
@(private = "file")
SingleRollingMixTest :: proc(t: ^testing.T, workerCount: i32, reference: ^DeterminismResult) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldDef.workerCount = u32(workerCount)
	worldId := b3.CreateWorld(&worldDef)

	groundDef := b3.DefaultBodyDef()
	groundId := b3.CreateBody(worldId, &groundDef)
	groundBox := b3.MakeBoxHull(40.0, 1.0, 40.0)
	groundShapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(groundId, &groundShapeDef, &groundBox.base)

	rowCount :: 8
	columnCount :: 8
	bodyCount :: rowCount * columnCount

	bodyIds: [bodyCount]b3.BodyId
	box := b3.MakeBoxHull(0.4, 0.4, 0.4)
	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.4}
	capsule := b3.Capsule{{-0.3, 0.0, 0.0}, {0.3, 0.0, 0.0}, 0.25}

	seed: u32 = 271828
	for i: i32 = 0; i < bodyCount; i += 1 {
		row := i / columnCount
		column := i % columnCount

		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{2.0 * f32(column) - 7.0, 1.45, 2.0 * f32(row) - 7.0}
		bodyIds[i] = b3.CreateBody(worldId, &bodyDef)

		seed = seed * 1664525 + 1013904223
		shapeDef := b3.DefaultShapeDef()
		shapeDef.density = 1.0
		shapeDef.baseMaterial.rollingResistance = (seed >> 28) < 5 ? 0.2 : 0.0

		switch i % 3 {
		case 0:
			b3.CreateSphereShape(bodyIds[i], &shapeDef, &sphere)
		case 1:
			b3.CreateCapsuleShape(bodyIds[i], &shapeDef, &capsule)
		case:
			b3.CreateHullShape(bodyIds[i], &shapeDef, &box.base)
		}

		sign: f32 = (i & 1) != 0 ? -1.0 : 1.0
		b3.Body_SetLinearVelocity(bodyIds[i], b3.Vec3{-0.0, -0.5, sign * 0.5})
		b3.Body_SetAngularVelocity(bodyIds[i], b3.Vec3{sign * 3.0, -0.0, -2.0 * sign})
	}

	hash: u32 = b3.HASH_INIT
	stepCount: i32 = 120
	for step: i32 = 0; step < stepCount; step += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)

		for i: i32 = 0; i < bodyCount; i += 1 {
			xf := b3.Body_GetTransform(bodyIds[i])
			hash = b3.Hash(hash, cast([^]u8)&xf, size_of(b3.WorldTransform))

			contactData: [4]b3.ContactData
			contactCount := b3.Body_GetContactData(bodyIds[i], &contactData[0], i32(len(contactData)))
			for c: i32 = 0; c < contactCount; c += 1 {
				for m: i32 = 0; m < contactData[c].manifoldCount; m += 1 {
					manifold := &contactData[c].manifolds[m]
					hash = HashFloats(hash, &manifold.rollingImpulse.x, 3)
					hash = HashFloats(hash, &manifold.frictionImpulse.x, 3)
					hash = HashFloats(hash, &manifold.twistImpulse, 1)

					for p: i32 = 0; p < manifold.pointCount; p += 1 {
						hash = HashFloats(hash, &manifold.points[p].normalImpulse, 1)
						hash = HashFloats(hash, &manifold.points[p].totalNormalImpulse, 1)
					}
				}
			}
		}
	}

	b3.DestroyWorld(worldId)

	ENSURE(t, EnsureRepeatable(t, reference, DeterminismResult{sleepStep = stepCount, hash = hash})) or_return
	return true
}

// Mesh contacts are grouped across lanes by manifold count and padded to the largest count in the group. Width 8 groups
// differ from width 4 groups, so mixing large and small shapes on a fine mesh gives each body a width dependent number
// of padded manifolds. The hash covers velocities and impulses bitwise to catch a padded manifold flipping a signed zero.
@(private = "file")
SingleMeshMixTest :: proc(t: ^testing.T, workerCount: i32, reference: ^DeterminismResult) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldDef.workerCount = u32(workerCount)
	worldId := b3.CreateWorld(&worldDef)

	mesh := b3.CreateWaveMesh(40, 40, 0.5, 0.3, 0.2, 0.3)
	{
		groundDef := b3.DefaultBodyDef()
		groundDef.position = b3.Pos{-10.0, 0.0, -10.0}
		groundId := b3.CreateBody(worldId, &groundDef)
		shapeDef := b3.DefaultShapeDef()
		b3.CreateMeshShape(groundId, &shapeDef, mesh, b3.Vec3_one)
	}

	rowCount :: 8
	columnCount :: 8
	bodyCount :: rowCount * columnCount

	bodyIds: [bodyCount]b3.BodyId
	smallBox := b3.MakeBoxHull(0.2, 0.2, 0.2)
	largeBox := b3.MakeBoxHull(0.9, 0.3, 0.9)
	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.3}
	capsule := b3.Capsule{{-0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, 0.2}

	seed: u32 = 314159
	for i: i32 = 0; i < bodyCount; i += 1 {
		row := i / columnCount
		column := i % columnCount

		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{2.2 * f32(column) - 7.7, 1.2, 2.2 * f32(row) - 7.7}
		bodyIds[i] = b3.CreateBody(worldId, &bodyDef)

		seed = seed * 1664525 + 1013904223
		shapeDef := b3.DefaultShapeDef()
		shapeDef.density = 1.0
		shapeDef.baseMaterial.rollingResistance = (seed >> 28) < 5 ? 0.2 : 0.0

		switch (seed >> 24) & 3 {
		case 0:
			b3.CreateHullShape(bodyIds[i], &shapeDef, &largeBox.base)
		case 1:
			b3.CreateHullShape(bodyIds[i], &shapeDef, &smallBox.base)
		case 2:
			b3.CreateSphereShape(bodyIds[i], &shapeDef, &sphere)
		case:
			b3.CreateCapsuleShape(bodyIds[i], &shapeDef, &capsule)
		}

		sign: f32 = (i & 1) != 0 ? -1.0 : 1.0
		b3.Body_SetLinearVelocity(bodyIds[i], b3.Vec3{-0.0, -0.5, -0.0})
		b3.Body_SetAngularVelocity(bodyIds[i], b3.Vec3{-0.0, sign * 0.5, -0.0})
	}

	hash: u32 = b3.HASH_INIT
	stepCount: i32 = 180
	for step: i32 = 0; step < stepCount; step += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)

		for i: i32 = 0; i < bodyCount; i += 1 {
			xf := b3.Body_GetTransform(bodyIds[i])
			hash = b3.Hash(hash, cast([^]u8)&xf, size_of(b3.WorldTransform))

			v := b3.Body_GetLinearVelocity(bodyIds[i])
			w := b3.Body_GetAngularVelocity(bodyIds[i])
			hash = HashFloats(hash, &v.x, 3)
			hash = HashFloats(hash, &w.x, 3)

			contactData: [8]b3.ContactData
			contactCount := b3.Body_GetContactData(bodyIds[i], &contactData[0], i32(len(contactData)))
			for c: i32 = 0; c < contactCount; c += 1 {
				for m: i32 = 0; m < contactData[c].manifoldCount; m += 1 {
					manifold := &contactData[c].manifolds[m]
					hash = HashFloats(hash, &manifold.rollingImpulse.x, 3)
					hash = HashFloats(hash, &manifold.frictionImpulse.x, 3)
					hash = HashFloats(hash, &manifold.twistImpulse, 1)

					for p: i32 = 0; p < manifold.pointCount; p += 1 {
						hash = HashFloats(hash, &manifold.points[p].normalImpulse, 1)
						hash = HashFloats(hash, &manifold.points[p].totalNormalImpulse, 1)
					}
				}
			}
		}
	}

	b3.DestroyWorld(worldId)
	b3.DestroyMesh(mesh)

	ENSURE(t, EnsureRepeatable(t, reference, DeterminismResult{sleepStep = stepCount, hash = hash})) or_return
	return true
}

@(private = "file")
SceneFcn :: #type proc(t: ^testing.T, workerCount: i32, reference: ^DeterminismResult) -> bool

@(private = "file")
RunSceneAtWidth :: proc(t: ^testing.T, scene: SceneFcn, width: i32, result: ^DeterminismResult) -> bool {
	b3.SetSIMDWidth(width)
	status := scene(t, 1, result)
	b3.SetSIMDWidth(0)
	return status
}

// Width 4 and the native width must produce bitwise identical simulations.
@(private = "file")
SIMDWidthTest :: proc(t: ^testing.T) -> bool {
	b3.SetSIMDWidth(0)
	nativeWidth := b3.GetSIMDWidth()
	if nativeWidth == 4 {
		printf("  subtest skipped: SIMDWidthTest, native SIMD width is 4\n")
		return true
	}

	scenes := [?]SceneFcn {
		SingleMultithreadingTest,
		SingleWavePileTest,
		SingleQuerySpawnTest,
		SingleMeshDropTest,
		SingleRollingMixTest,
		SingleMeshMixTest,
	}

	for i: i32 = 0; i < i32(len(scenes)); i += 1 {
		narrow: DeterminismResult = {}
		wide: DeterminismResult = {}

		narrowStatus := RunSceneAtWidth(t, scenes[i], 4, &narrow)
		wideStatus := RunSceneAtWidth(t, scenes[i], nativeWidth, &wide)

		ENSURE(t, narrowStatus) or_return
		ENSURE(t, wideStatus) or_return
		ENSURE(t, narrow.seeded && wide.seeded) or_return
		ENSURE(t, narrow.sleepStep == wide.sleepStep) or_return
		ENSURE(t, narrow.hash == wide.hash) or_return
		ENSURE(t, narrow.queryHitCount == wide.queryHitCount) or_return
		ENSURE(t, narrow.queryHash == wide.queryHash) or_return
	}

	return true
}

@(test)
DeterminismTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, MultithreadingTest) {return}
	if !RUN_SUBTEST(t, CrossPlatformTest) {return}
	if !RUN_SUBTEST(t, WavePileTest) {return}
	if !RUN_SUBTEST(t, QuerySpawnTest) {return}
	if !RUN_SUBTEST(t, MeshDropTest) {return}
	if !RUN_SUBTEST(t, SIMDWidthTest) {return}
}
