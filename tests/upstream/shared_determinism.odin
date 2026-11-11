// Ported from box3d shared/determinism.c and shared/determinism.h at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "base:runtime"

RAGDOLL_GROUP_SIZE :: 2
RAGDOLL_GRID_COUNT :: 2

RagdollGroup :: struct {
	humans: [RAGDOLL_GROUP_SIZE]Human,
}

FallingRagdollData :: struct {
	groups:      [RAGDOLL_GRID_COUNT * RAGDOLL_GRID_COUNT]RagdollGroup,
	gridMesh:    ^b3.MeshData,
	torusMesh:   ^b3.MeshData,
	columnCount: i32,
	columnIndex: i32,
	stepCount:   i32,
	sleepStep:   i32,
	hash:        u32,
}

// Convex pile dropped on a wave height field. Spheres, capsules, boxes, and rocks with
// rolling resistance so the pile sleeps quickly.
WAVE_PILE_BODY_COUNT :: 100

WavePileData :: struct {
	bodies:      [WAVE_PILE_BODY_COUNT]b3.BodyId,
	heightField: ^b3.HeightFieldData,
	stepCount:   i32,
	sleepStep:   i32,
	hash:        u32,
}

// Query driven spawning in an empty zero gravity world. Each step casts a ray, overlaps an
// AABB, and casts a sphere, then spawns a shape whose position, type, and size depend on the
// query results. Any query divergence cascades into the final state.
QUERY_SPAWN_COUNT :: 50
QUERY_SPAWN_CAST_RADIUS :: f32(0.5)

QuerySpawnData :: struct {
	bodies:            [QUERY_SPAWN_COUNT]b3.BodyId,
	spawnCount:        i32,
	queryHitCount:     i32,
	queryHash:         u32,
	stepCount:         i32,
	sleepStep:         i32,
	hash:              u32,

	// Last query cycle, recorded for visualization
	rayOrigin:         b3.Pos,
	rayTranslation:    b3.Vec3,
	rayPoint:          b3.Pos,
	rayNormal:         b3.Vec3,
	rayDidHit:         bool,
	overlapBounds:     b3.AABB,
	overlapCount:      i32,
	castFraction:      f32,
	lastSpawnPosition: b3.Pos,
}

@(private = "file")
GRID_SIZE :: f32(15.0)

@(private = "file")
CreateGroup :: proc(data: ^FallingRagdollData, worldId: b3.WorldId, rowIndex: i32, columnIndex: i32) {
	assert(rowIndex < RAGDOLL_GRID_COUNT && columnIndex < RAGDOLL_GRID_COUNT)

	groupIndex := rowIndex * RAGDOLL_GRID_COUNT + columnIndex

	span: f32 = RAGDOLL_GRID_COUNT * GRID_SIZE
	groupDistance := 1.0 * span / RAGDOLL_GRID_COUNT

	position: b3.Pos
	position.x = -0.5 * span + groupDistance * (f32(columnIndex) + 0.5)
	position.y = 15.0
	position.z = -0.5 * span + groupDistance * (f32(rowIndex) + 0.5)

	frictionTorque: f32 = 5.0
	hertz: f32 = 1.0
	dampingRatio: f32 = 0.7
	colorize := false

	for i: i32 = 0; i < RAGDOLL_GROUP_SIZE; i += 1 {
		human := &data.groups[groupIndex].humans[i]
		CreateHuman(human, worldId, position, frictionTorque, hertz, dampingRatio, groupIndex, nil, colorize)
		position.x += 0.75
	}
}

CreateFallingRagdolls :: proc(worldId: b3.WorldId) -> FallingRagdollData {
	data: FallingRagdollData = {}

	halfMeshGridRows: i32 = 4
	meshGridCellWidth := GRID_SIZE / (2.0 * f32(halfMeshGridRows))
	data.gridMesh = b3.CreateGridMesh(2 * halfMeshGridRows, 2 * halfMeshGridRows, meshGridCellWidth, 0, true)
	data.torusMesh = b3.CreateTorusMesh(16, 16, 0.25 * GRID_SIZE, 1.0)

	span: f32 = GRID_SIZE * RAGDOLL_GRID_COUNT
	bodyDef := b3.DefaultBodyDef()
	shapeDef := b3.DefaultShapeDef()

	bodyDef.position.x = -0.5 * span + 0.5 * GRID_SIZE
	for i: i32 = 0; i < RAGDOLL_GRID_COUNT; i += 1 {
		bodyDef.position.z = -0.5 * span + 0.5 * GRID_SIZE
		for j: i32 = 0; j < RAGDOLL_GRID_COUNT; j += 1 {
			body := b3.CreateBody(worldId, &bodyDef)
			b3.CreateMeshShape(body, &shapeDef, data.gridMesh, b3.Vec3_one)
			b3.CreateMeshShape(body, &shapeDef, data.torusMesh, b3.Vec3_one)

			CreateGroup(&data, worldId, i, j)

			bodyDef.position.z += GRID_SIZE
		}

		bodyDef.position.x += GRID_SIZE
	}

	return data
}

UpdateFallingRagdolls :: proc(worldId: b3.WorldId, data: ^FallingRagdollData) -> bool {
	if data.hash == 0 {
		bodyEvents := b3.World_GetBodyEvents(worldId)

		if bodyEvents.moveCount == 0 {
			awakeCount := b3.World_GetAwakeBodyCount(worldId)
			assert(awakeCount == 0)

			data.hash = b3.HASH_INIT
			for i: i32 = 0; i < RAGDOLL_GRID_COUNT; i += 1 {
				for j: i32 = 0; j < RAGDOLL_GRID_COUNT; j += 1 {
					for k: i32 = 0; k < RAGDOLL_GROUP_SIZE; k += 1 {
						groupIndex := i * RAGDOLL_GRID_COUNT + j

						human := &data.groups[groupIndex].humans[k]

						for b: i32 = 0; b < bone_count; b += 1 {
							bodyId := human.bones[b].bodyId
							xf := b3.Body_GetTransform(bodyId)
							data.hash = b3.Hash(data.hash, cast([^]u8)(&xf), size_of(b3.WorldTransform))
						}
					}
				}
			}

			data.sleepStep = data.stepCount
		}
	}

	data.stepCount += 1

	return data.hash != 0
}

DestroyFallingRagdolls :: proc(data: ^FallingRagdollData) {
	b3.DestroyMesh(data.gridMesh)
	b3.DestroyMesh(data.torusMesh)

	data.gridMesh = nil
	data.torusMesh = nil
}

@(private = "file")
WAVE_PILE_GRID :: 5
@(private = "file")
WAVE_PILE_LAYERS :: 4

CreateWavePile :: proc(worldId: b3.WorldId) -> WavePileData {
	data: WavePileData = {}

	g_randomSeed = 52977

	// Height fields grow from a corner, offset the body to center the patch.
	fieldCount: i32 = 21
	fieldScale := b3.Vec3{1.0, 0.6, 1.0}
	data.heightField = b3.CreateWave(fieldCount, fieldCount, fieldScale, 0.08, 0.06, false)

	{
		extent := fieldScale.x * f32(fieldCount - 1)
		bodyDef := b3.DefaultBodyDef()
		bodyDef.position.x = -0.5 * extent
		bodyDef.position.z = -0.5 * extent
		groundId := b3.CreateBody(worldId, &bodyDef)

		shapeDef := b3.DefaultShapeDef()
		b3.CreateHeightFieldShape(groundId, &shapeDef, data.heightField)
	}

	rock := b3.CreateRock(0.55)
	box := b3.MakeBoxHull(0.45, 0.3, 0.55)
	sphere := b3.Sphere{b3.Vec3_zero, 0.5}
	capsule := b3.Capsule{{0.0, -0.3, 0.0}, {0.0, 0.3, 0.0}, 0.35}

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody

	// Rolling resistance so the pile sleeps quickly.
	shapeDef := b3.DefaultShapeDef()
	shapeDef.baseMaterial.rollingResistance = 0.3

	spacing: f32 = 1.7
	index: i32 = 0
	for layer: i32 = 0; layer < WAVE_PILE_LAYERS; layer += 1 {
		for i: i32 = 0; i < WAVE_PILE_GRID; i += 1 {
			for j: i32 = 0; j < WAVE_PILE_GRID; j += 1 {
				jitter := RandomVec3Uniform(-0.3, 0.3)
				// C: 0.5f * ( WAVE_PILE_GRID - 1 ) folds to 2.0f
				bodyDef.position.x = spacing * (f32(i) - 0.5 * (WAVE_PILE_GRID - 1)) + jitter.x
				bodyDef.position.y = 2.5 + 1.6 * f32(layer) + 0.3 * jitter.y
				bodyDef.position.z = spacing * (f32(j) - 0.5 * (WAVE_PILE_GRID - 1)) + jitter.z
				bodyDef.rotation = RandomQuat()

				bodyId := b3.CreateBody(worldId, &bodyDef)
				data.bodies[index] = bodyId

				switch index % 4 {
				case 0:
					b3.CreateSphereShape(bodyId, &shapeDef, &sphere)
				case 1:
					b3.CreateCapsuleShape(bodyId, &shapeDef, &capsule)
				case 2:
					b3.CreateHullShape(bodyId, &shapeDef, &box.base)
				case:
					b3.CreateHullShape(bodyId, &shapeDef, rock)
				}

				index += 1
			}
		}
	}

	assert(index == WAVE_PILE_BODY_COUNT)

	// The world keeps its own copy.
	b3.DestroyHull(rock)

	return data
}

UpdateWavePile :: proc(worldId: b3.WorldId, data: ^WavePileData) -> bool {
	if data.hash == 0 {
		if b3.World_GetAwakeBodyCount(worldId) == 0 {
			data.hash = b3.HASH_INIT
			for i: i32 = 0; i < WAVE_PILE_BODY_COUNT; i += 1 {
				xf := b3.Body_GetTransform(data.bodies[i])
				data.hash = b3.Hash(data.hash, cast([^]u8)&xf, size_of(b3.WorldTransform))
			}

			data.sleepStep = data.stepCount
		}
	}

	data.stepCount += 1

	return data.hash != 0
}

DestroyWavePile :: proc(data: ^WavePileData) {
	b3.DestroyHeightField(data.heightField)
	data.heightField = nil
}

@(private = "file")
QuerySpawnOverlapContext :: struct {
	data:  ^QuerySpawnData,
	count: i32,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
QuerySpawnOverlapCallback :: proc "c" (shapeId: b3.ShapeId, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	shapeId := shapeId
	overlap := (^QuerySpawnOverlapContext)(context_)
	overlap.count += 1
	overlap.data.queryHitCount += 1
	overlap.data.queryHash = b3.Hash(overlap.data.queryHash, cast([^]u8)&shapeId.index1, size_of(shapeId.index1))
	return true
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
QuerySpawnCastCallback :: proc "c" (
	shapeId: b3.ShapeId,
	point: b3.Pos,
	normal: b3.Vec3,
	fraction: f32,
	userMaterialId: u64,
	triangleIndex: i32,
	childIndex: i32,
	context_: rawptr,
) -> f32 {
	context = runtime.default_context()
	// unified end
	_, _, _, _, _, _ = shapeId, point, normal, userMaterialId, triangleIndex, childIndex
	closest := (^f32)(context_)
	closest^ = fraction
	return fraction
}

@(private = "file")
QuerySpawnOnce :: proc(worldId: b3.WorldId, data: ^QuerySpawnData) {
	filter := b3.DefaultQueryFilter()

	// Ray result decides the spawn position, a miss seeds the cloud near the origin.
	rayOrigin := RandomPos(b3.Vec3{-12.0, -12.0, -12.0}, b3.Vec3{12.0, 12.0, 12.0})
	rayTranslation := b3.MulSV(30.0, RandomUnitVector())

	spawnPosition: b3.Pos
	ray := b3.World_CastRayClosest(worldId, rayOrigin, rayTranslation, filter)
	if ray.hit {
		data.queryHitCount += 1
		data.queryHash = b3.Hash(data.queryHash, cast([^]u8)&ray.fraction, size_of(ray.fraction))
		data.queryHash = b3.Hash(data.queryHash, cast([^]u8)&ray.normal, size_of(ray.normal))
		spawnPosition = b3.OffsetPos(ray.point, b3.MulSV(1.2, ray.normal))
	} else {
		spawnPosition = RandomPos(b3.Vec3{-6.0, -6.0, -6.0}, b3.Vec3{6.0, 6.0, 6.0})
	}

	data.rayOrigin = rayOrigin
	data.rayTranslation = rayTranslation
	data.rayDidHit = ray.hit
	data.rayPoint = ray.hit ? ray.point : b3.OffsetPos(rayOrigin, rayTranslation)
	data.rayNormal = ray.hit ? ray.normal : b3.Vec3_zero

	// Overlap count picks the shape type.
	center := RandomVec3Uniform(-10.0, 10.0)
	extent := RandomFloatRange(1.0, 4.0)
	aabb: b3.AABB
	aabb.lowerBound = b3.Vec3{center.x - extent, center.y - extent, center.z - extent}
	aabb.upperBound = b3.Vec3{center.x + extent, center.y + extent, center.z + extent}

	overlap := QuerySpawnOverlapContext{data, 0}
	b3.World_OverlapAABB(worldId, aabb, filter, QuerySpawnOverlapCallback, &overlap)

	data.overlapBounds = aabb
	data.overlapCount = overlap.count

	// Sphere cast fraction sets the spawn size.
	fraction: f32 = 1.0
	proxyPoint := b3.Vec3_zero
	proxy := b3.ShapeProxy{&proxyPoint, 1, QUERY_SPAWN_CAST_RADIUS}
	b3.World_CastShape(worldId, rayOrigin, &proxy, rayTranslation, filter, QuerySpawnCastCallback, &fraction)
	if fraction < 1.0 {
		data.queryHitCount += 1
		data.queryHash = b3.Hash(data.queryHash, cast([^]u8)&fraction, size_of(fraction))
	}

	data.castFraction = fraction

	size := 0.3 + 0.2 * fraction

	// Damping guarantees everything comes to rest.
	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = spawnPosition
	bodyDef.rotation = RandomQuat()
	bodyDef.linearVelocity = RandomVec3Uniform(-0.2, 0.2)
	bodyDef.angularVelocity = RandomVec3Uniform(-0.5, 0.5)
	bodyDef.linearDamping = 1.0
	bodyDef.angularDamping = 1.0

	bodyId := b3.CreateBody(worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()
	shapeDef.baseMaterial.rollingResistance = 0.2

	switch (data.spawnCount + overlap.count) % 3 {
	case 0:
		sphere := b3.Sphere{b3.Vec3_zero, size}
		b3.CreateSphereShape(bodyId, &shapeDef, &sphere)
	case 1:
		capsule := b3.Capsule{{0.0, -size, 0.0}, {0.0, size, 0.0}, 0.7 * size}
		b3.CreateCapsuleShape(bodyId, &shapeDef, &capsule)
	case:
		box := b3.MakeBoxHull(size, 0.7 * size, 0.5 * size)
		b3.CreateHullShape(bodyId, &shapeDef, &box.base)
	}

	data.bodies[data.spawnCount] = bodyId
	data.spawnCount += 1
	data.lastSpawnPosition = spawnPosition
}

CreateQuerySpawn :: proc(worldId: b3.WorldId) -> QuerySpawnData {
	data: QuerySpawnData = {}

	g_randomSeed = 71689

	// Empty space, motion comes only from spawn velocities and overlap pushes.
	b3.World_SetGravity(worldId, b3.Vec3_zero)

	return data
}

UpdateQuerySpawn :: proc(worldId: b3.WorldId, data: ^QuerySpawnData) -> bool {
	if data.spawnCount < QUERY_SPAWN_COUNT {
		QuerySpawnOnce(worldId, data)
	} else if data.hash == 0 && b3.World_GetAwakeBodyCount(worldId) == 0 {
		data.hash = b3.HASH_INIT
		for i: i32 = 0; i < QUERY_SPAWN_COUNT; i += 1 {
			xf := b3.Body_GetTransform(data.bodies[i])
			data.hash = b3.Hash(data.hash, cast([^]u8)&xf, size_of(b3.WorldTransform))
		}

		data.sleepStep = data.stepCount
	}

	data.stepCount += 1

	return data.hash != 0
}

DestroyQuerySpawn :: proc(data: ^QuerySpawnData) {
	_ = data
}
