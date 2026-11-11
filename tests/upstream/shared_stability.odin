// Ported from box3d shared/stability.c and shared/stability.h at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"

MESH_DROP_GRID_COUNT :: 20

// Thin fast boxes dropped on a wave mesh. Stresses continuous collision and mesh contact
// stability, and doubles as a determinism scenario via the sleep hash.
MeshDropData :: struct {
	mesh:      ^b3.MeshData,
	bodies:    [MESH_DROP_GRID_COUNT * MESH_DROP_GRID_COUNT]b3.BodyId,
	stepCount: i32,
	sleepStep: i32,
	hash:      u32,
}

CreateMeshDrop :: proc(worldId: b3.WorldId, origin: b3.Pos) -> MeshDropData {
	data: MeshDropData = {}
	{
		bodyDef := b3.DefaultBodyDef()
		bodyDef.position = origin
		groundId := b3.CreateBody(worldId, &bodyDef)

		gridCount: i32 = 40
		cellWidth: f32 = 1.0
		rowHz: f32 = 0.1
		columnHz: f32 = 0.2
		groundAmplitude: f32 = 0.5

		data.mesh = b3.CreateWaveMesh(gridCount, gridCount, cellWidth, groundAmplitude, rowHz, columnHz)
		shapeDef := b3.DefaultShapeDef()
		shapeDef.filter.categoryBits = 1
		b3.CreateMeshShape(groundId, &shapeDef, data.mesh, b3.Vec3_one)
	}

	{
		box := b3.MakeBoxHull(0.02, 0.2, 0.04)

		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody

		shapeDef := b3.DefaultShapeDef()
		shapeDef.baseMaterial.rollingResistance = 0.1

		// Don't allow shapes to collide with each other.
		shapeDef.filter.categoryBits = 2
		shapeDef.filter.maskBits = 1

		g_randomSeed = 3963634789

		gridCount: i32 = MESH_DROP_GRID_COUNT

		for i: i32 = 0; i < gridCount; i += 1 {
			for j: i32 = 0; j < gridCount; j += 1 {
				linearVelocity := RandomVec3Uniform(-1.0, 1.0)
				angularVelocity := RandomVec3Uniform(-5.0, 5.0)

				// C: 0.5f * ( i - 0.5f * gridCount ), each step rounded to float
				bodyDef.position = b3.OffsetPos(
					origin,
					b3.Vec3{0.5 * (f32(i) - 0.5 * f32(gridCount)), 5.0, 0.5 * (f32(j) - 0.5 * f32(gridCount))},
				)
				bodyDef.linearVelocity = linearVelocity
				bodyDef.angularVelocity = angularVelocity
				bodyId := b3.CreateBody(worldId, &bodyDef)
				data.bodies[i * gridCount + j] = bodyId

				b3.CreateHullShape(bodyId, &shapeDef, &box.base)
			}
		}
	}

	return data
}

UpdateMeshDrop :: proc(worldId: b3.WorldId, data: ^MeshDropData) -> bool {
	if data.hash == 0 {
		if b3.World_GetAwakeBodyCount(worldId) == 0 {
			data.hash = b3.HASH_INIT
			bodyCount: i32 = MESH_DROP_GRID_COUNT * MESH_DROP_GRID_COUNT
			for i: i32 = 0; i < bodyCount; i += 1 {
				xf := b3.Body_GetTransform(data.bodies[i])
				data.hash = b3.Hash(data.hash, cast([^]u8)&xf, size_of(b3.WorldTransform))
			}

			data.sleepStep = data.stepCount
		}
	}

	data.stepCount += 1

	return data.hash != 0
}

DestroyMeshDrop :: proc(data: ^MeshDropData) {
	b3.DestroyMesh(data.mesh)
}
