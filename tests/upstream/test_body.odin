// Ported from box3d test/test_body.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "core:testing"

// b3UpdateBodyMassData shifts each shape's inertia to the body center of mass with the parallel
// axis theorem. When shapes sit far from the body origin the shift term dwarfs the central inertia,
// so any error in the per shape framing blows up the tensor. Spheres make a clean oracle: the
// central inertia is isotropic and independent of placement, so the shift is the only thing tested.

@(private = "file")
SphereBodyMass :: proc(centers: [^]b3.Vec3, count: i32, radius: f32, density: f32) -> b3.MassData {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyId := b3.CreateBody(worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()
	shapeDef.density = density

	for i: i32 = 0; i < count; i += 1 {
		sphere := b3.Sphere{centers[i], radius}
		b3.CreateSphereShape(bodyId, &shapeDef, &sphere)
	}

	b3.Body_ApplyMassFromShapes(bodyId)
	massData := b3.Body_GetMassData(bodyId)

	b3.DestroyWorld(worldId)
	return massData
}

// One sphere far from the body origin. The center of mass lands on the sphere and the inertia about
// it must be the bare central inertia, with no trace of the offset.
@(private = "file")
FarSingleSphereMass :: proc(t: ^testing.T) -> bool {
	radius: f32 = 0.5
	density: f32 = 1.0
	center := b3.Vec3{100.0, -50.0, 75.0}
	md := SphereBodyMass(&center, 1, radius, density)

	mass := density * (f32(4.0) / f32(3.0)) * b3.PI * radius * radius * radius
	central := 0.4 * mass * radius * radius

	ENSURE_SMALL(t, md.mass - mass, 1e-4) or_return

	ENSURE_SMALL(t, md.center.x - center.x, 1e-3) or_return
	ENSURE_SMALL(t, md.center.y - center.y, 1e-3) or_return
	ENSURE_SMALL(t, md.center.z - center.z, 1e-3) or_return

	ENSURE_SMALL(t, md.inertia.cx.x - central, 1e-3) or_return
	ENSURE_SMALL(t, md.inertia.cy.y - central, 1e-3) or_return
	ENSURE_SMALL(t, md.inertia.cz.z - central, 1e-3) or_return

	ENSURE_SMALL(t, md.inertia.cy.x, 1e-3) or_return
	ENSURE_SMALL(t, md.inertia.cz.x, 1e-3) or_return
	ENSURE_SMALL(t, md.inertia.cz.y, 1e-3) or_return

	return true
}

// Eight equal spheres on the corners of a cube, the whole cube parked far from the body origin.
// The center of mass is the cube center and the products of inertia cancel by symmetry, so the
// tensor stays diagonal no matter how far out the cube sits.
@(private = "file")
FarCubeSphereMass :: proc(t: ^testing.T) -> bool {
	radius: f32 = 0.5
	density: f32 = 1.0
	h: f32 = 1.0
	p := b3.Vec3{100.0, 100.0, 100.0}

	centers: [8]b3.Vec3
	k := 0
	for sx: i32 = -1; sx <= 1; sx += 2 {
		for sy: i32 = -1; sy <= 1; sy += 2 {
			for sz: i32 = -1; sz <= 1; sz += 2 {
				centers[k] = b3.Vec3{p.x + f32(sx) * h, p.y + f32(sy) * h, p.z + f32(sz) * h}
				k += 1
			}
		}
	}

	md := SphereBodyMass(&centers[0], 8, radius, density)

	mass := density * (f32(4.0) / f32(3.0)) * b3.PI * radius * radius * radius
	totalMass := 8.0 * mass

	// Per sphere central inertia summed, plus the parallel axis term for each corner offset
	// (dy^2 + dz^2) = (h^2 + h^2) about every axis.
	diag := f32(8.0) * f32(0.4) * mass * radius * radius + 16.0 * mass * h * h

	ENSURE_SMALL(t, md.mass - totalMass, 1e-3) or_return

	ENSURE_SMALL(t, md.center.x - p.x, 1e-2) or_return
	ENSURE_SMALL(t, md.center.y - p.y, 1e-2) or_return
	ENSURE_SMALL(t, md.center.z - p.z, 1e-2) or_return

	ENSURE_SMALL(t, md.inertia.cx.x - diag, 1e-2) or_return
	ENSURE_SMALL(t, md.inertia.cy.y - diag, 1e-2) or_return
	ENSURE_SMALL(t, md.inertia.cz.z - diag, 1e-2) or_return

	ENSURE_SMALL(t, md.inertia.cy.x, 1e-2) or_return
	ENSURE_SMALL(t, md.inertia.cz.x, 1e-2) or_return
	ENSURE_SMALL(t, md.inertia.cz.y, 1e-2) or_return

	return true
}

// Shapes added with updateBodyMass = false defer the mass update, which is also the only place
// body extents are computed. A body that reaches the solver with minExtent == B3_HUGE never passes
// the continuous collision gate. The dirty mass flag must track the deferral, and both
// ApplyMassFromShapes and SetMassData must leave finite extents behind.
@(private = "file")
DeferredMassExtents :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody

	shapeDef := b3.DefaultShapeDef()
	shapeDef.density = 1.0
	shapeDef.updateBodyMass = false

	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}

	// Deferred create leaves mass and extents untouched but marks the body dirty.
	applyId := b3.CreateBody(worldId, &bodyDef)
	b3.CreateSphereShape(applyId, &shapeDef, &sphere)

	world := b3.GetWorld(i32(applyId.world0))
	applyBody := b3.GetBodyFullId(world, applyId)
	applySim := b3.GetBodySim(world, applyBody)

	ENSURE(t, (applyBody.flags & b3.dirtyMass) != 0) or_return
	ENSURE(t, applySim.minExtent == b3.HUGE) or_return

	// ApplyMassFromShapes computes extents and clears the flag.
	b3.Body_ApplyMassFromShapes(applyId)
	ENSURE(t, (applyBody.flags & b3.dirtyMass) == 0) or_return
	ENSURE(t, applySim.minExtent < b3.HUGE) or_return

	// SetMassData alone must also produce finite extents and clear the flag (the issue #35 repro).
	massId := b3.CreateBody(worldId, &bodyDef)
	b3.CreateSphereShape(massId, &shapeDef, &sphere)

	massBody := b3.GetBodyFullId(world, massId)
	massSim := b3.GetBodySim(world, massBody)
	ENSURE(t, (massBody.flags & b3.dirtyMass) != 0) or_return

	inertia := b3.Matrix3{{0.2, 0.0, 0.0}, {0.0, 0.2, 0.0}, {0.0, 0.0, 0.2}}
	massData := b3.MassData{2.0, {0.0, 0.0, 0.0}, inertia}
	b3.Body_SetMassData(massId, massData)

	ENSURE(t, (massBody.flags & b3.dirtyMass) == 0) or_return
	ENSURE(t, massSim.minExtent < b3.HUGE) or_return

	b3.DestroyWorld(worldId)
	return true
}

// b3Body_SetMassData overrides the mass properties directly, bypassing the shapes. It must derive
// everything the solver reads from the supplied tensor: the inverse mass, the local inverse inertia,
// and the world inverse inertia rotated by the body orientation. Fixed rotation zeros the angular part.
// These tests drive it through the public getters, no shapes required.

// Diagonal inertia with inverses that are exact in float, so tolerances stay tight.
@(private = "file")
kDiagInertia :: b3.Matrix3{{2.0, 0.0, 0.0}, {0.0, 4.0, 0.0}, {0.0, 0.0, 8.0}}

@(private = "file")
SetMassDataRoundTrip :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = b3.Pos{5.0, -3.0, 2.0}
	bodyId := b3.CreateBody(worldId, &bodyDef)

	center := b3.Vec3{0.1, 0.2, 0.3}
	massData := b3.MassData{3.0, center, kDiagInertia}
	b3.Body_SetMassData(bodyId, massData)

	ENSURE_SMALL(t, b3.Body_GetMass(bodyId) - 3.0, 1e-6) or_return
	ENSURE_SMALL(t, b3.Body_GetInverseMass(bodyId) - f32(1.0) / f32(3.0), 1e-6) or_return

	md := b3.Body_GetMassData(bodyId)
	ENSURE_SMALL(t, md.mass - 3.0, 1e-6) or_return
	ENSURE_SMALL(t, md.center.x - center.x, 1e-6) or_return
	ENSURE_SMALL(t, md.center.y - center.y, 1e-6) or_return
	ENSURE_SMALL(t, md.center.z - center.z, 1e-6) or_return
	ENSURE_SMALL(t, md.inertia.cx.x - 2.0, 1e-6) or_return
	ENSURE_SMALL(t, md.inertia.cy.y - 4.0, 1e-6) or_return
	ENSURE_SMALL(t, md.inertia.cz.z - 8.0, 1e-6) or_return

	localCenter := b3.Body_GetLocalCenter(bodyId)
	ENSURE_SMALL(t, localCenter.x - center.x, 1e-6) or_return
	ENSURE_SMALL(t, localCenter.y - center.y, 1e-6) or_return
	ENSURE_SMALL(t, localCenter.z - center.z, 1e-6) or_return

	localInertia := b3.Body_GetLocalRotationalInertia(bodyId)
	ENSURE_SMALL(t, localInertia.cx.x - 2.0, 1e-6) or_return
	ENSURE_SMALL(t, localInertia.cy.y - 4.0, 1e-6) or_return
	ENSURE_SMALL(t, localInertia.cz.z - 8.0, 1e-6) or_return

	// Identity rotation, so the world inverse inertia is just the local inverse: diag(1/2, 1/4, 1/8).
	invWorld := b3.Body_GetWorldInverseRotationalInertia(bodyId)
	ENSURE_SMALL(t, invWorld.cx.x - 0.5, 1e-5) or_return
	ENSURE_SMALL(t, invWorld.cy.y - 0.25, 1e-5) or_return
	ENSURE_SMALL(t, invWorld.cz.z - 0.125, 1e-5) or_return
	ENSURE_SMALL(t, invWorld.cy.x, 1e-5) or_return
	ENSURE_SMALL(t, invWorld.cz.x, 1e-5) or_return
	ENSURE_SMALL(t, invWorld.cx.y, 1e-5) or_return
	ENSURE_SMALL(t, invWorld.cz.y, 1e-5) or_return
	ENSURE_SMALL(t, invWorld.cx.z, 1e-5) or_return
	ENSURE_SMALL(t, invWorld.cy.z, 1e-5) or_return

	// World center of mass is the body origin plus the local center under identity rotation.
	worldCenter := b3.Body_GetWorldCenter(bodyId)
	ENSURE_SMALL(t, worldCenter.x - (5.0 + center.x), 1e-5) or_return
	ENSURE_SMALL(t, worldCenter.y - (-3.0 + center.y), 1e-5) or_return
	ENSURE_SMALL(t, worldCenter.z - (2.0 + center.z), 1e-5) or_return

	b3.DestroyWorld(worldId)
	return true
}

// The world inverse inertia must be the local inverse rotated into world space. A 90 degree turn about
// z swaps the x and y principal moments, so diag(1/2, 1/4, 1/8) becomes diag(1/4, 1/2, 1/8). This is the
// regression guard: before SetMassData rotated the tensor it left the world inverse inertia stale.
@(private = "file")
SetMassDataWorldInertiaRotated :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.rotation = b3.MakeQuatFromAxisAngle(b3.Vec3_axisZ, 0.5 * b3.PI)
	bodyId := b3.CreateBody(worldId, &bodyDef)

	massData := b3.MassData{1.0, {0.0, 0.0, 0.0}, kDiagInertia}
	b3.Body_SetMassData(bodyId, massData)

	// The local inertia is stored untouched by the world transform.
	localInertia := b3.Body_GetLocalRotationalInertia(bodyId)
	ENSURE_SMALL(t, localInertia.cx.x - 2.0, 1e-6) or_return
	ENSURE_SMALL(t, localInertia.cy.y - 4.0, 1e-6) or_return
	ENSURE_SMALL(t, localInertia.cz.z - 8.0, 1e-6) or_return

	invWorld := b3.Body_GetWorldInverseRotationalInertia(bodyId)
	ENSURE_SMALL(t, invWorld.cx.x - 0.25, 1e-4) or_return
	ENSURE_SMALL(t, invWorld.cy.y - 0.5, 1e-4) or_return
	ENSURE_SMALL(t, invWorld.cz.z - 0.125, 1e-4) or_return
	ENSURE_SMALL(t, invWorld.cy.x, 1e-4) or_return
	ENSURE_SMALL(t, invWorld.cz.x, 1e-4) or_return
	ENSURE_SMALL(t, invWorld.cx.y, 1e-4) or_return
	ENSURE_SMALL(t, invWorld.cz.y, 1e-4) or_return
	ENSURE_SMALL(t, invWorld.cx.z, 1e-4) or_return
	ENSURE_SMALL(t, invWorld.cy.z, 1e-4) or_return

	b3.DestroyWorld(worldId)
	return true
}

// Fixed rotation must leave the mass intact but drive the whole angular inertia to zero, even when the
// caller hands in a real tensor.
@(private = "file")
SetMassDataFixedRotation :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.motionLocks.angularX = true
	bodyDef.motionLocks.angularY = true
	bodyDef.motionLocks.angularZ = true
	bodyId := b3.CreateBody(worldId, &bodyDef)

	massData := b3.MassData{5.0, {0.0, 0.0, 0.0}, kDiagInertia}
	b3.Body_SetMassData(bodyId, massData)

	ENSURE_SMALL(t, b3.Body_GetMass(bodyId) - 5.0, 1e-6) or_return
	ENSURE_SMALL(t, b3.Body_GetInverseMass(bodyId) - 0.2, 1e-6) or_return

	localInertia := b3.Body_GetLocalRotationalInertia(bodyId)
	ENSURE_SMALL(t, localInertia.cx.x, 1e-6) or_return
	ENSURE_SMALL(t, localInertia.cy.y, 1e-6) or_return
	ENSURE_SMALL(t, localInertia.cz.z, 1e-6) or_return

	invWorld := b3.Body_GetWorldInverseRotationalInertia(bodyId)
	ENSURE_SMALL(t, invWorld.cx.x, 1e-6) or_return
	ENSURE_SMALL(t, invWorld.cy.y, 1e-6) or_return
	ENSURE_SMALL(t, invWorld.cz.z, 1e-6) or_return

	md := b3.Body_GetMassData(bodyId)
	ENSURE_SMALL(t, md.inertia.cx.x, 1e-6) or_return
	ENSURE_SMALL(t, md.inertia.cy.y, 1e-6) or_return
	ENSURE_SMALL(t, md.inertia.cz.z, 1e-6) or_return

	b3.DestroyWorld(worldId)
	return true
}

// Zero mass and a zero tensor have zero determinant, so the inverses must collapse to zero rather than
// divide by it.
@(private = "file")
SetMassDataZeroMass :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyId := b3.CreateBody(worldId, &bodyDef)

	massData := b3.MassData{0.0, {0.0, 0.0, 0.0}, b3.Mat3_zero}
	b3.Body_SetMassData(bodyId, massData)

	ENSURE_SMALL(t, b3.Body_GetInverseMass(bodyId), 1e-6) or_return

	localInertia := b3.Body_GetLocalRotationalInertia(bodyId)
	ENSURE_SMALL(t, localInertia.cx.x, 1e-6) or_return
	ENSURE_SMALL(t, localInertia.cy.y, 1e-6) or_return
	ENSURE_SMALL(t, localInertia.cz.z, 1e-6) or_return

	invWorld := b3.Body_GetWorldInverseRotationalInertia(bodyId)
	ENSURE_SMALL(t, invWorld.cx.x, 1e-6) or_return
	ENSURE_SMALL(t, invWorld.cy.y, 1e-6) or_return
	ENSURE_SMALL(t, invWorld.cz.z, 1e-6) or_return

	b3.DestroyWorld(worldId)
	return true
}

// The stored linear velocity tracks the center of mass. Moving the center picks a different material
// point, so a spinning body must have its velocity re-referenced by omega x (newCenter - oldCenter),
// otherwise the mass edit silently injects or drains kinetic energy. Under identity rotation the world
// shift equals the supplied local center, keeping the expected values exact in float.
@(private = "file")
SetMassDataConsistentVelocity :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = b3.Pos{7.0, 1.0, -4.0}
	bodyId := b3.CreateBody(worldId, &bodyDef)

	// Spin about the origin center, then shift the center of mass off the origin.
	omega := b3.Vec3{1.0, 2.0, 4.0}
	b3.Body_SetLinearVelocity(bodyId, b3.Vec3{1.0, -2.0, 3.0})
	b3.Body_SetAngularVelocity(bodyId, omega)

	center := b3.Vec3{0.5, 0.25, 0.125}
	massData := b3.MassData{3.0, center, kDiagInertia}
	b3.Body_SetMassData(bodyId, massData)

	// omega x center = ( 2*0.125 - 4*0.25, 4*0.5 - 1*0.125, 1*0.25 - 2*0.5 ) = ( -0.75, 1.875, -0.75 )
	v := b3.Body_GetLinearVelocity(bodyId)
	ENSURE_SMALL(t, v.x - (f32(1.0) - f32(0.75)), 1e-6) or_return
	ENSURE_SMALL(t, v.y - (f32(-2.0) + f32(1.875)), 1e-6) or_return
	ENSURE_SMALL(t, v.z - (f32(3.0) - f32(0.75)), 1e-6) or_return

	// Only the reference point moved, the angular velocity is untouched.
	w := b3.Body_GetAngularVelocity(bodyId)
	ENSURE_SMALL(t, w.x - omega.x, 1e-6) or_return
	ENSURE_SMALL(t, w.y - omega.y, 1e-6) or_return
	ENSURE_SMALL(t, w.z - omega.z, 1e-6) or_return

	b3.DestroyWorld(worldId)
	return true
}

// Extents bound the shapes about the center of mass, per axis. An offset shape must count its
// offset, not just its own size.
@(private = "file")
ShapeExtents :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	shapeDef := b3.DefaultShapeDef()
	shapeDef.density = 1.0

	// Kinematic bodies measure from the body origin
	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .kinematicBody

	capsuleId := b3.CreateBody(worldId, &bodyDef)
	capsule := b3.Capsule{{-2.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}, 0.2}
	b3.CreateCapsuleShape(capsuleId, &shapeDef, &capsule)

	maxExtent := b3.Body_GetMaxExtent(capsuleId)
	ENSURE_SMALL(t, maxExtent.x - 2.2, 1e-5) or_return
	ENSURE_SMALL(t, maxExtent.y - 0.2, 1e-5) or_return
	ENSURE_SMALL(t, maxExtent.z - 0.2, 1e-5) or_return
	ENSURE_SMALL(t, b3.Body_GetMinExtent(capsuleId) - 0.2, 1e-5) or_return

	sphereId := b3.CreateBody(worldId, &bodyDef)
	sphere := b3.Sphere{{1.0, 2.0, 3.0}, 0.5}
	b3.CreateSphereShape(sphereId, &shapeDef, &sphere)

	maxExtent = b3.Body_GetMaxExtent(sphereId)
	ENSURE_SMALL(t, maxExtent.x - 1.5, 1e-5) or_return
	ENSURE_SMALL(t, maxExtent.y - 2.5, 1e-5) or_return
	ENSURE_SMALL(t, maxExtent.z - 3.5, 1e-5) or_return
	ENSURE_SMALL(t, b3.Body_GetMinExtent(sphereId) - 0.5, 1e-5) or_return

	// Dynamic bodies measure from the center of mass. A light sphere hung off a cube pulls the
	// center toward it, so the far side of the sphere is the widest point.
	bodyDef.type = .dynamicBody
	cubeId := b3.CreateBody(worldId, &bodyDef)
	cube := b3.MakeCubeHull(0.5)
	b3.CreateHullShape(cubeId, &shapeDef, &cube.base)
	offsetSphere := b3.Sphere{{1.0, 0.0, 0.0}, 0.2}
	b3.CreateSphereShape(cubeId, &shapeDef, &offsetSphere)

	localCenter := b3.Body_GetLocalCenter(cubeId)
	ENSURE(t, 0.0 < localCenter.x && localCenter.x < 0.5) or_return

	maxExtent = b3.Body_GetMaxExtent(cubeId)
	ENSURE_SMALL(t, maxExtent.x - (1.2 - localCenter.x), 1e-5) or_return
	ENSURE_SMALL(t, maxExtent.y - 0.5, 1e-5) or_return
	ENSURE_SMALL(t, maxExtent.z - 0.5, 1e-5) or_return
	ENSURE_SMALL(t, b3.Body_GetMinExtent(cubeId) - 0.2, 1e-5) or_return

	originExtent := b3.Body_GetMaxExtentOrigin(cubeId)
	ENSURE_SMALL(t, originExtent.x - 1.2, 1e-5) or_return
	ENSURE_SMALL(t, originExtent.y - 0.5, 1e-5) or_return
	ENSURE_SMALL(t, originExtent.z - 0.5, 1e-5) or_return

	// A mesh has no mass, so the sphere alone places the center at x = 1 and the far edge of the
	// mesh at x = -3 is four units out.
	vertices := [6]b3.Vec3 {
		{-3.0, 0.0, -1.0},
		{-2.0, 0.0, 1.0},
		{-1.0, 0.0, -1.0},
		{1.0, 0.0, -1.0},
		{2.0, 0.0, 1.0},
		{3.0, 0.0, -1.0},
	}
	indices := [6]i32{0, 1, 2, 3, 4, 5}
	meshDef := b3.MeshDef{}
	meshDef.vertices = &vertices[0]
	meshDef.stride = size_of(b3.Vec3)
	meshDef.indices = &indices[0]
	meshDef.vertexCount = 6
	meshDef.triangleCount = 2
	mesh := b3.CreateMesh(&meshDef, nil, 0)

	meshId := b3.CreateBody(worldId, &bodyDef)
	b3.CreateSphereShape(meshId, &shapeDef, &offsetSphere)
	b3.CreateMeshShape(meshId, &shapeDef, mesh, b3.Vec3{1.0, 1.0, 1.0})

	localCenter = b3.Body_GetLocalCenter(meshId)
	ENSURE_SMALL(t, localCenter.x - 1.0, 1e-5) or_return

	maxExtent = b3.Body_GetMaxExtent(meshId)
	ENSURE_SMALL(t, maxExtent.x - 4.0, 1e-4) or_return
	ENSURE_SMALL(t, maxExtent.y - 0.2, 1e-4) or_return
	ENSURE_SMALL(t, maxExtent.z - 1.0, 1e-4) or_return
	ENSURE_SMALL(t, b3.Body_GetMinExtent(meshId) - 0.2, 1e-5) or_return

	b3.DestroyWorld(worldId)
	b3.DestroyMesh(mesh)
	return true
}

@(private = "file")
CompoundInertiaConsistency :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.rotation = b3.MakeQuatFromAxisAngle(b3.Normalize(b3.Vec3{0.3, -0.8, 0.5}), 1.3)
	bodyId := b3.CreateBody(worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()
	boxA := b3.MakeOffsetBoxHull(0.5, 0.25, 0.5, b3.Vec3{1.0, 1.0, 0.0})
	boxB := b3.MakeOffsetBoxHull(0.3, 0.6, 0.2, b3.Vec3{-0.8, -0.4, 0.3})
	b3.CreateHullShape(bodyId, &shapeDef, &boxA.base)
	b3.CreateHullShape(bodyId, &shapeDef, &boxB.base)
	b3.Body_ApplyMassFromShapes(bodyId)

	local := b3.Body_GetLocalRotationalInertia(bodyId)
	ENSURE(t, b3.AbsFloat(local.cy.x) > 0.1) or_return

	r := b3.MakeMatrixFromQuat(bodyDef.rotation)
	world := b3.MulMM(b3.MulMM(r, local), b3.Transpose(r))
	invWorld := b3.Body_GetWorldInverseRotationalInertia(bodyId)
	product := b3.MulMM(invWorld, world)

	ENSURE_SMALL(t, product.cx.x - 1.0, 1.0e-4) or_return
	ENSURE_SMALL(t, product.cy.y - 1.0, 1.0e-4) or_return
	ENSURE_SMALL(t, product.cz.z - 1.0, 1.0e-4) or_return
	ENSURE_SMALL(t, product.cx.y, 1.0e-4) or_return
	ENSURE_SMALL(t, product.cx.z, 1.0e-4) or_return
	ENSURE_SMALL(t, product.cy.x, 1.0e-4) or_return
	ENSURE_SMALL(t, product.cy.z, 1.0e-4) or_return
	ENSURE_SMALL(t, product.cz.x, 1.0e-4) or_return
	ENSURE_SMALL(t, product.cz.y, 1.0e-4) or_return

	q2 := b3.MakeQuatFromAxisAngle(b3.Normalize(b3.Vec3{-1.0, 0.2, 0.4}), -2.1)
	b3.Body_SetTransform(bodyId, b3.Body_GetPosition(bodyId), q2)
	r = b3.MakeMatrixFromQuat(q2)
	world = b3.MulMM(b3.MulMM(r, local), b3.Transpose(r))
	invWorld = b3.Body_GetWorldInverseRotationalInertia(bodyId)
	product = b3.MulMM(invWorld, world)
	ENSURE_SMALL(t, product.cx.x - 1.0, 1.0e-4) or_return
	ENSURE_SMALL(t, product.cy.y - 1.0, 1.0e-4) or_return
	ENSURE_SMALL(t, product.cz.z - 1.0, 1.0e-4) or_return
	ENSURE_SMALL(t, product.cy.x, 1.0e-4) or_return
	ENSURE_SMALL(t, product.cz.y, 1.0e-4) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
CreateSpinningHullBody :: proc(worldId: b3.WorldId, x: f32, hull: ^b3.HullData) -> b3.BodyId {
	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position.x = x
	bodyDef.rotation = b3.MakeQuatFromAxisAngle(b3.Normalize(b3.Vec3{0.3, -0.8, 0.5}), 1.3)
	bodyDef.angularVelocity = b3.Vec3{1.0, -2.0, 3.0}
	bodyId := b3.CreateBody(worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(bodyId, &shapeDef, hull)
	return bodyId
}

@(private = "file")
HasGyroscopicTorque :: proc(bodyId: b3.BodyId) -> bool {
	world := b3.GetWorld(i32(bodyId.world0))
	body := b3.GetBodyFullId(world, bodyId)
	return (body.flags & b3.enableGyroscopicTorque) != 0
}

// Bodies within the isotropic tolerance skip the gyroscopic solve, so in free flight the world
// angular velocity is untouched. Anisotropic bodies precess.
@(private = "file")
IsotropicInertiaTolerance :: proc(t: ^testing.T) -> bool {
	w0 := b3.Vec3{1.0, -2.0, 3.0}

	rock := b3.CreateRock(0.5)
	complex := b3.CreateComplexHull(0.5)
	cube := b3.MakeCubeHull(0.5)
	cubeTransform := b3.Transform{b3.Vec3_zero, b3.MakeQuatFromAxisAngle(b3.Normalize(b3.Vec3{1.0, 2.0, 3.0}), 0.7)}
	rotatedCube := b3.MakeTransformedBoxHull(0.5, 0.5, 0.5, cubeTransform)
	plank := b3.MakeBoxHull(0.5, 1.0, 0.25)

	{
		worldDef := b3.DefaultWorldDef()
		worldDef.gravity = b3.Vec3_zero
		worldId := b3.CreateWorld(&worldDef)

		rockId := CreateSpinningHullBody(worldId, 0.0, rock)
		complexId := CreateSpinningHullBody(worldId, 5.0, complex)
		cubeId := CreateSpinningHullBody(worldId, 10.0, &cube.base)
		rotatedCubeId := CreateSpinningHullBody(worldId, 15.0, &rotatedCube.base)
		plankId := CreateSpinningHullBody(worldId, 20.0, &plank.base)

		ENSURE(t, HasGyroscopicTorque(rockId) == false) or_return
		ENSURE(t, HasGyroscopicTorque(complexId) == false) or_return
		ENSURE(t, HasGyroscopicTorque(cubeId) == false) or_return
		ENSURE(t, HasGyroscopicTorque(rotatedCubeId) == false) or_return
		ENSURE(t, HasGyroscopicTorque(plankId)) or_return

		for i := 0; i < 10; i += 1 {
			b3.World_Step(worldId, f32(1.0) / f32(60.0), 4)
		}

		w := b3.Body_GetAngularVelocity(rockId)
		ENSURE(t, w.x == w0.x && w.y == w0.y && w.z == w0.z) or_return
		w = b3.Body_GetAngularVelocity(complexId)
		ENSURE(t, w.x == w0.x && w.y == w0.y && w.z == w0.z) or_return
		w = b3.Body_GetAngularVelocity(rotatedCubeId)
		ENSURE(t, w.x == w0.x && w.y == w0.y && w.z == w0.z) or_return
		w = b3.Body_GetAngularVelocity(plankId)
		ENSURE(t, b3.Length(b3.Sub(w, w0)) > 0.01) or_return

		b3.Body_SetMotionLocks(plankId, b3.MotionLocks{angularX = true, angularY = true, angularZ = true})
		ENSURE(t, HasGyroscopicTorque(plankId) == false) or_return

		// Gyroscopic torque stays off until a mass update proves the body anisotropic
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position.x = 25.0
		bodyDef.angularVelocity = w0
		emptyId := b3.CreateBody(worldId, &bodyDef)
		ENSURE(t, HasGyroscopicTorque(emptyId) == false) or_return

		bodyDef.position.x = 30.0
		deferredId := b3.CreateBody(worldId, &bodyDef)
		shapeDef := b3.DefaultShapeDef()
		shapeDef.updateBodyMass = false
		b3.CreateHullShape(deferredId, &shapeDef, &plank.base)
		ENSURE(t, HasGyroscopicTorque(deferredId) == false) or_return

		b3.Body_ApplyMassFromShapes(deferredId)
		ENSURE(t, HasGyroscopicTorque(deferredId)) or_return

		b3.DestroyWorld(worldId)
	}

	{
		worldDef := b3.DefaultWorldDef()
		worldDef.gravity = b3.Vec3_zero
		worldDef.isotropicInertiaTolerance = 0.0
		worldId := b3.CreateWorld(&worldDef)

		rockId := CreateSpinningHullBody(worldId, 0.0, rock)
		cubeId := CreateSpinningHullBody(worldId, 10.0, &cube.base)

		ENSURE(t, HasGyroscopicTorque(rockId)) or_return
		ENSURE(t, HasGyroscopicTorque(cubeId) == false) or_return

		for i := 0; i < 10; i += 1 {
			b3.World_Step(worldId, f32(1.0) / f32(60.0), 4)
		}

		w := b3.Body_GetAngularVelocity(rockId)
		ENSURE(t, b3.Length(b3.Sub(w, w0)) > 0.001) or_return

		b3.DestroyWorld(worldId)
	}

	b3.DestroyHull(rock)
	b3.DestroyHull(complex)
	return true
}

@(test)
BodyTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, FarSingleSphereMass) {return}
	if !RUN_SUBTEST(t, FarCubeSphereMass) {return}
	if !RUN_SUBTEST(t, DeferredMassExtents) {return}
	if !RUN_SUBTEST(t, SetMassDataRoundTrip) {return}
	if !RUN_SUBTEST(t, SetMassDataWorldInertiaRotated) {return}
	if !RUN_SUBTEST(t, SetMassDataFixedRotation) {return}
	if !RUN_SUBTEST(t, SetMassDataZeroMass) {return}
	if !RUN_SUBTEST(t, SetMassDataConsistentVelocity) {return}
	if !RUN_SUBTEST(t, ShapeExtents) {return}
	if !RUN_SUBTEST(t, CompoundInertiaConsistency) {return}
	if !RUN_SUBTEST(t, IsotropicInertiaTolerance) {return}
}
