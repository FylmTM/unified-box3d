// Ported from box3d test/test_mover.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import "base:runtime"
import "core:testing"

import b3 "../../box3d"

@(private = "file")
ParallelPlanes :: proc(t: ^testing.T) -> bool {
	planes: [3]b3.CollisionPlane = {}
	planes[0].plane.normal = b3.Vec3{0.0, 0.0, 1.0}
	planes[0].plane.offset = 0.5
	planes[0].pushLimit = b3.FLT_MAX
	planes[1].plane.normal = b3.Vec3{0.0, 0.0, 1.0}
	planes[1].plane.offset = 1.0
	planes[1].pushLimit = b3.FLT_MAX
	// planes[2].plane.normal = b3Normalize((b3Vec3){ 0.2f, 0.0f, 0.9f });
	// planes[2].plane.offset = 0.25f;
	// planes[2].pushLimit = FLT_MAX;

	target := b3.Vec3{0.0, 0.0, 0.0}
	result := b3.SolvePlanes(target, &planes[0], 2)

	ENSURE(t, result.iterationCount == 2) or_return
	ENSURE_SMALL(t, result.delta.z - 1.0, 0.0055) or_return

	return true
}

@(private = "file")
GamePlanes :: proc(t: ^testing.T) -> bool {
	// This scenario takes many iterations because the target is deep into the plane.
	planes: [3]b3.CollisionPlane = {}
	planes[0].plane.normal = b3.Vec3{0.0, -0.23941046, 0.970918416}
	planes[0].plane.offset = 0.390724182
	planes[0].pushLimit = b3.FLT_MAX
	planes[1].plane.normal = b3.Vec3{0.0, 0.0, 1.0}
	planes[1].plane.offset = 1.49998093
	planes[1].pushLimit = b3.FLT_MAX

	target := b3.Vec3{-2.5390625, 0.0, -73.6880798}

	planes[0].plane.offset -= b3.Dot(planes[0].plane.normal, target)
	planes[1].plane.offset -= b3.Dot(planes[1].plane.normal, target)
	target = b3.Vec3_zero

	result := b3.SolvePlanes(target, &planes[0], 2)

	ENSURE(t, result.iterationCount == 20) or_return

	return true
}

// Mover-collide overlap handling
// b3CollideMoverAndSphere / Capsule / Hull must never emit a plane with a
// degenerate (zero) normal, even when the mover deeply penetrates the shape.
// On deep overlap the GJK path returns a {0,0,0} normal; these tests guard the
// fix that replaces it with an analytic (sphere/capsule) or dropped (hull) result.
@(private = "file")
MoverSphereSeparated :: proc(t: ^testing.T) -> bool {
	shape := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}
	mover := b3.Capsule{{4.0, 3.0, 0.0}, {6.0, 3.0, 0.0}, 0.2}

	result: b3.PlaneResult = {}
	count := b3.CollideMoverAndSphere(&result, &shape, &mover)
	ENSURE(t, count == 0) or_return

	return true
}

@(private = "file")
MoverSphereTouching :: proc(t: ^testing.T) -> bool {
	shape := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}

	// Mover core segment runs along X at y = 0.6, leaving it 0.1 inside the
	// 0.7 combined radius.
	mover := b3.Capsule{{-1.0, 0.6, 0.0}, {1.0, 0.6, 0.0}, 0.2}

	result: b3.PlaneResult = {}
	count := b3.CollideMoverAndSphere(&result, &shape, &mover)
	ENSURE(t, count == 1) or_return
	ENSURE(t, b3.IsNormalized(result.plane.normal)) or_return

	// Push-out points from the sphere straight up toward the mover.
	ENSURE(t, result.plane.normal.y > 0.99) or_return
	ENSURE_SMALL(t, result.plane.offset - 0.1, 1e-5) or_return

	return true
}

@(private = "file")
MoverSphereDeepOverlap :: proc(t: ^testing.T) -> bool {
	shape := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}

	// Mover axis runs straight through the sphere center: the bug case where
	// GJK reports a zero normal.
	mover := b3.Capsule{{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.2}

	result: b3.PlaneResult = {}
	count := b3.CollideMoverAndSphere(&result, &shape, &mover)
	ENSURE(t, count == 1) or_return

	// The normal must still be a valid unit vector.
	ENSURE(t, b3.IsNormalized(result.plane.normal)) or_return

	// The fallback axis is perpendicular to the mover axis (X).
	ENSURE_SMALL(t, result.plane.normal.x, 1e-5) or_return

	// Deepest possible penetration: the full combined radius.
	ENSURE_SMALL(t, result.plane.offset - 0.7, 1e-5) or_return

	return true
}

@(private = "file")
MoverCapsuleSeparated :: proc(t: ^testing.T) -> bool {
	shape := b3.Capsule{{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.3}
	mover := b3.Capsule{{-1.0, 5.0, 0.0}, {1.0, 5.0, 0.0}, 0.2}

	result: b3.PlaneResult = {}
	count := b3.CollideMoverAndCapsule(&result, &shape, &mover)
	ENSURE(t, count == 0) or_return

	return true
}

@(private = "file")
MoverCapsuleTouching :: proc(t: ^testing.T) -> bool {
	shape := b3.Capsule{{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.3}

	// Parallel mover 0.4 above, leaving it 0.1 inside the 0.5 combined radius.
	mover := b3.Capsule{{-1.0, 0.4, 0.0}, {1.0, 0.4, 0.0}, 0.2}

	result: b3.PlaneResult = {}
	count := b3.CollideMoverAndCapsule(&result, &shape, &mover)
	ENSURE(t, count == 1) or_return
	ENSURE(t, b3.IsNormalized(result.plane.normal)) or_return
	ENSURE(t, result.plane.normal.y > 0.99) or_return
	ENSURE_SMALL(t, result.plane.offset - 0.1, 1e-5) or_return

	return true
}

@(private = "file")
MoverCapsuleDeepOverlap :: proc(t: ^testing.T) -> bool {
	// Shape capsule along X, mover capsule along Z; their core segments cross
	// exactly at the origin, so GJK reports a zero normal.
	shape := b3.Capsule{{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.3}
	mover := b3.Capsule{{0.0, 0.0, -1.0}, {0.0, 0.0, 1.0}, 0.2}

	result: b3.PlaneResult = {}
	count := b3.CollideMoverAndCapsule(&result, &shape, &mover)
	ENSURE(t, count == 1) or_return
	ENSURE(t, b3.IsNormalized(result.plane.normal)) or_return

	// The separating axis of two crossing segments is perpendicular to both.
	ENSURE_SMALL(t, result.plane.normal.x, 1e-5) or_return
	ENSURE_SMALL(t, result.plane.normal.z, 1e-5) or_return
	ENSURE_SMALL(t, result.plane.offset - 0.5, 1e-5) or_return

	return true
}

@(private = "file")
MoverCapsuleParallelOverlap :: proc(t: ^testing.T) -> bool {
	// Mover core segment coincides with the shape core segment: the cross-product
	// axis degenerates, so a perpendicular of the mover axis is used instead.
	shape := b3.Capsule{{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.3}
	mover := b3.Capsule{{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.2}

	result: b3.PlaneResult = {}
	count := b3.CollideMoverAndCapsule(&result, &shape, &mover)
	ENSURE(t, count == 1) or_return
	ENSURE(t, b3.IsNormalized(result.plane.normal)) or_return

	// The fallback axis is perpendicular to the mover axis (X).
	ENSURE_SMALL(t, result.plane.normal.x, 1e-5) or_return
	ENSURE_SMALL(t, result.plane.offset - 0.5, 1e-5) or_return

	return true
}

@(private = "file")
MoverHullSeparated :: proc(t: ^testing.T) -> bool {
	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	mover := b3.Capsule{{-0.3, 5.0, 0.0}, {0.3, 5.0, 0.0}, 0.2}

	result: b3.PlaneResult = {}
	count := b3.CollideMoverAndHull(&result, &box.base, &mover)
	ENSURE(t, count == 0) or_return

	return true
}

@(private = "file")
MoverHullTouching :: proc(t: ^testing.T) -> bool {
	box := b3.MakeBoxHull(0.5, 0.5, 0.5)

	// Mover core segment above the +Y face; the 0.2 radius reaches 0.1 into it.
	mover := b3.Capsule{{-0.3, 0.6, 0.0}, {0.3, 0.6, 0.0}, 0.2}

	result: b3.PlaneResult = {}
	count := b3.CollideMoverAndHull(&result, &box.base, &mover)
	ENSURE(t, count == 1) or_return
	ENSURE(t, b3.IsNormalized(result.plane.normal)) or_return
	ENSURE(t, result.plane.normal.y > 0.99) or_return
	ENSURE_SMALL(t, result.plane.offset - 0.1, 1e-4) or_return

	return true
}

@(private = "file")
MoverHullDeepOverlap :: proc(t: ^testing.T) -> bool {
	box := b3.MakeBoxHull(0.5, 0.5, 0.5)

	// Mover core segment lies entirely inside the box, so GJK reports overlap.
	mover := b3.Capsule{{-0.2, 0.0, 0.0}, {0.2, 0.0, 0.0}, 0.1}

	result: b3.PlaneResult = {}
	count := b3.CollideMoverAndHull(&result, &box.base, &mover)

	// The overlap guard drops the plane rather than emit a zero normal.
	// todo replace with SAT once b3CollideMoverAndHull resolves overlaps.
	ENSURE(t, count == 0) or_return

	return true
}

// Mover queries report which material a contact plane came from
// b3PlaneResult::materialIndex follows different paths per shape type. Meshes
// report the per triangle material index. Compounds remap the child result
// through the child material table. Convex shapes report index 0.
@(private = "file")
MakeMaterial :: proc(friction: f32, userId: u64) -> b3.SurfaceMaterial {
	m := b3.DefaultSurfaceMaterial()
	m.friction = friction
	m.userMaterialId = userId
	return m
}

// Two separated upward facing triangles on the y = 0 plane, one per material. The bake may
// reorder triangles but always keeps a triangle paired with its material index.
@(private = "file")
MakeTwoMaterialMesh :: proc() -> ^b3.MeshData {
	vertices := [6]b3.Vec3 {
		{-3.0, 0.0, -1.0},
		{-2.0, 0.0, 1.0},
		{-1.0, 0.0, -1.0},
		{1.0, 0.0, -1.0},
		{2.0, 0.0, 1.0},
		{3.0, 0.0, -1.0},
	}
	indices := [6]i32{0, 1, 2, 3, 4, 5}
	materialIndices := [2]u8{0, 1}

	def: b3.MeshDef = {}
	def.vertices = &vertices[0]
	def.stride = size_of(b3.Vec3)
	def.indices = &indices[0]
	def.materialIndices = &materialIndices[0]
	def.vertexCount = 6
	def.triangleCount = 2

	return b3.CreateMesh(&def, nil, 0)
}

// Flat 3x3 vertex field at y = 0, one cell material per entry. Caller destroys.
@(private = "file")
MakeFlatField :: proc(materials: [^]u8, clockwise: bool) -> ^b3.HeightFieldData {
	heights: [9]f32 = {}

	def: b3.HeightFieldDef = {}
	def.heights = &heights[0]
	def.materialIndices = materials
	def.scale = b3.Vec3{1.0, 1.0, 1.0}
	def.countX = 3
	def.countZ = 3
	def.globalMinimumHeight = -1.0
	def.globalMaximumHeight = 1.0
	def.clockwiseWinding = clockwise

	return b3.CreateHeightField(&def)
}

@(private = "file")
PlaneCapture :: struct {
	planes: [16]b3.PlaneResult,
	count:  i32,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
CapturePlaneFcn :: proc "c" (shapeId: b3.ShapeId, planes: ^b3.PlaneResult, planeCount: i32, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	_ = shapeId
	capture := (^PlaneCapture)(context_)
	planes := ([^]b3.PlaneResult)(planes)
	for i: i32 = 0; i < planeCount && capture.count < 16; i += 1 {
		capture.planes[capture.count] = planes[i]
		capture.count += 1
	}
	return true
}

@(private = "file")
MoverWorldMeshMaterials :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .staticBody
	bodyId := b3.CreateBody(worldId, &bodyDef)

	mesh := MakeTwoMaterialMesh()
	ENSURE(t, mesh != nil) or_return
	ENSURE(t, mesh.materialCount == 2) or_return

	materials := [2]b3.SurfaceMaterial{MakeMaterial(0.2, 1), MakeMaterial(0.8, 2)}
	shapeDef := b3.DefaultShapeDef()
	shapeDef.materials = &materials[0]
	shapeDef.materialCount = 2
	b3.CreateMeshShape(bodyId, &shapeDef, mesh, b3.Vec3{1.0, 1.0, 1.0})

	b3.World_Step(worldId, 1.0 / 60.0, 1)

	bakedMaterialIndices := b3.GetMeshMaterialIndices(mesh)

	// Mover hanging just above the first triangle so its radius reaches 0.05 into the surface
	mover := b3.Capsule{{-2.0, 0.15, 0.0}, {-2.0, 0.35, 0.0}, 0.2}

	capture: PlaneCapture = {}
	b3.World_CollideMover(worldId, b3.Pos_zero, &mover, b3.DefaultQueryFilter(), CapturePlaneFcn, &capture)

	ENSURE(t, capture.count == 1) or_return
	ENSURE(t, capture.planes[0].plane.normal.y > 0.99) or_return
	ENSURE_SMALL(t, capture.planes[0].plane.offset - 0.05, 1e-4) or_return
	ENSURE_SMALL(t, capture.planes[0].point.y, 1e-4) or_return
	ENSURE(t, capture.planes[0].materialIndex == 0) or_return
	ENSURE(t, capture.planes[0].triangleIndex >= 0) or_return
	ENSURE(t, capture.planes[0].triangleIndex < mesh.triangleCount) or_return
	ENSURE(t, bakedMaterialIndices[capture.planes[0].triangleIndex] == 0) or_return

	// Same mover over the second triangle
	mover2 := b3.Capsule{{2.0, 0.15, 0.0}, {2.0, 0.35, 0.0}, 0.2}

	capture2: PlaneCapture = {}
	b3.World_CollideMover(worldId, b3.Pos_zero, &mover2, b3.DefaultQueryFilter(), CapturePlaneFcn, &capture2)

	ENSURE(t, capture2.count == 1) or_return
	ENSURE(t, capture2.planes[0].plane.normal.y > 0.99) or_return
	ENSURE(t, capture2.planes[0].materialIndex == 1) or_return
	ENSURE(t, bakedMaterialIndices[capture2.planes[0].triangleIndex] == 1) or_return

	b3.DestroyWorld(worldId)
	b3.DestroyMesh(mesh)
	return true
}

@(private = "file")
MoverWorldCompoundMeshMaterials :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .staticBody
	bodyId := b3.CreateBody(worldId, &bodyDef)

	mesh := MakeTwoMaterialMesh()

	// The hull material and the two mesh materials are distinct, so the compound material
	// table gets one slot per material. Children bake in hull then mesh order.
	meshMaterials := [2]b3.SurfaceMaterial{MakeMaterial(0.3, 101), MakeMaterial(0.6, 202)}
	hullMaterial := MakeMaterial(0.9, 303)

	meshChild := b3.CompoundMeshDef {
		meshData      = mesh,
		transform     = b3.Transform_identity,
		scale         = {1.0, 1.0, 1.0},
		materials     = &meshMaterials[0],
		materialCount = 2,
	}

	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	hullChild := b3.CompoundHullDef {
		hull = &box.base,
		transform = {p = {-8.0, 0.0, 0.0}, q = b3.Quat_identity},
		material = hullMaterial,
	}

	compoundDef := b3.CompoundDef {
		hulls     = &hullChild,
		hullCount = 1,
		meshes    = &meshChild,
		meshCount = 1,
	}
	compound := b3.CreateCompound(&compoundDef)
	ENSURE(t, compound != nil) or_return
	ENSURE(t, compound.materialCount == 3) or_return

	bakedMaterials := b3.GetCompoundMaterials(compound)
	ENSURE(t, bakedMaterials[0].userMaterialId == 303) or_return
	ENSURE(t, bakedMaterials[1].userMaterialId == 101) or_return
	ENSURE(t, bakedMaterials[2].userMaterialId == 202) or_return

	shapeDef := b3.DefaultShapeDef()
	b3.CreateBakedCompoundShape(bodyId, &shapeDef, compound)

	b3.World_Step(worldId, 1.0 / 60.0, 1)

	// Mover on top of the hull child face at y = 0.5
	hullMover := b3.Capsule{{-8.0, 0.65, 0.0}, {-8.0, 0.85, 0.0}, 0.2}

	hullCapture: PlaneCapture = {}
	b3.World_CollideMover(worldId, b3.Pos_zero, &hullMover, b3.DefaultQueryFilter(), CapturePlaneFcn, &hullCapture)

	ENSURE(t, hullCapture.count == 1) or_return
	ENSURE(t, hullCapture.planes[0].plane.normal.y > 0.99) or_return
	ENSURE(t, hullCapture.planes[0].childIndex == 0) or_return
	ENSURE(t, hullCapture.planes[0].materialIndex == 0) or_return
	ENSURE(t, bakedMaterials[hullCapture.planes[0].materialIndex].userMaterialId == 303) or_return

	// Mover over the second mesh triangle. The mesh reports triangle material 1, which the
	// compound remaps through the child material table to the shared slot of meshMaterials[1].
	meshMover := b3.Capsule{{2.0, 0.15, 0.0}, {2.0, 0.35, 0.0}, 0.2}

	meshCapture: PlaneCapture = {}
	b3.World_CollideMover(worldId, b3.Pos_zero, &meshMover, b3.DefaultQueryFilter(), CapturePlaneFcn, &meshCapture)

	ENSURE(t, meshCapture.count == 1) or_return
	ENSURE(t, meshCapture.planes[0].plane.normal.y > 0.99) or_return
	ENSURE(t, meshCapture.planes[0].childIndex == 1) or_return
	ENSURE(t, meshCapture.planes[0].materialIndex == 2) or_return
	ENSURE(t, bakedMaterials[meshCapture.planes[0].materialIndex].userMaterialId == 202) or_return

	b3.DestroyWorld(worldId)
	b3.DestroyCompound(compound)
	b3.DestroyMesh(mesh)
	return true
}

@(private = "file")
MoverBodyMaterialIndices :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyId := b3.CreateBody(worldId, &bodyDef)

	// One shape of each convex type, spaced out along X
	shapeDef := b3.DefaultShapeDef()

	sphere := b3.Sphere{{0.0, 0.5, 0.0}, 0.5}
	b3.CreateSphereShape(bodyId, &shapeDef, &sphere)

	box := b3.MakeTransformedBoxHull(0.5, 0.5, 0.5, b3.Transform{p = {5.0, 0.0, 0.0}, q = b3.Quat_identity})
	b3.CreateHullShape(bodyId, &shapeDef, &box.base)

	capsule := b3.Capsule{{9.0, 0.0, 0.0}, {11.0, 0.0, 0.0}, 0.3}
	b3.CreateCapsuleShape(bodyId, &shapeDef, &capsule)

	bodyTransform := b3.WorldTransform {
		p = b3.Pos_zero,
		q = b3.Quat_identity,
	}

	// Convex shapes report the base material as index 0
	movers := [3]b3.Capsule {
		{{0.0, 1.15, 0.0}, {0.0, 1.35, 0.0}, 0.2},
		{{5.0, 0.65, 0.0}, {5.0, 0.85, 0.0}, 0.2},
		{{10.0, 0.45, 0.0}, {10.0, 0.65, 0.0}, 0.2},
	}

	for i: i32 = 0; i < 3; i += 1 {
		planes: [4]b3.BodyPlaneResult
		count := b3.Body_CollideMover(
			bodyId,
			&planes[0],
			4,
			b3.Pos_zero,
			&movers[i],
			b3.DefaultQueryFilter(),
			bodyTransform,
		)

		ENSURE(t, count == 1) or_return
		ENSURE(t, b3.Shape_IsValid(planes[0].shapeId)) or_return
		ENSURE(t, planes[0].result.plane.normal.y > 0.99) or_return
		ENSURE(t, planes[0].result.materialIndex == 0) or_return
		ENSURE(t, planes[0].result.childIndex == 0) or_return
	}

	b3.DestroyWorld(worldId)
	return true
}

@(private = "file")
MoverBodySkipsMeshAndCompound :: proc(t: ^testing.T) -> bool {
	// The body level mover query handles convex shapes only. Mesh and compound shapes are
	// skipped, so their material indices are only available through b3World_CollideMover.
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyId := b3.CreateBody(worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()

	mesh := MakeTwoMaterialMesh()
	ENSURE(t, mesh != nil) or_return
	b3.CreateMeshShape(bodyId, &shapeDef, mesh, b3.Vec3{1.0, 1.0, 1.0})

	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	hullChild := b3.CompoundHullDef {
		hull = &box.base,
		transform = {p = {10.0, 0.0, 0.0}, q = b3.Quat_identity},
	}
	compoundDef := b3.CompoundDef {
		hulls     = &hullChild,
		hullCount = 1,
	}
	compound := b3.CreateCompound(&compoundDef)
	ENSURE(t, compound != nil) or_return
	b3.CreateBakedCompoundShape(bodyId, &shapeDef, compound)

	bodyTransform := b3.WorldTransform {
		p = b3.Pos_zero,
		q = b3.Quat_identity,
	}

	// Mover over the mesh triangle and mover over the compound hull child both find nothing
	meshMover := b3.Capsule{{-2.0, 0.15, 0.0}, {-2.0, 0.35, 0.0}, 0.2}
	compoundMover := b3.Capsule{{10.0, 0.65, 0.0}, {10.0, 0.85, 0.0}, 0.2}

	movers := [2]b3.Capsule{meshMover, compoundMover}
	for i: i32 = 0; i < 2; i += 1 {
		planes: [4]b3.BodyPlaneResult
		count := b3.Body_CollideMover(
			bodyId,
			&planes[0],
			4,
			b3.Pos_zero,
			&movers[i],
			b3.DefaultQueryFilter(),
			bodyTransform,
		)
		ENSURE(t, count == 0) or_return
	}

	b3.DestroyWorld(worldId)
	b3.DestroyCompound(compound)
	b3.DestroyMesh(mesh)
	return true
}

// One sided mover collision.
// Mover queries keep only triangles facing the mover. The front side follows the
// baked winding: up for a default mesh or height field, down when the height
// field carries clockwiseWinding. A mirror in the mesh scale must not flip it.
@(private = "file")
MoverMeshBackside :: proc(t: ^testing.T) -> bool {
	mesh := MakeTwoMaterialMesh()
	shape := b3.Mesh {
		data  = mesh,
		scale = {1.0, 1.0, 1.0},
	}

	planes: [4]b3.PlaneResult

	// On the front of the left triangle, so a plane comes back
	above := b3.Capsule{{-2.0, 0.15, 0.0}, {-2.0, 0.35, 0.0}, 0.2}
	count := b3.CollideMoverAndMesh(&planes[0], 4, &shape, &above)
	ENSURE(t, count == 1) or_return
	ENSURE(t, planes[0].plane.normal.y > 0.99) or_return
	ENSURE(t, planes[0].childIndex == 0) or_return
	ENSURE(t, planes[0].triangleIndex >= 0 && planes[0].triangleIndex < mesh.triangleCount) or_return

	// The same spot seen from behind the face is culled
	below := b3.Capsule{{-2.0, -0.35, 0.0}, {-2.0, -0.15, 0.0}, 0.2}
	count = b3.CollideMoverAndMesh(&planes[0], 4, &shape, &below)
	ENSURE(t, count == 0) or_return

	b3.DestroyMesh(mesh)
	return true
}

@(private = "file")
MoverMeshMirroredScale :: proc(t: ^testing.T) -> bool {
	mesh := MakeTwoMaterialMesh()

	// Reflecting the scale flips the triangle winding, the collision swaps it back,
	// so the front stays up. Local x = -2 maps onto the source right triangle, material 1.
	shape := b3.Mesh {
		data  = mesh,
		scale = {-1.0, 1.0, 1.0},
	}

	planes: [4]b3.PlaneResult

	above := b3.Capsule{{-2.0, 0.15, 0.0}, {-2.0, 0.35, 0.0}, 0.2}
	count := b3.CollideMoverAndMesh(&planes[0], 4, &shape, &above)
	ENSURE(t, count == 1) or_return
	ENSURE(t, planes[0].plane.normal.y > 0.99) or_return
	ENSURE(t, planes[0].materialIndex == 1) or_return
	ENSURE(t, b3.GetMeshMaterialIndices(mesh)[planes[0].triangleIndex] == 1) or_return

	below := b3.Capsule{{-2.0, -0.35, 0.0}, {-2.0, -0.15, 0.0}, 0.2}
	count = b3.CollideMoverAndMesh(&planes[0], 4, &shape, &below)
	ENSURE(t, count == 0) or_return

	b3.DestroyMesh(mesh)
	return true
}

@(private = "file")
MoverHeightFieldBackside :: proc(t: ^testing.T) -> bool {
	materials := [4]u8{0, 0, 0, 0}
	hf := MakeFlatField(&materials[0], false)

	planes: [4]b3.PlaneResult

	// Standing on the front (upper) face of the default winding
	above := b3.Capsule{{0.3, 0.15, 0.25}, {0.3, 0.35, 0.25}, 0.2}
	count := b3.CollideMoverAndHeightField(&planes[0], 4, hf, &above)
	ENSURE(t, count == 1) or_return
	ENSURE(t, planes[0].plane.normal.y > 0.99) or_return
	ENSURE_SMALL(t, planes[0].plane.offset - 0.05, 1e-4) or_return

	// Under the surface is the back side and gets culled
	below := b3.Capsule{{0.3, -0.35, 0.25}, {0.3, -0.15, 0.25}, 0.2}
	count = b3.CollideMoverAndHeightField(&planes[0], 4, hf, &below)
	ENSURE(t, count == 0) or_return

	b3.DestroyHeightField(hf)
	return true
}

@(private = "file")
MoverHeightFieldReport :: proc(t: ^testing.T) -> bool {
	materials := [4]u8{1, 2, 0, 0}
	hf := MakeFlatField(&materials[0], false)

	planes: [4]b3.PlaneResult

	// (0.3, 0.25) sits on the x + z <= 1 side of cell (0,0), which holds triangle 0
	first := b3.Capsule{{0.3, 0.15, 0.25}, {0.3, 0.35, 0.25}, 0.2}
	count := b3.CollideMoverAndHeightField(&planes[0], 4, hf, &first)
	ENSURE(t, count == 1) or_return
	ENSURE(t, planes[0].triangleIndex == 0) or_return
	ENSURE(t, planes[0].childIndex == 0) or_return
	ENSURE(t, planes[0].materialIndex == 1) or_return

	// (1.3, 0.3) sits on the x + z <= 2 side of cell (0,1), which holds triangle 2
	second := b3.Capsule{{1.3, 0.15, 0.3}, {1.3, 0.35, 0.3}, 0.2}
	count = b3.CollideMoverAndHeightField(&planes[0], 4, hf, &second)
	ENSURE(t, count == 1) or_return
	ENSURE(t, planes[0].triangleIndex == 2) or_return
	ENSURE(t, planes[0].materialIndex == 2) or_return

	b3.DestroyHeightField(hf)
	return true
}

// A clockwise height field faces down, see HeightFieldWinding. Backside culling
// must respect the flag: below the surface is the front side, above is the back.
@(private = "file")
MoverHeightFieldClockwise :: proc(t: ^testing.T) -> bool {
	materials := [4]u8{0, 0, 0, 0}
	hf := MakeFlatField(&materials[0], true)

	planes: [4]b3.PlaneResult

	below := b3.Capsule{{0.3, -0.35, 0.25}, {0.3, -0.15, 0.25}, 0.2}
	count := b3.CollideMoverAndHeightField(&planes[0], 4, hf, &below)
	ENSURE(t, count == 1) or_return
	ENSURE(t, planes[0].plane.normal.y < -0.99) or_return
	ENSURE_SMALL(t, planes[0].plane.offset - 0.05, 1e-4) or_return
	ENSURE(t, planes[0].triangleIndex == 0 || planes[0].triangleIndex == 1) or_return

	above := b3.Capsule{{0.3, 0.15, 0.25}, {0.3, 0.35, 0.25}, 0.2}
	count = b3.CollideMoverAndHeightField(&planes[0], 4, hf, &above)
	ENSURE(t, count == 0) or_return

	b3.DestroyHeightField(hf)
	return true
}

// A mesh can be baked with more materials than the shape carries. The reported index must stay
// inside the shape's material array.
@(private = "file")
MoverWorldMeshMaterialClamp :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .staticBody
	bodyId := b3.CreateBody(worldId, &bodyDef)

	mesh := MakeTwoMaterialMesh()
	ENSURE(t, mesh.materialCount == 2) or_return

	// Base material only
	shapeDef := b3.DefaultShapeDef()
	shapeId := b3.CreateMeshShape(bodyId, &shapeDef, mesh, b3.Vec3{1.0, 1.0, 1.0})
	shapeMaterialCount := b3.Shape_GetMeshMaterialCount(shapeId)
	ENSURE(t, shapeMaterialCount == 1) or_return

	b3.World_Step(worldId, 1.0 / 60.0, 1)

	// Over the triangle baked with material 1
	mover := b3.Capsule{{2.0, 0.15, 0.0}, {2.0, 0.35, 0.0}, 0.2}
	capture: PlaneCapture = {}
	b3.World_CollideMover(worldId, b3.Pos_zero, &mover, b3.DefaultQueryFilter(), CapturePlaneFcn, &capture)

	ENSURE(t, capture.count == 1) or_return
	ENSURE(t, capture.planes[0].materialIndex == shapeMaterialCount - 1) or_return

	b3.DestroyWorld(worldId)
	b3.DestroyMesh(mesh)
	return true
}

@(test)
MoverTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, GamePlanes) {return}
	if !RUN_SUBTEST(t, ParallelPlanes) {return}

	if !RUN_SUBTEST(t, MoverSphereSeparated) {return}
	if !RUN_SUBTEST(t, MoverSphereTouching) {return}
	if !RUN_SUBTEST(t, MoverSphereDeepOverlap) {return}

	if !RUN_SUBTEST(t, MoverCapsuleSeparated) {return}
	if !RUN_SUBTEST(t, MoverCapsuleTouching) {return}
	if !RUN_SUBTEST(t, MoverCapsuleDeepOverlap) {return}
	if !RUN_SUBTEST(t, MoverCapsuleParallelOverlap) {return}

	if !RUN_SUBTEST(t, MoverHullSeparated) {return}
	if !RUN_SUBTEST(t, MoverHullTouching) {return}
	if !RUN_SUBTEST(t, MoverHullDeepOverlap) {return}

	if !RUN_SUBTEST(t, MoverWorldMeshMaterials) {return}
	if !RUN_SUBTEST(t, MoverWorldCompoundMeshMaterials) {return}
	if !RUN_SUBTEST(t, MoverBodyMaterialIndices) {return}
	if !RUN_SUBTEST(t, MoverBodySkipsMeshAndCompound) {return}
	if !RUN_SUBTEST(t, MoverMeshBackside) {return}
	if !RUN_SUBTEST(t, MoverMeshMirroredScale) {return}
	if !RUN_SUBTEST(t, MoverHeightFieldBackside) {return}
	if !RUN_SUBTEST(t, MoverHeightFieldReport) {return}
	if !RUN_SUBTEST(t, MoverHeightFieldClockwise) {return}
	if !RUN_SUBTEST(t, MoverWorldMeshMaterialClamp) {return}
}
