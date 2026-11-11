// Ported from box3d test/test_compound.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "base:runtime"
import "core:mem"
import "core:testing"

// CollideMoverAndCompound, GetCompoundChild, QueryCompound are internal

@(private = "file")
MakeMaterial :: proc(friction: f32, userId: u64) -> b3.SurfaceMaterial {
	m := b3.DefaultSurfaceMaterial()
	m.friction = friction
	m.userMaterialId = userId
	return m
}

@(private = "file")
CompoundCreateMixed :: proc(t: ^testing.T) -> bool {
	mat := b3.DefaultSurfaceMaterial()

	capsules := [1]b3.CompoundCapsuleDef{{capsule = {{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.25}, material = mat}}

	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	hulls := [1]b3.CompoundHullDef{{hull = &box.base, transform = b3.Transform_identity, material = mat}}

	meshData := b3.CreateBoxMesh(b3.Vec3_zero, b3.Vec3{0.5, 0.5, 0.5}, false)
	meshes := [1]b3.CompoundMeshDef {
		{
			meshData = meshData,
			transform = b3.Transform_identity,
			scale = {1.0, 1.0, 1.0},
			materials = &mat,
			materialCount = 1,
		},
	}

	spheres := [2]b3.CompoundSphereDef {
		{sphere = {{5.0, 0.0, 0.0}, 0.5}, material = mat},
		{sphere = {{-5.0, 0.0, 0.0}, 0.5}, material = mat},
	}

	def := b3.CompoundDef {
		capsules     = &capsules[0],
		capsuleCount = 1,
		hulls        = &hulls[0],
		hullCount    = 1,
		meshes       = &meshes[0],
		meshCount    = 1,
		spheres      = &spheres[0],
		sphereCount  = 2,
	}

	compound := b3.CreateCompound(&def)
	ENSURE(t, compound != nil) or_return
	ENSURE(t, compound.version == b3.COMPOUND_VERSION) or_return
	ENSURE(t, compound.byteCount > i32(size_of(b3.CompoundData))) or_return

	ENSURE(t, compound.capsuleCount == 1) or_return
	ENSURE(t, compound.hullCount == 1) or_return
	ENSURE(t, compound.meshCount == 1) or_return
	ENSURE(t, compound.sphereCount == 2) or_return

	ENSURE(t, compound.materialCount == 1) or_return
	ENSURE(t, compound.sharedHullCount == 1) or_return
	ENSURE(t, compound.sharedMeshCount == 1) or_return

	ENSURE(t, compound.nodeCount >= 2) or_return
	ENSURE(t, compound.nodeOffset > 0) or_return
	ENSURE(t, compound.bounds.lowerBound.x < compound.bounds.upperBound.x) or_return

	b3.DestroyCompound(compound)
	b3.DestroyMesh(meshData)
	return true
}

@(private = "file")
CompoundCreateSingleType :: proc(t: ^testing.T) -> bool {
	mat := b3.DefaultSurfaceMaterial()

	// Capsule only
	{
		cap := b3.CompoundCapsuleDef {
			capsule  = {{0, 0, 0}, {1, 0, 0}, 0.5},
			material = mat,
		}
		def := b3.CompoundDef {
			capsules     = &cap,
			capsuleCount = 1,
		}
		c := b3.CreateCompound(&def)
		ENSURE(
			t,
			c != nil && c.capsuleCount == 1 && c.hullCount == 0 && c.meshCount == 0 && c.sphereCount == 0,
		) or_return
		b3.DestroyCompound(c)
	}

	// Hull only
	{
		box := b3.MakeBoxHull(1, 1, 1)
		h := b3.CompoundHullDef {
			hull      = &box.base,
			transform = b3.Transform_identity,
			material  = mat,
		}
		def := b3.CompoundDef {
			hulls     = &h,
			hullCount = 1,
		}
		c := b3.CreateCompound(&def)
		ENSURE(t, c != nil && c.hullCount == 1 && c.sharedHullCount == 1 && c.capsuleCount == 0) or_return
		b3.DestroyCompound(c)
	}

	// Mesh only
	{
		md := b3.CreateBoxMesh(b3.Vec3_zero, b3.Vec3{1, 1, 1}, false)
		m := b3.CompoundMeshDef {
			meshData      = md,
			transform     = b3.Transform_identity,
			scale         = {1, 1, 1},
			materials     = &mat,
			materialCount = 1,
		}
		def := b3.CompoundDef {
			meshes    = &m,
			meshCount = 1,
		}
		c := b3.CreateCompound(&def)
		ENSURE(t, c != nil && c.meshCount == 1 && c.sharedMeshCount == 1) or_return
		b3.DestroyCompound(c)
		b3.DestroyMesh(md)
	}

	// Sphere only
	{
		s := b3.CompoundSphereDef {
			sphere   = {{0, 0, 0}, 1.0},
			material = mat,
		}
		def := b3.CompoundDef {
			spheres     = &s,
			sphereCount = 1,
		}
		c := b3.CreateCompound(&def)
		ENSURE(t, c != nil && c.sphereCount == 1) or_return
		b3.DestroyCompound(c)
	}

	return true
}

@(private = "file")
CompoundMaterialDedup :: proc(t: ^testing.T) -> bool {
	mat := MakeMaterial(0.4, 7)

	caps: [3]b3.CompoundCapsuleDef
	for i: i32 = 0; i < 3; i += 1 {
		caps[i].capsule = b3.Capsule{{f32(i), 0, 0}, {f32(i) + 1, 0, 0}, 0.25}
		caps[i].material = mat
	}

	def := b3.CompoundDef {
		capsules     = &caps[0],
		capsuleCount = 3,
	}
	c := b3.CreateCompound(&def)
	ENSURE(t, c.materialCount == 1) or_return
	for i: i32 = 0; i < 3; i += 1 {
		cc := b3.GetCompoundCapsule(c, i)
		ENSURE(t, cc.materialIndex == 0) or_return
	}
	b3.DestroyCompound(c)
	return true
}

@(private = "file")
CompoundMaterialDistinct :: proc(t: ^testing.T) -> bool {
	caps: [3]b3.CompoundCapsuleDef
	for i: i32 = 0; i < 3; i += 1 {
		caps[i].capsule = b3.Capsule{{f32(i), 0, 0}, {f32(i) + 1, 0, 0}, 0.25}
		caps[i].material = MakeMaterial(0.1 * f32(i + 1), u64(i + 1))
	}

	def := b3.CompoundDef {
		capsules     = &caps[0],
		capsuleCount = 3,
	}
	c := b3.CreateCompound(&def)
	ENSURE(t, c.materialCount == 3) or_return

	mats := b3.GetCompoundMaterials(c)
	ENSURE(t, mats != nil) or_return

	for i: i32 = 0; i < 3; i += 1 {
		cc := b3.GetCompoundCapsule(c, i)
		ENSURE(t, cc.materialIndex < 3) or_return
		ENSURE(t, mats[cc.materialIndex].userMaterialId == u64(i + 1)) or_return
	}

	b3.DestroyCompound(c)
	return true
}

@(private = "file")
CompoundMaterialCrossShape :: proc(t: ^testing.T) -> bool {
	// One material shared across capsule, hull, and sphere -> 1 material slot.
	mat := MakeMaterial(0.5, 99)

	cap := b3.CompoundCapsuleDef {
		capsule  = {{0, 0, 0}, {1, 0, 0}, 0.25},
		material = mat,
	}
	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	hull := b3.CompoundHullDef {
		hull      = &box.base,
		transform = b3.Transform_identity,
		material  = mat,
	}
	sph := b3.CompoundSphereDef {
		sphere   = {{5, 0, 0}, 0.5},
		material = mat,
	}

	def := b3.CompoundDef {
		capsules     = &cap,
		capsuleCount = 1,
		hulls        = &hull,
		hullCount    = 1,
		spheres      = &sph,
		sphereCount  = 1,
	}
	c := b3.CreateCompound(&def)
	ENSURE(t, c.materialCount == 1) or_return

	ENSURE(t, b3.GetCompoundCapsule(c, 0).materialIndex == 0) or_return
	ENSURE(t, b3.GetCompoundHull(c, 0).materialIndex == 0) or_return
	ENSURE(t, b3.GetCompoundSphere(c, 0).materialIndex == 0) or_return

	b3.DestroyCompound(c)
	return true
}

@(private = "file")
CompoundMaterialMeshShared :: proc(t: ^testing.T) -> bool {
	// Mesh material entries are routed through the same material map as convex
	// materials, so an identical material is deduped across mesh and convex.
	// (The comment in compound.c about meshes "not being shared" refers to the
	// per-instance materialIndices arrays, not the b3SurfaceMaterial table.)
	mat := MakeMaterial(0.3, 11)

	md := b3.CreateBoxMesh(b3.Vec3_zero, b3.Vec3{1, 1, 1}, false)
	ENSURE(t, md.materialCount == 1) or_return

	sph := b3.CompoundSphereDef {
		sphere   = {{5, 0, 0}, 0.5},
		material = mat,
	}
	mesh := b3.CompoundMeshDef {
		meshData      = md,
		transform     = b3.Transform_identity,
		scale         = {1, 1, 1},
		materials     = &mat,
		materialCount = 1,
	}

	def := b3.CompoundDef {
		meshes      = &mesh,
		meshCount   = 1,
		spheres     = &sph,
		sphereCount = 1,
	}
	c := b3.CreateCompound(&def)
	ENSURE(t, c.materialCount == 1) or_return

	b3.DestroyCompound(c)
	b3.DestroyMesh(md)
	return true
}

// ---------------------------------------------------------------------------
// Hull / mesh blob sharing
// ---------------------------------------------------------------------------

@(private = "file")
CompoundHullSharingPointer :: proc(t: ^testing.T) -> bool {
	mat := b3.DefaultSurfaceMaterial()
	box := b3.MakeBoxHull(1, 1, 1)

	hulls: [3]b3.CompoundHullDef
	for i: i32 = 0; i < 3; i += 1 {
		hulls[i].hull = &box.base
		hulls[i].transform = b3.Transform_identity
		hulls[i].transform.p.x = f32(4 * i)
		hulls[i].material = mat
	}

	def := b3.CompoundDef {
		hulls     = &hulls[0],
		hullCount = 3,
	}
	c := b3.CreateCompound(&def)
	ENSURE(t, c.hullCount == 3) or_return
	ENSURE(t, c.sharedHullCount == 1) or_return
	b3.DestroyCompound(c)
	return true
}

@(private = "file")
CompoundHullSharingContent :: proc(t: ^testing.T) -> bool {
	mat := b3.DefaultSurfaceMaterial()

	// Two box hulls built independently with identical args are byte-identical
	// (b3MakeBoxHull is deterministic and the hash is computed over the bytes).
	boxA := b3.MakeBoxHull(1, 1, 1)
	boxB := b3.MakeBoxHull(1, 1, 1)
	ENSURE(t, &boxA != &boxB) or_return

	hulls := [2]b3.CompoundHullDef {
		{hull = &boxA.base, transform = b3.Transform_identity, material = mat},
		{hull = &boxB.base, transform = b3.Transform_identity, material = mat},
	}
	hulls[1].transform.p.x = 5.0

	def := b3.CompoundDef {
		hulls     = &hulls[0],
		hullCount = 2,
	}
	c := b3.CreateCompound(&def)
	ENSURE(t, c.sharedHullCount == 1) or_return
	b3.DestroyCompound(c)
	return true
}

@(private = "file")
CompoundHullDistinct :: proc(t: ^testing.T) -> bool {
	mat := b3.DefaultSurfaceMaterial()
	boxA := b3.MakeBoxHull(1, 1, 1)
	boxB := b3.MakeBoxHull(2, 1, 1)

	hulls := [2]b3.CompoundHullDef {
		{hull = &boxA.base, transform = b3.Transform_identity, material = mat},
		{hull = &boxB.base, transform = b3.Transform_identity, material = mat},
	}
	hulls[1].transform.p.x = 5.0

	def := b3.CompoundDef {
		hulls     = &hulls[0],
		hullCount = 2,
	}
	c := b3.CreateCompound(&def)
	ENSURE(t, c.sharedHullCount == 2) or_return
	b3.DestroyCompound(c)
	return true
}

@(private = "file")
CompoundMeshSharingPointer :: proc(t: ^testing.T) -> bool {
	mat := b3.DefaultSurfaceMaterial()
	md := b3.CreateBoxMesh(b3.Vec3_zero, b3.Vec3{1, 1, 1}, false)

	meshes: [3]b3.CompoundMeshDef
	for i: i32 = 0; i < 3; i += 1 {
		meshes[i].meshData = md
		meshes[i].transform = b3.Transform_identity
		meshes[i].transform.p.x = f32(4 * i)
		meshes[i].scale = b3.Vec3{1, 1, 1}
		meshes[i].materials = &mat
		meshes[i].materialCount = 1
	}

	def := b3.CompoundDef {
		meshes    = &meshes[0],
		meshCount = 3,
	}
	c := b3.CreateCompound(&def)
	ENSURE(t, c.meshCount == 3) or_return
	ENSURE(t, c.sharedMeshCount == 1) or_return

	b3.DestroyCompound(c)
	b3.DestroyMesh(md)
	return true
}

@(private = "file")
CompoundMeshSharingContent :: proc(t: ^testing.T) -> bool {
	mat := b3.DefaultSurfaceMaterial()
	mdA := b3.CreateBoxMesh(b3.Vec3_zero, b3.Vec3{1, 1, 1}, false)
	mdB := b3.CreateBoxMesh(b3.Vec3_zero, b3.Vec3{1, 1, 1}, false)
	ENSURE(t, mdA != mdB) or_return

	meshes := [2]b3.CompoundMeshDef {
		{meshData = mdA, transform = b3.Transform_identity, scale = {1, 1, 1}, materials = &mat, materialCount = 1},
		{meshData = mdB, transform = b3.Transform_identity, scale = {1, 1, 1}, materials = &mat, materialCount = 1},
	}
	meshes[1].transform.p.x = 5.0

	def := b3.CompoundDef {
		meshes    = &meshes[0],
		meshCount = 2,
	}
	c := b3.CreateCompound(&def)
	ENSURE(t, c.sharedMeshCount == 1) or_return

	b3.DestroyCompound(c)
	b3.DestroyMesh(mdA)
	b3.DestroyMesh(mdB)
	return true
}

@(private = "file")
CompoundMeshDistinct :: proc(t: ^testing.T) -> bool {
	mat := b3.DefaultSurfaceMaterial()
	mdA := b3.CreateBoxMesh(b3.Vec3_zero, b3.Vec3{1, 1, 1}, false)
	mdB := b3.CreateBoxMesh(b3.Vec3_zero, b3.Vec3{2, 1, 1}, false)

	meshes := [2]b3.CompoundMeshDef {
		{meshData = mdA, transform = b3.Transform_identity, scale = {1, 1, 1}, materials = &mat, materialCount = 1},
		{meshData = mdB, transform = b3.Transform_identity, scale = {1, 1, 1}, materials = &mat, materialCount = 1},
	}
	meshes[1].transform.p.x = 5.0

	def := b3.CompoundDef {
		meshes    = &meshes[0],
		meshCount = 2,
	}
	c := b3.CreateCompound(&def)
	ENSURE(t, c.sharedMeshCount == 2) or_return

	b3.DestroyCompound(c)
	b3.DestroyMesh(mdA)
	b3.DestroyMesh(mdB)
	return true
}

@(private = "file")
CompoundChildDispatch :: proc(t: ^testing.T) -> bool {
	mat := b3.DefaultSurfaceMaterial()
	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	md := b3.CreateBoxMesh(b3.Vec3_zero, b3.Vec3{0.5, 0.5, 0.5}, false)

	caps := [2]b3.CompoundCapsuleDef {
		{capsule = {{0, 0, 0}, {1, 0, 0}, 0.2}, material = mat},
		{capsule = {{0, 2, 0}, {1, 2, 0}, 0.2}, material = mat},
	}
	hulls := [1]b3.CompoundHullDef{{hull = &box.base, transform = b3.Transform_identity, material = mat}}
	hulls[0].transform.p = b3.Vec3{5, 0, 0}
	meshes := [1]b3.CompoundMeshDef {
		{meshData = md, transform = b3.Transform_identity, scale = {1, 1, 1}, materials = &mat, materialCount = 1},
	}
	meshes[0].transform.p = b3.Vec3{0, 0, 5}
	spheres := [1]b3.CompoundSphereDef{{sphere = {{-5, 0, 0}, 0.5}, material = mat}}

	def := b3.CompoundDef {
		capsules     = &caps[0],
		capsuleCount = 2,
		hulls        = &hulls[0],
		hullCount    = 1,
		meshes       = &meshes[0],
		meshCount    = 1,
		spheres      = &spheres[0],
		sphereCount  = 1,
	}
	c := b3.CreateCompound(&def)

	// Index ordering is capsules -> hulls -> meshes -> spheres.
	ENSURE(t, b3.GetCompoundChild(c, 0).type == .capsuleShape) or_return
	ENSURE(t, b3.GetCompoundChild(c, 1).type == .capsuleShape) or_return
	ENSURE(t, b3.GetCompoundChild(c, 2).type == .hullShape) or_return
	ENSURE(t, b3.GetCompoundChild(c, 3).type == .meshShape) or_return
	ENSURE(t, b3.GetCompoundChild(c, 4).type == .sphereShape) or_return

	// Capsule and sphere children always report identity transform. The position
	// is encoded in the shape itself (capsule->center{1,2}, sphere->center).
	cap0 := b3.GetCompoundChild(c, 0)
	ENSURE_SMALL(t, cap0.transform.p.x, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, cap0.transform.p.y, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, cap0.transform.p.z, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, cap0.capsule.center2.x - 1.0, b3.FLT_EPSILON) or_return

	sph := b3.GetCompoundChild(c, 4)
	ENSURE_SMALL(t, sph.transform.p.x, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, sph.sphere.center.x + 5.0, b3.FLT_EPSILON) or_return

	// Hull and mesh children carry their stored transform.
	hull := b3.GetCompoundChild(c, 2)
	ENSURE_SMALL(t, hull.transform.p.x - 5.0, b3.FLT_EPSILON) or_return

	mesh := b3.GetCompoundChild(c, 3)
	ENSURE_SMALL(t, mesh.transform.p.z - 5.0, b3.FLT_EPSILON) or_return
	ENSURE(t, mesh.mesh.data != nil) or_return

	b3.DestroyCompound(c)
	b3.DestroyMesh(md)
	return true
}

@(private = "file")
CompoundAABBContainsChildren :: proc(t: ^testing.T) -> bool {
	mat := b3.DefaultSurfaceMaterial()
	spheres := [2]b3.CompoundSphereDef {
		{sphere = {{-3, 0, 0}, 1.0}, material = mat},
		{sphere = {{4, 0, 0}, 0.5}, material = mat},
	}
	def := b3.CompoundDef {
		spheres     = &spheres[0],
		sphereCount = 2,
	}
	c := b3.CreateCompound(&def)

	local := b3.ComputeCompoundAABB(c, b3.Transform_identity)
	ENSURE(t, local.lowerBound.x <= f32(-4.0) + f32(1e-5)) or_return
	ENSURE(t, local.upperBound.x >= f32(4.5) - f32(1e-5)) or_return
	ENSURE(t, local.lowerBound.y <= f32(-1.0) + f32(1e-5)) or_return
	ENSURE(t, local.upperBound.y >= f32(1.0) - f32(1e-5)) or_return

	// Translation commutes through the bounding-box transform.
	xf := b3.Transform {
		p = {10, 20, 30},
		q = b3.Quat_identity,
	}
	world := b3.ComputeCompoundAABB(c, xf)
	ENSURE_SMALL(t, world.lowerBound.x - (local.lowerBound.x + 10.0), 1e-4) or_return
	ENSURE_SMALL(t, world.upperBound.y - (local.upperBound.y + 20.0), 1e-4) or_return
	ENSURE_SMALL(t, world.lowerBound.z - (local.lowerBound.z + 30.0), 1e-4) or_return

	b3.DestroyCompound(c)
	return true
}

@(private = "file")
CompoundRayCastMiss :: proc(t: ^testing.T) -> bool {
	mat := b3.DefaultSurfaceMaterial()
	sph := b3.CompoundSphereDef {
		sphere   = {{0, 0, 0}, 0.5},
		material = mat,
	}
	def := b3.CompoundDef {
		spheres     = &sph,
		sphereCount = 1,
	}
	c := b3.CreateCompound(&def)

	// Ray well above the sphere on a parallel path.
	input := b3.RayCastInput {
		origin      = {-5, 5, 0},
		translation = {10, 0, 0},
		maxFraction = 1.0,
	}
	out := b3.RayCastCompound(c, &input)
	ENSURE(t, out.hit == false) or_return

	b3.DestroyCompound(c)
	return true
}

@(private = "file")
CompoundRayCastClosest :: proc(t: ^testing.T) -> bool {
	matA := MakeMaterial(0.4, 100)
	matB := MakeMaterial(0.4, 200)

	// Two unit spheres along +X. Ray from origin must hit the nearer one first.
	spheres := [2]b3.CompoundSphereDef {
		{sphere = {{5, 0, 0}, 1.0}, material = matA},
		{sphere = {{10, 0, 0}, 1.0}, material = matB},
	}
	def := b3.CompoundDef {
		spheres     = &spheres[0],
		sphereCount = 2,
	}
	c := b3.CreateCompound(&def)

	input := b3.RayCastInput {
		origin      = {0, 0, 0},
		translation = {20, 0, 0},
		maxFraction = 1.0,
	}
	out := b3.RayCastCompound(c, &input)
	ENSURE(t, out.hit == true) or_return

	// Front face of the nearer sphere is at x=4 -> fraction 4/20 = 0.2.
	ENSURE_SMALL(t, out.fraction - 0.2, 1e-4) or_return
	ENSURE_SMALL(t, out.normal.x + 1.0, 1e-4) or_return
	ENSURE(t, out.childIndex == 0) or_return

	mats := b3.GetCompoundMaterials(c)
	ENSURE(t, mats[out.materialIndex].userMaterialId == 100) or_return

	b3.DestroyCompound(c)
	return true
}

@(private = "file")
CompoundRayCastHullNormalRotation :: proc(t: ^testing.T) -> bool {
	// A unit box rotated 90 degrees about Z, placed at compound +X. The ray hits the
	// face that, in compound space, points back toward -X. Verifies that the
	// normal returned by the cast has been rotated from hull-local space back
	// into compound space.
	mat := b3.DefaultSurfaceMaterial()
	box := b3.MakeBoxHull(1, 1, 1)

	hull := b3.CompoundHullDef {
		hull = &box.base,
		transform = {p = {5, 0, 0}, q = b3.MakeQuatFromAxisAngle(b3.Vec3_axisZ, 0.5 * b3.PI)},
		material = mat,
	}
	def := b3.CompoundDef {
		hulls     = &hull,
		hullCount = 1,
	}
	c := b3.CreateCompound(&def)

	input := b3.RayCastInput {
		origin      = {0, 0, 0},
		translation = {20, 0, 0},
		maxFraction = 1.0,
	}
	out := b3.RayCastCompound(c, &input)
	ENSURE(t, out.hit == true) or_return
	// Box has |hx|=1; with the rotation a face still intersects the +X ray at x=4.
	ENSURE_SMALL(t, out.fraction - 0.2, 1e-4) or_return
	ENSURE_SMALL(t, out.normal.x + 1.0, 1e-3) or_return
	ENSURE_SMALL(t, out.normal.y, 1e-3) or_return
	ENSURE_SMALL(t, out.normal.z, 1e-3) or_return

	b3.DestroyCompound(c)
	return true
}

@(private = "file")
CompoundShapeCastClosest :: proc(t: ^testing.T) -> bool {
	mat := b3.DefaultSurfaceMaterial()
	spheres := [2]b3.CompoundSphereDef {
		{sphere = {{5, 0, 0}, 1.0}, material = mat},
		{sphere = {{10, 0, 0}, 1.0}, material = mat},
	}
	def := b3.CompoundDef {
		spheres     = &spheres[0],
		sphereCount = 2,
	}
	c := b3.CreateCompound(&def)

	point := b3.Vec3{0, 0, 0}
	input := b3.ShapeCastInput {
		proxy = {points = &point, count = 1, radius = 0.25},
		translation = {20, 0, 0},
		maxFraction = 1.0,
		canEncroach = false,
	}
	out := b3.ShapeCastCompound(c, &input)
	ENSURE(t, out.hit == true) or_return
	// Closest contact: caster radius 0.25 + sphere radius 1.0 -> first contact at x ~= 3.75.
	ENSURE_SMALL(t, out.fraction - f32(3.75) / f32(20.0), 1e-3) or_return
	ENSURE(t, out.childIndex == 0) or_return

	b3.DestroyCompound(c)
	return true
}

@(private = "file")
CompoundShapeCastMiss :: proc(t: ^testing.T) -> bool {
	mat := b3.DefaultSurfaceMaterial()
	sph := b3.CompoundSphereDef {
		sphere   = {{5, 0, 0}, 1.0},
		material = mat,
	}
	def := b3.CompoundDef {
		spheres     = &sph,
		sphereCount = 1,
	}
	c := b3.CreateCompound(&def)

	point := b3.Vec3{0, 5, 0}
	input := b3.ShapeCastInput {
		proxy = {points = &point, count = 1, radius = 0.25},
		translation = {20, 0, 0},
		maxFraction = 1.0,
		canEncroach = false,
	}
	ENSURE(t, b3.ShapeCastCompound(c, &input).hit == false) or_return

	// An empty proxy has nothing to sweep
	input.proxy.count = 0
	ENSURE(t, b3.ShapeCastCompound(c, &input).hit == false) or_return

	b3.DestroyCompound(c)
	return true
}

@(private = "file")
CompoundShapeCastHullNormalRotation :: proc(t: ^testing.T) -> bool {
	// Same setup as the ray cast rotation test. The result comes back in compound space, so the
	// hull local face the sweep lands on says nothing about the reported normal and point.
	mat := b3.DefaultSurfaceMaterial()
	box := b3.MakeBoxHull(1, 1, 1)

	hull := b3.CompoundHullDef {
		hull = &box.base,
		transform = {p = {5, 0, 0}, q = b3.MakeQuatFromAxisAngle(b3.Vec3_axisZ, 0.5 * b3.PI)},
		material = mat,
	}
	def := b3.CompoundDef {
		hulls     = &hull,
		hullCount = 1,
	}
	c := b3.CreateCompound(&def)

	point := b3.Vec3{0, 0, 0}
	input := b3.ShapeCastInput {
		proxy = {points = &point, count = 1, radius = 0.25},
		translation = {20, 0, 0},
		maxFraction = 1.0,
		canEncroach = false,
	}
	out := b3.ShapeCastCompound(c, &input)
	ENSURE(t, out.hit == true) or_return

	// Near face at x=4, caster radius 0.25 so contact at x=3.75
	ENSURE_SMALL(t, out.fraction - f32(3.75) / f32(20.0), 1e-3) or_return
	ENSURE_SMALL(t, out.normal.x + 1.0, 1e-3) or_return
	ENSURE_SMALL(t, out.normal.y, 1e-3) or_return
	ENSURE_SMALL(t, out.normal.z, 1e-3) or_return
	ENSURE_SMALL(t, out.point.x - 4.0, 1e-3) or_return
	ENSURE_SMALL(t, out.point.y, 1e-3) or_return
	ENSURE_SMALL(t, out.point.z, 1e-3) or_return
	ENSURE(t, out.childIndex == 0) or_return

	b3.DestroyCompound(c)
	return true
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

	def := b3.MeshDef{}
	def.vertices = &vertices[0]
	def.stride = size_of(b3.Vec3)
	def.indices = &indices[0]
	def.materialIndices = &materialIndices[0]
	def.vertexCount = 6
	def.triangleCount = 2

	return b3.CreateMesh(&def, nil, 0)
}

@(private = "file")
CompoundMeshMaterialRemap :: proc(t: ^testing.T) -> bool {
	// A mesh child carries its own small material table. Casts report the triangle material
	// after it has been remapped into the compound material array.
	matA := MakeMaterial(0.3, 100)
	matB := MakeMaterial(0.7, 200)
	materials := [2]b3.SurfaceMaterial{matA, matB}

	md := MakeTwoMaterialMesh()
	mesh := b3.CompoundMeshDef {
		meshData      = md,
		transform     = b3.Transform_identity,
		scale         = {1, 1, 1},
		materials     = &materials[0],
		materialCount = 2,
	}
	def := b3.CompoundDef {
		meshes    = &mesh,
		meshCount = 1,
	}
	c := b3.CreateCompound(&def)

	table := b3.GetCompoundMaterials(c)

	// Straight down onto the material 0 triangle
	rayA := b3.RayCastInput {
		origin      = {-2, 5, 0},
		translation = {0, -10, 0},
		maxFraction = 1.0,
	}
	outA := b3.RayCastCompound(c, &rayA)
	ENSURE(t, outA.hit == true) or_return
	ENSURE(t, table[outA.materialIndex].userMaterialId == 100) or_return

	rayB := b3.RayCastInput {
		origin      = {2, 5, 0},
		translation = {0, -10, 0},
		maxFraction = 1.0,
	}
	outB := b3.RayCastCompound(c, &rayB)
	ENSURE(t, outB.hit == true) or_return
	ENSURE(t, table[outB.materialIndex].userMaterialId == 200) or_return

	// The shape cast path does the same remap
	point := b3.Vec3{2, 5, 0}
	castInput := b3.ShapeCastInput {
		proxy = {points = &point, count = 1, radius = 0.1},
		translation = {0, -10, 0},
		maxFraction = 1.0,
		canEncroach = false,
	}
	outCast := b3.ShapeCastCompound(c, &castInput)
	ENSURE(t, outCast.hit == true) or_return
	ENSURE(t, table[outCast.materialIndex].userMaterialId == 200) or_return

	b3.DestroyCompound(c)
	b3.DestroyMesh(md)
	return true
}

// Six separated upward facing triangles, one per material index 0 to 5
@(private = "file")
MakeSixMaterialMesh :: proc() -> ^b3.MeshData {
	vertices: [18]b3.Vec3
	indices: [18]i32
	materialIndices: [6]u8

	for i: i32 = 0; i < 6; i += 1 {
		x := 3.0 * f32(i)
		vertices[3 * i + 0] = b3.Vec3{x - 1.0, 0.0, -1.0}
		vertices[3 * i + 1] = b3.Vec3{x, 0.0, 1.0}
		vertices[3 * i + 2] = b3.Vec3{x + 1.0, 0.0, -1.0}

		indices[3 * i + 0] = 3 * i + 0
		indices[3 * i + 1] = 3 * i + 1
		indices[3 * i + 2] = 3 * i + 2

		materialIndices[i] = u8(i)
	}

	def := b3.MeshDef{}
	def.vertices = &vertices[0]
	def.stride = size_of(b3.Vec3)
	def.indices = &indices[0]
	def.materialIndices = &materialIndices[0]
	def.vertexCount = 18
	def.triangleCount = 6

	return b3.CreateMesh(&def, nil, 0)
}

@(private = "file")
CompoundMeshManyMaterials :: proc(t: ^testing.T) -> bool {
	// The capsule material bakes first so the mesh map is not the identity
	capsuleMat := MakeMaterial(0.4, 1000)

	md := MakeSixMaterialMesh()
	ENSURE(t, md.materialCount == 6) or_return

	meshMats: [6]b3.SurfaceMaterial
	for i: i32 = 0; i < 6; i += 1 {
		meshMats[i] = MakeMaterial(0.2 + 0.1 * f32(i), u64(i + 1))
	}

	cap := b3.CompoundCapsuleDef {
		capsule  = {{-10.0, 0.0, 0.0}, {-9.0, 0.0, 0.0}, 0.25},
		material = capsuleMat,
	}
	mesh := b3.CompoundMeshDef {
		meshData      = md,
		transform     = b3.Transform_identity,
		scale         = {1.0, 1.0, 1.0},
		materials     = &meshMats[0],
		materialCount = 6,
	}

	def := b3.CompoundDef {
		capsules     = &cap,
		capsuleCount = 1,
		meshes       = &mesh,
		meshCount    = 1,
	}
	c := b3.CreateCompound(&def)
	ENSURE(t, c != nil) or_return
	ENSURE(t, c.materialCount == 7) or_return

	table := b3.GetCompoundMaterials(c)

	capChild := b3.GetCompoundChild(c, 0)
	ENSURE(t, capChild.type == .capsuleShape) or_return
	ENSURE(t, capChild.materialCount == 1) or_return
	ENSURE(t, table[capChild.materialIndices[0]].userMaterialId == 1000) or_return

	meshChild := b3.GetCompoundChild(c, 1)
	ENSURE(t, meshChild.type == .meshShape) or_return
	ENSURE(t, meshChild.materialCount == 6) or_return
	for k: i32 = 0; k < 6; k += 1 {
		ENSURE(t, table[meshChild.materialIndices[k]].userMaterialId == u64(k + 1)) or_return
	}

	// Sixth triangle, material index 5
	triangleMaterials := b3.GetMeshMaterialIndices(md)
	ray := b3.RayCastInput {
		origin      = {15.0, 5.0, 0.0},
		translation = {0.0, -10.0, 0.0},
		maxFraction = 1.0,
	}
	out := b3.RayCastCompound(c, &ray)
	ENSURE(t, out.hit == true) or_return
	ENSURE(t, out.childIndex == 1) or_return
	rawIndex := i32(triangleMaterials[out.triangleIndex])
	ENSURE(t, rawIndex >= 4) or_return
	ENSURE(t, table[out.materialIndex].userMaterialId == u64(rawIndex + 1)) or_return

	b3.DestroyCompound(c)
	b3.DestroyMesh(md)
	return true
}

// Flat quad with every triangle on material index 5, so the mesh has six materials
@(private = "file")
MakeFlatMaterial5Mesh :: proc() -> ^b3.MeshData {
	vertices := [4]b3.Vec3{{-5.0, 0.0, -5.0}, {-5.0, 0.0, 5.0}, {5.0, 0.0, -5.0}, {5.0, 0.0, 5.0}}
	indices := [6]i32{0, 1, 3, 3, 2, 0}
	materialIndices := [2]u8{5, 5}

	def := b3.MeshDef{}
	def.vertices = &vertices[0]
	def.stride = size_of(b3.Vec3)
	def.indices = &indices[0]
	def.materialIndices = &materialIndices[0]
	def.vertexCount = 4
	def.triangleCount = 2

	return b3.CreateMesh(&def, nil, 0)
}

@(private = "file")
s_compoundContactMaterialId: u64

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
CompoundContactFrictionCallback :: proc "c" (
	frictionA: f32,
	userMaterialIdA: u64,
	frictionB: f32,
	userMaterialIdB: u64,
) -> f32 {
	context = runtime.default_context()
	// unified end
	if userMaterialIdA == 6 || userMaterialIdB == 6 {
		s_compoundContactMaterialId = 6
	}
	return 0.5 * (frictionA + frictionB)
}

@(private = "file")
CompoundMeshContactMaterial :: proc(t: ^testing.T) -> bool {
	s_compoundContactMaterialId = 0

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)
	b3.World_SetFrictionCallback(worldId, CompoundContactFrictionCallback)

	groundBodyDef := b3.DefaultBodyDef()
	groundBodyDef.type = .staticBody
	groundBodyId := b3.CreateBody(worldId, &groundBodyDef)

	md := MakeFlatMaterial5Mesh()
	ENSURE(t, md.materialCount == 6) or_return

	meshMats: [6]b3.SurfaceMaterial
	for i: i32 = 0; i < 6; i += 1 {
		meshMats[i] = MakeMaterial(0.5, u64(i + 1))
	}

	meshChild := b3.CompoundMeshDef {
		meshData      = md,
		transform     = b3.Transform_identity,
		scale         = {1.0, 1.0, 1.0},
		materials     = &meshMats[0],
		materialCount = 6,
	}
	compoundDef := b3.CompoundDef {
		meshes    = &meshChild,
		meshCount = 1,
	}
	compound := b3.CreateCompound(&compoundDef)
	ENSURE(t, compound != nil) or_return

	groundShapeDef := b3.DefaultShapeDef()
	compoundShapeId := b3.CreateBakedCompoundShape(groundBodyId, &groundShapeDef, compound)
	ENSURE(t, b3.Shape_GetCompound(compoundShapeId) == compound) or_return

	sphereBodyDef := b3.DefaultBodyDef()
	sphereBodyDef.type = .dynamicBody
	sphereBodyDef.position = b3.Pos{0.0, 3.0, 0.0}
	sphereBodyId := b3.CreateBody(worldId, &sphereBodyDef)
	sphereShapeDef := b3.DefaultShapeDef()
	sphereShapeDef.density = 1.0
	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}
	b3.CreateSphereShape(sphereBodyId, &sphereShapeDef, &sphere)

	for i := 0; i < 120; i += 1 {
		b3.World_Step(worldId, f32(1.0) / f32(60.0), 4)
	}

	ENSURE(t, s_compoundContactMaterialId == 6) or_return

	b3.DestroyWorld(worldId)
	b3.DestroyCompound(compound)
	b3.DestroyMesh(md)
	return true
}

@(private = "file")
CompoundOverlap :: proc(t: ^testing.T) -> bool {
	mat := b3.DefaultSurfaceMaterial()
	spheres := [2]b3.CompoundSphereDef {
		{sphere = {{-3, 0, 0}, 0.5}, material = mat},
		{sphere = {{3, 0, 0}, 0.5}, material = mat},
	}
	def := b3.CompoundDef {
		spheres     = &spheres[0],
		sphereCount = 2,
	}
	c := b3.CreateCompound(&def)

	// Proxy at the origin lies in the gap between the two spheres.
	origin := b3.Vec3{0, 0, 0}
	gap := b3.ShapeProxy {
		points = &origin,
		count  = 1,
		radius = 0.25,
	}
	ENSURE(t, b3.OverlapCompound(c, b3.Transform_identity, &gap) == false) or_return

	// Proxy at the center of the second sphere overlaps it.
	onSecond := b3.Vec3{3, 0, 0}
	hit := b3.ShapeProxy {
		points = &onSecond,
		count  = 1,
		radius = 0.1,
	}
	ENSURE(t, b3.OverlapCompound(c, b3.Transform_identity, &hit) == true) or_return

	b3.DestroyCompound(c)
	return true
}

@(private = "file")
CompoundOverlapTransformed :: proc(t: ^testing.T) -> bool {
	// The tree and the child geometry live in the compound frame while the proxy arrives in world
	// space, so a compound that is moved and turned has to pull the proxy back before querying.
	mat := b3.DefaultSurfaceMaterial()
	spheres := [2]b3.CompoundSphereDef {
		{sphere = {{-3, 0, 0}, 0.5}, material = mat},
		{sphere = {{3, 0, 0}, 0.5}, material = mat},
	}
	def := b3.CompoundDef {
		spheres     = &spheres[0],
		sphereCount = 2,
	}
	c := b3.CreateCompound(&def)

	transform := b3.Transform {
		p = {10, 20, 30},
		q = b3.MakeQuatFromAxisAngle(b3.Vec3_axisZ, 0.5 * b3.PI),
	}

	gapPoint := b3.TransformPoint(transform, b3.Vec3{0, 0, 0})
	gap := b3.ShapeProxy {
		points = &gapPoint,
		count  = 1,
		radius = 0.25,
	}
	ENSURE(t, b3.OverlapCompound(c, transform, &gap) == false) or_return

	hitPoint := b3.TransformPoint(transform, b3.Vec3{3, 0, 0})
	hit := b3.ShapeProxy {
		points = &hitPoint,
		count  = 1,
		radius = 0.1,
	}
	ENSURE(t, b3.OverlapCompound(c, transform, &hit) == true) or_return

	// A proxy left in the compound frame must not register against the moved compound
	localPoint := b3.Vec3{3, 0, 0}
	stale := b3.ShapeProxy {
		points = &localPoint,
		count  = 1,
		radius = 0.1,
	}
	ENSURE(t, b3.OverlapCompound(c, transform, &stale) == false) or_return

	b3.DestroyCompound(c)
	return true
}

@(private = "file")
CompoundOverlapChildTypes :: proc(t: ^testing.T) -> bool {
	// One child of every type, spaced far enough apart that a hit names the type it came from.
	// Hull and mesh children add a child transform underneath the compound transform.
	mat := b3.DefaultSurfaceMaterial()
	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	md := b3.CreateBoxMesh(b3.Vec3_zero, b3.Vec3{0.5, 0.5, 0.5}, false)

	capsule := b3.CompoundCapsuleDef {
		capsule  = {{-10, 0, 0}, {-9, 0, 0}, 0.25},
		material = mat,
	}
	hull := b3.CompoundHullDef {
		hull = &box.base,
		transform = {p = {0, 0, 0}, q = b3.MakeQuatFromAxisAngle(b3.Vec3_axisZ, 0.25 * b3.PI)},
		material = mat,
	}
	mesh := b3.CompoundMeshDef {
		meshData = md,
		transform = {p = {10, 0, 0}, q = b3.MakeQuatFromAxisAngle(b3.Vec3_axisY, 0.5 * b3.PI)},
		scale = {1, 1, 1},
		materials = &mat,
		materialCount = 1,
	}
	sphere := b3.CompoundSphereDef {
		sphere   = {{20, 0, 0}, 0.5},
		material = mat,
	}

	def := b3.CompoundDef {
		capsules     = &capsule,
		capsuleCount = 1,
		hulls        = &hull,
		hullCount    = 1,
		meshes       = &mesh,
		meshCount    = 1,
		spheres      = &sphere,
		sphereCount  = 1,
	}
	c := b3.CreateCompound(&def)

	transform := b3.Transform {
		p = {100, 200, 300},
		q = b3.MakeQuatFromAxisAngle(b3.Vec3_axisZ, 0.5 * b3.PI),
	}

	// Points sitting on or inside each child, then carried out to world space
	localPoints := [4]b3.Vec3 {
		{-9.5, 0, 0}, // capsule axis
		{0, 0, 0}, // hull center
		{10, 0.45, 0}, // just under a mesh face
		{20, 0, 0}, // sphere center
	}

	for i := 0; i < 4; i += 1 {
		worldPoint := b3.TransformPoint(transform, localPoints[i])
		proxy := b3.ShapeProxy {
			points = &worldPoint,
			count  = 1,
			radius = 0.1,
		}
		ENSURE(t, b3.OverlapCompound(c, transform, &proxy) == true) or_return
	}

	// A gap between the hull and the mesh
	gapPoint := b3.TransformPoint(transform, b3.Vec3{5, 0, 0})
	gap := b3.ShapeProxy {
		points = &gapPoint,
		count  = 1,
		radius = 0.1,
	}
	ENSURE(t, b3.OverlapCompound(c, transform, &gap) == false) or_return

	b3.DestroyCompound(c)
	b3.DestroyMesh(md)
	return true
}

@(private = "file")
CompoundOverlapSegmentProxy :: proc(t: ^testing.T) -> bool {
	// A multi point proxy has to be carried into the compound frame point by point
	mat := b3.DefaultSurfaceMaterial()
	spheres := [2]b3.CompoundSphereDef {
		{sphere = {{-3, 0, 0}, 0.5}, material = mat},
		{sphere = {{3, 0, 0}, 0.5}, material = mat},
	}
	def := b3.CompoundDef {
		spheres     = &spheres[0],
		sphereCount = 2,
	}
	c := b3.CreateCompound(&def)

	transform := b3.Transform {
		p = {-40, 15, 7},
		q = b3.MakeQuatFromAxisAngle(b3.Vec3_axisY, 0.5 * b3.PI),
	}

	// Segment that stays inside the gap
	inGap := [2]b3.Vec3 {
		b3.TransformPoint(transform, b3.Vec3{-1, 0, 0}),
		b3.TransformPoint(transform, b3.Vec3{1, 0, 0}),
	}
	gap := b3.ShapeProxy {
		points = &inGap[0],
		count  = 2,
		radius = 0.25,
	}
	ENSURE(t, b3.OverlapCompound(c, transform, &gap) == false) or_return

	// Same segment stretched until one end reaches the second sphere
	reaching := [2]b3.Vec3 {
		b3.TransformPoint(transform, b3.Vec3{-1, 0, 0}),
		b3.TransformPoint(transform, b3.Vec3{2.9, 0, 0}),
	}
	hit := b3.ShapeProxy {
		points = &reaching[0],
		count  = 2,
		radius = 0.25,
	}
	ENSURE(t, b3.OverlapCompound(c, transform, &hit) == true) or_return

	b3.DestroyCompound(c)
	return true
}

@(private = "file")
QueryAccumulator :: struct {
	childIndices: [8]i32,
	count:        i32,
	stopAfter:    i32, // -1 = never stop early
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
QueryCallback :: proc "c" (compound: ^b3.CompoundData, childIndex: i32, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	_ = compound
	acc := (^QueryAccumulator)(context_)
	if acc.count < i32(len(acc.childIndices)) {
		acc.childIndices[acc.count] = childIndex
		acc.count += 1
	}
	if acc.stopAfter >= 0 && acc.count >= acc.stopAfter {
		return false
	}
	return true
}

@(private = "file")
CompoundQuery :: proc(t: ^testing.T) -> bool {
	mat := b3.DefaultSurfaceMaterial()
	spheres := [3]b3.CompoundSphereDef {
		{sphere = {{-10, 0, 0}, 0.5}, material = mat},
		{sphere = {{0, 0, 0}, 0.5}, material = mat},
		{sphere = {{10, 0, 0}, 0.5}, material = mat},
	}
	def := b3.CompoundDef {
		spheres     = &spheres[0],
		sphereCount = 3,
	}
	c := b3.CreateCompound(&def)

	// Tight box around the middle sphere. Only it should be reported.
	middle := b3.AABB{{-1, -1, -1}, {1, 1, 1}}
	acc := QueryAccumulator {
		stopAfter = -1,
	}
	b3.QueryCompound(c, middle, QueryCallback, &acc)
	ENSURE(t, acc.count == 1) or_return
	ENSURE(t, acc.childIndices[0] == 1) or_return

	// Wide box overlapping all three; early-exit after the first reported child.
	wide := b3.AABB{{-20, -1, -1}, {20, 1, 1}}
	stop := QueryAccumulator {
		stopAfter = 1,
	}
	b3.QueryCompound(c, wide, QueryCallback, &stop)
	ENSURE(t, stop.count == 1) or_return

	// Without early-exit, all three are visited.
	all := QueryAccumulator {
		stopAfter = -1,
	}
	b3.QueryCompound(c, wide, QueryCallback, &all)
	ENSURE(t, all.count == 3) or_return

	b3.DestroyCompound(c)
	return true
}

@(private = "file")
CompoundMover :: proc(t: ^testing.T) -> bool {
	mat := b3.DefaultSurfaceMaterial()
	box := b3.MakeBoxHull(0.5, 0.5, 0.5)

	// Two boxes side-by-side along X, gap of 1 between them.
	hulls := [2]b3.CompoundHullDef {
		{hull = &box.base, transform = {p = {-1.0, 0, 0}, q = b3.Quat_identity}, material = mat},
		{hull = &box.base, transform = {p = {1.0, 0, 0}, q = b3.Quat_identity}, material = mat},
	}
	def := b3.CompoundDef {
		hulls     = &hulls[0],
		hullCount = 2,
	}
	c := b3.CreateCompound(&def)

	// Small capsule mover sitting on top of the boxes, low enough to penetrate both.
	mover := b3.Capsule{{-1.0, 0.6, 0}, {1.0, 0.6, 0}, 0.2}

	planes: [8]b3.PlaneResult
	planeCount := b3.CollideMoverAndCompound(&planes[0], 8, c, &mover)

	// Both boxes contribute at least one plane each; the +Y face of each box
	// should produce a plane whose normal points roughly +Y in compound space.
	ENSURE(t, planeCount >= 2) or_return

	upPlanes := 0
	for i: i32 = 0; i < planeCount; i += 1 {
		if planes[i].plane.normal.y > 0.9 {
			upPlanes += 1
		}
	}
	ENSURE(t, upPlanes >= 2) or_return

	// Capacity cap is honored: ask for 1 and the call must not exceed it.
	one: [1]b3.PlaneResult
	capped := b3.CollideMoverAndCompound(&one[0], 1, c, &mover)
	ENSURE(t, capped <= 1) or_return

	b3.DestroyCompound(c)
	return true
}

@(private = "file")
CompoundMoverRotatedChild :: proc(t: ^testing.T) -> bool {
	// The child solves the mover in its own frame, so the plane it returns has to be carried
	// back into the compound frame before the caller sees it.
	mat := MakeMaterial(0.25, 77)
	box := b3.MakeBoxHull(0.5, 0.5, 0.5)

	// Quarter turn about z, so the compound space top face is a side face of the hull
	hull := b3.CompoundHullDef {
		hull = &box.base,
		transform = {p = {0, 0, 0}, q = b3.MakeQuatFromAxisAngle(b3.Vec3_axisZ, 0.5 * b3.PI)},
		material = mat,
	}
	def := b3.CompoundDef {
		hulls     = &hull,
		hullCount = 1,
	}
	c := b3.CreateCompound(&def)

	mover := b3.Capsule{{-0.1, 0.6, 0}, {0.1, 0.6, 0}, 0.2}

	planes: [8]b3.PlaneResult
	planeCount := b3.CollideMoverAndCompound(&planes[0], 8, c, &mover)
	ENSURE(t, planeCount >= 1) or_return

	ENSURE_SMALL(t, planes[0].plane.normal.x, 1e-3) or_return
	ENSURE_SMALL(t, planes[0].plane.normal.y - 1.0, 1e-3) or_return
	ENSURE_SMALL(t, planes[0].plane.normal.z, 1e-3) or_return
	ENSURE_SMALL(t, planes[0].point.y - 0.5, 1e-3) or_return

	ENSURE(t, planes[0].childIndex == 0) or_return
	table := b3.GetCompoundMaterials(c)
	ENSURE(t, table[planes[0].materialIndex].userMaterialId == 77) or_return

	b3.DestroyCompound(c)
	return true
}

@(private = "file")
BuildSerializableCompound :: proc(md: ^b3.MeshData) -> ^b3.CompoundData {
	mat := b3.DefaultSurfaceMaterial()
	box := b3.MakeBoxHull(0.5, 0.5, 0.5)

	cap := b3.CompoundCapsuleDef {
		capsule  = {{-2, 0, 0}, {-1, 0, 0}, 0.2},
		material = mat,
	}
	hull := b3.CompoundHullDef {
		hull = &box.base,
		transform = {p = {5, 0, 0}, q = b3.Quat_identity},
		material = mat,
	}
	mesh := b3.CompoundMeshDef {
		meshData = md,
		transform = {p = {0, 0, 5}, q = b3.Quat_identity},
		scale = {1, 1, 1},
		materials = &mat,
		materialCount = 1,
	}
	sph := b3.CompoundSphereDef {
		sphere   = {{-5, 0, 0}, 0.5},
		material = mat,
	}

	def := b3.CompoundDef {
		capsules     = &cap,
		capsuleCount = 1,
		hulls        = &hull,
		hullCount    = 1,
		meshes       = &mesh,
		meshCount    = 1,
		spheres      = &sph,
		sphereCount  = 1,
	}
	return b3.CreateCompound(&def)
}

@(private = "file")
CompoundSerializeRoundtrip :: proc(t: ^testing.T) -> bool {
	md := b3.CreateBoxMesh(b3.Vec3_zero, b3.Vec3{0.5, 0.5, 0.5}, false)
	a := BuildSerializableCompound(md)

	// Snapshot a few queries on the original.
	aabbA := b3.ComputeCompoundAABB(a, b3.Transform_identity)
	rayInput := b3.RayCastInput {
		origin      = {0, 0, 0},
		translation = {20, 0, 0},
		maxFraction = 1.0,
	}
	rayA := b3.RayCastCompound(a, &rayInput)
	ENSURE(t, rayA.hit) or_return

	byteCount := a.byteCount
	// unified: malloc/free -> make/delete on the test's allocator
	buffer := make([]u8, int(byteCount))
	ENSURE(t, buffer != nil) or_return

	reference := make([]u8, int(byteCount))
	ENSURE(t, reference != nil) or_return

	mem.copy(raw_data(buffer), a, int(byteCount))
	mem.copy(raw_data(reference), a, int(byteCount))
	b3.DestroyCompound(a)

	b := b3.ValidateCompound(raw_data(buffer), byteCount)
	ENSURE(t, b != nil) or_return
	ENSURE(t, rawptr(b) == raw_data(buffer)) or_return
	ENSURE(t, b.byteCount == byteCount) or_return
	ENSURE(t, b.capsuleCount == 1) or_return
	ENSURE(t, b.hullCount == 1) or_return
	ENSURE(t, b.meshCount == 1) or_return
	ENSURE(t, b.sphereCount == 1) or_return

	// Validation must not touch the bytes
	ENSURE(t, mem.compare(buffer, reference) == 0) or_return

	// Bounds and ray cast on the deserialized compound match the original.
	aabbB := b3.ComputeCompoundAABB(b, b3.Transform_identity)
	ENSURE_SMALL(t, aabbA.lowerBound.x - aabbB.lowerBound.x, 1e-5) or_return
	ENSURE_SMALL(t, aabbA.upperBound.z - aabbB.upperBound.z, 1e-5) or_return

	rayB := b3.RayCastCompound(b, &rayInput)
	ENSURE(t, rayB.hit == rayA.hit) or_return
	ENSURE_SMALL(t, rayB.fraction - rayA.fraction, 1e-5) or_return
	ENSURE(t, rayB.childIndex == rayA.childIndex) or_return

	delete(reference)
	delete(buffer)
	b3.DestroyMesh(md)
	return true
}

@(private = "file")
CompoundSerializeBadVersion :: proc(t: ^testing.T) -> bool {
	md := b3.CreateBoxMesh(b3.Vec3_zero, b3.Vec3{0.5, 0.5, 0.5}, false)
	a := BuildSerializableCompound(md)

	byteCount := a.byteCount
	buffer := make([]u8, int(byteCount))
	mem.copy(raw_data(buffer), a, int(byteCount))
	b3.DestroyCompound(a)

	// Corrupt the version word (first 8 bytes of b3Compound).
	(^b3.CompoundData)(raw_data(buffer)).version ~= 0x1

	b := b3.ValidateCompound(raw_data(buffer), byteCount)
	ENSURE(t, b == nil) or_return

	delete(buffer)
	b3.DestroyMesh(md)
	return true
}

@(private = "file")
CompoundSerializeWrongByteCount :: proc(t: ^testing.T) -> bool {
	md := b3.CreateBoxMesh(b3.Vec3_zero, b3.Vec3{0.5, 0.5, 0.5}, false)
	a := BuildSerializableCompound(md)

	byteCount := a.byteCount
	buffer := make([]u8, int(byteCount))
	mem.copy(raw_data(buffer), a, int(byteCount))
	b3.DestroyCompound(a)

	// Off-by-one in the declared length is rejected.
	b := b3.ValidateCompound(raw_data(buffer), byteCount + 1)
	ENSURE(t, b == nil) or_return

	delete(buffer)
	b3.DestroyMesh(md)
	return true
}

@(private = "file")
CompoundSerializeBadNodeCount :: proc(t: ^testing.T) -> bool {
	md := b3.CreateBoxMesh(b3.Vec3_zero, b3.Vec3{0.5, 0.5, 0.5}, false)
	a := BuildSerializableCompound(md)

	byteCount := a.byteCount
	buffer := make([]u8, int(byteCount))
	mem.copy(raw_data(buffer), a, int(byteCount))
	b3.DestroyCompound(a)

	header := (^b3.CompoundData)(raw_data(buffer))
	nodeCount := header.nodeCount

	// Node array runs off the end of the buffer
	header.nodeCount = byteCount
	ENSURE(t, b3.ValidateCompound(raw_data(buffer), byteCount) == nil) or_return

	// Overflow in 32 bit arithmetic must still be caught
	header.nodeCount = max(i32)
	ENSURE(t, b3.ValidateCompound(raw_data(buffer), byteCount) == nil) or_return

	header.nodeCount = nodeCount
	ENSURE(t, b3.ValidateCompound(raw_data(buffer), byteCount) != nil) or_return

	delete(buffer)
	b3.DestroyMesh(md)
	return true
}

@(test)
CompoundTest :: proc(t: ^testing.T) {
	LOCK()

	if !RUN_SUBTEST(t, CompoundCreateMixed) {return}
	if !RUN_SUBTEST(t, CompoundCreateSingleType) {return}

	if !RUN_SUBTEST(t, CompoundMaterialDedup) {return}
	if !RUN_SUBTEST(t, CompoundMaterialDistinct) {return}
	if !RUN_SUBTEST(t, CompoundMaterialCrossShape) {return}
	if !RUN_SUBTEST(t, CompoundMaterialMeshShared) {return}

	if !RUN_SUBTEST(t, CompoundHullSharingPointer) {return}
	if !RUN_SUBTEST(t, CompoundHullSharingContent) {return}
	if !RUN_SUBTEST(t, CompoundHullDistinct) {return}
	if !RUN_SUBTEST(t, CompoundMeshSharingPointer) {return}
	if !RUN_SUBTEST(t, CompoundMeshSharingContent) {return}
	if !RUN_SUBTEST(t, CompoundMeshDistinct) {return}

	if !RUN_SUBTEST(t, CompoundChildDispatch) {return}

	if !RUN_SUBTEST(t, CompoundAABBContainsChildren) {return}

	if !RUN_SUBTEST(t, CompoundRayCastMiss) {return}
	if !RUN_SUBTEST(t, CompoundRayCastClosest) {return}
	if !RUN_SUBTEST(t, CompoundRayCastHullNormalRotation) {return}
	if !RUN_SUBTEST(t, CompoundShapeCastClosest) {return}
	if !RUN_SUBTEST(t, CompoundShapeCastMiss) {return}
	if !RUN_SUBTEST(t, CompoundShapeCastHullNormalRotation) {return}
	if !RUN_SUBTEST(t, CompoundMeshMaterialRemap) {return}
	if !RUN_SUBTEST(t, CompoundMeshManyMaterials) {return}
	if !RUN_SUBTEST(t, CompoundMeshContactMaterial) {return}

	if !RUN_SUBTEST(t, CompoundOverlap) {return}
	if !RUN_SUBTEST(t, CompoundOverlapTransformed) {return}
	if !RUN_SUBTEST(t, CompoundOverlapChildTypes) {return}
	if !RUN_SUBTEST(t, CompoundOverlapSegmentProxy) {return}
	if !RUN_SUBTEST(t, CompoundQuery) {return}

	if !RUN_SUBTEST(t, CompoundMover) {return}
	if !RUN_SUBTEST(t, CompoundMoverRotatedChild) {return}

	if !RUN_SUBTEST(t, CompoundSerializeRoundtrip) {return}
	if !RUN_SUBTEST(t, CompoundSerializeBadVersion) {return}
	if !RUN_SUBTEST(t, CompoundSerializeWrongByteCount) {return}
	if !RUN_SUBTEST(t, CompoundSerializeBadNodeCount) {return}
}
