// Canonical, portable world images (SPEC.md, phase 2 items 3 and 4): image bytes don't depend on
// what fresh memory held, an image with its geometry loads into a world that never saw that geometry and runs on
// bit for bit, and the canonical hash sees solver internals.
package test_box3d_port

import "core:testing"

import b3 "../../box3d"

// A scene with every geometry kind and the state an image carries: a mesh ground and a height field, a box stack
// that sleeps, spheres and capsules rolling on the mesh (triangle caches), a custom hull and a compound, distance,
// weld, motor, mover and pogo joints, and a sensor.
Img_Scene :: struct {
	world:    b3.WorldId,
	mesh:     ^b3.MeshData,
	field:    ^b3.HeightFieldData,
	compound: ^b3.CompoundData,
	hull:     ^b3.HullData,
}

img_shape_def :: proc() -> b3.ShapeDef {
	sd := b3.DefaultShapeDef()
	sd.density = 1
	sd.enableSensorEvents = true
	return sd
}

img_body :: proc(w: b3.WorldId, type: b3.BodyType, pos: b3.Vec3) -> b3.BodyId {
	bd := b3.DefaultBodyDef()
	bd.type = type
	bd.position = pos
	return b3.CreateBody(w, &bd)
}

img_box :: proc(w: b3.WorldId, type: b3.BodyType, pos, half: b3.Vec3) -> b3.BodyId {
	body := img_body(w, type, pos)
	sd := img_shape_def()
	hull := b3.MakeBoxHull(half.x, half.y, half.z)
	b3.CreateHullShape(body, &sd, &hull.base)
	return body
}

// A hull no other world has: an irregular point cloud scaled by `seed`.
img_custom_hull :: proc(seed: f32) -> ^b3.HullData {
	points := [?]b3.Vec3 {
		{-0.4, -0.3, -0.35},
		{0.45, -0.3, -0.3},
		{0.3, -0.35, 0.4},
		{-0.35, -0.3, 0.45},
		{0, 0.5 * seed, 0},
		{0.2, 0.3, -0.2 * seed},
		{-0.25, 0.25, 0.1},
	}
	return b3.CreateHull(&points[0], len(points), 16)
}

img_scene_make :: proc(s: ^Img_Scene) {
	s.world = world_make()
	w := s.world
	sd := img_shape_def()

	// Grounds: a mesh, a box beside it, a height field
	s.mesh = b3.CreateGridMesh(16, 16, 1, 0, true)
	ground := img_body(w, .staticBody, {-8, 0, -8})
	b3.CreateMeshShape(ground, &sd, s.mesh, {1, 1, 1})
	img_box(w, .staticBody, {14, -0.5, 0}, {6, 0.5, 6})
	heights: [9 * 9]f32
	for &h, i in heights {
		h = 0.1 * f32(i % 3)
	}
	hd := b3.HeightFieldDef {
		heights             = &heights[0],
		scale               = {1, 1, 1},
		countX              = 9,
		countZ              = 9,
		globalMinimumHeight = 0,
		globalMaximumHeight = 1,
	}
	s.field = b3.CreateHeightField(&hd)
	hfBody := img_body(w, .staticBody, {-4, -0.3, 10})
	b3.CreateHeightFieldShape(hfBody, &sd, s.field)

	// A stack that sleeps on the box ground
	for i in 0 ..< 5 {
		img_box(w, .dynamicBody, {14, 0.5 + f32(i) * 1.01, 0}, {0.5, 0.5, 0.5})
	}

	// Spheres and capsules rolling on the mesh, a ball on the height field
	for i in 0 ..< 4 {
		b := img_body(w, .dynamicBody, {-5 + 2 * f32(i), 2, -3})
		b3.Body_SetLinearVelocity(b, {1, 0, 0.5 * f32(i)})
		if i % 2 == 0 {
			sphere := b3.Sphere{{0, 0, 0}, 0.4}
			b3.CreateSphereShape(b, &sd, &sphere)
		} else {
			capsule := b3.Capsule{{-0.3, 0, 0}, {0.3, 0, 0}, 0.25}
			b3.CreateCapsuleShape(b, &sd, &capsule)
		}
	}
	ball := img_body(w, .dynamicBody, {-1, 2, 13})
	sphere := b3.Sphere{{0, 0, 0}, 0.3}
	b3.CreateSphereShape(ball, &sd, &sphere)

	// A custom hull and a compound of two spheres and a capsule
	s.hull = img_custom_hull(1)
	hb := img_body(w, .dynamicBody, {2, 3, 2})
	b3.CreateHullShape(hb, &sd, s.hull)
	spheres := [2]b3.CompoundSphereDef {
		{sphere = {{-0.5, 0, 0}, 0.3}, material = b3.DefaultSurfaceMaterial()},
		{sphere = {{0.5, 0, 0}, 0.3}, material = b3.DefaultSurfaceMaterial()},
	}
	capsules := [1]b3.CompoundCapsuleDef {
		{capsule = {{0, 0, -0.4}, {0, 0, 0.4}, 0.2}, material = b3.DefaultSurfaceMaterial()},
	}
	cd := b3.CompoundDef {
		spheres      = &spheres[0],
		sphereCount  = 2,
		capsules     = &capsules[0],
		capsuleCount = 1,
	}
	s.compound = b3.CreateCompound(&cd)
	cb := img_body(w, .dynamicBody, {-3, 4, 3})
	b3.CreateBakedCompoundShape(cb, &sd, s.compound)

	// Joints: distance and weld between boxes, motor, mover and pogo to the ground
	a := img_box(w, .dynamicBody, {0, 3, 5}, {0.3, 0.3, 0.3})
	b := img_box(w, .dynamicBody, {1, 3, 5}, {0.3, 0.3, 0.3})
	c := img_box(w, .dynamicBody, {2, 3, 5}, {0.3, 0.3, 0.3})
	dj := b3.DefaultDistanceJointDef()
	dj.base.bodyIdA, dj.base.bodyIdB = a, b
	dj.length = 1
	dj.enableSpring = true
	dj.hertz, dj.dampingRatio = 4, 0.3
	b3.CreateDistanceJoint(w, &dj)
	wj := b3.DefaultWeldJointDef()
	wj.base.bodyIdA, wj.base.bodyIdB = b, c
	wj.base.localFrameA.p = {0.5, 0, 0}
	wj.base.localFrameB.p = {-0.5, 0, 0}
	wj.angularHertz, wj.angularDampingRatio = 3, 0.5
	b3.CreateWeldJoint(w, &wj)
	mj := b3.DefaultMotorJointDef()
	mj.base.bodyIdA, mj.base.bodyIdB = ground, a
	mj.linearVelocity = {0.5, 0, 0}
	mj.maxVelocityForce = 20
	b3.CreateMotorJoint(w, &mj)
	mover := img_box(w, .dynamicBody, {-6, 1, 4}, {0.3, 0.6, 0.3})
	mv := b3.DefaultMoverJointDef()
	mv.base.bodyIdA, mv.base.bodyIdB = ground, mover
	mv.linearVelocity = {1, 0, 0}
	mv.maxForce = 30
	b3.CreateMoverJoint(w, &mv)
	pj := b3.DefaultPogoJointDef()
	pj.base.bodyIdA, pj.base.bodyIdB = ground, mover
	pj.normal = {0, 1, 0}
	pj.restLength = 1
	pj.hertz, pj.dampingRatio = 5, 0.7
	b3.CreatePogoJoint(w, &pj)

	// A sensor over the mesh
	sensorBody := img_body(w, .staticBody, {-2, 1, -3})
	ssd := b3.DefaultShapeDef()
	ssd.isSensor = true
	ssd.enableSensorEvents = true
	box := b3.MakeBoxHull(2, 1, 2)
	b3.CreateHullShape(sensorBody, &ssd, &box.base)
}

img_scene_destroy :: proc(s: ^Img_Scene) {
	world_destroy(s.world)
	b3.DestroyMesh(s.mesh)
	b3.DestroyHeightField(s.field)
	b3.DestroyCompound(s.compound)
	b3.DestroyHull(s.hull)
}

img_bytes :: proc(st: ^b3.WorldState) -> []u8 {
	return b3.WorldState_GetData(st)[:b3.WorldState_GetSize(st)]
}

// The first offset where two images differ, -1 if equal.
img_first_diff :: proc(a, b: []u8) -> int {
	for i in 0 ..< min(len(a), len(b)) {
		if a[i] != b[i] {
			return i
		}
	}
	return len(a) == len(b) ? -1 : min(len(a), len(b))
}

// The same scene in worlds whose arenas start filled with different bytes saves byte-identical images.
@(test)
image_bytes_are_canonical :: proc(t: ^testing.T) {
	FILLS := [3]i32{0x00, 0xCD, 0x5A}
	scenes: [len(FILLS)]Img_Scene
	states: [len(FILLS)]^b3.WorldState
	for &s, k in scenes {
		// Only this test sets the fill; another test's world created meanwhile just gets filled memory.
		b3.arenaDebugFill = FILLS[k]
		img_scene_make(&s)
		states[k] = b3.CreateWorldState()
	}
	b3.arenaDebugFill = -1
	defer for &s, k in scenes {
		img_scene_destroy(&s)
		b3.DestroyWorldState(states[k])
	}

	for tick in 1 ..= 320 {
		for s in scenes {
			step(s.world, 1)
		}
		if tick % 16 != 0 {
			continue
		}
		withGeometry := tick % 64 == 0
		for s, k in scenes {
			testing.expect(t, b3.World_SaveImage(s.world, states[k], withGeometry) > 0)
		}
		h0 := b3.World_HashState(scenes[0].world)
		for k in 1 ..< len(scenes) {
			a, b := img_bytes(states[0]), img_bytes(states[k])
			d := img_first_diff(a, b)
			if !testing.expectf(
				t,
				d < 0,
				"tick %d fill %x: images differ at byte %d of %d",
				tick,
				FILLS[k],
				d,
				len(a),
			) {
				return
			}
			testing.expect_value(t, b3.World_HashState(scenes[k].world), h0)
		}
	}
}

// An image with its geometry, taken after shapes with new geometry were added, loads into a fresh world that never
// saw that geometry; both then step on bit for bit. An image without geometry is refused there, untouched.
@(test)
image_late_join :: proc(t: ^testing.T) {
	s: Img_Scene
	img_scene_make(&s)
	defer img_scene_destroy(&s)
	step(s.world, 50)
	spawned := img_custom_hull(1.7)
	defer b3.DestroyHull(spawned)
	for i in 0 ..< 3 {
		body := img_body(s.world, .dynamicBody, {f32(i), 4, -1})
		sd := img_shape_def()
		b3.CreateHullShape(body, &sd, spawned)
	}
	step(s.world, 50)

	full, refs := b3.CreateWorldState(), b3.CreateWorldState()
	defer b3.DestroyWorldState(full)
	defer b3.DestroyWorldState(refs)
	testing.expect(t, b3.World_SaveImage(s.world, full, true) > 0)
	testing.expect(t, b3.World_SaveImage(s.world, refs, false) > 0)
	testing.expect(t, len(img_bytes(full)) > len(img_bytes(refs)))

	// Without geometry: refused before anything is touched.
	empty := world_make()
	defer world_destroy(empty)
	h := b3.World_HashState(empty)
	testing.expect(t, !b3.World_LoadImage(empty, img_bytes(refs)))
	testing.expect_value(t, b3.World_HashState(empty), h)

	// With geometry, into a world that never saw it.
	late := world_make()
	defer world_destroy(late)
	testing.expect(t, b3.World_LoadImage(late, img_bytes(full)))
	testing.expect_value(t, b3.World_HashState(late), b3.World_HashState(s.world))
	// A reload of the same image, now with the geometry known: the same world.
	testing.expect(t, b3.World_LoadImage(late, img_bytes(refs)))
	testing.expect_value(t, b3.World_HashState(late), b3.World_HashState(s.world))

	for tick in 1 ..= 200 {
		step(s.world, 1)
		step(late, 1)
		ha, hb := b3.World_HashState(s.world), b3.World_HashState(late)
		if !testing.expectf(t, ha == hb, "tick %d after the late join: %x != %x", tick, ha, hb) {
			return
		}
	}
}

// A by-reference image restores in place: the hash of the image is the world's hash, and the run continues as
// before.
@(test)
image_restore_in_place :: proc(t: ^testing.T) {
	s: Img_Scene
	img_scene_make(&s)
	defer img_scene_destroy(&s)
	st := b3.CreateWorldState()
	defer b3.DestroyWorldState(st)
	step(s.world, 60)
	testing.expect(t, b3.World_SaveImage(s.world, st, false) > 0)
	h0 := b3.World_HashState(s.world)
	testing.expect_value(t, b3.rapidhash(raw_data(img_bytes(st)), len(img_bytes(st))), h0)
	step(s.world, 90)
	h1 := b3.World_HashState(s.world)
	testing.expect(t, h1 != h0)
	testing.expect(t, b3.World_LoadImage(s.world, img_bytes(st)))
	testing.expect_value(t, b3.World_HashState(s.world), h0)
	step(s.world, 90)
	testing.expect_value(t, b3.World_HashState(s.world), h1)
	// A damaged image is refused.
	testing.expect(t, !b3.World_LoadImage(s.world, img_bytes(st)[:20]))
}

// The canonical hash covers solver internals: one contact's warm-start impulse changes it.
@(test)
image_hash_sees_warm_starting :: proc(t: ^testing.T) {
	s: Img_Scene
	img_scene_make(&s)
	defer img_scene_destroy(&s)
	step(s.world, 30)
	h0 := b3.World_HashState(s.world)
	impulse := img_contact_impulse(s.world)
	testing.expect(t, impulse != nil)
	if impulse == nil {
		return
	}
	saved := impulse^
	impulse^ = transmute(f32)(transmute(u32)saved ~ 1)
	testing.expect(t, b3.World_HashState(s.world) != h0)
	impulse^ = saved
	testing.expect_value(t, b3.World_HashState(s.world), h0)
}

// The first touching contact's first manifold point's normal impulse (white box).
img_contact_impulse :: proc(id: b3.WorldId) -> ^f32 {
	world := b3.GetWorldFromId(id)
	for i in 0 ..< world.contacts.count {
		c := &world.contacts.data[i]
		if c.contactId == i &&
		   c.manifoldCount > 0 &&
		   c.manifolds[0].pointCount > 0 &&
		   c.manifolds[0].points[0].normalImpulse != 0 {
			return &c.manifolds[0].points[0].normalImpulse
		}
	}
	return nil
}
