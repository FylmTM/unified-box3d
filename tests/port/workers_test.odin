// Deterministic multithreading (SPEC.md, Phase 2 item 5): one scene stepped with 1, 2, 4
// and 8 workers of the built-in scheduler gives bit-identical bodies and events every tick, and identical
// canonical world images (World_HashState) every MT_IMAGE_EVERY ticks. The scene runs every parallel path
// of the step: a pile of 512 boxes (wide contacts, graph colouring, several solver blocks per stage), a plate carrying more contacts than there are colours (the overflow
// constraints), spheres, capsules and hulls on a height field, joint chains (revolute, spherical, weld,
// prismatic, distance, motor), a stack that falls asleep and is woken by a falling ball, a sensor, a
// mesh ground, bullets (continuous against dynamic bodies, spawned mid-run) and a fast non-bullet.
// About 0.75 s alone (1 worker ~0.3 s, then 2, 4, 8), most of it the pile's first two seconds.
package test_box3d_port

import "core:testing"

import b3 "../../box3d"

MT_TICKS :: #config(UNIFIED_MT_TICKS, 120) // a longer soak: -define:UNIFIED_MT_TICKS=n
MT_WORKERS := [?]i32{1, 2, 4, 8}
MT_SLEPT_BY :: 60 // the side stack sleeps by this tick; the ball wakes it later
MT_IMAGE_EVERY :: 8 // ticks between World_HashState comparisons (an image costs more than a step's hashing)

// What one tick must reproduce: bodies (pose and velocities as raw bits), contact begin/end/hit events,
// sensor begin/end events, body move events and joint events, each in Box3D's order (ids leave out the
// world index), and every MT_IMAGE_EVERY ticks the whole world's canonical image hash.
Mt_Tick :: struct {
	bodies, contacts, sensors, moves, joints, image: u64,
}

Mt_Run :: struct {
	ticks:     [MT_TICKS]Mt_Tick,
	overflow:  i32, // most contacts the overflow colour held on one tick
	events:    [4]i32, // contact begin, hit, sensor begin and joint events over the run
	slept:     bool, // the side stack fell asleep before the ball woke it
	woke:      bool,
	scheduled: bool, // the world runs that many workers on the built-in scheduler
}

Mt_Geometry :: struct {
	mesh:   ^b3.MeshData,
	height: ^b3.HeightFieldData,
}

Mt_Scene :: struct {
	world:  b3.WorldId,
	bodies: [dynamic]b3.BodyId,
	sd:     b3.ShapeDef,
	stack:  b3.BodyId, // the top box of the stack that sleeps
}

mt_shape_def :: proc() -> b3.ShapeDef {
	sd := b3.DefaultShapeDef()
	sd.density = 1
	sd.enableContactEvents = true
	sd.enableHitEvents = true
	sd.enableSensorEvents = true
	return sd
}

mt_body :: proc(s: ^Mt_Scene, type: b3.BodyType, pos: b3.Vec3, q := b3.Quat_identity, bullet := false) -> b3.BodyId {
	bd := b3.DefaultBodyDef()
	bd.type = type
	bd.position = pos
	bd.rotation = q
	bd.isBullet = bullet
	id := b3.CreateBody(s.world, &bd)
	if type != .staticBody {
		append(&s.bodies, id)
	}
	return id
}

mt_box :: proc(s: ^Mt_Scene, type: b3.BodyType, pos, half: b3.Vec3, q := b3.Quat_identity) -> b3.BodyId {
	id := mt_body(s, type, pos, q)
	hull := b3.MakeBoxHull(half.x, half.y, half.z)
	b3.CreateHullShape(id, &s.sd, &hull.base)
	return id
}

// A deterministic jitter in [-1, 1) from an integer (no transcendentals, no global state).
mt_jitter :: proc(i: int) -> f32 {
	h := u32(i) * 2654435761
	h ~= h >> 15
	h *= 2246822519
	h ~= h >> 13
	return f32(h & 0xffff) / 32768 - 1
}

mt_bullet :: proc(s: ^Mt_Scene, pos, vel: b3.Vec3) {
	id := mt_body(s, .dynamicBody, pos, bullet = true)
	sphere := b3.Sphere{{0, 0, 0}, 0.1}
	b3.CreateSphereShape(id, &s.sd, &sphere)
	b3.Body_SetLinearVelocity(id, vel)
}

// Joins a link below `a` (its frame at `ya` on a) to the link `b` (its frame at its top).
mt_joint_base :: proc(base: ^b3.JointDef, a, b: b3.BodyId, ya: f32) {
	base.bodyIdA = a
	base.bodyIdB = b
	base.localFrameA.p = {0, ya, 0}
	base.localFrameB.p = {0, 0.45, 0}
	base.forceThreshold = 40 // joint events
}

// A chain of box links hanging from `top` (static anchor when hung, else free), its joints cycling
// through the kinds.
mt_chain :: proc(s: ^Mt_Scene, top: b3.Vec3, links: int, seed: int, hung: bool) {
	prev := hung ? mt_body(s, .staticBody, top) : b3.BodyId{}
	for i in 0 ..< links {
		pos := b3.Vec3{top.x, top.y - 0.45 - f32(i) * 0.9, top.z}
		link := mt_box(s, .dynamicBody, pos, {0.15, 0.4, 0.15})
		if prev == {} {
			prev = link
			continue
		}
		ya: f32 = (hung && i == 0) ? 0 : -0.45
		switch (i + seed) % 5 {
		case 0:
			jd := b3.DefaultRevoluteJointDef()
			mt_joint_base(&jd.base, prev, link, ya)
			jd.enableLimit = true
			jd.lowerAngle, jd.upperAngle = -1, 1
			b3.CreateRevoluteJoint(s.world, &jd)
		case 1:
			jd := b3.DefaultSphericalJointDef()
			mt_joint_base(&jd.base, prev, link, ya)
			jd.enableConeLimit = true
			jd.coneAngle = 0.7
			b3.CreateSphericalJoint(s.world, &jd)
		case 2:
			jd := b3.DefaultWeldJointDef()
			mt_joint_base(&jd.base, prev, link, ya)
			jd.angularHertz, jd.angularDampingRatio = 4, 0.5
			b3.CreateWeldJoint(s.world, &jd)
		case 3:
			jd := b3.DefaultPrismaticJointDef()
			mt_joint_base(&jd.base, prev, link, ya)
			jd.enableLimit = true
			jd.lowerTranslation, jd.upperTranslation = -0.1, 0.1
			jd.enableSpring = true
			jd.hertz, jd.dampingRatio = 3, 0.3
			b3.CreatePrismaticJoint(s.world, &jd)
		case 4:
			jd := b3.DefaultDistanceJointDef()
			mt_joint_base(&jd.base, prev, link, ya)
			jd.length = 0.1
			jd.enableSpring = true
			jd.hertz, jd.dampingRatio = 5, 0.5
			b3.CreateDistanceJoint(s.world, &jd)
		}
		prev = link
	}
	b3.Body_SetLinearVelocity(prev, {3 * mt_jitter(seed), 0, 3})
}

mt_scene_make :: proc(s: ^Mt_Scene, g: Mt_Geometry) {
	s.sd = mt_shape_def()
	w := s.world

	// The mesh ground (48 m square) and a bumpy height field beside it.
	ground := mt_body(s, .staticBody, {0, 0, 0})
	b3.CreateMeshShape(ground, &s.sd, g.mesh, {1, 1, 1})
	field := mt_body(s, .staticBody, {-24, 0, 24})
	b3.CreateHeightFieldShape(field, &s.sd, g.height)

	// The pile: 8 layers of 8 x 8 boxes dropped with jitter (512 boxes, tumbling into each other).
	n := 0
	for y in 0 ..< 8 {
		for x in 0 ..< 8 {
			for z in 0 ..< 8 {
				p := b3.Vec3 {
					f32(x) * 1.1 + 0.2 * mt_jitter(n) + f32(y % 2) * 0.4,
					0.5 + f32(y) * 1.3,
					f32(z) * 1.1 + 0.2 * mt_jitter(n + 1000),
				}
				q := b3.MakeQuatFromAxisAngle({0, 1, 0}, 0.4 * mt_jitter(n + 2000))
				mt_box(s, .dynamicBody, p, {0.45, 0.45, 0.45}, q)
				n += 1
			}
		}
	}

	// The overflow: a dynamic plate carrying 49 boxes, more contacts on one body than there are colours.
	mt_box(s, .dynamicBody, {-12, 0.3, -12}, {4, 0.25, 4})
	for x in 0 ..< 7 {
		for z in 0 ..< 7 {
			mt_box(s, .dynamicBody, {-15 + f32(x), 0.9, -15 + f32(z)}, {0.3, 0.3, 0.3})
		}
	}

	// Mixed shapes rained onto the height field: spheres, capsules, hulls.
	pts: [12]b3.Vec3
	for i in 0 ..< 36 {
		p := b3.Vec3{-22 + f32(i % 6) * 2.2, 3 + f32(i / 6) * 1.5, 26 + f32((i * 7) % 6) * 2}
		id := mt_body(s, .dynamicBody, p)
		switch i % 3 {
		case 0:
			sphere := b3.Sphere{{0, 0, 0}, 0.3 + 0.1 * mt_jitter(i)}
			b3.CreateSphereShape(id, &s.sd, &sphere)
		case 1:
			capsule := b3.Capsule{{-0.3, 0, 0}, {0.3, 0, 0}, 0.25}
			b3.CreateCapsuleShape(id, &s.sd, &capsule)
			b3.Body_SetAngularVelocity(id, {0, 2, 1})
		case 2:
			for &q, k in pts {
				q = {0.5 * mt_jitter(i * 31 + k), 0.5 * mt_jitter(i * 37 + k), 0.5 * mt_jitter(i * 41 + k)}
			}
			hull := b3.CreateHull(&pts[0], i32(len(pts)), 16)
			b3.CreateHullShape(id, &s.sd, hull)
			b3.DestroyHull(hull)
		}
	}

	// Joints: two hanging chains among the falling shapes, two free chains dropped onto the pile, a
	// motor joint driving a box over the ground.
	mt_chain(s, {-19, 12, 29}, 8, 0, true)
	mt_chain(s, {-14, 12, 31}, 8, 2, true)
	mt_chain(s, {2, 14, 3}, 10, 1, false)
	mt_chain(s, {5, 16, 5}, 10, 3, false)
	driven := mt_box(s, .dynamicBody, {10, 0.5, -10}, {0.5, 0.5, 0.5})
	md := b3.DefaultMotorJointDef()
	md.base.bodyIdA = ground
	md.base.bodyIdB = driven
	md.base.localFrameA = b3.Transform{{10, 0.5, -10}, b3.Quat_identity}
	md.linearVelocity = {1, 0, 0}
	md.angularVelocity = {0, 2, 0}
	md.maxVelocityForce, md.maxVelocityTorque = 50, 50
	b3.CreateMotorJoint(w, &md)

	// Sleep and wake: a stack far away settles and sleeps; a ball dropped from high wakes it.
	for i in 0 ..< 4 {
		s.stack = mt_box(s, .dynamicBody, {18, 0.5 + f32(i), 18}, {0.5, 0.5, 0.5})
	}
	ball := mt_body(s, .dynamicBody, {18, 12, 18})
	sphere := b3.Sphere{{0, 0, 0}, 0.5}
	b3.CreateSphereShape(ball, &s.sd, &sphere)

	// A sensor over a corner of the pile.
	sensor := mt_body(s, .staticBody, {2, 3, 2})
	sensor_def := s.sd
	sensor_def.isSensor = true
	sensor_hull := b3.MakeBoxHull(3, 3, 3)
	b3.CreateHullShape(sensor, &sensor_def, &sensor_hull.base)

	// Continuous: four bullets into the pile, a fast non-bullet into the mesh.
	mt_bullet(s, {-10, 1, 4}, {150, 0, 0})
	mt_bullet(s, {20, 2, 3}, {-150, 0, 0})
	mt_bullet(s, {4, 1.5, -10}, {0, 0, 150})
	mt_bullet(s, {3, 2.5, 20}, {0, 0, -150})
	fast := mt_box(s, .dynamicBody, {12, 20, 12}, {0.2, 0.2, 0.2})
	b3.Body_SetLinearVelocity(fast, {0, -200, 0})
}

mt_id :: proc(buf: ^[dynamic]u8, index1: i32, generation: u32) {
	v := [2]u32{u32(index1), generation}
	append(buf, ..([^]u8)(&v)[:size_of(v)])
}

mt_f32s :: proc(buf: ^[dynamic]u8, vals: []f32) {
	append(buf, ..([^]u8)(raw_data(vals))[:len(vals) * size_of(f32)])
}

mt_tick :: proc(s: ^Mt_Scene) -> (t: Mt_Tick) {
	buf := make([dynamic]u8, context.temp_allocator)
	t.bodies = state_hash(s.bodies[:])

	ce := b3.World_GetContactEvents(s.world)
	for e in ce.beginEvents[:ce.beginCount] {
		mt_id(&buf, e.shapeIdA.index1, u32(e.shapeIdA.generation))
		mt_id(&buf, e.shapeIdB.index1, u32(e.shapeIdB.generation))
		mt_id(&buf, e.contactId.index1, e.contactId.generation)
	}
	for e in ce.endEvents[:ce.endCount] {
		mt_id(&buf, e.shapeIdA.index1, u32(e.shapeIdA.generation))
		mt_id(&buf, e.shapeIdB.index1, u32(e.shapeIdB.generation))
		mt_id(&buf, e.contactId.index1, e.contactId.generation)
	}
	for e in ce.hitEvents[:ce.hitCount] {
		mt_id(&buf, e.shapeIdA.index1, u32(e.shapeIdA.generation))
		mt_id(&buf, e.shapeIdB.index1, u32(e.shapeIdB.generation))
		mt_id(&buf, e.contactId.index1, e.contactId.generation)
		mt_f32s(&buf, {e.point.x, e.point.y, e.point.z, e.normal.x, e.normal.y, e.normal.z, e.approachSpeed})
	}
	counts := [3]i32{ce.beginCount, ce.endCount, ce.hitCount}
	append(&buf, ..([^]u8)(&counts)[:size_of(counts)])
	t.contacts = hash_bytes(buf[:])

	clear(&buf)
	se := b3.World_GetSensorEvents(s.world)
	for e in se.beginEvents[:se.beginCount] {
		mt_id(&buf, e.sensorShapeId.index1, u32(e.sensorShapeId.generation))
		mt_id(&buf, e.visitorShapeId.index1, u32(e.visitorShapeId.generation))
	}
	for e in se.endEvents[:se.endCount] {
		mt_id(&buf, e.sensorShapeId.index1, u32(e.sensorShapeId.generation))
		mt_id(&buf, e.visitorShapeId.index1, u32(e.visitorShapeId.generation))
	}
	scounts := [2]i32{se.beginCount, se.endCount}
	append(&buf, ..([^]u8)(&scounts)[:size_of(scounts)])
	t.sensors = hash_bytes(buf[:])

	clear(&buf)
	be := b3.World_GetBodyEvents(s.world)
	for e in be.moveEvents[:be.moveCount] {
		mt_id(&buf, e.bodyId.index1, u32(e.bodyId.generation))
		p, q := e.transform.p, e.transform.q
		mt_f32s(&buf, {p.x, p.y, p.z, q.v.x, q.v.y, q.v.z, q.s, e.fellAsleep ? 1 : 0})
	}
	t.moves = hash_bytes(buf[:])

	clear(&buf)
	je := b3.World_GetJointEvents(s.world)
	for e in je.jointEvents[:je.count] {
		mt_id(&buf, e.jointId.index1, u32(e.jointId.generation))
	}
	t.joints = hash_bytes(buf[:])
	return
}

mt_run :: proc(workers: i32, g: Mt_Geometry, run: ^Mt_Run) {
	w := world_make(int(workers))
	defer world_destroy(w)
	s := Mt_Scene {
		world = w,
	}
	defer delete(s.bodies)
	mt_scene_make(&s, g)
	world := b3.GetWorldFromId(s.world)
	run.scheduled = world.workerCount == workers && (workers == 1) == (world.scheduler == nil)
	for k in 0 ..< MT_TICKS {
		if k == MT_TICKS / 2 { 	// bodies created mid-run: a second volley of bullets
			mt_bullet(&s, {-10, 4, 2}, {160, -10, 0})
			mt_bullet(&s, {2, 4, -10}, {0, -10, 160})
		}
		b3.World_Step(s.world, DT, SUBSTEPS)
		run.ticks[k] = mt_tick(&s)
		if k % MT_IMAGE_EVERY == MT_IMAGE_EVERY - 1 {
			run.ticks[k].image = b3.World_HashState(s.world)
		}
		run.overflow = max(run.overflow, world.constraintGraph.colors[b3.OVERFLOW_INDEX].contacts.count)
		ce := b3.World_GetContactEvents(s.world)
		se, je := b3.World_GetSensorEvents(s.world), b3.World_GetJointEvents(s.world)
		run.events += {ce.beginCount, ce.hitCount, se.beginCount, je.count}
		if k == MT_SLEPT_BY {
			run.slept = !b3.Body_IsAwake(s.stack)
		}
		if run.slept && b3.Body_IsAwake(s.stack) {
			run.woke = true
		}
		free_all(context.temp_allocator)
	}
}

@(test)
workers_step_identically :: proc(t: ^testing.T) {
	mesh := b3.CreateGridMesh(48, 48, 1, 0, true)
	defer b3.DestroyMesh(mesh)
	heights: [16 * 16]f32
	materials: [15 * 15]u8
	for &h, i in heights {
		x, z := f32(i % 16) - 7.5, f32(i / 16) - 7.5
		h = 0.02 * (x * x - z * z) + 0.3 * mt_jitter(i) + 1
	}
	hd := b3.HeightFieldDef {
		heights             = &heights[0],
		materialIndices     = &materials[0],
		scale               = {1, 1, 1},
		countX              = 16,
		countZ              = 16,
		globalMinimumHeight = -16,
		globalMaximumHeight = 16,
	}
	height := b3.CreateHeightField(&hd)
	defer b3.DestroyHeightField(height)
	g := Mt_Geometry{mesh, height}

	ref := new(Mt_Run)
	defer free(ref)
	mt_run(1, g, ref)
	// The scene does run the paths it is meant to.
	testing.expectf(t, ref.overflow > 0, "no overflow constraints")
	testing.expectf(
		t,
		ref.events[0] > 0 && ref.events[1] > 0 && ref.events[2] > 0 && ref.events[3] > 0,
		"events %v",
		ref.events,
	)
	testing.expect(t, ref.slept && ref.woke, "the side stack should sleep, then wake")
	testing.expect(t, ref.scheduled, "1 worker: not on the caller's thread")

	run := new(Mt_Run)
	defer free(run)
	for workers in MT_WORKERS[1:] {
		run^ = {}
		mt_run(workers, g, run)
		testing.expectf(t, run.scheduled, "%d workers: not on the built-in scheduler", workers)
		for k in 0 ..< MT_TICKS {
			a, b := ref.ticks[k], run.ticks[k]
			if a != b {
				testing.expectf(
					t,
					false,
					"%d workers diverge at tick %d: bodies %v, contact events %v, sensor events %v, move events %v, joint events %v, image %v",
					workers,
					k,
					a.bodies == b.bodies,
					a.contacts == b.contacts,
					a.sensors == b.sensors,
					a.moves == b.moves,
					a.joints == b.joints,
					a.image == b.image,
				)
				break
			}
		}
	}
}
