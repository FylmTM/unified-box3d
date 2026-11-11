// The generated bindings against the built shared library (`./b3 test` builds it first): worlds, bodies with hull,
// sphere and capsule shapes, a joint, stepping, a ray cast and a friction mix through C callbacks, a snapshot round
// trip, a canonical image loaded into a second world, and the hash. layout.odin (generated) checks every public
// struct and enum against C's layout at compile time.
package test_box3d_bindings

import "core:sync"
import "core:testing"

import box3d "../../bindings"

DT :: f32(1.0 / 60)
SUBSTEPS :: 4

// The library's world table is global; Odin runs tests on several threads.
@(private = "file")
world_mutex: sync.Mutex

world_make :: proc() -> box3d.WorldId {
	sync.guard(&world_mutex)
	wd := box3d.DefaultWorldDef()
	return box3d.CreateWorld(&wd)
}

world_destroy :: proc(w: box3d.WorldId) {
	sync.guard(&world_mutex)
	box3d.DestroyWorld(w)
}

Scene :: struct {
	world:                   box3d.WorldId,
	ground, box, ball, pill: box3d.BodyId,
	arm:                     box3d.BodyId,
	joint:                   box3d.JointId,
}

scene_make :: proc() -> (s: Scene) {
	s.world = world_make()
	w := s.world

	bd := box3d.DefaultBodyDef()
	bd.position = {0, -0.5, 0}
	s.ground = box3d.CreateBody(w, &bd)
	sd := box3d.DefaultShapeDef()
	ground := box3d.MakeBoxHull(20, 0.5, 20)
	box3d.CreateHullShape(s.ground, &sd, &ground.base)

	dynamic_body :: proc(w: box3d.WorldId, pos: box3d.Vec3) -> box3d.BodyId {
		bd := box3d.DefaultBodyDef()
		bd.type = .dynamicBody
		bd.position = pos
		return box3d.CreateBody(w, &bd)
	}
	sd.density = 1
	s.box = dynamic_body(w, {0, 2, 0})
	cube := box3d.MakeBoxHull(0.5, 0.5, 0.5)
	box3d.CreateHullShape(s.box, &sd, &cube.base)

	s.ball = dynamic_body(w, {3, 4, 0})
	sphere := box3d.Sphere{{0, 0, 0}, 0.5}
	box3d.CreateSphereShape(s.ball, &sd, &sphere)

	s.pill = dynamic_body(w, {-3, 3, 0})
	capsule := box3d.Capsule{{-0.5, 0, 0}, {0.5, 0, 0}, 0.3}
	box3d.CreateCapsuleShape(s.pill, &sd, &capsule)

	// A pendulum: a box hinged to the ground at (0, 6, 4).
	s.arm = dynamic_body(w, {1, 6, 4})
	rod := box3d.MakeBoxHull(0.5, 0.1, 0.1)
	box3d.CreateHullShape(s.arm, &sd, &rod.base)
	jd := box3d.DefaultRevoluteJointDef()
	jd.base.bodyIdA = s.ground
	jd.base.bodyIdB = s.arm
	jd.base.localFrameA.p = {0, 6.5, 4}
	jd.base.localFrameB.p = {-1, 0, 0}
	s.joint = box3d.CreateRevoluteJoint(w, &jd)
	return
}

step :: proc(w: box3d.WorldId, n: int) {
	for _ in 0 ..< n {
		box3d.World_Step(w, DT, SUBSTEPS)
	}
}

@(test)
bodies_shapes_joints_step :: proc(t: ^testing.T) {
	s := scene_make()
	defer world_destroy(s.world)
	testing.expect(t, box3d.World_IsValid(s.world))
	testing.expect_value(t, box3d.Body_GetType(s.box), box3d.BodyType.dynamicBody)
	testing.expect_value(t, box3d.Joint_GetType(s.joint), box3d.JointType.revoluteJoint)
	arm0 := box3d.Body_GetPosition(s.arm)

	step(s.world, 120)

	// Everything fell onto the ground (top at y = 0) and rests there.
	box := box3d.Body_GetPosition(s.box)
	testing.expectf(t, abs(box.y - 0.5) < 0.05, "box at %v", box)
	ball := box3d.Body_GetPosition(s.ball)
	testing.expectf(t, abs(ball.y - 0.5) < 0.05, "ball at %v", ball)
	pill := box3d.Body_GetPosition(s.pill)
	testing.expectf(t, abs(pill.y - 0.3) < 0.05, "capsule at %v", pill)
	// The pendulum swung but stays on its hinge: its end is 1 from the pivot.
	arm := box3d.Body_GetPosition(s.arm)
	testing.expect(t, arm != arm0)
	pivot := box3d.Vec3{0, 6, 4}
	d := box3d.Sub(arm, pivot)
	testing.expectf(t, abs(box3d.Length(d) - 1) < 0.01, "arm %v is %v from the pivot", arm, box3d.Length(d))
	testing.expect(t, box3d.World_GetAwakeBodyCount(s.world) >= 1)
}

Ray_Hits :: struct {
	count:    int,
	closest:  f32,
	shape:    box3d.ShapeId,
	material: u64,
}

// A C callback through the library: keeps the closest hit.
ray_callback :: proc "c" (
	shapeId: box3d.ShapeId,
	point: box3d.Pos,
	normal: box3d.Vec3,
	fraction: f32,
	userMaterialId: u64,
	triangleIndex: i32,
	childIndex: i32,
	ctx: rawptr,
) -> f32 {
	hits := (^Ray_Hits)(ctx)
	hits.count += 1
	if fraction < hits.closest {
		hits.closest = fraction
		hits.shape = shapeId
	}
	return fraction
}

friction_calls: i32

// A friction mix through the library (from its worker: an atomic count).
friction_callback :: proc "c" (frictionA: f32, userMaterialIdA: u64, frictionB: f32, userMaterialIdB: u64) -> f32 {
	sync.atomic_add(&friction_calls, 1)
	return frictionA < frictionB ? frictionA : frictionB
}

@(test)
callbacks :: proc(t: ^testing.T) {
	s := scene_make()
	defer world_destroy(s.world)
	box3d.World_SetFrictionCallback(s.world, friction_callback)
	step(s.world, 120)
	testing.expect(t, sync.atomic_load(&friction_calls) > 0, "the friction callback never ran")

	// Straight down onto the box: it is the closest hit, on its top face.
	hits := Ray_Hits {
		closest = 2,
	}
	box := box3d.Body_GetPosition(s.box)
	origin := box3d.Pos{box.x, 10, box.z}
	box3d.World_CastRay(s.world, origin, {0, -20, 0}, box3d.DefaultQueryFilter(), ray_callback, &hits)
	testing.expect(t, hits.count >= 1)
	testing.expectf(t, box3d.Shape_GetBody(hits.shape) == s.box, "hit %v", hits.shape)
	testing.expectf(t, abs(10 - 20 * hits.closest - (box.y + 0.5)) < 0.01, "fraction %v", hits.closest)
	closest := box3d.World_CastRayClosest(s.world, origin, {0, -20, 0}, box3d.DefaultQueryFilter())
	testing.expect(t, closest.hit)
	testing.expect_value(t, closest.fraction, hits.closest)
}

@(test)
snapshot_round_trip :: proc(t: ^testing.T) {
	s := scene_make()
	defer world_destroy(s.world)
	step(s.world, 30)
	snap: box3d.WorldSnapshot
	defer box3d.DestroyWorldSnapshot(&snap)
	box3d.World_SaveSnapshot(s.world, &snap)
	testing.expect(t, snap.size > 0 && snap.data != nil)
	h0 := box3d.World_HashState(s.world)

	first: [30]u64
	for &h in first {
		step(s.world, 1)
		h = box3d.World_HashState(s.world)
	}
	testing.expect(t, box3d.World_RestoreSnapshot(s.world, &snap))
	testing.expect_value(t, box3d.World_HashState(s.world), h0)
	for h, k in first {
		step(s.world, 1)
		if !testing.expectf(t, box3d.World_HashState(s.world) == h, "tick %d after the restore differs", k) {
			return
		}
	}

	// A snapshot from another world is refused.
	other := world_make()
	defer world_destroy(other)
	testing.expect(t, !box3d.World_RestoreSnapshot(other, &snap))
}

@(test)
image_late_join :: proc(t: ^testing.T) {
	s := scene_make()
	defer world_destroy(s.world)
	step(s.world, 45)

	state := box3d.CreateWorldState()
	defer box3d.DestroyWorldState(state)
	size := box3d.World_SaveImage(s.world, state, true)
	testing.expect(t, size > 0)
	testing.expect_value(t, box3d.WorldState_GetSize(state), size)
	bytes := box3d.WorldState_GetData(state)

	late := world_make()
	defer world_destroy(late)
	testing.expect(t, box3d.World_LoadImage(late, bytes, size))
	testing.expect_value(t, box3d.World_HashState(late), box3d.World_HashState(s.world))
	for k in 0 ..< 60 {
		step(s.world, 1)
		step(late, 1)
		a, b := box3d.World_HashState(s.world), box3d.World_HashState(late)
		if !testing.expectf(t, a == b, "tick %d after the late join: %x != %x", k, a, b) {
			return
		}
	}
	// A damaged image is refused.
	testing.expect(t, !box3d.World_LoadImage(late, bytes, 20))

	// The former fork's API: save and restore through a cache, and from bytes.
	cache := box3d.CreateWorldStateCache()
	defer box3d.DestroyWorldStateCache(cache)
	fork := box3d.CreateWorldState()
	defer box3d.DestroyWorldState(fork)
	testing.expect(t, box3d.World_SaveState(s.world, cache, fork) > 0)
	h := box3d.World_HashState(s.world)
	step(s.world, 10)
	testing.expect(t, box3d.World_RestoreState(s.world, cache, fork))
	testing.expect_value(t, box3d.World_HashState(s.world), h)
	step(s.world, 10)
	testing.expect(
		t,
		box3d.World_RestoreStateFromBytes(
			s.world,
			cache,
			box3d.WorldState_GetData(fork),
			box3d.WorldState_GetSize(fork),
		),
	)
	testing.expect_value(t, box3d.World_HashState(s.world), h)
}

@(test)
inline_and_constants :: proc(t: ^testing.T) {
	// Inline math (copied from the port) and the length-scaled constants (over the library's length units).
	testing.expect_value(t, box3d.Dot(box3d.Vec3{1, 2, 3}, box3d.Vec3{4, 5, 6}), 32)
	testing.expect_value(t, box3d.LINEAR_SLOP(), 0.005)
	v := box3d.GetVersion()
	testing.expect(t, v.major >= 0 && v.minor >= 0)
	id := box3d.BodyId {
		index1     = 7,
		world0     = 2,
		generation = 3,
	}
	testing.expect_value(t, box3d.LoadBodyId(box3d.StoreBodyId(id)), id)
}
