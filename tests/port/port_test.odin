// Tests of what the port adds to upstream (SPEC.md): deterministic multithreading (workers_test.odin), canonical
// images and late join (image_test.odin), in-process snapshots (below). Run with `./b3 test`; the package runs
// apart from the upstream tests (those change Box3D's globals).
package test_box3d_port

import "core:sync"
import "core:testing"

import b3 "../../box3d"

DT :: f32(1.0 / 64)
SUBSTEPS :: 4

// b3CreateWorld and b3DestroyWorld touch Box3D's global world table, and Odin runs tests on several threads: every
// world is created and destroyed under this lock (stepping needs no lock, each world is its own).
@(private = "file")
world_mutex: sync.Mutex

// A world from DefaultWorldDef with `workers` workers (1: stepped on the caller's thread).
world_make :: proc(workers := 1) -> b3.WorldId {
	sync.guard(&world_mutex)
	wd := b3.DefaultWorldDef()
	wd.workerCount = u32(max(1, workers))
	return b3.CreateWorld(&wd)
}

world_destroy :: proc(w: b3.WorldId) {
	sync.guard(&world_mutex)
	b3.DestroyWorld(w)
}

hash_bytes :: proc(b: []u8) -> u64 {
	return b3.rapidhash(raw_data(b), len(b))
}

// The bodies' poses and velocities, as raw bits.
state_hash :: proc(bodies: []b3.BodyId) -> u64 {
	buf := make([dynamic]u8, context.temp_allocator)
	for id in bodies {
		p := b3.Body_GetPosition(id)
		q := b3.Body_GetRotation(id)
		v := b3.Body_GetLinearVelocity(id)
		a := b3.Body_GetAngularVelocity(id)
		vals := [13]f32{p.x, p.y, p.z, q.v.x, q.v.y, q.v.z, q.s, v.x, v.y, v.z, a.x, a.y, a.z}
		append(&buf, ..([^]u8)(&vals)[:size_of(vals)])
	}
	return hash_bytes(buf[:])
}

step :: proc(w: b3.WorldId, n: int) {
	for _ in 0 ..< n {
		b3.World_Step(w, DT, SUBSTEPS)
	}
}

// A snapshot restores the world exactly: after a rollback of any depth, re-stepping gives the same bodies and the
// same canonical hash as the first time, including across a body created and one destroyed inside the window.
@(test)
snapshot_rollback_is_exact :: proc(t: ^testing.T) {
	s: Img_Scene
	img_scene_make(&s)
	defer img_scene_destroy(&s)
	step(s.world, 20)

	snap: b3.WorldSnapshot
	defer b3.DestroyWorldSnapshot(&snap)
	b3.World_SaveSnapshot(s.world, &snap)
	h0 := b3.World_HashState(s.world)

	run :: proc(w: b3.WorldId) -> (hashes: [40]u64) {
		extra: b3.BodyId
		for k in 0 ..< len(hashes) {
			if k == 5 {
				extra = img_box(w, .dynamicBody, {0, 6, 0}, {0.4, 0.4, 0.4})
			}
			if k == 25 {
				b3.DestroyBody(extra)
			}
			step(w, 1)
			hashes[k] = b3.World_HashState(w)
		}
		return
	}
	first := run(s.world)
	for depth in ([?]int{0, 1, 40}) {
		testing.expect(t, b3.World_RestoreSnapshot(s.world, &snap))
		testing.expect_value(t, b3.World_HashState(s.world), h0)
		step(s.world, depth)
		if depth > 0 {
			testing.expect(t, b3.World_RestoreSnapshot(s.world, &snap))
		}
		again := run(s.world)
		for k in 0 ..< len(first) {
			if !testing.expectf(t, first[k] == again[k], "rollback after %d ticks: tick %d differs", depth, k) {
				return
			}
		}
	}
}
