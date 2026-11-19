// Images from anyone (a late joiner's snapshot can come from a hostile peer): World_LoadImage on damaged and made-up
// bytes either refuses them or loads a world that steps, saves and restores safely. Flips, cuts, random bytes and
// splices of real images (img_scene: every geometry kind, joints, contacts, a sensor, a sleeping stack).
//
// `./b3 test fuzz` builds this package with asserts off and bounds checks on: an out-of-bounds access still fails it,
// while Box3D's numeric asserts (a velocity that blew up) don't count. A world loaded from strange but well-formed
// numbers may simulate badly, as an honest world can; it must never touch memory it doesn't own.
package test_box3d_fuzz

import "core:fmt"
@(require) import "core:os" // FUZZ_TRACE writes each case
import "core:sync"
import "core:testing"

import b3 "../../box3d"

DT :: f32(1.0 / 64)
SUBSTEPS :: 4

@(private = "file")
world_mutex: sync.Mutex

world_make :: proc() -> b3.WorldId {
	sync.guard(&world_mutex)
	wd := b3.DefaultWorldDef()
	return b3.CreateWorld(&wd)
}

world_destroy :: proc(w: b3.WorldId) {
	sync.guard(&world_mutex)
	b3.DestroyWorld(w)
}

step :: proc(w: b3.WorldId, n: int) {
	for _ in 0 ..< n {
		b3.World_Step(w, DT, SUBSTEPS)
	}
}

img_bytes :: proc(st: ^b3.WorldState) -> []u8 {
	return b3.WorldState_GetData(st)[:b3.WorldState_GetSize(st)]
}


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

	// A custom hull falling onto a compound of two spheres and a capsule (compounds are static-only)
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
	cb := img_body(w, .staticBody, {2, 0.3, 2})
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

// With -define:FUZZ_LOAD_ONLY=true, only count what loads (nothing is stepped): the numbers for a loader that
// accepts what it shouldn't.
FUZZ_LOAD_ONLY :: #config(FUZZ_LOAD_ONLY, false)
// -define:FUZZ_LONG=true: every byte of the first seed flipped, a dense sample of the others, more of everything, and
// 120 ticks stepped (about 90 s, debug build); by default a sample that runs in seconds.
FUZZ_LONG :: #config(FUZZ_LONG, false)
when FUZZ_LONG {
	FUZZ_STEPS :: 120 // ticks a world that loaded is stepped
	FLIP_STRIDE :: [3]int{1, 7, 7}
	MULTI_CASES :: 3000
	CUT_STRIDE :: [3]int{1, 13, 13}
	RANDOM_CASES :: 2000
	SPLICE_CASES :: 1000
} else {
	FUZZ_STEPS :: 16
	FLIP_STRIDE :: [3]int{17, 23, 29}
	MULTI_CASES :: 600
	CUT_STRIDE :: [3]int{61, 67, 71}
	RANDOM_CASES :: 300
	SPLICE_CASES :: 200
}
FUZZ_TRACE :: #config(FUZZ_TRACE, false) // print each image that loads before stepping it (to find a crash's case)

@(private = "file")
fuzz_case: string

@(private = "file")
Fuzz_Rng :: struct {
	state: u64,
}

@(private = "file")
fuzz_next :: proc(rng: ^Fuzz_Rng) -> u64 {
	rng.state += 0x9E3779B97F4A7C15
	z := rng.state
	z = (z ~ (z >> 30)) * 0xBF58476D1CE4E5B9
	z = (z ~ (z >> 27)) * 0x94D049BB133111EB
	return z ~ (z >> 31)
}

@(private = "file")
fuzz_below :: proc(rng: ^Fuzz_Rng, n: int) -> int {
	return int(fuzz_next(rng) % u64(n))
}

@(private = "file")
Fuzz_Counts :: struct {
	tried, loaded: int,
}

// Loads `bytes` into a fresh world; if it loads, steps it, saves an image, snapshots and restores. Counts.
@(private = "file")
fuzz_try :: proc(bytes: []u8, counts: ^Fuzz_Counts, state: ^b3.WorldState) {
	counts.tried += 1
	when FUZZ_TRACE {
		fmt.eprintfln("fuzz: loading %s", fuzz_case)
		_ = os.write_entire_file(".out/tests/fuzz_case.bin", bytes)
	}
	world := world_make()
	defer world_destroy(world)
	if !b3.World_LoadImage(world, bytes) {
		when FUZZ_TRACE {
			fmt.eprintfln("fuzz: refused")
		}
		return
	}
	counts.loaded += 1
	when FUZZ_TRACE {
		fmt.eprintfln("fuzz: stepping %s", fuzz_case)
	}
	when !FUZZ_LOAD_ONLY {
		for _ in 0 ..< FUZZ_STEPS / 2 {
			b3.World_Step(world, DT, SUBSTEPS)
		}
		snapshot: b3.WorldSnapshot
		b3.World_SaveSnapshot(world, &snapshot)
		for _ in 0 ..< FUZZ_STEPS / 2 {
			b3.World_Step(world, DT, SUBSTEPS)
		}
		b3.World_RestoreSnapshot(world, &snapshot)
		b3.DestroyWorldSnapshot(&snapshot)
		b3.World_Step(world, DT, SUBSTEPS)
		_ = b3.World_HashState(world)
		_ = b3.World_SaveImage(world, state, true)
	}
}

// Images of the scene at a few ticks (with their geometry), the fuzzer's seeds.
@(private = "file")
fuzz_seeds :: proc(scene: ^Img_Scene) -> (seeds: [3][]u8) {
	state := b3.CreateWorldState()
	defer b3.DestroyWorldState(state)
	for &seed, k in seeds {
		step(scene.world, 40 + 80 * k) // moving, then a stack asleep
		size := b3.World_SaveImage(scene.world, state, true)
		assert(size > 0)
		seed = clone(img_bytes(state))
	}
	return
}

@(private = "file")
clone :: proc(bytes: []u8) -> []u8 {
	out := make([]u8, len(bytes))
	copy(out, bytes)
	return out
}

// The geometry blobs an image carries (with its geometry): each blob's header offset in the image, then its bytes.
@(private = "file")
Fuzz_Blob :: struct {
	header, bytes, count: int,
	kind:                 b3.GeometryKind,
}

@(private = "file")
fuzz_blobs :: proc(image: []u8) -> (blobs: [dynamic]Fuzz_Blob) {
	at := 16 // the snapshot header (magic, version, layout hash, flags)
	count := int((^i32le)(&image[at])^)
	at += 4
	for _ in 0 ..< count {
		header := (^b3.ImageGeometryHeader)(&image[at])
		append(&blobs, Fuzz_Blob{at, at + 16, int(header.byteCount), b3.GeometryKind(header.kind)})
		at += 16 + int(header.byteCount)
	}
	return
}

// What a crafted blob would carry: its content hash recomputed (world_image.odin, BakedContentHash), so it passes the
// hash check and only the blob's own validation stands between it and the world.
@(private = "file")
fuzz_rehash :: proc(image: []u8, blob: Fuzz_Blob) {
	bytes := image[blob.bytes:][:blob.count]
	hash: u64
	if blob.kind == .geometryCompound {
		hash = b3.Hash64NonZero(raw_data(bytes), i32(blob.count))
	} else {
		field := (^u64le)(&bytes[8])
		saved := field^
		field^ = 0
		hash = b3.Hash64NonZero(raw_data(bytes), i32(blob.count))
		field^ = saved
	}
	(^b3.ImageGeometryHeader)(&image[blob.header]).contentHash = hash
}

@(test)
image_load_is_safe_on_any_bytes :: proc(t: ^testing.T) {
	scene: Img_Scene
	img_scene_make(&scene)
	defer img_scene_destroy(&scene)
	seeds := fuzz_seeds(&scene)
	defer for seed in seeds {
		delete(seed)
	}
	state := b3.CreateWorldState()
	defer b3.DestroyWorldState(state)
	rng := Fuzz_Rng{0xD11A}
	work := make([]u8, len(seeds[2]) * 2)
	defer delete(work)

	// Every seed loads and steps as it is.
	for seed in seeds {
		counts: Fuzz_Counts
		fuzz_try(seed, &counts, state)
		testing.expect_value(t, counts.loaded, 1)
	}

	flip_stride, cut_stride := FLIP_STRIDE, CUT_STRIDE
	EDGES := [?]u32{0, 1, 0xFFFFFFFF, 0x7FFFFFFF, 0x80000000, 0x7FC00000, 0xFF800000, 1 << 20, 1 << 30}
	// One byte flipped (a random non-zero xor) at every FLIP_STRIDE-th offset.
	flips: Fuzz_Counts
	for seed, k in seeds {
		for offset := 0; offset < len(seed); offset += flip_stride[k] {
			bytes := work[:len(seed)]
			copy(bytes, seed)
			flip := u8(1 + fuzz_below(&rng, 255))
			bytes[offset] ~= flip
			when FUZZ_TRACE {
				fuzz_case = fmt.tprintf("seed %d byte %d ^ %02x", k, offset, flip)
			}
			fuzz_try(bytes, &flips, state)
		}
	}
	// Crafted geometry: a byte of a carried blob (hull, mesh, height field, compound) flipped, or a 32-bit word set to
	// an edge value, and the blob's content hash recomputed.
	blobs: Fuzz_Counts
	for seed, k in seeds {
		list := fuzz_blobs(seed)
		defer delete(list)
		for blob in list {
			stride := FUZZ_LONG ? 1 : 13
			for offset := 0; offset < blob.count; offset += stride {
				bytes := work[:len(seed)]
				copy(bytes, seed)
				if fuzz_below(&rng, 3) == 0 && offset + 4 <= blob.count {
					(^u32le)(&bytes[blob.bytes + offset &~ 3])^ = u32le(EDGES[fuzz_below(&rng, len(EDGES))])
				} else {
					bytes[blob.bytes + offset] ~= u8(1 + fuzz_below(&rng, 255))
				}
				fuzz_rehash(bytes, blob)
				when FUZZ_TRACE {
					fuzz_case = fmt.tprintf("seed %d blob %v at %d offset %d", k, blob.kind, blob.bytes, offset)
				}
				fuzz_try(bytes, &blobs, state)
			}
		}
	}
	// A few bytes at once, and a 32-bit word set to an edge value (an index or count made huge, negative, or -1).
	multi: Fuzz_Counts
	for case_index in 0 ..< MULTI_CASES {
		_ = case_index
		when FUZZ_TRACE {
			fuzz_case = fmt.tprintf("multi %d", case_index)
		}
		seed := seeds[fuzz_below(&rng, len(seeds))]
		bytes := work[:len(seed)]
		copy(bytes, seed)
		if fuzz_below(&rng, 2) == 0 {
			for _ in 0 ..< 1 + fuzz_below(&rng, 8) {
				at := fuzz_below(&rng, len(bytes))
				flip := u8(1 + fuzz_below(&rng, 255))
				bytes[at] ~= flip
				when FUZZ_TRACE {
					fuzz_case = fmt.tprintf("%s, byte %d ^ %02x", fuzz_case, at, flip)
				}
			}
		} else {
			at := fuzz_below(&rng, len(bytes) - 3) &~ 3
			value := EDGES[fuzz_below(&rng, len(EDGES))]
			(^u32le)(&bytes[at])^ = u32le(value)
			when FUZZ_TRACE {
				fuzz_case = fmt.tprintf("%s, u32 at %d = %x", fuzz_case, at, value)
			}
		}
		fuzz_try(bytes, &multi, state)
	}
	// Cuts at every CUT_STRIDE-th length.
	cuts: Fuzz_Counts
	for seed, k in seeds {
		for length := 0; length < len(seed); length += cut_stride[k] {
			when FUZZ_TRACE {
				fuzz_case = fmt.tprintf("cut seed %d at %d", k, length)
			}
			fuzz_try(seed[:length], &cuts, state)
		}
	}
	// Random bytes, alone and after a real header and first part.
	random: Fuzz_Counts
	for i in 0 ..< RANDOM_CASES {
		when FUZZ_TRACE {
			fuzz_case = fmt.tprintf("random %d", i)
		}
		seed := seeds[i % len(seeds)]
		length := 1 + fuzz_below(&rng, len(seed))
		bytes := work[:length]
		keep := i % 2 == 0 ? 0 : fuzz_below(&rng, length)
		copy(bytes[:keep], seed[:keep])
		for &b in bytes[keep:] {
			b = u8(fuzz_next(&rng))
		}
		fuzz_try(bytes, &random, state)
	}
	// Splices: the start of one seed with the rest of another, cut anywhere.
	splices: Fuzz_Counts
	for case_index in 0 ..< SPLICE_CASES {
		_ = case_index
		when FUZZ_TRACE {
			fuzz_case = fmt.tprintf("splice %d", case_index)
		}
		a, b := seeds[fuzz_below(&rng, len(seeds))], seeds[fuzz_below(&rng, len(seeds))]
		at := fuzz_below(&rng, min(len(a), len(b)))
		from := at + fuzz_below(&rng, 64) - 32
		from = clamp(from, 0, len(b))
		bytes := work[:at + len(b) - from]
		copy(bytes[:at], a[:at])
		copy(bytes[at:], b[from:])
		fuzz_try(bytes, &splices, state)
	}
	report :: proc(what: string, counts: Fuzz_Counts) -> string {
		return fmt.tprintf("%s %d of %d loaded", what, counts.loaded, counts.tried)
	}
	fmt.printfln(
		"image fuzz (seeds of %d, %d and %d bytes): %s; %s; %s; %s; %s; %s",
		len(seeds[0]),
		len(seeds[1]),
		len(seeds[2]),
		report("flips", flips),
		report("crafted geometry", blobs),
		report("multi", multi),
		report("cuts", cuts),
		report("random", random),
		report("splices", splices),
	)
	testing.expect_value(t, cuts.loaded, 0) // an image cut short never loads
}
