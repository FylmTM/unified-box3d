// Ported from box3d test/test_hash.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "core:mem"
import "core:testing"

// Separates the two things a repeated hash can mean. Identical bytes repeating is normal, since
// distinct generator parameters can still bake to the same geometry. Distinct bytes sharing a hash
// is the failure this file exists to catch, so the invariant under test is that different content
// gets different digests, never that different parameters do.
//
// Zero doubles as "no collision", since the hash reserves that value. Results are read back only
// after teardown so a failing test does not also dump a leak and bury which condition broke.
@(private = "file")
HashEntry :: struct {
	hash:      u64,
	byteCount: i32,
	bytes:     [^]u8,
}

@(private = "file")
HashProbe :: struct {
	set:        b3.HashSet,
	entries:    [^]HashEntry,
	count:      i32,
	capacity:   i32,
	collision:  u64,
	duplicates: i32,
	sawZero:    bool,
	overflow:   bool,
}

@(private = "file")
HashResult :: struct {
	collision:  u64,
	count:      i32,
	duplicates: i32,
	sawZero:    bool,
	overflow:   bool,
}

@(private = "file")
ProbeBegin :: proc(capacity: i32) -> HashProbe {
	probe := HashProbe{}
	probe.set = b3.CreateSet(capacity)
	probe.entries = b3.ALLOC(HashEntry, capacity)
	probe.capacity = capacity
	return probe
}

// Pass NULL bytes when the caller builds provably distinct inputs, so any repeat is a collision by
// construction and there is nothing to compare against.
@(private = "file")
ProbeAdd :: proc(probe: ^HashProbe, hash: u64, bytes: rawptr, byteCount: i32) {
	if hash == 0 {
		probe.sawZero = true
	}

	if probe.count == probe.capacity {
		probe.overflow = true
		return
	}

	if b3.AddKey(&probe.set, hash) {
		sameContent := false
		if bytes != nil {
			for i: i32 = 0; i < probe.count; i += 1 {
				if probe.entries[i].hash != hash {
					continue
				}

				sameContent =
					probe.entries[i].byteCount == byteCount &&
					mem.compare_ptrs(probe.entries[i].bytes, bytes, int(byteCount)) == 0
				break
			}
		}

		if sameContent {
			probe.duplicates += 1
		} else if probe.collision == 0 {
			probe.collision = hash
		}
	}

	entry := &probe.entries[probe.count]
	entry.hash = hash
	entry.byteCount = 0
	entry.bytes = nil

	if bytes != nil && byteCount > 0 {
		entry.byteCount = byteCount
		entry.bytes = b3.ALLOC(u8, byteCount)
		mem.copy(entry.bytes, bytes, int(byteCount))
	}

	probe.count += 1
}

@(private = "file")
ProbeEnd :: proc(probe: ^HashProbe) -> HashResult {
	result := HashResult{probe.collision, probe.count, probe.duplicates, probe.sawZero, probe.overflow}

	for i: i32 = 0; i < probe.count; i += 1 {
		if probe.entries[i].bytes != nil {
			b3.FREE(probe.entries[i].bytes, u8, probe.entries[i].byteCount)
		}
	}

	b3.FREE(probe.entries, HashEntry, probe.capacity)
	b3.DestroySet(&probe.set)
	return result
}

// Pins that the upper bytes of each 8 byte word reach the low bits of the hash. Multiply carries
// bits upward only, so a hash that keeps just the low half of its product leaves the low bits blind
// to those bytes, and blobs differing only up there collapse onto a tiny range no finalizer can
// spread back out. Varying the top byte of two words is the sharp case. A pairwise avalanche check
// cannot catch this, since a finalizer scatters any single difference across all bits no matter how
// weak the mixing behind it.
@(private = "file")
HashWordFamily :: proc(t: ^testing.T) -> bool {
	blobSize :: 16
	familySize :: 256 * 256

	probe := ProbeBegin(familySize)

	blob: [blobSize]u8
	for i: i32 = 0; i < blobSize; i += 1 {
		blob[i] = u8(0x5A + i)
	}

	for a: i32 = 0; a < 256; a += 1 {
		for b: i32 = 0; b < 256; b += 1 {
			blob[7] = u8(a)
			blob[15] = u8(b)
			ProbeAdd(&probe, b3.Hash64NonZero(&blob[0], blobSize), nil, 0)
		}
	}

	result := ProbeEnd(&probe)
	ENSURE(t, result.overflow == false) or_return
	ENSURE(t, result.count == familySize) or_return
	ENSURE(t, result.sawZero == false) or_return
	ENSURE(t, result.collision == 0) or_return
	return true
}

// Every input bit must move the digest, at every offset. Flipping one byte in one position is a
// weak check: a mixer can pass it while still being blind to whole regions of a longer blob.
@(private = "file")
HashBitSweep :: proc(t: ^testing.T) -> bool {
	blobSize :: 32
	bitCount :: 8 * blobSize

	probe := ProbeBegin(bitCount + 1)

	blob: [blobSize]u8
	for i: i32 = 0; i < blobSize; i += 1 {
		blob[i] = u8(0xA5 ~ (i * 17))
	}

	ProbeAdd(&probe, b3.Hash64NonZero(&blob[0], blobSize), nil, 0)

	for bit: i32 = 0; bit < bitCount; bit += 1 {
		mask := u8(1 << u32(bit & 7))
		blob[bit >> 3] ~= mask
		ProbeAdd(&probe, b3.Hash64NonZero(&blob[0], blobSize), nil, 0)
		blob[bit >> 3] ~= mask
	}

	result := ProbeEnd(&probe)
	ENSURE(t, result.overflow == false) or_return
	ENSURE(t, result.count == bitCount + 1) or_return
	ENSURE(t, result.sawZero == false) or_return
	ENSURE(t, result.collision == 0) or_return
	return true
}

// Baked blobs carry explicit padding and unused array slots, so long runs of zeros are common and
// content alone cannot separate them. Length has to reach the digest.
@(private = "file")
HashZeroLengths :: proc(t: ^testing.T) -> bool {
	maxLength :: 4096

	zeros := b3.ALLOC(u8, maxLength)
	mem.zero(zeros, maxLength)

	probe := ProbeBegin(maxLength)
	for n: i32 = 0; n < maxLength; n += 1 {
		ProbeAdd(&probe, b3.Hash64NonZero(zeros, n), nil, 0)
	}

	result := ProbeEnd(&probe)
	b3.FREE(zeros, u8, maxLength)

	ENSURE(t, result.overflow == false) or_return
	ENSURE(t, result.count == maxLength) or_return
	ENSURE(t, result.sawZero == false) or_return
	ENSURE(t, result.collision == 0) or_return
	return true
}

// Mirrored geometry differs from its original only in sign bits, which sit at the top of every
// float. A mixer that cannot carry high bits downward maps the whole family onto a few digests.
@(private = "file")
HashFloatSigns :: proc(t: ^testing.T) -> bool {
	floatCount :: 12
	comboCount :: 1 << floatCount

	probe := ProbeBegin(comboCount)

	values: [floatCount]f32
	for i: i32 = 0; i < floatCount; i += 1 {
		values[i] = 1.0 + 0.25 * f32(i)
	}

	for mask: i32 = 0; mask < comboCount; mask += 1 {
		flipped: [floatCount]f32
		for i: i32 = 0; i < floatCount; i += 1 {
			flipped[i] = (mask & (1 << u32(i))) != 0 ? -values[i] : values[i]
		}

		ProbeAdd(&probe, b3.Hash64NonZero(cast([^]u8)&flipped, i32(size_of(flipped))), nil, 0)
	}

	result := ProbeEnd(&probe)
	ENSURE(t, result.overflow == false) or_return
	ENSURE(t, result.count == comboCount) or_return
	ENSURE(t, result.sawZero == false) or_return
	ENSURE(t, result.collision == 0) or_return
	return true
}

// Procedurally placed vertices land one ulp apart. Those blobs differ in a single low mantissa bit
// buried in a long run of identical bytes.
@(private = "file")
HashFloatUlp :: proc(t: ^testing.T) -> bool {
	floatCount :: 64

	probe := ProbeBegin(floatCount + 1)

	values: [floatCount]f32
	for i: i32 = 0; i < floatCount; i += 1 {
		values[i] = 100.0 + f32(i)
	}

	ProbeAdd(&probe, b3.Hash64NonZero(cast([^]u8)&values, i32(size_of(values))), nil, 0)

	for i: i32 = 0; i < floatCount; i += 1 {
		saved := values[i]
		bits := transmute(u32)saved
		bits += 1
		values[i] = transmute(f32)bits

		ProbeAdd(&probe, b3.Hash64NonZero(cast([^]u8)&values, i32(size_of(values))), nil, 0)
		values[i] = saved
	}

	result := ProbeEnd(&probe)
	ENSURE(t, result.overflow == false) or_return
	ENSURE(t, result.count == floatCount + 1) or_return
	ENSURE(t, result.sawZero == false) or_return
	ENSURE(t, result.collision == 0) or_return
	return true
}

// Real baked hulls. Every box shares its entire topology section and differs in a handful of floats,
// which is the closest thing the engine produces to a worst case for a content hash.
@(private = "file")
HashBoxHulls :: proc(t: ^testing.T) -> bool {
	steps :: 16

	probe := ProbeBegin(steps * steps * steps)

	for i: i32 = 0; i < steps; i += 1 {
		for j: i32 = 0; j < steps; j += 1 {
			for k: i32 = 0; k < steps; k += 1 {
				box := b3.MakeBoxHull(0.5 + 0.25 * f32(i), 0.5 + 0.25 * f32(j), 0.5 + 0.25 * f32(k))
				ProbeAdd(&probe, box.base.hash, &box, box.base.byteCount)
			}
		}
	}

	result := ProbeEnd(&probe)
	ENSURE(t, result.overflow == false) or_return
	ENSURE(t, result.count == steps * steps * steps) or_return
	ENSURE(t, result.sawZero == false) or_return
	ENSURE(t, result.collision == 0) or_return
	return true
}

// Same box, moved and turned. The extent bytes are identical across the family so the hash has to
// separate these on transform alone.
@(private = "file")
HashTransformedBoxHulls :: proc(t: ^testing.T) -> bool {
	steps :: 12

	probe := ProbeBegin(steps * steps)

	for i: i32 = 0; i < steps; i += 1 {
		for j: i32 = 0; j < steps; j += 1 {
			axis := b3.Normalize(b3.Vec3{1.0, 0.5 + 0.1 * f32(j), 0.25})
			transform := b3.Transform {
				p = {0.125 * f32(i), -0.25 * f32(j), 0.5 * f32(i + j)},
				q = b3.MakeQuatFromAxisAngle(axis, 0.05 * f32(i * steps + j)),
			}

			box := b3.MakeTransformedBoxHull(1.0, 2.0, 3.0, transform)
			ProbeAdd(&probe, box.base.hash, &box, box.base.byteCount)
		}
	}

	result := ProbeEnd(&probe)
	ENSURE(t, result.overflow == false) or_return
	ENSURE(t, result.count == steps * steps) or_return
	ENSURE(t, result.sawZero == false) or_return
	ENSURE(t, result.collision == 0) or_return
	return true
}

// Tessellated hulls across a parameter sweep. Neighboring parameters produce blobs that agree
// almost everywhere, including identical vertex counts and topology.
@(private = "file")
HashProceduralHulls :: proc(t: ^testing.T) -> bool {
	probe := ProbeBegin(1024)

	for sides: i32 = 3; sides <= 18; sides += 1 {
		for r: i32 = 1; r <= 5; r += 1 {
			for h: i32 = 1; h <= 4; h += 1 {
				cylinder := b3.CreateCylinder(0.5 * f32(h), 0.25 * f32(r), 0.0, sides)
				if cylinder != nil {
					ProbeAdd(&probe, cylinder.hash, cylinder, cylinder.byteCount)
					b3.DestroyHull(cylinder)
				}

				// Cones need at least four slices
				if sides >= 4 {
					cone := b3.CreateCone(0.5 * f32(h), 0.25 * f32(r), 0.1 * f32(r), sides)
					if cone != nil {
						ProbeAdd(&probe, cone.hash, cone, cone.byteCount)
						b3.DestroyHull(cone)
					}
				}
			}
		}
	}

	result := ProbeEnd(&probe)
	ENSURE(t, result.overflow == false) or_return
	ENSURE(t, result.count > 0) or_return
	ENSURE(t, result.sawZero == false) or_return
	ENSURE(t, result.collision == 0) or_return
	return true
}

// Height fields are mostly a flat array of heights, so grids of nearby dimensions differ in very
// little beyond their counts. Small wave grids flatten to the plain grid, which is why the probe
// has to tell a duplicate blob apart from a collision.
@(private = "file")
HashHeightFields :: proc(t: ^testing.T) -> bool {
	probe := ProbeBegin(512)

	for rows: i32 = 2; rows <= 14; rows += 1 {
		for cols: i32 = 2; cols <= 14; cols += 1 {
			scale := b3.Vec3{1.0, 1.0, 1.0}

			grid := b3.CreateGrid(rows, cols, scale, false)
			ProbeAdd(&probe, grid.hash, grid, grid.byteCount)
			b3.DestroyHeightField(grid)

			wave := b3.CreateWave(rows, cols, scale, 0.25 * f32(rows), 0.125 * f32(cols), false)
			ProbeAdd(&probe, wave.hash, wave, wave.byteCount)
			b3.DestroyHeightField(wave)
		}
	}

	result := ProbeEnd(&probe)
	ENSURE(t, result.overflow == false) or_return
	ENSURE(t, result.count > 0) or_return
	ENSURE(t, result.sawZero == false) or_return
	ENSURE(t, result.collision == 0) or_return
	return true
}

// Meshes carry a baked BVH, so most of the blob is derived data that moves in lockstep with small
// parameter changes.
@(private = "file")
HashMeshes :: proc(t: ^testing.T) -> bool {
	probe := ProbeBegin(256)

	for x: i32 = 2; x <= 10; x += 1 {
		for z: i32 = 2; z <= 10; z += 1 {
			grid := b3.CreateGridMesh(x, z, 1.0, 1, false)
			ProbeAdd(&probe, grid.hash, grid, grid.byteCount)
			b3.DestroyMesh(grid)
		}
	}

	for radial: i32 = 3; radial <= 12; radial += 1 {
		for tubular: i32 = 3; tubular <= 12; tubular += 1 {
			torus := b3.CreateTorusMesh(radial, tubular, 1.0, 0.25)
			ProbeAdd(&probe, torus.hash, torus, torus.byteCount)
			b3.DestroyMesh(torus)
		}
	}

	result := ProbeEnd(&probe)
	ENSURE(t, result.overflow == false) or_return
	ENSURE(t, result.count > 0) or_return
	ENSURE(t, result.sawZero == false) or_return
	ENSURE(t, result.collision == 0) or_return
	return true
}

// Voxel colliders sit on a regular grid, so their coordinate floats vary only in high bits. The
// hull database takes its home bucket from the low bits of the hash, so a mixer that cannot carry
// high bits downward funnels every hull into one bucket. The digests stay distinct throughout,
// which is exactly why the collision tests above cannot see it. From issue 120.
@(private = "file")
HashVoxelDispersion :: proc(t: ^testing.T) -> bool {
	hullCount :: 3000
	lowBits :: 13
	lowCount :: 1 << lowBits

	seen := b3.ALLOC(bool, lowCount)
	mem.zero(seen, lowCount * size_of(bool))

	distinctLow: i32 = 0
	for i: i32 = 0; i < hullCount; i += 1 {
		cell: f32 : 0.25
		offset := b3.Vec3{f32(i % 15) * cell, f32((i / 15) % 20) * cell, f32(i / 300) * cell}

		hull := b3.MakeOffsetBoxHull(0.5 * cell, 0.5 * cell, 0.5 * cell, offset)

		low := i32(hull.base.hash & (lowCount - 1))
		if seen[low] == false {
			seen[low] = true
			distinctLow += 1
		}
	}

	b3.FREE(seen, bool, lowCount)

	// Filling 8192 slots with 3000 draws tops out near 2500. The 32 bit hash this replaced reached 1.
	ENSURE(t, distinctLow > hullCount / 2) or_return
	return true
}

// The consequence of the above, measured where it hurt. These hulls drove the world hull database
// to two million buckets and a hundred and fifty milliseconds, since every insert walked one chain.
@(private = "file")
HashVoxelHullDatabase :: proc(t: ^testing.T) -> bool {
	hullCount :: 3000

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyId := b3.CreateBody(worldId, &bodyDef)
	shapeDef := b3.DefaultShapeDef()

	world := b3.GetWorldFromId(worldId)
	map_ := (^b3.HullMap)(world.hullDatabase)

	created := true
	for i: i32 = 0; i < hullCount; i += 1 {
		cell: f32 : 0.25
		offset := b3.Vec3{f32(i % 15) * cell, f32((i / 15) % 20) * cell, f32(i / 300) * cell}

		hull := b3.MakeOffsetBoxHull(0.5 * cell, 0.5 * cell, 0.5 * cell, offset)
		shapeId := b3.CreateHullShape(bodyId, &shapeDef, &hull.base)
		if shapeId.index1 == 0 {
			created = false
			break
		}
	}

	entryCount := b3.HullMap_size(map_)
	bucketCount := b3.HullMap_bucket_count(map_)

	b3.DestroyWorld(worldId)

	ENSURE(t, created) or_return
	ENSURE(t, entryCount == hullCount) or_return

	// Healthy growth settles at 4096 for this many distinct hulls
	ENSURE(t, bucketCount <= 4 * hullCount) or_return
	return true
}

// Identical input must bake to an identical hash, or dedup silently stops working.
@(private = "file")
HashStability :: proc(t: ^testing.T) -> bool {
	box1 := b3.MakeBoxHull(1.0, 2.0, 3.0)
	box2 := b3.MakeBoxHull(1.0, 2.0, 3.0)
	ENSURE(t, box1.base.hash == box2.base.hash) or_return

	cylinder1 := b3.CreateCylinder(1.0, 0.5, 0.0, 12)
	cylinder2 := b3.CreateCylinder(1.0, 0.5, 0.0, 12)
	sameHash := cylinder1.hash == cylinder2.hash
	sameSize := cylinder1.byteCount == cylinder2.byteCount
	sameBytes := sameSize && mem.compare_ptrs(cylinder1, cylinder2, int(cylinder1.byteCount)) == 0
	b3.DestroyHull(cylinder1)
	b3.DestroyHull(cylinder2)

	ENSURE(t, sameHash) or_return
	ENSURE(t, sameBytes) or_return

	// An empty blob has no content to mix, so it takes the reserved value
	ENSURE(t, b3.Hash64NonZero(nil, 0) == 1) or_return
	return true
}

@(test)
HashTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, HashWordFamily) {return}
	if !RUN_SUBTEST(t, HashBitSweep) {return}
	if !RUN_SUBTEST(t, HashZeroLengths) {return}
	if !RUN_SUBTEST(t, HashFloatSigns) {return}
	if !RUN_SUBTEST(t, HashFloatUlp) {return}
	if !RUN_SUBTEST(t, HashBoxHulls) {return}
	if !RUN_SUBTEST(t, HashTransformedBoxHulls) {return}
	if !RUN_SUBTEST(t, HashProceduralHulls) {return}
	if !RUN_SUBTEST(t, HashHeightFields) {return}
	if !RUN_SUBTEST(t, HashMeshes) {return}
	if !RUN_SUBTEST(t, HashVoxelDispersion) {return}
	if !RUN_SUBTEST(t, HashVoxelHullDatabase) {return}
	if !RUN_SUBTEST(t, HashStability) {return}
}
