// Ported from box3d test/test_name_cache.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "core:testing"

// Direct unit test of the cache: dedup, reverse lookup, empty/null, arbitrary length, no leak.
@(private = "file")
CacheUnit :: proc(t: ^testing.T) -> bool {
	base := b3.GetByteCount()

	cache := b3.CreateNameCache()

	// Empty and NULL map to the null id, which resolves to no string.
	ENSURE(t, b3.AddName(&cache, "") == b3.NULL_NAME) or_return
	ENSURE(t, b3.AddName(&cache, nil) == b3.NULL_NAME) or_return
	ENSURE(t, b3.FindName(&cache, b3.NULL_NAME) == nil) or_return

	// Distinct strings get distinct ids and round-trip.
	crate := b3.AddName(&cache, "crate")
	barrel := b3.AddName(&cache, "barrel")
	ENSURE(t, crate != b3.NULL_NAME && barrel != b3.NULL_NAME && crate != barrel) or_return
	ENSURE(t, string(b3.FindName(&cache, crate)) == "crate") or_return
	ENSURE(t, string(b3.FindName(&cache, barrel)) == "barrel") or_return

	// Re-interning the same bytes returns the same id and stores no second entry.
	ENSURE(t, b3.AddName(&cache, "crate") == crate) or_return
	ENSURE(t, cache.entries.count == 2) or_return

	// A name far longer than any inline buffer round-trips exactly.
	longName: [4096]u8
	for i: i32 = 0; i < i32(size_of(longName)) - 1; i += 1 {
		longName[i] = u8('a' + i % 26)
	}
	longName[size_of(longName) - 1] = 0
	longId := b3.AddName(&cache, cstring(&longName[0]))
	ENSURE(t, string(b3.FindName(&cache, longId)) == string(cstring(&longName[0]))) or_return
	ENSURE(t, cache.entries.count == 3) or_return

	b3.DestroyNameCache(&cache)

	// The cache owns every interned copy, so all of it comes back on destroy.
	ENSURE(t, b3.GetByteCount() == base) or_return
	return true
}

// Build a ground plus named dynamic boxes. Two share a name to exercise dedup and one is longer
// than the old inline buffer to exercise variable length. Ground is ordinal 0, boxes 1..N.
@(private = "file")
s_bodyNames := [?]cstring{"crate", "barrel", "crate", "a_very_long_body_name_that_exceeds_any_inline_name_buffer"}

// Each box also carries a named hull shape. Shapes could not hold a name at all before the cache
// (the old inline shape buffer had length 0), so this is a new path. Two shapes share a name for
// dedup, one shape shares a body's name to prove shape ids intern independently, and one is far
// longer than any inline buffer.
@(private = "file")
s_shapeNames := [?]cstring {
	"box_hull",
	"box_hull",
	"crate",
	"a_very_long_shape_name_that_never_fit_the_old_inline_buffer",
}

#assert(len(s_shapeNames) == len(s_bodyNames), "name arrays must stay parallel")

@(private = "file")
BuildNamedScene :: proc(worldId: b3.WorldId) {
	b3.World_SetGravity(worldId, b3.Vec3{0.0, -10.0, 0.0})

	groundDef := b3.DefaultBodyDef()
	groundDef.type = .staticBody
	groundId := b3.CreateBody(worldId, &groundDef)
	groundBox := b3.MakeBoxHull(20.0, 1.0, 20.0)
	groundShape := b3.DefaultShapeDef()
	b3.CreateHullShape(groundId, &groundShape, &groundBox.base)

	boxShape := b3.DefaultShapeDef()
	boxShape.density = 1.0
	for i: i32 = 0; i < i32(len(s_bodyNames)); i += 1 {
		box := b3.MakeBoxHull(0.5, 0.5, 0.5)
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{0.0, 1.0 + 1.1 * f32(i), 0.0}
		bodyDef.name = s_bodyNames[i]
		bodyId := b3.CreateBody(worldId, &bodyDef)
		boxShape.name = s_shapeNames[i]
		b3.CreateHullShape(bodyId, &boxShape, &box.base)
	}
}

@(private = "file")
CheckReplayNames :: proc(t: ^testing.T, player: ^b3.RecPlayer) -> bool {
	for i: i32 = 0; i < i32(len(s_bodyNames)); i += 1 {
		id := b3.RecPlayer_GetBodyId(player, 1 + i) // ordinal 0 is the ground
		ENSURE(t, b3.Body_IsValid(id)) or_return
		name := b3.Body_GetName(id)
		ENSURE(t, name != nil && string(name) == string(s_bodyNames[i])) or_return

		// Shape names ride the same world cache, so the snapshot must restore them too.
		shapeId: b3.ShapeId
		ENSURE(t, b3.Body_GetShapes(id, &shapeId, 1) == 1) or_return
		shapeName := b3.Shape_GetName(shapeId)
		ENSURE(t, shapeName != nil && string(shapeName) == string(s_shapeNames[i])) or_return
	}
	return true
}

// Names live on the world, so the world snapshot must carry them. Record a snapshot-seeded session
// and confirm every body and shape name resolves on the reconstructed replay world.
@(private = "file")
NameRoundTrip :: proc(t: ^testing.T) -> bool {
	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)
	BuildNamedScene(worldId)

	// Record from a snapshot of the populated world, so names ride in the frame-0 image.
	b3.World_StartRecording(worldId, rec)
	for i: i32 = 0; i < 20; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)
	}
	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)

	ENSURE(t, b3.ValidateReplay(b3.Recording_GetData(rec), b3.Recording_GetSize(rec), 1)) or_return

	player := b3.CreatePlayer(b3.Recording_GetData(rec), b3.Recording_GetSize(rec), 1)
	ENSURE(t, player != nil) or_return

	b3.RecPlayer_SeekFrame(player, 20)
	ENSURE(t, !b3.RecPlayer_HasDiverged(player)) or_return
	ENSURE(t, CheckReplayNames(t, player)) or_return

	b3.DestroyPlayer(player)
	b3.DestroyRecording(rec)
	return true
}

// Rollback preserves names: a backward seek restores from a keyframe, which rebuilds world->names
// on each restore. Scrub across keyframe boundaries, confirm names resolve every time, and require
// the tracked byte count to return to baseline so a re-load that drops duplicate copies leaks nothing.
@(private = "file")
RollbackNames :: proc(t: ^testing.T) -> bool {
	base := b3.GetByteCount()

	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)
	BuildNamedScene(worldId)

	// Settle, then record a snapshot-seeded session long enough to span several keyframes.
	for i: i32 = 0; i < 10; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)
	}
	b3.World_StartRecording(worldId, rec)
	totalFrames: i32 : 80
	for i: i32 = 0; i < totalFrames; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)
	}
	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)

	player := b3.CreatePlayer(b3.Recording_GetData(rec), b3.Recording_GetSize(rec), 1)
	ENSURE(t, player != nil) or_return

	// Play to the end so the keyframe ring is populated.
	b3.RecPlayer_SeekFrame(player, totalFrames)
	ENSURE(t, !b3.RecPlayer_HasDiverged(player)) or_return

	// Scrub backward and forward across keyframe boundaries. Each restore rebuilds the name table.
	targets := [?]i32{10, 60, 5, 70, 1, 40}
	for k: i32 = 0; k < i32(len(targets)); k += 1 {
		b3.RecPlayer_SeekFrame(player, targets[k])
		ENSURE(t, b3.RecPlayer_GetFrame(player) == targets[k]) or_return
		ENSURE(t, !b3.RecPlayer_HasDiverged(player)) or_return
		ENSURE(t, CheckReplayNames(t, player)) or_return
	}

	b3.DestroyPlayer(player)
	b3.DestroyRecording(rec)

	// Every restore reloaded the name table; dropped duplicate copies must be freed, not leaked.
	ENSURE(t, b3.GetByteCount() == base) or_return
	return true
}

@(test)
NameCacheTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, CacheUnit) {return}
	if !RUN_SUBTEST(t, NameRoundTrip) {return}
	if !RUN_SUBTEST(t, RollbackNames) {return}
}
