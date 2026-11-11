// Ported from box3d test/test_recording.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "base:runtime"
import "core:mem"
import "core:os"
import "core:testing"

// unified: upstream writes its temp files to the working directory; ours go to the git-ignored .out/.
@(private = "file")
s_recPath :: #directory + "/../../.out/recording_allops_test.b3rec"
@(private = "file")
s_replayPath :: #directory + "/../../.out/replay_test.b3rec"
@(private = "file")
s_taggedPath :: #directory + "/../../.out/tagged_query_test.b3rec"
// unified end

// Sphere round-trip: record/step/stop, then replay and validate.
@(private = "file")
SphereRoundTrip :: proc(t: ^testing.T) -> bool {
	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	b3.World_StartRecording(worldId, rec)

	// Set a non-default gravity so the setter op appears in the stream.
	b3.World_SetGravity(worldId, b3.Vec3{0.0, -10.0, 0.0})

	// Static ground
	groundDef := b3.DefaultBodyDef()
	groundDef.type = .staticBody
	groundId := b3.CreateBody(worldId, &groundDef)

	groundBox := b3.MakeBoxHull(50.0, 1.0, 50.0)
	groundShapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(groundId, &groundShapeDef, &groundBox.base)

	// Dynamic body with a sphere shape
	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = b3.Pos{0.0, 5.0, 0.0}
	bodyId := b3.CreateBody(worldId, &bodyDef)

	sphere: b3.Sphere
	sphere.center = b3.Vec3{0.0, 0.0, 0.0}
	sphere.radius = 0.5
	sphereDef := b3.DefaultShapeDef()
	sphereDef.density = 1.0
	b3.CreateSphereShape(bodyId, &sphereDef, &sphere)

	timeStep: f32 = 1.0 / 60.0
	subStepCount: i32 = 4
	for i := 0; i < 30; i += 1 {
		b3.World_Step(worldId, timeStep, subStepCount)
	}

	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)

	ENSURE(t, b3.ValidateReplay(b3.Recording_GetData(rec), b3.Recording_GetSize(rec), 1)) or_return

	b3.DestroyRecording(rec)
	return true
}

// Hull dedup: three bodies sharing the same hull should produce one registry entry.
@(private = "file")
HullDedup :: proc(t: ^testing.T) -> bool {
	// Build a small convex hull
	pts := [8]b3.Vec3 {
		{-1.0, -1.0, -1.0},
		{1.0, -1.0, -1.0},
		{1.0, 1.0, -1.0},
		{-1.0, 1.0, -1.0},
		{-1.0, -1.0, 1.0},
		{1.0, -1.0, 1.0},
		{1.0, 1.0, 1.0},
		{-1.0, 1.0, 1.0},
	}
	hull := b3.CreateHull(&pts[0], 8, 8)
	ENSURE(t, hull != nil) or_return

	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	b3.World_StartRecording(worldId, rec)

	shapeDef := b3.DefaultShapeDef()
	shapeDef.density = 1.0

	for i := 0; i < 3; i += 1 {
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{f32(i * 3), 5.0, 0.0}
		bodyId := b3.CreateBody(worldId, &bodyDef)
		b3.CreateHullShape(bodyId, &shapeDef, hull)
	}

	timeStep: f32 = 1.0 / 60.0
	for i := 0; i < 5; i += 1 {
		b3.World_Step(worldId, timeStep, 4)
	}

	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)
	b3.DestroyHull(hull)

	// Validate the replay
	ENSURE(t, b3.ValidateReplay(b3.Recording_GetData(rec), b3.Recording_GetSize(rec), 1)) or_return

	// Confirm the registry was deduped to 1 hull entry.
	// Parse registryOffset from the header and count entries.
	bytes := b3.Recording_GetData(rec)
	sz := b3.Recording_GetSize(rec)
	ENSURE(t, sz >= 48) or_return

	regOff: u64 = 0
	mem.copy(&regOff, &bytes[32], 8) // registryOffset at offset 32 in b3RecHeader
	ENSURE(t, regOff != 0 && i32(regOff) + 4 <= sz) or_return

	// entryCount is a little-endian u32 at the start of the registry block
	rp := bytes[int(regOff):]
	entryCount := u32(rp[0]) | (u32(rp[1]) << 8) | (u32(rp[2]) << 16) | (u32(rp[3]) << 24)
	ENSURE(t, entryCount == 1) or_return

	b3.DestroyRecording(rec)
	return true
}

@(private = "file")
ValidateReplayAtWidth :: proc(rec: ^b3.Recording, width: i32) -> i32 {
	b3.SetSIMDWidth(width)
	valid := b3.ValidateReplay(b3.Recording_GetData(rec), b3.Recording_GetSize(rec), 1)
	b3.SetSIMDWidth(0)
	return valid ? 0 : 1
}

// A recording made at the native SIMD width must replay at width 4 and vice versa.
@(private = "file")
CrossWidthReplay :: proc(t: ^testing.T) -> bool {
	b3.SetSIMDWidth(0)
	nativeWidth := b3.GetSIMDWidth()
	if nativeWidth == 4 {
		printf("  subtest skipped: CrossWidthReplay, native SIMD width is 4\n")
		return true
	}

	hullCount :: 6

	hulls: [hullCount]^b3.HullData
	hulls[0] = b3.CreateRock(0.5)
	hulls[1] = b3.CreateComplexHull(0.5)
	hulls[2] = b3.CreateCylinder(0.8, 0.4, 0.0, 8)
	hulls[3] = b3.CreateCone(0.8, 0.5, 0.2, 7)

	seed: u32 = 12345
	for hullIndex := 4; hullIndex < hullCount; hullIndex += 1 {
		points: [20]b3.Vec3
		for i := 0; i < 20; i += 1 {
			v: [3]f32
			for k := 0; k < 3; k += 1 {
				seed = seed * 1664525 + 1013904223
				v[k] = (f32(seed >> 8) / f32(1 << 24) - 0.5) * (k == 1 ? 0.6 : 1.0)
			}
			points[i] = b3.Vec3{v[0], v[1], v[2]}
		}
		hulls[hullIndex] = b3.CreateHull(&points[0], 20, 20)
	}

	rec := b3.CreateRecording(0)

	b3.SetSIMDWidth(nativeWidth)

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	b3.World_StartRecording(worldId, rec)

	groundDef := b3.DefaultBodyDef()
	groundDef.type = .staticBody
	groundId := b3.CreateBody(worldId, &groundDef)
	groundBox := b3.MakeBoxHull(20.0, 1.0, 20.0)
	groundShape := b3.DefaultShapeDef()
	b3.CreateHullShape(groundId, &groundShape, &groundBox.base)

	dynamicShape := b3.DefaultShapeDef()
	dynamicShape.density = 1.0

	for i := 0; i < 18; i += 1 {
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{0.4 * f32(i % 3) - 0.4, 1.5 + 0.9 * f32(i), 0.3 * f32(i % 2)}
		bodyDef.rotation = b3.MakeQuatFromAxisAngle(b3.Vec3{0.0, 1.0, 0.0}, 0.37 * f32(i))
		bodyId := b3.CreateBody(worldId, &bodyDef)
		b3.CreateHullShape(bodyId, &dynamicShape, hulls[i % hullCount])
	}

	timeStep: f32 = 1.0 / 60.0
	for i := 0; i < 120; i += 1 {
		b3.World_Step(worldId, timeStep, 4)
	}

	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)
	b3.SetSIMDWidth(0)

	narrowStatus := ValidateReplayAtWidth(rec, 4)
	nativeStatus := ValidateReplayAtWidth(rec, nativeWidth)

	b3.DestroyRecording(rec)
	for i := 0; i < hullCount; i += 1 {
		b3.DestroyHull(hulls[i])
	}

	ENSURE(t, narrowStatus == 0) or_return
	ENSURE(t, nativeStatus == 0) or_return
	return true
}

// Mid-stream snapshot with only dynamic bodies floating in air (no contacts).
@(private = "file")
MidStreamNoContacts :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	b3.World_SetGravity(worldId, b3.Vec3{0.0, -10.0, 0.0})

	sphere: b3.Sphere
	sphere.center = b3.Vec3{0.0, 0.0, 0.0}
	sphere.radius = 0.5
	shapeDef := b3.DefaultShapeDef()
	shapeDef.density = 1.0

	// A few dynamic bodies well apart from each other so no contacts form
	for i := 0; i < 4; i += 1 {
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{f32(i * 10), 50.0, 0.0}
		bodyId := b3.CreateBody(worldId, &bodyDef)
		b3.CreateSphereShape(bodyId, &shapeDef, &sphere)
	}

	timeStep: f32 = 1.0 / 60.0
	subStepCount: i32 = 4
	for i := 0; i < 10; i += 1 {
		b3.World_Step(worldId, timeStep, subStepCount)
	}

	// Start recording mid-stream, with a snapshot of the current world
	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return
	b3.World_StartRecording(worldId, rec)

	for i := 0; i < 30; i += 1 {
		b3.World_Step(worldId, timeStep, subStepCount)
	}

	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)

	ENSURE(t, b3.ValidateReplay(b3.Recording_GetData(rec), b3.Recording_GetSize(rec), 1)) or_return

	b3.DestroyRecording(rec)
	return true
}

// Mid-stream snapshot with contacts: bodies touching ground with warm-start manifolds.
@(private = "file")
MidStreamContacts :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	b3.World_SetGravity(worldId, b3.Vec3{0.0, -10.0, 0.0})

	// Static ground using a box hull
	{
		groundDef := b3.DefaultBodyDef()
		groundDef.type = .staticBody
		groundId := b3.CreateBody(worldId, &groundDef)

		groundBox := b3.MakeBoxHull(50.0, 1.0, 50.0)
		groundShape := b3.DefaultShapeDef()
		b3.CreateHullShape(groundId, &groundShape, &groundBox.base)
	}

	dynamicShape := b3.DefaultShapeDef()
	dynamicShape.density = 1.0

	// A few dynamic boxes dropped onto the ground
	for i := 0; i < 3; i += 1 {
		box := b3.MakeBoxHull(0.5, 0.5, 0.5)

		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{f32(i * 2) - 2.0, 5.0, 0.0}
		bodyId := b3.CreateBody(worldId, &bodyDef)
		b3.CreateHullShape(bodyId, &dynamicShape, &box.base)
	}

	timeStep: f32 = 1.0 / 60.0
	subStepCount: i32 = 4

	// Let the scene settle: bodies hit ground, build manifolds, islands, graph colors
	for i := 0; i < 60; i += 1 {
		b3.World_Step(worldId, timeStep, subStepCount)
	}

	// Start recording with snapshot of the settled world (contacts, islands, warm starts)
	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return
	b3.World_StartRecording(worldId, rec)

	for i := 0; i < 30; i += 1 {
		b3.World_Step(worldId, timeStep, subStepCount)
	}

	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)

	ENSURE(t, b3.ValidateReplay(b3.Recording_GetData(rec), b3.Recording_GetSize(rec), 1)) or_return

	b3.DestroyRecording(rec)
	return true
}

// Record a scene with hull boxes settling on a ground plane, create a player, step to
// the end recording per-frame world hashes, then seek backward to several frames and
// verify each reproduces the recorded hash exactly.
@(private = "file")
ScrubBackward :: proc(t: ^testing.T) -> bool {
	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	b3.World_SetGravity(worldId, b3.Vec3{0.0, -10.0, 0.0})

	b3.World_StartRecording(worldId, rec)

	// Static ground
	{
		groundDef := b3.DefaultBodyDef()
		groundDef.type = .staticBody
		groundId := b3.CreateBody(worldId, &groundDef)
		groundBox := b3.MakeBoxHull(20.0, 1.0, 20.0)
		groundShape := b3.DefaultShapeDef()
		b3.CreateHullShape(groundId, &groundShape, &groundBox.base)
	}

	// A small stack of dynamic hull boxes
	boxShape := b3.DefaultShapeDef()
	boxShape.density = 1.0
	for i := 0; i < 4; i += 1 {
		box := b3.MakeBoxHull(0.5, 0.5, 0.5)
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{0.0, 2.0 + f32(i) * 1.5, 0.0}
		bodyId := b3.CreateBody(worldId, &bodyDef)
		b3.CreateHullShape(bodyId, &boxShape, &box.base)
	}

	timeStep: f32 = 1.0 / 60.0
	subStepCount: i32 = 4
	totalFrames: i32 = 80
	for i: i32 = 0; i < totalFrames; i += 1 {
		b3.World_Step(worldId, timeStep, subStepCount)
	}

	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)

	data := b3.Recording_GetData(rec)
	sz := b3.Recording_GetSize(rec)

	// Create the player
	player := b3.CreatePlayer(data, sz, 1)
	ENSURE(t, player != nil) or_return
	ENSURE(t, b3.RecPlayer_GetFrameCount(player) == totalFrames) or_return

	// Forward pass: record per-frame hashes
	hashes := ([^]u64)(b3.Alloc(int(totalFrames + 1) * size_of(u64)))
	hashes[0] = 0 // frame 0 before any step

	for !b3.RecPlayer_IsAtEnd(player) {
		b3.RecPlayer_StepFrame(player)
		f := b3.RecPlayer_GetFrame(player)
		if f <= totalFrames {
			wid := b3.RecPlayer_GetWorldId(player)
			w := b3.GetWorldFromId(wid)
			hashes[f] = b3.HashWorldState(w)
		}
	}
	ENSURE(t, b3.RecPlayer_GetFrame(player) == totalFrames) or_return
	ENSURE(t, !b3.RecPlayer_HasDiverged(player)) or_return

	// Backward seek to several interesting frames and verify hash matches
	seekTargets := [?]i32{totalFrames, totalFrames / 2, 5, totalFrames - 1, 0, 1}
	seekCount := len(seekTargets)
	for k := 0; k < seekCount; k += 1 {
		target := seekTargets[k]
		b3.RecPlayer_SeekFrame(player, target)
		ENSURE(t, b3.RecPlayer_GetFrame(player) == target) or_return
		ENSURE(t, !b3.RecPlayer_HasDiverged(player)) or_return

		if target > 0 {
			wid := b3.RecPlayer_GetWorldId(player)
			w := b3.GetWorldFromId(wid)
			got := b3.HashWorldState(w)
			ENSURE(t, got == hashes[target]) or_return
		}
	}

	b3.Free(hashes, int(totalFrames + 1) * size_of(u64))
	b3.DestroyPlayer(player)
	b3.DestroyRecording(rec)
	return true
}

// Record a scene that includes a mesh shape, create a player, seek backward, verify
// it works and no divergence is reported. Also checks the keyframe-by-geometry-id
// invariant: keyframeRec registry count should not grow beyond initial slot count.
@(private = "file")
SeekWithHull :: proc(t: ^testing.T) -> bool {
	pts := [8]b3.Vec3 {
		{-1.0, -1.0, -1.0},
		{1.0, -1.0, -1.0},
		{1.0, 1.0, -1.0},
		{-1.0, 1.0, -1.0},
		{-1.0, -1.0, 1.0},
		{1.0, -1.0, 1.0},
		{1.0, 1.0, 1.0},
		{-1.0, 1.0, 1.0},
	}
	hull := b3.CreateHull(&pts[0], 8, 8)
	ENSURE(t, hull != nil) or_return

	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	b3.World_SetGravity(worldId, b3.Vec3{0.0, -10.0, 0.0})
	b3.World_StartRecording(worldId, rec)

	// Static ground
	{
		groundDef := b3.DefaultBodyDef()
		groundDef.type = .staticBody
		groundId := b3.CreateBody(worldId, &groundDef)
		groundBox := b3.MakeBoxHull(20.0, 1.0, 20.0)
		gs := b3.DefaultShapeDef()
		b3.CreateHullShape(groundId, &gs, &groundBox.base)
	}

	// Dynamic bodies using the custom hull
	sd := b3.DefaultShapeDef()
	sd.density = 1.0
	for i := 0; i < 3; i += 1 {
		bd := b3.DefaultBodyDef()
		bd.type = .dynamicBody
		bd.position = b3.Pos{f32(i * 4) - 4.0, 5.0, 0.0}
		bodyId := b3.CreateBody(worldId, &bd)
		b3.CreateHullShape(bodyId, &sd, hull)
	}

	timeStep: f32 = 1.0 / 60.0
	totalFrames: i32 = 40
	for i: i32 = 0; i < totalFrames; i += 1 {
		b3.World_Step(worldId, timeStep, 4)
	}

	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)
	b3.DestroyHull(hull)

	data := b3.Recording_GetData(rec)
	sz := b3.Recording_GetSize(rec)

	player := b3.CreatePlayer(data, sz, 1)
	ENSURE(t, player != nil) or_return

	// Step to end
	for !b3.RecPlayer_IsAtEnd(player) {
		b3.RecPlayer_StepFrame(player)
	}
	ENSURE(t, !b3.RecPlayer_HasDiverged(player)) or_return

	midFrame := totalFrames / 2
	b3.RecPlayer_SeekFrame(player, midFrame)
	ENSURE(t, b3.RecPlayer_GetFrame(player) == midFrame) or_return
	ENSURE(t, !b3.RecPlayer_HasDiverged(player)) or_return

	b3.RecPlayer_SeekFrame(player, 0)
	ENSURE(t, b3.RecPlayer_GetFrame(player) == 0) or_return

	b3.DestroyPlayer(player)
	b3.DestroyRecording(rec)
	return true
}

// Host debug-shape callbacks just count create/destroy so the test can prove the player
// wires them into every world it builds. The returned token is opaque to the engine.
@(private = "file")
DebugShapeCounters :: struct {
	created:   i32,
	destroyed: i32,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
RecTestCreateDebugShape :: proc "c" (debugShape: ^b3.DebugShape, userContext: rawptr) -> rawptr {
	context = runtime.default_context()
	// unified end
	counters := (^DebugShapeCounters)(userContext)
	counters.created += 1
	return userContext // any non-NULL token; engine stores and hands it back to destroy
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
RecTestDestroyDebugShape :: proc "c" (userShape: rawptr, userContext: rawptr) {
	context = runtime.default_context()
	// unified end
	counters := (^DebugShapeCounters)(userContext)
	counters.destroyed += 1
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
RecTestDrawShape :: proc "c" (userShape: rawptr, transform: b3.WorldTransform, color: b3.HexColor, context_: rawptr) {
	context = runtime.default_context()
	// unified end
}

// b3World_Draw lazily fires createDebugShape for shapes entering the draw set, the same way the
// sample renderer does. Drive a draw so the player's wired callbacks actually run.
@(private = "file")
RecTestDrawWorld :: proc(worldId: b3.WorldId) {
	draw := b3.DefaultDebugDraw()
	draw.DrawShapeFcn = RecTestDrawShape
	draw.drawShapes = true
	big: f32 = 1.0e6
	draw.drawingBounds = b3.AABB{{-big, -big, -big}, {big, big, big}}
	b3.World_Draw(worldId, &draw, b3.DEFAULT_MASK_BITS)
}

// The 3D sample renderer builds per-shape GPU meshes through createDebugShape, so the replay
// world must carry the host callbacks. Verify b3RecPlayer_SetDebugShapeCallbacks rewinds, fires
// the callbacks for every replayed shape, keeps them across a backward-seek world rebuild, and
// balances create/destroy at teardown.
@(private = "file")
DebugShapeCallbacks :: proc(t: ^testing.T) -> bool {
	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	b3.World_StartRecording(worldId, rec)
	b3.World_SetGravity(worldId, b3.Vec3{0.0, -10.0, 0.0})

	groundDef := b3.DefaultBodyDef()
	groundDef.type = .staticBody
	groundId := b3.CreateBody(worldId, &groundDef)
	groundBox := b3.MakeBoxHull(20.0, 1.0, 20.0)
	groundShape := b3.DefaultShapeDef()
	b3.CreateHullShape(groundId, &groundShape, &groundBox.base)

	// Four dynamic boxes sharing one hull: ground + 4 boxes = 5 shapes total.
	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	boxShape := b3.DefaultShapeDef()
	boxShape.density = 1.0
	for i := 0; i < 4; i += 1 {
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{0.0, 1.0 + 1.1 * f32(i), 0.0}
		bodyId := b3.CreateBody(worldId, &bodyDef)
		b3.CreateHullShape(bodyId, &boxShape, &box.base)
	}

	totalFrames: i32 = 30
	for i: i32 = 0; i < totalFrames; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)
	}
	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)

	// Round-trip through a file, mirroring the replay sample's Generate/Load path exactly.
	path: cstring = s_replayPath
	ENSURE(t, b3.SaveRecordingToFile(rec, path)) or_return
	loaded := b3.LoadRecordingFromFile(path)
	ENSURE(t, loaded != nil) or_return

	player := b3.CreatePlayer(b3.Recording_GetData(loaded), b3.Recording_GetSize(loaded), 1)
	ENSURE(t, player != nil) or_return

	// Wiring the callbacks rebuilds the world and rewinds to frame 0.
	counters := DebugShapeCounters{0, 0}
	b3.RecPlayer_SetDebugShapeCallbacks(player, RecTestCreateDebugShape, RecTestDestroyDebugShape, &counters)
	ENSURE(t, b3.RecPlayer_GetFrame(player) == 0) or_return

	// Replay to the end, then draw: createDebugShape fires once per shape (ground + 4 boxes = 5).
	for !b3.RecPlayer_IsAtEnd(player) {
		b3.RecPlayer_StepFrame(player)
	}
	RecTestDrawWorld(b3.RecPlayer_GetWorldId(player))
	ENSURE(t, counters.created >= 5) or_return
	ENSURE(t, !b3.RecPlayer_HasDiverged(player)) or_return

	// A backward seek restores the empty seed in place, releasing the live debug shapes, then
	// forward stepping recreates them through the callbacks, so more creates fire.
	createdBefore := counters.created
	b3.RecPlayer_SeekFrame(player, 0)
	b3.RecPlayer_SeekFrame(player, totalFrames)
	RecTestDrawWorld(b3.RecPlayer_GetWorldId(player))
	ENSURE(t, counters.created > createdBefore) or_return

	// Teardown destroys the final world; every live shape is released, so the counts balance.
	b3.DestroyPlayer(player)
	ENSURE(t, counters.created == counters.destroyed) or_return

	b3.DestroyRecording(loaded)
	b3.DestroyRecording(rec)
	_ = os.remove(string(path))
	return true
}

// Exercise the viewer-facing player accessors: recording info, creation-ordinal body tracking
// (seeded from a snapshot), divergence frame, and keyframe policy. Recording starts after the
// bodies exist, so the snapshot seeds the outliner list and ordinals are stable from frame 0.
@(private = "file")
PlayerAccessors :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)
	b3.World_SetGravity(worldId, b3.Vec3{0.0, -10.0, 0.0})

	// Static ground (creation ordinal 0)
	groundDef := b3.DefaultBodyDef()
	groundDef.type = .staticBody
	groundId := b3.CreateBody(worldId, &groundDef)
	groundBox := b3.MakeBoxHull(20.0, 1.0, 20.0)
	groundShape := b3.DefaultShapeDef()
	b3.CreateHullShape(groundId, &groundShape, &groundBox.base)

	// Four dynamic boxes (ordinals 1..4)
	dynamicCount: i32 : 4
	boxShape := b3.DefaultShapeDef()
	boxShape.density = 1.0
	for i: i32 = 0; i < dynamicCount; i += 1 {
		box := b3.MakeBoxHull(0.5, 0.5, 0.5)
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{0.0, 2.0 + f32(i) * 1.5, 0.0}
		bodyId := b3.CreateBody(worldId, &bodyDef)
		b3.CreateHullShape(bodyId, &boxShape, &box.base)
	}

	timeStep: f32 = 1.0 / 60.0
	subStepCount: i32 = 4

	// Settle, then record with a snapshot of the populated world.
	for i := 0; i < 10; i += 1 {
		b3.World_Step(worldId, timeStep, subStepCount)
	}

	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return
	b3.World_StartRecording(worldId, rec)

	totalFrames: i32 = 80
	for i: i32 = 0; i < totalFrames; i += 1 {
		b3.World_Step(worldId, timeStep, subStepCount)
	}
	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)

	player := b3.CreatePlayer(b3.Recording_GetData(rec), b3.Recording_GetSize(rec), 1)
	ENSURE(t, player != nil) or_return

	// Info reflects the recorded tuning and a non-degenerate bounds.
	info := b3.RecPlayer_GetInfo(player)
	ENSURE(t, info.frameCount == totalFrames) or_return
	ENSURE(t, info.subStepCount == subStepCount) or_return
	ENSURE(t, info.timeStep > 0.0) or_return
	extent := b3.Sub(info.bounds.upperBound, info.bounds.lowerBound)
	ENSURE(t, extent.x > 0.0 && extent.y > 0.0 && extent.z > 0.0) or_return

	// Body ordinals: ground + 4 dynamic, seeded from the snapshot and present at frame 0.
	ENSURE(t, b3.RecPlayer_GetBodyCount(player) == 1 + dynamicCount) or_return
	ground := b3.RecPlayer_GetBodyId(player, 0)
	ENSURE(t, b3.Body_IsValid(ground)) or_return
	ENSURE(t, b3.Body_GetType(ground) == .staticBody) or_return
	for i: i32 = 1; i <= dynamicCount; i += 1 {
		id := b3.RecPlayer_GetBodyId(player, i)
		ENSURE(t, b3.Body_IsValid(id)) or_return
		ENSURE(t, b3.Body_GetType(id) == .dynamicBody) or_return
	}
	ENSURE(t, b3.IS_NULL(b3.RecPlayer_GetBodyId(player, 1 + dynamicCount))) or_return

	// No divergence on a clean serial replay.
	b3.RecPlayer_SeekFrame(player, totalFrames)
	ENSURE(t, !b3.RecPlayer_HasDiverged(player)) or_return
	ENSURE(t, b3.RecPlayer_GetDivergeFrame(player) == -1) or_return

	// Ordinals survive a backward seek that restores from a keyframe.
	before := b3.RecPlayer_GetBodyId(player, 2)
	b3.RecPlayer_SeekFrame(player, totalFrames / 2)
	b3.RecPlayer_SeekFrame(player, totalFrames)
	after := b3.RecPlayer_GetBodyId(player, 2)
	ENSURE(t, b3.ID_EQUALS(before, after)) or_return

	// Keyframe policy: defaults present, setter takes effect and clears the ring.
	ENSURE(t, b3.RecPlayer_GetKeyframeMinInterval(player) == 16) or_return
	b3.RecPlayer_SetKeyframePolicy(player, 256 * 1024 * 1024, 8)
	ENSURE(t, b3.RecPlayer_GetKeyframeMinInterval(player) == 8) or_return
	ENSURE(t, b3.RecPlayer_GetKeyframeInterval(player) == 8) or_return
	ENSURE(t, b3.RecPlayer_GetKeyframeBudget(player) == 256 * 1024 * 1024) or_return
	ENSURE(t, b3.RecPlayer_GetKeyframeBytes(player) == 0) or_return

	b3.DestroyPlayer(player)
	b3.DestroyRecording(rec)
	return true
}

// A keyframe restore is a deterministic replay state, so shapes that persist must keep their
// renderer handle rather than being torn down and rebuilt every seek. Record a snapshot-seeded
// session (shapes exist at frame 0), replay with handle callbacks, scrub backward across keyframes
// repeatedly, and verify no new handles are built per restore and the create/destroy counts still
// balance at teardown.
@(private = "file")
KeyframeHandleReuse :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)
	b3.World_SetGravity(worldId, b3.Vec3{0.0, -10.0, 0.0})

	groundDef := b3.DefaultBodyDef()
	groundDef.type = .staticBody
	groundId := b3.CreateBody(worldId, &groundDef)
	groundBox := b3.MakeBoxHull(20.0, 1.0, 20.0)
	groundShape := b3.DefaultShapeDef()
	b3.CreateHullShape(groundId, &groundShape, &groundBox.base)

	dynamicCount: i32 : 5
	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	boxShape := b3.DefaultShapeDef()
	boxShape.density = 1.0
	for i: i32 = 0; i < dynamicCount; i += 1 {
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{0.0, 1.0 + 1.1 * f32(i), 0.0}
		bodyId := b3.CreateBody(worldId, &bodyDef)
		b3.CreateHullShape(bodyId, &boxShape, &box.base)
	}
	shapeCount := 1 + dynamicCount

	// Settle, then record with a snapshot of the populated world so shapes exist at frame 0.
	for i := 0; i < 10; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)
	}

	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return
	b3.World_StartRecording(worldId, rec)

	totalFrames: i32 = 80
	for i: i32 = 0; i < totalFrames; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)
	}
	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)

	player := b3.CreatePlayer(b3.Recording_GetData(rec), b3.Recording_GetSize(rec), 1)
	ENSURE(t, player != nil) or_return

	counters := DebugShapeCounters{0, 0}
	b3.RecPlayer_SetDebugShapeCallbacks(player, RecTestCreateDebugShape, RecTestDestroyDebugShape, &counters)

	// Replay to the end and draw: one handle per shape.
	b3.RecPlayer_SeekFrame(player, totalFrames)
	RecTestDrawWorld(b3.RecPlayer_GetWorldId(player))
	ENSURE(t, !b3.RecPlayer_HasDiverged(player)) or_return
	ENSURE(t, counters.created == shapeCount) or_return

	// Scrub backward and forward across keyframes. Each restore keeps the persistent shapes'
	// handles, so drawing builds no new ones.
	createdAfterFirstDraw := counters.created
	seekTargets := [?]i32{40, 70, 20, 60, 8, 75}
	for k := 0; k < len(seekTargets); k += 1 {
		b3.RecPlayer_SeekFrame(player, seekTargets[k])
		RecTestDrawWorld(b3.RecPlayer_GetWorldId(player))
	}
	ENSURE(t, counters.created == createdAfterFirstDraw) or_return

	// Teardown releases exactly the live handles, so the leak-free invariant holds.
	b3.DestroyPlayer(player)
	ENSURE(t, counters.created == counters.destroyed) or_return

	b3.DestroyRecording(rec)
	return true
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
QueryReplayOverlapFcn :: proc "c" (shapeId: b3.ShapeId, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	return true
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
QueryReplayCastFcn :: proc "c" (
	shapeId: b3.ShapeId,
	point: b3.Pos,
	normal: b3.Vec3,
	fraction: f32,
	userMaterialId: u64,
	triangleIndex: i32,
	childIndex: i32,
	context_: rawptr,
) -> f32 {
	context = runtime.default_context()
	// unified end
	// Return the fraction to keep the closest hit, exercising the recorded user-return path.
	return fraction
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
CompoundHitCastFcn :: proc "c" (
	shapeId: b3.ShapeId,
	point: b3.Pos,
	normal: b3.Vec3,
	fraction: f32,
	userMaterialId: u64,
	triangleIndex: i32,
	childIndex: i32,
	context_: rawptr,
) -> f32 {
	context = runtime.default_context()
	// unified end
	if b3.Shape_GetType(shapeId) == .compoundShape {
		(^bool)(context_)^ = true
	}
	return 1.0
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
QueryReplayPlaneFcn :: proc "c" (
	shapeId: b3.ShapeId,
	planes: ^b3.PlaneResult,
	planeCount: i32,
	context_: rawptr,
) -> bool {
	context = runtime.default_context()
	// unified end
	return true
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
QueryReplayMoverFilterFcn :: proc "c" (shapeId: b3.ShapeId, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	return true
}

// Issue all seven world queries each frame, then replay. Every query is re-issued against the
// replay world and compared to what was recorded, so a clean (non-diverged) replay proves the
// queries reproduce. Also opens a player and confirms the per-frame query store surfaces all seven.
@(private = "file")
QueryReplay :: proc(t: ^testing.T) -> bool {
	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)
	b3.World_SetGravity(worldId, b3.Vec3{0.0, -10.0, 0.0})

	groundDef := b3.DefaultBodyDef()
	groundDef.type = .staticBody
	groundId := b3.CreateBody(worldId, &groundDef)
	groundBox := b3.MakeBoxHull(20.0, 1.0, 20.0)
	groundShape := b3.DefaultShapeDef()
	b3.CreateHullShape(groundId, &groundShape, &groundBox.base)

	// A few dynamic spheres for the queries to find.
	for i := 0; i < 4; i += 1 {
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{f32(i) - 1.5, 3.0, 0.0}
		bodyId := b3.CreateBody(worldId, &bodyDef)
		sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}
		sphereDef := b3.DefaultShapeDef()
		sphereDef.density = 1.0
		b3.CreateSphereShape(bodyId, &sphereDef, &sphere)
	}

	b3.World_StartRecording(worldId, rec)

	filter := b3.DefaultQueryFilter()

	totalFrames :: 30
	for i := 0; i < totalFrames; i += 1 {
		origin := b3.Pos{0.0, 6.0, 0.0}
		translation := b3.Vec3{0.0, -8.0, 0.0}
		aabb := b3.AABB{{-5.0, -1.0, -5.0}, {5.0, 6.0, 5.0}}

		proxyPts := b3.Vec3{0.0, 0.0, 0.0}
		proxy := b3.ShapeProxy{&proxyPts, 1, 0.5}
		mover := b3.Capsule{{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.3}

		b3.World_OverlapAABB(worldId, aabb, filter, QueryReplayOverlapFcn, nil)
		b3.World_OverlapShape(worldId, origin, &proxy, filter, QueryReplayOverlapFcn, nil)
		b3.World_CastRay(worldId, origin, translation, filter, QueryReplayCastFcn, nil)
		b3.World_CastRayClosest(worldId, origin, translation, filter)
		b3.World_CastShape(worldId, origin, &proxy, translation, filter, QueryReplayCastFcn, nil)
		b3.World_CastMover(worldId, origin, &mover, translation, filter, QueryReplayMoverFilterFcn, nil)
		b3.World_CollideMover(worldId, origin, &mover, filter, QueryReplayPlaneFcn, nil)

		b3.World_Step(worldId, 1.0 / 60.0, 4)
	}

	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)

	// Headless validation: re-issues every recorded query and compares the results.
	ENSURE(t, b3.ValidateReplay(b3.Recording_GetData(rec), b3.Recording_GetSize(rec), 1)) or_return

	// Player path: seek to a mid frame and confirm the per-frame store holds all seven queries.
	player := b3.CreatePlayer(b3.Recording_GetData(rec), b3.Recording_GetSize(rec), 1)
	ENSURE(t, player != nil) or_return

	b3.RecPlayer_SeekFrame(player, 15)
	ENSURE(t, !b3.RecPlayer_HasDiverged(player)) or_return
	ENSURE(t, b3.RecPlayer_GetFrameQueryCount(player) == 7) or_return

	first := b3.RecPlayer_GetFrameQuery(player, 0)
	ENSURE(t, first.type == .recQueryOverlapAABB) or_return

	// The ray cast should find at least the ground, so its recorded hit list is non-empty.
	sawCastRay := false
	for qi: i32 = 0; qi < b3.RecPlayer_GetFrameQueryCount(player); qi += 1 {
		info := b3.RecPlayer_GetFrameQuery(player, qi)
		if info.type == .recQueryCastRay {
			sawCastRay = true
			ENSURE(t, info.hitCount > 0) or_return
		}
	}
	ENSURE(t, sawCastRay) or_return

	b3.DestroyPlayer(player)
	b3.DestroyRecording(rec)
	return true
}

// Tagged queries: the caller (id, label) is hashed into a key that rides the QueryTag op, with the
// id and label interned in the trailing tag table. The same label under two entity ids is two
// distinct keys. All survive a file round-trip and resolve back through b3RecQueryInfo; untagged
// queries report key 0 / id 0 / name NULL.
@(private = "file")
TaggedQuery :: proc(t: ^testing.T) -> bool {
	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	groundDef := b3.DefaultBodyDef()
	groundDef.type = .staticBody
	groundId := b3.CreateBody(worldId, &groundDef)
	groundBox := b3.MakeBoxHull(20.0, 1.0, 20.0)
	groundShape := b3.DefaultShapeDef()
	b3.CreateHullShape(groundId, &groundShape, &groundBox.base)

	b3.World_StartRecording(worldId, rec)

	// Same label, different entity ids: distinct logical queries with distinct keys.
	bullet53 := b3.DefaultQueryFilter()
	bullet53.id = 53
	bullet53.name = "bullet"

	bullet54 := b3.DefaultQueryFilter()
	bullet54.id = 54
	bullet54.name = "bullet"

	untagged := b3.DefaultQueryFilter()

	key53 := b3.HashQueryTag(53, "bullet")
	key54 := b3.HashQueryTag(54, "bullet")
	ENSURE(t, key53 != 0 && key54 != 0 && key53 != key54) or_return

	totalFrames :: 10
	for i := 0; i < totalFrames; i += 1 {
		origin := b3.Pos{0.0, 6.0, 0.0}
		translation := b3.Vec3{0.0, -8.0, 0.0}
		aabb := b3.AABB{{-5.0, -1.0, -5.0}, {5.0, 6.0, 5.0}}

		// Two tagged rays sharing the label "bullet" plus one untagged overlap, every frame.
		b3.World_CastRay(worldId, origin, translation, bullet53, QueryReplayCastFcn, nil)
		b3.World_CastRay(worldId, origin, translation, bullet54, QueryReplayCastFcn, nil)
		b3.World_OverlapAABB(worldId, aabb, untagged, QueryReplayOverlapFcn, nil)

		b3.World_Step(worldId, 1.0 / 60.0, 4)
	}

	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)

	ENSURE(t, b3.ValidateReplay(b3.Recording_GetData(rec), b3.Recording_GetSize(rec), 1)) or_return

	// Round-trip through a file so the interned tag table is exercised on the persisted bytes.
	path: cstring = s_taggedPath
	ENSURE(t, b3.SaveRecordingToFile(rec, path)) or_return
	loaded := b3.LoadRecordingFromFile(path)
	ENSURE(t, loaded != nil) or_return

	player := b3.CreatePlayer(b3.Recording_GetData(loaded), b3.Recording_GetSize(loaded), 1)
	ENSURE(t, player != nil) or_return

	b3.RecPlayer_SeekFrame(player, 5)
	ENSURE(t, !b3.RecPlayer_HasDiverged(player)) or_return
	ENSURE(t, b3.RecPlayer_GetFrameQueryCount(player) == 3) or_return

	saw53, saw54, sawUntagged := false, false, false
	for qi: i32 = 0; qi < b3.RecPlayer_GetFrameQueryCount(player); qi += 1 {
		info := b3.RecPlayer_GetFrameQuery(player, qi)
		if info.key == key53 {
			saw53 = true
			ENSURE(t, info.id == 53 && info.name != nil && string(info.name) == "bullet") or_return
		} else if info.key == key54 {
			saw54 = true
			ENSURE(t, info.id == 54 && info.name != nil && string(info.name) == "bullet") or_return
		} else {
			sawUntagged = true
			ENSURE(t, info.key == 0 && info.id == 0 && info.name == nil) or_return
		}
	}
	ENSURE(t, saw53 && saw54 && sawUntagged) or_return

	b3.DestroyPlayer(player)
	b3.DestroyRecording(loaded)
	b3.DestroyRecording(rec)
	// unified: upstream leaves tagged_query_test.b3rec behind; we delete our temp file.
	_ = os.remove(string(path))
	return true
}

// Empty world: recording starts with no bodies and none are ever created. The empty world is still
// seed-serialized like any other, so replay validates and Restart restores in place with a stable
// world id rather than tearing down and rebuilding the world.
@(private = "file")
EmptyWorldRoundTrip :: proc(t: ^testing.T) -> bool {
	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	b3.World_StartRecording(worldId, rec)
	b3.World_SetGravity(worldId, b3.Vec3{0.0, -10.0, 0.0})

	for i := 0; i < 10; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)
	}

	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)

	data := b3.Recording_GetData(rec)
	size := b3.Recording_GetSize(rec)

	// The seed snapshot is written even with no bodies.
	hdr: b3.RecHeader
	mem.copy(&hdr, data, size_of(hdr))
	ENSURE(t, hdr.snapshotSize > 0) or_return

	ENSURE(t, b3.ValidateReplay(data, size, 1)) or_return

	// Restart restores in place, so the replay world id survives a rewind.
	player := b3.CreatePlayer(data, size, 1)
	ENSURE(t, player != nil) or_return

	worldKey := b3.StoreWorldId(b3.RecPlayer_GetWorldId(player))
	for !b3.RecPlayer_IsAtEnd(player) {
		b3.RecPlayer_StepFrame(player)
	}
	b3.RecPlayer_Restart(player)
	ENSURE(t, b3.StoreWorldId(b3.RecPlayer_GetWorldId(player)) == worldKey) or_return
	ENSURE(t, b3.RecPlayer_GetFrame(player) == 0) or_return
	ENSURE(t, !b3.RecPlayer_HasDiverged(player)) or_return

	b3.DestroyPlayer(player)
	b3.DestroyRecording(rec)
	return true
}

// Safety factor of the named body in a replay world, or -1 if the body is not in the list
@(private = "file")
ReplaySafetyFactor :: proc(player: ^b3.RecPlayer, name: string) -> f32 {
	count := b3.RecPlayer_GetBodyCount(player)
	for i: i32 = 0; i < count; i += 1 {
		bodyId := b3.RecPlayer_GetBodyId(player, i)
		if b3.Body_IsValid(bodyId) && string(b3.Body_GetName(bodyId)) == name {
			return b3.Body_GetSafetyFactor(bodyId)
		}
	}

	return -1.0
}

// Exercise every recorded op in a single session, then validate replay at two worker
// counts, round-trip through a file, and drive the incremental player.
@(private = "file")
AllOps :: proc(t: ^testing.T) -> bool {
	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return

	worldDef := b3.DefaultWorldDef()
	worldDef.workerCount = 1
	worldId := b3.CreateWorld(&worldDef)
	ENSURE(t, b3.World_IsValid(worldId)) or_return

	b3.World_StartRecording(worldId, rec)

	// Static ground with a box-hull shape
	groundDef := b3.DefaultBodyDef()
	groundDef.type = .staticBody
	groundId := b3.CreateBody(worldId, &groundDef)
	ENSURE(t, b3.Body_IsValid(groundId)) or_return
	groundBox := b3.MakeBoxHull(50.0, 1.0, 50.0)
	groundShapeDef := b3.DefaultShapeDef()
	groundShapeId := b3.CreateHullShape(groundId, &groundShapeDef, &groundBox.base)
	ENSURE(t, b3.Shape_IsValid(groundShapeId)) or_return

	// Dynamic body with a sphere shape.
	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = b3.Pos{0.0, 5.0, 0.0}
	bodyDef.name = "testBodyWithVeryLongNameThatIsAVeryLongNameLength"
	bodyId := b3.CreateBody(worldId, &bodyDef)
	ENSURE(t, b3.Body_IsValid(bodyId)) or_return

	sphereShapeDef := b3.DefaultShapeDef()
	sphereShapeDef.density = 1.0
	// Over-length shape name so replay exercises the clamp in the shape def reader, like the body
	// above.
	sphereShapeDef.name = "sphereNameThatExceedsTheLimit"
	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}
	sphereShapeId := b3.CreateSphereShape(bodyId, &sphereShapeDef, &sphere)
	ENSURE(t, b3.Shape_IsValid(sphereShapeId)) or_return

	// Capsule shape on a second dynamic body
	capsuleBodyDef := b3.DefaultBodyDef()
	capsuleBodyDef.type = .dynamicBody
	capsuleBodyDef.position = b3.Pos{3.0, 5.0, 0.0}
	capsuleBodyId := b3.CreateBody(worldId, &capsuleBodyDef)
	ENSURE(t, b3.Body_IsValid(capsuleBodyId)) or_return

	capsuleShapeDef := b3.DefaultShapeDef()
	capsuleShapeDef.density = 1.0
	capsule := b3.Capsule{{0.0, -0.4, 0.0}, {0.0, 0.4, 0.0}, 0.25}
	capsuleShapeId := b3.CreateCapsuleShape(capsuleBodyId, &capsuleShapeDef, &capsule)
	ENSURE(t, b3.Shape_IsValid(capsuleShapeId)) or_return

	// Custom hull shape on a third dynamic body
	hullPts := [8]b3.Vec3 {
		{-0.5, -0.5, -0.5},
		{0.5, -0.5, -0.5},
		{0.5, 0.5, -0.5},
		{-0.5, 0.5, -0.5},
		{-0.5, -0.5, 0.5},
		{0.5, -0.5, 0.5},
		{0.5, 0.5, 0.5},
		{-0.5, 0.5, 0.5},
	}
	customHull := b3.CreateHull(&hullPts[0], 8, 8)
	ENSURE(t, customHull != nil) or_return

	hullBodyDef := b3.DefaultBodyDef()
	hullBodyDef.type = .dynamicBody
	hullBodyDef.position = b3.Pos{-3.0, 5.0, 0.0}
	hullBodyId := b3.CreateBody(worldId, &hullBodyDef)
	ENSURE(t, b3.Body_IsValid(hullBodyId)) or_return

	hullShapeDef := b3.DefaultShapeDef()
	hullShapeDef.density = 1.0
	hullShapeId := b3.CreateHullShape(hullBodyId, &hullShapeDef, customHull)
	ENSURE(t, b3.Shape_IsValid(hullShapeId)) or_return

	// Box hull shape on a fourth dynamic body (b3MakeBoxHull path)
	boxBodyDef := b3.DefaultBodyDef()
	boxBodyDef.type = .dynamicBody
	boxBodyDef.position = b3.Pos{6.0, 5.0, 0.0}
	boxBodyId := b3.CreateBody(worldId, &boxBodyDef)
	ENSURE(t, b3.Body_IsValid(boxBodyId)) or_return

	boxShapeDef := b3.DefaultShapeDef()
	boxShapeDef.density = 2.0
	boxHull := b3.MakeBoxHull(0.5, 0.5, 0.5)
	boxShapeId := b3.CreateHullShape(boxBodyId, &boxShapeDef, &boxHull.base)
	ENSURE(t, b3.Shape_IsValid(boxShapeId)) or_return

	// Transformed hull shape on a fifth dynamic body (b3CreateTransformedHullShape path)
	xformBodyDef := b3.DefaultBodyDef()
	xformBodyDef.type = .dynamicBody
	xformBodyDef.position = b3.Pos{12.0, 5.0, 0.0}
	xformBodyId := b3.CreateBody(worldId, &xformBodyDef)
	ENSURE(t, b3.Body_IsValid(xformBodyId)) or_return
	xformShapeDef := b3.DefaultShapeDef()
	xformShapeDef.density = 1.0
	xformXf := b3.Transform{b3.Vec3{0.1, 0.2, -0.1}, b3.MakeQuatFromAxisAngle(b3.Vec3{0.0, 1.0, 0.0}, 0.4)}
	xformShapeId := b3.CreateTransformedHullShape(
		xformBodyId,
		&xformShapeDef,
		customHull,
		xformXf,
		b3.Vec3{1.25, 0.75, 1.5},
	)
	ENSURE(t, b3.Shape_IsValid(xformShapeId)) or_return

	// Mesh, height field, and compound static shapes (3D-only)
	meshBodyDef := b3.DefaultBodyDef()
	meshBodyDef.type = .staticBody
	meshBodyDef.position = b3.Pos{20.0, 0.0, 0.0}
	meshBodyId := b3.CreateBody(worldId, &meshBodyDef)
	meshData := b3.CreateGridMesh(3, 3, 2.0, 0, false)
	ENSURE(t, meshData != nil) or_return
	swapMeshData := b3.CreateGridMesh(4, 4, 1.5, 0, false)
	ENSURE(t, swapMeshData != nil) or_return
	meshShapeDef := b3.DefaultShapeDef()
	meshShapeId := b3.CreateMeshShape(meshBodyId, &meshShapeDef, meshData, b3.Vec3{1.0, 1.0, 1.0})
	ENSURE(t, b3.Shape_IsValid(meshShapeId)) or_return

	hfBodyDef := b3.DefaultBodyDef()
	hfBodyDef.type = .staticBody
	hfBodyDef.position = b3.Pos{-20.0, 0.0, 0.0}
	hfBodyId := b3.CreateBody(worldId, &hfBodyDef)
	hf := b3.CreateGrid(4, 4, b3.Vec3{2.0, 1.0, 2.0}, false)
	ENSURE(t, hf != nil) or_return
	hfShapeDef := b3.DefaultShapeDef()
	b3.CreateHeightFieldShape(hfBodyId, &hfShapeDef, hf)

	compoundBodyDef := b3.DefaultBodyDef()
	compoundBodyDef.type = .staticBody
	compoundBodyDef.position = b3.Pos{30.0, 0.0, 0.0}
	compoundBodyId := b3.CreateBody(worldId, &compoundBodyDef)
	compSphere: b3.CompoundSphereDef
	compSphere.sphere = b3.Sphere{{0.0, 0.0, 0.0}, 1.0}
	compSphere.material = b3.DefaultSurfaceMaterial()

	// Mesh child with five materials so the compound material map is recorded
	compoundMeshData := b3.CreateGridMesh(1, 5, 1.0, 5, false)
	ENSURE(t, compoundMeshData != nil) or_return
	ENSURE(t, compoundMeshData.materialCount == 5) or_return
	compoundMeshMaterials: [5]b3.SurfaceMaterial
	for i := 0; i < 5; i += 1 {
		compoundMeshMaterials[i] = b3.DefaultSurfaceMaterial()
	}
	compMesh := b3.CompoundMeshDef{}
	compMesh.meshData = compoundMeshData
	compMesh.transform = b3.Transform_identity
	compMesh.scale = b3.Vec3{1.0, 1.0, 1.0}
	compMesh.materials = &compoundMeshMaterials[0]
	compMesh.materialCount = 5

	compoundDef := b3.CompoundDef{}
	compoundDef.spheres = &compSphere
	compoundDef.sphereCount = 1
	compoundDef.meshes = &compMesh
	compoundDef.meshCount = 1
	compound := b3.CreateCompound(&compoundDef)
	ENSURE(t, compound != nil) or_return
	compoundShapeDef := b3.DefaultShapeDef()
	b3.CreateBakedCompoundShape(compoundBodyId, &compoundShapeDef, compound)

	// Throwaway shape to exercise DestroyShape
	tmpSphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.1}
	tmpShapeId := b3.CreateSphereShape(capsuleBodyId, &capsuleShapeDef, &tmpSphere)
	b3.DestroyShape(tmpShapeId, true)

	// Shape mutators: SetFriction, SetRestitution, SetDensity, SetSurfaceMaterial, SetMeshMaterial,
	// SetFilter, EnableSensorEvents, EnableContactEvents, EnableHitEvents, EnablePreSolveEvents,
	// ApplyWind, SetSphere, SetCapsule, SetHull, SetMesh, SetName
	b3.Shape_SetFriction(boxShapeId, 0.3)
	b3.Shape_SetRestitution(capsuleShapeId, 0.5)
	b3.Shape_SetDensity(boxShapeId, 3.0, true)
	surfMat := b3.DefaultSurfaceMaterial()
	surfMat.friction = 0.7
	surfMat.restitution = 0.1
	b3.Shape_SetSurfaceMaterial(capsuleShapeId, surfMat)
	shapeFilter := b3.DefaultFilter()
	shapeFilter.categoryBits = 0x2
	b3.Shape_SetFilter(boxShapeId, shapeFilter, false)
	b3.Shape_EnableSensorEvents(capsuleShapeId, true)
	b3.Shape_EnableContactEvents(capsuleShapeId, true)
	b3.Shape_EnableHitEvents(boxShapeId, true)
	b3.Shape_EnablePreSolveEvents(boxShapeId, true)
	b3.Shape_ApplyWind(capsuleShapeId, b3.Vec3{1.0, 0.0, 0.0}, 0.1, 0.0, 10.0, true)
	newSphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.45}
	b3.Shape_SetSphere(sphereShapeId, &newSphere)
	newCapsule := b3.Capsule{{0.0, -0.3, 0.0}, {0.0, 0.3, 0.0}, 0.3}
	b3.Shape_SetCapsule(capsuleShapeId, &newCapsule)
	b3.Shape_SetName(boxShapeId, "box")

	// Geometry swaps intern into the registry at the record site. The repeated SetHull takes the
	// shared hull short circuit, which changes nothing and so must leave the stream alone.
	swapHull := b3.MakeBoxHull(0.3, 0.7, 0.4)
	b3.Shape_SetHull(boxShapeId, &swapHull.base)
	b3.Shape_SetHull(boxShapeId, &swapHull.base)
	b3.Shape_SetMesh(meshShapeId, swapMeshData, b3.Vec3{1.0, 1.0, 1.0})
	b3.Shape_SetMeshMaterial(meshShapeId, surfMat, 0)

	// Body mutators: SetTransform, SetLinearVelocity/AngularVelocity (Vec3), SetName,
	// damping, gravity scale, sleep threshold, SetAwake, EnableSleep, SetBullet, SetMotionLocks,
	// SetMassData, ApplyMassFromShapes, SetType, SetTargetTransform, Disable/Enable,
	// EnableContactRecycling, EnableHitEvents, all force/impulse/torque variants
	b3.Body_SetTransform(bodyId, b3.Pos{1.0, 6.0, 0.0}, b3.Quat_identity)
	b3.Body_SetLinearVelocity(bodyId, b3.Vec3{0.5, 0.0, 0.0})
	b3.Body_SetAngularVelocity(bodyId, b3.Vec3{0.0, 0.25, 0.0})
	b3.Body_SetName(bodyId, "renamedBody")
	b3.Body_SetLinearDamping(bodyId, 0.1)
	b3.Body_SetAngularDamping(bodyId, 0.05)
	b3.Body_SetGravityScale(bodyId, 0.9)
	b3.Body_SetSleepThreshold(bodyId, 0.02)
	b3.Body_EnableSleep(bodyId, false)
	b3.Body_SetBullet(bodyId, true)
	b3.Body_EnableContactRecycling(bodyId, false)
	b3.Body_EnableHitEvents(bodyId, true)
	b3.Body_SetMotionLocks(bodyId, b3.MotionLocks{false, false, false, false, false, true})
	massData: b3.MassData
	massData.mass = 2.0
	massData.center = b3.Vec3{0.0, 0.0, 0.0}
	massData.inertia = b3.Mat3_identity
	b3.Body_SetMassData(bodyId, massData)
	b3.Body_ApplyMassFromShapes(bodyId)
	b3.Body_SetType(capsuleBodyId, .kinematicBody)
	b3.Body_SetType(capsuleBodyId, .dynamicBody)
	b3.Body_SetAwake(bodyId, true)

	// Kinematic body to exercise SetTargetTransform
	kinematicDef := b3.DefaultBodyDef()
	kinematicDef.type = .kinematicBody
	kinematicDef.position = b3.Pos{-6.0, 5.0, 0.0}
	kinematicId := b3.CreateBody(worldId, &kinematicDef)
	kinBox := b3.MakeBoxHull(0.4, 0.4, 0.4)
	kinShapeDef := b3.DefaultShapeDef()
	b3.CreateHullShape(kinematicId, &kinShapeDef, &kinBox.base)
	kinTarget: b3.WorldTransform
	kinTarget.p = b3.Pos{-5.0, 5.0, 0.0}
	kinTarget.q = b3.Quat_identity
	b3.Body_SetTargetTransform(kinematicId, kinTarget, 1.0 / 60.0, true)

	// Body to exercise Disable/Enable
	disableDef := b3.DefaultBodyDef()
	disableDef.type = .dynamicBody
	disableDef.position = b3.Pos{9.0, 5.0, 0.0}
	disableDef.name = "dropBody"
	disableId := b3.CreateBody(worldId, &disableDef)
	disableSphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.3}
	b3.CreateSphereShape(disableId, &sphereShapeDef, &disableSphere)
	b3.Body_Disable(disableId)
	b3.Body_Enable(disableId)
	b3.Body_SetSafetyFactor(disableId, 0.25)

	// A body that falls fast enough to stay on the continuous path for the whole session. The
	// non-default factor is what proves the widened body def payload round-trips, since every
	// other body records the default. Named so the replay check can find it.
	fastDef := b3.DefaultBodyDef()
	fastDef.type = .dynamicBody
	fastDef.position = b3.Pos{-14.0, 30.0, 0.0}
	fastDef.linearVelocity = b3.Vec3{0.0, -8.0, 0.0}
	fastDef.safetyFactor = 0.1
	fastDef.name = "fastBody"
	fastId := b3.CreateBody(worldId, &fastDef)
	ENSURE(t, b3.Body_GetSafetyFactor(fastId) == 0.1) or_return
	fastBox := b3.MakeBoxHull(0.5, 0.5, 0.5)
	b3.CreateHullShape(fastId, &sphereShapeDef, &fastBox.base)

	// Force/impulse/torque (Vec3 args in 3D)
	b3.Body_ApplyForce(bodyId, b3.Vec3{0.0, 50.0, 0.0}, b3.Pos{1.0, 6.0, 0.0}, true)
	b3.Body_ApplyForceToCenter(bodyId, b3.Vec3{5.0, 0.0, 0.0}, true)
	b3.Body_ApplyTorque(bodyId, b3.Vec3{0.0, 1.0, 0.0}, true)
	b3.Body_ApplyLinearImpulse(bodyId, b3.Vec3{0.1, 0.0, 0.0}, b3.Pos{1.0, 6.0, 0.0}, true)
	b3.Body_ApplyLinearImpulseToCenter(bodyId, b3.Vec3{0.0, 0.1, 0.0}, true)
	b3.Body_ApplyAngularImpulse(bodyId, b3.Vec3{0.0, 0.05, 0.0}, true)

	// Joint bodies: a row of dynamic bodies connected by each joint type
	jb: [9]b3.BodyId
	for i := 0; i < 9; i += 1 {
		jbd := b3.DefaultBodyDef()
		jbd.type = .dynamicBody
		jbd.position = b3.Pos{-8.0 + f32(i) * 2.0, 10.0, 0.0}
		jb[i] = b3.CreateBody(worldId, &jbd)
		js := b3.Sphere{{0.0, 0.0, 0.0}, 0.25}
		jsd := b3.DefaultShapeDef()
		jsd.density = 1.0
		b3.CreateSphereShape(jb[i], &jsd, &js)
	}

	// Revolute joint with full setter coverage and the generic joint mutators
	revDef := b3.DefaultRevoluteJointDef()
	revDef.base.bodyIdA = jb[0]
	revDef.base.bodyIdB = jb[1]
	revDef.base.localFrameA.p = b3.Vec3{1.0, 0.0, 0.0}
	revDef.base.localFrameB.p = b3.Vec3{-1.0, 0.0, 0.0}
	revId := b3.CreateRevoluteJoint(worldId, &revDef)
	ENSURE(t, b3.Joint_IsValid(revId)) or_return
	b3.RevoluteJoint_EnableLimit(revId, true)
	b3.RevoluteJoint_SetLimits(revId, -1.0, 1.0)
	b3.RevoluteJoint_EnableMotor(revId, true)
	b3.RevoluteJoint_SetMotorSpeed(revId, 0.5)
	b3.RevoluteJoint_SetMaxMotorTorque(revId, 10.0)
	b3.RevoluteJoint_EnableSpring(revId, true)
	b3.RevoluteJoint_SetSpringHertz(revId, 2.0)
	b3.RevoluteJoint_SetSpringDampingRatio(revId, 0.5)
	b3.RevoluteJoint_SetTargetAngle(revId, 0.25)
	b3.Joint_SetLocalFrameA(revId, b3.Transform{b3.Vec3{1.0, 0.0, 0.0}, b3.Quat_identity})
	b3.Joint_SetLocalFrameB(revId, b3.Transform{b3.Vec3{-1.0, 0.0, 0.0}, b3.Quat_identity})
	b3.Joint_SetConstraintTuning(revId, 60.0, 2.0)
	b3.Joint_SetForceThreshold(revId, 100.0)
	b3.Joint_SetTorqueThreshold(revId, 50.0)
	b3.Joint_SetCollideConnected(revId, false)
	b3.Joint_WakeBodies(revId)

	// Distance joint
	distDef := b3.DefaultDistanceJointDef()
	distDef.base.bodyIdA = jb[1]
	distDef.base.bodyIdB = jb[2]
	distDef.length = 2.0
	distId := b3.CreateDistanceJoint(worldId, &distDef)
	b3.DistanceJoint_SetLength(distId, 2.2)
	b3.DistanceJoint_EnableSpring(distId, true)
	b3.DistanceJoint_SetSpringHertz(distId, 3.0)
	b3.DistanceJoint_SetSpringDampingRatio(distId, 0.4)
	b3.DistanceJoint_SetSpringForceRange(distId, -50.0, 50.0)
	b3.DistanceJoint_EnableLimit(distId, true)
	b3.DistanceJoint_SetLengthRange(distId, 1.0, 4.0)
	b3.DistanceJoint_EnableMotor(distId, true)
	b3.DistanceJoint_SetMotorSpeed(distId, 0.3)
	b3.DistanceJoint_SetMaxMotorForce(distId, 5.0)

	// Filter joint (plus a throwaway to exercise DestroyJoint)
	filterDef := b3.DefaultFilterJointDef()
	filterDef.base.bodyIdA = jb[2]
	filterDef.base.bodyIdB = jb[3]
	filterId := b3.CreateFilterJoint(worldId, &filterDef)
	ENSURE(t, b3.Joint_IsValid(filterId)) or_return

	tmpJointDef := b3.DefaultDistanceJointDef()
	tmpJointDef.base.bodyIdA = jb[0]
	tmpJointDef.base.bodyIdB = jb[8]
	tmpJointDef.length = 5.0
	tmpJointId := b3.CreateDistanceJoint(worldId, &tmpJointDef)
	b3.DestroyJoint(tmpJointId, true)

	// Motor joint (Vec3 velocities in 3D)
	motorDef := b3.DefaultMotorJointDef()
	motorDef.base.bodyIdA = jb[3]
	motorDef.base.bodyIdB = jb[4]
	motorId := b3.CreateMotorJoint(worldId, &motorDef)
	b3.MotorJoint_SetLinearVelocity(motorId, b3.Vec3{0.1, 0.0, 0.0})
	b3.MotorJoint_SetAngularVelocity(motorId, b3.Vec3{0.0, 0.2, 0.0})
	b3.MotorJoint_SetMaxVelocityForce(motorId, 10.0)
	b3.MotorJoint_SetMaxVelocityTorque(motorId, 10.0)
	b3.MotorJoint_SetLinearHertz(motorId, 2.0)
	b3.MotorJoint_SetLinearDampingRatio(motorId, 0.5)
	b3.MotorJoint_SetAngularHertz(motorId, 2.0)
	b3.MotorJoint_SetAngularDampingRatio(motorId, 0.5)
	b3.MotorJoint_SetMaxSpringForce(motorId, 20.0)
	b3.MotorJoint_SetMaxSpringTorque(motorId, 20.0)

	// Prismatic joint
	prisDef := b3.DefaultPrismaticJointDef()
	prisDef.base.bodyIdA = jb[4]
	prisDef.base.bodyIdB = jb[5]
	prisId := b3.CreatePrismaticJoint(worldId, &prisDef)
	b3.PrismaticJoint_EnableSpring(prisId, true)
	b3.PrismaticJoint_SetSpringHertz(prisId, 2.0)
	b3.PrismaticJoint_SetSpringDampingRatio(prisId, 0.5)
	b3.PrismaticJoint_SetTargetTranslation(prisId, 0.1)
	b3.PrismaticJoint_EnableLimit(prisId, true)
	b3.PrismaticJoint_SetLimits(prisId, -1.0, 1.0)
	b3.PrismaticJoint_EnableMotor(prisId, true)
	b3.PrismaticJoint_SetMotorSpeed(prisId, 0.2)
	b3.PrismaticJoint_SetMaxMotorForce(prisId, 8.0)

	// Spherical joint (3D-only)
	sphDef := b3.DefaultSphericalJointDef()
	sphDef.base.bodyIdA = jb[5]
	sphDef.base.bodyIdB = jb[6]
	sphId := b3.CreateSphericalJoint(worldId, &sphDef)
	b3.SphericalJoint_EnableConeLimit(sphId, true)
	b3.SphericalJoint_SetConeLimit(sphId, 0.5)
	b3.SphericalJoint_EnableTwistLimit(sphId, true)
	b3.SphericalJoint_SetTwistLimits(sphId, -0.3, 0.3)
	b3.SphericalJoint_EnableSpring(sphId, true)
	b3.SphericalJoint_SetSpringHertz(sphId, 3.0)
	b3.SphericalJoint_SetSpringDampingRatio(sphId, 0.5)
	b3.SphericalJoint_SetTargetRotation(sphId, b3.Quat_identity)
	b3.SphericalJoint_EnableMotor(sphId, true)
	b3.SphericalJoint_SetMotorVelocity(sphId, b3.Vec3{0.0, 0.1, 0.0})
	b3.SphericalJoint_SetMaxMotorTorque(sphId, 5.0)

	// Weld joint
	weldDef := b3.DefaultWeldJointDef()
	weldDef.base.bodyIdA = jb[6]
	weldDef.base.bodyIdB = jb[7]
	weldId := b3.CreateWeldJoint(worldId, &weldDef)
	b3.WeldJoint_SetLinearHertz(weldId, 5.0)
	b3.WeldJoint_SetLinearDampingRatio(weldId, 0.6)
	b3.WeldJoint_SetAngularHertz(weldId, 5.0)
	b3.WeldJoint_SetAngularDampingRatio(weldId, 0.6)

	// Wheel joint (Box3D uses Suspension/Spin/Steering naming)
	wheelDef := b3.DefaultWheelJointDef()
	wheelDef.base.bodyIdA = jb[7]
	wheelDef.base.bodyIdB = jb[8]
	wheelId := b3.CreateWheelJoint(worldId, &wheelDef)
	b3.WheelJoint_EnableSuspension(wheelId, true)
	b3.WheelJoint_SetSuspensionHertz(wheelId, 4.0)
	b3.WheelJoint_SetSuspensionDampingRatio(wheelId, 0.7)
	b3.WheelJoint_EnableSuspensionLimit(wheelId, true)
	b3.WheelJoint_SetSuspensionLimits(wheelId, -0.5, 0.5)
	b3.WheelJoint_EnableSpinMotor(wheelId, true)
	b3.WheelJoint_SetSpinMotorSpeed(wheelId, 1.0)
	b3.WheelJoint_SetMaxSpinTorque(wheelId, 6.0)
	b3.WheelJoint_EnableSteering(wheelId, true)
	b3.WheelJoint_SetSteeringHertz(wheelId, 2.0)
	b3.WheelJoint_SetSteeringDampingRatio(wheelId, 0.5)
	b3.WheelJoint_SetMaxSteeringTorque(wheelId, 3.0)
	b3.WheelJoint_EnableSteeringLimit(wheelId, true)
	b3.WheelJoint_SetSteeringLimits(wheelId, -0.5, 0.5)
	b3.WheelJoint_SetTargetSteeringAngle(wheelId, 0.1)

	// Parallel joint (3D-only)
	parallelDef := b3.DefaultParallelJointDef()
	parallelDef.base.bodyIdA = groundId
	parallelDef.base.bodyIdB = bodyId
	parallelId := b3.CreateParallelJoint(worldId, &parallelDef)
	b3.ParallelJoint_SetSpringHertz(parallelId, 2.0)
	b3.ParallelJoint_SetSpringDampingRatio(parallelId, 0.5)
	b3.ParallelJoint_SetMaxTorque(parallelId, 20.0)

	// Dynamic mover: a rotation locked capsule driven by a mover joint and held up by a pogo joint
	moverBodyDef := b3.DefaultBodyDef()
	moverBodyDef.type = .dynamicBody
	moverBodyDef.position = b3.Pos{-20.0, 3.0, 0.0}
	moverBodyDef.motionLocks = b3.MotionLocks {
		angularX = true,
		angularY = true,
		angularZ = true,
	}
	moverBodyId := b3.CreateBody(worldId, &moverBodyDef)
	moverShapeDef := b3.DefaultShapeDef()
	moverShapeDef.density = 1.0
	moverShapeDef.baseMaterial.friction = 0.0
	moverCapsule := b3.Capsule{{0.0, -0.5, 0.0}, {0.0, 0.5, 0.0}, 0.3}
	b3.CreateCapsuleShape(moverBodyId, &moverShapeDef, &moverCapsule)

	moverDef := b3.DefaultMoverJointDef()
	moverDef.base.bodyIdA = groundId
	moverDef.base.bodyIdB = moverBodyId
	moverDef.base.collideConnected = true
	moverDef.linearVelocity = b3.Vec3{1.0, 0.0, 0.5}
	moverDef.maxForce = 0.5
	moverJointId := b3.CreateMoverJoint(worldId, &moverDef)

	pogoDef := b3.DefaultPogoJointDef()
	pogoDef.base.bodyIdA = groundId
	pogoDef.base.bodyIdB = moverBodyId
	pogoDef.base.localFrameA.p = b3.Vec3{-20.0, 1.0, 0.0}
	pogoDef.base.localFrameB.p = moverCapsule.center1
	pogoDef.base.collideConnected = true
	pogoDef.restLength = 1.0
	pogoDef.hertz = 5.0
	pogoDef.dampingRatio = 0.8
	pogoDef.maxCompressionForce = 1000.0
	pogoDef.maxTensionForce = 1000.0
	pogoJointId := b3.CreatePogoJoint(worldId, &pogoDef)
	b3.PogoJoint_SetRestLength(pogoJointId, 0.9)
	b3.PogoJoint_SetSpringHertz(pogoJointId, 4.0)
	b3.PogoJoint_SetSpringDampingRatio(pogoJointId, 0.7)
	b3.PogoJoint_SetNormal(pogoJointId, b3.Normalize(b3.Vec3{0.2, 1.0, 0.0}))
	b3.PogoJoint_SetMaxTensionForce(pogoJointId, 800.0)
	b3.PogoJoint_SetMaxCompressionForce(pogoJointId, 900.0)

	// World config mutators
	b3.World_SetGravity(worldId, b3.Vec3{0.0, -9.8, 0.0})
	b3.World_EnableSleeping(worldId, true)
	b3.World_EnableContinuous(worldId, false)
	b3.World_EnableWarmStarting(worldId, true)
	b3.World_EnableSpeculative(worldId, true)
	b3.World_SetRestitutionThreshold(worldId, 1.5)
	b3.World_SetRestitutionIterations(worldId, 3)
	b3.World_EnableRestitutionPropagation(worldId, true)
	b3.World_SetHitEventThreshold(worldId, 2.0)
	b3.World_SetContactTuning(worldId, 30.0, 10.0, 3.0)
	b3.World_SetContactRecycleDistance(worldId, 0.05)
	b3.World_SetMaximumLinearSpeed(worldId, 100.0)
	b3.World_RebuildStaticTree(worldId)

	// The tolerance only reaches bodies whose mass is computed afterwards. A free spinning rod made
	// after the change loses its gyroscopic torque, so replay diverges if the op goes missing.
	b3.World_SetIsotropicInertiaTolerance(worldId, 1.0)
	{
		rodDef := b3.DefaultBodyDef()
		rodDef.type = .dynamicBody
		rodDef.position = b3.Pos{40.0, 10.0, 0.0}
		rodDef.gravityScale = 0.0
		rodDef.angularVelocity = b3.Vec3{1.0, 5.0, 0.5}
		rodId := b3.CreateBody(worldId, &rodDef)
		rod := b3.MakeBoxHull(0.1, 1.0, 0.1)
		rodShapeDef := b3.DefaultShapeDef()
		b3.CreateHullShape(rodId, &rodShapeDef, &rod.base)
	}

	explosion := b3.DefaultExplosionDef()
	explosion.position = b3.Pos{0.0, 5.0, 0.0}
	explosion.radius = 3.0
	explosion.falloff = 1.0
	explosion.impulsePerArea = 2.0
	b3.World_Explode(worldId, &explosion)

	// Pre-step queries
	qfilter := b3.DefaultQueryFilter()
	qaabb := b3.AABB{{-10.0, -5.0, -10.0}, {10.0, 15.0, 10.0}}
	b3.World_OverlapAABB(worldId, qaabb, qfilter, QueryReplayOverlapFcn, nil)
	qorigin := b3.Pos{0.0, 15.0, 0.0}
	proxyPts := b3.Vec3{0.0, 0.0, 0.0}
	proxy := b3.ShapeProxy{&proxyPts, 1, 0.5}
	b3.World_OverlapShape(worldId, qorigin, &proxy, qfilter, QueryReplayOverlapFcn, nil)
	qTranslation := b3.Vec3{0.0, -20.0, 0.0}
	b3.World_CastRay(worldId, qorigin, qTranslation, qfilter, QueryReplayCastFcn, nil)
	b3.World_CastRayClosest(worldId, qorigin, qTranslation, qfilter)
	b3.World_CastShape(worldId, qorigin, &proxy, qTranslation, qfilter, QueryReplayCastFcn, nil)
	mover := b3.Capsule{{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.3}
	b3.World_CastMover(worldId, qorigin, &mover, qTranslation, qfilter, QueryReplayMoverFilterFcn, nil)
	b3.World_CollideMover(worldId, qorigin, &mover, qfilter, QueryReplayPlaneFcn, nil)

	timeStep: f32 = 1.0 / 60.0
	subStepCount: i32 = 4
	for i := 0; i < 12; i += 1 {
		// Inject mutators mid-simulation
		if i == 6 {
			b3.Body_ApplyLinearImpulseToCenter(capsuleBodyId, b3.Vec3{2.0, 0.0, 0.0}, true)
			b3.Body_SetGravityScale(bodyId, 1.0)
			b3.Body_SetSafetyFactor(fastId, 0.4)
			b3.MoverJoint_SetMaxVelocityForce(moverJointId, 60.0)
		}

		// The dynamic mover rebuilds its pogo every step, warm started from the old one
		if i >= 6 {
			pogoDef.impulse = b3.PogoJoint_GetImpulse(pogoJointId)
			pogoDef.velocity = b3.PogoJoint_GetCurrentSpeed(pogoJointId)
			pogoDef.base.localFrameA.p = b3.Body_GetLocalPoint(groundId, b3.Body_GetPosition(moverBodyId))
			pogoDef.base.localFrameA.p.y = 1.0
			b3.DestroyJoint(pogoJointId, false)
			pogoJointId = b3.CreatePogoJoint(worldId, &pogoDef)
			b3.MoverJoint_SetLinearVelocity(moverJointId, b3.Vec3{-0.5, 0.0, 1.0})
		}
		// Issue queries mid-loop to exercise recording across steps
		if i == 3 {
			b3.World_OverlapAABB(worldId, qaabb, qfilter, QueryReplayOverlapFcn, nil)
			b3.World_CastRay(worldId, qorigin, qTranslation, qfilter, QueryReplayCastFcn, nil)
		}

		b3.World_Step(worldId, timeStep, subStepCount)
	}

	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)

	// Free geometry allocated for this subtest
	b3.DestroyHull(customHull)
	b3.DestroyMesh(meshData)
	b3.DestroyMesh(swapMeshData)
	b3.DestroyHeightField(hf)
	b3.DestroyCompound(compound)
	b3.DestroyMesh(compoundMeshData)

	recData := b3.Recording_GetData(rec)
	recSize := b3.Recording_GetSize(rec)
	ENSURE(t, recSize > 0) or_return

	// Replay headless at worker count 1 and 4, a cross-thread determinism check matching Box2D.
	ENSURE(t, b3.ValidateReplay(recData, recSize, 1)) or_return
	ENSURE(t, b3.ValidateReplay(recData, recSize, 4)) or_return

	// File round-trip
	ENSURE(t, b3.SaveRecordingToFile(rec, s_recPath)) or_return
	loaded := b3.LoadRecordingFromFile(s_recPath)
	ENSURE(t, loaded != nil) or_return
	ENSURE(t, b3.ValidateReplay(b3.Recording_GetData(loaded), b3.Recording_GetSize(loaded), 1)) or_return
	b3.DestroyRecording(loaded)

	// Drive the incremental player. Exercises per-frame stepping, restart, getters, and the
	// draw path beyond what b3ValidateReplay covers.
	{
		player := b3.CreatePlayer(recData, recSize, 1)
		ENSURE(t, player != nil) or_return

		// Capture keyframes while every geometry kind is live. The default interval is longer
		// than this recording.
		b3.RecPlayer_SetKeyframePolicy(player, b3.RecPlayer_GetKeyframeBudget(player), 4)

		info := b3.RecPlayer_GetInfo(player)
		recExtents := b3.Sub(info.bounds.upperBound, info.bounds.lowerBound)
		ENSURE(t, recExtents.x > 0.0 && recExtents.y > 0.0) or_return

		// Build a no-op b3DebugDraw to exercise the draw path headlessly
		dd := b3.DefaultDebugDraw()
		dd.DrawShapeFcn = RecTestDrawShape
		dd.drawShapes = true
		dd.drawingBounds = b3.AABB{{-1.0e6, -1.0e6, -1.0e6}, {1.0e6, 1.0e6, 1.0e6}}

		frames := 0
		for b3.RecPlayer_StepFrame(player) {
			if frames % 2 == 0 {
				b3.RecPlayer_DrawFrameQueries(player, &dd, -1, -1)
			}
			frames += 1
		}
		ENSURE(t, frames == 12) or_return
		ENSURE(t, b3.RecPlayer_GetFrame(player) == 12) or_return
		ENSURE(t, b3.RecPlayer_IsAtEnd(player)) or_return
		ENSURE(t, b3.RecPlayer_HasDiverged(player) == false) or_return
		ENSURE(t, b3.RecPlayer_GetKeyframeBytes(player) > 0) or_return

		// Seek back onto the frame 4 keyframe, then skip forward onto the frame 8 keyframe. Each
		// restore rebuilds the compound from the registry, so a ray must still reach its tree and
		// the state hashes must hold through the re-stepped frames.
		seekTargets := [2]i32{6, 9}
		for k := 0; k < 2; k += 1 {
			b3.RecPlayer_SeekFrame(player, seekTargets[k])
			ENSURE(t, b3.RecPlayer_GetFrame(player) == seekTargets[k]) or_return

			hitCompound := false
			b3.World_CastRay(
				b3.RecPlayer_GetWorldId(player),
				b3.Pos{30.0, 10.0, 0.0},
				b3.Vec3{0.0, -20.0, 0.0},
				b3.DefaultQueryFilter(),
				CompoundHitCastFcn,
				&hitCompound,
			)
			ENSURE(t, hitCompound) or_return
		}
		for b3.RecPlayer_StepFrame(player) {
		}
		ENSURE(t, b3.RecPlayer_GetFrame(player) == 12) or_return
		ENSURE(t, b3.RecPlayer_HasDiverged(player) == false) or_return

		// The trailing DestroyWorld is an end marker; the world stays valid after end
		ENSURE(t, b3.World_IsValid(b3.RecPlayer_GetWorldId(player))) or_return

		// Restart reproduces the same run without reloading the file
		b3.RecPlayer_Restart(player)
		ENSURE(t, b3.RecPlayer_GetFrame(player) == 0) or_return
		ENSURE(t, b3.RecPlayer_IsAtEnd(player) == false) or_return

		frames2 := 0
		for b3.RecPlayer_StepFrame(player) {
			frames2 += 1
		}
		ENSURE(t, frames2 == 12) or_return
		ENSURE(t, b3.RecPlayer_HasDiverged(player) == false) or_return

		b3.DestroyPlayer(player)
	}

	// The state hash only proves the op stream stayed aligned. Read the safety factor back out of
	// the replay world so a value that is written but never restored is caught too.
	{
		player := b3.CreatePlayer(recData, recSize, 1)
		ENSURE(t, player != nil) or_return

		// Frame 0 replays the pre-step creates, so the def value is in place
		ENSURE(t, b3.RecPlayer_StepFrame(player)) or_return
		ENSURE(t, ReplaySafetyFactor(player, "fastBody") == 0.1) or_return
		ENSURE(t, ReplaySafetyFactor(player, "dropBody") == 0.25) or_return

		// The setter is injected at frame 6
		for b3.RecPlayer_StepFrame(player) {
		}
		ENSURE(t, ReplaySafetyFactor(player, "fastBody") == 0.4) or_return

		b3.DestroyPlayer(player)
	}

	b3.DestroyRecording(rec)
	_ = os.remove(s_recPath)
	return true
}

// A transformed hull bakes its transform and non-uniform scale into fresh hull data at create time.
// It must be recorded like any other shape create, else its shape id allocation is invisible to the
// player and every later id drifts. A plain hull created after it would then mismatch on replay.
@(private = "file")
TransformedHullRoundTrip :: proc(t: ^testing.T) -> bool {
	pts := [8]b3.Vec3 {
		{-1.0, -1.0, -1.0},
		{1.0, -1.0, -1.0},
		{1.0, 1.0, -1.0},
		{-1.0, 1.0, -1.0},
		{-1.0, -1.0, 1.0},
		{1.0, -1.0, 1.0},
		{1.0, 1.0, 1.0},
		{-1.0, 1.0, 1.0},
	}
	hull := b3.CreateHull(&pts[0], 8, 8)
	ENSURE(t, hull != nil) or_return

	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	b3.World_StartRecording(worldId, rec)

	shapeDef := b3.DefaultShapeDef()
	shapeDef.density = 1.0

	// Baked transform with a rotation and non-uniform scale, the path Unreal uses for instanced
	// hulls.
	xf := b3.Transform{b3.Vec3{0.25, 0.0, -0.5}, b3.MakeQuatFromAxisAngle(b3.Vec3{0.0, 0.0, 1.0}, 0.3)}
	scl := b3.Vec3{1.5, 0.5, 2.0}
	for i := 0; i < 3; i += 1 {
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{f32(i * 3), 5.0, 0.0}
		bodyId := b3.CreateBody(worldId, &bodyDef)
		sid := b3.CreateTransformedHullShape(bodyId, &shapeDef, hull, xf, scl)
		ENSURE(t, b3.Shape_IsValid(sid)) or_return
	}

	// A plain hull after the transformed ones: if the transformed creates desynced the id pool,
	// this shape's recorded id would not match what replay allocates and b3ValidateReplay would
	// fail.
	{
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{0.0, 10.0, 0.0}
		bodyId := b3.CreateBody(worldId, &bodyDef)
		sid := b3.CreateHullShape(bodyId, &shapeDef, hull)
		ENSURE(t, b3.Shape_IsValid(sid)) or_return
	}

	// Step past the keyframe interval so replay captures a keyframe. That path re-serializes the
	// live world and interns the baked hull, which must already be in the pre-seeded registry.
	for i := 0; i < 20; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)
	}

	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)
	b3.DestroyHull(hull)

	ENSURE(t, b3.ValidateReplay(b3.Recording_GetData(rec), b3.Recording_GetSize(rec), 1)) or_return
	ENSURE(t, b3.ValidateReplay(b3.Recording_GetData(rec), b3.Recording_GetSize(rec), 4)) or_return

	b3.DestroyRecording(rec)
	return true
}

// Patch the reserved header bytes to nonzero and confirm b3ValidateReplay ignores them.
// Guards a future change that starts validating them or shrinks the header.
@(private = "file")
ReservedHeaderBytes :: proc(t: ^testing.T) -> bool {
	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)
	b3.World_StartRecording(worldId, rec)
	b3.World_SetGravity(worldId, b3.Vec3{0.0, -10.0, 0.0})

	bd := b3.DefaultBodyDef()
	bd.type = .dynamicBody
	bd.position = b3.Pos{0.0, 5.0, 0.0}
	bodyId := b3.CreateBody(worldId, &bd)
	s := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}
	sd := b3.DefaultShapeDef()
	sd.density = 1.0
	b3.CreateSphereShape(bodyId, &sd, &s)

	for i := 0; i < 10; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)
	}

	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)

	recData := b3.Recording_GetData(rec)
	recSize := b3.Recording_GetSize(rec)
	ENSURE(t, recSize >= i32(size_of(b3.RecHeader))) or_return

	// Patch reserved fields at their byte offsets in b3RecHeader:
	//   byte 11 : reserved  (uint8_t at offset 11)
	//   bytes 16-19 : reserved2 (uint32_t at offset 16)
	//   bytes 20-23 : reserved3 (uint32_t at offset 20)
	patched := ([^]u8)(b3.Alloc(int(recSize)))
	mem.copy(patched, recData, int(recSize))
	patched[11] = 0xAB
	patched[16] = 0xCD
	patched[17] = 0xEF
	patched[18] = 0x12
	patched[19] = 0x34
	patched[20] = 0x56
	patched[21] = 0x78
	patched[22] = 0x9A
	patched[23] = 0xBC
	ENSURE(t, b3.ValidateReplay(patched, recSize, 1)) or_return
	b3.Free(patched, int(recSize))

	b3.DestroyRecording(rec)
	return true
}

// A corrupt compound in the geometry registry must fail the replay cleanly. A stale version or a
// wrong kind is rejected when the shape is created and a truncated entry is rejected at load.
@(private = "file")
CorruptCompound :: proc(t: ^testing.T) -> bool {
	sphereDef: b3.CompoundSphereDef
	sphereDef.sphere = b3.Sphere{{0.0, 0.0, 0.0}, 1.0}
	sphereDef.material = b3.DefaultSurfaceMaterial()
	compoundDef := b3.CompoundDef{}
	compoundDef.spheres = &sphereDef
	compoundDef.sphereCount = 1
	compound := b3.CreateCompound(&compoundDef)
	ENSURE(t, compound != nil) or_return
	compoundByteCount := compound.byteCount

	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)
	b3.World_StartRecording(worldId, rec)

	bd := b3.DefaultBodyDef()
	groundId := b3.CreateBody(worldId, &bd)
	sd := b3.DefaultShapeDef()
	b3.CreateBakedCompoundShape(groundId, &sd, compound)

	bd.type = .dynamicBody
	bd.position = b3.Pos{0.0, 3.0, 0.0}
	bodyId := b3.CreateBody(worldId, &bd)
	s := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}
	b3.CreateSphereShape(bodyId, &sd, &s)

	for i := 0; i < 10; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)
	}

	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)
	b3.DestroyCompound(compound)

	recData := b3.Recording_GetData(rec)
	recSize := b3.Recording_GetSize(rec)
	ENSURE(t, b3.ValidateReplay(recData, recSize, 1)) or_return

	// The registry entry is a kind byte and a 4 byte length, then the blob, which leads with its
	// version
	version: u64 = b3.COMPOUND_VERSION
	blobOffset: i32 = -1
	for i: i32 = 5; i + i32(size_of(version)) <= recSize; i += 1 {
		if mem.compare_ptrs(&recData[i], &version, size_of(version)) == 0 {
			blobOffset = i
			break
		}
	}
	ENSURE(t, blobOffset > 0) or_return
	ENSURE(t, recData[blobOffset - 5] == u8(b3.GeometryKind.geometryCompound)) or_return
	storedByteCount: i32
	mem.copy(&storedByteCount, &recData[blobOffset - 4], size_of(storedByteCount))
	ENSURE(t, storedByteCount == compoundByteCount) or_return

	patched := ([^]u8)(b3.Alloc(int(recSize)))

	mem.copy(patched, recData, int(recSize))
	patched[blobOffset] ~= 0xFF
	ENSURE(t, b3.ValidateReplay(patched, recSize, 1) == false) or_return

	mem.copy(patched, recData, int(recSize))
	shortByteCount: i32 = 8
	mem.copy(&patched[blobOffset - 4], &shortByteCount, size_of(shortByteCount))
	ENSURE(t, b3.CreatePlayer(patched, recSize, 1) == nil) or_return

	mem.copy(patched, recData, int(recSize))
	zeroByteCount: i32 = 0
	mem.copy(&patched[blobOffset - 4], &zeroByteCount, size_of(zeroByteCount))
	ENSURE(t, b3.CreatePlayer(patched, recSize, 1) == nil) or_return

	mem.copy(patched, recData, int(recSize))
	patched[blobOffset - 5] = u8(b3.GeometryKind.geometryMesh)
	ENSURE(t, b3.ValidateReplay(patched, recSize, 1) == false) or_return

	b3.Free(patched, int(recSize))
	b3.DestroyRecording(rec)
	return true
}

// Geometry that shares a content hash but differs in bytes must dedup exactly. Mirrors the keyframe
// flow that crashed: the seed appends every slot 1:1 (even byte-identical duplicates a hash
// collision left in an already-recorded file), then capture must resolve a live blob back to an
// existing slot without growing the registry. Forces the collision by handing the same hash to
// distinct blobs.
@(private = "file")
GeometryHashCollision :: proc(t: ^testing.T) -> bool {
	n :: 16
	sharedHash: u64 : 0xABCD1234

	reg := b3.GeometryRegistry{}

	blobA: [n]u8
	blobB: [n]u8
	mem.set(&blobA, 0xAA, n)
	mem.set(&blobB, 0xBB, n)

	// Distinct blobs colliding on the hash must become two entries, not a false dedup.
	idA := b3.InternGeometry(&reg, .geometryHull, sharedHash, &blobA[0], n)
	idB := b3.InternGeometry(&reg, .geometryHull, sharedHash, &blobB[0], n)
	ENSURE(t, idA != idB) or_return
	ENSURE(t, reg.entries.count == 2) or_return

	// Re-interning either blob must find it through the hash chain and never grow the registry,
	// including the one shadowed behind the bucket head. The old single-entry lookup missed the
	// shadowed blob and appended a duplicate, which is exactly what tripped the keyframe assert.
	blobA2: [n]u8
	mem.set(&blobA2, 0xAA, n)
	ENSURE(t, b3.InternGeometry(&reg, .geometryHull, sharedHash, &blobA2[0], n) == idA) or_return
	ENSURE(t, reg.entries.count == 2) or_return

	blobB2: [n]u8
	mem.set(&blobB2, 0xBB, n)
	ENSURE(t, b3.InternGeometry(&reg, .geometryHull, sharedHash, &blobB2[0], n) == idB) or_return
	ENSURE(t, reg.entries.count == 2) or_return

	b3.FreeRegistry(&reg)

	// Seed-then-capture: appending byte-identical duplicate slots keeps id == slot index, and a
	// later exact intern still resolves to one of them without appending a new entry.
	seeded := b3.GeometryRegistry{}
	slot0: [n]u8
	slot1: [n]u8
	slot2: [n]u8
	mem.set(&slot0, 0xAA, n)
	mem.set(&slot1, 0xBB, n)
	mem.set(&slot2, 0xAA, n) // duplicate of slot0
	ENSURE(t, b3.AppendGeometry(&seeded, .geometryHull, sharedHash, &slot0[0], n) == 0) or_return
	ENSURE(t, b3.AppendGeometry(&seeded, .geometryHull, sharedHash, &slot1[0], n) == 1) or_return
	ENSURE(t, b3.AppendGeometry(&seeded, .geometryHull, sharedHash, &slot2[0], n) == 2) or_return

	live: [n]u8
	mem.set(&live, 0xAA, n)
	resolved := b3.InternGeometry(&seeded, .geometryHull, sharedHash, &live[0], n)
	ENSURE(t, seeded.entries.count == 3) or_return // no growth
	ENSURE(t, resolved == 0 || resolved == 2) or_return // a valid slot index for that content
	ENSURE(t, seeded.entries.data[resolved].byteCount == n) or_return
	ENSURE(t, mem.compare_ptrs(seeded.entries.data[resolved].bytes, &slot0[0], n) == 0) or_return

	b3.FreeRegistry(&seeded)
	return true
}

// Staged stepping must reveal a mid-stream body at its creation transform. A body created and given
// an impulse in one recorded step is first placed by CreateBody, then displaced by the following
// Step. Atomic replay fuses the two, so the body is only ever seen already moved. Staged replay
// parks between them so the pre-integration pose is drawable. Verify the parked pose is the
// creation transform, that atomic replay does not show it, and that the extra park does not perturb
// the end state.
@(private = "file")
StagedStepCreationPose :: proc(t: ^testing.T) -> bool {
	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)
	b3.World_SetGravity(worldId, b3.Vec3{0.0, -10.0, 0.0})

	b3.World_StartRecording(worldId, rec)

	timeStep: f32 = 1.0 / 60.0
	subStepCount: i32 = 4

	// A few empty steps so the creation lands inside the stream, not at frame 0.
	leadFrames: i32 : 3
	for i: i32 = 0; i < leadFrames; i += 1 {
		b3.World_Step(worldId, timeStep, subStepCount)
	}

	// The creation transform under test. No ground, so the body is the only create and stays
	// ballistic. Components are exact in float so the round-trip pose compares tight.
	spawn :: b3.Pos{1.0, 20.0, -2.0}
	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = spawn
	bodyId := b3.CreateBody(worldId, &bodyDef)

	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}
	shapeDef := b3.DefaultShapeDef()
	shapeDef.density = 1.0
	b3.CreateSphereShape(bodyId, &shapeDef, &sphere)

	// Impulse along +x so the first integrated step moves the body a visible distance off the
	// spawn.
	b3.Body_ApplyLinearImpulseToCenter(bodyId, b3.Vec3{5.0, 0.0, 0.0}, true)

	creationFrame: i32 : leadFrames + 1
	b3.World_Step(worldId, timeStep, subStepCount) // advances to creationFrame

	totalFrames: i32 : creationFrame + 3
	for i: i32 = creationFrame; i < totalFrames; i += 1 {
		b3.World_Step(worldId, timeStep, subStepCount)
	}

	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)

	data := b3.Recording_GetData(rec)
	sz := b3.Recording_GetSize(rec)

	// Atomic replay fuses the create and its step, so at the creation frame the body is already
	// integrated and displaced along +x. Capture that pose and the final state hash.
	atomic := b3.CreatePlayer(data, sz, 1)
	ENSURE(t, atomic != nil) or_return
	atomicPose := b3.Pos{0.0, 0.0, 0.0}
	for !b3.RecPlayer_IsAtEnd(atomic) {
		b3.RecPlayer_StepFrame(atomic)
		if b3.RecPlayer_GetFrame(atomic) == creationFrame {
			id := b3.RecPlayer_GetBodyId(atomic, 0)
			ENSURE(t, b3.Body_IsValid(id)) or_return
			atomicPose = b3.Body_GetPosition(id)
		}
	}
	ENSURE(t, b3.RecPlayer_GetFrame(atomic) == totalFrames) or_return
	ENSURE(t, !b3.RecPlayer_HasDiverged(atomic)) or_return
	ENSURE(t, atomicPose.x - spawn.x > 0.01) or_return // the impulse moved it before it was ever seen
	atomicHash := b3.HashWorldState(b3.GetWorldFromId(b3.RecPlayer_GetWorldId(atomic)))
	b3.DestroyPlayer(atomic)

	// Staged replay: step forward until the first pre-step park. It must sit at the creation
	// frame's pre-integration state with the new body at exactly its spawn transform.
	staged := b3.CreatePlayer(data, sz, 1)
	ENSURE(t, staged != nil) or_return
	for !b3.RecPlayer_IsAtEnd(staged) {
		b3.RecPlayer_SubStepFrame(staged)
		if b3.RecPlayer_IsAtPreStep(staged) {
			break
		}
	}
	ENSURE(t, b3.RecPlayer_IsAtPreStep(staged)) or_return
	ENSURE(t, b3.RecPlayer_GetFrame(staged) == creationFrame - 1) or_return // parked before the step, frame not advanced

	stagedId := b3.RecPlayer_GetBodyId(staged, 0)
	ENSURE(t, b3.Body_IsValid(stagedId)) or_return
	parkPose := b3.Body_GetPosition(stagedId)
	ENSURE_SMALL(t, parkPose.x - spawn.x, 1.0e-4) or_return
	ENSURE_SMALL(t, parkPose.y - spawn.y, 1.0e-4) or_return
	ENSURE_SMALL(t, parkPose.z - spawn.z, 1.0e-4) or_return

	// Finishing the frame integrates the body, so it leaves the spawn pose on the next advance.
	b3.RecPlayer_SubStepFrame(staged)
	ENSURE(t, b3.RecPlayer_GetFrame(staged) == creationFrame) or_return
	ENSURE(t, !b3.RecPlayer_IsAtPreStep(staged)) or_return

	// Run staged to the end: the same op stream, so it must land on the atomic end state bit for
	// bit.
	for !b3.RecPlayer_IsAtEnd(staged) {
		b3.RecPlayer_SubStepFrame(staged)
	}
	ENSURE(t, b3.RecPlayer_GetFrame(staged) == totalFrames) or_return
	ENSURE(t, !b3.RecPlayer_HasDiverged(staged)) or_return
	stagedHash := b3.HashWorldState(b3.GetWorldFromId(b3.RecPlayer_GetWorldId(staged)))
	ENSURE(t, stagedHash == atomicHash) or_return
	b3.DestroyPlayer(staged)

	b3.DestroyRecording(rec)
	return true
}

// Shape names are debug only and do not feed the determinism hash, so b3ValidateReplay cannot catch
// a broken name round-trip. Replay through the player and read the names back to prove the def
// field and the ShapeSetName op survive serialization.
@(private = "file")
ShapeNameReplay :: proc(t: ^testing.T) -> bool {
	names := [3]cstring{"def", "set", "abcdefghijklmnopqrstuvwxyz"}

	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return

	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)
	b3.World_StartRecording(worldId, rec)

	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}

	// One shape per body so read-back never depends on per-body shape ordering.
	for i := 0; i < 3; i += 1 {
		bd := b3.DefaultBodyDef()
		bd.type = .dynamicBody
		bd.position = b3.Pos{3.0 * f32(i), 5.0, 0.0}
		body := b3.CreateBody(worldId, &bd)

		sd := b3.DefaultShapeDef()
		sd.density = 1.0
		if i != 1 {
			sd.name = names[i]
		}
		shape := b3.CreateSphereShape(body, &sd, &sphere)
		if i == 1 {
			b3.Shape_SetName(shape, names[i])
		}
	}

	for i := 0; i < 4; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)
	}

	b3.World_StopRecording(worldId)
	b3.DestroyWorld(worldId)

	data := b3.Recording_GetData(rec)
	sz := b3.Recording_GetSize(rec)

	player := b3.CreatePlayer(data, sz, 1)
	ENSURE(t, player != nil) or_return
	for b3.RecPlayer_StepFrame(player) {
	}
	ENSURE(t, b3.RecPlayer_HasDiverged(player) == false) or_return

	// Body ordinals follow creation order in the replayed world.
	for i := 0; i < 3; i += 1 {
		body := b3.RecPlayer_GetBodyId(player, i32(i))
		ENSURE(t, b3.Body_IsValid(body)) or_return

		shape: b3.ShapeId
		ENSURE(t, b3.Body_GetShapes(body, &shape, 1) == 1) or_return
		got := b3.Shape_GetName(shape)
		ENSURE(t, got != nil) or_return

		srcLen := len(names[i])
		ENSURE(t, len(got) == srcLen) or_return
		if srcLen > 0 {
			ENSURE(t, string(got)[:srcLen] == string(names[i])[:srcLen]) or_return
		}
	}

	b3.DestroyPlayer(player)
	b3.DestroyRecording(rec)
	return true
}

// A box sliding across a mesh floor, with the option to swap both geometries and retune a
// per-triangle material part way through. Returns the final state hash so the caller can prove the
// mutations move the simulation. Recording is optional so the same scene serves as the control.
@(private = "file")
RunGeometryMutatorScene :: proc(
	rec: ^b3.Recording,
	mutate: bool,
	meshA: ^b3.MeshData,
	meshB: ^b3.MeshData,
	swapHull: ^b3.HullData,
	swapFriction: f32,
) -> u64 {
	worldDef := b3.DefaultWorldDef()
	worldDef.workerCount = 1
	worldId := b3.CreateWorld(&worldDef)

	if rec != nil {
		b3.World_StartRecording(worldId, rec)
	}

	// The mesh body is created first so its ordinal is stable for the read back after replay.
	meshMaterials := [2]b3.SurfaceMaterial{b3.DefaultSurfaceMaterial(), b3.DefaultSurfaceMaterial()}
	meshMaterials[1].friction = 0.05

	meshBodyDef := b3.DefaultBodyDef()
	meshBodyDef.type = .staticBody
	meshBodyId := b3.CreateBody(worldId, &meshBodyDef)

	meshShapeDef := b3.DefaultShapeDef()
	meshShapeDef.materials = &meshMaterials[0]
	meshShapeDef.materialCount = 2
	meshShapeId := b3.CreateMeshShape(meshBodyId, &meshShapeDef, meshA, b3.Vec3{1.0, 1.0, 1.0})

	boxBodyDef := b3.DefaultBodyDef()
	boxBodyDef.type = .dynamicBody
	boxBodyDef.position = b3.Pos{-2.0, 2.0, 0.0}
	boxBodyDef.linearVelocity = b3.Vec3{4.0, 0.0, 0.0}
	boxBodyId := b3.CreateBody(worldId, &boxBodyDef)

	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	boxShapeDef := b3.DefaultShapeDef()
	boxShapeDef.density = 1.0
	boxShapeId := b3.CreateHullShape(boxBodyId, &boxShapeDef, &box.base)

	timeStep: f32 = 1.0 / 60.0
	for i := 0; i < 10; i += 1 {
		b3.World_Step(worldId, timeStep, 4)
	}

	if mutate {
		b3.Shape_SetHull(boxShapeId, swapHull)
		b3.Shape_SetMesh(meshShapeId, meshB, b3.Vec3{1.0, 1.0, 1.0})

		grippy := b3.DefaultSurfaceMaterial()
		grippy.friction = swapFriction
		b3.Shape_SetMeshMaterial(meshShapeId, grippy, 1)
	}

	for i := 0; i < 30; i += 1 {
		b3.World_Step(worldId, timeStep, 4)
	}

	hash := b3.HashWorldState(b3.GetWorldFromId(worldId))

	if rec != nil {
		b3.World_StopRecording(worldId)
	}
	b3.DestroyWorld(worldId)

	return hash
}

// Swapping a shape's hull or mesh, or retuning one of its per-triangle materials, is a world
// mutation like any other and has to ride the stream. The geometry pair interns into the registry
// at the record site so replay rebuilds the same shape instead of running on the geometry it was
// created with. The control run proves the mutations move the simulation, so the state hash gate
// has teeth, and the read back covers each op on its own where dynamics alone would not.
@(private = "file")
GeometryMutatorReplay :: proc(t: ^testing.T) -> bool {
	// Two flat floors with different triangulations, both carrying two material slots so the
	// material index stays live across the swap.
	meshA := b3.CreateGridMesh(8, 8, 2.0, 2, false)
	ENSURE(t, meshA != nil) or_return
	meshB := b3.CreateGridMesh(12, 12, 1.5, 2, false)
	ENSURE(t, meshB != nil) or_return
	ENSURE(t, meshA.triangleCount != meshB.triangleCount) or_return

	swapHull := b3.MakeBoxHull(0.25, 1.5, 0.25)
	swapFriction: f32 : 0.95

	controlHash := RunGeometryMutatorScene(nil, false, meshA, meshB, &swapHull.base, swapFriction)

	rec := b3.CreateRecording(0)
	ENSURE(t, rec != nil) or_return
	mutatedHash := RunGeometryMutatorScene(rec, true, meshA, meshB, &swapHull.base, swapFriction)

	// Without this the replay gate below could pass on a recording that never carried the ops.
	ENSURE(t, mutatedHash != controlHash) or_return

	data := b3.Recording_GetData(rec)
	size := b3.Recording_GetSize(rec)
	ENSURE(t, size > 0) or_return

	ENSURE(t, b3.ValidateReplay(data, size, 1)) or_return
	ENSURE(t, b3.ValidateReplay(data, size, 4)) or_return

	player := b3.CreatePlayer(data, size, 1)
	ENSURE(t, player != nil) or_return
	for b3.RecPlayer_StepFrame(player) {
	}
	ENSURE(t, b3.RecPlayer_HasDiverged(player) == false) or_return

	// Body ordinals follow creation order in the replayed world.
	replayMeshBody := b3.RecPlayer_GetBodyId(player, 0)
	replayBoxBody := b3.RecPlayer_GetBodyId(player, 1)
	ENSURE(t, b3.Body_IsValid(replayMeshBody) && b3.Body_IsValid(replayBoxBody)) or_return

	replayMeshShape: b3.ShapeId
	ENSURE(t, b3.Body_GetShapes(replayMeshBody, &replayMeshShape, 1) == 1) or_return
	replayBoxShape: b3.ShapeId
	ENSURE(t, b3.Body_GetShapes(replayBoxBody, &replayBoxShape, 1) == 1) or_return

	replayHull := b3.Shape_GetHull(replayBoxShape)
	ENSURE(t, replayHull != nil) or_return
	ENSURE(t, replayHull.hash == swapHull.base.hash) or_return

	replayMesh := b3.Shape_GetMesh(replayMeshShape)
	ENSURE(t, replayMesh.data != nil) or_return
	ENSURE(t, replayMesh.data.hash == meshB.hash) or_return
	ENSURE(t, replayMesh.data.triangleCount == meshB.triangleCount) or_return

	replayMaterial := b3.Shape_GetMeshSurfaceMaterial(replayMeshShape, 1)
	ENSURE(t, replayMaterial.friction == swapFriction) or_return

	b3.DestroyPlayer(player)
	b3.DestroyRecording(rec)
	b3.DestroyMesh(meshA)
	b3.DestroyMesh(meshB)
	return true
}

@(test)
RecordingTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, GeometryHashCollision) {return}
	if !RUN_SUBTEST(t, ShapeNameReplay) {return}
	if !RUN_SUBTEST(t, SphereRoundTrip) {return}
	if !RUN_SUBTEST(t, EmptyWorldRoundTrip) {return}
	if !RUN_SUBTEST(t, HullDedup) {return}
	if !RUN_SUBTEST(t, MidStreamNoContacts) {return}
	if !RUN_SUBTEST(t, MidStreamContacts) {return}
	if !RUN_SUBTEST(t, CrossWidthReplay) {return}
	if !RUN_SUBTEST(t, StagedStepCreationPose) {return}
	if !RUN_SUBTEST(t, ScrubBackward) {return}
	if !RUN_SUBTEST(t, SeekWithHull) {return}
	if !RUN_SUBTEST(t, DebugShapeCallbacks) {return}
	if !RUN_SUBTEST(t, PlayerAccessors) {return}
	if !RUN_SUBTEST(t, KeyframeHandleReuse) {return}
	if !RUN_SUBTEST(t, QueryReplay) {return}
	if !RUN_SUBTEST(t, TaggedQuery) {return}
	if !RUN_SUBTEST(t, TransformedHullRoundTrip) {return}
	if !RUN_SUBTEST(t, GeometryMutatorReplay) {return}
	if !RUN_SUBTEST(t, AllOps) {return}
	if !RUN_SUBTEST(t, ReservedHeaderBytes) {return}
	if !RUN_SUBTEST(t, CorruptCompound) {return}
}
