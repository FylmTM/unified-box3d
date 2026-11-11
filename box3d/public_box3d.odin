// Ported from box3d include/box3d/box3d.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

// unified: box3d.h declares the API, implemented in the .c files' ports; only the types that live in it are
// here. The opaque handles (b3Recording, b3RecPlayer, b3WorldStateCache, b3WorldState) are defined by the files
// that implement them. The BOX3D_DOUBLE_PRECISION b3CreateWorld rename is not ported (single precision only).

// Summary of a recording, read once at open so a viewer can frame and label it.
RecPlayerInfo :: struct {
	frameCount:   i32, // total recorded steps
	workerCount:  i32, // worker count requested for the replay world
	timeStep:     f32, // dt of the recorded steps
	subStepCount: i32, // recorded sub-steps
	lengthScale:  f32, // length units per meter in effect when recorded
	bounds:       AABB, // accumulated world bounds over the recording, zero-extent if unavailable
}

// The kind of a recorded spatial query, matching the public query and cast functions.
RecQueryType :: enum i32 {
	recQueryOverlapAABB,
	recQueryOverlapShape,
	recQueryCastRay,
	recQueryCastShape,
	recQueryCastRayClosest,
	recQueryCastMover,
	recQueryCollideMover,
}

// A spatial query recorded during a replayed frame, exposed for inspection.
RecQueryInfo :: struct {
	type:        RecQueryType,
	filter:      QueryFilter,
	aabb:        AABB, // world-space bounds of the query, swept for casts
	origin:      Pos, // query origin (zero for overlap AABB)
	translation: Vec3, // ray and cast translation
	hitCount:    i32, // number of recorded results
	key:         u64, // identity key, the hash of (id, name), 0 if untagged
	id:          u64, // query id, 0 if none
	name:        cstring, // query label, NULL if none
}

// One result of a recorded spatial query.
RecQueryHit :: struct {
	shape:    ShapeId,
	point:    Pos,
	normal:   Vec3,
	fraction: f32,
}
