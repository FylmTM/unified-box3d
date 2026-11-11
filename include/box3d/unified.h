// SPDX-FileCopyrightText: 2026 unified-box3d contributors
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d.h"

// unified-box3d's additions to the Box3D API (SPEC.md): in-process snapshots for rollback, canonical world images
// (portable: the same bytes on every platform, for late join and checksums), and the world-state API of our former
// C fork. Include this after (or instead of) box3d.h. Everything here is a step-boundary operation: call it between
// b3World_Step calls, never from a callback.


/**
 * @defgroup snapshot World snapshots
 * In-process snapshots: a copy of the world's memory, restored in place (a rollback). A snapshot is only valid in
 * the process and for the world that saved it, at any earlier step of that world. Ids saved with it are valid
 * again after the restore.
 * @{
 */

/// A snapshot buffer, owned by the caller. Zero-initialize it; saving into it again reuses its memory (it grows
/// when the world does). Free it with b3DestroyWorldSnapshot.
typedef struct b3WorldSnapshot
{
	uint8_t* data;
	int64_t size;
	int64_t capacity;
} b3WorldSnapshot;

/// Copy the world into the snapshot. Call between steps.
B3_API void b3World_SaveSnapshot( b3WorldId worldId, b3WorldSnapshot* snapshot );

/// Put the world back to the snapshot's state. Returns false if the snapshot is not from this world.
B3_API bool b3World_RestoreSnapshot( b3WorldId worldId, b3WorldSnapshot* snapshot );

/// Free the snapshot's memory and zero it.
B3_API void b3DestroyWorldSnapshot( b3WorldSnapshot* snapshot );

/**@}*/ // snapshot

/**
 * @defgroup world_state World state and images
 * Serialized world states. A canonical image (b3World_SaveImage) has identical bytes for identical states on every
 * platform; with its geometry it is self-contained and loads into a fresh world in another process (late join).
 * The b3World_SaveState family is the former C fork's API: images that name shape geometry through a
 * b3WorldStateCache shared by every save and restore of a world.
 * @{
 */

/// Opaque geometry registry shared by every save/restore of a world. Restored mesh, height
/// field and compound shapes point into it, so it must outlive the world.
typedef struct b3WorldStateCache b3WorldStateCache;

/// Opaque, reusable state buffer. Saving into the same buffer again reuses its memory.
typedef struct b3WorldState b3WorldState;

B3_API b3WorldStateCache* b3CreateWorldStateCache( void );
B3_API void b3DestroyWorldStateCache( b3WorldStateCache* cache );
B3_API b3WorldState* b3CreateWorldState( void );
B3_API void b3DestroyWorldState( b3WorldState* state );
B3_API const uint8_t* b3WorldState_GetData( const b3WorldState* state );
B3_API int b3WorldState_GetSize( const b3WorldState* state );

/// Save the world into state. Call at a step boundary. Returns the image size in bytes.
B3_API int b3World_SaveState( b3WorldId worldId, b3WorldStateCache* cache, b3WorldState* state );

/// Restore the world in place from a state saved with the same cache. Returns false on a bad image.
B3_API bool b3World_RestoreState( b3WorldId worldId, b3WorldStateCache* cache, const b3WorldState* state );

/// Restore from a state image held anywhere (one received over the network, or kept in a ring buffer) without
/// copying it into a b3WorldState first. Same contract as b3World_RestoreState; also loads canonical images.
B3_API bool b3World_RestoreStateFromBytes( b3WorldId worldId, b3WorldStateCache* cache, const uint8_t* bytes,
										   int size );

/// Save the canonical image of the world into state. With geometry, the image is self-contained (late join);
/// without, it names geometry by content hash and loads only into this world (or one that has the same geometry).
/// Returns the image size in bytes.
B3_API int b3World_SaveImage( b3WorldId worldId, b3WorldState* state, bool withGeometry );

/// Load an image saved by b3World_SaveImage (in any process, for a self-contained image) over the world, in place.
/// Returns false on a bad image; an image naming geometry this world lacks is refused before the world is touched.
B3_API bool b3World_LoadImage( b3WorldId worldId, const uint8_t* bytes, int size );

/// The checksum of the world: a hash of its canonical image, with geometry by content hash. Equal states give equal
/// hashes in any process.
B3_API uint64_t b3World_HashState( b3WorldId worldId );

/**@}*/ // world_state

