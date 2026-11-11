// Ported from box3d src/world_state.c and its declarations in include/box3d/box3d.h at 3362633
// (PORTING.md).
// unified: this whole file is unified's fork of Box3D (third_party/box3d.patch), not upstream.
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

// box3d.h: World state (FORK)
// Save and restore the full simulation state of a world for rollback.
// A state is a raw struct image of the world (bodies, shapes, contacts with warm-start impulses,
// islands and sleep sets, broad-phase trees, constraint graph, id pools). Restore overwrites the
// live world in place, so body/shape/joint ids saved with the state are valid again afterwards.
// Not captured: userData pointers (restored as NULL), host callbacks, pending events.
// Images are only valid for the same build and the same b3WorldStateCache.

// world_state.c

// FORK: public world save/restore for rollback, a thin wrapper over the internal
// snapshot serializer that the recording player uses for keyframes (world_snapshot.c).
// Save = b3SerializeWorld into a reusable buffer, interning shape geometry into a registry
// owned by the cache. Restore = b3DeserializeIntoShell in place over the live world, exactly
// like b3RecPlayerRestoreKeyframe, with a reader whose registry slots mirror the cache.

// Opaque geometry registry shared by every save/restore of a world. Restored mesh, height
// field and compound shapes point into it, so it must outlive the world.
WorldStateCache :: struct {
	// Only the geometry registry of this recording is used; its op buffer stays empty.
	registry:     ^Recording,

	// Reader view of the registry for b3DeserializeIntoShell. Entries are append only and each
	// owns its bytes, so the slots only need extending when the registry grows.
	reader:       RecReader,
	slotCapacity: i32,
}

// Opaque, reusable state buffer. Saving into the same buffer again reuses its memory.
WorldState :: struct {
	buffer: RecBuffer,
}

CreateWorldStateCache :: proc() -> ^WorldStateCache {
	cache := cast(^WorldStateCache)Alloc(size_of(WorldStateCache))
	cache^ = {}
	cache.registry = CreateRecording(64)
	return cache
}

DestroyWorldStateCache :: proc(cache: ^WorldStateCache) {
	if cache == nil {
		return
	}
	if cache.reader.slots != nil {
		Free(cache.reader.slots, int(cache.slotCapacity) * size_of(RegistrySlot))
	}
	DestroyRecording(cache.registry)
	Free(cache, size_of(WorldStateCache))
}

CreateWorldState :: proc() -> ^WorldState {
	state := cast(^WorldState)Alloc(size_of(WorldState))
	state^ = {}
	return state
}

DestroyWorldState :: proc(state: ^WorldState) {
	if state == nil {
		return
	}
	RecBufFree(&state.buffer)
	Free(state, size_of(WorldState))
}

WorldState_GetData :: proc(state: ^WorldState) -> [^]u8 {
	return state.buffer.data
}

WorldState_GetSize :: proc(state: ^WorldState) -> i32 {
	return state.buffer.size
}

// Save the world into state. Call at a step boundary. Returns the image size in bytes.
World_SaveState :: proc(worldId: WorldId, cache: ^WorldStateCache, state: ^WorldState) -> i32 {
	world := GetWorldFromId(worldId)
	assert(world.locked == false)
	if world.locked {
		return 0
	}

	// Reuse the buffer capacity: a steady-state save does not allocate.
	state.buffer.size = 0
	state.buffer.countOnly = false
	return SerializeWorld(world, &state.buffer, cache.registry)
}

// Restore the world in place from a state saved with the same cache. Returns false on a bad image.
World_RestoreState :: proc(worldId: WorldId, cache: ^WorldStateCache, state: ^WorldState) -> bool {
	world := GetWorldFromId(worldId)
	assert(world.locked == false)
	if world.locked || state.buffer.size == 0 {
		return false
	}

	// unified: the registry slot mirroring and the restore are World_RestoreStateFromBytes over the buffer
	return World_RestoreStateFromBytes(worldId, cache, state.buffer.data[:state.buffer.size])
	// unified end
}

// unified: restore from a state image held anywhere (e.g. one received over the network or kept in a
// ring buffer) without copying it into a b3WorldState first. Same contract as b3World_RestoreState.
World_RestoreStateFromBytes :: proc(worldId: WorldId, cache: ^WorldStateCache, bytes: []u8) -> bool {
	world := GetWorldFromId(worldId)
	assert(world.locked == false)
	if world.locked || len(bytes) == 0 || len(bytes) > int(max(i32)) {
		return false
	}

	// Mirror any registry entries added since the last restore into reader slots (id == index).
	reg := &cache.registry.registry
	count := reg.entries.count
	if count > cache.slotCapacity {
		newCap := count < 8 ? 8 : 2 * count
		cache.reader.slots = cast([^]RegistrySlot)GrowAlloc(
			cache.reader.slots,
			cache.slotCapacity * i32(size_of(RegistrySlot)),
			newCap * i32(size_of(RegistrySlot)),
		)
		cache.slotCapacity = newCap
	}
	for i := cache.reader.slotCount; i < count; i += 1 {
		entry := &reg.entries.data[i]
		cache.reader.slots[i] = RegistrySlot{entry.kind, entry.byteCount, entry.bytes}
	}
	cache.reader.slotCount = count

	return DeserializeIntoShell(raw_data(bytes), i32(len(bytes)), world, &cache.reader)
}
// unified end
