// Ported from box3d src/name_cache.c and src/name_cache.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:runtime"
import "core:mem"

NULL_NAME :: u32(0)

NameEntry :: struct {
	// Names are used for debugging and I'm favoring simplicity and
	// minimal storage. Collisions will be logged.
	hash:   u32,
	length: i32,
	name:   cstring,
}

NameCache :: struct {
	entries: Array(NameEntry),
	map_:    rawptr,
}

FindNameWithDefault :: proc(cache: ^NameCache, id: u32, def: cstring) -> cstring {
	name := FindName(cache, id)
	return name == nil ? def : name
}

// unified: the verstable b3NameMap (u32 -> int) is an Odin map on the heap allocator; it is only used for lookups.
// Its storage is not counted by GetByteCount.
@(private = "file")
NameMap :: map[u32]i32
// unified end

FNV32_OFFSET_BASIS :: 0x811c9dc5
FNV32_PRIME :: 0x01000193

// This is designed to hash small strings and while avoiding collisions
// for a 32-bit hash.
// Note: changing this will break recordings
Hash32 :: proc "contextless" (data: rawptr, length: int) -> u32 {
	// Cast to unsigned to avoid sign extension
	p := ([^]u8)(data)

	// FNV-1a
	h: u32 = FNV32_OFFSET_BASIS
	for i := 0; i < length; i += 1 {
		h ~= u32(p[i])
		h *= FNV32_PRIME
	}

	// Murmur3 finalizer to help with short strings
	h ~= h >> 16
	h *= 0x85ebca6b
	h ~= h >> 13
	h *= 0xc2b2ae35
	h ~= h >> 16
	return h
}

CreateNameCache :: proc() -> NameCache {
	HostScope() // unified: the name cache is host-side and append-only (world_arena.odin)
	cache := NameCache{}
	Array_Create(&cache.entries)

	m := cast(^NameMap)AllocHost(size_of(NameMap))
	// unified: b3NameMap_init
	m^ = make(NameMap, runtime.heap_allocator())
	// unified end
	cache.map_ = m
	return cache
}

DestroyNameCache :: proc(cache: ^NameCache) {
	count := cache.entries.count
	for i: i32 = 0; i < count; i += 1 {
		entry := &cache.entries.data[i]
		Free(rawptr(entry.name), int(entry.length + 1))
	}

	Array_Destroy(&cache.entries)
	// unified: b3NameMap_cleanup
	delete((^NameMap)(cache.map_)^)
	// unified end
	Free(cache.map_, size_of(NameMap))
}

AddName :: proc(cache: ^NameCache, name: cstring) -> u32 {
	HostScope() // unified: the name cache is host-side and append-only (world_arena.odin)
	if name == nil || ([^]u8)(name)[0] == 0 {
		return NULL_NAME
	}

	length := len(name)
	id := Hash32(rawptr(name), length)

	id = id == 0 ? 1 : id

	m := (^NameMap)(cache.map_)
	if index, found := m[id]; found {
		if string(cache.entries.data[index].name) != string(name) {
			// Different name, same hash
			Log("Hash collision on %s", name)
		}
		return id
	}

	clone := cast([^]u8)AllocHost(length + 1)
	mem.copy_non_overlapping(clone, rawptr(name), length + 1)
	index := cache.entries.count
	entry := NameEntry {
		hash   = id,
		name   = cstring(clone),
		length = i32(length),
	}

	Array_Push(&cache.entries, entry)

	m[id] = index
	return id
}

FindName :: proc(cache: ^NameCache, id: u32) -> cstring {
	m := (^NameMap)(cache.map_)
	if index, found := m[id]; found {
		entry := Array_Get(&cache.entries, index)
		return entry.name
	}

	return nil
}

// Load a name from a recording. Name ownership is transferred.
LoadName :: proc(cache: ^NameCache, id: u32, name: cstring, length: i32) {
	HostScope() // unified: the name cache is host-side and append-only (world_arena.odin)
	if name == nil {
		return
	}

	if ([^]u8)(name)[0] == 0 || id == NULL_NAME {
		Free(rawptr(name), int(length + 1))
		return
	}

	m := (^NameMap)(cache.map_)
	if _, found := m[id]; found {
		Free(rawptr(name), int(length + 1))
		return
	}

	index := cache.entries.count
	entry := NameEntry {
		hash   = id,
		name   = name,
		length = length,
	}

	Array_Push(&cache.entries, entry)
	m[id] = index
}
