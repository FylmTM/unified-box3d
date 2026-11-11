// Ported from box3d src/table.c and src/table.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "core:mem"

// I need a good hash because the keys are built from pairs of increasing integers.
// A simple hash like hash = (integer1 XOR integer2) has many collisions.
// https://lemire.me/blog/2018/08/15/fast-strongly-universal-64-bit-hashing-everywhere/
// https://preshing.com/20130107/this-hash-set-is-faster-than-a-judy-array/
// todo try: https://www.jandrewrogers.com/2019/02/12/fast-perfect-hashing/
// todo try:
// https://probablydance.com/2018/06/16/fibonacci-hashing-the-optimization-that-the-world-forgot-or-a-better-alternative-to-integer-modulo/
KeyHash :: #force_inline proc "contextless" (key: u64) -> u32 {
	h := key
	h ~= h >> 33
	h *= 0xff51afd7ed558ccd
	h ~= h >> 33
	h *= 0xc4ceb9fe1a85ec53
	h ~= h >> 33

	return u32(h)
}

SetItem :: struct {
	key:  u64,
	hash: u32,
}

HashSet :: struct {
	items:    [^]SetItem,
	capacity: u32,
	count:    u32,
}

SHAPE_MASK :: (MAX_SHAPES - 1)
CHILD_MASK :: (MAX_CHILD_SHAPES - 1)

ShapePairKey :: proc "contextless" (s1, s2, c: i32) -> u64 {
	if s1 < s2 {
		return(
			(u64(SHAPE_MASK & s1) << (64 - SHAPE_POWER)) |
			(u64(SHAPE_MASK & s2) << (64 - 2 * SHAPE_POWER)) |
			(u64(CHILD_MASK & c)) \
		)
	}

	return(
		(u64(SHAPE_MASK & s2) << (64 - SHAPE_POWER)) |
		(u64(SHAPE_MASK & s1) << (64 - 2 * SHAPE_POWER)) |
		(u64(CHILD_MASK & c)) \
	)
}

when B3_DEBUG {
	probeCount: AtomicInt
}

#assert(2 * SHAPE_POWER + CHILD_POWER == 64, "compound power")
#assert(CHILD_POWER > 8, "compound child power")

CreateSet :: proc(capacity: i32) -> HashSet {
	set := HashSet{}

	// Capacity must be a power of 2
	if capacity > 16 {
		set.capacity = u32(RoundUpPowerOf2(capacity))
	} else {
		set.capacity = 16
	}

	set.count = 0
	set.items = cast([^]SetItem)Alloc(int(set.capacity) * size_of(SetItem))
	mem.zero(set.items, int(set.capacity) * size_of(SetItem))

	return set
}

DestroySet :: proc(set: ^HashSet) {
	Free(set.items, int(set.capacity) * size_of(SetItem))
	set.items = nil
	set.count = 0
	set.capacity = 0
}

ClearSet :: proc(set: ^HashSet) {
	set.count = 0
	mem.zero(set.items, int(set.capacity) * size_of(SetItem))
}

@(private = "file")
FindSlot :: proc(set: ^HashSet, key: u64, hash: u32) -> i32 {
	capacity := set.capacity
	index := i32(hash & (capacity - 1))
	items := set.items
	for items[index].hash != 0 && items[index].key != key {
		when B3_DEBUG {
			AtomicFetchAddInt(&probeCount, 1)
		}
		index = i32((u32(index) + 1) & (capacity - 1))
	}

	return index
}

@(private = "file")
AddKeyHaveCapacity :: proc(set: ^HashSet, key: u64, hash: u32) {
	index := FindSlot(set, key, hash)
	items := set.items
	assert(items[index].hash == 0)

	items[index].key = key
	items[index].hash = hash
	set.count += 1
}

@(private = "file")
GrowTable :: proc(set: ^HashSet) {
	oldCount := set.count
	_ = oldCount

	oldCapacity := set.capacity
	oldItems := set.items

	set.count = 0
	// Capacity must be a power of 2
	set.capacity = 2 * oldCapacity
	set.items = cast([^]SetItem)AllocNear(oldItems, int(set.capacity) * size_of(SetItem)) // unified: world_arena.odin
	mem.zero(set.items, int(set.capacity) * size_of(SetItem))

	// Transfer items into new array
	for i: u32 = 0; i < oldCapacity; i += 1 {
		item := &oldItems[i]
		if item.hash == 0 {
			// this item was empty
			continue
		}

		AddKeyHaveCapacity(set, item.key, item.hash)
	}

	assert(set.count == oldCount)

	Free(oldItems, int(oldCapacity) * size_of(SetItem))
}

ContainsKey :: proc(set: ^HashSet, key: u64) -> bool {
	// key of zero is a sentinel
	assert(key != 0)
	hash := KeyHash(key)
	index := FindSlot(set, key, hash)
	return set.items[index].key == key
}

// The caller already has the hash, which lets a batch prefetch the slots before probing them.
ContainsHashedKey :: proc(set: ^HashSet, key: u64, hash: u32) -> bool {
	assert(key != 0)
	assert(hash == KeyHash(key))
	index := FindSlot(set, key, hash)
	return set.items[index].key == key
}

GetHashSetBytes :: proc(set: ^HashSet) -> i32 {
	return i32(set.capacity) * i32(size_of(SetItem))
}

AddKey :: proc(set: ^HashSet, key: u64) -> bool {
	// key of zero is a sentinel
	assert(key != 0)

	hash := KeyHash(key)
	assert(hash != 0)

	index := FindSlot(set, key, hash)
	if set.items[index].hash != 0 {
		// Already in set
		assert(set.items[index].hash == hash && set.items[index].key == key)
		return true
	}

	if 2 * set.count >= set.capacity {
		GrowTable(set)
	}

	AddKeyHaveCapacity(set, key, hash)
	return false
}

// See https://en.wikipedia.org/wiki/Open_addressing
RemoveKey :: proc(set: ^HashSet, key: u64) -> bool {
	hash := KeyHash(key)
	i := FindSlot(set, key, hash)
	items := set.items
	if items[i].hash == 0 {
		// Not in set
		return false
	}

	// Mark item i as unoccupied
	items[i].key = 0
	items[i].hash = 0

	assert(set.count > 0)
	set.count -= 1

	// Attempt to fill item i
	j := i
	capacity := set.capacity
	for {
		j = i32((u32(j) + 1) & (capacity - 1))
		if items[j].hash == 0 {
			break
		}

		// k is the first item for the hash of j
		k := i32(items[j].hash & (capacity - 1))

		// determine if k lies cyclically in (i,j]
		// i <= j: | i..k..j |
		// i > j: |.k..j  i....| or |....j     i..k.|
		if i <= j {
			if i < k && k <= j {
				continue
			}
		} else {
			if i < k || k <= j {
				continue
			}
		}

		// Move j into i
		items[i] = items[j]

		// Mark item j as unoccupied
		items[j].key = 0
		items[j].hash = 0

		i = j
	}

	return true
}

// This function is here because ctz.h is included by
// this file but not in bitset.c
CountSetBits :: proc(bitSet: ^BitSet) -> i32 {
	popCount: i32 = 0
	blockCount := bitSet.blockCount
	for i: u32 = 0; i < blockCount; i += 1 {
		popCount += PopCount64(bitSet.bits[i])
	}

	return popCount
}
