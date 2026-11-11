// Ported from box3d src/bitset.c and src/bitset.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "core:mem"

// Bit set provides fast operations on large arrays of bits.
BitSet :: struct {
	bits:          [^]u64,
	blockCapacity: u32,
	blockCount:    u32,
}

SetBit :: proc(bitSet: ^BitSet, bitIndex: u32) {
	blockIndex := bitIndex / 64
	assert(blockIndex < bitSet.blockCount)
	bitSet.bits[blockIndex] |= (u64(1) << (bitIndex % 64))
}

SetBitGrow :: proc(bitSet: ^BitSet, bitIndex: u32) {
	blockIndex := bitIndex / 64
	if blockIndex >= bitSet.blockCount {
		GrowBitSet(bitSet, blockIndex + 1)
	}
	bitSet.bits[blockIndex] |= (u64(1) << (bitIndex % 64))
}

ClearBit :: proc(bitSet: ^BitSet, bitIndex: u32) {
	blockIndex := bitIndex / 64
	if blockIndex >= bitSet.blockCount {
		return
	}
	bitSet.bits[blockIndex] &= ~(u64(1) << (bitIndex % 64))
}

GetBit :: proc(bitSet: ^BitSet, bitIndex: u32) -> bool {
	blockIndex := bitIndex / 64
	if blockIndex >= bitSet.blockCount {
		return false
	}
	return (bitSet.bits[blockIndex] & (u64(1) << (bitIndex % 64))) != 0
}

GetBitSetBytes :: proc(bitSet: ^BitSet) -> i32 {
	return i32(bitSet.blockCapacity * size_of(u64))
}

CreateBitSet :: proc(bitCapacity: u32) -> BitSet {
	bitSet := BitSet{}
	bitSet.blockCapacity = (bitCapacity + size_of(u64) * 8 - 1) / (size_of(u64) * 8)
	bitSet.blockCount = 0
	bitSet.bits = cast([^]u64)Alloc(int(bitSet.blockCapacity) * size_of(u64))
	mem.zero(bitSet.bits, int(bitSet.blockCapacity) * size_of(u64))
	return bitSet
}

DestroyBitSet :: proc(bitSet: ^BitSet) {
	Free(bitSet.bits, int(bitSet.blockCapacity) * size_of(u64))
	bitSet.blockCapacity = 0
	bitSet.blockCount = 0
	bitSet.bits = nil
}

SetBitCountAndClear :: proc(bitSet: ^BitSet, bitCount: u32) {
	blockCount := (bitCount + size_of(u64) * 8 - 1) / (size_of(u64) * 8)
	if bitSet.blockCapacity < blockCount {
		PushArenaNear(bitSet.bits) // unified: grows where it lives (world_arena.odin)
		DestroyBitSet(bitSet)
		newBitCapacity := bitCount + (bitCount >> 1)
		bitSet^ = CreateBitSet(newBitCapacity)
	}

	bitSet.blockCount = blockCount
	mem.zero(bitSet.bits, int(bitSet.blockCount) * size_of(u64))
}

GrowBitSet :: proc(bitSet: ^BitSet, blockCount: u32) {
	assert(blockCount > bitSet.blockCount)
	if blockCount > bitSet.blockCapacity {
		oldCapacity := bitSet.blockCapacity
		bitSet.blockCapacity = blockCount + blockCount / 2
		newBits := cast([^]u64)AllocNear(bitSet.bits, int(bitSet.blockCapacity) * size_of(u64)) // unified: world_arena.odin
		mem.zero(newBits, int(bitSet.blockCapacity) * size_of(u64))
		assert(bitSet.bits != nil)
		mem.copy_non_overlapping(newBits, bitSet.bits, int(oldCapacity) * size_of(u64))
		Free(bitSet.bits, int(oldCapacity) * size_of(u64))
		bitSet.bits = newBits
	}

	bitSet.blockCount = blockCount
}

InPlaceUnion :: proc(setA: ^BitSet, setB: ^BitSet) {
	assert(setA.blockCount == setB.blockCount)
	blockCount := setA.blockCount
	for i: u32 = 0; i < blockCount; i += 1 {
		setA.bits[i] |= setB.bits[i]
	}
}
