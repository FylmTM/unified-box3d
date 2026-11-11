// Ported from box3d src/block_allocator.c and src/block_allocator.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

// unified: blocks of 32 elements (upstream: 256), so a small world's snapshot doesn't carry empty manifold blocks
// (world_arena.odin: snapshots copy every live block)
BLOCK_EXPONENT :: 5
// unified end
BLOCK_SIZE :: (1 << BLOCK_EXPONENT)
BLOCK_ALIGNMENT :: 16

Block :: struct {
	memory: [^]u8,
}

BlockAllocator :: struct {
	blocks:          Array(Block),
	freeList:        rawptr,
	elementSize:     i32,
	nextIndex:       i32,
	allocationCount: i32,
}

// Element must be large enough to hold a pointer. The element size is rounded up to B3_BLOCK_ALIGNMENT.
CreateBlockAllocator :: proc(elementSize: i32, initialCount: i32) -> BlockAllocator {
	elementSize := elementSize
	assert(elementSize >= i32(size_of(rawptr)))

	// The free list stores a pointer inside each element, so the stride must keep every element aligned.
	// Blocks come from b3Alloc, so a stride that is a multiple of B3_BLOCK_ALIGNMENT aligns all elements.
	elementSize = ((elementSize - 1) | (BLOCK_ALIGNMENT - 1)) + 1

	allocator := BlockAllocator{}

	Array_Create(&allocator.blocks)
	allocator.elementSize = elementSize
	allocator.freeList = nil
	allocator.nextIndex = 0
	allocator.allocationCount = 0

	if initialCount > 0 {
		count := (initialCount + BLOCK_SIZE - 1) >> BLOCK_EXPONENT
		Array_Resize(&allocator.blocks, count)

		for i: i32 = 0; i < allocator.blocks.count; i += 1 {
			allocator.blocks.data[i].memory = cast([^]u8)Alloc(int(BLOCK_SIZE * elementSize))
		}
	}

	return allocator
}

DestroyBlockAllocator :: proc(allocator: ^BlockAllocator) {
	for i: i32 = 0; i < allocator.blocks.count; i += 1 {
		Free(allocator.blocks.data[i].memory, int(BLOCK_SIZE * allocator.elementSize))
	}

	Array_Destroy(&allocator.blocks)
}

// Returns one element of elementSize contiguous bytes. Address is stable until freed.
AllocateElement :: proc(allocator: ^BlockAllocator) -> rawptr {
	assert(allocator != nil)

	allocator.allocationCount += 1

	// Pop from free list first
	if allocator.freeList != nil {
		element := allocator.freeList
		allocator.freeList = (^rawptr)(element)^
		return element
	}

	index := allocator.nextIndex
	allocator.nextIndex += 1

	requiredBlockCount := (index >> BLOCK_EXPONENT) + 1

	if requiredBlockCount > allocator.blocks.count {
		oldCount := allocator.blocks.count
		Array_Resize(&allocator.blocks, requiredBlockCount)

		for i := oldCount; i < requiredBlockCount; i += 1 {
			allocator.blocks.data[i].memory = cast([^]u8)Alloc(int(BLOCK_SIZE * allocator.elementSize))
		}
	}

	blockIndex := index >> BLOCK_EXPONENT
	blockOffset := index & (BLOCK_SIZE - 1)

	return &allocator.blocks.data[blockIndex].memory[blockOffset * allocator.elementSize]
}

FreeElement :: proc(allocator: ^BlockAllocator, element: rawptr) {
	assert(allocator != nil)
	assert(element != nil)
	assert(allocator.allocationCount > 0)
	assert((uintptr(element) & (BLOCK_ALIGNMENT - 1)) == 0)

	allocator.allocationCount -= 1

	// Push onto free list
	(^rawptr)(element)^ = allocator.freeList
	allocator.freeList = element
}
