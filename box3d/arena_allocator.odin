// Ported from box3d src/arena_allocator.c and src/arena_allocator.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

MAX_STACK_ENTRIES :: 32

StackEntry :: struct {
	data:       [^]u8,
	name:       cstring,
	size:       i32,
	usedMalloc: bool,
}

// This is a stack-like arena allocator used for fast per step allocations.
// You must nest allocate/free pairs. The code will B3_ASSERT
// if you try to interleave multiple allocate/free pairs.
// This allocator uses the heap if space is insufficient.
// I could remove the need to free entries individually.
Stack :: struct {
	memory:        [^]u8,
	capacity:      i32,
	index:         i32,
	allocation:    i32,
	maxAllocation: i32,
	entries:       [MAX_STACK_ENTRIES]StackEntry,
	entryCount:    i32,
}

// Heap-allocated fallback block tracked when an arena bump overflows.
OverflowBlock :: struct {
	data: [^]u8,
	size: i32,
}

// Shared, heap-allocated state co-owned by every copy of a b3Arena.
// b3Arena is passed by value so its bump pointer auto-restores on
// function return, but overflow tracking and watermarks must persist
// across copies -- hence this pointer-shared block.
ArenaSharedState :: struct {
	overflows:     Array(OverflowBlock),
	maxIndex:      i32, // high water mark of the bump pointer this step
	overflowBytes: i32, // total bytes in overflow blocks this step
	peakDemand:    i32, // all-time peak of (maxIndex + overflowBytes), survives sync
}

Arena :: struct {
	memory:   [^]u8,
	capacity: i32,
	index:    i32,
	shared:   ^ArenaSharedState,
}

// 16-byte alignment for SSE2 + typical struct alignment.
ARENA_ALIGNMENT :: 16

Bump :: #force_inline proc(arena: ^Arena, size: i32) -> rawptr {
	if size == 0 {
		return nil
	}

	aligned := (arena.index + (ARENA_ALIGNMENT - 1)) & ~i32(ARENA_ALIGNMENT - 1)

	if aligned + size > arena.capacity {
		return ArenaOverflowAlloc(arena, size)
	}

	arena.index = aligned + size
	if arena.index > arena.shared.maxIndex {
		arena.shared.maxIndex = arena.index
	}
	return &arena.memory[aligned]
}

CreateStack :: proc(capacity: i32) -> Stack {
	assert(capacity >= 0)
	stack := Stack{}
	stack.capacity = capacity
	stack.memory = cast([^]u8)AllocHost(int(capacity))
	return stack
}

DestroyStack :: proc(stack: ^Stack) {
	Free(stack.memory, int(stack.capacity))
}

StackAlloc :: proc(stack: ^Stack, size: i32, name: cstring) -> rawptr {
	if stack.entryCount == MAX_STACK_ENTRIES {
		assert(false)
		return nil
	}

	alignedSize := ((size - 1) | (ALIGNMENT - 1)) + 1

	entry: StackEntry
	entry.size = alignedSize
	entry.name = name
	if stack.index + alignedSize > stack.capacity {
		// fall back to the heap (undesirable)
		entry.data = cast([^]u8)AllocHost(int(alignedSize))
		entry.usedMalloc = true

		assert((uintptr(entry.data) & (ALIGNMENT - 1)) == 0)
	} else {
		entry.data = &stack.memory[stack.index]
		entry.usedMalloc = false
		stack.index += alignedSize

		assert((uintptr(entry.data) & (ALIGNMENT - 1)) == 0)
	}

	stack.allocation += alignedSize
	if stack.allocation > stack.maxAllocation {
		stack.maxAllocation = stack.allocation
	}

	stack.entries[stack.entryCount] = entry
	stack.entryCount += 1
	return entry.data
}

StackFree :: proc(stack: ^Stack, mem_: rawptr) {
	entryCount := stack.entryCount
	assert(entryCount > 0)
	entry := &stack.entries[entryCount - 1]
	assert(mem_ == entry.data)
	if entry.usedMalloc {
		Free(mem_, int(entry.size))
	} else {
		stack.index -= entry.size
	}
	stack.allocation -= entry.size
	stack.entryCount -= 1
}

// Grow the stack based on usage
GrowStack :: proc(stack: ^Stack) {
	// Stack must not be in use
	assert(stack.allocation == 0)

	if stack.maxAllocation > stack.capacity {
		Free(stack.memory, int(stack.capacity))
		stack.capacity = stack.maxAllocation + stack.maxAllocation / 2
		stack.memory = cast([^]u8)AllocHost(int(stack.capacity))
	}
}

GetStackCapacity :: proc(stack: ^Stack) -> i32 {
	return stack.capacity
}

GetStackAllocation :: proc(stack: ^Stack) -> i32 {
	return stack.allocation
}

GetMaxStackAllocation :: proc(stack: ^Stack) -> i32 {
	return stack.maxAllocation
}

CreateArena :: proc(capacity: i32) -> Arena {
	c := capacity > 8 ? capacity : 8
	shared := cast(^ArenaSharedState)AllocHost(size_of(ArenaSharedState))
	shared^ = ArenaSharedState{}

	return Arena{memory = cast([^]u8)AllocHost(int(c)), capacity = c, index = 0, shared = shared}
}

DestroyArena :: proc(arena: ^Arena) {
	shared := arena.shared
	if shared != nil {
		for i: i32 = 0; i < shared.overflows.count; i += 1 {
			Free(shared.overflows.data[i].data, int(shared.overflows.data[i].size))
		}
		Array_Destroy(&shared.overflows)
		Free(shared, size_of(ArenaSharedState))
	}
	Free(arena.memory, int(arena.capacity))
	arena^ = Arena{}
}

// Heap-allocate an overflow block, register it in the shared state, return it.
ArenaOverflowAlloc :: proc(arena: ^Arena, size: i32) -> rawptr {
	HostScope() // unified: scratch is host memory (world_arena.odin)
	shared := arena.shared
	data := cast([^]u8)AllocHost(int(size))
	block := OverflowBlock{data, size}
	Array_Push(&shared.overflows, block)
	shared.overflowBytes += size
	return data
}

// Call between simulation steps. Frees this step's overflow blocks and grows
// the backing capacity if last step's demand (maxIndex + overflowBytes) exceeded it.
ArenaSync :: proc(arena: ^Arena) {
	shared := arena.shared

	for i: i32 = 0; i < shared.overflows.count; i += 1 {
		Free(shared.overflows.data[i].data, int(shared.overflows.data[i].size))
	}
	Array_Clear(&shared.overflows)

	demand := shared.maxIndex + shared.overflowBytes
	if demand > shared.peakDemand {
		shared.peakDemand = demand
	}
	if demand > arena.capacity {
		Free(arena.memory, int(arena.capacity))
		newCapacity := demand + demand / 2
		arena.memory = cast([^]u8)AllocHost(int(newCapacity))
		arena.capacity = newCapacity
	}

	arena.index = 0
	shared.maxIndex = 0
	shared.overflowBytes = 0
}
