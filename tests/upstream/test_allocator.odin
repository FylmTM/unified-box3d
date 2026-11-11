// Ported from box3d test/test_allocator.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "core:testing"

@(private = "file")
Foo :: struct {
	value1: i32,
	value2: f32,
}

#assert(size_of(Foo) >= size_of(rawptr), "too small")

@(private = "file")
TestBlockAllocate :: proc(t: ^testing.T) -> bool {
	allocator := b3.CreateBlockAllocator(i32(size_of(Foo)), 2)

	item1 := (^Foo)(b3.AllocateElement(&allocator))
	ENSURE(t, item1 != nil) or_return

	item2 := (^Foo)(b3.AllocateElement(&allocator))
	ENSURE(t, item2 != nil) or_return

	b3.DestroyBlockAllocator(&allocator)
	return true
}

@(private = "file")
TestBlockClear :: proc(t: ^testing.T) -> bool {
	allocator := b3.CreateBlockAllocator(i32(size_of(Foo)), 0)

	item1 := (^Foo)(b3.AllocateElement(&allocator))
	item2 := (^Foo)(b3.AllocateElement(&allocator))

	b3.FreeElement(&allocator, item1)
	b3.FreeElement(&allocator, item2)

	b3.DestroyBlockAllocator(&allocator)
	return true
}

// The free list link is a pointer stored in the element, so a stride that is not a multiple
// of the alignment leaves odd elements on a bad boundary. See issue #78.
@(private = "file")
TestBlockAlignment :: proc(t: ^testing.T) -> bool {
	// The manifold sizes are the reported case
	elementSizes := [?]i32{12, i32(size_of(b3.Manifold)), 3 * i32(size_of(b3.Manifold))}

	for i: i32 = 0; i < i32(len(elementSizes)); i += 1 {
		allocator := b3.CreateBlockAllocator(elementSizes[i], 0)

		// Hold the elements so the indices advance and a block boundary is crossed
		elements: [b3.BLOCK_SIZE + 8]rawptr
		count := i32(len(elements))

		for j: i32 = 0; j < count; j += 1 {
			elements[j] = b3.AllocateElement(&allocator)
			ENSURE(t, elements[j] != nil) or_return
			ENSURE(t, (uintptr(elements[j]) & (b3.BLOCK_ALIGNMENT - 1)) == 0) or_return
		}

		for j: i32 = 0; j < count; j += 1 {
			b3.FreeElement(&allocator, elements[j])
		}

		// Now drain the free list
		for j: i32 = 0; j < count; j += 1 {
			elements[j] = b3.AllocateElement(&allocator)
			ENSURE(t, elements[j] != nil) or_return
			ENSURE(t, (uintptr(elements[j]) & (b3.BLOCK_ALIGNMENT - 1)) == 0) or_return
		}

		for j: i32 = 0; j < count; j += 1 {
			b3.FreeElement(&allocator, elements[j])
		}

		b3.DestroyBlockAllocator(&allocator)
	}

	return true
}

@(test)
AllocatorTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, TestBlockAllocate) {return}
	if !RUN_SUBTEST(t, TestBlockClear) {return}
	if !RUN_SUBTEST(t, TestBlockAlignment) {return}
}
