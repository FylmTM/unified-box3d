// Ported from box3d src/container.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "core:mem"

// unified: b3Array(T) and its macros are one generic type and procs of the same names; b3DeclareArray and
// b3DeclareArrayNative have no counterpart (the type is generic).

// Define an array.
// It may be zero initialized:
// myArray: Array(i32)
Array :: struct($T: typeid) {
	data:     [^]T,
	count:    i32,
	capacity: i32,
}

// Alternative to zero initialization
Array_Create :: #force_inline proc(a: ^Array($T)) {
	a.data = nil
	a.count = 0
	a.capacity = 0
}

Array_CreateN :: #force_inline proc(a: ^Array($T), n: i32) {
	a.data = n > 0 ? cast([^]T)GrowAlloc(nil, 0, n * size_of(T)) : nil
	a.count = 0
	a.capacity = n
}

Array_Destroy :: #force_inline proc(a: ^Array($T)) {
	Free(a.data, int(a.capacity) * size_of(T))
	a.data = nil
	a.count = 0
	a.capacity = 0
}

Array_Reserve :: #force_inline proc(a: ^Array($T), n: i32) {
	if a.capacity < n {
		oldSize := a.capacity * size_of(T)
		newSize := n * size_of(T)
		a.data = cast([^]T)GrowAlloc(a.data, oldSize, newSize)
		a.capacity = n
	}
}

Array_Resize :: #force_inline proc(a: ^Array($T), n: i32) {
	Array_Reserve(a, n)
	a.count = n
}

// Push a new element by value
Array_Push :: #force_inline proc(a: ^Array($T), value: T) {
	if a.count >= a.capacity {
		oldSize := a.capacity * size_of(T)
		newCapacity := a.capacity == 0 ? 8 : 2 * a.capacity
		newSize := newCapacity * size_of(T)
		a.data = cast([^]T)GrowAlloc(a.data, oldSize, newSize)
		a.capacity = newCapacity
	}
	a.data[a.count] = value
	a.count += 1
}

// Get a pointer to an element
Array_Get :: #force_inline proc(a: ^Array($T), index: i32, loc := #caller_location) -> ^T {
	assert(0 <= index && index < a.count, loc = loc)
	return &a.data[index]
}

// Create a new uninitialized element and return a pointer to it
Array_Emplace :: #force_inline proc(a: ^Array($T)) -> ^T {
	return cast(^T)EmplaceHelper(cast(^rawptr)&a.data, &a.count, &a.capacity, size_of(T))
}

// Remove the last element and return it by value.
Array_Pop :: #force_inline proc(a: ^Array($T)) -> T {
	assert(0 < a.count)
	a.count -= 1
	return a.data[a.count]
}

// Add an uninitialized element and return its index.
Array_AddIndex :: #force_inline proc(a: ^Array($T)) -> i32 {
	EmplaceHelper(cast(^rawptr)&a.data, &a.count, &a.capacity, size_of(T))
	return a.count - 1
}

// Append a contiguous run of values.
Array_Append :: #force_inline proc(a: ^Array($T), src: [^]T, n: i32) {
	if a.count + n > a.capacity {
		req := a.count + n
		newCapacity := req > 2 ? req + (req >> 1) : 8
		oldSize := a.capacity * size_of(T)
		newSize := newCapacity * size_of(T)
		a.data = cast([^]T)GrowAlloc(a.data, oldSize, newSize)
		a.capacity = newCapacity
	}
	if n > 0 {
		mem.copy(&a.data[a.count], src, int(n) * size_of(T))
	}
	a.count += n
}

// Zero the entire allocated buffer (capacity, not just count).
Array_MemZero :: #force_inline proc(a: ^Array($T)) {
	if a.capacity > 0 {
		mem.zero(a.data, int(a.capacity) * size_of(T))
	}
}

// Remove and element by swapping with the last element. If the index is the last element it returns
// NULL_INDEX, otherwise it returns the index of the last element (which is now out of bounds).
Array_RemoveSwap :: #force_inline proc(a: ^Array($T), index: i32) -> i32 {
	return RemoveHelper(a.data, &a.count, index, size_of(T))
}

EmplaceHelper :: proc(data: ^rawptr, count: ^i32, capacity: ^i32, elem_size: i32) -> rawptr {
	if count^ >= capacity^ {
		oldCapacity := capacity^
		oldSize := oldCapacity * elem_size
		newCapacity := oldCapacity == 0 ? 16 : 2 * oldCapacity
		newSize := newCapacity * elem_size
		data^ = GrowAlloc(data^, oldSize, newSize)
		capacity^ = newCapacity
	}
	result := rawptr(uintptr(data^) + uintptr(count^ * elem_size))
	count^ += 1
	return result
}

RemoveHelper :: proc(data: rawptr, count: ^i32, index: i32, elementSize: i32) -> i32 {
	assert(0 <= index && index < count^, "Array index out of bounds")

	count^ -= 1
	if index != count^ {
		mem.copy(
			rawptr(uintptr(data) + uintptr(index * elementSize)),
			rawptr(uintptr(data) + uintptr(count^ * elementSize)),
			int(elementSize),
		)
		return count^
	}

	return NULL_INDEX
}

Array_Clear :: #force_inline proc(a: ^Array($T)) {
	a.count = 0
}

Array_ByteCount :: #force_inline proc(a: ^Array($T)) -> i32 {
	return a.capacity * size_of(T)
}
// unified end
