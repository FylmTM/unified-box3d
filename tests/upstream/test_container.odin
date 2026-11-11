// Ported from box3d test/test_container.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "core:testing"

@(private = "file")
Foo :: struct {
	a: i32,
	b: f32,
}

@(private = "file")
Bar :: struct {
	a: b3.Array(i32),
}

@(private = "file")
TestCreateDestroy :: proc(t: ^testing.T) -> bool {
	a: b3.Array(i32)
	b3.Array_Create(&a)
	b3.Array_Destroy(&a)
	return true
}

@(private = "file")
TestAccess :: proc(t: ^testing.T) -> bool {
	a: b3.Array(i32)
	b3.Array_Create(&a)
	b3.Array_Push(&a, 42)
	element := b3.Array_Get(&a, 0)
	ENSURE(t, element^ == 42) or_return
	b3.Array_Destroy(&a)
	return true
}

@(private = "file")
TestIteration :: proc(t: ^testing.T) -> bool {
	a := b3.Array(i32){}
	b3.Array_Push(&a, 1)
	b3.Array_Push(&a, 2)
	b3.Array_Push(&a, 3)

	sum: i32 = 0
	for i: i32 = 0; i < a.count; i += 1 {
		sum += a.data[i]
	}

	ENSURE(t, sum == 6) or_return
	b3.Array_Destroy(&a)
	return true
}

@(private = "file")
TestArrayOfStruct :: proc(t: ^testing.T) -> bool {
	a: b3.Array(Foo)
	b3.Array_Create(&a)
	b3.Array_Push(&a, Foo{a = 1, b = 5.0})
	b3.Array_Push(&a, Foo{a = 2, b = 6.0})
	b3.Array_Push(&a, Foo{a = 3, b = 7.0})

	sum1: i32 = 0
	sum2: f32 = 0.0
	for i: i32 = 0; i < a.count; i += 1 {
		sum1 += a.data[i].a
		sum2 += a.data[i].b
	}

	ENSURE(t, sum1 == 6) or_return
	ENSURE(t, sum2 == 18.0) or_return

	b3.Array_Destroy(&a)
	return true
}

@(private = "file")
TestStructWithArray :: proc(t: ^testing.T) -> bool {
	a: Bar
	b3.Array_Create(&a.a)
	b3.Array_Push(&a.a, 1)
	b3.Array_Push(&a.a, 2)
	b3.Array_Push(&a.a, 3)

	sum1: i32 = 0
	for i: i32 = 0; i < a.a.count; i += 1 {
		sum1 += a.a.data[i]
	}

	ENSURE(t, sum1 == 6) or_return

	b3.Array_Destroy(&a.a)
	return true
}

@(private = "file")
TestArrayEmplace :: proc(t: ^testing.T) -> bool {
	a := b3.Array(i32){}

	n: i32 = 100
	for i: i32 = 0; i < n; i += 1 {
		j := b3.Array_Emplace(&a)
		j^ = i
	}

	sum: i32 = 0
	for i: i32 = 0; i < a.count; i += 1 {
		sum += a.data[i]
	}

	ENSURE(t, sum == n * (n - 1) / 2) or_return

	b3.Array_Destroy(&a)
	return true
}

@(private = "file")
TestArrayRemove :: proc(t: ^testing.T) -> bool {
	a := b3.Array(i16){}

	n: i32 = 100
	b3.Array_Reserve(&a, n)
	ENSURE(t, a.capacity == n && a.count == 0) or_return

	for i: i32 = 0; i < n; i += 1 {
		b3.Array_Push(&a, i16(i))
	}

	sum: i32 = 0
	for i: i32 = 0; i < n; i += 1 {
		value := b3.Array_Get(&a, 0)
		sum += i32(value^)
		b3.Array_RemoveSwap(&a, 0)
	}

	ENSURE(t, sum == n * (n - 1) / 2) or_return

	b3.Array_Destroy(&a)
	return true
}

@(private = "file")
TestArrayPop :: proc(t: ^testing.T) -> bool {
	a := b3.Array(i32){}

	n: i32 = 10
	b3.Array_Resize(&a, n)
	ENSURE(t, a.capacity == n && a.count == n) or_return

	for i: i32 = 0; i < n; i += 1 {
		a.data[i] = i
	}

	sum: i32 = 0
	for a.count > 0 {
		sum += b3.Array_Pop(&a)
	}

	ENSURE(t, sum == n * (n - 1) / 2) or_return

	b3.Array_Destroy(&a)
	return true
}

@(test)
ContainerTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, TestCreateDestroy) {return}
	if !RUN_SUBTEST(t, TestAccess) {return}
	if !RUN_SUBTEST(t, TestIteration) {return}
	if !RUN_SUBTEST(t, TestArrayOfStruct) {return}
	if !RUN_SUBTEST(t, TestStructWithArray) {return}
	if !RUN_SUBTEST(t, TestArrayEmplace) {return}
	if !RUN_SUBTEST(t, TestArrayRemove) {return}
	if !RUN_SUBTEST(t, TestArrayPop) {return}
}
