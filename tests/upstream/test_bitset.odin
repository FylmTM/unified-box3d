// Ported from box3d test/test_bitset.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "core:math"
import "core:testing"

@(private = "file")
COUNT :: 169

@(private = "file")
TestBitMath :: proc(t: ^testing.T) -> bool {
	r1 := b3.CLZ32(9)
	ENSURE(t, r1 == 31 - 3) or_return

	for i: i32 = 1; i < 1000; i += 1 {
		e1 := b3.LowerPowerOf2Exponent(i)
		e2 := i32(math.floor(math.log2(f32(i))))
		ENSURE(t, e1 == e2) or_return
	}

	return true
}

@(private = "file")
TestBitSet :: proc(t: ^testing.T) -> bool {
	bitSet := b3.CreateBitSet(COUNT)

	b3.SetBitCountAndClear(&bitSet, COUNT)
	values: [COUNT]bool

	i1, i2: i32 = 0, 1
	b3.SetBit(&bitSet, u32(i1))
	values[i1] = true

	for i2 < COUNT {
		b3.SetBit(&bitSet, u32(i2))
		values[i2] = true
		next := i1 + i2
		i1 = i2
		i2 = next
	}

	for i: i32 = 0; i < COUNT; i += 1 {
		value := b3.GetBit(&bitSet, u32(i))
		ENSURE(t, value == values[i]) or_return
	}

	b3.DestroyBitSet(&bitSet)

	return true
}

@(test)
BitTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, TestBitMath) {return}
	if !RUN_SUBTEST(t, TestBitSet) {return}
}
