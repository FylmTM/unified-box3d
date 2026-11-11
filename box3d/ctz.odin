// Ported from box3d src/ctz.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:intrinsics"

// unified: one implementation on Odin's bit intrinsics instead of the MSVC and GCC/Clang branches. A zero input
// gives the bit width (32 or 64), as the MSVC b3CLZ32 path does; upstream never passes zero to the CTZ functions.

// https://en.wikipedia.org/wiki/Find_first_set

CTZ32 :: #force_inline proc "contextless" (block: u32) -> u32 {
	return intrinsics.count_trailing_zeros(block)
}

// This function doesn't need to be fast, so using the Ivy Bridge fallback.
CLZ32 :: #force_inline proc "contextless" (value: u32) -> u32 {
	return intrinsics.count_leading_zeros(value)
}

CTZ64 :: #force_inline proc "contextless" (block: u64) -> u32 {
	return u32(intrinsics.count_trailing_zeros(block))
}

PopCount64 :: #force_inline proc "contextless" (block: u64) -> i32 {
	return i32(intrinsics.count_ones(block))
}
// unified end

IsPowerOf2 :: proc "contextless" (x: i32) -> bool {
	return (x & (x - 1)) == 0
}

BoundingPowerOf2 :: proc "contextless" (x: i32) -> i32 {
	if x <= 1 {
		return 1
	}

	return 32 - i32(CLZ32(u32(x) - 1))
}

RoundUpPowerOf2 :: proc "contextless" (x: i32) -> i32 {
	if x <= 1 {
		return 1
	}

	return 1 << u32(32 - i32(CLZ32(u32(x) - 1)))
}

LowerPowerOf2Exponent :: proc(x: i32) -> i32 {
	assert(x > 0)
	clz := i32(CLZ32(u32(x)))

	// Position of most significant bit = floor(log2(M))
	return 31 - clz
}
