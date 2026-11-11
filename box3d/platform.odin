// Ported from box3d src/platform.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "core:sync"

// unified: b3Prefetch is not ported (a hint only); the atomics are core:sync's, sequentially consistent unless
// the C says otherwise.

AtomicStoreInt :: #force_inline proc "contextless" (a: ^AtomicInt, value: i32) {
	sync.atomic_store(&a.value, value)
}

AtomicLoadInt :: #force_inline proc "contextless" (a: ^AtomicInt) -> i32 {
	return sync.atomic_load(&a.value)
}

AtomicFetchAddInt :: #force_inline proc "contextless" (a: ^AtomicInt, increment: i32) -> i32 {
	return sync.atomic_add(&a.value, increment)
}

AtomicCompareExchangeInt :: #force_inline proc "contextless" (a: ^AtomicInt, expected, desired: i32) -> bool {
	_, ok := sync.atomic_compare_exchange_strong(&a.value, expected, desired)
	return ok
}

AtomicStoreU32 :: #force_inline proc "contextless" (a: ^AtomicU32, value: u32) {
	sync.atomic_store(&a.value, value)
}

AtomicLoadU32 :: #force_inline proc "contextless" (a: ^AtomicU32) -> u32 {
	return sync.atomic_load(&a.value)
}

// Relaxed load on a plain word. Lets a racing reader peek before paying for a read modify write.
AtomicLoadU32Raw :: #force_inline proc "contextless" (a: ^u32) -> u32 {
	return sync.atomic_load_explicit(a, .Relaxed)
}

AtomicFetchOrU32 :: #force_inline proc "contextless" (a: ^u32, mask: u32) -> u32 {
	return sync.atomic_or(a, mask)
}

AtomicFetchAddI64 :: #force_inline proc "contextless" (a: ^AtomicI64, increment: i64) -> i64 {
	return sync.atomic_add(&a.value, increment)
}

AtomicLoadI64 :: #force_inline proc "contextless" (a: ^AtomicI64) -> i64 {
	return sync.atomic_load(&a.value)
}
// unified end
