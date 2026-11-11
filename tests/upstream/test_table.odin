// Ported from box3d test/test_table.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "core:testing"

@(private = "file")
SET_SPAN :: 317
@(private = "file")
ITEM_COUNT :: ((SET_SPAN * SET_SPAN - SET_SPAN) / 2)

@(private = "file")
TableTestBody :: proc(t: ^testing.T) -> bool {
	power := b3.BoundingPowerOf2(3008)
	ENSURE(t, power == 12) or_return

	nextPowerOf2 := b3.RoundUpPowerOf2(3008)
	ENSURE(t, nextPowerOf2 == (1 << u32(power))) or_return

	N: i32 : SET_SPAN
	itemCount: u32 : ITEM_COUNT
	// unified: static so the 50k flags stay off the test thread's stack
	@(static) removed: [ITEM_COUNT]bool
	removed = {}
	// unified end

	for iter: i32 = 0; iter < 1; iter += 1 {
		set := b3.CreateSet(16)

		// Fill set
		for i: i32 = 0; i < N; i += 1 {
			for j := i + 1; j < N; j += 1 {
				key := b3.ShapePairKey(i, j, 0)
				b3.AddKey(&set, key)
			}
		}

		ENSURE(t, set.count == itemCount) or_return

		// Remove a portion of the set
		k: i32 = 0
		removeCount: u32 = 0
		for i: i32 = 0; i < N; i += 1 {
			for j := i + 1; j < N; j += 1 {
				if j == i + 1 {
					key := b3.ShapePairKey(i, j, 0)
					b3.RemoveKey(&set, key)
					removed[k] = true
					k += 1
					removeCount += 1
				} else {
					removed[k] = false
					k += 1
				}
			}
		}

		ENSURE(t, set.count == (itemCount - removeCount)) or_return

		when b3.B3_DEBUG {
			b3.AtomicStoreInt(&b3.probeCount, 0)
		}

		// Test key search
		// ~5ns per search on an AMD 7950x
		ticks := b3.GetTicks()

		k = 0
		for i: i32 = 0; i < N; i += 1 {
			for j := i + 1; j < N; j += 1 {
				key := b3.ShapePairKey(j, i, 0)
				ENSURE(t, b3.ContainsKey(&set, key) || removed[k]) or_return
				k += 1
			}
		}

		ms := b3.GetMilliseconds(ticks)
		printf(
			"set: count = %d, b3ContainsKey = %.5f ms, ave = %.5f us\n",
			itemCount,
			ms,
			1000.0 * ms / f32(itemCount),
		)

		when b3.B3_DEBUG {
			probeCount := b3.AtomicLoadInt(&b3.probeCount)
			aveProbeCount := f32(probeCount) / f32(itemCount)
			printf("item count = %d, probe count = %d, ave probe count %.2f\n", itemCount, probeCount, aveProbeCount)
		}

		// Remove all keys from set
		for i: i32 = 0; i < N; i += 1 {
			for j := i + 1; j < N; j += 1 {
				key := b3.ShapePairKey(i, j, 0)
				b3.RemoveKey(&set, key)
			}
		}

		ENSURE(t, set.count == 0) or_return

		b3.DestroySet(&set)
	}

	return true
}

// unified: upstream's TableTest is a top-level test with ENSUREs in its body; the body is a subtest here so the
// ENSUREs can use or_return.
@(test)
TableTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, TableTestBody) {return}
}
// unified end
