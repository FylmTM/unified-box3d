// Ported from box3d test/test_macros.h and test/main.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

@(require) import "core:fmt"
import "core:sync"
import "core:testing"

// -define:TRACE_SUBTESTS=true prints each subtest's name before it runs (to find a crash).
TRACE_SUBTESTS :: #config(TRACE_SUBTESTS, false)

// unified: upstream's tests run one after another in one process; Odin's runner is multi-threaded, and Box3D has
// globals (the world registry, the allocator's byte count). Every top-level test holds this lock.
@(private = "file")
box3d_lock: sync.Mutex

@(deferred_none = UNLOCK)
LOCK :: proc() {
	sync.mutex_lock(&box3d_lock)
}

UNLOCK :: proc() {
	sync.mutex_unlock(&box3d_lock)
}
// unified end

// Subtests return true when they pass: `RUN_SUBTEST(t, TestFoo) or_return` inside a subtest,
// `if !RUN_SUBTEST(t, TestFoo) {return}` inside a top-level @(test) proc.
RUN_SUBTEST :: proc(
	t: ^testing.T,
	T: proc(t: ^testing.T) -> bool,
	name := #caller_expression(T),
	loc := #caller_location,
) -> bool {
	when TRACE_SUBTESTS {
		fmt.eprintln("subtest:", name)
	}
	result := T(t)
	if !result {
		testing.expectf(t, false, "  subtest failed: %s", name, loc = loc)
	}
	return result
}

ENSURE :: proc(t: ^testing.T, C: bool, expr := #caller_expression(C), loc := #caller_location) -> bool {
	if C == false {
		testing.expectf(t, false, "condition false: %s", expr, loc = loc)
		return false
	}
	return true
}

ENSURE_SMALL :: proc(t: ^testing.T, C: f32, tol: f32, expr := #caller_expression(C), loc := #caller_location) -> bool {
	if C < -tol || tol < C {
		testing.expectf(t, false, "condition false: abs(%s) < %g (was %g)", expr, tol, C, loc = loc)
		return false
	}
	return true
}

// Upstream prints progress with printf; keep it quiet unless a test fails.
printf :: proc(format: string, args: ..any) {
	_ = format
	_ = args
}
