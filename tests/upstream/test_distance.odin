// Ported from box3d test/test_distance.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "core:testing"

@(private = "file")
SegmentDistanceTest :: proc(t: ^testing.T) -> bool {
	p1 := b3.Vec3{-1.0, -1.0, 0.0}
	q1 := b3.Vec3{-1.0, 1.0, 0.0}
	p2 := b3.Vec3{2.0, 0.0, 0.0}
	q2 := b3.Vec3{1.0, 0.0, 0.0}

	result := b3.SegmentDistance(p1, q1, p2, q2)

	ENSURE_SMALL(t, result.fraction1 - 0.5, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, result.fraction2 - 1.0, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, result.point1.x + 1.0, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, result.point1.y, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, result.point1.z, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, result.point2.x - 1.0, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, result.point2.y, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, result.point2.z, b3.FLT_EPSILON) or_return

	return true
}

@(private = "file")
ShapeDistanceTest :: proc(t: ^testing.T) -> bool {
	vas := [?]b3.Vec3{{-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}, {1.0, 1.0, 0.0}, {-1.0, 1.0, 0.0}}

	vbs := [?]b3.Vec3{{2.0, -1.0, 0.0}, {2.0, 1.0, 0.0}}

	input: b3.DistanceInput
	input.proxyA = b3.ShapeProxy{&vas[0], i32(len(vas)), 0.0}
	input.proxyB = b3.ShapeProxy{&vbs[0], i32(len(vbs)), 0.0}
	input.transform = b3.Transform_identity
	input.useRadii = false

	cache := b3.SimplexCache{}
	output := b3.ShapeDistance(&input, &cache, nil, 0)

	ENSURE_SMALL(t, output.distance - 1.0, b3.FLT_EPSILON) or_return

	return true
}

@(private = "file")
ShapeCastTest :: proc(t: ^testing.T) -> bool {
	vas := [?]b3.Vec3{{-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}, {1.0, 1.0, 0.0}, {-1.0, 1.0, 0.0}}

	vbs := [?]b3.Vec3{{2.0, -1.0, 0.0}, {2.0, 1.0, 0.0}}

	input: b3.ShapeCastPairInput
	input.proxyA = b3.ShapeProxy{&vas[0], i32(len(vas)), 0.0}
	input.proxyB = b3.ShapeProxy{&vbs[0], i32(len(vbs)), 0.0}
	input.transform = b3.Transform_identity
	input.translationB = b3.Vec3{-2.0, 0.0, 0.0}
	input.maxFraction = 1.0
	input.canEncroach = false

	output := b3.ShapeCast(&input)

	ENSURE(t, output.hit) or_return
	ENSURE_SMALL(t, output.fraction - 0.5, 0.005) or_return

	return true
}

// unit box with a segment resting 2mm from its +x face: inside the cast
// target distance, not overlapped
@(private = "file")
EncroachCastInput :: proc(translationB: b3.Vec3) -> b3.ShapeCastPairInput {
	@(static) vas := [?]b3.Vec3{{-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}, {1.0, 1.0, 0.0}, {-1.0, 1.0, 0.0}}
	@(static) vbs := [?]b3.Vec3{{1.002, -1.0, 0.0}, {1.002, 1.0, 0.0}}

	input: b3.ShapeCastPairInput
	input.proxyA = b3.ShapeProxy{&vas[0], i32(len(vas)), 0.0}
	input.proxyB = b3.ShapeProxy{&vbs[0], i32(len(vbs)), 0.0}
	input.transform = b3.Transform_identity
	input.translationB = translationB
	input.maxFraction = 1.0
	input.canEncroach = true
	return input
}

@(private = "file")
ShapeCastEncroachRecedeTest :: proc(t: ^testing.T) -> bool {
	input := EncroachCastInput(b3.Vec3{1.0, 0.0, 0.0})
	output := b3.ShapeCast(&input)
	ENSURE(t, output.hit == false) or_return

	return true
}

@(private = "file")
ShapeCastEncroachSlideTest :: proc(t: ^testing.T) -> bool {
	input := EncroachCastInput(b3.Vec3{0.0, 2.0, 0.0})
	output := b3.ShapeCast(&input)
	ENSURE(t, output.hit == false) or_return

	return true
}

@(private = "file")
ShapeCastEncroachClosingTest :: proc(t: ^testing.T) -> bool {
	input := EncroachCastInput(b3.Vec3{-1.0, 0.0, 0.0})
	output := b3.ShapeCast(&input)
	ENSURE(t, output.hit) or_return
	ENSURE_SMALL(t, output.fraction, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, output.normal.x - 1.0, 0.001) or_return

	return true
}

@(private = "file")
ShapeCastEncroachGradedTest :: proc(t: ^testing.T) -> bool {
	// 5mm gap: closing consumes the gap down to the retained rest separation
	@(static) vbs := [?]b3.Vec3{{1.005, -1.0, 0.0}, {1.005, 1.0, 0.0}}

	input := EncroachCastInput(b3.Vec3{-1.0, 0.0, 0.0})
	input.proxyB = b3.ShapeProxy{&vbs[0], i32(len(vbs)), 0.0}
	output := b3.ShapeCast(&input)
	ENSURE(t, output.hit) or_return
	ENSURE_SMALL(t, output.fraction - 0.0025, 0.0005) or_return
	ENSURE_SMALL(t, output.normal.x - 1.0, 0.001) or_return

	return true
}

@(private = "file")
ShapeCastEncroachMaxFractionTest :: proc(t: ^testing.T) -> bool {
	// 5mm gap closes at fraction 0.0025
	@(static) vbs := [?]b3.Vec3{{1.005, -1.0, 0.0}, {1.005, 1.0, 0.0}}

	input := EncroachCastInput(b3.Vec3{-1.0, 0.0, 0.0})
	input.proxyB = b3.ShapeProxy{&vbs[0], i32(len(vbs)), 0.0}
	input.maxFraction = 0.002
	output := b3.ShapeCast(&input)
	ENSURE(t, output.hit == false) or_return

	input.maxFraction = 0.003
	output = b3.ShapeCast(&input)
	ENSURE(t, output.hit) or_return
	ENSURE_SMALL(t, output.fraction - 0.0025, 0.0005) or_return

	return true
}

@(private = "file")
ShapeCastEncroachGrazeTest :: proc(t: ^testing.T) -> bool {
	// mostly tangential slide with a slight closing component: free
	@(static) vbs := [?]b3.Vec3{{1.005, -1.0, 0.0}, {1.005, 1.0, 0.0}}

	input := EncroachCastInput(b3.Vec3{-0.001, 2.0, 0.0})
	input.proxyB = b3.ShapeProxy{&vbs[0], i32(len(vbs)), 0.0}
	output := b3.ShapeCast(&input)
	ENSURE(t, output.hit == false) or_return

	return true
}

@(private = "file")
ShapeCastStartTouchingDefaultTest :: proc(t: ^testing.T) -> bool {
	input := EncroachCastInput(b3.Vec3{1.0, 0.0, 0.0})
	input.canEncroach = false
	output := b3.ShapeCast(&input)
	ENSURE(t, output.hit) or_return

	return true
}

@(private = "file")
ShapeCastEncroachOverlapTest :: proc(t: ^testing.T) -> bool {
	@(static) vcs := [?]b3.Vec3{{0.5, -1.0, 0.0}, {0.5, 1.0, 0.0}}

	input := EncroachCastInput(b3.Vec3{1.0, 0.0, 0.0})
	input.proxyB = b3.ShapeProxy{&vcs[0], i32(len(vcs)), 0.0}
	output := b3.ShapeCast(&input)
	ENSURE(t, output.hit) or_return

	return true
}

@(private = "file")
TimeOfImpactTest :: proc(t: ^testing.T) -> bool {
	vas := [?]b3.Vec3{{-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}, {1.0, 1.0, 0.0}, {-1.0, 1.0, 0.0}}

	vbs := [?]b3.Vec3{{2.0, -1.0, 0.0}, {2.0, 1.0, 0.0}}

	input: b3.TOIInput
	input.proxyA = b3.ShapeProxy{&vas[0], i32(len(vas)), 0.0}
	input.proxyB = b3.ShapeProxy{&vbs[0], i32(len(vbs)), 0.0}
	input.sweepA = b3.Sweep{b3.Vec3_zero, b3.Vec3_zero, b3.Vec3_zero, b3.Quat_identity, b3.Quat_identity}
	input.sweepB = b3.Sweep{b3.Vec3_zero, b3.Vec3_zero, b3.Vec3{-2.0, 0.0, 0.0}, b3.Quat_identity, b3.Quat_identity}
	input.maxFraction = 1.0

	output := b3.TimeOfImpact(&input)

	ENSURE(t, output.state == .toiStateHit) or_return
	ENSURE_SMALL(t, output.fraction - 0.5, 0.005) or_return

	return true
}

@(test)
DistanceTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, SegmentDistanceTest) {return}
	if !RUN_SUBTEST(t, ShapeDistanceTest) {return}
	if !RUN_SUBTEST(t, ShapeCastTest) {return}
	if !RUN_SUBTEST(t, ShapeCastEncroachRecedeTest) {return}
	if !RUN_SUBTEST(t, ShapeCastEncroachSlideTest) {return}
	if !RUN_SUBTEST(t, ShapeCastEncroachClosingTest) {return}
	if !RUN_SUBTEST(t, ShapeCastEncroachGradedTest) {return}
	if !RUN_SUBTEST(t, ShapeCastEncroachMaxFractionTest) {return}
	if !RUN_SUBTEST(t, ShapeCastEncroachGrazeTest) {return}
	if !RUN_SUBTEST(t, ShapeCastStartTouchingDefaultTest) {return}
	if !RUN_SUBTEST(t, ShapeCastEncroachOverlapTest) {return}
	if !RUN_SUBTEST(t, TimeOfImpactTest) {return}
}
