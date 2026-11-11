// Ported from box3d test/test_math.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "core:math"
import "core:testing"

// 0.0023 degrees
@(private = "file")
ATAN_TOL :: f32(0.00004)

@(private = "file")
MathTestBody :: proc(t: ^testing.T) -> bool {
	for t_: f32 = -10.0; t_ < 10.0; t_ += 0.01 {
		angle := b3.PI * t_
		cs := b3.ComputeCosSin(angle)
		c := math.cos(angle)
		s := math.sin(angle)

		// The cosine and sine approximations are accurate to about 0.1 degrees (0.002 radians)
		// printf( "%g %g\n", r.c - c, r.s - s );
		ENSURE_SMALL(t, cs.cosine - c, 0.002) or_return
		ENSURE_SMALL(t, cs.sine - s, 0.002) or_return

		xn := b3.UnwindAngle(angle)
		a := b3.Atan2(s, c)
		ENSURE(t, b3.IsValidFloat(a)) or_return

		diff := b3.AbsFloat(a - xn)

		// The two results can be off by 360 degrees (-pi and pi)
		if diff > b3.PI {
			diff -= 2.0 * b3.PI
		}

		// The approximate atan2 is quite accurate
		ENSURE_SMALL(t, diff, ATAN_TOL) or_return
	}

	for y: f32 = -1.0; y <= 1.0; y += 0.01 {
		for x: f32 = -1.0; x <= 1.0; x += 0.01 {
			a1 := b3.Atan2(y, x)
			a2 := math.atan2(y, x)
			diff := b3.AbsFloat(a1 - a2)
			ENSURE(t, b3.IsValidFloat(a1)) or_return
			ENSURE_SMALL(t, diff, ATAN_TOL) or_return
		}
	}

	{
		a1 := b3.Atan2(1.0, 0.0)
		a2 := math.atan2(f32(1.0), f32(0.0))
		diff := b3.AbsFloat(a1 - a2)
		ENSURE(t, b3.IsValidFloat(a1)) or_return
		ENSURE_SMALL(t, diff, ATAN_TOL) or_return
	}

	{
		a1 := b3.Atan2(-1.0, 0.0)
		a2 := math.atan2(f32(-1.0), f32(0.0))
		diff := b3.AbsFloat(a1 - a2)
		ENSURE(t, b3.IsValidFloat(a1)) or_return
		ENSURE_SMALL(t, diff, ATAN_TOL) or_return
	}

	{
		a1 := b3.Atan2(0.0, 1.0)
		a2 := math.atan2(f32(0.0), f32(1.0))
		diff := b3.AbsFloat(a1 - a2)
		ENSURE(t, b3.IsValidFloat(a1)) or_return
		ENSURE_SMALL(t, diff, ATAN_TOL) or_return
	}

	{
		a1 := b3.Atan2(0.0, -1.0)
		a2 := math.atan2(f32(0.0), f32(-1.0))
		diff := b3.AbsFloat(a1 - a2)
		ENSURE(t, b3.IsValidFloat(a1)) or_return
		ENSURE_SMALL(t, diff, ATAN_TOL) or_return
	}

	{
		a1 := b3.Atan2(0.0, 0.0)
		a2 := math.atan2(f32(0.0), f32(0.0))
		diff := b3.AbsFloat(a1 - a2)
		ENSURE(t, b3.IsValidFloat(a1)) or_return
		ENSURE_SMALL(t, diff, ATAN_TOL) or_return
	}

	zero := b3.Vec3{}
	one := b3.Vec3{1.0, 1.0, 1.0}
	two := b3.Vec3{2.0, 2.0, 2.0}

	v := b3.Add(one, two)
	ENSURE(t, v.x == 3.0 && v.y == 3.0) or_return

	v = b3.Sub(zero, two)
	ENSURE(t, v.x == -2.0 && v.y == -2.0) or_return

	v = b3.Add(two, two)
	ENSURE(t, v.x != 5.0 && v.y != 5.0) or_return

	axis := b3.Normalize(b3.Vec3{-0.75, 0.5, 1.0})
	transform1 := b3.Transform{{-2.0, 3.0, 0.0}, b3.Quat_identity}
	transform2 := b3.Transform{{1.0, 0.0, 0.0}, b3.MakeQuatFromAxisAngle(axis, b3.PI)}

	transform := b3.MulTransforms(transform2, transform1)

	v = b3.TransformPoint(transform2, b3.TransformPoint(transform1, two))

	u := b3.TransformPoint(transform, two)

	ENSURE_SMALL(t, u.x - v.x, 10.0 * b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, u.y - v.y, 10.0 * b3.FLT_EPSILON) or_return

	v = b3.TransformPoint(transform1, two)
	v = b3.InvTransformPoint(transform1, v)

	ENSURE_SMALL(t, v.x - two.x, 8.0 * b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, v.y - two.y, 8.0 * b3.FLT_EPSILON) or_return

	relTransform := b3.InvMulTransforms(transform1, transform2)
	v = b3.InvTransformPoint(transform1, b3.TransformPoint(transform2, two))
	u = b3.TransformPoint(relTransform, two)
	ENSURE_SMALL(t, u.x - v.x, 10.0 * b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, u.y - v.y, 10.0 * b3.FLT_EPSILON) or_return

	{
		axis = b3.Vec3{0.0, 0.0, 1.0}
		q1 := b3.MakeQuatFromAxisAngle(axis, -0.5 * b3.PI)
		q2 := b3.ComputeQuatBetweenUnitVectors(b3.Vec3{1.0, 0.0, 0.0}, b3.Vec3{0.0, -1.0, 0.0})

		ENSURE_SMALL(t, q1.v.x - q2.v.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, q1.v.y - q2.v.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, q1.v.z - q2.v.z, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, q1.s - q2.s, b3.FLT_EPSILON) or_return

		q3 := b3.NormalizeQuat(b3.Quat{{1.0, -2.0, 3.0}, 4.0})
		q4 := b3.InvMulQuat(q3, q1)
		q5 := b3.MulQuat(q3, q4)
		ENSURE_SMALL(t, q1.v.x - q5.v.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, q1.v.y - q5.v.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, q1.v.z - q5.v.z, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, q1.s - q5.s, b3.FLT_EPSILON) or_return

		q6 := b3.ComputeQuatBetweenUnitVectors(b3.Vec3{0.0, 1.0, 0.0}, b3.Vec3{0.0, -1.0, 0.0})
		ENSURE_SMALL(t, q6.s, b3.FLT_EPSILON) or_return
	}

	v = b3.Normalize(b3.Vec3{0.2, -0.5, 3.0})
	for z: f32 = -1.0; z <= 1.0; z += 0.02 {
		for y: f32 = -1.0; y <= 1.0; y += 0.02 {
			for x: f32 = -1.0; x <= 1.0; x += 0.02 {
				if x == 0.0 && y == 0.0 && z == 0.0 {
					continue
				}

				u = b3.Normalize(b3.Vec3{x, y, z})

				r := b3.ComputeQuatBetweenUnitVectors(v, u)
				ENSURE(t, b3.IsValidQuat(r)) or_return

				w := b3.RotateVector(r, v)

				ENSURE_SMALL(
					t,
					b3.Dot(r.v, b3.Cross(u, w)) - b3.ScalarTripleProduct(r.v, u, w),
					b3.FLT_EPSILON,
				) or_return

				// The quaternion between vectors can have lots of round off error at large angles.
				ENSURE_SMALL(t, w.x - u.x, 0.001) or_return
				ENSURE_SMALL(t, w.y - u.y, 0.001) or_return
				ENSURE_SMALL(t, w.z - u.z, 0.001) or_return

				// Twist angle testing
				twist := r.s < 0.0 ? b3.Atan2(-r.v.z, -r.s) : b3.Atan2(r.v.z, r.s)
				twist *= 2.0
				ENSURE(t, -b3.PI <= twist && twist <= b3.PI) or_return
			}
		}
	}

	{
		// More twist angle testing
		q := b3.Quat {
			v = {-0.0558656752, -0.188799798, 0.00689807534},
			s = -0.980401039,
		}
		twist := q.s < 0.0 ? b3.Atan2(-q.v.z, -q.s) : b3.Atan2(q.v.z, q.s)
		twist *= 2.0
		ENSURE(t, -b3.PI <= twist && twist <= b3.PI) or_return
	}

	{
		m := b3.Matrix3{{3.0, 1.0, -1.0}, {-1.0, 3.0, 1.0}, {1.0, -1.0, 3.0}}
		invM := b3.InvertMatrix(m)
		a := b3.MulMM(m, invM)
		ENSURE_SMALL(t, a.cx.x - 1.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, a.cx.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, a.cx.z, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, a.cy.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, a.cy.y - 1.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, a.cy.z, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, a.cz.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, a.cz.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, a.cz.z - 1.0, b3.FLT_EPSILON) or_return

		v = b3.Vec3{1.0, -2.0, 3.0}
		u = b3.MulMV(invM, b3.MulMV(m, v))
		ENSURE_SMALL(t, v.x - u.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, v.y - u.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, v.z - u.z, b3.FLT_EPSILON) or_return

		w := b3.MulMV(invM, v)
		u = b3.Solve3(m, v)
		ENSURE_SMALL(t, w.x - u.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, w.y - u.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, w.z - u.z, b3.FLT_EPSILON) or_return
	}

	{
		m := b3.Matrix2{{3.0, 1.0}, {-1.0, 3.0}}
		invM := b3.Invert2(m)
		a := b3.MulMM2(m, invM)
		ENSURE_SMALL(t, a.cx.x - 1.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, a.cx.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, a.cy.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, a.cy.y - 1.0, b3.FLT_EPSILON) or_return

		v2 := b3.Vec2{1.0, -2.0}
		u2 := b3.MulMV2(invM, b3.MulMV2(m, v2))
		ENSURE_SMALL(t, v2.x - u2.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, v2.y - u2.y, b3.FLT_EPSILON) or_return

		w := b3.MulMV2(invM, v2)
		u2 = b3.Solve2(m, v2)
		ENSURE_SMALL(t, w.x - u2.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, w.y - u2.y, b3.FLT_EPSILON) or_return

		w = b3.MulMV2(m, u2)
		ENSURE_SMALL(t, w.x - v2.x, 10.0 * b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, w.y - v2.y, 10.0 * b3.FLT_EPSILON) or_return
	}

	for i: i32 = 0; i < 100; i += 1 {
		a := RandomFloat()
		b := f64(a)
		c := f32(b)
		ENSURE(t, c == a) or_return
	}

	q1 := b3.Quat_identity
	q2 := b3.MakeQuatFromAxisAngle(b3.Vec3_axisZ, 0.5 * b3.PI)
	n: i32 = 100
	for i: i32 = 0; i <= n; i += 1 {
		alpha := f32(i) / f32(n)
		q := b3.NLerp(q1, q2, alpha)
		angle := b3.GetTwistAngle(q)
		ENSURE_SMALL(t, alpha * 0.5 * b3.PI - angle, 1.0 * b3.DEG_TO_RAD) or_return
		//printf( "angle = [%g %g %g]\n", alpha, alpha * 0.5f * B3_PI, angle );
	}

	{
		normal := b3.Vec3{0.504055440, 0.621548057, 0.599671543}
		perp := b3.ArbitraryPerp(normal)
		ENSURE_SMALL(t, b3.Dot(normal, perp), 2.0 * b3.FLT_EPSILON) or_return
	}

	{
		// World position boundary helpers. The query agrees with the built type sizes.
		ENSURE(t, b3.IsDoublePrecision() == (size_of(b3.Pos) > size_of(b3.Vec3))) or_return

		// Deltas and offsets round trip exactly for representable inputs in both modes.
		a := b3.Vec3{3.0, -5.0, 2.0}
		b := b3.Vec3{1.0, 4.0, -6.0}
		pa := b3.ToPos(a)
		pb := b3.ToPos(b)

		d := b3.SubPos(pa, pb)
		sub := b3.Sub(a, b)
		ENSURE(t, d.x == sub.x && d.y == sub.y && d.z == sub.z) or_return

		back := b3.SubPos(b3.OffsetPos(pb, sub), pa)
		ENSURE(t, back.x == 0.0 && back.y == 0.0 && back.z == 0.0) or_return

		r := b3.ToVec3(pa)
		ENSURE(t, r.x == a.x && r.y == a.y && r.z == a.z) or_return

		ENSURE(t, b3.IsValidPosition(pa)) or_return

		// World transform relative ops match the pure float transform ops. Float mode is
		// bit identical, double mode keeps the relative result in float.
		axis2 := b3.Normalize(b3.Vec3{0.3, -0.7, 0.5})
		tA := b3.Transform{a, b3.MakeQuatFromAxisAngle(axis2, 0.4)}
		tB := b3.Transform{b, b3.MakeQuatFromAxisAngle(axis2, -1.1)}
		wA := b3.MakeWorldTransform(tA)
		wB := b3.MakeWorldTransform(tB)
		ENSURE(t, b3.IsValidWorldTransform(wA)) or_return

		relRef := b3.InvMulTransforms(tA, tB)
		rel := b3.InvMulWorldTransforms(wA, wB)
		ENSURE_SMALL(t, rel.p.x - relRef.p.x, 1.0e-5) or_return
		ENSURE_SMALL(t, rel.p.y - relRef.p.y, 1.0e-5) or_return
		ENSURE_SMALL(t, rel.p.z - relRef.p.z, 1.0e-5) or_return
		ENSURE_SMALL(t, rel.q.s - relRef.q.s, 1.0e-5) or_return

		// Local point to world and back.
		local := b3.Vec3{0.5, -0.25, 1.5}
		back2 := b3.InvTransformWorldPoint(wA, b3.TransformWorldPoint(wA, local))
		ENSURE_SMALL(t, back2.x - local.x, 1.0e-5) or_return
		ENSURE_SMALL(t, back2.y - local.y, 1.0e-5) or_return
		ENSURE_SMALL(t, back2.z - local.z, 1.0e-5) or_return

		// Compose with a local transform, then strip it back off.
		relAB := b3.InvMulWorldTransforms(wA, b3.MulWorldTransforms(wA, tB))
		ENSURE_SMALL(t, relAB.p.x - tB.p.x, 1.0e-5) or_return
		ENSURE_SMALL(t, relAB.p.y - tB.p.y, 1.0e-5) or_return
		ENSURE_SMALL(t, relAB.p.z - tB.p.z, 1.0e-5) or_return
	}

	// unified: not ported: the BOX3D_DOUBLE_PRECISION block (the port is single precision only).

	return true
}

// unified: upstream's MathTest is a top-level test with ENSUREs in its body; the body is a subtest here so the
// ENSUREs can use or_return. The C loop variable `t` is `t_` (t is the testing.T), the inner `axis` of the world
// transform block is `axis2` (Odin rejects shadowing it in a nested block under -vet-shadowing).
@(test)
MathTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, MathTestBody) {return}
}
// unified end
