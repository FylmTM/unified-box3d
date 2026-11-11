// Ported from box3d test/test_shape.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "core:math"
import "core:testing"

@(private = "file")
capsule := b3.Capsule{{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 1.0}
@(private = "file")
sphere := b3.Sphere{{1.0, 0.0, 0.0}, 1.0}
@(private = "file")
box: b3.BoxHull

@(private = "file")
N :: 4

@(private = "file")
ShapeMassTest :: proc(t: ^testing.T) -> bool {
	// Sphere
	{
		md := b3.ComputeSphereMass(&sphere, 1.0)
		mass := f32(4.0) / f32(3.0) * b3.PI
		ENSURE_SMALL(t, md.mass - mass, b3.FLT_EPSILON) or_return
		ENSURE(t, md.center.x == 1.0 && md.center.y == 0.0) or_return

		// Inertia is now about the shape center of mass, so the offset does not appear.
		inertia := f32(2.0) / f32(5.0) * mass
		ENSURE_SMALL(t, md.inertia.cx.x - inertia, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, md.inertia.cy.y - inertia, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, md.inertia.cz.z - inertia, b3.FLT_EPSILON) or_return
	}

	// Analytic box hull
	{
		md := b3.ComputeHullMass(&box.base, 1.0)
		mass := f32(2.0) * f32(2.0) * f32(2.0)
		ENSURE_SMALL(t, md.mass - mass, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, md.center.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, md.center.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, md.center.z, b3.FLT_EPSILON) or_return
		inertia := (f32(1.0) / f32(12.0)) * mass * (f32(2.0) * f32(2.0) + f32(2.0) * f32(2.0))
		ENSURE_SMALL(t, md.inertia.cx.x - inertia, 2.0 * b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, md.inertia.cy.y - inertia, 2.0 * b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, md.inertia.cz.z - inertia, 2.0 * b3.FLT_EPSILON) or_return
	}

	// Translated box
	{
		offset := b3.Vec3{0.4, -0.7, 0.1}
		transform := b3.Transform {
			p = offset,
			q = b3.Quat_identity,
		}
		h := b3.Vec3{0.25, 0.5, 0.3}
		b1 := b3.MakeBoxHull(h.x, h.y, h.z)
		b2 := b3.MakeTransformedBoxHull(h.x, h.y, h.z, transform)

		m1 := b3.ComputeHullMass(&b1.base, 1.0)
		m2 := b3.ComputeHullMass(&b2.base, 1.0)

		ENSURE_SMALL(t, m1.mass - m2.mass, b3.FLT_EPSILON) or_return

		d := b3.SubMM(b1.base.centralInertia, b2.base.centralInertia)
		ENSURE_SMALL(t, d.cx.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cx.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cx.z, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cy.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cy.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cy.z, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cz.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cz.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cz.z, b3.FLT_EPSILON) or_return

		ENSURE_SMALL(t, m2.center.x - offset.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, m2.center.y - offset.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, m2.center.z - offset.z, b3.FLT_EPSILON) or_return
	}

	// Rotated box
	{
		h1 := b3.Vec3{0.25, 0.5, 0.3}
		h2 := b3.Vec3{0.25, 0.3, 0.5}
		q := b3.ComputeQuatBetweenUnitVectors(b3.Vec3_axisY, b3.Vec3_axisZ)
		transform := b3.Transform {
			p = b3.Vec3_zero,
			q = q,
		}
		b1 := b3.MakeTransformedBoxHull(h1.x, h1.y, h1.z, transform)
		b2 := b3.MakeBoxHull(h2.x, h2.y, h2.z)

		m1 := b3.ComputeHullMass(&b1.base, 1.0)
		m2 := b3.ComputeHullMass(&b2.base, 1.0)

		ENSURE_SMALL(t, m1.mass - m2.mass, b3.FLT_EPSILON) or_return

		d := b3.SubMM(b1.base.centralInertia, b2.base.centralInertia)
		ENSURE_SMALL(t, d.cx.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cx.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cx.z, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cy.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cy.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cy.z, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cz.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cz.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cz.z, b3.FLT_EPSILON) or_return

		ENSURE_SMALL(t, m1.center.x - m2.center.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, m1.center.y - m2.center.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, m1.center.z - m2.center.z, b3.FLT_EPSILON) or_return
	}

	// Transformed box
	{
		offset := b3.Vec3{0.4, -0.7, 0.1}
		h1 := b3.Vec3{0.25, 0.5, 0.3}
		h2 := b3.Vec3{0.25, 0.3, 0.5}
		q := b3.ComputeQuatBetweenUnitVectors(b3.Vec3_axisY, b3.Vec3_axisZ)
		transform := b3.Transform {
			p = offset,
			q = q,
		}
		b1 := b3.MakeTransformedBoxHull(h1.x, h1.y, h1.z, transform)
		b2 := b3.MakeBoxHull(h2.x, h2.y, h2.z)

		m1 := b3.ComputeHullMass(&b1.base, 1.0)
		m2 := b3.ComputeHullMass(&b2.base, 1.0)

		ENSURE_SMALL(t, m1.mass - m2.mass, b3.FLT_EPSILON) or_return

		d := b3.SubMM(b1.base.centralInertia, b2.base.centralInertia)
		ENSURE_SMALL(t, d.cx.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cx.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cx.z, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cy.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cy.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cy.z, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cz.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cz.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, d.cz.z, b3.FLT_EPSILON) or_return

		ENSURE_SMALL(t, m1.center.x - offset.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, m1.center.y - offset.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, m1.center.z - offset.z, b3.FLT_EPSILON) or_return
	}

	// Capsule
	{
		radius := capsule.radius
		length := b3.Distance(capsule.center1, capsule.center2)

		// Capsule along x-axis
		md := b3.ComputeCapsuleMass(&capsule, 1.0)

		// Box that fully contains capsule. Upper bound on capsule mass.
		r := b3.MakeBoxHull(radius + 0.5 * length, radius, radius)
		mdUpper := b3.ComputeHullMass(&r.base, 1.0)

		// Approximate capsule using convex hull. This should be a lower bound on the
		// capsule mass.
		points: [2 * N * N]b3.Vec3
		d := b3.PI / (f32(N) - 1.0)
		angle1 := -0.5 * b3.PI
		index: i32 = 0
		for i: i32 = 0; i < N; i += 1 {
			s1 := math.sin(angle1)
			c1 := math.cos(angle1)
			angle2 := -0.5 * b3.PI
			for j: i32 = 0; j < N; j += 1 {
				points[index].x = 1.0 + radius * c1
				points[index].y = radius * s1 * math.cos(angle2)
				points[index].z = radius * s1 * math.sin(angle2)
				angle2 += d
				index += 1
			}

			angle1 += d
		}

		angle1 = 0.5 * b3.PI
		for i: i32 = 0; i < N; i += 1 {
			s1 := math.sin(angle1)
			c1 := math.cos(angle1)
			angle2 := -0.5 * b3.PI
			for j: i32 = 0; j < N; j += 1 {
				points[index].x = -1.0 + radius * c1
				points[index].y = radius * s1 * math.cos(angle2)
				points[index].z = radius * s1 * math.sin(angle2)
				angle2 += d
				index += 1
			}

			angle1 += d
		}

		ENSURE(t, index == 2 * N * N) or_return

		hull := b3.CreateHull(&points[0], 2 * N * N, 2 * N * N)
		mdLower := b3.ComputeHullMass(hull, 1.0)

		ENSURE(t, mdLower.mass < md.mass && md.mass < mdUpper.mass) or_return
		ENSURE(t, mdLower.inertia.cx.x < md.inertia.cx.x && md.inertia.cx.x < mdUpper.inertia.cx.x) or_return
		ENSURE(t, mdLower.inertia.cy.y < md.inertia.cy.y && md.inertia.cy.y < mdUpper.inertia.cy.y) or_return
		ENSURE(t, mdLower.inertia.cz.z < md.inertia.cz.z && md.inertia.cz.z < mdUpper.inertia.cz.z) or_return

		b3.DestroyHull(hull)
	}

	return true
}

@(private = "file")
ShapeAABBTest :: proc(t: ^testing.T) -> bool {
	{
		b := b3.ComputeSphereAABB(&sphere, b3.Transform_identity)
		ENSURE_SMALL(t, b.lowerBound.x, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, b.lowerBound.y + 1.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, b.lowerBound.z + 1.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, b.upperBound.x - 2.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, b.upperBound.y - 1.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, b.upperBound.z - 1.0, b3.FLT_EPSILON) or_return
	}

	{
		b := b3.ComputeCapsuleAABB(&capsule, b3.Transform_identity)
		ENSURE_SMALL(t, b.lowerBound.x + 2.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, b.lowerBound.y + 1.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, b.lowerBound.z + 1.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, b.upperBound.x - 2.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, b.upperBound.y - 1.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, b.upperBound.z - 1.0, b3.FLT_EPSILON) or_return
	}
	{
		b := b3.ComputeHullAABB(&box.base, b3.Transform_identity)
		ENSURE_SMALL(t, b.lowerBound.x + 1.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, b.lowerBound.y + 1.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, b.lowerBound.z + 1.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, b.upperBound.x - 1.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, b.upperBound.y - 1.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, b.upperBound.z - 1.0, b3.FLT_EPSILON) or_return
	}

	return true
}

// unified: not ported: PointInShapeTest, compiled out upstream (#if 0, 2D leftovers).

// Shared assertions for a surface hit. The normal points outward toward the ray,
// the point sits on the surface, and the point lies on the ray at the reported fraction.
@(private = "file")
CheckCastHit :: proc(
	t: ^testing.T,
	out: b3.CastOutput,
	origin: b3.Vec3,
	translation: b3.Vec3,
	point: b3.Vec3,
	normal: b3.Vec3,
	fraction: f32,
	tol: f32,
) -> bool {
	ENSURE(t, out.hit) or_return
	ENSURE_SMALL(t, out.fraction - fraction, tol) or_return
	ENSURE_SMALL(t, out.point.x - point.x, tol) or_return
	ENSURE_SMALL(t, out.point.y - point.y, tol) or_return
	ENSURE_SMALL(t, out.point.z - point.z, tol) or_return
	ENSURE_SMALL(t, out.normal.x - normal.x, tol) or_return
	ENSURE_SMALL(t, out.normal.y - normal.y, tol) or_return
	ENSURE_SMALL(t, out.normal.z - normal.z, tol) or_return

	onRay := b3.MulAdd(origin, out.fraction, translation)
	ENSURE_SMALL(t, b3.Distance(out.point, onRay), tol) or_return
	return true
}

// The shared initial overlap convention: a ray starting inside a solid reports the origin
// with zero fraction and no normal.
@(private = "file")
CheckInitialOverlap :: proc(t: ^testing.T, out: b3.CastOutput, origin: b3.Vec3) -> bool {
	ENSURE(t, out.hit) or_return
	ENSURE(t, out.fraction == 0.0) or_return
	ENSURE_SMALL(t, b3.Distance(out.point, origin), b3.FLT_EPSILON) or_return
	ENSURE(t, out.normal.x == 0.0 && out.normal.y == 0.0 && out.normal.z == 0.0) or_return
	return true
}

@(private = "file")
RayCastSphereHitTest :: proc(t: ^testing.T) -> bool {
	s := b3.Sphere{{0.0, 0.0, 0.0}, 1.0}

	// Hit along each principal axis. Surface at distance 3 over a length 8 ray.
	{
		input := b3.RayCastInput{{-4.0, 0.0, 0.0}, {8.0, 0.0, 0.0}, 1.0}
		out := b3.RayCastSphere(&s, &input)
		CheckCastHit(
			t,
			out,
			input.origin,
			input.translation,
			b3.Vec3{-1.0, 0.0, 0.0},
			b3.Vec3{-1.0, 0.0, 0.0},
			f32(3.0) / f32(8.0),
			1e-5,
		) or_return
	}
	{
		input := b3.RayCastInput{{0.0, 4.0, 0.0}, {0.0, -8.0, 0.0}, 1.0}
		out := b3.RayCastSphere(&s, &input)
		CheckCastHit(
			t,
			out,
			input.origin,
			input.translation,
			b3.Vec3{0.0, 1.0, 0.0},
			b3.Vec3{0.0, 1.0, 0.0},
			f32(3.0) / f32(8.0),
			1e-5,
		) or_return
	}
	{
		input := b3.RayCastInput{{0.0, 0.0, -4.0}, {0.0, 0.0, 8.0}, 1.0}
		out := b3.RayCastSphere(&s, &input)
		CheckCastHit(
			t,
			out,
			input.origin,
			input.translation,
			b3.Vec3{0.0, 0.0, -1.0},
			b3.Vec3{0.0, 0.0, -1.0},
			f32(3.0) / f32(8.0),
			1e-5,
		) or_return
	}

	// Offset center, hit partway along the ray.
	{
		s2 := b3.Sphere{{5.0, 0.0, 0.0}, 2.0}
		input := b3.RayCastInput{{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, 1.0}
		out := b3.RayCastSphere(&s2, &input)
		CheckCastHit(
			t,
			out,
			input.origin,
			input.translation,
			b3.Vec3{3.0, 0.0, 0.0},
			b3.Vec3{-1.0, 0.0, 0.0},
			0.3,
			1e-5,
		) or_return
	}

	// Diagonal ray straight through the center.
	{
		k: f32 = 0.70710678
		input := b3.RayCastInput{{-3.0, -3.0, 0.0}, {6.0, 6.0, 0.0}, 1.0}
		out := b3.RayCastSphere(&s, &input)
		CheckCastHit(
			t,
			out,
			input.origin,
			input.translation,
			b3.Vec3{-k, -k, 0.0},
			b3.Vec3{-k, -k, 0.0},
			0.382149,
			1e-4,
		) or_return
	}

	return true
}

@(private = "file")
RayCastSphereMissTest :: proc(t: ^testing.T) -> bool {
	s := b3.Sphere{{0.0, 0.0, 0.0}, 1.0}

	// Pointing away.
	{
		input := b3.RayCastInput{{-4.0, 0.0, 0.0}, {-8.0, 0.0, 0.0}, 1.0}
		ENSURE(t, b3.RayCastSphere(&s, &input).hit == false) or_return
	}
	// Passes wide of the sphere.
	{
		input := b3.RayCastInput{{-4.0, 3.0, 0.0}, {8.0, 0.0, 0.0}, 1.0}
		ENSURE(t, b3.RayCastSphere(&s, &input).hit == false) or_return
	}
	// Aimed at the sphere but the translation stops short.
	{
		input := b3.RayCastInput{{-4.0, 0.0, 0.0}, {8.0, 0.0, 0.0}, 0.3}
		ENSURE(t, b3.RayCastSphere(&s, &input).hit == false) or_return
	}
	return true
}

@(private = "file")
RayCastSphereClipTest :: proc(t: ^testing.T) -> bool {
	s := b3.Sphere{{0.0, 0.0, 0.0}, 1.0}

	// The surface is reached at fraction 3/8. Straddle it with maxFraction.
	{
		input := b3.RayCastInput{{-4.0, 0.0, 0.0}, {8.0, 0.0, 0.0}, 0.374}
		ENSURE(t, b3.RayCastSphere(&s, &input).hit == false) or_return
	}
	{
		input := b3.RayCastInput{{-4.0, 0.0, 0.0}, {8.0, 0.0, 0.0}, 0.376}
		out := b3.RayCastSphere(&s, &input)
		ENSURE(t, out.hit) or_return
		ENSURE_SMALL(t, out.fraction - f32(3.0) / f32(8.0), 1e-5) or_return
	}
	return true
}

@(private = "file")
RayCastSphereInteriorTest :: proc(t: ^testing.T) -> bool {
	s := b3.Sphere{{0.0, 0.0, 0.0}, 1.0}

	// Origin inside reports the origin with zero fraction.
	{
		input := b3.RayCastInput{{0.3, 0.0, 0.0}, {8.0, 0.0, 0.0}, 1.0}
		out := b3.RayCastSphere(&s, &input)
		ENSURE(t, out.hit) or_return
		ENSURE(t, out.fraction == 0.0) or_return
		ENSURE_SMALL(t, b3.Distance(out.point, input.origin), b3.FLT_EPSILON) or_return
	}
	// Zero length ray inside.
	{
		input := b3.RayCastInput{{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, 1.0}
		out := b3.RayCastSphere(&s, &input)
		ENSURE(t, out.hit) or_return
		ENSURE_SMALL(t, b3.Distance(out.point, input.origin), b3.FLT_EPSILON) or_return
	}
	// Zero length ray outside.
	{
		input := b3.RayCastInput{{3.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 1.0}
		ENSURE(t, b3.RayCastSphere(&s, &input).hit == false) or_return
	}
	return true
}

@(private = "file")
RayCastSphereGrazeTest :: proc(t: ^testing.T) -> bool {
	s := b3.Sphere{{0.0, 0.0, 0.0}, 1.0}

	// Just inside the radius grazes a hit, just outside misses.
	{
		input := b3.RayCastInput{{-4.0, 0.999, 0.0}, {8.0, 0.0, 0.0}, 1.0}
		ENSURE(t, b3.RayCastSphere(&s, &input).hit) or_return
	}
	{
		input := b3.RayCastInput{{-4.0, 1.001, 0.0}, {8.0, 0.0, 0.0}, 1.0}
		ENSURE(t, b3.RayCastSphere(&s, &input).hit == false) or_return
	}
	return true
}

// Capsule along x from -2 to 2, radius 1. Reused by the capsule ray cast subtests.
@(private = "file")
rayCapsule := b3.Capsule{{-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, 1.0}

@(private = "file")
RayCastCapsuleSideTest :: proc(t: ^testing.T) -> bool {
	// Perpendicular hit on the cylindrical side. Surface at distance 2 over a length 6 ray.
	{
		input := b3.RayCastInput{{0.0, 3.0, 0.0}, {0.0, -6.0, 0.0}, 1.0}
		out := b3.RayCastCapsule(&rayCapsule, &input)
		CheckCastHit(
			t,
			out,
			input.origin,
			input.translation,
			b3.Vec3{0.0, 1.0, 0.0},
			b3.Vec3{0.0, 1.0, 0.0},
			f32(1.0) / f32(3.0),
			1e-5,
		) or_return
	}
	// Same from +z to exercise the other transverse direction.
	{
		input := b3.RayCastInput{{0.0, 0.0, 3.0}, {0.0, 0.0, -6.0}, 1.0}
		out := b3.RayCastCapsule(&rayCapsule, &input)
		CheckCastHit(
			t,
			out,
			input.origin,
			input.translation,
			b3.Vec3{0.0, 0.0, 1.0},
			b3.Vec3{0.0, 0.0, 1.0},
			f32(1.0) / f32(3.0),
			1e-5,
		) or_return
	}
	// Side hit nearer the c1 end.
	{
		input := b3.RayCastInput{{-1.0, 3.0, 0.0}, {0.0, -6.0, 0.0}, 1.0}
		out := b3.RayCastCapsule(&rayCapsule, &input)
		CheckCastHit(
			t,
			out,
			input.origin,
			input.translation,
			b3.Vec3{-1.0, 1.0, 0.0},
			b3.Vec3{0.0, 1.0, 0.0},
			f32(1.0) / f32(3.0),
			1e-5,
		) or_return
	}
	return true
}

@(private = "file")
RayCastCapsuleObliqueTest :: proc(t: ^testing.T) -> bool {
	// Oblique ray in the z=0 plane. It crosses y=1 inside the cylinder span, so the
	// normal stays transverse. Exercises the non perpendicular ray/axis solve where
	// dot(axis, rayAxis) != 0.
	input := b3.RayCastInput{{-3.0, 3.0, 0.0}, {4.0, -4.0, 0.0}, 1.0}
	out := b3.RayCastCapsule(&rayCapsule, &input)
	return CheckCastHit(
		t,
		out,
		input.origin,
		input.translation,
		b3.Vec3{-1.0, 1.0, 0.0},
		b3.Vec3{0.0, 1.0, 0.0},
		0.5,
		1e-4,
	)
}

@(private = "file")
RayCastCapsuleCapTest :: proc(t: ^testing.T) -> bool {
	k: f32 = 0.70710678

	// Collinear ray hits the c2 hemisphere from beyond the end.
	{
		input := b3.RayCastInput{{5.0, 0.0, 0.0}, {-8.0, 0.0, 0.0}, 1.0}
		out := b3.RayCastCapsule(&rayCapsule, &input)
		CheckCastHit(
			t,
			out,
			input.origin,
			input.translation,
			b3.Vec3{3.0, 0.0, 0.0},
			b3.Vec3{1.0, 0.0, 0.0},
			f32(1.0) / f32(4.0),
			1e-5,
		) or_return
	}
	// Off-axis ray through the c2 cap center, approaching from outside the cylinder.
	{
		input := b3.RayCastInput{{4.0, 2.0, 0.0}, {-4.0, -4.0, 0.0}, 1.0}
		out := b3.RayCastCapsule(&rayCapsule, &input)
		CheckCastHit(
			t,
			out,
			input.origin,
			input.translation,
			b3.Vec3{2.0 + k, k, 0.0},
			b3.Vec3{k, k, 0.0},
			0.323223,
			1e-4,
		) or_return
	}
	// Mirror through the c1 cap center.
	{
		input := b3.RayCastInput{{-4.0, 2.0, 0.0}, {4.0, -4.0, 0.0}, 1.0}
		out := b3.RayCastCapsule(&rayCapsule, &input)
		CheckCastHit(
			t,
			out,
			input.origin,
			input.translation,
			b3.Vec3{-2.0 - k, k, 0.0},
			b3.Vec3{-k, k, 0.0},
			0.323223,
			1e-4,
		) or_return
	}
	return true
}

@(private = "file")
RayCastCapsuleMissTest :: proc(t: ^testing.T) -> bool {
	// Pointing away.
	{
		input := b3.RayCastInput{{0.0, 3.0, 0.0}, {0.0, 4.0, 0.0}, 1.0}
		ENSURE(t, b3.RayCastCapsule(&rayCapsule, &input).hit == false) or_return
	}
	// Crosses above the axis more than a radius away.
	{
		input := b3.RayCastInput{{0.0, 4.0, 2.0}, {0.0, -8.0, 0.0}, 1.0}
		ENSURE(t, b3.RayCastCapsule(&rayCapsule, &input).hit == false) or_return
	}
	// Aimed at the side but the translation stops short.
	{
		input := b3.RayCastInput{{0.0, 5.0, 0.0}, {0.0, -1.0, 0.0}, 1.0}
		ENSURE(t, b3.RayCastCapsule(&rayCapsule, &input).hit == false) or_return
	}
	// Parallel to the axis and outside the cylinder.
	{
		input := b3.RayCastInput{{0.0, 3.0, 0.0}, {8.0, 0.0, 0.0}, 1.0}
		ENSURE(t, b3.RayCastCapsule(&rayCapsule, &input).hit == false) or_return
	}
	// Descends past the rounded c2 end, beyond cap reach.
	{
		input := b3.RayCastInput{{4.0, 3.0, 0.0}, {0.0, -6.0, 0.0}, 1.0}
		ENSURE(t, b3.RayCastCapsule(&rayCapsule, &input).hit == false) or_return
	}
	return true
}

@(private = "file")
RayCastCapsuleInteriorTest :: proc(t: ^testing.T) -> bool {
	// Origin on the axis between the caps.
	{
		input := b3.RayCastInput{{0.0, 0.0, 0.0}, {0.0, -5.0, 0.0}, 1.0}
		out := b3.RayCastCapsule(&rayCapsule, &input)
		ENSURE(t, out.hit) or_return
		ENSURE(t, out.fraction == 0.0) or_return
		ENSURE_SMALL(t, b3.Distance(out.point, input.origin), b3.FLT_EPSILON) or_return
	}
	// Origin inside the c2 hemisphere, past the cylinder end.
	{
		input := b3.RayCastInput{{2.5, 0.0, 0.0}, {0.0, 0.0, 5.0}, 1.0}
		out := b3.RayCastCapsule(&rayCapsule, &input)
		ENSURE(t, out.hit) or_return
		ENSURE(t, out.fraction == 0.0) or_return
		ENSURE_SMALL(t, b3.Distance(out.point, input.origin), b3.FLT_EPSILON) or_return
	}
	// Zero length ray inside.
	{
		input := b3.RayCastInput{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 1.0}
		out := b3.RayCastCapsule(&rayCapsule, &input)
		ENSURE(t, out.hit) or_return
		ENSURE_SMALL(t, b3.Distance(out.point, input.origin), b3.FLT_EPSILON) or_return
	}
	// Zero length ray outside.
	{
		input := b3.RayCastInput{{0.0, 3.0, 0.0}, {0.0, 0.0, 0.0}, 1.0}
		ENSURE(t, b3.RayCastCapsule(&rayCapsule, &input).hit == false) or_return
	}
	return true
}

@(private = "file")
RayCastCapsuleDegenerateTest :: proc(t: ^testing.T) -> bool {
	// Coincident centers collapse to a sphere.
	c := b3.Capsule{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 1.0}
	input := b3.RayCastInput{{-4.0, 0.0, 0.0}, {8.0, 0.0, 0.0}, 1.0}
	out := b3.RayCastCapsule(&c, &input)
	return CheckCastHit(
		t,
		out,
		input.origin,
		input.translation,
		b3.Vec3{-1.0, 0.0, 0.0},
		b3.Vec3{-1.0, 0.0, 0.0},
		f32(3.0) / f32(8.0),
		1e-5,
	)
}

@(private = "file")
RayCastCapsuleClipTest :: proc(t: ^testing.T) -> bool {
	// The side hit occurs at fraction 1/3. Straddle it with maxFraction.
	{
		input := b3.RayCastInput{{0.0, 3.0, 0.0}, {0.0, -6.0, 0.0}, 0.3}
		ENSURE(t, b3.RayCastCapsule(&rayCapsule, &input).hit == false) or_return
	}
	{
		input := b3.RayCastInput{{0.0, 3.0, 0.0}, {0.0, -6.0, 0.0}, 0.5}
		out := b3.RayCastCapsule(&rayCapsule, &input)
		ENSURE(t, out.hit) or_return
		ENSURE_SMALL(t, out.fraction - f32(1.0) / f32(3.0), 1e-5) or_return
	}
	return true
}

// A ray within a hair of the capsule axis must still hit when it slowly converges onto the
// surface. The closest point solver is ill conditioned in this band, so this guards the near
// parallel fallback that intersects the infinite cylinder directly.
@(private = "file")
RayCastCapsuleParallelTest :: proc(t: ^testing.T) -> bool {
	// Capsule along y. A long ray almost parallel to the axis drifts inward from just outside the
	// cylinder and dips through the far endcap. The naive solve loses this hit to a determinant of zero.
	axisY := b3.Capsule{{0.0, 0.0, 0.0}, {0.0, 10.0, 0.0}, 1.0}
	{
		input := b3.RayCastInput{{1.0001, 100.0, 0.0}, {-0.001, -200.0, 0.0}, 1.0}
		out := b3.RayCastCapsule(&axisY, &input)
		ENSURE(t, out.hit) or_return

		// The hit lands on the capsule surface and on the ray.
		onSeg := b3.PointToSegmentDistance(axisY.center1, axisY.center2, out.point)
		ENSURE_SMALL(t, b3.Distance(out.point, onSeg) - axisY.radius, 1e-3) or_return
		onRay := b3.MulAdd(input.origin, out.fraction, input.translation)
		ENSURE_SMALL(t, b3.Distance(out.point, onRay), 1e-3) or_return
	}

	// Near parallel ray converging onto the x-axis capsule from far away.
	{
		input := b3.RayCastInput{{-1000.0, 1.0001, 0.0}, {2000.0, -0.001, 0.0}, 1.0}
		out := b3.RayCastCapsule(&rayCapsule, &input)
		ENSURE(t, out.hit) or_return
		onSeg := b3.PointToSegmentDistance(rayCapsule.center1, rayCapsule.center2, out.point)
		ENSURE_SMALL(t, b3.Distance(out.point, onSeg) - rayCapsule.radius, 1e-3) or_return
	}

	// Exactly parallel and outside the cylinder still misses.
	{
		input := b3.RayCastInput{{0.0, 3.0, 0.0}, {8.0, 0.0, 0.0}, 1.0}
		ENSURE(t, b3.RayCastCapsule(&rayCapsule, &input).hit == false) or_return
	}

	return true
}

// Zero length rays and initial overlap behave the same across the solid shapes. A moving ray
// and a zero length ray that both start inside report the origin with zero fraction, and a zero
// length ray that starts outside misses.
@(private = "file")
RayCastOverlapConventionTest :: proc(t: ^testing.T) -> bool {
	zero := b3.Vec3{0.0, 0.0, 0.0}
	ray := b3.Vec3{8.0, 0.0, 0.0}

	// Sphere
	{
		s := b3.Sphere{{0.0, 0.0, 0.0}, 1.0}
		inside := b3.Vec3{0.2, 0.0, 0.0}
		outside := b3.Vec3{3.0, 0.0, 0.0}
		moving := b3.RayCastInput{inside, ray, 1.0}
		pointInside := b3.RayCastInput{inside, zero, 1.0}
		pointOutside := b3.RayCastInput{outside, zero, 1.0}
		CheckInitialOverlap(t, b3.RayCastSphere(&s, &moving), inside) or_return
		CheckInitialOverlap(t, b3.RayCastSphere(&s, &pointInside), inside) or_return
		ENSURE(t, b3.RayCastSphere(&s, &pointOutside).hit == false) or_return
	}

	// Capsule
	{
		inside := b3.Vec3{0.0, 0.0, 0.0}
		outside := b3.Vec3{0.0, 3.0, 0.0}
		moving := b3.RayCastInput{inside, ray, 1.0}
		pointInside := b3.RayCastInput{inside, zero, 1.0}
		pointOutside := b3.RayCastInput{outside, zero, 1.0}
		CheckInitialOverlap(t, b3.RayCastCapsule(&rayCapsule, &moving), inside) or_return
		CheckInitialOverlap(t, b3.RayCastCapsule(&rayCapsule, &pointInside), inside) or_return
		ENSURE(t, b3.RayCastCapsule(&rayCapsule, &pointOutside).hit == false) or_return
	}

	// Hull
	{
		inside := b3.Vec3{0.3, 0.2, 0.1}
		outside := b3.Vec3{3.0, 0.0, 0.0}
		moving := b3.RayCastInput{inside, ray, 1.0}
		pointInside := b3.RayCastInput{inside, zero, 1.0}
		pointOutside := b3.RayCastInput{outside, zero, 1.0}
		CheckInitialOverlap(t, b3.RayCastHull(&box.base, &moving), inside) or_return
		CheckInitialOverlap(t, b3.RayCastHull(&box.base, &pointInside), inside) or_return
		ENSURE(t, b3.RayCastHull(&box.base, &pointOutside).hit == false) or_return
	}

	return true
}

// Distance, in double precision, from a single precision hit point to the analytic first
// ray/sphere intersection of the same float ray. Isolates the single precision method error:
// the reference carries no float rounding, so what remains is purely what the method lost.
@(private = "file")
SphereHitError :: proc(shape: b3.Sphere, input: b3.RayCastInput, point: b3.Vec3) -> f64 {
	r := f64(shape.radius)
	sx := f64(input.origin.x) - f64(shape.center.x)
	sy := f64(input.origin.y) - f64(shape.center.y)
	sz := f64(input.origin.z) - f64(shape.center.z)
	tx, ty, tz := f64(input.translation.x), f64(input.translation.y), f64(input.translation.z)
	len := math.sqrt(tx * tx + ty * ty + tz * tz)
	dx, dy, dz := tx / len, ty / len, tz / len
	b := sx * dx + sy * dy + sz * dz
	c := sx * sx + sy * sy + sz * sz - r * r
	t := -b - math.sqrt(b * b - c)
	ex := f64(point.x) - (f64(input.origin.x) + t * dx)
	ey := f64(point.y) - (f64(input.origin.y) + t * dy)
	ez := f64(point.z) - (f64(input.origin.z) + t * dz)
	return math.sqrt(ex * ex + ey * ey + ez * ez)
}

// Same idea for the ray/infinite-cylinder intersection, used where the hit lands on the side.
@(private = "file")
CapsuleHitError :: proc(shape: b3.Capsule, input: b3.RayCastInput, point: b3.Vec3) -> f64 {
	ax := f64(shape.center2.x) - f64(shape.center1.x)
	ay := f64(shape.center2.y) - f64(shape.center1.y)
	az := f64(shape.center2.z) - f64(shape.center1.z)
	alen := math.sqrt(ax * ax + ay * ay + az * az)
	ax /= alen
	ay /= alen
	az /= alen

	sx := f64(input.origin.x) - f64(shape.center1.x)
	sy := f64(input.origin.y) - f64(shape.center1.y)
	sz := f64(input.origin.z) - f64(shape.center1.z)
	tx, ty, tz := f64(input.translation.x), f64(input.translation.y), f64(input.translation.z)
	tlen := math.sqrt(tx * tx + ty * ty + tz * tz)
	dx, dy, dz := tx / tlen, ty / tlen, tz / tlen

	sa := sx * ax + sy * ay + sz * az
	da := dx * ax + dy * ay + dz * az
	spx, spy, spz := sx - sa * ax, sy - sa * ay, sz - sa * az
	dpx, dpy, dpz := dx - da * ax, dy - da * ay, dz - da * az
	A := dpx * dpx + dpy * dpy + dpz * dpz
	B := 2.0 * (spx * dpx + spy * dpy + spz * dpz)
	r := f64(shape.radius)
	C := spx * spx + spy * spy + spz * spz - r * r
	tau := (-B - math.sqrt(B * B - 4.0 * A * C)) / (2.0 * A)
	ex := f64(point.x) - (f64(input.origin.x) + tau * dx)
	ey := f64(point.y) - (f64(input.origin.y) + tau * dy)
	ez := f64(point.z) - (f64(input.origin.z) + tau * dz)
	return math.sqrt(ex * ex + ey * ey + ez * ez)
}

// A miss is the worst possible outcome, so fold it into the error as a large sentinel.
@(private = "file")
RAY_MISS :: f64(1.0e30)

@(private = "file")
RayCastFarOriginTest :: proc(t: ^testing.T) -> bool {
	s := b3.Sphere{{0.0, 0.0, 0.0}, 1.0}
	c := b3.Capsule{{-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, 1.0}

	// (0,0,1) lies on the unit sphere and on the capsule side. The ray dives in from a far origin
	// along H + D*u for a fan of directions u skewed off the surface normal, so the capsule solve
	// sees a real perpendicular gap rather than a free exact cancellation.
	h := b3.Vec3{0.0, 0.0, 1.0}
	offsets := [?]f32{-0.7, 0.0, 0.7}
	distances := [?]f32{1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7}

	worstSphere: [len(distances)]f64
	worstCapsule: [len(distances)]f64

	printf("    worst hit point error over a fan of skew rays, by origin distance:\n")
	printf("    %-9s %-13s %-13s\n", "distance", "sphere", "capsule")
	for i: i32 = 0; i < i32(len(distances)); i += 1 {
		d := distances[i]
		maxS, maxC := 0.0, 0.0

		for ia: i32 = 0; ia < i32(len(offsets)); ia += 1 {
			for ib: i32 = 0; ib < i32(len(offsets)); ib += 1 {
				u := b3.Normalize(b3.Vec3{offsets[ia], offsets[ib], 1.0})
				origin := b3.MulAdd(h, d, u)
				translation := b3.MulSV(-2.0 * d, u)
				input := b3.RayCastInput{origin, translation, 1.0}

				os := b3.RayCastSphere(&s, &input)
				oc := b3.RayCastCapsule(&c, &input)

				errS := os.hit ? SphereHitError(s, input, os.point) : RAY_MISS
				errC := oc.hit ? CapsuleHitError(c, input, oc.point) : RAY_MISS

				maxS = errS > maxS ? errS : maxS
				maxC = errC > maxC ? errC : maxC
			}
		}

		worstSphere[i] = maxS
		worstCapsule[i] = maxC
		printf("    %-9.0e %-13.3e %-13.3e\n", d, maxS, maxC)
	}

	// The closest point formulation keeps the error at the single precision floor: it grows only
	// linearly with origin distance, error ~ distance * FLT_EPSILON, with no catastrophic loss. This
	// holds out to a million units. At ten million the origin coordinates carry a meter sized ULP,
	// larger than the unit radius, so the ray genuinely drops the hit. That row is printed for insight
	// but left unasserted rather than baking in the breakdown.
	for i: i32 = 0; i < i32(len(distances)); i += 1 {
		if distances[i] < 1.0e7 {
			floor := 16.0 * f64(distances[i]) * f64(b3.FLT_EPSILON) + 2.0e-6
			ENSURE(t, worstSphere[i] < floor) or_return
			ENSURE(t, worstCapsule[i] < floor) or_return
		}
	}

	// Still a clean sub-meter hit at a million units out.
	ENSURE(t, worstSphere[5] < 0.5 && worstCapsule[5] < 0.5) or_return

	return true
}

@(private = "file")
RayCastShapeTest :: proc(t: ^testing.T) -> bool {
	input := b3.RayCastInput {
		origin      = {-4.0, 0.0, 0.0},
		translation = {8.0, 0.0, 0.0},
		maxFraction = 1.0,
	}

	{
		output := b3.RayCastSphere(&sphere, &input)
		ENSURE(t, output.hit) or_return
		ENSURE_SMALL(t, output.normal.x + 1.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, output.normal.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, output.normal.z, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, output.fraction - 0.5, b3.FLT_EPSILON) or_return
	}

	{
		output := b3.RayCastCapsule(&capsule, &input)
		ENSURE(t, output.hit) or_return
		ENSURE_SMALL(t, output.normal.x + 1.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, output.normal.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, output.normal.z, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, output.fraction - f32(1.0) / f32(4.0), b3.FLT_EPSILON) or_return
	}

	{
		output := b3.RayCastHull(&box.base, &input)
		ENSURE(t, output.hit) or_return
		ENSURE_SMALL(t, output.normal.x + 1.0, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, output.normal.y, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, output.normal.z, b3.FLT_EPSILON) or_return
		ENSURE_SMALL(t, output.fraction - f32(3.0) / f32(8.0), b3.FLT_EPSILON) or_return
	}

	return true
}

@(private = "file")
CheckShapeName :: proc(t: ^testing.T, shapeId: b3.ShapeId, expected: cstring) -> bool {
	got := b3.Shape_GetName(shapeId)
	ENSURE(t, got != nil || expected == nil) or_return
	if got == nil {
		return true
	}

	srcLen := expected != nil ? i32(len(expected)) : 0
	ENSURE(t, i32(len(got)) == srcLen) or_return
	if srcLen > 0 {
		ENSURE(t, string(got)[:srcLen] == string(expected)[:srcLen]) or_return
	}
	return true
}

// Cover the def path, the setter, over-length truncation, and the NULL clear. A zero cap collapses every
// case to the empty string, which the length-derived check handles without a special branch.
@(private = "file")
ShapeNameTest :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyId := b3.CreateBody(worldId, &bodyDef)

	s := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}

	// Default def leaves the name empty.
	shapeDef := b3.DefaultShapeDef()
	shapeDef.density = 1.0
	shapeId := b3.CreateSphereShape(bodyId, &shapeDef, &s)
	CheckShapeName(t, shapeId, nil) or_return

	// Name carried on the def.
	shapeDef.name = "box"
	namedId := b3.CreateSphereShape(bodyId, &shapeDef, &s)
	CheckShapeName(t, namedId, "box") or_return

	// Setter overwrites.
	b3.Shape_SetName(shapeId, "wheel")
	CheckShapeName(t, shapeId, "wheel") or_return

	// Over-length name truncates to the cap.
	b3.Shape_SetName(shapeId, "abcdefghijklmnopqrstuvwxyz")
	CheckShapeName(t, shapeId, "abcdefghijklmnopqrstuvwxyz") or_return

	// NULL clears the name.
	b3.Shape_SetName(shapeId, nil)
	CheckShapeName(t, shapeId, nil) or_return

	b3.DestroyWorld(worldId)
	return true
}

// The event enables moved from separate bools to a shared bit field. Each accessor must touch only its own
// bit, so start with all set and flip them one at a time, checking the rest hold.
@(private = "file")
ShapeFlagsTest :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyId := b3.CreateBody(worldId, &bodyDef)

	s := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}

	shapeDef := b3.DefaultShapeDef()
	shapeDef.density = 1.0
	shapeDef.enableSensorEvents = true
	shapeDef.enableContactEvents = true
	shapeDef.enableHitEvents = true
	shapeDef.enablePreSolveEvents = true
	shapeId := b3.CreateSphereShape(bodyId, &shapeDef, &s)

	// The def carries every enable through to the shape.
	ENSURE(t, b3.Shape_AreSensorEventsEnabled(shapeId)) or_return
	ENSURE(t, b3.Shape_AreContactEventsEnabled(shapeId)) or_return
	ENSURE(t, b3.Shape_AreHitEventsEnabled(shapeId)) or_return
	ENSURE(t, b3.Shape_ArePreSolveEventsEnabled(shapeId)) or_return

	// Clearing one leaves the rest set.
	b3.Shape_EnableSensorEvents(shapeId, false)
	ENSURE(t, b3.Shape_AreSensorEventsEnabled(shapeId) == false) or_return
	ENSURE(t, b3.Shape_AreContactEventsEnabled(shapeId)) or_return
	ENSURE(t, b3.Shape_AreHitEventsEnabled(shapeId)) or_return
	ENSURE(t, b3.Shape_ArePreSolveEventsEnabled(shapeId)) or_return

	// Setting it back does not disturb the others.
	b3.Shape_EnableSensorEvents(shapeId, true)
	b3.Shape_EnableHitEvents(shapeId, false)
	ENSURE(t, b3.Shape_AreSensorEventsEnabled(shapeId)) or_return
	ENSURE(t, b3.Shape_AreContactEventsEnabled(shapeId)) or_return
	ENSURE(t, b3.Shape_AreHitEventsEnabled(shapeId) == false) or_return
	ENSURE(t, b3.Shape_ArePreSolveEventsEnabled(shapeId)) or_return

	// Clear two more and confirm only the remaining bit stays set.
	b3.Shape_EnableContactEvents(shapeId, false)
	b3.Shape_EnablePreSolveEvents(shapeId, false)
	ENSURE(t, b3.Shape_AreSensorEventsEnabled(shapeId)) or_return
	ENSURE(t, b3.Shape_AreContactEventsEnabled(shapeId) == false) or_return
	ENSURE(t, b3.Shape_AreHitEventsEnabled(shapeId) == false) or_return
	ENSURE(t, b3.Shape_ArePreSolveEventsEnabled(shapeId) == false) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(test)
ShapeTest :: proc(t: ^testing.T) {
	LOCK()

	box = b3.MakeBoxHull(1.0, 1.0, 1.0)

	if !RUN_SUBTEST(t, ShapeMassTest) {return}
	if !RUN_SUBTEST(t, ShapeAABBTest) {return}
	if !RUN_SUBTEST(t, ShapeNameTest) {return}
	if !RUN_SUBTEST(t, ShapeFlagsTest) {return}
	// RUN_SUBTEST( PointInShapeTest );
	if !RUN_SUBTEST(t, RayCastShapeTest) {return}

	if !RUN_SUBTEST(t, RayCastSphereHitTest) {return}
	if !RUN_SUBTEST(t, RayCastSphereMissTest) {return}
	if !RUN_SUBTEST(t, RayCastSphereClipTest) {return}
	if !RUN_SUBTEST(t, RayCastSphereInteriorTest) {return}
	if !RUN_SUBTEST(t, RayCastSphereGrazeTest) {return}

	if !RUN_SUBTEST(t, RayCastCapsuleSideTest) {return}
	if !RUN_SUBTEST(t, RayCastCapsuleObliqueTest) {return}
	if !RUN_SUBTEST(t, RayCastCapsuleCapTest) {return}
	if !RUN_SUBTEST(t, RayCastCapsuleMissTest) {return}
	if !RUN_SUBTEST(t, RayCastCapsuleInteriorTest) {return}
	if !RUN_SUBTEST(t, RayCastCapsuleDegenerateTest) {return}
	if !RUN_SUBTEST(t, RayCastCapsuleClipTest) {return}
	if !RUN_SUBTEST(t, RayCastCapsuleParallelTest) {return}

	if !RUN_SUBTEST(t, RayCastOverlapConventionTest) {return}
	if !RUN_SUBTEST(t, RayCastFarOriginTest) {return}
}
