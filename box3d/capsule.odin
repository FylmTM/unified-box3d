// Ported from box3d src/capsule.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:intrinsics"

ComputeCapsuleMass :: proc(shape: ^Capsule, density: f32) -> MassData {
	c1 := shape.center1
	c2 := shape.center2
	r := shape.radius

	// Cylinder
	cylinderHeight := Distance(c1, c2)
	cylinderVolume := PI * r * r * cylinderHeight
	cylinderMass := cylinderVolume * density

	// Sphere
	sphereVolume := (f32(4.0) / f32(3.0)) * PI * r * r * r
	sphereMass := sphereVolume * density

	// Local accumulated inertia
	inertia := AddMM(CylinderInertia(cylinderMass, r, cylinderHeight), SphereInertia(sphereMass, r))

	steiner := 0.125 * sphereMass * (3.0 * r + 2.0 * cylinderHeight) * cylinderHeight
	inertia.cx.x += steiner
	inertia.cz.z += steiner

	// Align capsule axis with chosen up-axis
	rotation := Mat3_identity
	if cylinderHeight * cylinderHeight > 1000.0 * FLT_MIN {
		direction := Normalize(Sub(c2, c1))
		q := ComputeQuatBetweenUnitVectors(Vec3_axisY, direction)
		rotation = MakeMatrixFromQuat(q)
	}

	mass := sphereMass + cylinderMass
	center := MulSV(0.5, Add(c1, c2))

	out: MassData
	out.mass = mass
	out.center = center

	// Rotate the central inertia into the shape frame
	out.inertia = MulMM(rotation, MulMM(inertia, Transpose(rotation)))

	return out
}

ComputeCapsuleAABB :: proc(shape: ^Capsule, transform: Transform) -> AABB {
	r := shape.radius

	center1 := TransformPoint(transform, shape.center1)
	center2 := TransformPoint(transform, shape.center2)
	extent := Vec3{r, r, r}

	aabb: AABB
	aabb.lowerBound = Sub(Min(center1, center2), extent)
	aabb.upperBound = Add(Max(center1, center2), extent)
	return aabb
}

ComputeSweptCapsuleAABB :: proc(shape: ^Capsule, xf1: Transform, xf2: Transform) -> AABB {
	r := Vec3{shape.radius, shape.radius, shape.radius}
	a := TransformPoint(xf1, shape.center1)
	b := TransformPoint(xf1, shape.center2)
	c := TransformPoint(xf2, shape.center1)
	d := TransformPoint(xf2, shape.center2)

	aabb := AABB {
		lowerBound = Sub(Min(Min(a, b), Min(c, d)), r),
		upperBound = Add(Max(Max(a, b), Max(c, d)), r),
	}
	return aabb
}

OverlapCapsule :: proc(shape: ^Capsule, shapeTransform: Transform, proxy: ^ShapeProxy) -> bool {
	input: DistanceInput
	input.proxyA = ShapeProxy{([^]Vec3)(&shape.center1), 2, shape.radius}
	input.proxyB = proxy^
	input.transform = InvMulTransforms(shapeTransform, Transform_identity)
	input.useRadii = true

	cache := SimplexCache{}
	output := ShapeDistance(&input, &cache, nil, 0)
	return output.distance < OVERLAP_SLOP
}

// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
// http://www.codercorner.com/blog/?p=321
RayCastCapsule :: proc(shape: ^Capsule, input: ^RayCastInput) -> CastOutput {
	assert(IsValidRay(input))

	c1 := shape.center1
	c2 := shape.center2
	r := shape.radius

	// Initialize result structure
	output := CastOutput{}

	d := Sub(c2, c1)

	// Fall back to sphere if the capsule is short
	tol: f32 = 0.01 * LINEAR_SLOP
	lengthSquared := LengthSquared(d)
	if lengthSquared < tol * tol {
		sphereCenter := MulSV(0.5, Add(shape.center1, shape.center2))
		sphere := Sphere{sphereCenter, shape.radius}
		return RayCastSphere(&sphere, input)
	}

	// Vector from first center to ray origin.
	s := Sub(input.origin, c1)

	// Capsule axis
	length := intrinsics.sqrt(lengthSquared)
	axis := MulSV(1.0 / length, d)

	// Project ray origin onto capsule axis.
	u := Dot(s, axis)

	// Closest point on infinite capsule axis, relative to c1.
	c := MulSV(u, axis)

	// Vector from closest point to ray origin
	sc := Sub(s, c)

	// Squared distance from ray origin to capsule axis
	sc2 := LengthSquared(sc)

	// Is the ray origin within the infinite cylinder along the capsule axis?
	if sc2 < r * r {
		// Clamped barycentric coordinate of ray origin projected onto capsule axis.
		uClamped := ClampFloat(u, 0.0, length)

		// The closest point on the bounded capsule segment, relative to c1.
		cp := MulSV(uClamped, axis)

		// Vector from ray origin to closest point on segment.
		scp := Sub(s, cp)

		// Squared distance of ray origin from capsule segment.
		scp2 := LengthSquared(scp)

		// Is the ray origin within the capsule?
		if scp2 < r * r {
			output.hit = true
			output.point = input.origin
			return output
		}

		// The ray can hit an endcap.
		sphere := Sphere {
			center = Add(c1, cp),
			radius = r,
		}

		return RayCastSphere(&sphere, input)
	}

	// Ray axis. A zero length ray reaching here starts outside the capsule, so it misses.
	// Same zero length convention as b3RayCastSphere.
	dr := input.translation
	rayLength: f32
	rayAxis := GetLengthAndNormalize(&rayLength, dr)
	if rayLength == 0.0 {
		return output
	}

	// Barycentric coordinate of ray end point.
	v := u + input.maxFraction * Dot(dr, axis)

	// Early out: does the projected ray fall outside the capsule?
	if (u < -r && v < -r) || (length + r < u && length + r < v) {
		return output
	}

	// Compute the closest point between the ray segment and the capsule segment.
	// See Real-Time Collision Detection, section 5.1.9

	// Closest point on capsule : a1 = segment unit axis, t1 = unknown fraction
	// p1 = t1 * a1

	// Closet point on ray : a2 = ray unit axis, t2 = unknown fraction
	// p2 = s + t2 * a2

	// Closest point perpendicularity conditions.
	// dot(p2 - p1, a1) = 0
	// dot(p2 - p1, a2) = 0

	// Expand
	// dot(t1 * a1 - s - t2 * a2, a1) = 0
	// dot(t1 * a1 - s - t2 * a2, a2) = 0

	// Expand
	// t1 - dot(s, a1) - t2 * dot(a1, a2) = 0
	// t1 * dot(a1, a2) - dot(s, a2) - t2 = 0

	// Group : a12 = dot(a1, a2), sa1 = dot(s, a1), sa2 = dot(s, a2)
	// t1       - a12 * t2 = sa1
	// a12 * t1 -       t2 = sa2

	// Solve
	// https://en.wikipedia.org/wiki/Cramer%27s_rule
	// I've flipped the signs of the numerator and denominator to give a positive determinant.
	// det = 1 - a12 * a12
	// t1 = (sa1 - a12 * sa2) / det
	// t2 = (a12 * sa1 - sa2) / det

	a1 := axis
	a2 := rayAxis
	a12 := Dot(a1, a2)

	// Ray distance to the near intersection with the infinite cylinder. Length units.
	tr: f32

	det := 1.0 - a12 * a12
	if det < FLT_EPSILON {
		// Solve the 2D problem of ray versus circle starting at the ray origin, where the circle is
		// the axial view of the infinite capsule cylinder. This works well when the ray origin is
		// not too far from the capsule axis.

		// Instead of a cross product, subtract the parallel part to get a perpendicular vector. Non-dimensional.
		perp := MulSub(a2, a12, a1)
		perp2 := LengthSquared(perp)

		// Project to origin to infinite capsule axis vector onto the perpendicular vector. beta has length units.
		beta := Dot(sc, perp)

		// Setup quadratic root finder.
		gamma := sc2 - r * r

		// Discriminant
		disc := beta * beta - perp2 * gamma

		// Casting away from the axis, or the perpendicular gap never closes to the radius.
		if beta >= 0.0 || disc < 0.0 {
			return output
		}

		// Quadratic near root. Expressed in an alternate form to avoid the (-beta - sqrt) cancellation as
		// the ray nears parallel.
		tr = gamma / (-beta + intrinsics.sqrt(disc))
	} else {
		// Ray and capsules axes are not parallel.

		// Closest points between the infinite ray and the infinite capsule axis.
		invDet := 1.0 / det
		sa1 := u
		sa2 := Dot(s, a2)

		t1 := (sa1 - a12 * sa2) * invDet
		t2 := (a12 * sa1 - sa2) * invDet

		// Closest points
		p1 := MulSV(t1, a1)
		p2 := MulAdd(s, t2, a2)

		// Vector from closest point on infinite capsule to infinite ray.
		g := Sub(p2, p1)

		g2 := LengthSquared(g)
		if g2 > r * r {
			// Early out: closest point on infinite ray is outside infinite cylinder.
			return output
		}

		// Intersect the infinite ray with the infinite cylinder. Like ray versus sphere this is done
		// relative to the closest point to avoid round-off errors. Length units, not a fraction.
		// https://en.wikipedia.org/wiki/Line-cylinder_intersection
		h := intrinsics.sqrt((r * r - g2) * invDet)

		tr = t2 - h
	}

	// Outside ray?
	if tr < 0.0 || input.maxFraction * rayLength < tr {
		return output
	}

	// The corresponding distance on the capsule axis. Length units.
	tc := u + tr * a12

	// Outside c1 end?
	if tc < 0.0 {
		// Ray cast sphere 1.
		sphere := Sphere {
			center = c1,
			radius = r,
		}

		return RayCastSphere(&sphere, input)
	}

	// Outside c2 end?
	if length < tc {
		// Ray cast sphere 2.
		sphere := Sphere {
			center = c2,
			radius = r,
		}

		return RayCastSphere(&sphere, input)
	}

	// Hit point on capsule side, relative to c1.
	p := MulAdd(s, tr, rayAxis)

	// Hit normal.
	normal := MulSub(p, tc, axis)
	normal = Normalize(normal)

	output.point = Add(c1, p)
	output.normal = normal
	output.fraction = ClampFloat(tr / rayLength, 0.0, input.maxFraction)
	output.hit = true
	return output
}

ShapeCastCapsule :: proc(capsule: ^Capsule, input: ^ShapeCastInput) -> CastOutput {
	pairInput: ShapeCastPairInput
	pairInput.proxyA = ShapeProxy{([^]Vec3)(&capsule.center1), 2, capsule.radius}
	pairInput.proxyB = input.proxy
	pairInput.transform = Transform_identity
	pairInput.translationB = input.translation
	pairInput.maxFraction = input.maxFraction
	pairInput.canEncroach = input.canEncroach

	output := ShapeCast(&pairInput)
	return output
}

CollideMoverAndCapsule :: proc(result: ^PlaneResult, shape: ^Capsule, mover: ^Capsule) -> i32 {
	totalRadius := mover.radius + shape.radius

	approach := SegmentDistance(shape.center1, shape.center2, mover.center1, mover.center2)

	// The normal points from the shape toward the mover.
	distance: f32
	normal := GetLengthAndNormalize(&distance, Sub(approach.point2, approach.point1))

	if distance > totalRadius {
		return 0
	}

	linearSlop: f32 = LINEAR_SLOP
	if distance < linearSlop {
		// Deep overlap: the core segments intersect. Pick an arbitrary direction perpendicular
		// the to capsule axis.
		moverLength: f32
		moverAxis := GetLengthAndNormalize(&moverLength, Sub(mover.center2, mover.center1))
		normal = moverLength > linearSlop ? Perp(moverAxis) : Vec3_axisY
		distance = 0.0
	}

	plane := Plane{normal, totalRadius - distance}
	result^ = PlaneResult{plane, approach.point1, 0, 0, 0}
	return 1
}
