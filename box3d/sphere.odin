// Ported from box3d src/sphere.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:intrinsics"

// Dirk Gregorius contributed portions of this code

ComputeSphereMass :: proc(shape: ^Sphere, density: f32) -> MassData {
	center := shape.center
	radius := shape.radius

	volume := f32(4.0) / f32(3.0) * PI * radius * radius * radius
	mass := volume * density
	ixx := 0.4 * mass * radius * radius

	out: MassData
	out.mass = mass
	out.center = center

	// Inertia about the center of mass
	out.inertia = MakeDiagonalMatrix(ixx, ixx, ixx)
	return out
}

ComputeSphereAABB :: proc(shape: ^Sphere, transform: Transform) -> AABB {
	center := TransformPoint(transform, shape.center)
	radius := shape.radius
	extent := Vec3{radius, radius, radius}
	return AABB{Sub(center, extent), Add(center, extent)}
}

ComputeSweptSphereAABB :: proc(shape: ^Sphere, xf1: Transform, xf2: Transform) -> AABB {
	r := Vec3{shape.radius, shape.radius, shape.radius}
	center1 := TransformPoint(xf1, shape.center)
	center2 := TransformPoint(xf2, shape.center)
	return AABB{Sub(Min(center1, center2), r), Add(Max(center1, center2), r)}
}

OverlapSphere :: proc(shape: ^Sphere, shapeTransform: Transform, proxy: ^ShapeProxy) -> bool {
	input: DistanceInput
	input.proxyA = ShapeProxy{([^]Vec3)(&shape.center), 1, shape.radius}
	input.proxyB = proxy^
	input.transform = InvMulTransforms(shapeTransform, Transform_identity)
	input.useRadii = true

	cache := SimplexCache{}
	output := ShapeDistance(&input, &cache, nil, 0)
	return output.distance < OVERLAP_SLOP
}

// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
// http://www.codercorner.com/blog/?p=321
RayCastSphere :: proc(shape: ^Sphere, input: ^RayCastInput) -> CastOutput {
	assert(IsValidRay(input))
	output := CastOutput{}

	p := shape.center

	// Shift ray so sphere center is the origin
	s := Sub(input.origin, p)

	r := shape.radius
	rr := r * r

	length: f32
	d := GetLengthAndNormalize(&length, input.translation)
	if length == 0.0 {
		// zero length ray

		if LengthSquared(s) < rr {
			// initial overlap
			output.point = input.origin
			output.hit = true
		}

		return output
	}

	// Find closest point on ray to origin

	// solve: dot(s + t * d, d) = 0
	t := -Dot(s, d)

	// c is the closest point on the line to the origin
	c := MulAdd(s, t, d)

	cc := Dot(c, c)

	if cc > rr {
		// closest point is outside the sphere
		return output
	}

	// Pythagoras
	h := intrinsics.sqrt(rr - cc)

	fraction := t - h

	if fraction < 0.0 || input.maxFraction * length < fraction {
		// intersection is point outside the range of the ray segment

		if LengthSquared(s) < rr {
			// initial overlap
			output.point = input.origin
			output.hit = true
		}

		return output
	}

	hitPoint := MulAdd(s, fraction, d)

	output.fraction = fraction / length

	if output.fraction > input.maxFraction {
		Log("sphere input fraction = %g, output fraction = %g", input.maxFraction, output.fraction)
		output.fraction = input.maxFraction
	}

	output.normal = Normalize(hitPoint)
	output.point = MulAdd(p, shape.radius, output.normal)
	output.hit = true

	return output
}

// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
// http://www.codercorner.com/blog/?p=321
// This will do interior hits.
RayCastHollowSphere :: proc(sphere: ^Sphere, input: ^RayCastInput) -> CastOutput {
	p := sphere.center

	output := CastOutput{}

	// Shift ray so sphere center is the origin
	s := Sub(input.origin, p)
	d := Normalize(input.translation)

	// Find closest point on ray to origin

	// solve: dot(s + t * d, d) = 0
	t := -Dot(s, d)

	// c is the closest point on the line to the origin
	c := MulAdd(s, t, d)

	cc := Dot(c, c)
	r := sphere.radius
	rr := r * r

	if cc > rr {
		// closest point is outside the sphere
		return output
	}

	// Pythagoras
	h := intrinsics.sqrt(rr - cc)

	fraction := t - h

	if fraction < 0.0 {
		fraction = t + h
	}

	if fraction < 0.0 {
		// behind the ray
		return output
	}

	if fraction > input.maxFraction {
		return output
	}

	hitPoint := MulAdd(s, fraction, d)

	output.fraction = fraction
	output.normal = Normalize(hitPoint)
	output.point = MulAdd(p, sphere.radius, output.normal)
	output.hit = true

	return output
}

ShapeCastSphere :: proc(sphere: ^Sphere, input: ^ShapeCastInput) -> CastOutput {
	pairInput: ShapeCastPairInput
	pairInput.proxyA = ShapeProxy{([^]Vec3)(&sphere.center), 1, sphere.radius}
	pairInput.proxyB = input.proxy
	pairInput.transform = Transform_identity
	pairInput.translationB = input.translation
	pairInput.maxFraction = input.maxFraction
	pairInput.canEncroach = input.canEncroach

	output := ShapeCast(&pairInput)
	return output
}

CollideMoverAndSphere :: proc(result: ^PlaneResult, shape: ^Sphere, mover: ^Capsule) -> i32 {
	totalRadius := mover.radius + shape.radius
	closest := PointToSegmentDistance(mover.center1, mover.center2, shape.center)

	// The normal points from the sphere toward the mover.
	distance: f32
	normal := GetLengthAndNormalize(&distance, Sub(closest, shape.center))

	if distance > totalRadius {
		return 0
	}

	linearSlop: f32 = LINEAR_SLOP
	if distance < linearSlop {
		// Deep overlap: the mover axis passes through the sphere center, so no
		// direction is preferred. Push perpendicular to the mover axis.
		length: f32
		axis := GetLengthAndNormalize(&length, Sub(mover.center2, mover.center1))
		normal = length > linearSlop ? Perp(axis) : Vec3_axisY
		distance = 0.0
	}

	plane := Plane{normal, totalRadius - distance}
	result^ = PlaneResult{plane, shape.center, 0, 0, 0}
	return 1
}
