// Ported from box3d src/math_functions.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:intrinsics"

// unified: isnan(a) is a != a, isinf(a) is |a| > FLT_MAX (no libm).

// Is this a valid number? Not NaN or infinity.
IsValidFloat :: proc(a: f32) -> bool {
	if a != a {
		return false
	}

	if abs(a) > FLT_MAX {
		return false
	}

	return true
}

// Is this a valid vector? Not NaN or infinity.
IsValidVec3 :: proc(a: Vec3) -> bool {
	if a.x != a.x || a.y != a.y || a.z != a.z {
		return false
	}

	if abs(a.x) > FLT_MAX || abs(a.y) > FLT_MAX || abs(a.z) > FLT_MAX {
		return false
	}

	return true
}

// Is this a valid quaternion? Not NaN or infinity. Is normalized.
IsValidQuat :: proc(a: Quat) -> bool {
	if a.v.x != a.v.x || a.v.y != a.v.y || a.v.z != a.v.z || a.s != a.s {
		return false
	}

	if abs(a.v.x) > FLT_MAX || abs(a.v.y) > FLT_MAX || abs(a.v.z) > FLT_MAX || abs(a.s) > FLT_MAX {
		return false
	}

	return IsNormalizedQuat(a)
}

// Is this a valid transform? Not NaN or infinity. Is normalized.
IsValidTransform :: proc(a: Transform) -> bool {
	return IsValidVec3(a.p) && IsValidQuat(a.q)
}

// Is this a valid matrix? Not NaN or infinity.
IsValidMatrix3 :: proc(a: Matrix3) -> bool {
	return IsValidVec3(a.cx) && IsValidVec3(a.cy) && IsValidVec3(a.cz)
}

// Is this a valid bounding box? Not Nan or infinity. Upper bound greater than or equal to lower bound.
IsValidAABB :: proc(a: AABB) -> bool {
	if IsValidVec3(a.lowerBound) == false {
		return false
	}

	if IsValidVec3(a.upperBound) == false {
		return false
	}

	if a.lowerBound.x > a.upperBound.x {
		return false
	}

	if a.lowerBound.y > a.upperBound.y {
		return false
	}

	if a.lowerBound.z > a.upperBound.z {
		return false
	}

	return true
}

// Is this AABB reasonably close to the origin? See B3_HUGE.
IsBoundedAABB :: proc(a: AABB) -> bool {
	if a.lowerBound.x < -HUGE || a.lowerBound.y < -HUGE || a.lowerBound.z < -HUGE {
		return false
	}

	if a.upperBound.x > HUGE || a.upperBound.y > HUGE || a.upperBound.z > HUGE {
		return false
	}

	return true
}

// Is this AABB valid and reasonable?
IsSaneAABB :: proc(a: AABB) -> bool {
	if IsValidAABB(a) == false {
		return false
	}

	if a.lowerBound.x < -HUGE || a.lowerBound.y < -HUGE || a.lowerBound.z < -HUGE {
		return false
	}

	if a.upperBound.x > HUGE || a.upperBound.y > HUGE || a.upperBound.z > HUGE {
		return false
	}

	return true
}

// Is this a valid plane? Normal is a unit vector. Not Nan or infinity.
IsValidPlane :: proc(a: Plane) -> bool {
	if IsValidVec3(a.normal) == false {
		return false
	}

	if IsNormalized(a.normal) == false {
		return false
	}

	return IsValidFloat(a.offset)
}

// Is this a valid world position? Not NaN or infinity.
IsValidPosition :: proc(p: Pos) -> bool {
	if p.x != p.x || p.y != p.y || p.z != p.z {
		return false
	}

	if abs(p.x) > FLT_MAX || abs(p.y) > FLT_MAX || abs(p.z) > FLT_MAX {
		return false
	}

	return true
}

// Is this a valid world transform? Not NaN or infinity. Rotation is normalized.
IsValidWorldTransform :: proc(t: WorldTransform) -> bool {
	return IsValidPosition(t.p) && IsValidQuat(t.q)
}

// Compute an approximate arctangent in the range [-pi, pi]
// This is hand coded for cross-platform determinism. The atan2f
// function in the standard library is not cross-platform deterministic.
// Accurate to around 0.0023 degrees.
// https://stackoverflow.com/questions/46210708/atan2-approximation-with-11bits-in-mantissa-on-x86with-sse2-and-armwith-vfpv4
Atan2 :: proc(y, x: f32) -> f32 {
	// Added check for (0,0) to match atan2f and avoid NaN
	if x == 0.0 && y == 0.0 {
		return 0.0
	}

	ax := AbsFloat(x)
	ay := AbsFloat(y)
	mx := MaxFloat(ay, ax)
	mn := MinFloat(ay, ax)
	a := mn / mx

	// Minimax polynomial approximation to atan(a) on [0,1]
	s := a * a
	c := s * a
	q := s * s
	r := 0.024840285 * q + 0.18681418
	t := -0.094097948 * q - 0.33213072
	r = r * s + t
	r = r * c + a

	// Map to full circle
	if ay > ax {
		r = 1.57079637 - r
	}

	if x < 0 {
		r = 3.14159274 - r
	}

	if y < 0 {
		r = -r
	}

	return r
}

// Compute the cosine and sine of an angle in radians. Implemented
// for cross-platform determinism.
// Approximate cosine and sine for determinism. In my testing cosf and sinf produced
// the same results on x64 and ARM using MSVC, GCC, and Clang. However, I don't trust
// this result.
// https://en.wikipedia.org/wiki/Bh%C4%81skara_I%27s_sine_approximation_formula
ComputeCosSin :: proc(radians: f32) -> CosSin {
	// unified: not ported: the disabled (#if 0) cosf/sinf variant.
	x := UnwindAngle(radians)
	pi2 := PI * PI

	// cosine needs angle in [-pi/2, pi/2]
	c: f32
	if x < -0.5 * PI {
		y := x + PI
		y2 := y * y
		c = -(pi2 - 4.0 * y2) / (pi2 + y2)
	} else if x > 0.5 * PI {
		y := x - PI
		y2 := y * y
		c = -(pi2 - 4.0 * y2) / (pi2 + y2)
	} else {
		y2 := x * x
		c = (pi2 - 4.0 * y2) / (pi2 + y2)
	}

	// sine needs angle in [0, pi]
	s: f32
	if x < 0.0 {
		y := x + PI
		s = -16.0 * y * (PI - y) / (5.0 * pi2 - 4.0 * y * (PI - y))
	} else {
		s = 16.0 * x * (PI - x) / (5.0 * pi2 - 4.0 * x * (PI - x))
	}

	mag := intrinsics.sqrt(s * s + c * c)
	invMag := mag > 0.0 ? 1.0 / mag : 0.0
	cs := CosSin{c * invMag, s * invMag}
	return cs
}

// Extract a quaternion from a rotation matrix.
MakeQuatFromMatrix :: proc(m: ^Matrix3) -> Quat {
	c1 := m.cx
	c2 := m.cy
	c3 := m.cz

	q: Quat

	trace := m.cx.x + m.cy.y + m.cz.z
	if trace >= 0.0 {
		q.v.x = c2.z - c3.y
		q.v.y = c3.x - c1.z
		q.v.z = c1.y - c2.x
		q.s = trace + 1.0
	} else {
		if c1.x > c2.y && c1.x > c3.z {
			q.v.x = c1.x - c2.y - c3.z + 1.0
			q.v.y = c2.x + c1.y
			q.v.z = c3.x + c1.z
			q.s = c2.z - c3.y
		} else if c2.y > c3.z {
			q.v.x = c1.y + c2.x
			q.v.y = c2.y - c3.z - c1.x + 1.0
			q.v.z = c3.y + c2.z
			q.s = c3.x - c1.z
		} else {
			q.v.x = c1.z + c3.x
			q.v.y = c2.z + c3.y
			q.v.z = c3.z - c1.x - c2.y + 1.0
			q.s = c1.y - c2.x
		}
	}

	// The algorithm is simplified and made more accurate by normalizing at the end
	return NormalizeQuat(q)
}

// Find a quaternion that rotates one vector to another.
ComputeQuatBetweenUnitVectors :: proc(v1, v2: Vec3) -> Quat {
	assert(IsNormalized(v1))
	assert(IsNormalized(v2))

	out: Quat

	m := Lerp(v1, v2, 0.5)
	tolerance := 100.0 * FLT_EPSILON
	if LengthSquared(m) > tolerance * tolerance {
		out.v = Cross(v1, m)
		out.s = Dot(v1, m)
	} else {
		// Anti-parallel: Use a perpendicular vector
		if AbsFloat(v1.x) > 0.5 {
			out.v.x = v1.y
			out.v.y = -v1.x
			out.v.z = 0.0
		} else {
			out.v.x = 0.0
			out.v.y = v1.z
			out.v.z = -v1.y
		}

		out.s = 0.0
	}

	// The algorithm is simplified and made more accurate by normalizing at the end
	return NormalizeQuat(out)
}

// Compute the closest points on two infinite lines.
LineDistance :: proc(p1, d1, p2, d2: Vec3) -> SegmentDistanceResult {
	result: SegmentDistanceResult

	// Solve A*x = b
	a11 := Dot(d1, d1)
	a12 := -Dot(d1, d2)
	a21 := Dot(d2, d1)
	a22 := -Dot(d2, d2)

	w := Sub(p1, p2)
	b1 := -Dot(d1, w)
	b2 := -Dot(d2, w)

	det := a11 * a22 - a12 * a21
	if det * det < 1000.0 * FLT_MIN {
		// Lines are parallel - project p2 onto line L1: x1 = p1 + s1 * d1
		s1 := Dot(Sub(p2, p1), d1) / Dot(d1, d1)
		s2: f32 = 0.0

		result.point1 = MulAdd(p1, s1, d1)
		result.fraction1 = s1
		result.point2 = MulAdd(p2, s2, d2)
		result.fraction2 = s2

		return result
	}

	s1 := (a22 * b1 - a12 * b2) / det
	s2 := (a11 * b2 - a21 * b1) / det

	result.point1 = MulAdd(p1, s1, d1)
	result.fraction1 = s1
	result.point2 = MulAdd(p2, s2, d2)
	result.fraction2 = s2
	return result
}

// Compute the closest points on two line segments.
SegmentDistance :: proc(p1, q1, p2, q2: Vec3) -> SegmentDistanceResult {
	result: SegmentDistanceResult

	d1 := Sub(q1, p1)
	d2 := Sub(q2, p2)
	r := Sub(p1, p2)

	a := Dot(d1, d1)
	b := Dot(d1, d2)
	c := Dot(d1, r)
	e := Dot(d2, d2)
	f := Dot(d2, r)

	// Check if one of the segments degenerates into a point
	if a < 100.0 * FLT_EPSILON && e < 100.0 * FLT_EPSILON {
		// Both segments degenerate into points
		result.point1 = p1
		result.fraction1 = 0.0
		result.point2 = p2
		result.fraction2 = 0.0

		return result
	}

	if a < 100.0 * FLT_EPSILON {
		// First segment degenerates into a point
		s2 := ClampFloat(f / e, 0.0, 1.0)

		result.point1 = p1
		result.fraction1 = 0.0
		result.point2 = MulAdd(p2, s2, d2)
		result.fraction2 = s2

		return result
	}

	if e < 100.0 * FLT_EPSILON {
		// Second segment degenerates into a point
		s1 := ClampFloat(-c / a, 0.0, 1.0)

		result.point1 = MulAdd(p1, s1, d1)
		result.fraction1 = s1
		result.point2 = p2
		result.fraction2 = 0.0

		return result
	}

	// Non-degenerate case
	denom := a * e - b * b
	s1 := denom > 1000.0 * FLT_MIN ? ClampFloat((b * f - c * e) / denom, 0.0, 1.0) : 0.0
	s2 := (b * s1 + f) / e

	// Clamp lambda2 and recompute lambda1 if necessary
	if s2 < 0.0 {
		s1 = ClampFloat(-c / a, 0.0, 1.0)
		s2 = 0.0
	} else if s2 > 1.0 {
		s1 = ClampFloat((b - c) / a, 0.0, 1.0)
		s2 = 1.0
	}

	result.point1 = MulAdd(p1, s1, d1)
	result.fraction1 = s1
	result.point2 = MulAdd(p2, s2, d2)
	result.fraction2 = s2

	return result
}

// Compute the closest point on the segment a-b to the target q.
PointToSegmentDistance :: proc(a, b, q: Vec3) -> Vec3 {
	ab := Sub(b, a)
	aq := Sub(q, a)

	alpha := Dot(ab, aq)

	if alpha <= 0.0 {
		// q projects outside interval [a, b] on the side of a
		return a
	} else {
		denominator := Dot(ab, ab)
		if alpha > denominator {
			// q projects outside interval [a, b] on the side of b
			return b
		} else {
			// q projects inside interval [a, b]
			alpha /= denominator
			return MulAdd(a, alpha, ab)
		}
	}
}

ClosestPointOnTriangle :: proc(a, b, c, q: Vec3) -> TrianglePoint {
	// Check if P lies in vertex region of A
	ab := Sub(b, a)
	ac := Sub(c, a)
	aq := Sub(q, a)

	d1 := Dot(ab, aq)
	d2 := Dot(ac, aq)
	if d1 <= 0.0 && d2 <= 0.0 {
		return TrianglePoint{a, .featureVertex1}
	}

	// Check if P lies in vertex region of B
	bq := Sub(q, b)

	d3 := Dot(ab, bq)
	d4 := Dot(ac, bq)
	if d3 > 0.0 && d4 <= d3 {
		return TrianglePoint{b, .featureVertex2}
	}

	// Check if P lies in edge region AB
	vc := d1 * d4 - d3 * d2
	if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
		t := d1 / (d1 - d3)
		return TrianglePoint{MulAdd(a, t, ab), .featureEdge1}
	}

	// Check if P lies in vertex region of C
	cq := Sub(q, c)

	d5 := Dot(ab, cq)
	d6 := Dot(ac, cq)
	if d6 >= 0.0 && d5 <= d6 {
		return TrianglePoint{c, .featureVertex3}
	}

	// Check if P lies in edge region AC
	vb := d5 * d2 - d1 * d6
	if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
		t := d2 / (d2 - d6)
		return TrianglePoint{MulAdd(a, t, ac), .featureEdge3}
	}

	// Check if P lies in edge region of BC
	va := d3 * d6 - d5 * d4
	if va <= 0.0 && d4 >= d3 && d5 >= d6 {
		bc := Sub(c, b)

		t := (d4 - d3) / ((d4 - d3) + (d5 - d6))
		return TrianglePoint{MulAdd(b, t, bc), .featureEdge2}
	}

	// P inside face region ABC
	t1 := vb / (va + vb + vc)
	t2 := vc / (va + vb + vc)

	p := MulAdd(a, t1, ab)
	p = MulAdd(p, t2, ac)
	return TrianglePoint{p, .featureTriangleFace}
}

SphereInertia :: proc(mass, radius: f32) -> Matrix3 {
	i := 0.4 * mass * radius * radius
	return MakeDiagonalMatrix(i, i, i)
}

CylinderInertia :: proc(mass, radius, height: f32) -> Matrix3 {
	ixx := mass * (3 * radius * radius + height * height) / 12.0
	iyy := 0.5 * mass * radius * radius
	return MakeDiagonalMatrix(ixx, iyy, ixx)
}

BoxInertia :: proc(mass: f32, min, max: Vec3) -> Matrix3 {
	delta := Sub(max, min)
	ixx := mass * (delta.y * delta.y + delta.z * delta.z) / 12.0
	iyy := mass * (delta.x * delta.x + delta.z * delta.z) / 12.0
	izz := mass * (delta.x * delta.x + delta.y * delta.y) / 12.0

	return MakeDiagonalMatrix(ixx, iyy, izz)
}

// Get the inertia tensor of an offset point.
// https://en.wikipedia.org/wiki/Parallel_axis_theorem
Steiner :: proc(mass: f32, origin: Vec3) -> Matrix3 {
	// Usage: Io = Ic + Is and Ic = Io - Is
	ixx := mass * (origin.y * origin.y + origin.z * origin.z)
	iyy := mass * (origin.x * origin.x + origin.z * origin.z)
	izz := mass * (origin.x * origin.x + origin.y * origin.y)
	ixy := -mass * origin.x * origin.y
	ixz := -mass * origin.x * origin.z
	iyz := -mass * origin.y * origin.z

	// Write
	out: Matrix3
	out.cx.x = ixx
	out.cy.x = ixy
	out.cz.x = ixz
	out.cx.y = ixy
	out.cy.y = iyy
	out.cz.y = iyz
	out.cx.z = ixz
	out.cy.z = iyz
	out.cz.z = izz

	return out
}

// Use this to ensure your ray cast input is valid and avoid internal assertions.
IsValidRay :: proc(input: ^RayCastInput) -> bool {
	isValid :=
		IsValidVec3(input.origin) &&
		IsValidVec3(input.translation) &&
		IsValidFloat(input.maxFraction) &&
		0.0 <= input.maxFraction &&
		input.maxFraction < HUGE
	return isValid
}
