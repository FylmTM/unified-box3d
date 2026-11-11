// Ported from box3d include/box3d/math_functions.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:intrinsics"

// unified: <float.h>'s limits, as typed constants (exact: 2^-23, 2^-126 and the largest finite f32).
FLT_EPSILON :: f32(1.1920929e-7)
FLT_MIN :: f32(1.17549435e-38)
FLT_MAX :: max(f32)
// unified end

// Math
// Vector math types and functions

// https://en.wikipedia.org/wiki/Pi
PI :: f32(3.14159265359)

// Convenience macro to convert from degrees to radians.
DEG_TO_RAD :: f32(0.01745329251)

// Convenience macro to convert from radians to degrees.
RAD_TO_DEG :: f32(57.2957795131)

// Minimum scale used for scaling collision meshes, etc.
MIN_SCALE :: f32(0.01)

// A 2D vector.
Vec2 :: struct {
	x: f32,
	y: f32,
}

// A 3D vector.
Vec3 :: struct {
	x: f32,
	y: f32,
	z: f32,
}

// Cosine and sine pair.
// This uses a custom implementation designed for cross-platform determinism.
CosSin :: struct {
	// cosine and sine
	cosine: f32,
	sine:   f32,
}

// A quaternion.
Quat :: struct {
	v: Vec3,
	s: f32,
}

// A rigid transform.
Transform :: struct {
	p: Vec3,
	q: Quat,
}

// unified: single precision only: the BOX3D_DOUBLE_PRECISION b3Pos and b3WorldTransform structs are not ported.

// In single precision mode these types are the same.
Pos :: Vec3

// In single precision mode these types are the same.
WorldTransform :: Transform

// A 3x3 matrix.
Matrix3 :: struct {
	cx, cy, cz: Vec3,
}

// Axis aligned bounding box.
AABB :: struct {
	lowerBound: Vec3,
	upperBound: Vec3,
}

// A plane.
// separation = dot(normal, point) - offset
Plane :: struct {
	normal: Vec3,
	offset: f32,
}

Vec3_zero :: Vec3{0.0, 0.0, 0.0}
Vec3_one :: Vec3{1.0, 1.0, 1.0}
Vec3_axisX :: Vec3{1.0, 0.0, 0.0}
Vec3_axisY :: Vec3{0.0, 1.0, 0.0}
Vec3_axisZ :: Vec3{0.0, 0.0, 1.0}
Quat_identity :: Quat{{0.0, 0.0, 0.0}, 1.0}
Transform_identity :: Transform{{0.0, 0.0, 0.0}, {{0.0, 0.0, 0.0}, 1.0}}
Mat3_zero :: Matrix3{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}
Mat3_identity :: Matrix3{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}

// Valid in both modes: 0.0f promotes to double, the identity rotation stays float
Pos_zero :: Pos{0.0, 0.0, 0.0}
WorldTransform_identity :: WorldTransform{{0.0, 0.0, 0.0}, {{0.0, 0.0, 0.0}, 1.0}}

// @return the minimum of two integers.
MinInt :: #force_inline proc(a, b: i32) -> i32 {
	return a < b ? a : b
}

// @return the maximum of two integers.
MaxInt :: #force_inline proc(a, b: i32) -> i32 {
	return a > b ? a : b
}

// @return an integer clamped between a lower and upper bound.
ClampInt :: #force_inline proc(a, lower, upper: i32) -> i32 {
	return a < lower ? lower : (upper < a ? upper : a)
}

// @return the absolute value of a float.
AbsFloat :: #force_inline proc(a: f32) -> f32 {
	return a < 0 ? -a : a
}

// @return the minimum of two floats.
MinFloat :: #force_inline proc(a, b: f32) -> f32 {
	return a < b ? a : b
}

// @return the maximum of two floats.
MaxFloat :: #force_inline proc(a, b: f32) -> f32 {
	return a > b ? a : b
}

// @return a float clamped between a lower and upper bound.
ClampFloat :: #force_inline proc(a, lower, upper: f32) -> f32 {
	return a < lower ? lower : (upper < a ? upper : a)
}

// Interpolate a scalar.
LerpFloat :: #force_inline proc(a, b, alpha: f32) -> f32 {
	return (1.0 - alpha) * a + alpha * b
}

// @deprecated
Sin :: #force_inline proc(radians: f32) -> f32 {
	cs := ComputeCosSin(radians)
	return cs.sine
}

// @deprecated
Cos :: #force_inline proc(radians: f32) -> f32 {
	cs := ComputeCosSin(radians)
	return cs.cosine
}

// unified: remainderf without libm. The IEEE remainder is exact (x - n * y, n = x / y rounded to nearest, ties to
// even), so every conforming remainderf returns these bits. Long division by doubling |y|: each subtraction is
// exact (Sterbenz), and the last quotient bit breaks the tie.
@(private = "file")
remainderf :: #force_inline proc(x, y: f32) -> f32 {
	if x != x || y != y {
		return x + y
	}

	ax := abs(x)
	ay := abs(y)
	if ay == 0.0 || ax > FLT_MAX {
		return (x * y) / (x * y)
	}

	if ay > FLT_MAX || ax == 0.0 {
		return x
	}

	r := ax
	odd := false
	if r >= ay {
		t := ay
		for t + t <= r {
			t += t
		}

		for {
			bit := r >= t
			if bit {
				r -= t
			}
			if t == ay {
				odd = bit
				break
			}
			t *= 0.5
		}
	}

	if r + r > ay || (r + r == ay && odd) {
		r -= ay
	}

	return x < 0.0 ? -r : r
}
// unified end

// Convert any angle into the range [-pi, pi].
UnwindAngle :: #force_inline proc(radians: f32) -> f32 {
	// Assuming this is deterministic
	return remainderf(radians, 2.0 * PI)
}

// Vector addition.
Add :: #force_inline proc(a, b: Vec3) -> Vec3 {
	return Vec3{a.x + b.x, a.y + b.y, a.z + b.z}
}

// Vector subtraction.
Sub :: #force_inline proc(a, b: Vec3) -> Vec3 {
	return Vec3{a.x - b.x, a.y - b.y, a.z - b.z}
}

// Vector component-wise multiplication.
Mul :: #force_inline proc(a, b: Vec3) -> Vec3 {
	return Vec3{a.x * b.x, a.y * b.y, a.z * b.z}
}

// Vector negation.
Neg :: #force_inline proc(a: Vec3) -> Vec3 {
	return Vec3{-a.x, -a.y, -a.z}
}

// Vector dot product.
Dot :: #force_inline proc(a, b: Vec3) -> f32 {
	return a.x * b.x + a.y * b.y + a.z * b.z
}

// Vector length.
Length :: #force_inline proc(v: Vec3) -> f32 {
	return intrinsics.sqrt(Dot(v, v))
}

// Vector length squared.
LengthSquared :: #force_inline proc(a: Vec3) -> f32 {
	return a.x * a.x + a.y * a.y + a.z * a.z
}

// Distance between two points.
Distance :: #force_inline proc(a, b: Vec3) -> f32 {
	dv := Vec3{b.x - a.x, b.y - a.y, b.z - a.z}
	return Length(dv)
}

// Squared distance between two points.
DistanceSquared :: #force_inline proc(a, b: Vec3) -> f32 {
	dv := Vec3{b.x - a.x, b.y - a.y, b.z - a.z}
	return dv.x * dv.x + dv.y * dv.y + dv.z * dv.z
}

// Normalize a vector. Returns a zero vector if the input vector is very small.
Normalize :: #force_inline proc(a: Vec3) -> Vec3 {
	lengthSquared := a.x * a.x + a.y * a.y + a.z * a.z
	if lengthSquared > 1000.0 * FLT_MIN {
		s := 1.0 / intrinsics.sqrt(lengthSquared)
		u := Vec3{s * a.x, s * a.y, s * a.z}
		return u
	}
	return Vec3{0.0, 0.0, 0.0}
}

// Normalize a vector and return the length. Returns a zero vector
// if the input is very small.
GetLengthAndNormalize :: #force_inline proc(length: ^f32, a: Vec3) -> Vec3 {
	lengthSquared := a.x * a.x + a.y * a.y + a.z * a.z
	if lengthSquared > 1000.0 * FLT_MIN {
		length^ = intrinsics.sqrt(lengthSquared)
		s := 1.0 / length^
		u := Vec3{s * a.x, s * a.y, s * a.z}
		return u
	}

	length^ = 0.0
	return Vec3{0.0, 0.0, 0.0}
}

// Get a unit vector that is perpendicular to the supplied vector.
Perp :: #force_inline proc(a: Vec3) -> Vec3 {
	// Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
	// Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component
	// of a unit vector must be greater or equal to 0.57735.
	p: Vec3
	if a.x < -0.5 || 0.5 < a.x {
		p = Vec3{a.y, -a.x, 0.0}
	} else {
		p = Vec3{0.0, a.z, -a.y}
	}

	return Normalize(p)
}

// Is a vector normalized? In other words, does it have unit length?
IsNormalized :: #force_inline proc(a: Vec3) -> bool {
	aa := Dot(a, a)
	return AbsFloat(1.0 - aa) < 100.0 * FLT_EPSILON
}

// a + s * b
MulAdd :: #force_inline proc(a: Vec3, s: f32, b: Vec3) -> Vec3 {
	return Vec3{a.x + s * b.x, a.y + s * b.y, a.z + s * b.z}
}

// a - s * b
MulSub :: #force_inline proc(a: Vec3, s: f32, b: Vec3) -> Vec3 {
	return Vec3{a.x - s * b.x, a.y - s * b.y, a.z - s * b.z}
}

// s * a
MulSV :: #force_inline proc(s: f32, a: Vec3) -> Vec3 {
	return Vec3{s * a.x, s * a.y, s * a.z}
}

// https://en.wikipedia.org/wiki/Cross_product
Cross :: #force_inline proc(a, b: Vec3) -> Vec3 {
	c: Vec3
	c.x = a.y * b.z - a.z * b.y
	c.y = a.z * b.x - a.x * b.z
	c.z = a.x * b.y - a.y * b.x
	return c
}

// Linearly interpolate between two vectors.
Lerp :: #force_inline proc(a, b: Vec3, alpha: f32) -> Vec3 {
	assert(0.0 <= alpha && alpha <= 1.0)

	c := Vec3{(1.0 - alpha) * a.x + alpha * b.x, (1.0 - alpha) * a.y + alpha * b.y, (1.0 - alpha) * a.z + alpha * b.z}
	return c
}

// Blend two vectors: s * a + t * b
Blend2 :: #force_inline proc(s: f32, a: Vec3, t: f32, b: Vec3) -> Vec3 {
	d := Vec3{s * a.x + t * b.x, s * a.y + t * b.y, s * a.z + t * b.z}
	return d
}

// Component-wise absolute value.
Abs :: #force_inline proc(a: Vec3) -> Vec3 {
	return Vec3{AbsFloat(a.x), AbsFloat(a.y), AbsFloat(a.z)}
}

// Component-wise -1 or 1 (1 if zero).
Sign :: #force_inline proc(a: Vec3) -> Vec3 {
	return Vec3{a.x >= 0.0 ? 1.0 : -1.0, a.y >= 0.0 ? 1.0 : -1.0, a.z >= 0.0 ? 1.0 : -1.0}
}

// Component-wise minimum value.
Min :: #force_inline proc(a, b: Vec3) -> Vec3 {
	return Vec3{MinFloat(a.x, b.x), MinFloat(a.y, b.y), MinFloat(a.z, b.z)}
}

// Component-wise maximum value.
Max :: #force_inline proc(a, b: Vec3) -> Vec3 {
	return Vec3{MaxFloat(a.x, b.x), MaxFloat(a.y, b.y), MaxFloat(a.z, b.z)}
}

// Component-wise clamped value.
Clamp :: #force_inline proc(a, lower, upper: Vec3) -> Vec3 {
	b: Vec3
	b.x = ClampFloat(a.x, lower.x, upper.x)
	b.y = ClampFloat(a.y, lower.y, upper.y)
	b.z = ClampFloat(a.z, lower.z, upper.z)
	return b
}

// Create a safe scaling value for scaling collision. This allows
// negative scale, but keeps scale sufficiently far from zero.
SafeScale :: #force_inline proc(a: Vec3) -> Vec3 {
	absScale := Abs(a)
	minScale := Vec3{MIN_SCALE, MIN_SCALE, MIN_SCALE}
	safeScale := Mul(Sign(a), Max(absScale, minScale))
	return safeScale
}

// Does the supplied quaternion have unit length?
IsNormalizedQuat :: #force_inline proc(q: Quat) -> bool {
	qq := q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z + q.s * q.s
	return 1.0 - 20.0 * FLT_EPSILON < qq && qq < 1.0 + 20.0 * FLT_EPSILON
}

// Rotate a vector.
RotateVector :: #force_inline proc(q: Quat, v: Vec3) -> Vec3 {
	// v + 2 * cross(q.v, cross(q.v, v) + q.s * v)
	// B3_ASSERT( b3IsNormalizedQuat( q ) );
	t1 := Cross(q.v, v)
	t2 := MulAdd(t1, q.s, v)
	t3 := Cross(q.v, t2)
	return MulAdd(v, 2.0, t3)
}

// Inverse rotate a vector.
InvRotateVector :: #force_inline proc(q: Quat, v: Vec3) -> Vec3 {
	// v + 2 * cross(q.v, cross(q.v, v) - q.s * v)
	// B3_ASSERT( b3IsNormalizedQuat( q ) );
	t1 := Cross(q.v, v)
	t2 := MulSub(t1, q.s, v)
	t3 := Cross(q.v, t2)
	return MulAdd(v, 2.0, t3)
}

// Compute dot product of two quaternions. Useful for polarity tests.
DotQuat :: #force_inline proc(a, b: Quat) -> f32 {
	return a.v.x * b.v.x + a.v.y * b.v.y + a.v.z * b.v.z + a.s * b.s
}

// Multiply two quaternions.
MulQuat :: #force_inline proc(q1, q2: Quat) -> Quat {
	t1 := Cross(q1.v, q2.v)
	t2 := MulAdd(t1, q1.s, q2.v)
	t3 := MulAdd(t2, q2.s, q1.v)
	q := Quat{t3, q1.s * q2.s - Dot(q1.v, q2.v)}
	return q
}

// Compute a relative quaternion.
// inv(q1) * q2
InvMulQuat :: #force_inline proc(q1, q2: Quat) -> Quat {
	t1 := Cross(q2.v, q1.v)
	t2 := MulAdd(t1, q1.s, q2.v)
	t3 := MulSub(t2, q2.s, q1.v)
	q := Quat{t3, q1.s * q2.s + Dot(q1.v, q2.v)}
	return q
}

// Quaternion conjugate (cheap inverse).
Conjugate :: #force_inline proc(q: Quat) -> Quat {
	return Quat{{-q.v.x, -q.v.y, -q.v.z}, q.s}
}

// Component-wise quaternion negation.
NegateQuat :: #force_inline proc(q: Quat) -> Quat {
	return Quat{{-q.v.x, -q.v.y, -q.v.z}, -q.s}
}

// Normalize a quaternion.
NormalizeQuat :: #force_inline proc(q: Quat) -> Quat {
	lengthSq := DotQuat(q, q)
	if lengthSq > 1000.0 * FLT_MIN {
		s := 1.0 / intrinsics.sqrt(lengthSq)
		qn := Quat{{s * q.v.x, s * q.v.y, s * q.v.z}, s * q.s}
		return qn
	}

	return Quat_identity
}

// Make a quaternion that is equivalent to rotating around an axis by a specified angle.
MakeQuatFromAxisAngle :: #force_inline proc(axis: Vec3, radians: f32) -> Quat {
	assert(IsNormalized(axis))
	cs := ComputeCosSin(0.5 * radians)
	q := Quat{{cs.sine * axis.x, cs.sine * axis.y, cs.sine * axis.z}, cs.cosine}
	return q
}

// Get the axis and angle from a quaternion. Assumes the quaternion is normalized.
GetAxisAngle :: #force_inline proc(radians: ^f32, q: Quat) -> Vec3 {
	length := intrinsics.sqrt(q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z)
	radians^ = 2.0 * Atan2(length, q.s)
	if length > 0.0 {
		invLength := 1.0 / length
		axis := Vec3{invLength * q.v.x, invLength * q.v.y, invLength * q.v.z}
		return axis
	}

	return Vec3_zero
}

// Get the angle for a quaternion in radians
GetQuatAngle :: #force_inline proc(q: Quat) -> f32 {
	length := intrinsics.sqrt(q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z)
	return 2.0 * Atan2(length, q.s)
}

// Twist angle around the z-axis, used for twist limit and revolute angle limit
GetTwistAngle :: #force_inline proc(q: Quat) -> f32 {
	// Account for polarity to keep the twist angle in range.
	// This is simpler than asking the user to check polarity or unwinding.
	twist := q.s < 0.0 ? Atan2(-q.v.z, -q.s) : Atan2(q.v.z, q.s)
	twist *= 2.0
	assert(-PI <= twist && twist <= PI)
	return twist
}

// Swing angle used for cone limit
GetSwingAngle :: #force_inline proc(q: Quat) -> f32 {
	// Polarity should not matter because all terms are squared.
	x := intrinsics.sqrt(q.v.z * q.v.z + q.s * q.s)
	y := intrinsics.sqrt(q.v.x * q.v.x + q.v.y * q.v.y)
	swing := 2.0 * Atan2(y, x)
	assert(0.0 <= swing && swing <= PI)
	return swing
}

// Linearly interpolate and normalize between two quaternions
NLerp :: #force_inline proc(q1, q2: Quat, alpha: f32) -> Quat {
	q1 := q1
	VALIDATE(0.0 <= alpha && alpha <= 1.0)
	if DotQuat(q1, q2) < 0.0 {
		q1 = Quat{{-q1.v.x, -q1.v.y, -q1.v.z}, -q1.s}
	}

	q: Quat
	q.v = Lerp(q1.v, q2.v, alpha)
	q.s = (1.0 - alpha) * q1.s + alpha * q2.s

	return NormalizeQuat(q)
}

// Multiply two transforms. If the result is applied to a point p local to frame B,
// the transform would first convert p to a point local to frame A, then into a point
// in the world frame. This is useful if frame B is a child of frame A.
MulTransforms :: #force_inline proc(a, b: Transform) -> Transform {
	out: Transform
	out.p = Add(RotateVector(a.q, b.p), a.p)
	out.q = MulQuat(a.q, b.q)
	return out
}

// Creates a transform that converts a local point in frame B to a local point in frame A.
// This is useful for transforming points between the local spaces of two frames that are
// in world space.
InvMulTransforms :: #force_inline proc(a, b: Transform) -> Transform {
	out: Transform
	out.p = InvRotateVector(a.q, Sub(b.p, a.p))
	out.q = InvMulQuat(a.q, b.q)
	return out
}

// Get the inverse of a transform.
InvertTransform :: #force_inline proc(t: Transform) -> Transform {
	out: Transform
	out.p = InvRotateVector(t.q, Neg(t.p))
	out.q = Conjugate(t.q)
	return out
}

// Transform a point.
TransformPoint :: #force_inline proc(t: Transform, v: Vec3) -> Vec3 {
	rv := RotateVector(t.q, v)
	return Add(rv, t.p)
}

// Inverse transform a point.
InvTransformPoint :: #force_inline proc(t: Transform, v: Vec3) -> Vec3 {
	return InvRotateVector(t.q, Sub(v, t.p))
}

// World position boundary. These cross between the double precision world space at the public
// boundary and the float interior. One set of bodies serves both modes: the typedefs collapse
// the types in float mode and the explicit float casts become no-ops.

// Convert a vector to a world position.
ToPos :: #force_inline proc(v: Vec3) -> Pos {
	return Pos{v.x, v.y, v.z}
}

// Lossy conversion of a world position to a float vector.
ToVec3 :: #force_inline proc(p: Pos) -> Vec3 {
	return Vec3{f32(p.x), f32(p.y), f32(p.z)}
}

// Narrow a world coordinate to float, rounding toward negative infinity. Use with
// b3RoundUpFloat to build a conservative float box that always contains the double bounds,
// where plain rounding far from the origin could clip. nextafterf is an exact IEEE operation,
// so this is cross-platform deterministic. With large world mode off this is a plain conversion.
RoundDownFloat :: #force_inline proc(x: f64) -> f32 {
	// unified: single precision only: the BOX3D_DOUBLE_PRECISION nextafterf branch is not ported.
	return f32(x)
}

// Narrow a world coordinate to float, rounding toward positive infinity.
RoundUpFloat :: #force_inline proc(x: f64) -> f32 {
	// unified: single precision only: the BOX3D_DOUBLE_PRECISION nextafterf branch is not ported.
	return f32(x)
}

// a - b, demoted to float. The primary precision boundary operation.
SubPos :: #force_inline proc(a, b: Pos) -> Vec3 {
	return Vec3{f32(a.x - b.x), f32(a.y - b.y), f32(a.z - b.z)}
}

// p + d
OffsetPos :: #force_inline proc(p: Pos, d: Vec3) -> Pos {
	return Pos{p.x + d.x, p.y + d.y, p.z + d.z}
}

// World position interpolation for sweeps and sampling.
LerpPosition :: #force_inline proc(a, b: Pos, t: f32) -> Pos {
	return Pos{(1.0 - t) * a.x + t * b.x, (1.0 - t) * a.y + t * b.y, (1.0 - t) * a.z + t * b.z}
}

// Transform a local point to a world position. Rotation in float, translation in double.
TransformWorldPoint :: #force_inline proc(t: WorldTransform, p: Vec3) -> Pos {
	r := RotateVector(t.q, p)
	return Pos{t.p.x + r.x, t.p.y + r.y, t.p.z + r.z}
}

// Transform a world position to a local point. One double subtraction, then float.
InvTransformWorldPoint :: #force_inline proc(t: WorldTransform, p: Pos) -> Vec3 {
	d := Vec3{f32(p.x - t.p.x), f32(p.y - t.p.y), f32(p.z - t.p.z)}
	return InvRotateVector(t.q, d)
}

// Compose a world transform with a local transform.
MulWorldTransforms :: #force_inline proc(A: WorldTransform, B: Transform) -> WorldTransform {
	C: WorldTransform
	C.q = MulQuat(A.q, B.q)
	r := RotateVector(A.q, B.p)
	C.p = Pos{A.p.x + r.x, A.p.y + r.y, A.p.z + r.z}
	return C
}

// Relative transform of frame B in frame A. The narrow phase boundary.
InvMulWorldTransforms :: #force_inline proc(A, B: WorldTransform) -> Transform {
	C: Transform
	C.q = InvMulQuat(A.q, B.q)
	d := Vec3{f32(B.p.x - A.p.x), f32(B.p.y - A.p.y), f32(B.p.z - A.p.z)}
	C.p = InvRotateVector(A.q, d)
	return C
}

// Shift a world transform into the frame of a base position.
ToRelativeTransform :: #force_inline proc(t: WorldTransform, base: Pos) -> Transform {
	r: Transform
	r.q = t.q
	r.p = Vec3{f32(t.p.x - base.x), f32(t.p.y - base.y), f32(t.p.z - base.z)}
	return r
}

// Promote a float transform to a world transform. Lossless.
MakeWorldTransform :: #force_inline proc(t: Transform) -> WorldTransform {
	w: WorldTransform
	w.p = ToPos(t.p)
	w.q = t.q
	return w
}

// Translate a local AABB by a world position, rounding outward so the single precision box
// encloses the double precision box. Far from the origin a plain conversion could clip a
// shape out of its own box.
// In single precision mode the the rounding is a no-op.
OffsetAABB :: #force_inline proc(localBox: AABB, origin: Pos) -> AABB {
	out: AABB
	out.lowerBound.x = RoundDownFloat(f64(origin.x + localBox.lowerBound.x))
	out.lowerBound.y = RoundDownFloat(f64(origin.y + localBox.lowerBound.y))
	out.lowerBound.z = RoundDownFloat(f64(origin.z + localBox.lowerBound.z))
	out.upperBound.x = RoundUpFloat(f64(origin.x + localBox.upperBound.x))
	out.upperBound.y = RoundUpFloat(f64(origin.y + localBox.upperBound.y))
	out.upperBound.z = RoundUpFloat(f64(origin.z + localBox.upperBound.z))
	return out
}

// Compute the determinant of a 3-by-3 matrix.
Det :: #force_inline proc(m: Matrix3) -> f32 {
	return Dot(m.cx, Cross(m.cy, m.cz))
}

// Multiply a matrix times a column vector.
MulMV :: #force_inline proc(m: Matrix3, a: Vec3) -> Vec3 {
	b := Vec3 {
		m.cx.x * a.x + m.cy.x * a.y + m.cz.x * a.z,
		m.cx.y * a.x + m.cy.y * a.y + m.cz.y * a.z,
		m.cx.z * a.x + m.cy.z * a.y + m.cz.z * a.z,
	}
	return b
}

// Negate a matrix.
NegateMat3 :: #force_inline proc(a: Matrix3) -> Matrix3 {
	return Matrix3{{-a.cx.x, -a.cx.y, -a.cx.z}, {-a.cy.x, -a.cy.y, -a.cy.z}, {-a.cz.x, -a.cz.y, -a.cz.z}}
}

// Matrix addition.
// @return a + b
AddMM :: #force_inline proc(a, b: Matrix3) -> Matrix3 {
	return Matrix3 {
		{a.cx.x + b.cx.x, a.cx.y + b.cx.y, a.cx.z + b.cx.z},
		{a.cy.x + b.cy.x, a.cy.y + b.cy.y, a.cy.z + b.cy.z},
		{a.cz.x + b.cz.x, a.cz.y + b.cz.y, a.cz.z + b.cz.z},
	}
}

// Matrix subtraction.
// @return a - b
SubMM :: #force_inline proc(a, b: Matrix3) -> Matrix3 {
	return Matrix3 {
		{a.cx.x - b.cx.x, a.cx.y - b.cx.y, a.cx.z - b.cx.z},
		{a.cy.x - b.cy.x, a.cy.y - b.cy.y, a.cy.z - b.cy.z},
		{a.cz.x - b.cz.x, a.cz.y - b.cz.y, a.cz.z - b.cz.z},
	}
}

// Multiply a matrix by a scalar, component-wise.
MulSM :: #force_inline proc(s: f32, a: Matrix3) -> Matrix3 {
	return Matrix3 {
		{s * a.cx.x, s * a.cx.y, s * a.cx.z},
		{s * a.cy.x, s * a.cy.y, s * a.cy.z},
		{s * a.cz.x, s * a.cz.y, s * a.cz.z},
	}
}

// Matrix multiplication.
// @return a * b
MulMM :: #force_inline proc(a, b: Matrix3) -> Matrix3 {
	out: Matrix3
	out.cx = MulMV(a, b.cx)
	out.cy = MulMV(a, b.cy)
	out.cz = MulMV(a, b.cz)
	return out
}

// Matrix transpose.
Transpose :: #force_inline proc(m: Matrix3) -> Matrix3 {
	out: Matrix3
	out.cx = Vec3{m.cx.x, m.cy.x, m.cz.x}
	out.cy = Vec3{m.cx.y, m.cy.y, m.cz.y}
	out.cz = Vec3{m.cx.z, m.cy.z, m.cz.z}

	return out
}

// General matrix inverse.
InvertMatrix :: #force_inline proc(m: Matrix3) -> Matrix3 {
	det := Det(m)
	if AbsFloat(det) > 1000.0 * FLT_MIN {
		invDet := 1.0 / det
		out: Matrix3
		out.cx = MulSV(invDet, Cross(m.cy, m.cz))
		out.cy = MulSV(invDet, Cross(m.cz, m.cx))
		out.cz = MulSV(invDet, Cross(m.cx, m.cy))

		return Transpose(out)
	}

	return Mat3_zero
}

// Solve a matrix equation.
// @return inv(m) * a
Solve3 :: #force_inline proc(m: Matrix3, a: Vec3) -> Vec3 {
	det := Det(m)
	if AbsFloat(det) > 1000.0 * FLT_MIN {
		invDet := 1.0 / det
		s: Matrix3
		s.cx = Cross(m.cy, m.cz)
		s.cy = Cross(m.cz, m.cx)
		s.cz = Cross(m.cx, m.cy)

		b := Vec3{invDet * Dot(s.cx, a), invDet * Dot(s.cy, a), invDet * Dot(s.cz, a)}

		return b
	}

	return Vec3_zero
}

// Invert a matrix.
InvertT :: #force_inline proc(m: Matrix3) -> Matrix3 {
	det := Det(m)
	if AbsFloat(det) > 1000.0 * FLT_MIN {
		invDet := 1.0 / det
		out: Matrix3
		out.cx = MulSV(invDet, Cross(m.cy, m.cz))
		out.cy = MulSV(invDet, Cross(m.cz, m.cx))
		out.cz = MulSV(invDet, Cross(m.cx, m.cy))
		return out
	}

	return Mat3_zero
}

// Get the component-wise absolute value of a matrix.
AbsMatrix3 :: #force_inline proc(m: Matrix3) -> Matrix3 {
	out: Matrix3
	out.cx = Abs(m.cx)
	out.cy = Abs(m.cy)
	out.cz = Abs(m.cz)

	return out
}

// Make a matrix from a quaternion. This is useful if you need to
// rotate many vectors.
// The force inline improves the performance of b3ShapeDistance.
MakeMatrixFromQuat :: #force_inline proc(q: Quat) -> Matrix3 {
	xx := q.v.x * q.v.x
	yy := q.v.y * q.v.y
	zz := q.v.z * q.v.z
	xy := q.v.x * q.v.y
	xz := q.v.x * q.v.z
	xw := q.v.x * q.s
	yz := q.v.y * q.v.z
	yw := q.v.y * q.s
	zw := q.v.z * q.s

	return Matrix3 {
		{1.0 - 2.0 * (yy + zz), 2.0 * (xy + zw), 2.0 * (xz - yw)},
		{2.0 * (xy - zw), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + xw)},
		{2.0 * (xz + yw), 2.0 * (yz - xw), 1.0 - 2.0 * (xx + yy)},
	}
}

// Get the AABB of a point cloud.
MakeAABB :: #force_inline proc(points: [^]Vec3, count: i32, radius: f32) -> AABB {
	assert(count > 0)
	a := AABB{points[0], points[0]}
	for i: i32 = 1; i < count; i += 1 {
		a.lowerBound = Min(a.lowerBound, points[i])
		a.upperBound = Max(a.upperBound, points[i])
	}

	r := Vec3{radius, radius, radius}
	a.lowerBound = Sub(a.lowerBound, r)
	a.upperBound = Add(a.upperBound, r)

	return a
}

// Does a fully contain b?
AABB_Contains :: #force_inline proc(a, b: AABB) -> bool {
	if a.lowerBound.x > b.lowerBound.x || b.upperBound.x > a.upperBound.x {
		return false
	}
	if a.lowerBound.y > b.lowerBound.y || b.upperBound.y > a.upperBound.y {
		return false
	}
	if a.lowerBound.z > b.lowerBound.z || b.upperBound.z > a.upperBound.z {
		return false
	}

	return true
}

// Get the surface area of an axis-aligned bounding box.
AABB_Area :: #force_inline proc(a: AABB) -> f32 {
	delta := Sub(a.upperBound, a.lowerBound)
	return 2.0 * (delta.x * delta.y + delta.y * delta.z + delta.z * delta.x)
}

// Get the center of an axis-aligned bounding box.
AABB_Center :: #force_inline proc(a: AABB) -> Vec3 {
	return MulSV(0.5, Add(a.upperBound, a.lowerBound))
}

// Get the extents (half-widths) of an axis-aligned bounding box.
AABB_Extents :: #force_inline proc(a: AABB) -> Vec3 {
	return MulSV(0.5, Sub(a.upperBound, a.lowerBound))
}

// Get the union of two axis-aligned bounding boxes.
AABB_Union :: #force_inline proc(a, b: AABB) -> AABB {
	out: AABB
	out.lowerBound = Min(a.lowerBound, b.lowerBound)
	out.upperBound = Max(a.upperBound, b.upperBound)
	return out
}

// Add uniform padding to an axis-aligned bounding box.
AABB_Inflate :: #force_inline proc(a: AABB, extension: f32) -> AABB {
	radius := Vec3{extension, extension, extension}

	out: AABB
	out.lowerBound = Sub(a.lowerBound, radius)
	out.upperBound = Add(a.upperBound, radius)
	return out
}

// Do two axis-aligned boxes overlap?
AABB_Overlaps :: #force_inline proc(a, b: AABB) -> bool {
	// No intersection if separated along one axis
	if a.upperBound.x < b.lowerBound.x || a.lowerBound.x > b.upperBound.x {
		return false
	}
	if a.upperBound.y < b.lowerBound.y || a.lowerBound.y > b.upperBound.y {
		return false
	}
	if a.upperBound.z < b.lowerBound.z || a.lowerBound.z > b.upperBound.z {
		return false
	}

	// Overlapping on all axis means bounds are intersecting
	return true
}

// Transform an axis-aligned bounding box. This can create a larger box
// than if you recomputed the AABB of the original shape with the transform
// applied.
AABB_Transform :: #force_inline proc(transform: Transform, a: AABB) -> AABB {
	center := TransformPoint(transform, AABB_Center(a))
	m := MakeMatrixFromQuat(transform.q)
	extent := MulMV(AbsMatrix3(m), AABB_Extents(a))
	out := AABB{Sub(center, extent), Add(center, extent)}
	return out
}

// Get the closest point on an axis-aligned bounding box.
ClosestPointToAABB :: #force_inline proc(point: Vec3, a: AABB) -> Vec3 {
	return Clamp(point, a.lowerBound, a.upperBound)
}

// One-dimensional mass-spring-damper simulation. Returns the new velocity given the position and
// time step. You can then compute the new position using:
// position += timeStep * newVelocity
// This drives towards a zero position. By using implicit integration we get a stable solution that
// doesn't require transcendental functions.
SpringDamper :: #force_inline proc(hertz, dampingRatio, position, velocity, timeStep: f32) -> f32 {
	omega := 2.0 * PI * hertz
	omegaH := omega * timeStep
	return (velocity - omega * omegaH * position) / (1.0 + 2.0 * dampingRatio * omegaH + omegaH * omegaH)
}

// The closest points between to segments or infinite lines.
SegmentDistanceResult :: struct {
	point1:    Vec3,
	fraction1: f32,
	point2:    Vec3,
	fraction2: f32,
}

// unified: not ported: the C++ math operator overloads (C++ only).
