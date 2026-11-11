// Ported from box3d shared/utils.c and shared/utils.h at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import "base:intrinsics"
import "core:os"

import b3 "../../box3d"

RAND_LIMIT :: 32767
RAND_SEED :: 12345

// Global seed for simple random number generator.
g_randomSeed: u32 = RAND_SEED

GetNumberOfCores :: proc() -> i32 {
	// unified: os.get_processor_core_count replaces the per-platform system calls
	return i32(os.get_processor_core_count())
	// unified end
}

// Simple random number generator. Using this instead of rand() for cross platform determinism.
RandomInt :: proc() -> i32 {
	// XorShift32 algorithm
	x := g_randomSeed
	x ~= x << 13
	x ~= x >> 17
	x ~= x << 5
	g_randomSeed = x

	// Map the 32-bit value to the range 0 to RAND_LIMIT
	return i32(x % (RAND_LIMIT + 1))
}

// Random integer in range [lo, hi]
RandomIntRange :: proc(lo, hi: i32) -> f32 {
	return f32(lo + RandomInt() % (hi - lo + 1))
}

// Random number in range [-1,1]
RandomFloat :: proc() -> f32 {
	r := f32(RandomInt() & (RAND_LIMIT))
	r /= RAND_LIMIT
	r = 2.0 * r - 1.0
	return r
}

// Random floating point number in range [lo, hi]
RandomFloatRange :: proc(lo, hi: f32) -> f32 {
	r := f32(RandomInt() & (RAND_LIMIT))
	r /= RAND_LIMIT
	r = (hi - lo) * r + lo
	return r
}

// Random vector with coordinates in range [lo, hi]
RandomVec3 :: proc(lo, hi: b3.Vec3) -> b3.Vec3 {
	v: b3.Vec3
	v.x = RandomFloatRange(lo.x, hi.x)
	v.y = RandomFloatRange(lo.y, hi.y)
	v.z = RandomFloatRange(lo.z, hi.z)
	return v
}

// Random world position with coordinates in range [lo, hi]
RandomPos :: proc(lo, hi: b3.Vec3) -> b3.Pos {
	v: b3.Pos
	v.x = RandomFloatRange(lo.x, hi.x)
	v.y = RandomFloatRange(lo.y, hi.y)
	v.z = RandomFloatRange(lo.z, hi.z)
	return v
}

RandomVec3Uniform :: proc(lo, hi: f32) -> b3.Vec3 {
	v: b3.Vec3
	v.x = RandomFloatRange(lo, hi)
	v.y = RandomFloatRange(lo, hi)
	v.z = RandomFloatRange(lo, hi)
	return v
}

RandomUnitVector :: proc() -> b3.Vec3 {
	// Generate uniformly distributed random quaternion using Shoemake's method
	// Reference: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III, 1992

	u1 := RandomFloatRange(0.0, 1.0)
	u2 := RandomFloatRange(0.0, 2.0 * b3.PI)
	u3 := RandomFloatRange(0.0, 2.0 * b3.PI)

	sqrt1MinusU1 := intrinsics.sqrt(1.0 - u1)
	sqrtU1 := intrinsics.sqrt(u1)

	cs2 := b3.ComputeCosSin(u2)
	cs3 := b3.ComputeCosSin(u3)

	v: b3.Vec3
	v.x = sqrt1MinusU1 * cs2.sine
	v.y = sqrt1MinusU1 * cs2.cosine
	v.z = sqrtU1 * cs3.sine

	return v
}

RandomQuat :: proc() -> b3.Quat {
	// Generate uniformly distributed random quaternion using Shoemake's method
	// Reference: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III, 1992

	u1 := RandomFloatRange(0.0, 1.0)
	u2 := RandomFloatRange(0.0, 2.0 * b3.PI)
	u3 := RandomFloatRange(0.0, 2.0 * b3.PI)

	sqrt1MinusU1 := intrinsics.sqrt(1.0 - u1)
	sqrtU1 := intrinsics.sqrt(u1)

	cs2 := b3.ComputeCosSin(u2)
	cs3 := b3.ComputeCosSin(u3)

	q: b3.Quat
	q.v.x = sqrt1MinusU1 * cs2.sine
	q.v.y = sqrt1MinusU1 * cs2.cosine
	q.v.z = sqrtU1 * cs3.sine
	q.s = sqrtU1 * cs3.cosine

	return q
}
