// Ported from box3d src/simd.c, src/simd.h and src/simd_sse2.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Dirk Gregorius contributed portions of this code

package box3d

import "base:intrinsics"
import "core:simd"

// unified: one 4-wide implementation on #simd[4]f32 with SSE2's semantics (simd_sse2.h): comparisons give
// all-ones lane masks, logic is bitwise, min/max return the second operand unless the first compares strictly
// less/greater (so NaN and -0/+0 give the second operand). simd_neon.h and simd_scalar.h are not ported.
// unified: not ported: the 8-wide AVX2 path; every platform runs width 4 so x64 and arm64 compute the same results

// ---- simd_sse2.h ----

// wide float holds 4 numbers
FloatW4 :: #simd[4]f32

@(private = "file")
UIntW4 :: #simd[4]u32

@(private = "file")
IntW4 :: #simd[4]i32

@(private = "file")
SIGN_MASK_W4 :: UIntW4{0x80000000, 0x80000000, 0x80000000, 0x80000000}

// _mm_movemask_ps: the sign bit of each lane
@(private = "file")
MoveMask :: #force_inline proc "contextless" (a: FloatW4) -> i32 {
	return i32(transmute(u8)simd.extract_msbs(transmute(UIntW4)a))
}

// _mm_min_ps: a < b ? a : b per lane
@(private = "file")
MinPs :: #force_inline proc "contextless" (a, b: FloatW4) -> FloatW4 {
	return simd.select(simd.lanes_lt(a, b), a, b)
}

// _mm_max_ps: a > b ? a : b per lane
@(private = "file")
MaxPs :: #force_inline proc "contextless" (a, b: FloatW4) -> FloatW4 {
	return simd.select(simd.lanes_gt(a, b), a, b)
}

ZeroW4 :: #force_inline proc "contextless" () -> FloatW4 {
	return FloatW4{}
}

SplatW4 :: #force_inline proc "contextless" (scalar: f32) -> FloatW4 {
	return FloatW4{scalar, scalar, scalar, scalar}
}

SetW4 :: #force_inline proc "contextless" (a, b, c, d: f32) -> FloatW4 {
	return FloatW4{a, b, c, d}
}

LoadW4 :: #force_inline proc "contextless" (data: [^]f32) -> FloatW4 {
	return intrinsics.unaligned_load((^FloatW4)(data))
}

StoreW4 :: #force_inline proc "contextless" (data: [^]f32, a: FloatW4) {
	intrinsics.unaligned_store((^FloatW4)(data), a)
}

NegW4 :: #force_inline proc "contextless" (a: FloatW4) -> FloatW4 {
	// Create a mask with the sign bit set for each element
	// XOR the input with the mask to negate each element
	return transmute(FloatW4)(transmute(UIntW4)a ~ SIGN_MASK_W4)
}

AbsW4 :: #force_inline proc "contextless" (a: FloatW4) -> FloatW4 {
	return transmute(FloatW4)(transmute(UIntW4)a &~ SIGN_MASK_W4)
}

AddW4 :: #force_inline proc "contextless" (a, b: FloatW4) -> FloatW4 {
	return a + b
}

SubW4 :: #force_inline proc "contextless" (a, b: FloatW4) -> FloatW4 {
	return a - b
}

MulW4 :: #force_inline proc "contextless" (a, b: FloatW4) -> FloatW4 {
	return a * b
}

DivW4 :: #force_inline proc "contextless" (a, b: FloatW4) -> FloatW4 {
	return a / b
}

SqrtW4 :: #force_inline proc "contextless" (a: FloatW4) -> FloatW4 {
	return simd.sqrt(a)
}

// a + b * c
MulAddW4 :: #force_inline proc "contextless" (a, b, c: FloatW4) -> FloatW4 {
	return a + (b * c)
}

MinW4 :: #force_inline proc "contextless" (a, b: FloatW4) -> FloatW4 {
	return MinPs(a, b)
}

MaxW4 :: #force_inline proc "contextless" (a, b: FloatW4) -> FloatW4 {
	return MaxPs(a, b)
}

AndW4 :: #force_inline proc "contextless" (a, b: FloatW4) -> FloatW4 {
	return transmute(FloatW4)(transmute(UIntW4)a & transmute(UIntW4)b)
}

OrW4 :: #force_inline proc "contextless" (a, b: FloatW4) -> FloatW4 {
	return transmute(FloatW4)(transmute(UIntW4)a | transmute(UIntW4)b)
}

// a & ~b
AndNotW4 :: #force_inline proc "contextless" (a, b: FloatW4) -> FloatW4 {
	return transmute(FloatW4)(transmute(UIntW4)a &~ transmute(UIntW4)b)
}

// This is used to optimize selection of contact softness.
SoftMaskW4 :: #force_inline proc "contextless" (indexA: [^]i32, indexB: [^]i32) -> FloatW4 {
	zero := IntW4{}
	a := simd.lanes_eq(intrinsics.unaligned_load((^IntW4)(indexA)), zero)
	b := simd.lanes_eq(intrinsics.unaligned_load((^IntW4)(indexB)), zero)
	return transmute(FloatW4)(a | b)
}

GreaterThanW4 :: #force_inline proc "contextless" (a, b: FloatW4) -> FloatW4 {
	return transmute(FloatW4)simd.lanes_gt(a, b)
}

LessThanW4 :: #force_inline proc "contextless" (a, b: FloatW4) -> FloatW4 {
	return transmute(FloatW4)simd.lanes_lt(a, b)
}

EqualsW4 :: #force_inline proc "contextless" (a, b: FloatW4) -> FloatW4 {
	return transmute(FloatW4)simd.lanes_eq(a, b)
}

AllZeroW4 :: #force_inline proc "contextless" (a: FloatW4) -> bool {
	// Compare each element with zero
	zero := FloatW4{}
	cmp := transmute(FloatW4)simd.lanes_eq(a, zero)

	// Create a mask from the comparison results
	mask := MoveMask(cmp)

	// If all elements are zero, the mask will be 0xF (1111 in binary)
	return mask == 0xF
}

AnyTrueW4 :: #force_inline proc "contextless" (mask: FloatW4) -> bool {
	return MoveMask(mask) != 0
}

MoveMaskW4 :: #force_inline proc "contextless" (mask: FloatW4) -> i32 {
	return MoveMask(mask)
}

// component-wise returns mask ? b : a
BlendW4 :: #force_inline proc "contextless" (a, b, mask: FloatW4) -> FloatW4 {
	m := transmute(UIntW4)mask
	return transmute(FloatW4)((m & transmute(UIntW4)b) | (transmute(UIntW4)a &~ m))
}

// Replace the low bitCount mantissa bits of each lane with baseIndex + lane. The value must be
// positive so the embedded index sorts with the value, and ties fall to the lower index.
EmbedIndexW4 :: #force_inline proc "contextless" (value: FloatW4, baseIndex: i32, bitCount: i32) -> FloatW4 {
	mask := (i32(1) << u32(bitCount)) - 1
	index := IntW4{baseIndex, baseIndex, baseIndex, baseIndex} + IntW4{0, 1, 2, 3}
	clearLow := transmute(UIntW4)IntW4{~mask, ~mask, ~mask, ~mask}
	return transmute(FloatW4)((transmute(UIntW4)value & clearLow) | transmute(UIntW4)index)
}

// Recovers the index embedded by b3EmbedIndexW4 from the lane holding the minimum.
MinIndexW4 :: #force_inline proc "contextless" (a: FloatW4, bitCount: i32) -> i32 {
	a := a
	a = MinPs(a, simd.swizzle(a, 1, 0, 3, 2))
	a = MinPs(a, simd.swizzle(a, 2, 3, 0, 1))
	return transmute(i32)simd.extract(a, 0) & ((i32(1) << u32(bitCount)) - 1)
}

TransposeW4 :: #force_inline proc "contextless" (r0, r1, r2, r3: FloatW4, c0, c1, c2, c3: ^FloatW4) {
	t0 := simd.shuffle(r0, r2, 0, 4, 1, 5)
	t1 := simd.shuffle(r1, r3, 0, 4, 1, 5)
	t2 := simd.shuffle(r0, r2, 2, 6, 3, 7)
	t3 := simd.shuffle(r1, r3, 2, 6, 3, 7)
	c0^ = simd.shuffle(t0, t1, 0, 4, 1, 5)
	c1^ = simd.shuffle(t0, t1, 2, 6, 3, 7)
	c2^ = simd.shuffle(t2, t3, 0, 4, 1, 5)
	c3^ = simd.shuffle(t2, t3, 2, 6, 3, 7)
}

// The following are SIMD definitions for operations where a wide register is
// used to store a single 3D vector, rather than SOA data for proper "wide" SIMD.
// For such uses, there's no benefit to using AVX2 8-wide registers; we're only using 3 lanes.
// For that reason, we unconditionally use SSE, even if AVX2 is available.

// wide float holds 4 numbers
V32 :: #simd[4]f32

zeroV :: V32{0.0, 0.0, 0.0, 0.0}
halfV :: V32{0.5, 0.5, 0.5, 0.5}
oneV :: V32{1.0, 1.0, 1.0, 1.0}

AddV :: #force_inline proc "contextless" (a, b: V32) -> V32 {
	return a + b
}

SubV :: #force_inline proc "contextless" (a, b: V32) -> V32 {
	return a - b
}

MulV :: #force_inline proc "contextless" (a, b: V32) -> V32 {
	return a * b
}

DivV :: #force_inline proc "contextless" (a, b: V32) -> V32 {
	return a / b
}

NegV :: #force_inline proc "contextless" (a: V32) -> V32 {
	return V32{} - a
}

LoadV :: #force_inline proc "contextless" (src: [^]f32) -> V32 {
	// Loads exactly 12 bytes: 8 via movsd, 4 via movss.
	// Result lane 3 is implicitly zero from the partial loads.
	return V32{src[0], src[1], src[2], 0.0} // { src[0], src[1], src[2], 0.0f }
}

ZeroV :: #force_inline proc "contextless" () -> V32 {
	return V32{}
}

GetXV :: #force_inline proc "contextless" (a: V32) -> f32 {
	return simd.extract(a, 0)
}

GetYV :: #force_inline proc "contextless" (a: V32) -> f32 {
	return simd.extract(a, 1)
}

GetZV :: #force_inline proc "contextless" (a: V32) -> f32 {
	return simd.extract(a, 2)
}

GetV :: #force_inline proc "contextless" (a: V32, index: i32) -> f32 {
	b := simd.to_array(a)
	return b[index]
}

SplatV :: #force_inline proc "contextless" (x: f32) -> V32 {
	return V32{x, x, x, x}
}

AbsV :: #force_inline proc "contextless" (a: V32) -> V32 {
	// Abs( V ) = Max( -V, V )
	zero := V32{}
	return MaxPs(zero - a, a)
}

MinV :: #force_inline proc "contextless" (a, b: V32) -> V32 {
	return MinPs(a, b)
}

MaxV :: #force_inline proc "contextless" (a, b: V32) -> V32 {
	return MaxPs(a, b)
}

CrossV :: #force_inline proc "contextless" (a, b: V32) -> V32 {
	yzX1 := simd.swizzle(a, 1, 2, 0, 3)
	zxY1 := simd.swizzle(a, 2, 0, 1, 3)
	yzX2 := simd.swizzle(b, 1, 2, 0, 3)
	zxY2 := simd.swizzle(b, 2, 0, 1, 3)

	return (yzX1 * zxY2) - (zxY1 * yzX2)
}

ModifiedCrossV :: #force_inline proc "contextless" (a, b: V32) -> V32 {
	yzX1 := simd.swizzle(a, 1, 2, 0, 3)
	zxY1 := simd.swizzle(a, 2, 0, 1, 3)
	yzX2 := simd.swizzle(b, 1, 2, 0, 3)
	zxY2 := simd.swizzle(b, 2, 0, 1, 3)

	return (yzX1 * zxY2) + (zxY1 * yzX2)
}

AnyLess3V :: #force_inline proc "contextless" (a, b: V32) -> bool {
	v := transmute(V32)simd.lanes_lt(a, b)
	return (MoveMask(v) & 0x07) != 0
}

AnyLessEq3V :: #force_inline proc "contextless" (a, b: V32) -> bool {
	v := transmute(V32)simd.lanes_le(a, b)
	return (MoveMask(v) & 0x07) != 0
}

AnyGreater3V :: #force_inline proc "contextless" (a, b: V32) -> bool {
	v := transmute(V32)simd.lanes_gt(a, b)
	return (MoveMask(v) & 0x07) != 0
}

AllLessEq3V :: #force_inline proc "contextless" (a, b: V32) -> bool {
	v := transmute(V32)simd.lanes_le(a, b)
	return (MoveMask(v) & 0x07) == 0x07
}

AABBV :: struct {
	lower: FloatW4,
	upper: FloatW4,
}

LoadAABBV :: #force_inline proc "contextless" (aabb: ^AABB) -> AABBV {
	base := ([^]f32)(&aabb.lowerBound.x)
	// Same offset as Neon to avoid UB.
	v1 := LoadW4(base[2:])
	result: AABBV
	result.lower = LoadW4(base)
	result.upper = simd.swizzle(v1, 1, 2, 3, 3)
	return result
}

OverlapAABBV :: #force_inline proc "contextless" (a, b: AABBV) -> bool {
	test := simd.lanes_le(a.lower, b.upper) & simd.lanes_le(b.lower, a.upper)
	return (MoveMask(transmute(FloatW4)test) & 0x7) == 0x7
}

OverlapNode :: #force_inline proc "contextless" (av: AABBV, node: ^TreeNode) -> bool {
	return OverlapAABBV(av, LoadAABBV(&node.aabb))
}

OverlapV :: #force_inline proc "contextless" (a, b: ^AABB) -> bool {
	return OverlapAABBV(LoadAABBV(a), LoadAABBV(b))
}

UnionAABBV :: #force_inline proc "contextless" (a, b: AABBV) -> AABBV {
	result: AABBV
	result.lower = MinPs(a.lower, b.lower)
	result.upper = MaxPs(a.upper, b.upper)
	return result
}

UnionPairV :: #force_inline proc "contextless" (pair: [^]TreeNode) -> AABBV {
	return UnionAABBV(LoadAABBV(&pair[0].aabb), LoadAABBV(&pair[1].aabb))
}

// Conditionally store an AABB. Avoids a branch.
StoreAABBV :: #force_inline proc "contextless" (aabb: ^AABB, value: AABBV, condition: bool) {
	lane3 := UIntW4{0, 0, 0, 0xFFFFFFFF}
	uxSplat := simd.swizzle(value.upper, 0, 0, 0, 0)
	raw0 := transmute(FloatW4)((transmute(UIntW4)value.lower &~ lane3) | (lane3 & transmute(UIntW4)uxSplat))
	raw1 := simd.shuffle(raw0, value.upper, 2, 3, 5, 6)

	base := ([^]f32)(&aabb.lowerBound.x)
	m: u32 = condition ? 0xFFFFFFFF : 0
	mask := UIntW4{m, m, m, m}
	old0 := transmute(UIntW4)LoadW4(base)
	old1 := transmute(UIntW4)LoadW4(base[2:])
	StoreW4(base, transmute(FloatW4)((mask & transmute(UIntW4)raw0) | (old0 &~ mask)))
	StoreW4(base[2:], transmute(FloatW4)((mask & transmute(UIntW4)raw1) | (old1 &~ mask)))
}

// ---- simd.h ----

// Wide vec3
Vec3W4 :: struct {
	X, Y, Z: FloatW4,
}

LoadVW4 :: #force_inline proc "contextless" (x, y, z: [^]f32) -> Vec3W4 {
	return Vec3W4{LoadW4(x), LoadW4(y), LoadW4(z)}
}

StoreVW4 :: #force_inline proc "contextless" (x, y, z: [^]f32, v: Vec3W4) {
	StoreW4(x, v.X)
	StoreW4(y, v.Y)
	StoreW4(z, v.Z)
}

SplatVW4 :: #force_inline proc "contextless" (v: Vec3) -> Vec3W4 {
	return Vec3W4{SplatW4(v.x), SplatW4(v.y), SplatW4(v.z)}
}

NegVW4 :: #force_inline proc "contextless" (a: Vec3W4) -> Vec3W4 {
	return Vec3W4{NegW4(a.X), NegW4(a.Y), NegW4(a.Z)}
}

// s * a
MulSVW4 :: #force_inline proc "contextless" (s: FloatW4, a: Vec3W4) -> Vec3W4 {
	return Vec3W4{MulW4(s, a.X), MulW4(s, a.Y), MulW4(s, a.Z)}
}

// a - s * b
MulSubSVW4 :: #force_inline proc "contextless" (a: Vec3W4, s: FloatW4, b: Vec3W4) -> Vec3W4 {
	return Vec3W4{SubW4(a.X, MulW4(s, b.X)), SubW4(a.Y, MulW4(s, b.Y)), SubW4(a.Z, MulW4(s, b.Z))}
}

// a + s * b
MulAddSVW4 :: #force_inline proc "contextless" (a: Vec3W4, s: FloatW4, b: Vec3W4) -> Vec3W4 {
	return Vec3W4{AddW4(a.X, MulW4(s, b.X)), AddW4(a.Y, MulW4(s, b.Y)), AddW4(a.Z, MulW4(s, b.Z))}
}

// a - b
SubVW4 :: #force_inline proc "contextless" (a, b: Vec3W4) -> Vec3W4 {
	return Vec3W4{SubW4(a.X, b.X), SubW4(a.Y, b.Y), SubW4(a.Z, b.Z)}
}

// a + b
AddVW4 :: #force_inline proc "contextless" (a, b: Vec3W4) -> Vec3W4 {
	return Vec3W4{AddW4(a.X, b.X), AddW4(a.Y, b.Y), AddW4(a.Z, b.Z)}
}

DotW4 :: #force_inline proc "contextless" (a, b: Vec3W4) -> FloatW4 {
	return AddW4(AddW4(MulW4(a.X, b.X), MulW4(a.Y, b.Y)), MulW4(a.Z, b.Z))
}

CrossW4 :: #force_inline proc "contextless" (a, b: Vec3W4) -> Vec3W4 {
	c: Vec3W4
	c.X = SubW4(MulW4(a.Y, b.Z), MulW4(a.Z, b.Y))
	c.Y = SubW4(MulW4(a.Z, b.X), MulW4(a.X, b.Z))
	c.Z = SubW4(MulW4(a.X, b.Y), MulW4(a.Y, b.X))
	return c
}

TestBoundsOverlap :: proc "contextless" (nodeMin1, nodeMax1, nodeMin2, nodeMax2: V32) -> bool {
	separation := MaxV(SubV(nodeMin2, nodeMax1), SubV(nodeMin1, nodeMax2))
	return AllLessEq3V(separation, zeroV)
}

// Test a ray for edge separation with an AABB (Gino, p80).
TestBoundsRayOverlap :: proc "contextless" (nodeMin, nodeMax, rayStart, rayDelta: V32) -> bool {
	rayStart := rayStart

	// Setup node
	nodeCenter := MulV(halfV, AddV(nodeMin, nodeMax))
	nodeExtent := SubV(nodeMax, nodeCenter)

	// Setup ray
	rayStart = SubV(rayStart, nodeCenter)

	// SAT: Edge separation
	edgeSeparation := SubV(AbsV(CrossV(rayDelta, rayStart)), ModifiedCrossV(AbsV(rayDelta), nodeExtent))
	return AllLessEq3V(edgeSeparation, zeroV)
}

UnionV :: #force_inline proc "contextless" (a, b: AABB) -> AABB {
	a, b := a, b
	result := AABB{}
	StoreAABBV(&result, UnionAABBV(LoadAABBV(&a), LoadAABBV(&b)), true)
	return result
}

// ---- simd.c ----

@(private = "file")
simdWidth: AtomicInt

// unified: not ported: b3CpuId, b3GetExtendedControlRegister and b3HasAVX2 (the AVX2 runtime dispatch)

@(private = "file")
DetectSIMDWidth :: proc "contextless" () -> i32 {
	return 4
}

GetSIMDWidth :: proc "contextless" () -> i32 {
	width := AtomicLoadInt(&simdWidth)
	if width == 0 {
		width = DetectSIMDWidth()
		AtomicStoreInt(&simdWidth, width)
	}

	return width
}

IsAVX2Available :: proc "contextless" () -> bool {
	return DetectSIMDWidth() == 8
}

SetSIMDWidth :: proc(width: i32) {
	width := width
	assert(width == 0 || width == 4 || width == 8)
	if width == 8 && DetectSIMDWidth() != 8 {
		width = 4
	}

	AtomicStoreInt(&simdWidth, width)
}

@(private = "file")
TRANSPOSE3 :: #force_inline proc "contextless" (C1, C2, C3: ^V32) {
	T1 := simd.shuffle(C1^, C2^, 0, 4, 1, 5)
	T2 := simd.shuffle(C1^, C2^, 2, 6, 3, 7)
	C1^ = simd.shuffle(T1, C3^, 0, 1, 4, 4)
	C2^ = simd.shuffle(T1, C3^, 2, 3, 5, 5)
	C3^ = simd.shuffle(T2, C3^, 0, 1, 6, 6)
}

@(private = "file")
SplatXV :: #force_inline proc "contextless" (v: V32) -> V32 {
	return simd.swizzle(v, 0, 0, 0, 0)
}

@(private = "file")
SplatYV :: #force_inline proc "contextless" (v: V32) -> V32 {
	return simd.swizzle(v, 1, 1, 1, 1)
}

@(private = "file")
SplatZV :: #force_inline proc "contextless" (v: V32) -> V32 {
	return simd.swizzle(v, 2, 2, 2, 2)
}

@(private = "file")
AnyGreaterEq3V :: #force_inline proc "contextless" (a, b: V32) -> bool {
	v := transmute(V32)simd.lanes_ge(a, b)
	return (MoveMask(v) & 0x07) != 0
}

@(private = "file")
Dot3V :: #force_inline proc "contextless" (a, b: V32) -> V32 {
	m := a * b
	x := simd.swizzle(m, 0, 0, 0, 0)
	y := simd.swizzle(m, 1, 1, 1, 1)
	z := simd.swizzle(m, 2, 2, 2, 2)

	return (x + y) + z
}

TestBoundsTriangleOverlap :: proc "contextless" (nodeCenter, nodeExtent, vertex1, vertex2, vertex3: V32) -> bool {
	vertex1, vertex2, vertex3 := vertex1, vertex2, vertex3
	two := SplatV(2.0)

	// Setup triangle
	vertex1 = SubV(vertex1, nodeCenter)
	vertex2 = SubV(vertex2, nodeCenter)
	vertex3 = SubV(vertex3, nodeCenter)

	// Face separation
	triangleMin := MinV(vertex1, MinV(vertex2, vertex3))
	triangleMax := MaxV(vertex1, MaxV(vertex2, vertex3))

	separation1 := SubV(triangleMin, nodeExtent)
	separation2 := AddV(triangleMax, nodeExtent)

	faceSeparation := MaxV(separation1, NegV(separation2))
	if AnyGreater3V(faceSeparation, zeroV) {
		return false
	}

	// SAT: Face separation
	edge1 := SubV(vertex2, vertex1)
	edge2 := SubV(vertex3, vertex2)
	edge3 := SubV(vertex1, vertex3)

	normal := CrossV(edge1, edge2)

	triangleSeparation := SubV(AbsV(Dot3V(normal, vertex1)), Dot3V(AbsV(normal), nodeExtent))
	if AnyGreater3V(triangleSeparation, zeroV) {
		return false
	}

	// SAT: Edge separation
	edgeSeparation1 := SubV(
		SubV(AbsV(CrossV(edge1, AddV(vertex1, vertex3))), AbsV(CrossV(edge1, edge3))),
		MulV(two, ModifiedCrossV(AbsV(edge1), nodeExtent)),
	)
	if AnyGreater3V(edgeSeparation1, zeroV) {
		return false
	}

	edgeSeparation2 := SubV(
		SubV(AbsV(CrossV(edge2, AddV(vertex1, vertex2))), AbsV(CrossV(edge2, edge1))),
		MulV(two, ModifiedCrossV(AbsV(edge2), nodeExtent)),
	)
	if AnyGreater3V(edgeSeparation2, zeroV) {
		return false
	}

	edgeSeparation3 := SubV(
		SubV(AbsV(CrossV(edge3, AddV(vertex2, vertex3))), AbsV(CrossV(edge3, edge2))),
		MulV(two, ModifiedCrossV(AbsV(edge3), nodeExtent)),
	)
	if AnyGreater3V(edgeSeparation3, zeroV) {
		return false
	}

	return true
}

IntersectRayTriangle :: proc "contextless" (rayStart, rayDelta, vertex1, vertex2, vertex3: V32) -> f32 {
	// Test if ray intersects this triangle sharing same calculations for each triangle
	{
		edge1 := SubV(vertex3, vertex2)
		edge2 := SubV(vertex1, vertex3)
		edge3 := SubV(vertex2, vertex1)

		midPoint1 := MulV(halfV, AddV(vertex2, vertex3))
		midPoint2 := MulV(halfV, AddV(vertex3, vertex1))
		midPoint3 := MulV(halfV, AddV(vertex1, vertex2))

		normal1 := CrossV(edge1, SubV(midPoint1, rayStart))
		normal2 := CrossV(edge2, SubV(midPoint2, rayStart))
		normal3 := CrossV(edge3, SubV(midPoint3, rayStart))
		TRANSPOSE3(&normal1, &normal2, &normal3)

		rayDeltaX := SplatXV(rayDelta)
		rayDeltaY := SplatYV(rayDelta)
		rayDeltaZ := SplatZV(rayDelta)

		volumes := AddV(AddV(MulV(normal1, rayDeltaX), MulV(normal2, rayDeltaY)), MulV(normal3, rayDeltaZ))
		if AnyLess3V(volumes, zeroV) {
			return 1.0
		}
	}

	// Compute intersection with triangle plane
	edge1 := SubV(vertex2, vertex1)
	edge2 := SubV(vertex3, vertex1)
	normal := CrossV(edge1, edge2)

	denominator := Dot3V(normal, rayDelta)
	if AnyGreaterEq3V(denominator, zeroV) {
		return 1.0
	}

	lambda := DivV(Dot3V(normal, SubV(vertex1, rayStart)), denominator)
	if AnyLessEq3V(lambda, zeroV) {
		return 1.0
	}

	lambda = MinV(lambda, oneV)
	return GetXV(lambda)
}
