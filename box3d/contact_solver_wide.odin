// Ported from box3d src/contact_solver_wide.inl at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

// This is the wide4.c instantiation (B3_SIMD_WIDTH 4): B3_WIDE(name) is name##W4.
// unified: not ported: wide8.c (the AVX2 instantiation); every platform runs width 4
// (that covers every B3_SIMD_WIDTH == 8 branch below)

// Wide vec2
Vec2W4 :: struct {
	x, y: FloatW,
}
Vec2W :: Vec2W4

// Wide quaternion
QuatW4 :: struct {
	V: Vec3W,
	S: FloatW,
}
QuatW :: QuatW4

// Wide symmetric matrix2
SymMatrix2W4 :: struct {
	cxx, cxy, cyy: FloatW,
}
SymMatrix2W :: SymMatrix2W4

// Wide symmetric matrix3
SymMatrix3W4 :: struct {
	cxx, cxy, cxz, cyy, cyz, czz: FloatW,
}
SymMatrix3W :: SymMatrix3W4

Matrix3W4 :: struct {
	cx, cy, cz: Vec3W,
}
Matrix3W :: Matrix3W4

// a + b
@(private = "file")
AddV2W :: proc(a: Vec2W, b: Vec2W) -> Vec2W {
	return Vec2W{AddW(a.x, b.x), AddW(a.y, b.y)}
}

// m * a
@(private = "file")
MulMV2W :: proc(m: SymMatrix2W, a: Vec2W) -> Vec2W {
	b := Vec2W{AddW(MulW(m.cxx, a.x), MulW(m.cxy, a.y)), AddW(MulW(m.cxy, a.x), MulW(m.cyy, a.y))}

	return b
}

// m * a
@(private = "file")
MulMVW :: proc(m: SymMatrix3W, a: Vec3W) -> Vec3W {
	b := Vec3W {
		AddW(MulW(m.cxx, a.X), AddW(MulW(m.cxy, a.Y), MulW(m.cxz, a.Z))),
		AddW(MulW(m.cxy, a.X), AddW(MulW(m.cyy, a.Y), MulW(m.cyz, a.Z))),
		AddW(MulW(m.cxz, a.X), AddW(MulW(m.cyz, a.Y), MulW(m.czz, a.Z))),
	}

	return b
}

// a - m * b
@(private = "file")
MulSubMVW :: proc(a: Vec3W, m: SymMatrix3W, b: Vec3W) -> Vec3W {
	c := Vec3W {
		AddW(MulW(m.cxx, b.X), AddW(MulW(m.cxy, b.Y), MulW(m.cxz, b.Z))),
		AddW(MulW(m.cxy, b.X), AddW(MulW(m.cyy, b.Y), MulW(m.cyz, b.Z))),
		AddW(MulW(m.cxz, b.X), AddW(MulW(m.cyz, b.Y), MulW(m.czz, b.Z))),
	}

	return Vec3W{SubW(a.X, c.X), SubW(a.Y, c.Y), SubW(a.Z, c.Z)}
}

// a + m * b
@(private = "file")
MulAddMVW :: proc(a: Vec3W, m: SymMatrix3W, b: Vec3W) -> Vec3W {
	c := Vec3W {
		AddW(MulW(m.cxx, b.X), AddW(MulW(m.cxy, b.Y), MulW(m.cxz, b.Z))),
		AddW(MulW(m.cxy, b.X), AddW(MulW(m.cyy, b.Y), MulW(m.cyz, b.Z))),
		AddW(MulW(m.cxz, b.X), AddW(MulW(m.cyz, b.Y), MulW(m.czz, b.Z))),
	}

	return Vec3W{AddW(a.X, c.X), AddW(a.Y, c.Y), AddW(a.Z, c.Z)}
}

@(private = "file")
MulM3VW :: proc(m: Matrix3W, a: Vec3W) -> Vec3W {
	b := Vec3W {
		AddW(MulW(m.cx.X, a.X), AddW(MulW(m.cy.X, a.Y), MulW(m.cz.X, a.Z))),
		AddW(MulW(m.cx.Y, a.X), AddW(MulW(m.cy.Y, a.Y), MulW(m.cz.Y, a.Z))),
		AddW(MulW(m.cx.Z, a.X), AddW(MulW(m.cy.Z, a.Y), MulW(m.cz.Z, a.Z))),
	}

	return b
}

@(private = "file")
InvRotateVectorW :: proc(q: QuatW, a: Vec3W) -> Vec3W {
	t := CrossW(q.V, a)
	t = Vec3W{AddW(t.X, t.X), AddW(t.Y, t.Y), AddW(t.Z, t.Z)}
	u := CrossW(q.V, t)
	return Vec3W {
		AddW(SubW(a.X, MulW(q.S, t.X)), u.X),
		AddW(SubW(a.Y, MulW(q.S, t.Y)), u.Y),
		AddW(SubW(a.Z, MulW(q.S, t.Z)), u.Z),
	}
}

// Soft contact constraints with sub-stepping support
// Uses fixed anchors for Jacobians for better behavior on rolling shapes (circles & capsules)
// http://mmacklin.com/smallsteps.pdf
// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf

ContactConstraintPointW4 :: struct {
	anchorAs, anchorBs:  Vec3W,
	baseSeparations:     FloatW,
	normalImpulses:      FloatW,
	totalNormalImpulses: FloatW,
	normalMasses:        FloatW,
	leverArms:           FloatW,
	relativeVelocities:  FloatW,
	restitutionImpulses: FloatW,
}
ContactConstraintPointWide :: ContactConstraintPointW4

ContactManifoldW4 :: struct #align (WIDE_ALIGNMENT) {
	normal:           Vec3W,
	tangent1:         Vec3W,
	tangent2:         Vec3W,

	// Friction centers
	centerA, centerB: Vec3W,
	twistMass:        FloatW,
	twistImpulse:     FloatW,
	tangentMass:      SymMatrix2W,
	frictionImpulse:  Vec2W,
	rollingImpulse:   Vec3W,
	rollingMask:      FloatW,
	tangentVelocity1: FloatW,
	tangentVelocity2: FloatW,
	pointCount:       i32,
	points:           [MAX_MANIFOLD_POINTS]ContactConstraintPointWide,
}
ContactManifoldWide :: ContactManifoldW4

// Solves one contact per lane
ContactConstraintW4 :: struct #align (WIDE_ALIGNMENT) {
	// These are base 1
	indexA:             [SIMD_WIDTH]i32,
	indexB:             [SIMD_WIDTH]i32,
	contacts:           [SIMD_WIDTH]^Contact,
	invMassA, invMassB: FloatW,
	invIA, invIB:       SymMatrix3W,
	rollingMass:        SymMatrix3W,
	friction:           FloatW,
	rollingResistance:  FloatW,
	restitution:        FloatW,
	manifoldCount:      i32,
}
ContactConstraintWide :: ContactConstraintW4

ConvexConstraintW4 :: struct {
	base:     ContactConstraintWide,
	manifold: ContactManifoldWide,
}
ConvexConstraintWide :: ConvexConstraintW4

#assert(offset_of(ConvexConstraintWide, manifold) == size_of(ContactConstraintWide), "wide constraint layout")
#assert(size_of(ContactConstraintWide) % WIDE_ALIGNMENT == 0, "wide constraint layout")
#assert(size_of(ContactManifoldWide) % WIDE_ALIGNMENT == 0, "wide constraint layout")

GetWideContactConstraintByteCountW4 :: proc() -> i32 {
	return i32(size_of(ConvexConstraintWide))
}

GetWideMeshConstraintByteCountW4 :: proc() -> i32 {
	return i32(size_of(ContactConstraintWide))
}

GetWideMeshManifoldByteCountW4 :: proc() -> i32 {
	return i32(size_of(ContactManifoldWide))
}

// wide version of b3BodyState
BodyStateW4 :: struct {
	v:  Vec3W,
	w:  Vec3W,
	dp: Vec3W,
	dq: QuatW,
}
BodyStateW :: BodyStateW4

// unified: not ported: the B3_SIMD_NONE b3GatherBodies and b3ScatterBodies (simd_scalar.h is not ported, simd.odin)

#assert(size_of(BodyState) == 64, "body state layout")
#assert(
	offset_of(BodyState, linearVelocity) == 0 &&
	offset_of(BodyState, angularVelocity) == 16 &&
	offset_of(BodyState, deltaPosition) == 32 &&
	offset_of(BodyState, deltaRotation) == 48,
	"body state layout",
)

@(private = "file")
GatherBodies :: #force_inline proc(states: [^]BodyState, indices: [^]i32) -> BodyStateW {
	identity := cast([^]f32)&identityBodyState

	// Indices are 0 for null
	p1 := indices[0] == 0 ? identity : cast([^]f32)&states[indices[0] - 1]
	p2 := indices[1] == 0 ? identity : cast([^]f32)&states[indices[1] - 1]
	p3 := indices[2] == 0 ? identity : cast([^]f32)&states[indices[2] - 1]
	p4 := indices[3] == 0 ? identity : cast([^]f32)&states[indices[3] - 1]

	s: BodyStateW
	pad: FloatW
	TransposeW(LoadW(p1), LoadW(p2), LoadW(p3), LoadW(p4), &s.v.X, &s.v.Y, &s.v.Z, &pad)
	TransposeW(LoadW(p1[4:]), LoadW(p2[4:]), LoadW(p3[4:]), LoadW(p4[4:]), &s.w.X, &s.w.Y, &s.w.Z, &pad)
	TransposeW(LoadW(p1[8:]), LoadW(p2[8:]), LoadW(p3[8:]), LoadW(p4[8:]), &s.dp.X, &s.dp.Y, &s.dp.Z, &pad)
	TransposeW(
		LoadW(p1[12:]),
		LoadW(p2[12:]),
		LoadW(p3[12:]),
		LoadW(p4[12:]),
		&s.dq.V.X,
		&s.dq.V.Y,
		&s.dq.V.Z,
		&s.dq.S,
	)
	return s
}

@(private = "file")
StoreBodyVelocity :: #force_inline proc(states: [^]BodyState, index: i32, v: FloatW, w: FloatW) {
	// Indices are 0 for null
	if index == 0 {
		return
	}

	state := &states[index - 1]
	flags := state.flags
	if (flags & dynamicFlag) == 0 {
		return
	}

	StoreW(cast([^]f32)state, v)
	StoreW((cast([^]f32)state)[4:], w)
}

// This writes only the velocities back to the solver bodies
@(private = "file")
ScatterBodies :: #force_inline proc(states: [^]BodyState, indices: [^]i32, simdBody: ^BodyStateW) {
	// I don't use any dummy body in the body array because this will lead to multithreaded sharing and the
	// associated cache flushing.
	zero := ZeroW()
	v1, v2, v3, v4: FloatW
	TransposeW(simdBody.v.X, simdBody.v.Y, simdBody.v.Z, zero, &v1, &v2, &v3, &v4)
	w1, w2, w3, w4: FloatW
	TransposeW(simdBody.w.X, simdBody.w.Y, simdBody.w.Z, zero, &w1, &w2, &w3, &w4)

	StoreBodyVelocity(states, indices[0], v1, w1)
	StoreBodyVelocity(states, indices[1], v2, w2)
	StoreBodyVelocity(states, indices[2], v3, w3)
	StoreBodyVelocity(states, indices[3], v4, w4)
}

#assert(offset_of(ManifoldPoint, anchorA) == 0, "manifold point layout")
#assert(offset_of(ManifoldPoint, anchorB) == 12, "manifold point layout")
#assert(offset_of(ManifoldPoint, separation) == 24, "manifold point layout")
#assert(offset_of(ManifoldPoint, normalImpulse) == 28, "manifold point layout")
#assert(offset_of(Manifold, twistImpulse) == offset_of(Manifold, normal) + 12, "manifold layout")
#assert(offset_of(Manifold, frictionImpulse) == offset_of(Manifold, normal) + 16, "manifold layout")
#assert(offset_of(Manifold, rollingImpulse) == offset_of(Manifold, normal) + 28, "manifold layout")
#assert(offset_of(Manifold, pointCount) == offset_of(Manifold, normal) + 40, "manifold layout")
#assert(offset_of(Matrix3, cy) == 12 && offset_of(Matrix3, cz) == 24 && size_of(Matrix3) == 36, "matrix layout")

@(private = "file", rodata)
zeroContact := Contact{}
@(private = "file", rodata)
zeroManifold := Manifold{}
@(private = "file", rodata)
zeroBodySim := BodySim{}

// unified: not ported: the B3_GATHER_LANES macro (it takes a field name); its uses below are expanded inline

@(private = "file")
GatherInvInertiaW :: proc(simLanes: [^]^BodySim) -> SymMatrix3W {
	i0 := cast([^]f32)&simLanes[0].invInertiaWorld.cx.x
	i1 := cast([^]f32)&simLanes[1].invInertiaWorld.cx.x
	i2 := cast([^]f32)&simLanes[2].invInertiaWorld.cx.x
	i3 := cast([^]f32)&simLanes[3].invInertiaWorld.cx.x

	m: SymMatrix3W
	unused: FloatW
	TransposeW(LoadW(i0), LoadW(i1), LoadW(i2), LoadW(i3), &m.cxx, &m.cxy, &m.cxz, &unused)
	TransposeW(LoadW(i0[4:]), LoadW(i1[4:]), LoadW(i2[4:]), LoadW(i3[4:]), &m.cyy, &m.cyz, &unused, &unused)
	m.czz = SetW(i0[8], i1[8], i2[8], i3[8])
	return m
}

@(private = "file")
AddSymW :: proc(a: SymMatrix3W, b: SymMatrix3W) -> SymMatrix3W {
	return SymMatrix3W {
		AddW(a.cxx, b.cxx),
		AddW(a.cxy, b.cxy),
		AddW(a.cxz, b.cxz),
		AddW(a.cyy, b.cyy),
		AddW(a.cyz, b.cyz),
		AddW(a.czz, b.czz),
	}
}

@(private = "file")
InvertSymW :: proc(m: SymMatrix3W) -> SymMatrix3W {
	cxx := SubW(MulW(m.cyy, m.czz), MulW(m.cyz, m.cyz))
	cxy := SubW(MulW(m.cxz, m.cyz), MulW(m.cxy, m.czz))
	cxz := SubW(MulW(m.cxy, m.cyz), MulW(m.cxz, m.cyy))
	cyy := SubW(MulW(m.cxx, m.czz), MulW(m.cxz, m.cxz))
	cyz := SubW(MulW(m.cxy, m.cxz), MulW(m.cxx, m.cyz))
	czz := SubW(MulW(m.cxx, m.cyy), MulW(m.cxy, m.cxy))

	det := AddW(MulW(m.cxx, cxx), AddW(MulW(m.cxy, cxy), MulW(m.cxz, cxz)))
	valid := GreaterThanW(AbsW(det), SplatW(1000.0 * FLT_MIN))
	invDet := BlendW(ZeroW(), DivW(SplatW(1.0), det), valid)

	return SymMatrix3W {
		MulW(invDet, cxx),
		MulW(invDet, cxy),
		MulW(invDet, cxz),
		MulW(invDet, cyy),
		MulW(invDet, cyz),
		MulW(invDet, czz),
	}
}

@(private = "file")
PerpW :: proc(a: Vec3W) -> Vec3W {
	zero := ZeroW()
	half := SplatW(0.5)
	mask := OrW(LessThanW(a.X, NegW(half)), GreaterThanW(a.X, half))

	p: Vec3W
	p.X = BlendW(zero, a.Y, mask)
	p.Y = BlendW(a.Z, NegW(a.X), mask)
	p.Z = BlendW(NegW(a.Y), zero, mask)

	lengthSquared := DotW(p, p)
	valid := GreaterThanW(lengthSquared, SplatW(1000.0 * FLT_MIN))
	s := BlendW(zero, DivW(SplatW(1.0), SqrtW(lengthSquared)), valid)
	return MulSVW(s, p)
}
@(private = "file")
IntsToFloatW :: proc(v: [^]i32) -> FloatW {
	return SetW(f32(v[0]), f32(v[1]), f32(v[2]), f32(v[3]))
}

// Get the contiguous manifold array that trails the constraint.
@(private = "file")
GetManifoldsW :: proc(c: ^ContactConstraintWide) -> [^]ContactManifoldWide {
	return cast([^]ContactManifoldWide)(cast([^]ContactConstraintWide)c)[1:]
}

// fixedManifoldCount is 1 to indicate a convex contact with one manifold.
@(private = "file")
GetConstraintW :: #force_inline proc(
	base: rawptr,
	manifoldStarts: [^]i32,
	index: i32,
	fixedManifoldCount: i32,
) -> ^ContactConstraintWide {
	if fixedManifoldCount > 0 {
		stride := size_of(ContactConstraintWide) + int(fixedManifoldCount) * size_of(ContactManifoldWide)
		return cast(^ContactConstraintWide)&(cast([^]u8)base)[int(index) * stride]
	}

	offset :=
		int(index) * size_of(ContactConstraintWide) +
		int(manifoldStarts[index] - manifoldStarts[0]) * size_of(ContactManifoldWide)
	return cast(^ContactConstraintWide)&(cast([^]u8)base)[offset]
}

@(private = "file")
NextConstraintW :: #force_inline proc(c: ^ContactConstraintWide, manifoldCount: i32) -> ^ContactConstraintWide {
	offset := size_of(ContactConstraintWide) + int(manifoldCount) * size_of(ContactManifoldWide)
	return cast(^ContactConstraintWide)&(cast([^]u8)c)[offset]
}

// Contact constraint data is stored in a heterogious stream. Constraints followed by an array of constraint manifolds.
// [c1 m11 m12 c2 m21 c3 m31 m32 m33 ... ]
@(private = "file")
PrepareConstraintW :: #force_inline proc(
	c: ^ContactConstraintWide,
	contacts: [^]^Contact,
	manifoldCount: i32,
	context_: ^StepContext,
	warmStartScale: FloatW,
) -> bool {
	sims := context_.sims
	states := context_.states
	when ENABLE_VALIDATION {
		bodies := context_.world.bodies.data
	}

	zeroW := ZeroW()
	oneW := SplatW(1.0)
	twoW := SplatW(2.0)
	invTau := SplatW(1.0 / SPECULATIVE_DISTANCE)
	minFrictionWeight := SplatW(MIN_FRICTION_WEIGHT)
	minDet := SplatW(1000.0 * FLT_MIN)

	contactLanes: [SIMD_WIDTH]^Contact
	simLanesA: [SIMD_WIDTH]^BodySim
	simLanesB: [SIMD_WIDTH]^BodySim
	manifoldCounts: [SIMD_WIDTH]i32
	hitEventLanes: i32 = 0
	maxManifoldCount: i32 = 0

	for lane: i32 = 0; lane < SIMD_WIDTH; lane += 1 {
		contact := contacts[lane]
		c.contacts[lane] = contact

		if contact != nil {
			indexA := DecodeAwakeIndex(contact.encodedBodySimA)
			indexB := DecodeAwakeIndex(contact.encodedBodySimB)

			when ENABLE_VALIDATION {
				bodyA := &bodies[contact.edges[0].bodyId]
				bodyB := &bodies[contact.edges[1].bodyId]
				assert(contact.encodedBodySimA == EncodeBodySimIndex(bodyA))
				assert(contact.encodedBodySimB == EncodeBodySimIndex(bodyB))
			}

			c.indexA[lane] = indexA + 1
			c.indexB[lane] = indexB + 1
			manifoldCounts[lane] = contact.manifoldCount

			contactLanes[lane] = contact
			simLanesA[lane] = indexA == NULL_INDEX ? &zeroBodySim : &sims[indexA]
			simLanesB[lane] = indexB == NULL_INDEX ? &zeroBodySim : &sims[indexB]
			hitEventLanes |= (contact.flags & simEnableHitEvent) != 0 ? i32(1) << u32(lane) : 0
			maxManifoldCount = MaxInt(maxManifoldCount, contact.manifoldCount)
		} else {
			c.indexA[lane] = 0
			c.indexB[lane] = 0
			manifoldCounts[lane] = 0

			contactLanes[lane] = &zeroContact
			simLanesA[lane] = &zeroBodySim
			simLanesB[lane] = &zeroBodySim
		}
	}

	VALIDATE(maxManifoldCount == manifoldCount)
	_ = maxManifoldCount
	c.manifoldCount = manifoldCount

	mA, mB: FloatW
	mA = SetW(simLanesA[0].invMass, simLanesA[1].invMass, simLanesA[2].invMass, simLanesA[3].invMass)
	mB = SetW(simLanesB[0].invMass, simLanesB[1].invMass, simLanesB[2].invMass, simLanesB[3].invMass)
	iA := GatherInvInertiaW(&simLanesA[0])
	iB := GatherInvInertiaW(&simLanesB[0])
	c.invMassA = mA
	c.invMassB = mB
	c.invIA = iA
	c.invIB = iB

	tangentVelocity: Vec3W
	c.friction = SetW(
		contactLanes[0].friction,
		contactLanes[1].friction,
		contactLanes[2].friction,
		contactLanes[3].friction,
	)
	c.rollingResistance = SetW(
		contactLanes[0].rollingResistance,
		contactLanes[1].rollingResistance,
		contactLanes[2].rollingResistance,
		contactLanes[3].rollingResistance,
	)
	c.restitution = SetW(
		contactLanes[0].restitution,
		contactLanes[1].restitution,
		contactLanes[2].restitution,
		contactLanes[3].restitution,
	)
	tangentVelocity.X = SetW(
		contactLanes[0].tangentVelocity.x,
		contactLanes[1].tangentVelocity.x,
		contactLanes[2].tangentVelocity.x,
		contactLanes[3].tangentVelocity.x,
	)
	tangentVelocity.Y = SetW(
		contactLanes[0].tangentVelocity.y,
		contactLanes[1].tangentVelocity.y,
		contactLanes[2].tangentVelocity.y,
		contactLanes[3].tangentVelocity.y,
	)
	tangentVelocity.Z = SetW(
		contactLanes[0].tangentVelocity.z,
		contactLanes[1].tangentVelocity.z,
		contactLanes[2].tangentVelocity.z,
		contactLanes[3].tangentVelocity.z,
	)

	invIAB := AddSymW(iA, iB)
	if AllZeroW(c.rollingResistance) == false {
		c.rollingMass = InvertSymW(invIAB)
	} else {
		c.rollingMass = SymMatrix3W{zeroW, zeroW, zeroW, zeroW, zeroW, zeroW}
	}

	rollingResistanceMask := GreaterThanW(c.rollingResistance, zeroW)

	manifolds := GetManifoldsW(c)
	for manifoldIndex: i32 = 0; manifoldIndex < manifoldCount; manifoldIndex += 1 {
		cm := &manifolds[manifoldIndex]

		manifoldLanes: [SIMD_WIDTH]^Manifold
		activeLanes: [SIMD_WIDTH]i32
		pointCounts: [SIMD_WIDTH]i32
		pointCount: i32 = 0
		for lane: i32 = 0; lane < SIMD_WIDTH; lane += 1 {
			if manifoldIndex < manifoldCounts[lane] {
				manifold := &contactLanes[lane].manifolds[manifoldIndex]
				manifoldLanes[lane] = manifold
				activeLanes[lane] = 1
				pointCounts[lane] = manifold.pointCount
			} else {
				manifoldLanes[lane] = &zeroManifold
				activeLanes[lane] = 0
				pointCounts[lane] = 0
			}

			pointCount = MaxInt(pointCount, pointCounts[lane])
		}

		VALIDATE(0 <= pointCount && pointCount <= MAX_MANIFOLD_POINTS)
		cm.pointCount = pointCount
		cm.rollingMask = AndW(rollingResistanceMask, GreaterThanW(IntsToFloatW(&activeLanes[0]), zeroW))

		normal, frictionImpulse, rollingImpulse: Vec3W
		twistImpulse: FloatW
		{
			m0 := cast([^]f32)&manifoldLanes[0].normal.x
			m1 := cast([^]f32)&manifoldLanes[1].normal.x
			m2 := cast([^]f32)&manifoldLanes[2].normal.x
			m3 := cast([^]f32)&manifoldLanes[3].normal.x
			TransposeW(LoadW(m0), LoadW(m1), LoadW(m2), LoadW(m3), &normal.X, &normal.Y, &normal.Z, &twistImpulse)
			TransposeW(
				LoadW(m0[4:]),
				LoadW(m1[4:]),
				LoadW(m2[4:]),
				LoadW(m3[4:]),
				&frictionImpulse.X,
				&frictionImpulse.Y,
				&frictionImpulse.Z,
				&rollingImpulse.X,
			)
			rollingImpulse.Y = SetW(m0[8], m1[8], m2[8], m3[8])
			rollingImpulse.Z = SetW(m0[9], m1[9], m2[9], m3[9])
		}

		tangent1 := PerpW(normal)
		tangent2 := CrossW(tangent1, normal)
		cm.normal = normal
		cm.tangent1 = tangent1
		cm.tangent2 = tangent2
		cm.tangentVelocity1 = DotW(tangentVelocity, tangent1)
		cm.tangentVelocity2 = DotW(tangentVelocity, tangent2)

		cm.twistImpulse = MulW(warmStartScale, twistImpulse)
		cm.rollingImpulse.X = BlendW(zeroW, MulW(warmStartScale, rollingImpulse.X), cm.rollingMask)
		cm.rollingImpulse.Y = BlendW(zeroW, MulW(warmStartScale, rollingImpulse.Y), cm.rollingMask)
		cm.rollingImpulse.Z = BlendW(zeroW, MulW(warmStartScale, rollingImpulse.Z), cm.rollingMask)
		cm.frictionImpulse.x = MulW(warmStartScale, DotW(frictionImpulse, tangent1))
		cm.frictionImpulse.y = MulW(warmStartScale, DotW(frictionImpulse, tangent2))

		pointCountW := IntsToFloatW(&pointCounts[0])

		centerA := Vec3W{zeroW, zeroW, zeroW}
		centerB := Vec3W{zeroW, zeroW, zeroW}
		totalFrictionWeight := zeroW

		for pointIndex: i32 = 0; pointIndex < pointCount; pointIndex += 1 {
			cp := &cm.points[pointIndex]
			pointMask := GreaterThanW(pointCountW, SplatW(f32(pointIndex)))

			p0 := cast([^]f32)&manifoldLanes[0].points[pointIndex]
			p1 := cast([^]f32)&manifoldLanes[1].points[pointIndex]
			p2 := cast([^]f32)&manifoldLanes[2].points[pointIndex]
			p3 := cast([^]f32)&manifoldLanes[3].points[pointIndex]

			rA, rB: Vec3W
			separation, normalImpulse: FloatW
			TransposeW(LoadW(p0), LoadW(p1), LoadW(p2), LoadW(p3), &rA.X, &rA.Y, &rA.Z, &rB.X)
			TransposeW(
				LoadW(p0[4:]),
				LoadW(p1[4:]),
				LoadW(p2[4:]),
				LoadW(p3[4:]),
				&rB.Y,
				&rB.Z,
				&separation,
				&normalImpulse,
			)

			rA.X = BlendW(zeroW, rA.X, pointMask)
			rA.Y = BlendW(zeroW, rA.Y, pointMask)
			rA.Z = BlendW(zeroW, rA.Z, pointMask)
			rB.X = BlendW(zeroW, rB.X, pointMask)
			rB.Y = BlendW(zeroW, rB.Y, pointMask)
			rB.Z = BlendW(zeroW, rB.Z, pointMask)
			separation = BlendW(zeroW, separation, pointMask)
			normalImpulse = BlendW(zeroW, normalImpulse, pointMask)

			// C0 friction center decay. Needed to prevent spinning top drift (GyroscopicPrecession sample).
			// See details in b3PrepareContacts_Mesh. This code should stay in sync.
			weight := MinW(MaxW(SubW(twoW, MulW(separation, invTau)), minFrictionWeight), oneW)
			weight = BlendW(zeroW, weight, pointMask)
			centerA = MulAddSVW(centerA, weight, rA)
			centerB = MulAddSVW(centerB, weight, rB)
			totalFrictionWeight = AddW(totalFrictionWeight, weight)

			cp.anchorAs = rA
			cp.anchorBs = rB
			cp.baseSeparations = SubW(separation, DotW(SubVW(rB, rA), normal))
			cp.normalImpulses = MulW(warmStartScale, normalImpulse)
			cp.totalNormalImpulses = zeroW
			cp.relativeVelocities = zeroW
			cp.restitutionImpulses = zeroW

			rnA := CrossW(rA, normal)
			rnB := CrossW(rB, normal)
			kNormal := AddW(mA, mB)
			kNormal = AddW(kNormal, DotW(rnA, MulMVW(iA, rnA)))
			kNormal = AddW(kNormal, DotW(rnB, MulMVW(iB, rnB)))
			valid := AndW(GreaterThanW(kNormal, zeroW), pointMask)
			cp.normalMasses = BlendW(zeroW, DivW(oneW, kNormal), valid)
		}

		frictionValid := GreaterThanW(totalFrictionWeight, zeroW)
		invWeight := BlendW(zeroW, DivW(oneW, totalFrictionWeight), frictionValid)
		centerA = MulSVW(invWeight, centerA)
		centerB = MulSVW(invWeight, centerB)
		cm.centerA = centerA
		cm.centerB = centerB

		for pointIndex: i32 = 0; pointIndex < pointCount; pointIndex += 1 {
			cp := &cm.points[pointIndex]
			pointMask := GreaterThanW(pointCountW, SplatW(f32(pointIndex)))
			d := SubVW(cp.anchorAs, centerA)
			cp.leverArms = BlendW(zeroW, SqrtW(DotW(d, d)), pointMask)
		}

		{
			rtA1 := CrossW(centerA, tangent1)
			rtA2 := CrossW(centerA, tangent2)
			rtB1 := CrossW(centerB, tangent1)
			rtB2 := CrossW(centerB, tangent2)
			iArtA1 := MulMVW(iA, rtA1)
			iArtA2 := MulMVW(iA, rtA2)
			iBrtB1 := MulMVW(iB, rtB1)
			iBrtB2 := MulMVW(iB, rtB2)

			kxx := AddW(AddW(AddW(mA, mB), DotW(rtA1, iArtA1)), DotW(rtB1, iBrtB1))
			kyy := AddW(AddW(AddW(mA, mB), DotW(rtA2, iArtA2)), DotW(rtB2, iBrtB2))
			kxy := AddW(DotW(rtA1, iArtA2), DotW(rtB1, iBrtB2))

			det := SubW(MulW(kxx, kyy), MulW(kxy, kxy))
			valid := AndW(GreaterThanW(AbsW(det), minDet), frictionValid)
			invDet := BlendW(zeroW, DivW(oneW, det), valid)
			cm.tangentMass.cxx = MulW(invDet, kyy)
			cm.tangentMass.cxy = NegW(MulW(invDet, kxy))
			cm.tangentMass.cyy = MulW(invDet, kxx)
		}

		{
			kTwist := DotW(normal, MulMVW(invIAB, normal))
			cm.twistMass = BlendW(zeroW, DivW(oneW, kTwist), GreaterThanW(kTwist, zeroW))
		}
	}

	// Only sample contact point normal velocity if needed.
	haveRestitution := AnyTrueW(GreaterThanW(c.restitution, zeroW))
	if hitEventLanes != 0 || haveRestitution {
		bA := GatherBodies(states, &c.indexA[0])
		bB := GatherBodies(states, &c.indexB[0])

		for manifoldIndex: i32 = 0; manifoldIndex < manifoldCount; manifoldIndex += 1 {
			cm := &manifolds[manifoldIndex]

			for pointIndex: i32 = 0; pointIndex < cm.pointCount; pointIndex += 1 {
				cp := &cm.points[pointIndex]

				vrA := AddVW(bA.v, CrossW(bA.w, cp.anchorAs))
				vrB := AddVW(bB.v, CrossW(bB.w, cp.anchorBs))
				vn := DotW(cm.normal, SubVW(vrB, vrA))
				cp.relativeVelocities = vn

				if hitEventLanes != 0 {
					normalVelocities: [SIMD_WIDTH]f32
					StoreW(&normalVelocities[0], vn)

					for lane: i32 = 0; lane < SIMD_WIDTH; lane += 1 {
						if (hitEventLanes & (i32(1) << u32(lane))) == 0 || manifoldIndex >= manifoldCounts[lane] {
							continue
						}

						manifold := &c.contacts[lane].manifolds[manifoldIndex]
						if pointIndex < manifold.pointCount {
							manifold.points[pointIndex].normalVelocity = normalVelocities[lane]
						}
					}
				}
			}
		}
	}

	return haveRestitution
}

PrepareContacts_ConvexW4 :: proc(block: SolverBlock, context_: ^StepContext) {
	world := context_.world
	spans := context_.widePrepareSpans
	wideBase := context_.wideConstraints
	warmStartScale := world.enableWarmStarting ? SplatW(1.0) : ZeroW()
	anyRestitution := false

	wideIndex := block.startIndex
	endWideIndex := block.startIndex + i32(block.count)

	// Find color for start index. Linear search but fast.
	colorIndex: i32 = 0
	for spans[colorIndex + 1].start <= wideIndex {
		colorIndex += 1
	}

	// Loop over block
	for wideIndex < endWideIndex {
		colorWideStart := spans[colorIndex].start
		colorWideEndIndex := MinInt(spans[colorIndex + 1].start, endWideIndex)
		colorContactCount := spans[colorIndex].count
		contactIds := spans[colorIndex].contacts

		// Loop over color
		for ; wideIndex < colorWideEndIndex; wideIndex += 1 {
			c := GetConstraintW(wideBase, nil, wideIndex, 1)
			localWideIndex := wideIndex - colorWideStart

			contacts: [SIMD_WIDTH]^Contact
			for lane: i32 = 0; lane < SIMD_WIDTH; lane += 1 {
				contactIndex := SIMD_WIDTH * localWideIndex + lane
				if contactIndex < colorContactCount {
					contacts[lane] = Array_Get(&world.contacts, contactIds[contactIndex])
					assert(contacts[lane].manifoldCount == 1)
				} else {
					contacts[lane] = nil
				}
			}

			haveRestitution := PrepareConstraintW(c, &contacts[0], 1, context_, warmStartScale)
			anyRestitution = anyRestitution || haveRestitution
		}

		// Advance to next color
		colorIndex += 1
	}

	if anyRestitution {
		AtomicStoreInt(&context_.anyRestitution, 1)
	}
}

PrepareContacts_MeshWideW4 :: proc(block: SolverBlock, context_: ^StepContext) {
	world := context_.world
	spans := context_.meshPrepareSpans
	wideBase := context_.wideMeshConstraints
	manifoldStarts := context_.wideMeshManifoldStarts
	warmStartScale := world.enableWarmStarting ? SplatW(1.0) : ZeroW()
	anyRestitution := false

	wideIndex := block.startIndex
	endWideIndex := block.startIndex + i32(block.count)

	colorIndex: i32 = 0
	for spans[colorIndex + 1].start <= wideIndex {
		colorIndex += 1
	}

	for wideIndex < endWideIndex {
		colorWideStart := spans[colorIndex].start
		colorWideEndIndex := MinInt(spans[colorIndex + 1].start, endWideIndex)
		colorContactCount := spans[colorIndex].count
		specs := spans[colorIndex].contacts

		// This order is used to group contacts with similar manifold counts to keep the lanes full.
		order := spans[colorIndex].order

		for ; wideIndex < colorWideEndIndex; wideIndex += 1 {
			c := GetConstraintW(wideBase, manifoldStarts, wideIndex, 0)
			localWideIndex := wideIndex - colorWideStart

			contacts: [SIMD_WIDTH]^Contact
			for lane: i32 = 0; lane < SIMD_WIDTH; lane += 1 {
				contactIndex := SIMD_WIDTH * localWideIndex + lane
				if contactIndex < colorContactCount {
					spec := &specs[order[contactIndex]]
					contact := Array_Get(&world.contacts, spec.contactId)
					assert(contact.contactId == spec.contactId)
					assert(contact.manifoldCount == i32(spec.manifoldCount))
					contacts[lane] = contact
				} else {
					contacts[lane] = nil
				}
			}

			manifoldCount := manifoldStarts[wideIndex + 1] - manifoldStarts[wideIndex]
			haveRestitution := PrepareConstraintW(c, &contacts[0], manifoldCount, context_, warmStartScale)
			anyRestitution = anyRestitution || haveRestitution
		}

		colorIndex += 1
	}

	if anyRestitution {
		AtomicStoreInt(&context_.anyRestitution, 1)
	}
}

@(private = "file")
WarmStartContactsW :: #force_inline proc(
	block: SolverBlock,
	context_: ^StepContext,
	base: rawptr,
	manifoldStarts: [^]i32,
	fixedManifoldCount: i32,
) {
	states := context_.states
	zeroW := ZeroW()

	c := GetConstraintW(base, manifoldStarts, block.startIndex, fixedManifoldCount)
	for i: i32 = 0; i < i32(block.count); i += 1 {
		manifoldCount := fixedManifoldCount > 0 ? fixedManifoldCount : c.manifoldCount
		anyRolling := AllZeroW(c.rollingResistance) == false

		linearImpulse := Vec3W{zeroW, zeroW, zeroW}
		angularImpulseA := Vec3W{zeroW, zeroW, zeroW}
		angularImpulseB := Vec3W{zeroW, zeroW, zeroW}

		manifolds := GetManifoldsW(c)
		for manifoldIndex: i32 = 0; manifoldIndex < manifoldCount; manifoldIndex += 1 {
			cm := &manifolds[manifoldIndex]

			totalNormalImpulse := zeroW
			momentA := Vec3W{zeroW, zeroW, zeroW}
			momentB := Vec3W{zeroW, zeroW, zeroW}

			for pointIndex: i32 = 0; pointIndex < cm.pointCount; pointIndex += 1 {
				cp := &cm.points[pointIndex]
				normalImpulse := cp.normalImpulses
				totalNormalImpulse = AddW(totalNormalImpulse, normalImpulse)
				momentA = MulAddSVW(momentA, normalImpulse, cp.anchorAs)
				momentB = MulAddSVW(momentB, normalImpulse, cp.anchorBs)
			}

			normal := cm.normal
			frictionImpulse := MulSVW(cm.frictionImpulse.x, cm.tangent1)
			frictionImpulse = MulAddSVW(frictionImpulse, cm.frictionImpulse.y, cm.tangent2)

			linear := MulAddSVW(frictionImpulse, totalNormalImpulse, normal)

			twistImpulse := MulSVW(cm.twistImpulse, normal)
			angularA := AddVW(AddVW(CrossW(momentA, normal), CrossW(cm.centerA, frictionImpulse)), twistImpulse)
			angularB := AddVW(AddVW(CrossW(momentB, normal), CrossW(cm.centerB, frictionImpulse)), twistImpulse)

			if anyRolling {
				angularA = AddVW(angularA, cm.rollingImpulse)
				angularB = AddVW(angularB, cm.rollingImpulse)
			}

			linearImpulse = AddVW(linearImpulse, linear)
			angularImpulseA = AddVW(angularImpulseA, angularA)
			angularImpulseB = AddVW(angularImpulseB, angularB)
		}

		bA := GatherBodies(states, &c.indexA[0])
		bB := GatherBodies(states, &c.indexB[0])

		bA.w = MulSubMVW(bA.w, c.invIA, angularImpulseA)
		bA.v = MulSubSVW(bA.v, c.invMassA, linearImpulse)
		bB.w = MulAddMVW(bB.w, c.invIB, angularImpulseB)
		bB.v = MulAddSVW(bB.v, c.invMassB, linearImpulse)

		ScatterBodies(states, &c.indexA[0], &bA)
		ScatterBodies(states, &c.indexB[0], &bB)

		c = NextConstraintW(c, manifoldCount)
	}
}

// Solve the non-penetration constraints with the soft bias. No friction and no restitution.
@(private = "file")
PushContactsW :: #force_inline proc(
	block: SolverBlock,
	context_: ^StepContext,
	base: rawptr,
	manifoldStarts: [^]i32,
	fixedManifoldCount: i32,
) {
	states := context_.states
	inv_h := SplatW(context_.inv_h)
	contactSpeed := SplatW(-context_.world.contactSpeed)
	oneW := SplatW(1.0)
	zeroW := ZeroW()

	// Stiffer for static contacts to avoid bodies getting pushed through the ground. Selected per
	// lane from the null body index instead of stored per constraint.
	dynamicBiasRate := SplatW(context_.contactSoftness.massScale * context_.contactSoftness.biasRate)
	dynamicMassScale := SplatW(context_.contactSoftness.massScale)
	dynamicImpulseScale := SplatW(context_.contactSoftness.impulseScale)
	staticBiasRate := SplatW(context_.staticSoftness.massScale * context_.staticSoftness.biasRate)
	staticMassScale := SplatW(context_.staticSoftness.massScale)
	staticImpulseScale := SplatW(context_.staticSoftness.impulseScale)

	c := GetConstraintW(base, manifoldStarts, block.startIndex, fixedManifoldCount)
	for i: i32 = 0; i < i32(block.count); i += 1 {
		manifoldCount := fixedManifoldCount > 0 ? fixedManifoldCount : c.manifoldCount
		bA := GatherBodies(states, &c.indexA[0])
		bB := GatherBodies(states, &c.indexB[0])

		softMask := SoftMaskW(&c.indexA[0], &c.indexB[0])
		biasRate := BlendW(dynamicBiasRate, staticBiasRate, softMask)
		massScale := BlendW(dynamicMassScale, staticMassScale, softMask)
		impulseScale := BlendW(dynamicImpulseScale, staticImpulseScale, softMask)

		dp := SubVW(bB.dp, bA.dp)
		invMass := AddW(c.invMassA, c.invMassB)

		manifolds := GetManifoldsW(c)
		for manifoldIndex: i32 = 0; manifoldIndex < manifoldCount; manifoldIndex += 1 {
			cm := &manifolds[manifoldIndex]

			// Convert to normals to local space to reduce transform math.
			// Read the normal rather than put on stack to avoid register spills.
			normalSeparation := DotW(cm.normal, dp)
			normalA := InvRotateVectorW(bA.dq, cm.normal)
			normalB := InvRotateVectorW(bB.dq, cm.normal)

			linearNormalVelocity := DotW(cm.normal, SubVW(bB.v, bA.v))
			manifoldImpulse := zeroW

			for pointIndex: i32 = 0; pointIndex < cm.pointCount; pointIndex += 1 {
				cp := &cm.points[pointIndex]

				// Fixed anchor points for applying impulses
				rA := cp.anchorAs
				rB := cp.anchorBs

				s := AddW(AddW(normalSeparation, SubW(DotW(normalB, rB), DotW(normalA, rA))), cp.baseSeparations)

				// Apply speculative bias if separation is greater than zero, otherwise apply soft constraint bias
				separated := GreaterThanW(s, zeroW)

				// Speculative bias - positive
				specBias := MulW(s, inv_h)

				// Overlap bias - negative
				overlapBias := MaxW(MulW(biasRate, s), contactSpeed)
				velocityBias := BlendW(overlapBias, specBias, separated)

				pointMassScale := BlendW(massScale, oneW, separated)
				pointImpulseScale := BlendW(impulseScale, zeroW, separated)

				uA := CrossW(rA, cm.normal)
				uB := CrossW(rB, cm.normal)
				gA := MulMVW(c.invIA, uA)
				gB := MulMVW(c.invIB, uB)

				vn := AddW(linearNormalVelocity, SubW(DotW(uB, bB.w), DotW(uA, bA.w)))

				negImpulse := AddW(
					MulW(cp.normalMasses, AddW(MulW(pointMassScale, vn), velocityBias)),
					MulW(pointImpulseScale, cp.normalImpulses),
				)
				impulse := MaxW(NegW(negImpulse), NegW(cp.normalImpulses))
				cp.normalImpulses = AddW(cp.normalImpulses, impulse)

				linearNormalVelocity = MulAddW(linearNormalVelocity, invMass, impulse)
				manifoldImpulse = AddW(manifoldImpulse, impulse)
				bA.w = MulSubSVW(bA.w, impulse, gA)
				bB.w = MulAddSVW(bB.w, impulse, gB)
			}

			P := MulSVW(manifoldImpulse, cm.normal)
			bA.v = MulSubSVW(bA.v, c.invMassA, P)
			bB.v = MulAddSVW(bB.v, c.invMassB, P)
		}

		ScatterBodies(states, &c.indexA[0], &bA)
		ScatterBodies(states, &c.indexB[0], &bB)

		c = NextConstraintW(c, manifoldCount)
	}
}

// Solve the normal constraint, friction, and rolling resistance.
@(private = "file")
SolveContactsW :: #force_inline proc(
	block: SolverBlock,
	context_: ^StepContext,
	base: rawptr,
	manifoldStarts: [^]i32,
	fixedManifoldCount: i32,
) {
	states := context_.states
	inv_h := SplatW(context_.inv_h)
	oneW := SplatW(1.0)
	zeroW := ZeroW()
	epsilonW := SplatW(FLT_EPSILON)

	c := GetConstraintW(base, manifoldStarts, block.startIndex, fixedManifoldCount)
	for i: i32 = 0; i < i32(block.count); i += 1 {
		manifoldCount := fixedManifoldCount > 0 ? fixedManifoldCount : c.manifoldCount
		bA := GatherBodies(states, &c.indexA[0])
		bB := GatherBodies(states, &c.indexB[0])

		anyRolling := AllZeroW(c.rollingResistance) == false
		dp := SubVW(bB.dp, bA.dp)
		invMass := AddW(c.invMassA, c.invMassB)

		manifolds := GetManifoldsW(c)
		for manifoldIndex: i32 = 0; manifoldIndex < manifoldCount; manifoldIndex += 1 {
			cm := &manifolds[manifoldIndex]

			// Read the normal rather than put on stack to avoid register spills.
			normalSeparation := DotW(cm.normal, dp)
			normalA := InvRotateVectorW(bA.dq, cm.normal)
			normalB := InvRotateVectorW(bB.dq, cm.normal)

			dv := SubVW(bB.v, bA.v)
			linearNormalVelocity := DotW(cm.normal, dv)
			linearTangentVelocity1 := SubW(DotW(cm.tangent1, dv), cm.tangentVelocity1)
			linearTangentVelocity2 := SubW(DotW(cm.tangent2, dv), cm.tangentVelocity2)
			manifoldImpulse := zeroW

			totalNormalImpulse := zeroW
			totalTwistLimit := zeroW

			// Normal contraints
			for pointIndex: i32 = 0; pointIndex < cm.pointCount; pointIndex += 1 {
				cp := &cm.points[pointIndex]

				// Fixed anchor points for applying impulses
				rA := cp.anchorAs
				rB := cp.anchorBs

				s := AddW(AddW(normalSeparation, SubW(DotW(normalB, rB), DotW(normalA, rA))), cp.baseSeparations)

				// Speculative bias, positive if separated and zero if overlapped
				velocityBias := MaxW(zeroW, MulW(s, inv_h))

				uA := CrossW(rA, cm.normal)
				uB := CrossW(rB, cm.normal)
				gA := MulMVW(c.invIA, uA)
				gB := MulMVW(c.invIB, uB)

				vn := AddW(linearNormalVelocity, SubW(DotW(uB, bB.w), DotW(uA, bA.w)))

				negImpulse := MulW(cp.normalMasses, AddW(vn, velocityBias))
				impulse := MaxW(NegW(negImpulse), NegW(cp.normalImpulses))
				newImpulse := AddW(cp.normalImpulses, impulse)
				cp.normalImpulses = newImpulse
				cp.totalNormalImpulses = AddW(cp.totalNormalImpulses, newImpulse)
				totalNormalImpulse = AddW(totalNormalImpulse, newImpulse)
				totalTwistLimit = AddW(totalTwistLimit, MulW(cp.leverArms, newImpulse))

				linearNormalVelocity = MulAddW(linearNormalVelocity, invMass, impulse)
				manifoldImpulse = AddW(manifoldImpulse, impulse)
				bA.w = MulSubSVW(bA.w, impulse, gA)
				bB.w = MulAddSVW(bB.w, impulse, gB)
			}

			// Rolling resistance. After normal constraints because it uses the normal impulse.
			if anyRolling {
				// flip A/B order to negate
				deltaImpulse := MulMVW(c.rollingMass, SubVW(bA.w, bB.w))
				oldImpulse := cm.rollingImpulse
				cm.rollingImpulse = AddVW(oldImpulse, deltaImpulse)

				maxImpulse := MulW(c.rollingResistance, totalNormalImpulse)
				lengthSquared := DotW(cm.rollingImpulse, cm.rollingImpulse)

				// if ( magSqr > maxLambda * maxLambda + FLT_EPSILON )
				//{
				//	c->rollingImpulse *= maxLambda / sqrtf( magSqr );
				// }

				mask := GreaterThanW(lengthSquared, MulAddW(epsilonW, maxImpulse, maxImpulse))

				// No approximate _mm_rsqrt_ps here to maintain cross-platform determinism
				normalize := DivW(maxImpulse, AddW(SqrtW(lengthSquared), epsilonW))
				scale := BlendW(oneW, normalize, mask)

				// Blend the result to ensure there are no +/-0 determinism problems, since 0 * -1 == -0.
				scaledImpulse := MulSVW(scale, cm.rollingImpulse)
				cm.rollingImpulse.X = BlendW(zeroW, scaledImpulse.X, cm.rollingMask)
				cm.rollingImpulse.Y = BlendW(zeroW, scaledImpulse.Y, cm.rollingMask)
				cm.rollingImpulse.Z = BlendW(zeroW, scaledImpulse.Z, cm.rollingMask)

				deltaImpulse = SubVW(cm.rollingImpulse, oldImpulse)

				bA.w = MulSubMVW(bA.w, c.invIA, deltaImpulse)
				bB.w = MulAddMVW(bB.w, c.invIB, deltaImpulse)
			}

			// Central twist friction. Friction goes last to improve stacking stability.
			{
				twistSpeed := DotW(cm.normal, SubVW(bB.w, bA.w))
				maxLambda := MulW(c.friction, totalTwistLimit)
				oldImpulse := cm.twistImpulse
				impulse := NegW(MulW(cm.twistMass, twistSpeed))
				impulse = MaxW(SubW(NegW(maxLambda), oldImpulse), MinW(impulse, SubW(maxLambda, oldImpulse)))
				cm.twistImpulse = AddW(oldImpulse, impulse)

				bA.w = MulSubSVW(bA.w, impulse, MulMVW(c.invIA, cm.normal))
				bB.w = MulAddSVW(bB.w, impulse, MulMVW(c.invIB, cm.normal))
			}

			// Central friction. Friction goes last to improve stacking stability.
			{
				tangent1 := cm.tangent1
				tangent2 := cm.tangent2

				// Fixed anchor points for applying impulses
				rA := cm.centerA
				rB := cm.centerB

				angularRelativeVelocity := SubVW(CrossW(bB.w, rB), CrossW(bA.w, rA))

				// Relative tangent velocity at contact
				vt := Vec2W {
					AddW(linearTangentVelocity1, DotW(angularRelativeVelocity, tangent1)),
					AddW(linearTangentVelocity2, DotW(angularRelativeVelocity, tangent2)),
				}

				// Incremental tangent impulse
				deltaImpulse := MulMV2W(cm.tangentMass, vt)
				deltaImpulse = Vec2W{NegW(deltaImpulse.x), NegW(deltaImpulse.y)}
				newImpulse := AddV2W(cm.frictionImpulse, deltaImpulse)

				maxImpulse := MulW(c.friction, totalNormalImpulse)

				// Clamp the accumulated impulse
				lengthSquared := AddW(MulW(newImpulse.x, newImpulse.x), MulW(newImpulse.y, newImpulse.y))

				// Max impulse can be zero
				mask := GreaterThanW(lengthSquared, MulW(maxImpulse, maxImpulse))

				// No approximate _mm_rsqrt_ps here to maintain cross-platform determinism. Add epsilon to avoid divide by
				// zero.
				normalize := DivW(maxImpulse, AddW(SqrtW(lengthSquared), epsilonW))
				scale := BlendW(oneW, normalize, mask)
				newImpulse = Vec2W{MulW(scale, newImpulse.x), MulW(scale, newImpulse.y)}

				deltaImpulse = Vec2W {
					SubW(newImpulse.x, cm.frictionImpulse.x),
					SubW(newImpulse.y, cm.frictionImpulse.y),
				}

				cm.frictionImpulse = newImpulse

				// Apply delta impulse
				P := AddVW(MulSVW(deltaImpulse.x, tangent1), MulSVW(deltaImpulse.y, tangent2))
				bA.w = MulSubMVW(bA.w, c.invIA, CrossW(rA, P))
				bB.w = MulAddMVW(bB.w, c.invIB, CrossW(rB, P))

				P = MulAddSVW(P, manifoldImpulse, cm.normal)
				bA.v = MulSubSVW(bA.v, c.invMassA, P)
				bB.v = MulAddSVW(bB.v, c.invMassB, P)
			}
		}

		ScatterBodies(states, &c.indexA[0], &bA)
		ScatterBodies(states, &c.indexB[0], &bB)

		c = NextConstraintW(c, manifoldCount)
	}
}

@(private = "file")
ApplyRestitutionW :: #force_inline proc(
	block: SolverBlock,
	context_: ^StepContext,
	base: rawptr,
	manifoldStarts: [^]i32,
	fixedManifoldCount: i32,
) {
	states := context_.states
	inv_h := SplatW(context_.inv_h)
	negRestitutionThreshold := SplatW(-context_.world.restitutionThreshold)
	zeroW := ZeroW()
	propagate := context_.world.enableRestitutionPropagation

	next := GetConstraintW(base, manifoldStarts, block.startIndex, fixedManifoldCount)
	for i: i32 = 0; i < i32(block.count); i += 1 {
		c := next
		manifoldCount := fixedManifoldCount > 0 ? fixedManifoldCount : c.manifoldCount
		next = NextConstraintW(c, manifoldCount)

		if propagate == false && AllZeroW(c.restitution) {
			continue
		}

		bA := GatherBodies(states, &c.indexA[0])
		bB := GatherBodies(states, &c.indexB[0])

		restitutionMask := GreaterThanW(c.restitution, zeroW)
		dp := SubVW(bB.dp, bA.dp)

		manifolds := GetManifoldsW(c)
		for manifoldIndex: i32 = 0; manifoldIndex < manifoldCount; manifoldIndex += 1 {
			cm := &manifolds[manifoldIndex]

			normalSeparation := DotW(cm.normal, dp)
			normalA := InvRotateVectorW(bA.dq, cm.normal)
			normalB := InvRotateVectorW(bB.dq, cm.normal)

			// Read the point count rather than put on stack to avoid register spills.
			for pointIndex: i32 = 0; pointIndex < cm.pointCount; pointIndex += 1 {
				cp := &cm.points[pointIndex]

				rA := cp.anchorAs
				rB := cp.anchorBs

				s := AddW(AddW(normalSeparation, SubW(DotW(normalB, rB), DotW(normalA, rA))), cp.baseSeparations)

				normalMass := propagate ? cp.normalMasses : BlendW(zeroW, cp.normalMasses, restitutionMask)

				compressionImpulse := SubW(cp.totalNormalImpulses, cp.restitutionImpulses)
				armed := AndW(
					AndW(restitutionMask, LessThanW(cp.relativeVelocities, negRestitutionThreshold)),
					GreaterThanW(compressionImpulse, zeroW),
				)

				specBias := MaxW(zeroW, MulW(s, inv_h))
				velocityBias := BlendW(specBias, MulW(c.restitution, cp.relativeVelocities), armed)

				vrA := AddVW(bA.v, CrossW(bA.w, rA))
				vrB := AddVW(bB.v, CrossW(bB.w, rB))
				vn := DotW(SubVW(vrB, vrA), cm.normal)

				negImpulse := MulW(normalMass, AddW(vn, velocityBias))

				newImpulse := MaxW(SubW(cp.normalImpulses, negImpulse), zeroW)
				impulse := SubW(newImpulse, cp.normalImpulses)

				approachImpulse := MinW(MaxW(NegW(MulW(normalMass, vn)), zeroW), MaxW(impulse, zeroW))
				allowance := SubW(
					MulW(c.restitution, AddW(compressionImpulse, approachImpulse)),
					cp.restitutionImpulses,
				)
				maxImpulse := AddW(approachImpulse, MaxW(allowance, zeroW))
				impulse = BlendW(impulse, MinW(impulse, maxImpulse), armed)

				cp.normalImpulses = AddW(cp.normalImpulses, impulse)
				cp.restitutionImpulses = AddW(cp.restitutionImpulses, SubW(impulse, approachImpulse))
				cp.totalNormalImpulses = AddW(cp.totalNormalImpulses, impulse)

				P := MulSVW(impulse, cm.normal)
				bA.w = MulSubMVW(bA.w, c.invIA, CrossW(rA, P))
				bA.v = MulSubSVW(bA.v, c.invMassA, P)
				bB.w = MulAddMVW(bB.w, c.invIB, CrossW(rB, P))
				bB.v = MulAddSVW(bB.v, c.invMassB, P)
			}
		}

		ScatterBodies(states, &c.indexA[0], &bA)
		ScatterBodies(states, &c.indexB[0], &bB)
	}
}

// Store impulses by contact constraint
@(private = "file")
StoreImpulsesW :: #force_inline proc(
	block: SolverBlock,
	context_: ^StepContext,
	workerIndex: i32,
	base: rawptr,
	manifoldStarts: [^]i32,
	fixedManifoldCount: i32,
) {
	world := context_.world
	taskContext := &world.taskContexts.data[workerIndex]
	hitEventBitSet := &taskContext.hitEventBitSet
	hasHitEvents := taskContext.hasHitEvents
	negHitThreshold := -world.hitEventThreshold

	c := GetConstraintW(base, manifoldStarts, block.startIndex, fixedManifoldCount)
	for i: i32 = 0; i < i32(block.count); i += 1 {
		manifoldCount := fixedManifoldCount > 0 ? fixedManifoldCount : c.manifoldCount
		manifolds := GetManifoldsW(c)

		// Pay the cache misses up front.
		laneManifolds: [SIMD_WIDTH][^]Manifold
		for lane: i32 = 0; lane < SIMD_WIDTH; lane += 1 {
			contact := c.contacts[lane]
			laneManifolds[lane] = contact != nil ? contact.manifolds : nil
		}

		for lane: i32 = 0; lane < SIMD_WIDTH; lane += 1 {
			contact := c.contacts[lane]
			if contact == nil {
				continue
			}

			laneManifoldCount := fixedManifoldCount > 0 ? fixedManifoldCount : contact.manifoldCount
			assert(laneManifoldCount == contact.manifoldCount && laneManifoldCount <= manifoldCount)

			for manifoldIndex: i32 = 0; manifoldIndex < laneManifoldCount; manifoldIndex += 1 {
				cm := &manifolds[manifoldIndex]
				m := &laneManifolds[lane][manifoldIndex]

				frictionImpulse1 := cast([^]f32)&cm.frictionImpulse.x
				frictionImpulse2 := cast([^]f32)&cm.frictionImpulse.y
				tangent1X := cast([^]f32)&cm.tangent1.X
				tangent1Y := cast([^]f32)&cm.tangent1.Y
				tangent1Z := cast([^]f32)&cm.tangent1.Z
				tangent2X := cast([^]f32)&cm.tangent2.X
				tangent2Y := cast([^]f32)&cm.tangent2.Y
				tangent2Z := cast([^]f32)&cm.tangent2.Z
				twistImpulse := cast([^]f32)&cm.twistImpulse
				rollingImpulseX := cast([^]f32)&cm.rollingImpulse.X
				rollingImpulseY := cast([^]f32)&cm.rollingImpulse.Y
				rollingImpulseZ := cast([^]f32)&cm.rollingImpulse.Z

				f1 := frictionImpulse1[lane]
				f2 := frictionImpulse2[lane]
				m.frictionImpulse = Vec3 {
					f1 * tangent1X[lane] + f2 * tangent2X[lane],
					f1 * tangent1Y[lane] + f2 * tangent2Y[lane],
					f1 * tangent1Z[lane] + f2 * tangent2Z[lane],
				}
				m.twistImpulse = twistImpulse[lane]
				m.rollingImpulse = Vec3{rollingImpulseX[lane], rollingImpulseY[lane], rollingImpulseZ[lane]}

				pointCount := m.pointCount
				assert(pointCount <= cm.pointCount)
				for pointIndex: i32 = 0; pointIndex < pointCount; pointIndex += 1 {
					cp := &cm.points[pointIndex]
					normalImpulse := cast([^]f32)&cp.normalImpulses
					totalNormalImpulse := cast([^]f32)&cp.totalNormalImpulses

					mp := &m.points[pointIndex]
					mp.normalImpulse = normalImpulse[lane]
					mp.totalNormalImpulse = totalNormalImpulse[lane]
				}
			}

			if (contact.flags & simEnableHitEvent) != 0 {
				flagged := false
				for manifoldIndex: i32 = 0; manifoldIndex < laneManifoldCount && flagged == false; manifoldIndex += 1 {
					m := &contact.manifolds[manifoldIndex]
					for pointIndex: i32 = 0; pointIndex < m.pointCount; pointIndex += 1 {
						mp := &m.points[pointIndex]

						// Need to check total impulse because the point may be speculative and not colliding
						if mp.normalVelocity < negHitThreshold && mp.totalNormalImpulse > 0.0 {
							SetBit(hitEventBitSet, u32(contact.contactId))
							hasHitEvents = true
							flagged = true
							break
						}
					}
				}
			}
		}

		c = NextConstraintW(c, manifoldCount)
	}

	taskContext.hasHitEvents = hasHitEvents
}

// These wrappers allow the implementation be shared between convex and mesh contacts.

WarmStartContacts_ConvexW4 :: proc(block: SolverBlock, context_: ^StepContext) {
	WarmStartContactsW(block, context_, context_.graph.colors[block.colorIndex].wideConstraints, nil, 1)
}

PushContacts_ConvexW4 :: proc(block: SolverBlock, context_: ^StepContext) {
	PushContactsW(block, context_, context_.graph.colors[block.colorIndex].wideConstraints, nil, 1)
}

SolveContacts_ConvexW4 :: proc(block: SolverBlock, context_: ^StepContext) {
	SolveContactsW(block, context_, context_.graph.colors[block.colorIndex].wideConstraints, nil, 1)
}

ApplyRestitution_ConvexW4 :: proc(block: SolverBlock, context_: ^StepContext) {
	ApplyRestitutionW(block, context_, context_.graph.colors[block.colorIndex].wideConstraints, nil, 1)
}

StoreImpulses_ConvexW4 :: proc(block: SolverBlock, context_: ^StepContext, workerIndex: i32) {
	StoreImpulsesW(block, context_, workerIndex, context_.wideConstraints, nil, 1)
}

WarmStartContacts_MeshWideW4 :: proc(block: SolverBlock, context_: ^StepContext) {
	color := &context_.graph.colors[block.colorIndex]
	WarmStartContactsW(block, context_, color.wideMeshConstraints, color.wideMeshManifoldStarts, 0)
}

PushContacts_MeshWideW4 :: proc(block: SolverBlock, context_: ^StepContext) {
	color := &context_.graph.colors[block.colorIndex]
	PushContactsW(block, context_, color.wideMeshConstraints, color.wideMeshManifoldStarts, 0)
}

SolveContacts_MeshWideW4 :: proc(block: SolverBlock, context_: ^StepContext) {
	color := &context_.graph.colors[block.colorIndex]
	SolveContactsW(block, context_, color.wideMeshConstraints, color.wideMeshManifoldStarts, 0)
}

ApplyRestitution_MeshWideW4 :: proc(block: SolverBlock, context_: ^StepContext) {
	color := &context_.graph.colors[block.colorIndex]
	ApplyRestitutionW(block, context_, color.wideMeshConstraints, color.wideMeshManifoldStarts, 0)
}

StoreImpulses_MeshWideW4 :: proc(block: SolverBlock, context_: ^StepContext, workerIndex: i32) {
	StoreImpulsesW(block, context_, workerIndex, context_.wideMeshConstraints, context_.wideMeshManifoldStarts, 0)
}
