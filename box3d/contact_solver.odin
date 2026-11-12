// Ported from box3d src/contact_solver.c and src/contact_solver.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:intrinsics"

// contact_solver.h

ManifoldConstraintPoint :: struct {
	rA, rB:             Vec3,
	baseSeparation:     f32,
	relativeVelocity:   f32,
	normalImpulse:      f32,
	totalNormalImpulse: f32,
	restitutionImpulse: f32,
	normalMass:         f32,
	leverArm:           f32,
}

ManifoldConstraint :: struct {
	points:           [MAX_MANIFOLD_POINTS]ManifoldConstraintPoint,
	pointCount:       i32,
	normal:           Vec3,
	tangent1:         Vec3,
	tangent2:         Vec3,
	// Friction centers
	centerA, centerB: Vec3,
	twistMass:        f32,
	twistImpulse:     f32,
	tangentMass:      Matrix2,
	frictionImpulse:  Vec2,
	rollingImpulse:   Vec3,
	tangentVelocity1: f32,
	tangentVelocity2: f32,
}

ContactConstraint :: struct {
	constraints:        [^]ManifoldConstraint,
	contact:            ^Contact,
	indexA:             i32,
	indexB:             i32,
	invMassA, invMassB: f32,
	invIA, invIB:       Matrix3,
	softness:           Softness,
	rollingMass:        Matrix3,
	friction:           f32,
	restitution:        f32,
	rollingResistance:  f32,
	manifoldCount:      i32,
}

// unified: the *_ConvexW4 and *_MeshWideW4 procs and their byte counts are declared in contact_solver.h and
// defined in contact_solver_wide.inl (contact_solver_wide.odin).
// unified: not ported: the *_ConvexW8 and *_MeshWideW8 declarations (wide8.c, the AVX2 instantiation); every
// platform runs width 4

// contact_solver.c

// todo clean this file up because it is only used for overflow now

// contact separation for sub-stepping
// s = s0 + dot(cB + rB - cA - rA, normal)
// normal is held constant
// body positions c can translation and anchors r can rotate
// s(t) = s0 + dot(cB(t) + rB(t) - cA(t) - rA(t), normal)
// s(t) = s0 + dot(cB0 + dpB + rot(dqB, rB0) - cA0 - dpA - rot(dqA, rA0), normal)
// s(t) = s0 + dot(cB0 - cA0, normal) + dot(dpB - dpA + rot(dqB, rB0) - rot(dqA, rA0), normal)
// s_base = s0 + dot(cB0 - cA0, normal)

// Prepare overflow constraints
PrepareContacts_Mesh :: proc(block: SolverBlock, context_: ^StepContext) {
	world := context_.world
	bodySims := context_.sims
	states := context_.states

	warmStartScale: f32 = world.enableWarmStarting ? 1.0 : 0.0
	anyRestitution := false

	// Used for friction center weighting.
	invTau: f32 = 1.0 / SPECULATIVE_DISTANCE

	assert(block.blockType == u8(SolverBlockType.overflowBlock))
	overflow := &world.constraintGraph.colors[OVERFLOW_INDEX]
	spans := context_.overflowSpans
	manifoldBase := overflow.manifoldConstraints
	base := overflow.contactConstraints

	index := block.startIndex
	endIndex := block.startIndex + i32(block.count)

	// Find color for start index. Linear search but fast.
	colorIndex: i32 = 0
	for spans[colorIndex + 1].start <= index {
		colorIndex += 1
	}

	// Loop over block
	for index < endIndex {
		colorStart := spans[colorIndex].start
		colorEndIndex := MinInt(spans[colorIndex + 1].start, endIndex)
		specs := spans[colorIndex].contacts

		// Loop over color
		for ; index < colorEndIndex; index += 1 {
			contactConstraint := &base[index]

			localIndex := index - colorStart
			assert(0 <= localIndex && localIndex < spans[colorIndex].count)
			contactId := specs[localIndex].contactId
			contact := Array_Get(&world.contacts, contactId)
			assert(contact.contactId == contactId)

			indexA := DecodeAwakeIndex(contact.encodedBodySimA)
			indexB := DecodeAwakeIndex(contact.encodedBodySimB)

			when ENABLE_VALIDATION {
				{
					bodyA := Array_Get(&world.bodies, contact.edges[0].bodyId)
					bodyB := Array_Get(&world.bodies, contact.edges[1].bodyId)
					assert(contact.encodedBodySimA == EncodeBodySimIndex(bodyA))
					assert(contact.encodedBodySimB == EncodeBodySimIndex(bodyB))
				}
			}

			// Body A data
			mA: f32
			iA: Matrix3

			if indexA == NULL_INDEX {
				mA = 0.0
				iA = Mat3_zero
			} else {
				simA := &bodySims[indexA]
				mA = simA.invMass
				iA = simA.invInertiaWorld
			}

			// Body B data
			mB: f32
			iB: Matrix3

			if indexB == NULL_INDEX {
				mB = 0.0
				iB = Mat3_zero
			} else {
				simB := &bodySims[indexB]
				mB = simB.invMass
				iB = simB.invInertiaWorld
			}

			manifoldCount := contact.manifoldCount
			contactConstraint.contact = contact
			contactConstraint.manifoldCount = manifoldCount
			contactConstraint.indexA = indexA
			contactConstraint.indexB = indexB
			contactConstraint.invIA = iA
			contactConstraint.invMassA = mA
			contactConstraint.invIB = iB
			contactConstraint.invMassB = mB
			contactConstraint.rollingMass = InvertMatrix(AddMM(iA, iB))
			contactConstraint.softness =
				(contact.flags & contactStaticFlag) != 0 ? context_.staticSoftness : context_.contactSoftness
			contactConstraint.friction = contact.friction
			contactConstraint.restitution = contact.restitution
			contactConstraint.rollingResistance = contact.rollingResistance

			// Only sample contact point normal velocity if needed.
			haveRestitution := contact.restitution > 0.0
			hitEvents := (contact.flags & simEnableHitEvent) != 0
			sampleVelocity := haveRestitution || hitEvents
			anyRestitution = anyRestitution || haveRestitution

			vA := Vec3_zero
			wA := Vec3_zero
			vB := Vec3_zero
			wB := Vec3_zero

			if sampleVelocity {
				if indexA != NULL_INDEX {
					vA = states[indexA].linearVelocity
					wA = states[indexA].angularVelocity
				}

				if indexB != NULL_INDEX {
					vB = states[indexB].linearVelocity
					wB = states[indexB].angularVelocity
				}
			}

			manifoldConstraints := manifoldBase[specs[localIndex].manifoldStart:]
			contactConstraint.constraints = manifoldConstraints

			for manifoldIndex: i32 = 0; manifoldIndex < manifoldCount; manifoldIndex += 1 {
				manifold := &contact.manifolds[manifoldIndex]
				constraint := &manifoldConstraints[manifoldIndex]
				pointCount := manifold.pointCount
				normal := manifold.normal
				tangent1 := Perp(normal)
				tangent2 := Cross(tangent1, normal)

				constraint.pointCount = pointCount
				constraint.normal = normal
				constraint.tangent1 = tangent1
				constraint.tangent2 = tangent2

				// Stiffer for static contacts to avoid bodies getting pushed through the ground
				constraint.tangentVelocity1 = Dot(contact.tangentVelocity, constraint.tangent1)
				constraint.tangentVelocity2 = Dot(contact.tangentVelocity, constraint.tangent2)

				centerA := Vec3_zero
				centerB := Vec3_zero
				totalFrictionWeight: f32 = 0.0

				for pointIndex: i32 = 0; pointIndex < pointCount; pointIndex += 1 {
					cp := &constraint.points[pointIndex]

					// Copy data from manifold point
					mp := &manifold.points[pointIndex]
					cp.rA = mp.anchorA
					cp.rB = mp.anchorB

					s := mp.separation
					cp.baseSeparation = s - Dot(Sub(cp.rB, cp.rA), normal)
					cp.normalImpulse = warmStartScale * mp.normalImpulse
					cp.totalNormalImpulse = 0.0
					cp.restitutionImpulse = 0.0

					rA := cp.rA
					rB := cp.rB

					rnA := Cross(rA, normal)
					rnB := Cross(rB, normal)
					kNormal := mA + mB + Dot(rnA, MulMV(iA, rnA)) + Dot(rnB, MulMV(iB, rnB))
					cp.normalMass = kNormal > 0.0 ? 1.0 / kNormal : 0.0

					// Only compute the normal velocity terms if needed.
					if sampleVelocity {
						vrA := Add(vA, Cross(wA, rA))
						vrB := Add(vB, Cross(wB, rB))
						vn := Dot(normal, Sub(vrB, vrA))

						cp.relativeVelocity = vn
						mp.normalVelocity = hitEvents ? vn : 0.0
					} else {
						cp.relativeVelocity = 0.0
						mp.normalVelocity = 0.0
					}

					// C0 friction center decay. Needed to prevent spinning top drift (GyroscopicPrecession sample).
					// Contacts with separation greater than twice the speculative distance only matter for CCD and
					// should not contribute to the friction center. They are not important for jitter reduction. Closer
					// points may begin to touch on and off, so the friction center needs to move smoothly.
					// Epsilon to avoid a branch below (or divide by zero). Small enough to get washed out normally.
					//
					// Dirk suggested to weight the points by their normal impulse. This would use the total impulse from
					// the previous time step. I suspect this could jump around if the active point set jitters, leading
					// to a limit cycle.
					weight := ClampFloat(2.0 - s * invTau, MIN_FRICTION_WEIGHT, 1.0)
					centerA = MulAdd(centerA, weight, rA)
					centerB = MulAdd(centerB, weight, rB)
					totalFrictionWeight += weight
				}

				invWeight := 1.0 / totalFrictionWeight
				centerA = MulSV(invWeight, centerA)
				centerB = MulSV(invWeight, centerB)
				constraint.centerA = centerA
				constraint.centerB = centerB

				for pointIndex: i32 = 0; pointIndex < pointCount; pointIndex += 1 {
					cp := &constraint.points[pointIndex]
					cp.leverArm = Distance(cp.rA, centerA)
				}

				rtA1 := Cross(centerA, tangent1)
				rtA2 := Cross(centerA, tangent2)
				rtB1 := Cross(centerB, tangent1)
				rtB2 := Cross(centerB, tangent2)

				{
					k: Matrix2
					k.cx.x = mA + mB + Dot(rtA1, MulMV(iA, rtA1)) + Dot(rtB1, MulMV(iB, rtB1))
					k.cy.y = mA + mB + Dot(rtA2, MulMV(iA, rtA2)) + Dot(rtB2, MulMV(iB, rtB2))
					k.cy.x = Dot(rtA1, MulMV(iA, rtA2)) + Dot(rtB1, MulMV(iB, rtB2))
					k.cx.y = k.cy.x

					constraint.tangentMass = Invert2(k)
					constraint.frictionImpulse.x = warmStartScale * Dot(manifold.frictionImpulse, tangent1)
					constraint.frictionImpulse.y = warmStartScale * Dot(manifold.frictionImpulse, tangent2)
				}

				{
					k := Dot(normal, MulMV(AddMM(iA, iB), normal))
					constraint.twistMass = k > 0.0 ? 1.0 / k : 0.0
					constraint.twistImpulse = warmStartScale * manifold.twistImpulse
				}

				{
					constraint.rollingImpulse = MulSV(warmStartScale, manifold.rollingImpulse)
				}
			}
		}

		// Advance to next color
		colorIndex += 1
	}

	if anyRestitution {
		AtomicStoreInt(&context_.anyRestitution, 1)
	}
}

WarmStartContacts_Mesh :: proc(block: SolverBlock, context_: ^StepContext) {
	world := context_.world
	color := &world.constraintGraph.colors[block.colorIndex]
	awakeSet := Array_Get(&world.solverSets, i32(SetType.awakeSet))
	states := awakeSet.bodyStates.data
	constraints := color.contactConstraints

	// This is a dummy state to represent a static body because static bodies don't have a solver body.
	dummyState := identityBodyState

	startIndex := block.startIndex
	endIndex := startIndex + i32(block.count)

	for constraintIndex := startIndex; constraintIndex < endIndex; constraintIndex += 1 {
		contactConstraint := &constraints[constraintIndex]

		linearImpulse := Vec3_zero
		angularImpulseA := Vec3_zero
		angularImpulseB := Vec3_zero

		manifoldCount := contactConstraint.manifoldCount
		for manifoldIndex: i32 = 0; manifoldIndex < manifoldCount; manifoldIndex += 1 {
			constraint := &contactConstraint.constraints[manifoldIndex]

			// Normal impulses
			normal := constraint.normal
			totalNormalImpulse: f32 = 0.0
			momentA := Vec3_zero
			momentB := Vec3_zero

			pointCount := constraint.pointCount
			for j: i32 = 0; j < pointCount; j += 1 {
				cp := &constraint.points[j]

				// fixed anchors
				totalNormalImpulse += cp.normalImpulse
				momentA = MulAdd(momentA, cp.normalImpulse, cp.rA)
				momentB = MulAdd(momentB, cp.normalImpulse, cp.rB)
			}

			// Central friction
			frictionImpulse := MulSV(constraint.frictionImpulse.x, constraint.tangent1)
			frictionImpulse = MulAdd(frictionImpulse, constraint.frictionImpulse.y, constraint.tangent2)

			linearImpulse = Add(linearImpulse, MulAdd(frictionImpulse, totalNormalImpulse, normal))

			// Central twist friction
			twistImpulse := MulSV(constraint.twistImpulse, normal)

			angularA := Add(Add(Cross(momentA, normal), Cross(constraint.centerA, frictionImpulse)), twistImpulse)
			angularB := Add(Add(Cross(momentB, normal), Cross(constraint.centerB, frictionImpulse)), twistImpulse)

			// Rolling resistance
			angularImpulseA = Add(angularImpulseA, Add(angularA, constraint.rollingImpulse))
			angularImpulseB = Add(angularImpulseB, Add(angularB, constraint.rollingImpulse))
		}

		indexA := contactConstraint.indexA
		indexB := contactConstraint.indexB

		stateA := indexA == NULL_INDEX ? &dummyState : &states[indexA]
		stateB := indexB == NULL_INDEX ? &dummyState : &states[indexB]

		if (stateA.flags & dynamicFlag) != 0 {
			stateA.linearVelocity = MulSub(stateA.linearVelocity, contactConstraint.invMassA, linearImpulse)
			stateA.angularVelocity = Sub(stateA.angularVelocity, MulMV(contactConstraint.invIA, angularImpulseA))
		}

		if (stateB.flags & dynamicFlag) != 0 {
			stateB.linearVelocity = MulAdd(stateB.linearVelocity, contactConstraint.invMassB, linearImpulse)
			stateB.angularVelocity = Add(stateB.angularVelocity, MulMV(contactConstraint.invIB, angularImpulseB))
		}
	}
}

// Merged normal and friction loops. This is much more stable for the Jenga stack.
// Solve the non-penetration constraints with the soft bias. No friction and no restitution.
PushContacts_Mesh :: proc(block: SolverBlock, context_: ^StepContext) {
	world := context_.world
	color := &world.constraintGraph.colors[block.colorIndex]
	contactConstraints := color.contactConstraints
	states := context_.states

	// This is a dummy state to represent a static body because static bodies have no solver body.
	dummyState := identityBodyState

	// The last block might not be full
	startIndex := block.startIndex
	endIndex := startIndex + i32(block.count)

	inv_h := context_.inv_h
	contactSpeed := context_.world.contactSpeed

	for i := startIndex; i < endIndex; i += 1 {
		contactConstraint := &contactConstraints[i]
		manifoldCount := contactConstraint.manifoldCount

		indexA := contactConstraint.indexA
		indexB := contactConstraint.indexB

		mA := contactConstraint.invMassA
		iA := contactConstraint.invIA
		mB := contactConstraint.invMassB
		iB := contactConstraint.invIB

		stateA := indexA == NULL_INDEX ? &dummyState : &states[indexA]
		vA := stateA.linearVelocity
		wA := stateA.angularVelocity
		dqA := stateA.deltaRotation

		stateB := indexB == NULL_INDEX ? &dummyState : &states[indexB]
		vB := stateB.linearVelocity
		wB := stateB.angularVelocity
		dqB := stateB.deltaRotation

		dp := Sub(stateB.deltaPosition, stateA.deltaPosition)
		softness := contactConstraint.softness

		for j: i32 = 0; j < manifoldCount; j += 1 {
			constraint := &contactConstraint.constraints[j]

			pointCount := constraint.pointCount
			normal := constraint.normal

			for pointIndex: i32 = 0; pointIndex < pointCount; pointIndex += 1 {
				cp := &constraint.points[pointIndex]

				// Fixed anchor points for applying impulses
				rA := cp.rA
				rB := cp.rB

				// compute current separation
				// this is subject to round-off error if the anchor is far from the body center of mass
				ds := Add(dp, Sub(RotateVector(dqB, rB), RotateVector(dqA, rA)))
				s := Dot(ds, normal) + cp.baseSeparation

				velocityBias: f32
				massScale: f32
				impulseScale: f32
				if s > 0.0 {
					// speculative bias is positive
					velocityBias = s * inv_h
					massScale = 1.0
					impulseScale = 0.0
				} else {
					// overlap bias is negative
					velocityBias = MaxFloat(softness.massScale * softness.biasRate * s, -contactSpeed)
					massScale = softness.massScale
					impulseScale = softness.impulseScale
				}

				// relative normal velocity at contact
				vrA := Add(vA, Cross(wA, rA))
				vrB := Add(vB, Cross(wB, rB))
				vn := Dot(Sub(vrB, vrA), normal)

				// incremental normal impulse
				deltaImpulse := -cp.normalMass * (massScale * vn + velocityBias) - impulseScale * cp.normalImpulse

				// clamp the accumulated impulse
				newImpulse := MaxFloat(cp.normalImpulse + deltaImpulse, 0.0)
				deltaImpulse = newImpulse - cp.normalImpulse
				cp.normalImpulse = newImpulse

				// apply normal impulse
				P := MulSV(deltaImpulse, normal)
				vA = MulSub(vA, mA, P)
				wA = Sub(wA, MulMV(iA, Cross(rA, P)))

				vB = MulAdd(vB, mB, P)
				wB = Add(wB, MulMV(iB, Cross(rB, P)))
			}
		}

		if (stateA.flags & dynamicFlag) != 0 {
			stateA.linearVelocity = vA
			stateA.angularVelocity = wA
		}

		if (stateB.flags & dynamicFlag) != 0 {
			stateB.linearVelocity = vB
			stateB.angularVelocity = wB
		}
	}
}

// Solve contacts: normal, friction and rolling resistance.
SolveContacts_Mesh :: proc(block: SolverBlock, context_: ^StepContext) {
	world := context_.world
	color := &world.constraintGraph.colors[block.colorIndex]
	contactConstraints := color.contactConstraints
	states := context_.states

	// This is a dummy state to represent a static body because static bodies have no solver body.
	dummyState := identityBodyState

	// The last block might not be full
	startIndex := block.startIndex
	endIndex := startIndex + i32(block.count)

	inv_h := context_.inv_h

	for i := startIndex; i < endIndex; i += 1 {
		contactConstraint := &contactConstraints[i]
		manifoldCount := contactConstraint.manifoldCount

		indexA := contactConstraint.indexA
		indexB := contactConstraint.indexB

		mA := contactConstraint.invMassA
		iA := contactConstraint.invIA
		mB := contactConstraint.invMassB
		iB := contactConstraint.invIB

		stateA := indexA == NULL_INDEX ? &dummyState : &states[indexA]
		vA := stateA.linearVelocity
		wA := stateA.angularVelocity
		dqA := stateA.deltaRotation

		stateB := indexB == NULL_INDEX ? &dummyState : &states[indexB]
		vB := stateB.linearVelocity
		wB := stateB.angularVelocity
		dqB := stateB.deltaRotation

		dp := Sub(stateB.deltaPosition, stateA.deltaPosition)
		friction := contactConstraint.friction
		rollingResistance := contactConstraint.rollingResistance

		for j: i32 = 0; j < manifoldCount; j += 1 {
			constraint := &contactConstraint.constraints[j]

			pointCount := constraint.pointCount
			normal := constraint.normal

			totalNormalImpulse: f32 = 0.0
			totalTwistLimit: f32 = 0.0

			for pointIndex: i32 = 0; pointIndex < pointCount; pointIndex += 1 {
				cp := &constraint.points[pointIndex]

				// Fixed anchor points for applying impulses
				rA := cp.rA
				rB := cp.rB

				// compute current separation
				// this is subject to round-off error if the anchor is far from the body center of mass
				ds := Add(dp, Sub(RotateVector(dqB, rB), RotateVector(dqA, rA)))
				s := Dot(ds, normal) + cp.baseSeparation

				// speculative bias, zero when overlapped
				velocityBias: f32 = s > 0.0 ? s * inv_h : 0.0

				// relative normal velocity at contact
				vrA := Add(vA, Cross(wA, rA))
				vrB := Add(vB, Cross(wB, rB))
				vn := Dot(Sub(vrB, vrA), normal)

				// incremental normal impulse
				deltaImpulse := -cp.normalMass * (vn + velocityBias)

				// clamp the accumulated impulse
				newImpulse := MaxFloat(cp.normalImpulse + deltaImpulse, 0.0)
				deltaImpulse = newImpulse - cp.normalImpulse
				cp.normalImpulse = newImpulse
				cp.totalNormalImpulse += newImpulse
				totalNormalImpulse += newImpulse
				totalTwistLimit += cp.leverArm * newImpulse

				// apply normal impulse
				P := MulSV(deltaImpulse, normal)
				vA = MulSub(vA, mA, P)
				wA = Sub(wA, MulMV(iA, Cross(rA, P)))

				vB = MulAdd(vB, mB, P)
				wB = Add(wB, MulMV(iB, Cross(rB, P)))
			}

			// Central twist friction
			{
				twistSpeed := Dot(constraint.normal, Sub(wB, wA))
				maxImpulse := friction * totalTwistLimit
				deltaImpulse := -constraint.twistMass * twistSpeed
				oldImpulse := constraint.twistImpulse
				constraint.twistImpulse = ClampFloat(oldImpulse + deltaImpulse, -maxImpulse, maxImpulse)
				deltaImpulse = constraint.twistImpulse - oldImpulse

				wA = Sub(wA, MulMV(iA, MulSV(deltaImpulse, constraint.normal)))
				wB = Add(wB, MulMV(iB, MulSV(deltaImpulse, constraint.normal)))
			}

			// Rolling resistance
			if rollingResistance > 0.0 {
				deltaImpulse := Neg(MulMV(contactConstraint.rollingMass, Sub(wB, wA)))
				oldImpulse := constraint.rollingImpulse
				constraint.rollingImpulse = Add(oldImpulse, deltaImpulse)

				maxImpulse := rollingResistance * totalNormalImpulse
				magSqr := Dot(constraint.rollingImpulse, constraint.rollingImpulse)
				if magSqr > maxImpulse * maxImpulse + FLT_EPSILON {
					constraint.rollingImpulse = MulSV(maxImpulse / intrinsics.sqrt(magSqr), constraint.rollingImpulse)
				}

				deltaImpulse = Sub(constraint.rollingImpulse, oldImpulse)

				wA = Sub(wA, MulMV(iA, deltaImpulse))
				wB = Add(wB, MulMV(iB, deltaImpulse))
			}

			// Central friction
			{
				tangent1 := constraint.tangent1
				tangent2 := constraint.tangent2

				// Fixed anchor points for applying impulses
				rA := constraint.centerA
				rB := constraint.centerB

				// Relative tangent velocity at contact
				vrA := Add(vA, Cross(wA, rA))
				vrB := Add(vB, Cross(wB, rB))
				vr := Sub(vrB, vrA)
				vt := Vec2 {
					Dot(vr, tangent1) - constraint.tangentVelocity1,
					Dot(vr, tangent2) - constraint.tangentVelocity2,
				}

				// Incremental tangent impulse
				tm := MulMV2(constraint.tangentMass, vt)
				deltaImpulse := Vec2{-tm.x, -tm.y}
				newImpulse := Vec2 {
					constraint.frictionImpulse.x + deltaImpulse.x,
					constraint.frictionImpulse.y + deltaImpulse.y,
				}

				maxImpulse := friction * totalNormalImpulse

				// Clamp the accumulated impulse
				lengthSquared := Dot2(newImpulse, newImpulse)
				if lengthSquared > maxImpulse * maxImpulse {
					scale := maxImpulse / intrinsics.sqrt(lengthSquared)
					newImpulse.x *= scale
					newImpulse.y *= scale
				}
				deltaImpulse = Sub2(newImpulse, constraint.frictionImpulse)
				constraint.frictionImpulse = newImpulse

				// Apply delta impulse
				P := Blend2(deltaImpulse.x, tangent1, deltaImpulse.y, tangent2)
				vA = MulSub(vA, mA, P)
				wA = Sub(wA, MulMV(iA, Cross(rA, P)))
				vB = MulAdd(vB, mB, P)
				wB = Add(wB, MulMV(iB, Cross(rB, P)))
			}
		}

		if (stateA.flags & dynamicFlag) != 0 {
			stateA.linearVelocity = vA
			stateA.angularVelocity = wA
		}

		if (stateB.flags & dynamicFlag) != 0 {
			stateB.linearVelocity = vB
			stateB.angularVelocity = wB
		}
	}
}

ApplyRestitution_Mesh :: proc(block: SolverBlock, context_: ^StepContext) {
	world := context_.world
	color := &world.constraintGraph.colors[block.colorIndex]
	contactConstraints := color.contactConstraints
	states := context_.states

	threshold := world.restitutionThreshold
	inv_h := context_.inv_h
	propagate := world.enableRestitutionPropagation

	dummyState := identityBodyState

	startIndex := block.startIndex
	endIndex := startIndex + i32(block.count)

	for i := startIndex; i < endIndex; i += 1 {
		contactConstraint := &contactConstraints[i]
		restitution := contactConstraint.restitution
		if propagate == false && restitution == 0.0 {
			continue
		}

		manifoldCount := contactConstraint.manifoldCount

		indexA := contactConstraint.indexA
		indexB := contactConstraint.indexB

		mA := contactConstraint.invMassA
		iA := contactConstraint.invIA
		mB := contactConstraint.invMassB
		iB := contactConstraint.invIB

		stateA := indexA == NULL_INDEX ? &dummyState : &states[indexA]
		vA := stateA.linearVelocity
		wA := stateA.angularVelocity
		dqA := stateA.deltaRotation

		stateB := indexB == NULL_INDEX ? &dummyState : &states[indexB]
		vB := stateB.linearVelocity
		wB := stateB.angularVelocity
		dqB := stateB.deltaRotation

		dp := Sub(stateB.deltaPosition, stateA.deltaPosition)

		for j: i32 = 0; j < manifoldCount; j += 1 {
			constraint := &contactConstraint.constraints[j]

			pointCount := constraint.pointCount
			normal := constraint.normal

			for pointIndex: i32 = 0; pointIndex < pointCount; pointIndex += 1 {
				cp := &constraint.points[pointIndex]

				rA := cp.rA
				rB := cp.rB

				// The total normal impulse is 0 for speculative points.
				compressionImpulse := cp.totalNormalImpulse - cp.restitutionImpulse
				armed := restitution > 0.0 && cp.relativeVelocity < -threshold && compressionImpulse > 0.0

				velocityBias: f32
				if armed {
					velocityBias = restitution * cp.relativeVelocity
				} else {
					ds := Add(dp, Sub(RotateVector(dqB, rB), RotateVector(dqA, rA)))
					s := Dot(ds, normal) + cp.baseSeparation

					velocityBias = s > 0.0 ? s * inv_h : 0.0
				}

				vrA := Add(vA, Cross(wA, rA))
				vrB := Add(vB, Cross(wB, rB))
				vn := Dot(Sub(vrB, vrA), normal)

				impulse := -cp.normalMass * (vn + velocityBias)

				newImpulse := MaxFloat(cp.normalImpulse + impulse, 0.0)
				impulse = newImpulse - cp.normalImpulse

				approachImpulse := MinFloat(MaxFloat(-cp.normalMass * vn, 0.0), MaxFloat(impulse, 0.0))

				if armed {
					// Poisson kinetic restitution guarantees no energy gain.
					allowance := restitution * (compressionImpulse + approachImpulse) - cp.restitutionImpulse
					impulse = MinFloat(impulse, approachImpulse + MaxFloat(allowance, 0.0))
				}

				cp.normalImpulse += impulse
				cp.restitutionImpulse += impulse - approachImpulse
				cp.totalNormalImpulse += impulse

				P := MulSV(impulse, normal)
				vA = MulSub(vA, mA, P)
				wA = Sub(wA, MulMV(iA, Cross(rA, P)))

				vB = MulAdd(vB, mB, P)
				wB = Add(wB, MulMV(iB, Cross(rB, P)))
			}
		}

		if (stateA.flags & dynamicFlag) != 0 {
			stateA.linearVelocity = vA
			stateA.angularVelocity = wA
		}

		if (stateB.flags & dynamicFlag) != 0 {
			stateB.linearVelocity = vB
			stateB.angularVelocity = wB
		}
	}
}

// Don't need to use spans for colors for this because the constraint to contact association
// is already linked by pointer.
StoreImpulses_Mesh :: proc(block: SolverBlock, context_: ^StepContext, workerIndex: i32) {
	world := context_.world

	assert(block.blockType == u8(SolverBlockType.overflowBlock))
	overflow := &world.constraintGraph.colors[OVERFLOW_INDEX]
	spans := context_.overflowSpans
	base := overflow.contactConstraints

	taskContext := &world.taskContexts.data[workerIndex]
	hitEventBitSet := &taskContext.hitEventBitSet
	hasHitEvents := taskContext.hasHitEvents
	negHitThreshold := -world.hitEventThreshold

	index := block.startIndex
	endIndex := block.startIndex + i32(block.count)

	// Find color for start index. Linear search but fast.
	colorIndex: i32 = 0
	for spans[colorIndex + 1].start <= index {
		colorIndex += 1
	}

	// Loop over block
	for index < endIndex {
		colorStart := spans[colorIndex].start
		colorEndIndex := MinInt(spans[colorIndex + 1].start, endIndex)

		// Loop over color
		for ; index < colorEndIndex; index += 1 {
			contactConstraint := &base[index]

			localIndex := index - colorStart
			_ = localIndex
			assert(0 <= localIndex && localIndex < spans[colorIndex].count)

			// Having this contact pointer simplifies impulse storage
			contact := contactConstraint.contact
			assert(contact != nil)

			// Catches the wrong-(base, spans) pairing: the contact pointer stashed by
			// b3PrepareContacts_Mesh at this flat slot must reference the same contact
			// the span at this slot describes.
			VALIDATE(contact.contactId == spans[colorIndex].contacts[localIndex].contactId)

			manifoldCount := contactConstraint.manifoldCount
			assert(manifoldCount == contact.manifoldCount)

			checkHitEvents := (contact.flags & simEnableHitEvent) != 0
			flagged := false

			for manifoldIndex: i32 = 0; manifoldIndex < manifoldCount; manifoldIndex += 1 {
				manifold := &contact.manifolds[manifoldIndex]
				constraint := &contactConstraint.constraints[manifoldIndex]
				manifold.twistImpulse = constraint.twistImpulse
				manifold.frictionImpulse = Blend2(
					constraint.frictionImpulse.x,
					constraint.tangent1,
					constraint.frictionImpulse.y,
					constraint.tangent2,
				)
				manifold.rollingImpulse = constraint.rollingImpulse

				count := constraint.pointCount
				assert(count == manifold.pointCount)
				for pointIndex: i32 = 0; pointIndex < count; pointIndex += 1 {
					cp := &constraint.points[pointIndex]
					mp := &manifold.points[pointIndex]
					mp.normalImpulse = cp.normalImpulse
					mp.totalNormalImpulse = cp.totalNormalImpulse

					if checkHitEvents &&
					   flagged == false &&
					   mp.normalVelocity < negHitThreshold &&
					   mp.totalNormalImpulse > 0.0 {
						SetBit(hitEventBitSet, u32(contact.contactId))
						hasHitEvents = true
						flagged = true
					}
				}
			}
		}

		// Advance to next color
		colorIndex += 1
	}

	taskContext.hasHitEvents = hasHitEvents
}

PrepareContacts_Overflow :: proc(context_: ^StepContext) {
	graph := context_.graph
	color := &graph.colors[OVERFLOW_INDEX]

	assert(color.contacts.count <= i32(max(u16)))
	count := u16(color.contacts.count)
	if count == 0 {
		return
	}

	block := SolverBlock {
		startIndex = 0,
		count      = count,
		blockType  = u8(SolverBlockType.overflowBlock),
		colorIndex = OVERFLOW_INDEX,
	}

	PrepareContacts_Mesh(block, context_)
}

WarmStartContacts_Overflow :: proc(context_: ^StepContext) {
	graph := context_.graph
	color := &graph.colors[OVERFLOW_INDEX]

	count := u16(color.contacts.count)
	if count == 0 {
		return
	}

	block := SolverBlock {
		startIndex = 0,
		count      = count,
		blockType  = u8(SolverBlockType.overflowBlock),
		colorIndex = OVERFLOW_INDEX,
	}

	WarmStartContacts_Mesh(block, context_)
}

PushContacts_Overflow :: proc(context_: ^StepContext) {
	graph := context_.graph
	color := &graph.colors[OVERFLOW_INDEX]

	count := u16(color.contacts.count)
	if count == 0 {
		return
	}

	block := SolverBlock {
		startIndex = 0,
		count      = count,
		blockType  = u8(SolverBlockType.overflowBlock),
		colorIndex = OVERFLOW_INDEX,
	}

	PushContacts_Mesh(block, context_)
}

SolveContacts_Overflow :: proc(context_: ^StepContext) {
	graph := context_.graph
	color := &graph.colors[OVERFLOW_INDEX]

	count := u16(color.contacts.count)
	if count == 0 {
		return
	}

	block := SolverBlock {
		startIndex = 0,
		count      = count,
		blockType  = u8(SolverBlockType.overflowBlock),
		colorIndex = OVERFLOW_INDEX,
	}

	SolveContacts_Mesh(block, context_)
}

ApplyRestitution_Overflow :: proc(context_: ^StepContext) {
	graph := context_.graph
	color := &graph.colors[OVERFLOW_INDEX]

	count := u16(color.contacts.count)
	if count == 0 {
		return
	}

	block := SolverBlock {
		startIndex = 0,
		count      = count,
		blockType  = u8(SolverBlockType.overflowBlock),
		colorIndex = OVERFLOW_INDEX,
	}

	ApplyRestitution_Mesh(block, context_)
}

StoreImpulses_Overflow :: proc(context_: ^StepContext) {
	graph := context_.graph
	color := &graph.colors[OVERFLOW_INDEX]

	count := u16(color.contacts.count)
	if count == 0 {
		return
	}

	block := SolverBlock {
		startIndex = 0,
		count      = count,
		blockType  = u8(SolverBlockType.overflowBlock),
		colorIndex = OVERFLOW_INDEX,
	}

	StoreImpulses_Mesh(block, context_, 0)
}

PrepareContacts_Convex :: proc(block: SolverBlock, context_: ^StepContext) {
	when SIMD_HAS_WIDTH_8 {
		if context_.world.simdWidth == 8 {
			PrepareContacts_ConvexW8(block, context_)
			return
		}
	}

	PrepareContacts_ConvexW4(block, context_)
}

WarmStartContacts_Convex :: proc(block: SolverBlock, context_: ^StepContext) {
	when SIMD_HAS_WIDTH_8 {
		if context_.world.simdWidth == 8 {
			WarmStartContacts_ConvexW8(block, context_)
			return
		}
	}

	WarmStartContacts_ConvexW4(block, context_)
}

PushContacts_Convex :: proc(block: SolverBlock, context_: ^StepContext) {
	when SIMD_HAS_WIDTH_8 {
		if context_.world.simdWidth == 8 {
			PushContacts_ConvexW8(block, context_)
			return
		}
	}

	PushContacts_ConvexW4(block, context_)
}

SolveContacts_Convex :: proc(block: SolverBlock, context_: ^StepContext) {
	when SIMD_HAS_WIDTH_8 {
		if context_.world.simdWidth == 8 {
			SolveContacts_ConvexW8(block, context_)
			return
		}
	}

	SolveContacts_ConvexW4(block, context_)
}

ApplyRestitution_Convex :: proc(block: SolverBlock, context_: ^StepContext) {
	when SIMD_HAS_WIDTH_8 {
		if context_.world.simdWidth == 8 {
			ApplyRestitution_ConvexW8(block, context_)
			return
		}
	}

	ApplyRestitution_ConvexW4(block, context_)
}

StoreImpulses_Convex :: proc(block: SolverBlock, context_: ^StepContext, workerIndex: i32) {
	when SIMD_HAS_WIDTH_8 {
		if context_.world.simdWidth == 8 {
			StoreImpulses_ConvexW8(block, context_, workerIndex)
			return
		}
	}

	StoreImpulses_ConvexW4(block, context_, workerIndex)
}

GetWideContactConstraintByteCount :: proc(simdWidth: i32) -> i32 {
	when SIMD_HAS_WIDTH_8 {
		if simdWidth == 8 {
			return GetWideContactConstraintByteCountW8()
		}
	} else {
		_ = simdWidth
	}

	return GetWideContactConstraintByteCountW4()
}

PrepareContacts_MeshWide :: proc(block: SolverBlock, context_: ^StepContext) {
	when SIMD_HAS_WIDTH_8 {
		if context_.world.simdWidth == 8 {
			PrepareContacts_MeshWideW8(block, context_)
			return
		}
	}

	PrepareContacts_MeshWideW4(block, context_)
}

WarmStartContacts_MeshWide :: proc(block: SolverBlock, context_: ^StepContext) {
	when SIMD_HAS_WIDTH_8 {
		if context_.world.simdWidth == 8 {
			WarmStartContacts_MeshWideW8(block, context_)
			return
		}
	}

	WarmStartContacts_MeshWideW4(block, context_)
}

PushContacts_MeshWide :: proc(block: SolverBlock, context_: ^StepContext) {
	when SIMD_HAS_WIDTH_8 {
		if context_.world.simdWidth == 8 {
			PushContacts_MeshWideW8(block, context_)
			return
		}
	}

	PushContacts_MeshWideW4(block, context_)
}

SolveContacts_MeshWide :: proc(block: SolverBlock, context_: ^StepContext) {
	when SIMD_HAS_WIDTH_8 {
		if context_.world.simdWidth == 8 {
			SolveContacts_MeshWideW8(block, context_)
			return
		}
	}

	SolveContacts_MeshWideW4(block, context_)
}

ApplyRestitution_MeshWide :: proc(block: SolverBlock, context_: ^StepContext) {
	when SIMD_HAS_WIDTH_8 {
		if context_.world.simdWidth == 8 {
			ApplyRestitution_MeshWideW8(block, context_)
			return
		}
	}

	ApplyRestitution_MeshWideW4(block, context_)
}

StoreImpulses_MeshWide :: proc(block: SolverBlock, context_: ^StepContext, workerIndex: i32) {
	when SIMD_HAS_WIDTH_8 {
		if context_.world.simdWidth == 8 {
			StoreImpulses_MeshWideW8(block, context_, workerIndex)
			return
		}
	}

	StoreImpulses_MeshWideW4(block, context_, workerIndex)
}

GetWideMeshConstraintByteCount :: proc(simdWidth: i32) -> i32 {
	when SIMD_HAS_WIDTH_8 {
		if simdWidth == 8 {
			return GetWideMeshConstraintByteCountW8()
		}
	} else {
		_ = simdWidth
	}

	return GetWideMeshConstraintByteCountW4()
}

GetWideMeshManifoldByteCount :: proc(simdWidth: i32) -> i32 {
	when SIMD_HAS_WIDTH_8 {
		if simdWidth == 8 {
			return GetWideMeshManifoldByteCountW8()
		}
	} else {
		_ = simdWidth
	}

	return GetWideMeshManifoldByteCountW4()
}
