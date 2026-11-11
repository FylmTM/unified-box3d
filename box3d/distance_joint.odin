// Ported from box3d src/distance_joint.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

// B3_REC is expanded inline at each call site (`if world.recording != nil {...}`), as in body.odin.

DistanceJoint_SetLength :: proc(jointId: JointId, length: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_DistanceJointSetLength{jointId, length}
		RecWrite_DistanceJointSetLength(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .distanceJoint)
	joint := &base.distanceJoint

	joint.length = ClampFloat(length, LINEAR_SLOP, HUGE)
	joint.impulse = 0.0
	joint.lowerImpulse = 0.0
	joint.upperImpulse = 0.0
}

DistanceJoint_GetLength :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .distanceJoint)
	joint := &base.distanceJoint
	return joint.length
}

DistanceJoint_EnableLimit :: proc(jointId: JointId, enableLimit: bool) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_DistanceJointEnableLimit{jointId, enableLimit}
		RecWrite_DistanceJointEnableLimit(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .distanceJoint)
	joint := &base.distanceJoint
	joint.enableLimit = enableLimit
}

DistanceJoint_IsLimitEnabled :: proc(jointId: JointId) -> bool {
	joint := GetJointSimCheckType(jointId, .distanceJoint)
	return joint.distanceJoint.enableLimit
}

DistanceJoint_SetLengthRange :: proc(jointId: JointId, minLength: f32, maxLength: f32) {
	minLength, maxLength := minLength, maxLength
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_DistanceJointSetLengthRange{jointId, minLength, maxLength}
		RecWrite_DistanceJointSetLengthRange(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .distanceJoint)
	joint := &base.distanceJoint

	minLength = ClampFloat(minLength, LINEAR_SLOP, HUGE)
	maxLength = ClampFloat(maxLength, LINEAR_SLOP, HUGE)
	joint.minLength = MinFloat(minLength, maxLength)
	joint.maxLength = MaxFloat(minLength, maxLength)
	joint.impulse = 0.0
	joint.lowerImpulse = 0.0
	joint.upperImpulse = 0.0
}

DistanceJoint_GetMinLength :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .distanceJoint)
	joint := &base.distanceJoint
	return joint.minLength
}

DistanceJoint_GetMaxLength :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .distanceJoint)
	joint := &base.distanceJoint
	return joint.maxLength
}

DistanceJoint_GetCurrentLength :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .distanceJoint)

	world := GetUnlockedWorld(i32(jointId.world0))
	if world == nil {
		return 0.0
	}

	transformA := GetBodyTransform(world, base.bodyIdA)
	transformB := GetBodyTransform(world, base.bodyIdB)

	pA := TransformWorldPoint(transformA, base.localFrameA.p)
	pB := TransformWorldPoint(transformB, base.localFrameB.p)
	d := SubPos(pB, pA)
	length := Length(d)
	return length
}

DistanceJoint_EnableSpring :: proc(jointId: JointId, enableSpring: bool) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_DistanceJointEnableSpring{jointId, enableSpring}
		RecWrite_DistanceJointEnableSpring(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .distanceJoint)
	base.distanceJoint.enableSpring = enableSpring
}

DistanceJoint_IsSpringEnabled :: proc(jointId: JointId) -> bool {
	base := GetJointSimCheckType(jointId, .distanceJoint)
	return base.distanceJoint.enableSpring
}


DistanceJoint_SetSpringForceRange :: proc(jointId: JointId, lowerForce: f32, upperForce: f32) {
	assert(lowerForce <= upperForce)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_DistanceJointSetSpringForceRange{jointId, lowerForce, upperForce}
		RecWrite_DistanceJointSetSpringForceRange(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .distanceJoint)
	base.distanceJoint.lowerSpringForce = lowerForce
	base.distanceJoint.upperSpringForce = upperForce
}

DistanceJoint_GetSpringForceRange :: proc(jointId: JointId, lowerForce: ^f32, upperForce: ^f32) {
	base := GetJointSimCheckType(jointId, .distanceJoint)
	lowerForce^ = base.distanceJoint.lowerSpringForce
	upperForce^ = base.distanceJoint.upperSpringForce
}

DistanceJoint_SetSpringHertz :: proc(jointId: JointId, hertz: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_DistanceJointSetSpringHertz{jointId, hertz}
		RecWrite_DistanceJointSetSpringHertz(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .distanceJoint)
	base.distanceJoint.hertz = hertz
}

DistanceJoint_SetSpringDampingRatio :: proc(jointId: JointId, dampingRatio: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_DistanceJointSetSpringDampingRatio{jointId, dampingRatio}
		RecWrite_DistanceJointSetSpringDampingRatio(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .distanceJoint)
	base.distanceJoint.dampingRatio = dampingRatio
}

DistanceJoint_GetSpringHertz :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .distanceJoint)
	joint := &base.distanceJoint
	return joint.hertz
}

DistanceJoint_GetSpringDampingRatio :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .distanceJoint)
	joint := &base.distanceJoint
	return joint.dampingRatio
}

DistanceJoint_EnableMotor :: proc(jointId: JointId, enableMotor: bool) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_DistanceJointEnableMotor{jointId, enableMotor}
		RecWrite_DistanceJointEnableMotor(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .distanceJoint)
	if enableMotor != joint.distanceJoint.enableMotor {
		joint.distanceJoint.enableMotor = enableMotor
		joint.distanceJoint.motorImpulse = 0.0
	}
}

DistanceJoint_IsMotorEnabled :: proc(jointId: JointId) -> bool {
	joint := GetJointSimCheckType(jointId, .distanceJoint)
	return joint.distanceJoint.enableMotor
}

DistanceJoint_SetMotorSpeed :: proc(jointId: JointId, motorSpeed: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_DistanceJointSetMotorSpeed{jointId, motorSpeed}
		RecWrite_DistanceJointSetMotorSpeed(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .distanceJoint)
	joint.distanceJoint.motorSpeed = motorSpeed
}

DistanceJoint_GetMotorSpeed :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .distanceJoint)
	return joint.distanceJoint.motorSpeed
}

DistanceJoint_GetMotorForce :: proc(jointId: JointId) -> f32 {
	world := GetWorld(i32(jointId.world0))
	base := GetJointSimCheckType(jointId, .distanceJoint)
	return world.inv_h * base.distanceJoint.motorImpulse
}

DistanceJoint_SetMaxMotorForce :: proc(jointId: JointId, force: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_DistanceJointSetMaxMotorForce{jointId, force}
		RecWrite_DistanceJointSetMaxMotorForce(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .distanceJoint)
	joint.distanceJoint.maxMotorForce = force
}

DistanceJoint_GetMaxMotorForce :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .distanceJoint)
	return joint.distanceJoint.maxMotorForce
}

GetDistanceJointForce :: proc(world: ^World, base: ^JointSim) -> Vec3 {
	joint := &base.distanceJoint

	transformA := GetBodyTransform(world, base.bodyIdA)
	transformB := GetBodyTransform(world, base.bodyIdB)

	pA := TransformWorldPoint(transformA, base.localFrameA.p)
	pB := TransformWorldPoint(transformB, base.localFrameB.p)
	d := SubPos(pB, pA)
	axis := Normalize(d)
	force := (joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse) * world.inv_h
	return MulSV(force, axis)
}

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k *

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

PrepareDistanceJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .distanceJoint)

	// chase body id to the solver set where the body lives
	idA := base.bodyIdA
	idB := base.bodyIdB

	world := context_.world
	bodyA := Array_Get(&world.bodies, idA)
	bodyB := Array_Get(&world.bodies, idB)

	assert(bodyA.setIndex == i32(SetType.awakeSet) || bodyB.setIndex == i32(SetType.awakeSet))

	setA := Array_Get(&world.solverSets, bodyA.setIndex)
	setB := Array_Get(&world.solverSets, bodyB.setIndex)

	localIndexA := bodyA.localIndex
	localIndexB := bodyB.localIndex

	bodySimA := Array_Get(&setA.bodySims, localIndexA)
	bodySimB := Array_Get(&setB.bodySims, localIndexB)

	mA := bodySimA.invMass
	iA := bodySimA.invInertiaWorld
	mB := bodySimB.invMass
	iB := bodySimB.invInertiaWorld

	base.invMassA = mA
	base.invMassB = mB
	base.invIA = iA
	base.invIB = iB

	joint := &base.distanceJoint

	joint.indexA = bodyA.setIndex == i32(SetType.awakeSet) ? localIndexA : NULL_INDEX
	joint.indexB = bodyB.setIndex == i32(SetType.awakeSet) ? localIndexB : NULL_INDEX

	// initial anchors in world space
	joint.anchorA = RotateVector(bodySimA.transform.q, Sub(base.localFrameA.p, bodySimA.localCenter))
	joint.anchorB = RotateVector(bodySimB.transform.q, Sub(base.localFrameB.p, bodySimB.localCenter))
	joint.deltaCenter = SubPos(bodySimB.center, bodySimA.center)

	rA := joint.anchorA
	rB := joint.anchorB
	separation := Add(Sub(rB, rA), joint.deltaCenter)
	axis := Normalize(separation)

	// compute effective mass
	crA := Cross(rA, axis)
	crB := Cross(rB, axis)
	k := mA + mB + Dot(crA, MulMV(iA, crA)) + Dot(crB, MulMV(iB, crB))
	joint.axialMass = k > 0.0 ? 1.0 / k : 0.0

	joint.distanceSoftness = MakeSoft(joint.hertz, joint.dampingRatio, context_.h)

	if context_.enableWarmStarting == false {
		joint.impulse = 0.0
		joint.lowerImpulse = 0.0
		joint.upperImpulse = 0.0
		joint.motorImpulse = 0.0
	}
}

WarmStartDistanceJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .distanceJoint)

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.distanceJoint
	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	rA := RotateVector(stateA.deltaRotation, joint.anchorA)
	rB := RotateVector(stateB.deltaRotation, joint.anchorB)

	ds := Add(Sub(stateB.deltaPosition, stateA.deltaPosition), Sub(rB, rA))
	separation := Add(joint.deltaCenter, ds)
	axis := Normalize(separation)

	axialImpulse := joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse
	P := MulSV(axialImpulse, axis)

	if (stateA.flags & dynamicFlag) != 0 {
		stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, P)
		stateA.angularVelocity = Sub(stateA.angularVelocity, MulMV(iA, Cross(rA, P)))
	}

	if (stateB.flags & dynamicFlag) != 0 {
		stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, P)
		stateB.angularVelocity = Add(stateB.angularVelocity, MulMV(iB, Cross(rB, P)))
	}
}

SolveDistanceJoint :: proc(base: ^JointSim, context_: ^StepContext, useBias: bool) {
	assert(base.type == .distanceJoint)

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.distanceJoint
	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity

	// current anchors
	rA := RotateVector(stateA.deltaRotation, joint.anchorA)
	rB := RotateVector(stateB.deltaRotation, joint.anchorB)

	// current separation
	ds := Add(Sub(stateB.deltaPosition, stateA.deltaPosition), Sub(rB, rA))
	separation := Add(joint.deltaCenter, ds)

	length := Length(separation)
	axis := Normalize(separation)

	// joint is soft if
	// - spring is enabled
	// - and (joint limit is disabled or limits are not equal)
	if joint.enableSpring && (joint.minLength < joint.maxLength || joint.enableLimit == false) {
		// spring
		if joint.hertz > 0.0 {
			// Cdot = dot(u, v + cross(w, r))
			vr := Add(Sub(vB, vA), Sub(Cross(wB, rB), Cross(wA, rA)))
			Cdot := Dot(axis, vr)
			C := length - joint.length
			bias := joint.distanceSoftness.biasRate * C

			m := joint.distanceSoftness.massScale * joint.axialMass
			oldImpulse := joint.impulse
			impulse := -m * (Cdot + bias) - joint.distanceSoftness.impulseScale * oldImpulse
			h := context_.h
			joint.impulse = ClampFloat(joint.impulse + impulse, joint.lowerSpringForce * h, joint.upperSpringForce * h)
			impulse = joint.impulse - oldImpulse

			P := MulSV(impulse, axis)
			vA = MulSub(vA, mA, P)
			wA = Sub(wA, MulMV(iA, Cross(rA, P)))
			vB = MulAdd(vB, mB, P)
			wB = Add(wB, MulMV(iB, Cross(rB, P)))
		}

		if joint.enableLimit {
			// lower limit
			{
				vr := Add(Sub(vB, vA), Sub(Cross(wB, rB), Cross(wA, rA)))
				Cdot := Dot(axis, vr)

				C := length - joint.minLength

				bias: f32 = 0.0
				massCoeff: f32 = 1.0
				impulseCoeff: f32 = 0.0
				if C > 0.0 {
					// speculative
					bias = C * context_.inv_h
				} else if useBias {
					bias = base.constraintSoftness.biasRate * C
					massCoeff = base.constraintSoftness.massScale
					impulseCoeff = base.constraintSoftness.impulseScale
				}

				impulse := -massCoeff * joint.axialMass * (Cdot + bias) - impulseCoeff * joint.lowerImpulse
				newImpulse := MaxFloat(0.0, joint.lowerImpulse + impulse)
				impulse = newImpulse - joint.lowerImpulse
				joint.lowerImpulse = newImpulse

				P := MulSV(impulse, axis)
				vA = MulSub(vA, mA, P)
				wA = Sub(wA, MulMV(iA, Cross(rA, P)))
				vB = MulAdd(vB, mB, P)
				wB = Add(wB, MulMV(iB, Cross(rB, P)))
			}

			// upper
			{
				vr := Add(Sub(vA, vB), Sub(Cross(wA, rA), Cross(wB, rB)))
				Cdot := Dot(axis, vr)

				C := joint.maxLength - length

				bias: f32 = 0.0
				massScale: f32 = 1.0
				impulseScale: f32 = 0.0
				if C > 0.0 {
					// speculative
					bias = C * context_.inv_h
				} else if useBias {
					bias = base.constraintSoftness.biasRate * C
					massScale = base.constraintSoftness.massScale
					impulseScale = base.constraintSoftness.impulseScale
				}

				impulse := -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.upperImpulse
				newImpulse := MaxFloat(0.0, joint.upperImpulse + impulse)
				impulse = newImpulse - joint.upperImpulse
				joint.upperImpulse = newImpulse

				P := MulSV(-impulse, axis)
				vA = MulSub(vA, mA, P)
				wA = Sub(wA, MulMV(iA, Cross(rA, P)))
				vB = MulAdd(vB, mB, P)
				wB = Add(wB, MulMV(iB, Cross(rB, P)))
			}
		}

		if joint.enableMotor {
			vr := Add(Sub(vB, vA), Sub(Cross(wB, rB), Cross(wA, rA)))
			Cdot := Dot(axis, vr)
			impulse := joint.axialMass * (joint.motorSpeed - Cdot)
			oldImpulse := joint.motorImpulse
			maxImpulse := context_.h * joint.maxMotorForce
			joint.motorImpulse = ClampFloat(joint.motorImpulse + impulse, -maxImpulse, maxImpulse)
			impulse = joint.motorImpulse - oldImpulse

			P := MulSV(impulse, axis)
			vA = MulSub(vA, mA, P)
			wA = Sub(wA, MulMV(iA, Cross(rA, P)))
			vB = MulAdd(vB, mB, P)
			wB = Add(wB, MulMV(iB, Cross(rB, P)))
		}
	} else {
		// rigid constraint
		vr := Add(Sub(vB, vA), Sub(Cross(wB, rB), Cross(wA, rA)))
		Cdot := Dot(axis, vr)

		C := length - joint.length

		bias: f32 = 0.0
		massScale: f32 = 1.0
		impulseScale: f32 = 0.0
		if useBias {
			bias = base.constraintSoftness.biasRate * C
			massScale = base.constraintSoftness.massScale
			impulseScale = base.constraintSoftness.impulseScale
		}

		impulse := -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.impulse
		joint.impulse += impulse

		P := MulSV(impulse, axis)
		vA = MulSub(vA, mA, P)
		wA = Sub(wA, MulMV(iA, Cross(rA, P)))
		vB = MulAdd(vB, mB, P)
		wB = Add(wB, MulMV(iB, Cross(rB, P)))
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

DrawDistanceJoint :: proc(draw: ^DebugDraw, base: ^JointSim, transformA: WorldTransform, transformB: WorldTransform) {
	assert(base.type == .distanceJoint)

	joint := &base.distanceJoint

	pA := TransformWorldPoint(transformA, base.localFrameA.p)
	pB := TransformWorldPoint(transformB, base.localFrameB.p)

	axis := Normalize(SubPos(pB, pA))

	if joint.minLength < joint.maxLength && joint.enableLimit {
		pMin := OffsetPos(pA, MulSV(joint.minLength, axis))
		pMax := OffsetPos(pA, MulSV(joint.maxLength, axis))

		if joint.minLength > LINEAR_SLOP {
			draw.DrawPointFcn(pMin, 6.0, .colorLightGreen, draw.context_)
		}

		if joint.maxLength < HUGE {
			draw.DrawPointFcn(pMax, 6.0, .colorRed, draw.context_)
		}

		if joint.minLength > LINEAR_SLOP && joint.maxLength < HUGE {
			draw.DrawSegmentFcn(pMin, pMax, .colorGray, draw.context_)
		}
	}

	draw.DrawSegmentFcn(pA, pB, .colorWhite, draw.context_)
	draw.DrawPointFcn(pA, 4.0, .colorWhite, draw.context_)
	draw.DrawPointFcn(pB, 4.0, .colorWhite, draw.context_)

	if joint.hertz > 0.0 && joint.enableSpring {
		pRest := OffsetPos(pA, MulSV(joint.length, axis))
		draw.DrawPointFcn(pRest, 4.0, .colorBlue, draw.context_)
	}
}
