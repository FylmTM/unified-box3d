// Ported from box3d src/motor_joint.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

// B3_REC is expanded inline at each call site (`if world.recording != nil {...}`), as in body.odin.

MotorJoint_SetLinearVelocity :: proc(jointId: JointId, velocity: Vec3) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_MotorJointSetLinearVelocity{jointId, velocity}
		RecWrite_MotorJointSetLinearVelocity(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .motorJoint)
	joint.motorJoint.linearVelocity = velocity
}

MotorJoint_GetLinearVelocity :: proc(jointId: JointId) -> Vec3 {
	joint := GetJointSimCheckType(jointId, .motorJoint)
	return joint.motorJoint.linearVelocity
}

MotorJoint_SetAngularVelocity :: proc(jointId: JointId, velocity: Vec3) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_MotorJointSetAngularVelocity{jointId, velocity}
		RecWrite_MotorJointSetAngularVelocity(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .motorJoint)
	joint.motorJoint.angularVelocity = velocity
}

MotorJoint_GetAngularVelocity :: proc(jointId: JointId) -> Vec3 {
	joint := GetJointSimCheckType(jointId, .motorJoint)
	return joint.motorJoint.angularVelocity
}

MotorJoint_SetMaxVelocityTorque :: proc(jointId: JointId, maxTorque: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_MotorJointSetMaxVelocityTorque{jointId, maxTorque}
		RecWrite_MotorJointSetMaxVelocityTorque(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .motorJoint)
	joint.motorJoint.maxVelocityTorque = maxTorque
}

MotorJoint_GetMaxVelocityTorque :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .motorJoint)
	return joint.motorJoint.maxVelocityTorque
}

MotorJoint_SetMaxVelocityForce :: proc(jointId: JointId, maxForce: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_MotorJointSetMaxVelocityForce{jointId, maxForce}
		RecWrite_MotorJointSetMaxVelocityForce(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .motorJoint)
	joint.motorJoint.maxVelocityForce = maxForce
}

MotorJoint_GetMaxVelocityForce :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .motorJoint)
	return joint.motorJoint.maxVelocityForce
}

MotorJoint_SetLinearHertz :: proc(jointId: JointId, hertz: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_MotorJointSetLinearHertz{jointId, hertz}
		RecWrite_MotorJointSetLinearHertz(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .motorJoint)
	joint.motorJoint.linearHertz = hertz
}

MotorJoint_GetLinearHertz :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .motorJoint)
	return joint.motorJoint.linearHertz
}

MotorJoint_SetLinearDampingRatio :: proc(jointId: JointId, damping: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_MotorJointSetLinearDampingRatio{jointId, damping}
		RecWrite_MotorJointSetLinearDampingRatio(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .motorJoint)
	joint.motorJoint.linearDampingRatio = damping
}

MotorJoint_GetLinearDampingRatio :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .motorJoint)
	return joint.motorJoint.linearDampingRatio
}

MotorJoint_SetAngularHertz :: proc(jointId: JointId, hertz: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_MotorJointSetAngularHertz{jointId, hertz}
		RecWrite_MotorJointSetAngularHertz(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .motorJoint)
	joint.motorJoint.angularHertz = hertz
}

MotorJoint_GetAngularHertz :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .motorJoint)
	return joint.motorJoint.angularHertz
}

MotorJoint_SetAngularDampingRatio :: proc(jointId: JointId, damping: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_MotorJointSetAngularDampingRatio{jointId, damping}
		RecWrite_MotorJointSetAngularDampingRatio(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .motorJoint)
	joint.motorJoint.angularDampingRatio = damping
}

MotorJoint_GetAngularDampingRatio :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .motorJoint)
	return joint.motorJoint.angularDampingRatio
}

MotorJoint_SetMaxSpringForce :: proc(jointId: JointId, maxForce: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_MotorJointSetMaxSpringForce{jointId, maxForce}
		RecWrite_MotorJointSetMaxSpringForce(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .motorJoint)
	joint.motorJoint.maxSpringForce = MaxFloat(0.0, maxForce)
}

MotorJoint_GetMaxSpringForce :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .motorJoint)
	return joint.motorJoint.maxSpringForce
}

MotorJoint_SetMaxSpringTorque :: proc(jointId: JointId, maxTorque: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_MotorJointSetMaxSpringTorque{jointId, maxTorque}
		RecWrite_MotorJointSetMaxSpringTorque(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .motorJoint)
	joint.motorJoint.maxSpringTorque = MaxFloat(0.0, maxTorque)
}

MotorJoint_GetMaxSpringTorque :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .motorJoint)
	return joint.motorJoint.maxSpringTorque
}

GetMotorJointForce :: proc(world: ^World, base: ^JointSim) -> Vec3 {
	force := MulSV(world.inv_h, Add(base.motorJoint.linearVelocityImpulse, base.motorJoint.linearSpringImpulse))
	return force
}

GetMotorJointTorque :: proc(world: ^World, base: ^JointSim) -> Vec3 {
	return MulSV(world.inv_h, Add(base.motorJoint.angularVelocityImpulse, base.motorJoint.angularSpringImpulse))
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

PrepareMotorJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .motorJoint)

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

	base.invMassA = bodySimA.invMass
	base.invMassB = bodySimB.invMass
	base.invIA = bodySimA.invInertiaWorld
	base.invIB = bodySimB.invInertiaWorld

	invInertiaSum := AddMM(base.invIA, base.invIB)
	base.fixedRotation = Det(invInertiaSum) < f32(1000.0) * FLT_MIN

	joint := &base.motorJoint
	joint.indexA = bodyA.setIndex == i32(SetType.awakeSet) ? localIndexA : NULL_INDEX
	joint.indexB = bodyB.setIndex == i32(SetType.awakeSet) ? localIndexB : NULL_INDEX

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.q = MulQuat(bodySimA.transform.q, base.localFrameA.q)
	joint.frameA.p = RotateVector(bodySimA.transform.q, Sub(base.localFrameA.p, bodySimA.localCenter))
	joint.frameB.q = MulQuat(bodySimB.transform.q, base.localFrameB.q)
	joint.frameB.p = RotateVector(bodySimB.transform.q, Sub(base.localFrameB.p, bodySimB.localCenter))

	// Compute the initial center delta. Incremental position updates are relative to this.
	joint.deltaCenter = SubPos(bodySimB.center, bodySimA.center)

	joint.linearSpring = MakeSoft(joint.linearHertz, joint.linearDampingRatio, context_.h)
	joint.angularSpring = MakeSoft(joint.angularHertz, joint.angularDampingRatio, context_.h)

	joint.angularMass = InvertMatrix(invInertiaSum)

	if context_.enableWarmStarting == false {
		joint.linearVelocityImpulse = Vec3_zero
		joint.angularVelocityImpulse = Vec3_zero
		joint.linearSpringImpulse = Vec3_zero
		joint.angularSpringImpulse = Vec3_zero
	}
}

WarmStartMotorJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .motorJoint)

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	joint := &base.motorJoint

	// dummy state for static bodies
	dummyState := identityBodyState

	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	rA := RotateVector(stateA.deltaRotation, joint.frameA.p)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.p)

	linearImpulse := Add(joint.linearVelocityImpulse, joint.linearSpringImpulse)
	angularImpulse := Add(joint.angularVelocityImpulse, joint.angularSpringImpulse)

	stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, linearImpulse)
	stateA.angularVelocity = Sub(stateA.angularVelocity, MulMV(iA, Add(Cross(rA, linearImpulse), angularImpulse)))
	stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, linearImpulse)
	stateB.angularVelocity = Add(stateB.angularVelocity, MulMV(iB, Add(Cross(rB, linearImpulse), angularImpulse)))
}

SolveMotorJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .motorJoint)

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.motorJoint
	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity

	quatA := MulQuat(stateA.deltaRotation, joint.frameA.q)
	quatB := MulQuat(stateB.deltaRotation, joint.frameB.q)

	if DotQuat(quatA, quatB) < 0.0 {
		// this keeps the rotation angle in the range [-pi, pi]
		quatB = NegateQuat(quatB)
	}

	relQ := InvMulQuat(quatA, quatB)

	// angular spring
	if joint.maxSpringTorque > 0.0 && joint.angularHertz > 0.0 {
		targetQuat := Quat_identity
		deltaRotation := DeltaQuatToRotation(relQ, targetQuat)
		c := Neg(RotateVector(quatA, deltaRotation))

		bias := MulSV(joint.angularSpring.biasRate, c)
		massScale := joint.angularSpring.massScale
		impulseScale := joint.angularSpring.impulseScale

		cdot := Sub(wB, wA)

		maxImpulse := context_.h * joint.maxSpringTorque
		oldImpulse := joint.angularSpringImpulse
		impulse := MulSub(MulSV(-massScale, MulMV(joint.angularMass, Add(cdot, bias))), impulseScale, oldImpulse)
		joint.angularSpringImpulse = Add(oldImpulse, impulse)
		if LengthSquared(joint.angularSpringImpulse) > maxImpulse * maxImpulse {
			joint.angularSpringImpulse = MulSV(maxImpulse, Normalize(joint.angularSpringImpulse))
		}
		impulse = Sub(joint.angularSpringImpulse, oldImpulse)

		wA = Sub(wA, MulMV(iA, impulse))
		wB = Add(wB, MulMV(iB, impulse))
	}

	// angular velocity
	if joint.maxVelocityTorque > 0.0 {
		cdot := Sub(Sub(wB, wA), joint.angularVelocity)
		impulse := Neg(MulMV(joint.angularMass, cdot))

		maxImpulse := context_.h * joint.maxVelocityTorque
		oldImpulse := joint.angularVelocityImpulse
		joint.angularVelocityImpulse = Add(oldImpulse, impulse)
		if LengthSquared(joint.angularVelocityImpulse) > maxImpulse * maxImpulse {
			joint.angularVelocityImpulse = MulSV(maxImpulse, Normalize(joint.angularVelocityImpulse))
		}
		impulse = Sub(joint.angularVelocityImpulse, oldImpulse)

		wA = Sub(wA, MulMV(iA, impulse))
		wB = Add(wB, MulMV(iB, impulse))
	}

	rA := RotateVector(stateA.deltaRotation, joint.frameA.p)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.p)

	// linear spring
	if joint.maxSpringForce > 0.0 && joint.linearHertz > 0.0 {
		dcA := stateA.deltaPosition
		dcB := stateB.deltaPosition
		c := Add(Add(Sub(dcB, dcA), Sub(rB, rA)), joint.deltaCenter)

		bias := MulSV(joint.linearSpring.biasRate, c)
		massScale := joint.linearSpring.massScale
		impulseScale := joint.linearSpring.impulseScale

		cdot := Sub(Add(vB, Cross(wB, rB)), Add(vA, Cross(wA, rA)))

		b := SolvePointConstraint(mA, mB, base.invIA, base.invIB, rA, rB, Add(cdot, bias))

		oldImpulse := joint.linearSpringImpulse
		impulse := MulSub(MulSV(-massScale, b), impulseScale, oldImpulse)
		maxImpulse := context_.h * joint.maxSpringForce
		joint.linearSpringImpulse = Add(joint.linearSpringImpulse, impulse)

		if LengthSquared(joint.linearSpringImpulse) > maxImpulse * maxImpulse {
			joint.linearSpringImpulse = MulSV(maxImpulse, Normalize(joint.linearSpringImpulse))
		}

		impulse = Sub(joint.linearSpringImpulse, oldImpulse)

		vA = MulSub(vA, mA, impulse)
		wA = Sub(wA, MulMV(iA, Cross(rA, impulse)))
		vB = MulAdd(vB, mB, impulse)
		wB = Add(wB, MulMV(iB, Cross(rB, impulse)))
	}

	// linear velocity
	if joint.maxVelocityForce > 0.0 {
		cdot := Sub(Add(vB, Cross(wB, rB)), Add(vA, Cross(wA, rA)))
		cdot = Sub(cdot, joint.linearVelocity)

		b := SolvePointConstraint(mA, mB, base.invIA, base.invIB, rA, rB, cdot)
		impulse := Neg(b)

		oldImpulse := joint.linearVelocityImpulse
		maxImpulse := context_.h * joint.maxVelocityForce
		joint.linearVelocityImpulse = Add(joint.linearVelocityImpulse, impulse)

		if LengthSquared(joint.linearVelocityImpulse) > maxImpulse * maxImpulse {
			joint.linearVelocityImpulse = MulSV(maxImpulse, Normalize(joint.linearVelocityImpulse))
		}

		impulse = Sub(joint.linearVelocityImpulse, oldImpulse)

		vA = MulSub(vA, mA, impulse)
		wA = Sub(wA, MulMV(iA, Cross(rA, impulse)))
		vB = MulAdd(vB, mB, impulse)
		wB = Add(wB, MulMV(iB, Cross(rB, impulse)))
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
