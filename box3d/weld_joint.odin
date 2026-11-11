// Ported from box3d src/weld_joint.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

// B3_REC is expanded inline at each call site (`if world.recording != nil {...}`), as in body.odin.

WeldJoint_SetLinearHertz :: proc(jointId: JointId, hertz: f32) {
	assert(IsValidFloat(hertz) && hertz >= 0.0)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WeldJointSetLinearHertz{jointId, hertz}
		RecWrite_WeldJointSetLinearHertz(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .weldJoint)
	base.weldJoint.linearHertz = hertz
}

WeldJoint_GetLinearHertz :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .weldJoint)
	return base.weldJoint.linearHertz
}

WeldJoint_SetLinearDampingRatio :: proc(jointId: JointId, dampingRatio: f32) {
	assert(IsValidFloat(dampingRatio) && dampingRatio >= 0.0)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WeldJointSetLinearDampingRatio{jointId, dampingRatio}
		RecWrite_WeldJointSetLinearDampingRatio(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .weldJoint)
	base.weldJoint.linearDampingRatio = dampingRatio
}

WeldJoint_GetLinearDampingRatio :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .weldJoint)
	return base.weldJoint.linearDampingRatio
}

WeldJoint_SetAngularHertz :: proc(jointId: JointId, hertz: f32) {
	assert(IsValidFloat(hertz) && hertz >= 0.0)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WeldJointSetAngularHertz{jointId, hertz}
		RecWrite_WeldJointSetAngularHertz(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .weldJoint)
	base.weldJoint.angularHertz = hertz
}

WeldJoint_GetAngularHertz :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .weldJoint)
	return base.weldJoint.angularHertz
}

WeldJoint_SetAngularDampingRatio :: proc(jointId: JointId, dampingRatio: f32) {
	assert(IsValidFloat(dampingRatio) && dampingRatio >= 0.0)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WeldJointSetAngularDampingRatio{jointId, dampingRatio}
		RecWrite_WeldJointSetAngularDampingRatio(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .weldJoint)
	base.weldJoint.angularDampingRatio = dampingRatio
}

WeldJoint_GetAngularDampingRatio :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .weldJoint)
	return base.weldJoint.angularDampingRatio
}

GetWeldJointForce :: proc(world: ^World, base: ^JointSim) -> Vec3 {
	force := MulSV(world.inv_h, base.weldJoint.linearImpulse)
	return force
}

GetWeldJointTorque :: proc(world: ^World, base: ^JointSim) -> Vec3 {
	return MulSV(world.inv_h, base.weldJoint.angularImpulse)
}

PrepareWeldJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .weldJoint)

	world := context_.world

	bodyA := Array_Get(&world.bodies, base.bodyIdA)
	bodyB := Array_Get(&world.bodies, base.bodyIdB)

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

	joint := &base.weldJoint
	joint.indexA = bodyA.setIndex == i32(SetType.awakeSet) ? localIndexA : NULL_INDEX
	joint.indexB = bodyB.setIndex == i32(SetType.awakeSet) ? localIndexB : NULL_INDEX

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.q = MulQuat(bodySimA.transform.q, base.localFrameA.q)
	joint.frameA.p = RotateVector(bodySimA.transform.q, Sub(base.localFrameA.p, bodySimA.localCenter))
	joint.frameB.q = MulQuat(bodySimB.transform.q, base.localFrameB.q)
	joint.frameB.p = RotateVector(bodySimB.transform.q, Sub(base.localFrameB.p, bodySimB.localCenter))

	joint.deltaCenter = SubPos(bodySimB.center, bodySimA.center)
	joint.angularMass = InvertMatrix(invInertiaSum)

	if joint.linearHertz == 0.0 {
		joint.linearSpring = base.constraintSoftness
	} else {
		joint.linearSpring = MakeSoft(joint.linearHertz, joint.linearDampingRatio, context_.h)
	}

	if joint.angularHertz == 0.0 {
		joint.angularSpring = base.constraintSoftness
	} else {
		joint.angularSpring = MakeSoft(joint.angularHertz, joint.angularDampingRatio, context_.h)
	}

	if context_.enableWarmStarting == false {
		joint.linearImpulse = Vec3_zero
		joint.angularImpulse = Vec3_zero
	}
}

WarmStartWeldJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .weldJoint)

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.weldJoint

	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity

	rA := RotateVector(stateA.deltaRotation, joint.frameA.p)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.p)

	vA = MulSub(vA, mA, joint.linearImpulse)
	wA = Sub(wA, MulMV(iA, Add(Cross(rA, joint.linearImpulse), joint.angularImpulse)))

	vB = MulAdd(vB, mB, joint.linearImpulse)
	wB = Add(wB, MulMV(iB, Add(Cross(rB, joint.linearImpulse), joint.angularImpulse)))

	if (stateA.flags & dynamicFlag) != 0 {
		stateA.linearVelocity = vA
		stateA.angularVelocity = wA
	}

	if (stateB.flags & dynamicFlag) != 0 {
		stateB.linearVelocity = vB
		stateB.angularVelocity = wB
	}
}

SolveWeldJoint :: proc(base: ^JointSim, context_: ^StepContext, useBias: bool) {
	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.weldJoint
	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity

	fixedRotation := base.fixedRotation
	quatA := MulQuat(stateA.deltaRotation, joint.frameA.q)
	quatB := MulQuat(stateB.deltaRotation, joint.frameB.q)

	if DotQuat(quatA, quatB) < 0.0 {
		// this keeps the rotation angle in the range [-pi, pi]
		quatB = NegateQuat(quatB)
	}

	relQ := InvMulQuat(quatA, quatB)

	// angular constraint
	if fixedRotation == false {
		bias := Vec3_zero
		massScale: f32 = 1.0
		impulseScale: f32 = 0.0
		if useBias || joint.angularHertz > 0.0 {
			targetQuat := Quat_identity
			deltaRotation := DeltaQuatToRotation(relQ, targetQuat)
			c := Neg(RotateVector(quatA, deltaRotation))

			bias = MulSV(joint.angularSpring.biasRate, c)
			massScale = joint.angularSpring.massScale
			impulseScale = joint.angularSpring.impulseScale
		}

		cdot := Sub(wB, wA)
		impulse := MulSub(
			MulSV(-massScale, MulMV(joint.angularMass, Add(cdot, bias))),
			impulseScale,
			joint.angularImpulse,
		)
		joint.angularImpulse = Add(joint.angularImpulse, impulse)

		wA = Sub(wA, MulMV(iA, impulse))
		wB = Add(wB, MulMV(iB, impulse))
	}

	// linear constraint
	{
		rA := RotateVector(stateA.deltaRotation, joint.frameA.p)
		rB := RotateVector(stateB.deltaRotation, joint.frameB.p)

		cdot := Sub(Add(vB, Cross(wB, rB)), Add(vA, Cross(wA, rA)))

		bias := Vec3_zero
		massScale: f32 = 1.0
		impulseScale: f32 = 0.0
		if useBias || joint.linearHertz > 0.0 {
			dcA := stateA.deltaPosition
			dcB := stateB.deltaPosition

			separation := Add(Add(Sub(dcB, dcA), Sub(rB, rA)), joint.deltaCenter)

			bias = MulSV(joint.linearSpring.biasRate, separation)
			massScale = joint.linearSpring.massScale
			impulseScale = joint.linearSpring.impulseScale
		}

		b := SolvePointConstraint(mA, mB, base.invIA, base.invIB, rA, rB, Add(cdot, bias))

		impulse := MulSub(MulSV(-massScale, b), impulseScale, joint.linearImpulse)
		joint.linearImpulse = Add(joint.linearImpulse, impulse)

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

DrawWeldJoint :: proc(
	draw: ^DebugDraw,
	base: ^JointSim,
	transformA: WorldTransform,
	transformB: WorldTransform,
	scale: f32,
) {
	frameA := MulWorldTransforms(transformA, base.localFrameA)
	frameB := MulWorldTransforms(transformB, base.localFrameB)

	extents := Vec3{0.1 * scale, 0.05 * scale, 0.025 * scale}
	draw.DrawBoxFcn(extents, frameA, .colorDarkOrange, draw.context_)
	draw.DrawBoxFcn(extents, frameB, .colorDarkCyan, draw.context_)
}
