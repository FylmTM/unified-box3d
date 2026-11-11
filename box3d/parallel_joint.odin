// Ported from box3d src/parallel_joint.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

// B3_REC is expanded inline at each call site (`if world.recording != nil {...}`), as in body.odin.

ParallelJoint_SetSpringHertz :: proc(jointId: JointId, hertz: f32) {
	assert(IsValidFloat(hertz) && hertz >= 0.0)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_ParallelJointSetSpringHertz{jointId, hertz}
		RecWrite_ParallelJointSetSpringHertz(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .parallelJoint)
	base.parallelJoint.hertz = hertz
}

ParallelJoint_GetSpringHertz :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .parallelJoint)
	return base.parallelJoint.hertz
}

ParallelJoint_SetSpringDampingRatio :: proc(jointId: JointId, dampingRatio: f32) {
	assert(IsValidFloat(dampingRatio) && dampingRatio >= 0.0)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_ParallelJointSetSpringDampingRatio{jointId, dampingRatio}
		RecWrite_ParallelJointSetSpringDampingRatio(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .parallelJoint)
	base.parallelJoint.dampingRatio = dampingRatio
}

ParallelJoint_GetSpringDampingRatio :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .parallelJoint)
	return base.parallelJoint.dampingRatio
}

ParallelJoint_SetMaxTorque :: proc(jointId: JointId, maxForce: f32) {
	assert(IsValidFloat(maxForce) && maxForce >= 0.0)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_ParallelJointSetMaxTorque{jointId, maxForce}
		RecWrite_ParallelJointSetMaxTorque(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .parallelJoint)
	base.parallelJoint.maxTorque = maxForce
}

ParallelJoint_GetMaxTorque :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .parallelJoint)
	return base.parallelJoint.maxTorque
}

GetParallelJointTorque :: proc(world: ^World, base: ^JointSim) -> Vec3 {
	joint := &base.parallelJoint

	relQ := InvMulQuat(joint.quatA, joint.quatB)
	joint.perpAxisX = MulSV(0.5, RotateVector(joint.quatA, Add(MulSV(relQ.s, Vec3_axisX), Cross(relQ.v, Vec3_axisX))))
	joint.perpAxisY = MulSV(0.5, RotateVector(joint.quatA, Add(MulSV(relQ.s, Vec3_axisY), Cross(relQ.v, Vec3_axisY))))

	angularImpulse := Blend2(joint.perpImpulse.x, joint.perpAxisX, joint.perpImpulse.y, joint.perpAxisY)
	torque := MulSV(world.inv_h, angularImpulse)
	return torque
}

PrepareParallelJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .parallelJoint)

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

	joint := &base.parallelJoint
	joint.indexA = bodyA.setIndex == i32(SetType.awakeSet) ? localIndexA : NULL_INDEX
	joint.indexB = bodyB.setIndex == i32(SetType.awakeSet) ? localIndexB : NULL_INDEX

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.quatA = MulQuat(bodySimA.transform.q, base.localFrameA.q)
	joint.quatB = MulQuat(bodySimB.transform.q, base.localFrameB.q)

	relQ := InvMulQuat(joint.quatA, joint.quatB)

	{
		// These are needed for warm starting
		joint.perpAxisX = MulSV(
			0.5,
			RotateVector(joint.quatA, Add(MulSV(relQ.s, Vec3_axisX), Cross(relQ.v, Vec3_axisX))),
		)
		joint.perpAxisY = MulSV(
			0.5,
			RotateVector(joint.quatA, Add(MulSV(relQ.s, Vec3_axisY), Cross(relQ.v, Vec3_axisY))),
		)
	}

	joint.softness = MakeSoft(joint.hertz, joint.dampingRatio, context_.h)

	if context_.enableWarmStarting == false {
		joint.perpImpulse = Vec2{0.0, 0.0}
	}
}

WarmStartParallelJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .parallelJoint)

	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.parallelJoint

	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	wA := stateA.angularVelocity
	wB := stateB.angularVelocity

	angularImpulse := Blend2(joint.perpImpulse.x, joint.perpAxisX, joint.perpImpulse.y, joint.perpAxisY)

	wA = Sub(wA, MulMV(iA, angularImpulse))
	wB = Add(wB, MulMV(iB, angularImpulse))

	if (stateA.flags & dynamicFlag) != 0 {
		stateA.angularVelocity = wA
	}

	if (stateB.flags & dynamicFlag) != 0 {
		stateB.angularVelocity = wB
	}
}

SolveParallelJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.parallelJoint
	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	wA := stateA.angularVelocity
	wB := stateB.angularVelocity

	fixedRotation := base.fixedRotation
	quatA := MulQuat(stateA.deltaRotation, joint.quatA)
	quatB := MulQuat(stateB.deltaRotation, joint.quatB)

	if DotQuat(quatA, quatB) < 0.0 {
		// this keeps the rotation angle in the range [-pi, pi]
		quatB = NegateQuat(quatB)
	}

	relQ := InvMulQuat(quatA, quatB)

	if fixedRotation == false && joint.maxTorque > 0.0 {
		c := Vec2{relQ.v.x, relQ.v.y}
		bias := Vec2{joint.softness.biasRate * c.x, joint.softness.biasRate * c.y}
		massScale := joint.softness.massScale
		impulseScale := joint.softness.impulseScale

		// Collinearity constraint as 2-by-2
		perpAxisX := MulSV(0.5, RotateVector(quatA, Add(MulSV(relQ.s, Vec3_axisX), Cross(relQ.v, Vec3_axisX))))
		perpAxisY := MulSV(0.5, RotateVector(quatA, Add(MulSV(relQ.s, Vec3_axisY), Cross(relQ.v, Vec3_axisY))))
		joint.perpAxisX = perpAxisX
		joint.perpAxisY = perpAxisY

		invInertiaSum := AddMM(iA, iB)
		kxx := Dot(perpAxisX, MulMV(invInertiaSum, perpAxisX))
		kyy := Dot(perpAxisY, MulMV(invInertiaSum, perpAxisY))
		kxy := Dot(perpAxisX, MulMV(invInertiaSum, perpAxisY))

		k := Matrix2{{kxx, kxy}, {kxy, kyy}}

		wRel := Sub(wB, wA)
		cdot := Vec2{Dot(wRel, perpAxisX), Dot(wRel, perpAxisY)}

		maxImpulse := context_.h * joint.maxTorque
		oldImpulse := joint.perpImpulse
		cdotPlusBias := Vec2{cdot.x + bias.x, cdot.y + bias.y}
		sol := Solve2(k, cdotPlusBias)
		deltaImpulse := Vec2 {
			-massScale * sol.x - impulseScale * oldImpulse.x,
			-massScale * sol.y - impulseScale * oldImpulse.y,
		}
		joint.perpImpulse = Vec2{oldImpulse.x + deltaImpulse.x, oldImpulse.y + deltaImpulse.y}
		if LengthSquared2(joint.perpImpulse) > maxImpulse * maxImpulse {
			s := maxImpulse / Length2(joint.perpImpulse)
			joint.perpImpulse = Vec2{s * joint.perpImpulse.x, s * joint.perpImpulse.y}
		}

		deltaImpulse = Sub2(joint.perpImpulse, oldImpulse)

		angularImpulse := Blend2(deltaImpulse.x, perpAxisX, deltaImpulse.y, perpAxisY)
		wA = Sub(wA, MulMV(iA, angularImpulse))
		wB = Add(wB, MulMV(iB, angularImpulse))
	}

	if (stateA.flags & dynamicFlag) != 0 {
		stateA.angularVelocity = wA
	}

	if (stateB.flags & dynamicFlag) != 0 {
		stateB.angularVelocity = wB
	}
}

DrawParallelJoint :: proc(
	draw: ^DebugDraw,
	base: ^JointSim,
	transformA: WorldTransform,
	transformB: WorldTransform,
	scale: f32,
) {
	length := 0.1 * scale

	frameA := MulWorldTransforms(transformA, base.localFrameA)
	draw.DrawSegmentFcn(
		frameA.p,
		OffsetPos(frameA.p, MulSV(length, RotateVector(frameA.q, Vec3_axisZ))),
		.colorGreen,
		draw.context_,
	)

	frameB := MulWorldTransforms(transformB, base.localFrameB)
	draw.DrawSegmentFcn(
		frameB.p,
		OffsetPos(frameB.p, MulSV(length, RotateVector(frameB.q, Vec3_axisZ))),
		.colorBlue,
		draw.context_,
	)
}
