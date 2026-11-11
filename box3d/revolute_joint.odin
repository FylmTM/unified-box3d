// Ported from box3d src/revolute_joint.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

// B3_REC is expanded inline at each call site (`if world.recording != nil {...}`), as in body.odin.

// Point-to-point linear constraint
// C = pB - pA
// Cdot = vB - vA
//      = vB + cross(wB, rB) - vA - cross(wA, rA)
// Cdot = J * v
// J = [-E -skew(rA) E skew(rB) ]
//
// K = J * invM * JT
//   = [(1/mA + 1/mB) * E - skew(rA) * invIA * skew(rA) - skew(rB) * invIB * skew(rB)]

// Perpendicularity constraint
// frameA = qA * localFrameA
// frameB = qB * localFrameB
// qRel = conj(frameA) * frameB
// C = [qRel.x; qRel.y]
// qRelDot = 0.5 * conj(frameA) * (wB - wA) * frameB
// Cdot = [qRelDot.x, qRelDot.y]
// Pulling out wB and wA
// sr = qRel.s
// vr = qRel.v
// Jx = 0.5 * rotate(frameA, sr * ex + cross(vr, ex))
// Jy = 0.5 * rotate(frameA, sr * ey + cross(vr, ey))

// Motor constraint
// Cdot = wB - wA
// J = [0 0 -E 0 0 E]
// K = invIA + invIB

RevoluteJoint_EnableLimit :: proc(jointId: JointId, enableLimit: bool) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_RevoluteJointEnableLimit{jointId, enableLimit}
		RecWrite_RevoluteJointEnableLimit(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	if enableLimit != base.revoluteJoint.enableLimit {
		base.revoluteJoint.lowerImpulse = 0.0
		base.revoluteJoint.upperImpulse = 0.0
	}
	base.revoluteJoint.enableLimit = enableLimit
}

RevoluteJoint_IsLimitEnabled :: proc(jointId: JointId) -> bool {
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	return base.revoluteJoint.enableLimit
}

RevoluteJoint_GetLowerLimit :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	return base.revoluteJoint.lowerAngle
}

RevoluteJoint_GetUpperLimit :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	return base.revoluteJoint.upperAngle
}

RevoluteJoint_SetLimits :: proc(jointId: JointId, lowerLimitRadians: f32, upperLimitRadians: f32) {
	assert(IsValidFloat(lowerLimitRadians) && IsValidFloat(upperLimitRadians))
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_RevoluteJointSetLimits{jointId, lowerLimitRadians, upperLimitRadians}
		RecWrite_RevoluteJointSetLimits(world.recording, &recArgs)
	}

	lowerAngle := MinFloat(lowerLimitRadians, upperLimitRadians)
	upperAngle := MaxFloat(lowerLimitRadians, upperLimitRadians)

	base := GetJointSimCheckType(jointId, .revoluteJoint)
	base.revoluteJoint.lowerAngle = ClampFloat(lowerAngle, -f32(0.99) * PI, f32(0.99) * PI)
	base.revoluteJoint.upperAngle = ClampFloat(upperAngle, -f32(0.99) * PI, f32(0.99) * PI)
}

RevoluteJoint_GetAngle :: proc(jointId: JointId) -> f32 {
	world := GetWorld(i32(jointId.world0))
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	transformA := GetBodyTransform(world, base.bodyIdA)
	transformB := GetBodyTransform(world, base.bodyIdB)

	quatA := MulQuat(transformA.q, base.localFrameA.q)
	quatB := MulQuat(transformB.q, base.localFrameB.q)

	if DotQuat(quatA, quatB) < 0.0 {
		// this keeps the twist angle in the range [-pi, pi]
		quatB = NegateQuat(quatB)
	}

	relQ := InvMulQuat(quatA, quatB)

	return GetTwistAngle(relQ)
}

RevoluteJoint_EnableSpring :: proc(jointId: JointId, enableSpring: bool) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_RevoluteJointEnableSpring{jointId, enableSpring}
		RecWrite_RevoluteJointEnableSpring(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	if enableSpring != base.revoluteJoint.enableSpring {
		base.revoluteJoint.springImpulse = 0.0
	}
	base.revoluteJoint.enableSpring = enableSpring
}

RevoluteJoint_IsSpringEnabled :: proc(jointId: JointId) -> bool {
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	return base.revoluteJoint.enableSpring
}

RevoluteJoint_SetTargetAngle :: proc(jointId: JointId, targetRadians: f32) {
	assert(IsValidFloat(targetRadians) && -PI <= targetRadians && targetRadians <= PI)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_RevoluteJointSetTargetAngle{jointId, targetRadians}
		RecWrite_RevoluteJointSetTargetAngle(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	base.revoluteJoint.targetAngle = targetRadians
}

RevoluteJoint_GetTargetAngle :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	return base.revoluteJoint.targetAngle
}

RevoluteJoint_SetSpringHertz :: proc(jointId: JointId, hertz: f32) {
	assert(IsValidFloat(hertz) && hertz >= 0.0)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_RevoluteJointSetSpringHertz{jointId, hertz}
		RecWrite_RevoluteJointSetSpringHertz(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	base.revoluteJoint.hertz = hertz
}

RevoluteJoint_GetSpringHertz :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	return base.revoluteJoint.hertz
}

RevoluteJoint_SetSpringDampingRatio :: proc(jointId: JointId, dampingRatio: f32) {
	assert(IsValidFloat(dampingRatio) && dampingRatio >= 0.0)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_RevoluteJointSetSpringDampingRatio{jointId, dampingRatio}
		RecWrite_RevoluteJointSetSpringDampingRatio(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	base.revoluteJoint.dampingRatio = dampingRatio
}

RevoluteJoint_GetSpringDampingRatio :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	return base.revoluteJoint.dampingRatio
}

RevoluteJoint_EnableMotor :: proc(jointId: JointId, enableMotor: bool) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_RevoluteJointEnableMotor{jointId, enableMotor}
		RecWrite_RevoluteJointEnableMotor(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	if enableMotor != base.revoluteJoint.enableMotor {
		base.revoluteJoint.motorImpulse = 0.0
	}
	base.revoluteJoint.enableMotor = enableMotor
}

RevoluteJoint_IsMotorEnabled :: proc(jointId: JointId) -> bool {
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	return base.revoluteJoint.enableMotor
}

RevoluteJoint_SetMotorSpeed :: proc(jointId: JointId, motorSpeed: f32) {
	assert(IsValidFloat(motorSpeed))
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_RevoluteJointSetMotorSpeed{jointId, motorSpeed}
		RecWrite_RevoluteJointSetMotorSpeed(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	base.revoluteJoint.motorSpeed = motorSpeed
}

RevoluteJoint_GetMotorSpeed :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	return base.revoluteJoint.motorSpeed
}

RevoluteJoint_SetMaxMotorTorque :: proc(jointId: JointId, maxForce: f32) {
	assert(IsValidFloat(maxForce) && maxForce >= 0.0)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_RevoluteJointSetMaxMotorTorque{jointId, maxForce}
		RecWrite_RevoluteJointSetMaxMotorTorque(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	base.revoluteJoint.maxMotorTorque = maxForce
}

RevoluteJoint_GetMaxMotorTorque :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	return base.revoluteJoint.maxMotorTorque
}

RevoluteJoint_GetMotorTorque :: proc(jointId: JointId) -> f32 {
	world := GetWorld(i32(jointId.world0))
	base := GetJointSimCheckType(jointId, .revoluteJoint)
	return world.inv_h * base.revoluteJoint.motorImpulse
}

GetRevoluteJointForce :: proc(world: ^World, base: ^JointSim) -> Vec3 {
	force := MulSV(world.inv_h, base.revoluteJoint.linearImpulse)
	return force
}

GetRevoluteJointTorque :: proc(world: ^World, base: ^JointSim) -> Vec3 {
	transformA := GetBodyTransform(world, base.bodyIdA)
	joint := &base.revoluteJoint
	axis := RotateVector(base.localFrameA.q, Vec3_axisZ)
	axis = RotateVector(transformA.q, axis)

	relQ := InvMulQuat(joint.frameA.q, joint.frameB.q)

	// These are needed for warm starting
	joint.perpAxisX = MulSV(
		0.5,
		RotateVector(joint.frameA.q, Add(MulSV(relQ.s, Vec3_axisX), Cross(relQ.v, Vec3_axisX))),
	)
	joint.perpAxisY = MulSV(
		0.5,
		RotateVector(joint.frameA.q, Add(MulSV(relQ.s, Vec3_axisY), Cross(relQ.v, Vec3_axisY))),
	)

	axialImpulse := joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse
	angularImpulse := Add(MulSV(joint.perpImpulse.x, joint.perpAxisX), MulSV(joint.perpImpulse.y, joint.perpAxisY))
	angularImpulse = MulAdd(angularImpulse, axialImpulse, joint.rotationAxisZ)

	// todo add pivot torque
	impulse := MulAdd(
		angularImpulse,
		joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse,
		axis,
	)
	torque := MulSV(world.inv_h, impulse)
	return torque
}

PrepareRevoluteJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .revoluteJoint)

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

	joint := &base.revoluteJoint
	joint.indexA = bodyA.setIndex == i32(SetType.awakeSet) ? localIndexA : NULL_INDEX
	joint.indexB = bodyB.setIndex == i32(SetType.awakeSet) ? localIndexB : NULL_INDEX

	// Compute joint anchor frames with world space rotation, relative to center of mass
	// Avoid round-off here as much as possible.
	// b3Vec3 pf = (xf.p - c) + rot(xf.q, f.p)
	// pf = xf.p - (xf.p + rot(xf.q, lc)) + rot(xf.q, f.p)
	// pf = rot(xf.q, f.p - lc)
	joint.frameA.q = MulQuat(bodySimA.transform.q, base.localFrameA.q)
	joint.frameA.p = RotateVector(bodySimA.transform.q, Sub(base.localFrameA.p, bodySimA.localCenter))
	joint.frameB.q = MulQuat(bodySimB.transform.q, base.localFrameB.q)
	joint.frameB.p = RotateVector(bodySimB.transform.q, Sub(base.localFrameB.p, bodySimB.localCenter))

	joint.deltaCenter = SubPos(bodySimB.center, bodySimA.center)

	{
		// Rotation axis is the z-axis of body A.
		rotationAxisZ := RotateVector(joint.frameA.q, Vec3_axisZ)
		k := Dot(rotationAxisZ, MulMV(invInertiaSum, rotationAxisZ))
		joint.axialMass = k > 0.0 ? 1.0 / k : 0.0
		joint.rotationAxisZ = rotationAxisZ
	}

	relQ := InvMulQuat(joint.frameA.q, joint.frameB.q)

	{
		// These are needed for warm starting
		joint.perpAxisX = MulSV(
			0.5,
			RotateVector(joint.frameA.q, Add(MulSV(relQ.s, Vec3_axisX), Cross(relQ.v, Vec3_axisX))),
		)
		joint.perpAxisY = MulSV(
			0.5,
			RotateVector(joint.frameA.q, Add(MulSV(relQ.s, Vec3_axisY), Cross(relQ.v, Vec3_axisY))),
		)
	}

	joint.springSoftness = MakeSoft(joint.hertz, joint.dampingRatio, context_.h)

	if context_.enableWarmStarting == false {
		joint.linearImpulse = Vec3_zero
		joint.perpImpulse = Vec2{0.0, 0.0}
		joint.motorImpulse = 0.0
		joint.springImpulse = 0.0
		joint.lowerImpulse = 0.0
		joint.upperImpulse = 0.0
	}
}

WarmStartRevoluteJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .revoluteJoint)

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.revoluteJoint

	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity

	rA := RotateVector(stateA.deltaRotation, joint.frameA.p)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.p)

	axialImpulse := joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse
	angularImpulse := Add(MulSV(joint.perpImpulse.x, joint.perpAxisX), MulSV(joint.perpImpulse.y, joint.perpAxisY))
	angularImpulse = MulAdd(angularImpulse, axialImpulse, joint.rotationAxisZ)

	vA = MulSub(vA, mA, joint.linearImpulse)
	wA = Sub(wA, MulMV(iA, Add(Cross(rA, joint.linearImpulse), angularImpulse)))

	vB = MulAdd(vB, mB, joint.linearImpulse)
	wB = Add(wB, MulMV(iB, Add(Cross(rB, joint.linearImpulse), angularImpulse)))

	if (stateA.flags & dynamicFlag) != 0 {
		stateA.linearVelocity = vA
		stateA.angularVelocity = wA
	}

	if (stateB.flags & dynamicFlag) != 0 {
		stateB.linearVelocity = vB
		stateB.angularVelocity = wB
	}
}

SolveRevoluteJoint :: proc(base: ^JointSim, context_: ^StepContext, useBias: bool) {
	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.revoluteJoint
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

	if fixedRotation == false && (joint.enableSpring || joint.enableMotor || joint.enableLimit) {
		axis := joint.rotationAxisZ
		gA := MulMV(iA, axis)
		gB := MulMV(iB, axis)
		k := Dot(axis, Add(gA, gB))
		axialCdot := Dot(Sub(wB, wA), axis)
		axialImpulse: f32 = 0.0

		// Solve spring
		if joint.enableSpring {
			// Get the substep relative rotation
			targetAngle := joint.targetAngle
			angle := GetTwistAngle(relQ)
			c := angle - targetAngle

			bias := joint.springSoftness.biasRate * c
			massScale := joint.springSoftness.massScale
			impulseScale := joint.springSoftness.impulseScale

			deltaImpulse := -massScale * joint.axialMass * (axialCdot + bias) - impulseScale * joint.springImpulse
			joint.springImpulse += deltaImpulse

			axialCdot += k * deltaImpulse
			axialImpulse += deltaImpulse
		}

		if joint.enableMotor {
			cdot := axialCdot - joint.motorSpeed
			oldImpulse := joint.motorImpulse
			maxImpulse := joint.maxMotorTorque * context_.h
			deltaImpulse := ClampFloat(-joint.axialMass * cdot, -maxImpulse - oldImpulse, maxImpulse - oldImpulse)
			joint.motorImpulse = oldImpulse + deltaImpulse

			axialCdot += k * deltaImpulse
			axialImpulse += deltaImpulse
		}

		if joint.enableLimit {
			angle := GetTwistAngle(relQ)

			// todo does an updated twist axis help?

			// Lower limit
			{
				c := angle - joint.lowerAngle
				bias: f32 = 0.0
				massScale: f32 = 1.0
				impulseScale: f32 = 0.0
				if c > 0.0 {
					// speculation
					bias = c * context_.inv_h
				} else if useBias {
					bias = base.constraintSoftness.biasRate * c
					massScale = base.constraintSoftness.massScale
					impulseScale = base.constraintSoftness.impulseScale
				}

				oldImpulse := joint.lowerImpulse
				deltaImpulse := MaxFloat(
					-massScale * joint.axialMass * (axialCdot + bias) - impulseScale * oldImpulse,
					-oldImpulse,
				)
				joint.lowerImpulse = oldImpulse + deltaImpulse

				axialCdot += k * deltaImpulse
				axialImpulse += deltaImpulse
			}

			// Upper limit
			{
				c := joint.upperAngle - angle
				bias: f32 = 0.0
				massScale: f32 = 1.0
				impulseScale: f32 = 0.0
				if c > 0.0 {
					// speculation
					bias = c * context_.inv_h
				} else if useBias {
					bias = base.constraintSoftness.biasRate * c
					massScale = base.constraintSoftness.massScale
					impulseScale = base.constraintSoftness.impulseScale
				}

				// sign flipped on Cdot
				cdot := -axialCdot
				oldImpulse := joint.upperImpulse
				deltaImpulse := MaxFloat(
					-massScale * joint.axialMass * (cdot + bias) - impulseScale * oldImpulse,
					-oldImpulse,
				)
				joint.upperImpulse = oldImpulse + deltaImpulse

				// sign flipped on applied impulse
				axialImpulse -= deltaImpulse
			}
		}

		wA = MulSub(wA, axialImpulse, gA)
		wB = MulAdd(wB, axialImpulse, gB)
	}

	// Collinearity constraint
	if fixedRotation == false {
		bias := Vec2{0.0, 0.0}
		massScale: f32 = 1.0
		impulseScale: f32 = 0.0

		if useBias {
			c := Vec2{relQ.v.x, relQ.v.y}
			bias = MulSV2(base.constraintSoftness.biasRate, c)
			massScale = base.constraintSoftness.massScale
			impulseScale = base.constraintSoftness.impulseScale
		}

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
		oldImpulse := joint.perpImpulse
		sol := Solve2(k, Add2(cdot, bias))
		deltaImpulse := Sub2(MulSV2(-massScale, sol), MulSV2(impulseScale, oldImpulse))
		joint.perpImpulse = Add2(joint.perpImpulse, deltaImpulse)

		angularImpulse := Add(MulSV(deltaImpulse.x, perpAxisX), MulSV(deltaImpulse.y, perpAxisY))
		wA = Sub(wA, MulMV(iA, angularImpulse))
		wB = Add(wB, MulMV(iB, angularImpulse))
	}

	// Solve point-to-point constraint
	{
		rA := RotateVector(stateA.deltaRotation, joint.frameA.p)
		rB := RotateVector(stateB.deltaRotation, joint.frameB.p)

		cdot := Sub(Sub(Add(vB, Cross(wB, rB)), vA), Cross(wA, rA))

		bias := Vec3_zero
		massScale: f32 = 1.0
		impulseScale: f32 = 0.0
		if useBias {
			dcA := stateA.deltaPosition
			dcB := stateB.deltaPosition

			separation := Add(Add(Sub(dcB, dcA), Sub(rB, rA)), joint.deltaCenter)

			bias = MulSV(base.constraintSoftness.biasRate, separation)
			massScale = base.constraintSoftness.massScale
			impulseScale = base.constraintSoftness.impulseScale
		}

		b := SolvePointConstraint(mA, mB, base.invIA, base.invIB, rA, rB, Add(cdot, bias))

		impulse := Sub(MulSV(-massScale, b), MulSV(impulseScale, joint.linearImpulse))
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

DrawRevoluteJoint :: proc(
	draw: ^DebugDraw,
	base: ^JointSim,
	transformA: WorldTransform,
	transformB: WorldTransform,
	scale: f32,
) {
	frameA := MulWorldTransforms(transformA, base.localFrameA)

	length1 := 0.1 * scale
	draw.DrawSegmentFcn(
		frameA.p,
		OffsetPos(frameA.p, MulSV(length1, RotateVector(frameA.q, Vec3_axisX))),
		.colorRed,
		draw.context_,
	)
	draw.DrawSegmentFcn(
		frameA.p,
		OffsetPos(frameA.p, MulSV(length1, RotateVector(frameA.q, Vec3_axisY))),
		.colorGreen,
		draw.context_,
	)
	draw.DrawSegmentFcn(
		frameA.p,
		OffsetPos(frameA.p, MulSV(length1, RotateVector(frameA.q, Vec3_axisZ))),
		.colorBlue,
		draw.context_,
	)

	frameB := MulWorldTransforms(transformB, base.localFrameB)

	joint := &base.revoluteJoint
	kSliceCount :: 16

	// Twist limit
	if joint.enableLimit {
		quatA := frameA.q
		quatB := frameB.q

		if DotQuat(quatA, quatB) < 0.0 {
			// this keeps the twist angle in the range [-pi, pi]
			quatB = NegateQuat(quatB)
		}

		relQ := InvMulQuat(quatA, quatB)

		wedgeRadius := 0.2 * scale
		for index: i32 = 0; index < kSliceCount; index += 1 {
			t1 := f32(index + 0) / kSliceCount
			alpha1 := (1.0 - t1) * joint.lowerAngle + t1 * joint.upperAngle
			t2 := f32(index + 1) / kSliceCount
			alpha2 := (1.0 - t2) * joint.lowerAngle + t2 * joint.upperAngle

			vertex1 := Vec3{wedgeRadius * Cos(alpha1), wedgeRadius * Sin(alpha1), 0.0}
			vertex2 := Vec3{wedgeRadius * Cos(alpha2), wedgeRadius * Sin(alpha2), 0.0}

			if index == 0 {
				draw.DrawSegmentFcn(frameA.p, TransformWorldPoint(frameA, vertex1), .colorCyan, draw.context_)
			}

			if index == kSliceCount - 1 {
				draw.DrawSegmentFcn(TransformWorldPoint(frameA, vertex2), frameA.p, .colorCyan, draw.context_)
			}
			draw.DrawSegmentFcn(
				TransformWorldPoint(frameA, vertex1),
				TransformWorldPoint(frameA, vertex2),
				.colorCyan,
				draw.context_,
			)
		}

		twistAngle := GetTwistAngle(relQ)
		p2 := Vec3{wedgeRadius * Cos(twistAngle), wedgeRadius * Sin(twistAngle), 0.0}
		draw.DrawSegmentFcn(frameA.p, TransformWorldPoint(frameA, p2), .colorYellow, draw.context_)
	}
}
