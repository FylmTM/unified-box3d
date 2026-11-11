// Ported from box3d src/spherical_joint.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:intrinsics"

// B3_REC is expanded inline at each call site (`if world.recording != nil {...}`), as in body.odin.

SphericalJoint_EnableConeLimit :: proc(jointId: JointId, enableLimit: bool) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_SphericalJointEnableConeLimit{jointId, enableLimit}
		RecWrite_SphericalJointEnableConeLimit(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	if enableLimit != base.sphericalJoint.enableConeLimit {
		base.sphericalJoint.swingImpulse = 0.0
	}
	base.sphericalJoint.enableConeLimit = enableLimit
}

SphericalJoint_IsConeLimitEnabled :: proc(jointId: JointId) -> bool {
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	return base.sphericalJoint.enableConeLimit
}

SphericalJoint_GetConeLimit :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	return base.sphericalJoint.coneAngle
}

SphericalJoint_SetConeLimit :: proc(jointId: JointId, angleRadians: f32) {
	assert(IsValidFloat(angleRadians) && 0 <= angleRadians && angleRadians <= f32(0.5) * PI)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_SphericalJointSetConeLimit{jointId, angleRadians}
		RecWrite_SphericalJointSetConeLimit(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	base.sphericalJoint.coneAngle = angleRadians
}

SphericalJoint_GetConeAngle :: proc(jointId: JointId) -> f32 {
	world := GetWorld(i32(jointId.world0))
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	transformA := GetBodyTransform(world, base.bodyIdA)
	transformB := GetBodyTransform(world, base.bodyIdB)

	quatA := MulQuat(transformA.q, base.localFrameA.q)
	quatB := MulQuat(transformB.q, base.localFrameB.q)

	if DotQuat(quatA, quatB) < 0.0 {
		// this keeps the swing angle in the range [0, pi]
		quatB = NegateQuat(quatB)
	}

	relQ := InvMulQuat(quatA, quatB)

	return GetSwingAngle(relQ)
}

SphericalJoint_EnableTwistLimit :: proc(jointId: JointId, enableLimit: bool) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_SphericalJointEnableTwistLimit{jointId, enableLimit}
		RecWrite_SphericalJointEnableTwistLimit(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	if enableLimit != base.sphericalJoint.enableTwistLimit {
		base.sphericalJoint.lowerTwistImpulse = 0.0
		base.sphericalJoint.upperTwistImpulse = 0.0
	}
	base.sphericalJoint.enableTwistLimit = enableLimit
}

SphericalJoint_IsTwistLimitEnabled :: proc(jointId: JointId) -> bool {
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	return base.sphericalJoint.enableTwistLimit
}

SphericalJoint_GetLowerTwistLimit :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	return base.sphericalJoint.lowerTwistAngle
}

SphericalJoint_GetUpperTwistLimit :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	return base.sphericalJoint.upperTwistAngle
}

SphericalJoint_SetTwistLimits :: proc(jointId: JointId, lowerLimitRadians: f32, upperLimitRadians: f32) {
	assert(IsValidFloat(lowerLimitRadians) && IsValidFloat(upperLimitRadians))
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_SphericalJointSetTwistLimits{jointId, lowerLimitRadians, upperLimitRadians}
		RecWrite_SphericalJointSetTwistLimits(world.recording, &recArgs)
	}

	lowerAngle := MinFloat(lowerLimitRadians, upperLimitRadians)
	upperAngle := MaxFloat(lowerLimitRadians, upperLimitRadians)

	base := GetJointSimCheckType(jointId, .sphericalJoint)
	base.sphericalJoint.lowerTwistAngle = ClampFloat(lowerAngle, f32(-0.99) * PI, f32(0.99) * PI)
	base.sphericalJoint.upperTwistAngle = ClampFloat(upperAngle, f32(-0.99) * PI, f32(0.99) * PI)
}

SphericalJoint_GetTwistAngle :: proc(jointId: JointId) -> f32 {
	world := GetWorld(i32(jointId.world0))
	base := GetJointSimCheckType(jointId, .sphericalJoint)
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

SphericalJoint_EnableSpring :: proc(jointId: JointId, enableSpring: bool) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_SphericalJointEnableSpring{jointId, enableSpring}
		RecWrite_SphericalJointEnableSpring(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	if enableSpring != base.sphericalJoint.enableSpring {
		base.sphericalJoint.springImpulse = Vec3_zero
	}
	base.sphericalJoint.enableSpring = enableSpring
}

SphericalJoint_IsSpringEnabled :: proc(jointId: JointId) -> bool {
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	return base.sphericalJoint.enableSpring
}

SphericalJoint_SetTargetRotation :: proc(jointId: JointId, targetRotation: Quat) {
	assert(IsValidQuat(targetRotation))
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_SphericalJointSetTargetRotation{jointId, targetRotation}
		RecWrite_SphericalJointSetTargetRotation(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	base.sphericalJoint.targetRotation = targetRotation
}

SphericalJoint_GetTargetRotation :: proc(jointId: JointId) -> Quat {
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	return base.sphericalJoint.targetRotation
}

SphericalJoint_SetSpringHertz :: proc(jointId: JointId, hertz: f32) {
	assert(IsValidFloat(hertz) && hertz >= 0.0)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_SphericalJointSetSpringHertz{jointId, hertz}
		RecWrite_SphericalJointSetSpringHertz(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	base.sphericalJoint.hertz = hertz
}

SphericalJoint_GetSpringHertz :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	return base.sphericalJoint.hertz
}

SphericalJoint_SetSpringDampingRatio :: proc(jointId: JointId, dampingRatio: f32) {
	assert(IsValidFloat(dampingRatio) && dampingRatio >= 0.0)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_SphericalJointSetSpringDampingRatio{jointId, dampingRatio}
		RecWrite_SphericalJointSetSpringDampingRatio(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	base.sphericalJoint.dampingRatio = dampingRatio
}

SphericalJoint_GetSpringDampingRatio :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	return base.sphericalJoint.dampingRatio
}

SphericalJoint_EnableMotor :: proc(jointId: JointId, enableMotor: bool) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_SphericalJointEnableMotor{jointId, enableMotor}
		RecWrite_SphericalJointEnableMotor(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	if enableMotor != base.sphericalJoint.enableMotor {
		base.sphericalJoint.motorImpulse = Vec3_zero
	}
	base.sphericalJoint.enableMotor = enableMotor
}

SphericalJoint_IsMotorEnabled :: proc(jointId: JointId) -> bool {
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	return base.sphericalJoint.enableMotor
}

SphericalJoint_SetMotorVelocity :: proc(jointId: JointId, motorVelocity: Vec3) {
	assert(IsValidVec3(motorVelocity))
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_SphericalJointSetMotorVelocity{jointId, motorVelocity}
		RecWrite_SphericalJointSetMotorVelocity(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	base.sphericalJoint.motorVelocity = motorVelocity
}

SphericalJoint_GetMotorVelocity :: proc(jointId: JointId) -> Vec3 {
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	return base.sphericalJoint.motorVelocity
}

SphericalJoint_SetMaxMotorTorque :: proc(jointId: JointId, maxForce: f32) {
	assert(IsValidFloat(maxForce) && maxForce >= 0.0)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_SphericalJointSetMaxMotorTorque{jointId, maxForce}
		RecWrite_SphericalJointSetMaxMotorTorque(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	base.sphericalJoint.maxMotorTorque = maxForce
}

SphericalJoint_GetMaxMotorTorque :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	return base.sphericalJoint.maxMotorTorque
}

SphericalJoint_GetMotorTorque :: proc(jointId: JointId) -> Vec3 {
	world := GetWorld(i32(jointId.world0))
	base := GetJointSimCheckType(jointId, .sphericalJoint)
	return MulSV(world.inv_h, base.sphericalJoint.motorImpulse)
}

GetSphericalJointForce :: proc(world: ^World, base: ^JointSim) -> Vec3 {
	force := MulSV(world.inv_h, base.sphericalJoint.linearImpulse)
	return force
}

GetSphericalJointTorque :: proc(world: ^World, base: ^JointSim) -> Vec3 {
	xfA := GetBodyTransform(world, base.bodyIdA)
	xfB := GetBodyTransform(world, base.bodyIdB)
	qA := MulQuat(xfA.q, base.localFrameA.q)
	qB := MulQuat(xfB.q, base.localFrameB.q)

	// Cone axis is the z-axis of body A.
	coneAxis := RotateVector(qA, Vec3_axisZ)
	twistAxis := RotateVector(qB, Vec3_axisZ)
	swingAxis := Normalize(Cross(coneAxis, twistAxis))

	joint := &base.sphericalJoint
	impulse := Add(joint.springImpulse, joint.motorImpulse)
	impulse = MulAdd(impulse, joint.lowerTwistImpulse - joint.upperTwistImpulse, twistAxis)
	impulse = MulAdd(impulse, joint.swingImpulse, swingAxis)
	torque := MulSV(world.inv_h, impulse)
	return torque
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I r1_skew I -r2_skew ]
// K = J * invM * transpose(J)
// transpose(skew(r)) = -skew(r)
// K = diag(1/m1 + 1/m2) - r1_skew * invI1 * r1_skew - r2_skew * invI2 * r2_skew

// r_skew = R * skew(r_local) * RT
// invI = R * invI_local * RT
// r_skew * invI * r_skew = R * skew(r_local) * RT * R * invI_local * RT * R * r_skew * RT
//                        = R * ( skew(r_local) * invI_local * skew(r_local) ) * RT

PrepareSphericalJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .sphericalJoint)

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

	joint := &base.sphericalJoint
	joint.indexA = bodyA.setIndex == i32(SetType.awakeSet) ? localIndexA : NULL_INDEX
	joint.indexB = bodyB.setIndex == i32(SetType.awakeSet) ? localIndexB : NULL_INDEX

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.q = MulQuat(bodySimA.transform.q, base.localFrameA.q)
	joint.frameA.p = RotateVector(bodySimA.transform.q, Sub(base.localFrameA.p, bodySimA.localCenter))
	joint.frameB.q = MulQuat(bodySimB.transform.q, base.localFrameB.q)
	joint.frameB.p = RotateVector(bodySimB.transform.q, Sub(base.localFrameB.p, bodySimB.localCenter))

	joint.deltaCenter = SubPos(bodySimB.center, bodySimA.center)

	// Cone axis is the z-axis of body A.
	coneAxis := RotateVector(joint.frameA.q, Vec3_axisZ)

	// Twist axis is the z-axis of body B.
	twistAxis := RotateVector(joint.frameB.q, Vec3_axisZ)

	if joint.enableConeLimit {
		// Swing axis may be zero
		swingAxis := Normalize(Cross(coneAxis, twistAxis))
		k := Dot(swingAxis, MulMV(invInertiaSum, swingAxis))
		joint.swingMass = k > 0.0 ? 1.0 / k : 0.0
		joint.swingAxis = swingAxis
	}

	if joint.enableTwistLimit {
		relQ := InvMulQuat(joint.frameA.q, joint.frameB.q)
		tanThetaOver2 := intrinsics.sqrt(
			(relQ.v.x * relQ.v.x + relQ.v.y * relQ.v.y) / (relQ.v.z * relQ.v.z + relQ.s * relQ.s),
		)

		// todo verify this Jacobian using a finite difference, unit test?
		swingAxis := Normalize(Cross(coneAxis, twistAxis))
		perpAxis := Cross(swingAxis, coneAxis)
		twistJacobian := MulAdd(coneAxis, tanThetaOver2, perpAxis)
		k := Dot(twistJacobian, MulMV(invInertiaSum, twistJacobian))
		joint.twistMass = k > 0.0 ? 1.0 / k : 0.0
		joint.twistJacobian = twistJacobian
	}

	if base.fixedRotation == false {
		joint.rotationMass = InvertMatrix(invInertiaSum)
	} else {
		joint.rotationMass = Mat3_zero
	}

	joint.springSoftness = MakeSoft(joint.hertz, joint.dampingRatio, context_.h)

	if context_.enableWarmStarting == false {
		joint.linearImpulse = Vec3_zero
		joint.motorImpulse = Vec3_zero
		joint.springImpulse = Vec3_zero
		joint.swingImpulse = 0.0
		joint.lowerTwistImpulse = 0.0
		joint.upperTwistImpulse = 0.0
	}
}

WarmStartSphericalJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .sphericalJoint)

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.sphericalJoint

	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity

	rA := RotateVector(stateA.deltaRotation, joint.frameA.p)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.p)

	angularImpulse := Add(joint.springImpulse, joint.motorImpulse)
	angularImpulse = MulSub(angularImpulse, joint.swingImpulse, joint.swingAxis)
	angularImpulse = MulAdd(angularImpulse, joint.lowerTwistImpulse - joint.upperTwistImpulse, joint.twistJacobian)

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

SolveSphericalJoint :: proc(base: ^JointSim, context_: ^StepContext, useBias: bool) {
	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.sphericalJoint
	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity

	fixedRotation := base.fixedRotation
	quatA := MulQuat(stateA.deltaRotation, joint.frameA.q)
	quatB := MulQuat(stateB.deltaRotation, joint.frameB.q)

	relQ := InvMulQuat(quatA, quatB)

	if fixedRotation == false &&
	   (joint.enableSpring || joint.enableMotor || joint.enableTwistLimit || joint.enableConeLimit) {
		invInertiaSum := AddMM(iA, iB)
		cdot := Sub(wB, wA)
		angularImpulse := Vec3_zero

		// Solve spring
		if joint.enableSpring {
			// Rotation constraint error
			deltaRotation := DeltaQuatToRotation(relQ, joint.targetRotation)
			c := Neg(RotateVector(quatA, deltaRotation))

			bias := MulSV(joint.springSoftness.biasRate, c)
			massScale := joint.springSoftness.massScale
			impulseScale := joint.springSoftness.impulseScale

			impulse := MulSub(
				MulSV(-massScale, MulMV(joint.rotationMass, Add(cdot, bias))),
				impulseScale,
				joint.springImpulse,
			)
			joint.springImpulse = Add(joint.springImpulse, impulse)

			cdot = Add(cdot, MulMV(invInertiaSum, impulse))
			angularImpulse = Add(angularImpulse, impulse)
		}

		if joint.enableMotor {
			// todo it might be more accurate to have the motor velocity in frameA due to sub-stepping.
			lambda := Neg(MulMV(joint.rotationMass, Sub(cdot, joint.motorVelocity)))
			newImpulse := Add(joint.motorImpulse, lambda)
			length := Length(newImpulse)
			maxImpulse := joint.maxMotorTorque * context_.h
			if length > maxImpulse {
				newImpulse = MulSV(maxImpulse / length, newImpulse)
			}

			lambda = Sub(newImpulse, joint.motorImpulse)
			joint.motorImpulse = newImpulse

			cdot = Add(cdot, MulMV(invInertiaSum, lambda))
			angularImpulse = Add(angularImpulse, lambda)
		}

		if joint.enableTwistLimit {
			twistAngle := GetTwistAngle(relQ)

			// todo does an updated twist axis help?

			twistJacobian := joint.twistJacobian
			twistResponse := MulMV(invInertiaSum, twistJacobian)

			// Lower limit
			{
				c := twistAngle - joint.lowerTwistAngle
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

				twistCdot := Dot(cdot, twistJacobian)
				oldImpulse := joint.lowerTwistImpulse
				deltaImpulse := MaxFloat(
					-massScale * joint.twistMass * (twistCdot + bias) - impulseScale * oldImpulse,
					-oldImpulse,
				)
				joint.lowerTwistImpulse = oldImpulse + deltaImpulse

				cdot = MulAdd(cdot, deltaImpulse, twistResponse)
				angularImpulse = MulAdd(angularImpulse, deltaImpulse, twistJacobian)
			}

			// Upper limit
			{
				c := joint.upperTwistAngle - twistAngle
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
				twistCdot := -Dot(cdot, twistJacobian)
				oldImpulse := joint.upperTwistImpulse
				deltaImpulse := MaxFloat(
					-massScale * joint.twistMass * (twistCdot + bias) - impulseScale * oldImpulse,
					-oldImpulse,
				)
				joint.upperTwistImpulse = oldImpulse + deltaImpulse

				// sign flipped on applied impulse
				cdot = MulSub(cdot, deltaImpulse, twistResponse)
				angularImpulse = MulSub(angularImpulse, deltaImpulse, twistJacobian)
			}
		}

		if joint.enableConeLimit {
			swingAngle := GetSwingAngle(relQ)

			// todo does an updated swing axis help?
			// b3Vec3 axisA = b3RotateVector( quatA, b3Vec3_axisZ );
			// b3Vec3 axisB = b3RotateVector( quatB, b3Vec3_axisZ );
			// b3Vec3 swingAxis = b3Normalize( b3Cross( axisA, axisB ) );
			// joint->swingAxis = swingAxis;

			swingAxis := joint.swingAxis

			c := joint.coneAngle - swingAngle
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
			swingCdot := -Dot(cdot, swingAxis)
			oldImpulse := joint.swingImpulse
			deltaImpulse := MaxFloat(
				-massScale * joint.swingMass * (swingCdot + bias) - impulseScale * oldImpulse,
				-oldImpulse,
			)
			joint.swingImpulse = oldImpulse + deltaImpulse

			// sign flipped on applied impulse
			angularImpulse = MulSub(angularImpulse, deltaImpulse, swingAxis)
		}

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

			separation := Add(Sub(dcB, dcA), Sub(rB, rA))
			separation = Add(separation, joint.deltaCenter)

			bias = MulSV(base.constraintSoftness.biasRate, separation)
			massScale = base.constraintSoftness.massScale
			impulseScale = base.constraintSoftness.impulseScale
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

DrawSphericalJoint :: proc(
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

	length2 := 0.2 * scale
	draw.DrawSegmentFcn(
		frameB.p,
		OffsetPos(frameB.p, MulSV(length2, RotateVector(frameB.q, Vec3_axisZ))),
		.colorOrange,
		draw.context_,
	)

	joint := &base.sphericalJoint
	kSliceCount :: 16

	// Twist limit
	if joint.enableTwistLimit {
		quatA := frameA.q
		quatB := frameB.q

		if DotQuat(quatA, quatB) < 0.0 {
			// this keeps the twist angle in the range [-pi, pi]
			quatB = NegateQuat(quatB)
		}

		relQ := InvMulQuat(quatA, quatB)

		wedgeRadius := 0.1 * scale
		for index: i32 = 0; index < kSliceCount; index += 1 {
			t1 := f32(index + 0) / kSliceCount
			alpha1 := (1.0 - t1) * joint.lowerTwistAngle + t1 * joint.upperTwistAngle
			t2 := f32(index + 1) / kSliceCount
			alpha2 := (1.0 - t2) * joint.lowerTwistAngle + t2 * joint.upperTwistAngle

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

	// Swing limit
	if joint.enableConeLimit {
		radius := 0.1 * scale
		coneRadius := radius * Sin(joint.coneAngle)
		coneHeight := radius * Cos(joint.coneAngle)

		for index: i32 = 0; index < kSliceCount; index += 1 {
			phi1 := 2.0 * f32(index + 0) / kSliceCount * PI
			phi2 := 2.0 * f32(index + 1) / kSliceCount * PI

			vertex1 := Vec3{coneRadius * Cos(phi1), coneRadius * Sin(phi1), coneHeight}
			vertex2 := Vec3{coneRadius * Cos(phi2), coneRadius * Sin(phi2), coneHeight}

			draw.DrawSegmentFcn(frameA.p, TransformWorldPoint(frameA, vertex1), .colorCyan, draw.context_)
			draw.DrawSegmentFcn(
				TransformWorldPoint(frameA, vertex1),
				TransformWorldPoint(frameA, vertex2),
				.colorCyan,
				draw.context_,
			)
		}
	}
}
