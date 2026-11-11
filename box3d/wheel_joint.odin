// Ported from box3d src/wheel_joint.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

// B3_REC is expanded inline at each call site (`if world.recording != nil {...}`), as in body.odin.

WheelJoint_EnableSuspension :: proc(jointId: JointId, enableSpring: bool) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WheelJointEnableSuspension{jointId, enableSpring}
		RecWrite_WheelJointEnableSuspension(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .wheelJoint)

	if enableSpring != joint.wheelJoint.enableSuspensionSpring {
		joint.wheelJoint.enableSuspensionSpring = enableSpring
		joint.wheelJoint.suspensionSpringImpulse = 0.0
	}
}

WheelJoint_IsSuspensionEnabled :: proc(jointId: JointId) -> bool {
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return joint.wheelJoint.enableSuspensionSpring
}

WheelJoint_SetSuspensionHertz :: proc(jointId: JointId, hertz: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WheelJointSetSuspensionHertz{jointId, hertz}
		RecWrite_WheelJointSetSuspensionHertz(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	joint.wheelJoint.suspensionHertz = hertz
}

WheelJoint_GetSuspensionHertz :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return joint.wheelJoint.suspensionHertz
}

WheelJoint_SetSuspensionDampingRatio :: proc(jointId: JointId, dampingRatio: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WheelJointSetSuspensionDampingRatio{jointId, dampingRatio}
		RecWrite_WheelJointSetSuspensionDampingRatio(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	joint.wheelJoint.suspensionDampingRatio = dampingRatio
}

WheelJoint_GetSuspensionDampingRatio :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return joint.wheelJoint.suspensionDampingRatio
}

WheelJoint_EnableSuspensionLimit :: proc(jointId: JointId, enableLimit: bool) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WheelJointEnableSuspensionLimit{jointId, enableLimit}
		RecWrite_WheelJointEnableSuspensionLimit(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	if joint.wheelJoint.enableSuspensionLimit != enableLimit {
		joint.wheelJoint.lowerSuspensionImpulse = 0.0
		joint.wheelJoint.upperSuspensionImpulse = 0.0
		joint.wheelJoint.enableSuspensionLimit = enableLimit
	}
}

WheelJoint_IsSuspensionLimitEnabled :: proc(jointId: JointId) -> bool {
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return joint.wheelJoint.enableSuspensionLimit
}

WheelJoint_GetLowerSuspensionLimit :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return joint.wheelJoint.lowerSuspensionLimit
}

WheelJoint_GetUpperSuspensionLimit :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return joint.wheelJoint.upperSuspensionLimit
}

WheelJoint_SetSuspensionLimits :: proc(jointId: JointId, lower: f32, upper: f32) {
	assert(lower <= upper)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WheelJointSetSuspensionLimits{jointId, lower, upper}
		RecWrite_WheelJointSetSuspensionLimits(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	if lower != joint.wheelJoint.lowerSuspensionLimit || upper != joint.wheelJoint.upperSuspensionLimit {
		joint.wheelJoint.lowerSuspensionLimit = lower
		joint.wheelJoint.upperSuspensionLimit = upper
		joint.wheelJoint.lowerSuspensionImpulse = 0.0
		joint.wheelJoint.upperSuspensionImpulse = 0.0
	}
}

WheelJoint_EnableSpinMotor :: proc(jointId: JointId, enableMotor: bool) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WheelJointEnableSpinMotor{jointId, enableMotor}
		RecWrite_WheelJointEnableSpinMotor(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	if joint.wheelJoint.enableSpinMotor != enableMotor {
		joint.wheelJoint.spinImpulse = 0.0
		joint.wheelJoint.enableSpinMotor = enableMotor
	}
}

WheelJoint_IsSpinMotorEnabled :: proc(jointId: JointId) -> bool {
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return joint.wheelJoint.enableSpinMotor
}

WheelJoint_SetSpinMotorSpeed :: proc(jointId: JointId, motorSpeed: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WheelJointSetSpinMotorSpeed{jointId, motorSpeed}
		RecWrite_WheelJointSetSpinMotorSpeed(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	joint.wheelJoint.spinSpeed = motorSpeed
}

WheelJoint_GetSpinMotorSpeed :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return joint.wheelJoint.spinSpeed
}

WheelJoint_SetMaxSpinTorque :: proc(jointId: JointId, torque: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WheelJointSetMaxSpinTorque{jointId, torque}
		RecWrite_WheelJointSetMaxSpinTorque(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	joint.wheelJoint.maxSpinTorque = torque
}

WheelJoint_GetMaxSpinTorque :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return joint.wheelJoint.maxSpinTorque
}

WheelJoint_EnableSteering :: proc(jointId: JointId, flag: bool) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WheelJointEnableSteering{jointId, flag}
		RecWrite_WheelJointEnableSteering(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	if joint.wheelJoint.enableSteering != flag {
		joint.wheelJoint.angularImpulse = Vec2{0.0, 0.0}
		joint.wheelJoint.enableSteering = flag
	}
}

WheelJoint_IsSteeringEnabled :: proc(jointId: JointId) -> bool {
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return joint.wheelJoint.enableSteering
}

WheelJoint_SetSteeringHertz :: proc(jointId: JointId, hertz: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WheelJointSetSteeringHertz{jointId, hertz}
		RecWrite_WheelJointSetSteeringHertz(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	joint.wheelJoint.steeringHertz = hertz
}

WheelJoint_GetSteeringHertz :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return joint.wheelJoint.steeringHertz
}

WheelJoint_SetSteeringDampingRatio :: proc(jointId: JointId, dampingRatio: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WheelJointSetSteeringDampingRatio{jointId, dampingRatio}
		RecWrite_WheelJointSetSteeringDampingRatio(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	joint.wheelJoint.steeringDampingRatio = dampingRatio
}

WheelJoint_GetSteeringDampingRatio :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return joint.wheelJoint.steeringDampingRatio
}

WheelJoint_SetMaxSteeringTorque :: proc(jointId: JointId, maxTorque: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WheelJointSetMaxSteeringTorque{jointId, maxTorque}
		RecWrite_WheelJointSetMaxSteeringTorque(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	joint.wheelJoint.maxSteeringTorque = maxTorque
}

WheelJoint_GetMaxSteeringTorque :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return joint.wheelJoint.maxSteeringTorque
}

WheelJoint_EnableSteeringLimit :: proc(jointId: JointId, flag: bool) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WheelJointEnableSteeringLimit{jointId, flag}
		RecWrite_WheelJointEnableSteeringLimit(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	if joint.wheelJoint.enableSteeringLimit != flag {
		joint.wheelJoint.lowerSteeringImpulse = 0.0
		joint.wheelJoint.upperSteeringImpulse = 0.0
		joint.wheelJoint.enableSteeringLimit = flag
	}
}

WheelJoint_IsSteeringLimitEnabled :: proc(jointId: JointId) -> bool {
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return joint.wheelJoint.enableSteeringLimit
}

WheelJoint_GetLowerSteeringLimit :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return joint.wheelJoint.lowerSteeringLimit
}

WheelJoint_GetUpperSteeringLimit :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return joint.wheelJoint.upperSteeringLimit
}

WheelJoint_SetSteeringLimits :: proc(jointId: JointId, lowerRadians: f32, upperRadians: f32) {
	assert(lowerRadians <= upperRadians)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WheelJointSetSteeringLimits{jointId, lowerRadians, upperRadians}
		RecWrite_WheelJointSetSteeringLimits(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	joint.wheelJoint.lowerSteeringLimit = lowerRadians
	joint.wheelJoint.upperSteeringLimit = upperRadians
}

WheelJoint_SetTargetSteeringAngle :: proc(jointId: JointId, radians: f32) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_WheelJointSetTargetSteeringAngle{jointId, radians}
		RecWrite_WheelJointSetTargetSteeringAngle(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	joint.wheelJoint.targetSteeringAngle = radians
}

WheelJoint_GetTargetSteeringAngle :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return joint.wheelJoint.targetSteeringAngle
}

WheelJoint_GetSpinSpeed :: proc(jointId: JointId) -> f32 {
	world := GetWorld(i32(jointId.world0))
	base := GetJointSimCheckType(jointId, .wheelJoint)

	idA := base.bodyIdA
	idB := base.bodyIdB

	bodyA := Array_Get(&world.bodies, idA)
	bodyB := Array_Get(&world.bodies, idB)

	setB := Array_Get(&world.solverSets, bodyB.setIndex)
	localIndexB := bodyB.localIndex
	bodySimB := Array_Get(&setB.bodySims, localIndexB)

	quatB := MulQuat(bodySimB.transform.q, base.localFrameB.q)
	spinAxis := RotateVector(quatB, Vec3_axisZ)

	wA := Vec3_zero
	stateA := GetBodyState(world, bodyA)
	if stateA != nil {
		wA = stateA.angularVelocity
	}

	wB := Vec3_zero
	stateB := GetBodyState(world, bodyB)
	if stateB != nil {
		wB = stateB.angularVelocity
	}

	speed := Dot(Sub(wB, wA), spinAxis)
	return speed
}

WheelJoint_GetSpinTorque :: proc(jointId: JointId) -> f32 {
	world := GetWorld(i32(jointId.world0))
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return world.inv_h * joint.wheelJoint.spinImpulse
}

WheelJoint_GetSteeringAngle :: proc(jointId: JointId) -> f32 {
	world := GetWorld(i32(jointId.world0))
	base := GetJointSimCheckType(jointId, .wheelJoint)

	idA := base.bodyIdA
	idB := base.bodyIdB

	bodyA := Array_Get(&world.bodies, idA)
	bodyB := Array_Get(&world.bodies, idB)

	setA := Array_Get(&world.solverSets, bodyA.setIndex)
	setB := Array_Get(&world.solverSets, bodyB.setIndex)

	localIndexA := bodyA.localIndex
	localIndexB := bodyB.localIndex

	bodySimA := Array_Get(&setA.bodySims, localIndexA)
	bodySimB := Array_Get(&setB.bodySims, localIndexB)

	quatA := MulQuat(bodySimA.transform.q, base.localFrameA.q)
	quatB := MulQuat(bodySimB.transform.q, base.localFrameB.q)

	matrixA := MakeMatrixFromQuat(quatA)
	matrixB := MakeMatrixFromQuat(quatB)

	// Twist around x-axis
	cs := Dot(matrixB.cz, matrixA.cz)
	ss := -Dot(matrixB.cz, matrixA.cy)

	return Atan2(ss, cs)
}

WheelJoint_GetSteeringTorque :: proc(jointId: JointId) -> f32 {
	world := GetWorld(i32(jointId.world0))
	joint := GetJointSimCheckType(jointId, .wheelJoint)
	return world.inv_h * joint.wheelJoint.steeringSpringImpulse
}

GetWheelJointForce :: proc(world: ^World, base: ^JointSim) -> Vec3 {
	transformA := GetBodyTransform(world, base.bodyIdA)
	joint := &base.wheelJoint

	// impulse in joint space
	impulse := Vec3 {
		joint.linearImpulse.x,
		joint.linearImpulse.y,
		joint.lowerSuspensionLimit + joint.upperSuspensionImpulse + joint.suspensionSpringImpulse,
	}

	// convert impulse to force
	force := MulSV(world.inv_h, impulse)

	// convert to body space
	force = RotateVector(base.localFrameA.q, force)

	// convert to world space
	force = RotateVector(transformA.q, force)
	return force
}

GetWheelJointTorque :: proc(world: ^World, base: ^JointSim) -> Vec3 {
	assert(base.type == .wheelJoint)

	// chase body id to the solver set where the body lives
	idA := base.bodyIdA
	// int idB = base->bodyIdB;

	bodyA := Array_Get(&world.bodies, idA)
	// b3Body* bodyB = b3Array_Get( world->bodies, idB  );

	setA := Array_Get(&world.solverSets, bodyA.setIndex)
	// b3SolverSet* setB = b3Array_Get( world->solverSets, bodyB->setIndex  );

	localIndexA := bodyA.localIndex
	// int localIndexB = bodyB->localIndex;

	bodySimA := Array_Get(&setA.bodySims, localIndexA)
	// b3BodySim* bodySimB = b3Array_Get( setB->bodySims, localIndexB  );

	qA := MulQuat(bodySimA.transform.q, base.localFrameA.q)

	matrixA := MakeMatrixFromQuat(qA)

	return MulSV(world.inv_h * base.wheelJoint.spinImpulse, matrixA.cz)
}

// See constraints.pdf

PrepareWheelJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .wheelJoint)

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

	joint := &base.wheelJoint

	joint.indexA = bodyA.setIndex == i32(SetType.awakeSet) ? localIndexA : NULL_INDEX
	joint.indexB = bodyB.setIndex == i32(SetType.awakeSet) ? localIndexB : NULL_INDEX

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.q = MulQuat(bodySimA.transform.q, base.localFrameA.q)
	joint.frameA.p = RotateVector(bodySimA.transform.q, Sub(base.localFrameA.p, bodySimA.localCenter))
	joint.frameB.q = MulQuat(bodySimB.transform.q, base.localFrameB.q)
	joint.frameB.p = RotateVector(bodySimB.transform.q, Sub(base.localFrameB.p, bodySimB.localCenter))

	// Compute the initial center delta. Incremental position updates are relative to this.
	joint.deltaCenter = SubPos(bodySimB.center, bodySimA.center)

	rA := joint.frameA.p
	rB := joint.frameB.p

	matrixA := MakeMatrixFromQuat(joint.frameA.q)
	matrixB := MakeMatrixFromQuat(joint.frameB.q)

	// todo use fresh effective masses in the sub-step to avoid divergence like I saw for the prismatic joint

	{
		suspensionAxis := matrixA.cx
		rAn := Cross(rA, suspensionAxis)
		rBn := Cross(rB, suspensionAxis)

		k := base.invMassA + base.invMassB + Dot(rAn, MulMV(base.invIA, rAn)) + Dot(rBn, MulMV(base.invIB, rBn))
		joint.suspensionMass = k > 0.0 ? 1.0 / k : 0.0
	}

	joint.suspensionSoftness = MakeSoft(joint.suspensionHertz, joint.suspensionDampingRatio, context_.h)
	joint.steeringSoftness = MakeSoft(joint.steeringHertz, joint.steeringDampingRatio, context_.h)

	{
		// Rotation axis is the z-axis of body A.
		spinAxis := matrixB.cz
		k := Dot(spinAxis, MulMV(invInertiaSum, spinAxis))
		joint.spinMass = k > 0.0 ? 1.0 / k : 0.0
	}

	{
		// Twist constraint around x-axis
		cs := Dot(matrixB.cz, matrixA.cz)
		ss := -Dot(matrixB.cz, matrixA.cy)
		den := cs * cs + ss * ss
		den = den > 0.0 ? 1.0 / den : 0.0
		steeringAxis := MulSV(den, Cross(matrixB.cz, Sub(MulSV(-cs, matrixA.cy), MulSV(ss, matrixA.cz))))

		k := Dot(steeringAxis, MulMV(invInertiaSum, steeringAxis))
		joint.steeringMass = k > 0.0 ? 1.0 / k : 0.0
	}

	if context_.enableWarmStarting == false {
		joint.linearImpulse = Vec2{0.0, 0.0}
		joint.angularImpulse = Vec2{0.0, 0.0}
		joint.spinImpulse = 0.0
		joint.suspensionSpringImpulse = 0.0
		joint.lowerSuspensionImpulse = 0.0
		joint.upperSuspensionImpulse = 0.0
		joint.steeringSpringImpulse = 0.0
		joint.lowerSteeringImpulse = 0.0
		joint.upperSteeringImpulse = 0.0
	}
}

WarmStartWheelJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .wheelJoint)

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.wheelJoint

	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	rA := RotateVector(stateA.deltaRotation, joint.frameA.p)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.p)

	d := Add(Add(Sub(stateB.deltaPosition, stateA.deltaPosition), joint.deltaCenter), Sub(rB, rA))

	quatA := MulQuat(stateA.deltaRotation, joint.frameA.q)
	quatB := MulQuat(stateB.deltaRotation, joint.frameB.q)
	if DotQuat(quatA, quatB) < 0.0 {
		// this keeps the rotation angle in the range [-pi, pi]
		quatB = NegateQuat(quatB)
	}

	matrixA := MakeMatrixFromQuat(quatA)
	matrixB := MakeMatrixFromQuat(quatB)

	sAx := Cross(Add(d, rA), matrixA.cx)
	sBx := Cross(rB, matrixA.cx)
	sAy := Cross(Add(d, rA), matrixA.cy)
	sBy := Cross(rB, matrixA.cy)
	sAz := Cross(Add(d, rA), matrixA.cz)
	sBz := Cross(rB, matrixA.cz)

	suspensionImpulse := joint.suspensionSpringImpulse + joint.lowerSuspensionImpulse - joint.upperSuspensionImpulse

	linearImpulseY := joint.linearImpulse.x
	linearImpulseZ := joint.linearImpulse.y
	angularImpulseX := joint.angularImpulse.x
	angularImpulseY := joint.angularImpulse.y

	linearImpulse := Blend3(suspensionImpulse, matrixA.cx, linearImpulseY, matrixA.cy, linearImpulseZ, matrixA.cz)
	angularImpulseA := Blend3(suspensionImpulse, sAx, linearImpulseY, sAy, linearImpulseZ, sAz)
	angularImpulseB := Blend3(suspensionImpulse, sBx, linearImpulseY, sBy, linearImpulseZ, sBz)
	angularImpulse := MulSV(joint.spinImpulse, matrixA.cz)

	spinAxis := matrixB.cz

	if joint.enableSteering {
		// Twist constraint around x-axis
		cs := Dot(matrixB.cz, matrixA.cz)
		ss := -Dot(matrixB.cz, matrixA.cy)
		den := cs * cs + ss * ss
		den = den > 0.0 ? 1.0 / den : 0.0
		steeringAxis := MulSV(den, Cross(matrixB.cz, Sub(MulSV(-cs, matrixA.cy), MulSV(ss, matrixA.cz))))

		perpAxis := Cross(spinAxis, matrixA.cx)
		steeringImpulse := joint.steeringSpringImpulse + joint.lowerSteeringImpulse - joint.upperSteeringImpulse
		angularImpulse = Blend3(angularImpulseX, perpAxis, joint.spinImpulse, spinAxis, steeringImpulse, steeringAxis)
	} else {
		relQ := InvMulQuat(quatA, quatB)
		perpAxisX := MulSV(0.5, RotateVector(quatA, Add(MulSV(relQ.s, Vec3_axisX), Cross(relQ.v, Vec3_axisX))))
		perpAxisY := MulSV(0.5, RotateVector(quatA, Add(MulSV(relQ.s, Vec3_axisY), Cross(relQ.v, Vec3_axisY))))
		angularImpulse = Add(
			angularImpulse,
			Blend3(angularImpulseX, perpAxisX, angularImpulseY, perpAxisY, joint.spinImpulse, spinAxis),
		)
	}

	if (stateA.flags & dynamicFlag) != 0 {
		stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, linearImpulse)
		stateA.angularVelocity = Sub(stateA.angularVelocity, MulMV(iA, Add(angularImpulseA, angularImpulse)))
	}

	if (stateB.flags & dynamicFlag) != 0 {
		stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, linearImpulse)
		stateB.angularVelocity = Add(stateB.angularVelocity, MulMV(iB, Add(angularImpulseB, angularImpulse)))
	}
}

SolveWheelJoint :: proc(base: ^JointSim, context_: ^StepContext, useBias: bool) {
	assert(base.type == .wheelJoint)

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.wheelJoint

	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity

	fixedRotation := base.fixedRotation

	// current anchors
	rA := RotateVector(stateA.deltaRotation, joint.frameA.p)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.p)

	quatA := MulQuat(stateA.deltaRotation, joint.frameA.q)
	quatB := MulQuat(stateB.deltaRotation, joint.frameB.q)

	if DotQuat(quatA, quatB) < 0.0 {
		// this keeps the rotation angle in the range [-pi, pi]
		quatB = NegateQuat(quatB)
	}

	relQ := InvMulQuat(quatA, quatB)
	matrixA := MakeMatrixFromQuat(quatA)
	matrixB := MakeMatrixFromQuat(quatB)

	// b3Vec3 spinAxis = b3RotateVector( quatB, b3Vec3_axisZ );

	d := Add(Add(Sub(stateB.deltaPosition, stateA.deltaPosition), joint.deltaCenter), Sub(rB, rA))
	sAx := Cross(Add(d, rA), matrixA.cx)
	sBx := Cross(rB, matrixA.cx)
	sAy := Cross(Add(d, rA), matrixA.cy)
	sBy := Cross(rB, matrixA.cy)
	sAz := Cross(Add(d, rA), matrixA.cz)
	sBz := Cross(rB, matrixA.cz)

	translation := Dot(matrixA.cx, d)

	// Steering param ib = cz_b, ia = cz_a, ja = -cy_a
	cs := Dot(matrixB.cz, matrixA.cz)
	ss := -Dot(matrixB.cz, matrixA.cy)
	den := cs * cs + ss * ss
	den = den > 0.0 ? 1.0 / den : 0.0
	steeringAxis := MulSV(den, Cross(matrixB.cz, Sub(MulSV(-cs, matrixA.cy), MulSV(ss, matrixA.cz))))

	// motor constraint
	if joint.enableSpinMotor && fixedRotation == false {
		spinAxis := matrixB.cz
		cdot := Dot(Sub(wB, wA), spinAxis) - joint.spinSpeed
		impulse := -joint.spinMass * cdot
		oldImpulse := joint.spinImpulse
		maxImpulse := context_.h * joint.maxSpinTorque
		joint.spinImpulse = ClampFloat(joint.spinImpulse + impulse, -maxImpulse, maxImpulse)
		impulse = joint.spinImpulse - oldImpulse

		wA = Sub(wA, MulMV(iA, MulSV(impulse, spinAxis)))
		wB = Add(wB, MulMV(iB, MulSV(impulse, spinAxis)))
	}

	// suspension
	if joint.enableSuspensionSpring {
		// This is a real spring and should be applied even during relax
		c := translation
		bias := joint.suspensionSoftness.biasRate * c
		massScale := joint.suspensionSoftness.massScale
		impulseScale := joint.suspensionSoftness.impulseScale

		cdot := Dot(matrixA.cx, Sub(vB, vA)) + Dot(sBx, wB) - Dot(sAx, wA)
		impulse := -massScale * joint.suspensionMass * (cdot + bias) - impulseScale * joint.suspensionSpringImpulse
		joint.suspensionSpringImpulse += impulse

		linearImpulse := MulSV(impulse, matrixA.cx)
		angularImpulseA := MulSV(impulse, sAx)
		angularImpulseB := MulSV(impulse, sBx)

		vA = MulSub(vA, mA, linearImpulse)
		wA = Sub(wA, MulMV(iA, angularImpulseA))
		vB = MulAdd(vB, mB, linearImpulse)
		wB = Add(wB, MulMV(iB, angularImpulseB))
	}

	// steering
	if joint.enableSteering && fixedRotation == false {
		steeringAngle := Atan2(ss, cs)

		{
			// This is a real spring and should be applied even during relax
			c := steeringAngle - joint.targetSteeringAngle
			bias := joint.steeringSoftness.biasRate * c
			massScale := joint.steeringSoftness.massScale
			impulseScale := joint.steeringSoftness.impulseScale

			cdot := Dot(steeringAxis, Sub(wB, wA))
			oldImpulse := joint.steeringSpringImpulse
			impulse := -massScale * joint.steeringMass * (cdot + bias) - impulseScale * oldImpulse
			maxImpulse := context_.h * joint.maxSteeringTorque
			joint.steeringSpringImpulse = ClampFloat(oldImpulse + impulse, -maxImpulse, maxImpulse)
			impulse = joint.steeringSpringImpulse - oldImpulse

			wA = Sub(wA, MulMV(iA, MulSV(impulse, steeringAxis)))
			wB = Add(wB, MulMV(iB, MulSV(impulse, steeringAxis)))
		}

		if joint.enableSteeringLimit {
			// Lower limit
			{
				c := steeringAngle - joint.lowerSteeringLimit
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

				cdot := Dot(steeringAxis, Sub(wB, wA))
				oldImpulse := joint.lowerSteeringImpulse
				impulse := -massScale * joint.steeringMass * (cdot + bias) - impulseScale * oldImpulse
				joint.lowerSteeringImpulse = MaxFloat(oldImpulse + impulse, 0.0)
				impulse = joint.lowerSteeringImpulse - oldImpulse

				wA = Sub(wA, MulMV(iA, MulSV(impulse, steeringAxis)))
				wB = Add(wB, MulMV(iB, MulSV(impulse, steeringAxis)))
			}

			// Upper limit
			// Note: signs are flipped to keep c positive when the constraint is satisfied.
			// This also keeps the impulse positive when the limit is active.
			{
				// sign flipped
				c := joint.upperSteeringLimit - steeringAngle
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

				// sign flipped on cdot
				cdot := Dot(steeringAxis, Sub(wA, wB))
				oldImpulse := joint.upperSteeringImpulse
				impulse := -massScale * joint.steeringMass * (cdot + bias) - impulseScale * oldImpulse
				joint.upperSteeringImpulse = MaxFloat(oldImpulse + impulse, 0.0)
				impulse = joint.upperSteeringImpulse - oldImpulse

				// sign flipped on applied impulse
				wA = Add(wA, MulMV(iA, MulSV(impulse, steeringAxis)))
				wB = Sub(wB, MulMV(iB, MulSV(impulse, steeringAxis)))
			}
		}
	}

	if joint.enableSuspensionLimit {
		// Lower limit
		{
			c := translation - joint.lowerSuspensionLimit
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

			cdot := Dot(matrixA.cx, Sub(vB, vA)) + Dot(sBx, wB) - Dot(sAx, wA)
			impulse := -massScale * joint.suspensionMass * (cdot + bias) - impulseScale * joint.lowerSuspensionImpulse
			oldImpulse := joint.lowerSuspensionImpulse
			joint.lowerSuspensionImpulse = MaxFloat(oldImpulse + impulse, 0.0)
			impulse = joint.lowerSuspensionImpulse - oldImpulse

			linearImpulse := MulSV(impulse, matrixA.cx)
			angularImpulseA := MulSV(impulse, sAx)
			angularImpulseB := MulSV(impulse, sBx)

			vA = MulSub(vA, mA, linearImpulse)
			wA = Sub(wA, MulMV(iA, angularImpulseA))
			vB = MulAdd(vB, mB, linearImpulse)
			wB = Add(wB, MulMV(iB, angularImpulseB))
		}

		// Upper limit
		// Note: signs are flipped to keep c positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			// sign flipped
			c := joint.upperSuspensionLimit - translation
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

			// sign flipped on cdot
			cdot := Dot(matrixA.cx, Sub(vA, vB)) + Dot(sAx, wA) - Dot(sBx, wB)
			impulse := -massScale * joint.suspensionMass * (cdot + bias) - impulseScale * joint.upperSuspensionImpulse
			oldImpulse := joint.upperSuspensionImpulse
			joint.upperSuspensionImpulse = MaxFloat(oldImpulse + impulse, 0.0)
			impulse = joint.upperSuspensionImpulse - oldImpulse

			linearImpulse := MulSV(impulse, matrixA.cx)
			angularImpulseA := MulSV(impulse, sAx)
			angularImpulseB := MulSV(impulse, sBx)

			// sign flipped on applied impulse
			vA = MulAdd(vA, mA, linearImpulse)
			wA = Add(wA, MulMV(iA, angularImpulseA))
			vB = MulSub(vB, mB, linearImpulse)
			wB = Sub(wB, MulMV(iB, angularImpulseB))
		}
	}

	// Collinearity constraint
	if fixedRotation == false {
		if joint.enableSteering == true {
			bias: f32 = 0.0
			massScale: f32 = 1.0
			impulseScale: f32 = 0.0
			if useBias {
				c := Dot(matrixA.cx, matrixB.cz)

				bias = base.constraintSoftness.biasRate * c
				massScale = base.constraintSoftness.massScale
				impulseScale = base.constraintSoftness.impulseScale
			}

			u := Cross(matrixB.cz, matrixA.cx)
			cdot := Dot(Sub(wB, wA), u)

			invInertiaSum := AddMM(iA, iB)
			k := Dot(u, MulMV(invInertiaSum, u))
			perpMass: f32 = k > 0.0 ? 1.0 / k : 0.0

			deltaImpulse := -massScale * perpMass * (cdot + bias) - impulseScale * joint.angularImpulse.x
			joint.angularImpulse.x += deltaImpulse

			wA = MulSub(wA, deltaImpulse, MulMV(iA, u))
			wB = MulAdd(wB, deltaImpulse, MulMV(iB, u))
		} else {
			bias := Vec2{0.0, 0.0}
			massScale: f32 = 1.0
			impulseScale: f32 = 0.0

			if useBias {
				c := Vec2{relQ.v.x, relQ.v.y}
				bias = Vec2{base.constraintSoftness.biasRate * c.x, base.constraintSoftness.biasRate * c.y}
				massScale = base.constraintSoftness.massScale
				impulseScale = base.constraintSoftness.impulseScale
			}

			// Collinearity constraint as 2-by-2
			perpAxisX := MulSV(0.5, RotateVector(quatA, Add(MulSV(relQ.s, Vec3_axisX), Cross(relQ.v, Vec3_axisX))))
			perpAxisY := MulSV(0.5, RotateVector(quatA, Add(MulSV(relQ.s, Vec3_axisY), Cross(relQ.v, Vec3_axisY))))

			invInertiaSum := AddMM(iA, iB)
			kxx := Dot(perpAxisX, MulMV(invInertiaSum, perpAxisX))
			kyy := Dot(perpAxisY, MulMV(invInertiaSum, perpAxisY))
			kxy := Dot(perpAxisX, MulMV(invInertiaSum, perpAxisY))

			k := Matrix2{{kxx, kxy}, {kxy, kyy}}

			wRel := Sub(wB, wA)
			cdot := Vec2{Dot(wRel, perpAxisX), Dot(wRel, perpAxisY)}
			oldImpulse := joint.angularImpulse
			cdotPlusBias := Vec2{cdot.x + bias.x, cdot.y + bias.y}
			sol := Solve2(k, cdotPlusBias)
			deltaImpulse := Vec2 {
				-massScale * sol.x - impulseScale * oldImpulse.x,
				-massScale * sol.y - impulseScale * oldImpulse.y,
			}
			joint.angularImpulse = Vec2{oldImpulse.x + deltaImpulse.x, oldImpulse.y + deltaImpulse.y}

			angularImpulse := Blend2(deltaImpulse.x, perpAxisX, deltaImpulse.y, perpAxisY)
			wA = Sub(wA, MulMV(iA, angularImpulse))
			wB = Add(wB, MulMV(iB, angularImpulse))
		}
	}

	// Solve point-to-line constraint
	{
		perpY := matrixA.cy
		perpZ := matrixA.cz

		bias := Vec2{0.0, 0.0}
		massScale: f32 = 1.0
		impulseScale: f32 = 0.0
		if useBias {
			c := Vec2{Dot(perpY, d), Dot(perpZ, d)}
			bias = Vec2{base.constraintSoftness.biasRate * c.x, base.constraintSoftness.biasRate * c.y}
			massScale = base.constraintSoftness.massScale
			impulseScale = base.constraintSoftness.impulseScale
		}

		vRel := Sub(Sub(Add(vB, Cross(wB, rB)), vA), Cross(wA, Add(rA, d)))
		cdot := Vec2{Dot(perpY, vRel), Dot(perpZ, vRel)}

		//// K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
		///// Jx = [-perpX, -cross(d + rA, perpX), perpX, cross(rB, perpX)]

		kyy := mA + mB + Dot(sAy, MulMV(iA, sAy)) + Dot(sBy, MulMV(iB, sBy))
		kyz := Dot(sAy, MulMV(iA, sAz)) + Dot(sBy, MulMV(iB, sBz))
		kzz := mA + mB + Dot(sAz, MulMV(iA, sAz)) + Dot(sBz, MulMV(iB, sBz))

		k := Matrix2{{kyy, kyz}, {kyz, kzz}}

		oldImpulse := joint.linearImpulse
		cdotPlusBias := Vec2{cdot.x + bias.x, cdot.y + bias.y}
		sol := Solve2(k, cdotPlusBias)
		deltaImpulse := Vec2 {
			-massScale * sol.x - impulseScale * oldImpulse.x,
			-massScale * sol.y - impulseScale * oldImpulse.y,
		}
		joint.linearImpulse = Vec2{oldImpulse.x + deltaImpulse.x, oldImpulse.y + deltaImpulse.y}

		linearImpulse := Blend2(deltaImpulse.x, perpY, deltaImpulse.y, perpZ)

		vA = MulSub(vA, mA, linearImpulse)
		wA = Sub(wA, MulMV(iA, Blend2(deltaImpulse.x, sAy, deltaImpulse.y, sAz)))
		vB = MulAdd(vB, mB, linearImpulse)
		wB = Add(wB, MulMV(iB, Blend2(deltaImpulse.x, sBy, deltaImpulse.y, sBz)))
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

// unified: not ported: the disabled (#if 0) b3WheelJoint_Dump (a leftover that doesn't compile in C)

DrawWheelJoint :: proc(
	draw: ^DebugDraw,
	base: ^JointSim,
	transformA: WorldTransform,
	transformB: WorldTransform,
	scale: f32,
) {
	assert(base.type == .wheelJoint)

	joint := &base.wheelJoint

	frameA := MulWorldTransforms(transformA, base.localFrameA)
	frameB := MulWorldTransforms(transformB, base.localFrameB)

	matrixA := MakeMatrixFromQuat(frameA.q)
	matrixB := MakeMatrixFromQuat(frameB.q)

	draw.DrawSegmentFcn(frameA.p, frameB.p, .colorBlue, draw.context_)

	if joint.enableSuspensionLimit {
		lower := OffsetPos(frameA.p, MulSV(joint.lowerSuspensionLimit, matrixA.cx))
		upper := OffsetPos(frameA.p, MulSV(joint.upperSuspensionLimit, matrixA.cx))
		perp := matrixA.cy
		draw.DrawSegmentFcn(lower, upper, .colorGray, draw.context_)
		draw.DrawSegmentFcn(
			OffsetPos(lower, MulSV(-0.1 * scale, perp)),
			OffsetPos(lower, MulSV(0.1 * scale, perp)),
			.colorGreen,
			draw.context_,
		)
		draw.DrawSegmentFcn(
			OffsetPos(upper, MulSV(-0.1 * scale, perp)),
			OffsetPos(upper, MulSV(0.1 * scale, perp)),
			.colorRed,
			draw.context_,
		)
	} else {
		draw.DrawSegmentFcn(
			OffsetPos(frameA.p, MulSV(-1.0 * scale, matrixA.cx)),
			OffsetPos(frameA.p, MulSV(1.0 * scale, matrixA.cx)),
			.colorGray,
			draw.context_,
		)
	}

	if joint.enableSteering && joint.enableSteeringLimit {
		// b3Quat quatA = frameA.q;
		// b3Quat quatB = frameB.q;

		// if ( b3DotQuat( quatA, quatB ) < 0.0f )
		//{
		//	// this keeps the twist angle in the range [-pi, pi]
		//	quatB = -quatB;
		// }

		// b3Quat relQ = b3InvMulQuat( quatA, quatB );

		frame := WorldTransform {
			p = frameB.p,
			q = frameA.q,
		}

		radius := 0.5 * scale
		sliceCount :: 16
		lower := joint.lowerSteeringLimit
		upper := joint.upperSteeringLimit

		cs := ComputeCosSin(lower)
		delta := ComputeCosSin((upper - lower) / sliceCount)
		vertex1 := TransformWorldPoint(frame, Vec3{0.0, -radius * cs.sine, radius * cs.cosine})

		for index: i32 = 0; index < sliceCount; index += 1 {
			cosine := delta.cosine * cs.cosine - delta.sine * cs.sine
			sine := delta.sine * cs.cosine + delta.cosine * cs.sine
			cs.cosine = cosine
			cs.sine = sine

			vertex2 := TransformWorldPoint(frame, Vec3{0.0, -radius * cs.sine, radius * cs.cosine})

			if index == 0 {
				draw.DrawSegmentFcn(frame.p, vertex1, .colorCyan, draw.context_)
			}

			if index == sliceCount - 1 {
				draw.DrawSegmentFcn(vertex2, frame.p, .colorCyan, draw.context_)
			}
			draw.DrawSegmentFcn(vertex1, vertex2, .colorCyan, draw.context_)

			vertex1 = vertex2
		}
	}

	draw.DrawSegmentFcn(
		OffsetPos(frameB.p, MulSV(-0.5 * scale, matrixB.cz)),
		OffsetPos(frameB.p, MulSV(0.5 * scale, matrixB.cz)),
		.colorMagenta,
		draw.context_,
	)

	draw.DrawPointFcn(frameA.p, 5.0, .colorGray, draw.context_)
	draw.DrawPointFcn(frameB.p, 5.0, .colorDimGray, draw.context_)
}
