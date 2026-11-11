// Ported from box3d src/prismatic_joint.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

// B3_REC is expanded inline at each call site (`if world.recording != nil {...}`), as in body.odin.

// Linear constraint (point-to-line)
// joint axis is along joint frame A local z-axis
// perpX and perpY are world vectors fixed in A
//
// d = pB - pA = xB + rB - xA - rA
// Cx = dot(perpX, d)
// Cy = dot(perpY, d)

// CdotX = dot(d, cross(wA, perpX)) + dot(perpX, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(perpX, vA) - dot(cross(d + rA, perpX), wA) + dot(perpX, vB) + dot(cross(rB, perpX), vB)
// Jx = [-perpX, -cross(d + rA, perpX), perpX, cross(rB, perpX)]
// similar for perpY
//
// Simplification dropping dot(d, cross(wA, perpX)) (todo needs testing)
// CdotXs = dot(perpX, vB + cross(wB, rB) - vA - cross(wA, rA))
// Jxs = [-perpX, -cross(rA, perpX), perpX, cross(rB, perpX)]

// Motor/limit/spring linear constraint
// axis is the world joint axis fixed in A

// C = dot(axis, d)
// Cdot = dot(d, cross(wA, axis)) + dot(axis, vB + cross(wB, rB) - vA - cross(wA, rA))
// Cdot = -dot(axis, vA) - dot(cross(d + rA, axis), wA) + dot(axis, vB) + dot(cross(rB, axis), vB)
// J = [-axis -cross(d + rA, axis) axis cross(rB, axis)]
//
// Simplified (todo needs testing)
// Cdot = -dot(axis, vA) - dot(cross(rA, axis), wA) + dot(axis, vB) + dot(cross(rB, axis), vB)
// J = [-axis -cross(rA, axis) axis cross(rB, axis)]

// Predictive limit is applied even when the limit is not active.
// Prevents a constraint speed that can lead to a constraint error in one time step.
// Want C2 = C1 + h * Cdot >= 0
// Or:
// Cdot + C1/h >= 0
// I do not apply a negative constraint error because that is handled in position correction.
// So:
// Cdot + max(C1, 0)/h >= 0

PrismaticJoint_EnableLimit :: proc(jointId: JointId, enableLimit: bool) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_PrismaticJointEnableLimit{jointId, enableLimit}
		RecWrite_PrismaticJointEnableLimit(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	if enableLimit != base.prismaticJoint.enableLimit {
		base.prismaticJoint.lowerImpulse = 0.0
		base.prismaticJoint.upperImpulse = 0.0
	}
	base.prismaticJoint.enableLimit = enableLimit
}

PrismaticJoint_IsLimitEnabled :: proc(jointId: JointId) -> bool {
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	return base.prismaticJoint.enableLimit
}

PrismaticJoint_GetLowerLimit :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	return base.prismaticJoint.lowerTranslation
}

PrismaticJoint_GetUpperLimit :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	return base.prismaticJoint.upperTranslation
}

PrismaticJoint_SetLimits :: proc(jointId: JointId, lower: f32, upper: f32) {
	assert(IsValidFloat(lower) && IsValidFloat(upper))
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_PrismaticJointSetLimits{jointId, lower, upper}
		RecWrite_PrismaticJointSetLimits(world.recording, &recArgs)
	}
	lowerAngle := MinFloat(lower, upper)
	upperAngle := MaxFloat(lower, upper)

	base := GetJointSimCheckType(jointId, .prismaticJoint)
	base.prismaticJoint.lowerTranslation = lowerAngle
	base.prismaticJoint.upperTranslation = upperAngle
}

PrismaticJoint_GetTranslation :: proc(jointId: JointId) -> f32 {
	world := GetWorld(i32(jointId.world0))
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	transformA := GetBodyTransform(world, base.bodyIdA)
	transformB := GetBodyTransform(world, base.bodyIdB)

	jointAxis := RotateVector(base.localFrameA.q, Vec3_axisX)
	jointAxis = RotateVector(transformA.q, jointAxis)

	anchorA := RotateVector(transformA.q, base.localFrameA.p)
	anchorB := RotateVector(transformB.q, base.localFrameB.p)
	d := Add(SubPos(transformB.p, transformA.p), Sub(anchorB, anchorA))
	translation := Dot(d, jointAxis)
	return translation
}

PrismaticJoint_EnableSpring :: proc(jointId: JointId, enableSpring: bool) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_PrismaticJointEnableSpring{jointId, enableSpring}
		RecWrite_PrismaticJointEnableSpring(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	if enableSpring != base.prismaticJoint.enableSpring {
		base.prismaticJoint.springImpulse = 0.0
	}
	base.prismaticJoint.enableSpring = enableSpring
}

PrismaticJoint_IsSpringEnabled :: proc(jointId: JointId) -> bool {
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	return base.prismaticJoint.enableSpring
}

PrismaticJoint_SetTargetTranslation :: proc(jointId: JointId, targetTranslation: f32) {
	assert(IsValidFloat(targetTranslation))
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_PrismaticJointSetTargetTranslation{jointId, targetTranslation}
		RecWrite_PrismaticJointSetTargetTranslation(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	base.prismaticJoint.targetTranslation = targetTranslation
}

PrismaticJoint_GetTargetTranslation :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	return base.prismaticJoint.targetTranslation
}

PrismaticJoint_SetSpringHertz :: proc(jointId: JointId, hertz: f32) {
	assert(IsValidFloat(hertz) && hertz >= 0.0)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_PrismaticJointSetSpringHertz{jointId, hertz}
		RecWrite_PrismaticJointSetSpringHertz(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	base.prismaticJoint.hertz = hertz
}

PrismaticJoint_GetSpringHertz :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	return base.prismaticJoint.hertz
}

PrismaticJoint_SetSpringDampingRatio :: proc(jointId: JointId, dampingRatio: f32) {
	assert(IsValidFloat(dampingRatio) && dampingRatio >= 0.0)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_PrismaticJointSetSpringDampingRatio{jointId, dampingRatio}
		RecWrite_PrismaticJointSetSpringDampingRatio(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	base.prismaticJoint.dampingRatio = dampingRatio
}

PrismaticJoint_GetSpringDampingRatio :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	return base.prismaticJoint.dampingRatio
}

PrismaticJoint_EnableMotor :: proc(jointId: JointId, enableMotor: bool) {
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_PrismaticJointEnableMotor{jointId, enableMotor}
		RecWrite_PrismaticJointEnableMotor(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	if enableMotor != base.prismaticJoint.enableMotor {
		base.prismaticJoint.motorImpulse = 0.0
	}
	base.prismaticJoint.enableMotor = enableMotor
}

PrismaticJoint_IsMotorEnabled :: proc(jointId: JointId) -> bool {
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	return base.prismaticJoint.enableMotor
}

PrismaticJoint_SetMotorSpeed :: proc(jointId: JointId, motorSpeed: f32) {
	assert(IsValidFloat(motorSpeed))
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_PrismaticJointSetMotorSpeed{jointId, motorSpeed}
		RecWrite_PrismaticJointSetMotorSpeed(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	base.prismaticJoint.motorSpeed = motorSpeed
}

PrismaticJoint_GetMotorSpeed :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	return base.prismaticJoint.motorSpeed
}

PrismaticJoint_SetMaxMotorForce :: proc(jointId: JointId, maxForce: f32) {
	assert(IsValidFloat(maxForce) && maxForce >= 0.0)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_PrismaticJointSetMaxMotorForce{jointId, maxForce}
		RecWrite_PrismaticJointSetMaxMotorForce(world.recording, &recArgs)
	}
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	base.prismaticJoint.maxMotorForce = maxForce
}

PrismaticJoint_GetMaxMotorForce :: proc(jointId: JointId) -> f32 {
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	return base.prismaticJoint.maxMotorForce
}

PrismaticJoint_GetMotorForce :: proc(jointId: JointId) -> f32 {
	world := GetWorld(i32(jointId.world0))
	base := GetJointSimCheckType(jointId, .prismaticJoint)
	return world.inv_h * base.prismaticJoint.motorImpulse
}

PrismaticJoint_GetSpeed :: proc(jointId: JointId) -> f32 {
	world := GetWorld(i32(jointId.world0))
	base := GetJointSimCheckType(jointId, .prismaticJoint)

	bodyA := Array_Get(&world.bodies, base.bodyIdA)
	bodyB := Array_Get(&world.bodies, base.bodyIdB)
	bodySimA := GetBodySim(world, bodyA)
	bodySimB := GetBodySim(world, bodyB)
	stateA := GetBodyState(world, bodyA)
	stateB := GetBodyState(world, bodyB)

	qA := bodySimA.transform.q
	qB := bodySimB.transform.q

	axisA := RotateVector(qA, RotateVector(base.localFrameA.q, Vec3_axisX))
	rA := RotateVector(qA, Sub(base.localFrameA.p, bodySimA.localCenter))
	rB := RotateVector(qB, Sub(base.localFrameB.p, bodySimB.localCenter))

	// Difference the centers in double so the speed stays exact far from the origin.
	d := Add(SubPos(bodySimB.center, bodySimA.center), Sub(rB, rA))

	vA := stateA != nil ? stateA.linearVelocity : Vec3_zero
	vB := stateB != nil ? stateB.linearVelocity : Vec3_zero
	wA := stateA != nil ? stateA.angularVelocity : Vec3_zero
	wB := stateB != nil ? stateB.angularVelocity : Vec3_zero

	vRel := Sub(Add(vB, Cross(wB, rB)), Add(vA, Cross(wA, rA)))

	// The axis moves with body A, so account for its rotation.
	speed := Dot(d, Cross(wA, axisA)) + Dot(axisA, vRel)
	return speed
}

GetPrismaticJointForce :: proc(world: ^World, base: ^JointSim) -> Vec3 {
	transformA := GetBodyTransform(world, base.bodyIdA)
	joint := &base.prismaticJoint

	// impulse in joint space
	impulse := Vec3 {
		joint.perpImpulse.x,
		joint.perpImpulse.y,
		joint.motorImpulse + joint.lowerImpulse + joint.upperImpulse + joint.springImpulse,
	}

	// convert impulse to force
	force := MulSV(world.inv_h, impulse)

	// convert to body space
	force = RotateVector(base.localFrameA.q, force)

	// convert to world space
	force = RotateVector(transformA.q, force)
	return force
}

GetPrismaticJointTorque :: proc(world: ^World, base: ^JointSim) -> Vec3 {
	transformA := GetBodyTransform(world, base.bodyIdA)
	joint := &base.prismaticJoint

	torque := MulSV(world.inv_h, joint.angularImpulse)
	torque = RotateVector(base.localFrameA.q, torque)
	torque = RotateVector(transformA.q, torque)
	return torque
}

PreparePrismaticJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .prismaticJoint)

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

	joint := &base.prismaticJoint
	joint.indexA = bodyA.setIndex == i32(SetType.awakeSet) ? localIndexA : NULL_INDEX
	joint.indexB = bodyB.setIndex == i32(SetType.awakeSet) ? localIndexB : NULL_INDEX

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.q = MulQuat(bodySimA.transform.q, base.localFrameA.q)
	joint.frameA.p = RotateVector(bodySimA.transform.q, Sub(base.localFrameA.p, bodySimA.localCenter))
	joint.frameB.q = MulQuat(bodySimB.transform.q, base.localFrameB.q)
	joint.frameB.p = RotateVector(bodySimB.transform.q, Sub(base.localFrameB.p, bodySimB.localCenter))

	joint.deltaCenter = SubPos(bodySimB.center, bodySimA.center)
	joint.rotationMass = InvertMatrix(invInertiaSum)

	// Initial joint axes in world space
	matrixA := MakeMatrixFromQuat(joint.frameA.q)
	joint.jointAxis = matrixA.cx
	joint.perpAxisY = matrixA.cy
	joint.perpAxisZ = matrixA.cz

	joint.springSoftness = MakeSoft(joint.hertz, joint.dampingRatio, context_.h)

	if context_.enableWarmStarting == false {
		joint.perpImpulse = Vec2{0.0, 0.0}
		joint.angularImpulse = Vec3{0.0, 0.0, 0.0}
		joint.motorImpulse = 0.0
		joint.springImpulse = 0.0
		joint.lowerImpulse = 0.0
		joint.upperImpulse = 0.0
	}
}

WarmStartPrismaticJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .prismaticJoint)

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.prismaticJoint

	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	// todo make this code and the wheel joint more similar

	rA := RotateVector(stateA.deltaRotation, joint.frameA.p)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.p)
	d := Add(Add(Sub(stateB.deltaPosition, stateA.deltaPosition), joint.deltaCenter), Sub(rB, rA))
	jointAxis := RotateVector(stateA.deltaRotation, joint.jointAxis)
	sAx := Cross(Add(rA, d), jointAxis)
	sBx := Cross(rB, jointAxis)

	perpY := RotateVector(stateA.deltaRotation, joint.perpAxisY)
	perpZ := RotateVector(stateA.deltaRotation, joint.perpAxisZ)
	sAy := Cross(Add(rA, d), perpY)
	sBy := Cross(rB, perpY)
	sAz := Cross(Add(rA, d), perpZ)
	sBz := Cross(rB, perpZ)

	axialImpulse := joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse
	perpImpulse := joint.perpImpulse

	P := Blend3(axialImpulse, jointAxis, perpImpulse.x, perpY, perpImpulse.y, perpZ)
	LA := Add(Blend3(axialImpulse, sAx, perpImpulse.x, sAy, perpImpulse.y, sAz), joint.angularImpulse)
	LB := Add(Blend3(axialImpulse, sBx, perpImpulse.x, sBy, perpImpulse.y, sBz), joint.angularImpulse)

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity
	vA = MulSub(vA, mA, P)
	wA = Sub(wA, MulMV(iA, LA))
	vB = MulAdd(vB, mB, P)
	wB = Add(wB, MulMV(iB, LB))

	if (stateA.flags & dynamicFlag) != 0 {
		stateA.linearVelocity = vA
		stateA.angularVelocity = wA
	}

	if (stateB.flags & dynamicFlag) != 0 {
		stateB.linearVelocity = vB
		stateB.angularVelocity = wB
	}
}

SolvePrismaticJoint :: proc(base: ^JointSim, context_: ^StepContext, useBias: bool) {
	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.prismaticJoint
	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity

	fixedRotation := base.fixedRotation
	rA := RotateVector(stateA.deltaRotation, joint.frameA.p)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.p)

	dcA := stateA.deltaPosition
	dcB := stateB.deltaPosition
	d := Add(Add(Sub(dcB, dcA), joint.deltaCenter), Sub(rB, rA))

	jointAxis := RotateVector(stateA.deltaRotation, joint.jointAxis)
	sAx := Cross(Add(rA, d), jointAxis)
	sBx := Cross(rB, jointAxis)
	jointTranslation := Dot(d, jointAxis)
	targetTranslation := joint.targetTranslation

	// The axial effective mass must be fresh to avoid divergence when the joint is stressed
	ka := mA + mB + Dot(sAx, MulMV(iA, sAx)) + Dot(sBx, MulMV(iB, sBx))
	axialMass := ka > 0.0 ? 1.0 / ka : 0.0

	// Solve spring
	if joint.enableSpring && fixedRotation == false {
		// Get the substep relative rotation
		c := jointTranslation - targetTranslation

		bias := joint.springSoftness.biasRate * c
		massScale := joint.springSoftness.massScale
		impulseScale := joint.springSoftness.impulseScale

		vRel := Sub(Sub(Add(vB, Cross(wB, rB)), vA), Cross(wA, Add(rA, d)))
		cdot := Dot(vRel, jointAxis)
		deltaImpulse := -massScale * axialMass * (cdot + bias) - impulseScale * joint.springImpulse
		joint.springImpulse += deltaImpulse

		P := MulSV(deltaImpulse, jointAxis)
		LA := MulSV(deltaImpulse, sAx)
		LB := MulSV(deltaImpulse, sBx)

		vA = MulSub(vA, mA, P)
		wA = Sub(wA, MulMV(iA, LA))
		vB = MulAdd(vB, mB, P)
		wB = Add(wB, MulMV(iB, LB))
	}

	if joint.enableMotor && fixedRotation == false {
		vRel := Sub(Sub(Add(vB, Cross(wB, rB)), vA), Cross(wA, Add(rA, d)))
		cdot := Dot(vRel, jointAxis) - joint.motorSpeed

		deltaImpulse := -axialMass * cdot
		newImpulse := joint.motorImpulse + deltaImpulse
		maxImpulse := joint.maxMotorForce * context_.h
		newImpulse = ClampFloat(newImpulse, -maxImpulse, maxImpulse)
		deltaImpulse = newImpulse - joint.motorImpulse
		joint.motorImpulse = newImpulse

		P := MulSV(deltaImpulse, jointAxis)
		LA := MulSV(deltaImpulse, sAx)
		LB := MulSV(deltaImpulse, sBx)

		vA = MulSub(vA, mA, P)
		wA = Sub(wA, MulMV(iA, LA))
		vB = MulAdd(vB, mB, P)
		wB = Add(wB, MulMV(iB, LB))
	}

	if joint.enableLimit && fixedRotation == false {
		speculativeDistance := 0.25 * (joint.upperTranslation - joint.lowerTranslation)

		// Lower limit
		{
			C := jointTranslation - joint.lowerTranslation

			if C < speculativeDistance {
				bias: f32 = 0.0
				massScale: f32 = 1.0
				impulseScale: f32 = 0.0
				if C > 0.0 {
					// speculation
					bias = C * context_.inv_h
				} else if useBias {
					bias = base.constraintSoftness.biasRate * C
					massScale = base.constraintSoftness.massScale
					impulseScale = base.constraintSoftness.impulseScale
				}

				vRel := Sub(Sub(Add(vB, Cross(wB, rB)), vA), Cross(wA, Add(rA, d)))
				cdot := Dot(vRel, jointAxis)
				oldImpulse := joint.lowerImpulse
				deltaImpulse := -massScale * axialMass * (cdot + bias) - impulseScale * oldImpulse
				joint.lowerImpulse = MaxFloat(oldImpulse + deltaImpulse, 0.0)
				deltaImpulse = joint.lowerImpulse - oldImpulse

				P := MulSV(deltaImpulse, jointAxis)
				LA := MulSV(deltaImpulse, sAx)
				LB := MulSV(deltaImpulse, sBx)

				vA = MulSub(vA, mA, P)
				wA = Sub(wA, MulMV(iA, LA))
				vB = MulAdd(vB, mB, P)
				wB = Add(wB, MulMV(iB, LB))
			} else {
				joint.lowerImpulse = 0.0
			}
		}

		// Upper limit
		{
			C := joint.upperTranslation - jointTranslation

			if C < speculativeDistance {
				bias: f32 = 0.0
				massScale: f32 = 1.0
				impulseScale: f32 = 0.0
				if C > 0.0 {
					// speculation
					bias = C * context_.inv_h
				} else if useBias {
					bias = base.constraintSoftness.biasRate * C
					massScale = base.constraintSoftness.massScale
					impulseScale = base.constraintSoftness.impulseScale
				}

				// sign flipped on Cdot
				vRel := Sub(Sub(Add(vB, Cross(wB, rB)), vA), Cross(wA, Add(rA, d)))
				cdot := -Dot(vRel, jointAxis)
				oldImpulse := joint.upperImpulse
				deltaImpulse := -massScale * axialMass * (cdot + bias) - impulseScale * oldImpulse
				joint.upperImpulse = MaxFloat(oldImpulse + deltaImpulse, 0.0)

				// sign flipped on applied impulse
				negDeltaImpulse := oldImpulse - joint.upperImpulse
				P := MulSV(negDeltaImpulse, jointAxis)
				LA := MulSV(negDeltaImpulse, sAx)
				LB := MulSV(negDeltaImpulse, sBx)

				vA = MulSub(vA, mA, P)
				wA = Sub(wA, MulMV(iA, LA))
				vB = MulAdd(vB, mB, P)
				wB = Add(wB, MulMV(iB, LB))
			} else {
				joint.upperImpulse = 0.0
			}
		}
	}

	// Rotation constraint
	if fixedRotation == false {
		bias := Vec3{0.0, 0.0, 0.0}
		massScale: f32 = 1.0
		impulseScale: f32 = 0.0

		if useBias {
			quatA := MulQuat(stateA.deltaRotation, joint.frameA.q)
			quatB := MulQuat(stateB.deltaRotation, joint.frameB.q)

			relQ := InvMulQuat(quatA, quatB)
			targetQuat := Quat_identity
			deltaRotation := DeltaQuatToRotation(relQ, targetQuat)
			c := Neg(RotateVector(quatA, deltaRotation))

			bias = MulSV(base.constraintSoftness.biasRate, c)
			massScale = base.constraintSoftness.massScale
			impulseScale = base.constraintSoftness.impulseScale
		}

		cdot := Sub(wB, wA)
		impulse := Sub(
			MulSV(-massScale, MulMV(joint.rotationMass, Add(cdot, bias))),
			MulSV(impulseScale, joint.angularImpulse),
		)
		joint.angularImpulse = Add(joint.angularImpulse, impulse)

		wA = Sub(wA, MulMV(iA, impulse))
		wB = Add(wB, MulMV(iB, impulse))
	}

	// Solve point-to-line constraint
	{
		perpY := RotateVector(stateA.deltaRotation, joint.perpAxisY)
		perpZ := RotateVector(stateA.deltaRotation, joint.perpAxisZ)

		bias := Vec2{0.0, 0.0}
		massScale: f32 = 1.0
		impulseScale: f32 = 0.0
		if useBias {
			c := Vec2{Dot(perpY, d), Dot(perpZ, d)}
			bias = MulSV2(base.constraintSoftness.biasRate, c)
			massScale = base.constraintSoftness.massScale
			impulseScale = base.constraintSoftness.impulseScale
		}

		vRel := Sub(Sub(Add(vB, Cross(wB, rB)), vA), Cross(wA, Add(rA, d)))
		cdot := Vec2{Dot(perpY, vRel), Dot(perpZ, vRel)}

		// K = [(1/mA + 1/mB) * eye(2) - skew(rA) * invIA * skew(rA) - skew(rB) * invIB * skew(rB)]
		// Jx = [-perpX, -cross(d + rA, perpX), perpX, cross(rB, perpX)]
		sAy := Cross(Add(rA, d), perpY)
		sBy := Cross(rB, perpY)
		sAz := Cross(Add(rA, d), perpZ)
		sBz := Cross(rB, perpZ)

		kyy := mA + mB + Dot(sAy, MulMV(iA, sAy)) + Dot(sBy, MulMV(iB, sBy))
		kyz := Dot(sAy, MulMV(iA, sAz)) + Dot(sBy, MulMV(iB, sBz))
		kzz := mA + mB + Dot(sAz, MulMV(iA, sAz)) + Dot(sBz, MulMV(iB, sBz))

		K := Matrix2{{kyy, kyz}, {kyz, kzz}}

		oldImpulse := joint.perpImpulse
		sol := Solve2(K, Add2(cdot, bias))
		deltaImpulse := Sub2(MulSV2(-massScale, sol), MulSV2(impulseScale, oldImpulse))
		joint.perpImpulse = Add2(oldImpulse, deltaImpulse)

		P := Blend2(deltaImpulse.x, perpY, deltaImpulse.y, perpZ)

		vA = MulSub(vA, mA, P)
		wA = Sub(wA, MulMV(iA, Blend2(deltaImpulse.x, sAy, deltaImpulse.y, sAz)))
		vB = MulAdd(vB, mB, P)
		wB = Add(wB, MulMV(iB, Blend2(deltaImpulse.x, sBy, deltaImpulse.y, sBz)))
	}

	assert(IsValidVec3(vA))
	assert(IsValidVec3(wA))
	assert(IsValidVec3(vB))
	assert(IsValidVec3(wB))

	if (stateA.flags & dynamicFlag) != 0 {
		stateA.linearVelocity = vA
		stateA.angularVelocity = wA
	}

	if (stateB.flags & dynamicFlag) != 0 {
		stateB.linearVelocity = vB
		stateB.angularVelocity = wB
	}
}

DrawPrismaticJoint :: proc(
	draw: ^DebugDraw,
	base: ^JointSim,
	transformA: WorldTransform,
	transformB: WorldTransform,
	scale: f32,
) {
	frameA := MulWorldTransforms(transformA, base.localFrameA)
	frameB := MulWorldTransforms(transformB, base.localFrameB)

	R := MakeMatrixFromQuat(frameA.q)
	axis := R.cx
	perpY := R.cy
	perpZ := R.cz

	s := 0.2 * scale
	draw.DrawSegmentFcn(frameA.p, OffsetPos(frameA.p, MulSV(s, perpY)), .colorGreen, draw.context_)
	draw.DrawSegmentFcn(frameA.p, OffsetPos(frameA.p, MulSV(s, perpZ)), .colorBlue, draw.context_)

	joint := &base.prismaticJoint
	if joint.enableLimit {
		p1 := OffsetPos(frameA.p, MulSV(joint.lowerTranslation, axis))
		p2 := OffsetPos(frameA.p, MulSV(joint.upperTranslation, axis))
		draw.DrawSegmentFcn(p1, p2, .colorOrange, draw.context_)
		draw.DrawPointFcn(p1, 10.0, .colorGreen, draw.context_)
		draw.DrawPointFcn(p2, 10.0, .colorRed, draw.context_)
	} else {
		p1 := OffsetPos(frameA.p, MulSV(-0.5 * scale, axis))
		p2 := OffsetPos(frameA.p, MulSV(0.5 * scale, axis))
		draw.DrawSegmentFcn(p1, p2, .colorOrange, draw.context_)
	}

	draw.DrawPointFcn(frameB.p, 8.0, .colorViolet, draw.context_)
}
