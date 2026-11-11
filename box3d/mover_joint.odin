// Ported from box3d src/mover_joint.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:intrinsics"

// B3_REC is expanded inline at each call site (`if world.recording != nil {...}`), as in body.odin.

MoverJoint_SetLinearVelocity :: proc(jointId: JointId, velocity: Vec3) {
	assert(IsValidVec3(velocity))
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_MoverJointSetLinearVelocity{jointId, velocity}
		RecWrite_MoverJointSetLinearVelocity(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .moverJoint)
	joint.moverJoint.linearVelocity = velocity
}

MoverJoint_GetLinearVelocity :: proc(jointId: JointId) -> Vec3 {
	joint := GetJointSimCheckType(jointId, .moverJoint)
	return joint.moverJoint.linearVelocity
}

MoverJoint_SetMaxVelocityForce :: proc(jointId: JointId, maxForce: f32) {
	assert(IsValidFloat(maxForce) && maxForce >= 0.0)
	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_MoverJointSetMaxVelocityForce{jointId, maxForce}
		RecWrite_MoverJointSetMaxVelocityForce(world.recording, &recArgs)
	}
	joint := GetJointSimCheckType(jointId, .moverJoint)
	joint.moverJoint.maxForce = maxForce
}

MoverJoint_GetMaxVelocityForce :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .moverJoint)
	return joint.moverJoint.maxForce
}

GetMoverJointForce :: proc(world: ^World, base: ^JointSim) -> Vec3 {
	joint := &base.moverJoint
	P := Blend2(joint.impulse.x, joint.axisX, joint.impulse.y, joint.axisZ)
	return MulSV(world.inv_h, P)
}

PrepareMoverJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .moverJoint)

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
	mB := bodySimB.invMass

	base.invMassA = mA
	base.invMassB = mB
	base.invIA = Mat3_zero
	base.invIB = Mat3_zero

	joint := &base.moverJoint
	joint.indexA = bodyA.setIndex == i32(SetType.awakeSet) ? localIndexA : NULL_INDEX
	joint.indexB = bodyB.setIndex == i32(SetType.awakeSet) ? localIndexB : NULL_INDEX

	quatB := MulQuat(bodySimB.transform.q, base.localFrameB.q)
	matrixB := MakeMatrixFromQuat(quatB)
	joint.axisX = matrixB.cx
	joint.axisZ = matrixB.cz

	k := mA + mB
	joint.linearMass = k > 0.0 ? 1.0 / k : 0.0

	if context_.enableWarmStarting == false {
		joint.impulse = Vec2{}
	}
}

WarmStartMoverJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .moverJoint)

	mA := base.invMassA
	mB := base.invMassB

	joint := &base.moverJoint

	// dummy state for static bodies
	dummyState := identityBodyState

	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	P := Blend2(joint.impulse.x, joint.axisX, joint.impulse.y, joint.axisZ)
	if (stateA.flags & dynamicFlag) != 0 {
		stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, P)
	}

	if (stateB.flags & dynamicFlag) != 0 {
		stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, P)
	}
}

SolveMoverJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .moverJoint)

	mA := base.invMassA
	mB := base.invMassB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.moverJoint
	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	vA := stateA.linearVelocity
	vB := stateB.linearVelocity

	maxForce := joint.maxForce
	vr := Sub(Sub(vB, vA), joint.linearVelocity)
	cdot := Vec2{Dot(vr, joint.axisX), Dot(vr, joint.axisZ)}
	deltaImpulse := MulSV2(-joint.linearMass, cdot)
	newImpulse := Vec2{joint.impulse.x + deltaImpulse.x, joint.impulse.y + deltaImpulse.y}

	maxImpulse := context_.h * maxForce

	lengthSquared := Dot2(newImpulse, newImpulse)
	if lengthSquared > maxImpulse * maxImpulse {
		scale := maxImpulse / intrinsics.sqrt(lengthSquared)
		newImpulse.x *= scale
		newImpulse.y *= scale
	}

	deltaImpulse = Sub2(newImpulse, joint.impulse)
	joint.impulse = newImpulse

	P := Blend2(deltaImpulse.x, joint.axisX, deltaImpulse.y, joint.axisZ)
	vA = MulSub(vA, mA, P)
	vB = MulAdd(vB, mB, P)

	if (stateA.flags & dynamicFlag) != 0 {
		stateA.linearVelocity = vA
	}

	if (stateB.flags & dynamicFlag) != 0 {
		stateB.linearVelocity = vB
	}
}

@(private = "file")
LerpColor :: proc(a: HexColor, b: HexColor, t: f32) -> HexColor {
	result: u32 = 0
	for shift: u32 = 0; shift < 24; shift += 8 {
		ca := f32((u32(a) >> shift) & 0xFF)
		cb := f32((u32(b) >> shift) & 0xFF)
		c := u32(LerpFloat(ca, cb, t) + 0.5)
		result |= c << shift
	}
	return HexColor(result)
}

@(private = "file")
DrawArrow :: proc(draw: ^DebugDraw, a: Pos, b: Pos, planeNormal: Vec3, size: f32, color: HexColor) {
	draw.DrawSegmentFcn(a, b, color, draw.context_)

	shaft := SubPos(b, a)
	shaftLen := Length(shaft)
	if shaftLen < 1e-6 {
		return
	}

	dir := Vec3{shaft.x / shaftLen, shaft.y / shaftLen, shaft.z / shaftLen}
	headLen := size

	perp := Normalize(Cross(planeNormal, dir))
	backFromTip := MulSV(-headLen, dir)
	sideStep := MulSV(headLen * 0.5, perp)
	tip1 := OffsetPos(b, Add(backFromTip, sideStep))
	tip2 := OffsetPos(b, Sub(backFromTip, sideStep))
	draw.DrawSegmentFcn(b, tip1, color, draw.context_)
	draw.DrawSegmentFcn(b, tip2, color, draw.context_)
}

DrawMoverJoint :: proc(
	draw: ^DebugDraw,
	world: ^World,
	base: ^JointSim,
	transformA: WorldTransform,
	transformB: WorldTransform,
) {
	assert(base.type == .moverJoint)
	_ = transformA

	joint := &base.moverJoint

	frameB := MulWorldTransforms(transformB, base.localFrameB)
	matrixB := MakeMatrixFromQuat(frameB.q)
	axisX := matrixB.cx
	axisZ := matrixB.cz

	bodyA := Array_Get(&world.bodies, base.bodyIdA)
	bodyB := Array_Get(&world.bodies, base.bodyIdB)
	stateA := GetBodyState(world, bodyA)
	stateB := GetBodyState(world, bodyB)
	vA := stateA != nil ? stateA.linearVelocity : Vec3_zero
	vB := stateB != nil ? stateB.linearVelocity : Vec3_zero

	velocityError := Sub(Sub(vB, vA), joint.linearVelocity)
	planarVelocityError := Vec2{Dot(velocityError, axisX), Dot(velocityError, axisZ)}
	targetVelocity := Vec2{Dot(joint.linearVelocity, axisX), Dot(joint.linearVelocity, axisZ)}

	// Colorize ring by ratio of velocity error magnitude to target speed.
	color := HexColor.colorWhite
	targetSpeed := Length2(targetVelocity)
	if targetSpeed > 0.0 {
		t := ClampFloat(Length2(planarVelocityError) / targetSpeed, 0.0, 1.0)
		color = LerpColor(.colorWhite, .colorBlue, t)
	}

	radius := draw.jointScale
	rx := MulSV(radius, axisX)
	rz := MulSV(radius, axisZ)
	segmentCount: i32 = 32
	cs := ComputeCosSin(f32(2.0) * PI / f32(segmentCount))
	x1, y1: f32 = 1.0, 0.0
	p1 := OffsetPos(frameB.p, rx)
	for i: i32 = 0; i < segmentCount; i += 1 {
		x2 := cs.cosine * x1 - cs.sine * y1
		y2 := cs.sine * x1 + cs.cosine * y1
		p2 := OffsetPos(frameB.p, Blend2(x2, rx, y2, rz))
		draw.DrawSegmentFcn(p1, p2, color, draw.context_)
		x1 = x2
		y1 = y2
		p1 = p2
	}

	if joint.maxForce > 0.0 {
		force := Blend2(joint.impulse.x, rx, joint.impulse.y, rz)
		dir := Normalize(force)
		origin := OffsetPos(frameB.p, MulSV(-radius, dir))
		scale := world.inv_h / joint.maxForce
		tip := OffsetPos(origin, MulSV(scale, force))
		DrawArrow(draw, origin, tip, matrixB.cy, 0.1 * draw.jointScale, .colorWhite)
	}
}
