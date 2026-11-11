// Ported from box3d src/pogo_joint.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "core:fmt"

// B3_REC is expanded inline at each call site (`if world.recording != nil {...}`), as in body.odin.

PogoJoint_SetRestLength :: proc(jointId: JointId, length: f32) {
	assert(IsValidFloat(length) && length >= 0.0)
	joint := GetJointSimCheckType(jointId, .pogoJoint)
	if joint.pogoJoint.restLength == length {
		return
	}

	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_PogoJointSetRestLength{jointId, length}
		RecWrite_PogoJointSetRestLength(world.recording, &recArgs)
	}
	joint.pogoJoint.restLength = length
}

PogoJoint_GetRestLength :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .pogoJoint)
	return joint.pogoJoint.restLength
}

PogoJoint_SetSpringHertz :: proc(jointId: JointId, hertz: f32) {
	assert(IsValidFloat(hertz) && hertz >= 0.0)
	joint := GetJointSimCheckType(jointId, .pogoJoint)
	if joint.pogoJoint.hertz == hertz {
		return
	}

	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_PogoJointSetSpringHertz{jointId, hertz}
		RecWrite_PogoJointSetSpringHertz(world.recording, &recArgs)
	}
	joint.pogoJoint.hertz = hertz
}

PogoJoint_GetSpringHertz :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .pogoJoint)
	return joint.pogoJoint.hertz
}

PogoJoint_SetSpringDampingRatio :: proc(jointId: JointId, dampingRatio: f32) {
	assert(IsValidFloat(dampingRatio) && dampingRatio >= 0.0)
	joint := GetJointSimCheckType(jointId, .pogoJoint)
	if joint.pogoJoint.dampingRatio == dampingRatio {
		return
	}

	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_PogoJointSetSpringDampingRatio{jointId, dampingRatio}
		RecWrite_PogoJointSetSpringDampingRatio(world.recording, &recArgs)
	}
	joint.pogoJoint.dampingRatio = dampingRatio
}

PogoJoint_GetSpringDampingRatio :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .pogoJoint)
	return joint.pogoJoint.dampingRatio
}

PogoJoint_SetMaxTensionForce :: proc(jointId: JointId, force: f32) {
	assert(IsValidFloat(force) && force >= 0.0)
	joint := GetJointSimCheckType(jointId, .pogoJoint)
	if joint.pogoJoint.maxTensionForce == force {
		return
	}

	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_PogoJointSetMaxTensionForce{jointId, force}
		RecWrite_PogoJointSetMaxTensionForce(world.recording, &recArgs)
	}
	joint.pogoJoint.maxTensionForce = force
}

PogoJoint_GetMaxTensionForce :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .pogoJoint)
	return joint.pogoJoint.maxTensionForce
}

PogoJoint_SetMaxCompressionForce :: proc(jointId: JointId, force: f32) {
	assert(IsValidFloat(force) && force >= 0.0)
	joint := GetJointSimCheckType(jointId, .pogoJoint)
	if joint.pogoJoint.maxCompressionForce == force {
		return
	}

	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_PogoJointSetMaxCompressionForce{jointId, force}
		RecWrite_PogoJointSetMaxCompressionForce(world.recording, &recArgs)
	}
	joint.pogoJoint.maxCompressionForce = force
}

PogoJoint_GetMaxCompressionForce :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .pogoJoint)
	return joint.pogoJoint.maxCompressionForce
}

PogoJoint_SetNormal :: proc(jointId: JointId, normal: Vec3) {
	assert(IsValidVec3(normal) && IsNormalized(normal))
	joint := GetJointSimCheckType(jointId, .pogoJoint)
	current := joint.pogoJoint.normal
	if current.x == normal.x && current.y == normal.y && current.z == normal.z {
		return
	}

	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_PogoJointSetNormal{jointId, normal}
		RecWrite_PogoJointSetNormal(world.recording, &recArgs)
	}
	joint.pogoJoint.normal = normal
}

PogoJoint_GetNormal :: proc(jointId: JointId) -> Vec3 {
	joint := GetJointSimCheckType(jointId, .pogoJoint)
	return joint.pogoJoint.normal
}

PogoJoint_GetCurrentLength :: proc(jointId: JointId) -> f32 {
	world := GetWorld(i32(jointId.world0))
	base := GetJointSimCheckType(jointId, .pogoJoint)

	worldTransformA := GetBodyTransform(world, base.bodyIdA)
	transformA := ToRelativeTransform(worldTransformA, worldTransformA.p)
	transformB := ToRelativeTransform(GetBodyTransform(world, base.bodyIdB), worldTransformA.p)

	axis := RotateVector(MulQuat(transformB.q, base.localFrameB.q), Vec3_axisY)
	pA := TransformPoint(transformA, base.localFrameA.p)
	pB := TransformPoint(transformB, base.localFrameB.p)
	return Dot(Sub(pB, pA), axis)
}

PogoJoint_GetImpulse :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .pogoJoint)
	return joint.pogoJoint.impulse
}

PogoJoint_GetCurrentSpeed :: proc(jointId: JointId) -> f32 {
	joint := GetJointSimCheckType(jointId, .pogoJoint)
	return joint.pogoJoint.velocity
}

GetPogoJointForce :: proc(world: ^World, base: ^JointSim) -> Vec3 {
	return MulSV(world.inv_h * base.pogoJoint.impulse, base.pogoJoint.normal)
}

PreparePogoJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .pogoJoint)

	world := context_.world

	// chase body id to the solver set where the body lives
	idA := base.bodyIdA
	idB := base.bodyIdB

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
	iA := bodySimA.invInertiaWorld
	iB := bodySimB.invInertiaWorld

	base.invMassA = mA
	base.invMassB = mB
	base.invIA = iA
	base.invIB = iB

	joint := &base.pogoJoint
	joint.indexA = bodyA.setIndex == i32(SetType.awakeSet) ? localIndexA : NULL_INDEX
	joint.indexB = bodyB.setIndex == i32(SetType.awakeSet) ? localIndexB : NULL_INDEX

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.q = MulQuat(bodySimA.transform.q, base.localFrameA.q)
	joint.frameA.p = RotateVector(bodySimA.transform.q, Sub(base.localFrameA.p, bodySimA.localCenter))
	joint.frameB.q = MulQuat(bodySimB.transform.q, base.localFrameB.q)
	joint.frameB.p = RotateVector(bodySimB.transform.q, Sub(base.localFrameB.p, bodySimB.localCenter))

	// Compute the initial center delta. Incremental position updates are relative to this.
	joint.deltaCenter = SubPos(bodySimB.center, bodySimA.center)

	rnA := Cross(joint.frameA.p, joint.normal)
	rnB := Cross(joint.frameB.p, joint.normal)
	k := mA + mB + Dot(rnA, MulMV(iA, rnA)) + Dot(rnB, MulMV(iB, rnB))
	joint.linearMass = k > 0.0 ? 1.0 / k : 0.0

	if context_.enableWarmStarting == false {
		joint.velocity = 0.0
		joint.impulse = 0.0
	}
}

WarmStartPogoJoint :: proc(base: ^JointSim, context_: ^StepContext) {
	assert(base.type == .pogoJoint)

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	joint := &base.pogoJoint

	if joint.hertz == 0.0 {
		joint.impulse = 0.0
		return
	}

	// dummy state for static bodies
	dummyState := identityBodyState

	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	rA := RotateVector(stateA.deltaRotation, joint.frameA.p)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.p)

	linearImpulse := MulSV(joint.impulse, joint.normal)

	if (stateA.flags & dynamicFlag) != 0 {
		stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, linearImpulse)
		stateA.angularVelocity = Sub(stateA.angularVelocity, MulMV(iA, Cross(rA, linearImpulse)))
	}

	if (stateB.flags & dynamicFlag) != 0 {
		stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, linearImpulse)
		stateB.angularVelocity = Add(stateB.angularVelocity, MulMV(iB, Cross(rB, linearImpulse)))
	}
}

SolvePogoJoint :: proc(base: ^JointSim, context_: ^StepContext, useBias: bool) {
	assert(base.type == .pogoJoint)

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.pogoJoint
	if joint.hertz == 0.0 {
		joint.impulse = 0.0
		joint.velocity = 0.0
		return
	}

	stateA := joint.indexA == NULL_INDEX ? &dummyState : &context_.states[joint.indexA]
	stateB := joint.indexB == NULL_INDEX ? &dummyState : &context_.states[joint.indexB]

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity

	rA := RotateVector(stateA.deltaRotation, joint.frameA.p)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.p)

	bias: f32 = 0.0
	if useBias {
		// This is a custom pogo position correction designed to avoid feeding velocity into
		// the mover body. Otherwise the mover can make huge jumps going up stairs. This still
		// works with continuous collision.

		dcA := stateA.deltaPosition
		dcB := stateB.deltaPosition
		d := Add(Add(Sub(dcB, dcA), Sub(rB, rA)), joint.deltaCenter)

		// The pogo axis is usually fixed as the world y-axis. Different than the ground normal.
		pogoAxis := RotateVector(joint.frameB.q, Vec3_axisY)
		c := Dot(pogoAxis, d) - joint.restLength

		// This could be divided by dot(joint->normal, pogoAxis) to account for the constraint
		// direction, but I'd rather diminish the pogo recovery rate than risk making it huge.
		joint.velocity = SpringDamper(joint.hertz, joint.dampingRatio, c, joint.velocity, context_.h)
		bias = -joint.velocity
	}

	vr := Sub(Add(vB, Cross(wB, rB)), Add(vA, Cross(wA, rA)))
	cdot := Dot(joint.normal, vr)

	maxTensionImpulse := context_.h * joint.maxTensionForce
	maxCompressionImpulse := context_.h * joint.maxCompressionForce
	oldImpulse := joint.impulse
	impulse := -joint.linearMass * (cdot + bias)
	joint.impulse = ClampFloat(joint.impulse + impulse, -maxTensionImpulse, maxCompressionImpulse)
	impulse = joint.impulse - oldImpulse

	P := MulSV(impulse, joint.normal)
	vA = MulSub(vA, mA, P)
	wA = Sub(wA, MulMV(iA, Cross(rA, P)))
	vB = MulAdd(vB, mB, P)
	wB = Add(wB, MulMV(iB, Cross(rB, P)))

	if (stateA.flags & dynamicFlag) != 0 {
		stateA.linearVelocity = vA
		stateA.angularVelocity = wA
	}

	if (stateB.flags & dynamicFlag) != 0 {
		stateB.linearVelocity = vB
		stateB.angularVelocity = wB
	}
}

DrawPogoJoint :: proc(
	draw: ^DebugDraw,
	world: ^World,
	base: ^JointSim,
	transformA: WorldTransform,
	transformB: WorldTransform,
	scale: f32,
) {
	assert(base.type == .pogoJoint)

	joint := &base.pogoJoint

	pA := TransformWorldPoint(transformA, base.localFrameA.p)
	pB := TransformWorldPoint(transformB, base.localFrameB.p)
	axis := RotateVector(MulQuat(transformB.q, base.localFrameB.q), Vec3_axisY)
	perp1 := Perp(axis)
	perp2 := Cross(axis, perp1)

	length := Dot(SubPos(pB, pA), axis)
	foot := OffsetPos(pB, MulSV(-length, axis))

	force := world.inv_h * joint.impulse

	// Tolerance for reaching max force.
	saturation: f32 = 0.99

	color: HexColor
	if joint.hertz == 0.0 {
		color = .colorGray
	} else if force > 0.0 && force >= saturation * joint.maxCompressionForce {
		color = .colorRed
	} else if force < 0.0 && -force >= saturation * joint.maxTensionForce {
		color = .colorRed
	} else if force > 0.0 {
		color = .colorLimeGreen
	} else if force < 0.0 {
		color = .colorDeepSkyBlue
	} else {
		color = .colorLightGray
	}

	// Spring
	segmentCount: i32 = 12
	width := 0.1 * scale
	p1 := pB
	for i: i32 = 1; i <= segmentCount; i += 1 {
		side: f32 = i == segmentCount ? 0.0 : ((i & 1) != 0 ? width : -width)
		t := f32(i) / f32(segmentCount)
		p2 := OffsetPos(pB, MulAdd(MulSV(-t * length, axis), side, perp2))
		draw.DrawSegmentFcn(p1, p2, color, draw.context_)
		p1 = p2
	}

	// Ground plane
	rest := OffsetPos(pB, MulSV(-joint.restLength, axis))
	tick := 1.5 * width
	draw.DrawSegmentFcn(
		OffsetPos(rest, MulSV(-tick, perp1)),
		OffsetPos(rest, MulSV(tick, perp1)),
		.colorWhite,
		draw.context_,
	)
	draw.DrawSegmentFcn(
		OffsetPos(rest, MulSV(-tick, perp2)),
		OffsetPos(rest, MulSV(tick, perp2)),
		.colorWhite,
		draw.context_,
	)

	if Length(SubPos(pA, foot)) > LINEAR_SLOP {
		draw.DrawSegmentFcn(foot, pA, .colorDimGray, draw.context_)
	}

	draw.DrawPointFcn(pA, 6.0, color, draw.context_)
	draw.DrawSegmentFcn(pA, OffsetPos(pA, MulSV(0.5 * scale, joint.normal)), .colorOrange, draw.context_)

	if draw.drawJointExtras {
		buffer: [96]u8
		// unified: snprintf is Odin's fmt into the buffer, NUL-terminated (a debug label only)
		s := fmt.bprintf(
			buffer[:len(buffer) - 1],
			"length = %.3f, rest = %.3f, f = %.1f, v = %.3f",
			length,
			joint.restLength,
			force,
			joint.velocity,
		)
		buffer[len(s)] = 0
		// unified end
		draw.DrawStringFcn(pB, cstring(raw_data(buffer[:])), color, draw.context_)
	}
}
