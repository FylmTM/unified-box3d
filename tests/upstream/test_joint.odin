// Ported from box3d test/test_joint.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import "core:testing"

import b3 "../../box3d"

// One sub-test per joint type. Each creates the joint, exercises the shared
// b3Joint_* API plus every type-specific accessor, then steps to make sure the
// joint solves without tripping a validation assert.

@(private = "file")
JointFixture :: struct {
	worldId:  b3.WorldId,
	groundId: b3.BodyId,
	bodyId:   b3.BodyId,
}

// Static ground plus a dynamic box, anchored so a point-coincident joint starts
// satisfied. Gravity is off so the body stays put across the handful of steps
// each sub-test takes.
@(private = "file")
CreateJointFixture :: proc() -> JointFixture {
	worldDef := b3.DefaultWorldDef()
	worldDef.gravity = b3.Vec3_zero

	f: JointFixture
	f.worldId = b3.CreateWorld(&worldDef)

	groundDef := b3.DefaultBodyDef()
	f.groundId = b3.CreateBody(f.worldId, &groundDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = b3.Pos{0.0, 4.0, 0.0}
	f.bodyId = b3.CreateBody(f.worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()
	shapeDef.density = 1.0
	box := b3.MakeCubeHull(0.5)
	b3.CreateHullShape(f.bodyId, &shapeDef, &box.base)

	return f
}

// Place the joint anchor at the dynamic body so both local frames map to the
// same world point.
@(private = "file")
SetCommonFrames :: proc(base: ^b3.JointDef, f: ^JointFixture) {
	base.bodyIdA = f.groundId
	base.bodyIdB = f.bodyId
	base.localFrameA.p = b3.Vec3{0.0, 4.0, 0.0}
	base.localFrameB.p = b3.Vec3{0.0, 0.0, 0.0}
}

// Step a few times, destroy the joint, then the world. Destroying the joint
// explicitly also covers b3DestroyJoint and stale-handle detection.
@(private = "file")
FinishJoint :: proc(t: ^testing.T, jointId: b3.JointId, worldId: b3.WorldId) -> bool {
	for i: i32 = 0; i < 8; i += 1 {
		b3.World_Step(worldId, 1.0 / 60.0, 4)
	}

	b3.DestroyJoint(jointId, true)
	ENSURE(t, b3.Joint_IsValid(jointId) == false) or_return

	b3.DestroyWorld(worldId)
	return true
}

// Exercise the API shared by every joint type. Frames are saved and restored so
// the caller's setup survives.
@(private = "file")
ExerciseJointBase :: proc(
	t: ^testing.T,
	jointId: b3.JointId,
	worldId: b3.WorldId,
	bodyIdA: b3.BodyId,
	bodyIdB: b3.BodyId,
	expectedType: b3.JointType,
) -> bool {
	ENSURE(t, b3.Joint_IsValid(jointId)) or_return
	ENSURE(t, b3.Joint_GetType(jointId) == expectedType) or_return
	ENSURE(t, b3.ID_EQUALS(b3.Joint_GetBodyA(jointId), bodyIdA)) or_return
	ENSURE(t, b3.ID_EQUALS(b3.Joint_GetBodyB(jointId), bodyIdB)) or_return

	// B3_ID_EQUALS does not work for world ids
	gotWorld := b3.Joint_GetWorld(jointId)
	ENSURE(t, gotWorld.index1 == worldId.index1 && gotWorld.generation == worldId.generation) or_return

	originalA := b3.Joint_GetLocalFrameA(jointId)
	originalB := b3.Joint_GetLocalFrameB(jointId)

	frameA := b3.Transform{{0.1, 0.2, 0.3}, b3.Quat_identity}
	b3.Joint_SetLocalFrameA(jointId, frameA)
	gotA := b3.Joint_GetLocalFrameA(jointId)
	ENSURE(t, gotA.p.x == frameA.p.x && gotA.p.y == frameA.p.y && gotA.p.z == frameA.p.z) or_return

	frameB := b3.Transform{{-0.4, 0.5, -0.6}, b3.Quat_identity}
	b3.Joint_SetLocalFrameB(jointId, frameB)
	gotB := b3.Joint_GetLocalFrameB(jointId)
	ENSURE(t, gotB.p.x == frameB.p.x && gotB.p.y == frameB.p.y && gotB.p.z == frameB.p.z) or_return

	b3.Joint_SetCollideConnected(jointId, true)
	ENSURE(t, b3.Joint_GetCollideConnected(jointId) == true) or_return
	b3.Joint_SetCollideConnected(jointId, false)
	ENSURE(t, b3.Joint_GetCollideConnected(jointId) == false) or_return

	userData: i32 = 0
	b3.Joint_SetUserData(jointId, &userData)
	ENSURE(t, b3.Joint_GetUserData(jointId) == &userData) or_return

	b3.Joint_SetConstraintTuning(jointId, 90.0, 3.0)
	hertz: f32 = 0.0
	dampingRatio: f32 = 0.0
	b3.Joint_GetConstraintTuning(jointId, &hertz, &dampingRatio)
	ENSURE(t, hertz == 90.0) or_return
	ENSURE(t, dampingRatio == 3.0) or_return

	b3.Joint_SetForceThreshold(jointId, 100.0)
	ENSURE(t, b3.Joint_GetForceThreshold(jointId) == 100.0) or_return

	b3.Joint_SetTorqueThreshold(jointId, 200.0)
	ENSURE(t, b3.Joint_GetTorqueThreshold(jointId) == 200.0) or_return

	b3.Joint_WakeBodies(jointId)

	// No stable value to assert before the first step, call for coverage
	force := b3.Joint_GetConstraintForce(jointId)
	torque := b3.Joint_GetConstraintTorque(jointId)
	linearSeparation := b3.Joint_GetLinearSeparation(jointId)
	_ = force
	_ = torque
	_ = linearSeparation

	// Wheel joint angular separation is an unimplemented todo in joint.c that
	// asserts. Every other type computes it.
	if expectedType != .wheelJoint {
		angularSeparation := b3.Joint_GetAngularSeparation(jointId)
		_ = angularSeparation
	}

	b3.Joint_SetLocalFrameA(jointId, originalA)
	b3.Joint_SetLocalFrameB(jointId, originalB)

	return true
}

@(private = "file")
TestParallelJoint :: proc(t: ^testing.T) -> bool {
	f := CreateJointFixture()

	def := b3.DefaultParallelJointDef()
	SetCommonFrames(&def.base, &f)
	def.hertz = 2.0
	def.dampingRatio = 0.5
	def.maxTorque = 100.0
	jointId := b3.CreateParallelJoint(f.worldId, &def)

	if ExerciseJointBase(t, jointId, f.worldId, f.groundId, f.bodyId, .parallelJoint) == false {
		return false
	}

	b3.ParallelJoint_SetSpringHertz(jointId, 5.0)
	ENSURE(t, b3.ParallelJoint_GetSpringHertz(jointId) == 5.0) or_return

	b3.ParallelJoint_SetSpringDampingRatio(jointId, 0.7)
	ENSURE(t, b3.ParallelJoint_GetSpringDampingRatio(jointId) == 0.7) or_return

	b3.ParallelJoint_SetMaxTorque(jointId, 250.0)
	ENSURE(t, b3.ParallelJoint_GetMaxTorque(jointId) == 250.0) or_return

	return FinishJoint(t, jointId, f.worldId)
}

@(private = "file")
TestDistanceJoint :: proc(t: ^testing.T) -> bool {
	f := CreateJointFixture()

	def := b3.DefaultDistanceJointDef()
	SetCommonFrames(&def.base, &f)
	def.length = 2.0
	jointId := b3.CreateDistanceJoint(f.worldId, &def)

	if ExerciseJointBase(t, jointId, f.worldId, f.groundId, f.bodyId, .distanceJoint) == false {
		return false
	}

	b3.DistanceJoint_SetLength(jointId, 3.0)
	ENSURE(t, b3.DistanceJoint_GetLength(jointId) == 3.0) or_return

	b3.DistanceJoint_EnableSpring(jointId, true)
	ENSURE(t, b3.DistanceJoint_IsSpringEnabled(jointId) == true) or_return

	b3.DistanceJoint_SetSpringForceRange(jointId, -50.0, 75.0)
	lowerForce: f32 = 0.0
	upperForce: f32 = 0.0
	b3.DistanceJoint_GetSpringForceRange(jointId, &lowerForce, &upperForce)
	ENSURE(t, lowerForce == -50.0 && upperForce == 75.0) or_return

	b3.DistanceJoint_SetSpringHertz(jointId, 4.0)
	ENSURE(t, b3.DistanceJoint_GetSpringHertz(jointId) == 4.0) or_return

	b3.DistanceJoint_SetSpringDampingRatio(jointId, 0.6)
	ENSURE(t, b3.DistanceJoint_GetSpringDampingRatio(jointId) == 0.6) or_return

	b3.DistanceJoint_EnableLimit(jointId, true)
	ENSURE(t, b3.DistanceJoint_IsLimitEnabled(jointId) == true) or_return

	b3.DistanceJoint_SetLengthRange(jointId, 1.0, 5.0)
	ENSURE(t, b3.DistanceJoint_GetMinLength(jointId) == 1.0) or_return
	ENSURE(t, b3.DistanceJoint_GetMaxLength(jointId) == 5.0) or_return

	currentLength := b3.DistanceJoint_GetCurrentLength(jointId)
	_ = currentLength

	b3.DistanceJoint_EnableMotor(jointId, true)
	ENSURE(t, b3.DistanceJoint_IsMotorEnabled(jointId) == true) or_return

	b3.DistanceJoint_SetMotorSpeed(jointId, 1.5)
	ENSURE(t, b3.DistanceJoint_GetMotorSpeed(jointId) == 1.5) or_return

	b3.DistanceJoint_SetMaxMotorForce(jointId, 25.0)
	ENSURE(t, b3.DistanceJoint_GetMaxMotorForce(jointId) == 25.0) or_return

	motorForce := b3.DistanceJoint_GetMotorForce(jointId)
	_ = motorForce

	return FinishJoint(t, jointId, f.worldId)
}

@(private = "file")
TestFilterJoint :: proc(t: ^testing.T) -> bool {
	f := CreateJointFixture()

	// The filter joint has no type-specific API. It only disables collision and
	// keeps both bodies in the same island.
	def := b3.DefaultFilterJointDef()
	def.base.bodyIdA = f.groundId
	def.base.bodyIdB = f.bodyId
	jointId := b3.CreateFilterJoint(f.worldId, &def)

	if ExerciseJointBase(t, jointId, f.worldId, f.groundId, f.bodyId, .filterJoint) == false {
		return false
	}

	return FinishJoint(t, jointId, f.worldId)
}

@(private = "file")
TestMotorJoint :: proc(t: ^testing.T) -> bool {
	f := CreateJointFixture()

	def := b3.DefaultMotorJointDef()
	SetCommonFrames(&def.base, &f)
	jointId := b3.CreateMotorJoint(f.worldId, &def)

	if ExerciseJointBase(t, jointId, f.worldId, f.groundId, f.bodyId, .motorJoint) == false {
		return false
	}

	linearVelocity := b3.Vec3{1.0, 2.0, 3.0}
	b3.MotorJoint_SetLinearVelocity(jointId, linearVelocity)
	gotLinear := b3.MotorJoint_GetLinearVelocity(jointId)
	ENSURE(t, gotLinear.x == 1.0 && gotLinear.y == 2.0 && gotLinear.z == 3.0) or_return

	angularVelocity := b3.Vec3{0.1, 0.2, 0.3}
	b3.MotorJoint_SetAngularVelocity(jointId, angularVelocity)
	gotAngular := b3.MotorJoint_GetAngularVelocity(jointId)
	ENSURE(t, gotAngular.x == 0.1 && gotAngular.y == 0.2 && gotAngular.z == 0.3) or_return

	b3.MotorJoint_SetMaxVelocityForce(jointId, 500.0)
	ENSURE(t, b3.MotorJoint_GetMaxVelocityForce(jointId) == 500.0) or_return

	b3.MotorJoint_SetMaxVelocityTorque(jointId, 600.0)
	ENSURE(t, b3.MotorJoint_GetMaxVelocityTorque(jointId) == 600.0) or_return

	b3.MotorJoint_SetLinearHertz(jointId, 3.0)
	ENSURE(t, b3.MotorJoint_GetLinearHertz(jointId) == 3.0) or_return

	b3.MotorJoint_SetLinearDampingRatio(jointId, 0.8)
	ENSURE(t, b3.MotorJoint_GetLinearDampingRatio(jointId) == 0.8) or_return

	b3.MotorJoint_SetAngularHertz(jointId, 4.0)
	ENSURE(t, b3.MotorJoint_GetAngularHertz(jointId) == 4.0) or_return

	b3.MotorJoint_SetAngularDampingRatio(jointId, 0.9)
	ENSURE(t, b3.MotorJoint_GetAngularDampingRatio(jointId) == 0.9) or_return

	b3.MotorJoint_SetMaxSpringForce(jointId, 700.0)
	ENSURE(t, b3.MotorJoint_GetMaxSpringForce(jointId) == 700.0) or_return

	b3.MotorJoint_SetMaxSpringTorque(jointId, 800.0)
	ENSURE(t, b3.MotorJoint_GetMaxSpringTorque(jointId) == 800.0) or_return

	return FinishJoint(t, jointId, f.worldId)
}

@(private = "file")
TestMoverJoint :: proc(t: ^testing.T) -> bool {
	f := CreateJointFixture()

	def := b3.DefaultMoverJointDef()
	SetCommonFrames(&def.base, &f)
	jointId := b3.CreateMoverJoint(f.worldId, &def)

	if ExerciseJointBase(t, jointId, f.worldId, f.groundId, f.bodyId, .moverJoint) == false {
		return false
	}

	velocity := b3.Vec3{1.0, 2.0, -3.0}
	b3.MoverJoint_SetLinearVelocity(jointId, velocity)
	gotVelocity := b3.MoverJoint_GetLinearVelocity(jointId)
	ENSURE(t, gotVelocity.x == 1.0 && gotVelocity.y == 2.0 && gotVelocity.z == -3.0) or_return

	// The y axis has no force budget, so it must stay at rest while x and z reach the target
	maxForce: f32 = 1000.0
	b3.MoverJoint_SetMaxVelocityForce(jointId, maxForce)
	gotForce := b3.MoverJoint_GetMaxVelocityForce(jointId)
	ENSURE(t, gotForce == 1000.0) or_return

	for i: i32 = 0; i < 30; i += 1 {
		b3.World_Step(f.worldId, 1.0 / 60.0, 4)
	}

	v := b3.Body_GetLinearVelocity(f.bodyId)
	ENSURE_SMALL(t, v.x - 1.0, 1.0e-4) or_return
	ENSURE_SMALL(t, v.y, 1.0e-4) or_return
	ENSURE_SMALL(t, v.z + 3.0, 1.0e-4) or_return

	w := b3.Body_GetAngularVelocity(f.bodyId)
	ENSURE(t, w.x == 0.0 && w.y == 0.0 && w.z == 0.0) or_return

	return FinishJoint(t, jointId, f.worldId)
}

// A box held up by a pogo spring under gravity should settle at the rest length
@(private = "file")
TestPogoJoint :: proc(t: ^testing.T) -> bool {
	f := CreateJointFixture()
	b3.World_SetGravity(f.worldId, b3.Vec3{0.0, -10.0, 0.0})

	restLength: f32 = 2.0
	mass := b3.Body_GetMass(f.bodyId)

	def := b3.DefaultPogoJointDef()
	def.base.bodyIdA = f.groundId
	def.base.bodyIdB = f.bodyId
	def.base.localFrameA.p = b3.Vec3{0.0, 1.0, 0.0}
	def.base.localFrameB.p = b3.Vec3{0.0, 0.0, 0.0}
	def.normal = b3.Vec3_axisY
	def.restLength = restLength
	def.hertz = 2.0
	def.dampingRatio = 1.0
	def.maxCompressionForce = f32(100.0) * f32(10.0) * mass
	def.maxTensionForce = f32(100.0) * f32(10.0) * mass
	jointId := b3.CreatePogoJoint(f.worldId, &def)

	if ExerciseJointBase(t, jointId, f.worldId, f.groundId, f.bodyId, .pogoJoint) == false {
		return false
	}

	b3.PogoJoint_SetRestLength(jointId, restLength)
	ENSURE(t, b3.PogoJoint_GetRestLength(jointId) == restLength) or_return

	b3.PogoJoint_SetSpringHertz(jointId, 3.0)
	ENSURE(t, b3.PogoJoint_GetSpringHertz(jointId) == 3.0) or_return

	b3.PogoJoint_SetSpringDampingRatio(jointId, 0.9)
	ENSURE(t, b3.PogoJoint_GetSpringDampingRatio(jointId) == 0.9) or_return

	maxTension := f32(50.0) * f32(10.0) * mass
	b3.PogoJoint_SetMaxTensionForce(jointId, maxTension)
	ENSURE(t, b3.PogoJoint_GetMaxTensionForce(jointId) == maxTension) or_return

	maxCompression := f32(60.0) * f32(10.0) * mass
	b3.PogoJoint_SetMaxCompressionForce(jointId, maxCompression)
	ENSURE(t, b3.PogoJoint_GetMaxCompressionForce(jointId) == maxCompression) or_return

	normal := b3.Normalize(b3.Vec3{0.2, 1.0, 0.0})
	b3.PogoJoint_SetNormal(jointId, normal)
	gotNormal := b3.PogoJoint_GetNormal(jointId)
	ENSURE(t, gotNormal.x == normal.x && gotNormal.y == normal.y && gotNormal.z == normal.z) or_return
	b3.PogoJoint_SetNormal(jointId, b3.Vec3_axisY)

	// Starts stretched by one meter
	ENSURE_SMALL(t, b3.PogoJoint_GetCurrentLength(jointId) - 3.0, 1.0e-5) or_return

	for i: i32 = 0; i < 300; i += 1 {
		b3.World_Step(f.worldId, 1.0 / 60.0, 4)
	}

	ENSURE_SMALL(t, b3.PogoJoint_GetCurrentLength(jointId) - restLength, 0.01) or_return
	ENSURE_SMALL(t, b3.Body_GetLinearVelocity(f.bodyId).y, 0.01) or_return

	// Holding the weight takes an impulse of m * g * h each sub-step
	ENSURE_SMALL(t, b3.PogoJoint_GetImpulse(jointId) - mass * 10.0 / 240.0, 0.001 * mass) or_return

	force := b3.Joint_GetConstraintForce(jointId)
	ENSURE_SMALL(t, force.y - mass * 10.0, 0.1 * mass) or_return

	return FinishJoint(t, jointId, f.worldId)
}

@(private = "file")
TestPrismaticJoint :: proc(t: ^testing.T) -> bool {
	f := CreateJointFixture()

	def := b3.DefaultPrismaticJointDef()
	SetCommonFrames(&def.base, &f)
	jointId := b3.CreatePrismaticJoint(f.worldId, &def)

	if ExerciseJointBase(t, jointId, f.worldId, f.groundId, f.bodyId, .prismaticJoint) == false {
		return false
	}

	b3.PrismaticJoint_EnableSpring(jointId, true)
	ENSURE(t, b3.PrismaticJoint_IsSpringEnabled(jointId) == true) or_return

	b3.PrismaticJoint_SetSpringHertz(jointId, 5.0)
	ENSURE(t, b3.PrismaticJoint_GetSpringHertz(jointId) == 5.0) or_return

	b3.PrismaticJoint_SetSpringDampingRatio(jointId, 0.5)
	ENSURE(t, b3.PrismaticJoint_GetSpringDampingRatio(jointId) == 0.5) or_return

	b3.PrismaticJoint_SetTargetTranslation(jointId, 1.0)
	ENSURE(t, b3.PrismaticJoint_GetTargetTranslation(jointId) == 1.0) or_return

	b3.PrismaticJoint_EnableLimit(jointId, true)
	ENSURE(t, b3.PrismaticJoint_IsLimitEnabled(jointId) == true) or_return

	b3.PrismaticJoint_SetLimits(jointId, -2.0, 2.0)
	ENSURE(t, b3.PrismaticJoint_GetLowerLimit(jointId) == -2.0) or_return
	ENSURE(t, b3.PrismaticJoint_GetUpperLimit(jointId) == 2.0) or_return

	b3.PrismaticJoint_EnableMotor(jointId, true)
	ENSURE(t, b3.PrismaticJoint_IsMotorEnabled(jointId) == true) or_return

	b3.PrismaticJoint_SetMotorSpeed(jointId, 1.5)
	ENSURE(t, b3.PrismaticJoint_GetMotorSpeed(jointId) == 1.5) or_return

	b3.PrismaticJoint_SetMaxMotorForce(jointId, 30.0)
	ENSURE(t, b3.PrismaticJoint_GetMaxMotorForce(jointId) == 30.0) or_return

	motorForce := b3.PrismaticJoint_GetMotorForce(jointId)
	translation := b3.PrismaticJoint_GetTranslation(jointId)
	speed := b3.PrismaticJoint_GetSpeed(jointId)
	_ = motorForce
	_ = translation
	_ = speed

	return FinishJoint(t, jointId, f.worldId)
}

@(private = "file")
TestRevoluteJoint :: proc(t: ^testing.T) -> bool {
	f := CreateJointFixture()

	def := b3.DefaultRevoluteJointDef()
	SetCommonFrames(&def.base, &f)
	jointId := b3.CreateRevoluteJoint(f.worldId, &def)

	if ExerciseJointBase(t, jointId, f.worldId, f.groundId, f.bodyId, .revoluteJoint) == false {
		return false
	}

	b3.RevoluteJoint_EnableSpring(jointId, true)
	ENSURE(t, b3.RevoluteJoint_IsSpringEnabled(jointId) == true) or_return

	b3.RevoluteJoint_SetSpringHertz(jointId, 5.0)
	ENSURE(t, b3.RevoluteJoint_GetSpringHertz(jointId) == 5.0) or_return

	b3.RevoluteJoint_SetSpringDampingRatio(jointId, 0.5)
	ENSURE(t, b3.RevoluteJoint_GetSpringDampingRatio(jointId) == 0.5) or_return

	b3.RevoluteJoint_SetTargetAngle(jointId, 0.5)
	ENSURE(t, b3.RevoluteJoint_GetTargetAngle(jointId) == 0.5) or_return

	angle := b3.RevoluteJoint_GetAngle(jointId)
	_ = angle

	b3.RevoluteJoint_EnableLimit(jointId, true)
	ENSURE(t, b3.RevoluteJoint_IsLimitEnabled(jointId) == true) or_return

	b3.RevoluteJoint_SetLimits(jointId, -1.0, 1.0)
	ENSURE(t, b3.RevoluteJoint_GetLowerLimit(jointId) == -1.0) or_return
	ENSURE(t, b3.RevoluteJoint_GetUpperLimit(jointId) == 1.0) or_return

	b3.RevoluteJoint_EnableMotor(jointId, true)
	ENSURE(t, b3.RevoluteJoint_IsMotorEnabled(jointId) == true) or_return

	b3.RevoluteJoint_SetMotorSpeed(jointId, 2.0)
	ENSURE(t, b3.RevoluteJoint_GetMotorSpeed(jointId) == 2.0) or_return

	b3.RevoluteJoint_SetMaxMotorTorque(jointId, 40.0)
	ENSURE(t, b3.RevoluteJoint_GetMaxMotorTorque(jointId) == 40.0) or_return

	motorTorque := b3.RevoluteJoint_GetMotorTorque(jointId)
	_ = motorTorque

	return FinishJoint(t, jointId, f.worldId)
}

@(private = "file")
TestSphericalJoint :: proc(t: ^testing.T) -> bool {
	f := CreateJointFixture()

	def := b3.DefaultSphericalJointDef()
	SetCommonFrames(&def.base, &f)
	jointId := b3.CreateSphericalJoint(f.worldId, &def)

	if ExerciseJointBase(t, jointId, f.worldId, f.groundId, f.bodyId, .sphericalJoint) == false {
		return false
	}

	b3.SphericalJoint_EnableConeLimit(jointId, true)
	ENSURE(t, b3.SphericalJoint_IsConeLimitEnabled(jointId) == true) or_return

	b3.SphericalJoint_SetConeLimit(jointId, 0.5)
	ENSURE(t, b3.SphericalJoint_GetConeLimit(jointId) == 0.5) or_return

	coneAngle := b3.SphericalJoint_GetConeAngle(jointId)
	_ = coneAngle

	b3.SphericalJoint_EnableTwistLimit(jointId, true)
	ENSURE(t, b3.SphericalJoint_IsTwistLimitEnabled(jointId) == true) or_return

	b3.SphericalJoint_SetTwistLimits(jointId, -0.5, 0.5)
	ENSURE(t, b3.SphericalJoint_GetLowerTwistLimit(jointId) == -0.5) or_return
	ENSURE(t, b3.SphericalJoint_GetUpperTwistLimit(jointId) == 0.5) or_return

	twistAngle := b3.SphericalJoint_GetTwistAngle(jointId)
	_ = twistAngle

	b3.SphericalJoint_EnableSpring(jointId, true)
	ENSURE(t, b3.SphericalJoint_IsSpringEnabled(jointId) == true) or_return

	b3.SphericalJoint_SetSpringHertz(jointId, 5.0)
	ENSURE(t, b3.SphericalJoint_GetSpringHertz(jointId) == 5.0) or_return

	b3.SphericalJoint_SetSpringDampingRatio(jointId, 0.5)
	ENSURE(t, b3.SphericalJoint_GetSpringDampingRatio(jointId) == 0.5) or_return

	// 90 degrees about z, a unit quaternion that round-trips through storage
	targetRotation := b3.Quat{{0.0, 0.0, 0.7071068}, 0.7071068}
	b3.SphericalJoint_SetTargetRotation(jointId, targetRotation)
	gotRotation := b3.SphericalJoint_GetTargetRotation(jointId)
	ENSURE_SMALL(t, gotRotation.v.x - targetRotation.v.x, 1.0e-5) or_return
	ENSURE_SMALL(t, gotRotation.v.y - targetRotation.v.y, 1.0e-5) or_return
	ENSURE_SMALL(t, gotRotation.v.z - targetRotation.v.z, 1.0e-5) or_return
	ENSURE_SMALL(t, gotRotation.s - targetRotation.s, 1.0e-5) or_return

	b3.SphericalJoint_EnableMotor(jointId, true)
	ENSURE(t, b3.SphericalJoint_IsMotorEnabled(jointId) == true) or_return

	motorVelocity := b3.Vec3{0.1, 0.2, 0.3}
	b3.SphericalJoint_SetMotorVelocity(jointId, motorVelocity)
	gotMotorVelocity := b3.SphericalJoint_GetMotorVelocity(jointId)
	ENSURE(t, gotMotorVelocity.x == 0.1 && gotMotorVelocity.y == 0.2 && gotMotorVelocity.z == 0.3) or_return

	b3.SphericalJoint_SetMaxMotorTorque(jointId, 50.0)
	ENSURE(t, b3.SphericalJoint_GetMaxMotorTorque(jointId) == 50.0) or_return

	motorTorque := b3.SphericalJoint_GetMotorTorque(jointId)
	_ = motorTorque

	return FinishJoint(t, jointId, f.worldId)
}

@(private = "file")
TestWeldJoint :: proc(t: ^testing.T) -> bool {
	f := CreateJointFixture()

	def := b3.DefaultWeldJointDef()
	SetCommonFrames(&def.base, &f)
	jointId := b3.CreateWeldJoint(f.worldId, &def)

	if ExerciseJointBase(t, jointId, f.worldId, f.groundId, f.bodyId, .weldJoint) == false {
		return false
	}

	b3.WeldJoint_SetLinearHertz(jointId, 3.0)
	ENSURE(t, b3.WeldJoint_GetLinearHertz(jointId) == 3.0) or_return

	b3.WeldJoint_SetLinearDampingRatio(jointId, 0.5)
	ENSURE(t, b3.WeldJoint_GetLinearDampingRatio(jointId) == 0.5) or_return

	b3.WeldJoint_SetAngularHertz(jointId, 4.0)
	ENSURE(t, b3.WeldJoint_GetAngularHertz(jointId) == 4.0) or_return

	b3.WeldJoint_SetAngularDampingRatio(jointId, 0.7)
	ENSURE(t, b3.WeldJoint_GetAngularDampingRatio(jointId) == 0.7) or_return

	return FinishJoint(t, jointId, f.worldId)
}

@(private = "file")
TestWheelJoint :: proc(t: ^testing.T) -> bool {
	f := CreateJointFixture()

	def := b3.DefaultWheelJointDef()
	SetCommonFrames(&def.base, &f)
	jointId := b3.CreateWheelJoint(f.worldId, &def)

	if ExerciseJointBase(t, jointId, f.worldId, f.groundId, f.bodyId, .wheelJoint) == false {
		return false
	}

	b3.WheelJoint_EnableSuspension(jointId, true)
	ENSURE(t, b3.WheelJoint_IsSuspensionEnabled(jointId) == true) or_return

	b3.WheelJoint_SetSuspensionHertz(jointId, 5.0)
	ENSURE(t, b3.WheelJoint_GetSuspensionHertz(jointId) == 5.0) or_return

	b3.WheelJoint_SetSuspensionDampingRatio(jointId, 0.5)
	ENSURE(t, b3.WheelJoint_GetSuspensionDampingRatio(jointId) == 0.5) or_return

	b3.WheelJoint_EnableSuspensionLimit(jointId, true)
	ENSURE(t, b3.WheelJoint_IsSuspensionLimitEnabled(jointId) == true) or_return

	b3.WheelJoint_SetSuspensionLimits(jointId, -1.0, 1.0)
	ENSURE(t, b3.WheelJoint_GetLowerSuspensionLimit(jointId) == -1.0) or_return
	ENSURE(t, b3.WheelJoint_GetUpperSuspensionLimit(jointId) == 1.0) or_return

	b3.WheelJoint_EnableSpinMotor(jointId, true)
	ENSURE(t, b3.WheelJoint_IsSpinMotorEnabled(jointId) == true) or_return

	b3.WheelJoint_SetSpinMotorSpeed(jointId, 6.0)
	ENSURE(t, b3.WheelJoint_GetSpinMotorSpeed(jointId) == 6.0) or_return

	b3.WheelJoint_SetMaxSpinTorque(jointId, 35.0)
	ENSURE(t, b3.WheelJoint_GetMaxSpinTorque(jointId) == 35.0) or_return

	spinSpeed := b3.WheelJoint_GetSpinSpeed(jointId)
	spinTorque := b3.WheelJoint_GetSpinTorque(jointId)
	_ = spinSpeed
	_ = spinTorque

	b3.WheelJoint_EnableSteering(jointId, true)
	ENSURE(t, b3.WheelJoint_IsSteeringEnabled(jointId) == true) or_return

	b3.WheelJoint_SetSteeringHertz(jointId, 7.0)
	ENSURE(t, b3.WheelJoint_GetSteeringHertz(jointId) == 7.0) or_return

	b3.WheelJoint_SetSteeringDampingRatio(jointId, 0.8)
	ENSURE(t, b3.WheelJoint_GetSteeringDampingRatio(jointId) == 0.8) or_return

	b3.WheelJoint_SetMaxSteeringTorque(jointId, 45.0)
	ENSURE(t, b3.WheelJoint_GetMaxSteeringTorque(jointId) == 45.0) or_return

	b3.WheelJoint_EnableSteeringLimit(jointId, true)
	ENSURE(t, b3.WheelJoint_IsSteeringLimitEnabled(jointId) == true) or_return

	b3.WheelJoint_SetSteeringLimits(jointId, -0.6, 0.6)
	ENSURE(t, b3.WheelJoint_GetLowerSteeringLimit(jointId) == -0.6) or_return
	ENSURE(t, b3.WheelJoint_GetUpperSteeringLimit(jointId) == 0.6) or_return

	b3.WheelJoint_SetTargetSteeringAngle(jointId, 0.25)
	ENSURE(t, b3.WheelJoint_GetTargetSteeringAngle(jointId) == 0.25) or_return

	steeringAngle := b3.WheelJoint_GetSteeringAngle(jointId)
	steeringTorque := b3.WheelJoint_GetSteeringTorque(jointId)
	_ = steeringAngle
	_ = steeringTorque

	return FinishJoint(t, jointId, f.worldId)
}

// Issue #117: a joint created with collideConnected false must clear contacts
// that already exist between the two bodies.
@(private = "file")
TestCreateJointClearsContacts :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldDef.gravity = b3.Vec3_zero
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody

	bodyDef.position = b3.Pos{0.0, 0.0, 0.0}
	bodyIdA := b3.CreateBody(worldId, &bodyDef)

	// Overlapping so a touching contact forms
	bodyDef.position = b3.Pos{0.5, 0.0, 0.0}
	bodyIdB := b3.CreateBody(worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()
	shapeDef.density = 1.0
	box := b3.MakeBoxHull(0.5, 0.5, 0.5)
	b3.CreateHullShape(bodyIdA, &shapeDef, &box.base)
	b3.CreateHullShape(bodyIdB, &shapeDef, &box.base)

	b3.World_Step(worldId, 1.0 / 60.0, 4)
	ENSURE(t, b3.Body_GetContactCapacity(bodyIdA) > 0) or_return

	jointDef := b3.DefaultRevoluteJointDef()
	jointDef.base.bodyIdA = bodyIdA
	jointDef.base.bodyIdB = bodyIdB
	jointDef.base.collideConnected = false
	b3.CreateRevoluteJoint(worldId, &jointDef)

	ENSURE(t, b3.Body_GetContactCapacity(bodyIdA) == 0) or_return
	ENSURE(t, b3.Body_GetContactCapacity(bodyIdB) == 0) or_return

	b3.World_Step(worldId, 1.0 / 60.0, 4)
	ENSURE(t, b3.Body_GetContactCapacity(bodyIdA) == 0) or_return

	b3.DestroyWorld(worldId)
	return true
}

// Issue #164: a joint with no dynamic body lives in the static set even when one body is an awake
// kinematic. Disabling and enabling that body has to respect this, whether the joint was created
// that way or got there through body type changes.
@(private = "file")
TestNonDynamicJointDisable :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	worldDef.gravity = b3.Vec3_zero
	worldId := b3.CreateWorld(&worldDef)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyIdA := b3.CreateBody(worldId, &bodyDef)
	bodyDef.position = b3.Pos{2.0, 0.0, 0.0}
	bodyIdB := b3.CreateBody(worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()
	box := b3.MakeCubeHull(0.5)
	b3.CreateHullShape(bodyIdA, &shapeDef, &box.base)
	b3.CreateHullShape(bodyIdB, &shapeDef, &box.base)

	jointDef := b3.DefaultWeldJointDef()
	jointDef.base.bodyIdA = bodyIdA
	jointDef.base.bodyIdB = bodyIdB
	weldId := b3.CreateWeldJoint(worldId, &jointDef)

	b3.Body_SetType(bodyIdA, .staticBody)
	b3.Body_SetType(bodyIdB, .kinematicBody)
	b3.World_Step(worldId, 1.0 / 60.0, 4)

	b3.Body_Disable(bodyIdB)
	b3.World_Step(worldId, 1.0 / 60.0, 4)
	b3.Body_Enable(bodyIdB)
	b3.World_Step(worldId, 1.0 / 60.0, 4)
	ENSURE(t, b3.Joint_IsValid(weldId)) or_return

	b3.Body_SetType(bodyIdB, .dynamicBody)
	b3.World_Step(worldId, 1.0 / 60.0, 4)
	ENSURE(t, b3.Joint_IsAwake(weldId)) or_return

	// Created directly between a static and a kinematic body
	bodyDef.type = .kinematicBody
	bodyDef.position = b3.Pos{4.0, 0.0, 0.0}
	bodyIdC := b3.CreateBody(worldId, &bodyDef)
	b3.CreateHullShape(bodyIdC, &shapeDef, &box.base)

	revoluteDef := b3.DefaultRevoluteJointDef()
	revoluteDef.base.bodyIdA = bodyIdA
	revoluteDef.base.bodyIdB = bodyIdC
	revoluteId := b3.CreateRevoluteJoint(worldId, &revoluteDef)

	b3.Body_Disable(bodyIdC)
	b3.Body_Enable(bodyIdC)
	b3.World_Step(worldId, 1.0 / 60.0, 4)
	ENSURE(t, b3.Joint_IsAwake(revoluteId) == false) or_return

	b3.DestroyWorld(worldId)
	return true
}

@(test)
JointTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, TestParallelJoint) {return}
	if !RUN_SUBTEST(t, TestDistanceJoint) {return}
	if !RUN_SUBTEST(t, TestFilterJoint) {return}
	if !RUN_SUBTEST(t, TestMotorJoint) {return}
	if !RUN_SUBTEST(t, TestMoverJoint) {return}
	if !RUN_SUBTEST(t, TestPogoJoint) {return}
	if !RUN_SUBTEST(t, TestPrismaticJoint) {return}
	if !RUN_SUBTEST(t, TestRevoluteJoint) {return}
	if !RUN_SUBTEST(t, TestSphericalJoint) {return}
	if !RUN_SUBTEST(t, TestWeldJoint) {return}
	if !RUN_SUBTEST(t, TestWheelJoint) {return}
	if !RUN_SUBTEST(t, TestCreateJointClearsContacts) {return}
	if !RUN_SUBTEST(t, TestNonDynamicJointDisable) {return}
}
