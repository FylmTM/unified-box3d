// Ported from box3d src/joint.c and src/joint.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:intrinsics"
import "core:fmt"
import "core:mem"

// joint.h

// A joint edge is used to connect bodies and joints together
// in a joint graph where each body is a node and each joint
// is an edge. A joint edge belongs to a doubly linked list
// maintained in each attached body. Each joint has two joint
// nodes, one for each attached body.
JointEdge :: struct {
	bodyId:  i32,
	prevKey: i32,
	nextKey: i32,
}

// Map from b3JointId to b3Joint in the solver sets
Joint :: struct {
	userData:         rawptr,

	// index of simulation set stored in b3World
	// B3_NULL_INDEX when slot is free
	setIndex:         i32,

	// index into the constraint graph color array, may be B3_NULL_INDEX for sleeping/disabled joints
	// B3_NULL_INDEX when slot is free
	colorIndex:       i32,

	// joint index within set or graph color
	// B3_NULL_INDEX when slot is free
	localIndex:       i32,
	edges:            [2]JointEdge,
	jointId:          i32,
	islandId:         i32,

	// Index into the island's joints array for O(1) swap-removal.
	// B3_NULL_INDEX when not in an island.
	islandIndex:      i32,
	drawScale:        f32,
	type:             JointType,

	// This is monotonically advanced when a body is allocated in this slot
	// Used to check for invalid b3JointId
	generation:       u16,
	collideConnected: bool,
}

DistanceJoint :: struct {
	length:           f32,
	hertz:            f32,
	dampingRatio:     f32,
	lowerSpringForce: f32,
	upperSpringForce: f32,
	minLength:        f32,
	maxLength:        f32,
	maxMotorForce:    f32,
	motorSpeed:       f32,
	impulse:          f32,
	lowerImpulse:     f32,
	upperImpulse:     f32,
	motorImpulse:     f32,
	indexA:           i32,
	indexB:           i32,
	anchorA:          Vec3,
	anchorB:          Vec3,
	deltaCenter:      Vec3,
	distanceSoftness: Softness,
	axialMass:        f32,
	enableSpring:     bool,
	enableLimit:      bool,
	enableMotor:      bool,
}

MotorJoint :: struct {
	linearVelocity:         Vec3,
	angularVelocity:        Vec3,
	maxVelocityForce:       f32,
	maxVelocityTorque:      f32,
	linearHertz:            f32,
	linearDampingRatio:     f32,
	maxSpringForce:         f32,
	angularHertz:           f32,
	angularDampingRatio:    f32,
	maxSpringTorque:        f32,
	linearVelocityImpulse:  Vec3,
	angularVelocityImpulse: Vec3,
	linearSpringImpulse:    Vec3,
	angularSpringImpulse:   Vec3,
	linearSpring:           Softness,
	angularSpring:          Softness,
	indexA:                 i32,
	indexB:                 i32,
	frameA:                 Transform,
	frameB:                 Transform,
	deltaCenter:            Vec3,
	angularMass:            Matrix3,
}

MoverJoint :: struct {
	linearVelocity: Vec3,
	maxForce:       f32,
	axisX:          Vec3,
	axisZ:          Vec3,
	impulse:        Vec2,
	indexA:         i32,
	indexB:         i32,
	linearMass:     f32,
}

PogoJoint :: struct {
	normal:              Vec3,
	restLength:          f32,
	hertz:               f32,
	dampingRatio:        f32,
	maxTensionForce:     f32,
	maxCompressionForce: f32,
	impulse:             f32,
	indexA:              i32,
	indexB:              i32,
	frameA:              Transform,
	frameB:              Transform,
	deltaCenter:         Vec3,
	linearMass:          f32,
	velocity:            f32,
}

ParallelJoint :: struct {
	hertz:        f32,
	dampingRatio: f32,
	maxTorque:    f32,
	perpImpulse:  Vec2,
	perpAxisX:    Vec3,
	perpAxisY:    Vec3,
	quatA:        Quat,
	quatB:        Quat,
	indexA:       i32,
	indexB:       i32,
	softness:     Softness,
}

PrismaticJoint :: struct {
	perpImpulse:       Vec2,
	angularImpulse:    Vec3,
	springImpulse:     f32,
	motorImpulse:      f32,
	lowerImpulse:      f32,
	upperImpulse:      f32,
	hertz:             f32,
	dampingRatio:      f32,
	maxMotorForce:     f32,
	motorSpeed:        f32,
	targetTranslation: f32,
	lowerTranslation:  f32,
	upperTranslation:  f32,
	indexA:            i32,
	indexB:            i32,
	frameA:            Transform,
	frameB:            Transform,
	jointAxis:         Vec3,
	perpAxisY:         Vec3,
	perpAxisZ:         Vec3,
	deltaCenter:       Vec3,
	deltaAngle:        f32,
	rotationMass:      Matrix3,
	springSoftness:    Softness,
	enableSpring:      bool,
	enableLimit:       bool,
	enableMotor:       bool,
}

RevoluteJoint :: struct {
	linearImpulse:  Vec3,
	perpImpulse:    Vec2,
	springImpulse:  f32,
	motorImpulse:   f32,
	lowerImpulse:   f32,
	upperImpulse:   f32,
	hertz:          f32,
	dampingRatio:   f32,
	maxMotorTorque: f32,
	motorSpeed:     f32,
	targetAngle:    f32,
	lowerAngle:     f32,
	upperAngle:     f32,
	indexA:         i32,
	indexB:         i32,
	frameA:         Transform,
	frameB:         Transform,
	rotationAxisZ:  Vec3,
	perpAxisX:      Vec3,
	perpAxisY:      Vec3,
	deltaCenter:    Vec3,
	deltaAngle:     f32,
	axialMass:      f32,
	springSoftness: Softness,
	enableSpring:   bool,
	enableMotor:    bool,
	enableLimit:    bool,
}

SphericalJoint :: struct {
	linearImpulse:     Vec3,
	springImpulse:     Vec3,
	motorImpulse:      Vec3,
	lowerTwistImpulse: f32,
	upperTwistImpulse: f32,
	swingImpulse:      f32,
	hertz:             f32,
	dampingRatio:      f32,
	maxMotorTorque:    f32,
	motorVelocity:     Vec3,
	lowerTwistAngle:   f32,
	upperTwistAngle:   f32,
	coneAngle:         f32,
	targetRotation:    Quat,
	indexA:            i32,
	indexB:            i32,
	frameA:            Transform,
	frameB:            Transform,
	deltaCenter:       Vec3,
	swingAxis:         Vec3,
	twistJacobian:     Vec3,
	rotationMass:      Matrix3,
	swingMass:         f32,
	twistMass:         f32,
	springSoftness:    Softness,
	enableSpring:      bool,
	enableMotor:       bool,
	enableConeLimit:   bool,
	enableTwistLimit:  bool,
}

WeldJoint :: struct {
	linearHertz:         f32,
	linearDampingRatio:  f32,
	angularHertz:        f32,
	angularDampingRatio: f32,
	linearSpring:        Softness,
	angularSpring:       Softness,
	linearImpulse:       Vec3,
	angularImpulse:      Vec3,
	indexA:              i32,
	indexB:              i32,
	frameA:              Transform,
	frameB:              Transform,
	deltaCenter:         Vec3,
	angularMass:         Matrix3,
}

WheelJoint :: struct {
	linearImpulse:           Vec2,
	angularImpulse:          Vec2,
	spinImpulse:             f32,
	maxSpinTorque:           f32,
	spinSpeed:               f32,
	suspensionSpringImpulse: f32,
	lowerSuspensionImpulse:  f32,
	upperSuspensionImpulse:  f32,
	lowerSuspensionLimit:    f32,
	upperSuspensionLimit:    f32,
	suspensionHertz:         f32,
	suspensionDampingRatio:  f32,
	steeringSpringImpulse:   f32,
	lowerSteeringImpulse:    f32,
	upperSteeringImpulse:    f32,
	lowerSteeringLimit:      f32,
	upperSteeringLimit:      f32,
	targetSteeringAngle:     f32,
	maxSteeringTorque:       f32,
	steeringHertz:           f32,
	steeringDampingRatio:    f32,
	indexA:                  i32,
	indexB:                  i32,
	frameA:                  Transform,
	frameB:                  Transform,
	deltaCenter:             Vec3,
	spinMass:                f32,
	suspensionMass:          f32,
	steeringMass:            f32,
	suspensionSoftness:      Softness,
	steeringSoftness:        Softness,
	enableSpinMotor:         bool,
	enableSuspensionSpring:  bool,
	enableSuspensionLimit:   bool,
	enableSteering:          bool,
	enableSteeringLimit:     bool,
	enableSteeringMotor:     bool,
}

// The base joint class. Joints are used to constraint two bodies together in
// various fashions. Some joints also feature limits and motors.
JointSim :: struct {
	jointId:                i32,
	bodyIdA:                i32,
	bodyIdB:                i32,
	type:                   JointType,

	// Joint frames local to body origin
	localFrameA:            Transform,
	localFrameB:            Transform,
	invMassA, invMassB:     f32,
	invIA, invIB:           Matrix3,
	constraintHertz:        f32,
	constraintDampingRatio: f32,
	constraintSoftness:     Softness,
	forceThreshold:         f32,
	torqueThreshold:        f32,
	fixedRotation:          bool,
	using _:                struct #raw_union {
		distanceJoint:  DistanceJoint,
		motorJoint:     MotorJoint,
		moverJoint:     MoverJoint,
		parallelJoint:  ParallelJoint,
		pogoJoint:      PogoJoint,
		revoluteJoint:  RevoluteJoint,
		sphericalJoint: SphericalJoint,
		prismaticJoint: PrismaticJoint,
		weldJoint:      WeldJoint,
		wheelJoint:     WheelJoint,
	},
}

// K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
SolvePointConstraint :: #force_inline proc(
	mA: f32,
	mB: f32,
	iA: Matrix3,
	iB: Matrix3,
	rA: Vec3,
	rB: Vec3,
	rhs: Vec3,
) -> Vec3 {
	axy := rA.z * iA.cy.y - rA.y * iA.cz.y
	axz := rA.z * iA.cy.z - rA.y * iA.cz.z
	ay := Sub(MulSV(rA.x, iA.cz), MulSV(rA.z, iA.cx))
	az := Sub(MulSV(rA.y, iA.cx), MulSV(rA.x, iA.cy))

	bxy := rB.z * iB.cy.y - rB.y * iB.cz.y
	bxz := rB.z * iB.cy.z - rB.y * iB.cz.z
	by := Sub(MulSV(rB.x, iB.cz), MulSV(rB.z, iB.cx))
	bz := Sub(MulSV(rB.y, iB.cx), MulSV(rB.x, iB.cy))

	m := mA + mB
	kxx := (axy * rA.z - axz * rA.y) + (bxy * rB.z - bxz * rB.y) + m
	kyy := (ay.z * rA.x - ay.x * rA.z) + (by.z * rB.x - by.x * rB.z) + m
	kzz := (az.x * rA.y - az.y * rA.x) + (bz.x * rB.y - bz.y * rB.x) + m
	kxy := (ay.y * rA.z - ay.z * rA.y) + (by.y * rB.z - by.z * rB.y)
	kxz := (az.y * rA.z - az.z * rA.y) + (bz.y * rB.z - bz.z * rB.y)
	kyz := (az.z * rA.x - az.x * rA.z) + (bz.z * rB.x - bz.x * rB.z)

	cxx := kyy * kzz - kyz * kyz
	cyy := kxx * kzz - kxz * kxz
	czz := kxx * kyy - kxy * kxy
	cxy := kxz * kyz - kxy * kzz
	cxz := kxy * kyz - kyy * kxz
	cyz := kxy * kxz - kxx * kyz

	det := kxx * cxx + kxy * cxy + kxz * cxz
	if AbsFloat(det) > f32(1000.0) * FLT_MIN {
		invDet := 1.0 / det
		b := Vec3 {
			invDet * (cxx * rhs.x + cxy * rhs.y + cxz * rhs.z),
			invDet * (cxy * rhs.x + cyy * rhs.y + cyz * rhs.z),
			invDet * (cxz * rhs.x + cyz * rhs.y + czz * rhs.z),
		}
		return b
	}

	return Vec3_zero
}

// joint.c

@(private = "file")
DefaultJointDef :: proc() -> JointDef {
	def := JointDef{}
	def.localFrameA.q = Quat_identity
	def.localFrameB.q = Quat_identity
	def.forceThreshold = FLT_MAX
	def.torqueThreshold = FLT_MAX
	def.constraintHertz = 60.0
	def.constraintDampingRatio = 2.0
	def.drawScale = GetLengthUnitsPerMeter()
	def.internalValue = SECRET_COOKIE
	return def
}

DefaultParallelJointDef :: proc() -> ParallelJointDef {
	def := ParallelJointDef{}
	def.base = DefaultJointDef()
	def.hertz = 1.0
	def.dampingRatio = 1.0
	def.maxTorque = FLT_MAX
	return def
}

DefaultDistanceJointDef :: proc() -> DistanceJointDef {
	def := DistanceJointDef{}
	def.base = DefaultJointDef()
	def.lowerSpringForce = -FLT_MAX
	def.upperSpringForce = FLT_MAX
	def.length = 1.0
	def.maxLength = HUGE
	return def
}

DefaultMotorJointDef :: proc() -> MotorJointDef {
	def := MotorJointDef{}
	def.base = DefaultJointDef()
	return def
}

DefaultMoverJointDef :: proc() -> MoverJointDef {
	def := MoverJointDef{}
	def.base = DefaultJointDef()
	return def
}

DefaultPogoJointDef :: proc() -> PogoJointDef {
	def := PogoJointDef{}
	def.base = DefaultJointDef()
	def.normal = Vec3_axisY
	return def
}

DefaultFilterJointDef :: proc() -> FilterJointDef {
	def := FilterJointDef{}
	def.base = DefaultJointDef()
	return def
}

DefaultPrismaticJointDef :: proc() -> PrismaticJointDef {
	def := PrismaticJointDef{}
	def.base = DefaultJointDef()
	return def
}

DefaultRevoluteJointDef :: proc() -> RevoluteJointDef {
	def := RevoluteJointDef{}
	def.base = DefaultJointDef()
	return def
}

DefaultSphericalJointDef :: proc() -> SphericalJointDef {
	def := SphericalJointDef{}
	def.base = DefaultJointDef()
	def.targetRotation = Quat_identity
	return def
}

DefaultWeldJointDef :: proc() -> WeldJointDef {
	def := WeldJointDef{}
	def.base = DefaultJointDef()
	return def
}

DefaultWheelJointDef :: proc() -> WheelJointDef {
	def := WheelJointDef{}
	def.base = DefaultJointDef()
	def.enableSuspensionSpring = true
	def.suspensionHertz = 1.0
	def.suspensionDampingRatio = 0.7
	def.steeringHertz = 1.0
	def.steeringDampingRatio = 0.7
	return def
}

DefaultExplosionDef :: proc() -> ExplosionDef {
	def := ExplosionDef{}
	def.maskBits = DEFAULT_MASK_BITS
	return def
}

GetJointFullId :: proc(world: ^World, jointId: JointId) -> ^Joint {
	id := jointId.index1 - 1
	joint := Array_Get(&world.joints, id)
	assert(joint.jointId == id && joint.generation == jointId.generation)
	return joint
}

GetJointSim :: proc(world: ^World, joint: ^Joint) -> ^JointSim {
	if joint.setIndex == i32(SetType.awakeSet) {
		assert(0 <= joint.colorIndex && joint.colorIndex < GRAPH_COLOR_COUNT)
		color := &world.constraintGraph.colors[joint.colorIndex]
		return Array_Get(&color.jointSims, joint.localIndex)
	}

	set := Array_Get(&world.solverSets, joint.setIndex)
	return Array_Get(&set.jointSims, joint.localIndex)
}

GetJointSimCheckType :: proc(jointId: JointId, type: JointType) -> ^JointSim {
	_ = type
	world := GetWorld(i32(jointId.world0))
	joint := GetJointFullId(world, jointId)
	assert(joint.type == type)
	jointSim := GetJointSim(world, joint)
	assert(jointSim.type == type)
	return jointSim
}

@(private = "file")
DestroyContactsBetweenBodies :: proc(world: ^World, bodyA: ^Body, bodyB: ^Body) {
	contactKey: i32
	otherBodyId: i32

	// use the smaller of the two contact lists
	if bodyA.contactCount < bodyB.contactCount {
		contactKey = bodyA.headContactKey
		otherBodyId = bodyB.id
	} else {
		contactKey = bodyB.headContactKey
		otherBodyId = bodyA.id
	}

	// no need to wake bodies when a joint removes collision between them
	wakeBodies := false

	// destroy the contacts
	for contactKey != NULL_INDEX {
		contactId := contactKey >> 1
		edgeIndex := contactKey & 1

		contact := Array_Get(&world.contacts, contactId)
		contactKey = contact.edges[edgeIndex].nextKey

		otherEdgeIndex := edgeIndex ~ 1
		if contact.edges[otherEdgeIndex].bodyId == otherBodyId {
			// Careful, this removes the contact from the current doubly linked list
			DestroyContact(world, contact, wakeBodies)
		}
	}

	ValidateSolverSets(world)
}

@(private = "file")
JointPair :: struct {
	joint:    ^Joint,
	jointSim: ^JointSim,
}

@(private = "file")
CreateJoint :: proc(world: ^World, def: ^JointDef, type: JointType) -> JointPair {
	assert(IsValidTransform(def.localFrameA))
	assert(IsValidTransform(def.localFrameB))
	assert(world.worldId == def.bodyIdA.world0)
	assert(world.worldId == def.bodyIdB.world0)
	assert(ID_EQUALS(def.bodyIdA, def.bodyIdB) == false)

	bodyA := GetBodyFullId(world, def.bodyIdA)
	bodyB := GetBodyFullId(world, def.bodyIdB)

	// If the joint prevents collisions, then destroy all contacts between attached bodies.
	// Do this first so world arrays are not disturbed in the following code.
	if def.collideConnected == false {
		DestroyContactsBetweenBodies(world, bodyA, bodyB)
	}

	bodyIdA := bodyA.id
	bodyIdB := bodyB.id
	maxSetIndex := MaxInt(bodyA.setIndex, bodyB.setIndex)

	// Create joint id and joint
	jointId := AllocId(&world.jointIdPool)
	if jointId == world.joints.count {
		Array_Push(&world.joints, Joint{})
	}

	joint := Array_Get(&world.joints, jointId)
	joint.jointId = jointId
	joint.userData = def.userData
	joint.generation += 1
	joint.setIndex = NULL_INDEX
	joint.colorIndex = NULL_INDEX
	joint.localIndex = NULL_INDEX
	joint.islandId = NULL_INDEX
	joint.islandIndex = NULL_INDEX
	joint.drawScale = def.drawScale
	joint.type = type
	joint.collideConnected = def.collideConnected

	// Doubly linked list on bodyA
	joint.edges[0].bodyId = bodyIdA
	joint.edges[0].prevKey = NULL_INDEX
	joint.edges[0].nextKey = bodyA.headJointKey

	keyA := (jointId << 1) | 0
	if bodyA.headJointKey != NULL_INDEX {
		jointA := Array_Get(&world.joints, bodyA.headJointKey >> 1)
		edgeA := &jointA.edges[bodyA.headJointKey & 1]
		edgeA.prevKey = keyA
	}
	bodyA.headJointKey = keyA
	bodyA.jointCount += 1

	// Doubly linked list on bodyB
	joint.edges[1].bodyId = bodyIdB
	joint.edges[1].prevKey = NULL_INDEX
	joint.edges[1].nextKey = bodyB.headJointKey

	keyB := (jointId << 1) | 1
	if bodyB.headJointKey != NULL_INDEX {
		jointB := Array_Get(&world.joints, bodyB.headJointKey >> 1)
		edgeB := &jointB.edges[bodyB.headJointKey & 1]
		edgeB.prevKey = keyB
	}
	bodyB.headJointKey = keyB
	bodyB.jointCount += 1

	jointSim: ^JointSim

	if bodyA.setIndex == i32(SetType.disabledSet) || bodyB.setIndex == i32(SetType.disabledSet) {
		// if either body is disabled, create in disabled set
		set := Array_Get(&world.solverSets, i32(SetType.disabledSet))
		joint.setIndex = i32(SetType.disabledSet)
		joint.localIndex = set.jointSims.count

		jointSim = Array_Emplace(&set.jointSims)
		mem.zero(jointSim, size_of(JointSim))

		jointSim.jointId = jointId
		jointSim.bodyIdA = bodyIdA
		jointSim.bodyIdB = bodyIdB
	} else if bodyA.type != .dynamicBody && bodyB.type != .dynamicBody {
		// joint is not attached to a dynamic body
		set := Array_Get(&world.solverSets, i32(SetType.staticSet))
		joint.setIndex = i32(SetType.staticSet)
		joint.localIndex = set.jointSims.count

		jointSim = Array_Emplace(&set.jointSims)
		mem.zero(jointSim, size_of(JointSim))

		jointSim.jointId = jointId
		jointSim.bodyIdA = bodyIdA
		jointSim.bodyIdB = bodyIdB
	} else if bodyA.setIndex == i32(SetType.awakeSet) || bodyB.setIndex == i32(SetType.awakeSet) {
		// if either body is sleeping, wake it
		if maxSetIndex >= i32(SetType.firstSleepingSet) {
			WakeSolverSet(world, maxSetIndex)
		}

		joint.setIndex = i32(SetType.awakeSet)

		jointSim = CreateJointInGraph(world, joint)
		jointSim.jointId = jointId
		jointSim.bodyIdA = bodyIdA
		jointSim.bodyIdB = bodyIdB
	} else {
		// joint connected between sleeping and/or static bodies
		assert(bodyA.setIndex >= i32(SetType.firstSleepingSet) || bodyB.setIndex >= i32(SetType.firstSleepingSet))
		assert(bodyA.setIndex != i32(SetType.staticSet) || bodyB.setIndex != i32(SetType.staticSet))

		// joint should go into the sleeping set (not static set)
		setIndex := maxSetIndex

		set := Array_Get(&world.solverSets, setIndex)
		joint.setIndex = setIndex
		joint.localIndex = set.jointSims.count

		jointSim = Array_Emplace(&set.jointSims)
		mem.zero(jointSim, size_of(JointSim))

		jointSim.jointId = jointId
		jointSim.bodyIdA = bodyIdA
		jointSim.bodyIdB = bodyIdB

		if bodyA.setIndex != bodyB.setIndex &&
		   bodyA.setIndex >= i32(SetType.firstSleepingSet) &&
		   bodyB.setIndex >= i32(SetType.firstSleepingSet) {
			// merge sleeping sets
			MergeSolverSets(world, bodyA.setIndex, bodyB.setIndex)
			assert(bodyA.setIndex == bodyB.setIndex)

			// fix potentially invalid set index
			setIndex = bodyA.setIndex

			mergedSet := Array_Get(&world.solverSets, setIndex)

			// Careful! The joint sim pointer was orphaned by the set merge.
			jointSim = Array_Get(&mergedSet.jointSims, joint.localIndex)
		}

		assert(joint.setIndex == setIndex)
	}

	jointSim.localFrameA = def.localFrameA
	jointSim.localFrameB = def.localFrameB
	jointSim.type = type
	jointSim.constraintHertz = def.constraintHertz
	jointSim.constraintDampingRatio = def.constraintDampingRatio
	jointSim.constraintSoftness = Softness {
		biasRate     = 0.0,
		massScale    = 1.0,
		impulseScale = 0.0,
	}

	assert(IsValidFloat(def.forceThreshold) && def.forceThreshold >= 0.0)
	assert(IsValidFloat(def.torqueThreshold) && def.torqueThreshold >= 0.0)

	jointSim.forceThreshold = def.forceThreshold
	jointSim.torqueThreshold = def.torqueThreshold

	assert(jointSim.jointId == jointId)
	assert(jointSim.bodyIdA == bodyIdA)
	assert(jointSim.bodyIdB == bodyIdB)

	if joint.setIndex > i32(SetType.disabledSet) {
		// Add edge to island graph
		LinkJoint(world, joint)
	}

	ValidateSolverSets(world)

	return JointPair{joint, jointSim}
}

Joint_SetConstraintTuning :: proc(jointId: JointId, hertz: f32, dampingRatio: f32) {
	assert(IsValidFloat(hertz) && hertz >= 0.0)
	assert(IsValidFloat(dampingRatio) && dampingRatio >= 0.0)

	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_JointSetConstraintTuning{jointId, hertz, dampingRatio}
		RecWrite_JointSetConstraintTuning(world.recording, &recArgs)
	}
	joint := GetJointFullId(world, jointId)
	base := GetJointSim(world, joint)
	base.constraintHertz = hertz
	base.constraintDampingRatio = dampingRatio
}

Joint_GetConstraintTuning :: proc(jointId: JointId, hertz: ^f32, dampingRatio: ^f32) {
	world := GetWorld(i32(jointId.world0))
	joint := GetJointFullId(world, jointId)
	base := GetJointSim(world, joint)
	hertz^ = base.constraintHertz
	dampingRatio^ = base.constraintDampingRatio
}

Joint_SetForceThreshold :: proc(jointId: JointId, threshold: f32) {
	assert(IsValidFloat(threshold) && threshold >= 0.0)

	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_JointSetForceThreshold{jointId, threshold}
		RecWrite_JointSetForceThreshold(world.recording, &recArgs)
	}
	joint := GetJointFullId(world, jointId)
	base := GetJointSim(world, joint)
	base.forceThreshold = threshold
}

Joint_GetForceThreshold :: proc(jointId: JointId) -> f32 {
	world := GetWorld(i32(jointId.world0))
	joint := GetJointFullId(world, jointId)
	base := GetJointSim(world, joint)
	return base.forceThreshold
}

Joint_SetTorqueThreshold :: proc(jointId: JointId, threshold: f32) {
	assert(IsValidFloat(threshold) && threshold >= 0.0)

	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_JointSetTorqueThreshold{jointId, threshold}
		RecWrite_JointSetTorqueThreshold(world.recording, &recArgs)
	}
	joint := GetJointFullId(world, jointId)
	base := GetJointSim(world, joint)
	base.torqueThreshold = threshold
}

Joint_GetTorqueThreshold :: proc(jointId: JointId) -> f32 {
	world := GetWorld(i32(jointId.world0))
	joint := GetJointFullId(world, jointId)
	base := GetJointSim(world, joint)
	return base.torqueThreshold
}

CreateDistanceJoint :: proc(worldId: WorldId, def: ^DistanceJointDef) -> JointId {
	CHECK_JOINT_DEF(def)
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return JointId{}
	}

	assert(IsValidFloat(def.length) && def.length > 0.0)
	assert(def.lowerSpringForce <= def.upperSpringForce)

	pair := CreateJoint(world, &def.base, .distanceJoint)

	joint := pair.jointSim

	joint.distanceJoint = DistanceJoint{}
	joint.distanceJoint.length = MaxFloat(def.length, LINEAR_SLOP)
	joint.distanceJoint.hertz = def.hertz
	joint.distanceJoint.dampingRatio = def.dampingRatio
	joint.distanceJoint.lowerSpringForce = def.lowerSpringForce
	joint.distanceJoint.upperSpringForce = def.upperSpringForce
	joint.distanceJoint.minLength = MaxFloat(def.minLength, LINEAR_SLOP)
	joint.distanceJoint.maxLength = MaxFloat(def.minLength, def.maxLength)
	joint.distanceJoint.maxMotorForce = def.maxMotorForce
	joint.distanceJoint.motorSpeed = def.motorSpeed
	joint.distanceJoint.enableSpring = def.enableSpring
	joint.distanceJoint.enableLimit = def.enableLimit
	joint.distanceJoint.enableMotor = def.enableMotor
	joint.distanceJoint.impulse = 0.0
	joint.distanceJoint.lowerImpulse = 0.0
	joint.distanceJoint.upperImpulse = 0.0
	joint.distanceJoint.motorImpulse = 0.0

	jointId := JointId{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := RecArgs_CreateDistanceJoint{worldId, def^}
		RecWriteRet_CreateDistanceJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

CreateMotorJoint :: proc(worldId: WorldId, def: ^MotorJointDef) -> JointId {
	CHECK_JOINT_DEF(def)
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return JointId{}
	}

	pair := CreateJoint(world, &def.base, .motorJoint)
	joint := pair.jointSim

	joint.motorJoint = MotorJoint{}
	joint.motorJoint.linearVelocity = def.linearVelocity
	joint.motorJoint.maxVelocityForce = def.maxVelocityForce
	joint.motorJoint.angularVelocity = def.angularVelocity
	joint.motorJoint.maxVelocityTorque = def.maxVelocityTorque
	joint.motorJoint.linearHertz = def.linearHertz
	joint.motorJoint.linearDampingRatio = def.linearDampingRatio
	joint.motorJoint.maxSpringForce = def.maxSpringForce
	joint.motorJoint.angularHertz = def.angularHertz
	joint.motorJoint.angularDampingRatio = def.angularDampingRatio
	joint.motorJoint.maxSpringTorque = def.maxSpringTorque

	jointId := JointId{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := RecArgs_CreateMotorJoint{worldId, def^}
		RecWriteRet_CreateMotorJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

CreateMoverJoint :: proc(worldId: WorldId, def: ^MoverJointDef) -> JointId {
	CHECK_JOINT_DEF(def)
	assert(IsValidVec3(def.linearVelocity))
	assert(IsValidFloat(def.maxForce) && def.maxForce >= 0.0)

	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return JointId{}
	}

	pair := CreateJoint(world, &def.base, .moverJoint)
	joint := pair.jointSim

	joint.moverJoint = MoverJoint{}
	joint.moverJoint.linearVelocity = def.linearVelocity
	joint.moverJoint.maxForce = def.maxForce

	jointId := JointId{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := RecArgs_CreateMoverJoint{worldId, def^}
		RecWriteRet_CreateMoverJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

CreatePogoJoint :: proc(worldId: WorldId, def: ^PogoJointDef) -> JointId {
	CHECK_JOINT_DEF(def)
	assert(IsValidVec3(def.normal) && IsNormalized(def.normal))
	assert(IsValidFloat(def.restLength) && def.restLength >= 0.0)
	assert(IsValidFloat(def.hertz) && def.hertz >= 0.0)
	assert(IsValidFloat(def.dampingRatio) && def.dampingRatio >= 0.0)
	assert(IsValidFloat(def.maxTensionForce) && def.maxTensionForce >= 0.0)
	assert(IsValidFloat(def.maxCompressionForce) && def.maxCompressionForce >= 0.0)
	assert(IsValidFloat(def.impulse))
	assert(IsValidFloat(def.velocity))

	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return JointId{}
	}

	pair := CreateJoint(world, &def.base, .pogoJoint)
	joint := pair.jointSim

	joint.pogoJoint = PogoJoint{}
	joint.pogoJoint.normal = def.normal
	joint.pogoJoint.restLength = def.restLength
	joint.pogoJoint.hertz = def.hertz
	joint.pogoJoint.dampingRatio = def.dampingRatio
	joint.pogoJoint.maxTensionForce = def.maxTensionForce
	joint.pogoJoint.maxCompressionForce = def.maxCompressionForce
	joint.pogoJoint.impulse = def.impulse
	joint.pogoJoint.velocity = def.velocity

	jointId := JointId{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := RecArgs_CreatePogoJoint{worldId, def^}
		RecWriteRet_CreatePogoJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

CreateFilterJoint :: proc(worldId: WorldId, def: ^FilterJointDef) -> JointId {
	CHECK_JOINT_DEF(def)
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return JointId{}
	}

	pair := CreateJoint(world, &def.base, .filterJoint)

	joint := pair.jointSim

	jointId := JointId{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := RecArgs_CreateFilterJoint{worldId, def^}
		RecWriteRet_CreateFilterJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

CreateParallelJoint :: proc(worldId: WorldId, def: ^ParallelJointDef) -> JointId {
	CHECK_JOINT_DEF(def)
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return JointId{}
	}

	assert(IsValidFloat(def.hertz) && def.hertz >= 0.0)
	assert(IsValidFloat(def.dampingRatio) && def.dampingRatio >= 0.0)
	assert(IsValidFloat(def.maxTorque) && def.maxTorque >= 0.0)

	pair := CreateJoint(world, &def.base, .parallelJoint)

	joint := pair.jointSim

	joint.parallelJoint = ParallelJoint{}
	joint.parallelJoint.hertz = def.hertz
	joint.parallelJoint.dampingRatio = def.dampingRatio
	joint.parallelJoint.maxTorque = def.maxTorque

	jointId := JointId{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := RecArgs_CreateParallelJoint{worldId, def^}
		RecWriteRet_CreateParallelJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

CreatePrismaticJoint :: proc(worldId: WorldId, def: ^PrismaticJointDef) -> JointId {
	CHECK_JOINT_DEF(def)
	assert(def.lowerTranslation <= def.upperTranslation)

	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return JointId{}
	}

	pair := CreateJoint(world, &def.base, .prismaticJoint)

	joint := pair.jointSim

	joint.prismaticJoint = PrismaticJoint{}
	joint.prismaticJoint.hertz = def.hertz
	joint.prismaticJoint.dampingRatio = def.dampingRatio
	joint.prismaticJoint.targetTranslation = def.targetTranslation
	joint.prismaticJoint.lowerTranslation = def.lowerTranslation
	joint.prismaticJoint.upperTranslation = def.upperTranslation
	joint.prismaticJoint.maxMotorForce = def.maxMotorForce
	joint.prismaticJoint.motorSpeed = def.motorSpeed
	joint.prismaticJoint.enableSpring = def.enableSpring
	joint.prismaticJoint.enableLimit = def.enableLimit
	joint.prismaticJoint.enableMotor = def.enableMotor

	jointId := JointId{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := RecArgs_CreatePrismaticJoint{worldId, def^}
		RecWriteRet_CreatePrismaticJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

CreateRevoluteJoint :: proc(worldId: WorldId, def: ^RevoluteJointDef) -> JointId {
	CHECK_JOINT_DEF(def)

	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return JointId{}
	}

	pair := CreateJoint(world, &def.base, .revoluteJoint)

	joint := pair.jointSim

	joint.revoluteJoint = RevoluteJoint{}
	joint.revoluteJoint.hertz = def.hertz
	joint.revoluteJoint.dampingRatio = def.dampingRatio
	joint.revoluteJoint.targetAngle = ClampFloat(def.targetAngle, -PI, PI)

	lowerAngle := MinFloat(def.lowerAngle, def.upperAngle)
	upperAngle := MaxFloat(def.lowerAngle, def.upperAngle)
	joint.revoluteJoint.lowerAngle = ClampFloat(lowerAngle, -f32(0.99) * PI, f32(0.99) * PI)
	joint.revoluteJoint.upperAngle = ClampFloat(upperAngle, -f32(0.99) * PI, f32(0.99) * PI)

	joint.revoluteJoint.maxMotorTorque = def.maxMotorTorque
	joint.revoluteJoint.motorSpeed = def.motorSpeed
	joint.revoluteJoint.enableSpring = def.enableSpring
	joint.revoluteJoint.enableLimit = def.enableLimit
	joint.revoluteJoint.enableMotor = def.enableMotor

	jointId := JointId{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := RecArgs_CreateRevoluteJoint{worldId, def^}
		RecWriteRet_CreateRevoluteJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

CreateSphericalJoint :: proc(worldId: WorldId, def: ^SphericalJointDef) -> JointId {
	CHECK_JOINT_DEF(def)
	assert(0.0 <= def.coneAngle && def.coneAngle <= f32(0.99) * PI)
	assert(IsValidQuat(def.targetRotation))

	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return JointId{}
	}

	pair := CreateJoint(world, &def.base, .sphericalJoint)

	joint := pair.jointSim

	joint.sphericalJoint = SphericalJoint{}
	joint.sphericalJoint.hertz = def.hertz
	joint.sphericalJoint.dampingRatio = def.dampingRatio
	joint.sphericalJoint.targetRotation = def.targetRotation
	joint.sphericalJoint.coneAngle = ClampFloat(def.coneAngle, 0.0, f32(0.5) * PI)

	lowerAngle := MinFloat(def.lowerTwistAngle, def.upperTwistAngle)
	upperAngle := MaxFloat(def.lowerTwistAngle, def.upperTwistAngle)
	joint.sphericalJoint.lowerTwistAngle = ClampFloat(lowerAngle, -f32(0.99) * PI, f32(0.99) * PI)
	joint.sphericalJoint.upperTwistAngle = ClampFloat(upperAngle, -f32(0.99) * PI, f32(0.99) * PI)

	joint.sphericalJoint.maxMotorTorque = def.maxMotorTorque
	joint.sphericalJoint.motorVelocity = def.motorVelocity
	joint.sphericalJoint.enableSpring = def.enableSpring
	joint.sphericalJoint.enableConeLimit = def.enableConeLimit
	joint.sphericalJoint.enableTwistLimit = def.enableTwistLimit
	joint.sphericalJoint.enableMotor = def.enableMotor

	jointId := JointId{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := RecArgs_CreateSphericalJoint{worldId, def^}
		RecWriteRet_CreateSphericalJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

CreateWeldJoint :: proc(worldId: WorldId, def: ^WeldJointDef) -> JointId {
	CHECK_JOINT_DEF(def)
	assert(0.0 <= def.angularHertz)
	assert(0.0 <= def.angularDampingRatio)
	assert(0.0 <= def.linearHertz)
	assert(0.0 <= def.linearDampingRatio)

	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return JointId{}
	}

	pair := CreateJoint(world, &def.base, .weldJoint)

	joint := pair.jointSim

	joint.weldJoint = WeldJoint{}
	joint.weldJoint.linearHertz = def.linearHertz
	joint.weldJoint.linearDampingRatio = def.linearDampingRatio
	joint.weldJoint.angularHertz = def.angularHertz
	joint.weldJoint.angularDampingRatio = def.angularDampingRatio

	jointId := JointId{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := RecArgs_CreateWeldJoint{worldId, def^}
		RecWriteRet_CreateWeldJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

CreateWheelJoint :: proc(worldId: WorldId, def: ^WheelJointDef) -> JointId {
	CHECK_JOINT_DEF(def)
	assert(def.lowerSuspensionLimit <= def.upperSuspensionLimit)

	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return JointId{}
	}

	pair := CreateJoint(world, &def.base, .wheelJoint)

	joint := pair.jointSim

	joint.wheelJoint = WheelJoint{}
	joint.wheelJoint.enableSuspensionSpring = def.enableSuspensionSpring
	joint.wheelJoint.suspensionHertz = def.suspensionHertz
	joint.wheelJoint.suspensionDampingRatio = def.suspensionDampingRatio
	joint.wheelJoint.enableSuspensionLimit = def.enableSuspensionLimit
	joint.wheelJoint.lowerSuspensionLimit = def.lowerSuspensionLimit
	joint.wheelJoint.upperSuspensionLimit = def.upperSuspensionLimit
	joint.wheelJoint.enableSpinMotor = def.enableSpinMotor
	joint.wheelJoint.maxSpinTorque = def.maxSpinTorque
	joint.wheelJoint.spinSpeed = def.spinSpeed

	joint.wheelJoint.enableSteering = def.enableSteering
	joint.wheelJoint.steeringHertz = def.steeringHertz
	joint.wheelJoint.steeringDampingRatio = def.steeringDampingRatio
	joint.wheelJoint.targetSteeringAngle = def.targetSteeringAngle
	joint.wheelJoint.maxSteeringTorque = def.maxSteeringTorque
	joint.wheelJoint.enableSteeringLimit = def.enableSteeringLimit
	joint.wheelJoint.lowerSteeringLimit = def.lowerSteeringLimit
	joint.wheelJoint.upperSteeringLimit = def.upperSteeringLimit

	jointId := JointId{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := RecArgs_CreateWheelJoint{worldId, def^}
		RecWriteRet_CreateWheelJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

DestroyJointInternal :: proc(world: ^World, joint: ^Joint, wakeBodies: bool) {
	jointId := joint.jointId

	edgeA := &joint.edges[0]
	edgeB := &joint.edges[1]

	idA := edgeA.bodyId
	idB := edgeB.bodyId
	bodyA := Array_Get(&world.bodies, idA)
	bodyB := Array_Get(&world.bodies, idB)

	// Remove from body A
	if edgeA.prevKey != NULL_INDEX {
		prevJoint := Array_Get(&world.joints, edgeA.prevKey >> 1)
		prevEdge := &prevJoint.edges[edgeA.prevKey & 1]
		prevEdge.nextKey = edgeA.nextKey
	}

	if edgeA.nextKey != NULL_INDEX {
		nextJoint := Array_Get(&world.joints, edgeA.nextKey >> 1)
		nextEdge := &nextJoint.edges[edgeA.nextKey & 1]
		nextEdge.prevKey = edgeA.prevKey
	}

	edgeKeyA := (jointId << 1) | 0
	if bodyA.headJointKey == edgeKeyA {
		bodyA.headJointKey = edgeA.nextKey
	}

	bodyA.jointCount -= 1

	// Remove from body B
	if edgeB.prevKey != NULL_INDEX {
		prevJoint := Array_Get(&world.joints, edgeB.prevKey >> 1)
		prevEdge := &prevJoint.edges[edgeB.prevKey & 1]
		prevEdge.nextKey = edgeB.nextKey
	}

	if edgeB.nextKey != NULL_INDEX {
		nextJoint := Array_Get(&world.joints, edgeB.nextKey >> 1)
		nextEdge := &nextJoint.edges[edgeB.nextKey & 1]
		nextEdge.prevKey = edgeB.prevKey
	}

	edgeKeyB := (jointId << 1) | 1
	if bodyB.headJointKey == edgeKeyB {
		bodyB.headJointKey = edgeB.nextKey
	}

	bodyB.jointCount -= 1

	if joint.islandId != NULL_INDEX {
		assert(joint.setIndex > i32(SetType.disabledSet))
		UnlinkJoint(world, joint)
	} else {
		assert(joint.setIndex <= i32(SetType.disabledSet))
	}

	// Remove joint from solver set that owns it
	setIndex := joint.setIndex
	localIndex := joint.localIndex

	if setIndex == i32(SetType.awakeSet) {
		RemoveJointFromGraph(world, joint.edges[0].bodyId, joint.edges[1].bodyId, joint.colorIndex, localIndex)
	} else {
		set := Array_Get(&world.solverSets, setIndex)
		movedIndex := Array_RemoveSwap(&set.jointSims, localIndex)
		if movedIndex != NULL_INDEX {
			// Fix moved joint
			movedJointSim := &set.jointSims.data[localIndex]
			movedId := movedJointSim.jointId
			movedJoint := Array_Get(&world.joints, movedId)
			assert(movedJoint.localIndex == movedIndex)
			movedJoint.localIndex = localIndex
		}
	}

	// Free joint and id (preserve joint revision)
	joint.setIndex = NULL_INDEX
	joint.localIndex = NULL_INDEX
	joint.colorIndex = NULL_INDEX
	joint.jointId = NULL_INDEX
	FreeId(&world.jointIdPool, jointId)

	if wakeBodies {
		WakeBody(world, bodyA)
		WakeBody(world, bodyB)
	}

	ValidateSolverSets(world)
}

DestroyJoint :: proc(jointId: JointId, wakeAttached: bool) {
	world := GetWorld(i32(jointId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_DestroyJoint{jointId, wakeAttached}
		RecWrite_DestroyJoint(world.recording, &recArgs)
	}

	joint := GetJointFullId(world, jointId)

	DestroyJointInternal(world, joint, wakeAttached)
}

Joint_GetType :: proc(jointId: JointId) -> JointType {
	world := GetWorld(i32(jointId.world0))
	joint := GetJointFullId(world, jointId)
	return joint.type
}

Joint_GetBodyA :: proc(jointId: JointId) -> BodyId {
	world := GetWorld(i32(jointId.world0))
	joint := GetJointFullId(world, jointId)
	return MakeBodyId(world, joint.edges[0].bodyId)
}

Joint_GetBodyB :: proc(jointId: JointId) -> BodyId {
	world := GetWorld(i32(jointId.world0))
	joint := GetJointFullId(world, jointId)
	return MakeBodyId(world, joint.edges[1].bodyId)
}

Joint_GetWorld :: proc(jointId: JointId) -> WorldId {
	world := GetWorld(i32(jointId.world0))
	return WorldId{u16(jointId.world0 + 1), world.generation}
}

Joint_SetLocalFrameA :: proc(jointId: JointId, localFrame: Transform) {
	assert(IsValidTransform(localFrame))

	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_JointSetLocalFrameA{jointId, localFrame}
		RecWrite_JointSetLocalFrameA(world.recording, &recArgs)
	}
	joint := GetJointFullId(world, jointId)
	jointSim := GetJointSim(world, joint)
	jointSim.localFrameA = localFrame
}

Joint_GetLocalFrameA :: proc(jointId: JointId) -> Transform {
	world := GetWorld(i32(jointId.world0))
	joint := GetJointFullId(world, jointId)
	jointSim := GetJointSim(world, joint)
	return jointSim.localFrameA
}

Joint_SetLocalFrameB :: proc(jointId: JointId, localFrame: Transform) {
	assert(IsValidTransform(localFrame))

	world := GetWorld(i32(jointId.world0))
	if world.recording != nil {
		recArgs := RecArgs_JointSetLocalFrameB{jointId, localFrame}
		RecWrite_JointSetLocalFrameB(world.recording, &recArgs)
	}
	joint := GetJointFullId(world, jointId)
	jointSim := GetJointSim(world, joint)
	jointSim.localFrameB = localFrame
}

Joint_GetLocalFrameB :: proc(jointId: JointId) -> Transform {
	world := GetWorld(i32(jointId.world0))
	joint := GetJointFullId(world, jointId)
	jointSim := GetJointSim(world, joint)
	return jointSim.localFrameB
}

Joint_SetCollideConnected :: proc(jointId: JointId, shouldCollide: bool) {
	world := GetUnlockedWorld(i32(jointId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_JointSetCollideConnected{jointId, shouldCollide}
		RecWrite_JointSetCollideConnected(world.recording, &recArgs)
	}

	joint := GetJointFullId(world, jointId)
	if joint.collideConnected == shouldCollide {
		return
	}

	joint.collideConnected = shouldCollide

	bodyA := Array_Get(&world.bodies, joint.edges[0].bodyId)
	bodyB := Array_Get(&world.bodies, joint.edges[1].bodyId)

	if shouldCollide {
		// need to tell the broad-phase to look for new pairs for one of the
		// two bodies. Pick the one with the fewest shapes.
		shapeCountA := bodyA.shapeCount
		shapeCountB := bodyB.shapeCount

		shapeId := shapeCountA < shapeCountB ? bodyA.headShapeId : bodyB.headShapeId
		for shapeId != NULL_INDEX {
			shape := Array_Get(&world.shapes, shapeId)

			if shape.proxyKey != NULL_INDEX {
				BroadPhase_MarkProxyMovedSerial(&world.broadPhase, shape.proxyKey)
			}

			shapeId = shape.nextShapeId
		}
	} else {
		DestroyContactsBetweenBodies(world, bodyA, bodyB)
	}
}

Joint_GetCollideConnected :: proc(jointId: JointId) -> bool {
	world := GetWorld(i32(jointId.world0))
	joint := GetJointFullId(world, jointId)
	return joint.collideConnected
}

Joint_SetUserData :: proc(jointId: JointId, userData: rawptr) {
	world := GetWorld(i32(jointId.world0))
	joint := GetJointFullId(world, jointId)
	joint.userData = userData
}

Joint_GetUserData :: proc(jointId: JointId) -> rawptr {
	world := GetWorld(i32(jointId.world0))
	joint := GetJointFullId(world, jointId)
	return joint.userData
}

Joint_WakeBodies :: proc(jointId: JointId) {
	world := GetUnlockedWorld(i32(jointId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_JointWakeBodies{jointId}
		RecWrite_JointWakeBodies(world.recording, &recArgs)
	}

	world.locked = true

	joint := GetJointFullId(world, jointId)
	bodyA := Array_Get(&world.bodies, joint.edges[0].bodyId)
	bodyB := Array_Get(&world.bodies, joint.edges[1].bodyId)

	WakeBody(world, bodyA)
	WakeBody(world, bodyB)

	world.locked = false
}

Joint_IsAwake :: proc(jointId: JointId) -> bool {
	world := GetWorld(i32(jointId.world0))
	joint := GetJointFullId(world, jointId)

	// Cheaper than checking if the bodies are awake.
	return joint.setIndex == i32(SetType.awakeSet)
}

GetJointReaction :: proc(world: ^World, sim: ^JointSim, invTimeStep: f32, force: ^f32, torque: ^f32) {
	linearImpulse: f32 = 0.0
	angularImpulse: f32 = 0.0

	#partial switch sim.type {
	case .parallelJoint:
		{
			joint := &sim.parallelJoint
			impulse := Vec3 {
				x = joint.perpImpulse.x,
				y = joint.perpImpulse.y,
				z = 0.0,
			}
			angularImpulse = Length(impulse)
		}

	case .distanceJoint:
		{
			joint := &sim.distanceJoint
			linearImpulse = AbsFloat(joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse)
		}

	case .motorJoint:
		{
			joint := &sim.motorJoint
			linearImpulse = Length(Add(joint.linearVelocityImpulse, joint.linearSpringImpulse))
			angularImpulse = Length(Add(joint.angularVelocityImpulse, joint.angularSpringImpulse))
		}

	case .prismaticJoint:
		{
			joint := &sim.prismaticJoint
			impulse := Vec3 {
				x = joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse,
				y = joint.perpImpulse.x,
				z = joint.perpImpulse.y,
			}
			linearImpulse = Length(impulse)
			angularImpulse = Length(joint.angularImpulse)
		}

	case .revoluteJoint:
		{
			joint := &sim.revoluteJoint
			linearImpulse = Length(joint.linearImpulse)
			impulse := Vec3 {
				x = joint.perpImpulse.x,
				y = joint.perpImpulse.y,
				z = joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse,
			}
			angularImpulse = Length(impulse)
		}

	case .sphericalJoint:
		{
			// todo improve performance
			joint := &sim.sphericalJoint
			linearImpulse = Length(joint.linearImpulse)

			xfA := GetBodyTransform(world, sim.bodyIdA)
			xfB := GetBodyTransform(world, sim.bodyIdB)
			qA := MulQuat(xfA.q, sim.localFrameA.q)
			qB := MulQuat(xfB.q, sim.localFrameB.q)

			// Cone axis is the z-axis of body A.
			coneAxis := RotateVector(qA, Vec3_axisZ)
			twistAxis := RotateVector(qB, Vec3_axisZ)
			swingAxis := Normalize(Cross(coneAxis, twistAxis))

			impulse := Add(joint.springImpulse, joint.motorImpulse)
			impulse = MulAdd(impulse, joint.lowerTwistImpulse - joint.upperTwistImpulse, twistAxis)
			impulse = MulAdd(impulse, joint.swingImpulse, swingAxis)

			angularImpulse = Length(impulse)
		}

	case .weldJoint:
		{
			joint := &sim.weldJoint
			linearImpulse = Length(joint.linearImpulse)
			angularImpulse = Length(joint.angularImpulse)
		}

	case .wheelJoint:
		{
			// todo probably wrong
			joint := &sim.wheelJoint
			perpImpulse := joint.linearImpulse
			axialImpulse := joint.suspensionSpringImpulse + joint.lowerSuspensionImpulse - joint.upperSuspensionImpulse
			linearImpulse = intrinsics.sqrt(
				perpImpulse.x * perpImpulse.x + perpImpulse.y * perpImpulse.y + axialImpulse * axialImpulse,
			)
			angularImpulse = AbsFloat(joint.spinImpulse)
		}

	case .moverJoint:
		{
			joint := &sim.moverJoint
			linearImpulse = Length2(joint.impulse)
		}

	case .pogoJoint:
		{
			joint := &sim.pogoJoint
			linearImpulse = AbsFloat(joint.impulse)
		}

	case:
	}

	force^ = linearImpulse * invTimeStep
	torque^ = angularImpulse * invTimeStep
}

@(private = "file")
GetJointConstraintForce :: proc(world: ^World, joint: ^Joint) -> Vec3 {
	base := GetJointSim(world, joint)

	switch joint.type {
	case .parallelJoint:
		return Vec3_zero

	case .distanceJoint:
		return GetDistanceJointForce(world, base)

	case .filterJoint:
		return Vec3_zero

	case .motorJoint:
		return GetMotorJointForce(world, base)

	case .prismaticJoint:
		return GetPrismaticJointForce(world, base)

	case .revoluteJoint:
		return GetRevoluteJointForce(world, base)

	case .sphericalJoint:
		return GetSphericalJointForce(world, base)

	case .weldJoint:
		return GetWeldJointForce(world, base)

	case .wheelJoint:
		return GetWheelJointForce(world, base)

	case .moverJoint:
		return GetMoverJointForce(world, base)

	case .pogoJoint:
		return GetPogoJointForce(world, base)

	case:
		assert(false)
		return Vec3_zero
	}
}

@(private = "file")
GetJointConstraintTorque :: proc(world: ^World, joint: ^Joint) -> Vec3 {
	base := GetJointSim(world, joint)

	switch joint.type {
	case .parallelJoint:
		return GetParallelJointTorque(world, base)

	case .distanceJoint:
		return Vec3_zero

	case .filterJoint:
		return Vec3_zero

	case .motorJoint:
		return GetMotorJointTorque(world, base)

	case .prismaticJoint:
		return GetPrismaticJointTorque(world, base)

	case .revoluteJoint:
		return GetRevoluteJointTorque(world, base)

	case .sphericalJoint:
		return GetSphericalJointTorque(world, base)

	case .weldJoint:
		return GetWeldJointTorque(world, base)

	case .wheelJoint:
		return GetWheelJointTorque(world, base)

	case .moverJoint, .pogoJoint:
		return Vec3_zero

	case:
		assert(false)
		return Vec3_zero
	}
}

Joint_GetConstraintForce :: proc(jointId: JointId) -> Vec3 {
	world := GetWorld(i32(jointId.world0))
	joint := GetJointFullId(world, jointId)
	return GetJointConstraintForce(world, joint)
}

Joint_GetConstraintTorque :: proc(jointId: JointId) -> Vec3 {
	world := GetWorld(i32(jointId.world0))
	joint := GetJointFullId(world, jointId)
	return GetJointConstraintTorque(world, joint)
}

Joint_GetLinearSeparation :: proc(jointId: JointId) -> f32 {
	world := GetWorld(i32(jointId.world0))
	joint := GetJointFullId(world, jointId)
	base := GetJointSim(world, joint)

	xfA := GetBodyTransform(world, joint.edges[0].bodyId)
	xfB := GetBodyTransform(world, joint.edges[1].bodyId)

	pA := TransformWorldPoint(xfA, base.localFrameA.p)
	pB := TransformWorldPoint(xfB, base.localFrameB.p)
	dp := SubPos(pB, pA)

	switch joint.type {
	case .parallelJoint:
		return 0.0

	case .distanceJoint:
		{
			distanceJoint := &base.distanceJoint
			length := Length(dp)
			if distanceJoint.enableSpring {
				if distanceJoint.enableLimit {
					if length < distanceJoint.minLength {
						return distanceJoint.minLength - length
					}

					if length > distanceJoint.maxLength {
						return length - distanceJoint.maxLength
					}

					return 0.0
				}

				return 0.0
			}

			return AbsFloat(length - distanceJoint.length)
		}

	case .motorJoint:
		return 0.0

	case .moverJoint:
		return 0.0

	case .pogoJoint:
		return 0.0

	case .filterJoint:
		return 0.0

	case .prismaticJoint:
		{
			prismaticJoint := &base.prismaticJoint
			axisA := RotateVector(xfA.q, Vec3_axisX)
			perpA := Perp(axisA)
			perpendicularSeparation := AbsFloat(Dot(perpA, dp))
			limitSeparation: f32 = 0.0

			if prismaticJoint.enableLimit {
				translation := Dot(axisA, dp)
				if translation < prismaticJoint.lowerTranslation {
					limitSeparation = prismaticJoint.lowerTranslation - translation
				}

				if prismaticJoint.upperTranslation < translation {
					limitSeparation = translation - prismaticJoint.upperTranslation
				}
			}

			return intrinsics.sqrt(
				perpendicularSeparation * perpendicularSeparation + limitSeparation * limitSeparation,
			)
		}

	case .revoluteJoint:
		return Length(dp)

	case .sphericalJoint:
		return Length(dp)

	case .weldJoint:
		{
			weldJoint := &base.weldJoint
			if weldJoint.linearHertz == 0.0 {
				return Length(dp)
			}

			return 0.0
		}

	case .wheelJoint:
		{
			wheelJoint := &base.wheelJoint
			axisA := RotateVector(xfA.q, Vec3_axisX)
			perpA := Perp(axisA)
			perpendicularSeparation := AbsFloat(Dot(perpA, dp))
			limitSeparation: f32 = 0.0

			if wheelJoint.enableSuspensionLimit {
				translation := Dot(axisA, dp)
				if translation < wheelJoint.lowerSuspensionLimit {
					limitSeparation = wheelJoint.lowerSuspensionLimit - translation
				}

				if wheelJoint.upperSuspensionLimit < translation {
					limitSeparation = translation - wheelJoint.upperSuspensionLimit
				}
			}

			return intrinsics.sqrt(
				perpendicularSeparation * perpendicularSeparation + limitSeparation * limitSeparation,
			)
		}

	case:
		assert(false)
		return 0.0
	}
}

Joint_GetAngularSeparation :: proc(jointId: JointId) -> f32 {
	world := GetWorld(i32(jointId.world0))
	joint := GetJointFullId(world, jointId)
	base := GetJointSim(world, joint)

	xfA := GetBodyTransform(world, joint.edges[0].bodyId)
	xfB := GetBodyTransform(world, joint.edges[1].bodyId)

	relQ := InvMulQuat(xfA.q, xfB.q)

	switch joint.type {
	case .parallelJoint:
		{
			// Remove hinge angle
			relQ.v.z = 0.0
			return GetQuatAngle(relQ)
		}

	case .distanceJoint:
		return 0.0

	case .motorJoint:
		return 0.0

	case .moverJoint:
		return 0.0

	case .pogoJoint:
		return 0.0

	case .filterJoint:
		return 0.0

	case .prismaticJoint:
		return GetQuatAngle(relQ)

	case .revoluteJoint:
		{
			revoluteJoint := &base.revoluteJoint
			if revoluteJoint.enableLimit {
				angle := GetTwistAngle(relQ)
				if angle < revoluteJoint.lowerAngle {
					return GetQuatAngle(relQ)
				}

				if revoluteJoint.upperAngle < angle {
					return GetQuatAngle(relQ)
				}
			}

			// Remove hinge angle
			relQ.v.z = 0.0
			return GetQuatAngle(relQ)
		}

	case .sphericalJoint:
		{
			sphericalJoint := &base.sphericalJoint
			sum: f32 = 0.0
			if sphericalJoint.enableConeLimit {
				swingAngle := GetSwingAngle(relQ)
				sum += MaxFloat(0.0, swingAngle - sphericalJoint.coneAngle)
			}

			if sphericalJoint.enableTwistLimit {
				twistAngle := GetTwistAngle(relQ)
				sum += MaxFloat(0.0, sphericalJoint.lowerTwistAngle - twistAngle)
				sum += MaxFloat(0.0, twistAngle - sphericalJoint.upperTwistAngle)
			}

			return sum
		}

	case .weldJoint:
		{
			weldJoint := &base.weldJoint
			if weldJoint.angularHertz == 0.0 {
				return GetQuatAngle(relQ)
			}

			return 0.0
		}

	case .wheelJoint:
		// todo
		assert(false)
		return 0.0

	case:
		assert(false)
		return 0.0
	}
}

// unified: an `#if 0` block upstream, kept inside `when false` (not type-checked; DotQuat and quaternion negation
// don't exist)
when false {
	Joint_SetSpringRotationTarget :: proc(jointId: JointId, relativeBodyRotation: Quat, hertz: f32) {
		assert(IsValidQuat(relativeBodyRotation))
		assert(IsValidFloat(hertz) && hertz > 0.0)

		world := GetWorld(i32(jointId.world0))
		joint := GetJointFullId(world, jointId)
		base := GetJointSim(world, joint)

		qA := base.localFrameA.q
		qB := MulQuat(relativeBodyRotation, base.localFrameB.q)

		// This keeps the twist angle in the range [-pi, pi]
		if DotQuat(qA, qB) < 0.0 {
			qA = -qA
		}

		relQ := InvMulQuat(qA, qB)

		#partial switch joint.type {
		case .revoluteJoint:
			base.revoluteJoint.targetAngle = GetTwistAngle(relQ)
			base.revoluteJoint.hertz = hertz

		case .sphericalJoint:
			base.sphericalJoint.targetRotation = relQ
			base.sphericalJoint.hertz = hertz

		case:
		}
	}
}
// unified end

PrepareJoint :: proc(joint: ^JointSim, context_: ^StepContext) {
	// Clamp joint hertz based on the time step to reduce jitter.
	hertz := MinFloat(joint.constraintHertz, 0.25 * context_.inv_h)
	joint.constraintSoftness = MakeSoft(hertz, joint.constraintDampingRatio, context_.h)

	switch joint.type {
	case .parallelJoint:
		PrepareParallelJoint(joint, context_)

	case .distanceJoint:
		PrepareDistanceJoint(joint, context_)

	case .filterJoint:

	case .motorJoint:
		PrepareMotorJoint(joint, context_)

	case .prismaticJoint:
		PreparePrismaticJoint(joint, context_)

	case .revoluteJoint:
		PrepareRevoluteJoint(joint, context_)

	case .sphericalJoint:
		PrepareSphericalJoint(joint, context_)

	case .weldJoint:
		PrepareWeldJoint(joint, context_)

	case .wheelJoint:
		PrepareWheelJoint(joint, context_)

	case .moverJoint:
		PrepareMoverJoint(joint, context_)

	case .pogoJoint:
		PreparePogoJoint(joint, context_)

	case:
		assert(false)
	}
}

WarmStartJoint :: proc(joint: ^JointSim, context_: ^StepContext) {
	switch joint.type {
	case .parallelJoint:
		WarmStartParallelJoint(joint, context_)

	case .distanceJoint:
		WarmStartDistanceJoint(joint, context_)

	case .filterJoint:

	case .motorJoint:
		WarmStartMotorJoint(joint, context_)

	case .prismaticJoint:
		WarmStartPrismaticJoint(joint, context_)

	case .revoluteJoint:
		WarmStartRevoluteJoint(joint, context_)

	case .sphericalJoint:
		WarmStartSphericalJoint(joint, context_)

	case .weldJoint:
		WarmStartWeldJoint(joint, context_)

	case .wheelJoint:
		WarmStartWheelJoint(joint, context_)

	case .moverJoint:
		WarmStartMoverJoint(joint, context_)

	case .pogoJoint:
		WarmStartPogoJoint(joint, context_)

	case:
		assert(false)
	}
}

SolveJoint :: proc(joint: ^JointSim, context_: ^StepContext, useBias: bool) {
	_ = useBias

	switch joint.type {
	case .parallelJoint:
		SolveParallelJoint(joint, context_)

	case .distanceJoint:
		SolveDistanceJoint(joint, context_, useBias)

	case .filterJoint:

	case .motorJoint:
		SolveMotorJoint(joint, context_)

	case .prismaticJoint:
		SolvePrismaticJoint(joint, context_, useBias)

	case .revoluteJoint:
		SolveRevoluteJoint(joint, context_, useBias)

	case .sphericalJoint:
		SolveSphericalJoint(joint, context_, useBias)

	case .weldJoint:
		SolveWeldJoint(joint, context_, useBias)

	case .wheelJoint:
		SolveWheelJoint(joint, context_, useBias)

	case .moverJoint:
		SolveMoverJoint(joint, context_)

	case .pogoJoint:
		SolvePogoJoint(joint, context_, useBias)

	case:
		assert(false)
	}
}

PrepareJoints_Overflow :: proc(context_: ^StepContext) {
	graph := context_.graph
	joints := graph.colors[OVERFLOW_INDEX].jointSims.data
	jointCount := graph.colors[OVERFLOW_INDEX].jointSims.count

	for i: i32 = 0; i < jointCount; i += 1 {
		joint := &joints[i]
		PrepareJoint(joint, context_)
	}
}

WarmStartJoints_Overflow :: proc(context_: ^StepContext) {
	graph := context_.graph
	joints := graph.colors[OVERFLOW_INDEX].jointSims.data
	jointCount := graph.colors[OVERFLOW_INDEX].jointSims.count

	for i: i32 = 0; i < jointCount; i += 1 {
		joint := &joints[i]
		WarmStartJoint(joint, context_)
	}
}

SolveJoints_Overflow :: proc(context_: ^StepContext, useBias: bool) {
	graph := context_.graph
	joints := graph.colors[OVERFLOW_INDEX].jointSims.data
	jointCount := graph.colors[OVERFLOW_INDEX].jointSims.count

	for i: i32 = 0; i < jointCount; i += 1 {
		joint := &joints[i]
		SolveJoint(joint, context_, useBias)
	}
}

DrawJoint :: proc(draw: ^DebugDraw, world: ^World, joint: ^Joint) {
	bodyA := Array_Get(&world.bodies, joint.edges[0].bodyId)
	bodyB := Array_Get(&world.bodies, joint.edges[1].bodyId)
	if bodyA.setIndex == i32(SetType.disabledSet) || bodyB.setIndex == i32(SetType.disabledSet) {
		return
	}

	jointSim := GetJointSim(world, joint)

	transformA := GetBodyTransformQuick(world, bodyA)
	transformB := GetBodyTransformQuick(world, bodyB)
	pA := TransformWorldPoint(transformA, jointSim.localFrameA.p)
	pB := TransformWorldPoint(transformB, jointSim.localFrameB.p)

	color := HexColor.colorDarkSeaGreen

	scale := MaxFloat(0.0001, draw.jointScale * joint.drawScale)

	switch joint.type {
	case .parallelJoint:
		DrawParallelJoint(draw, jointSim, transformA, transformB, scale)

	case .distanceJoint:
		DrawDistanceJoint(draw, jointSim, transformA, transformB)

	case .filterJoint:
		draw.DrawSegmentFcn(pA, pB, .colorGold, draw.context_)

	case .motorJoint:
		draw.DrawSegmentFcn(pA, pB, .colorPlum, draw.context_)
		draw.DrawPointFcn(pA, 8.0, .colorYellowGreen, draw.context_)
		draw.DrawPointFcn(pB, 8.0, .colorPlum, draw.context_)

	case .prismaticJoint:
		DrawPrismaticJoint(draw, jointSim, transformA, transformB, scale)

	case .revoluteJoint:
		DrawRevoluteJoint(draw, jointSim, transformA, transformB, scale)

	case .sphericalJoint:
		DrawSphericalJoint(draw, jointSim, transformA, transformB, scale)

	case .weldJoint:
		DrawWeldJoint(draw, jointSim, transformA, transformB, scale)

	case .wheelJoint:
		DrawWheelJoint(draw, jointSim, transformA, transformB, scale)

	case .moverJoint:
		DrawMoverJoint(draw, world, jointSim, transformA, transformB)

	case .pogoJoint:
		DrawPogoJoint(draw, world, jointSim, transformA, transformB, scale)

	case:
		draw.DrawSegmentFcn(transformA.p, pA, color, draw.context_)
		draw.DrawSegmentFcn(pA, pB, color, draw.context_)
		draw.DrawSegmentFcn(transformB.p, pB, color, draw.context_)
	}

	if draw.drawGraphColors {
		graphColors := [GRAPH_COLOR_COUNT]HexColor {
			.colorRed,
			.colorOrange,
			.colorYellow,
			.colorGreen,
			.colorCyan,
			.colorBlue,
			.colorViolet,
			.colorPink,
			.colorChocolate,
			.colorGoldenRod,
			.colorCoral,
			.colorRosyBrown,
			.colorAqua,
			.colorPeru,
			.colorLime,
			.colorGold,
			.colorPlum,
			.colorSnow,
			.colorTeal,
			.colorKhaki,
			.colorSalmon,
			.colorPeachPuff,
			.colorHoneyDew,
			.colorBlack,
		}

		colorIndex := joint.colorIndex
		if colorIndex != NULL_INDEX {
			p := LerpPosition(pA, pB, 0.5)
			draw.DrawPointFcn(p, 5.0, graphColors[colorIndex], draw.context_)
		}
	}

	if draw.drawJointExtras && joint.type != .moverJoint && joint.type != .pogoJoint {
		force := GetJointConstraintForce(world, joint)
		torque := GetJointConstraintTorque(world, joint)
		p := LerpPosition(pA, pB, 0.5)

		draw.DrawSegmentFcn(p, OffsetPos(p, MulSV(draw.forceScale, force)), .colorAzure, draw.context_)

		buffer: [64]u8
		// unified: snprintf is Odin's fmt into the buffer, NUL-terminated (Odin's %g can differ from C's in the digits
		// it prints; a debug label only)
		s := fmt.bprintf(buffer[:len(buffer) - 1], "f = %g, t = %g", Length(force), Length(torque))
		buffer[len(s)] = 0
		// unified end
		draw.DrawStringFcn(p, cstring(raw_data(buffer[:])), .colorAzure, draw.context_)
	}
}
