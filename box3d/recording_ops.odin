// Ported from box3d src/recording_ops.inl at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

// unified: the X-macro manifest is expanded by hand into everything its includes generate: the opcode
// constants and the b3RecArgs_* structs (recording.h), the b3RecWriteArgs_*, b3RecWrite_* and b3RecWriteRet_*
// writers (recording.c). Each section lists the ops in the manifest's order. The dispatch switch it generates in
// replay.c is expanded in RecDispatchOne (replay.odin).

// X-macro manifest for the recording system.
// Include with B3_REC_OP and ARG defined. No commas between ARG tokens.
//
// B3_REC_OP( opcode, Name, RET, ARGS )
//   RET in { RET_NONE, RET_BODYID, RET_SHAPEID, RET_JOINTID }
//   ARGS = zero or more ARG( TAG, fieldName ) tokens, NO commas between them
//
// Opcode ranges:
//   0x00-0x0F  world lifecycle and config
//   0x10-0x1F  body create/destroy
//   0x20-0x3F  body mutators
//   0x40-0x4F  shape create/destroy
//   0x50-0x6F  shape mutators
//   0x80       step
//   0x81-0x8F  mover and pogo joints
//   0x90-0xE7  joints (create, generic, per-type)
//   0xE8-0xEF  spatial queries
//   0xF0-0xFF  markers

// Opcode constants generated from the manifest, so call sites name an op instead of a raw byte and
// can't drift from the manifest if an op is renumbered.
recOpDestroyWorld :: 0x01
recOpStep :: 0x80
recOpWorldEnableSleeping :: 0x02
recOpWorldEnableContinuous :: 0x03
recOpWorldSetRestitutionThreshold :: 0x04
recOpWorldSetHitEventThreshold :: 0x05
recOpWorldSetGravity :: 0x06
recOpWorldExplode :: 0x07
recOpWorldSetContactTuning :: 0x08
recOpWorldSetContactRecycleDistance :: 0x09
recOpWorldSetMaximumLinearSpeed :: 0x0A
recOpWorldEnableWarmStarting :: 0x0B
recOpWorldRebuildStaticTree :: 0x0C
recOpWorldEnableSpeculative :: 0x0D
recOpWorldSetRestitutionIterations :: 0x0E
recOpWorldEnableRestitutionPropagation :: 0x0F
recOpWorldSetIsotropicInertiaTolerance :: 0x70
recOpCreateBody :: 0x10
recOpDestroyBody :: 0x11
recOpBodySetTransform :: 0x20
recOpBodySetLinearVelocity :: 0x21
recOpBodySetType :: 0x22
recOpBodySetName :: 0x23
recOpBodySetAngularVelocity :: 0x24
recOpBodySetTargetTransform :: 0x25
recOpBodyApplyForce :: 0x26
recOpBodyApplyForceToCenter :: 0x27
recOpBodyApplyTorque :: 0x28
recOpBodyApplyLinearImpulse :: 0x29
recOpBodyApplyLinearImpulseToCenter :: 0x2A
recOpBodyApplyAngularImpulse :: 0x2B
recOpBodySetMassData :: 0x2C
recOpBodyApplyMassFromShapes :: 0x2D
recOpBodySetLinearDamping :: 0x2E
recOpBodySetAngularDamping :: 0x2F
recOpBodySetGravityScale :: 0x30
recOpBodySetAwake :: 0x31
recOpBodyEnableSleep :: 0x32
recOpBodySetSleepThreshold :: 0x33
recOpBodyDisable :: 0x34
recOpBodyEnable :: 0x35
recOpBodySetMotionLocks :: 0x36
recOpBodySetBullet :: 0x37
recOpBodyEnableContactRecycling :: 0x38
recOpBodyEnableHitEvents :: 0x39
recOpBodyAllowFastRotation :: 0x3A
recOpBodySetSafetyFactor :: 0x3B
recOpCreateSphereShape :: 0x40
recOpCreateCapsuleShape :: 0x41
recOpCreateHullShape :: 0x42
recOpCreateMeshShape :: 0x43
recOpCreateHeightFieldShape :: 0x44
recOpCreateCompoundShape :: 0x45
recOpDestroyShape :: 0x46
recOpShapeSetDensity :: 0x50
recOpShapeSetFriction :: 0x51
recOpShapeSetRestitution :: 0x52
recOpShapeSetSurfaceMaterial :: 0x53
recOpShapeSetFilter :: 0x54
recOpShapeEnableSensorEvents :: 0x55
recOpShapeEnableContactEvents :: 0x56
recOpShapeEnablePreSolveEvents :: 0x57
recOpShapeEnableHitEvents :: 0x58
recOpShapeSetSphere :: 0x59
recOpShapeSetCapsule :: 0x5A
recOpShapeApplyWind :: 0x5B
recOpShapeSetName :: 0x5C
recOpShapeSetMeshMaterial :: 0x5D
recOpShapeSetHull :: 0x5E
recOpShapeSetMesh :: 0x5F
recOpCreateParallelJoint :: 0x90
recOpCreateDistanceJoint :: 0x91
recOpCreateFilterJoint :: 0x92
recOpCreateMotorJoint :: 0x93
recOpCreatePrismaticJoint :: 0x94
recOpCreateRevoluteJoint :: 0x95
recOpCreateSphericalJoint :: 0x96
recOpCreateWeldJoint :: 0x97
recOpCreateWheelJoint :: 0x98
recOpDestroyJoint :: 0x99
recOpJointSetLocalFrameA :: 0x9A
recOpJointSetLocalFrameB :: 0x9B
recOpJointSetCollideConnected :: 0x9C
recOpJointWakeBodies :: 0x9D
recOpJointSetConstraintTuning :: 0x9E
recOpJointSetForceThreshold :: 0x9F
recOpJointSetTorqueThreshold :: 0xA0
recOpParallelJointSetSpringHertz :: 0xA1
recOpParallelJointSetSpringDampingRatio :: 0xA2
recOpParallelJointSetMaxTorque :: 0xA3
recOpDistanceJointSetLength :: 0xA4
recOpDistanceJointEnableSpring :: 0xA5
recOpDistanceJointSetSpringForceRange :: 0xA6
recOpDistanceJointSetSpringHertz :: 0xA7
recOpDistanceJointSetSpringDampingRatio :: 0xA8
recOpDistanceJointEnableLimit :: 0xA9
recOpDistanceJointSetLengthRange :: 0xAA
recOpDistanceJointEnableMotor :: 0xAB
recOpDistanceJointSetMotorSpeed :: 0xAC
recOpDistanceJointSetMaxMotorForce :: 0xAD
recOpMotorJointSetLinearVelocity :: 0xAE
recOpMotorJointSetAngularVelocity :: 0xAF
recOpMotorJointSetMaxVelocityForce :: 0xB0
recOpMotorJointSetMaxVelocityTorque :: 0xB1
recOpMotorJointSetLinearHertz :: 0xB2
recOpMotorJointSetLinearDampingRatio :: 0xB3
recOpMotorJointSetAngularHertz :: 0xB4
recOpMotorJointSetAngularDampingRatio :: 0xB5
recOpMotorJointSetMaxSpringForce :: 0xB6
recOpMotorJointSetMaxSpringTorque :: 0xB7
recOpPrismaticJointEnableSpring :: 0xB8
recOpPrismaticJointSetSpringHertz :: 0xB9
recOpPrismaticJointSetSpringDampingRatio :: 0xBA
recOpPrismaticJointSetTargetTranslation :: 0xBB
recOpPrismaticJointEnableLimit :: 0xBC
recOpPrismaticJointSetLimits :: 0xBD
recOpPrismaticJointEnableMotor :: 0xBE
recOpPrismaticJointSetMotorSpeed :: 0xBF
recOpPrismaticJointSetMaxMotorForce :: 0xC0
recOpRevoluteJointEnableSpring :: 0xC1
recOpRevoluteJointSetSpringHertz :: 0xC2
recOpRevoluteJointSetSpringDampingRatio :: 0xC3
recOpRevoluteJointSetTargetAngle :: 0xC4
recOpRevoluteJointEnableLimit :: 0xC5
recOpRevoluteJointSetLimits :: 0xC6
recOpRevoluteJointEnableMotor :: 0xC7
recOpRevoluteJointSetMotorSpeed :: 0xC8
recOpRevoluteJointSetMaxMotorTorque :: 0xC9
recOpSphericalJointEnableConeLimit :: 0xCA
recOpSphericalJointSetConeLimit :: 0xCB
recOpSphericalJointEnableTwistLimit :: 0xCC
recOpSphericalJointSetTwistLimits :: 0xCD
recOpSphericalJointEnableSpring :: 0xCE
recOpSphericalJointSetSpringHertz :: 0xCF
recOpSphericalJointSetSpringDampingRatio :: 0xD0
recOpSphericalJointSetTargetRotation :: 0xD1
recOpSphericalJointEnableMotor :: 0xD2
recOpSphericalJointSetMotorVelocity :: 0xD3
recOpSphericalJointSetMaxMotorTorque :: 0xD4
recOpWeldJointSetLinearHertz :: 0xD5
recOpWeldJointSetLinearDampingRatio :: 0xD6
recOpWeldJointSetAngularHertz :: 0xD7
recOpWeldJointSetAngularDampingRatio :: 0xD8
recOpWheelJointEnableSuspension :: 0xD9
recOpWheelJointSetSuspensionHertz :: 0xDA
recOpWheelJointSetSuspensionDampingRatio :: 0xDB
recOpWheelJointEnableSuspensionLimit :: 0xDC
recOpWheelJointSetSuspensionLimits :: 0xDD
recOpWheelJointEnableSpinMotor :: 0xDE
recOpWheelJointSetSpinMotorSpeed :: 0xDF
recOpWheelJointSetMaxSpinTorque :: 0xE0
recOpWheelJointEnableSteering :: 0xE1
recOpWheelJointSetSteeringHertz :: 0xE2
recOpWheelJointSetSteeringDampingRatio :: 0xE3
recOpWheelJointSetMaxSteeringTorque :: 0xE4
recOpWheelJointEnableSteeringLimit :: 0xE5
recOpWheelJointSetSteeringLimits :: 0xE6
recOpWheelJointSetTargetSteeringAngle :: 0xE7
recOpCreateMoverJoint :: 0x81
recOpMoverJointSetLinearVelocity :: 0x82
recOpMoverJointSetMaxVelocityForce :: 0x83
recOpCreatePogoJoint :: 0x84
recOpPogoJointSetRestLength :: 0x85
recOpPogoJointSetSpringHertz :: 0x86
recOpPogoJointSetSpringDampingRatio :: 0x87
recOpPogoJointSetMaxTensionForce :: 0x88
recOpPogoJointSetMaxCompressionForce :: 0x89
recOpPogoJointSetNormal :: 0x8A
recOpQueryOverlapAABB :: 0xE8
recOpQueryOverlapShape :: 0xE9
recOpQueryCastRay :: 0xEA
recOpQueryCastShape :: 0xEB
recOpQueryCastRayClosest :: 0xEC
recOpQueryCastMover :: 0xED
recOpQueryCollideMover :: 0xEE
recOpQueryTag :: 0xEF
recOpStateHash :: 0xF1
recOpRecordingBounds :: 0xF2

// Codegen pass 1a: arg structs. Generated here so call sites (body.c, shape.c, etc.) can see them.

// Recordings are seeded with a world snapshot, not a CreateWorld op. DestroyWorld stays as the
// end-of-session marker the viewer reads.
RecArgs_DestroyWorld :: struct {
	world: RecCType_WORLDID,
}

RecArgs_Step :: struct {
	world:        RecCType_WORLDID,
	dt:           RecCType_F32,
	subStepCount: RecCType_I32,
}

// World config. The world arg is informational; replay always targets its own world.
RecArgs_WorldEnableSleeping :: struct {
	world: RecCType_WORLDID,
	flag:  RecCType_BOOL,
}

RecArgs_WorldEnableContinuous :: struct {
	world: RecCType_WORLDID,
	flag:  RecCType_BOOL,
}

RecArgs_WorldSetRestitutionThreshold :: struct {
	world: RecCType_WORLDID,
	value: RecCType_F32,
}

RecArgs_WorldSetHitEventThreshold :: struct {
	world: RecCType_WORLDID,
	value: RecCType_F32,
}

RecArgs_WorldSetGravity :: struct {
	world:   RecCType_WORLDID,
	gravity: RecCType_VEC3,
}

RecArgs_WorldExplode :: struct {
	world: RecCType_WORLDID,
	def:   RecCType_EXPLOSIONDEF,
}

RecArgs_WorldSetContactTuning :: struct {
	world:        RecCType_WORLDID,
	hertz:        RecCType_F32,
	dampingRatio: RecCType_F32,
	contactSpeed: RecCType_F32,
}

RecArgs_WorldSetContactRecycleDistance :: struct {
	world:           RecCType_WORLDID,
	recycleDistance: RecCType_F32,
}

RecArgs_WorldSetMaximumLinearSpeed :: struct {
	world:              RecCType_WORLDID,
	maximumLinearSpeed: RecCType_F32,
}

RecArgs_WorldEnableWarmStarting :: struct {
	world: RecCType_WORLDID,
	flag:  RecCType_BOOL,
}

RecArgs_WorldRebuildStaticTree :: struct {
	world: RecCType_WORLDID,
}

RecArgs_WorldEnableSpeculative :: struct {
	world: RecCType_WORLDID,
	flag:  RecCType_BOOL,
}

RecArgs_WorldSetRestitutionIterations :: struct {
	world:      RecCType_WORLDID,
	iterations: RecCType_I32,
}

RecArgs_WorldEnableRestitutionPropagation :: struct {
	world: RecCType_WORLDID,
	flag:  RecCType_BOOL,
}

RecArgs_WorldSetIsotropicInertiaTolerance :: struct {
	world:     RecCType_WORLDID,
	tolerance: RecCType_F32,
}

// Body
RecArgs_CreateBody :: struct {
	world: RecCType_WORLDID,
	def:   RecCType_BODYDEF,
}

RecArgs_DestroyBody :: struct {
	body: RecCType_BODYID,
}

RecArgs_BodySetTransform :: struct {
	body:     RecCType_BODYID,
	position: RecCType_POSITION,
	rotation: RecCType_QUAT,
}

RecArgs_BodySetLinearVelocity :: struct {
	body: RecCType_BODYID,
	v:    RecCType_VEC3,
}

RecArgs_BodySetType :: struct {
	body: RecCType_BODYID,
	type: RecCType_I32,
}

RecArgs_BodySetName :: struct {
	body: RecCType_BODYID,
	name: RecCType_STR,
}

RecArgs_BodySetAngularVelocity :: struct {
	body: RecCType_BODYID,
	w:    RecCType_VEC3,
}

RecArgs_BodySetTargetTransform :: struct {
	body:     RecCType_BODYID,
	target:   RecCType_WORLDXF,
	timeStep: RecCType_F32,
	wake:     RecCType_BOOL,
}

RecArgs_BodyApplyForce :: struct {
	body:  RecCType_BODYID,
	force: RecCType_VEC3,
	point: RecCType_POSITION,
	wake:  RecCType_BOOL,
}

RecArgs_BodyApplyForceToCenter :: struct {
	body:  RecCType_BODYID,
	force: RecCType_VEC3,
	wake:  RecCType_BOOL,
}

RecArgs_BodyApplyTorque :: struct {
	body:   RecCType_BODYID,
	torque: RecCType_VEC3,
	wake:   RecCType_BOOL,
}

RecArgs_BodyApplyLinearImpulse :: struct {
	body:    RecCType_BODYID,
	impulse: RecCType_VEC3,
	point:   RecCType_POSITION,
	wake:    RecCType_BOOL,
}

RecArgs_BodyApplyLinearImpulseToCenter :: struct {
	body:    RecCType_BODYID,
	impulse: RecCType_VEC3,
	wake:    RecCType_BOOL,
}

RecArgs_BodyApplyAngularImpulse :: struct {
	body:    RecCType_BODYID,
	impulse: RecCType_VEC3,
	wake:    RecCType_BOOL,
}

RecArgs_BodySetMassData :: struct {
	body:     RecCType_BODYID,
	massData: RecCType_MASSDATA,
}

RecArgs_BodyApplyMassFromShapes :: struct {
	body: RecCType_BODYID,
}

RecArgs_BodySetLinearDamping :: struct {
	body:    RecCType_BODYID,
	damping: RecCType_F32,
}

RecArgs_BodySetAngularDamping :: struct {
	body:    RecCType_BODYID,
	damping: RecCType_F32,
}

RecArgs_BodySetGravityScale :: struct {
	body:  RecCType_BODYID,
	scale: RecCType_F32,
}

RecArgs_BodySetAwake :: struct {
	body:  RecCType_BODYID,
	awake: RecCType_BOOL,
}

RecArgs_BodyEnableSleep :: struct {
	body: RecCType_BODYID,
	flag: RecCType_BOOL,
}

RecArgs_BodySetSleepThreshold :: struct {
	body:      RecCType_BODYID,
	threshold: RecCType_F32,
}

RecArgs_BodyDisable :: struct {
	body: RecCType_BODYID,
}

RecArgs_BodyEnable :: struct {
	body: RecCType_BODYID,
}

RecArgs_BodySetMotionLocks :: struct {
	body:  RecCType_BODYID,
	locks: RecCType_LOCKS,
}

RecArgs_BodySetBullet :: struct {
	body: RecCType_BODYID,
	flag: RecCType_BOOL,
}

RecArgs_BodyEnableContactRecycling :: struct {
	body: RecCType_BODYID,
	flag: RecCType_BOOL,
}

RecArgs_BodyEnableHitEvents :: struct {
	body: RecCType_BODYID,
	flag: RecCType_BOOL,
}

RecArgs_BodyAllowFastRotation :: struct {
	body: RecCType_BODYID,
	flag: RecCType_BOOL,
}

RecArgs_BodySetSafetyFactor :: struct {
	body:  RecCType_BODYID,
	value: RecCType_F32,
}

// Shape create/destroy
RecArgs_CreateSphereShape :: struct {
	body:   RecCType_BODYID,
	def:    RecCType_SHAPEDEF,
	sphere: RecCType_SPHERE,
}

RecArgs_CreateCapsuleShape :: struct {
	body:    RecCType_BODYID,
	def:     RecCType_SHAPEDEF,
	capsule: RecCType_CAPSULE,
}

RecArgs_CreateHullShape :: struct {
	body:       RecCType_BODYID,
	def:        RecCType_SHAPEDEF,
	geometryId: RecCType_GEOMID,
}

RecArgs_CreateMeshShape :: struct {
	body:       RecCType_BODYID,
	def:        RecCType_SHAPEDEF,
	geometryId: RecCType_GEOMID,
	scale:      RecCType_VEC3,
}

RecArgs_CreateHeightFieldShape :: struct {
	body:       RecCType_BODYID,
	def:        RecCType_SHAPEDEF,
	geometryId: RecCType_GEOMID,
}

RecArgs_CreateCompoundShape :: struct {
	body:       RecCType_BODYID,
	def:        RecCType_SHAPEDEF,
	geometryId: RecCType_GEOMID,
}

RecArgs_DestroyShape :: struct {
	shape:          RecCType_SHAPEID,
	updateBodyMass: RecCType_BOOL,
}

// Shape mutators
RecArgs_ShapeSetDensity :: struct {
	shape:          RecCType_SHAPEID,
	density:        RecCType_F32,
	updateBodyMass: RecCType_BOOL,
}

RecArgs_ShapeSetFriction :: struct {
	shape:    RecCType_SHAPEID,
	friction: RecCType_F32,
}

RecArgs_ShapeSetRestitution :: struct {
	shape:       RecCType_SHAPEID,
	restitution: RecCType_F32,
}

RecArgs_ShapeSetSurfaceMaterial :: struct {
	shape:    RecCType_SHAPEID,
	material: RecCType_MATERIAL,
}

RecArgs_ShapeSetFilter :: struct {
	shape:          RecCType_SHAPEID,
	filter:         RecCType_FILTER,
	invokeContacts: RecCType_BOOL,
}

RecArgs_ShapeEnableSensorEvents :: struct {
	shape: RecCType_SHAPEID,
	flag:  RecCType_BOOL,
}

RecArgs_ShapeEnableContactEvents :: struct {
	shape: RecCType_SHAPEID,
	flag:  RecCType_BOOL,
}

RecArgs_ShapeEnablePreSolveEvents :: struct {
	shape: RecCType_SHAPEID,
	flag:  RecCType_BOOL,
}

RecArgs_ShapeEnableHitEvents :: struct {
	shape: RecCType_SHAPEID,
	flag:  RecCType_BOOL,
}

RecArgs_ShapeSetSphere :: struct {
	shape:  RecCType_SHAPEID,
	sphere: RecCType_SPHERE,
}

RecArgs_ShapeSetCapsule :: struct {
	shape:   RecCType_SHAPEID,
	capsule: RecCType_CAPSULE,
}

RecArgs_ShapeApplyWind :: struct {
	shape:    RecCType_SHAPEID,
	wind:     RecCType_VEC3,
	drag:     RecCType_F32,
	lift:     RecCType_F32,
	maxSpeed: RecCType_F32,
	wake:     RecCType_BOOL,
}

RecArgs_ShapeSetName :: struct {
	shape: RecCType_SHAPEID,
	name:  RecCType_STR,
}

RecArgs_ShapeSetMeshMaterial :: struct {
	shape:    RecCType_SHAPEID,
	material: RecCType_MATERIAL,
	index:    RecCType_I32,
}

RecArgs_ShapeSetHull :: struct {
	shape:      RecCType_SHAPEID,
	geometryId: RecCType_GEOMID,
}

RecArgs_ShapeSetMesh :: struct {
	shape:      RecCType_SHAPEID,
	geometryId: RecCType_GEOMID,
	scale:      RecCType_VEC3,
}

// Joint create and destroy
RecArgs_CreateParallelJoint :: struct {
	world: RecCType_WORLDID,
	def:   RecCType_PARALLELJOINTDEF,
}

RecArgs_CreateDistanceJoint :: struct {
	world: RecCType_WORLDID,
	def:   RecCType_DISTANCEJOINTDEF,
}

RecArgs_CreateFilterJoint :: struct {
	world: RecCType_WORLDID,
	def:   RecCType_FILTERJOINTDEF,
}

RecArgs_CreateMotorJoint :: struct {
	world: RecCType_WORLDID,
	def:   RecCType_MOTORJOINTDEF,
}

RecArgs_CreatePrismaticJoint :: struct {
	world: RecCType_WORLDID,
	def:   RecCType_PRISMATICJOINTDEF,
}

RecArgs_CreateRevoluteJoint :: struct {
	world: RecCType_WORLDID,
	def:   RecCType_REVOLUTEJOINTDEF,
}

RecArgs_CreateSphericalJoint :: struct {
	world: RecCType_WORLDID,
	def:   RecCType_SPHERICALJOINTDEF,
}

RecArgs_CreateWeldJoint :: struct {
	world: RecCType_WORLDID,
	def:   RecCType_WELDJOINTDEF,
}

RecArgs_CreateWheelJoint :: struct {
	world: RecCType_WORLDID,
	def:   RecCType_WHEELJOINTDEF,
}

RecArgs_DestroyJoint :: struct {
	joint:        RecCType_JOINTID,
	wakeAttached: RecCType_BOOL,
}

// Generic joint mutators
RecArgs_JointSetLocalFrameA :: struct {
	joint:      RecCType_JOINTID,
	localFrame: RecCType_TRANSFORM,
}

RecArgs_JointSetLocalFrameB :: struct {
	joint:      RecCType_JOINTID,
	localFrame: RecCType_TRANSFORM,
}

RecArgs_JointSetCollideConnected :: struct {
	joint:         RecCType_JOINTID,
	shouldCollide: RecCType_BOOL,
}

RecArgs_JointWakeBodies :: struct {
	joint: RecCType_JOINTID,
}

RecArgs_JointSetConstraintTuning :: struct {
	joint:        RecCType_JOINTID,
	hertz:        RecCType_F32,
	dampingRatio: RecCType_F32,
}

RecArgs_JointSetForceThreshold :: struct {
	joint:     RecCType_JOINTID,
	threshold: RecCType_F32,
}

RecArgs_JointSetTorqueThreshold :: struct {
	joint:     RecCType_JOINTID,
	threshold: RecCType_F32,
}

// Parallel joint
RecArgs_ParallelJointSetSpringHertz :: struct {
	joint: RecCType_JOINTID,
	hertz: RecCType_F32,
}

RecArgs_ParallelJointSetSpringDampingRatio :: struct {
	joint:        RecCType_JOINTID,
	dampingRatio: RecCType_F32,
}

RecArgs_ParallelJointSetMaxTorque :: struct {
	joint:     RecCType_JOINTID,
	maxTorque: RecCType_F32,
}

// Distance joint
RecArgs_DistanceJointSetLength :: struct {
	joint:  RecCType_JOINTID,
	length: RecCType_F32,
}

RecArgs_DistanceJointEnableSpring :: struct {
	joint:        RecCType_JOINTID,
	enableSpring: RecCType_BOOL,
}

RecArgs_DistanceJointSetSpringForceRange :: struct {
	joint:      RecCType_JOINTID,
	lowerForce: RecCType_F32,
	upperForce: RecCType_F32,
}

RecArgs_DistanceJointSetSpringHertz :: struct {
	joint: RecCType_JOINTID,
	hertz: RecCType_F32,
}

RecArgs_DistanceJointSetSpringDampingRatio :: struct {
	joint:        RecCType_JOINTID,
	dampingRatio: RecCType_F32,
}

RecArgs_DistanceJointEnableLimit :: struct {
	joint:       RecCType_JOINTID,
	enableLimit: RecCType_BOOL,
}

RecArgs_DistanceJointSetLengthRange :: struct {
	joint:     RecCType_JOINTID,
	minLength: RecCType_F32,
	maxLength: RecCType_F32,
}

RecArgs_DistanceJointEnableMotor :: struct {
	joint:       RecCType_JOINTID,
	enableMotor: RecCType_BOOL,
}

RecArgs_DistanceJointSetMotorSpeed :: struct {
	joint:      RecCType_JOINTID,
	motorSpeed: RecCType_F32,
}

RecArgs_DistanceJointSetMaxMotorForce :: struct {
	joint: RecCType_JOINTID,
	force: RecCType_F32,
}

// Motor joint
RecArgs_MotorJointSetLinearVelocity :: struct {
	joint:    RecCType_JOINTID,
	velocity: RecCType_VEC3,
}

RecArgs_MotorJointSetAngularVelocity :: struct {
	joint:    RecCType_JOINTID,
	velocity: RecCType_VEC3,
}

RecArgs_MotorJointSetMaxVelocityForce :: struct {
	joint:    RecCType_JOINTID,
	maxForce: RecCType_F32,
}

RecArgs_MotorJointSetMaxVelocityTorque :: struct {
	joint:     RecCType_JOINTID,
	maxTorque: RecCType_F32,
}

RecArgs_MotorJointSetLinearHertz :: struct {
	joint: RecCType_JOINTID,
	hertz: RecCType_F32,
}

RecArgs_MotorJointSetLinearDampingRatio :: struct {
	joint:   RecCType_JOINTID,
	damping: RecCType_F32,
}

RecArgs_MotorJointSetAngularHertz :: struct {
	joint: RecCType_JOINTID,
	hertz: RecCType_F32,
}

RecArgs_MotorJointSetAngularDampingRatio :: struct {
	joint:   RecCType_JOINTID,
	damping: RecCType_F32,
}

RecArgs_MotorJointSetMaxSpringForce :: struct {
	joint:    RecCType_JOINTID,
	maxForce: RecCType_F32,
}

RecArgs_MotorJointSetMaxSpringTorque :: struct {
	joint:     RecCType_JOINTID,
	maxTorque: RecCType_F32,
}

// Prismatic joint
RecArgs_PrismaticJointEnableSpring :: struct {
	joint:        RecCType_JOINTID,
	enableSpring: RecCType_BOOL,
}

RecArgs_PrismaticJointSetSpringHertz :: struct {
	joint: RecCType_JOINTID,
	hertz: RecCType_F32,
}

RecArgs_PrismaticJointSetSpringDampingRatio :: struct {
	joint:        RecCType_JOINTID,
	dampingRatio: RecCType_F32,
}

RecArgs_PrismaticJointSetTargetTranslation :: struct {
	joint:       RecCType_JOINTID,
	translation: RecCType_F32,
}

RecArgs_PrismaticJointEnableLimit :: struct {
	joint:       RecCType_JOINTID,
	enableLimit: RecCType_BOOL,
}

RecArgs_PrismaticJointSetLimits :: struct {
	joint: RecCType_JOINTID,
	lower: RecCType_F32,
	upper: RecCType_F32,
}

RecArgs_PrismaticJointEnableMotor :: struct {
	joint:       RecCType_JOINTID,
	enableMotor: RecCType_BOOL,
}

RecArgs_PrismaticJointSetMotorSpeed :: struct {
	joint:      RecCType_JOINTID,
	motorSpeed: RecCType_F32,
}

RecArgs_PrismaticJointSetMaxMotorForce :: struct {
	joint: RecCType_JOINTID,
	force: RecCType_F32,
}

// Revolute joint
RecArgs_RevoluteJointEnableSpring :: struct {
	joint:        RecCType_JOINTID,
	enableSpring: RecCType_BOOL,
}

RecArgs_RevoluteJointSetSpringHertz :: struct {
	joint: RecCType_JOINTID,
	hertz: RecCType_F32,
}

RecArgs_RevoluteJointSetSpringDampingRatio :: struct {
	joint:        RecCType_JOINTID,
	dampingRatio: RecCType_F32,
}

RecArgs_RevoluteJointSetTargetAngle :: struct {
	joint: RecCType_JOINTID,
	angle: RecCType_F32,
}

RecArgs_RevoluteJointEnableLimit :: struct {
	joint:       RecCType_JOINTID,
	enableLimit: RecCType_BOOL,
}

RecArgs_RevoluteJointSetLimits :: struct {
	joint: RecCType_JOINTID,
	lower: RecCType_F32,
	upper: RecCType_F32,
}

RecArgs_RevoluteJointEnableMotor :: struct {
	joint:       RecCType_JOINTID,
	enableMotor: RecCType_BOOL,
}

RecArgs_RevoluteJointSetMotorSpeed :: struct {
	joint:      RecCType_JOINTID,
	motorSpeed: RecCType_F32,
}

RecArgs_RevoluteJointSetMaxMotorTorque :: struct {
	joint:  RecCType_JOINTID,
	torque: RecCType_F32,
}

// Spherical joint
RecArgs_SphericalJointEnableConeLimit :: struct {
	joint:       RecCType_JOINTID,
	enableLimit: RecCType_BOOL,
}

RecArgs_SphericalJointSetConeLimit :: struct {
	joint:        RecCType_JOINTID,
	angleRadians: RecCType_F32,
}

RecArgs_SphericalJointEnableTwistLimit :: struct {
	joint:       RecCType_JOINTID,
	enableLimit: RecCType_BOOL,
}

RecArgs_SphericalJointSetTwistLimits :: struct {
	joint: RecCType_JOINTID,
	lower: RecCType_F32,
	upper: RecCType_F32,
}

RecArgs_SphericalJointEnableSpring :: struct {
	joint:        RecCType_JOINTID,
	enableSpring: RecCType_BOOL,
}

RecArgs_SphericalJointSetSpringHertz :: struct {
	joint: RecCType_JOINTID,
	hertz: RecCType_F32,
}

RecArgs_SphericalJointSetSpringDampingRatio :: struct {
	joint:        RecCType_JOINTID,
	dampingRatio: RecCType_F32,
}

RecArgs_SphericalJointSetTargetRotation :: struct {
	joint:          RecCType_JOINTID,
	targetRotation: RecCType_QUAT,
}

RecArgs_SphericalJointEnableMotor :: struct {
	joint:       RecCType_JOINTID,
	enableMotor: RecCType_BOOL,
}

RecArgs_SphericalJointSetMotorVelocity :: struct {
	joint:         RecCType_JOINTID,
	motorVelocity: RecCType_VEC3,
}

RecArgs_SphericalJointSetMaxMotorTorque :: struct {
	joint:  RecCType_JOINTID,
	torque: RecCType_F32,
}

// Weld joint
RecArgs_WeldJointSetLinearHertz :: struct {
	joint: RecCType_JOINTID,
	hertz: RecCType_F32,
}

RecArgs_WeldJointSetLinearDampingRatio :: struct {
	joint:        RecCType_JOINTID,
	dampingRatio: RecCType_F32,
}

RecArgs_WeldJointSetAngularHertz :: struct {
	joint: RecCType_JOINTID,
	hertz: RecCType_F32,
}

RecArgs_WeldJointSetAngularDampingRatio :: struct {
	joint:        RecCType_JOINTID,
	dampingRatio: RecCType_F32,
}

// Wheel joint
RecArgs_WheelJointEnableSuspension :: struct {
	joint: RecCType_JOINTID,
	flag:  RecCType_BOOL,
}

RecArgs_WheelJointSetSuspensionHertz :: struct {
	joint: RecCType_JOINTID,
	hertz: RecCType_F32,
}

RecArgs_WheelJointSetSuspensionDampingRatio :: struct {
	joint:        RecCType_JOINTID,
	dampingRatio: RecCType_F32,
}

RecArgs_WheelJointEnableSuspensionLimit :: struct {
	joint: RecCType_JOINTID,
	flag:  RecCType_BOOL,
}

RecArgs_WheelJointSetSuspensionLimits :: struct {
	joint: RecCType_JOINTID,
	lower: RecCType_F32,
	upper: RecCType_F32,
}

RecArgs_WheelJointEnableSpinMotor :: struct {
	joint: RecCType_JOINTID,
	flag:  RecCType_BOOL,
}

RecArgs_WheelJointSetSpinMotorSpeed :: struct {
	joint: RecCType_JOINTID,
	speed: RecCType_F32,
}

// Wheel joint continued, overflow past the 0xDF range.
RecArgs_WheelJointSetMaxSpinTorque :: struct {
	joint:  RecCType_JOINTID,
	torque: RecCType_F32,
}

RecArgs_WheelJointEnableSteering :: struct {
	joint: RecCType_JOINTID,
	flag:  RecCType_BOOL,
}

RecArgs_WheelJointSetSteeringHertz :: struct {
	joint: RecCType_JOINTID,
	hertz: RecCType_F32,
}

RecArgs_WheelJointSetSteeringDampingRatio :: struct {
	joint:        RecCType_JOINTID,
	dampingRatio: RecCType_F32,
}

RecArgs_WheelJointSetMaxSteeringTorque :: struct {
	joint:  RecCType_JOINTID,
	torque: RecCType_F32,
}

RecArgs_WheelJointEnableSteeringLimit :: struct {
	joint: RecCType_JOINTID,
	flag:  RecCType_BOOL,
}

RecArgs_WheelJointSetSteeringLimits :: struct {
	joint: RecCType_JOINTID,
	lower: RecCType_F32,
	upper: RecCType_F32,
}

RecArgs_WheelJointSetTargetSteeringAngle :: struct {
	joint:   RecCType_JOINTID,
	radians: RecCType_F32,
}

// Mover joint
RecArgs_CreateMoverJoint :: struct {
	world: RecCType_WORLDID,
	def:   RecCType_MOVERJOINTDEF,
}

RecArgs_MoverJointSetLinearVelocity :: struct {
	joint:    RecCType_JOINTID,
	velocity: RecCType_VEC3,
}

RecArgs_MoverJointSetMaxVelocityForce :: struct {
	joint:    RecCType_JOINTID,
	maxForce: RecCType_F32,
}

// Pogo joint
RecArgs_CreatePogoJoint :: struct {
	world: RecCType_WORLDID,
	def:   RecCType_POGOJOINTDEF,
}

RecArgs_PogoJointSetRestLength :: struct {
	joint:  RecCType_JOINTID,
	length: RecCType_F32,
}

RecArgs_PogoJointSetSpringHertz :: struct {
	joint: RecCType_JOINTID,
	hertz: RecCType_F32,
}

RecArgs_PogoJointSetSpringDampingRatio :: struct {
	joint:        RecCType_JOINTID,
	dampingRatio: RecCType_F32,
}

RecArgs_PogoJointSetMaxTensionForce :: struct {
	joint: RecCType_JOINTID,
	force: RecCType_F32,
}

RecArgs_PogoJointSetMaxCompressionForce :: struct {
	joint: RecCType_JOINTID,
	force: RecCType_F32,
}

RecArgs_PogoJointSetNormal :: struct {
	joint:  RecCType_JOINTID,
	normal: RecCType_VEC3,
}

// Spatial queries. Inputs flow through the manifest (reader side). The hit tail and result are
// hand-written in recording.c / recording_replay.c since they are variable length.
RecArgs_QueryOverlapAABB :: struct {
	world:  RecCType_WORLDID,
	aabb:   RecCType_AABB,
	filter: RecCType_QUERYFILTER,
}

RecArgs_QueryOverlapShape :: struct {
	world:  RecCType_WORLDID,
	origin: RecCType_POSITION,
	proxy:  RecCType_SHAPEPROXY,
	filter: RecCType_QUERYFILTER,
}

RecArgs_QueryCastRay :: struct {
	world:       RecCType_WORLDID,
	origin:      RecCType_POSITION,
	translation: RecCType_VEC3,
	filter:      RecCType_QUERYFILTER,
}

RecArgs_QueryCastShape :: struct {
	world:       RecCType_WORLDID,
	origin:      RecCType_POSITION,
	proxy:       RecCType_SHAPEPROXY,
	translation: RecCType_VEC3,
	filter:      RecCType_QUERYFILTER,
}

RecArgs_QueryCastRayClosest :: struct {
	world:       RecCType_WORLDID,
	origin:      RecCType_POSITION,
	translation: RecCType_VEC3,
	filter:      RecCType_QUERYFILTER,
}

RecArgs_QueryCastMover :: struct {
	world:       RecCType_WORLDID,
	origin:      RecCType_POSITION,
	mover:       RecCType_CAPSULE,
	translation: RecCType_VEC3,
	filter:      RecCType_QUERYFILTER,
}

RecArgs_QueryCollideMover :: struct {
	world:  RecCType_WORLDID,
	origin: RecCType_POSITION,
	mover:  RecCType_CAPSULE,
	filter: RecCType_QUERYFILTER,
}

// Identity key (hash of the caller id + label) for the query that immediately follows. Emitted only
// for a tagged query. The id and label are interned in the trailing tag table, so only the 8 byte key
// rides the stream.
RecArgs_QueryTag :: struct {
	key: RecCType_U64,
}

RecArgs_StateHash :: struct {
	world: RecCType_WORLDID,
	hash:  RecCType_U64,
}

// Accumulated world bounds over the whole recording, written once at stop.
RecArgs_RecordingBounds :: struct {
	bounds: RecCType_AABB,
}

// Codegen pass 1b: arg writers

RecWriteArgs_DestroyWorld :: proc(rec: ^Recording, a: ^RecArgs_DestroyWorld) {
	RecW_WORLDID(&rec.buffer, a.world)
}

RecWriteArgs_Step :: proc(rec: ^Recording, a: ^RecArgs_Step) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_F32(&rec.buffer, a.dt)
	RecW_I32(&rec.buffer, a.subStepCount)
}

RecWriteArgs_WorldEnableSleeping :: proc(rec: ^Recording, a: ^RecArgs_WorldEnableSleeping) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_WorldEnableContinuous :: proc(rec: ^Recording, a: ^RecArgs_WorldEnableContinuous) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_WorldSetRestitutionThreshold :: proc(rec: ^Recording, a: ^RecArgs_WorldSetRestitutionThreshold) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_F32(&rec.buffer, a.value)
}

RecWriteArgs_WorldSetHitEventThreshold :: proc(rec: ^Recording, a: ^RecArgs_WorldSetHitEventThreshold) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_F32(&rec.buffer, a.value)
}

RecWriteArgs_WorldSetGravity :: proc(rec: ^Recording, a: ^RecArgs_WorldSetGravity) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_VEC3(&rec.buffer, a.gravity)
}

RecWriteArgs_WorldExplode :: proc(rec: ^Recording, a: ^RecArgs_WorldExplode) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_EXPLOSIONDEF(&rec.buffer, a.def)
}

RecWriteArgs_WorldSetContactTuning :: proc(rec: ^Recording, a: ^RecArgs_WorldSetContactTuning) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_F32(&rec.buffer, a.hertz)
	RecW_F32(&rec.buffer, a.dampingRatio)
	RecW_F32(&rec.buffer, a.contactSpeed)
}

RecWriteArgs_WorldSetContactRecycleDistance :: proc(rec: ^Recording, a: ^RecArgs_WorldSetContactRecycleDistance) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_F32(&rec.buffer, a.recycleDistance)
}

RecWriteArgs_WorldSetMaximumLinearSpeed :: proc(rec: ^Recording, a: ^RecArgs_WorldSetMaximumLinearSpeed) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_F32(&rec.buffer, a.maximumLinearSpeed)
}

RecWriteArgs_WorldEnableWarmStarting :: proc(rec: ^Recording, a: ^RecArgs_WorldEnableWarmStarting) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_WorldRebuildStaticTree :: proc(rec: ^Recording, a: ^RecArgs_WorldRebuildStaticTree) {
	RecW_WORLDID(&rec.buffer, a.world)
}

RecWriteArgs_WorldEnableSpeculative :: proc(rec: ^Recording, a: ^RecArgs_WorldEnableSpeculative) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_WorldSetRestitutionIterations :: proc(rec: ^Recording, a: ^RecArgs_WorldSetRestitutionIterations) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_I32(&rec.buffer, a.iterations)
}

RecWriteArgs_WorldEnableRestitutionPropagation :: proc(
	rec: ^Recording,
	a: ^RecArgs_WorldEnableRestitutionPropagation,
) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_WorldSetIsotropicInertiaTolerance :: proc(
	rec: ^Recording,
	a: ^RecArgs_WorldSetIsotropicInertiaTolerance,
) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_F32(&rec.buffer, a.tolerance)
}

RecWriteArgs_CreateBody :: proc(rec: ^Recording, a: ^RecArgs_CreateBody) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_BODYDEF(&rec.buffer, a.def)
}

RecWriteArgs_DestroyBody :: proc(rec: ^Recording, a: ^RecArgs_DestroyBody) {
	RecW_BODYID(&rec.buffer, a.body)
}

RecWriteArgs_BodySetTransform :: proc(rec: ^Recording, a: ^RecArgs_BodySetTransform) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_POSITION(&rec.buffer, a.position)
	RecW_QUAT(&rec.buffer, a.rotation)
}

RecWriteArgs_BodySetLinearVelocity :: proc(rec: ^Recording, a: ^RecArgs_BodySetLinearVelocity) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_VEC3(&rec.buffer, a.v)
}

RecWriteArgs_BodySetType :: proc(rec: ^Recording, a: ^RecArgs_BodySetType) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_I32(&rec.buffer, a.type)
}

RecWriteArgs_BodySetName :: proc(rec: ^Recording, a: ^RecArgs_BodySetName) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_STR(&rec.buffer, a.name)
}

RecWriteArgs_BodySetAngularVelocity :: proc(rec: ^Recording, a: ^RecArgs_BodySetAngularVelocity) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_VEC3(&rec.buffer, a.w)
}

RecWriteArgs_BodySetTargetTransform :: proc(rec: ^Recording, a: ^RecArgs_BodySetTargetTransform) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_WORLDXF(&rec.buffer, a.target)
	RecW_F32(&rec.buffer, a.timeStep)
	RecW_BOOL(&rec.buffer, a.wake)
}

RecWriteArgs_BodyApplyForce :: proc(rec: ^Recording, a: ^RecArgs_BodyApplyForce) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_VEC3(&rec.buffer, a.force)
	RecW_POSITION(&rec.buffer, a.point)
	RecW_BOOL(&rec.buffer, a.wake)
}

RecWriteArgs_BodyApplyForceToCenter :: proc(rec: ^Recording, a: ^RecArgs_BodyApplyForceToCenter) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_VEC3(&rec.buffer, a.force)
	RecW_BOOL(&rec.buffer, a.wake)
}

RecWriteArgs_BodyApplyTorque :: proc(rec: ^Recording, a: ^RecArgs_BodyApplyTorque) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_VEC3(&rec.buffer, a.torque)
	RecW_BOOL(&rec.buffer, a.wake)
}

RecWriteArgs_BodyApplyLinearImpulse :: proc(rec: ^Recording, a: ^RecArgs_BodyApplyLinearImpulse) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_VEC3(&rec.buffer, a.impulse)
	RecW_POSITION(&rec.buffer, a.point)
	RecW_BOOL(&rec.buffer, a.wake)
}

RecWriteArgs_BodyApplyLinearImpulseToCenter :: proc(rec: ^Recording, a: ^RecArgs_BodyApplyLinearImpulseToCenter) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_VEC3(&rec.buffer, a.impulse)
	RecW_BOOL(&rec.buffer, a.wake)
}

RecWriteArgs_BodyApplyAngularImpulse :: proc(rec: ^Recording, a: ^RecArgs_BodyApplyAngularImpulse) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_VEC3(&rec.buffer, a.impulse)
	RecW_BOOL(&rec.buffer, a.wake)
}

RecWriteArgs_BodySetMassData :: proc(rec: ^Recording, a: ^RecArgs_BodySetMassData) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_MASSDATA(&rec.buffer, a.massData)
}

RecWriteArgs_BodyApplyMassFromShapes :: proc(rec: ^Recording, a: ^RecArgs_BodyApplyMassFromShapes) {
	RecW_BODYID(&rec.buffer, a.body)
}

RecWriteArgs_BodySetLinearDamping :: proc(rec: ^Recording, a: ^RecArgs_BodySetLinearDamping) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_F32(&rec.buffer, a.damping)
}

RecWriteArgs_BodySetAngularDamping :: proc(rec: ^Recording, a: ^RecArgs_BodySetAngularDamping) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_F32(&rec.buffer, a.damping)
}

RecWriteArgs_BodySetGravityScale :: proc(rec: ^Recording, a: ^RecArgs_BodySetGravityScale) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_F32(&rec.buffer, a.scale)
}

RecWriteArgs_BodySetAwake :: proc(rec: ^Recording, a: ^RecArgs_BodySetAwake) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_BOOL(&rec.buffer, a.awake)
}

RecWriteArgs_BodyEnableSleep :: proc(rec: ^Recording, a: ^RecArgs_BodyEnableSleep) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_BodySetSleepThreshold :: proc(rec: ^Recording, a: ^RecArgs_BodySetSleepThreshold) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_F32(&rec.buffer, a.threshold)
}

RecWriteArgs_BodyDisable :: proc(rec: ^Recording, a: ^RecArgs_BodyDisable) {
	RecW_BODYID(&rec.buffer, a.body)
}

RecWriteArgs_BodyEnable :: proc(rec: ^Recording, a: ^RecArgs_BodyEnable) {
	RecW_BODYID(&rec.buffer, a.body)
}

RecWriteArgs_BodySetMotionLocks :: proc(rec: ^Recording, a: ^RecArgs_BodySetMotionLocks) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_LOCKS(&rec.buffer, a.locks)
}

RecWriteArgs_BodySetBullet :: proc(rec: ^Recording, a: ^RecArgs_BodySetBullet) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_BodyEnableContactRecycling :: proc(rec: ^Recording, a: ^RecArgs_BodyEnableContactRecycling) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_BodyEnableHitEvents :: proc(rec: ^Recording, a: ^RecArgs_BodyEnableHitEvents) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_BodyAllowFastRotation :: proc(rec: ^Recording, a: ^RecArgs_BodyAllowFastRotation) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_BodySetSafetyFactor :: proc(rec: ^Recording, a: ^RecArgs_BodySetSafetyFactor) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_F32(&rec.buffer, a.value)
}

RecWriteArgs_CreateSphereShape :: proc(rec: ^Recording, a: ^RecArgs_CreateSphereShape) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_SHAPEDEF(&rec.buffer, a.def)
	RecW_SPHERE(&rec.buffer, a.sphere)
}

RecWriteArgs_CreateCapsuleShape :: proc(rec: ^Recording, a: ^RecArgs_CreateCapsuleShape) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_SHAPEDEF(&rec.buffer, a.def)
	RecW_CAPSULE(&rec.buffer, a.capsule)
}

RecWriteArgs_CreateHullShape :: proc(rec: ^Recording, a: ^RecArgs_CreateHullShape) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_SHAPEDEF(&rec.buffer, a.def)
	RecW_GEOMID(&rec.buffer, a.geometryId)
}

RecWriteArgs_CreateMeshShape :: proc(rec: ^Recording, a: ^RecArgs_CreateMeshShape) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_SHAPEDEF(&rec.buffer, a.def)
	RecW_GEOMID(&rec.buffer, a.geometryId)
	RecW_VEC3(&rec.buffer, a.scale)
}

RecWriteArgs_CreateHeightFieldShape :: proc(rec: ^Recording, a: ^RecArgs_CreateHeightFieldShape) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_SHAPEDEF(&rec.buffer, a.def)
	RecW_GEOMID(&rec.buffer, a.geometryId)
}

RecWriteArgs_CreateCompoundShape :: proc(rec: ^Recording, a: ^RecArgs_CreateCompoundShape) {
	RecW_BODYID(&rec.buffer, a.body)
	RecW_SHAPEDEF(&rec.buffer, a.def)
	RecW_GEOMID(&rec.buffer, a.geometryId)
}

RecWriteArgs_DestroyShape :: proc(rec: ^Recording, a: ^RecArgs_DestroyShape) {
	RecW_SHAPEID(&rec.buffer, a.shape)
	RecW_BOOL(&rec.buffer, a.updateBodyMass)
}

RecWriteArgs_ShapeSetDensity :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetDensity) {
	RecW_SHAPEID(&rec.buffer, a.shape)
	RecW_F32(&rec.buffer, a.density)
	RecW_BOOL(&rec.buffer, a.updateBodyMass)
}

RecWriteArgs_ShapeSetFriction :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetFriction) {
	RecW_SHAPEID(&rec.buffer, a.shape)
	RecW_F32(&rec.buffer, a.friction)
}

RecWriteArgs_ShapeSetRestitution :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetRestitution) {
	RecW_SHAPEID(&rec.buffer, a.shape)
	RecW_F32(&rec.buffer, a.restitution)
}

RecWriteArgs_ShapeSetSurfaceMaterial :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetSurfaceMaterial) {
	RecW_SHAPEID(&rec.buffer, a.shape)
	RecW_MATERIAL(&rec.buffer, a.material)
}

RecWriteArgs_ShapeSetFilter :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetFilter) {
	RecW_SHAPEID(&rec.buffer, a.shape)
	RecW_FILTER(&rec.buffer, a.filter)
	RecW_BOOL(&rec.buffer, a.invokeContacts)
}

RecWriteArgs_ShapeEnableSensorEvents :: proc(rec: ^Recording, a: ^RecArgs_ShapeEnableSensorEvents) {
	RecW_SHAPEID(&rec.buffer, a.shape)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_ShapeEnableContactEvents :: proc(rec: ^Recording, a: ^RecArgs_ShapeEnableContactEvents) {
	RecW_SHAPEID(&rec.buffer, a.shape)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_ShapeEnablePreSolveEvents :: proc(rec: ^Recording, a: ^RecArgs_ShapeEnablePreSolveEvents) {
	RecW_SHAPEID(&rec.buffer, a.shape)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_ShapeEnableHitEvents :: proc(rec: ^Recording, a: ^RecArgs_ShapeEnableHitEvents) {
	RecW_SHAPEID(&rec.buffer, a.shape)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_ShapeSetSphere :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetSphere) {
	RecW_SHAPEID(&rec.buffer, a.shape)
	RecW_SPHERE(&rec.buffer, a.sphere)
}

RecWriteArgs_ShapeSetCapsule :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetCapsule) {
	RecW_SHAPEID(&rec.buffer, a.shape)
	RecW_CAPSULE(&rec.buffer, a.capsule)
}

RecWriteArgs_ShapeApplyWind :: proc(rec: ^Recording, a: ^RecArgs_ShapeApplyWind) {
	RecW_SHAPEID(&rec.buffer, a.shape)
	RecW_VEC3(&rec.buffer, a.wind)
	RecW_F32(&rec.buffer, a.drag)
	RecW_F32(&rec.buffer, a.lift)
	RecW_F32(&rec.buffer, a.maxSpeed)
	RecW_BOOL(&rec.buffer, a.wake)
}

RecWriteArgs_ShapeSetName :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetName) {
	RecW_SHAPEID(&rec.buffer, a.shape)
	RecW_STR(&rec.buffer, a.name)
}

RecWriteArgs_ShapeSetMeshMaterial :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetMeshMaterial) {
	RecW_SHAPEID(&rec.buffer, a.shape)
	RecW_MATERIAL(&rec.buffer, a.material)
	RecW_I32(&rec.buffer, a.index)
}

RecWriteArgs_ShapeSetHull :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetHull) {
	RecW_SHAPEID(&rec.buffer, a.shape)
	RecW_GEOMID(&rec.buffer, a.geometryId)
}

RecWriteArgs_ShapeSetMesh :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetMesh) {
	RecW_SHAPEID(&rec.buffer, a.shape)
	RecW_GEOMID(&rec.buffer, a.geometryId)
	RecW_VEC3(&rec.buffer, a.scale)
}

RecWriteArgs_CreateParallelJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateParallelJoint) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_PARALLELJOINTDEF(&rec.buffer, a.def)
}

RecWriteArgs_CreateDistanceJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateDistanceJoint) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_DISTANCEJOINTDEF(&rec.buffer, a.def)
}

RecWriteArgs_CreateFilterJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateFilterJoint) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_FILTERJOINTDEF(&rec.buffer, a.def)
}

RecWriteArgs_CreateMotorJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateMotorJoint) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_MOTORJOINTDEF(&rec.buffer, a.def)
}

RecWriteArgs_CreatePrismaticJoint :: proc(rec: ^Recording, a: ^RecArgs_CreatePrismaticJoint) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_PRISMATICJOINTDEF(&rec.buffer, a.def)
}

RecWriteArgs_CreateRevoluteJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateRevoluteJoint) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_REVOLUTEJOINTDEF(&rec.buffer, a.def)
}

RecWriteArgs_CreateSphericalJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateSphericalJoint) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_SPHERICALJOINTDEF(&rec.buffer, a.def)
}

RecWriteArgs_CreateWeldJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateWeldJoint) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_WELDJOINTDEF(&rec.buffer, a.def)
}

RecWriteArgs_CreateWheelJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateWheelJoint) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_WHEELJOINTDEF(&rec.buffer, a.def)
}

RecWriteArgs_DestroyJoint :: proc(rec: ^Recording, a: ^RecArgs_DestroyJoint) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.wakeAttached)
}

RecWriteArgs_JointSetLocalFrameA :: proc(rec: ^Recording, a: ^RecArgs_JointSetLocalFrameA) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_TRANSFORM(&rec.buffer, a.localFrame)
}

RecWriteArgs_JointSetLocalFrameB :: proc(rec: ^Recording, a: ^RecArgs_JointSetLocalFrameB) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_TRANSFORM(&rec.buffer, a.localFrame)
}

RecWriteArgs_JointSetCollideConnected :: proc(rec: ^Recording, a: ^RecArgs_JointSetCollideConnected) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.shouldCollide)
}

RecWriteArgs_JointWakeBodies :: proc(rec: ^Recording, a: ^RecArgs_JointWakeBodies) {
	RecW_JOINTID(&rec.buffer, a.joint)
}

RecWriteArgs_JointSetConstraintTuning :: proc(rec: ^Recording, a: ^RecArgs_JointSetConstraintTuning) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.hertz)
	RecW_F32(&rec.buffer, a.dampingRatio)
}

RecWriteArgs_JointSetForceThreshold :: proc(rec: ^Recording, a: ^RecArgs_JointSetForceThreshold) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.threshold)
}

RecWriteArgs_JointSetTorqueThreshold :: proc(rec: ^Recording, a: ^RecArgs_JointSetTorqueThreshold) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.threshold)
}

RecWriteArgs_ParallelJointSetSpringHertz :: proc(rec: ^Recording, a: ^RecArgs_ParallelJointSetSpringHertz) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.hertz)
}

RecWriteArgs_ParallelJointSetSpringDampingRatio :: proc(
	rec: ^Recording,
	a: ^RecArgs_ParallelJointSetSpringDampingRatio,
) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.dampingRatio)
}

RecWriteArgs_ParallelJointSetMaxTorque :: proc(rec: ^Recording, a: ^RecArgs_ParallelJointSetMaxTorque) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.maxTorque)
}

RecWriteArgs_DistanceJointSetLength :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointSetLength) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.length)
}

RecWriteArgs_DistanceJointEnableSpring :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointEnableSpring) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.enableSpring)
}

RecWriteArgs_DistanceJointSetSpringForceRange :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointSetSpringForceRange) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.lowerForce)
	RecW_F32(&rec.buffer, a.upperForce)
}

RecWriteArgs_DistanceJointSetSpringHertz :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointSetSpringHertz) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.hertz)
}

RecWriteArgs_DistanceJointSetSpringDampingRatio :: proc(
	rec: ^Recording,
	a: ^RecArgs_DistanceJointSetSpringDampingRatio,
) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.dampingRatio)
}

RecWriteArgs_DistanceJointEnableLimit :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointEnableLimit) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.enableLimit)
}

RecWriteArgs_DistanceJointSetLengthRange :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointSetLengthRange) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.minLength)
	RecW_F32(&rec.buffer, a.maxLength)
}

RecWriteArgs_DistanceJointEnableMotor :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointEnableMotor) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.enableMotor)
}

RecWriteArgs_DistanceJointSetMotorSpeed :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointSetMotorSpeed) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.motorSpeed)
}

RecWriteArgs_DistanceJointSetMaxMotorForce :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointSetMaxMotorForce) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.force)
}

RecWriteArgs_MotorJointSetLinearVelocity :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetLinearVelocity) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_VEC3(&rec.buffer, a.velocity)
}

RecWriteArgs_MotorJointSetAngularVelocity :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetAngularVelocity) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_VEC3(&rec.buffer, a.velocity)
}

RecWriteArgs_MotorJointSetMaxVelocityForce :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetMaxVelocityForce) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.maxForce)
}

RecWriteArgs_MotorJointSetMaxVelocityTorque :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetMaxVelocityTorque) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.maxTorque)
}

RecWriteArgs_MotorJointSetLinearHertz :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetLinearHertz) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.hertz)
}

RecWriteArgs_MotorJointSetLinearDampingRatio :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetLinearDampingRatio) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.damping)
}

RecWriteArgs_MotorJointSetAngularHertz :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetAngularHertz) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.hertz)
}

RecWriteArgs_MotorJointSetAngularDampingRatio :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetAngularDampingRatio) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.damping)
}

RecWriteArgs_MotorJointSetMaxSpringForce :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetMaxSpringForce) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.maxForce)
}

RecWriteArgs_MotorJointSetMaxSpringTorque :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetMaxSpringTorque) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.maxTorque)
}

RecWriteArgs_PrismaticJointEnableSpring :: proc(rec: ^Recording, a: ^RecArgs_PrismaticJointEnableSpring) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.enableSpring)
}

RecWriteArgs_PrismaticJointSetSpringHertz :: proc(rec: ^Recording, a: ^RecArgs_PrismaticJointSetSpringHertz) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.hertz)
}

RecWriteArgs_PrismaticJointSetSpringDampingRatio :: proc(
	rec: ^Recording,
	a: ^RecArgs_PrismaticJointSetSpringDampingRatio,
) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.dampingRatio)
}

RecWriteArgs_PrismaticJointSetTargetTranslation :: proc(
	rec: ^Recording,
	a: ^RecArgs_PrismaticJointSetTargetTranslation,
) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.translation)
}

RecWriteArgs_PrismaticJointEnableLimit :: proc(rec: ^Recording, a: ^RecArgs_PrismaticJointEnableLimit) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.enableLimit)
}

RecWriteArgs_PrismaticJointSetLimits :: proc(rec: ^Recording, a: ^RecArgs_PrismaticJointSetLimits) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.lower)
	RecW_F32(&rec.buffer, a.upper)
}

RecWriteArgs_PrismaticJointEnableMotor :: proc(rec: ^Recording, a: ^RecArgs_PrismaticJointEnableMotor) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.enableMotor)
}

RecWriteArgs_PrismaticJointSetMotorSpeed :: proc(rec: ^Recording, a: ^RecArgs_PrismaticJointSetMotorSpeed) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.motorSpeed)
}

RecWriteArgs_PrismaticJointSetMaxMotorForce :: proc(rec: ^Recording, a: ^RecArgs_PrismaticJointSetMaxMotorForce) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.force)
}

RecWriteArgs_RevoluteJointEnableSpring :: proc(rec: ^Recording, a: ^RecArgs_RevoluteJointEnableSpring) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.enableSpring)
}

RecWriteArgs_RevoluteJointSetSpringHertz :: proc(rec: ^Recording, a: ^RecArgs_RevoluteJointSetSpringHertz) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.hertz)
}

RecWriteArgs_RevoluteJointSetSpringDampingRatio :: proc(
	rec: ^Recording,
	a: ^RecArgs_RevoluteJointSetSpringDampingRatio,
) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.dampingRatio)
}

RecWriteArgs_RevoluteJointSetTargetAngle :: proc(rec: ^Recording, a: ^RecArgs_RevoluteJointSetTargetAngle) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.angle)
}

RecWriteArgs_RevoluteJointEnableLimit :: proc(rec: ^Recording, a: ^RecArgs_RevoluteJointEnableLimit) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.enableLimit)
}

RecWriteArgs_RevoluteJointSetLimits :: proc(rec: ^Recording, a: ^RecArgs_RevoluteJointSetLimits) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.lower)
	RecW_F32(&rec.buffer, a.upper)
}

RecWriteArgs_RevoluteJointEnableMotor :: proc(rec: ^Recording, a: ^RecArgs_RevoluteJointEnableMotor) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.enableMotor)
}

RecWriteArgs_RevoluteJointSetMotorSpeed :: proc(rec: ^Recording, a: ^RecArgs_RevoluteJointSetMotorSpeed) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.motorSpeed)
}

RecWriteArgs_RevoluteJointSetMaxMotorTorque :: proc(rec: ^Recording, a: ^RecArgs_RevoluteJointSetMaxMotorTorque) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.torque)
}

RecWriteArgs_SphericalJointEnableConeLimit :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointEnableConeLimit) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.enableLimit)
}

RecWriteArgs_SphericalJointSetConeLimit :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointSetConeLimit) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.angleRadians)
}

RecWriteArgs_SphericalJointEnableTwistLimit :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointEnableTwistLimit) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.enableLimit)
}

RecWriteArgs_SphericalJointSetTwistLimits :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointSetTwistLimits) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.lower)
	RecW_F32(&rec.buffer, a.upper)
}

RecWriteArgs_SphericalJointEnableSpring :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointEnableSpring) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.enableSpring)
}

RecWriteArgs_SphericalJointSetSpringHertz :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointSetSpringHertz) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.hertz)
}

RecWriteArgs_SphericalJointSetSpringDampingRatio :: proc(
	rec: ^Recording,
	a: ^RecArgs_SphericalJointSetSpringDampingRatio,
) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.dampingRatio)
}

RecWriteArgs_SphericalJointSetTargetRotation :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointSetTargetRotation) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_QUAT(&rec.buffer, a.targetRotation)
}

RecWriteArgs_SphericalJointEnableMotor :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointEnableMotor) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.enableMotor)
}

RecWriteArgs_SphericalJointSetMotorVelocity :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointSetMotorVelocity) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_VEC3(&rec.buffer, a.motorVelocity)
}

RecWriteArgs_SphericalJointSetMaxMotorTorque :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointSetMaxMotorTorque) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.torque)
}

RecWriteArgs_WeldJointSetLinearHertz :: proc(rec: ^Recording, a: ^RecArgs_WeldJointSetLinearHertz) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.hertz)
}

RecWriteArgs_WeldJointSetLinearDampingRatio :: proc(rec: ^Recording, a: ^RecArgs_WeldJointSetLinearDampingRatio) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.dampingRatio)
}

RecWriteArgs_WeldJointSetAngularHertz :: proc(rec: ^Recording, a: ^RecArgs_WeldJointSetAngularHertz) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.hertz)
}

RecWriteArgs_WeldJointSetAngularDampingRatio :: proc(rec: ^Recording, a: ^RecArgs_WeldJointSetAngularDampingRatio) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.dampingRatio)
}

RecWriteArgs_WheelJointEnableSuspension :: proc(rec: ^Recording, a: ^RecArgs_WheelJointEnableSuspension) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_WheelJointSetSuspensionHertz :: proc(rec: ^Recording, a: ^RecArgs_WheelJointSetSuspensionHertz) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.hertz)
}

RecWriteArgs_WheelJointSetSuspensionDampingRatio :: proc(
	rec: ^Recording,
	a: ^RecArgs_WheelJointSetSuspensionDampingRatio,
) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.dampingRatio)
}

RecWriteArgs_WheelJointEnableSuspensionLimit :: proc(rec: ^Recording, a: ^RecArgs_WheelJointEnableSuspensionLimit) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_WheelJointSetSuspensionLimits :: proc(rec: ^Recording, a: ^RecArgs_WheelJointSetSuspensionLimits) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.lower)
	RecW_F32(&rec.buffer, a.upper)
}

RecWriteArgs_WheelJointEnableSpinMotor :: proc(rec: ^Recording, a: ^RecArgs_WheelJointEnableSpinMotor) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_WheelJointSetSpinMotorSpeed :: proc(rec: ^Recording, a: ^RecArgs_WheelJointSetSpinMotorSpeed) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.speed)
}

RecWriteArgs_WheelJointSetMaxSpinTorque :: proc(rec: ^Recording, a: ^RecArgs_WheelJointSetMaxSpinTorque) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.torque)
}

RecWriteArgs_WheelJointEnableSteering :: proc(rec: ^Recording, a: ^RecArgs_WheelJointEnableSteering) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_WheelJointSetSteeringHertz :: proc(rec: ^Recording, a: ^RecArgs_WheelJointSetSteeringHertz) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.hertz)
}

RecWriteArgs_WheelJointSetSteeringDampingRatio :: proc(
	rec: ^Recording,
	a: ^RecArgs_WheelJointSetSteeringDampingRatio,
) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.dampingRatio)
}

RecWriteArgs_WheelJointSetMaxSteeringTorque :: proc(rec: ^Recording, a: ^RecArgs_WheelJointSetMaxSteeringTorque) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.torque)
}

RecWriteArgs_WheelJointEnableSteeringLimit :: proc(rec: ^Recording, a: ^RecArgs_WheelJointEnableSteeringLimit) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_BOOL(&rec.buffer, a.flag)
}

RecWriteArgs_WheelJointSetSteeringLimits :: proc(rec: ^Recording, a: ^RecArgs_WheelJointSetSteeringLimits) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.lower)
	RecW_F32(&rec.buffer, a.upper)
}

RecWriteArgs_WheelJointSetTargetSteeringAngle :: proc(rec: ^Recording, a: ^RecArgs_WheelJointSetTargetSteeringAngle) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.radians)
}

RecWriteArgs_CreateMoverJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateMoverJoint) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_MOVERJOINTDEF(&rec.buffer, a.def)
}

RecWriteArgs_MoverJointSetLinearVelocity :: proc(rec: ^Recording, a: ^RecArgs_MoverJointSetLinearVelocity) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_VEC3(&rec.buffer, a.velocity)
}

RecWriteArgs_MoverJointSetMaxVelocityForce :: proc(rec: ^Recording, a: ^RecArgs_MoverJointSetMaxVelocityForce) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.maxForce)
}

RecWriteArgs_CreatePogoJoint :: proc(rec: ^Recording, a: ^RecArgs_CreatePogoJoint) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_POGOJOINTDEF(&rec.buffer, a.def)
}

RecWriteArgs_PogoJointSetRestLength :: proc(rec: ^Recording, a: ^RecArgs_PogoJointSetRestLength) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.length)
}

RecWriteArgs_PogoJointSetSpringHertz :: proc(rec: ^Recording, a: ^RecArgs_PogoJointSetSpringHertz) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.hertz)
}

RecWriteArgs_PogoJointSetSpringDampingRatio :: proc(rec: ^Recording, a: ^RecArgs_PogoJointSetSpringDampingRatio) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.dampingRatio)
}

RecWriteArgs_PogoJointSetMaxTensionForce :: proc(rec: ^Recording, a: ^RecArgs_PogoJointSetMaxTensionForce) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.force)
}

RecWriteArgs_PogoJointSetMaxCompressionForce :: proc(rec: ^Recording, a: ^RecArgs_PogoJointSetMaxCompressionForce) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_F32(&rec.buffer, a.force)
}

RecWriteArgs_PogoJointSetNormal :: proc(rec: ^Recording, a: ^RecArgs_PogoJointSetNormal) {
	RecW_JOINTID(&rec.buffer, a.joint)
	RecW_VEC3(&rec.buffer, a.normal)
}

RecWriteArgs_QueryOverlapAABB :: proc(rec: ^Recording, a: ^RecArgs_QueryOverlapAABB) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_AABB(&rec.buffer, a.aabb)
	RecW_QUERYFILTER(&rec.buffer, a.filter)
}

RecWriteArgs_QueryOverlapShape :: proc(rec: ^Recording, a: ^RecArgs_QueryOverlapShape) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_POSITION(&rec.buffer, a.origin)
	RecW_SHAPEPROXY(&rec.buffer, a.proxy)
	RecW_QUERYFILTER(&rec.buffer, a.filter)
}

RecWriteArgs_QueryCastRay :: proc(rec: ^Recording, a: ^RecArgs_QueryCastRay) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_POSITION(&rec.buffer, a.origin)
	RecW_VEC3(&rec.buffer, a.translation)
	RecW_QUERYFILTER(&rec.buffer, a.filter)
}

RecWriteArgs_QueryCastShape :: proc(rec: ^Recording, a: ^RecArgs_QueryCastShape) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_POSITION(&rec.buffer, a.origin)
	RecW_SHAPEPROXY(&rec.buffer, a.proxy)
	RecW_VEC3(&rec.buffer, a.translation)
	RecW_QUERYFILTER(&rec.buffer, a.filter)
}

RecWriteArgs_QueryCastRayClosest :: proc(rec: ^Recording, a: ^RecArgs_QueryCastRayClosest) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_POSITION(&rec.buffer, a.origin)
	RecW_VEC3(&rec.buffer, a.translation)
	RecW_QUERYFILTER(&rec.buffer, a.filter)
}

RecWriteArgs_QueryCastMover :: proc(rec: ^Recording, a: ^RecArgs_QueryCastMover) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_POSITION(&rec.buffer, a.origin)
	RecW_CAPSULE(&rec.buffer, a.mover)
	RecW_VEC3(&rec.buffer, a.translation)
	RecW_QUERYFILTER(&rec.buffer, a.filter)
}

RecWriteArgs_QueryCollideMover :: proc(rec: ^Recording, a: ^RecArgs_QueryCollideMover) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_POSITION(&rec.buffer, a.origin)
	RecW_CAPSULE(&rec.buffer, a.mover)
	RecW_QUERYFILTER(&rec.buffer, a.filter)
}

RecWriteArgs_QueryTag :: proc(rec: ^Recording, a: ^RecArgs_QueryTag) {
	RecW_U64(&rec.buffer, a.key)
}

RecWriteArgs_StateHash :: proc(rec: ^Recording, a: ^RecArgs_StateHash) {
	RecW_WORLDID(&rec.buffer, a.world)
	RecW_U64(&rec.buffer, a.hash)
}

RecWriteArgs_RecordingBounds :: proc(rec: ^Recording, a: ^RecArgs_RecordingBounds) {
	RecW_AABB(&rec.buffer, a.bounds)
}

// Codegen: full writers. Setters may run on threads that each own a distinct object,
// so hold the lock across the whole record. Without it a concurrent writer splices its bytes between
// our begin and end and the record desyncs replay. Same lock the query commit path takes.

RecWrite_DestroyWorld :: proc(rec: ^Recording, a: ^RecArgs_DestroyWorld) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpDestroyWorld))
	RecWriteArgs_DestroyWorld(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_Step :: proc(rec: ^Recording, a: ^RecArgs_Step) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpStep))
	RecWriteArgs_Step(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WorldEnableSleeping :: proc(rec: ^Recording, a: ^RecArgs_WorldEnableSleeping) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWorldEnableSleeping))
	RecWriteArgs_WorldEnableSleeping(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WorldEnableContinuous :: proc(rec: ^Recording, a: ^RecArgs_WorldEnableContinuous) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWorldEnableContinuous))
	RecWriteArgs_WorldEnableContinuous(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WorldSetRestitutionThreshold :: proc(rec: ^Recording, a: ^RecArgs_WorldSetRestitutionThreshold) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWorldSetRestitutionThreshold))
	RecWriteArgs_WorldSetRestitutionThreshold(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WorldSetHitEventThreshold :: proc(rec: ^Recording, a: ^RecArgs_WorldSetHitEventThreshold) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWorldSetHitEventThreshold))
	RecWriteArgs_WorldSetHitEventThreshold(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WorldSetGravity :: proc(rec: ^Recording, a: ^RecArgs_WorldSetGravity) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWorldSetGravity))
	RecWriteArgs_WorldSetGravity(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WorldExplode :: proc(rec: ^Recording, a: ^RecArgs_WorldExplode) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWorldExplode))
	RecWriteArgs_WorldExplode(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WorldSetContactTuning :: proc(rec: ^Recording, a: ^RecArgs_WorldSetContactTuning) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWorldSetContactTuning))
	RecWriteArgs_WorldSetContactTuning(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WorldSetContactRecycleDistance :: proc(rec: ^Recording, a: ^RecArgs_WorldSetContactRecycleDistance) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWorldSetContactRecycleDistance))
	RecWriteArgs_WorldSetContactRecycleDistance(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WorldSetMaximumLinearSpeed :: proc(rec: ^Recording, a: ^RecArgs_WorldSetMaximumLinearSpeed) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWorldSetMaximumLinearSpeed))
	RecWriteArgs_WorldSetMaximumLinearSpeed(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WorldEnableWarmStarting :: proc(rec: ^Recording, a: ^RecArgs_WorldEnableWarmStarting) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWorldEnableWarmStarting))
	RecWriteArgs_WorldEnableWarmStarting(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WorldRebuildStaticTree :: proc(rec: ^Recording, a: ^RecArgs_WorldRebuildStaticTree) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWorldRebuildStaticTree))
	RecWriteArgs_WorldRebuildStaticTree(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WorldEnableSpeculative :: proc(rec: ^Recording, a: ^RecArgs_WorldEnableSpeculative) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWorldEnableSpeculative))
	RecWriteArgs_WorldEnableSpeculative(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WorldSetRestitutionIterations :: proc(rec: ^Recording, a: ^RecArgs_WorldSetRestitutionIterations) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWorldSetRestitutionIterations))
	RecWriteArgs_WorldSetRestitutionIterations(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WorldEnableRestitutionPropagation :: proc(rec: ^Recording, a: ^RecArgs_WorldEnableRestitutionPropagation) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWorldEnableRestitutionPropagation))
	RecWriteArgs_WorldEnableRestitutionPropagation(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WorldSetIsotropicInertiaTolerance :: proc(rec: ^Recording, a: ^RecArgs_WorldSetIsotropicInertiaTolerance) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWorldSetIsotropicInertiaTolerance))
	RecWriteArgs_WorldSetIsotropicInertiaTolerance(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_CreateBody :: proc(rec: ^Recording, a: ^RecArgs_CreateBody) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateBody))
	RecWriteArgs_CreateBody(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_DestroyBody :: proc(rec: ^Recording, a: ^RecArgs_DestroyBody) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpDestroyBody))
	RecWriteArgs_DestroyBody(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodySetTransform :: proc(rec: ^Recording, a: ^RecArgs_BodySetTransform) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodySetTransform))
	RecWriteArgs_BodySetTransform(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodySetLinearVelocity :: proc(rec: ^Recording, a: ^RecArgs_BodySetLinearVelocity) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodySetLinearVelocity))
	RecWriteArgs_BodySetLinearVelocity(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodySetType :: proc(rec: ^Recording, a: ^RecArgs_BodySetType) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodySetType))
	RecWriteArgs_BodySetType(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodySetName :: proc(rec: ^Recording, a: ^RecArgs_BodySetName) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodySetName))
	RecWriteArgs_BodySetName(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodySetAngularVelocity :: proc(rec: ^Recording, a: ^RecArgs_BodySetAngularVelocity) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodySetAngularVelocity))
	RecWriteArgs_BodySetAngularVelocity(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodySetTargetTransform :: proc(rec: ^Recording, a: ^RecArgs_BodySetTargetTransform) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodySetTargetTransform))
	RecWriteArgs_BodySetTargetTransform(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodyApplyForce :: proc(rec: ^Recording, a: ^RecArgs_BodyApplyForce) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodyApplyForce))
	RecWriteArgs_BodyApplyForce(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodyApplyForceToCenter :: proc(rec: ^Recording, a: ^RecArgs_BodyApplyForceToCenter) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodyApplyForceToCenter))
	RecWriteArgs_BodyApplyForceToCenter(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodyApplyTorque :: proc(rec: ^Recording, a: ^RecArgs_BodyApplyTorque) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodyApplyTorque))
	RecWriteArgs_BodyApplyTorque(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodyApplyLinearImpulse :: proc(rec: ^Recording, a: ^RecArgs_BodyApplyLinearImpulse) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodyApplyLinearImpulse))
	RecWriteArgs_BodyApplyLinearImpulse(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodyApplyLinearImpulseToCenter :: proc(rec: ^Recording, a: ^RecArgs_BodyApplyLinearImpulseToCenter) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodyApplyLinearImpulseToCenter))
	RecWriteArgs_BodyApplyLinearImpulseToCenter(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodyApplyAngularImpulse :: proc(rec: ^Recording, a: ^RecArgs_BodyApplyAngularImpulse) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodyApplyAngularImpulse))
	RecWriteArgs_BodyApplyAngularImpulse(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodySetMassData :: proc(rec: ^Recording, a: ^RecArgs_BodySetMassData) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodySetMassData))
	RecWriteArgs_BodySetMassData(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodyApplyMassFromShapes :: proc(rec: ^Recording, a: ^RecArgs_BodyApplyMassFromShapes) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodyApplyMassFromShapes))
	RecWriteArgs_BodyApplyMassFromShapes(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodySetLinearDamping :: proc(rec: ^Recording, a: ^RecArgs_BodySetLinearDamping) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodySetLinearDamping))
	RecWriteArgs_BodySetLinearDamping(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodySetAngularDamping :: proc(rec: ^Recording, a: ^RecArgs_BodySetAngularDamping) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodySetAngularDamping))
	RecWriteArgs_BodySetAngularDamping(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodySetGravityScale :: proc(rec: ^Recording, a: ^RecArgs_BodySetGravityScale) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodySetGravityScale))
	RecWriteArgs_BodySetGravityScale(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodySetAwake :: proc(rec: ^Recording, a: ^RecArgs_BodySetAwake) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodySetAwake))
	RecWriteArgs_BodySetAwake(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodyEnableSleep :: proc(rec: ^Recording, a: ^RecArgs_BodyEnableSleep) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodyEnableSleep))
	RecWriteArgs_BodyEnableSleep(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodySetSleepThreshold :: proc(rec: ^Recording, a: ^RecArgs_BodySetSleepThreshold) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodySetSleepThreshold))
	RecWriteArgs_BodySetSleepThreshold(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodyDisable :: proc(rec: ^Recording, a: ^RecArgs_BodyDisable) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodyDisable))
	RecWriteArgs_BodyDisable(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodyEnable :: proc(rec: ^Recording, a: ^RecArgs_BodyEnable) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodyEnable))
	RecWriteArgs_BodyEnable(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodySetMotionLocks :: proc(rec: ^Recording, a: ^RecArgs_BodySetMotionLocks) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodySetMotionLocks))
	RecWriteArgs_BodySetMotionLocks(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodySetBullet :: proc(rec: ^Recording, a: ^RecArgs_BodySetBullet) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodySetBullet))
	RecWriteArgs_BodySetBullet(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodyEnableContactRecycling :: proc(rec: ^Recording, a: ^RecArgs_BodyEnableContactRecycling) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodyEnableContactRecycling))
	RecWriteArgs_BodyEnableContactRecycling(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodyEnableHitEvents :: proc(rec: ^Recording, a: ^RecArgs_BodyEnableHitEvents) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodyEnableHitEvents))
	RecWriteArgs_BodyEnableHitEvents(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodyAllowFastRotation :: proc(rec: ^Recording, a: ^RecArgs_BodyAllowFastRotation) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodyAllowFastRotation))
	RecWriteArgs_BodyAllowFastRotation(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_BodySetSafetyFactor :: proc(rec: ^Recording, a: ^RecArgs_BodySetSafetyFactor) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpBodySetSafetyFactor))
	RecWriteArgs_BodySetSafetyFactor(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_CreateSphereShape :: proc(rec: ^Recording, a: ^RecArgs_CreateSphereShape) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateSphereShape))
	RecWriteArgs_CreateSphereShape(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_CreateCapsuleShape :: proc(rec: ^Recording, a: ^RecArgs_CreateCapsuleShape) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateCapsuleShape))
	RecWriteArgs_CreateCapsuleShape(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_CreateHullShape :: proc(rec: ^Recording, a: ^RecArgs_CreateHullShape) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateHullShape))
	RecWriteArgs_CreateHullShape(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_CreateMeshShape :: proc(rec: ^Recording, a: ^RecArgs_CreateMeshShape) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateMeshShape))
	RecWriteArgs_CreateMeshShape(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_CreateHeightFieldShape :: proc(rec: ^Recording, a: ^RecArgs_CreateHeightFieldShape) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateHeightFieldShape))
	RecWriteArgs_CreateHeightFieldShape(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_CreateCompoundShape :: proc(rec: ^Recording, a: ^RecArgs_CreateCompoundShape) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateCompoundShape))
	RecWriteArgs_CreateCompoundShape(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_DestroyShape :: proc(rec: ^Recording, a: ^RecArgs_DestroyShape) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpDestroyShape))
	RecWriteArgs_DestroyShape(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ShapeSetDensity :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetDensity) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpShapeSetDensity))
	RecWriteArgs_ShapeSetDensity(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ShapeSetFriction :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetFriction) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpShapeSetFriction))
	RecWriteArgs_ShapeSetFriction(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ShapeSetRestitution :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetRestitution) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpShapeSetRestitution))
	RecWriteArgs_ShapeSetRestitution(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ShapeSetSurfaceMaterial :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetSurfaceMaterial) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpShapeSetSurfaceMaterial))
	RecWriteArgs_ShapeSetSurfaceMaterial(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ShapeSetFilter :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetFilter) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpShapeSetFilter))
	RecWriteArgs_ShapeSetFilter(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ShapeEnableSensorEvents :: proc(rec: ^Recording, a: ^RecArgs_ShapeEnableSensorEvents) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpShapeEnableSensorEvents))
	RecWriteArgs_ShapeEnableSensorEvents(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ShapeEnableContactEvents :: proc(rec: ^Recording, a: ^RecArgs_ShapeEnableContactEvents) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpShapeEnableContactEvents))
	RecWriteArgs_ShapeEnableContactEvents(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ShapeEnablePreSolveEvents :: proc(rec: ^Recording, a: ^RecArgs_ShapeEnablePreSolveEvents) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpShapeEnablePreSolveEvents))
	RecWriteArgs_ShapeEnablePreSolveEvents(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ShapeEnableHitEvents :: proc(rec: ^Recording, a: ^RecArgs_ShapeEnableHitEvents) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpShapeEnableHitEvents))
	RecWriteArgs_ShapeEnableHitEvents(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ShapeSetSphere :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetSphere) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpShapeSetSphere))
	RecWriteArgs_ShapeSetSphere(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ShapeSetCapsule :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetCapsule) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpShapeSetCapsule))
	RecWriteArgs_ShapeSetCapsule(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ShapeApplyWind :: proc(rec: ^Recording, a: ^RecArgs_ShapeApplyWind) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpShapeApplyWind))
	RecWriteArgs_ShapeApplyWind(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ShapeSetName :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetName) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpShapeSetName))
	RecWriteArgs_ShapeSetName(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ShapeSetMeshMaterial :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetMeshMaterial) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpShapeSetMeshMaterial))
	RecWriteArgs_ShapeSetMeshMaterial(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ShapeSetHull :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetHull) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpShapeSetHull))
	RecWriteArgs_ShapeSetHull(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ShapeSetMesh :: proc(rec: ^Recording, a: ^RecArgs_ShapeSetMesh) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpShapeSetMesh))
	RecWriteArgs_ShapeSetMesh(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_CreateParallelJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateParallelJoint) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateParallelJoint))
	RecWriteArgs_CreateParallelJoint(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_CreateDistanceJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateDistanceJoint) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateDistanceJoint))
	RecWriteArgs_CreateDistanceJoint(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_CreateFilterJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateFilterJoint) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateFilterJoint))
	RecWriteArgs_CreateFilterJoint(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_CreateMotorJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateMotorJoint) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateMotorJoint))
	RecWriteArgs_CreateMotorJoint(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_CreatePrismaticJoint :: proc(rec: ^Recording, a: ^RecArgs_CreatePrismaticJoint) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreatePrismaticJoint))
	RecWriteArgs_CreatePrismaticJoint(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_CreateRevoluteJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateRevoluteJoint) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateRevoluteJoint))
	RecWriteArgs_CreateRevoluteJoint(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_CreateSphericalJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateSphericalJoint) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateSphericalJoint))
	RecWriteArgs_CreateSphericalJoint(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_CreateWeldJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateWeldJoint) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateWeldJoint))
	RecWriteArgs_CreateWeldJoint(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_CreateWheelJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateWheelJoint) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateWheelJoint))
	RecWriteArgs_CreateWheelJoint(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_DestroyJoint :: proc(rec: ^Recording, a: ^RecArgs_DestroyJoint) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpDestroyJoint))
	RecWriteArgs_DestroyJoint(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_JointSetLocalFrameA :: proc(rec: ^Recording, a: ^RecArgs_JointSetLocalFrameA) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpJointSetLocalFrameA))
	RecWriteArgs_JointSetLocalFrameA(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_JointSetLocalFrameB :: proc(rec: ^Recording, a: ^RecArgs_JointSetLocalFrameB) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpJointSetLocalFrameB))
	RecWriteArgs_JointSetLocalFrameB(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_JointSetCollideConnected :: proc(rec: ^Recording, a: ^RecArgs_JointSetCollideConnected) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpJointSetCollideConnected))
	RecWriteArgs_JointSetCollideConnected(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_JointWakeBodies :: proc(rec: ^Recording, a: ^RecArgs_JointWakeBodies) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpJointWakeBodies))
	RecWriteArgs_JointWakeBodies(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_JointSetConstraintTuning :: proc(rec: ^Recording, a: ^RecArgs_JointSetConstraintTuning) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpJointSetConstraintTuning))
	RecWriteArgs_JointSetConstraintTuning(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_JointSetForceThreshold :: proc(rec: ^Recording, a: ^RecArgs_JointSetForceThreshold) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpJointSetForceThreshold))
	RecWriteArgs_JointSetForceThreshold(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_JointSetTorqueThreshold :: proc(rec: ^Recording, a: ^RecArgs_JointSetTorqueThreshold) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpJointSetTorqueThreshold))
	RecWriteArgs_JointSetTorqueThreshold(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ParallelJointSetSpringHertz :: proc(rec: ^Recording, a: ^RecArgs_ParallelJointSetSpringHertz) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpParallelJointSetSpringHertz))
	RecWriteArgs_ParallelJointSetSpringHertz(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ParallelJointSetSpringDampingRatio :: proc(rec: ^Recording, a: ^RecArgs_ParallelJointSetSpringDampingRatio) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpParallelJointSetSpringDampingRatio))
	RecWriteArgs_ParallelJointSetSpringDampingRatio(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_ParallelJointSetMaxTorque :: proc(rec: ^Recording, a: ^RecArgs_ParallelJointSetMaxTorque) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpParallelJointSetMaxTorque))
	RecWriteArgs_ParallelJointSetMaxTorque(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_DistanceJointSetLength :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointSetLength) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpDistanceJointSetLength))
	RecWriteArgs_DistanceJointSetLength(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_DistanceJointEnableSpring :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointEnableSpring) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpDistanceJointEnableSpring))
	RecWriteArgs_DistanceJointEnableSpring(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_DistanceJointSetSpringForceRange :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointSetSpringForceRange) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpDistanceJointSetSpringForceRange))
	RecWriteArgs_DistanceJointSetSpringForceRange(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_DistanceJointSetSpringHertz :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointSetSpringHertz) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpDistanceJointSetSpringHertz))
	RecWriteArgs_DistanceJointSetSpringHertz(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_DistanceJointSetSpringDampingRatio :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointSetSpringDampingRatio) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpDistanceJointSetSpringDampingRatio))
	RecWriteArgs_DistanceJointSetSpringDampingRatio(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_DistanceJointEnableLimit :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointEnableLimit) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpDistanceJointEnableLimit))
	RecWriteArgs_DistanceJointEnableLimit(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_DistanceJointSetLengthRange :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointSetLengthRange) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpDistanceJointSetLengthRange))
	RecWriteArgs_DistanceJointSetLengthRange(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_DistanceJointEnableMotor :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointEnableMotor) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpDistanceJointEnableMotor))
	RecWriteArgs_DistanceJointEnableMotor(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_DistanceJointSetMotorSpeed :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointSetMotorSpeed) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpDistanceJointSetMotorSpeed))
	RecWriteArgs_DistanceJointSetMotorSpeed(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_DistanceJointSetMaxMotorForce :: proc(rec: ^Recording, a: ^RecArgs_DistanceJointSetMaxMotorForce) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpDistanceJointSetMaxMotorForce))
	RecWriteArgs_DistanceJointSetMaxMotorForce(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_MotorJointSetLinearVelocity :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetLinearVelocity) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpMotorJointSetLinearVelocity))
	RecWriteArgs_MotorJointSetLinearVelocity(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_MotorJointSetAngularVelocity :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetAngularVelocity) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpMotorJointSetAngularVelocity))
	RecWriteArgs_MotorJointSetAngularVelocity(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_MotorJointSetMaxVelocityForce :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetMaxVelocityForce) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpMotorJointSetMaxVelocityForce))
	RecWriteArgs_MotorJointSetMaxVelocityForce(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_MotorJointSetMaxVelocityTorque :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetMaxVelocityTorque) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpMotorJointSetMaxVelocityTorque))
	RecWriteArgs_MotorJointSetMaxVelocityTorque(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_MotorJointSetLinearHertz :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetLinearHertz) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpMotorJointSetLinearHertz))
	RecWriteArgs_MotorJointSetLinearHertz(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_MotorJointSetLinearDampingRatio :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetLinearDampingRatio) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpMotorJointSetLinearDampingRatio))
	RecWriteArgs_MotorJointSetLinearDampingRatio(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_MotorJointSetAngularHertz :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetAngularHertz) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpMotorJointSetAngularHertz))
	RecWriteArgs_MotorJointSetAngularHertz(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_MotorJointSetAngularDampingRatio :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetAngularDampingRatio) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpMotorJointSetAngularDampingRatio))
	RecWriteArgs_MotorJointSetAngularDampingRatio(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_MotorJointSetMaxSpringForce :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetMaxSpringForce) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpMotorJointSetMaxSpringForce))
	RecWriteArgs_MotorJointSetMaxSpringForce(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_MotorJointSetMaxSpringTorque :: proc(rec: ^Recording, a: ^RecArgs_MotorJointSetMaxSpringTorque) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpMotorJointSetMaxSpringTorque))
	RecWriteArgs_MotorJointSetMaxSpringTorque(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_PrismaticJointEnableSpring :: proc(rec: ^Recording, a: ^RecArgs_PrismaticJointEnableSpring) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpPrismaticJointEnableSpring))
	RecWriteArgs_PrismaticJointEnableSpring(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_PrismaticJointSetSpringHertz :: proc(rec: ^Recording, a: ^RecArgs_PrismaticJointSetSpringHertz) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpPrismaticJointSetSpringHertz))
	RecWriteArgs_PrismaticJointSetSpringHertz(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_PrismaticJointSetSpringDampingRatio :: proc(
	rec: ^Recording,
	a: ^RecArgs_PrismaticJointSetSpringDampingRatio,
) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpPrismaticJointSetSpringDampingRatio))
	RecWriteArgs_PrismaticJointSetSpringDampingRatio(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_PrismaticJointSetTargetTranslation :: proc(rec: ^Recording, a: ^RecArgs_PrismaticJointSetTargetTranslation) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpPrismaticJointSetTargetTranslation))
	RecWriteArgs_PrismaticJointSetTargetTranslation(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_PrismaticJointEnableLimit :: proc(rec: ^Recording, a: ^RecArgs_PrismaticJointEnableLimit) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpPrismaticJointEnableLimit))
	RecWriteArgs_PrismaticJointEnableLimit(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_PrismaticJointSetLimits :: proc(rec: ^Recording, a: ^RecArgs_PrismaticJointSetLimits) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpPrismaticJointSetLimits))
	RecWriteArgs_PrismaticJointSetLimits(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_PrismaticJointEnableMotor :: proc(rec: ^Recording, a: ^RecArgs_PrismaticJointEnableMotor) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpPrismaticJointEnableMotor))
	RecWriteArgs_PrismaticJointEnableMotor(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_PrismaticJointSetMotorSpeed :: proc(rec: ^Recording, a: ^RecArgs_PrismaticJointSetMotorSpeed) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpPrismaticJointSetMotorSpeed))
	RecWriteArgs_PrismaticJointSetMotorSpeed(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_PrismaticJointSetMaxMotorForce :: proc(rec: ^Recording, a: ^RecArgs_PrismaticJointSetMaxMotorForce) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpPrismaticJointSetMaxMotorForce))
	RecWriteArgs_PrismaticJointSetMaxMotorForce(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_RevoluteJointEnableSpring :: proc(rec: ^Recording, a: ^RecArgs_RevoluteJointEnableSpring) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpRevoluteJointEnableSpring))
	RecWriteArgs_RevoluteJointEnableSpring(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_RevoluteJointSetSpringHertz :: proc(rec: ^Recording, a: ^RecArgs_RevoluteJointSetSpringHertz) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpRevoluteJointSetSpringHertz))
	RecWriteArgs_RevoluteJointSetSpringHertz(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_RevoluteJointSetSpringDampingRatio :: proc(rec: ^Recording, a: ^RecArgs_RevoluteJointSetSpringDampingRatio) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpRevoluteJointSetSpringDampingRatio))
	RecWriteArgs_RevoluteJointSetSpringDampingRatio(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_RevoluteJointSetTargetAngle :: proc(rec: ^Recording, a: ^RecArgs_RevoluteJointSetTargetAngle) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpRevoluteJointSetTargetAngle))
	RecWriteArgs_RevoluteJointSetTargetAngle(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_RevoluteJointEnableLimit :: proc(rec: ^Recording, a: ^RecArgs_RevoluteJointEnableLimit) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpRevoluteJointEnableLimit))
	RecWriteArgs_RevoluteJointEnableLimit(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_RevoluteJointSetLimits :: proc(rec: ^Recording, a: ^RecArgs_RevoluteJointSetLimits) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpRevoluteJointSetLimits))
	RecWriteArgs_RevoluteJointSetLimits(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_RevoluteJointEnableMotor :: proc(rec: ^Recording, a: ^RecArgs_RevoluteJointEnableMotor) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpRevoluteJointEnableMotor))
	RecWriteArgs_RevoluteJointEnableMotor(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_RevoluteJointSetMotorSpeed :: proc(rec: ^Recording, a: ^RecArgs_RevoluteJointSetMotorSpeed) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpRevoluteJointSetMotorSpeed))
	RecWriteArgs_RevoluteJointSetMotorSpeed(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_RevoluteJointSetMaxMotorTorque :: proc(rec: ^Recording, a: ^RecArgs_RevoluteJointSetMaxMotorTorque) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpRevoluteJointSetMaxMotorTorque))
	RecWriteArgs_RevoluteJointSetMaxMotorTorque(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_SphericalJointEnableConeLimit :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointEnableConeLimit) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpSphericalJointEnableConeLimit))
	RecWriteArgs_SphericalJointEnableConeLimit(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_SphericalJointSetConeLimit :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointSetConeLimit) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpSphericalJointSetConeLimit))
	RecWriteArgs_SphericalJointSetConeLimit(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_SphericalJointEnableTwistLimit :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointEnableTwistLimit) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpSphericalJointEnableTwistLimit))
	RecWriteArgs_SphericalJointEnableTwistLimit(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_SphericalJointSetTwistLimits :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointSetTwistLimits) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpSphericalJointSetTwistLimits))
	RecWriteArgs_SphericalJointSetTwistLimits(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_SphericalJointEnableSpring :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointEnableSpring) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpSphericalJointEnableSpring))
	RecWriteArgs_SphericalJointEnableSpring(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_SphericalJointSetSpringHertz :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointSetSpringHertz) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpSphericalJointSetSpringHertz))
	RecWriteArgs_SphericalJointSetSpringHertz(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_SphericalJointSetSpringDampingRatio :: proc(
	rec: ^Recording,
	a: ^RecArgs_SphericalJointSetSpringDampingRatio,
) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpSphericalJointSetSpringDampingRatio))
	RecWriteArgs_SphericalJointSetSpringDampingRatio(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_SphericalJointSetTargetRotation :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointSetTargetRotation) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpSphericalJointSetTargetRotation))
	RecWriteArgs_SphericalJointSetTargetRotation(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_SphericalJointEnableMotor :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointEnableMotor) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpSphericalJointEnableMotor))
	RecWriteArgs_SphericalJointEnableMotor(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_SphericalJointSetMotorVelocity :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointSetMotorVelocity) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpSphericalJointSetMotorVelocity))
	RecWriteArgs_SphericalJointSetMotorVelocity(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_SphericalJointSetMaxMotorTorque :: proc(rec: ^Recording, a: ^RecArgs_SphericalJointSetMaxMotorTorque) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpSphericalJointSetMaxMotorTorque))
	RecWriteArgs_SphericalJointSetMaxMotorTorque(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WeldJointSetLinearHertz :: proc(rec: ^Recording, a: ^RecArgs_WeldJointSetLinearHertz) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWeldJointSetLinearHertz))
	RecWriteArgs_WeldJointSetLinearHertz(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WeldJointSetLinearDampingRatio :: proc(rec: ^Recording, a: ^RecArgs_WeldJointSetLinearDampingRatio) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWeldJointSetLinearDampingRatio))
	RecWriteArgs_WeldJointSetLinearDampingRatio(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WeldJointSetAngularHertz :: proc(rec: ^Recording, a: ^RecArgs_WeldJointSetAngularHertz) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWeldJointSetAngularHertz))
	RecWriteArgs_WeldJointSetAngularHertz(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WeldJointSetAngularDampingRatio :: proc(rec: ^Recording, a: ^RecArgs_WeldJointSetAngularDampingRatio) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWeldJointSetAngularDampingRatio))
	RecWriteArgs_WeldJointSetAngularDampingRatio(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WheelJointEnableSuspension :: proc(rec: ^Recording, a: ^RecArgs_WheelJointEnableSuspension) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWheelJointEnableSuspension))
	RecWriteArgs_WheelJointEnableSuspension(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WheelJointSetSuspensionHertz :: proc(rec: ^Recording, a: ^RecArgs_WheelJointSetSuspensionHertz) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWheelJointSetSuspensionHertz))
	RecWriteArgs_WheelJointSetSuspensionHertz(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WheelJointSetSuspensionDampingRatio :: proc(
	rec: ^Recording,
	a: ^RecArgs_WheelJointSetSuspensionDampingRatio,
) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWheelJointSetSuspensionDampingRatio))
	RecWriteArgs_WheelJointSetSuspensionDampingRatio(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WheelJointEnableSuspensionLimit :: proc(rec: ^Recording, a: ^RecArgs_WheelJointEnableSuspensionLimit) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWheelJointEnableSuspensionLimit))
	RecWriteArgs_WheelJointEnableSuspensionLimit(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WheelJointSetSuspensionLimits :: proc(rec: ^Recording, a: ^RecArgs_WheelJointSetSuspensionLimits) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWheelJointSetSuspensionLimits))
	RecWriteArgs_WheelJointSetSuspensionLimits(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WheelJointEnableSpinMotor :: proc(rec: ^Recording, a: ^RecArgs_WheelJointEnableSpinMotor) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWheelJointEnableSpinMotor))
	RecWriteArgs_WheelJointEnableSpinMotor(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WheelJointSetSpinMotorSpeed :: proc(rec: ^Recording, a: ^RecArgs_WheelJointSetSpinMotorSpeed) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWheelJointSetSpinMotorSpeed))
	RecWriteArgs_WheelJointSetSpinMotorSpeed(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WheelJointSetMaxSpinTorque :: proc(rec: ^Recording, a: ^RecArgs_WheelJointSetMaxSpinTorque) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWheelJointSetMaxSpinTorque))
	RecWriteArgs_WheelJointSetMaxSpinTorque(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WheelJointEnableSteering :: proc(rec: ^Recording, a: ^RecArgs_WheelJointEnableSteering) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWheelJointEnableSteering))
	RecWriteArgs_WheelJointEnableSteering(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WheelJointSetSteeringHertz :: proc(rec: ^Recording, a: ^RecArgs_WheelJointSetSteeringHertz) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWheelJointSetSteeringHertz))
	RecWriteArgs_WheelJointSetSteeringHertz(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WheelJointSetSteeringDampingRatio :: proc(rec: ^Recording, a: ^RecArgs_WheelJointSetSteeringDampingRatio) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWheelJointSetSteeringDampingRatio))
	RecWriteArgs_WheelJointSetSteeringDampingRatio(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WheelJointSetMaxSteeringTorque :: proc(rec: ^Recording, a: ^RecArgs_WheelJointSetMaxSteeringTorque) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWheelJointSetMaxSteeringTorque))
	RecWriteArgs_WheelJointSetMaxSteeringTorque(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WheelJointEnableSteeringLimit :: proc(rec: ^Recording, a: ^RecArgs_WheelJointEnableSteeringLimit) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWheelJointEnableSteeringLimit))
	RecWriteArgs_WheelJointEnableSteeringLimit(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WheelJointSetSteeringLimits :: proc(rec: ^Recording, a: ^RecArgs_WheelJointSetSteeringLimits) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWheelJointSetSteeringLimits))
	RecWriteArgs_WheelJointSetSteeringLimits(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_WheelJointSetTargetSteeringAngle :: proc(rec: ^Recording, a: ^RecArgs_WheelJointSetTargetSteeringAngle) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpWheelJointSetTargetSteeringAngle))
	RecWriteArgs_WheelJointSetTargetSteeringAngle(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_CreateMoverJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateMoverJoint) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateMoverJoint))
	RecWriteArgs_CreateMoverJoint(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_MoverJointSetLinearVelocity :: proc(rec: ^Recording, a: ^RecArgs_MoverJointSetLinearVelocity) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpMoverJointSetLinearVelocity))
	RecWriteArgs_MoverJointSetLinearVelocity(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_MoverJointSetMaxVelocityForce :: proc(rec: ^Recording, a: ^RecArgs_MoverJointSetMaxVelocityForce) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpMoverJointSetMaxVelocityForce))
	RecWriteArgs_MoverJointSetMaxVelocityForce(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_CreatePogoJoint :: proc(rec: ^Recording, a: ^RecArgs_CreatePogoJoint) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreatePogoJoint))
	RecWriteArgs_CreatePogoJoint(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_PogoJointSetRestLength :: proc(rec: ^Recording, a: ^RecArgs_PogoJointSetRestLength) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpPogoJointSetRestLength))
	RecWriteArgs_PogoJointSetRestLength(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_PogoJointSetSpringHertz :: proc(rec: ^Recording, a: ^RecArgs_PogoJointSetSpringHertz) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpPogoJointSetSpringHertz))
	RecWriteArgs_PogoJointSetSpringHertz(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_PogoJointSetSpringDampingRatio :: proc(rec: ^Recording, a: ^RecArgs_PogoJointSetSpringDampingRatio) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpPogoJointSetSpringDampingRatio))
	RecWriteArgs_PogoJointSetSpringDampingRatio(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_PogoJointSetMaxTensionForce :: proc(rec: ^Recording, a: ^RecArgs_PogoJointSetMaxTensionForce) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpPogoJointSetMaxTensionForce))
	RecWriteArgs_PogoJointSetMaxTensionForce(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_PogoJointSetMaxCompressionForce :: proc(rec: ^Recording, a: ^RecArgs_PogoJointSetMaxCompressionForce) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpPogoJointSetMaxCompressionForce))
	RecWriteArgs_PogoJointSetMaxCompressionForce(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_PogoJointSetNormal :: proc(rec: ^Recording, a: ^RecArgs_PogoJointSetNormal) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpPogoJointSetNormal))
	RecWriteArgs_PogoJointSetNormal(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_QueryOverlapAABB :: proc(rec: ^Recording, a: ^RecArgs_QueryOverlapAABB) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpQueryOverlapAABB))
	RecWriteArgs_QueryOverlapAABB(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_QueryOverlapShape :: proc(rec: ^Recording, a: ^RecArgs_QueryOverlapShape) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpQueryOverlapShape))
	RecWriteArgs_QueryOverlapShape(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_QueryCastRay :: proc(rec: ^Recording, a: ^RecArgs_QueryCastRay) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpQueryCastRay))
	RecWriteArgs_QueryCastRay(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_QueryCastShape :: proc(rec: ^Recording, a: ^RecArgs_QueryCastShape) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpQueryCastShape))
	RecWriteArgs_QueryCastShape(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_QueryCastRayClosest :: proc(rec: ^Recording, a: ^RecArgs_QueryCastRayClosest) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpQueryCastRayClosest))
	RecWriteArgs_QueryCastRayClosest(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_QueryCastMover :: proc(rec: ^Recording, a: ^RecArgs_QueryCastMover) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpQueryCastMover))
	RecWriteArgs_QueryCastMover(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_QueryCollideMover :: proc(rec: ^Recording, a: ^RecArgs_QueryCollideMover) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpQueryCollideMover))
	RecWriteArgs_QueryCollideMover(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_QueryTag :: proc(rec: ^Recording, a: ^RecArgs_QueryTag) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpQueryTag))
	RecWriteArgs_QueryTag(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_StateHash :: proc(rec: ^Recording, a: ^RecArgs_StateHash) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpStateHash))
	RecWriteArgs_StateHash(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWrite_RecordingBounds :: proc(rec: ^Recording, a: ^RecArgs_RecordingBounds) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpRecordingBounds))
	RecWriteArgs_RecordingBounds(rec, a)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

// Codegen: create-op writers that append the returned id inside the record

RecWriteRet_CreateBody :: proc(rec: ^Recording, a: ^RecArgs_CreateBody, id: BodyId) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateBody))
	RecWriteArgs_CreateBody(rec, a)
	RecW_BODYID(&rec.buffer, id)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWriteRet_CreateSphereShape :: proc(rec: ^Recording, a: ^RecArgs_CreateSphereShape, id: ShapeId) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateSphereShape))
	RecWriteArgs_CreateSphereShape(rec, a)
	RecW_SHAPEID(&rec.buffer, id)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWriteRet_CreateCapsuleShape :: proc(rec: ^Recording, a: ^RecArgs_CreateCapsuleShape, id: ShapeId) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateCapsuleShape))
	RecWriteArgs_CreateCapsuleShape(rec, a)
	RecW_SHAPEID(&rec.buffer, id)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWriteRet_CreateHullShape :: proc(rec: ^Recording, a: ^RecArgs_CreateHullShape, id: ShapeId) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateHullShape))
	RecWriteArgs_CreateHullShape(rec, a)
	RecW_SHAPEID(&rec.buffer, id)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWriteRet_CreateMeshShape :: proc(rec: ^Recording, a: ^RecArgs_CreateMeshShape, id: ShapeId) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateMeshShape))
	RecWriteArgs_CreateMeshShape(rec, a)
	RecW_SHAPEID(&rec.buffer, id)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWriteRet_CreateHeightFieldShape :: proc(rec: ^Recording, a: ^RecArgs_CreateHeightFieldShape, id: ShapeId) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateHeightFieldShape))
	RecWriteArgs_CreateHeightFieldShape(rec, a)
	RecW_SHAPEID(&rec.buffer, id)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWriteRet_CreateCompoundShape :: proc(rec: ^Recording, a: ^RecArgs_CreateCompoundShape, id: ShapeId) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateCompoundShape))
	RecWriteArgs_CreateCompoundShape(rec, a)
	RecW_SHAPEID(&rec.buffer, id)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWriteRet_CreateParallelJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateParallelJoint, id: JointId) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateParallelJoint))
	RecWriteArgs_CreateParallelJoint(rec, a)
	RecW_JOINTID(&rec.buffer, id)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWriteRet_CreateDistanceJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateDistanceJoint, id: JointId) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateDistanceJoint))
	RecWriteArgs_CreateDistanceJoint(rec, a)
	RecW_JOINTID(&rec.buffer, id)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWriteRet_CreateFilterJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateFilterJoint, id: JointId) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateFilterJoint))
	RecWriteArgs_CreateFilterJoint(rec, a)
	RecW_JOINTID(&rec.buffer, id)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWriteRet_CreateMotorJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateMotorJoint, id: JointId) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateMotorJoint))
	RecWriteArgs_CreateMotorJoint(rec, a)
	RecW_JOINTID(&rec.buffer, id)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWriteRet_CreatePrismaticJoint :: proc(rec: ^Recording, a: ^RecArgs_CreatePrismaticJoint, id: JointId) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreatePrismaticJoint))
	RecWriteArgs_CreatePrismaticJoint(rec, a)
	RecW_JOINTID(&rec.buffer, id)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWriteRet_CreateRevoluteJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateRevoluteJoint, id: JointId) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateRevoluteJoint))
	RecWriteArgs_CreateRevoluteJoint(rec, a)
	RecW_JOINTID(&rec.buffer, id)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWriteRet_CreateSphericalJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateSphericalJoint, id: JointId) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateSphericalJoint))
	RecWriteArgs_CreateSphericalJoint(rec, a)
	RecW_JOINTID(&rec.buffer, id)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWriteRet_CreateWeldJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateWeldJoint, id: JointId) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateWeldJoint))
	RecWriteArgs_CreateWeldJoint(rec, a)
	RecW_JOINTID(&rec.buffer, id)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWriteRet_CreateWheelJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateWheelJoint, id: JointId) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateWheelJoint))
	RecWriteArgs_CreateWheelJoint(rec, a)
	RecW_JOINTID(&rec.buffer, id)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWriteRet_CreateMoverJoint :: proc(rec: ^Recording, a: ^RecArgs_CreateMoverJoint, id: JointId) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreateMoverJoint))
	RecWriteArgs_CreateMoverJoint(rec, a)
	RecW_JOINTID(&rec.buffer, id)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}

RecWriteRet_CreatePogoJoint :: proc(rec: ^Recording, a: ^RecArgs_CreatePogoJoint, id: JointId) {
	LockMutex(rec.lock)
	RecBeginRecord(rec, u8(recOpCreatePogoJoint))
	RecWriteArgs_CreatePogoJoint(rec, a)
	RecW_JOINTID(&rec.buffer, id)
	RecEndRecord(rec)
	UnlockMutex(rec.lock)
}
