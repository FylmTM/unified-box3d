// Ported from box3d shared/human.c and shared/human.h at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"

BoneId :: enum i32 {
	bone_pelvis,
	bone_spine_01,
	bone_spine_03,
	bone_neck,
	bone_thigh_l,
	bone_calf_l,
	bone_thigh_r,
	bone_calf_r,
	bone_upper_arm_l,
	bone_lower_arm_l,
	bone_upper_arm_r,
	bone_lower_arm_r,
	bone_count,
}

bone_count :: i32(BoneId.bone_count)

Bone :: struct {
	bodyId:         b3.BodyId,
	jointId:        b3.JointId,
	anchorId:       b3.BodyId,
	anchorJointId:  b3.JointId,
	localFrameA:    b3.Transform,
	localFrameB:    b3.Transform,
	referenceFrame: b3.Transform,
	jointType:      b3.JointType,
	swingLimit:     f32,
	twistLimit:     b3.Vec2,
	jointFriction:  f32,
	parentIndex:    i32,
}

FILTER_JOINT_COUNT :: 9

// This must be zero initialized
Human :: struct {
	bones:            [bone_count]Bone,
	filterJoints:     [FILTER_JOINT_COUNT]b3.JointId,
	filterJointCount: i32,
	frictionTorque:   f32,
	isSpawned:        bool,
}

CreateHuman :: proc(
	human: ^Human,
	worldId: b3.WorldId,
	position: b3.Pos,
	frictionTorque: f32,
	hertz: f32,
	dampingRatio: f32,
	groupIndex: i32,
	userData: rawptr,
	colorize: bool,
) {
	assert(human.isSpawned == false)

	for i: i32 = 0; i < bone_count; i += 1 {
		human.bones[i].bodyId = b3.nullBodyId
		human.bones[i].anchorId = b3.nullBodyId
		human.bones[i].jointId = b3.nullJointId
		human.bones[i].jointFriction = 1.0
		human.bones[i].parentIndex = -1
	}

	for i: i32 = 0; i < FILTER_JOINT_COUNT; i += 1 {
		human.filterJoints[i] = b3.nullJointId
	}

	human.frictionTorque = frictionTorque

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.userData = userData

	shapeDef := b3.DefaultShapeDef()
	defaultDensity := shapeDef.density

	shirtColor := u32(b3.HexColor.colorMediumTurquoise)
	pantColor := u32(b3.HexColor.colorDodgerBlue)
	skinColors := [4]u32 {
		u32(b3.HexColor.colorNavajoWhite),
		u32(b3.HexColor.colorLightYellow),
		u32(b3.HexColor.colorPeru),
		u32(b3.HexColor.colorTan),
	}
	skinColor := skinColors[groupIndex % 4]

	{
		bone := &human.bones[BoneId.bone_pelvis]
		bone.parentIndex = -1

		bodyDef.name = "pelvis"
		bone.referenceFrame = b3.Transform{{0.000000, 0.996219, -0.023868}, {{1.000000, 0.000000, 0.000000}, 0.000000}}
		bodyDef.rotation = bone.referenceFrame.q
		bodyDef.position = b3.OffsetPos(position, bone.referenceFrame.p)
		bone.bodyId = b3.CreateBody(worldId, &bodyDef)

		shapeDef.filter.groupIndex = 0
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0

		shapeDef.baseMaterial.friction = 0.6
		shapeDef.baseMaterial.rollingResistance = 0.1
		capsule := b3.Capsule{{0.040000, 0.000001, 0.000000}, {-0.040000, -0.000001, 0.000000}, 0.150000}
		b3.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)
	}

	{
		bone := &human.bones[BoneId.bone_spine_01]
		bone.parentIndex = i32(BoneId.bone_pelvis)

		bodyDef.name = "spine_01"
		bone.referenceFrame = b3.Transform{{0.000000, 1.017288, -0.024882}, {{1.000000, 0.000000, 0.000000}, 0.000000}}
		bodyDef.rotation = bone.referenceFrame.q
		bodyDef.position = b3.OffsetPos(position, bone.referenceFrame.p)
		bone.bodyId = b3.CreateBody(worldId, &bodyDef)

		shapeDef.filter.groupIndex = -groupIndex
		shapeDef.baseMaterial.customColor = colorize ? shirtColor : 0

		shapeDef.baseMaterial.friction = 0.5
		shapeDef.baseMaterial.rollingResistance = 0.1
		capsule := b3.Capsule{{0.029876, -0.146581, 0.006260}, {-0.029876, -0.146574, 0.006260}, 0.145663}
		b3.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		bone.jointType = .sphericalJoint
		bone.localFrameA = b3.Transform{{0.000000, -0.021069, 0.001014}, {{-0.642737, 0.000000, 0.000000}, -0.766087}}
		bone.localFrameB = b3.Transform{{0.0, 0.0, 0.0}, {{-0.707107, 0.000000, 0.000000}, -0.707107}}
		bone.swingLimit = 35.0 * b3.DEG_TO_RAD
		bone.twistLimit = b3.Vec2{-17.5 * b3.DEG_TO_RAD, 17.5 * b3.DEG_TO_RAD}
	}

	{
		bone := &human.bones[BoneId.bone_spine_03]
		bone.parentIndex = i32(BoneId.bone_spine_01)

		bodyDef.name = "spine_03"
		bone.referenceFrame = b3.Transform{{0.000000, 1.267766, -0.022320}, {{1.000000, 0.000000, 0.000000}, 0.000000}}
		bodyDef.rotation = bone.referenceFrame.q
		bodyDef.position = b3.OffsetPos(position, bone.referenceFrame.p)
		bone.bodyId = b3.CreateBody(worldId, &bodyDef)

		shapeDef.filter.groupIndex = 0
		shapeDef.baseMaterial.customColor = colorize ? shirtColor : 0

		shapeDef.baseMaterial.friction = 0.5
		shapeDef.baseMaterial.rollingResistance = 0.1
		capsule := b3.Capsule{{0.063996, -0.117434, -0.040199}, {-0.063996, -0.117432, -0.040199}, 0.165004}
		b3.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		bone.jointType = .sphericalJoint
		bone.localFrameA = b3.Transform{{0.000000, -0.250478, -0.002562}, {{-0.642766, 0.000000, 0.000000}, -0.766063}}
		bone.localFrameB = b3.Transform{{0.0, 0.0, 0.0}, {{-0.707107, 0.000000, 0.000000}, -0.707107}}
		bone.swingLimit = 35.0 * b3.DEG_TO_RAD
		bone.twistLimit = b3.Vec2{-17.5 * b3.DEG_TO_RAD, 17.5 * b3.DEG_TO_RAD}
	}

	{
		bone := &human.bones[BoneId.bone_neck]
		bone.parentIndex = i32(BoneId.bone_spine_03)

		bodyDef.name = "neck_01"
		bone.referenceFrame = b3.Transform{{0.000000, 1.498783, 0.024462}, {{0.987879, 0.000000, 0.000000}, 0.155228}}
		bodyDef.rotation = bone.referenceFrame.q
		bodyDef.position = b3.OffsetPos(position, bone.referenceFrame.p)
		bone.bodyId = b3.CreateBody(worldId, &bodyDef)

		shapeDef.filter.groupIndex = 0
		shapeDef.baseMaterial.customColor = colorize ? skinColor : 0

		shapeDef.baseMaterial.friction = 0.2
		shapeDef.baseMaterial.rollingResistance = 0.05
		head := b3.Capsule{{0.000000, -0.212340, 0.000000}, {0.000000, -0.087340, 0.000000}, 0.110000}
		b3.CreateCapsuleShape(bone.bodyId, &shapeDef, &head)

		bone.jointType = .sphericalJoint
		bone.localFrameA = b3.Transform {
			{0.000000, -0.231017, -0.046781},
			{{-0.516157, -0.000002, 0.000003}, -0.856494},
		}
		bone.localFrameB = b3.Transform{{0.0, 0.0, 0.0}, {{-0.707108, -0.000002, 0.000002}, -0.707106}}
		bone.swingLimit = 30.0 * b3.DEG_TO_RAD
		bone.twistLimit = b3.Vec2{-17.5 * b3.DEG_TO_RAD, 17.5 * b3.DEG_TO_RAD}
	}

	{
		bone := &human.bones[BoneId.bone_thigh_l]
		bone.parentIndex = i32(BoneId.bone_pelvis)

		bodyDef.name = "thigh_l"
		bone.referenceFrame = b3.Transform {
			{0.092175, 0.971562, -0.011177},
			{{0.471412, 0.529510, -0.499888}, -0.497495},
		}
		bodyDef.rotation = bone.referenceFrame.q
		bodyDef.position = b3.OffsetPos(position, bone.referenceFrame.p)
		bone.bodyId = b3.CreateBody(worldId, &bodyDef)

		shapeDef.filter.groupIndex = -groupIndex
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0

		shapeDef.baseMaterial.friction = 0.6
		shapeDef.baseMaterial.rollingResistance = 0.1
		capsule := b3.Capsule{{-0.047269, 0.000001, 0.000000}, {-0.379769, 0.000005, 0.000000}, 0.091539}
		b3.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		bone.jointType = .sphericalJoint
		bone.localFrameA = b3.Transform{{0.092175, 0.024656, -0.012691}, {{-0.460186, -0.365368, -0.637662}, 0.498118}}
		bone.localFrameB = b3.Transform{{0.0, 0.0, 0.0}, {{-0.499997, -0.500003, 0.500002}, 0.499998}}
		bone.swingLimit = 30.0 * b3.DEG_TO_RAD
		bone.twistLimit = b3.Vec2{-10.0 * b3.DEG_TO_RAD, 10.0 * b3.DEG_TO_RAD}
	}

	{
		bone := &human.bones[BoneId.bone_calf_l]
		bone.parentIndex = i32(BoneId.bone_thigh_l)

		bodyDef.name = "calf_l"
		bone.referenceFrame = b3.Transform {
			{0.118720, 0.534541, -0.035362},
			{{0.521188, 0.480597, -0.546380}, -0.445935},
		}
		bodyDef.rotation = bone.referenceFrame.q
		bodyDef.position = b3.OffsetPos(position, bone.referenceFrame.p)
		bone.bodyId = b3.CreateBody(worldId, &bodyDef)

		shapeDef.filter.groupIndex = 0
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0

		shapeDef.baseMaterial.friction = 0.1
		shapeDef.baseMaterial.rollingResistance = 0.0
		shin := b3.Capsule{{-0.445000, 0.000001, 0.000000}, {0.005000, -0.000001, 0.000000}, 0.080000}
		b3.CreateCapsuleShape(bone.bodyId, &shapeDef, &shin)

		shapeDef.density = 0.5 * defaultDensity
		foot := b3.Capsule{{-0.456371, 0.129321, 0.000013}, {-0.359977, 0.014394, 0.000013}, 0.070000}
		b3.CreateCapsuleShape(bone.bodyId, &shapeDef, &foot)
		shapeDef.density = defaultDensity

		bone.jointType = .revoluteJoint
		bone.localFrameA = b3.Transform{{-0.438494, -0.000175, 0.000000}, {{0.344866, -0.938652, 0.000002}, -0.000008}}
		bone.localFrameB = b3.Transform{{0.0, 0.0, 0.0}, {{-0.008182, -0.999967, 0.000001}, -0.000007}}
		bone.twistLimit = b3.Vec2{-30.0 * b3.DEG_TO_RAD, 30.0 * b3.DEG_TO_RAD}
	}

	{
		bone := &human.bones[BoneId.bone_thigh_r]
		bone.parentIndex = i32(BoneId.bone_pelvis)

		bodyDef.name = "thigh_r"
		bone.referenceFrame = b3.Transform {
			{-0.092175, 0.971562, -0.011177},
			{{-0.497495, 0.499888, 0.529510}, -0.471412},
		}
		bodyDef.rotation = bone.referenceFrame.q
		bodyDef.position = b3.OffsetPos(position, bone.referenceFrame.p)
		bone.bodyId = b3.CreateBody(worldId, &bodyDef)

		shapeDef.filter.groupIndex = -groupIndex
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0

		shapeDef.baseMaterial.friction = 0.6
		shapeDef.baseMaterial.rollingResistance = 0.1
		capsule := b3.Capsule{{0.047269, -0.000001, 0.000000}, {0.379769, -0.000005, 0.000000}, 0.092587}
		b3.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		bone.jointType = .sphericalJoint
		bone.localFrameA = b3.Transform{{-0.092175, 0.024656, -0.012691}, {{0.380666, 0.491846, 0.488644}, -0.611889}}
		bone.localFrameB = b3.Transform{{0.0, 0.0, 0.0}, {{-0.500002, 0.499998, -0.499998}, 0.500002}}
		bone.swingLimit = 30.0 * b3.DEG_TO_RAD
		bone.twistLimit = b3.Vec2{-10.0 * b3.DEG_TO_RAD, 10.0 * b3.DEG_TO_RAD}
	}

	{
		bone := &human.bones[BoneId.bone_calf_r]
		bone.parentIndex = i32(BoneId.bone_thigh_r)

		bodyDef.name = "calf_r"
		bone.referenceFrame = b3.Transform {
			{-0.118720, 0.534541, -0.035362},
			{{-0.445935, 0.546380, 0.480597}, -0.521188},
		}
		bodyDef.rotation = bone.referenceFrame.q
		bodyDef.position = b3.OffsetPos(position, bone.referenceFrame.p)
		bone.bodyId = b3.CreateBody(worldId, &bodyDef)

		shapeDef.filter.groupIndex = 0
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0

		shapeDef.baseMaterial.friction = 0.1
		shapeDef.baseMaterial.rollingResistance = 0.0
		shin := b3.Capsule{{0.445000, -0.000002, 0.000003}, {-0.005000, 0.000000, 0.000003}, 0.080000}
		b3.CreateCapsuleShape(bone.bodyId, &shapeDef, &shin)

		shapeDef.density = 0.5 * defaultDensity
		foot := b3.Capsule{{0.369113, -0.006830, 0.000017}, {0.465507, -0.121757, 0.000017}, 0.070000}
		b3.CreateCapsuleShape(bone.bodyId, &shapeDef, &foot)
		shapeDef.density = defaultDensity

		bone.jointType = .revoluteJoint
		bone.localFrameA = b3.Transform{{0.438494, 0.000175, 0.000000}, {{0.000007, 0.000005, -0.344860}, -0.938654}}
		bone.localFrameB = b3.Transform{{0.0, 0.0, 0.0}, {{0.000006, 0.000005, 0.008188}, -0.999966}}
		bone.twistLimit = b3.Vec2{-30.0 * b3.DEG_TO_RAD, 30.0 * b3.DEG_TO_RAD}
	}

	{
		bone := &human.bones[BoneId.bone_upper_arm_l]
		bone.parentIndex = i32(BoneId.bone_spine_03)

		bodyDef.name = "upperarm_l"
		bone.referenceFrame = b3.Transform {
			{0.185817, 1.443630, 0.031306},
			{{0.560220, -0.307179, 0.366233}, -0.676512},
		}
		bodyDef.rotation = bone.referenceFrame.q
		bodyDef.position = b3.OffsetPos(position, bone.referenceFrame.p)
		bone.bodyId = b3.CreateBody(worldId, &bodyDef)

		shapeDef.filter.groupIndex = 0
		shapeDef.baseMaterial.customColor = colorize ? shirtColor : 0

		shapeDef.baseMaterial.friction = 0.6
		shapeDef.baseMaterial.rollingResistance = 0.1
		capsule := b3.Capsule{{0.296038, 0.004386, -0.000794}, {-0.016086, 0.004384, -0.016127}, 0.065000}
		b3.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		bone.jointType = .sphericalJoint
		bone.localFrameA = b3.Transform {
			{0.185817, -0.175865, -0.053625},
			{{-0.559466, -0.213784, -0.670199}, 0.438323},
		}
		bone.localFrameB = b3.Transform{{0.0, 0.0, 0.0}, {{-0.500953, -0.502617, -0.509318}, -0.486844}}
		bone.swingLimit = 45.0 * b3.DEG_TO_RAD
		bone.twistLimit = b3.Vec2{-20.0 * b3.DEG_TO_RAD, 20.0 * b3.DEG_TO_RAD}
		bone.jointFriction = 0.8
	}

	{
		bone := &human.bones[BoneId.bone_lower_arm_l]
		bone.parentIndex = i32(BoneId.bone_upper_arm_l)

		bodyDef.name = "lowerarm_l"
		bone.referenceFrame = b3.Transform {
			{0.347683, 1.193455, 0.029378},
			{{0.406736, -0.493012, 0.089135}, -0.763911},
		}
		bodyDef.rotation = bone.referenceFrame.q
		bodyDef.position = b3.OffsetPos(position, bone.referenceFrame.p)
		bone.bodyId = b3.CreateBody(worldId, &bodyDef)

		shapeDef.filter.groupIndex = 0
		shapeDef.baseMaterial.customColor = colorize ? skinColor : 0

		shapeDef.baseMaterial.friction = 0.3
		shapeDef.baseMaterial.rollingResistance = 0.1
		capsule := b3.Capsule{{0.312329, -0.008256, -0.016454}, {0.033153, 0.008259, -0.002741}, 0.060000}
		b3.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		bone.jointType = .revoluteJoint
		bone.localFrameA = b3.Transform{{0.297979, 0.000361, 0.000000}, {{0.916726, 0.399352, 0.000839}, -0.011456}}
		bone.localFrameB = b3.Transform{{0.0, 0.0, 0.0}, {{0.999922, -0.003857, 0.004659}, -0.010908}}
		bone.twistLimit = b3.Vec2{-35.0 * b3.DEG_TO_RAD, 35.0 * b3.DEG_TO_RAD}
		bone.jointFriction = 0.06
	}

	{
		bone := &human.bones[BoneId.bone_upper_arm_r]
		bone.parentIndex = i32(BoneId.bone_spine_03)

		bodyDef.name = "upperarm_r"
		bone.referenceFrame = b3.Transform{{-0.185817, 1.443630, 0.031306}, {{0.676512, 0.366233, 0.307179}, 0.560220}}
		bodyDef.rotation = bone.referenceFrame.q
		bodyDef.position = b3.OffsetPos(position, bone.referenceFrame.p)
		bone.bodyId = b3.CreateBody(worldId, &bodyDef)

		shapeDef.filter.groupIndex = 0
		shapeDef.baseMaterial.customColor = colorize ? shirtColor : 0

		shapeDef.baseMaterial.friction = 0.6
		shapeDef.baseMaterial.rollingResistance = 0.1
		capsule := b3.Capsule{{-0.300237, -0.000001, -0.000319}, {0.011887, 0.000001, 0.015013}, 0.065000}
		b3.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		bone.jointType = .sphericalJoint
		bone.localFrameA = b3.Transform{{-0.185817, -0.175865, -0.053625}, {{0.213793, 0.559449, 0.438334}, -0.670204}}
		bone.localFrameB = b3.Transform{{0.0, 0.0, 0.0}, {{-0.509319, 0.486843, 0.500954}, -0.502617}}
		bone.swingLimit = 45.0 * b3.DEG_TO_RAD
		bone.twistLimit = b3.Vec2{-20.0 * b3.DEG_TO_RAD, 20.0 * b3.DEG_TO_RAD}
		bone.jointFriction = 0.8
	}

	{
		bone := &human.bones[BoneId.bone_lower_arm_r]
		bone.parentIndex = i32(BoneId.bone_upper_arm_r)

		bodyDef.name = "lowerarm_r"
		bone.referenceFrame = b3.Transform{{-0.347683, 1.193455, 0.029378}, {{0.763911, 0.089135, 0.493012}, 0.406736}}
		bodyDef.rotation = bone.referenceFrame.q
		bodyDef.position = b3.OffsetPos(position, bone.referenceFrame.p)
		bone.bodyId = b3.CreateBody(worldId, &bodyDef)

		shapeDef.filter.groupIndex = 0
		shapeDef.baseMaterial.customColor = colorize ? skinColor : 0

		shapeDef.baseMaterial.friction = 0.3
		shapeDef.baseMaterial.rollingResistance = 0.1
		capsule := b3.Capsule{{-0.312649, -0.000003, 0.016471}, {-0.032986, 0.000000, 0.002732}, 0.060000}
		b3.CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule)

		bone.jointType = .revoluteJoint
		bone.localFrameA = b3.Transform{{-0.297979, -0.000361, 0.000000}, {{-0.405444, 0.914059, 0.010541}, 0.000852}}
		bone.localFrameB = b3.Transform{{0.0, 0.0, 0.0}, {{-0.002794, 0.999936, 0.010042}, 0.004364}}
		bone.twistLimit = b3.Vec2{-35.0 * b3.DEG_TO_RAD, 35.0 * b3.DEG_TO_RAD}
		bone.jointFriction = 0.06
	}

	for i: i32 = 1; i < bone_count; i += 1 {
		bone := &human.bones[i]
		parent := &human.bones[bone.parentIndex]

		bodyIdA := parent.bodyId
		bodyIdB := bone.bodyId

		bone.localFrameA.q = b3.NormalizeQuat(bone.localFrameA.q)
		bone.localFrameB.q = b3.NormalizeQuat(bone.localFrameB.q)

		if bone.jointType == .revoluteJoint {
			jointDef := b3.DefaultRevoluteJointDef()
			jointDef.base.bodyIdA = bodyIdA
			jointDef.base.bodyIdB = bodyIdB
			jointDef.base.localFrameA = bone.localFrameA
			jointDef.base.localFrameB = bone.localFrameB
			jointDef.enableLimit = true
			jointDef.lowerAngle = bone.twistLimit.x
			jointDef.upperAngle = bone.twistLimit.y
			jointDef.enableSpring = hertz > 0.0
			jointDef.hertz = hertz
			jointDef.dampingRatio = dampingRatio
			jointDef.enableMotor = true
			jointDef.maxMotorTorque = bone.jointFriction * frictionTorque
			bone.jointId = b3.CreateRevoluteJoint(worldId, &jointDef)
		} else if bone.jointType == .sphericalJoint {
			jointDef := b3.DefaultSphericalJointDef()
			jointDef.base.bodyIdA = bodyIdA
			jointDef.base.bodyIdB = bodyIdB
			jointDef.base.localFrameA = bone.localFrameA
			jointDef.base.localFrameB = bone.localFrameB
			jointDef.enableConeLimit = true
			jointDef.coneAngle = bone.swingLimit
			jointDef.enableTwistLimit = true
			jointDef.lowerTwistAngle = bone.twistLimit.x
			jointDef.upperTwistAngle = bone.twistLimit.y
			jointDef.enableSpring = hertz > 0.0
			jointDef.hertz = hertz
			jointDef.dampingRatio = dampingRatio
			jointDef.enableMotor = true
			jointDef.maxMotorTorque = bone.jointFriction * frictionTorque
			bone.jointId = b3.CreateSphericalJoint(worldId, &jointDef)
		}
	}

	// Disable some collisions
	human.filterJointCount = 0
	filterDef := b3.DefaultFilterJointDef()
	filterDef.base.bodyIdA = human.bones[BoneId.bone_thigh_l].bodyId
	filterDef.base.bodyIdB = human.bones[BoneId.bone_thigh_r].bodyId
	human.filterJoints[human.filterJointCount] = b3.CreateFilterJoint(worldId, &filterDef)
	human.filterJointCount += 1
	filterDef.base.bodyIdA = human.bones[BoneId.bone_neck].bodyId
	filterDef.base.bodyIdB = human.bones[BoneId.bone_upper_arm_l].bodyId
	human.filterJoints[human.filterJointCount] = b3.CreateFilterJoint(worldId, &filterDef)
	human.filterJointCount += 1
	filterDef.base.bodyIdA = human.bones[BoneId.bone_neck].bodyId
	filterDef.base.bodyIdB = human.bones[BoneId.bone_upper_arm_r].bodyId
	human.filterJoints[human.filterJointCount] = b3.CreateFilterJoint(worldId, &filterDef)
	human.filterJointCount += 1

	human.isSpawned = true
}

DestroyHuman :: proc(human: ^Human) {
	assert(human.isSpawned == true)

	for i: i32 = 0; i < human.filterJointCount; i += 1 {
		b3.DestroyJoint(human.filterJoints[i], false)
		human.filterJoints[i] = b3.nullJointId
	}

	for i: i32 = 0; i < bone_count; i += 1 {
		if b3.IS_NULL(human.bones[i].jointId) {
			continue
		}

		b3.DestroyJoint(human.bones[i].jointId, false)
		human.bones[i].jointId = b3.nullJointId
	}

	for i: i32 = 0; i < bone_count; i += 1 {
		if b3.IS_NULL(human.bones[i].bodyId) {
			continue
		}

		b3.DestroyBody(human.bones[i].bodyId)
		human.bones[i].bodyId = b3.nullBodyId
	}

	human.isSpawned = false
}

Human_SetVelocity :: proc(human: ^Human, velocity: b3.Vec3) {
	for i: i32 = 0; i < bone_count; i += 1 {
		bodyId := human.bones[i].bodyId

		if b3.IS_NULL(bodyId) {
			continue
		}

		b3.Body_SetLinearVelocity(bodyId, velocity)
	}
}

Human_ApplyRandomAngularImpulse :: proc(human: ^Human, magnitude: f32) {
	assert(human.isSpawned == true)
	range := b3.Vec3{magnitude, magnitude, magnitude}
	impulse := RandomVec3(b3.Neg(range), range)
	b3.Body_ApplyAngularImpulse(human.bones[BoneId.bone_spine_01].bodyId, impulse, true)
}

Human_SetJointFrictionTorque :: proc(human: ^Human, torque: f32) {
	assert(human.isSpawned == true)
	human.frictionTorque = torque

	for i: i32 = 1; i < bone_count; i += 1 {
		bone := &human.bones[i]
		if bone.jointType == .revoluteJoint {
			b3.RevoluteJoint_SetMaxMotorTorque(bone.jointId, bone.jointFriction * torque)
		} else {
			b3.SphericalJoint_SetMaxMotorTorque(bone.jointId, bone.jointFriction * torque)
		}
	}
}

Human_SetJointSpringHertz :: proc(human: ^Human, hertz: f32) {
	assert(human.isSpawned == true)
	for i: i32 = 1; i < bone_count; i += 1 {
		bone := &human.bones[i]
		if bone.jointType == .revoluteJoint {
			b3.RevoluteJoint_SetSpringHertz(bone.jointId, hertz)
		} else {
			b3.SphericalJoint_SetSpringHertz(bone.jointId, hertz)
		}
	}
}

Human_SetJointDampingRatio :: proc(human: ^Human, dampingRatio: f32) {
	assert(human.isSpawned == true)
	for i: i32 = 1; i < bone_count; i += 1 {
		bone := &human.bones[i]
		if bone.jointType == .revoluteJoint {
			b3.RevoluteJoint_SetSpringDampingRatio(bone.jointId, dampingRatio)
		} else {
			b3.SphericalJoint_SetSpringDampingRatio(bone.jointId, dampingRatio)
		}
	}
}

Human_AlignSpring :: proc(human: ^Human, worldId: b3.WorldId, groundId: b3.BodyId, hertz: f32, dampingRatio: f32) {
	assert(human.isSpawned == true)

	bone := &human.bones[BoneId.bone_pelvis]
	assert(b3.IS_NULL(bone.jointId) == true)
	q := b3.ComputeQuatBetweenUnitVectors(b3.Vec3_axisZ, b3.Vec3_axisY)
	qb := b3.Body_GetRotation(bone.bodyId)

	jointDef := b3.DefaultParallelJointDef()
	jointDef.base.bodyIdA = groundId
	jointDef.base.bodyIdB = bone.bodyId
	jointDef.base.localFrameA.q = q
	jointDef.base.localFrameB.q = b3.InvMulQuat(qb, q)
	jointDef.base.drawScale = 2.0
	jointDef.base.collideConnected = true
	jointDef.hertz = hertz
	jointDef.dampingRatio = dampingRatio

	bone.jointId = b3.CreateParallelJoint(worldId, &jointDef)
}

Human_CreateMotorAnchors :: proc(human: ^Human, worldId: b3.WorldId) {
	anchorDef := b3.DefaultBodyDef()
	anchorDef.type = .kinematicBody

	motorDef := b3.DefaultMotorJointDef()
	motorDef.angularHertz = 5.0
	motorDef.angularDampingRatio = 1.0
	motorDef.linearHertz = 5.0
	motorDef.linearDampingRatio = 1.0
	motorDef.maxSpringForce = b3.FLT_MAX
	motorDef.maxSpringTorque = b3.FLT_MAX

	for i: i32 = 0; i < bone_count; i += 1 {
		bone := &human.bones[i]

		bodyTransform := b3.Body_GetTransform(bone.bodyId)
		anchorDef.position = bodyTransform.p
		anchorDef.rotation = bodyTransform.q
		bone.anchorId = b3.CreateBody(worldId, &anchorDef)

		motorDef.base.bodyIdA = bone.anchorId
		motorDef.base.bodyIdB = bone.bodyId

		bone.anchorJointId = b3.CreateMotorJoint(worldId, &motorDef)
	}
}

Human_CreateParallelAnchors :: proc(human: ^Human, worldId: b3.WorldId) {
	anchorDef := b3.DefaultBodyDef()
	anchorDef.type = .kinematicBody

	qFrameWorld := b3.ComputeQuatBetweenUnitVectors(b3.Vec3_axisZ, b3.Vec3_axisY)
	jointDef := b3.DefaultParallelJointDef()
	jointDef.hertz = 8.0
	jointDef.dampingRatio = 1.0
	jointDef.maxTorque = 800.0

	for i: i32 = 0; i < bone_count; i += 1 {
		bone := &human.bones[i]

		bodyTransform := b3.Body_GetTransform(bone.bodyId)
		anchorDef.position = bodyTransform.p
		anchorDef.rotation = bodyTransform.q
		bone.anchorId = b3.CreateBody(worldId, &anchorDef)

		jointDef.base.bodyIdA = bone.anchorId
		jointDef.base.bodyIdB = bone.bodyId

		frameQuat := b3.InvMulQuat(bodyTransform.q, qFrameWorld)
		jointDef.base.localFrameA.q = frameQuat
		jointDef.base.localFrameB.q = frameQuat

		bone.anchorJointId = b3.CreateParallelJoint(worldId, &jointDef)
	}
}

Human_SetBullet :: proc(human: ^Human, flag: bool) {
	for i: i32 = 0; i < bone_count; i += 1 {
		bone := &human.bones[i]
		b3.Body_SetBullet(bone.bodyId, flag)
	}
}

// unified: not ported: the disabled (#if 0) pose control and motor helpers.
