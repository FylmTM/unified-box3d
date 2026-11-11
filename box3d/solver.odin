// Ported from box3d src/solver.c and src/solver.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:intrinsics"
import "base:runtime"
import "core:fmt"

// solver.h

// Solver work is partitioned into fixed-size blocks that worker threads claim
// in parallel via atomic CAS on a per-block syncIndex. The descriptor (b3SolverBlock)
// and the atomic counter sit in a wrapping b3SyncBlock so the CAS-winner can
// pass the descriptor by value into stage tasks without aliasing the atomic
// memory other threads are CAS-writing. Three properties of this design
// matter for performance:
//
// 1. Distributed contention. Per-block atomic syncIndex avoids the cache line stampede
//    that a single shared fetch_add counter would cause. Once a worker
//    settles into a block range, its CAS targets live in its own L1.
//
// 2. Monotonic syncIndex across iterations. Iterative stages (warm start,
//    solve, relax) reuse the same block array every sub-step iteration.
//    syncIndex grows each iteration; workers CAS (prev, prev+1), so the
//    main thread never touches any per-block state between iterations.
//    Non-iterative stages simply use syncIndex 1.
//
// 3. L2 affinity across iterations. Each worker picks a start offset from
//    its workerIndex, then scans forward and (after wrap) backward:
//
//      blocks:   [0] [1] [2] [3] [4] [5] [6] [7]
//                 ^           ^           ^   ^
//                 W0          W1          W2  W3   <- start offsets
//
//    W0 claims 0,1,2,3 (forward), W1 claims 4,5, etc. Under balanced load
//    each worker re-hits the same block range every iteration, keeping that
//    range's hot data resident in its L2. A failed CAS means a neighbour
//    already claimed the block, so the stealing worker stops -- preserving
//    locality under mild imbalance while still draining the queue.
//
// A graph color stage lays out joint blocks first, then contact blocks:
//
//      stage->blocks ->
//        +------+------+------+------+------+------+------+
//        |  J0  |  J1  |  J2  |  C0  |  C1  |  C2  |  C3  |
//        +------+------+------+------+------+------+------+
//        <-- graphJointBlocks --><---- graphContactBlocks ---->
//
// Each block carries its type so the dispatcher routes J-blocks to the joint
// solver and C-blocks to the SIMD contact solver; both kinds run concurrently
// within the stage -- no barrier between them. The type tag lives on the
// block (not the stage) so that mixed-type stages can keep the concurrency.
//
// The solver threading model is inspired by https://github.com/bepu/bepuphysics2

// Solver stages
SolverStageType :: enum i32 {
	stagePrepareJoints,
	stagePrepareWideContacts,
	stagePrepareContacts,
	stageIntegrateVelocities,
	stageWarmStart,
	stageSolve,
	stageIntegratePositions,
	stageRelax,
	stageRestitution,
	stageStoreWideImpulses,
	stageStoreImpulses,
}

SolverBlockType :: enum i32 {
	// Block for iterating across bodies.
	bodyBlock,

	// Block for iterating across joints. For prepare.
	jointBlock,

	// Block for iterating across wide contacts. For prepare and store.
	wideContactBlock,

	// Block for iterating across contacts. For prepare and store.
	contactBlock,

	// Block for iterating across joints of a single graph color.
	graphJointBlock,

	// Block for iterating across wide contacts of a single graph color.
	graphWideContactBlock,

	// Block for iterating across contacts of a single graph color.
	graphContactBlock,

	// Block for processing overflow constraints
	overflowBlock,
}

// Solver block describes a multithreaded unit of work.
SolverBlock :: struct {
	startIndex: i32,
	count:      u16,
	// b3SolverBlockType
	blockType:  u8,
	colorIndex: u8,
}

// A unit of multithreaded work along with atomic synchronization. The syncIndex grows
// monotonically allowing the solver block to be re-used across sub-steps.
SyncBlock :: struct {
	block:     SolverBlock,
	syncIndex: AtomicInt,
}

// Each stage must be completed before going to the next stage.
// Non-iterative stages use a stage instance once while iterative stages re-use the same instance each iteration.
SolverStage :: struct {
	blocks:          [^]SyncBlock,
	type:            SolverStageType,
	blockCount:      i32,
	colorIndex:      u8,
	completionCount: AtomicInt,
}

// Constraint softness
Softness :: struct {
	biasRate:     f32,
	massScale:    f32,
	impulseScale: f32,
}

// Prepare/store run as a flat parallel-for over the whole wide-constraint
// range. Each span maps a slice of that range back to the owning color's
// contacts so workers can decode flat wide-slot indices without touching
// graph state. The spans array has one entry per active color plus a sentinel
// whose start == wideContactCount.
WidePrepareSpan :: struct {
	start:    i32,
	count:    i32,
	contacts: [^]i32,
}

ContactPrepareSpan :: struct {
	start:    i32,
	count:    i32,
	contacts: [^]ContactSpec,
}

MeshPrepareSpan :: struct {
	start:    i32,
	count:    i32,
	contacts: [^]ContactSpec,
	order:    [^]i32,
}

JointPrepareSpan :: struct {
	start:  i32,
	count:  i32,
	joints: [^]JointSim,
}

// Context for a time step. Recreated each time step.
StepContext :: struct {
	// time step
	dt:                     f32,

	// inverse time step (0 if dt == 0).
	inv_dt:                 f32,

	// sub-step
	h:                      f32,
	inv_h:                  f32,
	subStepCount:           i32,
	contactSoftness:        Softness,
	staticSoftness:         Softness,
	maxLinearVelocity:      f32,
	world:                  ^World,
	graph:                  ^ConstraintGraph,

	// shortcut to body states from awake set
	states:                 [^]BodyState,

	// shortcut to body sims from awake set
	sims:                   [^]BodySim,

	// array of all shape ids for shapes that have enlarged AABBs
	enlargedShapes:         [^]i32,
	enlargedShapeCount:     i32,

	// Array of bullet bodies that need continuous collision handling
	bulletBodies:           [^]i32,
	bulletBodyCount:        AtomicInt,
	anyRestitution:         AtomicInt,

	// Contact ids for simplified parallel-for access. Used in narrow-phase.
	// These contacts may or may not be touching. They are associated with awake bodies.
	awakeContactIndices:    [^]i32,

	// Flat view of the wide contact constraint array used by prepare and store.
	// prepareSpans has activeColorCount + 1 entries, the last being a sentinel
	// at wideContactCount. wideContactConstraints is the contiguous base
	// pointer. Per-color slices live at colors[i].wideConstraints.
	wideConstraints:        rawptr,
	widePrepareSpans:       [^]WidePrepareSpan,
	wideContactCount:       i32,

	// Similar for overflow contact constraints.
	overflowSpans:          [^]ContactPrepareSpan,
	wideMeshConstraints:    rawptr,
	wideMeshManifoldStarts: [^]i32,
	meshPrepareSpans:       [^]MeshPrepareSpan,
	jointPrepareSpans:      [^]JointPrepareSpan,
	activeColorCount:       i32,
	workerCount:            i32,
	stages:                 [^]SolverStage,
	stageCount:             i32,
	enableWarmStarting:     bool,

	// padding to prevent false sharing
	padding1:               [64]u8,

	// This atomic is central to multi-threaded solver task synchronization.
	// It prevents ABA problems by monotonically growing as the solver advances.
	// This means a delayed worker thread will catch up without repeating already completed
	// work (causing a race condition).
	// sync index (16-bits) | stage type (16-bits)
	atomicSyncBits:         AtomicU32,

	// padding to prevent false sharing
	padding2:               [64]u8,

	// Race flag claimed by whichever runner reaches b3SolverTask with workerIndex 0 first.
	// The calling thread of b3World_Step also races for this slot so the orchestrator can
	// always make progress, regardless of how the user's task system schedules tasks (out
	// of order, fewer threads than workers, or synchronously inside enqueueTaskFcn). The
	// loser of the race no-ops as workerIndex 0.
	mainClaimed:            AtomicInt,

	// padding to prevent false sharing
	padding3:               [64]u8,
}

MakeSoft :: proc(hertz: f32, zeta: f32, h: f32) -> Softness {
	if hertz == 0.0 {
		return Softness{biasRate = 0.0, massScale = 0.0, impulseScale = 0.0}
	}

	omega := f32(2.0) * PI * hertz
	a1 := 2.0 * zeta + h * omega
	a2 := h * omega * a1
	a3 := 1.0 / (1.0 + a2)

	// bias = w / (2 * z + hw)
	// massScale = hw * (2 * z + hw) / (1 + hw * (2 * z + hw))
	// impulseScale = 1 / (1 + hw * (2 * z + hw))

	// If z == 0
	// bias = 1/h
	// massScale = hw^2 / (1 + hw^2)
	// impulseScale = 1 / (1 + hw^2)

	// w -> inf
	// bias = 1/h
	// massScale = 1
	// impulseScale = 0

	// if w = pi / 4  * inv_h
	// massScale = (pi/4)^2 / (1 + (pi/4)^2) = pi^2 / (16 + pi^2) ~= 0.38
	// impulseScale = 1 / (1 + (pi/4)^2) = 16 / (16 + pi^2) ~= 0.62

	// In all cases:
	// massScale + impulseScale == 1

	return Softness{biasRate = omega / a1, massScale = a2 * a3, impulseScale = a3}
}

// solver.c

// these are useful for solver testing
@(private = "file")
ITERATIONS :: 1
@(private = "file")
RELAX_ITERATIONS :: 1

// unified: one Pause on Odin's cpu_relax (pause on x86, yield on arm), for the per-compiler inline asm variants
@(private = "file")
Pause :: #force_inline proc "contextless" () {
	intrinsics.cpu_relax()
}
// unified end

@(private = "file")
WorkerContext :: struct {
	context_:    ^StepContext,
	workerIndex: i32,
	userTask:    rawptr,
}

// Integrate velocities, apply damping, and gyroscopic torque
@(private = "file")
IntegrateVelocitiesTask :: proc(block: SolverBlock, context_: ^StepContext) {
	VALIDATE(
		block.startIndex + i32(block.count) <= context_.world.solverSets.data[i32(SetType.awakeSet)].bodyStates.count,
	)

	states := context_.states
	sims := context_.sims

	gravity := context_.world.gravity
	h := context_.h

	for i := block.startIndex; i < block.startIndex + i32(block.count); i += 1 {
		sim := &sims[i]
		state := &states[i]

		v := state.linearVelocity
		w := state.angularVelocity

		// Damping math
		// Differential equation: dv/dt + c * v = 0
		// Solution: v(t) = v0 * exp(-c * t)
		// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v(t) * exp(-c * dt)
		// v2 = exp(-c * dt) * v1
		// Pade approximation:
		// v2 = v1 * 1 / (1 + c * dt)
		linearDamping := 1.0 / (1.0 + h * sim.linearDamping)
		angularDamping := 1.0 / (1.0 + h * sim.angularDamping)

		// Gravity scale will be zero for kinematic bodies
		gravityScale := sim.invMass > 0.0 ? sim.gravityScale : 0.0

		linearVelocityDelta := Blend2(h * sim.invMass, sim.force, h * gravityScale, gravity)
		v = MulAdd(linearVelocityDelta, linearDamping, v)

		angularVelocityDelta := MulSV(h, MulMV(sim.invInertiaWorld, sim.torque))
		w = MulAdd(angularVelocityDelta, angularDamping, w)

		// Gyroscopic torque by solving this nonlinear equation using Newton-Raphson.
		// I * (w2 - w1) + h * cross(w2, I * w2) = 0
		// This is all done in local coordinates where the Jacobian is easier to compute.
		// This improves the simulation of long skinny bodies.
		// If the principal inertia values of I are roughly equal (isotropic) then
		// cross(w2, I * w2) ~= 0, so this can be skipped.
		if (sim.flags & enableGyroscopicTorque) != 0 {
			// Get current rotation.
			q0 := sim.transform.q
			q := MulQuat(state.deltaRotation, q0)

			// Compute local angular velocity
			omega1 := InvRotateVector(q, w)
			omega2 := omega1

			// Symmetric inertia tensor: 6 unique entries (column-major)
			i00 := sim.inertia.cx.x
			i01 := sim.inertia.cy.x
			i02 := sim.inertia.cz.x
			i11 := sim.inertia.cy.y
			i12 := sim.inertia.cz.y
			i22 := sim.inertia.cz.z

			for gyroIteration: i32 = 0; gyroIteration < GYROSCOPIC_ITERATIONS; gyroIteration += 1 {
				w1 := omega2.x
				w2 := omega2.y
				w3 := omega2.z

				// Iw = I * omega2 (shared between residual and Jacobian)
				Iw1 := i00 * w1 + i01 * w2 + i02 * w3
				Iw2 := i01 * w1 + i11 * w2 + i12 * w3
				Iw3 := i02 * w1 + i12 * w2 + i22 * w3

				// Residual: b = I * (omega2 - omega1) + h * cross(omega2, I * omega2)
				dw := Sub(omega2, omega1)
				b := Vec3 {
					i00 * dw.x + i01 * dw.y + i02 * dw.z + h * (w2 * Iw3 - w3 * Iw2),
					i01 * dw.x + i11 * dw.y + i12 * dw.z + h * (w3 * Iw1 - w1 * Iw3),
					i02 * dw.x + i12 * dw.y + i22 * dw.z + h * (w1 * Iw2 - w2 * Iw1),
				}

				// Jacobian J = I + h * (skew(omega2) * I - skew(I * omega2))
				// Jacobian derived by Erin Catto, Ph.D. Do not attempt to do this without a Ph.D.
				// Doubled inertia terms above fold into Iw, e.g. row 2 col 1: i00*w3 - i02*w1 - Iw3.
				J := Matrix3 {
					{
						i00 + h * (w2 * i02 - w3 * i01),
						i01 + h * (w3 * i00 - w1 * i02 - Iw3),
						i02 + h * (w1 * i01 - w2 * i00 + Iw2),
					},
					{
						i01 + h * (w2 * i12 - w3 * i11 + Iw3),
						i11 + h * (w3 * i01 - w1 * i12),
						i12 + h * (w1 * i11 - w2 * i01 - Iw1),
					},
					{
						i02 + h * (w2 * i22 - w3 * i12 - Iw2),
						i12 + h * (w3 * i02 - w1 * i22 + Iw1),
						i22 + h * (w1 * i12 - w2 * i02),
					},
				}

				omega2 = Sub(omega2, Solve3(J, b))
			}

			w = RotateVector(q, omega2)
		}

		state.linearVelocity = v
		state.angularVelocity = w
	}
}

@(private = "file")
IntegratePositionsTask :: proc(block: SolverBlock, context_: ^StepContext) {
	VALIDATE(
		block.startIndex + i32(block.count) <= context_.world.solverSets.data[i32(SetType.awakeSet)].bodyStates.count,
	)

	states := context_.states
	h := context_.h
	maxLinearSpeed := context_.maxLinearVelocity
	maxAngularSpeed := MAX_ROTATION * context_.inv_dt
	maxLinearSpeedSquared := maxLinearSpeed * maxLinearSpeed
	maxAngularSpeedSquared := maxAngularSpeed * maxAngularSpeed

	for i := block.startIndex; i < block.startIndex + i32(block.count); i += 1 {
		state := &states[i]

		v := state.linearVelocity
		w := state.angularVelocity

		// Motion locks - these can be viewed as a constraint that come last
		v.x = (state.flags & lockLinearX) != 0 ? 0.0 : v.x
		v.y = (state.flags & lockLinearY) != 0 ? 0.0 : v.y
		v.z = (state.flags & lockLinearZ) != 0 ? 0.0 : v.z
		w.x = (state.flags & lockAngularX) != 0 ? 0.0 : w.x
		w.y = (state.flags & lockAngularY) != 0 ? 0.0 : w.y
		w.z = (state.flags & lockAngularZ) != 0 ? 0.0 : w.z

		// Clamp to max linear speed
		if Dot(v, v) > maxLinearSpeedSquared {
			ratio := maxLinearSpeed / Length(v)
			v = MulSV(ratio, v)
			state.flags |= isSpeedCapped
		}

		// Clamp to max angular speed
		if Dot(w, w) > maxAngularSpeedSquared && (state.flags & allowFastRotation) == 0 {
			ratio := maxAngularSpeed / Length(w)
			w = MulSV(ratio, w)
			state.flags |= isSpeedCapped
		}

		state.linearVelocity = v
		state.angularVelocity = w
		state.deltaPosition = MulAdd(state.deltaPosition, h, v)
		state.deltaRotation = IntegrateRotation(state.deltaRotation, MulSV(h, w))
	}
}

@(private = "file")
PrepareJointsTask :: proc(block: SolverBlock, context_: ^StepContext) {
	spans := context_.jointPrepareSpans

	index := block.startIndex
	endIndex := block.startIndex + i32(block.count)

	// Find color for start index. Linear search but fast.
	colorIndex: i32 = 0
	for spans[colorIndex + 1].start <= index {
		colorIndex += 1
	}

	// Loop over block
	for index < endIndex {
		colorStart := spans[colorIndex].start
		colorEndIndex := MinInt(spans[colorIndex + 1].start, endIndex)
		joints := spans[colorIndex].joints

		// Loop over color
		for ; index < colorEndIndex; index += 1 {
			assert(0 <= index - colorStart && index - colorStart < spans[colorIndex].count)
			joint := &joints[index - colorStart]
			PrepareJoint(joint, context_)
		}

		// Advance to next color
		colorIndex += 1
	}
}

@(private = "file")
WarmStartJointsTask :: proc(block: SolverBlock, context_: ^StepContext) {
	color := &context_.graph.colors[block.colorIndex]
	joints := color.jointSims.data

	for i := block.startIndex; i < block.startIndex + i32(block.count); i += 1 {
		joint := &joints[i]
		WarmStartJoint(joint, context_)
	}
}

@(private = "file")
SolveJointsTask :: proc(block: SolverBlock, context_: ^StepContext, useBias: bool, workerIndex: i32) {
	color := &context_.graph.colors[block.colorIndex]
	joints := color.jointSims.data

	assert(0 <= block.startIndex && block.startIndex + i32(block.count) <= color.jointSims.count)

	jointStateBitSet := &context_.world.taskContexts.data[workerIndex].jointStateBitSet

	for i := block.startIndex; i < block.startIndex + i32(block.count); i += 1 {
		joint := &joints[i]
		SolveJoint(joint, context_, useBias)

		if useBias &&
		   (joint.forceThreshold < FLT_MAX || joint.torqueThreshold < FLT_MAX) &&
		   GetBit(jointStateBitSet, u32(joint.jointId)) == false {
			force, torque: f32
			GetJointReaction(context_.world, joint, context_.inv_h, &force, &torque)

			// Check thresholds. A zero threshold means all awake joints get reported.
			if force >= joint.forceThreshold || torque >= joint.torqueThreshold {
				// Flag this joint for processing.
				SetBit(jointStateBitSet, u32(joint.jointId))
			}
		}
	}
}

@(private = "file")
B2_MAX_CONTINUOUS_SENSOR_HITS :: 8

@(private = "file")
ContinuousContext :: struct {
	world:              ^World,
	fastBodySim:        ^BodySim,
	fastShape:          ^Shape,
	centroid1:          Vec3,
	centroid2:          Vec3,
	sweep:              Sweep,
	// World base for re-centering sweeps. Keeps TOI in float precision far from the origin.
	base:               Pos,
	fraction:           f32,
	sensorHits:         [B2_MAX_CONTINUOUS_SENSOR_HITS]SensorHit,
	sensorFractions:    [B2_MAX_CONTINUOUS_SENSOR_HITS]f32,
	sensorCount:        i32,
	distanceIterations: i32,
	pushBackIterations: i32,
	rootIterations:     i32,
}

// This is called from b3DynamicTree_Query for continuous collision
@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
ContinuousQueryCallback :: proc "c" (proxyId: i32, userData: u64, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	_ = proxyId

	shapeId := i32(userData)
	continuousContext := (^ContinuousContext)(context_)

	fastShape := continuousContext.fastShape
	fastBodySim := continuousContext.fastBodySim

	assert(fastShape.sensorIndex == NULL_INDEX)

	// Skip same shape
	if shapeId == fastShape.id {
		return true
	}

	world := continuousContext.world
	shape := Array_Get(&world.shapes, shapeId)

	// Skip same body
	if shape.bodyId == fastShape.bodyId {
		return true
	}

	// Skip sensors unless both shapes want sensor events
	isSensor := shape.sensorIndex != NULL_INDEX
	if isSensor && ((shape.flags & enableSensorEvents) == 0 || (fastShape.flags & enableSensorEvents) == 0) {
		return true
	}

	// Skip filtered shapes
	canCollide := ShouldShapesCollide(fastShape.filter, shape.filter)
	if canCollide == false {
		return true
	}

	body := Array_Get(&world.bodies, shape.bodyId)

	bodySim := GetBodySim(world, body)
	assert(body.type == .staticBody || (fastBodySim.flags & isBullet) != 0)

	// Skip bullets
	if (body.flags & isBullet) != 0 {
		return true
	}

	// Skip filtered bodies
	fastBody := Array_Get(&world.bodies, fastBodySim.bodyId)
	canCollide = ShouldBodiesCollide(world, fastBody, body)
	if canCollide == false {
		return true
	}

	// Custom user filtering
	if (shape.flags & enableCustomFiltering) != 0 || (fastShape.flags & enableCustomFiltering) != 0 {
		customFilterFcn := world.customFilterFcn
		if customFilterFcn != nil {
			idA := ShapeId{shape.id + 1, world.worldId, shape.generation}
			idB := ShapeId{fastShape.id + 1, world.worldId, fastShape.generation}
			canCollide = customFilterFcn(idA, idB, world.customFilterContext)
			if canCollide == false {
				return true
			}
		}
	}

	// todo does having a sweep on shapeA help with bullets?
	sweepA := MakeRelativeSweep(bodySim, continuousContext.base)

	// Time of impact versus shape. Supports all shape types
	output := ShapeTimeOfImpact(shape, fastShape, &sweepA, &continuousContext.sweep, continuousContext.fraction)
	if isSensor {
		// Only accept a sensor hit that is sooner than the current solid hit.
		if output.fraction <= continuousContext.fraction &&
		   continuousContext.sensorCount < B2_MAX_CONTINUOUS_SENSOR_HITS {
			index := continuousContext.sensorCount

			// The hit shape is a sensor
			sensorHit := SensorHit {
				sensorId  = shape.id,
				visitorId = fastShape.id,
			}

			continuousContext.sensorHits[index] = sensorHit
			continuousContext.sensorFractions[index] = output.fraction
			continuousContext.sensorCount += 1
		}
	} else if 0.0 < output.fraction && output.fraction < continuousContext.fraction {
		didHit := true

		if didHit && ((shape.flags & enablePreSolveEvents) != 0 || (fastShape.flags & enablePreSolveEvents) != 0) {
			shapeIdA := ShapeId{shape.id + 1, world.worldId, shape.generation}
			shapeIdB := ShapeId{fastShape.id + 1, world.worldId, fastShape.generation}
			point := OffsetPos(continuousContext.base, output.point)
			didHit = world.preSolveFcn(shapeIdA, shapeIdB, point, output.normal, world.preSolveContext)
		}

		if didHit {
			continuousContext.fraction = output.fraction
			continuousContext.distanceIterations = MaxInt(
				continuousContext.distanceIterations,
				output.distanceIterations,
			)
			continuousContext.pushBackIterations = MaxInt(
				continuousContext.pushBackIterations,
				output.pushBackIterations,
			)
			continuousContext.rootIterations = MaxInt(continuousContext.rootIterations, output.rootIterations)
		}
	}

	// Continue query
	return true
}

@(private = "file")
IsShapeFast :: proc(shape: ^Shape, centroid1: Vec3, centroid2: Vec3, rotationChord: f32, safetyFactor: f32) -> bool {
	radius: f32
	#partial switch shape.type {
	case .sphereShape:
		radius = shape.sphere.radius

	case .capsuleShape:
		radius = 0.5 * Distance(shape.capsule.center1, shape.capsule.center2) + shape.capsule.radius

	case .hullShape:
		{
			farthestPoint := FarthestPointOnAABB(shape.hull.aabb, shape.localCentroid)
			radius = Distance(farthestPoint, shape.localCentroid)
		}

	case:
		return true
	}

	minExtent := ComputeShapeMinExtent(shape, shape.localCentroid)
	maxMotion := Distance(centroid1, centroid2) + rotationChord * radius
	return maxMotion > safetyFactor * minExtent
}

// Continuous collision of dynamic versus static
@(private = "file")
SolveContinuous :: proc(world: ^World, bodySimIndex: i32, taskContext: ^TaskContext, dt: f32) {
	awakeSet := Array_Get(&world.solverSets, i32(SetType.awakeSet))
	fastBodySim := Array_Get(&awakeSet.bodySims, bodySimIndex)
	VALIDATE((fastBodySim.flags & isFast) != 0)

	// Re-center the sweep on the fast body so the TOI and the swept query stay in float precision
	base := fastBodySim.center0

	sweep := MakeRelativeSweep(fastBodySim, base)

	xf1: Transform
	xf1.q = sweep.q1
	xf1.p = Sub(sweep.c1, RotateVector(sweep.q1, sweep.localCenter))

	xf2: Transform
	xf2.q = sweep.q2
	xf2.p = Sub(sweep.c2, RotateVector(sweep.q2, sweep.localCenter))

	staticTree := &world.broadPhase.trees[i32(BodyType.staticBody)]
	kinematicTree := &world.broadPhase.trees[i32(BodyType.kinematicBody)]
	dynamicTree := &world.broadPhase.trees[i32(BodyType.dynamicBody)]
	fastBody := Array_Get(&world.bodies, fastBodySim.bodyId)

	context_: ContinuousContext = {}
	context_.world = world
	context_.sweep = sweep
	context_.base = base
	context_.fastBodySim = fastBodySim
	context_.fraction = 1.0

	isBullet := (fastBodySim.flags & isBullet) != 0
	safetyFactor := fastBody.safetyFactor
	rotationChord := 2.0 * Length(InvMulQuat(sweep.q1, sweep.q2).v)

	shapeId := fastBody.headShapeId
	for shapeId != NULL_INDEX {
		fastShape := Array_Get(&world.shapes, shapeId)
		shapeId = fastShape.nextShapeId

		context_.fastShape = fastShape
		context_.centroid1 = TransformPoint(xf1, fastShape.localCentroid)
		context_.centroid2 = TransformPoint(xf2, fastShape.localCentroid)

		box1 := fastShape.aabb
		// xf2 is relative to the base, so translate the box back to world space, rounding outward
		box2 := OffsetAABB(ComputeShapeAABB(fastShape, xf2), base)

		// Store this to avoid double computation in the case there is no impact event
		fastShape.aabb = box2

		// No continuous collision for meshes
		if fastShape.type == .meshShape || fastShape.type == .heightShape {
			continue
		}

		// No continuous collision for sensors
		if fastShape.sensorIndex != NULL_INDEX {
			continue
		}

		if isBullet == false {
			isShapeFast := IsShapeFast(fastShape, context_.centroid1, context_.centroid2, rotationChord, safetyFactor)
			if isShapeFast == false {
				continue
			}
		}

		sweptBox := AABB_Union(box1, box2)
		DynamicTree_Query(staticTree, sweptBox, DEFAULT_MASK_BITS, false, ContinuousQueryCallback, &context_)

		if isBullet {
			DynamicTree_Query(kinematicTree, sweptBox, DEFAULT_MASK_BITS, false, ContinuousQueryCallback, &context_)
			DynamicTree_Query(dynamicTree, sweptBox, DEFAULT_MASK_BITS, false, ContinuousQueryCallback, &context_)
		}
	}

	speculativeScalar := SPECULATIVE_DISTANCE

	if context_.fraction < 1.0 {
		// Handle time of impact event. The sweep is relative to the base, so re-add the base
		// to return the advanced pose to world space.
		q := NLerp(sweep.q1, sweep.q2, context_.fraction)
		c := Lerp(sweep.c1, sweep.c2, context_.fraction)
		origin := Sub(c, RotateVector(q, sweep.localCenter))

		// Advance body
		transform := WorldTransform{OffsetPos(base, origin), q}
		center := OffsetPos(base, c)
		fastBodySim.transform = transform
		fastBodySim.center = center
		fastBodySim.rotation0 = q
		fastBodySim.center0 = center
		fastBodySim.flags |= hadTimeOfImpact

		// Timeloss means there is a lost gravity contribution. Other forces and torques are ignored for now.
		fastBodyState := Array_Get(&awakeSet.bodyStates, bodySimIndex)
		v := fastBodyState.linearVelocity
		timeLoss := (1.0 - context_.fraction) * dt
		dv := MulSV(-timeLoss * fastBodySim.gravityScale, world.gravity)
		dv.x = (fastBodyState.flags & lockLinearX) != 0 ? 0.0 : dv.x
		dv.y = (fastBodyState.flags & lockLinearY) != 0 ? 0.0 : dv.y
		dv.z = (fastBodyState.flags & lockLinearZ) != 0 ? 0.0 : dv.z
		fastBodyState.linearVelocity = Add(v, dv)

		// The move event was written before CCD, so correct it with the impact pose
		event := Array_Get(&world.bodyMoveEvents, bodySimIndex)
		event.transform = fastBodySim.transform

		// Prepare AABBs for broad-phase.
		// Even though a body is fast, it may not move much. So the AABB may not need enlargement.

		shapeId = fastBody.headShapeId
		for shapeId != NULL_INDEX {
			shape := Array_Get(&world.shapes, shapeId)

			// Must recompute aabb at the interpolated transform
			aabb := ComputeFatShapeAABB(shape, transform, speculativeScalar)
			shape.aabb = aabb

			shapeFatAABB := &world.fatAABBs.data[shapeId]
			if AABB_Contains(shapeFatAABB^, aabb) == false {
				shapeFatAABB^ = AABB_Inflate(aabb, shape.aabbMargin)

				// Regular bodies mark the hierarchy as moved using atomic operations.
				// Bullets are handled separately at a later stage.
				if isBullet == false {
					BroadPhase_MarkProxyMoved(&world.broadPhase, shape.proxyKey, shapeFatAABB^)
				} else {
					fastBodySim.flags |= enlargeBulletBounds
				}
			}

			shapeId = shape.nextShapeId
		}
	} else {
		// No time of impact event

		// Advance body
		fastBodySim.rotation0 = fastBodySim.transform.q
		fastBodySim.center0 = fastBodySim.center

		// Prepare AABBs for broad-phase
		shapeId = fastBody.headShapeId
		for shapeId != NULL_INDEX {
			shape := Array_Get(&world.shapes, shapeId)

			// shape->aabb is still valid from above

			shapeFatAABB := &world.fatAABBs.data[shapeId]
			if AABB_Contains(shapeFatAABB^, shape.aabb) == false {
				shapeFatAABB^ = AABB_Inflate(shape.aabb, shape.aabbMargin)

				if isBullet == false {
					BroadPhase_MarkProxyMoved(&world.broadPhase, shape.proxyKey, shapeFatAABB^)
				} else {
					fastBodySim.flags |= enlargeBulletBounds
				}
			}

			shapeId = shape.nextShapeId
		}
	}

	// Push sensor hits on the the task context for serial processing.
	for i: i32 = 0; i < context_.sensorCount; i += 1 {
		// Skip any sensor hits that occurred after a solid hit
		if context_.sensorFractions[i] < context_.fraction {
			Array_Push(&taskContext.sensorHits, context_.sensorHits[i])
		}
	}

	taskContext.distanceIterations = MaxInt(taskContext.distanceIterations, context_.distanceIterations)
	taskContext.pushBackIterations = MaxInt(taskContext.pushBackIterations, context_.pushBackIterations)
	taskContext.rootIterations = MaxInt(taskContext.rootIterations, context_.rootIterations)
}

// Implements b3ParallelForCallback
@(private = "file")
FinalizeBodiesTask :: proc(startIndex: i32, endIndex: i32, workerIndex: i32, context_: rawptr) {
	stepContext := (^StepContext)(context_)
	world := stepContext.world
	bodies := world.bodies.data
	sims := stepContext.sims
	states := stepContext.states

	assert(endIndex <= world.bodyMoveEvents.count)

	// unified: the local enableSleep is enableSleep_; the b3_enableSleep body flag is enableSleep in Odin
	enableSleep_ := world.enableSleep
	// unified end
	enableContinuous := world.enableContinuous
	timeStep := stepContext.dt
	invTimeStep := stepContext.inv_dt
	worldId := world.worldId

	// The body move event array has should already have the correct size
	moveEvents := world.bodyMoveEvents.data

	taskContext := &world.taskContexts.data[workerIndex]
	awakeIslandBitSet := &taskContext.awakeIslandBitSet

	speculativeScalar := SPECULATIVE_DISTANCE

	for simIndex := startIndex; simIndex < endIndex; simIndex += 1 {
		state := &states[simIndex]
		sim := &sims[simIndex]

		v := state.linearVelocity
		w := state.angularVelocity

		// unified: motion locks hold for the velocities a step leaves too . The relax iterations and
		// restitution run after the last IntegratePositions, and contacts solved with full 3D inertia (the
		// corners of a plane-locked box) leave locked components that would be reported and carried into the
		// next step's solve.
		v.x = (state.flags & lockLinearX) != 0 ? 0.0 : v.x
		v.y = (state.flags & lockLinearY) != 0 ? 0.0 : v.y
		v.z = (state.flags & lockLinearZ) != 0 ? 0.0 : v.z
		w.x = (state.flags & lockAngularX) != 0 ? 0.0 : w.x
		w.y = (state.flags & lockAngularY) != 0 ? 0.0 : w.y
		w.z = (state.flags & lockAngularZ) != 0 ? 0.0 : w.z
		state.linearVelocity = v
		state.angularVelocity = w
		// unified end

		localOmega := InvRotateVector(sim.transform.q, w)
		localDeltaRotation := InvRotateVector(sim.transform.q, state.deltaRotation.v)

		if IsValidVec3(v) == false || IsValidVec3(w) == false {
			name := FindNameWithDefault(&world.names, bodies[sim.bodyId].nameId, "NULL")
			Log("unstable: %s", name)
		}

		assert(IsValidVec3(v))
		assert(IsValidVec3(w))

		sim.center = OffsetPos(sim.center, state.deltaPosition)
		sim.transform.q = NormalizeQuat(MulQuat(state.deltaRotation, sim.transform.q))

		// Use the velocity of the farthest point on the body to account for rotation.
		velocityArc := ModifiedCross(Abs(localOmega), sim.maxExtent)
		maxVelocity := Length(v) + Length(velocityArc)

		// Sleep needs to observe position correction as well as true velocity.
		// q = [sin(theta/2) * v, cos(theta/2)]
		// for small angles abs(theta) ~= 2 * length(sin(theta/2) * v)
		rotationArc := ModifiedCross(Abs(localDeltaRotation), sim.maxExtent)
		maxDeltaPosition := Length(state.deltaPosition) + 2.0 * Length(rotationArc)

		// Position correction is not as important for sleep as true velocity.
		positionSleepFactor: f32 = 0.5
		sleepVelocity := MaxFloat(maxVelocity, positionSleepFactor * invTimeStep * maxDeltaPosition)

		// reset state deltas
		state.deltaPosition = Vec3_zero
		state.deltaRotation = Quat_identity

		sim.transform.p = OffsetPos(sim.center, Neg(RotateVector(sim.transform.q, sim.localCenter)))

		// cache miss here, however I need the shape list below
		body := &bodies[sim.bodyId]
		body.bodyMoveIndex = simIndex
		body.sleepVelocity = sleepVelocity

		moveEvents[simIndex].userData = body.userData
		moveEvents[simIndex].transform = sim.transform
		moveEvents[simIndex].bodyId = BodyId{sim.bodyId + 1, worldId, body.generation}
		moveEvents[simIndex].fellAsleep = false

		// reset applied force and torque
		sim.force = Vec3_zero
		sim.torque = Vec3_zero

		// If you hit this then it means you deferred mass computation but never called b3Body_ApplyMassFromShapes
		// or b3Body_SetMassData.
		assert((body.flags & dirtyMass) == 0)

		// Clear the transient flags (fast, speed capped, had TOI). These flags are conditionally set
		// as part of the code below.
		body.flags &~= bodyTransientFlags
		sim.flags &~= (isFast | bodyTransientFlags)

		// The body state flag knows about speed capping (used for debug draw).
		body.flags |= (state.flags & isSpeedCapped)
		state.flags &~= bodyTransientFlags

		if enableSleep_ == false || (body.flags & enableSleep) == 0 || sleepVelocity > body.sleepThreshold {
			// Body is not sleepy
			body.sleepTime = 0.0

			safetyFactor := body.safetyFactor
			maxMotion := MaxFloat(maxDeltaPosition, maxVelocity * timeStep)
			if body.type == .dynamicBody && enableContinuous && maxMotion > safetyFactor * sim.minExtent {
				// This flag is used for debug draw and contact recycling.
				sim.flags |= isFast

				// Store in fast array for the continuous collision stage
				// This is deterministic because the order of TOI sweeps doesn't matter
				if (sim.flags & isBullet) != 0 {
					bulletIndex := AtomicFetchAddInt(&stepContext.bulletBodyCount, 1)
					stepContext.bulletBodies[bulletIndex] = simIndex
				} else {
					SolveContinuous(world, simIndex, taskContext, timeStep)
				}
			} else {
				// Body is safe to advance
				sim.center0 = sim.center
				sim.rotation0 = sim.transform.q
			}
		} else {
			// Body is safe to advance and is falling asleep
			sim.center0 = sim.center
			sim.rotation0 = sim.transform.q
			body.sleepTime += timeStep
		}

		// Update world space inverse inertia tensor.
		rotationMatrix := MakeMatrixFromQuat(sim.transform.q)
		sim.invInertiaWorld = MulMM(MulMM(rotationMatrix, sim.invInertiaLocal), Transpose(rotationMatrix))

		// Any single body in an island can keep it awake
		island := Array_Get(&world.islands, body.islandId)
		if body.sleepTime < TIME_TO_SLEEP {
			// keep island awake
			islandIndex := island.localIndex
			SetBit(awakeIslandBitSet, u32(islandIndex))
		} else if island.constraintRemoveCount > 0 {
			// Body wants to sleep but its island needs splitting first. Track the sleepiest candidate.
			// Break sleep time ties using the island id to ensure determinism. The cross worker reduction
			// breaks ties the same way.
			if body.sleepTime > taskContext.splitSleepTime ||
			   (body.sleepTime == taskContext.splitSleepTime && body.islandId > taskContext.splitIslandId) {
				// pick the sleepiest candidate
				taskContext.splitIslandId = body.islandId
				taskContext.splitSleepTime = body.sleepTime
			}
		}

		// Update shapes AABBs
		// For fast non-bullet bodies the AABB has already been updated in b3SolveContinuous
		// For fast bullet bodies the AABB will be updated at a later stage
		if (sim.flags & isFast) == 0 {
			transform := sim.transform
			shapeId := body.headShapeId
			for shapeId != NULL_INDEX {
				shape := Array_Get(&world.shapes, shapeId)

				aabb := ComputeFatShapeAABB(shape, transform, speculativeScalar)
				shape.aabb = aabb

				shapeFatAABB := &world.fatAABBs.data[shapeId]
				if AABB_Contains(shapeFatAABB^, aabb) == false {
					shapeFatAABB^ = AABB_Inflate(aabb, shape.aabbMargin)

					// Mark the hierarchy as moved using atomic operations.
					BroadPhase_MarkProxyMoved(&world.broadPhase, shape.proxyKey, shapeFatAABB^)
				}

				shapeId = shape.nextShapeId
			}
		}
	}
}

@(private = "file")
BlockDim :: struct {
	// number of items per block (except last block)
	size:  i32,

	// total number of blocks
	count: i32,
}

// A block is a range of tasks, a start index and count as a sub-array. Each worker receives at
// most M blocks of work. The workers may receive less blocks if there is not sufficient work.
// Each block of work has a minimum number of elements (block size). This in turn may limit the
// number of blocks. If there are many elements then the block size is increased so there are
// still at most M blocks of work per worker. M is a tunable number that has two goals:
// 1. keep M small to reduce overhead
// 2. keep M large enough for other workers to be able to steal work
// The block size is a power of two to make math efficient.
@(private = "file")
ComputeBlockCount :: proc(itemCount: i32, minSize: i32, maxBlockCount: i32) -> BlockDim {
	dim: BlockDim = {}
	if itemCount == 0 {
		return dim
	}

	if itemCount <= minSize * maxBlockCount {
		dim.size = minSize
	} else {
		dim.size = (itemCount + maxBlockCount - 1) / maxBlockCount
	}

	dim.count = (itemCount + dim.size - 1) / dim.size

	assert(dim.count >= 1)
	assert(dim.size * dim.count >= itemCount)

	return dim
}

// Initialize solver blocks for a contiguous range of items. Computes block size internally
// from the same parameters used by b3ComputeBlockCount. The atomic claim counter is zeroed
// so workers can CAS (0, 1) on the first stage that owns these blocks.
@(private = "file")
InitBlocks :: proc(blocks: [^]SyncBlock, dim: BlockDim, itemCount: i32, blockType: u8, colorIndex: u8) {
	if dim.count == 0 {
		return
	}

	assert(itemCount >= dim.count)

	// Compute the number of elements per block
	blockSize := dim.size

	// Simulation too big
	assert(blockSize <= i32(max(u16)))

	for i: i32 = 0; i < dim.count; i += 1 {
		blocks[i].block.startIndex = i * blockSize
		blocks[i].block.count = u16(blockSize)
		blocks[i].block.blockType = blockType
		blocks[i].block.colorIndex = colorIndex
		AtomicStoreInt(&blocks[i].syncIndex, 0)
	}

	// The last block may not be full
	blocks[dim.count - 1].block.count = u16(itemCount - (dim.count - 1) * blockSize)

	VALIDATE(i32(blocks[dim.count - 1].block.count) <= blockSize)
	VALIDATE((dim.count - 1) * dim.size + i32(blocks[dim.count - 1].block.count) == itemCount)
}

@(private = "file")
InitStage :: proc(
	stage: ^SolverStage,
	type: SolverStageType,
	blocks: [^]SyncBlock,
	blockCount: i32,
	colorIndex: u8,
) -> ^SolverStage {
	stage.type = type
	stage.blocks = blocks
	stage.blockCount = blockCount
	stage.colorIndex = colorIndex
	AtomicStoreInt(&stage.completionCount, 0)
	return intrinsics.ptr_offset(stage, 1)
}

// Initialize one stage per color for each iteration. Used for warm start, solve, relax, and restitution.
// All iterations of a given color share the same b3SyncBlock array so the per-block syncIndex
// grows monotonically across stages within that color.
@(private = "file")
InitColorStages :: proc(
	stage: ^SolverStage,
	type: SolverStageType,
	iterations: i32,
	activeColorCount: i32,
	colorBlocks: [^][^]SyncBlock,
	colorBlockCounts: [^]i32,
	activeColorIndices: [^]i32,
) -> ^SolverStage {
	stage := stage
	for j: i32 = 0; j < iterations; j += 1 {
		for i: i32 = 0; i < activeColorCount; i += 1 {
			stage = InitStage(stage, type, colorBlocks[i], colorBlockCounts[i], u8(activeColorIndices[i]))
		}
	}
	return stage
}

@(private = "file")
ExecuteBlock :: proc(stage: ^SolverStage, context_: ^StepContext, block: SolverBlock, workerIndex: i32) {
	stageType := stage.type
	blockType := SolverBlockType(block.blockType)

	switch stageType {
	case .stagePrepareJoints:
		PrepareJointsTask(block, context_)

	case .stagePrepareWideContacts:
		PrepareContacts_Convex(block, context_)

	case .stagePrepareContacts:
		PrepareContacts_MeshWide(block, context_)

	case .stageIntegrateVelocities:
		IntegrateVelocitiesTask(block, context_)

	case .stageWarmStart:
		if blockType == .graphJointBlock {
			WarmStartJointsTask(block, context_)
		} else if blockType == .graphWideContactBlock {
			WarmStartContacts_Convex(block, context_)
		} else {
			WarmStartContacts_MeshWide(block, context_)
		}

	case .stageSolve:
		if blockType == .graphJointBlock {
			useBias := true
			SolveJointsTask(block, context_, useBias, workerIndex)
		} else if blockType == .graphWideContactBlock {
			PushContacts_Convex(block, context_)
		} else {
			PushContacts_MeshWide(block, context_)
		}

	case .stageIntegratePositions:
		IntegratePositionsTask(block, context_)

	case .stageRelax:
		if blockType == .graphJointBlock {
			useBias := false
			SolveJointsTask(block, context_, useBias, workerIndex)
		} else if blockType == .graphWideContactBlock {
			SolveContacts_Convex(block, context_)
		} else {
			SolveContacts_MeshWide(block, context_)
		}

	case .stageRestitution:
		if blockType == .graphWideContactBlock {
			ApplyRestitution_Convex(block, context_)
		} else if blockType == .graphContactBlock {
			ApplyRestitution_MeshWide(block, context_)
		}

	case .stageStoreWideImpulses:
		StoreImpulses_Convex(block, context_, workerIndex)

	case .stageStoreImpulses:
		StoreImpulses_MeshWide(block, context_, workerIndex)
	}
}

// This staggers the worker start indices so they avoid touching the same solver blocks
@(private = "file")
GetWorkerStartIndex :: proc(workerIndex: i32, blockCount: i32, workerCount: i32) -> i32 {
	if blockCount <= workerCount {
		return workerIndex < blockCount ? workerIndex : NULL_INDEX
	}

	blocksPerWorker := blockCount / workerCount
	remainder := blockCount - blocksPerWorker * workerCount
	return blocksPerWorker * workerIndex + MinInt(remainder, workerIndex)
}

// Execute a stage, which is an array of solver blocks, each controlled with an atomic sync index.
// Each worker starts at its home index and sweeps the ring, CAS-claiming any unclaimed blocks.
@(private = "file")
ExecuteStage :: proc(
	stage: ^SolverStage,
	context_: ^StepContext,
	previousSyncIndex: i32,
	syncIndex: i32,
	workerIndex: i32,
) {
	completedCount: i32 = 0
	blocks := stage.blocks
	blockCount := stage.blockCount

	startIndex := GetWorkerStartIndex(workerIndex, blockCount, context_.workerCount)
	if startIndex == NULL_INDEX {
		return
	}

	assert(0 <= startIndex && startIndex < blockCount)

	blockIndex := startIndex
	for i: i32 = 0; i < blockCount; i += 1 {
		if AtomicCompareExchangeInt(&blocks[blockIndex].syncIndex, previousSyncIndex, syncIndex) {
			assert(completedCount < blockCount)

			// Pass the descriptor by value -- the wrapping b3SyncBlock holds the atomic
			// syncIndex but we only copy .block, so the struct copy never aliases the CAS target.
			ExecuteBlock(stage, context_, blocks[blockIndex].block, workerIndex)
			completedCount += 1
		}

		blockIndex += 1
		if blockIndex >= blockCount {
			blockIndex = 0
		}
	}

	_ = AtomicFetchAddInt(&stage.completionCount, completedCount)
}

// Execute a stage on worker 0 (main thread).
@(private = "file")
ExecuteMainStage :: proc(stage: ^SolverStage, context_: ^StepContext, syncBits: u32) {
	blockCount := stage.blockCount
	if blockCount == 0 {
		return
	}

	workerIndex: i32 : 0

	if blockCount == 1 {
		ExecuteBlock(stage, context_, stage.blocks[0].block, workerIndex)
	} else {
		AtomicStoreU32(&context_.atomicSyncBits, syncBits)

		syncIndex := i32((syncBits >> 16) & 0xFFFF)
		assert(syncIndex > 0)
		previousSyncIndex := syncIndex - 1

		ExecuteStage(stage, context_, previousSyncIndex, syncIndex, workerIndex)

		// Spin waiting for thieves to finish
		for AtomicLoadInt(&stage.completionCount) != blockCount {
			Pause()
		}

		AtomicStoreInt(&stage.completionCount, 0)
	}
}

// Parallel solver task
@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
SolverTask :: proc "c" (taskContext: rawptr) {
	context = runtime.default_context()
	// unified end
	workerContext := (^WorkerContext)(taskContext)
	workerIndex := workerContext.workerIndex
	context_ := workerContext.context_
	activeColorCount := context_.activeColorCount
	stages := context_.stages
	profile := &context_.world.profile

	if workerIndex == 0 {
		// The orchestrator slot is a race. The calling thread of b3World_Step also enters here
		// as worker 0, so progress is guaranteed even if the user's task system schedules tasks
		// out of order, has fewer threads than workerCount, or runs the task synchronously
		// inside enqueueTaskFcn. Whoever wins the CAS becomes the orchestrator; the loser
		// returns and lets the spinner-only path handle workers >0.
		if AtomicCompareExchangeInt(&context_.mainClaimed, 0, 1) == false {
			return
		}

		// Main thread synchronizes the workers and does work itself.
		//
		// This needs to be a task for the main thread because the user's task system may execute
		// the tasks serially and this is the first task. This single task is able to fully
		// complete all work even if all other workers are blocked.

		// Stages are re-used by loops so that I don't need more stages for large substep counts.
		// The sync indices grow monotonically for the body/graph/constraint groupings because they share solver blocks.
		// The stage index and sync indices are combined in to sync bits for atomic synchronization.
		// The workers need to compute the previous sync index for a given stage so that CAS works correctly. This
		// setup makes this easy to do.

		/*
		b3_stagePrepareJoints,
		b3_stagePrepareContacts,
		b3_stageIntegrateVelocities,
		b3_stageWarmStart,
		b3_stageSolve,
		b3_stageIntegratePositions,
		b3_stageRelax,
		b3_stageStoreImpulses
		*/

		ticks := GetTicks()

		bodySyncIndex: i32 = 1
		stageIndex: i32 = 0

		// Prepare joint constraints
		jointSyncIndex: u32 = 1
		syncBits := (jointSyncIndex << 16) | u32(stageIndex)
		assert(stages[stageIndex].type == .stagePrepareJoints)
		ExecuteMainStage(&stages[stageIndex], context_, syncBits)
		stageIndex += 1
		jointSyncIndex += 1

		// Prepare convex contact constraints
		convexSyncIndex: u32 = 1
		syncBits = (convexSyncIndex << 16) | u32(stageIndex)
		assert(stages[stageIndex].type == .stagePrepareWideContacts)
		ExecuteMainStage(&stages[stageIndex], context_, syncBits)
		stageIndex += 1
		convexSyncIndex += 1

		// Prepare mesh contact constraints
		meshSyncIndex: u32 = 1
		syncBits = (meshSyncIndex << 16) | u32(stageIndex)
		assert(stages[stageIndex].type == .stagePrepareContacts)
		ExecuteMainStage(&stages[stageIndex], context_, syncBits)
		stageIndex += 1
		meshSyncIndex += 1

		// Single-threaded overflow work. These constraints don't fit in the graph coloring.
		PrepareJoints_Overflow(context_)
		PrepareContacts_Overflow(context_)

		profile.prepareConstraints += GetMillisecondsAndReset(&ticks)

		graphSyncIndex: i32 = 1
		subStepCount := context_.subStepCount
		for subStepIndex: i32 = 0; subStepIndex < subStepCount; subStepIndex += 1 {
			// stageIndex restarted each iteration
			// syncBits still increases monotonically because the upper bits increase each iteration
			iterationStageIndex := stageIndex

			// Integrate velocities
			syncBits = (u32(bodySyncIndex) << 16) | u32(iterationStageIndex)
			assert(stages[iterationStageIndex].type == .stageIntegrateVelocities)
			ExecuteMainStage(&stages[iterationStageIndex], context_, syncBits)
			iterationStageIndex += 1
			bodySyncIndex += 1

			profile.integrateVelocities += GetMillisecondsAndReset(&ticks)

			// Warm start constraints
			WarmStartJoints_Overflow(context_)
			WarmStartContacts_Overflow(context_)

			for colorIndex: i32 = 0; colorIndex < activeColorCount; colorIndex += 1 {
				syncBits = (u32(graphSyncIndex) << 16) | u32(iterationStageIndex)
				assert(stages[iterationStageIndex].type == .stageWarmStart)
				ExecuteMainStage(&stages[iterationStageIndex], context_, syncBits)
				iterationStageIndex += 1
			}
			graphSyncIndex += 1

			profile.warmStart += GetMillisecondsAndReset(&ticks)

			// Solve constraints
			useBias := true
			for j: i32 = 0; j < ITERATIONS; j += 1 {
				// Overflow constraints have lower priority. Typically these are dynamic-vs-dynamic.
				SolveJoints_Overflow(context_, useBias)
				PushContacts_Overflow(context_)

				for colorIndex: i32 = 0; colorIndex < activeColorCount; colorIndex += 1 {
					syncBits = (u32(graphSyncIndex) << 16) | u32(iterationStageIndex)
					assert(stages[iterationStageIndex].type == .stageSolve)
					ExecuteMainStage(&stages[iterationStageIndex], context_, syncBits)
					iterationStageIndex += 1
				}
				graphSyncIndex += 1
			}

			profile.solveImpulses += GetMillisecondsAndReset(&ticks)

			// Integrate positions
			assert(stages[iterationStageIndex].type == .stageIntegratePositions)
			syncBits = (u32(bodySyncIndex) << 16) | u32(iterationStageIndex)
			ExecuteMainStage(&stages[iterationStageIndex], context_, syncBits)
			iterationStageIndex += 1
			bodySyncIndex += 1

			profile.integratePositions += GetMillisecondsAndReset(&ticks)

			// Relax constraints
			useBias = false
			for j: i32 = 0; j < RELAX_ITERATIONS; j += 1 {
				SolveJoints_Overflow(context_, useBias)
				SolveContacts_Overflow(context_)

				for colorIndex: i32 = 0; colorIndex < activeColorCount; colorIndex += 1 {
					syncBits = (u32(graphSyncIndex) << 16) | u32(iterationStageIndex)
					assert(stages[iterationStageIndex].type == .stageRelax)
					ExecuteMainStage(&stages[iterationStageIndex], context_, syncBits)
					iterationStageIndex += 1
				}
				graphSyncIndex += 1
			}

			profile.relaxImpulses += GetMillisecondsAndReset(&ticks)
		}

		// Advance the stage according to the sub-stepping tasks just completed
		// integrate velocities / warm start / solve / integrate positions / relax
		stageIndex += 1 + activeColorCount + ITERATIONS * activeColorCount + 1 + RELAX_ITERATIONS * activeColorCount

		restitutionIterations := context_.world.restitutionIterations
		if restitutionIterations > 0 && AtomicLoadInt(&context_.anyRestitution) != 0 {
			for j: i32 = 0; j < restitutionIterations; j += 1 {
				ApplyRestitution_Overflow(context_)

				iterationStageIndex := stageIndex
				for colorIndex: i32 = 0; colorIndex < activeColorCount; colorIndex += 1 {
					syncBits = (u32(graphSyncIndex) << 16) | u32(iterationStageIndex)
					assert(stages[iterationStageIndex].type == .stageRestitution)
					ExecuteMainStage(&stages[iterationStageIndex], context_, syncBits)
					iterationStageIndex += 1
				}
				graphSyncIndex += 1
			}

			profile.restitution += GetMillisecondsAndReset(&ticks)
		}

		stageIndex += activeColorCount

		// Store impulses
		StoreImpulses_Overflow(context_)

		syncBits = (convexSyncIndex << 16) | u32(stageIndex)
		assert(stages[stageIndex].type == .stageStoreWideImpulses)
		ExecuteMainStage(&stages[stageIndex], context_, syncBits)
		stageIndex += 1

		syncBits = (meshSyncIndex << 16) | u32(stageIndex)
		assert(stages[stageIndex].type == .stageStoreImpulses)
		ExecuteMainStage(&stages[stageIndex], context_, syncBits)
		stageIndex += 1

		profile.storeImpulses += GetMillisecondsAndReset(&ticks)

		// Signal workers to finish
		AtomicStoreU32(&context_.atomicSyncBits, max(u32))

		assert(stageIndex == context_.stageCount)
		return
	}

	// Worker spins and waits for work
	lastSyncBits: u32 = 0
	// uint64_t maxSpinTime = 10;
	for true {
		// Spin until main thread bumps changes the sync bits. This can waste significant time overall, but it is necessary for
		// parallel simulation with graph coloring.
		// todo improve this spinner
		syncBits: u32
		spinCount: i32 = 0
		// unified: C's while ((syncBits = load) == lastSyncBits) as a loop with the load and test inside
		for {
			syncBits = AtomicLoadU32(&context_.atomicSyncBits)
			if syncBits != lastSyncBits {
				break
			}
			// unified end

			if spinCount > 5 {
				Yield()
				spinCount = 0
			} else {
				// Using the cycle counter helps to account for variation in mm_pause timing across different
				// CPUs. However, this is X64 only.
				// uint64_t prev = __rdtsc();
				// do
				//{
				//	b3Pause();
				//}
				// while ((__rdtsc() - prev) < maxSpinTime);
				// maxSpinTime += 10;
				Pause()
				Pause()
				spinCount += 1
			}
		}

		if syncBits == max(u32) {
			// sentinel hit
			break
		}

		stageIndex := i32(syncBits & 0xFFFF)
		assert(stageIndex < context_.stageCount)

		syncIndex := i32((syncBits >> 16) & 0xFFFF)
		assert(syncIndex > 0)

		previousSyncIndex := syncIndex - 1

		stage := &stages[stageIndex]
		ExecuteStage(stage, context_, previousSyncIndex, syncIndex, workerIndex)

		lastSyncBits = syncBits
	}
}

@(private = "file")
BulletBodyTask :: proc(startIndex: i32, endIndex: i32, workerIndex: i32, context_: rawptr) {
	stepContext := (^StepContext)(context_)
	taskContext := Array_Get(&stepContext.world.taskContexts, workerIndex)

	assert(startIndex <= endIndex)

	for i := startIndex; i < endIndex; i += 1 {
		simIndex := stepContext.bulletBodies[i]
		SolveContinuous(stepContext.world, simIndex, taskContext, stepContext.dt)
	}
}

// Solve with graph coloring
Solve :: proc(world: ^World, stepContext: ^StepContext) {
	simdShift: u32 = world.simdWidth == 8 ? 3 : 2

	// Only count steps that advance the simulation
	world.stepIndex += 1

	awakeSet := Array_Get(&world.solverSets, i32(SetType.awakeSet))
	awakeBodyCount := awakeSet.bodySims.count
	if awakeBodyCount == 0 {
		return
	}

	// Solve constraints using graph coloring
	{
		setupTicks := GetTicks()

		// Prepare buffers for continuous collision (fast bodies)
		AtomicStoreInt(&stepContext.bulletBodyCount, 0)
		stepContext.bulletBodies = cast([^]i32)StackAlloc(&world.stack, awakeBodyCount * size_of(i32), "bullet bodies")

		graph := &world.constraintGraph
		colors := &graph.colors

		stepContext.sims = awakeSet.bodySims.data
		stepContext.states = awakeSet.bodyStates.data

		// count contacts, joints, and colors
		activeColorCount: i32 = 0
		for i: i32 = 0; i < GRAPH_COLOR_COUNT - 1; i += 1 {
			perColorContactCount := colors[i].convexContacts.count + colors[i].contacts.count
			perColorJointCount := colors[i].jointSims.count
			occupancyCount := perColorContactCount + perColorJointCount
			activeColorCount += occupancyCount > 0 ? 1 : 0
		}

		// prepare for move events
		Array_Resize(&world.bodyMoveEvents, awakeBodyCount)

		workerCount := world.workerCount

		// Target 4 blocks per worker to allow work stealing
		maxBlockCount := 4 * workerCount

		// Body blocks are for parallel iteration over bodies directly (integration, update transforms)
		minBodiesPerBlock: i32 = 32
		bodyDim := ComputeBlockCount(awakeBodyCount, minBodiesPerBlock, maxBlockCount)

		minContactsPerBlock: i32 : 4
		minJointsPerBlock: i32 : 4

		// Configure blocks for tasks parallel-for each active graph color
		// The blocks are a mix of convex contact, mesh contact, and joint blocks
		activeColorIndices: [GRAPH_COLOR_COUNT]i32 = ---
		colorWideContactCounts: [GRAPH_COLOR_COUNT]i32 = ---
		colorMeshContactCounts: [GRAPH_COLOR_COUNT]i32 = ---
		colorMeshGroupCounts: [GRAPH_COLOR_COUNT]i32 = ---
		colorJointCounts: [GRAPH_COLOR_COUNT]i32 = ---
		graphWideContactDims: [GRAPH_COLOR_COUNT]BlockDim = ---
		graphMeshContactDims: [GRAPH_COLOR_COUNT]BlockDim = ---
		graphJointDims: [GRAPH_COLOR_COUNT]BlockDim = ---
		graphBlockCount: i32 = 0

		// c is the active color index
		wideContactCount: i32 = 0
		meshContactCount: i32 = 0
		meshGroupCount: i32 = 0
		jointCount: i32 = 0
		c: i32 = 0
		for i: i32 = 0; i < GRAPH_COLOR_COUNT - 1; i += 1 {
			color := &colors[i]
			colorConvexContactCount := color.convexContacts.count
			colorMeshContactCount := color.contacts.count
			colorJointCount := color.jointSims.count

			if colorConvexContactCount + colorMeshContactCount + colorJointCount == 0 {
				continue
			}

			activeColorIndices[c] = i

			// Ceiling for wide constraint count
			colorWideConstraintCount: i32 =
				colorConvexContactCount > 0 ? ((colorConvexContactCount - 1) >> simdShift) + 1 : 0
			wideContactCount += colorWideConstraintCount
			colorWideContactCounts[c] = colorWideConstraintCount

			colorMeshContactCounts[c] = colorMeshContactCount
			meshContactCount += colorMeshContactCount

			colorMeshGroupCount: i32 = colorMeshContactCount > 0 ? ((colorMeshContactCount - 1) >> simdShift) + 1 : 0
			colorMeshGroupCounts[c] = colorMeshGroupCount
			meshGroupCount += colorMeshGroupCount

			colorJointCounts[c] = colorJointCount
			jointCount += colorJointCount

			// Solver block dimensions
			graphWideContactDims[c] = ComputeBlockCount(colorWideConstraintCount, minContactsPerBlock, maxBlockCount)
			graphMeshContactDims[c] = ComputeBlockCount(colorMeshGroupCount, minContactsPerBlock, maxBlockCount)
			graphJointDims[c] = ComputeBlockCount(colorJointCount, minJointsPerBlock, maxBlockCount)
			graphBlockCount += graphWideContactDims[c].count + graphMeshContactDims[c].count + graphJointDims[c].count

			c += 1
		}
		activeColorCount = c

		// Prepare and store run as one flat parallel-for over the entire wide constraint range,
		// partitioned into uniformly sized blocks. Color info is consulted inside the task via
		// a small span array, so blocks do not need to honor color boundaries here.
		convexPrepareDim := ComputeBlockCount(wideContactCount, minContactsPerBlock, maxBlockCount)
		meshPrepareDim := ComputeBlockCount(meshGroupCount, minContactsPerBlock, maxBlockCount)
		jointPrepareDim := ComputeBlockCount(jointCount, minJointsPerBlock, maxBlockCount)

		wideContactByteCount := GetWideContactConstraintByteCount(world.simdWidth)
		wideConstraints := StackAlloc(&world.stack, wideContactCount * wideContactByteCount, "wide contacts")

		meshLaneOrder := cast([^]i32)StackAlloc(&world.stack, meshContactCount * size_of(i32), "mesh lane order")
		meshManifoldStarts := cast([^]i32)StackAlloc(
			&world.stack,
			(meshGroupCount + 1) * size_of(i32),
			"mesh manifold starts",
		)

		// Sort mesh contacts by manifold count so that contacts with more manifolds are solved
		// together keeping the SIMD lanes fuller and less ragged.
		meshSlotCount: i32 = 0
		{
			maxManifoldCount: i32 = 0
			for i: i32 = 0; i < activeColorCount; i += 1 {
				color := &colors[activeColorIndices[i]]
				specs := color.contacts.data
				colorContactCount := colorMeshContactCounts[i]
				for j: i32 = 0; j < colorContactCount; j += 1 {
					maxManifoldCount = MaxInt(maxManifoldCount, i32(specs[j].manifoldCount))
				}
			}

			bucketStarts := cast([^]i32)StackAlloc(&world.stack, (maxManifoldCount + 1) * size_of(i32), "mesh buckets")

			orderBase: i32 = 0
			groupBase: i32 = 0
			for i: i32 = 0; i < activeColorCount; i += 1 {
				color := &colors[activeColorIndices[i]]
				colorContactCount := colorMeshContactCounts[i]
				if colorContactCount == 0 {
					continue
				}

				specs := color.contacts.data
				order := meshLaneOrder[orderBase:]

				for k: i32 = 0; k <= maxManifoldCount; k += 1 {
					bucketStarts[k] = 0
				}

				for j: i32 = 0; j < colorContactCount; j += 1 {
					bucketStarts[specs[j].manifoldCount] += 1
				}

				start: i32 = 0
				for k := maxManifoldCount; k >= 0; k -= 1 {
					count := bucketStarts[k]
					bucketStarts[k] = start
					start += count
				}

				for j: i32 = 0; j < colorContactCount; j += 1 {
					bucket := &bucketStarts[specs[j].manifoldCount]
					order[bucket^] = j
					bucket^ += 1
				}

				colorMeshGroupCount := colorMeshGroupCounts[i]
				for g: i32 = 0; g < colorMeshGroupCount; g += 1 {
					slotCount := i32(specs[order[g << simdShift]].manifoldCount)
					meshManifoldStarts[groupBase + g] = meshSlotCount
					meshSlotCount += slotCount
				}

				orderBase += colorContactCount
				groupBase += colorMeshGroupCount
			}

			assert(orderBase == meshContactCount)
			assert(groupBase == meshGroupCount)
			meshManifoldStarts[meshGroupCount] = meshSlotCount

			StackFree(&world.stack, bucketStarts)
		}

		wideMeshConstraintByteCount := GetWideMeshConstraintByteCount(world.simdWidth)
		wideMeshManifoldByteCount := GetWideMeshManifoldByteCount(world.simdWidth)
		wideMeshConstraints := StackAlloc(
			&world.stack,
			meshGroupCount * wideMeshConstraintByteCount + meshSlotCount * wideMeshManifoldByteCount,
			"wide mesh constraints",
		)

		overflow := &colors[OVERFLOW_INDEX]
		overflowCount := overflow.contacts.count
		overflowManifoldCount: i32 = 0
		for i: i32 = 0; i < overflowCount; i += 1 {
			overflow.contacts.data[i].manifoldStart = overflowManifoldCount
			overflowManifoldCount += i32(overflow.contacts.data[i].manifoldCount)
		}

		overflow.contactConstraints = cast([^]ContactConstraint)StackAlloc(
			&world.stack,
			overflowCount * size_of(ContactConstraint),
			"overflow contacts",
		)
		overflow.manifoldConstraints = cast([^]ManifoldConstraint)StackAlloc(
			&world.stack,
			overflowManifoldCount * size_of(ManifoldConstraint),
			"overflow manifolds",
		)

		// Build the span table for the flat prepare/store parallel-for while I slice the
		// wide constraint buffer across colors. One entry per active color plus a sentinel
		// at wideContactCount.
		widePrepareSpans: [GRAPH_COLOR_COUNT + 1]WidePrepareSpan = ---
		meshPrepareSpans: [GRAPH_COLOR_COUNT + 1]MeshPrepareSpan = ---
		jointPrepareSpans: [GRAPH_COLOR_COUNT + 1]JointPrepareSpan = ---

		// Distribute transient constraints to each graph color and prepare spans
		{
			wideBase: i32 = 0
			contactBase: i32 = 0
			meshGroupBase: i32 = 0
			jointBase: i32 = 0
			for i: i32 = 0; i < activeColorCount; i += 1 {
				j := activeColorIndices[i]
				color := &colors[j]

				colorConvexContactCount := color.convexContacts.count
				widePrepareSpans[i].start = wideBase
				widePrepareSpans[i].count = colorConvexContactCount
				widePrepareSpans[i].contacts = color.convexContacts.data

				if colorConvexContactCount == 0 {
					color.wideConstraints = nil
					color.wideConstraintCount = 0
				} else {
					color.wideConstraints = &(cast([^]u8)wideConstraints)[wideBase * wideContactByteCount]

					colorContactCountW := ((colorConvexContactCount - 1) >> simdShift) + 1
					color.wideConstraintCount = colorContactCountW
					wideBase += colorContactCountW
				}

				colorContactCount := color.contacts.count
				meshPrepareSpans[i].start = meshGroupBase
				meshPrepareSpans[i].count = colorContactCount
				meshPrepareSpans[i].contacts = color.contacts.data
				meshPrepareSpans[i].order = meshLaneOrder[contactBase:]

				if colorContactCount == 0 {
					color.wideMeshConstraints = nil
					color.wideMeshManifoldStarts = nil
					color.wideMeshConstraintCount = 0
				} else {
					colorMeshGroupCount := colorMeshGroupCounts[i]
					color.wideMeshConstraints = &(cast([^]u8)wideMeshConstraints)[meshGroupBase * wideMeshConstraintByteCount + meshManifoldStarts[meshGroupBase] * wideMeshManifoldByteCount]
					color.wideMeshManifoldStarts = meshManifoldStarts[meshGroupBase:]
					color.wideMeshConstraintCount = colorMeshGroupCount
					meshGroupBase += colorMeshGroupCount
					contactBase += colorContactCount
				}

				jointPrepareSpans[i].start = jointBase
				jointPrepareSpans[i].count = color.jointSims.count
				jointPrepareSpans[i].joints = color.jointSims.data
				jointBase += color.jointSims.count
			}

			// Sentinels
			widePrepareSpans[activeColorCount].start = wideContactCount
			widePrepareSpans[activeColorCount].count = 0
			widePrepareSpans[activeColorCount].contacts = nil
			assert(wideBase == wideContactCount)

			meshPrepareSpans[activeColorCount].start = meshGroupCount
			meshPrepareSpans[activeColorCount].count = 0
			meshPrepareSpans[activeColorCount].contacts = nil
			meshPrepareSpans[activeColorCount].order = nil
			assert(contactBase == meshContactCount)
			assert(meshGroupBase == meshGroupCount)

			jointPrepareSpans[activeColorCount].start = jointCount
			jointPrepareSpans[activeColorCount].count = 0
			jointPrepareSpans[activeColorCount].joints = nil
			assert(jointBase == jointCount)
		}

		//// Special span for overflow to allow for function re-use
		overflowSpans: [2]ContactPrepareSpan = {}
		overflowSpans[0].start = 0
		overflowSpans[0].count = overflow.contacts.count
		overflowSpans[0].contacts = overflow.contacts.data
		overflowSpans[1].start = overflow.contacts.count
		overflowSpans[1].count = 0
		overflowSpans[1].contacts = nil

		stageCount: i32 = 0

		// b3_stagePrepareJoints
		stageCount += 1
		// b3_stagePrepareWideContacts
		stageCount += 1
		// b3_stagePrepareContacts
		stageCount += 1
		// b3_stageIntegrateVelocities
		stageCount += 1
		// b3_stageWarmStart
		stageCount += activeColorCount
		// b3_stageSolve
		stageCount += ITERATIONS * activeColorCount
		// b3_stageIntegratePositions
		stageCount += 1
		// b3_stageRelax
		stageCount += RELAX_ITERATIONS * activeColorCount
		// b3_stageRestitution
		stageCount += activeColorCount
		// b3_stageStoreWideImpulses
		stageCount += 1
		// b3_stageStoreImpulses
		stageCount += 1

		stages := cast([^]SolverStage)StackAlloc(&world.stack, stageCount * size_of(SolverStage), "stages")
		bodyBlocks := cast([^]SyncBlock)StackAlloc(&world.stack, bodyDim.count * size_of(SyncBlock), "body blocks")
		convexBlocks := cast([^]SyncBlock)StackAlloc(
			&world.stack,
			convexPrepareDim.count * size_of(SyncBlock),
			"convex blocks",
		)
		meshBlocks := cast([^]SyncBlock)StackAlloc(
			&world.stack,
			meshPrepareDim.count * size_of(SyncBlock),
			"mesh blocks",
		)
		jointBlocks := cast([^]SyncBlock)StackAlloc(
			&world.stack,
			jointPrepareDim.count * size_of(SyncBlock),
			"joint blocks",
		)
		graphBlocks := cast([^]SyncBlock)StackAlloc(&world.stack, graphBlockCount * size_of(SyncBlock), "graph blocks")

		// Split an awake island. This modifies:
		// - stack allocator
		// - world island array and solver set
		// - island indices on bodies, contacts, and joints
		// I'm squeezing this task in here because it may be expensive and this is a safe place to put it.
		// Note: cannot split islands in parallel with FinalizeBodies
		splitIslandTask: rawptr = nil
		if world.splitIslandId != NULL_INDEX {
			if world.taskCount < MAX_TASKS {
				splitIslandTask = world.enqueueTaskFcn(SplitIslandTask, world, world.userTaskContext, "split")
				world.taskCount += 1
				world.activeTaskCount += splitIslandTask == nil ? 0 : 1
			} else {
				SplitIslandTask(world)
			}
		}

		// Prepare body blocks
		InitBlocks(bodyBlocks, bodyDim, awakeBodyCount, u8(SolverBlockType.bodyBlock), max(u8))

		// Prepare blocks as a single flat parallel-for over the whole constraint range.
		// The task walks spans to decode flat slot indices back to per-color arrays.
		InitBlocks(convexBlocks, convexPrepareDim, wideContactCount, u8(SolverBlockType.wideContactBlock), max(u8))
		InitBlocks(meshBlocks, meshPrepareDim, meshGroupCount, u8(SolverBlockType.contactBlock), max(u8))
		InitBlocks(jointBlocks, jointPrepareDim, jointCount, u8(SolverBlockType.jointBlock), max(u8))

		// Prepare graph work blocks. Each color gets joint blocks followed by contact blocks.
		graphColorBlocks: [GRAPH_COLOR_COUNT][^]SyncBlock = {}
		baseGraphBlock := graphBlocks
		graphBlockCounts: [GRAPH_COLOR_COUNT]i32 = {}
		for i: i32 = 0; i < activeColorCount; i += 1 {
			graphColorBlocks[i] = baseGraphBlock

			colorIndex := u8(activeColorIndices[i])
			InitBlocks(
				baseGraphBlock,
				graphJointDims[i],
				colorJointCounts[i],
				u8(SolverBlockType.graphJointBlock),
				colorIndex,
			)
			baseGraphBlock = baseGraphBlock[graphJointDims[i].count:]

			InitBlocks(
				baseGraphBlock,
				graphWideContactDims[i],
				colorWideContactCounts[i],
				u8(SolverBlockType.graphWideContactBlock),
				colorIndex,
			)
			baseGraphBlock = baseGraphBlock[graphWideContactDims[i].count:]

			InitBlocks(
				baseGraphBlock,
				graphMeshContactDims[i],
				colorMeshGroupCounts[i],
				u8(SolverBlockType.graphContactBlock),
				colorIndex,
			)
			baseGraphBlock = baseGraphBlock[graphMeshContactDims[i].count:]

			graphBlockCounts[i] =
				graphJointDims[i].count + graphWideContactDims[i].count + graphMeshContactDims[i].count
		}

		assert(intrinsics.ptr_sub((^SyncBlock)(baseGraphBlock), (^SyncBlock)(graphBlocks)) == int(graphBlockCount))

		stage := &stages[0]
		stage = InitStage(stage, .stagePrepareJoints, jointBlocks, jointPrepareDim.count, max(u8))
		stage = InitStage(stage, .stagePrepareWideContacts, convexBlocks, convexPrepareDim.count, max(u8))
		stage = InitStage(stage, .stagePrepareContacts, meshBlocks, meshPrepareDim.count, max(u8))
		stage = InitStage(stage, .stageIntegrateVelocities, bodyBlocks, bodyDim.count, max(u8))
		stage = InitColorStages(
			stage,
			.stageWarmStart,
			1,
			activeColorCount,
			&graphColorBlocks[0],
			&graphBlockCounts[0],
			&activeColorIndices[0],
		)
		stage = InitColorStages(
			stage,
			.stageSolve,
			ITERATIONS,
			activeColorCount,
			&graphColorBlocks[0],
			&graphBlockCounts[0],
			&activeColorIndices[0],
		)
		stage = InitStage(stage, .stageIntegratePositions, bodyBlocks, bodyDim.count, max(u8))
		stage = InitColorStages(
			stage,
			.stageRelax,
			RELAX_ITERATIONS,
			activeColorCount,
			&graphColorBlocks[0],
			&graphBlockCounts[0],
			&activeColorIndices[0],
		)
		stage = InitColorStages(
			stage,
			.stageRestitution,
			1,
			activeColorCount,
			&graphColorBlocks[0],
			&graphBlockCounts[0],
			&activeColorIndices[0],
		)
		stage = InitStage(stage, .stageStoreWideImpulses, convexBlocks, convexPrepareDim.count, max(u8))
		stage = InitStage(stage, .stageStoreImpulses, meshBlocks, meshPrepareDim.count, max(u8))

		assert(i32(intrinsics.ptr_sub(stage, &stages[0])) == stageCount)

		assert(workerCount <= MAX_WORKERS)
		workerContext: [MAX_WORKERS]WorkerContext = ---

		stepContext.graph = graph
		stepContext.activeColorCount = activeColorCount
		stepContext.workerCount = workerCount
		stepContext.stageCount = stageCount
		stepContext.stages = stages
		stepContext.wideConstraints = wideConstraints
		stepContext.widePrepareSpans = &widePrepareSpans[0]
		stepContext.wideContactCount = wideContactCount
		stepContext.overflowSpans = &overflowSpans[0]
		stepContext.wideMeshConstraints = wideMeshConstraints
		stepContext.wideMeshManifoldStarts = meshManifoldStarts
		stepContext.meshPrepareSpans = &meshPrepareSpans[0]
		stepContext.jointPrepareSpans = &jointPrepareSpans[0]
		AtomicStoreU32(&stepContext.atomicSyncBits, 0)
		AtomicStoreInt(&stepContext.mainClaimed, 0)

		// unified: the spans live in this block's stack frame, as in C; clear the pointers when the block ends
		// so they don't dangle (Odin's escape check rejects them otherwise)
		defer stepContext.widePrepareSpans = nil
		defer stepContext.overflowSpans = nil
		defer stepContext.meshPrepareSpans = nil
		defer stepContext.jointPrepareSpans = nil
		// unified end

		world.profile.solverSetup = GetMillisecondsAndReset(&setupTicks)

		constraintTicks := GetTicks()

		jointIdCapacity := GetIdCapacity(&world.jointIdPool)
		contactIdCapacity := GetIdCapacity(&world.contactIdPool)
		for i: i32 = 0; i < workerCount; i += 1 {
			taskContext := Array_Get(&world.taskContexts, i)
			SetBitCountAndClear(&taskContext.jointStateBitSet, u32(jointIdCapacity))
			SetBitCountAndClear(&taskContext.hitEventBitSet, u32(contactIdCapacity))
			taskContext.hasHitEvents = false

			workerContext[i].context_ = stepContext
			workerContext[i].workerIndex = i

			if world.taskCount < MAX_TASKS {
				// unified: snprintf is fmt.bprintf into the zeroed buffer, which stays NUL-terminated
				buffer: [16]u8
				fmt.bprintf(buffer[:len(buffer) - 1], "solve[%d]", i)
				// unified end
				workerContext[i].userTask = world.enqueueTaskFcn(
					SolverTask,
					&workerContext[i],
					world.userTaskContext,
					cstring(&buffer[0]),
				)
				world.taskCount += 1
				world.activeTaskCount += workerContext[i].userTask == nil ? 0 : 1
			} else {
				workerContext[i].userTask = nil
				SolverTask(&workerContext[i])
			}
		}

		// The calling thread of b3World_Step also enters b3SolverTask as worker 0 and races for the
		// orchestrator slot via the CAS inside. This guarantees progress even when the user's task
		// system can't run the queued worker 0 promptly: it might schedule out of order, have fewer
		// threads than workerCount, or invert priority by parking the calling thread in finishTaskFcn.
		// Whoever wins the CAS becomes the orchestrator; the loser returns and lets the spinner-only
		// path handle workers >0.
		callerContext := WorkerContext{stepContext, 0, nil}
		SolverTask(&callerContext)

		// Finish constraint solve
		for i: i32 = 0; i < workerCount; i += 1 {
			if workerContext[i].userTask != nil {
				world.finishTaskFcn(workerContext[i].userTask, world.userTaskContext)
				world.activeTaskCount -= 1
			}
		}

		// Finish island split
		if splitIslandTask != nil {
			world.finishTaskFcn(splitIslandTask, world.userTaskContext)
			world.activeTaskCount -= 1
		}
		world.splitIslandId = NULL_INDEX

		world.profile.constraints = GetMillisecondsAndReset(&constraintTicks)

		transformTicks := GetTicks()

		// Prepare the island bit set used in body finalization.
		awakeIslandCount := awakeSet.islandSims.count
		for i: i32 = 0; i < world.workerCount; i += 1 {
			taskContext := &world.taskContexts.data[i]
			Array_Clear(&taskContext.sensorHits)
			SetBitCountAndClear(&taskContext.awakeIslandBitSet, u32(awakeIslandCount))
			taskContext.splitIslandId = NULL_INDEX
			taskContext.splitSleepTime = 0.0
		}

		// Finish the user tree task that was queued earlier in the time step. This must be complete before touching the
		// broad-phase.
		if world.userTreeTask != nil {
			world.finishTaskFcn(world.userTreeTask, world.userTaskContext)
			world.userTreeTask = nil
			world.activeTaskCount -= 1
		}

		ValidateNoMoved(&world.broadPhase)

		// Finalize bodies. Must happen after the constraint solver and after island splitting.
		ParallelFor(world, FinalizeBodiesTask, awakeBodyCount, 8, stepContext, "ccd")

		// Free in reverse order
		StackFree(&world.stack, graphBlocks)
		StackFree(&world.stack, jointBlocks)
		StackFree(&world.stack, meshBlocks)
		StackFree(&world.stack, convexBlocks)
		StackFree(&world.stack, bodyBlocks)
		StackFree(&world.stack, stages)
		StackFree(&world.stack, overflow.manifoldConstraints)
		StackFree(&world.stack, overflow.contactConstraints)
		StackFree(&world.stack, wideMeshConstraints)
		StackFree(&world.stack, meshManifoldStarts)
		StackFree(&world.stack, meshLaneOrder)
		StackFree(&world.stack, wideConstraints)

		world.profile.transforms = GetMilliseconds(transformTicks)
	}

	// Report joint events
	{
		jointEventTicks := GetTicks()

		// Gather bits for all joints that have force/torque events
		jointStateBitSet := &world.taskContexts.data[0].jointStateBitSet
		for i: i32 = 1; i < world.workerCount; i += 1 {
			InPlaceUnion(jointStateBitSet, &world.taskContexts.data[i].jointStateBitSet)
		}

		{
			wordCount := jointStateBitSet.blockCount
			bits := jointStateBitSet.bits

			jointArray := world.joints.data
			worldIndex0 := world.worldId

			for k: u32 = 0; k < wordCount; k += 1 {
				word := bits[k]
				for word != 0 {
					ctz := CTZ64(word)
					jointId := i32(64 * k + ctz)

					assert(jointId < world.joints.capacity)

					joint := &jointArray[jointId]

					assert(joint.setIndex == i32(SetType.awakeSet))

					event := JointEvent {
						jointId = {index1 = jointId + 1, world0 = worldIndex0, generation = joint.generation},
						userData = joint.userData,
					}

					Array_Push(&world.jointEvents, event)

					// Clear the smallest set bit
					word = word & (word - 1)
				}
			}
		}

		world.profile.jointEvents = GetMilliseconds(jointEventTicks)
	}

	// Report hit events
	{
		hitTicks := GetTicks()

		assert(world.contactHitEvents.count == 0)

		// Fast path: if no worker flagged any hit-event candidates during b2StoreImpulsesTask, skip entirely.
		anyHitEvents := false
		for i: i32 = 0; i < world.workerCount; i += 1 {
			if world.taskContexts.data[i].hasHitEvents {
				anyHitEvents = true
				break
			}
		}

		if anyHitEvents {
			// Union per-worker bits into worker 0's bit set.
			hitEventBitSet := &world.taskContexts.data[0].hitEventBitSet
			for i: i32 = 1; i < world.workerCount; i += 1 {
				if world.taskContexts.data[i].hasHitEvents {
					InPlaceUnion(hitEventBitSet, &world.taskContexts.data[i].hitEventBitSet)
				}
			}

			threshold := world.hitEventThreshold
			contactArray := world.contacts.data
			worldId := world.worldId

			wordCount := hitEventBitSet.blockCount
			bits := hitEventBitSet.bits
			for k: u32 = 0; k < wordCount; k += 1 {
				word := bits[k]
				for word != 0 {
					ctz := CTZ64(word)
					contactId := i32(64 * k + ctz)

					contact := &contactArray[contactId]
					assert(contact.setIndex == i32(SetType.awakeSet) && contact.colorIndex != NULL_INDEX)

					shapeA := Array_Get(&world.shapes, contact.shapeIdA)
					shapeB := Array_Get(&world.shapes, contact.shapeIdB)
					bodyA := Array_Get(&world.bodies, shapeA.bodyId)
					bodyB := Array_Get(&world.bodies, shapeB.bodyId)
					simA := GetBodySim(world, bodyA)
					simB := GetBodySim(world, bodyB)
					midCenter := LerpPosition(simA.center, simB.center, 0.5)

					event: ContactHitEvent = {}
					event.approachSpeed = threshold

					found := false
					triangleIndex: i32 = 0
					manifoldCount := contact.manifoldCount
					for i: i32 = 0; i < manifoldCount; i += 1 {
						manifold := &contact.manifolds[i]
						pointCount := manifold.pointCount
						for p: i32 = 0; p < pointCount; p += 1 {
							mp := &manifold.points[p]
							approachSpeed := -mp.normalVelocity

							// Need to check total impulse because the point may be speculative and not colliding
							if approachSpeed > event.approachSpeed && mp.totalNormalImpulse > 0.0 {
								event.approachSpeed = approachSpeed
								event.point = OffsetPos(midCenter, Lerp(mp.anchorA, mp.anchorB, 0.5))
								event.normal = manifold.normal
								triangleIndex = mp.triangleIndex
								found = true
							}
						}
					}

					if found == true {
						event.shapeIdA = ShapeId{shapeA.id + 1, worldId, shapeA.generation}
						event.shapeIdB = ShapeId{shapeB.id + 1, worldId, shapeB.generation}

						event.contactId = ContactId {
							index1     = contact.contactId + 1,
							world0     = worldId,
							padding    = 0,
							generation = contact.generation,
						}

						// shapeB is never a compound today (asserted in b3CreateContact), so the
						// childIndex argument is irrelevant for it. shapeA carries the compound.
						event.userMaterialIdA = GetShapeUserMaterialId(shapeA, contact.childIndex, triangleIndex)
						event.userMaterialIdB = GetShapeUserMaterialId(shapeB, 0, triangleIndex)

						Array_Push(&world.contactHitEvents, event)
					}

					// Clear the smallest set bit
					word = word & (word - 1)
				}
			}
		}

		world.profile.hitEvents = GetMilliseconds(hitTicks)
	}

	{
		refitTicks := GetTicks()

		DynamicTree_Refit(&world.broadPhase.trees[i32(BodyType.kinematicBody)])
		DynamicTree_Refit(&world.broadPhase.trees[i32(BodyType.dynamicBody)])

		ValidateBroadPhase(&world.broadPhase)

		world.profile.refit = GetMilliseconds(refitTicks)
	}

	bulletBodyCount := AtomicLoadInt(&stepContext.bulletBodyCount)
	if bulletBodyCount > 0 {
		bulletTicks := GetTicks()

		// Fast bullet bodies
		// Note: a bullet body may be moving slow
		minRange: i32 = 8
		ParallelFor(world, BulletBodyTask, bulletBodyCount, minRange, stepContext, "bullets")

		// Serially enlarge broad-phase proxies for bullet shapes
		broadPhase := &world.broadPhase
		dynamicTree := &broadPhase.trees[i32(BodyType.dynamicBody)]

		// Fast array access is important here
		bodyArray := world.bodies.data
		bodySimArray := awakeSet.bodySims.data
		shapeArray := world.shapes.data

		// Serially enlarge broad-phase proxies for bullet shapes
		bulletBodySimIndices := stepContext.bulletBodies

		// This loop has non-deterministic order but it shouldn't affect the result
		for i: i32 = 0; i < bulletBodyCount; i += 1 {
			bulletBodySim := &bodySimArray[bulletBodySimIndices[i]]
			if (bulletBodySim.flags & enlargeBulletBounds) == 0 {
				continue
			}

			// Clear flag
			bulletBodySim.flags &~= enlargeBulletBounds

			bodyId := bulletBodySim.bodyId
			assert(0 <= bodyId && bodyId < world.bodies.count)
			bulletBody := &bodyArray[bodyId]

			shapeId := bulletBody.headShapeId
			for shapeId != NULL_INDEX {
				shape := &shapeArray[shapeId]

				proxyKey := shape.proxyKey
				proxyId := PROXY_ID(proxyKey)
				VALIDATE(PROXY_TYPE(proxyKey) == .dynamicBody)

				treeAABB := DynamicTree_GetAABB(dynamicTree, proxyId)
				shapeFatAABB := world.fatAABBs.data[shapeId]

				if AABB_Contains(treeAABB, shapeFatAABB) == false {
					DynamicTree_EnlargeProxy(dynamicTree, proxyId, shapeFatAABB)
				}

				shapeId = shape.nextShapeId
			}
		}

		world.profile.bullets = GetMilliseconds(bulletTicks)
	}

	StackFree(&world.stack, stepContext.bulletBodies)
	stepContext.bulletBodies = nil
	AtomicStoreInt(&stepContext.bulletBodyCount, 0)

	// Report sensor hits. This may include bullets sensor hits.
	{
		sensorHitTicks := GetTicks()

		workerCount := world.workerCount
		assert(workerCount == world.taskContexts.count)

		for i: i32 = 0; i < workerCount; i += 1 {
			taskContext := &world.taskContexts.data[i]
			hitCount := taskContext.sensorHits.count
			hits := taskContext.sensorHits.data

			for j: i32 = 0; j < hitCount; j += 1 {
				hit := hits[j]
				sensorShape := Array_Get(&world.shapes, hit.sensorId)
				visitor := Array_Get(&world.shapes, hit.visitorId)

				sensor := Array_Get(&world.sensors, sensorShape.sensorIndex)
				shapeRef := Visitor {
					shapeId    = hit.visitorId,
					generation = visitor.generation,
				}
				Array_Push(&sensor.hits, shapeRef)
			}
		}

		world.profile.sensorHits = GetMilliseconds(sensorHitTicks)
	}

	// Island sleeping
	// This must be done last because putting islands to sleep invalidates the enlarged body bits.
	// todo_erin figure out how to do this in parallel with tree refit
	if world.enableSleep == true {
		sleepTicks := GetTicks()

		// Collect split island candidate for the next time step. No need to split if sleeping is disabled.
		assert(world.splitIslandId == NULL_INDEX)
		splitSleepTimer: f32 = 0.0
		for i: i32 = 0; i < world.workerCount; i += 1 {
			taskContext := &world.taskContexts.data[i]
			if taskContext.splitIslandId != NULL_INDEX && taskContext.splitSleepTime >= splitSleepTimer {
				assert(taskContext.splitSleepTime > 0.0)

				// Tie breaking for determinism. Largest island id wins. Needed due to work stealing.
				if taskContext.splitSleepTime == splitSleepTimer && taskContext.splitIslandId < world.splitIslandId {
					continue
				}

				world.splitIslandId = taskContext.splitIslandId
				splitSleepTimer = taskContext.splitSleepTime
			}
		}

		awakeIslandBitSet := &world.taskContexts.data[0].awakeIslandBitSet
		for i: i32 = 1; i < world.workerCount; i += 1 {
			InPlaceUnion(awakeIslandBitSet, &world.taskContexts.data[i].awakeIslandBitSet)
		}

		// Need to process in reverse because this moves islands to sleeping solver sets.
		islands := awakeSet.islandSims.data
		count := awakeSet.islandSims.count
		for islandIndex := count - 1; islandIndex >= 0; islandIndex -= 1 {
			if GetBit(awakeIslandBitSet, u32(islandIndex)) == true {
				// this island is still awake
				continue
			}

			island := &islands[islandIndex]
			islandId := island.islandId

			TrySleepIsland(world, islandId)
		}

		ValidateSolverSets(world)

		world.profile.sleepIslands = GetMilliseconds(sleepTicks)
	}
}
