// Ported from box3d test/test_restitution.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import "base:intrinsics"
import "core:testing"

import b3 "../../box3d"

@(private = "file")
TIME_STEP :: f32(1.0) / f32(60.0)
@(private = "file")
SUB_STEP_COUNT :: 4

// A flat landing converges its Gauss-Seidel coupling in four stage iterations and the default is two,
// so the tests that read the coefficient or the spin of a flat landing pin the count
@(private = "file")
RESTITUTION_ITERATIONS :: 4

// Impact speed shared by every scenario that measures a coefficient. Well above the default
// restitution threshold so the bounce is always armed.
@(private = "file")
IMPACT_SPEED :: f32(5.0)

// Each subtest prints all of its measurements before asserting, and the battery runs every subtest
// so one bad scenario does not hide the rest.
// unified: RUN_MEASUREMENT is RUN_SUBTEST without the early return (see RestitutionTest).

@(private = "file")
MakeWorld :: proc(gravityY: f32) -> b3.WorldId {
	worldDef := b3.DefaultWorldDef()
	worldDef.gravity = b3.Vec3{0.0, gravityY, 0.0}
	worldDef.enableSleep = false
	return b3.CreateWorld(&worldDef)
}

@(private = "file")
MakeWorldIterations :: proc(gravityY: f32, restitutionIterations: i32) -> b3.WorldId {
	worldDef := b3.DefaultWorldDef()
	worldDef.gravity = b3.Vec3{0.0, gravityY, 0.0}
	worldDef.enableSleep = false
	worldDef.restitutionIterations = restitutionIterations
	return b3.CreateWorld(&worldDef)
}

// Ground with its top surface at y = 0
@(private = "file")
MakeGround :: proc(worldId: b3.WorldId, restitution: f32) {
	bodyDef := b3.DefaultBodyDef()
	bodyDef.position = b3.Pos{0.0, -1.0, 0.0}
	groundId := b3.CreateBody(worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()
	shapeDef.baseMaterial.friction = 0.0
	shapeDef.baseMaterial.restitution = restitution
	box := b3.MakeBoxHull(40.0, 1.0, 40.0)
	b3.CreateHullShape(groundId, &shapeDef, &box.base)
}

@(private = "file")
MakeBall :: proc(worldId: b3.WorldId, x: f32, y: f32, velocityY: f32, restitution: f32) -> b3.BodyId {
	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = b3.Pos{x, y, 0.0}
	bodyDef.linearVelocity = b3.Vec3{0.0, velocityY, 0.0}
	bodyId := b3.CreateBody(worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()
	shapeDef.baseMaterial.friction = 0.0
	shapeDef.baseMaterial.restitution = restitution
	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}
	b3.CreateSphereShape(bodyId, &shapeDef, &sphere)
	return bodyId
}

// Two free spheres closing head on with no gravity. The coefficient is the ratio of relative normal
// speeds at the contact, which is the mass independent definition.
@(private = "file")
MeasureHeadOn :: proc(restitution: f32, densityB: f32, momentumError: ^f32) -> f32 {
	worldId := MakeWorld(0.0)

	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}
	shapeDef := b3.DefaultShapeDef()
	shapeDef.density = 1.0
	shapeDef.baseMaterial.friction = 0.0
	shapeDef.baseMaterial.restitution = restitution

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = b3.Pos{-1.0, 0.0, 0.0}
	bodyDef.linearVelocity = b3.Vec3{0.5 * IMPACT_SPEED, 0.0, 0.0}
	idA := b3.CreateBody(worldId, &bodyDef)
	b3.CreateSphereShape(idA, &shapeDef, &sphere)

	bodyDef.position = b3.Pos{1.0, 0.0, 0.0}
	bodyDef.linearVelocity = b3.Vec3{-0.5 * IMPACT_SPEED, 0.0, 0.0}
	idB := b3.CreateBody(worldId, &bodyDef)
	shapeDef.density = densityB
	b3.CreateSphereShape(idB, &shapeDef, &sphere)

	massA := b3.Body_GetMass(idA)
	massB := b3.Body_GetMass(idB)
	momentum0 := massA * b3.Body_GetLinearVelocity(idA).x + massB * b3.Body_GetLinearVelocity(idB).x

	for i: i32 = 0; i < 120; i += 1 {
		b3.World_Step(worldId, TIME_STEP, SUB_STEP_COUNT)
	}

	vA := b3.Body_GetLinearVelocity(idA).x
	vB := b3.Body_GetLinearVelocity(idB).x
	momentum1 := massA * vA + massB * vB

	b3.DestroyWorld(worldId)

	momentumError^ = (momentum1 - momentum0) / (massA + massB)
	return (vB - vA) / IMPACT_SPEED
}

// Ball driven into static ground with no gravity so the measurement carries no gravity bias. The
// gap shifts where inside the time step the impact lands.
@(private = "file")
MeasureGroundBounce :: proc(restitution: f32, gap: f32) -> f32 {
	worldId := MakeWorld(0.0)
	MakeGround(worldId, 0.0)

	ballId := MakeBall(worldId, 0.0, 0.5 + gap, -IMPACT_SPEED, restitution)

	for i: i32 = 0; i < 60; i += 1 {
		b3.World_Step(worldId, TIME_STEP, SUB_STEP_COUNT)
	}

	speed := b3.Body_GetLinearVelocity(ballId).y
	b3.DestroyWorld(worldId)
	return speed / IMPACT_SPEED
}

// Impactor driven onto a column of resting balls pinned against the ground. The support balls are
// dead so only the top contact can bounce. A supported target has infinite effective mass along the
// normal, so the coefficient must not depend on the column height.
@(private = "file")
MeasureSupportedBounce :: proc(restitution: f32, supportCount: i32, propagate: bool) -> f32 {
	worldDef := b3.DefaultWorldDef()
	worldDef.gravity = b3.Vec3_zero
	worldDef.enableSleep = false
	worldDef.enableRestitutionPropagation = propagate
	worldDef.restitutionIterations = RESTITUTION_ITERATIONS
	worldId := b3.CreateWorld(&worldDef)
	MakeGround(worldId, 0.0)

	for i: i32 = 0; i < supportCount; i += 1 {
		MakeBall(worldId, 0.0, 0.5 + 1.0 * f32(i), 0.0, 0.0)
	}

	y := 0.5 + 1.0 * f32(supportCount) + f32(0.5) * IMPACT_SPEED * TIME_STEP
	ballId := MakeBall(worldId, 0.0, y, -IMPACT_SPEED, restitution)

	for i: i32 = 0; i < 60; i += 1 {
		b3.World_Step(worldId, TIME_STEP, SUB_STEP_COUNT)
	}

	speed := b3.Body_GetLinearVelocity(ballId).y
	b3.DestroyWorld(worldId)
	return speed / IMPACT_SPEED
}

// Flat box landing on all four corners at once with no gravity
@(private = "file")
MeasureFlatBounce :: proc(restitution: f32, subStepCount: i32, spin: ^f32) -> f32 {
	worldId := MakeWorldIterations(0.0, RESTITUTION_ITERATIONS)
	MakeGround(worldId, 0.0)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = b3.Pos{0.0, f32(0.25) + f32(0.5) * IMPACT_SPEED * TIME_STEP, 0.0}
	bodyDef.linearVelocity = b3.Vec3{0.0, -IMPACT_SPEED, 0.0}
	boxId := b3.CreateBody(worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()
	shapeDef.density = 1.0
	shapeDef.baseMaterial.friction = 0.0
	shapeDef.baseMaterial.restitution = restitution
	box := b3.MakeBoxHull(1.0, 0.25, 1.0)
	b3.CreateHullShape(boxId, &shapeDef, &box.base)

	for i: i32 = 0; i < 60; i += 1 {
		b3.World_Step(worldId, TIME_STEP, subStepCount)
	}

	speed := b3.Body_GetLinearVelocity(boxId).y
	spin^ = b3.Length(b3.Body_GetAngularVelocity(boxId))
	b3.DestroyWorld(worldId)
	return speed / IMPACT_SPEED
}

// Perfectly elastic ball dropped under gravity
@(private = "file")
MeasureDrop :: proc(dropHeight: f32, apexes: [^]f32, capacity: i32) -> i32 {
	worldId := MakeWorld(-10.0)
	MakeGround(worldId, 0.0)

	ballId := MakeBall(worldId, 0.0, dropHeight, 0.0, 1.0)

	apexCount: i32 = 0
	previousSpeed: f32 = 0.0

	for i: i32 = 0; i < 4000 && apexCount < capacity; i += 1 {
		b3.World_Step(worldId, TIME_STEP, SUB_STEP_COUNT)

		speed := b3.Body_GetLinearVelocity(ballId).y
		if previousSpeed > 0.0 && speed <= 0.0 {
			apexes[apexCount] = f32(b3.Body_GetPosition(ballId).y)
			apexCount += 1
		}
		previousSpeed = speed
	}

	b3.DestroyWorld(worldId)
	return apexCount
}

@(private = "file")
HeadOnTest :: proc(t: ^testing.T) -> bool {
	restitutions := [?]f32{0.0, 0.25, 0.5, 0.75, 1.0}
	densities := [?]f32{1.0, 10.0, 100.0}
	tolerance: f32 = 0.02

	failed := 0

	for i: i32 = 0; i < i32(len(densities)); i += 1 {
		worstError: f32 = 0.0
		worstMomentum: f32 = 0.0

		for j: i32 = 0; j < i32(len(restitutions)); j += 1 {
			momentumError: f32 = 0.0
			measured := MeasureHeadOn(restitutions[j], densities[i], &momentumError)

			error := b3.AbsFloat(measured - restitutions[j])
			if error > worstError {
				worstError = error
			}

			if b3.AbsFloat(momentumError) > b3.AbsFloat(worstMomentum) {
				worstMomentum = momentumError
			}

			if error > tolerance {
				printf("    head on density %.0f e %.2f -> %.4f\n", densities[i], restitutions[j], measured)
				failed = 1
			}
		}

		printf(
			"    head on density %6.1f worst error %.4f momentum drift %.2e\n",
			densities[i],
			worstError,
			worstMomentum,
		)

		if b3.AbsFloat(worstMomentum) > 1.0e-4 {
			failed = 1
		}
	}

	return failed == 0
}

@(private = "file")
PhaseTest :: proc(t: ^testing.T) -> bool {
	restitutions := [?]f32{0.25, 0.5, 0.9}
	sampleCount: i32 = 16
	tolerance: f32 = 0.05

	failed := 0

	for j: i32 = 0; j < i32(len(restitutions)); j += 1 {
		minimum := b3.FLT_MAX
		maximum := -b3.FLT_MAX
		sum: f32 = 0.0

		for i: i32 = 0; i < sampleCount; i += 1 {
			// Sweep the gap across one step of travel so the impact lands at every phase
			gap := (f32(i) / f32(sampleCount)) * IMPACT_SPEED * TIME_STEP
			measured := MeasureGroundBounce(restitutions[j], gap)
			minimum = b3.MinFloat(minimum, measured)
			maximum = b3.MaxFloat(maximum, measured)
			sum += measured
		}

		mean := sum / f32(sampleCount)
		spread := maximum - minimum
		printf(
			"    phase e %.2f -> mean %.4f spread %.4f [%.4f, %.4f]\n",
			restitutions[j],
			mean,
			spread,
			minimum,
			maximum,
		)

		if spread > tolerance || b3.AbsFloat(mean - restitutions[j]) > tolerance {
			failed = 1
		}
	}

	return failed == 0
}

// By default the restitution stage sweeps only the contacts that carry restitution, so the ball
// pushes the column down and the ground under it gets no say until the next step: e = 0.9 gives
// about 0.52 on one support. With propagation enabled the stage sweeps every contact, the ground
// contact supplies the reaction within the stage and e = 0.9 gives 0.85 on one support and 0.72 on
// three at the pinned four iterations, decaying with depth because the column needs an iteration per
// level. Propagation costs a normal pass over every contact per iteration, hence the option. Both
// modes are gated: never above the free coefficient, which would be energy from nowhere, and never
// collapsed.
@(private = "file")
SupportedTest :: proc(t: ^testing.T) -> bool {
	restitutions := [?]f32{0.5, 0.9}

	failed := 0

	for propagate: i32 = 0; propagate <= 1; propagate += 1 {
		floor: f32 = propagate != 0 ? 0.75 : 0.5

		for j: i32 = 0; j < i32(len(restitutions)); j += 1 {
			for n: i32 = 0; n <= 3; n += 1 {
				measured := MeasureSupportedBounce(restitutions[j], n, propagate == 1)
				printf(
					"    supported e %.2f supports %d propagation %d -> %.4f\n",
					restitutions[j],
					n,
					propagate,
					measured,
				)

				if measured > restitutions[j] + 0.01 || measured < floor * restitutions[j] {
					failed = 1
				}
			}
		}
	}

	return failed == 0
}

// A symmetric four point landing. The stage solves the points in sequence, so a single sweep leaves
// the box spinning: the first impulse tilts it and the next point sees a larger closing speed. Four
// stage iterations converge the coupling and leave a small residual. The tolerance admits that
// residual and rejects the single sweep behavior.
@(private = "file")
FlatLandingTest :: proc(t: ^testing.T) -> bool {
	restitutions := [?]f32{0.5, 0.9}
	subStepCounts := [?]i32{4, 8}
	speedTolerance: f32 = 0.01
	spinTolerance: f32 = 0.03

	failed := 0

	for j: i32 = 0; j < i32(len(restitutions)); j += 1 {
		for k: i32 = 0; k < i32(len(subStepCounts)); k += 1 {
			spin: f32 = 0.0
			measured := MeasureFlatBounce(restitutions[j], subStepCounts[k], &spin)
			printf(
				"    flat e %.2f substeps %d -> %.4f spin %.4f\n",
				restitutions[j],
				subStepCounts[k],
				measured,
				spin,
			)

			// Only the shipping sub step count is a gate. The wider count is reported so a
			// convergence problem can be told apart from a formulation problem.
			if subStepCounts[k] != SUB_STEP_COUNT {
				continue
			}

			if b3.AbsFloat(measured - restitutions[j]) > speedTolerance || spin > spinTolerance {
				failed = 1
			}
		}
	}

	return failed == 0
}

// A flat box carrying spin about a horizontal axis. All four corners stay in contact and the normal
// constraints are linear in the body velocity, so perfect restitution reverses linear and angular
// velocity. Four stage iterations bring both within a few thousandths.
@(private = "file")
SpinTest :: proc(t: ^testing.T) -> bool {
	spins := [?]f32{0.0, 0.5, 1.0, 2.0}

	failed := 0

	for j: i32 = 0; j < i32(len(spins)); j += 1 {
		worldId := MakeWorldIterations(0.0, RESTITUTION_ITERATIONS)
		MakeGround(worldId, 0.0)

		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{0.0, f32(0.25) + f32(0.5) * IMPACT_SPEED * TIME_STEP, 0.0}
		bodyDef.linearVelocity = b3.Vec3{0.0, -IMPACT_SPEED, 0.0}
		bodyDef.angularVelocity = b3.Vec3{0.0, 0.0, spins[j]}
		boxId := b3.CreateBody(worldId, &bodyDef)

		shapeDef := b3.DefaultShapeDef()
		shapeDef.density = 1.0
		shapeDef.baseMaterial.friction = 0.0
		shapeDef.baseMaterial.restitution = 1.0
		box := b3.MakeBoxHull(1.0, 0.25, 1.0)
		b3.CreateHullShape(boxId, &shapeDef, &box.base)

		for i: i32 = 0; i < 60; i += 1 {
			b3.World_Step(worldId, TIME_STEP, SUB_STEP_COUNT)
		}

		speed := b3.Body_GetLinearVelocity(boxId).y
		spin := b3.Body_GetAngularVelocity(boxId).z
		b3.DestroyWorld(worldId)

		printf(
			"    spin in %+.2f -> vy %+.4f (want %+.4f)  wz %+.4f (want %+.4f)\n",
			spins[j],
			speed,
			IMPACT_SPEED,
			spin,
			-spins[j],
		)

		if b3.AbsFloat(speed - IMPACT_SPEED) > 0.05 || b3.AbsFloat(spin + spins[j]) > 0.05 {
			failed = 1
		}
	}

	return failed == 0
}

// Perfectly elastic ball under gravity. Only heights where continuous collision engages are used.
// Continuous collision lands the ball on the surface, so the bounce is armed from the true impact
// speed and the apex holds. Slower drops resolve the impact inside the overlap and the penetration
// recovery adds height, by design. The 10 m case is the long horizon read.
@(private = "file")
DropTest :: proc(t: ^testing.T) -> bool {
	heights := [?]f32{40.0, 20.0, 10.0}

	failed := 0

	for j: i32 = 0; j < i32(len(heights)); j += 1 {
		apexes: [6]f32 = {}
		apexCount := MeasureDrop(heights[j], &apexes[0], i32(len(apexes)))

		printf("    drop %5.1f ->", heights[j])
		for i: i32 = 0; i < apexCount; i += 1 {
			printf(" %8.3f", apexes[i])
		}
		printf("\n")

		if apexCount < i32(len(apexes)) {
			failed = 1
			continue
		}

		highest := apexes[0]
		for i: i32 = 1; i < apexCount; i += 1 {
			highest = b3.MaxFloat(highest, apexes[i])
		}

		// A later apex above the first, or a first apex above the drop, is energy from nowhere
		if highest > 1.001 * apexes[0] || apexes[0] > 1.001 * heights[j] || apexes[apexCount - 1] < 0.8 * apexes[0] {
			failed = 1
		}
	}

	return failed == 0
}

// A cube dropped flat with an aggressive continuous safety factor so it lands square on the surface.
// Perfectly elastic and frictionless, so it should come back to the drop height with no rotation. A
// flat landing solves four coplanar points in sequence and one sweep over-delivers: the first impulse
// tilts the cube, the next point sees a larger closing speed and the sum overshoots the rigid body
// answer, leaving spin that tilts the cube for the next landing. Four stage iterations converge the
// coupling. The bounds below reject the single sweep behavior and any apex above the drop.
@(private = "file")
SingleBoxTest :: proc(t: ^testing.T) -> bool {
	worldId := MakeWorldIterations(-10.0, RESTITUTION_ITERATIONS)
	MakeGround(worldId, 0.0)

	shapeDef := b3.DefaultShapeDef()
	shapeDef.density = 1.0
	shapeDef.baseMaterial.friction = 0.0
	shapeDef.baseMaterial.restitution = 1.0
	box := b3.MakeCubeHull(0.5)

	dropHeight: f32 = 10.0

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = b3.Pos{0.0, dropHeight, 0.0}
	bodyDef.safetyFactor = 0.01
	boxId := b3.CreateBody(worldId, &bodyDef)
	b3.CreateHullShape(boxId, &shapeDef, &box.base)

	firstSpin: f32 = 0.0
	apexes: [4]f32 = {}
	spins: [4]f32 = {}
	apexCount: i32 = 0
	previousSpeed: f32 = 0.0

	// A nearly elastic bounce from 10 m takes about 165 steps, so budget for four of them
	for i: i32 = 0; i < 1200 && apexCount < i32(len(apexes)); i += 1 {
		b3.World_Step(worldId, TIME_STEP, SUB_STEP_COUNT)

		speed := b3.Body_GetLinearVelocity(boxId).y

		if apexCount == 0 && previousSpeed <= 0.0 && speed > 0.0 {
			firstSpin = b3.Length(b3.Body_GetAngularVelocity(boxId))
		}

		if previousSpeed > 0.0 && speed <= 0.0 {
			apexes[apexCount] = f32(b3.Body_GetPosition(boxId).y)
			spins[apexCount] = b3.Length(b3.Body_GetAngularVelocity(boxId))
			apexCount += 1
		}
		previousSpeed = speed
	}

	b3.DestroyWorld(worldId)

	printf("    single box first bounce spin %.4f apexes", firstSpin)
	for i: i32 = 0; i < apexCount; i += 1 {
		printf(" %7.3f (spin %5.2f)", apexes[i], spins[i])
	}
	printf("\n")

	ENSURE(t, apexCount == i32(len(apexes))) or_return

	failed := 0

	if firstSpin > 0.01 {
		failed = 1
	}

	for i: i32 = 0; i < apexCount; i += 1 {
		if apexes[i] < 0.95 * dropHeight || apexes[i] > 1.001 * dropHeight {
			failed = 1
		}
	}

	return failed == 0
}

@(private = "file")
ThresholdTest :: proc(t: ^testing.T) -> bool {
	worldDef := b3.DefaultWorldDef()
	threshold := worldDef.restitutionThreshold

	scales := [?]f32{0.5, 0.9, 1.5, 4.0}

	failed := 0

	for j: i32 = 0; j < i32(len(scales)); j += 1 {
		speed := scales[j] * threshold

		worldId := MakeWorld(0.0)
		MakeGround(worldId, 0.0)
		ballId := MakeBall(worldId, 0.0, 0.5 + 0.5 * speed * TIME_STEP, -speed, 1.0)

		for i: i32 = 0; i < 60; i += 1 {
			b3.World_Step(worldId, TIME_STEP, SUB_STEP_COUNT)
		}

		ratio := b3.Body_GetLinearVelocity(ballId).y / speed
		b3.DestroyWorld(worldId)

		printf("    threshold %.2fx -> %.4f\n", scales[j], ratio)

		if scales[j] < 1.0 && ratio > 0.05 {
			failed = 1
		}

		if scales[j] > 1.0 && ratio < 0.9 {
			failed = 1
		}
	}

	return failed == 0
}

// High restitution must not wake a settled stack back up
@(private = "file")
RestingTest :: proc(t: ^testing.T) -> bool {
	worldId := MakeWorld(-10.0)
	MakeGround(worldId, 0.9)

	boxCount: i32 = 10
	boxIds: [10]b3.BodyId

	shapeDef := b3.DefaultShapeDef()
	shapeDef.density = 1.0
	shapeDef.baseMaterial.friction = 0.6
	shapeDef.baseMaterial.restitution = 0.9
	box := b3.MakeCubeHull(0.5)

	for i: i32 = 0; i < boxCount; i += 1 {
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{0.0, 0.5 + 1.0 * f32(i), 0.0}
		boxIds[i] = b3.CreateBody(worldId, &bodyDef)
		b3.CreateHullShape(boxIds[i], &shapeDef, &box.base)
	}

	for i: i32 = 0; i < 240; i += 1 {
		b3.World_Step(worldId, TIME_STEP, SUB_STEP_COUNT)
	}

	settled := f32(b3.Body_GetPosition(boxIds[boxCount - 1]).y)
	drift: f32 = 0.0
	peakSpeed: f32 = 0.0

	for i: i32 = 0; i < 300; i += 1 {
		b3.World_Step(worldId, TIME_STEP, SUB_STEP_COUNT)

		y := f32(b3.Body_GetPosition(boxIds[boxCount - 1]).y)
		drift = b3.MaxFloat(drift, b3.AbsFloat(y - settled))

		for j: i32 = 0; j < boxCount; j += 1 {
			peakSpeed = b3.MaxFloat(peakSpeed, b3.Length(b3.Body_GetLinearVelocity(boxIds[j])))
		}
	}

	b3.DestroyWorld(worldId)

	printf("    resting drift %.5f peak speed %.5f\n", drift, peakSpeed)

	ENSURE(t, drift < 0.01) or_return
	ENSURE(t, peakSpeed < 0.05) or_return
	return true
}

@(private = "file")
RunWorkerScene :: proc(workerCount: i32) -> u64 {
	worldDef := b3.DefaultWorldDef()
	worldDef.gravity = b3.Vec3{0.0, -10.0, 0.0}
	worldDef.enableSleep = false
	worldDef.workerCount = u32(workerCount)
	worldId := b3.CreateWorld(&worldDef)

	MakeGround(worldId, 0.6)

	for i: i32 = 0; i < 20; i += 1 {
		MakeBall(worldId, -10.0 + 1.05 * f32(i), 3.0 + 0.13 * f32(i), 0.0, 0.6)
	}

	for i: i32 = 0; i < 200; i += 1 {
		b3.World_Step(worldId, TIME_STEP, SUB_STEP_COUNT)
	}

	hash := b3.HashWorldState(b3.GetWorldFromId(worldId))
	b3.DestroyWorld(worldId)
	return hash
}

// The manifold point approach speed is published only for contacts that opted into hit events. It
// is not a stale value otherwise, it is zero, so a reader can tell "not measured" from "measured
// zero" by whether the shape enables the events. Restitution no longer reads it at all, so this is
// the only thing keeping the field alive.
@(private = "file")
MeasureFirstTouchNormalVelocity :: proc(enableHitEvents: bool) -> f32 {
	worldId := MakeWorld(0.0)
	MakeGround(worldId, 0.0)

	bodyDef := b3.DefaultBodyDef()
	bodyDef.type = .dynamicBody
	bodyDef.position = b3.Pos{0.0, 0.6, 0.0}
	ballId := b3.CreateBody(worldId, &bodyDef)

	shapeDef := b3.DefaultShapeDef()
	shapeDef.baseMaterial.friction = 0.0
	shapeDef.baseMaterial.restitution = 0.0
	shapeDef.enableHitEvents = enableHitEvents
	sphere := b3.Sphere{{0.0, 0.0, 0.0}, 0.5}
	b3.CreateSphereShape(ballId, &shapeDef, &sphere)

	normalVelocity := b3.FLT_MAX

	// Hold the closing speed so the sampled value does not depend on which step the pair is created
	for i: i32 = 0; i < 20; i += 1 {
		b3.Body_SetLinearVelocity(ballId, b3.Vec3{0.0, -IMPACT_SPEED, 0.0})
		b3.World_Step(worldId, TIME_STEP, SUB_STEP_COUNT)

		contactData: b3.ContactData
		if b3.Body_GetContactData(ballId, &contactData, 1) == 1 &&
		   contactData.manifoldCount > 0 &&
		   contactData.manifolds[0].pointCount > 0 {
			normalVelocity = contactData.manifolds[0].points[0].normalVelocity
			break
		}
	}

	b3.DestroyWorld(worldId)
	return normalVelocity
}

@(private = "file")
NormalVelocityTest :: proc(t: ^testing.T) -> bool {
	quiet := MeasureFirstTouchNormalVelocity(false)
	published := MeasureFirstTouchNormalVelocity(true)

	printf("    normal velocity hit events off %.4f on %.4f (closing at %.4f)\n", quiet, published, -IMPACT_SPEED)

	failed := 0

	if quiet != 0.0 {
		failed = 1
	}

	if published > -0.8 * IMPACT_SPEED {
		failed = 1
	}

	return failed == 0
}

// Mechanical energy of a body as symplectic Euler conserves it. The integrator updates velocity
// before position, so the quantity that is exactly constant in free flight is not kinetic plus
// potential but that plus half a sub-step of the gravity power, m * dot(v, g) * h / 2. Plain
// kinetic plus potential sampled at step boundaries jumps by about g * v * h at every bounce and
// drifts down by g^2 h^2 / 2 each sub-step, which would hide a real gain or fake one. With this
// measure a contact that never does positive work leaves the energy flat to round-off.
@(private = "file")
MeasureEnergy :: proc(bodyId: b3.BodyId, worldId: b3.WorldId) -> f32 {
	massData := b3.Body_GetMassData(bodyId)
	v := b3.Body_GetLinearVelocity(bodyId)
	gravity := b3.MulSV(b3.Body_GetGravityScale(bodyId), b3.World_GetGravity(worldId))
	h := TIME_STEP / SUB_STEP_COUNT

	// The inertia tensor is in the body frame, so bring the angular velocity back to it
	wLocal := b3.InvRotateVector(b3.Body_GetRotation(bodyId), b3.Body_GetAngularVelocity(bodyId))

	kinetic := 0.5 * massData.mass * b3.Dot(v, v) + 0.5 * b3.Dot(wLocal, b3.MulMV(massData.inertia, wLocal))
	potential := -massData.mass * b3.Dot(gravity, b3.ToVec3(b3.Body_GetWorldCenter(bodyId)))
	phase := 0.5 * h * massData.mass * b3.Dot(v, gravity)

	return kinetic + potential + phase
}

@(private = "file")
OvershootResult :: struct {
	firstApex:        f32,
	firstEnergyRatio: f32,
	peakEnergyRatio:  f32,
	speedRatio:       f32,
	bounceCount:      i32,
}

// Mirrors the Restitution Overshoot sample in samples/sample_issues.cpp: a unit cube dropped flat on
// a floor narrower than itself, perfectly elastic, so the four contact points sit inboard of the box
// corners. A perfectly elastic bounce cannot come back higher than it was dropped from and cannot
// gain energy.
@(private = "file")
MeasureOvershoot :: proc(continuous: bool) -> OvershootResult {
	worldId := MakeWorldIterations(-10.0, RESTITUTION_ITERATIONS)

	floorDef := b3.DefaultBodyDef()
	floorDef.position = b3.Pos{0.0, -0.25, 0.0}
	floorId := b3.CreateBody(worldId, &floorDef)

	floorShape := b3.DefaultShapeDef()
	floor := b3.MakeBoxHull(0.375, 0.25, 0.375)
	b3.CreateHullShape(floorId, &floorShape, &floor.base)

	dropHeight: f32 = 10.0

	boxDef := b3.DefaultBodyDef()
	boxDef.type = .dynamicBody
	boxDef.position = b3.Pos{0.0, dropHeight, 0.0}
	if continuous {
		boxDef.safetyFactor = 0.1
	}
	boxId := b3.CreateBody(worldId, &boxDef)

	boxShape := b3.DefaultShapeDef()
	boxShape.baseMaterial.restitution = 1.0
	box := b3.MakeCubeHull(0.5)
	b3.CreateHullShape(boxId, &boxShape, &box.base)

	result: OvershootResult = {}

	startEnergy := MeasureEnergy(boxId, worldId)
	impactSpeed: f32 = 0.0
	reboundSpeed: f32 = 0.0
	bounceHeight: f32 = 0.0
	previousSpeed: f32 = 0.0

	for i: i32 = 0; i < 600 && result.bounceCount < 2; i += 1 {
		before := b3.Body_GetLinearVelocity(boxId).y
		heightBefore := f32(b3.Body_GetPosition(boxId).y)
		b3.World_Step(worldId, TIME_STEP, SUB_STEP_COUNT)
		speed := b3.Body_GetLinearVelocity(boxId).y

		if reboundSpeed == 0.0 && before < 0.0 && speed > 0.0 {
			impactSpeed = -before
			reboundSpeed = speed
			bounceHeight = heightBefore
		}

		if reboundSpeed == 0.0 {
			continue
		}

		// Every step, not just at the apexes, because that is what the sample checks and the impact
		// step itself is where any gain appears
		ratio := MeasureEnergy(boxId, worldId) / startEnergy
		if ratio > result.peakEnergyRatio {
			result.peakEnergyRatio = ratio
		}

		if previousSpeed > 0.0 && speed <= 0.0 {
			if result.bounceCount == 0 {
				result.firstApex = f32(b3.Body_GetPosition(boxId).y)
				result.firstEnergyRatio = ratio
			}

			result.bounceCount += 1
		}

		previousSpeed = speed
	}

	b3.DestroyWorld(worldId)

	result.speedRatio = reboundSpeed / impactSpeed

	printf(
		"    overshoot continuous %d apex %.4f of %.4f at y %.4f, speed ratio %.4f, energy %.4f then %.4f\n",
		continuous ? 1 : 0,
		result.firstApex,
		dropHeight,
		bounceHeight,
		result.speedRatio,
		result.firstEnergyRatio,
		result.peakEnergyRatio,
	)

	return result
}

// Two runs, because two unrelated effects were tangled together here.
//
// The impulse must never return more speed than it received. That is the restitution invariant and it
// is checked in both runs; the unconverged four point solve failed it by 7.9 percent.
//
// Everything else depends on whether continuous collision engages. Without it the cube moves 0.23 m
// in the step before contact, just under the 0.25 m trigger, so the first manifold appears with the
// cube already 0.10 m deep. It bounces from down there and climbs back through that depth for free,
// which is worth about a tenth of a metre of apex and one percent of energy, and none of it is
// restitution. With continuous collision the cube lands on the surface and both the apex and the
// energy come in under where they started. The safety factor here matches the one the sample sets,
// which is the supported answer for a body whose elastic accuracy matters.
//
// The discrete run is only gated on its first bounce. Later bounces there are a genuine blow-up, not
// a tolerance question: the free height makes the cube tilt, the next landing is a deep corner impact
// at 14 m/s, and energy reaches 159 percent with the cube spinning at 26 rad/s. Lowering the safety
// factor is the fix and the sample does that; the discrete run is kept to pin what happens when it is
// left alone, and because its speed ratio is what caught the real bug. Details in
// .claude/same_step_restitution.md.
@(private = "file")
OvershootTest :: proc(t: ^testing.T) -> bool {
	dropHeight: f32 = 10.0

	discrete := MeasureOvershoot(false)
	continuous := MeasureOvershoot(true)

	// Perfectly elastic returns exactly the speed it received, so any excess is round-off. The
	// overflow and colored solvers round differently and either can land an ulp over one.
	speedTolerance: f32 = 1.0e-5

	failed := 0

	if discrete.speedRatio > 1.0 + speedTolerance || continuous.speedRatio > 1.0 + speedTolerance {
		failed = 1
	}

	if continuous.firstApex > dropHeight || continuous.peakEnergyRatio > 1.001 {
		failed = 1
	}

	if discrete.firstApex > dropHeight + 0.15 || discrete.firstEnergyRatio > 1.02 {
		failed = 1
	}

	// The continuous landing leaves the cube sliding sideways a few tenths of a metre per second, from
	// friction during the sub-steps, and a floor narrower than the cube does not catch it on the way
	// back down. That is sub-step friction, not restitution, so only the first bounce is required.
	if discrete.bounceCount < 2 || continuous.bounceCount < 1 {
		failed = 1
	}

	return failed == 0
}

@(private = "file")
ImpulseResult :: struct {
	worstError:      f32,
	approachSpeed:   f32,
	firstSeparation: f32,
	bounceImpulse:   f32,
	contactSteps:    i32,
	toiSteps:        i32,
	toiImpulseSteps: i32,
}

// Ball dropped under gravity with the contact impulse read back every step. Nothing else touches the
// ball, so once gravity is taken out the change in momentum over a step is the impulse the contact
// applied, and that is what the total normal impulse must report. A time of impact step is left out
// of the balance: no contact was solved on it, and the sweep hands back the gravity of the time it
// cut short.
@(private = "file")
MeasureDropImpulse :: proc(restitution: f32, dropHeight: f32) -> ImpulseResult {
	worldId := MakeWorld(-10.0)
	MakeGround(worldId, 0.0)

	ballId := MakeBall(worldId, 0.0, 0.5 + dropHeight, 0.0, restitution)
	world := b3.GetWorldFromId(worldId)

	mass := b3.Body_GetMass(ballId)
	gravityY := b3.World_GetGravity(worldId).y

	result: ImpulseResult = {}
	touched := false
	bouncing := false

	// Enough for the longest fall and the bounce that follows it
	for i: i32 = 0; i < 240; i += 1 {
		speedBefore := b3.Body_GetLinearVelocity(ballId).y
		b3.World_Step(worldId, TIME_STEP, SUB_STEP_COUNT)
		speedAfter := b3.Body_GetLinearVelocity(ballId).y

		measured: f32 = 0.0
		separation: f32 = 0.0
		contactData: [4]b3.ContactData
		contactCount := b3.Body_GetContactData(ballId, &contactData[0], i32(len(contactData)))
		for c: i32 = 0; c < contactCount; c += 1 {
			for m: i32 = 0; m < contactData[c].manifoldCount; m += 1 {
				manifold := &contactData[c].manifolds[m]
				for p: i32 = 0; p < manifold.pointCount; p += 1 {
					measured += manifold.points[p].totalNormalImpulse
					separation = manifold.points[p].separation
				}
			}
		}

		// Transient body flags are cleared and rewritten every step
		ball := b3.GetBodyFullId(world, ballId)
		ballSim := b3.GetBodySim(world, ball)
		if ballSim.flags & b3.hadTimeOfImpact != 0 {
			result.toiSteps += 1
			if measured != 0.0 {
				result.toiImpulseSteps += 1
			}
			continue
		}

		expected := mass * (speedAfter - speedBefore) - mass * gravityY * TIME_STEP
		result.worstError = b3.MaxFloat(result.worstError, b3.AbsFloat(measured - expected))

		if contactCount > 0 {
			if touched == false {
				touched = true
				bouncing = true
				result.approachSpeed = -speedBefore
				result.firstSeparation = separation
			}

			if bouncing {
				result.bounceImpulse += measured
				result.contactSteps += 1
			}
		} else {
			bouncing = false
		}
	}

	b3.DestroyWorld(worldId)
	return result
}

// Heights on both sides of the continuous collision threshold, so the impulse is checked for a ball
// that lands inside the overlap and for one the sweep sets down on the surface. The threshold is
// derived from the body so the split survives a change to the default safety factor.
//
// The step balance is the real gate. The bounce total is bracketed as well so the restitution sweep
// means something: the contact has to reverse the approach at the coefficient and may carry the
// weight for at most as long as it lasted. The stage runs after the sub-steps, so the reversal
// lands a step late and the ball can come in up to a step of gravity under it.
@(private = "file")
ImpulseTest :: proc(t: ^testing.T) -> bool {
	restitutions := [?]f32{0.0, 0.25, 0.5, 0.75, 1.0}
	heights := [?]f32{1.0, 5.0, 20.0, 45.0}

	mass: f32
	fastSpeed: f32
	{
		worldId := MakeWorld(-10.0)
		ballId := MakeBall(worldId, 0.0, 0.5, 0.0, 0.0)
		mass = b3.Body_GetMass(ballId)
		fastSpeed = b3.Body_GetSafetyFactor(ballId) * b3.Body_GetMinExtent(ballId) / TIME_STEP
		b3.DestroyWorld(worldId)
	}

	weightImpulse := mass * 10.0 * TIME_STEP

	failed := 0

	for j: i32 = 0; j < i32(len(heights)); j += 1 {
		impactSpeed := intrinsics.sqrt(20.0 * heights[j])
		expectToi := impactSpeed > fastSpeed

		// Float noise in the solver velocities scales with the impact speed, and the ball is heavy
		tolerance := 1e-5 * mass * (10.0 + impactSpeed)

		for k: i32 = 0; k < i32(len(restitutions)); k += 1 {
			result := MeasureDropImpulse(restitutions[k], heights[j])

			reversal := (1.0 + restitutions[k]) * mass * result.approachSpeed
			lower := reversal - weightImpulse
			upper := reversal + weightImpulse * f32(result.contactSteps)

			printf(
				"    impulse drop %4.1f e %.2f toi %d at %+.4f -> worst step error %.1e, bounce %.1f in [%.1f, %.1f] over %d " +
				"steps\n",
				heights[j],
				restitutions[k],
				result.toiSteps,
				result.firstSeparation,
				result.worstError,
				result.bounceImpulse,
				lower,
				upper,
				result.contactSteps,
			)

			if result.worstError > tolerance {
				failed = 1
			}

			if result.contactSteps == 0 || result.toiImpulseSteps > 0 {
				failed = 1
			}

			slack := f32(result.contactSteps) * tolerance
			if result.bounceImpulse < lower - slack || result.bounceImpulse > upper + slack {
				failed = 1
			}

			if (result.toiSteps > 0) != expectToi {
				printf(
					"    continuous collision %s at %.1f m/s (threshold %.1f m/s)\n",
					expectToi ? "expected" : "unexpected",
					impactSpeed,
					fastSpeed,
				)
				failed = 1
			}
		}
	}

	return failed == 0
}

@(private = "file")
EnergyScene :: struct {
	name:         cstring,
	safetyFactor: f32,
	friction:     f32,
	angle:        f32,
	spin:         f32,
	height:       f32,
	velocityY:    f32,
	halfWidth:    f32,
	halfHeight:   f32,
	bullet:       bool,
	stack:        bool,
	stepCount:    i32,
}

// Peak energy over the run relative to the start, for every dynamic body in the scene. Perfectly
// elastic and the energy may still only go down: friction and the soft contact position correction
// dissipate, and a bounce may return at most what the contact absorbed.
@(private = "file")
MeasurePeakEnergy :: proc(scene: ^EnergyScene) -> f32 {
	worldId := MakeWorld(-10.0)

	groundDef := b3.DefaultBodyDef()
	groundDef.position = b3.Pos{0.0, -1.0, 0.0}
	groundId := b3.CreateBody(worldId, &groundDef)
	groundShape := b3.DefaultShapeDef()
	groundShape.baseMaterial.friction = scene.friction
	ground := b3.MakeBoxHull(40.0, 1.0, 40.0)
	b3.CreateHullShape(groundId, &groundShape, &ground.base)

	shapeDef := b3.DefaultShapeDef()
	shapeDef.density = 1.0
	shapeDef.baseMaterial.friction = scene.friction
	shapeDef.baseMaterial.restitution = 1.0
	box := b3.MakeBoxHull(scene.halfWidth, scene.halfHeight, scene.halfWidth)

	bodyIds: [4]b3.BodyId
	bodyCount: i32 = 0

	if scene.stack {
		// Resting box under the dropped one, like the TwoBoxRestitution sample
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{0.0, scene.halfHeight, 0.0}
		bodyDef.safetyFactor = scene.safetyFactor
		bodyIds[bodyCount] = b3.CreateBody(worldId, &bodyDef)
		b3.CreateHullShape(bodyIds[bodyCount], &shapeDef, &box.base)
		bodyCount += 1
	}

	{
		bodyDef := b3.DefaultBodyDef()
		bodyDef.type = .dynamicBody
		bodyDef.position = b3.Pos{0.0, scene.height, 0.0}
		bodyDef.rotation = b3.MakeQuatFromAxisAngle(b3.Vec3_axisZ, scene.angle)
		bodyDef.angularVelocity = b3.Vec3{0.0, 0.0, scene.spin}
		bodyDef.linearVelocity = b3.Vec3{0.0, scene.velocityY, 0.0}
		bodyDef.safetyFactor = scene.safetyFactor
		bodyDef.isBullet = scene.bullet
		bodyIds[bodyCount] = b3.CreateBody(worldId, &bodyDef)
		b3.CreateHullShape(bodyIds[bodyCount], &shapeDef, &box.base)
		bodyCount += 1
	}

	start: f32 = 0.0
	for k: i32 = 0; k < bodyCount; k += 1 {
		start += MeasureEnergy(bodyIds[k], worldId)
	}

	peak := start
	peakStep: i32 = -1

	for i: i32 = 0; i < scene.stepCount; i += 1 {
		b3.World_Step(worldId, TIME_STEP, SUB_STEP_COUNT)

		energy: f32 = 0.0
		for k: i32 = 0; k < bodyCount; k += 1 {
			energy += MeasureEnergy(bodyIds[k], worldId)
		}

		if energy > peak {
			peak = energy
			peakStep = i
		}
	}

	b3.DestroyWorld(worldId)

	ratio := peak / start
	printf("    energy %-24s peak %.5f at step %d\n", scene.name, ratio, peakStep)
	return ratio
}

// Perfectly elastic material with friction, dropped every way that used to gain energy: a spinning
// edge landing, tumbling, a flat landing with spin, a box falling on a resting box, and a tall box
// landing flat on its narrow end. Energy must never rise above the start. This is the load bearing
// property of restitution. Accuracy of the coefficient and freedom from spin are secondary.
//
// The tall boxes guard the stage iteration count. Their contact points sit close together while the
// inertia is long, so the off diagonal of the multi point normal mass is large and one sweep
// overshoots.
@(private = "file")
EnergyTest :: proc(t: ^testing.T) -> bool {
	scenes := [?]EnergyScene {
		{"spinning edge", 0.01, 0.6, -1.43, 7.9, 0.7, -1.9, 0.5, 0.5, false, false, 300},
		{"spinning edge fast", 0.01, 0.6, -0.8, 12.0, 3.0, -8.0, 0.5, 0.5, false, false, 300},
		{"tumble", 0.01, 0.6, -0.3, 3.0, 8.0, 0.0, 0.5, 0.5, false, false, 600},
		{"tumble discrete", 0.5, 0.6, -0.3, 3.0, 8.0, 0.0, 0.5, 0.5, false, false, 600},
		{"flat with spin", 0.01, 0.6, 0.0, 2.0, 2.0, -5.0, 0.5, 0.5, false, false, 300},
		{"flat frictionless", 0.01, 0.0, 0.0, 0.0, 10.0, 0.0, 0.5, 0.5, false, false, 1200},
		{"edge", 0.01, 0.6, -0.2, 0.0, 5.0, 0.0, 0.5, 0.5, false, false, 600},
		{"two box", 0.02, 0.6, 0.0, 0.0, 4.0, 0.0, 0.5, 0.5, true, true, 1500},
		{"two box discrete", 0.5, 0.6, 0.0, 0.0, 4.0, 0.0, 0.5, 0.5, false, true, 1500},
		{"tall flat frictionless", 0.01, 0.0, 0.0, 0.0, 10.0, 0.0, 0.5, 1.0, false, false, 1200},
		{"tall flat friction", 0.01, 0.6, 0.0, 0.0, 10.0, 0.0, 0.5, 1.0, false, false, 1200},
		{"narrow flat frictionless", 0.01, 0.0, 0.0, 0.0, 10.0, 0.0, 0.25, 1.0, false, false, 1200},
		{"narrow flat friction", 0.01, 0.6, 0.0, 0.0, 10.0, 0.0, 0.25, 1.0, false, false, 1200},
	}

	// The soft contact position correction lifts a body that lands inside the ground, which is
	// potential energy from nowhere at any restitution. Continuous collision keeps landings on the
	// surface, so the tolerance only has to cover the stage. A four point flat landing does not
	// converge at the default two stage iterations the way the two point landing does in 2D: the cube
	// comes back with 0.2% extra. The in-relax scheme this replaced gained 2.5% to 29% on these scenes,
	// so the tolerance still separates the two.
	tolerance: f32 = 5.0e-3
	discreteTolerance: f32 = 2.0e-2

	failed := 0

	for i: i32 = 0; i < i32(len(scenes)); i += 1 {
		ratio := MeasurePeakEnergy(&scenes[i])
		limit := scenes[i].safetyFactor < 0.1 ? tolerance : discreteTolerance
		if ratio > 1.0 + limit {
			failed = 1
		}
	}

	return failed == 0
}

// The sampled approach speed and the Poisson accumulator are per manifold point state, so they must
// survive the split across workers
@(private = "file")
WorkerParityTest :: proc(t: ^testing.T) -> bool {
	hash1 := RunWorkerScene(1)
	hash2 := RunWorkerScene(2)
	hash4 := RunWorkerScene(4)

	printf("    worker hashes 0x%016x 0x%016x 0x%016x\n", hash1, hash2, hash4)

	ENSURE(t, hash1 == hash2) or_return
	ENSURE(t, hash1 == hash4) or_return
	return true
}

@(test)
RestitutionTest :: proc(t: ^testing.T) {
	LOCK()
	failureCount: i32 = 0

	// unified: RUN_MEASUREMENT( T ); RUN_SUBTEST reports the failure, the battery carries on
	if !RUN_SUBTEST(t, HeadOnTest) {failureCount += 1}
	if !RUN_SUBTEST(t, PhaseTest) {failureCount += 1}
	if !RUN_SUBTEST(t, SupportedTest) {failureCount += 1}
	if !RUN_SUBTEST(t, FlatLandingTest) {failureCount += 1}
	if !RUN_SUBTEST(t, SpinTest) {failureCount += 1}
	if !RUN_SUBTEST(t, DropTest) {failureCount += 1}
	if !RUN_SUBTEST(t, SingleBoxTest) {failureCount += 1}
	if !RUN_SUBTEST(t, ThresholdTest) {failureCount += 1}
	if !RUN_SUBTEST(t, RestingTest) {failureCount += 1}
	if !RUN_SUBTEST(t, NormalVelocityTest) {failureCount += 1}
	if !RUN_SUBTEST(t, OvershootTest) {failureCount += 1}
	if !RUN_SUBTEST(t, EnergyTest) {failureCount += 1}
	if !RUN_SUBTEST(t, ImpulseTest) {failureCount += 1}
	if !RUN_SUBTEST(t, WorkerParityTest) {failureCount += 1}
	// unified end

	_ = failureCount
}
