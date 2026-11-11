// Ported from box3d src/mover.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

SolvePlanes :: proc(targetDelta: Vec3, planes: [^]CollisionPlane, count: i32) -> PlaneSolverResult {
	for i: i32 = 0; i < count; i += 1 {
		planes[i].push = 0.0
	}

	delta := targetDelta
	tolerance := LINEAR_SLOP

	iteration: i32
	for iteration = 0; iteration < 20; iteration += 1 {
		totalPush: f32 = 0.0
		for planeIndex: i32 = 0; planeIndex < count; planeIndex += 1 {
			plane := &planes[planeIndex]

			// Add slop to prevent jitter
			separation := PlaneSeparation(plane.plane, delta) + LINEAR_SLOP

			push := -separation

			// Clamp accumulated push
			accumulatedPush := plane.push
			plane.push = ClampFloat(plane.push + push, 0.0, plane.pushLimit)
			push = plane.push - accumulatedPush
			delta = MulAdd(delta, push, plane.plane.normal)

			// Track maximum push for convergence
			totalPush += AbsFloat(push)
		}

		if totalPush < tolerance {
			break
		}
	}

	return PlaneSolverResult{delta = delta, iterationCount = iteration}
}

ClipVector :: proc(vector: Vec3, planes: [^]CollisionPlane, count: i32) -> Vec3 {
	v := vector

	for planeIndex: i32 = 0; planeIndex < count; planeIndex += 1 {
		plane := &planes[planeIndex]
		if plane.push == 0.0 || plane.clipVelocity == false {
			continue
		}

		v = MulSub(v, MinFloat(0.0, Dot(v, plane.plane.normal)), plane.plane.normal)
	}

	return v
}
