// Ported from box3d test/test_hull.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "core:math"
import "core:mem"
import "core:testing"

@(private = "file")
s_cubeCorners := [8]b3.Vec3 {
	{1.0, 1.0, 1.0},
	{-1.0, 1.0, 1.0},
	{-1.0, -1.0, 1.0},
	{1.0, -1.0, 1.0},
	{1.0, 1.0, -1.0},
	{-1.0, 1.0, -1.0},
	{-1.0, -1.0, -1.0},
	{1.0, -1.0, -1.0},
}

@(private = "file")
CreateHullCubeTest :: proc(t: ^testing.T) -> bool {
	hull := b3.CreateHull(&s_cubeCorners[0], 8, 8)
	ENSURE(t, hull != nil) or_return

	ENSURE(t, hull.vertexCount == 8) or_return
	ENSURE(t, hull.edgeCount == 24) or_return
	ENSURE(t, hull.faceCount == 6) or_return

	// Euler's identity for convex polyhedron
	ENSURE(t, hull.vertexCount - hull.edgeCount / 2 + hull.faceCount == 2) or_return

	ref := b3.MakeBoxHull(1.0, 1.0, 1.0)

	ENSURE_SMALL(t, hull.volume - ref.base.volume, 1e-4) or_return
	ENSURE_SMALL(t, hull.surfaceArea - ref.base.surfaceArea, 1e-4) or_return
	ENSURE_SMALL(t, hull.innerRadius - ref.base.innerRadius, b3.FLT_EPSILON) or_return

	ENSURE_SMALL(t, hull.center.x - ref.base.center.x, 1e-5) or_return
	ENSURE_SMALL(t, hull.center.y - ref.base.center.y, 1e-5) or_return
	ENSURE_SMALL(t, hull.center.z - ref.base.center.z, 1e-5) or_return

	ENSURE_SMALL(t, hull.aabb.lowerBound.x + 1.0, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.lowerBound.y + 1.0, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.lowerBound.z + 1.0, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.upperBound.x - 1.0, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.upperBound.y - 1.0, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.upperBound.z - 1.0, b3.FLT_EPSILON) or_return

	d := b3.SubMM(hull.centralInertia, ref.base.centralInertia)
	ENSURE_SMALL(t, d.cx.x, 1e-4) or_return
	ENSURE_SMALL(t, d.cy.y, 1e-4) or_return
	ENSURE_SMALL(t, d.cz.z, 1e-4) or_return
	ENSURE_SMALL(t, d.cx.y, 1e-4) or_return
	ENSURE_SMALL(t, d.cx.z, 1e-4) or_return
	ENSURE_SMALL(t, d.cy.z, 1e-4) or_return
	ENSURE_SMALL(t, d.cy.x, 1e-4) or_return
	ENSURE_SMALL(t, d.cz.x, 1e-4) or_return
	ENSURE_SMALL(t, d.cz.y, 1e-4) or_return

	b3.DestroyHull(hull)
	return true
}

@(private = "file")
CreateHullTetrahedronTest :: proc(t: ^testing.T) -> bool {
	points := [4]b3.Vec3{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}

	hull := b3.CreateHull(&points[0], 4, 4)
	ENSURE(t, hull != nil) or_return

	ENSURE(t, hull.vertexCount == 4) or_return
	ENSURE(t, hull.edgeCount == 12) or_return
	ENSURE(t, hull.faceCount == 4) or_return
	ENSURE(t, hull.vertexCount - hull.edgeCount / 2 + hull.faceCount == 2) or_return

	// Analytic values for the unit-corner tetrahedron at the origin.
	expectedVolume := f32(1.0) / f32(6.0)
	expectedSurfaceArea := 1.5 + 0.5 * math.sqrt(f32(3.0))
	expectedInnerRadius := 0.25 / math.sqrt(f32(3.0))

	ENSURE_SMALL(t, hull.volume - expectedVolume, 1e-5) or_return
	ENSURE_SMALL(t, hull.surfaceArea - expectedSurfaceArea, 1e-5) or_return
	ENSURE_SMALL(t, hull.innerRadius - expectedInnerRadius, 1e-5) or_return

	ENSURE_SMALL(t, hull.center.x - 0.25, 1e-5) or_return
	ENSURE_SMALL(t, hull.center.y - 0.25, 1e-5) or_return
	ENSURE_SMALL(t, hull.center.z - 0.25, 1e-5) or_return

	ENSURE_SMALL(t, hull.aabb.lowerBound.x, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.lowerBound.y, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.lowerBound.z, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.upperBound.x - 1.0, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.upperBound.y - 1.0, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.upperBound.z - 1.0, b3.FLT_EPSILON) or_return

	b3.DestroyHull(hull)
	return true
}

@(private = "file")
CreateHullDeterminismTest :: proc(t: ^testing.T) -> bool {
	h1 := b3.CreateHull(&s_cubeCorners[0], 8, 8)
	h2 := b3.CreateHull(&s_cubeCorners[0], 8, 8)
	ENSURE(t, h1 != nil && h2 != nil) or_return

	ENSURE(t, h1.byteCount == h2.byteCount) or_return
	ENSURE(t, h1.hash != 0) or_return
	ENSURE(t, h1.hash == h2.hash) or_return
	ENSURE(t, mem.compare_ptrs(h1, h2, int(h1.byteCount)) == 0) or_return

	b3.DestroyHull(h1)
	b3.DestroyHull(h2)
	return true
}

@(private = "file")
SPHERE_N :: 6

@(private = "file")
CreateHullMaxVertexTest :: proc(t: ^testing.T) -> bool {
	// Sphere-sampled point cloud, dense enough that the builder has room to grow.
	points: [SPHERE_N * SPHERE_N]b3.Vec3
	index: i32 = 0
	for i: i32 = 0; i < SPHERE_N; i += 1 {
		theta := b3.PI * f32(i) / f32(SPHERE_N - 1)
		for j: i32 = 0; j < SPHERE_N; j += 1 {
			phi := 2.0 * b3.PI * f32(j) / f32(SPHERE_N)
			points[index].x = math.sin(theta) * math.cos(phi)
			points[index].y = math.sin(theta) * math.sin(phi)
			points[index].z = math.cos(theta)
			index += 1
		}
	}
	count: i32 = SPHERE_N * SPHERE_N

	// maxVertexCount honored as a strict cap.
	h1 := b3.CreateHull(&points[0], count, 8)
	ENSURE(t, h1 != nil) or_return
	ENSURE(t, h1.vertexCount <= 8) or_return
	b3.DestroyHull(h1)

	// Below the floor: clamps up to 4.
	h2 := b3.CreateHull(&points[0], count, 1)
	ENSURE(t, h2 != nil) or_return
	ENSURE(t, h2.vertexCount >= 4 && h2.vertexCount <= b3.MAX_HULL_VERTICES) or_return
	b3.DestroyHull(h2)

	// Above the ceiling: clamps down to B3_MAX_HULL_VERTICES.
	h3 := b3.CreateHull(&points[0], count, 1000)
	ENSURE(t, h3 != nil) or_return
	ENSURE(t, h3.vertexCount >= 4 && h3.vertexCount <= b3.MAX_HULL_VERTICES) or_return
	b3.DestroyHull(h3)

	return true
}

@(private = "file")
CreateHullRedundantInputTest :: proc(t: ^testing.T) -> bool {
	// 8 cube corners + duplicates + interior points. Builder should produce the cube.
	points := [20]b3.Vec3 {
		{1.0, 1.0, 1.0}, // corners
		{-1.0, 1.0, 1.0},
		{-1.0, -1.0, 1.0},
		{1.0, -1.0, 1.0},
		{1.0, 1.0, -1.0},
		{-1.0, 1.0, -1.0},
		{-1.0, -1.0, -1.0},
		{1.0, -1.0, -1.0},
		{1.0, 1.0, 1.0}, // duplicates
		{1.0, 1.0, 1.0},
		{0.0, 0.0, 0.0}, // interior points
		{0.5, 0.0, 0.0},
		{0.0, 0.5, 0.0},
		{0.0, 0.0, 0.5},
		{-0.5, 0.0, 0.0},
		{0.0, -0.5, 0.0},
		{0.0, 0.0, -0.5},
		{0.25, 0.25, 0.25},
		{-0.25, -0.25, -0.25},
		{0.5, 0.5, 0.5},
	}

	hull := b3.CreateHull(&points[0], 20, 8)
	ENSURE(t, hull != nil) or_return

	ENSURE(t, hull.vertexCount == 8) or_return
	ENSURE(t, hull.edgeCount == 24) or_return
	ENSURE(t, hull.faceCount == 6) or_return

	ref := b3.MakeBoxHull(1.0, 1.0, 1.0)

	ENSURE_SMALL(t, hull.volume - ref.base.volume, 1e-4) or_return
	ENSURE_SMALL(t, hull.surfaceArea - ref.base.surfaceArea, 1e-4) or_return
	ENSURE_SMALL(t, hull.innerRadius - ref.base.innerRadius, b3.FLT_EPSILON) or_return

	ENSURE_SMALL(t, hull.center.x - ref.base.center.x, 1e-5) or_return
	ENSURE_SMALL(t, hull.center.y - ref.base.center.y, 1e-5) or_return
	ENSURE_SMALL(t, hull.center.z - ref.base.center.z, 1e-5) or_return

	ENSURE_SMALL(t, hull.aabb.lowerBound.x + 1.0, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.lowerBound.y + 1.0, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.lowerBound.z + 1.0, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.upperBound.x - 1.0, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.upperBound.y - 1.0, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.upperBound.z - 1.0, b3.FLT_EPSILON) or_return

	b3.DestroyHull(hull)
	return true
}

@(private = "file")
CreateHullCloneTest :: proc(t: ^testing.T) -> bool {
	original := b3.CreateHull(&s_cubeCorners[0], 8, 8)
	ENSURE(t, original != nil) or_return

	clone := b3.CloneHull(original)
	ENSURE(t, clone != nil) or_return
	ENSURE(t, clone.byteCount == original.byteCount) or_return
	ENSURE(t, mem.compare_ptrs(clone, original, int(original.byteCount)) == 0) or_return

	b3.DestroyHull(clone)
	b3.DestroyHull(original)
	return true
}

@(private = "file")
CreateHullCylinderTest :: proc(t: ^testing.T) -> bool {
	height: f32 : 2.0
	radius: f32 : 1.0
	sides: i32 : 8
	yOffset: f32 : 0.0

	hull := b3.CreateCylinder(height, radius, yOffset, sides)
	ENSURE(t, hull != nil) or_return

	ENSURE(t, hull.vertexCount == 2 * sides) or_return
	ENSURE(t, hull.edgeCount == 6 * sides) or_return
	ENSURE(t, hull.faceCount == sides + 2) or_return

	// Analytic n-gon prism values (exact targets, not the circular cylinder approximations).
	halfAngle := b3.PI / f32(sides)
	capArea := f32(sides) * 0.5 * radius * radius * math.sin(2.0 * halfAngle)
	chordLen := 2.0 * radius * math.sin(halfAngle)
	lateralArea := f32(sides) * chordLen * height

	expectedVolume := capArea * height
	expectedSurfaceArea := 2.0 * capArea + lateralArea
	expectedInnerRadius := radius * math.cos(halfAngle)

	ENSURE_SMALL(t, (hull.volume - expectedVolume) / expectedVolume, 1e-4) or_return
	ENSURE_SMALL(t, (hull.surfaceArea - expectedSurfaceArea) / expectedSurfaceArea, 1e-4) or_return
	ENSURE_SMALL(t, hull.innerRadius - expectedInnerRadius, 1e-5) or_return

	ENSURE_SMALL(t, hull.center.x, 1e-5) or_return
	ENSURE_SMALL(t, hull.center.y - (yOffset + 0.5 * height), 1e-5) or_return
	ENSURE_SMALL(t, hull.center.z, 1e-5) or_return

	ENSURE_SMALL(t, hull.aabb.lowerBound.x + radius, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.lowerBound.y - yOffset, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.lowerBound.z + radius, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.upperBound.x - radius, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.upperBound.y - (yOffset + height), b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hull.aabb.upperBound.z - radius, b3.FLT_EPSILON) or_return

	b3.DestroyHull(hull)
	return true
}

// Inlined XorShift32 + Shoemake unit-vector recipe matches shared/utils.h exactly so
// generated points are bit-identical to samples that share the seed.
@(private = "file")
FillSphereSample :: proc(points: [^]b3.Vec3, count: i32, seed: u32) {
	seed := seed
	RAND_LIMIT_LOCAL: u32 : 32767
	for i: i32 = 0; i < count; i += 1 {
		u: [3]f32
		for k := 0; k < 3; k += 1 {
			seed ~= seed << 13
			seed ~= seed >> 17
			seed ~= seed << 5
			r := f32(seed & RAND_LIMIT_LOCAL)
			r /= f32(RAND_LIMIT_LOCAL)
			u[k] = r
		}
		u1 := u[0]
		u2 := 2.0 * b3.PI * u[1]
		u3 := 2.0 * b3.PI * u[2]
		sqrt1MinusU1 := math.sqrt(1.0 - u1)
		sqrtU1 := math.sqrt(u1)
		points[i].x = sqrt1MinusU1 * math.sin(u2)
		points[i].y = sqrt1MinusU1 * math.cos(u2)
		points[i].z = sqrtU1 * math.sin(u3)
	}
}

// Reproduces the HullReduction sample (Sphere, 64 points, count=20) that used to assert on
// b->faceCount < b->faceCapacity. The free-list reclaim in NewFace/NewEdge keeps the bump
// counts proportional to the live hull instead of cumulative creations.
@(private = "file")
CreateHullSphereReductionTest :: proc(t: ^testing.T) -> bool {
	points: [64]b3.Vec3
	FillSphereSample(&points[0], 64, 12345) // RAND_SEED

	hull := b3.CreateHull(&points[0], 64, 20)
	ENSURE(t, hull != nil) or_return
	ENSURE(t, hull.vertexCount >= 4 && hull.vertexCount <= 20) or_return
	ENSURE(t, hull.vertexCount - hull.edgeCount / 2 + hull.faceCount == 2) or_return
	b3.DestroyHull(hull)

	return true
}

// XorShift32 + uniform-cube point generator. Same engine as FillSphereSample.
@(private = "file")
FillCubeSample :: proc(points: [^]b3.Vec3, count: i32, seed: u32) {
	seed := seed
	RAND_LIMIT_LOCAL: u32 : 32767
	for i: i32 = 0; i < count; i += 1 {
		v: [3]f32
		for k := 0; k < 3; k += 1 {
			seed ~= seed << 13
			seed ~= seed >> 17
			seed ~= seed << 5
			r := f32(seed & RAND_LIMIT_LOCAL)
			r /= f32(RAND_LIMIT_LOCAL)
			v[k] = 2.0 * r - 1.0
		}
		points[i].x = v[0]
		points[i].y = v[1]
		points[i].z = v[2]
	}
}

// Pushes the working-stage bump capacities (faceCapacity = 5*M - 10, edgeCapacity =
// 24*M - 48 in src/hull.c::b3ComputeHullWorkSizes) close to their peak by sweeping M
// on a dense random sphere. If the peak face/edge count exceeds the cap and the free
// list fails to reclaim slots in time, the assertion at src/hull.c::NewFace/NewEdge
// fires (exit code 3). A dense sphere hull is fully triangulated, so F = 2M - 4 and the
// final face limit B3_MAX_HULL_FACES binds well before the vertex or edge limits.
@(private = "file")
CreateHullSphereStressTest :: proc(t: ^testing.T) -> bool {
	N :: 512
	points: [N]b3.Vec3

	// Multiple seeds exercise different horizon-size / merge-cascade sequences.
	seeds := [?]u32{12345, 1, 0xdeadbeef, 0xcafef00d}

	// M kept <= 32 so the final face count 2M - 4 stays under B3_MAX_HULL_FACES
	M_values := [?]i32{16, 24, 32}

	for s: i32 = 0; s < i32(len(seeds)); s += 1 {
		FillSphereSample(&points[0], N, seeds[s])

		for m: i32 = 0; m < i32(len(M_values)); m += 1 {
			M := M_values[m]
			hull := b3.CreateHull(&points[0], N, M)
			ENSURE(t, hull != nil) or_return
			ENSURE(t, hull.vertexCount >= 4 && hull.vertexCount <= M) or_return
			ENSURE(t, hull.vertexCount - hull.edgeCount / 2 + hull.faceCount == 2) or_return
			ENSURE(t, hull.faceCount >= 4) or_return
			b3.DestroyHull(hull)
		}
	}

	return true
}

// Random points inside a cube produce a small final hull (8 corners, 6 faces) but
// generate heavy internal churn: every interior point gets fed to the conflict-list
// machinery, and most cone faces created during apex insertion are then merged out
// when their newly-created neighbors are coplanar. Exercises the merge cascade in
// b3HullBuilder_ConnectEdges/ConnectFaces and the corresponding RetireFace/RetireEdge
// reclaim path much harder than the sphere case (which has few merges).
@(private = "file")
CreateHullMergeChurnStressTest :: proc(t: ^testing.T) -> bool {
	N :: 4096
	@(static) points: [N]b3.Vec3

	seeds := [?]u32{12345, 0xdeadbeef}

	for s: i32 = 0; s < i32(len(seeds)); s += 1 {
		FillCubeSample(&points[0], N, seeds[s])

		// Stamp the 8 corners last so they're guaranteed extremes.
		for c: i32 = 0; c < 8; c += 1 {
			points[N - 8 + c].x = (c & 1) != 0 ? 1.0 : -1.0
			points[N - 8 + c].y = (c & 2) != 0 ? 1.0 : -1.0
			points[N - 8 + c].z = (c & 4) != 0 ? 1.0 : -1.0
		}

		hull := b3.CreateHull(&points[0], N, 64)
		ENSURE(t, hull != nil) or_return
		ENSURE(t, hull.vertexCount == 8) or_return
		ENSURE(t, hull.edgeCount == 24) or_return
		ENSURE(t, hull.faceCount == 6) or_return
		b3.DestroyHull(hull)
	}

	return true
}

// b3ComputeCosSin is a coarse approximation, so build rotations from libm to keep the
// analytic corner and plane positions exact.
@(private = "file")
ExactQuat :: proc(axis: b3.Vec3, radians: f32) -> b3.Quat {
	half := 0.5 * radians
	s := math.sin(half)
	return b3.Quat{{s * axis.x, s * axis.y, s * axis.z}, math.cos(half)}
}

// Corner sign pattern baked by b3MakeTransformedBoxHull, in order.
@(private = "file")
s_boxCornerSigns := [8]b3.Vec3 {
	{1.0, 1.0, 1.0},
	{-1.0, 1.0, 1.0},
	{-1.0, -1.0, 1.0},
	{1.0, -1.0, 1.0},
	{1.0, 1.0, -1.0},
	{-1.0, 1.0, -1.0},
	{-1.0, -1.0, -1.0},
	{1.0, -1.0, -1.0},
}

// The baked box hull is only exercised elsewhere through mass properties, which are analytic
// and never read boxPoints or boxPlanes. Pin the geometry itself against the transform so an
// axis swap in the point or plane bake cannot pass silently.
@(private = "file")
CheckTransformedBox :: proc(t: ^testing.T, h: b3.Vec3, xf: b3.Transform) -> bool {
	box := b3.MakeTransformedBoxHull(h.x, h.y, h.z, xf)

	tol: f32 : 1e-5

	// Each corner is the transform of the signed half extent.
	for i := 0; i < 8; i += 1 {
		local := b3.Vec3{s_boxCornerSigns[i].x * h.x, s_boxCornerSigns[i].y * h.y, s_boxCornerSigns[i].z * h.z}
		expected := b3.TransformPoint(xf, local)
		ENSURE_SMALL(t, box.boxPoints[i].x - expected.x, tol) or_return
		ENSURE_SMALL(t, box.boxPoints[i].y - expected.y, tol) or_return
		ENSURE_SMALL(t, box.boxPoints[i].z - expected.z, tol) or_return
	}

	// Face normals rotate with the box, offsets carry the rotated normal through the translation.
	localNormals := [6]b3.Vec3 {
		b3.Neg(b3.Vec3_axisX),
		b3.Vec3_axisX,
		b3.Neg(b3.Vec3_axisY),
		b3.Vec3_axisY,
		b3.Neg(b3.Vec3_axisZ),
		b3.Vec3_axisZ,
	}
	localOffsets := [6]f32{h.x, h.x, h.y, h.y, h.z, h.z}
	for i := 0; i < 6; i += 1 {
		n := b3.RotateVector(xf.q, localNormals[i])
		offset := localOffsets[i] + b3.Dot(n, xf.p)
		ENSURE_SMALL(t, box.boxPlanes[i].normal.x - n.x, tol) or_return
		ENSURE_SMALL(t, box.boxPlanes[i].normal.y - n.y, tol) or_return
		ENSURE_SMALL(t, box.boxPlanes[i].normal.z - n.z, tol) or_return
		ENSURE_SMALL(t, box.boxPlanes[i].offset - offset, tol) or_return
	}

	// SoA vertex mirrors match the AoS points on all eight lanes.
	for i := 0; i < 8; i += 1 {
		ENSURE_SMALL(t, box.vx[i] - box.boxPoints[i].x, tol) or_return
		ENSURE_SMALL(t, box.vy[i] - box.boxPoints[i].y, tol) or_return
		ENSURE_SMALL(t, box.vz[i] - box.boxPoints[i].z, tol) or_return
	}

	// SoA normal mirrors carry the six real faces and zero the two pad lanes.
	for i := 0; i < 6; i += 1 {
		ENSURE_SMALL(t, box.nx[i] - box.boxPlanes[i].normal.x, tol) or_return
		ENSURE_SMALL(t, box.ny[i] - box.boxPlanes[i].normal.y, tol) or_return
		ENSURE_SMALL(t, box.nz[i] - box.boxPlanes[i].normal.z, tol) or_return
	}
	ENSURE(t, box.nx[6] == 0.0 && box.nx[7] == 0.0) or_return
	ENSURE(t, box.ny[6] == 0.0 && box.ny[7] == 0.0) or_return
	ENSURE(t, box.nz[6] == 0.0 && box.nz[7] == 0.0) or_return

	// The stored AABB bounds every baked corner.
	pointBox := b3.MakeAABB(&box.boxPoints[0], 8, 0.0)
	ENSURE(t, b3.AABB_Contains(box.base.aabb, pointBox)) or_return

	return true
}

@(private = "file")
TransformedBoxHullTest :: proc(t: ^testing.T) -> bool {
	h := b3.Vec3{0.25, 0.5, 0.3}

	// Identity.
	CheckTransformedBox(t, h, b3.Transform_identity) or_return

	// Translation only.
	translated := b3.Transform{{0.4, -0.7, 0.1}, b3.Quat_identity}
	CheckTransformedBox(t, h, translated) or_return

	// Rotation only. A quarter turn about Y swaps the world X and Z extents.
	rotated := b3.Transform{b3.Vec3_zero, ExactQuat(b3.Vec3_axisY, 0.25 * b3.PI)}
	CheckTransformedBox(t, h, rotated) or_return

	// Translation and rotation together.
	transformed := b3.Transform{{3.0, -2.0, 1.5}, ExactQuat(b3.Vec3_axisZ, 0.25 * b3.PI)}
	CheckTransformedBox(t, h, transformed) or_return

	return true
}

@(private = "file")
CreateHullDegenerateTest :: proc(t: ^testing.T) -> bool {
	// Real (non-null) buffer; pointCount < 4 cases are guarded inside Construct().
	collinear := [8]b3.Vec3 {
		{0.0, 0.0, 0.0},
		{1.0, 0.0, 0.0},
		{2.0, 0.0, 0.0},
		{3.0, 0.0, 0.0},
		{4.0, 0.0, 0.0},
		{5.0, 0.0, 0.0},
		{6.0, 0.0, 0.0},
		{7.0, 0.0, 0.0},
	}

	// Empty input.
	ENSURE(t, b3.CreateHull(&collinear[0], 0, 8) == nil) or_return

	// Fewer than 4 points.
	ENSURE(t, b3.CreateHull(&collinear[0], 3, 8) == nil) or_return

	// 8 coincident points.
	coincident := [8]b3.Vec3 {
		{1.0, 2.0, 3.0},
		{1.0, 2.0, 3.0},
		{1.0, 2.0, 3.0},
		{1.0, 2.0, 3.0},
		{1.0, 2.0, 3.0},
		{1.0, 2.0, 3.0},
		{1.0, 2.0, 3.0},
		{1.0, 2.0, 3.0},
	}
	ENSURE(t, b3.CreateHull(&coincident[0], 8, 8) == nil) or_return

	// Collinear (along x-axis).
	ENSURE(t, b3.CreateHull(&collinear[0], 8, 8) == nil) or_return

	// Coplanar (in the xy-plane).
	coplanar := [6]b3.Vec3 {
		{0.0, 0.0, 0.0},
		{1.0, 0.0, 0.0},
		{0.0, 1.0, 0.0},
		{1.0, 1.0, 0.0},
		{2.0, 0.5, 0.0},
		{0.5, 2.0, 0.0},
	}
	ENSURE(t, b3.CreateHull(&coplanar[0], 6, 8) == nil) or_return

	return true
}

@(private = "file")
FillPoints2D :: proc(pts: [^]b3.Point2D, input: [^]b3.Vec2, count: i32) {
	for i: i32 = 0; i < count; i += 1 {
		pts[i].p = input[i]
		pts[i].separation = 0.0
		pts[i].originalIndex = i
	}
}

@(private = "file")
Hull2DArea :: proc(hull: [^]b3.Point2D, count: i32) -> f32 {
	sum: f32 = 0.0
	for i: i32 = 0; i < count; i += 1 {
		next := (i + 1) % count
		sum += hull[i].p.x * hull[next].p.y - hull[i].p.y * hull[next].p.x
	}

	return 0.5 * sum
}

// Interior points contribute no area, so they must not survive the hull.
@(private = "file")
Hull2DSquareTest :: proc(t: ^testing.T) -> bool {
	input := [8]b3.Vec2 {
		{-1.0, -1.0},
		{1.0, -1.0},
		{1.0, 1.0},
		{-1.0, 1.0},
		{0.0, 0.0},
		{0.5, 0.25},
		{-0.3, 0.6},
		{0.1, -0.4},
	}

	pts: [8]b3.Point2D
	hull: [16]b3.Point2D
	FillPoints2D(&pts[0], &input[0], 8)

	count := b3.Hull2D(&pts[0], 8, &hull[0])
	ENSURE(t, count == 4) or_return

	// The reduction code assumes counter clockwise winding
	ENSURE(t, Hull2DArea(&hull[0], count) > 0.0) or_return

	onHull: [8]bool
	for i: i32 = 0; i < count; i += 1 {
		ENSURE(t, 0 <= hull[i].originalIndex && hull[i].originalIndex < 8) or_return
		onHull[hull[i].originalIndex] = true
	}

	for i := 0; i < 4; i += 1 {
		ENSURE(t, onHull[i] == true) or_return
	}

	for i := 4; i < 8; i += 1 {
		ENSURE(t, onHull[i] == false) or_return
	}

	return true
}

@(private = "file")
Hull2DCollinearTest :: proc(t: ^testing.T) -> bool {
	input := [5]b3.Vec2{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}, {4.0, 0.0}}

	pts: [5]b3.Point2D
	hull: [10]b3.Point2D
	FillPoints2D(&pts[0], &input[0], 5)

	ENSURE(t, b3.Hull2D(&pts[0], 5, &hull[0]) == 2) or_return

	return true
}

// Coincident points weld and the lower input index wins, so a sort order change cannot
// swap which of the two survives.
@(private = "file")
Hull2DWeldTest :: proc(t: ^testing.T) -> bool {
	offset := 0.1 * b3.LINEAR_SLOP

	input := [5]b3.Vec2{{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}, {-1.0 + offset, -1.0 + offset}}

	pts: [5]b3.Point2D
	hull: [10]b3.Point2D
	FillPoints2D(&pts[0], &input[0], 5)

	count := b3.Hull2D(&pts[0], 5, &hull[0])
	ENSURE(t, count == 4) or_return

	for i: i32 = 0; i < count; i += 1 {
		ENSURE(t, hull[i].originalIndex != 4) or_return
	}

	return true
}

@(private = "file")
SimplifyHull2DTargetTest :: proc(t: ^testing.T) -> bool {
	pts: [20]b3.Point2D
	hull: [40]b3.Point2D

	for i: i32 = 0; i < 20; i += 1 {
		angle := 2.0 * b3.PI * f32(i) / 20.0
		pts[i].p = b3.Vec2{math.cos(angle), math.sin(angle)}
		pts[i].separation = 0.0
		pts[i].originalIndex = i
	}

	hullCount := b3.Hull2D(&pts[0], 20, &hull[0])
	ENSURE(t, hullCount == 20) or_return

	count := b3.SimplifyHull2D(&hull[0], hullCount, 6)
	ENSURE(t, count == 6) or_return
	ENSURE(t, Hull2DArea(&hull[0], count) > 0.0) or_return

	return true
}

@(private = "file")
SimplifyHull2DNoopTest :: proc(t: ^testing.T) -> bool {
	input := [4]b3.Vec2{{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}}

	pts: [4]b3.Point2D
	hull: [8]b3.Point2D
	FillPoints2D(&pts[0], &input[0], 4)

	hullCount := b3.Hull2D(&pts[0], 4, &hull[0])
	ENSURE(t, hullCount == 4) or_return

	ENSURE(t, b3.SimplifyHull2D(&hull[0], hullCount, 8) == 4) or_return
	ENSURE(t, b3.SimplifyHull2D(&hull[0], hullCount, 4) == 4) or_return

	return true
}

// The surviving point set must not depend on the order the points arrive in. The reduction
// runs inside the solver, so an input order dependence would show up as a determinism bug.
@(private = "file")
Hull2DOrderTest :: proc(t: ^testing.T) -> bool {
	input := [8]b3.Vec2 {
		{-2.0, -1.0},
		{0.0, -1.5},
		{2.0, -1.0},
		{2.5, 0.5},
		{1.0, 2.0},
		{-1.0, 1.8},
		{-2.5, 0.4},
		{0.2, 0.1},
	}

	permutation := [8]i32{5, 2, 7, 0, 4, 6, 1, 3}

	pts: [8]b3.Point2D
	hull: [16]b3.Point2D
	FillPoints2D(&pts[0], &input[0], 8)

	count1 := b3.Hull2D(&pts[0], 8, &hull[0])
	ENSURE(t, count1 == 7) or_return

	onHull1: [8]bool
	for i: i32 = 0; i < count1; i += 1 {
		onHull1[hull[i].originalIndex] = true
	}

	shuffled: [8]b3.Point2D
	shuffledHull: [16]b3.Point2D
	for i: i32 = 0; i < 8; i += 1 {
		shuffled[i].p = input[permutation[i]]
		shuffled[i].separation = 0.0
		shuffled[i].originalIndex = i
	}

	count2 := b3.Hull2D(&shuffled[0], 8, &shuffledHull[0])
	ENSURE(t, count2 == count1) or_return

	onHull2: [8]bool
	for i: i32 = 0; i < count2; i += 1 {
		onHull2[permutation[shuffledHull[i].originalIndex]] = true
	}

	for i := 0; i < 8; i += 1 {
		ENSURE(t, onHull1[i] == onHull2[i]) or_return
	}

	return true
}

@(test)
HullTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, CreateHullCubeTest) {return}
	if !RUN_SUBTEST(t, CreateHullTetrahedronTest) {return}
	if !RUN_SUBTEST(t, CreateHullDeterminismTest) {return}
	if !RUN_SUBTEST(t, CreateHullMaxVertexTest) {return}
	if !RUN_SUBTEST(t, CreateHullRedundantInputTest) {return}
	if !RUN_SUBTEST(t, CreateHullCloneTest) {return}
	if !RUN_SUBTEST(t, CreateHullCylinderTest) {return}
	if !RUN_SUBTEST(t, CreateHullSphereReductionTest) {return}
	if !RUN_SUBTEST(t, CreateHullSphereStressTest) {return}
	if !RUN_SUBTEST(t, CreateHullMergeChurnStressTest) {return}
	if !RUN_SUBTEST(t, CreateHullDegenerateTest) {return}
	if !RUN_SUBTEST(t, TransformedBoxHullTest) {return}
	if !RUN_SUBTEST(t, Hull2DSquareTest) {return}
	if !RUN_SUBTEST(t, Hull2DCollinearTest) {return}
	if !RUN_SUBTEST(t, Hull2DWeldTest) {return}
	if !RUN_SUBTEST(t, Hull2DOrderTest) {return}
	if !RUN_SUBTEST(t, SimplifyHull2DTargetTest) {return}
	if !RUN_SUBTEST(t, SimplifyHull2DNoopTest) {return}
}
