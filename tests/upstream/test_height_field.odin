// Ported from box3d test/test_height_field.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "core:os"
import "core:testing"

@(private = "file")
HeightFieldCreate :: proc(t: ^testing.T) -> bool {
	scale := b3.Vec3{1.0, 1.0, 1.0}
	hf := b3.CreateGrid(4, 4, scale, false)

	ENSURE(t, hf.rowCount == 4) or_return
	ENSURE(t, hf.columnCount == 4) or_return
	ENSURE(t, hf.clockwise == 0) or_return
	ENSURE(t, hf.version == b3.HEIGHT_FIELD_VERSION) or_return

	ENSURE_SMALL(t, hf.aabb.lowerBound.x, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hf.aabb.lowerBound.y, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hf.aabb.lowerBound.z, b3.FLT_EPSILON) or_return

	ENSURE_SMALL(t, hf.aabb.upperBound.x - 3.0, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hf.aabb.upperBound.y, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, hf.aabb.upperBound.z - 3.0, b3.FLT_EPSILON) or_return

	b3.DestroyHeightField(hf)
	return true
}

@(private = "file")
HeightFieldTriangleIndex :: proc(t: ^testing.T) -> bool {
	// Asymmetric grid (rows != cols) catches off-by-one errors between
	// vertex stride (columnCount) and cell stride (columnCount - 1).
	rowCount: i32 = 4
	columnCount: i32 = 5
	scale := b3.Vec3{1.0, 1.0, 1.0}
	hf := b3.CreateGrid(rowCount, columnCount, scale, false)

	triangleCount := 2 * (rowCount - 1) * (columnCount - 1)

	for triangleIndex: i32 = 0; triangleIndex < triangleCount; triangleIndex += 1 {
		quadIndex := triangleIndex >> 1
		sub := triangleIndex & 1
		row := quadIndex / (columnCount - 1)
		column := quadIndex - row * (columnCount - 1)

		index11 := row * columnCount + column
		index12 := index11 + 1
		index21 := (row + 1) * columnCount + column
		index22 := index21 + 1

		tri := b3.GetHeightFieldTriangle(hf, triangleIndex)

		if sub == 0 {
			// Triangle 0 (CCW): {11, 21, 12}
			ENSURE(t, tri.i1 == index11) or_return
			ENSURE(t, tri.i2 == index21) or_return
			ENSURE(t, tri.i3 == index12) or_return
		} else {
			// Triangle 1 (CCW): {22, 12, 21}
			ENSURE(t, tri.i1 == index22) or_return
			ENSURE(t, tri.i2 == index12) or_return
			ENSURE(t, tri.i3 == index21) or_return
		}
	}

	b3.DestroyHeightField(hf)
	return true
}

@(private = "file")
HeightFieldWinding :: proc(t: ^testing.T) -> bool {
	// Build the same flat 3x3 field with CCW and CW winding. The cross-product
	// normal of triangle 0 must flip sign accordingly.
	heights: [9]f32
	materials := [4]u8{0, 0, 0, 0}

	def := b3.HeightFieldDef{}
	def.heights = &heights[0]
	def.materialIndices = &materials[0]
	def.scale = b3.Vec3{1.0, 1.0, 1.0}
	def.countX = 3
	def.countZ = 3
	def.globalMinimumHeight = -1.0
	def.globalMaximumHeight = 1.0

	def.clockwiseWinding = false
	ccw := b3.CreateHeightField(&def)

	def.clockwiseWinding = true
	cw := b3.CreateHeightField(&def)

	ta := b3.GetHeightFieldTriangle(ccw, 0)
	tb := b3.GetHeightFieldTriangle(cw, 0)

	na := b3.Normalize(b3.Cross(b3.Sub(ta.vertices[1], ta.vertices[0]), b3.Sub(ta.vertices[2], ta.vertices[0])))
	nb := b3.Normalize(b3.Cross(b3.Sub(tb.vertices[1], tb.vertices[0]), b3.Sub(tb.vertices[2], tb.vertices[0])))

	ENSURE_SMALL(t, na.x, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, na.y - 1.0, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, na.z, b3.FLT_EPSILON) or_return

	ENSURE_SMALL(t, nb.x, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, nb.y + 1.0, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, nb.z, b3.FLT_EPSILON) or_return

	b3.DestroyHeightField(ccw)
	b3.DestroyHeightField(cw)
	return true
}

@(private = "file")
RayCastFlatField :: proc(t: ^testing.T) -> bool {
	// Build a flat 4x4 field with a tight quantization range so the recovered
	// surface stays within ~1e-5 of y=0 (b3CreateGrid uses -256..256 which
	// blows the 1/UINT16_MAX quantum up to ~4e-3 in y).
	heights: [16]f32
	materials: [9]u8

	def := b3.HeightFieldDef{}
	def.heights = &heights[0]
	def.materialIndices = &materials[0]
	def.scale = b3.Vec3{1.0, 1.0, 1.0}
	def.countX = 4
	def.countZ = 4
	def.globalMinimumHeight = -1.0
	def.globalMaximumHeight = 1.0
	def.clockwiseWinding = false

	hf := b3.CreateHeightField(&def)

	// Origin sits clearly inside triangle 0 of cell (1, 1). Off the cell
	// diagonal x+z = 3. The translation overshoots the surface so the hit
	// fraction is strictly less than maxFraction.
	input := b3.RayCastInput{}
	input.origin = b3.Vec3{1.25, 10.0, 1.25}
	input.translation = b3.Vec3{0.0, -20.0, 0.0}
	input.maxFraction = 1.0

	out := b3.RayCastHeightField(hf, &input)

	ENSURE(t, out.hit == true) or_return
	ENSURE_SMALL(t, out.fraction - 0.5, 1e-5) or_return
	ENSURE_SMALL(t, out.normal.x, 1e-5) or_return
	ENSURE_SMALL(t, out.normal.y - 1.0, 1e-5) or_return
	ENSURE_SMALL(t, out.normal.z, 1e-5) or_return

	b3.DestroyHeightField(hf)
	return true
}

@(private = "file")
OverlapAtSurface :: proc(t: ^testing.T) -> bool {
	scale := b3.Vec3{1.0, 1.0, 1.0}
	hf := b3.CreateGrid(4, 4, scale, false)

	// Sphere center 1.0 above the surface, radius 0.5. Clear gap.
	above := b3.Vec3{1.5, 1.0, 1.5}
	proxyAbove := b3.ShapeProxy{&above, 1, 0.5}
	hitAbove := b3.OverlapHeightField(hf, b3.Transform_identity, &proxyAbove)
	ENSURE(t, hitAbove == false) or_return

	// Sphere centered on the surface. Radius pokes through.
	through := b3.Vec3{1.5, 0.0, 1.5}
	proxyThrough := b3.ShapeProxy{&through, 1, 0.5}
	hitThrough := b3.OverlapHeightField(hf, b3.Transform_identity, &proxyThrough)
	ENSURE(t, hitThrough == true) or_return

	b3.DestroyHeightField(hf)
	return true
}

@(private = "file")
FileRoundtrip :: proc(t: ^testing.T) -> bool {
	heights := [9]f32{0.0, 0.5, -0.3, 0.1, 0.0, 0.0, 0.0, 0.2, 0.0}
	materials := [4]u8{0, b3.HEIGHT_FIELD_HOLE, 1, 2}

	def := b3.HeightFieldDef{}
	def.heights = &heights[0]
	def.materialIndices = &materials[0]
	def.scale = b3.Vec3{1.5, 2.0, 0.75}
	def.countX = 3
	def.countZ = 3
	def.globalMinimumHeight = -1.0
	def.globalMaximumHeight = 1.0
	def.clockwiseWinding = true

	path: cstring = "test_height_field_roundtrip.dat"
	b3.DumpHeightData(&def, path)

	loaded := b3.LoadHeightField(path)
	_ = os.remove(string(path))

	ENSURE(t, loaded != nil) or_return
	ENSURE(t, loaded.rowCount == def.countZ) or_return
	ENSURE(t, loaded.columnCount == def.countX) or_return
	ENSURE(t, loaded.clockwise == u8(def.clockwiseWinding)) or_return

	ENSURE_SMALL(t, loaded.scale.x - def.scale.x, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, loaded.scale.y - def.scale.y, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, loaded.scale.z - def.scale.z, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, loaded.minHeight - def.globalMinimumHeight, b3.FLT_EPSILON) or_return
	ENSURE_SMALL(t, loaded.maxHeight - def.globalMaximumHeight, b3.FLT_EPSILON) or_return

	cellCount := (def.countX - 1) * (def.countZ - 1)
	for i: i32 = 0; i < cellCount; i += 1 {
		ENSURE(t, b3.GetHeightFieldMaterialIndices(loaded)[i] == materials[i]) or_return
	}

	// Recovered heights round-trip within the quantization tolerance.
	quantum := (def.globalMaximumHeight - def.globalMinimumHeight) / f32(max(u16))
	for i: i32 = 0; i < def.countX * def.countZ; i += 1 {
		recovered := loaded.minHeight + loaded.heightScale * f32(b3.GetHeightFieldCompressedHeights(loaded)[i])
		ENSURE_SMALL(t, recovered - heights[i], 2.0 * quantum) or_return
	}

	b3.DestroyHeightField(loaded)
	return true
}

@(private = "file")
ShapeCastVerticalStraddle :: proc(t: ^testing.T) -> bool {
	// Regression: a vertical shape cast whose swept volume straddles a cell
	// boundary must test every cell it overlaps. The field is flat at y = 0 with
	// only cell (0,0) solid; the surrounding cells are holes. Each sphere is
	// dropped straight down with its center nudged just past a boundary of the
	// solid cell, so that cell sits on the trailing (-x / -z) side of the sweep.
	// A cull AABB pinned to the leading box corner skips the solid cell entirely
	// and reports a miss.
	heights: [9]f32
	materials := [4]u8{0, b3.HEIGHT_FIELD_HOLE, b3.HEIGHT_FIELD_HOLE, b3.HEIGHT_FIELD_HOLE}

	def := b3.HeightFieldDef{}
	def.heights = &heights[0]
	def.materialIndices = &materials[0]
	def.scale = b3.Vec3{1.0, 1.0, 1.0}
	def.countX = 3
	def.countZ = 3
	def.globalMinimumHeight = -1.0
	def.globalMaximumHeight = 1.0
	def.clockwiseWinding = false

	hf := b3.CreateHeightField(&def)

	// Solid cell (0,0) spans x,z in [0,1]. Radius 0.3 with the center 0.05 past a
	// boundary still reaches back into the solid cell.
	radius: f32 = 0.3

	// Straddle the x = 1 edge: solid cell is on the -x side. Contact lands on the
	// cell edge, sqrt(0.05^2 + cy^2) = radius -> cy = 0.2958040,
	// fraction = (10 - cy) / 20 = 0.4852098.
	{
		center := b3.Vec3{1.05, 10.0, 0.5}
		input := b3.ShapeCastInput{}
		input.proxy = b3.ShapeProxy{&center, 1, radius}
		input.translation = b3.Vec3{0.0, -20.0, 0.0}
		input.maxFraction = 1.0

		out := b3.ShapeCastHeightField(hf, &input)
		ENSURE(t, out.hit == true) or_return
		ENSURE_SMALL(t, out.fraction - 0.4852098, 2e-3) or_return
	}

	// Straddle the z = 1 edge: solid cell is on the -z side (same geometry).
	{
		center := b3.Vec3{0.5, 10.0, 1.05}
		input := b3.ShapeCastInput{}
		input.proxy = b3.ShapeProxy{&center, 1, radius}
		input.translation = b3.Vec3{0.0, -20.0, 0.0}
		input.maxFraction = 1.0

		out := b3.ShapeCastHeightField(hf, &input)
		ENSURE(t, out.hit == true) or_return
		ENSURE_SMALL(t, out.fraction - 0.4852098, 2e-3) or_return
	}

	// Straddle the (1,1) corner: solid cell is diagonally trailing. Contact lands
	// on the corner vertex, sqrt(2*0.05^2 + cy^2) = radius -> cy = 0.2915476,
	// fraction = (10 - cy) / 20 = 0.4854226.
	{
		center := b3.Vec3{1.05, 10.0, 1.05}
		input := b3.ShapeCastInput{}
		input.proxy = b3.ShapeProxy{&center, 1, radius}
		input.translation = b3.Vec3{0.0, -20.0, 0.0}
		input.maxFraction = 1.0

		out := b3.ShapeCastHeightField(hf, &input)
		ENSURE(t, out.hit == true) or_return
		ENSURE_SMALL(t, out.fraction - 0.4854226, 2e-3) or_return
	}

	b3.DestroyHeightField(hf)
	return true
}

// Brute-force shape cast: cast the proxy against every (non-hole) triangle and
// keep the closest hit. This is the ground truth for b3ShapeCastHeightField.
@(private = "file")
BruteForceShapeCast :: proc(hf: ^b3.HeightFieldData, input: ^b3.ShapeCastInput) -> b3.CastOutput {
	best := b3.CastOutput{}
	bestFraction := input.maxFraction

	triangleCount := b3.GetHeightFieldTriangleCount(hf)
	for tIndex: i32 = 0; tIndex < triangleCount; tIndex += 1 {
		cellIndex := tIndex >> 1
		if b3.GetHeightFieldMaterialIndices(hf)[cellIndex] == b3.HEIGHT_FIELD_HOLE {
			continue
		}

		tri := b3.GetHeightFieldTriangle(hf, tIndex)

		pair := b3.ShapeCastPairInput{}
		pair.proxyA = b3.ShapeProxy{&tri.vertices[0], 3, 0.0}
		pair.proxyB = input.proxy
		pair.transform = b3.Transform_identity
		pair.translationB = input.translation
		pair.maxFraction = bestFraction
		pair.canEncroach = input.canEncroach

		out := b3.ShapeCast(&pair)
		if out.hit && out.fraction < bestFraction {
			bestFraction = out.fraction
			best = out
			best.triangleIndex = tIndex
		}
	}

	return best
}

@(private = "file")
ShapeCastBruteForce :: proc(t: ^testing.T) -> bool {
	// b3ShapeCastHeightField walks the grid and culls cells; the brute-force cast
	// against every triangle is the ground truth. The grid walk must never miss a
	// closer hit, regardless of cast direction, origin or radius.
	scale := b3.Vec3{2.0, 1.5, 2.0}
	hf := b3.CreateWave(10, 10, scale, 0.1, 0.03333, false)

	// Documented repro from sample/sample_mesh.cpp "Height Field": a sphere cast
	// that moves only in z (and y). Body at (-9,0,-9), world origin (5.5,4,2.913)
	// -> local (14.5,4,11.913). The grid walk used to terminate one row early
	// because it compared a clamped-sweep fraction against an input-space one.
	{
		origin := b3.Vec3{14.5, 4.0, 11.913}
		input := b3.ShapeCastInput{}
		input.proxy = b3.ShapeProxy{&origin, 1, 0.2}
		input.translation = b3.Vec3{0.0, -8.0, 6.397}
		input.maxFraction = 1.0

		grid := b3.ShapeCastHeightField(hf, &input)
		brute := BruteForceShapeCast(hf, &input)
		ENSURE(t, brute.hit == true) or_return
		ENSURE(t, grid.hit == brute.hit) or_return
		ENSURE_SMALL(t, grid.fraction - brute.fraction, 2e-3) or_return
	}

	// Sweep origins across the field with assorted directions and radii.
	radii := [?]f32{0.15, 0.4, 0.9}
	deltas := [?]b3.Vec3 {
		{0.0, -8.0, 0.0}, // vertical
		{0.0, -8.0, 6.4}, // z only (+ y)
		{5.1, -8.0, 0.0}, // x only (+ y)
		{0.0, -8.0, -6.4}, // -z
		{-5.1, -8.0, 0.0}, // -x
		{6.0, -8.0, 5.0}, // diagonal
		{-7.0, -8.0, 4.0}, // diagonal, mixed sign
		{9.0, -3.0, -9.0}, // shallow diagonal
	}

	failures: i32 = 0
	for xi: i32 = 0; xi < 5; xi += 1 {
		for zi: i32 = 0; zi < 5; zi += 1 {
			// 0.05 nudge keeps the swept box straddling cell boundaries.
			origin := b3.Vec3{1.0 + 4.0 * f32(xi) + 0.05, 4.0, 1.0 + 4.0 * f32(zi) + 0.05}

			for di: i32 = 0; di < i32(len(deltas)); di += 1 {
				for ri: i32 = 0; ri < i32(len(radii)); ri += 1 {
					input := b3.ShapeCastInput{}
					input.proxy = b3.ShapeProxy{&origin, 1, radii[ri]}
					input.translation = deltas[di]
					input.maxFraction = 1.0

					grid := b3.ShapeCastHeightField(hf, &input)
					brute := BruteForceShapeCast(hf, &input)

					diff := grid.fraction - brute.fraction
					diff = diff < 0.0 ? -diff : diff

					if grid.hit != brute.hit || (brute.hit && diff > 2e-3) {
						printf(
							"  mismatch: origin=(%.2f,%.2f,%.2f) delta=(%.2f,%.2f,%.2f) r=%.2f" +
							" grid(hit=%d,f=%.5f) brute(hit=%d,f=%.5f tri=%d)\n",
							origin.x,
							origin.y,
							origin.z,
							deltas[di].x,
							deltas[di].y,
							deltas[di].z,
							radii[ri],
							grid.hit,
							grid.fraction,
							brute.hit,
							brute.fraction,
							brute.triangleIndex,
						)
						failures += 1
					}
				}
			}
		}
	}

	ENSURE(t, failures == 0) or_return

	b3.DestroyHeightField(hf)
	return true
}

// Brute-force ray cast: cast the ray against every (non-hole) triangle and keep
// the closest hit. b3GetHeightFieldTriangle returns vertices in the same winding
// order that b3ShapeCastHeightField feeds to b3IntersectRayTriangle, so this is a
// pure traversal/culling check. The per-triangle math is identical.
@(private = "file")
BruteForceRayCast :: proc(hf: ^b3.HeightFieldData, input: ^b3.RayCastInput) -> b3.CastOutput {
	best := b3.CastOutput{}
	bestFraction := input.maxFraction

	rayStart := b3.LoadV(&input.origin.x)
	rayDelta := b3.LoadV(&input.translation.x)

	triangleCount := b3.GetHeightFieldTriangleCount(hf)
	for tIndex: i32 = 0; tIndex < triangleCount; tIndex += 1 {
		cellIndex := tIndex >> 1
		if b3.GetHeightFieldMaterialIndices(hf)[cellIndex] == b3.HEIGHT_FIELD_HOLE {
			continue
		}

		tri := b3.GetHeightFieldTriangle(hf, tIndex)
		v1 := b3.LoadV(&tri.vertices[0].x)
		v2 := b3.LoadV(&tri.vertices[1].x)
		v3 := b3.LoadV(&tri.vertices[2].x)

		// b3IntersectRayTriangle returns 1.0 on a miss.
		alpha := b3.IntersectRayTriangle(rayStart, rayDelta, v1, v2, v3)
		if alpha < bestFraction {
			bestFraction = alpha
			best.hit = true
			best.fraction = alpha
			best.triangleIndex = tIndex
		}
	}

	return best
}

@(private = "file")
RayCastBruteForce :: proc(t: ^testing.T) -> bool {
	// b3RayCastHeightField routes through the same grid walk as the shape cast
	// (radius-zero point proxy), so it is subject to the same early-termination
	// bug. The brute-force cast against every triangle is the ground truth.
	scale := b3.Vec3{2.0, 1.5, 2.0}
	hf := b3.CreateWave(10, 10, scale, 0.1, 0.03333, false)

	deltas := [?]b3.Vec3 {
		{0.0, -8.0, 0.0}, // straight down
		{0.0, -8.0, 12.0}, // down + z
		{12.0, -8.0, 0.0}, // down + x
		{0.0, -8.0, -12.0}, // down - z
		{-12.0, -8.0, 0.0}, // down - x
		{14.0, -8.0, 11.0}, // diagonal
		{-13.0, -8.0, 9.0}, // diagonal, mixed sign
		{16.0, -4.0, -15.0}, // shallow diagonal
	}

	failures: i32 = 0
	for xi: i32 = 0; xi < 5; xi += 1 {
		for zi: i32 = 0; zi < 5; zi += 1 {
			// 0.05 nudge keeps the ray off cell boundaries.
			origin := b3.Vec3{1.0 + 4.0 * f32(xi) + 0.05, 4.0, 1.0 + 4.0 * f32(zi) + 0.05}

			for di: i32 = 0; di < i32(len(deltas)); di += 1 {
				input := b3.RayCastInput{}
				input.origin = origin
				input.translation = deltas[di]
				input.maxFraction = 1.0

				grid := b3.RayCastHeightField(hf, &input)
				brute := BruteForceRayCast(hf, &input)

				diff := grid.fraction - brute.fraction
				diff = diff < 0.0 ? -diff : diff

				if grid.hit != brute.hit || (brute.hit && diff > 1e-4) {
					printf(
						"  mismatch: origin=(%.2f,%.2f,%.2f) delta=(%.2f,%.2f,%.2f)" +
						" grid(hit=%d,f=%.6f tri=%d) brute(hit=%d,f=%.6f tri=%d)\n",
						origin.x,
						origin.y,
						origin.z,
						deltas[di].x,
						deltas[di].y,
						deltas[di].z,
						grid.hit,
						grid.fraction,
						grid.triangleIndex,
						brute.hit,
						brute.fraction,
						brute.triangleIndex,
					)
					failures += 1
				}
			}
		}
	}

	ENSURE(t, failures == 0) or_return

	b3.DestroyHeightField(hf)
	return true
}

// Flat 3x3 vertex field at y = 0, one cell material per entry. Caller destroys.
@(private = "file")
MakeFlatField :: proc(materials: [^]u8, clockwise: bool) -> ^b3.HeightFieldData {
	heights: [9]f32

	def := b3.HeightFieldDef{}
	def.heights = &heights[0]
	def.materialIndices = materials
	def.scale = b3.Vec3{1.0, 1.0, 1.0}
	def.countX = 3
	def.countZ = 3
	def.globalMinimumHeight = -1.0
	def.globalMaximumHeight = 1.0
	def.clockwiseWinding = clockwise

	return b3.CreateHeightField(&def)
}

@(private = "file")
ShapeCastBackside :: proc(t: ^testing.T) -> bool {
	materials := [4]u8{1, 0, 0, 0}
	hf := MakeFlatField(&materials[0], false)

	// Falling onto the front (upper) face reports the hit, the triangle, and its material.
	// The surface sits at y = 0, so a sphere of radius 0.2 stops with its center at y = 0.2.
	above := b3.Vec3{0.3, 5.0, 0.25}
	down := b3.ShapeCastInput{}
	down.proxy = b3.ShapeProxy{&above, 1, 0.2}
	down.translation = b3.Vec3{0.0, -10.0, 0.0}
	down.maxFraction = 1.0

	out := b3.ShapeCastHeightField(hf, &down)
	ENSURE(t, out.hit == true) or_return
	ENSURE_SMALL(t, out.fraction - 0.48, 1e-2) or_return
	ENSURE(t, out.normal.y > 0.99) or_return
	ENSURE(t, out.triangleIndex == 0 || out.triangleIndex == 1) or_return
	ENSURE(t, out.materialIndex == 1) or_return

	// Rising from behind the face is culled
	below := b3.Vec3{0.3, -5.0, 0.25}
	up := b3.ShapeCastInput{}
	up.proxy = b3.ShapeProxy{&below, 1, 0.2}
	up.translation = b3.Vec3{0.0, 10.0, 0.0}
	up.maxFraction = 1.0

	missed := b3.ShapeCastHeightField(hf, &up)
	ENSURE(t, missed.hit == false) or_return

	b3.DestroyHeightField(hf)
	return true
}

// A clockwise height field faces down, see HeightFieldWinding. Shape cast backside
// culling must respect the flag: a rising sphere hits from below, not from above.
@(private = "file")
ShapeCastClockwiseWinding :: proc(t: ^testing.T) -> bool {
	materials := [4]u8{1, 0, 0, 0}
	hf := MakeFlatField(&materials[0], true)

	below := b3.Vec3{0.3, -5.0, 0.25}
	up := b3.ShapeCastInput{}
	up.proxy = b3.ShapeProxy{&below, 1, 0.2}
	up.translation = b3.Vec3{0.0, 10.0, 0.0}
	up.maxFraction = 1.0

	out := b3.ShapeCastHeightField(hf, &up)
	ENSURE(t, out.hit == true) or_return
	ENSURE_SMALL(t, out.fraction - 0.48, 1e-2) or_return
	ENSURE(t, out.normal.y < -0.99) or_return
	ENSURE(t, out.triangleIndex == 0 || out.triangleIndex == 1) or_return
	ENSURE(t, out.materialIndex == 1) or_return

	above := b3.Vec3{0.3, 5.0, 0.25}
	down := b3.ShapeCastInput{}
	down.proxy = b3.ShapeProxy{&above, 1, 0.2}
	down.translation = b3.Vec3{0.0, -10.0, 0.0}
	down.maxFraction = 1.0

	missed := b3.ShapeCastHeightField(hf, &down)
	ENSURE(t, missed.hit == false) or_return

	b3.DestroyHeightField(hf)
	return true
}

// Ray casts run through the shape cast. The default winding faces up, so a ray from below is the
// back side and gets culled.
@(private = "file")
RayCastBackside :: proc(t: ^testing.T) -> bool {
	materials := [4]u8{1, 0, 0, 0}
	hf := MakeFlatField(&materials[0], false)

	down := b3.RayCastInput{}
	down.origin = b3.Vec3{0.3, 5.0, 0.25}
	down.translation = b3.Vec3{0.0, -10.0, 0.0}
	down.maxFraction = 1.0

	out := b3.RayCastHeightField(hf, &down)
	ENSURE(t, out.hit == true) or_return
	ENSURE_SMALL(t, out.fraction - 0.5, 1e-3) or_return
	ENSURE(t, out.normal.y > 0.99) or_return
	ENSURE(t, out.triangleIndex == 0 || out.triangleIndex == 1) or_return
	ENSURE(t, out.materialIndex == 1) or_return

	up := b3.RayCastInput{}
	up.origin = b3.Vec3{0.3, -5.0, 0.25}
	up.translation = b3.Vec3{0.0, 10.0, 0.0}
	up.maxFraction = 1.0

	missed := b3.RayCastHeightField(hf, &up)
	ENSURE(t, missed.hit == false) or_return

	b3.DestroyHeightField(hf)
	return true
}

// A clockwise field faces down. The winding swap already flips the triangle, so the ray normal
// must not be flipped a second time.
@(private = "file")
RayCastClockwiseWinding :: proc(t: ^testing.T) -> bool {
	materials := [4]u8{1, 0, 0, 0}
	hf := MakeFlatField(&materials[0], true)

	up := b3.RayCastInput{}
	up.origin = b3.Vec3{0.3, -5.0, 0.25}
	up.translation = b3.Vec3{0.0, 10.0, 0.0}
	up.maxFraction = 1.0

	out := b3.RayCastHeightField(hf, &up)
	ENSURE(t, out.hit == true) or_return
	ENSURE_SMALL(t, out.fraction - 0.5, 1e-3) or_return
	ENSURE(t, out.normal.y < -0.99) or_return
	ENSURE(t, out.triangleIndex == 0 || out.triangleIndex == 1) or_return
	ENSURE(t, out.materialIndex == 1) or_return

	down := b3.RayCastInput{}
	down.origin = b3.Vec3{0.3, 5.0, 0.25}
	down.translation = b3.Vec3{0.0, -10.0, 0.0}
	down.maxFraction = 1.0

	missed := b3.RayCastHeightField(hf, &down)
	ENSURE(t, missed.hit == false) or_return

	b3.DestroyHeightField(hf)
	return true
}

@(test)
HeightFieldTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, HeightFieldCreate) {return}
	if !RUN_SUBTEST(t, HeightFieldTriangleIndex) {return}
	if !RUN_SUBTEST(t, HeightFieldWinding) {return}
	if !RUN_SUBTEST(t, RayCastFlatField) {return}
	if !RUN_SUBTEST(t, OverlapAtSurface) {return}
	if !RUN_SUBTEST(t, FileRoundtrip) {return}
	if !RUN_SUBTEST(t, ShapeCastVerticalStraddle) {return}
	if !RUN_SUBTEST(t, ShapeCastBruteForce) {return}
	if !RUN_SUBTEST(t, RayCastBruteForce) {return}
	if !RUN_SUBTEST(t, ShapeCastBackside) {return}
	if !RUN_SUBTEST(t, ShapeCastClockwiseWinding) {return}
	if !RUN_SUBTEST(t, RayCastBackside) {return}
	if !RUN_SUBTEST(t, RayCastClockwiseWinding) {return}
}
