// Ported from box3d src/height_field.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:runtime"
import "core:fmt"
import "core:mem"
import "core:os"
import "core:strconv"

/*
	Convention

	index = row * columnCount + column
	height = minHeight + heightScale * compressedHeights[index];

	column = index % columnCount;
	row = index / columnCount;

	x-axis : columns
	z-axis : rows

	00 --- 01 --- 02 --- 03 X
	|  0   |  1   |  2   |
	04 --- 05 --- 06 --- 07
	|  3   |  4   |  5   |
	08 --- 09 --- 10 --- 11
	|  6   |  7   |  8   |
	12 --- 13 --- 14 --- 15
	Z

	The quads exist before the column and row ends: row < rowCount - 1 and column < columnCount - 1
	quadIndex = row * (columnCount - 1) + column

	Quad origin index from quad index (needs row):
	index = quadIndex + row * columnCount

	Triangle index is related to the quad index
	triangleIndex = 2 * quadIndex + (0/1)
	quadIndex = triangleIndex / 2

	Row and column from quad index:
	row = quadIndex / (columnCount - 1)
	column = quadIndex - row * (columnCount - 1)

	The triangle diagonal is fixed.

	triangle0 = {00, 04, 01} -> {11, 21, 12}
	triangle1 = {04, 05, 01} -> {22, 12, 21}

	11      12
	00 ---- 01
	|     / |
	| 0 / 1 | 1
	| /     |
	04 ---- 05
	21     22

	For adjacency we have

	   NA
	   00 ---- 01 ---- 02
	   |     / |     / |
	NA | 0 / 1 | 2 / 3 |
	   | /     | /     |
	   04 ---- 05 ---- 06
	   |     / |     / |
	NA | 6 / 7 | 8 / 9 |
	   | /     | /     |
	   08 ---- 09 ---- 10

	   0: NA, NA, 1
	   1: 0, 6, 2

	Triangle layouts

	   11  3  12
	   1 ----  3
	   |     /
	 1 | 0 / 2
	   | /
	   2
	   21

			  12
			  2
			/ |
		2 / 1 | 1
		/     |
	   3 ---- 1
	   21  3  22
 */

CreateHeightField :: proc(data: ^HeightFieldDef) -> ^HeightFieldData {
	columnCount := data.countX
	rowCount := data.countZ

	heightCount := columnCount * rowCount
	assert(heightCount >= 4)

	cellCount := (columnCount - 1) * (rowCount - 1)
	triangleCount := 2 * cellCount

	// Single blob: struct followed by the height, material, and flag arrays. Layout
	// mirrors b3HullData/b3MeshData so the recording path can copy it with one memcpy.
	byteCount := AlignUp8(size_of(HeightFieldData))
	heightsOffset := i32(byteCount)
	byteCount += AlignUp8(int(heightCount) * size_of(u16))
	materialOffset := i32(byteCount)
	byteCount += AlignUp8(int(cellCount) * size_of(u8))
	flagsOffset := i32(byteCount)
	byteCount += AlignUp8(int(triangleCount) * size_of(u8))

	// Zero the whole blob so alignment padding is defined. The construction-time hash
	// sweeps raw bytes and would otherwise pick up uninitialized padding.
	hf := cast(^HeightFieldData)AllocHost(byteCount) // unified: geometry is the user's, never in a world arena (world_arena.odin)
	mem.zero(hf, byteCount)

	hf.version = HEIGHT_FIELD_VERSION
	hf.byteCount = i32(byteCount)
	hf.scale = data.scale
	hf.columnCount = columnCount
	hf.rowCount = rowCount
	hf.heightsOffset = heightsOffset
	hf.materialOffset = materialOffset
	hf.flagsOffset = flagsOffset
	hf.clockwise = data.clockwiseWinding ? 1 : 0

	compressedHeights := cast([^]u16)(uintptr(hf) + uintptr(heightsOffset))
	materialIndices := cast([^]u8)(uintptr(hf) + uintptr(materialOffset))
	flags := cast([^]u8)(uintptr(hf) + uintptr(flagsOffset))

	heights := data.heights

	assert(data.globalMinimumHeight <= data.globalMaximumHeight)
	hf.minHeight = data.globalMinimumHeight
	hf.maxHeight = data.globalMaximumHeight

	height := MaxFloat(hf.maxHeight - hf.minHeight, LINEAR_SLOP)
	hf.heightScale = height / f32(max(u16))

	lowerHeightBound := hf.maxHeight
	upperHeightBound := hf.minHeight

	invHeightScale := 1.0 / hf.heightScale
	for i: i32 = 0; i < heightCount; i += 1 {
		clampedHeight := ClampFloat(heights[i], hf.minHeight, hf.maxHeight)
		scaledHeight := (clampedHeight - hf.minHeight) * invHeightScale
		compressedHeights[i] = u16(MinFloat(scaledHeight, f32(max(u16))))

		lowerHeightBound = MinFloat(lowerHeightBound, clampedHeight)
		upperHeightBound = MaxFloat(upperHeightBound, clampedHeight)
	}

	// Use decompressed heights for accurate convexity metrics.
	decompressedHeights := cast([^]f32)Alloc(int(heightCount) * size_of(f32))
	for i: i32 = 0; i < heightCount; i += 1 {
		decompressedHeights[i] = hf.minHeight + hf.heightScale * f32(compressedHeights[i])
	}
	heights = decompressedHeights

	if data.materialIndices != nil {
		for i: i32 = 0; i < cellCount; i += 1 {
			materialIndices[i] = data.materialIndices[i]
		}
	} else {
		for i: i32 = 0; i < cellCount; i += 1 {
			materialIndices[i] = 0
		}
	}

	hf.aabb.lowerBound = Vec3{0.0, hf.scale.y * lowerHeightBound, 0.0}
	hf.aabb.upperBound = Vec3 {
		hf.scale.x * f32(hf.columnCount - 1),
		hf.scale.y * upperHeightBound,
		hf.scale.z * f32(hf.rowCount - 1),
	}

	cos5Deg: f32 = 0.9962
	scale := hf.scale

	triangleIndex: i32 = 0
	for row: i32 = 0; row < rowCount - 1; row += 1 {
		for column: i32 = 0; column < columnCount - 1; column += 1 {
			// todo compute convexity flags
			// This requires a couple things
			// - determine all 3 adjacent triangles for each triangle
			// - consider clockwise winding
			// - consider borders where there is no adjacent triangle

			triangleIndex1 := triangleIndex
			triangleIndex2 := triangleIndex + 1
			triangleIndex += 2

			cellIndex := row * (columnCount - 1) + column

			if materialIndices[cellIndex] == HEIGHT_FIELD_HOLE {
				continue
			}

			flags1: i32 = 0
			flags2: i32 = 0

			plane1, plane2: Plane
			center1, center2: Vec3

			index11 := row * columnCount + column
			index12 := index11 + 1
			index21 := (row + 1) * columnCount + column
			index22 := index21 + 1

			{
				height11 := heights[index11]
				height12 := heights[index12]
				height21 := heights[index21]
				height22 := heights[index22]

				x1 := f32(column)
				x2 := f32(column + 1)
				z1 := f32(row)
				z2 := f32(row + 1)

				// triangle 0 : 11, 21, 12
				vs0: [3]Vec3
				vs0[0] = Mul(scale, Vec3{x1, height11, z1})
				vs0[1] = Mul(scale, Vec3{x1, height21, z2})
				vs0[2] = Mul(scale, Vec3{x2, height12, z1})
				plane1 = MakePlaneFromPoints(vs0[0], vs0[1], vs0[2])

				center1 = MulSV(1.0 / 3.0, Add(Add(vs0[0], vs0[1]), vs0[2]))

				// triangle 1 : 22, 12, 21
				vs1: [3]Vec3
				vs1[0] = Mul(scale, Vec3{x2, height22, z2})
				vs1[1] = Mul(scale, Vec3{x2, height12, z1})
				vs1[2] = Mul(scale, Vec3{x1, height21, z2})
				plane2 = MakePlaneFromPoints(vs1[0], vs1[1], vs1[2])

				center2 = MulSV(1.0 / 3.0, Add(Add(vs1[0], vs1[1]), vs1[2]))

				separation := PlaneSeparation(plane1, vs1[0])
				cosAngle := Dot(plane1.normal, plane2.normal)
				if separation > 0.0 || cosAngle > cos5Deg {
					flags1 |= concaveEdge2
					flags2 |= concaveEdge2
				}
				if separation < 0.0 || cosAngle > cos5Deg {
					flags1 |= inverseConcaveEdge2
					flags2 |= inverseConcaveEdge2
				}
			}

			_ = center1
			_ = center2

			// top
			topCellIndex := (row - 1) * (columnCount - 1) + column
			if row > 0 && materialIndices[topCellIndex] != HEIGHT_FIELD_HOLE {
				assert(0 <= topCellIndex && topCellIndex < cellCount)

				r := row - 1
				c := column

				i11 := r * columnCount + c
				i12 := i11 + 1
				i21 := (r + 1) * columnCount + c
				i22 := i21 + 1

				assert(i21 == index11)
				assert(i22 == index12)

				// float h11 = heights[i11];
				h12 := heights[i12]
				h21 := heights[i21]
				h22 := heights[i22]

				x1 := f32(c)
				x2 := f32(c + 1)
				z1 := f32(r)
				z2 := f32(r + 1)

				// triangle 1
				vs: [3]Vec3
				vs[0] = Mul(scale, Vec3{x2, h22, z2})
				vs[1] = Mul(scale, Vec3{x2, h12, z1})
				vs[2] = Mul(scale, Vec3{x1, h21, z2})

				n := Normalize(Cross(Sub(vs[1], vs[0]), Sub(vs[2], vs[0])))

				separation := PlaneSeparation(plane1, vs[1])
				cosAngle := Dot(plane1.normal, n)
				if separation > 0.0 || cosAngle > cos5Deg {
					flags1 |= concaveEdge3
				}
				if separation < 0.0 || cosAngle > cos5Deg {
					flags1 |= inverseConcaveEdge3
				}
			}

			bottomCellIndex := (row + 1) * (columnCount - 1) + column
			if row + 1 < rowCount - 1 && materialIndices[bottomCellIndex] != HEIGHT_FIELD_HOLE {
				assert(0 <= bottomCellIndex && bottomCellIndex < cellCount)

				r := row + 1
				c := column

				i11 := r * columnCount + c
				i12 := i11 + 1
				i21 := (r + 1) * columnCount + c
				// int i22 = i21 + 1;

				assert(i11 == index21)
				assert(i12 == index22)

				h11 := heights[i11]
				h12 := heights[i12]
				h21 := heights[i21]
				// float h22 = heights[i22];

				x1 := f32(c)
				x2 := f32(c + 1)
				z1 := f32(r)
				z2 := f32(r + 1)

				// triangle 0
				vs: [3]Vec3
				vs[0] = Mul(scale, Vec3{x1, h11, z1})
				vs[1] = Mul(scale, Vec3{x1, h21, z2})
				vs[2] = Mul(scale, Vec3{x2, h12, z1})

				n := Normalize(Cross(Sub(vs[1], vs[0]), Sub(vs[2], vs[0])))

				separation := PlaneSeparation(plane2, vs[1])
				cosAngle := Dot(plane2.normal, n)
				if separation > 0.0 || cosAngle > cos5Deg {
					flags2 |= concaveEdge3
				}
				if separation < 0.0 || cosAngle > cos5Deg {
					flags2 |= inverseConcaveEdge3
				}
			}

			leftCellIndex := row * (columnCount - 1) + column - 1
			if column - 1 >= 0 && materialIndices[leftCellIndex] != HEIGHT_FIELD_HOLE {
				assert(0 <= leftCellIndex && leftCellIndex < cellCount)

				r := row
				c := column - 1

				i11 := r * columnCount + c
				i12 := i11 + 1
				i21 := (r + 1) * columnCount + c
				i22 := i21 + 1

				assert(i12 == index11)
				assert(i22 == index21)

				// float h11 = heights[i11];
				h12 := heights[i12]
				h21 := heights[i21]
				h22 := heights[i22]

				x1 := f32(c)
				x2 := f32(c + 1)
				z1 := f32(r)
				z2 := f32(r + 1)

				// triangle 1
				vs: [3]Vec3
				vs[0] = Mul(scale, Vec3{x2, h22, z2})
				vs[1] = Mul(scale, Vec3{x2, h12, z1})
				vs[2] = Mul(scale, Vec3{x1, h21, z2})

				n := Normalize(Cross(Sub(vs[1], vs[0]), Sub(vs[2], vs[0])))

				separation := PlaneSeparation(plane1, vs[2])
				cosAngle := Dot(plane1.normal, n)
				if separation > 0.0 || cosAngle > cos5Deg {
					flags1 |= concaveEdge1
				}
				if separation < 0.0 || cosAngle > cos5Deg {
					flags1 |= inverseConcaveEdge1
				}
			}

			rightCellIndex := row * (columnCount - 1) + column + 1
			if column + 1 < columnCount - 1 && materialIndices[rightCellIndex] != HEIGHT_FIELD_HOLE {
				assert(0 <= rightCellIndex && rightCellIndex < cellCount)

				r := row
				c := column + 1

				i11 := r * columnCount + c
				i12 := i11 + 1
				i21 := (r + 1) * columnCount + c
				// int i22 = i21 + 1;

				assert(i11 == index12)
				assert(i21 == index22)

				h11 := heights[i11]
				h12 := heights[i12]
				h21 := heights[i21]
				// float h22 = heights[i22];

				x1 := f32(c)
				x2 := f32(c + 1)
				z1 := f32(r)
				z2 := f32(r + 1)

				// triangle 0
				vs: [3]Vec3
				vs[0] = Mul(scale, Vec3{x1, h11, z1})
				vs[1] = Mul(scale, Vec3{x1, h21, z2})
				vs[2] = Mul(scale, Vec3{x2, h12, z1})

				n := Normalize(Cross(Sub(vs[1], vs[0]), Sub(vs[2], vs[0])))

				separation := PlaneSeparation(plane2, vs[2])
				cosAngle := Dot(plane2.normal, n)
				if separation > 0.0 || cosAngle > cos5Deg {
					flags2 |= concaveEdge1
				}
				if separation < 0.0 || cosAngle > cos5Deg {
					flags2 |= inverseConcaveEdge1
				}
			}

			assert(0 <= flags1 && flags1 <= i32(max(u8)))
			assert(0 <= flags2 && flags2 <= i32(max(u8)))

			flags[triangleIndex1] = u8(flags1)
			flags[triangleIndex2] = u8(flags2)
		}
	}

	assert(triangleIndex == triangleCount)

	Free(decompressedHeights, int(heightCount) * size_of(f32))

	hf.hash = 0
	hf.hash = Hash64NonZero(cast([^]u8)hf, hf.byteCount)

	return hf
}

#assert(concaveEdge3 == 4 * concaveEdge1, "bit math")
#assert(inverseConcaveEdge3 == 4 * inverseConcaveEdge1, "bit math")

// Decode the four corner vertices of a height field cell into local space.
// Output order matches the index naming used throughout this file:
// corners[0] = (column, row), corners[1] = (column + 1, row),
// corners[2] = (column, row + 1), corners[3] = (column + 1, row + 1).
@(private = "file")
GetHeightFieldCellCorners :: #force_inline proc(hf: ^HeightFieldData, row: i32, column: i32, corners: ^[4]Vec3) {
	assert(0 <= row && row < hf.rowCount - 1 && 0 <= column && column < hf.columnCount - 1)

	columnCount := hf.columnCount
	index11 := row * columnCount + column
	index12 := index11 + 1
	index21 := (row + 1) * columnCount + column
	index22 := index21 + 1

	minHeight := hf.minHeight
	heightScale := hf.heightScale
	heights := GetHeightFieldCompressedHeights(hf)

	height11 := minHeight + heightScale * f32(heights[index11])
	height12 := minHeight + heightScale * f32(heights[index12])
	height21 := minHeight + heightScale * f32(heights[index21])
	height22 := minHeight + heightScale * f32(heights[index22])

	x1 := f32(column)
	x2 := f32(column + 1)
	z1 := f32(row)
	z2 := f32(row + 1)

	scale := hf.scale
	corners[0] = Mul(scale, Vec3{x1, height11, z1})
	corners[1] = Mul(scale, Vec3{x2, height12, z1})
	corners[2] = Mul(scale, Vec3{x1, height21, z2})
	corners[3] = Mul(scale, Vec3{x2, height22, z2})
}

GetHeightFieldTriangle :: proc(heightField: ^HeightFieldData, triangleIndex: i32) -> Triangle {
	assert(0 <= triangleIndex)
	assert(triangleIndex < 2 * (heightField.columnCount - 1) * (heightField.rowCount - 1))

	triangle: Triangle
	triangle.flags = i32(GetHeightFieldFlags(heightField)[triangleIndex])

	columnCount := heightField.columnCount
	quadIndex := triangleIndex >> 1
	row := quadIndex / (columnCount - 1)
	column := quadIndex - row * (columnCount - 1)

	index11 := row * columnCount + column
	index12 := index11 + 1
	index21 := (row + 1) * columnCount + column
	index22 := index21 + 1

	cellIndex := row * (columnCount - 1) + column

	assert(quadIndex == cellIndex)
	assert(GetHeightFieldMaterialIndices(heightField)[cellIndex] != HEIGHT_FIELD_HOLE)
	_ = cellIndex

	corners: [4]Vec3
	GetHeightFieldCellCorners(heightField, row, column, &corners)

	if (triangleIndex & 1) == 0 {
		triangle.vertices[0] = corners[0]
		triangle.vertices[1] = corners[2]
		triangle.vertices[2] = corners[1]
		triangle.i1 = index11
		triangle.i2 = index21
		triangle.i3 = index12
	} else {
		triangle.vertices[0] = corners[3]
		triangle.vertices[1] = corners[1]
		triangle.vertices[2] = corners[2]
		triangle.i1 = index22
		triangle.i2 = index12
		triangle.i3 = index21
	}

	if heightField.clockwise != 0 {
		triangle.vertices[1], triangle.vertices[2] = triangle.vertices[2], triangle.vertices[1]
		triangle.i2, triangle.i3 = triangle.i3, triangle.i2

		// Reversing winding swaps edge1 and edge3; edge2 (the diagonal) is preserved.
		flags := triangle.flags
		edge1Bits := flags & (concaveEdge1 | inverseConcaveEdge1)
		edge3Bits := flags & (concaveEdge3 | inverseConcaveEdge3)
		flags &= ~i32(concaveEdge1 | concaveEdge3 | inverseConcaveEdge1 | inverseConcaveEdge3)
		flags |= edge1Bits << 2
		flags |= edge3Bits >> 2
		triangle.flags = flags
	}

	return triangle
}

GetHeightFieldMaterial :: proc(heightField: ^HeightFieldData, triangleIndex: i32) -> i32 {
	assert(0 <= triangleIndex)
	assert(triangleIndex < 2 * (heightField.columnCount - 1) * (heightField.rowCount - 1))

	cellIndex := triangleIndex >> 1
	return i32(GetHeightFieldMaterialIndices(heightField)[cellIndex])
}

ComputeHeightFieldAABB :: proc(shape: ^HeightFieldData, transform: Transform) -> AABB {
	return AABB_Transform(transform, shape.aabb)
}

RayCastHeightField :: proc(heightField: ^HeightFieldData, input: ^RayCastInput) -> CastOutput {
	shapeCastInput := ShapeCastInput{}
	shapeCastInput.proxy = ShapeProxy{&input.origin, 1, 0.0}
	shapeCastInput.translation = input.translation
	shapeCastInput.maxFraction = input.maxFraction

	return ShapeCastHeightField(heightField, &shapeCastInput)
}

// todo advance cast to the grid border immediately if it starts outside the row/column range
// todo terminate the cast immediately if it leaves the row/column range
ShapeCastHeightField :: proc(heightField: ^HeightFieldData, input: ^ShapeCastInput) -> CastOutput {
	shapeBounds := MakeAABB(input.proxy.points, input.proxy.count, input.proxy.radius)
	shapeTranslation := input.translation
	scale := heightField.scale

	// The shape start is the center of the proxy.
	shapeStart := AABB_Center(shapeBounds)
	shapeDelta := MulSV(input.maxFraction, shapeTranslation)
	shapeEnd := Add(shapeStart, shapeDelta)

	result := CastOutput{}

	shapeExtents := AABB_Extents(shapeBounds)
	margin := Vec3{MAX_AABB_MARGIN, MAX_AABB_MARGIN, MAX_AABB_MARGIN}
	combinedBounds := AABB {
		Sub(Sub(heightField.aabb.lowerBound, shapeExtents), margin),
		Add(Add(heightField.aabb.upperBound, shapeExtents), margin),
	}

	minFraction, maxFraction: f32
	intersects := RayCastAABB(combinedBounds, shapeStart, shapeEnd, &minFraction, &maxFraction)
	if intersects == false {
		return result
	}

	// These are for walking the grid, not the triangle cast.
	// The triangle cast uses the unclamped ray and fraction.
	clampedStart := MulAdd(shapeStart, minFraction, shapeDelta)
	clampedDelta := MulSV(maxFraction - minFraction, shapeDelta)
	clampedEnd := Add(clampedStart, clampedDelta)

	// Preserve the un-shifted center sweep. clampedStart/clampedEnd get pushed out to the
	// leading box corner below to drive the grid DDA, but the swept-volume AABB used to
	// cull cells must stay centered on the actual shape path.
	centerStart := clampedStart
	centerEnd := clampedEnd

	// The grid traversal starts from the leading shape bounds corner
	signX, signZ: f32
	if shapeTranslation.x >= 0.0 {
		clampedStart.x += shapeExtents.x
		signX = 1.0
	} else {
		clampedStart.x -= shapeExtents.x
		signX = -1.0
	}

	if shapeTranslation.z >= 0.0 {
		clampedStart.z += shapeExtents.z
		signZ = 1.0
	} else {
		clampedStart.z -= shapeExtents.z
		signZ = -1.0
	}

	// Shift the end as well
	clampedEnd = Add(clampedStart, clampedDelta)

	// Row and column range for the shape cast
	columnStart := i32(floorf(clampedStart.x / scale.x))
	columnEnd := i32(floorf(clampedEnd.x / scale.x))
	rowStart := i32(floorf(clampedStart.z / scale.z))
	rowEnd := i32(floorf(clampedEnd.z / scale.z))

	absClampedDelta := Abs(clampedDelta)

	// Precompute increments for row and column traversal.
	// The ray can be slightly tilted yet remain within a single row or column
	// once rasterized.
	deltaAlphaX: f32
	nextFractionX: f32
	deltaColumn: i32

	if columnStart < columnEnd {
		assert(absClampedDelta.x > 0.0)

		// Going forward on x columns
		deltaAlphaX = scale.x / absClampedDelta.x
		nextFractionX = (scale.x * f32(columnStart + 1) - clampedStart.x) / absClampedDelta.x
		deltaColumn = 1
	} else if columnEnd < columnStart {
		assert(absClampedDelta.x > 0.0)

		// Going backwards on x columns
		deltaAlphaX = scale.x / absClampedDelta.x
		nextFractionX = (clampedStart.x - scale.x * f32(columnStart)) / absClampedDelta.x
		deltaColumn = -1
	} else {
		// Cast stays in a single column
		deltaAlphaX = 0.0
		nextFractionX = FLT_MAX
		deltaColumn = 0
	}

	deltaAlphaZ: f32
	nextFractionZ: f32
	deltaRow: i32

	if rowStart < rowEnd {
		assert(absClampedDelta.z > 0.0)

		// Going forward on z rows
		deltaAlphaZ = scale.z / absClampedDelta.z
		nextFractionZ = (scale.z * f32(rowStart + 1) - clampedStart.z) / absClampedDelta.z
		deltaRow = 1
	} else if rowEnd < rowStart {
		assert(absClampedDelta.z > 0.0)

		// Going backwards on z rows
		deltaAlphaZ = scale.z / absClampedDelta.z
		nextFractionZ = (clampedStart.z - scale.z * f32(rowStart)) / absClampedDelta.z
		deltaRow = -1
	} else {
		// Cast stays in a single row
		deltaAlphaZ = 0.0
		nextFractionZ = FLT_MAX
		deltaRow = 0
	}

	// Column and row range for 2D projected initial shape bounds
	boxColumnHead := columnStart
	boxRowHead := rowStart

	boxColumnTail := i32(floorf((clampedStart.x - 2.0 * signX * shapeExtents.x) / scale.x))
	boxRowTail := i32(floorf((clampedStart.z - 2.0 * signZ * shapeExtents.z) / scale.z))

	bestFraction := input.maxFraction

	// nextFractionX / nextFractionZ advance in units of the clamped sweep
	// [minFraction, maxFraction], but bestFraction is a fraction of the full input
	// translation. Precompute the affine map from clamped space to input space so
	// the loop termination test compares like with like. Otherwise it can exit
	// early and miss a closer hit in a later cell.
	gridFractionScale := input.maxFraction * (maxFraction - minFraction)
	gridFractionOffset := input.maxFraction * minFraction

	rowCount := heightField.rowCount
	columnCount := heightField.columnCount
	cellCount := (heightField.rowCount - 1) * (heightField.columnCount - 1)
	_ = cellCount

	pairInput := ShapeCastPairInput{}
	pairInput.proxyB = input.proxy
	pairInput.transform = Transform_identity
	pairInput.translationB = input.translation
	pairInput.canEncroach = input.canEncroach

	castBounds: AABB
	castBounds.lowerBound = Sub(Min(centerStart, centerEnd), shapeExtents)
	castBounds.upperBound = Add(Max(centerStart, centerEnd), shapeExtents)

	// The ray origin is the center of the shape.
	rayOrigin := LoadV(&shapeStart.x)
	rayTranslation := LoadV(&shapeTranslation.x)

	clockwise := heightField.clockwise != 0

	for true {
		column1, column2: i32
		if boxColumnTail < boxColumnHead {
			column1 = boxColumnTail
			column2 = boxColumnHead
		} else {
			column1 = boxColumnHead
			column2 = boxColumnTail
		}

		row1, row2: i32
		if boxRowTail < boxRowHead {
			row1 = boxRowTail
			row2 = boxRowHead
		} else {
			row1 = boxRowHead
			row2 = boxRowTail
		}

		for row := row1; row <= row2; row += 1 {
			if row < 0 || rowCount - 1 <= row {
				continue
			}

			for column := column1; column <= column2; column += 1 {
				if column < 0 || columnCount - 1 <= column {
					continue
				}

				cellIndex := row * (columnCount - 1) + column
				assert(cellIndex < cellCount)

				materialIndex := GetHeightFieldMaterialIndices(heightField)[cellIndex]
				if materialIndex == HEIGHT_FIELD_HOLE {
					continue
				}

				corners: [4]Vec3
				GetHeightFieldCellCorners(heightField, row, column, &corners)
				point11 := corners[0]
				point12 := corners[1]
				point21 := corners[2]
				point22 := corners[3]

				if clockwise {
					point12, point21 = point21, point12
				}

				// I know the min/max x and z values, but not the min/max heights.
				bounds: AABB
				bounds.lowerBound = Min(Min(point11, point12), Min(point21, point22))
				bounds.upperBound = Max(Max(point11, point12), Max(point21, point22))

				if AABB_Overlaps(castBounds, bounds) == false {
					continue
				}

				quadIndex := row * (columnCount - 1) + column
				triangleIndex1 := 2 * quadIndex
				triangleIndex2 := triangleIndex1 + 1

				if input.proxy.count == 1 && input.proxy.radius == 0.0 {
					// Ray cast
					{
						vertex1 := LoadV(&point11.x)
						vertex2 := LoadV(&point21.x)
						vertex3 := LoadV(&point12.x)

						alpha := IntersectRayTriangle(rayOrigin, rayTranslation, vertex1, vertex2, vertex3)
						assert(0 <= alpha && alpha <= 1.0)

						if alpha < bestFraction {
							edge1 := Sub(point21, point11)
							edge2 := Sub(point12, point11)
							normal := Cross(edge1, edge2)

							result.point = MulAdd(shapeStart, alpha, shapeTranslation)
							result.normal = Normalize(normal)
							result.fraction = alpha
							result.triangleIndex = triangleIndex1
							result.materialIndex = i32(materialIndex)
							result.hit = true
							bestFraction = alpha
						}
					}

					{
						vertex1 := LoadV(&point22.x)
						vertex2 := LoadV(&point12.x)
						vertex3 := LoadV(&point21.x)

						alpha := IntersectRayTriangle(rayOrigin, rayTranslation, vertex1, vertex2, vertex3)
						assert(0 <= alpha && alpha <= 1.0)

						if alpha < bestFraction {
							edge1 := Sub(point22, point21)
							edge2 := Sub(point12, point21)
							normal := Cross(edge1, edge2)

							result.point = MulAdd(shapeStart, alpha, shapeTranslation)
							result.normal = Normalize(normal)
							result.fraction = alpha
							result.triangleIndex = triangleIndex2
							result.materialIndex = i32(materialIndex)
							result.hit = true
							bestFraction = alpha
						}
					}
				} else {
					// Shape cast
					{
						// shapeStart is the center of the proxy.
						signedVolume := SignedVolume(point11, point21, point12, shapeStart)

						if signedVolume >= 0.0 {
							// Shift origin to first vertex
							origin := point11
							triangleVertices := [3]Vec3{Vec3_zero, Sub(point21, origin), Sub(point12, origin)}
							pairInput.proxyA = ShapeProxy{&triangleVertices[0], 3, 0.0}
							pairInput.maxFraction = bestFraction
							pairInput.transform.p = Neg(origin)

							pairOutput := ShapeCast(&pairInput)

							if pairOutput.hit {
								bestFraction = pairOutput.fraction
								result = pairOutput
								result.point = Add(result.point, origin)
								result.triangleIndex = triangleIndex1
								result.materialIndex = i32(materialIndex)
							}
						}
					}

					{
						// shapeStart is the center of the proxy.
						signedVolume := SignedVolume(point21, point22, point12, shapeStart)

						if signedVolume >= 0.0 {
							// Shift origin to first vertex
							origin := point21
							triangleVertices := [3]Vec3{Vec3_zero, Sub(point22, origin), Sub(point12, origin)}
							pairInput.proxyA = ShapeProxy{&triangleVertices[0], 3, 0.0}
							pairInput.maxFraction = bestFraction
							pairInput.transform.p = Neg(origin)

							pairOutput := ShapeCast(&pairInput)

							if pairOutput.hit {
								bestFraction = pairOutput.fraction
								result = pairOutput
								result.point = Add(result.point, origin)
								result.triangleIndex = triangleIndex2
								result.materialIndex = i32(materialIndex)
							}
						}
					}
				}
			}
		}

		// These fractions always increase to guarantee the loop eventually exits.
		// Map them from clamped-sweep space into input-translation space before
		// comparing against bestFraction.
		inputFractionX := nextFractionX == FLT_MAX ? FLT_MAX : gridFractionOffset + nextFractionX * gridFractionScale
		inputFractionZ := nextFractionZ == FLT_MAX ? FLT_MAX : gridFractionOffset + nextFractionZ * gridFractionScale
		if inputFractionX > bestFraction && inputFractionZ > bestFraction {
			break
		}

		// Advance the cast to the next column or row
		if nextFractionX <= nextFractionZ {
			if boxColumnHead == columnEnd {
				// Hit the end already
				break
			}

			// Advance to next column
			boxColumnHead += deltaColumn

			// Build a single column to cast
			boxColumnTail = boxColumnHead

			if shapeExtents.z == 0.0 {
				// Single row
				boxRowTail = boxRowHead
			} else {
				// Rasterize shape row
				rowIntercept := clampedStart.z + nextFractionX * clampedDelta.z
				boxRowTail = i32(floorf((rowIntercept - 2.0 * signZ * shapeExtents.z) / scale.z))
			}

			nextFractionX += deltaAlphaX
		} else {
			if boxRowHead == rowEnd {
				// Hit the end already
				break
			}

			// Advance to next row
			boxRowHead += deltaRow

			// Build a single row to cast
			boxRowTail = boxRowHead

			if shapeExtents.x == 0.0 {
				// Single column
				boxColumnTail = boxColumnHead
			} else {
				// Rasterize shape column
				columnIntercept := clampedStart.x + nextFractionZ * clampedDelta.x
				boxColumnTail = i32(floorf((columnIntercept - 2.0 * signX * shapeExtents.x) / scale.x))
			}

			nextFractionZ += deltaAlphaZ
		}
	}

	return result
}

OverlapHeightField :: proc(shape: ^HeightFieldData, shapeTransform: Transform, proxy: ^ShapeProxy) -> bool {
	buffer: [MAX_SHAPE_CAST_POINTS]Vec3 = ---
	localProxy := MakeLocalProxy(proxy, shapeTransform, &buffer[0])
	aabb := ComputeProxyAABB(&localProxy)

	scale := shape.scale
	minRow := i32(floorf(aabb.lowerBound.z / scale.z))
	maxRow := i32(floorf(aabb.upperBound.z / scale.z))
	minCol := i32(floorf(aabb.lowerBound.x / scale.x))
	maxCol := i32(floorf(aabb.upperBound.x / scale.x))

	boundsMin := LoadV(&aabb.lowerBound.x)
	boundsMax := LoadV(&aabb.upperBound.x)
	boundsCenter := MulV(halfV, AddV(boundsMin, boundsMax))
	boundsExtent := SubV(boundsMax, boundsCenter)

	input: DistanceInput
	input.proxyB = localProxy
	input.transform = Transform_identity
	input.useRadii = true

	cache := SimplexCache{}

	// Outer loop on rows and inner loop on columns so that triangle indices increase monotonically.
	for row := minRow; row <= maxRow; row += 1 {
		if row < 0 || shape.rowCount - 1 <= row {
			continue
		}

		for column := minCol; column <= maxCol; column += 1 {
			if column < 0 || shape.columnCount - 1 <= column {
				continue
			}

			cellIndex := row * (shape.columnCount - 1) + column
			assert(cellIndex < (shape.rowCount - 1) * (shape.columnCount - 1))
			material := GetHeightFieldMaterialIndices(shape)[cellIndex]
			if material == HEIGHT_FIELD_HOLE {
				continue
			}

			corners: [4]Vec3
			GetHeightFieldCellCorners(shape, row, column, &corners)
			point11 := corners[0]
			point12 := corners[1]
			point21 := corners[2]
			point22 := corners[3]

			v11 := LoadV(&point11.x)
			v12 := LoadV(&point12.x)
			v21 := LoadV(&point21.x)
			v22 := LoadV(&point22.x)

			if TestBoundsTriangleOverlap(boundsCenter, boundsExtent, v11, v21, v12) {
				triangleVertices := [3]Vec3{point11, point21, point12}
				input.proxyA = ShapeProxy{&triangleVertices[0], 3, 0.0}

				// reset the cache
				cache.count = 0

				// get distance between triangle and query shape
				output := ShapeDistance(&input, &cache, nil, 0)

				tolerance := 0.1 * LINEAR_SLOP
				if output.distance < tolerance {
					// overlap detected
					return true
				}
			}

			if TestBoundsTriangleOverlap(boundsCenter, boundsExtent, v21, v22, v12) {
				triangleVertices := [3]Vec3{point22, point12, point21}
				input.proxyA = ShapeProxy{&triangleVertices[0], 3, 0.0}

				// reset the cache
				cache.count = 0

				// get distance between triangle and query shape
				output := ShapeDistance(&input, &cache, nil, 0)

				tolerance := 0.1 * LINEAR_SLOP
				if output.distance < tolerance {
					// overlap detected
					return true
				}
			}
		}
	}

	return false
}

QueryHeightField :: proc(heightField: ^HeightFieldData, bounds: AABB, fcn: MeshQueryFcn, context_: rawptr) {
	scale := heightField.scale

	minRow := i32(floorf(bounds.lowerBound.z / scale.z))
	maxRow := i32(floorf(bounds.upperBound.z / scale.z))
	minCol := i32(floorf(bounds.lowerBound.x / scale.x))
	maxCol := i32(floorf(bounds.upperBound.x / scale.x))

	// Outer loop on rows and inner loop on columns so that triangle indices
	// increase monotonically.
	for row := minRow; row <= maxRow; row += 1 {
		if row < 0 || heightField.rowCount - 1 <= row {
			continue
		}

		for column := minCol; column <= maxCol; column += 1 {
			if column < 0 || heightField.columnCount - 1 <= column {
				continue
			}

			cellIndex := row * (heightField.columnCount - 1) + column
			assert(cellIndex < (heightField.rowCount - 1) * (heightField.columnCount - 1))
			material := GetHeightFieldMaterialIndices(heightField)[cellIndex]
			if material == HEIGHT_FIELD_HOLE {
				continue
			}

			corners: [4]Vec3
			GetHeightFieldCellCorners(heightField, row, column, &corners)
			point11 := corners[0]
			point12 := corners[1]
			point21 := corners[2]
			point22 := corners[3]

			// I know the min/max x and z values, but not the min/max heights.
			// This could be done with no branching in SIMD.
			cellBound: AABB
			cellBound.lowerBound = Min(Min(point11, point12), Min(point21, point22))
			cellBound.upperBound = Max(Max(point11, point12), Max(point21, point22))

			if AABB_Overlaps(bounds, cellBound) {
				quadIndex := row * (heightField.columnCount - 1) + column
				triangleIndex := 2 * quadIndex

				if heightField.clockwise != 0 {
					fcn(point11, point12, point21, triangleIndex, context_)
					fcn(point22, point21, point12, triangleIndex + 1, context_)
				} else {
					fcn(point11, point21, point12, triangleIndex, context_)
					fcn(point22, point12, point21, triangleIndex + 1, context_)
				}
			}
		}
	}
}

CollideMoverAndHeightField :: proc(
	planes: [^]PlaneResult,
	capacity: i32,
	shape: ^HeightFieldData,
	mover: ^Capsule,
) -> i32 {
	distanceInput := DistanceInput{}
	distanceInput.proxyB = ShapeProxy{&mover.center1, 2, 0.0}
	distanceInput.transform = Transform_identity
	distanceInput.useRadii = false

	cache := SimplexCache{}

	radius := mover.radius
	center := Lerp(mover.center1, mover.center2, 0.5)
	center1 := LoadV(&mover.center1.x)
	center2 := LoadV(&mover.center2.x)
	r := SplatV(radius)
	boundsMin := SubV(MinV(center1, center2), r)
	boundsMax := AddV(MaxV(center1, center2), r)
	boundsCenter := MulV(halfV, AddV(boundsMin, boundsMax))
	boundsExtent := SubV(boundsMax, boundsCenter)

	localMinX := GetXV(boundsMin)
	localMinZ := GetZV(boundsMin)
	localMaxX := GetXV(boundsMax)
	localMaxZ := GetZV(boundsMax)

	scale := shape.scale
	minRow := i32(floorf(localMinZ / scale.z))
	maxRow := i32(floorf(localMaxZ / scale.z))
	minCol := i32(floorf(localMinX / scale.x))
	maxCol := i32(floorf(localMaxX / scale.x))

	clockWise := shape.clockwise != 0

	planeCount: i32 = 0

	// Outer loop on rows and inner loop on columns so that triangle indices
	// increase monotonically.
	for row := minRow; row <= maxRow; row += 1 {
		if row < 0 || shape.rowCount - 1 <= row {
			continue
		}

		for column := minCol; column <= maxCol; column += 1 {
			if column < 0 || shape.columnCount - 1 <= column {
				continue
			}

			cellIndex := row * (shape.columnCount - 1) + column
			assert(cellIndex < (shape.rowCount - 1) * (shape.columnCount - 1))
			material := GetHeightFieldMaterialIndices(shape)[cellIndex]
			if material == HEIGHT_FIELD_HOLE {
				continue
			}

			corners: [4]Vec3
			GetHeightFieldCellCorners(shape, row, column, &corners)
			point11 := corners[0]
			point12 := corners[1]
			point21 := corners[2]
			point22 := corners[3]

			if clockWise {
				point12, point21 = point21, point12
			}

			v11 := LoadV(&point11.x)
			v12 := LoadV(&point12.x)
			v21 := LoadV(&point21.x)
			v22 := LoadV(&point22.x)

			overlap1 := TestBoundsTriangleOverlap(boundsCenter, boundsExtent, v11, v21, v12)
			if overlap1 {
				signedVolume := SignedVolume(point11, point21, point12, center)

				// Front side?
				if signedVolume >= 0.0 {
					triangleVertices := [3]Vec3{point11, point21, point12}
					distanceInput.proxyA = ShapeProxy{&triangleVertices[0], 3, 0.0}

					// reset the cache
					cache.count = 0

					// get distance between triangle and mover
					distanceOutput := ShapeDistance(&distanceInput, &cache, nil, 0)

					if distanceOutput.distance == 0.0 {
						// deep overlap
					} else if distanceOutput.distance <= mover.radius {
						triangleIndex := 2 * cellIndex
						plane := Plane{distanceOutput.normal, mover.radius - distanceOutput.distance}
						planes[planeCount] = PlaneResult{plane, distanceOutput.pointA, triangleIndex, 0, i32(material)}
						planeCount += 1

						if planeCount == capacity {
							return planeCount
						}
					}
				}
			}

			overlap2 := TestBoundsTriangleOverlap(boundsCenter, boundsExtent, v21, v22, v12)
			if overlap2 {
				signedVolume := SignedVolume(point22, point12, point21, center)

				// Front side?
				if signedVolume >= 0.0 {
					triangleVertices := [3]Vec3{point22, point12, point21}
					distanceInput.proxyA = ShapeProxy{&triangleVertices[0], 3, 0.0}

					// reset the cache
					cache.count = 0

					// get distance between triangle and mover
					distanceOutput := ShapeDistance(&distanceInput, &cache, nil, 0)

					if distanceOutput.distance == 0.0 {
						// deep overlap
					} else if distanceOutput.distance <= mover.radius {
						triangleIndex := 2 * cellIndex + 1
						plane := Plane{distanceOutput.normal, mover.radius - distanceOutput.distance}
						planes[planeCount] = PlaneResult{plane, distanceOutput.pointA, triangleIndex, 0, i32(material)}
						planeCount += 1

						if planeCount == capacity {
							return planeCount
						}
					}
				}
			}
		}
	}

	return planeCount
}

CreateGrid :: proc(rowCount: i32, columnCount: i32, scale: Vec3, makeHoles: bool) -> ^HeightFieldData {
	heightCount := rowCount * columnCount
	heights := cast([^]f32)Alloc(int(heightCount) * size_of(f32))

	for i: i32 = 0; i < rowCount; i += 1 {
		for j: i32 = 0; j < columnCount; j += 1 {
			k := i * columnCount + j
			heights[k] = 0.0
		}
	}

	cellCount := (rowCount - 1) * (columnCount - 1)
	materialIndices := cast([^]u8)Alloc(int(cellCount) * size_of(u8))

	for i: i32 = 0; i < rowCount - 1; i += 1 {
		for j: i32 = 0; j < columnCount - 1; j += 1 {
			k := i * (columnCount - 1) + j

			if makeHoles && k > 0 && k % 16 == 0 {
				materialIndices[k] = HEIGHT_FIELD_HOLE
			} else {
				materialIndices[k] = 0
			}
		}
	}

	data := HeightFieldDef{}
	data.heights = heights
	data.materialIndices = materialIndices
	data.scale = scale
	data.countX = columnCount
	data.countZ = rowCount
	data.globalMinimumHeight = -256.0
	data.globalMaximumHeight = 256.0
	data.clockwiseWinding = false

	heightField := CreateHeightField(&data)

	Free(heights, int(heightCount) * size_of(f32))
	Free(materialIndices, int(cellCount) * size_of(u8))

	return heightField
}

CreateWave :: proc(
	rowCount: i32,
	columnCount: i32,
	scale: Vec3,
	rowFrequency: f32,
	columnFrequency: f32,
	makeHoles: bool,
) -> ^HeightFieldData {
	heightCount := rowCount * columnCount
	heights := cast([^]f32)Alloc(int(heightCount) * size_of(f32))

	omegaZ := 2.0 * PI * rowFrequency
	omegaX := 2.0 * PI * columnFrequency

	cs := CosSin{}

	for i: i32 = 0; i < rowCount; i += 1 {
		cs = ComputeCosSin(omegaZ * f32(i))
		rowHeight := cs.sine

		for j: i32 = 0; j < columnCount; j += 1 {
			k := i * columnCount + j
			cs = ComputeCosSin(omegaX * f32(j))
			columnHeight := cs.sine
			heights[k] = rowHeight * columnHeight
		}
	}

	cellCount := (rowCount - 1) * (columnCount - 1)
	materialIndices := cast([^]u8)Alloc(int(cellCount) * size_of(u8))

	for i: i32 = 0; i < rowCount - 1; i += 1 {
		for j: i32 = 0; j < columnCount - 1; j += 1 {
			k := i * (columnCount - 1) + j

			if makeHoles && k > 0 && k % 16 == 0 {
				materialIndices[k] = HEIGHT_FIELD_HOLE
			} else {
				materialIndices[k] = 0
			}
		}
	}

	data := HeightFieldDef{}
	data.heights = heights
	data.materialIndices = materialIndices
	data.scale = scale
	data.countX = columnCount
	data.countZ = rowCount
	data.globalMinimumHeight = -256.0
	data.globalMaximumHeight = 256.0
	data.clockwiseWinding = false

	heightField := CreateHeightField(&data)

	Free(heights, int(heightCount) * size_of(f32))
	Free(materialIndices, int(cellCount) * size_of(u8))

	return heightField
}

DestroyHeightField :: proc(heightField: ^HeightFieldData) {
	Free(heightField, int(heightField.byteCount))
}

// unified: core:os and core:fmt instead of stdio (fopen/fprintf/fclose); %.9f prints the same digits
DumpHeightData :: proc(data: ^HeightFieldDef, fileName: cstring) {
	file, err := os.create(string(fileName))
	if err != nil {
		return
	}

	fmt.fprintf(file, "%d %d\n", data.countX, data.countZ)
	fmt.fprintf(file, "%.9f %.9f %.9f\n", data.scale.x, data.scale.y, data.scale.z)
	fmt.fprintf(file, "%.9f %.9f\n", data.globalMinimumHeight, data.globalMaximumHeight)
	fmt.fprintf(file, "%d\n", data.clockwiseWinding ? 1 : 0)

	heightCount := data.countX * data.countZ
	for i: i32 = 0; i < heightCount; i += 1 {
		fmt.fprintf(file, "%.9f\n", data.heights[i])
	}

	materialCount := (data.countX - 1) * (data.countZ - 1)
	for i: i32 = 0; i < materialCount; i += 1 {
		fmt.fprintf(file, "%d\n", data.materialIndices[i])
	}

	os.close(file)
}
// unified end

// unified: B3_FILE_SCAN (fscanf) is a whitespace-separated token scanner over the whole file read with core:os;
// numbers are parsed with core:strconv (correctly rounded, as strtof)
@(private = "file")
HeightFileScanner :: struct {
	text: []u8,
	pos:  int,
}

@(private = "file")
ScanToken :: proc(s: ^HeightFileScanner) -> string {
	for s.pos < len(s.text) &&
	    (s.text[s.pos] == ' ' ||
			    s.text[s.pos] == '\t' ||
			    s.text[s.pos] == '\n' ||
			    s.text[s.pos] == '\r' ||
			    s.text[s.pos] == '\v' ||
			    s.text[s.pos] == '\f') {
		s.pos += 1
	}

	start := s.pos
	for s.pos < len(s.text) &&
	    !(s.text[s.pos] == ' ' ||
			    s.text[s.pos] == '\t' ||
			    s.text[s.pos] == '\n' ||
			    s.text[s.pos] == '\r' ||
			    s.text[s.pos] == '\v' ||
			    s.text[s.pos] == '\f') {
		s.pos += 1
	}

	return string(s.text[start:s.pos])
}

@(private = "file")
ScanInt :: proc(s: ^HeightFileScanner, value: ^i32) -> bool {
	token := ScanToken(s)
	v, ok := strconv.parse_int(token, 10)
	if !ok {
		return false
	}
	value^ = i32(v)
	return true
}

@(private = "file")
ScanFloat :: proc(s: ^HeightFileScanner, value: ^f32) -> bool {
	token := ScanToken(s)
	v, ok := strconv.parse_f32(token)
	if !ok {
		return false
	}
	value^ = v
	return true
}
// unified end

LoadHeightField :: proc(fileName: cstring) -> ^HeightFieldData {
	// unified: the file is read whole with core:os, then scanned (see HeightFileScanner)
	text, err := os.read_entire_file(string(fileName), runtime.heap_allocator())
	if err != nil {
		return nil
	}
	defer delete(text, runtime.heap_allocator())
	scanner := HeightFileScanner{text, 0}
	file := &scanner
	// unified end

	data := HeightFieldDef{}

	// Read dimensions
	if !(ScanInt(file, &data.countX) && ScanInt(file, &data.countZ)) {
		return nil
	}

	// Read scale
	if !(ScanFloat(file, &data.scale.x) && ScanFloat(file, &data.scale.y) && ScanFloat(file, &data.scale.z)) {
		return nil
	}

	// Read global height bounds
	if !(ScanFloat(file, &data.globalMinimumHeight) && ScanFloat(file, &data.globalMaximumHeight)) {
		return nil
	}

	// Read clockwise winding
	clockwise: i32
	if !ScanInt(file, &clockwise) {
		return nil
	}
	data.clockwiseWinding = clockwise != 0

	// Allocate and read height data
	heightCount := data.countX * data.countZ
	data.heights = cast([^]f32)Alloc(int(heightCount) * size_of(f32))

	for i: i32 = 0; i < heightCount; i += 1 {
		if !ScanFloat(file, &data.heights[i]) {
			Free(data.heights, int(heightCount) * size_of(f32))
			return nil
		}
	}

	// Allocate and read material indices
	materialCount := (data.countX - 1) * (data.countZ - 1)
	data.materialIndices = cast([^]u8)Alloc(int(materialCount) * size_of(u8))

	for i: i32 = 0; i < materialCount; i += 1 {
		materialIndex: i32
		if !ScanInt(file, &materialIndex) {
			Free(data.heights, int(heightCount) * size_of(f32))
			Free(data.materialIndices, int(materialCount) * size_of(u8))
			return nil
		}
		data.materialIndices[i] = u8(materialIndex)
	}

	// Create height field from loaded data
	heightField := CreateHeightField(&data)

	// Clean up temporary allocations
	Free(data.heights, int(heightCount) * size_of(f32))
	Free(data.materialIndices, int(materialCount) * size_of(u8))

	return heightField
}
