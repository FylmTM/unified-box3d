// Ported from box3d test/test_mesh.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "core:testing"

// Two quads meeting at a concave crease along the shared edge 1-4. The crease exercises
// edge identification, which reads the baked winding rather than the input winding.
@(private = "file")
VALLEY_VERTEX_COUNT :: 6
@(private = "file")
VALLEY_TRIANGLE_COUNT :: 4

@(private = "file")
MakeValley :: proc(vertices: ^[VALLEY_VERTEX_COUNT]b3.Vec3, indices: ^[3 * VALLEY_TRIANGLE_COUNT]i32) {
	v := [VALLEY_VERTEX_COUNT]b3.Vec3 {
		{-1.0, 1.0, -1.0},
		{0.0, 0.0, -1.0},
		{1.0, 1.0, -1.0},
		{-1.0, 1.0, 1.0},
		{0.0, 0.0, 1.0},
		{1.0, 1.0, 1.0},
	}

	i := [3 * VALLEY_TRIANGLE_COUNT]i32 {
		0,
		3,
		1, // left quad
		3,
		4,
		1,
		1,
		4,
		2, // right quad
		4,
		5,
		2,
	}

	vertices^ = v
	indices^ = i
}

// A vertex laid out the way a renderer would hand it over. The position sits off the
// front of the struct so a non-zero base offset gets exercised too.
@(private = "file")
FatVertex :: struct {
	weight:   f32,
	position: b3.Vec3,
	uv:       [2]f32,
}

@(private = "file")
MakeFatVertices :: proc(fat: [^]FatVertex, vertices: [^]b3.Vec3, count: i32) {
	for i: i32 = 0; i < count; i += 1 {
		// Poison the padding so a stride mistake bakes obvious garbage instead of near misses
		fat[i].weight = 1000.0 + f32(i)
		fat[i].position = vertices[i]
		fat[i].uv[0] = -1000.0
		fat[i].uv[1] = -2000.0
	}
}

@(private = "file")
ReverseWinding :: proc(indices: [^]i32, triangleCount: i32) {
	for i: i32 = 0; i < triangleCount; i += 1 {
		temp := indices[3 * i + 1]
		indices[3 * i + 1] = indices[3 * i + 2]
		indices[3 * i + 2] = temp
	}
}

// The hash covers every byte of the mesh block, so this compares the tree, the vertices,
// the baked triangles and the edge flags in one shot.
@(private = "file")
MeshesMatch :: proc(mesh1: ^b3.MeshData, mesh2: ^b3.MeshData) -> bool {
	return mesh1.byteCount == mesh2.byteCount && mesh1.hash == mesh2.hash
}

@(private = "file")
VerticesMatch :: proc(mesh: ^b3.MeshData, expected: [^]b3.Vec3, count: i32) -> bool {
	if mesh.vertexCount != count {
		return false
	}

	vertices := b3.GetMeshVertices(mesh)
	for i: i32 = 0; i < count; i += 1 {
		if b3.DistanceSquared(vertices[i], expected[i]) > b3.FLT_EPSILON {
			return false
		}
	}

	return true
}

// The valley faces up, so every baked triangle should wind counter clockwise about +Y
@(private = "file")
FacesUp :: proc(mesh: ^b3.MeshData) -> bool {
	vertices := b3.GetMeshVertices(mesh)
	triangles := b3.GetMeshTriangles(mesh)

	for i: i32 = 0; i < mesh.triangleCount; i += 1 {
		v1 := vertices[triangles[i].index1]
		v2 := vertices[triangles[i].index2]
		v3 := vertices[triangles[i].index3]

		normal := b3.Cross(b3.Sub(v2, v1), b3.Sub(v3, v1))
		if normal.y <= 0.0 {
			return false
		}
	}

	return true
}

@(private = "file")
HasConcaveEdge :: proc(mesh: ^b3.MeshData) -> bool {
	flags := b3.GetMeshFlags(mesh)

	for i: i32 = 0; i < mesh.triangleCount; i += 1 {
		if (flags[i] & b3.allConcaveEdges) != 0 {
			return true
		}
	}

	return false
}

@(private = "file")
MakeValleyDef :: proc(vertices: [^]b3.Vec3, indices: [^]i32) -> b3.MeshDef {
	def := b3.MeshDef{}
	def.vertices = vertices
	def.stride = size_of(b3.Vec3)
	def.indices = indices
	def.vertexCount = VALLEY_VERTEX_COUNT
	def.triangleCount = VALLEY_TRIANGLE_COUNT
	def.identifyEdges = true
	return def
}

// Dense input is the reference the other cases are measured against
@(private = "file")
MeshDenseStride :: proc(t: ^testing.T) -> bool {
	vertices: [VALLEY_VERTEX_COUNT]b3.Vec3
	indices: [3 * VALLEY_TRIANGLE_COUNT]i32
	MakeValley(&vertices, &indices)

	def := MakeValleyDef(&vertices[0], &indices[0])
	mesh := b3.CreateMesh(&def, nil, 0)
	ENSURE(t, mesh != nil) or_return

	ENSURE(t, mesh.vertexCount == VALLEY_VERTEX_COUNT) or_return
	ENSURE(t, mesh.triangleCount == VALLEY_TRIANGLE_COUNT) or_return
	ENSURE(t, mesh.degenerateCount == 0) or_return
	ENSURE(t, VerticesMatch(mesh, &vertices[0], VALLEY_VERTEX_COUNT)) or_return
	ENSURE(t, FacesUp(mesh)) or_return
	ENSURE(t, HasConcaveEdge(mesh)) or_return

	b3.DestroyMesh(mesh)
	return true
}

// Interleaved input must bake to exactly the same mesh as dense input
@(private = "file")
MeshFatStride :: proc(t: ^testing.T) -> bool {
	vertices: [VALLEY_VERTEX_COUNT]b3.Vec3
	indices: [3 * VALLEY_TRIANGLE_COUNT]i32
	MakeValley(&vertices, &indices)

	denseDef := MakeValleyDef(&vertices[0], &indices[0])
	denseMesh := b3.CreateMesh(&denseDef, nil, 0)
	ENSURE(t, denseMesh != nil) or_return

	fat: [VALLEY_VERTEX_COUNT]FatVertex
	MakeFatVertices(&fat[0], &vertices[0], VALLEY_VERTEX_COUNT)

	fatDef := MakeValleyDef(&fat[0].position, &indices[0])
	fatDef.stride = size_of(FatVertex)
	fatMesh := b3.CreateMesh(&fatDef, nil, 0)
	ENSURE(t, fatMesh != nil) or_return

	ENSURE(t, VerticesMatch(fatMesh, &vertices[0], VALLEY_VERTEX_COUNT)) or_return
	ENSURE(t, FacesUp(fatMesh)) or_return
	ENSURE(t, MeshesMatch(denseMesh, fatMesh)) or_return

	b3.DestroyMesh(fatMesh)
	b3.DestroyMesh(denseMesh)
	return true
}

// Welding reads the source vertices through the stride as well
@(private = "file")
MeshStrideWeld :: proc(t: ^testing.T) -> bool {
	vertices: [VALLEY_VERTEX_COUNT]b3.Vec3
	indices: [3 * VALLEY_TRIANGLE_COUNT]i32
	MakeValley(&vertices, &indices)

	// Split the crease so each quad owns a copy of the shared edge
	splitVertices: [8]b3.Vec3
	copy(splitVertices[:], vertices[:])
	splitVertices[6] = vertices[1]
	splitVertices[7] = vertices[4]

	// Point the right quad at the copies
	splitIndices: [3 * VALLEY_TRIANGLE_COUNT]i32 = indices
	splitIndices[6] = 6
	splitIndices[7] = 7
	splitIndices[9] = 7

	fat: [8]FatVertex
	MakeFatVertices(&fat[0], &splitVertices[0], 8)

	def := MakeValleyDef(&fat[0].position, &splitIndices[0])
	def.stride = size_of(FatVertex)
	def.vertexCount = 8
	def.weldVertices = true
	def.weldTolerance = 0.01

	mesh := b3.CreateMesh(&def, nil, 0)
	ENSURE(t, mesh != nil) or_return

	ENSURE(t, mesh.vertexCount == VALLEY_VERTEX_COUNT) or_return
	ENSURE(t, mesh.triangleCount == VALLEY_TRIANGLE_COUNT) or_return
	ENSURE(t, VerticesMatch(mesh, &vertices[0], VALLEY_VERTEX_COUNT)) or_return
	ENSURE(t, FacesUp(mesh)) or_return
	ENSURE(t, HasConcaveEdge(mesh)) or_return

	b3.DestroyMesh(mesh)
	return true
}

// Clockwise input plus the flag must bake to the same mesh as counter clockwise input
@(private = "file")
MeshClockWise :: proc(t: ^testing.T) -> bool {
	vertices: [VALLEY_VERTEX_COUNT]b3.Vec3
	indices: [3 * VALLEY_TRIANGLE_COUNT]i32
	MakeValley(&vertices, &indices)

	ccwDef := MakeValleyDef(&vertices[0], &indices[0])
	ccwMesh := b3.CreateMesh(&ccwDef, nil, 0)
	ENSURE(t, ccwMesh != nil) or_return

	reversed: [3 * VALLEY_TRIANGLE_COUNT]i32 = indices
	ReverseWinding(&reversed[0], VALLEY_TRIANGLE_COUNT)

	cwDef := MakeValleyDef(&vertices[0], &reversed[0])
	cwDef.clockWiseWinding = true
	cwMesh := b3.CreateMesh(&cwDef, nil, 0)
	ENSURE(t, cwMesh != nil) or_return

	ENSURE(t, FacesUp(cwMesh)) or_return
	ENSURE(t, HasConcaveEdge(cwMesh)) or_return
	ENSURE(t, MeshesMatch(ccwMesh, cwMesh)) or_return

	b3.DestroyMesh(cwMesh)
	b3.DestroyMesh(ccwMesh)
	return true
}

// Without the flag the same clockwise input must bake inside out
@(private = "file")
MeshClockWiseIgnored :: proc(t: ^testing.T) -> bool {
	vertices: [VALLEY_VERTEX_COUNT]b3.Vec3
	indices: [3 * VALLEY_TRIANGLE_COUNT]i32
	MakeValley(&vertices, &indices)
	ReverseWinding(&indices[0], VALLEY_TRIANGLE_COUNT)

	def := MakeValleyDef(&vertices[0], &indices[0])
	mesh := b3.CreateMesh(&def, nil, 0)
	ENSURE(t, mesh != nil) or_return

	ENSURE(t, FacesUp(mesh) == false) or_return

	b3.DestroyMesh(mesh)
	return true
}

// Winding, stride and welding have to compose
@(private = "file")
MeshClockWiseStrideWeld :: proc(t: ^testing.T) -> bool {
	vertices: [VALLEY_VERTEX_COUNT]b3.Vec3
	indices: [3 * VALLEY_TRIANGLE_COUNT]i32
	MakeValley(&vertices, &indices)

	ccwDef := MakeValleyDef(&vertices[0], &indices[0])
	ccwDef.weldVertices = true
	ccwDef.weldTolerance = 0.01
	ccwMesh := b3.CreateMesh(&ccwDef, nil, 0)
	ENSURE(t, ccwMesh != nil) or_return

	reversed: [3 * VALLEY_TRIANGLE_COUNT]i32 = indices
	ReverseWinding(&reversed[0], VALLEY_TRIANGLE_COUNT)

	fat: [VALLEY_VERTEX_COUNT]FatVertex
	MakeFatVertices(&fat[0], &vertices[0], VALLEY_VERTEX_COUNT)

	cwDef := MakeValleyDef(&fat[0].position, &reversed[0])
	cwDef.stride = size_of(FatVertex)
	cwDef.clockWiseWinding = true
	cwDef.weldVertices = true
	cwDef.weldTolerance = 0.01
	cwMesh := b3.CreateMesh(&cwDef, nil, 0)
	ENSURE(t, cwMesh != nil) or_return

	ENSURE(t, FacesUp(cwMesh)) or_return
	ENSURE(t, MeshesMatch(ccwMesh, cwMesh)) or_return

	b3.DestroyMesh(cwMesh)
	b3.DestroyMesh(ccwMesh)
	return true
}

// The built in creators fill their own def, so a missed stride shows up here
@(private = "file")
MeshCreators :: proc(t: ^testing.T) -> bool {
	center := b3.Vec3{1.0, 2.0, 3.0}
	extent := b3.Vec3{0.5, 1.0, 1.5}

	meshes := [?]^b3.MeshData {
		b3.CreateGridMesh(4, 4, 1.0, 2, true),
		b3.CreateWaveMesh(4, 4, 1.0, 0.5, 1.0, 1.0),
		b3.CreateTorusMesh(8, 6, 2.0, 0.5),
		b3.CreateBoxMesh(center, extent, true),
		b3.CreateHollowBoxMesh(center, extent),
		b3.CreatePlatformMesh(center, 2.0, 1.0, 2.0),
	}

	for i: i32 = 0; i < i32(len(meshes)); i += 1 {
		ENSURE(t, meshes[i] != nil) or_return
		ENSURE(t, meshes[i].vertexCount >= 3) or_return
		ENSURE(t, meshes[i].triangleCount >= 1) or_return
		ENSURE(t, b3.IsSaneAABB(meshes[i].bounds)) or_return

		b3.DestroyMesh(meshes[i])
	}

	return true
}

@(test)
MeshTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, MeshDenseStride) {return}
	if !RUN_SUBTEST(t, MeshFatStride) {return}
	if !RUN_SUBTEST(t, MeshStrideWeld) {return}
	if !RUN_SUBTEST(t, MeshClockWise) {return}
	if !RUN_SUBTEST(t, MeshClockWiseIgnored) {return}
	if !RUN_SUBTEST(t, MeshClockWiseStrideWeld) {return}
	if !RUN_SUBTEST(t, MeshCreators) {return}
}
