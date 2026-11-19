// unified: what World_LoadImage checks before a loaded world is used (SPEC.md, Images from anyone). An image can
// come from anyone (a late joiner's snapshot from a peer), so the worst its bytes may do is fail to load:
// - every geometry blob the image carries (hull, mesh, height field, compound with its embedded hulls and meshes):
//   offsets and counts inside the blob, every index in range, trees acyclic and shallow enough for the fixed
//   traversal stacks, floats finite (ValidGeometryBlob, from ImageStoreLoad);
// - the world (ValidLoadedWorld): every id in range and live, every back-link (body <-> set <-> sims, shapes,
//   contacts and joints <-> their edges, islands, graph colors, sensors, broad-phase proxies), every list walk
//   bounded, the broad-phase trees, the id pools, the pair set, enums, and the floats that become positions and
//   boxes finite.
// Nothing here asserts or follows an index it has not checked. The rules are the invariants upstream's
// ValidateSolverSets, ValidateContacts, ValidateConnectivity and DynamicTree_Validate assert (physics_world.odin,
// dynamic_tree.odin), made total.
package box3d

@(private = "file")
MESH_DEPTH_MAX :: 60 // the shallowest mesh traversal stack holds 64 nodes

// A float a world may hold: finite, and below 1e18 (impulses, masses, inertias; what creation would accept).
@(private = "file")
Finite :: #force_inline proc "contextless" (x: f32) -> bool {
	return x - x == 0 && x <= 1e18 && x >= -1e18 // NaN and ±inf give NaN
}

// A coordinate: within HUGE of the origin (Box3D's own bound for positions and boxes, IsSaneAABB).
@(private = "file")
Coord :: #force_inline proc(x: f32) -> bool {
	return x - x == 0 && x <= HUGE && x >= -HUGE
}

@(private = "file")
Position :: #force_inline proc(v: Vec3) -> bool {
	return Coord(v.x) && Coord(v.y) && Coord(v.z)
}

@(private = "file")
Box :: #force_inline proc(box: AABB) -> bool {
	return(
		Position(box.lowerBound) &&
		Position(box.upperBound) &&
		box.lowerBound.x <= box.upperBound.x &&
		box.lowerBound.y <= box.upperBound.y &&
		box.lowerBound.z <= box.upperBound.z \
	)
}

// A rotation: a unit quaternion, as creation asserts (IsValidQuat).
@(private = "file")
Rotation :: #force_inline proc(q: Quat) -> bool {
	return Finite(q.v.x) && Finite(q.v.y) && Finite(q.v.z) && Finite(q.s) && IsNormalizedQuat(q)
}

@(private = "file")
Frame :: #force_inline proc(t: Transform) -> bool {
	return Position(t.p) && Rotation(t.q)
}

// A length: a radius or an extent, from 0 to HUGE.
@(private = "file")
Length :: #force_inline proc(x: f32) -> bool {
	return x >= 0 && x <= HUGE
}

@(private = "file")
FiniteVec :: #force_inline proc "contextless" (v: Vec3) -> bool {
	return Finite(v.x) && Finite(v.y) && Finite(v.z)
}

@(private = "file")
FiniteFloats :: proc "contextless" (p: rawptr, count: int) -> bool {
	floats := ([^]f32)(p)
	for i in 0 ..< count {
		if !Finite(floats[i]) {
			return false
		}
	}
	return true
}

@(private = "file")
FiniteOf :: #force_inline proc "contextless" (value: ^$T) -> bool {
	#assert(size_of(T) % 4 == 0)
	return FiniteFloats(value, size_of(T) / 4)
}

@(private = "file")
Positions :: proc(p: rawptr, count: int) -> bool {
	floats := ([^]f32)(p)
	for i in 0 ..< count {
		if !Coord(floats[i]) {
			return false
		}
	}
	return true
}

@(private = "file")
ValidSphere :: proc(sphere: Sphere) -> bool {
	return Position(sphere.center) && Length(sphere.radius)
}

@(private = "file")
ValidCapsule :: proc(capsule: Capsule) -> bool {
	return Position(capsule.center1) && Position(capsule.center2) && Length(capsule.radius)
}

@(private = "file")
In :: #force_inline proc "contextless" (index, count: i32) -> bool {
	return 0 <= index && index < count
}

// An array of `count` elements of `size` bytes at `offset` lies inside a blob of `byteCount` bytes, after its header,
// aligned for its elements.
@(private = "file")
Fits :: proc "contextless" (offset, count, size: i64, header, byteCount: i64) -> bool {
	align := min(size, 4)
	return offset >= header && offset % align == 0 && count >= 0 && offset + count * size <= byteCount
}

@(private = "file")
At :: #force_inline proc "contextless" (blob: rawptr, offset: i32) -> rawptr {
	return rawptr(uintptr(blob) + uintptr(offset))
}

// ---- geometry blobs ---------------------------------------------------------------------------------------------

// A geometry blob of `kind`, `byteCount` bytes, safe to use (ImageStoreLoad, after its version, size and hash).
ValidGeometryBlob :: proc(kind: GeometryKind, bytes: [^]u8, byteCount: i32) -> bool {
	switch kind {
	case .geometryHull:
		return ValidHullBlob((^HullData)(bytes), i64(byteCount)) && (^HullData)(bytes).byteCount == byteCount
	case .geometryMesh:
		return ValidMeshBlob((^MeshData)(bytes), i64(byteCount)) && (^MeshData)(bytes).byteCount == byteCount
	case .geometryHeightField:
		return ValidHeightFieldBlob((^HeightFieldData)(bytes), i64(byteCount))
	case .geometryCompound:
		return ValidCompoundBlob((^CompoundData)(bytes), i64(byteCount))
	}
	return false
}

// A hull blob within `limit` bytes: its arrays inside it, its half-edge mesh closed and consistent.
@(private = "file")
ValidHullBlob :: proc(hull: ^HullData, limit: i64) -> bool {
	if limit < size_of(HullData) || hull.version != HULL_VERSION {
		return false
	}
	n := i64(hull.byteCount)
	header := i64(size_of(HullData))
	if n < header || n > limit {
		return false
	}
	v, e, f := hull.vertexCount, hull.edgeCount, hull.faceCount
	if v < 4 ||
	   v > MAX_HULL_VERTICES ||
	   f < 4 ||
	   f > MAX_HULL_FACES ||
	   e < 12 ||
	   e > 2 * MAX_HULL_EDGES ||
	   e & 1 != 0 {
		return false
	}
	if v - e / 2 + f != 2 {
		return false
	}
	soaV, soaF := i64(GetHullSoaStride(v)), i64(GetHullSoaStride(f))
	if !Fits(i64(hull.vertexOffset), i64(v), size_of(HullVertex), header, n) ||
	   !Fits(i64(hull.pointOffset), i64(v), size_of(Vec3), header, n) ||
	   !Fits(i64(hull.edgeOffset), i64(e), size_of(HullHalfEdge), header, n) ||
	   !Fits(i64(hull.planeOffset), i64(f), size_of(Plane), header, n) ||
	   !Fits(i64(hull.faceOffset), i64(f), size_of(HullFace), header, n) ||
	   !Fits(i64(hull.soaVertexOffset), 3 * soaV, size_of(f32), header, n) ||
	   !Fits(i64(hull.soaNormalOffset), 3 * soaF, size_of(f32), header, n) ||
	   !Fits(i64(hull.edgeCosineOffset), i64(e / 2), size_of(f32), header, n) {
		return false
	}
	if !Box(hull.aabb) ||
	   !Position(hull.center) ||
	   !FiniteOf(&hull.centralInertia) ||
	   !Finite(hull.surfaceArea) ||
	   !Finite(hull.volume) ||
	   !Finite(hull.innerRadius) ||
	   !Positions(At(hull, hull.pointOffset), 3 * int(v)) ||
	   !FiniteFloats(At(hull, hull.planeOffset), 4 * int(f)) ||
	   !Positions(At(hull, hull.soaVertexOffset), 3 * int(soaV)) ||
	   !FiniteFloats(At(hull, hull.soaNormalOffset), 3 * int(soaF)) ||
	   !FiniteFloats(At(hull, hull.edgeCosineOffset), int(e / 2)) {
		return false
	}
	vertices := ([^]HullVertex)(At(hull, hull.vertexOffset))
	edges := ([^]HullHalfEdge)(At(hull, hull.edgeOffset))
	faces := ([^]HullFace)(At(hull, hull.faceOffset))
	for i in 0 ..< e {
		edge := edges[i]
		if i32(edge.next) >= e || i32(edge.twin) >= e || i32(edge.origin) >= v || i32(edge.face) >= f {
			return false
		}
		if i32(edge.twin) != i ~ 1 {
			return false
		}
		if edges[edge.next].origin != edges[edge.twin].origin {
			return false
		}
	}
	for i in 0 ..< v {
		if i32(vertices[i].edge) >= e || i32(edges[vertices[i].edge].origin) != i {
			return false
		}
	}
	for i in 0 ..< f {
		start := i32(faces[i].edge)
		if start >= e {
			return false
		}
		edge := start
		for steps: i32 = 0;; steps += 1 {
			if steps >= e || i32(edges[edge].face) != i {
				return false
			}
			edge = i32(edges[edge].next)
			if edge == start {
				break
			}
		}
	}
	return true
}

// A mesh blob within `limit` bytes: its arrays inside it, every triangle's vertices in range, its BVH a tree of
// leaves over the triangles, no deeper than the traversal stacks.
@(private = "file")
ValidMeshBlob :: proc(mesh: ^MeshData, limit: i64) -> bool {
	if limit < size_of(MeshData) || mesh.version != MESH_VERSION {
		return false
	}
	n := i64(mesh.byteCount)
	header := i64(size_of(MeshData))
	if n < header || n > limit {
		return false
	}
	nodes, vertices, triangles := mesh.nodeCount, mesh.vertexCount, mesh.triangleCount
	if nodes < 1 || vertices < 3 || triangles < 1 || mesh.treeHeight < 0 || mesh.treeHeight > MESH_DEPTH_MAX {
		return false
	}
	if !Fits(i64(mesh.nodeOffset), i64(nodes), size_of(MeshNode), header, n) ||
	   !Fits(i64(mesh.vertexOffset), i64(vertices), size_of(Vec3), header, n) ||
	   !Fits(i64(mesh.triangleOffset), i64(triangles), size_of(MeshTriangle), header, n) ||
	   !Fits(i64(mesh.materialOffset), i64(triangles), 1, header, n) ||
	   !Fits(i64(mesh.flagsOffset), i64(triangles), 1, header, n) {
		return false
	}
	if !Box(mesh.bounds) || !Finite(mesh.surfaceArea) || !Positions(At(mesh, mesh.vertexOffset), 3 * int(vertices)) {
		return false
	}
	tris := ([^]MeshTriangle)(At(mesh, mesh.triangleOffset))
	for i in 0 ..< triangles {
		t := tris[i]
		if !In(t.index1, vertices) || !In(t.index2, vertices) || !In(t.index3, vertices) {
			return false
		}
	}
	// The BVH: node i's first child is i + 1 and its second i + childOffset, so a walk from the root only goes forward.
	// Each node is reached once.
	meshNodes := ([^]MeshNode)(At(mesh, mesh.nodeOffset))
	Item :: struct {
		node, depth: i32,
	}
	stack := make([dynamic]Item, 0, 64)
	defer delete(stack)
	append(&stack, Item{0, 1})
	visited: i32 = 0
	for len(stack) > 0 {
		item := pop(&stack)
		visited += 1
		if visited > nodes || item.depth > MESH_DEPTH_MAX {
			return false
		}
		node := &meshNodes[item.node]
		if !Position(node.lowerBound) || !Position(node.upperBound) {
			return false
		}
		if node.data.asLeaf.type == 3 {
			count := i64(node.data.asLeaf.triangleCount)
			if count < 1 || i64(node.triangleOffset) + count > i64(triangles) {
				return false
			}
			continue
		}
		right := i64(item.node) + i64(node.data.asNode.childOffset)
		if item.node + 1 >= nodes || right <= i64(item.node) + 1 || right >= i64(nodes) {
			return false
		}
		append(&stack, Item{item.node + 1, item.depth + 1}, Item{i32(right), item.depth + 1})
	}
	return visited == nodes
}

// A height field blob within `limit` bytes: its grid, materials and flags inside it, its scales finite and positive.
@(private = "file")
ValidHeightFieldBlob :: proc(hf: ^HeightFieldData, limit: i64) -> bool {
	if limit < size_of(HeightFieldData) || hf.version != HEIGHT_FIELD_VERSION {
		return false
	}
	n := i64(hf.byteCount)
	header := i64(size_of(HeightFieldData))
	if n < header || n > limit || hf.columnCount < 2 || hf.rowCount < 2 {
		return false
	}
	points := i64(hf.columnCount) * i64(hf.rowCount)
	cells := i64(hf.columnCount - 1) * i64(hf.rowCount - 1)
	if points > 1 << 28 ||
	   !Fits(i64(hf.heightsOffset), points, size_of(u16), header, n) ||
	   !Fits(i64(hf.materialOffset), cells, 1, header, n) ||
	   !Fits(i64(hf.flagsOffset), 2 * cells, 1, header, n) {
		return false
	}
	ok :=
		Box(hf.aabb) &&
		Position(hf.scale) &&
		hf.scale.x > 0 &&
		hf.scale.y > 0 &&
		hf.scale.z > 0 &&
		Finite(hf.minHeight) &&
		Finite(hf.maxHeight) &&
		Finite(hf.heightScale)
	return ok
}

// A compound blob within `limit` bytes: its tree over its children, every child inside it (hulls and meshes are
// blobs of their own, checked as such), and every material index in range of the compound's materials.
@(private = "file")
ValidCompoundBlob :: proc(compound: ^CompoundData, limit: i64) -> bool {
	if limit < size_of(CompoundData) || compound.version != COMPOUND_VERSION {
		return false
	}
	n := i64(compound.byteCount)
	header := i64(size_of(CompoundData))
	if n < header || n > limit || !Box(compound.bounds) {
		return false
	}
	capsules, hulls, meshes, spheres :=
		compound.capsuleCount, compound.hullCount, compound.meshCount, compound.sphereCount
	if capsules < 0 || hulls < 0 || meshes < 0 || spheres < 0 || compound.materialCount < 0 {
		return false
	}
	children := i64(capsules) + i64(hulls) + i64(meshes) + i64(spheres)
	if children < 1 || children >= MAX_CHILD_SHAPES {
		return false
	}
	// An array with elements lies inside. An empty one is never read: CreateCompound (as upstream's) gives it the
	// running offset, where it would start, and an offset of 0 (the getters return nil) is accepted too.
	present :: proc(offset, count: i32, size: i64, header, n: i64) -> bool {
		return Fits(i64(offset), i64(count), size, header, n) || (count == 0 && offset == 0)
	}
	if !present(compound.materialOffset, compound.materialCount, size_of(SurfaceMaterial), header, n) ||
	   !present(compound.capsuleOffset, capsules, size_of(CompoundCapsule), header, n) ||
	   !present(compound.hullOffset, hulls, size_of(HullInstance), header, n) ||
	   !present(compound.meshOffset, meshes, size_of(MeshInstance), header, n) ||
	   !present(compound.sphereOffset, spheres, size_of(CompoundSphere), header, n) ||
	   !Fits(i64(compound.nodeOffset), i64(compound.nodeCount), size_of(TreeNode), header, n) {
		return false
	}
	materials := i64(max(compound.materialCount, 1))
	if capsules > 0 {
		list := ([^]CompoundCapsule)(At(compound, compound.capsuleOffset))
		for i in 0 ..< capsules {
			if i64(list[i].materialIndex) >= materials || !ValidCapsule(list[i].capsule) {
				return false
			}
		}
	}
	if spheres > 0 {
		list := ([^]CompoundSphere)(At(compound, compound.sphereOffset))
		for i in 0 ..< spheres {
			if i64(list[i].materialIndex) >= materials || !ValidSphere(list[i].sphere) {
				return false
			}
		}
	}
	if hulls > 0 {
		list := ([^]HullInstance)(At(compound, compound.hullOffset))
		first := i64(compound.hullOffset) + i64(hulls) * size_of(HullInstance)
		for i in 0 ..< hulls {
			offset := i64(list[i].hullOffset)
			if offset < first ||
			   offset % 8 != 0 ||
			   i64(list[i].materialIndex) >= materials ||
			   !Frame(list[i].transform) ||
			   !ValidHullBlob((^HullData)(At(compound, i32(offset))), n - offset) {
				return false
			}
		}
	}
	if meshes > 0 {
		list := ([^]MeshInstance)(At(compound, compound.meshOffset))
		first := i64(compound.meshOffset) + i64(meshes) * size_of(MeshInstance)
		for i in 0 ..< meshes {
			instance := &list[i]
			offset := i64(instance.meshOffset)
			if offset < first ||
			   offset % 8 != 0 ||
			   !Frame(instance.transform) ||
			   !Position(instance.scale) ||
			   !ValidMeshBlob((^MeshData)(At(compound, i32(offset))), n - offset) {
				return false
			}
			// The mesh's material map: one u16 per material the mesh's triangles name, each a compound material.
			mapCount := i64(instance.materialCount)
			if mapCount < 1 || !Fits(i64(instance.materialOffset), mapCount, size_of(u16), header, n) {
				return false
			}
			materialMap := ([^]u16)(At(compound, i32(instance.materialOffset)))
			for k in 0 ..< mapCount {
				if i64(materialMap[k]) >= materials {
					return false
				}
			}
			mesh := (^MeshData)(At(compound, i32(offset)))
			triangleMaterials := ([^]u8)(At(mesh, mesh.materialOffset))
			for t in 0 ..< mesh.triangleCount {
				if i64(triangleMaterials[t]) >= mapCount {
					return false
				}
			}
		}
	}
	// The tree: leaves name children.
	childCount := i32(children)
	return ValidTreeNodes(
		([^]TreeNode)(At(compound, compound.nodeOffset)),
		compound.nodeCount,
		nil,
		proc(leaf: ^TreeNode, data: rawptr) -> bool {return In(leaf.shapeIndex, (^i32)(data)^)},
		&childCount,
	)
}

// A GJK simplex cache (distance.odin, ShapeDistance): at most a tetrahedron (a triangle against a capsule: at most
// a triangle), each point a vertex of its proxy (A has `countA` points, B `countB`).
@(private = "file")
ValidSimplexCache :: proc(cache: ^SimplexCache, countA, countB: i32) -> bool {
	limit := countA == 3 ? 3 : 4 // MAX_SIMPLEX_VERTICES (distance.odin)
	if i32(cache.count) > i32(limit) || !Finite(cache.metric) {
		return false
	}
	for i in 0 ..< i32(cache.count) {
		if i32(cache.indexA[i]) >= countA || i32(cache.indexB[i]) >= countB {
			return false
		}
	}
	return true
}

// A SAT cache (convex_manifold.odin CollideHulls, triangle_manifold.odin CollideTriangleAndHull): the separating
// feature it names exists. `hullA` nil: A is a triangle (one face, three edges).
@(private = "file")
ValidSATCache :: proc(cache: ^SATCache, hullA, hullB: ^HullData) -> bool {
	if !In(i32(cache.type), i32(SeparatingFeature.closestPointsAxis) + 1) || !Finite(cache.separation) {
		return false
	}
	a, b := i32(cache.indexA), i32(cache.indexB)
	#partial switch SeparatingFeature(cache.type) {
	case .faceAxisA:
		return hullA == nil ? a == 0 : a < hullA.faceCount
	case .faceAxisB:
		return b < hullB.faceCount
	case .edgePairAxis:
		edgesA := hullA == nil ? 3 : hullA.edgeCount
		return (hullA == nil ? a < 3 : a & 1 == 0 && a + 1 < edgesA) && b & 1 == 0 && b + 1 < hullB.edgeCount
	}
	return true
}

// The nodes of a tree (a world's broad-phase tree, or a compound's): from the root, every internal node's child pair
// inside [2, nodeEnd) and even, its children's parents pointing back (when `parents` is given), every node reached
// once, every leaf accepted by `leafOk`. An empty tree is a root that is an empty node.
@(private = "file")
ValidTreeNodes :: proc(
	nodes: [^]TreeNode,
	nodeEnd: i32,
	parents: [^]i32,
	leafOk: proc(leaf: ^TreeNode, data: rawptr) -> bool,
	data: rawptr,
	visitedOut: ^[dynamic]bool = nil,
) -> bool {
	if nodeEnd < 2 || nodeEnd & 1 != 0 {
		return false
	}
	root := &nodes[ROOT_NODE]
	if IsEmptyNode(root) {
		return true
	}
	visited := make([]bool, nodeEnd)
	defer if visitedOut == nil {
		delete(visited)
	}
	stack := make([dynamic]i32, 0, 64)
	defer delete(stack)
	append(&stack, ROOT_NODE)
	for len(stack) > 0 {
		index := pop(&stack)
		if visited[index] {
			return false
		}
		visited[index] = true
		node := &nodes[index]
		if IsEmptyNode(node) || !Box(node.aabb) {
			return false
		}
		if IsLeaf(node) {
			if !leafOk(node, data) {
				return false
			}
			continue
		}
		pair := GetLeftChild(node)
		if pair & 1 != 0 || pair < 2 || pair >= nodeEnd {
			return false
		}
		if parents != nil && (parents[pair] != index || parents[pair + 1] != index) {
			return false
		}
		append(&stack, pair, pair + 1)
	}
	if visitedOut != nil {
		resize(visitedOut, int(nodeEnd))
		copy(visitedOut[:], visited)
		delete(visited)
	}
	return true
}

// ---- the world --------------------------------------------------------------------------------------------------

// The id pool of an array whose slot i is live when live(i), free otherwise: every slot below nextIndex is live or
// free, nextIndex is the array's count, and the free list holds exactly the free slots, once each.
@(private = "file")
ValidPool :: proc(
	pool: ^IdPool,
	count: i32,
	live: proc(i: i32, data: rawptr) -> (live, free: bool),
	data: rawptr,
) -> bool {
	if pool.nextIndex != count || pool.freeArray.count < 0 || pool.freeArray.count > count {
		return false
	}
	inFree := make([]bool, count)
	defer delete(inFree)
	for k in 0 ..< pool.freeArray.count {
		id := pool.freeArray.data[k]
		if !In(id, count) || inFree[id] {
			return false
		}
		inFree[id] = true
	}
	for i in 0 ..< count {
		isLive, isFree := live(i, data)
		if isLive == isFree || isFree != inFree[i] {
			return false
		}
	}
	return true
}

// Whether `world`, just loaded from an image, is safe to use (the file's comment). On false the world is broken;
// the caller restores its own copy or destroys it.
ValidLoadedWorld :: proc(world: ^World) -> bool {
	bodies, shapes, contacts, joints, islands, sets :=
		world.bodies.count,
		world.shapes.count,
		world.contacts.count,
		world.joints.count,
		world.islands.count,
		world.solverSets.count

	// World scalars
	if !FiniteVec(world.gravity) ||
	   !Finite(world.hitEventThreshold) ||
	   !Finite(world.restitutionThreshold) ||
	   !Finite(world.maxLinearSpeed) ||
	   !Finite(world.contactSpeed) ||
	   !Finite(world.contactHertz) ||
	   !Finite(world.contactDampingRatio) ||
	   !Finite(world.contactRecycleDistance) ||
	   !Finite(world.inv_h) ||
	   !Finite(world.inv_dt) ||
	   world.restitutionIterations < 0 ||
	   world.restitutionIterations > 64 ||
	   (world.endEventArrayIndex != 0 && world.endEventArrayIndex != 1) {
		return false
	}

	// Sparse arrays and their id pools
	if sets < i32(SetType.firstSleepingSet) || shapes >= MAX_SHAPES || world.fatAABBs.count < shapes {
		return false
	}
	bodyLive :: proc(i: i32, data: rawptr) -> (bool, bool) {
		world := (^World)(data)
		id := world.bodies.data[i].id
		return id == i, id == NULL_INDEX
	}
	shapeLive :: proc(i: i32, data: rawptr) -> (bool, bool) {
		world := (^World)(data)
		id := world.shapes.data[i].id
		return id == i, id == NULL_INDEX
	}
	contactLive :: proc(i: i32, data: rawptr) -> (bool, bool) {
		world := (^World)(data)
		id := world.contacts.data[i].contactId
		return id == i, id == NULL_INDEX
	}
	jointLive :: proc(i: i32, data: rawptr) -> (bool, bool) {
		world := (^World)(data)
		id := world.joints.data[i].jointId
		return id == i, id == NULL_INDEX
	}
	islandLive :: proc(i: i32, data: rawptr) -> (bool, bool) {
		world := (^World)(data)
		id := world.islands.data[i].islandId
		return id == i, id == NULL_INDEX
	}
	setLive :: proc(i: i32, data: rawptr) -> (bool, bool) {
		world := (^World)(data)
		id := world.solverSets.data[i].setIndex
		return id == i, id == NULL_INDEX
	}
	if !ValidPool(&world.bodyIdPool, bodies, bodyLive, world) ||
	   !ValidPool(&world.shapeIdPool, shapes, shapeLive, world) ||
	   !ValidPool(&world.contactIdPool, contacts, contactLive, world) ||
	   !ValidPool(&world.jointIdPool, joints, jointLive, world) ||
	   !ValidPool(&world.islandIdPool, islands, islandLive, world) ||
	   !ValidPool(&world.solverSetIdPool, sets, setLive, world) {
		return false
	}
	for s in 0 ..< i32(SetType.firstSleepingSet) {
		if world.solverSets.data[s].setIndex != s {
			return false
		}
	}
	liveBody :: #force_inline proc(world: ^World, id: i32) -> bool {
		return In(id, world.bodies.count) && world.bodies.data[id].id == id
	}
	liveShape :: #force_inline proc(world: ^World, id: i32) -> bool {
		return In(id, world.shapes.count) && world.shapes.data[id].id == id
	}
	liveContact :: #force_inline proc(world: ^World, id: i32) -> bool {
		return In(id, world.contacts.count) && world.contacts.data[id].contactId == id
	}
	liveJoint :: #force_inline proc(world: ^World, id: i32) -> bool {
		return In(id, world.joints.count) && world.joints.data[id].jointId == id
	}
	liveIsland :: #force_inline proc(world: ^World, id: i32) -> bool {
		return In(id, world.islands.count) && world.islands.data[id].islandId == id
	}
	liveSet :: #force_inline proc(world: ^World, id: i32) -> bool {
		return In(id, world.solverSets.count) && world.solverSets.data[id].setIndex == id
	}

	// Solver sets: every sim names a live object that names it back (set and local index), so with the counts below
	// each live body, island, and every contact and joint not in the graph is in exactly one set.
	setBodies, setContacts, setJoints, setIslands: i32
	awake := &world.solverSets.data[SetType.awakeSet]
	for s in 0 ..< sets {
		set := &world.solverSets.data[s]
		if set.setIndex != s {
			if set.bodySims.count != 0 ||
			   set.bodyStates.count != 0 ||
			   set.jointSims.count != 0 ||
			   set.contactIndices.count != 0 ||
			   set.islandSims.count != 0 {
				return false
			}
			continue
		}
		switch {
		case s == i32(SetType.staticSet):
			if set.contactIndices.count != 0 || set.islandSims.count != 0 || set.bodyStates.count != 0 {
				return false
			}
		case s == i32(SetType.disabledSet):
			if set.islandSims.count != 0 || set.bodyStates.count != 0 {
				return false
			}
		case s == i32(SetType.awakeSet):
			if set.bodyStates.count != set.bodySims.count || set.jointSims.count != 0 {
				return false
			}
		case:
			if set.bodyStates.count != 0 {
				return false
			}
		}
		for i in 0 ..< set.bodySims.count {
			sim := &set.bodySims.data[i]
			if !liveBody(world, sim.bodyId) {
				return false
			}
			body := &world.bodies.data[sim.bodyId]
			if body.setIndex != s || body.localIndex != i {
				return false
			}
			if !Frame(sim.transform) ||
			   !Position(sim.center) ||
			   !Rotation(sim.rotation0) ||
			   !Position(sim.center0) ||
			   !Position(sim.localCenter) ||
			   !FiniteVec(sim.force) ||
			   !FiniteVec(sim.torque) ||
			   !Position(sim.maxExtent) ||
			   !Length(sim.minExtent) ||
			   !Finite(sim.invMass) ||
			   !FiniteOf(&sim.inertia) ||
			   !FiniteOf(&sim.invInertiaLocal) ||
			   !FiniteOf(&sim.invInertiaWorld) ||
			   !Finite(sim.linearDamping) ||
			   !Finite(sim.angularDamping) ||
			   !Finite(sim.gravityScale) {
				return false
			}
		}
		for i in 0 ..< set.bodyStates.count {
			state := &set.bodyStates.data[i]
			if !FiniteVec(state.linearVelocity) ||
			   !FiniteVec(state.angularVelocity) ||
			   !Position(state.deltaPosition) ||
			   !Rotation(state.deltaRotation) {
				return false
			}
		}
		for i in 0 ..< set.contactIndices.count {
			id := set.contactIndices.data[i]
			if !liveContact(world, id) {
				return false
			}
			contact := &world.contacts.data[id]
			if contact.setIndex != s || contact.colorIndex != NULL_INDEX || contact.localIndex != i {
				return false
			}
		}
		for i in 0 ..< set.jointSims.count {
			id := set.jointSims.data[i].jointId
			if !liveJoint(world, id) {
				return false
			}
			joint := &world.joints.data[id]
			if joint.setIndex != s || joint.colorIndex != NULL_INDEX || joint.localIndex != i {
				return false
			}
		}
		for i in 0 ..< set.islandSims.count {
			id := set.islandSims.data[i].islandId
			if !liveIsland(world, id) {
				return false
			}
			island := &world.islands.data[id]
			if island.setIndex != s || island.localIndex != i {
				return false
			}
		}
		setBodies += set.bodySims.count
		setContacts += set.contactIndices.count
		setJoints += set.jointSims.count
		setIslands += set.islandSims.count
	}
	if setBodies != GetIdCount(&world.bodyIdPool) || setIslands != GetIdCount(&world.islandIdPool) {
		return false
	}

	// The constraint graph: awake touching contacts and awake joints, each where its color and local index say.
	graphContacts, graphJoints: i32
	for c in 0 ..< i32(GRAPH_COLOR_COUNT) {
		color := &world.constraintGraph.colors[c]
		for i in 0 ..< color.convexContacts.count {
			id := color.convexContacts.data[i]
			if !liveContact(world, id) {
				return false
			}
			contact := &world.contacts.data[id]
			if contact.setIndex != i32(SetType.awakeSet) ||
			   contact.colorIndex != c ||
			   contact.localIndex != i ||
			   contact.flags & simMeshContact != 0 ||
			   c == OVERFLOW_INDEX {
				return false
			}
		}
		for i in 0 ..< color.contacts.count {
			id := color.contacts.data[i].contactId
			if !liveContact(world, id) {
				return false
			}
			contact := &world.contacts.data[id]
			if contact.setIndex != i32(SetType.awakeSet) ||
			   contact.colorIndex != c ||
			   contact.localIndex != i ||
			   (contact.flags & simMeshContact == 0 && c != OVERFLOW_INDEX) {
				return false
			}
		}
		for i in 0 ..< color.jointSims.count {
			id := color.jointSims.data[i].jointId
			if !liveJoint(world, id) {
				return false
			}
			joint := &world.joints.data[id]
			if joint.setIndex != i32(SetType.awakeSet) || joint.colorIndex != c || joint.localIndex != i {
				return false
			}
		}
		graphContacts += color.convexContacts.count + color.contacts.count
		graphJoints += color.jointSims.count
	}
	if setContacts + graphContacts != GetIdCount(&world.contactIdPool) ||
	   setJoints + graphJoints != GetIdCount(&world.jointIdPool) {
		return false
	}

	// Bodies: their set, island, shape, contact and joint lists.
	shapesInLists: i32
	contactEdges, jointEdges: i32
	for b in 0 ..< bodies {
		body := &world.bodies.data[b]
		if body.id != b {
			continue
		}
		// unified: a move index names the body's slot in the last step's move events, which can outnumber the awake
		// bodies now (a body destroyed or put to sleep after the step), and images don't carry the events: it only has to
		// stay inside the bodies' range (TrySleepIsland checks it against the events it has)
		if !liveSet(world, body.setIndex) ||
		   !In(i32(body.type), i32(BodyType.bodyTypeCount)) ||
		   body.headChainId != NULL_INDEX ||
		   (body.bodyMoveIndex != NULL_INDEX && !In(body.bodyMoveIndex, bodies)) ||
		   !Finite(body.sleepThreshold) ||
		   !Finite(body.sleepTime) ||
		   !Finite(body.sleepVelocity) ||
		   !Finite(body.safetyFactor) ||
		   !Finite(body.mass) ||
		   body.shapeCount < 0 ||
		   body.contactCount < 0 ||
		   body.jointCount < 0 {
			return false
		}
		if body.islandId != NULL_INDEX {
			if !liveIsland(world, body.islandId) {
				return false
			}
			island := &world.islands.data[body.islandId]
			if !In(body.islandIndex, island.bodies.count) || island.bodies.data[body.islandIndex] != b {
				return false
			}
		}
		set := body.setIndex
		// A static body is in the static set (or disabled), and the static set holds only static bodies.
		if (set == i32(SetType.staticSet)) != (body.type == .staticBody) && set != i32(SetType.disabledSet) {
			return false
		}
		// Shapes, in order, their prev links back
		previous: i32 = NULL_INDEX
		shapeId := body.headShapeId
		count: i32 = 0
		for shapeId != NULL_INDEX {
			if count >= body.shapeCount || !liveShape(world, shapeId) {
				return false
			}
			shape := &world.shapes.data[shapeId]
			if shape.bodyId != b || shape.prevShapeId != previous {
				return false
			}
			previous = shapeId
			shapeId = shape.nextShapeId
			count += 1
		}
		if count != body.shapeCount {
			return false
		}
		shapesInLists += count
		// Contacts: a key is contact id << 1 | edge
		count = 0
		previousKey: i32 = NULL_INDEX
		key := body.headContactKey
		for key != NULL_INDEX {
			id, edge := key >> 1, key & 1
			if count >= body.contactCount || !liveContact(world, id) {
				return false
			}
			contact := &world.contacts.data[id]
			if contact.edges[edge].bodyId != b || contact.edges[edge].prevKey != previousKey {
				return false
			}
			if set == i32(SetType.disabledSet) {
				return false // a disabled body has no contacts
			}
			previousKey = key
			key = contact.edges[edge].nextKey
			count += 1
		}
		if count != body.contactCount {
			return false
		}
		contactEdges += count
		// Joints: a key is joint id << 1 | edge
		count = 0
		previousKey = NULL_INDEX
		key = body.headJointKey
		for key != NULL_INDEX {
			id, edge := key >> 1, key & 1
			if count >= body.jointCount || !liveJoint(world, id) {
				return false
			}
			joint := &world.joints.data[id]
			if joint.edges[edge].bodyId != b || joint.edges[edge].prevKey != previousKey {
				return false
			}
			previousKey = key
			key = joint.edges[edge].nextKey
			count += 1
		}
		if count != body.jointCount {
			return false
		}
		jointEdges += count
	}
	if shapesInLists != GetIdCount(&world.shapeIdPool) ||
	   contactEdges != 2 * GetIdCount(&world.contactIdPool) ||
	   jointEdges != 2 * GetIdCount(&world.jointIdPool) {
		return false
	}

	// Shapes: type and geometry, materials, the broad-phase proxy, the sensor.
	sensorShapes: i32
	compoundShapes: i32
	for s in 0 ..< shapes {
		shape := &world.shapes.data[s]
		if shape.id != s {
			continue
		}
		body := &world.bodies.data[shape.bodyId] // live: the body lists above reached every live shape
		if !In(i32(shape.type), i32(ShapeType.shapeTypeCount)) ||
		   !Box(shape.aabb) ||
		   !Position(shape.localCentroid) ||
		   !Finite(shape.density) ||
		   !Finite(shape.explosionScale) ||
		   !Length(shape.aabbMargin) ||
		   !Box(world.fatAABBs.data[s]) {
			return false
		}
		if shape.materials == nil ? shape.materialCount < 0 || shape.materialCount > 1 : shape.materialCount < 1 {
			return false
		}
		materials := i32(max(shape.materialCount, 1))
		switch shape.type {
		case .sphereShape:
			if !ValidSphere(shape.sphere) {
				return false
			}
		case .capsuleShape:
			if !ValidCapsule(shape.capsule) {
				return false
			}
		case .hullShape:
			if shape.hull == nil {
				return false
			}
		case .meshShape:
			if shape.mesh.data == nil || !Position(shape.mesh.scale) {
				return false
			}
		case .heightShape:
			if shape.heightField == nil {
				return false
			}
		case .compoundShape:
			if shape.compound == nil {
				return false
			}
			compoundShapes += 1
			// The children's material indices are the compound's (checked with the blob); the shape must have them.
			if max(shape.compound.materialCount, 1) > materials {
				return false
			}
		case .shapeTypeCount:
			return false
		}
		// The proxy: in the tree of the body's type unless the body is disabled.
		set := body.setIndex
		if set == i32(SetType.disabledSet) {
			if shape.proxyKey != NULL_INDEX {
				return false
			}
		} else {
			proxyType := PROXY_TYPE(shape.proxyKey)
			proxyId := PROXY_ID(shape.proxyKey)
			if shape.proxyKey < 0 || !In(i32(proxyType), i32(BodyType.bodyTypeCount)) {
				return false
			}
			if (set == i32(SetType.staticSet)) != (proxyType == .staticBody) {
				return false
			}
			tree := &world.broadPhase.trees[proxyType]
			if !In(proxyId, tree.proxyCapacity) {
				return false
			}
			proxy := &tree.proxies[proxyId]
			if !In(proxy.node, tree.nodeEnd) || proxy.userData != u64(s) {
				return false
			}
		}
		if shape.sensorIndex != NULL_INDEX {
			if !In(shape.sensorIndex, world.sensors.count) || world.sensors.data[shape.sensorIndex].shapeId != s {
				return false
			}
			sensorShapes += 1
		}
	}
	if sensorShapes != world.sensors.count || compoundShapes != world.compoundShapeCount {
		return false
	}
	for i in 0 ..< world.sensors.count {
		sensor := &world.sensors.data[i]
		if !liveShape(world, sensor.shapeId) || world.shapes.data[sensor.shapeId].sensorIndex != i {
			return false
		}
		lists := [3]^Array(Visitor){&sensor.hits, &sensor.overlaps1, &sensor.overlaps2}
		for list in lists {
			for k in 0 ..< list.count {
				if !In(list.data[k].shapeId, shapes) {
					return false
				}
			}
		}
	}

	// Contacts: their shapes and bodies, set and color, island, manifolds, mesh caches.
	for c in 0 ..< contacts {
		contact := &world.contacts.data[c]
		if contact.contactId != c {
			continue
		}
		if !liveShape(world, contact.shapeIdA) ||
		   !liveShape(world, contact.shapeIdB) ||
		   !liveSet(world, contact.setIndex) {
			return false
		}
		shapeA, shapeB := &world.shapes.data[contact.shapeIdA], &world.shapes.data[contact.shapeIdB]
		if contact.edges[0].bodyId != shapeA.bodyId || contact.edges[1].bodyId != shapeB.bodyId {
			return false
		}
		bodyA, bodyB := &world.bodies.data[shapeA.bodyId], &world.bodies.data[shapeB.bodyId]
		if contact.encodedBodySimA != EncodeBodySimIndex(bodyA) ||
		   contact.encodedBodySimB != EncodeBodySimIndex(bodyB) {
			return false
		}
		touching := contact.flags & contactTouchingFlag != 0
		// Between steps: the transient sim flags are consumed (UpdateContacts), the sim's touching flag mirrors the
		// contact's, a touching contact has manifolds and one that isn't has none.
		if contact.flags & (simDisjoint | simStartedTouching | simStoppedTouching) != 0 ||
		   (contact.flags & simTouchingFlag != 0) != touching ||
		   touching != (contact.manifoldCount > 0) {
			return false
		}
		// The island: a touching contact is in the island of each of its bodies that isn't static, one that isn't
		// touching in none (ValidateConnectivity).
		if !touching && contact.islandId != NULL_INDEX {
			return false
		}
		if touching {
			for body in ([2]^Body{bodyA, bodyB}) {
				if body.setIndex != i32(SetType.staticSet) && body.islandId != contact.islandId {
					return false
				}
			}
		}
		switch {
		case contact.setIndex == i32(SetType.awakeSet):
			if touching != (contact.colorIndex != NULL_INDEX) {
				return false
			}
		case contact.setIndex >= i32(SetType.firstSleepingSet):
			if !touching || contact.manifoldCount < 1 {
				return false
			}
		case:
			if touching || contact.setIndex != i32(SetType.disabledSet) {
				return false
			}
		}
		if contact.colorIndex != NULL_INDEX && !In(contact.colorIndex, GRAPH_COLOR_COUNT) {
			return false
		}
		if contact.islandId != NULL_INDEX {
			if !liveIsland(world, contact.islandId) {
				return false
			}
			island := &world.islands.data[contact.islandId]
			if !In(contact.islandIndex, island.contacts.count) ||
			   island.contacts.data[contact.islandIndex].contactId != c {
				return false
			}
		}
		if contact.manifoldCount < 0 || (contact.manifoldCount > 0) != (contact.manifolds != nil) {
			return false
		}
		// The child of a compound A, and the triangles a mesh-like A has.
		triangles: i32 = 0
		meshLike := false
		#partial switch shapeA.type {
		case .meshShape:
			meshLike, triangles = true, shapeA.mesh.data.triangleCount
		case .heightShape:
			meshLike, triangles = true, GetHeightFieldTriangleCount(shapeA.heightField)
		case .compoundShape:
			compound := shapeA.compound
			if !In(
				contact.childIndex,
				compound.capsuleCount + compound.hullCount + compound.meshCount + compound.sphereCount,
			) {
				return false
			}
			child := GetCompoundChild(compound, contact.childIndex)
			if child.type == .meshShape {
				meshLike, triangles = true, child.mesh.data.triangleCount
			}
		case:
			if contact.childIndex != 0 {
				return false
			}
		}
		// The pairs contact.odin registers: B is a sphere, a capsule or a hull; a capsule A meets no hull, a sphere A
		// only spheres.
		switch shapeB.type {
		case .sphereShape:
		case .capsuleShape:
			if shapeA.type == .sphereShape {
				return false
			}
		case .hullShape:
			if shapeA.type == .sphereShape || shapeA.type == .capsuleShape {
				return false
			}
		case .compoundShape, .meshShape, .heightShape, .shapeTypeCount:
			return false
		}
		if meshLike != (contact.flags & simMeshContact != 0) {
			return false
		}
		for m in 0 ..< contact.manifoldCount {
			manifold := &contact.manifolds[m]
			if !In(manifold.pointCount, MAX_MANIFOLD_POINTS + 1) ||
			   !FiniteVec(manifold.normal) ||
			   !Finite(manifold.twistImpulse) ||
			   !FiniteVec(manifold.frictionImpulse) ||
			   !FiniteVec(manifold.rollingImpulse) {
				return false
			}
			for p in 0 ..< manifold.pointCount {
				point := &manifold.points[p]
				if !Position(point.anchorA) ||
				   !Position(point.anchorB) ||
				   !Finite(point.separation) ||
				   !Finite(point.normalImpulse) ||
				   !Finite(point.totalNormalImpulse) ||
				   !Finite(point.normalVelocity) ||
				   !Finite(point.baseSeparation) ||
				   (meshLike && !In(point.triangleIndex, triangles)) {
					return false
				}
			}
		}
		// The warm-start caches (a raw union: the variant the pair's collider reads, contact.odin and
		// mesh_contact.odin): a hull A against a sphere or capsule keeps a GJK simplex, against a hull the SAT axis;
		// a mesh-like A keeps one per triangle (a simplex against a capsule, an axis against a hull).
		hullA: ^HullData = nil
		#partial switch shapeA.type {
		case .hullShape:
			hullA = shapeA.hull
		case .compoundShape:
			if child := GetCompoundChild(shapeA.compound, contact.childIndex); child.type == .hullShape {
				hullA = child.hull
			}
		}
		if !meshLike && hullA != nil {
			cache := &contact.convexContact.cache
			#partial switch shapeB.type {
			case .sphereShape:
				if !ValidSimplexCache(&cache.simplexCache, hullA.vertexCount, 1) {
					return false
				}
			case .capsuleShape:
				if !ValidSimplexCache(&cache.simplexCache, hullA.vertexCount, 2) {
					return false
				}
			case .hullShape:
				if !ValidSATCache(&cache.satCache, hullA, shapeB.hull) {
					return false
				}
			}
		}
		if meshLike {
			cache := &contact.meshContact.triangleCache
			if cache.count < 0 || (cache.count > 0 && cache.data == nil) || cache.capacity < cache.count {
				return false
			}
			for k in 0 ..< cache.count {
				entry := &cache.data[k]
				if !In(entry.triangleIndex, triangles) {
					return false
				}
				#partial switch shapeB.type {
				case .capsuleShape:
					if !ValidSimplexCache(&entry.cache.simplexCache, 3, 2) {
						return false
					}
				case .hullShape:
					if !ValidSATCache(&entry.cache.satCache, nil, shapeB.hull) {
						return false
					}
				}
			}
			if !Box(contact.meshContact.queryBounds) {
				return false
			}
		}
		if !Rotation(contact.cachedRotationA) ||
		   !Rotation(contact.cachedRotationB) ||
		   !Frame(contact.cachedRelativePose) ||
		   !Finite(contact.friction) ||
		   !Finite(contact.restitution) ||
		   !Finite(contact.rollingResistance) ||
		   !FiniteVec(contact.tangentVelocity) {
			return false
		}
	}

	// Joints: bodies, type, sim, island.
	for j in 0 ..< joints {
		joint := &world.joints.data[j]
		if joint.jointId != j {
			continue
		}
		if !liveBody(world, joint.edges[0].bodyId) ||
		   !liveBody(world, joint.edges[1].bodyId) ||
		   !In(i32(joint.type), i32(JointType.pogoJoint) + 1) ||
		   !liveSet(world, joint.setIndex) ||
		   !Finite(joint.drawScale) {
			return false
		}
		sim: ^JointSim
		if joint.colorIndex != NULL_INDEX {
			if !In(joint.colorIndex, GRAPH_COLOR_COUNT) {
				return false
			}
			sims := &world.constraintGraph.colors[joint.colorIndex].jointSims
			if !In(joint.localIndex, sims.count) {
				return false
			}
			sim = &sims.data[joint.localIndex]
		} else {
			sims := &world.solverSets.data[joint.setIndex].jointSims
			if !In(joint.localIndex, sims.count) {
				return false
			}
			sim = &sims.data[joint.localIndex]
		}
		if sim.jointId != j ||
		   sim.bodyIdA != joint.edges[0].bodyId ||
		   sim.bodyIdB != joint.edges[1].bodyId ||
		   sim.type != joint.type ||
		   !Frame(sim.localFrameA) ||
		   !Frame(sim.localFrameB) ||
		   !Finite(sim.invMassA) ||
		   !Finite(sim.invMassB) ||
		   !FiniteOf(&sim.invIA) ||
		   !FiniteOf(&sim.invIB) {
			return false
		}
		if joint.islandId != NULL_INDEX {
			if !liveIsland(world, joint.islandId) {
				return false
			}
			island := &world.islands.data[joint.islandId]
			if !In(joint.islandIndex, island.joints.count) || island.joints.data[joint.islandIndex].jointId != j {
				return false
			}
		}
	}

	// Islands: their lists name live objects that name the island back at that position.
	for i in 0 ..< islands {
		island := &world.islands.data[i]
		if island.islandId != i {
			continue
		}
		if island.constraintRemoveCount < 0 {
			return false
		}
		for k in 0 ..< island.bodies.count {
			id := island.bodies.data[k]
			if !liveBody(world, id) || world.bodies.data[id].islandId != i || world.bodies.data[id].islandIndex != k {
				return false
			}
		}
		for k in 0 ..< island.contacts.count {
			link := island.contacts.data[k]
			if !liveContact(world, link.contactId) {
				return false
			}
			contact := &world.contacts.data[link.contactId]
			if contact.islandId != i ||
			   contact.islandIndex != k ||
			   link.bodyIdA != contact.edges[0].bodyId ||
			   link.bodyIdB != contact.edges[1].bodyId {
				return false
			}
		}
		for k in 0 ..< island.joints.count {
			link := island.joints.data[k]
			if !liveJoint(world, link.jointId) {
				return false
			}
			joint := &world.joints.data[link.jointId]
			if joint.islandId != i ||
			   joint.islandIndex != k ||
			   link.bodyIdA != joint.edges[0].bodyId ||
			   link.bodyIdB != joint.edges[1].bodyId {
				return false
			}
		}
	}
	if world.splitIslandId != NULL_INDEX && !liveIsland(world, world.splitIslandId) {
		return false
	}

	// The broad phase: each tree's nodes and proxies, every leaf a live shape's proxy, the pair set.
	for t in 0 ..< i32(BodyType.bodyTypeCount) {
		if !ValidTree(world, &world.broadPhase.trees[t], BodyType(t)) {
			return false
		}
	}
	pairs := &world.broadPhase.pairSet
	if pairs.capacity > 0 {
		if pairs.count >= pairs.capacity || pairs.items == nil {
			return false
		}
		used: u32 = 0
		for k in 0 ..< pairs.capacity {
			used += pairs.items[k].key != 0 ? 1 : 0
		}
		if used != pairs.count {
			return false
		}
	} else if pairs.count != 0 {
		return false
	}
	return true
}

// A broad-phase tree of proxies of `type`: its nodes (ValidTreeNodes), every leaf's proxy pointing back at it and
// naming a live shape whose proxy key is this one, the proxy free list holding exactly the unused proxies, and the
// node pairs outside the tree all on the pair free list.
@(private = "file")
ValidTree :: proc(world: ^World, tree: ^DynamicTree, type: BodyType) -> bool {
	if tree.version != DYNAMIC_TREE_VERSION ||
	   tree.proxyCapacity < 1 ||
	   tree.proxyCount < 0 ||
	   tree.proxyCount > tree.proxyCapacity ||
	   tree.nodeEnd > tree.nodeCapacity {
		return false
	}
	Leaves :: struct {
		world: ^World,
		tree:  ^DynamicTree,
		type:  BodyType,
		nodes: [^]TreeNode,
		count: i32,
	}
	leaves := Leaves{world, tree, type, tree.nodes, 0}
	leafOk :: proc(leaf: ^TreeNode, data: rawptr) -> bool {
		leaves := (^Leaves)(data)
		tree := leaves.tree
		proxyId := GetProxyId(leaf)
		if !In(proxyId, tree.proxyCapacity) {
			return false
		}
		proxy := &tree.proxies[proxyId]
		index := i32((uintptr(leaf) - uintptr(leaves.nodes)) / size_of(TreeNode))
		shapeId := i32(u32(proxy.userData))
		if proxy.node != index || proxy.userData >> 32 != 0 || leaf.shapeIndex != shapeId {
			return false
		}
		world := leaves.world
		if !In(shapeId, world.shapes.count) ||
		   world.shapes.data[shapeId].id != shapeId ||
		   world.shapes.data[shapeId].proxyKey != PROXY_KEY(proxyId, leaves.type) {
			return false
		}
		leaves.count += 1
		return true
	}
	visited: [dynamic]bool
	defer delete(visited)
	if !ValidTreeNodes(tree.nodes, tree.nodeEnd, tree.parents, leafOk, &leaves, &visited) {
		return false
	}
	if leaves.count != tree.proxyCount {
		return false
	}
	// Proxies: used ones are leaves (counted above through their nodes); free ones are on the free list, once.
	used: i32 = 0
	for p in 0 ..< tree.proxyCapacity {
		if tree.proxies[p].node != NULL_INDEX {
			used += 1
		}
	}
	if used != tree.proxyCount {
		return false
	}
	free: i32 = 0
	for p := tree.proxyFreeList; p != NULL_INDEX; p = tree.proxies[p].next {
		if !In(p, tree.proxyCapacity) ||
		   tree.proxies[p].node != NULL_INDEX ||
		   free >= tree.proxyCapacity - tree.proxyCount {
			return false
		}
		free += 1
	}
	if free != tree.proxyCapacity - tree.proxyCount {
		return false
	}
	// Node pairs: in the tree or on the free list, never both, each free pair once and empty.
	inTree := len(visited) > 0
	for pair := tree.pairFreeList; pair != NULL_INDEX; pair = tree.parents[pair] {
		if pair & 1 != 0 || pair < 2 || pair >= tree.nodeEnd {
			return false
		}
		if inTree && (visited[pair] || visited[pair + 1]) {
			return false
		}
		if !IsEmptyNode(&tree.nodes[pair]) || !IsEmptyNode(&tree.nodes[pair + 1]) {
			return false
		}
		if !inTree {
			resize(&visited, int(tree.nodeEnd))
			inTree = true
		}
		visited[pair], visited[pair + 1] = true, true // reached once
	}
	if len(visited) > 0 {
		for i in 2 ..< tree.nodeEnd {
			if !visited[i] {
				return false
			}
		}
	} else if tree.nodeEnd != 2 {
		return false
	}
	return true
}
