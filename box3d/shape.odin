// Ported from box3d src/shape.c and src/shape.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:intrinsics"
import "base:runtime"
import "core:mem"

// B3_REC and B3_REC_CREATE are expanded inline at each call site (`if world.recording != nil {...}`):
// the macros paste the op name into the args type and writer names, which Odin cannot do.

// b3ShapeFlags
enableSensorEvents :: 0x01
enableContactEvents :: 0x02
enableCustomFiltering :: 0x04
enableHitEvents :: 0x08
enablePreSolveEvents :: 0x10
// 0x20 is free, it was the enlarged AABB flag before the broad-phase marked the tree
enableSpeculative :: 0x40

Shape :: struct {
	id:             i32,
	bodyId:         i32,
	prevShapeId:    i32,
	nextShapeId:    i32,
	sensorIndex:    i32,
	proxyKey:       i32,
	type:           ShapeType,
	density:        f32,
	explosionScale: f32,
	aabbMargin:     f32,
	aabb:           AABB,
	localCentroid:  Vec3,
	materialCount:  i32,
	material:       SurfaceMaterial,
	materials:      [^]SurfaceMaterial,
	filter:         Filter,
	userData:       rawptr,
	userShape:      rawptr,
	nameId:         u32,
	generation:     u16,

	// b3ShapeFlags
	flags:          u8,
	using _:        struct #raw_union {
		capsule:     Capsule,
		sphere:      Sphere,
		hull:        ^HullData,
		mesh:        Mesh,
		heightField: ^HeightFieldData,
		compound:    ^CompoundData,
	},
}

// A single material shape keeps its material inline. Multi material meshes and compounds own a heap
// array. Reach the materials the same way for both: a single material shape presents its inline
// material as a one element array. Do not cache the pointer, the shapes array can move.
GetShapeMaterials :: proc(shape: ^Shape) -> [^]SurfaceMaterial {
	return shape.materials != nil ? shape.materials : &shape.material
}

GetHeightFieldTriangleCount :: proc(heightField: ^HeightFieldData) -> i32 {
	cellCount := (heightField.rowCount - 1) * (heightField.columnCount - 1)
	return 2 * cellCount
}

// Mesh

// Making this inline made the tree benchmarks faster.
GetMeshTriangle :: #force_inline proc(mesh: ^Mesh, triangleIndex: i32) -> Triangle {
	assert(0 <= triangleIndex && triangleIndex < mesh.data.triangleCount)

	triangles := GetMeshTriangles(mesh.data)
	flags := GetMeshFlags(mesh.data)
	vertices := GetMeshVertices(mesh.data)

	result: Triangle
	triangle := triangles[triangleIndex]
	triangleFlags := flags[triangleIndex]

	scale := mesh.scale

	result.vertices[0] = Mul(scale, vertices[triangle.index1])
	result.i1 = triangle.index1

	if scale.x * scale.y * scale.z < 0.0 {
		result.vertices[1] = Mul(scale, vertices[triangle.index3])
		result.vertices[2] = Mul(scale, vertices[triangle.index2])

		result.i2 = triangle.index3
		result.i3 = triangle.index2

		// mesh is inverted, so concave edges are now convex
		result.flags = 0
		result.flags |= (triangleFlags & inverseConcaveEdge1) != 0 ? concaveEdge1 : 0
		result.flags |= (triangleFlags & inverseConcaveEdge2) != 0 ? concaveEdge2 : 0
		result.flags |= (triangleFlags & inverseConcaveEdge3) != 0 ? concaveEdge3 : 0
	} else {
		result.vertices[1] = Mul(scale, vertices[triangle.index2])
		result.vertices[2] = Mul(scale, vertices[triangle.index3])

		result.i2 = triangle.index2
		result.i3 = triangle.index3
		result.flags = i32(triangleFlags)
	}

	return result
}

ShouldShapesCollide :: proc(filterA: Filter, filterB: Filter) -> bool {
	if filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0 {
		return filterA.groupIndex > 0
	}

	return (filterA.maskBits & filterB.categoryBits) != 0 && (filterA.categoryBits & filterB.maskBits) != 0
}

ShouldQueryCollide :: proc(shapeFilter: ^Filter, queryFilter: ^QueryFilter) -> bool {
	return(
		(shapeFilter.categoryBits & queryFilter.maskBits) != 0 &&
		(shapeFilter.maskBits & queryFilter.categoryBits) != 0 \
	)
}

IsConvex :: proc(type: ShapeType) -> bool {
	return type == .sphereShape || type == .capsuleShape || type == .hullShape
}

@(private = "file")
GetShape :: proc(world: ^World, shapeId: ShapeId) -> ^Shape {
	id := shapeId.index1 - 1
	shape := Array_Get(&world.shapes, id)
	assert(shape.id == id && shape.generation == shapeId.generation)
	return shape
}

@(private = "file")
ComputeShapeMargin :: proc(shape: ^Shape) -> f32 {
	margin: f32 = 0.0

	#partial switch shape.type {
	case .sphereShape:
		{
			margin = shape.sphere.radius
		}

	case .capsuleShape:
		{
			margin = 0.5 * Distance(shape.capsule.center2, shape.capsule.center1) + shape.capsule.radius
		}

	case .hullShape:
		{
			hull := shape.hull
			points := GetHullPoints(hull)
			maxExtentSqr: f32 = 0.0
			count := hull.vertexCount
			for i: i32 = 0; i < count; i += 1 {
				distSqr := DistanceSquared(points[i], hull.center)
				maxExtentSqr = MaxFloat(maxExtentSqr, distSqr)
			}
			margin = intrinsics.sqrt(maxExtentSqr)
		}

	case .meshShape, .heightShape, .compoundShape:
		{
			// Static-only shapes: broadphase uses speculative distance for static
			// proxies, so the per-shape margin is never consumed in practice.
			// Return the cap so any incidental use is generous.
			return MAX_AABB_MARGIN
		}

	case:
		VALIDATE(false)
		return MAX_AABB_MARGIN
	}

	return MinFloat(MAX_AABB_MARGIN, AABB_MARGIN_FRACTION * margin)
}

@(private = "file")
UpdateShapeAABBs :: proc(shape: ^Shape, fatAABB: ^AABB, transform: WorldTransform, proxyType: BodyType) {
	// Compute a bounding box with a speculative margin
	speculativeDistance := SPECULATIVE_DISTANCE
	aabbMargin := shape.aabbMargin

	aabb := ComputeFatShapeAABB(shape, transform, speculativeDistance)
	shape.aabb = aabb

	// Smaller margin for static bodies. Cannot be zero due to TOI tolerance.
	margin := proxyType == .staticBody ? speculativeDistance : aabbMargin
	fatAABB^ = AABB_Inflate(aabb, margin)
}

@(private = "file")
CreateShapeInternal :: proc(
	world: ^World,
	body: ^Body,
	bodyTransform: WorldTransform,
	def: ^ShapeDef,
	geometry: rawptr,
	shapeType: ShapeType,
	shapeTransform: Transform,
	scale: Vec3,
	haveShapeTransform: bool,
) -> ^Shape {
	shapeId := AllocId(&world.shapeIdPool)

	if shapeId == world.shapes.count {
		Array_Push(&world.shapes, Shape{})
		Array_Push(&world.fatAABBs, AABB{})
	} else {
		assert(world.shapes.data[shapeId].id == NULL_INDEX)
	}

	assert(world.fatAABBs.count == world.shapes.count)

	shape := Array_Get(&world.shapes, shapeId)

	#partial switch shapeType {
	case .capsuleShape:
		shape.capsule = (^Capsule)(geometry)^

	case .compoundShape:
		// Compounds must be a static and not a sensor
		assert(body.type == .staticBody)
		assert(def.isSensor == false)
		shape.compound = (^CompoundData)(geometry)

	case .sphereShape:
		shape.sphere = (^Sphere)(geometry)^

	case .hullShape:
		if haveShapeTransform {
			// The transform and non-uniform scale are baked into fresh data, then shared.
			baked := CloneAndTransformHull((^HullData)(geometry), shapeTransform, scale)
			if baked == nil {
				// This can fail to produce a valid hull in extreme cases
				FreeId(&world.shapeIdPool, shapeId)
				shape.id = NULL_INDEX
				return nil
			}

			shape.hull = AddOwnedHullToDatabase(world, baked)
		} else {
			shape.hull = AddHullToDatabase(world, (^HullData)(geometry))
		}

	case .meshShape:
		{
			shape.mesh.data = (^MeshData)(geometry)
			shape.mesh.scale = SafeScale(scale)
		}

	case .heightShape:
		shape.heightField = (^HeightFieldData)(geometry)

	case:
		assert(false)
	}

	shape.id = shapeId
	shape.bodyId = body.id
	shape.type = shapeType
	shape.density = def.density
	shape.explosionScale = def.explosionScale
	shape.filter = def.filter
	shape.userData = def.userData
	shape.userShape = nil
	shape.flags = 0
	shape.flags |= def.enableSensorEvents ? enableSensorEvents : 0
	shape.flags |= def.enableContactEvents ? enableContactEvents : 0
	shape.flags |= def.enableCustomFiltering ? enableCustomFiltering : 0
	shape.flags |= def.enableHitEvents ? enableHitEvents : 0
	shape.flags |= def.enablePreSolveEvents ? enablePreSolveEvents : 0
	shape.flags |= def.enableSpeculativeContact ? enableSpeculative : 0
	shape.proxyKey = NULL_INDEX
	shape.localCentroid = GetShapeCentroid(shape)
	shape.aabbMargin = ComputeShapeMargin(shape)
	shape.aabb = AABB{Vec3_zero, Vec3_zero}
	world.fatAABBs.data[shapeId] = AABB{Vec3_zero, Vec3_zero}
	shape.nameId = AddName(&world.names, def.name)
	shape.generation += 1

	if shape.type == .compoundShape {
		world.compoundShapeCount += 1

		// Own a copy of the compound materials so every shape frees its array the same way. Compounds
		// are few, so the copy is cheap and avoids aliasing the geometry blob.
		materialCount := shape.compound.materialCount
		shape.materialCount = materialCount
		shape.materials = cast([^]SurfaceMaterial)Alloc(int(materialCount) * size_of(SurfaceMaterial))
		mem.copy(shape.materials, GetCompoundMaterials(shape.compound), int(materialCount) * size_of(SurfaceMaterial))
	} else if def.materialCount > 1 && def.materials != nil {
		// Per triangle materials need a heap array.
		shape.materialCount = def.materialCount
		shape.materials = cast([^]SurfaceMaterial)Alloc(int(def.materialCount) * size_of(SurfaceMaterial))
		mem.copy(shape.materials, def.materials, int(def.materialCount) * size_of(SurfaceMaterial))
	} else {
		// The common case is one material, stored inline with no allocation.
		shape.material = (def.materialCount == 1 && def.materials != nil) ? def.materials[0] : def.baseMaterial
		shape.materialCount = 1
		shape.materials = nil
	}

	if body.setIndex != i32(SetType.disabledSet) {
		proxyType := body.type
		forcePairCreation := def.invokeContactCreation && shape.type != .compoundShape
		CreateShapeProxy(world, shape, proxyType, bodyTransform, forcePairCreation)
	}

	// Add to shape doubly linked list
	if body.headShapeId != NULL_INDEX {
		headShape := Array_Get(&world.shapes, body.headShapeId)
		headShape.prevShapeId = shapeId
	}

	shape.prevShapeId = NULL_INDEX
	shape.nextShapeId = body.headShapeId
	body.headShapeId = shapeId
	body.shapeCount += 1

	if def.isSensor {
		shape.sensorIndex = world.sensors.count
		sensor := Array_Emplace(&world.sensors)
		Array_CreateN(&sensor.hits, 4)
		Array_CreateN(&sensor.overlaps1, 16)
		Array_CreateN(&sensor.overlaps2, 16)
		sensor.shapeId = shapeId
	} else {
		shape.sensorIndex = NULL_INDEX
	}

	ValidateSolverSets(world)

	return shape
}

@(private = "file")
CreateShape :: proc(
	bodyId: BodyId,
	def: ^ShapeDef,
	geometry: rawptr,
	shapeType: ShapeType,
	transform: Transform,
	scale: Vec3,
	haveTransform: bool,
) -> ShapeId {
	CHECK_DEF(def)
	assert(IsValidFloat(def.density) && def.density >= 0.0)
	assert(IsValidFloat(def.baseMaterial.friction) && def.baseMaterial.friction >= 0.0)
	assert(IsValidFloat(def.baseMaterial.restitution) && def.baseMaterial.restitution >= 0.0)

	world := GetUnlockedWorld(i32(bodyId.world0))
	if world == nil {
		return ShapeId{}
	}

	if world.shapes.count == MAX_SHAPES && world.shapeIdPool.freeArray.count == 0 {
		assert(false)
		return nullShapeId
	}

	body := GetBodyFullId(world, bodyId)
	if body.type != .staticBody && (shapeType == .compoundShape || shapeType == .heightShape) {
		// Compound and height shapes must be on static bodies.
		return nullShapeId
	}

	world.locked = true

	bodyTransform := GetBodyTransformQuick(world, body)

	shape := CreateShapeInternal(world, body, bodyTransform, def, geometry, shapeType, transform, scale, haveTransform)

	if shape == nil {
		world.locked = false
		return nullShapeId
	}

	if def.updateBodyMass == true {
		UpdateBodyMassData(world, body)
	} else if (body.flags & dirtyMass) == 0 {
		body.flags |= dirtyMass
		SyncBodyFlags(world, body)
	}

	ValidateSolverSets(world)

	id := ShapeId{shape.id + 1, bodyId.world0, shape.generation}

	world.locked = false

	return id
}

CreateSphereShape :: proc(bodyId: BodyId, def: ^ShapeDef, sphere: ^Sphere) -> ShapeId {
	shapeId := CreateShape(bodyId, def, sphere, .sphereShape, Transform_identity, Vec3_one, false)
	if shapeId.index1 != 0 {
		world := GetUnlockedWorld(i32(bodyId.world0))
		if world != nil {
			if world.recording != nil {
				recCreateArgs := RecArgs_CreateSphereShape{bodyId, def^, sphere^}
				RecWriteRet_CreateSphereShape(world.recording, &recCreateArgs, shapeId)
			}
		}
	}
	return shapeId
}

CreateCapsuleShape :: proc(bodyId: BodyId, def: ^ShapeDef, capsule: ^Capsule) -> ShapeId {
	lengthSqr := DistanceSquared(capsule.center1, capsule.center2)
	shapeId: ShapeId
	if lengthSqr <= LINEAR_SLOP * LINEAR_SLOP {
		sphere := Sphere{Lerp(capsule.center1, capsule.center2, 0.5), capsule.radius}
		shapeId = CreateShape(bodyId, def, &sphere, .sphereShape, Transform_identity, Vec3_one, false)
	} else {
		shapeId = CreateShape(bodyId, def, capsule, .capsuleShape, Transform_identity, Vec3_one, false)
	}
	if shapeId.index1 != 0 {
		world := GetUnlockedWorld(i32(bodyId.world0))
		if world != nil {
			if world.recording != nil {
				recCreateArgs := RecArgs_CreateCapsuleShape{bodyId, def^, capsule^}
				RecWriteRet_CreateCapsuleShape(world.recording, &recCreateArgs, shapeId)
			}
		}
	}
	return shapeId
}

CreateHullShape :: proc(bodyId: BodyId, def: ^ShapeDef, hull: ^HullData) -> ShapeId {
	VALIDATE(IsValidHull(hull))
	assert(hull.version == HULL_VERSION)
	assert(hull.hash != 0)

	if hull.version != HULL_VERSION {
		return nullShapeId
	}

	shapeId := CreateShape(bodyId, def, hull, .hullShape, Transform_identity, Vec3_one, false)
	if shapeId.index1 != 0 {
		world := GetUnlockedWorld(i32(bodyId.world0))
		if world != nil && world.recording != nil {
			geometryId := RecInternHull(world.recording, hull)
			createArgs := RecArgs_CreateHullShape{bodyId, def^, geometryId}
			RecWriteRet_CreateHullShape(world.recording, &createArgs, shapeId)
		}
	}
	return shapeId
}

CreateTransformedHullShape :: proc(
	bodyId: BodyId,
	def: ^ShapeDef,
	hull: ^HullData,
	transform: Transform,
	scale: Vec3,
) -> ShapeId {
	VALIDATE(IsValidHull(hull))
	assert(hull.version == HULL_VERSION)
	assert(hull.hash != 0)

	if hull.version != HULL_VERSION {
		return nullShapeId
	}

	shapeId := CreateShape(bodyId, def, hull, .hullShape, transform, scale, true)
	if shapeId.index1 != 0 {
		world := GetUnlockedWorld(i32(bodyId.world0))
		if world != nil && world.recording != nil {
			// The transform and scale are baked into fresh hull data at create time. Record the baked hull
			// as a plain hull shape so replay rebuilds identical geometry with no rebake, and the keyframe
			// registry, which interns the live baked hull, stays seeded.
			shape := Array_Get(&world.shapes, shapeId.index1 - 1)
			geometryId := RecInternHull(world.recording, shape.hull)
			createArgs := RecArgs_CreateHullShape{bodyId, def^, geometryId}
			RecWriteRet_CreateHullShape(world.recording, &createArgs, shapeId)
		}
	}
	return shapeId
}

CreateMeshShape :: proc(bodyId: BodyId, def: ^ShapeDef, mesh: ^MeshData, scale: Vec3) -> ShapeId {
	VALIDATE(IsValidMesh(mesh))
	assert(mesh.version == MESH_VERSION)
	assert(mesh.hash != 0)

	if mesh.version != MESH_VERSION {
		return nullShapeId
	}

	shapeId := CreateShape(bodyId, def, mesh, .meshShape, Transform_identity, scale, true)
	if shapeId.index1 != 0 {
		world := GetUnlockedWorld(i32(bodyId.world0))
		if world != nil && world.recording != nil {
			geometryId := RecInternMesh(world.recording, mesh)
			createArgs := RecArgs_CreateMeshShape{bodyId, def^, geometryId, scale}
			RecWriteRet_CreateMeshShape(world.recording, &createArgs, shapeId)
		}
	}
	return shapeId
}

CreateHeightFieldShape :: proc(bodyId: BodyId, def: ^ShapeDef, heightField: ^HeightFieldData) -> ShapeId {
	assert(heightField.version == HEIGHT_FIELD_VERSION)
	assert(heightField.hash != 0)

	if heightField.version != HEIGHT_FIELD_VERSION {
		return nullShapeId
	}

	shapeId := CreateShape(bodyId, def, heightField, .heightShape, Transform_identity, Vec3_one, false)
	if shapeId.index1 != 0 {
		world := GetUnlockedWorld(i32(bodyId.world0))
		if world != nil && world.recording != nil {
			geometryId := RecInternHeightField(world.recording, heightField)
			createArgs := RecArgs_CreateHeightFieldShape{bodyId, def^, geometryId}
			RecWriteRet_CreateHeightFieldShape(world.recording, &createArgs, shapeId)
		}
	}
	return shapeId
}

CreateBakedCompoundShape :: proc(bodyId: BodyId, def: ^ShapeDef, compound: ^CompoundData) -> ShapeId {
	assert(compound.version == COMPOUND_VERSION)

	if compound.version != COMPOUND_VERSION {
		return nullShapeId
	}

	shapeId := CreateShape(bodyId, def, compound, .compoundShape, Transform_identity, Vec3_one, false)
	if shapeId.index1 != 0 {
		world := GetUnlockedWorld(i32(bodyId.world0))
		if world != nil && world.recording != nil {
			geometryId := RecInternCompound(world.recording, compound)
			createArgs := RecArgs_CreateCompoundShape{bodyId, def^, geometryId}
			RecWriteRet_CreateCompoundShape(world.recording, &createArgs, shapeId)
		}
	}
	return shapeId
}

// Destroy a shape on a body. This doesn't need to be called when destroying a body.
@(private = "file")
DestroyShapeInternal :: proc(world: ^World, shape: ^Shape, body: ^Body, wakeBodies: bool) {
	shapeId := shape.id

	if shape.type == .compoundShape {
		assert(world.compoundShapeCount > 0)
		world.compoundShapeCount -= 1
	}

	// Remove the shape from the body's doubly linked list.
	if shape.prevShapeId != NULL_INDEX {
		prevShape := Array_Get(&world.shapes, shape.prevShapeId)
		prevShape.nextShapeId = shape.nextShapeId
	}

	if shape.nextShapeId != NULL_INDEX {
		nextShape := Array_Get(&world.shapes, shape.nextShapeId)
		nextShape.prevShapeId = shape.prevShapeId
	}

	if shapeId == body.headShapeId {
		body.headShapeId = shape.nextShapeId
	}

	body.shapeCount -= 1

	// Remove from broad-phase.
	DestroyShapeProxy(shape, &world.broadPhase)

	// Destroy any contacts associated with the shape.
	contactKey := body.headContactKey
	for contactKey != NULL_INDEX {
		contactId := contactKey >> 1
		edgeIndex := contactKey & 1

		contact := Array_Get(&world.contacts, contactId)
		contactKey = contact.edges[edgeIndex].nextKey

		if contact.shapeIdA == shapeId || contact.shapeIdB == shapeId {
			DestroyContact(world, contact, wakeBodies)
		}
	}

	if shape.sensorIndex != NULL_INDEX {
		sensor := Array_Get(&world.sensors, shape.sensorIndex)
		for i: i32 = 0; i < sensor.overlaps2.count; i += 1 {
			ref := &sensor.overlaps2.data[i]
			event := SensorEndTouchEvent {
				sensorShapeId = {index1 = shapeId + 1, world0 = world.worldId, generation = shape.generation},
				visitorShapeId = {index1 = ref.shapeId + 1, world0 = world.worldId, generation = ref.generation},
			}

			Array_Push(&world.sensorEndEvents[world.endEventArrayIndex], event)
		}

		// Destroy sensor
		Array_Destroy(&sensor.hits)
		Array_Destroy(&sensor.overlaps1)
		Array_Destroy(&sensor.overlaps2)

		movedIndex := Array_RemoveSwap(&world.sensors, shape.sensorIndex)
		if movedIndex != NULL_INDEX {
			// Fixup moved sensor
			movedSensor := Array_Get(&world.sensors, shape.sensorIndex)
			otherSensorShape := Array_Get(&world.shapes, movedSensor.shapeId)
			otherSensorShape.sensorIndex = shape.sensorIndex
		}
	}

	// Destroy every shape member from b3Alloc
	DestroyShapeAllocations(world, shape)

	// Return shape to free list.
	FreeId(&world.shapeIdPool, shapeId)
	shape.id = NULL_INDEX

	ValidateSolverSets(world)
}

DestroyShape :: proc(shapeId: ShapeId, updateBodyMass: bool) {
	world := GetUnlockedWorld(i32(shapeId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_DestroyShape{shapeId, updateBodyMass}
		RecWrite_DestroyShape(world.recording, &recArgs)
	}

	world.locked = true

	shape := GetShape(world, shapeId)

	// need to wake bodies because this might be a static body
	wakeBodies := true

	body := Array_Get(&world.bodies, shape.bodyId)
	DestroyShapeInternal(world, shape, body, wakeBodies)

	if updateBodyMass == true {
		UpdateBodyMassData(world, body)
	}

	world.locked = false
}

ComputeShapeAABB :: proc(shape: ^Shape, transform: Transform) -> AABB {
	#partial switch shape.type {
	case .capsuleShape:
		return ComputeCapsuleAABB(&shape.capsule, transform)

	case .compoundShape:
		return ComputeCompoundAABB(shape.compound, transform)

	case .heightShape:
		return ComputeHeightFieldAABB(shape.heightField, transform)

	case .hullShape:
		return ComputeHullAABB(shape.hull, transform)

	case .meshShape:
		return ComputeMeshAABB(shape.mesh.data, transform, shape.mesh.scale)

	case .sphereShape:
		return ComputeSphereAABB(&shape.sphere, transform)

	case:
		{
			assert(false)
			empty := AABB{transform.p, transform.p}
			return empty
		}
	}
}

ComputeFatShapeAABB :: proc(shape: ^Shape, transform: WorldTransform, extra: f32) -> AABB {
	r := Vec3{extra, extra, extra}
	// unified: single precision only: the BOX3D_DOUBLE_PRECISION branch is not ported.
	aabb := ComputeShapeAABB(shape, transform)
	aabb.lowerBound = Sub(aabb.lowerBound, r)
	aabb.upperBound = Add(aabb.upperBound, r)
	return aabb
}

ComputeSweptShapeAABB :: proc(shape: ^Shape, sweep: ^Sweep, time: f32) -> AABB {
	assert(0.0 <= time && time <= 1.0)
	xf1 := Transform{Sub(sweep.c1, RotateVector(sweep.q1, sweep.localCenter)), sweep.q1}
	xf2 := GetSweepTransform(sweep, time)

	#partial switch shape.type {
	case .capsuleShape:
		return ComputeSweptCapsuleAABB(&shape.capsule, xf1, xf2)

	case .hullShape:
		return ComputeSweptHullAABB(shape.hull, xf1, xf2)

	case .sphereShape:
		return ComputeSweptSphereAABB(&shape.sphere, xf1, xf2)

	case:
		assert(false)
		return AABB{xf1.p, xf1.p}
	}
}

GetShapeCentroid :: proc(shape: ^Shape) -> Vec3 {
	#partial switch shape.type {
	case .capsuleShape:
		return Lerp(shape.capsule.center1, shape.capsule.center2, 0.5)
	case .compoundShape:
		{
			aabb := ComputeCompoundAABB(shape.compound, Transform_identity)
			return AABB_Center(aabb)
		}
	case .sphereShape:
		return shape.sphere.center
	case .hullShape:
		return shape.hull.center
	case .meshShape:
		{
			aabb := ComputeMeshAABB(shape.mesh.data, Transform_identity, shape.mesh.scale)
			return AABB_Center(aabb)
		}
	case .heightShape:
		{
			aabb := ComputeHeightFieldAABB(shape.heightField, Transform_identity)
			return AABB_Center(aabb)
		}
	case:
		return Vec3_zero
	}
}

GetShapeArea :: proc(shape: ^Shape) -> f32 {
	// todo_erin fix these
	#partial switch shape.type {
	case .capsuleShape:
		return 2.0 * Length(Sub(shape.capsule.center1, shape.capsule.center2)) + 2.0 * PI * shape.capsule.radius

	case .hullShape:
		return shape.hull.surfaceArea

	case .sphereShape:
		return 2.0 * PI * shape.sphere.radius

	case:
		return 0.0
	}
}

// This projects the shape surface area onto a plane
GetShapeProjectedArea :: proc(shape: ^Shape, planeNormal: Vec3) -> f32 {
	#partial switch shape.type {
	case .capsuleShape:
		{
			radius := shape.capsule.radius
			axis := Sub(shape.capsule.center2, shape.capsule.center1)
			projectedLength := Length(Cross(axis, planeNormal))
			cylinderArea := 2.0 * radius * projectedLength
			sphereArea := PI * radius * radius
			return sphereArea + cylinderArea
		}

	case .hullShape:
		return ComputeHullProjectedArea(shape.hull, planeNormal)

	case .sphereShape:
		return PI * shape.sphere.radius * shape.sphere.radius

	case:
		return 0.0
	}
}

ComputeShapeMass :: proc(shape: ^Shape) -> MassData {
	#partial switch shape.type {
	case .capsuleShape:
		return ComputeCapsuleMass(&shape.capsule, shape.density)

	case .hullShape:
		return ComputeHullMass(shape.hull, shape.density)

	case .sphereShape:
		return ComputeSphereMass(&shape.sphere, shape.density)

	case:
		return MassData{}
	}
}

ComputeShapeExtent :: proc(shape: ^Shape, localCenter: Vec3) -> ShapeExtent {
	extent := ShapeExtent{}

	#partial switch shape.type {
	case .capsuleShape:
		{
			radius := shape.capsule.radius
			extent.minExtent = radius
			c1 := Sub(shape.capsule.center1, localCenter)
			c2 := Sub(shape.capsule.center2, localCenter)
			r := Vec3{radius, radius, radius}
			extent.maxExtent = Add(Max(Abs(c1), Abs(c2)), r)
		}

	case .compoundShape:
		{
			// This is shouldn't be needed but here for completeness
			aabb := ComputeCompoundAABB(shape.compound, Transform_identity)
			r1 := Length(Sub(aabb.lowerBound, localCenter))
			r2 := Length(Sub(aabb.upperBound, localCenter))
			extent.minExtent = MinFloat(r1, r2)
			p := FarthestPointOnAABB(aabb, localCenter)
			extent.maxExtent = Abs(Sub(p, localCenter))
		}

	case .sphereShape:
		{
			radius := shape.sphere.radius
			extent.minExtent = radius
			h := Abs(Sub(shape.sphere.center, localCenter))
			r := Vec3{radius, radius, radius}
			extent.maxExtent = Add(h, r)
		}

	case .hullShape:
		extent = ComputeHullExtent(shape.hull, localCenter)

	case .meshShape:
		{
			// This is needed for kinematic mesh sleeping
			aabb := ComputeMeshAABB(shape.mesh.data, Transform_identity, shape.mesh.scale)
			r1 := Length(Sub(aabb.lowerBound, localCenter))
			r2 := Length(Sub(aabb.upperBound, localCenter))
			extent.minExtent = MinFloat(r1, r2)
			p := FarthestPointOnAABB(aabb, localCenter)
			extent.maxExtent = Abs(Sub(p, localCenter))
		}

	case:
	}

	return extent
}

ComputeShapeMinExtent :: proc(shape: ^Shape, localCenter: Vec3) -> f32 {
	#partial switch shape.type {
	case .capsuleShape:
		return shape.capsule.radius

	case .sphereShape:
		return shape.sphere.radius

	case .hullShape:
		return shape.hull.innerRadius

	case:
		return ComputeShapeExtent(shape, localCenter).minExtent
	}
}

RayCastShape :: proc(shape: ^Shape, transform: Transform, input: ^RayCastInput) -> CastOutput {
	localInput := input^
	localInput.origin = InvTransformPoint(transform, input.origin)
	localInput.translation = InvRotateVector(transform.q, input.translation)

	output := CastOutput{}
	#partial switch shape.type {
	case .capsuleShape:
		output = RayCastCapsule(&shape.capsule, &localInput)
	case .compoundShape:
		output = RayCastCompound(shape.compound, &localInput)
	case .sphereShape:
		output = RayCastSphere(&shape.sphere, &localInput)
	case .hullShape:
		output = RayCastHull(shape.hull, &localInput)
	case .meshShape:
		output = RayCastMesh(&shape.mesh, &localInput)
	case .heightShape:
		output = RayCastHeightField(shape.heightField, &localInput)
	case:
		return output
	}

	output.point = TransformPoint(transform, output.point)
	output.normal = RotateVector(transform.q, output.normal)
	return output
}

ShapeCastShape :: proc(shape: ^Shape, transform: Transform, input: ^ShapeCastInput) -> CastOutput {
	localInput := input^
	localPoints: [MAX_SHAPE_CAST_POINTS]Vec3 = ---

	localInput.proxy.count = MinInt(input.proxy.count, MAX_SHAPE_CAST_POINTS)
	for i: i32 = 0; i < localInput.proxy.count; i += 1 {
		localPoints[i] = InvTransformPoint(transform, input.proxy.points[i])
	}

	localInput.proxy.points = &localPoints[0]
	localInput.translation = InvRotateVector(transform.q, input.translation)

	output := CastOutput{}
	#partial switch shape.type {
	case .capsuleShape:
		output = ShapeCastCapsule(&shape.capsule, &localInput)

	case .compoundShape:
		output = ShapeCastCompound(shape.compound, &localInput)

	case .heightShape:
		output = ShapeCastHeightField(shape.heightField, &localInput)

	case .hullShape:
		output = ShapeCastHull(shape.hull, &localInput)

	case .meshShape:
		output = ShapeCastMesh(&shape.mesh, &localInput)

	case .sphereShape:
		output = ShapeCastSphere(&shape.sphere, &localInput)
	case:
		return output
	}

	output.point = TransformPoint(transform, output.point)
	output.normal = RotateVector(transform.q, output.normal)
	return output
}

OverlapShape :: proc(shape: ^Shape, transform: Transform, proxy: ^ShapeProxy) -> bool {
	type := shape.type
	#partial switch type {
	case .capsuleShape:
		return OverlapCapsule(&shape.capsule, transform, proxy)

	case .compoundShape:
		return OverlapCompound(shape.compound, transform, proxy)

	case .heightShape:
		return OverlapHeightField(shape.heightField, transform, proxy)

	case .hullShape:
		return OverlapHull(shape.hull, transform, proxy)

	case .meshShape:
		return OverlapMesh(&shape.mesh, transform, proxy)

	case .sphereShape:
		return OverlapSphere(&shape.sphere, transform, proxy)

	case:
		assert(false)
		return false
	}
}

CollideMover :: proc(
	planes: [^]PlaneResult,
	planeCapacity: i32,
	shape: ^Shape,
	transform: Transform,
	mover: ^Capsule,
) -> i32 {
	if planeCapacity == 0 {
		return 0
	}

	localMover: Capsule
	localMover.center1 = InvTransformPoint(transform, mover.center1)
	localMover.center2 = InvTransformPoint(transform, mover.center2)
	localMover.radius = mover.radius

	planeCount: i32 = 0
	#partial switch shape.type {
	case .capsuleShape:
		planeCount = CollideMoverAndCapsule(planes, &shape.capsule, &localMover)

	case .compoundShape:
		planeCount = CollideMoverAndCompound(planes, planeCapacity, shape.compound, &localMover)

	case .sphereShape:
		planeCount = CollideMoverAndSphere(planes, &shape.sphere, &localMover)

	case .hullShape:
		planeCount = CollideMoverAndHull(planes, shape.hull, &localMover)

	case .meshShape:
		planeCount = CollideMoverAndMesh(planes, planeCapacity, &shape.mesh, &localMover)

	case .heightShape:
		planeCount = CollideMoverAndHeightField(planes, planeCapacity, shape.heightField, &localMover)

	case:
		assert(false)
	}

	for i: i32 = 0; i < planeCount; i += 1 {
		planes[i].plane.normal = RotateVector(transform.q, planes[i].plane.normal)
		planes[i].point = TransformPoint(transform, planes[i].point)
		planes[i].materialIndex = ClampInt(planes[i].materialIndex, 0, shape.materialCount - 1)
	}

	return planeCount
}

CreateShapeProxy :: proc(
	world: ^World,
	shape: ^Shape,
	type: BodyType,
	transform: WorldTransform,
	forcePairCreation: bool,
) {
	assert(shape.proxyKey == NULL_INDEX)

	fatAABB := &world.fatAABBs.data[shape.id]
	UpdateShapeAABBs(shape, fatAABB, transform, type)

	// Create proxies in the broad-phase.
	shape.proxyKey = BroadPhase_CreateProxy(
		&world.broadPhase,
		type,
		fatAABB^,
		shape.filter.categoryBits,
		shape.id,
		forcePairCreation,
	)
	assert(PROXY_TYPE(shape.proxyKey) < .bodyTypeCount)
}

DestroyShapeProxy :: proc(shape: ^Shape, bp: ^BroadPhase) {
	if shape.proxyKey != NULL_INDEX {
		BroadPhase_DestroyProxy(bp, shape.proxyKey)
		shape.proxyKey = NULL_INDEX
	}
}

@(private = "file")
DestroyShapeAllocationForShapeChange :: proc(world: ^World, shape: ^Shape) {
	type := shape.type
	#partial switch type {
	case .hullShape:
		RemoveHullFromDatabase(world, shape.hull)
		shape.hull = nil

	case:
	}

	if shape.userShape != nil {
		world.destroyDebugShape(shape.userShape, world.userDebugShapeContext)
		shape.userShape = nil
	}
}

DestroyShapeAllocations :: proc(world: ^World, shape: ^Shape) {
	DestroyShapeAllocationForShapeChange(world, shape)

	if shape.materials != nil {
		assert(shape.materialCount > 0)
		Free(shape.materials, int(shape.materialCount) * size_of(SurfaceMaterial))
		shape.materials = nil
		shape.materialCount = 0
	}

	// Name is stored inline. Sensor data is destroyed elsewhere
}

MakeShapeProxy :: proc(shape: ^Shape) -> ShapeProxy {
	#partial switch shape.type {
	case .capsuleShape:
		return ShapeProxy{&shape.capsule.center1, 2, shape.capsule.radius}

	case .sphereShape:
		return ShapeProxy{&shape.sphere.center, 1, shape.sphere.radius}

	case .hullShape:
		{
			hull := shape.hull
			points := GetHullPoints(hull)
			return ShapeProxy{points, hull.vertexCount, 0.0}
		}

	case:
		{
			assert(false)
			return ShapeProxy{}
		}
	}
}

MakeLocalProxy :: proc(proxy: ^ShapeProxy, transform: Transform, buffer: [^]Vec3) -> ShapeProxy {
	invTransform := InvertTransform(transform)
	R := MakeMatrixFromQuat(invTransform.q)

	count := MinInt(proxy.count, MAX_SHAPE_CAST_POINTS)
	for i: i32 = 0; i < count; i += 1 {
		buffer[i] = Add(MulMV(R, proxy.points[i]), invTransform.p)
	}

	return ShapeProxy{points = buffer, count = count, radius = proxy.radius}
}

ComputeProxyAABB :: proc(proxy: ^ShapeProxy) -> AABB {
	points := proxy.points
	aabb := AABB {
		lowerBound = points[0],
		upperBound = points[0],
	}

	for i: i32 = 1; i < proxy.count; i += 1 {
		aabb.lowerBound = Min(aabb.lowerBound, points[i])
		aabb.upperBound = Max(aabb.upperBound, points[i])
	}

	r := Vec3{proxy.radius, proxy.radius, proxy.radius}
	aabb.lowerBound = Sub(aabb.lowerBound, r)
	aabb.upperBound = Add(aabb.upperBound, r)
	return aabb
}

Shape_GetBody :: proc(shapeId: ShapeId) -> BodyId {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	return MakeBodyId(world, shape.bodyId)
}

Shape_GetWorld :: proc(shapeId: ShapeId) -> WorldId {
	world := GetWorld(i32(shapeId.world0))
	return WorldId{u16(shapeId.world0 + 1), world.generation}
}

Shape_SetUserData :: proc(shapeId: ShapeId, userData: rawptr) {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	shape.userData = userData
}

Shape_GetUserData :: proc(shapeId: ShapeId) -> rawptr {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	return shape.userData
}

Shape_SetName :: proc(shapeId: ShapeId, name: cstring) {
	world := GetWorld(i32(shapeId.world0))

	if world.recording != nil {
		recArgs := RecArgs_ShapeSetName{shapeId, name}
		RecWrite_ShapeSetName(world.recording, &recArgs)
	}

	shape := GetShape(world, shapeId)
	shape.nameId = AddName(&world.names, name)
}

Shape_GetName :: proc(shapeId: ShapeId) -> cstring {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	return FindNameWithDefault(&world.names, shape.nameId, "")
}

Shape_IsSensor :: proc(shapeId: ShapeId) -> bool {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	return shape.sensorIndex != NULL_INDEX
}

// todo no tests
Shape_RayCast :: proc(shapeId: ShapeId, origin: Pos, translation: Vec3) -> WorldCastOutput {
	assert(IsValidPosition(origin))
	assert(IsValidVec3(translation))

	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)

	// Re-center on the origin so the cast runs in float precision far from the world origin
	transform := ToRelativeTransform(GetBodyTransform(world, shape.bodyId), origin)

	// The ray starts at the origin, so its origin in the re-centered frame is zero
	input := RayCastInput{Vec3_zero, translation, 1.0}

	// Lift the re-centered float result back to a world position
	local := RayCastShape(shape, transform, &input)
	output: WorldCastOutput
	output.normal = local.normal
	output.point = OffsetPos(origin, local.point)
	output.fraction = local.fraction
	output.iterations = local.iterations
	output.triangleIndex = local.triangleIndex
	output.childIndex = local.childIndex
	output.materialIndex = local.materialIndex
	output.hit = local.hit

	return output
}

Shape_SetDensity :: proc(shapeId: ShapeId, density: f32, updateBodyMass: bool) {
	assert(IsValidFloat(density) && density >= 0.0)

	world := GetUnlockedWorld(i32(shapeId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_ShapeSetDensity{shapeId, density, updateBodyMass}
		RecWrite_ShapeSetDensity(world.recording, &recArgs)
	}

	shape := GetShape(world, shapeId)
	if density == shape.density {
		// early return to avoid expensive function
		return
	}

	shape.density = density

	if updateBodyMass == true {
		body := Array_Get(&world.bodies, shape.bodyId)
		UpdateBodyMassData(world, body)
	}
}

Shape_GetDensity :: proc(shapeId: ShapeId) -> f32 {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	return shape.density
}

Shape_SetFriction :: proc(shapeId: ShapeId, friction: f32) {
	assert(IsValidFloat(friction) && friction >= 0.0)
	world := GetWorld(i32(shapeId.world0))
	if world.recording != nil {
		recArgs := RecArgs_ShapeSetFriction{shapeId, friction}
		RecWrite_ShapeSetFriction(world.recording, &recArgs)
	}
	shape := GetShape(world, shapeId)
	assert(shape.type != .compoundShape)
	GetShapeMaterials(shape)[0].friction = friction
}

Shape_GetFriction :: proc(shapeId: ShapeId) -> f32 {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	return GetShapeMaterials(shape)[0].friction
}

Shape_SetRestitution :: proc(shapeId: ShapeId, restitution: f32) {
	assert(IsValidFloat(restitution) && restitution >= 0.0)
	world := GetWorld(i32(shapeId.world0))
	if world.recording != nil {
		recArgs := RecArgs_ShapeSetRestitution{shapeId, restitution}
		RecWrite_ShapeSetRestitution(world.recording, &recArgs)
	}
	shape := GetShape(world, shapeId)
	assert(shape.type != .compoundShape)
	GetShapeMaterials(shape)[0].restitution = restitution
}

Shape_GetRestitution :: proc(shapeId: ShapeId) -> f32 {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	return GetShapeMaterials(shape)[0].restitution
}

Shape_SetSurfaceMaterial :: proc(shapeId: ShapeId, surfaceMaterial: SurfaceMaterial) {
	assert(IsValidFloat(surfaceMaterial.friction) && surfaceMaterial.friction >= 0.0)
	assert(IsValidFloat(surfaceMaterial.restitution) && surfaceMaterial.restitution >= 0.0)
	assert(IsValidFloat(surfaceMaterial.rollingResistance) && surfaceMaterial.rollingResistance >= 0.0)
	assert(IsValidVec3(surfaceMaterial.tangentVelocity))

	world := GetWorld(i32(shapeId.world0))
	if world.recording != nil {
		recArgs := RecArgs_ShapeSetSurfaceMaterial{shapeId, surfaceMaterial}
		RecWrite_ShapeSetSurfaceMaterial(world.recording, &recArgs)
	}
	shape := GetShape(world, shapeId)
	assert(shape.type != .compoundShape)
	GetShapeMaterials(shape)[0] = surfaceMaterial
}

Shape_GetSurfaceMaterial :: proc(shapeId: ShapeId) -> SurfaceMaterial {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	return GetShapeMaterials(shape)[0]
}

Shape_GetMeshMaterialCount :: proc(shapeId: ShapeId) -> i32 {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	return shape.materialCount
}

Shape_SetMeshMaterial :: proc(shapeId: ShapeId, surfaceMaterial: SurfaceMaterial, index: i32) {
	assert(IsValidFloat(surfaceMaterial.friction) && surfaceMaterial.friction >= 0.0)
	assert(IsValidFloat(surfaceMaterial.restitution) && surfaceMaterial.restitution >= 0.0)
	assert(IsValidFloat(surfaceMaterial.rollingResistance) && surfaceMaterial.rollingResistance >= 0.0)
	assert(IsValidVec3(surfaceMaterial.tangentVelocity))

	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)

	assert(0 <= index && index < shape.materialCount)
	assert(shape.type != .compoundShape)

	if world.recording != nil {
		recArgs := RecArgs_ShapeSetMeshMaterial{shapeId, surfaceMaterial, index}
		RecWrite_ShapeSetMeshMaterial(world.recording, &recArgs)
	}
	GetShapeMaterials(shape)[index] = surfaceMaterial
}

Shape_GetMeshSurfaceMaterial :: proc(shapeId: ShapeId, index: i32) -> SurfaceMaterial {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	assert(0 <= index && index < shape.materialCount)
	return GetShapeMaterials(shape)[index]
}

Shape_GetFilter :: proc(shapeId: ShapeId) -> Filter {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	return shape.filter
}

@(private = "file")
ResetProxy :: proc(world: ^World, shape: ^Shape, wakeBodies: bool, destroyProxy: bool) {
	body := Array_Get(&world.bodies, shape.bodyId)

	shapeId := shape.id

	// destroy all contacts associated with this shape
	contactKey := body.headContactKey
	for contactKey != NULL_INDEX {
		contactId := contactKey >> 1
		edgeIndex := contactKey & 1

		contact := Array_Get(&world.contacts, contactId)
		contactKey = contact.edges[edgeIndex].nextKey

		if contact.shapeIdA == shapeId || contact.shapeIdB == shapeId {
			DestroyContact(world, contact, wakeBodies)
		}
	}

	transform := GetBodyTransformQuick(world, body)
	if shape.proxyKey != NULL_INDEX {
		proxyType := PROXY_TYPE(shape.proxyKey)
		fatAABB := &world.fatAABBs.data[shapeId]
		UpdateShapeAABBs(shape, fatAABB, transform, proxyType)

		if destroyProxy {
			BroadPhase_DestroyProxy(&world.broadPhase, shape.proxyKey)

			forcePairCreation := true
			shape.proxyKey = BroadPhase_CreateProxy(
				&world.broadPhase,
				proxyType,
				fatAABB^,
				shape.filter.categoryBits,
				shapeId,
				forcePairCreation,
			)
		} else {
			BroadPhase_MoveProxy(&world.broadPhase, shape.proxyKey, fatAABB^)
		}
	} else {
		proxyType := body.type
		UpdateShapeAABBs(shape, &world.fatAABBs.data[shapeId], transform, proxyType)
	}

	ValidateSolverSets(world)
}

Shape_SetFilter :: proc(shapeId: ShapeId, filter: Filter, invokeContacts: bool) {
	world := GetUnlockedWorld(i32(shapeId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_ShapeSetFilter{shapeId, filter, invokeContacts}
		RecWrite_ShapeSetFilter(world.recording, &recArgs)
	}

	shape := GetShape(world, shapeId)
	if filter.maskBits == shape.filter.maskBits &&
	   filter.categoryBits == shape.filter.categoryBits &&
	   filter.groupIndex == shape.filter.groupIndex {
		return
	}

	shape.filter = filter

	if invokeContacts {
		world.locked = true
		wakeBodies := true

		// If the category bits change, I need to destroy the proxy because it affects the tree sorting.
		destroyProxy := filter.categoryBits == shape.filter.categoryBits

		// need to wake bodies because a filter change may destroy contacts
		ResetProxy(world, shape, wakeBodies, destroyProxy)
		world.locked = false
	}

	// note: this does not immediately update sensor overlaps. Instead sensor
	// overlaps are updated the next time step
}

Shape_EnableSensorEvents :: proc(shapeId: ShapeId, flag: bool) {
	world := GetUnlockedWorld(i32(shapeId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_ShapeEnableSensorEvents{shapeId, flag}
		RecWrite_ShapeEnableSensorEvents(world.recording, &recArgs)
	}

	shape := GetShape(world, shapeId)
	shape.flags = flag ? shape.flags | enableSensorEvents : shape.flags &~ enableSensorEvents
}

Shape_AreSensorEventsEnabled :: proc(shapeId: ShapeId) -> bool {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	return (shape.flags & enableSensorEvents) != 0
}

Shape_EnableContactEvents :: proc(shapeId: ShapeId, flag: bool) {
	world := GetUnlockedWorld(i32(shapeId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_ShapeEnableContactEvents{shapeId, flag}
		RecWrite_ShapeEnableContactEvents(world.recording, &recArgs)
	}

	shape := GetShape(world, shapeId)
	shape.flags = flag ? shape.flags | enableContactEvents : shape.flags &~ enableContactEvents
}

Shape_AreContactEventsEnabled :: proc(shapeId: ShapeId) -> bool {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	return (shape.flags & enableContactEvents) != 0
}

Shape_EnablePreSolveEvents :: proc(shapeId: ShapeId, flag: bool) {
	world := GetUnlockedWorld(i32(shapeId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_ShapeEnablePreSolveEvents{shapeId, flag}
		RecWrite_ShapeEnablePreSolveEvents(world.recording, &recArgs)
	}

	shape := GetShape(world, shapeId)
	shape.flags = flag ? shape.flags | enablePreSolveEvents : shape.flags &~ enablePreSolveEvents
}

Shape_ArePreSolveEventsEnabled :: proc(shapeId: ShapeId) -> bool {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	return (shape.flags & enablePreSolveEvents) != 0
}

Shape_EnableHitEvents :: proc(shapeId: ShapeId, flag: bool) {
	world := GetUnlockedWorld(i32(shapeId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_ShapeEnableHitEvents{shapeId, flag}
		RecWrite_ShapeEnableHitEvents(world.recording, &recArgs)
	}

	shape := GetShape(world, shapeId)
	shape.flags = flag ? shape.flags | enableHitEvents : shape.flags &~ enableHitEvents
}

Shape_AreHitEventsEnabled :: proc(shapeId: ShapeId) -> bool {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	return (shape.flags & enableHitEvents) != 0
}

Shape_GetType :: proc(shapeId: ShapeId) -> ShapeType {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	return shape.type
}

Shape_GetSphere :: proc(shapeId: ShapeId) -> Sphere {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	assert(shape.type == .sphereShape)
	return shape.sphere
}

Shape_GetCapsule :: proc(shapeId: ShapeId) -> Capsule {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	assert(shape.type == .capsuleShape)
	return shape.capsule
}

Shape_GetHull :: proc(shapeId: ShapeId) -> ^HullData {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	assert(shape.type == .hullShape)
	return shape.hull
}

Shape_GetMesh :: proc(shapeId: ShapeId) -> Mesh {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	assert(shape.type == .meshShape)
	return shape.mesh
}

Shape_GetHeightField :: proc(shapeId: ShapeId) -> ^HeightFieldData {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	assert(shape.type == .heightShape)
	return shape.heightField
}

Shape_GetCompound :: proc(shapeId: ShapeId) -> ^CompoundData {
	world := GetWorld(i32(shapeId.world0))
	shape := GetShape(world, shapeId)
	assert(shape.type == .compoundShape)
	return shape.compound
}

Shape_SetSphere :: proc(shapeId: ShapeId, sphere: ^Sphere) {
	world := GetUnlockedWorld(i32(shapeId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_ShapeSetSphere{shapeId, sphere^}
		RecWrite_ShapeSetSphere(world.recording, &recArgs)
	}

	world.locked = true

	shape := GetShape(world, shapeId)

	DestroyShapeAllocationForShapeChange(world, shape)

	shape.sphere = sphere^
	shape.type = .sphereShape
	shape.aabbMargin = ComputeShapeMargin(shape)

	// need to wake bodies so they can react to the shape change
	wakeBodies := true
	destroyProxy := true
	ResetProxy(world, shape, wakeBodies, destroyProxy)

	world.locked = false
}

Shape_SetCapsule :: proc(shapeId: ShapeId, capsule: ^Capsule) {
	world := GetUnlockedWorld(i32(shapeId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_ShapeSetCapsule{shapeId, capsule^}
		RecWrite_ShapeSetCapsule(world.recording, &recArgs)
	}

	world.locked = true

	shape := GetShape(world, shapeId)

	DestroyShapeAllocationForShapeChange(world, shape)

	shape.capsule = capsule^
	shape.type = .capsuleShape
	shape.aabbMargin = ComputeShapeMargin(shape)

	// need to wake bodies so they can react to the shape change
	wakeBodies := true
	destroyProxy := true
	ResetProxy(world, shape, wakeBodies, destroyProxy)

	world.locked = false
}

Shape_SetHull :: proc(shapeId: ShapeId, hull: ^HullData) {
	VALIDATE(IsValidHull(hull))
	VALIDATE(hull.hash != 0)

	world := GetUnlockedWorld(i32(shapeId.world0))
	if world == nil {
		return
	}

	world.locked = true

	shape := GetShape(world, shapeId)

	// Acquire the new hull before releasing the old so the input may safely alias
	// the shape's current shared data.
	data := AddHullToDatabase(world, hull)

	// Same shared hull, avoid destroying contacts and recreating the proxy
	if shape.type == .hullShape && data == shape.hull {
		RemoveHullFromDatabase(world, data)
		world.locked = false
		return
	}

	if world.recording != nil {
		// Intern the shared hull.
		geometryId := RecInternHull(world.recording, data)
		setArgs := RecArgs_ShapeSetHull{shapeId, geometryId}
		RecWrite_ShapeSetHull(world.recording, &setArgs)
	}

	DestroyShapeAllocationForShapeChange(world, shape)

	shape.hull = data
	shape.type = .hullShape
	shape.aabbMargin = ComputeShapeMargin(shape)

	// need to wake bodies so they can react to the shape change
	wakeBodies := true
	destroyProxy := true
	ResetProxy(world, shape, wakeBodies, destroyProxy)

	world.locked = false
}

Shape_SetMesh :: proc(shapeId: ShapeId, meshData: ^MeshData, scale: Vec3) {
	assert(IsValidVec3(scale))
	assert(meshData != nil && IsValidMesh(meshData))

	world := GetUnlockedWorld(i32(shapeId.world0))
	if world == nil {
		return
	}

	world.locked = true

	if world.recording != nil {
		geometryId := RecInternMesh(world.recording, meshData)
		setArgs := RecArgs_ShapeSetMesh{shapeId, geometryId, scale}
		RecWrite_ShapeSetMesh(world.recording, &setArgs)
	}

	shape := GetShape(world, shapeId)

	DestroyShapeAllocationForShapeChange(world, shape)

	shape.mesh.data = meshData
	shape.mesh.scale = SafeScale(scale)
	shape.type = .meshShape
	shape.aabbMargin = ComputeShapeMargin(shape)

	// need to wake bodies so they can react to the shape change
	wakeBodies := true
	destroyProxy := true
	ResetProxy(world, shape, wakeBodies, destroyProxy)

	world.locked = false
}

Shape_GetContactCapacity :: proc(shapeId: ShapeId) -> i32 {
	world := GetUnlockedWorld(i32(shapeId.world0))
	if world == nil {
		return 0
	}

	shape := GetShape(world, shapeId)
	if shape.sensorIndex != NULL_INDEX {
		return 0
	}

	body := Array_Get(&world.bodies, shape.bodyId)

	// Conservative and fast
	return body.contactCount
}

Shape_GetContactData :: proc(shapeId: ShapeId, contactData: [^]ContactData, capacity: i32) -> i32 {
	world := GetUnlockedWorld(i32(shapeId.world0))
	if world == nil {
		return 0
	}

	shape := GetShape(world, shapeId)
	if shape.sensorIndex != NULL_INDEX {
		return 0
	}

	body := Array_Get(&world.bodies, shape.bodyId)
	contactKey := body.headContactKey
	index: i32 = 0
	for contactKey != NULL_INDEX && index < capacity {
		contactId := contactKey >> 1
		edgeIndex := contactKey & 1

		contact := Array_Get(&world.contacts, contactId)

		// Does contact involve this shape and is it touching?
		if (contact.shapeIdA == shapeId.index1 - 1 || contact.shapeIdB == shapeId.index1 - 1) &&
		   (contact.flags & contactTouchingFlag) != 0 {
			shapeA := &world.shapes.data[contact.shapeIdA]
			shapeB := &world.shapes.data[contact.shapeIdB]

			contactData[index].contactId = ContactId{contact.contactId + 1, shapeId.world0, 0, contact.generation}
			contactData[index].shapeIdA = ShapeId{shapeA.id + 1, shapeId.world0, shapeA.generation}
			contactData[index].shapeIdB = ShapeId{shapeB.id + 1, shapeId.world0, shapeB.generation}
			contactData[index].manifolds = contact.manifolds
			contactData[index].manifoldCount = contact.manifoldCount
			index += 1
		}

		contactKey = contact.edges[edgeIndex].nextKey
	}

	assert(index <= capacity)

	return index
}

Shape_GetSensorCapacity :: proc(shapeId: ShapeId) -> i32 {
	world := GetUnlockedWorld(i32(shapeId.world0))
	if world == nil {
		return 0
	}

	shape := GetShape(world, shapeId)
	if shape.sensorIndex == NULL_INDEX {
		return 0
	}

	sensor := Array_Get(&world.sensors, shape.sensorIndex)
	return sensor.overlaps2.count
}

Shape_GetSensorData :: proc(shapeId: ShapeId, visitorIds: [^]ShapeId, capacity: i32) -> i32 {
	world := GetUnlockedWorld(i32(shapeId.world0))
	if world == nil {
		return 0
	}

	shape := GetShape(world, shapeId)
	if shape.sensorIndex == NULL_INDEX {
		return 0
	}

	sensor := Array_Get(&world.sensors, shape.sensorIndex)

	count := MinInt(sensor.overlaps2.count, capacity)
	refs := sensor.overlaps2.data
	for i: i32 = 0; i < count; i += 1 {
		visitorId := ShapeId {
			index1     = refs[i].shapeId + 1,
			world0     = shapeId.world0,
			generation = refs[i].generation,
		}

		visitorIds[i] = visitorId
	}

	return count
}

Shape_GetAABB :: proc(shapeId: ShapeId) -> AABB {
	world := GetWorld(i32(shapeId.world0))
	if world == nil {
		return AABB{}
	}

	shape := GetShape(world, shapeId)
	return shape.aabb
}

Shape_ComputeMassData :: proc(shapeId: ShapeId) -> MassData {
	world := GetWorld(i32(shapeId.world0))
	if world == nil {
		return MassData{}
	}

	shape := GetShape(world, shapeId)
	return ComputeShapeMass(shape)
}

Shape_GetClosestPoint :: proc(shapeId: ShapeId, target: Vec3) -> Vec3 {
	target := target
	world := GetWorld(i32(shapeId.world0))
	if world == nil {
		return Vec3_zero
	}

	shape := GetShape(world, shapeId)
	body := Array_Get(&world.bodies, shape.bodyId)
	// Low level closest point query is a documented float carve-out far from the origin
	transform := ToRelativeTransform(GetBodyTransformQuick(world, body), Pos_zero)

	input: DistanceInput
	input.proxyA = MakeShapeProxy(shape)
	input.proxyB = ShapeProxy{&target, 1, 0.0}
	input.transform = InvMulTransforms(transform, Transform_identity)
	input.useRadii = true

	cache := SimplexCache{}
	output := ShapeDistance(&input, &cache, nil, 0)

	// Witness point comes back in frame A, lift it back to the query frame
	return TransformPoint(transform, output.pointA)
}

DEBUG_WIND :: false

// https://en.wikipedia.org/wiki/Density_of_air
// https://www.engineeringtoolbox.com/wind-load-d_1775.html
// force = 0.5 * air_density * velocity^2 * area
// https://en.wikipedia.org/wiki/Lift_(force)
Shape_ApplyWind :: proc(shapeId: ShapeId, wind: Vec3, drag: f32, lift: f32, maxSpeed: f32, wake: bool) {
	world := GetWorld(i32(shapeId.world0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := RecArgs_ShapeApplyWind{shapeId, wind, drag, lift, maxSpeed, wake}
		RecWrite_ShapeApplyWind(world.recording, &recArgs)
	}

	shape := GetShape(world, shapeId)

	shapeType := shape.type
	if shapeType != .sphereShape && shapeType != .capsuleShape && shapeType != .hullShape {
		return
	}

	body := Array_Get(&world.bodies, shape.bodyId)

	if body.type != .dynamicBody {
		return
	}

	if body.setIndex == i32(SetType.disabledSet) {
		return
	}

	if body.setIndex >= i32(SetType.firstSleepingSet) && wake == false {
		return
	}

	sim := GetBodySim(world, body)

	if body.setIndex != i32(SetType.awakeSet) {
		// Must wake for state to exist
		WakeBodyWithLock(world, body)
	}

	assert(body.setIndex == i32(SetType.awakeSet))

	state := GetBodyState(world, body)
	// Only the rotation is used below, so the demoted world transform is exact
	transform := ToRelativeTransform(sim.transform, Pos_zero)

	lengthUnits := GetLengthUnitsPerMeter()
	volumeUnits := lengthUnits * lengthUnits * lengthUnits

	airDensity := 1.2250 / (volumeUnits)

	force := Vec3{}
	torque := Vec3{}

	#partial switch shape.type {
	case .sphereShape:
		{
			radius := shape.sphere.radius
			centroid := shape.localCentroid
			lever := RotateVector(transform.q, Sub(centroid, sim.localCenter))
			shapeVelocity := Add(state.linearVelocity, Cross(state.angularVelocity, lever))
			relativeVelocity := MulSub(wind, drag, shapeVelocity)
			speed: f32
			direction := GetLengthAndNormalize(&speed, relativeVelocity)
			speed = MinFloat(speed, maxSpeed)
			projectedArea := PI * radius * radius
			force = MulSV(0.5 * airDensity * projectedArea * speed * speed, direction)
			torque = Cross(lever, force)
		}

	case .capsuleShape:
		{
			centroid := shape.localCentroid
			lever := RotateVector(transform.q, Sub(centroid, sim.localCenter))
			shapeVelocity := Add(state.linearVelocity, Cross(state.angularVelocity, lever))
			relativeVelocity := MulSub(wind, drag, shapeVelocity)
			speed: f32
			direction := GetLengthAndNormalize(&speed, relativeVelocity)
			speed = MinFloat(speed, maxSpeed)

			d := Sub(shape.capsule.center2, shape.capsule.center1)
			d = RotateVector(transform.q, d)

			radius := shape.capsule.radius
			projectedArea := PI * radius * radius + 2.0 * radius * Length(Cross(d, direction))

			// Normal that opposes the wind
			e := Normalize(d)
			normal := Sub(MulSV(Dot(direction, e), e), direction)

			// portion of wind that is perpendicular to surface
			liftDirection := Cross(Cross(normal, direction), direction)

			forceMagnitude := 0.5 * airDensity * projectedArea * speed * speed
			force = MulSV(forceMagnitude, MulAdd(direction, lift, liftDirection))

			edgeLever := MulAdd(lever, radius, normal)
			torque = Cross(edgeLever, force)
		}

	case .hullShape:
		{
			matrix_ := MakeMatrixFromQuat(transform.q)

			faceCount := shape.hull.faceCount
			points := GetHullPoints(shape.hull)
			faces := GetHullFaces(shape.hull)
			edges := GetHullEdges(shape.hull)
			planes := GetHullPlanes(shape.hull)

			linearVelocity := state.linearVelocity
			angularVelocity := state.angularVelocity
			localCenterOfMass := sim.localCenter

			for i: i32 = 0; i < faceCount; i += 1 {
				face := &faces[i]
				edge1 := &edges[face.edge]
				edge2 := &edges[edge1.next]
				edge3 := &edges[edge2.next]

				assert(edge1 != edge3)
				assert(i32(edge1.origin) < shape.hull.vertexCount)
				assert(i32(edge2.origin) < shape.hull.vertexCount)

				localPoint1 := points[edge1.origin]
				localPoint2 := points[edge2.origin]
				v1 := MulMV(matrix_, localPoint1)
				v2 := MulMV(matrix_, localPoint2)
				normal := MulMV(matrix_, planes[i].normal)

				for {
					assert(i32(edge3.origin) < shape.hull.vertexCount)
					localPoint3 := points[edge3.origin]
					v3 := MulMV(matrix_, localPoint3)

					// Triangle center
					localCenter := MulSV(0.333333, Add(localPoint1, Add(localPoint2, localPoint3)))

					// Lever arm from center of mass to triangle center in world space
					lever := MulMV(matrix_, Sub(localCenter, localCenterOfMass))

					// Velocity of the triangle center in world space
					centerVelocity := Add(linearVelocity, Cross(angularVelocity, lever))

					relativeVelocity := MulSub(wind, drag, centerVelocity)
					speed: f32
					direction := GetLengthAndNormalize(&speed, relativeVelocity)

					// Check for back-side
					if Dot(normal, direction) < -FLT_EPSILON {
						projectedArea := -0.5 * Dot(Cross(Sub(v2, v1), Sub(v3, v1)), direction)
						VALIDATE(projectedArea >= -FLT_EPSILON)

						liftDirection := Cross(Cross(normal, direction), direction)

						speed = MinFloat(speed, maxSpeed)

						forceMagnitude := 0.5 * airDensity * projectedArea * speed * speed
						deltaForce := MulSV(forceMagnitude, MulAdd(direction, lift, liftDirection))
						deltaTorque := Cross(lever, deltaForce)

						force = Add(force, deltaForce)
						torque = Add(torque, deltaTorque)

						when DEBUG_WIND {
							lineIndex := world.taskContexts.data[0].lineCount
							if lineIndex < DEBUG_LINE_CAPACITY {
								line := &world.taskContexts.data[0].lines[lineIndex]
								line.p1 = OffsetPos(sim.transform.p, MulMV(matrix_, localCenter))
								line.p2 = OffsetPos(line.p1, deltaForce)
								line.label = i
								line.color = .colorBlanchedAlmond
								world.taskContexts.data[0].lineCount += 1
							}
						}
					}

					edge2 = edge3
					edge3 = &edges[edge3.next]
					v2 = v3
					localPoint2 = localPoint3

					if edge1 == edge3 {
						break
					}
				}
			}
		}

	case:
	}

	sim.force = Add(sim.force, force)
	sim.torque = Add(sim.torque, torque)
}

MeshImpactContext :: struct {
	toiInput:            TOIInput,
	toiOutput:           TOIOutput,
	// Centroid of shape in body B local space
	localCentroidB:      Vec3,
	// Centroid of shape at beginning and end of sweep in mesh local space. Used for early out.
	meshLocalCentroidB1: Vec3,
	meshLocalCentroidB2: Vec3,
	fallbackRadius:      f32,
	isSensor:            bool,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
MeshTimeOfImpactFcn :: proc "c" (a: Vec3, b: Vec3, c: Vec3, triangleIndex: i32, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	_ = triangleIndex

	toiContext := (^MeshImpactContext)(context_)

	// Early out for parallel movement
	c1 := toiContext.meshLocalCentroidB1
	c2 := toiContext.meshLocalCentroidB2

	n := Normalize(Cross(Sub(b, a), Sub(c, a)))
	offset1 := Dot(n, Sub(c1, a))
	offset2 := Dot(n, Sub(c2, a))

	if offset1 < 0.0 {
		// Started behind or finished in front
		return true
	}

	if toiContext.isSensor == false &&
	   offset1 - offset2 < toiContext.fallbackRadius &&
	   offset2 > toiContext.fallbackRadius {
		// Finished in front
		return true
	}

	triangle := [3]Vec3{a, b, c}
	// The rawptr conversion keeps C's pointer to stack memory without Odin's escape warning.
	toiContext.toiInput.proxyA.points = cast([^]Vec3)rawptr(&triangle[0])
	toiContext.toiInput.proxyA.count = 3

	output := TimeOfImpact(&toiContext.toiInput)

	// It is possible for a hit at fraction == 0

	if 0.0 < output.fraction && output.fraction < toiContext.toiInput.maxFraction {
		toiContext.toiOutput = output
		toiContext.toiInput.maxFraction = output.fraction
	} else if 0.0 == output.fraction {
		// fallback to TOI of a small circle around the fast shape centroid
		fallbackInput := toiContext.toiInput
		fallbackInput.proxyB = ShapeProxy{&toiContext.localCentroidB, 1, toiContext.fallbackRadius + LINEAR_SLOP}
		output = TimeOfImpact(&fallbackInput)

		if 0.0 < output.fraction && output.fraction < toiContext.toiInput.maxFraction {
			toiContext.toiOutput = output
			toiContext.toiInput.maxFraction = output.fraction
			toiContext.toiOutput.usedFallback = true
		}
	}

	// Continue the query
	return true
}

CompoundImpactContext :: struct {
	toiInput:          TOIInput,
	toiOutput:         TOIOutput,
	compoundTransform: Transform,

	// Bounds local to compound
	localSweepBoundsB: AABB,

	// Centroid of shape in body B local space
	localCentroidB:    Vec3,
	fallbackRadius:    f32,
}

// Implements b3CompoundQueryFcn
@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
CompoundTimeOfImpactFcn :: proc "c" (compound: ^CompoundData, childIndex: i32, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	toiContext := (^CompoundImpactContext)(context_)

	child := GetCompoundChild(compound, childIndex)

	output := TOIOutput{}
	toiContext.toiInput.sweepA = MakeCompoundChildSweep(toiContext.compoundTransform, child.transform)

	#partial switch child.type {
	case .capsuleShape:
		{
			toiContext.toiInput.proxyA.points = cast([^]Vec3)rawptr(&child.capsule.center1)
			toiContext.toiInput.proxyA.count = 2
			toiContext.toiInput.proxyA.radius = child.capsule.radius
			output = TimeOfImpact(&toiContext.toiInput)
		}

	case .hullShape:
		{
			toiContext.toiInput.proxyA.points = GetHullPoints(child.hull)
			toiContext.toiInput.proxyA.count = child.hull.vertexCount
			toiContext.toiInput.proxyA.radius = 0.0
			output = TimeOfImpact(&toiContext.toiInput)
		}

	case .meshShape:
		{
			meshContext := MeshImpactContext{}
			meshContext.toiInput = toiContext.toiInput
			meshContext.isSensor = false
			meshContext.localCentroidB = toiContext.localCentroidB
			meshContext.fallbackRadius = toiContext.fallbackRadius

			meshWorldTransform := MulTransforms(toiContext.compoundTransform, child.transform)

			sweepB := &toiContext.toiInput.sweepB
			xfB1 := Transform {
				p = Sub(sweepB.c1, RotateVector(sweepB.q1, sweepB.localCenter)),
				q = sweepB.q1,
			}

			xfB2 := Transform {
				p = Sub(sweepB.c2, RotateVector(sweepB.q2, sweepB.localCenter)),
				q = sweepB.q2,
			}

			meshContext.meshLocalCentroidB1 = InvTransformPoint(
				meshWorldTransform,
				TransformPoint(xfB1, meshContext.localCentroidB),
			)
			meshContext.meshLocalCentroidB2 = InvTransformPoint(
				meshWorldTransform,
				TransformPoint(xfB2, meshContext.localCentroidB),
			)

			// Bounds local to mesh
			localBounds := AABB_Transform(InvertTransform(child.transform), toiContext.localSweepBoundsB)

			QueryMesh(&child.mesh, localBounds, MeshTimeOfImpactFcn, &meshContext)

			output = meshContext.toiOutput
		}

	case .sphereShape:
		{
			toiContext.toiInput.proxyA.points = cast([^]Vec3)rawptr(&child.sphere.center)
			toiContext.toiInput.proxyA.count = 1
			toiContext.toiInput.proxyA.radius = child.sphere.radius
			output = TimeOfImpact(&toiContext.toiInput)
		}

	case:
		assert(false)
	}

	if 0.0 < output.fraction && output.fraction < toiContext.toiInput.maxFraction {
		toiContext.toiOutput = output
		toiContext.toiInput.maxFraction = output.fraction
	}

	// Clear this to be safe
	toiContext.toiInput.proxyA = ShapeProxy{}

	// Continue the query
	return true
}

ShapeTimeOfImpact :: proc(
	shapeA: ^Shape,
	shapeB: ^Shape,
	sweepA: ^Sweep,
	sweepB: ^Sweep,
	maxFraction: f32,
) -> TOIOutput {
	isSensor := shapeA.sensorIndex != NULL_INDEX

	typeA := shapeA.type
	if typeA == .compoundShape {
		// todo implement b3CompoundTimeOfImpact
		context_ := CompoundImpactContext{}
		context_.toiInput.proxyB = MakeShapeProxy(shapeB)
		context_.toiInput.sweepB = sweepB^
		context_.toiInput.maxFraction = maxFraction

		context_.compoundTransform = Transform {
			p = sweepA.c1,
			q = sweepA.q1,
		}

		localCentroidB := GetShapeCentroid(shapeB)
		context_.localCentroidB = localCentroidB

		minExtent := ComputeShapeMinExtent(shapeB, context_.localCentroidB)
		context_.fallbackRadius = MaxFloat(0.75 * minExtent, SPECULATIVE_DISTANCE)

		// Swept bounds of shapeB
		bounds := ComputeSweptShapeAABB(shapeB, sweepB, maxFraction)

		// Bounds local to mesh
		localBounds := AABB_Transform(InvertTransform(context_.compoundTransform), bounds)
		context_.localSweepBoundsB = localBounds

		QueryCompound(shapeA.compound, localBounds, CompoundTimeOfImpactFcn, &context_)

		return context_.toiOutput
	}

	if typeA == .heightShape || typeA == .meshShape {
		// todo implement b3MeshTimeOfImpact and b3HeightFieldTimeOfImpact
		// Note: assuming mesh is static

		context_ := MeshImpactContext{}
		context_.toiInput.sweepA = sweepA^
		context_.toiInput.proxyA.count = 3
		context_.toiInput.proxyB = MakeShapeProxy(shapeB)
		context_.toiInput.sweepB = sweepB^
		context_.toiInput.maxFraction = maxFraction
		context_.isSensor = isSensor

		localCentroidB := GetShapeCentroid(shapeB)
		context_.localCentroidB = localCentroidB

		// Assume mesh is static
		xfA := Transform {
			p = Sub(sweepA.c1, RotateVector(sweepA.q1, sweepA.localCenter)),
			q = sweepA.q1,
		}

		xfB1 := Transform {
			p = Sub(sweepB.c1, RotateVector(sweepB.q1, sweepB.localCenter)),
			q = sweepB.q1,
		}

		xfB2 := Transform {
			p = Sub(sweepB.c2, RotateVector(sweepB.q2, sweepB.localCenter)),
			q = sweepB.q2,
		}

		context_.meshLocalCentroidB1 = InvTransformPoint(xfA, TransformPoint(xfB1, localCentroidB))
		context_.meshLocalCentroidB2 = InvTransformPoint(xfA, TransformPoint(xfB2, localCentroidB))

		minExtent := ComputeShapeMinExtent(shapeB, context_.localCentroidB)
		context_.fallbackRadius = MaxFloat(0.5 * minExtent, LINEAR_SLOP)

		// Swept bounds of shapeB
		// todo pass in xfA to get local bounds directly
		bounds := ComputeSweptShapeAABB(shapeB, sweepB, maxFraction)

		// Bounds local to mesh
		localBounds := AABB_Transform(InvertTransform(xfA), bounds)

		if typeA == .meshShape {
			QueryMesh(&shapeA.mesh, localBounds, MeshTimeOfImpactFcn, &context_)
		} else if typeA == .heightShape {
			QueryHeightField(shapeA.heightField, localBounds, MeshTimeOfImpactFcn, &context_)
		}

		return context_.toiOutput
	}

	assert(shapeB.type != .compoundShape && shapeB.type != .meshShape && shapeB.type != .heightShape)

	input: TOIInput
	input.proxyA = MakeShapeProxy(shapeA)
	input.proxyB = MakeShapeProxy(shapeB)
	input.sweepA = sweepA^
	input.sweepB = sweepB^
	input.maxFraction = maxFraction

	output := TimeOfImpact(&input)

	when false {
		// todo I'm not sure this is worth it for convex vs convex.
		if 0.0 < output.fraction && output.fraction < maxFraction {
			return output
		}

		if 0.0 == output.fraction {
			// fallback to TOI of a small circle around the fast shape centroid
			centroid := GetShapeCentroid(shapeB)
			input.proxyB = ShapeProxy{&centroid, 1, SPECULATIVE_DISTANCE}
			output = TimeOfImpact(&input)
			return output
		}
	}

	return output
}

// Resolve the user material id for a hit point on the given shape. Mesh/heightfield shapes
// use the manifold-point triangleIndex to pick a per-triangle material. Compound shapes use
// the contact's childIndex to find the participating child, then for a mesh child apply the
// child's materialIndices indirection on top of the per-triangle index. Convex shapes fall
// back to materials[0]. childIndex is unused for non-compound shapes.
GetShapeUserMaterialId :: proc(shape: ^Shape, childIndex: i32, triangleIndex: i32) -> u64 {
	if shape.materialCount == 0 {
		return 0
	}

	materialIndex: i32 = 0
	if shape.type == .meshShape {
		indices := GetMeshMaterialIndices(shape.mesh.data)
		if indices != nil {
			materialIndex = i32(indices[triangleIndex])
		}
	} else if shape.type == .heightShape {
		materialIndex = GetHeightFieldMaterial(shape.heightField, triangleIndex)
	} else if shape.type == .compoundShape {
		child := GetCompoundChild(shape.compound, childIndex)
		if child.type == .meshShape {
			indices := GetMeshMaterialIndices(child.mesh.data)
			meshMaterialIndex := indices != nil ? i32(indices[triangleIndex]) : 0
			assert(0 <= meshMaterialIndex && meshMaterialIndex < child.materialCount)
			materialIndex = i32(child.materialIndices[meshMaterialIndex])
		} else {
			materialIndex = i32(child.materialIndices[0])
		}
	}

	materialIndex = ClampInt(materialIndex, 0, shape.materialCount - 1)
	return GetShapeMaterials(shape)[materialIndex].userMaterialId
}
