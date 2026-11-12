// Ported from box3d src/sensor.c and src/sensor.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:runtime"

// Used to track shapes that hit sensors using time of impact
SensorHit :: struct {
	sensorId:  i32,
	visitorId: i32,
}

Visitor :: struct {
	shapeId:    i32,
	generation: u16,
}

Sensor :: struct {
	hits:      Array(Visitor),
	overlaps1: Array(Visitor),
	overlaps2: Array(Visitor),
	shapeId:   i32,
}

SensorTaskContext :: struct {
	eventBits: BitSet,
}

// sensor.c

SensorQueryContext :: struct {
	world:       ^World,
	taskContext: ^SensorTaskContext,
	sensor:      ^Sensor,
	sensorShape: ^Shape,
	transform:   Transform,
}

@(private = "file")
OverlapSensor :: proc(
	sensorShape: ^Shape,
	sensorTransform: Transform,
	visitorShape: ^Shape,
	visitorTransform: Transform,
) -> bool {
	type := sensorShape.type

	proxy := MakeShapeProxy(visitorShape)

	// Get the visitor shape in the frame of the sensor
	relativeTransform := InvMulTransforms(sensorTransform, visitorTransform)

	localPoints: [MAX_SHAPE_CAST_POINTS]Vec3 = ---
	localProxy: ShapeProxy

	localProxy.count = MinInt(proxy.count, MAX_SHAPE_CAST_POINTS)
	for i: i32 = 0; i < localProxy.count; i += 1 {
		localPoints[i] = TransformPoint(relativeTransform, proxy.points[i])
	}

	localProxy.points = raw_data(&localPoints)
	localProxy.radius = proxy.radius

	#partial switch type {
	case .capsuleShape:
		return OverlapCapsule(&sensorShape.capsule, Transform_identity, &localProxy)

	case .compoundShape:
		return OverlapCompound(sensorShape.compound, Transform_identity, &localProxy)

	case .heightShape:
		return OverlapHeightField(sensorShape.heightField, Transform_identity, &localProxy)

	case .hullShape:
		return OverlapHull(sensorShape.hull, Transform_identity, &localProxy)

	case .meshShape:
		return OverlapMesh(&sensorShape.mesh, Transform_identity, &localProxy)

	case .sphereShape:
		return OverlapSphere(&sensorShape.sphere, Transform_identity, &localProxy)

	case:
		assert(false)
		return false
	}
}

// Sensor shapes need to
// - detect begin and end overlap events
// - events must be reported in deterministic order
// - maintain an active list of overlaps for query

// Assumption
// - sensors don't detect shapes on the same body

// Algorithm
// Query all sensors for overlaps
// Check against previous overlaps

// Data structures
// Each sensor has an double buffered array of overlaps
// These overlaps use a shape reference with index and generation

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
SensorQueryCallback :: proc "c" (proxyId: i32, userData: u64, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	_ = proxyId

	shapeId := i32(userData)

	queryContext := (^SensorQueryContext)(context_)
	sensorShape := queryContext.sensorShape
	sensorShapeId := sensorShape.id

	if shapeId == sensorShapeId {
		return true
	}

	world := queryContext.world
	otherShape := Array_Get(&world.shapes, shapeId)

	// Visitors must be convex.
	if IsConvex(otherShape.type) == false {
		return true
	}

	// Are sensor events enabled on the other shape?
	if (otherShape.flags & enableSensorEvents) == 0 {
		return true
	}

	// Skip shapes on the same body
	if otherShape.bodyId == sensorShape.bodyId {
		return true
	}

	// Check filter
	if ShouldShapesCollide(sensorShape.filter, otherShape.filter) == false {
		return true
	}

	// Custom user filter
	if (sensorShape.flags & enableCustomFiltering) != 0 || (otherShape.flags & enableCustomFiltering) != 0 {
		customFilterFcn := queryContext.world.customFilterFcn
		if customFilterFcn != nil {
			idA := ShapeId{sensorShapeId + 1, world.worldId, sensorShape.generation}
			idB := ShapeId{shapeId + 1, world.worldId, otherShape.generation}
			shouldCollide := customFilterFcn(idA, idB, queryContext.world.customFilterContext)
			if shouldCollide == false {
				return true
			}
		}
	}

	otherTransform := ToRelativeTransform(GetBodyTransform(world, otherShape.bodyId), Pos_zero)

	overlap := OverlapSensor(sensorShape, queryContext.transform, otherShape, otherTransform)
	if overlap == false {
		return true
	}

	// Record the overlap
	sensor := queryContext.sensor
	shapeRef := Array_Emplace(&sensor.overlaps2)
	shapeRef.shapeId = shapeId
	shapeRef.generation = otherShape.generation

	return true
}

// unified: C sorts with libc qsort and this comparator (never 0, so the order of equal shape ids is up to the
// libc); the port sorts with QSORT (qsort.odin) and a less-than on the shape id, the same on every platform
@(private = "file")
CompareVisitors :: proc(a, b: Visitor) -> bool {
	return a.shapeId < b.shapeId
}
// unified end

@(private = "file")
SensorTask :: proc(startIndex: i32, endIndex: i32, workerIndex: i32, context_: rawptr) {
	world := (^World)(context_)
	assert(i32(workerIndex) < world.workerCount)
	taskContext := &world.sensorTaskContexts.data[workerIndex]

	assert(startIndex < endIndex)

	trees := &world.broadPhase.trees
	for sensorIndex := startIndex; sensorIndex < endIndex; sensorIndex += 1 {
		sensor := Array_Get(&world.sensors, sensorIndex)
		sensorShape := Array_Get(&world.shapes, sensor.shapeId)

		// Swap overlap arrays
		temp := sensor.overlaps1
		sensor.overlaps1 = sensor.overlaps2
		sensor.overlaps2 = temp
		Array_Clear(&sensor.overlaps2)

		// Append sensor hits
		Array_Append(&sensor.overlaps2, sensor.hits.data, sensor.hits.count)

		// Clear the hits
		Array_Clear(&sensor.hits)

		body := Array_Get(&world.bodies, sensorShape.bodyId)
		if body.setIndex == i32(SetType.disabledSet) || (sensorShape.flags & enableSensorEvents) == 0 {
			if sensor.overlaps1.count != 0 {
				// This sensor is dropping all overlaps because it has been disabled.
				SetBit(&taskContext.eventBits, u32(sensorIndex))
			}
			continue
		}

		transform := ToRelativeTransform(GetBodyTransformQuick(world, body), Pos_zero)

		queryContext := SensorQueryContext {
			world       = world,
			taskContext = taskContext,
			sensor      = sensor,
			sensorShape = sensorShape,
			transform   = transform,
		}

		assert(sensorShape.sensorIndex == sensorIndex)
		queryBounds := sensorShape.aabb

		// Query all trees
		DynamicTree_Query(
			&trees[0],
			queryBounds,
			sensorShape.filter.maskBits,
			false,
			SensorQueryCallback,
			&queryContext,
		)
		DynamicTree_Query(
			&trees[1],
			queryBounds,
			sensorShape.filter.maskBits,
			false,
			SensorQueryCallback,
			&queryContext,
		)
		DynamicTree_Query(
			&trees[2],
			queryBounds,
			sensorShape.filter.maskBits,
			false,
			SensorQueryCallback,
			&queryContext,
		)

		// Sort the overlaps to enable finding begin and end events.
		// unified: QSORT instead of libc qsort (see CompareVisitors)
		QSORT(sensor.overlaps2.data, sensor.overlaps2.count, CompareVisitors)
		// unified end

		// Remove duplicates from overlaps2 (sorted). Duplicates are possible due to the hit events appended earlier.
		uniqueCount: i32 = 0
		overlapCount := sensor.overlaps2.count
		overlapData := sensor.overlaps2.data
		for i: i32 = 0; i < overlapCount; i += 1 {
			if uniqueCount == 0 || overlapData[i].shapeId != overlapData[uniqueCount - 1].shapeId {
				overlapData[uniqueCount] = overlapData[i]
				uniqueCount += 1
			}
		}
		sensor.overlaps2.count = uniqueCount

		count1 := sensor.overlaps1.count
		count2 := sensor.overlaps2.count
		if count1 != count2 {
			// something changed
			SetBit(&taskContext.eventBits, u32(sensorIndex))
		} else {
			for i: i32 = 0; i < count1; i += 1 {
				s1 := &sensor.overlaps1.data[i]
				s2 := &sensor.overlaps2.data[i]

				if s1.shapeId != s2.shapeId || s1.generation != s2.generation {
					// something changed
					SetBit(&taskContext.eventBits, u32(sensorIndex))
					break
				}
			}
		}
	}
}

OverlapSensors :: proc(world: ^World) {
	sensorCount := world.sensors.count
	if sensorCount == 0 {
		return
	}

	assert(world.workerCount > 0)

	for i: i32 = 0; i < world.workerCount; i += 1 {
		SetBitCountAndClear(&world.sensorTaskContexts.data[i].eventBits, u32(sensorCount))
	}

	// Parallel-for sensors overlaps
	minRange: i32 = 16
	ParallelFor(world, SensorTask, sensorCount, minRange, world, "sensors")

	bitSet := &world.sensorTaskContexts.data[0].eventBits
	for i: i32 = 1; i < world.workerCount; i += 1 {
		InPlaceUnion(bitSet, &world.sensorTaskContexts.data[i].eventBits)
	}

	// Iterate sensors bits and publish events
	// Process sensor state changes. Iterate over set bits
	bits := bitSet.bits
	blockCount := bitSet.blockCount

	for k: u32 = 0; k < blockCount; k += 1 {
		word := bits[k]
		for word != 0 {
			ctz := CTZ64(word)
			sensorIndex := i32(64 * k + ctz)

			sensor := Array_Get(&world.sensors, sensorIndex)
			sensorShape := Array_Get(&world.shapes, sensor.shapeId)
			sensorId := ShapeId{sensor.shapeId + 1, world.worldId, sensorShape.generation}

			count1 := sensor.overlaps1.count
			count2 := sensor.overlaps2.count
			refs1 := sensor.overlaps1.data
			refs2 := sensor.overlaps2.data

			// overlaps1 can have overlaps that end
			// overlaps2 can have overlaps that begin
			index1, index2: i32 = 0, 0
			for index1 < count1 && index2 < count2 {
				r1 := &refs1[index1]
				r2 := &refs2[index2]
				if r1.shapeId == r2.shapeId {
					if r1.generation < r2.generation {
						// end
						visitorId := ShapeId{r1.shapeId + 1, world.worldId, r1.generation}
						event := SensorEndTouchEvent {
							sensorShapeId  = sensorId,
							visitorShapeId = visitorId,
						}
						Array_Push(&world.sensorEndEvents[world.endEventArrayIndex], event)
						index1 += 1
					} else if r1.generation > r2.generation {
						// begin
						visitorId := ShapeId{r2.shapeId + 1, world.worldId, r2.generation}
						event := SensorBeginTouchEvent{sensorId, visitorId}
						Array_Push(&world.sensorBeginEvents, event)
						index2 += 1
					} else {
						// persisted
						index1 += 1
						index2 += 1
					}
				} else if r1.shapeId < r2.shapeId {
					// end
					visitorId := ShapeId{r1.shapeId + 1, world.worldId, r1.generation}
					event := SensorEndTouchEvent{sensorId, visitorId}
					Array_Push(&world.sensorEndEvents[world.endEventArrayIndex], event)
					index1 += 1
				} else {
					// begin
					visitorId := ShapeId{r2.shapeId + 1, world.worldId, r2.generation}
					event := SensorBeginTouchEvent{sensorId, visitorId}
					Array_Push(&world.sensorBeginEvents, event)
					index2 += 1
				}
			}

			for index1 < count1 {
				// end
				r1 := &refs1[index1]
				visitorId := ShapeId{r1.shapeId + 1, world.worldId, r1.generation}
				event := SensorEndTouchEvent{sensorId, visitorId}
				Array_Push(&world.sensorEndEvents[world.endEventArrayIndex], event)
				index1 += 1
			}

			for index2 < count2 {
				// begin
				r2 := &refs2[index2]
				visitorId := ShapeId{r2.shapeId + 1, world.worldId, r2.generation}
				event := SensorBeginTouchEvent{sensorId, visitorId}
				Array_Push(&world.sensorBeginEvents, event)
				index2 += 1
			}

			// Clear the smallest set bit
			word = word & (word - 1)
		}
	}
}

DestroySensor :: proc(world: ^World, sensorShape: ^Shape) {
	sensor := Array_Get(&world.sensors, sensorShape.sensorIndex)
	for i: i32 = 0; i < sensor.overlaps2.count; i += 1 {
		ref := &sensor.overlaps2.data[i]
		event := SensorEndTouchEvent {
			sensorShapeId = {index1 = sensorShape.id + 1, world0 = world.worldId, generation = sensorShape.generation},
			visitorShapeId = {index1 = ref.shapeId + 1, world0 = world.worldId, generation = ref.generation},
		}

		Array_Push(&world.sensorEndEvents[world.endEventArrayIndex], event)
	}

	// Destroy sensor
	Array_Destroy(&sensor.hits)
	Array_Destroy(&sensor.overlaps1)
	Array_Destroy(&sensor.overlaps2)

	movedIndex := Array_RemoveSwap(&world.sensors, sensorShape.sensorIndex)
	if movedIndex != NULL_INDEX {
		// Fixup moved sensor
		movedSensor := Array_Get(&world.sensors, sensorShape.sensorIndex)
		otherSensorShape := Array_Get(&world.shapes, movedSensor.shapeId)
		otherSensorShape.sensorIndex = sensorShape.sensorIndex
	}
}
