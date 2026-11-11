// Ported from box3d include/box3d/id.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

// Note: this file should be stand-alone

// Ids
// These ids serve as handles to internal Box3D objects.
// These should be considered opaque data and passed by value.
// Include this header if you need the id types and not the whole Box3D API.
// All ids are considered null if initialized to zero.
//
// For example:
//
// worldId := WorldId{}
//
// This is considered null.
//
// @warning Do not use the internals of these ids. They are subject to change. Ids should be treated as opaque objects.
// @warning You should use ids to access objects in Box3D. Do not access files within the src folder. Such usage is unsupported.

// World id references a world instance. This should be treated as an opaque handle.
WorldId :: struct {
	index1:     u16,
	generation: u16,
}

// Body id references a body instance. This should be treated as an opaque handle.
BodyId :: struct {
	index1:     i32,
	world0:     u16,
	generation: u16,
}

// Shape id references a shape instance. This should be treated as an opaque handle.
ShapeId :: struct {
	index1:     i32,
	world0:     u16,
	generation: u16,
}

// Joint id references a joint instance. This should be treated as an opaque handle.
JointId :: struct {
	index1:     i32,
	world0:     u16,
	generation: u16,
}

// Contact id references a contact instance. This should be treated as an opaque handle.
ContactId :: struct {
	index1:     i32,
	world0:     u16,
	padding:    i16,
	generation: u32,
}

// Use these to make your identifiers null.
// You may also use zero initialization to get null.
nullWorldId :: WorldId{}
nullBodyId :: BodyId{}
nullShapeId :: ShapeId{}
nullJointId :: JointId{}
nullContactId :: ContactId{}

// Macro to determine if any id is null.
IS_NULL :: #force_inline proc(id: $T) -> bool {
	return id.index1 == 0
}

// Macro to determine if any id is non-null.
IS_NON_NULL :: #force_inline proc(id: $T) -> bool {
	return id.index1 != 0
}

// Compare two ids for equality. Doesn't work for b3WorldId. Don't mix types.
ID_EQUALS :: #force_inline proc(id1, id2: $T) -> bool {
	return id1.index1 == id2.index1 && id1.world0 == id2.world0 && id1.generation == id2.generation
}

// Store a world id into a uint32_t.
StoreWorldId :: proc(id: WorldId) -> u32 {
	return (u32(id.index1) << 16) | u32(id.generation)
}

// Load a uint32_t into a world id.
LoadWorldId :: proc(x: u32) -> WorldId {
	id := WorldId{u16(x >> 16), u16(x)}
	return id
}

// Store a body id into a uint64_t.
StoreBodyId :: proc(id: BodyId) -> u64 {
	return (u64(id.index1) << 32) | (u64(id.world0)) << 16 | u64(id.generation)
}

// Load a uint64_t into a body id.
LoadBodyId :: proc(x: u64) -> BodyId {
	id := BodyId{i32(x >> 32), u16(x >> 16), u16(x)}
	return id
}

// Store a shape id into a uint64_t.
StoreShapeId :: proc(id: ShapeId) -> u64 {
	return (u64(id.index1) << 32) | (u64(id.world0)) << 16 | u64(id.generation)
}

// Load a uint64_t into a shape id.
LoadShapeId :: proc(x: u64) -> ShapeId {
	id := ShapeId{i32(x >> 32), u16(x >> 16), u16(x)}
	return id
}

// Store a joint id into a uint64_t.
StoreJointId :: proc(id: JointId) -> u64 {
	return (u64(id.index1) << 32) | (u64(id.world0)) << 16 | u64(id.generation)
}

// Load a uint64_t into a joint id.
LoadJointId :: proc(x: u64) -> JointId {
	id := JointId{i32(x >> 32), u16(x >> 16), u16(x)}
	return id
}

// Store a contact id into three uint32 values
StoreContactId :: proc(id: ContactId, values: ^[3]u32) {
	values[0] = u32(id.index1)
	values[1] = u32(id.world0)
	values[2] = u32(id.generation)
}

// Load a contact id from three uint32 values.
LoadContactId :: proc(values: ^[3]u32) -> ContactId {
	id: ContactId
	id.index1 = i32(values[0])
	id.world0 = u16(values[1])
	id.padding = 0
	id.generation = u32(values[2])
	return id
}
