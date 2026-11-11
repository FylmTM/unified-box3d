// Ported from box3d src/id_pool.c and src/id_pool.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

IdPool :: struct {
	freeArray: Array(i32),
	nextIndex: i32,
}

GetIdCount :: proc(pool: ^IdPool) -> i32 {
	return pool.nextIndex - pool.freeArray.count
}

GetIdCapacity :: proc(pool: ^IdPool) -> i32 {
	return pool.nextIndex
}

GetIdBytes :: proc(pool: ^IdPool) -> i32 {
	return Array_ByteCount(&pool.freeArray)
}

CreateIdPool :: proc() -> IdPool {
	pool := IdPool{}
	Array_Reserve(&pool.freeArray, 32)
	return pool
}

DestroyIdPool :: proc(pool: ^IdPool) {
	Array_Destroy(&pool.freeArray)
	pool^ = IdPool{}
}

AllocId :: proc(pool: ^IdPool) -> i32 {
	count := pool.freeArray.count
	if count > 0 {
		id := Array_Pop(&pool.freeArray)
		return id
	}

	id := pool.nextIndex
	pool.nextIndex += 1
	return id
}

FreeId :: proc(pool: ^IdPool, id: i32) {
	assert(pool.nextIndex > 0)
	assert(0 <= id && id < pool.nextIndex)

	// todo does not work with assertion above
	// should probably be `id == pool->nextIndex - 1`
	if id == pool.nextIndex {
		pool.nextIndex -= 1
		return
	}

	Array_Push(&pool.freeArray, id)
}

when ENABLE_VALIDATION {

	ValidateFreeId :: proc(pool: ^IdPool, id: i32) {
		freeCount := pool.freeArray.count
		for i: i32 = 0; i < freeCount; i += 1 {
			if pool.freeArray.data[i] == id {
				return
			}
		}

		assert(false)
	}

} else {

	ValidateFreeId :: proc(pool: ^IdPool, id: i32) {
		_ = pool
		_ = id
	}

}
