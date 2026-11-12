// unified: the world arena (SPEC.md, phase 2 items 1 and 2). Not upstream.
//
// Everything a world keeps between steps is allocated from one arena per world. The arena reserves its address range
// when the world is created and never moves, so in-process save and restore are a memcpy of the World struct and the
// arena's used bytes, and every pointer inside stays valid. The allocator is deterministic (segregated free lists that split
// larger free blocks, then a bump pointer), and its state lives at the start of the arena, so a restore brings it back
// too. Memory that host-side fields point to (the scratch stack, the scheduler, recordings, the hull database, the
// name cache, image buffers) is allocated with AllocHost and never comes from the arena.
//
// Which arena an Alloc uses: the world lookups (GetWorld, GetWorldFromId, ...) push the world's arena for the rest
// of the calling scope (@(deferred_out)), and the built-in scheduler's threads push it while they run the world's
// tasks. Outside any world's scope, Alloc uses the heap. Free routes by address.
package box3d

import "base:intrinsics"
import "core:mem"
import "core:mem/virtual"
import "core:sync"

// Address space reserved per world; pages are committed as the arena grows.
ARENA_RESERVE :: #config(BOX3D_ARENA_RESERVE, 1 << 32)
@(private = "file")
ARENA_COMMIT_STEP :: 1 << 20
@(private = "file")
ARENA_SUBCLASSES :: 4 // size classes per doubling, from 128 bytes up (64 and 128 bytes are single classes)
@(private = "file")
ARENA_CLASS_COUNT :: 2 + 28 * ARENA_SUBCLASSES
@(private = "file")
ARENA_BLOCK_HEADER :: ALIGNMENT // keeps every payload 32-byte aligned
@(private = "file")
ARENA_MAGIC :: 0xB3A7E4A0

// Lives at offset 0 of the arena, so it is saved and restored with the bytes.
@(private = "file")
ArenaHeader :: struct {
	top:       u64, // bump offset
	freeLists: [ARENA_CLASS_COUNT]u64, // offset of the first free block per class, 0 when empty
	nonEmpty:  [(ARENA_CLASS_COUNT + 63) / 64]u64, // a bit per class with free blocks
}

@(private = "file")
BlockHeader :: struct {
	magic:     u32,
	_:         u32,
	next:      u64, // free-list link (offset) while the block is free
	size:      u64, // requested bytes while in use; 0 while free
	blockSize: u64, // the whole block, header included
}
#assert(size_of(BlockHeader) == ARENA_BLOCK_HEADER)

// Host-side state of a world's arena (in the World struct, kept as is by a restore).
WorldArena :: struct {
	base:      [^]u8,
	reserved:  int,
	committed: int,
	mutex:     sync.Mutex, // worker threads allocate manifolds concurrently
	debugFill: i32, // -1: none; else every new block's payload is filled with this byte (tests)
}

// Fill pattern for the arenas of worlds created from now on (-1: none). For tests that prove image bytes don't
// depend on what fresh memory held.
arenaDebugFill: i32 = -1

@(private = "file")
header :: #force_inline proc "contextless" (arena: ^WorldArena) -> ^ArenaHeader {
	return (^ArenaHeader)(arena.base)
}

// Every world's arena sits in one address range reserved once (MAX_WORLDS × ARENA_RESERVE, address space only),
// world i's at slot i, so the arena that holds a pointer is a range check and a division (Free, AllocNear).
@(private = "file")
arenaRegion: uintptr
@(private = "file")
arenaRegionOnce: sync.Once

@(private = "file")
ReserveArenaRegion :: proc() {
	data, err := virtual.reserve(uint(MAX_WORLDS) * uint(ARENA_RESERVE))
	assert(err == nil, "Box3D: reserving the world arenas' address space failed (lower BOX3D_ARENA_RESERVE)")
	sync.atomic_store(&arenaRegion, uintptr(raw_data(data)))
}

// The world index whose arena slot holds `p`, or -1.
ArenaIndexOf :: #force_inline proc "contextless" (p: rawptr) -> int {
	region := sync.atomic_load_explicit(&arenaRegion, .Relaxed)
	if region == 0 || uintptr(p) < region {
		return -1
	}
	index := int((uintptr(p) - region) / ARENA_RESERVE)
	return index < MAX_WORLDS ? index : -1
}

CreateWorldArena :: proc(arena: ^WorldArena, worldIndex: i32) {
	sync.once_do(&arenaRegionOnce, ReserveArenaRegion)
	arena.base = ([^]u8)(arenaRegion + uintptr(worldIndex) * ARENA_RESERVE)
	arena.reserved = ARENA_RESERVE
	arena.committed = 0
	arena.debugFill = arenaDebugFill
	ArenaCommit(arena, size_of(ArenaHeader))
	h := header(arena)
	h^ = {}
	h.top = u64(mem.align_forward_int(size_of(ArenaHeader), ARENA_BLOCK_HEADER))
}

// Gives the world's pages back; its slot of the address range stays reserved for the next world there.
DestroyWorldArena :: proc(arena: ^WorldArena) {
	if arena.base != nil && arena.committed > 0 {
		virtual.decommit(arena.base, uint(arena.committed))
	}
	arena^ = {}
}

@(private = "file")
ArenaCommit :: proc(arena: ^WorldArena, end: int) {
	if end <= arena.committed {
		return
	}
	newCommitted := mem.align_forward_int(end, ARENA_COMMIT_STEP)
	assert(newCommitted <= arena.reserved, "Box3D: the world arena is full (raise BOX3D_ARENA_RESERVE)")
	err := virtual.commit(&arena.base[arena.committed], uint(newCommitted - arena.committed))
	assert(err == nil, "Box3D: committing world arena memory failed")
	arena.committed = newCommitted
}

ArenaContains :: #force_inline proc "contextless" (arena: ^WorldArena, p: rawptr) -> bool {
	a := uintptr(arena.base)
	return a <= uintptr(p) && uintptr(p) < a + uintptr(arena.reserved)
}

ArenaUsedBytes :: proc(arena: ^WorldArena) -> int {
	return int(header(arena).top)
}

// Free lists are segregated by size class: 64, 128, then four per doubling (160, 192, 224, 256, 320, ...). A free
// block sits in the largest class that fits inside it; an allocation looks from the smallest class that surely fits
// up, splits off what it doesn't need (a new free block), and only then bumps the top. Sizes are multiples of 32.
ArenaClassOf :: proc "contextless" (need: int) -> u32 {
	if need <= 64 {
		return 0
	}
	if need <= 128 {
		return 1
	}
	octave := u32(63 - intrinsics.count_leading_zeros(u64(need - 1))) // base = 1 << octave, base < need <= 2*base
	base := 1 << octave
	step := base / ARENA_SUBCLASSES
	sub := u32((need - base + step - 1) / step) // 1..4
	return 2 + (octave - 7) * ARENA_SUBCLASSES + sub - 1
}

ArenaClassSize :: proc "contextless" (class: u32) -> int {
	if class == 0 {
		return 64
	}
	if class == 1 {
		return 128
	}
	octave := (class - 2) / ARENA_SUBCLASSES + 7
	sub := (class - 2) % ARENA_SUBCLASSES + 1
	base := 1 << octave
	return base + int(sub) * (base / ARENA_SUBCLASSES)
}

// The largest class whose size is at most `blockSize` (where a free block of that size is listed).
@(private = "file")
FloorClass :: proc "contextless" (blockSize: int) -> u32 {
	class := ArenaClassOf(blockSize)
	if ArenaClassSize(class) > blockSize {
		class -= 1
	}
	return class
}

@(private = "file")
PushFree :: proc "contextless" (arena: ^WorldArena, offset: u64, blockSize: int) {
	h := header(arena)
	block := (^BlockHeader)(&arena.base[offset])
	class := FloorClass(blockSize)
	block^ = {
		magic     = ARENA_MAGIC,
		next      = h.freeLists[class],
		blockSize = u64(blockSize),
	}
	h.freeLists[class] = offset
	h.nonEmpty[class / 64] |= 1 << (class % 64)
}

@(private = "file")
PopFree :: proc "contextless" (arena: ^WorldArena, class: u32) -> u64 {
	h := header(arena)
	offset := h.freeLists[class]
	block := (^BlockHeader)(&arena.base[offset])
	h.freeLists[class] = block.next
	if block.next == 0 {
		h.nonEmpty[class / 64] &~= 1 << (class % 64)
	}
	return offset
}

// The first non-empty free list at or above `class`, or ARENA_CLASS_COUNT.
@(private = "file")
FirstFree :: proc "contextless" (arena: ^WorldArena, class: u32) -> u32 {
	h := header(arena)
	for word := class / 64; word < len(h.nonEmpty); word += 1 {
		bits := h.nonEmpty[word]
		if word == class / 64 {
			bits &= ~u64(0) << (class % 64)
		}
		if bits != 0 {
			return word * 64 + u32(intrinsics.count_trailing_zeros(bits))
		}
	}
	return ARENA_CLASS_COUNT
}

ArenaAlloc :: proc(arena: ^WorldArena, size: int) -> rawptr {
	need := mem.align_forward_int(size + ARENA_BLOCK_HEADER, ARENA_BLOCK_HEADER)
	class := ArenaClassOf(need)
	assert(class < ARENA_CLASS_COUNT)

	sync.mutex_lock(&arena.mutex)
	h := header(arena)
	offset: u64
	blockSize: int
	if found := FirstFree(arena, class); found < ARENA_CLASS_COUNT {
		offset = PopFree(arena, found)
		blockSize = int((^BlockHeader)(&arena.base[offset]).blockSize)
		assert(blockSize >= need)
		if blockSize - need >= 64 {
			PushFree(arena, offset + u64(need), blockSize - need)
			blockSize = need
		}
	} else {
		offset = h.top
		blockSize = need
		ArenaCommit(arena, int(offset) + blockSize)
		h.top = offset + u64(blockSize)
	}
	block := (^BlockHeader)(&arena.base[offset])
	block^ = {
		magic     = ARENA_MAGIC,
		size      = u64(size),
		blockSize = u64(blockSize),
	}
	sync.mutex_unlock(&arena.mutex)

	payload := rawptr(&arena.base[offset + ARENA_BLOCK_HEADER])
	if arena.debugFill >= 0 {
		mem.set(payload, u8(arena.debugFill), size)
	}
	return payload
}

ArenaFree :: proc(arena: ^WorldArena, p: rawptr) {
	block := (^BlockHeader)(uintptr(p) - ARENA_BLOCK_HEADER)
	assert(block.magic == ARENA_MAGIC && block.size != 0, "Box3D: freeing memory that is not a live arena block")
	sync.mutex_lock(&arena.mutex)
	PushFree(arena, u64(uintptr(block) - uintptr(arena.base)), int(block.blockSize))
	sync.mutex_unlock(&arena.mutex)
}

ArenaClassStats :: struct {
	live, free:                           i32, // blocks
	liveBytes, freeBytes, requestedBytes: int,
}

// Walks the arena's blocks: per size class (the largest that fits a block), live and free blocks, their bytes, and
// the bytes the live ones asked for.
ArenaStats :: proc(arena: ^WorldArena) -> (stats: [ARENA_CLASS_COUNT]ArenaClassStats, top: int) {
	h := header(arena)
	offset := u64(mem.align_forward_int(size_of(ArenaHeader), ARENA_BLOCK_HEADER))
	for offset < h.top {
		block := (^BlockHeader)(&arena.base[offset])
		assert(block.magic == ARENA_MAGIC)
		s := &stats[FloorClass(int(block.blockSize))]
		if block.size != 0 {
			s.live += 1
			s.liveBytes += int(block.blockSize)
			s.requestedBytes += int(block.size)
		} else {
			s.free += 1
			s.freeBytes += int(block.blockSize)
		}
		offset += block.blockSize
	}
	return stats, int(h.top)
}

// The per-thread stack of arenas Alloc uses (innermost last).
@(private = "file", thread_local)
arenaStack: [32]^WorldArena
@(private = "file", thread_local)
arenaDepth: i32

// The arena Alloc uses: the innermost pushed one, unless its world has since been destroyed (a caller outside the
// engine that looked a world up keeps it pushed until the end of its scope).
CurrentArena :: #force_inline proc "contextless" () -> ^WorldArena {
	if arenaDepth <= 0 || arenaDepth > len(arenaStack) {
		return nil
	}
	arena := arenaStack[arenaDepth - 1]
	return arena != nil && arena.base != nil ? arena : nil
}

PushArena :: proc "contextless" (arena: ^WorldArena) {
	if arenaDepth < len(arenaStack) {
		arenaStack[arenaDepth] = arena
	}
	arenaDepth += 1
}

PopArena :: proc "contextless" () {
	arenaDepth -= 1
}

// Until the end of the caller's scope, Alloc uses the host: host-side structures that grow inside a world's scope.
@(deferred_none = PopArena)
HostScope :: proc "contextless" () {
	PushArena(nil)
}

// The world lookups push their world's arena (none while the world has no arena, during creation).
EnterWorld :: proc "contextless" (world: ^World) {
	PushArena(world.arena.base != nil ? &world.arena : nil)
}

// For @(deferred_out) on the world lookups: undo their PushArena at the end of the caller's scope.
LeaveWorld :: proc "contextless" (world: ^World) {
	_ = world
	PopArena()
}

// In-process snapshots: the World struct, then the arena's used bytes.
WorldSnapshot :: struct {
	data:     [^]u8,
	size:     int,
	capacity: int,
}

DestroyWorldSnapshot :: proc(snapshot: ^WorldSnapshot) {
	FreeHost(snapshot.data, snapshot.capacity)
	snapshot^ = {}
}

// A save copies the arena's used bytes in a few large ranges: everything below the top except the payloads of free
// blocks of ARENA_SKIP_MIN bytes or more (found from the free lists of their classes, at most ARENA_SKIP_MAX of the
// largest). Walking every block to copy only live bytes costs a cache miss per block header, more than copying the
// small free blocks and the rounding past each request. What a snapshot holds for the skipped payloads doesn't
// matter: nothing reads a free payload before an allocation writes it.
@(private = "file")
ARENA_SKIP_MIN :: 4096
@(private = "file")
ARENA_SKIP_MAX :: 32

// The arena's byte ranges a snapshot leaves out: [begin, end) offsets, sorted.
@(private = "file")
ArenaSkips :: struct {
	count:  int,
	ranges: [ARENA_SKIP_MAX][2]u64,
}

@(private = "file")
ArenaFindSkips :: proc(arena: ^WorldArena) -> (skips: ArenaSkips) {
	h := header(arena)
	for class := FloorClass(ARENA_SKIP_MIN + ARENA_BLOCK_HEADER) + 1; class < ARENA_CLASS_COUNT; class += 1 {
		for offset := h.freeLists[class]; offset != 0; {
			block := (^BlockHeader)(&arena.base[offset])
			r := [2]u64{offset + ARENA_BLOCK_HEADER, offset + block.blockSize}
			// Keep the largest: replace the smallest when full.
			if skips.count < ARENA_SKIP_MAX {
				skips.ranges[skips.count] = r
				skips.count += 1
			} else {
				smallest := 0
				for i in 1 ..< ARENA_SKIP_MAX {
					si, ss := skips.ranges[i], skips.ranges[smallest]
					if si[1] - si[0] < ss[1] - ss[0] {
						smallest = i
					}
				}
				ss := skips.ranges[smallest]
				if r[1] - r[0] > ss[1] - ss[0] {
					skips.ranges[smallest] = r
				}
			}
			offset = block.next
		}
	}
	// Sorted by offset (insertion sort: a few dozen at most).
	for i in 1 ..< skips.count {
		r := skips.ranges[i]
		j := i
		for j > 0 && skips.ranges[j - 1][0] > r[0] {
			skips.ranges[j] = skips.ranges[j - 1]
			j -= 1
		}
		skips.ranges[j] = r
	}
	return
}

// Copies the world into the snapshot. Call between steps.
World_SaveSnapshot :: proc(worldId: WorldId, snapshot: ^WorldSnapshot) {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}
	skips := ArenaFindSkips(&world.arena)
	used := ArenaUsedBytes(&world.arena)
	need := size_of(World) + size_of(ArenaSkips) + used
	if need > snapshot.capacity {
		FreeHost(snapshot.data, snapshot.capacity)
		snapshot.capacity = need + need / 4
		snapshot.data = cast([^]u8)AllocHost(snapshot.capacity)
	}
	mem.copy_non_overlapping(snapshot.data, world, size_of(World))
	snapshot.size = size_of(World) + ArenaPack(&world.arena, &skips, &snapshot.data[size_of(World)])
}

// Copies the arena's state into `dst` (room for size_of(ArenaSkips) + ArenaUsedBytes): the skipped ranges, then the
// bytes below the top outside them. Returns the bytes written.
@(private = "file")
ArenaPack :: proc(arena: ^WorldArena, skips: ^ArenaSkips, dst: [^]u8) -> int {
	mem.copy_non_overlapping(dst, skips, size_of(ArenaSkips))
	n := size_of(ArenaSkips)
	from := u64(0)
	for i in 0 ..< skips.count {
		r := skips.ranges[i]
		mem.copy_non_overlapping(&dst[n], &arena.base[from], int(r[0] - from))
		n += int(r[0] - from)
		from = r[1]
	}
	top := header(arena).top
	mem.copy_non_overlapping(&dst[n], &arena.base[from], int(top - from))
	return n + int(top - from)
}

// The inverse of ArenaPack: puts the packed bytes back at the same offsets.
@(private = "file")
ArenaUnpack :: proc(arena: ^WorldArena, src: [^]u8, size: int) {
	skips := (^ArenaSkips)(src)
	n := size_of(ArenaSkips)
	from := u64(0)
	for i in 0 ..< skips.count {
		r := skips.ranges[i]
		mem.copy_non_overlapping(&arena.base[from], &src[n], int(r[0] - from))
		n += int(r[0] - from)
		from = r[1]
	}
	mem.copy_non_overlapping(&arena.base[from], &src[n], size - n)
	assert(header(arena).top == from + u64(size - n))
}

// Puts the world back to the snapshot's state. The snapshot must come from this world (any earlier tick).
World_RestoreSnapshot :: proc(worldId: WorldId, snapshot: ^WorldSnapshot) -> bool {
	world := GetUnlockedWorldFromId(worldId)
	if world == nil || snapshot.size < size_of(World) {
		return false
	}
	saved := (^World)(snapshot.data)
	if saved.worldId != world.worldId || saved.generation != world.generation {
		return false
	}

	// Host-side fields keep their current values: they point outside the arena (or are the host's settings).
	host := world^
	mem.copy_non_overlapping(world, snapshot.data, size_of(World))
	world.arena = host.arena
	world.stack = host.stack
	world.taskContexts = host.taskContexts
	world.sensorTaskContexts = host.sensorTaskContexts
	world.manifoldAllocatorMutex = host.manifoldAllocatorMutex
	world.hullDatabase = host.hullDatabase
	world.names = host.names
	world.createDebugShape = host.createDebugShape
	world.destroyDebugShape = host.destroyDebugShape
	world.userDebugShapeContext = host.userDebugShapeContext
	world.frictionCallback = host.frictionCallback
	world.restitutionCallback = host.restitutionCallback
	world.profile = host.profile
	world.preSolveFcn = host.preSolveFcn
	world.preSolveContext = host.preSolveContext
	world.customFilterFcn = host.customFilterFcn
	world.customFilterContext = host.customFilterContext
	world.workerCount = host.workerCount
	world.simdWidth = host.simdWidth
	world.enqueueTaskFcn = host.enqueueTaskFcn
	world.finishTaskFcn = host.finishTaskFcn
	world.userTaskContext = host.userTaskContext
	world.userTreeTask = host.userTreeTask
	world.scheduler = host.scheduler
	world.userData = host.userData
	world.recording = host.recording

	ArenaUnpack(&world.arena, &snapshot.data[size_of(World)], snapshot.size - size_of(World))
	return true
}
