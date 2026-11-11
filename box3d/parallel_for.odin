// Ported from box3d src/parallel_for.c and src/parallel_for.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:runtime"

// Callback invoked by b3ParallelFor to process a range of items. May be called
// multiple times per worker: work is divided into blocks that workers claim
// atomically, so a worker that finishes early picks up the next unclaimed
// block instead of sitting idle. workerIndex is the worker identity and is
// stable across all invocations from the same worker, so it is safe to use as
// an index into per-worker state (e.g. world->taskContexts.data + workerIndex).
ParallelForCallback :: #type proc(startIndex: i32, endIndex: i32, workerIndex: i32, context_: rawptr)

// Shared state for one b3ParallelFor invocation. Workers race on nextBlock to
// claim work, so a slow chunk can't strand the other threads.
ParallelForShared :: struct {
	nextBlock:  AtomicInt,
	blockCount: i32,
	blockSize:  i32,
	itemCount:  i32,
	callback:   ParallelForCallback,
	context_:   rawptr,
}

ParallelForTask :: struct {
	shared:      ^ParallelForShared,
	workerIndex: i32,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
ParallelForTrampoline :: proc "c" (taskContext: rawptr) {
	context = runtime.default_context()
	// unified end
	task := (^ParallelForTask)(taskContext)
	shared := task.shared
	workerIndex := task.workerIndex
	context_ := shared.context_
	callback := shared.callback

	blockCount := shared.blockCount
	blockSize := shared.blockSize
	itemCount := shared.itemCount

	for {
		blockIndex := AtomicFetchAddInt(&shared.nextBlock, 1)
		if blockIndex >= blockCount {
			break
		}

		start := blockIndex * blockSize
		end := start + blockSize
		if end > itemCount {
			end = itemCount
		}

		callback(start, end, workerIndex, context_)
	}
}

// Divide [0, itemCount) into blocks and process them with cooperative claiming:
// up to world->workerCount tasks are enqueued, and each task loops, atomically
// claiming the next unclaimed block until the range is drained. Blocks the
// caller until all work is complete. minRange is the minimum block size; block
// size grows once itemCount exceeds 4 * workerCount * minRange so block count
// stays bounded.
ParallelFor :: proc(
	world: ^World,
	callback: ParallelForCallback,
	itemCount: i32,
	minRange: i32,
	context_: rawptr,
	name: cstring,
) {
	if itemCount <= 0 {
		return
	}

	assert(minRange > 0)

	workerCount := world.workerCount
	assert(0 < workerCount && workerCount <= MAX_WORKERS)

	// Target multiple blocks per worker to reduce thread stalls.
	// block size grows once items exceed maxBlockCount * minRange
	// so the block count stays bounded and per-block sync overhead stays low.
	// Benchmarking shows 32 is optimal for the convex pile benchmark and others.
	// This is a lot different than Box2D because the 3D narrow phase cost
	// varies a lot.
	blocksPerWorker: i32 = 32
	maxBlockCount := blocksPerWorker * workerCount

	blockSize: i32
	blockCount: i32
	if itemCount <= minRange * maxBlockCount {
		blockSize = minRange
		blockCount = (itemCount + blockSize - 1) / blockSize
	} else {
		blockSize = (itemCount + maxBlockCount - 1) / maxBlockCount
		blockCount = (itemCount + blockSize - 1) / blockSize
	}
	assert(blockCount >= 1)
	assert(blockSize * blockCount >= itemCount)

	// No point enqueueing more tasks than blocks.
	taskCount := workerCount < blockCount ? workerCount : blockCount

	shared: ParallelForShared
	shared.blockCount = blockCount
	shared.blockSize = blockSize
	shared.itemCount = itemCount
	shared.callback = callback
	shared.context_ = context_
	AtomicStoreInt(&shared.nextBlock, 0)

	tasks: [MAX_WORKERS]ParallelForTask
	handles: [MAX_WORKERS]rawptr
	for i: i32 = 0; i < taskCount; i += 1 {
		tasks[i].shared = &shared
		tasks[i].workerIndex = i

		if world.taskCount < MAX_TASKS {
			handles[i] = world.enqueueTaskFcn(ParallelForTrampoline, &tasks[i], world.userTaskContext, name)
			world.taskCount += 1
		} else {
			handles[i] = nil
			ParallelForTrampoline(&tasks[i])
		}
	}

	for i: i32 = 0; i < taskCount; i += 1 {
		if handles[i] != nil {
			world.finishTaskFcn(handles[i], world.userTaskContext)
		}
	}
}
