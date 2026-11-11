// Ported from box3d src/scheduler.c and src/scheduler.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:runtime"
import "core:fmt"
import "core:mem"

// unified: b3TaskCallback is declared once, in public_types.odin (types.h has the same typedef).

SchedulerTaskStatus :: enum i32 {
	schedulerFree     = 0,
	schedulerPending  = 1,
	schedulerClaimed  = 2,
	schedulerComplete = 3,
}

SchedulerTask :: struct {
	callback:    TaskCallback,
	taskContext: rawptr,
	status:      AtomicInt,
}

SchedulerWorkerContext :: struct {
	scheduler:   ^Scheduler,
	threadIndex: i32,
}

Scheduler :: struct {
	// unified: the world arena its threads allocate from while they run the world's tasks (world_arena.odin)
	arena:          ^WorldArena,
	// unified end
	threads:        [MAX_WORKERS]^Thread,
	workerContexts: [MAX_WORKERS]SchedulerWorkerContext,

	// total workers including main thread
	workerCount:    i32,

	// threads created = workerCount - 1
	threadCount:    i32,
	tasks:          [MAX_TASKS]SchedulerTask,
	nextSlot:       AtomicInt,
	taskSemaphore:  ^Semaphore,
	shutdown:       AtomicInt,
}

// Try to claim and execute one pending task.
// Returns true if work was performed, false otherwise.
@(private = "file")
SchedulerExecuteOne :: proc(scheduler: ^Scheduler) -> bool {
	taskCount := AtomicLoadInt(&scheduler.nextSlot)
	for t: i32 = 0; t < taskCount; t += 1 {
		task := &scheduler.tasks[t]
		if AtomicLoadInt(&task.status) != i32(SchedulerTaskStatus.schedulerPending) {
			continue
		}

		if AtomicCompareExchangeInt(
			   &task.status,
			   i32(SchedulerTaskStatus.schedulerPending),
			   i32(SchedulerTaskStatus.schedulerClaimed),
		   ) ==
		   false {
			continue
		}

		task.callback(task.taskContext)

		AtomicStoreInt(&task.status, i32(SchedulerTaskStatus.schedulerComplete))
		return true
	}

	return false
}

// Background worker thread entry point.
@(private = "file")
SchedulerWorkerMain :: proc(context_: rawptr) {
	workerContext := (^SchedulerWorkerContext)(context_)
	scheduler := workerContext.scheduler

	for true {
		WaitSemaphore(scheduler.taskSemaphore)

		if AtomicLoadInt(&scheduler.shutdown) != 0 {
			break
		}

		// Claim and execute all available work
		// unified: in the world's arena (world_arena.odin)
		PushArena(scheduler.arena)
		for SchedulerExecuteOne(scheduler) {
		}
		PopArena()
		// unified end
	}
}

CreateScheduler :: proc(workerCount: i32) -> ^Scheduler {
	assert(0 < workerCount && workerCount <= MAX_WORKERS)

	scheduler := cast(^Scheduler)AllocHost(size_of(Scheduler))
	mem.zero(scheduler, size_of(Scheduler))

	scheduler.workerCount = workerCount
	threadCount := workerCount - 1
	scheduler.threadCount = threadCount
	scheduler.taskSemaphore = CreateSemaphore(0)
	AtomicStoreInt(&scheduler.shutdown, 0)
	AtomicStoreInt(&scheduler.nextSlot, 0)

	// Background threads use indices 1..workerCount-1.
	// Main thread uses index 0.
	for i: i32 = 0; i < threadCount; i += 1 {
		scheduler.workerContexts[i].scheduler = scheduler
		scheduler.workerContexts[i].threadIndex = i + 1

		name: [16]u8
		fmt.bprintf(name[:len(name) - 1], "box3d_worker_%02d", i + 1)
		scheduler.threads[i] = CreateThread(SchedulerWorkerMain, &scheduler.workerContexts[i], cstring(&name[0]))
	}

	return scheduler
}

DestroyScheduler :: proc(scheduler: ^Scheduler) {
	AtomicStoreInt(&scheduler.shutdown, 1)

	// Wake all background threads so they see the shutdown flag
	for i: i32 = 0; i < scheduler.threadCount; i += 1 {
		SignalSemaphore(scheduler.taskSemaphore)
	}

	for i: i32 = 0; i < scheduler.threadCount; i += 1 {
		JoinThread(scheduler.threads[i])
		scheduler.threads[i] = nil
	}

	DestroySemaphore(scheduler.taskSemaphore)
	Free(scheduler, size_of(Scheduler))
}

ResetScheduler :: proc(scheduler: ^Scheduler) {
	AtomicStoreInt(&scheduler.nextSlot, 0)
}

// See b3EnqueueTaskCallback and b3FinishTaskCallback
// unified: C calling convention (the library's API is upstream's C ABI)
SchedulerEnqueueTask :: proc "c" (
	task: TaskCallback,
	taskContext: rawptr,
	userContext: rawptr,
	name: cstring,
) -> rawptr {
	context = runtime.default_context()
	// unified end
	_ = name
	scheduler := (^Scheduler)(userContext)

	slot := AtomicFetchAddInt(&scheduler.nextSlot, 1)
	assert(slot < MAX_TASKS)

	schedulerTask := &scheduler.tasks[slot]
	schedulerTask.callback = task
	schedulerTask.taskContext = taskContext

	// Memory fence: status must be published after callback and context are written
	AtomicStoreInt(&schedulerTask.status, i32(SchedulerTaskStatus.schedulerPending))

	// One wake per enqueue is enough: at most one worker picks up each task.
	SignalSemaphore(scheduler.taskSemaphore)

	return schedulerTask
}

// unified: C calling convention (the library's API is upstream's C ABI)
SchedulerFinishTask :: proc "c" (userTask: rawptr, userContext: rawptr) {
	context = runtime.default_context()
	// unified end
	if userTask == nil {
		return
	}

	scheduler := (^Scheduler)(userContext)
	waitTask := (^SchedulerTask)(userTask)

	// Main thread helps execute any available work while waiting for the
	// target task to complete. This keeps the main thread from idling when
	// background threads are busy on other tasks from the same phase.
	for AtomicLoadInt(&waitTask.status) != i32(SchedulerTaskStatus.schedulerComplete) {
		if SchedulerExecuteOne(scheduler) == false {
			Yield()
		}
	}
}
