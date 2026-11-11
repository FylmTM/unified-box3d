// Ported from box3d src/timer.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:intrinsics"
import "core:sync"
import "core:thread"
import "core:time"

NAME_LENGTH :: 16

// unified: one implementation on core:time, core:sync and core:thread instead of the Windows, Linux, macOS and
// fallback branches: ticks are core:time's monotonic nanoseconds (the Linux branch), the mutex is a sync.Mutex,
// the semaphore a sync.Sema, the thread a core:thread thread (which applies the name, truncated to the OS limit).
// These are host-side helpers for profiling and the task system; they never affect simulation results.

// The prototypes for these live in core.h.
ThreadFunction :: #type proc(context_: rawptr)

GetTicks :: proc() -> u64 {
	return u64(time.tick_now()._nsec)
}

GetMilliseconds :: proc(ticks: u64) -> f32 {
	ticksNow := GetTicks()
	return f32(f64(ticksNow - ticks) / 1000000.0)
}

GetMillisecondsAndReset :: proc(ticks: ^u64) -> f32 {
	ticksNow := GetTicks()
	ms := f32(f64(ticksNow - ticks^) / 1000000.0)
	ticks^ = ticksNow
	return ms
}

Yield :: proc() {
	thread.yield()
}

Sleep :: proc(milliseconds: i32) {
	time.sleep(time.Duration(milliseconds) * time.Millisecond)
}

Mutex :: struct {
	mtx: sync.Mutex,
}

CreateMutex :: proc() -> ^Mutex {
	m := cast(^Mutex)AllocHost(size_of(Mutex))
	m^ = Mutex{}
	return m
}

DestroyMutex :: proc(m: ^Mutex) {
	m^ = Mutex{}
	Free(m, size_of(Mutex))
}

LockMutex :: proc(m: ^Mutex) {
	sync.mutex_lock(&m.mtx)
}

UnlockMutex :: proc(m: ^Mutex) {
	sync.mutex_unlock(&m.mtx)
}

Semaphore :: struct {
	semaphore: sync.Sema,
}

CreateSemaphore :: proc(initCount: i32) -> ^Semaphore {
	s := cast(^Semaphore)AllocHost(size_of(Semaphore))
	s^ = Semaphore{}
	if initCount > 0 {
		sync.sema_post(&s.semaphore, int(initCount))
	}
	return s
}

DestroySemaphore :: proc(s: ^Semaphore) {
	s^ = Semaphore{}
	Free(s, size_of(Semaphore))
}

WaitSemaphore :: proc(s: ^Semaphore) {
	sync.sema_wait(&s.semaphore)
}

SignalSemaphore :: proc(s: ^Semaphore) {
	sync.sema_post(&s.semaphore)
}

Thread :: struct {
	thread:   ^thread.Thread,
	function: ThreadFunction,
	context_: rawptr,
	name:     [NAME_LENGTH]u8,
}

@(private = "file")
ThreadStart :: proc(th: ^thread.Thread) {
	t := (^Thread)(th.data)
	t.function(t.context_)
}

CreateThread :: proc(function: ThreadFunction, context_: rawptr, name: cstring) -> ^Thread {
	t := cast(^Thread)AllocHost(size_of(Thread))
	t^ = Thread{}
	t.function = function
	t.context_ = context_
	threadName: Maybe(string)
	if name != nil {
		// snprintf( t->name, sizeof( t->name ), "%s", name )
		src := ([^]u8)(name)
		i := 0
		for ; i < NAME_LENGTH - 1 && src[i] != 0; i += 1 {
			t.name[i] = src[i]
		}
		t.name[i] = 0
		threadName = string(t.name[:i])
	} else {
		t.name[0] = 0
	}
	t.thread = thread.create(ThreadStart, .Normal, threadName)
	t.thread.data = t
	thread.start(t.thread)
	return t
}

JoinThread :: proc(t: ^Thread) {
	thread.join(t.thread)
	thread.destroy(t.thread)
	t^ = Thread{}
	Free(t, size_of(Thread))
}
// unified end

// djb2 hash, folded 8 bytes per iteration to shorten the dependency chain.
// memcpy lowers to a single load on most targets; on big-endian we byte-swap so
// the hash value is identical across endianness (preserving cross-platform determinism).
// Equivalent to byte-wise djb2 only in spirit; values differ from the original recurrence.
Hash :: proc "contextless" (hash: u32, data: [^]u8, count: i32) -> u32 {
	result := hash
	i: i32 = 0

	for i + 8 <= count {
		word := intrinsics.unaligned_load((^u64)(&data[i]))
		when ODIN_ENDIAN == .Big {
			word = intrinsics.byte_swap(word)
		}
		result = (result << 5) + result + u32(word)
		result = (result << 5) + result + u32(word >> 32)
		i += 8
	}

	for i < count {
		result = (result << 5) + result + u32(data[i])
		i += 1
	}

	return result
}
