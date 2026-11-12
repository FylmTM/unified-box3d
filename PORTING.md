# Porting Box3D to Odin: the rules

The port is `box3d/` (package `box3d`). Upstream: https://github.com/erincatto/box3d at
`336263320469e5e9febfe18697ab562488f7e97a` (`UPSTREAM_COMMIT` in `tools/main.odin`; `./b3 upstream` checks it out in
`.out/upstream/src/`).

Goal: a direct, file-by-file port, so an upstream diff can be applied file by file. When in doubt, do what the C
does, in the same order, with the same names. Upstream's tests, ported 1:1 to `tests/upstream/`, must pass.

## Files

- One Odin file per upstream `.c` file, same base name: `src/body.c` → `body.odin`. Its `.h` (`src/body.h`) goes at
  the top of the same file, in the header's order, then the `.c` body in the `.c`'s order.
- A header with no `.c` gets its own file: `src/math_internal.h` → `math_internal.odin`, `src/simd.h` →
  `simd.odin`.
- Public headers `include/box3d/<name>.h` → `public_<name>.odin` (`types.h` → `public_types.odin`). Their
  implementations stay in the `.c` file that has them (`b3CreateBody` stays in `body.odin`).
- Every file starts with:

  ```odin
  // Ported from box3d src/body.c and src/body.h at 3362633 (PORTING.md).
  // SPDX-FileCopyrightText: 2025 Erin Catto
  // SPDX-License-Identifier: MIT
  package box3d
  ```

  (copy the year from the upstream file's SPDX line).
- Keep upstream comments (they are the documentation), trimmed of Doxygen markup (`@brief`, `@param` lines may stay
  as plain text). Drop `#include`s, `#pragma once`, include guards and `clang-format` markers.

## Names

- Drop the `b3`/`b3_`/`B3_` prefix, keep the rest exactly: `b3CreateBody` → `CreateBody`, `b3BodySim` → `BodySim`,
  `b3MakeRot` → `MakeRot`, `B3_HUGE` → `HUGE`, `B3_NULL_INDEX` → `NULL_INDEX`, `b3_lengthUnitsPerMeter` →
  `lengthUnitsPerMeter`. Field and parameter names unchanged.
- If a stripped name collides with an Odin keyword or builtin, or with another declaration in the package, append
  `_` (`b3Foo` → `Foo_`) and add a line to the renames list at the end of this file.
- `static` functions and file-level `static` variables: same name, `@(private = "file")`. Everything else is
  package-public (Odin has no headers; the supported API is listed in the SPEC, internals stay reachable for the
  white-box upstream tests).

## Types

| C | Odin |
|---|---|
| `float` / `double` | `f32` / `f64` |
| `int`, `int32_t` | `i32` (never Odin `int`: keep C's widths, overflow and layouts) |
| `unsigned`, `uint32_t`, `uint16_t`, `uint8_t`, `int16_t`, `int64_t`, `uint64_t` | `u32`, `u16`, `u8`, `i16`, `i64`, `u64` |
| `size_t`, `ptrdiff_t`, `intptr_t`/`uintptr_t` | `int`, `int`, `int`/`uintptr` |
| `char` (text) / `const char*` | `u8` / `cstring` (C strings stay `cstring`) |
| `T*` (one object) / `T*` (array) / `void*` | `^T` / `[^]T` / `rawptr` |
| `T a[N]` field | `a: [N]T` |
| `struct` | `struct` with the same fields in the same order; `_Alignas(N)` → `#align(N)` |
| `union` | `struct #raw_union` |
| `enum` with explicit values or used as an int | `enum i32 { staticBody = 0, ... }` with the `b3_` prefix dropped from members; members are written `.staticBody` or `BodyType.staticBody`. Where C does integer math with enum values or indexes arrays with them, cast explicitly (`i32(x)`). |
| enum used as flags (`b3_xxxFlag = 0x0001`) | keep as integer constants (`xxxFlag :: 0x0001`) and a `u32`/`u64` field, as C does; don't convert to `bit_set` |
| function pointer typedef `typedef float b3FrictionCallback(...)` (public headers) | `FrictionCallback :: #type proc "c" (...) -> f32`: the C calling convention, since the library exports upstream's C API (marked `// unified:`). A proc assigned to one is `proc "c"` and sets `context = runtime.default_context()` first if it calls Odin procs (internal callbacks that only touch memory skip it: they run per tree leaf or triangle). Internal-only function pointers (`b3ParallelForCallback`, `b3ThreadFunction`) stay Odin procs. |
| `bool` | `bool` |

- Struct literals: `(b3Vec3){ 1, 2, 3 }` → `Vec3{1, 2, 3}`; designated `{ .x = 1 }` → `{x = 1}`. `= { 0 }` → `= {}`.
- `static const` tables → `@(rodata)` package (or file-private) variables; `#define` constants → `::` constants with
  the C value's type (`HUGE :: f32(100000.0) * ...` as in C).
- C casts: keep them explicit (`f32(i)`, `i32(f)`); Odin won't convert implicitly. Pointer casts `(T*)p` → `(^T)(p)`
  or `cast([^]T)p`.
- Pointer arithmetic `p + i` on arrays → `&p[i]` or `([^]T)(p)[i:]`; `memcpy`/`memset` → `mem.copy`/`mem.zero` or
  `intrinsics.mem_copy_non_overlapping`/`mem_zero` (`core:mem`), with byte counts as in C.
- `sizeof(T)` → `size_of(T)` (cast to `i32` where C stores it in an int).

## Macros and helpers

- Function-like macros become procs (`#force_inline` when trivial) with the macro's name minus the prefix:
  `B3_ASSERT(c)` → `assert(c)` (Odin's: disabled by `-disable-assert`, as C's is by NDEBUG), `B3_VALIDATE(c)` →
  `VALIDATE(c)`, `B3_ARRAY_COUNT(a)` → `i32(len(a))`, `B3_UNUSED(x)` → `_ = x`, `B3_SWAP(a, b)` → `a, b = b, a`,
  `B3_ALLOC(T, n)` → `ALLOC(T, n)`, `B3_FREE(p, T, n)` → `FREE(p, T, n)`, `B3_GROW(...)` → `GROW(...)`.
- `B3_INLINE` → plain proc, except in the math headers (`public_math_functions.odin`, `math_internal.odin`):
  `#force_inline`, since every C file inlines them and Odin's split modules otherwise can't;
  `B3_FORCE_INLINE` → `#force_inline proc`. `B3_RESTRICT`/`restrict` → drop. A release build is one LLVM module,
  so LLVM inlines most other `static inline` helpers by itself; the few it leaves as calls where clang inlines them
  (it doesn't get C's inline hint) are `#force_inline`: `SolvePointConstraint`, `AllocateManifolds`, `Bump`,
  `SetBitGrow`. Check after a port: build the release library with `-debug` and list its out-of-line procs
  (`llvm-pdbutil dump --symbols`) against the original's objects (`llvm-nm`).
- A local array C leaves uninitialized (a traversal stack, a batch, a scratch buffer) → `= ---`, as C: Odin zeroes
  locals, which costs a memset of the whole array on every call.
- Tracy macros (`b3TracyCZone*`, `b3TracyCFrame`) → drop. `b3Prefetch(addr)` → `Prefetch(addr)` (`platform.odin`,
  `intrinsics.prefetch_read_data`). `B3_CHECK_DEF(def)` → `CHECK_DEF(def)` (an assert, in `core.odin`).
- `b3Array(T)` → `Array(T)` and the `b3Array_*` macros → procs of the same name taking `^Array(T)`
  (`b3Array_Push(a, v)` → `Array_Push(&a, v)`, `b3Array_Get(a, i)` → `Array_Get(&a, i)` returning `^T`). See
  `container.odin`. `b3DeclareArray(T)` lines → nothing (the type is generic).
- `do { ... } while (0)` → a plain block.
- `for (int i = 0; i < n; ++i)` → `for i: i32 = 0; i < n; i += 1` (keep `i32` counters where C uses int).
- `switch` with fall-through → `switch` + `fallthrough`; `case` without `break` must be checked.
- `goto` → restructure minimally (labelled `for`/`break`), with a `// unified: goto` note.
- C's `x++` inside expressions → split into statements in the same order.
- Odin's `a / b` on integers truncates like C. `%` on negative ints: Odin's `%` truncates like C (`%%` is floored:
  don't use it).
- Ternary `c ? a : b` → `c ? a : b` (Odin has it) or `a if c else b`.
- Comparisons of pointers, `NULL` → `nil`.

## Floating point (determinism, AGENTS.md)

- f32 only in simulation code; no FMA (Odin doesn't contract; never call `fused_mul_add`); no `core:math`.
- `sqrtf(x)` → `intrinsics.sqrt(x)`; `fabsf(x)` → `abs(x)`; `floorf(x)` → `floorf(x)` from `core.odin` (exact); `isnan(x)` → `x != x`; `fminf`/`fmaxf` → `min`/`max` only where C uses `b3MinFloat`-style
  `a < b ? a : b` semantics — keep C's exact expression when NaN or −0 handling could differ.
- `FLT_MAX` → `max(f32)`, `FLT_EPSILON` → `f32(1.1920929e-7)`, `INT_MAX` → `max(i32)`, `INFINITY` →
  `f32(0h7F800000)`.
- **Constant folding differs from C.** Odin folds an expression of untyped literals exactly and rounds once; C
  rounds each `f` literal and each step to f32. `0.05 * 0.05` assigned to an f32 is 1 ulp off C's
  `0.05f * 0.05f`. Write every literal of a multi-step float expression typed: `f32(0.05) * f32(0.05)`,
  `f32(4.0) / f32(3.0) * PI`. A single literal (`x * 0.5`) is fine.
- The one `cosf`/`sinf` call (`math_functions.c`, `b3MakeRot` from radians for user convenience) → our own
  deterministic `ComputeCosSin` path, marked `// unified:`.

## SIMD

Upstream has SSE2, NEON and scalar 4-wide kernels (`simd_sse2.h`, `simd_neon.h`, `simd_scalar.h`) and an 8-wide AVX2
one. The port has the 4-wide kernels on `#simd[4]f32` (`core:simd`), with SSE2's semantics as the reference (e.g.
`_mm_min_ps` returns the second operand when either is NaN), on every platform, and the 8-wide AVX2 ones on
`#simd[8]f32` with the same semantics per lane, on x64 (`SIMD_HAS_WIDTH_8`), chosen at run time when the CPU has AVX2
(`DetectSIMDWidth`), as upstream. A lane computes the same at either width, so arm64, x64 and x64 with AVX2 give the
same results: upstream's `SIMDWidthTest` checks it, and `./b3 compare` runs both widths of both libraries.

The wide code is upstream's `.inl` files compiled twice (`wide4.c`, `wide8.c` with AVX2 enabled). The port has the
4-wide instantiations (`contact_solver_wide.odin`, `convex_manifold_wide.odin`), ported by hand with their
`B3_SIMD_WIDTH == 8` branches as `when SIMD_WIDTH == 8`; `./b3 gen` writes the 8-wide ones from them
(`*_wide8.odin`, `tools/gen_wide.odin`: W4 names become W8, `SIMD_WIDTH` is 8, every proc gets
`@(enable_target_feature = "avx2")`), so they are never edited and can't drift. Everything that takes or returns an
8-wide vector is compiled with AVX2 (the W8 kernels in `simd.odin` too): a call between AVX2 and non-AVX2 code passes
a 256-bit vector differently on each side.

## Customizations

Anything that is not a direct translation is marked:

```odin
// unified: <why, in one line>
...changed code...
// unified end
```

A removed part is one line: `// unified: not ported: <what and why>`. Every marker gets a line in `SPEC.md`'s
customization list. Phase 1 (now) has only the customizations forced by the language or by determinism; behaviour
changes for rollback come in phase 2.

## Threads and atomics

- `b3AtomicInt` etc. stay structs; the `platform.h` helpers become procs on `core:sync` atomics with C's memory
  orders (`__ATOMIC_SEQ_CST` → `sync.atomic_*` default, `__ATOMIC_RELAXED` → `*_explicit(..., .Relaxed)`).
- `b3Mutex`, `b3Semaphore`, `b3Thread` → `core:sync` and `core:thread`.
- The task callbacks (`b3EnqueueTaskCallback`, `b3FinishTaskCallback`) keep their shape.

## Bundled third-party code

- `qsort.h` → `qsort.odin`, ported 1:1 (identical ordering of equal keys matters).
- `rapidhash.h` → `rapidhash.odin`, ported 1:1 (hash values are compared by tests).
- `verstable.h` (generic hash maps, keyed by pointers or u64) → Odin's `map` where the map is only used for
  lookups. If code iterates a map in a way that can change results, stop and flag it: pointer-keyed iteration order
  is not deterministic.

## Checking your work

`odin check box3d -no-entry-point -vet -strict-style -vet-tabs` from the repo root, then `./b3 test`.

## Tests

Upstream's `test/test_<name>.c` → `tests/upstream/test_<name>.odin`, package `test_box3d_upstream`, importing
`b3 "../../box3d"` (so `b3CreateWorld` → `b3.CreateWorld`; internals the C test reaches through `src/`
headers are package-public). `test_macros.odin` has the harness:

- A subtest `int TestFoo(void)` → `TestFoo :: proc(t: ^testing.T) -> bool`, `return 0` → `return true`.
- `ENSURE(c)` → `ENSURE(t, c) or_return`; `ENSURE_SMALL(c, tol)` → `ENSURE_SMALL(t, c, tol) or_return`;
  `RUN_SUBTEST(TestFoo)` → `RUN_SUBTEST(t, TestFoo) or_return`.
- A top-level test (`int MathTest(void)`, listed in `main.c`) → `@(test) MathTest :: proc(t: ^testing.T)` that
  starts with `LOCK()` (Box3D has globals; Odin runs tests on several threads) and calls each subtest with
  `if !RUN_SUBTEST(t, TestFoo) {return}`.
- `printf` → `printf` (the harness's quiet one). Tests may use `core:math` (they aren't simulation code).
- Data files, if a test needs any, are read from `.out/upstream/src/` (`./b3 upstream`).

## Renames (collisions)

- `context` → `context_` (an Odin keyword), wherever upstream names a parameter or field `context`.
- `map` → `map_`, `matrix` → `matrix_`, `where` → `where_` (keywords), `mem` → `mem_` (the `core:mem` import).
- A local or parameter named like a package-level flag constant gets `_` (`enableSleep` → `enableSleep_` in
  `Body_EnableSleep` and `FinalizeBodiesTask`).
- `compound.c`'s static `TreeQueryCallbackFcn` → `TreeQueryCallbackFcn_` (the type of the same name).
- `#define`s in a `.c` file are `@(private = "file")` (C scopes them to the file; `LEAF_NODE` exists twice).
- Flag constants are untyped, and Odin rejects `~` on them: C's `x & ~F` → `x &~ F`, `x &= ~F` → `x &~= F`.
- `B3_REC(...)` / `B3_REC_CREATE(...)` are expanded at each call site:
  `if world.recording != nil { recArgs := RecArgs_X{...}; RecWrite_X(world.recording, &recArgs) }`.

## Interfaces worth knowing

- `QSORT(A: [^]$T, n: i32, $LESS: proc(a, b: T) -> bool)` (`qsort.odin`).
- `rapidhash(key: rawptr, len: int) -> u64`.
- `simd_wide.odin`: `FloatW`, `Vec3W` and every `b3*W` macro as aliases of the 4-wide version, for the 4-wide
  instantiations; `SIMD_WIDTH` and `WIDE_ALIGNMENT` are file-private constants of each (an alias can't be
  file-private), and the generated 8-wide files use the W8 names directly.
- `ThreadFunction` is in `timer.odin`.
- `FLT_EPSILON`, `FLT_MIN`, `FLT_MAX` are typed `f32` constants (`public_math_functions.odin`); so are `PI` and
  the other float `#define`s (`PI :: f32(...)`), which keeps constant folding in f32 like C.
- The length-scaled constants (`HUGE`, `LINEAR_SLOP`, `MIN_CAPSULE_LENGTH`, `OVERLAP_SLOP`, `SPECULATIVE_DISTANCE`,
  `MESH_REST_OFFSET`, `CONTACT_RECYCLE_DISTANCE`, `MAX_AABB_MARGIN`) are `f32` package variables, used as values;
  `UpdateLengthUnitConstants()` recomputes them (`public_constants.odin`).
- Id macros: `IS_NULL(id)`, `IS_NON_NULL(id)`, `ID_EQUALS(a, b)`; null ids `nullBodyId` etc.; `VALIDATE(cond)`.
- Anonymous unions are `using _: struct #raw_union {...}` (`TreeNode.height`, `ChildShape.hull`, ...); C bit-fields
  are `bit_field u32` (`MeshNode.data.asNode.childOffset`). `b3MeshEdgeFlags` are plain constants (`flatEdge1`).
