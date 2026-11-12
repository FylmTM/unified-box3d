# unified-box3d

[Box3D](https://github.com/erincatto/box3d), the 3D physics engine by Erin Catto, ported to [Odin](https://odin-lang.org)
and extended for **deterministic rollback netcode**: games that save the physics world every tick, roll back a few
ticks when late input arrives, re-simulate, and must compute bit-identical results on every machine.

It is the physics engine of unified, a core for cooperative games with deterministic rollback (Odin, with Godot as
the host), and it is usable on its own.

## What it is

- **A 1:1 port** of upstream Box3D at commit
  [`336263320469e5e9febfe18697ab562488f7e97a`](https://github.com/erincatto/box3d/tree/336263320469e5e9febfe18697ab562488f7e97a):
  one Odin file per C file, same functions in the same order, same names minus the `b3` prefix, so upstream
  changes can be ported file by file ([PORTING.md](PORTING.md)). Upstream's 28 test files are ported 1:1 and pass.
- **A shared library with upstream's C API.** The library exports exactly the functions and struct layouts of
  upstream's headers (`include/box3d/`), plus ours (`include/box3d/unified.h`), so it can replace the original
  without changing a line of calling code, and CI runs the same C programs against both.
- **Odin bindings** for the library (`bindings/`, generated).

## What's different from upstream

Every deviation is marked `// unified: <why>` in the code; [SPEC.md](SPEC.md) lists them all. In short:

- **Same results on every platform.** Upstream's 4-wide SIMD path everywhere and its 8-wide AVX2 path on x64 CPUs
  that have it, as upstream, both with SSE2's semantics per lane, so every platform and width computes the same
  (tested); sensor overlaps sorted with the ported sort instead of libc's `qsort`, whose order of equal keys differs
  between platforms; no fused multiply-add.
- **Rollback-friendly memory.** Everything a world keeps between steps lives in one arena per world that never
  moves; contact manifold blocks are smaller (32 elements, upstream 256), so small worlds snapshot cheaply.
- **Plane-locked bodies report clean velocities** (zero off-plane components after a step).
- Not ported: the profiler macros, double precision, C++ operator overloads.

## What's new

- **Snapshots in a memcpy:** `b3World_SaveSnapshot` / `b3World_RestoreSnapshot` save and restore a whole world
  (contacts, warm starts, islands, trees: everything) in-process, for rollback.
- **Canonical, portable images:** `b3World_SaveImage` / `b3World_LoadImage` write the same bytes for the same state on
  every platform (no pointer or padding bytes) and carry their geometry, so a late joiner can load a world whose
  hulls, meshes and compounds were created after the game began.
- **A whole-world checksum:** `b3World_HashState` hashes the canonical image, so a desync is caught at the tick it
  happens, internals included. `b3World_HashAwakeBodies` is a cheap one (the awake bodies' poses and velocities) for
  the ticks in between.
- **Proven determinism across worker counts:** 1, 2, 4 and 8 workers step bit-identically (tested).

## Proof

On Linux x64 and arm64, Windows x64 and macOS ARM64, CI:

- runs upstream's ported tests and ours (`./b3 test`);
- builds the original Box3D (C, pristine, at the commit above) and this library, runs the **same C comparison
  program** against both (stacks, piles, every joint type, meshes, height fields, compounds, bullets, sensors,
  sleep, queries, one and four workers, the 4-wide and the 8-wide SIMD path) and requires **bit-identical per-tick
  traces** (`./b3 compare`);
- runs upstream's public-API C tests against both.

## Performance

On par with the original: within a few percent of it in every one of upstream's benchmarks (noise is about that
size), and rollback is faster, its save and restore being a copy. Both are built alike: our library as released
(Odin `-o:speed`, no asserts or bounds checks), the original as upstream builds it (clang `-O2`, `NDEBUG`, no FP
contraction), both for `x86-64-v2` with the 8-wide AVX2 path chosen at run time (`tools/cc.odin`). Windows x64, one
machine (i7-14700KF), each benchmark pinned to performance cores at high priority.

Upstream's benchmarks (its benchmark program against both, as `./b3 compare --bench` builds it, one worker; ms, the
fastest of 3 runs, interleaved with the original's; the final body, shape, contact and joint counts match in every
one):

| benchmark | original | this library | ratio |
|---|---:|---:|---:|
| convex_pile | 3715 | 3647 | 0.98 |
| joint_grid | 717 | 696 | 0.97 |
| junkyard | 8430 | 8428 | 1.00 |
| large_pyramid | 863 | 862 | 1.00 |
| large_world | 3.8 | 3.9 | 1.02 |
| many_pyramids | 998 | 1012 | 1.01 |
| mesh_drop | 1028 | 1071 | 1.04 |
| rain | 1187 | 1192 | 1.00 |
| sleep | 423 | 430 | 1.02 |
| spinner | 1660 | 1634 | 0.98 |
| trees25 | 254 | 252 | 0.99 |
| trees50 | 123 | 122 | 0.99 |
| trees100 | 76 | 75 | 0.98 |
| village | 1450 | 1447 | 1.00 |
| washer | 12637 | 12875 | 1.02 |

Rollback (`./b3 bench`: boxes stacked on a ground; µs, median, the mean of two runs; 8 performance cores). Upstream
has no save/restore: the original saves and restores through its own world serializer (`./b3 bench --original`, the
one its replay keyframes use), this library copies its world arena.

| 1000 boxes | original | this library |
|---|---:|---:|
| step, 1 / 4 workers | 255 / 156 | 240 / 153 |
| save | 105 | 58 |
| restore | 172 | 60 |
| 8-tick rollback (restore + 8 × (step + save)), 1 / 4 workers | 3235 / 2391 | 2647 / 1881 |

For rollback:

- **Save only what a rollback can return to.** While re-simulating, a tick older than the newest confirmed one is
  never restored again; its save is wasted, and saves are about a fifth of an 8-tick rollback at 1000 boxes.
- **Hash the whole state only on the ticks peers compare.** `b3World_HashState` serializes the world (78 µs at 1000
  boxes, a third of a step); `b3World_HashAwakeBodies` takes 6 µs, for the ticks in between.
- **Small worlds: one worker.** Below a few hundred bodies more workers don't pay for themselves (100 boxes: 26 µs
  a step with 1 worker, 29 with 4).

## Use

Releases (`<upstream commit>-r<revision>`, e.g. `3362633-r1`) carry, per platform (Windows x64, Linux x64 and
arm64, macOS ARM64): the shared library (release and debug), the C headers, the Odin bindings, this README and
the license. From C, include `box3d/box3d.h` (and `box3d/unified.h` for the additions) and link the library. From
Odin, import `bindings/`.

## Build

Needs [Odin](https://odin-lang.org) (the pinned version: `.github/workflows/ci.yml`) and clang (for the comparison
with the original). `./b3 help` (`b3.cmd` on Windows) lists the commands:

```
./b3 build      # the shared library for this platform (lib/)
./b3 test       # upstream's tests, ours, and the bindings against the library
./b3 compare    # the original Box3D and this library, side by side
./b3 bench      # the scaling benchmark: the port compiled in, through the library (--original: on the original)
```

## AI disclosure

This port was written by AI: Claude (Anthropic) agents translated upstream's C to Odin file by file, designed and
built the rollback additions, the tooling and the CI, directed and reviewed by the maintainer, who signed off the
design (the porting rules, the API, the rollback features) but did not review every ported line. Its correctness
rests on the evidence above, not on line-by-line review: upstream's own tests, bit-identical traces against the
original on four platforms, and the layout checks of every public struct against upstream's headers. Read the code
with that in mind, and please report anything that differs from upstream in a way `SPEC.md` doesn't list.

## License

MIT, like Box3D ([LICENSE](LICENSE)).
