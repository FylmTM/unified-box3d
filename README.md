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

- **Same results on every platform.** One 4-wide SIMD path everywhere (upstream's 8-wide AVX2 path is not ported,
  so x64 and arm64 compute the same); sensor overlaps sorted with the ported sort instead of libc's `qsort`, whose
  order of equal keys differs between platforms; no fused multiply-add.
- **Rollback-friendly memory.** Everything a world keeps between steps lives in one arena per world that never
  moves; contact manifold blocks are smaller (32 elements, upstream 256), so small worlds snapshot cheaply.
- **Plane-locked bodies report clean velocities** (zero off-plane components after a step).
- Not ported: the AVX2 path, prefetch hints, the profiler macros, double precision, C++ operator overloads.

## What's new

- **Snapshots in a memcpy:** `b3World_SaveSnapshot` / `b3World_RestoreSnapshot` save and restore a whole world
  (contacts, warm starts, islands, trees: everything) in-process, for rollback.
- **Canonical, portable images:** `b3World_SaveImage` / `b3World_LoadImage` write the same bytes for the same state on
  every platform (no pointer or padding bytes) and carry their geometry, so a late joiner can load a world whose
  hulls, meshes and compounds were created after the game began.
- **A whole-world checksum:** `b3World_HashState` hashes the canonical image, so a desync is caught at the tick it
  happens, internals included.
- **Proven determinism across worker counts:** 1, 2, 4 and 8 workers step bit-identically (tested).

## Proof

On Linux x64 and arm64, Windows x64 and macOS ARM64, CI:

- runs upstream's ported tests and ours (`./b3 test`);
- builds the original Box3D (C, pristine, at the commit above) and this library, runs the **same C comparison
  program** against both (stacks, piles, every joint type, meshes, height fields, compounds, bullets, sensors,
  sleep, queries, one and four workers) and requires **bit-identical per-tick traces** (`./b3 compare`);
- runs upstream's public-API C tests and benchmarks against both.

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
./b3 bench      # the scaling benchmark: the port compiled in, and through the library
```

## License

MIT, like Box3D ([LICENSE](LICENSE)).
