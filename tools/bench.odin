// `./b3 bench`: the scaling benchmark (bench/scale.odin) built from source, then the same file against the shared
// library through bindings/. The prebuilt library must perform like the compiled-in port, and end in the same worlds.
// --original runs the same scene on upstream's C Box3D (bench/scale_original.c, linked against `./b3 upstream`'s
// static library), with upstream's world serializer for save and restore: what rollback costs without the port. Its
// hashes are upstream's own, so they are not compared with ours. The flags pick what runs; none: source and library.
package tools

import "base:runtime"

import "core:fmt"
import "core:os"
import "core:strings"

@(init)
register_bench :: proc "contextless" () {
	context = runtime.default_context()
	register(
		Command {
			name = "bench",
			usage = "[--source] [--library] [--original]",
			summary = "the scaling benchmark from source and against the shared library; fails if their worlds differ (--original: on upstream's C Box3D, its serializer for rollback; flags pick what runs, default the first two)",
			run = cmd_bench,
		},
	)
}

cmd_bench :: proc(args: []string) -> bool {
	source, library, original: bool
	for a in args {
		switch a {
		case "--source":
			source = true
		case "--library":
			library = true
		case "--original":
			original = true
		case:
			return fail("bench: unknown argument %q", a)
		}
	}
	if !source && !library && !original {
		source, library = true, true
	}
	dir := out("bench")
	os.make_directory_all(dir)
	flags := []string{"-o:speed", "-microarch:x86-64-v2" when ODIN_ARCH == .amd64 else "-microarch:generic"}

	src_out, lib_out: string
	if source {
		exe := fmt.tprintf("%s/scale_source%s", dir, EXE)
		cmd := make([dynamic]string, context.temp_allocator)
		append(&cmd, "odin", "build", path("bench"), fmt.tprintf("-out:%s", exe))
		append(&cmd, ..flags)
		if !run(cmd[:]) {
			return fail("bench: bench/ did not build")
		}
		log("bench: from source (the port compiled in)")
		ok: bool
		src_out, ok = run_capture({exe})
		if !ok {
			return fail("bench: the source build failed")
		}
		fmt.print(src_out)
	}
	if library {
		lib := build_lib(false) or_return
		// The same file, importing the bindings instead of the port.
		raw, err := os.read_entire_file(path("bench/scale.odin"), context.temp_allocator)
		if err != nil {
			return fail("bench: reading bench/scale.odin: %v", err)
		}
		text, replaced := strings.replace(
			string(raw),
			`import b3 "../box3d"`,
			`import b3 "../../../bindings"`,
			1,
			context.temp_allocator,
		)
		if !replaced {
			return fail(`bench: bench/scale.odin no longer has the line import b3 "../box3d"`)
		}
		pkg := fmt.tprintf("%s/library", dir)
		os.make_directory_all(pkg)
		if werr := os.write_entire_file(fmt.tprintf("%s/scale.odin", pkg), transmute([]u8)text); werr != nil {
			return fail("bench: writing %s: %v", pkg, werr)
		}
		exe := fmt.tprintf("%s/scale_library%s", pkg, EXE)
		cmd := make([dynamic]string, context.temp_allocator)
		append(&cmd, "odin", "build", pkg, fmt.tprintf("-out:%s", exe))
		append(&cmd, ..flags)
		if !run(cmd[:]) {
			return fail("bench: the library variant did not build")
		}
		log("bench: against the shared library (%s)", lib)
		place_lib(exe, lib) or_return
		ok: bool
		lib_out, ok = run_capture({exe})
		if !ok {
			return fail("bench: the library run failed")
		}
		fmt.print(lib_out)
	}
	if original {
		bench_original() or_return
	}
	if source && library {
		a, b := bench_hashes(src_out), bench_hashes(lib_out)
		if a != b {
			return fail("bench: the library's worlds differ from the source build's (hash column)")
		}
		log(
			"bench: the library ends in the same worlds as the source build (%d runs)",
			len(strings.split_lines(a, context.temp_allocator)) - 1,
		)
	}
	return true
}

// bench/scale_original.c, compiled like upstream and with its internal headers (the serializer), linked against its
// static library, then run.
bench_original :: proc() -> bool {
	lib := ensure_upstream() or_return
	c := cc_toolchain() or_return
	dir := out("bench", "original")
	exe := path_join(dir, fmt.tprintf("scale_original%s", EXE))
	src := strs(path("bench", "scale_original.c"))
	cc_link_exe(c, upstream_flags(c), src, strs(lib), exe, path_join(dir, "obj")) or_return
	log("bench: the original (upstream's C Box3D, its serializer for save and restore; hashes are upstream's)")
	output, ok := run_capture({exe})
	fmt.print(output)
	if !ok {
		return fail("bench: the original's run failed")
	}
	return true
}

// The `bodies workers hash` of every result row.
bench_hashes :: proc(output: string) -> string {
	b := strings.builder_make(context.temp_allocator)
	for line in strings.split_lines(output, context.temp_allocator) {
		f := strings.fields(line, context.temp_allocator)
		if len(f) < 3 || f[0] == "bodies" {
			continue
		}
		fmt.sbprintfln(&b, "%s %s %s", f[0], f[1], f[len(f) - 1])
	}
	return strings.to_string(b)
}
