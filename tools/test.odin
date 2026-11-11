// `./b3 test`: upstream's tests (alone: they change Box3D's globals), the port's own tests, and the bindings against
// the shared library (built first if it is out of date).
package tools

import "base:runtime"

import "core:fmt"
import "core:os"
import "core:strings"

@(init)
register_test :: proc "contextless" () {
	context = runtime.default_context()
	register(
		Command {
			name = "test",
			usage = "[upstream|port|bindings]... [--debug]",
			summary = "run the test suites (default: all, in that order; --debug: bindings against the debug library)",
			run = cmd_test,
		},
	)
}

TEST_SUITES := [?]string{"upstream", "port", "bindings"}

EXE :: ".exe" when ODIN_OS == .Windows else ""

cmd_test :: proc(args: []string) -> bool {
	debug := false
	chosen := make(map[string]bool, context.temp_allocator)
	for a in args {
		if a == "--debug" {
			debug = true
			continue
		}
		known := false
		for s in TEST_SUITES {
			known ||= a == s
		}
		if !known {
			return fail("test: unknown argument %q", a)
		}
		chosen[a] = true
	}
	os.make_directory_all(out("tests"))
	for suite in TEST_SUITES {
		if len(chosen) > 0 && !chosen[suite] {
			continue
		}
		log("test: %s", suite)
		exe := out("tests", fmt.tprintf("%s%s", suite, EXE))
		if suite == "bindings" {
			lib := build_lib(debug) or_return
			cmd := make([dynamic]string, context.temp_allocator)
			append(&cmd, "odin", "build", path("tests/bindings"), "-build-mode:test", fmt.tprintf("-out:%s", exe))
			if debug {
				append(&cmd, "-define:BOX3D_LIB_DEBUG=true")
			}
			if !run(cmd[:]) {
				return fail("test: tests/bindings did not build")
			}
			place_lib(exe, lib) or_return
			if !run({exe}) {
				return fail("test: bindings FAILED")
			}
			continue
		}
		if !run({"odin", "test", path("tests", suite), fmt.tprintf("-out:%s", exe)}) {
			return fail("test: %s FAILED", suite)
		}
	}
	return true
}

// Makes `exe` find the shared library `lib` when we run it: a copy next to it (Windows searches the executable's
// directory), and that directory on LD_LIBRARY_PATH / DYLD_LIBRARY_PATH (our environment, which children inherit).
place_lib :: proc(exe: string, lib: string) -> bool {
	dir := exe[:strings.last_index_byte(exe, '/')]
	name := lib[strings.last_index_byte(lib, '/') + 1:]
	data, err := os.read_entire_file(lib, context.temp_allocator)
	if err != nil {
		return fail("cannot read %s: %v", lib, err)
	}
	if werr := os.write_entire_file(fmt.tprintf("%s/%s", dir, name), data); werr != nil {
		return fail("cannot copy %s next to %s: %v", lib, exe, werr)
	}
	when ODIN_OS == .Linux {
		os.set_env("LD_LIBRARY_PATH", dir)
	} else when ODIN_OS == .Darwin {
		os.set_env("DYLD_LIBRARY_PATH", dir)
	}
	return true
}
