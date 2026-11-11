// `./b3 build`: the shared library (export/ over the port) for this platform, into lib/<platform>/.
package tools

import "base:runtime"

import "core:fmt"
import "core:os"
import "core:strings"
import "core:time"

@(init)
register_build :: proc "contextless" () {
	context = runtime.default_context()
	register(
		Command {
			name = "build",
			usage = "[--debug] [--force]",
			summary = "build the shared library into lib/<platform>/ (--debug: unoptimized with debug info, into <platform>-debug)",
			run = cmd_build,
		},
	)
}

cmd_build :: proc(args: []string) -> bool {
	debug, force := false, false
	for a in args {
		switch a {
		case "--debug":
			debug = true
		case "--force":
			force = true
		case:
			return fail("build: unknown argument %q", a)
		}
	}
	_, ok := build_lib(debug, force)
	return ok
}

// Writes <dir>/box3d.def (every b3 export) and makes <dir>/box3d.lib from it (Windows).
import_library :: proc(dir: string) -> bool {
	src, ok := read_text(path("export", "export.odin"))
	if !ok {
		return fail("build: cannot read export/export.odin")
	}
	def := strings.builder_make(context.temp_allocator)
	strings.write_string(&def, "LIBRARY box3d.dll\nEXPORTS\n")
	n := 0
	for line in strings.split_lines_iterator(&src) {
		i := strings.index(line, "link_name = \"")
		if i < 0 {
			continue
		}
		rest := line[i + len("link_name = \""):]
		j := strings.index_byte(rest, '"')
		if j <= 0 {
			continue
		}
		fmt.sbprintf(&def, "\t%s\n", rest[:j])
		n += 1
	}
	def_path := fmt.tprintf("%s/box3d.def", dir)
	write_text(def_path, strings.to_string(def)) or_return
	c := cc_toolchain() or_return
	lib := fmt.tprintf("%s/box3d.lib", dir)
	os.remove(lib)
	cmd := strs(c.ar, "/nologo", fmt.tprintf("/def:%s", def_path), "/machine:x64", fmt.tprintf("/out:%s", lib))
	if out, rok := cc_exec(cmd, cc_env()); !rok {
		return fail("build: making %s failed:\n%s", lib, out)
	}
	log("build: %s (%d exports)", lib, n)
	return true
}

// lib/<platform>[-debug] (git-ignored; the release zips use the same layout)
lib_dir :: proc(debug: bool) -> string {
	return path("lib", fmt.tprintf("%s%s", platform(), debug ? "-debug" : ""))
}

// The shared library's file name on this platform (Windows also gets the import library box3d.lib beside it).
lib_name :: proc() -> string {
	when ODIN_OS == .Windows {
		return "box3d.dll"
	} else when ODIN_OS == .Darwin {
		return "libbox3d.dylib"
	} else {
		return "libbox3d.so"
	}
}

// The newest modification time of the .odin files in `dirs` (repo-relative).
newest_source :: proc(dirs: ..string) -> time.Time {
	newest: time.Time
	for d in dirs {
		infos, err := os.read_all_directory_by_path(path(d), context.temp_allocator)
		if err != nil {
			continue
		}
		for fi in infos {
			if strings.has_suffix(fi.name, ".odin") && time.diff(newest, fi.modification_time) > 0 {
				newest = fi.modification_time
			}
		}
	}
	return newest
}

// Builds the library unless it is newer than every source of the port and export/. Returns its path.
build_lib :: proc(debug: bool, force := false) -> (lib: string, ok: bool) {
	dir := lib_dir(debug)
	lib = fmt.tprintf("%s/%s", dir, lib_name())
	if !force {
		if fi, err := os.stat(lib, context.temp_allocator); err == nil {
			if time.diff(newest_source("box3d", "export"), fi.modification_time) > 0 {
				log("build: %s is up to date", lib)
				return lib, true
			}
		}
	}
	if err := os.make_directory_all(dir); err != nil {
		return lib, fail("build: cannot create %s: %v", dir, err)
	}
	cmd := make([dynamic]string, context.temp_allocator)
	append(&cmd, "odin", "build", path("export"), "-build-mode:dll", fmt.tprintf("-out:%s", lib))
	if debug {
		append(&cmd, "-debug", "-o:none")
	} else {
		// Like upstream's release build (NDEBUG, C): no asserts and no bounds checks. The debug variant keeps both.
		append(&cmd, "-o:speed", "-disable-assert", "-no-bounds-check")
	}
	// A portable baseline, so a library built on CI runs on any machine of the platform (Odin never contracts
	// a * b + c into an FMA, whatever the target allows).
	when ODIN_ARCH == .amd64 {
		append(&cmd, "-microarch:x86-64-v2")
	} else {
		append(&cmd, "-microarch:generic")
	}
	when ODIN_OS == .Linux {
		// Only the b3 API is exported: the library's Odin runtime symbols must not interpose with (or be
		// interposed by) the identically named ones of an Odin program loading it.
		script := fmt.tprintf("%s/box3d.map", dir)
		if err := os.write_entire_file(script, transmute([]u8)string("{ global: b3*; local: *; };\n")); err != nil {
			return lib, fail("build: writing %s: %v", script, err)
		}
		append(&cmd, fmt.tprintf("-extra-linker-flags:-Wl,-soname,libbox3d.so -Wl,--version-script=%s", script))
	} else when ODIN_OS == .Darwin {
		append(&cmd, "-extra-linker-flags:-Wl,-install_name,@rpath/libbox3d.dylib")
	}
	if !run(cmd[:]) {
		return lib, fail("build: the library did not build")
	}
	when ODIN_OS == .Windows {
		// The import library Odin's linker writes lacks the import descriptor some MSVC linkers need, so it's made
		// the standard way: a .def of the exports (export/export.odin's link names), through llvm-lib / lib.exe.
		import_library(dir) or_return
	}
	log("build: %s", lib)
	return lib, true
}
