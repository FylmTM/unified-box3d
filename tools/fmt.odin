// `./b3 fmt`: odinfmt (odinfmt.json) over the hand-written Odin code. The generated files (export/, bindings/,
// tests/bindings/layout.odin) are left as `./b3 gen` writes them. odinfmt is ODINFMT, or odinfmt on PATH; without
// it the command says so and does nothing.
package tools

import "base:runtime"

import "core:fmt"
import "core:os"
import "core:strings"

@(init)
register_fmt :: proc "contextless" () {
	context = runtime.default_context()
	register(
		Command {
			name = "fmt",
			usage = "[--check]",
			summary = "format the Odin code with odinfmt (ODINFMT or PATH; --check: list what would change, fail if any)",
			run = cmd_fmt,
		},
	)
}

FMT_DIRS := [?]string{"box3d", "tools", "bench", "tests/upstream", "tests/port", "tests/bindings"}
FMT_GENERATED := [?]string{"tests/bindings/layout.odin"}

cmd_fmt :: proc(args: []string) -> bool {
	check := false
	for a in args {
		switch a {
		case "--check":
			check = true
		case:
			return fail("fmt: unknown argument %q", a)
		}
	}
	exe := os.get_env("ODINFMT", context.temp_allocator)
	if exe == "" {
		exe = which("odinfmt")
	}
	if exe == "" {
		log("fmt: odinfmt not found (set ODINFMT or put it on PATH); nothing formatted")
		return true
	}
	config := fmt.tprintf("-config:%s", path("odinfmt.json"))

	// Format copies under .out/fmt, then write back (or report) the files that changed.
	tmp := out("fmt")
	os.remove_all(tmp)
	files := make([dynamic]string, context.temp_allocator) // repo-relative
	for d in FMT_DIRS {
		infos, err := os.read_all_directory_by_path(path(d), context.temp_allocator)
		if err != nil {
			continue
		}
		os.make_directory_all(fmt.tprintf("%s/%s", tmp, d))
		outer: for fi in infos {
			if !strings.has_suffix(fi.name, ".odin") {
				continue
			}
			rel := fmt.tprintf("%s/%s", d, fi.name)
			for g in FMT_GENERATED {
				if rel == g {
					continue outer
				}
			}
			data, rerr := os.read_entire_file(path(rel), context.temp_allocator)
			if rerr != nil {
				return fail("fmt: reading %s: %v", rel, rerr)
			}
			if werr := os.write_entire_file(fmt.tprintf("%s/%s", tmp, rel), data); werr != nil {
				return fail("fmt: copying %s: %v", rel, werr)
			}
			append(&files, rel)
		}
	}
	if _, ok := run_capture({exe, config, "-w", tmp}); !ok {
		return fail("fmt: odinfmt failed")
	}
	changed := 0
	for rel in files {
		before, _ := os.read_entire_file(path(rel), context.temp_allocator)
		after, _ := os.read_entire_file(fmt.tprintf("%s/%s", tmp, rel), context.temp_allocator)
		if string(before) == string(after) {
			continue
		}
		changed += 1
		if check {
			log("fmt: %s is not formatted", rel)
		} else if err := os.write_entire_file(path(rel), after); err != nil {
			return fail("fmt: writing %s: %v", rel, err)
		} else {
			log("fmt: formatted %s", rel)
		}
	}
	if check && changed > 0 {
		return fail("fmt: %d files are not formatted (run ./b3 fmt)", changed)
	}
	log("fmt: %d files checked, %d %s", len(files), changed, check ? "unformatted" : "formatted")
	return true
}
