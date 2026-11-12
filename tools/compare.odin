// `./b3 compare`: our library against the original. Builds upstream Box3D unpatched (`./b3 upstream`) and our
// shared library (`./b3 build`), compiles tests/compare/compare.c against each through the same headers, runs the
// scenes with 1 and 4 workers and diffs the traces tick by tick; also the original against itself (run to run, and
// 1 against 4 workers). --tests builds upstream's own C tests that use only the public headers against both and
// runs each; --bench builds upstream's benchmark app against both and prints the times side by side (a report, not
// a gate; --runs <n> repeats each benchmark and keeps the fastest run).
package tools

import "base:runtime"
import "core:fmt"
import "core:os"
import "core:strconv"
import "core:strings"
import "core:sync"
import "core:thread"
import "core:time"

@(init)
register_compare :: proc "contextless" () {
	context = runtime.default_context()
	register(
		Command {
			name = "compare",
			usage = "[--tests] [--bench [--runs <n>]] [--original | --lib <dir>] [--scene <name>]",
			summary = "our library against the original: the scenes' traces bit for bit (--tests: upstream's C tests on both; --bench: its benchmarks, a report, the fastest of --runs runs (1); --original: the original alone; --lib: ours from <dir>, not built)",
			run = cmd_compare,
		},
	)
}

// A library a program links against.
@(private = "file")
Lib :: struct {
	name:    string, // "original", "ours"
	link:    []string, // files for the linker
	runtime: string, // the shared library to put next to the program ("" for a static library)
	key:     string, // identifies the library's build (part of the program stamps)
}

@(private = "file")
Compare_Opts :: struct {
	tests, bench, original: bool,
	scene, lib_dir:         string,
	runs:                   int,
}

cmd_compare :: proc(args: []string) -> bool {
	opts := Compare_Opts {
		runs = 1,
	}
	for i := 0; i < len(args); i += 1 {
		switch args[i] {
		case "--tests":
			opts.tests = true
		case "--bench":
			opts.bench = true
		case "--original":
			opts.original = true
		case "--scene", "--lib", "--runs":
			if i + 1 >= len(args) {
				return fail("compare: %s needs a value", args[i])
			}
			switch args[i] {
			case "--scene":
				opts.scene = args[i + 1]
			case "--lib":
				opts.lib_dir = args[i + 1]
			case "--runs":
				n, n_ok := strconv.parse_int(args[i + 1])
				if !n_ok || n < 1 {
					return fail("compare: --runs needs a positive count, not %q", args[i + 1])
				}
				opts.runs = n
			}
			i += 1
		case:
			return fail("compare: unknown argument %q (./b3 help)", args[i])
		}
	}

	up_path := ensure_upstream() or_return
	original := Lib {
		name = "original",
		link = strs(up_path),
		key  = file_key(up_path),
	}
	libs := make([dynamic]Lib, context.temp_allocator)
	append(&libs, original)
	if !opts.original {
		ours, ok := ensure_ours(opts.lib_dir)
		if !ok {
			return false
		}
		append(&libs, ours)
	}
	c := cc_toolchain() or_return

	ok := compare_traces(c, libs[:], opts.scene)
	if opts.tests {
		ok = compare_tests(c, libs[:]) && ok
	}
	if opts.bench {
		ok = compare_bench(c, libs[:], opts.runs) && ok
	}
	return ok
}

// Our shared library: `./b3 build` (in process), then lib/<platform>/; or the one in lib_dir as it is.
@(private = "file")
ensure_ours :: proc(lib_dir: string) -> (lib: Lib, ok: bool) {
	built := false
	dir := lib_dir
	if dir == "" {
		for cmd in commands {
			if cmd.name == "build" {
				if !cmd.run(nil) {
					return {}, fail("compare: ./b3 build failed")
				}
				built = true
			}
		}
		dir = path("lib", platform())
	}
	lib.name = "ours"
	when ODIN_OS == .Windows {
		lib.link = strs(path_join(dir, "box3d.lib"))
		lib.runtime = path_join(dir, "box3d.dll")
	} else when ODIN_OS == .Darwin {
		lib.link = strs(path_join(dir, "libbox3d.dylib"))
		lib.runtime = lib.link[0]
	} else {
		lib.link = strs(path_join(dir, "libbox3d.so"))
		lib.runtime = lib.link[0]
	}
	if !os.is_file(lib.runtime) || !os.is_file(lib.link[0]) {
		if built {
			return {}, fail("compare: ./b3 build didn't produce %s", lib.runtime)
		}
		return {}, fail("compare: no library in %s (--original compares the original alone)", dir)
	}
	lib.key = fmt.tprintf("%s %s", file_key(lib.link[0]), file_key(lib.runtime))
	return lib, true
}

// Size and modification time: enough to notice a rebuilt library.
@(private = "file")
file_key :: proc(p: string) -> string {
	fi, err := os.stat(p, context.temp_allocator)
	if err != nil {
		return "missing"
	}
	return fmt.tprintf("%s:%d:%v", base_of(p), fi.size, time.to_unix_nanoseconds(fi.modification_time))
}

// The headers programs compile against: this repo's include/ (upstream's headers verbatim, plus unified.h), else
// upstream's.
@(private = "file")
include_dir :: proc() -> string {
	if os.is_file(path("include", "box3d", "box3d.h")) {
		return path("include")
	}
	return path_join(upstream_src(), "include")
}

@(private = "file")
include_flag :: proc(c: Cc, dir: string) -> string {
	return fmt.tprintf("%s%s", c.msvc ? "/I" : "-I", dir)
}

// Compiles and links a program against `lib` into .out/compare/<lib>/<name>, unless its stamp (sources, headers,
// flags, compiler, library) is current. Returns the executable.
@(private = "file")
build_program :: proc(
	c: Cc,
	lib: Lib,
	name: string,
	sources: []string,
	extra_flags: []string,
) -> (
	exe: string,
	ok: bool,
) {
	dir := out("compare", lib.name)
	exe = path_join(dir, fmt.tprintf("%s%s", name, ODIN_OS == .Windows ? ".exe" : ""))
	flags := concat(cc_flags(c), strs(include_flag(c, include_dir())), extra_flags)
	if lib.runtime != "" && c.msvc {
		flags = concat(flags, strs("/DBOX3D_DLL"))
	}

	key_src := strings.builder_make(context.temp_allocator)
	fmt.sbprintf(
		&key_src,
		"%s|%s|%s|%s|",
		c.version,
		strings.join(flags, " ", context.temp_allocator),
		lib.key,
		sources,
	)
	for s in sources {
		text, _ := read_text(s)
		strings.write_string(&key_src, text)
	}
	for h in files_with_suffix(path_join(include_dir(), "box3d"), ".h") {
		text, _ := read_text(h)
		strings.write_string(&key_src, text)
	}
	key := sha256_hex(transmute([]byte)strings.to_string(key_src))
	stamp := path_join(dir, fmt.tprintf("%s.stamp", name))
	if os.is_file(exe) && stamp_ok(stamp, key) {
		return exe, true
	}
	os.make_directory_all(dir)
	rpath := lib.runtime != "" ? dir_of(lib.runtime) : ""
	cc_link_exe(c, flags, sources, lib.link, exe, path_join(dir, fmt.tprintf("obj-%s", name)), rpath) or_return
	if lib.runtime != "" {
		// Next to the program as well: Windows finds DLLs there, and it doesn't depend on the install name.
		if err := os.copy_file(path_join(dir, base_of(lib.runtime)), lib.runtime); err != nil {
			return "", fail("cannot copy %s: %v", lib.runtime, err)
		}
	}
	write_text(stamp, key) or_return
	return exe, true
}

// --- running programs ---------------------------------------------------------------------------------------------

@(private = "file")
Run :: struct {
	cmd:    []string,
	output: string,
	ok:     bool,
	secs:   f64,
}

// Runs the programs in parallel (each in its own directory, finding shared libraries there), capturing stdout and
// stderr.
@(private = "file")
run_programs :: proc(runs: []Run, parallel := true) {
	env := program_env()
	Job :: struct {
		runs: []Run,
		env:  []string,
		next: int,
	}
	job := Job {
		runs = runs,
		env  = env,
	}
	worker :: proc(job: ^Job) {
		for {
			i := sync.atomic_add(&job.next, 1)
			if i >= len(job.runs) {
				return
			}
			r := &job.runs[i]
			start := time.tick_now()
			r.output, r.ok = cc_exec(r.cmd, job.env, dir_of(r.cmd[0]))
			r.secs = time.duration_seconds(time.tick_since(start))
		}
	}
	n := parallel ? clamp(os.get_processor_core_count() / 2, 1, len(runs)) : 1
	threads := make([]^thread.Thread, n, context.temp_allocator)
	for &t in threads {
		t = thread.create_and_start_with_poly_data(&job, worker)
	}
	for t in threads {
		thread.join(t)
		thread.destroy(t)
	}
}

// Our environment, plus the program's own directory on the library search path (`.`: programs run there).
@(private = "file")
program_env :: proc() -> []string {
	when ODIN_OS == .Windows {
		return nil
	} else {
		env, _ := os.environ(context.temp_allocator)
		out := make([dynamic]string, context.temp_allocator)
		var := ODIN_OS == .Darwin ? "DYLD_LIBRARY_PATH" : "LD_LIBRARY_PATH"
		found := false
		for e in env {
			if strings.has_prefix(e, fmt.tprintf("%s=", var)) {
				append(&out, fmt.tprintf("%s:.", e))
				found = true
			} else {
				append(&out, e)
			}
		}
		if !found {
			append(&out, fmt.tprintf("%s=.", var))
		}
		return out[:]
	}
}

// --- the traces ---------------------------------------------------------------------------------------------------

@(private = "file")
Trace :: struct {
	label:  string,
	scenes: [dynamic]string, // in order
	lines:  map[string][dynamic]string, // scene -> its lines
}

@(private = "file")
parse_trace :: proc(label, text: string) -> Trace {
	t := Trace {
		label  = label,
		scenes = make([dynamic]string, context.temp_allocator),
		lines  = make(map[string][dynamic]string, context.temp_allocator),
	}
	rest := text
	for line in strings.split_lines_iterator(&rest) {
		l := strings.trim_right(line, "\r")
		sp := strings.index_byte(l, ' ')
		if sp <= 0 {
			continue
		}
		scene := l[:sp]
		if scene not_in t.lines {
			append(&t.scenes, scene)
			t.lines[scene] = make([dynamic]string, context.temp_allocator)
		}
		append(&t.lines[scene], l)
	}
	return t
}

// The last n lines of a text.
@(private = "file")
tail :: proc(text: string, n: int) -> string {
	t := strings.trim_right_space(text)
	end := len(t)
	for _ in 0 ..< n {
		i := strings.last_index_byte(t[:end], '\n')
		if i < 0 {
			return t
		}
		end = i
	}
	return t[end + 1:]
}

// The first difference per scene between a trace and the reference; true if identical.
@(private = "file")
diff_traces :: proc(ref, got: Trace) -> bool {
	same := true
	for scene in ref.scenes {
		a := ref.lines[scene]
		b, has := got.lines[scene]
		if !has {
			fmt.printfln("  %s: no trace from %s (nor of the scenes after it)", scene, got.label)
			same = false
			break
		}
		for i in 0 ..< max(len(a), len(b)) {
			la := i < len(a) ? a[i] : "(end)"
			lb := i < len(b) ? b[i] : "(end)"
			if la != lb {
				fmt.printfln("  %s: %s differs from %s first at line %d:", scene, got.label, ref.label, i + 1)
				fmt.printfln("    %-9s %s", ref.label, la)
				fmt.printfln("    %-9s %s", got.label, lb)
				same = false
				break
			}
		}
	}
	return same
}

@(private = "file")
compare_traces :: proc(c: Cc, libs: []Lib, scene: string) -> bool {
	src := path("tests", "compare", "compare.c")
	exes := make([]string, len(libs), context.temp_allocator)
	for lib, i in libs {
		exes[i] = build_program(c, lib, "compare", strs(src), nil) or_return
	}

	// The reference twice (run to run) and with 4 workers; ours with 1 and 4.
	Spec :: struct {
		lib:     int,
		workers: string,
		label:   string,
	}
	specs := make([dynamic]Spec, context.temp_allocator)
	append(&specs, Spec{0, "1", "original"}, Spec{0, "1", "original-again"}, Spec{0, "4", "original-4w"})
	if len(libs) > 1 {
		append(&specs, Spec{1, "1", "ours"}, Spec{1, "4", "ours-4w"})
	}
	runs := make([]Run, len(specs), context.temp_allocator)
	for s, i in specs {
		cmd := make([dynamic]string, context.temp_allocator)
		append(&cmd, exes[s.lib], s.workers)
		if scene != "" {
			append(&cmd, scene)
		}
		runs[i].cmd = cmd[:]
	}
	start := time.tick_now()
	run_programs(runs)
	traces := make([]Trace, len(specs), context.temp_allocator)
	same := true
	for r, i in runs {
		trace_file := out("compare", fmt.tprintf("trace-%s.txt", specs[i].label))
		write_text(trace_file, r.output)
		if !r.ok {
			// The trace up to the crash is still diffed (each tick is flushed), to show whether it diverged first.
			fmt.printfln("%s crashed; the end of its output (%s):\n%s", specs[i].label, trace_file, tail(r.output, 3))
			same = false
			if i == 0 {
				return fail("compare: the original crashed")
			}
		}
		traces[i] = parse_trace(specs[i].label, r.output)
	}
	if len(traces[0].scenes) == 0 {
		return fail("compare: the original printed no trace")
	}

	labels := make([dynamic]string, context.temp_allocator)
	for i in 1 ..< len(traces) {
		if diff_traces(traces[0], traces[i]) {
			append(&labels, traces[i].label)
		} else {
			same = false
		}
	}
	ticks := 0
	for s in traces[0].scenes {
		ticks += len(traces[0].lines[s]) - 1 // tick 0 is the scene as built
	}
	if !same {
		return fail("compare: the traces differ or a program crashed (all in .out/compare/trace-*.txt)")
	}
	fmt.printfln(
		"identical: %d scenes x %d ticks (%s), original = %s, in %.1fs",
		len(traces[0].scenes),
		ticks,
		strings.join(traces[0].scenes[:], " ", context.temp_allocator),
		strings.join(labels[:], " = ", context.temp_allocator),
		time.duration_seconds(time.tick_since(start)),
	)
	if len(libs) == 1 {
		fmt.println("(the original alone: our library not compared)")
	}
	return true
}

// --- upstream's C tests -------------------------------------------------------------------------------------------

// Upstream's test files that include only public headers (box3d/*.h) and test_macros.h, with their entry points.
@(private = "file")
public_tests :: proc() -> (files, funcs: []string) {
	fs := make([dynamic]string, context.temp_allocator)
	fn := make([dynamic]string, context.temp_allocator)
	for f in files_with_suffix(path_join(upstream_src(), "test"), ".c") {
		if base_of(f) == "main.c" {
			continue
		}
		text, _ := read_text(f)
		public := true
		entry := ""
		rest := text
		for line in strings.split_lines_iterator(&rest) {
			l := strings.trim_space(line)
			if strings.has_prefix(l, "#include \"") {
				inc := strings.trim_suffix(strings.trim_prefix(l, "#include \""), "\"")
				if !strings.has_prefix(inc, "box3d/") && inc != "test_macros.h" {
					public = false
				}
			}
			if strings.has_prefix(l, "int ") && strings.has_suffix(l, "( void )") && entry == "" {
				entry = strings.trim_space(strings.trim_suffix(strings.trim_prefix(l, "int "), "( void )"))
			}
		}
		if public && entry != "" {
			append(&fs, f)
			append(&fn, entry)
		}
	}
	return fs[:], fn[:]
}

@(private = "file")
compare_tests :: proc(c: Cc, libs: []Lib) -> bool {
	files, funcs := public_tests()
	if len(files) == 0 {
		return fail("compare: no public-API tests found in upstream's test/")
	}
	// A runner that runs one test by name (each in its own process: a crash is that test's failure).
	main := strings.builder_make(context.temp_allocator)
	strings.write_string(&main, "// Generated by ./b3 compare --tests.\n#include <stdio.h>\n#include <string.h>\n\n")
	for f in funcs {
		fmt.sbprintf(&main, "extern int %s( void );\n", f)
	}
	strings.write_string(&main, "\nint main( int argc, char** argv )\n{\n\tif ( argc != 2 ) return 2;\n")
	for f in funcs {
		fmt.sbprintf(
			&main,
			"\tif ( strcmp( argv[1], \"%s\" ) == 0 ) {{ int r = %s(); printf( \"%%s %%s\\n\", r == 0 ? \"passed\" : \"failed\", argv[1] ); return r; }}\n",
			f,
			f,
		)
	}
	strings.write_string(&main, "\treturn 2;\n}\n")
	main_c := out("compare", "tests_main.c")
	if old, _ := read_text(main_c); old != strings.to_string(main) {
		write_text(main_c, strings.to_string(main)) or_return
	}
	sources := concat(strs(main_c), files)
	extra := strs(include_flag(c, path_join(upstream_src(), "test")))
	if c.msvc {
		extra = concat(extra, strs("/D_CRT_SECURE_NO_WARNINGS"))
	}

	exes := make([]string, len(libs), context.temp_allocator)
	for lib, i in libs {
		exes[i] = build_program(c, lib, "tests", sources, extra) or_return
	}
	runs := make([]Run, len(libs) * len(funcs), context.temp_allocator)
	for f, t in funcs {
		for _, i in libs {
			runs[t * len(libs) + i].cmd = strs(exes[i], f)
		}
	}
	run_programs(runs)

	fmt.printfln(
		"upstream's C tests (public API only; %d of %d test files):",
		len(funcs),
		len(files_with_suffix(path_join(upstream_src(), "test"), ".c")) - 1,
	)
	fmt.printf("  %-16s", "test")
	for lib in libs {
		fmt.printf(" %-10s", lib.name)
	}
	fmt.println()
	ok := true
	for f, t in funcs {
		fmt.printf("  %-16s", f)
		for _, i in libs {
			r := runs[t * len(libs) + i]
			fmt.printf(" %-10s", r.ok ? "pass" : "FAIL")
			ok = ok && r.ok
		}
		fmt.println()
	}
	if !ok {
		for r, i in runs {
			if !r.ok {
				fmt.printfln("--- %s on %s:\n%s", r.cmd[1], libs[i % len(libs)].name, r.output)
			}
		}
		return fail("compare: upstream's C tests failed")
	}
	return true
}

// --- upstream's benchmarks ----------------------------------------------------------------------------------------

@(private = "file")
Bench :: struct {
	name:     string,
	ms:       f64,
	counters: string,
}

@(private = "file")
parse_bench :: proc(text: string) -> [dynamic]Bench {
	out := make([dynamic]Bench, context.temp_allocator)
	rest := text
	for line in strings.split_lines_iterator(&rest) {
		l := strings.trim_space(line)
		if strings.has_prefix(l, "benchmark: ") {
			name := strings.trim_prefix(l, "benchmark: ")
			if comma := strings.index_byte(name, ','); comma >= 0 {
				name = name[:comma]
			}
			append(&out, Bench{name = name, ms = -1})
		} else if strings.has_prefix(l, "run ") && len(out) > 0 {
			colon := strings.index(l, ": ")
			if colon < 0 {
				continue
			}
			v := strings.trim_suffix(strings.trim_space(l[colon + 2:]), " (ms)")
			if ms, ok := strconv.parse_f64(v); ok {
				b := &out[len(out) - 1]
				b.ms = b.ms < 0 ? ms : min(b.ms, ms)
			}
		} else if strings.has_prefix(l, "body ") && len(out) > 0 {
			out[len(out) - 1].counters = l
		}
	}
	return out
}

@(private = "file")
compare_bench :: proc(c: Cc, libs: []Lib, repeats: int) -> bool {
	src := upstream_src()
	sources := strs(
		path_join(src, "benchmark", "main.c"),
		path_join(src, "shared", "benchmarks.c"),
		path_join(src, "shared", "human.c"),
		path_join(src, "shared", "utils.c"),
	)
	extra := strs(include_flag(c, path_join(src, "shared")))
	if c.msvc {
		extra = concat(extra, strs("/D_CRT_SECURE_NO_WARNINGS"))
	}
	exes := make([]string, len(libs), context.temp_allocator)
	for lib, i in libs {
		exes[i] = build_program(c, lib, "benchmark", sources, extra) or_return
	}
	// One at a time (timing), one worker, `repeats` runs of each benchmark (parse_bench keeps the fastest); each program
	// writes its CSVs in its own directory.
	results := make([][dynamic]Bench, len(libs), context.temp_allocator)
	for exe, i in exes {
		runs := []Run{{cmd = strs(exe, "-t=1", "-w=1", fmt.tprintf("-r=%d", repeats))}}
		run_programs(runs, parallel = false)
		if !runs[0].ok {
			fmt.println(runs[0].output)
			return fail("compare: the benchmark on %s failed", libs[i].name)
		}
		results[i] = parse_bench(runs[0].output)
	}
	fmt.printfln("upstream's benchmarks, 1 worker, fastest of %d run(s) (ms; a report, not a gate):", repeats)
	fmt.printf("  %-16s", "benchmark")
	for lib in libs {
		fmt.printf(" %10s", lib.name)
	}
	if len(libs) > 1 {
		fmt.printf(" %8s  %s", "ours/orig", "final counters")
	}
	fmt.println()
	for b, k in results[0] {
		fmt.printf("  %-16s", b.name)
		for r in results {
			fmt.printf(" %10.1f", k < len(r) ? r[k].ms : -1)
		}
		if len(libs) > 1 && k < len(results[1]) {
			o := results[1][k]
			same := o.counters == b.counters ? "same" : fmt.tprintf("DIFFER: %q vs %q", b.counters, o.counters)
			fmt.printf(" %8.2f  %s", o.ms / b.ms, same)
		}
		fmt.println()
	}
	return true
}
