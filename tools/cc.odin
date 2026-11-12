// The C toolchain the comparison builds with (`./b3 upstream`, `./b3 compare`): clang on Linux and macOS, clang-cl
// on Windows (with the MSVC environment from vswhere + vcvarsall unless the shell already has INCLUDE and LIB), the
// static CRT like Odin's. Plus parallel compiles, stamps and a few file helpers.
package tools

import "core:crypto/hash"
import "core:encoding/hex"
import "core:fmt"
import "core:os"
import "core:strings"
import "core:sync"
import "core:thread"

Cc :: struct {
	cc:      string, // clang, or clang-cl
	ar:      string, // llvm-ar / ar, or llvm-lib / lib.exe
	msvc:    bool,
	version: string, // `cc --version`'s first line (part of every stamp)
}

@(private = "file")
cc_env_cache: Maybe([]string)

// The environment for the compiler's processes (nil: ours): on Windows MSVC's, computed once when first needed
// (vcvarsall takes a second, so an up-to-date build doesn't pay for it).
cc_env :: proc() -> []string {
	if e, ok := cc_env_cache.?; ok {
		return e
	}
	e: []string
	when ODIN_OS == .Windows {
		e = cc_msvc_env()
	}
	cc_env_cache = e
	return e
}

@(private = "file")
cc_cache: Maybe(Cc)

// The flags upstream Box3D is built with for determinism (CMake's, minus warnings): C17, -O2 -DNDEBUG, no FP
// contraction, hidden symbols, the static CRT on Windows; the AVX2 path in, chosen at run time as upstream ships it.
// The target CPU is our library's (tools/build.odin: -microarch:x86-64-v2 on x64, generic on arm64), and -O2 is the
// LLVM pipeline Odin's -o:speed runs, so the two are compiled alike. The comparison program and upstream's tests
// compile with the same.
cc_flags :: proc(c: Cc) -> []string {
	when ODIN_ARCH == .amd64 {
		cpu :: "-march=x86-64-v2"
	} else {
		cpu :: "-mcpu=generic"
	}
	if c.msvc {
		return strs("/nologo", "/MT", "/O2", "/DNDEBUG", "/std:c17", "/clang:-ffp-contract=off", "/clang:" + cpu)
	}
	flags := strs("-O2", "-DNDEBUG", "-std=gnu17", "-fvisibility=hidden", "-ffp-contract=off", cpu)
	when ODIN_OS == .Darwin {
		flags = concat(flags, strs("-arch", "arm64", "-mmacosx-version-min=11.0"))
	}
	return flags
}

cc_toolchain :: proc() -> (c: Cc, ok: bool) {
	if t, cached := cc_cache.?; cached {
		return t, true
	}
	when ODIN_OS == .Windows {
		c.msvc = true
		c.cc = cc_find_clang_cl()
		if c.cc == "" {
			return {}, fail("clang-cl not found: install LLVM (or Visual Studio's \"C++ Clang tools\")")
		}
		dir := dir_of(c.cc)
		c.ar = os.is_file(path_join(dir, "llvm-lib.exe")) ? path_join(dir, "llvm-lib.exe") : "lib.exe"
	} else {
		c.cc = which("clang")
		if c.cc == "" {
			return {}, fail("clang must be on PATH")
		}
		c.ar = which("llvm-ar")
		if c.ar == "" {
			c.ar = "ar"
		}
	}
	out, vok := cc_exec({c.cc, "--version"})
	if !vok {
		return {}, fail("%s --version failed:\n%s", c.cc, out)
	}
	lines := strings.split_lines(strings.trim_space(out), context.temp_allocator)
	c.version = strings.clone(strings.trim_space(lines[0]))
	cc_cache = c
	return c, true
}

// Runs a command with `env` (nil: ours), capturing stdout and stderr together.
cc_exec :: proc(cmd: []string, env: []string = nil, dir := "") -> (output: string, ok: bool) {
	desc := os.Process_Desc {
		command     = cmd,
		env         = env,
		working_dir = dir,
	}
	st, stdout, stderr, err := exec(desc, context.allocator)
	if err != nil {
		return fmt.aprintf("cannot run %s: %v", cmd[0], err), false
	}
	s := strings.concatenate({string(stdout), string(stderr)})
	delete(stdout)
	delete(stderr)
	return s, st.exited && st.exit_code == 0
}

// Runs every command, as many at once as there are cores; prints the output of the ones that fail.
cc_exec_all :: proc(cmds: [][]string, env: []string) -> bool {
	Job :: struct {
		cmds:   [][]string,
		env:    []string,
		next:   int,
		failed: bool,
		mutex:  sync.Mutex,
	}
	job := Job {
		cmds = cmds,
		env  = env,
	}
	worker :: proc(job: ^Job) {
		for {
			i := sync.atomic_add(&job.next, 1)
			if i >= len(job.cmds) {
				return
			}
			out, ok := cc_exec(job.cmds[i], job.env)
			if !ok {
				sync.mutex_lock(&job.mutex)
				fmt.eprintfln("%s\n%s", strings.join(job.cmds[i], " ", context.temp_allocator), out)
				job.failed = true
				sync.mutex_unlock(&job.mutex)
			}
			delete(out)
		}
	}
	n := clamp(os.get_processor_core_count(), 1, len(cmds))
	threads := make([]^thread.Thread, n, context.temp_allocator)
	for &t in threads {
		t = thread.create_and_start_with_poly_data(&job, worker)
	}
	for t in threads {
		thread.join(t)
		thread.destroy(t)
	}
	return !job.failed
}

// Compiles C sources (in parallel) to objects in obj_dir; returns the object paths.
cc_compile :: proc(c: Cc, flags, sources: []string, obj_dir: string) -> (objs: []string, ok: bool) {
	os.remove_all(obj_dir)
	if err := os.make_directory_all(obj_dir); err != nil {
		return nil, fail("cannot create %s: %v", obj_dir, err)
	}
	cmds := make([][]string, len(sources), context.temp_allocator)
	objs = make([]string, len(sources), context.temp_allocator)
	for src, i in sources {
		objs[i] = path_join(obj_dir, fmt.tprintf("%s%s", stem(src), c.msvc ? ".obj" : ".o"))
		cmd := make([dynamic]string, context.temp_allocator)
		append(&cmd, c.cc)
		append(&cmd, ..flags)
		if c.msvc {
			append(&cmd, "/c", src, fmt.tprintf("/Fo%s", objs[i]))
		} else {
			append(&cmd, "-c", src, "-o", objs[i])
		}
		cmds[i] = cmd[:]
	}
	return objs, cc_exec_all(cmds, cc_env())
}

// Archives objects into a static library.
cc_archive :: proc(c: Cc, objs: []string, lib: string) -> bool {
	os.remove(lib)
	cmd := make([dynamic]string, context.temp_allocator)
	if c.msvc {
		append(&cmd, c.ar, "/nologo", fmt.tprintf("/out:%s", lib))
	} else {
		append(&cmd, c.ar, "rcs", lib)
	}
	append(&cmd, ..objs)
	if out, ok := cc_exec(cmd[:], cc_env()); !ok {
		return fail("archiving %s failed:\n%s", lib, out)
	}
	return true
}

// Compiles and links an executable from sources plus libraries (static archives, import libraries or shared
// libraries, by path). `rpath`: a directory the executable finds shared libraries in (not Windows).
cc_link_exe :: proc(c: Cc, flags, sources, libs: []string, exe, obj_dir: string, rpath := "") -> bool {
	objs := cc_compile(c, flags, sources, obj_dir) or_return
	cmd := make([dynamic]string, context.temp_allocator)
	append(&cmd, c.cc)
	if c.msvc {
		append(&cmd, "/nologo", "/MT", fmt.tprintf("/Fe%s", exe))
		append(&cmd, ..objs)
		append(&cmd, "/link", "/INCREMENTAL:NO")
		append(&cmd, ..libs)
	} else {
		append(&cmd, ..flags)
		append(&cmd, "-o", exe)
		append(&cmd, ..objs)
		append(&cmd, ..libs)
		if rpath != "" {
			append(&cmd, fmt.tprintf("-Wl,-rpath,%s", rpath))
		}
		append(&cmd, "-lm")
		when ODIN_OS == .Linux {
			append(&cmd, "-lpthread")
		}
	}
	os.remove(exe)
	if out, ok := cc_exec(cmd[:], cc_env()); !ok {
		return fail("linking %s failed:\n%s", exe, out)
	}
	return true
}

when ODIN_OS == .Windows {
	@(private = "file")
	cc_vs_install_path :: proc() -> string {
		pf86 := os.get_env("ProgramFiles(x86)", context.temp_allocator)
		if pf86 == "" {
			pf86 = `C:\Program Files (x86)`
		}
		vswhere := path_join(pf86, "Microsoft Visual Studio", "Installer", "vswhere.exe")
		if !os.is_file(vswhere) {
			return ""
		}
		out, ok := cc_exec(
			{
				vswhere,
				"-latest",
				"-products",
				"*",
				"-requires",
				"Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
				"-property",
				"installationPath",
			},
		)
		return ok ? strings.trim_space(out) : ""
	}

	// The vcvarsall.bat x64 environment, unless this shell already is a developer prompt (INCLUDE and LIB set).
	@(private = "file")
	cc_msvc_env :: proc() -> []string {
		if os.get_env("INCLUDE", context.temp_allocator) != "" && os.get_env("LIB", context.temp_allocator) != "" {
			return nil
		}
		vs := cc_vs_install_path()
		if vs == "" {
			log("b3: warning: Visual Studio with the C++ tools not found (vswhere); using this shell's environment")
			return nil
		}
		vcvars := path_join(vs, "VC", "Auxiliary", "Build", "vcvarsall.bat")
		// Separate arguments: Odin quotes each one that needs it.
		out, ok := cc_exec({"cmd", "/c", "call", vcvars, "x64", ">nul", "&&", "set"})
		if !ok {
			log("b3: warning: vcvarsall.bat failed: %s", strings.trim_space(out))
			return nil
		}
		env: [dynamic]string
		text := out
		for line in strings.split_lines_iterator(&text) {
			if strings.index_byte(line, '=') > 0 {
				append(&env, strings.clone(strings.trim_right(line, "\r")))
			}
		}
		return env[:]
	}

	@(private = "file")
	cc_find_clang_cl :: proc() -> string {
		if p := which("clang-cl"); p != "" {
			return p
		}
		pf := os.get_env("ProgramFiles", context.temp_allocator)
		if pf == "" {
			pf = `C:\Program Files`
		}
		if c := path_join(pf, "LLVM", "bin", "clang-cl.exe"); os.is_file(c) {
			return c
		}
		if vs := cc_vs_install_path(); vs != "" {
			if c := path_join(vs, "VC", "Tools", "Llvm", "x64", "bin", "clang-cl.exe"); os.is_file(c) {
				return c
			}
		}
		return ""
	}
}

// The full path of an executable on PATH, or "".
which :: proc(name: string) -> string {
	sep := ODIN_OS == .Windows ? ";" : ":"
	exts := ODIN_OS == .Windows ? []string{".exe", ".cmd", ".bat", ""} : []string{""}
	p := os.get_env("PATH", context.temp_allocator)
	for dir in strings.split(p, sep, context.temp_allocator) {
		if dir == "" {
			continue
		}
		for ext in exts {
			cand := path_join(dir, fmt.tprintf("%s%s", name, ext))
			if os.is_file(cand) {
				return cand
			}
		}
	}
	return ""
}

// --- small helpers ------------------------------------------------------------------------------------------------

strs :: proc(s: ..string) -> []string {
	out := make([]string, len(s), context.temp_allocator)
	copy(out, s)
	return out
}

concat :: proc(lists: ..[]string) -> []string {
	out := make([dynamic]string, context.temp_allocator)
	for l in lists {
		append(&out, ..l)
	}
	return out[:]
}

path_join :: proc(parts: ..string) -> string {
	return strings.join(parts, "/", context.temp_allocator)
}

dir_of :: proc(p: string) -> string {
	i := max(strings.last_index_byte(p, '/'), strings.last_index_byte(p, '\\'))
	return i < 0 ? "." : p[:i]
}

base_of :: proc(p: string) -> string {
	i := max(strings.last_index_byte(p, '/'), strings.last_index_byte(p, '\\'))
	return p[i + 1:]
}

stem :: proc(p: string) -> string {
	b := base_of(p)
	if i := strings.last_index_byte(b, '.'); i > 0 {
		return b[:i]
	}
	return b
}

// The files in `dir` whose names end with `suffix`, sorted (full paths).
files_with_suffix :: proc(dir, suffix: string) -> []string {
	infos, err := os.read_all_directory_by_path(dir, context.temp_allocator)
	if err != nil {
		return nil
	}
	out := make([dynamic]string, context.temp_allocator)
	for fi in infos {
		if fi.type == .Regular && strings.has_suffix(fi.name, suffix) {
			append(&out, path_join(dir, fi.name))
		}
	}
	sort_strings(out[:])
	return out[:]
}

sort_strings :: proc(s: []string) {
	for i in 1 ..< len(s) {
		for j := i; j > 0 && s[j] < s[j - 1]; j -= 1 {
			s[j], s[j - 1] = s[j - 1], s[j]
		}
	}
}

sha256_hex :: proc(data: []byte) -> string {
	digest: [32]byte
	hash.hash_bytes_to_buffer(.SHA256, data, digest[:])
	return string(hex.encode(digest[:], context.temp_allocator))
}

read_text :: proc(p: string) -> (string, bool) {
	data, err := os.read_entire_file_from_path(p, context.temp_allocator)
	return string(data), err == nil
}

write_text :: proc(p: string, text: string) -> bool {
	os.make_directory_all(dir_of(p))
	if err := os.write_entire_file_from_string(p, text); err != nil {
		return fail("cannot write %s: %v", p, err)
	}
	return true
}

// A stamp file holds the key of what produced the files next to it.
stamp_ok :: proc(p, key: string) -> bool {
	text, ok := read_text(p)
	return ok && strings.trim_space(text) == key
}
