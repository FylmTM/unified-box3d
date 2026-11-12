// `./b3 upstream`: the original Box3D at UPSTREAM_COMMIT, unpatched, as a static C library: the reference the
// comparison (`./b3 compare`) holds our library to. Sources in .out/upstream/src (a shallow fetch of that commit),
// the library in .out/upstream/lib/<platform>/, rebuilt only when the commit, flags or compiler change.
package tools

import "base:runtime"
import "core:fmt"
import "core:os"
import "core:strings"
import "core:time"

@(init)
register_upstream :: proc "contextless" () {
	context = runtime.default_context()
	register(Command {
		name = "upstream",
		usage = "",
		summary = "fetch upstream Box3D at the pinned commit and build it as a static C library",
		run = proc(args: []string) -> bool {
			if len(args) > 0 {
				return fail("upstream takes no arguments")
			}
			_, ok := ensure_upstream()
			return ok
		},
	})
}

// .out/upstream/src: the upstream checkout.
upstream_src :: proc() -> string {
	return out("upstream", "src")
}

// The upstream static library (built if needed); its path.
ensure_upstream :: proc() -> (lib: string, ok: bool) {
	fetch_upstream() or_return
	c := cc_toolchain() or_return
	src := upstream_src()
	lib_dir := out("upstream", "lib", platform())
	lib = path_join(lib_dir, c.msvc ? "box3d.lib" : "libbox3d.a")
	flags := upstream_flags(c)
	key := fmt.tprintf("%s %s %s", UPSTREAM_COMMIT, c.version, strings.join(flags, " ", context.temp_allocator))
	stamp := path_join(lib_dir, "box3d.stamp")
	if os.is_file(lib) && stamp_ok(stamp, key) {
		return lib, true
	}
	start := time.tick_now()
	sources := files_with_suffix(path_join(src, "src"), ".c")
	if len(sources) < 40 {
		return "", fail("upstream has only %d sources in %s/src", len(sources), src)
	}
	log("b3: building upstream Box3D (%d sources, %s)", len(sources), base_of(c.cc))
	objs := cc_compile(c, flags, sources, out("upstream", "obj", platform())) or_return
	os.make_directory_all(lib_dir)
	cc_archive(c, objs, lib) or_return
	write_text(stamp, key) or_return
	log("b3: upstream Box3D built in %.1fs", time.duration_seconds(time.tick_since(start)))
	return lib, true
}

// cc_flags plus upstream's public and internal headers: what upstream is built with, and what a program that calls
// its internals (bench/scale_original.c) compiles with.
upstream_flags :: proc(c: Cc) -> []string {
	src := upstream_src()
	inc := c.msvc ? "/I" : "-I"
	return concat(
		cc_flags(c),
		strs(fmt.tprintf("%s%s", inc, path_join(src, "include")), fmt.tprintf("%s%s", inc, path_join(src, "src"))),
	)
}

// A shallow checkout of UPSTREAM_COMMIT in .out/upstream/src (init + fetch of that one commit), unless it's there.
fetch_upstream :: proc() -> bool {
	dir := upstream_src()
	stamp := out("upstream", "src.stamp")
	if stamp_ok(stamp, UPSTREAM_COMMIT) && os.is_file(path_join(dir, "src", "physics_world.c")) {
		return true
	}
	git :: proc(dir: string, args: ..string) -> bool {
		cmd := make([dynamic]string, context.temp_allocator)
		append(&cmd, "git", "-c", "core.autocrlf=false", "-c", "advice.detachedHead=false", "-C", dir)
		append(&cmd, ..args)
		if output, ok := cc_exec(cmd[:]); !ok {
			return fail("git %s failed:\n%s", args[0], output)
		}
		return true
	}
	log("b3: fetching %s @ %s", UPSTREAM_URL, UPSTREAM_COMMIT[:12])
	if !os.is_directory(path_join(dir, ".git")) {
		os.remove_all(dir)
		os.make_directory_all(dir)
		git(dir, "init", "-q") or_return
		git(dir, "remote", "add", "origin", UPSTREAM_URL) or_return
	}
	git(dir, "fetch", "-q", "--depth", "1", "origin", UPSTREAM_COMMIT) or_return
	git(dir, "checkout", "-q", "--force", "--detach", UPSTREAM_COMMIT) or_return
	git(dir, "clean", "-q", "-fdx") or_return
	return write_text(stamp, UPSTREAM_COMMIT)
}
