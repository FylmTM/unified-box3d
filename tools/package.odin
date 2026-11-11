// `./b3 package`: the release archive for this platform, as the release workflow publishes it. Builds the library
// (release and debug, `./b3 build`) and zips, under one top directory unified-box3d-<tag>-<platform>/:
//   lib/<platform>/         the release library (box3d.dll + box3d.lib import library, libbox3d.so, libbox3d.dylib)
//   lib/<platform>-debug/   the same unoptimized, with debug info
//   include/box3d/          the C headers (upstream's, plus unified.h)
//   bindings/               the Odin bindings
//   LICENSE, README.md
// into .out/package/unified-box3d-<tag>-<platform>.zip, plus a .sha256 line for SHA256SUMS. A tag is
// `<upstream commit prefix>-r<N>` (the commit is UPSTREAM_COMMIT); without --tag, `<12 hex>-dev`.
package tools

import "base:runtime"
import "core:fmt"
import "core:os"
import "core:strings"

@(init)
register_package :: proc "contextless" () {
	context = runtime.default_context()
	register(
		Command {
			name = "package",
			usage = "[--tag <commit>-r<N>]",
			summary = "build the release and debug libraries and zip them with include/, bindings/, LICENSE and README.md into .out/package/",
			run = cmd_package,
		},
	)
}

cmd_package :: proc(args: []string) -> bool {
	tag := fmt.tprintf("%s-dev", UPSTREAM_COMMIT[:12])
	for i := 0; i < len(args); i += 1 {
		if args[i] == "--tag" && i + 1 < len(args) {
			tag = args[i + 1]
			i += 1
			check_tag(tag) or_return
		} else {
			return fail("package: unknown argument %q (./b3 help)", args[i])
		}
	}

	// The libraries, through the build command.
	build: Maybe(Command)
	for c in commands {
		if c.name == "build" {
			build = c
		}
	}
	b, has_build := build.?
	if !has_build {
		return fail("package: there is no build command")
	}
	b.run(nil) or_return
	b.run(strs("--debug")) or_return

	name := fmt.tprintf("unified-box3d-%s-%s", tag, platform())
	stage := out("package", name)
	os.remove_all(out("package")) // only this archive in it (the release workflow uploads *.zip)
	for variant in ([]string{platform(), fmt.tprintf("%s-debug", platform())}) {
		src := path("lib", variant)
		files := lib_files(src)
		if len(files) == 0 {
			return fail("package: no library in %s", src)
		}
		for f in files {
			copy_into(path_join(stage, "lib", variant, base_of(f)), f) or_return
		}
	}
	for h in files_with_suffix(path("include", "box3d"), ".h") {
		copy_into(path_join(stage, "include", "box3d", base_of(h)), h) or_return
	}
	bindings := files_with_suffix(path("bindings"), ".odin")
	if len(bindings) == 0 {
		return fail("package: no bindings in %s", path("bindings"))
	}
	for f in bindings {
		copy_into(path_join(stage, "bindings", base_of(f)), f) or_return
	}
	for f in ([]string{"LICENSE", "README.md"}) {
		if !os.is_file(path(f)) {
			return fail("package: %s is missing", f)
		}
		copy_into(path_join(stage, f), path(f)) or_return
	}

	zip := out("package", fmt.tprintf("%s.zip", name))
	os.remove(zip)
	make_zip(out("package"), name, zip) or_return
	data, err := os.read_entire_file_from_path(zip, context.temp_allocator)
	if err != nil {
		return fail("package: cannot read %s: %v", zip, err)
	}
	sum := fmt.tprintf("%s  %s.zip\n", sha256_hex(data), name)
	write_text(fmt.tprintf("%s.sha256", zip), sum) or_return
	fmt.printf("%s (%d bytes)\n%s", zip, len(data), sum)
	return true
}

// A release tag: `<upstream commit prefix, 7 to 40 hex>-r<N>`, the prefix being UPSTREAM_COMMIT's.
@(private = "file")
check_tag :: proc(tag: string) -> bool {
	dash := strings.index(tag, "-r")
	if dash < 7 || dash > 40 || dash + 2 >= len(tag) {
		return fail("package: tag %q is not <upstream commit>-r<N>", tag)
	}
	commit, rev := tag[:dash], tag[dash + 2:]
	for ch in commit {
		if !strings.contains_rune("0123456789abcdef", ch) {
			return fail("package: tag %q: %q is not a commit (lowercase hex)", tag, commit)
		}
	}
	for ch in rev {
		if ch < '0' || ch > '9' {
			return fail("package: tag %q: the revision after -r must be a number", tag)
		}
	}
	if !strings.has_prefix(UPSTREAM_COMMIT, commit) {
		return fail("package: tag %q names upstream %s, but this repo ports %s", tag, commit, UPSTREAM_COMMIT)
	}
	return true
}

// The files of a built library directory (everything but build leftovers).
@(private = "file")
lib_files :: proc(dir: string) -> []string {
	infos, err := os.read_all_directory_by_path(dir, context.temp_allocator)
	if err != nil {
		return nil
	}
	out := make([dynamic]string, context.temp_allocator)
	for fi in infos {
		keep := false
		for ext in ([]string{".dll", ".lib", ".pdb", ".so", ".dylib"}) {
			keep = keep || strings.has_suffix(fi.name, ext)
		}
		if fi.type == .Regular && keep {
			append(&out, path_join(dir, fi.name))
		}
	}
	sort_strings(out[:])
	return out[:]
}

@(private = "file")
copy_into :: proc(dst, src: string) -> bool {
	os.make_directory_all(dir_of(dst))
	if err := os.copy_file(dst, src); err != nil {
		return fail("package: cannot copy %s: %v", src, err)
	}
	return true
}

// Zips `dir/name` (a directory) into `zip`, with paths starting at name/: Windows' bsdtar, elsewhere zip.
@(private = "file")
make_zip :: proc(dir, name, zip: string) -> bool {
	cmd: []string
	when ODIN_OS == .Windows {
		system := os.get_env("SystemRoot", context.temp_allocator)
		if system == "" {
			system = `C:\Windows`
		}
		// System32's tar is bsdtar (zip by the suffix with -a); Git's GNU tar can't write zip.
		cmd = strs(path_join(system, "System32", "tar.exe"), "-a", "-c", "-f", zip, name)
	} else {
		if which("zip") == "" {
			return fail("package: zip is not installed")
		}
		cmd = strs("zip", "-q", "-r", "-X", "-y", zip, name)
	}
	if output, ok := cc_exec(cmd, nil, dir); !ok {
		return fail("package: zipping failed:\n%s", output)
	}
	return true
}
