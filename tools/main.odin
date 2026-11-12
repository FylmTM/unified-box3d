// The unified-box3d CLI (`./b3 help`). One file per command; each registers itself with `register` in an @(init)
// proc. Helpers here: running processes, paths, the pinned upstream commit.
package tools

import "base:runtime"

import "core:fmt"
import "core:os"
import "core:slice"
import "core:strings"
import "core:thread"
import "core:time"

// The upstream Box3D this repo ports (README.md).
UPSTREAM_URL :: "https://github.com/erincatto/box3d.git"
UPSTREAM_COMMIT :: "336263320469e5e9febfe18697ab562488f7e97a"

Command :: struct {
	name:    string,
	usage:   string, // arguments, after the name
	summary: string,
	run:     proc(args: []string) -> bool,
}

@(private)
commands: [dynamic]Command

register :: proc(c: Command) {
	append(&commands, c)
}

// The repository root (the directory holding tools/).
root :: proc() -> string {
	return strings.trim_suffix(strings.trim_right(#directory, "/\\"), "/tools") // #directory ends with a slash
}

path :: proc(parts: ..string) -> string {
	all := make([dynamic]string, context.temp_allocator)
	append(&all, root())
	append(&all, ..parts)
	return strings.join(all[:], "/", context.temp_allocator)
}

// Generated output, git-ignored.
out :: proc(parts: ..string) -> string {
	all := make([dynamic]string, context.temp_allocator)
	append(&all, ".out")
	append(&all, ..parts)
	return path(..all[:])
}

log :: proc(format: string, args: ..any) {
	fmt.eprintfln(format, ..args)
}

fail :: proc(format: string, args: ..any) -> bool {
	fmt.eprint("b3: ")
	fmt.eprintfln(format, ..args)
	return false
}

// Runs a command with our stdout and stderr; true if it exits 0.
run :: proc(cmd: []string, dir := "") -> bool {
	log("+ %s", strings.join(cmd, " ", context.temp_allocator))
	desc := os.Process_Desc {
		command     = cmd,
		working_dir = dir,
		stdin       = os.stdin,
		stdout      = os.stdout,
		stderr      = os.stderr,
	}
	p, err := os.process_start(desc)
	if err != nil {
		return fail("cannot start %s: %v", cmd[0], err)
	}
	st, werr := os.process_wait(p)
	if werr != nil {
		return fail("waiting for %s: %v", cmd[0], werr)
	}
	return st.exited && st.exit_code == 0
}

// os.process_exec, without its busy loop: it polls the child's pipes without ever blocking, so it keeps a core busy
// for as long as the child runs, which slows down whatever runs beside it (benchmarks most of all). Here the reads
// block: stdout on this thread, stderr on another.
exec :: proc(
	desc: os.Process_Desc,
	allocator := context.allocator,
) -> (
	state: os.Process_State,
	stdout, stderr: []u8,
	err: os.Error,
) {
	stdout_r, stdout_w := os.pipe() or_return
	defer os.close(stdout_r)
	stderr_r, stderr_w := os.pipe() or_return
	defer os.close(stderr_r)
	process: os.Process
	{
		// The write ends close in any case, so the reads see the end of the output.
		defer os.close(stdout_w)
		defer os.close(stderr_w)
		d := desc
		d.stdout = stdout_w
		d.stderr = stderr_w
		process = os.process_start(d) or_return
	}

	read_all :: proc(r: ^os.File, allocator: runtime.Allocator) -> []u8 {
		b := make([dynamic]u8, allocator)
		buf: [4096]u8 = ---
		for {
			n, rerr := os.read(r, buf[:])
			if n > 0 {
				append(&b, ..buf[:n])
			}
			if rerr != nil || n == 0 {
				return b[:]
			}
		}
	}
	Stderr_Job :: struct {
		r:         ^os.File,
		allocator: runtime.Allocator,
		out:       []u8,
	}
	job := Stderr_Job{stderr_r, allocator, nil}
	t := thread.create_and_start_with_poly_data(&job, proc(job: ^Stderr_Job) {
		job.out = read_all(job.r, job.allocator)
	})
	stdout = read_all(stdout_r, allocator)
	thread.join(t)
	thread.destroy(t)
	stderr = job.out
	state = os.process_wait(process) or_return
	return
}

// Runs a command and returns its stdout (stderr goes to ours).
run_capture :: proc(cmd: []string, dir := "") -> (output: string, ok: bool) {
	desc := os.Process_Desc {
		command     = cmd,
		working_dir = dir,
	}
	st, stdout, stderr, err := exec(desc, context.temp_allocator)
	if err != nil {
		return "", fail("cannot run %s: %v", cmd[0], err)
	}
	if len(stderr) > 0 {
		os.write(os.stderr, stderr)
	}
	return string(stdout), st.exited && st.exit_code == 0
}

// This platform's name in release files: windows-x64, linux-x64, linux-arm64, macos-arm64.
platform :: proc() -> string {
	when ODIN_OS == .Windows && ODIN_ARCH == .amd64 {
		return "windows-x64"
	} else when ODIN_OS == .Linux && ODIN_ARCH == .amd64 {
		return "linux-x64"
	} else when ODIN_OS == .Linux && ODIN_ARCH == .arm64 {
		return "linux-arm64"
	} else when ODIN_OS == .Darwin && ODIN_ARCH == .arm64 {
		return "macos-arm64"
	} else {
		#panic("unified-box3d: unsupported platform")
	}
}

main :: proc() {
	args := os.args[1:]
	if len(args) == 0 || args[0] == "help" || args[0] == "-h" {
		slice.sort_by(commands[:], proc(a, b: Command) -> bool {return a.name < b.name})
		fmt.println("usage: ./b3 <command> [args]   (b3.cmd on Windows)\n")
		for c in commands {
			fmt.printfln("  %-10s %-28s %s", c.name, c.usage, c.summary)
		}
		return
	}
	for c in commands {
		if c.name == args[0] {
			start := time.tick_now()
			ok := c.run(args[1:])
			log("b3: %s %s in %.1fs", c.name, ok ? "done" : "FAILED", time.duration_seconds(time.tick_since(start)))
			os.exit(ok ? 0 : 1)
		}
	}
	fmt.eprintfln("b3: unknown command %q (./b3 help)", args[0])
	os.exit(2)
}
