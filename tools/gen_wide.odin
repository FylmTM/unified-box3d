// `./b3 gen`, the wide8.c instantiations. Upstream compiles contact_solver_wide.inl and convex_manifold_wide.inl
// twice: at B3_SIMD_WIDTH 4 (wide4.c) and at 8 (wide8.c, with AVX2 enabled for the whole file), and picks one at run
// time. The port has the width-4 instantiations, hand-ported with the B3_SIMD_WIDTH == 8 branches kept; this writes
// the width-8 ones from them:
// - every name ending in W4 ends in W8 (B3_WIDE(name) is name##W8), and the aliases standing for W4 names
//   (simd_wide.odin's FloatW, AddW, ..., and the file's own Vec2W, ContactManifoldWide, ...) are replaced by the W8
//   names they would stand for (an alias can't be file-private in Odin, so each file can't have its own);
// - the file-private SIMD_WIDTH is 8;
// - every proc enables AVX2 (`enable_target_feature`, upstream's B3_AVX2_BEGIN), and the file builds only on x64
//   (B3_SIMD_HAS_WIDTH_8); the procs are called only on a CPU that has AVX2 (simd.odin, DetectSIMDWidth).
package tools

import "core:fmt"
import "core:os"
import "core:strings"

// The width-4 instantiations and the width-8 files written from them.
GEN_WIDE := [?][2]string {
	{"box3d/contact_solver_wide.odin", "box3d/contact_solver_wide8.odin"},
	{"box3d/convex_manifold_wide.odin", "box3d/convex_manifold_wide8.odin"},
}

AVX2_ATTRIBUTE :: `enable_target_feature = "avx2"`

// `Name :: TargetW4` at the start of a line: an alias of a W4 name (its target), or "" if the line is not one.
@(private = "file")
wide_alias :: proc(line: string) -> (name, target: string) {
	sep := strings.index(line, " :: ")
	if sep <= 0 || line[0] == '\t' {
		return
	}
	n, t := line[:sep], strings.trim_right_space(line[sep + 4:])
	if !strings.has_suffix(t, "W4") {
		return
	}
	for s in ([]string{n, t}) {
		for i in 0 ..< len(s) {
			if !is_ident_byte(s[i]) {
				return
			}
		}
	}
	return n, t
}

// The width-8 file for the width-4 one at src (repo-relative).
gen_wide8 :: proc(src: string) -> (text: string, ok: bool) {
	read :: proc(rel: string) -> (string, bool) {
		data, err := os.read_entire_file(path(rel), context.temp_allocator)
		if err != nil {
			return "", fail("gen: reading %s: %v", rel, err)
		}
		return string(data), true
	}
	aliases := make(map[string]string, context.temp_allocator)
	simd_wide := read("box3d/simd_wide.odin") or_return
	for line in strings.split_lines(simd_wide, context.temp_allocator) {
		if n, t := wide_alias(line); n != "" {
			aliases[n] = fmt.tprintf("%sW8", t[:len(t) - 2])
		}
	}
	source := read(src) or_return
	lines := strings.split_lines(source, context.temp_allocator)
	for line in lines {
		if n, t := wide_alias(line); n != "" {
			aliases[n] = fmt.tprintf("%sW8", t[:len(t) - 2])
		}
	}

	// The names: an alias becomes its W8 target, any other name ending in W4 ends in W8.
	rename :: proc(line: string, aliases: map[string]string) -> string {
		b := strings.builder_make(context.temp_allocator)
		i := 0
		for i < len(line) {
			if !is_ident_byte(line[i]) {
				strings.write_byte(&b, line[i])
				i += 1
				continue
			}
			j := i
			for j < len(line) && is_ident_byte(line[j]) {
				j += 1
			}
			word := line[i:j]
			if t, found := aliases[word]; found && !(line[i] >= '0' && line[i] <= '9') {
				strings.write_string(&b, t)
			} else if strings.has_suffix(word, "W4") && !(line[i] >= '0' && line[i] <= '9') {
				strings.write_string(&b, word[:len(word) - 2])
				strings.write_string(&b, "W8")
			} else {
				strings.write_string(&b, word)
			}
			i = j
		}
		return strings.to_string(b)
	}

	out := make([dynamic]string, context.temp_allocator)
	append(&out, "#+build amd64")
	width_set, intro_done := false, false
	skip_intro := false
	for line, k in lines {
		if k == len(lines) - 1 && line == "" {
			break
		}
		if n, _ := wide_alias(line); n != "" {
			continue // its uses are renamed
		}
		if skip_intro {
			if strings.has_prefix(line, "// ") {
				continue
			}
			skip_intro = false
		}
		if strings.has_prefix(line, "// This is the wide4.c instantiation") {
			append(
				&out,
				"// This is the wide8.c instantiation (B3_SIMD_WIDTH 8): B3_WIDE(name) is name##W8, and every proc enables",
				"// AVX2 (B3_AVX2_BEGIN). It is called only on a CPU that has AVX2 (simd.odin, DetectSIMDWidth).",
			)
			intro_done = true
			skip_intro = true
			continue
		}
		if line == "SIMD_WIDTH :: 4" {
			append(&out, "SIMD_WIDTH :: 8")
			width_set = true
			continue
		}
		renamed := rename(line, aliases)
		if k == 0 {
			append(&out, renamed, fmt.tprintf("// %s From %s, at width 8 (tools/gen_wide.odin).", GEN_NOTE, src))
			continue
		}

		// A proc declaration: `Name :: proc(`, `Name :: proc "contextless" (`, `Name :: #force_inline proc...`.
		body := strings.trim_left(renamed, "\t")
		indent := renamed[:len(renamed) - len(body)]
		if sep := strings.index(body, " :: "); sep > 0 {
			rest := strings.trim_prefix(body[sep + 4:], "#force_inline ")
			is_name := true
			for c in transmute([]u8)body[:sep] {
				is_name = is_name && is_ident_byte(c)
			}
			if is_name && (strings.has_prefix(rest, "proc(") || strings.has_prefix(rest, "proc \"")) {
				prev := len(out) > 0 ? strings.trim_left(out[len(out) - 1], "\t") : ""
				if strings.has_prefix(prev, "@(") && strings.has_suffix(prev, ")") {
					last := out[len(out) - 1]
					out[len(out) - 1] = fmt.tprintf("%s, %s)", last[:len(last) - 1], AVX2_ATTRIBUTE)
				} else {
					append(&out, fmt.tprintf("%s@(%s)", indent, AVX2_ATTRIBUTE))
				}
			}
		}
		append(&out, renamed)
	}
	if !width_set || !intro_done {
		return "", fail("gen: %s has no `SIMD_WIDTH :: 4` line or no wide4.c intro comment", src)
	}
	append(&out, "")
	return strings.join(out[:], "\n", context.temp_allocator), true
}
