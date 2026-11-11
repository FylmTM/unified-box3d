// Ported from box3d src/rapidhash.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Nicolas De Carli
// SPDX-License-Identifier: MIT
package box3d

import "base:intrinsics"

/*
 * rapidhash V3 - Very fast, high quality, platform-independent hashing algorithm.
 *
 * Based on 'wyhash', by Wang Yi <godspeed_china@yeah.net>
 *
 * Copyright (C) 2025 Nicolas De Carli
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * You can contact the author at:
 *   - rapidhash source repository: https://github.com/Nicoshev/rapidhash
 */

/*
 *  Unrolled macro.
 *  Improves large input speed, but increases code size and worsens small input speed.
 *
 *  RAPIDHASH_COMPACT: Normal behavior.
 *  RAPIDHASH_UNROLLED:
 *
 */
RAPIDHASH_UNROLLED :: false

/*
 *  Protection macro, alters behaviour of rapid_mum multiplication function.
 *
 *  RAPIDHASH_FAST: Normal behavior, max speed.
 *  RAPIDHASH_PROTECTED: Extra protection against entropy loss.
 */
RAPIDHASH_PROTECTED :: false

// unified: the endianness macros are ODIN_ENDIAN; the likely/unlikely hints are dropped.

/*
 *  Default secret parameters.
 */
@(rodata)
rapid_secret := [8]u64 {
	0x2d358dccaa6c78a5,
	0x8bb84b93962eacc9,
	0x4b33a62ed433d4a3,
	0x4d5a2da51de1aa47,
	0xa0761d6478bd642f,
	0xe7037ed1a0b428db,
	0x90ed1765281c388c,
	0xaaaaaaaaaaaaaaaa,
}

/*
 *  64*64 -> 128bit multiply function.
 *
 *  @param A  Address of 64-bit number.
 *  @param B  Address of 64-bit number.
 *
 *  Calculates 128-bit C = *A * *B.
 *
 *  When RAPIDHASH_FAST is defined:
 *  Overwrites A contents with C's low 64 bits.
 *  Overwrites B contents with C's high 64 bits.
 *
 *  When RAPIDHASH_PROTECTED is defined:
 *  Xors and overwrites A contents with C's low 64 bits.
 *  Xors and overwrites B contents with C's high 64 bits.
 */
rapid_mum :: #force_inline proc "contextless" (A: ^u64, B: ^u64) {
	// unified: the __uint128_t branch, on Odin's u128 (the MSVC and portable branches compute the same)
	r := u128(A^)
	r *= u128(B^)
	when RAPIDHASH_PROTECTED {
		A^ ~= u64(r)
		B^ ~= u64(r >> 64)
	} else {
		A^ = u64(r)
		B^ = u64(r >> 64)
	}
	// unified end
}

/*
 *  Multiply and xor mix function.
 *
 *  @param A  64-bit number.
 *  @param B  64-bit number.
 *
 *  Calculates 128-bit C = A * B.
 *  Returns 64-bit xor between high and low 64 bits of C.
 */
rapid_mix :: #force_inline proc "contextless" (A: u64, B: u64) -> u64 {
	A, B := A, B
	rapid_mum(&A, &B)
	return A ~ B
}

/*
 *  Read functions.
 */
when ODIN_ENDIAN == .Little {
	rapid_read64 :: #force_inline proc "contextless" (p: [^]u8) -> u64 {
		return intrinsics.unaligned_load((^u64)(p))
	}
	rapid_read32 :: #force_inline proc "contextless" (p: [^]u8) -> u64 {
		return u64(intrinsics.unaligned_load((^u32)(p)))
	}
} else {
	rapid_read64 :: #force_inline proc "contextless" (p: [^]u8) -> u64 {
		return intrinsics.byte_swap(intrinsics.unaligned_load((^u64)(p)))
	}
	rapid_read32 :: #force_inline proc "contextless" (p: [^]u8) -> u64 {
		return u64(intrinsics.byte_swap(intrinsics.unaligned_load((^u32)(p))))
	}
}

/*
 *  rapidhash main function.
 *
 *  @param key     Buffer to be hashed.
 *  @param len     @key length, in bytes.
 *  @param seed    64-bit seed used to alter the hash result predictably.
 *  @param secret  Triplet of 64-bit secrets used to alter hash result predictably.
 *
 *  Returns a 64-bit hash.
 */
rapidhash_internal :: #force_inline proc "contextless" (key: rawptr, len: int, seed: u64, secret: [^]u64) -> u64 {
	seed := seed
	p := ([^]u8)(key)
	seed ~= rapid_mix(seed ~ secret[2], secret[1])
	a, b: u64 = 0, 0
	i := uint(len)
	if len <= 16 {
		if len >= 4 {
			seed ~= u64(len)
			if len >= 8 {
				plast := p[len - 8:]
				a = rapid_read64(p)
				b = rapid_read64(plast)
			} else {
				plast := p[len - 4:]
				a = rapid_read32(p)
				b = rapid_read32(plast)
			}
		} else if len > 0 {
			a = (u64(p[0]) << 45) | u64(p[len - 1])
			b = u64(p[len >> 1])
		} else {
			a = 0
			b = 0
		}
	} else {
		if len > 112 {
			see1, see2 := seed, seed
			see3, see4 := seed, seed
			see5, see6 := seed, seed
			when !RAPIDHASH_UNROLLED {
				for {
					seed = rapid_mix(rapid_read64(p) ~ secret[0], rapid_read64(p[8:]) ~ seed)
					see1 = rapid_mix(rapid_read64(p[16:]) ~ secret[1], rapid_read64(p[24:]) ~ see1)
					see2 = rapid_mix(rapid_read64(p[32:]) ~ secret[2], rapid_read64(p[40:]) ~ see2)
					see3 = rapid_mix(rapid_read64(p[48:]) ~ secret[3], rapid_read64(p[56:]) ~ see3)
					see4 = rapid_mix(rapid_read64(p[64:]) ~ secret[4], rapid_read64(p[72:]) ~ see4)
					see5 = rapid_mix(rapid_read64(p[80:]) ~ secret[5], rapid_read64(p[88:]) ~ see5)
					see6 = rapid_mix(rapid_read64(p[96:]) ~ secret[6], rapid_read64(p[104:]) ~ see6)
					p = p[112:]
					i -= 112
					if !(i > 112) {
						break
					}
				}
			} else {
				for i > 224 {
					seed = rapid_mix(rapid_read64(p) ~ secret[0], rapid_read64(p[8:]) ~ seed)
					see1 = rapid_mix(rapid_read64(p[16:]) ~ secret[1], rapid_read64(p[24:]) ~ see1)
					see2 = rapid_mix(rapid_read64(p[32:]) ~ secret[2], rapid_read64(p[40:]) ~ see2)
					see3 = rapid_mix(rapid_read64(p[48:]) ~ secret[3], rapid_read64(p[56:]) ~ see3)
					see4 = rapid_mix(rapid_read64(p[64:]) ~ secret[4], rapid_read64(p[72:]) ~ see4)
					see5 = rapid_mix(rapid_read64(p[80:]) ~ secret[5], rapid_read64(p[88:]) ~ see5)
					see6 = rapid_mix(rapid_read64(p[96:]) ~ secret[6], rapid_read64(p[104:]) ~ see6)
					seed = rapid_mix(rapid_read64(p[112:]) ~ secret[0], rapid_read64(p[120:]) ~ seed)
					see1 = rapid_mix(rapid_read64(p[128:]) ~ secret[1], rapid_read64(p[136:]) ~ see1)
					see2 = rapid_mix(rapid_read64(p[144:]) ~ secret[2], rapid_read64(p[152:]) ~ see2)
					see3 = rapid_mix(rapid_read64(p[160:]) ~ secret[3], rapid_read64(p[168:]) ~ see3)
					see4 = rapid_mix(rapid_read64(p[176:]) ~ secret[4], rapid_read64(p[184:]) ~ see4)
					see5 = rapid_mix(rapid_read64(p[192:]) ~ secret[5], rapid_read64(p[200:]) ~ see5)
					see6 = rapid_mix(rapid_read64(p[208:]) ~ secret[6], rapid_read64(p[216:]) ~ see6)
					p = p[224:]
					i -= 224
				}
				if i > 112 {
					seed = rapid_mix(rapid_read64(p) ~ secret[0], rapid_read64(p[8:]) ~ seed)
					see1 = rapid_mix(rapid_read64(p[16:]) ~ secret[1], rapid_read64(p[24:]) ~ see1)
					see2 = rapid_mix(rapid_read64(p[32:]) ~ secret[2], rapid_read64(p[40:]) ~ see2)
					see3 = rapid_mix(rapid_read64(p[48:]) ~ secret[3], rapid_read64(p[56:]) ~ see3)
					see4 = rapid_mix(rapid_read64(p[64:]) ~ secret[4], rapid_read64(p[72:]) ~ see4)
					see5 = rapid_mix(rapid_read64(p[80:]) ~ secret[5], rapid_read64(p[88:]) ~ see5)
					see6 = rapid_mix(rapid_read64(p[96:]) ~ secret[6], rapid_read64(p[104:]) ~ see6)
					p = p[112:]
					i -= 112
				}
			}
			seed ~= see1
			see2 ~= see3
			see4 ~= see5
			seed ~= see6
			see2 ~= see4
			seed ~= see2
		}
		if i > 16 {
			seed = rapid_mix(rapid_read64(p) ~ secret[2], rapid_read64(p[8:]) ~ seed)
			if i > 32 {
				seed = rapid_mix(rapid_read64(p[16:]) ~ secret[2], rapid_read64(p[24:]) ~ seed)
				if i > 48 {
					seed = rapid_mix(rapid_read64(p[32:]) ~ secret[1], rapid_read64(p[40:]) ~ seed)
					if i > 64 {
						seed = rapid_mix(rapid_read64(p[48:]) ~ secret[1], rapid_read64(p[56:]) ~ seed)
						if i > 80 {
							seed = rapid_mix(rapid_read64(p[64:]) ~ secret[2], rapid_read64(p[72:]) ~ seed)
							if i > 96 {
								seed = rapid_mix(rapid_read64(p[80:]) ~ secret[1], rapid_read64(p[88:]) ~ seed)
							}
						}
					}
				}
			}
		}
		a = rapid_read64(p[i - 16:]) ~ u64(i)
		b = rapid_read64(p[i - 8:])
	}
	a ~= secret[1]
	b ~= seed
	rapid_mum(&a, &b)
	return rapid_mix(a ~ secret[7], b ~ secret[1] ~ u64(i))
}

/*
 *  rapidhashMicro main function.
 *
 *  @param key     Buffer to be hashed.
 *  @param len     @key length, in bytes.
 *  @param seed    64-bit seed used to alter the hash result predictably.
 *  @param secret  Triplet of 64-bit secrets used to alter hash result predictably.
 *
 *  Returns a 64-bit hash.
 */
rapidhashMicro_internal :: #force_inline proc "contextless" (key: rawptr, len: int, seed: u64, secret: [^]u64) -> u64 {
	seed := seed
	p := ([^]u8)(key)
	seed ~= rapid_mix(seed ~ secret[2], secret[1])
	a, b: u64 = 0, 0
	i := uint(len)
	if len <= 16 {
		if len >= 4 {
			seed ~= u64(len)
			if len >= 8 {
				plast := p[len - 8:]
				a = rapid_read64(p)
				b = rapid_read64(plast)
			} else {
				plast := p[len - 4:]
				a = rapid_read32(p)
				b = rapid_read32(plast)
			}
		} else if len > 0 {
			a = (u64(p[0]) << 45) | u64(p[len - 1])
			b = u64(p[len >> 1])
		} else {
			a = 0
			b = 0
		}
	} else {
		if i > 80 {
			see1, see2 := seed, seed
			see3, see4 := seed, seed
			for {
				seed = rapid_mix(rapid_read64(p) ~ secret[0], rapid_read64(p[8:]) ~ seed)
				see1 = rapid_mix(rapid_read64(p[16:]) ~ secret[1], rapid_read64(p[24:]) ~ see1)
				see2 = rapid_mix(rapid_read64(p[32:]) ~ secret[2], rapid_read64(p[40:]) ~ see2)
				see3 = rapid_mix(rapid_read64(p[48:]) ~ secret[3], rapid_read64(p[56:]) ~ see3)
				see4 = rapid_mix(rapid_read64(p[64:]) ~ secret[4], rapid_read64(p[72:]) ~ see4)
				p = p[80:]
				i -= 80
				if !(i > 80) {
					break
				}
			}
			seed ~= see1
			see2 ~= see3
			seed ~= see4
			seed ~= see2
		}
		if i > 16 {
			seed = rapid_mix(rapid_read64(p) ~ secret[2], rapid_read64(p[8:]) ~ seed)
			if i > 32 {
				seed = rapid_mix(rapid_read64(p[16:]) ~ secret[2], rapid_read64(p[24:]) ~ seed)
				if i > 48 {
					seed = rapid_mix(rapid_read64(p[32:]) ~ secret[1], rapid_read64(p[40:]) ~ seed)
					if i > 64 {
						seed = rapid_mix(rapid_read64(p[48:]) ~ secret[1], rapid_read64(p[56:]) ~ seed)
					}
				}
			}
		}
		a = rapid_read64(p[i - 16:]) ~ u64(i)
		b = rapid_read64(p[i - 8:])
	}
	a ~= secret[1]
	b ~= seed
	rapid_mum(&a, &b)
	return rapid_mix(a ~ secret[7], b ~ secret[1] ~ u64(i))
}

/*
 *  rapidhashNano main function.
 *
 *  @param key     Buffer to be hashed.
 *  @param len     @key length, in bytes.
 *  @param seed    64-bit seed used to alter the hash result predictably.
 *  @param secret  Triplet of 64-bit secrets used to alter hash result predictably.
 *
 *  Returns a 64-bit hash.
 */
rapidhashNano_internal :: #force_inline proc "contextless" (key: rawptr, len: int, seed: u64, secret: [^]u64) -> u64 {
	seed := seed
	p := ([^]u8)(key)
	seed ~= rapid_mix(seed ~ secret[2], secret[1])
	a, b: u64 = 0, 0
	i := uint(len)
	if len <= 16 {
		if len >= 4 {
			seed ~= u64(len)
			if len >= 8 {
				plast := p[len - 8:]
				a = rapid_read64(p)
				b = rapid_read64(plast)
			} else {
				plast := p[len - 4:]
				a = rapid_read32(p)
				b = rapid_read32(plast)
			}
		} else if len > 0 {
			a = (u64(p[0]) << 45) | u64(p[len - 1])
			b = u64(p[len >> 1])
		} else {
			a = 0
			b = 0
		}
	} else {
		if i > 48 {
			see1, see2 := seed, seed
			for {
				seed = rapid_mix(rapid_read64(p) ~ secret[0], rapid_read64(p[8:]) ~ seed)
				see1 = rapid_mix(rapid_read64(p[16:]) ~ secret[1], rapid_read64(p[24:]) ~ see1)
				see2 = rapid_mix(rapid_read64(p[32:]) ~ secret[2], rapid_read64(p[40:]) ~ see2)
				p = p[48:]
				i -= 48
				if !(i > 48) {
					break
				}
			}
			seed ~= see1
			seed ~= see2
		}
		if i > 16 {
			seed = rapid_mix(rapid_read64(p) ~ secret[2], rapid_read64(p[8:]) ~ seed)
			if i > 32 {
				seed = rapid_mix(rapid_read64(p[16:]) ~ secret[2], rapid_read64(p[24:]) ~ seed)
			}
		}
		a = rapid_read64(p[i - 16:]) ~ u64(i)
		b = rapid_read64(p[i - 8:])
	}
	a ~= secret[1]
	b ~= seed
	rapid_mum(&a, &b)
	return rapid_mix(a ~ secret[7], b ~ secret[1] ~ u64(i))
}

/*
 *  rapidhash seeded hash function.
 *
 *  @param key     Buffer to be hashed.
 *  @param len     @key length, in bytes.
 *  @param seed    64-bit seed used to alter the hash result predictably.
 *
 *  Calls rapidhash_internal using provided parameters and default secrets.
 *
 *  Returns a 64-bit hash.
 */
rapidhash_withSeed :: #force_inline proc "contextless" (key: rawptr, len: int, seed: u64) -> u64 {
	return rapidhash_internal(key, len, seed, &rapid_secret[0])
}

/*
 *  rapidhash general purpose hash function.
 *
 *  @param key     Buffer to be hashed.
 *  @param len     @key length, in bytes.
 *
 *  Calls rapidhash_withSeed using provided parameters and the default seed.
 *
 *  Returns a 64-bit hash.
 */
rapidhash :: #force_inline proc "contextless" (key: rawptr, len: int) -> u64 {
	return rapidhash_withSeed(key, len, 0)
}

/*
 *  rapidhashMicro seeded hash function.
 *
 *  Designed for HPC and server applications, where cache misses make a noticeable performance detriment.
 *  Clang-18+ compiles it to ~140 instructions without stack usage, both on x86-64 and aarch64.
 *  Faster for sizes up to 512 bytes, just 15%-20% slower for inputs above 1kb.
 *
 *  @param key     Buffer to be hashed.
 *  @param len     @key length, in bytes.
 *  @param seed    64-bit seed used to alter the hash result predictably.
 *
 *  Calls rapidhash_internal using provided parameters and default secrets.
 *
 *  Returns a 64-bit hash.
 */
rapidhashMicro_withSeed :: #force_inline proc "contextless" (key: rawptr, len: int, seed: u64) -> u64 {
	return rapidhashMicro_internal(key, len, seed, &rapid_secret[0])
}

/*
 *  rapidhashMicro hash function.
 *
 *  @param key     Buffer to be hashed.
 *  @param len     @key length, in bytes.
 *
 *  Calls rapidhash_withSeed using provided parameters and the default seed.
 *
 *  Returns a 64-bit hash.
 */
rapidhashMicro :: #force_inline proc "contextless" (key: rawptr, len: int) -> u64 {
	return rapidhashMicro_withSeed(key, len, 0)
}

/*
 *  rapidhashNano seeded hash function.
 *
 *  @param key     Buffer to be hashed.
 *  @param len     @key length, in bytes.
 *  @param seed    64-bit seed used to alter the hash result predictably.
 *
 *  Calls rapidhash_internal using provided parameters and default secrets.
 *
 *  Returns a 64-bit hash.
 */
rapidhashNano_withSeed :: #force_inline proc "contextless" (key: rawptr, len: int, seed: u64) -> u64 {
	return rapidhashNano_internal(key, len, seed, &rapid_secret[0])
}

/*
 *  rapidhashNano hash function.
 *
 *  Designed for Mobile and embedded applications, where keeping a small code size is a top priority.
 *  Clang-18+ compiles it to less than 100 instructions without stack usage, both on x86-64 and aarch64.
 *  The fastest for sizes up to 48 bytes, but may be considerably slower for larger inputs.
 *
 *  @param key     Buffer to be hashed.
 *  @param len     @key length, in bytes.
 *
 *  Calls rapidhash_withSeed using provided parameters and the default seed.
 *
 *  Returns a 64-bit hash.
 */
rapidhashNano :: #force_inline proc "contextless" (key: rawptr, len: int) -> u64 {
	return rapidhashNano_withSeed(key, len, 0)
}
