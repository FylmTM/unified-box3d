// Ported from box3d src/simd_wide.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

// Macros to avoid name clashes between W4 and W8 functions existing in the same code base.
// At the same time this helps to avoid code duplication.

// unified: the b3*W names are aliases of the W4 types and procs, for the wide4.c instantiations
// (contact_solver_wide.odin, convex_manifold_wide.odin), which keep upstream's names. B3_SIMD_WIDTH and
// B3_WIDE_ALIGNMENT are file-private constants of each instantiation (an alias can't be file-private in Odin), and
// `./b3 gen` writes the wide8.c instantiations with these names resolved to the W8 ones (PORTING.md, SIMD).
// unified end

FloatW :: FloatW4
Vec3W :: Vec3W4

ZeroW :: ZeroW4
SplatW :: SplatW4
SetW :: SetW4
LoadW :: LoadW4
StoreW :: StoreW4
NegW :: NegW4
AbsW :: AbsW4
AddW :: AddW4
SubW :: SubW4
MulW :: MulW4
DivW :: DivW4
SqrtW :: SqrtW4
MulAddW :: MulAddW4
MinW :: MinW4
MaxW :: MaxW4
AndW :: AndW4
OrW :: OrW4
AndNotW :: AndNotW4
SoftMaskW :: SoftMaskW4
GreaterThanW :: GreaterThanW4
LessThanW :: LessThanW4
EqualsW :: EqualsW4
AllZeroW :: AllZeroW4
AnyTrueW :: AnyTrueW4
BlendW :: BlendW4
EmbedIndexW :: EmbedIndexW4
MinIndexW :: MinIndexW4
TransposeW :: TransposeW4
LoadVW :: LoadVW4
StoreVW :: StoreVW4
SplatVW :: SplatVW4
NegVW :: NegVW4
MulSVW :: MulSVW4
MulSubSVW :: MulSubSVW4
MulAddSVW :: MulAddSVW4
SubVW :: SubVW4
AddVW :: AddVW4
DotW :: DotW4
CrossW :: CrossW4
