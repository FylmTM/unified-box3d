// Ported from box3d test/test_id.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "core:testing"

@(test)
IdTest :: proc(t: ^testing.T) {
	LOCK()
	x: u64 = 0x0123456789ABCDEF

	{
		id := b3.LoadBodyId(x)
		y := b3.StoreBodyId(id)
		if !ENSURE(t, x == y) {return}
	}

	{
		id := b3.LoadShapeId(x)
		y := b3.StoreShapeId(id)
		if !ENSURE(t, x == y) {return}
	}

	{
		id := b3.LoadJointId(x)
		y := b3.StoreJointId(id)
		if !ENSURE(t, x == y) {return}
	}
}
