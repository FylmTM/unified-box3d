// Ported from box3d test/test_dynamic_tree.c at 3362633 (src/box3d_port/PORTING.md, "Tests").
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
package test_box3d_upstream

import b3 "../../box3d"
import "base:runtime"
import "core:mem"
import "core:os"
import "core:slice"
import "core:testing"

@(private = "file")
PROXY_COUNT :: 300

@(private = "file")
s_seed: u32 = 12345

@(private = "file")
NextRandom :: proc() -> u32 {
	s_seed = 1664525 * s_seed + 1013904223
	return s_seed
}

@(private = "file")
RandomFloat :: proc(lo: f32, hi: f32) -> f32 {
	t := f32(NextRandom() >> 8) / f32(1 << 24)
	return lo + t * (hi - lo)
}

@(private = "file")
RandomBox :: proc(extent: f32, size: f32) -> b3.AABB {
	center := b3.Vec3{RandomFloat(-extent, extent), RandomFloat(-extent, extent), RandomFloat(-extent, extent)}
	half := b3.Vec3{RandomFloat(0.1, size), RandomFloat(0.1, size), RandomFloat(0.1, size)}
	return b3.AABB{b3.Sub(center, half), b3.Add(center, half)}
}

@(private = "file")
HitSet :: struct {
	ids:   [PROXY_COUNT]i32,
	count: i32,
	bad:   bool,
}

// Every test seeds user data with the index of the box it created the proxy from, so the sets
// are keyed on user data. Proxy ids recycle, box indices do not.
// A callback cannot fail the test directly, so bad news rides back on the context.
@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
CollectCallback :: proc "c" (proxyId: i32, userData: u64, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	set := (^HitSet)(context_)
	if set.count >= PROXY_COUNT || i32(userData) < 0 || i32(userData) >= PROXY_COUNT {
		set.bad = true
		return false
	}

	_ = proxyId
	set.ids[set.count] = i32(userData)
	set.count += 1
	return true
}

@(private = "file")
SameSet :: proc(t: ^testing.T, a: ^HitSet, b: ^HitSet) -> bool {
	ENSURE(t, a.bad == false) or_return
	ENSURE(t, b.bad == false) or_return
	ENSURE(t, a.count == b.count) or_return
	// unified: qsort with CompareInt is slice.sort (same ascending order of ints)
	slice.sort(a.ids[:a.count])
	slice.sort(b.ids[:b.count])
	// unified end
	for i: i32 = 0; i < a.count; i += 1 {
		ENSURE(t, a.ids[i] == b.ids[i]) or_return
	}

	return true
}

// The tree must agree with a linear scan over the stored boxes. This is the gate on the
// SIMD box helpers, which have three separate lane layouts.
@(private = "file")
TreeQueryBruteForce :: proc(t: ^testing.T) -> bool {
	s_seed = 12345

	tree := b3.DynamicTree_Create(16)

	boxes: [PROXY_COUNT]b3.AABB
	proxies: [PROXY_COUNT]i32

	for i: i32 = 0; i < PROXY_COUNT; i += 1 {
		boxes[i] = RandomBox(6.0, 1.5)
		proxies[i] = b3.DynamicTree_CreateProxy(&tree, boxes[i], b3.DEFAULT_CATEGORY_BITS, u64(i))
		ENSURE(t, proxies[i] == i) or_return
	}

	b3.DynamicTree_Validate(&tree)
	ENSURE(t, b3.DynamicTree_GetProxyCount(&tree) == PROXY_COUNT) or_return

	totalHits: i32 = 0
	for round: i32 = 0; round < 200; round += 1 {
		queryBox := RandomBox(7.0, 3.0)

		treeHits := HitSet{}
		b3.DynamicTree_Query(&tree, queryBox, b3.DEFAULT_MASK_BITS, false, CollectCallback, &treeHits)

		bruteHits := HitSet{}
		for i: i32 = 0; i < PROXY_COUNT; i += 1 {
			if b3.AABB_Overlaps(boxes[i], queryBox) {
				bruteHits.ids[bruteHits.count] = i
				bruteHits.count += 1
			}
		}

		ENSURE(t, SameSet(t, &treeHits, &bruteHits)) or_return
		totalHits += treeHits.count
	}

	// Guard against the comparison above agreeing on nothing
	ENSURE(t, totalHits > 1000) or_return

	b3.DynamicTree_Destroy(&tree)
	return true
}

// Moving and enlarging every proxy exercises remove, insert, the ordered flag, the sweep refit
// and the partial rebuild, then the query has to still agree with the scan.
@(private = "file")
TreeMoveRefitRebuild :: proc(t: ^testing.T) -> bool {
	s_seed = 777

	tree := b3.DynamicTree_Create(16)

	boxes: [PROXY_COUNT]b3.AABB
	proxies: [PROXY_COUNT]i32

	for i: i32 = 0; i < PROXY_COUNT; i += 1 {
		boxes[i] = RandomBox(6.0, 1.0)
		proxies[i] = b3.DynamicTree_CreateProxy(&tree, boxes[i], b3.DEFAULT_CATEGORY_BITS, u64(i))
	}

	b3.DynamicTree_Rebuild(&tree, true)
	b3.DynamicTree_Validate(&tree)
	ENSURE(t, tree.dfsOrdered) or_return

	totalHits: i32 = 0
	for round: i32 = 0; round < 8; round += 1 {
		// Half the proxies move, half grow in place
		for i: i32 = 0; i < PROXY_COUNT; i += 1 {
			if (i & 1) == 0 {
				boxes[i] = RandomBox(6.0, 1.0)
				b3.DynamicTree_MoveProxy(&tree, proxies[i], boxes[i])
			} else {
				grown := b3.AABB_Inflate(boxes[i], 0.35)
				boxes[i] = grown
				b3.DynamicTree_EnlargeProxy(&tree, proxies[i], grown)
			}
		}

		b3.DynamicTree_Refit(&tree)
		b3.DynamicTree_Validate(&tree)

		b3.DynamicTree_Rebuild(&tree, false)
		b3.DynamicTree_Validate(&tree)
		b3.DynamicTree_ValidateNoMoved(&tree)
		ENSURE(t, tree.dfsOrdered) or_return

		// Every stored box must still be inside its leaf and inside the root
		rootBounds := b3.DynamicTree_GetRootBounds(&tree)
		for i: i32 = 0; i < PROXY_COUNT; i += 1 {
			ENSURE(t, b3.AABB_Contains(b3.DynamicTree_GetAABB(&tree, proxies[i]), boxes[i])) or_return
			ENSURE(t, b3.AABB_Contains(rootBounds, boxes[i])) or_return
			ENSURE(t, i32(b3.DynamicTree_GetUserData(&tree, proxies[i])) == i) or_return
		}

		queryBox := RandomBox(7.0, 3.0)

		treeHits := HitSet{}
		b3.DynamicTree_Query(&tree, queryBox, b3.DEFAULT_MASK_BITS, false, CollectCallback, &treeHits)

		bruteHits := HitSet{}
		for i: i32 = 0; i < PROXY_COUNT; i += 1 {
			if b3.AABB_Overlaps(boxes[i], queryBox) {
				bruteHits.ids[bruteHits.count] = i
				bruteHits.count += 1
			}
		}

		ENSURE(t, SameSet(t, &treeHits, &bruteHits)) or_return
		totalHits += treeHits.count
	}

	// Guard against the comparison above agreeing on nothing
	ENSURE(t, totalHits > 50) or_return

	b3.DynamicTree_Destroy(&tree)
	return true
}

// Destroying proxies in a scrambled order recycles sibling pairs through the free list and
// leaves the array unordered, which is the path the fallback refit has to handle.
@(private = "file")
TreeDestroyScrambled :: proc(t: ^testing.T) -> bool {
	s_seed = 4242

	tree := b3.DynamicTree_Create(16)

	boxes: [PROXY_COUNT]b3.AABB
	proxies: [PROXY_COUNT]i32
	alive: [PROXY_COUNT]bool

	for i: i32 = 0; i < PROXY_COUNT; i += 1 {
		boxes[i] = RandomBox(10.0, 0.8)
		proxies[i] = b3.DynamicTree_CreateProxy(&tree, boxes[i], b3.DEFAULT_CATEGORY_BITS, u64(i))
		alive[i] = true
	}

	for i: i32 = 0; i < PROXY_COUNT; i += 3 {
		b3.DynamicTree_DestroyProxy(&tree, proxies[i])
		alive[i] = false
	}

	b3.DynamicTree_Validate(&tree)

	// Re-create into the holes
	for i: i32 = 0; i < PROXY_COUNT; i += 3 {
		boxes[i] = RandomBox(10.0, 0.8)
		proxies[i] = b3.DynamicTree_CreateProxy(&tree, boxes[i], b3.DEFAULT_CATEGORY_BITS, u64(i))
		alive[i] = true
	}

	b3.DynamicTree_Validate(&tree)
	ENSURE(t, b3.DynamicTree_GetProxyCount(&tree) == PROXY_COUNT) or_return

	// The user data has to have followed each proxy through the churn
	for i: i32 = 0; i < PROXY_COUNT; i += 1 {
		ENSURE(t, alive[i]) or_return
		ENSURE(t, i32(b3.DynamicTree_GetUserData(&tree, proxies[i])) == i) or_return
		ENSURE(t, b3.AABB_Contains(b3.DynamicTree_GetAABB(&tree, proxies[i]), boxes[i])) or_return
	}

	queryBox := b3.AABB{{-10.0, -10.0, -10.0}, {10.0, 10.0, 10.0}}
	treeHits := HitSet{}
	b3.DynamicTree_Query(&tree, queryBox, b3.DEFAULT_MASK_BITS, false, CollectCallback, &treeHits)

	bruteHits := HitSet{}
	for i: i32 = 0; i < PROXY_COUNT; i += 1 {
		if b3.AABB_Overlaps(boxes[i], queryBox) {
			bruteHits.ids[bruteHits.count] = i
			bruteHits.count += 1
		}
	}

	ENSURE(t, SameSet(t, &treeHits, &bruteHits)) or_return
	ENSURE(t, treeHits.count == PROXY_COUNT) or_return

	// Empty the tree
	for i: i32 = 0; i < PROXY_COUNT; i += 1 {
		b3.DynamicTree_DestroyProxy(&tree, proxies[i])
	}

	b3.DynamicTree_Validate(&tree)
	ENSURE(t, b3.DynamicTree_GetProxyCount(&tree) == 0) or_return
	ENSURE(t, b3.DynamicTree_GetHeight(&tree) == 0) or_return

	// Queries on an empty tree must be silent
	emptyHits := HitSet{}
	b3.DynamicTree_Query(&tree, queryBox, b3.DEFAULT_MASK_BITS, false, CollectCallback, &emptyHits)
	ENSURE(t, emptyHits.count == 0) or_return

	b3.DynamicTree_Destroy(&tree)
	return true
}

@(private = "file")
LeafRootHit :: struct {
	count:    i32,
	proxyId:  i32,
	userData: u64,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
LeafRootCallback :: proc "c" (proxyId: i32, userData: u64, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	hit := (^LeafRootHit)(context_)
	hit.count += 1
	hit.proxyId = proxyId
	hit.userData = userData
	return true
}

// A one proxy tree is a leaf at the root with an empty node beside it, which every traversal
// has to survive.
@(private = "file")
TreeLeafRoot :: proc(t: ^testing.T) -> bool {
	tree := b3.DynamicTree_Create(16)

	box := b3.AABB{{-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}}
	proxyId := b3.DynamicTree_CreateProxy(&tree, box, b3.DEFAULT_CATEGORY_BITS, 7)

	b3.DynamicTree_Validate(&tree)
	ENSURE(t, b3.DynamicTree_GetProxyCount(&tree) == 1) or_return
	ENSURE(t, b3.DynamicTree_GetHeight(&tree) == 0) or_return
	ENSURE(t, b3.DynamicTree_GetAreaRatio(&tree) == 0.0) or_return
	ENSURE(t, b3.DynamicTree_GetUserData(&tree, proxyId) == 7) or_return

	hit := LeafRootHit{}
	hitBox := b3.AABB{{0.5, 0.5, 0.5}, {2.0, 2.0, 2.0}}
	b3.DynamicTree_Query(&tree, hitBox, b3.DEFAULT_MASK_BITS, false, LeafRootCallback, &hit)
	ENSURE(t, hit.count == 1) or_return
	ENSURE(t, hit.proxyId == proxyId) or_return
	ENSURE(t, hit.userData == 7) or_return

	hit.count = 0
	missBox := b3.AABB{{4.0, 4.0, 4.0}, {5.0, 5.0, 5.0}}
	b3.DynamicTree_Query(&tree, missBox, b3.DEFAULT_MASK_BITS, false, LeafRootCallback, &hit)
	ENSURE(t, hit.count == 0) or_return

	// The rebuild has one build leaf and the refit has nothing above it
	b3.DynamicTree_Rebuild(&tree, true)
	b3.DynamicTree_Validate(&tree)
	b3.DynamicTree_Refit(&tree)
	b3.DynamicTree_Validate(&tree)
	ENSURE(t, b3.DynamicTree_GetProxyCount(&tree) == 1) or_return

	grown := b3.AABB_Inflate(box, 1.0)
	b3.DynamicTree_EnlargeProxy(&tree, proxyId, grown)
	ENSURE(t, b3.AABB_Contains(b3.DynamicTree_GetAABB(&tree, proxyId), grown)) or_return

	b3.DynamicTree_Destroy(&tree)
	return true
}

// Category bits live on the proxy now, so the filter has to bite at the leaf.
@(private = "file")
TreeCategoryFilter :: proc(t: ^testing.T) -> bool {
	s_seed = 99

	tree := b3.DynamicTree_Create(16)

	boxes: [PROXY_COUNT]b3.AABB
	bits: [PROXY_COUNT]u64
	proxies: [PROXY_COUNT]i32

	for i: i32 = 0; i < PROXY_COUNT; i += 1 {
		boxes[i] = RandomBox(8.0, 0.6)
		bits[i] = u64(1) << u32(i % 3)
		proxies[i] = b3.DynamicTree_CreateProxy(&tree, boxes[i], bits[i], u64(i))
		ENSURE(t, b3.DynamicTree_GetCategoryBits(&tree, proxies[i]) == bits[i]) or_return
	}

	queryBox := b3.AABB{{-8.0, -8.0, -8.0}, {8.0, 8.0, 8.0}}

	// A single bit mask makes requireAll equivalent to the any-bit test, so the last pass is
	// the one that proves requireAll accepts rather than just rejects
	masks := [3]u64{0x1, 0x3, 0x2}
	requireAlls := [3]bool{false, false, true}

	for pass := 0; pass < 3; pass += 1 {
		mask := masks[pass]
		requireAll := requireAlls[pass]

		treeHits := HitSet{}
		b3.DynamicTree_Query(&tree, queryBox, mask, requireAll, CollectCallback, &treeHits)

		bruteHits := HitSet{}
		for i: i32 = 0; i < PROXY_COUNT; i += 1 {
			accept := requireAll ? (bits[i] & mask) == mask : (bits[i] & mask) != 0
			if accept && b3.AABB_Overlaps(boxes[i], queryBox) {
				bruteHits.ids[bruteHits.count] = i
				bruteHits.count += 1
			}
		}

		ENSURE(t, SameSet(t, &treeHits, &bruteHits)) or_return
		ENSURE(t, treeHits.count > PROXY_COUNT / 4) or_return
	}

	// requireAll with two bits can never match a single bit proxy
	noHits := HitSet{}
	b3.DynamicTree_Query(&tree, queryBox, 0x5, true, CollectCallback, &noHits)
	ENSURE(t, noHits.count == 0) or_return

	// Changing the bits has to change the filter without touching the structure
	b3.DynamicTree_SetCategoryBits(&tree, proxies[0], 0x8)
	ENSURE(t, b3.DynamicTree_GetCategoryBits(&tree, proxies[0]) == 0x8) or_return
	b3.DynamicTree_Validate(&tree)

	oneHit := HitSet{}
	b3.DynamicTree_Query(&tree, queryBox, 0x8, false, CollectCallback, &oneHit)
	ENSURE(t, oneHit.count == 1) or_return
	ENSURE(t, oneHit.ids[0] == 0) or_return

	b3.DynamicTree_Destroy(&tree)
	return true
}

// Returning -1 skips the proxy without shrinking the fraction, so the traversal keeps going
// and reports every proxy whose node box the segment crosses.
@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
RayCastCollectCallback :: proc "c" (input: ^b3.RayCastInput, proxyId: i32, userData: u64, context_: rawptr) -> f32 {
	context = runtime.default_context()
	// unified end
	_ = input
	_ = proxyId
	set := (^HitSet)(context_)
	if set.count >= PROXY_COUNT || i32(userData) < 0 || i32(userData) >= PROXY_COUNT {
		set.bad = true
		return 0.0
	}

	set.ids[set.count] = i32(userData)
	set.count += 1
	return -1.0
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
BoxCastCollectCallback :: proc "c" (input: ^b3.BoxCastInput, proxyId: i32, userData: u64, context_: rawptr) -> f32 {
	context = runtime.default_context()
	// unified end
	_ = input
	_ = proxyId
	set := (^HitSet)(context_)
	if set.count >= PROXY_COUNT || i32(userData) < 0 || i32(userData) >= PROXY_COUNT {
		set.bad = true
		return 0.0
	}

	set.ids[set.count] = i32(userData)
	set.count += 1
	return -1.0
}

// The casts pop a pair, test both nodes and order the survivors. A ray that skips every hit
// must still reach every proxy its segment crosses.
@(private = "file")
TreeCastBruteForce :: proc(t: ^testing.T) -> bool {
	s_seed = 31337

	tree := b3.DynamicTree_Create(16)

	boxes: [PROXY_COUNT]b3.AABB
	proxies: [PROXY_COUNT]i32

	for i: i32 = 0; i < PROXY_COUNT; i += 1 {
		boxes[i] = RandomBox(6.0, 1.0)
		proxies[i] = b3.DynamicTree_CreateProxy(&tree, boxes[i], b3.DEFAULT_CATEGORY_BITS, u64(i))
	}

	b3.DynamicTree_Rebuild(&tree, true)
	b3.DynamicTree_Validate(&tree)

	totalCrossings: i32 = 0
	for round: i32 = 0; round < 100; round += 1 {
		origin := b3.Vec3{RandomFloat(-8.0, 8.0), RandomFloat(-8.0, 8.0), RandomFloat(-8.0, 8.0)}
		translation := b3.Vec3{RandomFloat(-16.0, 16.0), RandomFloat(-16.0, 16.0), RandomFloat(-16.0, 16.0)}

		rayInput := b3.RayCastInput {
			origin      = origin,
			translation = translation,
			maxFraction = 1.0,
		}

		treeHits := HitSet{}
		b3.DynamicTree_RayCast(&tree, &rayInput, b3.DEFAULT_MASK_BITS, false, RayCastCollectCallback, &treeHits)
		ENSURE(t, treeHits.bad == false) or_return

		// The tree may report a superset because it tests the node box, not the shape. Every
		// proxy the segment actually crosses has to be in there.
		p2 := b3.MulAdd(origin, 1.0, translation)
		for i: i32 = 0; i < PROXY_COUNT; i += 1 {
			minFraction: f32 = 0.0
			maxFraction: f32 = 1.0
			if b3.RayCastAABB(boxes[i], origin, p2, &minFraction, &maxFraction) == false {
				continue
			}

			found := false
			for j: i32 = 0; j < treeHits.count; j += 1 {
				if treeHits.ids[j] == i {
					found = true
					break
				}
			}

			ENSURE(t, found) or_return
			totalCrossings += 1
		}

		// Every reported proxy has to at least overlap the segment box
		segmentBox := b3.AABB{b3.Min(origin, p2), b3.Max(origin, p2)}
		for j: i32 = 0; j < treeHits.count; j += 1 {
			ENSURE(t, b3.AABB_Overlaps(boxes[treeHits.ids[j]], segmentBox)) or_return
		}

		// A zero extent box cast has to reach the same proxies as the ray
		castBox := b3.AABB{origin, origin}
		boxInput := b3.BoxCastInput {
			box         = castBox,
			translation = translation,
			maxFraction = 1.0,
		}

		boxHits := HitSet{}
		b3.DynamicTree_BoxCast(&tree, &boxInput, b3.DEFAULT_MASK_BITS, false, BoxCastCollectCallback, &boxHits)
		ENSURE(t, boxHits.bad == false) or_return

		for j: i32 = 0; j < treeHits.count; j += 1 {
			found := false
			for k: i32 = 0; k < boxHits.count; k += 1 {
				if boxHits.ids[k] == treeHits.ids[j] {
					found = true
					break
				}
			}

			ENSURE(t, found) or_return
		}
	}

	// Guard against the comparisons above agreeing on nothing
	ENSURE(t, totalCrossings > 100) or_return

	b3.DynamicTree_Destroy(&tree)
	return true
}

@(private = "file")
ClosestResult :: struct {
	boxes: [^]b3.AABB,
	point: b3.Vec3,
	bad:   bool,
}

@(private = "file")
DistanceToBoxSqr :: proc(point: b3.Vec3, box: b3.AABB) -> f32 {
	r := b3.Sub(point, b3.Clamp(point, box.lowerBound, box.upperBound))
	return b3.Dot(r, r)
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
ClosestCollectCallback :: proc "c" (distanceSqrMin: f32, proxyId: i32, userData: u64, context_: rawptr) -> f32 {
	context = runtime.default_context()
	// unified end
	result := (^ClosestResult)(context_)
	if i32(userData) != proxyId {
		result.bad = true
		return distanceSqrMin
	}

	d := DistanceToBoxSqr(result.point, result.boxes[proxyId])
	return d < distanceSqrMin ? d : distanceSqrMin
}

// The closest query orders by distance and prunes on the running minimum, so seeding it from
// the root pair has to be right or it prunes away a whole half of the tree.
@(private = "file")
TreeQueryClosest :: proc(t: ^testing.T) -> bool {
	s_seed = 5150

	tree := b3.DynamicTree_Create(16)

	boxes: [PROXY_COUNT]b3.AABB
	for i: i32 = 0; i < PROXY_COUNT; i += 1 {
		boxes[i] = RandomBox(6.0, 0.9)
		proxyId := b3.DynamicTree_CreateProxy(&tree, boxes[i], b3.DEFAULT_CATEGORY_BITS, u64(i))
		ENSURE(t, proxyId == i) or_return
	}

	for round: i32 = 0; round < 100; round += 1 {
		point := b3.Vec3{RandomFloat(-8.0, 8.0), RandomFloat(-8.0, 8.0), RandomFloat(-8.0, 8.0)}

		bruteBest := b3.FLT_MAX
		for i: i32 = 0; i < PROXY_COUNT; i += 1 {
			d := DistanceToBoxSqr(point, boxes[i])
			if d < bruteBest {
				bruteBest = d
			}
		}

		// The callback reports the exact box distance, so the pruned search has to land on the
		// same minimum as the scan. Same arithmetic both ways, so this is an exact compare.
		result := ClosestResult {
			boxes = &boxes[0],
			point = point,
		}
		minDistanceSqr := b3.FLT_MAX
		b3.DynamicTree_QueryClosest(
			&tree,
			point,
			b3.DEFAULT_MASK_BITS,
			false,
			ClosestCollectCallback,
			&result,
			&minDistanceSqr,
		)

		ENSURE(t, result.bad == false) or_return
		ENSURE(t, minDistanceSqr == bruteBest) or_return
	}

	b3.DynamicTree_Destroy(&tree)
	return true
}

// A saved tree carries only its live node range.
@(private = "file")
TreeSaveLoadRoundtrip :: proc(t: ^testing.T) -> bool {
	s_seed = 4242

	tree := b3.DynamicTree_Create(16)

	boxes: [PROXY_COUNT]b3.AABB
	proxies: [PROXY_COUNT]i32

	for i: i32 = 0; i < PROXY_COUNT; i += 1 {
		boxes[i] = RandomBox(6.0, 1.0)
		proxies[i] = b3.DynamicTree_CreateProxy(&tree, boxes[i], b3.DEFAULT_CATEGORY_BITS, u64(i))
	}

	// Destroy a few so the pair free list is populated and the node tail is stale
	for i: i32 = 0; i < PROXY_COUNT; i += 37 {
		b3.DynamicTree_DestroyProxy(&tree, proxies[i])
	}

	b3.DynamicTree_Rebuild(&tree, true)
	b3.DynamicTree_Validate(&tree)

	treePath: cstring = "test_dynamic_tree_roundtrip.dat"
	b3.DynamicTree_Save(&tree, treePath)

	loaded := b3.DynamicTree_Load(treePath, 1.0)
	_ = os.remove(string(treePath))

	ENSURE(t, loaded.nodes != nil) or_return
	ENSURE(t, loaded.nodeEnd == tree.nodeEnd) or_return
	ENSURE(t, loaded.nodeCapacity == tree.nodeEnd) or_return
	ENSURE(t, b3.DynamicTree_GetProxyCount(&loaded) == b3.DynamicTree_GetProxyCount(&tree)) or_return
	ENSURE(t, mem.compare_ptrs(loaded.nodes, tree.nodes, int(tree.nodeEnd) * size_of(b3.TreeNode)) == 0) or_return
	ENSURE(t, mem.compare_ptrs(loaded.parents, tree.parents, int(tree.nodeEnd) * size_of(i32)) == 0) or_return
	b3.DynamicTree_Validate(&loaded)

	b3.DynamicTree_Destroy(&loaded)
	b3.DynamicTree_Destroy(&tree)
	return true
}

@(test)
DynamicTreeTest :: proc(t: ^testing.T) {
	LOCK()
	if !RUN_SUBTEST(t, TreeLeafRoot) {return}
	if !RUN_SUBTEST(t, TreeQueryBruteForce) {return}
	if !RUN_SUBTEST(t, TreeCategoryFilter) {return}
	if !RUN_SUBTEST(t, TreeMoveRefitRebuild) {return}
	if !RUN_SUBTEST(t, TreeDestroyScrambled) {return}
	if !RUN_SUBTEST(t, TreeCastBruteForce) {return}
	if !RUN_SUBTEST(t, TreeQueryClosest) {return}
	if !RUN_SUBTEST(t, TreeSaveLoadRoundtrip) {return}
}
