// Ported from box3d src/dynamic_tree.c and src/dynamic_tree.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "core:mem"
import "core:os"

TREE_STACK_SIZE :: 512

// Used to mark a node as being moved such that pairs need to be generated and the tree
// may need to be rebuilt. This mark comes from a few sources:
// - updating the transform and AABB in the solver from movement
// - a joint disabling collision
// - creating a proxy
// - setting the transform on a body

MOVED_NODE :: u32(1) << 30
LEAF_NODE :: u32(1) << 31
NODE_INDEX_MASK :: u32(0xFFFFFFFF) & ~(MOVED_NODE | LEAF_NODE)

// Used to indicate empty nodes. It also passes as a leaf so that internal node
// processing will naturally skip it.
EMPTY_NODE :: NODE_INDEX_MASK | LEAF_NODE

ROOT_NODE :: 0

IsLeaf :: #force_inline proc(node: ^TreeNode) -> bool {
	return (node.flagIndex & LEAF_NODE) == LEAF_NODE
}

IsNodeMoved :: #force_inline proc(node: ^TreeNode) -> bool {
	return (node.flagIndex & MOVED_NODE) == MOVED_NODE
}

IsEmptyNode :: #force_inline proc(node: ^TreeNode) -> bool {
	return node.flagIndex == EMPTY_NODE
}

GetLeftChild :: #force_inline proc(node: ^TreeNode) -> i32 {
	return i32(node.flagIndex & NODE_INDEX_MASK)
}

GetProxyId :: #force_inline proc(node: ^TreeNode) -> i32 {
	return i32(node.flagIndex & NODE_INDEX_MASK)
}

GetRootPair :: #force_inline proc(nodes: [^]TreeNode) -> i32 {
	root := &nodes[ROOT_NODE]
	return IsLeaf(root) ? ROOT_NODE : GetLeftChild(root)
}

MakeEmptyNode :: #force_inline proc() -> TreeNode {
	// unified: INFINITY as an exact f32 bit pattern (no core:math)
	inf := f32(0h7F800000)
	// unified end
	node: TreeNode
	node.aabb = AABB {
		lowerBound = {inf, inf, inf},
		upperBound = {-inf, -inf, -inf},
	}
	node.flagIndex = EMPTY_NODE
	node.height = 0
	return node
}

TreeView :: struct {
	nodes:   [^]TreeNode,
	proxies: [^]TreeProxy,
}

MakeTreeView :: proc(tree: ^DynamicTree) -> TreeView {
	view := TreeView {
		nodes   = tree.nodes,
		proxies = tree.proxies,
	}
	return view
}

HasTreeMoved :: proc(tree: ^DynamicTree) -> bool {
	return IsNodeMoved(&tree.nodes[ROOT_NODE])
}

NeedsRebuild :: proc(tree: ^DynamicTree) -> bool {
	return IsNodeMoved(&tree.nodes[ROOT_NODE]) || tree.dfsOrdered == false
}

#assert(ROOT_NODE == 0, "bad root")
#assert(size_of(TreeNode) == 32, "expected size")
#assert(size_of(TreeProxy) == 24, "expected size")

// The free sibling pair list is chained through the parent index of their first node. A pair comes from the
// free list when there is one and from the bump pointer otherwise. A rebuild empties the free list, so
// allocations can just bump.
@(private = "file")
AllocateSiblingPair :: proc(tree: ^DynamicTree) -> i32 {
	// Allocate from free list.
	if tree.pairFreeList != NULL_INDEX {
		pair := tree.pairFreeList
		tree.pairFreeList = tree.parents[pair]
		return pair
	}

	// Extend the bump allocator if needed.
	if tree.nodeEnd + 2 > tree.nodeCapacity {
		oldCapacity := tree.nodeCapacity
		newCapacity := oldCapacity + (oldCapacity >> 1)
		newCapacity += newCapacity & 1
		tree.nodes = GROW_ZERO(tree.nodes, oldCapacity, newCapacity)
		tree.parents = GROW_ZERO(tree.parents, oldCapacity, newCapacity)
		tree.nodeCapacity = newCapacity

		// The spare has to match, the rebuild allocates it again.
		Free(tree.swapNodes, int(oldCapacity) * size_of(TreeNode))
		tree.swapNodes = nil
	}

	pair := tree.nodeEnd
	tree.nodeEnd += 2
	return pair
}

@(private = "file")
FreePair :: proc(tree: ^DynamicTree, pair: i32) {
	assert((pair & 1) == 0 && 2 <= pair && pair < tree.nodeEnd)
	tree.nodes[pair] = MakeEmptyNode()
	tree.nodes[pair + 1] = MakeEmptyNode()
	tree.parents[pair] = tree.pairFreeList
	tree.parents[pair + 1] = NULL_INDEX
	tree.pairFreeList = pair
}

DynamicTree_Create :: proc(proxyCapacity: i32) -> DynamicTree {
	capacity := MaxInt(proxyCapacity, 16)

	// Intentionally _not_ initialized with brace initialization, which can leave
	// uninitialized gaps.
	tree: DynamicTree

	// memset needed for deterministic serialization
	mem.zero(&tree, size_of(DynamicTree))

	tree.version = DYNAMIC_TREE_VERSION

	// A tree of n proxies has 2n - 1 nodes plus the empty node beside the root
	tree.nodeCapacity = 2 * capacity
	tree.nodes = cast([^]TreeNode)AllocZero(int(tree.nodeCapacity) * size_of(TreeNode))
	tree.parents = cast([^]i32)AllocZero(int(tree.nodeCapacity) * size_of(i32))
	tree.pairFreeList = NULL_INDEX

	// The root and the empty node always exist
	tree.nodes[ROOT_NODE] = MakeEmptyNode()
	tree.nodes[ROOT_NODE + 1] = MakeEmptyNode()
	tree.parents[ROOT_NODE] = NULL_INDEX
	tree.parents[ROOT_NODE + 1] = NULL_INDEX

	// This is the bump index.
	tree.nodeEnd = 2
	tree.dfsOrdered = true

	tree.proxyCapacity = capacity
	tree.proxyCount = 0
	tree.proxies = cast([^]TreeProxy)AllocZero(int(tree.proxyCapacity) * size_of(TreeProxy))

	// Build a linked list for the free list.
	for i: i32 = 0; i < tree.proxyCapacity - 1; i += 1 {
		tree.proxies[i].node = NULL_INDEX
		tree.proxies[i].next = i + 1
	}

	tree.proxies[tree.proxyCapacity - 1].node = NULL_INDEX
	tree.proxies[tree.proxyCapacity - 1].next = NULL_INDEX
	tree.proxyFreeList = 0

	tree.swapNodes = nil
	tree.leafIndices = nil
	tree.leafNodes = nil
	tree.leafBoxes = nil
	tree.leafCenters = nil
	tree.binIndices = nil
	tree.rebuildCapacity = 0

	return tree
}

DynamicTree_Destroy :: proc(tree: ^DynamicTree) {
	Free(tree.nodes, int(tree.nodeCapacity) * size_of(TreeNode))
	Free(tree.parents, int(tree.nodeCapacity) * size_of(i32))
	Free(tree.proxies, int(tree.proxyCapacity) * size_of(TreeProxy))
	Free(tree.swapNodes, int(tree.nodeCapacity) * size_of(TreeNode))
	Free(tree.leafIndices, int(tree.rebuildCapacity) * size_of(i32))
	Free(tree.leafNodes, int(tree.rebuildCapacity) * size_of(TreeNode))
	Free(tree.leafBoxes, int(tree.rebuildCapacity) * size_of(AABB))
	Free(tree.leafCenters, int(tree.rebuildCapacity) * size_of(Vec3))
	Free(tree.binIndices, int(tree.rebuildCapacity) * size_of(i32))

	mem.zero(tree, size_of(DynamicTree))
}

// Allocate a proxy from the pool. Grow the pool if necessary.
@(private = "file")
AllocateProxy :: proc(tree: ^DynamicTree) -> i32 {
	// Expand the pool as needed.
	if tree.proxyFreeList == NULL_INDEX {
		assert(tree.proxyCount == tree.proxyCapacity)

		// The free list is empty. Rebuild a bigger pool.
		// A restored pool can have capacity 1, so guarantee strict growth.
		oldCapacity := tree.proxyCapacity
		tree.proxyCapacity += MaxInt(oldCapacity >> 1, 1)
		tree.proxies = GROW_ZERO(tree.proxies, oldCapacity, tree.proxyCapacity)

		// Build a linked list for the free list.
		for i := oldCapacity; i < tree.proxyCapacity - 1; i += 1 {
			tree.proxies[i].node = NULL_INDEX
			tree.proxies[i].next = i + 1
		}

		tree.proxies[tree.proxyCapacity - 1].node = NULL_INDEX
		tree.proxies[tree.proxyCapacity - 1].next = NULL_INDEX
		tree.proxyFreeList = oldCapacity
	}

	// Peel a proxy off the free list.
	proxyIndex := tree.proxyFreeList
	tree.proxyFreeList = tree.proxies[proxyIndex].next
	mem.zero(&tree.proxies[proxyIndex], size_of(TreeProxy))
	tree.proxies[proxyIndex].node = NULL_INDEX
	tree.proxies[proxyIndex].next = NULL_INDEX
	tree.proxyCount += 1
	return proxyIndex
}

// Return a proxy to the pool.
@(private = "file")
FreeProxy :: proc(tree: ^DynamicTree, proxyId: i32) {
	assert(0 <= proxyId && proxyId < tree.proxyCapacity)
	assert(0 < tree.proxyCount)
	tree.proxies[proxyId].node = NULL_INDEX
	tree.proxies[proxyId].next = tree.proxyFreeList
	tree.proxyFreeList = proxyId
	tree.proxyCount -= 1
}

@(private = "file")
GetNodeHeight :: proc(node: ^TreeNode) -> i32 {
	return IsLeaf(node) ? 0 : node.height
}

// The internal node above a children pair
@(private = "file")
MakeInternalNode :: proc(nodes: [^]TreeNode, pair: i32) -> TreeNode {
	c1 := &nodes[pair]
	c2 := &nodes[pair + 1]

	node: TreeNode
	StoreAABBV(&node.aabb, UnionPairV(&nodes[pair]), true)
	node.flagIndex = u32(pair) | ((c1.flagIndex | c2.flagIndex) & MOVED_NODE)
	node.height = 1 + MaxInt(GetNodeHeight(c1), GetNodeHeight(c2))
	return node
}

@(private = "file")
MakeLeafNode :: proc(aabb: AABB, proxyId: i32, userData: u64, moved: bool) -> TreeNode {
	node: TreeNode
	node.aabb = aabb
	node.flagIndex = u32(proxyId) | LEAF_NODE | (moved ? MOVED_NODE : 0)
	node.shapeIndex = i32(u32(userData))
	return node
}

// A node landed at a new index, so tell what hangs below it
@(private = "file")
LinkChildren :: proc(tree: ^DynamicTree, nodeIndex: i32) {
	node := &tree.nodes[nodeIndex]
	if IsLeaf(node) {
		tree.proxies[GetProxyId(node)].node = nodeIndex
	} else {
		pair := GetLeftChild(node)
		tree.parents[pair] = nodeIndex
		tree.parents[pair + 1] = nodeIndex
	}
}

// The sweep refit visits indices from high to low, so it needs every child above its parent
@(private = "file")
IsNodeOrdered :: proc(nodes: [^]TreeNode, nodeIndex: i32) -> bool {
	node := &nodes[nodeIndex]
	return IsLeaf(node) || nodeIndex < GetLeftChild(node)
}

// Greedy algorithm for sibling selection using the SAH
// We have three nodes A-(B,C) and want to add a leaf D, there are three choices.
// 1: make a new parent for A and D : E-(A-(B,C), D)
// 2: associate D with B
//   a: B is a leaf : A-(E-(B,D), C)
//   b: B is an internal node: A-(B{D},C)
// 3: associate D with C
//   a: C is a leaf : A-(B, E-(C,D))
//   b: C is an internal node: A-(B, C{D})
// All of these have a clear cost except when B or C is an internal node. Hence we need to be greedy.

// The cost for cases 1, 2a, and 3a can be computed using the sibling cost formula.
// cost of sibling H = area(union(H, D)) + increased area of ancestors

// Suppose B (or C) is an internal node, then the lowest cost would be one of two cases:
// case1: D becomes a sibling of B
// case2: D becomes a descendant of B along with a new internal node of area(D).
@(private = "file")
FindBestSibling :: proc(tree: ^DynamicTree, boxD: AABB) -> i32 {
	centerD := AABB_Center(boxD)
	areaD := Perimeter(boxD)

	nodes := tree.nodes
	nodeIndex: i32 = ROOT_NODE

	if IsLeaf(&nodes[nodeIndex]) {
		return nodeIndex
	}

	rootBox := nodes[nodeIndex].aabb

	// Area of current node
	areaBase := Perimeter(rootBox)

	// Area of inflated node
	directCost := Perimeter(AABB_Union(rootBox, boxD))
	inheritedCost: f32 = 0.0

	bestSibling := nodeIndex
	bestCost := directCost

	// Descend the tree, following a single greedy path.
	for {
		child1 := GetLeftChild(&nodes[nodeIndex])
		child2 := child1 + 1

		// Cost of creating a new parent for this node and the new leaf
		cost := directCost + inheritedCost

		// Sometimes there are multiple identical costs within tolerance.
		// This breaks the ties using the centroid distance.
		if cost < bestCost {
			bestSibling = nodeIndex
			bestCost = cost
		}

		// Inheritance cost seen by children
		inheritedCost += directCost - areaBase

		leaf1 := IsLeaf(&nodes[child1])
		leaf2 := IsLeaf(&nodes[child2])

		// Cost of descending into child 1
		lowerCost1 := max(f32)
		box1 := nodes[child1].aabb
		directCost1 := Perimeter(AABB_Union(box1, boxD))
		area1: f32 = 0.0
		if leaf1 {
			// Child 1 is a leaf
			// Cost of creating new node and increasing area of node P
			cost1 := directCost1 + inheritedCost
			if cost1 < bestCost {
				bestSibling = child1
				bestCost = cost1
			}
		} else {
			// Child 1 is an internal node
			area1 = Perimeter(box1)

			// Lower bound cost of inserting under child 1. The minimum accounts for two possibilities:
			// 1. Child1 could be the sibling with cost1 = inheritedCost + directCost1
			// 2. A descendant of child1 could be the sibling with the lower bound cost of
			//       cost1 = inheritedCost + (directCost1 - area1) + areaD
			// This minimum here leads to the minimum of these two costs.
			lowerCost1 = inheritedCost + directCost1 + MinFloat(areaD - area1, 0.0)
		}

		// Cost of descending into child 2
		lowerCost2 := max(f32)
		box2 := nodes[child2].aabb
		directCost2 := Perimeter(AABB_Union(box2, boxD))
		area2: f32 = 0.0
		if leaf2 {
			cost2 := directCost2 + inheritedCost
			if cost2 < bestCost {
				bestSibling = child2
				bestCost = cost2
			}
		} else {
			area2 = Perimeter(box2)
			lowerCost2 = inheritedCost + directCost2 + MinFloat(areaD - area2, 0.0)
		}

		if leaf1 && leaf2 {
			break
		}

		// Can the cost possibly be decreased?
		if bestCost <= lowerCost1 && bestCost <= lowerCost2 {
			break
		}

		if lowerCost1 == lowerCost2 && leaf1 == false {
			assert(lowerCost1 < max(f32))
			assert(lowerCost2 < max(f32))

			// No clear choice based on lower bound surface area. This can happen when both
			// children fully contain D. Fall back to node distance.
			d1 := Sub(AABB_Center(box1), centerD)
			d2 := Sub(AABB_Center(box2), centerD)
			lowerCost1 = LengthSquared(d1)
			lowerCost2 = LengthSquared(d2)
		}

		// Descend
		if lowerCost1 < lowerCost2 && leaf1 == false {
			nodeIndex = child1
			areaBase = area1
			directCost = directCost1
		} else {
			nodeIndex = child2
			areaBase = area2
			directCost = directCost2
		}
	}

	return bestSibling
}

// Swap a child of A with a grandchild under the other child.
// Example swapping B with G:
// (A (B (D E) C (F G)) -> (A (G) C (F B (D E))
@(private = "file")
SwapNodes :: proc(tree: ^DynamicTree, iDown, iUp: i32) {
	nodes := tree.nodes
	nodes[iDown], nodes[iUp] = nodes[iUp], nodes[iDown]
	LinkChildren(tree, iDown)
	LinkChildren(tree, iUp)

	if IsNodeOrdered(nodes, iDown) == false || IsNodeOrdered(nodes, iUp) == false {
		tree.dfsOrdered = false
	}

	// The sibling of the node that went down holds a different subtree now
	iC := iDown ~ 1
	nodes[iC] = MakeInternalNode(nodes, GetLeftChild(&nodes[iC]))
}

// Perform a left or right rotation if node A is imbalanced.
// Tree: (A (B (D E) C (F G))
@(private = "file")
RotateNodes :: proc(tree: ^DynamicTree, iA: i32) {
	nodes := tree.nodes
	A := &nodes[iA]
	assert(IsLeaf(A) == false)

	iB := GetLeftChild(A)
	iC := iB + 1
	B := &nodes[iB]
	C := &nodes[iC]

	leafB := IsLeaf(B)
	leafC := IsLeaf(C)
	if leafB && leafC {
		return
	}

	bestDown: i32 = NULL_INDEX
	bestUp: i32 = NULL_INDEX
	bestDelta: f32 = 0.0

	if leafC == false {
		// Swap B with F or G
		iF := GetLeftChild(C)
		iG := iF + 1
		areaC := Perimeter(C.aabb)

		// B <-> F then C (B G)
		deltaBF := Perimeter(UnionV(B.aabb, nodes[iG].aabb)) - areaC
		if deltaBF < bestDelta {
			bestDown = iB
			bestUp = iF
			bestDelta = deltaBF
		}

		// B <-> G then C (F B)
		deltaBG := Perimeter(UnionV(B.aabb, nodes[iF].aabb)) - areaC
		if deltaBG < bestDelta {
			bestDown = iB
			bestUp = iG
			bestDelta = deltaBG
		}
	}

	if leafB == false {
		// Swap C with D or E
		iD := GetLeftChild(B)
		iE := iD + 1
		areaB := Perimeter(B.aabb)

		// C <-> D then B (C E)
		deltaCD := Perimeter(UnionV(C.aabb, nodes[iE].aabb)) - areaB
		if deltaCD < bestDelta {
			bestDown = iC
			bestUp = iD
			bestDelta = deltaCD
		}

		// C <-> E then B (D C)
		deltaCE := Perimeter(UnionV(C.aabb, nodes[iD].aabb)) - areaB
		if deltaCE < bestDelta {
			bestDown = iC
			bestUp = iE
			bestDelta = deltaCE
		}
	}

	if bestDown != NULL_INDEX {
		SwapNodes(tree, bestDown, bestUp)
	}
}

@(private = "file")
InsertLeaf :: proc(tree: ^DynamicTree, aabb: AABB, proxyId: i32, moved: bool, shouldRotate: bool) {
	proxy := &tree.proxies[proxyId]
	leaf := MakeLeafNode(aabb, proxyId, proxy.userData, moved)

	if IsEmptyNode(&tree.nodes[ROOT_NODE]) {
		tree.nodes[ROOT_NODE] = leaf
		proxy.node = ROOT_NODE
		return
	}

	sibling := FindBestSibling(tree, aabb)

	// The sibling's position becomes the new parent. The sibling moves down into a new pair
	// beside the leaf, which puts it below its own children when it is internal.
	pair := AllocateSiblingPair(tree)
	nodes := tree.nodes
	parents := tree.parents

	nodes[pair] = nodes[sibling]
	nodes[pair + 1] = leaf
	parents[pair] = sibling
	parents[pair + 1] = sibling
	LinkChildren(tree, pair)
	proxy.node = pair + 1

	nodes[sibling] = MakeInternalNode(nodes, pair)

	if IsNodeOrdered(nodes, sibling) == false || IsNodeOrdered(nodes, pair) == false {
		tree.dfsOrdered = false
	}

	// Walk back up the tree refitting ancestors, the root included.
	index := sibling
	for index != NULL_INDEX {
		if shouldRotate {
			RotateNodes(tree, index)
		}

		nodes[index] = MakeInternalNode(nodes, GetLeftChild(&nodes[index]))
		index = parents[index]
	}
}

@(private = "file")
RemoveLeaf :: proc(tree: ^DynamicTree, proxyId: i32) {
	nodes := tree.nodes
	parents := tree.parents

	leaf := tree.proxies[proxyId].node
	assert(0 <= leaf && leaf < tree.nodeEnd)
	assert(IsLeaf(&nodes[leaf]) && GetProxyId(&nodes[leaf]) == proxyId)

	if leaf == ROOT_NODE {
		nodes[ROOT_NODE] = MakeEmptyNode()
		return
	}

	// The sibling takes the parent's position and the pair is freed. This keeps the order,
	// the sibling only moves down.
	parent := parents[leaf]
	nodes[parent] = nodes[leaf ~ 1]
	LinkChildren(tree, parent)
	FreePair(tree, leaf & ~i32(1))

	// Update ancestors.
	index := parents[parent]
	for index != NULL_INDEX {
		nodes[index] = MakeInternalNode(nodes, GetLeftChild(&nodes[index]))
		index = parents[index]
	}
}

// Create a proxy in the tree as a leaf node. We return the index of the node instead of a pointer so that we can grow
// the node pool.
CreateTreeProxyInternal :: proc(
	tree: ^DynamicTree,
	aabb: AABB,
	categoryBits: u64,
	userData: u64,
	markMoved: bool,
) -> i32 {
	VALIDATE(IsValidAABB(aabb))

	proxyId := AllocateProxy(tree)

	proxy := &tree.proxies[proxyId]
	proxy.categoryBits = categoryBits
	proxy.userData = userData

	shouldRotate := true
	InsertLeaf(tree, aabb, proxyId, markMoved, shouldRotate)

	return proxyId
}

DynamicTree_CreateProxy :: proc(tree: ^DynamicTree, aabb: AABB, categoryBits: u64, userData: u64) -> i32 {
	return CreateTreeProxyInternal(tree, aabb, categoryBits, userData, false)
}

DynamicTree_DestroyProxy :: proc(tree: ^DynamicTree, proxyId: i32) {
	assert(0 <= proxyId && proxyId < tree.proxyCapacity)

	RemoveLeaf(tree, proxyId)
	FreeProxy(tree, proxyId)
}

DynamicTree_GetProxyCount :: proc(tree: ^DynamicTree) -> i32 {
	return tree.proxyCount
}

DynamicTree_MoveProxyInternal :: proc(tree: ^DynamicTree, proxyId: i32, aabb: AABB, markMoved: bool) {
	VALIDATE(IsValidAABB(aabb))
	VALIDATE(aabb.upperBound.x - aabb.lowerBound.x < HUGE)
	VALIDATE(aabb.upperBound.y - aabb.lowerBound.y < HUGE)
	VALIDATE(aabb.upperBound.z - aabb.lowerBound.z < HUGE)
	assert(0 <= proxyId && proxyId < tree.proxyCapacity)

	RemoveLeaf(tree, proxyId)

	shouldRotate := false
	InsertLeaf(tree, aabb, proxyId, markMoved, shouldRotate)
}

DynamicTree_MoveProxy :: proc(tree: ^DynamicTree, proxyId: i32, aabb: AABB) {
	DynamicTree_MoveProxyInternal(tree, proxyId, aabb, false)
}

DynamicTree_EnlargeProxy :: proc(tree: ^DynamicTree, proxyId: i32, aabb: AABB) {
	VALIDATE(IsValidAABB(aabb))
	VALIDATE(aabb.upperBound.x - aabb.lowerBound.x < HUGE)
	VALIDATE(aabb.upperBound.y - aabb.lowerBound.y < HUGE)
	VALIDATE(aabb.upperBound.z - aabb.lowerBound.z < HUGE)
	assert(0 <= proxyId && proxyId < tree.proxyCapacity)

	nodes := tree.nodes
	parents := tree.parents

	index := tree.proxies[proxyId].node
	node := &nodes[index]
	assert(IsLeaf(node))

	// Caller must ensure this
	VALIDATE(AABB_Contains(node.aabb, aabb) == false)

	node.aabb = aabb
	node.flagIndex |= MOVED_NODE

	index = parents[index]
	for index != NULL_INDEX {
		node = &nodes[index]
		changed := EnlargeAABB(&node.aabb, aabb)

		// This is marked to ensure the root is marked in this loop or the one below.
		node.flagIndex |= MOVED_NODE

		index = parents[index]

		if changed == false {
			break
		}
	}

	// Mark all the way up to the root.
	for index != NULL_INDEX {
		node = &nodes[index]
		if node.flagIndex & MOVED_NODE != 0 {
			// Early out because this ancestor was previously ascended and marked as moved.
			break
		}

		node.flagIndex |= MOVED_NODE
		index = parents[index]
	}
}

DynamicTree_SetCategoryBits :: proc(tree: ^DynamicTree, proxyId: i32, categoryBits: u64) {
	assert(0 <= proxyId && proxyId < tree.proxyCapacity)
	tree.proxies[proxyId].categoryBits = categoryBits
}

DynamicTree_GetCategoryBits :: proc(tree: ^DynamicTree, proxyId: i32) -> u64 {
	assert(0 <= proxyId && proxyId < tree.proxyCapacity)
	return tree.proxies[proxyId].categoryBits
}

DynamicTree_GetHeight :: proc(tree: ^DynamicTree) -> i32 {
	return GetNodeHeight(&tree.nodes[ROOT_NODE])
}

// The area ratio is the thing that SAH seeks to minimize. SAH
// cannot do anything about leaf boxes or the root box. It seeks
// to minimize the area of all non-root internal nodes. Divide this
// by the root area to make the metric non-dimensional.
// So this becomes a meaningful measure of tree quality.
DynamicTree_GetAreaRatio :: proc(tree: ^DynamicTree) -> f32 {
	if tree.proxyCount == 0 {
		return 0.0
	}

	nodes := tree.nodes
	rootArea := Perimeter(nodes[ROOT_NODE].aabb)
	if rootArea <= 0.0 {
		return 0.0
	}

	// Free nodes and the empty node are leaf tagged.
	internalArea: f32 = 0.0
	nodeEnd := tree.nodeEnd
	for i: i32 = 2; i < nodeEnd; i += 1 {
		if IsLeaf(&nodes[i]) == false {
			internalArea += Perimeter(nodes[i].aabb)
		}
	}

	return internalArea / rootArea
}

DynamicTree_GetRootBounds :: proc(tree: ^DynamicTree) -> AABB {
	if tree.proxyCount == 0 {
		return AABB{Vec3_zero, Vec3_zero}
	}

	return tree.nodes[ROOT_NODE].aabb
}

when ENABLE_VALIDATION {

	@(private = "file")
	ValidateSubtree :: proc(tree: ^DynamicTree, nodeIndex: i32, leafCount: ^i32) -> i32 {
		assert(0 <= nodeIndex && nodeIndex < tree.nodeEnd)
		node := &tree.nodes[nodeIndex]
		assert(IsEmptyNode(node) == false)
		assert(IsValidAABB(node.aabb))

		if IsLeaf(node) {
			proxyId := GetProxyId(node)
			assert(0 <= proxyId && proxyId < tree.proxyCapacity)
			assert(tree.proxies[proxyId].node == nodeIndex)
			assert(i32(u32(tree.proxies[proxyId].userData)) == node.shapeIndex)
			leafCount^ += 1
			return 0
		}

		pair := GetLeftChild(node)
		assert((pair & 1) == 0 && 2 <= pair && pair < tree.nodeEnd)
		assert(tree.parents[pair] == nodeIndex)
		assert(tree.parents[pair + 1] == nodeIndex)
		assert(tree.dfsOrdered == false || nodeIndex < pair)

		c1 := &tree.nodes[pair]
		c2 := &tree.nodes[pair + 1]
		assert(AABB_Contains(node.aabb, c1.aabb))
		assert(AABB_Contains(node.aabb, c2.aabb))
		assert(IsNodeMoved(node) == (IsNodeMoved(c1) || IsNodeMoved(c2)))

		// A bad tree can stack overflow, but that is validation on its own.
		height1 := ValidateSubtree(tree, pair, leafCount)
		height2 := ValidateSubtree(tree, pair + 1, leafCount)
		height := 1 + MaxInt(height1, height2)
		assert(node.height == height)
		return height
	}

}

DynamicTree_Validate :: proc(tree: ^DynamicTree) {
	when ENABLE_VALIDATION {
		assert(2 <= tree.nodeEnd && tree.nodeEnd <= tree.nodeCapacity)
		assert((tree.nodeEnd & 1) == 0)
		assert(tree.parents[ROOT_NODE] == NULL_INDEX)
		assert(IsEmptyNode(&tree.nodes[ROOT_NODE + 1]))

		freePairCount: i32 = 0
		pair := tree.pairFreeList
		for pair != NULL_INDEX {
			assert((pair & 1) == 0 && 2 <= pair && pair < tree.nodeEnd)
			assert(IsEmptyNode(&tree.nodes[pair]))
			assert(IsEmptyNode(&tree.nodes[pair + 1]))
			pair = tree.parents[pair]
			freePairCount += 1
			assert(2 * freePairCount < tree.nodeEnd)
		}

		// Validate proxy free list.
		freeProxyCount: i32 = 0
		freeIndex := tree.proxyFreeList
		for freeIndex != NULL_INDEX {
			assert(0 <= freeIndex && freeIndex < tree.proxyCapacity)
			assert(tree.proxies[freeIndex].node == NULL_INDEX)
			freeIndex = tree.proxies[freeIndex].next
			freeProxyCount += 1
		}
		assert(tree.proxyCount + freeProxyCount == tree.proxyCapacity)

		assert(tree.nodeEnd == 2 * MaxInt(tree.proxyCount, 1) + 2 * freePairCount)

		if tree.proxyCount == 0 {
			assert(IsEmptyNode(&tree.nodes[ROOT_NODE]))
			return
		}

		leafCount: i32 = 0
		ValidateSubtree(tree, ROOT_NODE, &leafCount)
		assert(leafCount == tree.proxyCount)

	} else {
		_ = tree
	}
}

DynamicTree_ValidateNoMoved :: proc(tree: ^DynamicTree) {
	when ENABLE_VALIDATION {
		nodes := tree.nodes
		nodeEnd := tree.nodeEnd
		for i: i32 = 0; i < nodeEnd; i += 1 {
			assert(IsNodeMoved(&nodes[i]) == false)
		}
	} else {
		_ = tree
	}
}

DynamicTree_GetByteCount :: proc(tree: ^DynamicTree) -> i32 {
	size := size_of(DynamicTree)
	size += int(tree.nodeCapacity) * size_of(TreeNode)
	size += int(tree.nodeCapacity) * size_of(i32)
	size += int(tree.proxyCapacity) * size_of(TreeProxy)
	size += tree.swapNodes == nil ? 0 : int(tree.nodeCapacity) * size_of(TreeNode)

	// leafIndices
	size += int(tree.rebuildCapacity) * size_of(i32)
	// leafNodes
	size += int(tree.rebuildCapacity) * size_of(TreeNode)
	// leafBoxes
	size += int(tree.rebuildCapacity) * size_of(AABB)
	// leafCenters
	size += int(tree.rebuildCapacity) * size_of(Vec3)
	// binIndices
	size += int(tree.rebuildCapacity) * size_of(i32)

	return i32(size)
}

@(private = "file")
TestCategory :: #force_inline proc(categoryBits: u64, maskBits: u64, requireAllBits: bool) -> bool {
	return requireAllBits ? (categoryBits & maskBits) == maskBits : (categoryBits & maskBits) != 0
}

@(private = "file")
QueryTreeView :: #force_inline proc(
	view: ^TreeView,
	aabb: AABB,
	maskBits: u64,
	requireAllBits: bool,
	callback: TreeQueryCallbackFcn,
	context_: rawptr,
	hasProxies: bool,
) -> TreeStats {
	result := TreeStats{}

	if IsEmptyNode(&view.nodes[ROOT_NODE]) {
		return result
	}

	nodes := view.nodes

	stack: [TREE_STACK_SIZE]i32 = ---
	stackCount: i32 = 0
	stack[stackCount] = GetRootPair(nodes)
	stackCount += 1

	aabb := aabb
	boxv := LoadAABBV(&aabb)

	for stackCount > 0 {
		stackCount -= 1
		pair := stack[stackCount]
		result.nodeVisits += 1

		for i: i32 = 0; i < 2; i += 1 {
			node := &nodes[pair + i]
			if OverlapNode(boxv, node) == false {
				continue
			}

			if IsLeaf(node) {
				proxyId := GetProxyId(node)
				userData := u64(u32(node.shapeIndex))
				if hasProxies {
					proxy := &view.proxies[proxyId]
					if TestCategory(proxy.categoryBits, maskBits, requireAllBits) == false {
						continue
					}
					userData = proxy.userData
				}

				proceed := callback(proxyId, userData, context_)
				result.leafVisits += 1

				if proceed == false {
					return result
				}
			} else {
				if stackCount < TREE_STACK_SIZE - 1 {
					stack[stackCount] = GetLeftChild(node)
					stackCount += 1
				} else {
					assert(stackCount < TREE_STACK_SIZE - 1)
				}
			}
		}
	}

	return result
}

@(private = "file")
DistanceToNodeSqr :: #force_inline proc(point: Vec3, node: ^TreeNode) -> f32 {
	r := Sub(point, Clamp(point, node.aabb.lowerBound, node.aabb.upperBound))
	return Dot(r, r)
}

@(private = "file")
QueryClosestItem :: struct {
	nodeIndex:         i32,
	distanceToNodeSqr: f32,
}

@(private = "file")
QueryClosestTreeView :: #force_inline proc(
	view: ^TreeView,
	point: Vec3,
	maskBits: u64,
	requireAllBits: bool,
	callback: TreeQueryClosestCallbackFcn,
	context_: rawptr,
	minDistanceSqr: ^f32,
	hasProxies: bool,
) -> TreeStats {
	result := TreeStats{}

	if IsEmptyNode(&view.nodes[ROOT_NODE]) {
		return result
	}

	nodes := view.nodes

	minSqr := minDistanceSqr^
	stack: [TREE_STACK_SIZE]QueryClosestItem = ---
	stackCount: i32 = 0

	rootPair := GetRootPair(nodes)
	seed1 := QueryClosestItem {
		nodeIndex         = rootPair,
		distanceToNodeSqr = DistanceToNodeSqr(point, &nodes[rootPair]),
	}

	if rootPair == ROOT_NODE {
		stack[stackCount] = seed1
		stackCount += 1
	} else {
		seed2 := QueryClosestItem {
			nodeIndex         = rootPair + 1,
			distanceToNodeSqr = DistanceToNodeSqr(point, &nodes[rootPair + 1]),
		}

		// Ensure we iterate the closest child first as we pop off the stack
		if seed2.distanceToNodeSqr < seed1.distanceToNodeSqr {
			stack[stackCount] = seed1
			stackCount += 1
			stack[stackCount] = seed2
			stackCount += 1
		} else {
			stack[stackCount] = seed2
			stackCount += 1
			stack[stackCount] = seed1
			stackCount += 1
		}
	}

	for stackCount > 0 {
		stackCount -= 1
		item := stack[stackCount]
		node := &nodes[item.nodeIndex]
		result.nodeVisits += 1

		if item.distanceToNodeSqr >= minSqr {
			continue
		}

		if IsLeaf(node) {
			proxyId := GetProxyId(node)
			userData := u64(u32(node.shapeIndex))
			if hasProxies {
				proxy := &view.proxies[proxyId]
				if TestCategory(proxy.categoryBits, maskBits, requireAllBits) == false {
					continue
				}
				userData = proxy.userData
			}

			// callback to user code with minimum distance squared so far and proxy id
			dd := callback(minSqr, proxyId, userData, context_)

			if dd < minSqr {
				minSqr = dd
			}

			result.leafVisits += 1
			continue
		}

		if stackCount + 2 > TREE_STACK_SIZE {
			assert(stackCount + 2 <= TREE_STACK_SIZE)
			continue
		}

		pair := GetLeftChild(node)

		// Store the distance to node in the stack instead of recomputing after pop
		item1 := QueryClosestItem {
			nodeIndex         = pair,
			distanceToNodeSqr = DistanceToNodeSqr(point, &nodes[pair]),
		}

		item2 := QueryClosestItem {
			nodeIndex         = pair + 1,
			distanceToNodeSqr = DistanceToNodeSqr(point, &nodes[pair + 1]),
		}

		// Ensure we iterate the closest child first as we pop off the stack
		if item2.distanceToNodeSqr < item1.distanceToNodeSqr {
			stack[stackCount] = item1
			stackCount += 1
			stack[stackCount] = item2
			stackCount += 1
		} else {
			stack[stackCount] = item2
			stackCount += 1
			stack[stackCount] = item1
			stackCount += 1
		}
	}

	minDistanceSqr^ = minSqr

	return result
}

// A lot of optimization work went into this. The children of a popped pair are both tested
// before either is followed, then the survivors are ordered so the closer one narrows first.
@(private = "file")
RayCastTreeView :: #force_inline proc(
	view: ^TreeView,
	input: ^RayCastInput,
	maskBits: u64,
	requireAllBits: bool,
	callback: TreeRayCastCallbackFcn,
	context_: rawptr,
	hasProxies: bool,
) -> TreeStats {
	result := TreeStats{}

	if IsEmptyNode(&view.nodes[ROOT_NODE]) {
		return result
	}

	p1 := input.origin
	d := input.translation

	pv1 := LoadV(&p1.x)
	dv := LoadV(&d.x)

	maxFraction := input.maxFraction

	p2 := MulAdd(p1, maxFraction, d)

	// Build a bounding box for the segment.
	segmentAABB := AABB{Min(p1, p2), Max(p1, p2)}
	boxv := LoadAABBV(&segmentAABB)

	nodes := view.nodes

	stack: [TREE_STACK_SIZE]i32 = ---
	stackCount: i32 = 0
	stack[stackCount] = GetRootPair(nodes)
	stackCount += 1

	subInput := input^

	for stackCount > 0 {
		stackCount -= 1
		pair := stack[stackCount]
		result.nodeVisits += 1

		hit: [2]^TreeNode = ---
		isLeaf: [2]bool = ---
		hitCount: i32 = 0
		for i: i32 = 0; i < 2; i += 1 {
			node := &nodes[pair + i]

			if OverlapNode(boxv, node) == false {
				continue
			}

			nodeAABB := node.aabb
			lower := LoadV(&nodeAABB.lowerBound.x)
			upper := LoadV(&nodeAABB.upperBound.x)
			if TestBoundsRayOverlap(lower, upper, pv1, dv) == false {
				continue
			}

			isLeaf[hitCount] = IsLeaf(node)
			hit[hitCount] = node
			hitCount += 1
		}

		if hitCount == 2 && isLeaf[0] == false && isLeaf[1] == false {
			center1 := AABB_Center(hit[0].aabb)
			center2 := AABB_Center(hit[1].aabb)
			d1 := DistanceSquared(center1, p1)
			d2 := DistanceSquared(center2, p1)

			// Want to push the closest one last. Both have the same isLeaf, so they don't swap.
			if d1 < d2 {
				hit[0], hit[1] = hit[1], hit[0]
			}
		}

		for i: i32 = 0; i < hitCount; i += 1 {
			if isLeaf[i] {
				proxyId := GetProxyId(hit[i])
				userData := u64(u32(hit[i].shapeIndex))
				if hasProxies {
					proxy := &view.proxies[proxyId]
					if TestCategory(proxy.categoryBits, maskBits, requireAllBits) == false {
						continue
					}
					userData = proxy.userData
				}

				subInput.maxFraction = maxFraction

				value := callback(&subInput, proxyId, userData, context_)
				result.leafVisits += 1

				// The user may return -1 to indicate this shape should be skipped

				if value == 0.0 {
					// The client has terminated the ray cast.
					return result
				}

				if 0.0 < value && value <= maxFraction {
					// Update segment bounding box.
					maxFraction = value
					p2 = MulAdd(p1, maxFraction, d)
					segmentAABB.lowerBound = Min(p1, p2)
					segmentAABB.upperBound = Max(p1, p2)
					boxv = LoadAABBV(&segmentAABB)
				}
			} else {
				if stackCount < TREE_STACK_SIZE - 1 {
					stack[stackCount] = GetLeftChild(hit[i])
					stackCount += 1
				} else {
					assert(stackCount < TREE_STACK_SIZE - 1)
				}
			}
		}
	}

	return result
}

// Follows structure of ray cast with small tweaks to handle a swept box.
@(private = "file")
BoxCastTreeView :: #force_inline proc(
	view: ^TreeView,
	input: ^BoxCastInput,
	maskBits: u64,
	requireAllBits: bool,
	callback: TreeBoxCastCallbackFcn,
	context_: rawptr,
	hasProxies: bool,
) -> TreeStats {
	result := TreeStats{}

	if IsEmptyNode(&view.nodes[ROOT_NODE]) {
		return result
	}

	// The caller folds the shape radius and the world origin into the box
	originAABB := input.box

	p1 := AABB_Center(originAABB)
	extension := AABB_Extents(originAABB)

	d := input.translation

	pv1 := LoadV(&p1.x)
	dv := LoadV(&d.x)
	ev := LoadV(&extension.x)

	maxFraction := input.maxFraction

	// Build total box for the cast
	t := MulSV(maxFraction, input.translation)
	totalAABB := AABB {
		Min(originAABB.lowerBound, Add(originAABB.lowerBound, t)),
		Max(originAABB.upperBound, Add(originAABB.upperBound, t)),
	}
	boxv := LoadAABBV(&totalAABB)

	nodes := view.nodes

	stack: [TREE_STACK_SIZE]i32 = ---
	stackCount: i32 = 0
	stack[stackCount] = GetRootPair(nodes)
	stackCount += 1

	subInput := input^

	for stackCount > 0 {
		stackCount -= 1
		pair := stack[stackCount]
		result.nodeVisits += 1

		hit: [2]^TreeNode = ---
		isLeaf: [2]bool = ---
		hitCount: i32 = 0
		for i: i32 = 0; i < 2; i += 1 {
			node := &nodes[pair + i]

			if OverlapNode(boxv, node) == false {
				continue
			}

			// radius extension is added to the node in this case
			nodeAABB := node.aabb
			lower := SubV(LoadV(&nodeAABB.lowerBound.x), ev)
			upper := AddV(LoadV(&nodeAABB.upperBound.x), ev)
			if TestBoundsRayOverlap(lower, upper, pv1, dv) == false {
				continue
			}

			isLeaf[hitCount] = IsLeaf(node)
			hit[hitCount] = node
			hitCount += 1
		}

		if hitCount == 2 && isLeaf[0] == false && isLeaf[1] == false {
			center1 := AABB_Center(hit[0].aabb)
			center2 := AABB_Center(hit[1].aabb)
			d1 := DistanceSquared(center1, p1)
			d2 := DistanceSquared(center2, p1)

			// Want to push the closest one last. Both have the same isLeaf, so they don't swap.
			if d1 < d2 {
				hit[0], hit[1] = hit[1], hit[0]
			}
		}

		for i: i32 = 0; i < hitCount; i += 1 {
			if isLeaf[i] {
				proxyId := GetProxyId(hit[i])
				userData := u64(u32(hit[i].shapeIndex))
				if hasProxies {
					proxy := &view.proxies[proxyId]
					if TestCategory(proxy.categoryBits, maskBits, requireAllBits) == false {
						continue
					}
					userData = proxy.userData
				}

				subInput.maxFraction = maxFraction

				value := callback(&subInput, proxyId, userData, context_)
				result.leafVisits += 1

				if value == 0.0 {
					// The client has terminated the cast.
					return result
				}

				if 0.0 < value && value < maxFraction {
					maxFraction = value
					t = MulSV(maxFraction, input.translation)
					totalAABB.lowerBound = Min(originAABB.lowerBound, Add(originAABB.lowerBound, t))
					totalAABB.upperBound = Max(originAABB.upperBound, Add(originAABB.upperBound, t))
					boxv = LoadAABBV(&totalAABB)
				}
			} else {
				if stackCount < TREE_STACK_SIZE - 1 {
					stack[stackCount] = GetLeftChild(hit[i])
					stackCount += 1
				} else {
					assert(stackCount < TREE_STACK_SIZE - 1)
				}
			}
		}
	}

	return result
}

DynamicTree_Query :: proc(
	tree: ^DynamicTree,
	aabb: AABB,
	maskBits: u64,
	requireAllBits: bool,
	callback: TreeQueryCallbackFcn,
	context_: rawptr,
) -> TreeStats {
	if tree.proxyCount == 0 {
		return TreeStats{}
	}

	view := MakeTreeView(tree)
	return QueryTreeView(&view, aabb, maskBits, requireAllBits, callback, context_, true)
}

DynamicTree_QueryClosest :: proc(
	tree: ^DynamicTree,
	point: Vec3,
	maskBits: u64,
	requireAllBits: bool,
	callback: TreeQueryClosestCallbackFcn,
	context_: rawptr,
	minDistanceSqr: ^f32,
) -> TreeStats {
	if tree.proxyCount == 0 {
		return TreeStats{}
	}

	view := MakeTreeView(tree)
	return QueryClosestTreeView(&view, point, maskBits, requireAllBits, callback, context_, minDistanceSqr, true)
}

DynamicTree_RayCast :: proc(
	tree: ^DynamicTree,
	input: ^RayCastInput,
	maskBits: u64,
	requireAllBits: bool,
	callback: TreeRayCastCallbackFcn,
	context_: rawptr,
) -> TreeStats {
	if tree.proxyCount == 0 {
		return TreeStats{}
	}

	view := MakeTreeView(tree)
	return RayCastTreeView(&view, input, maskBits, requireAllBits, callback, context_, true)
}

DynamicTree_BoxCast :: proc(
	tree: ^DynamicTree,
	input: ^BoxCastInput,
	maskBits: u64,
	requireAllBits: bool,
	callback: TreeBoxCastCallbackFcn,
	context_: rawptr,
) -> TreeStats {
	if tree.proxyCount == 0 {
		return TreeStats{}
	}

	view := MakeTreeView(tree)
	return BoxCastTreeView(&view, input, maskBits, requireAllBits, callback, context_, true)
}

TreeView_Query :: proc(
	view: ^TreeView,
	aabb: AABB,
	maskBits: u64,
	requireAllBits: bool,
	callback: TreeQueryCallbackFcn,
	context_: rawptr,
) -> TreeStats {
	if view.proxies == nil {
		return QueryTreeView(view, aabb, maskBits, requireAllBits, callback, context_, false)
	}

	return QueryTreeView(view, aabb, maskBits, requireAllBits, callback, context_, true)
}

TreeView_QueryClosest :: proc(
	view: ^TreeView,
	point: Vec3,
	maskBits: u64,
	requireAllBits: bool,
	callback: TreeQueryClosestCallbackFcn,
	context_: rawptr,
	minDistanceSqr: ^f32,
) -> TreeStats {
	if view.proxies == nil {
		return QueryClosestTreeView(view, point, maskBits, requireAllBits, callback, context_, minDistanceSqr, false)
	}

	return QueryClosestTreeView(view, point, maskBits, requireAllBits, callback, context_, minDistanceSqr, true)
}

TreeView_RayCast :: proc(
	view: ^TreeView,
	input: ^RayCastInput,
	maskBits: u64,
	requireAllBits: bool,
	callback: TreeRayCastCallbackFcn,
	context_: rawptr,
) -> TreeStats {
	if view.proxies == nil {
		return RayCastTreeView(view, input, maskBits, requireAllBits, callback, context_, false)
	}

	return RayCastTreeView(view, input, maskBits, requireAllBits, callback, context_, true)
}

TreeView_BoxCast :: proc(
	view: ^TreeView,
	input: ^BoxCastInput,
	maskBits: u64,
	requireAllBits: bool,
	callback: TreeBoxCastCallbackFcn,
	context_: rawptr,
) -> TreeStats {
	if view.proxies == nil {
		return BoxCastTreeView(view, input, maskBits, requireAllBits, callback, context_, false)
	}

	return BoxCastTreeView(view, input, maskBits, requireAllBits, callback, context_, true)
}

// Median split == 0, Surface area heuristic == 1
TREE_HEURISTIC :: 0

when TREE_HEURISTIC == 0 {

	// Median split heuristic
	@(private = "file")
	PartitionMid :: proc(indices: [^]i32, centers: [^]Vec3, count: i32) -> i32 {
		// Handle trivial case
		if count <= 2 {
			return count / 2
		}

		lowerBound := centers[0]
		upperBound := centers[0]

		for i: i32 = 1; i < count; i += 1 {
			lowerBound = Min(lowerBound, centers[i])
			upperBound = Max(upperBound, centers[i])
		}

		d := Sub(upperBound, lowerBound)
		c := MulSV(0.5, Add(lowerBound, upperBound))

		// Partition longest axis using the Hoare partition scheme
		// https://en.wikipedia.org/wiki/Quicksort
		// https://nicholasvadivelu.com/2021/01/11/array-partition/
		i1, i2: i32 = 0, count
		if d.x >= d.y && d.x >= d.z {
			pivot := c.x

			for i1 < i2 {
				for i1 < i2 && centers[i1].x < pivot {
					i1 += 1
				}

				for i1 < i2 && centers[i2 - 1].x >= pivot {
					i2 -= 1
				}

				if i1 < i2 {
					// Swap indices
					{
						temp := indices[i1]
						indices[i1] = indices[i2 - 1]
						indices[i2 - 1] = temp
					}

					// Swap centers
					{
						temp := centers[i1]
						centers[i1] = centers[i2 - 1]
						centers[i2 - 1] = temp
					}

					i1 += 1
					i2 -= 1
				}
			}
		} else if d.y >= d.z {
			pivot := c.y

			for i1 < i2 {
				for i1 < i2 && centers[i1].y < pivot {
					i1 += 1
				}

				for i1 < i2 && centers[i2 - 1].y >= pivot {
					i2 -= 1
				}

				if i1 < i2 {
					// Swap indices
					{
						temp := indices[i1]
						indices[i1] = indices[i2 - 1]
						indices[i2 - 1] = temp
					}

					// Swap centers
					{
						temp := centers[i1]
						centers[i1] = centers[i2 - 1]
						centers[i2 - 1] = temp
					}

					i1 += 1
					i2 -= 1
				}
			}
		} else {
			pivot := c.z

			for i1 < i2 {
				for i1 < i2 && centers[i1].z < pivot {
					i1 += 1
				}

				for i1 < i2 && centers[i2 - 1].z >= pivot {
					i2 -= 1
				}

				if i1 < i2 {
					// Swap indices
					{
						temp := indices[i1]
						indices[i1] = indices[i2 - 1]
						indices[i2 - 1] = temp
					}

					// Swap centers
					{
						temp := centers[i1]
						centers[i1] = centers[i2 - 1]
						centers[i2 - 1] = temp
					}

					i1 += 1
					i2 -= 1
				}
			}
		}
		assert(i1 == i2)

		if i1 > 0 && i1 < count {
			return i1
		}

		return count / 2
	}

} else {

	BIN_COUNT :: 8

	TreeBin :: struct {
		aabb:  AABB,
		count: i32,
	}

	TreePlane :: struct {
		leftAABB:   AABB,
		rightAABB:  AABB,
		leftCount:  i32,
		rightCount: i32,
	}

	// "On Fast Construction of SAH-based Bounding Volume Hierarchies" by Ingo Wald
	// Returns the left child count
	@(private = "file")
	PartitionSAH :: proc(indices: [^]i32, binIndices: [^]i32, boxes: [^]AABB, count: i32) -> i32 {
		assert(count > 0)

		bins: [BIN_COUNT]TreeBin
		planes: [BIN_COUNT - 1]TreePlane

		center := AABB_Center(boxes[0])
		centroidAABB: AABB
		centroidAABB.lowerBound = center
		centroidAABB.upperBound = center

		for i: i32 = 1; i < count; i += 1 {
			center = AABB_Center(boxes[i])
			centroidAABB.lowerBound = Min(centroidAABB.lowerBound, center)
			centroidAABB.upperBound = Max(centroidAABB.upperBound, center)
		}

		d := Sub(centroidAABB.upperBound, centroidAABB.lowerBound)

		// Find longest axis
		axisIndex: i32
		invD: f32
		if d.x > d.y {
			axisIndex = 0
			invD = d.x
		} else {
			axisIndex = 1
			invD = d.y
		}

		invD = invD > 0.0 ? 1.0 / invD : 0.0

		// Initialize bin bounds and count
		for i: i32 = 0; i < BIN_COUNT; i += 1 {
			bins[i].aabb.lowerBound = Vec3{max(f32), max(f32), 0.0}
			bins[i].aabb.upperBound = Vec3{-max(f32), -max(f32), 0.0}
			bins[i].count = 0
		}

		// Assign boxes to bins and compute bin boxes
		// TODO_ERIN optimize
		binCount: f32 = BIN_COUNT
		lowerBoundArray := [2]f32{centroidAABB.lowerBound.x, centroidAABB.lowerBound.y}
		minC := lowerBoundArray[axisIndex]
		for i: i32 = 0; i < count; i += 1 {
			c := AABB_Center(boxes[i])
			cArray := [2]f32{c.x, c.y}
			binIndex := i32(binCount * (cArray[axisIndex] - minC) * invD)
			binIndex = ClampInt(binIndex, 0, BIN_COUNT - 1)
			binIndices[i] = binIndex
			bins[binIndex].count += 1
			bins[binIndex].aabb = AABB_Union(bins[binIndex].aabb, boxes[i])
		}

		planeCount: i32 = BIN_COUNT - 1

		// Prepare all the left planes, candidates for left child
		planes[0].leftCount = bins[0].count
		planes[0].leftAABB = bins[0].aabb
		for i: i32 = 1; i < planeCount; i += 1 {
			planes[i].leftCount = planes[i - 1].leftCount + bins[i].count
			planes[i].leftAABB = AABB_Union(planes[i - 1].leftAABB, bins[i].aabb)
		}

		// Prepare all the right planes, candidates for right child
		planes[planeCount - 1].rightCount = bins[planeCount].count
		planes[planeCount - 1].rightAABB = bins[planeCount].aabb
		for i := planeCount - 2; i >= 0; i -= 1 {
			planes[i].rightCount = planes[i + 1].rightCount + bins[i + 1].count
			planes[i].rightAABB = AABB_Union(planes[i + 1].rightAABB, bins[i + 1].aabb)
		}

		// Find best split to minimize SAH
		minCost := max(f32)
		bestPlane: i32 = 0
		for i: i32 = 0; i < planeCount; i += 1 {
			leftArea := Perimeter(planes[i].leftAABB)
			rightArea := Perimeter(planes[i].rightAABB)
			leftCount := planes[i].leftCount
			rightCount := planes[i].rightCount

			cost := f32(leftCount) * leftArea + f32(rightCount) * rightArea
			if cost < minCost {
				bestPlane = i
				minCost = cost
			}
		}

		// Partition node indices and boxes using the Hoare partition scheme
		// https://en.wikipedia.org/wiki/Quicksort
		// https://nicholasvadivelu.com/2021/01/11/array-partition/
		i1, i2: i32 = 0, count
		for i1 < i2 {
			for i1 < i2 && binIndices[i1] < bestPlane {
				i1 += 1
			}

			for i1 < i2 && binIndices[i2 - 1] >= bestPlane {
				i2 -= 1
			}

			if i1 < i2 {
				// Swap indices
				{
					temp := indices[i1]
					indices[i1] = indices[i2 - 1]
					indices[i2 - 1] = temp
				}

				// Swap boxes
				{
					temp := boxes[i1]
					boxes[i1] = boxes[i2 - 1]
					boxes[i2 - 1] = temp
				}

				i1 += 1
				i2 -= 1
			}
		}
		assert(i1 == i2)

		if i1 > 0 && i1 < count {
			return i1
		} else {
			return count / 2
		}
	}

}

// Temporary data used to track the rebuild of a tree node.
@(private = "file")
RebuildItem :: struct {
	// Where this node is written and the pair its children go in.
	nodeIndex:  i32,
	pair:       i32,
	childCount: i32,

	// Leaf indices
	startIndex: i32,
	splitIndex: i32,
	endIndex:   i32,
}

@(private = "file")
CopyItem :: struct {
	oldPair:  i32,
	newIndex: i32,
}

// Bump allocate a pair of sibling nodes. Returns index to the first one.
@(private = "file")
BumpPair :: proc(tree: ^DynamicTree, parent: i32) -> i32 {
	pair := tree.nodeEnd
	assert(pair + 2 <= tree.nodeCapacity)
	tree.nodeEnd += 2
	tree.parents[pair] = parent
	tree.parents[pair + 1] = parent
	return pair
}

@(private = "file")
SetLeftChild :: proc(node: ^TreeNode, pair: i32) {
	node.flagIndex = (node.flagIndex & ~NODE_INDEX_MASK) | u32(pair)
}

// Copy a retained subtree from the old array into the rebuilt DFS array using
// a bump allocator.
@(private = "file")
CopySubtree :: proc(tree: ^DynamicTree, node: TreeNode, newIndex: i32) {
	node := node
	oldNodes := tree.nodes
	newNodes := tree.swapNodes
	proxies := tree.proxies

	if IsLeaf(&node) {
		newNodes[newIndex] = node
		proxies[GetProxyId(&node)].node = newIndex
		return
	}

	stack: [TREE_STACK_SIZE]CopyItem = ---
	stackCount: i32 = 0

	oldPair := GetLeftChild(&node)
	newPair := BumpPair(tree, newIndex)
	SetLeftChild(&node, newPair)

	// Copy the subtree root. This can be a left or right sibling (even or odd).
	newNodes[newIndex] = node

	for {
		pair := newNodes[newPair:]

		// Copy the siblings.
		pair[0] = oldNodes[oldPair]
		pair[1] = oldNodes[oldPair + 1]

		if IsLeaf(&pair[1]) {
			// Hook up proxy on right sibling.
			proxyId := GetProxyId(&pair[1])
			proxies[proxyId].node = newPair + 1
		} else {
			// Push the left child of the right sibling.
			if stackCount < TREE_STACK_SIZE {
				leftChild := GetLeftChild(&pair[1])
				stack[stackCount] = CopyItem{leftChild, newPair + 1}
				stackCount += 1
			} else {
				// todo fail gracefully if the stack runs out in release
				assert(stackCount < TREE_STACK_SIZE)
			}
		}

		if IsLeaf(&pair[0]) == false {
			// Descend into left subtree.
			leftIndex := newPair
			oldPair = GetLeftChild(&pair[0])
			newPair = BumpPair(tree, leftIndex)
			SetLeftChild(&newNodes[leftIndex], newPair)
			continue
		}

		// Hook up proxy on left sibling.
		proxies[GetProxyId(&pair[0])].node = newPair

		if stackCount == 0 {
			break
		}

		// Descend a right subtree that was previously pushed.
		stackCount -= 1
		item := stack[stackCount]
		oldPair = item.oldPair
		newPair = BumpPair(tree, item.newIndex)
		SetLeftChild(&newNodes[item.newIndex], newPair)
	}
}

@(private = "file")
PlaceLeaf :: proc(tree: ^DynamicTree, leaf: TreeNode, newIndex: i32) {
	leaf := leaf
	if IsLeaf(&leaf) {
		tree.swapNodes[newIndex] = leaf
		tree.proxies[GetProxyId(&leaf)].node = newIndex
	} else {
		CopySubtree(tree, leaf, newIndex)
	}
}

@(private = "file")
BuildTree :: proc(tree: ^DynamicTree, leafCount: i32) {
	nodes := tree.swapNodes
	leaves := tree.leafNodes
	leafIndices := tree.leafIndices

	when TREE_HEURISTIC == 0 {
		leafCenters := tree.leafCenters
	} else {
		leafBoxes := tree.leafBoxes
		binIndices := tree.binIndices
	}

	// Bump allocation into the swap nodes.
	tree.nodeEnd = 2
	nodes[ROOT_NODE + 1] = MakeEmptyNode()
	tree.parents[ROOT_NODE] = NULL_INDEX
	tree.parents[ROOT_NODE + 1] = NULL_INDEX

	if leafCount == 1 {
		PlaceLeaf(tree, leaves[leafIndices[0]], ROOT_NODE)
		return
	}

	stack: [TREE_STACK_SIZE]RebuildItem = ---
	top: i32 = 0

	stack[0].nodeIndex = ROOT_NODE
	stack[0].pair = BumpPair(tree, ROOT_NODE)
	stack[0].childCount = -1
	stack[0].startIndex = 0
	stack[0].endIndex = leafCount
	when TREE_HEURISTIC == 0 {
		stack[0].splitIndex = PartitionMid(leafIndices, leafCenters, leafCount)
	} else {
		stack[0].splitIndex = PartitionSAH(leafIndices, binIndices, leafBoxes, leafCount)
	}

	for true {
		item := &stack[top]
		item.childCount += 1

		if item.childCount == 2 {
			// Both children written, so the parent node can be finalized.
			nodes[item.nodeIndex] = MakeInternalNode(nodes, item.pair)
			if top == 0 {
				break
			}

			top -= 1
			continue
		}

		slot := item.childCount
		startIndex := slot == 0 ? item.startIndex : item.splitIndex
		endIndex := slot == 0 ? item.splitIndex : item.endIndex
		count := endIndex - startIndex
		assert(count > 0)

		nodeIndex := item.pair + slot
		if count == 1 {
			PlaceLeaf(tree, leaves[leafIndices[startIndex]], nodeIndex)
			continue
		}

		// todo fail gracefully if the stack runs out in release
		assert(top < TREE_STACK_SIZE - 1)

		top += 1
		newItem := &stack[top]
		newItem.nodeIndex = nodeIndex
		newItem.pair = BumpPair(tree, nodeIndex)
		newItem.childCount = -1
		newItem.startIndex = startIndex
		newItem.endIndex = endIndex
		newItem.splitIndex = startIndex
		when TREE_HEURISTIC == 0 {
			newItem.splitIndex += PartitionMid(leafIndices[startIndex:], leafCenters[startIndex:], count)
		} else {
			newItem.splitIndex += PartitionSAH(
				leafIndices[startIndex:],
				binIndices[startIndex:],
				leafBoxes[startIndex:],
				count,
			)
		}
	}
}

// Rebuild the stale parts of the tree. The entire tree is put into DFS order. This makes
// refitting much faster. This is done async with threading, so the cost is hidden.
// Not safe to access tree during this operation.
DynamicTree_Rebuild :: proc(tree: ^DynamicTree, fullBuild: bool) -> i32 {
	proxyCount := tree.proxyCount
	if proxyCount == 0 {
		return 0
	}

	// An unordered tree is rebuilt even when nothing moved, the sweep refit needs the order
	nodes := tree.nodes
	root := &nodes[ROOT_NODE]
	if fullBuild == false && IsNodeMoved(root) == false && tree.dfsOrdered {
		return 0
	}

	if tree.swapNodes == nil {
		tree.swapNodes = cast([^]TreeNode)AllocZero(int(tree.nodeCapacity) * size_of(TreeNode))
	}

	if proxyCount > tree.rebuildCapacity {
		oldCapacity := tree.rebuildCapacity
		newCapacity := proxyCount + proxyCount / 2

		Free(tree.leafIndices, int(oldCapacity) * size_of(i32))
		tree.leafIndices = cast([^]i32)Alloc(int(newCapacity) * size_of(i32))

		Free(tree.leafNodes, int(oldCapacity) * size_of(TreeNode))
		tree.leafNodes = cast([^]TreeNode)Alloc(int(newCapacity) * size_of(TreeNode))

		when TREE_HEURISTIC == 0 {
			Free(tree.leafCenters, int(oldCapacity) * size_of(Vec3))
			tree.leafCenters = cast([^]Vec3)Alloc(int(newCapacity) * size_of(Vec3))
		} else {
			Free(tree.leafBoxes, int(oldCapacity) * size_of(AABB))
			tree.leafBoxes = cast([^]AABB)Alloc(int(newCapacity) * size_of(AABB))
			Free(tree.binIndices, int(oldCapacity) * size_of(i32))
			tree.binIndices = cast([^]i32)Alloc(int(newCapacity) * size_of(i32))
		}
		tree.rebuildCapacity = newCapacity
	}

	leafIndices := tree.leafIndices
	leaves := tree.leafNodes
	when TREE_HEURISTIC == 0 {
		leafCenters := tree.leafCenters
	} else {
		leafBoxes := tree.leafBoxes
	}

	// Gather build leaves. A marked internal node is descended and abandoned in the old array.
	// Everything else is a build leaf, kept subtrees included. A leaf root, or an unmarked root
	// on an unordered array, is the one build leaf.
	leafCount: i32 = 0
	stack: [TREE_STACK_SIZE]i32 = ---
	stackCount: i32 = 0
	if IsLeaf(root) == false && (fullBuild || IsNodeMoved(root)) {
		stack[stackCount] = GetLeftChild(root)
		stackCount += 1
	} else {
		node := root^
		if fullBuild == false {
			node.flagIndex &= ~MOVED_NODE
		}
		leafIndices[0] = 0
		leaves[0] = node
		when TREE_HEURISTIC == 0 {
			leafCenters[0] = AABB_Center(node.aabb)
		} else {
			leafBoxes[0] = node.aabb
		}
		leafCount = 1
	}

	for stackCount > 0 {
		stackCount -= 1
		pair := stack[stackCount]
		for i: i32 = 0; i < 2; i += 1 {
			node := nodes[pair + i]
			if IsLeaf(&node) == false && (fullBuild || IsNodeMoved(&node)) {
				// todo fail gracefully if the stack runs out in release
				assert(stackCount < TREE_STACK_SIZE)
				stack[stackCount] = GetLeftChild(&node)
				stackCount += 1
				continue
			}

			if fullBuild == false {
				node.flagIndex &= ~MOVED_NODE
			}
			leafIndices[leafCount] = leafCount
			leaves[leafCount] = node
			when TREE_HEURISTIC == 0 {
				leafCenters[leafCount] = AABB_Center(node.aabb)
			} else {
				leafBoxes[leafCount] = node.aabb
			}
			leafCount += 1
		}
	}

	assert(0 < leafCount && leafCount <= proxyCount)

	BuildTree(tree, leafCount)

	// The spare is now the tree and the old array is the spare. The build is dense, so the
	// stale tail of the old array sits above the end and is never read.
	tree.nodes, tree.swapNodes = tree.swapNodes, tree.nodes
	tree.pairFreeList = NULL_INDEX
	tree.dfsOrdered = true

	DynamicTree_Validate(tree)
	if fullBuild == false {
		DynamicTree_ValidateNoMoved(tree)
	}

	return leafCount
}

// Set the moved flag on the ancestors of the proxy. Serial use case.
DynamicTree_MarkProxyMovedSerial :: proc(tree: ^DynamicTree, proxyId: i32) {
	VALIDATE(0 <= proxyId && proxyId < tree.proxyCapacity)

	nodes := tree.nodes
	parents := tree.parents

	index := tree.proxies[proxyId].node
	VALIDATE(0 <= index && index < tree.nodeEnd)
	VALIDATE(IsLeaf(&nodes[index]))

	for index != NULL_INDEX {
		nodes[index].flagIndex |= MOVED_NODE
		index = parents[index]
	}
}

// Update a proxy AABB and flag the ancestors as moved. Thread-safe using atomics.
DynamicTree_MarkProxyMoved :: proc(tree: ^DynamicTree, proxyId: i32, aabb: AABB) {
	VALIDATE(0 <= proxyId && proxyId < tree.proxyCapacity)

	nodes := tree.nodes
	parents := tree.parents

	index := tree.proxies[proxyId].node
	VALIDATE(0 <= index && index < tree.nodeEnd)

	node := &nodes[index]
	VALIDATE(IsLeaf(node))
	VALIDATE(AABB_Contains(node.aabb, aabb) == false)

	// This is not raced since it is the leaf.
	node.aabb = aabb
	node.flagIndex |= MOVED_NODE

	index = parents[index]
	for index != NULL_INDEX {
		node = &nodes[index]

		// Read first to avoid the FetchOr if possible.
		if AtomicLoadU32Raw(&node.flagIndex) & MOVED_NODE != 0 {
			break
		}

		previousFlags := AtomicFetchOrU32(&node.flagIndex, MOVED_NODE)
		if previousFlags & MOVED_NODE != 0 {
			// Ancestor already visited.
			break
		}

		index = parents[index]
	}
}

// Clear the moved flags from the entire tree.
DynamicTree_ClearMoved :: proc(tree: ^DynamicTree) {
	nodes := tree.nodes

	root := &nodes[ROOT_NODE]
	if IsNodeMoved(root) == false {
		return
	}

	root.flagIndex &= ~MOVED_NODE
	if IsLeaf(root) {
		return
	}

	stack: [TREE_STACK_SIZE]i32 = ---
	stackCount: i32 = 0
	stack[stackCount] = GetLeftChild(root)
	stackCount += 1

	for stackCount > 0 {
		stackCount -= 1
		pair := stack[stackCount]
		for i: i32 = 0; i < 2; i += 1 {
			node := &nodes[pair + i]
			if node.flagIndex & MOVED_NODE != 0 {
				node.flagIndex &= ~MOVED_NODE
				if IsLeaf(node) == false {
					if stackCount < TREE_STACK_SIZE {
						stack[stackCount] = GetLeftChild(node)
						stackCount += 1
					} else {
						// Bad stuff will happen if the moved flags don't get cleared.
						assert(stackCount < TREE_STACK_SIZE)
					}
				}
			}
		}
	}
}

DynamicTree_GatherMovedProxies :: proc(tree: ^DynamicTree, proxyIds: [^]i32) -> i32 {
	nodes := tree.nodes
	root := &nodes[ROOT_NODE]
	if IsNodeMoved(root) == false {
		return 0
	}

	if IsLeaf(root) {
		proxyIds[0] = GetProxyId(root)
		return 1
	}

	count: i32 = 0
	stack: [TREE_STACK_SIZE]i32 = ---
	stackCount: i32 = 0
	stack[stackCount] = GetLeftChild(root)
	stackCount += 1

	for stackCount > 0 {
		stackCount -= 1
		pair := stack[stackCount]
		for i: i32 = 0; i < 2; i += 1 {
			node := &nodes[pair + i]
			if IsNodeMoved(node) == false {
				continue
			}

			if IsLeaf(node) {
				proxyIds[count] = GetProxyId(node)
				count += 1
			} else {
				if stackCount < TREE_STACK_SIZE {
					stack[stackCount] = GetLeftChild(node)
					stackCount += 1
				} else {
					assert(stackCount < TREE_STACK_SIZE)
				}
			}
		}
	}

	return count
}

// Slow refit for unit tests.
@(private = "file")
RefitSubtree :: proc(nodes: [^]TreeNode, rootIndex: i32) {
	root := &nodes[rootIndex]
	if IsLeaf(root) || IsNodeMoved(root) == false {
		return
	}

	// Negative entries indicate deferred unions. A parent is pushed before its children so the
	// LIFO order guarantees both children are finished when the union runs.
	stack: [TREE_STACK_SIZE]i32 = ---
	stackCount: i32 = 0
	// ~0 == -1
	stack[stackCount] = ~rootIndex
	stackCount += 1
	stack[stackCount] = GetLeftChild(root)
	stackCount += 1
	stack[stackCount] = GetLeftChild(root) + 1
	stackCount += 1

	for stackCount > 0 {
		stackCount -= 1
		item := stack[stackCount]
		if item < 0 {
			node := &nodes[~item]
			pair := GetLeftChild(node)
			node.aabb = UnionV(nodes[pair].aabb, nodes[pair + 1].aabb)
			continue
		}

		node := &nodes[item]
		if IsLeaf(node) || IsNodeMoved(node) == false {
			continue
		}

		if stackCount + 3 <= TREE_STACK_SIZE {
			pair := GetLeftChild(node)
			stack[stackCount] = ~item
			stackCount += 1
			stack[stackCount] = pair
			stackCount += 1
			stack[stackCount] = pair + 1
			stackCount += 1
		} else {
			assert(stackCount + 3 <= TREE_STACK_SIZE)
		}
	}
}

DynamicTree_Refit :: proc(tree: ^DynamicTree) {
	if HasTreeMoved(tree) == false {
		return
	}

	nodes := tree.nodes

	if tree.dfsOrdered == false {
		// This only happens in unit tests.
		RefitSubtree(nodes, ROOT_NODE)
		return
	}

	for pair := tree.nodeEnd - 2; pair >= 0; pair -= 2 {
		node := nodes[pair:]
		flags1 := node[0].flagIndex
		flags2 := node[1].flagIndex

		// Did either move?
		if ((flags1 | flags2) & MOVED_NODE) == 0 {
			continue
		}

		// Is the node internal and moved?
		refit1 := (flags1 & (LEAF_NODE | MOVED_NODE)) == MOVED_NODE
		refit2 := (flags2 & (LEAF_NODE | MOVED_NODE)) == MOVED_NODE

		// If the node needs a refit then get the child index. Otherwise use a valid dummy index.
		// The store function won't actually modify the node at the dummy index. This
		// avoids an unpredictable branch. SIMD FTW.
		children1 := refit1 ? i32(flags1 & NODE_INDEX_MASK) : pair
		children2 := refit2 ? i32(flags2 & NODE_INDEX_MASK) : pair

		// Conditionally store the union.
		StoreAABBV(&node[0].aabb, UnionPairV(&nodes[children1]), refit1)
		StoreAABBV(&node[1].aabb, UnionPairV(&nodes[children2]), refit2)
	}
}

// unified: core:os instead of stdio; the mode is "wb" (create) or "rb" (open)
@(private = "file")
OpenTreeFile :: proc(fileName: cstring, mode: cstring) -> ^os.File {
	file: ^os.File
	err: os.Error
	if mode == "wb" {
		file, err = os.create(string(fileName))
	} else {
		file, err = os.open(string(fileName))
	}
	if err != nil {
		return nil
	}

	return file
}
// unified end

DynamicTree_Save :: proc(tree: ^DynamicTree, fileName: cstring) {
	if tree.nodes == nil || tree.parents == nil || tree.proxies == nil {
		return
	}

	file := OpenTreeFile(fileName, "wb")
	if file == nil {
		return
	}

	// Copy to allow setting some fields to zero
	temp := tree^

	// Zero pointers and temp data
	temp.nodes = nil
	temp.parents = nil
	temp.proxies = nil
	temp.swapNodes = nil
	temp.leafIndices = nil
	temp.leafNodes = nil
	temp.leafBoxes = nil
	temp.leafCenters = nil
	temp.binIndices = nil
	temp.rebuildCapacity = 0

	temp.nodeCapacity = tree.nodeEnd

	// unified: os.write instead of fwrite
	_, _ = os.write(file, mem.ptr_to_bytes(&temp))

	if tree.nodeEnd > 0 {
		_, _ = os.write(file, mem.byte_slice(tree.nodes, int(tree.nodeEnd) * size_of(TreeNode)))
		_, _ = os.write(file, mem.byte_slice(tree.parents, int(tree.nodeEnd) * size_of(i32)))
	}

	if tree.proxyCapacity > 0 {
		_, _ = os.write(file, mem.byte_slice(tree.proxies, int(tree.proxyCapacity) * size_of(TreeProxy)))
	}

	os.close(file)
	// unified end
}

DynamicTree_Load :: proc(fileName: cstring, scale: f32) -> DynamicTree {
	tree := DynamicTree{}

	file := OpenTreeFile(fileName, "rb")
	if file == nil {
		return tree
	}

	// unified: os.read instead of fread; a read is complete when it returns the full byte count
	readCount, _ := os.read(file, mem.ptr_to_bytes(&tree))
	if readCount != size_of(DynamicTree) {
		os.close(file)
		mem.zero(&tree, size_of(DynamicTree))
		return tree
	}
	// unified end

	if tree.version != DYNAMIC_TREE_VERSION {
		os.close(file)
		mem.zero(&tree, size_of(DynamicTree))
		return tree
	}

	tree.nodes = nil
	tree.parents = nil
	tree.proxies = nil
	tree.swapNodes = nil
	tree.leafIndices = nil
	tree.leafNodes = nil
	tree.leafBoxes = nil
	tree.leafCenters = nil
	tree.binIndices = nil
	tree.rebuildCapacity = 0

	ok :=
		tree.nodeCapacity >= 2 &&
		tree.proxyCapacity >= 1 &&
		(tree.nodeEnd & 1) == 0 &&
		tree.nodeEnd >= 2 &&
		tree.nodeEnd <= tree.nodeCapacity

	if ok {
		tree.nodes = cast([^]TreeNode)Alloc(int(tree.nodeCapacity) * size_of(TreeNode))
		tree.parents = cast([^]i32)Alloc(int(tree.nodeCapacity) * size_of(i32))
		tree.proxies = cast([^]TreeProxy)Alloc(int(tree.proxyCapacity) * size_of(TreeProxy))

		// unified: os.read instead of fread
		nodeBytes := int(tree.nodeCapacity) * size_of(TreeNode)
		n, _ := os.read(file, mem.byte_slice(tree.nodes, nodeBytes))
		ok = n == nodeBytes
		if ok {
			parentBytes := int(tree.nodeCapacity) * size_of(i32)
			n, _ = os.read(file, mem.byte_slice(tree.parents, parentBytes))
			ok = n == parentBytes
		}
		if ok {
			proxyBytes := int(tree.proxyCapacity) * size_of(TreeProxy)
			n, _ = os.read(file, mem.byte_slice(tree.proxies, proxyBytes))
			ok = n == proxyBytes
		}
		// unified end
	}

	if ok == false {
		Free(tree.nodes, int(tree.nodeCapacity) * size_of(TreeNode))
		Free(tree.parents, int(tree.nodeCapacity) * size_of(i32))
		Free(tree.proxies, int(tree.proxyCapacity) * size_of(TreeProxy))
		os.close(file)
		mem.zero(&tree, size_of(DynamicTree))
		return tree
	}

	// An empty node has infinite bounds, so the scale has to skip it.
	for i: i32 = 0; i < tree.nodeEnd; i += 1 {
		node := &tree.nodes[i]
		if IsEmptyNode(node) {
			continue
		}

		node.aabb.lowerBound = MulSV(scale, node.aabb.lowerBound)
		node.aabb.upperBound = MulSV(scale, node.aabb.upperBound)
	}

	os.close(file)

	return tree
}
