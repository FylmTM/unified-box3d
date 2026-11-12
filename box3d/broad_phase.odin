// Ported from box3d src/broad_phase.c and src/broad_phase.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:runtime"
import "core:mem"

// Store the proxy type in the lower 2 bits of the proxy key. This leaves 30 bits for the id.
PROXY_TYPE :: #force_inline proc(KEY: i32) -> BodyType {
	return BodyType(KEY & 3)
}

PROXY_ID :: #force_inline proc(KEY: i32) -> i32 {
	return KEY >> 2
}

PROXY_KEY :: #force_inline proc(ID: i32, TYPE: BodyType) -> i32 {
	return (ID << 2) | i32(TYPE)
}

// The broad-phase is used for computing pairs and performing volume queries and ray casts.
// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
// It is up to the client to consume the new pairs and to track subsequent overlap.
BroadPhase :: struct {
	// One tree for each body type.
	trees:         [i32(BodyType.bodyTypeCount)]DynamicTree,

	// The moved siblings gathered from the dynamic body tree.
	movedSiblings: [^]i32,

	// Tracks shape pairs that have a b3Contact
	// todo pairSet can grow quite large on the first time step and remain large
	pairSet:       HashSet,
}

BroadPhase_MarkProxyMovedSerial :: proc(bp: ^BroadPhase, proxyKey: i32) {
	proxyType := PROXY_TYPE(proxyKey)
	proxyId := PROXY_ID(proxyKey)
	DynamicTree_MarkProxyMovedSerial(&bp.trees[proxyType], proxyId)
}

BroadPhase_MarkProxyMoved :: proc(bp: ^BroadPhase, proxyKey: i32, aabb: AABB) {
	proxyType := PROXY_TYPE(proxyKey)
	proxyId := PROXY_ID(proxyKey)
	DynamicTree_MarkProxyMoved(&bp.trees[proxyType], proxyId, aabb)
}

// broad_phase.c

CreateBroadPhase :: proc(bp: ^BroadPhase, capacity: ^Capacity) {
	#assert(BodyType.bodyTypeCount == BodyType(3), "must be three body types")

	bp.pairSet = CreateSet(MaxInt(32, 2 * capacity.contactCount))

	staticCapacity := MaxInt(16, capacity.staticShapeCount)
	bp.trees[BodyType.staticBody] = DynamicTree_Create(staticCapacity)

	kinematicCapacity: i32 = 16
	bp.trees[BodyType.kinematicBody] = DynamicTree_Create(kinematicCapacity)

	dynamicCapacity := MaxInt(16, capacity.dynamicShapeCount)
	bp.trees[BodyType.dynamicBody] = DynamicTree_Create(dynamicCapacity)

	bp.movedSiblings = nil
}

DestroyBroadPhase :: proc(bp: ^BroadPhase) {
	for i: i32 = 0; i < i32(BodyType.bodyTypeCount); i += 1 {
		DynamicTree_Destroy(&bp.trees[i])
	}

	DestroySet(&bp.pairSet)

	mem.zero(bp, size_of(BroadPhase))
}

BroadPhase_CreateProxy :: proc(
	bp: ^BroadPhase,
	proxyType: BodyType,
	aabb: AABB,
	categoryBits: u64,
	shapeIndex: i32,
	forcePairCreation: bool,
) -> i32 {
	assert(BodyType(0) <= proxyType && proxyType < BodyType.bodyTypeCount)

	mark := (proxyType != .staticBody || forcePairCreation)

	proxyId := CreateTreeProxyInternal(&bp.trees[proxyType], aabb, categoryBits, u64(shapeIndex), mark)
	proxyKey := PROXY_KEY(proxyId, proxyType)
	return proxyKey
}

BroadPhase_DestroyProxy :: proc(bp: ^BroadPhase, proxyKey: i32) {
	proxyType := PROXY_TYPE(proxyKey)
	proxyId := PROXY_ID(proxyKey)

	assert(BodyType(0) <= proxyType && proxyType < BodyType.bodyTypeCount)
	DynamicTree_DestroyProxy(&bp.trees[proxyType], proxyId)
}

BroadPhase_MoveProxy :: proc(bp: ^BroadPhase, proxyKey: i32, aabb: AABB) {
	proxyType := PROXY_TYPE(proxyKey)
	proxyId := PROXY_ID(proxyKey)

	mark := true
	DynamicTree_MoveProxyInternal(&bp.trees[proxyType], proxyId, aabb, mark)
}

// Gather the sibling pairs with a moved node. This is done serially, it is cache friendly.
@(private = "file")
GatherMovedSiblings :: proc(tree: ^DynamicTree, pairIndices: [^]i32) -> i32 {
	nodes := tree.nodes
	nodeEnd := tree.nodeEnd

	count: i32 = 0

	// Skip the root.
	for pair: i32 = 2; pair < nodeEnd; pair += 2 {
		// Push when either sibling moved.
		if ((nodes[pair].flagIndex | nodes[pair + 1].flagIndex) & MOVED_NODE) != 0 {
			pairIndices[count] = pair
			count += 1
		}
	}

	return count
}

@(private = "file")
CANDIDATE_BATCH :: 32

CandidatePair :: struct {
	shapeIdA: i32,
	shapeIdB: i32,
}

PairContext :: struct {
	world:          ^World,
	pairKeys:       ^Array(u64),
	batch:          [CANDIDATE_BATCH]CandidatePair,
	batchCount:     i32,

	// Compound shapes are static only, so only the static cross pass can meet one. Elsewhere
	// this stays false and the batch never reads a shape type.
	checkCompounds: bool,
}

NodePair :: struct {
	a: TreeNode,
	b: TreeNode,
}

IndexPair :: struct {
	a, b: i32,
}

@(private = "file")
PrefetchHash :: #force_inline proc(set: ^HashSet, hash: u32) {
	capacity := set.capacity
	index := hash & (capacity - 1)
	Prefetch(&set.items[index])
}

// The shape pair gauntlet. Everything here is a property of the pair, not of a compound child,
// so a compound runs it once for the whole pair.
@(private = "file")
ShouldCreatePair :: proc(world: ^World, shapeIdA: i32, shapeIdB: i32) -> bool {
	shapeA := Array_Get(&world.shapes, shapeIdA)
	shapeB := Array_Get(&world.shapes, shapeIdB)

	bodyIdA := shapeA.bodyId
	bodyIdB := shapeB.bodyId

	// Are the shapes on the same body?
	if bodyIdA == bodyIdB {
		return false
	}

	// Sensors are handled elsewhere
	if shapeA.sensorIndex != NULL_INDEX || shapeB.sensorIndex != NULL_INDEX {
		return false
	}

	if ShouldShapesCollide(shapeA.filter, shapeB.filter) == false {
		return false
	}

	// Does a joint override collision?
	bodyA := Array_Get(&world.bodies, bodyIdA)
	bodyB := Array_Get(&world.bodies, bodyIdB)
	if ShouldBodiesCollide(world, bodyA, bodyB) == false {
		return false
	}

	// Custom user filter
	if (shapeA.flags & enableCustomFiltering) != 0 || (shapeB.flags & enableCustomFiltering) != 0 {
		customFilterFcn := world.customFilterFcn
		if customFilterFcn != nil {
			idA := ShapeId{shapeIdA + 1, world.worldId, shapeA.generation}
			idB := ShapeId{shapeIdB + 1, world.worldId, shapeB.generation}
			if customFilterFcn(idA, idB, world.customFilterContext) == false {
				return false
			}
		}
	}

	return true
}

CompoundPairContext :: struct {
	pairContext:     ^PairContext,
	pairSet:         ^HashSet,
	compoundShapeId: i32,
	otherShapeId:    i32,
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
CompoundChildCallback :: proc "c" (proxyId: i32, userData: u64, context_: rawptr) -> bool {
	context = runtime.default_context()
	// unified end
	_ = proxyId

	compoundContext := (^CompoundPairContext)(context_)
	childIndex := i32(userData)

	key := ShapePairKey(compoundContext.compoundShapeId, compoundContext.otherShapeId, childIndex)
	if ContainsKey(compoundContext.pairSet, key) == false {
		Array_Push(compoundContext.pairContext.pairKeys, key)
	}

	return true
}

// One of the two shapes is a compound, so the pair becomes one pair per overlapping child. The
// child boxes live in the compound frame, so the other shape's box has to be demoted and pulled
// into that frame first.
@(private = "file")
EmitCompoundPairs :: proc(context_: ^PairContext, compoundShapeId: i32, otherShapeId: i32) {
	world := context_.world

	compoundShape := Array_Get(&world.shapes, compoundShapeId)
	otherShape := Array_Get(&world.shapes, otherShapeId)

	otherProxyKey := otherShape.proxyKey
	assert(otherProxyKey != NULL_INDEX)
	otherAABB := DynamicTree_GetAABB(&world.broadPhase.trees[PROXY_TYPE(otherProxyKey)], PROXY_ID(otherProxyKey))

	// Query bounds are float world space, so the demoted transform is the matching float frame
	compoundTransform := ToRelativeTransform(GetBodyTransform(world, compoundShape.bodyId), Pos_zero)
	localAABB := AABB_Transform(InvertTransform(compoundTransform), otherAABB)

	compoundContext := CompoundPairContext {
		pairContext     = context_,
		pairSet         = &world.broadPhase.pairSet,
		compoundShapeId = compoundShapeId,
		otherShapeId    = otherShapeId,
	}

	startCount := context_.pairKeys.count

	view := GetCompoundTreeView(compoundShape.compound)
	TreeView_Query(&view, localAABB, DEFAULT_MASK_BITS, false, CompoundChildCallback, &compoundContext)

	if context_.pairKeys.count > startCount && ShouldCreatePair(world, compoundShapeId, otherShapeId) == false {
		Array_Resize(context_.pairKeys, startCount)
	}
}

@(private = "file")
FlushCandidatePairs :: proc(context_: ^PairContext) {
	world := context_.world
	bp := &world.broadPhase

	count1 := context_.batchCount
	context_.batchCount = 0

	plain: [CANDIDATE_BATCH]CandidatePair = ---
	plainCount: i32 = 0

	if context_.checkCompounds {
		// A compound expands into one pair per child, so its key is not known yet and it cannot
		// ride the batched cull below.
		shapes := world.shapes.data
		for i: i32 = 0; i < count1; i += 1 {
			candidate := context_.batch[i]
			compoundA := shapes[candidate.shapeIdA].type == .compoundShape
			compoundB := shapes[candidate.shapeIdB].type == .compoundShape

			if compoundA {
				// Compound versus compound is not supported
				assert(compoundB == false)
				EmitCompoundPairs(context_, candidate.shapeIdA, candidate.shapeIdB)
			} else if compoundB {
				EmitCompoundPairs(context_, candidate.shapeIdB, candidate.shapeIdA)
			} else {
				plain[plainCount] = candidate
				plainCount += 1
			}
		}
	} else {
		mem.copy(&plain, &context_.batch, int(count1) * size_of(CandidatePair))
		plainCount = count1
	}

	// Prefetch hash set entries. Less than 1% gain in Box2D but the 3D shape is fatter.
	keys: [CANDIDATE_BATCH]u64 = ---
	hashes: [CANDIDATE_BATCH]u32 = ---
	for i: i32 = 0; i < plainCount; i += 1 {
		keys[i] = ShapePairKey(plain[i].shapeIdA, plain[i].shapeIdB, 0)
		hashes[i] = KeyHash(keys[i])
		PrefetchHash(&bp.pairSet, hashes[i])
	}

	// Cull existing pairs.
	candidates: [CANDIDATE_BATCH]CandidatePair = ---
	count2: i32 = 0
	for i: i32 = 0; i < plainCount; i += 1 {
		if ContainsHashedKey(&bp.pairSet, keys[i], hashes[i]) == false {
			candidates[count2] = plain[i]
			keys[count2] = keys[i]
			count2 += 1
		}
	}

	// Prefetch shapes.
	shapes := world.shapes.data
	for i: i32 = 0; i < count2; i += 1 {
		Prefetch(&shapes[candidates[i].shapeIdA])
		Prefetch(&shapes[candidates[i].shapeIdB])
	}

	// Filter candidates.
	for i: i32 = 0; i < count2; i += 1 {
		if ShouldCreatePair(world, candidates[i].shapeIdA, candidates[i].shapeIdB) {
			// The pair passed the gauntlet. A new contact will be created.
			Array_Push(context_.pairKeys, keys[i])
		}
	}
}

@(private = "file")
AddCandidatePair :: #force_inline proc(shapeIdA: i32, shapeIdB: i32, context_: ^PairContext) {
	// Follow shape index order.
	candidate := &context_.batch[context_.batchCount]
	candidate.shapeIdA = MinInt(shapeIdA, shapeIdB)
	candidate.shapeIdB = MaxInt(shapeIdA, shapeIdB)
	context_.batchCount += 1
	if context_.batchCount == CANDIDATE_BATCH {
		FlushCandidatePairs(context_)
	}
}

// Did either move and if so do they overlap?
@(private = "file")
TestPair :: #force_inline proc(a: ^TreeNode, b: ^TreeNode) -> bool {
	if ((a.flagIndex | b.flagIndex) & MOVED_NODE) == 0 {
		return false
	}

	return OverlapV(&a.aabb, &b.aabb)
}

@(private = "file")
CollideProxyAndSubtree :: proc(proxy: ^TreeNode, nodes: [^]TreeNode, pair: i32, context_: ^PairContext) {
	pair := pair
	proxyMark := proxy.flagIndex & MOVED_NODE
	boxv := LoadAABBV(&proxy.aabb)
	shapeId := proxy.shapeIndex

	stack: [TREE_STACK_SIZE]i32 = ---
	stackCount: i32 = 0
	stack[stackCount] = pair
	stackCount += 1

	for stackCount > 0 {
		stackCount -= 1
		pair = stack[stackCount]
		for i: i32 = 0; i < 2; i += 1 {
			node := &nodes[pair + i]
			if ((node.flagIndex | proxyMark) & MOVED_NODE) == 0 {
				continue
			}

			if OverlapNode(boxv, node) == false {
				continue
			}

			if IsLeaf(node) {
				AddCandidatePair(shapeId, node.shapeIndex, context_)
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
}

// Helper for b3CollideCrossPairs to avoid code duplication.
@(private = "file")
VisitPair :: #force_inline proc(
	arrayA: [^]TreeNode,
	arrayB: [^]TreeNode,
	nodeA: ^TreeNode,
	nodeB: ^TreeNode,
	stack: [^]IndexPair,
	stackCount: ^i32,
	context_: ^PairContext,
) {
	if TestPair(nodeA, nodeB) == false {
		return
	}

	leafA := IsLeaf(nodeA)
	leafB := IsLeaf(nodeB)
	if leafA && leafB {
		AddCandidatePair(nodeA.shapeIndex, nodeB.shapeIndex, context_)
	} else if leafA {
		CollideProxyAndSubtree(nodeA, arrayB, GetLeftChild(nodeB), context_)
	} else if leafB {
		CollideProxyAndSubtree(nodeB, arrayA, GetLeftChild(nodeA), context_)
	} else {
		if stackCount^ < TREE_STACK_SIZE {
			stack[stackCount^] = IndexPair {
				a = GetLeftChild(nodeA),
				b = GetLeftChild(nodeB),
			}
			stackCount^ += 1
		} else {
			assert(stackCount^ < TREE_STACK_SIZE)
		}
	}
}

// This collides two sub-trees against each other. They can live in the same dynamic tree.
// This can only generate pairs cross sub-tree, but not within a sub-tree. This fact means
// this does not generate duplicate pairs.
// For example consider the full binary tree A (B (D  E) C (F G))
// Colliding children of A (B and C) can give pairs (D,F) (D,G) (E,F) and (E,G).
// Then colliding children of B can give the pair (D,E) and for C (F,G).
// So no duplicates even when used for self-collision.
// Whenever a proxy is moved, the flag is propagated up the hierarchy to the root. So
// self collision gathers all those moved internal nodes and collides their subtrees together.
// See Real-time collision detection section 6.3.2. This is faster than querying every moved
// proxy against the whole tree. Scaling is linear instead of linear * log.
// Many other physics engines do this (Bepu, Rapier, etc). So nothing new here.
@(private = "file")
CollideCrossPairs :: proc(
	arrayA: [^]TreeNode,
	arrayB: [^]TreeNode,
	subtreeA: ^TreeNode,
	subtreeB: ^TreeNode,
	context_: ^PairContext,
) {
	stack: [TREE_STACK_SIZE]IndexPair = ---
	stackCount: i32 = 0

	// Seed the stack.
	VisitPair(arrayA, arrayB, subtreeA, subtreeB, raw_data(&stack), &stackCount, context_)

	for stackCount > 0 {
		stackCount -= 1
		pair := stack[stackCount]
		for i: i32 = 0; i < 2; i += 1 {
			for j: i32 = 0; j < 2; j += 1 {
				VisitPair(
					arrayA,
					arrayB,
					&arrayA[pair.a + i],
					&arrayB[pair.b + j],
					raw_data(&stack),
					&stackCount,
					context_,
				)
			}
		}
	}
}

// This takes moved internal nodes and collides their sub-trees against each other.
@(private = "file")
SelfPairsTask :: proc(startIndex: i32, endIndex: i32, workerIndex: i32, context_: rawptr) {
	world := (^World)(context_)
	bp := &world.broadPhase
	tree := &bp.trees[BodyType.dynamicBody]
	nodes := tree.nodes
	siblingIndices := bp.movedSiblings

	pairContext := PairContext {
		world    = world,
		pairKeys = &world.taskContexts.data[workerIndex].pairKeys,
	}

	for i := startIndex; i < endIndex; i += 1 {
		nodeIndex := siblingIndices[i]
		CollideCrossPairs(nodes, nodes, &nodes[nodeIndex], &nodes[nodeIndex + 1], &pairContext)
	}

	FlushCandidatePairs(&pairContext)
}

@(private = "file")
CROSS_SEED_COUNT :: 64
#assert((CROSS_SEED_COUNT & (CROSS_SEED_COUNT - 1)) == 0, "must be power of 2")

// This does a serial cross-tree breadth first search until the queue is full. Then it returns
// the queue pairs as seeds for a parallel search.
@(private = "file")
GatherCrossSeeds :: proc(treeA: ^DynamicTree, treeB: ^DynamicTree, seeds: [^]NodePair) -> i32 {
	nodesA := treeA.nodes
	nodesB := treeB.nodes

	// Breadth-first search from the two roots as one pair. An empty tree has a sentinel root that
	// survives nothing.
	queue: [2 * CROSS_SEED_COUNT]NodePair = ---
	mask: i32 = 2 * CROSS_SEED_COUNT - 1
	head: i32 = 0
	tail: i32 = 0

	rootA := &nodesA[ROOT_NODE]
	rootB := &nodesB[ROOT_NODE]
	if TestPair(rootA, rootB) {
		queue[tail & mask] = NodePair {
			a = rootA^,
			b = rootB^,
		}
		tail += 1
	}

	seedCount: i32 = 0
	for head < tail && seedCount + (tail - head) + 3 < CROSS_SEED_COUNT {
		pair := queue[head & mask]
		head += 1

		if IsLeaf(&pair.a) || IsLeaf(&pair.b) {
			seeds[seedCount] = pair
			seedCount += 1
			continue
		}

		a := ([^]TreeNode)(&nodesA[GetLeftChild(&pair.a)])
		b := ([^]TreeNode)(&nodesB[GetLeftChild(&pair.b)])

		// Nodes have two children each, so four combinations.
		for i: i32 = 0; i < 2; i += 1 {
			for j: i32 = 0; j < 2; j += 1 {
				if TestPair(&a[i], &b[j]) {
					queue[tail & mask] = NodePair{a[i], b[j]}
					tail += 1
				}
			}
		}
	}

	for head < tail {
		seeds[seedCount] = queue[head & mask]
		seedCount += 1
		head += 1
	}

	return seedCount
}

CrossContext :: struct {
	world:           ^World,
	seeds:           [^]NodePair,
	staticSeedCount: i32,
	checkCompounds:  bool,
}

@(private = "file")
CrossPairsTask :: proc(startIndex: i32, endIndex: i32, workerIndex: i32, context_: rawptr) {
	crossContext := (^CrossContext)(context_)
	world := crossContext.world
	bp := &world.broadPhase
	staticNodes := bp.trees[BodyType.staticBody].nodes
	kinematicNodes := bp.trees[BodyType.kinematicBody].nodes
	dynamicNodes := bp.trees[BodyType.dynamicBody].nodes

	pairContext := PairContext {
		world    = world,
		pairKeys = &world.taskContexts.data[workerIndex].pairKeys,
	}

	// Static seeds come first, so the two passes are contiguous sub-ranges. The batch is drained
	// between them because only the static pass can meet a compound.
	staticEnd := MinInt(endIndex, crossContext.staticSeedCount)

	pairContext.checkCompounds = crossContext.checkCompounds
	for i := startIndex; i < staticEnd; i += 1 {
		seed := crossContext.seeds[i]
		CollideCrossPairs(dynamicNodes, staticNodes, &seed.a, &seed.b, &pairContext)
	}
	FlushCandidatePairs(&pairContext)

	pairContext.checkCompounds = false
	for i := MaxInt(startIndex, crossContext.staticSeedCount); i < endIndex; i += 1 {
		seed := crossContext.seeds[i]
		CollideCrossPairs(dynamicNodes, kinematicNodes, &seed.a, &seed.b, &pairContext)
	}
	FlushCandidatePairs(&pairContext)
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
UpdateTreesTask :: proc "c" (context_: rawptr) {
	context = runtime.default_context()
	// unified end
	world := (^World)(context_)
	DynamicTree_Rebuild(&world.broadPhase.trees[BodyType.dynamicBody], false)
	DynamicTree_Rebuild(&world.broadPhase.trees[BodyType.kinematicBody], false)
}

// Task that can be done in parallel with the narrow-phase
// - rebuild the collision tree for dynamic and kinematic bodies to keep their query performance good
@(private = "file")
EnqueueTreeUpdate :: proc(world: ^World) {
	if world.taskCount < MAX_TASKS {
		world.userTreeTask = world.enqueueTaskFcn(UpdateTreesTask, world, world.userTaskContext, "rebuild tree")
		world.taskCount += 1
		world.activeTaskCount += world.userTreeTask == nil ? 0 : 1
	} else {
		world.userTreeTask = nil
		UpdateTreesTask(world)
	}
}

// unified: the QSORT LESS/SWAP macros for the pair keys are a less-than proc (QSORT, PORTING.md)
@(private = "file")
PairKeyLess :: proc(a, b: u64) -> bool {
	return a < b
}
// unified end

UpdateBroadPhasePairs :: proc(world: ^World) {
	bp := &world.broadPhase

	needUpdate := HasTreeMoved(&bp.trees[BodyType.staticBody])
	needUpdate = needUpdate || NeedsRebuild(&bp.trees[BodyType.kinematicBody])
	needUpdate = needUpdate || NeedsRebuild(&bp.trees[BodyType.dynamicBody])

	if needUpdate == false {
		return
	}

	alloc := &world.stack

	for i: i32 = 0; i < world.workerCount; i += 1 {
		Array_Clear(&world.taskContexts.data[i].pairKeys)
	}

	// Generate pairs by querying the dynamic body tree against itself and against
	// the kinematic and static trees.
	{
		// Get the sibling pairs of the dynamic body tree that have moved.
		dynamicTree := &bp.trees[BodyType.dynamicBody]
		pairCapacity := MaxInt(dynamicTree.nodeEnd / 2, 1)
		bp.movedSiblings = cast([^]i32)StackAlloc(alloc, pairCapacity * size_of(i32), "moved pairs")
		dynamicMoveCount := GatherMovedSiblings(dynamicTree, bp.movedSiblings)

		// Get seeds for colliding against the static and kinematic trees.
		crossSeeds: [2 * CROSS_SEED_COUNT]NodePair = ---
		staticSeedCount := GatherCrossSeeds(dynamicTree, &bp.trees[BodyType.staticBody], raw_data(&crossSeeds))
		assert(staticSeedCount <= CROSS_SEED_COUNT)
		kinematicSeedCount := GatherCrossSeeds(
			dynamicTree,
			&bp.trees[BodyType.kinematicBody],
			([^]NodePair)(&crossSeeds[staticSeedCount]),
		)
		assert(kinematicSeedCount <= CROSS_SEED_COUNT)
		crossMoveCount := staticSeedCount + kinematicSeedCount

		// Collide the dynamic body tree against the static and kinematic trees.
		crossContext := CrossContext {
			world           = world,
			seeds           = raw_data(&crossSeeds),
			staticSeedCount = staticSeedCount,
			checkCompounds  = world.compoundShapeCount > 0,
		}
		ParallelFor(world, CrossPairsTask, crossMoveCount, 1, &crossContext, "cross pairs")

		// Collide the dynamic body tree against itself.
		ParallelFor(world, SelfPairsTask, dynamicMoveCount, 64, world, "self pairs")
	}

	DynamicTree_ClearMoved(&bp.trees[BodyType.staticBody])

	// Update stale trees.
	EnqueueTreeUpdate(world)

	// Pairs arrive in deterministic order but scrambled relative to body and shape order.
	// Sorting them here improves solver performance and makes contact order independent of
	// tree structure.
	pairCount: i32 = 0
	for i: i32 = 0; i < world.workerCount; i += 1 {
		pairCount += world.taskContexts.data[i].pairKeys.count
	}

	pairKeys := cast([^]u64)StackAlloc(alloc, MaxInt(pairCount, 1) * size_of(u64), "pair keys")
	keyCount: i32 = 0
	for i: i32 = 0; i < world.workerCount; i += 1 {
		workerKeys := &world.taskContexts.data[i].pairKeys
		if workerKeys.count > 0 {
			mem.copy(&pairKeys[keyCount], workerKeys.data, int(workerKeys.count) * size_of(u64))
			keyCount += workerKeys.count
		}
	}

	assert(keyCount == pairCount)

	{
		QSORT(pairKeys, pairCount, PairKeyLess)
	}

	for i: i32 = 0; i < keyCount; i += 1 {
		key := pairKeys[i]

		// The traversal is cross subtree and the seeds are disjoint, so a duplicate key means a
		// real bug. Trap it in a validation build, and skip rather than double add to the pair
		// set, which a later destroy would then remove once.
		duplicate := i > 0 && key == pairKeys[i - 1]
		VALIDATE(duplicate == false)
		if duplicate {
			continue
		}

		shapeIdA := i32((key >> (64 - SHAPE_POWER)) & SHAPE_MASK)
		shapeIdB := i32((key >> (64 - 2 * SHAPE_POWER)) & SHAPE_MASK)
		childIndex := i32(key & CHILD_MASK)

		shapeA := Array_Get(&world.shapes, shapeIdA)
		shapeB := Array_Get(&world.shapes, shapeIdB)

		CreateContact(world, shapeA, shapeB, childIndex)
	}

	StackFree(alloc, pairKeys)

	StackFree(alloc, bp.movedSiblings)
	bp.movedSiblings = nil

	ValidateSolverSets(world)
}

BroadPhase_TestOverlap :: proc(bp: ^BroadPhase, proxyKeyA: i32, proxyKeyB: i32) -> bool {
	typeIndexA := PROXY_TYPE(proxyKeyA)
	proxyIdA := PROXY_ID(proxyKeyA)
	typeIndexB := PROXY_TYPE(proxyKeyB)
	proxyIdB := PROXY_ID(proxyKeyB)

	aabbA := DynamicTree_GetAABB(&bp.trees[typeIndexA], proxyIdA)
	aabbB := DynamicTree_GetAABB(&bp.trees[typeIndexB], proxyIdB)
	return AABB_Overlaps(aabbA, aabbB)
}

BroadPhase_GetShapeIndex :: proc(bp: ^BroadPhase, proxyKey: i32) -> i32 {
	typeIndex := PROXY_TYPE(proxyKey)
	proxyId := PROXY_ID(proxyKey)

	return i32(DynamicTree_GetUserData(&bp.trees[typeIndex], proxyId))
}

ValidateBroadPhase :: proc(bp: ^BroadPhase) {
	DynamicTree_Validate(&bp.trees[BodyType.dynamicBody])
	DynamicTree_Validate(&bp.trees[BodyType.kinematicBody])

	// todo validate every shape AABB is contained in tree AABB
}

ValidateNoMoved :: proc(bp: ^BroadPhase) {
	when ENABLE_VALIDATION {
		for j: i32 = 0; j < i32(BodyType.bodyTypeCount); j += 1 {
			tree := &bp.trees[j]
			DynamicTree_ValidateNoMoved(tree)
		}
	} else {
		_ = bp
	}
}
