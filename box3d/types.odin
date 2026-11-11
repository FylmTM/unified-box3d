// Ported from box3d src/types.c at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

DefaultWorldDef :: proc() -> WorldDef {
	lengthUnits := GetLengthUnitsPerMeter()

	def := WorldDef{}
	def.gravity.x = 0.0
	def.gravity.y = -10.0
	def.hitEventThreshold = 1.0 * lengthUnits
	def.restitutionThreshold = 1.0 * lengthUnits
	def.restitutionIterations = 2
	def.enableRestitutionPropagation = false
	def.contactSpeed = 3.0 * lengthUnits
	def.contactHertz = 30.0
	def.contactDampingRatio = 10.0

	// 400 meters per second, faster than the speed of sound
	def.maximumLinearSpeed = 400.0 * lengthUnits

	def.isotropicInertiaTolerance = 0.2

	def.enableSleep = true
	def.enableContinuous = true
	def.internalValue = SECRET_COOKIE
	return def
}

DefaultBodyDef :: proc() -> BodyDef {
	def := BodyDef{}
	def.type = .staticBody
	def.rotation = Quat_identity
	def.sleepThreshold = 0.05 * GetLengthUnitsPerMeter()
	def.safetyFactor = 0.5
	def.gravityScale = 1.0
	def.enableSleep = true
	def.isAwake = true
	def.isEnabled = true
	def.enableContactRecycling = true
	def.internalValue = SECRET_COOKIE
	return def
}

DefaultFilter :: proc() -> Filter {
	filter := Filter{DEFAULT_CATEGORY_BITS, DEFAULT_MASK_BITS, 0}
	return filter
}

DefaultQueryFilter :: proc() -> QueryFilter {
	filter := QueryFilter{DEFAULT_CATEGORY_BITS, DEFAULT_MASK_BITS, 0, nil}
	return filter
}

DefaultSurfaceMaterial :: proc() -> SurfaceMaterial {
	surfaceMaterial := SurfaceMaterial{}
	surfaceMaterial.friction = 0.6
	return surfaceMaterial
}

DefaultShapeDef :: proc() -> ShapeDef {
	lengthUnits := GetLengthUnitsPerMeter()

	def := ShapeDef{}
	def.baseMaterial = DefaultSurfaceMaterial()
	// density of water
	def.density = 1000.0 / (lengthUnits * lengthUnits * lengthUnits)
	def.explosionScale = 1.0
	def.filter = DefaultFilter()
	def.updateBodyMass = true
	def.invokeContactCreation = true
	def.enableSpeculativeContact = true
	def.internalValue = SECRET_COOKIE
	return def
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
EmptyDrawShape :: proc "c" (userShape: rawptr, transform: WorldTransform, color: HexColor, context_: rawptr) {
	// unified end
	_, _, _, _ = userShape, transform, color, context_
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
EmptyDrawSegment :: proc "c" (p1: Pos, p2: Pos, color: HexColor, context_: rawptr) {
	// unified end
	_, _, _, _ = p1, p2, color, context_
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
EmptyDrawTransform :: proc "c" (transform: WorldTransform, context_: rawptr) {
	// unified end
	_, _ = transform, context_
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
EmptyDrawPoint :: proc "c" (p: Pos, size: f32, color: HexColor, context_: rawptr) {
	// unified end
	_, _, _, _ = p, size, color, context_
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
EmptyDrawSphere :: proc "c" (p: Pos, radius: f32, color: HexColor, alpha: f32, context_: rawptr) {
	// unified end
	_, _, _, _, _ = p, radius, color, alpha, context_
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
EmptyDrawCapsule :: proc "c" (p1: Pos, p2: Pos, radius: f32, color: HexColor, alpha: f32, context_: rawptr) {
	// unified end
	_, _, _, _, _, _ = p1, p2, radius, color, alpha, context_
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
EmptyDrawBounds :: proc "c" (aabb: AABB, color: HexColor, context_: rawptr) {
	// unified end
	_, _, _ = aabb, color, context_
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
EmptyDrawBox :: proc "c" (extents: Vec3, transform: WorldTransform, color: HexColor, context_: rawptr) {
	// unified end
	_, _, _, _ = extents, transform, color, context_
}

@(private = "file")
// unified: C calling convention (the library's API is upstream's C ABI)
EmptyDrawString :: proc "c" (p: Pos, s: cstring, color: HexColor, context_: rawptr) {
	// unified end
	_, _, _, _ = p, s, color, context_
}

DefaultDebugDraw :: proc() -> DebugDraw {
	draw := DebugDraw{}

	// These allow the user to skip some implementations and not hit null exceptions.
	draw.DrawShapeFcn = EmptyDrawShape
	draw.DrawSegmentFcn = EmptyDrawSegment
	draw.DrawTransformFcn = EmptyDrawTransform
	draw.DrawPointFcn = EmptyDrawPoint
	draw.DrawSphereFcn = EmptyDrawSphere
	draw.DrawCapsuleFcn = EmptyDrawCapsule
	draw.DrawBoundsFcn = EmptyDrawBounds
	draw.DrawBoxFcn = EmptyDrawBox
	draw.DrawStringFcn = EmptyDrawString

	// Not too small, not too big.
	h := 100.0 * GetLengthUnitsPerMeter()
	draw.drawingBounds = AABB {
		lowerBound = {-h, -h, -h},
		upperBound = {h, h, h},
	}

	draw.jointScale = 1.0
	draw.forceScale = 0.001

	return draw
}
