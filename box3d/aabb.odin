// Ported from box3d src/aabb.c and src/aabb.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

// Get the surface area (perimeter)
Perimeter :: proc "contextless" (a: AABB) -> f32 {
	wx := a.upperBound.x - a.lowerBound.x
	wy := a.upperBound.y - a.lowerBound.y
	wz := a.upperBound.z - a.lowerBound.z
	return 2.0 * (wx * wz + wy * wx + wz * wy)
}

/// Enlarge a to contain b
/// @return true if the AABB grew
EnlargeAABB :: proc "contextless" (a: ^AABB, b: AABB) -> bool {
	changed := false
	if b.lowerBound.x < a.lowerBound.x {
		a.lowerBound.x = b.lowerBound.x
		changed = true
	}

	if b.lowerBound.y < a.lowerBound.y {
		a.lowerBound.y = b.lowerBound.y
		changed = true
	}

	if b.lowerBound.z < a.lowerBound.z {
		a.lowerBound.z = b.lowerBound.z
		changed = true
	}

	if a.upperBound.x < b.upperBound.x {
		a.upperBound.x = b.upperBound.x
		changed = true
	}

	if a.upperBound.y < b.upperBound.y {
		a.upperBound.y = b.upperBound.y
		changed = true
	}

	if a.upperBound.z < b.upperBound.z {
		a.upperBound.z = b.upperBound.z
		changed = true
	}

	return changed
}

// unified: not ported: b3OverlapAABBs (behind `#if 0` upstream)

FarthestPointOnAABB :: proc "contextless" (b: AABB, p: Vec3) -> Vec3 {
	return Vec3 {
		x = (p.x - b.lowerBound.x) > (b.upperBound.x - p.x) ? b.lowerBound.x : b.upperBound.x,
		y = (p.y - b.lowerBound.y) > (b.upperBound.y - p.y) ? b.lowerBound.y : b.upperBound.y,
		z = (p.z - b.lowerBound.z) > (b.upperBound.z - p.z) ? b.lowerBound.z : b.upperBound.z,
	}
}

// Similar to Real-time Collision Detection, p179.
// todo try
// https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection.html
// Ray cast an AABB. This is a custom function used by height fields.
RayCastAABB :: proc(a: AABB, p1: Vec3, p2: Vec3, minFraction: ^f32, maxFraction: ^f32) -> bool {
	FLT_EPSILON :: f32(1.1920929e-7)

	// Ray direction and length
	d := Sub(p2, p1)
	rayLength := Length(d)

	// Handle degenerate ray
	if rayLength < FLT_EPSILON {
		// Check if point is inside AABB
		if p1.x >= a.lowerBound.x &&
		   p1.x <= a.upperBound.x &&
		   p1.y >= a.lowerBound.y &&
		   p1.y <= a.upperBound.y &&
		   p1.z >= a.lowerBound.z &&
		   p1.z <= a.upperBound.z {
			minFraction^ = 0.0
			maxFraction^ = 0.0
			return true
		}

		return false
	}

	rayDir := MulSV(1.0 / rayLength, d)

	// Slab method for ray-AABB intersection
	tMin: f32 = 0.0
	tMax := rayLength

	// x-axis
	{
		rayComponent := rayDir.x
		rayStart := p1.x
		boxMin := a.lowerBound.x
		boxMax := a.upperBound.x

		if AbsFloat(rayComponent) < FLT_EPSILON {
			// Ray is parallel to slab, check if ray origin is within slab
			if rayStart < boxMin || rayStart > boxMax {
				return false
			}
		} else {
			// Compute intersection distances
			t1 := (boxMin - rayStart) / rayComponent
			t2 := (boxMax - rayStart) / rayComponent

			// Ensure t1 <= t2
			if t1 > t2 {
				temp := t1
				t1 = t2
				t2 = temp
			}

			// Update intersection interval
			tMin = MaxFloat(tMin, t1)
			tMax = MinFloat(tMax, t2)

			// Check for no intersection
			if tMin > tMax {
				return false
			}
		}
	}

	// y-axis
	{
		rayComponent := rayDir.y
		rayStart := p1.y
		boxMin := a.lowerBound.y
		boxMax := a.upperBound.y

		if AbsFloat(rayComponent) < FLT_EPSILON {
			// Ray is parallel to slab, check if ray origin is within slab
			if rayStart < boxMin || rayStart > boxMax {
				return false
			}
		} else {
			// Compute intersection distances
			t1 := (boxMin - rayStart) / rayComponent
			t2 := (boxMax - rayStart) / rayComponent

			// Ensure t1 <= t2
			if t1 > t2 {
				temp := t1
				t1 = t2
				t2 = temp
			}

			// Update intersection interval
			tMin = MaxFloat(tMin, t1)
			tMax = MinFloat(tMax, t2)

			// Check for no intersection
			if tMin > tMax {
				return false
			}
		}
	}

	// z-axis
	{
		rayComponent := rayDir.z
		rayStart := p1.z
		boxMin := a.lowerBound.z
		boxMax := a.upperBound.z

		if AbsFloat(rayComponent) < FLT_EPSILON {
			// Ray is parallel to slab, check if ray origin is within slab
			if rayStart < boxMin || rayStart > boxMax {
				return false
			}
		} else {
			// Compute intersection distances
			t1 := (boxMin - rayStart) / rayComponent
			t2 := (boxMax - rayStart) / rayComponent

			// Ensure t1 <= t2
			if t1 > t2 {
				temp := t1
				t1 = t2
				t2 = temp
			}

			// Update intersection interval
			tMin = MaxFloat(tMin, t1)
			tMax = MinFloat(tMax, t2)

			// Check for no intersection
			if tMin > tMax {
				return false
			}
		}
	}

	// Check if intersection is behind ray start
	if tMax < 0.0 {
		return false
	}

	// Convert distances to fractions
	minFraction^ = ClampFloat(tMin / rayLength, 0.0, 1.0)
	maxFraction^ = ClampFloat(tMax / rayLength, 0.0, 1.0)

	return true
}
