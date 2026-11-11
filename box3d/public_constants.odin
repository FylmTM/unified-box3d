// Ported from box3d include/box3d/constants.h at 3362633 (PORTING.md).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
package box3d

import "base:runtime"

// unified: the macros that scale with b3GetLengthUnitsPerMeter() (B3_HUGE, B3_LINEAR_SLOP and those built on
// them) can't be Odin constants, and procs would change every use site. They are package variables, set by
// UpdateLengthUnitConstants (end of file) from the C expressions in the C order, so the f32 values match.

// Used to detect bad values. In float mode positions greater than about 16km have precision
// problems, so 100km is a safe limit. Large world mode keeps coordinates accurate much farther
// from the origin, so the sanity limit widens to keep valid far-field positions from tripping it.
// unified: single precision only: 1.0e5f * b3GetLengthUnitsPerMeter()
HUGE: f32

// Maximum parallel workers. Used for some fixed size arrays.
MAX_WORKERS :: 32

// Maximum number of tasks queued per world step. b3EnqueueTaskCallback will never be called
// more than this per world step. This is related to B3_MAX_WORKERS. With 32 workers,
// the maximum observed task count is 130. This allows an external task system to use a fixed
// size array for Box3D task, which may help with creating stable user task pointers.
MAX_TASKS :: 256

// Maximum number of colors in the constraint graph. Constraints that cannot
// find a color are added to the overflow set which are solved single-threaded.
// The compound barrel benchmark has minor overflow with 24 colors
GRAPH_COLOR_COUNT :: 24

// Number of contact point buckets for counting the number of contact points per
// shape contact pair. This is just for reporting and doesn't affect simulation.
CONTACT_MANIFOLD_COUNT_BUCKETS :: 8

// A small length used as a collision and constraint tolerance. Usually it is
// chosen to be numerically significant, but visually insignificant. In meters.
// @warning modifying this can have a significant impact on stability
// (0.005f * b3GetLengthUnitsPerMeter())
LINEAR_SLOP: f32

// The minimum length of a capsules. Very short capsules should be created as spheres
// to avoid numerical problems.
// (B3_LINEAR_SLOP)
MIN_CAPSULE_LENGTH: f32

// Minimum contact point friction weight, lower bound for speculative points. Made small
// enough to be washed away by weights that hit 1.
MIN_FRICTION_WEIGHT :: f32(1e-10)

// The distance between shapes where they are considered overlapped. This is needed
// because GJK may return small positive values for overlapped shapes in degenerate
// configurations.
// (0.1f * B3_LINEAR_SLOP)
OVERLAP_SLOP: f32

// Maximum number of simultaneous worlds that can be allocated
MAX_WORLDS :: #config(B3_MAX_WORLDS, 128)

// The maximum rotation of a body per time step. This limit is very large and is used
// to prevent numerical problems. You shouldn't need to adjust this.
// @warning increasing this to 0.5f * B3_PI or greater will break continuous collision.
MAX_ROTATION :: (0.25 * PI)

// @warning modifying this can have a significant impact on performance and stability
// (4.0f * B3_LINEAR_SLOP)
SPECULATIVE_DISTANCE: f32

// The rest offset is used for mesh contact to reduce ghost collisions and assist with CCD.
// The rest offset adjusts the contact point separation value, making the solver push the shapes
// apart by this distance.
// Must be at least B3_LINEAR_SLOP and less than B3_SPECULATIVE_DISTANCE.
// (1.0f * B3_LINEAR_SLOP)
MESH_REST_OFFSET: f32

// The default contact recycling distance.
// (10.0f * B3_LINEAR_SLOP)
CONTACT_RECYCLE_DISTANCE: f32

// The default contact recycling world angle threshold. For performance this value
// is cos(angle/2)^2. This value corresponds to 10 degrees.
CONTACT_RECYCLE_ANGULAR_DISTANCE :: f32(0.99240388)

// This is used to fatten AABBs in the dynamic tree. This allows proxies
// to move by a small amount without triggering a tree adjustment. This is in meters.
// @warning modifying this can have a significant impact on performance
// (0.05f * b3GetLengthUnitsPerMeter())
MAX_AABB_MARGIN: f32

// Per-shape AABB margin is a fraction of the shape extent (capped by B3_MAX_AABB_MARGIN).
// Small shapes get small margins; large shapes are clamped to the cap.
AABB_MARGIN_FRACTION :: f32(0.125)

// The time that a body must be still before it will go to sleep. In seconds.
TIME_TO_SLEEP :: f32(0.5)

// The maximum number of restitution iterations. Needed to avoid a solver overflow.
MAX_RESTITUTION_ITERATIONS :: 63

// The maximum number of contact points between two touching shapes.
// The default and minimum is 4 and this case uses a fast approximate hull to reduce the
// point count. A larger value builds a 2D convex hull of the candidate points and then
// simplifies it to the target by dropping the vertices that contribute the least area.
// Useful for testing, but I don't recommend shipping with it adjusted.
// WARNING: if you change this you risk breaking ABI and a corrupted runtime.
MAX_MANIFOLD_POINTS :: #config(B3_MAX_MANIFOLD_POINTS, 4)

// The number of iterations for gyroscopic torques.
GYROSCOPIC_ITERATIONS :: #config(B3_GYROSCOPIC_ITERATIONS, 1)

// The maximum number of convex hull vertices. This is fixed for performance reasons.
MAX_HULL_VERTICES :: 128

// The maximum number of convex hull faces.
MAX_HULL_FACES :: 128

// The maximum number of convex hull edges. Full edges, not half-edges.
MAX_HULL_EDGES :: 128

// Relative tolerance used to determine if two edges are parallel.
PARALLEL_EDGE_TOL :: f32(0.005)

// The maximum number points to use for shape cast proxies (swept point cloud).
MAX_SHAPE_CAST_POINTS :: MAX_HULL_VERTICES

// These generous limits allow for easy hashing. See b3ShapePairKey.
SHAPE_POWER :: 22
CHILD_POWER :: (64 - 2 * SHAPE_POWER)
MAX_SHAPES :: (1 << SHAPE_POWER)
MAX_CHILD_SHAPES :: (1 << CHILD_POWER)

// This is the limit on how many mesh or heightfield triangles a single convex shape can collide with.
// Increasing this will increase stack usage, so be careful. I recommend to simplify your collision data
// before increasing this. For example, using render mesh for collision often leads to poor performance.
MAX_MESH_CONTACT_TRIANGLES :: #config(B3_MAX_MESH_CONTACT_TRIANGLES, 512)

// unified: the length-scaled constants above, computed at startup and again by SetLengthUnitsPerMeter.
@(init)
InitLengthUnitConstants :: proc "contextless" () {
	context = runtime.default_context()
	UpdateLengthUnitConstants()
}

UpdateLengthUnitConstants :: proc() {
	lengthUnits := GetLengthUnitsPerMeter()
	HUGE = 1.0e5 * lengthUnits
	LINEAR_SLOP = 0.005 * lengthUnits
	MIN_CAPSULE_LENGTH = LINEAR_SLOP
	OVERLAP_SLOP = 0.1 * LINEAR_SLOP
	SPECULATIVE_DISTANCE = 4.0 * LINEAR_SLOP
	MESH_REST_OFFSET = 1.0 * LINEAR_SLOP
	CONTACT_RECYCLE_DISTANCE = 10.0 * LINEAR_SLOP
	MAX_AABB_MARGIN = 0.05 * lengthUnits
}
// unified end
