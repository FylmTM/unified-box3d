// The comparison program (`./b3 compare`): a set of scenes stepped through Box3D's public C API, printing one line
// per tick with hashes of everything observable. Built twice from this one file and the same headers: linked
// statically against the original upstream library, and dynamically against ours. The traces must be identical.
//
//   compare <workers> [--w4] [scene]  trace every scene (or one) with that many workers (1: serial; more: the
//                                     library's built-in scheduler); --w4: the 4-wide SIMD path even where the
//                                     CPU has AVX2 (b3World_EnableSSE2Fallback), which must give the same trace
//   compare --list                the scene names
//
// A trace line: `<scene> <tick> s=<state> e=<events> q=<queries> c=<counters> <counts>`. The hashes are 64-bit
// FNV-1a over raw bits: `state` over every tracked body's id, position, rotation, linear and angular velocity and
// awake and enabled flags (in creation order) and every joint's constraint force, torque and separations; `events`
// over contact begin, end and hit events, sensor begin and end events, body move events and joint events (ids and
// payloads); `queries` over the scene's ray casts, shape casts, overlaps, mover queries and sensor contents;
// `counters` over the world's body, shape, contact, joint and island counts, tree heights, graph colors and awake
// counts. Tick 0 is the scene as built, with `e=` hashing what it was built from (mass data, AABBs, extents). The
// counts after the hashes are for reading a diff; NON-FINITE flags a state with an inf or NaN.
//
// Avoids what the port changes on purpose (SPEC.md, Customizations): no motion locks (#87 zeroes the locked
// velocity components after the relax iterations, upstream doesn't), and no sensor overlaps that tie on shape id.
// Calls only what upstream's debug build accepts (its release build compiles B3_ASSERT out; ours keeps asserts).
//
// Uses only the public headers; no libm (the trigonometry is Box3D's own), so the program itself is the same on
// every platform. Geometry and positions come from an integer LCG.

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_BODIES 2048
#define MAX_JOINTS 512
#define MAX_SHAPES 64
#define MAX_GEOMETRY 64

// --- hashing --------------------------------------------------------------------------------------------------------

#define FNV_OFFSET 0xcbf29ce484222325ull
#define FNV_PRIME 0x100000001b3ull

static void HashBytes( uint64_t* h, const void* data, size_t n )
{
	const uint8_t* p = (const uint8_t*)data;
	for ( size_t i = 0; i < n; ++i )
	{
		*h = ( *h ^ p[i] ) * FNV_PRIME;
	}
}

static void HashU32( uint64_t* h, uint32_t x )
{
	HashBytes( h, &x, sizeof( x ) );
}

static void HashInt( uint64_t* h, int x )
{
	HashU32( h, (uint32_t)x );
}

static void HashU64( uint64_t* h, uint64_t x )
{
	HashBytes( h, &x, sizeof( x ) );
}

static int g_nonFinite; // non-finite floats seen in the state (a scene that blew up), shown in the trace

static void HashFloat( uint64_t* h, float f )
{
	uint32_t u;
	memcpy( &u, &f, sizeof( u ) );
	if ( ( u & 0x7f800000u ) == 0x7f800000u )
	{
		g_nonFinite += 1;
	}
	HashU32( h, u );
}

static void HashVec( uint64_t* h, b3Vec3 v )
{
	HashFloat( h, v.x );
	HashFloat( h, v.y );
	HashFloat( h, v.z );
}

static void HashQuat( uint64_t* h, b3Quat q )
{
	HashVec( h, q.v );
	HashFloat( h, q.s );
}

static void HashShapeId( uint64_t* h, b3ShapeId id )
{
	HashInt( h, id.index1 );
	HashU32( h, id.generation );
}

static void HashBodyId( uint64_t* h, b3BodyId id )
{
	HashInt( h, id.index1 );
	HashU32( h, id.generation );
}

static void HashJointId( uint64_t* h, b3JointId id )
{
	HashInt( h, id.index1 );
	HashU32( h, id.generation );
}

static void HashContactId( uint64_t* h, b3ContactId id )
{
	HashInt( h, id.index1 );
	HashU32( h, id.generation );
}

static void HashAABB( uint64_t* h, b3AABB a )
{
	HashVec( h, a.lowerBound );
	HashVec( h, a.upperBound );
}

static void HashMassData( uint64_t* h, b3MassData m )
{
	HashBytes( h, &m, sizeof( m ) ); // floats only: no padding
}

// --- random numbers (integer LCG; floats from 24 bits, exact) ------------------------------------------------------

static uint32_t g_seed;

static float RandFloat( float lo, float hi )
{
	g_seed = g_seed * 1664525u + 1013904223u;
	return lo + ( hi - lo ) * ( (float)( g_seed >> 8 ) * ( 1.0f / 16777216.0f ) );
}

static int RandInt( int n )
{
	g_seed = g_seed * 1664525u + 1013904223u;
	return (int)( ( g_seed >> 8 ) % (uint32_t)n );
}

static b3Quat RandRotation( void )
{
	b3Vec3 axis = { RandFloat( -1.0f, 1.0f ), RandFloat( -1.0f, 1.0f ), RandFloat( -1.0f, 1.0f ) };
	float length = b3Length( axis );
	if ( length < 0.1f )
	{
		return b3Quat_identity;
	}
	axis = b3MulSV( 1.0f / length, axis );
	return b3MakeQuatFromAxisAngle( axis, RandFloat( -3.0f, 3.0f ) );
}

// --- the scene context ------------------------------------------------------------------------------------------------

typedef struct Ctx
{
	b3WorldId world;
	b3BodyId bodies[MAX_BODIES];
	int bodyCount;
	b3JointId joints[MAX_JOINTS];
	int jointCount;
	b3ShapeId sensors[MAX_SHAPES];
	int sensorCount;
	b3ShapeId shapes[MAX_SHAPES]; // shapes the scene queries directly
	int shapeCount;

	// Geometry the scene owns (freed after the world).
	b3HullData* hulls[MAX_GEOMETRY];
	int hullCount;
	b3MeshData* meshes[MAX_GEOMETRY];
	int meshCount;
	b3HeightFieldData* heightFields[MAX_GEOMETRY];
	int heightFieldCount;
	b3CompoundData* compounds[MAX_GEOMETRY];
	int compoundCount;

	uint64_t queryHash;
	int queryCount; // results, for reading a diff
	b3BodyId user[16]; // per-scene handles
	b3JointId userJoints[16];
} Ctx;

static b3BodyId AddBody( Ctx* c, const b3BodyDef* def )
{
	b3BodyId id = b3CreateBody( c->world, def );
	if ( c->bodyCount < MAX_BODIES )
	{
		c->bodies[c->bodyCount++] = id;
	}
	return id;
}

static b3JointId AddJoint( Ctx* c, b3JointId id )
{
	if ( c->jointCount < MAX_JOINTS )
	{
		c->joints[c->jointCount++] = id;
	}
	return id;
}

static b3BodyDef BodyAt( b3BodyType type, float x, float y, float z )
{
	b3BodyDef def = b3DefaultBodyDef();
	def.type = type;
	def.position = (b3Pos){ x, y, z };
	return def;
}

static b3ShapeDef EventShapeDef( void )
{
	b3ShapeDef def = b3DefaultShapeDef();
	def.enableContactEvents = true;
	def.enableHitEvents = true;
	def.enableSensorEvents = true;
	return def;
}

static b3ShapeId AddBox( b3BodyId body, const b3ShapeDef* def, float hx, float hy, float hz )
{
	b3BoxHull box = b3MakeBoxHull( hx, hy, hz );
	return b3CreateHullShape( body, def, &box.base );
}

static b3ShapeId AddSphere( b3BodyId body, const b3ShapeDef* def, float radius )
{
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, radius };
	return b3CreateSphereShape( body, def, &sphere );
}

static b3ShapeId AddCapsule( b3BodyId body, const b3ShapeDef* def, b3Vec3 a, b3Vec3 b, float radius )
{
	b3Capsule capsule = { a, b, radius };
	return b3CreateCapsuleShape( body, def, &capsule );
}

static b3HullData* OwnHull( Ctx* c, b3HullData* hull )
{
	c->hulls[c->hullCount++] = hull;
	return hull;
}

// A static ground box, top at y = 0.
static b3BodyId AddGround( Ctx* c, float halfSize )
{
	b3BodyDef def = BodyAt( b3_staticBody, 0.0f, -1.0f, 0.0f );
	b3BodyId ground = AddBody( c, &def );
	b3ShapeDef sd = EventShapeDef();
	AddBox( ground, &sd, halfSize, 1.0f, halfSize );
	return ground;
}

// One of several convex shapes, chosen by `kind`.
static void AddMixedShape( Ctx* c, b3BodyId body, const b3ShapeDef* def, int kind )
{
	switch ( kind % 6 )
	{
		case 0:
			AddSphere( body, def, RandFloat( 0.2f, 0.5f ) );
			break;
		case 1:
		{
			float h = RandFloat( 0.1f, 0.5f );
			AddCapsule( body, def, (b3Vec3){ -h, 0.0f, 0.0f }, (b3Vec3){ h, 0.0f, 0.0f }, RandFloat( 0.15f, 0.3f ) );
			break;
		}
		case 2:
			AddBox( body, def, RandFloat( 0.2f, 0.5f ), RandFloat( 0.2f, 0.5f ), RandFloat( 0.2f, 0.5f ) );
			break;
		case 3:
			b3CreateHullShape( body, def, c->hulls[0] ); // cylinder
			break;
		case 4:
			b3CreateHullShape( body, def, c->hulls[1] ); // rock
			break;
		default:
		{
			b3Transform xf = { { 0.1f, 0.0f, 0.0f }, RandRotation() };
			b3Vec3 scale = { RandFloat( 0.5f, 1.2f ), RandFloat( 0.5f, 1.2f ), RandFloat( 0.5f, 1.2f ) };
			b3CreateTransformedHullShape( body, def, c->hulls[2], xf, scale ); // cone, scaled
			break;
		}
	}
}

// The shared convex hulls AddMixedShape uses: hulls[0..2].
static void MakeMixedHulls( Ctx* c )
{
	OwnHull( c, b3CreateCylinder( 0.6f, 0.3f, 0.0f, 12 ) );
	OwnHull( c, b3CreateRock( 0.4f ) );
	OwnHull( c, b3CreateCone( 0.6f, 0.35f, 0.1f, 10 ) );
}

static void DropMixed( Ctx* c, int count, b3Vec3 center, b3Vec3 spread, bool randomRotation )
{
	b3ShapeDef sd = EventShapeDef();
	for ( int i = 0; i < count; ++i )
	{
		b3BodyDef bd = BodyAt( b3_dynamicBody, center.x + RandFloat( -spread.x, spread.x ),
							   center.y + RandFloat( -spread.y, spread.y ), center.z + RandFloat( -spread.z, spread.z ) );
		if ( randomRotation )
		{
			bd.rotation = RandRotation();
		}
		b3BodyId body = AddBody( c, &bd );
		sd.baseMaterial.friction = RandFloat( 0.2f, 0.9f );
		sd.baseMaterial.restitution = RandInt( 4 ) == 0 ? RandFloat( 0.0f, 0.6f ) : 0.0f;
		sd.baseMaterial.rollingResistance = RandInt( 4 ) == 0 ? 0.05f : 0.0f;
		sd.density = RandFloat( 500.0f, 2000.0f );
		AddMixedShape( c, body, &sd, i );
	}
}

// --- queries --------------------------------------------------------------------------------------------------------

static float CastAllCallback( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId,
							  int triangleIndex, int childIndex, void* context )
{
	Ctx* c = (Ctx*)context;
	HashShapeId( &c->queryHash, shapeId );
	HashVec( &c->queryHash, point );
	HashVec( &c->queryHash, normal );
	HashFloat( &c->queryHash, fraction );
	HashU64( &c->queryHash, userMaterialId );
	HashInt( &c->queryHash, triangleIndex );
	HashInt( &c->queryHash, childIndex );
	c->queryCount += 1;
	return 1.0f; // every hit
}

static float CastClosestCallback( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId,
								  int triangleIndex, int childIndex, void* context )
{
	CastAllCallback( shapeId, point, normal, fraction, userMaterialId, triangleIndex, childIndex, context );
	return fraction; // clip
}

static bool OverlapCallback( b3ShapeId shapeId, void* context )
{
	Ctx* c = (Ctx*)context;
	HashShapeId( &c->queryHash, shapeId );
	c->queryCount += 1;
	return true;
}

static bool PlaneCallback( b3ShapeId shapeId, const b3PlaneResult* planes, int planeCount, void* context )
{
	Ctx* c = (Ctx*)context;
	HashShapeId( &c->queryHash, shapeId );
	for ( int i = 0; i < planeCount; ++i )
	{
		HashVec( &c->queryHash, planes[i].plane.normal );
		HashFloat( &c->queryHash, planes[i].plane.offset );
		HashVec( &c->queryHash, planes[i].point );
		HashInt( &c->queryHash, planes[i].triangleIndex );
		HashInt( &c->queryHash, planes[i].childIndex );
		HashInt( &c->queryHash, planes[i].materialIndex );
	}
	c->queryCount += 1;
	return true;
}

static void HashStats( Ctx* c, b3TreeStats s )
{
	HashInt( &c->queryHash, s.nodeVisits );
	HashInt( &c->queryHash, s.leafVisits );
}

static void HashRayResult( Ctx* c, b3RayResult r )
{
	HashShapeId( &c->queryHash, r.shapeId );
	HashVec( &c->queryHash, r.point );
	HashVec( &c->queryHash, r.normal );
	HashU64( &c->queryHash, r.userMaterialId );
	HashFloat( &c->queryHash, r.fraction );
	HashInt( &c->queryHash, r.triangleIndex );
	HashInt( &c->queryHash, r.childIndex );
	HashInt( &c->queryHash, r.nodeVisits );
	HashInt( &c->queryHash, r.leafVisits );
	HashInt( &c->queryHash, r.hit );
	c->queryCount += r.hit ? 1 : 0;
}

// The queries every scene runs each tick: a fan of closest-hit rays down onto the scene, and an AABB overlap.
static void BasicQueries( Ctx* c, int tick, float extent )
{
	b3QueryFilter filter = b3DefaultQueryFilter();
	for ( int i = 0; i < 5; ++i )
	{
		float x = -extent + 2.0f * extent * (float)( ( tick + 3 * i ) % 17 ) / 16.0f;
		b3Pos origin = { x, 30.0f, 0.4f * x - 1.0f + 0.5f * (float)i };
		b3RayResult r = b3World_CastRayClosest( c->world, origin, (b3Vec3){ 0.1f, -60.0f, 0.05f }, filter );
		HashRayResult( c, r );
	}
	float t = (float)( tick % 40 ) / 40.0f;
	b3AABB box = { { -extent + t * extent, 0.0f, -2.0f }, { t * extent, 3.0f, 2.0f } };
	HashStats( c, b3World_OverlapAABB( c->world, box, filter, OverlapCallback, c ) );
}

// --- scenes ---------------------------------------------------------------------------------------------------------

typedef struct Scene
{
	const char* name;
	int ticks;
	void ( *create )( Ctx* c );
	void ( *tick )( Ctx* c, int tick ); // before the step; may be NULL
	float queryExtent;
} Scene;

// A box pyramid and a tall stack: stacking, warm starting, islands, sleep.
static void CreatePyramid( Ctx* c )
{
	AddGround( c, 40.0f );
	b3ShapeDef sd = EventShapeDef();
	int base = 12;
	float h = 0.5f;
	for ( int row = 0; row < base; ++row )
	{
		for ( int i = 0; i < base - row; ++i )
		{
			float x = ( 2.0f * i - ( base - row ) + 1.0f ) * ( h + 0.01f );
			b3BodyDef bd = BodyAt( b3_dynamicBody, x, h + 2.0f * h * row, 0.0f );
			b3BodyId body = AddBody( c, &bd );
			AddBox( body, &sd, h, h, h );
		}
	}
	for ( int i = 0; i < 14; ++i )
	{
		b3BodyDef bd = BodyAt( b3_dynamicBody, 12.0f + 0.02f * ( i % 3 ), 0.4f + 0.82f * i, 3.0f );
		b3BodyId body = AddBody( c, &bd );
		AddBox( body, &sd, 0.6f, 0.4f, 0.6f );
	}
}

static void TickPyramid( Ctx* c, int tick )
{
	if ( tick == 150 || tick == 260 )
	{
		// A heavy ball into the sleeping pyramid, then one into the tower.
		b3BodyDef bd = BodyAt( b3_dynamicBody, tick == 150 ? -14.0f : 12.0f, 2.0f, tick == 150 ? 0.0f : -8.0f );
		bd.linearVelocity = tick == 150 ? (b3Vec3){ 30.0f, 2.0f, 0.0f } : (b3Vec3){ 0.0f, 3.0f, 25.0f };
		bd.isBullet = tick == 260;
		b3BodyId ball = AddBody( c, &bd );
		b3ShapeDef sd = EventShapeDef();
		sd.density = 5000.0f;
		AddSphere( ball, &sd, 0.6f );
	}
}

// A pile of mixed convex shapes in a walled bin, with material mixing callbacks.
static float FrictionMix( float frictionA, uint64_t materialA, float frictionB, uint64_t materialB )
{
	( void )materialA;
	( void )materialB;
	return frictionA < frictionB ? frictionA : frictionB;
}

static float RestitutionMix( float restitutionA, uint64_t materialA, float restitutionB, uint64_t materialB )
{
	( void )materialA;
	( void )materialB;
	return restitutionA > restitutionB ? restitutionA : restitutionB;
}

static void AddBin( Ctx* c, float half, float height )
{
	b3BodyDef bd = BodyAt( b3_staticBody, 0.0f, 0.0f, 0.0f );
	b3BodyId bin = AddBody( c, &bd );
	b3ShapeDef sd = EventShapeDef();
	b3BoxHull floor = b3MakeOffsetBoxHull( half + 1.0f, 0.5f, half + 1.0f, (b3Vec3){ 0.0f, -0.5f, 0.0f } );
	b3CreateHullShape( bin, &sd, &floor.base );
	b3BoxHull wall1 = b3MakeOffsetBoxHull( 0.5f, height, half, (b3Vec3){ half + 0.5f, height, 0.0f } );
	b3BoxHull wall2 = b3MakeOffsetBoxHull( 0.5f, height, half, (b3Vec3){ -half - 0.5f, height, 0.0f } );
	b3BoxHull wall3 = b3MakeOffsetBoxHull( half, height, 0.5f, (b3Vec3){ 0.0f, height, half + 0.5f } );
	b3BoxHull wall4 = b3MakeOffsetBoxHull( half, height, 0.5f, (b3Vec3){ 0.0f, height, -half - 0.5f } );
	b3CreateHullShape( bin, &sd, &wall1.base );
	b3CreateHullShape( bin, &sd, &wall2.base );
	b3CreateHullShape( bin, &sd, &wall3.base );
	b3CreateHullShape( bin, &sd, &wall4.base );
}

static void CreatePile( Ctx* c )
{
	b3World_SetFrictionCallback( c->world, FrictionMix );
	b3World_SetRestitutionCallback( c->world, RestitutionMix );
	MakeMixedHulls( c );
	AddBin( c, 4.0f, 2.0f );
	DropMixed( c, 160, (b3Vec3){ 0.0f, 8.0f, 0.0f }, (b3Vec3){ 3.0f, 5.0f, 3.0f }, true );
}

// Chains of every joint type.
static void LinkFrames( b3JointDef* base, b3BodyId a, b3BodyId b, b3Pos anchor )
{
	base->bodyIdA = a;
	base->bodyIdB = b;
	b3Pos pa = b3Body_GetPosition( a );
	b3Pos pb = b3Body_GetPosition( b );
	base->localFrameA = (b3Transform){ b3Sub( anchor, pa ), b3Quat_identity };
	base->localFrameB = (b3Transform){ b3Sub( anchor, pb ), b3Quat_identity };
}

static b3BodyId ChainLink( Ctx* c, float x, float y, float z, int capsule )
{
	b3BodyDef bd = BodyAt( b3_dynamicBody, x, y, z );
	b3BodyId body = AddBody( c, &bd );
	b3ShapeDef sd = EventShapeDef();
	if ( capsule )
	{
		AddCapsule( body, &sd, (b3Vec3){ -0.4f, 0.0f, 0.0f }, (b3Vec3){ 0.4f, 0.0f, 0.0f }, 0.12f );
	}
	else
	{
		AddBox( body, &sd, 0.4f, 0.12f, 0.12f );
	}
	return body;
}

static void CreateJoints( Ctx* c )
{
	b3BodyId ground = AddGround( c, 60.0f );
	float y = 12.0f;
	int links = 6;

	// Revolute: motor, limits, a spring.
	{
		b3BodyId prev = ground;
		for ( int i = 0; i < links; ++i )
		{
			b3BodyId link = ChainLink( c, -30.0f + 0.5f + i, y, 0.0f, 1 );
			b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
			LinkFrames( &def.base, prev, link, (b3Pos){ -30.0f + i, y, 0.0f } );
			def.enableLimit = i % 2 == 1;
			def.lowerAngle = -0.5f;
			def.upperAngle = 0.7f;
			def.enableMotor = i == 0;
			def.motorSpeed = 1.0f;
			def.maxMotorTorque = 200.0f;
			def.enableSpring = i == 2;
			def.hertz = 2.0f;
			def.dampingRatio = 0.3f;
			def.base.forceThreshold = 150.0f;
			def.base.torqueThreshold = 60.0f;
			AddJoint( c, b3CreateRevoluteJoint( c->world, &def ) );
			prev = link;
		}
	}

	// Spherical: cone and twist limits, a spring, a motor.
	{
		b3BodyId prev = ground;
		for ( int i = 0; i < links; ++i )
		{
			b3BodyId link = ChainLink( c, -22.0f + 0.5f + i, y, 0.0f, 1 );
			b3SphericalJointDef def = b3DefaultSphericalJointDef();
			LinkFrames( &def.base, prev, link, (b3Pos){ -22.0f + i, y, 0.0f } );
			def.enableConeLimit = i % 2 == 0;
			def.coneAngle = 0.6f;
			def.enableTwistLimit = i % 3 == 0;
			def.lowerTwistAngle = -0.3f;
			def.upperTwistAngle = 0.3f;
			def.enableSpring = i == 3;
			def.hertz = 1.5f;
			def.dampingRatio = 0.5f;
			def.enableMotor = i == 1;
			def.maxMotorTorque = 50.0f;
			def.motorVelocity = (b3Vec3){ 0.0f, 1.0f, 0.5f };
			AddJoint( c, b3CreateSphericalJoint( c->world, &def ) );
			prev = link;
		}
	}

	// Weld: a soft cantilever and a rigid one.
	for ( int k = 0; k < 2; ++k )
	{
		b3BodyId prev = ground;
		float z = -3.0f * k;
		for ( int i = 0; i < links; ++i )
		{
			b3BodyId link = ChainLink( c, -14.0f + 0.5f + i, y, z, 0 );
			b3WeldJointDef def = b3DefaultWeldJointDef();
			LinkFrames( &def.base, prev, link, (b3Pos){ -14.0f + i, y, z } );
			def.linearHertz = k == 0 ? 5.0f : 0.0f;
			def.angularHertz = k == 0 ? 3.0f : 0.0f;
			def.linearDampingRatio = 0.7f;
			def.angularDampingRatio = 0.7f;
			def.base.torqueThreshold = 30.0f;
			AddJoint( c, b3CreateWeldJoint( c->world, &def ) );
			prev = link;
		}
	}

	// Distance: a rope with a spring, limits and a motor.
	{
		b3BodyId prev = ground;
		for ( int i = 0; i < links; ++i )
		{
			b3BodyDef bd = BodyAt( b3_dynamicBody, -6.0f + i, y, 0.0f );
			b3BodyId link = AddBody( c, &bd );
			b3ShapeDef sd = EventShapeDef();
			AddSphere( link, &sd, 0.2f );
			b3DistanceJointDef def = b3DefaultDistanceJointDef();
			def.base.bodyIdA = prev;
			def.base.bodyIdB = link;
			b3Vec3 anchorA = i == 0 ? b3Sub( (b3Pos){ -7.0f, y, 0.0f }, b3Body_GetPosition( ground ) ) : b3Vec3_zero;
			def.base.localFrameA = (b3Transform){ anchorA, b3Quat_identity };
			def.base.localFrameB = b3Transform_identity;
			def.length = 1.0f;
			def.enableSpring = i % 2 == 0;
			def.hertz = 3.0f;
			def.dampingRatio = 0.2f;
			def.lowerSpringForce = -500.0f;
			def.upperSpringForce = 500.0f;
			def.enableLimit = true;
			def.minLength = 0.6f;
			def.maxLength = 1.4f;
			def.enableMotor = i == 4;
			def.maxMotorForce = 20.0f;
			def.motorSpeed = 0.5f;
			AddJoint( c, b3CreateDistanceJoint( c->world, &def ) );
			prev = link;
		}
	}

	// Prismatic: sliders with motors, limits, springs.
	{
		b3BodyId prev = ground;
		for ( int i = 0; i < 3; ++i )
		{
			b3BodyId link = ChainLink( c, 2.0f + 1.2f * i, y, 0.0f, 0 );
			b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
			LinkFrames( &def.base, prev, link, (b3Pos){ 2.0f + 1.2f * i, y, 0.0f } );
			def.base.localFrameA.q = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.3f * ( i + 1 ) );
			def.base.localFrameB.q = def.base.localFrameA.q;
			def.enableLimit = true;
			def.lowerTranslation = -0.5f;
			def.upperTranslation = 1.0f;
			def.enableMotor = i == 0;
			def.motorSpeed = 0.8f;
			def.maxMotorForce = 500.0f;
			def.enableSpring = i == 1;
			def.hertz = 2.0f;
			def.dampingRatio = 0.5f;
			AddJoint( c, b3CreatePrismaticJoint( c->world, &def ) );
			prev = link;
		}
	}

	// Wheel: a car with suspension, spin motors and steering.
	{
		b3BodyDef cd = BodyAt( b3_dynamicBody, 12.0f, 1.2f, 0.0f );
		b3BodyId chassis = AddBody( c, &cd );
		b3ShapeDef sd = EventShapeDef();
		AddBox( chassis, &sd, 1.5f, 0.3f, 0.8f );
		for ( int i = 0; i < 4; ++i )
		{
			float wx = i < 2 ? 1.1f : -1.1f;
			float wz = i % 2 == 0 ? 1.0f : -1.0f;
			b3BodyDef wd = BodyAt( b3_dynamicBody, 12.0f + wx, 0.45f, wz );
			b3BodyId wheel = AddBody( c, &wd );
			AddSphere( wheel, &sd, 0.4f );
			b3WheelJointDef def = b3DefaultWheelJointDef();
			LinkFrames( &def.base, chassis, wheel, (b3Pos){ 12.0f + wx, 0.45f, wz } );
			def.enableSuspensionSpring = true;
			def.suspensionHertz = 4.0f;
			def.suspensionDampingRatio = 0.7f;
			def.enableSuspensionLimit = true;
			def.lowerSuspensionLimit = -0.2f;
			def.upperSuspensionLimit = 0.2f;
			def.enableSpinMotor = true;
			def.maxSpinTorque = 40.0f;
			def.spinSpeed = 4.0f;
			def.enableSteering = i < 2;
			def.steeringHertz = 5.0f;
			def.steeringDampingRatio = 0.7f;
			def.maxSteeringTorque = 100.0f;
			def.enableSteeringLimit = true;
			def.lowerSteeringLimit = -0.4f;
			def.upperSteeringLimit = 0.4f;
			def.targetSteeringAngle = 0.2f;
			c->userJoints[i] = AddJoint( c, b3CreateWheelJoint( c->world, &def ) );
		}
	}

	// Motor: a box driven by velocity and springs relative to the ground.
	{
		b3BodyDef bd = BodyAt( b3_dynamicBody, 20.0f, 3.0f, 0.0f );
		b3BodyId body = AddBody( c, &bd );
		b3ShapeDef sd = EventShapeDef();
		AddBox( body, &sd, 0.5f, 0.5f, 0.5f );
		b3MotorJointDef def = b3DefaultMotorJointDef();
		LinkFrames( &def.base, ground, body, (b3Pos){ 20.0f, 3.0f, 0.0f } );
		def.linearVelocity = (b3Vec3){ 0.5f, 0.0f, 0.0f };
		def.maxVelocityForce = 200.0f;
		def.angularVelocity = (b3Vec3){ 0.0f, 1.0f, 0.0f };
		def.maxVelocityTorque = 100.0f;
		def.linearHertz = 2.0f;
		def.linearDampingRatio = 0.7f;
		def.maxSpringForce = 300.0f;
		def.angularHertz = 2.0f;
		def.angularDampingRatio = 0.7f;
		def.maxSpringTorque = 100.0f;
		c->userJoints[4] = AddJoint( c, b3CreateMotorJoint( c->world, &def ) );
	}

	// Mover and pogo: a dynamic character capsule held up by a pogo and pushed by a mover.
	{
		b3BodyDef bd = BodyAt( b3_dynamicBody, 24.0f, 1.5f, 0.0f );
		b3BodyId body = AddBody( c, &bd );
		b3ShapeDef sd = EventShapeDef();
		AddCapsule( body, &sd, (b3Vec3){ 0.0f, -0.4f, 0.0f }, (b3Vec3){ 0.0f, 0.4f, 0.0f }, 0.3f );
		b3MoverJointDef md = b3DefaultMoverJointDef();
		LinkFrames( &md.base, ground, body, (b3Pos){ 24.0f, 1.5f, 0.0f } );
		md.linearVelocity = (b3Vec3){ 1.0f, 0.0f, 0.5f };
		md.maxForce = 300.0f;
		c->userJoints[5] = AddJoint( c, b3CreateMoverJoint( c->world, &md ) );
		b3PogoJointDef pd = b3DefaultPogoJointDef();
		LinkFrames( &pd.base, ground, body, (b3Pos){ 24.0f, 0.0f, 0.0f } );
		pd.normal = b3Vec3_axisY;
		pd.hertz = 5.0f;
		pd.dampingRatio = 0.5f;
		pd.restLength = 1.2f;
		pd.maxTensionForce = 100.0f;
		pd.maxCompressionForce = 2000.0f;
		AddJoint( c, b3CreatePogoJoint( c->world, &pd ) );
	}

	// Parallel: keeps a tipping box upright. Filter: two overlapping boxes that don't collide.
	{
		b3BodyDef bd = BodyAt( b3_dynamicBody, 28.0f, 2.0f, 0.0f );
		bd.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.8f );
		b3BodyId body = AddBody( c, &bd );
		b3ShapeDef sd = EventShapeDef();
		AddBox( body, &sd, 0.3f, 1.0f, 0.3f );
		b3ParallelJointDef def = b3DefaultParallelJointDef();
		LinkFrames( &def.base, ground, body, (b3Pos){ 28.0f, 2.0f, 0.0f } );
		def.base.localFrameA.q = b3MakeQuatFromAxisAngle( b3Vec3_axisX, -1.5707964f );
		def.base.localFrameB.q = b3MakeQuatFromAxisAngle( b3Vec3_axisX, -1.5707964f );
		def.hertz = 3.0f;
		def.dampingRatio = 0.5f;
		def.maxTorque = 500.0f;
		AddJoint( c, b3CreateParallelJoint( c->world, &def ) );

		b3BodyDef ad = BodyAt( b3_dynamicBody, 32.0f, 1.0f, 0.0f );
		b3BodyId a = AddBody( c, &ad );
		AddBox( a, &sd, 0.5f, 0.5f, 0.5f );
		b3BodyDef bd2 = BodyAt( b3_dynamicBody, 32.3f, 1.2f, 0.0f );
		b3BodyId b = AddBody( c, &bd2 );
		AddBox( b, &sd, 0.5f, 0.5f, 0.5f );
		b3FilterJointDef fd = b3DefaultFilterJointDef();
		fd.base.bodyIdA = a;
		fd.base.bodyIdB = b;
		AddJoint( c, b3CreateFilterJoint( c->world, &fd ) );
	}

	// Something to hit: a few boxes under the chains.
	b3ShapeDef sd = EventShapeDef();
	for ( int i = 0; i < 10; ++i )
	{
		b3BodyDef bd = BodyAt( b3_dynamicBody, -28.0f + 3.0f * i, 0.5f, 0.0f );
		b3BodyId body = AddBody( c, &bd );
		AddBox( body, &sd, 0.5f, 0.5f, 0.5f );
	}
}

static void TickJoints( Ctx* c, int tick )
{
	if ( tick == 120 )
	{
		for ( int i = 0; i < 2; ++i )
		{
			b3WheelJoint_SetTargetSteeringAngle( c->userJoints[i], -0.3f );
		}
		b3MotorJoint_SetLinearVelocity( c->userJoints[4], (b3Vec3){ -0.5f, 0.2f, 0.0f } );
		b3MoverJoint_SetLinearVelocity( c->userJoints[5], (b3Vec3){ -1.0f, 0.0f, 0.0f } );
	}
}

// Ragdolls: capsule limbs, spherical joints with cone and twist limits at hips, shoulders and neck, revolute
// joints with limits at knees and elbows; falling onto stairs and each other.
static b3BodyId Limb( Ctx* c, b3Pos p, b3Vec3 a, b3Vec3 b, float radius, int group )
{
	b3BodyDef bd = BodyAt( b3_dynamicBody, p.x, p.y, p.z );
	bd.linearVelocity = (b3Vec3){ 1.0f, 0.0f, 0.5f };
	b3BodyId body = AddBody( c, &bd );
	b3ShapeDef sd = EventShapeDef();
	sd.filter.groupIndex = -group; // the limbs of one ragdoll don't collide
	AddCapsule( body, &sd, a, b, radius );
	return body;
}

static void Ball( Ctx* c, b3BodyId a, b3BodyId b, b3Pos anchor, float cone, float twist )
{
	b3SphericalJointDef def = b3DefaultSphericalJointDef();
	LinkFrames( &def.base, a, b, anchor );
	def.enableConeLimit = true;
	def.coneAngle = cone;
	def.enableTwistLimit = true;
	def.lowerTwistAngle = -twist;
	def.upperTwistAngle = twist;
	def.enableSpring = true;
	def.hertz = 0.0f;
	def.enableMotor = true;
	def.maxMotorTorque = 2.0f; // joint friction
	AddJoint( c, b3CreateSphericalJoint( c->world, &def ) );
}

static void Hinge( Ctx* c, b3BodyId a, b3BodyId b, b3Pos anchor, float lower, float upper )
{
	b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
	LinkFrames( &def.base, a, b, anchor );
	def.enableLimit = true;
	def.lowerAngle = lower;
	def.upperAngle = upper;
	def.enableMotor = true;
	def.maxMotorTorque = 1.0f;
	AddJoint( c, b3CreateRevoluteJoint( c->world, &def ) );
}

static void CreateRagdoll( Ctx* c, b3Pos o, int group )
{
	b3Vec3 up = { 0.0f, 0.2f, 0.0f };
	b3Vec3 down = { 0.0f, -0.2f, 0.0f };
	b3BodyId torso = Limb( c, o, down, up, 0.18f, group );
	b3BodyId head = Limb( c, (b3Pos){ o.x, o.y + 0.6f, o.z }, (b3Vec3){ 0.0f, -0.05f, 0.0f },
						  (b3Vec3){ 0.0f, 0.05f, 0.0f }, 0.14f, group );
	Ball( c, torso, head, (b3Pos){ o.x, o.y + 0.42f, o.z }, 0.5f, 0.4f );
	for ( int side = -1; side <= 1; side += 2 )
	{
		float s = (float)side;
		b3BodyId thigh = Limb( c, (b3Pos){ o.x + 0.12f * s, o.y - 0.65f, o.z }, down, up, 0.09f, group );
		b3BodyId shin = Limb( c, (b3Pos){ o.x + 0.12f * s, o.y - 1.15f, o.z }, down, up, 0.08f, group );
		Ball( c, torso, thigh, (b3Pos){ o.x + 0.12f * s, o.y - 0.4f, o.z }, 0.8f, 0.3f );
		Hinge( c, thigh, shin, (b3Pos){ o.x + 0.12f * s, o.y - 0.9f, o.z }, -1.8f, 0.0f );
		b3BodyId upper = Limb( c, (b3Pos){ o.x + 0.45f * s, o.y + 0.3f, o.z }, (b3Vec3){ -0.15f, 0.0f, 0.0f },
							   (b3Vec3){ 0.15f, 0.0f, 0.0f }, 0.07f, group );
		b3BodyId lower = Limb( c, (b3Pos){ o.x + 0.8f * s, o.y + 0.3f, o.z }, (b3Vec3){ -0.15f, 0.0f, 0.0f },
							   (b3Vec3){ 0.15f, 0.0f, 0.0f }, 0.06f, group );
		Ball( c, torso, upper, (b3Pos){ o.x + 0.25f * s, o.y + 0.3f, o.z }, 1.2f, 0.5f );
		Hinge( c, upper, lower, (b3Pos){ o.x + 0.63f * s, o.y + 0.3f, o.z }, 0.0f, 2.0f );
	}
}

static void CreateRagdolls( Ctx* c )
{
	AddGround( c, 30.0f );
	b3BodyDef sd = BodyAt( b3_staticBody, 0.0f, 0.0f, 0.0f );
	b3BodyId stairs = AddBody( c, &sd );
	b3ShapeDef shape = EventShapeDef();
	for ( int i = 0; i < 8; ++i )
	{
		b3BoxHull step = b3MakeOffsetBoxHull( 0.5f, 0.25f * ( i + 1 ), 3.0f, (b3Vec3){ 4.0f - i, 0.25f * ( i + 1 ), 0.0f } );
		b3CreateHullShape( stairs, &shape, &step.base );
	}
	for ( int i = 0; i < 8; ++i )
	{
		CreateRagdoll( c, (b3Pos){ -2.0f + 0.7f * ( i % 4 ), 4.0f + 2.5f * ( i / 4 ) + 0.3f * i, -1.0f + 0.6f * ( i % 3 ) },
					   i + 1 );
	}
}

// A triangle mesh ground (a wave) with a box mesh obstacle; mixed shapes fall on it.
static void CreateMeshGround( Ctx* c )
{
	MakeMixedHulls( c );
	b3MeshData* wave = b3CreateWaveMesh( 30, 30, 1.0f, 0.8f, 0.07f, 0.11f );
	c->meshes[c->meshCount++] = wave;
	b3MeshData* box = b3CreateBoxMesh( (b3Vec3){ 3.0f, 1.0f, 0.0f }, (b3Vec3){ 1.0f, 1.0f, 2.0f }, true );
	c->meshes[c->meshCount++] = box;
	b3BodyDef bd = BodyAt( b3_staticBody, 0.0f, 0.0f, 0.0f );
	b3BodyId ground = AddBody( c, &bd );
	b3ShapeDef sd = EventShapeDef();
	c->shapes[c->shapeCount++] = b3CreateMeshShape( ground, &sd, wave, (b3Vec3){ 1.0f, 1.0f, 1.0f } );
	c->shapes[c->shapeCount++] = b3CreateMeshShape( ground, &sd, box, (b3Vec3){ 1.0f, 1.5f, -1.0f } );
	DropMixed( c, 80, (b3Vec3){ 0.0f, 6.0f, 0.0f }, (b3Vec3){ 8.0f, 3.0f, 8.0f }, true );
}

// A height-field ground (a wave, with holes).
static void CreateHeightField( Ctx* c )
{
	MakeMixedHulls( c );
	b3HeightFieldData* hf = b3CreateWave( 33, 33, (b3Vec3){ 0.75f, 1.2f, 0.75f }, 0.06f, 0.09f, true );
	c->heightFields[c->heightFieldCount++] = hf;
	b3BodyDef bd = BodyAt( b3_staticBody, -12.0f, 0.0f, -12.0f );
	b3BodyId ground = AddBody( c, &bd );
	b3ShapeDef sd = EventShapeDef();
	c->shapes[c->shapeCount++] = b3CreateHeightFieldShape( ground, &sd, hf );
	AddGround( c, 30.0f ); // a floor under it, for what falls through the holes
	DropMixed( c, 80, (b3Vec3){ 0.0f, 6.0f, 0.0f }, (b3Vec3){ 8.0f, 3.0f, 8.0f }, true );
}

// A static baked compound (spheres, capsules, hulls) and dynamic runtime compounds (several shapes per body,
// transformed hulls).
static void CreateCompound( Ctx* c )
{
	MakeMixedHulls( c );
	b3BoxHull block = b3MakeBoxHull( 0.8f, 0.3f, 0.8f );
	b3SurfaceMaterial mat = b3DefaultSurfaceMaterial();

	b3CompoundSphereDef spheres[6];
	b3CompoundCapsuleDef capsules[6];
	b3CompoundHullDef hulls[12];
	for ( int i = 0; i < 6; ++i )
	{
		spheres[i] = (b3CompoundSphereDef){ { { -5.0f + 2.0f * i, 0.0f, 3.0f }, 0.6f }, mat };
		capsules[i] = (b3CompoundCapsuleDef){
			{ { -5.0f + 2.0f * i, 0.2f, -3.0f }, { -4.0f + 2.0f * i, 0.2f, -3.5f }, 0.4f }, mat };
	}
	for ( int i = 0; i < 12; ++i )
	{
		b3Transform xf = { { -6.0f + 1.1f * i, -0.2f + 0.05f * ( i % 3 ), 0.0f },
						   b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.25f * i ) };
		hulls[i] = (b3CompoundHullDef){ i % 3 == 2 ? c->hulls[1] : &block.base, xf, mat };
	}
	b3CompoundDef def = { 0 };
	def.spheres = spheres;
	def.sphereCount = 6;
	def.capsules = capsules;
	def.capsuleCount = 6;
	def.hulls = hulls;
	def.hullCount = 12;
	b3CompoundData* compound = b3CreateCompound( &def );
	c->compounds[c->compoundCount++] = compound;

	AddGround( c, 30.0f );
	b3BodyDef bd = BodyAt( b3_staticBody, 0.0f, 0.6f, 0.0f );
	b3BodyId body = AddBody( c, &bd );
	b3ShapeDef sd = EventShapeDef();
	c->shapes[c->shapeCount++] = b3CreateBakedCompoundShape( body, &sd, compound );

	for ( int i = 0; i < 30; ++i )
	{
		b3BodyDef dd = BodyAt( b3_dynamicBody, RandFloat( -6.0f, 6.0f ), 4.0f + 0.8f * i, RandFloat( -3.0f, 3.0f ) );
		dd.rotation = RandRotation();
		b3BodyId d = AddBody( c, &dd );
		AddSphere( d, &sd, 0.25f );
		b3Capsule cap = { { 0.3f, 0.0f, 0.0f }, { 0.3f, 0.6f, 0.0f }, 0.15f };
		b3CreateCapsuleShape( d, &sd, &cap );
		b3Transform xf = { { -0.3f, 0.1f, 0.2f }, b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.4f ) };
		b3CreateTransformedHullShape( d, &sd, c->hulls[i % 3], xf, (b3Vec3){ 0.6f, 0.8f, 0.6f } );
	}
}

// Fast bodies: bullets fired through thin walls, fast non-bullets against static geometry (continuous).
static void CreateBullets( Ctx* c )
{
	AddGround( c, 50.0f );
	b3BodyDef wd = BodyAt( b3_staticBody, 0.0f, 0.0f, 0.0f );
	b3BodyId walls = AddBody( c, &wd );
	b3ShapeDef sd = EventShapeDef();
	for ( int i = 0; i < 3; ++i )
	{
		b3BoxHull wall = b3MakeOffsetBoxHull( 0.02f, 3.0f, 6.0f, (b3Vec3){ 10.0f + 3.0f * i, 3.0f, 0.0f } );
		b3CreateHullShape( walls, &sd, &wall.base );
	}
	// A thin dynamic box stack to knock over.
	for ( int i = 0; i < 6; ++i )
	{
		b3BodyDef bd = BodyAt( b3_dynamicBody, 6.0f, 0.5f + 1.0f * i, 0.0f );
		b3BodyId body = AddBody( c, &bd );
		AddBox( body, &sd, 0.1f, 0.5f, 1.0f );
	}
}

static void TickBullets( Ctx* c, int tick )
{
	if ( tick % 12 != 1 || tick > 150 )
	{
		return;
	}
	int k = tick / 12;
	b3ShapeDef sd = EventShapeDef();
	b3BodyDef bd = BodyAt( b3_dynamicBody, -10.0f, 1.0f + 0.3f * ( k % 8 ), -2.0f + 0.35f * ( k % 11 ) );
	bd.isBullet = k % 2 == 0;
	bd.linearVelocity = (b3Vec3){ 150.0f + 10.0f * k, 2.0f, 0.0f };
	bd.angularVelocity = (b3Vec3){ 0.0f, 0.0f, 20.0f };
	b3BodyId body = AddBody( c, &bd );
	if ( k % 3 == 0 )
	{
		AddSphere( body, &sd, 0.1f );
	}
	else if ( k % 3 == 1 )
	{
		AddCapsule( body, &sd, (b3Vec3){ -0.15f, 0.0f, 0.0f }, (b3Vec3){ 0.15f, 0.0f, 0.0f }, 0.05f );
	}
	else
	{
		AddBox( body, &sd, 0.1f, 0.1f, 0.1f );
	}
	// A fast falling non-bullet onto the ground (continuous against static).
	b3BodyDef fd = BodyAt( b3_dynamicBody, -5.0f + 0.5f * k, 20.0f, 4.0f );
	fd.linearVelocity = (b3Vec3){ 0.0f, -120.0f, 0.0f };
	b3BodyId fast = AddBody( c, &fd );
	AddBox( fast, &sd, 0.15f, 0.15f, 0.15f );
}

// Sensors: a static sensor box and a moving kinematic sensor sphere; visitors fall through and roll by.
static void CreateSensor( Ctx* c )
{
	AddGround( c, 30.0f );
	b3ShapeDef sensorDef = b3DefaultShapeDef();
	sensorDef.isSensor = true;
	sensorDef.enableSensorEvents = true;

	b3BodyDef bd = BodyAt( b3_staticBody, 0.0f, 2.0f, 0.0f );
	b3BodyId zone = AddBody( c, &bd );
	c->sensors[c->sensorCount++] = AddBox( zone, &sensorDef, 3.0f, 2.0f, 3.0f );

	b3BodyDef kd = BodyAt( b3_kinematicBody, -8.0f, 1.0f, 0.0f );
	kd.linearVelocity = (b3Vec3){ 2.0f, 0.0f, 0.0f };
	b3BodyId mover = AddBody( c, &kd );
	c->user[0] = mover;
	c->sensors[c->sensorCount++] = AddSphere( mover, &sensorDef, 1.5f );

	MakeMixedHulls( c );
	DropMixed( c, 50, (b3Vec3){ 0.0f, 8.0f, 0.0f }, (b3Vec3){ 5.0f, 4.0f, 3.0f }, true );
	// Some visitors with sensor events off: must not show up.
	b3ShapeDef quiet = b3DefaultShapeDef();
	for ( int i = 0; i < 5; ++i )
	{
		b3BodyDef qd = BodyAt( b3_dynamicBody, -2.0f + i, 10.0f, 0.5f );
		b3BodyId q = AddBody( c, &qd );
		AddSphere( q, &quiet, 0.3f );
	}
}

static void TickSensor( Ctx* c, int tick )
{
	if ( tick == 160 )
	{
		b3Body_SetLinearVelocity( c->user[0], (b3Vec3){ -2.0f, 0.0f, 0.5f } );
	}
	for ( int i = 0; i < c->sensorCount; ++i )
	{
		b3ShapeId visitors[256];
		int n = b3Shape_GetSensorData( c->sensors[i], visitors, 256 );
		HashInt( &c->queryHash, n );
		for ( int j = 0; j < n; ++j )
		{
			HashShapeId( &c->queryHash, visitors[j] );
		}
		c->queryCount += n;
	}
}

// Sleep and wake: stacks fall asleep; then a body is woken, one is destroyed under a stack, one disabled and
// enabled, one teleported, impulses and forces applied, and new bodies added onto sleeping ones.
static void CreateSleep( Ctx* c )
{
	AddGround( c, 30.0f );
	b3ShapeDef sd = EventShapeDef();
	for ( int s = 0; s < 4; ++s )
	{
		for ( int i = 0; i < 5; ++i )
		{
			b3BodyDef bd = BodyAt( b3_dynamicBody, -9.0f + 6.0f * s, 0.5f + 1.0f * i, 0.0f );
			b3BodyId body = AddBody( c, &bd );
			AddBox( body, &sd, 0.5f, 0.5f, 0.5f );
			if ( i == 0 )
			{
				c->user[s] = body; // the bottom box of each stack
			}
		}
	}
	b3BodyDef bd = BodyAt( b3_dynamicBody, 0.0f, 0.5f, 6.0f );
	c->user[4] = AddBody( c, &bd );
	AddSphere( c->user[4], &sd, 0.5f );
	b3BodyDef kd = BodyAt( b3_kinematicBody, 0.0f, 3.0f, -6.0f );
	c->user[5] = AddBody( c, &kd );
	AddBox( c->user[5], &sd, 2.0f, 0.2f, 2.0f );
}

static void TickSleep( Ctx* c, int tick )
{
	switch ( tick )
	{
		case 200:
			b3Body_SetAwake( c->user[1], true );
			b3Body_ApplyLinearImpulseToCenter( c->user[4], (b3Vec3){ 0.0f, 3000.0f, -800.0f }, true );
			break;
		case 230:
			b3DestroyBody( c->user[0] ); // the stack above falls
			break;
		case 260:
			b3Body_Disable( c->user[2] );
			break;
		case 290:
			b3Body_Enable( c->user[2] );
			b3Body_SetTransform( c->user[3], (b3Pos){ 6.0f, 6.0f, 2.0f },
								 b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.3f ) );
			break;
		case 320:
		{
			b3ShapeDef sd = EventShapeDef();
			for ( int i = 0; i < 3; ++i )
			{
				b3BodyDef bd = BodyAt( b3_dynamicBody, -3.0f + 0.2f * i, 6.0f + 1.0f * i, 0.0f );
				b3BodyId body = AddBody( c, &bd );
				AddBox( body, &sd, 0.4f, 0.4f, 0.4f );
			}
			b3Body_ApplyForce( c->user[1], (b3Vec3){ 0.0f, 0.0f, 400.0f }, (b3Pos){ -3.0f, 1.0f, 0.0f }, false );
			b3Body_ApplyTorque( c->user[4], (b3Vec3){ 0.0f, 50.0f, 0.0f }, true );
			break;
		}
		default:
			break;
	}
	if ( tick >= 340 && tick < 400 )
	{
		// The kinematic platform comes down onto the sphere's spot and goes back.
		float y = tick < 370 ? 3.0f - 0.05f * ( tick - 340 ) : 1.5f + 0.05f * ( tick - 370 );
		b3WorldTransform target = { { 0.0f, y, -6.0f + 0.02f * ( tick - 340 ) }, b3Quat_identity };
		b3Body_SetTargetTransform( c->user[5], target, 1.0f / 60.0f, true );
	}
	if ( tick == 400 )
	{
		b3Body_SetLinearVelocity( c->user[5], b3Vec3_zero );
	}
}

// Explosions over a field of boxes and spheres.
static void CreateExplosion( Ctx* c )
{
	AddGround( c, 30.0f );
	b3ShapeDef sd = EventShapeDef();
	for ( int i = 0; i < 10; ++i )
	{
		for ( int j = 0; j < 10; ++j )
		{
			b3BodyDef bd = BodyAt( b3_dynamicBody, -4.5f + i, 0.5f, -4.5f + j );
			b3BodyId body = AddBody( c, &bd );
			if ( ( i + j ) % 3 == 0 )
			{
				AddSphere( body, &sd, 0.45f );
			}
			else
			{
				AddBox( body, &sd, 0.45f, 0.45f, 0.45f );
			}
		}
	}
}

static void TickExplosion( Ctx* c, int tick )
{
	if ( tick == 60 || tick == 140 )
	{
		b3ExplosionDef def = b3DefaultExplosionDef();
		def.position = tick == 60 ? (b3Pos){ 0.0f, -0.5f, 0.0f } : (b3Pos){ 2.0f, 0.0f, -1.0f };
		def.radius = 4.0f;
		def.falloff = 2.0f;
		def.impulsePerArea = tick == 60 ? 20.0f : 35.0f;
		b3World_Explode( c->world, &def );
	}
}

// Queries: ray casts (closest and all hits), shape casts, overlaps and mover casts over a mixed world with mesh,
// height field and compound geometry, every tick.
static void CreateQueries( Ctx* c )
{
	MakeMixedHulls( c );
	AddGround( c, 40.0f );
	b3MeshData* mesh = b3CreateWaveMesh( 12, 12, 1.0f, 0.5f, 0.1f, 0.13f );
	c->meshes[c->meshCount++] = mesh;
	b3BodyDef md = BodyAt( b3_staticBody, -15.0f, 0.5f, 0.0f );
	b3BodyId mb = AddBody( c, &md );
	b3ShapeDef sd = EventShapeDef();
	c->shapes[c->shapeCount++] = b3CreateMeshShape( mb, &sd, mesh, (b3Vec3){ 1.0f, 1.0f, 1.0f } );
	b3HeightFieldData* hf = b3CreateWave( 17, 17, (b3Vec3){ 0.75f, 1.0f, 0.75f }, 0.1f, 0.07f, false );
	c->heightFields[c->heightFieldCount++] = hf;
	b3BodyDef hd = BodyAt( b3_staticBody, 9.0f, 0.2f, -6.0f );
	b3BodyId hb = AddBody( c, &hd );
	c->shapes[c->shapeCount++] = b3CreateHeightFieldShape( hb, &sd, hf );
	DropMixed( c, 60, (b3Vec3){ 0.0f, 5.0f, 0.0f }, (b3Vec3){ 14.0f, 4.0f, 5.0f }, true );
}

static void TickQueries( Ctx* c, int tick )
{
	b3QueryFilter filter = b3DefaultQueryFilter();
	float t = (float)( tick % 60 ) / 60.0f;

	// All hits along a sweeping ray, and the closest by clipping.
	for ( int i = 0; i < 4; ++i )
	{
		b3Pos origin = { -20.0f, 0.3f + 0.6f * i, -4.0f + 8.0f * t };
		b3Vec3 translation = { 40.0f, 0.5f - 0.3f * i, 1.0f };
		HashStats( c, b3World_CastRay( c->world, origin, translation, filter, CastAllCallback, c ) );
		HashStats( c, b3World_CastRay( c->world, origin, translation, filter, CastClosestCallback, c ) );
	}

	// Shape casts: a sphere, a capsule and a box proxy.
	b3Vec3 sphere[1] = { { 0.0f, 0.0f, 0.0f } };
	b3Vec3 capsule[2] = { { -0.3f, 0.0f, 0.0f }, { 0.3f, 0.0f, 0.0f } };
	b3Vec3 box[8];
	for ( int i = 0; i < 8; ++i )
	{
		box[i] = (b3Vec3){ ( i & 1 ) ? 0.4f : -0.4f, ( i & 2 ) ? 0.3f : -0.3f, ( i & 4 ) ? 0.2f : -0.2f };
	}
	b3ShapeProxy proxies[3] = { { sphere, 1, 0.3f }, { capsule, 2, 0.15f }, { box, 8, 0.0f } };
	for ( int i = 0; i < 3; ++i )
	{
		b3Pos origin = { -18.0f + 36.0f * t, 8.0f, -3.0f + 3.0f * i };
		HashStats( c, b3World_CastShape( c->world, origin, &proxies[i], (b3Vec3){ 0.5f, -12.0f, 0.2f }, filter,
										 CastClosestCallback, c ) );
		b3Pos at = { -16.0f + 32.0f * t, 0.6f, -2.0f + 2.0f * i };
		HashStats( c, b3World_OverlapShape( c->world, at, &proxies[i], filter, OverlapCallback, c ) );
	}

	// Mover: cast a character capsule and collect its planes.
	b3Capsule mover = { { 0.0f, 0.3f, 0.0f }, { 0.0f, 1.1f, 0.0f }, 0.3f };
	b3Pos moverOrigin = { -12.0f + 24.0f * t, 0.05f, 1.0f };
	float fraction = b3World_CastMover( c->world, moverOrigin, &mover, (b3Vec3){ 2.0f, -0.5f, 0.3f }, filter, NULL, NULL );
	HashFloat( &c->queryHash, fraction );
	b3World_CollideMover( c->world, moverOrigin, &mover, filter, PlaneCallback, c );

	// Direct shape and body ray casts against the static geometry.
	for ( int i = 0; i < c->shapeCount; ++i )
	{
		b3Pos o = { -20.0f + 30.0f * t, 6.0f, -5.0f + 2.0f * i };
		b3WorldCastOutput out = b3Shape_RayCast( c->shapes[i], o, (b3Vec3){ 2.0f, -10.0f, 1.0f } );
		HashVec( &c->queryHash, out.normal );
		HashVec( &c->queryHash, out.point );
		HashFloat( &c->queryHash, out.fraction );
		HashInt( &c->queryHash, out.triangleIndex );
		HashInt( &c->queryHash, out.hit );
		HashAABB( &c->queryHash, b3Shape_GetAABB( c->shapes[i] ) );
	}
	if ( c->bodyCount > 10 && b3Body_IsValid( c->bodies[10] ) )
	{
		b3BodyId body = c->bodies[10];
		b3BodyCastResult r = b3Body_CastRay( body, (b3Pos){ -20.0f, 1.0f, 0.0f }, (b3Vec3){ 40.0f, 0.0f, 0.0f }, filter,
											 1.0f, b3Body_GetTransform( body ) );
		HashShapeId( &c->queryHash, r.shapeId );
		HashVec( &c->queryHash, r.point );
		HashFloat( &c->queryHash, r.fraction );
		HashInt( &c->queryHash, r.hit );
	}
}

// Mutations through the API during the run: custom filtering and pre-solve callbacks, a one-way platform, gravity
// changes, filter changes, body type changes of free bodies, destroying shapes and joints, a conveyor (tangent
// speed), damping and gravity scale.
static bool CustomFilter( b3ShapeId shapeIdA, b3ShapeId shapeIdB, void* context )
{
	( void )context;
	// Shapes with odd indices don't collide with each other.
	return ( ( shapeIdA.index1 & 1 ) & ( shapeIdB.index1 & 1 ) ) == 0;
}

static bool PreSolve( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3Pos point, b3Vec3 normal, void* context )
{
	( void )point;
	b3ShapeId platform = *(const b3ShapeId*)context;
	float sign;
	if ( shapeIdA.index1 == platform.index1 && shapeIdA.generation == platform.generation )
	{
		sign = 1.0f;
	}
	else if ( shapeIdB.index1 == platform.index1 && shapeIdB.generation == platform.generation )
	{
		sign = -1.0f;
	}
	else
	{
		return true;
	}
	// One-way: only from above (the normal points from A to B).
	return sign * normal.y > 0.1f;
}

static b3ShapeId g_platform;

static void CreateMutations( Ctx* c )
{
	b3BodyId ground = AddGround( c, 30.0f );
	( void )ground;
	b3ShapeDef sd = EventShapeDef();
	sd.enablePreSolveEvents = true;

	// The one-way platform.
	b3BodyDef pd = BodyAt( b3_staticBody, 0.0f, 3.0f, 0.0f );
	b3BodyId platform = AddBody( c, &pd );
	g_platform = AddBox( platform, &sd, 3.0f, 0.1f, 3.0f );
	b3World_SetPreSolveCallback( c->world, PreSolve, &g_platform );

	// A conveyor.
	b3ShapeDef conveyor = sd;
	conveyor.baseMaterial.tangentVelocity = (b3Vec3){ 2.0f, 0.0f, 0.0f };
	b3BodyDef cd = BodyAt( b3_staticBody, 10.0f, 0.5f, 0.0f );
	b3BodyId belt = AddBody( c, &cd );
	AddBox( belt, &conveyor, 4.0f, 0.2f, 1.5f );

	// Bodies that use custom filtering.
	b3World_SetCustomFilterCallback( c->world, CustomFilter, NULL );
	b3ShapeDef custom = sd;
	custom.enableCustomFiltering = true;
	for ( int i = 0; i < 24; ++i )
	{
		b3BodyDef bd = BodyAt( b3_dynamicBody, -2.0f + 0.5f * ( i % 8 ), 1.0f + 1.2f * ( i / 8 ), -1.0f + 0.3f * ( i % 5 ) );
		bd.linearVelocity = (b3Vec3){ 0.0f, i % 2 == 0 ? 6.0f : 0.0f, 0.0f }; // some jump through the platform
		bd.linearDamping = 0.1f * ( i % 3 );
		bd.angularDamping = 0.2f * ( i % 2 );
		bd.gravityScale = i % 4 == 0 ? 0.5f : 1.0f;
		b3BodyId body = AddBody( c, &bd );
		if ( i % 2 == 0 )
		{
			AddBox( body, &custom, 0.2f, 0.2f, 0.2f );
		}
		else
		{
			AddSphere( body, &custom, 0.2f );
		}
	}
	// Boxes on the conveyor.
	for ( int i = 0; i < 4; ++i )
	{
		b3BodyDef bd = BodyAt( b3_dynamicBody, 7.5f + 1.2f * i, 1.2f, 0.0f );
		b3BodyId body = AddBody( c, &bd );
		AddBox( body, &sd, 0.4f, 0.4f, 0.4f );
	}
	// A pair with a joint that gets destroyed, and a body whose shape gets destroyed.
	b3BodyDef ad = BodyAt( b3_dynamicBody, -8.0f, 4.0f, 0.0f );
	b3BodyId a = AddBody( c, &ad );
	AddBox( a, &sd, 0.5f, 0.5f, 0.5f );
	b3BodyDef bd2 = BodyAt( b3_dynamicBody, -8.0f, 2.5f, 0.0f );
	b3BodyId b = AddBody( c, &bd2 );
	AddSphere( b, &sd, 0.4f );
	c->user[1] = b;
	b3BodyDef anchorDef = BodyAt( b3_staticBody, -8.0f, 5.0f, 0.0f );
	b3BodyId anchor = AddBody( c, &anchorDef );
	b3RevoluteJointDef rd = b3DefaultRevoluteJointDef();
	LinkFrames( &rd.base, anchor, a, (b3Pos){ -8.0f, 4.6f, 0.0f } );
	c->userJoints[0] = AddJoint( c, b3CreateRevoluteJoint( c->world, &rd ) );
	b3WeldJointDef wd = b3DefaultWeldJointDef();
	LinkFrames( &wd.base, a, b, (b3Pos){ -8.0f, 3.25f, 0.0f } );
	c->userJoints[1] = AddJoint( c, b3CreateWeldJoint( c->world, &wd ) );
	b3BodyDef ed = BodyAt( b3_dynamicBody, 5.0f, 2.0f, 4.0f );
	b3BodyId e = AddBody( c, &ed );
	c->user[3] = e;
	c->shapes[c->shapeCount++] = AddBox( e, &sd, 0.5f, 0.5f, 0.5f );
	AddSphere( e, &sd, 0.6f );
	// A free body that changes type: kinematic, then dynamic.
	b3BodyDef td = BodyAt( b3_dynamicBody, -5.0f, 6.0f, 4.0f );
	c->user[4] = AddBody( c, &td );
	AddBox( c->user[4], &sd, 0.5f, 0.5f, 0.5f );
}

static void TickMutations( Ctx* c, int tick )
{
	switch ( tick )
	{
		case 60:
			b3World_SetGravity( c->world, (b3Vec3){ 1.0f, -8.0f, 0.5f } );
			break;
		case 90:
			b3DestroyJoint( c->userJoints[1], true );
			break;
		case 100:
			b3DestroyShape( c->shapes[0], true );
			c->shapeCount = 0;
			break;
		case 120:
		{
			b3ShapeId shapes[4];
			int n = b3Body_GetShapes( c->user[1], shapes, 4 );
			b3Filter f = b3DefaultFilter();
			f.categoryBits = 2;
			f.maskBits = 1;
			for ( int i = 0; i < n; ++i )
			{
				b3Shape_SetFilter( shapes[i], f, true );
				b3Shape_SetFriction( shapes[i], 0.1f );
				b3Shape_SetRestitution( shapes[i], 0.5f );
			}
			break;
		}
		case 140:
			b3DestroyJoint( c->userJoints[0], true );
			b3Body_SetType( c->user[4], b3_kinematicBody );
			b3Body_SetLinearVelocity( c->user[4], (b3Vec3){ 1.0f, 0.0f, 0.0f } );
			break;
		case 180:
			b3Body_SetType( c->user[4], b3_dynamicBody );
			b3World_SetGravity( c->world, (b3Vec3){ 0.0f, -10.0f, 0.0f } );
			break;
		default:
			break;
	}
}

// A large scene, so the multithreaded stages split work across workers: pyramids of ~800 boxes, and a dynamic
// plank carrying more contacts than the graph has colors (the overflow constraints).
static void CreateLarge( Ctx* c )
{
	AddGround( c, 80.0f );
	b3ShapeDef sd = EventShapeDef();
	int base = 20;
	float h = 0.5f;
	for ( int p = 0; p < 4; ++p )
	{
		float z = -15.0f + 10.0f * p;
		for ( int row = 0; row < base; ++row )
		{
			for ( int i = 0; i < base - row; ++i )
			{
				float x = ( 2.0f * i - ( base - row ) + 1.0f ) * ( h + 0.02f );
				b3BodyDef bd = BodyAt( b3_dynamicBody, x, h + 2.0f * h * row, z );
				b3BodyId body = AddBody( c, &bd );
				AddBox( body, &sd, h, h, h );
			}
		}
	}
	b3BodyDef pd = BodyAt( b3_dynamicBody, 30.0f, 0.5f, 0.0f );
	b3BodyId plank = AddBody( c, &pd );
	AddBox( plank, &sd, 4.0f, 0.5f, 4.0f );
	for ( int i = 0; i < 64; ++i )
	{
		b3BodyDef bd = BodyAt( b3_dynamicBody, 26.5f + 1.0f * ( i % 8 ), 1.3f + 0.01f * ( i % 3 ), -3.5f + 1.0f * ( i / 8 ) );
		b3BodyId body = AddBody( c, &bd );
		AddSphere( body, &sd, 0.3f );
	}
}

static void TickLarge( Ctx* c, int tick )
{
	if ( tick == 100 )
	{
		b3ExplosionDef def = b3DefaultExplosionDef();
		def.position = (b3Pos){ 0.0f, 0.0f, -5.0f };
		def.radius = 6.0f;
		def.falloff = 3.0f;
		def.impulsePerArea = 15.0f;
		b3World_Explode( c->world, &def );
	}
}

static const Scene g_scenes[] = {
	{ "pyramid", 400, CreatePyramid, TickPyramid, 12.0f },
	{ "pile", 360, CreatePile, NULL, 5.0f },
	{ "joints", 300, CreateJoints, TickJoints, 30.0f },
	{ "ragdolls", 300, CreateRagdolls, NULL, 6.0f },
	{ "mesh", 300, CreateMeshGround, NULL, 12.0f },
	{ "heightfield", 300, CreateHeightField, NULL, 12.0f },
	{ "compound", 300, CreateCompound, NULL, 8.0f },
	{ "bullets", 240, CreateBullets, TickBullets, 15.0f },
	{ "sensor", 300, CreateSensor, TickSensor, 8.0f },
	{ "sleep", 480, CreateSleep, TickSleep, 10.0f },
	{ "explosion", 240, CreateExplosion, TickExplosion, 6.0f },
	{ "queries", 200, CreateQueries, TickQueries, 18.0f },
	{ "mutations", 240, CreateMutations, TickMutations, 10.0f },
	{ "large", 200, CreateLarge, TickLarge, 30.0f },
};

#define SCENE_COUNT ( (int)( sizeof( g_scenes ) / sizeof( g_scenes[0] ) ) )

// --- the trace ------------------------------------------------------------------------------------------------------

static uint64_t StateHash( Ctx* c, int* awake )
{
	uint64_t h = FNV_OFFSET;
	*awake = 0;
	for ( int i = 0; i < c->bodyCount; ++i )
	{
		b3BodyId id = c->bodies[i];
		if ( b3Body_IsValid( id ) == false )
		{
			HashInt( &h, -1 );
			continue;
		}
		HashBodyId( &h, id );
		HashVec( &h, b3Body_GetPosition( id ) );
		HashQuat( &h, b3Body_GetRotation( id ) );
		HashVec( &h, b3Body_GetLinearVelocity( id ) );
		HashVec( &h, b3Body_GetAngularVelocity( id ) );
		bool isAwake = b3Body_IsAwake( id );
		HashInt( &h, isAwake );
		HashInt( &h, b3Body_IsEnabled( id ) );
		*awake += isAwake ? 1 : 0;
	}
	for ( int i = 0; i < c->jointCount; ++i )
	{
		b3JointId id = c->joints[i];
		if ( b3Joint_IsValid( id ) == false )
		{
			HashInt( &h, -1 );
			continue;
		}
		HashJointId( &h, id );
		HashVec( &h, b3Joint_GetConstraintForce( id ) );
		HashVec( &h, b3Joint_GetConstraintTorque( id ) );
		HashFloat( &h, b3Joint_GetLinearSeparation( id ) );
		if ( b3Joint_GetType( id ) != b3_wheelJoint )
		{
			// Upstream's wheel joint has no angular separation yet: B3_ASSERT( false ) (a debug build stops there).
			HashFloat( &h, b3Joint_GetAngularSeparation( id ) );
		}
	}
	return h;
}

// What the scene is built from, hashed at tick 0: mass data and bounds of every body.
static uint64_t SetupHash( Ctx* c )
{
	uint64_t h = FNV_OFFSET;
	for ( int i = 0; i < c->bodyCount; ++i )
	{
		b3BodyId id = c->bodies[i];
		if ( b3Body_IsValid( id ) == false )
		{
			continue;
		}
		HashMassData( &h, b3Body_GetMassData( id ) );
		if ( b3Body_GetShapeCount( id ) > 0 )
		{
			HashAABB( &h, b3Body_ComputeAABB( id ) );
		}
		HashFloat( &h, b3Body_GetMinExtent( id ) );
		HashVec( &h, b3Body_GetMaxExtent( id ) );
	}
	return h;
}

typedef struct EventCounts
{
	int begin, end, hit, sensorBegin, sensorEnd, moves, sleeps, jointEvents;
} EventCounts;

static uint64_t EventHash( Ctx* c, EventCounts* n )
{
	uint64_t h = FNV_OFFSET;
	b3ContactEvents ce = b3World_GetContactEvents( c->world );
	n->begin = ce.beginCount;
	n->end = ce.endCount;
	n->hit = ce.hitCount;
	HashInt( &h, ce.beginCount );
	for ( int i = 0; i < ce.beginCount; ++i )
	{
		HashShapeId( &h, ce.beginEvents[i].shapeIdA );
		HashShapeId( &h, ce.beginEvents[i].shapeIdB );
		HashContactId( &h, ce.beginEvents[i].contactId );
	}
	HashInt( &h, ce.endCount );
	for ( int i = 0; i < ce.endCount; ++i )
	{
		HashShapeId( &h, ce.endEvents[i].shapeIdA );
		HashShapeId( &h, ce.endEvents[i].shapeIdB );
		HashContactId( &h, ce.endEvents[i].contactId );
	}
	HashInt( &h, ce.hitCount );
	for ( int i = 0; i < ce.hitCount; ++i )
	{
		const b3ContactHitEvent* e = ce.hitEvents + i;
		HashShapeId( &h, e->shapeIdA );
		HashShapeId( &h, e->shapeIdB );
		HashContactId( &h, e->contactId );
		HashVec( &h, e->point );
		HashVec( &h, e->normal );
		HashFloat( &h, e->approachSpeed );
		HashU64( &h, e->userMaterialIdA );
		HashU64( &h, e->userMaterialIdB );
	}

	b3SensorEvents se = b3World_GetSensorEvents( c->world );
	n->sensorBegin = se.beginCount;
	n->sensorEnd = se.endCount;
	HashInt( &h, se.beginCount );
	for ( int i = 0; i < se.beginCount; ++i )
	{
		HashShapeId( &h, se.beginEvents[i].sensorShapeId );
		HashShapeId( &h, se.beginEvents[i].visitorShapeId );
	}
	HashInt( &h, se.endCount );
	for ( int i = 0; i < se.endCount; ++i )
	{
		HashShapeId( &h, se.endEvents[i].sensorShapeId );
		HashShapeId( &h, se.endEvents[i].visitorShapeId );
	}

	b3BodyEvents be = b3World_GetBodyEvents( c->world );
	n->moves = be.moveCount;
	n->sleeps = 0;
	HashInt( &h, be.moveCount );
	for ( int i = 0; i < be.moveCount; ++i )
	{
		const b3BodyMoveEvent* e = be.moveEvents + i;
		HashBodyId( &h, e->bodyId );
		HashVec( &h, e->transform.p );
		HashQuat( &h, e->transform.q );
		HashInt( &h, e->fellAsleep );
		n->sleeps += e->fellAsleep ? 1 : 0;
	}

	b3JointEvents je = b3World_GetJointEvents( c->world );
	n->jointEvents = je.count;
	HashInt( &h, je.count );
	for ( int i = 0; i < je.count; ++i )
	{
		HashJointId( &h, je.jointEvents[i].jointId );
	}
	return h;
}

static uint64_t CounterHash( Ctx* c, b3Counters* k )
{
	*k = b3World_GetCounters( c->world );
	uint64_t h = FNV_OFFSET;
	HashInt( &h, k->bodyCount );
	HashInt( &h, k->shapeCount );
	HashInt( &h, k->contactCount );
	HashInt( &h, k->jointCount );
	HashInt( &h, k->islandCount );
	HashInt( &h, k->awakeContactCount );
	HashInt( &h, k->staticTreeHeight );
	HashInt( &h, k->treeHeight );
	for ( int i = 0; i < 24; ++i )
	{
		HashInt( &h, k->colorCounts[i] );
	}
	HashInt( &h, b3World_GetAwakeBodyCount( c->world ) );
	return h;
}

static void FreeGeometry( Ctx* c )
{
	for ( int i = 0; i < c->hullCount; ++i )
	{
		b3DestroyHull( c->hulls[i] );
	}
	for ( int i = 0; i < c->meshCount; ++i )
	{
		b3DestroyMesh( c->meshes[i] );
	}
	for ( int i = 0; i < c->heightFieldCount; ++i )
	{
		b3DestroyHeightField( c->heightFields[i] );
	}
	for ( int i = 0; i < c->compoundCount; ++i )
	{
		b3DestroyCompound( c->compounds[i] );
	}
}

static Ctx g_ctx;
static bool g_width4;

static void RunScene( const Scene* scene, int workers )
{
	Ctx* c = &g_ctx;
	memset( c, 0, sizeof( *c ) );
	g_seed = 12345u;
	for ( const char* p = scene->name; *p; ++p )
	{
		g_seed = g_seed * 31u + (uint8_t)*p;
	}

	b3WorldDef def = b3DefaultWorldDef();
	def.workerCount = (uint32_t)workers;
	c->world = b3CreateWorld( &def );
	if ( g_width4 )
	{
		b3World_EnableSSE2Fallback( c->world, true );
	}
	scene->create( c );

	{
		b3Counters k;
		int awake;
		uint64_t s = StateHash( c, &awake );
		uint64_t setup = SetupHash( c );
		uint64_t counters = CounterHash( c, &k );
		printf( "%s 0 s=%016llx e=%016llx q=0000000000000000 c=%016llx bodies=%d awake=%d\n", scene->name,
				(unsigned long long)s, (unsigned long long)setup, (unsigned long long)counters, k.bodyCount, awake );
	}

	for ( int tick = 1; tick <= scene->ticks; ++tick )
	{
		c->queryHash = FNV_OFFSET;
		c->queryCount = 0;
		if ( scene->tick != NULL )
		{
			scene->tick( c, tick );
		}
		b3World_Step( c->world, 1.0f / 60.0f, 4 );

		EventCounts n;
		uint64_t events = EventHash( c, &n );
		BasicQueries( c, tick, scene->queryExtent );
		int awake;
		g_nonFinite = 0;
		uint64_t state = StateHash( c, &awake );
		int nonFinite = g_nonFinite;
		b3Counters k;
		uint64_t counters = CounterHash( c, &k );
		printf( "%s %d s=%016llx e=%016llx q=%016llx c=%016llx bodies=%d awake=%d contacts=%d islands=%d begin=%d "
				"end=%d hit=%d sbegin=%d send=%d moves=%d sleeps=%d jev=%d hits=%d\n",
				scene->name, tick, (unsigned long long)state, (unsigned long long)events,
				(unsigned long long)c->queryHash, (unsigned long long)counters, k.bodyCount, awake, k.contactCount,
				k.islandCount, n.begin, n.end, n.hit, n.sensorBegin, n.sensorEnd, n.moves, n.sleeps, n.jointEvents,
				c->queryCount, nonFinite > 0 ? " NON-FINITE" : "" );
		fflush( stdout ); // a crash keeps the ticks before it
	}

	b3DestroyWorld( c->world );
	FreeGeometry( c );
}

int main( int argc, char** argv )
{
	if ( argc >= 3 && strcmp( argv[2], "--w4" ) == 0 )
	{
		g_width4 = true;
		for ( int i = 2; i + 1 < argc; ++i )
		{
			argv[i] = argv[i + 1];
		}
		argc -= 1;
	}
	if ( argc >= 2 && strcmp( argv[1], "--list" ) == 0 )
	{
		for ( int i = 0; i < SCENE_COUNT; ++i )
		{
			printf( "%s %d\n", g_scenes[i].name, g_scenes[i].ticks );
		}
		return 0;
	}
	if ( argc < 2 || argc > 3 )
	{
		fprintf( stderr, "usage: compare <workers> [--w4] [scene] | --list\n" );
		return 2;
	}
	int workers = atoi( argv[1] );
	if ( workers < 1 || workers > 64 )
	{
		fprintf( stderr, "compare: bad worker count %s\n", argv[1] );
		return 2;
	}
	const char* only = argc == 3 ? argv[2] : NULL;
	int ran = 0;
	for ( int i = 0; i < SCENE_COUNT; ++i )
	{
		if ( only != NULL && strcmp( only, g_scenes[i].name ) != 0 )
		{
			continue;
		}
		RunScene( g_scenes + i, workers );
		ran += 1;
	}
	if ( ran == 0 )
	{
		fprintf( stderr, "compare: no scene %s\n", only );
		return 2;
	}
	fflush( stdout );
	return 0;
}
