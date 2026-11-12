// The scaling benchmark (bench/scale.odin) on the original: upstream's C Box3D at the pinned commit, with rollback
// through upstream's own world serializer (src/world_snapshot.c, what replay.c's keyframes seek with). The same scene,
// constants and measurements as scale.odin: save = b3SerializeWorld into a buffer, restore = b3DeserializeIntoShell of
// that buffer into the same live world, rollback = restore + 8 x (step + save into a ring of 8 buffers). Upstream has
// no separate portable image, so those columns print -. The hash is upstream's b3HashWorldState (body transforms and
// velocities): the same for every worker count, but not comparable with scale.odin's World_HashState.
//
// Built by `./b3 bench --original` against .out/upstream's static library, with upstream's internal headers: the
// serializer is not public API, but its functions are not static either, so no patch is needed.
#include "box3d/box3d.h"

#include "physics_world.h"
#include "world_snapshot.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DT ( 1.0f / 64.0f )
#define SUBSTEPS 4
#define SETTLE 240 // steps before measuring, so contacts and islands exist
#define REPS 64
#define ROLLBACK 8

typedef struct Result
{
	double step, save, restore, rollback; // µs, median
	int snapshot;						  // bytes
	uint64_t hash;						  // the world after the rollbacks
} Result;

// The serializer interns shape geometry (the hulls) into a recording's registry, and the deserializer resolves it
// through a reader's slots, which must match the registry ids; replay.c keeps the two in step by seeding the registry
// from the slots (b3RecSeedKeyframeRegistry). Here it goes the other way: the slots alias the registry's entries,
// refreshed whenever a save interned something new (only the first, in this scene).
typedef struct Serializer
{
	b3World* world;
	b3Recording* rec;
	b3RecReader rdr;
} Serializer;

static void Save( Serializer* s, b3RecBuffer* buf )
{
	buf->size = 0; // reuse the buffer's memory, as a ring of keyframes would
	b3SerializeWorld( s->world, buf, s->rec );
	b3GeometryRegistry* reg = &s->rec->registry;
	if ( reg->entries.count != s->rdr.slotCount )
	{
		s->rdr.slotCount = reg->entries.count;
		s->rdr.slots = realloc( s->rdr.slots, (size_t)s->rdr.slotCount * sizeof( b3RegistrySlot ) );
		for ( int i = 0; i < s->rdr.slotCount; ++i )
		{
			b3GeometryEntry* e = reg->entries.data + i;
			s->rdr.slots[i] = (b3RegistrySlot){ e->kind, e->byteCount, e->bytes };
		}
	}
}

static void Restore( Serializer* s, const b3RecBuffer* buf )
{
	if ( b3DeserializeIntoShell( buf->data, buf->size, s->world, &s->rdr ) == false )
	{
		fprintf( stderr, "scale_original: b3DeserializeIntoShell rejected a %d byte image\n", buf->size );
		exit( 1 );
	}
}

static int CompareDoubles( const void* a, const void* b )
{
	double x = *(const double*)a, y = *(const double*)b;
	return ( x > y ) - ( x < y );
}

static double Median( double* xs, int n )
{
	qsort( xs, (size_t)n, sizeof( double ), CompareDoubles );
	return xs[n / 2];
}

static double Us( uint64_t ticks )
{
	return 1000.0 * b3GetMilliseconds( ticks );
}

static void Fail( int n, int workers, const char* what, int tick )
{
	fprintf( stderr, "scale_original: %d bodies, %d workers: %s (tick %d)\n", n, workers, what, tick );
	exit( 1 );
}

static Result Run( int n, int workers )
{
	Result r = { 0 };
	b3WorldDef wd = b3DefaultWorldDef();
	wd.workerCount = (uint32_t)workers; // > 1: upstream's own scheduler (src/scheduler.c), as its benchmark app uses
	wd.enableSleep = false;
	b3WorldId w = b3CreateWorld( &wd );

	b3BodyDef bd = b3DefaultBodyDef();
	bd.position = (b3Pos){ 0, -0.5f, 0 };
	b3BodyId ground = b3CreateBody( w, &bd );
	b3ShapeDef sd = b3DefaultShapeDef();
	b3BoxHull gh = b3MakeBoxHull( 100, 0.5f, 100 );
	b3CreateHullShape( ground, &sd, &gh.base );
	b3BoxHull box = b3MakeBoxHull( 0.4f, 0.4f, 0.4f );
	int side = 10;
	for ( int i = 0; i < n; ++i )
	{
		int col = i % ( side * side );
		int layer = i / ( side * side );
		b3BodyDef d = b3DefaultBodyDef();
		d.type = b3_dynamicBody;
		d.position =
			(b3Pos){ (float)( col % side ) * 1.0f - 5, 0.4f + (float)layer * 0.85f, (float)( col / side ) * 1.0f - 5 };
		b3BodyId body = b3CreateBody( w, &d );
		b3ShapeDef s = b3DefaultShapeDef();
		s.density = 1;
		b3CreateHullShape( body, &s, &box.base );
	}
	for ( int i = 0; i < SETTLE; ++i )
	{
		b3World_Step( w, DT, SUBSTEPS );
	}

	Serializer ser = { .world = b3GetWorldFromId( w ), .rec = b3CreateRecording( 0 ) };
	b3RecBuffer snap = { 0 };
	b3RecBuffer ring[ROLLBACK] = { 0 };

	// Before timing anything: a restore must put the world back exactly. Save, step ROLLBACK ticks for the reference,
	// then restore over the moved-on world and step again with a save and restore before every tick; the body
	// transforms and velocities must match the reference at every tick.
	Save( &ser, &snap );
	uint64_t saved = b3HashWorldState( ser.world );
	uint64_t reference[ROLLBACK];
	for ( int k = 0; k < ROLLBACK; ++k )
	{
		b3World_Step( w, DT, SUBSTEPS );
		reference[k] = b3HashWorldState( ser.world );
	}
	Restore( &ser, &snap );
	if ( b3HashWorldState( ser.world ) != saved )
	{
		Fail( n, workers, "the restored world differs from the saved one", 0 );
	}
	for ( int k = 0; k < ROLLBACK; ++k )
	{
		Save( &ser, &ring[k] );
		Restore( &ser, &ring[k] );
		b3World_Step( w, DT, SUBSTEPS );
		if ( b3HashWorldState( ser.world ) != reference[k] )
		{
			Fail( n, workers, "stepping after a restore differs from stepping without one", k + 1 );
		}
	}
	Restore( &ser, &snap ); // back to the settled tick: what scale.odin measures at

	double samples[REPS];
	for ( int i = 0; i < REPS; ++i )
	{
		uint64_t t = b3GetTicks();
		Save( &ser, &snap );
		samples[i] = Us( t );
	}
	r.save = Median( samples, REPS );
	r.snapshot = snap.size;
	for ( int i = 0; i < REPS; ++i )
	{
		uint64_t t = b3GetTicks();
		Restore( &ser, &snap );
		samples[i] = Us( t );
	}
	r.restore = Median( samples, REPS );
	for ( int i = 0; i < REPS; ++i )
	{
		Restore( &ser, &snap );
		uint64_t t = b3GetTicks();
		b3World_Step( w, DT, SUBSTEPS );
		samples[i] = Us( t );
	}
	r.step = Median( samples, REPS );

	// The rollback a client does on a late input: back to the saved tick, then re-simulate and re-save 8 ticks.
	double rb[16];
	for ( int i = 0; i < 16; ++i )
	{
		uint64_t t = b3GetTicks();
		Restore( &ser, &snap );
		for ( int k = 0; k < ROLLBACK; ++k )
		{
			b3World_Step( w, DT, SUBSTEPS );
			Save( &ser, &ring[k] );
		}
		rb[i] = Us( t );
	}
	r.rollback = Median( rb, 16 );
	r.hash = b3HashWorldState( ser.world );
	// The timed saves are of the check's starting tick, so the rollback ends where its reference did.
	if ( r.hash != reference[ROLLBACK - 1] )
	{
		Fail( n, workers, "the rollback ends somewhere else than the reference", ROLLBACK );
	}

	for ( int k = 0; k < ROLLBACK; ++k )
	{
		b3RecBufFree( &ring[k] );
	}
	b3RecBufFree( &snap );
	free( ser.rdr.slots );
	b3DestroyRecording( ser.rec );
	b3DestroyWorld( w );
	return r;
}

int main( void )
{
	printf( "bodies workers    step µs    save µs restore µs  rollback(8) µs  snapshot bytes  image µs  image bytes"
			"  hash\n" );
	int counts[] = { 100, 1000, 2000 };
	int workerCounts[] = { 1, 2, 4 };
	for ( int c = 0; c < 3; ++c )
	{
		for ( int k = 0; k < 3; ++k )
		{
			Result r = Run( counts[c], workerCounts[k] );
			printf( "%6d %7d %10.1f %10.1f %10.1f %15.1f %15d %9s %12s  %016llx\n", counts[c], workerCounts[k], r.step,
					r.save, r.restore, r.rollback, r.snapshot, "-", "-", (unsigned long long)r.hash );
			fflush( stdout );
		}
	}
	return 0;
}
