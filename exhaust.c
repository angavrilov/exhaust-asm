/* exhaust.c: crippled pmars-like redcode simulator
 * $Id: exhaust.c,v 1.15 2002/12/28 05:49:45 rowan Exp $
 */

/* This file is part of `exhaust', a memory array redcode simulator.
 * Copyright (C) 2002 M Joonas Pihlaja
 * 
 * This file falls under the GNU Public License, version 2.  See the
 * file COPYING for details.  */

#define VERSION 1
#define REVISION 9
#define PATCHLEVEL 2

/* Note pmars incompatibility: Minimum warrior separation (minsep) is
   handled wrong by pmars (prior to 0.8.6) when minsep < MAXLENGTH.
   This affects running warriors in small cores since the other
   warriors might be loaded outside of core (it might even crash).
*/
#include <stdio.h>
#ifdef SYSV
#include <strings.h>
#else
#include <string.h>
#endif
#include <stdlib.h>
#include <time.h>
#include <ctype.h>

#include "exhaust.h"		/* types */
#include "asm.h"		/* assembler proto */
#include "sim.h"		/* simulator proto */

static unsigned int	NWarriors = 0;
static warrior_t	*Warriors = NULL; /* [NWarriors] */

static char *		*WarriorNames = NULL; /* [NWarriors] */
				/* File names of warriors. */

static field_t		*Positions = NULL; /* [NWarriors] */
static field_t		*StartPositions = NULL; /* [NWarriors] */
				/* Starting position of each warrior.
				 * The StartPositions[] array is a copy
				 * of Positions[], rotated by the number of
				 * the round, to emulate pMARS.
				 */

static unsigned int	*Deaths = NULL; /* [NWarriors] */

static u32_t		**Results = NULL; /* [NWarriors][NWarriors+1] */
				/* Results[war_id][j] is the number of
				   rounds in which warrior war_id
				   survives until the end with exactly
				   j-1 other warriors.  For j=0, then
				   it is the number of rounds where
				   the warrior died. */

static pspace_t **	PSpaces = NULL; /* [NWarriors] */
				/* p-spaces of warriors. */


/* Globals to communicate options from readargs() to main() */
int	OPT_cycles = 80000;	/* cycles until tie */
unsigned int OPT_rounds = 1;	/* number of rounds to fight */
unsigned int 	OPT_coresize = 8000; /* default core size */
unsigned int 	OPT_minsep = 0;	/* minimum distance between warriors */
int	OPT_processes = 8000;	/* default max. procs./warrior */
int	OPT_F = 0;		/* initial position of warrior 2 */
int	OPT_k = 0;		/* nothing (`koth' format flag) */
int	OPT_b = 0;		/* nothing (we are always `brief') */
int	OPT_m = 0;		/* multi-warrior output format. */

char *prog_name;
char errmsg[256];

/*---------------------------------------------------------------
 * Utilities
 */

#define min(x,y) ((x)<(y) ? (x) : (y))

/* xbasename(): custom basename(1); returns the filename from a path
 */
static
char *
xbasename(char *s)
{
    char *b;
    b = strrchr(s,'/');
    if (!b) {
	b = strrchr(s,'\\');
    }
    return b ? 1+b : s;
}

void
panic( const char *msg )
{
    fprintf(stderr, "%s: ", prog_name );
    fprintf(stderr, "%s", msg );
    exit(1);
}


/* pmars/rng.c random number generator
 */
s32_t
rng(s32_t seed)
{
    register s32_t temp = seed;
    temp = 16807 * (temp % 127773) - 2836 * (temp / 127773);
    if (temp < 0)
	temp += 2147483647;
    return temp;
}

/* Like realloc(3), but works with NULL pointers. */
void *
xrealloc(void *p, size_t newsize)
{
    if (p==NULL) {
	return malloc(newsize);
    } 
    return realloc(p, newsize);
}

void
usage()
{
    printf("%s v%d.%d", prog_name, VERSION, REVISION);
    if (PATCHLEVEL>0){
	printf(".%d", PATCHLEVEL);
    }
    printf("\n");
    printf("usage: %s [opts] warriors...\n", prog_name );
    printf("opts: -r <rounds>, -c <cycles>, -F <pos>, -s <coresize>,\n"
	   "      -p <maxprocs>, -d <minsep>, -bk\n");
    exit(1);
}

/*---------------------------------------------------------------
 * Warrior initialisations.
 */

void
free_related_memory()
{
    if (Warriors) { free(Warriors); Warriors = NULL; }
    if (Positions) { free(Positions); Positions = NULL; }
    if (StartPositions) { free(StartPositions); StartPositions = NULL; }
    if (Deaths) { free(Deaths); Deaths = NULL; }
    if (Results) {
	if (Results[0]) { free(Results[0]); Results[0] = NULL; }
	free(Results); Results = NULL;
    }
    if (PSpaces) { free(PSpaces); PSpaces = NULL; }
    if (WarriorNames) { free(WarriorNames); WarriorNames = NULL; }
}

void
alloc_related_memory()
{
    unsigned int i;

    Warriors = (warrior_t*)malloc(sizeof(warrior_t)*NWarriors);
    Positions = (field_t*)malloc(sizeof(field_t)*NWarriors);
    StartPositions = (field_t*)malloc(sizeof(field_t)*NWarriors);
    Deaths = (unsigned int*)malloc(sizeof(unsigned int*)*NWarriors);

    /* The results matrix */
    Results = (u32_t**)malloc(sizeof(u32_t*)*NWarriors);
    Results[0] = (u32_t*)malloc(sizeof(u32_t)*NWarriors*(NWarriors+1));
    for (i=1; i<NWarriors; i++) {
	Results[i] = Results[i-1] + NWarriors+1;
    }

    /* Warrior P-spaces */
    PSpaces = (pspace_t **)malloc(sizeof(pspace_t*) * NWarriors);

    if (!(Warriors && Positions && StartPositions && Deaths &&
	  Results && Results[0] && PSpaces))
    {
	free_related_memory();
	panic("Can't allocate memory.");
    }
}


void
import_warriors()
{
    unsigned int i;

    for (i=0; i<NWarriors; i++) {
	asm_fname( WarriorNames[i], &Warriors[i], OPT_coresize );
    }
}

void
check_sanity()
{
    u32_t space_used;
    unsigned int i;

    /* Make sure each warrior has some code. */
    for (i=0; i<NWarriors; i++) {
	if (Warriors[i].len == 0) {
	    sprintf(errmsg,"warrior %d has no code\n", i);
	    panic(errmsg);
	}
    }

    /* Make sure there is some minimum sepation. */
    if ( OPT_minsep == 0 ) {
	OPT_minsep = min( OPT_coresize/NWarriors, MAXLENGTH );
    }

    /* Make sure minsep dominates the lengths of all warriors. */
    for (i=0; i<NWarriors; i++) {
	if ( OPT_minsep < Warriors[i].len ) {
	    panic("minimum separation must be >= longest warrior\n");
	}
    }

    /* Make sure there is space for all warriors to be loaded. */
    space_used = NWarriors*OPT_minsep;
    if ( space_used > OPT_coresize ) {
	panic("warriors or minimum separation too large to fit into core\n");
    }
}


/*---------------------------------------------------------------
 * Warrior positioning algorithms
 *
 * These are pMARS compatible.  Warrior 0 is always positioned at 0.
 * posit() and npos() are transcribed from pmars/pos.c.  */

#define RETRIES1 20                /* how many times to try generating one
				    * position */
#define RETRIES2 4                /* how many times to start backtracking */
int
posit(s32_t *seed)
{
    unsigned int pos = 1, i;
    unsigned int retries1 = RETRIES1, retries2 = RETRIES2;
    int     diff;

    do {
	/* generate */
	*seed = rng(*seed);
	Positions[pos] =
	    (*seed % (OPT_coresize - 2*OPT_minsep+1))+OPT_minsep;

	/* test for overlap */
	for (i = 1; i < pos; ++i) {
	    /* calculate positive difference */
	    diff = (int) Positions[pos] - Positions[i];
	    if (diff < 0)
		diff = -diff;
	    if ((unsigned int)diff < OPT_minsep)
		break;		/* overlap! */
	}

	if (i == pos)		/* no overlap, generate next number */
	    ++pos;
	else {			/* overlap */
	    if (!retries2)
		return 1;	/* exceeded attempts, fail */
	    if (!retries1) {	/* backtrack: generate new sequence starting */
		pos = i;	/* at arbitrary position (last conflict) */
		--retries2;
		retries1 = RETRIES1;
	    } else		/* generate new current number (pos not
                                 * incremented) */
		--retries1;
	}
    } while (pos < NWarriors);
    return 0;
}

void
npos(s32_t *seed)
{
    unsigned int i, j;
    unsigned int temp;
    unsigned int room = OPT_coresize - OPT_minsep*NWarriors+1;

    /* Choose NWarriors-1 positions from the available room. */
    for (i = 1; i < NWarriors; i++) {
	*seed = rng(*seed);
	temp = *seed % room;
	for (j = i - 1; j > 0; j--) {
	    if (temp > Positions[j])
		break;
	    Positions[j+1] = Positions[j];
	}
	Positions[j+1] = temp;
    }

    /* Separate the positions by OPT_minsep cells. */
    temp = OPT_minsep;
    for (i = 1; i < NWarriors; i++) {
	Positions[i] += temp;
	temp += OPT_minsep;
    }

    /* Random permutation of positions. */
    for (i = 1; i < NWarriors; i++) {
	*seed = rng(*seed);
	j = *seed % (NWarriors - i) + i;
	temp = Positions[j];
	Positions[j] = Positions[i];
	Positions[i] = temp;
    }
}

s32_t
compute_positions(s32_t seed)
{
    u32_t avail = OPT_coresize+1 - NWarriors*OPT_minsep;

    Positions[0] = 0;

    /* Case single warrior. */
    if (NWarriors == 1) return seed;

    /* Case one on one. */
    if (NWarriors == 2) {
	Positions[1] = OPT_minsep + seed % avail;
	seed = rng(seed);
	return seed;
    }

    if (NWarriors>2) {
	if (posit(&seed)) {
	    npos(&seed);
	}
    }
    return seed;
}


/*---------------------------------------------------------------
 * Misc.
 */

void
save_pspaces()
{
    pspace_t **ps;
    ps = sim_get_pspaces();
    memcpy( PSpaces, ps, sizeof(pspace_t *)*NWarriors );
}

void
amalgamate_pspaces()
{
    unsigned int i, j;

    /* Share p-space according to PINs. */
    for (i=0; i<NWarriors; i++) {
	if ( Warriors[i].have_pin ) {
	    for (j=0; j<i; j++) {
		if ( Warriors[j].have_pin &&
		     Warriors[j].pin == Warriors[i].pin )
		{
		    pspace_share( PSpaces[i], PSpaces[j] );
		}
	    }
	}
    }
}

void
load_warriors()
{
    unsigned int i;
    for (i=0; i<NWarriors; i++) {
	sim_load_warrior(Positions[i], &Warriors[i].code[0], Warriors[i].len);
    }
}

void
set_starting_order(unsigned int round)
{
    unsigned int i;
    pspace_t **ps;

    /* Copy load positions into starting positions array
       with a cyclic shift of rounds places. */
    for (i=0; i<NWarriors; i++) {
	unsigned int j = (i+round) % NWarriors;
	StartPositions[i] =
	    ( Positions[j] + Warriors[j].start ) % OPT_coresize;
    }

    /* Copy p-spaces into simulator p-space array with a
       cyclic shift of rounds places. */
    ps = sim_get_pspaces();
    for (i=0; i<NWarriors; i++) {
	ps[i] = PSpaces[(i + round) % NWarriors];
    }
}

void
clear_results()
{
    unsigned int i, j;
    for (i=0; i<NWarriors; i++) {
	for (j=0; j<=NWarriors; j++) {
	    Results[i][j] = 0;
	}
    }
}

void
accumulate_results()
{
    unsigned int i;

    /* Fetch the results of the last round from p-space location 0
       that has been updated by the simulator. */
    for (i=0; i<NWarriors; i++) {
	unsigned int result;
	result = pspace_get(PSpaces[i], 0);
	Results[i][result]++;
    }
}


void
output_results()
{
    unsigned int i;
    unsigned int j;

    if (NWarriors == 2 && !OPT_m) {
	printf("%ld %ld\n", Results[0][1], Results[0][2]);
	printf("%ld %ld\n", Results[1][1], Results[1][2]);
    } else {
	for (i=0; i<NWarriors; i++) {
	    for (j=1; j<=NWarriors; j++) {
		printf("%ld ", Results[i][j]);
	    }
	    printf("%ld\n", Results[i][0]);
	}
    }
}


/*---------------------------------------------------------------
 * Command line arguments.
 */

/*
 * parse options
 */
void
readargs( int argc, char **argv )
{
  int n;
  char c;
  int cix;
  int tmp;

  n = 1;
  while ( n < argc ) {
    cix = 0;
    c = argv[n][cix++];
    if ( c == '-' && argv[n][1] ) {
      do {
	c = argv[n][cix++];
	if (c)
	  switch (c) {

	  case 'k': OPT_k = 1; break;
	  case 'b': OPT_b = 1; break;
	  case 'm': OPT_m = 1; break;

	  case 'F':
	    if ( n == argc-1 || !isdigit(argv[n+1][0]) )
	      panic( "bad argument for option -F\n");
	    c = 0;
	    OPT_F = atoi( argv[++n] );
	    break;

	  case 's':
	    if ( n == argc-1 || !isdigit(argv[n+1][0]) )
	      panic( "bad argument for option -s\n");
	    c = 0;
	    OPT_coresize = atoi( argv[++n] );
	    if ( (int)OPT_coresize <= 0 )
	      panic( "core size must be > 0\n");
	    break;

	  case 'd':
	    if ( n == argc-1 || !isdigit(argv[n+1][0]) )
	      panic( "bad argument for option -d\n");
	    c = 0;
	    OPT_minsep = atoi( argv[++n] );
	    if ( (int)OPT_minsep <= 0 )
	      panic( "minimum warrior separation must be > 0\n" );
	    break;

	  case 'p':
	    if ( n == argc-1 || !isdigit(argv[n+1][0]) )
	      panic( "bad argument for option -p\n");
	    c = 0;
	    OPT_processes = atoi( argv[++n] );
	    if ( OPT_processes <= 0 )
	      panic( "max processes must be > 0\n" );
	    break;

	  case 'r':
	    if ( n == argc-1 || !isdigit(argv[n+1][0]) )
	      panic( "bad argument for option -r\n");
	    c = 0;
	    tmp = atoi( argv[++n] );
	    if ( tmp < 0 )
	      panic( "can't do a negative number of rounds!\n" );
	    OPT_rounds = tmp;
	    break;
	  case 'c':
	    if ( n == argc-1 || !isdigit(argv[n+1][0]) )
	      panic( "bad argument for option -c\n");
	    c = 0;
	    OPT_cycles = atoi( argv[++n] );
	    if ( OPT_cycles <= 0 )
	      panic( "cycles must be > 0\n" );
	    break;
	  default:
	    sprintf(errmsg,"unknown option '%c'\n", c);
	    panic(errmsg);
	  }
      } while (c);

    } else /* it's a file name */ {
	++NWarriors;
	WarriorNames = xrealloc(WarriorNames, NWarriors*sizeof(char*));
	WarriorNames[NWarriors-1] = argv[n];
    }
    n++;
  }

  if ( NWarriors == 0 )
    usage();
}



/*-------------------------------------------------------------------------
 * Main
 */


int
main( int argc, char **argv )
{
    unsigned int n;		/* round counter */
    s32_t seed;			/* rnd seed. */

    prog_name = xbasename(argv[0]);
    readargs(argc, argv);
    alloc_related_memory();

    import_warriors();
    check_sanity();
    clear_results();

    seed = OPT_F ? OPT_F-OPT_minsep : rng(time(0)*0x1d872b41);

    /*
     * Allocate simulator buffers and initialise p-spaces.
     */
    if (! sim_alloc_bufs( NWarriors, OPT_coresize, OPT_processes, OPT_cycles))
	panic("can't allocate memory.\n");

    save_pspaces();
    amalgamate_pspaces();	/* Share P-spaces with equal PINs */

    /*
     * Fight OPT_rounds rounds.
     */
    for ( n = 0; n < OPT_rounds; n++ ) {
	int nalive;
	sim_clear_core();

	seed = compute_positions(seed);
	load_warriors();
	set_starting_order(n);

	nalive = sim_mw( NWarriors, &StartPositions[0], &Deaths[0] );
	if (nalive<0)
	    panic("simulator panic!\n");

	accumulate_results();
    }

    sim_free_bufs();
    output_results();
    free_related_memory();
    return 0;
}
