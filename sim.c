/* sim.c: simulator functions
 * $Id: sim.c,v 1.10 2002/12/28 05:48:16 rowan Exp $
 */

/* This file is part of `exhaust', a memory array redcode simulator.
 * Author: M Joonas Pihlaja
 * Public Domain.
 */

/*
 * Thanks go to the pMARS authors and Ken Espiritu whose ideas have
 * been used in this simulator.  Especially Ken's effective addressing
 * calculation code in pMARS 0.8.6 has been adapted for use here.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "exhaust.h"
#include "asm.h"
#include "sim.h"
#include "insn.h"

/* Should we strip flags from instructions when loading?  By default,
   yes.  If so, then the simulator won't bother masking them off.  */
#ifndef SIM_STRIP_FLAGS
#define SIM_STRIP_FLAGS 1
#endif


/* DEBUG level:
 *     0: none
 *   >=1: disassemble each instruction (no output)
 *     2: print each instruction as it is executed
 */

/*
 * File scoped stuff
 */

/* internal warrior structure */
typedef struct w_st {
  unsigned long head;		/* next process to run from queue */
  unsigned long tail;		/* next free location to queue a process */
  struct w_st *succ;		/* next warrior alive */
  unsigned long nprocs;		/* number of live processes in this warrior */
  struct w_st *pred;		/* previous warrior alive */
  long id;			/* index (or identity) of warrior */
} w_t;

#define DEF_MAX_WARS 2
#define DEF_CORESIZE 8000
#define DEF_PROCESSES 8000
#define DEF_CYCLES 80000

static unsigned int Coresize   = 0;
static unsigned int Queue_Size = 0;
static unsigned int Processes  = 0;
static unsigned int NWarriors  = 0;
static unsigned int Cycles     = 0;

static w_t           *War_Tab = NULL;
static core_insn_t   *Core_Mem = NULL;
static void          *Real_Core_Mem = NULL;
static unsigned long *Queue_Mem = NULL;

/* P-space */
static unsigned int PSpace_size = 0;    /* # p-space slots per warrior. */
static pspace_t	    **PSpaces;	        /* p-spaces of each warrior. */

/* protos */
static int sim_proper( unsigned int, const field_t *, unsigned int * );
static void alloc_pspaces( unsigned int nwars, unsigned int pspacesize );
static void free_pspaces( unsigned int nwars );

/*---------------------------------------------------------------
 * Simulator memory management
 */

void _do_clear_core(core_insn_t *core, unsigned size);

void
sim_clear_core()
{
  _do_clear_core(Core_Mem, Coresize);
}


/* NAME
 *     sim_alloc_bufs, sim_alloc_bufs2, sim_free_bufs --
 *				alloc and free buffers used in simulation
 * 
 * SYNOPSIS
 *     core_insn_t *sim_alloc_bufs( unsigned int nwars, unsigned int coresize,
 *                                  unsigned int processes, unsigned int cycles );
 *     core_insn_t *sim_alloc_bufs2( unsigned int nwars, unsigned int coresize,
 *				     unsigned int processes, unsigned int cycles,
 *				     unsigned int pspacesize );
 *     void sim_free_bufs();
 * 
 * INPUTS
 *     nwar        -- number of warriors
 *     coresize    -- size of core
 *     processes   -- max no of processes / warrior
 *     cycles      -- the number of cycles to play before tie.
 *     pspacesize  -- size of p-space per warrior.  For sim_alloc_bufs(),
 *		      it defaults to min(1,coresize/16).
 * 
 * RESULTS
 *     These functions manage the core, queue, and w_t warrior info
 *     struct array memories.  
 *
 *     Core_Mem, Queue_Mem, War_Tab and PSpace_mem memories are allocated
 *     or freed as requested.  Any earlier memories are freed before
 *     new allocations are made.
 * 
 * RETURN VALUE
 *     sim_alloc_bufs(): the address of core memory, or NULL if
 *			 out of memory.
 *     sim_free_bufs(): none
 *
 * GLOBALS
 *     All file scoped globals.
 */

void
sim_free_bufs()
{
  free_pspaces(NWarriors);
  if ( Core_Mem ) free( Real_Core_Mem ); Core_Mem = NULL; Coresize = 0;
  if ( Queue_Mem ) free( Queue_Mem ); Queue_Mem = NULL; Processes = 0;
  if ( War_Tab ) free( War_Tab ); War_Tab = NULL; NWarriors = 0;
}


core_insn_t *
sim_alloc_bufs2( unsigned int nwars, unsigned int coresize,
		 unsigned int processes, unsigned int cycles,
		 unsigned int pspace )
{
  unsigned int queue_size;

  sim_free_bufs();

  Real_Core_Mem = malloc( sizeof(core_insn_t) * (coresize+2) );
  Core_Mem = (core_insn_t*)(((unsigned long)Real_Core_Mem) & ~15) + 1;

  queue_size = nwars*processes+1;
  Queue_Size = 2;
  while (Queue_Size < queue_size)
    Queue_Size <<= 1;

  Queue_Mem = (unsigned long*)malloc( sizeof(unsigned long)*Queue_Size );
  War_Tab = (w_t*)malloc( sizeof(w_t)*nwars );
  alloc_pspaces(nwars, pspace);

  if ( Core_Mem && Queue_Mem && War_Tab && PSpaces ) {
    Cycles = cycles;
    NWarriors = nwars;
    Coresize = coresize;
    Processes = processes;
    sim_clear_pspaces();
    return Core_Mem;
  }
  sim_free_bufs();
  return NULL;
}

core_insn_t *
sim_alloc_bufs( unsigned int nwars, unsigned int coresize,
		unsigned int processes, unsigned int cycles )
{
  unsigned int pspace;
  pspace = coresize/16 == 0 ? 1 : coresize/16;
  return sim_alloc_bufs2( nwars, coresize, processes, cycles, pspace );
}


/* NAME
 *     sim_load_warrior -- load warrior code into core.
 * 
 * SYNOPSIS
 *     sim_load_warrior(unsigned int pos, core_insn_t *code, unsigned int len);
 * 
 * INPUTS
 *     pos -- The core address to load the warrior into.
 *     code -- array of instructions.
 *     len -- 
 * 
 * DESCRIPTION
 *     This function is the preferred method to load warriors into core.
 *     It strips the instructions of any flags they may possess, and
 *     copies the instructions into core.
 * 
 *     The code will happily overwrite any previous contents of core,
 *     so beware not to load warriors so that their code overlaps.
 * 
 * NOTE
 *     The core must have been allocated previously with sim(), or
 *     preferably sim_alloc_bufs() and not freed since.
 * 
 * RETURN VALUE
 *     0 -- warrior loaded OK.
 *    -1 -- core memory not allocated.
 *    -2 -- warrior length > core size.
 */

extern signed int _opcode_handler_table[18][7][8][8];

void
sim_compile_warrior(core_insn_t *output, const insn_t *code, unsigned int len)
{
  unsigned int i, mode_a, mode_b, modifier;
  u32_t in;
  char buffer[256];
  unsigned long table_base;
  signed int rc;

  table_base = (unsigned long)_opcode_handler_table;

  for (i=0; i<len; i++) {
#if SIM_STRIP_FLAGS
    in = code[i].in & iMASK;
#else
    in = code[i].in;
#endif

    mode_a = in & mMASK;
    in >>= mBITS;
    mode_b = in & mMASK;
    in >>= mBITS;
    modifier = in & moMASK;
    in >>= moBITS;

    rc = _opcode_handler_table[in & opMASK][modifier][mode_b][mode_a];

    if (rc == 0) {
      dis1(buffer, code[i], Coresize);
      printf("Opcode not supported: %s\n", buffer);
      exit(1);
    }

    output[i].in = table_base + rc;
    output[i].a = code[i].a;
    output[i].b = code[i].b;
  }
}

int
sim_load_warrior(unsigned int pos, const core_insn_t *code, unsigned int len)
{
  unsigned int sz1, sz2;

  if ( Core_Mem == NULL )  return -1;
  if ( len > Coresize ) return -2;

  sz1 = Coresize - pos;
  if (sz1 > len)
    sz1 = len;

  memcpy(Core_Mem + pos, code, sz1*sizeof(core_insn_t));

  sz2 = len - sz1;

  if (sz2 > 0)
    memcpy(Core_Mem, code+sz1, sz2*sizeof(core_insn_t));

  return 0;
}


/*---------------------------------------------------------------
 * P-Space management.
 */

static void
free_pspaces(unsigned int nwars)
{
  unsigned int i;
  if ( nwars>0 ) {
    if (PSpaces) {
      for (i=0; i<nwars; i++) {
	pspace_free(PSpaces[i]);
      }
      free(PSpaces);
    }
  }
  PSpace_size = 0;
  PSpaces = NULL;
}


static void
alloc_pspaces(unsigned int nwars, unsigned int pspacesize)
{
  unsigned int i;
  int success = 0;

  PSpaces = NULL;
  PSpace_size = 0;
  if (nwars==0) { return; }
  
  if (( PSpaces = (pspace_t**)malloc(sizeof(pspace_t*)*nwars))) {
    success = 1;
    for (i=0; i<nwars; i++) { PSpaces[i] = NULL; }
    for (i=0; success && i<nwars; i++) {
	PSpaces[i] = pspace_alloc(pspacesize);
	success = success && PSpaces[i] != NULL;
    }
  }

  if ( !success ) {
    free_pspaces(nwars);
  } else {
    PSpace_size = pspacesize;
  }
}


/* NAME
 *     sim_get_pspaces -- get array of p-spaces of warriors.
 *     sim_get_pspace -- get a single p-space.
 * 
 * SYNOPSIS
 *     pspace_t **sim_get_pspaces(void);
 *     pspace_t *sim_get_pspace(unsigned int war_id)
 *
 * DESCRIPTION
 *     Return an array of pointers to p-space structures of the warriors.
 *     The pspace at index i is used as the pspace of the ith warrior
 *     when fighting.
 */
pspace_t **
sim_get_pspaces()
{
    return PSpaces;
}

pspace_t *
sim_get_pspace(unsigned int war_id)
{
    return PSpaces[war_id];
}

/* NAME
 *     sim_clear_pspaces -- clear and/or reinitialise p-spaces
 *     sim_reset_pspaces 
 *
 * SYNOPSIS
 *     void sim_clear_pspaces(void);
 *     void sim_reset_pspaces(void);
 *
 * DESCRIPTION
 *	All p-spaces are cleared and the P-space locations 0 are set
 *	to CORESIZE-1.  For sim_reset_pspaces(): All p-spaces are made
 *	private.
 * */

void
sim_clear_pspaces()
{
  unsigned int i;
  for (i=0; i<NWarriors; i++) {
    pspace_clear(PSpaces[i]);
    pspace_set(PSpaces[i], 0, Coresize-1);
  }
}

void
sim_reset_pspaces()
{
  unsigned int i;
  for (i=0; i<NWarriors; i++) {
    pspace_privatise(PSpaces[i]);
  }
  sim_clear_pspaces();
}



/*---------------------------------------------------------------
 * Simulator interface
 */

/* NAME
 *     sim, sim_mw -- public functions to simulate a round of Core War
 * 
 * SYNOPSIS
 *     int sim_mw( unsigned int nwar, const field_t *war_pos_tab, 
 *                 unsigned int *death_tab );
 *     int sim( int nwar, field_t w1_start, field_t w2_start,
 * 		unsigned int cycles, void **ptr_result );
 * 
 * INPUTS
 *     nwar        -- number of warriors
 *     w1_start, w2_start -- core addresses of first processes
 *                    warrior 1 and warrior 2. Warrior 1 executes first.
 *     cycles      -- the number of cycles to play before tie.
 *     ptr_result  -- NULL, except when requesting the address of core.
 *     war_pos_tab -- core addresses where warriors are loaded in
 *		      the order they are to be executed.
 *     death_tab   -- the table where dead warrior indices are stored
 * 
 * DESCRIPTION
 *     The real simulator is inside sim_proper() to which sim() and
 *     sim_mw() are proxies.  sim_mw() reads the warrior position
 *     of the ith warrior from war_tab_pos[i-1].
 *
 * RESULTS
 *     The warriors fight their fight in core which gets messed up in
 *     the process.  If a warrior died during the fight then its p-space
 *     location 0 is cleared.  Otherwise the number of warriors alive
 *     at the end of the battle is stored into its p-space location 0.
 *
 *     sim_mw() stores indices of warriors that die into the death_tab
 *     array in the order of death.  Warrior indices start from 0.
 *
 *     For sim(): If nwar == -1 then buffers of default size for
 *     max. two warriors are allocated and the address of the core
 *     memory is returned via the ptr_result pointer.
 * 
 * RETURN VALUE
 *     sim_mw(): the number of warriors still alive at the end of the
 *		 battle.
 *       -1: simulator panic attack -- something's gone wrong
 *
 *     sim(): 
 *       single warrior: 0: warrior suicided, 1: warrior didn't die.
 *       one-on-one two warriors:
 * 	      0: warrior 1 won, 1: warrior 2 won, 2: tie
 *       -1: simulator panic attack -- something's gone wrong
 * 
 * GLOBALS
 *     All file scoped globals */

int
sim_mw( unsigned int nwar, const field_t *war_pos_tab,
	unsigned int *death_tab )
{
  int alive_count;
  if ( !Core_Mem || !Queue_Mem || !War_Tab || !PSpaces ) return -1;

  alive_count = sim_proper( nwar, war_pos_tab, death_tab );

  /* Update p-space locations 0. */
  if (alive_count >= 0) {
    unsigned int nalive = alive_count;
    unsigned int i;

    for (i=0; i<nwar; i++) {
      pspace_set( PSpaces[i], 0, nalive);
    }
    for (i=0; i<nwar-nalive; i++) {
      pspace_set( PSpaces[death_tab[i]], 0, 0);
    }
  }
  return alive_count;
}


int
sim( int nwar,
     field_t w1_start,
     field_t w2_start,
     unsigned int cycles,
     void **ptr_result )
{
  field_t war_pos_tab[2];
  unsigned int death_tab[2];
  int alive_cnt;

  /* if the caller requests for the address of core, allocate 
   * the default buffers and give it
   */
  if ( nwar < 0 ) {
    if ( nwar == -1 && ptr_result ) {
      *ptr_result = sim_alloc_bufs( DEF_MAX_WARS, DEF_CORESIZE,
				    DEF_PROCESSES, DEF_CYCLES );
      return 0;
    }
    return -1;
  }
  if ( nwar > 2 ) return -1;

  /* otherwise set up things for sim_mw() */
  Cycles = cycles;
  war_pos_tab[0] = w1_start;
  war_pos_tab[1] = w2_start;

  alive_cnt = sim_mw( nwar, war_pos_tab, death_tab );
  if ( alive_cnt < 0 ) return -1;

  if ( nwar == 1) return alive_cnt;

  if ( alive_cnt == 2 ) return 2;
  return death_tab[0] == 0 ? 1 : 0;
}



/*-------------------------------------------------------------------------
 * private functions
 */

/* NAME
 *     sim_proper -- the real simulator code
 * 
 * SYNOPSIS
 *     int sim_proper( unsigned int nwar,
 *                     const field_t *war_pos_tab,
 *                     unsigned int *death_tab );
 * 
 * INPUTS
 *     nwar        -- number of warriors
 *     war_pos_tab -- core addresses where warriors are loaded in
 *		      the order they are to be executed.
 *     death_tab   -- the table where dead warrior indices are stored
 * 
 * RESULTS
 *     The warriors fight their fight in core which gets messed up in
 *     the process.  The indices of warriors that die are stored into
 *     the death_tab[] array in the order of death.  Warrior indices
 *     start from 0.
 * 
 * RETURN VALUE
 *     The number of warriors still alive at the end of the
 *     battle or -1 on an anomalous condition.
 * 
 * GLOBALS
 *     All file scoped globals
 */

/* Various macros:
 *
 *  queue(x): Inserts a core address 'x' to the head of the current
 *            warrior's process queue.  Assumes the warrior's
 * 	      tail pointer is inside the queue buffer.
 *
 * x, y must be in 0..coresize-1 for the following macros:
 *
 * INCMOD(x): x = x+1 mod coresize
 * DECMOD(x): x = x-1 mod coresize
 * ADDMOD(z,x,y): z = x+y mod coresize
 * SUBMOD(z,x,y): z = x-y mod coresize
 */

#define queue(x)  do { *w->tail++ = (x); if ( w->tail == queue_end )\
                                          w->tail = queue_start; } while (0)

#define INCMOD(x) do { if ( ++(x) == coresize ) (x) = 0; } while (0)
#define DECMOD(x) do { if ((x)-- == 0) (x) = coresize1; } while (0)
#define ADDMOD(z,x,y) do { (z) = (x)+(y); if ((z)>=coresize) (z) -= coresize; } while (0)
#define SUBMOD(z,x,y) do { (z) = (x)-(y); if ((int)(z)<0) (z) += coresize; } while (0)

/* private macros to access p-space. */
#define UNSAFE_PSPACE_SET(warid, paddr, val) do {\
    if (paddr) {\
	pspaces_[(warid)]->mem[(paddr)] = (val);\
    } else {\
	pspaces_[(warid)]->lastresult = (val);\
    }\
} while(0)

#define UNSAFE_PSPACE_GET(warid, paddr) \
	( (paddr) ? pspaces_[(warid)]->mem[(paddr)]\
		  : pspaces_[(warid)]->lastresult )

int _do_simulate(core_insn_t *core, long core_size, unsigned long *qptr, unsigned long queue_mask,
                 w_t *cur_warrior, unsigned long cycles, unsigned long proc_limit, unsigned *death_tab);


int
sim_proper( unsigned int nwar, const field_t *war_pos_tab,
	    unsigned int *death_tab )
{
  /*
   * Core and Process queue memories.
   *
   * The warriors share a common cyclic buffer for use as a process
   * queue which the contains core addresses where active processes
   * are.  The buffer has size N*P+1, where N = number of warriors,
   * P = maximum number of processes / warrior.
   *
   * Each warrior has a fixed slice of the buffer for its own process
   * queue which are initially allocated to the warriors in reverse
   * order. i.e. if the are N warriors w1, w2, ..., wN, the slice for
   * wN is 0..P-1, w{N-1} has P..2P-1, until w1 has (N-1)P..NP-1.
   *
   * The core address of the instruction is fetched from the head of
   * the process queue and processes are pushed to the tail, so the
   * individual slices slide along at one location per executed
   * instruction.  The extra '+1' in the buffer size is to have free
   * space to slide the slices along.
   * 
   * For two warriors w1, w2:
   *
   * |\......../|\......../| |
   * | w2 queue | w1 queue | |
   * 0          P         2P 2P+1
   */

  core_insn_t *core;
  unsigned long *queue_start, *queue_end; /* queue mem. start, end */

  /*
   * Cache Registers.
   *
   * The '94 draft specifies that the redcode processor model be
   * 'in-register'.  That is, the current instruction and the
   * instructions at the effective addresses (ea's) be cached in
   * registers during instruction execution, rather than have
   * core memory accessed directly when the operands are needed.  This
   * causes differences from the 'in-memory' model.  e.g. MOV 0,>0
   * doesn't change the instruction's b-field since the instruction at
   * the a-field's effective address (i.e. the instruction itself) was
   * cached before the post-increment happened.
   *
   * There are conceptually three registers: IN, A, and B.  IN is the
   * current instruction, and A, B are the ones at the a- and
   * b-fields' effective addresses respectively.
   *
   * We don't actually cache the complete instructions, but rather
   * only the *values* of their a- and b-field.  This is because
   * currently there is no way effective address computations can
   * modify the opcode, modifier, or addressing modes of an
   * instruction.
   */

  u32_t in;			/* 'in' field of current insn for decoding */
  u32_t ra_a, ra_b,		/* A register values */
        rb_a, rb_b;		/* B register values */
  u32_t da, db;			/* effective address of instruction's
				 * a- and b-fields */
  /* 
   * alias some register names to real variables
   */
#define in_a ra_a
#define in_b rb_b

  unsigned *pofs;
  unsigned long *pqofs;
  core_insn_t *pt;
  unsigned int mode;

  /*
   * misc.
   */
  w_t *w;			/* current warrior */
  u32_t ip;			/* current instruction pointer */
  u32_t ftmp, t;		/* temps */
  unsigned int coresize, coresize1; /* size of core, size of core - 1 */
  int cycles;			/* instructions to execute before tie counter*/
  int alive_cnt;
  pspace_t **pspaces_;
  u32_t pspacesize;

#if DEBUG >= 1
  core_insn_t insn;		/* used for disassembly */
  char debug_line[256];		/* ditto */
#endif


  /*
   * Setup queue and core pointers
   */
  alive_cnt = nwar;
  cycles = nwar * Cycles; /* set instruction executions until tie counter */
  coresize = Coresize;
  coresize1 = Coresize-1;
  core = Core_Mem;
  queue_start = Queue_Mem;
  queue_end = Queue_Mem + NWarriors*Processes+1;
  pspaces_ = PSpaces;
  pspacesize = PSpace_size;

  /* Setup War_Tab and links all around */
  pqofs = queue_end-1;		/* init. wars[] table */
  War_Tab[0].succ = &War_Tab[nwar-1];
  War_Tab[nwar-1].pred = &War_Tab[0];
  t = nwar-1;
  ftmp = 0;
  do {
    if ( t > 0 ) War_Tab[t].succ = &War_Tab[t-1];
    if ( t < nwar-1 ) War_Tab[t].pred = &War_Tab[t+1];
    pqofs -= Processes;
    *pqofs = war_pos_tab[ftmp]*sizeof(core_insn_t);
    War_Tab[t].head = (pqofs-queue_start);
    War_Tab[t].tail = (pqofs-queue_start)+1;
    War_Tab[t].nprocs = 1;
    War_Tab[t].id = ftmp;
    --t;
    ftmp++;
  } while ( ftmp < nwar );


  /*
   * Main loop is run for each executed instruction
   */
  w = &War_Tab[ nwar-1 ];

  return _do_simulate(core, coresize*sizeof(core_insn_t), queue_start, Queue_Size-1, w, cycles, Processes, death_tab);

#if 0
  do {

    ip = *w->head++;		/* load current instruction */
    if ( w->head == queue_end ) w->head = queue_start;
    in = core[ ip ].in;		/* note: flags must be unset! */
#if !SIM_STRIP_FLAGS
    in = in & iMASK;		/* strip flags. */
#endif
    in_a = core[ ip ].a;
    in_b = core[ ip ].b;

#if DEBUG >= 1
    insn = core[ip];
    dis1( debug_line, insn, coresize);
#endif

    /*
     * a-field effective address calculation.
     *
     * This bit was assimilated from Ken Espiritu's 
     * pmars 0.8.6 patch.  Ditto for the b-field 
     * calculation.
     */
    mode = in & mMASK;
    in >>= mBITS;		/* shift out a-mode bits */
    rb_a = in_a;		/* necessary if B-mode is immediate */

    if ( mode != IMMEDIATE ) {
      ADDMOD( da, in_a, ip );
      pt = core+da;		/* pt is where field points to (without */
				/* indirection). */

      if ( mode != DIRECT ) {
	if ( INDIR_A(mode) ) {
	  mode = RAW_MODE(mode);
	  pofs = &(pt->a);
	} else
	  pofs = &(pt->b);

	t = *pofs;		/* pofs is the indirection offset */
	if (mode == PREDEC) {
	  DECMOD(t);
	  *pofs = t;
	}
	ADDMOD(da, t, da);

	pt = core+da;		/* now pt is the final destination */
				/* of indiretion. */
	ra_a = pt->a;		/* read in registers */
	ra_b = pt->b;

	if (mode == POSTINC) {
	  INCMOD(t);
	  *pofs = t;
	}
      } else /* DIRECT */{
	ra_a = pt->a;
	ra_b = pt->b;
      }
    } else /* IMMEDIATE */ {
      ra_b = in_b;
      da = ip;
    }


    /*
     * b-field effective address calculation
     */
    mode = in & mMASK;
    in >>= mBITS;		/* shift out b-mode bits */

    if ( mode != IMMEDIATE ) {
      ADDMOD( db, in_b, ip );
      pt = core+db;

      if ( mode != DIRECT ) {
	if ( INDIR_A(mode) ) {
	  mode = RAW_MODE(mode);
	  pofs = &(pt->a);
	} else
	  pofs = &(pt->b);

	t = *pofs;
	if (mode == PREDEC) {
	  DECMOD(t);
	  *pofs = t;
	}
	ADDMOD(db, t, db);

	pt = core+db;
	rb_a = pt->a;
	rb_b = pt->b;

	if (mode == POSTINC) {
	  INCMOD(t);
	  *pofs = t;
	}
      } else /* DIRECT */{
	rb_a = pt->a;
	rb_b = pt->b;
      }
    } else /* IMMEDIATE */ {
      db = ip;
    }

#if DEBUG == 2
    /* Debug output */
    printf("%6d %4ld  %s  |%4ld, d %4ld,%4ld a %4ld,%4ld b %4ld,%4ld\n",
	   cycles, ip, debug_line,
	   w->nprocs, da, db, 
	   ra_a, ra_b, rb_a, rb_b );
#endif

    /*
     * Execute the instruction on opcode.modifier
     */
    switch ( in ) {

    case _OP(DAT, mA):
    case _OP(DAT, mB):
    case _OP(DAT, mAB):
    case _OP(DAT, mBA):
    case _OP(DAT, mX):
    case _OP(DAT, mF):
    case _OP(DAT, mI):
    die:
      if ( --w->nprocs > 0 )
	goto noqueue;
      w->pred->succ = w->succ;
      w->succ->pred = w->pred;
      *death_tab++ = w->id;
      cycles = cycles - cycles/alive_cnt; /* nC+k -> (n-1)C+k */
      if ( --alive_cnt <= 1 ) 
	goto out;
      goto noqueue;


    case _OP(SPL, mA):
    case _OP(SPL, mB):
    case _OP(SPL, mAB):
    case _OP(SPL, mBA):
    case _OP(SPL, mX):
    case _OP(SPL, mF):
    case _OP(SPL, mI):
      INCMOD(ip);
      queue(ip);
      if ( w->nprocs < Processes ) {
	++w->nprocs;
	queue(da);
      }
      goto noqueue;


    case _OP(MOV, mA):
      core[db].a = ra_a;
      break;
    case _OP(MOV, mF):
      core[db].a = ra_a;
    case _OP(MOV, mB):
      core[db].b = ra_b;
      break;
    case _OP(MOV, mAB):
      core[db].b = ra_a;
      break;
    case _OP(MOV, mX):
      core[db].b = ra_a;
    case _OP(MOV, mBA):
      core[db].a = ra_b;
      break;
    case _OP(MOV, mI):
      core[db].a = ra_a;
      core[db].b = ra_b;
      core[db].in = core[da].in;
      break;


    case _OP(DJN,mBA):
    case _OP(DJN,mA):
      t = core[db].a;
      DECMOD(t);
      core[db].a = t;
      if ( rb_a == 1 ) break;
      queue(da);
      goto noqueue;

    case _OP(DJN,mAB):
    case _OP(DJN,mB):
      t = core[db].b;
      DECMOD(t);
      core[db].b = t;
      if ( rb_b == 1 ) break;
      queue(da);
      goto noqueue;

    case _OP(DJN,mX):
    case _OP(DJN,mI):
    case _OP(DJN,mF):
      t = core[db].a;      DECMOD(t);      core[db].a = t;
      t = core[db].b;      DECMOD(t);      core[db].b = t;
      if ( rb_a == 1 && rb_b == 1 ) break;
      queue(da);
      goto noqueue;


    case _OP(ADD, mI):
    case _OP(ADD, mF):
      ADDMOD( t, ra_b, rb_b ); core[db].b = t;
    case _OP(ADD, mA):
      ADDMOD( t, ra_a, rb_a ); core[db].a = t; 
      break;
    case _OP(ADD, mB):
      ADDMOD( t, ra_b, rb_b ); core[db].b = t;
      break;
    case _OP(ADD, mX):
      ADDMOD( t, ra_b, rb_a ); core[db].a = t;
    case _OP(ADD, mAB):
      ADDMOD( t, ra_a, rb_b ); core[db].b = t;
      break;
    case _OP(ADD, mBA):
      ADDMOD( t, ra_b, rb_a ); core[db].a = t;
      break;


    case _OP(JMZ, mBA):
    case _OP(JMZ, mA):
      if ( rb_a )
	break;
      queue(da);
      goto noqueue;

    case _OP(JMZ, mAB):
    case _OP(JMZ, mB):
      if ( rb_b )
	break;
      queue(da);
      goto noqueue;

    case _OP(JMZ, mX):
    case _OP(JMZ, mF):
    case _OP(JMZ, mI):
      if ( rb_a || rb_b )
	break;
      queue(da);
      goto noqueue;


    case _OP(SUB, mI):
    case _OP(SUB, mF):
      SUBMOD( t, rb_b, ra_b ); core[db].b = t;
    case _OP(SUB, mA):
      SUBMOD( t, rb_a, ra_a); core[db].a = t;
      break;
    case _OP(SUB, mB):
      SUBMOD( t, rb_b, ra_b ); core[db].b = t;
      break;
    case _OP(SUB, mX):
      SUBMOD( t, rb_a, ra_b ); core[db].a = t;
    case _OP(SUB, mAB):
      SUBMOD( t, rb_b, ra_a ); core[db].b = t;
      break;
    case _OP(SUB, mBA):
      SUBMOD( t, rb_a, ra_b ); core[db].a = t;
      break;


    case _OP(SEQ, mA):
      if ( ra_a == rb_a )
	INCMOD(ip);
      break;
    case _OP(SEQ, mB):
      if ( ra_b == rb_b )
	INCMOD(ip);
      break;
    case _OP(SEQ, mAB):
      if ( ra_a == rb_b )
	INCMOD(ip);
      break;
    case _OP(SEQ, mBA):
      if ( ra_b == rb_a )
	INCMOD(ip);
      break;

    case _OP(SEQ, mI):
#if !SIM_STRIP_FLAGS
#define mask  (1<<(flPOS)-1)
      if ( core[da].in & bitmask != core[db].in & bitmask )
	break;
#else
      if ( core[da].in != core[db].in )
	break;
#endif
      /* fallthrough */
    case _OP(SEQ, mF):
      if ( ra_a == rb_a && ra_b == rb_b )
	INCMOD(ip);
      break;
    case _OP(SEQ, mX):
      if ( ra_a == rb_b && ra_b == rb_a )
	INCMOD(ip);
      break;


    case _OP(SNE, mA):
      if ( ra_a != rb_a )
	INCMOD(ip);
      break;
    case _OP(SNE, mB):
      if ( ra_b != rb_b )
	INCMOD(ip);
      break;
    case _OP(SNE, mAB):
      if ( ra_a != rb_b )
	INCMOD(ip);
      break;
    case _OP(SNE, mBA):
      if ( ra_b != rb_a )
	INCMOD(ip);
      break;

    case _OP(SNE, mI):
      if ( core[da].in != core[db].in ) {
	INCMOD(ip);
	break;
      }
      /* fall through */
    case _OP(SNE, mF):
      if ( ra_a != rb_a || ra_b != rb_b )
	INCMOD(ip);
      break;
    case _OP(SNE, mX):
      if ( ra_a != rb_b || ra_b != rb_a )
	INCMOD(ip);
      break;


    case _OP(JMN, mBA):
    case _OP(JMN, mA):
      if (! rb_a )
	break;
      queue(da);
      goto noqueue;

    case _OP(JMN, mAB):
    case _OP(JMN, mB):
      if (! rb_b )
	break;
      queue(da);
      goto noqueue;

    case _OP(JMN, mX):
    case _OP(JMN, mF):
    case _OP(JMN, mI):
      if (!rb_a && !rb_b)
	break;
      queue(da);
      goto noqueue;


    case _OP(JMP, mA):
    case _OP(JMP, mB):
    case _OP(JMP, mAB):
    case _OP(JMP, mBA):
    case _OP(JMP, mX):
    case _OP(JMP, mF):
    case _OP(JMP, mI):
      queue(da);
      goto noqueue;



    case _OP(SLT, mA):
      if (ra_a < rb_a)
	INCMOD(ip);
      break;
    case _OP(SLT, mAB):
      if (ra_a < rb_b)
	INCMOD(ip);
      break;
    case _OP(SLT, mB):
      if (ra_b < rb_b)
	INCMOD(ip);
      break;
    case _OP(SLT, mBA):
      if (ra_b < rb_a)
	INCMOD(ip);
      break;
    case _OP(SLT, mI):
    case _OP(SLT, mF):
      if (ra_a < rb_a && ra_b < rb_b)
	INCMOD(ip);
      break;
    case _OP(SLT, mX):
      if (ra_a < rb_b && ra_b < rb_a)
	INCMOD(ip);
      break;


    case _OP(MODM, mI):
    case _OP(MODM, mF):
      ftmp = 0;			/* we must try to do both modulos even if
				   one fails */
      if ( ra_a ) core[db].a = rb_a % ra_a; else ftmp = 1;
      if ( ra_b ) core[db].b = rb_b % ra_b; else ftmp = 1;
      if ( ftmp ) goto die;
      break;
    case _OP(MODM, mX):
      ftmp = 0;			/* we must try to do both modulos even if
				   one fails */
      if ( ra_b ) core[db].a = rb_a%ra_b; else ftmp = 1;
      if ( ra_a ) core[db].b = rb_b%ra_a; else ftmp = 1;
      if ( ftmp ) goto die;
      break;
    case _OP(MODM, mA):
      if ( !ra_a ) goto die;
      core[db].a = rb_a % ra_a;
      break;
    case _OP(MODM, mB):
      if ( !ra_b ) goto die;
      core[db].b = rb_b % ra_b;
      break;
    case _OP(MODM, mAB):
      if ( !ra_a ) goto die;
      core[db].b = rb_b % ra_a;
      break;
    case _OP(MODM, mBA):
      if ( !ra_b ) goto die;
      core[db].a = rb_a % ra_b;
      break;


    case _OP(MUL, mI):
    case _OP(MUL, mF):
      core[db].b = (rb_b * ra_b) % coresize;
    case _OP(MUL, mA):
      core[db].a = (rb_a * ra_a) % coresize;
      break;
    case _OP(MUL, mB):
      core[db].b = (rb_b * ra_b) % coresize;
      break;
    case _OP(MUL, mX):
      core[db].a = (rb_a * ra_b) % coresize;
    case _OP(MUL, mAB):
      core[db].b = (rb_b * ra_a) % coresize;
      break;
    case _OP(MUL, mBA):
      core[db].a = (rb_a * ra_b) % coresize;
      break;


    case _OP(DIV, mI):
    case _OP(DIV, mF):
      ftmp = 0;			/* we must try to do both divisions even if
				   one fails */
      if ( ra_a ) core[db].a = rb_a / ra_a; else ftmp = 1;
      if ( ra_b ) core[db].b = rb_b / ra_b; else ftmp = 1;
      if ( ftmp ) goto die;
      break;
    case _OP(DIV, mX):
      ftmp = 0;			/* we must try to do both divisions even if
				   one fails */
      if ( ra_b ) core[db].a = rb_a / ra_b; else ftmp = 1;
      if ( ra_a ) core[db].b = rb_b / ra_a; else ftmp = 1;
      if ( ftmp ) goto die;
      break;
    case _OP(DIV, mA):
      if ( !ra_a ) goto die;
      core[db].a = rb_a / ra_a;
      break;
    case _OP(DIV, mB):
      if ( !ra_b ) goto die;
      core[db].b = rb_b / ra_b;
      break;
    case _OP(DIV, mAB):
      if ( !ra_a ) goto die;
      core[db].b = rb_b / ra_a;
      break;
    case _OP(DIV, mBA):
      if ( !ra_b ) goto die;
      core[db].a = rb_a / ra_b;
      break;


    case _OP(NOP,mI):
    case _OP(NOP,mX):
    case _OP(NOP,mF):
    case _OP(NOP,mA):
    case _OP(NOP,mAB):
    case _OP(NOP,mB):
    case _OP(NOP,mBA):
      break;

    case _OP(LDP,mA):
      ftmp = ra_a % pspacesize;
      core[db].a = UNSAFE_PSPACE_GET(w->id, ftmp);
      break;
    case _OP(LDP,mAB):
      ftmp = ra_a % pspacesize;
      core[db].b = UNSAFE_PSPACE_GET(w->id, ftmp);
      break;
    case _OP(LDP,mBA):
      ftmp = ra_b % pspacesize;
      core[db].a = UNSAFE_PSPACE_GET(w->id, ftmp);
      break;
    case _OP(LDP,mF):
    case _OP(LDP,mX):
    case _OP(LDP,mI):
    case _OP(LDP,mB):
      ftmp = ra_b % pspacesize;
      core[db].b = UNSAFE_PSPACE_GET(w->id, ftmp);
      break;

    case _OP(STP,mA):
      ftmp = rb_a % pspacesize;
      UNSAFE_PSPACE_SET(w->id, ftmp, ra_a);
      break;
    case _OP(STP,mAB):
      ftmp = rb_b % pspacesize;
      UNSAFE_PSPACE_SET(w->id, ftmp, ra_a);
      break;
    case _OP(STP,mBA):
      ftmp = rb_a % pspacesize;
      UNSAFE_PSPACE_SET(w->id, ftmp, ra_b);
      break;
    case _OP(STP,mF):
    case _OP(STP,mX):
    case _OP(STP,mI):
    case _OP(STP,mB):
      ftmp = rb_b % pspacesize;
      UNSAFE_PSPACE_SET(w->id, ftmp, ra_b);
      break;

#if DEBUG > 0
    default:
      alive_cnt = -1;
      goto out;
#endif
    }

    INCMOD(ip);
    queue(ip);
  noqueue:
    w = w->succ;
  } while(--cycles>0);

 out:
#if DEBUG == 1
  printf("cycles: %d\n", cycles);
#endif
  return alive_cnt;
#endif
}
