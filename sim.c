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
#include "sim_core_api.h"

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

#define DEF_MAX_WARS 2
#define DEF_CORESIZE 8000
#define DEF_PROCESSES 8000
#define DEF_CYCLES 80000

static unsigned int Coresize   = 0;
static unsigned int Queue_Size = 0;
static unsigned int Processes  = 0;
static unsigned int NWarriors  = 0;
static unsigned int Cycles     = 0;

static sim_warrior_t *War_Tab = NULL;
static core_insn_t   *Core_Mem = NULL;
static void          *Real_Core_Mem = NULL;
static unsigned      *Queue_Mem = NULL;

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

void
sim_clear_core()
{
  sim_core_clear(Core_Mem, Coresize);
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
 *     These functions manage the core, queue, and sim_warrior_t warrior info
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

  Queue_Mem = (unsigned*)malloc( sizeof(unsigned)*(Queue_Size+1) );
  War_Tab = (sim_warrior_t*)malloc( sizeof(sim_warrior_t)*nwars );
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
 *     sim_compile_warrior, sim_load_warrior -- load warrior code into core.
 * 
 * SYNOPSIS
 *     sim_compile_warrior(core_insn_t *output, const insn_t *code, unsigned int len)
 *     sim_load_warrior(unsigned int pos, core_insn_t *code, unsigned int len);
 * 
 * INPUTS
 *     output -- array to write compiled instructions to
 *     pos -- The core address to load the warrior into.
 *     code -- array of instructions.
 *     len -- size of the warrior
 * 
 * DESCRIPTION
 *     These functions are the preferred method to load warriors into core.
 *     Compiling strips the instructions of any flags they may possess, and
 *     converts them into the internal representation.
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

void
sim_compile_warrior(core_insn_t *output, const insn_t *code, unsigned int len)
{
  unsigned int i;

  for (i=0; i<len; i++) {
    output[i].handler = SIM_CORE_INSTR2HANDLER(code[i].in);
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

int
sim_proper( unsigned int nwar, const field_t *war_pos_tab,
	    unsigned int *death_tab )
{
  /*
   * Core and Process queue memories.
   *
   * The warriors share a common cyclic buffer for use as a process
   * queue which the contains core addresses where active processes
   * are.  The buffer has size N*P+1 rounded up to the nearest
   * power of 2, where N = number of warriors, P = maximum number
   * of processes / warrior.
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

  unsigned *queue_start, *queue_end; /* queue mem. start, end */

  unsigned *pqofs;
  u32_t ftmp, t;		/* temps */
  int cycles;			/* instructions to execute before tie counter*/

  /*
   * Setup queue and core pointers
   */
  cycles = nwar * Cycles; /* set instruction executions until tie counter */
  queue_start = Queue_Mem;
  queue_end = Queue_Mem + NWarriors*Processes+1;

  memset(Queue_Mem, 0, sizeof(unsigned)*(Queue_Size+1));

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
    /* The process queue contains actual offsets instead of indexes */
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
  return sim_core_run(Core_Mem, Coresize, Queue_Mem, Queue_Size-1,
                      &War_Tab[nwar-1], cycles, Processes, death_tab,
                      PSpaces, PSpace_size);
}

