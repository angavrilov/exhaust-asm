/* sim.h: prototype
 * $Id: sim.h,v 1.4 2002/05/13 03:05:33 rowan Exp $
 */
#ifndef SIM_H
#define SIM_H

#include "pspace.h"

core_insn_t *sim_alloc_bufs( unsigned int nwar, unsigned int coresize,
			unsigned int processes, unsigned int cycles );

core_insn_t *sim_alloc_bufs2( unsigned int nwar, unsigned int coresize,
			 unsigned int processes, unsigned int cycles,
			 unsigned int pspace );

void sim_free_bufs();

void sim_clear_core(void);


pspace_t **sim_get_pspaces(void);

pspace_t *sim_get_pspace(unsigned int war_id);

void sim_clear_pspaces(void);

void sim_reset_pspaces(void);

void sim_compile_warrior(core_insn_t *output, insn_t const *code, unsigned int len);

int  sim_load_warrior(unsigned int pos, core_insn_t const *code, unsigned int len);



int sim( int nwar_arg, field_t w1_start, field_t w2_start,
	 unsigned int cycles, void **ptr_result );

int sim_mw( unsigned int nwar, const field_t *war_pos_tab,
	    unsigned int *death_tab );

#endif /* SIM_H */
