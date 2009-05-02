#ifndef SIM_CORE_API_H
#define SIM_CORE_API_H

#include "sim_core.h"

/* A table of opcode handler offsets */

extern const signed int sim_core_opcode_handlers[18][7][8][8];

#define SIM_CORE_OFS2HANDLER(ofs) \
          ((void*)(((unsigned long)sim_core_opcode_handlers) + (ofs)))

#define SIM_CORE_FIND_HANDLER(op,mod,mode_a,mode_b) \
          SIM_CORE_OFS2HANDLER(sim_core_opcode_handlers[op][mod][mode_b][mode_a])

#define SIM_CORE_INSTR2HANDLER(in) \
          SIM_CORE_FIND_HANDLER(((in)>>opPOS)&opMASK,((in)>>moPOS)&moMASK, \
                                ((in)>>maPOS)&mMASK,((in)>>mbPOS)&mMASK)

/* Fill the core with DAT.F 0, 0 instructions */

void sim_core_clear(core_insn_t *core, unsigned size);

/* Run the simulation.

   Arguments:
     core               Pointer to the core array. Must be aligned to 16 bytes.
     core_size          Size of the core. 
                          Note: The interpreter may write to core[core_size],
                                so allocate a buffer of appropriate size.

     qptr               Pointer to the process queue array.
                          For efficiency reasons, instead of instruction
                          indexes the queue holds offsets into the core
                          buffer, i.e. idx*sizeof(core_insn_t)
     queue_mask         Mask to use for queue index wrapping; equal to 2^N-1.
                          Note: The interpreter may read qptr[queue_mask+1].

     cur_warrior        Pointer to the warrior that should be executed first.

     cycles             Number of instructions to execute before timing out.
     proc_limit         Maximum number of processes per warrior.

     death_tab          Every time a warrior dies, its ID is added to this array.

     pspaces            Pointer to the pspace table.
     pspace_size        Size of the pspace available to one warrior.

   Return value:
     Number of warriors still alive.
 */

int sim_core_run(core_insn_t *core, long core_size, unsigned *qptr, unsigned long queue_mask,
                 sim_warrior_t *cur_warrior, unsigned long cycles, unsigned long proc_limit,
                 unsigned *death_tab, pspace_t **pspaces, unsigned long pspace_size);

#endif

