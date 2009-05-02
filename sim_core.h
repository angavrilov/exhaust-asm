#ifndef SIM_CORE_H
#define SIM_CORE_H

/* Internal warrior scheduler structure */

typedef struct sim_warrior_st {
  unsigned long head;		/* Index of the next process to run in the queue array */
  unsigned long tail;		/* Index of the next free location to queue a process */
  struct sim_warrior_st *succ;	/* Next active warrior */
  unsigned long nprocs;		/* Number of live processes in this warrior */
  struct sim_warrior_st *pred;	/* Previous active warrior */
  long id;			/* Index (or identity) of the warrior */
} sim_warrior_t;

/* Instructions in core */

typedef struct core_insn_st {
  void *handler;                /* Address of the instruction handler. */
  unsigned a, b;                /* a-value, b-value */
} core_insn_t;

/* Opcode descriptors and decompiling */

typedef struct sim_opcode_dtor_st {
  const unsigned in;
  const unsigned table_idx;
  const unsigned reserved1, reserved2;
} sim_opcode_dtor_t;

#define SIM_CORE_GET_OP_DTOR(handler)  (((sim_opcode_dtor_t*)handler)-1)
#define SIM_CORE_GET_OP_INSTR(handler) (SIM_CORE_GET_OP_DTOR(handler)->in)

#endif
