#ifndef ASM_H 
#define ASM_H
/* asm.h: token codes and macros for them. protos.
 * $Id: asm.h,v 1.6 2002/10/01 21:55:00 rowan Exp $
 */

/* This file is part of `exhaust', a memory array redcode simulator.
 * Author: M Joonas Pihlaja
 * Public Domain.
 */

#include <stdio.h>
/* macros:
 *	is_tok_opcode()		just what it says
 *	is_tok_pseudoop()	
 *	is_tok_modifier()	
 *	is_tok_mode()		is a token an addressing mode
 */

/*
 * asm_line() exit codes:
 *
 */

#define ASMLINE_PIN  -3		/* PIN encountered */
#define ASMLINE_ORG  -2		/* ORG encountered */
#define ASMLINE_DONE -1		/* END encountered */
#define ASMLINE_NONE  0		/* nothing to assemble, or something ignored */
#define ASMLINE_OK    1		/* assembled instruction OK */

/*
 * Token codes for the lexer.
 */
enum {
  TOK_FIRST = 256,		/* not used */

  TOK_OPCODE_START,		/* opcodes */
  TOK_DAT,			/* same order as insn.h opcodes. */
  TOK_SPL,
  TOK_MOV,
  TOK_DJN,
  TOK_ADD,
  TOK_JMZ,
  TOK_SUB,
  TOK_SEQ,
  TOK_SNE,
  TOK_SLT,
  TOK_JMN,
  TOK_JMP,
  TOK_NOP,
  TOK_MUL,
  TOK_MOD,
  TOK_DIV,
  TOK_LDP,
  TOK_STP,			/* 18 */
  TOK_OPCODE_STOP,

  TOK_PSEUDOOP_START,
    TOK_ORG,			/* pseudo-ops and start label */
    TOK_END,
    TOK_PIN,
  TOK_PSEUDOOP_STOP,
  
  TOK_MODIFIER_START,
    TOK_mF, TOK_mA, TOK_mB,	/* modifiers */
    TOK_mAB, TOK_mBA, TOK_mX, TOK_mI,
  TOK_MODIFIER_STOP,

  TOK_MODE_START,
    TOK_DIRECT, TOK_IMMEDIATE,	/* addressing modes */
    TOK_BINDIRECT,  TOK_BPREDEC,
    TOK_BPOSTINC,  TOK_AINDIRECT,
    TOK_APREDEC,  TOK_APOSTINC,	/* 8 */
  TOK_MODE_STOP,

  TOK_START,
  TOK_STR, TOK_INT,		/* string token, int token  */

  TOK_LAST			/* sentinel */
};


#define TOK_ERR TOK_FIRST	/* unused */

/*
 * Macros to identify a token type from a token code.
 */
#define is_tok_opcode(t)   ( t > TOK_OPCODE_START && t < TOK_OPCODE_STOP )
#define is_tok_pseudoop(t) ( t > TOK_PSEUDOOP_START && t < TOK_PSEUDOOP_STOP )
#define is_tok_modifier(t) ( t > TOK_MODIFIER_START && t < TOK_MODIFIER_STOP )
#define is_tok_mode(t)     ( t > TOK_MODE_START && t < TOK_MODE_STOP )


/*
 * protos
 */
int asm_line( const char *line, insn_t *in, unsigned int CORESIZE );
void asm_file( FILE *F, warrior_t *w, unsigned int CORESIZE );
void asm_fname( const char *fname, warrior_t *w, unsigned int CORESIZE );
void dis1( char *s, insn_t in, unsigned int CORESIZE );
void discore( const insn_t *core, int start, int end, unsigned int CORESIZE );


#endif /* ASM_H */
