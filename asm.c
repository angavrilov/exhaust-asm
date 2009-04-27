/* asm.c: primitive redcode assembler
 * $Id: asm.c,v 1.8 2002/10/01 22:24:46 rowan Exp $
 */

/* This file is part of `exhaust', a memory array redcode simulator.
 * Author: M Joonas Pihlaja
 * Public Domain.
 */

/* The format of lines with instructions should be:
 *
 * [START]	OPCODE.MODIFIER   A-MODE INT , B-MODE INT
 *
 * The ORG pseudo-op is ignored, as is the label after an optional
 * END (if given). The only label recognised is START.  No fuss over
 * the amount of white space, as long as it exists where required.
 * 
 * Comments are recognised and discarded as is any line starting
 * with "Program".  The output from `pmars -r 0 Your_Real_Source.red'
 * should assemble fine with this tiny assembler.
 *
 *
 * Functions in this file:
 *     
 *     asm_line(), asm_file(), asm_fname(), dis1(),
 *     discore()
 */
#include <stdio.h>
#ifdef SYSV
#include <strings.h>
#else
#include <string.h>
#endif
#include <stdlib.h>
#include <ctype.h>

#include "exhaust.h"
#include "insn.h"
#include "asm.h"

/* str_tok_t: container for tokens we identify.
 */
typedef struct str_toks_st {
  char *s;			/* name of token */
  int c;			/* token code */
} str_toks_t;

/* Data
 *
 * tok_buf[]: globally used to keep the contents of string tokens
 * tok_int:   if the token was a TOK_INT, the value of the token is here
 *
 * str_toks[]: table of multicharacter tokens we identify
 *
 */

#define MAX_ALL_CHARS 256
static char tok_buf[MAX_ALL_CHARS];
static int tok_int;

static str_toks_t str_toks[] = {
    { "DAT", TOK_DAT },		/* opcodes */
    { "SPL", TOK_SPL },
    { "MOV", TOK_MOV },
    { "DJN", TOK_DJN },
    { "ADD", TOK_ADD },
    { "JMZ", TOK_JMZ },
    { "SUB", TOK_SUB },
    { "MOD", TOK_MOD },
    { "CMP", TOK_SEQ },
    { "SEQ", TOK_SEQ },
    { "JMP", TOK_JMP },
    { "JMN", TOK_JMN },
    { "SNE", TOK_SNE },
    { "MUL", TOK_MUL },
    { "DIV", TOK_DIV },
    { "SLT", TOK_SLT },
    { "NOP", TOK_NOP },
    { "LDP", TOK_LDP },
    { "STP", TOK_STP },

    { "ORG", TOK_ORG },		/* pseudo-ops */
    { "END", TOK_END },
    { "PIN", TOK_PIN },
    { "START", TOK_START },

    { "F", TOK_mF },		/* modifiers */
    { "A", TOK_mA },
    { "B", TOK_mB },
    { "AB", TOK_mAB },
    { "BA", TOK_mBA },
    { "X", TOK_mX },
    { "I", TOK_mI },
    { NULL, 0 }			/* sentinel */
};



/* NAME
 *     get_tok -- read the next token from a string
 * 
 * SYNOPSIS
 *     const char *get_tok( const char *s, int *tok );
 * 
 * INPUTS
 *     s -- string to read token from
 *     tok -- where we store the token code of the read token
 * 
 * RESULTS
 *     The token code of the read token is stored into *tok,
 *     with 0 signifying end of input.
 *
 *     If the token was an integer, its value is stored into
 *     the global `tok_int'.  Integers may be in any base >= 10
 *     as according to strtol().
 *
 *     String tokens are converted to upper case when storing
 *     them into the global `tok_str[]'.  They are concatenated
 *     at 255 characters.
 * 
 * RETURN VALUE
 *      Pointer to the character past the read token, or 
 *	to the nul character if at end of input.
 * 
 * GLOBALS
 *     tok_buf[]    -- a string or char token is copied here
 *     tok_int      -- the value of an integer token
 *     str_toks[]   -- used to identify string tokens
 */

/* skip_white(): returns ptr. to next non-whitespace char in s */
static
const char *
skip_white(const char *s)
{
  while ( isspace(*s) ) s++;
  return s;
}

static
const char *
get_tok( const char *s, int *tok )
{
  char *tok_str = tok_buf;
  int i;

  s = skip_white(s);
  if ( *s == 0 )    return (*tok = 0, s);

  /*
   * Tokenize strings.
   *
   * String tokens must start with a letter and consist of
   * letters, digits, and underscores.  Strings are
   * converted to upper case.
   */
  tok_buf[1] = tok_buf[0] = 0;

  i = 0;
  if ( isalpha(*s) )
    while ( (isalnum(*s) || *s == '_') && ++i < MAX_ALL_CHARS )
      *tok_str++ = toupper(*s++);
  *tok_str = 0;

  if ( tok_str > tok_buf ) {
    /*
     * was a string token -- identify it by searching through
     * the str_toks[] array.
     */
    for ( i = 0; str_toks[i].s ; i++ ) {
      if ( 0 == strcmp( str_toks[i].s, tok_buf ) ) {
	*tok = str_toks[i].c;
	return s;
      }
    }
    *tok = TOK_STR;		/* normal string, not special */
    return s;
  }


  /*
   * Tokenize ints.
   * Must match /-?[0-9]/
   */
  if ( isdigit(*s) ||  ( *s == '-' && isdigit(*(s+1)) )) {
    char *endptr;
    tok_int = strtol( s, &endptr, 0 );
    *tok = TOK_INT;
    return endptr;
  }


  /*
   * Tokenize addressing modes and pass single chars
   */

  tok_buf[0] = *s;		/* store char value as single */
  tok_buf[1] = 0;		/* char string. */

  switch ( *tok = *s++ ) {
  case '$': *tok = TOK_DIRECT;          break;
  case '#': *tok = TOK_IMMEDIATE;	break;
  case '*': *tok = TOK_AINDIRECT;	break;
  case '@': *tok = TOK_BINDIRECT;	break;
  case '{': *tok = TOK_APREDEC;		break;
  case '<': *tok = TOK_BPREDEC;		break;
  case '}': *tok = TOK_APOSTINC;	break;
  case '>': *tok = TOK_BPOSTINC;	break;
  }

  return s;
}



/* NAME
 *     panic_bad_token -- issue an error message for a bad token and exit(1)
 * 
 * SYNOPSIS
 *     void panic_bad_token( int tok, const char *expected );
 * 
 * INPUTS
 *     tok -- token code of unexpected token
 *     expected -- a string describing what kind of token
 *		   was expected.  e.g. "a modifier".
 * 
 * RESULTS
 *     A message Informing the user of the unexpected token,
 *     its possible semantic value, and what type of token
 *     was expected instead.
 * 
 * GLOBALS
 *     tok_buf, tok_int -- if the token has semantic value we look
 *                         for it here.
 * BUGS
 *     The error message should be much better -- not even location
 *     in the source is given here. *sigh*
 */
static
void
panic_bad_token( int tok, const char *expected ) 
{
  char *errstr = NULL;
  char buf[30];

  memset(buf, 0, 30);

  /* make an errstr
   */
  if ( tok_buf[0] )
    errstr = tok_buf;
  if ( tok == TOK_INT ) {
    sprintf(buf, "%d", tok_int );
    errstr = buf;
  }

  /* complain and exit with error code
   */
  fprintf(stderr, "token '%s' not %s\n", errstr, expected );
  exit(1);
}


/* NAME
 *     asm_line -- assemble a line to an instruction
 * 
 * SYNOPSIS
 *     int asm_line( const char *line, insn_t *in, unsigned int CORESIZE );
 * 
 * INPUTS
 *     line -- line to assemble
 *     in   -- instruction to assemble into
 *     CORESIZE -- size of core
 * 
 * RESULTS
 *     If there was anything to assemble, it is assembled into
 *     `in'.  If there was a START label, the corresponding flag
 *     is set in the instructions flags.  Incomplete or erroneous
 *     input prompt a quick error message and exit(1).
 *
 *     If the 'ORG start-address' construct is encountered where
 *     `start-address' is an integer, then the `in->a' field contains
 *     the offset in instructions from the start of the warrior
 *     where the warrior should start execution.
 *
 *     If 'PIN id' is encountered, where `id' is an integer, then the
 *     `in->a' field contains the `id'.
 * 
 * RETURN VALUE
 *     ASMLINE_PIN  : pseudo-op 'PIN' encountered, id saved in `in->a'.
 *     ASMLINE_ORG  : pseudo-op 'ORG' encountered, warrior start
 *                    saved in `in->a'.
 *     ASMLINE_DONE : done assembling, END opcode found, nothing assembled.
 *     ASMLINE_NONE : nothing to assemble on this line.
 *     ASMLINE_OK   : assembled instruction into `in' OK.
 * 
 * GLOBALS
 *     tok_int, tok_buf[], str_toks[] somewhere down the line.
 */

int
asm_line( const char *line, insn_t *in, unsigned int CORESIZE  )
{
  const char *s = line;
  int tok;
  int flags = 0;
  int op, m, ma, mb;		/* opcode, modifier, a-mode, b-mode */

  s = get_tok( s, &tok );
  if ( tok == 0 ) return ASMLINE_NONE;

  /*
   * Ignore string lines '^Program.*' and comments.
   */
  if ( tok == TOK_STR && 0 == strcmp( "PROGRAM", tok_buf ))
  {
    return ASMLINE_NONE;
  }
  if ( tok == ';' ) return ASMLINE_NONE;

  /*
   * Now match the instruction's various components:
   *   [START label,] opcode, modifier, a-mode, a-value, b-mode, b-value
   */

  /* Match possible start label
   */
  if ( tok == TOK_START ) {
    flags |= fl_START;
    s = get_tok( s, &tok );
  }

  /* Match opcode
   */
  if ( is_tok_pseudoop(tok) ) {
    switch ( tok ) {
    case TOK_END:
      return ASMLINE_DONE;	/* signal done assembling */

    case TOK_ORG:
      s = get_tok( s, &tok );	/* get the next token */

      if ( tok == TOK_START )	/* ignore: */
	return ASMLINE_NONE;	/* start label already matched and processed */

      if ( tok != TOK_INT ) {
	panic_bad_token( tok, "an integer -- an int or \"START\" "
			 "follows ORG" );
      }
      in->a = tok_int;
      return ASMLINE_ORG;

    case TOK_PIN:
      s = get_tok( s, &tok );
      if ( tok != TOK_INT ) {
	panic_bad_token( tok, "an integer -- PIN must be an unsigned integer");
      }
      in->a = tok_int;
      return ASMLINE_PIN;

    default:
      panic_bad_token( tok, "a pseudo-op (internal assembler error)" );
    }
  }
  if (!( is_tok_opcode(tok)))
    panic_bad_token( tok, "an opcode" );

  op = DAT;
  switch(tok) {
  case TOK_DAT: op = DAT; break;
  case TOK_SPL: op = SPL; break;
  case TOK_MOV: op = MOV; break;
  case TOK_JMP: op = JMP; break;
  case TOK_JMZ: op = JMZ; break;
  case TOK_JMN: op = JMN; break;
  case TOK_ADD: op = ADD; break;
  case TOK_SUB: op = SUB; break;
  case TOK_SEQ: op = SEQ; break;
  case TOK_SNE: op = SNE; break;
  case TOK_MUL: op = MUL; break;
  case TOK_DIV: op = DIV; break;
  case TOK_DJN: op = DJN; break;
  case TOK_SLT: op = SLT; break;
  case TOK_MOD: op = MODM; break;
  case TOK_NOP: op = NOP; break;
  case TOK_LDP: op = LDP; break;
  case TOK_STP: op = STP; break;
  default:
    panic_bad_token( tok, "an opcode" );
  }

  /* Match modifier
   */
  s = get_tok( s, &tok );	/* first the '.' */
  if ( tok != '.' )
    panic_bad_token( tok, "'.'" );

  s = get_tok( s, &tok );	/* then the modifier itself */
  if ( ! is_tok_modifier(tok) )
    panic_bad_token( tok, "a modifier");
  m = tok - TOK_mF;

  /* Match a-field addressing mode and a-field
   */
  s = get_tok( s, &tok );
  if ( ! is_tok_mode(tok) )
    panic_bad_token( tok, "an addressing mode specifier");
  ma = tok - TOK_DIRECT;

  s = get_tok( s, &tok );
  if ( tok != TOK_INT )
    panic_bad_token( tok, "an integer");
  in->a = MODS(tok_int,CORESIZE);

  /* Match comma
   */
  s = get_tok( s, &tok );
  if ( tok != ',' )
    panic_bad_token( tok, "','" );

  /* Match b-field addressing mode and a-field
   */
  s = get_tok( s, &tok );
  if ( ! is_tok_mode(tok) )
    panic_bad_token( tok, "an addressing mode specifier");
  mb = tok - TOK_DIRECT;

  s = get_tok( s, &tok );
  if ( tok != TOK_INT )
    panic_bad_token( tok, "an integer");
  in->b = MODS(tok_int,CORESIZE);


  /*
   * Set flags and ignore the rest of the line
   */
  in->in = (flags << flPOS) | OP( op, m, ma, mb );
  return ASMLINE_OK;
}




/* NAME
 *     asm_file, asm_fname -- assemble a FILE into a warrior
 * 
 * SYNOPSIS
 *     void asm_file( FILE *F, warrior_t *w, unsigned int CORESIZE );
 *     void asm_fname( const char *filename, warrior_t *w,
 *    	               unsigned int CORESIZE );
 * 
 * INPUTS
 *     w        -- warrior_t to assemble into.
 *     F        -- stream to read warrior source from
 *     filename -- path to source file.  May be '-'
 *		   which is interpreted as stdin.
 *     CORESIZE -- just that
 * 
 * DESCRIPTION
 *     These functions assemble a source file into a
 *     warrior_t setting all the non-info fields.
 * 
 * RESULTS
 *    If the warrior assembled correctly, then warrior_t
 *    contains its code and starting offset.  If an error
 *    occured during assembly, an error message is issued
 *    and the program exit()s.
 * 
 * GLOBALS
 *     none as such, subroutines use tok_buf[], tok_int, str_toks[],
 *     MAXLENGTH constant
 * 
 * SEE ALSO
 *     asm_line()
 * 
 * BUGS
 *     Its not really acceptable to exit() on an assembly error.
 */
void
asm_file( FILE *F, warrior_t *w, unsigned int CORESIZE )
{
  char line[MAX_ALL_CHARS];
  insn_t *c;
  int ret;			/* return code from asm_line() */

  w->len = w->start = 0;
  w->have_pin = 0;
  w->pin = 0;
  c = w->code;

  while ( fgets(line, MAX_ALL_CHARS, F) ) {
    ret = asm_line( line, c, CORESIZE );
    if ( ret == ASMLINE_DONE ) break;

    switch ( ret ) {
    case ASMLINE_OK:
      if ( get_flags( c->in ) & fl_START ) {
	w->start = w->len;
	clr_flags( c->in, fl_START );
      }
      if ( w->len < MAXLENGTH) c++; 
      w->len++;
      break;

    case ASMLINE_ORG:
      w->start =  c->a;		/* was `ORG int', get the starting address */
      break;

    case ASMLINE_NONE:
      break;			/* nop */

    case ASMLINE_PIN:
      w->have_pin = 1;
      w->pin = c->a;		/* save PIN. */
      break;

    default:
      fprintf(stderr,"asm.c/asm_file(): illegal return code from asm_line()\n");
      exit(1);
    }
    if ( w->len > MAXLENGTH ) {
      fprintf(stderr, "too many instructions in warrior %d\n", w->no);
      exit(1);
    }
  }
  if ( w->start >= w->len ) {
    fprintf(stderr, "starting address must be inside warrior body\n" );
    exit(1);
  }
}


void
asm_fname( const char *fname, warrior_t *w, unsigned int CORESIZE )
{
  FILE *F;
  int is_stdin = 0;
 
  if ( strcmp( fname, "-" ) == 0 ) {
    F = stdin;
    is_stdin = 1;
  } 
  else
    if (!( F = fopen(fname, "r") )) {
      fprintf(stderr, "can't open file %s\n", fname);
      exit(1);
    }

  asm_file(F, w, CORESIZE);

  if ( !is_stdin ) fclose(F);
}



/* NAME
 *     dis1 -- disasemble an instruction
 *     discore -- disasemble a segment of core
 * 
 * SYNOPSIS
 *     void dis1( char *s, inst_t in, unsigned int CORESIZE );
 *     void discore( inst_t *core, unsigned int start, unsigned int end,
 * 		     unsigned int CORESIZE );
 * 
 * INPUTS
 *     s -- string to print disassembled instruction to. A string
 *          of length 60 should be more than sufficient.
 *     in -- instruction to disassemble
 *     core -- pointer to start of core
 *     start -- core segment start offset
 *     end -- core segment end offset (excluded)
 * 
 * RESULTS
 *     dis1 -- The disassembled instruction is printed to `s'.
 *     discore -- A segment of core is dissasembled and printed
 *                to stdout with core addresses.
 */

void
dis1(char *buf, insn_t in, unsigned int CORESIZE)
{
  int x;
  char *op_s, *mo_s, *ma_s, *mb_s;
  int af, bf;

  x = (in.in >> opPOS) & opMASK;
  switch( x ) {
  case DAT: op_s = "dat"; break;
  case SPL: op_s = "spl"; break;
  case MOV: op_s = "mov"; break;
  case JMP: op_s = "jmp"; break;
  case JMZ: op_s = "jmz"; break;
  case JMN: op_s = "jmn"; break;
  case ADD: op_s = "add"; break;
  case SUB: op_s = "sub"; break;
  case SEQ: op_s = "seq"; break;
  case SNE: op_s = "sne"; break;
  case MUL: op_s = "mul"; break;
  case DIV: op_s = "div"; break;
  case DJN: op_s = "djn"; break;
  case SLT: op_s = "slt"; break;
  case MODM: op_s = "mod"; break;
  case NOP: op_s = "nop"; break;
  case LDP: op_s = "ldp"; break;
  case STP: op_s = "stp"; break;
  default:
    op_s = "???";
  }

  x = (in.in >> moPOS) & moMASK;
  switch ( x ) {
  case mF:  mo_s = "f "; break;
  case mA:  mo_s = "a "; break;
  case mB:  mo_s = "b "; break;
  case mAB: mo_s = "ab"; break;
  case mBA: mo_s = "ba"; break;
  case mX:  mo_s = "x "; break;
  case mI:  mo_s = "i "; break;
  default:
    mo_s = "?";
  }


  x = (in.in >> maPOS) & mMASK;
  switch (x) {
  case DIRECT: ma_s = "$"; break;
  case IMMEDIATE: ma_s = "#"; break;
  case AINDIRECT: ma_s = "*"; break;
  case BINDIRECT: ma_s = "@"; break;
  case APREDEC: ma_s = "{"; break;
  case APOSTINC: ma_s = "}"; break;
  case BPREDEC: ma_s = "<"; break;
  case BPOSTINC: ma_s = ">"; break;
  default: ma_s = "?";
  }

  x = (in.in >> mbPOS) & mMASK;
  switch (x) {
  case DIRECT:    mb_s = "$"; break;
  case IMMEDIATE: mb_s = "#"; break;
  case AINDIRECT: mb_s = "*"; break;
  case BINDIRECT: mb_s = "@"; break;
  case APREDEC:   mb_s = "{"; break;
  case APOSTINC:  mb_s = "}"; break;
  case BPREDEC:   mb_s = "<"; break;
  case BPOSTINC:  mb_s = ">"; break;
  default: mb_s = "?";
  }

  af = in.a <= CORESIZE/2 ? in.a : in.a - CORESIZE;
  bf = in.b <= CORESIZE/2 ? in.b : in.b - CORESIZE;

  sprintf(buf,"%s.%s %s%5d , %s%5d", op_s, mo_s, ma_s, af, mb_s, bf);
}


void
discore( const insn_t *core,
	 int start,
	 int end,
	 unsigned int CORESIZE )
{
  int adr;
  char line[MAX_ALL_CHARS];
  for ( adr = start; adr < end; adr++ ) {
    int i = MODS(adr, CORESIZE);
    dis1( line, core[i], CORESIZE );
    printf("%4d    %s\n", adr, line);
  }
}
