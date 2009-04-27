#ifndef INSN_H
#define INSN_H
/* insn.h: Instruction encoding definition
 * $Id: insn.h,v 1.5 2002/10/01 22:25:18 rowan Exp $
 */

/* This file is part of `exhaust', a memory array redcode simulator.
 * Copyright (C) 2002 M Joonas Pihlaja
 * Public Domain.
 */

/*
 * Instruction encoding:
 *
 * Instructions are held in a insn_t typed struct with three fields:
 *   in:	instruction flags, opcode, modifier, a-mode, b-mode
 *   a:		a-value
 *   b: 	b-value
 *
 * The layout of the `in' field is as follows:
 *
 * bit         15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
 * field   | flags | |- op-code  -| |-.mod-| |b-mode| |a-mode|
 *
 * Currently there is only one flag, the fl_START flag, which
 * the assembler uses to figure out the starting instruction
 * of a warrior (i.e. the one given by the START label).
 */

#define mBITS 3			/* number of bits for mode */
#define opBITS 5		/* bits for opcode */
#define moBITS 3		/* bits for modifier */
#define flBITS 2		/* bits for flags */

/* Positions of various fields
 */
#define maPOS 0
#define mbPOS (maPOS + mBITS)
#define moPOS (mbPOS + mBITS)
#define opPOS (moPOS + moBITS)
#define flPOS (opPOS + opBITS)

/* Various masks.  These extract a field once it has been
 * shifted to the least significant bits of the word.
 */
#define moMASK ((1<<moBITS)-1)
#define opMASK ((1<<opBITS)-1)
#define mMASK ((1<<mBITS)-1)
#define flMASK ((1<<flBITS)-1)
#define iMASK ( (1<<flPOS)-1 )

/*
 * Extract the flags of an instruction `in' field
 */
#define get_flags(in) ( ((in)>>flPOS) & flMASK )
#define set_flags(in,flags) ( (in) |= ((flags)<<flPOS) )
#define clr_flags(in,flags) ( (in) &=~ ((flags)<<flPOS) )


/*
 * OP(o,m,ma,mb): This macro encodes an instruction `in' field
 *		  from its various components (not flags).
 *
 * 	o: opcode
 *	m: modifier
 *	ma: a-mode
 *	mb: b-mode
 *
 *  e.g. OP(SPL, mF, DIRECT, BPREDEC )
 *  is a
 *       spl.f $  , < 
 */
#define _OP(o,m) ( ((o)<<moBITS) | (m) )
#define OP( o, m, ma, mb ) ((_OP((o),(m))<<moPOS) | ((mb) << mbPOS) | (ma))


/*
 * Encodings for various fields of the `in' field.
 * 
 */
enum {
  DAT,				/* must be 0 */
  SPL,
  MOV,
  DJN,
  ADD,
  JMZ,
  SUB,
  SEQ,
  SNE,
  SLT,
  JMN,
  JMP,
  NOP,
  MUL,
  MODM,
  DIV,
  LDP,
  STP				/* 18 */
};

enum {
  mF, mA, mB, mAB, mBA, mX, mI	/* 7 */
};

enum {				/* must start from 0,
				   the ordering is important */
  DIRECT,
  IMMEDIATE,
  BINDIRECT,
  BPREDEC,
  BPOSTINC,
  AINDIRECT,
  APREDEC,
  APOSTINC		/* 8 */
};

#define INDIRECT BINDIRECT
#define PREDEC   BPREDEC
#define POSTINC  BPOSTINC

#define INDIR_A(mode) ((mode) >= AINDIRECT)
#define RAW_MODE(mode) ((mode) + (INDIRECT-AINDIRECT))

/*
 * flags
 */
enum {
  flB_START			/* start label */
};

#define fl_START (1<<flB_START)


/* Macros to take things mod CORESIZE
 *
 * Mod here is the `mathematical' modulo, with non-negative
 * results even with x negative.
 * 
 * MOD(x,M):	x mod CORESIZE
 * MODS(x,M):	x mod CORESIZE
 */
#define MODS(x,M) ( (int)(x)%(int)(M) >= 0 \
		  ? (int)(x)%(int)(M) \
		  : (M) + ((int)(x)%(int)(M)) )
#define MOD(x,M) ( (x) % (M) )

#endif /* INSN_H */
