#ifndef EXHAUST_H
#define EXHAUST_H
/*  exhaust.h:  Global constants, structures, and types
 * $Id: exhaust.h,v 1.4 2002/05/13 03:05:33 rowan Exp $
 */

/* This file is part of `exhaust', a memory array redcode simulator.
 * Copyright (C) 2002 M Joonas Pihlaja
 * Public Domain.
 */


/* global debug level */
#ifndef DEBUG
#define DEBUG 0
#endif


/*
 * Global constants
 *
 */

#define MAXLENGTH 100
  /* max length of warrior */


/*
 * Global types
 *
 */

/* misc. integral types */
typedef unsigned char  u8_t;
typedef unsigned short u16_t;
typedef unsigned int   u32_t;
typedef int            s32_t;

  /* Choose an appropriate field_t width.  In a field_t variable
   * we hold unsigned integers in 0..CORESIZE-1
   */
#define FIELD_T_WIDTH 32

#if FIELD_T_WIDTH <= 8
  typedef u8_t field_t;
#elif FIELD_T_WIDTH <= 16
  typedef u16_t field_t;
#else
  typedef u32_t field_t;
#endif

  /*
   * Instructions in core:
   */
  typedef struct insn_st {
    u16_t in;                   /* flags, opcode, modifier, a- and b-modes */
    field_t a, b;               /* a-value, b-value */
  } insn_t;


  /*
   * Warrior struct
   */
  typedef struct warrior_st {
    insn_t code[ MAXLENGTH ];   /* code of warrior */
    unsigned int len;		/* length of -"- */
    unsigned int start;		/* start relative to first insn */

    int have_pin;		/* does warrior have pin? */
    u32_t pin;			/* pin of warrior or garbage. */

    /* info fields -- these aren't automatically set or used */
    char *name;
    int no;                     /* warrior no. */
  } warrior_t;


#endif /* EXHAUST_H */
