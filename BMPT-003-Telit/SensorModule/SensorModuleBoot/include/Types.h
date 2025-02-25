/*******************************************************************************
 * Copyright 2004 Appareo Systems
 * All rights reserved
 * This software and/or material is the property of Appareo Systems.
 * All use, disclosure, and/or reproduction not specifically authorized in
 * writing by Appareo Systems is prohibited.
 *
 * PROJECT:       Navigation Suite
 *
 * FILE NAME:     $Workfile:   NavSuite.c  $
 *
 * DESCRIPTION:   User Defined Variable Types.
 *
 * REVISION HISTORY:
 *
 * $Log:$
*******************************************************************************/

#ifndef _TYPES_H_
#define _TYPES_H_

typedef unsigned char      Bool;

typedef unsigned char       U8 ;
typedef unsigned short      U16;
typedef unsigned long       U32;
typedef unsigned long long  U64;
typedef signed char         S8 ;
typedef signed short        S16;
typedef signed long         S32;
typedef signed long long    S64;


/* User defined types (unsigned). */
typedef unsigned char      BYTE;
typedef unsigned short     WORD;
typedef unsigned long      DWORD;
typedef unsigned int       UINT;


/* User defined types (signed). */
typedef signed char        SBYTE;
typedef signed short       SWORD;
typedef signed long        SDWORD;

/* User defined types. */
typedef char               BOOL;

/* Define the unsigned shift parameter type (USP) as a 16-bit unsigned
   value. */
typedef unsigned short     USP;
typedef signed short       SSP;

#define BYTE_MAX     (0xFF)
#define WORD_MAX     (0xFFFFU)
#define DWORD_MAX    (0xFFFFFFFFUL)
#define USP_MAX      (0xFFFFU)

#define SBYTE_MAX    (0x7F)
#define SBYTE_MIN    (-SBYTE_MAX-1)
#define SWORD_MAX    (0x7FFF)
#define SWORD_MIN    (-SWORD_MAX-1)
#define SDWORD_MAX   (0x7FFFFFFFL)
#define SDWORD_MIN   (-SDWORD_MAX-1)
#define SSP_MAX      (0x7FFF)
#define SSP_MIN      (-SSP_MAX-1)

#define BIN0(X)   (X)
#define BIN1(X)   ((X) * 0x0002)
#define BIN2(X)   ((X) * 0x0004)
#define BIN3(X)   ((X) * 0x0008)
#define BIN4(X)   ((X) * 0x0010)
#define BIN5(X)   ((X) * 0x0020)
#define BIN6(X)   ((X) * 0x0040)
#define BIN7(X)   ((X) * 0x0080)
#define BIN8(X)   ((X) * 0x0100)
#define BIN9(X)   ((X) * 0x0200)
#define BIN10(X)  ((X) * 0x0400)
#define BIN11(X)  ((X) * 0x0800)
#define BIN12(X)  ((X) * 0x1000)
#define BIN13(X)  ((X) * 0x2000)
#define BIN14(X)  ((X) * 0x4000)
#define BIN15(X)  ((X) * 0x8000)
#define BIN16(X)  ((X) * 0x10000L)


#define BIN_1(X)  ((X) / 0x0002)
#define BIN_2(X)  ((X) / 0x0004)
#define BIN_3(X)  ((X) / 0x0008)
#define BIN_4(X)  ((X) / 0x0010)
#define BIN_5(X)  ((X) / 0x0020)
#define BIN_6(X)  ((X) / 0x0040)
#define BIN_7(X)  ((X) / 0x0080)
#define BIN_8(X)  ((X) / 0x0100)
#define BIN_9(X)  ((X) / 0x0200)
#define BIN_10(X) ((X) / 0x0400)
#define BIN_11(X) ((X) / 0x0800)
#define BIN_12(X) ((X) / 0x1000)
#define BIN_13(X) ((X) / 0x2000)
#define BIN_14(X) ((X) / 0x4000)
#define BIN_15(X) ((X) / 0x8000)
#define BIN_16(X) ((X) / 0x10000L)


/* Boolean Definitions   */
 //#ifndef TRUE
 //   #define TRUE 1
 //   #define FALSE 0
 //#endif

#ifndef ON
   #define ON  1
   #define OFF 0
#endif


typedef union
{
   BYTE Byte[4];
   WORD Word[2];
   DWORD DWord;
} BWD_UNION;

typedef union
{
   BYTE Byte[2];
   WORD Word;
} BW_UNION;


typedef union
{
  U16 h   ;     // h as HALF-WORD
  U8  b[2];     // b as BYTE
} Union16;

typedef union
{
  U32 w   ;     // w as WORD
  U16 h[2];     // h as HALF-WORD
  U8  b[4];     // b as BYTE
} Union32;

typedef union
{
  U64 d   ;     // d as DOUBLE-WORD
  U32 w[2];     // w as WORD
  U16 h[4];     // h as HALF-WORD
  U8  b[8];     // b as BYTE
} Union64;

//_____ M A C R O S ____________________________________________________________

//! Some usefull macros...
    // Max(a, b): Take the max between a and b
    // Min(a, b): Take the min between a and b
    // Align_up(val, n):   Around (up)   the number (val) on the (n) boundary
    // Align_down(val, n): Around (down) the number (val) on the (n) boundary
#define Max(a, b)          ( (a)>(b) ? (a) : (b) )
#define Min(a, b)          ( (a)<(b) ? (a) : (b) )
#define Align_up(val, n)   ( ((val)+(n)-1) & ~((n)-1) )
#define Align_down(val, n) (  (val)        & ~((n)-1) )

//! Bit and bytes manipulations
#define Low(data_w)                ((U8)data_w)
#define High(data_w)               ((U8)(data_w>>8))
#define Tst_bit_x(addrx,mask)   (*addrx & mask)
#define Set_bit_x(addrx,mask)   (*addrx = (*addrx |  mask))
#define Clr_bit_x(addrx,mask)   (*addrx = (*addrx & ~mask))

//! The default value to initialise pointers
#define  NULL              ((void *)0)

//! Constants
#define ENABLE   1
#define ENABLED  1
#define DISABLED 0
#define DISABLE  0
#define FALSE   (0==1)
#define TRUE    (1==1)
#define KO      0
#define OK      1
#define CLR     0
#define SET     1
#define OFF     0
#define ON      1



#endif /* _TYPES_H_ */
/*******************************************************************************/


/*******************************************************************************
                                End of File
*******************************************************************************/
