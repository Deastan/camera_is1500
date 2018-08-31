/*****************************************************************************
*
*  File Name:     types.h
*  Description:   Global Intersense Types
*  Created:       12/19/96
*  Author:        Mike Harrington
*
*  Copyright:     InterSense 2002 - All rights Reserved.
*  Comments: Fix the size of variables so they stay the same
*            from platform to platform.
*
*
******************************************************************************/
#ifndef _ISENSE_INC_types_h
#define _ISENSE_INC_types_h
#define INTERSENSE64  0

#if defined WINX64
#undef INTERSENSE64
#define INTERSENSE64        1
#endif


// Compensation level for slow CPU.
// This definition is here instead of ibase.h to guarantee
// that correct data types are used when only types.h is included.
//
// Level 0 - None, default
// Level 1 - Use 32 bit floats instead of doubles
// Level 2 - Slower data rate
// Level 3 - Slower than level 2
// -------------------------------------------------------------------
#define SLOW_CPU_COMPENSATION  0

#if defined _WIN32_WCE
#undef SLOW_CPU_COMPENSATION
#define SLOW_CPU_COMPENSATION  3
#endif

#ifdef POWERPC
#undef SLOW_CPU_COMPENSATION
#define SLOW_CPU_COMPENSATION 0
#endif

#ifdef _XILINK_ISENSE
#undef SLOW_CPU_COMPENSATION
#define SLOW_CPU_COMPENSATION  3
#endif

// SH4 has hardware floating point.
// ---------------------------------------------------------
#if (defined SHx && defined SH4) || defined ARMV4
#undef SLOW_CPU_COMPENSATION
#define SLOW_CPU_COMPENSATION  2
#endif

typedef unsigned char       BYTE;
typedef float               FLOAT32;
typedef double              FLOAT64;

typedef signed char         INT8;
typedef short               INT16;
typedef unsigned char       UINT8;
typedef unsigned short      UINT16;
typedef unsigned int        UINT;
typedef int                 INT;

#if defined UNDER_RTSS
typedef unsigned short      WORD;
typedef unsigned long       DWORD;
typedef int                 INT32;
typedef unsigned int        UINT32;
typedef int                 Bool;

#elif defined _WIN32

typedef int                 INT32;
typedef unsigned int        UINT32;
typedef int                 Bool;

// 64-bit Linux or MacOS X (Intel)
#elif defined __x86_64__
#undef INTERSENSE64
#define INTERSENSE64        1
typedef int                 INT32;
typedef unsigned int        UINT32;
typedef unsigned int        DWORD;
typedef INT32               LONG;
typedef INT32               Bool;
typedef UINT16              WORD;
typedef float               FLOAT;
typedef int                 HWND;


// 64 bit HP Unix
#elif defined __LP64__

#undef INTERSENSE64
#define INTERSENSE64        1

typedef int                 INT32;
typedef unsigned int        UINT32;
typedef unsigned int        DWORD;  
typedef INT32               LONG;
typedef INT32               Bool;
typedef UINT16              WORD;
typedef float               FLOAT;
typedef int                 HWND;

#elif defined _XILINK_ISENSE

typedef Xint32              INT32;
typedef Xuint32             UINT32;
typedef Xuint32             DWORD;  
typedef INT32               LONG;
typedef INT32               Bool;
typedef UINT16              WORD;
typedef float               FLOAT;
typedef Xint32              HWND;

#else
typedef long                INT32;
typedef unsigned long       UINT32;
typedef unsigned long       DWORD;  // definitions found in Windef.h
typedef INT32               LONG;
typedef INT32               BOOL;
typedef INT32               Bool;
typedef UINT16              WORD;
typedef float               FLOAT;
typedef long				HWND;
#endif

#define Hwnd                HWND

#if SLOW_CPU_COMPENSATION > 0
typedef FLOAT32 REAL;
#else
typedef FLOAT64 REAL;
#endif


typedef INT     IntEle;
typedef IntEle *IntElePtr;
typedef IntEle *IntVectPtr;
typedef IntEle *IntMatPtr;

typedef const IntEle *ConstIntElePtr;
typedef const IntEle *ConstIntVectPtr;
typedef const IntEle *ConstIntMatPtr;

typedef REAL RealEle;
typedef RealEle *RealElePtr;
typedef RealEle *RealVectPtr;
typedef RealEle *RealMatPtr;

typedef const RealEle *ConstRealElePtr;
typedef const RealEle *ConstRealVectPtr;
typedef const RealEle *ConstRealMatPtr;


#endif



