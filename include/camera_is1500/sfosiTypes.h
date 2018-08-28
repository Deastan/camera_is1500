///////////////////////////////////////////////////////////////////////////////
// sfosiTypes.h - common type definitions
// Copyright (c) InterSense LLC 2012. All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SFOSITYPES_H_
#define SFOSITYPES_H_

// Standard numeric types
#if !defined QT_PROJECT && defined WIN32
typedef unsigned __int8 uint8_t;
typedef __int16 int16_t;
typedef unsigned __int16 uint16_t;
typedef __int32 int32_t;
typedef unsigned __int32 uint32_t;
typedef __int64 int64_t;
typedef unsigned __int64 uint64_t;
#else
#include <stdint.h>
#endif

#endif // SFOSITYPES_H_
