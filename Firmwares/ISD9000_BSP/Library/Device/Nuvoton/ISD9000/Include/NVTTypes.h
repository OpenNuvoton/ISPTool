/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __NVTTYPES_H__
#define __NVTTYPES_H__

// To ADS, kl modified on 2009/09/30
#if defined (__ARMCC_VERSION)
#include <stddef.h>
#endif
// To ADS, kl modified on 2009/09/30

#define CONST				const

#define FALSE				0
#define TRUE				1

#ifndef __NVTTYPE_PVOID_DEFINED__
#define __NVTTYPE_PVOID_DEFINED__
typedef void *				PVOID;
#endif	// #ifndef __NVTTYPE_PVOID_DEFINED__

#ifndef __NVTTYPE_BOOL_DEFINED__
#define __NVTTYPE_BOOL_DEFINED__
typedef unsigned char		BOOL;
#endif	// #ifndef __NVTTYPE_BOOL_DEFINED__

#ifndef __NVTTYPE_PBOOL_DEFINED__
#define __NVTTYPE_PBOOL_DEFINED__
typedef unsigned char *		PBOOL;
#endif	// #ifndef __NVTTYPE_PBOOL_DEFINED__

#ifndef	__NVTTYPE_INT8__
#define __NVTTYPE_INT8__
typedef signed char			INT8;
#endif	// #ifndef __NVTTYPE_INT8_DEFINED__

#ifndef	__NVTTYPE_PINT8_DEFINED__
#define __NVTTYPE_PINT8_DEFINED__
typedef signed char *		PINT8;
#endif	// #ifndef __NVTTYPE_PINT8_DEFINED__

#ifndef	__NVTTYPE_UINT8_DEFINED__
#define __NVTTYPE_UINT8_DEFINED__
typedef unsigned char		UINT8;
#endif	// #ifndef __NVTTYPE_UINT8_DEFINED__

#ifndef	__NVTTYPE_PUINT8_DEFINED__
#define __NVTTYPE_PUINT8_DEFINED__
typedef unsigned char *		PUINT8;
#endif	// #ifndef __NVTTYPE_PUINT8_DEFINED__

#ifndef	__NVTTYPE_INT16_DEFINED__
#define __NVTTYPE_INT16_DEFINED__
typedef signed short		INT16;
#endif	// #ifndef __NVTTYPE_INT16_DEFINED__

#ifndef	__NVTTYPE_PINT16_DEFINED__
#define __NVTTYPE_PINT16_DEFINED__
typedef signed short *		PINT16;
#endif	// #ifndef __NVTTYPE_PINT16_DEFINED__

#ifndef	__NVTTYPE_UINT16_DEFINED__
#define __NVTTYPE_UINT16_DEFINED__
typedef unsigned short		UINT16;
#endif	// #ifndef __NVTTYPE_UINT16_DEFINED__

#ifndef	__NVTTYPE_PUINT16_DEFINED__
#define __NVTTYPE_PUINT16_DEFINED__
typedef unsigned short *	PUINT16;
#endif	// #ifndef __NVTTYPE_PUINT16_DEFINED__

#ifndef	__NVTTYPE_INT32_DEFINED__
#define __NVTTYPE_INT32_DEFINED__
typedef signed int			INT32;
#endif	// #ifndef __NVTTYPE_INT32_DEFINED__

#ifndef	__NVTTYPE_PINT32__
#define __NVTTYPE_PINT32__
typedef signed int *		PINT32;
#endif	// #ifndef __NVTTYPE_PINT32__

#ifndef	__NVTTYPE_UINT32_DEFINED__
#define __NVTTYPE_UINT32_DEFINED__
typedef unsigned int		UINT32;
#endif	// #ifndef __NVTTYPE_UINT32_DEFINED__

#ifndef	__NVTTYPE_PUINT32_DEFINED__
#define __NVTTYPE_PUINT32_DEFINED__
typedef unsigned int *		PUINT32;
#endif	// #ifndef __NVTTYPE_PUINT32_DEFINED__

#ifdef __GNUC__

#ifndef	__NVTTYPE_INT64_DEFINED__
#define __NVTTYPE_INT64_DEFINED__
typedef signed long long	INT64;
#endif	// #ifndef __NVTTYPE_INT64_DEFINED__

#ifndef	__NVTTYPE_PINT64_DEFINED__
#define __NVTTYPE_PINT64_DEFINED__
typedef signed long long *	PINT64;
#endif	// #ifndef __NVTTYPE_PINT64_DEFINED__

#ifndef	__NVTTYPE_UINT64_DEFINED__
#define __NVTTYPE_UINT64_DEFINED__
typedef unsigned long long	UINT64;
#endif	// #ifndef __NVTTYPE_UINT64_DEFINED__

#ifndef	__NVTTYPE_PUINT64_DEFINED__
#define __NVTTYPE_PUINT64_DEFINED__
typedef unsigned long long *PUINT64;
#endif	// #ifndef __NVTTYPE_PUINT64_DEFINED__

#elif defined (__ARMCC_VERSION)

#ifndef	__NVTTYPE_INT64_DEFINED__
#define __NVTTYPE_INT64_DEFINED__
typedef signed __int64		INT64;
#endif	// #ifndef __NVTTYPE_INT64_DEFINED__

#ifndef	__NVTTYPE_PINT64_DEFINED__
#define __NVTTYPE_PINT64_DEFINED__
typedef signed __int64 *	PINT64;
#endif	// #ifndef __NVTTYPE_PINT64_DEFINED__

#ifndef	__NVTTYPE_UINT64_DEFINED__
#define __NVTTYPE_UINT64_DEFINED__
typedef unsigned __int64	UINT64;
#endif	// #ifndef __NVTTYPE_UINT64_DEFINED__

#ifndef	__NVTTYPE_PUINT64_DEFINED__
#define __NVTTYPE_PUINT64_DEFINED__
typedef unsigned __int64	*PUINT64;
#endif	// #ifndef __NVTTYPE_PUINT64_DEFINED__

#endif	// __GNUC__

#ifndef	__NVTTYPE_FLOAT_DEFINED__
#define __NVTTYPE_FLOAT_DEFINED__
typedef float				FLOAT;
#endif	// #ifndef __NVTTYPE_FLOAT_DEFINED__

#ifndef	__NVTTYPE_PFLOAT_DEFINED__
#define __NVTTYPE_PFLOAT_DEFINED__
typedef float *				PFLOAT;
#endif	// #ifndef __NVTTYPE_PFLOAT_DEFINED__

#ifndef	__NVTTYPE_DOUBLE_DEFINED__
#define __NVTTYPE_DOUBLE_DEFINED__
typedef double				DOUBLE;
#endif	// #ifndef __NVTTYPE_DOUBLE_DEFINED__

#ifndef	__NVTTYPE_PDOUBLE_DEFINED__
#define __NVTTYPE_PDOUBLE_DEFINED__
typedef double *			PDOUBLE;
#endif	// #ifndef __NVTTYPE_PDOUBLE_DEFINED__

#ifndef	__NVTTYPE_CHAR_DEFINED__
#define __NVTTYPE_CHAR_DEFINED__
typedef signed char			CHAR;
#endif	// #ifndef __NVTTYPE_CHAR_DEFINED__

#ifndef	__NVTTYPE_PCHAR_DEFINED__
#define __NVTTYPE_PCHAR_DEFINED__
typedef signed char *		PCHAR;
#endif	// #ifndef __NVTTYPE_PCHAR_DEFINED__

#ifndef	__NVTTYPE_PSTR_DEFINED__
#define __NVTTYPE_PSTR_DEFINED__
typedef signed char *		PSTR;
#endif	// #ifndef __NVTTYPE_PSTR_DEFINED__

#ifndef	__NVTTYPE_PCSTR_DEFINED__
#define __NVTTYPE_PCSTR_DEFINED__
typedef const signed char *	PCSTR;
#endif	// #ifndef __NVTTYPE_PCSTR_DEFINED__

#ifdef __GNUC__
#ifndef	__NVTTYPE_WCHAR_DEFINED__
#define __NVTTYPE_WCHAR_DEFINED__
typedef	UINT16				WCHAR;
#endif	// #ifndef __NVTTYPE_WCHAR_DEFINED__

#ifndef	__NVTTYPE_PWCHAR_DEFINED__
#define __NVTTYPE_PWCHAR_DEFINED__
typedef	UINT16 *			PWCHAR;
#endif	// #ifndef __NVTTYPE_PWCHAR_DEFINED__

#ifndef	__NVTTYPE_PWSTR_DEFINED__
#define __NVTTYPE_PWSTR_DEFINED__
typedef	UINT16 *			PWSTR;
#endif	// #ifndef __NVTTYPE_PWSTR_DEFINED__

#ifndef	__NVTTYPE_PCWSTR_DEFINED__
#define __NVTTYPE_PCWSTR_DEFINED__
typedef	const UINT16 *		PCWSTR;
#endif	// #ifndef __NVTTYPE_PCWSTR_DEFINED__

#elif defined (__ARMCC_VERSION)
#ifndef	__NVTTYPE_WCHAR_DEFINED__
#define __NVTTYPE_WCHAR_DEFINED__
typedef	wchar_t				WCHAR;
#endif	// #ifndef __NVTTYPE_WCHAR_DEFINED__

#ifndef	__NVTTYPE_PWCHAR_DEFINED__
#define __NVTTYPE_PWCHAR_DEFINED__
typedef	wchar_t *			PWCHAR;
#endif	// #ifndef __NVTTYPE_PWCHAR_DEFINED__

#ifndef	__NVTTYPE_PWSTR_DEFINED__
#define __NVTTYPE_PWSTR_DEFINED__
typedef	wchar_t *			PWSTR;
#endif	// #ifndef __NVTTYPE_PWSTR_DEFINED__

#ifndef	__NVTTYPE_PCWSTR_DEFINED__
#define __NVTTYPE_PCWSTR_DEFINED__
typedef	const wchar_t *		PCWSTR;
#endif	// #ifndef __NVTTYPE_PCWSTR_DEFINED__

#endif	// __GNUC__

#ifndef	__NVTTYPE_SIZE_T_DEFINED__
#define __NVTTYPE_SIZE_T_DEFINED__
typedef UINT32				SIZE_T;
#endif	// #ifndef __NVTTYPE_SIZE_T_DEFINED__

#ifndef	__NVTTYPE_REG8_DEFINED__
#define __NVTTYPE_REG8_DEFINED__
typedef volatile UINT8		REG8;
#endif	// #ifndef __NVTTYPE_REG8_DEFINED__

#ifndef	__NVTTYPE_REG16_DEFINED__
#define __NVTTYPE_REG16_DEFINED__
typedef volatile UINT16		REG16;
#endif	// #ifndef __NVTTYPE_REG16_DEFINED__

#ifndef	__NVTTYPE_REG32_DEFINED__
#define __NVTTYPE_REG32_DEFINED__
typedef volatile UINT32		REG32;
#endif	// #ifndef __NVTTYPE_REG32_DEFINED__

#ifndef	__NVTTYPE_BYTE_DEFINED__
#define __NVTTYPE_BYTE_DEFINED__
typedef	unsigned char		BYTE;
#endif	// #ifndef __NVTTYPE_BYTE_DEFINED__

typedef UINT8				ERRCODE;

#endif /* __NVTTYPES_H__ */

