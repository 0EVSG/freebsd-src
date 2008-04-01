#ifndef	_SPL_ISA_DEFS_H
#define	_SPL_ISA_DEFS_H

#ifdef  __cplusplus
extern "C" {
#endif

/* x86_64 arch specific defines */
#if defined(__x86_64) || defined(__x86_64__)

#if !defined(__x86_64)
#define __x86_64
#endif

#if !defined(__amd64)
#define __amd64
#endif

#if !defined(__x86)
#define __x86
#endif

#if !defined(_LP64)
#define _LP64
#endif

/* i386 arch specific defines */
#elif defined(__i386) || defined(__i386__)

#if !defined(__i386)
#define __i386
#endif

#if !defined(__x86)
#define __x86
#endif

#if !defined(_ILP32)
#define _ILP32
#endif

#else /* Currently only x86_64 and i386 arches supported */
#error "Unsupported ISA type"
#endif

#if defined(_ILP32) && defined(_LP64)
#error "Both _ILP32 and _LP64 are defined"
#endif

#include <sys/byteorder.h>

#if defined(__LITTLE_ENDIAN) && !defined(_LITTLE_ENDIAN)
#define _LITTLE_ENDIAN __LITTLE_ENDIAN
#endif

#if defined(__BIG_ENDIAN) && !defined(_BIG_ENDIAN)
#define _BIG_ENDIAN __BIG_ENDIAN
#endif

#if defined(_LITTLE_ENDIAN) && defined(_BIG_ENDIAN)
#error "Both _LITTLE_ENDIAN and __BIG_ENDIAN are defined"
#endif

#ifdef  __cplusplus
}
#endif

#endif	/* _SPL_ISA_DEFS_H */
