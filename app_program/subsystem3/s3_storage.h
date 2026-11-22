/* Single-task bit-fields only; shared records use masks, atomics and sequences. */
#ifndef S3_STORAGE_H
#define S3_STORAGE_H
#include <tk/typedef.h>
#if defined(__MINGW32__)
#define S3_LOCAL_BITS __attribute__((gcc_struct))
#else
#define S3_LOCAL_BITS
#endif
#endif
