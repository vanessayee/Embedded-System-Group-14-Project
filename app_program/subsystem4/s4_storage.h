/* Task-owned bounded data; shared words retain their explicit packers. */
#ifndef S4_STORAGE_H
#define S4_STORAGE_H
#include <tk/typedef.h>
#if defined(__MINGW32__)
#define S4_LOCAL_BITS __attribute__((gcc_struct))
#else
#define S4_LOCAL_BITS
#endif
#endif
