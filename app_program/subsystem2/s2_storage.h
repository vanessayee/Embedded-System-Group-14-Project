/* Local bounded application state. Shared words retain app_sync ownership. */
#ifndef S2_STORAGE_H
#define S2_STORAGE_H
#include <tk/typedef.h>
#if defined(__MINGW32__)
#define S2_LOCAL_BITS __attribute__((gcc_struct))
#else
#define S2_LOCAL_BITS
#endif
#define S2_LINE_SEQ_NONE 16U /* four-bit sequence 0..15 plus one sentinel */
typedef struct S2_LOCAL_BITS {
 UW time_ms;
 unsigned char seq : 4;
 unsigned char seen : 1;
} S2_FRESH;
static inline BOOL s2_fresh_sample(S2_FRESH *s, UW seq, UW now_ms, UW stale_ms)
{
 if (!s->seen || seq != s->seq) { s->seen=TRUE; s->seq=seq; s->time_ms=now_ms; }
 return (UW)(now_ms-s->time_ms) <= stale_ms;
}
#endif
