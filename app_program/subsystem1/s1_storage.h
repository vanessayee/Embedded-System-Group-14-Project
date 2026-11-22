/* Task-owned storage layouts. Never serialize these C bit-fields or share
 * their read-modify-write accesses across tasks/ISRs. Wire formats continue
 * to use explicit masks and shifts. */
#ifndef S1_STORAGE_H
#define S1_STORAGE_H
#include <tk/typedef.h>
/* MinGW otherwise defaults to Microsoft bit-field allocation, unlike ARM GCC. */
#if defined(__MINGW32__)
#define S1_LOCAL_PACKED __attribute__((packed, gcc_struct))
#else
#define S1_LOCAL_PACKED __attribute__((packed))
#endif
#define S1_CMD_RATE_MAX 5U
#define S1_DUP_WINDOW 4U

typedef struct {
 unsigned char dup_n : 3;      /* 0..4 */
 unsigned char dup_next : 2;   /* 0..3 */
 unsigned char rate_count : 3; /* commands counted in this window, 0..5 */
} S1_RX_STATE;

typedef struct {
 unsigned char have_prev : 1;
 unsigned char prev_avoid_count : 4;
 unsigned char boot_report_sent : 1;
 unsigned char boot_marker_sent : 1;
} S1_TLM_STATE;

/* Rejected floods cannot wrap a narrowed counter and become accepted. */
static inline BOOL s1_rate_accept(S1_RX_STATE *s)
{
 if (s->rate_count >= S1_CMD_RATE_MAX) return FALSE;
 s->rate_count++;
 return TRUE;
}
#endif
