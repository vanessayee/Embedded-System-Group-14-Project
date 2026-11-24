/* Task-owned bounded state only. Shared messages retain mask/shift packers. */
#ifndef S5_STORAGE_H
#define S5_STORAGE_H
#include <tk/typedef.h>
#if defined(__MINGW32__)
#define S5_LOCAL_BITS __attribute__((gcc_struct))
#else
#define S5_LOCAL_BITS
#endif
/* 8-bit fault runs retain their existing saturation/wrap behaviour. */
typedef struct S5_LOCAL_BITS {
 unsigned int silent_run : 8;
 unsigned int good_run : 8;
 unsigned int detect_run : 8;
 unsigned int side_count : 2;
} S5_SCAN_HEALTH;
typedef struct S5_LOCAL_BITS {
 unsigned int mode : 2;
 unsigned int side_active : 1;
 signed int side_angle : 8;
} S5_SCAN_CONTROL;
/* 8/2 are private unset sentinels beyond the 3/1-bit message ranges. */
#define S5_PROFILE_ID_NONE 8U
#define S5_PROFILE_PHASE_NONE 2U
typedef struct S5_LOCAL_BITS {
 unsigned int n : 7;
 unsigned int cur_id : 4;
 unsigned int cur_phase : 2;
 unsigned int limit : 8;
 unsigned int aborted : 1;
} S5_PROFILE_STATE;
typedef struct S5_LOCAL_BITS {
 unsigned int seen_run : 2;
 unsigned int clear_run : 2;
 unsigned int bad_run : 4;
 unsigned int last_seq : 4;
 unsigned int seen : 1;
 unsigned int ready : 1;
 unsigned int corrected : 1; /* one heading correction per pass */
} S5_PASS_STATE;
#endif
