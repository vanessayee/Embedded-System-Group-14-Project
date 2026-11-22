/*
 *	nav_map.c
 *	Barcode symbol -> navigation command table (see nav_map.h).
 */

#include "nav_map.h"
#include "../subsystem2/motion_msg.h"
#if !HOST_TEST
#include "../subsystem2/app_sync.h"
#endif

typedef struct {
	UB	symbol;		/* Code 39 index (barcode.h) */
	UB	nav;		/* NAV_* */
} NAV_ENTRY;

/* Default barcode assignments for Buddy 3. */
LOCAL const NAV_ENTRY S3_NAV_MAP_DEFAULT[] = {
	{ 10, NAV_RIGHT },	/* A */
	{ 32, NAV_LEFT },	/* W */
	{ 35, NAV_LEFT },	/* Z */
	{ 33, NAV_STRAIGHT },	/* X */
	{ 34, NAV_UTURN },	/* Y */
	{ 0xFF, 0 }		/* terminator */
};

/* Ten 3-bit entries per aligned word; no entry crosses a word boundary.
   Five values: LEFT, RIGHT, STRAIGHT, UTURN and UNMAPPED. */
#define NAV_ENTRIES_PER_WORD 10U
#define NAV_WORDS ((NAV_MAP_SIZE + NAV_ENTRIES_PER_WORD - 1U) / NAV_ENTRIES_PER_WORD)
LOCAL UW nav_tab[NAV_WORDS];

LOCAL UW map_load(UINT index)
{
#if HOST_TEST
 return __atomic_load_n(&nav_tab[index], __ATOMIC_ACQUIRE);
#else
 return app_load(&nav_tab[index]);
#endif
}

LOCAL void map_store(UINT index, UW value)
{
#if HOST_TEST
 __atomic_store_n(&nav_tab[index], value, __ATOMIC_RELEASE);
#else
 app_store(&nav_tab[index], value);
#endif
}

/* Preserve other entries if updates from different tasks ever overlap.
   Readers see a complete aligned word; shared C bit-fields are not used. */
LOCAL void map_update(UB symbol, UB value)
{
 UINT index = symbol / NAV_ENTRIES_PER_WORD;
 UINT shift = (symbol % NAV_ENTRIES_PER_WORD) * 3U;
 UW mask = 7UL << shift;
#if HOST_TEST || TK_SUPPORT_SMP
 UW old = map_load(index);
 for (;;) {
  UW next = (old & ~mask) | ((UW)value << shift);
#if HOST_TEST
  if (__atomic_compare_exchange_n(&nav_tab[index], &old, next, FALSE,
                                  __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) return;
#else
  UW seen = atomic_cmpxchg(&nav_tab[index], next, old);
  if (seen == old) return;
  old = seen;
#endif
 }
#else
 UINT saved;
 DI(saved);
 nav_tab[index] = (nav_tab[index] & ~mask) | ((UW)value << shift);
 EI(saved);
#endif
}

EXPORT void nav_map_clear(void)
{
 UINT i;
 UW empty = 0;
 for (i=0;i<NAV_ENTRIES_PER_WORD;i++) empty |= (UW)NAV_UNMAPPED << (i*3U);
 for (i=0;i<NAV_WORDS;i++) map_store(i,empty);
}

EXPORT void nav_map_reset(void)
{
 UINT i;
 nav_map_clear();
 for (i=0;S3_NAV_MAP_DEFAULT[i].symbol!=0xFFU;i++)
  (void)nav_map_set(S3_NAV_MAP_DEFAULT[i].symbol,S3_NAV_MAP_DEFAULT[i].nav);
}

EXPORT ER nav_map_set(UB symbol, UB nav)
{
 if (nav>NAV_UTURN || symbol>=NAV_MAP_SIZE) return E_PAR;
 map_update(symbol,nav);
 return E_OK;
}

EXPORT ER nav_map_remove(UB symbol)
{
 if (symbol>=NAV_MAP_SIZE) return E_PAR;
 map_update(symbol,NAV_UNMAPPED);
 return E_OK;
}

EXPORT BOOL nav_map_lookup(UB symbol, UB *nav)
{
 UW value;
 if (symbol>=NAV_MAP_SIZE) return FALSE;
 value=(map_load(symbol/NAV_ENTRIES_PER_WORD)
        >> ((symbol%NAV_ENTRIES_PER_WORD)*3U)) & 7U;
 if (value>NAV_UTURN) return FALSE;
 *nav=(UB)value;
 return TRUE;
}
EXPORT const char *nav_name(UB nav)
{
	switch (nav) {
	case NAV_LEFT:		return "LEFT";
	case NAV_RIGHT:		return "RIGHT";
	case NAV_STRAIGHT:	return "STRAIGHT";
	case NAV_UTURN:		return "U-TURN";
	default:		return "?";
	}
}

EXPORT void nav_to_motion(UB nav, UW *mcmd_type, INT *arg)
{
	switch (nav) {
	case NAV_LEFT:
		*mcmd_type = MCMD_TURN;
		*arg = -S3_TURN_DEG;
		break;
	case NAV_RIGHT:
		*mcmd_type = MCMD_TURN;
		*arg = S3_TURN_DEG;
		break;
	case NAV_UTURN:
		*mcmd_type = MCMD_TURN;
		*arg = S3_UTURN_DEG;
		break;
	default:
		*mcmd_type = MCMD_MOVE;
		*arg = S3_STRAIGHT_MM;
		break;
	}
}
