/*
 *	nav_map.c
 *	Barcode symbol -> navigation command table (see nav_map.h).
 */

#include "nav_map.h"
#include "../subsystem2/motion_msg.h"

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

/* One atomic word per symbol: MQTT and barcode tasks run concurrently. */
LOCAL UW nav_tab[NAV_MAP_SIZE];

EXPORT void nav_map_clear(void)
{
	UINT i;
	for (i = 0; i < NAV_MAP_SIZE; i++) __atomic_store_n(&nav_tab[i], NAV_UNMAPPED, __ATOMIC_RELEASE);
}

EXPORT void nav_map_reset(void)
{
	UINT i;
	nav_map_clear();
	for (i = 0; S3_NAV_MAP_DEFAULT[i].symbol != 0xFFU; i++)
		(void)nav_map_set(S3_NAV_MAP_DEFAULT[i].symbol, S3_NAV_MAP_DEFAULT[i].nav);
}

EXPORT ER nav_map_set(UB symbol, UB nav)
{
	if (nav > NAV_UTURN || symbol >= NAV_MAP_SIZE) return E_PAR;
	__atomic_store_n(&nav_tab[symbol], nav, __ATOMIC_RELEASE);
	return E_OK;
}

EXPORT ER nav_map_remove(UB symbol)
{
	if (symbol >= NAV_MAP_SIZE) return E_PAR;
	__atomic_store_n(&nav_tab[symbol], NAV_UNMAPPED, __ATOMIC_RELEASE);
	return E_OK;
}

EXPORT BOOL nav_map_lookup(UB symbol, UB *nav)
{
	UW value;
	if (symbol >= NAV_MAP_SIZE) return FALSE;
	value = __atomic_load_n(&nav_tab[symbol], __ATOMIC_ACQUIRE);
	if (value > NAV_UTURN) return FALSE;
	*nav = (UB)value;
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
