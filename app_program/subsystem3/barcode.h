/*
 *	barcode.h
 *	Code 39 symbol decoding (Buddy 3).  Pure logic, host-testable.
 *
 *	Evidence (subsystem3/docs/BARCODE_ANALYSIS.md): every barcode supplied
 *	with the brief -- Barcode Sample.pdf ("A", "Z"), BARCODE.pptx ("A",
 *	"F", "Z", "W", "X", "Y") and the photo in Barcode & Line
 *	Specification.pdf -- measures as 3 symbols of 9 elements (5 bars, 4
 *	spaces) with exactly 3 wide elements, wide:narrow = 3:1, a narrow
 *	inter-character gap, and decodes as '*' + one character + '*'.  That
 *	is the Code 39 structure; the table below is generated from the
 *	Code 39 rule (bar pattern x wide-space group) rather than typed in,
 *	and the unit tests check it against the measured sample widths.
 *
 *	Symbol index (6 bits, 0..43): '0'..'9' = 0..9, 'A'..'Z' = 10..35,
 *	'-' 36, '.' 37, ' ' 38, '$' 39, '/' 40, '+' 41, '%' 42, '*' 43.
 *	C39_NONE (63) = no symbol.
 *
 *	Pattern: 9 bits, element 1 (first bar) = bit 8 ... element 9 = bit 0,
 *	1 = wide.
 */

#ifndef BARCODE_H
#define BARCODE_H

#include <tk/typedef.h>

#define C39_NSYM		44
#define C39_STAR		43
#define C39_NONE		63
#define C39_ELEMENTS		9	/* per symbol: b s b s b s b s b */
#define C39_STRIDE		10	/* symbol + inter-character gap  */
#define C39_SINGLE_ELEMENTS	29U
#define C39_MERGED_ELEMENTS	27U	/* one group of three merged into one */

/* Decoder errors (negative) */
#define C39_ERR_COUNT		(-1)	/* element count is not 10k-1, k >= 3 */
#define C39_ERR_WIDTH		(-2)	/* wide/narrow ambiguous              */
#define C39_ERR_PATTERN		(-3)	/* not a Code 39 pattern              */
#define C39_ERR_STARTSTOP	(-4)	/* no '*' at both ends / '*' inside    */
#define C39_ERR_GAP		(-5)	/* inter-character gap too wide       */
#define C39_ERR_SPACE		(-6)	/* output buffer too small            */

/* Minimum wide/narrow ratio accepted, x10 (nominal print ratio is 3:1) */
#define C39_MIN_RATIO_X10	15
#define C39_MIN_BAR_RATIO_X10	13	/* tolerate compressed wide/narrow bar timing */

IMPORT UH   c39_pattern(UINT index);		/* 0x1FF on bad index */
IMPORT char c39_char(UINT index);		/* '?' on bad index   */
IMPORT INT  c39_index_of_char(char c);		/* -1 if not Code 39  */
IMPORT INT  c39_lookup(UH pattern);		/* index or -1        */

/* Classify 9 element widths (any linear unit) into a 9-bit pattern.
   Bars are compared with bars and spaces with spaces, so a constant
   black/white bias (ink spread, IR spot blur, threshold offset) cancels.
   Returns the pattern, or C39_ERR_WIDTH. */
IMPORT INT  c39_classify(const UW w[C39_ELEMENTS]);

/* Maximum gap based on the median narrow white space of a symbol. */
IMPORT UW c39_gap_limit(const UW w[C39_ELEMENTS], UH pattern);

/* Decode a whole element run (bar, space, bar, ... bar).  Tries the
   forward order, then the reversed order (car driving the other way).
   A 27-element capture can recover one merged run in an uppercase label.
   data[] receives the data symbols between the '*' delimiters.
   Returns the number of data symbols (>= 1) or a C39_ERR_* code. */
IMPORT INT  c39_decode(const UW *el, UINT n, UB *data, UINT max_data,
		       BOOL *reversed);


/*
 *	barcode.h
 *	Barcode capture and decoding state machine (Buddy 3), fed by the edge
 *	timestamps recorded by the IR_RIGHT (GP26) interrupt (Tribunal 1,
 *	Version B: interrupt edge capture, asynchronous decoding).  Pure
 *	logic, host-testable.
 *
 * Element widths are GPIO transition durations in microseconds. Code 39
 * uses relative narrow/wide timing within each symbol. Decoding is independent
 * of wheel speed and odometry.
 * Start after white, finish after white lasting 2.5 times the longest
 * element (at least 100 ms). A 3 s edge timeout bounds stalled captures.
 * Keep relative sensor/print speed reasonably steady within each symbol.
 */



#define BC_MAX_EDGES		64	/* 4 data symbols: 6*10-1 elements */
#define BC_MAX_DATA		4

/* Capture timing is defined in barcode.c; no wheel-distance limits apply. */
#define BC_END_QUIET_X10	25	/* ... and >= 2.5 x widest element */
#define BC_IDLE_TIMEOUT_US	3000000	/* no edge for 3 s: car stopped   */

/* States (2 bits in the barcode status word) */
#define BCS_IDLE		0U
#define BCS_CAPTURING		1U
#define BCS_DECODING		2U	/* used by the task while executing */
#define BCS_EXECUTING		3U

/* Results (2 bits) */
#define BCR_NONE		0U
#define BCR_VALID		1U
#define BCR_INVALID		2U
#define BCR_DUPLICATE		3U

/* Capture failures beyond the C39_ERR_* codes */
#define BC_ERR_LONG		(-10)	/* element longer than a barcode bar */
#define BC_ERR_OVERFLOW		(-11)	/* too many edges                    */
#define BC_ERR_WINDOW		(-12)	/* reserved legacy error code        */
#define BC_ERR_STALLED		(-13)	/* no edge for too long              */
#define BC_ERR_ABORTED		(-14)	/* aborted by the caller             */

typedef struct {
	UB	result;			/* BCR_* */
	UB	n_data;
	UB	data[BC_MAX_DATA];	/* Code 39 indices */
	BOOL	reversed;
	INT	err;			/* when INVALID */
	UB	n_elements;		/* after glitch merging */
	UB	n_diag;			/* widths available for serial diagnosis */
	UW	element_us[BC_MAX_EDGES]; /* measured durations in us */
} BC_RESULT;

typedef struct {
	UB	state;
	UB	level;			/* 1 = black under the sensor */
	BOOL	level_known;
	UW	t_last_us;		/* last edge */
	UW	edge_t[BC_MAX_EDGES];
	UINT	n_edges;
	BOOL	have_last;
	UB	last_sym;
	UW	widest_us;
	UW	last_decode_us;		/* time-based duplicate suppression */
} BC_DEC;

IMPORT void bc_init(BC_DEC *d);
IMPORT void bc_abort(BC_DEC *d, BC_RESULT *res);
IMPORT BOOL bc_capturing(const BC_DEC *d);

/* One digital edge: black is the level AFTER the edge. */
IMPORT void bc_edge(BC_DEC *d, UW t_us, UB black, BC_RESULT *res);

/* Periodic check without an edge: completion and stall */
IMPORT void bc_poll(BC_DEC *d, UW now_us, BC_RESULT *res);


#endif /* BARCODE_H */
