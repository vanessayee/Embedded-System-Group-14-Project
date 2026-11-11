# Subsystem 3 barcode analysis

This document describes the current implementation in `code39.c`,
`barcode_dec.c`, `drv_ir.c`, `task_barcode.c`, and `nav_map.c`. The decoder is
host-testable; hardware performance and the printed-track setup still need
physical validation.

## Barcode format

The supplied samples were measured as Code 39: nine alternating bar/space
elements per symbol, with five bars and four spaces, and a narrow
inter-character gap. The measured narrow-to-wide ratio is approximately 1:3.
The sample start and stop symbol is `*`; a one-character label therefore has
29 captured runs (three nine-element symbols and two gaps). These measurements
are the basis for using Code 39, rather than an assumption based only on the
printed label.

`code39.c` generates the 44-symbol Code 39 pattern table using the standard
bar-pattern and wide-space groups. The classifier ranks bars against bars and
spaces against spaces, so a constant bias between dark and light durations
does not directly affect the decision. It accepts a minimum overall
wide:narrow ratio of 1.5:1; for bars, the second-widest versus third-widest
comparison uses 1.3:1 to tolerate compressed timing. It checks the start/stop
symbols, forbids `*` as data, validates the inter-character gaps, and tries
the captured run in both directions. A 27-run special case attempts to
recover one missed transition pair in a one-uppercase-letter `*X*` label.

The unit-test vectors in `s3_unit_tests.c` record the measured sample widths.
S3-U01 checks pattern-table properties and measured patterns; S3-U02 checks
the supplied vectors. S3-U03 and S3-U04 cover reverse orientation and changed
scale. S3-U05..U09 cover malformed count, width, pattern/start-stop, and gap
cases. These are software tests, not physical track trials.

## Sensor capture and timing

The barcode sensor is IR_RIGHT on GP26. Its raw level is converted to
black/not-black using the line task's calibrated black polarity. `drv_ir.c`
supports two capture modes:

* Default `S3_CAPTURE_POLL_1MS=1`: RP2040 TIMER alarm 3 samples GP26 every
  1 ms. A candidate level must remain stable for 3 ms before an edge is
  recorded, using the candidate's initial timestamp. This filters brief
  glitches but can miss or distort narrow features depending on sensor speed
  and alignment.
* `S3_CAPTURE_POLL_1MS=0`: the shared GPIO handler records GP26 edge
  timestamps. Each record stores a microsecond timestamp with 2 us resolution
  and the post-edge level. The ISR only enqueues records and wakes the
  consumer task.

Both modes use a 64-entry single-producer/single-consumer ring. If it fills,
overflow is flagged and the barcode task aborts the current capture and sets
`SYS_FLT_BARCODE_OVERFLOW`. `S3_BARCODE_ANALOG` optionally selects ADC0 input
with threshold/hysteresis; analog mode requires 1 ms polling.

The decoder uses **elapsed time in microseconds** between recorded edges as
element width. Its speed and odometry arguments are currently ignored; it
does not convert edge times to distance or compensate for speed changes.
Consequently, vehicle speed must be reasonably steady during one barcode,
and the sample rate/filter must resolve its narrow elements. Previous claims
of encoder-based width conversion, a distance-based capture window, or an
active 450 mm timeout do not describe the current code.

## Capture and decode state machine

`bc_edge()` starts on a white-to-black transition after at least 30,000 us of
white (or treats the first observed black edge as having sufficient prior
quiet). Repeated same-level events are ignored. During capture, a single
element longer than `BC_IDLE_TIMEOUT_US` (3 s) fails as `BC_ERR_LONG`; the
64-edge limit fails as overflow. Interior runs shorter than 1,000 us are
merged with their adjacent runs to remove a missed-transition pair.

While capturing, `bc_poll()` only considers completion after at least one
whole symbol has been observed and the current level is white. It completes
when the trailing quiet interval reaches the larger of 100,000 us or 2.5 times
the widest observed duration, capped at 3 s. If no completion occurs and the
edge quiet period exceeds 3 s, the capture fails as stalled. Although
`barcode_dec.h` defines distance-related limits and lockout fields, the current
`barcode_dec.c` does not enforce a distance window or post-decode start
lockout. Do not treat those declarations as active behavior.

Successful decodes return data symbols between `*` delimiters. The decoder
stores only the first symbol for duplicate comparison. A repeated first
symbol decoded within 2,000,000 us is marked `BCR_DUPLICATE`; this is
time-based, despite the distance-named declarations in the header. Invalid
captures set the invalid result and are reported by the task; they do not
issue a motion command. Line following continues independently.

## Navigation behavior

`nav_map_reset()` installs these current defaults:

| Code 39 symbol | Navigation command |
|---|---|
| A | Right |
| F | Left |
| U | Left |
| X | Straight |
| Y | U-turn |

Other valid Code 39 symbols decode but have no navigation action unless
mapped. `nav_to_motion()` maps left/right/U-turn to Buddy 2 turn commands and
straight to a move of `S3_STRAIGHT_MM`.

The current `S3_NAV_AT_JUNCTION` default is `1`. In Normal mode, a mapped
barcode command is retained until the line follower reports a new junction;
the barcode task then sends the command to Buddy 2 and waits up to
`S3_NAV_WAIT_MS` for its result. A pending command is cleared on init or safe
stop. When `S3_NAV_AT_JUNCTION` is set to `0`, a mapped command is sent as
soon as it is decoded. The barcode task does not directly drive the motors.

S3-U15 checks the default map; S3-U16 checks overrides and rejected inputs;
S3-U17 checks the navigation-to-motion command conversion.

## Validation status and limits

The documented unit tests exercise the decoder, capture logic, navigation
mapping, and line-following logic in software. They do not establish the
physical barcode read rate or reliability. Before relying on barcode
navigation on the vehicle, measure the sensor's black/white output and
recorded edge intervals on the actual print while moving at the intended
speed. In particular, verify that the default 1 ms sampling plus 3 ms stable
filter preserves enough transitions for the narrowest printed element. Also
verify GP26 wiring and check ring overflow during an extended run.

References for the symbology: [Zebra Code 39 character sets](https://docs.zebra.com/us/en/scanners/general/sm72-ig/ascii-character-sets.html) and [Zebra ZPL Code 39 command](https://cpws.zebra.com/cpws/docs/zpl/zpl-zbi2-pm-en.pdf).
