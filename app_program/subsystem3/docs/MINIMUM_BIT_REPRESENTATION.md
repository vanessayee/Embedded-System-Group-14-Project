# Buddy 3 minimum bit representation

This stage implements bounded state storage and a compact navigation table for
Buddy 3 on 21 November 2026. It preserves the existing decoder rules, sensor
capture, line tuning, queued junction instructions, word protocols and task
ownership. The baseline already includes the Buddy 1 and Buddy 2 changes.
This report covers Buddy 3, not full project compliance.

## Representation and precision

New C bit-fields belong to a single task or its local working records. They are
never sent directly over MQTT or copied directly into a shared status object.
Explicit masks and shifts in s3_status.c continue to define the shared words.
Word-aligned arithmetic is retained; sizes include ABI padding.

| Field or record | Valid range or meaning | Logical bits | Implementation |
|---|---|---:|---|
| Sensor black polarities | one Boolean for each of three sensors | 3 | LINE_FOLLOW.black_levels; lf_black_level reads a selected bit |
| Calibrated | Boolean | 1 | LINE_FOLLOW.calibrated |
| Line state | four LS states | 2 | LINE_FOLLOW.state |
| Line position | eight LP positions | 3 | LINE_FOLLOW.pos |
| Tracking error | -3..3 | 3 signed | err; preserve the actual negative lost-line error |
| Previous steering | -63..63 | 7 signed | steer_cmd |
| Current/held edge side | includes -3, -1, 0 and +1 | 3 signed each | edge_side, hold_side |
| Remaining correction hold | 0..380 ms | 9 | hold_ms |
| Remaining settle | 0..300 ms | 9 | settle_ms |
| Remaining straight handoff | 0..150 ms | 8 | straight_ms |
| Lost confirmation pending | Boolean | 1 | lost_pending |
| Lost-line turn active | Boolean | 1 | turning |
| Junction active and already reported | Boolean each | 1 each | in_junction, junction_reported |
| Post-turn correction active | Boolean | 1 | post_turn |
| Post-turn pivot side | -1, 0, +1 | 2 signed | post_pivot |
| Edge, lost, turn, junction and post-turn elapsed times | original unsigned 32-bit increments/wrap | 32 each | unchanged; some accumulate indefinitely, and arbitrary dt values must be checked before narrowing |
| Lost/turn odometer references | signed 32-bit millimetres | 32 each | lost_start_mm, turn_start_mm unchanged |
| Line output | steer -63..63, speed fraction 0..15, event mask 0..15, publish flag | 7 signed + 4 + 4 + 1 | LF_OUT; four-byte aligned local container |
| Decoder state | four BCS codes | 2 | BC_DEC.state |
| Current sensor level / known flag | Boolean each | 1 each | level, level_known |
| Capture edge count | 0..64 inclusive | 7 | n_edges; six bits would wrap at the valid full-buffer boundary |
| Previous decoded label exists | Boolean | 1 | have_last |
| Previous symbol | Code 39 indices plus NONE=63 | 6 | last_sym |
| Widest individual captured interval | 0..3000000 microseconds | 22 | widest_us; guard follows BC_IDLE_TIMEOUT_US |
| Last edge/decode timestamps | absolute unsigned modulo-2^32 microseconds | 32 each | t_last_us, last_decode_us unchanged |
| Captured edge timestamps | 64 absolute microsecond timestamps | 64 x 32 | edge_t unchanged; timestamp rollover and resolution are preserved |
| Barcode result | four BCR values | 2 | BC_RESULT.result |
| Result data count | 0..4 | 3 | n_data |
| Read in reverse | Boolean | 1 | reversed; a temporary BOOL remains the addressable decoder output |
| Decoder error | existing errors down to -14, plus 0 | 5 signed | err; four signed bits cannot represent -14 |
| Element/diagnostic counts | 0..63 | 6 each | n_elements, n_diag; at most 64 captured edges yield 63 intervals |
| Diagnostic widths | 64 unsigned widths in microseconds | 64 x 32 | element_us unchanged; merged widths can exceed the individual-edge timeout |
| Result symbol array | four Code 39 indices, each 0..43 | 6 meaningful bits per byte | byte-addressed UB data[4] output retained for c39_decode; two zero high padding bits per valid symbol |
| Barcode snapshot | same 32-bit schema as before | 32 total | S3_BC_STATUS now occupies four bytes locally |
| Line snapshot | same status fields and two 32-bit timestamps | 32 + 64 on publication | bounded local fields packed; raw steer remains signed 8-bit until the existing +/-63 saturation |
| Queued junction instruction | valid, direction, two junction counters | 1 + 2 + 4 + 4 | barcode-task-owned pending record; two-byte container |
| Line task prior-mode flags and status sequence | three Booleans, modulo-16 sequence | 3 + 4 | task-owned tracking byte |
| Calibration count | 0..10 | 4 meaningful bits | byte-addressable counter for the existing pointer helper; reduced from UINT to UB |
| Calibration epochs | 0..7 and existing 255 sentinel | 8 each | UB replaces UW without changing the sentinel or comparison |
| Diagnostic junction/lost counts | original modulo-256 counters | 8 each | retained so console counts are not reduced to the four-bit published values |
| Startup samples and debug previous values | byte-addressed raw levels / existing 255 sentinel | existing byte range | preserve calibration API and first-log behaviour |

Hold/settle/straight configuration limits are checked at compile time. Capture
limits also have guards, so increasing capacity or timeout beyond the represented
range requires changing the widths rather than silently wrapping.

The Code 39 pattern rules, narrow/wide classification, noise merging, 27-element
recovery, forward/reverse handling, duplicate suppression, quiet zones and
scan timeout thresholds are unchanged. The decoder still receives full-width
working arrays and a real BOOL pointer. Only its persistent/result representation
changes. All three supplied barcode scales retain the existing relative-width
classification, with no wheel-speed dependency added.

## Navigation table and concurrency

The runtime table still supports all 43 non-star Code 39 symbols. Each entry has
five values: four navigation actions and UNMAPPED, requiring three bits. Ten
entries occupy each aligned UW; an entry never straddles a word boundary.
Five words hold the table: 20 bytes, compared with 43 words / 172 bytes before.
There are 129 meaningful bits and 31 padding/unused bits in those five words.
The byte-addressed default configuration and its existing sentinel remain.

Readers acquire a complete word and extract one entry. On the Pico SMP build,
updates use the kernel's atomic_cmpxchg in a retry loop, preserving other entries
in the same word. The host uses the equivalent GCC atomic compare/exchange.
The single-core fallback masks interrupts around the update. Reset/clear retain
the existing per-table initialization policy; no new whole-table transaction is
claimed. Default letters and all 26 dashboard overrides keep the same meaning.

The IR capture driver is unchanged: the ISR/task ring retains one packed 32-bit
record per edge (31 timestamp bits plus level), separate aligned head/tail words,
overflow exchange and publication barriers. Do not pack independently written
ISR/task indexes into shared C bit-fields. drv_ir.c/h, s3_status.c, nav_map.h and
subsystem3.h are byte-identical to the baseline. Sensor pins, 1 ms polling and
3 ms stability filtering are unchanged.

## Measured ARM sizes

ARM GCC 15.2, Cortex-M0+, normal profile
smp1-usb_cdc-wificyw43-netlwip-mqtt1-crash0-test0.

| Item | Before bytes | After bytes |
|---|---:|---:|
| LINE_FOLLOW | 92 | 36 |
| LF_OUT | 12 | 4 |
| BC_DEC | 288 | 272 |
| BC_RESULT | 276 | 264 |
| S3_LINE_STATUS | 20 | 16 |
| S3_BC_STATUS | 11 | 4 |
| Runtime navigation table | 172 | 20 |
| Pending instruction state | 7 | 2 |
| Firmware .data | 128 | 128 |
| Firmware .bss | 85068 | 84832 |
| Firmware static RAM | 85196 | 84960 |
| Firmware flash text | 393182 | 394902 |

Measured static RAM falls by 236 bytes. Flash text rises by 1720 bytes. Some
local stack records also shrink, but task stack reservations and kernel queue
allocations are unchanged; do not add local sizeof reductions to the measured
static saving. Borrowing telemetry snapshots can shrink through the updated
public types without changing their values or source code.

## Message comparison

| Shared data or message | Before | After |
|---|---:|---:|
| Line status | 32 bits | 32 bits |
| Line publication timestamps | 2 x 32 bits | 2 x 32 bits |
| Barcode status | 32 bits | 32 bits |
| Sensor capture record | 32 bits | 32 bits |
| Steering word to Buddy 2 | existing 17 meaningful bits in a word | unchanged |
| Navigation command to Buddy 2 | existing 28 meaningful bits in a word | unchanged |
| Line event flags to recovery | existing three bits | unchanged |
| MQTT frame schemas and command mappings | existing Buddy 1 protocol | unchanged |

## Verification

- 327 host checks pass with -Wall -Wextra -Werror, including seven new storage,
  edge-capacity/overflow, timestamp rollover, negative error, neighbour-update,
  timer/steering/polarity and pre-saturation checks.
- Saved pre-change/current implementations match 80000 line transitions:
  hash 14534894818042353407. Resets, calibration, turn completion, changing
  sensor bits and different period increments are included.
- 312 scans cover all 26 letters, both directions, three scales and merged-run
  variants. Before/current hash: 10430111541687861644.
- The original full 320-check suite passes in isolated before/current source
  trees. All 373 test-output lines match, covering barcode-to-junction commands,
  left/right turns, line reacquisition, obstacle priority, startup readiness,
  telemetry, safety and recovery integration.
- 400000 concurrent host updates to adjacent entries in one packed word pass,
  with immediate readback and unchanged neighbouring entries. This exercises
  the production host atomic path; the target uses the kernel CAS primitive.
- All 43 backend tests and 14 standalone codec checks pass.
- Test and normal Pico firmware builds pass without warnings. The single-core
  mapping fallback also compiles as an ARM object; a full single-core firmware
  run is not claimed.
- All 94 files in Buddies 1, 2, 4 and 5 match the comparison snapshot.

## Remaining hardware evidence

The firmware has not been flashed or physically tested in this stage. Matching
software outputs do not establish unchanged execution time: compact field
accesses and synchronized mapping updates add instructions. Compare existing
TASK_STAT maximum times, periods and overruns with the saved baseline on the
Pico. Supervise barcode reads in both directions, dashboard map updates,
left/right/U-turn junctions, normal line tracking and obstacle line reacquisition
before declaring demonstration readiness. No tuning or hardware thresholds were
changed here.
