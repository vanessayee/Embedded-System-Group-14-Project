# Buddy 1 minimum bit representation

This report covers the Buddy 1 implementation on 20 November 2026. It compares
its saved pre-change firmware with the normal SMP USB CDC WiFi/lwIP/MQTT build.
Buddies 2 to 5 are not modified by this stage. Their representations remain
subject to their own audits. This is not a claim of full project compliance.

## Storage and ownership

C bit-fields are used only for records owned by one task, including local
encode/decode scratch records. They are never copied directly to MQTT or
published as shared structures. The explicit bit-stream encoder still defines
all wire positions. The GCC local layout is size-checked on the host and Pico;
MinGW uses gcc_struct to match ARM GCC allocation rather than its MS default.

| Stored information | Valid range or meaning | Required bits | Implementation |
|---|---|---:|---|
| Connection state | 8 states, 0..7 | 3 | COMM_FSM.state |
| Elapsed time | unsigned modulo-2^32 milliseconds | 32 | COMM_FSM.in_state_ms; unchanged wrap arithmetic |
| Retry delay | integer milliseconds, up to 16000 | 14 | COMM_FSM.backoff_ms |
| Failed attempts | 0..255, saturating | 8 | COMM_FSM.attempts; every-third recovery uses the full counter |
| Reconnect count | 0..255, saturating | 8 | COMM_FSM.reconnects; clamp to 4 bits on publication |
| Previously online | Boolean | 1 | COMM_FSM.was_online |
| WiFi loss | Boolean | 1 | COMM_FSM.wifi_loss; current FSM observation, shared fault uses SYS_FLT_WIFI_LOSS |
| MQTT loss | Boolean | 1 | COMM_FSM.mqtt_loss; shared fault uses SYS_FLT_MQTT_LOSS |
| Reconnect pending | Boolean | 1 | COMM_FSM.reconnect_req; task-local request |
| Remembered command IDs | 4 full byte IDs | 4 x 8 | dup_ids[4] |
| Duplicate count | 0..4 | 3 | S1_RX_STATE.dup_n |
| Duplicate cursor | 0..3 | 2 | S1_RX_STATE.dup_next |
| Commands counted in window | 0..5 | 3 | S1_RX_STATE.rate_count; reject without increment once full |
| Previous telemetry available | Boolean | 1 | S1_TLM_STATE.have_prev |
| Previous avoidance count | 0..15, source counter modulo 16 | 4 | S1_TLM_STATE.prev_avoid_count |
| Boot fault report sent | Boolean | 1 | S1_TLM_STATE.boot_report_sent |
| Boot identity marker sent | Boolean | 1 | S1_TLM_STATE.boot_marker_sent |
| Telemetry, event, heartbeat and ACK sequences | modulo 256 | 8 each | tlm_seq, evt_seq, hb_seq, ack_seq |
| Boot identity | 0..65535 | 16 | boot_token |
| Boot, heartbeat, tick and rate-window times | unsigned modulo-2^32 milliseconds | 32 each | boot_marker_ms, hb_ms, last_tick_ms, rate_start_ms; preserve wrap-safe subtraction and delayed activations |
| Drop/reject totals | full unsigned modulo-2^32 diagnostic totals | 32 each | s1_tlm_dropped, tx_dropped, s1_rx_rejected; saturate only the 8-bit reported values |
| Communication publication | state 3, availability 2, link 2, session 2, attempts 4, reconnects 4 | 17 | s1_comm_pack; existing single aligned 32-bit word |
| Communication snapshot | state 3, availability 2, link 2, session 2, raw attempts 8, raw reconnects 8, drops 8, rejects 8 | 41 | 6-byte S1_COMM_STATUS; accepts full counters before publication clamps them |
| Heartbeat scratch record | uptime 16, mode 3, comm 3, critical 1, faults 5, raw reconnects 8 | 36 | 5-byte HB_RECORD; raw reconnects must retain values such as 200 until the encoder saturates to 15 |
| Event scratch record | code slot 4 and event-specific data union 32 | 36 | 5-byte EVT_RECORD; full 32-bit body required by boot token + uptime |
| Command scratch record | command 3, untrusted v1 argument slot 10, ID 8 | 21 | 3-byte CMD_RECORD; validated application argument uses 9 bits |
| ACK scratch record | ID 8, command 3, result 4, detail 5 | 20 | 3-byte ACK_RECORD |
| Parsed frame header | version nibble 4, type nibble 4, sequence 8, untrusted length 8 | 24 | 3-byte MP_HDR; retain invalid input until validation |

Telemetry storage cur and prev each use the 279 field bits below, with one tail
padding bit: 35 bytes. Values retain the same units, sentinels and resolutions.

| # | Field | Off | Bits | Signed | Unit / scale | Valid range | Special | Producer |
|---|---|---|---|---|---|---|---|---|
| 1 | uptime_ds | 0 | 16 | no | 0.1 s | 0..6553.5 s, wraps | — | B1 |
| 2 | mode | 16 | 3 | no | INIT, NORMAL, NAV_ACTION, OBSTACLE, SAFE_STOP | 0..4 | 5..7 unused | B2 |
| 3 | epoch | 19 | 3 | no | mode epoch | 0..7 | — | B2 |
| 4 | faults | 22 | 27 | no | fault bitmask (section 6) | bits 0..26 | — | all |
| 5 | mstate | 49 | 2 | no | IDLE, STRAIGHT, TURNING, FAULT | 0..3 | — | B2 |
| 6 | msource | 51 | 2 | no | LINE, BARCODE, AVOID, SUPER | 0..3 | — | B2 |
| 7 | cmd_active | 53 | 1 | no | flag | 0/1 | — | B2 |
| 8 | speed_l | 54 | 11 | yes | mm/s | −1023..1023 | −1024 = no encoder data | B2 |
| 9 | speed_r | 65 | 11 | yes | mm/s | −1023..1023 | −1024 = no encoder data | B2 |
| 10 | odometer_cm | 76 | 16 | no | cm | 0..65535, saturates | — | B2 |
| 11 | heading2 | 92 | 8 | no | 2° (encoder heading, + clockwise) | 0..179 (0..358°) | — | B2 |
| 12 | cruise10 | 100 | 6 | no | 10 mm/s (cruise setting, not a wheel target) | 0..63 (0..630 mm/s) | — | B2 |
| 13 | ir_black | 106 | 3 | no | bit0 IR_L (GP2), bit1 IR_C (GP6), bit2 IR_R (GP26, also the barcode sensor); 1 = black; see §11 for the car's sides | 0..7 | — | B3 |
| 14 | line_pos | 109 | 3 | no | CENTER, LEFT_NEAR, LEFT_FAR, RIGHT, LOST_LEFT, LOST_RIGHT, JUNCTION, RIGHT_NEAR (line follower's own IR_L/IR_R labels) | 0..7 | — | B3 |
| 15 | line_state | 112 | 2 | no | ON_LINE, SEARCHING, LOST, UNKNOWN | 0..3 | 3 = not calibrated | B3 |
| 16 | steer | 114 | 7 | yes | steer units (+ = right) | −63..63 | — | B3 |
| 17 | bc_state | 121 | 2 | no | IDLE, CAPTURING, DECODING, EXECUTING | 0..3 | — | B3 |
| 18 | bc_symbol | 123 | 6 | no | Code 39 index (§5.1.1) | 0..43 | 63 = none | B3 |
| 19 | bc_result | 129 | 2 | no | NONE, VALID, INVALID, DUPLICATE | 0..3 | — | B3 |
| 20 | nav | 131 | 2 | no | LEFT, RIGHT, STRAIGHT, U_TURN | 0..3 | valid only if nav_valid | B3 |
| 21 | nav_valid | 133 | 1 | no | flag | 0/1 | — | B3 |
| 22 | bc_valid | 134 | 6 | no | count of valid reads | mod 64 | — | B3 |
| 23 | bc_invalid | 140 | 6 | no | count of invalid reads | mod 64 | — | B3 |
| 24 | imu_status | 146 | 2 | no | INIT, CALIBRATING, OK, FAULT | 0..3 | — | B4 |
| 25 | pitch | 148 | 8 | yes | 1° (nose up +) | −90..90 | −128 = invalid | B4 |
| 26 | roll | 156 | 8 | yes | 1° (right side down +) | −90..90 | −128 = invalid | B4 |
| 27 | yaw_rate | 164 | 9 | yes | 1 °/s (left / CCW +); LSM303DLHC: change of the magnetometer heading over 200 ms | −255..255 | −256 = invalid | B4 |
| 28 | heading | 173 | 9 | no | 1°, + counter-clockwise from calibration (LSM303DLHC: magnetometer, no gyroscope) | 0..359 | 511 = invalid | B4 |
| 29 | motion_class | 182 | 2 | no | STATIONARY, MOVING, TURN_LEFT, TURN_RIGHT | 0..3 | — | B4 |
| 30 | tilt | 184 | 1 | no | flag | 0/1 | — | B4 |
| 31 | collision | 185 | 1 | no | latched until CLEAR / RESUME | 0/1 | — | B4 |
| 32 | shock | 186 | 5 | no | 0.5 g (peak horizontal impact) | 0..31 (0..15.5 g) | — | B4 |
| 33 | hump_last | 191 | 6 | no | 2 mm | 0..50 (0..100 mm) | 63 = none yet | B4 |
| 34 | hump_max | 197 | 6 | no | 2 mm, highest since boot | 0..50 (0..100 mm) | 63 = none yet | B4 |
| 35 | hump_count | 203 | 4 | no | count | 0..15, saturates | — | B4 |
| 36 | hump_active | 207 | 1 | no | flag | 0/1 | — | B4 |
| 37 | us_dist | 208 | 10 | no | 5 mm, latest ranging at the current servo angle (forward only while monitoring, §11) | 0..800 (0..4000 mm) | 1022 = no echo (clear), 1023 = invalid | B5 |
| 38 | us_result | 218 | 2 | no | OK, NO_ECHO, NO_RESPONSE, INVALID | 0..3 | — | B5 |
| 39 | avoid_state | 220 | 4 | no | IDLE, STOPPING, COARSE, FINE, PLANNING, EXECUTING, SEARCHING, ALIGNING, FAILED | 0..8 | 9..15 unused | B5 |
| 40 | avoid_side | 224 | 2 | no | NONE, LEFT, RIGHT | 0..2 | — | B5 |
| 41 | avoid_outcome | 226 | 2 | no | NONE, SUCCESS, FAILED, CLEARED | 0..3 | — | B5 |
| 42 | obst_width | 228 | 8 | no | 5 mm | 0..254 (0..1270 mm) | 255 = unknown | B5 |
| 43 | obst_bearing | 236 | 8 | yes | 1° (+ = right) | −127..127 | — | B5 |
| 44 | clear_left | 244 | 8 | no | 10 mm | 0..253; 254 = ≥ 2.54 m | 255 = unknown (never free) | B5 |
| 45 | clear_right | 252 | 8 | no | 10 mm | as clear_left | 255 = unknown | B5 |
| 46 | comm_state | 260 | 3 | no | INIT, CONNECTING_WIFI, CONNECTING_MQTT, SUBSCRIBING, ONLINE, RETRY_WAIT, RECOVERY, DISABLED | 0..7 | — | B1 |
| 47 | tlm_dropped | 263 | 8 | no | frames dropped (queues) | 0..255, saturates | — | B1 |
| 48 | rx_rejected | 271 | 8 | no | command frames rejected | 0..255, saturates | — | B1 |

Fields 42–45 are all zero until the first obstacle profile (the profile
words start at zero), which reads as a 0 mm obstacle straight ahead.

Fields come from each owner's interface (motion
status words, line and barcode status words, IMU snapshot, hump word,
scan/avoid status words, obstacle profile). They are mapped in
`tlm_build.c` without changing scale, except the three profile fields
(width, clearances), which are compacted there. Integration test INT-15
checks every field at its limit and at its invalid code, from each
subsystem's pack function to the decoded server value.

#### 5.1.1 Code 39 symbol index

`0`..`9` → 0..9, `A`..`Z` → 10..35, `-` 36, `.` 37, space 38, `$` 39,
`/` 40, `+` 41, `%` 42, `*` 43. 63 = no symbol.

TLM_SOURCES is a 144-byte engineering snapshot produced by getters from all
five buddies. Its borrowed sub-records retain their owning buddies' definitions;
it is not sent or treated as a packed shared object. Wider arithmetic locals,
bit-stream cursors, kernel IDs and pointers are calculation/infrastructure
values, not separate Boolean/state application storage. Preserve their required
precision; do not truncate before validating untrusted fields.

## Shared access

Shared communication state is still an aligned word published with app_store and
read with app_load. The 2-bit session and 1-bit reconnect bridge request retain
separate aligned words: the bridge exchanges the request atomically, while the
session has a separate writer. A direct shared C bit-field update would be an
unsafe read-modify-write. Diagnostic counters retain their original one-writer
word publication. The firmware-wide fault mask remains in Buddy 2 and uses
atomic OR/clear/exchange. No task/ISR ownership is changed here.

## Measured sizes

ARM GCC 15.2, normal profile smp1-usb_cdc-wificyw43-netlwip-mqtt1-crash0-test0.
Sizes are from arm-none-eabi-size and ARM-compiled sizeof probes, not estimates
of application field types. See testing/subsystem1/logs/2026-11-20_minimum_bits.txt.

| Item | Before bytes | After bytes |
|---|---:|---:|
| COMM_FSM | 32 | 9 |
| TLM_RECORD, each | 60 | 35 |
| S1_COMM_STATUS | 8 | 6 |
| HB_RECORD | 8 | 5 |
| EVT_RECORD | 8 | 5 |
| CMD_RECORD | 6 | 3 |
| ACK_RECORD | 4 | 3 |
| MP_HDR | 4 | 3 |
| RX counter/cursor state | 12 | 1 |
| Telemetry flags plus previous avoidance count | 13 | 1 |
| TLM_SOURCES including ABI padding | 144 | 144 |
| Normal .data | 128 | 128 |
| Normal .bss | 85348 | 85220 |
| Normal static RAM, .data + .bss | 85476 | 85348 |
| Normal flash text | 389038 | 391526 |

Static RAM falls by 128 bytes. Flash text rises by 2488 bytes because accesses
to compact fields require additional instructions on Cortex-M0+. Queue buffers
are dynamically allocated from the kernel's fixed heap, already counted in
.bss: their 288-byte reduction increases available heap and must not be counted
as another reduction in .bss.

## Message size comparison

| Item | Before bytes | After bytes |
|---|---:|---:|
| Telemetry Collection to Communication, actual largest message | 41 | 41 |
| Communication to MQTT bridge, actual largest message | 41 | 41 |
| Configured maximum outbound queue message | 65 | 41 |
| Four telemetry queue slots, including kernel per-message overhead/alignment | 288 | 192 |
| Eight MQTT TX queue slots, same overhead/alignment | 576 | 384 |
| RX queue maximum, deliberately retaining malformed-frame handling | 64 | 64 |
| MQTT telemetry payload / full application frame | 35 / 40 | 35 / 40 |
| MQTT heartbeat payload / full frame | 4 / 9 | 4 / 9 |
| MQTT event payload / full frame | 5 / 10 | 5 / 10 |
| MQTT command payload / full frame | 3 / 8 | 3 / 8 |
| MQTT ACK payload / full frame | 3 / 8 | 3 / 8 |

Outbound queues retain 4 and 8 slots and the same drop-oldest policy. Sizing uses
APP_MBF_BUFSZ(n,sz)=n*(4+round_up_to_4(sz)). A frame bigger than the current
largest encoded frame is rejected before copying to a smaller local buffer.

The command argument has 9 application bits, sufficient for speed 400 and all
26 letter mappings. The next bit remains zero compatibility padding at its
original v1 location, so command IDs, CRCs and every accepted frame remain
unchanged. Nonzero padding is rejected as MPE_ARG; it is never silently masked
into a valid argument. Commands retain three final padding bits. No dashboard
or backend deployment is needed for this compatible change.

## Correctness and performance evidence

- 313 host checks pass with -Wall -Wextra -Werror, including six new storage,
  command-flood, counter-saturation and invalid-padding checks.
- 200000 connection-FSM steps match the saved baseline: states, elapsed time,
  backoff, counts, fault observations, reconnect request and availability.
- 10000 telemetry records and their JSON forms match byte for byte, including
  signed field boundaries and invalid sentinel codes.
- All 4096 command code/argument combinations over 0..511 match the baseline
  bytes and acceptance/error results. The bit-512 rejection has its own test.
- All 43 backend unit tests and 14 standalone codec checks pass. Existing
  golden telemetry/heartbeat/command/event/ACK vectors are unchanged.
- TEST=1 and TEST=0 ARM firmware builds pass. The self-test image includes the
  new storage tests, but has not been executed on the Pico in this change.

On-target timing and course tests remain outstanding. Do not infer unchanged
execution time from unchanged outputs or host timing. Use the existing
S1_STAT_REPORT_S instrumentation and compare telemetry execution maximum,
motion/line/IMU periods and overruns against the saved pre-change firmware with
the same hotspot and MQTT conditions. Confirm STOP, RESUME, all 26 dashboard
mappings, barcode events, hump events and obstacle events on hardware before the
demonstration. No physical test or live robot command was performed here.
