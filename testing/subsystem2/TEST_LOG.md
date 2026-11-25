# Subsystem 2 — test log

**Subsystem:** Motion control, motor arbitration and vehicle mode  ·  **Student ID:** 2502591  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for the project test and result-recording rules.

## Test cases

Define each test before running it. Test IDs are permanent. These automated checks are implemented in `app_program/subsystem2/s2_unit_tests.c` and `subsystem2_test.c`.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | command word round trip at field limits (arg -2048) | IF-M1..M3 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U01 PASS` |
| S2-U01b | command word arg +2047 | IF-M1..M3 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U01b PASS` |
| S2-U02 | reserved bits, unknown type and source rules rejected | IF-M1..M3 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | argument ranges enforced at the boundaries | IF-M1..M3 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U03 PASS` |
| S2-U04 | result, steering (clamped -63) and speed-cap words | IF-M1..M3 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U04 PASS` |
| S2-U05 | motion status 2-word pack/unpack at limits | IF-M1..M3 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U05 PASS` |
| S2-U06 | zero error: output = feed-forward | motion control | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U06 PASS` |
| S2-U07 | saturated output: integrator frozen (anti-windup) | motion control | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U07 PASS` |
| S2-U08 | turn arc for 90 deg = pi*track/4 using the measured wheel spacing | motion control | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U08 PASS` |
| S2-U08a | unpowered simulated wheels decay to rest without integer-rounding drift | motion control | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U08a PASS` |
| S2-U09 | MOVE 500 mm: DONE, within 25 mm, wheels within 15 mm | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U09 PASS` |
| S2-U09a | MOVE accepted in Obstacle Handling | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U09a PASS` |
| S2-U09b | bounded AVOID reverse accepted only in Obstacle Handling | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U09b PASS` |
| S2-U09c | negative MOVE drives both wheels backward and completes | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U09c PASS` |
| S2-U10 | TURN +90: left forward, right backward, arc within 12 mm | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10 PASS` |
| S2-U10b | barcode RIGHT: equal wheel arcs, right-leading contact then confirmed white gap | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10b PASS` |
| S2-U10c | barcode LEFT: cross the centre black edge and stop in the centre-right white gap | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10c PASS` |
| S2-U10d | both sensors black together do not confirm the right branch | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10d PASS` |
| S2-U10e | line seen only far from 90 deg: fault at the angle limit | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10e PASS` |
| S2-U10f | no fresh IR word: stop at encoder target and report unconfirmed turn | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10f PASS` |
| S2-U10g | barcode U-turn: no advance, turns on the spot | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10g PASS` |
| S2-U10h | barcode LEFT: missing tape faults at the angle limit | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10h PASS` |
| S2-U10i | left turn uses the recorded GP2 exit 7 mm before command start | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10i PASS` |
| S2-U10j | barcode LEFT: GP2 alone cannot falsely complete the turn | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10j PASS` |
| S2-U10k | barcode LEFT: late centre contact must release before turn completion | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10k PASS` |
| S2-U10l | barcode LEFT: centre remaining black cannot report corridor alignment | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10l PASS` |
| S2-U10m | barcode LEFT: early crossing contact cannot validate later white space | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10m PASS` |
| S2-U10n | barcode LEFT: both-black junction cannot validate a white corridor | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10n PASS` |
| S2-U10o | left and right physically advance the same distance before pivoting | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10o PASS` |
| S2-U10p | mirrored sensor order produces mirrored left/right wheel paths and equal turn angle | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10p PASS` |
| S2-U10q | LEFT outgoing line before 70 encoder degrees is accepted after clearing junction | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10q PASS` |
| S2-U10r | RIGHT uses the same early contact acquisition with mirrored sensors | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10r PASS` |
| S2-U10s | final alignment sustains motor drive instead of ineffective one-tick pulses | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10s PASS` |
| S2-U11 | queued TURN starts straight from creep speed (no full stop) | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U11 PASS` |
| S2-U11a | imbalanced avoidance pivots retain controllable wheel speed in both directions | IF-M1..M3, NFR12 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U11a PASS` |
| S2-U12 | AVOID command rejected outside Obstacle Handling | IF-M1, IF-M4 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U12 PASS` |
| S2-U13 | BARCODE command rejected in Obstacle Handling | IF-M1, IF-M4 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U13 PASS` |
| S2-U14 | command from an older mode epoch rejected as stale | IF-M1, IF-M4 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U14 PASS` |
| S2-U15 | nothing accepted in INIT or SAFE_STOP | IF-M1, IF-M4 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U15 PASS` |
| S2-U16 | bounded command queue: overflow rejected (QUEUE) | IF-M1, IF-M4 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U16 PASS` |
| S2-U17 | STOP bypass aborts the running command (ABORTED, nav finished) | IF-M1, IF-M4 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U17 PASS` |
| S2-U18 | stop(): PWM forced to zero in the very next period | IF-M1, IF-M4 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U18 PASS` |
| S2-U19 | stall (duty, no ticks for 400 ms): FAULT result and MOTOR_STALL | NFR6, FR8, IF-M2/IF-M5 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U19 PASS` |
| S2-U20 | FAULT -> IDLE only after 300 ms clear; motors off meanwhile | NFR6, FR8, IF-M2/IF-M5 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U20 PASS` |
| S2-U21 | 3rd consecutive stall raises MOTION_ESCALATED (bounded recovery) | NFR6, FR8, IF-M2/IF-M5 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U21 PASS` |
| S2-U22 | one wheel without ticks while driven: motion FAULT raised | NFR6, FR8, IF-M2/IF-M5 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U22 PASS` |
| S2-U23 | positive steer turns right (left wheel faster) | NFR6, FR8, IF-M2/IF-M5 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U23 PASS` |
| S2-U24 | stale steering: the car stops instead of driving blind | NFR6, FR8, IF-M2/IF-M5 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U24 PASS` |
| S2-U25 | obstacle speed cap limits line-following speed (FR8) | NFR6, FR8, IF-M2/IF-M5 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U25 PASS` |
| S2-U26 | steering ignored outside Normal Navigation | NFR6, FR8, IF-M2/IF-M5 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U26 PASS` |
| S2-U27 | SAFE_STOP: zero motor output whatever the inputs | NFR6, FR8, IF-M2/IF-M5 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U27 PASS` |
| S2-U28 | INIT waits for both IR and IMU calibration | vehicle mode, IF-S1/IF-S2 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U28 PASS` |
| S2-U29 | not armed (test image): stays in INIT | vehicle mode, IF-S1/IF-S2 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U29 PASS` |
| S2-U30 | INIT -> NORMAL when calibrated and armed | vehicle mode, IF-S1/IF-S2 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U30 PASS` |
| S2-U31 | barcode command -> NAVIGATION ACTION | vehicle mode, IF-S1/IF-S2 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U31 PASS` |
| S2-U32 | obstacle during a barcode action -> OBSTACLE, new epoch | vehicle mode, IF-S1/IF-S2 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U32 PASS` |
| S2-U33 | avoidance done + line found -> NORMAL | vehicle mode, IF-S1/IF-S2 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U33 PASS` |
| S2-U34 | critical fault (collision) -> SAFE_STOP | vehicle mode, IF-S1/IF-S2 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U34 PASS` |
| S2-U35 | SAFE_STOP holds; events cannot leave it | vehicle mode, IF-S1/IF-S2 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U35 PASS` |
| S2-U36 | validated RESUME -> INIT with calibration required again | vehicle mode, IF-S1/IF-S2 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U36 PASS` |
| S2-U37 | initialisation timeout -> SAFE_STOP with SYS_FLT_INIT | vehicle mode, IF-S1/IF-S2 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U37 PASS` |
| S2-U38 | severity: network/line-lost/stall degraded; FR12 and escalation critical | vehicle mode, IF-S1/IF-S2 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U38 PASS` |
| S2-U39 | condition faults clear themselves; event faults need CLEAR | vehicle mode, IF-S1/IF-S2 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U39 PASS` |
| S2-U40 | stop for an obstacle: wheels coast to rest, no reverse duty (no runaway backwards) | IF-M1/IF-M3/IF-M4, NFR6 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U40 PASS` |
| S2-U41 | microsecond TIMER runs at 1 MHz against the kernel clock | IF-M1/IF-M3/IF-M4, NFR6 | auto (target only) | `subsystem2/subsystem2_test.c` | `[TEST] S2-U41 PASS` |
| S2-U42 | Motion Control runs every 20 ms and finishes within half its period | IF-M1/IF-M3/IF-M4, NFR6 | auto (target only) | `subsystem2/subsystem2_test.c` | `[TEST] S2-U42 PASS` |
| S2-U43 | test image disarmed: INIT or SAFE_STOP only; motor watchdog never tripped | IF-M1/IF-M3/IF-M4, NFR6 | auto (target only) | `subsystem2/subsystem2_test.c` | `[TEST] S2-U43 PASS` |
| S2-U44 | stop() bypass served by Motion Control within 100 ms (NFR6, software path) | IF-M1/IF-M3/IF-M4, NFR6 | auto (target only) | `subsystem2/subsystem2_test.c` | `[TEST] S2-U44 PASS` |
| S2-U45 | IF-M1/M3 on target: a BARCODE MOVE outside navigation is answered REJECTED (MODE) | IF-M1/IF-M3/IF-M4, NFR6 | auto (target only) | `subsystem2/subsystem2_test.c` | `[TEST] S2-U45 PASS` |
| S2-U46 | remote STOP: DONE, SAFE_STOP and REMOTE_STOP latched within 60 ms | IF-M1/IF-M3/IF-M4, NFR6 | auto (target only) | `subsystem2/subsystem2_test.c` | `[TEST] S2-U46 PASS` |
| S2-U47 | RESUME clears latched events; leaves SAFE_STOP only when no critical condition remains | IF-M1/IF-M3/IF-M4, NFR6 | auto (target only) | `subsystem2/subsystem2_test.c` | `[TEST] S2-U47 PASS` |
| S2-U60 | each of five buddies independently gates startup |  | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U60 PASS` |
| S2-U61 | calibrated local buddies wait indefinitely for hotspot and MQTT |  | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U61 PASS` |
| S2-U62 | MQTT ready automatically releases startup hold |  | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U62 PASS` |
| S2-U63 | later network outage retains existing autonomous policy |  | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U63 PASS` |
| S2-U64 | missing local ultrasonic readiness times out safely |  | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U64 PASS` |
| S2-U65 | one invalid ping after long MQTT wait does not latch INIT fault |  | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U65 PASS` |
| S2-U66 | fresh ping and MQTT release startup after transient sensor failure |  | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U66 PASS` |
| S2-U70 | bounded local records compact without changing shared wire words |  | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U70 PASS` |
| S2-U71 | freshness preserves timestamp wrap, sequence wrap and exact stale boundary |  | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U71 PASS` |
| S2-U72 | three-bit count holds all four queued commands without wrapping |  | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U72 PASS` |
| S2-U73 | vehicle mode, maximum epoch and calibration flags coexist without overlap |  | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U73 PASS` |
| S2-U74 | PID arithmetic retains values beyond motor-specific narrow limits |  | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U74 PASS` |
| S2-U75 | signed duties, both directions and all fault/result flags retain their full range |  | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U75 PASS` |
| S2-U76 | turn sentinel, settle/handoff timers and retry saturation remain representable |  | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U76 PASS` |

**ID issue to resolve before recording U40:** `s2_unit_tests.c` and `subsystem2_test.c` both currently use `S2-U40` for different checks. Treat the U40 result as ambiguous until one code-side test ID is changed and that change is committed. The groupings above retain the current source IDs for traceability.

The target image also runs Subsystem 2 integration checks (`s2_integration_tests.c`). Record those under their `INT-*` IDs in `testing/integration/TEST_LOG.md`, as integration results are shared across subsystem owners.

## How to run for accurate results

1. Commit the exact source you intend to test; record its short hash with the results. Use the same commit for the host and Pico runs.
2. Run the host simulation suite first. From the repository root, with GNU Make and a C compiler available:

   ```sh
   make -C testing/host clean
   make -C testing/host SUITES=s2
   ```

   Require a successful exit and PASS output for the S2 checks. This uses a simulated wheel plant; it does not verify physical motor/encoder behavior or the Pico scheduler.
3. Build the on-target test image from the firmware build directory:

   ```sh
   cd build_make
   make TEST=1 -j8
   ```

   Flash `mtk3pico_smp1_usb_cdc_wifi_test.uf2` to the Pico. The image runs the automated subsystem tests at startup and keeps the motors disarmed.
4. Capture the complete serial console from before resetting the board through the final `[TEST] subsystem2 TOTAL ...` line. Save it as `testing/subsystem2/logs/YYYY-MM-DD_subsystem2_test.txt`. Use the correct serial port and baud/configuration for this firmware; on Windows PuTTY, enable **Session → Logging → All session output**. Start capture before reset so startup checks are not missed.
5. Check that every expected `[TEST] S2-U.. PASS` line appears and there are no FAIL lines. Also inspect the total: other subsystems' tests may run in the same image, so the global total alone does not identify Subsystem 2 failures. Add a result row per executed test ID and link the saved log. Do not record a PASS for a test that did not emit a PASS line.
6. For repeatable hardware timing, use the same board, power source, firmware commit and test setup; allow a clean reset between runs. Pico-only timing results are measurements of that run, not proof of real-world motor accuracy.

`make TEST=1` runs the firmware's full test image, including other subsystems. The host target `SUITES=s2` selects Subsystem 2 unit/integration logic in the host harness; host-only results do not replace the Pico-only S2-U40..U47 checks. Manual car calibration and physical motor accuracy tests are not automated by these suites.

## Results

Add one row for every test result and every run. Never edit or delete old rows. For grouped checks, add rows using the individual emitted test IDs.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-18 | S2-U01 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U01b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U02 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U03 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U04 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U05 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U06 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U07 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U08 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U09a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U09 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U09b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U09c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U10 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U11 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U12 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U13 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U14 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U15 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U16 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U17 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U18 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U19 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U20 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U21 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U22 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U23 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U24 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U25 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U26 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U27 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U40 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies unit check; this ID collides with the Pico runtime ID. |
| 2026-11-18 | S2-U28 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U29 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U30 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U31 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U32 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U33 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U34 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U35 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U36 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U37 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U38 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U39 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-18 | S2-U47 | 42f79b1 (main; tested 4eef391) | Vanessa | FAIL | `logs/2026-11-18_subsystem2_target.txt` | Pico output: SAFE_STOP -> INIT, faults cleared. Likely stale matching result: `motion_wait_result` matches only 4-bit sequence and type; confirm by inspecting returned result. |
| 2026-11-18 | S2-U40 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem2_target.txt` | Target runtime check; ID collides with host unit test. |
| 2026-11-18 | S2-U41 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem2_target.txt` | Pico target run. |
| 2026-11-18 | S2-U42 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem2_target.txt` | Pico target run. |
| 2026-11-18 | S2-U43 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem2_target.txt` | Pico target run. |
| 2026-11-18 | S2-U44 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem2_target.txt` | Pico target run. |
| 2026-11-18 | S2-U45 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem2_target.txt` | Pico target run. |
| 2026-11-18 | S2-U46 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem2_target.txt` | Pico target run. |
| 2026-11-25 | S2-U01 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U01b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U02 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U03 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U04 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U05 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U06 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U07 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U08 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U08a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U09a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U09 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U09b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U09c | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10c | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10s | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10o | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10p | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10q | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10r | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10j | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10h | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10k | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10l | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10m | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10n | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10d | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10e | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10f | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10g | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U10i | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U11 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U11a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U12 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U13 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U14 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U15 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U16 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U17 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U18 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U19 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U20 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U21 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U22 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U23 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U24 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U25 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U26 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U27 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U40 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U28 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U29 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U30 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U31 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U32 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U33 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U34 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U35 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U36 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U37 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U38 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U39 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U60 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U61 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U62 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U63 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U64 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U65 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U66 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U70 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U71 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U72 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U73 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U74 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U75 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | S2-U76 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_host.txt` | Host all-buddies run. |
| 2026-11-25 | all S2-U/M host checks (78 PASS) | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `../integration/logs/2026-11-25_host_after_cleanup.txt` | Re-run after removing unused code (gpio_irq_enable_pin, sys_fault_severity). |
| 2026-11-25 | S2-U41 | 9d0e75f + 25 Nov cleanup | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_target.txt` | Pico target run (TEST=1 image). |
| 2026-11-25 | S2-U42 | 9d0e75f + 25 Nov cleanup | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_target.txt` | Pico target run (TEST=1 image). |
| 2026-11-25 | S2-U43 | 9d0e75f + 25 Nov cleanup | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_target.txt` | Pico target run (TEST=1 image). |
| 2026-11-25 | S2-U44 | 9d0e75f + 25 Nov cleanup | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_target.txt` | Pico target run (TEST=1 image). |
| 2026-11-25 | S2-U45 | 9d0e75f + 25 Nov cleanup | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_target.txt` | Pico target run (TEST=1 image). |
| 2026-11-25 | S2-U46 | 9d0e75f + 25 Nov cleanup | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_target.txt` | Pico target run (TEST=1 image). |
| 2026-11-25 | S2-U47 | 9d0e75f + 25 Nov cleanup | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_target.txt` | Pico target run (TEST=1 image). |
| 2026-11-25 | all other S2-U/M checks on the Pico (78 PASS) | 9d0e75f + 25 Nov cleanup | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem2_target.txt` | Same unit checks as the host run, executed on the RP2040 target; subsystem2 TOTAL 0 failed. |
