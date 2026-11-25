# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-10a–d | a: box 150 mm wide at 300 mm: packed profile -> plan; obstacle event -> OBSTACLE; b: every planned step crosses the interface packed and completes; search stops on the line; c: car edge keeps the configured gap; back on line, heading within 10 deg; d: AVOID_DONE -> NORMAL: line steering drives the car again | IF-M1, IF-M3, IF-S1 | auto, host + target | `subsystem2/s2_integration_tests.c` | each `[TEST]` check PASS |
| INT-14a–d | Buddy 3 line loss/found events during Buddy 5 avoidance; hand-back to NORMAL or Safe Stop | IF-L1, IF-L2, IF-S1 | auto, host + target | `subsystem2/s2_integration_tests.c` | `[TEST] INT-14a-d PASS` |
| INT-18a–c | Critical safety stop overrides Buddy 5 completion and all motion commands | IF-M3, IF-S2 | auto, host + target | `subsystem2/s2_integration_tests.c` | `[TEST] INT-18a-c PASS` |
| INT-11a–d | a: valid unmapped barcode is decoded and reported without motion; b: A/W/Z/X/Y wait for GP6+GP2 junction -> RIGHT/LEFT/LEFT/STRAIGHT/U-TURN -> DONE -> NORMAL; c: barcode status word carries symbol, command and motion result; d: Y at the junction pivots about 180 degrees in place and returns to NORMAL |  | auto, host + target | `subsystem2/s2_integration_tests.c` | each `[TEST]` check PASS |
| INT-12a–b | a: obstacle during a barcode turn: OBSTACLE wins, turn ABORTED, motors off, line loss kept; b: then: stale command, barcode command and speed change refused; Buddy 5's command accepted |  | auto, host + target | `subsystem2/s2_integration_tests.c` | each `[TEST]` check PASS |
| INT-13a–e | a: degraded faults from Buddies 1, 3, 5 are reported; the car keeps driving; b: IMU fault -> SAFE_STOP in one cycle, motors held off; RESUME refused while it persists; c: condition gone: RESUME -> INIT; both recalibrations required before NORMAL; d: collision latched until CLEAR; CLEAR alone never leaves SAFE_STOP, RESUME does; e: jammed wheels: 3 local motion faults, then escalation to SAFE_STOP (no endless cycling) |  | auto, host + target | `subsystem2/s2_integration_tests.c` | each `[TEST]` check PASS |
| INT-15a–b | a: Buddies 1-5 at their limits and invalid codes: all 48 fields reach the server unchanged; b: engineering values arrive within their steps; clearance stays conservative after edge rounding (edges rounded outward, clearance down) |  | auto, host + target | `subsystem1/s1_integration_tests.c` | each `[TEST]` check PASS |
| INT-16a–e | a: 30 mm hump measured by Buddy 4 arrives at the server within +-4 mm; b: after humps of 30, 20, 45 mm: count 3, highest 45 mm within +-4 mm; c: one EVT_HUMP frame carries last, highest and count; d: mode, barcode, obstacle and collision events decode to their fields; no change, no event; e: a failed avoidance (counter unchanged) sends one OBSTACLE event with its reason |  | auto, host + target | `subsystem1/s1_integration_tests.c` | each `[TEST]` check PASS |
| INT-17a–b | a: WiFi lost for 9 s while driving: WIFI_LOSS + RECOVERING reported, not critical; car keeps going; b: link back: automatic reconnection clears the network faults; driving never interrupted |  | auto, host + target | `subsystem2/s2_integration_tests.c` | each `[TEST]` check PASS |
| INT-19a–b | a: motion queue full: the 6th command is refused (QUEUE); the 5 accepted run in order; b: barcode edge buffer full: capture rejected as INVALID (never a command), decoder idle |  | auto, host + target | `subsystem2/s2_integration_tests.c` | each `[TEST]` check PASS |
| INT-20a–d | a: MQTT SET_SPEED 300 -> validated -> supervisory request -> applied, ACK OK; b: same request during avoidance: refused by Buddy 2, ACK carries the reason (MODE); c: out-of-range or corrupted MQTT commands are rejected by Buddy 1, never reach Buddy 2; d: remote STOP accepted in all 5 modes; RESUME refused outside SAFE_STOP |  | auto, host + target | `subsystem2/s2_integration_tests.c` | each `[TEST]` check PASS |
| INT-22a–e | a: 'A' RIGHT at a cross junction: outgoing tape between centre and right after wheel coast; b: LEFT at a cross junction: outgoing tape between centre and right after wheel coast; c: no outgoing line: junction turn faults and holds both motors off; d: after LEFT, Buddy 3 steering drives forward along the outgoing branch; e: after RIGHT, Buddy 3 steering drives forward along the outgoing branch | IF-L1, IF-M1 | auto, host + target | `subsystem2/s2_integration_tests.c` | each `[TEST]` check PASS |
| INT-23 | Operator STOP during obstacle handling (Buddies 1, 2, 5): Stop pressed while the servo is sweeping, and again while bypassing | IF-S1, IF-M3; FR12, NFR6 | manual (robot + dashboard) | Flash the normal UF2; let the car detect a box; press **■ Stop** mid-sweep. Resume, repeat during the side pass. Save the serial log | Motors stop at once; `servo: scan N stopped by operator ... centring` and `avoid: STOP from operator - obstacle handling abandoned, waiting for RESUME`; no `avoid: FAILED`; faults show REMOTE_STOP only; one **Resume** returns to INIT/NORMAL |
| INT-24 | RESUME clears the hump record (Buddies 2, 4, 1) | IF-S2; FR6 | manual (robot + dashboard) | Drive over a hump so "Last hump height" and "Highest this run" show values; press **■ Stop**, then **Resume** (no reboot) | Serial `hump: RESUME - last/highest height cleared (count N kept)`; "Last hump height" and "Highest this run" return to **—**; the count keeps running within the boot; the next hump is reported and stored as a new row |
| INT-25 | Barcode rules applied while driving (dashboard → Buddy 1 → Buddy 3) | FR13, FR14 | manual (robot + dashboard) | With the car following the line (NORMAL), change one letter on the dashboard and press **Apply all barcode rules**; then scan that letter | All 26 letters confirmed (~8–10 s); a letter refused during a barcode is retried automatically; the scanned letter performs the new action; Stop stays available throughout |

> **ID clash resolved (2026-11-25).** INT-21 is the telemetry-pipeline test defined in `testing/subsystem1/TEST_LOG.md` (run 2026-10-30). The cross-junction barcode-turn checks added on 2026-11-11 under INT-21a–e are renumbered **INT-22a–e** in `app_program/subsystem2/s2_integration_tests.c`. In logs dated 2026-11-11..24, `[TEST] INT-21a`–`e` lines mean those checks (now INT-22a–e).

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-18 | INT-15a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-15b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-16a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-16b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-16c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-16d | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-16e | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-10a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-10b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-10c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-10d | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-11a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-11b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-11c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-12a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-12b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-13a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-13b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-13c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-13d | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-13e | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-14a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-14b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-14c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-14d | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-17a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-17b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-18a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-18b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-18c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-19a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-19b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-20a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-20b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-20c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-20d | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-25 | INT-10a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-10b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-10c | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-10d | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-11a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-11d | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-11b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-11c | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-22a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-22e | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-22b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-22d | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-22c | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-12a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-12b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-13a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-13b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-13c | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-13d | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-13e | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-14a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-14b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-14c | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-14d | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-17a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-17b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-18a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-18b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-18c | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-19a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-19b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-20a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-20b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-20c | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-20d | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-15a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-15b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-16a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-16b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-16c | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-16d | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-16e | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_host_all_buddies.txt` | Host all-buddies run. |
| 2026-11-25 | INT-10a..INT-22e on the Pico (41 PASS lines) | 9d0e75f + 25 Nov cleanup | TeyHaiSeang | PASS | `../logs/2026-11-25_pico_selftest.txt` | Pico target run (TEST=1 image), integration checks inside the subsystem1/2 sections; INT-16e result line overwritten by concurrent console output, covered by "subsystem1 TOTAL 0 failed check(s)". |
