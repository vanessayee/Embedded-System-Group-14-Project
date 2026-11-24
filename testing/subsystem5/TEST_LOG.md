# Subsystem 5 — test log

**Subsystem:** Obstacle scanning, avoidance and line recovery  ·
**Student ID:** 2502075  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Test definitions below mirror the current automated checks. Pico-only tests must be run on target.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | 10-bit distance code: 5 mm steps, 0..4 m, 1022/1023 reserved |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | scan sample message round trip at limits |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U02 PASS` |
| S5-U02b | scan sample positive limits |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U02b PASS` |
| S5-U03 | scan request message round trip |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U03 PASS` |
| S5-U04 | profile message: limits kept, 'unknown' clearance stays unknown |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U04 PASS` |
| S5-U05 | scan and avoidance status words round trip |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U05 PASS` |
| S5-U06 | width within 10 % and distance within 1 cm (4 boxes, one full fine scan) |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U06 PASS` |
| S5-U07 | obstacle left of the path: negative bearing, more room on the right |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U07 PASS` |
| S5-U08 | no echo anywhere: path clear (NO_OBSTACLE) |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U08 PASS` |
| S5-U09 | object beyond 700 mm is not an obstacle to avoid |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U09 PASS` |
| S5-U10 | wall across the whole scan: edges unknown (INSUFFICIENT) |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U10 PASS` |
| S5-U11 | identical range at every angle: servo suspect (INVALID) |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U11 PASS` |
| S5-U12 | too few valid samples: INVALID |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U12 PASS` |
| S5-U13 | wall beside the obstacle limits that side's clearance |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13 PASS` |
| S5-U13b | isolated 3-degree dropout remains one obstacle; open sides ignore deeper background reflections |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13b PASS` |
| S5-U13c | extreme-angle near echoes do not extend the obstacle beyond the symmetric front-plane band |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13c PASS` |
| S5-U13d | captured hardware scan produces a valid profile with two known edges for open-board planning |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13d PASS` |
| S5-U13e | hardware-calibrated beam correction removes false obstacle width without losing clearance |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13e PASS` |
| S5-U13f | narrow target width comes from one measured angular scan cell, not a fixed millimetre minimum |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13f PASS` |
| S5-U13g | central median rejects one short reflection in the front-gap estimate |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13g PASS` |
| S5-U13h | central three-ray median retains a narrow obstacle despite valid background echoes |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13h PASS` |
| S5-U13n | corner echoes at the edges do not widen a recorded 130 mm box (24 Nov scan, was 174 mm) |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13n PASS` |
| S5-U13o | corner trimming removes at most one scan cell per side |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13o PASS` |
| S5-U13p | both recorded 130 mm scans measure 115..160 mm through the sensor path with the 45 mm servo radius (was 125..140 before the radius, 25 Nov) |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13p PASS` |
| S5-U13q | echo envelope moves an edge inward by at most the 15 mm edge reserve |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13q PASS` |
| S5-U13r | off-centre 15 cm box 205 mm from the servo shaft (beam model refitted from the shaft, 25 Nov): no edge more than the 15 mm reserve inside the box |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13r PASS` |
| S5-U13s | recorded 15 cm box at 160 mm (25 Nov): the box's own corner echoes are not side obstacles, both sides clear the 190 mm corridor and the plan succeeds (was clear 3/15 mm, Safe Stop) |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13s PASS` |
| S5-U13t | second recorded 15 cm scan (25 Nov left bypass) cut to the +-69 deg sweep: 47 samples, width 135..180 mm, both sides clear, plan succeeds |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13t PASS` |
| S5-U13u | 45 mm servo-to-transducer radius: lateral geometry from the shaft, front gap still from the bumper |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13u PASS` || S5-U14 | both sides free: use spare forward space, calculated departure, pass, shallow return, search, align |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U14 PASS` |
| S5-U15 | calculated 30–75° path gives the configured 100 mm robot-to-obstacle side gap and clears the front face |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U15 PASS` |
| S5-U15b | more front clearance produces a longer forward approach before departure |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U15b PASS` |
| S5-U16 | right-shifted obstacle chooses the shorter calculated route on the left |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U16 PASS` |
| S5-U16b | open-road planning has no hardcoded 320 mm board-width limit |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U16b PASS` |
| S5-U16c | route beyond the motion-command capacity is rejected safely |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U16c PASS` |
| S5-U17 | front gap below the calculated path requirement: no unsafe plan |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U17 PASS` |
| S5-U17b | 120 mm close stop reverses only to the calculated 150 mm path gap |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U17b PASS` |
| S5-U17c | required reverse beyond 150 mm is refused |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U17c PASS` |
| S5-U17d | 150 mm bumper gap plans directly without unnecessary reverse |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U17d PASS` |
| S5-U18a | open-road mode ignores captured 240/130 mm echo reach; equal route geometry prefers right |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U18a PASS` |
| S5-U18 | no known safe side: no plan (Safe Stop), never an invented bypass |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U18 PASS` |
| S5-U19 | no valid profile: no plan |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U19 PASS` |
| S5-U20 | every planned step is a valid AVOID motion command |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U20 PASS` |
| S5-U20d | side ranges give the pass convergence angle (24 Nov right bypass: ~13 deg); one 5 mm range step is not a correction |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U20d PASS` |
| S5-U20e | least-squares side trend detects ~16 deg closing within 40 mm (25 Nov left bypass); 5 mm flicker and too-short spans do not trigger |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U20e PASS` |
| S5-U21 | Obstacle Scan ranges every 60 ms (+ one 10 ms tick) while monitoring |  | auto (target only) | `subsystem5/subsystem5_test.c` | `[TEST] S5-U21 PASS` |
| S5-U22 | ranging result and distance code agree; a silent sensor raises US_NO_RESPONSE |  | auto (target only) | `subsystem5/subsystem5_test.c` | `[TEST] S5-U22 PASS` |
| S5-U23 | outside navigation: avoidance idle, servo centred, forward monitoring |  | auto (target only) | `subsystem5/subsystem5_test.c` | `[TEST] S5-U23 PASS` |
| S5-M01 | request, sample and status locals fit one aligned word |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-M01 PASS` |
| S5-M02 | bounded task and plan storage uses aligned words |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-M02 PASS` |
| S5-M03 | fault-run saturation and original byte detection wrap are retained |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-M03 PASS` |
| S5-M04 | unset profile identity cannot alias the first real sample |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-M04 PASS` |
| S5-M05 | full profile capacity and abs(-128) limit fit without neighbouring corruption |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-M05 PASS` |
| S5-M06 | pass confirmation, faults and signed side-watch angle coexist at limits |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-M06 PASS` |
| S5-M07 | full sensor range and actuator argument fit compact local storage |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-M07 PASS` |
| S5-M08 | unknown echo and negative turn retained; public line-lock counter not truncated |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-M08 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-18 | S5-U01 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U02 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U02b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U03 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U04 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U05 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U06 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U07 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U08 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U09 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U10 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U11 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U12 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U13 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U13b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U13c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U13d | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U13e | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U13f | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U13g | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U13h | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U14 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U15 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U15b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U15c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U16 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U16b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U16c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U17 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U17b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U17d | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U17c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U18a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U18 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U19 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U20 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U21 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U22 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U23 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem5_host.txt` |  |
| 2026-11-18 | S5-U21 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem5_target.txt` | Pico target run. |
| 2026-11-18 | S5-U22 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem5_target.txt` | Pico target run. |
| 2026-11-18 | S5-U23 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem5_target.txt` | Pico target run. |
