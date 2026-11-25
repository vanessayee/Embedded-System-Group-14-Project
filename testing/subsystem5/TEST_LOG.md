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
| S5-U04b | profile packing rounds outward and never narrows the obstacle |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U04b PASS` |
| S5-U05 | scan and avoidance status words round trip |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U05 PASS` |
| S5-U06 | width within 10 % and distance within 1 cm (4 boxes, full fine scan) |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U06 PASS` |
| S5-U06a | range consensus preserves median of clustered pings |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U06a PASS` |
| S5-U06b | range consensus rejects isolated background reflection |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U06b PASS` |
| S5-U06c | range consensus rejects isolated short echo |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U06c PASS` |
| S5-U06d | three inconsistent pings cannot invent a distance |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U06d PASS` |
| S5-U06e | two pings require relative agreement at short range |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U06e PASS` |
| S5-U06f | invalid counts and ranges cannot produce a distance |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U06f PASS` |
| S5-U07 | obstacle left of the path: negative bearing, more room on the right |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U07 PASS` |
| S5-U08 | no echo anywhere: path clear (NO_OBSTACLE) |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U08 PASS` |
| S5-U09 | object beyond 700 mm is not an obstacle to avoid |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U09 PASS` |
| S5-U10 | wall across the whole scan: edges unknown (INSUFFICIENT) |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U10 PASS` |
| S5-U11 | identical range at every angle: servo suspect (INVALID) |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U11 PASS` |
| S5-U12 | too few valid samples: INVALID |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U12 PASS` |
| S5-U13 | wall beside the obstacle limits that side's clearance |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13 PASS` |
| S5-U13b | single 3-deg dropout stays in one obstacle; open sides ignore deep background |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13b PASS` |
| S5-U13c | extreme-angle near echoes stay outside the symmetric front-plane band |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13c PASS` |
| S5-U13d | captured production profile has two known edges for planning |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13d PASS` |
| S5-U13e | production retains observed width and clearance without uncalibrated shrinkage |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13e PASS` |
| S5-U13f | default production correction preserves a small measured obstacle |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13f PASS` |
| S5-U13g | central median rejects one short reflection in the front-gap estimate |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13g PASS` |
| S5-U13h | central three-ray median retains a narrow obstacle despite background echoes |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13h PASS` |
| S5-U13i | excessive calibration is bounded and cannot invent free side space |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13i PASS` |
| S5-U13j | unknown side remains unknown even with optional correction |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13j PASS` |
| S5-U13k | 50/130/200/300 mm objects at three ranges retain geometry within scan resolution |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13k PASS` |
| S5-U13l | near-range plane tolerance rejects peripheral same-range beam spill |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13l PASS` |
| S5-U13m | recorded target keeps central face without merging nearer echo groups |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13m PASS` |
| S5-U13n | corner echoes at the edges do not widen a recorded 130 mm box |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13n PASS` |
| S5-U13o | corner trimming removes at most one scan cell per side |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13o PASS` |
| S5-U13p | both recorded 130 mm scans measure 115..160 mm |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13p PASS` |
| S5-U13q | echo envelope moves an edge inward by at most the 15 mm reserve |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13q PASS` |
| S5-U13r | off-centre 15 cm box: no edge lies more than the 15 mm reserve inside the box |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13r PASS` |
| S5-U13s | recorded 15 cm box at 160 mm: own corner echoes are not side obstacles; plan succeeds |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13s PASS` |
| S5-U13t | second recorded 15 cm scan, +-69 deg sweep: width within reserve, plan succeeds |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13t PASS` |
| S5-U13u | servo-to-transducer radius: lateral geometry from the shaft, front gap from the bumper |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13u PASS` |
| S5-U14 | plan uses forward space, calculated departure, pass, shallow return and align |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U14 PASS` |
| S5-U14a | all-white surface cannot prove return-to-line alignment |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U14a PASS` |
| S5-U14b | white pair interrupts centre-edge confirmation |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U14b PASS` |
| S5-U14c | centre-only black requires three consecutive samples |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U14c PASS` |
| S5-U14d | junction black on both sensors is not the centre-only edge |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U14d PASS` |
| S5-U14e | confirmed centre-only black establishes the recovery edge |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U14e PASS` |
| S5-U14f | post-edge advance covers body clearance plus the edge still ahead |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U14f PASS` |
| S5-U15 | calculated path preserves the configured side gap and clears the front face |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U15 PASS` |
| S5-U15b | more measured front clearance produces a longer forward approach |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U15b PASS` |
| S5-U15c | live side gap shortens return while preserving the configured clearance |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U15c PASS` |
| S5-U15d | calculated return uses less forward travel than the fixed 30 degree return |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U15d PASS` |
| S5-U16 | right-shifted obstacle: shorter calculated route bypasses on the left |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U16 PASS` |
| S5-U16b | planning has no hardcoded 320 mm board-width limit |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U16b PASS` |
| S5-U16c | route beyond the motion-command capacity is rejected safely |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U16c PASS` |
| S5-U17 | bumper gap below minimum: no plan; Safe Stop |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U17 PASS` |
| S5-U17b | 100 mm close stop reverses only to the calculated safe path gap |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U17b PASS` |
| S5-U17c | reverse beyond the 150 mm bound is refused |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U17c PASS` |
| S5-U17d | minimum calculated bumper gap plans directly without reverse |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U17d PASS` |
| S5-U18 | no known safe side: no plan (Safe Stop), never an invented bypass |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U18 PASS` |
| S5-U18a | 240 mm left corridor qualifies while 130 mm right corridor is too narrow |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U18a PASS` |
| S5-U19 | no valid profile: no plan |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U19 PASS` |
| S5-U20 | every planned step is valid and every avoidance MOVE is forward |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U20 PASS` |
| S5-U20a | inflated scan profile cannot hide a real 300 mm side return behind a 200 mm gap gate |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U20a PASS` |
| S5-U20b | mirrored departure covers the same measured profile envelope |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U20b PASS` |
| S5-U20c | both turn directions allow the wheel arc at creep plus bounded slack |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U20c PASS` |
| S5-U20d | **Retired 2026-11-25** (with `av_side_convergence_deg()`, replaced by the `AV_TREND` fit, see S5-U20e). Was: side ranges give the convergence angle; one 5 mm range step is not a correction |  | retired | — | no longer run |
| S5-U20e | side trend detects 16 deg closing within 40 mm; 5 mm flicker and short spans do not trigger |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U20e PASS` |
| S5-U21 | Obstacle Scan ranges every 60 ms (+ one 10 ms tick) while monitoring |  | auto (target only) | `subsystem5/subsystem5_test.c` | `[TEST] S5-U21 PASS` |
| S5-U21a | blocked shorter left route cannot override the usable right route |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U21a PASS` |
| S5-U21b | mirrored blocked shorter right route chooses usable left |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U21b PASS` |
| S5-U21c | two narrow routes produce no move or unhelpful reverse |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U21c PASS` |
| S5-U21d | profile packing cannot round an insufficient corridor into acceptance |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U21d PASS` |
| S5-U21e | exact required corridor qualifies; unknown opposite side cannot win |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U21e PASS` |
| S5-U21f | equal qualifying routes retain right preference |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U21f PASS` |
| S5-U21g | outward edge quantization consumes clearance instead of inventing room |  | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U21g PASS` |
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
| 2026-11-25 | S5-U01 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U02 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U02b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U03 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U04b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U04 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U05 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U06a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U06b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U06c | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U06d | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U06e | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U06f | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U06 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U07 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U08 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U09 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U10 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U11 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U12 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13c | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13d | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13e | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13f | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13i | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13j | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13k | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13g | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13h | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13l | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13m | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13n | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13p | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13q | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13r | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13s | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13t | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13u | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U13o | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U14a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U14b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U14c | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U14d | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U14e | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U14f | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U14 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U15d | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U15 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U15b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U15c | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U16 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U16b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U16c | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U17 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U17b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U17d | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U17c | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U18a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U18 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U19 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U20 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U20a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U20b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U20c | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U20d | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U20e | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U21a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U21b | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U21c | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U21d | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U21e | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U21f | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-U21g | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-M01 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-M02 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-M03 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-M04 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-M05 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-M06 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-M07 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | S5-M08 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_host.txt` | Host all-buddies run. |
| 2026-11-25 | all S5-U/M host checks (82 PASS) | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `../integration/logs/2026-11-25_host_after_cleanup.txt` | Re-run after removing unused code (av_side_convergence_deg; S5-U20d retired). |
| 2026-11-25 | S5-U21 | 9d0e75f + 25 Nov cleanup | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_target.txt` | Pico target run (TEST=1 image). |
| 2026-11-25 | S5-U22 | 9d0e75f + 25 Nov cleanup | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_target.txt` | Pico target run (TEST=1 image). |
| 2026-11-25 | S5-U23 | 9d0e75f + 25 Nov cleanup | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_target.txt` | Pico target run (TEST=1 image). |
| 2026-11-25 | all other S5-U/M checks on the Pico (82 PASS) | 9d0e75f + 25 Nov cleanup | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem5_target.txt` | Same unit checks as the host run, executed on the RP2040 target; subsystem5 TOTAL 0 failed. |
