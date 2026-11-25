# Subsystem 4 — test log


**Subsystem:** MU monitoring and hump detection  ·
**Student ID:** 2500320  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Test definitions below mirror the current automated checks. Pico-only tests must be run on target.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | integer square root exact (floor) incl. 32-bit maximum |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | sine within 0.002 at 0/30/90/-30/180/270 deg |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U02 PASS` |
| S4-U03 | atan2 within 0.3 deg (0, 45, 90, -135, 10 deg) |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U03 PASS` |
| S4-U04 | stationary calibration recovers gyro bias and level offsets |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U04 PASS` |
| S4-U05 | car moving during calibration: rejected (retry) |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U05 PASS` |
| S4-U06 | gravity not on the configured up axis: calibration failure |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U06 PASS` |
| S4-U07 | no samples: failure |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U07 PASS` |
| S4-U08 | static 10 deg pitch measured within 0.5 deg |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U08 PASS` |
| S4-U09 | short braking/bump spike: pitch error < 1 deg (filter) |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U09 PASS` |
| S4-U10 | tilt event only after 200 ms beyond 15 deg (no false tilt) |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U10 PASS` |
| S4-U11 | 2.5 g impact detected once (refractory), peak recorded |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U11 PASS` |
| S4-U12 | 0.9 g bump below the collision threshold |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U12 PASS` |
| S4-U13 | still for 300 ms: STATIONARY |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U13 PASS` |
| S4-U14 | +30 dps yaw: TURN_LEFT, turn rate reported |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U14 PASS` |
| S4-U15 | -30 dps yaw: TURN_RIGHT |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U15 PASS` |
| S4-U16 | vibration/acceleration: MOVING |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U16 PASS` |
| S4-U17 | frozen raw data -> invalid run -> IMU fault |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U17 PASS` |
| S4-U18 | saturated sample rejected, never used |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U18 PASS` |
| S4-U19 | 40 mm hump measured within +-4 mm (Tribunal 1 success criterion) |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U19 PASS` |
| S4-U20 | highest hump of the run tracked at full resolution |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U20 PASS` |
| S4-U21 | flat track with vibration: no hump |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U21 PASS` |
| S4-U22 | long ramp (> 1 m) is not reported as a hump |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U22 PASS` |
| S4-U23 | invalid sample mid-hump: measurement discarded |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U23 PASS` |
| S4-U24 | 6-bit hump code: 2 mm steps, saturation flag, 63 = none |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U24 PASS` |
| S4-U25 | quantisation error <= 1 mm over 0..100 mm |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U25 PASS` |
| S4-U26 | IMU status words round trip at limits and invalid codes |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U26 PASS` |
| S4-U27 | hump status word (count saturates at 15) |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U27 PASS` |
| S4-U28 | IMU->hump queue message: pitch clamped to 11-bit range |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U28 PASS` |
| S4-U29 | compaction rounds to nearest and saturates |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U29 PASS` |
| S4-U30 | IMU Monitoring runs every 10 ms (100 per s) |  | auto (target only) | `subsystem4/subsystem4_test.c` | `[TEST] S4-U30 PASS` |
| S4-U31 | IMU snapshot honest and fresh: valid data only from a calibrated chip, else invalid codes + IMU fault |  | auto (target only) | `subsystem4/subsystem4_test.c` | `[TEST] S4-U31 PASS` |
| S4-U32 | standing car: no hump reported; IMU -> hump queue never overflowed |  | auto (target only) | `subsystem4/subsystem4_test.c` | `[TEST] S4-U32 PASS` |
| S4-U33 | no gyro: static 10 deg pitch from the accelerometer within 0.5 deg |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U33 PASS` |
| S4-U34 | no gyro: single bump spike rejected, pitch error < 1 deg |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U34 PASS` |
| S4-U35 | no gyro: tilt only after the filter settles and 200 ms beyond 15 deg |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U35 PASS` |
| S4-U36 | no gyro: 2.5 g impact detected once, peak recorded |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U36 PASS` |
| S4-U37 | no gyro: left turn from the magnetometer, rate within 3 dps, heading within 2 deg |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U37 PASS` |
| S4-U38 | no gyro: right turn gives a negative yaw rate |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U38 PASS` |
| S4-U39 | no gyro: heading tilt-compensated, climbing 10 deg moves it < 2 deg, no false turn |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U39 PASS` |
| S4-U40 | no gyro: magnetometer overflow invalidates heading only, not the IMU |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U40 PASS` |
| S4-U41 | no gyro: still for 300 ms: STATIONARY |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U41 PASS` |
| S4-U42 | no gyro: 40 mm hump through the accelerometer filter within +-4 mm, no false turn |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U42 PASS` |
| S4-U43 | no gyro: 40 mm hump at 30 mm/s (0.3 mm per sample) within +-4 mm |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U43 PASS` |
| S4-U44 | shallow road grade is not accumulated into a later hump candidate |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U44 PASS` |
| S4-U45 | reverse motion rejects an active forward hump measurement |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U45 PASS` |
| S4-U50 | level 1g remains normal; gentle level lifting cannot be inferred |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U50 PASS` |
| S4-U51 | sustained vertical pickup jolt triggers handling stop |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U51 PASS` |
| S4-U52 | sustained low-g/free-fall triggers handling stop |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U52 PASS` |
| S4-U53 | robot held on its side triggers severe-tilt handling stop |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U53 PASS` |
| S4-U54 | ordinary 20-degree hump tilt does not trigger severe handling stop |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U54 PASS` |
| S4-U55 | 30 mm/s hump survives real 20 ms whole-mm odometry sampled at 10 ms |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U55 PASS` |
| S4-U56 | sustained uphill tilt without wheel travel cannot start a hump |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U56 PASS` |
| S4-U57 | confirmed uphill tilt starts with actual forward travel after zero increments |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U57 PASS` |
| S4-U58 | one-wheel 20-degree roll on either side measures about 37.6 mm with sparse odometry |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U58 PASS` |
| S4-U59 | stationary sideways tilt cannot start a terrain measurement |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U59 PASS` |
| S4-U60 | one-wheel height tracks peak but waits for level before publishing a completed hump |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U60 PASS` |
| S4-U61 | lost IMU sample discards the side hump and blocks partial remeasurement until level |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U61 PASS` |
| S4-U62 | overturn/severe tilt is not reported as a hump |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U62 PASS` |
| S4-U63 | long sideways slope is rejected without rearming before level |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U63 PASS` |
| S4-U64 | LSM303 roll filter and packed queue measure a one-wheel hump at slow speed |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U64 PASS` |
| S4-U65 | 5 mm sideways hump on either wheel reports exact 5 mm code |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U65 PASS` |
| S4-U66 | 50 ms moving roll transient rejected |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U66 PASS` |
| S4-U67 | tilt with only 1 mm encoder travel rejected |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U67 PASS` |
| S4-U68 | fine codes retain 6-bit format, invalid codes and exact 5 mm reporting |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U68 PASS` |
| S4-U69 | boot/reset clears hump history |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U69 PASS` |
| S4-U70 | 5 mm target survives accelerometer filter and packed queue |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U70 PASS` |
| S4-M01 | local flags compact without narrowing numeric sensor values |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-M01 PASS` |
| S4-M02 | snapshot storage bounded; explicit shared-word packers retained |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-M02 PASS` |
| S4-M03 | stuck and invalid counters retain 255 saturation without wrap |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-M03 PASS` |
| S4-M04 | full collision refractory delay retained |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-M04 PASS` |
| S4-M05 | magnetic history retains full window and circular index |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-M05 PASS` |
| S4-M06 | hump count saturates while highest peak keeps micrometre precision |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-M06 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-18 | S4-U01 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U02 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U03 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U04 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U05 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U06 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U07 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U08 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U09 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U10 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U11 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U12 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U13 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U14 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U15 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U16 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U17 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U18 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U33 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U34 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U35 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U36 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U37 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U38 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U39 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U40 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U41 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U42 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U43 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U19 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U20 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U21 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U22 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U23 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U24 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U25 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U44 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U45 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U26 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U27 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U28 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U29 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U30 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U31 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U32 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-18 | S4-U30 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem4_target.txt` | Pico target run. |
| 2026-11-18 | S4-U31 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem4_target.txt` | Pico target run. |
| 2026-11-18 | S4-U32 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem4_target.txt` | Pico target run. |

## 2026-11-24: 5 mm sideways target (host verification)
S4-U65..70 and all existing host suites passed. Added sustained-tilt/motion qualification and reserved fine height codes. Protocol/ingestion tests: 26 passed. Physical 5 mm measurement and flat-course false-positive rate still require validation. See app_program/subsystem4/docs/HUMP_5MM_TARGET.md. External logs: outputs/hump5mm/host-tests.log and backend-tests.log in the Codex workspace.
Firmware build NET=lwip MQTT=1 TEST=0 passed. Expanded protocol/ingestion/storage/dashboard regression: 42 passed, 0 failed. New firmware not flashed; no movement or safety-clear command sent.
| 2026-11-25 | S4-U01 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U02 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U03 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U04 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U05 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U06 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U07 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U08 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U09 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U10 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U11 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U12 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U13 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U14 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U15 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U16 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U17 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U18 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U33 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U34 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U35 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U36 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U37 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U38 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U39 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U40 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U41 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U42 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U43 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U50 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U51 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U52 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U53 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U54 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U55 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U64 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U70 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U19 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U20 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U21 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U22 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U23 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U24 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U25 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U44 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U45 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U56 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U57 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U58 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U59 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U60 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U61 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U62 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U63 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U65 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U66 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U67 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U68 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U69 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U26 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U27 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U28 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U29 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-M01 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-M02 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-M03 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-M04 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-M05 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-M06 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_host.txt` | Host all-buddies run. |
| 2026-11-25 | S4-U30 | 9d0e75f + 25 Nov cleanup | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_target.txt` | Pico target run (TEST=1 image). |
| 2026-11-25 | S4-U31 | 9d0e75f + 25 Nov cleanup | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_target.txt` | Pico target run (TEST=1 image). |
| 2026-11-25 | S4-U32 | 9d0e75f + 25 Nov cleanup | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_target.txt` | Pico target run (TEST=1 image). |
| 2026-11-25 | all other S4-U/M checks on the Pico (69 PASS) | 9d0e75f + 25 Nov cleanup | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem4_target.txt` | Same unit checks as the host run, executed on the RP2040 target; subsystem4 TOTAL 0 failed. |
