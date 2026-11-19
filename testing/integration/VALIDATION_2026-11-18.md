# Robot validation — 18 November 2026

## Verified on the laptop

- All 258 firmware host checks passed across Buddies 1–5 and their integrations.
- The left-turn advance now uses the measured geometry formula. The extra 50 mm calibration offset was reset to 0 because it caused the pivot to miss the branch in the cross-junction model.
- The left turn settled at approximately 88 degrees in simulation with the outgoing tape under the centre sensor.
- INT-21d exercises the real line-following module after the left turn. It verifies continued forward travel in NORMAL along the outgoing branch.
- All 34 telemetry backend unit checks passed.
- A simulated barcode stream passed through the running Mosquitto, Node-RED, PostgreSQL and Grafana stack using this repository's backend.
- All 35 dashboard panel queries completed without errors.
- Host output is saved in logs/2026-11-18_host_all_buddies.txt.

## Firmware images

- build_make/group14_selftest.uf2 runs the target checks with motor arming disabled.
- build_make/group14_robot.uf2 runs the normal robot, including the repaired left turn.
- Barcode assignments are A right, F and U left, X straight and Y U-turn. Z is unmapped.
- Wi-Fi SSID is Xiang_Zenbook and the broker is 192.168.137.1 port 1883. These must match the laptop hotspot.

## Pico bench results

The supplied COM4 log is saved in logs/2026-11-18_pico_selftest.txt. It contains 281 passing checks and one failed assertion, S2-U47.

- S2-U47 incorrectly required both SAFE_STOP and INIT after an accepted RESUME. The actual log shows the expected SAFE_STOP to INIT transition. The assertion is corrected in subsystem2_test.c; the corrected target image still needs a rerun.
- Physical motion and line tasks report 50 runs/s. IR calibration completes.
- The LSM303DLHC is detected and calibrated, its data is valid, and the IMU task reports 101 runs/s.
- Ultrasonic ranging reports 17 samples in 1.2 s and approximately 350 mm, with a centred servo.
- The Pico acquired 192.168.137.42. The broker previously listened only on loopback. After adding the hotspot listener at 192.168.137.1:1883, an actual Pico TCP session was observed.
- The running backend reports source live ONLINE. Grafana's actual queries return ONLINE, INIT and backend OK without query errors.
- No valid barcode has been received in this live session yet.
- Startup maximum execution times include 90.569 ms for IMU and 167.226 ms for scanning. The measured rates pass, but these maximums do not prove steady-state timing budgets.

## Course verification still required

The bench results above verify actual Pico task/sensor operation and live telemetry. The disarmed image does not perform physical motor turns, barcode traversal, hump measurements or obstacle bypasses. Those course checks remain pending.

1. Hold BOOTSEL while connecting USB, then copy group14_selftest.uf2 to RPI-RP2.
2. Keep the robot stationary and level. Open its USB serial port at 115200 and reset it to capture the complete output. Check each subsystem's target totals, sensor identities, freshness and faults.
3. Keep the laptop hotspot and its MQTT listener enabled and verify live telemetry continues. The current broker has listeners on loopback and the hotspot interface.
4. Upload group14_robot.uf2 for supervised course testing.
5. Buddy 1 — confirm live telemetry, decoded letter/INVALID, heartbeat, event delivery, command acknowledgement and reconnection after an outage.
6. Buddy 2 — confirm both encoders respond, wheel speeds track requests and STOP holds both motors off.
7. Buddy 3 — read a known barcode in both directions, verify the saved command waits for the plus junction, then verify left/right/U-turn and continued tracking on the outgoing branch.
8. Buddy 4 — traverse a measured hump. Compare reported height, confirm the highest value remains after a lower hump, and verify it arrives on the dashboard.
9. Buddy 5 — measure a known obstacle, verify the scan, collision clearance, bypass and return to line on the course. A no-path case should stop.

Passing simulated tests establishes the exercised software behaviour. Sensor wiring, physical geometry, wheel slip, ultrasonic reflections and real task scheduling need these hardware checks.
 
## Corrected target rerun

The second COM4 log is saved in logs/2026-11-18_pico_selftest_rerun.txt.

- All 282 checks passed; every subsystem reports TOTAL 0 failed.
- S2-U47 now passes with the correct RESUME expectation.
- The Pico acquired 192.168.137.42, connected to MQTT at 192.168.137.1, subscribed to csc14/cmd, and reached ONLINE during the run.
- Motion and line tasks each report 50 runs/s. The IMU is valid and reports 101 runs/s. Ultrasonic code 74 represents approximately 370 mm.
- The corrected self-test remains disarmed in INIT. Driving, physical junction turns, barcode traversal, measured humps and obstacle bypass still require the normal firmware course test.
- IMU and scan maximum run times remain high in the lifetime counters. This run does not establish steady-state timing budgets.
