# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

Defined on 25 November 2026 from the ten demonstration capabilities in
`telemetry/docs/DEMONSTRATION.md`, plus startup readiness and operator
stop/reset. All are run on the robot with the normal UF2
(`build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`), the dashboard open,
and the serial console logged to `testing/system/logs/YYYY-MM-DD_<test>.txt`.
Earlier whole-robot evidence (before these IDs existed) is in
`testing/integration/VALIDATION_2026-11-18.md`,
`testing/integration/DEMONSTRATION_VALIDATION_2026-11-18.md` and
`testing/logs/2026-11-21_full_robot_reverification.txt`.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | Startup readiness | all buddies | manual | Power on with both wheels on level ground, IR on the line, hotspot on | Motors stay off in INIT until every buddy reports ready and MQTT is ONLINE; then the car starts driving by itself |
| SYS-02 | Follow line | Buddy 3 | manual | Let the car track the course (black tape between GP6 and GP2) incl. curves | Stays on the line; dashboard shows centre/right states and steering |
| SYS-03 | Detect and decode barcode | Buddy 3 | manual | Drive over each assigned letter, in both directions | Correct letter (or INVALID) on dashboard and console |
| SYS-04 | Follow barcode instruction | Buddies 3, 2 | manual | Assign LEFT/RIGHT/STRAIGHT/U-TURN letters on the dashboard (can be applied while driving, INT-25); scan each, reach the junction | Each action is carried out at the junction, then NAV_ACTION → NORMAL and line following continues |
| SYS-05 | Hump height | Buddy 4; FR5, FR6 | manual | One wheel over a measured hump, other wheel on the flat | Last height within the documented tolerance of the measured hump; highest and count update; RESUME clears them (INT-24) |
| SYS-06 | Detect obstacle | Buddy 5; FR8–FR12 | manual | Place a 10, 13 or 15 cm box on the line ahead | Car stops ≈ 15 cm from the box (trigger 155 mm) and runs one −69..+69° sweep |
| SYS-07 | Measure width | Buddy 5; FR8–FR12 | manual | Record box width shown on the dashboard / `profile:` console line for each box size | Width never below the true width; record the error per box (recorded scans: +2..+45 mm) |
| SYS-08 | Avoid and rejoin | Buddies 5, 2, 3; FR8–FR12, NFR5 | manual | Let the car bypass the box; repeat with the box offset so it goes left and right | No contact; ≈ 5 cm side gap; line reacquired and NORMAL resumed (avoid SUCCESS) |
| SYS-09 | MQTT telemetry | Buddy 1; FR13, FR14 | manual | Watch the dashboard while driving; switch the hotspot off for ~10 s and back on | ~2 frames/s, age ≤ 5 s; outage shown stale/offline; reconnects automatically, car keeps driving |
| SYS-10 | Handling / collision stop | Buddy 4, 2; NFR6 | manual | Carefully lift/tilt/knock the car | Latched stop, SAFE_STOP, zero wheel speeds; Clear + Resume needed |
| SYS-11 | Operator Stop / Resume / Reset | Buddies 1, 2, 5; NFR6 | manual | Press **■ Stop** while driving, while scanning an obstacle and while bypassing (INT-23); press **Resume**; press **↻ Reset Pico** | Stop halts everything at once (motors, sweep, avoidance) with no extra fault; Resume restarts; Reset reboots (new boot, dashboard session resets) |
| SYS-12 | Full course run | all | manual | Complete demonstration course: line, barcodes, hump, obstacle, telemetry | All of SYS-02..SYS-09 observed in one run without intervention |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
