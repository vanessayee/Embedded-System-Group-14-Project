# Demonstration update validation — 18 November 2026

Implemented a local mission console at http://127.0.0.1:1880/csc14/dashboard.
The existing Grafana dashboard links to it and retains its historical charts.
The console exposes ten demonstration evidence indicators, decoded letter or
INVALID, mapped instruction, IR states, wheel speeds, last/highest hump,
ultrasonic distance/context, obstacle width, avoidance state/side/outcome,
handling/collision stop, fault list and event feed. Offline values are marked
stale; evidence means observed telemetry, not verified physical performance.

Runtime A-Z navigation rules use new binary MQTT commands SET_NAV_MAP (6)
and GET_NAV_MAP (7), with CRC and ACK correlation. Mapping entries are atomic
words shared across cores. Updates require INIT/SAFE_STOP and no queued or
executing barcode. All 26 entries fit, including unmapped. Dashboard changes
are paced below five commands/second and require robot OK acknowledgements.
STOP bypasses configuration queues. Other motion controls are blocked during
configuration. Cross-origin/header-less POST controls are rejected. Desired
rules persist on the laptop; the robot map is RAM-only and must be reapplied
after a power cycle. No automatic movement or mapping commands were sent to
the physical robot during verification.

Added accelerometer handling detection for sustained deviation from 1g,
free-fall and severe tilt; it feeds the existing latched collision stop and
MQTT status/event. A gentle level lift remains unobservable with only this
accelerometer. The console and guide explicitly disclose that limitation.

Validation:
- 293 firmware host checks passed, including new map protocol/table and
  handling thresholds; existing turn, avoidance, hump and safety checks pass.
- 40 backend tests passed, including matching/rejected/missing ACKs,
  all letter/action wire encodings and configuration/HTTP guards.
- All 35 SQL panel queries passed through the running Grafana datasource.
- New console and snapshot endpoints returned HTTP 200 against the local DB.
- Browser verified all ten evidence rows, 27 dropdown choices (blank + A-Z),
  duplicate-letter rejection, offline read feedback and no horizontal overflow
  at the available browser viewport. Screenshot saved to Documents/Codex/
  group14-dashboard-preview.jpg. No live motion buttons were pressed.
- Normal MQTT UF2 rebuilt: build_make/group14_robot.uf2.

The Pico was offline during final validation. Live command ACKs and physical
course behaviour require retesting after flashing the new UF2. Do not claim
all ten physical capabilities validated solely from these software results.
Startup and demonstration instructions are in telemetry/docs/DEMONSTRATION.md.

Low-speed hump start correction (18 November 2026)

Found the start gate required pitch >=3 degrees AND positive encoder displacement on two consecutive 10 ms IMU samples. Production displacement comes from whole-mm odometry published by the 20 ms motion task, producing intervening zero increments even with real forward motion. The earlier 30 mm/s simulation passed fractional 0.3 mm increments every 10 ms and did not reproduce this production failure.

The start gate now confirms two consecutive uphill pitch samples and requires accumulated forward travel over that uphill sequence. Zero displacement no longer clears valid pitch confirmation; actual travel is still mandatory, pitch dropping below threshold resets the candidate, and stationary tilt cannot start a hump. The 3 mm minimum peak, downhill <=-3 degrees, level <1.5 degrees for three samples, 1 m limit and invalid-data rejection remain unchanged.

Added a no-gyro/filter regression for a 40 mm hump at 30 mm/s using 20 ms whole-mm odometry read at 10 ms; measured within the existing +/-4 mm tolerance. Added stationary-tilt and subsequent real-forward-step checks. All 296 host checks passed. Serial logs now report climb start, descent/peak and discarded measurements. Final height/max/count still publish only after completing descent and level confirmation. Normal UF2 rebuilt at build_make/group14_robot.uf2; physical retest required.

One-wheel sideways hump measurement (18 November 2026)

User confirmed one wheel climbs while the opposite wheel stays on the flat road. Default HUMP_SIDEWAYS=1 now selects roll for the existing packed IMU-to-hump queue and calls hump_roll_step. The queue field is named tilt_ddeg rather than pitch_ddeg; packing widths/positions and MQTT fields are unchanged. Height estimates physical wheel-centre span (110 mm) times sin(abs(peak roll)). It does not integrate roll along forward travel, and deliberately does not use the effective 130 mm turn calibration. Motor-turn settings are untouched.

Both roll signs work. Start requires >=3 degrees for two consecutive attitude samples and actual accumulated forward travel. It tolerates intervening zero odometry increments. Peak height is tracked and a falling peak followed by <1.5 degrees for three samples completes a hump, without negative pitch or opposite-sign roll. Last/highest/count publish through the existing MQTT/dashboard path. Static tilt cannot start a terrain measurement. Invalid/drop/reverse, excessive traversal and severe tilt discard the candidate and block partial remeasurement until level. Handling/collision latches invalidate terrain samples. Legacy forward-pitch logic remains available under HUMP_SIDEWAYS=0.

Seven new checks cover both sides, sparse odometry, stationary tilt, finish timing, lost samples, severe tilt, long grades and the no-gyro filter plus packed queue. All 303 host checks passed; normal MQTT UF2 rebuilt as build_make/group14_robot.uf2. A 20-degree pure roll estimates 37.6 mm and is transmitted as 38 mm. This is a geometry estimate: validate against a known hump, with level startup and the opposite wheel on original flat road. No physical sideways-hump height accuracy test was performed.

### Startup readiness and physical RUN reset

Added all-five-buddy readiness gate before NORMAL. Local calibration/task/sensor
readiness retains the 8-second timeout; hotspot/MQTT waiting remains INIT with
motors off indefinitely. Runtime disconnection retains existing autonomy policy.
Added five pure FSM checks for individual buddy gating, long network wait,
automatic release, later network loss and local-sensor timeout. All host suites
passed (307 checks). Backend tests pass 42/42, including a quick physical reset
below the old uptime-detection tolerance, repeated boot marker, backend restart,
and selecting a fresh session before its first telemetry snapshot.

The new BOOT event (code 7) carries uptime_ds[15:0] and a ROSC-derived 16-bit
per-boot token[31:16], repeated every 5 seconds while online. Mission console
clears all current session values/evidence/logs/chart/map confirmations; desired
laptop rules and historical data remain. Grafana current stat cards and latest
decoded letter are scoped to latest boot. No on-robot reset/movement was issued;
physical startup and RUN reset still need supervised validation after flashing.

### Buddy 5 pause after profile generation
The supplied serial run reached NORMAL after all five readiness bits became 1,
then triggered avoidance and completed all 51 fine scan samples. It paused after
profile generation, before the plan summary. Startup was not the blocker in
that run. Optimized the nested return-clearance search from a linear millimetre
sweep to a binary search over the same monotone body-clearance predicate, and
reused each heading's sine/cosine across the four body corners. A standalone
comparison against the saved pre-change implementation checked 2,511 angle/gap
combinations and 45 complete plans with identical results. Aggregate pose-sweep
calls fell from 111,490 to 19,527; geometry and configured 80 mm gap unchanged.
All host suites passed. Added actual on-device planning-duration/result logging.
Physical completion of avoidance still needs a supervised run of the new UF2.


### Requested rollback to initial all-buddies-ready version
Reverted the subsequent continuous-readiness timeout change, planner binary-search optimization and planning-duration diagnostics. The initial all-five-buddy/MQTT startup gate, dashboard RUN-reset handling and configured geometry/clearances remain. The prior planner optimization entry above describes the reverted experiment. Servo driver was not changed during the interrupted investigation.
