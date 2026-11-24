# Buddy 5: scanning, profiling, avoidance and recovery notes

Code: `drv_ultrasonic.c`, `drv_servo.c` (hardware); `s5_msg.c`,
`obstacle_profile.c`, `avoid_plan.c` (pure logic); `task_scan.c`,
`task_profile.c`, `task_avoid.c` (tasks). Tests: `s5_unit_tests.c`
S5-U01..U20, minimum-bit checks S5-M01..M08, on-target S5-U21..U23,
integration INT-10, INT-14, INT-18.

## 1. Three tasks, kept separate (Tribunal 1 Version B: "No change")

| Task | Prio | Owns | Does |
|---|---|---|---|
| Obstacle Scan | 15, core 1 | HC-SR04 (TRIG GP4, ECHO GP5 via the shared GPIO interrupt), SG90 servo (GP15, PWM slice 7B) | forward ranging every 60 ms; detection begins at 210 mm so braking settles near the requested 150 mm bumper gap; speed cap 100 mm/s below 360 mm; emergency stop at 100 mm bumper gap; one full-resolution sweep on request |
| Obstacle Profile | 19 | nothing | collects one packed scan phase, builds the profile, sends it on |
| Avoidance & Recovery | 16 | nothing | stop → full fine scan → profile → plan → execute through Buddy 2 → search line → align → resume, all bounded |

Messages between them are packed words over bounded message buffers
(`s5_msg.h`):

* scan request, 25 bits;
* sample, 25 bits, carrying a 3-bit scan id, a phase bit, a last-sample
  bit and a 2-bit status;
* profile, 2 words.

Each round-trips exactly (S5-U01..U05). Distances use 10 bits at **5 mm**:
the HC-SR04 resolves about 3 mm and is accurate to about 1 cm, so 5 mm
keeps the measurement (Tribunal 1 Buddy 5: "verify unpacked == original";
risk R4).

## 2. Ranging results are never faked

| Result | Meaning | Code / fault |
|---|---|---|
| `US_OK` | 20..4000 mm | distance code 0..800 |
| `US_NO_ECHO` | echo pulse longer than 4 m range: **nothing there** (a valid "clear" reading) | code 1022, no fault |
| `US_NO_RESPONSE` | ECHO never rose (sensor missing / dead) | code 1023; after 5 in a row `SYS_FLT_US_NO_RESPONSE` (critical condition, `SYS_US_REQUIRED 1`); cleared after 3 good pings |
| `US_INVALID` | impossible pulse (< 2 cm, ECHO stuck high) | code 1023; `SYS_FLT_US_INVALID` (degraded); one retry during sweeps |

An SG90 gives no position feedback. A frozen servo is detected indirectly:
identical ranges (± 10 mm) over a ≥ 60° sweep give an INVALID profile
(S5-U11).

Scan motion is slew-limited to 3° every 20 ms (about 150°/s) and allowed
50 ms to settle before each range measurement.  These values are hardware
calibration points in `drv_servo.h`; PWM remains at the servo-required 50 Hz.
The per-scan timeout is 22 s. The 51-point −75°..+75° fine scan uses three
pings per angle. Normal echoes finish in about 10 s, but many NO_ECHO angles
can approach 18 s because each waits for the ultrasonic timeout.

With `SERVO_DEBUG=1`, the console reports servo initialisation, each target
angle and PWM pulse, the slew step count, the detection distance, and every
scan sample.  Sample result codes are `0=OK`, `1=NO_ECHO`,
`2=NO_RESPONSE`, and `3=INVALID`.  Reported angles are commanded positions;
the SG90 has no physical position feedback.

With `US_DEBUG=1`, ultrasonic output is event-based rather than printed for
every 60 ms ping. It reports when a target enters `S5_DETECT_MM`, while a
nearby distance changes by at least 25 mm, once when it clears the threshold
(with 50 mm hysteresis), and on sensor fault/recovery transitions. Each event
still includes its sequence number, result code, final ECHO pin level,
captured edge mask (`1=rise`, `2=fall`, `3=both`), trigger-to-rise time, echo
pulse width, and converted distance. This keeps the serial monitor readable
without losing the timing needed to diagnose the sensor.

During the full fine sweep the scanner takes three pings at every settled
servo angle. Two valid returns must agree within 80 mm; three valid returns
use their median. Two NO_ECHO results establish open space. This prevents one
off-axis/background echo from becoming an obstacle edge. The normal forward
monitor remains one ping per 60 ms cycle so it can stop promptly.

## 3. Obstacle profile (`obstacle_profile.c`; Briefing p.8: W ≈ f(dL, dC, dR, θ))

Each sample becomes a point: lateral x = d·sin θ, forward y = d·cos θ.

1. **Validity:** at least 5 readings (OK or NO_ECHO), and not a frozen
   sweep. Otherwise INVALID.
2. **Nearest face:** the median forward distance at central −3°, 0° and +3° is
   used when available, with the smallest return within ±30° only as a
   fallback. This prevents one short/specular sample from making a rescanned
   obstacle appear tens of millimetres closer. If it is farther than 700 mm
   the path is clear (NO_OBSTACLE).
3. **Extent:** neighbouring samples belong to the obstacle while their
   **forward** distance is within a measured symmetric ±50 mm band around
   that face.
   The lower bound is important: a high-angle reflection can project much
   closer than the actual front face and must not extend the obstacle to the
   scan limit. Forward distance, not range, is used, so a flat face or wall
   is recognised as one surface (S5-U10, S5-U13c).
4. **Edges:** midway between the last hit and the first miss. The fine
   scan (3° steps) narrows that gap. Its fixed ±75° bounds measure the widest
   corridor visible to the sensor without needing a preliminary coarse pass.
   Edge positions are taken on the
   face plane: x = y_near · tan θ.
5. **Clearance per side:** from the edge to the nearest other object
   within the ±50 mm front-plane band, or to the lateral reach of the scan
   at that depth. Nearer specular returns and deeper background reflections
   are not treated as an immediate side wall. One isolated fine-scan miss is
   bridged when a front-face hit exists immediately beyond it; two consecutive
   misses still mark an edge. Clearance is capped at 2.54 m.
6. **Unknown is never free:** if the obstacle reaches the scan limit on a
   side, that edge and its clearance are UNKNOWN (code 255). If both are
   unknown the profile is INSUFFICIENT, and no bypass is ever planned
   through unknown space (brief: "do not invent a successful bypass").

Accuracy against ideal ray-cast boxes is checked in simulation; physical
ultrasonic width remains an estimate because of beam spread and reflections.
Production `S5_BEAM_COMP_DEG` is now zero: preserve the observed scan edges.
The former 24-degree-per-edge correction came from one approximately 200 mm
object and could substantially underestimate other objects, including a
130 mm target. It also incorrectly increased reported free side clearance.

There is now only one optional correction stage. Any future calibrated
allowance is limited to one 3-degree scan cell per edge, retains at least
one cell of obstacle span, and cannot increase measured side clearance.
Unknown edges are never corrected. The console retains both final and raw
widths for calibration. Tests cover 50/130/200/300 mm ideal targets at three
ranges, excessive calibration, and unknown clearance. No physical obstacle
width is hardcoded into the estimator. An uncorrected acoustic envelope can
overestimate the physical width; validate against several measured objects
before applying any nonzero correction.

## 4. Avoidance plan (`avoid_plan.c`; Briefing p.8 workflow)

* **Side choice:** with `AV_ASSUME_OPEN_BOARD=1`, the board is treated like an
  open road. The lateral reach of the scan is not interpreted as a board edge.
  Each known obstacle edge defines a candidate route and the planner chooses
  the one requiring less lateral deviation; equal routes prefer right
  (`AV_PREFER_RIGHT`). The path uses the 125 mm robot width
  (`AV_ROBOT_WIDTH_MM`), the requested 50 mm side gap (`AV_SIDE_GAP_MM`, was 80 mm) and a
  15 mm edge-uncertainty allowance (`AV_EDGE_UNCERTAINTY_MM`). If both
  obstacle edges are unknown →
  `OBSTACLE_UNAVOIDABLE` → Safe Stop (FR12).
* **Manoeuvre** (side s = ±1):
  `[MOVE approach], TURN s·T, MOVE d1, TURN −s·T, PASS, TURN −s·30°, SEARCH ≤ dS, TURN s·30°`.
  The planner uses all measured spare forward space, then calculates
  the smallest safe integer departure angle T between 30° and 75° from the
  measured lateral offset and remaining front clearance. Here
  d1 = lateral offset / sin T. During PASS the servo looks 80° toward
  the obstacle. Two side hits establish it; three consecutive readings beyond
  a plan-derived side-range limit (or NO_ECHO) establish the far edge. The
  limit is the planned sensor-to-edge distance at 80° plus a 40 mm range/pose
  tolerance, rather than a fixed board-clearance number. Buddy 5 records the
  closest valid side range, subtracts 10 mm measurement uncertainty, and uses
  the resulting live body-to-obstacle gap to find the earliest safe 30° return.
  The corner calculation checks the complete in-place pivot and subsequent
  diagonal move while retaining the requested 50 mm gap. A conservative full
  body-length fallback remains when the side measurement proves no spare gap.
  The far-edge search itself is bounded only by Buddy 2's 2 m single-motion
  safety limit.
  The configured geometry is 125 mm width, 175 mm length
  (`AV_ROBOT_WIDTH_MM`, `AV_ROBOT_LENGTH_MM`), with the sensor
  directly above the front bumper. The forward monitor triggers at 210 mm,
  including 60 mm of stopping allowance, so the requested stopped gap is
  approximately 150 mm. The planner may then use verified spare space and move
  to a 120 mm minimum calculated turn-start gap. A plan is accepted only when its diagonal
  can establish the side gap before the front face; otherwise bounded
  reverse/rescan is attempted.
* **Bounded drift (NFR5):** the lateral offset is measured obstacle edge +
  62 mm half-width + requested 50 mm side gap + 15 mm edge-uncertainty
  allowance (`avoid_plan.c`). Obstacle width and route offset are not fixed. There is no board
  width or board length assumption; only the motion command's representable
  distance remains a hard safety limit.
* **Too-close recovery:** when a bypass side is already known safe but the
  calculated 75° departure cannot establish the required lateral offset
  before the obstacle, Buddy 5 reverses only the missing distance along the
  path just travelled, plus a 20 mm rescan tolerance so range noise cannot
  leave the second profile just short of the same limit. The reverse is
  capped at 150 mm, runs at 25 mm/s, stops, and repeats the complete
  full-resolution scan. At the 120 mm calculated turn-start gap no reverse is needed;
  the 100 mm regression case reverses only to the calculated safe gap,
  rather than backing away toward a roughly 300 mm front gap. Reverse remains disabled
  for every other source and mode; failure or a second unsafe scan → Safe Stop.
* **Commands:** every step is an `MSRC_AVOID` command to Buddy 2 at speed
  level 2 (50 mm/s; search at level 1). Each waits for its done/aborted/fault
  result with a timeout (Tribunal 1: "Buddy 5 waits on the completion
  status"). Buddy 5 never touches the motors.

## 5. Recovery and line reacquisition (FR11, FR12)

* **SEARCH:** move towards the line. A side-sensor `LEV_FOUND` event only
  wakes the task; it continues until the centre IR sensor is black. The
  centre sensor is approximately 80 mm ahead of the wheel axle, so the robot
  then advances that calibrated distance before its final alignment turn.
  This puts the body on the line instead of turning as soon as the front
  sensor touches it. If the search command finishes just before Buddy 3's next
  20 ms sample, a 300 ms stationary grace period accepts that final centre-IR
  reading. One retry uses a 20° steeper approach.
* **Bounds:** the whole avoidance is limited to 90 s
  (`RECOVERY_TIMEOUT`), including one bounded reverse and rescan. An emergency
  stop (something new within 100 mm)
  allows one re-plan from where the car stands.
* **Failures → Safe Stop** with a reason (telemetry `avoid_state`
  FAILED, fail code): profile invalid, no clearance, manoeuvre failed, line
  not found, timeout. Leaving Safe Stop takes a supervisory RESUME, and only
  once no critical condition remains.
* **Mode hand-over:** Buddy 5 posts `SYS_EV_OBSTACLE`; Buddy 2 switches to
  OBSTACLE (new epoch, current command aborted, motors stopped). Line
  steering is ignored in that mode. On success Buddy 5 posts
  `SYS_EV_AVOID_DONE` and Buddy 2 returns to NORMAL (INT-10, INT-14).

Simulated bypass of a 150 mm box at 300 mm with the plant model (INT-10,
host run on the uncommitted working tree, 24 November 2026, after the side
gap was reduced from 80 mm to 50 mm):

* the car's body stays at least 52 mm from the box while beside it
  (configured side gap 50 mm; it was 84 mm with the former 80 mm setting);
* back on the line (offset 0 mm) after 740 mm of travel;
* end heading 0.036° (within ±10°).

The straight move after the far edge is now the body-clearance distance
plus the distance the edge may still be ahead of the sensor
(`av_post_edge_advance_mm`, S5-U14f). The former max(body, average) dropped
that edge allowance; on the 24 November robot run the return pivot started
about 39 mm earlier than the clearance check assumed.

This is a simulation with the configured robot geometry. It does not replace
the track test in section 6.

## 6. Remaining hardware validation (assumption A6 stays open)

1. HC-SR04 at 3.3 V: compare with known distances (100 mm–1.5 m) and
   angles; record repeatability (Tribunal 1).
2. Servo: calibrate `SERVO_CENTER_US`, `SERVO_US_PER_DEG_X10` and
   `SERVO_DIR`; measure the settle time per degree.
3. Scan known boxes at several distances: profile width and edges against
   the tape measure. Leave `S5_BEAM_COMP_DEG` at zero until repeated
   measurements justify a small correction across different widths and ranges.
4. Measure the car's width, length, turning behaviour and the margins in
   `avoid_plan.h`.
5. Full avoidance on the track (FR8–FR12), including the recovery timeout
   and an unavoidable obstacle → Safe Stop.


## Near-range profile and pass regression (24 November 2026)

The 130 mm physical target was reported as 341 mm with compensation disabled.
Its plan used a 277 mm centre offset. That run failed during the first -69-degree
pivot with MOTOR_STALL/AVOID_FAILED, before PASS; it does not demonstrate a
far-edge failure. Motor/encoder operation on the actual surface still requires
checking. Stall and collision protection remain enabled.

Front-face membership now uses a range-scaled depth tolerance: 5% of measured
front distance, bounded to 10..50 mm, instead of always permitting 50 mm.
Side-neighbour classification uses the same plane band so rejected peripheral
returns do not become false adjacent walls. This is a calibrated noise model,
not a guarantee of physical width accuracy. No-echo rays prevent the identical-
range check from falsely marking a narrow, edged target as a frozen servo.
The completed-sweep angle/range/status dump supports validation on real targets.

PASS acquisition covers the original profile transformed to the bypass offset,
rather than just the requested gap. Once the side is confirmed, the threshold
can tighten to that measured surface plus the existing 40 mm allowance, so
farther background does not indefinitely hide its end. If the side is never
confirmed after the expected front overlap plus one robot length and allowance,
the robot stops instead of driving the entire actuator-distance limit. This is
not permission to return blindly. A compact side-watch message is limited to
once per second during PASS. Existing three-clear confirmation and body-corner
return-clearance calculations remain in place.

Buddy 5's command wait now covers the same encoder wheel arc at creep as Buddy 2,
plus 3000 ms slack; the former fixed 25 ms/degree wait could expire first.
Failure diagnostics distinguish a wait expiry from a returned motion result.
Regression tests cover peripheral beam spill, real front-plane dropout,
mirrored side acquisition bounds, and slow-turn wait symmetry. Physical testing
must confirm width, actual side gap, side acquisition, far-edge detection, and
line reacquisition before calling the maneuver validated.


## Corner echoes at the obstacle edges (24 November scan)

A second scan of the same ~130 mm box (24 November robot log) reported
174 mm. A smooth face echoes straight back only while its perpendicular
foot is inside the beam: rays -15..+18 deg all read 205 mm. Beyond that the
sensor hears the corner, at sqrt(205^2 + 65^2) = 215 mm, and the +-21 deg
rays read exactly 215. Projected forward (215 cos 21 = 200.7 mm) they fell
inside the +-10 mm face band and moved each edge out by one scan cell.

`op_build` now drops the outermost face ray on each side when it looks like
such a corner echo: outside the 15 deg direct beam, at least 8 mm farther
than the face (more than one 5 mm range step) and projecting at least 3 mm
nearer than the face plane. Only one ray per side is dropped, so a
genuinely wider face loses at most one scan cell, which the 15 mm edge
uncertainty reserve covers.

| Scan of the ~130 mm box | Before | After |
|---|---:|---:|
| 24 Nov robot run (S5-U13n) | 174 mm | 140 mm |
| Earlier capture (S5-U13m) | 125 mm | 125 mm |

The result is the same for direct-beam angles 9-18 deg and range margins
6-10 mm, so it is not tuned to a single value. Both scans are of one box at
one distance (205 mm); other widths and distances still need recorded scans
(the `profile: scan ... rays` dump) and a tape measurement.

Known limitation, unchanged by this work: in a rough-surface model a wide
face turned 5-10 deg toward the car is reported up to 103 mm too narrow
(300 mm box at 205 mm, 10 deg), because a sloping face leaves the +-10 mm
same-plane band. Real scans of an angled box are needed before changing it.

## Captured 130 mm target and final-alignment correction

Replaying the 51 angle/range/status samples supplied on 24 November produced
208 mm with the previous code. A valid short return outside the front-plane
band was bridged to a further group of hits, inflating the left edge. Such a
nearer mismatch is no longer treated as a dropout. Where another hit follows,
the first mismatched ray is retained as a bounded edge allowance instead of
joining the full group. Missing echoes and far-background dropouts retain their
previous bridging behavior. The exact replay now yields 125 mm (left -55,
right +70, front 205 mm), versus the user's approximately 130 mm measurement.
No 130 mm target size or fixed width subtraction is used by production code.
This result validates that captured scan; other objects and poses still require
physical checking. Existing side-gap, uncertainty and geometry values remain.

The same run completed the outward pivots, parallel pass, far-edge detection,
return and first line contact. It failed at the final alignment pivot while
requesting 25 mm/s and reporting MOTOR_STALL. Final alignment now uses the same
50 mm/s speed level as the earlier successful pivots. Fresh IR centre-only
confirmation still ends the pivot; its angle/time limits are safety bounds.
Motor stall, encoder and collision protections remain active. A motor/encoder
failure during alignment is reported as AVF_MOTION rather than being labelled
only as missing line recovery. Incomplete alignment prints elapsed time, mode,
result, sensor bits and faults. These changes need another supervised robot
run to validate motor torque and physical line alignment.
