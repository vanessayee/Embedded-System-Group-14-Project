# Obstacle recovery and junction review

## Left versus right junction turns

The previous right turn used a fresh centre-sensor transition to end the pivot, inside the 70–105 degree encoder window. The previous left turn instead stopped its pivot about 3 degrees early, waited 120 ms, and searched forward up to 60 mm. It accepted either tracking sensor, including GP2 alone. After 20 mm its search curved right. Consequently the left turn could miss a centre contact during the pivot, finish from the wrong edge, or change heading again before handing back to tracking.

Both junction turns now use the working right-turn completion rule. The centre sensor must first clear the crossing, then reacquire black while GP2 is white, within the permitted angle window. The left turn keeps rotating until that evidence arrives. GP2 alone cannot complete it. Missing or stale line evidence causes a fault at the bounded encoder limit.

The approach distances remain different because GP6 and GP2 are mounted on one side of the desired tracking corridor. Right approach is 75 mm and left approach is 55 mm with the current measured geometry, before the GP2 crossing-edge correction. Copying the right advance distance directly to the left would shift its pivot position by 20 mm.

The previous left-only forward-search state, configuration and GP2-only handoff are removed. Both successful turns now enter the existing short straight handoff and normal line steering.

All firmware host tests pass, including centre contact before and after 90 degrees, GP2-only rejection, missing-line failure and continued forward line following after a left turn. The latest simulated left completion is approximately 85.4 degrees with the outgoing tape under GP6. Right completion is approximately 99.4 degrees. These are simulated angles; a course run must verify the calibrated geometry and coasting on the actual chassis.

## Current obstacle sequence

1. Forward monitoring takes a range about every 60 ms. Two readings below 210 mm trigger avoidance. This threshold is 150 mm requested front gap plus 60 mm stopping allowance. The source therefore triggers at up to 21 cm, despite the requested nominal 15 cm gap.
2. Buddy 5 posts the obstacle event. Buddy 2 enters OBSTACLE and stops other motion. Buddy 5 waits up to 1.5 s for idle wheels below 20 mm/s, then holds for 300 ms.
3. One fine scan covers -75 to +75 degrees in 3-degree steps. It takes three pings at each of 51 angles, uses agreement/median filtering, and waits up to 22 s for its profile.
4. The profile estimates front distance and obstacle edges. A usable edge is required. No board-width constraint is used because open-board mode is enabled.
5. Planning uses the 152 by 215 mm body, 100 mm requested side gap and 25 mm edge allowance. It calculates a departure angle between 30 and 75 degrees and a corresponding diagonal move. When necessary it permits one reverse of up to 150 mm followed by a rescan.
6. The robot turns outward, moves diagonally and counter-turns parallel. During PASS the ultrasonic points 80 degrees toward the obstacle. Two near side readings establish the obstacle, then five clear readings identify the far edge. There is also a 60 mm minimum parallel travel.
7. Side distance and body-corner geometry determine the remaining straight clearance before the shallow 30-degree return.
8. The return search drives toward the line until GP6 sees black. It then moves another 80 mm to bring the wheel axle to the line and performs the final encoder alignment turn.
9. On completion Buddy 5 posts AVOID_DONE and Buddy 2 returns to NORMAL. Profile, sensor, motion, clearance or recovery failures can lead to Safe Stop.

## Conditions that can block or compromise completion

| Priority | Location and trigger | Result |
|---|---|---|
| High | task_avoid.c execute_plan adds a 20-degree retry turn after the first line search fails, but retains the plan's original final 30-degree alignment turn. | If the retry finds the line, the extra heading change is not cancelled. For a right bypass the robot can finish about 20 degrees left of its original heading; the opposite bypass mirrors the error. |
| High | task_avoid.c line_centered checks only the GP6 bit. It does not require a fresh calibrated line status, GP2 white, or a stable new contact. | A stale or momentary black reading can end search prematurely. |
| High | execute_plan advances 80 mm and turns after GP6 contact, then handle_obstacle posts success without checking the final line pose again. vehicle_fsm.c accepts AVOID_DONE directly. | A slip or an incorrect retry alignment can be reported as success even though the line follower is handed an off-line robot. The normal both-white corridor permits straight driving, so it does not always recover an unseen line. |
| High | An emergency triggers replanning from the current pose. There is no stored original-line pose or accumulated bypass displacement in task_avoid.c. | A second plan treats the new scan pose as its reference. Its calculated return distance can differ from the actual distance to the original line. |
| Medium | pass_obstacle requires two present side readings before it accepts five clears. | A short, narrow or poorly reflecting obstacle may already be behind the side sensor when PASS starts. All-clear readings then cannot finish the pass. The robot reaches its travel limit or recovery timeout and stops. |
| Medium | SREQ_SIDE monitors only the side. Forward emergency ranging is suspended during PASS. | A new obstacle ahead cannot be detected by that forward emergency check while the servo is looking sideways. |
| Medium | Recovery has a 90 s overall budget including scan, optional reverse/rescan, turns, pass and return search. | Two scans can consume about 44 s; reverse and a long bypass can consume the remainder. Slow or extended runs can fail despite valid individual steps. This is not a hard wall-clock limit because blocking profile/motion waits check their own limits and the global deadline is checked between steps and inside the pass/search loops. |
| Medium | US_NO_RESPONSE for five pings outside INIT is critical, and ten unusable side readings fail the pass. | Wiring, supply interruptions or out-of-range sensor behaviour can force Safe Stop even when physical space is available. |
| Medium | Profile edges are unknown, returns disagree, the sensor appears frozen, or the profile is not delivered within 22 s. | Planning stops with a profile/clearance fault. These checks intentionally avoid constructing a route from insufficient evidence. |
| Medium | Reverse is permitted only once and only up to 150 mm. There is no rear distance sensor. | A close obstacle may require more retreat than permitted. Rear space is an assumption that needs supervised physical testing. |

These are source-level findings in the asynchronous avoidance task. Existing host profile/planning and integration tests do not exercise every sensor timing and side-watch sequence in task_avoid.c. The successful Pico self-test confirms stationary sensor/task operation, but it does not establish that the physical bypass/recovery completes.

The table above records the findings before the following avoidance update.

## Avoidance update

- The side-gap target remains 100 mm. The extra profile/motion reserve is now 15 mm rather than 25 mm. Packed edges round outward, preventing the old inward truncation.
- Return angles are selected from 30–60 degrees to minimize remaining forward travel while checking body-corner clearance. The far-edge decision uses three consecutive clears rather than five, and the fixed 60 mm parallel-travel minimum is removed.
- A side obstacle must still be confirmed before clear readings can end PASS. If that never happens, the robot stops within a 120 mm acquisition move rather than driving the full 2 m command bound.
- Every 100 mm during PASS, the robot stops, samples forward, then restores side monitoring. A new forward obstruction stops the manoeuvre. This adds pauses while keeping the driven route shorter.
- A retry's extra heading change is included in the final alignment turn.
- Line search requires two fresh calibrated centre-only samples. Final alignment checks fresh tracking contact or a contact-to-white transition near its target heading before reporting success.
- Blocking motion/profile waits are capped by the remaining recovery budget.
- Automatic emergency replanning from a displaced pose is disabled. A new hazard stops the manoeuvre rather than creating a return path with an incorrect original-line reference.

The 150 mm-wide, 200 mm-deep integration box now finishes at approximately 808 mm along the original travel axis, compared with about 1,198 mm previously. A new audit clips the rotated body edges to the actual obstacle depth and measures a minimum side clearance of 102 mm. The older centre-position-only overlap check assumed a constant longitudinal body projection during turns; the new check measures the body footprint throughout the path.

All 261 firmware host checks pass. The asynchronous task changes compile into the normal Pico W firmware. Actual ultrasonic reflections, the shorter confirmation timing, repeated forward-check pauses and final line acquisition still require a course run. An oversized measured obstacle width can still increase initial deviation; the sensor evidence must support any further width reduction.
 
## Revised body measurements

The user confirmed a 125 mm overall width including wheels and approximately 175 mm overall length. AV_ROBOT_WIDTH_MM and AV_ROBOT_LENGTH_MM now use those values. All 261 host checks passed with this geometry. The synthetic bypass finishes approximately 770 mm along the travel axis and the body footprint audit retains at least 100 mm side clearance.

TRACK_WIDTH_MM remains 130 mm pending an independent wheel-tread centre measurement. That value is inconsistent with a 125 mm overall outside width and must be corrected from measurement before treating encoder turn angles as calibrated.
 
## Wheel-spacing calibration update

The user measured wheel-tread centre spacing as approximately 110 mm. TRACK_WIDTH_MM now uses 110 mm. Heading calculations in the simulated sensor tests now derive from TRACK_WIDTH_MM rather than assuming the former 130 mm spacing.

With the new spacing, a 15 mm obstacle reserve produced slightly less than 100 mm side clearance in the body-footprint audit. The reserve is now 20 mm. All 261 host checks pass, and the simulated minimum side gap is approximately 101 mm. The normal robot UF2 was rebuilt. Physical course calibration remains necessary because the supplied spacing is approximate.
 
## Working motion calibration restored

The user clarified that the previous turn and movement behaviour was correct despite the earlier inaccurate body measurements. The 125 mm overall width and 175 mm length remain the current physical body measurements. The approximately 110 mm wheel-centre distance is recorded as a physical measurement.

TRACK_WIDTH_MM is restored to 130 mm as the effective calibration used by the previously working turn controller. It should not be interpreted as a claim that the physical wheel spacing is 130 mm. The obstacle reserve returns to 15 mm with that calibration. The normal UF2 is rebuilt using this combination.
 
## Continuous side monitoring requested

Periodic 100 mm forward checks have been removed at the user's request. PASS now uses one continuous straight command while the servo watches the obstacle side. Forward monitoring resumes after far-edge detection or on a failure. A fresh usable range is required when entering side view, with bounded settling time and a view-timeout diagnostic. During PASS, the single ultrasonic sensor is directed sideways.
 
## Buddy 5 restored from ES_Sample_Code

At the user's request, all 18 Buddy 5 C source/header files were restored from the current local ES_Sample_Code repository. The only differences within Buddy 5 are AV_ROBOT_WIDTH_MM = 125 and AV_ROBOT_LENGTH_MM = 175.

This restores the old obstacle detection, scan, path planning, side monitoring, return and recovery behaviour, including its original thresholds and 30-degree return. The newer Buddy 5 recovery changes described above are superseded by this restoration. Other subsystems retain their current implementation.

The restored firmware builds successfully. Validation of this combination reports one failure, INT-10c: the current rotated-body simulation measures approximately 87 mm side clearance against the requested 100 mm. This result is recorded rather than changing the restored old algorithm. The rest of the host checks pass. Physical clearance needs verification before treating the restored configuration as calibrated.
 
## Old recovery with shorter avoidance distances

The restored old recovery flow is retained, with these selected distance changes reapplied at the user's request.

- Body width 125 mm and length 175 mm.
- Side gap 100 mm plus a 15 mm planning reserve.
- Profile packing rounds obstacle edges outward.
- Return angle is calculated between 30 and 60 degrees to minimize forward travel under the body-clearance model.
- Three side-clear readings end PASS after confirmed obstacle presence. The fixed 60 mm parallel-travel minimum is removed.
- The far-edge forward allowance uses measured side ranges.
- The old reverse, line search, retry and final alignment flow remains active. Periodic forward-check pauses are absent.

All 261 host checks pass. The synthetic bypass finishes approximately 770 mm along the travel axis with at least 100 mm side clearance in the rotated-body audit. The UF2 builds successfully. Actual sonar response and course recovery require physical verification.

## Requested 50 mm side gap

AV_SIDE_GAP_MM is now 50 mm. The planning reserve remains 15 mm, so the nominal gap to a measured edge is 65 mm. AV_RETURN_PATH_ALLOWANCE_MM is increased from 5 to 10 mm because the smaller-gap regression initially measured only 47 mm during return. With the final configuration, all 261 host checks pass and the rotated-body audit measures approximately 52 mm minimum side clearance.

For the synthetic 150 mm obstacle, planned centre deviation drops from 247 mm to 197 mm. The centre shift includes the measured obstacle edge and half the robot width; it is not the body-to-obstacle gap.

Other possible causes of a large actual gap are an overestimated ultrasonic obstacle edge, skipped beam compensation when both edges are not known, and physical motion differing from encoder calibration. The current algorithm has no closed-loop lateral gap correction while departing. A one-time side-obstacle-confirmed diagnostic now reports the measured body gap beside the planned gap for course comparison. The restored old recovery flow is retained.

Ultrasonic consensus accuracy update (18 November 2026)

Scan pings now need an agreeing pair within 10% of their mean, with a minimum tolerance of 10 mm. Three unrelated valid distances are rejected instead of automatically accepting the median. A clustered triple uses its median; a single outlier uses the mean of the agreeing pair. Added six host regression checks covering clustered readings, both outlier directions, inconsistent triples, pairs, invalid distances and sample counts.

Parallel side monitoring now filters a rolling window of three valid pings at the existing 60 ms cadence. Two readings are needed initially. History resets for every scan-mode request and non-valid return. No-echo results pass through immediately so stale obstacle ranges cannot prevent far-edge detection. Forward obstacle and emergency monitoring keep their existing immediate sampling behaviour.

Host regression suite passed with zero failures. Normal MQTT firmware rebuilt as build_make/group14_robot.uf2. No live hardware accuracy measurement was performed. The 50 mm requested side gap, geometric allowances, robot dimensions and historical 24-degree beam compensation are unchanged. Verify physical distance and object width using known measurements before claiming absolute accuracy.

IR-guided obstacle recovery and reduced post-edge travel (18 November 2026)

The requested body side clearance remains 80 mm. After the centre sensor finds the returning line and the existing sensor-to-axle advance completes, recovery now replaces the final planned encoder-angle turn with bounded IR-controlled turns. It seeks a confirmed centre-black/right-white edge, then turns left until the centre releases to a confirmed white/white tracking corridor. GP2 uses internal IR_L on this mirrored assembly; GP26 barcode readings do not participate. Three fresh samples are required for edge/corridor confirmation. White alone cannot establish alignment without prior centre-black contact. The right-bypass release seeks centre contact again while turning left, because the initial rightward alignment can coast past that edge. Left bypass permits the confirmed preceding centre contact.

Recovery verifies the corridor during a 20 mm forward move at speed level 1. Either tracking sensor touching black stops the verification move and triggers another bounded correction (up to three attempts). Stale or uncalibrated IR, operator stop, movement errors, turn limits and timeouts cannot report success. The existing normal line follower resumes after successful verification; barcode decoding and junction-turn logic are unchanged. Both-white sensors do not measure an exact lateral centre position or prove heading indefinitely, so course validation is still required.

Post-edge extra advance now targets half of the former body-path-plus-edge allowance, rounded upward, with the calculated body-path distance as a floor. This preserves the configured side clearance during the return sweep. It may therefore reduce by less than half when body clearance dominates. Existing side-clear confirmation and sensor-to-axle distance remain unchanged.

Six new pure-logic checks exercise edge-before-corridor acquisition, right-black rejection, confirmation counts, junction rejection, centre acquisition and half-distance/body-floor calculation. All 273 host checks passed. The host geometry simulation covers the planned path, not the new asynchronous hardware IR manoeuvre. Normal MQTT firmware rebuilt to build_make/group14_robot.uf2. No live course test performed.

Barcode LEFT sensor-gap completion (18 November 2026)

Left and right previously shared a centre-black/right-white finish. LEFT now confirms two distinct centre-only black samples in the permitted angle window after clearing the original crossing. Final left acquisition slows to an axle-centred pivot at the configured minimum controllable wheel speed. After contact it waits for wheel coast, then uses short left/right correction pulses separated by settling periods. Completion requires two distinct centre/right-white samples while both measured wheel speeds are at most 3 mm/s. Missing/stale IR, both-black ambiguity, angle limits and existing command timeout cannot report success. RIGHT completion and its straight handoff are preserved.

Buddy 3 resets tracking directly after successful LEFT, so normal edge correction begins immediately rather than inheriting the 150 ms right-turn straight hold. Barcode decoding and mapping are unchanged. Pre-turn advance settings remain LEFT 55 mm and RIGHT 75 mm, referenced to the recorded GP2 junction exit and existing half-line-width compensation.

Corrected the left-turn unit fixture to centre-first/right-second sensor order. Added checks for centre never clearing, only early contact, both-black ambiguity and immediate post-left edge correction. Updated the geometric cross-junction test to require actual white sensors on opposite sides of the outgoing tape after coasting, then verify continued forward line following. Whole-mm offsets truncate; full micrometre sensor geometry remains the pass criterion.

All 277 host checks passed. Simulated left final heading was -95.412 degrees, within the existing 10-degree tolerance; both tracking sensors were white on opposite sides of the outgoing tape. Normal MQTT firmware rebuilt to build_make/group14_robot.uf2. Physical course validation remains necessary.

Equal junction advance (18 November 2026)

Both LEFT and RIGHT now use the same 75 mm advance setting, retaining the GP2 crossing-exit reference and tape half-width compensation. LEFT aliases RIGHT so future edits cannot accidentally diverge. Left pivot offset is derived from the shared advance and IR-ahead distance (80 - 75 = 5 mm); retaining the old 25 mm left offset with the longer advance failed geometric line acquisition. The working RIGHT pivot remains 25 mm. Left final IR-guided acquisition, pulse corrections and immediate line-following handoff remain enabled. Final left wheel-speed settling threshold tightened from 3 to 1 mm/s before white-gap confirmation.

Fixed an independent host plant rounding issue: zero-power wheels could remain at +/-3 mm/s indefinitely because integer lag updates truncated to zero. Unpowered wheels now decay the remaining integer speed toward zero. Added a regression for this and a simulated-motion check that LEFT/RIGHT actually advance equally before turning. All 279 host checks passed, including geometric left corridor alignment after coast and continued line following. Normal firmware rebuilt to build_make/group14_robot.uf2. Physical course test remains outstanding.

Avoidance recovery handoff loop fix (18 November 2026)

Confirmed a control-flow cause for repeated left pivots after left-side avoidance: align_line always issued another left release turn after finding centre-black, and the 20 mm verification aborted on either tracking sensor touching black and restarted alignment. These behaviours could keep Buddy 5 owning the wheels while ordinary line-following edge correction was needed.

Removed the extra release pivot and verification/restart loop. Recovery now seeks three fresh centre-black/right-white samples in the return direction, stops and waits for wheel settling (at most 1.5 seconds), then posts the existing avoidance-done event. Buddy 3 resets tracking on NORMAL entry and applies forward-moving GP6-left / GP2-right corrections toward the white corridor. The final fixed plan turn remains skipped. Removed unused release/verification movement constants. Search contact now checks calibration and sample freshness and requires a successful stop acknowledgement.

Added two line-follower handoff regressions: centre contact moves forward while correcting left; overshoot onto GP2 moves forward while correcting right. All 281 host checks passed. These tests cover pure follower handoff behaviour; the asynchronous Buddy 5 recovery still requires a physical course test. Normal firmware rebuilt to build_make/group14_robot.uf2. The 80 mm requested obstacle gap and shortened post-edge travel calculation remain unchanged.

Shared mirrored barcode turn controller (18 November 2026)

LEFT and RIGHT now share one axle-centred pivot, speed profile, acquisition slowdown, settle/trim controller, confirmation count, angle bounds and immediate line-following handoff. Removed the left-specific pivot, speed phase, trim function and state names. Nominal command remains +/-90 degrees; wheel directions are mirrored. Sensor order is direction-aware: RIGHT acquires GP2 first, LEFT acquires GP6 first. Both finish after confirmed centre/right-white samples at settled wheel speeds. This supersedes the right centre-contact/150 ms straight hold and the former left-only correction loop.

Both advance settings now use the same sensor-to-axle distance, 80 mm, with the existing GP2 crossing-exit/tape-width reference correction. This replaces the earlier 75 mm advance and direction-dependent offset pivots. Sensor mounting is asymmetric, so real sensor-based stop headings need not be identical despite identical motion settings. Geometric simulation ended RIGHT at +81.940 degrees and LEFT at -95.790 degrees, both within the existing 10-degree tolerance and both straddling the outgoing tape. Do not present those as measured hardware results.

Added a mirror regression comparing wheel paths and headings under mirrored sensor sequences. The cross-junction test now checks continued forward line following after BOTH left and right. All 283 host checks passed; normal firmware rebuilt as build_make/group14_robot.uf2. No physical course validation performed. Buddy 5 recovery, the 80 mm obstacle gap and barcode decoding/mapping are unchanged.

Early outgoing branch contact from hardware trace (18 November 2026)

User trace decoded reversed F successfully and queued/executed LEFT. During the pivot, centre-only black appeared while targets were still approximately -47/+53 mm/s, then cleared and GP2 touched black before the minimum-speed -22/+22 phase. This strongly points to outgoing contact before the former 70 encoder-degree acceptance window. Contact outside that window reset confirmation; later white samples could not establish a leading edge. Fault change 0x0900040 -> 0x0980040 adds SYS_FLT_RECOVERY_TIMEOUT (bit 19), consistent with unconfirmed turn termination. Wi-Fi retries are separate from this qualification failure.

Shared left/right junction acquisition and slowdown now start at a 45-degree initial-contact guard, rather than 70. The original crossing must clear BOTH tracking sensors for two fresh samples before outgoing contact can qualify. Leading-only contact still needs two fresh samples, followed by the same settled white-gap alignment; the 105-degree maximum and timeout remain. U-turn acquisition window is unchanged. Added regressions for a branch at 60-65 encoder degrees in both directions; prior early-original-contact, ambiguous, missing and stale-sensor failures still pass.

Added low-volume motion logs on sensor-pattern and turn-stage changes, showing LEFT/RIGHT, encoder angle, C/R, clear-junction / initial-angle-guard / seek-branch / align state and contact count. Existing advance log is retained. All 285 host checks passed; normal MQTT firmware rebuilt as build_make/group14_robot.uf2. Requires hardware retest to confirm the observed contact angle and successful handoff. 80 mm shared advance and Buddy 5 behaviour are unchanged.

Final alignment shaking from hardware trace (18 November 2026)

The latest trace confirms successful branch acceptance at 59-60 encoder degrees (contacts 1/2 then align 2/2). The previous early-angle rejection is resolved in this run. Final alignment then produced one-tick target +/-16 mm/s pulses followed by zero-duty waits; measured speeds fluctuated around 1-2 mm/s. This exposed inadequate pulse duration through acceleration ramp/static friction and an overly strict 1 mm/s settling threshold. The run was subsequently aborted by a 4011 mg collision event; whether that impact arose from handling or the observed shake is not established.

Replaced repeated one-tick pulse/200 ms wait corrections with continuous minimum-controllable-speed correction while one tracking sensor remains black. The initial 200 ms settle remains. On both-white, stop immediately and require two fresh white-gap samples with speeds within 3 mm/s, allowing the observed small encoder jitter. Repeated unchanged-but-fresh readings maintain motor drive rather than cutting it each tick. Both turn directions use the same logic. Collision protection, original-crossing clearance, 45-degree initial guard, 105-degree limit and timeouts remain.

Added a regression verifying sustained nonzero drive for at least three consecutive alignment cycles; all 286 host checks passed. Normal firmware rebuilt to build_make/group14_robot.uf2. No live hardware retest performed.
