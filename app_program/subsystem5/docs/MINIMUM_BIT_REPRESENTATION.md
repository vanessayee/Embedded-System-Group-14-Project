# Buddy 5: minimum-bit representation and final integration check

Implemented on 2026-11-21 for Change Requirement 1. The change reduces task-owned storage while preserving the configured avoidance behaviour. No motor commands or flashing were performed.

## ARM storage measurements

| Local type | Before (bytes) | After (bytes) |
| --- | ---: | ---: |
| S5_SCAN_REQ | 5 | 4 |
| S5_SAMPLE | 8 | 4 |
| S5_PROFILE | 32 | 32 |
| S5_SCAN_STATUS | 8 | 4 |
| S5_AVOID_STATUS | 7 | 4 |
| OP_SAMPLE | 4 | 4 |
| AV_STEP | 8 | 4 |
| AV_PLAN | 104 | 72 |
| AV_LINE_LOCK | 4 | 4 |

Measured using the configured Cortex-M0+ ARM compiler. Alignment is preserved; local bitfield layouts are never used as queue or network encodings.

Normal firmware before/after Buddy 5: text 395686 -> 397030 bytes (+1344), data 128 -> 128, BSS 84788 -> 84776 (-12). Static RAM falls 84916 -> 84904 bytes. Smaller local objects also reduce their object footprint; configured task stacks remain the same and stack high-water usage was not measured.

Across all five requirement stages, the saved original normal firmware was text 389038, data 128, BSS 85348. Current static RAM is 572 bytes lower and text is 7992 bytes higher. Buddy 1 additionally reduces dynamic queue allocations by 288 bytes; this is separate from static RAM savings.

## Bounds and preserved values

- Request: type 2 bits, scan ID 3, signed angles 8 each, step 4. Sample: angle 8, distance code 10 (including 1022/1023 sentinels), ID 3, phase/last 1 each, result 2.
- Scan status uses its existing 28 logical bits. Avoidance status uses its existing 20 logical bits, including four-bit modulo count and three-bit failure reason.
- OP_SAMPLE uses signed 8-bit angle, 2-bit status and signed 13-bit millimetre distance (-1 and 0..4000), in one aligned word. Both 64-sample buffers retain their capacity and four-byte stride.
- AV_STEP uses 2-bit kind and signed 12-bit argument, covering all generated turns and moves up to MCMD_MOVE_MAX_MM=2000. AV_PLAN retains a four-bit count including 8 completed steps and two-bit side. All calculated geometry fields stay full 32-bit engineering values.
- Scan health uses the original 8-bit silent/good/detection runs and a 2-bit three-reading history count. Existing saturation and detection-counter byte wrap are preserved, rather than changing behaviour as part of this requirement.
- Scan control uses two-bit mode, Boolean side activity and a signed eight-bit angle. Three-ping vote counters use two bits each.
- Profile collection uses seven bits for count 0..64, four bits for scan ID including an unset sentinel 8, two bits for phase including unset 2, eight bits for absolute angle limit including 128, and one-bit aborted flag. The former private 255 sentinels are replaced by values outside the real message ranges; the first scan still always resets collection.
- Pass confirmation runs use 2/2/4 bits for thresholds 2/3/10, with a four-bit sequence and Boolean flags. The code still requires seeing the obstacle before clear readings can establish its far edge.
- The public AV_LINE_LOCK.run counter remains full-width: its function continues incrementing after confirmation, so narrowing it to two bits would change the API behaviour. Geometry calculations, coordinates, odometry, elapsed times and deadline arithmetic also remain full width. Profile values are not narrowed before existing clamping/rounding.
- Compile-time checks protect plan argument, scan-count, sensor range, servo-angle, vote and confirmation storage bounds. The side-selection function uses a temporary addressable byte before assigning its result to the plan bitfield.

## Behaviour and integration retained

(Values as of 21 November; see "Update 25 November 2026" below for the current gap and trigger.) Sensor calibration and GPIO, 125 x 175 mm body geometry, requested 80 mm side gap plus 15 mm uncertainty reserve, detection/caution/emergency distances, servo motion, scan coverage, three-ping consensus, beam compensation, calculated turns, reverse limits, live far-edge detection, reduced post-edge advance and sensor-based line handoff are unchanged.

The 150 mm configured detection gap still includes the existing 60 mm stopping allowance, giving the same 210 mm sensor trigger threshold with the sensor above the bumper. This implementation does not retune that distance.

The three Buddy 5 tasks retain their priorities, queues, queue capacities, ownership and timeouts. Shared atomic status words, two-word sequence-protected profiles, motion command/result formats and MQTT frames are unchanged. Buddy 2 still owns motors, Buddy 3 still supplies fresh IR status and resumes moving line following, and Buddy 1 still reports the same telemetry and events. Drivers, profile geometry algorithm, message packers and subsystem initialization/test orchestration files are byte-identical to the saved baseline. All 97 Buddy 1-4 files match the comparison snapshot taken during this stage.

## Verification

- Current host suite: 341 checks in suite summaries, zero failures, with -O2 -Wall -Wextra -Werror and the existing unused-parameter exception. Eight new checks cover sizes, counter limits/wrap, profile sentinels/capacity, pass flags, full sensor range, signed actuator arguments and the non-truncated line-lock counter.
- All original 333-check suites run against isolated before/after Buddy 5 sources: all 388 test output lines identical. Integration covers avoidance motion commands, line recovery, barcode junction actions, command priority, startup, safety faults, network loss and all 48 telemetry fields.
- Deterministic before/after trace: 100000 wire cases and 600 synthetic scan/profile/plan cases, identical hash 13128727554881016042. Includes signed angles, distance sentinels, oversized engineering values reaching the original packer clamps, both bypass directions, invalid echoes, calculated reverse/plan steps and return-clearance calculations.
- Backend: 43 tests pass. Binary codec: 14 checks pass, including golden frames.
- Normal and TEST=1 MQTT firmware builds succeed without compiler warnings. The friendly group14_robot.uf2 and group14_selftest.uf2 files are refreshed.
- No new unused helper/function was introduced; new storage types and fields are referenced by their owning tasks. Previous cleanup is retained.

Per-stage equivalence evidence for Buddies 1-4 remains in their MINIMUM_BIT_REPRESENTATION.md reports and testing/subsystemN/logs. These checks support unchanged software behaviour on the tested inputs; they do not certify identical execution time or physical sensor performance. Compact-field accesses add instructions, as reflected in the flash increase.

## Supervised robot verification

Use the normal UF2 for the course test. Confirm startup waits for readiness/MQTT; normal line following and barcode instructions in both scan directions; left/right/U-turn completion and subsequent tracking; obstacle detection, width, bypass and rejoin; sideways hump last/highest height on the dashboard; and motor stopping on handling/collision. Compare task timing/overruns against the saved firmware under the same hotspot conditions. The self-test image has been built but not executed on the Pico in this stage.

## Update 25 November 2026 (after the obstacle-accuracy and STOP changes)

Behaviour changes made after the 21 November stage, with their storage:

| Change | Storage added / changed | Bound and reason |
| --- | --- | --- |
| Side gap 80 → 50 mm; corridor 190 mm | none (compile-time constants) | `AV_SIDE_GAP_MM`, `AV_REQUIRED_CORRIDOR_MM` |
| Trigger 210 → 155 mm (150 mm gap + 5 mm stopping allowance) | none (constants) | `S5_STOPPING_ALLOWANCE_MM 5` |
| Sweep ±75° → ±69° (51 → 47 samples) | none; profile count still 7 bits | 47 ≤ `OP_MAX_SAMPLES` 64; abs limit 69 fits the 8-bit field |
| 45 mm servo-to-transducer radius, face-band + echo-envelope edges | locals only (`op_build_sensor`, edge helpers) | no new stored state |
| Pass heading correction | `S5_PASS_STATE.corrected` : 1 bit | at most one correction per pass |
| Least-squares side trend `AV_TREND` | `n` : 16-bit saturating (UH); odometry `t_first/t_last` 32-bit; sums 64-bit | a 2 m pass gives < 700 samples; sums of mm² over 700 samples need 64 bits, so they stay full width like other geometry arithmetic |
| Operator STOP ends sweep / avoidance | none: reads the existing `SYS_FLT_REMOTE_STOP` bit (`S5_OPERATOR_STOP()`) | no new shared word |

Packed messages, queue formats and MQTT frames are unchanged, except that
the profile message now rounds obstacle edges outward and side clearance
down (`s5_profile_pack`), so packing never shrinks the obstacle or enlarges
free space (INT-15b). Host suite after these changes: 375 checks, 0 failed
(`testing/integration/logs/2026-11-25_host_all_buddies.txt`).
