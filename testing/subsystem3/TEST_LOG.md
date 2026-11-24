# Subsystem 3 — test log


**Subsystem:** Line following and barcode navigation  ·
**Student ID:** 2501720  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Test definitions below mirror the current automated checks. Pico-only tests must be run on target.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | Code 39 table: 44 unique patterns, 3 wide each; * A Z as measured |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | every supplied sample decodes (PDF A/Z, PPTX A/F/Z/W/X/Y, photo A) |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U02 PASS` |
| S3-U03 | barcode read in the opposite direction decodes (reversed flag) |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U03 PASS` |
| S3-U04 | print-scale independent (ratio classification) |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U04 PASS` |
| S3-U05 | invalid: element count not 10k-1 |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U05 PASS` |
| S3-U06 | invalid: no wide/narrow contrast |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U06 PASS` |
| S3-U07 | invalid: corrupted start symbol rejected (no command) |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U07 PASS` |
| S3-U08 | invalid: wide inter-character gap rejected |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U08 PASS` |
| S3-U09 | invalid: missing start/stop character |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U09 PASS` |
| S3-U10 | 15/15 reads at 100, 200 and 300 mm/s (+-15 % ripple, 5 % speed bias, 0.6 mm blur) |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U10 PASS` |
| S3-U11 | same barcode again within 2 s: DUPLICATE (no second command) |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U11 PASS` |
| S3-U11a | first pass over Z: VALID, with 29 diagnostic element widths |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U11a PASS` |
| S3-U12 | isolated black run times out without a complete symbol |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U12 PASS` |
| S3-U13 | no edge for 3 s while capturing: aborted (bounded) |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U13 PASS` |
| S3-U14 | short white gap keeps sensor capture open |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U14 PASS` |
| S3-U15 | default barcode mapping: A=RIGHT, W=LEFT, Z=LEFT, X=STRAIGHT, Y=U-TURN; F/U unmapped |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U15 PASS` |
| S3-U15m | runtime mapping holds all 26 letters independently and supports unmapping |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U15m PASS` |
| S3-U16 | mapping overrides work; '*' and nav > 3 rejected; reset restores defaults |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U16 PASS` |
| S3-U17 | LEFT/RIGHT/U-TURN/STRAIGHT -> encoder TURN/MOVE commands |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U17 PASS` |
| S3-U18 | stable centre-on-black start: configured black-high polarity |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U18 PASS` |
| S3-U19 | all-white start is stable and immediately qualifies IR startup |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U19 PASS` |
| S3-U20 | all-black start is stable and immediately qualifies IR startup |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U20 PASS` |
| S3-U21 | flickering sensor while still -> calibration failure |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U21 PASS` |
| S3-U22 | centre black: gradual physical-left correction through mirrored steering |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U22 PASS` |
| S3-U23 | wide line filling the C-to-IR_L corridor: drive straight |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U23 PASS` |
| S3-U24 | line on outer boundary: smooth moving correction, never a pivot |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U24 PASS` |
| S3-U25 | explicit known loss uses an arc, then pivot; normal white corridor does not trigger it |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U25 PASS` |
| S3-U26 | line reacquired on centre -> FOUND and gradual physical-left correction |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U26 PASS` |
| S3-U27 | line never found: keeps pivoting towards IR_R; no RECOVERY_FAIL |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U27 PASS` |
| S3-U28 | GP6 + GP2 black for 60 ms, barcode white: one JUNCTION event |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U28 PASS` |
| S3-U29 | black under the barcode sensor (IR_R) beside the line does not disturb steering |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U29 PASS` |
| S3-U30 | line status word round trip at limits (steer -63) |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U30 PASS` |
| S3-U31 | barcode status word round trip at limits |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U31 PASS` |
| S3-U32 | Line Following runs every 20 ms (50 per s) within half its period |  | auto (target only) | `subsystem3/subsystem3_test.c` | `[TEST] S3-U32 PASS` |
| S3-U33 | line status word republished every period with a fresh timestamp (IF-L1) |  | auto (target only) | `subsystem3/subsystem3_test.c` | `[TEST] S3-U33 PASS` |
| S3-U34 | IR calibration decided within 1 s of INIT (calibrated, or IR_CAL fault): never stalls |  | auto (target only) | `subsystem3/subsystem3_test.c` | `[TEST] S3-U34 PASS` |
| S3-U35 | barcode reader idle outside NORMAL, no edge overflow; GPIO callbacks frozen once running |  | auto (target only) | `subsystem3/subsystem3_test.c` | `[TEST] S3-U35 PASS` |
| S3-U52 | white corridor straight; centre black ramps physical-left correction |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U52 PASS` |
| S3-U53 | black under IR_R alone is not the line: lost, turning towards IR_R, search not ended |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U53 PASS` |
| S3-U54 | lost-line turn, then IR_L has the line: FOUND, smooth moving correction |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U54 PASS` |
| S3-U55 | lost-line arc ends after LF_LOST_ARC_MM of travel, then pivot |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U55 PASS` |
| S3-U56 | line spanning both corridor edges after a find: drive straight |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U56 PASS` |
| S3-U57 | short white corridor remains straight and ramps smoothly at the centre edge |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U57 PASS` |
| S3-U58 | white corridor gradually returns to straight with no loss event |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U58 PASS` |
| S3-U59 | physical-right-only black start qualifies and preserves its position |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U59 PASS` |
| S3-U60 | stationary wheels: digital transition timing decodes A |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U60 PASS` |
| S3-U61 | recorded reverse A with damaged final delimiter rejected as reverse |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U61 PASS` |
| S3-U62 | measured reverse A decodes when final delimiter is valid |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U62 PASS` |
| S3-U63 | exact recorded A: wide first white space preserved, biased gaps accepted |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U63 PASS` |
| S3-U64 | barcode toggles on GP26 do not block stable GP6/GP2 startup |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U64 PASS` |
| S3-U65 | barcode input cannot change line steering, speed, junction or recovery events |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U65 PASS` |
| S3-U66 | 30 s both-white corridor remains straight; barcode toggling never triggers search |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U66 PASS` |
| S3-U67 | centre LEFT -> right sensor RIGHT -> white: hold, then STRAIGHT; fixed speed |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U67 PASS` |
| S3-U68 | edge correction in the first 20 ms; reverse correction within 40 ms, wheels keep moving |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U68 PASS` |
| S3-U69 | centre stays black (curve): correction grows to LF_EDGE_STEER_MAX, never beyond |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U69 PASS` |
| S3-U70 | short contact -> straight; sustained contact -> turn held LF_HOLD_MS, then straight |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U70 PASS` |
| S3-U71 | held turn ends at once when the other sensor sees black or at a junction |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U71 PASS` |
| S3-U72 | recorded reverse F survives one short narrow-space measurement |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U72 PASS` |
| S3-U73 | reverse F still rejects a genuinely oversized gap |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U73 PASS` |
| S3-U74 | all A–Z decode in forward direction |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U74 PASS` |
| S3-U75 | all A–Z decode in reverse direction |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U75 PASS` |
| S3-U76 | recorded reverse F with 1.35x bar separation decodes |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U76 PASS` |
| S3-U77 | recorded forward scan with 1.19x bar separation is rejected |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U77 PASS` |
| S3-U78 | A-Z capture and decode both ways at 170, 250 and 339 mm (156 cases) |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U78 PASS` |
| S3-U79 | 25-element scan shared by A and B cannot queue a turn |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U79 PASS` |
| S3-U80 | 27-element recovery covers A-Z both ways with no wrong letter across all merge positions |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U80 PASS` |
| S3-U81 | ambiguous 27-element captures are rejected |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U81 PASS` |
| S3-U82 | after a barcode turn: GP2 on the line -> pivot right until white, then straight |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U82 PASS` |
| S3-U84 | 339 mm W with 2 s merged run finishes capture before the stall timeout |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U84 PASS` |
| S3-U85 | right branch under GP6: drive straight briefly then resume normal steering |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U85 PASS` |
| S3-U86 | left gap handoff resumes centre-edge correction without straight-only delay |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U86 PASS` |
| S3-U87 | avoidance centre contact hands to forward left correction and white-gap tracking |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U87 PASS` |
| S3-U88 | avoidance overshoot onto GP2 uses forward right correction instead of search pivot |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U88 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-18 | S3-U35 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` |  |
| 2026-11-18 | S3-U01 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U02 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U03 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U04 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U05 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U06 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U07 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U08 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U09 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U10 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U11a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U11 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U12 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U13 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U14 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U15 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U16 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U17 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U18 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U19 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U20 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U21 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U22 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U23 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U24 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U52 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U25 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U26 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U58 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U57 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U55 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U56 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U27 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U28 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U29 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U53 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U54 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U30 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U31 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U32 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U33 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U34 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U35 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-18 | S3-U32 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem3_target.txt` | Pico target run. |
| 2026-11-18 | S3-U33 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem3_target.txt` | Pico target run. |
| 2026-11-18 | S3-U34 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem3_target.txt` | Pico target run. |
| 2026-11-18 | S3-U35 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem3_target.txt` | Pico target run. |
| 2026-11-25 | S3-U01 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U02 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U03 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U61 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U62 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U04 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U05 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U79 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U06 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U07 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U08 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U09 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U72 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U73 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U74 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U75 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U78 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U80 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U81 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U76 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U77 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U10 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U11a | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U11 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U12 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U13 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U14 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U60 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U63 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U84 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U15 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U15m | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U16 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U17 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U18 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U19 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U20 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U59 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U21 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U22 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U23 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U24 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U52 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U25 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U26 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U58 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U57 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U55 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U56 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U27 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U28 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U29 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U53 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U54 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U64 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U65 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U66 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U67 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U68 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U69 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U70 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U71 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U82 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U85 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U86 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U87 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U88 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U30 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
| 2026-11-25 | S3-U31 | 6a5b715 + 25 Nov working tree | TeyHaiSeang | PASS | `logs/2026-11-25_subsystem3_host.txt` | Host all-buddies run. |
