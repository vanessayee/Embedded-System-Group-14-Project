# Subsystem 3 — test log

**Subsystem:** Line following and barcode decoding (Buddy 3) · **Student ID:** 2501720 · **Code:** `app_program/subsystem3/`

See [testing/README.md](../README.md) for how to run tests and record results.
Automated IDs below are defined in `s3_unit_tests.c` and
`subsystem3_test.c`. The target checks require the `TEST=1` image and a
running RP2040. A passing software simulation does not establish performance
with the physical sensor and track.

## Test cases

Define a test here before running it. IDs are kept as assigned in the source;
gaps are intentional and IDs are never reused.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | Generated Code 39 table and measured `*`, `A`, `Z` patterns | Barcode symbology | auto | `s3_run_unit_tests()` | 44 unique patterns; expected patterns match |
| S3-U02 | Eleven supplied barcode sample vectors | Barcode sample decoding | auto | Decode all vectors in `s3_unit_tests.c` | All decode to the printed symbol |
| S3-U03 | Barcode scanned in reverse | Read direction | auto | Reverse the measured A run and decode | A decodes with reversed flag |
| S3-U04 | Scale independence | Decode across print scales | auto | Scale measured A widths by 3.5 | A decodes |
| S3-U05–U09 | Invalid count, width contrast, corrupted/missing delimiters, and oversized gap | Reject malformed scans | auto | Run malformed vectors through `c39_decode()` | Each malformed input is rejected |
| S3-U10 | Capture simulation over A at three speeds | Barcode capture | auto | 15 trials each at 100, 200, 300 mm/s with simulated ripple and optical bias | 15/15 valid reads at each speed |
| S3-U11a | First Z capture and diagnostic durations | Capture result and diagnostics | auto | Simulate first pass over Z | Valid Z with 29 diagnostic durations |
| S3-U11 | Repeated Z capture | Duplicate suppression | auto | Decode Z again within the simulated repeat interval | Result is `BCR_DUPLICATE` |
| S3-U12–U13 | Isolated dark run and stalled partial scan | Invalid capture handling | auto | Feed incomplete edges and poll past timeout | Invalid result; stall reports `BC_ERR_STALLED` |
| S3-U14 | Odometry jump during partial capture | Current decoder timing interface | auto | Change odometry without adding sensor edges | Capture remains open; odometry does not end it |
| S3-U15–U17 | Default navigation map, map overrides, and command conversion | Barcode-to-motion mapping | auto | Check defaults, invalid map inputs, and motion arguments | Mappings and Buddy 2 command types/arguments match code |
| S3-U18–U29 | IR calibration, steering, line loss/recovery, junction detection, and barcode-sensor isolation | Line-following behavior | auto | Run line-following state-machine cases in `s3_unit_tests.c` | Each case meets its stated steering/event expectation |
| S3-U30–U31 | Line and barcode status-word packing | IF-L1 / IF-L3 status interfaces | auto | Pack then unpack boundary-value structures | Fields round-trip correctly |
| S3-U32 | Line task period and overrun count | Line task runtime | auto (target) | Observe task statistics during test image run | 48–52 runs/s and no overruns |
| S3-U33 | Line status sequence and timestamp advance | IF-L1 | auto (target) | Compare status before and after 200 ms | Sequence and timestamp advance within stated bounds |
| S3-U34 | IR calibration decision during INIT | IR startup | auto (target) | Wait up to one second in INIT | Calibration succeeds or `SYS_FLT_IR_CAL` is set |
| S3-U35 | Barcode idle/overflow state and frozen GPIO callback registration | Barcode task and GPIO interface | auto (target) | Inspect status and attempt late callback registration | Idle, no overflow, registration returns `E_OBJ` |
| S3-U52–U59 | Line corridor, recovery, mirrored sensor, calibration, and barcode-sensor independence cases | Line following and sensor separation | auto | Run the corresponding logic cases in `s3_unit_tests.c` | Assertions pass; barcode input does not perturb line tracking |
| S3-U60 | Decode with stationary-wheel inputs | Edge-time decoding | auto | Feed a synthetic A using zero wheel speed/odometry | A decodes from edge timing |
| S3-U61–U63 | Damaged/recovered reverse scans and recorded A durations | Code 39 robustness | auto | Feed recorded timing vectors | Direction/error classification and valid A result match assertions |
| S3-U64–U71 | Barcode toggle independence, long corridor, steering slew/response, and post-turn behavior | Line-following behavior | auto | Run corresponding state-machine cases | Steering/events remain within asserted behavior |
| S3-U72–U73 | Recorded reverse F timing and gap validation | Reverse decoding | auto | Decode recorded F, then enlarge its gap | F accepted; oversized gap rejected |
| S3-U74–U75 | Full uppercase alphabet, forward and reverse | Code 39 alphabet support | auto | Decode A–Z patterns in both directions | All 26 letters decode in each direction |
| S3-U76–U77 | Recorded reverse F with weak bar separation; ambiguous forward scan | Width classification | auto | Feed the two recorded vectors | Reverse F accepted; ambiguous width rejected |
| S3-U78 | A–Z captures at three simulated label lengths, both directions | Capture and decode scales | auto | Simulate 170, 250, 339 mm labels | All 156 cases decode correctly |
| S3-U79 | Ambiguous 25-element scan | Reject insufficient element count | auto | Decode the ambiguous partial vector | `C39_ERR_COUNT`; cannot produce a command |
| S3-U80–U81 | 27-element single-run merge recovery and ambiguity rejection | Missed-transition recovery | auto | Test merge positions across A–Z in both directions | Recoverable cases return correct letters; ambiguous cases reject |
| S3-U82 | Post-turn line reacquisition | Line following after barcode manoeuvre | auto | Exercise turn completion and sensor sequences | Pivot/straight recovery follows assertions |
| S3-U84 | Long, merged-run W capture | Capture completion before stall timeout | auto | Simulate a 339 mm W with a 2 s merged run | W decodes as a 27-element recovery |
| S3-U85 | Right-turn straight settling | Line following after right turn | auto | Exercise right-turn completion and centre-line samples | Settles straight, then resumes steering |
| S3-M01 | Physical barcode read reliability on the track | Hardware validation | manual | On the vehicle, test the actual print at intended speeds; save a video/log with setup and outcomes | Record the observed read count and failures; no pass threshold is claimed by the software tests |
| S3-M02 | GP26 wiring, sensor polarity/contrast, and edge capture behavior | Hardware interface validation | manual | Check raw sensor output on white/black and observe captured transitions; record setup/evidence | Confirm wiring and that transitions are preserved at operating speed |

*Method:* **auto** = `TEST_CHECK` in code; result printed on the console.
**manual** = a person observes the result on hardware and saves evidence.
S3-M01 and S3-M02 are planned manual checks, not recorded results.

## Results

Add a row for each test ID and each run. Do not enter results until the test
has actually been run; keep the console log or manual evidence under
`testing/subsystem3/logs/`.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
