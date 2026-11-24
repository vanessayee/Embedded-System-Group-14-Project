# Buddy 2 minimum bit representation

Buddy 2 now stores bounded motion and vehicle state in explicit C bit-fields
owned by the motion task. This stage follows the Buddy 1 change. It does not
change calibration, motion algorithms, the PID arithmetic, command priority,
shared word encodings, or the motor/encoder/GPIO drivers. This is Buddy 2 evidence;
the remaining buddies still need their own representation reviews.

## Ownership and synchronization

The new local bit-field layouts are never transmitted or shared directly with
another task or ISR. MC_IN, MC_OUT, MOTION_CTRL, VFSM and their helper records
belong to the motion task; tests use isolated instances. Other buddies continue
to use the same motion API and packed words. Motion status is still published
as two 32-bit words under the existing sequence counter.

app_sync.h, sys_state.c, motion_api.c and all motor/encoder/GPIO driver source
files are byte-identical to the saved baseline. Shared fault/event/readiness
updates retain atomics. Encoder multiword records retain their sequence counter.
Watchdog, motor-ready and encoder-direction flags retain aligned word storage
and existing ownership. They are not combined into shared C bit-fields.
Per-source sequence bytes remain separate because different tasks own them;
packing multiple producers into the same byte would introduce a read-modify-write
race. Their existing masks restrict sequence values to four bits.

## Representation table

Sizes below include C ABI padding. Field widths describe application information;
word and byte padding are reported separately, rather than claiming that padding
represents another application value. Normal alignment is retained for arithmetic
and the 20 ms control loop. C layouts are never used as a wire specification.

| Record or field | Valid range or meaning | Required logical bits | Storage and justification |
|---|---|---:|---|
| Controller state | IDLE, STRAIGHT, TURNING, FAULT | 2 | MOTION_CTRL.state |
| Active command present | Boolean | 1 | has_active |
| Queue head | 0..3 | 2 | q_head |
| Queue count | 0..4 | 3 | q_count; still four queued commands plus an active command |
| Active command and each queued command | existing packed command schema | 28 | existing UW containers retain four reserved zero bits; format validation occurs before queuing |
| Outgoing line edge observed | Boolean | 1 | seek_seen |
| Turn contact samples | 0..2 currently; capacity 0..3 | 2 | turn_contact_samples |
| Turn gap samples | 0..2 currently; capacity 0..3 | 2 | turn_gap_samples |
| Previous turn IR sequence | 0..15 or no previous sample | 5 | turn_line_seq; private sentinel changes from 255 to 16 |
| Turn alignment active | Boolean | 1 | turn_align |
| Remaining settling delay | 0..200 ms | 8 | turn_wait_ms |
| Advancing to junction turn point | Boolean | 1 | advancing |
| Crossing black observed | Boolean | 1 | adv_r_black |
| Advance reference captured | Boolean | 1 | adv_ref |
| Both crossing sensors black | Boolean | 1 | junction_black |
| Saved junction edge lifetime | 0..200 ms | 8 | junction_edge_ms |
| Controller measurement primed | Boolean | 1 | primed |
| Immediate stop pending | Boolean | 1 | hard_stop |
| Left and right stall flags | Boolean each | 1 each | stall_l, stall_r |
| Fault retry count | 0..255, existing saturation | 8 | fault_retries; not reduced to the two-bit reported escalation count |
| Accumulated local fault mask | existing SYS_FLT namespace | 27 | fault_bits |
| Escalation latched | Boolean | 1 | escalated |
| Vehicle mode | five modes, codes 0..4 | 3 | VFSM.mode; codes 5..7 still reach the invalid-mode fallback |
| Vehicle epoch | modulo 8 | 3 | VFSM.epoch |
| Vehicle calibration and line flags | Boolean each | 1 each | ir_ok, imu_ok, line_lost |
| Local buddies seen ready in this INIT | Boolean | 1 | VFSM.local_seen; uses a spare bit of the existing word, so VFSM stays 8 bytes (S2-U65/U66) |
| Vehicle events | ten defined independent bits | 10 | VFSM_IN.events |
| Vehicle fault and critical masks | 27 defined independent bits each | 27 each | VFSM_IN.faults, crit_mask |
| Startup readiness | five independent buddy bits | 5 | VFSM_IN.ready |
| Vehicle input flags | Boolean each | 1 each | armed, nav_started, nav_finished, resume |
| Vehicle output changed / previous mode / fault mask | Boolean / modes / defined faults | 1 / 3 / 27 | VFSM_OUT |
| Motion input flags | Boolean each | 1 each | enc_valid, armed, steer_fresh, line_lost_cap, line_seen_fresh |
| Motion input vehicle mode | five modes plus invalid codes 5..7 | 3 | MC_IN.vmode |
| Motion input speed cap | 0..1023 mm/s from existing SCAP word | 10 | MC_IN.cap_mms |
| Motion output duties | -1000..1000 permille each | 11 signed each | MC_OUT.duty_l_pm, duty_r_pm |
| Motion output directions | -1, 0, +1 each | 2 signed each | MC_OUT.dir_l, dir_r |
| Motion results | existing result schema per source | 13 each | UH result[4]; three zero tail bits per 16-bit local slot, widened when published to shared words |
| Sources with new results | four independent bits | 4 | MC_OUT.result_mask |
| Motion output fault masks | 27 defined independent bits each | 27 each | flt_set, flt_clr |
| Motion output navigation flags | Boolean each | 1 each | nav_started, nav_finished |
| PID measurement primed | Boolean | 1 | PID.primed; struct still has alignment padding |
| Freshness sequence / observation flag | 0..15 / Boolean | 4 / 1 | S2_FRESH.seq, seen |
| Freshness timestamp | unsigned modulo-2^32 milliseconds | 32 | S2_FRESH.time_ms; identical wrap-safe comparison |
| Speed-window count and cursor | 0..5 / 0..4 | 3 / 3 | SPD_WIN.n, i; samples and timestamps retain full precision |
| Pending task flags | Boolean each | 1 each | pend_nav_started, pend_nav_finished, pend_resume, enc_valid, btn_done, turn_log_align |
| Motion status state / source | four possibilities each | 2 / 2 | MOTION_STATUS.mstate, source |
| Motion status flags | Boolean each | 1 each | cmd_active, stall_l, stall_r, enc_ok |
| Raw motion-status speed fields | signed 16-bit getter/packer input, including invalid value | 16 each | retain H before existing 11-bit saturation/encoding |
| Motion-status odometer, heading and cruise inputs | raw unsigned 16-bit engineering values | 16 each | retain UH before existing saturation, normalization and scaling |

Full-resolution arithmetic and cumulative state are deliberately preserved:

- start_l_um, start_r_um, target_um, seek_from_um, seek_extra_um, advance_um,
  junction_edge_um, last_l_um, last_r_um, moved_l_um and moved_r_um keep signed
  32-bit micrometre values and the original calculations.
- tgt_l_mms, tgt_r_mms, cruise_mms, measured speed/distance inputs and task
  odometry retain the original signed/unsigned 32-bit domains. Transient sensor
  measurements are not assumed to stay inside a commanded speed limit.
- elapsed_ms, timeout_ms, stall_ms_l, stall_ms_r, fault_clear_ms, healthy_ms,
  vehicle elapsed time, task clocks and diagnostic counters retain 32-bit
  arithmetic. In particular, APIs accept arbitrary 32-bit elapsed increments;
  narrowing a timer before its threshold comparison could wrap and suppress a
  timeout. Normal fixed-cycle values are not a justification for changing this.
- PID kp_q8, ki_q8, kd_q8, i_acc, i_lim, prev_meas, out_min and out_max retain
  the signed range accepted by pid_init/update and the Q8 calculations. The
  generic PID API accepts limits beyond motor duty +/-1000: the new test uses
  a 100000 limit and verifies a 40000 output. Controller state is included in
  the requirement; the one-bit validity field is reduced without inventing a
  smaller numeric API or changing tuning.
- Packed steering and line-seen input words remain fixed word containers,
  not expanded Boolean fields. Kernel IDs, pointers and register values are
  infrastructure rather than bounded application enums.

The turn configuration has a compile-time guard: widening confirmation counts
past three or settling/junction lifetime past 255 ms requires revisiting the
local widths. Current configuration and thresholds are unchanged.

## Measured ARM storage

Measured with ARM GCC 15.2, Cortex-M0+, using the same normal build profile:
smp1-usb_cdc-wificyw43-netlwip-mqtt1-crash0-test0. The baseline already includes
Buddy 1's minimum-bit work. These savings therefore belong to this Buddy 2 stage.

| Item | Before bytes | After bytes |
|---|---:|---:|
| MOTION_CTRL | 252 | 184 |
| MC_IN | 52 | 28 |
| MC_OUT | 52 | 20 |
| VFSM | 20 | 8 |
| VFSM_IN | 36 | 20 |
| VFSM_OUT | 12 | 4 |
| MOTION_STATUS | 16 | 12 |
| PID | 36 | 36 |
| Freshness record, each | 12 | 8 |
| Speed window, each | 48 | 44 |
| Six task flags | 24 | 1 |
| Firmware .data | 128 | 128 |
| Firmware .bss | 85220 | 85068 |
| Firmware static RAM | 85348 | 85196 |
| Firmware flash text | 391526 | 393182 |

Static RAM falls by 152 bytes. Flash text rises by 1656 bytes. Local stack
records are smaller as well; those reductions must not be added to the .bss
saving because task stack reservations are unchanged. Reduced struct sizes can
also reduce a borrowing task's snapshot size without changing its values.

## Message comparison

| Message or publication | Logical bits | Before bytes | After bytes |
|---|---:|---:|---:|
| Motion command | 28 | 4 | 4 |
| Per-source shared motion result | 13 | 4 | 4 |
| Steering including reverse flag | 17 | 4 | 4 |
| Line-seen word | 8 | 4 | 4 |
| Obstacle speed cap | 15 | 4 | 4 |
| Motion status, two words | 60 | 8 | 8 |
| Vehicle status word | 18 | 4 | 4 |
| Shared fault/event bitmasks | 27 / 10 | 4 / 4 | 4 / 4 |

The existing packed messages were already compact. Their aligned publication,
queue depth, field positions and reserved bits are retained. MQTT frames and
Buddy 1's decoder/golden vectors are unchanged. No dashboard deployment or
configuration change is needed for this stage.

## Correctness evidence

- All 320 host checks pass with -Wall -Wextra -Werror. Seven new checks cover
  record sizes, freshness rollover/expiry, a full four-entry queue plus active
  command, queue overflow rejection, maximum epoch and flags, wide PID values,
  signed duties, complete fault masks, turn sentinel and retry saturation.
- The saved pre-change and current versions run the original full unit and
  cross-buddy suite in separate source trees. Instrumentation hashes controller
  state, active queue contents, PID state, every motor duty/direction, result
  and fault mask after each motion step and command submission, plus each
  vehicle-state step. The private no-sample sentinel is normalized from 255 to
  16 for this semantic comparison.
- Both produce hash 12586360152025161012 over 11918 motion steps, 78 command
  submissions and 7055 vehicle-state steps. All 366 test/trace output lines
  match. This covers left/right junctions, barcode instructions, avoidance,
  line reacquisition, stalls, encoder faults, command priority, STOP/RESUME,
  startup readiness and network loss in the simulated tests.
- Another 200000 freshness samples match the original helper, including
  timestamp/sequence rollover: hash 15494051068731991138.
- All 43 backend tests and 14 standalone codec checks pass.
- All 77 files in Buddies 1, 3, 4 and 5 match the comparison snapshot. The
  shared API functions, PID/FSM algorithm sources, calibration, encoder ISR,
  motor driver and GPIO dispatcher are unchanged.
- Normal and self-test ARM builds pass without compiler warnings. The self-test
  UF2 contains the new tests, but they have not been run on the Pico here.

## Hardware performance remains to be checked

Software comparisons establish matching outputs on the tested inputs, not
identical runtime. Bit-field accesses add instructions. Do not claim measured
20 ms timing, watchdog margins or course accuracy from host tests. Run the
existing TASK_STAT instrumentation on the pre-change and new firmware under
identical conditions; compare maximum execution time, periods and overruns.
Then supervise normal line following, left/right/U-turn instructions, obstacle
recovery and safety stopping. This change does not alter calibration, but a
physical test is still required before claiming demonstration readiness.
