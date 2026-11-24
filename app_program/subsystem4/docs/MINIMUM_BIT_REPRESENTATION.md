# Buddy 4: minimum-bit representation

Implemented for Change Requirement 1 on 2026-11-21. This change reduces storage for task-owned flags, states, bounded indexes and local snapshots. Sensor-processing algorithms, calibration, sensor thresholds, motor policy, sample timing and explicit shared-message encodings are preserved.

## Storage on RP2040

Measured with the configured ARM compiler (Cortex-M0+, ARM EABI), compared with the repository immediately before this Buddy 4 change.

| Type | Before (bytes) | After (bytes) |
| --- | ---: | ---: |
| IMU_IN | 52 | 40 |
| IMU_CAL_ACC | 76 | 76 |
| IMU_CAL | 28 | 28 |
| IMU_PROC | 252 | 220 |
| HUMP | 36 | 32 |
| HUMP_EVT | 16 | 12 |
| S4_IMU_STATUS | 20 | 16 |
| S4_HUMP_STATUS | 7 | 4 |
| S4_HUMP_SAMPLE | 6 | 4 |

Normal MQTT firmware: text 394902 -> 395686 bytes (+784); data 128 -> 128; BSS 84832 -> 84788 (-44). Static RAM (data + BSS) falls from 84960 to 84916 bytes. Smaller task-local objects also reduce their object footprint; configured task stacks are not reduced and stack high-water usage was not measured. Bit operations cost some flash, so this is a storage/representation change, not a claimed execution-speed improvement.

## Field bounds and retained precision

- Boolean flags: 1 bit. IMU state, chip family, motion class and hump state: 2 bits.
- Magnetic history index and populated count: 5 bits each, including the full 20-sample window. A compile-time check rejects configurations exceeding 31.
- Collision refractory countdown: 9 bits for the existing 500 ms range; the original countdown calculation is unchanged. A compile-time check rejects values exceeding 511 ms.
- Sensor raw hash: 16 bits. Stuck-sample and invalid-run counters: 8 bits each because they still saturate at 255, even though fault thresholds are lower.
- Hump rising/level confirmation counters: 2 bits each. A compile-time check protects the level-confirmation bound. Hump count remains 8 bits, saturating at 255; publication still saturates it to 15.
- Hump status height codes: 6 bits each; flags: 1 bit; centimetre length: 8 bits. Status count intentionally stays 8 bits before the packer's saturation.
- Queue sample local storage retains signed 16-bit tilt before the existing 11-bit saturating packer, 8-bit distance, 1-bit validity and 4-bit sequence.
- IMU task flags/state/4-bit hump sequence share task-owned storage; sample sequence remains 8 bits. Calibration epoch and capped recovery-attempt counter use bytes. Probe output uses an addressable temporary chip byte before updating the task field.
- Calibration sums/count, raw/calibrated sensor vectors, angle/filter/history arithmetic, elapsed timers, sample totals and micrometre hump heights remain full resolution. Additive timers are not narrowed just because a comparison threshold is small. Highest peak is still retained in micrometres.

Local bitfields use natural alignment, with gcc_struct on MinGW so host layout matches the ARM checks. No C structure layout is sent over MQTT or copied into the inter-task queue as a message. Existing mask/shift packers remain authoritative.

## Integration and unchanged behaviour

The two-word IMU snapshot plus publication timestamp, sequence-lock synchronization, one-word hump snapshot, 32-bit IMU-to-hump queue messages and queue capacity remain unchanged. Sensor drivers, processing and packing implementation files are byte-identical to the saved baseline. The IMU task retains collision/handling latching, fault events and operator-controlled clearing. Hump handling retains invalid-sample/queue-gap rejection, sideways-climb measurement and highest-peak reporting. Startup readiness and links to Buddies 1, 2, 3 and 5 are unchanged.

## Verification

- All 333 host checks pass, compiled with -O2 -Wall -Wextra -Werror (existing unused-parameter exception).
- Six added checks cover ARM-compatible structure sizes, 255-counter saturation, collision countdown, the complete magnetic window and micrometre peak/count preservation.
- A deterministic before/after trace of 200000 IMU samples and 400000 hump steps produces the identical hash 6534271118668648874. It covers gyro/no-gyro, magnetic validity, invalid/saturated inputs, handling, collision, both hump modes and packed hump reports.
- Complete original 327-check suite compared against saved before-change Buddy 4, with the same test source on both sides; see the verification log for comparison results.
- Backend tests: 43 pass. Binary codec checks: 14 pass, including golden frames and invalid codes.
- Normal and TEST=1 MQTT ARM firmware builds succeed. Friendly UF2 copies are refreshed in build_make.
- An existing test's HUMP_EVT last fixture is now zero-initialized to avoid an optimizer warning; this does not alter firmware operation.

These are software and build checks. The firmware has not been flashed during this change. Physical sensor calibration, timing and course behaviour still need a supervised hardware run; software checks alone cannot certify them.

## Update 25 November 2026 (RESUME clears hump history)

The hump task now starts a new height record when the vehicle leaves Safe Stop (only a
validated RESUME can do that): `hump_init()` clears last height and highest
height as at boot; the 8-bit count is restored so it keeps numbering the
boot's humps (the server stores one row per boot and count). The only added
state is one bit,
`was_safe_stop`, in the task's existing `sequence` bit-field (expect 4 bits,
first 1, was_active 1, was_safe_stop 1 = 7 bits in one byte). The queue wait
is now bounded to 100 ms (`HUMP_MODE_POLL_MS`) so the transition is seen
even when no IMU samples arrive. Shared words, queue messages and packers are
unchanged. S4-U69 covers the cleared state; the RESUME transition itself is a
robot check (INT-24).
