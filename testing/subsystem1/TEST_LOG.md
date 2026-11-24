# Subsystem 1 — test log

**Subsystem:** WiFi / MQTT communication, commands and telemetry (Buddy 1)  ·  **Student ID:** 2501573  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | CRC-16/CCITT-FALSE: catalogue check value 0x29B1; empty input 0xFFFF |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | LSB-first bit order (known bytes FD BC DA), signed fields, overflow/underflow flagged |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | heartbeat = golden bytes 12 2A 04 34 12 EC 3D 60 E5; decodes; reconnects saturate at 15 |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U03 PASS` |
| S1-U27 | server codec: golden frames decode and re-encode byte for byte; fault severity, IR sides, field ranges | FR14, Tribunal 1 R1 | auto | `node telemetry/lib/codec.js` | 14 × `PASS` |
| S1-U28 | telemetry backend unit tests: codec, ingest (sessions, wrap, gaps, saturation, link states), store (outage, overflow), generated flow and dashboard | FR6, FR13, FR14, NFR11 | auto | `telemetry/bin/csc14 test` | `pass 34`, `fail 0` |
| INT-16e | a failed avoidance (counter unchanged) sends one OBSTACLE event with its reason | FR12, FR7 | auto | `make -C testing/host SUITES="s1 int" run` | `[TEST] INT-16e PASS` |
| INT-21 | telemetry pipeline on isolated services: real Node-RED, PostgreSQL, Grafana (routing, rejection, runs, restart, reboot, stale/offline, database outage, overflow, ACK, commands, every panel query) | FR13, FR14, NFR9–NFR11 | auto | `telemetry/bin/csc14 test integration` | every subtest ✔ |
| S1-U29 | telemetry backend measurements: latency, CPU/RAM, storage, outages | NFR3, A1 | auto | `telemetry/bin/csc14 bench` | report written; nothing lost in a 30 s database outage |
| S1-U30 | live delivery: Pico → broker → Node-RED → PostgreSQL → dashboard | FR7, FR13, FR14 | manual | `telemetry/docs/FIRMWARE.md` §5 | link ONLINE, 2 telemetry/s, real IR sides, outage shown as a gap |
| S1-U31 | control-task timing (TASK_STAT) with telemetry running, broker down, Wi-Fi down | NFR9, A1 | manual | `telemetry/docs/FIRMWARE.md` §6 (measurement image) | no extra overruns; periods unchanged |
| S1-U04 | command frame = golden bytes; decoded and validated (SET_SPEED 250, id 0x5A) |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U04 PASS` |
| S1-U05 | acknowledgement and event frames = golden bytes; both decode |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U05 PASS` |
| S1-U06 | telemetry: 279 bits -> 40-byte frame = golden frame; decodes field by field |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U06 PASS` |
| S1-U07 | telemetry round trip with every field at its maximum and at its minimum |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U07 PASS` |
| S1-U08 | rejected: bad CRC, truncated, version, unknown type, length byte, wrong type, schema size |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U08 PASS` |
| S1-U09, S1-U09m | S1-U09: command fields: reserved bits and invalid map args rejected; SET_SPEED 50..400 only; m: all A-Z runtime map actions and query commands validate; invalid actions rejected |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | each `[TEST]` check PASS |
| S1-U10 | decoder errors map to distinct acknowledgement codes |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U10 PASS` |
| S1-U11 | commands map to supervisory requests only; Buddy 2's refusal is acknowledged with its reason |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U11 PASS` |
| S1-U12 | every encoder refuses a buffer one byte too small (no overrun) |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U12 PASS` |
| S1-U13 | binary telemetry frame (40 B) at least 10x smaller than the JSON text form |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U13 PASS` |
| S1-U14 | no network stack in the image: DISABLED without faults; starts once configured |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U14 PASS` |
| S1-U15 | connect sequence WIFI -> MQTT -> SUBSCRIBE -> ONLINE |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U15 PASS` |
| S1-U16 | WiFi lost while online: WIFI_LOSS (degraded) and RECOVERING reported |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U16 PASS` |
| S1-U17 | retries wait 1, 2, 4, 8, 16, 16 s (capped, non-blocking); every 3rd restarts the client |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U17 PASS` |
| S1-U18 | automatic reconnection: ONLINE again, counted, back-off reset, faults cleared |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U18 PASS` |
| S1-U19 | broker lost with WiFi up: MQTT_LOSS only; each wait for the broker bounded to 10 s |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U19 PASS` |
| S1-U20 | communication status word: 17 bits, counters saturate at 15 |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U20 PASS` |
| S1-U21 | message buffers, event flag and both cyclic handlers exist and run |  | auto (target only) | `subsystem1/subsystem1_test.c` | `[TEST] S1-U21 PASS` |
| S1-U22 | telemetry task: one frame per 500 ms, never over its 50 ms budget |  | auto (target only) | `subsystem1/subsystem1_test.c` | `[TEST] S1-U22 PASS` |
| S1-U23 | connection state machine running (DISABLED without NET=lwip MQTT=1, no false fault) |  | auto (target only) | `subsystem1/subsystem1_test.c` | `[TEST] S1-U23 PASS` |
| S1-U24 | MQTT hand-over: wrong topic and empty frame refused; PING answered with ACK OK |  | auto (target only) | `subsystem1/subsystem1_test.c` | `[TEST] S1-U24 PASS` |
| S1-U25 | SET_SPEED reaches Buddy 2 and is acknowledged; a corrupted frame is counted, flagged, never run |  | auto (target only) | `subsystem1/subsystem1_test.c` | `[TEST] S1-U25 PASS` |
| S1-U26 | bounded queue full: the OLDEST frame is dropped and counted (overflow policy) |  | auto (target only) | `subsystem1/subsystem1_test.c` | `[TEST] S1-U26 PASS` |
| S1-U32 | new rate window admits commands after resetting the counter |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U32 PASS` |
| S1-U33 | 255 attempts and 16000 ms backoff retain the original recovery cadence |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U33 PASS` |
| S1-U34 | packed reconnect counter saturates without wrapping |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U34 PASS` |
| S1-U35 | v1 compatibility bit cannot convert an invalid argument into a valid command |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U35 PASS` |

> **ID clash to resolve.** S1-U30 and S1-U31 above were defined on 2026-10-30 as manual tests. On 2026-11-22 two automated checks in `app_program/subsystem1/s1_unit_tests.c` were given the same IDs ("task-owned records use minimum fields and only byte-tail padding" and "command flood never wraps the packed rate counter or bypasses the limit"). Until the code checks are renumbered, a `[TEST] S1-U30` or `[TEST] S1-U31` line in a console log refers to those automated checks, not to the manual tests in this table.

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-18 | S1-U01 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U02 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U03 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U04 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U05 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U06 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U07 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U08 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U09 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U10 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U11 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U12 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U13 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U14 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U15 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U16 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U17 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U18 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U19 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U20 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U21 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U22 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U25 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U26 | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_subsystem1_host.txt` | Host all-buddies run. |
| 2026-11-18 | S1-U21 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem1_target.txt` | Pico target run. |
| 2026-11-18 | S1-U22 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem1_target.txt` | Pico target run. |
| 2026-11-18 | S1-U25 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem1_target.txt` | Pico target run. |
| 2026-11-18 | S1-U26 | 42f79b1 (main; tested 4eef391) | Vanessa | PASS | `logs/2026-11-18_subsystem1_target.txt` | Pico target run. |
| 2026-10-30 | S1-U27 | `d81f8b9` | Claude (for Buddy 1) | PASS (7 checks, before the changes) | `logs/2026-10-30_before_d81f8b9_decoder_and_host_tests.txt` | baseline |
| 2026-10-30 | S1-U27 | `d81f8b9` + uncommitted | Claude (for Buddy 1) | PASS (14 checks) | `logs/2026-10-30_after_decoder_and_host_tests.txt` | changes not committed yet |
| 2026-10-30 | INT-16e | `d81f8b9` + uncommitted | Claude (for Buddy 1) | PASS; all host suites 0 failed | `logs/2026-10-30_after_decoder_and_host_tests.txt` | |
| 2026-10-30 | S1-U28 | `d81f8b9` + uncommitted | Claude (for Buddy 1) | PASS (34/34) | `logs/2026-10-30_backend_unit_tests.txt` | |
| 2026-10-30 | INT-21 | `d81f8b9` + uncommitted | Claude (for Buddy 1) | PASS (12/12 subtests) | `logs/2026-10-30_backend_integration_tests.txt` | simulated frames, isolated broker and database |
| 2026-10-30 | S1-U29 | `d81f8b9` + uncommitted | Claude (for Buddy 1) | measured | `logs/2026-10-30_telemetry_backend_bench.md`, `logs/2026-10-30_dashboard_simulated_demo.png` | Mac, simulated frames; see telemetry/docs/MEASUREMENTS.md |
