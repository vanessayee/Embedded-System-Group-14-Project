# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-10a/b | Buddy 5 profile → plan → packed AVOID commands → Buddy 2; line search stops on the line | IF-M1, IF-M3, IF-S1 | auto, host + target | `subsystem2/s2_integration_tests.c` | `[TEST] INT-10a/b PASS` |
| INT-14a–d | Buddy 3 line loss/found events during Buddy 5 avoidance; hand-back to NORMAL or Safe Stop | IF-L1, IF-L2, IF-S1 | auto, host + target | `subsystem2/s2_integration_tests.c` | `[TEST] INT-14a-d PASS` |
| INT-18a–c | Critical safety stop overrides Buddy 5 completion and all motion commands | IF-M3, IF-S2 | auto, host + target | `subsystem2/s2_integration_tests.c` | `[TEST] INT-18a-c PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-18 | INT-15a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-15b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-16a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-16b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-16c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-16d | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-16e | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-10a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-10b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-10c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-10d | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-11a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-11b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-11c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-12a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-12b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-13a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-13b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-13c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-13d | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-13e | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-14a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-14b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-14c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-14d | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-17a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-17b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-18a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-18b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-18c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-19a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-19b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-20a | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-20b | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-20c | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| 2026-11-18 | INT-20d | 42f79b1 (main; tested 47ad264) | Vanessa | PASS | `logs/2026-11-18_host_all_buddies.txt` |  |
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
