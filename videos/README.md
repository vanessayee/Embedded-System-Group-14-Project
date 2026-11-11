# Test and demo videos

Use this page to index short videos of the five Buddy subsystems running on
real hardware. Each video should provide evidence for one completed test or
clearly identify itself as a demo. A video does not replace the test result
and console log in `testing/`.

## Recording checklist

- Keep each clip **15–30 seconds** and focused on one test or feature.
- Show or say the test ID at the beginning (for example, `S3-M01`).
- Show the hardware response and the serial output when it helps verify the
  result. Keep the board, sensor or vehicle visible where practical.
- Include the tested commit in the table. If practical, briefly show
  `git rev-parse --short HEAD` in the recording too.
- Record in landscape with steady framing and enough light to see the result.
- Record only after the test has been run. Do not label a planned test as a
  pass.

The `TEST=1` firmware prints automated checks for several subsystems in one
run. Frame the serial console on the relevant `[TEST]` lines and show the
hardware during the same clip. The test image may keep outputs safely
disarmed; use a separate, controlled manual test when a video needs to show
actual motion or sensor response. Follow the test steps and expected results
in the relevant `testing/subsystemN/TEST_LOG.md`.

## 15–30 second video ideas for each Buddy

These are filming guides, not claims that the tests have passed. Use the
matching test ID in the subsystem test log, and add a manual test entry there
first if the behavior is not already listed.

| Buddy | What the clip should show | Suggested 15–30 second shot |
|---|---|---|
| **1 — WiFi/MQTT communication** | The board's WiFi/MQTT connection or telemetry and supervisory command handling | Start with the S1 test ID and commit visible; show the serial/network status or MQTT client receiving/publishing the test message, then show the resulting telemetry/status. Use the test procedure and ID recorded in `testing/subsystem1/TEST_LOG.md`. |
| **2 — Motion control** | Motor/encoder response and the vehicle mode or command result | Start with the S2 test ID and commit; show the serial result and encoder/motion status. If demonstrating actual motor movement, use a controlled manual test listed in `testing/subsystem2/TEST_LOG.md`, secure the vehicle appropriately, and show the command and resulting motion. The automated test image keeps motors disarmed. |
| **3 — Line following and barcode navigation** | The line sensor/vehicle following the track, reading a mapped barcode, and acting at the next junction | Start with `S3-M01`; show the vehicle on the line, the barcode pass, and the mapped turn at the next junction. Include the barcode/command serial output if visible. The `TEST=1` logic simulation alone does not demonstrate physical barcode reading. |
| **4 — IMU, collision and terrain monitoring** | A sensor reading or status change in response to a controlled movement/tilt or documented test stimulus | Start with the S4 test ID and commit; show the serial monitor/status, apply the stimulus from the subsystem test procedure, then show the resulting reading or event. Keep the device stationary otherwise. Use an existing S4 test ID or add the manual check to `testing/subsystem4/TEST_LOG.md` before recording. |
| **5 — Ultrasonic scan and obstacle handling** | Servo scan, ultrasonic readings/profile, or a documented obstacle response | Start with an S5 test ID and commit; show the sensor/servo and serial status while an object is placed at a measured distance, then show the reported range/result. For an avoidance maneuver, use its logged test procedure and keep the vehicle area clear. The S5 test log specifies which checks work with the sensor present or absent. |

For automated tests, capture the `[TEST] <ID> PASS` line and the final subsystem
summary if it fits. For a manual test, show enough of the stimulus and response
to make the result understandable. If a 15–30 second clip cannot show the
whole procedure, include a brief evidence note in the test log.

## Share and record

1. Upload to YouTube as **Unlisted** so teammates and the instructor can use
   the link. Do not use Private or Public visibility.
2. Do not commit video files to this repository or attach them to GitHub; video
   files are ignored because they permanently enlarge repository history.
3. Add the video link to the index below and to the *Log / evidence* column of
   the corresponding test result in `testing/`.
4. Record the date, exact tested commit, Buddy, test ID, and any relevant
   conditions in both records.

## Video index

Add new rows at the bottom. Keep previous entries.

| Date | Subsystem | Test ID / demo | Commit | Student ID | Link | Notes |
|---|---|---|---|---|---|---|
