# Stationary width calibration

Build: `make -j8 NET=lwip MQTT=1 TEST=0 S5_SCAN_ONLY=1` from `build_make`.
The output filename ends in `_scan_only.uf2`. Ordinary builds default to
`S5_SCAN_ONLY=0`; their driving behavior is unchanged.

This image starts disarmed and its motor driver always writes zero output.
Dashboard/resume commands cannot enable motor output in this image. It uses
the normal scan task, three-ping range filter, profile task and width estimator.
The calibration loop does not submit movement commands or avoidance events.
It runs independently of vehicle readiness and WiFi/MQTT connection.

1. Flash the `_scan_only.uf2` using BOOTSEL.
2. Open the USB serial monitor, 115200 baud. Keep the robot stationary.
3. Centre the 10 cm object, with its face square to the sensor, 20 cm from
   the ultrasonic sensor's front face. Keep other objects out of the sweep.
4. The first sweep starts after 10 seconds. Each sweep goes from -75 to
   +75 degrees in 3-degree steps. Do not move the object during the sweep.
5. Save the `calib: BEGIN`, all `profile: ... rays` lines, `profile: scan`
   summary and `calib: END` messages. Status 0 is a valid profile; other
   statuses and timeouts must not be treated as successful measurements.
6. Leave the object in position for five scans (five-second pauses between
   them). At `calib: GROUP COMPLETE`, there are 20 seconds to replace it.
7. Use the 13 cm object for group 2 and the 15 cm object for group 3. Record
   which object belongs to each group. Groups repeat until power-off/reset.

The complete-sweep profile line contains the raw millimetre estimate. The
`calib: END` summary is unpacked from the normal profile message, so width is
quantized to 5 mm and edge/clearance positions to 10 mm. Calibration does not
change the estimator or introduce fixed 10/13/15 cm size recognition.

Restore the normal driving UF2 to resume movement. A physical reset alone
restarts this scan-only firmware; it does not restore normal driving.

The serial capture, not the dashboard, is the calibration record: the scan
runs without motion and may not follow normal avoidance telemetry events.
