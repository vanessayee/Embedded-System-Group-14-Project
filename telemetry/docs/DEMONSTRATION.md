# Group 14 demonstration console

Open **http://127.0.0.1:3000/d/csc14-robot?theme=light&kiosk** for the
mission console. This Grafana dashboard has a single panel that embeds the
live console page served by Node-RED at
**http://127.0.0.1:1880/csc14/dashboard** (which also works on its own).
Grafana no longer has separate historical charts; past data stays in
PostgreSQL.

## Starting on this laptop

PostgreSQL (local port 55432), Mosquitto (1883) and Grafana (3000) use the
existing installation in Documents/Codex/csc14-local. Enable the laptop's
2.4 GHz hotspot and connect the Pico using the configured credentials.
Flash firmware built with `make NET=lwip MQTT=1 -j8` (repository
`README.md` §2.3) once to enable remote mapping. `config/wifi_secrets.h`
must hold this hotspot's name and password and this laptop's address on it.

From the repository root in PowerShell:

    .\telemetry\tools\start-windows-dashboard.ps1

Use `-Restart` after **any** change under `telemetry/` (backend `lib/`, flows,
or the dashboard page `dashboard/index.html` / `dashboard.js`): Node-RED reads
all of them once at start-up, so a running service keeps serving the old
version until restarted. Then hard-refresh the browser (Ctrl+F5). Without
`-Restart` the script only reports that the service is already running.
This starts Node-RED in the
background using the existing local dependencies and credentials. It does
not start or move the robot. Logs are in csc14-local/group14/nodered.log.
The script expects the existing PostgreSQL and Mosquitto services to run.

## Changing the professor's barcode instructions

1. Wait for ROBOT ONLINE and recent telemetry. The robot may be stopped or
   driving; rules can be applied in any mode.
2. Choose the A–Z letter for LEFT, RIGHT, STRAIGHT and U-TURN. Use the blank
   choice to disable an instruction. Each selected letter must be different.
3. Press **Apply all barcode rules**. It sends all 26 assignments and waits
   for robot acknowledgements. Unselected letters are unmapped. They can
   still decode but do not request navigation.
4. Check **Confirmed by robot**, or press **Read robot rules** to query all
   26 entries from the Pico. A saved choice is not proof the robot applied it.
5. If the robot was stopped, place it safely on the course, clear faults if
   required and press **Resume**. Resume can start movement after
   initialization.

While a barcode is being decoded or its instruction is executing, the robot
refuses a map change (ACK REJ_STATE) so that instruction keeps its meaning;
the backend waits and retries that letter every 0.5 s for up to 15 s. Each
letter changes atomically, but the 26 letters are sent one by one (about
8–10 s in total, paced below the firmware's five-per-second limit), so a
barcode read during the transfer uses the old or the new rule for its
letter. Apply on a stretch with no barcode ahead for a clean switch.
**■ Stop** remains available during configuration; Resume/Clear are held
until the transfer ends.

The laptop saves desired rules in its runtime barcode-rules.json. The Pico
keeps the active table in RAM. After powering the Pico off/on it starts with
the firmware default table (A=RIGHT, W=LEFT, Z=LEFT, X=STRAIGHT, Y=U-TURN);
press **Apply all barcode rules** again to restore the saved choices. No
firmware rebuild is needed to change letters. The new map applies to
subsequent barcode scans.

**■ Stop** halts everything: motors, any obstacle sweep (the servo
centres) and obstacle avoidance, without adding an avoidance fault. One
**Resume** continues; it also clears the last and highest hump height
as a reboot does (the hump count keeps running within the boot so the database
keeps one row per hump).

## Ten demonstration capabilities

| Capability | Dashboard evidence | Physical test |
|---|---|---|
| Detect barcode | Capture state/read counters | Drive GP26 across the barcode |
| Decode code | Letter or INVALID, scan events | Check known letters in both directions |
| Hump height | Last height, highest peak, count | Drive one wheel over a measured hump, opposite wheel on flat road |
| Follow line | Centre/right states, steering, wheel chart | Track black between GP6 and GP2 |
| Follow barcode instruction | Mapped action and NAV_ACTION-to-NORMAL event | Scan each assigned letter, reach the junction |
| Detect obstacle | Ultrasonic range and avoidance state | Place an obstacle ahead |
| Measure width | Measured width in cm, profile history | Compare against an object of measured width |
| Avoid/rejoin | Side, recovery state and SUCCESS-to-NORMAL evidence | Observe the full bypass and line handoff |
| MQTT telemetry | Online/age, measurements, frame/evidence history | Connect over hotspot; check updates/outages |
| Handling/collision stop | Latched stop, shock, pitch/roll, SAFE_STOP and zero speeds | Carefully test handling, tilt and stop |

Evidence is marked OBSERVED only from received data for the current robot
boot. It is not an automatic grade or a substitute for the physical test.
Offline data is explicitly marked stale. The activity feed also contains
older backend/link events and is not restricted to one boot.

## Sideways hump height

The terrain configuration now measures **sideways one-wheel humps** from
roll: estimated height = 110 mm physical wheel-centre spacing × sin(abs(peak
roll)). It works on either side and publishes after returning within 1.5
degrees of level for three samples. It does not need negative forward pitch.
Forward travel is required to start. Side roll must reach 3 degrees for two
samples (about 5.8 mm height difference with this geometry); the retained
3 mm height rejection floor is an additional noise check.

Power on with both wheels on level ground for IMU calibration. The
opposite wheel must remain on the original flat road for this height estimate
to represent the hump's height. Wheel spacing, mounting, tyre compression
and acceleration affect accuracy; compare with a known measured hump.
The effective 130 mm encoder-turn calibration is deliberately not used.
Handling/collision, missing samples and severe tilt discard a terrain
candidate. HUMP_SIDEWAYS=0 restores the legacy forward-pitch method when
building for a course where both wheels cross together.

## Accelerometer-only pickup limitation

The existing LSM303 has no ground-contact or height sensor. A gentle, level
lift can remain indistinguishable from stationary 1g and is **not guaranteed
to stop**. The implemented handling stop detects:

- Departure from 1g greater than 400 mg for at least 80 ms.
- Low-g/free-fall below 350 mg for at least 60 ms.
- Filtered pitch or roll of at least 60 degrees for at least 200 ms.
- The existing horizontal collision threshold of 1500 mg.

Ordinary 20-degree hump tilt does not trigger the severe-tilt rule. Actual
course vibration and handling must still be tested. Handling uses the
existing latched collision stop/event and dashboard label; the current
wire protocol does not separately identify pickup versus collision.
Place the robot down upright before clearing and resuming. A ground-contact
sensor is needed if the assessment requires reliable gentle-level pickup
detection under every handling condition.

## Verification

Firmware tests cover map command ranges, all 26 runtime entries, handling
thresholds, hump processing, turn geometry, safety arbitration and telemetry.
Backend tests cover all letter/action encodings, matching robot ACKs,
rejected/missing ACKs, configuration guards and same-origin HTTP control.
The new console was checked against the local database and in the browser.
No live motion commands were issued during implementation verification.

### Startup and physical RUN reset

The normal firmware holds both motors in INIT until all five buddies are ready:
MQTT ONLINE (Wi-Fi/IP and command subscription) plus the telemetry task; motion
and encoder snapshots; calibrated line sensors and barcode capture; calibrated,
fresh IMU plus hump task; and servo initialization, fresh forward ultrasonic
result, profile and avoidance tasks. A valid no-echo result is accepted as clear
space; a silent/missing sensor is not. This does not mechanically test the motors
or servo. Serial startup messages show each buddy's 0/1 readiness.

Local readiness/calibration has an 8-second limit. Waiting for hotspot/MQTT has
no timeout and motors remain off. Once every check passes, movement starts
automatically. A later network outage keeps the existing autonomous operation.

Pressing the Pico physical RUN/reset button restarts the firmware. Its new boot
identity resets the mission console's current measurements, ten evidence flags,
event list, chart and confirmed barcode rules after MQTT reconnects. Saved laptop
barcode choices remain available to reapply (in any mode). Historical database
records remain in PostgreSQL; the console's current values show only the
latest boot. The boot identity is repeated every five seconds to
recover from a dropped MQTT event; reconnecting alone does not reset the session.

## Live activation — 19 November 2026

Main entry point: http://127.0.0.1:3000/d/csc14-robot?theme=light&kiosk
The iframe now loads the actual /csc14/dashboard route with JavaScript enabled,
not the sample preview. All 43 backend tests passed. The real snapshot endpoint
and initialized A-Z selectors were verified without sending motor commands.
The Pico was offline during activation, correctly shown as stale/offline.

The active provisioning folder and Grafana search contain only csc14-robot.
The former dashboards, preview generators and cleanup backups have been removed
at the user's request. The single live dashboard JSON is the source of truth.
No PostgreSQL robot telemetry was deleted. The HTML files remain the page source.
