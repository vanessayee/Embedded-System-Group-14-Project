# CSC14 — Autonomous Robotic Car

> **Project architecture: Draft 7** · Team onboarding and implementation guide
>
> **Status:** Planned repository structure. Files shown below are proposed; they are not assumed to be implemented. Update this README as the project progresses.

## 1. Project overview

CSC14 is building an autonomous robotic car using a **Raspberry Pi Pico W on the Cytron Robo Pico platform**. The robot must follow a black line, decode navigation barcodes, detect and estimate hump heights, scan and avoid obstacles, reacquire the line, and report telemetry over Wi-Fi. It must continue its local navigation and safety functions even when network communication is unavailable.

The system is divided into **five buddy-owned subsystems**, integrated as **one firmware application**. The team's Week 6 Design Review proposes **10 RTOS tasks**, common command/event mechanisms, and motor actuation exclusively owned by Buddy 2.

### Technology choices

| Area | Technology | Runs on |
|---|---|---|
| Main robot firmware | **C11** and Raspberry Pi **Pico C/C++ SDK** (C APIs) | Pico W |
| Task scheduling and inter-task communication | **micro T-Kernel RTOS** | Pico W |
| Firmware build | **CMake**, ARM embedded toolchain | Development computer |
| Wi-Fi/MQTT client | C | Pico W |
| Basic telemetry viewer (Draft 7) | Python 3, `paho-mqtt` | Laptop/computer |
| MQTT broker | Mosquitto or compatible MQTT broker (installed separately) | Laptop/computer or other host |
| Collaboration | Git and GitHub | Team computers |

**Coding requirement:** Follow the **Barr C Coding Standard** as specified in the Week 6 Design Review.

**Scope note:** Draft 7's lightweight Python telemetry receiver is *not* the complete PostgreSQL + historical-monitoring solution described by FR14 in the Week 6 Design Review. The team must either implement the full agreed telemetry architecture or formally revise the relevant requirement and design artefacts. Do not mark FR14 complete based only on console MQTT output.

## 2. Repository layout (Draft 7)

```text
Embedded-System-Group-14-Project/
│
├── README.md
├── .gitignore
│
├── firmware/
│   ├── CMakeLists.txt
│   ├── pico_sdk_import.cmake
│   │
│   ├── src/
│   │   ├── main.c
│   │   │
│   │   ├── buddy1_communication/
│   │   │   ├── wifi.c
│   │   │   ├── mqtt.c
│   │   │   ├── telemetry.c
│   │   │   ├── communication_task.c
│   │   │   └── telemetry_task.c
│   │   │
│   │   ├── buddy2_motion/
│   │   │   ├── motor.c
│   │   │   ├── encoder.c
│   │   │   ├── pid.c
│   │   │   └── motion_task.c
│   │   │
│   │   ├── buddy3_navigation/
│   │   │   ├── ir_sensor.c
│   │   │   ├── line_following.c
│   │   │   ├── barcode.c
│   │   │   ├── line_task.c
│   │   │   └── barcode_task.c
│   │   │
│   │   ├── buddy4_imu/
│   │   │   ├── imu.c
│   │   │   ├── imu_filter.c
│   │   │   ├── hump_detection.c
│   │   │   ├── imu_task.c
│   │   │   └── terrain_task.c
│   │   │
│   │   ├── buddy5_obstacle/
│   │   │   ├── ultrasonic.c
│   │   │   ├── servo.c
│   │   │   ├── obstacle_scan.c
│   │   │   ├── obstacle_profile.c
│   │   │   ├── avoidance.c
│   │   │   ├── line_recovery.c
│   │   │   ├── obstacle_scan_task.c
│   │   │   ├── obstacle_profile_task.c
│   │   │   └── avoidance_task.c
│   │   │
│   │   ├── shared/
│   │   │   ├── robot_state.c
│   │   │   ├── command_router.c
│   │   │   └── safety_manager.c
│   │   │
│   │   └── rtos/
│   │       ├── task_manager.c
│   │       ├── queue_manager.c
│   │       └── event_manager.c
│   │
│   ├── include/
│   │   ├── buddy1.h
│   │   ├── buddy2.h
│   │   ├── buddy3.h
│   │   ├── buddy4.h
│   │   ├── buddy5.h
│   │   ├── robot_config.h
│   │   ├── pin_config.h
│   │   ├── robot_types.h
│   │   ├── robot_state.h
│   │   ├── motion_commands.h
│   │   └── rtos_config.h
│   │
│   └── build/                  # Generated; Git ignored
│
├── telemetry/
│   ├── README.md
│   ├── mqtt_receiver.py
│   ├── mqtt_config.example.json
│   └── requirements.txt
│
├── tests/
│   ├── buddy1/
│   ├── buddy2/
│   ├── buddy3/
│   ├── buddy4/
│   ├── buddy5/
│   └── integration/
│
├── docs/
│   ├── design_review/
│   ├── diagrams/
│   └── testing/
│
└── .github/
    └── workflows/
        └── firmware-build.yml
```

**Note:** Git does not track empty directories. Add content or a `.gitkeep` file when creating the skeleton. `firmware/build/` is generated locally and must not be committed.

## 3. What belongs where?

| Path | Purpose |
|---|---|
| `firmware/src/main.c` | Firmware entry point: initialise the platform and start the application's RTOS coordination. |
| `firmware/src/buddyN_.../` | The C implementation owned by each buddy. |
| `firmware/include/` | Public headers, shared types, state definitions, pin mappings, and RTOS configuration. |
| `firmware/src/shared/` | System-wide state handling, command arbitration, and safety coordination. |
| `firmware/src/rtos/` | Application-level task creation, queues and events—not necessarily the micro T-Kernel kernel itself. |
| `firmware/CMakeLists.txt` | Defines source files, include directories, linked SDK/kernel libraries and firmware outputs. |
| `firmware/pico_sdk_import.cmake` | Pico SDK import helper, copied from a compatible official Pico SDK distribution. |
| `telemetry/` | Optional laptop-side MQTT receiver and local configuration; not compiled into Pico W firmware. |
| `tests/` | Buddy-specific and cross-subsystem test code/results. |
| `docs/` | Design review material, diagrams, test plans and evidence. |
| `.github/workflows/` | Continuous integration workflows, once implemented. |

### `src/` versus `include/`

- **`.c` in `src/`:** function implementations and task logic.
- **`.h` in `include/`:** declarations and types needed by other modules. We intentionally start with a **small public header set**, rather than making one header for every `.c` file.
- Use `#include "buddy2.h"` (for example) to access the agreed public interface; avoid reaching into another buddy's private implementation.
- Local helper functions used only in one source file should usually be `static`. Introduce an internal header later only when multiple `.c` files genuinely need shared private declarations.

## 4. Buddy responsibilities and task ownership

| Buddy | Source folder | Primary responsibility | RTOS tasks |
|---|---|---|---|
| **1** | `buddy1_communication/` | Wi-Fi connection/reconnection, MQTT topics and commands, telemetry and heartbeat | Communication Task; Telemetry Collection Task |
| **2** | `buddy2_motion/` | Motor driver, encoders, PID speed control, encoder-based turns | Motion Control Task |
| **3** | `buddy3_navigation/` | Three IR sensors, line tracking, barcode detection/decoding | Line Following Task; Barcode Decoding Task |
| **4** | `buddy4_imu/` | IMU sampling/calibration/filtering, motion/tilt, hump-height and collision analysis | IMU Monitoring Task; Hump & Terrain Analysis Task |
| **5** | `buddy5_obstacle/` | Ultrasonic and servo scanning, obstacle profiling, avoidance and original-line recovery | Obstacle Scan Task; Obstacle Profile Task; Avoidance & Recovery Task |

This is **10 RTOS tasks total**. Buddy-specific `*_task.c` files implement the task behaviour; `src/rtos/task_manager.c` creates/registers tasks using the chosen micro T-Kernel APIs.

**Motor control ownership:** Only Buddy 2 writes motor direction/PWM outputs. Buddies 3 and 5 request motion via agreed command interfaces rather than independently driving motors. Buddy 1's remote commands, if supported, must be validated high-level requests—not direct unsafe motor commands.

## 5. Shared interfaces: agree before coding

Before teammates develop separate modules, agree on the interfaces in `firmware/include/`:

| Header | Expected content |
|---|---|
| `buddy1.h` … `buddy5.h` | Each subsystem's supported public functions and events. |
| `robot_types.h` | Shared data types for sensor readings, obstacles, motion and telemetry. |
| `motion_commands.h` | Command kinds, source, units, priority, timeout/completion rules. |
| `robot_state.h` | Overall robot operating state and transitions. |
| `pin_config.h` | Verified GPIO/power assignments; never silently change pins. |
| `robot_config.h` | Shared calibration constants and operational limits (validated, not guessed). |
| `rtos_config.h` | Task periods, preliminary priorities and resource budgets. |

**Command priority (from design review):**

1. Safe Stop/Fault
2. Obstacle Avoidance
3. Barcode Navigation
4. Normal Line Following

A normal FIFO queue alone does **not** enforce this priority order. `command_router.c` and `safety_manager.c` must make command arbitration, cancellation/preemption, stale-message handling and recovery explicit. The team's design uses events/states and prioritized commands instead of a separately scheduled Vehicle Controller RTOS task.

### Illustrative interaction

1. Buddy 3 identifies a line/barcode steering request.
2. Buddy 5 detects an obstacle and requests the robot to stop or slow.
3. Shared coordination gives safety/obstacle handling precedence over ordinary steering.
4. Buddy 2 executes the accepted motor command and reports completion/fault.
5. Buddy 5 scans, profiles, plans avoidance and requests movement through Buddy 2.
6. Buddy 3 reports line reacquisition; normal navigation resumes only when the system state allows it.

The exact queue/event layouts and APIs are **to be implemented and tested**.

## 6. RTOS design and timing

The Week 6 Design Review proposes these initial periods/priorities:

| Task | Activation | Preliminary priority |
|---|---|---|
| Wi-Fi/MQTT Communication | Events + heartbeat/reconnect checks | Medium |
| Telemetry Collection | Every 500–1000 ms | Low |
| Motion Control | Every 20–50 ms | High |
| Line Following | Every 20–50 ms | High |
| Barcode Decoding | Event-driven | Medium |
| IMU Monitoring | Every 10 ms | High |
| Hump & Terrain Analysis | Event-driven | Medium |
| Obstacle Scan | Periodic/event-driven | High |
| Obstacle Profile | Queue/event-driven | Medium |
| Avoidance & Recovery | Event-driven | High |

These are **design targets, not measured results**. Confirm actual execution time, task stack usage, queue capacity and timer behaviour after integrating micro T-Kernel. Avoid blocking operations in high-priority tasks. Test the design-review stopping requirement: motor output must halt within **100 ms of `stop()` being called**.

### Critical dependency: micro T-Kernel port

**Do not assume that a working RP2040/Pico W micro T-Kernel port is already available.** Before building all 10 tasks, the team must identify and verify a compatible kernel version/port and confirm that context switching, interrupts, timers, Pico SDK calls and Wi-Fi operate together. Keep any kernel integration clearly versioned and documented. The draft tree shows the team's RTOS *application wrappers*, not a complete kernel distribution.

## 7. Getting started on a development PC

### Prerequisites

- Git and a GitHub account with access to the team repository.
- VS Code (recommended) with the Raspberry Pi Pico extension, or equivalent SDK/toolchain setup.
- CMake, an ARM embedded compiler, a build tool such as Ninja, and a compatible Pico C SDK.
- A **verified micro T-Kernel port/configuration** before attempting the final RTOS firmware build.
- Python 3 only if working on the laptop-side telemetry receiver/tests.

### Clone the project

```bash
git clone https://github.com/vanessayee/Embedded-System-Group-14-Project.git
cd Embedded-System-Group-14-Project
```

### Build the Pico W firmware

When `firmware/CMakeLists.txt`, the Pico SDK and required kernel dependencies have been configured:

```bash
cmake -S firmware -B firmware/build -DPICO_BOARD=pico_w
cmake --build firmware/build
```

A successful firmware build is expected to generate a `.uf2` file in `firmware/build/`, assuming `pico_add_extra_outputs()` is configured. Flash the `.uf2` using the Pico W's supported BOOTSEL procedure.

**Important:** These are **target commands** once the project is implemented. The folder tree alone will not compile; CMake must list the actual source files and link the verified RTOS and SDK components. Do not treat placeholder `.c` files as completed functionality.

### CMake responsibilities

The `firmware/CMakeLists.txt` should:

1. Import/initialise the compatible Pico SDK.
2. Set the Pico W board target and C language standard.
3. Compile `src/main.c` and the buddy/shared/RTOS application source files.
4. Add `firmware/include/` to the compiler's header search path.
5. Link the appropriate Pico SDK libraries and verified micro T-Kernel implementation.
6. Generate ELF/UF2 firmware outputs.

**There is only one final Pico W firmware executable**; every buddy contributes to it.

## 8. Telemetry: what happens on the robot versus the laptop?

### Pico W side (`firmware/src/buddy1_communication/`)

- `wifi.c`: connect/reconnect to Wi-Fi.
- `mqtt.c`: connect to broker, publish and subscribe, validate accepted commands.
- `telemetry.c`: build structured measurements/status messages.
- `communication_task.c`: operate network communication and heartbeat/retries.
- `telemetry_task.c`: collect/publish periodic telemetry through the communication path.

### Laptop side (`telemetry/`)

| File | Role |
|---|---|
| `README.md` | How to run the telemetry receiver. |
| `mqtt_receiver.py` | Subscribe to topics; parse, validate and display messages. |
| `mqtt_config.example.json` | Example broker address/port/topics; copy for local configuration. |
| `requirements.txt` | Python dependencies, such as `paho-mqtt`. |

**A receiver is not an MQTT broker.** The Pico W and Python receiver both connect to a broker (e.g. Mosquitto) that must be installed/configured separately. Docker is **not required** for this simple option.

Do not commit Wi-Fi passwords, broker credentials or other secrets. Use local ignored configuration/environment variables and document safe example settings.

### Open design-review requirement (FR14)

The Week 6 Design Review describes Node-RED validation, PostgreSQL storage and Grafana current/historical visualisation. **Draft 7 implements only the proposed lightweight receiver layout**. The team must decide and document whether to:

- **Keep FR14:** build the database/monitoring services (which can run on a laptop or Raspberry Pi), expanding `telemetry/` as needed; or
- **Revise the approved architecture:** change FR14 and the related diagrams/justifications before treating basic receiver output as sufficient.

Any change after the Week 6 architecture freeze should be supported by engineering justification and agreed by the team/instructor where required.

## 9. GitHub collaboration workflow

Use **one repository** and separate feature branches:

| Buddy | Suggested branch |
|---|---|
| Buddy 1 | `feature/buddy1-communication` |
| Buddy 2 | `feature/buddy2-motion` |
| Buddy 3 | `feature/buddy3-navigation` |
| Buddy 4 | `feature/buddy4-imu` |
| Buddy 5 | `feature/buddy5-obstacle` |

`main` should contain reviewed, integrated work. Short-lived feature branches are also welcome for small changes.

### Normal workflow

```bash
# Start from an updated main branch
git checkout main
git pull origin main

# Create a branch for your work
git checkout -b feature/buddy5-obstacle

# After writing and testing code
git add firmware/src/buddy5_obstacle/
git commit -m "Add obstacle scanning implementation"
git push -u origin feature/buddy5-obstacle
```

Then open a **pull request** into `main`. Request a teammate's review, especially if modifying `firmware/include/`, `src/shared/`, `src/rtos/`, `CMakeLists.txt`, or pin definitions. Update your branch with changes from `main` and resolve conflicts before merging.

### Team rules

1. **Do not push incomplete or untested integrations directly to `main`.**
2. **Do not edit other buddies' interfaces without discussing the change first.**
3. **Do not directly control motors outside Buddy 2's module.**
4. **Do not duplicate GPIO pin definitions across buddy modules.** Use `pin_config.h`.
5. **Do not hardcode secrets or commit `firmware/build/`.**
6. Keep changes small and describe what was tested in every pull request.
7. If a new source file is created, update CMake accordingly.
8. Record hardware calibration results and relevant failures in `docs/testing/`.

## 10. Testing and integration plan

Every buddy should first demonstrate their subsystem independently, then participate in staged integration.

| Phase | Minimum evidence |
|---|---|
| **1. Toolchain/RTOS bring-up** | Successful Pico W flash; task scheduling demonstrated; port compatibility documented. |
| **2. Hardware bring-up** | Actual pin mapping and electrical requirements verified; individual sensor/motor tests. |
| **3. Buddy unit tests** | Repeated measurements, representative inputs and documented failure conditions. |
| **4. Motion + navigation** | Straight tracking, barcode navigation, calibrated encoder-based turns. |
| **5. IMU integration** | Reliable calibration, tilt/turn-rate events and hump measurements. |
| **6. Obstacle integration** | Coarse/fine scans, profile, avoidance, line reacquisition and safe-stop testing. |
| **7. Communication integration** | Telemetry received; heartbeat and network-loss recovery tested without disrupting local safety. |
| **8. Full mission** | Complete track test, accuracy, completion time, resource measurements and repeatability. |

Key open assumptions from the design review include MQTT non-blocking behaviour, encoder reliability, creep-speed deceleration, barcode discrimination using the three IR sensors, IMU noise/calibration, and ultrasonic/servo scan repeatability. Record test results before considering these validated.

### Definition of done for a buddy module

- Compiles in the shared firmware build (where dependencies are available).
- Uses agreed public headers and common commands/events.
- Has an independent test or documented hardware demonstration.
- Does not bypass subsystem ownership or safety rules.
- Includes evidence of edge cases/fault handling.
- Has a reviewed pull request and short integration notes.

## 11. Before full implementation: decisions to freeze

- [ ] Confirm actual hardware and validated Robo Pico pin mapping/electrical supply.
- [ ] Confirm micro T-Kernel version/port and Pico SDK compatibility.
- [ ] Agree on all buddy public APIs and common data structures.
- [ ] Define RTOS queues, event semantics, timeouts and command priorities/preemption.
- [ ] Agree on safe-stop, sensor-fault and line-recovery behaviour.
- [ ] Define MQTT topics, payload fields, units, validation and authentication.
- [ ] Decide whether to retain FR14's PostgreSQL/Grafana architecture or formally revise it.
- [ ] Configure a reproducible build and GitHub review workflow.

## 12. Project references

Internal team documents to place in `docs/design_review/` as appropriate:

- **CSC14 Week 6 Design Review** — requirements (FR/NFR), pin layout, ten RTOS tasks, task interactions, statecharts, and assumptions.
- **Project Briefing** — five-buddy responsibilities and expected deliverables.
- **Project Write-up** — mission requirements, subsystem work and assessment considerations.

> **Reminder to the team:** The repository structure is an organisational plan, not a working robot by itself. Keep implementations consistent with the approved requirements and update the documentation whenever the agreed architecture changes.
