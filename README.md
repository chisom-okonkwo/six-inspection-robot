# Six — Autonomous Indoor Inspection Robot

A Python-controlled indoor inspection prototype built on the **EZ-Robot Revolution Six** hexapod and **EZ-B v4** controller. Six uses an onboard camera, YOLO object detection, and an **HC-SR04 ultrasonic range sensor** to recognize selected obstacles, decide when they are close enough to avoid, and change heading. A separate safety pipeline holds movement when a person is detected. Inspection events are written to JSON Lines logs.

> **Project scope:** This is a supervised research/demo robot, not a certified collision-avoidance or human-safety system. Test in a clear indoor space, with a person ready to cut power. The current obstacle policy reacts to **backpacks and books**, plus **very close, unclassified surfaces** (the “wall” fallback). Other objects can still be physically hazardous.

## Quick start

From the **repository root**, on a machine with Python installed:

```bash
python -m venv .venv
source .venv/bin/activate      # Linux / WSL / macOS
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

On **Windows PowerShell**, activate the environment with `.venv\Scripts\Activate.ps1` instead. Use the same virtual environment for model downloads and running the app.

**Before joining the robot's Wi-Fi**, download the weights as described in [Download the AI models](#download-the-ai-models). Connect your computer to the EZ-B network, then run:

```bash
python -m app.autonomous_app --mode observe   # vision only; no movement
python -m app.autonomous_app --mode patrol    # autonomous robot movement
```

The first command is the recommended starting point. **Patrol moves physical servos.** Check the wiring, servo calibration, sensor readings, and safety behavior before enabling it.

## What this project does

| Capability | Implementation |
|---|---|
| Recognize people | YOLO object detection + dedicated safety event and clear-delay supervisor |
| Recognize navigation targets | YOLO labels filtered by `perception/object_policy.py` (currently `backpack` and `book`) |
| Confirm a target is near | HC-SR04 range; object avoidance starts at approximately **45 cm** |
| Detect an otherwise unrecognized close surface | HC-SR04-only **15 cm** threshold (called “wall” detection; it cannot identify a wall by type) |
| Select an avoidance heading | Camera target position, optionally aided by monocular depth for left/right clearance |
| Avoid | Interrupt forward walking, back up once, perform a committed turn, add incremental turns when necessary |
| Detect lack of progress | OpenCV optical-flow monitor while the robot is commanded forward |
| Log inspections | `inspection/` components write session/event records to `data/logs/*.jsonl` |
| Run without ARC | Python communicates **directly with EZ-B v4 over TCP**. ARC may be used offline to tune/export gait actions but is not the application's runtime controller. |

### Control flow

```text
EZ-B camera (TCP :24) ──> PerceptionWorker ──> YOLO person ─> SafetySupervisor
                      │                                     │
                      └──> backpack/book candidate          └──> safety stop
                                    │
HC-SR04 (D20/D21) ──────────────────┼──> UltrasonicWorker
                                    │          │
                                    │          ├─ camera + <=45 cm: object event
                                    │          └─ no camera target + <=15 cm: close-surface event
                                    │
Optical-flow monitor ──> stuck event│
                                    ▼
                            MotionWorker (priority-controlled)
                                    ▼
                     SixRobot / gait actions -> EZB TCP :23 -> servos
```

All thresholds are **starting values**, not universally safe stopping distances. In particular, the hexapod's legs extend past the body and the ultrasonic transducers may not see a small object near a foot.

## Hardware and wiring

**Expected reference platform:** Revolution Six, EZ-B v4, EZ-B-compatible camera, 12 servos, and an HC-SR04 facing approximately forward. A different EZ-B robot is possible but requires a robot-specific servo map, calibrated gait, and mounting/clearance tests.

| HC-SR04 pin | Connection on reference setup |
|---|---|
| `VCC` | **Regulated 5 V**, not an EZ-B digital red power pin |
| `GND` | Common ground shared by the 5 V supply and EZ-B |
| `TRIG` | EZ-B **D20 signal** pin |
| `ECHO` | EZ-B **D21 signal** pin |

**Electrical warning:** The EZ-B v4 **analog red** pins (such as A7) supply regulated **3.3 V**, usually insufficient for a standard 5 V HC-SR04. The **digital red** pins follow battery/Vin voltage (often around 7.4 V), so **do not** use them to power a 5 V sensor. The EZ-B v4 digital inputs are 5 V tolerant and its outputs are 3.3 V; verify the sensor variant and controller specs before wiring. On a different controller, its Echo pin may need level shifting. Refer to the [Synthiam EZ-B v4 hardware guide](https://synthiam.com/Support/Hardware/EZ-B-v4).

**Servo mapping used by the reference Six configuration:**

| Leg | Hip | Knee |
|---|---|---|
| Front right (FR) | D0 | D1 |
| Front left (FL) | D3 | D4 |
| Middle left (ML) | D6 | D7 |
| Rear left (RL) | D9 | D10 |
| Rear right (RR) | D12 | D13 |
| Middle right (MR) | D15 | D16 |

Check `robot/six_robot.py` and the gait configuration before moving *your* robot. ARC-exported `AutoPosition` frames contain **absolute servo positions**: do not apply old procedural `servo_directions.json` signs a second time to those frames. If your branch uses `robot/arc_gait.py`, it also needs the matching exported `six_stock_gait.AutoPosition` file at the location configured by that loader. The exact gait-file path and import API depend on your checked-out `SixRobot` implementation.

## Network setup

The EZ-B supports two connection styles:

| Mode | Description | Typical controller address |
|---|---|---|
| **AP / direct Wi-Fi** (reference setup) | Your laptop joins the Wi-Fi network created by Six. Internet is usually unavailable on that connection. | `192.168.1.1` |
| **Client mode** | Both laptop and EZ-B join your router. Internet may remain available. | Router-assigned IP; update `robot/ezb.py` and camera config as needed |

On the reference AP setup, the Python processes use **TCP port 23** for EZ-B commands and **TCP port 24** for the camera. Confirm the laptop is on the robot's network; close other programs (including ARC) that may already be using the camera/controller.

```bash
ping 192.168.1.1
python -c "import socket; s=socket.create_connection(('192.168.1.1',23),5); print('EZ-B controller port OK'); s.close()"
python -c "import socket; s=socket.create_connection(('192.168.1.1',24),5); print('EZ-B camera port OK'); s.close()"
```

Connection addresses and ports are specified in `robot/ezb.py` and `perception/camera.py` respectively. If you use WSL, make sure your WSL Python environment can reach the laptop's active Wi-Fi connection.

## Installation

1. Clone or download this repository and open a terminal in its **root directory** (the folder containing `app/`, `autonomy/`, `perception/`, `robot/`, and this README).
2. Install **Python 3.10 or 3.11** as a practical baseline, then create and activate a virtual environment.
3. Run `python -m pip install -r requirements.txt` while connected to ordinary Internet.
4. Download both model files into `models/` while Internet is available.
5. Make sure your robot-specific `SixRobot` gait setup and HC-SR04 wiring are correct.
6. Connect to the robot's Wi-Fi, run the sensor tests, then start `observe` mode before `patrol`.

The explicit direct Python dependencies are **Ultralytics**, **OpenCV**, and **NumPy**. `ultralytics` installs PyTorch and its supported transitive dependencies. If you use an NVIDIA GPU, follow [PyTorch's official installation instructions](https://pytorch.org/get-started/locally/) to choose the correct CUDA build; otherwise CPU inference works, but can be slower. Standard-library modules (`socket`, `threading`, `time`, `json`, `argparse`, etc.) do not need separate `pip` installs.

### Download the AI models

These large `.pt` files are **not included in Git**:

```text
models/
├── yolo26n.pt          # required: YOLO26 nano object detector
└── yolo26n-depth.pt    # optional but currently used when available
```

While connected to **regular Internet** (not the robot's AP Wi-Fi), run from the repository root:

```bash
mkdir -p models
cd models
python -c "from ultralytics import YOLO; YOLO('yolo26n.pt'); YOLO('yolo26n-depth.pt')"
cd ..
```

In **PowerShell**, create the folder using `New-Item -ItemType Directory -Force models`, then `Set-Location models`, run the same Python command, and `Set-Location ..`. Ultralytics downloads pretrained weights on first use; confirm the two files really appear in `models/`. If your installation saved them elsewhere, locate and copy the downloaded `.pt` files there **before** connecting to Six. See the [official YOLO26](https://docs.ultralytics.com/models/yolo26/) and [YOLO depth](https://docs.ultralytics.com/tasks/depth/) documentation.

Verify them offline, from the repository root:

```bash
python -c "from pathlib import Path; print({p.name: p.exists() for p in (Path('models/yolo26n.pt'), Path('models/yolo26n-depth.pt'))})"
python -c "from ultralytics import YOLO; YOLO('models/yolo26n.pt'); YOLO('models/yolo26n-depth.pt'); print('Models loaded from local files')"
```

**Model roles:** `yolo26n.pt` is needed for person and backpack/book recognition. The depth model supplies **secondary camera-depth and left/right clearance estimates**; it does **not** replace HC-SR04 distance gating. In the current implementation `PerceptionWorker` still attempts to load it, so keep it installed to reproduce the complete tested pipeline. With the depth file missing or unsupported, `DepthEstimator` is designed to disable itself, but that reduces navigation information and may attempt a download on first load. Avoid relying on network downloads at runtime.

> Git tip: Do not commit model weights or inspection logs. Typical `.gitignore` entries: `models/*.pt`, `.venv/`, `__pycache__/`, `*.pyc`, and `data/logs/`. Keep `requirements.txt` and the model-download instructions committed.

## Run modes and CLI reference

Run commands **from the repository root**. Use `python -m ...` to keep package imports consistent.

### Main application

```bash
# Observe only (default): camera/YOLO/depth/safety/UI/logging; no servo movement
python -m app.autonomous_app --mode observe

# Autonomous patrol: starts EZ-B control, ultrasonic fusion, and motor movement
python -m app.autonomous_app --mode patrol

# Change YOLO inference candidate floor and person clear delay
python -m app.autonomous_app --mode patrol --confidence 0.30 --clear-delay 3

# Explicit detector weights path
python -m app.autonomous_app --mode observe --model models/yolo26n.pt

# Show authoritative flags in your checked-out version
python -m app.autonomous_app --help
```

| Flag | Default | Meaning |
|---|---|---|
| `--mode {observe,patrol}` | `observe` | `observe`: computer vision only; `patrol`: full autonomous locomotion and sensor fusion |
| `--model PATH` | `models/yolo26n.pt` | Object detector weights; **not** the depth model path |
| `--confidence FLOAT` | `0.25` | YOLO inference *candidate floor*, not the final person or obstacle-class threshold |
| `--clear-delay FLOAT` | `2.0` | Seconds the safety supervisor waits after a person clears before releasing its safety hold |
| `--help` | — | List flags supported by your installed version |

**Important:** The current main app does **not** expose `--cycles`, `--wall-distance`, `--object-distance`, `--turn-degrees`, or `--depth-model` command-line flags. Those values are edited in the source files documented in [Tuning behavior](#tuning-behavior). Do not pass them to `app.autonomous_app` unless you have separately added CLI support.

**Observe-mode detail:** In the current application, ultrasonic polling starts **only in patrol mode**, after the EZ-B motor-controller TCP connection is established. To test HC-SR04 without moving Six, use `scripts.test_sensor` instead. `observe` still runs perception, safety logic, progress-monitor thread (inactive when not patrolling), and inspection logging.

### Tests and calibration

| Command | Purpose | Relevant arguments |
|---|---|---|
| `python -m scripts.test_sensor` | Read HC-SR04 range without starting patrol | `--trigger 20 --echo 21 --interval 0.10` |
| `python -m scripts.test_depth` | Show camera and estimated monocular depth | No flags in the supplied test script |
| `python -m scripts.test_rotate_cycles left --cycles 1` | Measure the physical turn of one ARC Rotate Left action | Positional `left` or `right`; `--cycles N` |
| `python -m scripts.test_rotate_cycles right --cycles 3` | Calibrate multiple right-turn cycles | Positional `left` or `right`; `--cycles N` |

These commands require their corresponding `scripts/*.py` files to be present in your checkout. `test_depth` needs the depth checkpoint and the EZ-B camera. `test_sensor` needs the EZ-B controller connection and correctly powered HC-SR04. **Rotate-cycle tests move the robot with no autonomous person-safety supervisor**: use a clear/elevated test setup and hands-off clearance.

```bash
# Adjust test sensor pins and polling rate if you rewired it
python -m scripts.test_sensor --trigger 20 --echo 21 --interval 0.10

# Test a single calibrated rotate cycle
python -m scripts.test_rotate_cycles left --cycles 1

# Test two cycles (not an exact angular command)
python -m scripts.test_rotate_cycles right --cycles 2
```

**Want only one forward walking cycle?** There is **no global `--cycles` flag in `autonomous_app.py`**. `MotionWorker.run()` currently requests `robot.forward(cycles=1, ...)` each patrol iteration and repeats until interrupted. The robot API may also expose `backward(cycles=N)` / `rotate_left(cycles=N)` / `rotate_right(cycles=N)`, but those are **Python method arguments**, not application CLI options. Use a purpose-built manual test script with an operator and appropriate safety precautions; do not assume the main autonomous app limits itself to a specified number of cycles.

## Tuning behavior

The code deliberately separates *what to react to*, *when to react*, and *how to move*. The table lists the currently implemented **source-level** settings; stop the program, edit, save, and restart to apply changes.

| Goal | File / setting | Reference default |
|---|---|---|
| Recognize only backpack and book for navigation | `perception/object_policy.py` → `OBSTACLE_CLASSES` | `{"backpack", "book"}` |
| Raise/lower class-specific filtering | `perception/object_policy.py` → `OBSTACLE_MIN_CONFIDENCE` | Per-class thresholds |
| Restrict vision candidates to projected path | `perception/obstacle_analyzer.py` → corridor-overlap settings | Corridor approximately 28–72% of frame width |
| Start backpack/book avoidance later/earlier | `app/autonomous_app.py` → `UltrasonicWorker(object_trigger_distance_cm=...)` | **45 cm** |
| Clear object condition | Same → `object_clear_distance_cm` | **55 cm** |
| Detect a very close unclassified surface | Same → `wall_trigger_distance_cm` | **15 cm** |
| Clear wall/close-surface condition | Same → `wall_clear_distance_cm` | **22 cm** |
| Change sensor pins | Same → `trigger_port`, `echo_port` | **D20 / D21** |
| Back up once before object turn | `autonomy/motion_worker.py` → `backoff_cycles` | `1` |
| Initial object turn | Same → `initial_turn_degrees` | `70.0` (nominal) |
| Initial wall turn | Same → `wall_initial_turn_degrees` | `70.0` (nominal) |
| Additional turn after failed avoidance | Same → `turn_increment_degrees` and `wall_turn_increment_degrees` | `25.0` (nominal) |
| Max extra object turn attempts | Same → `max_turn_extensions` | `4` |
| Stuck detection sensitivity | `autonomy/progress_monitor.py` → `min_motion_score`, timing thresholds | Tune from observed camera movement |
| Reduce CPU cost of depth | `perception/perception_worker.py` → `depth_every_n_frames` | `3` |

### Why an initial “70° turn” may execute three cycles

This is **not** an IMU-measured heading. The motion worker converts a desired approximate angle into stock ARC rotate cycles:

```python
cycles = math.ceil(initial_turn_degrees / estimated_degrees_per_rotate_cycle)
```

With the reference values `70.0 / 25.0`, Six executes **3 ARC rotate cycles**. To request **2 initial cycles**, change `initial_turn_degrees=50.0`; independently set `wall_initial_turn_degrees=50.0` if you want the same reduction for walls. The `estimated_degrees_per_rotate_cycle` value is a **calibration estimate**, not a way to arbitrarily suppress turns. Run `scripts.test_rotate_cycles`, measure the physical heading change, and calculate `measured_degrees / cycles` to calibrate it. Even after calibration, leg slip makes exact 90°/180° yaw unreliable without heading feedback.

The normal sequence is: **detect → interrupt forward gait → one backoff → committed turn → fresh perception/range → incremental turn if still blocked**. When the same object reappears quickly, recent-obstacle memory reduces the tendency to back up repeatedly. For wall-like hazards, the system uses the separate ultrasonic threshold and wall-turn settings.

## Project structure (reference)

```text
.
├── app/
│   └── autonomous_app.py        # CLI entry point and UI
├── autonomy/
│   ├── motion_worker.py         # patrol, backoff, turn, wall and stuck recovery
│   ├── progress_monitor.py      # visual optical flow / stalled movement
│   ├── robot_state.py           # thread-safe shared status and frame data
│   ├── safety_supervisor.py     # high-priority person hold / release
│   └── ultrasonic_worker.py     # HC-SR04 polling and camera/range fusion
├── perception/
│   ├── camera.py                # EZ-B camera stream
│   ├── detector.py              # YOLO detections and person filtering
│   ├── depth_estimator.py       # optional secondary monocular depth
│   ├── object_policy.py         # obstacle class and confidence policy
│   ├── obstacle_analyzer.py     # camera corridor / target / escape-side analysis
│   └── perception_worker.py     # camera + models + event updates
├── robot/
│   ├── ezb.py                   # EZ-B TCP protocol and native HC-SR04 command
│   └── six_robot.py             # servo/gait motion interface (ARC gait if configured)
├── inspection/
│   ├── incident_logger.py      # JSONL event logger
│   └── inspection_manager.py   # inspection lifecycle/event worker
├── scripts/
│   ├── test_sensor.py           # HC-SR04 distance test
│   ├── test_depth.py            # optional depth test
│   └── test_rotate_cycles.py    # physical turn calibration
├── models/                      # local .pt weights (not committed)
├── data/logs/                   # generated inspection JSONL logs
├── requirements.txt
└── README.md
```

Some branches additionally contain `robot/arc_gait.py`, exported `.AutoPosition` gait data, procedural `six_config.py`, or servo-direction calibration files. Those are **robot-implementation-dependent**. This README describes the most recent integrated Python/ultrasonic workflow; use the checked-out source and `--help` as the final authority if your repository differs.

## Inspection logs and runtime controls

The application records inspection events in a time-stamped JSON Lines file, for example:

```text
data/logs/inspection_YYYYMMDD_HHMMSS.jsonl
```

Each line is a JSON object; the inspection manager records session start/end and other implemented events. The terminal prints the precise log path at startup.

| Control | Action |
|---|---|
| `Q` in the OpenCV window | Request graceful shutdown |
| `E` in the OpenCV window | Emergency **servo release**: drops holding torque; Six may fall or collapse |
| `Ctrl+C` in the terminal | Request shutdown |

**Safety priorities:** person safety preempts movement; then very close wall/unknown-surface hazards; then configured object avoidance; then stuck recovery; then forward patrol. Camera failure triggers a conservative fail-safe stop rather than allowing blind patrol. These are software controls, **not** a physical emergency-stop circuit.

## Troubleshooting

| Symptom | What to check |
|---|---|
| `Download failure`, `Environment may be offline` | Download both `.pt` weights on regular Internet **before** joining the robot AP; verify files exist under `models/` |
| `Camera=CONNECTING` then `timed out` | Check Wi-Fi, test TCP **24**, close ARC/other camera users, reboot EZ-B, check camera cable |
| Controller connection fails | Check EZ-B IP and TCP **23**, Wi-Fi, board power; only one controller client may be allowed |
| HC-SR04 reads `0` or `255` repeatedly | Check **regulated 5 V**, common ground, D20 TRIG, D21 ECHO, sensor alignment, and sensor variant |
| Backpack/book detected but no avoidance | Is it within the camera corridor? Does sensor range cross the **45 cm** trigger for enough readings? Check `ultrasonic_worker.py` status and the `navigation_obstacle_event` path |
| Everything causes avoidance | Check `OBSTACLE_CLASSES`; remember that a separate **15 cm range-only** close-surface event is intentional |
| Three turns when you expected two | Check `initial_turn_degrees` / `wall_initial_turn_degrees` and the measured degrees-per-cycle calibration |
| Repeat backing up or turning | Verify recent-obstacle memory, range hysteresis, fresh perception, and that you are running the latest `motion_worker.py` |
| Sensor appears fine but movement stalls | See `ProgressMonitor` optical-flow status; low texture, camera vibration, or leg slip can create false positives/negatives |
| OpenCV window fails under WSL | Check WSLg/display support; run on desktop Python with GUI support if needed |
| Servos move incorrectly | Verify your exact servo mapping and ARC exported gait; test with body supported before floor patrol |

## Suggested first-run checklist

1. **Power off** the robot and check all HC-SR04 and servo wiring, including the regulated 5 V supply and common ground.
2. With regular Internet, install dependencies and download the two model files locally.
3. Connect to the EZ-B Wi-Fi; test TCP ports 23 and 24.
4. Run `python -m scripts.test_sensor` and move a flat object through known distances.
5. Run `python -m app.autonomous_app --mode observe`; verify model load, person safety, class filtering, camera FPS, and logs.
6. Calibrate turns using `python -m scripts.test_rotate_cycles left --cycles 1` in a controlled setup.
7. Start patrol **with the body supported**, then test one backpack/book in a large clear space.
8. Test a wall/unknown surface last. **15 cm may be too close for Six's protruding legs**; increase the wall threshold if required.

## Known limitations and extension points

- **HC-SR04 reports distance, not identity.** A “wall” event can also be furniture, a person, or any other close reflector. A narrow sensor field of view can miss side/low obstacles.
- **YOLO detects only the classes in its checkpoint.** `object_policy.py` currently allows `backpack` and `book`; a different bag class may not be recognized as a backpack.
- **Single-camera depth is estimated**, not a substitute for a calibrated physical range sensor. It remains useful for approximate directional clearance but consumes processing time.
- **Optical flow is not odometry.** It approximates visual motion and cannot conclusively prove that the chassis is translating.
- **Turn “degrees” are estimates.** Exact angle control needs calibrated visual odometry, an IMU, or another heading-feedback mechanism.
- **Custom hardware requires adaptation.** Mounting, servo addresses, gait files, thresholds, IP addresses, and safety clearance are specific to your build.

For hardware/network details consult [Synthiam's EZ-B v4 documentation](https://synthiam.com/Support/Hardware/EZ-B-v4); for models consult [Ultralytics YOLO usage](https://docs.ultralytics.com/usage/python) and [depth model documentation](https://docs.ultralytics.com/tasks/depth/).
