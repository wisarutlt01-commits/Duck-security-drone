# Drone Tracker — Refactor Plan & Task List

> **Audience:** an AI coding agent (Sonnet) that will execute this refactor end-to-end.
> **Author of plan:** Opus (analysis pass over the existing `drone_tracker/drone_tracker/` tree).
> **Style rules for the executor:** keep changes mechanical and reviewable, preserve runtime
> behaviour for the two surviving modes, match the existing code style (type hints, dataclasses,
> module docstrings), and do **not** invent new features. When in doubt, prefer deleting code over
> keeping a half-used path.

---

## 1. Goal (what "done" looks like)

Refactor the existing drone-tracking codebase so it is **easy to maintain and easy for a human to
read**, with two — and only two — run modes:

| Mode | Where it runs | Camera source | MAVLink link | YOLO device |
|------|---------------|---------------|--------------|-------------|
| **`hardware`** | Raspberry Pi 5 (no AI HAT yet) | Pi Camera via V4L2/OpenCV | serial `/dev/ttyAMA0` | `device` cfg, **default CPU** |
| **`gazebo`** | Tracker on **Pi 5**, Gazebo+SITL on a **desktop PC** (networked) | H.264/RTP UDP stream from desktop | UDP to desktop SITL | `device` cfg, **default CPU** |

Two hard requirements from the user:

1. **Remove the entire "pure simulation" section.** The fake `--sim` mode (no hardware, commands
   just logged) must be deleted everywhere — flags, config fields, branches, and the tools that
   depend on it.
2. **Target Raspberry Pi 5 without an AI HAT (for now).** Inference defaults to **CPU**, but keep
   the inference device **configurable** (`device` defaults to `"cpu"`) so a future AI HAT can be
   enabled by changing one setting — **do not hardcode `"cpu"`**. Simplify the device logic, but
   leave the pass-through intact.

Plus: **compact the layout** into a small, flat package (kill the double-nesting and committed junk).

> ⚠️ Keep the Gazebo mode. The user explicitly wants the second "version" where Gazebo runs on a
> computer and the tracker runs on the Pi 5. "Remove all simulation" refers to the **pure-Python
> `--sim`** mode, **not** Gazebo.

---

## 2. Target directory layout

Flatten the double-nested `drone_tracker/drone_tracker/...` down to a single compact package.
Merge the two vision files into one `VisionSystem` driven by a pluggable **frame source**.

```
drone_tracker/
├── README.md                  # rewritten: only hardware + gazebo modes
├── REFACTOR_PLAN.md           # this file (can be deleted once done)
├── requirements.txt
├── main.py                    # single entry point: --mode hardware|gazebo
├── tracker/
│   ├── __init__.py
│   ├── config.py              # was config/settings.py (sim fields removed)
│   ├── vision.py              # unified VisionSystem + Detection + TrackState
│   ├── camera.py              # FrameSource base + V4L2Camera + GazeboUDPStream
│   ├── controller.py          # FlightController (sim branches removed)
│   ├── mavlink.py             # MAVLinkInterface (sim branches removed)
│   ├── pid.py                 # unchanged logic
│   ├── kalman.py              # unchanged logic
│   └── logger.py              # unchanged logic
├── tools/
│   └── test_camera.py         # standalone camera+YOLO smoke test (keep, de-sim)
├── models/
│   └── drone_yolo.pt          # trained weights (gitignored if large)
├── gazebo/                    # desktop-side Gazebo assets (renamed from sim/)
│   ├── README_GAZEBO.md       # how to launch Gazebo+SITL on the PC and stream to Pi
│   ├── worlds/
│   ├── models/
│   └── launch/
└── .gitignore                 # venv, __pycache__, logs, debug_frames, *.tlog, etc.
```

**Naming note:** `core/` → `tracker/`, `config/settings.py` → `tracker/config.py`,
`core/mavlink_interface.py` → `tracker/mavlink.py`, `sim/` → `gazebo/`. Update **all** imports
accordingly (`from config.settings import` → `from tracker.config import`, etc.).

---

## 3. Files to DELETE outright

Remove these from the repo (and add patterns to `.gitignore` so they don't come back):

| Path | Reason |
|------|--------|
| `drone_tracker/drone_tracker/venv/` | committed virtualenv — never commit |
| `drone_tracker/drone_tracker/**/__pycache__/` | build artifacts |
| `drone_tracker/drone_tracker/debug_frames/` | scratch images |
| `drone_tracker/drone_tracker/logs/*.log*` | runtime logs |
| `drone_tracker/drone_tracker/eeprom.bin`, `mav.tlog`, `mav.tlog.raw` | SITL runtime junk |
| `drone_tracker/drone_tracker/debug_detect.py` | ad-hoc debug script tied to sim |
| `drone_tracker/drone_tracker/tools/inject_detection.py` | **pure-sim** detection injector — delete |
| `drone_tracker/drone_tracker/tools/calibrate_pid.py` | runs the **pure-sim** stack — delete |
| `drone_tracker/drone_tracker/launch_sim.sh` | superseded by `gazebo/launch/` (see §7) — review then delete or move |
| `**/.vscode/browse.vc.db` | IDE cache |

> If `tools/calibrate_pid.py`'s PID-tuning value is wanted later, it can be reborn against the
> `gazebo` mode — but it is **out of scope** here. Delete it now.

---

## 4. Step-by-step tasks

Work in this order. Each step is independently committable.

### Task 0 — Set up the new skeleton
- [ ] Create the flat `drone_tracker/` layout from §2 (move, don't rewrite-from-scratch, the files
      that survive).
- [ ] Add `.gitignore` covering: `venv/`, `.venv/`, `__pycache__/`, `*.pyc`, `logs/`,
      `debug_frames/`, `*.tlog*`, `eeprom.bin`, `models/*.pt` (keep a `.gitkeep`).
- [ ] Delete everything in §3.

### Task 1 — Strip PURE SIM from config
File: `tracker/config.py` (was `config/settings.py`).
- [ ] Delete the `sim_mode: bool` field.
- [ ] Keep `gazebo_mode` **or** replace both booleans with a single `mode: str = "hardware"`
      (`"hardware"` | `"gazebo"`). **Recommended:** use the single `mode` string — it's clearer than
      two mutually-exclusive bools. Add a helper `@property is_gazebo(self) -> bool`.
- [ ] Keep the inference `device` field but change the default `"auto"` → `"cpu"` (Pi 5 has no GPU
      today). **Keep it configurable** — the user plans to add an AI HAT later, so it must be a
      one-line setting change, not a code edit. Add a comment listing valid values
      (`"cpu"` | a future accelerator string). Optionally expose `--infer-device` on the CLI.
- [ ] Keep the Gazebo UDP fields (`gst_udp_host`, `gst_udp_port`, `sitl_connection`). For the
      Pi↔desktop setup, **`gst_udp_host`/`sitl_connection` must point at the desktop PC's IP**, so
      update the comments to say "set to the Gazebo PC's LAN IP", and change the default host from
      `127.0.0.1` to a clearly-fake placeholder comment (keep `127.0.0.1` as default but document it).
- [ ] Remove now-unused topic fields tied to deleted backends (`gazebo_camera_topic`,
      `gazebo_classic_topic`, `sitl_target_conn`) **iff** §5 drops the gz-transport/pygazebo backends.

### Task 2 — Collapse the two vision files into one
This is the biggest readability win. `core/vision.py` and `core/vision_gazebo.py` duplicate
`Detection`, `TrackState`, the Kalman wiring, `_process_results`, `_draw_overlay`, `_update_fps`,
`get_track_state`, `reset_track`, and `_update_detection`. The **only** real difference is the
frame source and the YOLO `device`/`half` flags.

- [ ] Create `tracker/camera.py` with a tiny frame-source abstraction:
  ```python
  class FrameSource(Protocol):
      def read(self) -> Optional[np.ndarray]: ...   # latest BGR frame or None
      def release(self) -> None: ...
  ```
  - [ ] `V4L2Camera(FrameSource)` — moves the `_open_camera()` logic out of `vision.py`
        (V4L2 backend, BUFFERSIZE=1, MJPG, width/height/fps). `read()` wraps `cap.read()`.
  - [ ] `GazeboUDPStream(FrameSource)` — the **UDP** camera path from `vision_gazebo.py`. Keep
        `_GstUDPSubscriber` if OpenCV has GStreamer, else `_FFmpegUDPSubscriber`. Expose a single
        `read()` returning the latest frame. (See §5 for which backends to keep.)
- [ ] Rewrite `tracker/vision.py` so `VisionSystem.__init__` takes a `FrameSource` (and `cfg`,
      `shutdown`). The `run()` loop becomes source-agnostic:
  ```python
  frame = self.source.read()
  if frame is None: time.sleep(0.01); continue
  results = self.model.predict(frame, imgsz=..., conf=..., iou=...,
                               classes=..., verbose=False, half=False, device="cpu")
  ```
- [ ] `main.py` builds the right `FrameSource` for the chosen mode and injects it.
- [ ] **Delete `core/vision_gazebo.py`** once its UDP subscriber classes have moved into
      `tracker/camera.py`. There should be exactly **one** `VisionSystem`, one `Detection`,
      one `TrackState`.
- [ ] **Keep** a *simplified* device pass-through: read `cfg.device` (default `"cpu"`), pass it to
      `model.predict(..., device=self.device, half=(self.device != "cpu"))`. Drop only the CUDA
      auto-probe (`torch.cuda.is_available()`); keep the field so an AI HAT can be selected later.

### Task 3 — Strip PURE SIM from MAVLink
File: `tracker/mavlink.py` (was `core/mavlink_interface.py`).
- [ ] Delete every `if (self.cfg.sim_mode and not self.cfg.gazebo_mode) ...` early-return branch in
      `connect()`, `disconnect()`, `safety_checks()`, `send_velocity()`, `set_mode()`.
- [ ] Keep the real serial path (hardware) and the UDP path (gazebo). Selection is now just:
      `conn_str = cfg.sitl_connection if cfg.is_gazebo else cfg.serial_device`.
- [ ] Keep `MAVLINK_AVAILABLE` guard for a missing `pymavlink` import, but it should now **fail
      loudly** (return False / raise) instead of silently dropping into a fake sim.
- [ ] Leave the heartbeat listener, safety checks, and velocity/yaw masks unchanged.

### Task 4 — De-sim the controller & main
- [ ] `tracker/controller.py`: imports change to `from tracker.vision import ...` etc. Logic is
      mode-agnostic already — no behavioural change. Just fix imports and the `VisionSystem` type
      hint (it's now the unified class).
- [ ] `main.py`: replace `--sim`/`--gazebo` mutually-exclusive group with a single
      `--mode {hardware,gazebo}` (default `hardware`). Delete the pure-sim startup messaging and the
      `if cfg.sim_mode` skip of safety checks. Keep the Gazebo GUIDED-mode set + the
      hardware-only `safety_checks()` gate.
- [ ] Build the `FrameSource` in `main()` based on `--mode` and pass it to `VisionSystem`.
- [ ] Update the module docstring usage block to show only the two surviving commands.

### Task 5 — Tools
- [ ] Keep `tools/test_camera.py`; remove any `--sim` argument and any import of deleted modules.
      It should just open the camera (V4L2) + run YOLO on CPU + print/preview detections.
- [ ] Confirm `tools/inject_detection.py` and `tools/calibrate_pid.py` are deleted (Task §3).

### Task 6 — Gazebo assets (desktop side)
- [ ] Rename `sim/` → `gazebo/`. Keep `worlds/`, `models/`, `launch/`, and the README files.
      Consolidate `README_GAZEBO.md`, `QUICKFIX.md`, `README_PROTOBUF_FIX.md` into a single
      `gazebo/README_GAZEBO.md` (keep the genuinely useful fixes, drop duplication).
- [ ] Update `launch/start_sim.sh` (and friends) so the GstCameraPlugin streams to the **Pi 5's
      IP**, and SITL exposes its MAVLink UDP to the Pi. Document the two IPs clearly at the top of
      the script (`PI_IP`, `DESKTOP_IP`).
- [ ] `gazebo/scripts/target_mover.py` stays (it drives the target drone in SITL).

### Task 7 — Docs & requirements
- [ ] Rewrite `README.md`: drop the "Step 2: pure sim" section; document exactly two modes with
      copy-paste commands (see §6). Add a short "Networked Gazebo testing (PC ↔ Pi 5)" section.
- [ ] `requirements.txt`: keep `opencv-python`, `ultralytics`, `numpy`, `pymavlink`. Remove the
      pygazebo/gz-transport notes **iff** those backends are dropped in §5; otherwise keep them in a
      clearly-labelled "Gazebo desktop only" block. Add a comment that torch will be the **CPU**
      wheel on the Pi.

---

## 5. Decision: which Gazebo camera backends to keep

The current `vision_gazebo.py` ships **four** backends. For the "Gazebo on desktop, tracker on Pi 5"
topology the camera **must cross the network**, so:

- ✅ **Keep `_GstUDPSubscriber`** (GStreamer UDP) — works across machines, this is the primary path.
- ✅ **Keep `_FFmpegUDPSubscriber`** — fallback when OpenCV lacks GStreamer. Good to keep.
- ❌ **Drop `_GzTransportSubscriber`** (gz-transport) — same-machine discovery; awkward across hosts.
- ❌ **Drop `_PyGazeboSubscriber`** (pygazebo / Gazebo Classic) — legacy, same-machine.

**Decision is final: UDP-only.** The confirmed topology is Gazebo + ArduPilot SITL on a desktop PC
and the tracker on a separate Pi 5. gz-transport relies on Gazebo's local (same-host) discovery and
does not bridge two machines cleanly, so it cannot deliver camera frames to the Pi — **drop it**
along with pygazebo. Dropping both removes ~120 lines and the `gz`/`pygazebo` import probing.

> Net effect: `tracker/camera.py`'s `GazeboUDPStream` tries GStreamer UDP, then ffmpeg UDP, then
> raises a clear error telling the user to install one of them. No `gz`/`pygazebo` imports remain.

---

## 6. Target CLI after refactor

```bash
# ── Mode 1: real flight on the Pi 5 (no AI HAT, CPU inference) ──
python main.py --mode hardware \
    --model models/drone_yolo.pt \
    --device /dev/ttyAMA0 --baud 57600 \
    --conf 0.3

# ── Mode 2: tracker on Pi 5, Gazebo+SITL on desktop PC ──
#   On the DESKTOP: launch Gazebo + SITL, stream camera to <PI_IP>:5600,
#                   expose SITL MAVLink to <PI_IP>:14550  (see gazebo/README_GAZEBO.md)
#   On the PI:
python main.py --mode gazebo \
    --model models/drone_yolo.pt \
    --sitl-conn udp:<DESKTOP_IP>:14550 \
    --gst-host 0.0.0.0 --gst-port 5600
```

No `--sim` anywhere. `--mode` defaults to `hardware`.

---

## 7. Networking notes for the Pi 5 ↔ desktop Gazebo setup

The executor should capture these in `gazebo/README_GAZEBO.md`:

- **Camera (desktop → Pi):** `ardupilot_gazebo`'s `GstCameraPlugin` sends H.264/RTP over UDP.
  Point its `udpHost` at the Pi's IP and `udpPort=5600`. The Pi listens with
  `--gst-host 0.0.0.0 --gst-port 5600`.
- **MAVLink (bidirectional):** start SITL with an extra output to the Pi, e.g.
  `sim_vehicle.py ... --out=udp:<PI_IP>:14550`, and run the tracker with
  `--sitl-conn udp:<DESKTOP_IP>:14550`.
- Same LAN, no firewall blocking 5600/UDP and 14550/UDP. Document a `ping` + `nc -u` sanity check.

---

## 8. Acceptance criteria (how the executor self-checks)

- [ ] `grep -ri "sim_mode\|--sim\|PURE SIM\|inject_detection\|calibrate_pid" tracker/ main.py` returns **nothing**.
- [ ] `grep -ri "cuda\|_resolve_device\|gz.transport\|pygazebo" tracker/` returns **nothing** (gz-transport/pygazebo dropped per §5; CUDA auto-probe removed). Note: a configurable `device` field defaulting to `"cpu"` **must remain** (future AI HAT).
- [ ] Exactly one definition each of `class VisionSystem`, `class Detection`, `class TrackState`.
- [ ] No committed `venv/`, `__pycache__/`, `*.pyc`, `logs/`, `debug_frames/`, `*.tlog`.
- [ ] `python -c "import ast,glob; [ast.parse(open(f).read()) for f in glob.glob('tracker/*.py')+['main.py']]"` parses clean.
- [ ] `python main.py --help` lists `--mode {hardware,gazebo}` and no `--sim`.
- [ ] Import smoke test: `python -c "import main"` (with deps installed) raises no ImportError.
- [ ] `README.md` documents only the two modes and the networked-Gazebo setup.
- [ ] Line count of the merged vision layer is meaningfully smaller than `vision.py + vision_gazebo.py` combined (duplication is gone).

---

## 9. Out of scope (do NOT do)

- No new tracking features, no model retraining, no PID retuning.
- Don't change the Kalman math, PID math, or MAVLink message masks.
- Don't add ROS2.
- Don't try to run Gazebo or fly hardware from inside the refactor — this is a code-structure task;
  validation is static (parse/import/grep) plus the manual smoke tests above.
```
