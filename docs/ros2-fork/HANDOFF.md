# Kinefly → ROS 2: context and plan

Handoff from the Claude Code session of 2026-09-22 to 2026-10-21, for starting the ROS 2 work in a fresh session.
Written 2026-10-19; updated 2026-10-21 with the camera facts, the camera tests and the camera-path decision.

## 1. Goal and decisions

**Goal:** Kinefly tracks a live camera in real time and sends its results to other systems through ROS 2.

**Decisions** (team, 2026-10-19):
- **Priorities:** live and real-time tracking come first. Offline analysis of recordings must stay possible but is secondary.
- **Base for the ROS 2 version:** a fork of **elhananby/kinefly** (https://github.com/elhananby/kinefly), branch `modernize-imaging`.
  - It's a pure Python 3 rewrite with a package layout, a camera abstraction, an event bus, plugins and 168 tests.
  - Its vectorized image-processing functions are kept.
  - Build on it with a ROS 2 image subscription for input, plus ROS 2 topics and rosbag2 for output.
- **Our standalone version stays as the offline tool,** and as the **reference ("oracle")** the fork is tested against.
  - Repo: `C:\Users\labadmin\Documents\GitHub\reiserlab\Kinefly`, branch `standalone-video`.
- **GUI toolkit: PyQt6, not PySide6** (Isabel, 2026-10-19). The ROS 2 environment already ships PyQt6 6.8.1 and `python_qt_binding` (used by rqt), and two Qt bindings in one process clash. Porting the fork is small:
  - Files: `src/kinefly/app.py`, `gui/window.py`, `gui/widgets.py`, `tests/gui/test_window.py`, and the `PySide6` dependency in `pyproject.toml`.
  - Mostly `PySide6.*` → `PyQt6.*` and `Signal` → `pyqtSignal`.
  - Licensing: PyQt6 is GPL (or commercial), where PySide6 is LGPL. That's fine for in-lab use, but it matters if the software is ever distributed.
- **No cv_bridge** (Isabel, 2026-10-19). Convert `sensor_msgs/Image` ↔ numpy with the small tested helper in §7. Reasons:
  - It isn't packaged for this ROS 2 build. Building `vision_opencv` on Windows needs Boost.Python, which isn't in the toolchain.
  - It would tie Kinefly to the ROS environment's OpenCV build.
  - It gives no speed benefit for mono8.
  - Reconsider if a C++ node (for example a camera driver in the same process) needs it.
- **Camera images are mono8** (for now). The helper accepts only mono8/8UC1 and rejects anything else loudly. If the camera changes to Bayer or color, add those encodings; `cv2.cvtColor` handles Bayer → gray, so cv_bridge still isn't needed.
- **Wings use the original edge trackers, not area trackers.**
  - Head = area, abdomen = tip, left/right = edge, as in `params_example.yaml` and `params_walking.yaml` (the walking version uses `n_edges_max: 3`).
- **Camera path: BIAS owns the camera, and a new BIAS plugin passes frames to Kinefly through Windows shared memory** (Isabel, 2026-10-21; plan in §5, Camera path).
  - BIAS stays the only program connected to the camera, and keeps recording `.ufmf` of every experiment.
  - Kinefly reads frames from shared memory in its own process, tracks them, and publishes results to ROS 2.
  - Each frame carries the camera's frame ID and timestamp, so live results can be matched to the recorded `.ufmf` frame by frame.
  - Alternatives considered (§2): Kinefly owning the camera through Arena would lose BIAS's `.ufmf` recording. Multicast to a listener on the same PC failed in testing. Multicast to a second PC is untested and needs more hardware.

## 2. The camera (the real target)

### The camera

**Lucid Triton2 1.6 MP, Sony IMX273, mono: TRT016S-M.** Product page: https://thinklucid.com/product/triton2-16-mp-imx273/

Read from the camera on 2026-10-21:

| Item | Value |
|---|---|
| Serial | 262101398 |
| Firmware | 1.62.0.0 |
| Sensor | 1440×1080, global shutter, 3.45 µm pixels, 12-bit ADC |
| Link | 2.5GBASE-T on **Ethernet 4** (Realtek 2.5GbE, jumbo frames 9014 enabled on the NIC). Link-local addresses: camera 169.254.229.179, PC 169.254.168.2. |
| Pixel format | Mono8 (also Mono10/10p/12/12p/16) |
| Packet size | `GevSCPSPacketSize` = 1400. Full-rate streaming worked at this size with no losses. |
| Exposure | `ExposureAuto` = Continuous, `ExposureTime` max 5953.6 µs at full rate (19.6 µs minimum) |
| Frame rate | `AcquisitionFrameRateEnable` = False; the `AcquisitionFrameRate` node max is 166.306 |
| Trigger | Off |
| PTP | Off (available) |
| Other features (spec page) | 128 MB image buffer; binning, decimation, ROI, flips; chunk data with timestamp; hardware trigger I/O |

**The camera actually runs at 168.4 fps.**
- **Free-running:** with `AcquisitionFrameRateEnable` False, the camera runs at the sensor maximum. Measured from its own timestamps: **168.416 fps, 5.938 ms between frames.** That's also exactly the rate of the .avi test recording.
- **The 166.3 fps on the spec page** is the maximum frame-rate setting, which reserves 10% of the link for resending packets. BIAS turns the frame-rate limit on when it connects (§8), so under BIAS the camera should run at the configured rate, at most 166.3 fps. Confirm the rate BIAS actually gets in step C0 (§5).

| Quantity | At 166.3 fps | At 168.4 fps (free-running) |
|---|---|---|
| Frame period | 6.01 ms | 5.94 ms |
| Data rate (mono8, 1.56 MB/frame) | 259 MB/s (2.07 Gb/s) | 262 MB/s (2.10 Gb/s) |
| Wingbeat estimate (180–220 Hz band) | Valid; needs 146.7–180 fps | Valid; needs 146.7–180 fps |

Both data rates need the 2.5 Gb/s link; they won't fit 1 GbE.

### Tracking cost

Measured on this PC at 1440×1080 with the real handle file (`oracle_handles_1440x1080.yaml`, wing outer radius about 450 px). Milliseconds of tracking per frame, not counting decoding or getting the frame:

| Implementation | First frame | After that (mean / 95th percentile) | Keeps up with ~6 ms frames? |
|---|---|---|---|
| Rewrite, full resolution | 68 | 13.5 / 14.6 | **No** (≈74 fps max) |
| Standalone, full resolution | 2672 | 15.4 / 16.7 | No |
| Standalone, `--scale 0.5` | 644 | 4.2 / 4.8 | Yes, with ~1.2–1.8 ms to spare |

So the ROS 2 version must reduce the work per frame. Options:
- implement the rewrite's (currently ignored) `scale_image`;
- crop to a region of interest;
- use smaller tracking regions;
- bin on the camera. That would also shrink BIAS's recordings, since BIAS records the same frames.

Copying a frame out of the camera driver into numpy costs another 0.5 ms (measured). Re-measure on flying-fly footage, since handle sizes drive the cost.

### Camera access tests (2026-10-21)

Run with Lucid's Python API, `arena_api` 2.9.5 on Arena SDK 1.0.80.56, at full resolution and rate, 1440×1080 Mono8. Each test copied every frame into numpy. Scripts are in `docs/ros2-fork/camera_tests/` (`mc_test.py`, `mc_analyze.py`, `mc_join.py`; see `arena_env.py` for setup).

| Test | Result |
|---|---|
| One receiver, no multicast, 10 s | **1685 / 1685 frames**, none incomplete. Python keeps up with the full stream. |
| Multicast on, controlling program only | **No frames.** The camera streamed to 239.10.10.10, but Windows never joined that group on Ethernet 4, so the NIC dropped the packets (0 multicast packets counted). |
| Same, with a helper socket joined to 239.10.10.10 (`mc_join.py`) | **1348 / 1348 frames**; 1.55 M multicast packets counted on the NIC. |
| Controlling program + read-only listener on the **same PC** (group joined) | Controller: 4208 of 4211 frames (3 missing and 1 incomplete when the listener started). **Listener: no frames.** |

Conclusions:
- **Same-PC multicast doesn't work as tested.** Lucid's filter driver seems to deliver the stream to only one receiver per PC. Arena has no setting to switch to ordinary sockets.
- **Untested:** disabling the Lucid filter driver on Ethernet 4. It's an admin-level change that also affects BIAS, and probably costs much more CPU at 2 Gb/s.
- **Multicast to a second PC** is the case Lucid documents. It needs a second PC with a 2.5 GbE port and an IGMP-capable switch. Not tested.
- **After the tests, the camera was back to normal single-receiver streaming,** with the settings it started with.

Practical notes:
- **Firewall:** a Python whose `python.exe` has no Windows Firewall inbound rule finds no camera. The pixi Pythons on this PC already have rules.
- **Other filter drivers:** three GigE Vision filter drivers are bound to Ethernet 4: Lucid, Teledyne DALSA Sapera and Point Grey. Disabling the other two on that port is worth trying if discovery or streaming misbehaves.
- **After a reboot, the camera took a while to answer discovery.**

## 3. Where things are

### Repos

- **Standalone (ours):** `C:\Users\labadmin\Documents\GitHub\reiserlab\Kinefly`, branch `standalone-video`.
  - Commits: `6f7c143` (port + .avi/.mp4/.ufmf), then Isabel's `02125bc` and `60c779f` (params files).
  - Run: `pixi run kinefly VIDEO` (see README → "Standalone video mode"), tests `pixi run test`, compare `pixi run compare A.csv B.csv`.
  - Key files: `nodes/kinefly.py` (CLI, GUI, run loop), `nodes/io_adapters.py` (VideoFileSource, UfmfSource, RealtimePlayer, CsvSink), `nodes/compare_runs.py`.
  - The tracking core (`fly.py`, `tracker_*.py`, `bodypart_*.py`, `imageprocessing.py`) is a line-for-line port of the 2016 code. Its algorithms are unchanged.
- **Rewrite (upstream of the fork):** https://github.com/elhananby/kinefly
  - `main` is the 2016 ROS code, the same base as ours.
  - `python3-migration` (Apr 2026) is the rewrite; `modernize-imaging` = that + vectorized imaging (`c5d053b`) + a macOS camera fix (`a793c3c`, branch head).
- **This handoff:** `docs/ros2-fork/` in the standalone repo.
  - `HANDOFF.md`: this file.
  - `run_fork_on_video.py`: runs the rewrite or fork offline on a video and writes a CSV in the standalone format.
  - `oracle_handles_1440x1080.yaml`: snapshot of the handle file used for all comparisons (`~/kinefly.yaml` as of 2026-10-13, for the 1440×1080 .mp4s).
  - `test_ros_image_helper.py`: tests for the §7 image helper.
  - `camera_tests/`: the Arena camera and multicast tests from §2.

### Test videos (`C:\Users\labadmin\Videos`)

| File | Format | Notes |
|---|---|---|
| `bias_video_cam_0_date_2026_08_25_time_14_10_14_v001a.mp4` / `v001b.mp4` | 1440×1080 H.264, 60.76 fps, 574 / 3733 frames | Headers overcount the frames (576 / 3872). The handle snapshot fits these. |
| `bias_video_cam_0_date_2026_09_10_time_11_28_21_v001.avi` | 1440×1080 MPEG-4, 168.4 fps, 5233 frames | |
| `bias_video_cam_0_date_2026_09_29_time_10_28_53_v001.ufmf` | 720×540 BIAS ufmf v4, 271.4 fps, 4801 frames | A ball, no fly; for testing real-time playback only. |

The .avi and .mp4 recordings show a tethered fly **walking with folded wings**, so wing angles barely move. Head angles from these are unreliable in both implementations: the half-scale and full-scale runs don't correlate.

### Software on this PC (Windows 11)

- **ROS 2 Lyrical:** `C:\dev\lyrical` (a pixi environment).
  - Contents: Python 3.12, numpy 1.26, OpenCV 4.10, rclpy, rosbag2, image_transport, PyQt6.
  - **No cv_bridge, by decision** (§1). Use the helper in §7.
  - To run Python with ROS 2 available (PowerShell): `cd C:\dev\lyrical; pixi run powershell -NoProfile -Command ". .\local_setup.ps1; python SCRIPT.py"`
  - A second workspace with build tooling is at `C:\pixi_ws\ros2-windows`.
- **BIAS (camera acquisition):** `C:\Users\labadmin\Documents\GitHub\reiserlab\bias`.
  - C++/Qt, MSYS2 ucrt64, Lucid Arena backend.
  - Launch without rebuilding: `build_and_run_bias.bat -n`.
  - Writes `.ufmf` v4 (format in `src/gui/video_writer_ufmf.cpp`). The plugin framework is in `src/plugin/`.
  - BIAS does **not** publish ROS 2. Details that matter for the camera path are in §8.
- **Lucid Arena SDK** 1.0.80.56: `C:\Program Files\Lucid Vision Labs\Arena SDK`.
  - C, C++ and .NET libraries are in `x64Release`; HTML docs and examples (including multicast) are in `docs\html`.
  - Python API: `arena_api` 2.9.5 at `C:\ProgramData\Lucid Vision Labs\ArenaView\ArenaPy\arena_api-2.9.5-py3-none-any.whl`. Not installed in any environment; it needs `pywin32`.
- **uv** (`~/.local/bin/uv`):
  - It defaults to Python 3.14, which breaks the rewrite's `harvester` extra (GenICam has no 3.14 wheels).
  - uv's own Python 3.12 link is broken here. Create the venv from an explicit interpreter:
    `uv venv -p C:/Users/labadmin/Documents/GitHub/reiserlab/Kinefly/.pixi/envs/default/python.exe .venv`
    `uv pip install -p .venv/Scripts/python.exe -e . pytest pyzmq msgpack pyserial matplotlib`
  - With that, the rewrite's tests give **168 passed, 2 skipped**.

## 4. What we know about the rewrite

### Verified by running both implementations on the same video and handles

- **Fully verified:** the fork was run end to end and compared frame by frame with the standalone version.
  - Setup: `…v001b.mp4`, 3733 frames, oracle handles, full resolution.
  - **Abdomen (tip), both wings (edge) and aux (intensity) are identical on every frame.**
  - **Head (area) is different** (head angle correlation −0.20).
- **The feathering bug** (§5 Phase 1). When feathering is 0, `WindowFunctions.create_tukey` substitutes a 0.25 taper; the original meant 0 to be "no window".
  - Only the area tracker reads the windowed polar image (`trackers/area.py:129`; original `tracker_area.py:89`). Edge and tip read the unwindowed image (`edge.py:257`, `tip.py:136`).
  - **With only the 0 / >1 rule fixed, the head also matched on all 3733 frames.**
- **The Tukey window has 32-bit rounding.**
  - `_tukey_1d` rounds each 1D window to float32 before `np.outer`; the original multiplies in float64 and rounds once.
  - Result: ~6e-8 differences at feathering 0.1–1.0 (up to 1444 pixels per window). Their tests pass only because they compare with `rtol=1e-5`.
  - The effect on tracking output is not measured. The area tracker's autozero could amplify it.
- **The vectorized polar transform matches the original loops** (as far as tested).
- **`scale_image` and `camera.realtime` are defined but never used.** Grep confirms only their definitions in `config/models.py`.
- **The GUI loop** (`app.py`) runs one frame per 16 ms `QTimer` tick, which caps it at about 60 fps. It has no frame queue, drop counting or display throttling.
- **Video playback** (`camera/videofile.py`) sleeps to the file's fps and never drops frames. The CLI always uses that paced mode.
- **No results file.** Output goes only to EventBus callbacks or ZeroMQ.

### From a line-by-line code review (not individually confirmed by running)

**Output-changing:**
1. **Auto invert-color never runs.** `Fly.set_params` is never called, so `ptBodyCenter_i` is never set and `fly.py:315-318` skips `get_invertcolor`.
   - Calling it would currently crash: `get_intersection` returns float64, used as slice indices.
   - Our port hit the same thing: the 2016 code made that first call through ROS dynamic_reconfigure at startup. We fixed it by calling `fly.set_params(...)` explicitly and casting the slice indices to int.
2. **Dragging a head or abdomen hinge updates only that tracker** (`app.py:193-204`). The wings keep the old body axis until restart. The original re-ran `fly.set_params` on every drag.
3. **Angle drags don't order lo < hi or add 2π** (`app.py:219-224`). The wedge can invert, flipping the sign of `anglePerPixel` / `angleOffset`.
   - Radius drags are unclamped. Equal radii make the rClip computation divide by zero (`base.py:585-591`).
   - `legalizeParams` is gone.
4. **Tracking keeps running during drags,** and every mouse move resets the area tracker's comparison image. The original paused tracking while dragging.
5. **Hinges are stored as int** (`app.py:213-215`); the original used float.
6. **The dt fallback is 1/60 s;** the original fell back to wall-clock time.
7. **`get_flystate` returns live tracker state objects** (`fly.py:348-352`). Asynchronous consumers (ZeroMQ, and soon ROS 2 publishers) can see later mutations. Copy at publish time.

**Configuration and UI:**
- `BodyPartConfig` defaults to tracker `area` and feathering 0.0 for every part (`config/models.py:137-141`). The original had abdomen = tip and feathering 0.25 except the head.
  - **Always set the wing tracker to `edge` explicitly in rig configs** until the default is fixed.
- The default handle layout differs; for example, the left/right wedges point inward.
- Symmetric mode is gone (a disabled checkbox).
- The aux ellipse can't be rotated.
- The background image isn't saved to disk.
- The Stabilize and combined LR-subtract buttons are gone.

## 5. Plan

### Phase 0: Set up the fork

1. Fork elhananby/kinefly on GitHub. Branch from `modernize-imaging` (`a793c3c`).
2. Clone it next to the standalone repo and create the venv as in §3. Confirm 168 passed, 2 skipped.
3. Run `docs/ros2-fork/run_fork_on_video.py` against the untouched fork (Phase 2 commands) and record the baseline: only head values should differ.

### Phase 1: Fix the feathering bug

In `src/kinefly/core/imaging.py`:
```python
def create_tukey(self, shape: tuple[int, int], feathering: float = 0.25) -> np.ndarray:
    (height, width) = shape
    alpha = feathering
    # As in the original: alpha <= 0 is rectangular (no taper), and so is alpha > 1.
    if (height <= 1) or (width <= 1) or (alpha <= 0.0) or (alpha > 1.0):
        return np.ones(shape, dtype=np.float32)
    # Form the product in float64 and round once, as the original loop does.
    return np.outer(_tukey_1d(height, alpha), _tukey_1d(width, alpha)).astype(np.float32)
```
Also make `_tukey_1d` return float64: drop its `.astype(np.float32)`, and check for other callers.

**Verified offline (2026-10-19):** this rule gives a window **bit-identical** to the 2016 loop for shapes (40,120), (17,63), (64,64), (2,2) and feathering 0, 0.1, 0.25, 0.5, 1.0, 1.5.

In `tests/core/test_imaging_modernized.py`:
- **Replace `_ref_create_tukey`** with a verbatim copy of the 2016 loop: `git show main:nodes/imageprocessing.py`, `WindowFunctions.create_tukey`. The current reference copies the rewrite's changed rule.
- **Compare with `np.testing.assert_array_equal`** (exact), not `rtol`.
- **Add feathering 0, 1.0 and 1.5** to the parametrized values. Include odd and non-square shapes.
- **Replace `test_zero_feathering_uses_default`** with a test that feathering 0 returns all ones. Its docstring "per original logic" is wrong.

How the bug came in, for the commit message or an upstream issue:
- `574bef1` dropped the parameter and hard-coded 0.25. The 2026-04-10 migration plan in their repo already had it hard-coded.
- `2da903d` re-added the parameter, but kept `feathering if feathering > 0 else 0.25`.
- The tests added in `c5d053b` copied that rule as the "original".
- The original's comment says "alpha==0 gives rectangular".

### Phase 2: Test the fork against the standalone version (the oracle)

Run both on the same video, handles and parameters, then compare frame by frame. Use separate processes, since both packages are named `kinefly`:
PowerShell, from the standalone repo:
```powershell
$v = "C:\Users\labadmin\Videos\bias_video_cam_0_date_2026_08_25_time_14_10_14_v001b.mp4"
Copy-Item docs\ros2-fork\oracle_handles_1440x1080.yaml $env:TEMP\h_ours.yaml
pixi run python nodes/kinefly.py $v --offline --no-gui --scale 1.0 --params params_example.yaml --gui-yaml $env:TEMP\h_ours.yaml --csv $env:TEMP\ours.csv
pixi run python docs/ros2-fork/run_fork_on_video.py $v --fork PATH_TO_FORK --params params_example.yaml --gui-yaml docs\ros2-fork\oracle_handles_1440x1080.yaml --csv $env:TEMP\fork.csv
pixi run compare $env:TEMP\ours.csv $env:TEMP\fork.csv
```
- Use a **copy** of the handle file for the standalone run, which saves handles on exit. `run_fork_on_video.py` doesn't modify its handle file.
- **Expected after Phase 1:** every value column identical (`max |diff|` 0), except `head_radius`/`abdomen_radius` differing by ≤ 5e-5 from CSV rounding. `aux_freq` should match too.
- **Before Phase 1 (baseline, measured on `v001a`):** head_angle mean |diff| 0.0003, max 0.006; everything else 0.
- Also run with `params_walking.yaml` (3 edges per wing) and on `v001a.mp4`.

**Make it a standing test in the fork:**
- Store the standalone version's CSV for a short clip (for example the first ~500 frames of `v001a.mp4`) plus the handle and params files as fixtures.
- Assert the fork reproduces it exactly.
- The tracking core is deterministic offline, so exact equality is the right bar. Any intended algorithm change must update the fixture deliberately.
- Rerun after each fix in Phase 3.

### Phase 3: Fix the other output-changing bugs

Work through §4 items 1–7, checking each against the oracle:
- **Item 1 (`Fly.set_params` never called):** affects invert-color and the axis tracker.
- **Items 2–3 (drag handling):** affect any session where handles are moved. The oracle can't see them, because they need GUI interaction; add unit tests for the param updates.
- **Item 7 (live state objects):** matters as soon as ROS 2 publishes asynchronously.
- **Rig defaults:** decide whether to restore the original ones (abdomen = tip; feathering 0.25 except the head; tracker defaults).

### Camera path: BIAS shared-memory plugin (decided 2026-10-21)

This runs in parallel with Phases 0–3: it's BIAS work plus a new Kinefly frame source, and it doesn't touch the tracking core. BIAS facts it depends on are in §8.

**C0. Test BIAS on its own at the camera's full rate first.** BIAS has only been verified at 720×540 and 271 fps, about 40% of this data rate.
1. Set the logging format to **ufmf**; BIAS defaults to JPG.
2. Record several minutes at 1440×1080 with the frame rate at its maximum.
3. Check for dropped frames. BIAS skips them silently, so look for gaps in the timestamps in `stamp_log_cam0.txt` larger than about 1.5 frame periods.
4. Check whether the ufmf writer reports skipped frames (its warning flag), and look at the CPU load.
5. Record the frame rate BIAS actually runs at (166.3 or 168.4 fps, see §2).
6. If BIAS can't keep up, fix that first. Candidates: its two copies per frame in the Arena backend, the per-frame `std::endl` flush of the stamp log, and the number of ufmf compressor threads.

**C1. Make BIAS frames traceable.**
- Add the camera's frame ID (`acImageGetFrameId`) to `StampedImage`, and keep the camera timestamp in full nanoseconds. Today BIAS uses its own frame counter and truncates the timestamp to microseconds.
- Write the frame ID to the stamp log.
- Count timeouts and incomplete frames instead of silently skipping them (`image_grabber.cpp:183-187`).

**C2. Write the shared-memory plugin (C++, inside BIAS, MinGW).**
- **Shared memory:** one Win32 named file mapping per camera (for example `Local\kinefly_bias_cam0`), plus a named auto-reset event (`Local\kinefly_bias_cam0_frame`) signalled once per frame.
- **Layout:** a header (magic, version, number of slots, width, height, step, pixel format, index of the newest slot) and a small ring of slots, for example 4. Each slot holds a sequence counter (odd while being written), the camera frame ID, the camera timestamp in ns, BIAS's frame count, and the mono8 pixels.
- **Writer:** `processFrames` writes each frame into the next slot, updates the newest-slot index, and signals the event. It must **never wait for the reader**:
  - If Kinefly is slow or not running, older slots are simply overwritten.
  - The plugin queue in BIAS is unbounded, so `processFrames` must stay fast. Copying 1.56 MB is well under a millisecond.
- **Registration:** BIAS registers plugins in five hard-coded places (§8).
- **Constraint:** BIAS runs only **one plugin per camera**, so this one can't run alongside `stampede` or `grab_detector`. If an experiment needs those, either fold their function into this plugin or change BIAS to allow several plugins.

**C3. Write the Kinefly side: a shared-memory frame source (Python).**
- **Reading:** open the mapping with `mmap.mmap(-1, size, tagname='Local\\kinefly_bias_cam0')`, and wait on the event through `ctypes`/`pywin32` (`OpenEventW`, `WaitForSingleObject`).
- **Each frame:** take the newest slot, copy the pixels out (≈0.5 ms), and accept the frame only if the slot's sequence counter didn't change during the copy. Then track it.
- **Drops:** count them from gaps in the camera frame ID. Latency is measured from the camera timestamp, which needs camera and PC clocks related: use PTP, or measure the offset at start-up.
- **Output:** Kinefly tracks in this process and publishes the `Flystate` message to ROS 2, so full images never have to go through ROS 2. Publish images only if something needs them (display, debugging), downscaled or at a lower rate.
- **Loop design:** this follows the standalone real-time design: newest frame only, display decoupled, drop and latency statistics.

**C4. Check live results against the recording.**
- Re-track the `.ufmf` that BIAS recorded during a live session, offline, and match frames by camera timestamp (or frame ID, after C1).
- Results should be identical for every frame the live run tracked. The remaining frames are the live drops.
- This works both as the end-to-end check and as the regular way to audit a session.

### Later phases (discussed, not yet planned in detail)

1. **Benchmark ROS 2 image throughput, only if full-rate images must go through ROS 2.** With the camera path above, Kinefly gets frames from shared memory, and only results go through ROS 2.
   - If full-rate images do become necessary: a dummy publisher at 1440×1080 mono8 at ~168 Hz (262 MB/s), and an rclpy subscriber doing the numpy conversion, then tracking.
   - Measure delivered rate, latency and drops.
   - If rclpy can't sustain it: intra-process or composable nodes, downscaling or cropping at the source, or loaned messages.
2. **Camera into ROS 2: decided.** BIAS with the shared-memory plugin; see Camera path above.
3. **Real-time frame loop:**
   - The newest frame (from shared memory in C3, or from an image subscription) drives tracking: process only the newest frame and count drops.
   - The display runs separately at about 30 Hz and never blocks tracking. Replace the rewrite's 16 ms QTimer-per-frame loop.
   - Standalone Kinefly already implements this design (`RealtimePlayer`, `get_frame`, `display_hz`, drop and latency stats in `nodes/kinefly.py`).
   - **Qt binding:** PyQt6 (decided, see §1); port the fork's GUI from PySide6.
   - **One OpenCV per environment:** the fork's `pyproject.toml` pulls PyPI `opencv-python`, but the ROS environment already has conda-forge OpenCV 4.10. Installing both gives two `cv2` modules. Use the ROS environment's OpenCV when running inside it. Its Qt-based `cv2.imshow` is slow on this PC (§6), so keep the main display in PyQt6.
4. **Make reduced resolution work:** implement `scale_image` in the fork (§2), and pass scaled handle params as the original `scale_params` does.
5. **Output:**
   - A small interfaces package with a `Flystate` message (head/abdomen/left/right/aux: angles, radii, gradients, intensity, freq + header stamp). Building it on Windows needs the `C:\pixi_ws` toolchain.
   - A publisher hooked to the EventBus. Copy the state (§4 item 7).
   - rosbag2 recording.
6. **Offline through ROS 2,** to eventually keep a single tracking core:
   - Replay a bag, or a small node that publishes a video or .ufmf as an image stream.
   - Reuse the standalone `UfmfSource` and `RealtimePlayer`, keeping frame-dropping playback.

## 6. Machine-specific gotchas (learned the hard way)

- **OpenCV window latency:** the conda-forge OpenCV's Qt window makes `cv2.waitKey(1)` and `cv2.pollKey()` block about 16 ms on this PC. That caused 20% dropped frames in real-time mode.
  - The PyPI `opencv-python` build (Win32 window) with `pollKey` takes about 1.2 ms.
  - The rewrite displays through PySide6 instead; measure its per-frame display cost too.
- **numpy's MKL build** crashed `np.linalg` (0xc06d007f, missing DLL) after switching OpenCV builds in pixi. Pin `libblas` to `*openblas`.
- **`time.sleep` oversleeps by about 0.55 ms.**
- **CPU power saving:** real-time runs measured 1.3–2.5× slower tracking per frame than offline runs of the same frames, most likely because the CPU clocks down between frames. Try a high-performance Windows power plan on the rig.
- **Startup stall in the original (and standalone) code:** building the polar transform and Tukey window with Python loops takes 80 ms to 2.7 s whenever handles change, dropping frames in real time. The rewrite's vectorized versions fix this (68 ms). Real-time code should still avoid rebuilding tables mid-stream.
- **Frame counts in .mp4 headers aren't reliable.** Read until decoding stops; standalone `VideoFileSource` does.

## 7. Image conversion helper (mono8, no cv_bridge)

Put this in the fork, for example `src/kinefly/ros/image.py`:
```python
import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image


def image_to_numpy(msg: Image) -> np.ndarray:
    """mono8 sensor_msgs/Image -> (height, width) uint8 array, without copying."""
    if msg.encoding not in ('mono8', '8UC1'):
        raise ValueError('unsupported encoding: %s' % msg.encoding)
    a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    return a[:, :msg.width]  # Drop any row padding.


def numpy_to_image(a: np.ndarray, stamp: Time, frame_id: str = 'camera') -> Image:
    """(height, width) uint8 array -> mono8 sensor_msgs/Image."""
    if (a.ndim != 2) or (a.dtype != np.uint8):
        raise ValueError('expected a 2D uint8 array, got %s %s' % (a.shape, a.dtype))
    msg = Image(height=a.shape[0], width=a.shape[1], encoding='mono8', is_bigendian=0, step=a.shape[1])
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.data = np.ascontiguousarray(a).tobytes()
    return msg
```

**Tested 2026-10-19** in the Lyrical environment with the real `sensor_msgs/Image` class (`docs/ros2-fork/test_ros_image_helper.py`).

| Check | Result |
|---|---|
| Round trip at 1440×1080 | Exact |
| Header stamp | Preserved |
| Cropped (non-contiguous) input | Works |
| Padded rows (`step > width`) | Cropped correctly |
| `bgr8` | Rejected |
| `image_to_numpy` | Zero-copy, about 0.001 ms/frame |
| `numpy_to_image` | 0.82 ms/frame |

`numpy_to_image` costs more because rclpy copies the bytes into its own `array('B')`. That's fine for test publishers at the camera's rate (frame budget 5.94–6.01 ms, §2). Optimize it only if a publisher becomes a bottleneck.

Things to keep in mind:
- **The array `image_to_numpy` returns is a view of the message's bytes.** rclpy creates a new message per callback, so keeping a frame after the callback is fine. Copy before modifying a frame in place if the same message is also published or recorded.
- **The tracking core needs a contiguous 2D uint8 array.** Padded images give a non-contiguous view; call `np.ascontiguousarray` there if a tracker or OpenCV function complains. The camera is expected to send unpadded rows (`step == width`).
- **Stamps:** pass `msg.header.stamp` to the tracker as seconds (`sec + nanosec*1e-9`). That's the camera's time, which is what `Fly.update(image, timestamp)` should get, not the time of arrival.

## 8. BIAS: what the camera path depends on

From a code review on 2026-10-21; nothing below was built or run. Paths are relative to `C:\Users\labadmin\Documents\GitHub\reiserlab\bias`.

**Frame path**
- **Frame type:** each frame is a `StampedImage` (`src/utility/stamped_image.hpp:8-14`): a `cv::Mat image`, a `double timeStamp` (camera time in seconds since the first frame), `dtEstimate`, and `frameCount`.
  - `frameCount` is **BIAS's own counter, not the camera's frame ID**, which BIAS never reads.
  - The camera timestamp comes from `acImageGetTimestampNs` but is kept with only microsecond precision (`camera_device_arena.cpp:431-445`).
- **Grabbing:** `CameraDevice_arena::grabImage` (`camera_device_arena.cpp:289-355`) waits up to 5 s for a buffer, converts it even when it's already Mono8, then copies it into a `cv::Mat`: **two copies per frame**.
  - The stream uses `StreamBufferHandlingMode=NewestOnly` (`:235-238`).
- **Silent drops:** incomplete or timed-out frames return an empty image, and the grabber just skips them (`image_grabber.cpp:183-187`). Dropped frames are visible only as timestamp gaps.
- **Threads:**
  - The grabber thread (`image_grabber.cpp:78-313`) runs at TimeCritical priority, pinned to one CPU core, and skips the first 10 frames.
  - The dispatcher (`image_dispatcher.cpp:106-189`) hands each frame to the logger, plugin and display. It also writes `stamp_log_cam<N>.txt`, flushing on every frame.
- **ufmf writer** (`video_writer_ufmf.cpp`):
  - 15 compressor threads by default.
  - Its to-do queue holds 250 frames; when full, frames are skipped and only a warning flag is raised.
  - If the logger's queue exceeds 1000 frames, capture stops with an error.
  - Mono8 only.
- **Display:** a timer at 15 Hz by default, 60 Hz at most.

**Plugins** (`src/plugin/`)
- **Base class:** `BiasPlugin` (`src/plugin/base/bias_plugin.hpp:22-89`) is a `QDialog`.
  - BIAS calls `processFrames(QList<StampedImage>)` from a thread-pool thread at normal priority (`src/gui/plugin_handler.cpp:106-158`).
  - **Every frame is delivered,** in batches if the plugin is slow.
  - The plugin queue is unbounded: `MAX_IMAGE_QUEUE_SIZE` is declared but unused.
  - Plugins also get `reset` (capture start), `stop`, a configuration map, and `runCmdFromMap`, which is reachable over the HTTP control interface.
- **One plugin per camera** (the "current" one). It must be enabled before capture starts. By default the current plugin is `grabDetector`, and plugins are disabled.
- **Registering a new plugin needs five edits:**
  - an `#include` in `src/gui/camera_window.cpp` (around `:57-59`);
  - a `pluginMap_[...] = new ...` in `camera_window.cpp` (around `:2608-2610`);
  - `include_directories` in `CMakeLists.txt:126-128`;
  - `add_subdirectory` in `CMakeLists.txt:207-210`;
  - its `.ui` file in `src/gui/CMakeLists.txt` FORMS (`:14-16`), plus `target_link_libraries` (`:107-112`).
  - Each plugin is a static library; copy the pattern in `src/plugin/stampede/`.
- **Existing plugins:**
  - `grab_detector`: ROI threshold, plus a serial pulse.
  - `stampede`: timed stimulus events over serial to a pulse device and a panels controller.
  - `signal_slot_demo`: passes frame data from camera 0 to camera 1 inside BIAS.
  - None of them sends frames to another process.

**External control**
- An HTTP/JSON server on port 5010 for camera 0 (`5000 + 10*(camera+1)`), on by default (`src/gui/ext_ctl_http_server.cpp`).
- Requests look like `GET /?cmd=value`. Commands include connect, start-capture, stop-capture, enable-logging, set-configuration and get-status, plus `plugin-cmd`, which passes JSON to the current plugin.
- Camera and logging settings can't be changed while capturing.
- Usage: `docs/building-on-windows.md` §4.

**Camera settings BIAS applies** (`camera_device_arena.cpp:158-177`, `:228-247`)
- `ExposureAuto` Off and `GainAuto` Off.
- **`AcquisitionFrameRateEnable` = true,** with the frame rate clamped to the node range (max 166.3 here).
- Free-run trigger by default; Line0 rising edge for an external trigger.
- Packet size auto-negotiation and packet resend on.
- Pixel format is never set; BIAS uses whatever the camera is set to (Mono8 now).
- ROI is supported; binning and decimation aren't.
- The camera is opened with read-write control and a normal single-receiver (unicast) stream. Multicast isn't supported.

**Toolchain**
- MSYS2 ucrt64 GCC 16.1 (MinGW), Qt 5.15.19 and OpenCV 4.13 from MSYS2. It links Arena's **C** API (`ArenaC_v140`), because the MSVC-built C++ API won't link with MinGW.
- **ROS 2 (MSVC) can't be linked into BIAS:**
  - the MinGW and MSVC C++ ABIs are incompatible;
  - BIAS builds as C++11 (`-std=gnu++0x`), but rclcpp needs C++17;
  - ROS 2's MSVC Qt and OpenCV DLLs would clash with BIAS's.
- That's why the camera path uses plain Win32 shared memory, which works across both toolchains.
- `docs/pixi-conversion-plan.md` in the BIAS repo scopes an MSVC port of BIAS. It hasn't been done.
