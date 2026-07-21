# specter-sense

Privacy-first spatial awareness for LLM-powered home assistants.

`specter-sense` turns Kinect V2 depth frames into compact local room-zone occupancy state. Raw camera frames remain in memory and are not persisted. The core service stops at depth acquisition, room-coordinate foreground detection, configured zones, and local state publication.

## Current scope

- Native C++20 depth pipeline using `libfreenect2`
- Explicit camera-to-room metric transform
- Stable zones defined as floor polygons with height bounds
- Slowly adapting per-pixel background model
- Separate enter/exit evidence thresholds and time hysteresis
- Boolean occupancy, evidence score, metric centroid, and range bounds
- Anonymous multi-object clustering with ephemeral process-local track IDs
- Geometry-only human/animal/object/unknown classification and coarse posture
- Atomic current-state JSON and structured operational logs
- Bounded sensor reconnect backoff
- Synthetic source and hardware-independent core tests

Home Assistant, MQTT, dashboards, skeletal pose inference, persistent identities, biometrics, furniture reconstruction, and distributed processing are deliberately out of scope. Track IDs are anonymous and expire on sensor reset or process restart.

## Build

The local source build of libfreenect2 can be used directly:

```sh
cmake -S . -B build \
  -DFREENECT2_ROOT=/home/alu52/libfreenect2 \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
ctest --test-dir build --output-on-failure
```

The optional calibration executable also requires the Debian `libglfw3-dev` and OpenGL development packages. If they are absent, CMake reports that condition and still builds the dependency-light headless service.

`FREENECT2_ROOT` must contain `include/libfreenect2` and either `lib`, `build/lib`, or an otherwise discoverable `libfreenect2` library. If it is omitted and the library cannot be found, CMake still builds a synthetic-only binary.

When using an uninstalled source build, its shared library may need to be exposed at runtime:

```sh
export LD_LIBRARY_PATH=/home/alu52/libfreenect2/build/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
```

## Run without hardware

The synthetic scene warms up with an empty background, introduces a nearer rectangular object, and then removes it. It exercises the same processing and publication path as the Kinect source:

```sh
cp .env.example .env
./build/specter-sense --frames 120
```

The active source, room configuration, socket, and optional state output live in the ignored `.env` file. Command-line options remain available for one-off overrides and take precedence over `.env`. Set `SPECTER_SENSE_ENV=/path/to/file` to use a different environment file. Omit `--frames` to run until `SIGINT` or `SIGTERM`.

## Run with Kinect V2

```sh
cp config/specter-sense.example.json config/specter-sense.local.json
cp .env.example .env
# Edit .env: select kinect and config/specter-sense.local.json.
./build/specter-sense
```

Only the depth stream is requested. Startup and reconnect events go to stderr as one-line JSON. After three consecutive frame timeouts the device is reopened with exponential backoff capped at 32 seconds. Socket state—and the optional state file when enabled—reports timeout, stale, and reconnect conditions instead of retaining a silently healthy sensor state.

## Room coordinates and calibration

Persisted zones are metric room geometry, never perspective-bound image masks. Coordinates use metres with `x` and `y` on the floor and `z` upward. `camera_to_room` is a row-major 4x4 affine calibration transform:

```text
[ r00 r01 r02 tx
  r10 r11 r12 ty
  r20 r21 r22 tz
    0   0   0  1 ]
```

Depth pixels are first deprojected using the Kinect IR intrinsics, producing right-handed camera coordinates (`x` right, `y` down, `z` forward), and then multiplied by this transform. The calibration viewport mirrors its horizontal screen projection without reflecting the underlying room geometry, so point-cloud display, zone picking, and gizmos agree while runtime coordinates remain well-formed. The example transform maps camera forward to room `y`, camera up to room `z`, and places the camera one metre above the floor.

For an initial hand calibration:

1. Measure the camera height and its position relative to the room origin.
2. Measure its yaw and downward/upward pitch; encode those rotations and translation in `camera_to_room`.
3. Define each zone as a floor polygon in room `(x, y)` coordinates plus `min_height_m` and `max_height_m`.
4. Run against recorded/synthetic or live depth and tune the transform before tuning occupancy thresholds.

The interactive tool below provides automatic floor-plane fitting plus manual correction. Its saved transform uses this same canonical configuration contract.

## Interactive calibration tool

`specter-sense-calibrate` is a separate GLFW/OpenGL utility. It reuses the service's Kinect source, depth deprojection, room transform, configuration validation, occupancy pipeline, and atomic config writer. The headless executable does not depend on the calibrator at runtime.

```sh
cp config/specter-sense.example.json config/specter-sense.local.json
cp .env.example .env
# Point SPECTER_SENSE_CONFIG at the local JSON and select the desired source.
LD_LIBRARY_PATH=/home/alu52/libfreenect2/build/lib \
  ./build/specter-sense-calibrate
```

Use `--source synthetic` to learn the editor without the camera. The utility never requests RGB and does not persist depth frames. The side panel can freeze a representative depth point cloud in memory while geometry is edited.

Object tracking visualization is optional and off by default in the calibrator. Click **Object Tracking Off** in the side panel to enable it, or start with `--object-tracking`. The overlay uses the same runtime clustering, classification, association, and posture pipeline: confirmed tracks appear as colored room-space bounds labeled with their ephemeral ID, class, and posture; coasting tracks remain visible with thinner bounds. Toggling the overlay clears its ephemeral tracks but preserves the learned foreground background, so tracking can restart without another warmup period. This display-only toggle does not change the saved `tracking.enabled` configuration value.

Bounded ignore planes mark reflective room surfaces whose depth rays are untrustworthy. Press `Ctrl+M` to create a one-metre-wide, 1.5-metre-tall plane at the current view target, initially vertical and facing the calibrated camera. Ignore planes are edited like zones but remain rigid rectangles: click the center and drag an X/Y/Z gizmo to move the plane, or click a corner and drag to resize it within its current plane. The panel provides rename, copy, delete, enable/disable, margin, and five-degree yaw/pitch controls. Planes render as translucent warning surfaces with an outline, corner handles, normal indicator, label, and live rejected-point count. With validation enabled, rejected returns appear magenta and disappear from foreground evidence and anonymous tracks before saving.

The editor has three view modes. **3D** is the free orbiting room view, **Top** is the orthographic floor-plan view, and **Camera** uses the current depth frame's exact Kinect intrinsics (`fx`, `fy`, `cx`, `cy`, width, and height) with letterboxing as needed. Camera view therefore matches depth-image pixel/ray coordinates rather than the horizontally mirrored presentation used by the free 3D viewport. In 3D and Top views, the calibrated Kinect is rendered as a physical wireframe body with local axes, optical direction, real depth frustum, and a `KINECT DEPTH CAMERA` label; clicking it switches to Camera view. Start directly in that mode with `--camera-view`.

In Camera view, the authored ignore surface outline and its expanded effective margin are rendered separately. Dragging an ignore-surface corner casts a ray through the corresponding depth pixel and intersects the existing room-space plane, allowing screen-space resizing without losing the plane's physical depth or coplanarity. Whole-plane depth/translation and orientation remain explicit 3D gizmo or panel operations because a single camera pixel cannot determine depth.

Suggested bedroom-mapping workflow:

1. Aim the Kinect so a useful patch of floor and the relevant occupied volumes are visible.
2. Click **Freeze Frame**, then **Estimate Floor**. Inspect the colored room axes and point cloud.
3. Press `Ctrl+N`, the editor's only command shortcut, to create a one-metre-square, two-metre-high bounding box at the current view target.
4. Click any top or bottom corner. Drag the red, green, or blue gizmo axis to change that corner's room X, Y, or Z coordinate. Moving a top/bottom corner on Z adjusts the corresponding height plane.
5. Use the panel to switch perspective/top-down views, rename, duplicate, delete, select, undo, and redo. In top-down mode, drag inside a selected zone to translate the whole footprint.
6. Click **Validation** to overlay live foreground evidence and display occupancy. Tune evidence thresholds and delays with the panel's minus/plus controls.
7. Click **Review and Save**. Inspect the destination and structured change preview, then click the atomic replacement button or cancel.

Viewport and editing controls:

- Left-drag empty space: orbit in perspective
- Click a corner, then drag the red/green/blue X/Y/Z gizmo
- Drag inside a selected top-down polygon: translate the entire zone
- Right-drag: pan; mouse wheel: zoom
- `Ctrl+N`: create a new bounding box
- `Ctrl+M`: create a new bounded ignore plane facing the calibrated camera
- **Object Tracking On/Off**: toggle anonymous track bounds and labels without changing saved configuration
- **3D / Top / Camera**: switch among free room editing, floor-plan editing, and the Kinect's horizontally mirrored depth projection
- Click the rendered Kinect in 3D or Top view: enter exact Camera view
- Mouse wheel in Camera view: zoom around the optical axis; the outlined sensor FOV remains visible as a reference
- Every other editor action is a clickable panel control

The editor rejects duplicate names, too-small or self-intersecting polygons, adjacent duplicate vertices, reversed/unreasonable height bounds, and coordinates outside ±50 metres before saving.

## Processing configuration

See [config/specter-sense.example.json](config/specter-sense.example.json).

- `min_depth_m`, `max_depth_m`: reject invalid or irrelevant measurements.
- `foreground_delta_m`: a valid sample must be this much nearer than its background depth.
- `background_alpha`: adaptation rate for non-foreground background samples.
- `warmup_frames`: background-learning frames before foreground evidence is emitted.
- `enter_points`, `exit_points`: Schmitt-trigger evidence thresholds; enter must be at least exit.
- `enter_after_ms`, `exit_after_ms`: sustained evidence/absence required for a state transition. A longer exit delay prevents flicker.

The optional top-level `tracking` object controls anonymous object tracking:

- `enabled`: enable clustering, association, classification, and coarse posture.
- `voxel_size_m`: room-space clustering resolution. Smaller values preserve detail but are more sensitive to holes.
- `min_cluster_points`: discard foreground components below this raw-point count.
- `association_max_distance_m`: maximum predicted-centroid displacement allowed when matching a track.
- `confirmation_frames`: observations required before a tentative track is published; also supplies posture hysteresis.
- `max_missed_frames`: bounded disappearance window before a track expires.

When `tracking` is absent, the documented defaults are used. Classification and posture confidence values express strength of geometric evidence, not calibrated probabilities. `unknown` is an expected result for partial views, merged objects, ambiguous sitting/crouching geometry, and shapes outside the conservative rules.

The optional top-level `ignore_planes` array contains rigid bounded rectangles in room coordinates:

- `name`: stable unique plane name, such as `wardrobe_mirror`.
- `enabled`: whether the plane participates in depth rejection.
- `corners_m`: four ordered `{x,y,z}` corners forming a nondegenerate rectangle.
- `margin_m`: bounded expansion beyond each edge, from 0 to 1 metre.
- `surface_tolerance_m`: distance before the mathematical surface also treated as untrustworthy, from 0 to 0.2 metre.
- `noise_threshold_points` (optional): reject matching plane returns only when their per-frame count is at or below this value. Omit it to reject every matching return, preserving the original behavior.

For each frame geometry, the service precomputes the nearest enabled plane intersection for every camera pixel. A valid depth return geometrically matches when its camera ray intersects the bounded plane and the return lies on or behind that surface. A plane without `noise_threshold_points` rejects all matches. A thresholded plane first learns its normal image-space background, then counts only foreground changes among its geometric matches. It rejects those foreground samples while their count is small enough to be noise; once activity exceeds the threshold, the whole foreground set passes through so real-object geometry is not partially erased. Static wall or room returns behind the plane do not inflate the sensitivity count. Rejected values never contribute to background adaptation, occupancy, foreground point clouds, clustering, tracking, classification, or posture. Changing ignore-plane geometry invalidates the ray lookup without discarding the learned image-space background. Existing configurations without `ignore_planes` remain valid and behave as an empty list.

Thresholds are sensor-resolution and scene dependent. Tune them from observations in the real room rather than treating the example values as universal.

## Live local API

The primary integration interface is a permission-controlled Unix domain socket. By default the service listens at:

```text
$XDG_RUNTIME_DIR/specter-sense.sock
```

If `XDG_RUNTIME_DIR` is unavailable, it falls back to `/tmp/specter-sense-<uid>.sock`. The socket is created with mode `0600`, supports multiple simultaneous clients, detects and removes stale socket files, and refuses to replace an active listener or a non-socket filesystem entry.

Each client receives newline-delimited JSON. Connecting immediately produces a complete snapshot:

```json
{"type":"snapshot","sequence":12,"state":{"schema_version":1,"sensor":{},"zones":{},"tracks":{}}}
```

The service then emits complete state envelopes with a reason:

```json
{"type":"state","sequence":13,"reason":"occupancy_changed","state":{}}
{"type":"state","sequence":14,"reason":"health_changed","state":{}}
{"type":"state","sequence":15,"reason":"observation","state":{}}
```

Occupancy and health transitions publish immediately. Observation messages default to 10 Hz so consumers receive current evidence, centroid, and range data without tying updates to disk writes. `sequence` increases monotonically for the lifetime of the process. Clients should reconnect after EOF and treat the next `snapshot` as authoritative.

`tracks` is an additive schema-v1 field keyed by an ephemeral ID such as `track-3`. Each item reports `tracking_state` (`confirmed` or `coasting`), conservative classification and posture labels with confidence, room-space centroid/velocity/bounds, foreground evidence, intersected zone names, occlusion state, and freshness. IDs are meaningful only during the current uninterrupted sensor session and must never be treated as a person identity. Existing v1 consumers may ignore this field.

The additive `ignore_planes` state object reports each configured plane's enabled state, raw geometric `matched_points`, foreground `activity_points`, `rejected_points`, and nullable `noise_threshold_points` in the latest valid frame. Activity monitors compare `activity_points` with the threshold to distinguish noise suppression from object pass-through. It contains derived counters and configuration only—never image or depth-frame data.

Inspect the stream from a terminal:

```sh
socat -u UNIX-CONNECT:"$XDG_RUNTIME_DIR/specter-sense.sock" -
```

When `XDG_RUNTIME_DIR` is unavailable, connect to `/tmp/specter-sense-$(id -u).sock` instead. Override behavior with:

```text
--socket PATH
--socket-interval-ms 100
--no-socket
```

Slow or abandoned consumers are disconnected once their pending output exceeds 1 MiB, preventing them from stalling depth processing.

## User service and tray indicator

After building, generate relocatable-to-this-checkout user units:

```sh
./scripts/systemd-user-service.sh
```

The generated files land in `build/systemd/` for review. To install and start both the sensor and its tray indicator:

```sh
./scripts/systemd-user-service.sh --install
```

The helper creates `.env` from `.env.example` when it is absent, installs only into the current user's systemd directory, reloads the user manager, and enables both units. Use `--no-start` to enable without starting them yet. Re-run the helper after moving the checkout because the generated units contain absolute paths.

The tray icon reads the authenticated Unix socket and shows four states: green for streaming with clear zones, blue when one or more zones are occupied, orange for a timeout/reconnect problem, and gray when the service is stopped. Left-click toggles `specter-sense.service`; the menu provides the same start/stop action. It requires Python 3 and PyQt6, and uses Qt's native system-tray/StatusNotifier integration on Plasma and other supported desktops.

The enabled tray unit belongs to the user manager's `default.target`, not to
the sensor service. Stopping the sensor from the tray therefore leaves the tray
controller running, and an enabled tray controller is started again when the
user manager starts. It waits until the graphical session exposes a system tray
before showing the icon.

Useful service commands:

```sh
systemctl --user status specter-sense.service
systemctl --user restart specter-sense.service
journalctl --user -u specter-sense.service -f
```

## Optional state file

The service does not write state to disk by default. Pass `--output PATH` when a human-readable last-known snapshot is useful; it is atomically replaced at most once per second instead of once per depth frame. `--no-state-file` explicitly disables it when wrapping an older command line. A snapshot resembles:

```json
{
  "schema_version": 1,
  "generated_at": "2026-07-19T20:00:00.000Z",
  "last_valid_frame_at": "2026-07-19T20:00:00.000Z",
  "sensor": {
    "connected": true,
    "reconnecting": false,
    "status": "streaming"
  },
  "zones": {
    "desk": {
      "occupied": true,
      "occupancy_score": 0.87,
      "foreground_points": 1234,
      "observed_at": "2026-07-19T20:00:00.000Z",
      "age_ms": 0,
      "nearest_range_m": 1.18,
      "farthest_range_m": 1.91,
      "centroid_m": {"x": 0.42, "y": 1.55, "z": 0.83}
    }
  },
  "tracks": {
    "track-3": {
      "tracking_state": "confirmed",
      "classification": "likely_human",
      "classification_confidence": 0.82,
      "posture": "standing",
      "posture_confidence": 0.76,
      "centroid_m": {"x": 0.4, "y": 1.8, "z": 0.9},
      "velocity_mps": {"x": 0.1, "y": 0.0, "z": 0.0},
      "bounds_m": {"width": 0.55, "depth": 0.38, "height": 1.71},
      "foreground_points": 1320,
      "zones": ["Futon"],
      "occluded": false,
      "observed_at": "2026-07-20T20:00:00.000Z",
      "age_ms": 0
    }
  }
}
```

`occupancy_score` is explicitly not a probability. It is the observed foreground-point count divided by `enter_points`, capped at 1. Range values are camera ranges; centroids, velocities, and bounds are in room coordinates/metres. Consumers must check sensor health and freshness rather than trusting stale zone or track state.

## Privacy and debug policy

The product is derived occupancy, anonymous tracking, and non-image ignore-plane diagnostic state. No code path currently writes RGB, IR, or depth frames. Track state contains geometry and motion only, and process-local IDs are cleared on sensor reset. Ignore planes reject untrusted rays; they never reconstruct, correct, or persist reflected depth geometry. Any future frame capture, replay recording, biometric classification, or cross-session identity feature must be explicit, visibly enabled, and separately reviewed.
