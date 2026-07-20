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
- Atomic current-state JSON and structured operational logs
- Bounded sensor reconnect backoff
- Synthetic source and hardware-independent core tests

Home Assistant, MQTT, dashboards, pose inference, identities, person counts, furniture reconstruction, and distributed processing are deliberately out of scope.

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
./build/specter-sense \
  --source synthetic \
  --frames 120 \
  --config config/specter-sense.example.json \
  --output state/specter-sense.json
```

Omit `--frames` to run until `SIGINT` or `SIGTERM`.

## Run with Kinect V2

```sh
./build/specter-sense \
  --source kinect \
  --config config/specter-sense.local.json \
  --output state/specter-sense.json
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
LD_LIBRARY_PATH=/home/alu52/libfreenect2/build/lib \
  ./build/specter-sense-calibrate \
  --source kinect \
  --config config/specter-sense.local.json
```

Use `--source synthetic` to learn the editor without the camera. The utility never requests RGB and does not persist depth frames. The side panel can freeze a representative depth point cloud in memory while geometry is edited.

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

Thresholds are sensor-resolution and scene dependent. Tune them from observations in the real room rather than treating the example values as universal.

## Live local API

The primary integration interface is a permission-controlled Unix domain socket. By default the service listens at:

```text
$XDG_RUNTIME_DIR/specter-sense.sock
```

If `XDG_RUNTIME_DIR` is unavailable, it falls back to `/tmp/specter-sense-<uid>.sock`. The socket is created with mode `0600`, supports multiple simultaneous clients, detects and removes stale socket files, and refuses to replace an active listener or a non-socket filesystem entry.

Each client receives newline-delimited JSON. Connecting immediately produces a complete snapshot:

```json
{"type":"snapshot","sequence":12,"state":{"schema_version":1,"sensor":{},"zones":{}}}
```

The service then emits complete state envelopes with a reason:

```json
{"type":"state","sequence":13,"reason":"occupancy_changed","state":{}}
{"type":"state","sequence":14,"reason":"health_changed","state":{}}
{"type":"state","sequence":15,"reason":"observation","state":{}}
```

Occupancy and health transitions publish immediately. Observation messages default to 10 Hz so consumers receive current evidence, centroid, and range data without tying updates to disk writes. `sequence` increases monotonically for the lifetime of the process. Clients should reconnect after EOF and treat the next `snapshot` as authoritative.

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
  }
}
```

`occupancy_score` is explicitly not a probability. It is the observed foreground-point count divided by `enter_points`, capped at 1. Range values are camera ranges; the centroid is in room coordinates. Consumers must check sensor health and freshness rather than trusting a stale `occupied` value.

## Privacy and debug policy

The product is derived occupancy state. No code path currently writes RGB, IR, or depth frames. Any future frame capture or replay-recording feature must be explicit, visibly enabled, and off by default.
