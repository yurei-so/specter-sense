# specter-sense

Privacy-first spatial awareness for LLM-powered home assistants.

`specter-sense` turns Kinect V2 depth frames into compact local room-zone occupancy state. Raw camera frames remain in memory and are not persisted. The first milestone intentionally stops at depth acquisition, room-coordinate foreground detection, configured zones, and atomic JSON publication.

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

Only the depth stream is requested. Startup and reconnect events go to stderr as one-line JSON. After three consecutive frame timeouts the device is reopened with exponential backoff capped at 32 seconds. The state file reports timeout, stale, and reconnect conditions instead of retaining a silently healthy sensor state.

## Room coordinates and calibration

Persisted zones are metric room geometry, never perspective-bound image masks. Coordinates use metres with `x` and `y` on the floor and `z` upward. `camera_to_room` is a row-major 4x4 rigid transform:

```text
[ r00 r01 r02 tx
  r10 r11 r12 ty
  r20 r21 r22 tz
    0   0   0  1 ]
```

Depth pixels are first deprojected using the Kinect IR intrinsics, producing camera coordinates (`x` right, `y` down, `z` forward), and then multiplied by this transform. The example transform maps camera forward to room `y`, camera up to room `z`, and places the camera one metre above the floor.

For an initial hand calibration:

1. Measure the camera height and its position relative to the room origin.
2. Measure its yaw and downward/upward pitch; encode those rotations and translation in `camera_to_room`.
3. Define each zone as a floor polygon in room `(x, y)` coordinates plus `min_height_m` and `max_height_m`.
4. Run against recorded/synthetic or live depth and tune the transform before tuning occupancy thresholds.

Automatic floor-plane fitting and a calibration UI are future work. The configuration contract already supports a floor-derived transform without changing zone definitions.

## Processing configuration

See [config/specter-sense.example.json](config/specter-sense.example.json).

- `min_depth_m`, `max_depth_m`: reject invalid or irrelevant measurements.
- `foreground_delta_m`: a valid sample must be this much nearer than its background depth.
- `background_alpha`: adaptation rate for non-foreground background samples.
- `warmup_frames`: background-learning frames before foreground evidence is emitted.
- `enter_points`, `exit_points`: Schmitt-trigger evidence thresholds; enter must be at least exit.
- `enter_after_ms`, `exit_after_ms`: sustained evidence/absence required for a state transition. A longer exit delay prevents flicker.

Thresholds are sensor-resolution and scene dependent. Tune them from observations in the real room rather than treating the example values as universal.

## State contract

The primary interface is one atomically replaced JSON file. A snapshot resembles:

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
