# AUV Localization Architecture

This document describes the localization architecture for the RACE AUV platform. It captures the current state, the path forward, and the rationale for the sensor-fusion split between `robot_localization`'s EKF and a new gtsam-based factor-graph node.

Scope: the production localization pipeline — IMU, DVL, pressure, AprilTags, USBL — and how each sensor's data reaches both the high-rate controller and the dock-relative pose estimator.

---

## 1. Context

The AUV platform runs on a Stonefish simulation and on real hardware. Both deployments need:

- **A high-rate state estimate** for `mvp_control`'s closed-loop control loops (200 Hz target, 20 Hz achievable).
- **A globally accurate, low-rate pose** for the dock-approach phase, where sub-decimeter accuracy is required relative to the docking station.
- **Runtime-togglable sensors** so the same pipeline operates with the full sensor stack in simulation, with a subset on a stripped-down AUV, and with graceful degradation when a sensor drops out.

The current pipeline (`apriltag_fuser_node` in `race_auv_sim_pkg`) implements a single snapshot joint Umeyama solve over per-camera tag detections. It is a deterministic least-squares fit per tick — no temporal smoothing, no IMU/DVL integration, no global positioning. It is sufficient for tag-only dock-relative pose but cannot extend to multi-sensor fusion.

This document defines the architecture that replaces the current fuser with a two-layer pipeline: a `robot_localization` EKF layer for high-rate inertial dead-reckoning, and a gtsam factor-graph layer for global corrections (tags, USBL). Each sensor is a toggleable factor.

---

## 2. Layered architecture

```
                     +--------------------------------+
                     |          mvp_control           |
                     |   (reads odometry/filtered)    |
                     +----------------+---------------+
                                      |
                                      v
+---------------------------------------------+    +--------------------------------+
|             robot_localization EKF          |    |          gtsam_dock_fuser      |
|                                             |    |                                |
|  IMU  -> imu0                              |    |  EKF output  -> BetweenFactor  |
|  DVL  -> twist0                            |    |  Tags        -> Projection     |
|  Pressure -> odom0 (z only)                |    |  USBL        -> PriorFactor    |
|                                             |    |  Pressure    -> PriorFactor    |
|  Output:                                    |    |       (optional cross-check)   |
|  - /odometry/filtered (nav_msgs/Odometry)   |    |                                |
|  - TF: odom -> base_link @ 20 Hz            |    |  Output:                       |
|                                             |    |  - /dock_point/pose            |
|  Function:                                  |    |  - TF: base_link -> dock_point |
|  Inertial prediction + dead reckoning.      |    |       @ 5 Hz                   |
|  Field-proven for AUV stacks.               |    |                                |
|  Drift bounded by DVL/IMU only.             |    |  Function:                     |
+----------------------------------+          |    |  Joint optimization across     |
                                   |          |    |  tag observations + USBL +     |
                                   |          |    |  EKF odometry. Globally        |
                                   |          |    |  consistent dock-relative pose.|
                                   |          |    +----------------+---------------+
                                   |          |                      |
                                   v          |                      v
                          +----------------+ |             +------------------+
                          |      odom      | |             |    dock_point    |
                          +-------+--------+ |             +--------+---------+
                                  |          |                      |
                                  |          |                      |
                          +-------v--------+ |                      |
                          |   base_link    +<+----------------------+
                          +----------------+
                                  ^
                                  |
                          +-------+--------+
                          |     world      |
                          +----------------+
```

The two layers publish into different TF edges and do not contend. The EKF owns `odom -> base_link` (high-rate, drives control). gtsam owns `base_link -> dock_point` (low-rate, drives docking). The TF tree composes them automatically.

---

## 3. Sensor-to-factor mapping

| Sensor topic | Producer | EKF input | gtsam factor | Update rate |
|---|---|---|---|---|
| IMU | Stonefish or hardware IMU driver | `imu0` (predict) | `ImuFactor` (preintegrated) | 200 Hz |
| DVL | Stonefish or DVL driver | `twist0` (correct) | `BetweenFactorPose3` (velocity · Δt) | 10 Hz |
| Pressure | Stonefish or pressure sensor | `odom0` (z only) | `PriorFactor<Pose3>` on z (optional sanity check) | 10 Hz |
| AprilTags | `sync_and_detect` (tagslam) | — | `GenericProjectionFactorCal3_S2` per corner | 5 Hz |
| USBL | USBL driver | `odom1` (optional) | `PriorFactor<Pose3>` on xyz | 1 Hz |

Key observations:

- **Pressure is owned by the EKF by default.** Adding it to gtsam as well is a cross-check, not a redundancy. The current code wires pressure into EKF only.
- **Tags are gtsam-only.** They require a factor graph (multi-corner reprojection across cameras) and have no representation in the EKF config.
- **USBL is gtsam-only in the first cut.** If global waypoints relative to the dock become a requirement later, USBL can also feed back into the EKF as `odom1`.
- **IMU and DVL are processed by both layers.** The EKF produces `odometry/filtered` (high-rate, fused). gtsam reads that output as a between-factor. The EKF does not need to know that gtsam is downstream.

---

## 4. The `gtsam_dock_fuser` node

A new ROS2 node in `race_auv_sim_pkg`. Owns the gtsam factor graph for the dock-relative pose.

### Inputs

| Topic | Type | Source |
|---|---|---|
| `odometry/filtered` | `nav_msgs/Odometry` | robot_localization EKF |
| `/cam_*/apriltag_detection/detections3d` | `apriltag_msgs/AprilTagDetectionArray` | `sync_and_detect` (tagslam) |
| `/usbl/odom` | `nav_msgs/Odometry` | USBL driver |
| `pressure` (optional) | sensor_msgs/FluidPressure | pressure driver |

### Outputs

| Topic | Type | Description |
|---|---|---|
| `/dock_point/pose` | `geometry_msgs/PoseStamped` | Optimized pose in `race_auv/base_link` |
| TF | `race_auv/base_link -> dock_point` | Optimized transform |

### Sensor toggle

Each sensor is enabled/disabled at runtime via a `set_bool` service or dynamic reconfigure:

```python
for sensor in ['ekf', 'tags', 'usbl', 'pressure']:
    if self._sensor_enabled(sensor) and self._sensor_alive(sensor):
        self._add_factors_for(sensor)
```

Mirrors the pattern used in [TURTLMap](https://github.com/umfieldrobotics/TURTLMap) (`sensor_list.isXUsed`) but expressed as a set of ROS params rather than a YAML block. The `enabled` flag is the user-facing switch; the `alive` flag auto-disables sensors whose topic has gone stale.

### Factor graph structure

```
For each keyframe K (triggered by tag arrival or fixed dt):

    State at K: X_K = (Pose3, velocity_3, accel_bias_3, gyro_bias_3)

    Between K-1 and K:
        ImuFactor(X_{K-1}, X_K, preintegrated_imu)              [if IMU streaming]
        BetweenFactorPose3(X_{K-1}, X_K, T_odom_Δ)                 [if EKF streaming]

    At K:
        PartialPriorFactorPose3(X_K, pressure_depth, z_only)      [if pressure enabled + alive]
        PriorFactorPose3(X_K, usbl_pose, xy+z)                    [if USBL enabled + alive]
        For each visible tag T in each camera C, for each corner of T:
            GenericProjectionFactorCal3_S2(
                X_K, T, corner_observed,
                T_camera_to_imu @ T_imu_to_tag,
                intrinsics_C)                                     [if tags enabled + alive]

    Anchor (first keyframe only):
        PriorFactorPose3(X_0, initial_pose, very_tight_noise)    [gauge-fixing]

Optimize with ISAM2 -> recover X_K.
Publish dock_point pose = X_K @ T_base_to_dock (URDF, not from gtsam).
```

### Pinned tools

- **gtsam 4.3a0** (already installed in this workspace). Python bindings include `PreintegratedImuMeasurements`, `ImuFactor`, `BetweenFactorPose3`, `PriorFactorPose3`, `GenericProjectionFactorCal3_S2`, `PinholeCameraCal3_S2`, `ISAM2`.
- **apriltag_msgs** + **sync_and_detect** from tagslam for time-synchronized multi-camera tag detection. The current `apriltag_detector_node` only emits `vision_msgs/Detection3DArray` with a single 6-DoF pose per tag — insufficient for corner-level reprojection. `sync_and_detect` emits `apriltag_msgs/AprilTagDetectionArray` with four corner pixels per tag, which is what `GenericProjectionFactorCal3_S2` consumes.
- **URDF `link_transform_from_base`** (already in `race_auv_sim_pkg/urdf_tag_parser.py`) for `T_base_to_dock_Point`. Single source of truth for the dock offset; recomputed at startup, never at runtime.

---

## 5. TF tree

```
world
  └─ odom                                          (static TF, set once)
       └─ base_link                                (EKF, 20 Hz)
            └─ dock_point                          (gtsam_dock_fuser, 5 Hz)
```

The EKF does not need to be updated by gtsam. The dock-relative pose is independent of the EKF's global drift. `mvp_control` reads `odom -> base_link` as today. A new behavior in `mvp_mission` does `lookupTransform(base_link, dock_point)` for the final approach.

---

## 6. Why this split

| Concern | EKF-only | gtsam-only | EKF + gtsam (chosen) |
|---|---|---|---|
| 200 Hz IMU prediction | Native | Slow (factor graph per cycle) | Native (EKF) |
| Control loop latency | <10 ms | >50 ms | <10 ms (EKF) |
| Tag corner reprojection | Coarse `T_cam_to_tag` only | Sub-pixel (4 corners) | Sub-pixel (gtsam) |
| USBL global corrections | Loose fusion | Tight (PriorFactor) | Tight (gtsam) |
| `mvp_control` compatibility | Unchanged | Requires new control-rate source | Unchanged |
| Runtime sensor toggling | Per-input config, restart needed | Native (gtsam factor enabled/disabled) | Native for gtsam path |
| Deployability | Live today | 3-4 weeks of code | 1-2 weeks of code |

The accuracy difference between EKF-only and the hybrid is small during EKF-only phases (DVL dead-reckoning is the same in both). The difference becomes significant during the dock approach. At that point, both tags and USBL are typically visible, and gtsam's joint optimization across them outperforms the EKF's sequential corrections.

---

## 7. Topics and frame conventions

### Topics

| Topic | Type | Owner | Notes |
|---|---|---|---|
| `odometry/filtered` | `nav_msgs/Odometry` | EKF | Existing. Drives `mvp_control`. |
| `/cam_*/apriltag_detection/detections3d` | `apriltag_msgs/AprilTagDetectionArray` | per-camera `sync_and_detect` | Crow's-foot pattern for multi-camera. Each detection carries 4 corner pixels. |
| `/usbl/odom` | `nav_msgs/Odometry` | USBL driver | Optional; node auto-disables when stale. |
| `/dock_point/pose` | `geometry_msgs/PoseStamped` | `gtsam_dock_fuser` | The new authoritative topic for dock-relative pose. |

### Frames

| Frame | Parent | Owner | Description |
|---|---|---|---|
| `world` | — | static TF | World frame |
| `odom` | `world` | static TF | Odom frame |
| `base_link` | `odom` | EKF | AUV body frame |
| `dock_point` | `base_link` | `gtsam_dock_fuser` | Docking station reference |

The station's URDF lives in `race_station_description/urdf/base.urdf`. The link `dock_point` is defined at `(-0.415, 0.33, -0.05)` relative to `base_link`. The `T_base_to_dock` offset is read from the URDF by `urdf_tag_parser.link_transform_from_base`, not duplicated in any config file.

---

## 8. Sensors in detail

### IMU

- Frequency: 200 Hz (predict), used in EKF.
- gtsam: `PreintegratedImuMeasurements` with kalibr-style noise parameters (`gyroscope_noise_density`, `accelerometer_noise_density`, `gyroscope_random_walk`, `accelerometer_random_walk`).
- Calibration: kalibr or per-IMU datasheet values.
- Disable: EKF puts `imu0` in inactive mode; gtsam skips `ImuFactor` between keyframes.

### DVL

- Frequency: 10 Hz.
- EKF input: `twist0` (linear velocity only, 3-DoF).
- gtsam input: `BetweenFactorPose3` with `T = Pose3(Exp(v · Δt))` between consecutive keyframes. Velocity is reported in the DVL's body frame; the robot's heading comes from the EKF or from gtsam's own orientation estimate.
- Disable: same as IMU.

### Pressure

- Frequency: 10 Hz.
- EKF input: `odom0` with `[false, false, true, false, ...]` (z-only).
- gtsam: optional `PartialPriorFactor<Pose3>` on z with high noise (sanity check against EKF).
- Disable: EKF skips the slot; gtsam skips the prior.

### AprilTags

- Tags: 4 × tag25h9 (21 cm) + 2 × tag36h11 (12.5 cm), defined in `race_station_description/urdf/base.urdf`. The 4 cm tag36h11 tags are present in the URDF but currently commented out.
- Detector: `apriltag_detector` plugin (`umich` or `mit`) loaded by `sync_and_detect` from tagslam. `sync_and_detect` is the canonical detector in this architecture; the legacy `apriltag_detector_node` in `race_auv_sim_pkg` is removed.
- Output: `apriltag_msgs/AprilTagDetectionArray` per camera, one element per tag with 4 corner pixels and the family/id.
- gtsam factor: `GenericProjectionFactorCal3_S2` per corner, per tag, per camera. With 6 tags visible across 2 cameras and 4 corners each, this is up to 48 residuals per keyframe — substantially more information than the current coarse `T_cam_to_tag` solve.
- Calibration: camera intrinsics come from `apriltag.yaml` (generated into `cameras.yaml` for `sync_and_detect`). Camera-to-IMU extrinsics come from the URDF (e.g., `race_auv/cam_front` chain).
- Disable: `gtsam_dock_fuser` skips `ProjectionFactor` initialization. The fuser still publishes based on EKF odometry alone.

**Breaking change:** any consumer reading the legacy `vision_msgs/Detection3DArray` topic must be updated. The only such consumer in the workspace today is `apriltag_fuser_node`, which is being replaced. A grep for `Detection3DArray` across all `race_auv_*` packages must be run before deployment to confirm there are no other stragglers.

### USBL

- Frequency: 1 Hz typical.
- gtsam factor: `PriorFactor<Pose3>` on full position (x, y, z) with reported covariance. If only xy is reliable, use a partial prior.
- Disable: same as pressure.

### Note on current pipeline

The current `apriltag_fuser_node` (in `race_auv_sim_pkg`) is being replaced by `gtsam_dock_fuser`. The current `apriltag_detector_node` is being replaced by `sync_and_detect` from tagslam. The current `vision_msgs/Detection3DArray` (single 6-DoF pose per tag) is being replaced by `apriltag_msgs/AprilTagDetectionArray` (4 corner pixels per tag). The single-output TF `race_auv/base_link -> dock_point` (already wired through the in-progress fuser changes) becomes the contract for the new node. The legacy `vision_msgs/Detection3DArray` topic is removed; no adapter is published. Any downstream consumer not updated to subscribe to `apriltag_msgs/AprilTagDetectionArray` will silently stop receiving tag data.

---

## 9. Sensor toggling in detail

The gtsam node reads a YAML/param file with this structure:

```yaml
gtsam_dock_fuser:
  ros__parameters:
    enabled_sensors:
      ekf:        true
      imu:        true
      dvl:        true
      pressure:   true
      tags:       true
      usbl:       false
    sensor_timeout: 0.5      # seconds; sensor is "alive" if it published within this window
    publish_rate:  5.0
    # noise model parameters
    ekf_odom_noise: 0.05
    imu_preintegration:
      gyro_noise_density: 8.73e-05
      accel_noise_density: 0.000245
      gyro_random_walk: 6.13e-04
      accel_random_walk: 6.20e-03
    usbl_position_noise: 1.0  # meters
    # tag config
    tag_urdf_package: race_station_description
    tag_urdf_filename: urdf/base.urdf
    dock_link_name:   dock_point
    tags_topics:      # one entry per camera, apriltag_msgs/AprilTagDetectionArray
      - /cam_front/apriltag_detection/detections3d
      - /cam_down/apriltag_detection/detections3d
    # frame names
    reference_frame: race_auv/base_link
    output_frame:    dock_point
    output_pose_topic: dock_point/pose
```

At runtime, setting `enabled_sensors.tags = false` skips tag factor initialization. Setting `enabled_sensors.usbl = true` adds a USBL prior. The node never crashes on a missing sensor; it simply runs the optimization with whatever factors are available.

If all sensors are disabled, the node still publishes the EKF odometry as a fallback (after applying the URDF `T_base_to_dock` offset). This guarantees the dock-point TF is always available, even if no global corrections are active.

---

## 10. Failover and degraded modes

| Scenario | Behavior |
|---|---|
| All sensors healthy | EKF + gtsam both running. Dock-point TF is gtsam-optimized. |
| Tags lose visibility | EKF keeps dead-reckoning. gtsam keeps running with EKF odometry + USBL + pressure. |
| DVL bottom-out (lost bottom lock) | EKF degrades to IMU-only. Pressure still corrects z. gtsam inherits the degraded EKF pose. |
| USBL drops out | gtsam runs on tags + EKF odometry only. |
| Pressure sensor fails | EKF drifts in z. gtsam inherits the drift. |
| All sensors off | gtsam publishes EKF pose + URDF offset as the dock-point TF. |
| gtsam node dies | The TF `base_link -> dock_point` is lost. mvp_control keeps running on EKF. `mvp_mission` cannot enter dock-approach mode. |

The architecture fails open: the controller never stops, the dock approach only works when both layers are alive.

---

## 11. Deployment plan

### Phase 1 — gtsam_dock_fuser skeleton (1 week)

1. Vendor `sync_and_detect` from tagslam (along with `apriltag_msgs` and `apriltag_detector` plugins).
2. Generate `cameras.yaml` and `tagslam.yaml` from `apriltag.yaml` + `base.urdf` using a small generator script.
3. Sketch `gtsam_dock_fuser` with EKF-input only (no tags, no USBL). Confirm the TF `base_link -> dock_point` is published with the URDF offset.
4. Verify in simulation: dock-point TF tracks the EKF pose plus the URDF offset.

### Phase 2 — Add tags (1 week)

1. Add `GenericProjectionFactorCal3_S2` per corner per tag per camera.
2. Subscribe to `apriltag_msgs/AprilTagDetectionArray` on the per-camera topics in `tags_topics`.
3. Verify in simulation: dock-point TF snaps to the true dock pose when tags are visible; falls back to EKF when tags leave.

### Phase 3 — Add USBL (2-3 days)

1. Add `PriorFactor<Pose3>` on USBL measurement.
2. Verify in simulation: dock-point TF ignores USBL when disabled, factors it in when enabled.

### Phase 4 — Add IMU preintegration (optional, 1 week)

1. Read raw IMU messages.
2. Build `PreintegratedImuMeasurements`, add `ImuFactor` between keyframes.
3. Compare to "EKF-only odometry" path. The marginal accuracy gain in the dock-approach phase is small because the EKF already integrates IMU/DVL tightly. Decide whether to keep the redundant preintegration based on the comparison.

### Phase 5 — Removal (1-2 days)

1. Delete `apriltag_fuser_node`, `apriltag_detector_node`, `apriltag.launch.py`, `apriltag.yaml` from `race_auv_sim_pkg`. Remove their entries from `setup.py`'s `console_scripts` and the package's `data_files`.
2. Search the workspace for any remaining subscriber to `vision_msgs/Detection3DArray` or any string `Detection3DArray`:
   ```bash
   rg "Detection3DArray" /home/dark3090/ros2_ws/src/race_auv
   rg "vision_msgs.msg.Detection3DArray" /home/dark3090/ros2_ws/src/race_auv
   ```
   For each hit, either remove the consumer or migrate it to subscribe to `apriltag_msgs/AprilTagDetectionArray` on the per-camera topic in `tags_topics`.
3. Update `bringup_simulation.launch.py` to launch `sync_and_detect.launch.py` (from tagslam) + `gtsam_dock_fuser` instead of the old `apriltag_pipeline`.
4. After deployment, confirm no `Detection3DArray` subscribers remain in any launched package: `ros2 node list`, then `ros2 node info <node>` for each node that handles imagery.

### Phase 6 — Optional: EKF feedback (when global waypoints relative to dock become a requirement)

1. Publish `/dock_point/pose` in `odom` coordinates via `lookupTransform(odom, base_link) @ lookupTransform(base_link, dock_point)`.
2. Add `odom1` to `robot_localization_sim.yaml` with the docked pose as a global correction.
3. Verify that mission waypoints defined in the dock frame remain consistent with the EKF state.

---

## 12. Verification

```bash
# Build
colcon build --packages-select race_auv_sim_pkg

# Confirm gtsam Python bindings are present
python3 -c "import gtsam; print(gtsam.PreintegratedImuMeasurements, gtsam.ISAM2, gtsam.GenericProjectionFactorCal3_S2)"

# Bring up simulation
ros2 launch race_auv_bringup bringup_simulation.launch.py

# Inspect the TF tree
ros2 run tf2_tools view_frames.py

# Confirm the dock-point TF is being published
ros2 run tf2_ros tf2_echo race_auv/base_link dock_point

# Confirm the dock-point pose topic
ros2 topic echo /dock_point/pose --once

# Toggle a sensor at runtime
ros2 param set /gtsam_dock_fuser enabled_sensors.tags false
ros2 param set /gtsam_dock_fuser enabled_sensors.tags true

# Kill the gtsam node and confirm the controller keeps running
ros2 lifecycle set /gtsam_dock_fuser shutdown
ros2 topic echo /odometry/filtered --once
```

The dock-point TF should be a pure translation of the EKF pose when no gtsam factors are active (because `T_base_to_dock` has rpy=0). It should snap to the true dock pose when tags are visible.

---

## 13. References

- **TURTLMap** (UMich Field Robotics, IROS 2024) — the `sensor_list` toggle pattern, `BarometerFactor`, `VelocityIntegrationFactor` design. <https://github.com/umfieldrobotics/TURTLMap>
- **TagSLAM** — `sync_and_detect` for multi-camera time-synchronized tag detection. <https://github.com/berndpfrommer/tagslam>
- **gtsam 4.3a0** — factor graph library, Python bindings include `PreintegratedImuMeasurements`, `ImuFactor`, `BetweenFactorPose3`, `PriorFactorPose3`, `GenericProjectionFactorCal3_S2`, `ISAM2`. <https://gtsam.org/>
- **robot_localization** — Tom Moore's EKF/UKF for ROS2, already wired for IMU + DVL + pressure in `race_auv_bringup`. <http://docs.ros.org/en/jazzy/p/robot_localization/>
- **Kalibr** — IMU noise model documentation, used for kalibr-style `gyroscope_noise_density` etc. <https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model>
- **OpenVINS** — reference for visual-inertial state estimation architecture. <https://github.com/rpng/open_vins>
