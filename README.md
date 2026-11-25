# SparX ROS 2 description

This package is an initial Fusion-derived ROS description. The Fusion exporter reads the master assembly into a JSON manifest; the ROS generator combines that manifest with the separately maintained CAD-to-ROS mapping. It avoids generating one URDF link for every bolt or electronics detail.

## Frame architecture

```text
map -> odom -> base_footprint -> base_link -> head_link
                                                ├─ laser_frame
                                                ├─ camera_link -> Orbbec-owned frames
                                                └─ mic_link (pending)
```

This package publishes only the static robot description. SLAM/localisation will own `map -> odom`, and odometry will own `odom -> base_footprint`. The neck is fixed for this first version. Fusion contains an as-built revolute joint between `Face:1` and `Body Frame:1`, but its joint geometry was inaccessible at the current timeline position; do not enable articulation until its axis and centre are verified.

## Fusion inspection and datums

Inspected through Fusion MCP on 2026-11-25. The active design is `Complete Assembly`, uses mm display units and contains 639 occurrences. The exporter composes nested occurrence transforms and stores them in Fusion's internal cm units. Exact paths are in `config/cad_mapping.yaml`; missing paths and missing datum features stop export.

The user confirmed the installed camera is Gemini 335. Fusion nests it under a subassembly named `Gemini 355 asm`. The user-defined `Mounting point datum point` in that parent assembly is exported as a root-coordinate point for inspection. The camera occurrence origin is not the `camera_link` measurement origin. The mapping retains a local offset calibrated to preserve the physically validated scan/cloud overlay; the driver-owned optical descendants remain unchanged.

Two reference points were added to the Fusion root from offset construction planes:

| Fusion datum | Root coordinate (cm) | Meaning |
|---|---:|---|
| `ROS_base_link_origin` | `(2.425, 2.810, 4.625)` | Centre of the existing base plate, as a practical chassis datum |
| `ROS_base_footprint_origin` | `(2.425, 2.810, -14.600)` | Vertical projection to the lowest CAD track envelope; ground-contact candidate |

The exporter reads these points every run. `base_link` uses the named point for translation and the `/Left Side:1` chassis occurrence for orientation. Fusion MCP inspection found `/Left Side:1`, `/Right Side:1`, `/Face:1`, and the head share the same neutral orientation. This removes the shared CAD-root rotation from `base_link -> head_link`. The track contact plane remains a candidate pending a physical height check.

The C1 model bounding box is centred on local X/Z and spans 41.3 mm along local Y, consistent with the published envelope. `laser_frame` uses the published 29.8 mm scan-plane height above the inferred bottom reference. The local 180° scan yaw was physically validated against the depth point cloud; the C1 rotating-centre translation remains to be checked against a known target.

## Candidate transforms

Translations are metres and RPY is radians. Sensor measurement-origin translations remain provisional where the CAD does not define those origins.

## Sensor alignment corrections (2026-11-25)

The user physically validated that a local 180° yaw on the RPLIDAR measurement frame makes the scan align and move with the depth point cloud. This is stored as `measurement_frame_correction_rpy_rad: [0, 0, pi]`. The generator applies it after the CAD-derived origin translation, so the LiDAR mounting translation is preserved.

Fusion MCP inspection found an approximately 1° pitch in the `RPLIDAR C1:1` occurrence relative to the camera/head. The C1 is connected to `Attachment Plate:1` by the `Rigid 2` as-built rigid joint; the surrounding mount occurrences are identity-oriented. The mapping has a distinct `cad_mounting_correction_rpy_rad` for mechanical alignment. The camera has its own small CAD-frame correction so its body frame is level with the neutral head. Its existing validated frame translation is retained by a separate local origin offset. The LiDAR CAD and measurement corrections are rotations only and preserve the LiDAR mounting translation. No Orbbec optical descendant transforms are generated or modified.

The approximately 0.986° shared angular offset is common to the chassis sides and the face/head in Fusion; it is not an offset in the head joint. The as-built `Revolute 2` joint reports zero rotation. The exporter now records `/Left Side:1` axes, and the generator uses those axes for both `base_link` and `base_footprint`. The generated head attitude is therefore zero in the neutral position. No `map -> odom` or `odom -> base_footprint` transform is added.

The CAD joint itself is left intact. The mapping corrections are separate from the model geometry and remain in place when the manifest is regenerated.

| Edge | Translation | RPY | Evidence and limitation |
|---|---|---|---|
| `base_footprint -> base_link` | `[-0.003308, 0.000558, 0.192221]` | `[0, 0, 0]` | CAD ground projection, expressed in chassis axes |
| `base_link -> head_link` | `[-0.042286, -0.027729, 0.013366]` | `[0, 0, 0]` | Chassis and head share their neutral Fusion orientation |
| `head_link -> laser_frame` | `[0.135765, 0.010880, 0.379222]` | `[0, 0, 3.141593]` | Existing CAD-derived translation preserved; 180° measurement-frame yaw applied |
| `head_link -> camera_link` | `[0.166863, 0.013369, 0.288399]` | `[0, 0, 0]` | Prior physically validated point-cloud/scan translation retained |

The supplied `view_frames` PDF confirms `camera_link` is the Orbbec TF root and that depth, color and optical frames are below it. The URDF supplies only the mount edge to `camera_link`; it does not duplicate the driver's descendants. User-confirmed topic frame IDs:

| Topic | Frame ID |
|---|---|
| `/scan` | `laser_frame` |
| `/camera/depth/image_raw` | `camera_depth_optical_frame` |
| `/camera/depth/camera_info` | `camera_depth_optical_frame` |
| `/sparx/depth/points` | `camera_depth_optical_frame` |

## Copy and run on ROS 2 Jazzy

Copy the complete `sparx_description` folder into a colcon workspace's `src` directory, then from the workspace root run:

```bash
sudo apt install python3-yaml ros-jazzy-xacro ros-jazzy-robot-state-publisher
python3 src/sparx_description/scripts/generate_description.py --allow-unvalidated
colcon build --symlink-install --packages-select sparx_description
source install/setup.bash
ros2 launch sparx_description description.launch.py
```

For repeat use, regenerate the Fusion manifest by running `cad_export/fusion_export.py` inside Fusion first, then run the ROS generator again. It reads the named points and full nested occurrence transforms; it does not edit mechanical geometry. Review the JSON and mapping diff after changing CAD hierarchy. Mesh export is not enabled yet.

## Git checkout

The package lives in the configured GitHub repository. Review and merge the sensor-alignment pull request, then update each robot with `git pull` before rebuilding.

## TF ownership and validation

The description publishes `base_footprint -> base_link`, `base_link -> head_link`, `head_link -> laser_frame`, and `head_link -> camera_link`. SLAM/localisation owns `map -> odom`; odometry owns `odom -> base_footprint`. The Orbbec driver owns its camera/depth/color/optical descendants. Confirm no other node publishes a robot-mounted parent for `camera_link` before launching, to avoid duplicate TF publishers.

On the machine running the sensor drivers:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link laser_frame
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo camera_link camera_depth_optical_frame
ros2 topic echo /scan --once --field header.frame_id
ros2 topic echo /camera/depth/image_raw --once --field header.frame_id
```

Validate the description with:

```bash
xacro install/sparx_description/share/sparx_description/urdf/sparx.urdf.xacro > /tmp/sparx.urdf
check_urdf /tmp/sparx.urdf
```

In RViz2, set Fixed Frame to `base_link`, then add RobotModel, TF, LaserScan (`/scan`) and PointCloud2 (`/sparx/depth/points`). The cloud's frame should be `camera_depth_optical_frame`. A fixed overlay displacement suggests a translation error; divergence that grows with range suggests angular error; a mirrored scan sweep suggests orientation/handedness error. Switch RViz to `odom` once odometry publishes it.

## Remaining calibration

- Confirm the chassis orientation reference still maps to physical forward/left/up after CAD hierarchy changes, and check the lowest CAD track envelope against the real contact plane.
- Confirm the meaning of the user-created camera mounting point and relate it to the installed camera's driver `camera_link` origin. The PDF gives frame topology, not numeric transforms. `tf2_echo camera_link camera_depth_optical_frame` verifies the driver's internal optical transform.
- Confirm C1 rotating-centre placement in RViz against a known target; scan yaw has already been physically validated.
- Confirm the live driver does not already publish a robot-mounted parent edge to `camera_link`.
- Identify the microphone datum and resolve the neck revolute joint before adding those frames.
- Run Xacro, `check_urdf` and RViz validation on a ROS 2 Jazzy machine; the Windows workspace does not include the ROS toolchain.
