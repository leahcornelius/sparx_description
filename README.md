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

The user confirmed the installed camera is Gemini 335. Fusion nests it under a subassembly named `Gemini 355 asm`. The user-defined `Mounting point datum point` in that parent assembly is exported as a root-coordinate point for inspection. The camera sensor's occurrence origin is near the housing centre and is not a measurement origin; the generated `camera_link` transform remains a housing-origin proxy until the driver frame datum is tied to CAD.

Two reference points were added to the Fusion root from offset construction planes:

| Fusion datum | Root coordinate (cm) | Meaning |
|---|---:|---|
| `ROS_base_link_origin` | `(2.425, 2.810, 4.625)` | Centre of the existing base plate, as a practical chassis datum |
| `ROS_base_footprint_origin` | `(2.425, 2.810, -14.600)` | Vertical projection to the lowest CAD track envelope; ground-contact candidate |

The exporter reads these points every run. The current axis assumption is Fusion root X forward, Y left, Z up. The user approved the chassis centre choice; the CAD axes and track contact plane still need a physical sanity check before calibration is marked complete.

The C1 model bounding box is centred on local X/Z and spans 41.3 mm along local Y, consistent with the published envelope. `laser_frame` uses the published 29.8 mm scan-plane height above the inferred bottom reference. Confirm the component bottom datum and scan azimuth against the assembled robot.

## Candidate transforms

Translations are metres and RPY is radians. These are CAD-derived candidates, not validated physical calibration.

| Edge | Translation | RPY | Evidence and limitation |
|---|---|---|---|
| `base_footprint -> base_link` | `[0, 0, 0.192250]` | `[0, 0, 0]` | CAD base-plate centre above lowest track-envelope point |
| `base_link -> head_link` | `[-0.042057, -0.027758, 0.014011]` | `[0.002903, 0.017208, -0.000224]` | Head occurrence relative to named Fusion chassis datum |
| `head_link -> laser_frame` | `[0.135765, 0.010880, 0.379222]` | `[-0.003729, -0.031698, 0.017533]` | C1 occurrence plus scan-height datum; confirm centre and azimuth |
| `head_link -> camera_link` | `[0.166863, 0.013369, 0.288399]` | `[-0.003428, -0.014248, 0.017475]` | Gemini 335 housing-origin proxy; measurement origin unresolved |

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

- Check Fusion root X/Y/Z against physical forward/left/up and confirm the lowest CAD track envelope is the real contact plane.
- Confirm the meaning of the user-created camera mounting point and relate it to the installed camera's driver `camera_link` origin. The PDF gives frame topology, not numeric transforms. `tf2_echo camera_link camera_depth_optical_frame` verifies the driver's internal optical transform.
- Confirm C1 rotating-centre placement and azimuth in RViz against `/scan` and a known target.
- Confirm the live driver does not already publish a robot-mounted parent edge to `camera_link`.
- Identify the microphone datum and resolve the neck revolute joint before adding those frames.
- Run Xacro, `check_urdf` and RViz validation on a ROS 2 Jazzy machine; the Windows workspace does not include the ROS toolchain.
