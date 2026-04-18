# Overview

ROS front-end for object and marker detectors such as reflectors, ALVAR bundles, UWB anchors, vertical cylinders, and color cues in a point-cloud ROI.

Two detector families are supported:

- Primitive detections: detections that come directly from a sensor source.
- Composite detections: detections built from other detections.

Available detector types:

- Primitive:
  - `reflector`
  - `alvar`
  - `tmk_uwb`
  - `vertical_cylinder`
  - `color_in_roi`
- Composite:
  - `baseline_pair`

The ROS nodes are called `*_perceptor` because the word detector is reserved for the C++ classes that perform the actual detection work.

# Launch And Configuration

Each detector instance is defined by a namespace. The same namespace must exist:

- in YAML, where detector parameters are stored
- in launch, where the corresponding perceptor node is started

Example reflector detector:

```yaml
reflectors:
  type: reflector
  enabled_by_default: true
  vizbose: true
  robot_frame: "platform"
  lidars: ["lidar_front", "lidar_back"]
  scan_type: "sensor_msgs/LaserScan" # or sick_safetyscanners/ExtendedLaserScanMsg
  min_reflector_intensity: 250        # only used for sensor_msgs/LaserScan
  reflector_size: 0.04
  max_detection_range: 20
  scan_decimation: 5
  override_support_points: 0          # 0 keeps the adaptive threshold, any other value forces that exact minimum support count
```

```xml
<group ns="reflectors">
  <node
    pkg="target_detector"
    type="reflector_perceptor"
    name="perceptor"
    output="screen">
    <remap from="/tf_static" to="/$(arg robot_id)/tf_static"/>
    <remap from="lidar_front" to="/$(arg robot_id)/devices/lidar_front/scan"/>
    <remap from="lidar_back" to="/$(arg robot_id)/devices/lidar_back/scan"/>
  </node>
</group>
```

Detector type to node mapping:

- `reflector` -> `reflector_perceptor`
- `alvar` -> `alvar_perceptor`
- `tmk_uwb` -> `tmk_uwb_perceptor`
- `vertical_cylinder` -> `vertical_cylinder_perceptor`
- `color_in_roi` -> `color_in_roi_perceptor`
- `baseline_pair` -> `baseline_pair_perceptor`

# Frames

Published detections are referenced to `robot_frame` (typically `platform`).

The target pose convention is:

- X axis points normal to the target surface, towards the observer
- Z axis points up from the object
- Y axis follows the right-hand rule: `Z x X = Y`

# ROS API

## Parameters

### `reflector`

- `enabled_by_default` (`bool`)
- `vizbose` (`bool`)
- `robot_frame` (`string`)
- `lidars` (`string[]`) names of the input scan topics to subscribe to
- `scan_type` (`string`) one of `sensor_msgs/LaserScan` or `sick_safetyscanners/ExtendedLaserScanMsg`
- `min_reflector_intensity` (`double`) only used when `scan_type` is `sensor_msgs/LaserScan`
- `reflector_size` (`double`)
- `max_detection_range` (`double`)
- `scan_decimation` (`int`) process one out of every N scans
- `override_support_points` (`int`) if `0`, the detector computes the minimum support count from reflector geometry and range, with a minimum of 3; if non-zero, that exact value is used as the minimum support count

When `scan_type` is `sick_safetyscanners/ExtendedLaserScanMsg`, the detector uses the reflector-hit flag from the message instead of intensity thresholding.

### `alvar`

- `enabled_by_default` (`bool`)
- `vizbose` (`bool`)
- `robot_frame` (`string`)

The perceptor expects bundle detections from `ar_track_alvar`. Single tags are not detected directly here; the primitive input is an ALVAR bundle message.

### `tmk_uwb`

- `enabled_by_default` (`bool`)
- `vizbose` (`bool`)
- `robot_frame` (`string`)

### `vertical_cylinder`

- `enabled_by_default` (`bool`)
- `vizbose` (`bool`)
- `robot_frame` (`string`)
- `source_name` (`string`)
- `default_diameter` (`double`) startup diameter, only used when `enabled_by_default` is `true`
- `min_cloud_points` (`int`)
- `crop_min` (`double[3]`)
- `crop_max` (`double[3]`)
- `voxel_size` (`double[3]`)

Validation notes:

- `default_diameter` must be `> 0` when `enabled_by_default=true`
- `min_cloud_points` must be `>= 1`
- `crop_min` and `crop_max` must contain exactly 3 elements, with each `crop_min[i] < crop_max[i]`
- all `voxel_size` components must be `> 0`

### `color_in_roi`

- `enabled_by_default` (`bool`)
- `vizbose` (`bool`)
- `robot_frame` (`string`)
- `source_name` (`string`)
- `default_color` (`string`) one of `red`, `green`, `blue`, `unknown`
- `red_hue_center_deg` (`double`)
- `green_hue_center_deg` (`double`)
- `blue_hue_center_deg` (`double`)
- `min_cloud_points` (`int`)
- `min_color_inliers_points` (`int`)
- `crop_min` (`double[3]`)
- `crop_max` (`double[3]`)

Validation notes:

- `min_cloud_points` must be `>= 1`
- `min_color_inliers_points` must be `>= 1`
- `crop_min` and `crop_max` must contain exactly 3 elements, with each `crop_min[i] < crop_max[i]`

`color_in_roi` classifies three logical colors: red, green, and blue. It converts ROI points from RGB to HSV and counts how many points fall near the configured hue center of each bucket. The bucket centers are configurable in YAML, but the published value is still one of the `target_detector/Color` codes.

### `baseline_pair`

- `vizbose` (`bool`)
- `robot_frame` (`string`)
- `baseline_tolerance` (`double`)

Composite detectors cannot be enabled by default because they depend on another detector stream.

## Topics And Services

Each perceptor publishes:

- `detections` (`target_detector/Detections`)
- `visuals` (`visualization_msgs/Marker`) when `vizbose=true`
- `enable` (`target_detector/DetectorEnable`)

Detector-specific inputs and extra outputs:

| Type | Input | Extra outputs / notes |
| --- | --- | --- |
| `reflector` | subscribes to every topic listed in `lidars` | output `source_name` is the input lidar name |
| `alvar` | `detections_in` from `ar_track_alvar_msgs/AlvarMarkers` | publishes `bundle_enable_detection` (`std_msgs/Bool`) to drive upstream ALVAR bundle detection |
| `tmk_uwb` | `detections_in` from `tmk_uwb/UwbMeasurement` | output `source_name` is `tmk_uwb_<anchor_id>` |
| `vertical_cylinder` | `point_cloud_in` (`sensor_msgs/PointCloud2`) | publishes `cloud_out` when `vizbose=true` |
| `color_in_roi` | `point_cloud_in` (`sensor_msgs/PointCloud2`) | publishes `cloud_out` when `vizbose=true` |
| `baseline_pair` | `detections_in` (`target_detector/Detections`) | expects incoming detections already expressed in `robot_frame` |

`DetectorEnable.srv` contains fields for every detector type, but each perceptor only uses the fields relevant to it:

- `reflector`: only `enable`
- `alvar`: `enable`, `alvar_marker_id`
- `tmk_uwb`: only `enable`
- `baseline_pair`: `enable`, `baseline`
- `vertical_cylinder`: `enable`, `diameter`
- `color_in_roi`: `enable`, `color`

Runtime notes:

- `baseline_pair` requires a positive `baseline` when enabling.
- `vertical_cylinder` requires a positive `diameter` when enabling.
- `color_in_roi` uses the requested `color` from `target_detector/Color`.
- `default_color` is only the startup target color before the first enable request overrides it.
- when the active target color is `unknown`, `color_in_roi` acts as a wildcard and publishes whichever of red, green, or blue wins the ROI classification
- The action-side alias `color="any"` maps to `unknown`.

## Messages

Useful message details:

- `target_detector/Detections` contains `header`, `source_name`, and an array of `target_detector/Detection`
- `target_detector/Detection` includes detector `type`, `id`, `pose`, `supports`, and detector-specific fields such as `baseline`, `radius`, and `color`
- `target_detector/Color` uses `UNKNOWN`, `RED`, `GREEN`, and `BLUE`

# Tracking

A tracker can be launched alongside a perceptor and configured to track only that detector stream.

Example:

```xml
<group ns="docking_pairs">
  <node
    pkg="target_detector"
    type="baseline_pair_perceptor"
    name="perceptor"
    output="screen">
    <remap from="/tf_static" to="/$(arg robot_id)/tf_static"/>
    <remap from="detections_in" to="/$(arg robot_id)/perception/reflectors/detections"/>
  </node>

  <node
    pkg="target_tracker"
    type="target_tracker"
    name="tracker"
    output="screen">
    <remap from="odom" to="/$(arg robot_id)/odom"/>
    <remap from="/tf" to="/$(arg robot_id)/navigation/tf"/>
    <remap from="/tf_static" to="/$(arg robot_id)/tf_static"/>
  </node>
</group>
```

# How Units Use It

The package-local launch files are minimal examples. The current robot stacks in this repo are better references for real usage.

## `abc_rides`

`abc_rides` uses:

- `reflectors` from one `sick_safetyscanners/ExtendedLaserScanMsg` stream
- `docking_pairs` as a composite detector over reflector detections
- `beacons` from TMK UWB measurements

Config example:

```yaml
reflectors:
  type: reflector
  enabled_by_default: true
  vizbose: true
  robot_frame: "platform"
  lidars: ["lidar"]
  scan_type: "sick_safetyscanners/ExtendedLaserScanMsg"
  reflector_size: 0.02
  max_detection_range: 10
  scan_decimation: 1
  override_support_points: 2

beacons:
  type: tmk_uwb
  enabled_by_default: true
  vizbose: true
  robot_frame: "platform"
```

Launch wiring:

```xml
<group ns="perception">
  <rosparam file="$(arg unit_path)/config/perception/primitive_detectors.yaml" command="load"/>
  <rosparam file="$(arg unit_path)/config/perception/composite_detectors.yaml" command="load"/>
  <rosparam file="$(arg unit_path)/config/perception/trackers.yaml" command="load"/>

  <group ns="reflectors">
    <node pkg="target_detector" type="reflector_perceptor" name="perceptor" output="screen">
      <remap from="/tf_static" to="/$(arg robot_id)/tf_static"/>
      <remap from="lidar" to="/$(arg robot_id)/devices/lidar/extended_laser_scan"/>
    </node>
  </group>

  <group ns="docking_pairs">
    <node pkg="target_detector" type="baseline_pair_perceptor" name="perceptor" output="screen">
      <remap from="/tf_static" to="/$(arg robot_id)/tf_static"/>
      <remap from="detections_in" to="/$(arg robot_id)/perception/reflectors/detections"/>
    </node>
  </group>

  <group ns="beacons">
    <node pkg="target_detector" type="tmk_uwb_perceptor" name="perceptor" output="screen">
      <remap from="/tf_static" to="/$(arg robot_id)/tf_static"/>
      <remap from="detections_in" to="/$(arg robot_id)/devices/onboard_beacon/uwb_measurements"/>
    </node>
  </group>
</group>
```

`abc_rides/trees/dock_extended.xml` enables `docking_pairs` with a runtime reflector baseline before tracking and docking.

## `electro_jet`

`electro_jet` loads a model-specific primitive detector file with `$(arg robot_model)_primitive_detectors.yaml`. The current `robogripper` and `robohook` configs use:

- `reflectors`
- `docking_pairs`
- `barrels`
- `colored_small_barrels`

Config example:

```yaml
reflectors:
  type: reflector
  enabled_by_default: true
  vizbose: true
  robot_frame: "platform"
  lidars: ["lidar_top"]
  scan_type: "sensor_msgs/LaserScan"
  min_reflector_intensity: 600
  reflector_size: 0.05
  max_detection_range: 30
  scan_decimation: 1
  override_support_points: 0

barrels:
  type: vertical_cylinder
  enabled_by_default: false
  vizbose: true
  robot_frame: "platform"
  source_name: "camera"
  default_diameter: 0.6
  min_cloud_points: 1000
  crop_min: [-0.6, 0.1, 0.4]
  crop_max: [0.6, 0.6, 1.4]
  voxel_size: [0.02, 0.02, 0.02]

colored_small_barrels:
  type: color_in_roi
  enabled_by_default: false
  default_color: "red"
  vizbose: true
  robot_frame: "platform"
  source_name: "camera"
  red_hue_center_deg: 0
  green_hue_center_deg: 120
  blue_hue_center_deg: 220
  crop_min: [-0.1, 0.1, 0.55]
  crop_max: [0.1, 0.2, 1.0]
  min_cloud_points: 1000
  min_color_inliers_points: 100
```

Launch wiring:

```xml
<group ns="perception">
  <rosparam file="$(arg unit_path)/config/perception/$(arg robot_model)_primitive_detectors.yaml" command="load"/>
  <rosparam file="$(arg unit_path)/config/perception/composite_detectors.yaml" command="load"/>
  <rosparam file="$(arg unit_path)/config/perception/trackers.yaml" command="load"/>

  <group ns="reflectors">
    <node pkg="target_detector" type="reflector_perceptor" name="perceptor" output="screen">
      <remap from="/tf_static" to="/$(arg robot_id)/tf_static"/>
      <remap from="lidar_right" to="/$(arg robot_id)/devices/lidar_right/scan"/>
      <remap from="lidar_left" to="/$(arg robot_id)/devices/lidar_left/scan"/>
      <remap from="lidar_top" to="/$(arg robot_id)/devices/lidar_top/scan"/>
    </node>
  </group>

  <group ns="barrels">
    <node pkg="target_detector" type="vertical_cylinder_perceptor" name="perceptor" output="screen">
      <remap from="/tf_static" to="/$(arg robot_id)/tf_static"/>
      <remap from="/tf" to="/$(arg robot_id)/navigation/tf"/>
      <remap from="point_cloud_in" to="/$(arg robot_id)/devices/camera/depth/color/points"/>
    </node>
  </group>

  <group ns="colored_small_barrels">
    <node pkg="target_detector" type="color_in_roi_perceptor" name="perceptor" output="screen">
      <remap from="/tf_static" to="/$(arg robot_id)/tf_static"/>
      <remap from="/tf" to="/$(arg robot_id)/navigation/tf"/>
      <remap from="point_cloud_in" to="/$(arg robot_id)/devices/camera/depth/color/points"/>
    </node>
  </group>
</group>
```

The launch exposes `lidar_right`, `lidar_left`, and `lidar_top` remaps. The current `robogripper` and `robohook` YAML files select `lidar_top` through the `lidars` parameter.

Runtime examples from the BT trees:

- `electro_jet/trees/dock_reflector_pair.xml` enables `docking_pairs` with `reflector_baseline`
- `electro_jet/trees/dock_barrel.xml` enables `barrels` with `diameter`
- `electro_jet/trees/wait_for_color.xml` waits on `colored_small_barrels` with a requested `color`
