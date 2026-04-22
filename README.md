# Overview

ROS "front-end" for object/marker detectors such as reflectors, alvars, columns and combinations of those.
We can launch detections of two types:

- Primitive detections: those arising from a sensor source. Available now:
	- Reflectors from lidar intensity
	- Columns from lidar scans
	- Alvar bundles (we do not detect single alvars for now, so the simplest alvar detection is a bundle)
	- TMK UWB measurements
	- Vertical cylinders from point clouds
	- Color in a ROI from point clouds

- Composite detections: patterns of detections (patterns of primitive detections in general, but could be of composite detections as well). Available now:
	- Baseline pair: finds detections that are separated a given baseline. Example: docking reflector pairs.

Primitive detectors subscribe to some sensor source (i.e. lidar scan, camera stream, ar_track_alvar output).

# Launch

To launch detectors you need to:

- Define them in a yaml file, giving a name to each detector
- Add them to a launch file, using the detector name as the namespace

Example:

In the yaml file:

```yaml
small_reflectors: # same as node namespace
  type: reflector # type here is only informative, but should be in consonance with node in launch, i.e.: reflector_perceptor type
  enabled_by_default: true
  vizbose: true
  robot_frame: "platform"
  lidars: ["lidar_front", "lidar_back"]
  min_reflector_intensity: 250
  reflector_size: 0.04
  max_detection_range: 20
  rate: 5.0
```

In the launch file:

```xml
<group ns="small_reflectors">
	<node
		pkg="target_detector"
		type="reflector_perceptor"
		name="perceptor"
		output="screen" >
		<remap from="/tf_static" to="/$(arg robot_id)/tf_static"/>
		<remap from="lidar_front" to="/$(arg robot_id)/devices/lidar_front/scan"/>
		<remap from="lidar_back" to="/$(arg robot_id)/devices/lidar_back/scan"/>
	</node>
</group>
```

For yaml files, the available types now are:

- Primitive:
	- reflector
	- column
	- alvar
	- tmk_uwb
	- vertical_cylinder
	- color_in_roi
- Composite:
	- baseline_pair

So in the launch files, you can use the following corresponding node types:

- reflector_perceptor
- column_perceptor
- alvar_perceptor
- tmk_uwb_perceptor
- vertical_cylinder_perceptor
- color_in_roi_perceptor
- baseline_pair_perceptor

The nodes are called perceptors because the detectors word is reserved for the c++ classes that actually contain the code that detects stuff.


# Frames

The published detections are referenced to the platform frame. The X axis of the detected target object (marker, ...) is normal to the target object surface, pointing to the observer. The Z axis of the detected object is pointing up from the object, and the Y axis fulfills the right hand rule Z x X = Y.

# ROS API

## Parameters

Type reflector:
- enabled_by_default (bool)
- vizbose (bool)
- robot_frame (string) -> output detections are referenced to this frame
- lidars (string array for lidar topics)
- min_reflector_intensity (double)
- reflector_size (double)
- max_detection_range (double) -> cut-off distance
- rate (double) -> detections publish rate

Type column:
- enabled_by_default (bool)
- vizbose (bool)
- robot_frame (string) -> output detections are referenced to this frame
- lidars (string array for lidar topics)
- column_size (double) -> side length for square columns, diameter for cylindrical columns
- max_detection_range (double) -> cut-off distance
- scan_decimation (int) -> only every N-th scan is processed
- column_isolation_distance (double) -> optional isolation margin around the estimated column footprint; `0` disables the check
- override_support_points (int) -> 0 keeps the automatic support threshold, otherwise forces a fixed minimum

The column perceptor also exposes `column_size`, `max_detection_range`, `scan_decimation`, `column_isolation_distance` and `override_support_points` through dynamic reconfigure. The yaml values are still applied at startup; dynamic reconfigure is intended for runtime tuning after the node has started.

When `override_support_points` is `0`, the detector computes the minimum required support points from geometry. It first estimates the candidate column center, then computes the expected angular aperture of the column as `2 * asin(column_radius / estimated_center_range)`. That aperture is divided by the scan angular increment to obtain the theoretical support count, and the automatic threshold is set to `ceil(0.75 * theoretical_support_count)`. The 0.75 factor deliberately leaves margin for filtered scans and for missing edge returns, which is especially relevant on rounded columns. Finally, the automatic threshold is clamped to at least `8` points, so very close or coarse scans still require a minimum amount of evidence before publishing a detection.

Type alvar:
- enabled_by_default (bool)
- vizbose (bool)
- robot_frame (string)

Type tmk_uwb:
- enabled_by_default (bool)
- vizbose (bool)
- robot_frame (string)

Type vertical_cylinder:
- enabled_by_default (bool)
- vizbose (bool)
- robot_frame (string)
- source_name (string)
- default_diameter (double) -> boot-time diameter, used only when enabled_by_default is true
- min_cloud_points (int)
- crop_min (double[3])
- crop_max (double[3])
- voxel_size (double[3])

Type color_in_roi:
- enabled_by_default (bool)
- vizbose (bool)
- robot_frame (string)
- source_name (string)
- default_color (string) -> boot-time target color, one of `red`, `green`, `blue`, `unknown` (`any` in action-side requests maps to `unknown`)
- red_hue_center_deg (double) -> hue center for the red bucket
- green_hue_center_deg (double) -> hue center for the green bucket
- blue_hue_center_deg (double) -> hue center for the blue bucket
- min_cloud_points (int) -> minimum number of points in the input cloud before any processing
- min_color_inliers_points (int) -> minimum number of cropped ROI points that must match the winning color to publish a detection
- crop_min (double[3])
- crop_max (double[3])

Type baseline_pair:
- vizbose (bool)
- baseline_tolerance (double)

Composite detectors can not be enabled by default, since they depend on the enabled state of another detector.

`color_in_roi` currently classifies only three logical colors: red, green and blue. It does this by converting ROI points from RGB to HSV and counting how many points fall near the configured hue center of each bucket. The bucket centers are configurable in yaml, but the detector still publishes one of the three `target_detector/Color` values.

## API

Each perceptor generates:

- a detections topic where detections are published
- a visuals topic where markers are publish if vizbose
- a enable server, to enable/disable the processing. The enable request contains detector-specific fields, and each detector only uses the relevant ones:
	- `baseline_pair` uses `baseline`
	- `alvar` uses `alvar_marker_id`
	- `vertical_cylinder` uses `diameter`
	- `color_in_roi` uses `color`
	- `column` only uses the `enable` flag

For vertical cylinders, `enable=true` requires a positive `diameter` in the enable service request. If `enabled_by_default` is true, the detector starts with `default_diameter`.

For columns, the detector geometry is configured from yaml. The published `radius` field carries `column_size / 2` as a nominal size for downstream consumers, even when the real column shape is not cylindrical.

If `column_isolation_distance` is positive, the detector also rejects candidates that have other scan points from different clusters inside the estimated footprint radius plus that extra margin. This is useful to suppress wall and wall-corner false positives when landmarks are expected to be freestanding.

For `color_in_roi`, `enable=true` should provide the requested color in the `color` field of `target_detector/DetectorEnable`. The request type is `target_detector/Color`, and published `target_detector/Detection` messages also include a `color` field for color detections.

For `color_in_roi`, `default_color` is only the boot-time target color used before the first enable request overrides it. The yaml parser accepts `red`, `green`, `blue` and `unknown`, case-insensitively.

If `enabled_by_default` is true and `default_color` is `unknown`, the node starts subscribed and processing input and will publish whichever of red, green or blue wins the ROI classification. This wildcard behavior is also what the action-side `color="any"` alias uses.

If `enabled_by_default` is false, `default_color` is mostly a startup placeholder, because the first enable request sets the active target color.

## Tracking

If needed, a tracker can be launched alongside a perceptor, and that tracker will only track detections from the companion perceptor. Example:

```xml
<group ns="docking_pairs">

	<node
		pkg="target_detector"
		type="baseline_pair_perceptor"
		name="perceptor"
		output="screen" >
		<remap from="/tf_static" to="/$(arg robot_id)/tf_static"/>
		<remap from="detections_in" to="/$(arg robot_id)/perception/small_reflectors/detections"/>
	</node>

	<node
		pkg="target_tracker"
		type="target_tracker"
		name="tracker"
		output="screen" >
		<remap from="odom" to="/$(arg robot_id)/odom"/>
		<remap from="/tf" to="/$(arg robot_id)/navigation/tf"/>
		<remap from="/tf_static" to="/$(arg robot_id)/tf_static"/>
	</node>

</group>
```

## Reference

See duna config and launch for more info on remapping, namespacing, and tracking of those detections. In particular, see how Detect and Track actions in BT trees now have parameters for detector name and type.
