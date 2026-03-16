# Overview

ROS "front-end" for object/marker detectors such as reflectors, alvars, columns and combinations of those.
We can launch detections of two types:

- Primitive detections: those arising from a sensor source. Available now:
    - Reflectors from lidar intensity
	- Alvar bundles (we do not detect single alvars for now, so the simplest alvar detection is a bundle)
	- TMK UWB measurements
    - Vertical cylinders from point clouds

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
    - alvar
	- tmk_uwb
    - vertical_cylinder
- Composite:
    - baseline_pair

So in the launch files, you can use the following corresponding node types:

- reflector_perceptor
- alvar_perceptor
- tmk_uwb_perceptor
- vertical_cylinder_perceptor
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

Type baseline_pair:
- vizbose (bool)
- baseline_tolerance (double)

Composite detectors can not be enabled by default, since they depend on the enabled state of another detector.

## API

Each perceptor generates:

- a detections topic where detections are published
- a visuals topic where markers are publish if vizbose
- a enable server, to enable/disable the processing. Note that alvars, baseline_pair and vertical_cylinder need parameters when enabling.

For vertical cylinders, `enable=true` requires a positive `diameter` in the enable service request. If `enabled_by_default` is true, the detector starts with `default_diameter`.

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
