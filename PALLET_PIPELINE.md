# Pallet Detection Pipeline

End-to-end flow of the `pallet_perceptor` node — from incoming point cloud to published detection.

## Pipeline

```mermaid
graph TD
    A[PointCloud2 input<br/>sensor frame] --> B[1. ROS → PCL]
    B --> C[2. Y-band crop<br/>floor − H ≤ y ≤ floor]
    C -. gated .-> P1[/cloud_no_plane<br/>sensor frame/]
    C --> D[3. Voxel downsample]
    D --> E[4. Euclidean clustering]
    E --> F[5. tryDetectPalletInCluster<br/>per cluster]

    subgraph PERCLUSTER [Per-cluster detection]
        direction TB
        G1[5a. RANSAC face-normal<br/>→ axis_u] --> G2[5b. Project to 2D grid u,v]
        G2 --> G3[5c. Slide EUR/EPAL template<br/>→ chi score]
        G3 --> G4{5d. Width gate}
        G4 --> G5[5e. Extract matched points]
        G5 --> G6[5f. RANSAC stringer plane<br/>→ yaw + x,z<br/>PCA fallback]
    end

    F --> PERCLUSTER
    PERCLUSTER --> H[6. Keep best chi cluster]
    H --> I[Build pose in sensor frame]
    I --> J[tf2 transform<br/>sensor → robot frame]
    J --> P3[/detections<br/>robot frame/]
    I --> P4[/pallet_pose<br/>sensor frame/]
    I -. vizbose .-> P5[/cloud_out: inliers<br/>sensor frame/]
    I -. vizbose .-> P6[/visuals: OBB + arrow<br/>sensor frame/]

    classDef gated stroke-dasharray: 5 5,stroke:#888
    classDef vizbose stroke-dasharray: 2 2,stroke:#a60
    classDef always stroke:#0a0,stroke-width:2px
    class P1 gated
    class P5,P6 vizbose
    class P3,P4 always
```

**Legend:**
- Green = always published on every detection
- Dashed grey = subscriber-gated (only serialized if someone is listening)
- Dashed orange = `vizbose__` debug-only

## Published Topics

| Topic | Type | When | Frame |
|---|---|---|---|
| `detections` | `target_detector/Detections` | every detection | robot |
| `pallet_pose` | `geometry_msgs/PoseStamped` | every detection | sensor |
| `cloud_no_plane` | `sensor_msgs/PointCloud2` | subscriber-gated | sensor |
| `cloud_out` | `sensor_msgs/PointCloud2` | `vizbose` only | sensor |
| `visuals` | `visualization_msgs/Marker` (OBB + arrow) | `vizbose` only | sensor |

## Why these stages?

| Stage | Purpose |
|---|---|
| **Y-band crop** | Sole spatial filter. Replaces a separate XYZ ROI box: the camera is mounted at a fixed, known height, so `floor_y` is a constant and anything outside the pallet height band cannot be the pallet. Dropping outside-band points up-front shrinks the work for every later stage. |
| **Voxel downsample** | Reduce point density before clustering and per-point work. |
| **Euclidean clustering** | Isolate individual candidate objects in the band. |
| **Face-normal RANSAC (5a)** | Find the pallet face direction so the 2D projection follows the true face, not camera X. Makes detection yaw-aware up to ~25°. |
| **Template matching (5b–5c)** | Recognize the EUR/EPAL fork-pocket pattern (top deck + 3 stringers). |
| **Width gate (5d)** | Rejects matches whose width doesn't fit the expected pallet dimensions. |
| **Stringer-zone RANSAC (5f)** | Refine yaw and (x, z) using only stringer points — excludes any box on the top deck that could bias the fit. |
| **TF to robot frame** | Downstream consumers (navigation, manipulation) expect robot-relative coordinates. |
