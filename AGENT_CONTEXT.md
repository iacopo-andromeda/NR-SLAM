# NR-SLAM Agent Context (Andromeda-first)

> Canonical unified documentation now lives in `README.md`.
> Keep this file only as a compatibility reference and avoid updating it independently.

Purpose: provide a compact, high-signal context file for coding agents working on this repository.
This document keeps both (1) theoretical/paper context and (2) current code reality, with drift notes where they differ.

## 1) Scope and Source of Truth

- Theory/background: `NR-SLAM.pdf` (paper) and `result.md` (paper-to-code traceability snapshot).
- Current executable truth: source code in `apps/`, `modules/`, `data/`, and CMake/build files.
- When there is conflict, prefer current code behavior and mark theory as historical context.

## 2) What This Repo Is (Current Practical View)

- NR-SLAM is a non-rigid monocular SLAM system with tracking, mapping, and visualization modules.
- The repository is configured to build only the Andromeda app entrypoint: `andromeda`.
- Primary workflow in this repo is Andromeda bag playback through `apps/andromeda.cc`.

### Repository pruning status (current)

- Removed app sources: `apps/hamlyn.cc`, `apps/endomapper.cc`, `apps/simulation.cc`.
- Removed dataset loader sources/headers:
  - `modules/datasets/hamlyn.{cc,h}`
  - `modules/datasets/endomapper.{cc,h}`
  - `modules/datasets/simulation.{cc,h}`
- `apps/CMakeLists.txt` now defines only one executable target: `andromeda`.
- `modules/CMakeLists.txt` now includes only `datasets/andromeda.{cc,h}` in `DATASETS_SOURCES`.

## 3) Build and Runtime Reality

### Build

- Build script: `./build.sh`.
- `build.sh` now runs under `zsh` and auto-loads the user `ros_setup`
  function from `~/.andromeda_profile` when available, so non-interactive VS
  Code task shells inherit the ROS/ament environment.
- Typical commands:
  - `./build.sh --all`
  - `./build.sh --target andromeda`
  - `./build.sh --dbg --target andromeda`
- Build outputs:
  - executables in `build/bin/`
  - libraries in `build/lib/`

### Toolchain/dependencies that matter in practice

- CMake enforces C++17.
- Top-level CMake prepends `/opt/ros/jazzy` in `CMAKE_PREFIX_PATH`.
- `andromeda` links ROS message package (`sensor_msgs`) and `cpp_bag_reader`.

### Known build blocker in this environment

- Historical build failure was environment-related, not app-target wiring:
  - `ModuleNotFoundError: No module named 'ament_package'`
  - Failure occurred while configuring ROS/ament dependencies from
    `cpp_bag_reader`.
- Current mitigation: `build.sh` sources the Andromeda profile when needed and
  calls `ros_setup` before running CMake.
- If this appears again, treat it as a ROS Python environment issue before
  changing NR-SLAM source logic.

## 4) Andromeda Operational Contract (Most Important)

Primary implementation: `apps/andromeda.cc`.

### Input topics

- Main image topic: `/image_raw/compressed`
- Secondary topic (counted/ignored): `/robot/state`

### Key flags

- `--dataset_path`: rosbag directory path.
- `--settings_path`: YAML settings path.
- `--starting_frame` and `--end_frame`: interpreted by `--range_mode`.
- `--range_mode`: `timestamp_ns` (default) or `message_index`.
- `--max_images`: max processed main-topic images; `0` means unlimited.
- `--output_dir`: root directory for all run outputs.  The binary writes
  `slam.log` here and passes this path to `System`, which derives
  `{output_dir}/map_viz/`, `{output_dir}/viz/`, and `{output_dir}/eval/`.
  Replaces the old `--log_file` flag and the three `*.save_path` YAML keys.

### Range semantics (important)

- `timestamp_ns` mode:
  - bag iterator opened with start/end timestamps.
  - processing also stops when `message.timestamp_ns >= end_frame` if `end_frame > 0`.
- `message_index` mode:
  - only main-topic image messages increment index.
  - `starting_frame` and `end_frame` apply to that image index, not global message index.

### Processing loop summary

1. Open bag.
2. Skip non-main topics (but count `/robot/state`).
3. Deserialize `sensor_msgs::msg::CompressedImage`.
4. Decode with OpenCV (`cv::imdecode`).
5. Call `System::TrackImage(image)`.
6. Stop on `max_images` or end-bound.
7. Report counters and decode failures.

## 5) Runtime Pipeline in `System`

Primary implementation: `modules/SLAM/system.cc`.

- `System::TrackImage()` flow is currently sequential in one call path:
  1. preprocess image (gray + CLAHE),
  2. generate masks,
  3. tracking (`tracker_->TrackImage`),
  4. mapping (`mapper_->DoMapping`),
  5. visualization update,
  6. per-frame performance logging (`[PERF]`, CSV via `PerformanceLogger`).
- `MapVisualizer` runs in its own thread.
- Per-frame performance CSV currently defaults to `slam_performance.csv` in working directory.
- Per-frame performance CSV (`slam_performance.csv`) and pose comparison CSV (`pose_comparison.csv`) are written to `output_dir` when provided, otherwise to the working directory.

## 6) Tracking Behavior That Recently Matters

Primary implementation: `modules/tracking/tracking.cc` and `modules/tracking/tracking.h`.

- Tracking status machine: `NOT_INITIALIZED`, `TRACKING`, `LOST`.
- LOST handling includes monocular re-bootstrap (`LostModeBootstrap`) instead of hard stop.
- LOST bootstrap can skip frames with `lost_bootstrap_frame_stride`.
- Recovery grace period uses lower threshold (`lost_recovery_min_tracked_points`) before returning to strict threshold (`min_tracked_points_abort`).
- Metrics are logged with `[TRACK_METRICS]` lines; analysis scripts parse these.

## 7) Settings Contract (Andromeda)

Main files:

- `data/andromeda/settings.yaml`
- `modules/SLAM/settings.h`
- `modules/SLAM/settings.cc`

Active groups used by current code:

- Camera model/intrinsics (`Camera.*`, including `DistortedPinHole`).
- Masking filter file (`Masking.filterFile`, e.g., `./data/andromeda/filters.txt`).
- Visualization paths and views (`MapVisualizer.*` views; save paths come from `--output_dir`).
- Tracking/KLT parameters (`Tracking.klt_*`, keyframe cadence, stale-point pruning, LOST thresholds, etc.).
- Feature extraction (`Features.max_corners`, `Features.quality_level`, `Features.min_distance`).

## 8) Theoretical Basis by Pipeline Step (Tracking and Mapping)

This section maps code steps to the paper-level objectives in `NR-SLAM.pdf` and to the traceability summary in `result.md`.

### 8.1 Tracking: theory-to-code mapping

1. **Short-term data association (KLT + photometric validation)**
  - Theory: Section IV-A (short-term KLT tracking with photometric consistency checks).
  - Code path: `Tracking::DataAssociation()` calls `LucasKanadeTracker::Track()` with SSIM thresholding (`klt_min_SSIM`) and mask constraints.

2. **Rigid camera pose seed and refinement**
  - Theory: first stage of coarse-to-fine tracking (rigid-only pose solve before full deformable refinement).
  - Code path: `Tracking::CameraPoseEstimation()` then `CameraPoseOptimization()`.
  - Objective (pose-only): minimize reprojection error over currently tracked 3D points.

3. **Joint camera + deformation optimization**
  - Theory: joint minimization of reprojection + visco-elastic regularization terms.
  - Code path: `Tracking::CameraPoseAndDeformationEstimation()` then `CameraPoseAndDeformationOptimization()`.
  - Implemented energy decomposition in optimizer:
    - reprojection term (`ReprojectionErrorWithDeformation`),
    - spatial regularizer (`SpatialRegularizerWithDeformation`) acting like viscous coupling,
    - position regularizer (`PositionRegularizerWithDeformation`) acting like elastic/rest-distance prior.
  - Robustification: Huber kernels + chi-square outlier checks; outliers are demoted from `TRACKED_WITH_3D`.

4. **Dynamic deformation graph weighting and pruning**
  - Theory: weighted neighbor coupling with Gaussian-like weight and edge pruning by stretch criterion.
  - Code path: `RegularizationGraph` + `InterpolationWeight()`.
  - Current weight form exactly implemented as:
    - `w(d) = exp(-(d^2)/(2*sigma^2))`
  - Edge consistency gate uses relative stretch:
    - `|d_max - d_min| / d_min > streching_th  => edge marked BAD`.

5. **Mid-term association / point reuse**
  - Theory: recover temporarily lost points by projection and re-tracking (paper mid-term association concept).
  - Code path: `Tracking::PointReuse()` projects map points into image, performs local KLT tracking from projected seeds, and validates with reprojection error.

6. **LOST state and monocular re-bootstrap**
  - Theory anchor: map initialization logic (monocular geometric bootstrap) reused as recovery mechanism.
  - Code path: `Tracking::LostModeBootstrap()` + `MonocularMapInitializer`.
  - Practical details:
    - uses recovered two-view geometry and triangulated structure,
    - rescales map from median depth to a target scale (`3.0` world units),
    - re-initializes regularization graph and KLT references.

### 8.2 Mapping: theory-to-code mapping

1. **Dual mapping modes (keyframe vs frame mapping)**
  - Theory: mapping alternates between global consistency refinement and local map growth.
  - Code path: `Mapping::DoMapping()`.
    - If an unmapped keyframe exists: `KeyFrameMapping()`.
    - Else: `FrameMapping()` (triangulation-focused update).

2. **Keyframe mapping as local deformable BA**
  - Theory: deformable bundle adjustment over camera states and map structure with regularization.
  - Code path: `LocalDeformableBundleAdjustment(map, calibration, scale)`.
  - Conceptually corresponds to paper deformable BA objective (reprojection + visco-elastic constraints across observations/graph connections).

3. **Frame mapping: candidate triangulation and model selection**
  - Theory: choose rigid vs deformable triangulation according to evidence quality.
  - Code path: `Mapping::LandmarkTriangulation()`.
  - Current selection policy mirrors the paper-style comparison:
    - if rigid successes > `1.5 *` deformable successes, prefer rigid set,
    - if deformable successes >= `1.5 *` rigid successes, prefer deformable set,
    - otherwise no forced selection (tie path).

4. **Rigid triangulation gates (geometry sanity checks)**
  - Theory: enforce triangulation validity before inserting landmarks.
  - Code checks include:
    - track length / rigidity check,
    - parallax bounds,
    - positive depth in both views,
    - bounded reprojection error.

5. **Deformable triangulation optimization**
  - Theory: estimate landmark trajectory across frames with reprojection + temporal/spatial regularization.
  - Code path: `DeformableTriangulation()` (g2o problem with per-frame landmark vertices and regularization terms).
  - Seed strategy uses neighbor depth priors from temporal buffer before optimization.

6. **Graph growth after landmark insertion**
  - Theory: new landmarks must be coupled into DDG to participate in later deformation inference.
  - Code path: after triangulation, new landmarks are connected in `RegularizationGraph` via relative-position edges.

### 8.3 Practical interpretation for agents

- Treat tracking as a two-stage estimator: pose-only stabilization first, then pose+deformation refinement.
- Treat mapping as a continuous balance between:
  - improving existing state consistency (keyframe BA), and
  - adding trustworthy new 3D structure (triangulation path).
- When debugging quality drops, inspect terms in this order:
  1. data association/KLT survival,
  2. reprojection inlier ratio,
  3. regularization graph health,
  4. triangulation acceptance metrics.

## 9) Theory vs Current Code (Drift Notes)

These are common pitfalls for agents reading older docs first.

1. **Historical docs may mention removed datasets/apps**
  - Earlier project materials may reference Hamlyn/Endomapper/simulation flows.
  - Current repo scope is Andromeda-only execution and maintenance.

2. **Environment assumptions drifted**
   - README states testing on Ubuntu 20.04.
   - Current CMake and app dependencies include ROS Jazzy path/package assumptions.

3. **Architecture wording differs from current orchestration**
   - Theory/docs often describe three parallel threads.
   - Current code clearly keeps tracking + mapping in the same `TrackImage` call flow; visualizer is threaded.

4. **API surface vs implementation mismatch exists**
   - `System` header declares stereo/depth tracking methods.
   - Stereo/depth implementations are currently commented out in `system.cc`.

5. **Traceability document date**
   - `result.md` is useful context but dated (analysis snapshot from 2025-02-09).

## 10) Agent Workflow (Recommended First Reads)

When an agent starts a task, read in this order:

1. `apps/andromeda.cc`
2. `modules/SLAM/system.cc`
3. `modules/tracking/tracking.cc`
4. `modules/SLAM/settings.cc` + target dataset YAML
5. `apps/CMakeLists.txt` + root `CMakeLists.txt` + `build.sh`
6. `result.md` and `NR-SLAM.pdf` for theory mapping/background only


## 11) Known Good Commands (Andromeda)

refer to README.MD

## 12) Maintenance Rules for This File

Update this file whenever any of these change:

- app flags or topic contracts in `apps/andromeda.cc`
- settings keys/default behavior in `modules/SLAM/settings.*` or `data/*/settings.yaml`
- pipeline/status behavior in `modules/SLAM/system.cc` or `modules/tracking/tracking.cc`
- build/dependency assumptions in `build.sh` or CMake files
- shell/environment bootstrap requirements used by `build.sh` (for example
  `ros_setup` sourcing)

Keep this file concise and operational: describe current behavior first, keep historical theory notes clearly labeled as such.

## 13) How to fix bugs

- Always perform root-cause analysis instead of patching the symptom
- Be conscious of data model assumptions
- Avoid unnecessary copy 
- Update log outputs
- Update this file in case of updates  