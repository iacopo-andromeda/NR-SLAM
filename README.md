# NR-SLAM: Non-Rigid Monocular SLAM

Version 0.1 (June 28th, 2023)

Authors: [Juan J. Gomez Rodriguez](https://jj-gomez.github.io/), [Jose M. M. Montiel](http://webdiis.unizar.es/~josemari/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/)

NR-SLAM is a monocular deformable SLAM system based on a Dynamic Deformation Graph (DDG) and a visco-elastic deformation model. It supports pinhole and fisheye camera models.

This README is the canonical operational source for:

- project usage and build instructions,
- current Andromeda-first runtime contract,
- paper-to-code traceability summary,
- known theory-vs-implementation drift points.

Detailed historical paper-to-code analysis is maintained in `result.md`.

Video example: [YouTube](https://youtu.be/N-N0ugRjR2s)

## Related Publications

- [NR-SLAM] Juan J. Gomez Rodriguez, Jose M. M. Montiel and Juan D. Tardos, **NR-SLAM: Non-Rigid Monocular SLAM**, *ArXiv*. [PDF](https://arxiv.org/abs/2308.04036)
- [Deformable tracking] Juan J. Gomez Rodriguez, Jose M. M. Montiel and Juan D. Tardos, **Tracking monocular camera pose and deformation for SLAM inside the human body**, *IROS 2022*. [PDF](https://arxiv.org/abs/2204.08309)

## 1. Current Scope (Code Truth)

- Active app entrypoint is `andromeda` (`apps/andromeda.cc`).
- Current repository maintenance scope is Andromeda-first.
- Historical app/dataset flows (Hamlyn, Endomapper, simulation) are not part of the active runtime path in this workspace.
- If paper text and current implementation differ, prefer current code behavior.

## 2. License

NR-SLAM is released under [AGPL license](https://github.com/endomapper/NR-SLAM/LICENSE).

For dependency licenses, see [Dependencies.md](Dependencies.md).

For a closed-source version of NR-SLAM for commercial purposes, contact the authors:

- jjgomez (at) unizar (dot) es
- josemari (at) unizar (dot) es
- tardos (at) unizar (dot) es

If you use NR-SLAM in academic work, cite:

```bibtex
@article{NR-SLAM,
        title={{NR-SLAM}: Non-Rigid Monocular {SLAM}},
        author={Gomez, Juan J. AND Montiel, Jose M. M. AND Tardos, Juan D.},
        journal={ArXiv},
        year={2023}
}
```

## 3. Prerequisites

Originally tested on Ubuntu 20.04.4 LTS. The current workspace also includes ROS Jazzy assumptions in CMake/build flow.

- C++17
- [Pangolin](https://github.com/stevenlovegrove/Pangolin)
- [OpenCV](http://opencv.org) (3.x/4.x)
- [Eigen3](http://eigen.tuxfamily.org) (>= 3.1.0)
- [Boost](https://www.boost.org/)
- [MLPACK](https://www.mlpack.org/)

## 4. Build

Clone:

```bash
git clone https://github.com/endomapper/NR-SLAM NR_SLAM
cd NR_SLAM
```

Build script:

```bash
chmod +x build.sh
./build.sh --target andromeda
```

Debug build:

```bash
./build.sh --dbg --target andromeda
```

Artifacts:

- executables: `build/bin/`
- libraries: `build/lib/`

### Build environment note

`build.sh` runs under `zsh`, and attempts to source `~/.andromeda_profile` when `ros_setup` is not already available, then runs `ros_setup` before CMake. This addresses ROS/ament bootstrap failures such as:

```text
ModuleNotFoundError: No module named 'ament_package'
```

## 5. Andromeda Runtime Contract

Main implementation: `apps/andromeda.cc`

External pose integration:

- `/robot/state` can be decoded and passed into `System::TrackImage(image, external_pose)`.
- system-level pose comparison telemetry is emitted in logs and CSV.

### Input topics

- main image topic: `/image_raw/compressed`
- secondary topic (counted/ignored by SLAM path): `/robot/state`

### CLI flags

- `--dataset_path`: rosbag directory
- `--settings_path`: YAML settings
- `--starting_frame`: range start
- `--end_frame`: range end
- `--range_mode`: `timestamp_ns` (default) or `message_index`
- `--max_images`: max processed main-topic images (`0` means unlimited)
- `--output_dir`: output root. Derived subdirectories are `{output_dir}/map_viz/`, `{output_dir}/viz/`, and `{output_dir}/eval/`.

### Range semantics

- `timestamp_ns` mode:
  - bag iterator uses start/end timestamps,
  - processing stops when `message.timestamp_ns >= end_frame` if `end_frame > 0`.
- `message_index` mode:
  - only main-topic image messages increment the index,
  - `starting_frame`/`end_frame` apply to image index, not global message index.

### Example run

```bash
./build/bin/andromeda \
        --dataset_path /home/galactus/Documents/robot-bags/rosbag2_13-02-2026_08-57-17 \
        --settings_path ./data/andromeda/settings.yaml \
        --starting_frame 1771001959417433296 \
        --end_frame 1771002836540509598 \
        --range_mode timestamp_ns \
        --max_images 200 \
        --output_dir /tmp/slam_out
```

## 6. System Pipeline (Current Implementation)

Main implementation: `modules/SLAM/system.cc`

`System::TrackImage()` runs tracking and mapping sequentially in one call path:

1. preprocess image (gray + CLAHE),
2. generate masks,
3. tracking (`tracker_->TrackImage`),
4. mapping (`mapper_->DoMapping`),
5. visualization update,
6. performance logging (`[PERF]`, CSV via `PerformanceLogger`).

Threading note:

- `MapVisualizer` runs in its own thread,
- tracking and mapping are not parallel threads in current code.

Output note:

- performance CSV and pose comparison CSV are written into `output_dir` when provided, otherwise working directory.

## 7. Tracking and Mapping Behavior

### Tracking status machine

Main implementation: `modules/tracking/tracking.cc`

- `NOT_INITIALIZED`
- `TRACKING`
- `LOST`

LOST handling includes monocular re-bootstrap (`LostModeBootstrap`) with frame stride and recovery thresholds.

### Mapping modes

Main implementation: `modules/mapping/mapping.cc`

- Keyframe mapping path: local deformable BA (`LocalDeformableBundleAdjustment`)
- Frame mapping path: triangulation-focused updates (`LandmarkTriangulation`)

Triangulation model selection policy currently uses a 1.5x rule:

- prefer rigid if rigid successes > `1.5 * deformable` successes,
- prefer deformable if deformable successes >= `1.5 * rigid` successes.

## 8. Settings Contract (Andromeda)

Main files:

- `data/andromeda/settings.yaml`
- `modules/SLAM/settings.h`
- `modules/SLAM/settings.cc`

Active setting groups in current code:

- Camera intrinsics/model (`Camera.*`)
- Masking filter file (`Masking.filterFile`)
- Visualization paths/views (`MapVisualizer.*`)
- Tracking/KLT parameters (`Tracking.klt_*`, LOST thresholds, keyframe cadence)
- Feature extraction (`Features.*`)

## 9. Paper-to-Code Mapping (Quick Reference)

Analysis snapshot: 2026-03-23 (includes external-pose runtime integration updates)

### Core correspondences

- Eq. 1 (elastic regularizer): implemented via `PositionRegularizer` variants
- Eq. 2 (viscous regularizer): implemented via `SpatialRegularizer` variants
- Eq. 3 (pairwise weight): `InterpolationWeight`, exact Gaussian form
- Eq. 4 (edge pruning by stretch): `RegularizationGraph::UpdateConnection`, exact criterion
- Eq. 5 (pose-only tracking optimization): `CameraPoseOptimization`
- Eqs. 6-9 (joint pose + deformation): `CameraPoseAndDeformationOptimization`
- Eqs. 10-12 (deformable BA): `LocalDeformableBundleAdjustment` (absolute-position parameterization)
- Eq. 14 (deformable triangulation): `DeformableTriangulation`

### Confidence summary

- High-confidence direct implementations: most core tracking/mapping equations
- Medium-confidence conceptual equivalents:
        - mid-term association uses optimization-based propagation,
        - DBA uses absolute positions instead of explicit deformation variables,
        - monocular initialization uses an 8-point flavor in current implementation path.

### Important implementation additions beyond paper text

- IQR-based deformation outlier filtering
- lost-point deformation propagation pass
- CLAHE image preprocessing
- DBSCAN-based outlier filtering in initialization-related paths
- LOST mode re-bootstrap behavior
- optional external camera-pose ingestion (`/robot/state`)
- one-time world-frame alignment to external pose
- pose comparison telemetry (`[POSE_CMP_*]` and `pose_comparison.csv`)

## 10. Drift Notes (Theory vs Current Code)

Common pitfalls when reading older docs first:

1. Historical dataset/app references may not match current Andromeda-only runtime scope.
2. Paper/legacy wording may imply three parallel threads; current code runs tracking + mapping sequentially.
3. `System` API still contains stereo/depth surface area, while active path is monocular Andromeda.
4. Build/environment assumptions evolved with ROS Jazzy and `ros_setup` bootstrap in `build.sh`.

## 11. Recommended Reading Order

For implementation work, read in this order:

1. `apps/andromeda.cc`
2. `modules/SLAM/system.cc`
3. `modules/tracking/tracking.cc`
4. `modules/SLAM/settings.cc`
5. `modules/mapping/mapping.cc`
6. `modules/optimization/g2o_optimization.cc`
7. `modules/map/regularization_graph.cc`

## 12. Known Good Commands

Build:

```bash
./build.sh --target andromeda
```

Debug build:

```bash
./build.sh --dbg --target andromeda
```

Run (example): see Section 5.
