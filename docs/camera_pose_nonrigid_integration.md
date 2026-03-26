# Camera Pose Vector Integration Plan (Tailored to NR-SLAM)

## Scope and assumption

This plan is tailored to the current NR-SLAM codebase and assumes each frame can receive an external vector that represents camera pose in a base frame.

Notation used in this document:

- `T_base_cam(t)`: camera pose at frame `t` from the external vector.
- In this repository, frame pose is currently stored in `Frame::camera_transformation_world_` via `MutableCameraTransformationWorld()` and read via `CameraTransformationWorld()`.
- For implementation simplicity, treat `base` as the SLAM world frame unless explicitly configured otherwise.

Primary objective:

- keep a strict separation between camera egomotion (rigid global transform) and non-rigid map motion (local deformation).

---

## Best integration points

The following are the highest-value integration points in this codebase.

### 1) Input ingestion and timestamp alignment

#### Where

- `apps/andromeda.cc` (already consumes both `/image_raw/compressed` and `/robot/state`).
- `modules/SLAM/system.h` and `modules/SLAM/system.cc` (`System::TrackImage`).

#### Plan

- Add a pose-aware track entrypoint, e.g. `System::TrackImageWithPose(const cv::Mat&, const Sophus::SE3f& T_base_cam, uint64_t timestamp_ns, bool pose_valid)`.
- In `apps/andromeda.cc`, cache latest `/robot/state` pose vector by timestamp and attach the best-aligned pose to each image.
- Add alignment policy for image/pose association (nearest, previous, interpolation-ready).

#### Affects and effected result

- Affects: all downstream modules that consume frame pose seeds and temporal pose history.
- Effected result: deterministic per-frame pose prior availability; reduced ambiguity at tracking and optimization stages.

#### Not affecting

- Does not change feature extraction (`ShiTomasiCV`) logic.
- Does not change camera intrinsics/projection model.
- Does not directly change map-point deformation equations.

#### Trade-offs

- Tight timestamp gating reduces wrong priors but can drop usable frames.
- Loose gating keeps throughput but increases risk of mixing stale pose with current image.

#### Settings to add

- `PoseInput.enabled` (bool, default `1`)
- `PoseInput.topic` (string, default `"/robot/state"`)
- `PoseInput.timestamp_tolerance_ms` (float, default `20.0`)
- `PoseInput.sync_mode` (string: `nearest|previous`, default `nearest`)
- `PoseInput.drop_frame_if_pose_missing` (bool, default `0`)

---

### 2) Frame-level pose representation (measurement vs estimate)

#### Where

- `modules/map/frame.h` and `modules/map/frame.cc`.

#### Plan

- Keep existing `camera_transformation_world_` as the optimized/working camera pose.
- Add distinct storage for external pose measurement and metadata, e.g.:
	- `has_external_pose`
	- `external_camera_transformation_world`
	- `external_pose_confidence` (or covariance scalar)
	- `external_pose_timestamp_ns`
- Use explicit API names so call sites cannot confuse measurement with estimate.

#### Affects and effected result

- Affects: tracking seed logic and optimization prior construction.
- Effected result: explicit separation of "what sensor says" vs "what optimizer currently estimates".

#### Not affecting

- Does not alter observation storage model (`observations_` vs legacy vectors).
- Does not alter map-point indexing (`mappoint_id_to_index_`, `index_to_mappoint_id_`).

#### Trade-offs

- Extra fields increase frame memory and API surface.
- Strongly improves debuggability and prevents accidental overwrite of optimized pose by raw sensor input.

#### Settings to add

- `PoseInput.store_measurement_in_frame` (bool, default `1`)
- `PoseInput.confidence_default` (float in `[0,1]`, default `0.7`)

---

### 3) Tracking seed and motion model fusion

#### Where

- `modules/tracking/tracking.cc`:
	- `Tracking::CameraPoseEstimation()`
	- `Tracking::TrackCameraAndDeformation()`
	- motion model update in `CameraPoseAndDeformationEstimation()`

#### Plan

- Replace single-source seed (`motion_model_ * current`) with configurable fusion of:
	- motion-model prediction, and
	- external pose prior.
- Define policy when external pose is invalid/missing:
	- fallback to motion model only.
- Keep the two-stage structure already present:
	- pose-only optimization (`CameraPoseOptimization`)
	- pose+deformation optimization (`CameraPoseAndDeformationOptimization`)

#### Affects and effected result

- Affects: initial pose sent to optimization and KLT projection consistency.
- Effected result: better convergence basin for pose solve, lower risk of deformation absorbing global camera motion.

#### Not affecting

- Does not change KLT residual definitions or status labels.
- Does not change keyframe insertion policy directly.

#### Trade-offs

- Heavy reliance on external prior helps during fast motion but can inject bias if prior is wrong.
- Motion-model-only is more self-consistent but drifts more during weak visual conditions.

#### Settings to add

- `Tracking.pose_seed_mode` (string: `motion_model|external|fused`, default `fused`)
- `Tracking.pose_seed_external_weight` (float `[0,1]`, default `0.6`)
- `Tracking.pose_seed_use_when_lost` (bool, default `1`)
- `Tracking.pose_seed_reset_on_jump` (bool, default `1`)
- `Tracking.pose_seed_jump_translation_m` (float, default `0.5`)
- `Tracking.pose_seed_jump_angle_deg` (float, default `25.0`)

---

### 4) Pose-only optimization prior (rigid camera term)

#### Where

- `modules/optimization/g2o_optimization.cc` in `CameraPoseOptimization(...)`.

#### Plan

- Add an explicit SE(3) prior term on camera vertex using external pose measurement.
- Use robust kernel and configurable information weight.
- Keep reprojection residual as primary geometric term; prior should guide, not dominate by default.

#### Affects and effected result

- Affects: camera vertex update during the pose-only stage.
- Effected result: more stable rigid pose estimate before entering deformation optimization.

#### Not affecting

- Does not directly move deformation vertices (none exist in pose-only stage).
- Does not change regularization graph connectivity.

#### Trade-offs

- Strong prior suppresses drift and jitter.
- Over-strong prior can lock wrong pose and increase reprojection residuals.

#### Settings to add

- `Optimization.pose_prior.enabled` (bool, default `1`)
- `Optimization.pose_prior.translation_sigma_m` (float, default `0.10`)
- `Optimization.pose_prior.rotation_sigma_deg` (float, default `5.0`)
- `Optimization.pose_prior.robust_delta` (float, default `5.99`)
- `Optimization.pose_prior.max_weight_scale` (float, default `1.0`)

---

### 5) Joint pose + deformation optimization balance

#### Where

- `modules/optimization/g2o_optimization.cc` in `CameraPoseAndDeformationOptimization(...)`.

#### Plan

- Keep existing variable split (camera vertex + per-point deformation vertices).
- Add the same external pose prior to the camera vertex in this joint stage.
- Add schedule control so camera term and deformation regularizers can be rebalanced (e.g., first iterations emphasize camera prior/reprojection, later iterations allow more local deformation).
- Keep current outlier handling and median deformation tracking.

#### Affects and effected result

- Affects: optimization decomposition between rigid camera updates and local deformation updates.
- Effected result: cleaner interpretation of residuals; deformation captures scene motion rather than camera trajectory errors.

#### Not affecting

- Does not change map point identity lifecycle.
- Does not change temporal buffer insertion rules.

#### Trade-offs

- Strong camera prior + strong spatial regularization yields stable maps but can underfit real non-rigid events.
- Weak camera prior + weak regularization captures rich deformation but risks egomotion leakage into deformation field.

#### Settings to add

- `Optimization.joint_pose_deform.camera_prior_weight` (float, default `1.0`)
- `Optimization.joint_pose_deform.reprojection_weight` (float, default `1.0`)
- `Optimization.joint_pose_deform.position_regularizer_weight` (float, default `1.0`)
- `Optimization.joint_pose_deform.spatial_regularizer_weight` (float, default `1.0`)
- `Optimization.joint_pose_deform.iteration_schedule` (string: `fixed|camera_first`, default `camera_first`)

---

### 6) Temporal buffer semantics (pose source of truth)

#### Where

- `modules/map/map.cc` (`SetLastFrame`)
- `modules/map/temporal_buffer.h` and `modules/map/temporal_buffer.cc`

#### Plan

- Ensure snapshots carry the final optimized pose used by geometry modules.
- Optionally carry both optimized and external pose for diagnostics.
- Keep triangulation and rigidity checks (`GetCameraTransformWorld`, `CheckRigidity`) bound to optimized pose by default.

#### Affects and effected result

- Affects: rigid and deformable triangulation in mapping (`TriangulateMidPoint`, `DeformableTriangulation`).
- Effected result: consistent camera trajectory assumptions across tracking and mapping.

#### Not affecting

- Does not change feature candidate selection criteria by itself.
- Does not change map visualizer rendering API.

#### Trade-offs

- Storing both pose versions increases traceability but uses more memory and log bandwidth.
- Storing only one pose is simpler but hides root cause when rigid/non-rigid separation degrades.

#### Settings to add

- `TemporalBuffer.store_external_pose` (bool, default `1`)
- `TemporalBuffer.pose_source_for_triangulation` (string: `optimized|external`, default `optimized`)
- `TemporalBuffer.pose_source_for_rigidity` (string: `optimized|external`, default `optimized`)

---

### 7) LOST-mode and bootstrap behavior with external pose

#### Where

- `modules/tracking/tracking.cc`:
	- LOST transition in `TrackImage(...)`
	- `LostModeBootstrap(...)`

#### Plan

- Preserve current fallback bootstrap path.
- When external pose is available, allow world anchoring during recovery so re-bootstrap does not create pose discontinuity.
- Keep current `pose_at_lost_entry_` logic and extend it with optional external-pose anchor checks.

#### Affects and effected result

- Affects: continuity of camera trajectory and map frame consistency after LOST events.
- Effected result: less trajectory jump and lower risk of false non-rigid deformation spikes immediately after relocalization.

#### Not affecting

- Does not alter base monocular initializer math when external pose is unavailable.
- Does not change regular operation while tracking status remains `TRACKING`.

#### Trade-offs

- Aggressive external anchoring speeds recovery but can lock in sensor outliers.
- Conservative anchoring is safer but may require more frames to recover.

#### Settings to add

- `Tracking.lost_mode.use_external_pose_anchor` (bool, default `1`)
- `Tracking.lost_mode.external_anchor_weight` (float, default `0.7`)
- `Tracking.lost_mode.max_anchor_error_translation_m` (float, default `1.0`)
- `Tracking.lost_mode.max_anchor_error_angle_deg` (float, default `35.0`)

---

### 8) Diagnostics to verify camera-vs-map motion separation

#### Where

- `modules/tracking/tracking.cc` logs
- `modules/optimization/g2o_optimization.cc` logs
- `modules/SLAM/system.cc` performance logging path

#### Plan

- Add per-frame decomposition metrics:
	- camera prior residual (translation/rotation)
	- reprojection residual before/after optimization
	- median deformation magnitude
	- ratio: deformation energy / total residual energy
- Emit warnings when deformation grows while camera prior residual is low (symptom of over-flexible deformation model).

#### Affects and effected result

- Affects: observability and tuning speed.
- Effected result: faster root-cause analysis when rigid/non-rigid coupling drifts.

#### Not affecting

- Does not change numerical optimization outputs directly.
- Does not alter map topology.

#### Trade-offs

- More logging adds runtime overhead and larger logs.
- Essential for safe tuning of new pose-prior weights.

#### Settings to add

- `Debug.motion_decomposition.enable` (bool, default `1`)
- `Debug.motion_decomposition.log_every_n_frames` (int, default `1`)
- `Debug.motion_decomposition.warn_deform_to_pose_ratio` (float, default `2.0`)

---

## What this plan explicitly protects

The plan enforces this distinction:

- Camera motion path (rigid):
	- input pose vector -> frame external pose -> camera vertex prior -> optimized `CameraTransformationWorld()`
- Map independent motion path (non-rigid):
	- deformation vertices + spatial/position regularizers + reprojection residual after camera compensation

Any implementation should fail review if it allows deformation terms to absorb systematic camera pose residuals without constraint.

---

## Recommended rollout order

1. Ingestion + frame storage separation (Integration points 1 and 2).
2. Tracking seed fusion (Integration point 3).
3. Pose prior in pose-only optimization (Integration point 4).
4. Pose prior and weight schedule in joint optimization (Integration point 5).
5. Temporal buffer semantics and diagnostics (Integration points 6 and 8).
6. LOST-mode anchoring improvements (Integration point 7).

This order minimizes risk: data plumbing first, then optimization constraints, then recovery and diagnostics.

---

## Minimal acceptance criteria

The implementation is considered successful when all are true:

- external pose can be attached to each processed frame with explicit validity metadata,
- camera prior affects camera vertex updates in both optimization stages,
- deformation magnitude does not spike on pure camera motion sequences,
- pose discontinuity across LOST recovery is reduced,
- diagnostics can show whether residual motion is assigned to camera or deformation.
