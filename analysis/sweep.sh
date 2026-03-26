#!/usr/bin/env bash
# Parameter sweep to improve Tracking % on the andromeda dataset.
#
# Each experiment gets its own directory under /tmp/slam_out/<name>/ containing:
#   config.yaml          – copy of data/andromeda/settings.yaml with overrides
#   stdout.log           – combined stdout + stderr (tee'd)
#   slam.log             – absl log file written by the binary
#   slam_performance.csv – per-frame performance CSV written directly here
#   pose_comparison.csv  – per-frame pose comparison CSV written directly here
#
# Results are appended to /tmp/slam_out/summary.txt as each run completes.
set -euo pipefail

WDIR=/home/galactus/workspace/NR-SLAM
BIN="$WDIR/build/bin/andromeda"
DATASET="$WDIR/data/andromeda"
BASE_YAML="$WDIR/data/andromeda/settings.yaml"
OUT_ROOT=/tmp/slam_out
SUMMARY="$OUT_ROOT/summary.txt"

mkdir -p "$OUT_ROOT"
> "$SUMMARY"   # start fresh

# ---------------------------------------------------------------------------
# make_config <name> [Key: value ...]
#   Creates $OUT_ROOT/<name>/config.yaml from a copy of the base YAML.
#   Then applies each "Key: value" override using sed on the copy.
# ---------------------------------------------------------------------------
make_config() {
    local name="$1"; shift
    local dir="$OUT_ROOT/$name"
    mkdir -p "$dir"
    local cfg="$dir/config.yaml"
    cp "$BASE_YAML" "$cfg"

    # Apply caller-supplied overrides: each arg is "Key: value"
    for kv in "$@"; do
        key="${kv%%:*}"
        val="${kv#*: }"
        # Escape dots in key for sed BRE so they match literally
        local escaped_key
        escaped_key=$(printf '%s' "$key" | sed 's/\./\\./g')
        sed -i "s|^${escaped_key}:.*|${key}: ${val}|" "$cfg"
    done

    echo "$cfg"
}

# ---------------------------------------------------------------------------
# run_exp <name> <description> [Key: value ...]
#   Builds the config, runs the binary, saves logs, extracts Tracking %.
# ---------------------------------------------------------------------------
run_exp() {
    local name="$1"
    local descr="$2"; shift 2
    local dir="$OUT_ROOT/$name"

    local cfg
    cfg=$(make_config "$name" "$@")

    echo ""
    echo "========================================================"
    echo " Experiment : $name"
    echo " Description: $descr"
    echo " Config     : $cfg"
    echo "========================================================"

    "$BIN" \
        --dataset_path "$DATASET" \
        --settings_path "$cfg" \
        --starting_frame 0 \
        --end_frame 1000 \
        --output_dir "$dir" \
        |& tee "$dir/stdout.log" || true

    # Extract summary numbers from stdout.log
    local tracking_pct lost_pct init_pct
    tracking_pct=$(grep -oP 'TRACKING\s+:\s+\d+\s+\(\K[\d.]+(?=%)' "$dir/stdout.log" 2>/dev/null | tail -1 || echo "N/A")
    lost_pct=$(    grep -oP 'LOST\s+:\s+\d+\s+\(\K[\d.]+(?=%)'     "$dir/stdout.log" 2>/dev/null | tail -1 || echo "N/A")
    init_pct=$(    grep -oP 'NOT_INITIALIZED\s+:\s+\d+\s+\(\K[\d.]+(?=%)' "$dir/stdout.log" 2>/dev/null | tail -1 || echo "N/A")

    printf '%-30s | Tracking%%=%-5s | Lost%%=%-5s | Init%%=%-4s | %s\n' \
        "$name" "$tracking_pct" "$lost_pct" "$init_pct" "$descr" \
        | tee -a "$SUMMARY"
}

# ===========================================================================
# Experiments
# ===========================================================================

# E0 – baseline: no overrides, pure copy of default settings
run_exp "E0_baseline" \
    "Baseline: all defaults"

# E1 – wider KLT window + extra pyramid level
run_exp "E1_wide_window" \
    "klt_window_size=31, klt_max_level=5" \
    "Tracking.klt_window_size: 31" \
    "Tracking.klt_max_level: 5"

# E2 – softer SSIM acceptance threshold
run_exp "E2_soft_ssim" \
    "klt_min_SSIM=0.5" \
    "Tracking.klt_min_SSIM: 0.5"

# E3 – more and denser features
run_exp "E3_more_features" \
    "max_corners=2000, quality_level=0.05, min_distance=5" \
    "Features.max_corners: 2000" \
    "Features.quality_level: 0.05" \
    "Features.min_distance: 5"

# E4 – more frequent keyframes
run_exp "E4_fast_keyframes" \
    "images_to_insert_keyframe=3" \
    "Tracking.images_to_insert_keyframe: 3"

# E5 – lenient lost: lower abort threshold, longer grace, stride=1
run_exp "E5_lenient_lost" \
    "min_tracked_points_abort=5, grace_frames=10, bootstrap_stride=1" \
    "Tracking.min_tracked_points_abort: 5" \
    "Tracking.lost_recovery_grace_frames: 10" \
    "Tracking.lost_bootstrap_frame_stride: 1"

# E6 – longer map-point lifetime
run_exp "E6_long_stale" \
    "stale_mappoint_max_age_frames=60" \
    "Tracking.stale_mappoint_max_age_frames: 60"

# E7 – more triangulation lookback
run_exp "E7_more_lookback" \
    "triangulation_track_lookback_frames=10" \
    "Tracking.triangulation_track_lookback_frames: 10"

# E8 – combined: all best individual changes together
run_exp "E8_combined" \
    "win=31 lvl=5 SSIM=0.5 corners=2000 q=0.05 dist=5 kf=3 abort=5 grace=10 stride=1 stale=60 lookback=10" \
    "Tracking.klt_window_size: 31" \
    "Tracking.klt_max_level: 5" \
    "Tracking.klt_min_SSIM: 0.5" \
    "Features.max_corners: 2000" \
    "Features.quality_level: 0.05" \
    "Features.min_distance: 5" \
    "Tracking.images_to_insert_keyframe: 3" \
    "Tracking.min_tracked_points_abort: 5" \
    "Tracking.lost_recovery_grace_frames: 10" \
    "Tracking.lost_bootstrap_frame_stride: 1" \
    "Tracking.stale_mappoint_max_age_frames: 60" \
    "Tracking.triangulation_track_lookback_frames: 10"

# E9 – high iterations + very soft SSIM (blurry/fast-motion frames)
run_exp "E9_high_iters_soft_ssim" \
    "klt_max_iters=30, klt_min_SSIM=0.45, klt_epsilon=0.00005" \
    "Tracking.klt_max_iters: 30" \
    "Tracking.klt_min_SSIM: 0.45" \
    "Tracking.klt_epsilon: 0.00005"

# E10 – fine features (tighter grid, lower quality threshold)
run_exp "E10_fine_features" \
    "corners=1500, quality=0.07, min_dist=4, stale=50, kf=4" \
    "Features.max_corners: 1500" \
    "Features.quality_level: 0.07" \
    "Features.min_distance: 4" \
    "Tracking.stale_mappoint_max_age_frames: 50" \
    "Tracking.images_to_insert_keyframe: 4"

# ===========================================================================
echo ""
echo "========== SWEEP COMPLETE =========="
echo "Results: $SUMMARY"
echo ""
cat "$SUMMARY"
