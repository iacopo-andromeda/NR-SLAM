#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 /path/to/nrslam.log"
  exit 1
fi

LOGFILE="$1"

awk '
function extract_value(line, pattern,    token) {
  if (match(line, pattern)) {
    token = substr(line, RSTART, RLENGTH)
    sub(/^[^=]*=/, "", token)
    return token
  }
  return ""
}

BEGIN {
  track_lines=0; klt_lines=0;
}
# ---------------- TRACK_METRICS ----------------
$0 ~ /\[TRACK_METRICS\]/ {
  track_lines++;

  v = extract_value($0, "abort=[01]")
  if (v != "") { aborts += v; abort_n++ }

  v = extract_value($0, "reuse_gain=-?[0-9]+")
  if (v != "") { reuse += v; reuse_n++ }

  v = extract_value($0, "ms_update_triangulated=[0-9]+")
  if (v != "") { mut += v; mut_n++ }

  v = extract_value($0, "ms_track_deformation=[0-9]+")
  if (v != "") { mtd += v; mtd_n++ }

  v = extract_value($0, "ms_point_reuse=[0-9]+")
  if (v != "") { mpr += v; mpr_n++ }

  v = extract_value($0, "ms_keyframe_insertion=[0-9]+")
  if (v != "") { mki += v; mki_n++ }

  v = extract_value($0, "ms_total=[0-9]+")
  if (v != "") {
    mtot += v; mtot_n++;
    if (v <= 33) rt33++;
  }

  v = extract_value($0, "tracked_before_reuse=[0-9]+")
  if (v != "") { tbr += v; tbr_n++ }

  v = extract_value($0, "tracked_after_reuse=[0-9]+")
  if (v != "") { tar += v; tar_n++ }
}

# ---------------- KLT Tracking Results ----------------
$0 ~ /\[KLT Tracking\] Results/ {
  klt_lines++;

  v = extract_value($0, "survival=[0-9.]+")
  if (v != "") { surv += v; surv_n++ }

  v = extract_value($0, "BadFeature=[0-9]+")
  if (v != "") { bf += v }

  v = extract_value($0, "OutOfBounds=[0-9]+")
  if (v != "") { oob += v }

  v = extract_value($0, "BadDisp=[0-9]+")
  if (v != "") { bd += v }

  v = extract_value($0, "LowSSIM=[0-9]+")
  if (v != "") { ls += v }
}

# ---------------- POINT_REUSE ----------------
$0 ~ /\[POINT_REUSE\]/ {
  pr_lines++;

  v = extract_value($0, "candidates_in_image=[0-9]+")
  if (v != "") { cand += v; cand_n++ }

  v = extract_value($0, "klt_tracked=[0-9]+")
  if (v != "") { klt += v; klt_n++ }

  v = extract_value($0, "reproj_rejected=[0-9]+")
  if (v != "") { rej += v; rej_n++ }

  v = extract_value($0, "reused=[0-9]+")
  if (v != "") { reused += v; reused_n++ }

  v = extract_value($0, "updated_existing=[0-9]+")
  if (v != "") { upd += v; upd_n++ }

  v = extract_value($0, "inserted_new=[0-9]+")
  if (v != "") { ins += v; ins_n++ }
}

END {
  print "================ NR-SLAM Tracking Summary ================";
  print "";

  print "[TRACK_METRICS]";
  printf("lines=%d\n", track_lines);
  if (abort_n) printf("abort_rate=%.2f%% (%d/%d)\n", 100.0*aborts/abort_n, aborts, abort_n);
  if (reuse_n) printf("avg_reuse_gain=%.2f\n", reuse/reuse_n);
  if (tbr_n)   printf("avg_tracked_before_reuse=%.2f\n", tbr/tbr_n);
  if (tar_n)   printf("avg_tracked_after_reuse=%.2f\n", tar/tar_n);
  if (mut_n)   printf("avg_ms_update_triangulated=%.2f\n", mut/mut_n);
  if (mtd_n)   printf("avg_ms_track_deformation=%.2f\n", mtd/mtd_n);
  if (mpr_n)   printf("avg_ms_point_reuse=%.2f\n", mpr/mpr_n);
  if (mki_n)   printf("avg_ms_keyframe_insertion=%.2f\n", mki/mki_n);
  if (mtot_n)  printf("avg_ms_total=%.2f | <=33ms: %.2f%% (%d/%d)\n",
                      mtot/mtot_n, 100.0*rt33/mtot_n, rt33, mtot_n);

  print "";
  print "[KLT Tracking]";
  printf("lines=%d\n", klt_lines);
  if (surv_n) printf("avg_survival=%.4f\n", surv/surv_n);
  printf("failures_total: BadFeature=%d OutOfBounds=%d BadDisp=%d LowSSIM=%d\n",
         bf, oob, bd, ls);

  print "";
  print "[POINT_REUSE]";
  printf("lines=%d\n", pr_lines);
  if (cand_n)   printf("avg_candidates_in_image=%.2f\n", cand/cand_n);
  if (klt_n)    printf("avg_klt_tracked=%.2f\n", klt/klt_n);
  if (rej_n)    printf("avg_reproj_rejected=%.2f\n", rej/rej_n);
  if (reused_n) printf("avg_reused=%.2f\n", reused/reused_n);
  if (upd_n)    printf("avg_updated_existing=%.2f\n", upd/upd_n);
  if (ins_n)    printf("avg_inserted_new=%.2f\n", ins/ins_n);

  print "";
  print "==========================================================";
}
' "$LOGFILE"