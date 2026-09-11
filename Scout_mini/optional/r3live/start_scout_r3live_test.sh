#!/usr/bin/env bash
# Authorized six-scene extrinsic; automatic health checks and bounded recording.
set -eo pipefail
task_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_trial="$task_workspace/config/accepted_20260911"
test -f "$task_trial/trial_calibration.yaml" || {
  echo "Reviewed calibration missing: $task_trial/trial_calibration.yaml" >&2
  exit 1
}
exec "$task_workspace/start_scout_r3live.sh" \
  calibration_file:="$task_trial/trial_calibration.yaml" \
  output_root:="$task_workspace/logs/indoor_tests" record_bag:=true test_duration:=600 "$@"
