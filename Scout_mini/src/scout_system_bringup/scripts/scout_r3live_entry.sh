#!/usr/bin/env bash
# Discoverable from the normal Scout workspace; the optional overlay owns its environment.
set -eo pipefail
task_entry="${HOME}/livox_fastlio/optional/r3live_ws/start_scout_r3live.sh"
test -x "$task_entry" || { echo "R3LIVE is not installed: $task_entry" >&2; exit 1; }
exec "$task_entry" --session-node "$@"
