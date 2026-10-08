#!/usr/bin/env bash
# run_dashboard_v2.sh — dashboard + RViz2 on the v2 (DexPilot) retargeting pipeline.
#
# run_dashboard.sh now runs this same pipeline, so the two are equivalent; this
# one is kept as the explicit name, and run_dashboard.sh additionally takes
# --v1 to fall back to the old geometric retargeting.
#
#   ./run_dashboard_v2.sh real
#   ./run_dashboard_v2.sh sim --hand right
DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
exec "$DIR/run_teleop_v2.sh" "${@:-real}" --classic --rviz
