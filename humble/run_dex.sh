#!/usr/bin/env bash
# ==============================================================================
# run_dex.sh — 손끝 거리 리타게팅(DexPilot)으로 실물 로봇 텔레옵, 대시보드 + RViz2
#
# 이것 하나로 끝난다:
#   ./run_dex.sh                 # 연결된 손을 찾아 그 손으로 실행
#   ./run_dex.sh --hand right    # 두 손이 다 연결됐을 때 오른손 지정
#   ./run_dex.sh --smooth 9      # 떨리면 평활화를 키운다
#
# 콕핏 화면으로 보려면 --cockpit 을 붙인다. 나머지 옵션은 run_teleop_v6_5.sh 와 같다.
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
exec "$SCRIPT_DIR/run_teleop_v6_5.sh" real --classic --dexpilot "$@"
