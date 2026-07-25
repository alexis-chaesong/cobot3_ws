#!/usr/bin/env bash
set -e
ISAAC="$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$ISAAC/exts/isaacsim.ros2.bridge/humble/lib"
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$ISAAC/python.sh" isaacpjt/M0609/17_dual_task_select_tool_changer_integrated.py "$@"
