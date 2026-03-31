#!/usr/bin/env bash
# Source this file to set up the full HIT integration environment.
# The HIT sim_nav workspace was built extending sentry_planning_ws,
# so this single source gives you:
#   HIT/sim_nav (bot_sim, local_frame_bridge, map_image_publisher) [highest priority]
#   sentry_planning_ws (trajectory_generation, tracking_node, waypoint_generator)
#   Navigation-filter-test / ws_livox / opt/ros/noetic
#
# Usage:
#   source /path/to/RM_Sentry_2026/HIT_Integrated_test/hit_env.bash
#
# DO NOT source sentry_planning_ws/devel/setup.bash afterwards —
# it will reset ROS_PACKAGE_PATH and lose the HIT sim_nav priority.

# Resolve paths relative to this script's location
_HIT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_REPO_ROOT="$(cd "${_HIT_DIR}/.." && pwd)"

source "${_HIT_DIR}/sim_nav/devel/setup.bash"

# Add DecisionNode workspace so roslaunch can find decision_node (mcu_communicator)
# Points to top-level AstarTraining/DecisionNode (canonical, up-to-date copy)
_DECISION_NODE="$(cd "${_REPO_ROOT}/.." && pwd)/DecisionNode"
export ROS_PACKAGE_PATH="${_DECISION_NODE}/src:${ROS_PACKAGE_PATH}"
export CMAKE_PREFIX_PATH="${_DECISION_NODE}/devel:${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="${_DECISION_NODE}/devel/lib:${LD_LIBRARY_PATH}"

echo "[HIT env] Active workspace chain:"
python3 << 'EOF'
import os
paths = os.environ.get('CMAKE_PREFIX_PATH', '').split(':')
for p in paths:
    if p:
        print('  ' + p)
EOF
echo "[HIT env] bot_sim resolves to: $(rospack find bot_sim 2>/dev/null || echo NOT FOUND)"
