#!/usr/bin/env bash
# Source this file to set up the full HIT integration environment.
#
# Workspace chain sourced (bottom → top priority):
#   /opt/ros/noetic
#   → ws_livox          (livox_ros_driver2)
#   → ws_cloud          (livox_cloudpoint_processor)
#   → sentry_planning_ws (ocs2, trajectory_generation, tracking_node)
#   → sim_nav           (bot_sim, hdl_localization, etc.)  [highest priority]
#   + DecisionNode      (decision_node / mcu_communicator) [added to PATH]
#
# Usage:
#   source /path/to/RM_Sentry_2026/HIT_Integrated_test/hit_env.bash
#
# DO NOT source sentry_planning_ws/devel/setup.bash afterwards —
# it will reset ROS_PACKAGE_PATH and lose the workspace priority order.
#
# BUILD ORDER (first-time setup on a new machine):
#   1. cd $REPO_ROOT/Livox-SDK2 && mkdir -p build && cd build && cmake .. && make -j$(nproc)
#   2. cd $REPO_ROOT/HIT_code/sentry_deps/Sophus && mkdir -p build && cd build && cmake .. && sudo make install
#   3. cd $REPO_ROOT/ws_livox            && catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5
#   4. source $REPO_ROOT/ws_livox/devel/setup.bash
#   5. cd $REPO_ROOT/ws_cloud            && catkin_make
#   6. source $REPO_ROOT/ws_cloud/devel/setup.bash
#   7. cd $REPO_ROOT/HIT_code/sentry_planning_ws && catkin_make
#   8. source $REPO_ROOT/HIT_code/sentry_planning_ws/devel/setup.bash
#   9. cd $REPO_ROOT/HIT_Integrated_test/sim_nav && catkin_make
#   10. source $REPO_ROOT/HIT_Integrated_test/sim_nav/devel/setup.bash
#   11. cd $REPO_ROOT/DecisionNode        && catkin_make
#   Then: source $REPO_ROOT/HIT_Integrated_test/hit_env.bash

# ── Auto-detect this repo's root (one level above this script) ──────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ── Source the top of the workspace chain (sim_nav) ─────────────────────────
SIM_NAV_SETUP="${REPO_ROOT}/HIT_Integrated_test/sim_nav/devel/setup.bash"
if [[ ! -f "${SIM_NAV_SETUP}" ]]; then
    echo "[HIT env] ERROR: sim_nav not built yet. Run catkin_make in ${REPO_ROOT}/HIT_Integrated_test/sim_nav first."
    return 1
fi
source "${SIM_NAV_SETUP}"

# ── Add DecisionNode workspace (mcu_communicator) ───────────────────────────
DECISION_NODE_SETUP="${REPO_ROOT}/DecisionNode/devel/setup.bash"
if [[ -f "${DECISION_NODE_SETUP}" ]]; then
    # Extend PATH/PYTHONPATH without resetting the chain: manually add to variables
    export ROS_PACKAGE_PATH="${REPO_ROOT}/DecisionNode/src:${ROS_PACKAGE_PATH}"
    export CMAKE_PREFIX_PATH="${REPO_ROOT}/DecisionNode/devel:${CMAKE_PREFIX_PATH}"
    export LD_LIBRARY_PATH="${REPO_ROOT}/DecisionNode/devel/lib:${LD_LIBRARY_PATH}"
else
    echo "[HIT env] WARNING: DecisionNode not built yet (mcu_communicator will be missing)."
    echo "           Run: cd ${REPO_ROOT}/DecisionNode && catkin_make"
fi

echo "[HIT env] REPO_ROOT: ${REPO_ROOT}"
echo "[HIT env] Active workspace chain:"
python3 -c "
import os
paths = os.environ.get('CMAKE_PREFIX_PATH','').split(':')
for p in paths:
    if p: print('  ' + p)
"
echo "[HIT env] bot_sim resolves to: $(rospack find bot_sim 2>/dev/null || echo NOT FOUND)"
echo "[HIT env] decision_node resolves to: $(rospack find decision_node 2>/dev/null || echo NOT FOUND)"
