#!/usr/bin/env python3
"""
Exit monitor: checks for all trial_done_N.flag files.
When all AGENTS flags exist, shuts down the ROS 2 launch.
"""
import os
import sys
import time

AGENTS = int(os.environ.get("MULTI_USV_AGENTS", "3"))
BASE_DIR = "/home/lu/paper2/core_ws"
CHECK_INTERVAL = 2.0


def all_done():
    for i in range(1, AGENTS + 1):
        if not os.path.exists(f"{BASE_DIR}/trial_done_{i}.flag"):
            return False
    return True


def main():
    while not all_done():
        time.sleep(CHECK_INTERVAL)

    # Small delay so metrics are flushed
    time.sleep(3.0)

    print(f"[exit_monitor] All {AGENTS} USVs passed. Shutting down...")
    os.system("pkill -9 -f rviz2 2>/dev/null")
    os.system("pkill -9 -f usv_planner_node 2>/dev/null")
    os.system("pkill -9 -f prm_node 2>/dev/null")
    os.system("pkill -9 -f dynamic_obstacle 2>/dev/null")
    os.system("pkill -9 -f usv_simulator 2>/dev/null")
    os.system("pkill -9 -f scene_visualizer 2>/dev/null")
    os.system("pkill -9 -f robot_state_publisher 2>/dev/null")
    os.system("pkill -9 -f odom_tf_broadcaster 2>/dev/null")
    os.system("pkill -9 -f static_transform_publisher 2>/dev/null")
    print("[exit_monitor] All processes terminated.")


if __name__ == "__main__":
    main()
