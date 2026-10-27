#!/usr/bin/env python3
"""
Exit monitor v2: subscribes to USV odom, writes trial_done_{uid}.flag
when x > pass_x_threshold. Uses ROS clock time (supports use_sim_time).
Also shuts down all processes when all USVs pass.
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time as TimeMsg

PASS_X = float(os.environ.get('MULTI_USV_PASS_X', '30.0'))
AGENTS = int(os.environ.get('MULTI_USV_AGENTS', '3'))
FLAG_DIR = '/home/lu/paper2/core_ws'
TIMEOUT = int(os.environ.get('MULTI_USV_TIMEOUT', '120'))


class ExitMonitor(Node):
    def __init__(self):
        super().__init__('exit_monitor')
        self.passed = {}
        self.started = False
        self.start_stamp = None
        self.wall_start = time.time()

        for uid in range(1, AGENTS + 1):
            self.create_subscription(
                Odometry, f'/usv_{uid}/odom',
                lambda msg, u=uid: self.cb(msg, u), 10)

        self.create_timer(2.0, self.check)
        self.get_logger().info(f'ExitMonitor: {AGENTS} USVs, pass_x={PASS_X}')

    def get_elapsed(self, msg_stamp):
        """Get elapsed seconds from odom header stamp (true sim time)."""
        if not self.started:
            return 0.0
        return (msg_stamp.sec - self.start_stamp.sec) + (msg_stamp.nanosec - self.start_stamp.nanosec) / 1e9

    def cb(self, msg, uid):
        if uid in self.passed:
            return
        x = msg.pose.pose.position.x
        if x > PASS_X:
            if not self.started:
                self.start_stamp = msg.header.stamp
                self.started = True
            elapsed = self.get_elapsed(msg.header.stamp)
            self.passed[uid] = elapsed
            with open(f'{FLAG_DIR}/trial_done_{uid}.flag', 'w') as f:
                f.write(f'{elapsed:.1f},True\n')
            self.get_logger().info(f'USV-{uid} passed x={PASS_X} at {elapsed:.1f}s')

    def check(self):
        wall_elapsed = time.time() - self.wall_start

        if len(self.passed) >= AGENTS:
            self.get_logger().info(f'All {AGENTS} passed, shutting down')
            time.sleep(3.0)
            os.system("pkill -9 -f teb_wrapper_node 2>/dev/null")
            os.system("pkill -9 -f usv_planner_node 2>/dev/null")
            os.system("pkill -9 -f prm_node 2>/dev/null")
            os.system("pkill -9 -f dynamic_obstacle 2>/dev/null")
            os.system("pkill -9 -f usv_simulator 2>/dev/null")
            os.system("pkill -9 -f rviz2 2>/dev/null")
            raise SystemExit(0)

        if wall_elapsed > TIMEOUT:
            self.get_logger().warn(f'Timeout {TIMEOUT}s reached')
            for uid in range(1, AGENTS + 1):
                if uid not in self.passed:
                    elapsed = 0.0
                    with open(f'{FLAG_DIR}/trial_done_{uid}.flag', 'w') as f:
                        f.write(f'{elapsed:.1f},False\n')
            time.sleep(2.0)
            raise SystemExit(0)


def main():
    rclpy.init()
    node = ExitMonitor()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
