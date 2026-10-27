#!/usr/bin/env python3
"""
Collision monitor: polls flag files independently, writes results immediately.
- USV: circle radius=1.0m
- Vessel: rectangle 2m×5m (width×length)
- CPU% recorded every 2s

Runs in a background thread so ROS shutdown doesn't block writes.
"""

import math
import os
import signal
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

USV_RADIUS = 1.0
VESSEL_W = 2.0
VESSEL_H = 5.0
CORRIDOR_Y_MIN = -18.0
CORRIDOR_Y_MAX = 18.0
FLAG_DIR = '/home/lu/paper2/core_ws'


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def circle_vs_rect(cx, cy, cr, rx, ry, ryaw, rw, rh):
    dx, dy = cx - rx, cy - ry
    c, s = math.cos(-ryaw), math.sin(-ryaw)
    lx = dx * c - dy * s
    ly = dx * s + dy * c
    hw, hh = rw / 2.0, rh / 2.0
    cpx = max(-hw, min(hw, lx))
    cpy = max(-hh, min(hh, ly))
    d = math.hypot(lx - cpx, ly - cpy)
    return d <= cr, d


class CollisionMonitor(Node):
    def __init__(self):
        super().__init__('collision_monitor')
        self.declare_parameter('usv_ids', '1,2,3')
        self.declare_parameter('vessel_names', '')
        self.declare_parameter('max_vessels', 8)
        self.declare_parameter('run', 0)
        self.declare_parameter('algorithm', 'ours')
        self.declare_parameter('difficulty', 'medium')
        self.declare_parameter('results_dir', '/home/lu/paper2/results')

        usv_ids_str = str(self.get_parameter('usv_ids').value)
        vessel_names_str = str(self.get_parameter('vessel_names').value)
        self.run_id = int(self.get_parameter('run').value)
        self.algorithm = str(self.get_parameter('algorithm').value)
        self.difficulty = str(self.get_parameter('difficulty').value)
        self.results_dir = str(self.get_parameter('results_dir').value)
        self.max_vessels = int(self.get_parameter('max_vessels').value)

        self.usv_ids = [int(x) for x in usv_ids_str.split(',') if x.strip()]
        self.vessel_names = [x.strip() for x in vessel_names_str.split(',') if x.strip()][:self.max_vessels]

        # Volatile state (updated by ROS callbacks)
        self.usv_state = {}
        self.vessel_state = {}
        self._lock = threading.Lock()

        # Output files
        os.makedirs(f'{self.results_dir}/{self.algorithm}', exist_ok=True)
        base = f'{self.results_dir}/{self.algorithm}/{self.difficulty}'
        self.results_path = f'{base}_results.csv'
        self.traj_path = f'{base}_traj.csv'
        self.cpu_path = f'{base}_cpu.csv'

        for path, header in [
            (self.results_path, 'run,agent,duration_s,timeout,coll_vessel,coll_neighbor,coll_wall,min_clearance\n'),
            (self.traj_path, 'run,agent,t,x,y\n'),
            (self.cpu_path, 'run,agent,t,cpu_pct\n'),
        ]:
            if not os.path.exists(path):
                with open(path, 'w') as f:
                    f.write(header)

        # ROS subscriptions
        for uid in self.usv_ids:
            self.create_subscription(Odometry, f'/usv_{uid}/odom',
                                     lambda m, u=uid: self._usv_cb(m, u), 10)
        for vn in self.vessel_names:
            self.create_subscription(Odometry, f'/{vn}/odom',
                                     lambda m, v=vn: self._vessel_cb(m, v), 10)

        # Background writer thread (polls flags independently)
        self._done = False
        self._writer = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer.start()
        # CPU recorder timer
        self.create_timer(2.0, self._record_cpu)

        self.get_logger().info(
            f'Monitor: {len(self.usv_ids)} USVs, {len(self.vessel_names)} vessels, '
            f'{self.algorithm}/{self.difficulty} run={self.run_id}')

    def _usv_cb(self, msg, uid):
        with self._lock:
            self.usv_state[uid] = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'v': msg.twist.twist.linear.x,
                'yaw': quat_to_yaw(msg.pose.pose.orientation),
            }

    def _vessel_cb(self, msg, name):
        with self._lock:
            self.vessel_state[name] = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'yaw': quat_to_yaw(msg.pose.pose.orientation),
            }

    def _record_cpu(self):
        for uid in self.usv_ids:
            try:
                out = os.popen(f'ps aux | grep "usv_planner_node.*usv_{uid}" | grep -v grep').read().strip()
                if out:
                    cpu = float(out.split()[2])
                    t = time.time()
                    with open(self.cpu_path, 'a') as f:
                        f.write(f'{self.run_id},{uid},{t:.1f},{cpu:.2f}\n')
            except Exception:
                pass

    def _writer_loop(self):
        """Background thread: polls flags at 1Hz, writes results + trajectory."""
        coll_vessel = {uid: False for uid in self.usv_ids}
        coll_neighbor = {uid: False for uid in self.usv_ids}
        coll_wall = {uid: False for uid in self.usv_ids}
        min_clearance = {uid: float('inf') for uid in self.usv_ids}
        arrival_time = {uid: None for uid in self.usv_ids}
        timeout_flag = {uid: False for uid in self.usv_ids}
        traj_buf = {uid: [] for uid in self.usv_ids}
        results_written = {uid: False for uid in self.usv_ids}
        start = time.time()

        while not self._done:
            now = time.time()
            elapsed = now - start

            with self._lock:
                usv = dict(self.usv_state)
                ves = dict(self.vessel_state)

            # --- Collect trajectory + check collisions ---
            for uid in self.usv_ids:
                if uid not in usv:
                    continue
                u = usv[uid]
                traj_buf[uid].append((elapsed, u['x'], u['y']))

                # Vessel collision (circle vs rect)
                if not coll_vessel[uid]:
                    for v in ves.values():
                        coll, _ = circle_vs_rect(u['x'], u['y'], USV_RADIUS,
                                                 v['x'], v['y'], v['yaw'],
                                                 VESSEL_W, VESSEL_H)
                        if coll:
                            coll_vessel[uid] = True
                            break

                # Neighbor collision
                if not coll_neighbor[uid]:
                    for oid in self.usv_ids:
                        if oid == uid or oid not in usv:
                            continue
                        o = usv[oid]
                        if math.hypot(u['x'] - o['x'], u['y'] - o['y']) < USV_RADIUS * 2:
                            coll_neighbor[uid] = True
                            break

                # Wall collision
                if not coll_wall[uid]:
                    if u['y'] < CORRIDOR_Y_MIN + USV_RADIUS or u['y'] > CORRIDOR_Y_MAX - USV_RADIUS:
                        coll_wall[uid] = True

                # Min clearance
                for v in ves.values():
                    _, d = circle_vs_rect(u['x'], u['y'], 0.0,
                                          v['x'], v['y'], v['yaw'],
                                          VESSEL_W, VESSEL_H)
                    min_clearance[uid] = min(min_clearance[uid], d)

            # --- Check flag files & write results ---
            for uid in self.usv_ids:
                fp = f'{FLAG_DIR}/trial_done_{uid}.flag'
                if os.path.exists(fp) and not results_written[uid]:
                    try:
                        with open(fp) as f:
                            content = f.read().strip()
                    except Exception:
                        content = ''
                    # Parse ROS simulation time from flag content
                    # Format: "passed 33.1s" or "timeout 120.0s" or "33.1,True"
                    sim_dur = elapsed  # fallback to wall time
                    if content:
                        try:
                            # Try "passed 33.1s" format
                            sim_dur = float(content.split()[1].rstrip('s'))
                        except (ValueError, IndexError):
                            try:
                                # Try "33.1,True" format
                                sim_dur = float(content.split(',')[0])
                            except (ValueError, IndexError):
                                pass
                    arrival_time[uid] = sim_dur
                    timeout_flag[uid] = content.startswith('timeout')
                    with open(self.results_path, 'a') as f:
                        f.write(f'{self.run_id},{uid},{sim_dur:.2f},{1 if timeout_flag[uid] else 0},'
                                f'{1 if coll_vessel[uid] else 0},'
                                f'{1 if coll_neighbor[uid] else 0},'
                                f'{1 if coll_wall[uid] else 0},'
                                f'{min_clearance[uid]:.3f}\n')
                    results_written[uid] = True
                    self.get_logger().info(f'USV-{uid} result written: dur={elapsed:.1f}s')

            # --- Flush trajectory every 1s ---
            lines = []
            for uid in self.usv_ids:
                for t, x, y in traj_buf[uid]:
                    lines.append(f'{self.run_id},{uid},{t:.2f},{x:.3f},{y:.3f}\n')
                traj_buf[uid].clear()
            if lines:
                with open(self.traj_path, 'a') as f:
                    f.writelines(lines)

            # All done?
            if all(results_written.values()):
                self.get_logger().info(f'=== Run {self.run_id} complete ===')
                break

            # Safety
            if elapsed > 180:
                self.get_logger().warn('Timeout — forcing exit')
                for uid in self.usv_ids:
                    if not results_written[uid]:
                        with open(self.results_path, 'a') as f:
                            f.write(f'{self.run_id},{uid},999.0,1,0,0,0,0.0\n')
                break

            time.sleep(0.5)

        self._done = True


def main():
    rclpy.init()
    node = CollisionMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
