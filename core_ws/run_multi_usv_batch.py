#!/usr/bin/env python3
import csv
import os
import signal
import subprocess
import time
from pathlib import Path

ROOT = Path('/home/lu/paper2/core_ws')
RESULT = ROOT / 'results_multi_usv.csv'
SETUP = ROOT / 'install/setup.bash'
TCP_XML = ROOT / 'fastdds_tcp.xml'
LAUNCH = 'ros2 launch multi_usv_planner planner.launch.py use_rviz:=false use_models:=false'
TRIALS = 5
WAIT_PER_TRIAL = 600  # max safety timeout (sync with episode_timeout_s_ in C++)
VACUUM_SEC = 6       # 轮间真空时间：确保上一轮进程全死+IPC清干净
ARRIVAL_CHECK_INTERVAL = 5.0  # check if all agents arrived every 5s
AGENT_COUNT = 2


PROC_PATTERNS = [
    'heron_single_node',
    'usv_planner_node_exe',
    'prm_node',
    'usv_simulator',
    'kinematic_vessel_node',
    'dynamic_obstacle_node',
    'scene_visualizer_node',
    'topology_selector_node',
    'online_trajectory',
    'rviz2',
    'robot_state_publisher',
]


def _run_bash(command: str):
    subprocess.run(
        command,
        shell=True,
        cwd='/home/lu/paper2',
        executable='/bin/bash',
        check=False,
    )


def cleanup_ipc():
    _run_bash('rm -f /dev/shm/fastrtps_* /tmp/fastrtps_port* /tmp/lock* || true')


def kill_all():
    """杀进程+清理IPC+真空等待，确保上一轮完全释放"""
    # SIGINT → SIGTERM → SIGKILL 三轮递进
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        for pattern in PROC_PATTERNS:
            _run_bash(f'pkill -{sig.value} -f "{pattern}" || true')
        time.sleep(2 if sig == signal.SIGKILL else 1)

    # 额外清理：所有 ros2 相关进程
    _run_bash('pkill -9 -f "ros2 launch" || true')
    _run_bash('pkill -9 -f "ros2.*multi_usv" || true')
    time.sleep(1)

    cleanup_ipc()

    # 真空等待：共享内存/端口/系统完全释放
    time.sleep(VACUUM_SEC)


def clear_done_flags():
    for f in ROOT.glob('trial_done_*.flag'):
        f.unlink()


def all_agents_done():
    """Check if all AGENT_COUNT agents have written done flags."""
    done_ids = set()
    for f in ROOT.glob('trial_done_*.flag'):
        done_ids.add(int(f.stem.split('_')[-1]))
    return len(done_ids) >= AGENT_COUNT


def run_trial(i: int):
    print(f'=== trial {i} (agents={AGENT_COUNT}) ===', flush=True)
    kill_all()
    clear_done_flags()
    if RESULT.exists():
        RESULT.unlink()
    env = os.environ.copy()
    env['MULTI_USV_SEED'] = str(i)
    env['MULTI_USV_AGENTS'] = str(AGENT_COUNT)
    env['RMW_FASTRTPS_TRANSPORT_SHARED_MEMORY'] = '0'
    env['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(TCP_XML)
    env['MULTI_USV_DYNAMIC_OBS'] = '1'  # dynamic obstacles enabled
    cmd = f'source "{SETUP}" && {LAUNCH}'
    proc = subprocess.Popen(cmd, shell=True, cwd=str(ROOT), executable='/bin/bash', env=env)

    # Poll for all agents done, with max timeout
    start = time.time()
    while time.time() - start < WAIT_PER_TRIAL:
        time.sleep(ARRIVAL_CHECK_INTERVAL)
        if all_agents_done():
            elapsed = time.time() - start
            print(f'  all {AGENT_COUNT} agents arrived at goal in {elapsed:.0f}s ✓', flush=True)
            break

    kill_all()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()

    if not RESULT.exists():
        print('no result file', flush=True)
        return []

    with RESULT.open() as f:
        rows = list(csv.DictReader(f))
    print(f'rows={len(rows)}', flush=True)
    return rows


def summarize(rows):
    total = len(rows)
    if total == 0:
        return {}
    avg_duration = sum(float(r['duration_s']) for r in rows) / total
    timeout_rate = sum(int(r['timeout']) for r in rows) / total
    dyn_coll = sum(int(r['collision_dynamic']) for r in rows)
    sta_coll = sum(int(r['collision_static']) for r in rows)
    nb_coll = sum(int(r['collision_neighbor']) for r in rows)
    return {
        'count': total,
        'avg_duration_s': round(avg_duration, 3),
        'timeout_rate': round(timeout_rate, 3),
        'dynamic_collision_count': dyn_coll,
        'static_collision_count': sta_coll,
        'neighbor_collision_count': nb_coll,
    }


def main():
    out_path = ROOT / 'results_multi_usv_trials.csv'

    # 读取已有数据，确定起始 trial 编号
    existing_rows = []
    if out_path.exists():
        with out_path.open('r') as f:
            existing_rows = list(csv.DictReader(f))
    start_trial = 1
    if existing_rows:
        existing_trials = sorted(set(int(r['trial']) for r in existing_rows))
        start_trial = max(existing_trials) + 1
        print(f'found {len(existing_rows)} existing rows (trials {min(existing_trials)}-{max(existing_trials)}), continuing from trial {start_trial}')

    all_rows = list(existing_rows)
    for i in range(start_trial, start_trial + TRIALS):
        rows = run_trial(i)
        for r in rows:
            r['trial'] = i
        all_rows.extend(rows)
        # 每轮后立即写盘，防止中途中断丢失
        if all_rows:
            fieldnames = ['trial'] + [k for k in all_rows[0].keys() if k != 'trial']
            with out_path.open('w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(all_rows)
        print(f'written: {out_path}')
        print('summary:', summarize(all_rows))
    print(f'\nfinal total: {summarize(all_rows)}')


if __name__ == '__main__':
    main()
