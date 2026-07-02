#!/usr/bin/env python3
"""
Multi-USV experiment runner & analyzer.

Methodology from "Topology-Driven Parallel Trajectory Optimization in Dynamic
Environments" (IEEE TRO 2025, de Groot et al., Section V-C).

Metrics (cf. Table II):
  1. Task duration — time for each agent to pass x>pass_x_threshold (done=1)
  2. Safety rate   — % experiments with zero collisions (dynamic/static/neighbor)
  3. Smoothness    — std dev of speed across trajectory

Usage:
  # First run: 20 experiments, 30s each
  ./run_experiments.py --n 20 --duration 30

  # Append 20 more (total 40)
  ./run_experiments.py --n 20 --duration 30 --start-seed 70 --append

  # Analyze existing CSV only
  ./run_experiments.py --no-run

  # Final: 100 experiments
  export EXPERIMENTS_DIR=/path/to/save && ./run_experiments.py --n 100 --duration 30
"""

import argparse
import csv
import os
import signal
import subprocess
import sys
import time
import numpy as np

RESULTS_CSV = os.environ.get("MULTI_USV_RESULTS_CSV", "")
if not RESULTS_CSV:
    debris = os.environ.get("MULTI_USV_DEBRIS_COUNT", "12")
    colregs = os.environ.get("MULTI_USV_COLREGS", "1")
    tag = f"ours_d{debris}_colregs{colregs}"
    RESULTS_CSV = f"/home/lu/paper2/core_ws/results_{tag}.csv"

SUMMARY_LOG = RESULTS_CSV.replace(".csv", "_summary.csv")
LAUNCH_DIR = "/home/lu/paper2/core_ws"
NO_KILL = False  # set via --no-kill flag
AGENTS = 3
EXP_DURATION_S = 120  # seconds per experiment; agents that finish before this get done=1

# CSV columns (no done column since v2):
# t,agent,colregs,coll_dyn,coll_st,coll_nb,clr_dyn,clr_st,clr_nb,speed
COL_T = 0
COL_AGENT = 1
COL_COLREGS = 2
COL_COLL_DYN = 3
COL_COLL_ST = 4
COL_COLL_NB = 5
COL_CLR_DYN = 6
COL_CLR_ST = 7
COL_CLR_NB = 8
COL_SPEED = 9


# ─────────────────────────────────────────────────────────
#  Process management
# ─────────────────────────────────────────────────────────

def kill_ros_processes():
    """Kill any remaining ros2/exe processes between experiments."""
    global NO_KILL
    if NO_KILL:
        return
    # Be specific: avoid matching this script's own path (contains "usv_planner")
    patterns = [
        "ros2 launch",
        "usv_planner_node",
        "dynamic_obstacle",
        "prm_node",
        "gzserver",
        "gzclient",
        "gazebo",
    ]
    for pat in patterns:
        subprocess.run(["pkill", "-9", "-f", pat],
                       capture_output=True)
    time.sleep(2.0)  # wait for clean exit


def run_single_experiment(seed: int, duration: float) -> bool:
    """Run one experiment via ros2 launch. Returns True if OK."""
    env = os.environ.copy()
    env["MULTI_USV_SEED"] = str(seed)
    env["MULTI_USV_AGENTS"] = str(AGENTS)
    env["MULTI_USV_DYNAMIC_OBS"] = "1"
    env["MULTI_USV_DEBRIS_COUNT"] = os.environ.get("MULTI_USV_DEBRIS_COUNT", "12")
    env["MULTI_USV_RESULTS_CSV"] = RESULTS_CSV

    # Source workspace before launching — required for new terminals
    setup_script = os.path.join(LAUNCH_DIR, "install", "setup.bash")
    bash_cmd = (
        f"source {setup_script} && "
        f"timeout -s TERM {int(duration) + 5} "
        f"ros2 launch multi_usv_planner planner.launch.py "
        f"use_rviz:=false use_models:=false"
    )
    cmd = ["bash", "-c", bash_cmd]

    # Use a separate process group so timeout SIGTERM doesn't reach us
    proc = subprocess.run(cmd, cwd=LAUNCH_DIR, env=env,
                          capture_output=True, text=True,
                          preexec_fn=os.setpgrp)

    for line in proc.stderr.split("\n"):
        ll = line.lower()
        if "process has died" in ll or "traceback" in ll:
            return False
    return True


# ─────────────────────────────────────────────────────────
#  Data parsing
# ─────────────────────────────────────────────────────────

def parse_csv() -> tuple:
    """
    Parse CSV into per-cycle rows and summary rows.

    CSV layout per experiment:
      [per-cycle t,agent,...] [summary agent_id,...]×3  [gap ~2.7s]

    Per-cycle: 10 cols (t,agent,colregs,coll_dyn,coll_st,coll_nb,clr_dyn,clr_st,clr_nb,speed)
    Summary:   9 cols (agent_id,duration_s,timeout,coll_dyn,coll_st,coll_nb,min_dyn_clr,min_st_clr,min_nb_clr)

    Summary lines are written at the END of each agent's run (before the gap).
    Collision flags in summaries are the ground truth — cumulative final state.
    Per-cycle collision flags are also cumulative but less reliable for per-experiment stats.

    Returns (cycle_rows, summaries_list) where summaries_list has entries in CSV order.
    """
    if not os.path.exists(RESULTS_CSV):
        return [], []

    cycle_rows = []
    summaries_list = []

    with open(RESULTS_CSV, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            # Summary line: agent_id(1-4),duration_s,timeout,...
            if len(row) == 9:
                try:
                    aid = int(row[0])
                    if 1 <= aid <= AGENTS:
                        summaries_list.append(dict(
                            agent=aid,
                            duration=float(row[1]),
                            timeout=int(row[2]),
                            coll_dyn=int(row[3]),
                            coll_st=int(row[4]),
                            coll_nb=int(row[5]),
                            clr_dyn=float(row[6]),
                            clr_st=float(row[7]),
                            clr_nb=float(row[8]),
                        ))
                        continue
                except (ValueError, IndexError):
                    pass

            # Per-cycle line
            if len(row) < 10:
                continue
            try:
                t = float(row[COL_T])
                if t < 1e9:  # skip old-format data
                    continue
                a = int(row[COL_AGENT])
                cycle_rows.append(dict(
                    t=t, agent=a,
                    colregs=int(row[COL_COLREGS]),
                    coll_dyn=int(row[COL_COLL_DYN]),
                    coll_st=int(row[COL_COLL_ST]),
                    coll_nb=int(row[COL_COLL_NB]),
                    clr_dyn=float(row[COL_CLR_DYN]),
                    clr_st=float(row[COL_CLR_ST]),
                    clr_nb=float(row[COL_CLR_NB]),
                    speed=float(row[COL_SPEED]),
                ))
            except (ValueError, IndexError):
                continue

    return cycle_rows, summaries_list


def detect_experiments(cycle_rows: list, gap_s: float = 2.5) -> list:
    """Split cycle_rows into experiments by timestamp gaps."""
    if not cycle_rows:
        return []

    sorted_rows = sorted(cycle_rows, key=lambda r: r["t"])
    groups = []
    current = [sorted_rows[0]]

    for i in range(1, len(sorted_rows)):
        gap = sorted_rows[i]["t"] - sorted_rows[i-1]["t"]
        if gap > gap_s:
            groups.append(current)
            current = [sorted_rows[i]]
        else:
            current.append(sorted_rows[i])

    if current:
        groups.append(current)
    return groups


def match_summaries(summaries_list: list, n_experiments: int) -> list:
    """
    Match summary lines to experiments.
    Each experiment produces AGENTS summary lines.
    The summaries are in CSV order, grouped after each experiment's cycle data.
    Corner case: last experiment may not have summaries yet (if still running).
    Returns list of list-of-dicts: [exp0_summaries, exp1_summaries, ...]
    """
    matched = []
    for i in range(n_experiments):
        start = i * AGENTS
        end = start + AGENTS
        if end <= len(summaries_list):
            matched.append(summaries_list[start:end])
        else:
            matched.append([])  # no summaries for this experiment yet
    return matched


def analyze_experiment(exp_rows: list, exp_summaries: list, exp_idx: int) -> dict:
    """
    Analyze a single experiment.
    
    Collision data: from summary lines (ground truth — final cumulative flags).
    Task duration: from summary lines (precise time from episode_start to finish).
    Speed/smoothness: from per-cycle data.
    """
    if not exp_rows:
        return None

    agent_stats = {}
    for aid in range(1, AGENTS + 1):
        ar = sorted([r for r in exp_rows if r["agent"] == aid],
                    key=lambda r: r["t"])
        if not ar:
            continue

        # Find summary for this agent
        summary = None
        for s in exp_summaries:
            if s["agent"] == aid:
                summary = s
                break

        if summary:
            duration = summary["duration"]
            coll_dyn = bool(summary["coll_dyn"])
            coll_st = bool(summary["coll_st"])
            coll_nb = bool(summary["coll_nb"])
            timeout = bool(summary["timeout"])
        else:
            # Fallback: use time span
            duration = max(r["t"] for r in ar) - min(r["t"] for r in ar)
            coll_dyn = any(r["coll_dyn"] for r in ar)
            coll_st = any(r["coll_st"] for r in ar)
            coll_nb = any(r["coll_nb"] for r in ar)
            timeout = False

        speeds = np.array([r["speed"] for r in ar])

        agent_stats[aid] = dict(
            duration=duration,
            n_cycles=len(ar),
            speed_mean=float(np.mean(speeds)) if len(speeds) > 0 else 0,
            speed_std=float(np.std(speeds, ddof=1)) if len(speeds) > 1 else 0,
            coll_dyn=coll_dyn,
            coll_st=coll_st,
            coll_nb=coll_nb,
            coll_any=coll_dyn or coll_st or coll_nb,
            finished=not timeout,
        )

    if not agent_stats:
        return None

    all_speeds = np.array([r["speed"] for r in exp_rows])

    return dict(
        exp_idx=exp_idx,
        agents=agent_stats,
        n_agents=len(agent_stats),
        task_duration=max(s["duration"] for s in agent_stats.values()),
        any_collision=any(s["coll_any"] for s in agent_stats.values()),
        speed_mean=float(np.mean(all_speeds)) if len(all_speeds) > 0 else 0,
        speed_std=float(np.std(all_speeds, ddof=1)) if len(all_speeds) > 1 else 0,
        all_finished=all(s["finished"] for s in agent_stats.values()),
        n_colregs=sum(1 for r in exp_rows if r["colregs"] > 0),
    )


def compute_summary(experiments: list) -> dict:
    """Aggregate across experiments — matches Table II format."""
    if not experiments:
        return {}

    durations = np.array([e["task_duration"] for e in experiments])
    coll_rates = np.array([1 if e["any_collision"] else 0 for e in experiments])

    # Per-agent
    agent_durs = {a: [] for a in range(1, AGENTS + 1)}
    agent_coll = {a: [] for a in range(1, AGENTS + 1)}
    agent_finish = {a: [] for a in range(1, AGENTS + 1)}
    for exp in experiments:
        for aid, s in exp["agents"].items():
            agent_durs[aid].append(s["duration"])
            agent_coll[aid].append(1 if s["coll_any"] else 0)
            agent_finish[aid].append(1 if s["finished"] else 0)

    all_speeds = np.concatenate([
        np.array([r["speed"] for r in exp_rows])
        for exp_rows in [e.get("_raw", []) for e in experiments]
        for r in exp_rows
    ]) if experiments else np.array([])

    stats = dict(
        n_experiments=len(experiments),
        n_agents=AGENTS,
        # Task duration (cf. Table II)
        duration_mean=float(np.mean(durations)),
        duration_std=float(np.std(durations, ddof=1)),
        duration_median=float(np.median(durations)),
        duration_min=float(np.min(durations)),
        duration_max=float(np.max(durations)),
        # Safety
        safe_count=int(np.sum(1 - coll_rates)),
        safe_rate=float(np.mean(1 - coll_rates) * 100),
        collision_count=int(np.sum(coll_rates)),
        collision_rate=float(np.mean(coll_rates) * 100),
        # Finish rate
        all_finished_count=sum(1 for e in experiments if e["all_finished"]),
        all_finished_rate=sum(1 for e in experiments if e["all_finished"]) / max(len(experiments), 1) * 100,
        # Smoothness
        speed_mean_global=float(np.mean(all_speeds)) if len(all_speeds) > 0 else 0,
        speed_std_global=float(np.std(all_speeds, ddof=1)) if len(all_speeds) > 1 else 0,
    )

    for aid in range(1, AGENTS + 1):
        d = np.array(agent_durs[aid])
        stats[f"agent_{aid}_duration_mean"] = float(np.mean(d)) if len(d) > 0 else 0
        stats[f"agent_{aid}_duration_std"] = float(np.std(d, ddof=1)) if len(d) > 1 else 0
        stats[f"agent_{aid}_safe_count"] = int(np.sum(1 - np.array(agent_coll[aid])))
        stats[f"agent_{aid}_safe_rate"] = float(np.mean(1 - np.array(agent_coll[aid])) * 100)
        stats[f"agent_{aid}_finish_rate"] = float(np.mean(np.array(agent_finish[aid])) * 100)

    return stats


# ─────────────────────────────────────────────────────────
#  Reporting
# ─────────────────────────────────────────────────────────

HEADER_SHOWN = False

def print_header():
    global HEADER_SHOWN
    if HEADER_SHOWN:
        return
    HEADER_SHOWN = True
    print("\n" + "=" * 74)
    print("  EXPERIMENT SUMMARY — Table II (de Groot et al., TRO 2025)")
    print("=" * 74)


def print_results(stats: dict, show_detail: bool = True):
    print_config(stats)
    print_task_duration_table(stats)
    print_safety_table(stats)

    if show_detail:
        print_per_experiment()

    print()
    write_summary_csv(stats)


def print_config(stats: dict):
    print(f"\n  Configuration:")
    print(f"    Experiments  : {stats.get('n_experiments', 'N/A')} × {stats.get('n_agents', 4)} agents")
    print(f"    Environment  : 4 USVs + 4 dynamic obstacle vessels")
    print(f"    Solver       : Acados EXTERNAL+EXACT+MIRROR + ellipsoid soft penalty")
    print(f"    Finish rate  : {stats.get('all_finished_rate', 0):.0f}% (all 4 agents passed x > threshold)")


def print_task_duration_table(stats: dict):
    print(f"\n  ┌──────────────────────────────────┬──────────────┬──────────────┐")
    print(f"  │ Task duration (s)                │      Mean    │      Std     │")
    print(f"  ├──────────────────────────────────┼──────────────┼──────────────┤")

    dm = stats.get("duration_mean", 0)
    ds = stats.get("duration_std", 0)
    dmed = stats.get("duration_median", 0)
    print(f"  │ Aggregate (max across agents)     │ {dm:>10.2f}  │ {ds:>10.2f}  │")
    print(f"  │ Median                            │ {dmed:>10.2f}  │              │")
    print(f"  │ Min / Max                         │ {stats.get('duration_min', 0):>7.2f}  │ {stats.get('duration_max', 0):>10.2f}  │")

    for aid in range(1, AGENTS + 1):
        dm_a = stats.get(f"agent_{aid}_duration_mean", 0)
        ds_a = stats.get(f"agent_{aid}_duration_std", 0)
        fr_a = stats.get(f"agent_{aid}_finish_rate", 0)
        print(f"  │   Agent {aid}               │ {dm_a:>10.2f}  │ {ds_a:>10.2f}  │")

    print(f"  ├──────────────────────────────────┼──────────────┼──────────────┤")
    print(f"  │ Smoothness                       │     Value    │              │")
    print(f"  ├──────────────────────────────────┼──────────────┼──────────────┤")
    print(f"  │ Speed mean (m/s)                 │ {stats.get('speed_mean_global', 0):>10.3f}  │              │")
    print(f"  │ Speed σ (m/s)                    │ {stats.get('speed_std_global', 0):>10.3f}  │              │")
    print(f"  └──────────────────────────────────┴──────────────┴──────────────┘")


def print_safety_table(stats: dict):
    print(f"\n  ┌──────────────────────────────────┬──────────────────────────────┐")
    print(f"  │ Safety metric                    │         Value                │")
    print(f"  ├──────────────────────────────────┼──────────────────────────────┤")

    safe_rate = stats.get("safe_rate", 0)
    coll_rate = stats.get("collision_rate", 0)
    safe_cnt = stats.get("safe_count", 0)
    coll_cnt = stats.get("collision_count", 0)
    n = stats.get("n_experiments", 1)
    print(f"  │ Safe experiments (no collision)   │ {safe_cnt:>3d}/{n:>3d}  ({safe_rate:>5.1f}%)           │")
    print(f"  │ Any collision                     │ {coll_cnt:>3d}/{n:>3d}  ({coll_rate:>5.1f}%)           │")

    for aid in range(1, AGENTS + 1):
        safe_a = stats.get(f"agent_{aid}_safe_rate", 0)
        cnt = stats.get(f"agent_{aid}_safe_count", 0)
        print(f"  │   Agent {aid} safe             │ {cnt:>3d}/{n:>3d}  ({safe_a:>5.1f}%)           │")

    print(f"  └──────────────────────────────────┴──────────────────────────────┘")


def print_per_experiment():
    print(f"\n  Per-experiment:")
    print(f"  {'exp':>4s}  {'dur':>6s}  {'coll':>6s}  {'fin':>4s}  {'spd_m':>6s}  {'spd_s':>6s}")
    print(f"  {'────':>4s}  {'──────':>6s}  {'──────':>6s}  {'────':>4s}  {'──────':>6s}  {'──────':>6s}")
    for exp in analyzed:
        coll = "✗COLL" if exp["any_collision"] else "✓SAFE"
        fin = "✓" if exp["all_finished"] else "✗"
        print(f"  {exp['exp_idx']:>4d}  {exp['task_duration']:>6.1f}  {coll:>6s}  {fin:>4s}  {exp['speed_mean']:>6.3f}  {exp['speed_std']:>6.3f}")


# ─────────────────────────────────────────────────────────
#  CSV I/O
# ─────────────────────────────────────────────────────────

def write_summary_csv(stats: dict):
    fieldnames = [
        "n_experiments", "n_agents",
        "duration_mean", "duration_std", "duration_median",
        "duration_min", "duration_max",
        "safe_count", "safe_rate", "collision_count", "collision_rate",
        "all_finished_count", "all_finished_rate",
        "speed_mean_global", "speed_std_global",
    ]
    for aid in range(1, AGENTS + 1):
        fieldnames += [
            f"agent_{aid}_duration_mean", f"agent_{aid}_duration_std",
            f"agent_{aid}_safe_count", f"agent_{aid}_safe_rate",
            f"agent_{aid}_finish_rate",
        ]

    with open(SUMMARY_LOG, "w") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerow({k: stats.get(k, "") for k in fieldnames})
    print(f"  Summary → {SUMMARY_LOG}")


# ─────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Multi-USV experiment runner")
    parser.add_argument("--n", type=int, default=20, help="Number of experiments")
    parser.add_argument("--duration", type=int, default=EXP_DURATION_S,
                        help="Duration per experiment (s)")
    parser.add_argument("--start-seed", type=int, default=70, help="Starting seed")
    parser.add_argument("--append", action="store_true",
                        help="Append to existing CSV (don't backup)")
    parser.add_argument("--no-run", action="store_true",
                        help="Only analyze existing CSV")
    parser.add_argument("--cleanup", action="store_true",
                        help="Kill stale ROS processes before anything")
    parser.add_argument("--no-kill", action="store_true",
                        help="Skip killing ROS processes between experiments")
    args = parser.parse_args()
    global NO_KILL
    NO_KILL = args.no_kill

    if args.cleanup:
        print("  Cleaning up stale ROS processes...")
        kill_ros_processes()

    # ── Run experiments ──
    if not args.no_run:
        print(f"\n{'=' * 74}")
        print(f"  Multi-USV Experiment Runner")
        print(f"  {args.n} experiments × {args.duration}s × {AGENTS} agents")
        print(f"  Seeds {args.start_seed} → {args.start_seed + args.n - 1}")
        print(f"{'=' * 74}\n")

        if not args.append and os.path.exists(RESULTS_CSV):
            bak = RESULTS_CSV + f".bak.{int(time.time())}"
            os.rename(RESULTS_CSV, bak)
            print(f"  Backed up → {bak}")

        t_start = time.time()
        for i in range(args.n):
            seed = args.start_seed + i
            t0 = time.time()
            print(f"  [{i+1}/{args.n}] seed={seed} ...", end=" ", flush=True)

            # Kill stale processes before each experiment
            kill_ros_processes()
            ok = run_single_experiment(seed, args.duration)

            dt = time.time() - t0
            print(f"{'OK' if ok else 'WARN'} ({dt:.0f}s)")

        total = time.time() - t_start
        avg = total / args.n
        print(f"\n  Done: {args.n} exps in {total:.0f}s ({avg:.1f}s/exp)")

        # Clean up after done
        kill_ros_processes()

    # ── Analyze ──
    print(f"\n{'─' * 74}")
    print(f"  Analyzing {RESULTS_CSV} ...")

    cycle_rows, summaries_list = parse_csv()
    print(f"  Cycle rows: {len(cycle_rows)}, summary lines: {len(summaries_list)}")

    if not cycle_rows:
        print("  No data found. Run experiments first.")
        return

    experiments_data = detect_experiments(cycle_rows)
    print(f"  Experiments detected: {len(experiments_data)}")

    if len(experiments_data) < 1:
        print("  No experiments detected.")
        return

    # Match summaries to experiments (each exp produces AGENTS summaries in order)
    matched_summaries = match_summaries(summaries_list, len(experiments_data))
    n_with_summaries = sum(1 for m in matched_summaries if len(m) == AGENTS)
    print(f"  Experiments with complete summaries: {n_with_summaries}")

    global analyzed
    analyzed = []
    for i, exp_rows in enumerate(experiments_data):
        result = analyze_experiment(exp_rows, matched_summaries[i], i)
        if result:
            result["_raw"] = exp_rows
            analyzed.append(result)

    if not analyzed:
        print("  No experiments could be analyzed.")
        return

    print(f"  Successfully analyzed: {len(analyzed)}")

    # Summary
    print_header()
    stats = compute_summary(analyzed)
    print_results(stats, show_detail=(len(analyzed) <= 30))


if __name__ == "__main__":
    analyzed = []
    main()
