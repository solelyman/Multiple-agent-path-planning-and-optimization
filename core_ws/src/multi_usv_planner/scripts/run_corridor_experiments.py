
import argparse
import csv
import os
import subprocess
import sys
import time
import math

BASE_DIR = "/home/lu/paper2/core_ws"
RESULTS_DIR = "/home/lu/paper2/core_ws"
AGENTS = 3
EXP_TIMEOUT_S = 300  # 5 min per experiment max

# ─────────────────────────────────────────────────────────
#  Process management
# ─────────────────────────────────────────────────────────

def kill_clean():
    """Kill all corridor-related processes."""
    patterns = [
        "planner_corridor",
        "usv_planner_node",
        "dynamic_obstacle",
        "prm_node",
        "usv_simulator",
        "scene_visualizer",
        "robot_state_publisher",
        "odom_tf_broadcaster",
        "static_transform_publisher.*map_to_odom",
        "rviz2",
    ]
    for pat in patterns:
        subprocess.run(["pkill", "-9", "-f", pat],
                       capture_output=True)
    time.sleep(1.0)


def remove_done_flags():
    for i in range(1, AGENTS + 1):
        path = f"{BASE_DIR}/trial_done_{i}.flag"
        if os.path.exists(path):
            os.remove(path)


def count_done_flags():
    count = 0
    for i in range(1, AGENTS + 1):
        if os.path.exists(f"{BASE_DIR}/trial_done_{i}.flag"):
            count += 1
    return count


def wait_until_done_or_timeout(duration: float, check_interval: float = 2.0):
    """Wait until all AGENTS have written their .flag files or timeout."""
    waited = 0.0
    while waited < duration:
        done = count_done_flags()
        if done >= AGENTS:
            return True, "all_done"
        time.sleep(check_interval)
        waited += check_interval
    return False, "timeout"


# ─────────────────────────────────────────────────────────
#  Experiment runner
# ─────────────────────────────────────────────────────────

def run_experiment(seed: int, difficulty: int, use_rviz: bool) -> bool:
    """Run one corridor experiment. Returns True if all agents completed."""
    kill_clean()
    remove_done_flags()

    results_csv = os.path.join(RESULTS_DIR, f"results_corridor_d{difficulty}.csv")

    env = os.environ.copy()
    env["MULTI_USV_SEED"] = str(seed)
    env["MULTI_USV_DEBRIS_COUNT"] = str(difficulty)
    env["MULTI_USV_AGENTS"] = str(AGENTS)
    env["MULTI_USV_RESULTS_CSV"] = results_csv

    rviz_flag = "true" if use_rviz else "false"
    setup_script = os.path.join(BASE_DIR, "install", "local_setup.bash")
    source_cmd = f"source {setup_script}"

    # Staggered launch for smoother startup
    bash_cmds = [
        f"{source_cmd} && ros2 launch multi_usv_planner planner_corridor.launch.py use_rviz:=false",
    ]
    if use_rviz:
        # Launch RViz separately after a delay
        bash_cmds.append(
            f"sleep 5 && {source_cmd} && ros2 run rviz2 rviz2 "
            f"-d {BASE_DIR}/src/multi_usv_planner/rviz/corridor.rviz"
        )

    processes = []
    for cmd in bash_cmds:
        proc = subprocess.Popen(
            ["bash", "-c", cmd], cwd=BASE_DIR, env=env,
            preexec_fn=os.setpgrp,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        processes.append(proc)

    # Wait for completion
    all_done, reason = wait_until_done_or_timeout(EXP_TIMEOUT_S)

    # Clean up
    kill_clean()

    if not all_done:
        print(f"  ⚠ seed={seed} d={difficulty}: {reason} (only {count_done_flags()}/{AGENTS} done)")
        return False

    print(f"  ✓ seed={seed} d={difficulty}: all {AGENTS} agents completed")
    return True


# ─────────────────────────────────────────────────────────
#  Analysis
# ─────────────────────────────────────────────────────────

def parse_csv(path: str):
    """Parse corridor results CSV into per-agent summary dicts."""
    summaries = []
    if not os.path.exists(path):
        return summaries

    with open(path, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            if len(row) == 9:
                try:
                    aid = int(row[0])
                    if 1 <= aid <= AGENTS:
                        summaries.append({
                            "agent": aid,
                            "duration": float(row[1]),
                            "timeout": int(row[2]),
                            "coll_dyn": int(row[3]),
                            "coll_st": int(row[4]),
                            "coll_nb": int(row[5]),
                            "clr_dyn": float(row[6]),
                            "clr_st": float(row[7]),
                            "clr_nb": float(row[8]),
                        })
                except (ValueError, IndexError):
                    pass
    return summaries


def analyze(difficulty: int):
    path = os.path.join(RESULTS_DIR, f"results_corridor_d{difficulty}.csv")
    summaries = parse_csv(path)

    if not summaries:
        print(f"No data found for difficulty={difficulty} at {path}")
        return

    n_agents = max(s["agent"] for s in summaries)
    n_experiments = len(summaries) // n_agents

    print(f"\n{'='*60}")
    print(f"Corridor Experiment Results — difficulty={difficulty} ({n_experiments} reps × {n_agents} agents)")
    print(f"{'='*60}")

    # Per-metric aggregation
    durations = [s["duration"] for s in summaries]
    coll_dyn = sum(s["coll_dyn"] for s in summaries)
    coll_st = sum(s["coll_st"] for s in summaries)
    coll_nb = sum(s["coll_nb"] for s in summaries)
    total_coll = coll_dyn + coll_st + coll_nb
    safety_rate = 1.0 - total_coll / max(len(summaries), 1)

    print(f"  Duration:  μ={sum(durations)/len(durations):.1f}s  σ={std(durations):.1f}s  min={min(durations):.1f}s  max={max(durations):.1f}s")
    print(f"  Collisions: dyn={coll_dyn} st={coll_st} nb={coll_nb} total={total_coll}")
    print(f"  Safety rate: {safety_rate*100:.1f}%")

    clr_dyn_vals = [s["clr_dyn"] for s in summaries if s["clr_dyn"] < 50]
    clr_nb_vals = [s["clr_nb"] for s in summaries if s["clr_nb"] < 50]
    if clr_dyn_vals:
        print(f"  Min dynamic clearance: μ={sum(clr_dyn_vals)/len(clr_dyn_vals):.2f}m  min={min(clr_dyn_vals):.2f}m")
    if clr_nb_vals:
        print(f"  Min neighbor clearance: μ={sum(clr_nb_vals)/len(clr_nb_vals):.2f}m  min={min(clr_nb_vals):.2f}m")

    # Per-agent breakdown
    for aid in range(1, n_agents + 1):
        agent_s = [s for s in summaries if s["agent"] == aid]
        if not agent_s:
            continue
        d_vals = [s["duration"] for s in agent_s]
        coll_a = sum(s["coll_dyn"] + s["coll_st"] + s["coll_nb"] for s in agent_s)
        print(f"  Agent {aid}:  μ_d={sum(d_vals)/len(d_vals):.1f}s  collisions={coll_a}/{len(agent_s)}")

    print()


def std(vals):
    if len(vals) <= 1:
        return 0.0
    m = sum(vals) / len(vals)
    return math.sqrt(sum((v - m)**2 for v in vals) / (len(vals) - 1))


# ─────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Corridor experiment runner")
    parser.add_argument("--difficulty", type=int, default=0,
                        help="4=easy, 6=medium, 8=hard (0 = run all 3)")
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4],
                        help="Seed values (default: 0 1 2 3 4)")
    parser.add_argument("--rviz", action="store_true", help="Show RViz during experiment")
    parser.add_argument("--analyze", action="store_true", help="Analyze existing results only, no run")

    args = parser.parse_args()

    if args.analyze:
        if args.difficulty:
            analyze(args.difficulty)
        else:
            for d in [4, 6, 8]:
                analyze(d)
        return

    difficulties = [args.difficulty] if args.difficulty else [4, 6, 8]
    seeds = args.seeds

    print(f"Corridor experiment plan: difficulties={difficulties} seeds={seeds}")
    print(f"  Agents={AGENTS}  Timeout per exp={EXP_TIMEOUT_S}s  RViz={'yes' if args.rviz else 'no'}")

    for d in difficulties:
        print(f"\n{'─'*50}")
        print(f"Difficulty {d} — {len(seeds)} experiments × {AGENTS} agents")
        print(f"{'─'*50}")

        success = 0
        for s in seeds:
            ok = run_experiment(s, d, args.rviz)
            if ok:
                success += 1

        print(f"  Result: {success}/{len(seeds)} experiments completed")
        analyze(d)


if __name__ == "__main__":
    main()
