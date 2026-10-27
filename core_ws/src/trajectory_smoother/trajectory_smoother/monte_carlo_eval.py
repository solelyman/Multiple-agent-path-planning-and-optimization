import argparse
import csv
import itertools
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass
class AgentState:
    x: float
    y: float
    vx: float
    nominal_vx: float
    lane: float
    lock_steps: int = 0
    stable_count: int = 0
    last_best_lane: float = 0.0


@dataclass
class EpisodeMetrics:
    success: bool
    collision: bool
    timeout: bool
    steps: int
    avg_decision_ms: float
    control_effort: float
    min_agent_dist: float
    min_obstacle_dist: float
    lock_skip_pct: float
    avg_lock_skip_ms: float
    avg_full_eval_ms: float
    lock_skip_count: int
    full_eval_count: int


class Scenario:
    def __init__(self, rng: np.random.Generator, num_agents: int = 4, difficulty: str = "medium"):
        self.num_agents = num_agents
        self.difficulty = difficulty
        self.start_x = -30.0
        self.finish_x = 125.0
        self.zone_x_min = 0.0
        self.zone_x_max = 100.0
        self.safe_dist = 5.5
        self.comm_radius = 75.0
        self.perception_radius = 80.0
        self.non_neighbor_colregs_gain = 300.0
        self.non_neighbor_collision_distance = 12.0
        self.dt = 0.2
        self.max_steps = 460
        self.max_lateral_speed = 2.0
        self.lane_candidates = np.array([-27.0, -18.0, -9.0, 0.0, 9.0, 18.0, 27.0], dtype=float)
        self.collision_penalty = 1e6
        self.colregs_penalty = 90.0
        self.lock_horizon_steps = 4
        self.intent_sigma = 10.0
        self.intent_weight = 35.0
        self.intent_horizon_steps = 3
        self.commit_stable_steps = 3
        self.commit_margin = 18.0
        self.commit_min_safe_obs = 16.0
        self.commit_min_safe_neighbor = 10.0
        self.auction_yield_weight = 55.0
        self.single_path_bonus = 28.0
        self.obstacles = self._make_obstacles(rng)
        self.static_obstacles = self._make_static_obstacles(rng)

    def _make_obstacles(self, rng: np.random.Generator) -> List[Dict[str, float]]:
        # 分阶段进入障碍区，避免“一次性全出”导致无意义死局
        obstacles = []
        if self.difficulty == "easy":
            phases = [0, 70, 140, 210]
            rmin, rmax = 2.5, 4.0
            v_cross = (0.4, 0.9)
            v_hori = (-0.8, -0.4)
            self.safe_dist = 5.0
            self.intent_weight = 28.0
            self.intent_horizon_steps = 2
            self.commit_margin = 14.0
            self.auction_yield_weight = 42.0
        elif self.difficulty == "hard":
            phases = [0, 45, 90, 135, 180, 225, 270, 315]
            rmin, rmax = 3.5, 5.5
            v_cross = (0.7, 1.4)
            v_hori = (-1.4, -0.8)
            self.safe_dist = 6.0
            self.intent_weight = 45.0
            self.intent_horizon_steps = 4
            self.commit_margin = 22.0
            self.auction_yield_weight = 70.0
        elif self.difficulty == "demo":
            phases = [0, 25, 50, 75, 105, 135, 165, 200]
            rmin, rmax = 2.2, 3.6
            v_cross = (0.30, 0.65)
            v_hori = (-0.70, -0.35)
            self.safe_dist = 4.8
            self.intent_weight = 38.0
            self.intent_horizon_steps = 4
            self.commit_margin = 18.0
            self.auction_yield_weight = 60.0
        else:
            phases = [0, 110, 220]
            rmin, rmax = 2.5, 4.2
            v_cross = (0.30, 0.65)
            v_hori = (-0.65, -0.35)
            self.safe_dist = 5.0
            self.intent_weight = 35.0
            self.intent_horizon_steps = 3
            self.commit_margin = 18.0
            self.auction_yield_weight = 58.0
        for i, phase in enumerate(phases):
            if i % 2 == 0:
                # 自下向上穿越
                obstacles.append(
                    {
                        "x0": float(rng.uniform(15.0, 110.0)),
                        "y0": float(rng.uniform(-75.0, -45.0)),
                        "vx": float(rng.uniform(-0.1, 0.1)),
                        "vy": float(rng.uniform(*v_cross)),
                        "start_step": phase,
                        "radius": float(rng.uniform(rmin, rmax)),
                    }
                )
            else:
                # 自右向左横穿
                obstacles.append(
                    {
                        "x0": float(rng.uniform(80.0, 125.0)),
                        "y0": float(rng.uniform(15.0, 55.0)),
                        "vx": float(rng.uniform(*v_hori)),
                        "vy": float(rng.uniform(-0.1, 0.1)),
                        "start_step": phase,
                        "radius": float(rng.uniform(rmin, rmax)),
                    }
                )
        return obstacles

    def _make_static_obstacles(self, rng: np.random.Generator) -> List[Dict[str, float]]:
        # 通过交错岛礁链塑造可见通道，迫使轨迹发生真实弯折
        if self.difficulty == "demo":
            base_layout = [
                (18.0, -7.0, 5.8),
                (32.0, 12.0, 4.2),
                (47.0, -14.0, 4.6),
                (64.0, -10.0, 6.0),
                (78.0, 12.0, 4.4),
                (92.0, 7.5, 5.8),
            ]
        else:
            return []

        static_obstacles: List[Dict[str, float]] = []
        for x, y, radius in base_layout:
            static_obstacles.append(
                {
                    "x": float(x + rng.uniform(-2.0, 2.0)),
                    "y": float(y + rng.uniform(-2.5, 2.5)),
                    "radius": float(radius + rng.uniform(-0.6, 0.6)),
                }
            )
        return static_obstacles

    def obstacle_positions(self, step: int) -> List[Tuple[float, float, float]]:
        positions = []
        for obs in self.obstacles:
            if step < obs["start_step"]:
                continue
            dt_step = (step - obs["start_step"]) * self.dt
            x = obs["x0"] + obs["vx"] * dt_step
            y = obs["y0"] + obs["vy"] * dt_step
            positions.append((x, y, obs["radius"]))
        for obs in self.static_obstacles:
            positions.append((obs["x"], obs["y"], obs["radius"]))
        return positions

    def dynamic_obstacle_positions(self, step: int) -> List[Tuple[float, float, float]]:
        positions = []
        for obs in self.obstacles:
            if step < obs["start_step"]:
                continue
            dt_step = (step - obs["start_step"]) * self.dt
            x = obs["x0"] + obs["vx"] * dt_step
            y = obs["y0"] + obs["vy"] * dt_step
            positions.append((x, y, obs["radius"]))
        return positions


class Evaluator:
    def __init__(self, scenario: Scenario, seed: int):
        self.sc = scenario
        self.rng = np.random.default_rng(seed)
        self._lock_skip_count = 0
        self._full_eval_count = 0
        self._lock_skip_total_us = 0.0
        self._full_eval_total_us = 0.0

    def init_agents(self) -> List[AgentState]:
        ys = [-13.0, -4.0, 4.0, 13.0]
        agents = []
        for i in range(self.sc.num_agents):
            agents.append(
                AgentState(
                    x=self.sc.start_x + (i % 2) * 4.0,
                    y=ys[i],
                    vx=float(self.rng.uniform(2.7, 3.3)),
                    nominal_vx=0.0,
                    lane=ys[i],
                    lock_steps=0,
                    stable_count=0,
                    last_best_lane=ys[i],
                )
            )
            agents[-1].nominal_vx = agents[-1].vx
        return agents

    def _distance(self, a: AgentState, b: AgentState) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    def _collision_cost(
        self,
        lane_choice: float,
        me: AgentState,
        others: List[AgentState],
        obs_positions: List[Tuple[float, float, float]],
        horizon_steps: int = 1,
    ) -> float:
        cost = 0.0
        for h in range(1, horizon_steps + 1):
            px = me.x + me.vx * self.sc.dt * h
            py = me.y + np.clip(
                lane_choice - me.y,
                -self.sc.max_lateral_speed * self.sc.dt * h,
                self.sc.max_lateral_speed * self.sc.dt * h,
            )

            for ox, oy, r in obs_positions:
                d = math.hypot(px - ox, py - oy) - (r + self.sc.safe_dist * 0.30)
                if d < 0.0:
                    if h == 1:
                        return self.sc.collision_penalty
                    cost += 350.0
                    continue
                cost += 15.0 / max(d, 1.0)

            for other in others:
                # 对邻居做短时直线预测
                ox = other.x + other.vx * self.sc.dt * h
                oy = other.y
                d = math.hypot(px - ox, py - oy)
                if d < self.sc.safe_dist:
                    if h == 1:
                        return self.sc.collision_penalty
                    cost += 300.0
                    continue
                cost += 18.0 / max(d, 1.0)
        return cost

    def _colregs_side_penalty(self, me: AgentState, lane_choice: float, other: AgentState) -> float:
        # 简化版：若对向/交叉临近时仍向“左侧抢行”增加惩罚
        relx = other.x - me.x
        if abs(relx) > 20.0:
            return 0.0
        desired_dy = lane_choice - me.y
        # 若对方在右前方且本船向左切，施加规则代价
        if relx > 0.0 and other.y > me.y and desired_dy < -1e-3:
            return self.sc.colregs_penalty
        return 0.0

    def _intent_risk_cost(self, lane_choice: float, me: AgentState, neighbors: List[AgentState]) -> float:
        # 以邻居广播状态为先验，构造短时“意图占据风险”
        risk = 0.0
        sigma2 = max(1.0, self.sc.intent_sigma * self.sc.intent_sigma)
        for h in range(1, self.sc.intent_horizon_steps + 1):
            px = me.x + me.vx * self.sc.dt * h
            py = me.y + np.clip(
                lane_choice - me.y,
                -self.sc.max_lateral_speed * self.sc.dt * h,
                self.sc.max_lateral_speed * self.sc.dt * h,
            )
            for other in neighbors:
                ox = other.x + other.vx * self.sc.dt * h
                oy = other.y
                d2 = (px - ox) * (px - ox) + (py - oy) * (py - oy)
                ahead_bias = 1.2 if (other.x > me.x and abs(other.x - me.x) < 30.0) else 1.0
                risk += ahead_bias * math.exp(-d2 / (2.0 * sigma2)) / float(h)
        return self.sc.intent_weight * risk

    def _active_neighbors(self, idx: int, agents: List[AgentState]) -> List[int]:
        out = []
        for j, other in enumerate(agents):
            if j == idx:
                continue
            if self._distance(agents[idx], other) <= self.sc.comm_radius:
                out.append(j)
        return out

    def _evaluate_lane_candidates(
        self,
        idx: int,
        agents: List[AgentState],
        obs_positions: List[Tuple[float, float, float]],
        yield_lane: float = None,
        risk_horizon_steps: int = 3,
    ) -> Tuple[List[Tuple[float, float]], int]:
        me = agents[idx]
        neighbors = self._active_neighbors(idx, agents)
        other_states = [agents[j] for j in neighbors]
        ranked: List[Tuple[float, float]] = []
        feasible_count = 0

        non_neighbor_others = []
        for j, o in enumerate(agents):
            if j == idx or j in neighbors:
                continue
            if self._distance(me, o) <= self.sc.perception_radius:
                non_neighbor_others.append(o)

        for lane in self.sc.lane_candidates:
            cost = abs(lane - me.y) * 2.0
            c_obs = self._collision_cost(lane, me, other_states, obs_positions, horizon_steps=risk_horizon_steps)
            if c_obs >= self.sc.collision_penalty:
                continue
            feasible_count += 1
            cost += c_obs
            cost += self._intent_risk_cost(lane, me, other_states)

            for j in neighbors:
                o = agents[j]
                free_margin = min(abs(o.y - self.sc.lane_candidates.min()), abs(self.sc.lane_candidates.max() - o.y))
                if abs(o.x - me.x) < 25.0 and free_margin < 4.5 and abs(lane - o.y) < 7.0:
                    cost += 40.0
                if abs(lane - o.lane) < 5.0:
                    cost += 55.0
                cost += self._colregs_side_penalty(me, lane, o)

            for o in non_neighbor_others:
                for h in range(1, risk_horizon_steps + 1):
                    px = me.x + me.vx * self.sc.dt * h
                    py = me.y + np.clip(
                        lane - me.y,
                        -self.sc.max_lateral_speed * self.sc.dt * h,
                        self.sc.max_lateral_speed * self.sc.dt * h,
                    )
                    ox = o.x + o.vx * self.sc.dt * h
                    oy = o.y
                    d_nn = math.hypot(px - ox, py - oy)
                    if d_nn < self.sc.non_neighbor_collision_distance:
                        if h == 1:
                            c_obs = self.sc.collision_penalty
                        else:
                            cost += 300.0
                    else:
                        cost += 15.0 / max(d_nn, 1.0)
                relx = o.x - me.x
                if abs(relx) <= 25.0:
                    desired_dy = lane - me.y
                    if relx > 0.0 and o.y > me.y and desired_dy < -1e-3:
                        cost += self.sc.non_neighbor_colregs_gain

            if yield_lane is not None and abs(lane - yield_lane) < 5.0:
                cost += self.sc.auction_yield_weight

            ranked.append((lane, cost))

        ranked.sort(key=lambda x: (x[1], abs(x[0] - me.y)))
        return ranked, feasible_count

    def _choose_lane_ours(
        self,
        idx: int,
        agents: List[AgentState],
        obs_positions: List[Tuple[float, float, float]],
        lock_enabled: bool,
        yield_lane: float = None,
        risk_horizon_steps: int = 3,
    ):
        me = agents[idx]
        if lock_enabled and me.lock_steps > 0:
            self._lock_skip_count += 1
            t0 = time.perf_counter()
            result = (me.lane, float("inf"), True)
            self._lock_skip_total_us += (time.perf_counter() - t0) * 1e6
            return result

        self._full_eval_count += 1
        t0 = time.perf_counter()
        ranked, feasible_count = self._evaluate_lane_candidates(
            idx, agents, obs_positions, yield_lane=yield_lane, risk_horizon_steps=risk_horizon_steps
        )
        if not ranked:
            self._full_eval_total_us += (time.perf_counter() - t0) * 1e6
            return me.lane, -1e6, False

        best_lane, best_cost = ranked[0]
        second_cost = ranked[1][1] if len(ranked) > 1 else best_cost + self.sc.commit_margin
        margin = second_cost - best_cost
        # 拍卖 bid：优势越明显、可行路越少，bid 越高
        bid = margin + (self.sc.single_path_bonus if feasible_count <= 1 else 0.0)

        # 条件锁定：连续稳定 + 优势足够 + 局部安全
        if abs(best_lane - me.last_best_lane) < 1e-6:
            me.stable_count += 1
        else:
            me.stable_count = 1
            me.last_best_lane = best_lane

        if lock_enabled:
            nearest_obs = float("inf")
            for ox, oy, _ in obs_positions:
                nearest_obs = min(nearest_obs, math.hypot(me.x - ox, me.y - oy))
            nearest_neighbor = float("inf")
            for j in self._active_neighbors(idx, agents):
                o = agents[j]
                nearest_neighbor = min(nearest_neighbor, math.hypot(me.x - o.x, me.y - o.y))
            if (
                me.stable_count >= self.sc.commit_stable_steps
                and margin >= self.sc.commit_margin
                and nearest_obs > self.sc.commit_min_safe_obs
                and nearest_neighbor > self.sc.commit_min_safe_neighbor
            ):
                me.lock_steps = self.sc.lock_horizon_steps

        self._full_eval_total_us += (time.perf_counter() - t0) * 1e6
        return best_lane, bid, False

    def _choose_lanes_decentralized_iterative(
        self,
        agents: List[AgentState],
        obs_positions: List[Tuple[float, float, float]],
        lock_enabled: bool,
        rounds: int,
        risk_horizon_steps: int = 3,
    ) -> List[float]:
        chosen = [a.lane for a in agents]
        for _ in range(max(1, rounds)):
            for j, a in enumerate(agents):
                a.lane = chosen[j]

            prelim_lane = [a.lane for a in agents]
            bids = [0.0 for _ in agents]
            committed = [False for _ in agents]

            # 阶段1：软拍卖，先算每船最优与 bid
            for i in range(len(agents)):
                lane, bid, is_committed = self._choose_lane_ours(
                    i,
                    agents,
                    obs_positions,
                    lock_enabled=lock_enabled,
                    yield_lane=None,
                    risk_horizon_steps=risk_horizon_steps,
                )
                prelim_lane[i] = lane
                bids[i] = bid
                committed[i] = is_committed

            # 阶段2：局部 token 分配，非 winner 让行重选
            for i in range(len(agents)):
                if committed[i]:
                    chosen[i] = prelim_lane[i]
                    continue
                group = [i] + self._active_neighbors(i, agents)
                winner = max(group, key=lambda j: (bids[j], -j))
                if winner == i:
                    chosen[i] = prelim_lane[i]
                else:
                    lane2, _, _ = self._choose_lane_ours(
                        i,
                        agents,
                        obs_positions,
                        lock_enabled=lock_enabled,
                        yield_lane=prelim_lane[winner],
                        risk_horizon_steps=risk_horizon_steps,
                    )
                    chosen[i] = lane2
        return chosen

    def _choose_lane_reactive(self, idx: int, agents: List[AgentState], obs_positions: List[Tuple[float, float, float]]):
        me = agents[idx]
        best_lane = me.lane
        best_cost = float("inf")
        # 纯反应式：不做通信图协商，仅盯最近 1 艘船
        others_sorted = sorted(
            [a for j, a in enumerate(agents) if j != idx],
            key=lambda a: math.hypot(a.x - me.x, a.y - me.y),
        )
        local_others = others_sorted[:1]
        for lane in self.sc.lane_candidates:
            c = self._collision_cost(lane, me, local_others, obs_positions, horizon_steps=1)
            c += abs(lane - me.y) * 1.5
            if c < best_cost:
                best_cost = c
                best_lane = lane
        return best_lane

    def _choose_lanes_centralized(self, agents: List[AgentState], obs_positions: List[Tuple[float, float, float]]) -> List[float]:
        best_combo = [a.lane for a in agents]
        best_cost = float("inf")
        # 有限搜索：每艘船只保留离当前横向位置最近的 3 个候选 lane
        lane_sets: List[List[float]] = []
        all_lanes = self.sc.lane_candidates.tolist()
        for ag in agents:
            ranked = sorted(all_lanes, key=lambda l: abs(l - ag.y))
            lane_sets.append(ranked[:3])

        for combo in itertools.product(*lane_sets):
            cost = 0.0
            bad = False
            for i, me in enumerate(agents):
                lane = combo[i]
                others = [agents[j] for j in range(len(agents)) if j != i]
                c = self._collision_cost(lane, me, others, obs_positions, horizon_steps=2)
                if c >= self.sc.collision_penalty:
                    bad = True
                    break
                cost += c + abs(lane - me.y) * 1.2
            if bad:
                continue
            # 组合内部碰撞检查（下步预测）
            pred = []
            for i, me in enumerate(agents):
                py = me.y + np.clip(combo[i] - me.y, -self.sc.max_lateral_speed * self.sc.dt, self.sc.max_lateral_speed * self.sc.dt)
                px = me.x + me.vx * self.sc.dt
                pred.append((px, py))
            for i in range(len(pred)):
                for j in range(i + 1, len(pred)):
                    if math.hypot(pred[i][0] - pred[j][0], pred[i][1] - pred[j][1]) < self.sc.safe_dist:
                        bad = True
                        break
                if bad:
                    break
            if bad:
                continue
            if cost < best_cost:
                best_cost = cost
                best_combo = list(combo)
        return best_combo

    def run_episode(
        self,
        policy: str,
        collect_trace: bool = False,
    ) -> Tuple[EpisodeMetrics, List[Dict[str, float]], List[Dict[str, float]]]:
        self._lock_skip_count = 0
        self._full_eval_count = 0
        self._lock_skip_total_us = 0.0
        self._full_eval_total_us = 0.0
        agents = self.init_agents()
        total_decision_t = 0.0
        decision_calls = 0
        control_effort = 0.0
        min_agent_dist = float("inf")
        min_obstacle_dist = float("inf")
        trace_rows: List[Dict[str, float]] = []
        obstacle_rows: List[Dict[str, float]] = []

        def _build_metrics(success, collision, timeout, steps):
            avg_ms = 1000.0 * total_decision_t / max(decision_calls, 1)
            total_evals = self._lock_skip_count + self._full_eval_count
            skip_pct = 100.0 * self._lock_skip_count / max(total_evals, 1)
            avg_skip_us = self._lock_skip_total_us / max(self._lock_skip_count, 1)
            avg_eval_us = self._full_eval_total_us / max(self._full_eval_count, 1)
            return EpisodeMetrics(
                success, collision, timeout, steps, avg_ms, control_effort,
                min_agent_dist, min_obstacle_dist,
                skip_pct, avg_skip_us / 1000.0, avg_eval_us / 1000.0,
                self._lock_skip_count, self._full_eval_count,
            )

        for step in range(self.sc.max_steps):
            obs_positions = self.sc.obstacle_positions(step)
            dyn_obs_positions = self.sc.dynamic_obstacle_positions(step)
            headings = [0.0 for _ in agents]

            t0 = time.perf_counter()
            if policy == "ours":
                chosen = self._choose_lanes_decentralized_iterative(
                    agents, obs_positions, lock_enabled=True, rounds=5, risk_horizon_steps=5
                )
            elif policy == "ablation_no_lock":
                chosen = self._choose_lanes_decentralized_iterative(
                    agents, obs_positions, lock_enabled=False, rounds=5, risk_horizon_steps=5
                )
            elif policy == "centralized":
                chosen = self._choose_lanes_centralized(agents, obs_positions)
            elif policy == "reactive":
                chosen = [self._choose_lane_reactive(i, agents, obs_positions) for i in range(len(agents))]
            else:
                raise ValueError(f"Unknown policy: {policy}")
            total_decision_t += (time.perf_counter() - t0)
            decision_calls += 1

            # 状态推进
            for i, ag in enumerate(agents):
                old_y = ag.y
                dy_cmd = np.clip(chosen[i] - ag.y, -self.sc.max_lateral_speed * self.sc.dt, self.sc.max_lateral_speed * self.sc.dt)
                ag.y += dy_cmd
                ag.lane = chosen[i]
                lateral_v = dy_cmd / self.sc.dt
                headings[i] = math.atan2(lateral_v, max(ag.vx, 1e-3))
                # 欠驱动现实：局部冲突时允许减速通过障碍区（参考 single 的保守避障思想）
                nearest_obs = float("inf")
                for ox, oy, r in obs_positions:
                    if ox < ag.x:
                        continue
                    if abs(ag.y - oy) < (r + self.sc.safe_dist):
                        nearest_obs = min(nearest_obs, ox - ag.x)
                nearest_neighbor = float("inf")
                for j, other in enumerate(agents):
                    if i == j:
                        continue
                    if other.x < ag.x:
                        continue
                    if abs(ag.y - other.y) < self.sc.safe_dist:
                        nearest_neighbor = min(nearest_neighbor, other.x - ag.x)

                target_v = ag.nominal_vx
                if policy in ("ours", "ablation_no_lock"):
                    if nearest_obs < 18.0 or nearest_neighbor < 14.0:
                        target_v = max(0.8, 0.35 * ag.nominal_vx)
                    elif nearest_obs < 30.0 or nearest_neighbor < 22.0:
                        target_v = max(1.0, 0.55 * ag.nominal_vx)
                else:
                    if nearest_obs < 12.0 or nearest_neighbor < 10.0:
                        target_v = max(1.1, 0.50 * ag.nominal_vx)
                    elif nearest_obs < 22.0 or nearest_neighbor < 16.0:
                        target_v = max(1.4, 0.72 * ag.nominal_vx)
                ag.vx += 0.35 * (target_v - ag.vx)
                ag.vx = float(np.clip(ag.vx, 1.0, 3.4))
                ag.x += ag.vx * self.sc.dt
                if ag.lock_steps > 0:
                    ag.lock_steps -= 1
                control_effort += abs(ag.y - old_y)

            # 记录时序数据，供轨迹图/距离曲线使用
            if collect_trace:
                for obs_id, (ox, oy, r) in enumerate(dyn_obs_positions):
                    obstacle_rows.append(
                        {
                            "step": step,
                            "time": step * self.sc.dt,
                            "obstacle_id": obs_id,
                            "x": ox,
                            "y": oy,
                            "radius": r,
                            "kind": "dynamic",
                        }
                    )
                for obs_id, obs in enumerate(self.sc.static_obstacles, start=len(dyn_obs_positions)):
                    obstacle_rows.append(
                        {
                            "step": step,
                            "time": step * self.sc.dt,
                            "obstacle_id": obs_id,
                            "x": obs["x"],
                            "y": obs["y"],
                            "radius": obs["radius"],
                            "kind": "static",
                        }
                    )
                for i, ag in enumerate(agents):
                    nearest_agent = float("inf")
                    nearest_obs = float("inf")
                    for j, other in enumerate(agents):
                        if i == j:
                            continue
                        nearest_agent = min(nearest_agent, self._distance(ag, other))
                    for ox, oy, r in obs_positions:
                        nearest_obs = min(nearest_obs, math.hypot(ag.x - ox, ag.y - oy) - r)
                    trace_rows.append(
                        {
                            "step": step,
                            "time": step * self.sc.dt,
                            "agent_id": i,
                            "x": ag.x,
                            "y": ag.y,
                            "vx": ag.vx,
                            "psi": headings[i],
                            "lane": ag.lane,
                            "nearest_agent_dist": nearest_agent,
                            "nearest_obstacle_dist": nearest_obs,
                        }
                    )

            # 碰撞检测：船-船
            for i in range(len(agents)):
                for j in range(i + 1, len(agents)):
                    dij = self._distance(agents[i], agents[j])
                    min_agent_dist = min(min_agent_dist, dij)
                    if dij < self.sc.safe_dist:
                        return _build_metrics(False, True, False, step + 1), trace_rows, obstacle_rows

            # 碰撞检测：船-障碍
            for ag in agents:
                for ox, oy, r in obs_positions:
                    dob = math.hypot(ag.x - ox, ag.y - oy) - r
                    min_obstacle_dist = min(min_obstacle_dist, dob)
                    if dob < (self.sc.safe_dist * 0.35):
                        return _build_metrics(False, True, False, step + 1), trace_rows, obstacle_rows

            # 终止条件：全部越过障碍区并到达出口
            if all(ag.x >= self.sc.finish_x for ag in agents):
                return _build_metrics(True, False, False, step + 1), trace_rows, obstacle_rows

        return _build_metrics(False, False, True, self.sc.max_steps), trace_rows, obstacle_rows


def summarize(metrics: List[EpisodeMetrics]) -> Dict[str, float]:
    n = max(1, len(metrics))
    success = sum(1 for m in metrics if m.success)
    collision = sum(1 for m in metrics if m.collision)
    timeout = sum(1 for m in metrics if m.timeout)
    return {
        "success_rate": 100.0 * success / n,
        "collision_rate": 100.0 * collision / n,
        "timeout_rate": 100.0 * timeout / n,
        "avg_steps": float(np.mean([m.steps for m in metrics])),
        "avg_decision_ms": float(np.mean([m.avg_decision_ms for m in metrics])),
        "avg_control_effort": float(np.mean([m.control_effort for m in metrics])),
        "avg_min_agent_dist": float(np.mean([m.min_agent_dist for m in metrics])),
        "avg_min_obstacle_dist": float(np.mean([m.min_obstacle_dist for m in metrics])),
        "avg_lock_skip_pct": float(np.mean([m.lock_skip_pct for m in metrics])),
        "avg_lock_skip_ms": float(np.mean([m.avg_lock_skip_ms for m in metrics])),
        "avg_full_eval_ms": float(np.mean([m.avg_full_eval_ms for m in metrics])),
        "avg_lock_skip_count": float(np.mean([m.lock_skip_count for m in metrics])),
        "avg_full_eval_count": float(np.mean([m.full_eval_count for m in metrics])),
    }


def _write_csv(path: Path, rows: List[Dict[str, float]]):
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def run_benchmark(num_runs: int, seed: int, difficulty: str, export_dir: Optional[Path] = None):
    policies = ["ours", "centralized", "reactive", "ablation_no_lock"]
    all_results: Dict[str, List[EpisodeMetrics]] = {k: [] for k in policies}
    run_rows: List[Dict[str, float]] = []

    for run_id in range(num_runs):
        run_seed = seed + run_id * 17
        for p in policies:
            sc = Scenario(np.random.default_rng(run_seed), num_agents=4, difficulty=difficulty)
            ev = Evaluator(sc, seed=run_seed + 7)
            metrics, _, _ = ev.run_episode(p, collect_trace=False)
            all_results[p].append(metrics)
            run_rows.append(
                {
                    "difficulty": difficulty,
                    "policy": p,
                    "run_id": run_id,
                    "seed": run_seed,
                    "success": float(metrics.success),
                    "collision": float(metrics.collision),
                    "timeout": float(metrics.timeout),
                    "steps": metrics.steps,
                    "avg_decision_ms": metrics.avg_decision_ms,
                    "control_effort": metrics.control_effort,
                    "min_agent_dist": metrics.min_agent_dist,
                    "min_obstacle_dist": metrics.min_obstacle_dist,
                    "lock_skip_pct": metrics.lock_skip_pct,
                    "avg_lock_skip_ms": metrics.avg_lock_skip_ms,
                    "avg_full_eval_ms": metrics.avg_full_eval_ms,
                    "lock_skip_count": metrics.lock_skip_count,
                    "full_eval_count": metrics.full_eval_count,
                }
            )
        if (run_id + 1) % max(1, num_runs // 10) == 0:
            print(f"[progress] completed {run_id + 1}/{num_runs} episodes")

    print(f"\nMonte Carlo runs: {num_runs}, difficulty={difficulty}")
    print(
        "Policy".ljust(20)
        + "Success(%)".rjust(12)
        + "Collision(%)".rjust(14)
        + "Timeout(%)".rjust(12)
        + "AvgStep".rjust(10)
        + "Dec(ms)".rjust(10)
        + "CtrlEff".rjust(12)
        + "Skip%".rjust(8)
        + "Skip(us)".rjust(10)
        + "Full(ms)".rjust(10)
    )
    print("-" * 110)
    summary_rows: List[Dict[str, float]] = []
    for p in policies:
        s = summarize(all_results[p])
        summary_rows.append({"difficulty": difficulty, "policy": p, **s})
        print(
            p.ljust(20)
            + f"{s['success_rate']:10.2f}".rjust(12)
            + f"{s['collision_rate']:12.2f}".rjust(14)
            + f"{s['timeout_rate']:10.2f}".rjust(12)
            + f"{s['avg_steps']:8.2f}".rjust(10)
            + f"{s['avg_decision_ms']:8.3f}".rjust(10)
            + f"{s['avg_control_effort']:10.2f}".rjust(12)
            + f"{s['avg_lock_skip_pct']:6.1f}".rjust(8)
            + f"{s['avg_lock_skip_ms']*1000:8.1f}".rjust(10)
            + f"{s['avg_full_eval_ms']:8.3f}".rjust(10)
        )

    if export_dir is not None:
        _write_csv(export_dir / f"{difficulty}_summary.csv", summary_rows)
        _write_csv(export_dir / f"{difficulty}_episodes.csv", run_rows)


def export_trace(seed: int, difficulty: str, policy: str, export_dir: Path):
    sc = Scenario(np.random.default_rng(seed), num_agents=4, difficulty=difficulty)
    ev = Evaluator(sc, seed=seed + 7)
    metrics, trace_rows, obstacle_rows = ev.run_episode(policy, collect_trace=True)
    export_dir.mkdir(parents=True, exist_ok=True)
    _write_csv(export_dir / f"trace_{difficulty}_{policy}_seed{seed}.csv", trace_rows)
    _write_csv(export_dir / f"obstacles_{difficulty}_{policy}_seed{seed}.csv", obstacle_rows)
    _write_csv(
        export_dir / f"trace_summary_{difficulty}_{policy}_seed{seed}.csv",
        [
            {
                "difficulty": difficulty,
                "policy": policy,
                "seed": seed,
                "success": float(metrics.success),
                "collision": float(metrics.collision),
                "timeout": float(metrics.timeout),
                "steps": metrics.steps,
                "avg_decision_ms": metrics.avg_decision_ms,
                "control_effort": metrics.control_effort,
                "min_agent_dist": metrics.min_agent_dist,
                "min_obstacle_dist": metrics.min_obstacle_dist,
                "lock_skip_pct": metrics.lock_skip_pct,
                "avg_lock_skip_ms": metrics.avg_lock_skip_ms,
                "avg_full_eval_ms": metrics.avg_full_eval_ms,
                "lock_skip_count": metrics.lock_skip_count,
                "full_eval_count": metrics.full_eval_count,
            }
        ],
    )


def main():
    parser = argparse.ArgumentParser(description="Finite-horizon Monte Carlo benchmark for multi-USV intent coordination.")
    parser.add_argument("--runs", type=int, default=200, help="Number of Monte Carlo episodes.")
    parser.add_argument("--seed", type=int, default=42, help="Base random seed.")
    parser.add_argument("--difficulty", type=str, default="medium", choices=["easy", "medium", "hard", "demo"])
    parser.add_argument("--export-dir", type=str, default="", help="Directory to export CSV results.")
    parser.add_argument("--export-trace-policy", type=str, default="", choices=["", "ours", "centralized", "reactive", "ablation_no_lock"])
    parser.add_argument("--trace-only", action="store_true", help="Export only representative trace without running benchmark.")
    args = parser.parse_args()
    export_dir = Path(args.export_dir) if args.export_dir else None
    if not args.trace_only:
        run_benchmark(num_runs=max(1, args.runs), seed=args.seed, difficulty=args.difficulty, export_dir=export_dir)
    if export_dir is not None and args.export_trace_policy:
        export_trace(seed=args.seed, difficulty=args.difficulty, policy=args.export_trace_policy, export_dir=export_dir)


if __name__ == "__main__":
    main()
