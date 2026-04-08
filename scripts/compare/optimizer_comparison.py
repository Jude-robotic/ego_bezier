import argparse
import csv
import inspect
import math
import os
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np


def flatten_f(mat: np.ndarray) -> np.ndarray:
    return np.asarray(mat, dtype=float).reshape(-1, order="F")


def unflatten_f(vec: np.ndarray, rows: int, cols: int) -> np.ndarray:
    return np.asarray(vec, dtype=float).reshape((rows, cols), order="F")


def set_plot_style(font_scale: float = 4.0) -> None:
    """Set a global plotting style. Default font size is doubled (x2)."""
    base_font = 10.0
    plt.rcParams.update(
        {
            "font.size": base_font * font_scale,
            "axes.titlesize": 12.0 * font_scale,
            "axes.labelsize": 10.0 * font_scale,
            "xtick.labelsize": 9.0 * font_scale,
            "ytick.labelsize": 9.0 * font_scale,
            "legend.fontsize": 9.0 * font_scale,
            "figure.titlesize": 13.0 * font_scale,
        }
    )


def parse_float_list(raw: str) -> List[float]:
    values = [float(x.strip()) for x in raw.split(",") if x.strip()]
    if not values:
        raise ValueError("Empty float list is not allowed.")
    return sorted(set(values))


@dataclass
class CostBreakdown:
    smoothness: float
    feasibility: float
    fitness: float
    total: float


@dataclass
class OptimizeResult:
    x_opt: np.ndarray
    converged: bool
    iterations: int
    eval_count: int
    runtime_ms: float
    cost_history: np.ndarray
    grad_norm_history: np.ndarray


class BBArmijoOptimizer:
    """Python version of the gradient descent style used in the C++ codebase."""

    def __init__(
        self,
        iter_limit: int = 250,
        invoke_limit: int = 200000,
        min_grad: float = 1e-3,
        c1: float = 1e-4,
    ) -> None:
        self.iter_limit = iter_limit
        self.invoke_limit = invoke_limit
        self.min_grad = min_grad
        self.c1 = c1

    def optimize(
        self,
        x0: np.ndarray,
        objfun: Callable[[np.ndarray], Tuple[float, np.ndarray]],
    ) -> OptimizeResult:
        x = np.array(x0, dtype=float)
        t_start = time.perf_counter()

        f, g = objfun(x)
        eval_count = 1
        cost_history = [f]
        grad_norm_history = [float(np.linalg.norm(g))]

        x_prev = None
        g_prev = None
        converged = grad_norm_history[-1] < self.min_grad

        for _ in range(self.iter_limit):
            if converged or eval_count >= self.invoke_limit:
                break

            if x_prev is None:
                max_grad = float(np.max(np.abs(g)))
                alpha = 1.0 if max_grad < 0.1 else 0.1 / max(max_grad, 1e-12)
            else:
                s = x - x_prev
                y = g - g_prev
                denom = float(np.dot(y, y))
                if denom <= 1e-16:
                    alpha = 1.0
                else:
                    alpha = float(np.dot(s, y) / denom)
                    if not np.isfinite(alpha) or alpha <= 1e-12:
                        alpha = 1.0

            step = alpha
            f_new = None
            g_new = None
            g_norm_sq = float(np.dot(g, g))

            while True:
                x_try = x - step * g
                f_try, g_try = objfun(x_try)
                eval_count += 1

                if f_try <= f - self.c1 * step * g_norm_sq or step < 1e-12:
                    f_new, g_new = f_try, g_try
                    x_new = x_try
                    break

                step *= 0.5
                if eval_count >= self.invoke_limit:
                    f_new, g_new = f_try, g_try
                    x_new = x_try
                    break

            x_prev, g_prev = x, g
            x, f, g = x_new, f_new, g_new

            cost_history.append(float(f))
            grad_norm = float(np.linalg.norm(g))
            grad_norm_history.append(grad_norm)
            converged = grad_norm < self.min_grad

        runtime_ms = (time.perf_counter() - t_start) * 1000.0

        return OptimizeResult(
            x_opt=x,
            converged=converged,
            iterations=max(len(cost_history) - 1, 0),
            eval_count=eval_count,
            runtime_ms=runtime_ms,
            cost_history=np.array(cost_history, dtype=float),
            grad_norm_history=np.array(grad_norm_history, dtype=float),
        )


class BezierRefineObjective:
    """Core refine objective adapted from bezier_optimizer.cpp (ROS-independent)."""

    def __init__(
        self,
        init_q: np.ndarray,
        ts: float,
        max_vel: float,
        max_acc: float,
        lambda_smooth: float,
        lambda_feas: float,
        lambda_fit: float,
    ) -> None:
        if (init_q.shape[1] - 1) % 3 != 0:
            raise ValueError("Bezier control point count must be 3*k+1.")
        self.init_q = np.array(init_q, dtype=float)
        self.ref_pts = np.array(init_q, dtype=float)
        self.ts = ts
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.lambda_smooth = lambda_smooth
        self.lambda_feas = lambda_feas
        self.lambda_fit = lambda_fit

    @property
    def x0(self) -> np.ndarray:
        return flatten_f(self.init_q)

    def unpack(self, x: np.ndarray) -> np.ndarray:
        return unflatten_f(x, 3, self.init_q.shape[1])

    def value_and_grad(self, x: np.ndarray) -> Tuple[float, np.ndarray]:
        q = self.unpack(x)
        breakdown, grad = self.cost_and_grad(q)
        return breakdown.total, flatten_f(grad)

    def cost_and_grad(self, q: np.ndarray) -> Tuple[CostBreakdown, np.ndarray]:
        grad = np.zeros_like(q)

        f_smooth, g_smooth = self._smoothness_cost(q)
        f_feas, g_feas = self._feasibility_cost(q)
        f_fit, g_fit = self._fitness_cost(q)

        total = (
            self.lambda_smooth * f_smooth
            + self.lambda_feas * f_feas
            + self.lambda_fit * f_fit
        )
        grad += (
            self.lambda_smooth * g_smooth
            + self.lambda_feas * g_feas
            + self.lambda_fit * g_fit
        )

        return CostBreakdown(f_smooth, f_feas, f_fit, total), grad

    def _smoothness_cost(self, q: np.ndarray) -> Tuple[float, np.ndarray]:
        cost = 0.0
        grad = np.zeros_like(q)
        n_seg = (q.shape[1] - 1) // 3

        jerk_weight = 10.0
        cont_weight_vel = 10000.0
        cont_weight_acc = 10000.0

        for k in range(n_seg):
            idx = 3 * k
            jerk = q[:, idx + 3] - 3 * q[:, idx + 2] + 3 * q[:, idx + 1] - q[:, idx]
            cost += jerk_weight * float(np.dot(jerk, jerk))
            grad_j = 2.0 * jerk_weight * jerk
            grad[:, idx] += -grad_j
            grad[:, idx + 1] += 3.0 * grad_j
            grad[:, idx + 2] += -3.0 * grad_j
            grad[:, idx + 3] += grad_j

        for k in range(n_seg - 1):
            idx = 3 * k

            diff_vel = q[:, idx + 4] - 2 * q[:, idx + 3] + q[:, idx + 2]
            cost += cont_weight_vel * float(np.dot(diff_vel, diff_vel))
            grad_v = 2.0 * cont_weight_vel * diff_vel
            grad[:, idx + 4] += grad_v
            grad[:, idx + 3] += -2.0 * grad_v
            grad[:, idx + 2] += grad_v

            diff_acc = q[:, idx + 1] - 2 * q[:, idx + 2] + 2 * q[:, idx + 4] - q[:, idx + 5]
            cost += cont_weight_acc * float(np.dot(diff_acc, diff_acc))
            grad_a = 2.0 * cont_weight_acc * diff_acc
            grad[:, idx + 1] += grad_a
            grad[:, idx + 2] += -2.0 * grad_a
            grad[:, idx + 4] += 2.0 * grad_a
            grad[:, idx + 5] += -grad_a

        return cost, grad

    def _feasibility_cost(self, q: np.ndarray) -> Tuple[float, np.ndarray]:
        cost = 0.0
        grad = np.zeros_like(q)
        n_seg = (q.shape[1] - 1) // 3
        ts = self.ts

        for k in range(n_seg):
            idx = 3 * k
            p0, p1 = q[:, idx], q[:, idx + 1]
            p2, p3 = q[:, idx + 2], q[:, idx + 3]

            velocities = [
                ((p1 - p0) * 3.0 / ts, idx, idx + 1),
                ((p2 - p1) * 3.0 / ts, idx + 1, idx + 2),
                ((p3 - p2) * 3.0 / ts, idx + 2, idx + 3),
            ]

            for v, i0, i1 in velocities:
                for d in range(3):
                    if abs(v[d]) > self.max_vel:
                        diff = abs(v[d]) - self.max_vel
                        cost += diff * diff
                        sgn = 1.0 if v[d] > 0 else -1.0
                        g = 2.0 * diff * sgn
                        grad[d, i1] += g * 3.0 / ts
                        grad[d, i0] += g * -3.0 / ts

            accelerations = [
                ((p2 - 2 * p1 + p0) * 6.0 / (ts * ts), [idx, idx + 1, idx + 2]),
                ((p3 - 2 * p2 + p1) * 6.0 / (ts * ts), [idx + 1, idx + 2, idx + 3]),
            ]

            for a, ids in accelerations:
                for d in range(3):
                    if abs(a[d]) > self.max_acc:
                        diff = abs(a[d]) - self.max_acc
                        cost += diff * diff
                        sgn = 1.0 if a[d] > 0 else -1.0
                        g = 2.0 * diff * sgn
                        factor = 6.0 / (ts * ts)
                        grad[d, ids[0]] += g * factor
                        grad[d, ids[1]] += g * (-2.0) * factor
                        grad[d, ids[2]] += g * factor

        return cost, grad

    def _fitness_cost(self, q: np.ndarray) -> Tuple[float, np.ndarray]:
        diff = q - self.ref_pts
        cost = float(np.sum(diff * diff))
        grad = 2.0 * diff
        return cost, grad

    def feasibility_violation(self, q: np.ndarray) -> Tuple[float, float]:
        n_seg = (q.shape[1] - 1) // 3
        max_v_excess = 0.0
        max_a_excess = 0.0

        for k in range(n_seg):
            idx = 3 * k
            p0, p1 = q[:, idx], q[:, idx + 1]
            p2, p3 = q[:, idx + 2], q[:, idx + 3]

            velocities = [
                (p1 - p0) * 3.0 / self.ts,
                (p2 - p1) * 3.0 / self.ts,
                (p3 - p2) * 3.0 / self.ts,
            ]
            accelerations = [
                (p2 - 2 * p1 + p0) * 6.0 / (self.ts * self.ts),
                (p3 - 2 * p2 + p1) * 6.0 / (self.ts * self.ts),
            ]

            for v in velocities:
                max_v_excess = max(max_v_excess, float(np.max(np.abs(v) - self.max_vel)))
            for a in accelerations:
                max_a_excess = max(max_a_excess, float(np.max(np.abs(a) - self.max_acc)))

        return max(0.0, max_v_excess), max(0.0, max_a_excess)


class BSplineRefineObjective:
    """Core refine objective adapted from bspline_optimizer.cpp (ROS-independent)."""

    def __init__(
        self,
        init_q: np.ndarray,
        ts: float,
        max_vel: float,
        max_acc: float,
        lambda_smooth: float,
        lambda_feas: float,
        lambda_fit: float,
        order: int = 3,
    ) -> None:
        if init_q.shape[1] <= 2 * order:
            raise ValueError("Control point count too small for B-spline refine mode.")

        self.init_q = np.array(init_q, dtype=float)
        self.ts = ts
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.lambda_smooth = lambda_smooth
        self.lambda_feas = lambda_feas
        self.lambda_fit = lambda_fit
        self.order = order

        self.start_id = order
        self.end_id = self.init_q.shape[1] - order

        # ref_pts_.size() ~= q.cols() - 2 for cubic B-spline in original implementation.
        self.ref_pts = (self.init_q[:, :-2] + 4 * self.init_q[:, 1:-1] + self.init_q[:, 2:]) / 6.0

    @property
    def x0(self) -> np.ndarray:
        return flatten_f(self.init_q[:, self.start_id : self.end_id])

    def unpack(self, x: np.ndarray) -> np.ndarray:
        q = self.init_q.copy()
        q[:, self.start_id : self.end_id] = unflatten_f(
            x,
            3,
            self.end_id - self.start_id,
        )
        return q

    def _pack_grad(self, grad_full: np.ndarray) -> np.ndarray:
        return flatten_f(grad_full[:, self.start_id : self.end_id])

    def value_and_grad(self, x: np.ndarray) -> Tuple[float, np.ndarray]:
        q = self.unpack(x)
        breakdown, grad_full = self.cost_and_grad(q)
        return breakdown.total, self._pack_grad(grad_full)

    def cost_and_grad(self, q: np.ndarray) -> Tuple[CostBreakdown, np.ndarray]:
        grad = np.zeros_like(q)

        f_smooth, g_smooth = self._smoothness_cost(q)
        f_feas, g_feas = self._feasibility_cost(q)
        f_fit, g_fit = self._fitness_cost(q)

        total = (
            self.lambda_smooth * f_smooth
            + self.lambda_feas * f_feas
            + self.lambda_fit * f_fit
        )
        grad += (
            self.lambda_smooth * g_smooth
            + self.lambda_feas * g_feas
            + self.lambda_fit * g_fit
        )

        return CostBreakdown(f_smooth, f_feas, f_fit, total), grad

    def _smoothness_cost(self, q: np.ndarray) -> Tuple[float, np.ndarray]:
        cost = 0.0
        grad = np.zeros_like(q)

        for i in range(q.shape[1] - 3):
            jerk = q[:, i + 3] - 3 * q[:, i + 2] + 3 * q[:, i + 1] - q[:, i]
            cost += float(np.dot(jerk, jerk))
            temp_j = 2.0 * jerk
            grad[:, i] += -temp_j
            grad[:, i + 1] += 3.0 * temp_j
            grad[:, i + 2] += -3.0 * temp_j
            grad[:, i + 3] += temp_j

        return cost, grad

    def _feasibility_cost(self, q: np.ndarray) -> Tuple[float, np.ndarray]:
        # Uses the active #else branch in bspline_optimizer.cpp.
        cost = 0.0
        grad = np.zeros_like(q)

        ts_inv2 = 1.0 / (self.ts * self.ts)

        for i in range(q.shape[1] - 1):
            vi = (q[:, i + 1] - q[:, i]) / self.ts
            for d in range(3):
                if vi[d] > self.max_vel:
                    diff = vi[d] - self.max_vel
                    cost += diff * diff * ts_inv2
                    g = 2.0 * diff / self.ts * ts_inv2
                    grad[d, i] += -g
                    grad[d, i + 1] += g
                elif vi[d] < -self.max_vel:
                    diff = vi[d] + self.max_vel
                    cost += diff * diff * ts_inv2
                    g = 2.0 * diff / self.ts * ts_inv2
                    grad[d, i] += -g
                    grad[d, i + 1] += g

        for i in range(q.shape[1] - 2):
            ai = (q[:, i + 2] - 2 * q[:, i + 1] + q[:, i]) * ts_inv2
            for d in range(3):
                if ai[d] > self.max_acc:
                    diff = ai[d] - self.max_acc
                    cost += diff * diff
                    grad[d, i] += 2.0 * diff * ts_inv2
                    grad[d, i + 1] += -4.0 * diff * ts_inv2
                    grad[d, i + 2] += 2.0 * diff * ts_inv2
                elif ai[d] < -self.max_acc:
                    diff = ai[d] + self.max_acc
                    cost += diff * diff
                    grad[d, i] += 2.0 * diff * ts_inv2
                    grad[d, i + 1] += -4.0 * diff * ts_inv2
                    grad[d, i + 2] += 2.0 * diff * ts_inv2

        return cost, grad

    def _fitness_cost(self, q: np.ndarray) -> Tuple[float, np.ndarray]:
        # This follows the refine-mode fitness in bspline_optimizer.cpp.
        cost = 0.0
        grad = np.zeros_like(q)

        end_idx = q.shape[1] - self.order
        a2, b2 = 25.0, 1.0

        for i in range(self.order - 1, end_idx + 1):
            x = (q[:, i - 1] + 4.0 * q[:, i] + q[:, i + 1]) / 6.0 - self.ref_pts[:, i - 1]
            v_raw = self.ref_pts[:, i] - self.ref_pts[:, i - 2]
            v_norm = float(np.linalg.norm(v_raw))
            if v_norm < 1e-9:
                continue
            v = v_raw / v_norm

            xdotv = float(np.dot(x, v))
            xcrossv = np.cross(x, v)

            f = (xdotv * xdotv) / a2 + float(np.dot(xcrossv, xcrossv)) / b2
            cost += f

            m = np.array(
                [
                    [0.0, -v[2], v[1]],
                    [v[2], 0.0, -v[0]],
                    [-v[1], v[0], 0.0],
                ]
            )
            df_dx = 2.0 * xdotv / a2 * v + 2.0 / b2 * (m @ xcrossv)

            grad[:, i - 1] += df_dx / 6.0
            grad[:, i] += 4.0 * df_dx / 6.0
            grad[:, i + 1] += df_dx / 6.0

        return cost, grad

    def feasibility_violation(self, q: np.ndarray) -> Tuple[float, float]:
        ts_inv2 = 1.0 / (self.ts * self.ts)
        max_v_excess = 0.0
        max_a_excess = 0.0

        for i in range(q.shape[1] - 1):
            vi = (q[:, i + 1] - q[:, i]) / self.ts
            max_v_excess = max(max_v_excess, float(np.max(np.abs(vi) - self.max_vel)))

        for i in range(q.shape[1] - 2):
            ai = (q[:, i + 2] - 2 * q[:, i + 1] + q[:, i]) * ts_inv2
            max_a_excess = max(max_a_excess, float(np.max(np.abs(ai) - self.max_acc)))

        return max(0.0, max_v_excess), max(0.0, max_a_excess)


@dataclass
class ExperimentConfig:
    trials: int
    seed: int
    num_ctrl: int
    ts: float
    max_vel: float
    max_acc: float
    lambda_smooth: float
    lambda_feas: float
    lambda_fit: float
    iter_limit: int
    min_grad: float
    output_dir: str


def generate_initial_control_points(num_ctrl: int, rng: np.random.Generator) -> np.ndarray:
    t = np.linspace(0.0, 1.0, num_ctrl)

    base = np.vstack(
        [
            12.0 * t,
            2.5 * np.sin(2.5 * np.pi * t),
            1.2 + 0.5 * np.cos(2.0 * np.pi * t),
        ]
    )
    noise = np.vstack(
        [
            rng.normal(0.0, 0.35, num_ctrl),
            rng.normal(0.0, 0.9, num_ctrl),
            rng.normal(0.0, 0.25, num_ctrl),
        ]
    )
    q = base + noise

    q[:, 0] = np.array([0.0, 0.0, 1.0])
    q[:, -1] = np.array([12.0, 0.0, 1.0])
    q[:, 1] = np.array([0.8, 0.2, 1.1])
    q[:, -2] = np.array([11.2, -0.2, 1.0])

    return q


def run_single_trial(
    trial_id: int,
    cfg: ExperimentConfig,
    rng: np.random.Generator,
) -> Tuple[List[Dict[str, float]], Dict[str, np.ndarray], Dict[str, np.ndarray]]:
    q0 = generate_initial_control_points(cfg.num_ctrl, rng)

    solver = BBArmijoOptimizer(
        iter_limit=cfg.iter_limit,
        min_grad=cfg.min_grad,
    )

    bez_obj = BezierRefineObjective(
        q0,
        ts=cfg.ts,
        max_vel=cfg.max_vel,
        max_acc=cfg.max_acc,
        lambda_smooth=cfg.lambda_smooth,
        lambda_feas=cfg.lambda_feas,
        lambda_fit=cfg.lambda_fit,
    )
    bez_init_breakdown, _ = bez_obj.cost_and_grad(q0)
    bez_res = solver.optimize(bez_obj.x0, bez_obj.value_and_grad)
    q_bez = bez_obj.unpack(bez_res.x_opt)
    bez_breakdown, _ = bez_obj.cost_and_grad(q_bez)
    bez_vio_v, bez_vio_a = bez_obj.feasibility_violation(q_bez)

    bsp_obj = BSplineRefineObjective(
        q0,
        ts=cfg.ts,
        max_vel=cfg.max_vel,
        max_acc=cfg.max_acc,
        lambda_smooth=cfg.lambda_smooth,
        lambda_feas=cfg.lambda_feas,
        lambda_fit=cfg.lambda_fit,
        order=3,
    )
    bsp_init_breakdown, _ = bsp_obj.cost_and_grad(q0)
    bsp_res = solver.optimize(bsp_obj.x0, bsp_obj.value_and_grad)
    q_bsp = bsp_obj.unpack(bsp_res.x_opt)
    bsp_breakdown, _ = bsp_obj.cost_and_grad(q_bsp)
    bsp_vio_v, bsp_vio_a = bsp_obj.feasibility_violation(q_bsp)

    records = [
        {
            "trial": trial_id,
            "optimizer": "Bezier",
            "runtime_ms": bez_res.runtime_ms,
            "iterations": bez_res.iterations,
            "eval_count": bez_res.eval_count,
            "initial_total_cost": bez_init_breakdown.total,
            "final_total_cost": bez_breakdown.total,
            "cost_reduction_ratio": (bez_init_breakdown.total - bez_breakdown.total)
            / max(bez_init_breakdown.total, 1e-12),
            "smoothness_cost": bez_breakdown.smoothness,
            "feasibility_cost": bez_breakdown.feasibility,
            "fitness_cost": bez_breakdown.fitness,
            "max_vel_violation": bez_vio_v,
            "max_acc_violation": bez_vio_a,
            "converged": 1.0 if bez_res.converged else 0.0,
        },
        {
            "trial": trial_id,
            "optimizer": "BSpline",
            "runtime_ms": bsp_res.runtime_ms,
            "iterations": bsp_res.iterations,
            "eval_count": bsp_res.eval_count,
            "initial_total_cost": bsp_init_breakdown.total,
            "final_total_cost": bsp_breakdown.total,
            "cost_reduction_ratio": (bsp_init_breakdown.total - bsp_breakdown.total)
            / max(bsp_init_breakdown.total, 1e-12),
            "smoothness_cost": bsp_breakdown.smoothness,
            "feasibility_cost": bsp_breakdown.feasibility,
            "fitness_cost": bsp_breakdown.fitness,
            "max_vel_violation": bsp_vio_v,
            "max_acc_violation": bsp_vio_a,
            "converged": 1.0 if bsp_res.converged else 0.0,
        },
    ]

    histories = {
        "Bezier": bez_res.cost_history,
        "BSpline": bsp_res.cost_history,
    }
    trajectories = {
        "init": q0,
        "Bezier": q_bez,
        "BSpline": q_bsp,
    }
    return records, histories, trajectories


def pad_histories(histories: List[np.ndarray]) -> np.ndarray:
    max_len = max(len(h) for h in histories)
    padded = []
    for h in histories:
        if len(h) < max_len:
            h_pad = np.pad(h, (0, max_len - len(h)), mode="edge")
        else:
            h_pad = h
        padded.append(h_pad)
    return np.vstack(padded)


def summarize(records: List[Dict[str, float]]) -> Dict[str, Dict[str, float]]:
    out: Dict[str, Dict[str, float]] = {}
    optimizers = sorted(set(r["optimizer"] for r in records))
    for name in optimizers:
        rows = [r for r in records if r["optimizer"] == name]
        out[name] = {
            "mean_runtime_ms": float(np.mean([r["runtime_ms"] for r in rows])),
            "mean_iterations": float(np.mean([r["iterations"] for r in rows])),
            "mean_eval_count": float(np.mean([r["eval_count"] for r in rows])),
            "mean_initial_total_cost": float(np.mean([r["initial_total_cost"] for r in rows])),
            "mean_final_total_cost": float(np.mean([r["final_total_cost"] for r in rows])),
            "mean_cost_reduction_ratio": float(np.mean([r["cost_reduction_ratio"] for r in rows])),
            "mean_feasibility_cost": float(np.mean([r["feasibility_cost"] for r in rows])),
            "mean_max_vel_violation": float(np.mean([r["max_vel_violation"] for r in rows])),
            "mean_max_acc_violation": float(np.mean([r["max_acc_violation"] for r in rows])),
            "converged_ratio": float(np.mean([r["converged"] for r in rows])),
        }
    return out


def save_records_csv(records: List[Dict[str, float]], output_dir: str) -> None:
    csv_path = os.path.join(output_dir, "per_trial_metrics.csv")
    fieldnames = [
        "trial",
        "optimizer",
        "runtime_ms",
        "iterations",
        "eval_count",
        "initial_total_cost",
        "final_total_cost",
        "cost_reduction_ratio",
        "smoothness_cost",
        "feasibility_cost",
        "fitness_cost",
        "max_vel_violation",
        "max_acc_violation",
        "converged",
    ]
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in records:
            writer.writerow(row)


def save_summary_txt(summary: Dict[str, Dict[str, float]], cfg: ExperimentConfig) -> None:
    txt_path = os.path.join(cfg.output_dir, "summary.txt")
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("Optimizer Comparison Summary\n")
        f.write("=============================\n")
        f.write(f"trials={cfg.trials}, seed={cfg.seed}, num_ctrl={cfg.num_ctrl}\n")
        f.write(
            f"ts={cfg.ts}, max_vel={cfg.max_vel}, max_acc={cfg.max_acc}, "
            f"lambda_smooth={cfg.lambda_smooth}, lambda_feas={cfg.lambda_feas}, lambda_fit={cfg.lambda_fit}\n\n"
        )
        for name, stats in summary.items():
            f.write(f"[{name}]\n")
            for key, value in stats.items():
                f.write(f"  {key}: {value:.6f}\n")
            f.write("\n")


def plot_convergence(
    history_bank: Dict[str, List[np.ndarray]],
    output_dir: str,
) -> None:
    plt.figure(figsize=(14, 8))
    for name, histories in history_bank.items():
        normalized = [h / max(h[0], 1e-12) for h in histories]
        arr = pad_histories(normalized)
        mean_curve = arr.mean(axis=0)
        std_curve = arr.std(axis=0)

        x = np.arange(len(mean_curve))
        plt.plot(x, mean_curve, label=f"{name} mean normalized cost")
        plt.fill_between(x, mean_curve - std_curve, mean_curve + std_curve, alpha=0.18)

    plt.yscale("log")
    plt.xlabel("Accepted iteration")
    plt.ylabel("Normalized total cost (log scale)")
    plt.title("Convergence Curves")
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "convergence_curves.png"), dpi=160)
    plt.close()


def plot_box_metrics(records: List[Dict[str, float]], output_dir: str) -> None:
    bez = [r for r in records if r["optimizer"] == "Bezier"]
    bsp = [r for r in records if r["optimizer"] == "BSpline"]

    def one(metric: str, title: str, fname: str, ylabel: str) -> None:
        plt.figure(figsize=(10, 8))
        data = [
            [r[metric] for r in bez],
            [r[metric] for r in bsp],
        ]

        # Matplotlib >=3.9 uses tick_labels, while older versions use labels.
        boxplot_params = inspect.signature(plt.boxplot).parameters
        if "tick_labels" in boxplot_params:
            plt.boxplot(data, tick_labels=["Bezier", "BSpline"], showmeans=True)
        else:
            plt.boxplot(data, labels=["Bezier", "BSpline"], showmeans=True)
        plt.ylabel(ylabel)
        plt.title(title)
        plt.grid(alpha=0.25)
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, fname), dpi=160)
        plt.close()

    one("runtime_ms", "Runtime Distribution", "runtime_boxplot.png", "Runtime (ms)")
    one("final_total_cost", "Final Total Cost", "final_total_cost_boxplot.png", "Cost")

    for r in records:
        r["feas_violation"] = max(r["max_vel_violation"], r["max_acc_violation"])
    one(
        "feas_violation",
        "Final Feasibility Violation",
        "feasibility_violation_boxplot.png",
        "Violation magnitude",
    )


def plot_example_trajectory(trajectories: Dict[str, np.ndarray], output_dir: str) -> None:
    q0 = trajectories["init"]
    qb = trajectories["Bezier"]
    qs = trajectories["BSpline"]

    plt.figure(figsize=(16, 8))

    plt.subplot(1, 2, 1)
    plt.plot(q0[0], q0[1], "k--", alpha=0.7, label="Initial")
    plt.plot(qb[0], qb[1], "-o", markersize=3, label="Bezier optimized")
    plt.plot(qs[0], qs[1], "-s", markersize=3, label="BSpline optimized")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Control Points in XY")
    plt.axis("equal")
    plt.grid(alpha=0.3)
    plt.legend()

    plt.subplot(1, 2, 2)
    idx = np.arange(q0.shape[1])
    plt.plot(idx, q0[2], "k--", alpha=0.7, label="Initial z")
    plt.plot(idx, qb[2], "-o", markersize=3, label="Bezier z")
    plt.plot(idx, qs[2], "-s", markersize=3, label="BSpline z")
    plt.xlabel("Control point index")
    plt.ylabel("z")
    plt.title("Altitude Profile")
    plt.grid(alpha=0.3)
    plt.legend()

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "example_trajectory.png"), dpi=160)
    plt.close()


def _normalize_metric(values: List[float], positive: bool) -> List[float]:
    arr = np.array(values, dtype=float)
    lo = float(arr.min())
    hi = float(arr.max())
    if hi - lo < 1e-12:
        norm = np.ones_like(arr)
    else:
        norm = (arr - lo) / (hi - lo)
    if positive:
        return norm.tolist()
    return (1.0 - norm).tolist()


def _score_scan_entries(entries: List[Dict[str, float]]) -> None:
    # Robust score: higher cost-reduction, lower violation/runtime/variance.
    for optimizer in sorted(set(e["optimizer"] for e in entries)):
        subset_idx = [i for i, e in enumerate(entries) if e["optimizer"] == optimizer]
        reductions = [entries[i]["mean_cost_reduction_ratio"] for i in subset_idx]
        violations = [entries[i]["mean_feas_violation"] for i in subset_idx]
        runtimes = [entries[i]["mean_runtime_ms"] for i in subset_idx]
        variances = [entries[i]["std_final_total_cost"] for i in subset_idx]

        red_n = _normalize_metric(reductions, positive=True)
        vio_n = _normalize_metric(violations, positive=False)
        run_n = _normalize_metric(runtimes, positive=False)
        var_n = _normalize_metric(variances, positive=False)

        for j, idx in enumerate(subset_idx):
            score = 0.45 * red_n[j] + 0.30 * vio_n[j] + 0.15 * run_n[j] + 0.10 * var_n[j]
            entries[idx]["robust_score"] = float(score)


def _save_parameter_scan_csv(entries: List[Dict[str, float]], output_dir: str) -> None:
    csv_path = os.path.join(output_dir, "parameter_scan_metrics.csv")
    fieldnames = [
        "optimizer",
        "lambda_smooth",
        "lambda_feas",
        "lambda_fit",
        "mean_runtime_ms",
        "mean_final_total_cost",
        "std_final_total_cost",
        "mean_cost_reduction_ratio",
        "mean_feas_violation",
        "robust_score",
    ]
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in entries:
            writer.writerow({k: row[k] for k in fieldnames})


def _build_scan_lookup(entries: List[Dict[str, float]]) -> Dict[Tuple[str, float, float, float], Dict[str, float]]:
    lookup: Dict[Tuple[str, float, float, float], Dict[str, float]] = {}
    for e in entries:
        key = (e["optimizer"], e["lambda_smooth"], e["lambda_feas"], e["lambda_fit"])
        lookup[key] = e
    return lookup


def plot_parameter_scan_heatmaps(
    entries: List[Dict[str, float]],
    lambda_smooth_values: List[float],
    lambda_feas_values: List[float],
    lambda_fit_values: List[float],
    output_dir: str,
) -> None:
    lookup = _build_scan_lookup(entries)
    optimizers = sorted(set(e["optimizer"] for e in entries))

    n_fit = len(lambda_fit_values)
    nrows, ncols = 2, 2
    per_page = nrows * ncols
    n_pages = int(math.ceil(n_fit / per_page))

    for optimizer in optimizers:
        mats: List[np.ndarray] = []
        for l_fit in lambda_fit_values:
            mat = np.full((len(lambda_smooth_values), len(lambda_feas_values)), np.nan, dtype=float)
            for i, l_s in enumerate(lambda_smooth_values):
                for j, l_f in enumerate(lambda_feas_values):
                    key = (optimizer, l_s, l_f, l_fit)
                    if key in lookup:
                        mat[i, j] = lookup[key]["robust_score"]
            mats.append(mat)

        finite_vals = [m[np.isfinite(m)] for m in mats if np.isfinite(m).any()]
        if finite_vals:
            all_vals = np.concatenate(finite_vals)
            vmin, vmax = float(np.min(all_vals)), float(np.max(all_vals))
        else:
            vmin, vmax = 0.0, 1.0

        for page in range(n_pages):
            start = page * per_page
            end = min(start + per_page, n_fit)

            fig, axes = plt.subplots(
                nrows,
                ncols,
                figsize=(9 * ncols, 9 * nrows),
                squeeze=False,
                constrained_layout=True,
            )
            ref_im = None

            for slot in range(per_page):
                idx = start + slot
                r, c = divmod(slot, ncols)
                ax = axes[r][c]

                if idx >= end:
                    ax.axis("off")
                    continue

                l_fit = lambda_fit_values[idx]
                mat = mats[idx]
                im = ax.imshow(mat, origin="lower", aspect="auto", cmap="viridis", vmin=vmin, vmax=vmax)
                ref_im = im

                ax.set_title(f"lambda_fit={l_fit:g}")
                ax.set_xlabel("lambda_feas")
                ax.set_ylabel("lambda_smooth")
                ax.set_xticks(np.arange(len(lambda_feas_values)))
                ax.set_yticks(np.arange(len(lambda_smooth_values)))
                ax.set_xticklabels([f"{v:g}" for v in lambda_feas_values], rotation=45, ha="right")
                ax.set_yticklabels([f"{v:g}" for v in lambda_smooth_values])

                if np.isfinite(mat).any():
                    best_flat = int(np.nanargmax(mat))
                    bi, bj = np.unravel_index(best_flat, mat.shape)
                    ax.scatter([bj], [bi], marker="*", s=500, c="white", edgecolors="black", linewidths=1.2)

            if ref_im is not None:
                cbar = fig.colorbar(ref_im, ax=axes.ravel().tolist(), shrink=0.88, pad=0.02)
                cbar.set_label("Robust score (higher is better)")

            if n_pages > 1:
                fig.suptitle(
                    f"{optimizer} Robustness Heatmaps (lambda_smooth/lambda_feas/lambda_fit) - page {page + 1}/{n_pages}"
                )
            else:
                fig.suptitle(f"{optimizer} Robustness Heatmaps (lambda_smooth/lambda_feas/lambda_fit)")

            if n_pages > 1:
                file_name = f"parameter_scan_heatmap_{optimizer.lower()}_p{page + 1}.png"
            else:
                file_name = f"parameter_scan_heatmap_{optimizer.lower()}.png"
            fig.savefig(os.path.join(output_dir, file_name), dpi=160)
            plt.close(fig)


def summarize_parameter_scan(
    entries: List[Dict[str, float]],
    top_fraction: float,
    output_dir: str,
) -> None:
    txt_path = os.path.join(output_dir, "parameter_scan_summary.txt")
    top_fraction = min(max(top_fraction, 0.01), 0.5)

    lines: List[str] = []
    lines.append("Parameter Scan Robustness Summary")
    lines.append("================================")
    lines.append(f"top_fraction={top_fraction:.3f}")
    lines.append("")

    grouped: Dict[str, List[Dict[str, float]]] = {}
    for e in entries:
        grouped.setdefault(e["optimizer"], []).append(e)

    for optimizer in sorted(grouped.keys()):
        rows = sorted(grouped[optimizer], key=lambda x: x["robust_score"], reverse=True)
        k = max(1, int(len(rows) * top_fraction))
        top_rows = rows[:k]
        best = rows[0]

        s_vals = [r["lambda_smooth"] for r in top_rows]
        f_vals = [r["lambda_feas"] for r in top_rows]
        t_vals = [r["lambda_fit"] for r in top_rows]

        lines.append(f"[{optimizer}]")
        lines.append(
            "  best_config: "
            f"lambda_smooth={best['lambda_smooth']:.6g}, "
            f"lambda_feas={best['lambda_feas']:.6g}, "
            f"lambda_fit={best['lambda_fit']:.6g}, "
            f"robust_score={best['robust_score']:.6f}"
        )
        lines.append(
            "  robust_interval(top configs): "
            f"lambda_smooth in [{min(s_vals):.6g}, {max(s_vals):.6g}], "
            f"lambda_feas in [{min(f_vals):.6g}, {max(f_vals):.6g}], "
            f"lambda_fit in [{min(t_vals):.6g}, {max(t_vals):.6g}]"
        )
        lines.append("")

    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def run_parameter_scan(
    base_cfg: ExperimentConfig,
    scan_trials: int,
    lambda_smooth_values: List[float],
    lambda_feas_values: List[float],
    lambda_fit_values: List[float],
    scan_iter_limit: int,
    top_fraction: float,
) -> None:
    scan_output_dir = os.path.join(base_cfg.output_dir, "parameter_scan")
    os.makedirs(scan_output_dir, exist_ok=True)

    total_combo = len(lambda_smooth_values) * len(lambda_feas_values) * len(lambda_fit_values)
    combo_id = 0
    rng = np.random.default_rng(base_cfg.seed + 2026)

    scan_entries: List[Dict[str, float]] = []

    for l_s in lambda_smooth_values:
        for l_f in lambda_feas_values:
            for l_t in lambda_fit_values:
                combo_id += 1
                cfg = ExperimentConfig(
                    trials=scan_trials,
                    seed=base_cfg.seed,
                    num_ctrl=base_cfg.num_ctrl,
                    ts=base_cfg.ts,
                    max_vel=base_cfg.max_vel,
                    max_acc=base_cfg.max_acc,
                    lambda_smooth=l_s,
                    lambda_feas=l_f,
                    lambda_fit=l_t,
                    iter_limit=scan_iter_limit,
                    min_grad=base_cfg.min_grad,
                    output_dir=scan_output_dir,
                )

                combo_records: List[Dict[str, float]] = []
                for trial in range(scan_trials):
                    trial_seed = int(rng.integers(0, 1_000_000_000))
                    trial_rng = np.random.default_rng(trial_seed)
                    records, _, _ = run_single_trial(trial, cfg, trial_rng)
                    combo_records.extend(records)

                for optimizer in ("Bezier", "BSpline"):
                    rows = [r for r in combo_records if r["optimizer"] == optimizer]
                    viols = [max(r["max_vel_violation"], r["max_acc_violation"]) for r in rows]
                    finals = [r["final_total_cost"] for r in rows]
                    scan_entries.append(
                        {
                            "optimizer": optimizer,
                            "lambda_smooth": float(l_s),
                            "lambda_feas": float(l_f),
                            "lambda_fit": float(l_t),
                            "mean_runtime_ms": float(np.mean([r["runtime_ms"] for r in rows])),
                            "mean_final_total_cost": float(np.mean(finals)),
                            "std_final_total_cost": float(np.std(finals)),
                            "mean_cost_reduction_ratio": float(np.mean([r["cost_reduction_ratio"] for r in rows])),
                            "mean_feas_violation": float(np.mean(viols)),
                            "robust_score": 0.0,
                        }
                    )

                if combo_id % 10 == 0 or combo_id == total_combo:
                    print(f"Parameter scan progress: {combo_id}/{total_combo}")

    _score_scan_entries(scan_entries)
    _save_parameter_scan_csv(scan_entries, scan_output_dir)
    plot_parameter_scan_heatmaps(
        scan_entries,
        lambda_smooth_values,
        lambda_feas_values,
        lambda_fit_values,
        scan_output_dir,
    )
    summarize_parameter_scan(scan_entries, top_fraction, scan_output_dir)
    print(f"Parameter scan finished. Results directory: {scan_output_dir}")


def run_experiment(cfg: ExperimentConfig) -> None:
    os.makedirs(cfg.output_dir, exist_ok=True)

    rng = np.random.default_rng(cfg.seed)
    all_records: List[Dict[str, float]] = []
    history_bank: Dict[str, List[np.ndarray]] = {"Bezier": [], "BSpline": []}
    example_trajectories = None

    for trial in range(cfg.trials):
        trial_seed = int(rng.integers(0, 1_000_000_000))
        trial_rng = np.random.default_rng(trial_seed)
        records, histories, trajectories = run_single_trial(trial, cfg, trial_rng)

        all_records.extend(records)
        history_bank["Bezier"].append(histories["Bezier"])
        history_bank["BSpline"].append(histories["BSpline"])

        if trial == 0:
            example_trajectories = trajectories

    summary = summarize(all_records)

    save_records_csv(all_records, cfg.output_dir)
    save_summary_txt(summary, cfg)
    plot_convergence(history_bank, cfg.output_dir)
    plot_box_metrics(all_records, cfg.output_dir)
    if example_trajectories is not None:
        plot_example_trajectory(example_trajectories, cfg.output_dir)

    print("Experiment finished.")
    print(f"Results directory: {cfg.output_dir}")
    for name, stats in summary.items():
        print(f"[{name}]")
        for key in [
            "mean_runtime_ms",
            "mean_iterations",
            "mean_final_total_cost",
            "mean_cost_reduction_ratio",
            "mean_feasibility_cost",
            "converged_ratio",
        ]:
            print(f"  {key}: {stats[key]:.6f}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare optimization performance between B-spline and Bezier refine objectives "
            "without ROS dependencies."
        )
    )
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--num-ctrl", type=int, default=16, help="Control point count (Bezier needs 3k+1)")
    parser.add_argument("--ts", type=float, default=0.35, help="Segment/interval duration")
    parser.add_argument("--max-vel", type=float, default=3.0)
    parser.add_argument("--max-acc", type=float, default=4.0)
    parser.add_argument("--lambda-smooth", type=float, default=1.0)
    parser.add_argument("--lambda-feas", type=float, default=1.0)
    parser.add_argument("--lambda-fit", type=float, default=0.2)
    parser.add_argument("--iter-limit", type=int, default=220)
    parser.add_argument("--min-grad", type=float, default=1e-3)
    parser.add_argument("--font-scale", type=float, default=2.0, help="Global plotting font scale. 2.0 means 2x.")

    parser.add_argument("--run-scan", action="store_true", help="Run lambda_smooth/lambda_feas/lambda_fit scan.")
    parser.add_argument("--scan-trials", type=int, default=4, help="Trials per lambda tuple in parameter scan.")
    parser.add_argument("--scan-iter-limit", type=int, default=140, help="Iteration limit used in parameter scan.")
    parser.add_argument("--scan-smooth", type=str, default="0.2,0.5,1.0,2.0,4.0")
    parser.add_argument("--scan-feas", type=str, default="0.5,1.0,2.0,4.0,8.0")
    parser.add_argument("--scan-fit", type=str, default="0.05,0.1,0.2,0.4")
    parser.add_argument("--scan-top-fraction", type=float, default=0.15)

    parser.add_argument("--output-dir", type=str, default="analysis_results")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if (args.num_ctrl - 1) % 3 != 0:
        raise ValueError("--num-ctrl must satisfy 3k+1 for the Bezier objective.")

    set_plot_style(args.font_scale)

    cfg = ExperimentConfig(
        trials=args.trials,
        seed=args.seed,
        num_ctrl=args.num_ctrl,
        ts=args.ts,
        max_vel=args.max_vel,
        max_acc=args.max_acc,
        lambda_smooth=args.lambda_smooth,
        lambda_feas=args.lambda_feas,
        lambda_fit=args.lambda_fit,
        iter_limit=args.iter_limit,
        min_grad=args.min_grad,
        output_dir=args.output_dir,
    )
    run_experiment(cfg)

    if args.run_scan:
        lambda_smooth_values = parse_float_list(args.scan_smooth)
        lambda_feas_values = parse_float_list(args.scan_feas)
        lambda_fit_values = parse_float_list(args.scan_fit)

        run_parameter_scan(
            cfg,
            scan_trials=args.scan_trials,
            lambda_smooth_values=lambda_smooth_values,
            lambda_feas_values=lambda_feas_values,
            lambda_fit_values=lambda_fit_values,
            scan_iter_limit=args.scan_iter_limit,
            top_fraction=args.scan_top_fraction,
        )


if __name__ == "__main__":
    main()
