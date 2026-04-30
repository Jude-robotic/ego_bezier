#!/usr/bin/env python3
"""Plot Bezier optimizer performance from parsed CSV files."""

import argparse
import csv
import math
import os
from collections import defaultdict
from typing import Dict, List

import matplotlib.pyplot as plt


def to_float(v: str) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def load_overview(path: str) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                "replan_id": to_float(row.get("replan_id", "nan")),
                "first_optimize_success": to_float(row.get("first_optimize_success", "nan")),
                "final_plan_success": to_float(row.get("final_plan_success", "nan")),
                "total_time": to_float(row.get("total_time", "nan")),
                "optimize_time": to_float(row.get("optimize_time", "nan")),
                "refine_time": to_float(row.get("refine_time", "nan")),
                "iter_count": to_float(row.get("iter_count", "nan")),
                "last_total": to_float(row.get("last_total", "nan")),
            })
    rows.sort(key=lambda x: x["replan_id"])
    return rows


def load_iterations(path: str) -> Dict[int, Dict[str, List[float]]]:
    by_replan: Dict[int, Dict[str, List[float]]] = defaultdict(lambda: {
        "iter": [], "smooth": [], "dist": [], "feas": [], "form": [], "total": [], "grad_norm": []
    })
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rid = int(float(row.get("replan_id", "0")))
            by_replan[rid]["iter"].append(to_float(row.get("iter", "nan")))
            by_replan[rid]["smooth"].append(to_float(row.get("smooth", "nan")))
            by_replan[rid]["dist"].append(to_float(row.get("dist", "nan")))
            by_replan[rid]["feas"].append(to_float(row.get("feas", "nan")))
            by_replan[rid]["form"].append(to_float(row.get("form", "nan")))
            by_replan[rid]["total"].append(to_float(row.get("total", "nan")))
            by_replan[rid]["grad_norm"].append(to_float(row.get("grad_norm", "nan")))

    # Keep each replan sorted by iteration index.
    for rid, data in by_replan.items():
        order = sorted(range(len(data["iter"])), key=lambda i: data["iter"][i])
        for k in list(data.keys()):
            data[k] = [data[k][i] for i in order]
    return by_replan


def finite(values: List[float]) -> List[float]:
    return [v for v in values if not math.isnan(v)]


def choose_replans(ids: List[int], max_count: int) -> List[int]:
    if len(ids) <= max_count:
        return ids
    # Uniformly sample ids across timeline.
    chosen = []
    for i in range(max_count):
        idx = round(i * (len(ids) - 1) / (max_count - 1))
        chosen.append(ids[idx])
    return sorted(set(chosen))


def plot_overview(rows: List[Dict[str, float]], out_path: str) -> None:
    replan = [int(r["replan_id"]) for r in rows]
    total = [r["total_time"] for r in rows]
    opt = [r["optimize_time"] for r in rows]
    ref = [r["refine_time"] for r in rows]
    succ = [r["final_plan_success"] for r in rows]
    iters = [r["iter_count"] for r in rows]

    fig, axs = plt.subplots(2, 2, figsize=(14, 9))

    axs[0, 0].plot(replan, total, marker="o", linewidth=1.2, label="total_time")
    axs[0, 0].plot(replan, opt, marker="x", linewidth=1.0, label="optimize_time")
    axs[0, 0].plot(replan, ref, marker="^", linewidth=1.0, label="refine_time")
    axs[0, 0].set_title("Planner Runtime by Replan")
    axs[0, 0].set_xlabel("Replan ID")
    axs[0, 0].set_ylabel("Time (s)")
    axs[0, 0].grid(alpha=0.3)
    axs[0, 0].legend()

    succ_int = [1 if (not math.isnan(v) and v > 0) else 0 for v in succ]
    axs[0, 1].bar(replan, succ_int, color=["#2e7d32" if s > 0 else "#c62828" for s in succ_int])
    axs[0, 1].set_title("Final Plan Success")
    axs[0, 1].set_xlabel("Replan ID")
    axs[0, 1].set_ylabel("Success (0/1)")
    axs[0, 1].set_ylim(-0.05, 1.1)
    axs[0, 1].grid(alpha=0.3)

    axs[1, 0].scatter(iters, total, c=succ_int, cmap="RdYlGn", alpha=0.8, edgecolors="k", linewidths=0.4)
    axs[1, 0].set_title("Iteration Count vs Total Time")
    axs[1, 0].set_xlabel("Logged Iteration Points")
    axs[1, 0].set_ylabel("Total Time (s)")
    axs[1, 0].grid(alpha=0.3)

    finite_total = finite(total)
    axs[1, 1].hist(finite_total, bins=min(20, max(5, len(finite_total) // 2)), color="#1565c0", alpha=0.85)
    axs[1, 1].set_title("Total Runtime Distribution")
    axs[1, 1].set_xlabel("Total Time (s)")
    axs[1, 1].set_ylabel("Count")
    axs[1, 1].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def plot_convergence(by_replan: Dict[int, Dict[str, List[float]]], out_path: str) -> None:
    ids = sorted(by_replan.keys())
    selected = choose_replans(ids, max_count=6)

    fig, axs = plt.subplots(2, 1, figsize=(14, 10), sharex=False)

    for rid in selected:
        data = by_replan[rid]
        axs[0].plot(data["iter"], data["total"], linewidth=1.5, label=f"replan {rid}")
    axs[0].set_title("Cost Convergence (total)")
    axs[0].set_xlabel("Iteration")
    axs[0].set_ylabel("Total Cost")
    axs[0].grid(alpha=0.3)
    axs[0].legend(ncol=3, fontsize=9)

    if selected:
        rid_mid = selected[len(selected) // 2]
        d = by_replan[rid_mid]
        axs[1].plot(d["iter"], d["smooth"], label="smooth")
        axs[1].plot(d["iter"], d["dist"], label="dist")
        axs[1].plot(d["iter"], d["feas"], label="feas")
        if any(not math.isnan(v) and abs(v) > 0 for v in d["form"]):
            axs[1].plot(d["iter"], d["form"], label="form")
        axs[1].plot(d["iter"], d["total"], linewidth=2.0, linestyle="--", label="total")
        axs[1].set_title(f"Cost Components Example (replan {rid_mid})")
    else:
        axs[1].set_title("Cost Components Example")

    axs[1].set_xlabel("Iteration")
    axs[1].set_ylabel("Cost")
    axs[1].grid(alpha=0.3)
    axs[1].legend()

    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot Bezier optimizer performance figures from parsed CSV files.")
    parser.add_argument("--overview", required=True, help="Path to *_overview.csv")
    parser.add_argument("--iterations", required=True, help="Path to *_iterations.csv")
    parser.add_argument("--outdir", default="scripts/bezier_perf/output", help="Directory for generated figures")
    parser.add_argument("--prefix", default="bezier_perf", help="Output figure name prefix")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    rows = load_overview(args.overview)
    by_replan = load_iterations(args.iterations)

    overview_png = os.path.join(args.outdir, f"{args.prefix}_overview.png")
    convergence_png = os.path.join(args.outdir, f"{args.prefix}_convergence.png")

    plot_overview(rows, overview_png)
    plot_convergence(by_replan, convergence_png)

    print("[Plot Done]")
    print(f"  overview_png: {overview_png}")
    print(f"  convergence_png: {convergence_png}")


if __name__ == "__main__":
    main()
