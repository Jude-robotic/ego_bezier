#!/usr/bin/env python3
"""Parse Bezier optimizer logs and export structured performance data."""

import argparse
import csv
import json
import math
import os
import re
from dataclasses import dataclass, asdict, field
from typing import Dict, List, Optional

ANSI_ESCAPE_RE = re.compile(r"\x1B\[[0-?]*[ -/]*[@-~]")
REPLAN_RE = re.compile(r"\[rebo replan\]:\s*-+(\d+)")
ITER_RE = re.compile(r"\[(?:Opt|Bezier FORMATION) iter\s+(\d+)\](.*)")
KV_RE = re.compile(r"([A-Za-z_]+)=([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)")
TIME_RE = re.compile(
    r"total\s*time:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,\s*"
    r"optimize:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,\s*"
    r"refine:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"
)
FIRST_SUCCESS_RE = re.compile(r"first_optimize_step_success\s*=\s*(\d+)")
FINAL_SUCCESS_RE = re.compile(r"final_plan_success\s*=\s*(\d+)")
FINAL_COST_RE = re.compile(r"final_cost\s*=\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)")


@dataclass
class IterRecord:
    iteration: int
    smooth: float = math.nan
    dist: float = math.nan
    feas: float = math.nan
    form: float = math.nan
    total: float = math.nan
    grad_norm: float = math.nan


@dataclass
class ReplanRecord:
    replan_id: int
    first_optimize_success: Optional[int] = None
    final_plan_success: Optional[int] = None
    total_time: float = math.nan
    optimize_time: float = math.nan
    refine_time: float = math.nan
    lbfgs_final_cost: float = math.nan
    iterations: List[IterRecord] = field(default_factory=list)

    def to_overview_row(self) -> Dict[str, object]:
        last_iter = self.iterations[-1] if self.iterations else None
        return {
            "replan_id": self.replan_id,
            "first_optimize_success": self.first_optimize_success,
            "final_plan_success": self.final_plan_success,
            "total_time": self.total_time,
            "optimize_time": self.optimize_time,
            "refine_time": self.refine_time,
            "lbfgs_final_cost": self.lbfgs_final_cost,
            "iter_count": len(self.iterations),
            "last_iter": last_iter.iteration if last_iter else "",
            "last_smooth": last_iter.smooth if last_iter else math.nan,
            "last_dist": last_iter.dist if last_iter else math.nan,
            "last_feas": last_iter.feas if last_iter else math.nan,
            "last_form": last_iter.form if last_iter else math.nan,
            "last_total": last_iter.total if last_iter else math.nan,
            "last_grad_norm": last_iter.grad_norm if last_iter else math.nan,
        }


def strip_ansi(text: str) -> str:
    return ANSI_ESCAPE_RE.sub("", text)


def parse_float(value: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def ensure_current(replans: List[ReplanRecord], current: Optional[ReplanRecord]) -> ReplanRecord:
    if current is not None:
        return current
    next_id = replans[-1].replan_id + 1 if replans else 0
    current = ReplanRecord(replan_id=next_id)
    replans.append(current)
    return current


def parse_log(log_path: str) -> List[ReplanRecord]:
    replans: List[ReplanRecord] = []
    current: Optional[ReplanRecord] = None

    with open(log_path, "r", encoding="utf-8", errors="ignore") as f:
        for raw_line in f:
            line = strip_ansi(raw_line).strip()
            if not line:
                continue

            replan_match = REPLAN_RE.search(line)
            if replan_match:
                current = ReplanRecord(replan_id=int(replan_match.group(1)))
                replans.append(current)
                continue

            iter_match = ITER_RE.search(line)
            if iter_match:
                current = ensure_current(replans, current)
                iteration = int(iter_match.group(1))
                metrics_blob = iter_match.group(2)
                pairs = {k: parse_float(v) for k, v in KV_RE.findall(metrics_blob)}
                current.iterations.append(
                    IterRecord(
                        iteration=iteration,
                        smooth=pairs.get("smooth", math.nan),
                        dist=pairs.get("dist", math.nan),
                        feas=pairs.get("feas", math.nan),
                        form=pairs.get("form", math.nan),
                        total=pairs.get("total", math.nan),
                        grad_norm=pairs.get("grad_norm", math.nan),
                    )
                )
                continue

            time_match = TIME_RE.search(line)
            if time_match:
                current = ensure_current(replans, current)
                current.total_time = parse_float(time_match.group(1))
                current.optimize_time = parse_float(time_match.group(2))
                current.refine_time = parse_float(time_match.group(3))
                continue

            first_success = FIRST_SUCCESS_RE.search(line)
            if first_success:
                current = ensure_current(replans, current)
                current.first_optimize_success = int(first_success.group(1))
                continue

            final_success = FINAL_SUCCESS_RE.search(line)
            if final_success:
                current = ensure_current(replans, current)
                current.final_plan_success = int(final_success.group(1))
                continue

            final_cost = FINAL_COST_RE.search(line)
            if final_cost:
                current = ensure_current(replans, current)
                current.lbfgs_final_cost = parse_float(final_cost.group(1))

    return replans


def nanmean(values: List[float]) -> float:
    valid = [v for v in values if not math.isnan(v)]
    return float(sum(valid) / len(valid)) if valid else math.nan


def summarize(replans: List[ReplanRecord]) -> Dict[str, float]:
    final_success = [r.final_plan_success for r in replans if r.final_plan_success is not None]
    success_rate = float(sum(1 for s in final_success if s > 0) / len(final_success)) if final_success else math.nan

    return {
        "num_replans": len(replans),
        "success_rate": success_rate,
        "avg_total_time": nanmean([r.total_time for r in replans]),
        "avg_optimize_time": nanmean([r.optimize_time for r in replans]),
        "avg_refine_time": nanmean([r.refine_time for r in replans]),
        "avg_lbfgs_final_cost": nanmean([r.lbfgs_final_cost for r in replans]),
        "avg_iter_count": nanmean([float(len(r.iterations)) for r in replans]),
    }


def write_csv(replans: List[ReplanRecord], outdir: str, prefix: str) -> Dict[str, str]:
    overview_path = os.path.join(outdir, f"{prefix}_overview.csv")
    iterations_path = os.path.join(outdir, f"{prefix}_iterations.csv")

    with open(overview_path, "w", newline="", encoding="utf-8") as f:
        rows = [r.to_overview_row() for r in replans]
        fieldnames = [
            "replan_id", "first_optimize_success", "final_plan_success",
            "total_time", "optimize_time", "refine_time", "lbfgs_final_cost",
            "iter_count", "last_iter", "last_smooth", "last_dist",
            "last_feas", "last_form", "last_total", "last_grad_norm",
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    with open(iterations_path, "w", newline="", encoding="utf-8") as f:
        fieldnames = ["replan_id", "iter", "smooth", "dist", "feas", "form", "total", "grad_norm"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for repl in replans:
            for it in repl.iterations:
                writer.writerow({
                    "replan_id": repl.replan_id,
                    "iter": it.iteration,
                    "smooth": it.smooth,
                    "dist": it.dist,
                    "feas": it.feas,
                    "form": it.form,
                    "total": it.total,
                    "grad_norm": it.grad_norm,
                })

    return {"overview_csv": overview_path, "iterations_csv": iterations_path}


def write_json(replans: List[ReplanRecord], summary: Dict[str, float], outdir: str, prefix: str) -> str:
    json_path = os.path.join(outdir, f"{prefix}_structured.json")
    payload = {
        "summary": summary,
        "replans": [
            {
                **r.to_overview_row(),
                "iterations": [asdict(it) for it in r.iterations],
            }
            for r in replans
        ],
    }
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    return json_path


def main() -> None:
    parser = argparse.ArgumentParser(description="Parse Bezier optimizer runtime logs into structured files.")
    parser.add_argument("--log", required=True, help="Path to raw planner log file.")
    parser.add_argument("--outdir", default="scripts/bezier_perf/output", help="Directory for parsed outputs.")
    parser.add_argument("--prefix", default="bezier_perf", help="Output filename prefix.")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    replans = parse_log(args.log)
    summary = summarize(replans)
    csv_paths = write_csv(replans, args.outdir, args.prefix)
    json_path = write_json(replans, summary, args.outdir, args.prefix)

    print("[Parse Done]")
    print(f"  log: {args.log}")
    print(f"  replan_count: {summary['num_replans']}")
    print(f"  success_rate: {summary['success_rate']:.3f}" if not math.isnan(summary["success_rate"]) else "  success_rate: nan")
    print(f"  overview_csv: {csv_paths['overview_csv']}")
    print(f"  iterations_csv: {csv_paths['iterations_csv']}")
    print(f"  structured_json: {json_path}")


if __name__ == "__main__":
    main()
