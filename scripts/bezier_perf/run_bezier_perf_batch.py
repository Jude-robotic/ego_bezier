#!/usr/bin/env python3
"""Run planner command multiple times, parse logs, and plot performance."""

import argparse
import csv
import os
import subprocess
import sys
from datetime import datetime
from typing import List, Dict

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from bezier_opt_perf_parse import parse_log, summarize  # noqa: E402


def run_once(command: str, timeout: int) -> str:
    proc = subprocess.run(
        command,
        shell=True,
        executable="/bin/bash",
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout,
        check=False,
    )
    return proc.stdout


def write_text(path: str, content: str) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)


def aggregate_rows(run_idx: int, summary: Dict[str, float]) -> Dict[str, object]:
    return {
        "run_index": run_idx,
        "num_replans": summary.get("num_replans"),
        "success_rate": summary.get("success_rate"),
        "avg_total_time": summary.get("avg_total_time"),
        "avg_optimize_time": summary.get("avg_optimize_time"),
        "avg_refine_time": summary.get("avg_refine_time"),
        "avg_lbfgs_final_cost": summary.get("avg_lbfgs_final_cost"),
        "avg_iter_count": summary.get("avg_iter_count"),
    }


def write_csv(path: str, rows: List[Dict[str, object]]) -> None:
    if not rows:
        return
    with open(path, "w", newline="", encoding="utf-8") as f:
        fieldnames = list(rows[0].keys())
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Execute planner command repeatedly, save raw logs, and output per-run summary CSV. "
            "For plotting each run, call bezier_opt_perf_parse.py and bezier_opt_perf_plot.py on raw logs."
        )
    )
    parser.add_argument("--command", required=True, help="Planner command to execute (quoted).")
    parser.add_argument("--runs", type=int, default=5, help="Number of runs.")
    parser.add_argument("--timeout", type=int, default=600, help="Timeout per run in seconds.")
    parser.add_argument("--outdir", default="scripts/bezier_perf/output/batch", help="Output directory.")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    aggregate: List[Dict[str, object]] = []

    for i in range(args.runs):
        print(f"[Run {i + 1}/{args.runs}] executing command...")
        try:
            output = run_once(args.command, args.timeout)
        except subprocess.TimeoutExpired:
            output = "[TIMEOUT] command timed out.\n"

        log_path = os.path.join(args.outdir, f"run_{i + 1:03d}_{ts}.log")
        write_text(log_path, output)

        replans = parse_log(log_path)
        summary = summarize(replans)
        aggregate.append(aggregate_rows(i + 1, summary))

        print(
            f"  saved: {log_path} | replans={summary.get('num_replans')} "
            f"success_rate={summary.get('success_rate')} avg_total_time={summary.get('avg_total_time')}"
        )

    csv_path = os.path.join(args.outdir, f"batch_summary_{ts}.csv")
    write_csv(csv_path, aggregate)
    print("[Batch Done]")
    print(f"  summary_csv: {csv_path}")


if __name__ == "__main__":
    main()
