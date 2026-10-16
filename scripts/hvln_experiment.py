#!/usr/bin/env python3
"""Run the hierarchical-VLN experiment matrix and aggregate the episode summaries.

    # Ubuntu GPU station, Docker image from docker/Dockerfile.ros2, VLM server on this host
    python3 scripts/hvln_experiment.py --out outputs/exp_2026-10-20 --runner docker
    # native ROS 2 (shell already sourced: /opt/ros/$ROS_DISTRO/setup.bash + .venv-ros)
    python3 scripts/hvln_experiment.py --out outputs/exp_2026-10-20 --runner native
    # only rebuild the tables from finished episodes
    python3 scripts/hvln_experiment.py --out outputs/exp_2026-10-20 --aggregate-only

Episodes that already have summary.json are skipped, so an interrupted run resumes.
Outputs: <out>/<condition>/<task>/seed<k>/{summary.json, feedback.jsonl, ...},
<out>/episodes.csv, <out>/results.md, <out>/measurement/topic_stats.json + topic_tables.md.
Needs only Python 3 + PyYAML (python3-yaml, installed with ROS 2); no ROS import, so it can drive Docker.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import shlex
import statistics
import subprocess
import sys
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]


def launch_command(args, condition: dict, task: str, seed: int, out: Path, extra: dict) -> str:
    params = {"task": task, "seed": seed, "condition": condition["name"], "planner_mode": condition["planner_mode"],
              "vlm_model": condition["vlm_model"], "vlm_url": args.vlm_url, **extra}
    rel = out.resolve().relative_to(ROOT)
    params["output_dir"] = f"/ws/{rel}" if args.runner == "docker" else str(out.resolve())
    joined = " ".join(f"{k}:={shlex.quote(str(v))}" for k, v in params.items())
    return f"ros2 launch launch/g1_hvln.launch.py {joined}"


def run_episode(args, command: str, log: Path, timeout: float) -> int:
    env = dict(os.environ)
    if args.runner == "docker":
        docker = ["docker", "run", "--rm", "-v", f"{ROOT}:/ws", "-e", f"ROS_DOMAIN_ID={args.domain_id}"]
        docker += shlex.split(args.docker_args)
        full = docker+[args.image, "bash", "-c", f"timeout {int(timeout)} {command}"]
    else:
        env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        env.setdefault("CYCLONEDDS_URI", f"file://{ROOT}/configs/cyclonedds.xml")
        env.setdefault("MUJOCO_GL", "egl")
        env["ROS_DOMAIN_ID"] = str(args.domain_id)
        full = ["bash", "-c", f"timeout {int(timeout)} {command}"]
    with log.open("w") as stream:
        stream.write("$ "+" ".join(shlex.quote(c) for c in full)+"\n")
        stream.flush()
        return subprocess.run(full, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT).returncode


def mean(values):
    values = [v for v in values if v is not None]
    return statistics.fmean(values) if values else None


def fmt(value, pct=False, digits=2):
    if value is None:
        return "-"
    return f"{100*value:.0f}%" if pct else f"{value:.{digits}f}"


COLUMNS = ["condition", "task", "seed", "state", "success", "grounding_correct", "navigation_complete", "spl",
           "final_distance_to_expected_place_m", "collision_events", "fallen", "replans", "verdict_correct",
           "vlm_latency_mean_s", "task_sim_time_s", "travelled_m", "shortest_path_m", "cross_track_rmse_m",
           "planned_subgoals", "verdict", "answer"]


def aggregate(out: Path):
    rows = []
    for summary_path in sorted(out.glob("*/*/seed*/summary.json")):
        s = json.loads(summary_path.read_text(encoding="utf-8"))
        inspection = s.get("inspection") or {}
        rmse = s.get("cross_track_rmse_m_per_goal") or []
        rows.append({**{k: s.get(k) for k in COLUMNS if k in s},
                     "condition": summary_path.parts[-4], "cross_track_rmse_m": mean(rmse),
                     "planned_subgoals": " > ".join(s.get("planned_subgoals") or []),
                     "verdict": inspection.get("verdict"), "answer": inspection.get("answer")})
    if not rows:
        print("No finished episodes yet.")
        return
    with (out/"episodes.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows({k: r.get(k) for k in COLUMNS} for r in rows)
    lines = ["# Hierarchical VLN results", "",
             "| 조건 | 에피소드 | 성공률 | grounding 정확도 | 항법 완료 | SPL | 최종 오차(m) | 충돌/넘어짐 | 판정 정확도 | 재계획 | VLM 지연(s) | 경로추종 RMSE(m) | 작업 시간(sim s) |",
             "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for condition in sorted({r["condition"] for r in rows}):
        group = [r for r in rows if r["condition"] == condition]
        verdicts = [r["verdict_correct"] for r in group if r["verdict_correct"] is not None]
        lines.append(
            f"| {condition} | {len(group)} | {fmt(mean([float(bool(r['success'])) for r in group]), True)} | "
            f"{fmt(mean([float(bool(r['grounding_correct'])) for r in group]), True)} | "
            f"{fmt(mean([float(bool(r['navigation_complete'])) for r in group]), True)} | "
            f"{fmt(mean([r['spl'] for r in group]))} | {fmt(mean([r['final_distance_to_expected_place_m'] for r in group]))} | "
            f"{sum(1 for r in group if r['collision_events'])}/{sum(1 for r in group if r['fallen'])} | "
            f"{fmt(mean([float(v) for v in verdicts]), True)} | {sum(r['replans'] or 0 for r in group)} | "
            f"{fmt(mean([r['vlm_latency_mean_s'] for r in group]))} | {fmt(mean([r['cross_track_rmse_m'] for r in group]), digits=3)} | "
            f"{fmt(mean([r['task_sim_time_s'] for r in group]), digits=1)} |")
    lines += ["", "## 작업별 성공률", "", "| 작업 | "+" | ".join(sorted({r['condition'] for r in rows}))+" |",
              "|---|"+"---:|"*len({r['condition'] for r in rows})]
    for task in sorted({r["task"] for r in rows}):
        cells = []
        for condition in sorted({r["condition"] for r in rows}):
            group = [r for r in rows if r["condition"] == condition and r["task"] == task]
            cells.append(f"{sum(bool(r['success']) for r in group)}/{len(group)}" if group else "-")
        lines.append(f"| {task} | "+" | ".join(cells)+" |")
    (out/"results.md").write_text("\n".join(lines)+"\n", encoding="utf-8")
    print("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=ROOT/"configs/hvln_experiment.yaml")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--runner", choices=["docker", "native"], default="docker")
    parser.add_argument("--image", default="g1-sim-ros2:humble")
    parser.add_argument("--docker-args", default="--gpus all --network host --ipc host",
                        help="Extra docker run flags (macOS without NVIDIA: \"--add-host host.docker.internal:host-gateway\")")
    parser.add_argument("--vlm-url", default="http://127.0.0.1:11434/v1")
    parser.add_argument("--domain-id", type=int, default=42, help="ROS_DOMAIN_ID isolating these runs")
    parser.add_argument("--conditions", nargs="*", help="Subset of condition names")
    parser.add_argument("--tasks", nargs="*", help="Subset of task ids")
    parser.add_argument("--seeds", nargs="*", type=int, help="Subset of seeds")
    parser.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE", help="Override launch_args")
    parser.add_argument("--skip-measurement", action="store_true")
    parser.add_argument("--aggregate-only", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    out = args.out if args.out.is_absolute() else (Path.cwd()/args.out)
    if args.runner == "docker" and ROOT not in out.resolve().parents:
        parser.error("--out must be inside the repository when using the Docker runner (it is mounted at /ws)")
    out.mkdir(parents=True, exist_ok=True)
    if args.aggregate_only:
        aggregate(out)
        return 0
    spec = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    extra = dict(spec.get("launch_args", {}))
    for item in args.set:
        key, _, value = item.partition("=")
        extra[key] = value
    conditions = [{"name": n, **c} for n, c in spec["conditions"].items() if not args.conditions or n in args.conditions]
    tasks = [t for t in spec["tasks"] if not args.tasks or t in args.tasks]
    seeds = [s for s in spec["seeds"] if not args.seeds or s in args.seeds]
    timeout = float(spec.get("episode_wall_timeout_s", 3600))
    (out/"experiment.json").write_text(json.dumps({"config": spec, "launch_args": extra, "argv": sys.argv}, indent=2)+"\n")
    episodes = [(c, t, s) for c in conditions for t in tasks for s in seeds]
    if spec.get("measurement") and not args.skip_measurement:
        m = spec["measurement"]
        episodes.insert(0, ({"name": "measurement", **spec["conditions"][m["condition"]]}, m["task"], m["seed"]))
    for index, (condition, task, seed) in enumerate(episodes, 1):
        measuring = condition["name"] == "measurement"
        episode = out/"measurement" if measuring else out/condition["name"]/task/f"seed{seed}"
        if (episode/"summary.json").exists():
            print(f"[{index}/{len(episodes)}] skip {episode.relative_to(out)} (done)")
            continue
        episode_args = {**extra, **(spec["measurement"].get("launch_args", {}) if measuring else {}), "monitor": measuring}
        command = launch_command(args, condition, task, seed, episode, episode_args)
        print(f"[{index}/{len(episodes)}] {condition['name']} {task} seed{seed}", flush=True)
        if args.dry_run:
            print("   ", command)
            continue
        episode.mkdir(parents=True, exist_ok=True)
        begin = time.monotonic()
        code = run_episode(args, command, episode.with_suffix(".log") if not measuring else out/"measurement.log", timeout)
        summary = episode/"summary.json"
        result = json.loads(summary.read_text(encoding="utf-8")) if summary.exists() else {}
        print(f"    exit={code} success={result.get('success')} state={result.get('state')} "
              f"wall={time.monotonic()-begin:.0f}s", flush=True)
        if measuring and (episode/"topic_stats.json").exists():
            table = subprocess.run([sys.executable, str(ROOT/"scripts/topic_report.py"), str(episode/"topic_stats.json")],
                                   capture_output=True, text=True)
            (episode/"topic_tables.md").write_text(table.stdout, encoding="utf-8")
    if not args.dry_run:
        aggregate(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
