#!/usr/bin/env python3
"""Play a recorded run (launch instruction:=... output_dir:=<run>) on the minimap, without ROS: to show a run again,
or to look at the minimap on a computer without ROS.

    python3 naive-pp/scripts/minimap_replay.py naive-pp/outputs/<run> [--speed 4] [--port 8765] [--loop]
    # then open http://localhost:8765

The robot moves where the localizer put it (trajectory.csv, map columns) and naive_pp's feedback events arrive at
their simulation times. The recording keeps only the last /plan (plan.csv): the first plan is computed again from the
planned event's start pose (the planner is deterministic without LiDAR obstacles), and from the first replan on the
minimap shows plan.csv.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from naive_pp.grounding import load_keywords  # noqa: E402
from naive_pp.minimap import Minimap, serve  # noqa: E402
from naive_pp.package import load_package  # noqa: E402
from naive_pp.planner import NaivePlanner, PlannerConfig  # noqa: E402


def local(path):
    """A package path recorded inside the Docker container (/ws = the repository) on this computer."""
    if not path:
        return None
    path = Path(path)
    if not path.is_dir() and path.parts[:2] == ("/", "ws"):
        path = HERE.parent.joinpath(*path.parts[2:])
    return path if path.is_dir() else None


def timeline(run: Path):
    events = [json.loads(line) for line in (run/"feedback.jsonl").read_text(encoding="utf-8").splitlines() if line]
    items = [(e["sim_time_s"], 1, "event", e) for e in events]
    with open(run/"trajectory.csv", newline="") as f:
        for row in csv.DictReader(f):
            pose = tuple(float(row[k]) for k in ("map_x", "map_y", "map_yaw"))
            if all(math.isfinite(v) for v in pose):
                items.append((float(row["t"]), 0, "pose", pose))
    return sorted(items, key=lambda item: item[:2])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path)
    ap.add_argument("--package", type=Path, help="map package (default: the run's launch_args.json, else campus_dev)")
    ap.add_argument("--speed", type=float, default=4.0, help="simulation seconds per wall second")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--delay", type=float, default=3.0, help="wall seconds before the replay starts")
    ap.add_argument("--loop", action="store_true", help="play again and again")
    args = ap.parse_args()
    run = args.run
    launch = json.loads((run/"launch_args.json").read_text()) if (run/"launch_args.json").is_file() else {}
    package = args.package or local(launch.get("package")) or HERE/"maps/campus_dev"
    pkg = load_package(package)
    pp = NaivePlanner(pkg, PlannerConfig(goal_mode=launch.get("goal_mode", "center")),
                      load_keywords(HERE/"configs/place_keywords.yaml"))
    instruction = (run/"instruction.txt").read_text(encoding="utf-8").strip()
    last_plan = None
    if (run/"plan.csv").is_file():
        with open(run/"plan.csv", newline="") as f:
            last_plan = [(float(r["x"]), float(r["y"])) for r in csv.DictReader(f)]
    items = timeline(run)
    view = Minimap(pkg)
    serve(view, args.host, args.port)
    print(f"minimap replay of {run.name} on {pkg.folder.name}: http://localhost:{args.port}  (Ctrl+C to stop)")
    try:
        while True:
            time.sleep(args.delay)
            view.set_localization("localized")  # the recording client sends the instruction once the robot has a fix
            t0, wall0 = items[0][0], time.monotonic()
            view.on_instruction(instruction, t0)
            for t, _, kind, item in items:
                wait = (t-t0)/args.speed-(time.monotonic()-wall0)
                if wait > 0:
                    time.sleep(wait)
                if kind == "pose":
                    view.set_pose(item, t)
                    continue
                if item["state"] == "planned":
                    g = pp.ground(instruction)
                    view.set_plan(pp.plan(tuple(item["start"]), g.target, g.waypoints).points)
                elif item["state"] == "replanned" and last_plan:
                    view.set_plan(last_plan)
                view.on_feedback(item)
            print(f"{run.name}: {items[-1][3].get('state') if items[-1][2] == 'event' else 'end'}")
            if not args.loop:
                while True:
                    time.sleep(3600)
            time.sleep(5.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
