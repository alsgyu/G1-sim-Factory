#!/usr/bin/env python3
"""Figure and numbers of one recorded run (launch instruction:=... output_dir:=<run>): what was planned, where the
robot walked, where it stopped.

    python3 naive-pp/scripts/report_run.py naive-pp/outputs/<run> [more runs ...]

Writes <run>/route.png and <run>/report.json. Simulation only, and only for evaluation: with --truth (default when the
package has evaluation.json) the simulator's /odom (MuJoCo without odom drift = the true pose) is moved into the map
frame with evaluation.json, to measure the localizer's error and whether the robot truly stands in the target room.
Nothing here feeds back into the robot.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import ImageDraw

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE/"scripts"))
from naive_pp.package import load_package  # noqa: E402
from naive_pp.planner import NaivePlanner  # noqa: E402
from g1_factory.localization import compose, inverse  # noqa: E402
from g1_factory.map_package import world_from_map  # noqa: E402
from plan_offline import figure  # noqa: E402


def read_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    return np.array([[float(v) for v in r] for r in rows[1:]], float).reshape(-1, len(rows[0])) if rows else None


def report(run: Path, pp: NaivePlanner, truth: bool, obstacles=None):
    summary = json.loads((run/"summary.json").read_text(encoding="utf-8"))
    events = [json.loads(line) for line in (run/"feedback.jsonl").read_text(encoding="utf-8").splitlines() if line]
    planned = [e for e in events if e["state"] in ("planned", "replanned")]
    final = summary["final_event"]
    traj = read_csv(run/"trajectory.csv")
    plan_pts = read_csv(run/"plan.csv") if (run/"plan.csv").is_file() else None
    out = {"instruction": summary["instruction"], "state": summary["state"],
           "target": planned[0]["target"]["label"] if planned else None,
           "route": planned[0]["route_text"] if planned else None,
           "planned_length_m": planned[0]["length_m"] if planned else None,
           "replans": sum(e["state"] == "replanned" for e in events),
           "duration_s": summary["duration_s"], "travelled_m": final.get("travelled_m"),
           "goal_distance_m": final.get("goal_distance_m"), "final_place": final.get("place"),
           "in_target_place": final.get("in_target_place")}
    true_map = None
    if truth and traj is not None and len(traj):
        w = world_from_map(pp.pkg.folder)
        map_from_world = inverse((w[0, 2], w[1, 2], math.atan2(w[1, 0], w[0, 0])))
        true_map = np.array([compose(map_from_world, tuple(r[4:7])) for r in traj])
        est = traj[:, 1:4]
        ok = np.isfinite(est).all(axis=1)
        err = np.hypot(*(est[ok, :2]-true_map[ok, :2]).T)
        goal = planned[-1]["goal"] if planned else None
        tx, ty, _ = true_map[-1]
        target = planned[0]["target"] if planned else None
        place = pp.pkg.place_at((tx, ty))
        out["truth"] = {"localization_error_mean_m": round(float(err.mean()), 3),
                        "localization_error_max_m": round(float(err.max()), 3),
                        "final_true_pose_map": [round(float(v), 3) for v in true_map[-1]],
                        "true_goal_distance_m": round(math.hypot(tx-goal[0], ty-goal[1]), 3) if goal else None,
                        "true_place": pp.graph.label(place) if place else None,
                        "truly_in_target_place": bool(target and target["kind"] == "place" and place == target["id"])}
    title = f"{run.name}: {out['state']}  |  {out['target']}  |  planned {out['planned_length_m']} m, " \
            f"walked {out['travelled_m']} m in {out['duration_s']} s"
    img = figure(pp, None, title)
    d = ImageDraw.Draw(img, "RGBA")
    h = pp.pkg.grid.shape[0]
    px = lambda x, y: ((x-pp.pkg.origin[0])/pp.pkg.resolution*3, (h-(y-pp.pkg.origin[1])/pp.pkg.resolution)*3)
    if plan_pts is not None and len(plan_pts) > 1:
        d.line([px(*q) for q in plan_pts], fill=(220, 60, 30, 200), width=5)
    if traj is not None and len(traj) > 1:
        est = traj[np.isfinite(traj[:, 1:3]).all(axis=1), 1:3]
        d.line([px(*q) for q in est], fill=(40, 120, 220, 255), width=3)
        if true_map is not None:
            d.line([px(*q) for q in true_map[:, :2]], fill=(20, 20, 20, 160), width=1)
    boxes = obstacles if obstacles is not None else []
    if not boxes and (run/"launch_args.json").is_file():
        boxes = json.loads(json.loads((run/"launch_args.json").read_text()).get("unmapped_obstacles", "[]"))
    if boxes and truth:  # simulator-frame boxes the map does not have, drawn where they really are
        w = world_from_map(pp.pkg.folder)
        map_from_world = inverse((w[0, 2], w[1, 2], math.atan2(w[1, 0], w[0, 0])))
        for bx, by, sx, sy, *_ in boxes:
            corners = [compose(map_from_world, (bx+dx*sx/2, by+dy*sy/2, 0.0))[:2]
                       for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
            d.polygon([px(*q) for q in corners], fill=(150, 60, 200, 200), outline=(80, 20, 120))
        out["unmapped_obstacles"] = boxes
    if planned:
        gx, gy, gyaw = planned[-1]["goal"]
        x, y = px(gx, gy)
        d.ellipse([x-9, y-9, x+9, y+9], fill=(30, 160, 70), outline=(255, 255, 255), width=2)
        d.line([(x, y), (x+22*math.cos(gyaw), y-22*math.sin(gyaw))], fill=(30, 160, 70), width=4)
    d.text((10, img.height-12), "red: last plan   blue: localizer pose   thin black: simulator truth   purple: box not "
                                "on the map (evaluation only)",
           fill=(30, 30, 30), anchor="ls")
    img.save(run/"route.png")
    (run/"report.json").write_text(json.dumps(out, indent=1, ensure_ascii=False)+"\n", encoding="utf-8")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--package", type=Path, default=HERE/"maps/campus_dev")
    ap.add_argument("--no-truth", action="store_true")
    ap.add_argument("--obstacles", help="JSON [[x, y, sx, sy, sz]] simulator-frame boxes not on the map "
                                        "(default: from <run>/launch_args.json)")
    args = ap.parse_args()
    pp = NaivePlanner(load_package(args.package))
    truth = not args.no_truth and (args.package/"evaluation.json").is_file()
    for run in args.runs:
        print(json.dumps(report(run, pp, truth, json.loads(args.obstacles) if args.obstacles else None),
                         ensure_ascii=False))


if __name__ == "__main__":
    main()
