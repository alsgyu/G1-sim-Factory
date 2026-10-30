#!/usr/bin/env python3
"""2. Localisation and zone judgement for the report, from the records of launch/g1_localization_eval.launch.py.

    bash scripts/g1_docker.sh python3 scripts/eval_localization.py outputs/metrics/localization

2.1 localisation on the final map with LiDAR
    tracking   error of the localised pose against the truth while walking the route (mean, RMSE, median, 95th
               percentile, max; heading), and after a best-fit rigid alignment of the whole run (removes the
               package's global offset to the simulator frame, which comes from the mapping start)
    first fix  fresh searches from rough guesses along the route: correct / refused / wrong, inside the search
               window (0.5-1.5 m, 0-20 deg) and outside it (2.5-5 m, 0-30 deg); first fix of the run
2.2 zone judgement: the zone from the localised pose and the annotated room bounds (as vln_navigator decides it)
    against the simulator's zone at the true pose; accuracy, per zone, confusion matrix, where the misses are.
    5 Hz samples along a route are not independent: the 95 % interval uses one sample per metre walked.
    --grid: also the same rule at evenly sampled true positions (no localisation error), for every place.
Writes <dir>/localization_metrics.json, localization_metrics.md, fig_localization.png.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from g1_factory import metrics as X  # noqa: E402
from g1_factory import slam_map as S  # noqa: E402


def font(size, bold=False):
    path = "/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf" % ("-Bold" if bold else "")
    try:
        return ImageFont.truetype(path, size)
    except OSError:
        return ImageFont.load_default()


def rows(path: Path):
    with path.open() as f:
        return list(csv.DictReader(f))


def boundary_distance(xy, zones: dict) -> float:
    """Distance from a point to the nearest edge of any zone rectangle."""
    x, y = xy
    best = math.inf
    for x0, y0, x1, y1 in zones.values():
        dx = max(x0-x, 0.0, x-x1)
        dy = max(y0-y, 0.0, y-y1)
        if dx == 0 and dy == 0:
            best = min(best, x-x0, x1-x, y-y0, y1-y)
        else:
            best = min(best, math.hypot(dx, dy))
    return best


def thin_per_metre(samples):
    out, last = [], -math.inf
    for s in samples:
        if float(s["travelled_m"]) >= last+1.0:
            out.append(s)
            last = float(s["travelled_m"])
    return out


def tracking(samples, status) -> dict:
    err = [float(s["position_error_m"]) for s in samples]
    est = np.array([[float(s["x_m"]), float(s["y_m"])] for s in samples])
    truth = np.array([[float(s["true_x_m"]), float(s["true_y_m"])] for s in samples])
    fixes = [s for s in status if s["state"] == "localized"]
    return {"samples": len(samples), "path_m": float(samples[-1]["travelled_m"]) if samples else 0.0,
            "duration_s": round(float(samples[-1]["sim_time_s"])-float(samples[0]["sim_time_s"]), 1) if samples else 0,
            "error_m": X.stats(err), "yaw_error_deg": X.stats([float(s["yaw_error_deg"]) for s in samples]),
            "error_m_best_fit": X.ate(est, truth, align=True) if len(samples) > 2 else {},
            "first_fix": {"sim_time_s": fixes[0]["t"], "correction_m": fixes[0].get("correction_m"),
                          "correction_deg": fixes[0].get("correction_deg"), "inliers": fixes[0].get("inliers")}
            if fixes else None,
            "lost_events": sum(1 for s in status if s["state"] == "lost")}


def fix_trials(trials) -> dict:
    out = {}
    for window in ("in_window", "out_of_window"):
        group = [t for t in trials if t["window"] == window]
        outcomes = []
        for t in group:
            err = float(t["error_m"]) if t["error_m"] not in ("", "None") else None
            deg = float(t["error_deg"]) if t["error_deg"] not in ("", "None") else None
            outcomes.append(X.fix_outcome(t["localized"] == "True", err, deg))
        n = len(group)
        correct = [float(t["error_m"]) for t, o in zip(group, outcomes) if o == "correct"]
        wrong = [float(t["error_m"]) for t, o in zip(group, outcomes) if o == "wrong" and t["error_m"] not in ("", "None")]
        out[window] = {"trials": n, "correct": X.rate(outcomes.count("correct"), n),
                       "refused": X.rate(outcomes.count("refused"), n), "wrong": X.rate(outcomes.count("wrong"), n),
                       "error_m_when_correct": X.stats(correct), "error_m_when_wrong": X.stats(wrong),
                       "latency_s": X.stats([float(t["latency_s"]) for t in group])}
    out["tolerance"] = {"position_m": 0.5, "heading_deg": 5.0}
    return out


def zones(samples, truth_zones) -> dict:
    out = X.confusion([(s["zone_truth"], s["zone_robot"]) for s in samples])
    thin = thin_per_metre(samples)
    out["accuracy_per_metre"] = X.rate(sum(s["zone_truth"] == s["zone_robot"] for s in thin), len(thin))
    miss = [s for s in samples if s["zone_truth"] != s["zone_robot"]]
    if truth_zones:
        out["miss_boundary_distance_m"] = X.stats([boundary_distance((float(s["true_x_m"]), float(s["true_y_m"])),
                                                                     truth_zones) for s in miss])
    return out


def grid_zones(package: Path, truth: dict, meta_path: Path, per_place=60) -> dict:
    """Zone rule at evenly sampled true positions of every place (no localisation error): the map's share."""
    from g1_factory.map_package import load_package, vln_inputs
    from g1_factory.recognition import sample_poses
    from g1_factory.roadmap import region_at
    semantic = load_package(package, "world")[3]
    rooms = vln_inputs(semantic)[0]
    truth_zones = {z["id"]: tuple(z["bounds"]) for z in truth["zones"]}
    poses = sample_poses(truth, json.loads(meta_path.read_text()), per_place=per_place, seed=0)
    out = X.confusion([(region_at(truth_zones, (p["x"], p["y"])), region_at(rooms, (p["x"], p["y"]))) for p in poses])
    out["poses"] = len(poses)
    return out


def figure(samples, package: Path, out_path: Path, scale=2):
    grid, origin, res = S.read_map_yaml(package/"map_world.yaml")
    rgb = np.full(grid.shape+(3,), (214, 217, 222), np.uint8)
    rgb[grid == S.FREE] = (255, 255, 255)
    rgb[grid == S.OCCUPIED] = (110, 114, 122)
    img = Image.fromarray(np.flipud(rgb)).resize((grid.shape[1]*scale, grid.shape[0]*scale), Image.NEAREST)
    d = ImageDraw.Draw(img)
    h = grid.shape[0]
    px = lambda x, y: ((x-origin[0])/res*scale, (h-(y-origin[1])/res)*scale)
    err = np.array([float(s["position_error_m"]) for s in samples])
    top = max(0.3, float(err.max())) if len(err) else 0.3
    for s, e in zip(samples, err):
        x, y = px(float(s["true_x_m"]), float(s["true_y_m"]))
        t = min(e/top, 1.0)
        c = (int(40+215*t), int(170-120*t), 60)
        r = 3 if s["zone_truth"] == s["zone_robot"] else 6
        d.ellipse([x-r, y-r, x+r, y+r], fill=c, outline=None if r == 3 else (20, 20, 20))
    chart_h = 260
    out = Image.new("RGB", (img.width, img.height+70+chart_h), (255, 255, 255))
    out.paste(img, (0, 70))
    dd = ImageDraw.Draw(out)
    dd.text((12, 8), "Localisation along the route (simulator frame)", fill=(20, 20, 20), font=font(24, True))
    dd.text((12, 42), f"true path coloured by position error (green 0 .. red {top:.2f} m); large dots: wrong zone",
            fill=(80, 80, 80), font=font(16))
    y0, x0, w = img.height+90, 60, img.width-90
    hgt = chart_h-50
    dist = np.array([float(s["travelled_m"]) for s in samples])
    if len(dist) > 1:
        dd.line([(x0, y0), (x0, y0+hgt), (x0+w, y0+hgt)], fill=(120, 120, 120))
        pts = [(x0+w*di/max(dist[-1], 1e-6), y0+hgt-hgt*min(e/top, 1.0)) for di, e in zip(dist, err)]
        dd.line(pts, fill=(235, 110, 20), width=2)
        for k in range(5):
            v = top*k/4
            dd.text((x0-6, y0+hgt-hgt*k/4), f"{v:.2f}", fill=(80, 80, 80), anchor="rm", font=font(13))
        dd.text((x0+w/2, y0+hgt+20), f"metres walked (0 .. {dist[-1]:.0f} m); y: position error (m)",
                fill=(60, 60, 60), anchor="mm", font=font(14))
    out.save(out_path, optimize=True)


def fmt(v, d=3):
    return "-" if v is None else f"{v:.{d}f}"


def ci(r):
    return f"{fmt(r['rate']*100 if r['rate'] is not None else None, 1)}% ({r['k']}/{r['n']}, 95% {fmt(r['ci95'][0]*100, 1)}-{fmt(r['ci95'][1]*100, 1)}%)"


def markdown(m: dict) -> str:
    t, f, z = m["tracking"], m["first_fix"], m["zone"]
    e, b = t["error_m"], t["error_m_best_fit"]
    first = t["first_fix"] or {}
    lines = ["# 위치 및 구역 판정", "", f"지도 묶음 `{m['package']}`, 경로 {t['path_m']:.0f} m, 시뮬레이션 {t['duration_s']} s, 표본 {t['samples']}개", "",
             "## 2.1 최종 지도에서 LiDAR 위치 추정", "",
             "| 지표 | 평균 | RMSE | 중앙값 | 95% | 최대 |", "|---|---:|---:|---:|---:|---:|",
             f"| 위치 오차, 정답 대비 (m) | {fmt(e.get('mean'))} | {fmt(e.get('rmse'))} | {fmt(e.get('median'))} | {fmt(e.get('p95'))} | {fmt(e.get('max'))} |",
             f"| 위치 오차, 최적 정렬 후 (m) | {fmt(b.get('mean'))} | {fmt(b.get('rmse'))} | {fmt(b.get('median'))} | {fmt(b.get('p95'))} | {fmt(b.get('max'))} |",
             f"| 방향 오차 (도) | {fmt(t['yaw_error_deg'].get('mean'), 2)} | {fmt(t['yaw_error_deg'].get('rmse'), 2)} | {fmt(t['yaw_error_deg'].get('median'), 2)} | {fmt(t['yaw_error_deg'].get('p95'), 2)} | {fmt(t['yaw_error_deg'].get('max'), 2)} |",
             "", f"시작 위치 추정: 시뮬레이션 {fmt(first.get('sim_time_s'), 1)} s, 초기 자세에서 {fmt(first.get('correction_m'), 2)} m, "
             f"{fmt(first.get('correction_deg'), 1)}도 보정. lost {t['lost_events']}회.", "",
             "| 첫 위치 추정 시험 | 시도 | 맞음 | 거절 | 잘못 맞춤 | 맞았을 때 오차 평균 (m) | 잘못 맞췄을 때 오차 최소-최대 (m) | 계산 시간 평균 (s) |",
             "|---|---:|---|---|---|---:|---:|---:|"]
    lines.insert(-2, "맞음은 정답에서 0.5 m, 5도 안, 잘못 맞춤은 위치를 찾았다고 했지만 그보다 먼 경우, 거절은 받아들이지 않은 경우입니다.")
    lines.insert(-2, "")
    for key, name in (("in_window", "초기 자세 0.5-1.5 m, 0-20도 틀림"), ("out_of_window", "초기 자세 2.5-5 m, 0-30도 틀림")):
        r = f[key]
        w = r["error_m_when_wrong"]
        wrong = f"{fmt(w['min'], 2)}-{fmt(w['max'], 2)}" if w.get("n") else "-"
        lines.append(f"| {name} | {r['trials']} | {ci(r['correct'])} | {ci(r['refused'])} | {ci(r['wrong'])} | "
                     f"{fmt(r['error_m_when_correct'].get('mean'))} | {wrong} | {fmt(r['latency_s'].get('mean'), 2)} |")
    lines += ["", "## 2.2 위치와 구역 범위로 현재 구역 판정", "",
              f"정확도 {ci(z['accuracy'])}, 1 m마다 한 표본으로 줄인 정확도와 95% 구간 {ci(z['accuracy_per_metre'])}.", ""]
    if z.get("miss_boundary_distance_m", {}).get("n"):
        s = z["miss_boundary_distance_m"]
        lines += [f"틀린 표본이 정답 구역 경계에서 떨어진 거리: 평균 {fmt(s['mean'], 2)} m, 최대 {fmt(s['max'], 2)} m.", ""]
    lines += ["| 정답 구역 | 표본 | 정확도 |", "|---|---:|---:|"]
    lines += [f"| {k} | {v['n']} | {fmt(v['rate']*100 if v['rate'] is not None else None, 1)}% |" for k, v in z["per_zone"].items()]
    if m.get("zone_grid"):
        g = m["zone_grid"]
        lines += ["", f"참고, 위치 오차 없이 정답 위치 {g['poses']}곳(장소마다 고르게)에서 같은 규칙: {ci(g['accuracy'])}. "
                  "지도의 구역 범위(사람 주석)만의 오차입니다."]
    return "\n".join(lines)+"\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folder", type=Path, help="output_dir of g1_localization_eval.launch.py")
    parser.add_argument("--grid", action="store_true", help="also the zone rule at evenly sampled true positions")
    parser.add_argument("--meta", type=Path, default=ROOT/"outputs/isaac/campus_meta.json")
    args = parser.parse_args()
    run = json.loads((args.folder/"run.json").read_text())
    package = Path(run["package"])
    if not package.is_dir():  # recorded inside the container (/ws/...)
        package = ROOT/Path(*Path(run["package"]).parts[2:])
    samples = rows(args.folder/"samples.csv")
    trials = rows(args.folder/"fix_trials.csv")
    status = [json.loads(line) for line in (args.folder/"status.jsonl").read_text().splitlines() if line]
    truth_path = Path(run["truth_semantic"])
    if not truth_path.is_file():
        truth_path = ROOT/"outputs/isaac/semantic_map.json"
    truth = json.loads(truth_path.read_text()) if truth_path.is_file() else None
    truth_zones = {z["id"]: tuple(z["bounds"]) for z in truth["zones"]} if truth else {}
    result = {"package": str(package), "run": run, "tracking": tracking(samples, status),
              "first_fix": fix_trials(trials), "zone": zones(samples, truth_zones)}
    if args.grid and truth and args.meta.is_file():
        result["zone_grid"] = grid_zones(package, truth, args.meta)
    figure(samples, package, args.folder/"fig_localization.png")
    (args.folder/"localization_metrics.json").write_text(json.dumps(result, indent=1, ensure_ascii=False))
    (args.folder/"localization_metrics.md").write_text(markdown(result), encoding="utf-8")
    print(markdown(result))


if __name__ == "__main__":
    main()
