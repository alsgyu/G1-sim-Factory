#!/usr/bin/env python3
"""Map accuracy for the report (simulation only): 1.1 SLAM trajectory error, 1.2 doorway detection and position.

    bash scripts/g1_docker.sh python3 scripts/eval_map.py maps/campus --runs maps/campus maps/campus_r2 maps/campus_r3 \
        --out outputs/metrics/map

Inputs per map package (launch/g1_slam_mapping.launch.py with evaluate:=true, then scripts/build_map_package.py):
trajectory.csv, trajectory_truth.csv, evaluation.json, map.yaml, annotations.json, semantic_truth.json.
  1.1 for every package in --runs (default: the deployed one): ATE aligned at the start pose, ATE after a best-fit
      rigid 2-D alignment, relative error over 10 m of path (drift), heading error; mean and spread over the runs.
  1.2 on the deployed package (first argument), twice with the same code: with the person's annotations (as the
      robot uses the map) and with the simulator's zones in place of the person (annotation error removed).
      Doorway precision, recall, F1 and position error; room IoU of the annotation for reference.
Writes <out>/map_metrics.json, map_metrics.md, fig_trajectory.png, fig_doorways.png. Definitions: docs/METRICS.md.
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
from g1_factory import map_package as P  # noqa: E402
from g1_factory import metrics as X  # noqa: E402
from g1_factory import slam_map as S  # noqa: E402


def font(size, bold=False):
    path = "/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf" % ("-Bold" if bold else "")
    try:
        return ImageFont.truetype(path, size)
    except OSError:
        return ImageFont.load_default()


def read_csv(path: Path) -> np.ndarray:
    with path.open() as f:
        rows = list(csv.reader(f))
    return np.array(rows[1:], float)


def world_from_map(pkg: Path):
    W = P.world_from_map(pkg)
    return W, math.atan2(W[1, 0], W[0, 0])


# ------------------------------------------------------------------ 1.1 trajectory
def trajectory_metrics(pkg: Path) -> dict:
    ev = json.loads((pkg/"evaluation.json").read_text())
    out = {"package": str(pkg), "evaluation_json": ev.get("trajectory"), "map_vs_truth": ev.get("map")}
    if not (pkg/"trajectory_truth.csv").is_file():
        out["note"] = "trajectory_truth.csv missing (map made before it was saved): start-aligned numbers only"
        return out
    est, truth = read_csv(pkg/"trajectory.csv"), read_csv(pkg/"trajectory_truth.csv")
    W, wyaw = world_from_map(pkg)
    est_world = (np.c_[est[:, 1:3], np.ones(len(est))]@W.T)[:, :2]
    heading = np.degrees(np.abs(X.wrap(est[:, 4]+wyaw-truth[:, 3])))
    path = float(np.linalg.norm(np.diff(truth[:, 1:3], axis=0), axis=1).sum())
    out.update(samples=len(est), path_m=round(path, 1), duration_s=round(float(est[-1, 0]-est[0, 0]), 1),
               ate_start_aligned_m=X.ate(est_world, truth[:, 1:3]),
               ate_best_fit_m=X.ate(est[:, 1:3], truth[:, 1:3], align=True),
               rpe_10m=X.rpe(est[:, [1, 2, 4]], truth[:, 1:4], 10.0),
               heading_error_deg=X.stats(heading))
    out["end_point_drift_pct"] = round(100*out["ate_start_aligned_m"]["final"]/max(path, 1e-6), 3)
    return out


def summarise_runs(runs: list) -> dict:
    full = [r for r in runs if "ate_start_aligned_m" in r]
    pick = lambda f: [f(r) for r in full]
    return {"runs": len(runs), "runs_with_truth_trajectory": len(full),
            "ate_start_aligned_rmse_m": X.stats(pick(lambda r: r["ate_start_aligned_m"]["rmse"])),
            "ate_best_fit_rmse_m": X.stats(pick(lambda r: r["ate_best_fit_m"]["rmse"])),
            "ate_start_aligned_max_m": X.stats(pick(lambda r: r["ate_start_aligned_m"]["max"])),
            "rpe_10m_position_pct": X.stats(pick(lambda r: r["rpe_10m"]["position_pct"].get("mean"))),
            "rpe_10m_heading_deg_per_m": X.stats(pick(lambda r: r["rpe_10m"]["heading_deg_per_m"].get("mean"))),
            "path_m": X.stats(pick(lambda r: r["path_m"]))}


# ------------------------------------------------------------------ 1.2 doorways
def truth_annotations(truth: dict) -> dict:
    """The simulator's zones and building corridors as if a person had drawn them exactly."""
    zones = [{"id": z["id"], "label": z["label"], "kind": "room", "rects": [z["bounds"]]} for z in truth["zones"]]
    labels = {p["id"]: p.get("label", p["id"]) for p in truth["places"]}
    zones += [{"id": pid, "label": labels.get(pid, pid), "kind": "corridor", "rects": [b]}
              for pid, b in truth.get("aisle_segments", {}).items()]
    return {"zones": zones, "anchors": [{"label": a["label"], "xy": a["xy"]} for a in truth.get("anchors", [])]}


def doorway_eval(grid, origin, res, annotations: dict, truth: dict) -> dict:
    ann = P.load_annotations(annotations)
    labels, places = P.label_free_space(grid, origin, res, ann["zones"])
    sem = P.build_semantic(grid, origin, res, ann)
    place_map = X.match_places(labels, places, origin, res, X.truth_place_fn(truth))
    out = X.doorway_metrics(sem["edges"], truth["edges"], place_map)
    out.update(places=len(sem["places"]), unnamed_areas=sum(p["kind"] == "area" for p in sem["places"]),
               place_map=place_map, room_iou=X.room_iou([z for z in ann["zones"] if z["kind"] == "room"], truth["zones"]))
    out["_semantic"] = sem
    return out


# ------------------------------------------------------------------ figures
def base_image(grid, scale):
    rgb = np.full(grid.shape+(3,), (214, 217, 222), np.uint8)
    rgb[grid == S.FREE] = (255, 255, 255)
    rgb[grid == S.OCCUPIED] = (110, 114, 122)
    return Image.fromarray(np.flipud(rgb)).resize((grid.shape[1]*scale, grid.shape[0]*scale), Image.NEAREST)


def titled(img, title, sub):
    out = Image.new("RGB", (img.width, img.height+70), (255, 255, 255))
    out.paste(img, (0, 70))
    d = ImageDraw.Draw(out)
    d.text((12, 8), title, fill=(20, 20, 20), font=font(24, True))
    d.text((12, 42), sub, fill=(80, 80, 80), font=font(16))
    return out


def trajectory_figure(pkg: Path, grid, origin, res, out_path: Path, scale=2):
    if not (pkg/"trajectory_truth.csv").is_file():
        return False
    est, truth = read_csv(pkg/"trajectory.csv"), read_csv(pkg/"trajectory_truth.csv")
    W, _ = world_from_map(pkg)
    truth_map = (np.c_[truth[:, 1:3], np.ones(len(truth))]@np.linalg.inv(W).T)[:, :2]
    err = np.linalg.norm(est[:, 1:3]-truth_map, axis=1)
    img = base_image(grid, scale)
    d = ImageDraw.Draw(img)
    h = grid.shape[0]
    px = lambda x, y: ((x-origin[0])/res*scale, (h-(y-origin[1])/res)*scale)
    d.line([px(*p) for p in truth_map[::4]], fill=(60, 110, 200), width=5)
    top = max(0.3, float(np.percentile(err, 99)))
    for p, e in zip(est[::4, 1:3], err[::4]):
        t = min(e/top, 1.0)
        x, y = px(*p)
        c = (int(40+215*t), int(170-120*t), 60)
        d.ellipse([x-2, y-2, x+2, y+2], fill=c)
    titled(img, f"SLAM trajectory vs truth: {pkg.name}",
           f"blue: true LiDAR path; dots: SLAM, green 0 m .. red {top:.2f} m error (map frame, start-aligned)").save(out_path)
    return True


def doorway_figure(grid, origin, res, sem, truth, result, out_path: Path, scale=2):
    img = base_image(grid, scale)
    d = ImageDraw.Draw(img, "RGBA")
    h = grid.shape[0]
    px = lambda x, y: ((x-origin[0])/res*scale, (h-(y-origin[1])/res)*scale)
    for z in sem["zones"]:
        b = z["bounds"]
        d.rectangle([px(b[0], b[3]), px(b[2], b[1])], outline=(90, 140, 220, 160), width=2)
    for z in truth["zones"]:
        b = z["bounds"]
        d.rectangle([px(b[0], b[3]), px(b[2], b[1])], outline=(40, 160, 70, 160), width=2)
    truth_door = {frozenset((e["a"], e["b"])): e["door"] for e in truth["edges"]}
    pred_door = {frozenset((e["a"], e["b"])): e["door"] for e in sem["edges"]}
    for det in result["detected"]:
        a, b = px(*truth_door[frozenset(det["truth"])]), px(*pred_door[frozenset(det["pred"])])
        d.line([a, b], fill=(30, 30, 30, 220), width=2)
        d.ellipse([b[0]-7, b[1]-7, b[0]+7, b[1]+7], fill=(60, 110, 200), outline=(255, 255, 255), width=2)
        d.text((b[0]+9, b[1]), f"{det['error_m']:.2f}", fill=(20, 20, 20), anchor="lm", font=font(14, True),
               stroke_width=3, stroke_fill=(255, 255, 255))
    for key in truth_door:
        x, y = px(*truth_door[key])
        d.rectangle([x-6, y-6, x+6, y+6], fill=(40, 160, 70))
    for key in result["missed"]:
        x, y = px(*truth_door[frozenset(key)])
        d.line([(x-10, y-10), (x+10, y+10)], fill=(210, 40, 40), width=4)
        d.line([(x-10, y+10), (x+10, y-10)], fill=(210, 40, 40), width=4)
    for pair in result["false"]:
        x, y = px(*pred_door[frozenset(pair)])
        d.ellipse([x-7, y-7, x+7, y+7], fill=(210, 40, 40), outline=(255, 255, 255), width=2)
    titled(img, f"Doorways: precision {result['precision']}, recall {result['recall']}, F1 {result['f1']}",
           "green squares/boxes: truth doorways and zones; blue dots/boxes: automatic doorways (number = error m) and "
           "annotated rooms; red: missed (x) or false (dot)").save(out_path)


# ------------------------------------------------------------------ report table
def fmt(v, d=3):
    return "-" if v is None else f"{v:.{d}f}"


def markdown(result: dict) -> str:
    t, dep = result["trajectory_runs"], result["trajectory_deployed"]
    lines = ["# 지도 정확도", "", f"배포한 지도 묶음: `{result['deployed']}`, 매핑 실행 {t['runs']}회 "
             f"(정답 궤적이 있는 실행 {t['runs_with_truth_trajectory']}회)", "",
             "## 1.1 SLAM 궤적 오차", "",
             "| 지표 | 평균 | 표준편차 | 최소 | 최대 |", "|---|---:|---:|---:|---:|"]
    for key, name in (("ate_start_aligned_rmse_m", "ATE RMSE, 시작 자세 정렬 (m)"),
                      ("ate_best_fit_rmse_m", "ATE RMSE, 최적 정렬 (m)"),
                      ("ate_start_aligned_max_m", "최대 오차, 시작 자세 정렬 (m)"),
                      ("rpe_10m_position_pct", "10 m 구간 상대 위치 오차 (%)"),
                      ("rpe_10m_heading_deg_per_m", "10 m 구간 방향 오차 (도/m)"),
                      ("path_m", "매핑 경로 길이 (m)")):
        s = t[key]
        lines.append(f"| {name} | {fmt(s.get('mean'))} | {fmt(s.get('std'))} | {fmt(s.get('min'))} | {fmt(s.get('max'))} |")
    if "ate_start_aligned_m" not in dep:
        old = dep.get("evaluation_json") or {}
        lines += ["", "배포한 지도는 trajectory_truth.csv가 없는 지도라(이 기능 전에 만든 지도) 최적 정렬과 구간 오차를 계산하지 못했습니다. "
                  f"evaluation.json의 시작 자세 정렬 값: ATE RMSE {fmt(old.get('ate_rmse_m'))} m, 최대 {fmt(old.get('max_m'))} m, "
                  f"끝점 {fmt(old.get('final_m'))} m, 경로 {old.get('path_m', '-')} m."]
    if "ate_start_aligned_m" in dep:
        a = dep["ate_start_aligned_m"]
        lines += ["", f"배포한 지도: ATE RMSE {fmt(a['rmse'])} m(시작 자세 정렬), {fmt(dep['ate_best_fit_m']['rmse'])} m(최적 정렬), "
                  f"최대 {fmt(a['max'])} m, 끝점 {fmt(a['final'])} m({fmt(dep['end_point_drift_pct'], 2)} %), 경로 {dep['path_m']} m."]
    lines += ["", "## 1.2 토폴로지 출입구 검출과 위치 오차", "",
              "| 조건 | 정답 출입구 | 검출 | 오검출 | 미검출 | 정밀도 | 재현율 (95% 구간) | F1 | 위치 오차 평균 / 중앙값 / 최대 (m) | 벽의 출입구 평균 (m) | 벽 없는 경계 평균 (m) |",
              "|---|---:|---:|---:|---:|---:|---:|---:|---|---:|---:|"]
    for key, name in (("annotated", "사람 주석 (실제 사용)"), ("truth_zones", "정답 구역 (주석 오차 제거)")):
        r = result["doorways"][key]
        e = r["error_m"]
        lines.append(f"| {name} | {r['truth']} | {r['tp']} | {r['fp']} | {r['fn']} | {fmt(r['precision'])} | "
                     f"{fmt(r['recall'])} ({fmt(r['recall_ci95'][0], 2)}-{fmt(r['recall_ci95'][1], 2)}) | {fmt(r['f1'])} | "
                     f"{fmt(e.get('mean'), 2)} / {fmt(e.get('median'), 2)} / {fmt(e.get('max'), 2)} | "
                     f"{fmt(r['error_m_door'].get('mean'), 2)} | {fmt(r['error_m_open'].get('mean'), 2)} |")
    iou = result["doorways"]["annotated"]["room_iou"]
    lines += ["", f"참고: 사람 주석의 방 IoU 평균 {fmt(iou['mean'])} "
              f"({', '.join(f'{k} {v:.2f}' for k, v in iou['per_zone'].items())})."]
    return "\n".join(lines)+"\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("package", type=Path, help="the deployed map package (used for VLN and localisation)")
    parser.add_argument("--runs", type=Path, nargs="*", help="every mapping run for 1.1 (default: the package)")
    parser.add_argument("--out", type=Path, default=ROOT/"outputs/metrics/map")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    pkg = args.package
    grid, origin, res = S.read_map_yaml(pkg/"map.yaml")
    truth = json.loads((pkg/"semantic_truth.json").read_text())
    runs = [trajectory_metrics(r) for r in (args.runs or [pkg])]
    deployed = trajectory_metrics(pkg)
    annotated = doorway_eval(grid, origin, res, json.loads((pkg/"annotations.json").read_text()), truth)
    ideal = doorway_eval(grid, origin, res, truth_annotations(truth), truth)
    trajectory_figure(pkg, grid, origin, res, args.out/"fig_trajectory.png")
    doorway_figure(grid, origin, res, annotated.pop("_semantic"), truth, annotated, args.out/"fig_doorways.png")
    ideal.pop("_semantic")
    result = {"deployed": str(pkg), "trajectory_runs": summarise_runs(runs), "trajectory_per_run": runs,
              "trajectory_deployed": deployed, "doorways": {"annotated": annotated, "truth_zones": ideal}}
    (args.out/"map_metrics.json").write_text(json.dumps(result, indent=1, ensure_ascii=False))
    (args.out/"map_metrics.md").write_text(markdown(result), encoding="utf-8")
    print(markdown(result))


if __name__ == "__main__":
    main()
