#!/usr/bin/env python3
"""Gather the SLAM map and the VLN runs into two folders laid out for a presentation.

    bash scripts/collect_results.sh ~/Workspace/G1-isaacsim-slam ~/Workspace/G1-result

SLAM folder (from the map package, default maps/campus):
  README.md            what the robot did, accuracy against the simulator, figures
  figures/             slam_map.png (3-D cloud, occupancy grid, comparison), topology.png (annotated map)
  01_slam_map/         map.yaml, map.pgm, map.png, cloud.pcd, trajectory.csv, meta.json
  02_check_vs_truth/   evaluation.json, comparison.png, map_world.yaml/.pgm (simulation only)
  03_annotation/       annotations.json, semantic_map.json, topology.png, annotation_check.json (when annotated)
VLN folder (every run under outputs/ with summary.json and feedback.jsonl):
  README.md            one table of all runs, then each run with its route and what the VLM saw
  experiments/<name>/  results.md, episodes.csv, experiment.json of experiment matrices
  runs/<NN>_<name>/    summary.json, instruction.txt, route.png, frames.jpg, inspection.jpg, decisions/
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import re
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/"scripts"))


def copy(src: Path, dst_dir: Path, name=None):
    if src.is_file():
        dst_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst_dir/(name or src.name))
        return True
    return False


def pct(v):
    return "-" if v is None else f"{100*v:.0f}%"


# ------------------------------------------------------------------ SLAM
def collect_slam(pkg: Path, out: Path) -> dict:
    meta = json.loads((pkg/"meta.json").read_text())
    for name in ("map.yaml", "map.pgm", "map.png", "cloud.pcd", "trajectory.csv", "meta.json"):
        copy(pkg/name, out/"01_slam_map")
    for name in ("evaluation.json", "comparison.png", "map_world.yaml", "map_world.pgm"):
        copy(pkg/name, out/"02_check_vs_truth")
    annotated = (pkg/"annotations.json").is_file()
    for name in ("annotations.json", "semantic_map.json", "topology.png", "annotation_check.json"):
        copy(pkg/name, out/"03_annotation")
    figures = out/"figures"
    figures.mkdir(parents=True, exist_ok=True)
    import plot_slam_map
    sys.argv = ["plot_slam_map.py", str(pkg), "--out", str(figures)]
    plot_slam_map.main()
    copy(pkg/"topology.png", figures)

    traj, mp = meta.get("trajectory", {}), meta.get("map", {})
    rows = [("걸은 거리와 시간", f"{traj.get('path_m', '-')} m, 시뮬레이션 {meta.get('duration_s', 0)/60:.1f}분, 스캔 {meta.get('scans', '-')}개"),
            ("지도 크기", f"{meta['size_cells'][0]}x{meta['size_cells'][1]} 칸 ({meta['resolution_m']} m), 3D 점 {meta.get('cloud_points', 0):,}개"),
            ("수평 보정, 벽 정렬", ", ".join(f"{meta[k]}도" if k in meta else "-"
                                         for k in ("floor_levelling_deg", "wall_alignment_deg")))]
    if traj:
        rows += [("위치 오차 (평균 제곱근 / 최대 / 출발점 복귀)", f"{traj['ate_rmse_m']} m / {traj['max_m']} m / {traj['final_m']} m")]
    if mp:
        rows += [("장애물 정밀도 / 재현율 (허용 오차 0.2 m)", f"{pct(mp['obstacle_precision'])} / {pct(mp['obstacle_recall'])}"),
                 ("실제 빈 공간 중 빈 공간 / 장애물 / 미확인", f"{pct(mp['free_marked_free'])} / {pct(mp['free_marked_occupied'])} / {pct(mp['free_unknown'])}")]
    check = json.loads((pkg/"annotation_check.json").read_text()) if (pkg/"annotation_check.json").is_file() else None
    if check:
        rows += [("사람이 그린 방과 정답 구역의 겹침 (IoU 평균)", str(check.get("room_iou_mean"))),
                 ("자동 출입구", f"{check.get('edges')}개, 방 연결 {check.get('rooms_with_doorway')}")]
    lines = ["# G1 SLAM 지도", "",
             f"G1이 캠퍼스를 한 바퀴 걸으면서 머리의 LiDAR와 IMU만으로 만든 지도입니다(FAST-LIO2, 지도 묶음 `{pkg.name}`).",
             "", "| 항목 | 결과 |", "|---|---|"] + [f"| {k} | {v} |" for k, v in rows] + [
             "", "![3D 점군, 점유 지도, 정답과 비교](figures/slam_map.png)", ""]
    if annotated and (figures/"topology.png").is_file():
        lines += ["## 사람 주석과 자동 토폴로지", "",
                  "사람이 지도 위에 방과 통로를 그리고 이름을 붙였고, 방 사이의 연결과 출입구는 자동으로 만들어졌습니다.", "",
                  "![주석과 장소 그래프](figures/topology.png)", ""]
    else:
        lines += ["## 다음 단계", "", "주석 도구(`tools/map_annotator.html`)로 방과 통로를 그린 뒤 "
                  "`scripts/build_map_package.py`로 지도 묶음을 완성하면 여기에 주석 그림이 추가됩니다.", ""]
    lines += ["## 폴더", "", "- `01_slam_map`: 점유 지도(ROS map_server 형식), 3D 점군, 경로, 설정",
              "- `02_check_vs_truth`: 시뮬레이터 정답과의 비교(시뮬레이션에서만)",
              "- `03_annotation`: 사람 주석과 자동 토폴로지(주석을 그린 뒤)", ""]
    (out/"README.md").write_text("\n".join(lines), encoding="utf-8")
    return {"slam": rows}


# ------------------------------------------------------------------ VLN
def find_runs(root: Path):
    runs = []
    for summary in sorted(root.rglob("summary.json")):
        folder = summary.parent
        if (folder/"feedback.jsonl").is_file() and "maps" not in folder.parts:
            runs.append(folder)
    return runs


def run_info(folder: Path, root: Path) -> dict:
    s = json.loads((folder/"summary.json").read_text(encoding="utf-8"))
    args = json.loads((folder/"launch_args.json").read_text()) if (folder/"launch_args.json").is_file() else {}
    rel = folder.relative_to(root)
    kind = "console" if rel.parts[0] == "console" else ("experiment" if re.match(r"seed\d+$", folder.name) else "single")
    if kind == "experiment":
        name = "_".join(rel.parts)
    else:
        name = folder.name
    if args.get("map_package"):
        map_kind = "로봇 SLAM 지도 + 사람 주석" + (" (지도 좌표)" if args.get("package_frame") == "map" else "")
    elif args.get("map_file"):
        map_kind = "로봇 SLAM 지도"
    else:
        map_kind = "시뮬레이터 지도"
    sim = {"true": "MuJoCo (개발)", "false": "Isaac Sim"}.get(str(args.get("sim", "")).lower(), "Isaac Sim" if kind == "experiment" else "-")
    result = s.get("result") or {}
    return {"folder": folder, "name": re.sub(r"[^\w.-]+", "_", name)[:80], "kind": kind,
            "instruction": s.get("instruction", ""), "success": s.get("success"), "sim": sim, "map": map_kind,
            "model": args.get("vlm_model", "-"), "decisions": s.get("vln_decisions", s.get("decisions")),
            "distance": s.get("final_distance_to_target_m"),
            "verdict": s.get("verdict") or result.get("verdict") or (s.get("inspection") or {}).get("verdict"),
            "verdict_correct": s.get("verdict_correct"), "time": s.get("task_sim_time_s"),
            "answer": result.get("answer") or (s.get("inspection") or {}).get("answer", "")}


def frame_picks(folder: Path, decisions):
    files = sorted((folder/"decisions").glob("*.jpg"))
    picks = []
    for k, d in enumerate(decisions[:len(files)]):
        if k == 0 or "looked around" in (d.get("note") or "") or d.get("action") == "approach":
            picks.append(k)
    picks = sorted(set(picks))[-4:]
    if (folder/"inspection.jpg").is_file():
        picks.append("inspection")
    return picks


def collect_vln(root: Path, out: Path) -> dict:
    import plot_vln_episode as V
    runs = [run_info(f, root) for f in find_runs(root)]
    runs.sort(key=lambda r: (r["kind"] != "experiment", str(r["folder"])))
    for exp in sorted({p.parent for p in root.rglob("results.md")}):
        for name in ("results.md", "episodes.csv", "experiment.json"):
            copy(exp/name, out/"experiments"/exp.name)
    table = ["| # | 지시 | 시뮬레이터 | 지도 | VLM | 결과 | 결정 | 대상까지 | 판정 | 시뮬레이션 시간 |",
             "|---:|---|---|---|---|---|---:|---:|---|---:|"]
    sections = []
    for k, r in enumerate(runs, 1):
        dst = out/"runs"/f"{k:02d}_{r['name']}"
        dst.mkdir(parents=True, exist_ok=True)
        folder = r["folder"]
        for name in ("summary.json", "inspection.jpg", "launch_args.json", "trajectory.csv", "feedback.jsonl"):
            copy(folder/name, dst)
        (dst/"instruction.txt").write_text(r["instruction"]+"\n", encoding="utf-8")
        if (folder/"decisions").is_dir():
            shutil.copytree(folder/"decisions", dst/"decisions", dirs_exist_ok=True)
        figures = []
        try:
            summary, decisions = V.load_episode(folder)
            V.route_figure(folder, dst/"route.png")
            figures.append(("route.png", "결정 지점과 경로"))
            picks = frame_picks(folder, decisions)
            if picks:
                V.frame_strip(folder, picks, decisions).save(dst/"frames.jpg", quality=88)
                figures.append(("frames.jpg", "VLM이 본 영상"))
        except Exception as exc:  # a run without decisions or a broken file still gets its row
            print(f"  {r['name']}: figures skipped ({exc})")
        result = {True: "성공", False: "실패"}.get(r["success"], "-")
        verdict = r["verdict"] or "-"
        if r["verdict_correct"] is not None:
            verdict += " (정답)" if r["verdict_correct"] else " (오답)"
        dist = f"{r['distance']:.2f} m" if isinstance(r["distance"], (int, float)) else "-"
        time = f"{r['time']:.0f} s" if isinstance(r["time"], (int, float)) else "-"
        table.append(f"| {k} | {r['instruction'][:60]} | {r['sim']} | {r['map']} | {r['model']} | {result} | "
                     f"{r['decisions'] if r['decisions'] is not None else '-'} | {dist} | {verdict} | {time} |")
        sec = [f"### {k}. {r['instruction']}", "", f"{r['sim']}, {r['map']}, {r['model']}. 결과: {result}, 판정 {verdict}, "
               f"대상까지 {dist}. 답변: {r['answer'] or '-'}", ""]
        sec += [f"![{cap}](runs/{dst.name}/{fig})" for fig, cap in figures]
        sections += sec+[""]
    n_ok = sum(1 for r in runs if r["success"])
    n_scored = sum(1 for r in runs if r["success"] is not None)
    lines = ["# G1 VLN 결과", "",
             f"실행 {len(runs)}개 (성공 {n_ok}/{n_scored}, 콘솔 실행은 정답이 없어 성공 여부를 매기지 않음). "
             "지도의 숫자는 VLN이 결정을 내린 지점과 순서, 사진은 그 결정에서 VLM이 본 영상입니다.", ""]
    if (out/"experiments").is_dir():
        lines += ["실험 행렬의 집계표는 `experiments/<이름>/results.md`에 있습니다.", ""]
    lines += table+["", "## 실행별", ""]+sections
    (out/"README.md").write_text("\n".join(lines), encoding="utf-8")
    return {"runs": len(runs), "success": n_ok, "scored": n_scored}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--map", type=Path, default=ROOT/"maps/campus", help="map package folder")
    parser.add_argument("--runs", type=Path, default=ROOT/"outputs", help="folder searched for VLN runs")
    parser.add_argument("--slam-out", type=Path, required=True)
    parser.add_argument("--vln-out", type=Path, required=True)
    args = parser.parse_args()
    report = {}
    if (args.map/"meta.json").is_file():
        report.update(collect_slam(args.map, args.slam_out))
        print(f"SLAM: {args.slam_out}/README.md")
        for k, v in report["slam"]:
            print(f"  {k}: {v}")
    else:
        print(f"no map package at {args.map} (meta.json missing); SLAM folder skipped")
    vln = collect_vln(args.runs, args.vln_out)
    print(f"VLN: {args.vln_out}/README.md  ({vln['runs']} runs, {vln['success']}/{vln['scored']} successful)")


if __name__ == "__main__":
    main()
