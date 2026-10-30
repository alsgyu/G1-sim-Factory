#!/usr/bin/env python3
"""3. VLN accuracy for the report, from the episodes of scripts/hvln_experiment.py (or any episode folders).

    bash scripts/g1_docker.sh python3 scripts/eval_vln.py outputs/metrics/vln --out outputs/metrics/vln

Reads every <dir>/**/summary.json written by the evaluator (g1_factory/ros2/external_agent_node.py).
3.1 target finding: the VLN found the target in the camera image and the position it reports (bounding box + depth,
    moved into the simulator frame) lies within 1.0 m of the target object's footprint (grounding_correct).
    Over all episodes (no target reported = not found), and among episodes that reported a target.
3.2 verdict: the answer to the check question in front of the target matches the scene (verdict_correct). Over all
    episodes (no answer = wrong), among answered episodes, and among episodes whose target was found.
Rates with Wilson 95 % intervals; per task and per condition; success, SPL, collisions and localisation error as
context. Writes <out>/vln_metrics.json, vln_metrics.md.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from g1_factory import metrics as X  # noqa: E402


def load(folders):
    episodes = []
    for folder in folders:
        for path in sorted(Path(folder).glob("**/summary.json")):
            if "measurement" in path.parts:
                continue
            s = json.loads(path.read_text(encoding="utf-8"))
            if "grounding_correct" not in s:
                continue
            launch = path.parent/"launch_args.json"
            args = json.loads(launch.read_text()) if launch.is_file() else {}
            s["_dir"] = str(path.parent)
            s["_package_frame"] = args.get("package_frame", "") if args.get("map_package") else ""
            s["_model"] = args.get("vlm_model", "")
            s["_condition"] = s.get("condition") or path.parts[-4]
            episodes.append(s)
    return episodes


def metrics(group) -> dict:
    n = len(group)
    reported = [s for s in group if s.get("target_xy")]
    found = [s for s in group if s.get("grounding_correct")]
    answered = [s for s in group if s.get("verdict_correct") is not None]
    correct = [s for s in group if s.get("verdict_correct") is True]
    loc = [s["localization"]["position_error_m"] for s in group if (s.get("localization") or {}).get("position_error_m")]
    return {
        "episodes": n,
        "target_found": X.rate(len(found), n),
        "target_reported": X.rate(len(reported), n),
        "target_found_among_reported": X.rate(len(found), len(reported)),
        "target_error_m": X.stats([s.get("target_localization_error_m") for s in reported]),
        "verdict_correct": X.rate(len(correct), n),
        "verdict_correct_among_answered": X.rate(len(correct), len(answered)),
        "verdict_correct_when_found": X.rate(sum(1 for s in found if s.get("verdict_correct") is True), len(found)),
        "success": X.rate(sum(1 for s in group if s.get("success")), n),
        "spl": X.stats([s.get("spl") for s in group]),
        "collisions": sum(1 for s in group if s.get("collision_events")),
        "falls": sum(1 for s in group if s.get("fallen")),
        "decisions": X.stats([s.get("vln_decisions") for s in group]),
        "vlm_latency_s": X.stats([s.get("vlm_latency_mean_s") for s in group]),
        "task_time_s": X.stats([s.get("task_sim_time_s") for s in group]),
        "localization_error_mean_m": X.stats([e.get("mean") for e in loc]),
        "localization_error_max_m": X.stats([e.get("max") for e in loc]),
    }


def fmt(v, d=3):
    return "-" if v is None else f"{v:.{d}f}"


def ci(r):
    if not r["n"]:
        return "-"
    return f"{r['rate']*100:.1f}% ({r['k']}/{r['n']}, 95% {r['ci95'][0]*100:.1f}-{r['ci95'][1]*100:.1f}%)"


def markdown(result) -> str:
    lines = ["# VLN 정확도", ""]
    for cond, m in result["conditions"].items():
        lines += [f"## 조건 `{cond}`: 에피소드 {m['episodes']}개 ({', '.join(result['frames'][cond]) or '-'})", "",
                  "| 지표 | 값 |", "|---|---|",
                  f"| 3.1 목표 대상을 찾은 정확도 (전체 에피소드 기준) | {ci(m['target_found'])} |",
                  f"| 대상 위치를 보고한 비율 | {ci(m['target_reported'])} |",
                  f"| 보고한 위치가 맞은 비율 | {ci(m['target_found_among_reported'])} |",
                  f"| 보고한 대상 위치 오차 평균 / 최대 (m) | {fmt(m['target_error_m'].get('mean'), 2)} / {fmt(m['target_error_m'].get('max'), 2)} |",
                  f"| 3.2 판정 정확도 (전체 에피소드 기준) | {ci(m['verdict_correct'])} |",
                  f"| 답한 에피소드 중 판정 정확도 | {ci(m['verdict_correct_among_answered'])} |",
                  f"| 대상을 맞게 찾은 에피소드 중 판정 정확도 | {ci(m['verdict_correct_when_found'])} |",
                  f"| (참고) 과제 성공률 | {ci(m['success'])} |",
                  f"| (참고) SPL 평균 | {fmt(m['spl'].get('mean'))} |",
                  f"| (참고) 충돌 / 넘어짐 에피소드 | {m['collisions']} / {m['falls']} |",
                  f"| (참고) 결정 수 평균, VLM 응답 평균 (s) | {fmt(m['decisions'].get('mean'), 1)}, {fmt(m['vlm_latency_s'].get('mean'), 1)} |",
                  f"| (참고) 에피소드별 위치 추정 오차 평균의 평균 / 최대의 최대 (m) | {fmt(m['localization_error_mean_m'].get('mean'))} / {fmt(m['localization_error_max_m'].get('max'))} |",
                  "", "| 과제 | 에피소드 | 대상 찾음 | 판정 정답 | 성공 |", "|---|---:|---:|---:|---:|"]
        for task, t in sorted(result["per_task"][cond].items()):
            lines.append(f"| {task} | {t['episodes']} | {t['target_found']['k']}/{t['episodes']} | "
                         f"{t['verdict_correct']['k']}/{t['episodes']} | {t['success']['k']}/{t['episodes']} |")
        lines.append("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folders", type=Path, nargs="+", help="experiment output folders or episode folders")
    parser.add_argument("--out", type=Path, help="default: the first folder")
    args = parser.parse_args()
    episodes = load(args.folders)
    if not episodes:
        raise SystemExit("no summary.json with VLN results found")
    by_cond = defaultdict(list)
    for s in episodes:
        by_cond[s["_condition"]].append(s)
    result = {"folders": [str(f) for f in args.folders], "conditions": {}, "per_task": {}, "frames": {},
              "episodes": [{k: s.get(k) for k in ("_dir", "_condition", "_package_frame", "task", "seed", "success",
                                                  "grounding_correct", "target_localization_error_m",
                                                  "verdict_correct", "state")} for s in episodes]}
    for cond, group in sorted(by_cond.items()):
        result["conditions"][cond] = metrics(group)
        result["frames"][cond] = sorted({s["_package_frame"] or "simulator map" for s in group}) + \
            sorted({s["_model"] for s in group if s["_model"]})
        tasks = defaultdict(list)
        for s in group:
            tasks[s["task"]].append(s)
        result["per_task"][cond] = {t: metrics(g) for t, g in tasks.items()}
    out = args.out or args.folders[0]
    out.mkdir(parents=True, exist_ok=True)
    (out/"vln_metrics.json").write_text(json.dumps(result, indent=1, ensure_ascii=False))
    (out/"vln_metrics.md").write_text(markdown(result), encoding="utf-8")
    print(markdown(result))


if __name__ == "__main__":
    main()
