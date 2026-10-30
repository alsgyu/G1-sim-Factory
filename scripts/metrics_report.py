#!/usr/bin/env python3
"""Final measurement report: the six metrics of docs/METRICS.md in one Markdown document with figures.

    bash scripts/g1_docker.sh python3 scripts/metrics_report.py --map outputs/metrics/map \
        --loc outputs/metrics/localization --vln outputs/metrics/vln --out outputs/metrics/report \
        --env "GPU=RTX 5080" "Isaac Sim=5.1.0" "Ollama=0.x"

Reads map_metrics.json (scripts/eval_map.py), localization_metrics.json (scripts/eval_localization.py) and
vln_metrics.json (scripts/eval_vln.py); any part may be missing. Writes <out>/REPORT.md, <out>/report.json (every
number) and <out>/figures/. The measurement procedure itself is docs/METRICS.md.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def read(path: Path):
    return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else None


def git_commit() -> str:
    try:
        out = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"], capture_output=True, text=True)
        if out.returncode == 0:
            return out.stdout.strip()
    except OSError:
        pass
    head = (ROOT/".git/HEAD").read_text().strip() if (ROOT/".git/HEAD").is_file() else ""
    if head.startswith("ref: "):
        ref = ROOT/".git"/head[5:]
        return ref.read_text().strip()[:7] if ref.is_file() else head
    return head[:7]


def fmt(v, d=3):
    return "-" if v is None else f"{v:.{d}f}"


def pct(r):
    if not r or not r.get("n"):
        return "-"
    return f"{r['rate']*100:.1f}% ({r['k']}/{r['n']}, 95% 구간 {r['ci95'][0]*100:.1f}-{r['ci95'][1]*100:.1f}%)"


def summary_rows(m, loc, vln):
    rows = []
    if m:
        t, dep = m["trajectory_runs"], m["trajectory_deployed"]
        if t["runs_with_truth_trajectory"]:
            fit, start = t["ate_best_fit_rmse_m"], t["ate_start_aligned_rmse_m"]
            spread = f" ± {fmt(fit.get('std'))}" if t["runs_with_truth_trajectory"] > 1 else ""
            rows.append(("1.1", "SLAM 궤적 오차 (ATE RMSE, 최적 정렬)",
                         f"{fmt(fit.get('mean'))} m{spread} (매핑 {t['runs_with_truth_trajectory']}회)",
                         f"시작 자세 정렬 {fmt(start.get('mean'))} m, 10 m 구간 {fmt(t['rpe_10m_position_pct'].get('mean'), 2)}%"))
        else:
            old = dep.get("evaluation_json") or {}
            rows.append(("1.1", "SLAM 궤적 오차 (ATE RMSE, 시작 자세 정렬)", f"{fmt(old.get('ate_rmse_m'))} m",
                         "정답 궤적 파일 없는 지도: 최적 정렬 값 없음"))
        a, i = m["doorways"]["annotated"], m["doorways"]["truth_zones"]
        rows.append(("1.2", "출입구 검출 (정밀도 / 재현율 / F1), 위치 오차 평균",
                     f"{fmt(a['precision'], 2)} / {fmt(a['recall'], 2)} / {fmt(a['f1'], 2)}, {fmt(a['error_m'].get('mean'), 2)} m "
                     f"(정답 {a['truth']}개)",
                     f"벽의 출입구 {fmt(a['error_m_door'].get('mean'), 2)} m, 벽 없는 경계 {fmt(a['error_m_open'].get('mean'), 2)} m; "
                     f"정답 구역 사용 시 F1 {fmt(i['f1'], 2)}"))
    if loc:
        e, f = loc["tracking"]["error_m"], loc["first_fix"]["in_window"]
        rows.append(("2.1", "LiDAR 위치 추정 오차 (평균 / 95% / 최대)",
                     f"{fmt(e.get('mean'))} / {fmt(e.get('p95'))} / {fmt(e.get('max'))} m "
                     f"({loc['tracking']['path_m']:.0f} m 주행)",
                     f"첫 위치 추정 {pct(f['correct'])}, 잘못 맞춤 {f['wrong']['k']}회; "
                     f"범위 밖 초기 자세 잘못 맞춤 {loc['first_fix']['out_of_window']['wrong']['k']}/{loc['first_fix']['out_of_window']['trials']}"))
        z = loc["zone"]
        rows.append(("2.2", "현재 구역 판정 정확도 (위치 + 구역 범위)", pct(z["accuracy_per_metre"]),
                     f"5 Hz 전체 표본 {fmt(z['accuracy']['rate']*100 if z['accuracy']['rate'] is not None else None, 1)}%"
                     + (f"; 위치 오차 없는 격자 {pct(loc['zone_grid']['accuracy'])}" if loc.get("zone_grid") else "")))
    if vln:
        for cond, c in vln["conditions"].items():
            rows.append(("3.1", f"목표 대상 찾기 정확도 ({cond})", pct(c["target_found"]),
                         f"위치를 보고한 에피소드 중 {pct(c['target_found_among_reported'])}"))
            rows.append(("3.2", f"확인 질문 판정 정확도 ({cond})", pct(c["verdict_correct"]),
                         f"답한 에피소드 중 {pct(c['verdict_correct_among_answered'])}"))
    return rows


def environment(args, m, loc, vln) -> list:
    lines = [f"- 보고서 작성: {dt.datetime.now().strftime('%Y-%m-%d %H:%M')}, 코드 커밋 `{git_commit()}`"]
    lines += [f"- {e}" for e in args.env]
    if m:
        pkg = Path(m["deployed"])
        meta = read(pkg/"meta.json") or read(ROOT/Path(*pkg.parts[2:])/"meta.json") or {}
        lines.append(f"- 지도 묶음 `{m['deployed']}`: {meta.get('slam', '-')}, 해상도 {meta.get('resolution_m', '-')} m, "
                     f"{meta.get('size_cells', ['-', '-'])[0]}x{meta.get('size_cells', ['-', '-'])[1]} 칸, 스캔 {meta.get('scans', '-')}개")
    if loc:
        r = loc["run"]
        lines.append(f"- 위치 추정 측정: 경로 {r.get('travelled_m', '-')} m, 표본 {r.get('samples', '-')}개, 첫 위치 추정 시험 {r.get('trials', '-')}회, "
                     f"정답 토픽 `{r.get('truth_topic', '-')}`")
    if vln:
        for cond, c in vln["conditions"].items():
            lines.append(f"- VLN 조건 `{cond}`: 에피소드 {c['episodes']}개, {', '.join(vln['frames'][cond])}")
    return lines


METHOD = """## 측정 방법 요약

자세한 절차와 명령은 `docs/METRICS.md`에 있습니다. 모든 정답은 시뮬레이터(Isaac Sim)의 장면 정의에서 나오며, 로봇은 정답을 받지 않습니다.

- 1.1 SLAM 궤적 오차: 매핑할 때 FAST-LIO2가 추정한 LiDAR 위치와 같은 시각의 정답 LiDAR 위치를 비교합니다. 시작 자세로 정렬한 값(실제 지도 좌표를 정하는 방식)과, 궤적 전체를 가장 잘 맞도록 회전하고 이동한 값(표준 ATE)을 함께 냅니다. 10 m 구간 상대 오차는 누적 표류를 나타냅니다.
- 1.2 출입구 검출: 지도 묶음의 자동 장소 그래프를 시뮬레이터 장소 그래프와 비교합니다. 장소는 이름이 아니라 빈 칸이 놓인 자리로 짝을 짓고, 짝지은 두 장소가 정답에서도 이어져 있으면 검출로 셉니다. 사람 주석을 그대로 쓴 결과(실제 사용)와 정답 구역을 넣은 결과(사람 오차 제거)를 함께 냅니다.
- 2.1 위치 추정: 로봇이 정해진 경로를 걷는 동안 localizer가 지도 묶음에서 찾은 위치를 같은 시각의 정답 위치와 비교합니다(측정 중 localizer는 로봇을 조종하지 않음). 경로를 따라 일정 거리마다, 일부러 틀린 초기 자세에서 새로 위치를 찾게 하여 맞음, 거절, 잘못 맞춤을 셉니다.
- 2.2 구역 판정: 로봇이 실행 중 쓰는 규칙(위치 추정 결과가 사람이 그린 어느 방 범위 안인가)으로 낸 구역을 정답 위치의 정답 구역과 비교합니다. 연속 표본은 서로 독립이 아니므로 95% 구간은 1 m마다 한 표본으로 계산합니다.
- 3.1 대상 찾기: VLN이 카메라 영상에서 찾아 보고한 대상 위치(상자 + 깊이, 지도 좌표를 시뮬레이터 좌표로 변환)가 정답 대상의 바닥 면적에서 1.0 m 안이면 맞음입니다. 위치를 보고하지 못한 에피소드는 틀림으로 셉니다.
- 3.2 판정: 대상 앞에서 확인 질문에 답한 yes/no가 장면의 정답과 같으면 맞음입니다. 답하지 못한 에피소드는 틀림으로 셉니다.
- 비율의 95% 구간은 Wilson 구간입니다.
"""


def limitations(m, loc, vln) -> list:
    out = ["- 시뮬레이션 결과입니다. 실제 G1과 현장에서는 센서 잡음, 사람과 물건의 이동, 조명이 다릅니다."]
    if m and m["trajectory_runs"]["runs_with_truth_trajectory"] < 3:
        out.append(f"- 정답 궤적이 있는 매핑이 {m['trajectory_runs']['runs_with_truth_trajectory']}회라 "
                   "SLAM 오차의 편차를 말하기에 부족합니다(3회 이상 권장).")
    if loc:
        out.append("- 위치 오차는 지도 묶음을 시뮬레이터 좌표로 옮기는 변환(매핑 시작 자세 기준)을 거치므로 지도 자체의 오차가 포함됩니다. "
                   "최적 정렬 후 값이 그 영향을 뺀 값입니다.")
    if vln:
        n = sum(c["episodes"] for c in vln["conditions"].values())
        out.append(f"- VLN 에피소드 {n}개의 비율이며 95% 구간이 넓습니다. 과제와 시작 자세(seed)에 따라 결과가 크게 다릅니다.")
    out.append("- 정답 출입구 점은 시뮬레이터의 장소 그래프 정의를 따르며, 벽 없는 구역의 경계에서는 자동 출입구(맞닿은 경계의 가운데)와 정의가 달라 1-2 m 차이가 납니다.")
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--map", type=Path)
    parser.add_argument("--loc", type=Path)
    parser.add_argument("--vln", type=Path)
    parser.add_argument("--out", type=Path, default=ROOT/"outputs/metrics/report")
    parser.add_argument("--env", nargs="*", default=[], help='"key=value" lines for the environment section')
    args = parser.parse_args()
    m = read(args.map/"map_metrics.json") if args.map else None
    loc = read(args.loc/"localization_metrics.json") if args.loc else None
    vln = read(args.vln/"vln_metrics.json") if args.vln else None
    figures = args.out/"figures"
    figures.mkdir(parents=True, exist_ok=True)
    copied = {}
    for folder, names in ((args.map, ("fig_trajectory.png", "fig_doorways.png")), (args.loc, ("fig_localization.png",))):
        for name in names:
            if folder and (folder/name).is_file():
                shutil.copy2(folder/name, figures/name)
                copied[name] = f"figures/{name}"
    lines = ["# G1 지도 기반 VLN 측정 보고서", "", "## 결과 요약", "",
             "| 번호 | 지표 | 결과 | 비고 |", "|---|---|---|---|"]
    lines += [f"| {a} | {b} | {c} | {d} |" for a, b, c, d in summary_rows(m, loc, vln)]
    lines += ["", "## 측정 환경", "", *environment(args, m, loc, vln), "", METHOD]
    for title, folder, md, figs in (("1. 지도 정확도", args.map, "map_metrics.md", ("fig_trajectory.png", "fig_doorways.png")),
                                    ("2. 위치 및 구역 판정", args.loc, "localization_metrics.md", ("fig_localization.png",)),
                                    ("3. VLN 정확도", args.vln, "vln_metrics.md", ())):
        if folder and (folder/md).is_file():
            body = (folder/md).read_text(encoding="utf-8").split("\n", 1)[1]  # drop the part's own title
            lines += [f"## {title}", body.replace("\n## ", "\n### ")]
            lines += [f"![{name}]({copied[name]})" for name in figs if name in copied]
            lines.append("")
    lines += ["## 한계와 주의", "", *limitations(m, loc, vln), ""]
    (args.out/"REPORT.md").write_text("\n".join(lines), encoding="utf-8")
    (args.out/"report.json").write_text(json.dumps({"map": m, "localization": loc, "vln": vln}, indent=1, ensure_ascii=False))
    print(f"report: {args.out/'REPORT.md'}")


if __name__ == "__main__":
    main()
