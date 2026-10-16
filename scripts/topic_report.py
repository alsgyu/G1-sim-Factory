#!/usr/bin/env python3
"""Turn a topic_monitor measurement (topic_stats.json) into Markdown tables.

    python3 scripts/topic_report.py outputs/hvln_measure/<task>/topic_stats.json > table.md

Sizes are serialized CDR bytes per message as carried by DDS (excluding RTPS/UDP
headers and fragmentation overhead). Rates are measured in simulation time.
"""
from __future__ import annotations

import json
import sys

PAYLOAD = {
    "/camera/camera/color/image_raw": "640×480 rgb8 (921,600 B 픽셀)",
    "/camera/camera/aligned_depth_to_color/image_raw": "640×480 16UC1 mm (614,400 B 픽셀)",
    "/camera/camera/color/camera_info": "K/D/R/P, plumb_bob",
    "/camera/camera/aligned_depth_to_color/camera_info": "color와 동일 내부 파라미터",
    "/livox/lidar": "20,000 ray/scan 중 반사점만, 26 B/점 (x,y,z,intensity,tag,line,timestamp)",
    "/livox/imu": "각속도+가속도, orientation 없음",
    "/map": "0.1 m 격자 780×340 = 265,200 셀 int8",
    "/odom": "pose+twist(+공분산 72 double)",
    "/joint_states": "다리 12관절 position/velocity/effort",
    "/tf": "odom→base_link 1개",
    "/tf_static": "map→odom, base_link→camera_link→optical, base_link→livox_frame",
    "/plan": "0.1 m 간격 PoseStamped 목록 (경로 길이에 비례)",
    "/vln/inspection_image": "JPEG q85 640×480",
    "/vln/instruction": "UTF-8 지시문 또는 JSON",
    "/vln/feedback": "JSON 진행/결과 이벤트",
    "/nav/status": "JSON (planned/following/reached/failed)",
    "/sim/status": "JSON (구역·충돌·넘어짐·RTF·처리시간)",
    "/clock": "sim time",
    "/cmd_vel": "vx, vy, yaw rate",
    "/goal_pose": "map 좌표 목표 자세",
    "/vln/subgoals": "하위목표 자세 목록",
}


# Nominal publish rates of periodic topics (launch defaults); everything else is event-driven.
NOMINAL_HZ = {"/clock": 100, "/camera/camera/color/image_raw": 30, "/camera/camera/color/camera_info": 30,
              "/camera/camera/aligned_depth_to_color/image_raw": 30,
              "/camera/camera/aligned_depth_to_color/camera_info": 30, "/livox/lidar": 10, "/livox/imu": 200,
              "/odom": 50, "/joint_states": 50, "/tf": 50, "/sim/status": 2, "/cmd_vel": 20}


def short_qos(q):
    if not q:
        return "-"
    rel = {"RELIABLE": "Reliable", "BEST_EFFORT": "BestEffort"}.get(q["reliability"], q["reliability"])
    dur = {"VOLATILE": "Volatile", "TRANSIENT_LOCAL": "TransientLocal"}.get(q["durability"], q["durability"])
    hist = f"KeepLast({q['depth']})" if q["history"] == "KEEP_LAST" else q["history"]
    return f"{rel}/{dur}/{hist}"


def human(n):
    if n is None:
        return "-"
    for unit, scale in (("MB", 1e6), ("KB", 1e3)):
        if n >= scale:
            return f"{n/scale:,.2f} {unit}"
    return f"{n:,.0f} B"


def main(path):
    stats = json.load(open(path))
    order = sorted(stats, key=lambda k: -(stats[k].get("mean_bytes") or 0)*NOMINAL_HZ.get(k, 0.001))
    print("| 토픽 | 메시지 타입 | 발행 노드 → 구독 노드 | 데이터 | 메시지 크기(평균, 최소–최대) | 주기(sim) | 대역폭(평균×주기) |")
    print("|---|---|---|---|---|---:|---:|")
    for name in order:
        s = stats[name]
        pubs = ", ".join(sorted({p["node"] for p in s.get("publishers", [])})) or "-"
        subs = ", ".join(sorted({p["node"] for p in s.get("subscribers", [])})) or "(없음)"
        rate = s.get("rate_hz_sim")
        mean = s.get("mean_bytes")
        size = f"{human(mean)} ({human(s['min_bytes'])}–{human(s['max_bytes'])})" if mean else "-"
        if name in NOMINAL_HZ and rate:
            rate_text, bw = f"{NOMINAL_HZ[name]} Hz (측정 {rate:.1f})", human(mean*NOMINAL_HZ[name])+"/s"
        else:
            rate_text, bw = f"이벤트 ({s['count']}회/에피소드)", "-"
        print(f"| `{name}` | `{s['type']}` | {pubs} → {subs} | {PAYLOAD.get(name, '')} | {size} | {rate_text} | {bw} |")
    print()
    print("| 토픽 | 발행 QoS (노드) | 구독 QoS (노드) |")
    print("|---|---|---|")
    for name in order:
        s = stats[name]
        pubs = "<br>".join(f"{short_qos(p['qos'])} ({p['node']})" for p in s.get("publishers", [])) or "-"
        subs = "<br>".join(f"{short_qos(p['qos'])} ({p['node']})" for p in s.get("subscribers", [])) or "-"
        print(f"| `{name}` | {pubs} | {subs} |")


if __name__ == "__main__":
    main(sys.argv[1])
