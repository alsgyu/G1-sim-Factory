# 접근 정책 비교와 기록

`approach_eval`은 시뮬레이션 비교를 위한 명시적 선택 항목이다. 기본값은
`disabled`이며 기존 접근 정책과 기록 동작을 유지한다.

| 모드 | 접근 보류 | 이격 기준 |
|---|---|---|
| depth | 없음 | 영상 근거리 깊이 근사점 |
| gate | 셀 교차길이 | 영상 근거리 깊이 근사점 |
| edge | 없음 | 지도 경계 우선 |
| full | 셀 교차길이 | 지도 경계 우선 |
| legacy | 5cm 샘플링 | 지도 경계 우선 |

모든 모드에 기존 목표점 보정, 경로계획과 보행기를 남긴다. 주행 비교에서는
모델, 프롬프트, 지도, USD, 초기 자세와 실행 예산을 동일하게 유지한다.

```bash
python3 scripts/hvln_experiment.py --runner docker --simulator isaac \
  --conditions vln_8b --tasks inspect_meeting_table_en --seeds 0 \
  --skip-measurement --out outputs/approach_pilot/full \
  --set approach_eval=full map_file=/ws/path/to/validated_map.yaml
```

지도 경로는 실제 검증한 지도로 바꾼다. 점유도에서 FREE 셀이 없거나 로봇
여유를 만족하는 웨이포인트가 없으면 항법이 실패한다. UNKNOWN을 FREE로
재분류해 통과시키지 않는다. 시뮬레이터 형상으로 만든 지도를 쓰면
`GT_geometry_control`로 기록하고 SLAM 성능과 구분한다.

각 에피소드의 `approaches/`에 동일 관측의 RGB-D, TF, 카메라 파라미터와
검출 박스 NPZ를 저장한다. `events.jsonl`에는 후보별 보류·계획점·보정점과
선택 모드에서의 `hold`, `path_sent`, `already_close`, `no_depth`,
`outside_region`, `no_free_pose`, `unreachable` 분기를 기록한다.
다른 후보의 함수 결과는 그 모드를 실제 주행한 결과가 아니다.

실제 주행에서는 각 모드를 별도 실행해야 한다. 경계·격자 조건의 반복과
동일 에피소드의 프레임을 독립 주행 표본으로 세지 않는다. 계획점,
보정점, 도착 직후 관측, 충분히 안정된 실제 정지 자세도 구분한다.
기존 `success`는 독립적인 대상 ID·점검 시야 검증을 대체하지 않는다.
Isaac의 `footprint_proxy`도 실제 물리 접촉 측정과 구분한다.

```bash
python3 -m pytest -q tests/test_vln_navigator.py \
  tests/test_partition_geometry.py tests/test_empty_roadmap.py \
  tests/test_approach_validation.py tests/test_approach_audit.py
```

`test_approach_audit.py`는 실제 `act()`를 추출해 ROS 입출력 경계만 대역으로
실행하는 검사다. ROS/DDS 또는 Isaac 통합 시험으로 해석하지 않는다.
