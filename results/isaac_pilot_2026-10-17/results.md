# Hierarchical VLN results

| 조건 | 에피소드 | 성공률 | grounding 정확도 | 항법 완료 | SPL | 최종 오차(m) | 충돌/넘어짐 | 판정 정확도 | 재계획 | VLM 지연(s) | 경로추종 RMSE(m) | 작업 시간(sim s) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| vln_8b | 9 | 56% | 89% | 100% | 0.28 | 2.81 | 0/0 | 67% | 0 | 4.28 | 0.022 | 189.5 |

## 작업별 성공률

| 작업 | vln_8b |
|---|---:|
| check_electrical_panel_en | 0/1 |
| go_fire_cabinet_ko | 0/1 |
| inspect_conveyor | 1/1 |
| inspect_dock_cart_ko | 1/1 |
| inspect_dock_en | 0/1 |
| inspect_meeting_table_en | 0/1 |
| inspect_servers_office | 1/1 |
| inspect_station_paraphrase | 1/1 |
| json_agent_subgoals | 1/1 |
