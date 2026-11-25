# Autonomous Driving Vehicle 2026 Fall — Final Project

50×50 맵에서 2WDD 로봇을 자율주행시켜 고정 타깃 3개의 위치 (x, y)를 찾는 프로젝트.

## 마감
- 코드 작성: 11/27 → 통합: 11/30
- 각자 맵으로 테스트: 12/2
- 보고서·PPT·스크립트 제출: 12/6 23:59
- 발표: 12/7 / 12/9 (영어, 10분 + Q&A 2분)

## 역할
| 담당 | 분야 | 세부 |
|---|---|---|
| A 김서연 | 주행 제어 및 장애물 회피 | 바퀴 속도 제어, scan 처리, 충돌 방지 |
| B 박은혜 | 맵 탐색 및 경로 계획 | occupancy grid, frontier 탐색, 경로 계획 |
| C 차소정 | 타깃 탐지 및 위치 추정 | image 처리, 삼각측량, 좌표 계산 |
| D 정아현 | 시스템 통합 및 시각화 | 메인 컨트롤러, 모듈 연결, 테스트, 그래프 |

## 구조
```
main.m              메인 (D)
get_env.m           테스트용 맵 (update_robot_open 사용)
update_robot*.p     제공 시뮬레이터
mapping/            B: occupancy grid (init_map, update_map, get_state ...)
planning/           B: frontier 탐색 + 경로 계획 (plan_path ...)
test_planner.m      B 모듈 단독 테스트 (.p 없이 가상 스캔)
```

## B 모듈 인터페이스
```matlab
addpath('mapping', 'planning');
map = init_map(50, 0.2);
% 매 스텝
map = update_map(map, robot, angles, ranges, maxRange);   % scan -> angles/ranges 변환 필요
[wp, goal, info] = plan_path(map, robot, [], opts);       % 탐색 모드
[wp, goal, info] = plan_path(map, robot, [gx gy], opts);  % 지정 목표 모드
% wp (Mx2) -> A 주행 제어로 전달, info.status == 'done' 이면 탐색 종료
```
