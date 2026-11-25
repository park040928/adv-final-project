% TEST_PLANNER  B 모듈(맵 생성 + frontier 탐색 + 경로 계획) 단독 테스트
%   update_robot 없이 get_env 벽으로 가상 레이저를 쏴서 맵을 만들고,
%   로봇을 waypoint로 순간이동시키며 탐색이 끝까지 되는지 확인한다.
%
%   담당: B
close all; clear; clc;
addpath('mapping', 'planning');

[walls, target, mapSize] = get_env();
res      = 0.1;
maxRange = 4;                          % 가상 센서 최대 거리 [m]
angles   = linspace(-pi, pi, 181);     % 가상 센서 빔 각도
angles(end) = [];

map   = init_map(mapSize, res);
robot = [1.5 1.5 0];                   % 시작 위치 (외벽 안쪽)
opts  = struct('inflate', 0.3, 'minDist', 0.5);
traj  = robot(1:2);

figure('Name', 'B module test');
for it = 1:200
    ranges = fake_scan(robot, angles, maxRange, walls);
    map = update_map(map, robot, angles, ranges, maxRange);

    [wp, goal, info] = plan_path(map, robot, [], opts);

    % ---- 시각화 ----
    clf; hold on; axis equal; axis([0 mapSize 0 mapSize]);
    S = info.S;
    img = 0.5 * ones(size(S)); img(S == 0) = 1; img(S == 1) = 0;
    imagesc([res/2 mapSize-res/2], [res/2 mapSize-res/2], img);
    colormap(gray); caxis([0 1]); set(gca, 'YDir', 'normal');
    [fr, fc] = find(info.frontier);
    [fx, fy] = grid2world(map, fr, fc);
    plot(fx, fy, 'c.', 'MarkerSize', 4);
    plot(traj(:,1), traj(:,2), 'b-', 'LineWidth', 1.5);
    plot(robot(1), robot(2), 'bo', 'MarkerFaceColor', 'b');
    plot(target(:,1), target(:,2), 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    if ~isempty(wp)
        plot([robot(1); wp(:,1)], [robot(2); wp(:,2)], 'g-o', 'LineWidth', 1.5);
        plot(goal(1), goal(2), 'mx', 'MarkerSize', 12, 'LineWidth', 2);
    end
    title(sprintf('iter %d  status: %s', it, info.status));
    drawnow;
    if it == 1, saveas(gcf, 'test_planner_start.png'); end

    if strcmp(info.status, 'done')
        fprintf('탐색 완료: %d 회\n', it);
        saveas(gcf, 'test_planner_result.png');
        break;
    end

    % 첫 waypoint까지 이동 (실제로는 A의 주행 제어가 담당)
    nxt = wp(1, :);
    robot = [nxt atan2(nxt(2) - robot(2), nxt(1) - robot(1))];
    traj(end+1, :) = nxt; %#ok<SAGROW>
end
