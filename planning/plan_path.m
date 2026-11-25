function [wp, goalXY, info] = plan_path(map, robot, goalXY, opts)
% PLAN_PATH  탐색/목표 경로 계획
%   [wp, goalXY, info] = plan_path(map, robot, [], opts)   % frontier 탐색 모드
%   [wp, goalXY, info] = plan_path(map, robot, [gx gy], opts) % 지정 목표 모드
%
%   robot : [x y theta]
%   opts.inflate   : 장애물 팽창 반경 [m]       (기본 0.35)
%   opts.minDist   : 너무 가까운 frontier 무시 [m] (기본 0.8)
%   opts.gainW     : 정보 이득 가중치             (기본 0.05)
%   opts.blacklist : Kx2 [x y] 이미 실패한 목표 (반경 1m 내 frontier 제외)
%
%   wp     : Mx2 waypoint -> A(주행 제어)에게 전달
%   goalXY : 실제로 선택된 목표 [x y] (없으면 [])
%   info.status : 'ok' | 'done'(남은 frontier 없음) | 'unreachable'
%
%   담당: B

if nargin < 4, opts = struct(); end
if ~isfield(opts, 'inflate'),   opts.inflate = 0.35; end
if ~isfield(opts, 'minDist'),   opts.minDist = 0.8;  end
if ~isfield(opts, 'gainW'),     opts.gainW   = 0.05; end
if ~isfield(opts, 'blacklist'), opts.blacklist = zeros(0, 2); end

S = get_state(map);
blocked = inflate_obstacles(S == 1, opts.inflate, map.res);
trav = (S == 0) & ~blocked;

[r0, c0] = world2grid(map, robot(1), robot(2));
trav(r0, c0) = true;                       % 시작 셀은 항상 허용
[dist, parent] = bfs_grid(trav, r0, c0);
distM = dist * map.res;

info = struct('status', 'ok', 'S', S, 'trav', trav, 'frontier', []);

if isempty(goalXY)
    % ---------- frontier 탐색 모드 ----------
    [F, gain] = find_frontiers(S, trav);
    info.frontier = F;
    cand = F & isfinite(dist) & (distM >= opts.minDist);
    if ~isempty(opts.blacklist)
        [cr, cc] = find(cand);
        [cx, cy] = grid2world(map, cr, cc);
        for b = 1:size(opts.blacklist, 1)
            bad = hypot(cx - opts.blacklist(b,1), cy - opts.blacklist(b,2)) < 1.0;
            cand(sub2ind(size(cand), cr(bad), cc(bad))) = false;
        end
    end
    if ~any(cand(:))
        wp = zeros(0, 2); goalXY = [];
        info.status = 'done';
        return;
    end
    cost = distM - opts.gainW * gain;      % 가깝고 미탐색 많은 곳 우선
    cost(~cand) = inf;
    [~, g] = min(cost(:));
else
    % ---------- 지정 목표 모드 ----------
    [rg, cg] = world2grid(map, goalXY(1), goalXY(2));
    g = sub2ind(size(dist), rg, cg);
    if ~isfinite(dist(g))
        % 도달 불가 -> 목표에 가장 가까운 도달 가능 셀로 대체
        [rr, cc] = find(isfinite(dist));
        [xx, yy] = grid2world(map, rr, cc);
        [~, kmin] = min(hypot(xx - goalXY(1), yy - goalXY(2)));
        if isempty(kmin)
            wp = zeros(0, 2); info.status = 'unreachable'; return;
        end
        g = sub2ind(size(dist), rr(kmin), cc(kmin));
        info.status = 'unreachable';
    end
end

% ---------- 역추적 ----------
idx = g; cells = idx;
while parent(idx) ~= 0
    idx = parent(idx);
    cells(end+1) = idx; %#ok<AGROW>
end
cells = flipud(cells(:));
[pr, pc] = ind2sub(size(dist), cells);
[px, py] = grid2world(map, pr, pc);
pathXY = [robot(1) robot(2); px(2:end) py(2:end)];

wp = simplify_path(map, pathXY, trav);
[gx, gy] = grid2world(map, pr(end), pc(end));
goalXY = [gx gy];
end
