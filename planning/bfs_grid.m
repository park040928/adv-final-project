function [dist, parent] = bfs_grid(trav, r0, c0)
% BFS_GRID  격자 위 최단거리 탐색 (8방향, 대각 corner-cutting 금지)
%   trav   : logical 주행 가능 mask
%   (r0,c0): 시작 셀
%   dist   : 시작점부터의 거리 [cell] (도달 불가 = Inf)
%   parent : 역추적용 선형 인덱스 (0 = 없음)
%
%   * 대각 비용 sqrt(2) 반영: 더 짧은 거리를 찾으면 갱신 후 다시 큐에 넣는
%     label-correcting 방식 (결과는 Dijkstra와 동일)
%
%   담당: B

[n, m] = size(trav);
dist   = inf(n, m);
parent = zeros(n, m);

dr = [-1  1  0  0 -1 -1  1  1];
dc = [ 0  0 -1  1 -1  1 -1  1];
w  = [ 1  1  1  1  sqrt(2) sqrt(2) sqrt(2) sqrt(2)];

queue = zeros(n * m * 2, 1);
head = 1; tail = 1;
s = sub2ind([n m], r0, c0);
queue(tail) = s; tail = tail + 1;
dist(s) = 0;

while head < tail
    cur = queue(head); head = head + 1;
    [r, c] = ind2sub([n m], cur);
    for k = 1:8
        rn = r + dr(k); cn = c + dc(k);
        if rn < 1 || rn > n || cn < 1 || cn > m || ~trav(rn, cn)
            continue;
        end
        if k > 4 && (~trav(r, cn) || ~trav(rn, c))   % 모서리 끼임 방지
            continue;
        end
        nd = dist(cur) + w(k);
        if nd < dist(rn, cn) - 1e-9
            dist(rn, cn) = nd;
            parent(rn, cn) = cur;
            if tail > numel(queue)
                queue = [queue; zeros(numel(queue), 1)]; %#ok<AGROW>
            end
            queue(tail) = sub2ind([n m], rn, cn); tail = tail + 1;
        end
    end
end
end
