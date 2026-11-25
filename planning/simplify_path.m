function wp = simplify_path(map, pathXY, trav)
% SIMPLIFY_PATH  격자 경로 -> 꺾이는 점만 남긴 waypoint (line-of-sight 기준)
%   pathXY : Nx2 [x y] 셀 중심 좌표 경로
%   trav   : 주행 가능 mask
%   wp     : Mx2 waypoint (첫 점 = 현재 위치 제외, 마지막 = 목표)
%
%   담당: B

N = size(pathXY, 1);
if N <= 1
    wp = pathXY;
    return;
end

wp = zeros(0, 2);
i = 1;
while i < N
    j = N;
    while j > i + 1 && ~line_free(map, pathXY(i,:), pathXY(j,:), trav)
        j = j - 1;
    end
    wp(end+1, :) = pathXY(j, :); %#ok<AGROW>
    i = j;
end
end

function ok = line_free(map, p, q, trav)
L = norm(q - p);
n = max(2, ceil(L / (map.res * 0.5)));
t = linspace(0, 1, n)';
pts = p + t .* (q - p);
[r, c, v] = world2grid(map, pts(:,1), pts(:,2));
ok = all(v) && all(trav(sub2ind(size(trav), r, c)));
end
