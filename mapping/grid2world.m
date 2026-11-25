function [x, y] = grid2world(map, r, c)
% GRID2WORLD  격자 인덱스 -> 셀 중심 월드 좌표 [m]

x = (c - 0.5) .* map.res;
y = (r - 0.5) .* map.res;
end
