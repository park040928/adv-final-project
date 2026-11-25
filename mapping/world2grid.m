function [r, c, valid] = world2grid(map, x, y)
% WORLD2GRID  월드 좌표 [m] -> 격자 인덱스 (row, col)
%   범위 밖 좌표는 valid = false, 인덱스는 경계로 clamp

c = floor(x ./ map.res) + 1;
r = floor(y ./ map.res) + 1;
valid = (r >= 1) & (r <= map.n) & (c >= 1) & (c <= map.n);
r = min(max(r, 1), map.n);
c = min(max(c, 1), map.n);
end
