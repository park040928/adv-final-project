function blocked = inflate_obstacles(occ, radius, res)
% INFLATE_OBSTACLES  장애물을 radius [m] 만큼 팽창 (C-space)
%   occ    : logical occupied mask
%   radius : 팽창 반경 [m] (로봇 반지름 + 여유)
%   res    : 격자 해상도 [m/cell]
%   (Image Processing Toolbox 없이 conv2 사용)
%
%   담당: B

k = ceil(radius / res);
[dx, dy] = meshgrid(-k:k, -k:k);
disk = double(dx.^2 + dy.^2 <= k^2);
blocked = conv2(double(occ), disk, 'same') > 0;
end
