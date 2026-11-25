function map = init_map(mapSize, res)
% INIT_MAP  Occupancy grid (log-odds) 초기화
%   map = init_map(mapSize, res)
%   mapSize : 맵 한 변 길이 [m] (예: 50)
%   res     : 격자 해상도 [m/cell] (예: 0.2)
%
%   grid 인덱스: map.L(row, col)  row <-> y, col <-> x
%
%   담당: B (맵 탐색 및 경로 계획)

map.res  = res;
map.size = mapSize;
map.n    = ceil(mapSize / res);
map.L    = zeros(map.n, map.n);   % log-odds, 0 = unknown (p = 0.5)

% log-odds 파라미터
map.lOcc  =  0.85;   % hit 시 증가량
map.lFree = -0.40;   % 통과 시 감소량
map.lMin  = -5;
map.lMax  =  5;

% 상태 판정 임계값 (log-odds)
map.thOcc  =  0.6;
map.thFree = -0.6;
end
