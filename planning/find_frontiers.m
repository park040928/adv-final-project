function [F, gain] = find_frontiers(S, trav)
% FIND_FRONTIERS  탐색 경계(frontier) 셀 검출
%   S     : get_state 결과 (1 occ / 0 free / -1 unknown)
%   trav  : 주행 가능 mask (팽창 반영)
%   F     : logical, frontier 셀 (주행 가능한 free 셀 중 unknown과 인접)
%   gain  : 각 셀 주변 unknown 셀 개수 (정보 이득 추정치)
%
%   담당: B

unknown = (S == -1);
k4 = [0 1 0; 1 0 1; 0 1 0];
nearUnknown = conv2(double(unknown), k4, 'same') > 0;
F = trav & (S == 0) & nearUnknown;

win = ones(9);                       % 주변 9x9 셀 범위의 미탐색량
gain = conv2(double(unknown), win, 'same');
end
