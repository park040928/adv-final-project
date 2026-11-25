function S = get_state(map)
% GET_STATE  log-odds -> 셀 상태
%   S =  1 : occupied
%   S =  0 : free
%   S = -1 : unknown

S = -ones(map.n);
S(map.L <= map.thFree) = 0;
S(map.L >= map.thOcc)  = 1;
end
