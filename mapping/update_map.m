function map = update_map(map, robot, angles, ranges, maxRange)
% UPDATE_MAP  레이저 스캔으로 occupancy grid 갱신 (log-odds)
%   map      : init_map 결과
%   robot    : [x y theta]  (theta [rad])
%   angles   : 빔 각도 (로봇 기준, [rad]) 벡터
%   ranges   : 측정 거리 [m] 벡터 (같은 길이)
%   maxRange : 센서 최대 거리 [m]. 이 이상(또는 NaN/Inf)은 hit 없음으로 처리
%
%   * scan 실제 형식이 확인되면 main에서 angles/ranges로 변환해서 넘길 것
%
%   담당: B

x0 = robot(1); y0 = robot(2); th = robot(3);
angles = angles(:); ranges = ranges(:);

step = map.res * 0.5;          % 레이 샘플 간격
freeMask = false(map.n);
occMask  = false(map.n);

for k = 1:numel(ranges)
    rk = ranges(k);
    hit = isfinite(rk) && rk < maxRange * 0.98;
    if ~isfinite(rk) || rk > maxRange
        rk = maxRange;
    end
    a = th + angles(k);
    ca = cos(a); sa = sin(a);

    % 통과 구간 (끝점 바로 앞까지) -> free
    d = 0:step:max(rk - map.res, 0);
    [rr, cc, v] = world2grid(map, x0 + d * ca, y0 + d * sa);
    freeMask(sub2ind([map.n map.n], rr(v), cc(v))) = true;

    % 끝점 -> occupied
    if hit
        [re, ce, ve] = world2grid(map, x0 + rk * ca, y0 + rk * sa);
        if ve
            occMask(re, ce) = true;
        end
    end
end

freeMask(occMask) = false;     % 같은 스캔 안에서는 hit 우선
map.L(freeMask) = map.L(freeMask) + map.lFree;
map.L(occMask)  = map.L(occMask)  + map.lOcc;
map.L = min(max(map.L, map.lMin), map.lMax);
end
