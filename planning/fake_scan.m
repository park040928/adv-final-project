function ranges = fake_scan(robot, angles, maxRange, walls)
% FAKE_SCAN  테스트용 가상 레이저 (update_robot 없이 B 모듈 검증용)
% 직사각형 벽 [x1 y1 x2 y2] 들에 대한 레이 캐스팅 (slab method)
ranges = maxRange * ones(size(angles));
for k = 1:numel(angles)
    a  = robot(3) + angles(k);
    d  = [cos(a) sin(a)];
    for w = 1:size(walls, 1)
        lo = walls(w, 1:2); hi = walls(w, 3:4);
        t1 = (lo - robot(1:2)) ./ d;
        t2 = (hi - robot(1:2)) ./ d;
        tmin = max(min(t1, t2)); tmax = min(max(t1, t2));
        if tmax >= max(tmin, 0) && tmin < ranges(k) && tmin > 0
            ranges(k) = tmin;
        end
    end
end
ranges = ranges + 0.01 * randn(size(ranges));   % 센서 노이즈
end
