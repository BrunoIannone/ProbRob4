function T = compute_sensor_odometry(U_r, u_s)
    T = zeros(size(U_r, 1), 3);

    for i = 1:size(U_r, 1),
        u_r = U_r(i, 1:3)';

        T(i, 1:3) = t2v(v2t(u_r) * v2t(u_s))';
    end

end
