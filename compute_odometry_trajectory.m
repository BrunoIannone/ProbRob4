function odom = compute_odometry_trajectory(x, ticks, encoder_max_values)
    odom = zeros(size(ticks, 1), 3);
    odom(1, :) = [0, 0, 0];
    curr_theta = odom(1, 3);

    for i = 2:size(ticks, 1)

        odom(i, :) = (odom(i - 1, :)' + h_odom(x, ticks(i, :), curr_theta, encoder_max_values))';
        odom(i, 3) = normalizeAngle(odom(i, 3));
        curr_theta = odom(i, 3);

    end

end
