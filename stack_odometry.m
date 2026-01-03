function odom = stack_odometry(x, ticks)
    odom = zeros(size(ticks, 1), 3);
    curr_theta = 0;
    
    for i = 1:size(ticks, 1)

        if i == 1
            odom(i, :) = h_odom(x, ticks(i, :), curr_theta)';
            odom(i, 3) = wrapToPi(odom(i, 3));
            curr_theta = odom(i, 3);
        else
            odom(i, :) = (odom(i-1, :)' + h_odom(x, ticks(i, :), curr_theta))';
            odom(i, 3) = wrapToPi(odom(i, 3));
            curr_theta = odom(i, 3);
        end

    end

end
