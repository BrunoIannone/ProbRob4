function odom = stack_odometry(x, ticks)
    #odom = zeros(size(ticks, 1), 3);

    for i = 1:size(ticks, 1)
        if all(ticks(i, :) == [0 0])
            continue
        end

        odom(i, :) = h_odom(x, ticks(i, :))';
    end
end
