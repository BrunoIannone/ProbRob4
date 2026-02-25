function odom = stack_odometry(x, ticks,encoder_max_values)
    odom = zeros(size(ticks, 1), 3);

    for i = 1:size(ticks, 1)
        % if (ticks(i, 2) == [0])
        %     continue
        % end

        odom(i, :) = h_odom(x, ticks(i, :),encoder_max_values)';
    end
end
