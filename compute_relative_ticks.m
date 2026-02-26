%Re-adapted from https://www.youtube.com/watch?v=m_2GKHqBmM0

function [incremental] = compute_relative_ticks(incremental_values, max_value)
    half_max_value = max_value / 2;

    incremental = zeros(size(incremental_values, 1) - 1, 1);

    for (i = 1:size(incremental_values, 1) - 1)

        difference = incremental_values(i + 1, :) - incremental_values(i, :);

        if (difference < 0 && abs(difference) >= half_max_value)
            incremental(i, :) = (max_value - incremental_values(i, :)) + incremental_values(i + 1, :);

        elseif abs(difference) >= half_max_value
            incremental(i, :) = (incremental_values(i + 1, :) - max_value) - incremental_values(i, :)

        else
            incremental(i, :) = difference;
        end

    endfor

endfunction
