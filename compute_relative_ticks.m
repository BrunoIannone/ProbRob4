%Re-adapted from https://www.youtube.com/watch?v=m_2GKHqBmM0

function [incremental] = compute_relative_ticks(incremental_values, max_value)

    incremental = zeros(size(incremental_values, 1) - 1, 1);

    for (i = 1:size(incremental_values, 1) - 1)

        difference = incremental_values(i + 1, :) - incremental_values(i, :);

        if (difference < 0 && abs(difference) >= max_value / 2)
            incremental(i, :) = (max_value - incremental_values(i, :)) + incremental_values(i + 1, :);

        

        elseif abs(difference) >= max_value / 2
            incremental(i, :) = (incremental_values(i + 1, :) - max_value) - incremental_values(i, :)

        else
            incremental(i, :) = difference;
        end

    endfor

endfunction
