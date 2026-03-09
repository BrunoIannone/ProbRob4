%Re-working angle normalization formula
function [incremental] = compute_relative_ticks(incremental_values, max_value)
    
    half_max_value = max_value / 2;
    size_ = size(incremental_values, 1);
    
    incremental = [];

    for (i = 1:size_-1)

        difference = incremental_values(i+1, :) - incremental_values(i, :);

        incremental = [incremental; mod(difference + half_max_value, max_value) - half_max_value];

    endfor

endfunction
