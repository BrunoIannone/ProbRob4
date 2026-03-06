%Re-working angle normalization formula

function [incremental] = compute_relative_ticks(incremental_values, max_value)
    half_max_value = max_value / 2;

    size_ = size(incremental_values, 1) - 1;
    incremental = zeros(size_, 1);

    for (i = 1:size_)

        difference = incremental_values(i + 1, :) - incremental_values(i, :);

        incremental(i, :) = mod(difference + half_max_value, max_value) - half_max_value;

    endfor

endfunction
