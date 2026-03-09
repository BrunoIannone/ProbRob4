%Re-working angle normalization formula

function [incremental,skipped] = compute_relative_ticks(incremental_values, max_value,time_values)
    curr_time = time_values(1);
    curr_idx = 1;
    half_max_value = max_value / 2;
    skipped = [];
    size_ = size(incremental_values, 1) 
    incremental = [];

    for (i = 1:size_)
        if(abs(curr_time-time_values(i))>0.3)
        
            difference = incremental_values(i , :) - incremental_values(curr_idx, :);

            incremental = [incremental; mod(difference + half_max_value, max_value) - half_max_value];
            curr_time = time_values(i);
            curr_idx = i;
        else
            skipped = [skipped;i];
        endif
    endfor

endfunction
