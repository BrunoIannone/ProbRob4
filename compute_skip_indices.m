function [skip_indices] = compute_skip_indices(time_values,threshold=0.3)

    size_ = size(time_values, 1);

    curr_time = time_values(1);

    skip_indices = [];

    for (i = 2:size_)

        if (abs(curr_time - time_values(i)) > threshold)

            curr_time = time_values(i);

            continue;
        else
            skip_indices = [skip_indices; i];
            
        endif

    endfor

endfunction
