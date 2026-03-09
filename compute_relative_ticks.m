%Re-working angle normalization formula
function [incremental] = compute_relative_ticks(incremental_values, max_value)

    half_max_value = max_value / 2;

    incremental = mod(diff(incremental_values) + half_max_value, max_value) - half_max_value; %Using octave sugar

endfunction
