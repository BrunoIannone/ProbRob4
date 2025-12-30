function [absolute,incremental]=refine_ticks(absolute_values,absolute_max,incremental_values,incremental_max)
    
    absolute = zeros(size(absolute_values));
    incremental = zeros(size(incremental_values));

    for (i = 1:size(absolute_values,1))
               
        incremental(i) = mod(incremental_values(i), incremental_max);
        absolute(i) = mod(absolute_values(i), absolute_max);
    
    endfor
endfunction
    