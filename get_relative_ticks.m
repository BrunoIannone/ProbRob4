function [absolute,incremental]=get_relative_ticks(absolute_values,incremental_values)
    
    absolute = zeros(size(absolute_values,1)-1,1);
    incremental = zeros(size(incremental_values,1)-1,1);

    for (i = 1:size(absolute_values,1))
         
         if(i+1 > size(absolute_values,1)) 
            break
         endif     
        incp1 = incremental_values(i+1)
        inc = incremental_values(i)
        
        incremental(i,:) = incremental_values(i+1,:) - incremental_values(i,:);
        incremental(i,:)
        disp("_--")
        absolute(i,:) = absolute_values(i+1,:) - absolute_values(i,:);
    
    endfor
endfunction
    