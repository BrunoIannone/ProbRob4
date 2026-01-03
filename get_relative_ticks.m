function [incremental]=get_relative_ticks(incremental_values)
    max_value = 2^32;
    threshold  = (max_value*80)/100  # 4080218931
    disp("sjsjs")
    disp(threshold>max_value/2)
    overflow_incoming = false;

    %absolute = zeros(size(absolute_values,1)-1,1);
    incremental = zeros(size(incremental_values,1)-1,1);
    
    for (i = 1:size(incremental_values,1))
        
        if( incremental_values(i,:) > threshold && !overflow_incoming)
           overflow_incoming = true;

        endif
        
        if(i+1 > size(incremental_values,1)) 
            break
        endif     
        
        difference = incremental_values(i+1,:) - incremental_values(i,:);
        
        
        if(difference < 0  && overflow_incoming)
            if(incremental_values(i,:) > threshold && incremental_values(i+1,:)<max_value/2)
                
                incremental(i,:) = (max_value - incremental_values(i,:)) + incremental_values(i+1,:);
                overflow_incoming = false;
            endif

        

        else
            incremental(i,:) = difference;
        end
        
    
    endfor
endfunction
    