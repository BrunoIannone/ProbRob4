function [incremental]=get_relative_ticks(incremental_values)
    max_value = 2^32;
    threshold  = max_value/2;  # 4080218931
    overflow_incoming = false;

    %absolute = zeros(size(absolute_values,1)-1,1);
    incremental = zeros(size(incremental_values,1)-1,1);

    for (i = 1:size(incremental_values,1))
        
        if( incremental_values(i,:) > threshold && !overflow_incoming)
           overflow_incoming = true; 
           %disp("Close to overflow. Threshold surpassed")
        endif
        
        if(i+1 > size(incremental_values,1)) 
            break
        endif     
        
        
        %incremental_values(i+1,:)
        %incremental_values(i,:)
        difference = incremental_values(i+1,:) - incremental_values(i,:);
        %difference
        
        if(difference < 0  && overflow_incoming)
            if(incremental_values(i,:) > threshold && incremental_values(i+1,:)<max_value/2)
                %disp("overflow occurred")
                incremental(i,:) = incremental_values(i+1,:);
                overflow_incoming = false;
            endif

        

        else
            incremental(i,:) = difference;
        end
        
        %incremental(i,:)
        
        %disp("--------")
        %pause (3)
    
    endfor
endfunction
    