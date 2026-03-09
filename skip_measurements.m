function [skipped_data] = skip_measurements(data,skip_indices)
if(size(skip_indices,1)) == 0
    skipped_data = data;
    disp("return")
    return;
endif

size_ = size(data,1);
n_skip = size(skip_indices,1);
skipped_data = [];
k = 1;
for i=1:size_
    skip_indices(k);
    if(i == skip_indices(k))
        
        if(k+1<=n_skip) 
            k = k+1;
        endif
        continue;
    else
        skipped_data = [skipped_data;data(i,:)];
    end

endfor

endfunction