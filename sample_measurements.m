function sampled_data = sample_data(data,sample_value=5)
    sampled_data = data(1:sample_value:size(data,1), :);
endfunction