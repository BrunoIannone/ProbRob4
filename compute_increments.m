function increment = compute_increments(data)
    
    increment = zeros(size(data,1)-1,3);

    for (i = 1:size(data,1)-1)
        w_T_0   = v2t(data(i,:));
        w_T_1 =   v2t(data(i+1,:));

        T_delta = inv(w_T_0) * w_T_1;

        increment(i,:) = t2v(T_delta)';

    end
end