function skipped_meas = skipped_measurements(odometry,skip_value=5)
    skipped_meas = [];
    for (i =1:size(odometry,1))
        if mod(i,skip_value) == 0

            skipped_meas = [skipped_meas; odometry(i,:)];
        endif
    endfor
endfunction