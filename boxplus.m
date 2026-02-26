function x_new = boxplus(x, increment_vector)

    robot_state = x(1:4);
    robot_increment = increment_vector(1:4);
    robot_state += robot_increment;

    sensor_state = x(5:7);
    sensor_increment = increment_vector(5:7);
    sensor_state = t2v(v2t(sensor_increment) * v2t(sensor_state))'; %row

    x_new = [robot_state, sensor_state];
endfunction
