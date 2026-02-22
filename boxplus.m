function [sensor_state robot_state] = boxplus(x, increment_vector)

robot_state = x(1:4);
robot_increment = increment_vector(1:4);
robot_state+=robot_increment;

sensor_state = x(5:7);
sensor_increment = increment_vector(5:7);
sensor_state = t2v(v2t(sensor_increment)*v2t(sensor_state))'; %row

endfunction