function pred = get_prediction(x,ticks,encoder_max_values)
  
  delta = h_odom(x, ticks, encoder_max_values); 
  if( all(delta == [0;0;0])) % If the robot didn't move, do nothing
    pred = -1;
    return;
  endif
  
  pred = 0;
  r_T_s = v2t(x(5:7));
  s_T_r = inv(r_T_s);

  
  T_delta = v2t(delta);


  pred = s_T_r*T_delta*r_T_s; % sensor increment (this was painfully hard to find)
end
