function pred = get_prediction(x,ticks,encoder_max_values)
  
  r_T_s = v2t(x(5:7));
  s_T_r = inv(r_T_s);

  delta = h_odom(x, ticks, encoder_max_values);
  T_delta = v2t(delta);


  pred = s_T_r*T_delta*r_T_s; % sensor increment (this was painfully hard to find)
end
