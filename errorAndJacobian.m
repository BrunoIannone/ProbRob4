function [e, J,status] = errorAndJacobian(x, Z, encoder_max_values)
  ticks = Z(1:2);
  sensor_meas = Z(3:5);
  status = 0;
  pred = get_prediction(x, ticks, encoder_max_values);
  if all(pred == [0; 0; 0])
    status = -1;
    return;
  endif
  e = t2v(v2t(sensor_meas)*inv(pred));
  
  J = zeros(3, 7);
  epsilon = 1e-6;
  
  for k = 1:7
    dx_plus  = zeros(1, 7);
    dx_minus = zeros(1, 7);
    dx_plus(k)  =  epsilon;
    dx_minus(k) = -epsilon;
    
    x_plus  = boxplus(x, dx_plus);
    x_minus = boxplus(x, dx_minus);
    
    pred_plus  = get_prediction(x_plus,  ticks, encoder_max_values);
    pred_minus = get_prediction(x_minus, ticks, encoder_max_values);
    
    e_plus  = t2v(v2t(sensor_meas)*inv(pred_plus) );
    e_minus = t2v(v2t( sensor_meas)* inv(pred_minus));
    diff_ = e_plus - e_minus;
    diff_(3) = normalizeAngle(diff_(3));
    J(:, k) = diff_ / (2 * epsilon);
  
  end
end