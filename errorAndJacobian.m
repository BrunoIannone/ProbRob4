function [e, J,status] = errorAndJacobian(x,z,current_pose)
  global encoder_max_values
  status = 0;
  ticks = z(1:2);
  meas  = z(3:5);
  %current_pose(3) = wrapToPi(current_pose(3));
  delta_robot = h_odom(x, ticks, current_pose(3), encoder_max_values);
  % if(delta_robot==[0;0;0])

  %   status = -1;
  % endif
  


  pred = t2v(v2t(current_pose) * v2t(delta_robot)*v2t(x(5:7)))';
  pred(3)=wrapToPi(pred(3));
  
  e     = (pred-meas)';
  e(3) = wrapToPi(e(3));
  e;
  J     = zeros(3,7);
  epsilon = 1e-3;
  inv_eps2= 0.5/epsilon;
  
  for (i=1:7)
  
    e_vec = zeros(1,7);
    e_vec(i)=epsilon;
    
    xp = x; xp(i) += epsilon;
    xm = x; xm(i) -= epsilon;

    % if i == 7

    %   xp(i) = wrapToPi(xp(i));
    %   xm(i) = wrapToPi(xm(i));

    % endif
    xp;
    xm;
    
    delta_robot_p = h_odom(xp, ticks, current_pose(3), encoder_max_values);
    pred_p = t2v(v2t(current_pose)*v2t(delta_robot_p)*v2t(xp(5:7)))';

    delta_robot_m = h_odom(xm, ticks, current_pose(3), encoder_max_values);
    pred_m = t2v(v2t(current_pose) * v2t(delta_robot_m)*v2t(xm(5:7)))';
    % pred_p(3) = wrapToPi(pred_p(3));
    % pred_m(3) = wrapToPi(pred_m(3));

    pred_p;
    pred_m;

    diffe = pred_p - pred_m;
    diffe(3) = wrapToPi(diffe(3));
    
    J(:,i) = inv_eps2 * (diffe)';
    
  endfor;
  
  
endfunction
