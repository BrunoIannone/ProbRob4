function delta = h_odom(x,ticks,encoder_max_values)
k_s = x(1);
k_t = x(2);
steering_offset = x(3);
l = x(4);
t_s = ticks(1); % absolute
t_t = ticks(2); % incremental

max_steering=encoder_max_values(1);
max_incremental = encoder_max_values(2);
phi = k_s*(normalizeAngle(t_s*2*pi/max_steering))+ steering_offset;
ds = k_t*(t_t/max_incremental);
if(ds == 0)
    delta = [0;0;0];

else
    
    dth = (sin(phi)/l)*ds;
    dx = cos(0)*cos(phi)*ds;
    dy = sin(0)*cos(phi)*ds;
    delta = [dx;dy;dth];
endif
endfunction