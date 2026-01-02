function delta = h_odom(x,ticks,theta)
k_s = x(1);
k_t = x(2);
steering_offset = x(3);
l = x(4);
t_s = ticks(1); % absolute
t_t = ticks(2); % incremental
%t_t%
phi = mod(k_s*(t_s-steering_offset)+pi,2*pi)-pi;
ds = k_t*t_t;
if(ds == 0)
    delta = [0;0;0];

else
    
    dth = (sin(phi)/l)*ds;
    %dphi = k_s*(t_s - steering_offset);
    dx = cos(theta+phi)*ds;
    dy = sin(theta+phi)*ds;
    delta = [dx;dy;dth];
end