#computes the trajectory of the robot by chaining up
#the incremental movements of the odometry vector
#U:	a Nx3 matrix, each row contains the odoemtry ux, uy utheta
#T:	a Nx3 matrix, each row contains the robot position (starting from 0,0,0)

function T=compute_sensor_odometry(U_r,u_s)
	T=zeros(size(U_r,1),3);
	
	for i=1:size(U_r,1),
		u_r=U_r(i,1:3)';
		
		T(i,1:3)=t2v(v2t(u_r)*v2t(u_s))';
	end
end
