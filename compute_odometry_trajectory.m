#computes the trajectory of the robot by chaining up
#the incremental movements of the odometry vector
#U:	a Nx3 matrix, each row contains the odoemtry ux, uy utheta
#T:	a Nx3 matrix, each row contains the robot position (starting from 0,0,0)

function T=compute_odometry_trajectory(U_r,U_s)
	T=zeros(size(U_r,1),3);
	current_T=v2t(zeros(1,3));
	for i=1:size(U_r,1),
		u_r=U_r(i,1:3)';
		u_s=U_s(i,1:3)';
		# this is P in the slides (thank bart)

		# HINT : current_T is the absoulute pose obtained by concatenating 
		# all the relative transformation until the current timestamp i
		current_T *= v2t(u_r)*v2t(u_s); %TODO
		T(i,1:3)=t2v(current_T)';
	end
end
