#computes the trajectory of the robot by chaining up
#the incremental movements of the odometry vector
#U:	a Nx3 matrix, each row contains the odoemtry ux, uy utheta
#T:	a Nx3 matrix, each row contains the robot position (starting from 0,0,0)

function T = compute_odometry_trajectory(U, initial_pose = [0, 0, 0])
    T = zeros(size(U, 1), 3);
    current_T = v2t(initial_pose);

    for i = 1:size(U, 1),
        u = U(i, 1:3)';

        current_T *= v2t(u);
        T(i, 1:3) = t2v(current_T)';
    end

end
