function [x_new, chi_record] = oneRound(x, Z, n_iterations,encoder_max_values)

    nmeas = size(Z, 1);
    chi_record = zeros(1, n_iterations)
    x_new = x
    odom = Z(:,6:8)
    for (j = 1:n_iterations)
        j

        H = zeros(7, 7);
        b = zeros(7, 1);
        chi = 0;
        current_pose = [0,0,0];
        for (i = 1:nmeas)

            [e, J] = errorAndJacobian(x_new, Z(i, :),encoder_max_values,current_pose);
            
            chi += e' * e;

            H += J' * J;
            b += J' * e;

        endfor

        dx=-H \ b;
        [sensor_state robot_state] = boxplus(x_new,dx');
        x_new = [sensor_state'; robot_state']'

        chi_record(j) = chi;
    endfor

end
