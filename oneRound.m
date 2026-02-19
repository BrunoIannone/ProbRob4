function [x_new, chi_record] = oneRound(x, Z, n_iterations,encoder_max_values)

    nmeas = size(Z, 1);
    chi_record = zeros(1, n_iterations)
    x_new = x
    kernel_threshold = 100;
    
    for (j = 1:n_iterations)
        j

        H = zeros(7, 7);
        b = zeros(7, 1);
        chi = 0;
        current_pose = [0,0,0];
        for (i = 1:nmeas)

            [e, J, current_pose,status] = errorAndJacobian(x_new, Z(i, :),encoder_max_values,current_pose);
            % if(status==-1)
            %   continue
            % endif
            chi += e' * e;

            % if (chi > kernel_threshold)
            %     e *= sqrt(kernel_threshold / chi);
            %     chi = kernel_threshold;
            % endif

            
            H += J' * J;
            b += J' * e;

        endfor
        H += eye(7) * 0.01;   % initialize with damping once

        dx=-H \ b;
        x_new += dx';

        x_new(7) = wrapToPi(x_new(7));

        chi_record(j) = chi;
    endfor

end
