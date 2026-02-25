function [x_new, chi_record] = oneRound(x, Z, n_iterations,encoder_max_values)

    nmeas = size(Z, 1);
    chi_record = zeros(1, n_iterations);
    x_new = x
    for (j = 1:n_iterations)
        j

        H = zeros(7, 7);
        b = zeros(7, 1);
        chi = 0;
        for (i = 1:nmeas)

            [e, J,status] = errorAndJacobian(x_new, Z(i, :),encoder_max_values);
            if(status == -1)
                continue
            endif
            chi += e' * e;

            H += J' * J;
            b += J' * e;

        endfor
        H+= eye(7)*0.6;
        dx=-H \ b;
        x_new= boxplus(x_new,dx')

        chi_record(j) = chi;
    endfor

end
