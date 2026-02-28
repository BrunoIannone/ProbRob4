function [x_new, chi_record] = calibrate(x, Z, n_iterations, encoder_max_values,damping, plot_, live_plot)

    if plot_
        calibrated_my_robot_odometry = compute_odometry_trajectory(stack_odometry(x(1:4), [Z(:, 1), Z(:, 2)], encoder_max_values), -x(5:7));

        calibrated_my_sensor_odometry = compute_sensor_odometry(calibrated_my_robot_odometry, x(5:7));

        h6 = my_plot(calibrated_my_robot_odometry, live_plot, 2, 'g-');
        h7 = my_plot(calibrated_my_sensor_odometry, live_plot, 2, 'm-');
        legend([h6 h7], {'Robot calibrated odometry', "My sensor calibrated odometry"});
        drawnow;
        hold off;
    endif

    nmeas = size(Z, 1);
    chi_record = zeros(1, n_iterations);
    x_new = x

    for (j = 1:n_iterations)
        j

        H = zeros(7, 7);
        b = zeros(7, 1);
        chi = 0;

        for (i = 1:nmeas)

            [e, J, status] = errorAndJacobian(x_new, Z(i, :), encoder_max_values);

            if (status == -1)
                continue
            endif

            chi += e' * e;

            H += J' * J;
            b += J' * e;

        endfor

        H += eye(7) * damping; %0.001;
        dx=-H \ b;
        x_new = boxplus(x_new, dx')

        chi_record(j) = chi;

        if plot_

            calibrated_my_robot_odometry = compute_odometry_trajectory(stack_odometry(x_new(1:4), [Z(:, 1), Z(:, 2)], encoder_max_values), -x_new(5:7));

            calibrated_my_sensor_odometry = compute_sensor_odometry(calibrated_my_robot_odometry, x_new(5:7));

            h6 = my_plot(calibrated_my_robot_odometry, live_plot, 2, 'g-');
            h7 = my_plot(calibrated_my_sensor_odometry, live_plot, 2, 'm-');
            legend([h6 h7], {'Robot calibrated odometry', "My sensor calibrated odometry"});
            drawnow;
            hold off;

        endif

    endfor

end
