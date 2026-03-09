function [x_new, chi_record] = calibrate(x, Z, n_iterations, encoder_max_values, damping, plot_)

    if plot_
        live_plot = init_figure(5, 'Live calibration', 'World x', 'World y');

        calibrated_my_robot_odometry = compute_odometry_trajectory(stack_odometry(x(1:4), [Z(:, 1), Z(:, 2)], encoder_max_values), t2v(inv(v2t(x))));
        calibrated_my_sensor_odometry = compute_sensor_odometry(calibrated_my_robot_odometry, x(5:7));

        h7 = my_plot(calibrated_my_robot_odometry, live_plot, 2, 'g-');
        h8 = my_plot(calibrated_my_sensor_odometry, live_plot, 2, 'm-');
        legend([h7 h8], {'Robot calibrated odometry', 'My sensor calibrated odometry'});
        drawnow;
        hold off;

        h_param_fig = init_figure(6, 'Live parameter evolution', 'iteration', 'value');
        x_record(:, 1) = x;
        plot_param_evolution(h_param_fig, x_record, 0, 0, 0);
    endif

    #### LS LOOP ####
    nmeas = size(Z, 1);
    chi_record = zeros(1, n_iterations);
    x_new = x;

    for (j = 1:n_iterations)
        Iteration = j

        H = zeros(7, 7);
        b = zeros(7, 1);
        chi = 0;

        for (i = 1:nmeas)
            pred = get_prediction(x_new, Z(i, 1:2), encoder_max_values);

            if pred == -1
                continue;
            endif

            [e, J] = errorAndJacobian(x_new, Z(i, :), encoder_max_values, pred);

            chi += e' * e;
            H += J' * J;
            b += J' * e;
        endfor

        H += eye(7) * damping;
        dx = -H \ b;
        x_new = boxplus(x_new, dx');
        ##############################

        x_record(:, j + 1) = x_new;
        chi_record(j) = chi;

        if plot_
            calibrated_my_robot_odometry = compute_odometry_trajectory(stack_odometry(x_new(1:4), [Z(:, 1), Z(:, 2)], encoder_max_values), t2v(inv(v2t(x_new(5:7)))));
            calibrated_my_sensor_odometry = compute_sensor_odometry(calibrated_my_robot_odometry, x_new(5:7));

            set(h7, 'XData', calibrated_my_robot_odometry(:, 1), 'YData', calibrated_my_robot_odometry(:, 2));
            set(h8, 'XData', calibrated_my_sensor_odometry(:, 1), 'YData', calibrated_my_sensor_odometry(:, 2));
            drawnow;

            plot_param_evolution(h_param_fig, x_record, chi_record, dx, j);
        endif

    endfor

end
