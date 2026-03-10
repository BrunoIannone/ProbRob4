close all
clear
clc
more off;

#import 2d geometry utils
source "./tools/utilities/geometry_helpers_2d.m"

#### Flags ####
plot_ = true;
save_gif = false;
threshold = 0.3;
############

#### Load data matrix ####
disp('Loading data matrix');
Z = load("./data/dataset_octave.txt");

#### Assign data ####
disp('Assign data');
time_values = Z(:, 1);
absolute_values = Z(:, 2);
incremental_values = Z(:, 3);
robot_odometry_values = Z(:, 4:6);
sensor_gt_values = Z(:, 7:9);
################################

#### Constants ####
ENCODER_MAX_VALUES = [8192 5000];
MAX_INCREMENTAL_VARIABLE = 2^32;

NOMINAL_PARAMS = [0.1 0.0106141 0 1.4];
SENSOR_TRANSLATION_WRT_ROBOT = [1.5, 0, 0];
DAMPING = 1.5;
N_ITERATIONS = 20;
################################

if (plot_)% Init figures

    robot_odometry_plot = init_figure(1, 'Robot Odometry', 'World x', 'World y');
    sensor_odometry_plot = init_figure(2, 'Sensor Odometry', 'World x', 'World y');
    calibrated_sensor_odometry_plot = init_figure(3, 'Calibrated sensor Odometry', 'World x', 'World y');
    chi_plot = init_figure(4, 'Chi evolution', 'Iteration', 'Chi value y');
endif

skip_indices = compute_skip_indices(time_values, threshold);

incremental_values = skip_measurements(incremental_values, skip_indices);
incremental_values_rel = compute_relative_ticks(incremental_values, MAX_INCREMENTAL_VARIABLE);

absolute_values = skip_measurements(absolute_values, skip_indices);
absolute_values = absolute_values(1:end - 1, :);

my_robot_odometry = compute_odometry_trajectory(stack_odometry(NOMINAL_PARAMS, [absolute_values, incremental_values_rel], ENCODER_MAX_VALUES));

if (plot_)%Robot uncalibrated odometry validation

    h1 = my_plot(robot_odometry_values, robot_odometry_plot, 5, 'r-');

    h2 = my_plot(my_robot_odometry, robot_odometry_plot, linewidth = 2, 'g-');

    legend([h1 h2], {'Robot odometry', "My robot odometry"});
endif

my_sensor_odometry = compute_sensor_odometry(my_robot_odometry, SENSOR_TRANSLATION_WRT_ROBOT);

if (plot_)% Sensor uncalibrated odometry validation

    h3 = my_plot(sensor_gt_values, sensor_odometry_plot, linewidth = 2, 'r-');

    h4 = my_plot(my_sensor_odometry, sensor_odometry_plot, linewidth = 2, 'g-');

    legend([h3 h4], {'Sensor GT', "My sensor odometry"});
endif

sensor_gt_values = skip_measurements(sensor_gt_values, skip_indices);

if (plot_)

    h5 = my_plot(sensor_gt_values, calibrated_sensor_odometry_plot, 2, 'r-');

    legend([h5], {'Sensor GT'});

endif

sensor_gt_rel = compute_increments(sensor_gt_values);

% calibrated_my_sensor_gt_odometry = compute_odometry_trajectory([[0,0,0];sensor_gt_rel],[6.50242e-05,-0.00354605,0.000941697 ]); % Uncomment to validate compute increments
% my_plot(calibrated_my_sensor_gt_odometry, calibrated_sensor_odometry_plot, 1, 'k-'); % Uncomment to validate compute increments

% Compute the calibration parameters
[X, chi] = calibrate([NOMINAL_PARAMS, SENSOR_TRANSLATION_WRT_ROBOT], [absolute_values, incremental_values_rel, sensor_gt_rel], N_ITERATIONS, ENCODER_MAX_VALUES, DAMPING, plot_, save_gif);

calibrated_my_robot_odometry = compute_odometry_trajectory(stack_odometry(X(1:4), [absolute_values, incremental_values_rel], ENCODER_MAX_VALUES), t2v(inv(v2t(X(5:7)))));
calibrated_my_sensor_odometry = compute_sensor_odometry(calibrated_my_robot_odometry, X(5:7));

if plot_

    %h5 = my_plot(sensor_gt_values, calibrated_sensor_odometry_plot, 5, 'r-');
    h6 = my_plot(calibrated_my_sensor_odometry, calibrated_sensor_odometry_plot, 2, 'g-');
    legend([h5 h6], {'Sensor GT', "My sensor odometry"}, 'Location', 'northwest');

    h7 = my_plot([[1:N_ITERATIONS]',chi'], chi_plot, 2, 'k-');

endif

if (plot_)
    disp("Press anything to exit");
    pause;

endif
