close all
clear
clc
more off;

#import 2d geometry utils
source "./tools/utilities/geometry_helpers_2d.m"
source "./tools/utilities/J_numeric.m"

pkg load quaternion
pkg load mapping

#### Flags ####
plot_ = true;
############

#### Load data matrix ####
disp('Loading data matrix');
Z = load("./data/dataset_octave.txt");

#### Assign data ####
disp('Assign data');
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
################################

if (plot_) % Init figures

    robot_odometry_plot = init_figure(1, 'Robot Odometry', 'World x', 'World y');
    sensor_odometry_plot = init_figure(2, 'Sensor Odometry', 'World x', 'World y');
    calibrated_sensor_odometry_plot = init_figure(3, 'Calibrated sensor Odometry', 'World x', 'World y');

endif

incremental_values_rel = get_relative_ticks(incremental_values, MAX_INCREMENTAL_VARIABLE);
incremental_values_rel(size(incremental_values_rel, 1) + 1) = zeros(1, size(incremental_values_rel, 2)); % Padding

my_robot_odometry = compute_odometry_trajectory(NOMINAL_PARAMS, [absolute_values, incremental_values_rel], ENCODER_MAX_VALUES);

if (plot_)
    h1 = my_plot(robot_odometry_values, robot_odometry_plot, 5, 'r-');
    h2 = my_plot(my_robot_odometry, robot_odometry_plot, linewidth = 2, 'g-');
    legend([h1 h2], {'Robot odometry', "My robot odometry"});
endif

sensor_odometry_values = compute_sensor_odometry(robot_odometry_values, SENSOR_TRANSLATION_WRT_ROBOT);
my_sensor_odometry = compute_sensor_odometry(my_robot_odometry, SENSOR_TRANSLATION_WRT_ROBOT);

if (plot_)
    h3 = my_plot(sensor_odometry_values, sensor_odometry_plot, 5, 'r-');
    h4 = my_plot(my_sensor_odometry, sensor_odometry_plot, linewidth = 2, 'g-');
    legend([h3 h4], {'Sensor odometry', "My robot odometry"});
endif

if (plot_)

    h5 = my_plot(sensor_gt_values, calibrated_sensor_odometry_plot, 5, 'r-');
    legend([h5], {'Calibrated sensor odometry'}); %, "My robot odometry"});

endif

%skip_value = 5
% n_iterations = 30
% skipped_meas = skip_measurements(sensor_gt_values, skip_value = skip_value);
% h6 = plot(skipped_meas(:,1),skipped_meas(:,2), 'g.', 'linewidth', 2);
%     hold on;
% disp('computing calibration parameters');

%skipped_absolute_values = skip_measurements(absolute_values(1:size(absolute_values,1)-1,:),skip_value = skip_value);

% skipped_absolute_values = skip_measurements(absolute_values,skip_value = skip_value);
%skipped_incremental_values_rel = skip_measurements(incremental_values_rel, skip_value = skip_value);
% skipped_sensor_gt = skip_measurements(sensor_gt_values,skip_value = skip_value);
% skipped_my_robot_odometry = skip_measurements(my_robot_odometry,skip_value = skip_value);

% % %compute the calibration parameters
% [X,chi]=oneRound([nominal_params,1.5,0,0],[skipped_absolute_values,skipped_incremental_values_rel,skipped_sensor_gt],skipped_my_robot_odometry, n_iterations);
% X
% calibrated_my_robot_odometry = stack_odometry(X(1:4),[absolute_values,incremental_values_rel], encoder_max_values);
% calibrated_my_sensor_odometry = compute_sensor_odometry(calibrated_my_robot_odometry,X(5:7));
% h5 = plot(calibrated_my_sensor_odometry(:,1),calibrated_my_sensor_odometry(:,2), 'g-', 'linewidth', 2);
%     hold on;

% if plot_

%     j = figure(2);
%     j1 = plot(chi(:), 'k-', 'linewidth', 2);
%         hold on;
% endif

if (plot_)
    disp("Press Enter to exit");
    pause;

endif
