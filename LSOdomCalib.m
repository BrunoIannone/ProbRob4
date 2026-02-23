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

SAMPLE_VALUE = 5;
N_ITERATIONS = 10;
################################

if (plot_) % Init figures

    robot_odometry_plot = init_figure(1, 'Robot Odometry', 'World x', 'World y');
    sensor_odometry_plot = init_figure(2, 'Sensor Odometry', 'World x', 'World y');
    calibrated_sensor_odometry_plot = init_figure(3, 'Calibrated sensor Odometry', 'World x', 'World y');

endif

incremental_values_rel = compute_relative_ticks(incremental_values, MAX_INCREMENTAL_VARIABLE);
%incremental_values_rel(size(incremental_values_rel, 1) + 1) = zeros(1, size(incremental_values_rel, 2)); % Padding

absolute_values = absolute_values(1:size(absolute_values,1)-1);
my_robot_odometry = compute_odometry_trajectory_corso(stack_odometry(NOMINAL_PARAMS, [absolute_values, incremental_values_rel], ENCODER_MAX_VALUES));
if (plot_)
    h1 = my_plot(robot_odometry_values, robot_odometry_plot, 5, 'r-');
    h2 = my_plot(my_robot_odometry, robot_odometry_plot, linewidth = 2, 'k-');

    %legend([h1 h2], {'Robot odometry', "My robot odometry"});
endif

sensor_odometry_values = compute_sensor_odometry(robot_odometry_values, SENSOR_TRANSLATION_WRT_ROBOT);
my_sensor_odometry = compute_sensor_odometry(my_robot_odometry, SENSOR_TRANSLATION_WRT_ROBOT);

if (plot_)
    h3 = my_plot(sensor_odometry_values, sensor_odometry_plot, 5, 'r-');
    h4 = my_plot(my_sensor_odometry, sensor_odometry_plot, linewidth = 2, 'k-');

    %legend([h3 h4], {'Sensor odometry', "My robot odometry"});
endif

if (plot_)

    h5 = my_plot(sensor_gt_values, calibrated_sensor_odometry_plot, 5, 'r-');
    %legend([h5], {'Calibrated sensor odometry'}); %, "My robot odometry"});

endif


% norm(diff(sensor_gt_values(:,1:2)),2,"rows")

% sampled_calibrated_sensor = sample_data(sensor_gt_values, sample_value = SAMPLE_VALUE);

% sensor_gt_values_rel = diff(sampled_calibrated_sensor);

% for (i = 1:size(sensor_gt_values_rel,1))
%     sensor_gt_values_rel(i:3) = normalizeAngle(sensor_gt_values_rel(i:3));
% endfor

% sampled_my_sensor_odometry = sample_data(my_sensor_odometry,sample_value = SAMPLE_VALUE);
% size(sampled_calibrated_sensor)
% %sampled_robot_odometry = sample_data(robot_odometry_values,sample_value = SAMPLE_VALUE);
% sampled_my_robot_odometry = sample_data(my_robot_odometry,sample_value = SAMPLE_VALUE);

% sampled_absolute_values = sample_data(absolute_values,sample_value = SAMPLE_VALUE);
% size(sampled_absolute_values)
% sampled_incremental_values_rel = sample_data(incremental_values_rel, sample_value = SAMPLE_VALUE);
% size(sampled_incremental_values_rel)
% if(plot_)

%     h6 = my_plot(sampled_calibrated_sensor,calibrated_sensor_odometry_plot, 2, 'y.');
%     h7 = my_plot(sampled_calibrated_sensor,calibrated_sensor_odometry_plot, 2, 'g-');

%     %disp('computing calibration parameters');


%     h7 = my_plot(sampled_my_robot_odometry,robot_odometry_plot, 10, 'y.');

%     h8 = my_plot(sampled_my_sensor_odometry,sensor_odometry_plot, 10, 'y.');
% endif

% Compute the calibration parameters

% [X,chi]=oneRound([NOMINAL_PARAMS,SENSOR_TRANSLATION_WRT_ROBOT],[sampled_absolute_values,sampled_incremental_values_rel,sampled_calibrated_sensor], N_ITERATIONS,ENCODER_MAX_VALUES);
% calibrated_my_robot_odometry = compute_odometry_trajectory(X(1:4),[absolute_values,incremental_values_rel], ENCODER_MAX_VALUES);
% calibrated_my_sensor_odometry = compute_sensor_odometry(calibrated_my_robot_odometry,X(5:7));
% h10 = my_plot(calibrated_my_sensor_odometry,calibrated_sensor_odometry_plot, 2, 'k-');

% h5 = plot(calibrated_my_sensor_odometry(:,1),calibrated_my_sensor_odometry(:,2), 'g-', 'linewidth', 2);
%     hold on;

% if plot_

%     j = figure(4);
%     j1 = plot(chi(:), 'k-', 'linewidth', 2);
%         hold on;
% endif

if (plot_)
    disp("Press anything to exit");
    pause;

endif
