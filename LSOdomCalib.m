close all
clear
clc
more off;

#import 2d geometry utils
source "./tools/utilities/geometry_helpers_2d.m"
source "./tools/utilities/J_numeric.m"

%pkg load quaternion
%pkg load mapping

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
SAMPLE_VALUE = 2;
N_ITERATIONS = 2;
################################

if (plot_) % Init figures

    robot_odometry_plot = init_figure(1, 'Robot Odometry', 'World x', 'World y');
    sensor_odometry_plot = init_figure(2, 'Sensor Odometry', 'World x', 'World y');
    calibrated_sensor_odometry_plot = init_figure(3, 'Calibrated sensor Odometry', 'World x', 'World y');
    

endif

incremental_values_rel = compute_relative_ticks([4294859756;incremental_values], MAX_INCREMENTAL_VARIABLE);

%absolute_values = absolute_values(1:size(absolute_values,1)-1);

my_robot_odometry = compute_odometry_trajectory(stack_odometry(NOMINAL_PARAMS, [absolute_values, incremental_values_rel], ENCODER_MAX_VALUES),[-1.5,0,0]);

if (plot_)
    h1 = my_plot(robot_odometry_values, robot_odometry_plot, 5, 'r-');
    h2 = my_plot(my_robot_odometry, robot_odometry_plot, linewidth = 2, 'g-');

    %legend([h1 h2], {'Robot odometry', "My robot odometry"});
endif

sensor_odometry_values = compute_sensor_odometry(robot_odometry_values, SENSOR_TRANSLATION_WRT_ROBOT);
my_sensor_odometry = compute_sensor_odometry(my_robot_odometry, SENSOR_TRANSLATION_WRT_ROBOT);
%sensor_gt_values =compute_sensor_odometry( sensor_gt_values,SENSOR_TRANSLATION_WRT_ROBOT)

if (plot_)
    h3 = my_plot(sensor_odometry_values, sensor_odometry_plot, 5, 'r-');
    h4 = my_plot(my_sensor_odometry, sensor_odometry_plot, linewidth = 2, 'g-');

    %legend([h3 h4], {'Sensor odometry', "My robot odometry"});
endif

if (plot_)

    h5 = my_plot(sensor_gt_values, calibrated_sensor_odometry_plot, 5, 'r-');

    %legend([h5], {'Calibrated sensor odometry'}); %, "My robot odometry"});

endif

sensor_gt_rel = compute_increments([[0,0,0];sensor_gt_values]); % Anchor to 0 or it won't be correct

calibrated_my_sensor_gt_odometry = compute_odometry_trajectory(sensor_gt_rel);
h8 = my_plot(calibrated_my_sensor_gt_odometry,calibrated_sensor_odometry_plot, 1, 'k-');

% Compute the calibration parameters

[X,chi]=oneRound([NOMINAL_PARAMS,SENSOR_TRANSLATION_WRT_ROBOT],[absolute_values,incremental_values_rel,sensor_gt_rel], N_ITERATIONS,ENCODER_MAX_VALUES);

calibrated_my_robot_odometry = compute_odometry_trajectory(stack_odometry(X(1:4), [absolute_values, incremental_values_rel], ENCODER_MAX_VALUES),t2v(inv(v2t(X(5:7)))));

calibrated_my_sensor_odometry = compute_sensor_odometry(calibrated_my_robot_odometry,X(5:7));



if plot_
    %h6 = my_plot(calibrated_my_robot_odometry,calibrated_sensor_odometry_plot, 2, 'm-');
    h7 = my_plot(calibrated_my_sensor_odometry,calibrated_sensor_odometry_plot, 2, 'g-');

    j = figure(4);
    j1 = plot(chi(:), 'k-', 'linewidth', 2);
        hold on;
endif

if (plot_)
    disp("Press anything to exit");
    pause;

endif
