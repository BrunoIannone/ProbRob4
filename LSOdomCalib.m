close all
clear
clc

#import 2d geometry utils
source "./tools/utilities/geometry_helpers_2d.m"
source "./tools/utilities/J_numeric.m"

#addpath "./exercise/"
pkg load quaternion
pkg load mapping
plot_ = true;
if(plot_)
    h = figure(1);
endif

more off;
#load the calibration matrix
sensor_translation_wrt_robot = [1.5,0,0]

disp('Loading data matrix');
Z=load("./data/dataset_octave.txt");
disp('Assign data');

nominal_params = [0.1 0.0106141 0 1.4];
global encoder_max_values;
encoder_max_values = [8192 5000];
max_incremental_variable = 2^32;

absolute_values = Z(100:2400,2);
incremental_values = Z(100:2400,3);
robot_odometry_values = Z(100:2400,4:6);
sensor_gt_values = Z(100:2400,7:9);
sensor_odometry_values=compute_sensor_odometry(robot_odometry_values,sensor_translation_wrt_robot);

%rot = quaternion(config(4,1:4)')
r_T_l = [[1,0,1.5];
         [0,1, 0 ];
         [0,0, 1 ];
         ];
#compute the ground truth trajectory
if(plot_)
    %disp("Display robot odometry")

    % h1 = plot(robot_odometry_values(:,1),robot_odometry_values(:,2),'b-', 'linewidth', 5);
    % hold on;

    disp("Display sensor GT")
    h2 = plot(sensor_gt_values(:,1),sensor_gt_values(:,2),'r-', 'linewidth',2);

    hold on;

    % disp("Display sensor odometry")
    % h3 = plot(sensor_odometry_values(:,1),sensor_odometry_values(:,2),'y-', 'linewidth', 5);

    % hold on;
    
    %legend([h1 h2 h3], {'Robot odometry', 'Sensor GT', 'Sensor odometry'});

endif

incremental_values_rel= get_relative_ticks(incremental_values,max_incremental_variable);
    
my_robot_odometry = stack_odometry(nominal_params,[absolute_values(1:size(absolute_values,1)-1,:),incremental_values_rel], encoder_max_values);

% disp("Display my robot odometry")
% if(plot_)
%     h4 = plot(my_robot_odometry(:,1),my_robot_odometry(:,2), 'g-', 'linewidth', 2);
%     %legend([h1 h2 h3 h4], {'Robot odometry', 'Sensor GT', 'Sensor odometry', "My robot odometry"});
%     hold on;
% endif

my_sensor_odometry = compute_sensor_odometry(my_robot_odometry,sensor_translation_wrt_robot);
if(plot_)
    
    h5 = plot(my_sensor_odometry(:,1),my_sensor_odometry(:,2), 'k-', 'linewidth', 2);
    hold on;
    %legend([h1 h2 h3 h4 h5], {'Robot odometry', 'Sensor GT', 'Sensor odometry', "My robot odometry", "My sensor odometry"});

endif
skipped_meas = skip_measurements(sensor_gt_values);
h6 = plot(skipped_meas(:,1),skipped_meas(:,2), 'g.', 'linewidth', 2);
    hold on;
disp('computing calibration parameters');
skipped_absolute_values = skip_measurements(absolute_values(1:size(absolute_values,1)-1,:));
skipped_incremental_values_rel = skip_measurements(incremental_values_rel);
skipped_sensor_gt = skip_measurements(sensor_gt_values(1:size(absolute_values,1)-1,:));
skipped_my_robot_odometry = skip_measurements(my_robot_odometry)

% %compute the calibration parameters
[X,chi]=oneRound([nominal_params,1.5,0,0],[skipped_absolute_values,skipped_incremental_values_rel,skipped_sensor_gt],skipped_my_robot_odometry);
X
calibrated_my_robot_odometry = stack_odometry(X(1:4),[absolute_values(1:size(absolute_values,1)-1,:),incremental_values_rel], encoder_max_values);
calibrated_my_sensor_odometry = compute_sensor_odometry(calibrated_my_robot_odometry,X(5:7));
h5 = plot(calibrated_my_sensor_odometry(:,1),calibrated_my_sensor_odometry(:,2), 'g-', 'linewidth', 2);
    hold on;

if plot_

    j = figure(2);
    j1 = plot(chi(:), 'k-', 'linewidth', 2);
        hold on;
endif

if(plot_)
    waitfor(h);
endif