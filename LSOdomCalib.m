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

encoder_max_values = [8192 5000];
max_incremental_variable = 2^32;

absolute_values = Z(:,2);
incremental_values = Z(:,3);
robot_odometry_values = Z(:,4:6);
sensor_gt_values = Z(:,7:9);
sensor_odometry_values=compute_sensor_odometry(robot_odometry_values,sensor_translation_wrt_robot);

%rot = quaternion(config(4,1:4)')
r_T_l = [[1,0,1.5];
         [0,1, 0 ];
         [0,0, 1 ];
         ];
#compute the ground truth trajectory
if(plot_)
    disp("Display robot odometry")

    h1 = plot(robot_odometry_values(:,1),robot_odometry_values(:,2),'b-', 'linewidth', 5);
    hold on;

    disp("Display sensor GT")
    h2 = plot(sensor_gt_values(:,1),sensor_gt_values(:,2),'r-', 'linewidth',2);

    hold on;

    disp("Display sensor odometry")
    h3 = plot(sensor_odometry_values(:,1),sensor_odometry_values(:,2),'y-', 'linewidth', 5);

    hold on;
    
    %legend([h1 h2 h3], {'Robot odometry', 'Sensor GT', 'Sensor odometry'});

endif

incremental_values_rel= get_relative_ticks(incremental_values,max_incremental_variable);
    
my_robot_odometry = stack_odometry([ 0.1, 0.0106141, 0,1.4],[absolute_values(1:size(absolute_values,1)-1,:),incremental_values_rel], encoder_max_values);

disp("Display my robot odometry")
if(plot_)
    h4 = plot(my_robot_odometry(:,1),my_robot_odometry(:,2), 'g-', 'linewidth', 2);
    %legend([h1 h2 h3 h4], {'Robot odometry', 'Sensor GT', 'Sensor odometry', "My robot odometry"});
    hold on;
endif

my_sensor_odometry = compute_sensor_odometry(my_robot_odometry,sensor_translation_wrt_robot);
if(plot_)
    
    h5 = plot(my_sensor_odometry(:,1),my_sensor_odometry(:,2), 'k-', 'linewidth', 2);
    hold on;
    legend([h1 h2 h3 h4 h5], {'Robot odometry', 'Sensor GT', 'Sensor odometry', "My robot odometry", "My sensor odometry"});

endif

disp('computing calibration parameters');
% %compute the calibration parameters
% [X,chi]=oneRound(nominal_params,[absolute_values(1:size(absolute_values,1)-1,:),incremental_values_rel,Z(1:2433,7:9)]')
% my_robot_odometry2 = stack_odometry(X',[absolute_values(1:size(absolute_values,1)-1,:),incremental_values_rel], encoder_max_values);
% h6 = plot(my_robot_odometry2(:,1),my_robot_odometry2(:,2), 'r-', 'linewidth', 10);

%X
% pause(1);

% disp('computing calibrated odometry');
% #COdosm=apply_odometry_correction(X,odom);
% COdom = stack_odometry(X,Z(:,1:2));

% CalTrajectory=compute_odometry_trajectory(COdom);
% hold on;
% plot(CalTrajectory(:,1),CalTrajectory(:,2), 'b-', 'linewidth', 2);
if(plot_)
    waitfor(h);
endif