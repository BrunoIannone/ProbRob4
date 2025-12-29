close all
clear
clc

#import 2d geometry utils
source "./tools/utilities/geometry_helpers_2d.m"
#addpath "./exercise/"
pkg load quaternion
h = figure(1);

more off;
#load the calibration matrix
sensor_translation_wrt_robot = [1.5,0,0]

disp('Loading data matrix');
Z=load("./data/dataset_octave.txt");
disp('Assing data');

nominal_params = [0.1 0.0106141 1.4 0];

encoder_max_values = [8192 5000];

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

disp("Display robot odometry")

h1 = plot(robot_odometry_values(:,1),robot_odometry_values(:,2),'b-', 'linewidth', 2);
hold on;

disp("Display sensor GT")
h2 = plot(sensor_gt_values(:,1),sensor_gt_values(:,2),'r-', 'linewidth',2);

hold on;

disp("Display sensor odometry")
h3 = plot(sensor_odometry_values(:,1),sensor_odometry_values(:,2),'y-', 'linewidth', 2);

hold on;

legend([h1 h2 h3], {'Robot odometry', 'Sensor GT', 'Sensor odometry'});
pause(1)

for (i=1:size(incremental_values,1)),
    if (i == 1),
        disp("inizio")
        pause(2)
        continue
    endif
    if((i+1)> size(incremental_values,1)),
        disp("fine")
        pause(2)
        break
    endif    
    incremental_values(i)
    incremental_values(i+1)
    mod(incremental_values(i+1) -  incremental_values(i),2^32)
    disp("-----")
endfor
nominal_params = [1.5,0];
% odom = stack_odometry(nominal_params,Z(:,1:2));
% #compute the uncalibrated odometry
% OdomTrajectory=compute_odometry_trajectory(odom);
% disp('odometry');
% hold on;
% plot(OdomTrajectory(:,1),OdomTrajectory(:,2), 'g-', 'linewidth', 2);
% pause(1);

% disp('computing calibration parameters');
%compute the calibration parameters
%[X,chi]=oneRound(nominal_params,sensor_odometry,Z(:,7:8));
%X
% pause(1);

% disp('computing calibrated odometry');
% #COdosm=apply_odometry_correction(X,odom);
% COdom = stack_odometry(X,Z(:,1:2));

% CalTrajectory=compute_odometry_trajectory(COdom);
% hold on;
% plot(CalTrajectory(:,1),CalTrajectory(:,2), 'b-', 'linewidth', 2);

waitfor(h);
