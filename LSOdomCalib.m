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
disp('Loading data matrix');
Z=load("./data/dataset_octave.txt");
disp('Assing data');

nominal_params = [0.1 0.0106141 1.4 0];

encoder_max_values = [8192 5000];

%rot = quaternion(config(4,1:4)')
r_T_l = [[1,0,1.5];
         [0,1, 0 ];
         [0,0, 1 ];
         ];
#compute the ground truth trajectory

disp("Display robot odometry")

h1 = plot(Z(:,4),Z(:,5),'b-', 'linewidth', 2);
hold on;

disp("Display sensor GT")
h2 = plot(Z(:,7),Z(:,8),'r-', 'linewidth',2);

hold on;

disp("Display sensor odometry")
SensorTrajectory=compute_sensor_odometry(Z(:,4:6),[1.5,0,0]);
h3 = plot(SensorTrajectory(:,1),SensorTrajectory(:,2),'y-', 'linewidth', 2);

hold on;

legend([h1 h2 h3], {'Robot odometry', 'Sensor GT', 'Sensor odometry'});
pause(1)



% TrueTrajectory=compute_odometry_trajectory(Z(:,4:6),Z(:,7:9));
% disp('ground truth');
% hold on;
% plot(TrueTrajectory(:,1),TrueTrajectory(:,2), 'g-', 'linewidth', 2);
% pause(1);
% nominal_params = [-1;1;0.3];
% odom = stack_odometry(nominal_params,Z(:,1:2));
% #compute the uncalibrated odometry
% OdomTrajectory=compute_odometry_trajectory(odom);
% disp('odometry');
% hold on;
% plot(OdomTrajectory(:,1),OdomTrajectory(:,2), 'g-', 'linewidth', 2);
% pause(1);

% disp('computing calibration parameters');
% #compute the calibration parameters
% [X,chi]=oneRound(nominal_params,Z');
% X
% pause(1);

% disp('computing calibrated odometry');
% #COdosm=apply_odometry_correction(X,odom);
% COdom = stack_odometry(X,Z(:,1:2));

% CalTrajectory=compute_odometry_trajectory(COdom);
% hold on;
% plot(CalTrajectory(:,1),CalTrajectory(:,2), 'b-', 'linewidth', 2);

waitfor(h);
