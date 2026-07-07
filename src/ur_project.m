%% RDKDC Final Project - Main Script

clc;
clear;
close all;

type = 'ur5e'; % 'ur5' or 'ur5e'
operating_mode = 'RVIZ'; % 'RVIZ' or 'REAL'

fprintf('--- UR5(e) Circular Drawing Project ---\n');
fprintf('Select the control method:\n');
fprintf('1: Inverse Kinematics (IK) Based Control\n');
fprintf('2: Resolved-Rate (RR) Based Control\n');

choice = input('Enter your choice (1 or 2): ');

if choice == 1
    disp('Starting IK-based control...');
    ik_control(type,operating_mode);
elseif choice == 2
    disp('Starting RR-based control...');
    rr_control(type,operating_mode);
else
    fprintf('Invalid selection. Please run the script again.\n');
end