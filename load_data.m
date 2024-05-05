% Load telemetry data from excell
clear all
clc

% Defining file path
file_path = 'C:\Users\franc\Desktop\Vehicle_Dynamics_project\telemetrie_2012_per_2023.xls';

% Defining the variables where the data are saved
[num_data] = xlsread(file_path);

time = num_data(:,1);       % [time] = s
dist = num_data(:,2);       % [dist] = m
speed = num_data(:,3);      % [speed] = km/h
