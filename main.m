clear;
clc;
close all;

%% Project setup

projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(projectRoot));

%% Vehicle parameters

P = vehicle_params();

disp(P)