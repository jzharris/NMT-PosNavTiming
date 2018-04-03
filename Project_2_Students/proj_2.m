%% Script/Function Description:
%   Main script which generates the probabilities and calls the IMU functions
%
% NOTES:
%   - Matrix(row, col, time)
%   - Vector(row, time)
%
% REFERENCES:
%   Navigation Systems textbook by P. Groves
%
clear all   % Remove all items from workspace - release system memory
close all   % Close all open figures (i.e. plots)
clc         % Clear the command window

load_constants;                     % Load a file of constants

Fs = constants.Fs;                  % Sample frequency (Hz)
dt = constants.dt;                  % Sample interval (sec)

%==============================================================================
% Generate the probabilities concerning bias terms
%==============================================================================
constants.sigma_BI = degtorad(0.012);   % STd. dev of BI: taken from datasheet
constants.T_c = 3600;                   % Correlation time in seconds
constants.Q = 2 * sigma_BI^2 / T_c;

