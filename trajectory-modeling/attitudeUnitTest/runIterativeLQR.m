clear;clc;close all

%% DEFINE CONFIG

config.runClosedLoop = 0; % 1 = run LQR control
                               % 0 = run open loop
config.endTime = 12;
config.tStep = 0.01;
config.simHz = 1/config.tStep;
config.tSpan = 0:config.tStep:config.endTime;
config.refAtd = ones( length(config.tSpan),3) .* [0 0 0];

% Get MC param xlsx file name
config.xlsxFile = '?';

%% RUN MONTE CARLOS

numRuns = 10;
log = solveIterativeLQR(config,numRuns,0);