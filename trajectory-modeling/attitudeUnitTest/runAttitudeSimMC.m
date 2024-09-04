clear;clc;close all

%% DEFINE CONFIG

mcConfig.runClosedLoop = 1; % 1 = run LQR control
                               % 0 = run open loop
mcConfig.endTime = 12;
mcConfig.tStep = 0.01;
mcConfig.simHz = 1/mcConfig.tStep;
mcConfig.tSpan = 0:mcConfig.tStep:mcConfig.endTime;
mcConfig.refAtd = ones( length(mcConfig.tSpan),3) .* [0 0 0];

% Get MC param xlsx file name
mcConfig.xlsxFile = 'dummyAttitudeParams.xlsx';

%% RUN MONTE CARLOS

numRuns = 10;
mcLog = solveAttitudeMonteCarlos(mcConfig,numRuns,0);