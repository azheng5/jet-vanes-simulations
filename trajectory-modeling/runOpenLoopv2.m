% Better version of run open loop, includes final fitted aero model, monte carlos, etc.
% run_open_loop.m to be deleted soon

clear;clc;close all

%% CONFIG

fprintf("Configuring open loop simulation...\n");

mcConfig.numStates = 13;
mcConfig.numInputs = 3;

mcConfig.tStep = 0.01;
mcConfig.endTime = 20;
mcConfig.simHz = 1/mcConfig.tStep;
mcConfig.tSpan = 0:mcConfig.tStep:mcConfig.endTime;

mcConfig.initState = [0; % px
                      0; % py
                      0; % pz
                      5.729; % vx
                      0.1; % vy
                      0; % vz
                      1; % q0
                      0; % q1
                      0; % q2
                      0; % q3
                      0; % wx
                      0; % wy
                      0]; % wz
% TODO replace with updated thrust profile (in sharepoint)
mcConfig.wetMass = 38.87; % mass with loaded motor (kg)
mcConfig.dryMass = 30.96; % mass with emptied motor (kg)
mcConfig.CG0 = 2.433; % initial CG from tip of nose (m)
mcConfig.finalCG = 2.34; % final CG from tip of nose (m)
mcConfig.length = 3.4; % length of rocket (m)
mcConfig.diameter = 0.203; % m
mcConfig.radius = mcConfig.diameter/2;
mcConfig.gravity = 9.81; % m/s^2
mcConfig.wetMOI= [0.2109 0 0;
                  0 21.1212 0;
                  0 0 21.1202];
mcConfig.dryMOI = [0.2036 0 0;
                   0 18.99 0;
                   0 0 18.988];
% mcConfig.wetIb = [21.1212 -0.0046 -0.0005;
%                   -0.0046 0.2109 0.015;
%                   -0.0005 0.015 21.1202];
% mcConfig.dryIb = [18.99 -0.0038 -0.0005;
%                   -0.0038 0.2036 0.0118;
%                   -0.0005 0.0118 18.988];
% mcConfig.baseIb = [0.00077421 0 0;
%               0 0.0626 0;
%               0 0 0.0626];
% mcConfig.invBaseIb = [1291.6 0 0;
%                  0 16.0 0;
%                  0 0 16.0];
mcConfig.g = 9.81;
mcConfig.peakThrust = 2262; % N
mcConfig.burnTime = 13.1;
mcConfig.thrustCurve = generate_thrust_curve(mcConfig.peakThrust, mcConfig.burnTime); %2000 N T_max, 13 seconds burn time


% Use impulses
impulse_x = generateImpulse(0,0.0,20,2,mcConfig.tSpan);
impulse_x(1) = 0; % if impulse at t-0.0, huge discontinuity occurs
impulse_y = generateImpulse(0.0,0.00,10,200,mcConfig.tSpan);
impulse_z = generateImpulse(5,0.00,10,2,mcConfig.tSpan);
impulse_z(1) = 0;
impulse = [impulse_x impulse_y impulse_z];
% mcConfig.uSpan = impulse;

mcConfig.uSpan = zeros(length(mcConfig.tSpan),3);

mcConfig.options = odeset('Events',@rocket_events);

mcConfig.xlsxFile = 'openLoopParams.xlsx';

%% RUN MONTE CARLOS

% Set to 1 if running singular simulation
numRuns = 4;
calcXdots = 0; % speeds up sims if turned off, but loses some plotting ability
log = solveOpenLoop(mcConfig,numRuns,calcXdots);