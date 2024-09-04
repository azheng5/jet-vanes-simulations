clear;clc;close all

%% DEFINE CONFIG

atdConfig.runClosedLoop = 1; % 1 = run LQR control
                             % 0 = run open loop
atdConfig.Ixx = 0.00077421;
atdConfig.Iyy = 0.0626;
atdConfig.Izz = 0.0626;
atdConfig.K = loadAttitudeLQR(atdConfig.Ixx, atdConfig.Iyy, atdConfig.Izz, ...
                              0,0,0);
atdConfig.initTrueAtd = [1;0;0;0;0;0;0;0;0;0];
atdConfig.initAtd = [0;0;0;0;0;0];

atdConfig.endTime = 5;
atdConfig.tStep = 0.01;
atdConfig.simHz = 1/atdConfig.tStep;
atdConfig.tSpan = 0:atdConfig.tStep:atdConfig.endTime;
atdConfig.refAtd = ones( length(atdConfig.tSpan),3) .* [0 0 0];

% Use impulses
impulse_x = generateImpulse(0,0.001,20,2,atdConfig.tSpan);
impulse_x(1) = 0; % if impulse at t-0.0, huge discontinuity occurs
impulse_y = generateImpulse(0,0.00,30,200,atdConfig.tSpan);
impulse_z = generateImpulse(0,0.00,10,200,atdConfig.tSpan);
impulse = [impulse_x impulse_y impulse_z];
atdConfig.uSpan = impulse;

% atdConfig.uSpan = zeros(length(atdConfig.tSpan),3);

%% RUN SIM ONCE
% fprintf('Simulating nonlinear attitude trajectory...\n')
% [trueAtdT, trueAtdX] = ode45(@(t,x) attitudeNonlinModel(t,x,atdConfig), ...
%                              atdConfig.tSpan, ...
%                              atdConfig.initTrueAtd);
% trueAtdXdot = zeros(length(trueAtdT),10);
% for i = 1:length(trueAtdT)
%     trueAtdXdot(i,:) = attitudeNonlinModel(trueAtdT(i),trueAtdX(i,:)',atdConfig).';
% end

fprintf('Simulating linearized attitude trajectory...\n')
[atdT, atdX] = ode45(@(t,x) attitudeLinModel(t,x,atdConfig), ...
                     atdConfig.tSpan, ...
                     atdConfig.initAtd);
atdXdot = zeros(length(atdT),6);
for i = 1:length(atdT)
    atdXdot(i,:) = attitudeLinModel(atdT(i),atdX(i,:)',atdConfig).';
end

fprintf('Done!')


%% LOG OUTPUT
% log.true.q_bf = trueAtdX(:,1:4);
% log.true.angVel = trueAtdX(:,5:7);
% log.true.angAccel = trueAtdXdot(:,5:7);
% log.true.time = trueAtdT;

log.linearized.q_bf = [ones(length(atdT),1) , atdX(:,1:3)];
log.linearized.angVel = atdX(:,4:6);
log.linearized.angAccel = atdXdot(:,4:6);
log.linearized.time = atdT;