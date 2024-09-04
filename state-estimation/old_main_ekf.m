clear
clc

%%%%%%%%%%%%%%% INPUT FILES %%%%%%%%%%%%%%%
trajectory_file = '../../trajectory-modeling/altitude_traj.csv';

%%%%%%%%%%%%%%%%%%% Define attitude struct (atd) %%%%%%%%%%%%%%%%%%%%%
atd.q = [0.7071;0;0.7071;0];

%%%%%%%%%% Define EKF struct (ekf) %%%%%%%%%%%%%%

%% Dimensions
ekf.nx = 2; % number of states
ekf.nz = 1; % number of measurements
ekf.nu = 0; % number of control inputs

%% Core variables
ekf.x_n = zeros([ekf.nx,1]); % current state x_n,n (nx by 1)
ekf.u = 0; % control input u_n (nu by 1)
ekf.G = 0; % control matrix G (nx by nu)
ekf.z = zeros([ekf.nz,1]); % measurement z_n (nz by 1)

ekf.x_prev = zeros([ekf.nx,1]); % previously predicted state x_n,n-1 (nx by 1)
ekf.P_prev = zeros(ekf.nx); % previously predicted estimate covariance P_n,n-1 (nx by nx)

ekf.x_next = zeros([ekf.nx,1]); % predicted future state x_n+1,n (nx by 1)
ekf.P_next = zeros(ekf.nx); % predicted future estimate covariance P_n+1,n (nx by nx)

%% Intermediate variables
ekf.K_n = zeros([ekf.nx,ekf.nz]); % kalman gain Kn (nx by nz)
ekf.f = zeros([ekf.nx,1]);% state transition fn (nx by 1)
ekf.dfdx = zeros([ekf.nx,ekf.nx]); % linearized state transition fn (nx by nx)
ekf.h = zeros([ekf.nz,1]); % observation fn (nz by 1)
ekf.dhdx = zeros([ekf.nz,ekf.nx]); % linearized observation fn (nz by nx)

%% Plotting variables
%TODO call function that auto gens csv files
truth_data = readmatrix(trajectory_file); % t velo alt
ekf.t = 0; % current time dependent on csv data
ekf.time_step = 0; % variable time step dependent on csv data
[ekf.loop_size ~] = size(truth_data); % dependent on csv data

% EKF history struct
ekf_history.x_n = zeros(ekf.loop_size,ekf.nx);
ekf_history.x_true = truth_data(:,2:4); % true posn velo accelo (pregenerated from trajectory generation model)
ekf_history.t = truth_data(:,1);

%% Define sensor struct

% Sensor parameters
sensor.gps.stdev = 5;
sensor.accelerometer.stdev = 10;
sensor.barometer.stdev = 0.1;
% sensor.magnetometer.stdev = 0.1;

% Generate sensor data/history
sensor.gps.history = normrnd(truth_data(:,2),sensor.gps.stdev); % history of gps measurements
sensor.accelerometer.history = normrnd(truth_data(:,4),sensor.accelerometer.stdev); % history of accelerometer measurements
sensor.barometer.history = normrnd(truth_data(:,2),sensor.barometer.stdev); % history of barometer measurements
% sensor.magnetometer.history = ?

% EKF sensors
ekf.gps = 0; % current gps measurement
ekf.accelerometer = 0; % current accelerometer measurement
ekf.barometer = 0; % current barometer measurement
ekf.magnetometer = 0; % current magnetometer measurement

%% Covariances

% current estimate covariance P_n,n (nx by nx)
ekf.P_n = 1*eye(ekf.nx);

% process noise covariance Q (nx by nx)
ekf.Q = 2 * [0 0;
                0 1];
% ekf.Q = 0.01 *[1 0 0 0 0 0 0 0 0 0 0 0 0;
%                 0 1 0 0 0 0 0 0 0 0 0 0 0;
%                 0 0 1 0 0 0 0 0 0 0 0 0 0;
%                 0 0 0 1 0 0 0 0 0 0 0 0 0;
%                 0 0 0 0 1 0 0 0 0 0 0 0 0;
%                 0 0 0 0 0 1 0 0 0 0 0 0 0;
%                 0 0 0 0 0 0 0 0 0 0 0 0 0;
%                 0 0 0 0 0 0 0 0 0 0 0 0 0;
%                 0 0 0 0 0 0 0 0 0 0 0 0 0;
%                 0 0 0 0 0 0 0 0 0 0 0 0 0;
%                 0 0 0 0 0 0 0 0 0 0 0 0 0;
%                 0 0 0 0 0 0 0 0 0 0 0 0 0;
%                 0 0 0 0 0 0 0 0 0 0 0 0 0];

% measurement covariance R (nz by nz)
ekf.R = (sensor.gps.stdev^2)*eye(ekf.nz);


%% %%%%%%%%%%%% Main loop %%%%%%%%%%%%%

% Run EKF loop
for i = 1:ekf.loop_size

    % Time step
    if i > 1
        ekf.time_step = ekf.t - ekf_history.t(i-1);
    end

    % Read sensor data (simulates calling driver fns)
    ekf.gps = sensor.gps.history(i);
    ekf.accelerometer = sensor.accelerometer.history(i);
    ekf.barometer = sensor.barometer.history(i); % if gps read fails, decrease barometer covar?
    

    %% EKF
    % Measurement function
    ekf.z = ekf.gps;

    % Update
    ekf.h = observation_function(ekf);
    ekf.dhdx = observation_jacobian(ekf);
    ekf.K_n = kalman_gain(ekf);
    ekf.x_n = update_state(ekf);
    ekf.P_n = update_covariance(ekf);

    % Predict
    ekf.f = state_transition_function(ekf);
    ekf.dfdx = state_transition_jacobian(ekf);
    ekf.x_next = predict_state(ekf);
    ekf.P_next = predict_covariance(ekf);

    %% Attitude estimation

    
    
    %% Move to next time step
    if i < ekf.loop_size
        ekf.t = ekf_history.t(i+1);
    end
    ekf.x_prev = ekf.x_next;
    ekf.P_prev = ekf.P_next;


    % Update ekf history for plotting
    %ekf_history.t(i) = ekf.t; % already pregened
    ekf_history.x_n(i,:) = ekf.x_n';
    ekf_history.z(i) = ekf.z;

end

%% Plotting
figure

plot(ekf_history.t,ekf_history.x_true(:,1),"LineWidth",2)
hold on
plot(ekf_history.t,ekf_history.z(:),"LineWidth",2)
hold on
plot(ekf_history.t,ekf_history.x_n(:,1),"LineWidth",2)

legend('true','gps','estimate')

set(gca,'fontsize',14);
grid on
title('EKF')
xlabel('Time (seconds)')
ylabel('State')
% xlim([0,10])
