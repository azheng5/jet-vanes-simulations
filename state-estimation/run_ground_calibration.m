clear;clc;close all

%% Load simulation inputs
define_sensor_specs;
num_samples = 1000;
IMU.SampleRate = 10;
acceleration = zeros(num_samples,3);
angVel = zeros(num_samples,3);

[accel_reading, gyro_reading, mag_reading] = IMU(acceleration,angVel);
times = (0:(num_samples-1))/IMU.SampleRate;

%% %%% SETUP %%%

% Initialize EKFs
x_n0 = [0;0;0;0;0;0];
P_n0 = 10*eye(6);
Q = 0.1*eye(6);
q1 = 0.1;
q2 = 0.1;
Q = [q1 0 0 0 0 0;
     0 q1 0 0 0 0;
     0 0 q1 0 0 0;
     0 0 0 q2 0 0;
     0 0 0 0 q2 0;
     0 0 0 0 0 q2];
R(1:3,1:3) = 0.01*ones(3);
R(4:6,4:6) = 0.01*ones(3);
R(4:6,1:3) = 0*ones(3);
R(1:3,4:6) = 0*ones(3);
[gekf, gekf_log] = make_ekf_struct(6,6,x_n0,P_n0,Q,R,num_samples);
          
% Add EKF sensors
gekf.accelerometer = 0; % current accelerometerometer measurement
gekf.gyro = 0;
gekf.magneto = 0;

%% %%%%%%%%%%%% Main loop %%%%%%%%%%%%%

% Run EKF loop
for i = 1:gekf.loop_size

    % Calculate time step
    gekf.t = times(i);
    gekf.dt = 1/IMU.SampleRate;

    % Read sensor data (simulates calling driver fns)
    gekf.accelerometer = accel_reading(i,:);
    gekf.gyro = gyro_reading(i,:);
    gekf.magneto = mag_reading(i,:);

    %% Ground EKF step

    % Measurement
    gekf.z = [gekf.accelerometer(1);
              gekf.accelerometer(2);
              gekf.accelerometer(3);
              gekf.gyro(1);
              gekf.gyro(2);
              gekf.gyro(3)];

    % Update
    gekf.h = observation_fn(gekf);
    gekf.dhdx = observation_jacob(gekf);
    gekf.K_n = kalman_gain(gekf);
    gekf.x_n = update_state(gekf);
    gekf.P_n = update_covariance(gekf);

    % Predict
    gekf.f = state_transition_fn(gekf);
    gekf.dfdx = state_transition_jacob(gekf);
    gekf.x_next = predict_state(gekf);
    gekf.P_next = predict_covariance(gekf);
    
    gekf.x_prev = gekf.x_next;
    gekf.P_prev = gekf.P_next;


    %% Data logging
    gekf_log.t(i) = gekf.t;
    gekf_log.P_n(i,1:gekf.nx,1:gekf.nx) = gekf.P_n;
    gekf_log.x_n(i,:) = gekf.x_n';
    gekf_log.z(i,:) = gekf.z';

end

%% Store sensor compensation results
final_bias.accel_bias = gekf_log.x_n(end,1:3);
final_bias.gyro_bias = gekf_log.x_n(end,4:6);

%% Plot generation
figure(1)

subplot(2,1,1)
plot(gekf_log.t,gekf_log.x_n(:,1))
hold on
plot(gekf_log.t,gekf_log.x_n(:,2))
hold on
plot(gekf_log.t,gekf_log.x_n(:,3))
grid on
title('Accel bias')

subplot(2,1,2)
plot(gekf_log.t,rad2deg(gekf_log.x_n(:,4)))
hold on
plot(gekf_log.t,rad2deg(gekf_log.x_n(:,5)))
hold on
plot(gekf_log.t,rad2deg(gekf_log.x_n(:,6)))
grid on
title('Gyro bias (deg)')

figure(2)

subplot(2,3,1)
plot(gekf_log.t,gekf_log.z(:,1))
hold on
plot(gekf_log.t,gekf_log.z(:,1)-gekf_log.x_n(:,1))
grid on
title('accelerometer x')

subplot(2,3,2)
plot(gekf_log.t,gekf_log.z(:,2))
hold on
plot(gekf_log.t,gekf_log.z(:,2)-gekf_log.x_n(:,2))
grid on
title('accelerometer y')

subplot(2,3,3)
plot(gekf_log.t,gekf_log.z(:,3))
hold on
plot(gekf_log.t,gekf_log.z(:,3)-gekf_log.x_n(:,3))
grid on
title('accelerometer z')
legend('raw reading','compensated reading')

subplot(2,3,4)
plot(gekf_log.t,rad2deg(gekf_log.z(:,4)))
hold on
plot(gekf_log.t,rad2deg(gekf_log.z(:,4)-gekf_log.x_n(:,4)))
grid on
title('gyro x (deg)')

subplot(2,3,5)
plot(gekf_log.t,rad2deg(gekf_log.z(:,5)))
hold on
plot(gekf_log.t,rad2deg(gekf_log.z(:,5)-gekf_log.x_n(:,5)))
grid on
title('gyro y (deg)')

subplot(2,3,6)
plot(gekf_log.t,rad2deg(gekf_log.z(:,6)))
hold on
plot(gekf_log.t,rad2deg(gekf_log.z(:,6)-gekf_log.x_n(:,6)))
grid on
title('gyro z (deg)')

figure(3)

subplot(2,1,1)
plot(gekf_log.t,gekf_log.P_n(:,1,1))
hold on
plot(gekf_log.t,gekf_log.P_n(:,2,2))
hold on
plot(gekf_log.t,gekf_log.P_n(:,3,3))
grid on
title('accelo bias covariance')

subplot(2,1,2)
plot(gekf_log.t,gekf_log.P_n(:,4,4))
hold on
plot(gekf_log.t,gekf_log.P_n(:,5,5))
hold on
plot(gekf_log.t,gekf_log.P_n(:,6,6))
grid on
title('gyro bias covariance')


%% Local functions %%
function f = state_transition_fn(ekf)
    % Calculates next state for ground EKF

    f = [ekf.x_n(1);
         ekf.x_n(2);
         ekf.x_n(3);
         ekf.x_n(4);
         ekf.x_n(5);
         ekf.x_n(6)];
end

function dfdx = state_transition_jacob(ekf)
    % Calculates state transition jacobian for ground EKF

    dfdx = [1,0,0,0,0,0;
            0,1,0,0,0,0;
            0,0,1,0,0,0;
            0,0,0,1,0,0;
            0,0,0,0,1,0;
            0,0,0,0,0,1];
    

end

function h = observation_fn(ekf)
    % Calculates state-to-measurement relation for translational EKF

    h = [0 + ekf.x_n(1);
         0 + ekf.x_n(2);
         9.8 + ekf.x_n(3);
         0 + ekf.x_n(4);
         0 + ekf.x_n(5);
         0 + ekf.x_n(6)];
    
end

function dhdx = observation_jacob(ekf)
    % Calculates observation jacobian for ground EKF
    
    dhdx = [1,0,0,0,0,0;
            0,1,0,0,0,0;
            0,0,1,0,0,0;
            0,0,0,1,0,0;
            0,0,0,0,1,0;
            0,0,0,0,0,1];

end