%% NOT WORKING RIGHT NOW

%% Load simulation inputs%%

% True trajectory
trajectory_file = '../data/altitude_traj.csv';
trajectory = readmatrix(trajectory_file); % use csv file

% Generate sensor data
define_sensor_specs;
[position_reading,velocity_reading,groundspeed,course] = GPS(true_pos,true_velo);
[accel_reading, gyro_reading, mag_reading] = IMU(acceleration,angVel);

% TODO add logic that uses the csv trajectory if unit testing run_in_flight_ekf, but the trajectory struct
% gen'ed by trajectory generator if doing integration testing
% idea create global flag struct that signifies if its integration or unit testing

% atd.q = [0.7071;0;0.7071;0];

%%% SETUP %%%

% Initialize EKFs
tekf = make_ekf_struct();
atdf = make_atdf_struct();

% General algorithm params
%TODO call function that auto gens csv files
tekf.t = 0; % current time
tekf.t_prev = 0; % previous time

atdf.t = 0;
atdf.t_prev = 0;

tekf.dt = 0; % current variable time step
atdf.f.dt = 0;
[tekf.loop_size ~] = size(trajectory);

% Add EKF sensors
tekf.gps = 0; % current gps measurement
tekf.accelerometer = 0; % current accelerometerometer measurement
atdf.gyro = 0;

% Define sensor model (stdev for now)
tekf.gps_config = 5;
tekf.accelerometer_config = 10;
tekf.gyro_config = 0.1;

% Pre allocate memory for data logging
tekf_log.x_n = zeros(tekf.loop_size,tekf.nx);

%% %%%%%%%%%%%% Main loop %%%%%%%%%%%%%

% Run EKF loop
for i = 1:tekf.loop_size

    % Calculate time step
    tekf.t = trajectory(i,1);
    atdf.t = trajectory(i,1);
    if i > 1
        tekf.dt = tekf.t - tekf_t_prev;
        atdf.dt = atdf.t - atdf.t_prev;

    end
    tekf_t_prev = tekf.t;
    atdf.t_prev = atdf.t;
    % Read sensor data (simulates calling driver fns)
    tekf.gps = position_reading(i,:)
    tekf.accelerometer = accel_reading(i,:);
    atdf.wx = gyro_reading(i,1);

    % Pre compensation for sensors
    % TODO currently just reading 
    % tekf.accel = reading2accel(tekf)

    %% Translational EKF step

    % Measurement
    tekf.z = [tekf.gps(1); tekf.gps(2); tekf.gps(3)];

    % Update
    tekf.h = observation_fn(tekf);
    tekf.dhdx = observation_jacob(tekf);
    tekf.K_n = kalman_gain(tekf);
    tekf.x_n = update_state(tekf);
    tekf.P_n = update_covariance(tekf);

    % Predict
    tekf.f = state_transition_fn(tekf);
    tekf.dfdx = state_transition_jacob(tekf);
    tekf.x_next = predict_state(tekf);
    tekf.P_next = predict_covariance(tekf);
    
    tekf.x_prev = tekf.x_next;
    tekf.P_prev = tekf.P_next;

    %% Attitude Update

    atdf.q_delt = gyroToInstantaneousRotationQuat(atdf.wx, atdf.wy, atdf.wz, atdf.dt);

    atdf.q_new = quatMultiply(atdf.q_current, atdf.q_delt);
    atdf.q_current = atdf.q_new;

    %% Data logging
    tekf_log.t(i) = ekf.t;
    tekf_log.x_n(i,:) = tekf.x_n';
    tekf_log.z(i) = tekf.z;

    atdf_log.qs(i) = atdf.q_current(1);
    atdf_log.qx(i) = atdf.q_current(2);
    atdf_log.qy(i) = atdf.q_current(3);
    atdf_log.qz(i) = atdf.q_current(4);

end

%% Plot generation

%% Quaternion Visualization
    q_init = quaternion(atdf_log.qs(1). atdf_log.qx(1), atdf_log.qy(1), atdf_log.qz(1));
    patch = poseplot(q_init)
    xlabel('North')
    ylabel('East')
    zlabel('Down')

    for i = 1:tekf.loop_size
        quat_plotted = quaternion(atdf_log.qs(i). atdf_log.qx(i). atdf_log.qy(i), atdf_log.qz(i));
        set(patch, Orientation=quat_plotted)
        drawnow
    end

%% Local functions %%
function f = state_transition_fn(ekf)
    % Calculates next state for translational EKF
    f1 = ekf.x_n(1) + ekf.x_n(2)*ekf.dt + 0.5*ekf.dt^2*ekf.accel(1); % 3rd term negligible
    f2 = ekf.x_n(2) + ekf.dt*ekf.accel(1);
    f3 = ekf.x_n(3) + ekf.x_n(4) + 0.5*ekf.dt^2*ekf.accel(2);
    f4 = ekf.x_n(4) + ekf.dt*ekf.accel(2);
    f5 = ekf.x_n(5) + ekf.x_n(6)*ekf.dt + 0.5*ekf.dt^2*ekf.accel(3);
    f6 = ekf.x_n(6) + ekf.dt*ekf.accel(3);

    f = [f1;f2;f3;f4;f5;f6];
end

function dfdx = state_transition_jacob(ekf)
    % Calculates state transition jacobian for translational EKF
    dfdx = [1 ekf.dt 0 0 0 0;
            0 1 0 0 0 0;
            0 0 1 ekf.dt 0 0;
            0 0 0 1 0 0;
            0 0 0 0 1 ekf.dt;
            0 0 0 0 0 1];

end

function h = observation_fn(ekf)
    % Calculates state-to-measurement relation for translational EKF

    h1 = ekf.x_n(1);
    h2 = ekf.x_n(3);
    h3 = ekf.x_n(5);
    
end

function dhdx = observation_jacob(ekf)
    % Calculates observation jacobian for translational EKF
    
    dhdx = [1 0 0 0 0 0;
            0 0 1 0 0 0;
            0 0 0 0 0 1];

end