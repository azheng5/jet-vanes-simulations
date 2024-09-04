function [ekf ekf_log] = make_ekf_struct(nx,nz,x_n0,P_n0,Q,R,loop_size)
%make_ekf_struct Makes ekf struct

%% General variables
ekf.g = 9.81; % gravity at surface
ekf.t = 0; % current time
ekf.t_prev = 0; % previous time
ekf.dt = 0; % current variable time step
ekf.loop_size = loop_size;

%% Dimensions
ekf.nx = nx; % number of states
ekf.nz = nz; % number of measurements

%% Core variables
ekf.x_n = x_n0; % current state x_n,n (nx by 1)
ekf.z = zeros([nz,1]); % measurement z_n (nz by 1)

ekf.x_prev = x_n0; % previously predicted state x_n,n-1 (nx by 1)
ekf.P_prev = P_n0; % previously predicted estimate covariance P_n,n-1 (nx by nx)

ekf.x_next = zeros([nx,1]); % predicted future state x_n+1,n (nx by 1)
ekf.P_next = zeros([nx,nx]); % predicted future estimate covariance P_n+1,n (nx by nx)

%% Intermediate variables
ekf.K_n = zeros([nx,nz]); % kalman gain Kn (nx by nz)
ekf.f = zeros([nx,1]);% state transition fn (nx by 1)
ekf.dfdx = zeros([nx,nx]); % linearized state transition fn (nx by nx)
ekf.h = zeros([nz,1]); % observation fn (nz by 1)
ekf.dhdx = zeros([nz,nx]); % linearized observation fn (nz by nx)

% Covariance
ekf.P_n = zeros([nx,nx]); % current estimate covariance P_n,n (nx by nx)
ekf.Q = Q; % process noise covariance Q (nx by nx)
ekf.R = R; % measurement covariance R (nz by nz)

% Pre allocate memory for data logging
ekf_log.t = zeros(ekf.loop_size,1);
ekf_log.x_n = zeros(ekf.loop_size,ekf.nx);
ekf_log.z = zeros(ekf.loop_size,ekf.nz);
ekf_log.P_n = zeros(ekf.loop_size,ekf.nx,ekf.nx);
    
end