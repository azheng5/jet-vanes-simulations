clear;clc;close all

%%%%%%%%%% Setup %%%%%%%%%

fprintf('Defining rocket parameters...\n');
rkt.nx = 13; % number of states
rkt.nu = 3; % number of control inputs

rkt_time_step = 0.01;
rkt_end_time = 10;
rkt_time_span = 0:rkt_time_step:rkt_end_time;

impulse1 = generateImpulse(0,0.0,20,100,rkt_time_span); % causes moment abt z
impulse2 = generateImpulse(0,0.0,100,25,rkt_time_span); % causes moment abt y
impulse3 = impulse1;
impulse4 = impulse2;
impulse = [impulse1 impulse2 impulse3 impulse4];
impulse(1:20,:) = zeros(20,4);

rkt.thrust_curve = generate_thrust_curve(2000, 15.571); %2000 N T_max, 13 seconds burn time

%% Physical rocket parameters
rkt.cgSpan = load('cg_location.csv');
rkt.motor_burn_time = 15.571; % motor burn time (s)
rkt.m = 38.551; % mass with motors (kg)
rkt.d = 0.203; % diameter (m)
rkt.R = rkt.d/2; % radius (m)
rkt.length = 3.37; % length (m)
rkt.airDensity = 1.225; % air density (kg/m^3)
rkt.g = 9.81; % gravity (m/s^2)
rkt.baseIb = [0.00077421 0 0;
              0 0.0626 0;
              0 0 0.0626];
rkt.invBaseIb = [1291.6 0 0;
                 0 16.0 0;
                 0 0 16.0];


%% Initial state
rktIC = [0;0;0;0;0;0;1;0;0;0;0;0;0];

%%%%%%%%%%%%%% Open loop control %%%%%%%%%%%%%%%%%


%%%%%%%% Generate trajectory %%%%%%%%

% define event function
rkt_options = odeset('Events',@rocket_events);

% Generate trajectory with ode45
% t (time history)
% x (state history)
% te (times at which events occurred),
% ye (state at times of events)
% ie (indices into the vector returned by the event fn)
fprintf('Solving rocket ode...\n')


% [traj.t, traj.x, traj.te, traj.ye, traj.ie] = ode45(@(t,x) rocket_ode(t,x,impulse,rkt_time_span,rkt), ...
%                                                            rkt_time_span, ...
%                                                            rktIC, ...
%                                                            rkt_options);
[traj.t, traj.x, traj.te, traj.ye, traj.ie] = ode45(@(t,x) xDotPrimitive(t,x,impulse,rkt_time_span,rkt), ...
                                                           rkt_time_span, ...
                                                           rktIC, ...
                                                           rkt_options);

% Log xdot history
traj.velBody = traj.x(:,1:3);
traj.angVel = traj.x(:,4:6);
traj.qWorld = traj.x(:,7:10);
traj.posWorld = traj.x(:,11:13);

[traj.length ~] = size(traj.t); % length of trajectory time array
traj.xdot = zeros([traj.length, rkt.nx]);
traj.alpha = zeros([length(traj.t),1]);
traj.beta = zeros([length(traj.t),1]);
% traj.D0 = zeros([length(traj.t),1]);
% traj.L = zeros([length(traj.t),1]);
% traj.Di = zeros([length(traj.t),1]);
% traj.M_ps = zeros([length(traj.t),1]);
traj.MT_b = zeros([length(traj.t),3]);
traj.FT_b = zeros([length(traj.t),3]);
traj.cg = zeros([length(traj.t)],1);
traj.F_b = zeros([length(traj.t),3]);
% traj.MA_b = zeros([length(traj.t),3]);
for i = 1:traj.length
    [xdot,log] = xDotPrimitive(traj.t(i),traj.x(i,:),impulse,rkt_time_span,rkt);
    traj.xdot(i,:) = xdot';
    traj.alpha(i) = log.alpha;
    traj.beta(i) = log.beta;
%     traj.D0(i) = log.D0;
%     traj.Di(i) = log.Di;
%     traj.M_ps(i) = log.M_ps;
%     traj.L(i) = log.L;
    traj.F_b(i,:) = log.F_b;
    traj.MT_b(i,:) = log.MT_b;
    traj.FT_b(i,:) = log.FT_b;
%     traj.MA_b(i,:) = log.MA_b;
    traj.cg(i) = log.cg;
end
traj.angVelDot = traj.xdot(:,4:6);

fprintf('Done!\n');

% writematrix([traj.t, traj.altitude, traj.x(:,1), traj.xdot(:,1)]) %TODO make it write to data directory