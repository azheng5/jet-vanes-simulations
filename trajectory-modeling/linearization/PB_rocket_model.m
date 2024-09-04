function [XDOT]= PB_rocket_model(x,u)
%Takes in state (x,u), outputs the xdot
    % Time varying system due to varying thrust and mass
trim_point_time = 1; %Set the second for which you are finding the trim point for
%% Extract state vector and control input
% velocity in body frame v_b (m/s)
x1 = x(1); %u
x2 = x(2); %v
x3 = x(3); %w

% angular velocity wrt earth frame expressed in body frame (rad/s)
x4 = x(4); %P
x5 = x(5); %Q
x6 = x(6); %R

% unit quat that rotates from V to B (q = q_bv)
x7 = x(7); %q0
x8 = x(8); %q1
x9 = x(9); %q2
x10 = x(10); %q3

% jet vane deflections (rad/s)
u1 = u(1); %M_roll
u2 = u(2); %M_pitch

% thrust (considered a control input)
u3 = u(3); %Thrust, N

%% Time/altitude varying values
if (trim_point_time <= 8)
    rho = 1.15598;%density at 600 m above sea level
else
    rho = 1.06862; %density at 1400 m above sea level
end


m = get_mass(trim_point_time); %mass, kg

[Ixx, Iyy, Izz] = get_inertias(trim_point_time);
Ib = [Ixx 0 0;
        0 Iyy 0;
        0 0 Izz]; %Inertia matrix, assumed to be diagonal.
invIb = inv(Ib);

CG0 = 2.433;
finalCG = 2.34;
burn_time = 13.1;
CG = CG0 - ((CG0 - finalCG)/burn_time) * trim_point_time;

alpha = atan2(x2,x1); % angle of attack (rad)
beta = atan2(x3,(x1^2 + x2^2)^0.5); % angle of sideslip (rad)

%% Intermediate variables

g = 9.81; %gravity, m/s^2
Va = sqrt(x1^2 + x2^2 + x3^2); % airspeed (m/s)
alpha = atan2(x3,x1); % angle of attack (rad)
beta = asin(x2/Va); % angle of sideslip (rad)
Q = 0.5*rho*Va^2;  % dynamic pressure (Pa)

V_b = [x1;x2;x3]; % velocity in body frame (m/s)
wbe_b = [x4;x5;x6]; % angular velocity wrt earth frame expressed in body frame (rad/s)
q = [x7;x8;x9;x10]; % quaternion in vector form
q_obj = quaternion(q'); % quaternion in quat object form

%% Aerodynamic coefficients
%CL = a1*alpha; % coefficient of lift
%CY = a1*beta;  % coefficient of sideforce
CD = 0.31; % coefficient of drag

%CR = 0; % coefficient of rolling moment
%CM = a2*alpha; % coefficient of pitching moment
%CN = a2*beta; % coefficient of yawing moment

%% Gravity force
FG_v = -m*g*[1;0;0]; % gravity force in vehicle carried frame (aligned with flat earth frame)
q_bf = [x7 x8 x9 x10];
q_bf = quatnormalize(q_bf);
FG_b = quat_rotate(q_bf,FG_v'); % parts(quatconj(q_obj)*quaternion([0;FG_v]')*q_obj);
FG_b = FG_b';

%% Thrust forces and moments
%thrust force in body frame
FT_b = [0.92*u3;
        0;
        0];

%thrust moment in body frame
MT_b = [0;
        0;
        0];

%% Aerodynamic forces and moments

[FA_b, MA_b] = calcAeroBody(alpha,norm(V_b),rho,CG);

%% Control input moments

M_roll = u1; %Nm
% F_roll = M_roll/r_vane_to_rocket_x_axis

M_pitch = u2; %Nm
% F_pitch = M_roll/r_vane_to_CG

M_u = [M_roll;
       M_pitch;
       0];

%% State derivatives

% uvw eqns of motion
F_b = FG_b + FT_b + FA_b;
V_b_dot = (1/m)*F_b - cross(wbe_b, V_b);

% pqr eqns of motion
M_b = MT_b + MA_b + M_u;
wbe_b_dot = invIb * (M_b - cross(wbe_b, Ib*wbe_b));

% quaternion eqns of motion
omega = [0 -x4 -x5 -x6;
         x4 0 x6 -x5;
         x5 -x6 0 x4;
         x6 x5 -x4 0];
q_dot = 0.5 .* omega * q;

% stack components
XDOT = [V_b_dot; wbe_b_dot; q_dot];
end
