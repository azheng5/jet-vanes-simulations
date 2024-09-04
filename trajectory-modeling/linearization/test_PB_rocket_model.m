%Takes in state (x,u), outputs the xdot
    % Time varying system due to varying thrust and mass


x = [200 0 0 0 0 0 0.7071 0 0.7071 0 0 0 -500]; %u, v, w, p, q, r, q0, q1, q2, q3, pN, pE, pD
u = [0 0 500]; %M_roll, M_pitch, thrust
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

% position expressed in earth/NED frame (m)
x11 = x(11); %PN
x12 = x(12); %PE
x13 = x(13); %PD

% jet vane deflections (rad/s)
u1 = u(1); %M_roll
u2 = u(2); %M_pitch

% thrust
u3 = u(3); %thrust, N

%% Time/altitude varying values
rho = 1.225; %air density, kg/m^3
m = 33; %mass, kg

Ib = [297 0 0;
        0 45844 0;
        0 0 45844]; %Inertia matrix, assumed to be diagonal
invIb = inv(Ib);

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
%CD = 0.31; % coefficient of drag

%CR = 0; % coefficient of rolling moment
%CM = a2*alpha; % coefficient of pitching moment
%CN = a2*beta; % coefficient of yawing moment

%% Gravity force
FG_v = m*g*[0;0;1]; % gravity force in vehicle carried NED frame
[~, FG_b1, FG_b2, FG_b3] = parts(quatconj(q_obj)*quaternion([0;FG_v]')*q_obj); % q'*FG_v*q
FG_b = [FG_b1;FG_b2;FG_b3];

%% Thrust forces and moments
%thrust force in body frame
FT_b = [u3;
        0;
        0];

%thrust moment in body frame
MT_b = [0;
        0;
        0];

%% Aerodynamic forces and moments

CD = 0.04; %rocket drag coefficient, dimensionless
Sx = 0.1; %rocket reference aerodynamic area in yz plane, m^2
Syz = 0.35; %rocket reference aerodynamic area in xy or xz plane, m^2

% aerodynamic force in body frame
FA_b = [-CD*(x1^2)*rho*Sx;
        -CD*(x2^2)*rho*Syz;
        -CD*(x3^2)*rho*Syz]; %(0.5*CD*rho*0.00487) in 3rd ele

% aerodynamic moment about CG in body frame
MA_b = [0;
        0;
        0];

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

% position eqns of motion
[~, P_v_dot1, P_v_dot2, P_v_dot3] = parts(q_obj*quaternion([0;V_b]')*quatconj(q_obj)); % P_V_dot = q_bv*V_b*q_bv'
P_v_dot = [P_v_dot1;P_v_dot2;P_v_dot3];

% stack components
%XDOT = [V_b_dot; wbe_b_dot; q_dot; P_v_dot]
XDOT = [V_b_dot; wbe_b_dot; q_dot]
