function [XDOT] = rocket_model(X,U)
%rocket_model Takes in state vector and control input, outputs the xdot

%% Extract state vector and control input
% velocity in body frame v_b (m/s)
x1 = X(1); %u
x2 = X(2); %v
x3 = X(3); %w

% angular velocity wrt earth frame expressed in body frame (rad/s)
x4 = X(4); %p
x5 = X(5); %q
x6 = X(6); %r

% ZYX euler angles (rad/s)
x7 = X(7); %phi
x8 = X(8); %theta
x9 = X(9); %psi

% position in earth frame (m)
x10 = X(10); %Px
x11 = X(11); %Py
x12 = X(12); %Pz

% jet vane deflections (rad/s)
u1 = U(1);
u2 = U(2);
u3 = U(3);
u4 = U(4);

%% Define constants
T = 30; %thrust (N) % real thrust 30
m = 1.201; % mass (kg)
d = 0.0787; % diameter (m)
R = d/2; % radius (m)
h = 0.864; % length (m)
r_CM = 0.36; % distance btw nozzle and CM (m)
rho = 1.225; % air density (kg/m^3)
g = 9.81; % gravity (m/s^2)

%% Control limits

% Define control limits
u1min = -10*pi/180;
u1max = 10*pi/180;

u2min = -10*pi/180;
u2max = 10*pi/180;

u3min = -10*pi/180;
u3max = 10*pi/180;

u4min = -10*pi/180;
u4max = 10*pi/180;

% Clamp control limits
if (u1>u1max)
    u1 = u1max;
elseif (u1<u1min)
    u1 = u1min;
end

if (u2>u2max)
    u2 = u2max;
elseif (u2<u2min)
    u2 = u2min;
end

if (u3>u3max)
    u3 = u3max;
elseif (u3<u3min)
    u3 = u3min;
end

if (u4>u4max)
    u4 = u4max;
elseif (u4<u4min)
    u4 = u4min;
end

%% Intermediate variables
Va = sqrt(x1^2 + x2^2 + x3^2); % airspeed (m/s)
alpha = atan2(x3,x1); % angle of attack (rad)
beta = asin(x2/Va); % angle of sideslip (rad)
Q = 0.5*rho*Va^2;  % dynamic pressure (Pa)

V_b = [x1;x2;x3]; % velocity in body frame (m/s)
wbe_b = [x4;x5;x6]; % angular velocity wrt earth frame expressed in body frame (rad/s)

%% Aerodynamic coefficients
%CL = a1*alpha; % coefficient of lift
%CY = a1*beta;  % coefficient of sideforce
CD = 0.31; % coefficient of drag

%CR = 0; % coefficient of rolling moment
%CM = a2*alpha; % coefficient of pitching moment
%CN = a2*beta; % coefficient of yawing moment

%% Gravity force
% gravity force in body frame
FG_b = m*g*[-cos(x8)*cos(x9);
            cos(x7)*sin(x9) - cos(x9)*sin(x7)*sin(x8);
            -sin(x7)*sin(x9) + cos(x7)*cos(x9)*sin(x8)];

%% Thrust forces and moments
%thrust force in body frame
FT_b = [T;
        0.25*T*(u1+u3);
        0.25*T*(u2+u4)];

%thrust moment in body frame
MT_b = [(T/4)*(R/4)*(-u1+u2+u3-u4);
        (T/4)*r_CM*(u2+u4);
        -(T/4)*r_CM*(u1+u3)];

%% Aerodynamic forces and moments
% aerodynamic force in body frame
FA_b = [0;
        0;
        0]; %(0.5*CD*rho*0.00487) in 3rd ele

% aerodynamic moment about CG in body frame
MA_b = [0;
        0;
        0];

%% State derivatives
% moment of inertia (kgm^2)
Ib = m*[0.00077421 0 0;
        0 0.0626 0;
        0 0 0.0626];
invIb = (1/m)*[1291.6 0 0;
               0 16.0 0;
               0 0 16.0];

% uvw eqns of motion
F_b = FG_b + FT_b + FA_b;
V_b_dot = (1/m)*F_b;% - cross(wbe_b, V_b);
%V_b_dot = (1/m)*F_b - cross(wbe_b, V_b);


% pqr eqns of motion
M_b = MT_b + MA_b;
wbe_b_dot = invIb * (M_b - cross(wbe_b, Ib*wbe_b));

% euler angles eqns of motion
H_phi = [1 sin(x7)*tan(x8) cos(x7)*tan(x8);
         0 cos(x7) -sin(x7);
         0 sin(x7)/cos(x8) cos(x7)/cos(x8)];
bigphi_dot = H_phi*wbe_b;

% position eqns of motion
Cvb = [cos(x8)*cos(x9), cos(x9)*sin(x7)*sin(x8) - cos(x7)*sin(x9), sin(x7)*sin(x9) + cos(x7)*cos(x9)*sin(x8);
       cos(x8)*sin(x9), cos(x7)*cos(x9) + sin(x7)*sin(x8)*sin(x9), cos(x7)*sin(x8)*sin(x9) - cos(x9)*sin(x7);
       -sin(x8),        cos(x8)*sin(x7),                           cos(x7)*cos(x8)];
P_v_dot = Cvb*V_b;

% stack components
XDOT = [V_b_dot; wbe_b_dot; bigphi_dot; P_v_dot];

end