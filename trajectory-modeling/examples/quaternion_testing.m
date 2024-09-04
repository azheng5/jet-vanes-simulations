%% Testing out and validating quaternions

%% Derive initial quaternion (initial rotation from NED to body)
phi = 0;
theta = pi/2;
psi = 0;
q = angle2quat(psi,theta,phi); % assumes a ZYX rotn sequence
% hence initial quaternion state is: q =[0 0.7071 0 0.7071]
% q = unit quat from v to b = q_bv

%% Test out quaternion rotation on gravity
m = 10;
g = 9.8;

% unit quat that rotates from V to B (q = q_bv)
q = quaternion([0.7071;0;0.7071;0]');

% gravity force in vehicle carried NED frame
FG_v = m*g*[0;0;1];

% gravity force in body frame (assume rocket is upright)
[~, FG_b1, FG_b2, FG_b3] = parts(qwsuatconj(q)*quaternion([0;FG_v]')*q); % q'*FG_v*q
FG_b = [FG_b1;FG_b2;FG_b3];
% should be FG_b = -m*g*[1;0;0]

%% Test out quaternion kinematic equations
% Is it q_bv_dot = 0.5 * omega * q_bv
% OR q_vb_dot = 0.5 * omega * q_vb?

