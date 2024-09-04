function xDot = attitudeNonlinModel(t,x,atdConfig)
% attitudeNonlinModel Takes in open loop control inputs in the form of
% moments
% x = [q0 q1 q3 q3 wx wy wz phi tht psi]
% tic
fprintf('Current nonlinear attitude model time %f\n',t);

%% Extract config variables
tSpan = atdConfig.tSpan;
uSpan = atdConfig.uSpan;
Ixx = atdConfig.Ixx;
Iyy = atdConfig.Iyy;
Izz = atdConfig.Izz;

%% Define state
q0 = x(1);
q1 = x(2);
q2 = x(3);
q3 = x(4);
wx = x(5);
wy = x(6);
wz = x(7);
phi = x(8); % roll/rotn about x
tht = x(9); % pitch/rotn abt y
psi = x(10); % yaw/rotn abt z

q_bf = [q0 q1 q2 q3]; % quat from flat earth to body
% q_bf = quatnormalize(q_bf);
q_bf = q_bf';

Mx = interp1(tSpan,uSpan(:,1),t);
My = interp1(tSpan,uSpan(:,2),t);
Mz = interp1(tSpan,uSpan(:,3),t);

w = [wx;wy;wz];
M = [Mx;My;Mz];
BigPhi = [phi;tht;psi];

qDot0 = 0.5 * (-wx*q1 - wy*q2 - wz*q3);
qDot1 = 0.5 * (wx*q0 + wz*q2 - wy*q3);
qDot2 = 0.5 * (wy*q0 - wz*q1 + wx*q3);
qDot3 = 0.5 * (wz*q0 + wy*q1 -wx*q2);
wDotx = (Iyy*wy*wz - Izz*wy*wz + Mx)/Ixx;
wDoty = (-Ixx*wx*wz + Izz*wx*wz + My)/Iyy;
wDotz = (Ixx*wx*wy - Iyy*wx*wy + Mz)/Izz;

H = [1 sin(phi)*tan(tht) cos(phi)*tan(tht);
     0 cos(phi) -sin(phi);
     0 sin(phi)*sec(tht) cos(phi)*sec(tht)];
BigPhiDot = H*w;

xDot = [qDot0; qDot1; qDot2; qDot3; wDotx; wDoty; wDotz; BigPhiDot];

%toc
end