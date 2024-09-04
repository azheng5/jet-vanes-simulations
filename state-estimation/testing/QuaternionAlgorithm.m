%% Quaternion Algorithm Checking

phi = 0;
theta = pi/2;
psi = 0;

Cb_v = zeros(3,3);

Cb_v(1,1) = cos(theta)*cos(psi);
Cb_v(1,2) = cos(theta)*sin(psi);
Cb_v(1,3) = -sin(theta);

Cb_v(2,1) = cos(psi)*sin(theta)*sin(phi) - cos(phi)*sin(psi);
Cb_v(2,2) = cos(phi)*cos(psi)+sin(theta)*sin(phi)*sin(psi);
Cb_v(2,3) = cos(theta)*sin(phi);

Cb_v(3,1) = cos(phi)*cos(psi)*sin(theta) + sin(phi)*sin(psi);
Cb_v(3,2) = -cos(psi)*sin(phi) + cos(phi)*sin(theta)*sin(psi);
Cb_v(3,3) = cos(theta)*cos(phi);

q_ = zeros(4,1);

q_(1) = sqrt(0.25*(1+Cb_v(1,1) + Cb_v(2,2) + Cb_v(3,3)));
q_(2) = sqrt(0.25*(1+Cb_v(1,1) - Cb_v(2,2) - Cb_v(3,3)));
q_(3) = sqrt(0.25*(1-Cb_v(1,1) + Cb_v(2,2) - Cb_v(3,3)));
q_(4) = sqrt(0.25*(1-Cb_v(1,1) - Cb_v(2,2) + Cb_v(3,3)));

[m, ind] = max(q_);
q = zeros(4,1);

if ind == 1
    q(1) = m;
    q(2) = (Cb_v(2,3) - Cb_v(3,2))/(4*m);
    q(3) = (Cb_v(3,1) - Cb_v(1,3))/(4*m);
    q(4) = (Cb_v(1,2) - Cb_v(2,1))/(4*m);

elseif ind == 2
    q(1) = (Cb_v(2,3) - Cb_v(3,2))/(4*m);
    q(2) = m;
    q(3) = (Cb_v(1,2) - Cb_v(2,1))/(4*m);
    q(4) = (Cb_v(3,1) - Cb_v(1,3))/(4*m);

elseif ind == 3
    q(1) = (Cb_v(3,1) - Cb_v(1,3))/(4*m);
    q(2) = (Cb_v(1,2) + Cb_v(2,1))/(4*m);
    q(3) = m;
    q(4) = (Cb_v(2,3) - Cb_v(3,2))/(4*m);
else
    q(1) = (Cb_v(1,2) - Cb_v(2,1))/(4*m);
    q(2) = (Cb_v(3,1) + Cb_v(1,3))/(4*m);
    q(3) = (Cb_v(2,3) - Cb_v(3,2))/(4*m);
    q(4) = m;
end

%% Rocket on Pad Assuming No Heading
quat = quaternion(q(1), q(2), q(3), q(4));
poseplot(quat)

%% Original NED Frame

quat2 = quaternion(1,0,0,0)
poseplot(quat2)

%%
% 39.25 rad/s over dt = 0.02 s for rotation of 45 degrees
dt = 0.02;
wx =  0;
wy = 0;
wz = 0; % rad/s

BigOmega = [0 wx -wy -wz; wy 0 wz -wy; wy -wz 0 wx; -wz wy -wx 0];

q_new = (dt/2)*BigOmega*q + q;

quat_new = quaternion(q_new(1), q_new(2), q_new(3), q_new(4));
poseplot(quat_new);

%% Try axis angle method
wx = 0;
wy = 0;
wz = 39.25;
dt = 0.02;
omegaVec = [wx wy wz];

if norm(omegaVec) == 0
    omegaVec = omegaVec + 0.001;
end
axis = omegaVec./norm(omegaVec);
angle = dt*norm(omegaVec);

q_delt_s = cos(angle/2);
q_delt_x = axis(1)*sin(angle/2);
q_delt_y = axis(2)*sin(angle/2);
q_delt_z = axis(3)*sin(angle/2);

q_delt = [q_delt_s; q_delt_x; q_delt_y; q_delt_z];

q_s_new = q(1)*q_delt(1) - q(2)*q_delt(2)- q(3)*q_delt(3)- q(4)*q_delt(4);
q_ang_new = q(1).*q_delt(2:4) + q_delt(1).*q(2:4) + cross(q(2:4), q_delt(2:4));

q_new = [q_s_new; q_ang_new];

quat_new = quaternion(q_new(1), q_new(2), q_new(3), q_new(4));
poseplot(quat_new)

%%
poseplot(quat)


