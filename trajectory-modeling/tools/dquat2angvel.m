function w = dquat2angvel(q1,q2,dt)
%dquat2angvel Gets angular velocity from two quaternions at consecutive
%points in time
% q1 - q at time t (1x4)
% q2 - q at time t+dt (1x4)
% dt - time interval
% w - angular velocity (1x3)

w = (2/dt) .* [q1(1)*q2(2) - q1(2)*q2(1) - q1(3)*q2(4) + q1(4)*q2(3);
               q1(1)*q2(3) + q1(2)*q2(4) - q1(3)*q2(1) - q1(4)*q2(2);
               q1(1)*q2(4) - q1(2)*q2(3) + q1(3)*q2(2) - q1(4)*q2(1)];
w = w';

end