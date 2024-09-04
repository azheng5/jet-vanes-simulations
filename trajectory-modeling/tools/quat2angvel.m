function w = quat2angvel(q,t)
%quat2angvel Gets angular velocity from quaternion data set
% q - q data (nx4)
% t - time data (nx1)
% w - angular velocity (nx3)

dims = size(q);
w = zeros(dims(1),3);

for i = 2:dims(1)
    dt = t(i) - t(i-1);
    w_i = dquat2angvel(q(i-1,:),q(i,:),dt);
    w(i,:) = w_i;
end

end