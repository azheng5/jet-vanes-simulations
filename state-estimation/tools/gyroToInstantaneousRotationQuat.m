function [q_delt] = gyroToInstantaneousRotationQuat(wx, wy, wz, timeStep)

omegaVec = [wx wy wz];

if norm(omegaVec) == 0
    omegaVec = omegaVec + 0.001;
end

axis = omegaVec./norm(omegaVec);
angle = timeStep*norm(omegaVec);

q_delt_s = cos(angle/2);
q_delt_x = axis(1)*sin(angle/2);
q_delt_y = axis(2)*sin(angle/2);
q_delt_z = axis(3)*sin(angle/2);

q_delt = [q_delt_s; q_delt_x; q_delt_y; q_delt_z];

end