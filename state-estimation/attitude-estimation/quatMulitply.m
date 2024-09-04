function [quat_new]= quatMulitply(q1,q2)

q_s_new = q1(1)*q2(1) - q1(2)*q2(2)- q1(3)*q2(3)- q1(4)*q2(4);
q_ang_new = q1(1).*q2(2:4) + q2(1).*q1(2:4) + cross(q1(2:4), q2(2:4));

quat_new = [q_s_new; q_ang_new];

end