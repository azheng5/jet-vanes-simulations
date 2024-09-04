function [initial_q]= getInitialAttitude(phi,theta,psi)

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

initial_q = q;

end