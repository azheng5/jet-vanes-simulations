function [M, L, D]= get_aero(a, v)

% a in angle of attack in degrees
% v is airspeed (sqrt(u^2 + v^2 + w^2))
% units are lbf*ft, lbf
rho = 0.00217847;

CM = a.*((4.12421 ./ (27.81553 + 16.83430 * sqrt(1 - (v./1097.40).^2))) + -0.00014 .* a + 0.01273);
CD = 0.00253 * a.^2 + 0.13932;
CL = 0.64181 .* a .* (0.64181 ./ (3.59306 + sqrt(1 - (v./1097.40).^2)));

M = CM * 0.5 * rho * v.^2 * 0.9375;
D = CD * 0.5 * rho * v.^2 * 0.9375;
L = CL * 0.5 * rho * v.^2 * 0.9375;

M = M/0.737562;
D = D*4.448;
L = L*4.448;

end