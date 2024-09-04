function [m]= get_mass(t)

burn_time = 13.1; % seconds
frac_burn_time = t/burn_time; 

m_0 = 38.87; % kg
m_f = 30.96;

m = (1 - frac_burn_time)*m_0 + frac_burn_time*m_f;

end