function [Ixx, Iyy, Izz]= get_inertias(t)

burn_time = 13.1; %seconds

Ixx_0 = 0.2109; %all inertias in kg*m^2 units
Ixx_f = 0.2036;

Iyy_0 = 21.12;
Iyy_f = 18.99;

Izz_0 = 21.12;
Izz_f = 18.99;

frac_burn_time = t/burn_time;

Ixx = (1 - frac_burn_time)*Ixx_0 + frac_burn_time*Ixx_f;
Iyy = (1 - frac_burn_time)*Iyy_0 + frac_burn_time*Iyy_f;
Izz = (1 - frac_burn_time)*Izz_0 + frac_burn_time*Izz_f;

end