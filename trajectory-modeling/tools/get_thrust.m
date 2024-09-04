function [T]= get_thrust(thrust_curve, elapsed_time, motor_burn_time)

arb_long_time = 1000000;
tack_on = zeros(1, arb_long_time); %Make it so that thrust has value of zero for values of seconds far greater than burn time.
thrust_curve = [thrust_curve; tack_on'];

%elapsed_time is time since launch

current_sec = floor(elapsed_time);
sec_fraction = mod(elapsed_time, current_sec);

T_component_1 = sec_fraction*thrust_curve(current_sec + 2); %+2 is merely for proper indexing according to how thrust_curve is set up
T_component_2 = (1 - sec_fraction)*thrust_curve(current_sec + 1); 

T = T_component_1 + T_component_2;

if elapsed_time >= motor_burn_time
    T = 0;
end

end