function x_n = update_state(ekf)
%update_state Update current estimated state

x_n = ekf.x_prev + ekf.K_n * (ekf.z - ekf.h);

end