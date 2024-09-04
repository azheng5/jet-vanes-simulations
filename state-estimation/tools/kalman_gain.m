function K_n = kalman_gain(ekf)
%kalman_gain Calculates intermediate variable kalman gain

K_n = ekf.P_prev * ekf.dhdx' * inv( ekf.dhdx * ekf.P_prev * ekf.dhdx' + ekf.R);

end