function P_n = update_covariance(ekf)
%update_covariance Update current state estimate covariance

P_n = (eye(ekf.nx) - ekf.K_n * ekf.dhdx) * ekf.P_prev * (eye(ekf.nx) - ekf.K_n * ekf.dhdx)' + ekf.K_n * ekf.R * ekf.K_n';

end