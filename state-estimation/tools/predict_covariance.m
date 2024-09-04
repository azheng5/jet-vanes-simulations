function P_next = predict_covariance(ekf)
%predict_covariance Predicts future covariance

P_next = ekf.dfdx * ekf.P_n * ekf.dfdx' + ekf.Q;

end