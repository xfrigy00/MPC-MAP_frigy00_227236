function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)
%UPDATE_KALMAN_FILTER Summary of this function goes here

mu = public_vars.mu;
sigma = public_vars.sigma;

% I. Prediction
u = [public_vars.motion_vector(1) public_vars.motion_vector(2)];
[mu, sigma] = ekf_predict(mu, sigma, u, public_vars.kf, read_only_vars.sampling_period);

% II. Measurement
z = [read_only_vars.gnss_position(1); read_only_vars.gnss_position(2); mu(3)];
[public_vars.mu, public_vars.sigma] = kf_measure(mu, sigma, z, public_vars.kf);

end

