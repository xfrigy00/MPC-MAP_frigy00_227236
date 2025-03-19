function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)
%UPDATE_KALMAN_FILTER Summary of this function goes here

    % mu = public_vars.mu;
    % sigma = public_vars.sigma;
    
    % I. Prediction
    u = [public_vars.motion_vector(1) public_vars.motion_vector(2)];
    
    if read_only_vars.counter == 1
        [public_vars.mu, public_vars.sigma] = ekf_predict(public_vars.mu, public_vars.sigma, u, public_vars.kf, read_only_vars.sampling_period);
    else
        [public_vars.mu, public_vars.sigma] = ekf_predict(read_only_vars.est_position_history(read_only_vars.counter, 1 : 3), public_vars.sigma, u, public_vars.kf, read_only_vars.sampling_period);
    end
    
    % II. Measurement
    z = [read_only_vars.gnss_position(1); read_only_vars.gnss_position(2); public_vars.mu(3)];
    [public_vars.mu, public_vars.sigma] = kf_measure(public_vars.mu, public_vars.sigma, z, public_vars.kf);
    mu = public_vars.mu;
    sigma = public_vars.sigma;
end

