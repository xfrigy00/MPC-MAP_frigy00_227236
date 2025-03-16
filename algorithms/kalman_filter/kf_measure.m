function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)
%KF_MEASURE Summary of this function goes here

    C = kf.C;
    Q = kf.Q;

    % K_t
    K_t = sigma * C' / (C * sigma * C' + Q);

    % Update mu
    new_mu = mu + K_t * (z - C * mu);

    % Update the sigma
    new_sigma = (eye(size(sigma, 1)) - K_t * C) * sigma;
end
