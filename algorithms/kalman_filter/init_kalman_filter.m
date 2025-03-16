function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

public_vars.kf.C = eye(3);
public_vars.kf.R = diag([0.2 0.2 0.05]); % Process noise, higher -> lower trust

cov_matrix = [0.24 0.02;
              0.02 0.26];
public_vars.kf.Q = [cov_matrix(1, 1) 0 0;
                    0 cov_matrix(2, 2) 0;
                    0 0 0.3];

mean_x = 2.0147;
mean_y = 1.8779;
public_vars.mu = [mean_x; mean_y; pi/2 * 1.1];
public_vars.sigma = [0.08 0 0;
                     0 0.08 0;
                     0 0 0.6];

end

