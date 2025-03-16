function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)
    
    % Position and orientation
    x = mu(1);
    y = mu(2);
    theta = mu(3);
    
    % Control inputs
    v_right = u(1);         % Linear velocity for x axe
    v_left = u(2);          % Linear velocity for y axe

    L = 0.2;                            % Distance between wheels
    v = (v_right + v_left) / 2;         % Average linear velocity
    omega = (v_right - v_left) / L;     % Angular velocity
    
    % Time step
    dt = sampling_period;

    % Predicted stat
    new_x = x + cos(theta) * v * dt;
    new_y = y + sin(theta) * v * dt;
    new_theta = theta + omega * dt;
    
    new_mu = [new_x; new_y; new_theta];

    % Jacobian matrix
    G_t = [1 0 -sin(theta) * v * dt;
           0 1 cos(theta) * v * dt;
           0 0 1];

    % Update covariance matrix
    new_sigma = G_t * sigma * G_t' + kf.R;
end
