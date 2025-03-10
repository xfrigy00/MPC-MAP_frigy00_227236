function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
%PREDICT_POSE Summary of this function goes here
    
    % Pose and control input
    x = old_pose(1, 1);
    y = old_pose(1, 2);
    theta = old_pose(1, 3);

    v_right = motion_vector(1, 1); % Linear velocity of the right wheel
    v_left = motion_vector(1, 2); % Linear velocity of the left wheel
    
    L = read_only_vars.agent_drive.interwheel_dist; % Distance between wheels
    % Linear and angular velocities using the kinematic model
    v = (v_right + v_left) / 2;  % Average linear velocity
    w = (v_right - v_left) / L;  % Angular velocity
    
    dt = read_only_vars.sampling_period; % Time step
    % Update pose using the motion model
    theta_new = theta + w * dt;
    x_new = x + v * cos(theta_new) * dt;
    y_new = y + v * sin(theta_new) * dt;
    
    % Add noise to the updated pose
    noise_v_w = [0.05 0.05 0.01];
    noise_x_new = noise_v_w(1, 1) * randn(); % Noise for x position
    noise_y_new = noise_v_w(1, 2) * randn(); % Noise for y position
    noise_theta_new = noise_v_w(1, 3) * randn(); % Noise for orientation
    
    % New pose with added noise
    new_pose = [x_new + noise_x_new, y_new + noise_y_new, theta_new + noise_theta_new];

end