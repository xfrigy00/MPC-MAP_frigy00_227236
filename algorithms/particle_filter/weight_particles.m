function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here

sigma = 0.05;  % Standard deviation from previous assignment (week_2)

% Number of particle measurements
M = size(particle_measurements, 1);
    
% Initialize weights for result weights
weights = ones(M, 1);
    
    % For each particle measurement
    for m = 1:M
        % Initialize weight for accumulating
        weight = 1;
    
        % For each lidar ray
        for j = 1 : length(lidar_distances)
    
            % Error (difference between actual and predicted)
            error_m = lidar_distances(j) - particle_measurements(m, j);
            
            % Dividing by sigma
            err_div_sig = error_m / sigma;
        
            % Square the error
            error_squared_m = err_div_sig^2;
        
            % Apply exponential function
            exp_func = exp(- 1 / 2 * error_squared_m);
        
            % Accumulating the product
            weight = weight * exp_func;
        end
        weights(m) = weight;
    
    end

    % Normalize weights
    weights = weights / sum(weights);
end