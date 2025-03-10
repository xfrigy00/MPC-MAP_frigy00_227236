function [new_particles] = resample_particles(particles, weights)
%RESAMPLE_PARTICLES Summary of this function goes here

    % Number of particles
    N = length(weights);

    % Initialization of cumulative sum of normalized weights
    cumulative_weights = cumsum(weights);
    
    % Initialize resampled particles
    resampled_particles = zeros(N, size(particles, 2));

    % Index for particles
    index = 1; 

    % Generate the first random number
    u_0 = rand / N;

    % Systematic sampling points
    u = zeros(N, 1);    % Initialize the vector of systematic sampling points
    for k = 1:N
        u(k) = u_0 + (k - 1) / N;
    end 

    for k = 1:N
        % Move through the cumulative sum and pick the particle
        while index < N && u(k) > cumulative_weights(index)
            index = index + 1;
        end
        
        % Assign the selected particle
        resampled_particles(k, :) = particles(index, :);
    end
    new_particles = resampled_particles;
end

