function [new_path] = smooth_path(old_path)
%SMOOTH_PATH Summary of this function goes here

    N = length(old_path);
    Y = old_path;  % Initialize the smoothed path as a copy of the original
    X = old_path;  % Keep the original path
    
    % Smoothing parameters
    alpha = 0.1; 
    beta = 0.3;   
    iterations = 100;  % Number of iterations to run

    % Smoothing the path
    for k = 1:iterations
        % A copy of the previous smoothed path for updating
        Y_new = Y;
        
        % Update the points in the path but not the first and the last points
        for i = 2 : N - 1
            Y_new(i) = Y(i) + alpha * (X(i) - Y(i)) + beta * (Y(i - 1) + Y(i + 1) - 2 * Y(i));
        end
        
        % Update the smoothed path
        Y = Y_new;
    end
    
    % The smoothed path
    new_path = Y;

end

