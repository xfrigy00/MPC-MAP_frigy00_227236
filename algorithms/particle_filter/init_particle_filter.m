function [public_vars] = init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER Summary of this function goes here

% Number of points
N = 1000;

% Random N values between x_from and x_to
x_from = read_only_vars.map.limits(1);
x_to = read_only_vars.map.limits(3);
x = x_from + (x_to - x_from) .* rand(N,1); % Generate N random numbers from x_to to x_from

% Random y values between y_from and y_to
y_from = read_only_vars.map.limits(2);
y_to = read_only_vars.map.limits(4);
y = y_from + (y_to - y_from) .* rand(N,1);

% Random angles between 0 and 2 * pi
theta = 2 * pi * rand(N, 1);

% Generated points
public_vars.particles = [x y theta];
end

