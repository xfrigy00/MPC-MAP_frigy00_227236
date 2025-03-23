function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
          
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

end

% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);

%% Week 2 - Task 2 – Sensor uncertainty

public_vars.sigma_vals(read_only_vars.counter, 1:8) = read_only_vars.lidar_distances(1, :);   % Setting sigma
public_vars.sigma_vals(read_only_vars.counter, 9:10) = read_only_vars.gnss_position(1, :);

% if (read_only_vars.counter == 200)
%     % If variable read_only_vars.counter has a wanted value
%     fprintf('Variable read_only_vars.counter has a value %d\n', read_only_vars.counter);
% 
%     figure;
%     hold on;
%     num_lidar_channels = 8;
%     for i = 1:num_lidar_channels
%         lidar = public_vars.sigma_vals(:, i);      % Working only with one channel
%         lidar_avg = mean(lidar);                                    % Average value
%         lidar_devs = lidar - lidar_avg;                             % Deviations
%         lidar_devs_sq_devs = lidar_devs.^2;                         % Squared deviations
%         lidar_mean_sq_dev = mean(lidar_devs_sq_devs);               % Variance
%         lidar_std_value(i) = sqrt(lidar_mean_sq_dev);               % Sigma for lidar
%         disp(['Value of the standard deviation for channel ', num2str(i), ' of the LiDAR is ', num2str(lidar_std_value(i))]);
%         subplot(2, 4, i);
%         histogram(lidar);                                           % Histogram
%         title(['LiDAR  Channel ', num2str(i)]);
%         xlabel('Distance');
%         ylabel('Amount');
%     end
%     sgtitle('LiDAR sensor histograms');
% 
%     figure;
%     hold on;
%     num_gnss_channels = 2;
%     for i = 1 + num_lidar_channels: num_gnss_channels + num_lidar_channels
%         gnss = public_vars.sigma_vals(:, i);                                  % Working only with one channel
%         gnss_avg = mean(gnss);                                                % Average value
%         gnss_devs = gnss - gnss_avg;                                          % Deviations
%         gnss_devs_sq_devs = gnss_devs.^2;                                     % Squared deviations
%         gnss_mean_sq_dev = mean(gnss_devs_sq_devs);                           % Variance
%         gnss_std_value(i - num_lidar_channels) = sqrt(gnss_mean_sq_dev);      % Sigma for gnss
%         disp(['Value of the standard deviation for channel ', num2str(i - num_lidar_channels), ' of the GNSS is ', num2str(gnss_std_value(i - num_lidar_channels))]);
%         subplot(1, 2, i - num_lidar_channels);
%         histogram(gnss);                                                      % Histogram
%         title(['GNSS  Channel ', num2str(i - num_lidar_channels)]);
%         xlabel('Position');
%         ylabel('Amount');
%     end
%     sgtitle('GNSS sensor histograms');
% 
%     %% Week 2 - Task 3 – Covariance matrix
%     % Lidar data
%     lidar_data = public_vars.sigma_vals(:, 1:num_lidar_channels);   
% 
%     % Covariance matrix for lidar
%     lidar_cov = cov(lidar_data)
% 
%     % Size of the covariance matrix should be 8x8
%     size(lidar_cov);
% 
%     % Diagonal elements are the variance, std^2
%     lidar_diag = diag(lidar_cov);
%     disp('LiDAR: Values on the main diagonal are almost equal to sigma^2');
%     diff_lidar_diag_lidar_std_sq = lidar_diag - (lidar_std_value').^2
% 
%     % GNSS Data
%     gnss_data = public_vars.sigma_vals(:, 1 + num_lidar_channels:2 + num_lidar_channels);
% 
%     % Covariance matrix for GNSS
%     gnss_cov = cov(gnss_data)
% 
%     % Size of the covariance matrix should be 2x2
%     size(gnss_cov);
% 
%     % Diagonal elements are the variance, std^2
%     gnss_diag = diag(gnss_cov);
%     disp('GNSS: Values on the main diagonal are almost equal to sigma^2');
%     diff_gnss_diag_gnss_std_sq = gnss_diag - (gnss_std_value').^2
% 
%     %% Week 2 - Task 4 – Normal distribution
%     % x: values at which to evaluate the pdf
%     x_vals = linspace(-2, 2, 100);
% 
%     % mu: mean
%     public_vars.mu = 0;
% 
%     % Using norm_pdf function for LiDAR and GNSS
%     LiDAR_pdf = norm_pdf(x_vals, public_vars.mu, lidar_std_value(1)); % Lidar, 1st channel
%     GNSS_pdf = norm_pdf(x_vals, public_vars.mu, gnss_std_value(1));   % X GNSS axis
% 
%     % Plots
%     figure;
%     plot(x_vals, LiDAR_pdf, 'r-', 'LineWidth', 2); % LiDAR
%     hold on;
%     plot(x_vals, GNSS_pdf, 'b-', 'LineWidth', 2);  % GNSS
%     hold off;
% 
%     xlabel('Values at which to evaluate the pdf');
%     ylabel('Probability density');
%     title('LiDAR and GNSS, illustrating the noise');
%     legend('1st LiDAR channel', 'X GNSS axis');
%     grid on;
% end

%intersections = ray_cast([2 8.5], read_only_vars.map.walls, 0)

    %% Week 5 - Task 1 – Preparation

    % Standard deviations GNSS channels from one of the previous assignments
    sigma_GNSS_x = 0.485;
    sigma_GNSS_y = 0.507;
    
    % GNSS samples
    gnss_data = [0.944093693072098 1.502972670403505; 
                 2.132131458121698 2.217480150624847; 
                 1.376864828275910 2.302875803616523; 
                 2.095313290646735 1.822534483283383;
                 2.331106855594518 2.446641689937425;
                 2.566858412053019 1.451315673994478;
                 2.291246625424902 2.246271676708273;
                 2.726362860864620 1.664421529245411;
                 1.865923682827822 1.446293577405589
                 1.816826489683763 1.678323977575767];
    
    % Mean for both of the GNSS channels
    public_vars.mean_x = mean(gnss_data(:,1)); % 
    public_vars.mean_y = mean(gnss_data(:,2)); % 
    
    % Covariance
    N = size(gnss_data, 1);
    cov_xy = sum((gnss_data(:, 1) - public_vars.mean_x) .* (gnss_data(:, 2) - public_vars.mean_y)) / (N - 1);

    % Covariance matrix 
    public_vars.cov_matrix = [sigma_GNSS_x ^ 2 cov_xy;
                              cov_xy sigma_GNSS_y ^ 2];

end








