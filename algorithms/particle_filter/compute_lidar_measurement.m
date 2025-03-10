function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_MEASUREMENTS Summary of this function goes here
    
    % Set nubmer of lidar rays
    lidar_rays = 8;

    % Extract the pose
    x = pose(1); % Particle X position
    y = pose(2); % Particle Y position
    theta = pose(3); % Particle orientation

    for i = 1 : lidar_rays
        LiDAR_angle = lidar_config(i) + theta; % Lidar ray orientation
        
        intersections = ray_cast([x y], map.walls, LiDAR_angle);
        
        closest_dist = 999;
        for j = 1 : size(intersections, 1)
            % Count distances from intersection
            distance = sqrt((intersections(j, 1) - x)^2 + (intersections(j, 2) - y)^2);
            
            % Save distance from the closest intersection
            if distance < closest_dist
                closest_dist = distance;
            end
        end
        
        % Vector of measured distances 
        measurement(i) = closest_dist;
    end

end