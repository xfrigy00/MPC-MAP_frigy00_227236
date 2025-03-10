function [public_vars] = plan_motion(read_only_vars, public_vars)
    %PLAN_MOTION Summary of this function goes here
    
    public_vars.motion_vector = [0.1, 0.1];   % Prave koleso, lave koleso

    %% Week 2 - Task 5 – Motion uncertainty
    % Loaded map: indoor_1
    % In main.m line "read_only_vars.counter = read_only_vars.counter + 1"
    % needs to be without ";"

    % if read_only_vars.counter < 300
    %     public_vars.motion_vector = [0.2, 0.2];   % Prave koleso, lave koleso
    % elseif read_only_vars.counter > 300 && read_only_vars.counter < 330   
    %     public_vars.motion_vector = [0, 0.1];   % Prave koleso, lave koleso
    % elseif read_only_vars.counter > 330 && read_only_vars.counter < 450   
    %     public_vars.motion_vector = [0.2, 0.2];   % Prave koleso, lave koleso
    % elseif read_only_vars.counter > 450 && read_only_vars.counter < 480   
    %     public_vars.motion_vector = [0, 0.1];   % Prave koleso, lave koleso
    % elseif read_only_vars.counter > 480 && read_only_vars.counter < 860   
    %     public_vars.motion_vector = [0.2, 0.2];   % Prave koleso, lave koleso
    % elseif read_only_vars.counter > 860 && read_only_vars.counter < 890   
    %     public_vars.motion_vector = [0.1, 0];   % Prave koleso, lave koleso
    % elseif read_only_vars.counter > 890 && read_only_vars.counter < 950   
    %     public_vars.motion_vector = [0.2, 0.2];   % Prave koleso, lave koleso
    % elseif read_only_vars.counter > 950 && read_only_vars.counter < 970   
    %     public_vars.motion_vector = [0.1, 0];   % Prave koleso, lave koleso
    % elseif read_only_vars.counter > 970 && read_only_vars.counter < 1500   
    %     public_vars.motion_vector = [0.2, 0.2];   % Prave koleso, lave koleso
    % end

    % I. Pick navigation target
    
    target = get_target(public_vars.estimated_pose, public_vars.path);
    
    % II. Compute motion vector

    %% Week 3 - Task 2 – Create a path

    % x_1st_c = linspace(pi/2, 0, 20);    % c - curve
    % first_c_offset_x = 4;
    % first_c_offset_y = 7.5;
    % 
    % x_2nd_c = linspace(pi, 3/2*pi, 20);    % c - curve
    % second_c_offset_x = 6;
    % second_c_offset_y = 2;
    % 
    % x_3rd_c = linspace(3/2 * pi, 2 * pi, 20);    % c - curve
    % third_c_offset_x = 8;
    % third_c_offset_y = 2;
    % 
    % % Planning the path
    % public_vars.path = [2 8.5;
    %  2.5 8.5;
    %  3 8.5;
    %  3.5 8.5;
    %  4 8.5;
    %  cos(x_1st_c)' + first_c_offset_x sin(x_1st_c)' + first_c_offset_y;
    %  5 7;
    %  5 6.5;
    %  5 6;
    %  5 5.5;
    %  5 5;
    %  5 4.5;
    %  5 4;
    %  5 3.5;
    %  5 3;
    %  5 2.5;
    %  5 2;
    %  cos(x_2nd_c)' + second_c_offset_x sin(x_2nd_c)' + second_c_offset_y;
    %  6.5 1;
    %  7 1;
    %  7.5 1;
    %  8 1;
    %  cos(x_3rd_c)' + third_c_offset_x sin(x_3rd_c)' + third_c_offset_y;
    %  9 2.5;
    %  9 3;
    %  9 3.5;
    %  9 4;
    %  9 4.5;
    %  9 5;
    %  9 5.5;
    %  9 6;
    %  9 6.5;
    %  9 7;
    %  9 7.5;
    %  9 8;
    %  9 8.5;
    %  9 9];
    
    % In this moment, path is already defined

    %% Week 3 - Task 3 – Motion control

    % % Robot position
    % x_robot = read_only_vars.mocap_pose(1);
    % y_robot = read_only_vars.mocap_pose(2);
    % theta_robot = read_only_vars.mocap_pose(3); % Orientation in radians
    % 
    % lookahead_distance = 1.5; % Lookahead distance
    % 
    % % Setup
    % if read_only_vars.counter == 1
    %     public_vars.min_index = 1;
    % end
    % 
    % path_size = size(public_vars.path, 1);
    % 
    % % Find closest point to the robot
    % for current_index = public_vars.min_index : path_size
    %     % Calculate Euclidean distance from current point to the robot's position
    %     public_vars.dx = public_vars.path(current_index, 1) - x_robot;
    %     public_vars.dy = public_vars.path(current_index, 2) - y_robot;
    %     dist = sqrt(public_vars.dx^2 + public_vars.dy^2);
    % 
    %     % Point with max dist but less than lookahead_distance. 
    %     % Using the last
    %     if dist < lookahead_distance
    % 
    %        % Avoiding error in the end
    %        if current_index + 1 ~= path_size + 1
    %             public_vars.min_index = current_index + 1;
    %        end
    % 
    %        % Saving dx and dy of points
    %        public_vars.correct_dx = public_vars.path(current_index, 1) - x_robot;
    %        public_vars.correct_dy = public_vars.path(current_index, 2) - y_robot;
    %     end
    % end
    % 
    % % Calculate the angle to the lookahead point
    % alpha = atan2(public_vars.correct_dy, public_vars.correct_dx) - theta_robot;
    % 
    % % Calculate the steering angle
    % delta = atan2(2 * lookahead_distance * sin(alpha), dist);   % Steering angle
    % 
    % WB = read_only_vars.agent_drive.interwheel_dist;            % Distance between the wheels - wheelbase
    % v = 0.3;                                    % Desired linear velocity
    % 
    % % Compute the angular velocity based on the steering angle
    % omega = (v * tan(delta)) / WB;              % Angular velocity [rad/s]
    % 
    % % Calculate the wheel velocities
    % v_L = v - (WB / 2) * omega;                 % Left wheel velocity
    % v_R = v + (WB / 2) * omega;                 % Right wheel velocity
    % 
    % public_vars.motion_vector = [v_R, v_L];     % Right wheel, Left wheel
end