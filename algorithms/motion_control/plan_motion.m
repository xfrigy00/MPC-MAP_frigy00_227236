function [public_vars] = plan_motion(read_only_vars, public_vars)
    %PLAN_MOTION Summary of this function goes here
    
    % I. Pick navigation target
    
    target = get_target(public_vars.estimated_pose, public_vars.path);
    
    % II. Compute motion vector


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
end