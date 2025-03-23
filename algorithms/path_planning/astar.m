function [path] = astar(read_only_vars, public_vars)
%ASTAR Summary of this function goes here

    % Week 6 - Task 1
    
    % Load a map
    map = read_only_vars.discrete_map.map;
    map = draw_squares(map); % Draw squares into old map to make borders wider

    % MA - Map Arena
    % BM - Binary Map
    % Start and end of the path in the MPC-MAP Arena
    path_start_MA = [round(public_vars.mu(1)) round(public_vars.mu(2))];
    path_end_MA = read_only_vars.map.goal;
    
    % Calculate h_cost for all elements of map from the start to the goal position
    rows = size(map , 1);
    cols = size(map , 2);
    
    % Start and end of the path in the occupance grid
    path_start_BM = path_start_MA * 5 + 1;
    path_end_BM = path_end_MA * 5 + 1;
    f_cost_start = 0;
    
    % Initialize distance matrix
    h_cost = zeros(rows, cols);
    
    x_0 = path_end_BM(1);
    y_0 = path_end_BM(2);
    % Compute h_cost matrix
    for x =  1: cols
        for y = 1:rows
            h_cost(y, x) = sqrt((x - x_0)^2 + (y - y_0)^2); % Distances from the end node
        end
    end
    
    % Initialize parents matrix
    parents_matrix = cell(rows, cols);   % Create a rows x cols cell array
    
    % Making the path
    open_list = [path_start_BM f_cost_start]; % x, y, f_cost_start = 0
    closed_list = [];
    
    g_cost = zeros(rows, cols);     % Distance from the starting node
    f_cost = g_cost + h_cost;       % g_cost + h_cost
    
    index = 1;

    goal_reached = 0;
    while goal_reached == 0 
        [~, idx] = min(open_list(:, 3));            % Find node in OPEN with the lowest f_cost
        node = open_list(idx, :);                   % Node with the lowest f_cost
        open_list(idx, :) = [];                     % Remove current node from OPEN
        closed_list = [closed_list; node];          % Add current node to CLOSED
        
        if node(1: 2) == path_end_BM
            goal_reached = 1;
            break;
        end
    
        % For each node of the current node
        for dx = node(1) - 1: node(1) + 1  % Move in x direction
            for dy = node(2) - 1: node(2) + 1  % Move in y direction

                if dx == node(1) - 1 && dy == node(2) + 1 || dx == node(1) - 1 && dy == node(2) - 1 || dx == node(1) + 1 && dy == node(2) + 1 || dx == node(1) + 1 && dy == node(2) - 1
                    continue;   % Skip, avoid diagonal moves
                end

                if dx == node(1) && dy == node(2)
                    continue;   % Skip, because it's the actual node
                end

                if map(dy, dx) == 1 % If its not traverslable
                    continue;   % Go to the next neighbour
                end

                 % If neighbout is in CLOSED
                for i = 1 : size(closed_list, 1)
                    if closed_list(i, 1) == dx && closed_list(i, 2) == dy
                        continue;
                    end
                end
                
                % Add g_cost based on the direction of a move; 10 for vertical
                % and horizontal and 14 for diagonal ... 2^(-1/2) * 10
                if dx == node(1) - 1 && dy == node(2) || dx == node(1) && dy == node(2) + 1 || dx == node(1) + 1 && dy == node(2) || dx == node(1) && dy == node(2) - 1
                    g_cost_add = 10; % Vertical and horizontal move
                % else
                %     g_cost_add = 14; % Diagonal move
                end
    
                % If new path to the neighbour is shorter or if neighbour is not in the open list
                if g_cost(node(2), node(1)) + g_cost_add < g_cost(dy, dx) || ~any(ismember(open_list, node, 'rows'))
                    g_cost(dy, dx)= g_cost(dy, dx) + g_cost_add;
                    f_cost(dy, dx) = g_cost(dy, dx) + h_cost(dy, dx);
                    parents_matrix{dy, dx} = [node(1), node(2) index];   % Set parent of neighbour to current
                    index = index + 1;
                    open_list = [open_list; dx dy f_cost(dy, dx)];
                end
            end
        end
    end
    
    path_BM = backtracking(parents_matrix, path_end_BM); % Here is final path BM

    path_MA = (path_BM - 1) / 5;
    path = path_MA;

end

