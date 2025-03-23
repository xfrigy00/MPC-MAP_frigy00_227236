function [path] = backtracking(parents_matrix, path_end_BM); % Here is final path BMC
%BACKTRACKING Summary of this function goes here
    
    % Initial coordinates of the target cell
    x = path_end_BM(1);
    y = path_end_BM(2);
    
    % Possible shifts
    directions = [-1, -1; -1, 0; -1, 1; 0, -1; 0, 1; 1, -1; 1, 0; 1, 1];
    
    % Field for saving the path
    path = [];
    
    while true
        % Saving coordinates to a path
        path = [path; x y];
    
        % Get the current cell and its value
        current_cell = parents_matrix{y, x};
        if isempty(current_cell)
            break; % If the cell does not exist, end
        end
        current_index = current_cell(3);
    
        % Termination condition
        if current_index == 1 || current_index == 2 || current_index == 3
            break;
        end
    
        % Finding the best neighboring cell
        min_index = Inf;
        best_y = y;
        best_x = x;
    
        for d = 1 : size(directions, 1)
            new_y = y + directions(d, 1);
            new_x = x + directions(d, 2);
    
            % Border control
            if new_y < 1 || new_y > size(parents_matrix, 1) || new_x < 1 || new_x > size(parents_matrix, 2)
                continue;
            end
    
            % Get the value of an adjacent cell
            neighbor_cell = parents_matrix{new_y, new_x};
            if isempty(neighbor_cell)
                continue;
            end
    
            neighbor_index = neighbor_cell(3);
    
            % Select the cell with the lowest value
            if neighbor_index < min_index
                min_index = neighbor_index;
                best_y = new_y;
                best_x = new_x;
            end
        end
    
        % Move to a new cell
        x = best_x;
        y = best_y;
    end
    path;
end

