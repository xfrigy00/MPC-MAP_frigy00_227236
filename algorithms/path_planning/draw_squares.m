function [map] = draw_squares(map_input)
%DRAW_SQUARES Summary of this function goes here

    % Get the size of the input matrix
    [rows, cols] = size(map_input);
    square_size = 3; % Square size of the drawn squares

    % Loop through all the elements in the map_input
    for i = 1:rows
        for j = 1:cols

            % Look for 1s
            if map_input(i, j) == 1
                
                % Top - left corner of the square
                top_left_row = max(i - floor(square_size/2), 1);
                top_left_col = max(j - floor(square_size/2), 1);
                
                % Square will not be drawn out of bounds
                bottom_right_row = min(top_left_row + square_size - 1, rows);
                bottom_right_col = min(top_left_col + square_size - 1, cols);
                
                % Square with 1s in the new map matrix 
                map(top_left_row:bottom_right_row, top_left_col:bottom_right_col) = 1;
            end
        end
    end
end
