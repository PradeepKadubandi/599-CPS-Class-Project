classdef parking
    methods (Static)
        function [cost_map] = generateCostMap(path_f, max_row, min_row, max_col, min_col)
            %% Initialize cost map
            SCALE = 0.000001;
            rows = ceil((max_row - min_row)/SCALE);
            cols = ceil((max_col - min_col)/SCALE);
            cost_map = zeros(rows, cols);

            %% Fill Cost map
            road_l = size(path_f);

            % For each path
            for idx = 1:road_l

                % Get each pair of lines
                road_i = path_f(idx);
                line_l = size(road_i.X, 2);
                for pair = 1:line_l-2
                    x_0 = parking.normalize_val(road_i.X(pair), min_col, max_col, cols);
                    x_1 = parking.normalize_val(road_i.X(pair+1), min_col, max_col, cols);
                    y_0 = parking.normalize_val(road_i.Y(pair), min_row, max_row, rows);
                    y_1 = parking.normalize_val(road_i.Y(pair+1), min_row, max_row, rows);
            %       fprintf("pt_0 = (%d, %d)\n", x_0, y_0);
            %       fprintf("pt_1 = (%d, %d)\n", x_1, y_1);
                    m = (y_1 - y_0)/(x_1 - x_0);
                    c = y_1 - m*x_1;
            %       fprintf("y = %fx + %f\n", m, c);
                    x_st = x_0;
                    x_en = x_1;
                    if x_1 < x_0
                        x_st = x_1;
                        x_en = x_0;
                    end
                    % List out all points in the line
                    for col_i = x_st:x_en
                        row_i = ceil(m*col_i + c);
                        cost_map(row_i, col_i) = 1;
                        row_i = floor(m*col_i + c);
                        cost_map(row_i, col_i) = 1;
            %           fprintf("(%d, %d)\n", row_i, col_i);
                    end
                end
            end
        end

        function [normed_val] = normalize_val(val, min_val, max_val, to_val)
            normed_val = ceil(((val - min_val)/(max_val - min_val)) * to_val) + 1;
        end
        
    end
end