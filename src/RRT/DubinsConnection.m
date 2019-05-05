classdef DubinsConnection
    %DUBINSPATHSEGMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(SetAccess = private)
        MinTurnRadius;
        InterpolationSteps;
        UseInterpolationSteps;
    end
    
    methods
        function obj = DubinsConnection(minTurnRadius, interpolationSteps, useInterpolationSteps)
            obj.MinTurnRadius = minTurnRadius;
            obj.InterpolationSteps = interpolationSteps;
            obj.UseInterpolationSteps = useInterpolationSteps;
        end
        
        function dubinsPathSegment = computeDubinsPath(obj, start, goal)
            c = 1 / obj.MinTurnRadius;

            s_yaw = start(3);
            diff = goal - start;
            c_w_to_v = [cos(s_yaw), -sin(s_yaw), 0; sin(s_yaw), cos(s_yaw), 0; 0, 0 , 1]; % World to Vehicle transform

            [motion_lengths, motion_types, optimalCost, path] = obj.computeDubinsInternal(diff * c_w_to_v, c);

            c_v_to_w = [cos(-s_yaw), -sin(-s_yaw), 0; sin(-s_yaw), cos(-s_yaw), 0; 0, 0 , 1]; % Vehicle to World transform
            path = path * c_v_to_w + start;
            path(:, 3) = wrapToPi(path(:, 3));

            dubinsPathSegment = DubinsPathSegment(motion_lengths, motion_types, optimalCost, path);
        end

        function [motion_lengths, motion_types, optimalCost, path] = computeDubinsInternal(obj, diff, c)
            D = norm(diff(1:2));
            theta = wrapTo2Pi(atan2(diff(2), diff(1)));
            alpha = wrapTo2Pi(0.0 - theta);
            beta = wrapTo2Pi(diff(3) - theta);

            optimalCost = inf;
            motion_types = nan;
            motion_lengths = nan;
            path = nan;
            allModes = {@obj.LSL, @obj.RSR, @obj.LSR, @obj.RSL, @obj.RLR, @obj.LRL};

            for i = 1:length(allModes)
                [t1, t2, t3, mode] = allModes{i}(alpha, beta, D * c);
                currentCost = abs(t1) + abs(t2) + abs(t3);
                if ((~isnan(t1)) && (currentCost < optimalCost))
                    optimalCost = currentCost;
                    [best_t1, best_t2, best_t3, motion_types] = deal(t1, t2, t3, mode);
                end
            end

            if (~isnan(motion_types))
                motion_lengths = [best_t1, best_t2, best_t3];
                path = obj.generate_path(motion_lengths, motion_types, c);
            end
        end

        function path = generate_path(obj, motion_lengths, motion_types, c)
            path = [0, 0, 0];
            counter = 1;
            for i = 1:3
                pd = 0.0;
                m = motion_types(i);
                l = motion_lengths(i);

                if (m == 'S')
                    d = 0.3 * c;
                else
                    d = deg2rad(3.0);
                end

                reachedMaxSteps = false;
                while (pd < abs(l - d))
                    last = path(end, :);
                    transform = obj.get_transform_helper(d, m, c, last(3));
                    path(end+1, :) = last + transform;
                    pd = pd + d;
                    if (obj.UseInterpolationSteps && (obj.InterpolationSteps == counter))
                        reachedMaxSteps = true;
                        break;
                    end
                    counter = counter + 1;
                end

                if (reachedMaxSteps)
                    break;
                else
                    last = path(end, :);
                    transform = obj.get_transform_helper(l-pd, m, c, last(3));
                    path(end+1, :) = last + transform;
                end
            end
        end

        function transform = get_transform_helper(obj, d, m, c, theta)
            if (m == 'L')
                offset = d;
            elseif (m == 'S')
                offset = 0;
            else
                offset = -d;
            end
            transform = [(d/c) * cos(theta), (d/c) * sin(theta), offset];
        end

        function [t1, t2, t3, mode] = LSL(obj, alpha, beta, d)
            sa = sin(alpha);
            sb = sin(beta);
            ca = cos(alpha);
            cb = cos(beta);
            c_ab = cos(alpha - beta);
            mode = ['L', 'S', 'L'];

            p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb));

            if (p_squared < 0)
                [t1, t2, t3] = deal(nan, nan, nan);
            else
                tmp0 = d + sa - sb;
                tmp1 = atan2((cb-ca), tmp0);
                t1 = wrapTo2Pi(-alpha + tmp1);
                t2 = sqrt(p_squared);
                t3 = wrapTo2Pi(beta - tmp1);
            end
        end

        function [t1, t2, t3, mode] = RSR(obj, alpha, beta, d)
            sa = sin(alpha);
            sb = sin(beta);
            ca = cos(alpha);
            cb = cos(beta);
            c_ab = cos(alpha - beta);
            mode = ['R', 'S', 'R'];

            p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa));

            if (p_squared < 0)
                [t1, t2, t3] = deal(nan, nan, nan);
            else
                tmp0 = d - sa + sb;
                tmp1 = atan2((ca-cb), tmp0);
                t1 = wrapTo2Pi(alpha - tmp1);
                t2 = sqrt(p_squared);
                t3 = wrapTo2Pi(-beta + tmp1);
            end
        end

        function [t1, t2, t3, mode] = LSR(obj, alpha, beta, d)
            sa = sin(alpha);
            sb = sin(beta);
            ca = cos(alpha);
            cb = cos(beta);
            c_ab = cos(alpha - beta);
            mode = ['L', 'S', 'R'];

            p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb));

            if (p_squared < 0)
                [t1, t2, t3] = deal(nan, nan, nan);
            else
                tmp0 = d + sa + sb;
                t2 = sqrt(p_squared);
                tmp1 = atan2((-cb-ca), tmp0) - atan2(-2.0, t2);
                t1 = wrapTo2Pi(-alpha + tmp1);
                t3 = wrapTo2Pi(-wrapTo2Pi(beta) + tmp1);
            end
        end

        function [t1, t2, t3, mode] = RSL(obj, alpha, beta, d)
            sa = sin(alpha);
            sb = sin(beta);
            ca = cos(alpha);
            cb = cos(beta);
            c_ab = cos(alpha - beta);
            mode = ['R', 'S', 'L'];

            p_squared = -2 + (d * d) + (2 * c_ab) - (2 * d * (sa + sb));

            if (p_squared < 0)
                [t1, t2, t3] = deal(nan, nan, nan);
            else
                t2 = sqrt(p_squared);
                tmp0 = d - sa - sb;
                tmp1 = atan2((cb+ca), tmp0) - atan2(2.0, t2);
                t1 = wrapTo2Pi(alpha - tmp1);
                t3 = wrapTo2Pi(beta - tmp1);
            end
        end

        function [t1, t2, t3, mode] = RLR(obj, alpha, beta, d)
            sa = sin(alpha);
            sb = sin(beta);
            ca = cos(alpha);
            cb = cos(beta);
            c_ab = cos(alpha - beta);
            mode = ['R', 'L', 'R'];

            tmp_rlr = (6.0 - (d * d) + (2.0 * c_ab) + (2.0 * d * (sa - sb))) / 8.0;

            if (abs(tmp_rlr) > 1.0)
                [t1, t2, t3] = deal(nan, nan, nan);
            else
                t2 = wrapTo2Pi((2 * pi) - acos(tmp_rlr));
                tmp0 = d - sa + sb;
                tmp1 = atan2((ca-cb), tmp0);
                t1 = wrapTo2Pi(alpha - tmp1 + wrapTo2Pi(t2 / 2.0));
                t3 = wrapTo2Pi(alpha - beta - t1 + wrapTo2Pi(t2));
            end
        end

        function [t1, t2, t3, mode] = LRL(obj, alpha, beta, d)
            sa = sin(alpha);
            sb = sin(beta);
            ca = cos(alpha);
            cb = cos(beta);
            c_ab = cos(alpha - beta);
            mode = ['L', 'R', 'L'];

            tmp_lrl = (6.0 - (d * d) + (2.0 * c_ab) + (2.0 * d * (- sa + sb))) / 8.0;

            if (abs(tmp_lrl) > 1.0)
                [t1, t2, t3] = deal(nan, nan, nan);
            else
                t2 = wrapTo2Pi((2 * pi) - acos(tmp_lrl));
                tmp0 = d + sa - sb;
                tmp1 = atan2((ca-cb), tmp0);
                t1 = wrapTo2Pi(-alpha - tmp1 + (t2/2.0));
                t3 = wrapTo2Pi(wrapTo2Pi(beta) - alpha - t1 + wrapTo2Pi(t2));
            end
        end
    end
end
%% References
% https://courses.cs.washington.edu/courses/cse490r/18wi/homework/dubins_path_planning.pdf
% https://gitlab.cs.washington.edu/cse490r_18wi/lab4/blob/master/src/Dubins.py