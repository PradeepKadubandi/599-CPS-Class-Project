classdef rrt < handle
    properties(SetAccess = private)
        minTurning = -1;
        startPos = [];
        goalPos = [];
        nodes = {[]};
        iterations = -1;
        costmap = nan;
        s = [];
        t = [];
        weight=[];
        distance = -1;
        tolerance = -1;
    end
    
    methods
        % constructor for rrt
        function RRT= rrt(costmap, iterations, startPos, goalPos, minTurning, distance, tolerance)
            RRT.costmap = costmap;
            RRT.iterations = iterations;
            RRT.startPos = startPos;
            RRT.goalPos = goalPos;
            RRT.minTurning = minTurning;
            RRT.distance = distance;
            RRT.tolerance = tolerance;
        end
        
        function [path, Tree] = run(Tree)
            goalFlag = false;
            i = 2;
            path = {[]};
            Tree.nodes{1} = Tree.startPos;
            while i < Tree.iterations
                rand = random(Tree);
                [near, disFlag] = nearestNeighbor(Tree, rand); 
                if ~disFlag %true = satisfies min distance requirement
                    continue;
                end
                [newNode, newFlag] = generateNewNode(Tree, rand, near);
                if ~newFlag
                    continue;
                end
                i = i+1;
                if reachedGoal(Tree, near)
                    goalFlag = true;
%                     path = findPath(Tree);
%                     break;
                end
            end
            if goalFlag
                path = findPath(Tree);
            end
        end
        
        %returns a path from startPos to goalPos
        function pathPos = findPath(Tree)
            pathPos={[]};
            G = graph(Tree.s, Tree.t, Tree.weight);
            [goalIndex, flag] = findGoalNode(Tree);
            if flag
                path = shortestpath(G, 1, goalIndex);
                for i = 1:length(path)
                    pathPos{i} = Tree.nodes{path(i)};
                end
            end
        end
        
        function [index, flag] = findGoalNode(Tree)
            index = -1; 
            flag = false;
            for i = 1:length(Tree.nodes)
                if(isequal(Tree.nodes{i},Tree.goalPos))
                    index = i;
                    flag = true;
                    break;
                end
            end
        end
        
        % return the nearest node in tree that is at least as far as
        % specified connection distance
        function [near, flag] = nearestNeighbor(Tree, rand_node)
            dist = realmax;
            flag = true;
            near = -1;
            for i = 1:length(Tree.nodes)
                node = Tree.nodes{i};
                dis = sqrt(double((node(1) - rand_node(1))^2) + double((node(2) - rand_node(2))^2)); 
                if(dis < dist)
                    near = i;
                    dist = dis;
                end
            end
            if dist >= Tree.distance
               flag = true;
            end
%             fprintf('shortest distance: %.2f\n', distance);
%             fprintf('nearest node index: %.2f\n ', near);
            
        end
        
        function [node, flag] = generateNewNode(Tree, rand, near)
            flag = false;
            node = [rand(1), rand(2), -1];
            dis = sqrt(double((Tree.nodes{near}(1) - rand(1))^2) + double((Tree.nodes{near}(2) - rand(2))^2)); 
            if dis >= Tree.minTurning
                i = length(Tree.nodes);
                node(3) = fulltan(double(rand(2) - Tree.nodes{near}(2)), double(rand(1) - Tree.nodes{near}(1)));
                Tree.nodes{i+1} = node;
                Tree.s(i) = near;
                Tree.t(i) = i+1;
                Tree.weight(i) = dis;
                flag = true;
                return;
            end
        end
        
        %generate a random node to explore, this method only checks if the random node is free not the entire path
        function rand_node = random(Tree)
             near_index = ceil(rand * length(Tree.nodes));
             rand_node_in_tree = Tree.nodes{near_index};
%             sz = size(Tree.costmap);
%             x = sz(1)
%             y = sz(2)
            rand_node = [-1, -1];
            while true
                r = -2*Tree.distance + (4*Tree.distance)*rand(2,1);
                x_rand = rand_node_in_tree(1) + r(1);
                y_rand = rand_node_in_tree(2) + r(2);
%                 x_rand = rand*x;
%                 y_rand = rand*y;
%                 curr_node = int64([x_rand(1), y_rand(1)]);
                %if not inbound, generate again
                if ~inBound(Tree, x_rand, y_rand)
                    continue;
                end
                
                dis = sqrt(double((x_rand - rand_node_in_tree(1))^2) + double(y_rand - rand_node_in_tree(2))^2); 
                if dis < Tree.distance
                    continue;
                end 
                curr_node = [int64(x_rand), int64(y_rand)];
                if ~inTree(Tree, curr_node)
                    if parking.isFree(Tree.costmap, curr_node)
                        rand_node = double(curr_node);
%                         fprintf('generated new node dis is %.1f', dis);
                        break;
                    end
                end
            end
            
        end
        
        function flag = inBound(Tree, x, y)
            sz = size(Tree.costmap);
            x_map = sz(1);
            y_map = sz(2);
            if x <= x_map && y <= y_map && x >=1 && y >=1
                flag = true;
            else
                flag = false;
            end
        end
        
        function flag = inTree(Tree, curr_node)
            flag = false; %flag to mark if node is already explored
            for i = 1:length(Tree.nodes)
                if(isequal(Tree.nodes{i},curr_node))
                    flag = true;
                    break;
                end
            end
        end
        
        %returns true if state is reached and add goalPos to tree
        function [flag, distance] = reachedGoal(Tree, near)
            i = length(Tree.nodes);
            distance = -1;
            flag = false;
            if i <= 1
                return;
            end
            lastNode = Tree.nodes{i};
            dis = sqrt(double((lastNode(1) - Tree.goalPos(1))^2) + double((lastNode(2) - Tree.goalPos(2))^2));
            if(dis <= Tree.tolerance)
                nearNode = Tree.nodes{near};
                dis2 = sqrt(double((nearNode(1) - Tree.goalPos(1))^2) + double((nearNode(2) - Tree.goalPos(2))^2));
                if(dis2 >= Tree.minTurning)
                    Tree.nodes{i} = Tree.goalPos;
                    Tree.t(i-1) = i;
%                      disp(Tree.nodes{Tree.s(i-1)});
%                      disp(Tree.nodes{Tree.t(i-1)});
                    Tree.weight(i-1) = dis2;
                    flag = true;
                end
            else
                flag = false;
            end
        end
        
    end    
end