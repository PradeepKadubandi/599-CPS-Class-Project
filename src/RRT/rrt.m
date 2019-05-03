classdef rrt < handle
    properties(SetAccess = private)
        minTurning = -1;
        startPos = [];
        goalPos = [];
        nodes = {[]};
        % edges = {[]};
        iterations = -1;
        costmap = nan;
        s = [];
        t = [];
        weight=[];
    end
    
    methods
        % constructor for rrt
        function RRT= rrt(costmap, iterations, startPos, goalPos, minTurning)
            RRT.costmap = costmap;
            RRT.iterations = iterations;
            RRT.startPos = startPos;
            RRT.goalPos = goalPos;
            RRT.minTurning = minTurning;
        end
        
        function [path, Tree] = run(Tree)
            i = 2;
            path = {[]};
            Tree.nodes{1} = Tree.startPos;
            while i < Tree.iterations
                rand = random(Tree);
                [near, distance] = nearestNeighbor(Tree, rand);
                newNode = generateNewNode(Tree, rand, Tree.nodes{near});
                Tree.nodes{i} = newNode;
                % add new node to list of nodes
                % Tree.edges{i-1} = [near, distance, newNode]; 

                %add an edge btw nearst node in tree and new node
                Tree.s(i-1) = near;
                Tree.t(i-1) = i;
                Tree.weight(i-1) = distance;
                i = i+1;
                if reachedGoal(Tree)
                    path = findPath(Tree);
                    break;
                end
            end
        end
        
        %returns a path from startPos to goalPos
        function pathPos = findPath(Tree)
            pathPos={[]};
            % fprintf('number of nodes in graph: %.1f\n', length(Tree.nodes));
            
            G = graph(Tree.s, Tree.t, Tree.weight);
            path = shortestpath(G, 1, length(Tree.nodes));
            % disp(path);
            for i = 1:length(path)
                pathPos{i} = Tree.nodes{path(i)};
            end
        end
        
        % return the nearest node in tree
        function [near, distance] = nearestNeighbor(Tree, rand_node)
            distance = realmax;
            near = -1;
            for i = 1:length(Tree.nodes)
                node = Tree.nodes{i};
                dis = sqrt(double((node(1) - rand_node(1))^2) + double((node(2) - rand_node(2))^2)); 
                if(dis < distance)
                    near = i;
                    distance = dis;
                end
            end
%             fprintf('shortest distance: %.2f\n', distance);
%             fprintf('nearest node index: %.2f\n ', near);
            
        end
        
        function node = generateNewNode(Tree, rand, near)
            node = [rand(1), rand(2), -1];
%             dis = sqrt((rand(1) - near(1))^2 - (rand(2) - near(2))^2); 
            node(3) = fulltan(double(rand(2) - near(2)), double(rand(1) - near(1)));
        end
        
        %generate a random node to explore, this method only checks if the random node is free not the entire path
        function rand_node = random(Tree)
            sz = size(Tree.costmap);
            x = sz(1);
            y = sz(2);
            rand_node = [-1, -1];
            while true
                x_rand = rand * x;
                y_rand = rand * y;
                curr_node = [int64(x_rand(1)), int64(y_rand(1))];
                if curr_node(1) == 0
                    curr_node(1) = 1;
                end
                if curr_node(2) == 0
                    curr_node(2) = 1;
                end
                flag = false; %flag to mark if node is already explored
                for i = 1:length(Tree.nodes)
                    if(isequal(Tree.nodes{i},curr_node))
                        flag = true;
                        break;
                    end
                end
                if(~flag && isFree(Tree, curr_node))
                    rand_node = double(curr_node);
                    break;
                end
            end
        end
        
        %check if this point is collision free
        function flag = isFree(Tree, node)
            x = node(1);
            y = node(2);
            if(Tree.costmap(x,y) == 1)
                flag = false;
                return
            else
                flag = true;
                return
            end
       end
        
        %returns true if state is reached and add goalPos to tree
        function [flag, dis] = reachedGoal(Tree)
            i = length(Tree.nodes);
            lastNode = Tree.nodes{i};
            dis = sqrt(double((lastNode(1) - Tree.goalPos(1))^2) + double((lastNode(2) - Tree.goalPos(2))^2)); 
            if(dis <= Tree.minTurning)
                Tree.nodes{i+1} = Tree.goalPos;
%                 Tree.edges{i} = [lastNode, Tree.nodes{i+1}];
                Tree.s(i) = i;
                Tree.t(i) = i+1;
                Tree.weight(i) = dis;
                flag = true;
            else
                flag = false;
            end
        end
        
    end    
end