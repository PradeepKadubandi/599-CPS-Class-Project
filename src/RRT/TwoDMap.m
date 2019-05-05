classdef TwoDMap
    %TWODMAP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(SetAccess = private)
        topLevelMap;
    end
    
    methods
        function obj = TwoDMap()
            %TWODMAP Construct an instance of this class
            %   Detailed explanation goes here
            obj.topLevelMap = containers.Map('KeyType', 'uint32', 'ValueType', 'any');
        end
        
        function put(obj, key, value)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            f_key = key(1);
            s_key = key(2);
            if (~isKey(obj.topLevelMap, f_key))
                obj.topLevelMap(f_key) = containers.Map('KeyType', 'uint32', 'ValueType', 'any');
            end
            secondLevelMap = obj.topLevelMap(f_key);
            secondLevelMap(s_key) = value;
        end
        
        function value = get(obj, key)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            f_key = key(1);
            s_key = key(2);
            if (~isKey(obj.topLevelMap, f_key))
                ME = MException('TwoDMap:KeyNotFound', ...
                    'Key %i not found', key);
                throw(ME);
            end
            secondLevelMap = obj.topLevelMap(f_key);
            if (~isKey(secondLevelMap, s_key))
                ME = MException('TwoDMap:KeyNotFound', ...
                    'Key %i not found', key);
                throw(ME);
            end
            value = secondLevelMap(s_key);
        end
    end
end

