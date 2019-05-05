classdef DubinsPathSegment
    %DUBINSPATHSEGMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(SetAccess = private)
        MotionLengths;
        MotionTypes;
        Distance;
        PathPoses;
    end
    
    methods
        function obj = DubinsPathSegment(motionLengths,motionTypes, distance, pathPoses)
            %DUBINSPATHSEGMENT Construct an instance of this class
            %   Detailed explanation goes here
            obj.MotionLengths = motionLengths;
            obj.MotionTypes = motionTypes;
            obj.Distance = distance;
            obj.PathPoses = pathPoses;
        end
    end
end

