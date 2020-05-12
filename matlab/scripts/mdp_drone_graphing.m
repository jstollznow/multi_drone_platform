classdef mdp_drone_graphing
    %MDP_DRONE_GRAPHING Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(SetAccess = private)
        DroneID
        X;
        Y;
        Z;
        Time;
    end
    
    methods
        function obj = mdp_drone_graphing(id)
            %MDP_DRONE_GRAPHING Construct an instance of this class
            %   Detailed explanation goes here
            obj.DroneID = id;
            obj.X = [];
            obj.Y = [];
            obj.Z = [];
            obj.Time = [];
        end
        
        function obj = add_data(obj, pos)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.X = [obj.X, pos.X];
            obj.Y = [obj.Y, pos.Y];
            obj.Z = [obj.Z, pos.Z];
            obj.Time = [obj.Time, pos.TimeStampSec];
        end
        
        function XArr = get_X(obj)
            XArr = obj.X;
        end
        
        function YArr = get_Y(obj)
            YArr = obj.Y;
        end
        
        function ZArr = get_Z(obj)
            ZArr = obj.Z;
        end
        
        function TArr = get_Time(obj)
            TArr = obj.Time;
        end
        
        function id = get_ID(obj)
           id = obj.DroneID; 
        end
    end
end

