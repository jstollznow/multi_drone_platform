
classdef mdp_position_data
    properties
        DroneID = 0
        TimeStampNSec = 0
        X = 0.0
        Y = 0.0
        Z = 0.0
        Yaw = 0.0
    end

    methods
        function valid = isvalid(obj)
            valid = (obj.TimeStampNSec > 0);
        end
    end
end
