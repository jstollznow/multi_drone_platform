
classdef mdp_velocity_data
    properties
        DroneID = 0
        TimeStampSec = 0.0
        X = 0.0
        Y = 0.0
        Z = 0.0
        YawRate = 0.0
    end

    methods
        function valid = isvalid(obj)
            valid = (obj.TimeStampSec > 0.0);
        end
    end
end
