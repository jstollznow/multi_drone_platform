
classdef mdp_timings
    properties
        TimeStampNSec = 0
        MoCapUpdateRate = 0
        DesiredDroneServerUpdateRate = 0
        ActualDroneServerUpdateRate = 0
        TimeToUpdateDrones = 0
        WaitTimePerFrame = 0
    end

    methods
        function valid = isvalid(obj)
            valid = (obj.TimeStampNSec > 0);
        end
    end
end