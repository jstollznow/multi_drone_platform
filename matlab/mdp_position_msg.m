
classdef mdp_position_msg
    properties
        Position = zeros(1, 3)
        Relative = false
        KeepHeight = false
        Duration = 0.0
        Yaw = 0.0
    end
end
