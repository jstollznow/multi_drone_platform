
classdef mdp_id
    properties
        numeric_id {mustBeNumeric}
        drone_name
    end
    
    methods
        function id = getRosID(obj)
            id = rosmessage("multi_drone_platform/mdpID");
            id.numeric_id = obj.numeric_id;
            id.name = obj.drone_name;
        end
    end
end