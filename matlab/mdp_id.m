
classdef mdp_id
    properties
        NumericId {mustBeNumeric}
        Name
    end
    
    methods
        function obj = mdp_id(numeric_id, name)
            obj.NumericId = numeric_id;
            obj.Name = name;
        end
    end
end