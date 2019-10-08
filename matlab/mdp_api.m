classdef mdp_api
    %MDP_API Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        APIpub
        APIsrv
    end
    
    methods
        function obj = mdp_api()
            %MDP_API Construct an instance of this class
            %   Detailed explanation goes here
            obj.APIpub = rospublisher('/mdp_api', 'multi_drone_platform/inputAPI');
            obj.APIsrv = rossvcclient('/mdp_api_srv');
            % pause for 2 seconds to ensure the publisher has initialised
            pause(2)
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

