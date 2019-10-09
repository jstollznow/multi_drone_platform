
classdef mdp_api
    %MDP_API Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        pub
        data_srv_cli
        list_srv_cli
    end
    
    methods
        function obj = mdp_api()
            %MDP_API Construct an instance of this class
            %   Detailed explanation goes here
            rosinit();
            obj.pub = rospublisher('/mdp_api', 'geometry_msgs/Vector3');
            obj.data_srv_cli = rossvcclient('/mdp_api_srv');
            obj.list_srv_cli = rossvcclient('/mdp_api_list_srv');
            obj.data_srv_cli = rossvcclient('/mdp_api_data_srv');
            % pause for 2 seconds to ensure the publisher has initialised
            pause(2);
        end
        
        function delete(obj)
            rosshutdown();
        end

        function drone_list = getAllDrones(obj)
            
        end

        function pos = getPosition(obj, drone_id)

        end

        function setPosition(obj, drone_id, pos_x, pos_y, pos_z, duration)

        end    
        
        function vel = getVelocity(obj, drone_id)

        end

        function setVelocity(obj, drone_id, vel_x, vel_y, vel_z, yaw_rate)

        end
        
        function cmdTakeOff(obj, drone_id)
            msg = rosmessage(obj.pub);
            msg.msg_type = "TAKEOFF";
            drone_id_msg = rosmessage("multi_drone_platform/mdpID");
            msg.drone_id = drone_id;
            obj.pub.send(msg);
        end
        
        function cmdLand(obj, drone_id)
            
        end
        
        function cmdEmergency(obj, drone_id)
            
        end
        
        function cmdHover(obj, drone_id)
            
        end
        
        function setHomePosition(obj, drone_id, pos_x, pos_y, pos_z, forward_x, forward_y)
            
        end
        
        function pos = getHomePosition(obj, drone_id)
            
        end
        
        function cmdGoHome(obj, drone_id)
            
        end

        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

