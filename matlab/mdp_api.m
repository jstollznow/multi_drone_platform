
classdef mdp_api
    %MDP_API Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        pub
        data_srv_cli
        list_srv_cli
    end
    
    methods (Access = private)
        function msg = genAPImsg(obj, drone_id, msg_type, varargin)
            p = inputParser;
            p.addOptional('posvel', [0.0, 0.0, 0.0]);
            p.addOptional('heading', [0.0, 0.0]);
            p.addOptional('yaw_rate', 0.0);
            p.addOptional('duration', 0.0);
            parse(p);
            
            msg = rosmessage(obj.pub);
            
            msg.Header.Stamp.Sec = drone_id;
            msg.ChildFrameId = msg_type;
            
            msg.Transform.Translation.X = p.Results.posvel(1);
            msg.Transform.Translation.Y = p.Results.posvel(2);
            msg.Transform.Translation.Z = p.Results.posvel(3);
            
            msg.Transform.Rotation.X = p.Results.heading(1);
            msg.Transform.Rotation.Y = p.Results.heading(2);
            msg.Transform.Rotation.Z = p.Results.yaw_rate;
            msg.Transform.Rotation.W = p.Results.duration;
        end
    end
    
    methods
        function obj = mdp_api()
            %MDP_API Construct an instance of this class
            %   Detailed explanation goes here
            rosinit();
            obj.pub = rospublisher('/mdp_api', 'geometry_msgs/TransformStamped');
            obj.list_srv_cli = rossvcclient('/mdp_api_list_srv');
            obj.data_srv_cli = rossvcclient('/mdp_api_data_srv');
            % pause for 2 seconds to ensure the publisher has initialised
        end
        
        function delete(obj)
            rosshutdown();
        end

        function DroneList = getalldrones(obj)
            req = rosmessage('tf2_msgs/FrameGraphRequest');
            res = call(obj.list_srv_cli, req);
            
            drones_str = split(res.FrameYaml, ' ');

            DroneList = [];
            for i = 1 : size(drones_str)
                if (drones_str(i) == "")
                    continue
                end
                id_arr = split(drones_str(i), ':');
                id = mdp_id(0, '');
                id.NumericId = str2double(id_arr(1));
                id.Name = id_arr(2);
                DroneList = [DroneList id];
            end
        end

        function Pos = getposition(obj, drone_id)
            req = rosmessage(obj.data_srv_cli);
            req.Start.Header.Stamp.Sec = drone_id.NumericId;
            req.Goal.Header.FrameId = 'POSITION';
            
            res = call(obj.data_srv_cli, req);
            
            Pos = mdp_position_data;
            Pos.X = res.Plan.Poses(1).Pose.Position.X;
            Pos.Y = res.Plan.Poses(1).Pose.Position.Y;
            Pos.Z = res.Plan.Poses(1).Pose.Position.Z;
            
            Pos.HeadingX = res.Plan.Poses(1).Pose.Orientation.X;
            Pos.HeadingY = res.Plan.Poses(1).Pose.Orientation.Y;
        end

        function setposition(obj, drone_id, pos_x, pos_y, pos_z, duration)
            msg = genAPImsg(obj, drone_id.NumericId, "POSITION", 'posvel', [pos_x, pos_y, pos_z], 'duration', duration);
            send(obj.pub, msg);
        end    
        
        function Vel = getvelocity(obj, drone_id)
            req = rosmessage(obj.data_srv_cli);
            req.Start.Header.Stamp.Sec = drone_id.NumericId;
            req.Goal.Header.FrameId = 'VELOCITY';
            
            res = call(obj.data_srv_cli, req);
            
            Vel = mdp_velocity_data;
            Vel.X = res.Plan.Poses(1).Pose.Position.X;
            Vel.Y = res.Plan.Poses(1).Pose.Position.Y;
            Vel.Z = res.Plan.Poses(1).Pose.Position.Z;
            
            Vel.HeadingX = res.Plan.Poses(1).Pose.Orientation.X;
            Vel.HeadingY = res.Plan.Poses(1).Pose.Orientation.Y;
            Vel.YawRate = res.Plan.Poses(1).Pose.Orientation.Z;
        end

        function setvelocity(obj, drone_id, vel_x, vel_y, vel_z, yaw_rate, duration)
            msg = genAPImsg(obj, drone_id.NumericId, "VELOCITY", 'posvel', [vel_x, vel_y, vel_z], 'yaw_rate', yaw_rate, 'duration', duration);
            send(obj.pub, msg);
        end
        
        function cmdtakeoff(obj, drone_id)
            msg = genAPImsg(obj, drone_id.NumericId, "TAKE_OFF");
            send(obj.pub, msg);
        end
        
        function cmdland(obj, drone_id)
            msg = genAPImsg(obj, drone_id.NumericId, "LAND");
            send(obj.pub, msg);
        end
        
        function cmdemergency(obj, drone_id)
            msg = genAPImsg(obj, drone_id.NumericId, "EMERGENCY");
            send(obj.pub, msg);
        end
        
        function cmdhover(obj, drone_id)
            msg = genAPImsg(obj, drone_id.NumericId, "HOVER");
            send(obj.pub, msg);
        end
        
        function sethomeposition(obj, drone_id, pos_x, pos_y, pos_z, heading_x, heading_y)
            msg = genAPImsg(obj, drone_id.NumericId, "SET_HOME", 'posvel', [pos_x, pos_y, pos_z], 'heading', [heading_x, heading_y]);
            send(obj.pub, msg);
        end
        
        function Pos = gethomeposition(obj, drone_id)
            req = rosmessage(obj.data_srv_cli);
            req.Start.Header.Stamp.Sec = drone_id.NumericId;
            req.Goal.Header.FrameId = 'GET_HOME';
            
            res = call(obj.data_srv_cli, req);
            
            Pos = mdp_position_data;
            Pos.X = res.Plan.Poses(1).Pose.Position.X;
            Pos.Y = res.Plan.Poses(1).Pose.Position.Y;
            Pos.Z = res.Plan.Poses(1).Pose.Position.Z;
            
            Pos.HeadingX = res.Plan.Poses(1).Pose.Orientation.X;
            Pos.HeadingY = res.Plan.Poses(1).Pose.Orientation.Y;
        end
        
        function cmdgohome(obj, drone_id)
            msg = genAPImsg(obj, drone_id.NumericId, "GOTO_HOME");
            send(obj.pub, msg);
        end
        
    end
end

