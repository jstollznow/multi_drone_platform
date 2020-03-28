
classdef mdp_api
    %MDP_API Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        APINode
        pub
        data_srv_cli
        list_srv_cli
        loop_rate_value
        loop_rate
        DroneDataMap = containers.Map('KeyType', 'uint64')
    end
    
    methods (Access = private)
        function msg = genAPImsg(obj, drone_id, msgType, varargin)
            p = inputParser;
            p.addParameter('posVel', [0.0, 0.0, 0.0]);
            p.addParameter('relative', 0.0);
            p.addParameter('yaw', 0.0);
            p.addParameter('duration', 0.0);
            parse(p, varargin{:});
            
            msg = rosmessage(obj.pub);
            
            msg.Header.Stamp.Sec = drone_id;
            msg.ChildFrameId = msgType;
            
            msg.Transform.Translation.X = p.Results.posVel(1);
            msg.Transform.Translation.Y = p.Results.posVel(2);
            msg.Transform.Translation.Z = p.Results.posVel(3);
            
            msg.Transform.Rotation.X = p.Results.relative;
            msg.Transform.Rotation.Z = p.Results.yaw;
            msg.Transform.Rotation.W = p.Results.duration;
        end

        function Encoded = encoderelative(obj, relative, keep_height)
            Encoded = ((1.0 * relative) + (2.0 * keep_height));
        end
    end
    
    methods
        function obj = mdp_api(UpdateRate, NodeName)
            %MDP_API Construct an instance of this class
            %   Detailed explanation goes here
            rosshutdown();
            rosinit();
            obj.APINode = ros.Node(NodeName);

            fprintf("Initialising Client API Connection\n");
            obj.pub = rospublisher('/mdp_api', 'geometry_msgs/TransformStamped');
            obj.list_srv_cli = rossvcclient('/mdp_api_list_srv');
            obj.data_srv_cli = rossvcclient('/mdp_api_data_srv');

            obj.loop_rate_value = UpdateRate;
            obj.loop_rate = rosrate(UpdateRate);

            pause(1.0);
            fprintf("Initialised Client API Connection\n");
            % pause for 1 seconds to ensure the publisher has initialised
        end
        
        function delete(obj)
            fprintf("Shutting Down Client API Connection\n");
            Drones = getalldrones(obj);
            for i = 1 : size(Drones)
                cmdland(obj, Drones(i));
            end
            for i = 1 : size(Drones)
                sleepuntilidle(obj, Drones(i));
            end
            fprintf("Finished Client API Connection\n");
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
                CellArr = id_arr(2);
                id.Name = CellArr{:};
                DroneList = [DroneList id];

                % if drone does not exist in DroneDataMap, then add it
                if not iskey(obj.DroneDataMap, id.NumericId)
                    DroneData = mdp_drone_data(obj.APINode, id.NumericId);
                    obj.DroneDataMap(id.NumericId) = DroneData;
                end
            end
        end

        function Pos = getposition(obj, drone_id)
            Pos = mdp_position_data;

            if (iskey(obj.DroneDataMap, drone_id.NumericId))
                Msg = obj.DroneDataMap(drone_id.NumericId).getlatestpose();

                Pos.DroneID = drone_id.NumericId;
                Pos.TimeStampNSec = Msg.Header.Stamp.tonsec();
                Pos.X = Msg.Pose.Position.X;
                Pos.Y = Msg.Pose.Position.Y;
                Pos.Z = Msg.Pose.Position.Z;
                Pos.Yaw = Msg.Pose.Orientation.Y;
            end
        end

        function setposition(obj, drone_id, position_msg)
            EncodedRel = obj.encoderelative(position_msg.Relative, position_msg.KeepHeight);
            
            msg = genAPImsg(obj, drone_id.NumericId, "POSITION", 'posVel', position_msg.Position, 'duration', position_msg.Duration, 'yaw', position_msg.Yaw, 'relative', EncodedRel);
            
            send(obj.pub, msg);
        end    
        
        function Vel = getvelocity(obj, drone_id)
            Vel = mdp_velocity_data;

            if (iskey(obj.DroneDataMap, drone_id.NumericId))
                Msg = obj.DroneDataMap(drone_id.NumericId).getlatesttwist();

                Vel.DroneID = drone_id.NumericId;
                Vel.TimeStampNSec = Msg.Header.Stamp.tonsec();
                Vel.X = Msg.Twist.Linear.X;
                Vel.Y = Msg.Twist.Linear.Y;
                Vel.Z = Msg.Twist.Linear.Z;
                Vel.YawRate = Msg.Twist.Angular.Y;
            end
        end

        function setvelocity(obj, drone_id, velocity_msg)
            EncodedRel = obj.encoderelative(velocity_msg.Relative, velocity_msg.KeepHeight);
            
            msg = genAPImsg(obj, drone_id.NumericId, "VELOCITY", 'posVel', velocity_msg.Velocity, 'duration', velocity_msg.Duration, 'yaw', velocity_msg.YawRate, 'relative', EncodedRel);

            send(obj.pub, msg);
        end
        
        function cmdtakeoff(obj, drone_id, height, duration)
            msg = genAPImsg(obj, drone_id.NumericId, "TAKEOFF", 'posVel', [0.0, 0.0, height], 'duration', duration);
            send(obj.pub, msg);
        end
        
        function cmdland(obj, drone_id, duration)
            msg = genAPImsg(obj, drone_id.NumericId, "LAND", 'duration', duration);
            send(obj.pub, msg);
        end
        
        function cmdemergency(obj, drone_id)
            msg = genAPImsg(obj, drone_id.NumericId, "EMERGENCY");
            send(obj.pub, msg);
        end
        
        function cmdhover(obj, drone_id, duration)
            msg = genAPImsg(obj, drone_id.NumericId, "HOVER", 'duration', duration);
            send(obj.pub, msg);
        end
        
        function sethomeposition(obj, drone_id, position_msg)
            EncodedRel = obj.encoderelative(position_msg.Relative, position_msg.KeepHeight);
            msg = genAPImsg(obj, drone_id.NumericId, "SET_HOME", 'posVel', position_msg.Position, 'yaw', position_msg.Yaw, 'relative', EncodedRel);
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
        end
        
        function cmdgohome(obj, drone_id, height, duration)
            msg = genAPImsg(obj, drone_id.NumericId, "GOTO_HOME", 'posVel', [0.0, 0.0, height],'duration', duration);
            send(obj.pub, msg);
        end
        
        function setdroneserverfrequency(obj, freq)
            msg = genAPImsg(obj, 0, "DRONE_SERVER_FREQ", 'posVel', [freq, 0.0, 0.0]);
            send(obj.pub, msg);
        end

        function Timings = getservertimings(obj)
            req = rosmessage(obj.data_srv_cli);
            req.Goal.Header.FrameId = 'TIME';
            
            res = call(obj.data_srv_cli, req);

            Timings = mdp_timings;

            Timings.DesiredDroneServerUpdateRate = res.Plan.Poses(1).Pose.Position.X;
            Timings.ActualDroneServerUpdateRate = res.Plan.Poses(1).Pose.Position.Y;
            Timings.MoCapUpdateRate = res.Plan.Poses(1).Pose.Position.Z;
            Timings.TimeToUpdateDrones = res.Plan.Poses(1).Pose.Orientation.X;
            Timings.WaitTimePerFrame = res.Plan.Poses(1).Pose.Orientation.Y;
        end

        function spinuntilrate(obj)
            obj.loop_rate.waitfor();
        end

        function sleepuntilidle(obj, drone)
            fprintf("Sleeping until drone '%s' goes idle\n", drone.Name);

            obj.loop_rate.reset();
            obj.loop_rate.waitfor();

            StateParam = strcat("mdp/drone_", num2str(drone.NumericId), "/state");
            DroneState = "";
            Ptree = rosparam();
            DroneState = Ptree.get(StateParam);
            while true
                if (DroneState == "DELETED" || DroneState == "LANDED" || DroneState == "HOVERING" || DroneState == "" || DroneState == "UNKNOWN")
                    break;
                end

                obj.spinuntilrate();
                DroneState = Ptree.get(StateParam);
            end
        end

        function State = getstate(obj, drone)
            StateParam = strcat("mdp/drone_", num2str(drone.NumericId), "/state");
            Ptree = rosparam();
            StrState = Ptree.get(StateParam);
            State = mdp_flight_state.convertstringtoflightstate(StrState);
        end
    end
end

