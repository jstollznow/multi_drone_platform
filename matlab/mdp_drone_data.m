
classdef mdp_drone_data
    properties (GetAccess=private)
        PoseSubscriber
        TwistSubscriber
    end

    methods
        function obj = mdp_drone_data(RosNode, DroneID)
            TopicHeader = strcat('/mdp/drone_', num2str(DroneID));
            obj.PoseSubscriber = robotics.ros.Subscriber(RosNode, strcat(TopicHeader, '/curr_pose'))
            obj.TwistSubscriber = robotics.ros.Subscriber(RosNode, strcat(TopicHeader, '/curr_twist'))
        end

        function Pose = getlatestpose(obj)
            Pose = obj.PoseSubscriber.LatestMessage;
        end

        function Twist = getlatesttwist(obj)
            Twist = obj.TwistSubscriber.LatestMessage;
        end
    end
end