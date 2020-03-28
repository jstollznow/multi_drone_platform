
classdef mdp_drone_data
    properties (GetAccess=private)
        PoseSubscriber
        TwistSubscriber
    end

    methods
        function obj = mdp_drone_data(RosNode, DroneID)
            TopicHeader = strcat('mdp/drone_', num2str(DroneID));
            PoseSubscriber = ros.Subscriber(RosNode, strcat(TopicHeader, '/curr_pose'));
            TwistSubscriber = ros.Subscriber(RosNode, strcat(TopicHeader, '/curr_twist'));
        end

        function Pose = getlatestpose(obj)
            Pose = PoseSubscriber.LatestMessage;
        end

        function Twist = getlatesttwist(obj)
            Twist = TwistSubscriber.LatestMessage;
        end
    end
end