// #include "ros/ros.h"
#include "rigidBody.h"
// #include "multi_drone_platform/inputAPI.h"
// #include "multi_drone_platform/movementFeedbackSRV.h"
// #include "geometry_msgs/PoseStamped.h"





int main(int argc, char **argv)
{
    // ros::init(argc, argv, "myWorld");

    // ros::NodeHandle Node;
    
    // ros::Publisher moCap = Node.advertise(, API_get_data_srv);
    // ros::Subscriber Sub = Node.subscribe(SUB_TOPIC, 10, &API_input);
    
    // ros::Rate LoopRate(LOOP_RATE_HZ);
    rigidBody cf1 = rigidBody("cflie1");
    std::cout>>cf1.getCurrPos();
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     LoopRate.sleep();
    // }

    return 0;
}