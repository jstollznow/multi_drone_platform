#include "logger.h"

void logger::postLog(logType type, std::string caller, std::string message, ros::Publisher LogPublisher)
{
    multi_drone_platform::log myLogPost;
    myLogPost.type = type;
    myLogPost.timeStamp = ros::Time::now().toSec();
    myLogPost.logMessage = message;

    LogPublisher.publish(myLogPost);

    switch (type)
    {
        case INFO:
        case DEBUG:
            ROS_INFO("%s: %s", caller.c_str(), message.c_str());
            break;
        case WARN:
            ROS_WARN("%s: %s", caller.c_str(), message.c_str());
            break;
        case ERROR:
            ROS_ERROR("%s: %s", caller.c_str(), message.c_str());
            break;
    }


}