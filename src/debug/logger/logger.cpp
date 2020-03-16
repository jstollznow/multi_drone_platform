#include "logger.h"

void logger::post_log(log_type type, std::string caller, std::string message, ros::Publisher& logPublisher)
{
    std::map<log_type,std::string> logLevel = {
        {INFO, "INFO"}, {DEBUG, "DEBUG"},
        {WARN, "WARN"}, {ERROR, "ERROR"}
    };

    multi_drone_platform::log myLogPost;
    myLogPost.type = logLevel[type];
    myLogPost.timeStamp = ros::Time::now();
    myLogPost.logMessage = message;
    logPublisher.publish(myLogPost);

    switch (type) {
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