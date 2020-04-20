#include "logger.h"

std::map<logger::log_type,std::string> logLevel = {
        {logger::INFO, "INFO"}, {logger::DEBUG, "DEBUG"},
        {logger::WARN, "WARN"}, {logger::ERROR, "ERROR"}
};

void logger::post_log(const log_type type, const std::string& caller, const ros::Publisher& logPublisher, const std::string& message) {

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