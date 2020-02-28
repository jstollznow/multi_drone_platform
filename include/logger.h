
#include <ros/ros.h>
#include "multi_drone_platform/log.h"

class logger {
    public:
    enum logType {
        INFO, 
        DEBUG, 
        WARN, 
        ERROR
    };
    static void postLog(logType type, std::string caller, std::string message, ros::Publisher LogPublisher);
};