
#include <ros/ros.h>
#include "multi_drone_platform/log.h"

class logger {
    public:
    enum log_type {
        INFO, 
        DEBUG, 
        WARN, 
        ERROR
    };
    static void post_log(log_type type, std::string caller, std::string message, ros::Publisher& logPublisher);
};