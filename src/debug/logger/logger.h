
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
    static void post_log(const log_type type, const std::string& caller, const ros::Publisher& logPublisher, const std::string& message);
};