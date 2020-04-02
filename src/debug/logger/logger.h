
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
    static void post_log(log_type type, std::string caller, ros::Publisher &logPublisher, std::string message);
    template <class T>
    static void post_log(log_type type, std::string caller, ros::Publisher &logPublisher, std::string dataLabel, T data);
};