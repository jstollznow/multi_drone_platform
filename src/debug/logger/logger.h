
#include <ros/ros.h>
#include "multi_drone_platform/log.h"

class logger {
    public:
    /**
     * Used to define different verbosity levels for the global logging procedure.
     */
    enum log_type {
        INFO, 
        DEBUG, 
        WARN, 
        ERROR
    };

    /**
     * Generic function used for logging across different aspects of the platform. This method ensures consistent
     * textual feedback to the user.
     * @param type The log verbosity level.
     * @param caller The aspect of the platform which this message is associated with.
     * @param logPublisher A publisher which is used to publish the message
     * @param message The string which is the message itself.
     */
    static void post_log(const log_type type, const std::string& caller, const ros::Publisher& logPublisher, const std::string& message);
};