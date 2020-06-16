#include "rigidbody.h"
#include <boost/asio.hpp>

/* ensure the name of this file is identical to the name of the class, this will be the tag of the new drone. If this
 * file is included in the /wrappers/ folder, then it will automatically be compiled with the drone server. When developing
 * a new drone wrapper, run catkin_make in the base folder of the catkin workspace to recompile the platform including the new drone.
 */

// @TODO: WARNING, the tello wrapper has not been completed. test this entire system.
class DRONE_WRAPPER(tello, ip_address)
private:
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket{io_service};
    boost::asio::ip::udp::endpoint endpoint;


    bool send_message_to_drone(const std::string& message) {
        if (!this->socket.is_open()) {
            this->log(logger::log_type::WARN, "Tello socket is not open");
            return false;
        }

        boost::system::error_code err;
        this->socket.send_to(boost::asio::buffer(message), this->endpoint, 0, err);
        if (err.value() != boost::system::errc::success) {
            std::stringstream ss;
            ss << "Error sending message to tello '" << this->get_tag() << "': " << err.message();
            this->log(logger::log_type::WARN, ss.str());
            return false;
        }
        return true;
    }

public:
    void on_init(std::vector<std::string> args) final {
        const std::string& ip_address = args[0];

        boost::asio::ip::address address = boost::asio::ip::address::from_string(ip_address.c_str());
        if (!address.is_v4()) {
            this->log(logger::log_type::ERROR, "Tello was given an invalid IPV4 address");
            return;
        }

        endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(ip_address.c_str()), 8889);
        socket.open(boost::asio::ip::udp::v4());
        if (!send_message_to_drone("Command")) {
            // @TODO: failed to send a message to the drone...
        }
    };

    void on_deinit() final {
        if (this->socket.is_open()) {
            this->socket.close();
        }
    }

    void on_set_position(geometry_msgs::Vector3 pos, float yaw, float duration, bool isRelative) final {
        if (!isRelative) {
            // force relative
            pos.x -= this->currentPose.position.x;
            pos.y -= this->currentPose.position.y;
            pos.z -= this->currentPose.position.z;
        }
        // convert from meters to centimeters
        pos.x /= 100.0;
        pos.y /= 100.0;
        pos.z /= 100.0;

        const double movement_magnitude = std::sqrt((pos.x * pos.x) + (pos.y * pos.y) + (pos.z * pos.z));
        const int speed_cm = movement_magnitude / duration;

        char message_buffer[64];
        sprintf(message_buffer,"go %d %d %d %d",
                (int)std::round(pos.x), (int)std::round(pos.y), (int)std::round(pos.z), speed_cm);
        if (!send_message_to_drone(message_buffer)) {
            // @TODO: failed to send message to drone...
        }
    }

    void on_set_velocity(geometry_msgs::Vector3 vel, float yawrate, float duration, bool relativeHeight) final {
        if (!relativeHeight) {
            // force relative
            vel.z -= this->currentPose.position.z;
        }
        // convert from meters to centimeters
        vel.x /= 100.0;
        vel.y /= 100.0;
        vel.z /= 100.0;

        const double movement_magnitude = std::sqrt((vel.x * vel.x) + (vel.y * vel.y) + (vel.z * vel.z));
        const int speed_cm = (int)std::floor(movement_magnitude / duration);

        char message_buffer[64];
        sprintf(message_buffer,"go %d %d %d %d",
                (int)std::round(vel.x), (int)std::round(vel.y), (int)std::round(vel.z), speed_cm);
        if (!send_message_to_drone(message_buffer)) {
            // @TODO: failed to send message to drone...
        }
    }

    void on_motion_capture(const geometry_msgs::PoseStamped& msg) final {

    }

    void on_update() final {
        /* @TODO: perform movement adjustments based on differences between optitrack recorded tello location and
         * what it's desired is. i.e. if the desired position is {0, 0, 1} but its optitrack recorded is {0, 0, 0.9} and
         * the drone is in state HOVERING, then push a command to the tello to move the extra 0.1 to match the desired
         * position of {0, 0, 1}. This has to be done here as the tello itself does not manage motion capture internally
         * as the crazyflie does.
        */
    }

    void on_takeoff(float height, float duration) final {
        if (!send_message_to_drone("takeoff")) {
            // @TODO: failed to send message to drone...
        }
    }

    void on_land(float duration) final {
        if (!send_message_to_drone("land")) {
            // @TODO: failed to send message to drone...
        }
    }

    void on_emergency() final {
        if (!send_message_to_drone("emergency")) {
            // @TODO: failed to send message to drone...
        }
    }
};


