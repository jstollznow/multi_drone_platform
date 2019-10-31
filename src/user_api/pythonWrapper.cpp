#include <boost/python.hpp>
#include "../../include/user_api.h"

namespace py = boost::python;

BOOST_PYTHON_MODULE(mdp_api)
{

    // id
    py::class_<mdp_api::id>("id")
    .add_property("numeric_id",&mdp_api::id::numeric_id)
    .add_property("name", &mdp_api::id::name)
    ;

    // position_data
    py::class_<mdp_api::position_data>("position_data")
    .add_property("x",&mdp_api::position_data::x)
    .add_property("y",&mdp_api::position_data::y)
    .add_property("z",&mdp_api::position_data::z)
    .add_property("yaw",&mdp_api::position_data::yaw)
    ;

    // position_msg
    // std::array<double, 3> position = {{0.0, 0.0, 0.0}};
    py::class_<mdp_api::position_msg>("position_msg")
    .add_property("relative", &mdp_api::position_msg::relative)
    .add_property("keep_height", &mdp_api::position_msg::keep_height)
    .add_property("duration", &mdp_api::position_msg::duration)
    .add_property("yaw", &mdp_api::position_msg::yaw)
    ;
    // velocity_msg
    // std::array<double, 3> velocity = {{0.0, 0.0, 0.0}};
    py::class_<mdp_api::velocity_msg>("velocity_msg")
    .add_property("relative",&mdp_api::velocity_msg::relative)
    .add_property("keep_height", &mdp_api::velocity_msg::keep_height)
    .add_property("duration",&mdp_api::velocity_msg::duration)
    .add_property("yaw_rate", &mdp_api::velocity_msg::yaw_rate)
    ;
    

    // timings
    py::class_<mdp_api::timings>("timings")
    .add_property("motion_capture_update_rate",&mdp_api::timings::motion_capture_update_rate)
    .add_property("desired_drone_server_update_rate",&mdp_api::timings::desired_drone_server_update_rate)
    .add_property("achieved_drone_server_update_rate",&mdp_api::timings::achieved_drone_server_update_rate)
    .add_property("time_to_update_drones", &mdp_api::timings::time_to_update_drones)
    .add_property("wait_time_per_frame", &mdp_api::timings::wait_time_per_frame)
    ;

    py::class_<mdp_api::velocity_data>("velocity_data")
    .add_property("x",&mdp_api::velocity_data::x)
    .add_property("y",&mdp_api::velocity_data::y)
    .add_property("z",&mdp_api::velocity_data::z)
    .add_property("yaw",&mdp_api::velocity_data::yaw)
    ;

    
}