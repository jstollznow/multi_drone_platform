#include "../include/user_api.h"
#include "ros/ros.h"

#define DO_FLIGHT_TEST      0
#define DO_HOVER_TEST       1
#define DO_BASEBALL_RUN     0
#define DO_FIGURE_EIGHT     0   // velocity control, may be a bit risky


void do_drone_flight_test(mdp_api::id drone)
{
    auto drones = mdp_api::get_all_rigidbodies();
    if (drones.size() < 1) return;
    mdp_api::cmd_takeoff(drones[0], 0.5, 2.0); // takeoff to a height of 0.5 over 2.0 seconds
    // mdp_api::cmd_takeoff(drones[1], 0.5, 2.0); // takeoff to a height of 0.5 over 2.0 seconds

    mdp_api::sleep_until_idle(drones[0]);
    // mdp_api::sleep_until_idle(drones[1]);
    // mdp_api::sleep_until_idle(drone);        // sleep api program until drone is idle (takeoff command has finished)

    mdp_api::position_msg msg;         // construct a position msg
    msg.relative = true;
    msg.keep_height = true;
    msg.position = {2.0, 0.0, 0.0};
    msg.duration = 2.0;
    msg.yaw = 0.0;

    for (size_t i = 0; i < 100; i++) {
        mdp_api::set_drone_position(drones[0], msg);    // tell drone to go to position outlined in msg
        mdp_api::spin_once();
    }
    // mdp_api::set_drone_position(drones[1], msg);

    // //mdp_api::sleep_until_idle(drone);    // sleep api program until drone is idle

    // //mdp_api::goto_home(drone);     // tell drone to go home and land (as height is set to 0.0)

    // //mdp_api::sleep_until_idle(drone);

    mdp_api::sleep_until_idle(drones[0]);
    // mdp_api::sleep_until_idle(drones[1]);

    mdp_api::goto_home(drones[0]);
    // mdp_api::goto_home(drones[1]);

    mdp_api::sleep_until_idle(drones[0]);
    // mdp_api::sleep_until_idle(drones[1]);

    mdp_api::cmd_land(drones[0]);
    // mdp_api::cmd_land(drones[1]);

    mdp_api::sleep_until_idle(drones[0]);
    // mdp_api::sleep_until_idle(drones[1]);
}

void hover_test(mdp_api::id drone)
{
    mdp_api::cmd_takeoff(drone, 1.0);

    ros::Duration d(1.0);

    mdp_api::sleep_until_idle(drone);

    mdp_api::position_msg msg;
    msg.relative = true;
    msg.keep_height = true;
    msg.position = {2.5, 0.0, 0.0};
    msg.yaw = 0.0;
    msg.duration = 3.0;

    mdp_api::set_drone_position(drone, msg);

    //msg.position = {0.0, 0.5, 0.0};
    //d.sleep();

    //mdp_api::set_drone_position(drone, msg);

    mdp_api::sleep_until_idle(drone);

    mdp_api::goto_home(drone, 4.0, 0.0);

    mdp_api::sleep_until_idle(drone);
}

void do_baseball_base_run(std::vector<std::array<double, 3>> positions)
{
    auto drones = mdp_api::get_all_rigidbodies();
    /* there are at least 2 drones, do a baseball base running thing */
    /* i.e drone 1 gets positions to fly to, drone 2 is 1 position behind drone 1 at all times */

    mdp_api::cmd_takeoff(drones[0], 1.0);
    mdp_api::cmd_takeoff(drones[1], 0.5);

    mdp_api::sleep_until_idle(drones[0]);
    mdp_api::sleep_until_idle(drones[1]);

    mdp_api::position_msg msg;
    msg.relative = false;
    msg.keep_height = true;
    msg.position = positions[0];
    msg.duration = 2.0;
    msg.yaw = 0.0;

    mdp_api::set_drone_position(drones[0], msg);    // tell drone 0 to go to first position

    mdp_api::sleep_until_idle(drones[0]);

    for (size_t i = 1; i < positions.size(); i++) {     // for all positions in positions array
        mdp_api::set_drone_position(drones[1], msg);    // tell drone 1 to go to position i-1
        msg.position = positions[i];
        mdp_api::set_drone_position(drones[0], msg);    // tell drone 0 to go to position i
        mdp_api::sleep_until_idle(drones[0]);
        mdp_api::sleep_until_idle(drones[1]);
    }

    mdp_api::set_drone_position(drones[1], msg);    // send drone 1 to final position and drone 0 to home
    mdp_api::goto_home(drones[0], 2.0, 1.0);                  // no height set so goto home position at current height
    mdp_api::sleep_until_idle(drones[0]);
    mdp_api::sleep_until_idle(drones[1]);

    mdp_api::goto_home(drones[1], 2.0, 1.0);                  // send drone 1 to home
    mdp_api::sleep_until_idle(drones[1]);

    mdp_api::cmd_land(drones[0]);                   // land both drones
    mdp_api::cmd_land(drones[1]);

    mdp_api::sleep_until_idle(drones[0]);
    mdp_api::sleep_until_idle(drones[1]);
}

void do_figure_eight_with_follower()
{
    auto drones = mdp_api::get_all_rigidbodies();
    if (drones.size() < 2) return;

    // might be able to add to position msg a 'target' drone
    // if a 'target' drone is set, position becomes an offset from the target's position
    // relative can then transform the offset from world space to drone space of subject drone
    // i.e. drones[1]: target drones[0], offset to {0.3, 0.0, 0.0}, relative to true
    // means send drones[1] to drones[0]'s position but 0.3m closer to drones[1] current position

    mdp_api::cmd_takeoff(drones[0], 0.5);
    mdp_api::cmd_takeoff(drones[1], 0.4);

    mdp_api::position_msg msg;
    msg.relative = false;
    msg.position = {0.0, 0.0, 0.5};
    msg.yaw = 0.0;
    msg.duration = 1.0;

    // set drone 0 to origin
    mdp_api::set_drone_position(drones[0], msg);

    // set drone 1 to just under drone 0
    msg.position = {-0.3, 0.0, 0.25};
    mdp_api::set_drone_position(drones[1], msg);

    // set position message to target drones[0] at offset of 10cm behind relative to drones[1]
    msg.relative = false;
    msg.position = {0.1, 0.0, 0.0};

    // create velocity message for drones[0]
    mdp_api::velocity_msg vel_msg;
    vel_msg.relative = true;
    vel_msg.velocity = {1.0, 0.0, 0.0};
    vel_msg.duration = 0.2;
    vel_msg.yaw_rate = 180.0;   // 360 degrees in 2 seconds

    // loop for 10 seconds
    int frames_every_two_seconds = (mdp_api::rate() * 2);
    float seconds_to_run_for = 10.0f;
    int frames = seconds_to_run_for * mdp_api::rate();

    mdp_api::spin_once();   // reset loop rate before beginning loop dependant code

    for (int i = 0; i < frames; i++) {
        // loop for 10 seconds
        if ((i % frames_every_two_seconds) == 0) {
            // every 2 seconds reverse yawrate of drones[0], i.e. about when the drone does a full circle
            vel_msg.yaw_rate = -vel_msg.yaw_rate;
        }
        mdp_api::set_drone_velocity(drones[0], vel_msg);
        mdp_api::set_drone_position(drones[1], msg);

        mdp_api::spin_once();
    }

    // set drones[1] height to just under drones[0] (so they dont crash into each other going home
    msg.relative = true;
    msg.keep_height = false;
    msg.position = {0.0, 0.0, 0.25};
    mdp_api::set_drone_position(drones[1], msg);
    mdp_api::sleep_until_idle(drones[1]);

    // send both drones home to land
    mdp_api::goto_home(drones[0], 2.0, 0.0);
    mdp_api::goto_home(drones[1], 2.0, 0.0);

    mdp_api::sleep_until_idle(drones[0]);
    mdp_api::sleep_until_idle(drones[1]);
}

int main(int argc, char** argv)
{
    mdp_api::initialise(10); // update rate of 10Hz
    auto drones = mdp_api::get_all_rigidbodies();

    #if (DO_FLIGHT_TEST)
        if (drones.size() > 0) {
            do_drone_flight_test(drones[0]);
        }
    #endif /* DO_FLIGHT_TEST */

    #if (DO_HOVER_TEST)
        if (drones.size() > 0) {
            hover_test(drones[0]);
        }
    #endif /* DO_HOVER_TEST */

    #if (DO_BASEBALL_RUN)
        do_baseball_base_run({
            {-1.0, 0, 0.0},
            {0.0, -1.0, 0.0},
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {-1.0, 0.0, 0.0}
        });
    #endif /* DO_BASEBALL_RUN */

    #if (DO_FIGURE_EIGHT)
        do_figure_eight_with_follower();
    #endif /* DO_FIGURE_EIGHT */

    mdp_api::terminate();

    return 0;
}