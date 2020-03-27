#include "user_api.h"
#include "ros/ros.h"

#define DO_FLIGHT_TEST      0
#define DO_HOVER_TEST       0
#define DO_BASEBALL_RUN     0
#define DO_FIGURE_EIGHT     0   // velocity control, may be a bit risky
#define DO_DONUTS           1


void do_drone_flight_test(mdp::id drone)
{
    auto drones = mdp::get_all_rigidbodies();
    if (drones.size() < 1) return;
    mdp::cmd_takeoff(drones[0], 1.0, 5.0); // takeoff to a height of 0.5 over 2.0 seconds
    // mdp::cmd_takeoff(drones[1], 0.5, 2.0); // takeoff to a height of 0.5 over 2.0 seconds

    mdp::sleep_until_idle(drones[0]);
    // mdp::sleep_until_idle(drones[1]);
    // mdp::sleep_until_idle(drone);        // sleep api program until drone is idle (takeoff command has finished)

    mdp::position_msg msg;         // construct a position msg
    msg.relative = true;
    msg.keepHeight = true;
    msg.position = {2.0, 0.0, 0.0};
    msg.duration = 2.0;
    msg.yaw = 0.0;

    for (size_t i = 0; i < 100; i++) {
        mdp::set_drone_position(drones[0], msg);    // tell drone to go to position outlined in msg
        mdp::spin_until_rate();
    }
    // mdp::set_drone_position(drones[1], msg);

    // //mdp::sleep_until_idle(drone);    // sleep api program until drone is idle

    // //mdp::goto_home(drone);     // tell drone to go home and land (as height is set to 0.0)

    // //mdp::sleep_until_idle(drone);

    mdp::sleep_until_idle(drones[0]);
    // mdp::sleep_until_idle(drones[1]);

    mdp::go_to_home(drones[0]);
    // mdp::goto_home(drones[1]);

    mdp::sleep_until_idle(drones[0]);
    // mdp::sleep_until_idle(drones[1]);

    mdp::cmd_land(drones[0]);
    // mdp::cmd_land(drones[1]);

    mdp::sleep_until_idle(drones[0]);
    // mdp::sleep_until_idle(drones[1]);
}

void hover_test(mdp::id drone)
{
    mdp::cmd_takeoff(drone, 1.0);

    ros::Duration d(1.0);

    mdp::sleep_until_idle(drone);

    mdp::position_msg msg;
    msg.relative = true;
    msg.keepHeight = true;
    msg.position = {2.5, 0.0, 0.0};
    msg.yaw = 0.0;
    msg.duration = 3.0;

    mdp::set_drone_position(drone, msg);

    //msg.position = {0.0, 0.5, 0.0};
    //d.sleep();

    //mdp::set_drone_position(drone, msg);

    mdp::sleep_until_idle(drone);

    mdp::go_to_home(drone, 4.0, 0.0);

    mdp::sleep_until_idle(drone);
}

void do_baseball_base_run(std::vector<std::array<double, 3>> positions)
{
    auto drones = mdp::get_all_rigidbodies();
    /* there are at least 2 drones, do a baseball base running thing */
    /* i.e drone 1 gets positions to fly to, drone 2 is 1 position behind drone 1 at all times */

    mdp::cmd_takeoff(drones[0], 1.0);
    mdp::cmd_takeoff(drones[1], 0.5);

    mdp::sleep_until_idle(drones[0]);
    mdp::sleep_until_idle(drones[1]);

    mdp::position_msg msg;
    msg.relative = false;
    msg.keepHeight = true;
    msg.position = positions[0];
    msg.duration = 2.0;
    msg.yaw = 0.0;

    mdp::set_drone_position(drones[0], msg);    // tell drone 0 to go to first position

    mdp::sleep_until_idle(drones[0]);

    for (size_t i = 1; i < positions.size(); i++) {     // for all positions in positions array
        mdp::set_drone_position(drones[1], msg);    // tell drone 1 to go to position i-1
        msg.position = positions[i];
        mdp::set_drone_position(drones[0], msg);    // tell drone 0 to go to position i
        mdp::sleep_until_idle(drones[0]);
        mdp::sleep_until_idle(drones[1]);
    }

    mdp::set_drone_position(drones[1], msg);    // send drone 1 to final position and drone 0 to home
    mdp::go_to_home(drones[0], 2.0, 1.0);                  // no height set so goto home position at current height
    mdp::sleep_until_idle(drones[0]);
    mdp::sleep_until_idle(drones[1]);

    mdp::go_to_home(drones[1], 2.0, 1.0);                  // send drone 1 to home
    mdp::sleep_until_idle(drones[1]);

    mdp::cmd_land(drones[0]);                   // land both drones
    mdp::cmd_land(drones[1]);

    mdp::sleep_until_idle(drones[0]);
    mdp::sleep_until_idle(drones[1]);
}

void do_figure_eight_with_follower()
{
    auto drones = mdp::get_all_rigidbodies();
    if (drones.size() < 2) return;

    // might be able to add to position msg a 'target' drone
    // if a 'target' drone is set, position becomes an offset from the target's position
    // relative can then transform the offset from world space to drone space of subject drone
    // i.e. drones[1]: target drones[0], offset to {0.3, 0.0, 0.0}, relative to true
    // means send drones[1] to drones[0]'s position but 0.3m closer to drones[1] current position

    mdp::cmd_takeoff(drones[0], 0.5);
    mdp::cmd_takeoff(drones[1], 0.4);

    mdp::position_msg msg;
    msg.relative = false;
    msg.position = {0.0, 0.0, 0.5};
    msg.yaw = 0.0;
    msg.duration = 1.0;

    // set drone 0 to origin
    mdp::set_drone_position(drones[0], msg);

    // set drone 1 to just under drone 0
    msg.position = {-0.3, 0.0, 0.25};
    mdp::set_drone_position(drones[1], msg);

    // set position message to target drones[0] at offset of 10cm behind relative to drones[1]
    msg.relative = false;
    msg.position = {0.1, 0.0, 0.0};

    // create velocity message for drones[0]
    mdp::velocity_msg vel_msg;
    vel_msg.relative = true;
    vel_msg.velocity = {1.0, 0.0, 0.0};
    vel_msg.duration = 0.2;
    vel_msg.yawRate = 180.0;   // 360 degrees in 2 seconds

    // loop for 10 seconds
    int frames_every_two_seconds = (10 * 2);
    float seconds_to_run_for = 10.0f;
    int frames = seconds_to_run_for * 10;

    mdp::spin_until_rate();   // reset loop rate before beginning loop dependant code

    for (int i = 0; i < frames; i++) {
        // loop for 10 seconds
        if ((i % frames_every_two_seconds) == 0) {
            // every 2 seconds reverse yawrate of drones[0], i.e. about when the drone does a full circle
            vel_msg.yawRate = -vel_msg.yawRate;
        }
        mdp::set_drone_velocity(drones[0], vel_msg);
        mdp::set_drone_position(drones[1], msg);

        mdp::spin_until_rate();
    }

    // set drones[1] height to just under drones[0] (so they dont crash into each other going home
    msg.relative = true;
    msg.keepHeight = false;
    msg.position = {0.0, 0.0, 0.25};
    mdp::set_drone_position(drones[1], msg);
    mdp::sleep_until_idle(drones[1]);

    // send both drones home to land
    mdp::go_to_home(drones[0], 2.0, 0.0);
    mdp::go_to_home(drones[1], 2.0, 0.0);

    mdp::sleep_until_idle(drones[0]);
    mdp::sleep_until_idle(drones[1]);
}

void do_donuts(mdp::id baseDrone, mdp::id donutDrone) {
    mdp::cmd_takeoff(baseDrone);
    mdp::cmd_takeoff(donutDrone, 1.0);

    mdp::sleep_until_idle(baseDrone);
    mdp::sleep_until_idle(donutDrone);

    std::array<std::array<double, 3>, 5> positions = {
            {{-0.5, 0, 0.0},
            {0.0, -0.5, 0.0},
            {1.0, 0.0, 0.0},
            {0.0, 0.5, 0.0},
            {-0.5, 0.0, 0.0}}
    };

    mdp::position_msg baseMsg;
    baseMsg.relative = false;
    baseMsg.keepHeight = true;
    baseMsg.position = positions[0];
    baseMsg.duration = 6.0;
    baseMsg.yaw = 0.0;

    auto baseDronePos = mdp::get_position(baseDrone);

    mdp::position_msg donutMsg;
    donutMsg.relative = false;
    donutMsg.keepHeight = true;
    donutMsg.position = {baseDronePos.x, baseDronePos.y, 0.0};
    donutMsg.duration = 3.0;
    donutMsg.yaw = 0.0;

    mdp::set_drone_position(donutDrone, donutMsg);
    mdp::sleep_until_idle(donutDrone);
    donutMsg.duration = 0.5;

    mdp::spin_until_rate();
    for (size_t i = 0; i < positions.size(); i++) {
        baseMsg.position = positions[i];
        mdp::set_drone_position(baseDrone, baseMsg);
        mdp::spin_until_rate();

        while (mdp::get_state(baseDrone) != mdp::drone_state::HOVERING) {
            baseDronePos = mdp::get_position(baseDrone);

            double timeNow = ros::Time::now().toSec();
            double donutX = sin(timeNow) * 0.6;
            double donutY = cos(timeNow) * 0.6;

            donutMsg.position = {baseDronePos.x + donutX, baseDronePos.y + donutY, 0.0};
            mdp::set_drone_position(donutDrone, donutMsg);

            mdp::spin_until_rate();
        }
    }

    mdp::go_to_home(baseDrone);
    mdp::go_to_home(donutDrone);

    mdp::sleep_until_idle(baseDrone);
    mdp::sleep_until_idle(donutDrone);
}

int main(int argc, char** argv)
{
    mdp::initialise(10, "test_program"); // update rate of 10Hz
    auto drones = mdp::get_all_rigidbodies();

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

#if (DO_DONUTS)
        if (drones.size() >= 2) {
            do_donuts(drones[0], drones[1]);
        }
#endif

    mdp::terminate();

    return 0;
}