//
// Created by jacob on 18/5/20.
//

#ifndef MULTI_DRONE_PLATFORM_POTENTIAL_FIELDS_H
#define MULTI_DRONE_PLATFORM_POTENTIAL_FIELDS_H

#include "rigidbody.h"

/**
 * Indicates when the attractive velocity should reduce to ensure the goalpoint is not overshot
 */
#define ATTRACTIVE_DIST 0.10f

/**
 * The gain associated relative position between the obstacle and the drone in the repulsive velocity equation
 */
#define K_P 6.0f

/**
 * Gain associated with the relative velocity between the obstacle and the drone in the repulsive velocity equation
 */
#define K_D 0.8f

class potential_fields {
private:
    /**
     * Used to generate the velocity related to repulsive forces for a given drone obstacle.
     * @param d The given drone for which the obstacle positions are relative to.
     * @param rigidbodies The list of all rigidbodies (treated as obstacles).
     * @return The repulsive velocity vector.
     */
    static geometry_msgs::Vector3 replusive_forces(rigidbody* d, std::vector<rigidbody*>& rigidbodies);

    /**
     * Used to generate the velocity related to attractive forces for a given drone obstacle.
     * @param d The given drone for which the goal point is relative to.
     * @param remainingDuration This is used as a factor in the calculation as determines how quickly the given drone
     * should attempt to reach its destination.
     * @return The attractive velocity vector.
     */
    static geometry_msgs::Vector3 attractive_forces(rigidbody* d, double remainingDuration);

    /**
     * Used to calculate the velocity required to reach the specified destination in time.
     * @param d The subject drone.
     * @param remainingDuration The remaining duration on the set_position command.
     * @return Returns a velocity vector reflecting the required velocity of the drone.
     */
    static geometry_msgs::Vector3 calculate_req_velocity(rigidbody* d, double remainingDuration);

    /**
     * The main function which adds the repulsive and attractive velocity vectors to determine the next state for the
     * given drone.
     * @param d The subject drone.
     * @param rigidbodies All rigidbody objects tracked by the drone server.
     */
    static void position_based_pf(rigidbody* d, std::vector<rigidbody *> &rigidbodies);
public:
    /**
     * Determines whether to apply potential fields based on the command type, currently only applies to position-based
     * commands.
     * @param d The subject drone.
     * @param rigidbodies All rigidbody objects tracked and known by the drone server.
     * @return
     */
    static bool check(rigidbody* d, std::vector<rigidbody*>& rigidbodies);

    /**
    * Variables used to track relative distance related statistics
    */
    static double closest;
    static double closestThisRound;
    static double lastClosestRound;


};

#endif //MULTI_DRONE_PLATFORM_POTENTIAL_FIELDS_H
