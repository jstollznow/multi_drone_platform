#include "ros/ros.h"

#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

/*
**
** To get tf to work on rviz, run this in yet another terminal:
** rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0 0.0 map vflie 100
** 

  before next meeting (7):
  create a diagram of input and output data flows for ALL planned packages. i.e. a top down approach to system design


*/

#define KINEMATIC false
#define LOOP_RATE_HZ 100
#define FRAME_TIME (1.0 / (double)LOOP_RATE_HZ)

#define START_X_PARAM   "start_x"
#define START_Y_PARAM   "start_y"
#define START_Z_PARAM   "start_z"
#define START_YAW_PARAM "start_yaw"

#define FRAME_ID "vflie"
#define INPUT_TOPIC "cmd_vel"

#define GRAVITY_BIAS 30000.0f

/* Declarations (these can be put here because virtualflie.cpp should never be included in another file) */
class virtual_drone
{
private:
  geometry_msgs::Pose Pose;
  geometry_msgs::Vector3 Velocity;

  float YawRate;
  float Pitch;
  float Roll;
  float Thrust;

  float DesiredYawRate;
  float DesiredPitch;
  float DesiredRoll;
  float DesiredThrust;

  ros::NodeHandle node;
  ros::Publisher  pub;
  ros::Subscriber sub;
  mutable unsigned int SeqCount;

public:
  virtual_drone();
  ~virtual_drone();
  void inputCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void updateDrone();
  void publish() const;
  void logPose() const;

};

struct euler_rotation
{
  float Pitch;
  float Yaw;
  float Roll;
};

geometry_msgs::Quaternion toQuaternion(euler_rotation pEulerRotation);
euler_rotation toEuler(geometry_msgs::Quaternion pQuaternion);
geometry_msgs::Vector3 getUpVector(geometry_msgs::Quaternion pQuaternion);
inline float degreesToRadians(float pDegrees);
inline float lerpFloat(float begin, float end, float t);


/* Implementations */

virtual_drone::virtual_drone()
: SeqCount(0)
{
  node = ros::NodeHandle("~");
  pub  = node.advertise<geometry_msgs::PoseStamped> (FRAME_ID, 1);
  sub  = node.subscribe(INPUT_TOPIC, 1, &virtual_drone::inputCallback, this);

  if (!node.hasParam(START_X_PARAM)) {
    node.getParam(START_X_PARAM, this->Pose.position.x);
  } else {
    this->Pose.position.x = 0.0;
  }

  if (!node.hasParam(START_Y_PARAM)) {
    node.getParam(START_Y_PARAM, this->Pose.position.y);
  } else {
    this->Pose.position.y = 0.0;
  }

  if (!node.hasParam(START_Z_PARAM)) {
    node.getParam(START_Z_PARAM, this->Pose.position.z);
  } else {
    this->Pose.position.z = 0.0;
  }

  if (!node.hasParam(START_YAW_PARAM)) {
    float yaw_input = 0.0f;
    node.getParam(START_YAW_PARAM, yaw_input);
    euler_rotation y_rotate = {0.0f, yaw_input, 0.0f};
    this->Pose.orientation = toQuaternion(y_rotate);
    ROS_INFO("YAW %f", yaw_input);
  } else {
    euler_rotation zero_rotate = {0.0f, 0.0f, 0.0f};
    this->Pose.orientation = toQuaternion(zero_rotate);
  }

  euler_rotation e_rot = toEuler(this->Pose.orientation);
  ROS_INFO("drone starting position [%f, %f, %f]", this->Pose.position.x, this->Pose.position.y, this->Pose.position.z);
  ROS_INFO("drone starting rotation (%f, %f, %f)", e_rot.Pitch, e_rot.Yaw, e_rot.Roll);
}

virtual_drone::~virtual_drone()
{
}

void virtual_drone::inputCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  this->DesiredThrust  = msg->linear.z;
  this->DesiredRoll    = msg->linear.x;
  this->DesiredPitch   = msg->linear.y;
  this->DesiredYawRate = msg->angular.z;

  ROS_INFO("<- sub {thrust: %.1f, (%.1f, %.1fdeg/sec, %.1f)}",
    this->DesiredThrust,
    this->DesiredRoll, this->DesiredYawRate, this->DesiredPitch
  );
}

void virtual_drone::updateDrone()
{
  if (!KINEMATIC) {
    /* lerp values semi-naturally */
    this->Thrust = lerpFloat(this->Thrust, this->DesiredThrust, 0.75f);
    this->YawRate = lerpFloat(this->YawRate, this->DesiredYawRate, 0.75f);
    this->Roll = lerpFloat(this->Roll, this->DesiredRoll, 0.5f);
    this->Pitch = lerpFloat(this->Pitch, this->DesiredPitch, 0.5f);

    /* set orientation of drone */
    euler_rotation rot = toEuler(this->Pose.orientation);
    rot.Yaw = rot.Yaw + degreesToRadians(this->YawRate) * FRAME_TIME;
    rot.Pitch = degreesToRadians(this->Pitch);
    rot.Roll = degreesToRadians(this->Roll);
    this->Pose.orientation = toQuaternion(rot);

    /* set position of drone */
    /* multiply drone up vecto by drone thrust value */
    geometry_msgs::Vector3 accel = getUpVector(this->Pose.orientation);
    float thrust_this_frame = (this->Thrust / 5000.0f) * FRAME_TIME;
    accel.x *= thrust_this_frame;
    accel.y *= thrust_this_frame;
    accel.z *= thrust_this_frame;

    /* apply gravity vector to velocity Vector3 */
    accel.z -= (GRAVITY_BIAS / 5000.0f) * FRAME_TIME;

    /* apply acceleration values to drone velocity */
    this->Velocity.x += accel.x;
    this->Velocity.y += accel.y;
    this->Velocity.z += accel.z;

    /* apply velocity to drone position */
    this->Pose.position.x += this->Velocity.x;
    this->Pose.position.y += this->Velocity.y;
    this->Pose.position.z += this->Velocity.z;

    /* dont allow the drone to fall under the 'ground' (at z = 0) */
    if (this->Pose.position.z < 0.0f) {
      this->Pose.position.z = 0.0f;
      this->Velocity.x = 0.0f;
      this->Velocity.y = 0.0f;
      this->Velocity.z = 0.0f;
    }
  } else {
    /* custom drone positions and orientations can be placed in here with KINEMATIC set to true */
    euler_rotation rotation = {
      sin(ros::Time::now().toSec()) * 0.5f,
      toEuler(this->Pose.orientation).Yaw + 0.002f,
      sin(ros::Time::now().toSec()/2.0) * 0.5f
    };
    this->Pose.orientation = toQuaternion(rotation);

    geometry_msgs::Vector3 v3 = getUpVector(this->Pose.orientation);
    this->Pose.position.x += v3.x * FRAME_TIME * 0.1f;
    this->Pose.position.y += v3.y * FRAME_TIME * 0.1f;
    this->Pose.position.z += v3.z * FRAME_TIME * 0.1f;
    //this->Pose.position.z = sin(ros::Time::now().toSec())*0.25f + 0.25f;
  }
}

void virtual_drone::publish() const
{
  geometry_msgs::PoseStamped msg;
  msg.pose = this->Pose;
  msg.header.seq = this->SeqCount++;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = FRAME_ID;
  pub.publish(msg);
}

void virtual_drone::logPose() const
{
  euler_rotation e_rot = toEuler(this->Pose.orientation);
  ROS_INFO("%s | pos:[%05.1f %05.1f %05.1f] rot:(%05.1f %05.1f %05.1f)",
    FRAME_ID,
    this->Pose.position.x, this->Pose.position.y, this->Pose.position.z,
    e_rot.Pitch, e_rot.Yaw, e_rot.Roll
  );
}

geometry_msgs::Quaternion toQuaternion(euler_rotation pEulerRotation)
{
  double cy = cos(pEulerRotation.Yaw * 0.5);
  double sy = sin(pEulerRotation.Yaw * 0.5);
  double cp = cos(pEulerRotation.Pitch * 0.5);
  double sp = sin(pEulerRotation.Pitch * 0.5);
  double cr = cos(pEulerRotation.Roll * 0.5);
  double sr = sin(pEulerRotation.Roll * 0.5);

  geometry_msgs::Quaternion q;
  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = sy * cp * sr + cy * sp * cr;
  q.z = sy * cp * cr - cy * sp * sr;

  return q;
}

euler_rotation toEuler(geometry_msgs::Quaternion pQuaternion)
{
  euler_rotation angles;

  // roll
  double sinr_cosp = +2.0 * (pQuaternion.w * pQuaternion.x + pQuaternion.y * pQuaternion.z);
  double cosr_cosp = +1.0 - 2.0 * (pQuaternion.x * pQuaternion.x + pQuaternion.y * pQuaternion.y);
  angles.Roll = atan2(sinr_cosp, cosr_cosp);

  // pitch
  double sinp = +2.0 * (pQuaternion.w * pQuaternion.y - pQuaternion.z * pQuaternion.x);
  if (fabs(sinp) >= 1)
    angles.Pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    angles.Pitch = asin(sinp);

  // yaw
  double siny_cosp = +2.0 * (pQuaternion.w * pQuaternion.z + pQuaternion.x * pQuaternion.y);
  double cosy_cosp = +1.0 - 2.0 * (pQuaternion.y * pQuaternion.y + pQuaternion.z * pQuaternion.z);  
  angles.Yaw = atan2(siny_cosp, cosy_cosp);

  return angles;
}

geometry_msgs::Vector3 getUpVector(geometry_msgs::Quaternion pQuaternion)
{
  geometry_msgs::Vector3 v;
  // RIGHT (y)
  //v.x = 2 * (pQuaternion.x*pQuaternion.y - pQuaternion.w*pQuaternion.z);
  //v.y = 1 - 2 * (pQuaternion.x*pQuaternion.x + pQuaternion.z*pQuaternion.z);
  //v.z = 2 * (pQuaternion.y*pQuaternion.z + pQuaternion.w*pQuaternion.x);

  // FORWARD (x)
  //v.x = 1 - 2 * (pQuaternion.y*pQuaternion.y + pQuaternion.z*pQuaternion.z);
  //v.y = 2 * (pQuaternion.x*pQuaternion.y + pQuaternion.w*pQuaternion.z);
  //v.z = 2 * (pQuaternion.x*pQuaternion.z - pQuaternion.w*pQuaternion.y);

  // UP (z)
  v.x = 2 * (pQuaternion.x*pQuaternion.z + pQuaternion.w*pQuaternion.y);
  v.y = 2 * (pQuaternion.y*pQuaternion.z - pQuaternion.w*pQuaternion.x);
  v.z = 1 - 2 * (pQuaternion.x*pQuaternion.x + pQuaternion.y*pQuaternion.y);
  return v;
}

float lerpFloat(float begin, float end, float t)
{
  return (begin + ((end - begin) * t));
}

float degreesToRadians(float pDegrees)
{
  return pDegrees * 0.01745329f;
}


/* ENTRY POINT */

int main(int argc, char **argv)
{
  ros::init(argc, argv, FRAME_ID);

  virtual_drone Drone;

  ros::Rate loop_rate(LOOP_RATE_HZ);

  int frames_per_100ms = (LOOP_RATE_HZ/10);
  unsigned int count = 0;
  while (ros::ok())
  {
    /* update drone orientations and positions */
    Drone.updateDrone();

    /* construct stamped pose and publish to virtualflie topic */
    Drone.publish();

    if ((count % frames_per_100ms) == 0) {
      Drone.logPose();
    }
    
    /* spin ROS */
    ros::spinOnce();
    loop_rate.sleep();
    count += 1;
  }

  printf("\nShutting down virtualflie drone\n");
  return 0;
}