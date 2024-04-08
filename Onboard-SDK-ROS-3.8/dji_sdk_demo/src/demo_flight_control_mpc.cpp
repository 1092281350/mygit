/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */
#include "dji_sdk_demo/demo_flight_control.h"
#include "dji_sdk/dji_sdk.h"
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include<geometry_msgs/Vector3Stamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include<std_msgs/UInt8.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include "dji_sdk_demo/pid.h"
#include "dji_sdk_demo/type_defines.h"

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;
    double roll,pitch;
float current_yawRate,dis;
double MaxVel=1.0;
int c=-1;
bool takeoff_result;
double current_angular_vel_z;
boost::array<double, 36UL> pose_covar;
boost::array<double, 36UL> twist_covar;
ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

 ros::Publisher set_complete_status;
 ros::Publisher ctrlVelPub;
 ros::Publisher ctrlBrakePub;
 ros::Publisher next_pharse;
ros::Publisher odom_pub;
ros::Publisher ref_pub;
// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
std_msgs::Header header;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos;
geometry_msgs::Quaternion current_imu;
geometry_msgs::Vector3Stamped current_vel;
geometry_msgs::Vector3 current_angular;
geometry_msgs::Point lastpoint;
nav_msgs::Odometry odom;
sensor_msgs::Imu::ConstPtr cov;
geometry_msgs::Vector3 rp;


//线性mpc
// ki太大收敛慢且画不圆
// kp太小，就成方形了

//1.0,0.5,0.05可以画圆形，但收敛还是不够快
//1.1, 0.4,   0.5，响应很快，但是画的不够圆
//1.1, 0.5,   0.5，画的比较圆，响应慢
// 0.8, -0.8, 1.0,0.5,0.05 接近完美，响应很快，就是收敛有点慢，不够圆
PID *rPid= new PID( 1/50.0,0.8, -0.8, 1.0,0.5,0.05);
PID *pPid= new PID( 1/50.0,0.8, -0.8, 1.0,0.5,0.05);
PID *yPid= new PID( 1/50.0, 1.0, -1.0, 1.0,0.5,0.05 );

// Global random values
int withinboard=0;

Mission square_mission;

void odompub(){

  pose_covar.assign(0);
    twist_covar.assign(0);
  odom.pose.pose.position=current_local_pos;
  odom.pose.pose.orientation=current_atti;
    odom.twist.twist.linear=current_vel.vector;
    odom.twist.twist.angular=current_angular;
    odom.header.frame_id="world";
    odom.header.seq=header.seq;
     odom.header.stamp=ros::Time::now();

      odom.pose.covariance=pose_covar;
      odom.twist.covariance=twist_covar;
    rp=toEulerAngle(current_atti);
      odom_pub.publish(odom);
}

void mpc_control(const mav_msgs::RollPitchYawrateThrustConstPtr& msg){

  sensor_msgs::Joy controlVelYawRate;

    controlVelYawRate.axes.push_back(msg->roll);
    controlVelYawRate.axes.push_back( msg->pitch);
    controlVelYawRate.axes.push_back(2);
    controlVelYawRate.axes.push_back(msg->yaw_rate);
    // controlVelYawRate.axes.push_back(flag);

    ctrlBrakePub.publish(controlVelYawRate);
}


void local_velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
  current_vel.vector.x=msg->vector.x;
  current_vel.vector.y=msg->vector.y;
  current_vel.vector.z=msg->vector.z;

}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){

  current_imu=msg->orientation;
  current_angular_vel_z=msg->angular_velocity.z;
    current_angular=msg->angular_velocity;
    cov=msg;
    // odompub();
      // if(takeoff_result)odompub();
}
void odom_callback(const nav_msgs::Odometry::ConstPtr& odo){
  odom=*odo;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_flight_control_mpc");
  ros::NodeHandle nh;
  

  //ros::NodeHandle n("~");

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);

  // 新增的
  ros::Subscriber localv = nh.subscribe("dji_sdk/velocity", 10, &local_velocity_callback);
  ros::Subscriber localimu = nh.subscribe("dji_sdk/imu", 10, &imu_callback);
     ros::Subscriber odomsub = nh.subscribe("robot_fuse/odom", 10, &odom_callback);
  // ros::Subscriber pub_cmd = nh.subscribe("/planning/pos_cmd", 10, &control_callback,ros::TransportHints().tcpNoDelay());//

  ros::Subscriber rpy_Sub = nh.subscribe("/firefly/command/roll_pitch_yawrate_thrust", 10, &mpc_control );
  // next_pharse=nh.advertise<std_msgs::UInt8>("/dtu_controller/next_pharse",1);

  // Publish the control signal
   ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 10);
  // odom_pub = nh.advertise<nav_msgs::Odometry>("/firefly/ground_truth/odometry", 10);
  // odom_pub = nh.advertise<nav_msgs::Odometry>("/firefly/msf_core/odometry", 10);
  ref_pub = nh.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 1);

  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  // ros::Timer loopTimer = nh.createTimer( ros::Duration(0.05),calculateDistanceCallback);

  bool obtain_control_result = obtain_control();

  if (!set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }
  ROS_INFO("A3/N3 taking off!");
  takeoff_result = monitoredTakeoff();

  if(takeoff_result)
  {
   square_mission.reset();
    ROS_INFO("A3/N3 taking off  successfully");
  }
  // loopTimer.start();
  ros::spin();
  return 0;
}


// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/
void
localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
}


geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}


bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& attitude_quaternion_msg)
{
  current_atti = attitude_quaternion_msg->quaternion;
header=attitude_quaternion_msg->header;

}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& position_msg)
{

   current_local_pos.x = position_msg->point.y;
  current_local_pos.y = -position_msg->point.x;
  current_local_pos.z = position_msg->point.z + 0.115;
  // ROS_INFO("CURRENT LOCAL X IS %.4f, LOCAL Y IS %.4f",current_local_pos.x,current_local_pos.y);

}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_gps = *msg;

}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}


/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
          (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}


/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
      current_gps.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }

  return true;
}

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  geometry_msgs::PoseStamped tmp;
  tmp.header.frame_id="local";
  tmp.header.seq=0;
  tmp.header.stamp=ros::Time::now();
  tmp.pose.position=current_local_pos;
  tmp.pose.orientation=current_atti;
  // ref_pub.publish(tmp);
  return localPosReferenceSetter.response.result;
}
