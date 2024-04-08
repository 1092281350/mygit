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
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <dji_sdk_demo/controller1.h>
#include <boost/format.hpp>
#include "std_msgs/Float32.h"
#include <quadrotor_msgs/PositionCommand.h>
#include<nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlBrakePub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos;
geometry_msgs::Vector3Stamped current_vel;
geometry_msgs::Quaternion current_imu;
trajectory_msgs::MultiDOFJointTrajectory circle_traj;
Mission square_mission;

double yaw,yaw_rate;
DLQR dlqr;

void lqrcontrol_callback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd)
{
  Desired_State_t des;
  double roll,pitch;
  des.p=Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des.v= Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des.a= Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                             cmd->acceleration.z);
  yaw              = cmd->yaw;
  yaw_rate          = cmd->yaw_dot;

  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  
  // Down sampled to 50Hz loop
  if(elapsed_time > ros::Duration(0.02))
  {   
    // calculateDistanceCallback();    

  dlqr.DLQR_Control(des,current_local_pos,current_vel,current_imu,roll,pitch);
    
    sensor_msgs::Joy controlPosYaw;
    controlPosYaw.axes.push_back(roll);
    controlPosYaw.axes.push_back( pitch);
    controlPosYaw.axes.push_back(2);
    controlPosYaw.axes.push_back(0);// current_angular.z   

    obtain_control();
    ctrlPosYawPub.publish(controlPosYaw);
   }

}

void local_velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
  current_vel.vector.x=msg->vector.x;
  current_vel.vector.y=msg->vector.y;
  current_vel.vector.z=msg->vector.z;
}
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  current_imu=msg->orientation;

}
void odom_callback(const nav_msgs::Odometry::ConstPtr& odom){
  
}
void lqrcontrol_circle(const trajectory_msgs::MultiDOFJointTrajectory& cmd){
    Desired_State_t des;
  double roll,pitch;
  des.p=Eigen::Vector3d(cmd.points[0].transforms[0].translation.x, cmd.points[0].transforms[0].translation.y ,cmd.points[0].transforms[0].translation.z);
  des.v= Eigen::Vector3d(cmd.points[0].velocities[0].linear.x,cmd.points[0].velocities[0].linear.y, cmd.points[0].velocities[0].linear.z);
  des.a= Eigen::Vector3d(cmd.points[0].accelerations[0].linear.x,cmd.points[0].accelerations[0].linear.y,
                             cmd.points[0].accelerations[0].linear.z);
  yaw              = 0;
  yaw_rate          = 0;

  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  
    // calculateDistanceCallback();    

  dlqr.DLQR_Control(des,current_local_pos,current_vel,current_imu,roll,pitch);
    
    sensor_msgs::Joy controlPosYaw;
    controlPosYaw.axes.push_back(roll);
    controlPosYaw.axes.push_back( pitch);
    controlPosYaw.axes.push_back(2);
    controlPosYaw.axes.push_back(0);// current_angular.z   

    obtain_control();
    ctrlPosYawPub.publish(controlPosYaw);

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_flight_lqr_node");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
 ros::Subscriber localv = nh.subscribe("dji_sdk/velocity", 10, &local_velocity_callback);
  ros::Subscriber localimu = nh.subscribe("dji_sdk/imu", 10, &imu_callback);
   ros::Subscriber odomsub = nh.subscribe("robot_fuse/odom", 10, &odom_callback);
  // Publish the control signal
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 10);
    // ros::Subscriber pub_cmd = nh.subscribe("/planning/pos_cmd", 100, &lqrcontrol_callback);//
     ros::Subscriber pub_cmd = nh.subscribe("/firefly/command/trajectory", 100, &lqrcontrol_circle);//
  // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here, but  
  // we use dji_sdk/flight_control_setpoint_generic to demonstrate how to set the flag
  // properly in function Mission::step()
  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;
  if (!set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }

  if(is_M100())
  {
    ROS_INFO("M100 taking off!");
    takeoff_result = M100monitoredTakeoff();
  }
  else
  {
    ROS_INFO("A3/N3 taking off!");
    takeoff_result = monitoredTakeoff();
  }

//lqr参数初始化

  if(takeoff_result)
  {
    square_mission.reset();
    square_mission.start_gps_location = current_gps;
    square_mission.start_local_position = current_local_pos;

  }

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

//FLU--->ENU
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_local_pos = msg->point;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  current_gps = *msg;

  // Down sampled to 50Hz loop
  if(elapsed_time > ros::Duration(0.02))
  {
    start_time = ros::Time::now();
    
  }
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
  return localPosReferenceSetter.response.result;
}
