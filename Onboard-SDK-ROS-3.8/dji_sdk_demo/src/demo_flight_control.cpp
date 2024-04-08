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
#include "dji_sdk_demo/pid.h"
#include "dji_sdk_demo/type_defines.h"
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
#include "dji_sdk_demo/PIDController.h"


const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;
    double roll,pitch;
float current_yawRate,dis;
double MaxVel=1.0;
int c=-1;
int count_step=0;
int a=0;
float altitude=1.2;
double current_angular_vel_z;
ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

 ros::Publisher set_complete_status;
 ros::Publisher ctrlVelPub;
 ros::Publisher ctrlBrakePub;
 ros::Publisher next_pharse;
ros::Publisher ctrlPosYawPub;
ros::Publisher curPub;
ros::Publisher auxPub;
// ros::Publisher cmdStatusPub;
ros::Subscriber mystart_sub;
// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos;
geometry_msgs::Quaternion current_imu;
geometry_msgs::Vector3Stamped current_vel;
geometry_msgs::Vector3 current_angular;
geometry_msgs::Point lastpoint;

// Global random values
uint8_t controlStatus = STOP_CONTROLLER;
float loopFrequency=50;
bool start=false;
int withinboard=0;
  std_msgs::UInt8 a1;

Mission square_mission;

typedef struct ServiceAck
{
  bool         result;
  int          cmd_set;
  int          cmd_id;
  unsigned int ack_data;
  ServiceAck(bool res, int set, int id, unsigned int ack)
    : result(res)
    , cmd_set(set)
    , cmd_id(id)
    , ack_data(ack)
  {
  }
  ServiceAck()
  {
  }
} ServiceAck;
bool mystart=false;
void mystart_func(const std_msgs::UInt8::ConstPtr &msg){
mystart=true;
}
// void calculateDistanceCallback( ){//const ros::TimerEvent&
//   Eigen::Vector3d l(lastpoint.x,lastpoint.y,0);
//     Eigen::Vector3d cur(current_local_pos.x,current_local_pos.y,0);
//     dis= (l-cur).norm();
//     if(dis<1.0){
//       controlStatus=STOP_CONTROLLER;
//     }
//     // ROS_INFO("dis is %.4f",dis);
// }
void ct()
{
std_msgs::UInt8 ms;
std_msgs::UInt8 a1;

  if(a==2){
    // square_mission.state = 4;
    // controlStatus=STOP_CONTROLLER;
  //     ms.data=0;
  // cmdStatusPub.publish(ms);
    a=0;
    a1.data=0;
    auxPub.publish(a1);
    ROS_INFO("aux pub ,data is %d",a1.data);
  }else if((count_step==1 || count_step==2)&& a==1){
    // square_mission.state = 3;
    // controlStatus=RUNNING;
  //     ms.data=2;
  // cmdStatusPub.publish(ms);
    a=2;
    a1.data= ((count_step==1) ? 12 : 22);
    auxPub.publish(a1);
    ROS_INFO("aux pub ,data is %d",a1.data);
  }
  else{
      // square_mission.state = 4;
    // controlStatus=STOP_CONTROLLER;
  //         ms.data=0;
  // cmdStatusPub.publish(ms);
  a1.data=0;
    auxPub.publish(a1);
    ROS_INFO("aux pub ,data is %d",a1.data);
  }
}
void change_state( const std_msgs::UInt8::ConstPtr &msg ){

  if(a==2){
    square_mission.state = 4;
    a=0;
    //     a1.data=0;
    // auxPub.publish(a1);
  }else if((count_step==1 || count_step==2)&& a==1){
    square_mission.state = 3;
    a=2;
    // a1.data= ((count_step==1) ? 12 : 22);
    // auxPub.publish(a1);
    ROS_INFO("State change successfully 3");
  }
  else{
      square_mission.state = 4;
    //     a1.data=0;
    // auxPub.publish(a1);
    ROS_INFO("State change successfully 4");
  }
  square_mission.finished=true;
  ROS_INFO("Recevice  successfully %d",msg->data);
}
void getlast_point(const geometry_msgs::Point::ConstPtr &last_point){

    lastpoint.x=last_point->x;
    lastpoint.y=last_point->y;
    lastpoint.z=last_point->z;
    ROS_INFO("update last point  x=%.4f,y=%.5f",lastpoint.x,lastpoint.y);
    // std_msgs::UInt8 c;
    // c.data=c;
    // auxcPub.publish(ms);
}

void local_velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
  current_vel.vector.x=msg->vector.x;
  current_vel.vector.y=msg->vector.y;
  current_vel.vector.z=msg->vector.z;

}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  current_imu=msg->orientation;
  current_angular_vel_z=msg->angular_velocity.z;
  
}

ServiceAck
releaseCtrlAuthority() {
  dji_sdk::SDKControlAuthority sdkAuthority;
  sdkAuthority.request.control_enable = 0;
  sdk_ctrl_authority_service.call(sdkAuthority);
  if(!sdkAuthority.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set, sdkAuthority.response.cmd_id);
    ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
  }
  return {sdkAuthority.response.result, sdkAuthority.response.cmd_set,
          sdkAuthority.response.cmd_id, sdkAuthority.response.ack_data};
}
void myCallback( const std_msgs::UInt8 value )
{
    ct();
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_flight_control_node");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);

  // 新增的
  ros::Subscriber localv = nh.subscribe("dji_sdk/velocity", 10, &local_velocity_callback);
  ros::Subscriber localimu = nh.subscribe("dji_sdk/imu", 10, &imu_callback);
  // ros::Subscriber pub_cmd = nh.subscribe("/planning/pos_cmd", 10, &control_callback,ros::TransportHints().tcpNoDelay());//
// ros::Subscriber controlStatusSub = nh.subscribe("/dtu_controller/control_status", 1, &myCallback);
  // cmdStatusPub = nh.advertise<std_msgs::UInt8> ("/dtu_controller/controlpid_status",1);
  ros::Subscriber stateSub  =  nh.subscribe("/dtu_controller/update", 1, &change_state);
  ros::Subscriber last_point_Sub = nh.subscribe("/dtu_controller/last_point", 1, &getlast_point);

     auxPub=nh.advertise<std_msgs::UInt8>("/dtu_controller/aux", 1);
  // next_pharse=nh.advertise<std_msgs::UInt8>("/dtu_controller/next_pharse",1);

  // Publish the control signal
  ctrlVelPub=nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 100);
   ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
     ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
  curPub=nh.advertise<geometry_msgs::PoseStamped>("/rviz/3d_nav_goal", 1);

  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  // ros::Timer loopTimer = nh.createTimer( ros::Duration(0.05),calculateDistanceCallback);
     
  bool obtain_control_result = obtain_control();
  bool takeoff_result;
  if (!set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }

  ROS_INFO("A3/N3 taking off!");
  mystart_sub=nh.subscribe("/mystart",1,mystart_func);
ros::AsyncSpinner spinner(0);
spinner.start();
while(!mystart){
  ROS_INFO("--------------waiting----------------------");
}
spinner.stop();

  takeoff_result = monitoredTakeoff();

  if(takeoff_result)
  {
    square_mission.reset();
    square_mission.start_gps_location = current_gps;
    square_mission.start_local_position = current_local_pos;
    square_mission.setTarget(3.1147,-2.7280,altitude, 60);
    square_mission.state = 1;
    ROS_INFO("##### Start route %d ....", square_mission.state);
  }
  // loopTimer.start();
  ros::spin();
  return 0;
}

void Mission::step()
{
  static int info_counter = 0;
  geometry_msgs::Vector3     localOffset;

  float speedFactor         = 2;
  float yawThresholdInDeg   = 2;

  float xCmd, yCmd, zCmd;

  localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

  double xOffsetRemaining = target_offset_x - localOffset.x;
  double yOffsetRemaining = target_offset_y - localOffset.y;
  double zOffsetRemaining = target_offset_z - localOffset.z;

  double yawDesiredRad     = deg2rad * target_yaw;
  double yawThresholdInRad = deg2rad * yawThresholdInDeg;
  double yawInRad          = toEulerAngle(current_atti).z;

  info_counter++;
  if(info_counter > 25)
  {
    info_counter = 0;
    ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x,localOffset.y, localOffset.z,yawInRad);
    ROS_INFO("+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining,yOffsetRemaining, zOffsetRemaining,yawInRad - yawDesiredRad);
  }
  if (abs(xOffsetRemaining) >= speedFactor)
    xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    xCmd = xOffsetRemaining;

  if (abs(yOffsetRemaining) >= speedFactor)
    yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    yCmd = yOffsetRemaining;

  zCmd = start_local_position.z + target_offset_z;


  /*!
   * @brief: if we already started breaking, keep break for 50 sample (1sec)
   *         and call it done, else we send normal command
   */

  if (break_counter > 50)
  {
    ROS_INFO("##### Route %d finished....", state);
    finished = true;
    return;
  }
  else if(break_counter > 0)
  {
    sensor_msgs::Joy controlVelYawRate;
    uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_GROUND   |
                DJISDK::STABLE_ENABLE);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(flag);

    ctrlBrakePub.publish(controlVelYawRate);
    break_counter++;
    return;
  }
  else //break_counter = 0, not in break stage
  {
    sensor_msgs::Joy controlPosYaw;


    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back(yCmd);
    controlPosYaw.axes.push_back(zCmd);
    controlPosYaw.axes.push_back(yawDesiredRad);
    ctrlPosYawPub.publish(controlPosYaw);
  }

  if (std::abs(xOffsetRemaining) < 0.5 &&
      std::abs(yOffsetRemaining) < 0.5 &&
      std::abs(zOffsetRemaining) < 0.5 &&
      std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
  {
    //! 1. We are within bounds; start incrementing our in-bound counter
    inbound_counter ++;
  }
  else
  {
    if (inbound_counter != 0)
    {
      //! 2. Start incrementing an out-of-bounds counter
      outbound_counter ++;
    }
  }

  //! 3. Reset withinBoundsCounter if necessary
  if (outbound_counter > 10)
  {
    ROS_INFO("##### Route %d: out of bounds, reset....", state);
    inbound_counter  = 0;
    outbound_counter = 0;
  }

  if (inbound_counter > 50)
  {
    ROS_INFO("##### Route %d start break....", state);
    break_counter = 1;
  }
  start=false;
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
  // current_gps = *msg;
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  current_gps = *msg;
      ServiceAck service_ack;
  // Down sampled to 50Hz loop
  if(elapsed_time > ros::Duration(0.02))
  {
    start_time = ros::Time::now();
    switch(square_mission.state)
    {
      case 0:
      if(start){
      service_ack = releaseCtrlAuthority();
        if (service_ack.result) {
          ROS_INFO("Release SDK control Authority successfully");
          start=false;
        } else {
          ROS_WARN("Failed release SDK control Authority");
          start=true;
        }
      }
        break;
      case 1:
        if(!square_mission.finished)
        {
          square_mission.step();
        }
        else
        {
          obtain_control();
          ROS_INFO("Obtain SDK control Authority successfully");
          square_mission.reset();
          square_mission.start_gps_location = current_gps;
          square_mission.start_local_position = current_local_pos;
          if(count_step==0){
            square_mission.setTarget(1.7103,1.9594, 0, 0);
          }else if(count_step==1)
          {
            square_mission.setTarget(-1.5149,-1.7553, 0, 0);
          }
          else if(count_step==2)
          {
            square_mission.setTarget(1.8889,-1.6148, 0, 0);
          }else if(count_step==3)
          {
            square_mission.setTarget(-7.0252,6.0559, 0, 0);
          }else if(count_step==4)
          {
            square_mission.setTarget(-4.8658,-5.5447, 0, 0);
          }
          
          
          square_mission.state = 2;
          ROS_INFO("##### Start route %d ....", square_mission.state);
        }
        break;

      case 2:
        if(!square_mission.finished)
        {
          square_mission.step();
        }
        else
        {
          obtain_control();
                ROS_INFO("Obtain SDK control Authority successfully");
          square_mission.reset();
          square_mission.start_gps_location = current_gps;
          square_mission.start_local_position = current_local_pos;
           if(count_step==0){
             square_mission.setTarget(6.8969,-5.9907, 0, 0);
             square_mission.state = 3;
           }else if(count_step==1)
          {
            square_mission.setTarget(1.3963,-1.6052, 0, 0);
              square_mission.state = 3;
          }else if(count_step==2)
          {
            square_mission.setTarget(2.3898,2.8985, 0, 0);
              square_mission.state = 5;
          }else if(count_step==3)
          {
            square_mission.setTarget(2.3652,2.6730, 0, 0);
              square_mission.state = 5;
          }else if(count_step==4)
          {
               square_mission.state = 0;//结束
          }
          
          ROS_INFO("##### Start route %d ....", square_mission.state);
        }
        break;
      case 3:
        if(!square_mission.finished)
        {
          square_mission.step();
        }
        else
        {
          square_mission.reset();
          square_mission.start_gps_location = current_gps;
          square_mission.start_local_position = current_local_pos;
         
              //发送消息
             geometry_msgs::PoseStamped m;
             m.header.frame_id="a";
             m.header.seq=0;
             m.header.stamp=start_time;
             m.pose.orientation.w=0;
             m.pose.orientation.x=0;
             m.pose.orientation.y=0;
             m.pose.orientation.z=0;
             curPub.publish(m);
          square_mission.finished=true;
          // controlStatus=RUNNING;
          // std_msgs::UInt8 ms;
          // ms.data=2;
          // cmdStatusPub.publish(ms);
          square_mission.state = 0;
          start=true;
          if(a!=2){
          a=1;//已经发了一次
          }
           if((count_step==1 || count_step==2)&& a==1){
             a1.data= ((count_step==1) ? 11 : 21);
            auxPub.publish(a1);
            ROS_INFO("##### aux publish successfully###########");
           }

          ROS_INFO("##### Start route %d ....", square_mission.state);
        }
        break;
      case 4:
        if(!square_mission.finished)
        {
          square_mission.step();
        }
        else
        {
          obtain_control();
                ROS_INFO("Obtain SDK control Authority successfully");
          ROS_INFO("##### Mission %d Finished ....", square_mission.state);
          square_mission.state = 1;
          count_step++;
          //发送消息
        }
        break;
      case 5:
        if(!square_mission.finished)
        {
          square_mission.step();
        }
        else
        {
          obtain_control();
                ROS_INFO("Obtain SDK control Authority successfully");
          square_mission.reset();
          square_mission.start_gps_location = current_gps;
          square_mission.start_local_position = current_local_pos;
          if(count_step==2){
          square_mission.setTarget(-1.6955,1.5947, 0, 0);
          }else if(count_step==3)
          {
            square_mission.setTarget(-1.6841,1.4268, 0, 0);
              // square_mission.state = 3;
          }

          square_mission.state = 3;
          ROS_INFO("##### Start route %d ....", square_mission.state);
        }
        break;
    }
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
