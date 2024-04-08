#include <iostream>
#include <cmath>
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
#include<std_msgs/Float32MultiArray.h>
#include "dji_sdk_demo/PIDController.h"
#include "dji_sdk_demo/ladrc.h"
#include "dji_sdk_demo/fuzzy_PID.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3

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

using namespace std;

//  begin
float loopFrequency=50;
bool start=false;
bool usepid=true;
int m=0,y=0;//记录a和count_step的值
int cl=0,withinbounder=0;
  Eigen::Vector3d des_pos_,des_vel_,des_acc_;
  double yaw,yaw_rate;
std_msgs::UInt8 my;
ros::Timer timer;
ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

geometry_msgs::Point current_local_pos;
geometry_msgs::Vector3Stamped current_vel;
geometry_msgs::Point lastpoint;
ros::Publisher ctrlVelPub;
ros::Publisher mysPub;
ros::Publisher ok_pub;
ros::Publisher desvel_pub;
ros::Publisher desacc_pub;
ros::Publisher ladrcx_pub,ladrcy_pub;
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
PID *xPid= new PID( 1/loopFrequency, 1.5,  -1.5, 1.0, 0.05,  0.5 );
PID *yPid= new PID( 1/loopFrequency, 1.5,  -1.5, 1.0, 0.05,  0.5 );

FuzzyPID fuzzypidx(10,1.8,0.3,0.4,0.2,1.0, 0.5, 0.05);
FuzzyPID fuzzypidy(10,1.8,0.3,0.4,0.2,1.0, 0.5, 0.05);

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

void ct(int a,int count_step)
{
    geometry_msgs::PoseStamped my;
    my.header.frame_id="a";
    my.header.seq=0;
    my.header.stamp=ros::Time::now();
    my.pose.orientation=geometry_msgs::Quaternion();
    my.pose.position=geometry_msgs::Point();
    cl++;
  ok_pub.publish(my);
}

void getlast_point(const geometry_msgs::Point::ConstPtr &last_point){

    lastpoint.x=last_point->x;
    lastpoint.y=last_point->y;
    lastpoint.z=last_point->z;
    ROS_INFO("update last point  x=%.4f,y=%.5f in PID 's getlast_point callback function ",lastpoint.x,lastpoint.y);

}
void calculate_dis()
{
  Eigen::Vector3d l(lastpoint.y,lastpoint.x,0);
    Eigen::Vector3d cur(current_local_pos.x,current_local_pos.y,0);
    std_msgs::UInt8 my;
    float dis;
    ROS_INFO("CURRENT LOCAL X IS %.4f, LOCAL Y IS %.4f",current_local_pos.x,current_local_pos.y);
    ROS_INFO("last  point X IS %.4f,  Y IS %.4f,cl is %d",lastpoint.x,lastpoint.y,cl);
    dis= (l-cur).norm();
    if(cl ==0 && dis<11.25){   
       if(abs(current_vel.vector.x)<=0.05 && abs(current_vel.vector.y)<=0.1 ){   
        withinbounder++;
      }else{
        withinbounder=0;
      }
      if(withinbounder >400){
        withinbounder=0;
        ct(m,y);
      }
    }else if(cl>0){
        if( abs(current_vel.vector.x)<=0.1 && abs(current_vel.vector.y)<=0.1){   
        withinbounder++;
      }else{
        withinbounder=0;
      }
      if(withinbounder >400){
        withinbounder=0;
        ct(m,y);
      }
    }

}

void local_velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
  current_vel.vector.x=msg->vector.x;
  current_vel.vector.y=msg->vector.y;
  current_vel.vector.z=msg->vector.z;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& position_msg)
{
   current_local_pos.x = position_msg->point.y;
  current_local_pos.y = -position_msg->point.x;
  current_local_pos.z = position_msg->point.z + 0.115;
  // ROS_INFO("CURRENT LOCAL X IS %.4f, LOCAL Y IS %.4f",current_local_pos.x,current_local_pos.y);
}

void circle_callback(const trajectory_msgs::MultiDOFJointTrajectory& cmd)
{
  des_pos_= Eigen::Vector3d(cmd.points[0].transforms[0].translation.x, cmd.points[0].transforms[0].translation.y ,cmd.points[0].transforms[0].translation.z);
  des_vel_= Eigen::Vector3d(cmd.points[0].velocities[0].linear.x,cmd.points[0].velocities[0].linear.y, cmd.points[0].velocities[0].linear.z);
  des_acc_= Eigen::Vector3d(cmd.points[0].accelerations[0].linear.x,cmd.points[0].accelerations[0].linear.y,
                             cmd.points[0].accelerations[0].linear.z);
  yaw              = 0;
  yaw_rate          = 0;

}
void control_callback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd)
{


  des_pos_= Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_= Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_= Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                             cmd->acceleration.z);
  yaw              = cmd->yaw;
  yaw_rate          = cmd->yaw_dot;

  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  
  // Down sampled to 50Hz loop
  if(elapsed_time > ros::Duration(0.02))
  {   
    // calculateDistanceCallback();    

   }

}

// //pid
// void callback(const ros::TimerEvent& event)
// {

//     //距离目标点小于0.2m,进行下一阶段
//     calculate_dis();
//     float xCmd = xPid->calculate(des_vel_[0], current_vel.vector.x);
//     float yCmd = yPid->calculate(des_vel_[1], current_vel.vector.y);
//     // ROS_INFO("expect velx is %.4f,expect vely is %.4f",des_vel_[0],des_vel_[1]);

//       xPid->useint();
//       yPid->useint();

//     // if(xCmd>MaxVel)xCmd=MaxVel;
//     // if(yCmd>MaxVel)yCmd=MaxVel;
//     sensor_msgs::Joy controlPosYaw;
//     controlPosYaw.axes.push_back(xCmd);
//     controlPosYaw.axes.push_back( yCmd);
//     controlPosYaw.axes.push_back( 0);
//     controlPosYaw.axes.push_back(0);// current_angular.z   

//       obtain_control();
//     ctrlVelPub.publish(controlPosYaw);

// }
//ladrc
void callback(const ros::TimerEvent& event)
{
  std_msgs::Float32MultiArray mypx,mypy,desv,desa;
      sensor_msgs::Joy controlPosYaw;
    //距离目标点小于0.2m,进行下一阶段
    // calculate_dis();
if(usepid){
     float xCmd = xPid->calculate(des_vel_[0], current_vel.vector.x);
     float yCmd = yPid->calculate(des_vel_[1], current_vel.vector.y);
      xPid->useint();
      yPid->useint();

    // float xCmd = fuzzypidx.realize(des_vel_[0], current_vel.vector.x);
    //  float yCmd =fuzzypidy.realize(des_vel_[1], current_vel.vector.y);

    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back(yCmd);
    }else{
    LADRC_Loop(&Yaw_Sysparam,&des_vel_[0],&current_vel.vector.x);
    LADRC_Loop(&Depth_Sysparam,&des_vel_[1],&current_vel.vector.y);
    controlPosYaw.axes.push_back(Yaw_Sysparam.u);
    controlPosYaw.axes.push_back( Depth_Sysparam.u);
    }
  //发布期望速度
  //   desv.data.push_back(des_vel_[0]);
  //   desv.data.push_back(des_vel_[1]);
  //   desvel_pub.publish(desv);

  //   desa.data.push_back(des_acc_[0]);
  //   desa.data.push_back(des_acc_[1]);
  // desacc_pub.publish(desa);
  //   ROS_INFO("expect velx is %.4f,current velx is %.4f, current output u is %.4f",des_vel_[0],current_vel.vector.x,Yaw_Sysparam.u);
  //   ROS_INFO("expect vely is %.4f,current vely is %.4f, current output u is %.4f",des_vel_[1],current_vel.vector.y,Depth_Sysparam.u);
  //   mypx.data.push_back(Yaw_Sysparam.z1);
  //   mypx.data.push_back(Yaw_Sysparam.z2);
  //  mypx.data.push_back(Yaw_Sysparam.z3);
  //  mypx.data.push_back(Yaw_Sysparam.v1);
  //   mypx.data.push_back(Yaw_Sysparam.v2);

  //      mypy.data.push_back(Depth_Sysparam.z1);
  //   mypy.data.push_back(Depth_Sysparam.z2);
  //  mypy.data.push_back(Depth_Sysparam.z3);
  //  mypy.data.push_back(Depth_Sysparam.v1);
  //   mypy.data.push_back(Depth_Sysparam.v2);
  //    ladrcx_pub.publish(mypx);
  //    ladrcy_pub.publish(mypy);


  //   controlPosYaw.axes.push_back(Yaw_Sysparam.u);
  //   controlPosYaw.axes.push_back( Depth_Sysparam.u);
    controlPosYaw.axes.push_back( 0);
    controlPosYaw.axes.push_back(0);// current_angular.z   

      obtain_control();
    ctrlVelPub.publish(controlPosYaw);

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_flight_pid_node");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);

  // 新增的
  ros::Subscriber localv = nh.subscribe("dji_sdk/velocity", 10, &local_velocity_callback);
  // ros::Subscriber pub_cmd = nh.subscribe("/planning/pos_cmd", 100, &control_callback);//
ros::Subscriber pub_cmd = nh.subscribe("/firefly/command/trajectory", 100, &circle_callback);
  // ros::Subscriber controlStatusSub = nh.subscribe("/dtu_controller/controlpid_status", 1, &pidStatusCallback );
  ros::Subscriber last_point_Sub = nh.subscribe("/dtu_controller/last_point", 1, &getlast_point );

  // Publish the control signal
  ctrlVelPub=nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 100);
    mysPub = nh.advertise<std_msgs::UInt8> ("/dtu_controller/update",1);
ok_pub = nh.advertise<geometry_msgs::PoseStamped>("/rviz/3d_nav_goal", 10);
ladrcx_pub = nh.advertise<std_msgs::Float32MultiArray>("/ladrc/param_velx", 10);
ladrcy_pub = nh.advertise<std_msgs::Float32MultiArray>("/ladrc/param_vely", 10);
desvel_pub = nh.advertise<std_msgs::Float32MultiArray>("/my/des_vel", 10);
desacc_pub = nh.advertise<std_msgs::Float32MultiArray>("/my/des_acc", 10);
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  LADRC_Init(&Yaw_Sysparam,5);
   LADRC_Init(&Depth_Sysparam,5);
   //fuzzy PID init
   int deltaKpMatrix[7][7]={{PB,PB,PM,PM,PS,ZO,ZO},
	                         {PB,PB,PM,PS,PS,ZO,NS},
						     {PM,PM,PM,PS,ZO,NS,NS},
	                         {PM,PM,PS,ZO,NS,NM,NM},
	                         {PS,PS,ZO,NS,NS,NM,NM},
	                         {PS,ZO,NS,NM,NM,NM,NB},
	                         {ZO,ZO,NM,NM,NM,NB,NB}};
	int deltaKiMatrix[7][7]={{NB,NB,NM,NM,NS,ZO,ZO},
	                         {NB,NB,NM,NS,NS,ZO,ZO},
						     {NB,NM,NS,NS,ZO,PS,PS},
	                         {NM,NM,NS,ZO,PS,PM,PM},
	                         {NM,NS,ZO,PS,PS,PM,PB},
	                         {ZO,ZO,PS,PS,PM,PB,PB},
	                         {ZO,ZO,PS,PM,PM,PB,PB}};
	int deltaKdMatrix[7][7]={{PS,NS,NB,NB,NB,NM,PS},
	                         {PS,NS,NB,NM,NM,NS,ZO},
						     {ZO,NS,NM,NM,NS,NS,ZO},
	                         {ZO,NS,NS,NS,NS,NS,ZO},
	                         {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
	                         {PB,NS,PS,PS,PS,PS,PB},
	                         {PB,PM,PM,PM,PS,PS,PB}};
	float e_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
	float de_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
	float Kp_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
	float Ki_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
	float Kd_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
	fuzzypidx.setMf("trimf",e_mf_paras,"trimf",de_mf_paras,"trimf",Kp_mf_paras,"trimf",Ki_mf_paras,"trimf",Kd_mf_paras);
	fuzzypidx.setRuleMatrix(deltaKpMatrix,deltaKiMatrix,deltaKdMatrix);
  fuzzypidy.setMf("trimf",e_mf_paras,"trimf",de_mf_paras,"trimf",Kp_mf_paras,"trimf",Ki_mf_paras,"trimf",Kd_mf_paras);
	fuzzypidy.setRuleMatrix(deltaKpMatrix,deltaKiMatrix,deltaKdMatrix);
 double desired_frequency = 50.0;  // 回调函数期望的频率为50Hz
    ros::Duration desired_period(1.0 / desired_frequency);
    timer = nh.createTimer(desired_period, callback);
  // ros::Timer loopTimer = nh.createTimer( ros::Duration(0.05),calculateDistanceCallback);
  bool obtain_control_result = obtain_control();
  bool takeoff_result;
  if (!set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }

    ROS_INFO("A3/N3 taking off!");
    takeoff_result = monitoredTakeoff();


  timer.start();
  ros::spin();
  return 0;
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

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}
