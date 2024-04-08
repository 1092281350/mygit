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
#include "dji_sdk_demo/PIDController.h"
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
uint8_t controlStatus = STOP_CONTROLLER;
float loopFrequency=50;
bool start=false;
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

PID *xPid= new PID( 1/loopFrequency, 1.5,  -1.5, 1.0, 0.05,  0.5 );
PID *yPid= new PID( 1/loopFrequency, 1.5,  -1.5, 1.0, 0.05,  0.5 );

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
         ServiceAck service_ack;
  if(a==2){
    // square_mission.state = 4;
    my.data=4;
    controlStatus=STOP_CONTROLLER;
    m=0;
    service_ack = releaseCtrlAuthority();
      if (service_ack.result) {
        ROS_INFO("Release SDK control Authority successfully in PID 's control callback function ");
        start=true;
      } else {
        ROS_WARN("Failed release SDK control Authority");
      }
  }else if((count_step==1 || count_step==2)&& a==1){
    // square_mission.state = 3;
        my.data=3;
    controlStatus=RUNNING;
    m=2;
    y=count_step;
    ROS_INFO("count step is 1  in PID 's control callback function ");
  }
  else{
    //   square_mission.state = 4;
        my.data=4;
    controlStatus=STOP_CONTROLLER;
    service_ack = releaseCtrlAuthority();
      if (service_ack.result) {
        ROS_INFO("Release SDK control Authority successfully in PID 's control callback function ");
        start=true;
      } else {
        ROS_WARN("Failed release SDK control Authority");
      }
      cl++;
  }
  ROS_INFO("dis < 0.5,control status is %d, a is %d in PID 's ct callback function ,my data is %d",controlStatus,a,my.data);
  mysPub.publish(my);
}

void getlast_point(const geometry_msgs::Point::ConstPtr &last_point){

    lastpoint.x=last_point->x;
    lastpoint.y=last_point->y;
    lastpoint.z=last_point->z;
    ROS_INFO("update last point  x=%.4f,y=%.5f in PID 's getlast_point callback function ",lastpoint.x,lastpoint.y);

}
void calculate_dis()
{
  Eigen::Vector3d l(lastpoint.x,lastpoint.y,0);
    Eigen::Vector3d cur(current_local_pos.x,current_local_pos.y,0);
    std_msgs::UInt8 my;
    float dis;
    // ROS_INFO("CURRENT LOCAL X IS %.4f, LOCAL Y IS %.4f",current_local_pos.x,current_local_pos.y);
    // ROS_INFO("last  point X IS %.4f,  Y IS %.4f,cl is %d",lastpoint.x,lastpoint.y,cl);
    dis= (l-cur).norm();
    if(cl ==0 && dis<8.25){   
       if(current_vel.vector.x<=0.1 &&current_vel.vector.y<=0.1 ){   
        withinbounder++;
      }else{
        withinbounder=0;
      }
      if(withinbounder >400){
        withinbounder=0;
        ct(m,y);
      }
    }else if(cl>0){
        if(current_vel.vector.x<=0.1 &&current_vel.vector.y<=0.1 ){   
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
void change_acount(const std_msgs::UInt8 &msg){
    int a=msg.data;
    m=a%10;
    int count_step=0;
    int tmp=a/10;
    ROS_INFO("State change successfully,is %d. count_step is %d in PID 's change_state callback function ",m,tmp);
    if(tmp == 1)
    {
        count_step=1;
        y=1;
    }else if(tmp == 2)
    {
        count_step=2;
        y=2;
    }else{
        count_step=0;
        y=0;
    }
}
// void pidStatusCallback( const std_msgs::UInt8 value )
// {
//   if( controlStatus != value.data ) ROS_INFO("Control Status changed to %d in PID 's Status callback function ", value.data);
//   if( value.data == RESET_CONTROLLERS ) {
//     ROS_INFO("RESETING PID CONTROL");
//     controlStatus = STOP_CONTROLLER;
//   }
//   else
//   {
//     controlStatus = value.data;
//     if(controlStatus==2){
//         start=true;
//     }
//   }
// }
void checkControlStatusCallback( const std_msgs::UInt8 value )
{
  if( controlStatus != value.data ) ROS_INFO("Control Status changed to %d ", value.data);
  if( value.data == RESET_CONTROLLERS ) {
    ROS_INFO("RESETING PID CONTROL");
    controlStatus = STOP_CONTROLLER;
  }
  else
  {
    controlStatus = value.data;
    if(controlStatus==2){
        start=true;
    }
  }
}
void callback(const ros::TimerEvent& event)
{

    if( controlStatus == RUNNING)
     {
    //距离目标点小于0.2m,进行下一阶段
    calculate_dis();
    float xCmd = xPid->calculate(des_vel_[0], current_vel.vector.x);
    float yCmd = yPid->calculate(des_vel_[1], current_vel.vector.y);
    // ROS_INFO("expect velx is %.4f,expect vely is %.4f",des_vel_[0],des_vel_[1]);

      xPid->useint();
      yPid->useint();

    // if(xCmd>MaxVel)xCmd=MaxVel;
    // if(yCmd>MaxVel)yCmd=MaxVel;
    sensor_msgs::Joy controlPosYaw;
    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back( yCmd);
    controlPosYaw.axes.push_back( 0);
    controlPosYaw.axes.push_back(0);// current_angular.z   

    //   obtain_control();
    // ctrlVelPub.publish(controlPosYaw);
     }else{
      ServiceAck service_ack;
      if(!start){
        service_ack = releaseCtrlAuthority();
        if (service_ack.result) {
          ROS_INFO("Release SDK control Authority successfully in PID 's control callback function ");
          start=true;
        } else {
          ROS_WARN("Failed release SDK control Authority");
        }
        }
     }
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_flight_pid_node");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);

  // 新增的
  ros::Subscriber localv = nh.subscribe("dji_sdk/velocity", 10, &local_velocity_callback);
  ros::Subscriber pub_cmd = nh.subscribe("/planning/pos_cmd", 100, &control_callback,ros::TransportHints().tcpNoDelay());//

  // ros::Subscriber controlStatusSub = nh.subscribe("/dtu_controller/controlpid_status", 1, &pidStatusCallback );
  ros::Subscriber last_point_Sub = nh.subscribe("/dtu_controller/last_point", 1, &getlast_point );
    ros::Subscriber auxSub=nh.subscribe("/dtu_controller/aux", 1,&change_acount);
  // next_pharse=nh.advertise<std_msgs::UInt8>("/dtu_controller/next_pharse",1);
      ros::Subscriber controlStatusSub = nh.subscribe("/dtu_controller/control_status", 1, &checkControlStatusCallback);
  // Publish the control signal
  ctrlVelPub=nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 100);
    mysPub = nh.advertise<std_msgs::UInt8> ("/dtu_controller/update",1);
//    ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
//      ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);

  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

 double desired_frequency = 50.0;  // 回调函数期望的频率为50Hz
    ros::Duration desired_period(1.0 / desired_frequency);
    timer = nh.createTimer(desired_period, callback);
  // ros::Timer loopTimer = nh.createTimer( ros::Duration(0.05),calculateDistanceCallback);
  obtain_control();

  timer.start();
  ros::spin();
  return 0;
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

