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

#include <cmath>
#include<vector>
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

#define row 37
#define dis_max 3
#define dis_min 0.8
#define angle_split 90
#define high 2

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;
double roll,pitch;
float current_yawRate,dis;
double MaxVel=1.0;
int c=-1;
int count_step=0;
int a=0;
float altitude=1.2;
  float x=0,y=0;
double current_angular_vel_z;
std::vector<Eigen::Vector2d> z,q;
int property[row][2];//属性数组，第一列代表前段，第二列代表后端，值为0表示直线，1表示曲线
const float s[row][2]={{3.1147,-2.7280},{1.7103,1.9594},{6.8969,-5.9907},{-4.6354,-1.1727},{-4.3903,1.2768},{-1.8848,1.3558},{-1.5149,-1.7553},{1.3963,-1.6052},{-1.1497,-0.5307},{-0.4204,-0.6146},{0.1980,-1.4452},{0.6225,-0.4964},{0.9417,-0.0921},{0.7014,0.3674},{0.4272,0.6326},{0.0769,0.7363},{5.4777,-1.3707},{5.1236,1.0470},{1.8386,1.1371},{1.8889,-1.6148},{2.3898,2.8985},{-1.6955,1.5947},{1.5207,4.7301},{-1.1182,5.0345},{-3.2949,3.6038},{-5.7683,1.5824},{-0.5704,-0.0049},{4.7090,-3.1625},{2.1907,-4.3879},{0.0315,-3.7493},{-0.5479,-1.2541},{-7.0252,6.0559},{2.3652,2.6730},{-1.6841,1.4268},{-2.3595,-0.6330},{-0.6963,0.0407},{-4.8658,-5.5447}};
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
std_msgs::Header header;

// Global random values
uint8_t controlStatus = STOP_CONTROLLER;
float loopFrequency=50;
bool start=false;
int withinboard=0;

Mission square_mission;

PID *xPid= new PID( 1/loopFrequency, 1.5,  -1.5, 1.0, 0.05,  0.5 );
PID *yPid= new PID( 1/loopFrequency, 1.5,  -1.5, 1.0, 0.05,  0.5 );


bool arePointsCollinear(const std::vector<Eigen::Vector2d>& points) {
	Eigen::MatrixXd A(points.size(), 2);
	Eigen::VectorXd b(points.size());

	for (size_t i = 0; i < points.size(); ++i) {
		A(i, 0) = points[i](0);
		A(i, 1) = 1;
		b(i) = points[i](1);
	}

	Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

	Eigen::VectorXd residuals = b - A * x;
	double maxResidual = residuals.lpNorm<Eigen::Infinity>();

	double threshold = 4e-2;

	return maxResidual <= threshold;
}


//  计算角度
double calculateAngle(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
	double dotProduct = p1.dot(p2);
	double normProduct = p1.norm() * p2.norm();
	return std::acos(dotProduct / normProduct) * 180.0 / C_PI;
}

// 计算角度
double calculateTriangleAngle(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3) {
	Eigen::Vector3d v1 = p2 - p1;
	Eigen::Vector3d v2 = p3 - p1;
	return calculateAngle(v1, v2);
}

//预处理函数,i代表当前点的索引，返回true ------ >   i++
bool preparework(Eigen::Vector3d p1,Eigen::Vector3d p2,Eigen::Vector3d p3,int i){
  double angle = calculateTriangleAngle(p1, p2, p3);
  double dis = (p2 - p1).norm();
  if((dis>dis_max) and (angle<=90)){
    property[i][1]=0;//当前点后直
    property[i+1][0]=0;//下一点前直
    return false;
  }else if((dis <dis_min) and (angle >90)){
        property[i][1]=1;//当前点后曲
    property[i+1][0]=1;//下一点前曲
    return false;
  }else{
    //dis>3 angle>90   ;  dis<0.8 angle <=90   ;  0.8<dis<3  angle 任意
    std::vector<Eigen::Vector2d> points;
    points.push_back(Eigen::Vector2d(p1.x(),p1.y()));
    points.push_back(Eigen::Vector2d(p2.x(), p2.y()));
    points.push_back(Eigen::Vector2d(p3.x(),p3.y()));

    bool collinear = arePointsCollinear(points);
    if(collinear){
        property[i][1]=0;//当前点后直
        property[i+1][0]=0;//下一点前直
        property[i+1][1]=0;//下一点后直
        property[i+2][0]=0;//下下一点前直
        return true;
    }else if(angle <= 90){
        property[i][1]=1;//当前点后曲
        property[i+1][0]=1;//下一点前曲
        return false;
    }else{
        property[i][1]=1;//当前点后曲
        property[i+1][0]=1;//下一点前曲
        property[i+1][1]=1;//下一点后曲
        property[i+2][0]=1;//下下一点前曲
        return true;
    }
  }
}

//处理每个点
 void copewithpoints(){
  int i=0;
    //给属性数组赋值
    for(;i<row;i++){
        Eigen::Vector3d p1(s[i+1][0],s[i+1][1],high);
        Eigen::Vector3d p2(s[i][0],s[i][1],high);
        Eigen::Vector3d p3(s[i+2][0],s[i+2][1],high);
        if(preparework(p1,p2,p3,i)){
          i++;
        }
    }
 }

//飞直线
Eigen::Vector2d exec_zx(){
  float x=0,y=0;
  // for(int tm=0;tm<z.size();tm++){
    x=z[0](0);
    y=z[0](1);
  // }
  return Eigen::Vector2d(x,y);
}

//飞曲线
void exec_qx(){
  geometry_msgs::PoseStamped pos;
  pos.pose.position.x=current_local_pos.x;
  pos.pose.position.y=current_local_pos.y;
  for(int i=0;i<q.size();i++){
    pos.header=header;
    pos.pose.position.x+=q[i](0);
    pos.pose.position.y+=q[i](1);
     curPub.publish(pos);
  }
  pos.pose.position.z=-1;
  curPub.publish(pos);
}

void getlast_point(const geometry_msgs::Point::ConstPtr &last_point){

    lastpoint.x=last_point->x;
    lastpoint.y=last_point->y;
    lastpoint.z=last_point->z;
    ROS_INFO("update last point  x=%.4f,y=%.5f",lastpoint.x,lastpoint.y);
    // c++;
    // if(c>2){
    //   xPid->reset_control();
    //   yPid->reset_control();
    // }
}

void checkControlStatusCallback( const std_msgs::UInt8 value )
{
  if( controlStatus != value.data ) ROS_INFO("Control Status changed to %d", value.data);
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

void local_velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
  current_vel.vector.x=msg->vector.x;
  current_vel.vector.y=msg->vector.y;
  current_vel.vector.z=msg->vector.z;

}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  current_imu=msg->orientation;
  current_angular_vel_z=msg->angular_velocity.z;
  header=msg->header;
}

void control_callback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd)
{
  Eigen::Vector3d des_pos_,des_vel_,des_acc_;
  double yaw,yaw_rate;

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

    if( controlStatus == RUNNING)
  {
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
    // if(dis<1 and !start){
    //    controlStatus=STOP_CONTROLLER;
    //   //  std_msgs::UInt8 msg;
    //   //   msg.data=0;
    //   //  set_complete_status.publish(msg);
    //    ROS_INFO("dis < 1,stop control");
    // }
      obtain_control();
    ctrlVelPub.publish(controlPosYaw);
  }
  else
  {
    // ROS_INFO("control is stop_control");

    // xPid->reset_control();
    // yPid->reset_control();

    // sensor_msgs::Joy controlVelYawRate;

    // controlVelYawRate.axes.push_back(0);
    // controlVelYawRate.axes.push_back(0);
    // controlVelYawRate.axes.push_back(0);
    // controlVelYawRate.axes.push_back(0);

    // ctrlBrakePub.publish(controlVelYawRate);
  }
}
}

void change_state(const std_msgs::UInt8 &msg){
  // if(a==2){
  //   square_mission.state = 4;
  //   controlStatus=STOP_CONTROLLER;
  //   a=0;
  // }else if((count_step==1 || count_step==2)&& a==1){
  //   square_mission.state = 3;
  //   controlStatus=RUNNING;
  //   a=2;
  // }
  // else{
      square_mission.state = 2;
    controlStatus=STOP_CONTROLLER;
  // }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_flight_control_node");
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
  ros::Subscriber pub_cmd = nh.subscribe("/planning/pos_cmd", 10, &control_callback,ros::TransportHints().tcpNoDelay());//

  ros::Subscriber controlStatusSub = nh.subscribe("/dtu_controller/control_status", 1, &checkControlStatusCallback );
  ros::Subscriber last_point_Sub = nh.subscribe("/dtu_controller/last_point", 1, &getlast_point );
    ros::Subscriber stateSub=nh.subscribe("/change_state", 1,&change_state);
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
  takeoff_result = monitoredTakeoff();
  
  //给属性数组赋值
  copewithpoints();

  if(takeoff_result)
  {
    square_mission.reset();
    square_mission.start_gps_location = current_gps;
    square_mission.start_local_position = current_local_pos;
    square_mission.setTarget(3.1147,-2.7280,altitude, 0);
    square_mission.state = 2;
    // square_mission.finished=true;
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

  // Down sampled to 50Hz loop
  if(elapsed_time > ros::Duration(0.02))
  {
    start_time = ros::Time::now();
    switch(square_mission.state)
    {
      case 0:
        break;

      case 1:
        if(!square_mission.finished)
        {
          square_mission.step();
        }
        else
        {
          square_mission.reset();
          // square_mission.start_gps_location = current_gps;
          // square_mission.start_local_position = current_local_pos;

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
          square_mission.reset();

          //当前点后直 下一点前直   传入下一点到直线数组
          if((property[count_step][1] ==0) and (property[count_step+1][0] ==0)){
            // z.push_back(Eigen::Vector2d(s[count_step+1][0],s[count_step+1][1]));
            // if((property[count_step+1][0] ==0) and (property[count_step+1][1] ==1)){
              //飞直线数组，进入状态1
              // Eigen::Vector2d v=exec_zx();
              square_mission.start_gps_location = current_gps;
               square_mission.start_local_position = current_local_pos;
               square_mission.setTarget(s[count_step+1][0],s[count_step+1][1], 0, 0);
              count_step++;
              // z.clear();
              square_mission.state = 1;
              ROS_INFO("#####-------line search  -------------------.%d.-----------..",count_step);
              // ROS_INFO("##### Start route %d ....", square_mission.state);
            // }
          }else if((property[count_step][1] ==1) and (property[count_step+1][0] ==1)){//当前点后曲 下一点前曲   传入下一点到曲线数组
            q.push_back(Eigen::Vector2d(s[count_step+1][0],s[count_step+1][1]));
            if((property[count_step+1][0] ==1) and (property[count_step+1][1] ==0)){
              //飞曲线数组，进入状态0
              exec_qx();
              q.clear();
              square_mission.state = 0;
              // ROS_INFO("##### Start route %d ....", square_mission.state);
            }
            count_step++;
            ROS_INFO("#########curve search  -------------------.%d.-----------..",count_step);
          }else{
            //  count_step++;
            ROS_INFO("#####impossible  --..");
          }
         
          if(count_step>=37){
            square_mission.state = 0;//结束
          }
          //  square_mission.state = 0;//结束
          
          // ROS_INFO("##### Start route %d ....", square_mission.state);
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
