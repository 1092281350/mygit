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
double current_angular_vel_z;
ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlPosPub;
ros::Publisher ctrlBrakePub;
 ros::Publisher set_complete_status;
 ros::Publisher ctrlVelPub;

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
bool referenceUpdated = false;
// geometry_msgs::Twist goalReference;

Mission square_mission;
PID *xPid= new PID( 1/loopFrequency, 1.5,  -1.5, 1.0, 0.05,  0.5 );
PID *yPid= new PID( 1/loopFrequency, 1.5,  -1.5, 1.0, 0.05,  0.5 );
PIDController rollPIDMatlab = PIDController(0.02);
PIDController pitchPIDMatlab = PIDController(0.02);
PID *yawRatePid = new PID( 1/loopFrequency, 0.8, -0.8, 1.5, 0.0, 0.1 );

PID *velxPid= new PID( 1/loopFrequency, 1.5,  -1.5, 1.0, 0.05,  0.5 );
PID *velyPid= new PID( 1/loopFrequency, 1.5,  -1.5, 1.0, 0.05,  0.5 );

// void updateReferenceCallback( const geometry_msgs::Twist reference )
// {
//   // ROS_INFO("Reference udpated");
//   // goalReference = reference;
//   referenceUpdated = true;
// }
void calculateDistanceCallback( ){//const ros::TimerEvent&
  Eigen::Vector3d l(lastpoint.x,lastpoint.y,0);
    Eigen::Vector3d cur(current_local_pos.x,current_local_pos.y,0);
    dis= (l-cur).norm();
    if(dis<1.0){
      controlStatus=STOP_CONTROLLER;
      // std_msgs::UInt8 msg;
      //   msg.data=0;
      // set_complete_status.publish(msg);
    }
    // ROS_INFO("dis is %.4f",dis);
}
void getlast_point(const geometry_msgs::Point::ConstPtr &last_point){
    // Eigen::Vector3d l(last_point->x,last_point->y,0);
    // Eigen::Vector3d cur(current_local_pos.x,current_local_pos.y,0);
    // dis= (l-cur).norm();
    lastpoint.x=last_point->x;
    lastpoint.y=last_point->y;
    lastpoint.z=last_point->z;
    ROS_INFO("update last point  x=%.4f,y=%.5f",lastpoint.x,lastpoint.y);
}
float getYawRate(const geometry_msgs::Vector3 u, const tf::Vector3 w)
{
  tf::Matrix3x3 R_y;
  tf::Matrix3x3 R_z;
  R_y.setRPY(0.0, u.y, 0.0);
  R_z.setRPY(0.0, 0.0, u.z);
  tf::Vector3 E_1 = (R_z * R_y).getColumn(0);
  tf::Vector3 E_2 = R_z.getColumn(1);
  return (w - w.dot(E_1) * E_1 - w.dot(E_2) * E_2).getZ();
}

void checkControlStatusCallback( const std_msgs::UInt8 value )
{
  if( controlStatus != value.data ) ROS_INFO("Control Status changed to %d", value.data);
  if( value.data == RESET_CONTROLLERS ) {
    ROS_INFO("RESETING PID CONTROL");

    yawRatePid->reset_control();

    controlStatus = STOP_CONTROLLER;
    // std_msgs::UInt8 msg;
    //     msg.data=0;
    // set_complete_status.publish(msg);
  }
  else
  {
    controlStatus = value.data;
  }
}

//无用定义，防止报错
class Imu_Data_t{


};

// class Command_Data_t{

// };
//Odom_Data_t 结构定义
class Odom_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Quaterniond q;
    Eigen::Vector3d w;

    nav_msgs::Odometry msg;
    ros::Time rcv_stamp;
    bool recv_new_msg;

    Odom_Data_t(){};
    void feed(nav_msgs::OdometryConstPtr pMsg) {

    ros::Time now = ros::Time::now();

    msg = *pMsg;
    rcv_stamp = now;
    recv_new_msg = true;

    

#define VEL_IN_BODY
#ifdef VEL_IN_BODY /* Set to 1 if the velocity in odom topic is relative to current body frame, not to world frame.*/
    Eigen::Quaternion<double> wRb_q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    Eigen::Matrix3d wRb = wRb_q.matrix();
    v = wRb * v;

    static int count = 0;
    if (count++ % 500 == 0)
        ROS_WARN("VEL_IN_BODY!!!");
#endif

    // check the frequency
    static int one_min_count = 9999;
    static ros::Time last_clear_count_time = ros::Time(0.0);
    if ((now - last_clear_count_time).toSec() > 1.0 ) {

        if ( one_min_count < 100 ) {

            ROS_WARN("ODOM frequency seems lower than 100Hz, which is too low!");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count ++;
}
};

//Controller_Output_t结构定义
struct Controller_Output_t
{

    // Orientation of the body frame with respect to the world frame
    Eigen::Quaterniond q;
    Eigen::Quaterniond orientation;
  
    // Body rates in body frame
    Eigen::Vector3d bodyrates; // [rad/s]
    double roll_rate;
	double pitch_rate;
	double yaw_rate;
    double normalized_thrust;
    // Collective mass normalized thrust
    double thrust;

    Eigen::Vector3d des_v_real;
};

// Desired_State_t 结构定义
template <typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 1> quaternion_to_ypr(const Eigen::Quaternion<Scalar_t>& q_) {
    Eigen::Quaternion<Scalar_t> q = q_.normalized();

    Eigen::Matrix<Scalar_t, 3, 1> ypr;
    ypr(2) = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    ypr(1) = asin(2 * (q.w() * q.y() - q.z() * q.x()));
    ypr(0) = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

    return ypr;
}

template <typename Scalar_t>
geometry_msgs::Quaternion to_quaternion_msg(const Eigen::Quaternion<Scalar_t>& q) {
    geometry_msgs::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
}

template <typename Scalar_t>
Scalar_t get_yaw_from_quaternion(const Eigen::Quaternion<Scalar_t>& q) {
    return quaternion_to_ypr(q)(0);
}


template <typename Scalar_t>
Scalar_t normalize_angle(Scalar_t a) {
    int cnt = 0;
    while (true) {
        cnt++;

        if (a < -M_PI) {
            a += M_PI * 2.0;
        } else if (a > M_PI) {
            a -= M_PI * 2.0;
        }

        if (-M_PI <= a && a <= M_PI) {
            break;
        };

        assert(cnt < 10 && "[uav_utils/geometry_msgs] INVALID INPUT ANGLE");
    }

    return a;
}

template <typename T, typename T2>
void
limit_range(T& value, const T2& low, const T2& high)
{
  ROS_ASSERT_MSG(low < high, "%f < %f?", low, high);
  if (value < low)
  {
    value = low;
  }

  if (value > high)
  {
    value = high;
  }

  return;
}
template <typename T, typename T2>
void
limit_range(T& value, const T2& limit)
{
  ROS_ASSERT_MSG(limit > 0, "%f > 0?", limit);
  limit_range(value, -limit, limit);
}

class Command_Data_t
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    double yaw;
    double yaw_rate;
    double head_rate;
    bool cmd_init;

    quadrotor_msgs::PositionCommand msg;
    ros::Time rcv_stamp;

    Command_Data_t(){};
    void feed(quadrotor_msgs::PositionCommandConstPtr pMsg) {

    static double last_time;
    static double last_yaw;
    double now_time;

    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    now_time = ros::Time::now().toSec();

    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;

    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;

    a(0) = msg.acceleration.x;
    a(1) = msg.acceleration.y;
    a(2) = msg.acceleration.z;

    j(0) = msg.jerk.x;
    j(1) = msg.jerk.y;
    j(2) = msg.jerk.z;

    // std::cout << "j1=" << j.transpose() << std::endl;

    yaw = normalize_angle(msg.yaw);
    yaw_rate = msg.yaw_dot;
    if(!cmd_init){
        last_time = now_time;
        head_rate = 0.0;
        last_yaw = yaw;
    }
    else{
        double diff_time = now_time-last_time;
        last_time = now_time;
        double diff_yaw;
        double angle1 = yaw;
        double angle2 = last_yaw;
        last_yaw = yaw;

        double TwoPi = 2*M_PI;
        if (angle1 < 0)
            angle1 = TwoPi + angle1;
        if (angle2 < 0)
            angle2 = TwoPi + angle2;
        double dist = angle1 - angle2;
        if (dist > M_PI)
            angle1 = angle1 - TwoPi;
        //if two much on other side then invert second angle
        else if (dist < -M_PI)
            angle2 = angle2 - TwoPi;
        diff_yaw = (angle1-angle2);
        diff_time = 0.01;//hzchzc
        head_rate = diff_yaw/diff_time;
        limit_range(head_rate,1.0);     
        // printf("angle1: %f, angle2: %f, head_rate: %f \n, diff_time: %f",angle1,angle2,head_rate,diff_time);
    }
    cmd_init = true;
}
};

struct Desired_State_t
{
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    Eigen::Quaterniond q;
    double yaw;
    double yaw_rate;
    double head_rate;

    Desired_State_t(){};

    Desired_State_t(geometry_msgs::QuaternionStamped &quat,Eigen::Vector3d pos)
        : p(pos),
          v(Eigen::Vector3d::Zero()),
          a(Eigen::Vector3d::Zero()),
          j(Eigen::Vector3d::Zero()),
          q(Eigen::Quaterniond(quat.quaternion.w, quat.quaternion.x, quat.quaternion.y, quat.quaternion.z)),
          yaw(get_yaw_from_quaternion(Eigen::Quaterniond(quat.quaternion.w, quat.quaternion.x, quat.quaternion.y, quat.quaternion.z))),
          yaw_rate(0),
          head_rate(0){};
      void setpvay(Eigen::Vector3d cp,Eigen::Vector3d cv,Eigen::Vector3d ca,double cyaw,double cyaw_rate){
        p=cp;
        v=cv;
        a=ca;
        yaw=cyaw;
        yaw_rate=cyaw_rate;
      };
};

Desired_State_t get_cmd_des(const Command_Data_t& cmd_data) {

    Desired_State_t des;
    des.p = cmd_data.p;
    des.v = cmd_data.v;
    des.a = cmd_data.a;
    des.j = cmd_data.j;
    des.yaw = cmd_data.yaw;
    des.yaw_rate = cmd_data.yaw_rate;

    return des;
}
//  ------------------------------------------------------------------------------全局变量声明------------------------------------------------------------------------------
Desired_State_t des;
Controller_Output_t u;
Command_Data_t cmd_data;
geometry_msgs::QuaternionStamped myquat;

//   publish_attitude_ctrl(u, now_time);  
 void publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp) {

    myquat.header.stamp = stamp;
    myquat.header.frame_id = std::string("FCU");

    myquat.quaternion.x = u.q.x();
    myquat.quaternion.y = u.q.y();
    myquat.quaternion.z = u.q.z();
    myquat.quaternion.w = u.q.w();

    // msg.thrust = u.thrust;

    // ctrl_FCU_pub.publish(msg);
}

// DLQR_Control(des, odom_data, imu_data, u); 
Eigen::Vector3d int_e_v;
Eigen::Matrix3d Kp;
Eigen::Matrix3d Kv;
Eigen::Matrix3d Kvi;
Eigen::Matrix3d Ka;
double Kyaw;
double dt;
Eigen::MatrixXd A, B, Q, N,R,K;

void DARE(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                    const Eigen::MatrixXd &N, Eigen::MatrixXd *K, const double eps ) {
  // check if dimensions are compatible
  if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() ||
      Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols() ||
      N.rows() != A.rows() || N.cols() != B.cols()) {
    std::cout << "One or more matrices have incompatible dimensions. Aborting."
              << std::endl;
  }

  // precompute as much as possible
  Eigen::MatrixXd B_T = B.transpose();
  Eigen::MatrixXd Acal = A - B * R.inverse() * N.transpose();
  Eigen::MatrixXd Acal_T = Acal.transpose();
  Eigen::MatrixXd Qcal = Q - N * R.inverse() * N.transpose();

  // initialize P with Q
  Eigen::MatrixXd P = Q;

  // iterate until P converges
  unsigned int numIterations = 0;
  Eigen::MatrixXd Pold = P;
  while (true) {
    numIterations++;

    // compute new P
    P = Acal_T * P * Acal -
        Acal_T * P * B * (R + B_T * P * B).inverse() * B_T * P * Acal + Qcal;

    // update delta
    Eigen::MatrixXd delta = P - Pold;
    if (fabs(delta.maxCoeff()) < eps) {
    //   std::cout << "Number of iterations until convergence: " << numIterations
    //             << std::endl;
      break;
    }
    Pold = P;
  }

  // compute K from P
  *K = (R + B_T * P * B).inverse() * (B_T * P * A + N.transpose());
}

#define param_normal_gain_Kv2 3.5
 #define param_normal_gain_Kp2 1.5
 #define ctrl_freq_max 50.0
 #define param_gra 9.81

// imu ,odom参数不需要
void DLQR_Control(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u) {

    Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
    dt = 1.0 / (double)ctrl_freq_max;
    // [[1, 0, dt, 0],
    //  [0, 1, 0, dt],
    //  [0, 0, 1, 0],
    //  [0, 0, 0, 1]]
    A = Eigen::MatrixXd::Identity(4, 4);
    A(0, 2) = dt;
    A(1, 3) = dt;
    B = Eigen::MatrixXd::Zero(4, 2);
    B(2, 0) = dt;
    B(3, 1) = dt;
    // [[0, 0],
    // [0, 0],
    // [dt, 0],
    // [0, dt]]
    Q = 2 * Eigen::MatrixXd::Identity(4,4);
	R = 0.1* Eigen::MatrixXd::Identity(2,2);
    N= Eigen::MatrixXd::Zero(4, 2);
    DARE(A, B, Q, R, N, &K,1e-15);
    // std::cout << K << std::endl;

    const Eigen::Vector4d state_des = Eigen::Vector4d(des.p(0), des.p(1), des.v(0), des.v(1));
    const Eigen::Vector4d state_now = Eigen::Vector4d(current_local_pos.x, current_local_pos.y, current_vel.vector.x,current_vel.vector.y);
    const Eigen::Vector2d des_a = Eigen::Vector2d(des.a(0), des.a(1));
    Eigen::Vector2d out_acc = -K * (  state_now-state_des) + des_a;

    des_acc(0) = out_acc(0);
    des_acc(1) =out_acc(1);
    des_acc(2) = param_normal_gain_Kv2* (des.v(2) -current_vel.vector.z) + param_normal_gain_Kp2 * (des.p(2) - current_local_pos.z)+des.a(2);
    des_acc += Eigen::Vector3d(0,0,param_gra);
    
    // u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
    double yaw;
    Eigen::Quaterniond mycur_attu;
    mycur_attu.x()=current_atti.x;
    mycur_attu.y()=current_atti.y;
    mycur_attu.z()=current_atti.z;
    mycur_attu.w()=current_atti.w;
    Eigen::Quaterniond mycur_imu;
    mycur_imu.x()=current_imu.x;
    mycur_imu.y()=-current_imu.y;
    mycur_imu.z()=current_imu.z;
    mycur_imu.w()=current_imu.w;
    double yaw_odom = atan2(2 * (mycur_attu.x()*mycur_attu.y() + mycur_attu.w()*mycur_attu.z()), mycur_attu.w()*mycur_attu.w() + mycur_attu.x()*mycur_attu.x() - mycur_attu.y()*mycur_attu.y() - mycur_attu.z()*mycur_attu.z());
    double sin = std::sin(yaw_odom);
    double cos = std::cos(yaw_odom);
    roll = (des_acc(0) * sin - des_acc(1) * cos )/ param_gra;
    pitch = (des_acc(0) * cos + des_acc(1) * sin )/ param_gra;
    
    // yaw_imu = toEulerAngle(current_atti);//根据四元数计算出欧拉角，ros的odom消息机制为ZYX，无人机的是ZXY
    // Eigen::Quaterniond q = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())
    //   * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
    //   * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());

    Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw,Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
    u.q = mycur_imu * mycur_attu.inverse() * q;// Align with FCU frame

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

void control_callback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd)
{
  Eigen::Vector3d des_pos_,des_vel_,des_acc_;
  double yaw,yaw_rate;
  Odom_Data_t odom_data;
  Imu_Data_t imu_data;
sensor_msgs::Joy controlPosYaw;

  des_pos_= Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_= Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_= Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                             cmd->acceleration.z);
  yaw              = cmd->yaw;
  yaw_rate          = cmd->yaw_dot;

  des.setpvay(des_pos_,des_vel_,des_acc_,yaw,yaw_rate);
  // DLQR_Control(des, odom_data, imu_data, u); 



// float pitchCmd = pitchPIDMatlab.update(currentReference.linear.x, currentPose.linear.x);
// float rollCmd = rollPIDMatlab.update(currentReference.linear.y, currentPose.linear.y);

  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  
  // Down sampled to 50Hz loop
  if(elapsed_time > ros::Duration(0.02))
  {   
    calculateDistanceCallback();
    //  geometry_msgs::Quaternion quat;
    // geometry_msgs::Vector3 a;
    
    // quat=to_quaternion_msg(u.q);//FCU
    // a=toEulerAngle(quat);
    // controlPosYaw.axes.push_back(a.x);
    // controlPosYaw.axes.push_back(a.y);
    // controlPosYaw.axes.push_back(1.2);
    // controlPosYaw.axes.push_back(a.z);
    

    if( controlStatus == RUNNING)
  {

    // rampReferenceUpdate();
    //PID计算yawRate
    // float xCmd = xPid->calculate(des_pos_[0], current_local_pos.x);
    // float yCmd = yPid->calculate(des_pos_[1], current_local_pos.y);
    double start_int=(ros::Time::now() - start_time).toSec();
    float xCmd = xPid->calculate(des_vel_[0], current_vel.vector.x);
    float yCmd = yPid->calculate(des_vel_[1], current_vel.vector.y);
    // ROS_INFO("expect velx is %.4f,expect vely is %.4f",des_vel_[0],des_vel_[1]);
    // if(start_int>3.0){
      xPid->useint();
      yPid->useint();
    //   // ROS_INFO("---------------------use int---------------------");
    // }
    //    float pitchCmd = pitchPIDMatlab.update(des_pos_[0], current_local_pos.x);
    // float rollCmd = rollPIDMatlab.update(des_pos_[1], current_local_pos.y);

    // float  yawRate = yawRatePid->calculate(yaw_rate,current_angular_vel_z);
    tf2::Quaternion q_orig, q_rot, q_new;
    
    // double r=3.14159, p=0, y=0;  // Rotate the previous pose by 180* about X
    q_rot.setRPY(-current_angular.x, -current_angular.y, current_angular.z);
    tf2::Matrix3x3 rot(q_rot);
    tf2::Vector3 cmdVal(pitch, roll, 0);

    // ROS_INFO("%.3f %.3f %.3f %.3f", q_rot.getX(), q_rot.getY(), q_rot.getZ(), q_rot.getW());

    cmdVal = cmdVal*rot;

    // ROS_INFO("%.3f %.3f | %.3f %.3f", pitchCmd, rollCmd, cmdVal[0], -cmdVal[1]);


    // controlPosYaw.axes.push_back(- cmdVal[1]);
    // controlPosYaw.axes.push_back( cmdVal[0]);

    // float xCmd = sin(current_angular.z) * pitchCmd - cos(current_angular.z) * rollCmd;
    // float yCmd= cos(current_angular.z) * pitchCmd + sin(current_angular.z) * rollCmd;

    // controlPosYaw.axes.push_back(xCmd);
    // controlPosYaw.axes.push_back( yCmd);

    if(xCmd>MaxVel)xCmd=MaxVel;
    if(yCmd>MaxVel)yCmd=MaxVel;

        controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back( yCmd);
    controlPosYaw.axes.push_back( 0);
    controlPosYaw.axes.push_back(0);// current_angular.z   
    if(dis<1){
       controlStatus=STOP_CONTROLLER;
      //  std_msgs::UInt8 msg;
      //   msg.data=0;
      //  set_complete_status.publish(msg);
       ROS_INFO("dis < 1,stop control");
    }
    // controlValue.axes[0] = sin(currentPose.angular.z) * pitchCmd - cos(currentPose.angular.z) * rollCmd;
    // controlValue.axes[1] = cos(currentPose.angular.z) * pitchCmd + sin(currentPose.angular.z) * rollCmd;

    // controlValue.axes[1] = cos(currentPose.angular.z) * pitchCmd + sin(currentPose.angular.z) * rollCmd;
    // controlValue.axes[0] = -(-sin(currentPose.angular.z) * pitchCmd + cos(currentPose.angular.z) * rollCmd);

  //  ctrlPosYawPub.publish(controlPosYaw);
    // controlValuePub.publish(controlValue);
    ctrlVelPub.publish(controlPosYaw);
  }
  else if( controlStatus == STOP_CONTROLLER )
  {
    ROS_INFO("control is stop_control");
    // xPid->reset_control();
    // yPid->reset_control();
    velxPid->reset_control();
    velyPid->reset_control();
    // pitchPIDMatlab.reset();
    // rollPIDMatlab.reset();
    yawRatePid->reset_control();
    // controlPosYaw.axes.push_back(0);
    // controlPosYaw.axes.push_back(0);
    // controlPosYaw.axes.push_back(1.2);
    // controlPosYaw.axes.push_back(0);
    sensor_msgs::Joy controlVelYawRate;
    // uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
    //             DJISDK::HORIZONTAL_VELOCITY |
    //             DJISDK::YAW_RATE            |
    //             DJISDK::HORIZONTAL_GROUND   |
    //             DJISDK::STABLE_ENABLE);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    // controlVelYawRate.axes.push_back(flag);

    ctrlBrakePub.publish(controlVelYawRate);
  }
  else{
    ROS_WARN("control is reset_control");
  }
  
  }
  
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
  ros::Subscriber pub_cmd = nh.subscribe("/planning/pos_cmd", 100, &control_callback,
                                                  ros::TransportHints().tcpNoDelay());

  ros::Subscriber controlStatusSub = nh.subscribe("/dtu_controller/control_status", 1, &checkControlStatusCallback );
  ros::Subscriber last_point_Sub = nh.subscribe("/dtu_controller/last_point", 1, &getlast_point );
  // set_complete_status=nh.advertise<std_msgs::UInt8>("/dtu_controller/get_status", 1 );

    // ros::Subscriber referenceSub = nh.subscribe("current_frame_goal_reference", 1, &updateReferenceCallback );
  // position_cmd_sub_ =
  //   nh.subscribe("planning/pos_cmd", 10, &control_callback,
  //               this, ros::TransportHints().tcpNoDelay());

// // 定期计算即可
//   DLQR_Control(des, odom_data, imu_data, u); 
  
  // Publish the control signal
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 100);
  ctrlPosPub=nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 100);
  ctrlVelPub=nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 100);
  // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here(flight_control_setpoint_generic), but
  // we use dji_sdk/flight_control_setpoint_generic to demonstrate how to set the flag
  // properly in function Mission::step()
  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
  
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");


  // ros::Timer loopTimer = nh.createTimer( ros::Duration(0.05),calculateDistanceCallback);
  
  // PID controller for roll (y position)
  rollPIDMatlab.set_params(
      0.214806178818882,
      0.00446243743880562,
      0.22259440859191,
      21.0028858304873,
      0.978065112623045,
      0.00437223722492023
  );
  rollPIDMatlab.reset();
  rollPIDMatlab.use_int = 1;//启用I积分

  // PID controller for roll (x position)
  pitchPIDMatlab.set_params(
      0.214806178818882,
      0.00446243743880562,
      0.22259440859191,
      21.0028858304873,
      0.978065112623045,
      0.00437223722492023
  );
  pitchPIDMatlab.reset();
  pitchPIDMatlab.use_int = 1;//启用I积分

 
  bool obtain_control_result = obtain_control();
  bool takeoff_result;
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
    // square_mission.start_gps_location = current_gps;
    // square_mission.start_local_position = current_local_pos;
    // square_mission.setTarget(0, 20, 3, 60);
    // square_mission.state = 1;
    ROS_INFO("##### controlStatus  is  Stoping ....");
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

// void Mission::step()
// {
//   static int info_counter = 0;
//   geometry_msgs::Vector3     localOffset;

//   float speedFactor         = 2;
//   float yawThresholdInDeg   = 2;

//   float xCmd, yCmd, zCmd;

//   localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

//   double xOffsetRemaining = target_offset_x - localOffset.x;
//   double yOffsetRemaining = target_offset_y - localOffset.y;
//   double zOffsetRemaining = target_offset_z - localOffset.z;

//   double yawDesiredRad     = deg2rad * target_yaw;
//   double yawThresholdInRad = deg2rad * yawThresholdInDeg;
//   double yawInRad          = toEulerAngle(current_atti).z;

//   info_counter++;
//   if(info_counter > 25)
//   {
//     info_counter = 0;
//     ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x,localOffset.y, localOffset.z,yawInRad);
//     ROS_INFO("+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining,yOffsetRemaining, zOffsetRemaining,yawInRad - yawDesiredRad);
//   }
//   if (abs(xOffsetRemaining) >= speedFactor)
//     xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
//   else
//     xCmd = xOffsetRemaining;

//   if (abs(yOffsetRemaining) >= speedFactor)
//     yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
//   else
//     yCmd = yOffsetRemaining;

//   zCmd = start_local_position.z + target_offset_z;


//   /*!
//    * @brief: if we already started breaking, keep break for 50 sample (1sec)
//    *         and call it done, else we send normal command
//    */

//   if (break_counter > 50)
//   {
//     ROS_INFO("##### Route %d finished....", state);
//     finished = true;
//     return;
//   }
//   else if(break_counter > 0)
//   {
//     sensor_msgs::Joy controlVelYawRate;
//     uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
//                 DJISDK::HORIZONTAL_VELOCITY |
//                 DJISDK::YAW_RATE            |
//                 DJISDK::HORIZONTAL_GROUND   |
//                 DJISDK::STABLE_ENABLE);
//     controlVelYawRate.axes.push_back(0);
//     controlVelYawRate.axes.push_back(0);
//     controlVelYawRate.axes.push_back(0);
//     controlVelYawRate.axes.push_back(0);
//     controlVelYawRate.axes.push_back(flag);

//     ctrlBrakePub.publish(controlVelYawRate);
//     break_counter++;
//     return;
//   }
//   else //break_counter = 0, not in break stage
//   {
//     sensor_msgs::Joy controlPosYaw;


//     controlPosYaw.axes.push_back(xCmd);
//     controlPosYaw.axes.push_back(yCmd);
//     controlPosYaw.axes.push_back(zCmd);
//     controlPosYaw.axes.push_back(yawDesiredRad);
//     ctrlPosYawPub.publish(controlPosYaw);
//   }

//   if (std::abs(xOffsetRemaining) < 0.5 &&
//       std::abs(yOffsetRemaining) < 0.5 &&
//       std::abs(zOffsetRemaining) < 0.5 &&
//       std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
//   {
//     //! 1. We are within bounds; start incrementing our in-bound counter
//     inbound_counter ++;
//   }
//   else
//   {
//     if (inbound_counter != 0)
//     {
//       //! 2. Start incrementing an out-of-bounds counter
//       outbound_counter ++;
//     }
//   }

//   //! 3. Reset withinBoundsCounter if necessary
//   if (outbound_counter > 10)
//   {
//     ROS_INFO("##### Route %d: out of bounds, reset....", state);
//     inbound_counter  = 0;
//     outbound_counter = 0;
//   }

//   if (inbound_counter > 50)
//   {
//     ROS_INFO("##### Route %d start break....", state);
//     break_counter = 1;
//   }

// }

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
  // current_atti = msg->quaternion;

  tf::Quaternion current_target_quat(attitude_quaternion_msg->quaternion.x, attitude_quaternion_msg->quaternion.y, attitude_quaternion_msg->quaternion.z, attitude_quaternion_msg->quaternion.w);
  // target_pose.orientation.w = attitude_quaternion_msg->quaternion.w; // -
  // target_pose.orientation.x = attitude_quaternion_msg->quaternion.x; // -
  // target_pose.orientation.y = attitude_quaternion_msg->quaternion.y;
  // target_pose.orientation.z = attitude_quaternion_msg->quaternion.z;
  
  geometry_msgs::Vector3 current_attitude;
  tf::Matrix3x3 R_FLU2ENU(current_target_quat);
  R_FLU2ENU.getRPY(current_attitude.x, current_attitude.y, current_attitude.z);

  tf::Matrix3x3 R_B_W(current_target_quat);
  R_B_W.getRPY(current_attitude.x, current_attitude.y, current_attitude.z);

  if( (current_attitude.z > -M_PI) && (current_attitude.z < -M_PI_2 ) ) current_attitude.z += M_PI_2 + M_PI;
  else current_attitude.z -= M_PI_2;

  current_target_quat.setRPY(current_attitude.x, current_attitude.y, current_attitude.z);

  current_atti.w = current_target_quat.getW(); // -
  current_atti.x = current_target_quat.getX(); // -
  current_atti.y = current_target_quat.getY();
  current_atti.z = current_target_quat.getZ();

  // ROS_INFO("Quat XYZW = %.2f   %.2f   %.2f   %.2f", attitude_quaternion_msg->quaternion.x, attitude_quaternion_msg->quaternion.y, attitude_quaternion_msg->quaternion.z, attitude_quaternion_msg->quaternion.w);

  tf::Quaternion attitude;
  attitude.setW(attitude_quaternion_msg->quaternion.w);
  attitude.setX(attitude_quaternion_msg->quaternion.x);
  attitude.setY(attitude_quaternion_msg->quaternion.y);
  attitude.setZ(attitude_quaternion_msg->quaternion.z);

  double roll1, pitch1, yaw1;
  tf::Matrix3x3(attitude).getRPY(roll1, pitch1, yaw1);

  current_angular.x = roll1;
  current_angular.y = pitch1;
  current_angular.z = yaw1;

    tf::Vector3 w_BW_B(current_imu.x, current_imu.y, current_imu.z);
  tf::Vector3 w_BW_W = R_B_W * w_BW_B;
  current_yawRate = getYawRate(current_attitude, w_BW_W);
  // _contactYawRate = getYawRate(_contactAttitude, w_BW_W);

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
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  current_gps = *msg;

  // Down sampled to 50Hz loop
  // if(elapsed_time > ros::Duration(0.02))
  // {
  //   start_time = ros::Time::now();
    // geometry_msgs::Quaternion quat;
    // geometry_msgs::Vector3 a;
    // sensor_msgs::Joy controlPosYaw;

    // quat.x=command.qx;
    // quat.y=command.qy;
    // quat.z=command.qz;
    // quat.w=command.qw;
    // a=toEulerAngle(quat);
    
    // controlPosYaw.axes.push_back(a.x);
    // controlPosYaw.axes.push_back(a.y);
    // controlPosYaw.axes.push_back(a.z);
    // controlPosYaw.axes.push_back(0);
    // ctrlPosYawPub.publish(controlPosYaw);
  //   switch(square_mission.state)
  //   {
  //     case 0:
  //       break;

  //     case 1:
  //       if(!square_mission.finished)
  //       {
  //         square_mission.step();
  //       }
  //       else
  //       {
  //         square_mission.reset();
  //         square_mission.start_gps_location = current_gps;
  //         square_mission.start_local_position = current_local_pos;
  //         square_mission.setTarget(20, 0, 0, 0);
  //         square_mission.state = 2;
  //         ROS_INFO("##### Start route %d ....", square_mission.state);
  //       }
  //       break;

  //     case 2:
  //       if(!square_mission.finished)
  //       {
  //         square_mission.step();
  //       }
  //       else
  //       {
  //         square_mission.reset();
  //         square_mission.start_gps_location = current_gps;
  //         square_mission.start_local_position = current_local_pos;
  //         square_mission.setTarget(0, -20, 0, 0);
  //         square_mission.state = 3;
  //         ROS_INFO("##### Start route %d ....", square_mission.state);
  //       }
  //       break;
  //     case 3:
  //       if(!square_mission.finished)
  //       {
  //         square_mission.step();
  //       }
  //       else
  //       {
  //         square_mission.reset();
  //         square_mission.start_gps_location = current_gps;
  //         square_mission.start_local_position = current_local_pos;
  //         square_mission.setTarget(-20, 0, 0, 0);
  //         square_mission.state = 4;
  //         ROS_INFO("##### Start route %d ....", square_mission.state);
  //       }
  //       break;
  //     case 4:
  //       if(!square_mission.finished)
  //       {
  //         square_mission.step();
  //       }
  //       else
  //       {
  //         ROS_INFO("##### Mission %d Finished ....", square_mission.state);
  //         square_mission.state = 0;
  //       }
  //       break;
  //   }
  // }
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
