/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

// #include <mavros_msgs/AttitudeTarget.h>
#include <queue>
#include<ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>
//#include "ROScallback.h"
#include <Eigen/Dense>

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

};

double fromQuaternion2yaw(Eigen::Quaterniond q) {

    double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
    return yaw;
}
class DLQR
{
public:
      Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd N;
    Eigen::MatrixXd R;
    Eigen::MatrixXd K;
    double gra=9.81;
        double dt;

    DLQR()
    {
    double qvalue=2.0;
    double rvalue=0.5;
    ros::param::get("qvalue",qvalue);
    ros::param::get("rvalue",rvalue);
    ROS_INFO("qvalue is %.4f,rvalue is %.4f",qvalue,rvalue);
        dt = 1.0 / (double)50.0;//50HZ freq control
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
    Q = qvalue * Eigen::MatrixXd::Identity(4,4);
	R = rvalue* Eigen::MatrixXd::Identity(2,2);
    N= Eigen::MatrixXd::Zero(4, 2);
    DARE(A, B, Q, R, N, &K,1e-15);
 
    }
/**
 * https://github.com/schlagenhauf/lqr_solve/blob/master/lqr_solve.cpp
 * @brief Computes the LQR gain matrix (usually denoted K) for a discrete time
 * infinite horizon problem.
 *
 * @param A State matrix of the underlying system
 * @param B Input matrix of the underlying system
 * @param Q Weight matrix penalizing the state
 * @param R Weight matrix penalizing the controls
 * @param N Weight matrix penalizing state / control pairs
 * @param K Pointer to the generated matrix (has to be a double/dynamic size
 * matrix!)
 * @param eps Delta between iterations that determines when convergence is
 * reached
 */

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


//    const Odom_Data_t &odom,    const Imu_Data_t &imu, 
void DLQR_Control(const Desired_State_t &des,
geometry_msgs::Point& current_local_pos,geometry_msgs::Vector3Stamped& current_vel,
geometry_msgs::Quaternion current_imu,double& roll,double& pitch) 
{

    Eigen::Vector3d des_acc(0.0, 0.0, 0.0);

    // std::cout << K << std::endl;

    const Eigen::Vector4d state_des = Eigen::Vector4d(des.p(0), des.p(1), des.v(0), des.v(1));
    const Eigen::Vector4d state_now = Eigen::Vector4d(current_local_pos.x, current_local_pos.y, current_vel.vector.x, current_vel.vector.y);
    const Eigen::Vector2d des_a = Eigen::Vector2d(des.a(0), des.a(1));
    Eigen::Vector2d out_acc = -K * (  state_now-state_des) + des_a;

    des_acc(0) = out_acc(0);
    des_acc(1) =out_acc(1);
    des_acc += Eigen::Vector3d(0,0,gra);

    Eigen::Quaterniond cur_imu(current_imu.w,current_imu.x,current_imu.y,current_imu.z);//imu.q  ---- FLU2ENU
    //可能imu中的四元数还要进行坐标转换，转换到大地坐标系下。
    double yaw_odom = fromQuaternion2yaw(cur_imu);
    double sin = std::sin(yaw_odom);
    double cos = std::cos(yaw_odom);
    roll = (des_acc(0) * sin - des_acc(1) * cos )/gra;
    pitch = (des_acc(0) * cos + des_acc(1) * sin )/ gra;
    
    // Eigen::Quaterniond cur_imu(current_imu.w,current_imu.x,current_imu.y,current_imu.z);//imu.q
    // yaw_imu = fromQuaternion2yaw(cur_imu);//根据四元数计算出欧拉角，ros的odom消息机制为ZYX，无人机的是ZXY
    // // Eigen::Quaterniond q = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())
    // //   * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
    // //   * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());
    // Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw,Eigen::Vector3d::UnitZ())
    // * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
    // * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
    // u.q = cur_imu* cur_q.inverse() * q;// Align with FCU frame

}
};


#endif