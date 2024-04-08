//  run_circle_locus.cpp
// 飞一个圆形轨迹

#include <thread>
#include <chrono>
#include <cmath>
#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

ros::Publisher pathPub;
geometry_msgs::PoseStamped poseROS;
nav_msgs::Path             mypath;
const float gh[25][2]={{-4.6354,-1.1727},{-4.3903,1.2768},{-1.8848,1.3558},{-1.1497,-0.5307},{-0.4204,-0.6146},{0.1980,-1.4452},{0.6225,-0.4964},{0.9417,-0.0921},{0.7014,0.3674},{0.4272,0.6326},{0.0769,0.7363},{5.4777,-1.3707},{5.1236,1.0470},{1.8386,1.1371},{1.5207,4.7301},{-1.1182,5.0345},{-3.2949,3.6038},{-5.7683,1.5824},{-0.5704,-0.0049},{4.7090,-3.1625},{2.1907,-4.3879},{0.0315,-3.7493},{-0.5479,-1.2541},{-2.3595,-0.6330},{-0.6963,0.0407}};

int main(int argc, char** argv) {
    ros::init(argc, argv, "circle_locus_example");
    ros::NodeHandle nh;
    // Create a private node handle for accessing node parameters.
    ros::NodeHandle nh_private("~");
    ros::Publisher trajectory_pub =
        nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
            "/firefly/command/trajectory", 10);
    ROS_INFO("Started circle locus example.");
    pathPub   = nh.advertise<nav_msgs::Path>( "/circle/path", 100);
    // std_srvs::Empty srv;
    // bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    // unsigned int i = 0;
  // Path

    
    // // Trying to unpause Gazebo for 10 seconds.
    // while (i <= 10 && !unpaused) {s
    // ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    // ++i;
    // }

    // if (!unpaused) {
    // ROS_FATAL("Could not wake up Gazebo.");
    // return -1;
    // } else {
    // ROS_INFO("Unpaused the Gazebo simulation.");
    // }

    // Wait for 5 seconds to let the Gazebo GUI show up.
    ros::Duration(5.0).sleep();

    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();
    double begin_t = ros::Time::now().toSec();
    double t = 0;
    double angle = 0;
    bool comp=false;
    ros::Rate rate(10);
    // Default desired position and yaw.
    Eigen::Vector3d desired_position(0.0, 0.0, 1.2);
    double desired_yaw = 0.0;
    int i=0;
    // Overwrite defaults if set as node parameters.
    nh_private.param("x", desired_position.x(), desired_position.x());
    nh_private.param("y", desired_position.y(), desired_position.y());
    nh_private.param("z", desired_position.z(), desired_position.z());
    nh_private.param("yaw", desired_yaw, desired_yaw);

   auto  begin=ros::Time::now().toSec();

     poseROS.header.frame_id ="world";
  poseROS.pose.orientation.w = 1;
  poseROS.pose.orientation.x = 0;
  poseROS.pose.orientation.y = 0;
  poseROS.pose.orientation.z =0;    
    poseROS.pose.position.z = 1.2;

    while (ros::ok())
    {
        if(2 * M_PI / 10 * t>2 * M_PI and i >50){
        continue;    
        }
        t = ros::Time::now().toSec() - begin_t;
        angle = fmod(2 * M_PI / 10 * t , 2 * M_PI); // 10s飞一圈
        desired_position.x() = 3*cos(angle)-3; // 圆的半径是3
        desired_position.y() = 3*sin(angle);
        desired_position.z() = 1.2;
        desired_yaw = fmod(angle + M_PI/2 , 2 * M_PI);

        // desired_position.x() += gh[i][0]; // 圆的半径是1
        // desired_position.y() += gh[i][1];
        // desired_position.z() = 2;
        // desired_yaw = 0;

        //visual
    poseROS.header.seq = i;
    poseROS.header.stamp =ros::Time::now();
    poseROS.pose.position.x =  3*cos(angle)-3;
    poseROS.pose.position.y = 3*sin(angle);
     mypath.header = poseROS.header;
    mypath.poses.push_back(poseROS);
 

        trajectory_msg.header.stamp = ros::Time::now();
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
        desired_position, desired_yaw, &trajectory_msg);
        trajectory_pub.publish(trajectory_msg);
        rate.sleep();
        ros::spinOnce();

         i++;

    pathPub.publish(mypath);
    }

    ros::shutdown();

    return 0;
}
