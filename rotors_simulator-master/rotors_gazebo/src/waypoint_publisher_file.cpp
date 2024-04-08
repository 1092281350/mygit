/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fstream>
#include <iostream>

#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>


bool sim_running = false;

static const int64_t kNanoSecondsInSecond = 1000000000;

void callback(const sensor_msgs::ImuPtr& msg) {
  sim_running = true;
}

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0), yaw(0.0) {
  }

  WaypointWithTime(double t, float x, float y, float z, float _yaw)
      : position(x, y, z), yaw(_yaw), waiting_time(t) {
  }

  Eigen::Vector3d position;
  double yaw;
  double waiting_time;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_publisher");
  ros::NodeHandle nh;

  ROS_INFO("Started waypoint_publisher.");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  // if (args.size() != 2 && args.size() != 3) {
  //   ROS_ERROR("Usage: waypoint_publisher <waypoint_file>"
  //       "\nThe waypoint file should be structured as: space separated: wait_time [s] x[m] y[m] z[m] yaw[deg])");
  //   return -1;
  // }

  std::vector<WaypointWithTime> waypoints;
  const float DEG_2_RAD = M_PI / 180.0;

  // std::ifstream wp_file(args.at(1).c_str());

  // if (wp_file.is_open()) {
    double x, y, z, yaw;
    int t=0;
    z=2.0;
    yaw=0;
     const float gh[37][2]={{3.1147,-2.7280},{1.7103,1.9594},{6.8969,-5.9907},{-4.6354,-1.1727},{-4.3903,1.2768},{-1.8848,1.3558},{-1.5149,-1.7553},{1.3963,-1.6052},{-1.1497,-0.5307},{-0.4204,-0.6146},{0.1980,-1.4452},{0.6225,-0.4964},{0.9417,-0.0921},{0.7014,0.3674},{0.4272,0.6326},{0.0769,0.7363},{5.4777,-1.3707},{5.1236,1.0470},{1.8386,1.1371},{1.8889,-1.6148},{2.3898,2.8985},{-1.6955,1.5947},{1.5207,4.7301},{-1.1182,5.0345},{-3.2949,3.6038},{-5.7683,1.5824},{-0.5704,-0.0049},{4.7090,-3.1625},{2.1907,-4.3879},{0.0315,-3.7493},{-0.5479,-1.2541},{-7.0252,6.0559},{2.3652,2.6730},{-1.6841,1.4268},{-2.3595,-0.6330},{-0.6963,0.0407},{-4.8658,-5.5447}};
    // Only read complete waypoints.

    while (t<36) {
      x+=gh[t][0];
      y+=gh[t][1];
      waypoints.push_back(WaypointWithTime(0.0, x, y, z, yaw * DEG_2_RAD));
      t++;
    }
  //   wp_file.close();
  //   ROS_INFO("Read %d waypoints.", (int) waypoints.size());
  // } else {
  //   ROS_ERROR_STREAM("Unable to open poses file: " << args.at(1));
  //   return -1;
  // }

  // The IMU is used, to determine if the simulator is running or not.
  ros::Subscriber sub = nh.subscribe("imu", 10, &callback);

  ros::Publisher wp_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ROS_INFO("Wait for simulation to become ready...");

  while (!sim_running && ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("...ok");

  // Wait for 30s such that everything can settle and the mav flies to the initial position.
  // ros::Duration(30).sleep();

  ROS_INFO("Start publishing waypoints.");

  trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
  msg->header.stamp = ros::Time::now();
  msg->points.resize(waypoints.size());
  msg->joint_names.push_back("base_link");
  int64_t time_from_start_ns = 0;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    WaypointWithTime& wp = waypoints[i];

    mav_msgs::EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = wp.position;
    trajectory_point.setFromYaw(wp.yaw);
    trajectory_point.time_from_start_ns = time_from_start_ns;

    time_from_start_ns += static_cast<int64_t>(wp.waiting_time * kNanoSecondsInSecond);

    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
  }
  wp_pub.publish(msg);

  ros::spinOnce();
  ros::shutdown();

  return 0;
}
