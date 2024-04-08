#include <mav_trajectory_generation_example/example_planner.h>
#include <mav_visualization/hexacopter_marker.h>

#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <cmath>
#include "mav_trajectory_generation_ros/feasibility_analytic.h"
#include "mav_trajectory_generation_ros/feasibility_recursive.h"
#include "mav_trajectory_generation_ros/feasibility_sampling.h"
#include "mav_trajectory_generation_ros/feasibility_base.h"

using namespace mav_trajectory_generation;
ExamplePlanner::ExamplePlanner(ros::NodeHandle& nh)
    : nh_(nh),
      max_v_(2.0),
      max_a_(2.0),
      max_ang_v_(2.0),
      max_ang_a_(2.0),
      current_velocity_(Eigen::Vector3d::Zero()),
      current_angular_velocity_(Eigen::Vector3d::Zero()),
      current_pose_(Eigen::Affine3d::Identity()) {
        
  // Load params
  if (!nh_.getParam(ros::this_node::getName() + "/max_v", max_v_)){
    ROS_WARN("[example_planner] param max_v not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_a", max_a_)){
    ROS_WARN("[example_planner] param max_a not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_ang_v", max_ang_v_)){
    ROS_WARN("[example_planner] param max_ang_v not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_ang_a", max_ang_a_)){
    ROS_WARN("[example_planner] param max_ang_a not found");
  }
        
  // create publisher for RVIZ markers
  pub_markers_ =
      nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);

  pub_trajectory_ =
      nh.advertise<mav_planning_msgs::PolynomialTrajectory>("trajectory",
                                                              0);

  // subscriber for Odometry
  sub_odom_ =
      nh.subscribe("uav_pose", 1, &ExamplePlanner::uavOdomCallback, this);
}

// Callback to get current Pose of UAV
void ExamplePlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

  // store current position in our planner
  tf::poseMsgToEigen(odom->pose.pose, current_pose_);

  // store current velocity
  tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
  tf::vectorMsgToEigen(odom->twist.twist.angular, current_angular_velocity_);
}

// Method to set maximum speed.
void ExamplePlanner::setMaxSpeed(const double max_v) {
  max_v_ = max_v;
}

// Plans a trajectory from the current position to the a goal position and velocity
bool ExamplePlanner::planTrajectory(
    const Eigen::VectorXd& goal_pos, const Eigen::VectorXd& goal_vel,
    mav_trajectory_generation::Trajectory* trajectory) {
  assert(trajectory);
  trajectory->clear();

  // 3 Dimensional trajectory => 3D position
  // 4 Dimensional trajectory => 3D position + yaw
  // 6 Dimensional trajectory => through SE(3) space, position and orientation
  const int dimension = goal_pos.size();
  bool success = false;

  if (dimension == 6) 
  {
    mav_trajectory_generation::Trajectory trajectory_trans, trajectory_rot;

    // Translation trajectory.
    Eigen::Vector3d goal_position = goal_pos.head(3);
    Eigen::Vector3d goal_lin_vel = goal_vel.head(3);
    success = planTrajectory(
        goal_position, goal_lin_vel, current_pose_.translation(),
        current_velocity_, max_v_, max_a_, &trajectory_trans);

    // Rotation trajectory.
    Eigen::Vector3d goal_rotation = goal_pos.tail(3);
    Eigen::Vector3d goal_ang_vel = goal_vel.tail(3);
    Eigen::Vector3d current_rot_vec;
    mav_msgs::vectorFromRotationMatrix(
        current_pose_.rotation(), &current_rot_vec);
    success &= planTrajectory(
        goal_rotation, goal_ang_vel, current_rot_vec, current_angular_velocity_,
        max_ang_v_, max_ang_a_, &trajectory_rot);

    // Combine trajectories.
    success &= trajectory_trans.getTrajectoryWithAppendedDimension(
            trajectory_rot, &(*trajectory));
    return success;
  } 
  else if (dimension == 3) 
  {
    success = planTrajectory(
        goal_pos, goal_vel, current_pose_.translation(), current_velocity_,
        max_v_, max_a_, &(*trajectory));
    return success;
  } 
  else if (dimension == 4) 
  {
    Eigen::Vector4d start_pos_4d, start_vel_4d;
    start_pos_4d << current_pose_.translation(),
        mav_msgs::yawFromQuaternion(
            (Eigen::Quaterniond)current_pose_.rotation());
    start_vel_4d << current_velocity_, 0.0;
    success = planTrajectory(
        goal_pos, goal_vel, start_pos_4d, start_vel_4d, max_v_, max_a_,
        &(*trajectory));
    return success;
  } 
  else 
  {
    LOG(WARNING) << "Dimension must be 3, 4 or 6 to be valid.";
    return false;
  }
}

//---------------------------test feasibility-----------------------------------
void writeResultsToFile(const std::string& file_name,
                        const std::vector<InputFeasibilityResult>& results) {
  std::ofstream file;
  file.open(file_name, std::ofstream::out | std::ofstream::trunc);
  if (file.is_open()) {
    for (const InputFeasibilityResult& result : results) {
      file << std::to_string(result) + "\n";
    }
    file.close();
  } else {
    std::cout << "Unable to open file " << file_name << "." << std::endl;
  }
}
//TEST(FeasibilityTest, CompareFeasibilityTests) 
int CompareFeasibilityTests(const Trajectory* mytrajectory){
  Segment::Vector segments(37, Segment(10, 6));
      //------------------------------处理自己的轨迹-----------------------------------------
    mytrajectory->getSegments(&segments);
    int kNumSegments=segments.size();
    for (size_t i = 0; i < kNumSegments; i++) {
      segments[i]=mytrajectory->segments()[i];
      
    }

    ROS_INFO("segments size is %d",segments.size());
    //--------------------------------处理完------------------------------

  // Set regular input constraints.
  InputConstraints input_constraints;
  input_constraints.setDefaultValues();

  // Create feasibility checks with different user settings.
  FeasibilitySampling feasibility_sampling_01(input_constraints);
  feasibility_sampling_01.settings_.setSamplingIntervalS(0.01);
  FeasibilitySampling feasibility_sampling_05(input_constraints);
  feasibility_sampling_05.settings_.setSamplingIntervalS(0.05);
  FeasibilitySampling feasibility_sampling_10(input_constraints);
  feasibility_sampling_10.settings_.setSamplingIntervalS(0.10);

  FeasibilityRecursive feasibility_recursive_01(input_constraints);
  feasibility_recursive_01.settings_.setMinSectionTimeS(0.01);
  FeasibilityRecursive feasibility_recursive_05(input_constraints);
  feasibility_recursive_05.settings_.setMinSectionTimeS(0.05);
  FeasibilityRecursive feasibility_recursive_10(input_constraints);
  feasibility_recursive_10.settings_.setMinSectionTimeS(0.10);

  FeasibilityAnalytic feasibility_analytic_01(input_constraints);
  feasibility_analytic_01.settings_.setMinSectionTimeS(0.01);
  FeasibilityAnalytic feasibility_analytic_05(input_constraints);
  feasibility_analytic_05.settings_.setMinSectionTimeS(0.05);
  FeasibilityAnalytic feasibility_analytic_10(input_constraints);
  feasibility_analytic_10.settings_.setMinSectionTimeS(0.10);

  // Some statistics.
  std::vector<InputFeasibilityResult> result_sampling_01(kNumSegments);
  std::vector<InputFeasibilityResult> result_sampling_05(kNumSegments);
  std::vector<InputFeasibilityResult> result_sampling_10(kNumSegments);

  std::vector<InputFeasibilityResult> result_recursive_01(kNumSegments);
  std::vector<InputFeasibilityResult> result_recursive_05(kNumSegments);
  std::vector<InputFeasibilityResult> result_recursive_10(kNumSegments);

  std::vector<InputFeasibilityResult> result_analytic_01(kNumSegments);
  std::vector<InputFeasibilityResult> result_analytic_05(kNumSegments);
  std::vector<InputFeasibilityResult> result_analytic_10(kNumSegments);

  timing::Timer time_sampling_01("time_sampling_01", false);
  timing::Timer time_sampling_05("time_sampling_05", false);
  timing::Timer time_sampling_10("time_sampling_10", false);

  timing::Timer time_recursive_01("time_recursive_01", false);
  timing::Timer time_recursive_05("time_recursive_05", false);
  timing::Timer time_recursive_10("time_recursive_10", false);

  timing::Timer time_analytic_01("time_analytic_01", false);
  timing::Timer time_analytic_05("time_analytic_05", false);
  timing::Timer time_analytic_10("time_analytic_10", false);

  for (size_t i = 0; i < segments.size() ; i++) {
    // Sampling.
    time_sampling_01.Start();
    result_sampling_01[i] =
        feasibility_sampling_01.checkInputFeasibility(segments[i]);
    time_sampling_01.Stop();

    time_sampling_05.Start();
    result_sampling_05[i] =
        feasibility_sampling_05.checkInputFeasibility(segments[i]);
    time_sampling_05.Stop();

    time_sampling_10.Start();
    result_sampling_10[i] =
        feasibility_sampling_10.checkInputFeasibility(segments[i]);
    time_sampling_10.Stop();

    // Recursive.
    time_recursive_01.Start();
    result_recursive_01[i] =
        feasibility_recursive_01.checkInputFeasibility(segments[i]);
    time_recursive_01.Stop();

    time_recursive_05.Start();
    result_recursive_05[i] =
        feasibility_recursive_05.checkInputFeasibility(segments[i]);
    time_recursive_05.Stop();

    time_recursive_10.Start();
    result_recursive_10[i] =
        feasibility_recursive_10.checkInputFeasibility(segments[i]);
    time_recursive_10.Stop();

    // Analtic.
    time_analytic_01.Start();
    result_analytic_01[i] =
        feasibility_analytic_01.checkInputFeasibility(segments[i]);
    time_analytic_01.Stop();

    time_analytic_05.Start();
    result_analytic_05[i] =
        feasibility_analytic_05.checkInputFeasibility(segments[i]);
    time_analytic_05.Stop();

    time_analytic_10.Start();
    result_analytic_10[i] =
        feasibility_analytic_10.checkInputFeasibility(segments[i]);
    time_analytic_10.Stop();
ROS_INFO("feasibility is %d",result_recursive_01[i]);
  }

  // Write results to txt.
  writeResultsToFile("result_sampling_01.txt", result_sampling_01);
  writeResultsToFile("result_sampling_05.txt", result_sampling_05);
  writeResultsToFile("result_sampling_10.txt", result_sampling_10);
  writeResultsToFile("result_recursive_01.txt", result_recursive_01);
  writeResultsToFile("result_recursive_05.txt", result_recursive_05);
  writeResultsToFile("result_recursive_10.txt", result_recursive_10);
  writeResultsToFile("result_analytic_01.txt", result_recursive_01);
  writeResultsToFile("result_analytic_05.txt", result_recursive_05);
  writeResultsToFile("result_analytic_10.txt", result_recursive_10);

  // Write timing to txt.
  std::ofstream time_file;
  std::string time_file_name("feasibility_times");
  time_file.open("feasibility_times.txt",
                 std::ofstream::out | std::ofstream::trunc);
  if (time_file.is_open()) {
    time_file << timing::Timing::Print() + "\n";
    time_file.close();
  } else {
    std::cout << "Unable to open file " << time_file_name << "." << std::endl;
  }
  ROS_INFO("test feas node writed down-------");
}

//TEST(FeasibilityTest, HalfPlaneFeasibility) 
int HalfPlaneFeasibility() {
  FeasibilityBase half_space_check;
  Eigen::VectorXd coeffs_x(Eigen::Vector3d::Zero());
  Eigen::VectorXd coeffs_y(Eigen::Vector3d::Zero());
  Eigen::VectorXd coeffs_z(Eigen::Vector3d::Zero());
  // Parabola.
  coeffs_x(1) = 1;
  coeffs_z(2) = 1;

  Segment segment(3, 3);
  segment[0] = Polynomial(coeffs_x);
  segment[1] = Polynomial(coeffs_y);
  segment[2] = Polynomial(coeffs_z);
  segment.setTime(1.0);

  Eigen::Vector3d point(0.0, 0.0, 0.0);
  Eigen::Vector3d normal(-1.0, 0.0, 1.0);
  // Shift boundary down.
  while (point.z() > -1.0) {
    half_space_check.half_plane_constraints_.emplace_back(point, normal);
    bool feasible = half_space_check.checkHalfPlaneFeasibility(segment);
    if (point.z() >= -0.25) {
      // EXPECT_FALSE(feasible) << point.transpose();
    } else {
      // EXPECT_TRUE(feasible) << point.transpose();;
    }
    half_space_check.half_plane_constraints_.clear();
    point.z() -= 0.05;
  }

  FeasibilityBase box_check;
  Eigen::Vector3d box_center(0.0, 0.0, 0.0);

  // Grow box.
  double l = 0.0;
  while(l < 4.0) {
    Eigen::Vector3d box_size(Eigen::Vector3d::Constant(l));
    box_check.half_plane_constraints_ = HalfPlane::createBoundingBox(box_center, box_size);
    bool feasible = box_check.checkHalfPlaneFeasibility(segment);
    if(l <= 2.0) {
      // EXPECT_FALSE(feasible);
    }
    else {
      // EXPECT_TRUE(feasible);
    }
    l += 0.05;
  }
}
//----------------------------test finish-----------------------------------------
// Plans a trajectory from a start position and velocity to a goal position and velocity
bool ExamplePlanner::planTrajectory(const Eigen::VectorXd& goal_pos,
                                    const Eigen::VectorXd& goal_vel,
                                    const Eigen::VectorXd& start_pos,
                                    const Eigen::VectorXd& start_vel,
                                    double v_max, double a_max,
                                    mav_trajectory_generation::Trajectory* trajectory) {
  assert(trajectory);
  const int dimension = goal_pos.size();
  // Array for all waypoints and their constraints
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimze up to 4th order derivative (SNAP)
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::SNAP;

  // we have 2 vertices:
  // start = desired start vector
  // end = desired end vector
  mav_trajectory_generation::Vertex start(dimension), end(dimension),middle(dimension);

  /******* Configure start point *******/
  start.makeStartOrEnd(start_pos, derivative_to_optimize);
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      start_vel);
  vertices.push_back(start);
  double begin_t = ros::Time::now().toSec();
  double t = 0;
  double angle = 0;
  Eigen::Vector3d desired_position(0.0, 0.0,1.2);
  double desired_yaw = 0.0;

  double radius=5.0;
  double speed=1.0;
  double sampling_frequency=10;
double omega = fabs(speed / radius);
double angle_step = fabs(omega / sampling_frequency);

  double x=0;
  double y=0;
  // const float gh[37][2]={{3.1147,-2.7280},{1.7103,1.9594},{6.8969,-5.9907},{-4.6354,-1.1727},{-4.3903,1.2768},{-1.8848,1.3558},{-1.5149,-1.7553},{1.3963,-1.6052},{-1.1497,-0.5307},{-0.4204,-0.6146},{0.1980,-1.4452},{0.6225,-0.4964},{0.9417,-0.0921},{0.7014,0.3674},{0.4272,0.6326},{0.0769,0.7363},{5.4777,-1.3707},{5.1236,1.0470},{1.8386,1.1371},{1.8889,-1.6148},{2.3898,2.8985},{-1.6955,1.5947},{1.5207,4.7301},{-1.1182,5.0345},{-3.2949,3.6038},{-5.7683,1.5824},{-0.5704,-0.0049},{4.7090,-3.1625},{2.1907,-4.3879},{0.0315,-3.7493},{-0.5479,-1.2541},{-7.0252,6.0559},{2.3652,2.6730},{-1.6841,1.4268},{-2.3595,-0.6330},{-0.6963,0.0407},{-4.8658,-5.5447}};
  for(int i =1;i<20;i++){
    angle = fmod(2 * M_PI / 20 *i , 2 * M_PI); 
    const double cos_phi = cos(angle);
    const double sin_phi = sin(angle);
    x=radius*cos_phi-radius;
    y=radius*sin_phi;
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(x,y,1.2));
    // middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,radius*omega*  Eigen::Vector3d(-sin_phi, cos_phi ,0.0));
    // middle.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,radius*omega*omega*  Eigen::Vector3d(-cos_phi, -sin_phi ,0.0));
    vertices.push_back(middle);
  }

    // point.acceleration =
    //     radius * pow(omega, 2.0) * Eigen::Vector3d(-cos_phi, -sin_phi, 0.0);
    // point.jerk =
    //     radius * pow(omega, 3.0) * Eigen::Vector3d(sin_phi, -cos_phi, 0.0);
    // point.snap =
    //     radius * pow(omega, 4.0) * Eigen::Vector3d(cos_phi, sin_phi, 0.0);

    // trajectory.points.push_back(point);

  // Eigen::Vector3d position,rotation_vec;
  // Eigen::VectorXd pose1;
  // position << x, y, 2.0;
  //   Eigen::Matrix3d rotation_mat;
  // rotation_mat = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) 
  //             * Eigen::AngleAxisd(M_PI / 2.0,  Eigen::Vector3d::UnitY())
  //             * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());
  // mav_msgs::vectorFromRotationMatrix(rotation_mat, &rotation_vec);
  // pose1 << position, rotation_vec;
  /******* Configure end point *******/
  // set end point constraints to desired position and set all derivatives to zero
  end.makeStartOrEnd(goal_pos, derivative_to_optimize);
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                   goal_vel);
  // end.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
  //     radius*omega*  Eigen::Vector3d(-cos(angle),-sin(angle) ,0.0));
vertices.push_back(end);

  // setimate initial segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, v_max, a_max);

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // set up optimization problem
  const int N = 12;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  // constrain velocity and acceleration
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);

  opt.solveLinear();
  // solve trajectory
  opt.optimize();

  // get trajectory as polynomial parameters
  opt.getTrajectory(&(*trajectory));
  // trajectory->scaleSegmentTimesToMeetConstraints(v_max, a_max);
  
  // CompareFeasibilityTests(trajectory);
  return true;
}

bool ExamplePlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::MarkerArray markers;
  double distance =
      0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string frame_id = "world";

  mav_trajectory_generation::drawMavTrajectory(trajectory,
                                               distance,
                                               frame_id,
                                               &markers);
  pub_markers_.publish(markers);

  // send trajectory to be executed on UAV
  mav_planning_msgs::PolynomialTrajectory msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
                                                                 &msg);
  msg.header.frame_id = "world";
  pub_trajectory_.publish(msg);

  return true;
}



