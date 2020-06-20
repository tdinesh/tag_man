#include <tag_swarm/tag_manager.h>

// Standard C++
#include <math.h>
#include <string>

// ROS Related
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>

// quadrotor_control
#include <kr_tracker_msgs/Transition.h>

// Strings
static const std::string line_tracker_min_jerk("std_trackers/LineTrackerMinJerk");
static const std::string traj_tracker("std_trackers/TrajectoryTracker");

// Constructor (init)
TAGManager::TAGManager(std::string ns)
    : nh_(ns),
    priv_nh_("~"),
    last_odom_t_(ros::Time::now()),
    active_tracker_(""),
    line_tracker_min_jerk_client_(nh_, "trackers_manager/line_tracker_min_jerk/LineTracker", true),
    trajectory_tracker_client_(nh_, "trackers_manager/trajectory_tracker/TrajectoryTracker", true),
    global_offset_init_(false),
    tf_listener_(tf_buffer_),
    gtsam_init_(false),
    symbol_cnt_(0)
{
  //std::shared_ptr<tf2_ros::TransformListener> tf_listener_(new tf2_ros::TransformListener(tf_buffer_));

  const double server_wait_time = 0.5;
  if (!line_tracker_min_jerk_client_.waitForServer(ros::Duration(server_wait_time))) {
    ROS_ERROR("LineTrackerMinJerkAction server not found.");
  }

  if (!trajectory_tracker_client_.waitForServer(ros::Duration(server_wait_time))) {
    ROS_ERROR("TrajectoryTrackerAction server not found.");
  }

  odom_tag_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_tag", 10);
  //odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&TAGManager::odometry_cb, this, _1));

  traj_sub_ = nh_.subscribe<kr_tracker_msgs::TrajectoryTrackerGoal>("traj", 1, boost::bind(&TAGManager::traj_cb, this, _1));

  // Tag subscriber
  tag_pose_sub_ = nh_.subscribe<apriltag_msgs::ApriltagPoseStamped>("tag_poses", 1, boost::bind(&TAGManager::tag_pose_cb,this, _1));

  // cmd_vel subscriber
  cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("tag_cmd_vel", 1, boost::bind(&TAGManager::cmd_vel_cb,this, _1));
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("world_cmd_vel", 10);

  // Services
  srv_transition_ = nh_.serviceClient<kr_tracker_msgs::Transition>("trackers_manager/transition");

  priv_nh_.param("tag_filter_alpha", tag_filter_alpha_, 0.1f);
  priv_nh_.param("origin_tag_id", origin_tag_id_, 0);

  priv_nh_.param<std::string>("world_frame", world_frame_, "world");
  priv_nh_.param<std::string>("common_origin_frame", common_origin_frame_, "common_origin");

  // Tag_swarm
  //tf_drift_global_.setIdentity();
  global_offset_lin_.setZero();
  global_offset_yaw_ = 0;

  srv_goto_ = priv_nh_.advertiseService("goTo", &TAGManager::goTo_cb, this);
  srv_goto_timed_ = priv_nh_.advertiseService("goToTimed", &TAGManager::goToTimed_cb, this);

}

void TAGManager::tag_pose_cb(const apriltag_msgs::ApriltagPoseStamped::ConstPtr& msg)
{
  //Find tag with id same as origin tag
  bool found_tag = false;
  unsigned int tag_ind = 0;

  for(unsigned int i = 0; i < msg->apriltags.size(); i++)
  {
    if(origin_tag_id_ == msg->apriltags[i].id)
    {
      tag_ind = i;
      found_tag = true;
      break;
    }
  }

  if(!found_tag)
    return;

  geometry_msgs::Pose ps_cam_tag; //Tag pose in camera frame
  ps_cam_tag = msg->posearray.poses[tag_ind];
  // ROS_INFO("Tag in %s frame: lin = (%2.2f, %2.2f, %2.2f)",
  //   msg->header.frame_id.c_str(),
  //   ps_cam_tag.position.x,
  //   ps_cam_tag.position.y,
  //   ps_cam_tag.position.z);

  geometry_msgs::TransformStamped tf_world_cam;
  ros::Time cur = ros::Time::now();
  try
  {
    //tf_world_cam = tf_buffer_.lookupTransform(world_frame_, msg->header.frame_id, msg->header.stamp);
    tf_world_cam = tf_buffer_.lookupTransform(world_frame_, msg->header.frame_id, ros::Time(0));

  }
  catch(tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
    return;
  }

  geometry_msgs::Pose ps_world_tag; //Tag pose in world frame
  tf2::doTransform(ps_cam_tag, ps_world_tag, tf_world_cam);

  geometry_msgs::TransformStamped tf_world_tag;

  tf_world_tag.header.stamp = msg->header.stamp;
  tf_world_tag.header.frame_id = world_frame_;
  tf_world_tag.child_frame_id = common_origin_frame_;

  tf_world_tag.transform.translation.x = ps_world_tag.position.x;
  tf_world_tag.transform.translation.y = ps_world_tag.position.y;
  tf_world_tag.transform.translation.z = ps_world_tag.position.z;
  tf_world_tag.transform.rotation = ps_world_tag.orientation;

  //send the transform
  tf_broadcaster_.sendTransform(tf_world_tag);


  // ROS_INFO("Tag in %s frame: lin = (%2.2f, %2.2f, %2.2f)",
  //   world_frame_.c_str(),
  //   ps_world_tag.position.x,
  //   ps_world_tag.position.y,
  //   ps_world_tag.position.z);

/*
  if (!global_offset_init_){
    tf_drift_global_ = tf_drift_global;
    global_offset_init_ = true;
  } else {
    tf_drift_global_.setOrigin(
      tf_drift_global_.getOrigin().lerp(tf_drift_global.getOrigin(), tag_filter_alpha_));
    tf_drift_global_.setRotation(
      tf_drift_global_.getRotation().slerp(tf_drift_global.getRotation(), tag_filter_alpha_));
  }

  // tf_drift_global_.setOrigin(tf_drift_global.getOrigin());
  // tf_drift_global_.setRotation(tf_drift_global.getRotation());

  ROS_INFO("Raw drift in global frame: lin = (%2.2f, %2.2f, %2.2f)",
      tf_drift_global.getOrigin().getX(),
      tf_drift_global.getOrigin().getY(),
      tf_drift_global.getOrigin().getZ());

  ROS_INFO("Filtered drift in global frame: lin = (%2.2f, %2.2f, %2.2f)",
      tf_drift_global_.getOrigin().getX(),
      tf_drift_global_.getOrigin().getY(),
      tf_drift_global_.getOrigin().getZ());
*/

  //Query the tf world -> hires

  //Transform pose to world frame

  //Publish transform world to origin

  do_ekf(ps_world_tag);

}

void TAGManager::do_ekf(geometry_msgs::Pose& ps_world_tag)
{

  ROS_INFO_STREAM("tag" << ps_world_tag);

  if(!gtsam_init_)
  {
    // Create the Kalman Filter initialization point
    gtsam::Point2 x_initial(ps_world_tag.position.x, ps_world_tag.position.y);
    gtsam::SharedDiagonal P_initial = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1, 0.1));

    // Create Key for initial pose
    gtsam::Symbol x0('x',symbol_cnt_);

    // Create an ExtendedKalmanFilter object
    ekf_.reset(new gtsam::ExtendedKalmanFilter<gtsam::Point2>(x0, x_initial, P_initial));

    gtsam_init_ = true;

    symbol_cnt_++;

    return;
  }


  // Now predict the state at t=1, i.e. argmax_{x1} P(x1) = P(x1|x0) P(x0)
  // In Kalman Filter notation, this is x_{t+1|t} and P_{t+1|t}
  // For the Kalman Filter, this requires a motion model, f(x_{t}) = x_{t+1|t)
  // Assuming the system is linear, this will be of the form f(x_{t}) = F*x_{t} + B*u_{t} + w
  // where F is the state transition model/matrix, B is the control input model,
  // and w is zero-mean, Gaussian white noise with covariance Q
  // Note, in some models, Q is actually derived as G*w*G^T where w models uncertainty of some
  // physical property, such as velocity or acceleration, and G is derived from physics
  //
  // For the purposes of this example, let us assume we are using a constant-position model and
  // the controls are driving the point to the right at 1 m/s. Then, F = [1 0 ; 0 1], B = [1 0 ; 0 1]
  // and u = [1 ; 0]. Let us also assume that the process noise Q = [0.1 0 ; 0 0.1].
  gtsam::Vector u = gtsam::Vector2(0.0, 0.0);
  gtsam::SharedDiagonal Q = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1, 0.1), true);

  gtsam::Symbol x0('x',symbol_cnt_ - 1);

  // This simple motion can be modeled with a BetweenFactor
  // Create Key for next pose
  gtsam::Symbol x1('x',symbol_cnt_);
  // Predict delta based on controls
  gtsam::Point2 difference(0,0);
  // Create Factor
  gtsam::BetweenFactor<gtsam::Point2> factor1(x0, x1, difference, Q);

  // Predict the new value with the EKF class
  gtsam::Point2 x1_predict = ekf_->predict(factor1);
  gtsam::traits<gtsam::Point2>::Print(x1_predict, "X1 Predict");



  // Now, a measurement, z1, has been received, and the Kalman Filter should be "Updated"/"Corrected"
  // This is equivalent to saying P(x1|z1) ~ P(z1|x1)*P(x1)
  // For the Kalman Filter, this requires a measurement model h(x_{t}) = \hat{z}_{t}
  // Assuming the system is linear, this will be of the form h(x_{t}) = H*x_{t} + v
  // where H is the observation model/matrix, and v is zero-mean, Gaussian white noise with covariance R
  //
  // For the purposes of this example, let us assume we have something like a GPS that returns
  // the current position of the robot. Then H = [1 0 ; 0 1]. Let us also assume that the measurement noise
  // R = [0.25 0 ; 0 0.25].
  gtsam::SharedDiagonal R = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.25, 0.25), true);

  // This simple measurement can be modeled with a PriorFactor
  gtsam::Point2 z1(ps_world_tag.position.x, ps_world_tag.position.y);
  gtsam::PriorFactor<gtsam::Point2> factor2(x1, z1, R);

  // Update the Kalman Filter with the measurement
  gtsam::Point2 x1_update = ekf_->update(factor2);
  gtsam::traits<gtsam::Point2>::Print(x1_update, "X1 Update");

  symbol_cnt_++;
}

bool TAGManager::goTo_cb(kr_mav_manager::Vec4::Request &req, kr_mav_manager::Vec4::Response &res)
{
  res.success = goToTimed(req.goal[0], req.goal[1], req.goal[2], req.goal[3], 0.0f, 0.0f, false, ros::Duration(0),
                               ros::Time::now());
  res.message = "Going To Timed...";
  return res.success;
}

bool TAGManager::goToTimed_cb(kr_mav_manager::GoalTimed::Request &req, kr_mav_manager::GoalTimed::Response &res)
{
  res.success = goToTimed(req.goal[0], req.goal[1], req.goal[2], req.goal[3], 0.0f, 0.0f, false, req.duration,
                               req.t_start);
  res.message = "Going To Timed...";
  return res.success;
}

void TAGManager::traj_cb(const kr_tracker_msgs::TrajectoryTrackerGoal::ConstPtr &msg)
{

  if(msg->waypoints.size() > 0 &&
     (msg->waypoint_times.size() == 0 || msg->waypoint_times.size() == msg->waypoints.size()))
  {

    geometry_msgs::TransformStamped tf_world_common;
    ros::Time cur = ros::Time::now();
    try
    {
      tf_world_common = tf_buffer_.lookupTransform(world_frame_, common_origin_frame_, ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
      return;
    }

    kr_tracker_msgs::TrajectoryTrackerGoal goal_world;

    for(unsigned int i = 0; i < msg->waypoints.size(); i++)
    {
      //convert pose in origin_frame to world_frame
      geometry_msgs::Pose ps_origin_goal = msg->waypoints[i];

      //tf2::Quaternion q_origin;
      //q_origin.setRPY(0.0, 0.0, 0.0);
      //ps_origin_goal.orientation = tf2::toMsg(q_origin);

      geometry_msgs::Pose ps_world_goal; //Tag pose in world frame
      tf2::doTransform(ps_origin_goal, ps_world_goal, tf_world_common);

      ROS_INFO("goal (%2.2f, %2.2f, %2.2f) in global frame: lin = (%2.2f, %2.2f, %2.2f) %2.2f",
          ps_origin_goal.position.x,
          ps_origin_goal.position.y,
          ps_origin_goal.position.z,
          ps_world_goal.position.x,
          ps_world_goal.position.y,
          ps_world_goal.position.z, msg->waypoint_times[i]);

      goal_world.waypoints.push_back(ps_world_goal);
      //goal_world.waypoint_times.push_back(msg->waypoint_times[i]);
    }

    trajectory_tracker_client_.sendGoal(goal_world);
    //trajectory_tracker_client_.sendGoal(goal_world, boost::bind(&TAGManager::traj_done_callback, this, _1, _2),
    //                                       ClientType::SimpleActiveCallback(), ClientType::SimpleFeedbackCallback());
    ROS_ERROR("Tracking traj");

    this->transition(traj_tracker);
    return;
  }
}

void TAGManager::traj_done_callback(const actionlib::SimpleClientGoalState& state, const kr_tracker_msgs::TrajectoryTrackerResultConstPtr& result)
{
  ROS_INFO("Traj finished with state %s after %2.2f s. and %2.2f 2.f m.", state.toString().c_str(), result->total_time, result->total_distance_travelled);
}

void TAGManager::tracker_done_callback(const actionlib::SimpleClientGoalState& state, const kr_tracker_msgs::LineTrackerResultConstPtr& result)
{
  ROS_INFO("Goal (%2.2f, %2.2f, %2.2f, %2.2f) finished with state %s after %2.2f s. and %2.2f m.", result->x, result->y, result->z, result->yaw, state.toString().c_str(), result->duration, result->length);
}

void TAGManager::odometry_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;

  vel_(0) = msg->twist.twist.linear.x;
  vel_(1) = msg->twist.twist.linear.y;
  vel_(2) = msg->twist.twist.linear.z;

  odom_q_ = Quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

  yaw_ = tf::getYaw(msg->pose.pose.orientation);
  yaw_dot_ = msg->twist.twist.angular.z;

  last_odom_t_ = ros::Time::now();

  geometry_msgs::Pose ps_world_odom; //odom in world frame
  ps_world_odom = msg->pose.pose;

  geometry_msgs::TransformStamped tf_common_world;
  ros::Time cur = ros::Time::now();
  try
  {
    tf_common_world = tf_buffer_.lookupTransform(common_origin_frame_, world_frame_, ros::Time(0));
  }
  catch(tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
    return;
  }

  geometry_msgs::Pose ps_common_odom; //odom in common frame
  tf2::doTransform(ps_world_odom, ps_common_odom, tf_common_world);

  //Publish odometry in the origin_frame
  nav_msgs::Odometry odom_tag;
  odom_tag.header = msg->header;

  //TODO populate twist correctly
  odom_tag.pose.pose = ps_common_odom,
  odom_tag_pub_.publish(odom_tag);
}

void TAGManager::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
  geometry_msgs::TransformStamped tf_world_common;
  ros::Time cur = ros::Time::now();
  try
  {
    tf_world_common = tf_buffer_.lookupTransform(world_frame_, common_origin_frame_, ros::Time(0));
  }
  catch(tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
    return;
  }

  tf::Quaternion q;
  tf::quaternionMsgToTF(tf_world_common.transform.rotation, q);
  double yaw_world = tf::getYaw(q);

  geometry_msgs::Twist goal;
  goal.linear.x = msg->linear.x * std::cos(yaw_world) - msg->linear.y * std::sin(yaw_world);
  goal.linear.y = msg->linear.x * std::sin(yaw_world) + msg->linear.y * std::cos(yaw_world);
  goal.linear.z = msg->linear.z;
  goal.angular.z = msg->angular.z;

  //ROS_WARN("yaw %g", yaw_world*180/3.142);

  cmd_vel_pub_.publish(goal);
}

bool TAGManager::goToTimed(float x, float y, float z, float yaw, float v_des, float a_des, bool relative, ros::Duration duration, ros::Time t_start)
{
  //convert pose in origin_frame to world_frame
  geometry_msgs::Pose ps_origin_goal; //goal in origin frame
  ps_origin_goal.position.x = x;
  ps_origin_goal.position.y = y;
  ps_origin_goal.position.z = z;

  tf2::Quaternion q_origin;
  q_origin.setRPY(0.0, 0.0, yaw);
  ps_origin_goal.orientation = tf2::toMsg(q_origin);

  geometry_msgs::TransformStamped tf_world_common;
  ros::Time cur = ros::Time::now();
  try
  {
    tf_world_common = tf_buffer_.lookupTransform(world_frame_, common_origin_frame_, ros::Time(0));
  }
  catch(tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
    return false;
  }

  geometry_msgs::Pose ps_world_goal; //Tag pose in world frame
  tf2::doTransform(ps_origin_goal, ps_world_goal, tf_world_common);

  //tf2::Quaternion q;
  //tf2::fromMsg(ps_world_goal.orientation, q);
  tf::Quaternion q;
  tf::quaternionMsgToTF(ps_world_goal.orientation, q);
  double yaw_world = tf::getYaw(q);

  kr_tracker_msgs::LineTrackerGoal goal;
  goal.x   = ps_world_goal.position.x;
  goal.y   = ps_world_goal.position.y;
  goal.z   = ps_world_goal.position.z;
  goal.yaw = yaw_world;
  goal.duration = duration;
  goal.t_start = t_start;
  goal.v_des = v_des;
  goal.a_des = a_des;
  goal.relative = relative;

  line_tracker_min_jerk_client_.sendGoal(goal, boost::bind(&TAGManager::tracker_done_callback, this, _1, _2),
                                         ClientType::SimpleActiveCallback(), ClientType::SimpleFeedbackCallback());
  ROS_INFO("Going to {%2.2f, %2.2f, %2.2f, %2.2f}%s with duration %2.2f", x, y, z, yaw,
           (relative ? " relative to the current position." : ""), duration.toSec());

  return this->transition(line_tracker_min_jerk);

}

/*
bool TAGManager::goToTimed(float x, float y, float z, float yaw, float v_des, float a_des, bool relative, ros::Duration duration, ros::Time t_start) {

  if((ros::Time::now() - last_odom_t_).toSec() > 2.0){
    ROS_ERROR("odom not updated since %f sec", ros::Time::now() - last_odom_t_).toSec());
    return false;
  }

  // If goal is in global frame
  if (!relative){ // TODO aw: add 'and if odom is relative'
    if (!global_offset_init_){
      ROS_WARN("Commanded global goal, but the global offset hasn't been initialized");
      return false;
    }

    // Transform the global goal into the drifted frame
    else {
      ROS_INFO("Go to, global frame has been init");
      // Create tf Pose for global goal
      tf::Vector3 goal_position_global(x,y,z);
      tf::Quaternion goal_orientation_global;
      goal_orientation_global.setEuler(yaw, 0, 0);
      tf::Pose tf_goal_global(goal_orientation_global, goal_position_global);

      // Transform into drift frame
      tf::Pose tf_goal_drift;
      tf_goal_drift = tf_drift_global_.inverse() * tf_goal_global;

      // Modify the goal to command to drift frame
      x = tf_goal_drift.getOrigin().getX();
      y = tf_goal_drift.getOrigin().getY();
      z = tf_goal_drift.getOrigin().getZ();
      double yaw_new, pitch, roll;
      tf::Matrix3x3(tf_goal_drift.getRotation()).getEulerYPR(yaw_new, pitch, roll);
      yaw = (float)yaw_new;
      ROS_INFO("Goal in drift frame is (%2.2f, %2.2f, %2.2f, %2.2f)", x, y, z, yaw);
    }
  }

  // TODO aw: make this work with tag origin by going through common goto function
  kr_tracker_msgs::LineTrackerGoal goal;
  goal.x   = x;
  goal.y   = y;
  goal.z   = z;
  goal.yaw = yaw;
  goal.duration = duration;
  goal.t_start = t_start;
  goal.v_des = v_des;
  goal.a_des = a_des;
  goal.relative = relative;

  line_tracker_min_jerk_client_.sendGoal(goal, boost::bind(&TAGManager::tracker_done_callback, this, _1, _2),
                                         ClientType::SimpleActiveCallback(), ClientType::SimpleFeedbackCallback());
  ROS_INFO("Going to {%2.2f, %2.2f, %2.2f, %2.2f}%s with duration %2.2f", x, y, z, yaw,
           (relative ? " relative to the current position." : ""), duration.toSec());

  return this->transition(line_tracker_min_jerk);
}
*/
/*
void TAGManager::odometry_cb(const nav_msgs::Odometry::ConstPtr &msg) {
  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;

  vel_(0) = msg->twist.twist.linear.x;
  vel_(1) = msg->twist.twist.linear.y;
  vel_(2) = msg->twist.twist.linear.z;

  odom_q_ = Quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

  yaw_ = tf::getYaw(msg->pose.pose.orientation);
  yaw_dot_ = msg->twist.twist.angular.z;

  last_odom_t_ = ros::Time::now();

  //Publish odometry in the tag frame
  if (global_offset_init_){

    nav_msgs::Odometry odom_tag;
    odom_tag.header = msg->header;

    //TODO populate twist correctly, use TF to transform frames.
    odom_tag.pose.pose.position.x = tf_drift_global_.getOrigin().getX(),
    odom_tag.pose.pose.position.y = tf_drift_global_.getOrigin().getY();
    odom_tag.pose.pose.position.z = tf_drift_global_.getOrigin().getZ();

    odom_tag.pose.pose.orientation.x = tf_drift_global_.getRotation().getX(),
    odom_tag.pose.pose.orientation.y = tf_drift_global_.getRotation().getY();
    odom_tag.pose.pose.orientation.z = tf_drift_global_.getRotation().getZ();
    odom_tag.pose.pose.orientation.w = tf_drift_global_.getRotation().getW();

    odom_tag_pub_.publish(odom_tag);
  }
}*/

/*
void TAGManager::tag_pose_cb(const apriltag_msgs::ApriltagPoseStamped &msg){

  // TODO aw: only use the tf from hires to base_link here. Something is going wrong with the pose estimator here
  // TODO aw: incoperate measurements into goto framework. ie what happens when I see a tag while going to a global position

  // Read the tag to odom transformation

  if((ros::Time::now() - last_odom_t_).toSec() > 2.0){
    ROS_ERROR("odom not updated since %d sec", 2.0);
    return;
  }

  tf::StampedTransform tf_tag_odom;
  try {
    tf_listener_.lookupTransform("/base_link", "/tag_0", ros::Time(0), tf_tag_odom);
  } catch (tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
    return;
  }

  ROS_INFO("Tag location is (%2.2f, %2.2f, %2.2f)",
      tf_tag_odom.getOrigin().x(),
      tf_tag_odom.getOrigin().y(),
      tf_tag_odom.getOrigin().z());

  //TODO make tag_bound param.
  float tag_bound = 100;
  if(tf_tag_odom.getOrigin().absolute().maxAxis() > tag_bound){
    ROS_WARN("Tag detection is out of bounds");
    return;
  }

  // Convert odom to a tf transform dtype
  tf::Transform tf_odom_drift;

  tf_odom_drift.setOrigin(tf::Vector3(pos_(0), pos_(1), pos_(2))); //Use odom timestamp or get latest odom
  tf_odom_drift.setRotation(tf::Quaternion(
      odom_q_.x(), odom_q_.y(), odom_q_.z(), odom_q_.w()));

  ROS_INFO("Odom center is (%2.2f, %2.2f, %2.2f)",
      tf_odom_drift.getOrigin().x(),
      tf_odom_drift.getOrigin().y(),
      tf_odom_drift.getOrigin().z());

  tf::Transform tf_tag_global;
  tf_tag_global.setIdentity();

  tf::Transform tf_drift_global;
  tf_drift_global = tf_tag_global * tf_odom_drift * tf_tag_odom;
  tf_drift_global = tf_drift_global.inverse();

  // Determine the initial yaw offset
  double yaw, pitch, roll;
  tf::Matrix3x3(tf_drift_global.getRotation()).getEulerYPR(yaw, pitch, roll);

  // Low pass filter the result
  // TODO aw: get drift filter to work. Probably need to get rid of NaNs and infs

  // global_offset_lin_(0) = (1.0-alpha) * global_offset_lin_(0) + alpha * tf_drift_global.getOrigin().getX();
  // global_offset_lin_(1) = (1.0-alpha) * global_offset_lin_(1) + alpha * tf_drift_global.getOrigin().getY();
  // global_offset_lin_(2) = (1.0-alpha) * global_offset_lin_(2) + alpha * tf_drift_global.getOrigin().getZ();
  // global_offset_yaw_    = (1.0-alpha) * global_offset_yaw_    + alpha * yaw;


  // TODO aw: use the filtered one here
  // tf_drift_global_.setOrigin(tf::Vector3(global_offset_lin_(0), global_offset_lin_(1), global_offset_lin_(2)));
  // tf_drift_global_.setRotation(tf_drift_global.getRotation());

  if (!global_offset_init_){
    tf_drift_global_ = tf_drift_global;
    global_offset_init_ = true;
  } else {
    tf_drift_global_.setOrigin(
      tf_drift_global_.getOrigin().lerp(tf_drift_global.getOrigin(), tag_filter_alpha_));
    tf_drift_global_.setRotation(
      tf_drift_global_.getRotation().slerp(tf_drift_global.getRotation(), tag_filter_alpha_));
  }

  // tf_drift_global_.setOrigin(tf_drift_global.getOrigin());
  // tf_drift_global_.setRotation(tf_drift_global.getRotation());

  ROS_INFO("Raw drift in global frame: lin = (%2.2f, %2.2f, %2.2f)",
      tf_drift_global.getOrigin().getX(),
      tf_drift_global.getOrigin().getY(),
      tf_drift_global.getOrigin().getZ());

  ROS_INFO("Filtered drift in global frame: lin = (%2.2f, %2.2f, %2.2f)",
      tf_drift_global_.getOrigin().getX(),
      tf_drift_global_.getOrigin().getY(),
      tf_drift_global_.getOrigin().getZ());


  static tf::TransformBroadcaster tf_broadcaster;
  tf_broadcaster.sendTransform(
      tf::StampedTransform(tf_drift_global_, ros::Time::now(), "global", "drift")); //TODO use timestamp from tag


  // TODO aw: Update state machine accordingly
  // TODO aw: establish protocol for large jumps. Should we replan a trajectory?
}
*/

bool TAGManager::transition(const std::string &tracker_str)
{
  kr_tracker_msgs::Transition transition_cmd;
  transition_cmd.request.tracker = tracker_str;

  if (srv_transition_.call(transition_cmd) && transition_cmd.response.success)
  {
    active_tracker_ = tracker_str;
    ROS_INFO("Current tracker: %s", tracker_str.c_str());
    return true;
  }

  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tag_manager");
  ros::NodeHandle nh;

  auto tag_man = std::make_shared<TAGManager>();

  ros::spin();

  return 0;
}
