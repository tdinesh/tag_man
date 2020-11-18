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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>
#include <angles/angles.h>

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
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&TAGManager::odometry_cb, this, _1));

  traj_sub_ = nh_.subscribe<kr_tracker_msgs::TrajectoryTrackerGoal>("traj", 1, boost::bind(&TAGManager::traj_cb, this, _1));

  // Tag subscriber
  tag_pose_sub_ = nh_.subscribe<apriltag_msgs::ApriltagPoseStamped>("tag_poses", 1, boost::bind(&TAGManager::tag_pose_cb,this, _1));

  // cmd_vel subscriber
  cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("tag_cmd_vel", 1, boost::bind(&TAGManager::cmd_vel_cb,this, _1));
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("odom_cmd_vel", 10);

  // Services
  srv_transition_ = nh_.serviceClient<kr_tracker_msgs::Transition>("trackers_manager/transition");

  priv_nh_.param("tag_filter_alpha", tag_filter_alpha_, 0.1f);
  priv_nh_.param("origin_tag_id", origin_tag_id_, 0);

  priv_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
  priv_nh_.param<std::string>("global_frame", global_frame_, "global_origin");

  priv_nh_.param("clamp_tag", clamp_tag_, false);

  global_offset_yaw_ = 0;

  srv_goto_ = priv_nh_.advertiseService("goTo", &TAGManager::goTo_cb, this);
  srv_goto_timed_ = priv_nh_.advertiseService("goToTimed", &TAGManager::goToTimed_cb, this);

  ekf_meas_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("ekf_measurement", 2);
  ekf_up_pub_ =  nh_.advertise<geometry_msgs::PoseStamped>("ekf_update", 2);
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
  {
    ROS_WARN_THROTTLE(1, "Tag Num %d not found", origin_tag_id_);
    return;
  }

  geometry_msgs::Pose ps_cam_tag; //Tag pose in camera frame
  ps_cam_tag = msg->posearray.poses[tag_ind];
  // ROS_INFO("Tag in %s frame: lin = (%2.2f, %2.2f, %2.2f)",
  //   msg->header.frame_id.c_str(),
  //   ps_cam_tag.position.x,
  //   ps_cam_tag.position.y,
  //   ps_cam_tag.position.z);

  geometry_msgs::TransformStamped gtfs_odom_cam;
  ros::Time cur = ros::Time::now();
  try
  {
    gtfs_odom_cam = tf_buffer_.lookupTransform(odom_frame_, msg->header.frame_id, msg->header.stamp);
    //gtfs_odom_cam = tf_buffer_.lookupTransform(odom_frame_, msg->header.frame_id, ros::Time(0));

  }
  catch(tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
    return;
  }

  geometry_msgs::Pose ps_odom_tag; //Tag pose in odom frame
  tf2::doTransform(ps_cam_tag, ps_odom_tag, gtfs_odom_cam);

  geometry_msgs::TransformStamped gtfs_odom_tag;

  gtfs_odom_tag.header.stamp = msg->header.stamp;
  gtfs_odom_tag.header.frame_id = odom_frame_;
  gtfs_odom_tag.child_frame_id = global_frame_;

  gtfs_odom_tag.transform.translation.x = ps_odom_tag.position.x;
  gtfs_odom_tag.transform.translation.y = ps_odom_tag.position.y;
  gtfs_odom_tag.transform.translation.z = ps_odom_tag.position.z;
  gtfs_odom_tag.transform.rotation = ps_odom_tag.orientation;

  geometry_msgs::PoseStamped ekf_meas;
  ekf_meas.header = gtfs_odom_tag.header;
  ekf_meas.pose = ps_odom_tag;
  ekf_meas_pub_.publish(ekf_meas);

  //send the transform
  //tf_broadcaster_.sendTransform(gtfs_odom_tag);

  // ROS_INFO("Tag in %s frame: lin = (%2.2f, %2.2f, %2.2f)",
  //   odom_frame_.c_str(),
  //   ps_odom_tag.position.x,
  //   ps_odom_tag.position.y,
  //   ps_odom_tag.position.z);

  do_ekf(gtfs_odom_tag);

}

void TAGManager::do_ekf(geometry_msgs::TransformStamped& gtfs_odom_tag)
{

  ROS_INFO_STREAM("tag" << gtfs_odom_tag);

  tf2::Stamped< tf2::Transform > tf2_odom_tag;
  tf2::fromMsg(gtfs_odom_tag, tf2_odom_tag);

  tf2::Vector3 trans = tf2_odom_tag.getOrigin();
  tf2::Matrix3x3 rot = tf2_odom_tag.getBasis();

  double r, p, y;
  rot.getRPY(r, p, y);
  ROS_INFO_STREAM("yaw " << angles::to_degrees(y) << " roll " << angles::to_degrees(r)  << " pitch " << angles::to_degrees(p));

  if(!gtsam_init_)
  {
    // Create the Kalman Filter initialization point
    gtsam::Point3 pt_initial(trans[0], trans[1], trans[2]);


    gtsam::Rot3 rot_initial(rot[0][0], rot[0][1], rot[0][2],
                            rot[1][0], rot[1][1], rot[1][2],
                            rot[2][0], rot[2][1], rot[2][2]);

    gtsam::Pose3 x_initial(rot_initial,pt_initial);

    gtsam::Vector6 poseNoise_initial;
    poseNoise_initial << gtsam::Vector3::Constant(0.2), gtsam::Vector3::Constant(0.2);// 0.2 rad on roll,pitch,yaw and 20cm std on x,y,z
    gtsam::SharedDiagonal P_initial = gtsam::noiseModel::Diagonal::Sigmas(poseNoise_initial, true);

    // Create Key for initial pose
    gtsam::Symbol x0('x',symbol_cnt_);

    //gtsam::traits<gtsam::Pose3>::Print(x_initial, "X0 initial");
    x_initial.print("X0 initial");
    P_initial->print("P initial ");

    // Create an ExtendedKalmanFilter object
    ekf_.reset(new gtsam::ExtendedKalmanFilter<gtsam::Pose3>(x0, x_initial, P_initial));

    gtsam_init_ = true;

    symbol_cnt_++;

    ROS_INFO("\n GTSAM INIT");
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

  //gtsam::Vector6 u;
  //u << (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  gtsam::Vector6 process_noise;
  process_noise << gtsam::Vector3::Constant(0.05), gtsam::Vector3::Constant(0.1); // 0.05 rad on roll,pitch,yaw and 10cm std on x,y,z
  gtsam::SharedDiagonal Q = gtsam::noiseModel::Diagonal::Sigmas(process_noise, true);
  //Q->print("Q process noise ");

  gtsam::Symbol x0('x',symbol_cnt_ - 1);

  // This simple motion can be modeled with a BetweenFactor
  // Create Key for next pose
  gtsam::Symbol x1('x',symbol_cnt_);

  // Predict delta based on controls
  gtsam::Point3 pt_diff(0, 0, 0);
  gtsam::Rot3 rot_diff(1, 0, 0,
                       0, 1, 0,
                       0, 0, 1);
  gtsam::Pose3 difference(rot_diff,pt_diff);
  //difference.print("difference ");

  // Create Factor
  gtsam::BetweenFactor<gtsam::Pose3> factor1(x0, x1, difference, Q);

  // Predict the new value with the EKF class
  gtsam::Pose3 x1_predict = ekf_->predict(factor1);
  //gtsam::traits<gtsam::Pose3>::Print(x1_predict, "X1 Predict");

  // Now, a measurement, z1, has been received, and the Kalman Filter should be "Updated"/"Corrected"
  // This is equivalent to saying P(x1|z1) ~ P(z1|x1)*P(x1)
  // For the Kalman Filter, this requires a measurement model h(x_{t}) = \hat{z}_{t}
  // Assuming the system is linear, this will be of the form h(x_{t}) = H*x_{t} + v
  // where H is the observation model/matrix, and v is zero-mean, Gaussian white noise with covariance R
  //
  // For the purposes of this example, let us assume we have something like a GPS that returns
  // the current position of the robot. Then H = [1 0 ; 0 1]. Let us also assume that the measurement noise
  // R = [0.25 0 ; 0 0.25].
  gtsam::Vector6 measurement_noise;
  measurement_noise << gtsam::Vector3::Constant(0.05), gtsam::Vector3::Constant(0.1); // 0.05 rad on roll,pitch,yaw and 10cm std on x,y,z
  gtsam::SharedDiagonal R = gtsam::noiseModel::Diagonal::Sigmas(measurement_noise, true);
  //R->print("R measurement noise ");

  // This simple measurement can be modeled with a PriorFactor
  gtsam::Point3 z_pt(trans[0], trans[1], trans[2]);

  gtsam::Rot3 z_rot(rot[0][0], rot[0][1], rot[0][2],
                    rot[1][0], rot[1][1], rot[1][2],
                    rot[2][0], rot[2][1], rot[2][2]);

  gtsam::Pose3 z1(z_rot,z_pt);

  gtsam::PriorFactor<gtsam::Pose3> factor2(x1, z1, R);

  // Update the Kalman Filter with the measurement
  gtsam::Pose3 x1_update = ekf_->update(factor2);
  //gtsam::traits<gtsam::Pose3>::Print(x1_update, "X1 Update");

  gtsam::Point3 t1_update = x1_update.translation();
  gtsam::Rot3 rot1_update = x1_update.rotation();

  tf2::Vector3 trans_update(t1_update.x(), t1_update.y(), t1_update.z());
  tf2::Matrix3x3 rot_update(rot1_update.r1().x(), rot1_update.r2().x(), rot1_update.r3().x(),
                            rot1_update.r1().y(), rot1_update.r2().y(), rot1_update.r3().y(),
                            rot1_update.r1().z(), rot1_update.r2().z(), rot1_update.r3().z());
  double ru, pu, yu;
  rot_update.getRPY(ru, pu, yu);
  ROS_INFO_STREAM("yaw " << angles::to_degrees(yu) << " roll " << angles::to_degrees(ru)  << " pitch " << angles::to_degrees(pu));

  tf2::Stamped< tf2::Transform > tf2_odom_tag_update;
  if(!clamp_tag_)
  {
    tf2_odom_tag_update.setOrigin(trans_update);
    tf2_odom_tag_update.setBasis(rot_update);
  }
  else
  {
    tf2::Vector3 trans_update_clamped(t1_update.x(), t1_update.y(), 0.0);
    tf2::Matrix3x3 rot_update_clamped;
    rot_update_clamped.setRPY(0.0, 0.0, yu);

    tf2_odom_tag_update.setOrigin(trans_update_clamped);
    tf2_odom_tag_update.setBasis(rot_update_clamped);
  }

  geometry_msgs::TransformStamped gtfs_odom_tag_update = tf2::toMsg(tf2_odom_tag_update);

  gtfs_odom_tag_update.header.stamp = gtfs_odom_tag.header.stamp;
  gtfs_odom_tag_update.header.frame_id = gtfs_odom_tag.header.frame_id;
  gtfs_odom_tag_update.child_frame_id = gtfs_odom_tag.child_frame_id;

  //send the transform
  tf_broadcaster_.sendTransform(gtfs_odom_tag_update);

  geometry_msgs::PoseStamped ekf_up;
  ekf_up.header = gtfs_odom_tag_update.header;
  ekf_up.pose.position.x = gtfs_odom_tag_update.transform.translation.x;
  ekf_up.pose.position.y = gtfs_odom_tag_update.transform.translation.y;
  ekf_up.pose.position.z = gtfs_odom_tag_update.transform.translation.z;
  ekf_up.pose.orientation = gtfs_odom_tag_update.transform.rotation;
  ekf_up_pub_.publish(ekf_up);

  ROS_INFO_STREAM(gtfs_odom_tag_update);
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

    geometry_msgs::TransformStamped tf_odom_global;
    ros::Time cur = ros::Time::now();
    try
    {
      tf_odom_global = tf_buffer_.lookupTransform(odom_frame_, global_frame_, ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
      return;
    }

    kr_tracker_msgs::TrajectoryTrackerGoal goal_odom;

    for(unsigned int i = 0; i < msg->waypoints.size(); i++)
    {
      //convert pose in global_frame to odom_frame
      geometry_msgs::Pose ps_global_goal = msg->waypoints[i];

      //tf2::Quaternion q_origin;
      //q_origin.setRPY(0.0, 0.0, 0.0);
      //ps_global_goal.orientation = tf2::toMsg(q_origin);

      geometry_msgs::Pose ps_odom_goal; //Tag pose in odom frame
      tf2::doTransform(ps_global_goal, ps_odom_goal, tf_odom_global);

      ROS_INFO("goal (%2.2f, %2.2f, %2.2f) in global frame: lin = (%2.2f, %2.2f, %2.2f) %2.2f",
          ps_global_goal.position.x,
          ps_global_goal.position.y,
          ps_global_goal.position.z,
          ps_odom_goal.position.x,
          ps_odom_goal.position.y,
          ps_odom_goal.position.z, msg->waypoint_times[i]);

      goal_odom.waypoints.push_back(ps_odom_goal);
      //goal_odom.waypoint_times.push_back(msg->waypoint_times[i]);
    }

    trajectory_tracker_client_.sendGoal(goal_odom);
    //trajectory_tracker_client_.sendGoal(goal_odom, boost::bind(&TAGManager::traj_done_callback, this, _1, _2),
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
  yaw_ = tf::getYaw(msg->pose.pose.orientation);
  yaw_dot_ = msg->twist.twist.angular.z;

  last_odom_t_ = ros::Time::now();

  geometry_msgs::Pose ps_odom;
  ps_odom = msg->pose.pose;

  geometry_msgs::TransformStamped tf_global_odom;
  ros::Time cur = ros::Time::now();
  try
  {
    tf_global_odom = tf_buffer_.lookupTransform(global_frame_, odom_frame_, ros::Time(0));
    //TODO Use msg->header.stamp?
  }
  catch(tf2::TransformException &ex)
  {
    ROS_WARN_THROTTLE(1, "%s",ex.what());
    return;
  }

  geometry_msgs::Pose ps_global_odom; //odom in global frame
  tf2::doTransform(ps_odom, ps_global_odom, tf_global_odom);

  //Publish odometry in the global_frame
  nav_msgs::Odometry odom_tag;
  odom_tag.header = msg->header;
  odom_tag.header.frame_id = global_frame_;

  //TODO populate twist correctly
  odom_tag.pose.pose = ps_global_odom,
  odom_tag_pub_.publish(odom_tag);
}

void TAGManager::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
  geometry_msgs::TransformStamped tf_odom_global;
  ros::Time cur = ros::Time::now();
  try
  {
    tf_odom_global = tf_buffer_.lookupTransform(odom_frame_, global_frame_, ros::Time(0));
  }
  catch(tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
    return;
  }

  tf::Quaternion q;
  tf::quaternionMsgToTF(tf_odom_global.transform.rotation, q);
  double yaw_odom = tf::getYaw(q);

  geometry_msgs::Twist goal;
  goal.linear.x = msg->linear.x * std::cos(yaw_odom) - msg->linear.y * std::sin(yaw_odom);
  goal.linear.y = msg->linear.x * std::sin(yaw_odom) + msg->linear.y * std::cos(yaw_odom);
  goal.linear.z = msg->linear.z;
  goal.angular.z = msg->angular.z;

  //ROS_WARN("yaw %g", anglges::to_degrees(yaw_odom));
  cmd_vel_pub_.publish(goal);
}

bool TAGManager::goToTimed(float x, float y, float z, float yaw, float v_des, float a_des, bool relative, ros::Duration duration, ros::Time t_start)
{
  //convert pose in global_frame to odom_frame
  geometry_msgs::Pose ps_global_goal; //goal in origin frame
  ps_global_goal.position.x = x;
  ps_global_goal.position.y = y;
  ps_global_goal.position.z = z;

  tf2::Quaternion q_origin;
  q_origin.setRPY(0.0, 0.0, yaw);
  ps_global_goal.orientation = tf2::toMsg(q_origin);

  geometry_msgs::TransformStamped tf_odom_global;
  ros::Time cur = ros::Time::now();
  try
  {
    tf_odom_global = tf_buffer_.lookupTransform(odom_frame_, global_frame_, ros::Time(0));
  }
  catch(tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
    return false;
  }

  geometry_msgs::Pose ps_odom_goal; //Tag pose in odom frame
  tf2::doTransform(ps_global_goal, ps_odom_goal, tf_odom_global);

  //tf2::Quaternion q;
  //tf2::fromMsg(ps_odom_goal.orientation, q);
  tf::Quaternion q;
  tf::quaternionMsgToTF(ps_odom_goal.orientation, q);
  double yaw_odom = tf::getYaw(q);

  kr_tracker_msgs::LineTrackerGoal goal;
  goal.x   = ps_odom_goal.position.x;
  goal.y   = ps_odom_goal.position.y;
  goal.z   = ps_odom_goal.position.z;
  goal.yaw = yaw_odom;
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
