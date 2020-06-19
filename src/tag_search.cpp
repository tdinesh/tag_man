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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>
#include <kr_mav_manager/Vec4.h>
#include <nav_msgs/Odometry.h>

#include <angles/angles.h>
#include <apriltag_msgs/ApriltagPoseStamped.h>

class TagSearch
{
public:
  TagSearch(std::string ns = "");
  void searchTag();
private:

  void tagPoseCb(const apriltag_msgs::ApriltagPoseStamped::ConstPtr& msg);
  bool triggerCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void odomCb(const nav_msgs::Odometry::ConstPtr &msg);

  ros::NodeHandle nh_, pnh_;
  ros::ServiceServer srv_trigger_;
  ros::ServiceClient goto_rel_client_;
  ros::Subscriber tag_pose_sub_, odom_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool correct_tag_found_;
  bool trigger_search_;
  int tag_id_;
  float search_yaw_;
  double current_yaw_;
  geometry_msgs::Pose current_pose_;
  std::string world_frame_;
};

TagSearch::TagSearch(std::string ns): nh_(ns), pnh_("~"), tf_listener_(tf_buffer_)
{
  // Tag subscriber
  tag_pose_sub_ = nh_.subscribe<apriltag_msgs::ApriltagPoseStamped>("tag_poses", 1, boost::bind(&TagSearch::tagPoseCb,this, _1));
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&TagSearch::odomCb, this, _1));

  current_yaw_ = 0.0;
  correct_tag_found_ = false;
  trigger_search_ = false;
  pnh_.param("tag_id", tag_id_, 0);
  pnh_.param("search_yaw", search_yaw_, 0.2f);

  pnh_.param<std::string>("world_frame", world_frame_, "world");

  goto_rel_client_ = nh_.serviceClient<kr_mav_manager::Vec4>("mav_services/goToRelative");

  srv_trigger_ = nh_.advertiseService("tag_search", &TagSearch::triggerCb, this);
}

bool TagSearch::triggerCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  ROS_WARN("Tag search triggered id %d", tag_id_);
  trigger_search_ = true;
  correct_tag_found_ = false;
  res.success = true;
  return res.success;
}

void TagSearch::searchTag()
{
  if(!trigger_search_)
    return;

  if(correct_tag_found_)
    return;

  boost::array<float ,4> goto_goal = {0.0, 0.0, 0.0, search_yaw_};
  kr_mav_manager::Vec4 cmd_pos_vec;
  cmd_pos_vec.request.goal = goto_goal;
  if(goto_rel_client_.call(cmd_pos_vec))
  {
    //ROS_INFO("Messsage %s", cmd_pos_vec.response.message.c_str());
  }
  else
    ROS_ERROR("Failed to call service Messsage %s", cmd_pos_vec.response.message.c_str());

}

void TagSearch::odomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
  current_yaw_ = tf::getYaw(msg->pose.pose.orientation);
  current_pose_ = msg->pose.pose;
}

void TagSearch::tagPoseCb(const apriltag_msgs::ApriltagPoseStamped::ConstPtr& msg)
{
  if(!trigger_search_)
    return;

  ROS_WARN("Tag callback");

  //Find tag with id same as origin tag
  bool found_tag = false;
  unsigned int tag_ind = 0;

  for(unsigned int i = 0; i < msg->apriltags.size(); i++)
  {
    if(tag_id_ == msg->apriltags[i].id)
    {
      tag_ind = i;
      found_tag = true;
      break;
    }
  }

  if(!found_tag)
    return;

  correct_tag_found_ = true;
  trigger_search_ = false;

  //Hover the mav
  boost::array<float ,4> goto_goal = {0.0, 0.0, 0.0, 0.0};
  kr_mav_manager::Vec4 cmd_pos_vec;
  cmd_pos_vec.request.goal = goto_goal;
  if(goto_rel_client_.call(cmd_pos_vec))
  {
    //ROS_INFO("Messsage %s", cmd_pos_vec.response.message.c_str());
  }
  else
    ROS_ERROR("Failed to call service Messsage %s", cmd_pos_vec.response.message.c_str());

  geometry_msgs::Pose ps_cam_tag; //Tag pose in camera frame
  ps_cam_tag = msg->posearray.poses[tag_ind];
  // ROS_INFO("Tag in %s frame: lin = (%2.2f, %2.2f, %2.2f)",
  //   msg->header.frame_id.c_str(),
  //   ps_cam_tag.position.x,
  //   ps_cam_tag.position.y,
  //   ps_cam_tag.position.z);

  geometry_msgs::TransformStamped tf_world_cam;
  //ros::Time cur = ros::Time::now();
  try
  {
    tf_world_cam = tf_buffer_.lookupTransform(world_frame_, msg->header.frame_id, msg->header.stamp);
  }
  catch(tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
    return;
  }

  geometry_msgs::Pose ps_world_tag; //Tag pose in world frame
  tf2::doTransform(ps_cam_tag, ps_world_tag, tf_world_cam);

  ROS_INFO("Tag in %s frame: lin = (%2.2f, %2.2f, %2.2f)",
    world_frame_.c_str(),
    ps_world_tag.position.x,
    ps_world_tag.position.y,
    ps_world_tag.position.z);

  ROS_INFO("Current pos (%2.2f, %2.2f, %2.2f)", current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);

  double ang = std::atan2(ps_world_tag.position.y - current_pose_.position.y, ps_world_tag.position.x - current_pose_.position.x);
  ROS_INFO("Relative yaw %2.2f", ang);

  //double yaw = tf::getYaw(ps_world_tag.orientation);
  float shortest_ang_robot = angles::shortest_angular_distance(current_yaw_, ang);
  ROS_INFO("tag yaw: %2.2f, curr: %2.2f, shortest ang: %2.2f", ang, current_yaw_, shortest_ang_robot);

  boost::array<float ,4> goto_rel_goal = {0.0, 0.0, 0.0, shortest_ang_robot};
  cmd_pos_vec.request.goal = goto_rel_goal;
  if(goto_rel_client_.call(cmd_pos_vec))
  {
    //ROS_INFO("Messsage %s", cmd_pos_vec.response.message.c_str());
  }
  else
    ROS_ERROR("Failed to call service Messsage %s", cmd_pos_vec.response.message.c_str());

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tag_search");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  double rate;
  pnh.param("loop_rate", rate, 3.0);

  auto tag_man = std::make_shared<TagSearch>();

  ros::Rate loop_rate(rate);

  while (ros::ok())
  {
    tag_man->searchTag();

    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();

  return 0;
}
