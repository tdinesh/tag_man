#ifndef TAG_MANAGER_H
#define TAG_MANAGER_H

// Standard C++
#include <string>
#include <Eigen/Geometry>
#include <array>

// ROS related
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <actionlib/client/simple_action_client.h>

#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <apriltag_msgs/ApriltagPoseStamped.h>

#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_tracker_msgs/LineTrackerAction.h>
#include <kr_tracker_msgs/TrajectoryTrackerAction.h>
#include <kr_mav_manager/Vec4.h>
#include <kr_mav_manager/GoalTimed.h>

#include <gtsam/nonlinear/ExtendedKalmanFilter.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

class TAGManager
{
  public:

    // Typedefs
    typedef Eigen::Vector2f    Vec2;
    typedef Eigen::Vector3f    Vec3;
    typedef Eigen::Vector4f    Vec4;
    typedef Eigen::Quaternionf Quat;

    TAGManager(std::string ns = "");

    Vec3 global_offset_lin() { return global_offset_lin_; }
    float global_offset_yaw() { return global_offset_yaw_; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
    typedef actionlib::SimpleActionClient<kr_tracker_msgs::LineTrackerAction> ClientType;
    typedef actionlib::SimpleActionClient<kr_tracker_msgs::TrajectoryTrackerAction> TrajClientType;

    void odometry_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void tracker_done_callback(const actionlib::SimpleClientGoalState& state, const kr_tracker_msgs::LineTrackerResultConstPtr& result);
    void traj_done_callback(const actionlib::SimpleClientGoalState& state, const kr_tracker_msgs::TrajectoryTrackerResultConstPtr& result);
    void traj_cb(const kr_tracker_msgs::TrajectoryTrackerGoal::ConstPtr &msg);

    void tag_pose_cb(const apriltag_msgs::ApriltagPoseStamped::ConstPtr& msg);

    bool goTo_cb(kr_mav_manager::Vec4::Request &req, kr_mav_manager::Vec4::Response &res);
    bool goToTimed_cb(kr_mav_manager::GoalTimed::Request &req, kr_mav_manager::GoalTimed::Response &res);

    bool goToTimed(float x, float y, float z, float yaw, float v_des = 0.0f, float a_des = 0.0f, bool relative = false,
                   ros::Duration duration = ros::Duration(0), ros::Time start_time = ros::Time::now());

    bool transition(const std::string &tracker_str);

    void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr &msg);

    void do_ekf(geometry_msgs::Pose& ps_cam_tag, geometry_msgs::TransformStamped& gtfs_odom_tag);

    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    ros::Publisher odom_tag_pub_, cmd_vel_pub_;
    ros::Time last_odom_t_;

    tf2_ros::Buffer tf_buffer_;
    //std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // Actionlibs
    ClientType line_tracker_min_jerk_client_;
    TrajClientType trajectory_tracker_client_;

    // Subscribers
    ros::Subscriber odom_sub_, tag_pose_sub_, traj_sub_, cmd_vel_sub_;

    // Services
    ros::ServiceClient srv_transition_;
    ros::ServiceServer srv_goto_, srv_goto_timed_;

    std::string active_tracker_;

    Vec3 pos_, vel_;
    Quat odom_q_;
    float yaw_, yaw_dot_;

    // Estimated offset from global frame measured from tag
    //tf2::Transform tf_drift_global_;
    int origin_tag_id_;
    bool global_offset_init_;
    Vec3 global_offset_lin_;
    double global_offset_yaw_;
    float tag_filter_alpha_;
    // TODO aw: should the tag always be called the origin? For now yes
    // TODO aw: make new variable in case global origin isn't needed

    std::string odom_frame_, global_frame_;

    bool gtsam_init_;

    std::unique_ptr<gtsam::ExtendedKalmanFilter<gtsam::Pose3> > ekf_;

    int symbol_cnt_;
};

#endif /* TAG_MANAGER_H */
