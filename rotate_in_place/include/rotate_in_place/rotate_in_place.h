#pragma once
#include <mbf_costmap_core/costmap_recovery.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <mbf_msgs/CheckPose.h>
#include <mbf_exposed_services/check_pose.h>
#include <rotate_in_place/pid.h>
#include <rotate_in_place/RotateInPlaceConfig.h>

namespace gm=geometry_msgs;
namespace cmap=costmap_2d;

namespace rotate_in_place
{

/// Recovery behavior that takes a given twist and tries to execute it for up to
/// d seconds, or until reaching an obstacle.  
class RotateInPlace : public mbf_costmap_core::CostmapRecovery
{
public:
  
    /// Doesn't do anything: initialize is where the actual work happens
    RotateInPlace()
      : local_costmap_(NULL),
        tf_(NULL),
        initialized_(false),
        canceled_(true)
    {
        zero_twist_.linear.x = 0.0;
        zero_twist_.linear.y = 0.0;
        zero_twist_.linear.z = 0.0;
        zero_twist_.angular.x = 0.0;
        zero_twist_.angular.y = 0.0;
        zero_twist_.angular.z = 0.0;

        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;

    }

    /// Initialize the parameters of the behavior
    virtual void initialize (std::string n, tf2_ros::Buffer* tf,
                     costmap_2d::Costmap2DROS* global_costmap,
                     costmap_2d::Costmap2DROS* local_costmap);

    /// Run the behavior
    virtual uint32_t runBehavior(std::string& message);

    virtual bool cancel();

    virtual ~RotateInPlace() { };

    void reconfigure_callback(rotate_in_place::RotateInPlaceConfig& config, uint32_t level);

    private:
    gm::Pose2D getCurrentRobotPose() const;
    geometry_msgs::PoseStamped getRotatedRobotPose(double yaw) const;
    uint32_t rotate();
    uint32_t publishStop() const;

    ros::NodeHandle nh_;
    costmap_2d::Costmap2DROS* local_costmap_;
    tf2_ros::Buffer* tf_;
    ros::Publisher cmd_vel_pub_;
    bool initialized_;
    bool canceled_;

    gm::Twist zero_twist_, twist;

    //mbf check_pose_cost 
    mbf_msgs::CheckPose check_pose_srv;
    mbf_msgs::CheckPose::Response check_pose_res_prev;
    mbf_exposed_services::CheckPose check_pose;
    double rotate_safety_dist; 
    double rotate_lookahead;
    bool enable_check_pose;

    // dynamic reconfigure
    dynamic_reconfigure::Server<rotate_in_place::RotateInPlaceConfig>* config_server;
    dynamic_reconfigure::Server<rotate_in_place::RotateInPlaceConfig>::CallbackType f;

    double relative_yaw_reference;
    double kp;
    double ki;
    double kd;
    double max_output;
    double min_output;
    double tolerance;
    double timeout;
    double controller_frequency;
    double projected_pose_angle;

};

} // namespace rotate_in_place
