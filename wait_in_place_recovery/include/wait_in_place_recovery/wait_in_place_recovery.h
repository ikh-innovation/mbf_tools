#pragma once

#include <mbf_costmap_core/costmap_recovery.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>

namespace gm=geometry_msgs;
namespace cmap=costmap_2d;

namespace wait_in_place_recovery
{

/// Recovery behavior that takes a given twist and tries to execute it for up to
/// d seconds, or until reaching an obstacle.  
class WaitRecovery : public mbf_costmap_core::CostmapRecovery
{
public:
  
    /// Doesn't do anything: initialize is where the actual work happens
    WaitRecovery()
      : local_costmap_(NULL),
        tf_(NULL),
        initialized_(false),
        canceled_(true)
    {

    }

    /// Initialize the parameters of the behavior
    virtual void initialize (std::string n, tf2_ros::Buffer* tf,
                     costmap_2d::Costmap2DROS* global_costmap,
                     costmap_2d::Costmap2DROS* local_costmap);

    /// Run the behavior
    virtual uint32_t runBehavior(std::string& message);

    virtual bool cancel();

    virtual ~WaitRecovery() { };

    private:
    gm::Pose2D getCurrentRobotPose() const;
    uint32_t wait_routine();

    ros::NodeHandle nh_;
    costmap_2d::Costmap2DROS* local_costmap_;
    tf2_ros::Buffer* tf_;
    bool initialized_;
    bool canceled_;

    double wait_time_;
    double preemption_test_interval_;
};

} // namespace wait_in_place_recovery
