#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <wait_in_place_recovery/wait_in_place_recovery.h>
#include <mbf_msgs/ExePathResult.h>

// register as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(wait_in_place_recovery::WaitRecovery, mbf_costmap_core::CostmapRecovery)

namespace wait_in_place_recovery
{

void WaitRecovery::initialize(std::string name, tf2_ros::Buffer* tf,
                                cmap::Costmap2DROS* /*global_costmap*/, cmap::Costmap2DROS* local_costmap)
{
    tf_ = tf;
    local_costmap_ = local_costmap;

    ros::NodeHandle private_nh("~/" + name);

    private_nh.param("wait_time", wait_time_, 20.0);
    private_nh.param("preemption_test_interval", preemption_test_interval_, 1.0);

    initialized_ = true;
}

// Get pose in local costmap frame
gm::Pose2D WaitRecovery::getCurrentRobotPose() const
{
    geometry_msgs::PoseStamped p;
    local_costmap_->getRobotPose(p);
    gm::Pose2D pose;

    pose.x = p.pose.position.x;
    pose.y = p.pose.position.y;
    pose.theta = tf2::getYaw(p.pose.orientation);
    return pose;
}


uint32_t WaitRecovery::wait_routine()
{
    // while{
        ros::Duration(wait_time_).sleep();
        // TODO: Normally preemption checking should happen in user-defined intervals instead of waiting the whole wait_time_
        // if (canceled_) {
        //     return mbf_msgs::ExePathResult::CANCELED;
        // }
    // }
    return mbf_msgs::ExePathResult::SUCCESS;
}

bool WaitRecovery::cancel() {
    canceled_ = true;
    return true;
}

uint32_t WaitRecovery::runBehavior (std::string& message)
{
    ROS_ASSERT (initialized_);
    ROS_INFO("Start Wait-in-place-Recovery.");
    canceled_ = false;

    // Figure out how long we can safely run the behavior
    // const gm::Pose2D& initialPose = getCurrentRobotPose();

    // initial pose
    // ROS_DEBUG("initial pose (%.2f, %.2f, %.2f)", initialPose.x,
    //                 initialPose.y, initialPose.theta);

    ROS_INFO("Waiting for %f",wait_time_);
    wait_routine();
    ROS_INFO("complete waiting.");

    // double final_diff = getCurrentDiff(initialPose);
    // ROS_DEBUG("final_diff = %.2f",final_diff);

    ROS_INFO("Finished Wait-in-place-Recovery.");
}


} // namespace wait_in_place_recovery
