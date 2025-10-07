#include <mbf_msgs/RecoveryResult.h>
#include <rotate_in_place/rotate_in_place.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>

// register as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(rotate_in_place::RotateInPlace,
                       mbf_costmap_core::CostmapRecovery)

namespace rotate_in_place {

void RotateInPlace::initialize(
    std::string name, tf2_ros::Buffer *tf,
    cmap::Costmap2DROS * /*global_costmap*/,
    cmap::Costmap2DROS *local_costmap) {
  tf_ = tf;
  local_costmap_ = local_costmap;

  cmd_vel_pub_ = nh_.advertise<gm::Twist>("cmd_vel", 10);
  ros::NodeHandle private_nh("~/" + name);

  private_nh.param("yaw_goal", relative_yaw_reference, 0.0);
  private_nh.param("kp", kp, 1.0);
  private_nh.param("ki", ki, 0.0);
  private_nh.param("kd", kd, 0.0);
  private_nh.param("max_cmd", max_output, 1.0);
  private_nh.param("tolerance", tolerance, 0.09);
  private_nh.param("timeout", timeout, 10.0);
  private_nh.param("controller_frequency", controller_frequency, 10.0);
  min_output = - max_output;

  // check_pose related stuff
  private_nh.param("aristos_behaviour/CHECKPOSE_ROTATE_ENABLE", enable_check_pose, true);
  private_nh.param("aristos_behaviour/CHECKPOSE_ROTATE_DIST", rotate_safety_dist, 0.3);
  private_nh.param("aristos_behaviour/CHECKPOSE_ROTATE_LOOKAHEAD", rotate_lookahead, 0.3);

  // dynamic reconfigure
  config_server = new dynamic_reconfigure::Server<rotate_in_place::RotateInPlaceConfig>(private_nh);
  dynamic_reconfigure::Server<rotate_in_place::RotateInPlaceConfig>::CallbackType cb =
      boost::bind(&RotateInPlace::reconfigure_callback, this, _1, _2);
  config_server->setCallback(cb);

  initialized_ = true;
}

// Get pose in local costmap frame
gm::Pose2D RotateInPlace::getCurrentRobotPose() const {
  geometry_msgs::PoseStamped p;
  local_costmap_->getRobotPose(p);
  gm::Pose2D pose;

  pose.x = p.pose.position.x;
  pose.y = p.pose.position.y;
  pose.theta = tf2::getYaw(p.pose.orientation);
  return pose;
}

geometry_msgs::PoseStamped
RotateInPlace::getRotatedRobotPose(double yaw) const {
  geometry_msgs::PoseStamped p;
  local_costmap_->getRobotPose(p);

  tf2::Quaternion q_orig, q_rot, q_new;
  tf2::fromMsg(p.pose.orientation, q_orig);

  q_rot.setRPY(0, 0, yaw);  
  q_new = q_rot * q_orig;
  q_new.normalize();

  p.pose.orientation = tf2::toMsg(q_new);
  return p;
}

uint32_t RotateInPlace::rotate() {

auto pid = PIDController(kp, ki, kd, min_output, max_output);
gm::Twist twist;
ros::Rate r(controller_frequency);
double cmd = 0.0;
double error = 0.0;
const gm::Pose2D initialPose = getCurrentRobotPose();
double reference_yaw = initialPose.theta + relative_yaw_reference;
double elapsed_time = 0.0;
ros::Time init_time = ros::Time::now();

while (true)
{
  if (canceled_) {
      return mbf_msgs::RecoveryResult::CANCELED;
    }

  const gm::Pose2D currPose = getCurrentRobotPose();

  ros::Time time = ros::Time::now();
  pid.compute(currPose.theta, reference_yaw, time, cmd, error);

  if(abs(error)<tolerance)
  { 
    ROS_INFO("Rotation succeeded. Final error: %f", error);
    return mbf_msgs::RecoveryResult::SUCCESS;
  }

  elapsed_time = (time-init_time).toSec();
  if(elapsed_time>timeout)
  {
    ROS_ERROR("Rotation failed. Patience Exceeded. ");
    return mbf_msgs::RecoveryResult::PAT_EXCEEDED;
  }  

  if (enable_check_pose) {
    // check current pose safety
    geometry_msgs::PoseStamped p;
    local_costmap_->getRobotPose(p);
    check_pose_srv.request.pose = p;
    check_pose_srv.request.safety_dist = 0.0; 
    check_pose.check_pose_cost(local_costmap_, check_pose_srv.request, check_pose_srv.response);

    ROS_INFO("Projected pose: x: %f, y:%f, yaw:%f", p.pose.position.x, p.pose.position.y, tf2::getYaw(p.pose.orientation));

    if (check_pose_srv.response.state == check_pose_srv.response.LETHAL)
    {
      ROS_ERROR("Rotation failed. Robot in collision. ");
      return mbf_msgs::RecoveryResult::IMPASSABLE;
    }
    if (check_pose_srv.response.state == check_pose_srv.response.UNKNOWN || check_pose_srv.response.state == check_pose_srv.response.OUTSIDE)
    {
      ROS_ERROR("Rotation failed. Robot outside or in unknown map. ");
      return mbf_msgs::RecoveryResult::TF_ERROR;
    }

    // check projected pose safety
    geometry_msgs::PoseStamped rotated_pose = getRotatedRobotPose(std::copysign(projected_pose_angle, cmd));
    ROS_INFO("Projected pose: x: %f, y:%f, yaw:%f", rotated_pose.pose.position.x, rotated_pose.pose.position.y, tf2::getYaw(rotated_pose.pose.orientation));
    check_pose_srv.request.pose = rotated_pose;
    check_pose_srv.request.safety_dist = 0.0; 
    check_pose.check_pose_cost(local_costmap_, check_pose_srv.request, check_pose_srv.response);

    if (check_pose_srv.response.state == check_pose_srv.response.LETHAL)
    {
      ROS_ERROR("Rotation failed. Imminent collision. ");
      return mbf_msgs::RecoveryResult::IMPASSABLE;
    }
    if (check_pose_srv.response.state == check_pose_srv.response.UNKNOWN || check_pose_srv.response.state == check_pose_srv.response.OUTSIDE)
    {
      ROS_ERROR("Rotation failed. Robot outside or in unknown map. ");
      return mbf_msgs::RecoveryResult::TF_ERROR;
    }
  }

  ROS_INFO("Current error: %f, Current elapsed time: %f, Sending cmd: %f", error, elapsed_time, cmd);
  twist.angular.z = cmd;
  cmd_vel_pub_.publish(twist);

  ros::spinOnce();
  r.sleep();  
  
}

}

uint32_t RotateInPlace::publishStop() const {
  ros::Rate r(controller_frequency);
  for (double t = 0; t < 1.0; t += 1 / controller_frequency) {
    cmd_vel_pub_.publish(zero_twist_);
    if (canceled_) {
      return mbf_msgs::RecoveryResult::CANCELED;
    }
    r.sleep();
  }
  return mbf_msgs::RecoveryResult::SUCCESS;
}

bool RotateInPlace::cancel() {
  canceled_ = true;
  return true;
}

uint32_t RotateInPlace::runBehavior(std::string &message) {
  ROS_ASSERT(initialized_);
  ROS_INFO("Start RotateInPlace-Recovery.");
  canceled_ = false;

  ROS_INFO("attempting to rotate");
  auto res = rotate();
  ROS_INFO("rotation attempt completed");

  publishStop();
  ROS_INFO("Finished RotateInPlace-Recovery");

  return res;
}

void RotateInPlace::reconfigure_callback(rotate_in_place::RotateInPlaceConfig& config, uint32_t level)
{
  // not thread safe, as its going to be called outside of rotate()
  relative_yaw_reference = config.yaw_goal;
  kp = config.kp;
  ki = config.ki;
  kd = config.kd;
  max_output = config.max_cmd;
  min_output = -max_output;
  tolerance = config.tolerance;
  timeout = config.timeout;
  controller_frequency = config.controller_frequency;
  projected_pose_angle = config.projected_pose_angle;
  ROS_INFO("Rotate In Place Recovery parameters updated!");

}

} // namespace rotate_in_place
