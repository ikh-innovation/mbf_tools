/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <swing_recovery/swing_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>
#include <mbf_msgs/ExePathResult.h>


// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(swing_recovery::SwingRecovery, mbf_costmap_core::CostmapRecovery)

namespace swing_recovery
{
SwingRecovery::SwingRecovery(): local_costmap_(NULL), initialized_(false), world_model_(NULL)
{
}

void SwingRecovery::initialize(std::string name, tf2_ros::Buffer*,
                                costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialized_)
  {
    local_costmap_ = local_costmap;

    // get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    // we'll simulate every degree by default
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);
    private_nh.param("rotation_angle", rotation_angle_, 20.0);


    acc_lim_th_ = nav_core::loadParameterWithDeprecation(blp_nh, "acc_lim_theta", "acc_lim_th", 3.2);
    max_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "max_vel_theta", "max_rotational_vel", 1.0);
    min_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "min_in_place_vel_theta", "min_in_place_rotational_vel", 0.4);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

SwingRecovery::~SwingRecovery()
{
  delete world_model_;
}

bool SwingRecovery::cancel() {
    canceled_ = true;
    return true;
}

uint32_t SwingRecovery::runBehavior (std::string& message)
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return mbf_msgs::ExePathResult::CANCELED;
  }
  if (local_costmap_ == NULL)
  {
    ROS_ERROR("The costmap passed to the SwingRecovery object cannot be NULL. Doing nothing.");
    return mbf_msgs::ExePathResult::CANCELED;
  }
  ROS_WARN("Swing recovery behavior started.");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  geometry_msgs::PoseStamped global_pose;
  local_costmap_->getRobotPose(global_pose);

  double current_angle = tf2::getYaw(global_pose.pose.orientation);
  double previous_angle = current_angle;
  double start_angle_pos = current_angle;
  double start_angle_neg;
  bool got_pos_angle = false;
  bool got_neg_angle = false;
  bool returned = false ; 
  double augmented_current_angle = current_angle ;
  double previous_augmented_current_angle ; 
  double vel;

  while (n.ok()  &&  (!got_pos_angle || !got_neg_angle || !returned))
  {
    if (canceled_) {
      return mbf_msgs::ExePathResult::CANCELED;
    }
    // Update Current Angle
    local_costmap_->getRobotPose(global_pose);
    previous_angle = current_angle ; 
    current_angle = tf2::getYaw(global_pose.pose.orientation);
    previous_augmented_current_angle = augmented_current_angle ;
    // ROS_WARN("Current value is ");
    // printf("%f \n", current_angle/M_PI*180.0);
    // compute the distance left to rotate
    double dist_pos_left, dist_neg_left;


    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;

    if (!got_pos_angle)
    {       
      if ((previous_angle > M_PI/2.0) && (current_angle < -M_PI/2.0)) //catch 179 to -179 angle switching
      {
        // the augmented angle will not switch in value and will be used to calculate the distance left to cover
        augmented_current_angle = previous_augmented_current_angle + (M_PI - previous_angle) + (current_angle + M_PI);
      }
      else
      {
        augmented_current_angle = previous_augmented_current_angle + fabs(current_angle - previous_angle);
      }
      dist_pos_left = rotation_angle_*M_PI/180.0 - fabs(augmented_current_angle - start_angle_pos)  ;
      //printf("Current angle is %f \n", current_angle/M_PI*180.0);
      if (dist_pos_left < tolerance_)
      {
        got_pos_angle = true;
        start_angle_neg = augmented_current_angle; 
        cmd_vel.angular.z = 0.0;
        vel_pub.publish(cmd_vel);
      }
      else
      {
        vel = sqrt(2 * acc_lim_th_ * dist_pos_left);
        vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);
        cmd_vel.angular.z = vel;
        vel_pub.publish(cmd_vel);
      }
      //printf("Current positive value to cover is %f \n", dist_pos_left/M_PI*180.0);
    }
    else if (!got_neg_angle)
    {
      if ((previous_angle < -M_PI/2.0) && (current_angle > M_PI/2.0)) //catch -179 to 179 angle switching
      {
        // the augmented angle will not switch in value and will be used to calculate the distance left to cover
        augmented_current_angle = previous_augmented_current_angle - (M_PI + previous_angle) - (- current_angle + M_PI);
      }
      else
      {
        augmented_current_angle = previous_augmented_current_angle - fabs(current_angle - previous_angle);
      }
      dist_neg_left = 2*rotation_angle_*M_PI/180.0 - fabs(start_angle_neg - augmented_current_angle)  ;

      if (dist_neg_left < tolerance_)
      {
        got_neg_angle = true;
        start_angle_pos = augmented_current_angle; 
        cmd_vel.angular.z = 0.0;
        vel_pub.publish(cmd_vel);
      }
      else
      {
        vel = sqrt(2 * acc_lim_th_ * dist_neg_left);
        vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);
        cmd_vel.angular.z = -vel;
        vel_pub.publish(cmd_vel);
      }
    }
    else
    {
      //printf("----------------------------------------------------------------Return to initial position \n");
      if ((previous_angle > M_PI/2.0) && (current_angle < -M_PI/2.0)) //catch 179 to -179 angle switching
      {
        // the augmented angle will not switch in value and will be used to calculate the distance left to cover
        augmented_current_angle = previous_augmented_current_angle + (M_PI - previous_angle) + (current_angle + M_PI);
      }
      else
      {
        augmented_current_angle = previous_augmented_current_angle + fabs(current_angle - previous_angle);
      }
      dist_pos_left = rotation_angle_*M_PI/180.0 - fabs(augmented_current_angle - start_angle_pos)  ;
      if (dist_pos_left < tolerance_)
      {
        returned = true;
        cmd_vel.angular.z = 0.0;
        vel_pub.publish(cmd_vel);
      }
      else
      {
        vel = sqrt(2 * acc_lim_th_ * dist_pos_left);
        vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);
        cmd_vel.angular.z = vel;
        vel_pub.publish(cmd_vel);
      }
    }

    double x = global_pose.pose.position.x, y = global_pose.pose.position.y;

    // check if that velocity is legal by forward simulating
    double sim_angle = 0.0;
    double angle_lim = 0.0;
    if (!got_pos_angle)
    {
      angle_lim = dist_pos_left;
    }
    else if (!got_neg_angle)
    {
      angle_lim = dist_neg_left;
    } 
    else
    {
      angle_lim = dist_pos_left;
    }

    while (sim_angle < angle_lim)
    {
      double theta = current_angle + sim_angle;

      // make sure that the point is legal, if it isn't... we'll abort
      double footprint_cost = world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
      if (footprint_cost < 0.0)
      {
        ROS_ERROR("Swing recovery can't rotate in place because there is a potential collision. Cost: %.2f", footprint_cost);
        return mbf_msgs::ExePathResult::CANCELED;
      }

      sim_angle += sim_granularity_;
    }

    r.sleep();
  }
  ROS_WARN("Swing recovery behavior finished.");
  return mbf_msgs::ExePathResult::SUCCESS;
}
};  // namespace swing_recovery
