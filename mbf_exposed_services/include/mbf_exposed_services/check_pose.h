#pragma once

#include <tf/tf.h>
#include "mbf_costmap_nav/footprint_helper.h"
#include "mbf_costmap_nav/costmap_navigation_server.h"

namespace mbf_exposed_services
{

class CheckPose
{
public:
  CheckPose();
  ~CheckPose();

  bool check_pose_cost_from_costmap_and_footprint(costmap_2d::Costmap2D*, std::vector<geometry_msgs::Point>,
                                                  mbf_msgs::CheckPose::Request&, mbf_msgs::CheckPose::Response&);

  bool check_pose_cost(costmap_2d::Costmap2DROS* costmap, mbf_msgs::CheckPose::Request& request,
                       mbf_msgs::CheckPose::Response& response);
};
};  // namespace mbf_exposed_services
