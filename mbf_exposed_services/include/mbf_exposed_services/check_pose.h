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

    bool check_pose_cost(costmap_2d::Costmap2DROS* costmap, mbf_msgs::CheckPose::Request &request, mbf_msgs::CheckPose::Response &response);

  };
};
