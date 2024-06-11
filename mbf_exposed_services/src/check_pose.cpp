
#include <mbf_exposed_services/check_pose.h>

namespace mbf_exposed_services
{

  CheckPose::CheckPose() = default;

  CheckPose::~CheckPose() = default;

  bool CheckPose::check_pose_cost(costmap_2d::Costmap2DROS* costmap, mbf_msgs::CheckPose::Request &request, mbf_msgs::CheckPose::Response &response)
  {
    double x = request.pose.pose.position.x;
    double y = request.pose.pose.position.y;
    double yaw = tf::getYaw(request.pose.pose.orientation);

    // ensure costmap is active so cost reflects latest sensor readings
    // costmap->checkActivate();

    // pad raw footprint to the requested safety distance; note that we discard footprint_padding parameter effect
    std::vector<geometry_msgs::Point> footprint = costmap->getUnpaddedRobotFootprint();
    costmap_2d::padFootprint(footprint, request.safety_dist);

    // use footprint helper to get all the cells totally or partially within footprint polygon
    std::vector<mbf_costmap_nav::Cell> footprint_cells =
      mbf_costmap_nav::FootprintHelper::getFootprintCells(x, y, yaw, footprint, *costmap->getCostmap(), true);
    response.state = mbf_msgs::CheckPose::Response::FREE;
    if (footprint_cells.empty())
    {
      // no cells within footprint polygon must mean that robot is at least partly outside of the map
      response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPose::Response::OUTSIDE));
    }
    else
    {
      // lock costmap so content doesn't change while adding cell costs
      boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getCostmap()->getMutex()));

      // integrate the cost of all cells; state value precedence is UNKNOWN > LETHAL > INSCRIBED > FREE
      for (int i = 0; i < footprint_cells.size(); ++i)
      {
        unsigned char cost = costmap->getCostmap()->getCost(footprint_cells[i].x, footprint_cells[i].y);
        switch (cost)
        {
          case costmap_2d::NO_INFORMATION:
            response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPose::Response::UNKNOWN));
            response.cost += cost * (request.unknown_cost_mult ? request.unknown_cost_mult : 1.0);
            break;
          case costmap_2d::LETHAL_OBSTACLE:
            response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPose::Response::LETHAL));
            response.cost += cost * (request.lethal_cost_mult ? request.lethal_cost_mult : 1.0);
            break;
          case costmap_2d::INSCRIBED_INFLATED_OBSTACLE:
            response.state = std::max(response.state, static_cast<uint8_t>(mbf_msgs::CheckPose::Response::INSCRIBED));
            response.cost += cost * (request.inscrib_cost_mult ? request.inscrib_cost_mult : 1.0);
            break;
          default:
            response.cost += cost;
            break;
        }
      }
    }

    // Provide some details of the outcome
    switch (response.state)
    {
      case mbf_msgs::CheckPose::Response::OUTSIDE:
        ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is outside the map (cost = " << response.cost
                                  << "; safety distance = " << request.safety_dist << ")");
        break;
      case mbf_msgs::CheckPose::Response::UNKNOWN:
        ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is in unknown space! (cost = " << response.cost
                                  << "; safety distance = " << request.safety_dist << ")");
        break;
      case mbf_msgs::CheckPose::Response::LETHAL:
        ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is in collision! (cost = " << response.cost
                                  << "; safety distance = " << request.safety_dist << ")");
        break;
      case mbf_msgs::CheckPose::Response::INSCRIBED:
        ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is near an obstacle (cost = " << response.cost
                                  << "; safety distance = " << request.safety_dist << ")");
        break;
      case mbf_msgs::CheckPose::Response::FREE:
        ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is free (cost = " << response.cost
                                  << "; safety distance = " << request.safety_dist << ")");
        break;
    }

    // costmap->checkDeactivate();
    return true;
  }

}