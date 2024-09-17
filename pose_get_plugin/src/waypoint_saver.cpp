#include "pose_get_plugin/waypoint_saver.h"
#include <fstream>
#include <ros/ros.h>

namespace waypoint_saver
{
WaypointSaver::WaypointSaver(const std::string& filename) : filename_(filename) {}

void WaypointSaver::addWaypoint(const geometry_msgs::Pose& pose, int id)
{
  waypoints_.emplace_back(id, pose);
}

void WaypointSaver::removeLastWaypoint()
{
  if (!waypoints_.empty())
  {
    waypoints_.pop_back();
  }
}

void WaypointSaver::saveWaypoints()
{
  std::ofstream file(filename_);
  if (!file.is_open())
  {
    ROS_ERROR_STREAM("Failed to open file: " << filename_);
    return;
  }

  for (const auto& waypoint : waypoints_)
  {
    file << waypoint.first << " "
         << waypoint.second.position.x << " "
         << waypoint.second.position.y << " "
         << waypoint.second.position.z << " "
         << waypoint.second.orientation.x << " "
         << waypoint.second.orientation.y << " "
         << waypoint.second.orientation.z << " "
         << waypoint.second.orientation.w << "\n";
  }

  file.close();
  ROS_INFO_STREAM("Waypoints saved to file: " << filename_);
}

} // namespace waypoint_saver