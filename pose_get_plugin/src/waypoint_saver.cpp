#include <fstream>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

namespace waypoint_saver
{
class WaypointSaver
{
public:
  WaypointSaver(const std::string& filename) : filename_(filename) {}

  void addWaypoint(const geometry_msgs::Pose& pose)
  {
    waypoints_.push_back(pose);
  }

  void saveWaypoints()
  {
    std::ofstream file(filename_);
    if (!file.is_open())
    {
      ROS_ERROR_STREAM("Failed to open file: " << filename_);
      return;
    }

    for (const auto& waypoint : waypoints_)
    {
      file << waypoint.position.x << " " << waypoint.position.y << " " << waypoint.position.z << " "
           << waypoint.orientation.x << " " << waypoint.orientation.y << " " << waypoint.orientation.z << " " << waypoint.orientation.w << "\n";
    }

    file.close();
    ROS_INFO_STREAM("Waypoints saved to file: " << filename_);
  }

  void setFilename(const std::string& filename)
  {
    filename_ = filename;
  }

private:
  std::string filename_;
  std::vector<geometry_msgs::Pose> waypoints_;
};
} // namespace waypoint_saver